package public_api_tests

import "core:testing"
import "base:runtime"
import util "entasis:entasis_utilities"
import e "entasis:entasis"
import p "entasis:entasis_physics"

Trigger_Test_Sleep_State :: enum u8
{
	Awake,
	Asleep,
}

Trigger_Test_Shape_Kind :: enum u8
{
	Convex,
	Compound,
}

Trigger_Test_Capacity_Limit :: enum u8
{
	Pairs,
	Candidates,
}

Trigger_Test_Scene :: struct
{
	world:e.World, shape:e.Shape_Handle, sensor:e.Static_Handle, body:e.Body_Handle, sensor_ref:e.Collidable_Reference, body_ref:e.Collidable_Reference
}

trigger_test_scene :: proc(t:^testing.T, s:^Trigger_Test_Scene, workers:i32=1, scope:e.Allocation_Scope=.Legacy, mobility:e.Body_Mobility=.Dynamic)->e.Status
{
	d: e.World_Description = small_world_description();
	d.gravity={};
	d.damping={};
	d.threading.worker_count=workers;
	d.capacity.pairs=64;
	d.capacity.pending_pairs_per_worker=32;
	st: e.Status = e.world_init_with_allocation_scope(&s.world, d, scope);
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return st;
	}
	shape: e.Shape_Handle;
	shape, st=e.shape_add(&s.world, e.sphere(1));
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return st;
	};
	s.shape=shape;
	s.sensor, st=e.static_add(&s.world, e.static_body(shape, e.pose()), .None);
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return st;
	}
	s.sensor_ref, _=p.collidable_reference_static(s.sensor);
	inertia: e.Body_Inertia;
	inertia, _ = e.shape_inertia(e.sphere(1), 1);
	description: e.Body_Description = e.body_dynamic(shape, inertia, e.pose({1, 0, 0}), {}, e.body_activity(-1, 255));
	if mobility == .Kinematic
	{
		description=e.body_kinematic(shape, e.pose({1, 0, 0}), {}, e.body_activity(-1, 255));
	}
	s.body, st=e.body_add(&s.world, description);
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return st;
	}
	s.body_ref, _=p.collidable_reference_body(mobility, s.body);
	st=e.world_enable_triggers(&s.world, {pair_capacity=32, candidates_per_worker=64, child_capacity=128});
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return st;
	}
	st=e.trigger_set(&s.world, s.sensor_ref, {user_id=42});
	testing.expect_value(t, st, e.Status.Ok);
	return st;
}

@(test)
triggers_enter_exit_stay_retry_without_solver_response :: proc(t:^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s:Trigger_Test_Scene;
		if trigger_test_scene(t, &s, workers) != .Ok
		{
			return;
		};
		defer e.world_destroy(&s.world);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		testing.expect_value(t, e.trigger_set_user_id(&s.world, s.body_ref, 12345), e.Status.Ok);
		before: e.Body_State;
		before, _ = e.body_get(&s.world, s.body);
		if !testing.expect_value(t, e.world_step(&s.world, 1.0/60), e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		after: e.Body_State;
		after, _ = e.body_get(&s.world, s.body);
		testing.expect_value(t, after.pose, before.pose);
		testing.expect_value(t, after.velocity, before.velocity);
		n: int;
		required: int;
		status: e.Status;
		n, required, status = e.trigger_events_drain(&s.world, nil);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, n, 0);
		testing.expect_value(t, required, 1);
		index: u64 = sim.step_index;
		testing.expect_value(t, e.world_step(&s.world, 1.0/60), e.Status.Invalid_Argument);
		testing.expect_value(t, sim.step_index, index);
		events:[8]e.Trigger_Event;
		n, required, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
		testing.expect_value(t, events[0].pair.user_a, u64(42));
		testing.expect_value(t, events[0].pair.user_b, u64(12345));
		token: u64 = events[0].pair.token_a;
		testing.expect(t, token!=0&&events[0].pair.token_b!=0);
		testing.expect_value(t, e.world_step(&s.world, 1.0/60), e.Status.Ok);
		n, _, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, n, 0);
		testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref, {user_id=42, stay=.Enabled}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&s.world, 1.0/60), e.Status.Ok);
		n, _, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Stay);
		testing.expect_value(t, events[0].pair.token_a, token);
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({8, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&s.world, 1.0/60), e.Status.Ok);
		n, _, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	}
}

@(test)
triggers_kinematic_static_and_static_static_optin :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s, 2, .All_Owned, .Kinematic) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	events:[8]e.Trigger_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	st: e.Status;
	n, _, st = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, e.body_remove(&s.world, s.body), e.Status.Ok);
	other: e.Static_Handle;
	add: e.Status;
	other, add = e.static_add(&s.world, e.static_body(s.shape, e.pose({1, 0, 0})), .None);
	testing.expect_value(t, add, e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref, {static_static=.Enabled}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
	testing.expect_value(t, e.static_set_pose(&s.world, other, e.pose({10, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
}

@(test)
triggers_handle_reuse_does_not_reuse_overlap_lifetime :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	events:[8]e.Trigger_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, events[:]);
	old: e.Trigger_Pair = events[0].pair;
	description: e.Body_State;
	description, _ = e.body_get(&s.world, s.body);
	testing.expect_value(t, e.body_remove(&s.world, s.body), e.Status.Ok);
	next: e.Body_Handle;
	st: e.Status;
	next, st = e.body_add(&s.world, description);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, next, s.body);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	n, _, _ = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 2);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	testing.expect_value(t, events[0].pair, old);
	testing.expect_value(t, events[1].kind, e.Trigger_Event_Kind.Enter);
	testing.expect(t, events[1].pair.token_b!=old.token_b);
}

@(test)
triggers_unchanged_sleepers_retained_moving_static_does_not_wake :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	events:[8]e.Trigger_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, e.bodies_sleep_group(&s.world, []e.Body_Handle{s.body}), e.Status.Ok);
	loc: p.Body_Memory_Location;
	loc, _ = p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, loc.set_index>0);
	for _ in 0..<3
	{
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		n: int;
		n, _, _ = e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, n, 0);
	}
	testing.expect_value(t, e.static_set_pose(&s.world, s.sensor, e.pose({10, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	n, _, _ = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	loc, _=p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, loc.set_index>0);
	testing.expect_value(t, e.static_set_pose(&s.world, s.sensor, e.pose()), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
	loc, _=p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, loc.set_index>0);
}

@(test)
triggers_stepper_consumes_physics_once_on_backpressure :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	stepper: e.Fixed_Stepper = e.fixed_stepper(0.01, 8);
	events:[8]e.Trigger_Event;
	result: e.Trigger_Update_Result;
	status: e.Status;
	result, status = e.trigger_stepper_update(&stepper, &s.world, 0.025, nil);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect(t, result.notification_pending == .Pending);
	result, status=e.trigger_stepper_update(&stepper, &s.world, 0, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.events_written, i32(1));
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect_value(t, sim.step_index, u64(2));
}

Trigger_Filter_Test :: struct
{
	allow, child: e.Collision_Testing_State, configured:int
}

trigger_test_allow :: proc "contextless" (user:rawptr, worker:int, a, b:e.Collidable_Reference, margin:^f32) -> e.Collision_Testing_State
{
	_=worker;
	_=a;
	_=b;
	margin^=100;
	return (^Trigger_Filter_Test)(user).allow;
}

trigger_test_allow_child :: proc "contextless" (user:rawptr, worker:int, a, b:e.Collidable_Reference, ca, cb:int) -> e.Collision_Testing_State
{
	_=worker;
	_=a;
	_=b;
	_=ca;
	_=cb;
	return (^Trigger_Filter_Test)(user).child;
}

trigger_test_configure :: proc "contextless" (user:rawptr, worker:int, a, b:e.Collidable_Reference, manifold:^e.Manifold_Result, material:^e.Contact_Material) -> e.Collision_Testing_State
{
	_=worker;
	_=a;
	_=b;
	_=manifold;
	_=material;
	(^Trigger_Filter_Test)(user).configured+=1;
	return .Allow;
}

@(test)
triggers_geometry_filters_and_compound_parent_reduction :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	policy: Trigger_Filter_Test = Trigger_Filter_Test{allow=.Allow, child=.Allow};
	saved: p.Narrow_Phase_Callbacks = sim.narrow_phase.callbacks;
	defer sim.narrow_phase.callbacks=saved;
	sim.narrow_phase.callbacks={user_context=&policy, allow=trigger_test_allow, allow_child=trigger_test_allow_child, configure=trigger_test_configure};
	events:[8]e.Trigger_Event;
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1.5, 1.5, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	n, _, _ = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 0);
	// speculative margin from an application callback must not turn separation into Enter
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({2.01, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 0);
	children: [2]e.Compound_Child = [2]e.Compound_Child{e.compound_child(s.shape, e.pose({-0.3, 0, 0})), e.compound_child(s.shape, e.pose({0.3, 0, 0}))};
	compound: e.Shape_Handle;
	st: e.Status;
	compound, st = e.shape_import_compound(&s.world, children[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(compound, e.pose()), .None), e.Status.Ok);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({0, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
	testing.expect_value(t, policy.configured, 0);
	testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
	policy.child=.Reject;
	testing.expect_value(t, e.trigger_mark_filters_dirty(&s.world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	testing.expect_value(t, events[0].reason, e.Trigger_Exit_Reason.Filter_Changed);
	policy.child=.Allow;
	policy.allow=.Reject;
	testing.expect_value(t, e.trigger_mark_filters_dirty(&s.world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 0);
}

@(test)
triggers_switch_existing_contact_to_sensor_including_sleeping_contact :: proc(t:^testing.T)
{
	for sleep_state in ([2]Trigger_Test_Sleep_State{.Awake, .Asleep})
	{
		s:Trigger_Test_Scene;
		if trigger_test_scene(t, &s) != .Ok
		{
			return;
		};
		defer e.world_destroy(&s.world);
		testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		testing.expect(t, sim.narrow_phase.pair_cache.mapping.count>0);
		if sleep_state == .Asleep
		{
			scaffold: ^p.Island_Scaffold = &sim.sleeper.scaffold;
			scaffold.island_bodies.memory[0]=s.body;
			scaffold.island_constraints.memory[0]=sim.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
			testing.expect_value(t, p.island_sleeper_sleep_island(&sim.sleeper, scaffold, 1, 1, nil), p.Physics_Status.Ok);
			testing.expect(t, sim.narrow_phase.pair_cache.inactive_count>0);
		}
		testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref), e.Status.Ok);
		testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		testing.expect_value(t, sim.narrow_phase.pair_cache.inactive_count, 0);
		if sleep_state == .Asleep
		{
			loc: p.Body_Memory_Location;
			loc, _ = p.bodies_resolve(&sim.bodies, s.body);
			testing.expect(t, loc.set_index>0);
		}
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		events:[8]e.Trigger_Event;
		n: int;
		status: e.Status;
		n, _, status = e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 1);
		// waking after inactive-contact compaction must not resolve a freed contact handle
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		_, _, _=e.trigger_events_drain(&s.world, events[:]);
	}
}

@(test)
triggers_published_batch_survives_removal_and_clear_resets_epoch :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	testing.expect_value(t, e.body_remove(&s.world, s.body), e.Status.Ok);
	events:[8]e.Trigger_Event;
	n: int;
	st: e.Status;
	n, _, st = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
	old: e.Trigger_Pair = events[0].pair;
	epoch: u64 = events[0].epoch;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, _=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].pair, old);
	testing.expect_value(t, events[0].reason, e.Trigger_Exit_Reason.Removed);
	testing.expect_value(t, e.world_clear(&s.world), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect(t, sim.triggers.epoch>epoch);
	testing.expect_value(t, sim.triggers.event_count, 0);
}

@(test)
triggers_all_owned_failure_cleanup_and_reuse :: proc(t:^testing.T)
{
	tracker:allocation_test_tracker;
	world:e.World;
	d: e.World_Description = small_world_description();
	d.allocator=allocation_test_allocator(&tracker);
	d.threading.worker_count=2;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, d, .All_Owned), e.Status.Ok)
	{
		return;
	}
	c: e.Trigger_Configuration = e.Trigger_Configuration{pair_capacity=8, candidates_per_worker=16, child_capacity=64};
	before: int = tracker.requests;
	if !testing.expect_value(t, e.world_enable_triggers(&world, c), e.Status.Ok)
	{
		_=e.world_destroy(&world);
		return;
	}
	allocations: int = tracker.requests-before;
	for _ in 0..<3
	{
		before=tracker.requests;
		testing.expect_value(t, e.trigger_reserve(&world, c), e.Status.Ok);
		testing.expect_value(t, tracker.requests, before);
	}
	tracker.fail_from=tracker.requests+1;
	testing.expect_value(t, e.world_disable_triggers(&world), e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
	for fail in 1..=allocations
	{
		tracker={};
		world=nil;
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, d, .All_Owned), e.Status.Ok)
		{
			return;
		}
		tracker.fail_from=tracker.requests+fail;
		status: e.Status = e.world_enable_triggers(&world, c);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&world);
		testing.expect(t, sim.triggers==nil);
		before=tracker.requests;
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		testing.expect_value(t, tracker.requests, before);
		allocation_test_empty(t, &tracker);
	}
}

@(test)
triggers_native_custom_shape_uses_registered_collision_route :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s, 2) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	type_id: e.Shape_Type_ID;
	ok: bool;
	type_id, ok = register_custom_sphere(t, &s.world);
	if !ok
	{
		return;
	}
	custom: e.Shape_Handle;
	st: e.Status;
	custom, st = e.custom_shape_add(&s.world, type_id, &Custom_Sphere{radius=1});
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.body_set_shape(&s.world, s.body, custom), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	events:[4]e.Trigger_Event;
	n: int;
	status: e.Status;
	n, _, status = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
}

Trigger_Custom_Step :: struct
{
	mode:int, sleep_handle:e.Body_Handle
}

trigger_test_step :: proc "contextless" (user:rawptr, sim:^p.Simulation, dt:f32, dispatcher:^util.Thread_Dispatcher_Boundary) -> p.Physics_Status
{
	context=runtime.default_context();
	state: ^Trigger_Custom_Step = (^Trigger_Custom_Step)(user);
	if state.mode==1
	{
		sim.step_index+=1;
		return .Ok;
	}
	status: p.Physics_Status = p.simulation_predict_bounding_boxes(sim, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_collision_detection(sim, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	if state.mode==2
	{
		return .Invalid_Description;
	}
	if state.mode==3
	{
		status=p.simulation_collision_detection(sim, dt, dispatcher);
		if status!=.Ok
		{
			return status;
		}
	}
	if state.mode==4
	{
		status=p.simulation_solve(sim, dt, dispatcher);
		if status!=.Ok
		{
			return status;
		}
		scaffold: ^p.Island_Scaffold = &sim.sleeper.scaffold;
		status=p.island_scaffold_ensure_capacity(scaffold, 1, max(1, int(scaffold.island_constraints.length)));
		if status!=.Ok
		{
			return status;
		}
		scaffold.island_bodies.memory[0]=state.sleep_handle;
		status=p.island_sleeper_sleep_island(&sim.sleeper, scaffold, 1, 0, dispatcher);
		if status!=.Ok
		{
			return status;
		}
	}
	sim.step_index+=1;
	return .Ok;
}

@(test)
triggers_custom_step_sampling_errors_and_recovery :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	state:Trigger_Custom_Step;
	sim.timestepper={step=trigger_test_step, user_context=&state};
	events:[8]e.Trigger_Event;
	state.mode=2;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Invalid_Description);
	testing.expect_value(t, sim.step_index, u64(0));
	n: int;
	st: e.Status;
	n, _, st = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 0);
	state.mode=3;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, st=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 1);
	state.mode=1;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	testing.expect_value(t, sim.step_index, u64(2));
	_, _, st=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Invalid_Argument);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Invalid_Argument);
	testing.expect_value(t, sim.step_index, u64(2));
	testing.expect_value(t, e.trigger_reset_history(&s.world), e.Status.Ok);
	state.mode=0;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, st=e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 1);
}

@(test)
triggers_trigger_pair_and_explicit_joint_remain_distinct :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if trigger_test_scene(t, &s, 2) != .Ok
	{
		return;
	};
	defer e.world_destroy(&s.world);
	type_id: e.Constraint_Type_ID;
	ok: bool;
	type_id, ok = register_velocity_target_constraint(t, &s.world);
	if !ok
	{
		return;
	}
	handles: [4]e.Body_Handle = [4]e.Body_Handle{s.body, e.body_handle_invalid(), e.body_handle_invalid(), e.body_handle_invalid()};
	target: Velocity_Target_Constraint = Velocity_Target_Constraint{target_speed=0.2};
	joint: e.Constraint_Handle;
	st: e.Status;
	joint, st = e.custom_constraint_add_typed(&s.world, handles[:1], type_id, &target);
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.trigger_set(&s.world, s.body_ref, {user_id=99}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	events:[8]e.Trigger_Event;
	n: int;
	status: e.Status;
	n, _, status = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].pair.flags&3, u32(3));
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	_, status=p.solver_resolve(&sim.solver, joint);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
	body: e.Body_State;
	body, _ = e.body_get(&s.world, s.body);
	testing.expect(t, body.velocity.linear.x>0);
}

@(test)
triggers_reserved_warm_steps_and_failed_growth_preserve_history :: proc(t:^testing.T)
{
	for fail in 1..=16
	{
		tracker:allocation_test_tracker;
		world:e.World;
		d: e.World_Description = small_world_description();
		d.allocator=allocation_test_allocator(&tracker);
		d.gravity={};
		d.threading.worker_count=2;
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, d, .All_Owned), e.Status.Ok)
		{
			return;
		}
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.sphere(1));
		sensor: e.Static_Handle;
		sensor, _ = e.static_add(&world, e.static_body(shape, e.pose()), .None);
		ref: e.Collidable_Reference;
		ref, _ = e.static_collidable_reference(&world, sensor);
		add: e.Status;
		_, add = e.body_add(&world, e.body_kinematic(shape, e.pose({1, 0, 0}), {}, e.body_activity(-1, 255)));
		testing.expect_value(t, add, e.Status.Ok);
		c: e.Trigger_Configuration = e.Trigger_Configuration{8, 32, 64};
		testing.expect_value(t, e.world_enable_triggers(&world, c), e.Status.Ok);
		testing.expect_value(t, e.trigger_set(&world, ref), e.Status.Ok);
		events:[4]e.Trigger_Event;
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
		_, _, _=e.trigger_events_drain(&world, events[:]);
		first: e.Trigger_Pair = events[0].pair;
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
		requests: int = tracker.requests;
		tracker.fail_from=requests+1;
		for _ in 0..<3
		{
			testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
		}
		testing.expect_value(t, tracker.requests, requests);
		tracker.fail_from=tracker.requests+fail;
		growth: e.Status = e.trigger_reserve(&world, {64, 256, 512});
		testing.expect(t, growth==.Ok||growth==.Capacity_Missing);
		tracker.fail_from=0;
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
		pairs:[4]e.Trigger_Pair;
		n: int;
		st: e.Status;
		n, _, st = e.trigger_overlaps(&world, pairs[:]);
		testing.expect_value(t, st, e.Status.Ok);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, pairs[0], first);
		tracker.fail_from=tracker.requests+1;
		requests=tracker.requests;
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		testing.expect_value(t, tracker.requests, requests);
		allocation_test_empty(t, &tracker);
	}
}

@(test)
triggers_sleep_transition_revalidates_post_solve_pose :: proc(t: ^testing.T)
{
	for transition in ([2]e.Trigger_Event_Kind{.Exit, .Enter})
	{
		s: Trigger_Test_Scene;
		if trigger_test_scene(t, &s) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		start: f32 = 1;
		speed: f32 = 400;
		if transition == .Enter
		{
			start = 5;
			speed = -400;
		}
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({start, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&s.world, s.body, {linear={speed, 0, 0}}), e.Status.Ok);
		if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
		{
			return;
		}
		events: [4]e.Trigger_Event;
		first: int;
		status: e.Status;
		first, _, status = e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, first, 0 if transition == .Enter else 1);
		body: e.Body_State;
		body, _ = e.body_get(&s.world, s.body);
		testing.expect(t, body.pose.position.x < 2 if transition == .Enter else body.pose.position.x > 2);
		testing.expect_value(t, e.bodies_sleep_group(&s.world, []e.Body_Handle{s.body}), e.Status.Ok);
		if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
		{
			return;
		}
		count: int;
		result: e.Status;
		count, _, result = e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, result, e.Status.Ok);
		testing.expect_value(t, count, 1);
		if count == 1
		{
			testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter if transition == .Enter else e.Trigger_Event_Kind.Exit);
		}
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		loc: p.Body_Memory_Location;
		loc, _ = p.bodies_resolve(&sim.bodies, s.body);
		testing.expect(t, loc.set_index > 0);
	}
}

@(test)
triggers_single_unique_candidate_fits_exact_capacity :: proc(t: ^testing.T)
{
	world: e.World;
	d: e.World_Description = small_world_description();
	d.gravity = {};
	d.damping = {};
	if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, _ = e.shape_add(&world, e.sphere(1));
	sensor: e.Static_Handle;
	sensor, _ = e.static_add(&world, e.static_body(shape, e.pose()), .None);
	reference: e.Collidable_Reference;
	reference, _ = e.static_collidable_reference(&world, sensor);
	add: e.Status;
	_, add = e.body_add(&world, e.body_kinematic(shape, e.pose({1, 0, 0}), {}, e.body_activity(-1, 255)));
	testing.expect_value(t, add, e.Status.Ok);
	testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=1, candidates_per_worker=1, child_capacity=16}), e.Status.Ok);
	testing.expect_value(t, e.trigger_set(&world, reference), e.Status.Ok);
	if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
	{
		return;
	}
	events: [1]e.Trigger_Event;
	count: int;
	status: e.Status;
	count, _, status = e.trigger_events_drain(&world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
}

Trigger_Discovery_Count :: struct
{
	calls: int
}

trigger_discovery_allow :: proc "contextless" (user: rawptr, worker: int, a, b: e.Collidable_Reference, margin: ^f32) -> e.Collision_Testing_State
{
	_ = worker;
	_ = a;
	_ = b;
	_ = margin;
	(^Trigger_Discovery_Count)(user).calls += 1;
	return .Allow;
}

@(test)
triggers_dormant_dirty_pair_is_discovered_once :: proc(t: ^testing.T)
{
	world: e.World;
	d: e.World_Description = small_world_description();
	d.gravity = {};
	if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, _ = e.shape_add(&world, e.sphere(1));
	a: e.Static_Handle;
	a, _ = e.static_add(&world, e.static_body(shape, e.pose()), .None);
	b: e.Static_Handle;
	b, _ = e.static_add(&world, e.static_body(shape, e.pose({1, 0, 0})), .None);
	ra: e.Collidable_Reference;
	ra, _ = e.static_collidable_reference(&world, a);
	rb: e.Collidable_Reference;
	rb, _ = e.static_collidable_reference(&world, b);
	testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=1, candidates_per_worker=1, child_capacity=16}), e.Status.Ok);
	testing.expect_value(t, e.trigger_set(&world, ra, {static_static=.Enabled, stay=.Enabled}), e.Status.Ok);
	testing.expect_value(t, e.trigger_set(&world, rb, {static_static=.Enabled}), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&world);
	calls: Trigger_Discovery_Count;
	saved: p.Narrow_Phase_Callbacks = sim.narrow_phase.callbacks;
	defer sim.narrow_phase.callbacks = saved;
	sim.narrow_phase.callbacks = {user_context=&calls, allow=trigger_discovery_allow};
	events: [1]e.Trigger_Event;
	for iteration in 0..<4
	{
		calls.calls = 0;
		if iteration == 1
		{
			testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
		}
		if iteration == 2
		{
			testing.expect_value(t, e.trigger_mark_geometry_dirty(&world, rb), e.Status.Ok);
			testing.expect_value(t, e.trigger_mark_geometry_dirty(&world, ra), e.Status.Ok);
		}
		if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, calls.calls, 0 if iteration == 3 else 1);
		n: int;
		status: e.Status;
		n, _, status = e.trigger_events_drain(&world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter if iteration == 0 else e.Trigger_Event_Kind.Stay);
	}
}

@(test)
triggers_empty_route_preserves_mode_lifecycle :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	if trigger_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	events: [4]e.Trigger_Event;
	testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref), e.Status.Ok);
	testing.expect_value(t, sim.triggers.enabled_count, 1);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _ = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok);
	testing.expect_value(t, sim.triggers.enabled_count, 0);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	st: e.Status;
	n, _, st = e.trigger_events_drain(&s.world, events[:]);
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	testing.expect(t, sim.narrow_phase.pair_cache.mapping.count > 0);
	testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref), e.Status.Ok);
	testing.expect_value(t, e.trigger_reset_history(&s.world), e.Status.Ok);
	testing.expect_value(t, sim.triggers.enabled_count, 1);
	testing.expect_value(t, e.static_remove(&s.world, s.sensor), e.Status.Ok);
	testing.expect_value(t, sim.triggers.enabled_count, 0);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	testing.expect_value(t, e.world_clear(&s.world), e.Status.Ok);
	testing.expect_value(t, sim.triggers.enabled_count, 0);
}

@(test)
triggers_custom_step_late_sleep_preserves_invalidation :: proc(t: ^testing.T)
{
	for transition in ([2]e.Trigger_Event_Kind{.Exit, .Enter})
	{
		s: Trigger_Test_Scene;
		if trigger_test_scene(t, &s) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		start: f32 = 1;
		speed: f32 = 400;
		if transition == .Enter
		{
			start=5;
			speed=-400;
		}
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({start, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&s.world, s.body, {linear={speed, 0, 0}}), e.Status.Ok);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		state: Trigger_Custom_Step = Trigger_Custom_Step{mode=4, sleep_handle=s.body};
		sim.timestepper={step=trigger_test_step, user_context=&state};
		if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
		{
			return;
		}
		events:[4]e.Trigger_Event;
		n: int;
		st: e.Status;
		n, _, st = e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, st, e.Status.Ok);
		testing.expect_value(t, n, 0 if transition == .Enter else 1);
		loc: p.Body_Memory_Location;
		loc, _ = p.bodies_resolve(&sim.bodies, s.body);
		testing.expect(t, loc.set_index>0);
		state.mode=0;
		if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
		{
			return;
		}
		n, _, st=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, st, e.Status.Ok);
		testing.expect_value(t, n, 1);
		if n==1
		{
			testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter if transition == .Enter else e.Trigger_Event_Kind.Exit);
		}
	}
}

// more parent pairs than a worker's bounded batch scratch. candidate IDs must
// remain global across flushes, including compound child reduction and retries
@(test)
triggers_dense_candidates_stream_through_bounded_scratch :: proc(t: ^testing.T)
{
	pair_count :: 1025;
	for workers in ([3]i32{1, 2, 4})
	{
		for shape_kind in ([2]Trigger_Test_Shape_Kind{.Convex, .Compound})
		{
			world: e.World;
			tracker: allocation_test_tracker;
			d: e.World_Description = small_world_description();
			d.gravity = {};
			d.threading.worker_count = workers;
			d.capacity.statics = 2*pair_count;
			d.allocator = allocation_test_allocator(&tracker);
			if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, d, .All_Owned), e.Status.Ok)
			{
				return;
			}
			defer
			{
				requests: int = tracker.requests;
				testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
				testing.expect_value(t, tracker.requests, requests);
				allocation_test_empty(t, &tracker);
			}
			shape: e.Shape_Handle;
			st: e.Status;
			shape, st = e.shape_add(&world, e.sphere(1));
			if !testing.expect_value(t, st, e.Status.Ok)
			{
				return;
			}
			sensor_shape: e.Shape_Handle = shape;
			if shape_kind == .Compound
			{
				children: [2]e.Compound_Child = [2]e.Compound_Child{
					e.compound_child(shape, e.pose({-0.25, 0, 0})),
					e.compound_child(shape, e.pose({0.25, 0, 0})),
				};
				sensor_shape, st = e.shape_import_compound(&world, children[:]);
				if !testing.expect_value(t, st, e.Status.Ok)
				{
					return;
				}
			}
			c: e.Trigger_Configuration = e.Trigger_Configuration{pair_capacity=pair_count, candidates_per_worker=pair_count, child_capacity=4096};
			testing.expect_value(t, e.world_enable_triggers(&world, c), e.Status.Ok);
			for i in 0..<pair_count
			{
				sensor: e.Static_Handle;
				ss: e.Status;
				sensor, ss = e.static_add(&world, e.static_body(sensor_shape, e.pose({f32(6*i), 0, 0})), .None);
				if !testing.expect_value(t, ss, e.Status.Ok)
				{
					return;
				}
				vs: e.Status;
				_, vs = e.static_add(&world, e.static_body(shape, e.pose({f32(6*i)+1, 0, 0})), .None);
				if !testing.expect_value(t, vs, e.Status.Ok)
				{
					return;
				}
				ref: e.Collidable_Reference;
				ref, _ = e.static_collidable_reference(&world, sensor);
				testing.expect_value(t, e.trigger_set(&world, ref, {static_static=.Enabled, stay=.Enabled, user_id=u64(i+1)}), e.Status.Ok);
			}
			sim: ^e.Simulation;
			sim, _ = e.world_borrow_simulation(&world);
			events: []e.Trigger_Event = make([]e.Trigger_Event, pair_count);
			defer delete(events);
			for iteration in 0..<3
			{
				if iteration == 2
				{
					testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
				}
				step_status: e.Status = e.world_step(&world, 0.01);
				if !testing.expectf(t, step_status == .Ok, "workers=%d compound=%v iteration=%d status=%v failure_threshold=%d requests=%d narrow=%v", workers, shape_kind, iteration, step_status, tracker.fail_from, tracker.requests, sim.narrow_phase.last_stage)
				{
					return;
				}
				if iteration != 1
				{
					streamed: int = 0;
					for &w in sim.triggers.workers
					{
						if w.count > int(w.storage.length)
						{
							streamed += 1;
						}
					}
					testing.expect(t, streamed > 0);
				}
				n: int;
				required: int;
				status: e.Status;
				n, required, status = e.trigger_events_drain(&world, events[:pair_count-1]);
				testing.expect_value(t, status, e.Status.Capacity_Missing);
				testing.expect_value(t, n, 0);
				testing.expect_value(t, required, pair_count);
				n, required, status = e.trigger_events_drain(&world, events);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect_value(t, n, pair_count);
				seen: [pair_count]p.Reference_State;
				for &event in events[:n]
				{
					testing.expect_value(t, event.kind, e.Trigger_Event_Kind.Enter if iteration == 0 else e.Trigger_Event_Kind.Stay);
					id: u64 = event.pair.user_a if event.pair.flags&1 != 0 else event.pair.user_b;
					if !testing.expect(t, id >= 1 && id <= pair_count)
					{
						return;
					}
					testing.expect(t, seen[id-1] == .Missing);
					seen[id-1] = .Present;
				}
				testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
				for &w in sim.triggers.workers
				{
					testing.expect_value(t, w.batcher.pair_count, 0);
				}
			}
			// isolate trigger geometry from unrelated default tree-optimization warmup.
			// reusing the same immutable candidates must need no allocator request
			requests: int = tracker.requests;
			tracker.fail_from = requests+1;
			for &w in sim.triggers.workers
			{
				if w.count == 0
				{
					continue;
				}
				testing.expect_value(t, p.trigger_geometry(&w), e.Status.Ok);
				testing.expect_value(t, w.batcher.pair_count, 0);
				for hit in w.hits[:w.count]
				{
					testing.expect(t, hit == .Present);
				}
			}
			testing.expect_value(t, tracker.requests, requests);
		}
	}
}

@(test)
triggers_dormant_history_does_not_touch_reserved_hash_capacity :: proc(t: ^testing.T)
{
	world: e.World;
	d: e.World_Description = small_world_description();
	d.gravity = {};
	d.capacity.statics = 8;
	if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, _ = e.shape_add(&world, e.sphere(1));
	testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=65536, candidates_per_worker=8, child_capacity=16}), e.Status.Ok);
	refs: [3]e.Collidable_Reference;
	for i in 0..<3
	{
		sensor: e.Static_Handle;
		sensor, _ = e.static_add(&world, e.static_body(shape, e.pose({f32(i*6), 0, 0})), .None);
		_, _ = e.static_add(&world, e.static_body(shape, e.pose({f32(i*6)+1, 0, 0})), .None);
		refs[i], _ = e.static_collidable_reference(&world, sensor);
		testing.expect_value(t, e.trigger_set(&world, refs[i], {static_static=.Enabled, stay=.Enabled}), e.Status.Ok);
	}
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&world);
	events: [4]e.Trigger_Event;
	testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
	n: int;
	status: e.Status;
	n, _, status = e.trigger_events_drain(&world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 3);
	// this index is scratch, not retained identity. a candidate-free sample must
	// neither clear nor query it. the next actual candidate pass rebuilds it
	for &slot in sim.triggers.pair_slots
	{
		slot = -1;
	}
	for iteration in 0..<2
	{
		if iteration == 1
		{
			testing.expect_value(t, e.trigger_remove(&world, refs[1]), e.Status.Ok);
		}
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
		n, _, status = e.trigger_events_drain(&world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 3);
		for slot in sim.triggers.pair_slots
		{
			if !testing.expect_value(t, slot, i32(-1))
			{
				return;
			}
		}
		exits: int = 0;
		for event in events[:n]
		{
			if event.kind == .Exit
			{
				exits += 1;
				testing.expect_value(t, event.reason, e.Trigger_Exit_Reason.Disabled);
			}
		}
		testing.expect_value(t, exits, iteration);
	}
	testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
	n, _, status = e.trigger_events_drain(&world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 2);
	for slot in sim.triggers.pair_slots
	{
		if !testing.expect(t, slot >= 0)
		{
			return;
		}
	}
}

@(test)
triggers_stepper_pending_capacity_and_limit_validation :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	stepper: e.Fixed_Stepper = e.fixed_stepper(0.01, 8);
	events: [4]e.Trigger_Event;
	result: e.Trigger_Update_Result;
	status: e.Status;
	result, status = e.trigger_stepper_update(&stepper, &s.world, 0.01, nil);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.required, i32(1));
	testing.expect(t, result.notification_pending == .Pending);
	result, status = e.trigger_stepper_update(&stepper, &s.world, 0, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(0));
	testing.expect_value(t, result.events_written, i32(1));
	testing.expect_value(t, result.required, i32(0));
	testing.expect(t, result.notification_pending == .Ready);
	stepper.maximum_steps = 0;
	before: e.Fixed_Stepper = stepper;
	result, status = e.trigger_stepper_update(&stepper, &s.world, 0.01, events[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, stepper, before);
	testing.expect_value(t, result.completed_steps, i32(0));
}

@(test)
triggers_real_capacity_failure_preserves_committed_history :: proc(t: ^testing.T)
{
	initial_pairs :: 512;
	for capacity_limit in ([2]Trigger_Test_Capacity_Limit{.Pairs, .Candidates})
	{
		world: e.World;
		d: e.World_Description = small_world_description();
		d.gravity = {};
		d.capacity.statics = 2*(initial_pairs+1);
		if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.sphere(1));
		c: e.Trigger_Configuration = e.Trigger_Configuration{
			pair_capacity=initial_pairs+1 if capacity_limit == .Candidates else initial_pairs,
			candidates_per_worker=initial_pairs if capacity_limit == .Candidates else initial_pairs+1,
			child_capacity=64,
		};
		testing.expect_value(t, e.world_enable_triggers(&world, c), e.Status.Ok);
		events: []e.Trigger_Event = make([]e.Trigger_Event, initial_pairs+1);
		defer delete(events);
		pairs: []e.Trigger_Pair = make([]e.Trigger_Pair, initial_pairs+1);
		defer delete(pairs);
		for i in 0..=initial_pairs
		{
			if i == initial_pairs
			{
				if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
				{
					return;
				}
				n: int;
				st: e.Status;
				n, _, st = e.trigger_events_drain(&world, events);
				testing.expect_value(t, st, e.Status.Ok);
				testing.expect_value(t, n, initial_pairs);
			}
			sensor: e.Static_Handle;
			sensor, _ = e.static_add(&world, e.static_body(shape, e.pose({f32(6*i), 0, 0})), .None);
			_, _ = e.static_add(&world, e.static_body(shape, e.pose({f32(6*i)+1, 0, 0})), .None);
			reference: e.Collidable_Reference;
			reference, _ = e.static_collidable_reference(&world, sensor);
			testing.expect_value(t, e.trigger_set(&world, reference, {static_static=.Enabled}), e.Status.Ok);
		}
		testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&world);
		index: u64 = sim.step_index;
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Capacity_Missing);
		testing.expect_value(t, sim.step_index, index);
		count: int;
		required: int;
		status: e.Status;
		count, required, status = e.trigger_events_drain(&world, nil);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, required, 0);
		count, _, status = e.trigger_overlaps(&world, pairs);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, count, initial_pairs);
		for i in 0..<count
		{
			testing.expect_value(t, pairs[i], events[i].pair);
		}
		c.pair_capacity=initial_pairs+1;
		c.candidates_per_worker=initial_pairs+1;
		testing.expect_value(t, e.trigger_reserve(&world, c), e.Status.Ok);
		if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
		{
			return;
		}
		count, _, status = e.trigger_events_drain(&world, events);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, count, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
		count, _, status = e.trigger_overlaps(&world, pairs);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, count, initial_pairs+1);
	}
}

@(test)
triggers_mixed_ownership_identity_and_mode_conflicts :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	if trigger_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	sim: ^p.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect_value(t, p.mixed_colliders_initialize(sim.triggers, {instance_capacity=4, part_capacity=8}), e.Status.Ok);
	storage: ^p.Mixed_Collider_Storage = sim.triggers.mixed;
	info: p.Collider_Part_Info;
	status: e.Status;
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 1, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Trigger, user_id=72});
	testing.expect_value(t, status, e.Status.Ok);
	first: p.Collider_Part_Info = info;
	testing.expect_value(t, e.trigger_set(&s.world, s.sensor_ref), e.Status.Invalid_Argument);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Solid, user_id=77});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, info.identity, first.identity);
	testing.expect_value(t, e.static_set_pose(&s.world, s.sensor, e.pose({10, 0, 0})), e.Status.Ok);
	info, status = p.mixed_colliders_part_get(storage, s.sensor_ref, 0);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, info.identity, first.identity);
	testing.expect_value(t, p.mixed_colliders_part_remove(storage, s.sensor_ref, 0), e.Status.Ok);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, info.identity.serial > first.identity.serial);
	testing.expect_value(t, info.identity.incarnation, first.identity.incarnation);
	shape: e.Shape_Handle;
	shape, status = e.shape_add(&s.world, e.sphere(2));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(shape, e.pose()), .None), e.Status.Ok);
	info, status = p.mixed_colliders_part_get(storage, s.sensor_ref, 0);
	testing.expect_value(t, status, e.Status.Not_Found);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, info.identity.incarnation > first.identity.incarnation);
	testing.expect_value(t, info.identity.instance, first.identity.instance);
	testing.expect_value(t, e.static_remove(&s.world, s.sensor, .None), e.Status.Ok);
	info, status = p.mixed_colliders_part_get(storage, s.sensor_ref, 0);
	testing.expect_value(t, status, e.Status.Not_Found);
	next: e.Static_Handle;
	next, status = e.static_add(&s.world, e.static_body(s.shape, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, next, s.sensor);
	info, status = p.mixed_colliders_part_set(storage, s.sensor_ref, 0, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, info.identity.instance > first.identity.instance);
}

@(test)
triggers_mixed_solid_sensor_reduction :: proc(t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s: Trigger_Test_Scene;
		if trigger_test_scene(t, &s, workers) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok);
		children: [2]e.Compound_Child = {e.compound_child(s.shape, e.pose()), e.compound_child(s.shape, e.pose({3, 0, 0}))};
		shape: e.Shape_Handle;
		status: e.Status;
		shape, status = e.shape_import_compound(&s.world, children[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(shape, e.pose()), .None), e.Status.Ok);
		sim: ^p.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		testing.expect_value(t, p.mixed_colliders_initialize(sim.triggers, {instance_capacity=4, part_capacity=8}), e.Status.Ok);
		_, status = p.mixed_colliders_part_set(sim.triggers.mixed, s.sensor_ref, 1, {role=.Trigger});
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		after: e.Body_Description;
		after, _ = e.body_get(&s.world, s.body);
		testing.expect_value(t, after.velocity.linear, util.Vector3{});
		events: [8]e.Trigger_Event;
		written, required: int;
		written, required, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 1);
		testing.expect_value(t, required, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Enter);
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({0.5, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 1);
		after, _ = e.body_get(&s.world, s.body);
		testing.expect(t, after.velocity.linear.x > 0);
		written, required, status=e.trigger_events_drain(&s.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 1);
		testing.expect_value(t, events[0].kind, e.Trigger_Event_Kind.Exit);
	}
}

mixed_test_scene :: proc(t:^testing.T, s:^Trigger_Test_Scene, workers:i32=1, mobility:e.Body_Mobility=.Dynamic)->e.Status
{
	if trigger_test_scene(t, s, workers, .Legacy, mobility) != .Ok
	{
		return .Invalid_Argument;
	}
	if !testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok)
	{
		return .Invalid_Argument;
	}
	children:[2]e.Compound_Child={e.compound_child(s.shape, e.pose()), e.compound_child(s.shape, e.pose({3, 0, 0}))};
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_import_compound(&s.world, children[:]);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return .Invalid_Argument;
	}
	if !testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(shape, e.pose()), .None), e.Status.Ok)
	{
		return .Invalid_Argument;
	}
	if !testing.expect_value(t, e.collider_parts_reserve(&s.world, {instance_capacity=16, part_capacity=64,
				pair_capacity=32, observations_per_worker=128, event_subscription=.Enabled}), e.Status.Ok)
	{
		return .Invalid_Argument;
	}
	_, status=e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, user_id=91});
	return .Ok if testing.expect_value(t, status, e.Status.Ok) else .Invalid_Argument;
}

@(test)
triggers_mixed_part_stream_atomic_retry_and_both_responses :: proc(t:^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s:Trigger_Test_Scene;
		if mixed_test_scene(t, &s, workers) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1.5, 0, 0})), e.Status.Ok);
		if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
		{
			return;
		}
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 1);
		body: e.Body_State;
		body, _ = e.body_get(&s.world, s.body);
		testing.expect(t, body.velocity.linear.x>0);
		n: int;
		required: int;
		status: e.Status;
		n, required, status = e.trigger_part_events_drain(&s.world, nil);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, n, 0);
		testing.expect_value(t, required, 1);
		parent:[8]e.Trigger_Event;
		parts:[8]e.Collider_Part_Event;
		n, _, status=e.trigger_events_drain(&s.world, parent[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 1);
		step: u64 = sim.step_index;
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Invalid_Argument);
		testing.expect_value(t, sim.step_index, step);
		testing.expect_value(t, e.world_disable_triggers(&s.world), e.Status.Invalid_Argument);
		// a pending event is immutable even when its originating override is removed
		testing.expect_value(t, e.collider_part_remove(&s.world, s.sensor_ref, 1), e.Status.Ok);
		n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, parts[0].kind, e.Trigger_Event_Kind.Enter);
		sensor: e.Collider_Part_Endpoint = parts[0].pair.a;
		if sensor.reference.packed!=s.sensor_ref.packed
		{
			sensor=parts[0].pair.b;
		}
		testing.expect_value(t, sensor.child_index, i32(1));
		testing.expect_value(t, sensor.user_id, u64(91));
		testing.expect(t, sensor.serial!=0&&sensor.incarnation!=0&&sensor.instance!=0);
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		n, _, _=e.trigger_events_drain(&s.world, parent[:]);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, parent[0].kind, e.Trigger_Event_Kind.Exit);
		n, _, _=e.trigger_part_events_drain(&s.world, parts[:]);
		testing.expect_value(t, n, 1);
		testing.expect_value(t, parts[0].kind, e.Trigger_Event_Kind.Exit);
	}
}

@(test)
triggers_mixed_stepper_two_stream_backpressure :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2, .Kinematic) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
	stepper: e.Fixed_Stepper = e.fixed_stepper(0.01, 4);
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	result: e.Collider_Part_Update_Result;
	status: e.Status;
	result, status = e.collider_part_stepper_update(&stepper, &s.world, 0.02, parent[:], nil);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.events_written, i32(0));
	testing.expect_value(t, result.part_required, i32(1));
	testing.expect(t, .Part in result.pending);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect_value(t, sim.step_index, u64(1));
	result, status=e.collider_part_stepper_update(&stepper, &s.world, 0, parent[:], parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.events_written, i32(1));
	testing.expect_value(t, result.part_events_written, i32(1));
	testing.expect_value(t, result.required, i32(0));
	testing.expect_value(t, result.part_required, i32(0));
	testing.expect_value(t, sim.step_index, u64(2));
	testing.expect_value(t, stepper.accumulator, f32(0));
	testing.expect_value(t, parts[0].step, parent[0].step);
}

@(test)
triggers_mixed_part_lifetimes_and_dormant_history :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_remove(&s.world, s.body), e.Status.Ok);
	other: e.Static_Handle;
	status: e.Status;
	other, status = e.static_add(&s.world, e.static_body(s.shape, e.pose({3.5, 0, 0})), .None);
	testing.expect_value(t, status, e.Status.Ok);
	_, status=e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, user_id=13, options={.Static_Static}});
	testing.expect_value(t, status, e.Status.Ok);
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n:int;
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, status, e.Status.Ok);
	old: e.Collider_Part_Pair = parts[0].pair;
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	for index in 0..<int(sim.triggers.mixed.pair_slots.length)
	{
		sim.triggers.mixed.pair_slots.memory[index]=i32(0x55aa);
	}
	for _ in 0..<3
	{
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		_, _, _=e.trigger_events_drain(&s.world, parent[:]);
		n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
		testing.expect_value(t, n, 0);
		testing.expect_value(t, status, e.Status.Ok);
	}
	for index in 0..<int(sim.triggers.mixed.pair_slots.length)
	{
		testing.expect_value(t, sim.triggers.mixed.pair_slots.memory[index], i32(0x55aa));
	}
	testing.expect_value(t, e.static_remove(&s.world, other, .None), e.Status.Ok);
	replacement: e.Static_Handle;
	replacement, status=e.static_add(&s.world, e.static_body(s.shape, e.pose({3.5, 0, 0})), .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, replacement, other);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, n, 2);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, parts[0].kind, e.Trigger_Event_Kind.Exit);
	testing.expect_value(t, parts[0].pair, old);
	testing.expect_value(t, parts[0].reason, e.Trigger_Exit_Reason.Removed);
	testing.expect_value(t, parts[1].kind, e.Trigger_Event_Kind.Enter);
	testing.expect_value(t, e.trigger_reset_history(&s.world), e.Status.Ok);
	overlaps:[8]e.Collider_Part_Pair;
	n, _, status=e.trigger_part_overlaps(&s.world, overlaps[:]);
	testing.expect_value(t, n, 0);
	testing.expect_value(t, status, e.Status.Ok);
}

@(test)
triggers_mixed_static_mutation_wakes_only_solid_bounds :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({7, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.bodies_sleep_group(&s.world, []e.Body_Handle{s.body}), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	location: p.Body_Memory_Location;
	location, _ = p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, location.set_index>0);
	// at x=3 the solid child spans [2,4], sensor spans [5,7]. only the sensor reaches the sleeper
	testing.expect_value(t, e.static_set_pose(&s.world, s.sensor, e.pose({3, 0, 0})), e.Status.Ok);
	location, _=p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, location.set_index>0);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	n: int;
	status: e.Status;
	n, _, status = e.trigger_events_drain(&s.world, parent[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	location, _=p.bodies_resolve(&sim.bodies, s.body);
	testing.expect(t, location.set_index>0);
	// moving the solid child into the actor now must awaken its island
	testing.expect_value(t, e.static_set_pose(&s.world, s.sensor, e.pose({7, 0, 0})), e.Status.Ok);
	location, _=p.bodies_resolve(&sim.bodies, s.body);
	testing.expect_value(t, location.set_index, i32(0));

	// an ordinary solid static must also ignore the sleeping body's sensor side
	reversed: Trigger_Test_Scene;
	if trigger_test_scene(t, &reversed) != .Ok
	{
		return;
	}
	defer e.world_destroy(&reversed.world);
	testing.expect_value(t, e.trigger_remove(&reversed.world, reversed.sensor_ref), e.Status.Ok);
	testing.expect_value(t, e.collider_parts_reserve(&reversed.world,
		{instance_capacity=4, part_capacity=8, pair_capacity=4, observations_per_worker=8, event_subscription=.Enabled}), e.Status.Ok);
	children: [2]e.Compound_Child = {
		e.compound_child(reversed.shape, e.pose()),
		e.compound_child(reversed.shape, e.pose({3, 0, 0})),
	};
	compound: e.Shape_Handle;
	compound, status = e.shape_import_compound(&reversed.world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	inertia: e.Body_Inertia;
	inertia, status = e.shape_inertia(e.sphere(1), 1);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.body_apply(&reversed.world, reversed.body,
		e.body_dynamic(compound, inertia, e.pose(), {}, e.body_activity(-1, 255))), e.Status.Ok);
	_, status = e.collider_part_set(&reversed.world, reversed.body_ref, 1, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.static_set_pose(&reversed.world, reversed.sensor, e.pose({3, 0, 0}), .None), e.Status.Ok);
	testing.expect_value(t, e.bodies_sleep_group(&reversed.world, []e.Body_Handle{reversed.body}), e.Status.Ok);
	reversed_sim: ^e.Simulation;
	reversed_sim, _ = e.world_borrow_simulation(&reversed.world);
	testing.expect_value(t, e.static_set_pose(&reversed.world, reversed.sensor, e.pose({4, 0, 0})), e.Status.Ok);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect(t, location.set_index > 0);
	testing.expect_value(t, e.static_remove(&reversed.world, reversed.sensor), e.Status.Ok);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect(t, location.set_index > 0);
	reversed.sensor, status = e.static_add(&reversed.world, e.static_body(reversed.shape, e.pose({4, 0, 0})), .None);
	testing.expect_value(t, status, e.Status.Ok);

	// explicit no-awakening and a rejecting filter remain authoritative
	testing.expect_value(t, e.static_set_pose(&reversed.world, reversed.sensor, e.pose(), .None), e.Status.Ok);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect(t, location.set_index > 0);
	filter_calls: int;
	testing.expect_value(t, e.static_apply_filtered(&reversed.world, reversed.sensor,
		e.static_body(reversed.shape, e.pose()),
		proc "contextless" (user: rawptr, body: e.Body_Handle) -> bool
		{
			(^int)(user)^ += 1;
			return false;
		}, &filter_calls), e.Status.Ok);
	testing.expect_value(t, filter_calls, 1);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect(t, location.set_index > 0);
	testing.expect_value(t, e.static_set_pose(&reversed.world, reversed.sensor, e.pose()), e.Status.Ok);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect_value(t, location.set_index, i32(0));
	testing.expect_value(t, e.bodies_sleep_group(&reversed.world, []e.Body_Handle{reversed.body}), e.Status.Ok);
	testing.expect_value(t, e.static_remove(&reversed.world, reversed.sensor), e.Status.Ok);
	location, _ = p.bodies_resolve(&reversed_sim.bodies, reversed.body);
	testing.expect_value(t, location.set_index, i32(0));

}

@(test)
triggers_mixed_reservation_failure_and_warmed_ownership :: proc(t:^testing.T)
{
	for fail in 0..<24
	{
		tracker:allocation_test_tracker;
		world:e.World;
		d: e.World_Description = small_world_description();
		d.gravity={};
		d.damping={};
		d.threading.worker_count=2;
		d.allocator=allocation_test_allocator(&tracker);
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, d, .All_Owned), e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=8, candidates_per_worker=16, child_capacity=64}), e.Status.Ok);
		if fail>0
		{
			tracker.fail_from=tracker.requests+fail;
		}
		status: e.Status = e.collider_parts_reserve(&world, {instance_capacity=8, part_capacity=16, pair_capacity=8, observations_per_worker=32, event_subscription=.Enabled});
		if status==.Ok
		{
			tracker.fail_from=0;
			shape: e.Shape_Handle;
			ss: e.Status;
			shape, ss = e.shape_add(&world, e.sphere(1));
			testing.expect_value(t, ss, e.Status.Ok);
			a: e.Static_Handle;
			as: e.Status;
			a, as = e.static_add(&world, e.static_body(shape, e.pose()), .None);
			testing.expect_value(t, as, e.Status.Ok);
			bs: e.Status;
			_, bs = e.static_add(&world, e.static_body(shape, e.pose({1, 0, 0})), .None);
			testing.expect_value(t, bs, e.Status.Ok);
			reference: p.Collidable_Reference;
			reference, _ = p.collidable_reference_static(a);
			_, status=e.collider_part_set(&world, reference, 0, {role=.Trigger, options={.Static_Static, .Stay}});
			testing.expect_value(t, status, e.Status.Ok);
			parent:[8]e.Trigger_Event;
			parts:[8]e.Collider_Part_Event;
			for _ in 0..<3
			{
				testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
				_, _, _=e.trigger_events_drain(&world, parent[:]);
				_, _, _=e.trigger_part_events_drain(&world, parts[:]);
			}
			requests: int = tracker.requests;
			tracker.fail_from=requests+1;
			for _ in 0..<20
			{
				testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
				_, _, _=e.trigger_events_drain(&world, parent[:]);
				_, _, _=e.trigger_part_events_drain(&world, parts[:]);
			}
			testing.expect_value(t, tracker.requests, requests);
		}
		else
		{
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			sim: ^e.Simulation;
			sim, _ = e.world_borrow_simulation(&world);
			testing.expect(t, sim.triggers.mixed==nil);
		}
		tracker.fail_from=tracker.requests+1;
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
}

@(test)
triggers_mixed_nested_admission_and_queries :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	inner: e.Shape_Handle;
	st: e.Status;
	inner, st = e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(s.shape, e.pose())});
	testing.expect_value(t, st, e.Status.Ok);
	outer: e.Shape_Handle;
	outer, st=e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(inner, e.pose()), e.compound_child(s.shape, e.pose({3, 0, 0}))});
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	point: e.Shape_Distance_Result;
	point_status: e.Status;
	point, point_status = e.shape_closest_point(&s.world, {0, 3, 0}, outer, e.pose());
	testing.expect_value(t, point_status, e.Status.Ok);
	testing.expect(t, abs(point.geometry.distance-2) < 1e-4);
	testing.expect_value(t, point.child_b, i32(0));
	distance: e.Shape_Distance_Result;
	distance_status: e.Status;
	distance, distance_status = e.shape_distance(&s.world, s.shape, e.pose({0, 4, 0}), outer, e.pose());
	testing.expect_value(t, distance_status, e.Status.Ok);
	testing.expect(t, abs(distance.geometry.distance-2) < 1e-4);
	result: e.Manifold_Result;
	collision_status: e.Status;
	result, collision_status = e.collision_query(&s.world, s.shape, e.pose({0, 1, 0}), outer, e.pose());
	testing.expect_value(t, collision_status, e.Status.Ok);
	testing.expect(t, result.nonconvex.count > 0);
	_, st=e.collider_part_get(&s.world, s.sensor_ref, 1);
	testing.expect_value(t, st, e.Status.Ok);
}

@(test)
triggers_mixed_filters_custom_routes_and_failed_sample :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2, .Kinematic) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	type_id: e.Shape_Type_ID;
	ok: bool;
	type_id, ok = register_custom_sphere(t, &s.world);
	if !ok
	{
		return;
	}
	custom: e.Shape_Handle;
	st: e.Status;
	custom, st = e.custom_shape_add(&s.world, type_id, &Custom_Sphere{radius=1});
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.body_set_shape(&s.world, s.body, custom), e.Status.Ok);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	// register the pre-existing compound traversal for the selected native type
	_, st=e.collision_task_register(&s.world, e.collision_task_compound(type_id, e.SHAPE_TYPE_COMPOUND, 16, .Convex_Compound, {.Subtask_Generator, .Child_Order}));
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n: int;
	status: e.Status;
	n, _, status = e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	committed: e.Collider_Part_Pair = parts[0].pair;
	policy: Trigger_Filter_Test = Trigger_Filter_Test{allow=.Allow, child=.Reject};
	saved: p.Narrow_Phase_Callbacks = sim.narrow_phase.callbacks;
	defer sim.narrow_phase.callbacks=saved;
	sim.narrow_phase.callbacks={user_context=&policy, allow=trigger_test_allow, allow_child=trigger_test_allow_child, configure=trigger_test_configure};
	testing.expect_value(t, e.trigger_mark_filters_dirty(&s.world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, parts[0].reason, e.Trigger_Exit_Reason.Filter_Changed);
	policy.child=.Allow;
	custom_step: Trigger_Custom_Step = Trigger_Custom_Step{mode=2};
	saved_step: p.Timestepper = sim.timestepper;
	defer sim.timestepper=saved_step;
	sim.timestepper={user_context=&custom_step, step=trigger_test_step};
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Invalid_Description);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 0);
	custom_step.mode=3;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, parts[0].pair.a.serial, committed.a.serial);
	testing.expect_value(t, parts[0].pair.b.serial, committed.b.serial);
	testing.expect_value(t, policy.configured, 0);
}

@(test)
triggers_mixed_shared_geometry_is_instance_owned :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	sensor: e.Static_State;
	sensor, _ = e.static_get(&s.world, s.sensor);
	sensor.pose=e.pose({20, 0, 0});
	other: e.Static_Handle;
	st: e.Status;
	other, st = e.static_add(&s.world, sensor, .None);
	testing.expect_value(t, st, e.Status.Ok);
	body: e.Body_State;
	body, _ = e.body_get(&s.world, s.body);
	body.pose=e.pose({23.5, 0, 0});
	visitor: e.Body_Handle;
	bs: e.Status;
	visitor, bs = e.body_add(&s.world, body);
	testing.expect_value(t, bs, e.Status.Ok);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 1);
	a: e.Body_State;
	a, _ = e.body_get(&s.world, s.body);
	b: e.Body_State;
	b, _ = e.body_get(&s.world, visitor);
	testing.expect_value(t, a.velocity.linear, util.Vector3{});
	testing.expect(t, b.velocity.linear.x>0);
	other_reference: e.Collidable_Reference;
	other_reference, _ = e.static_collidable_reference(&s.world, other);
	status: e.Status;
	_, status = e.collider_part_get(&s.world, other_reference, 1);
	testing.expect_value(t, status, e.Status.Not_Found);
	parent:[4]e.Trigger_Event;
	parts:[4]e.Collider_Part_Event;
	n:int;
	n, _, status=e.trigger_events_drain(&s.world, parent[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, status, e.Status.Ok);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, status, e.Status.Ok);
}

@(test)
triggers_mixed_dense_chunks_and_part_capacity_retry :: proc(t:^testing.T)
{
	pair_count :: 1025;
	for workers in ([3]i32{1, 2, 4})
	{
		world:e.World;
		d: e.World_Description = small_world_description();
		d.gravity={};
		d.threading.worker_count=workers;
		d.capacity.statics=2*pair_count;
		if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		st: e.Status;
		shape, st = e.shape_add(&world, e.sphere(1));
		testing.expect_value(t, st, e.Status.Ok);
		compound: e.Shape_Handle;
		cs: e.Status;
		compound, cs = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(shape, e.pose()), e.compound_child(shape, e.pose({3, 0, 0}))});
		testing.expect_value(t, cs, e.Status.Ok);
		testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=pair_count, candidates_per_worker=2*pair_count, child_capacity=4096}), e.Status.Ok);
		configuration: e.Collider_Part_Configuration = e.Collider_Part_Configuration{instance_capacity=2*pair_count, part_capacity=2*pair_count, pair_capacity=pair_count-1, observations_per_worker=2*pair_count, event_subscription=.Enabled};
		testing.expect_value(t, e.collider_parts_reserve(&world, configuration), e.Status.Ok);
		for index in 0..<pair_count
		{
			a: e.Static_Handle;
			as: e.Status;
			a, as = e.static_add(&world, e.static_body(compound, e.pose({f32(index*8), 0, 0})), .None);
			testing.expect_value(t, as, e.Status.Ok);
			bs: e.Status;
			_, bs = e.static_add(&world, e.static_body(shape, e.pose({f32(index*8)+3.5, 0, 0})), .None);
			testing.expect_value(t, bs, e.Status.Ok);
			reference: e.Collidable_Reference;
			reference, _ = e.static_collidable_reference(&world, a);
			status: e.Status;
			_, status = e.collider_part_set(&world, reference, 1, {role=.Trigger, user_id=u64(index+1), options={.Static_Static, .Stay}});
			testing.expect_value(t, status, e.Status.Ok);
		}
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&world);
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Capacity_Missing);
		testing.expect_value(t, sim.step_index, u64(0));
		testing.expect_value(t, sim.triggers.event_count, 0);
		testing.expect_value(t, sim.triggers.mixed.event_count, 0);
		testing.expect_value(t, sim.triggers.previous_count, 0);
		testing.expect_value(t, sim.triggers.mixed.previous_count, 0);
		configuration.pair_capacity=pair_count;
		testing.expect_value(t, e.collider_parts_reserve(&world, configuration), e.Status.Ok);
		parent: []e.Trigger_Event = make([]e.Trigger_Event, pair_count);
		defer delete(parent);
		parts: []e.Collider_Part_Event = make([]e.Collider_Part_Event, pair_count);
		defer delete(parts);
		for iteration in 0..<3
		{
			if iteration==2
			{
				testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
			}
			if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
			{
				return;
			}
			n: int;
			required: int;
			status: e.Status;
			n, required, status = e.trigger_events_drain(&world, parent);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, n, pair_count);
			testing.expect_value(t, required, pair_count);
			n, required, status=e.trigger_part_events_drain(&world, parts[:pair_count-1]);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, n, 0);
			testing.expect_value(t, required, pair_count);
			n, required, status=e.trigger_part_events_drain(&world, parts);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, n, pair_count);
			for &event in parts
			{
				testing.expect_value(t, event.kind, e.Trigger_Event_Kind.Enter if iteration==0 else e.Trigger_Event_Kind.Stay);
			}
			testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		}
	}
}

@(test)
triggers_mixed_static_options_are_per_part :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_remove(&s.world, s.body), e.Status.Ok);
	// only the far child opts into static/static. the nearer sensor must not inherit it
	st: e.Status;
	_, st = e.collider_part_set(&s.world, s.sensor_ref, 0, {role=.Trigger});
	testing.expect_value(t, st, e.Status.Ok);
	_, st=e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, options={.Static_Static}});
	testing.expect_value(t, st, e.Status.Ok);
	visitor: e.Static_Handle;
	vs: e.Status;
	visitor, vs = e.static_add(&s.world, e.static_body(s.shape, e.pose({0.5, 0, 0})), .None);
	testing.expect_value(t, vs, e.Status.Ok);
	parent:[4]e.Trigger_Event;
	parts:[4]e.Collider_Part_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n: int;
	status: e.Status;
	n, _, status = e.trigger_events_drain(&s.world, parent[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 0);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 0);
	testing.expect_value(t, e.static_set_pose(&s.world, visitor, e.pose({3.5, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	n, _, status=e.trigger_events_drain(&s.world, parent[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect(t, parts[0].pair.a.child_index==1 || parts[0].pair.b.child_index==1);
}

@(test)
triggers_mixed_mesh_surface_parts_follow_reduced_contacts :: proc(t:^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s:Trigger_Test_Scene;
		if trigger_test_scene(t, &s, workers, .All_Owned, .Kinematic) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		triangles: [2]p.Triangle = [2]p.Triangle{{{-4, 0, -4}, {-4, 0, 4}, {4, 0, 4}}, {{-4, 0, -4}, {4, 0, 4}, {4, 0, -4}}};
		mesh:p.Mesh;
		testing.expect_value(t, p.mesh_create(&mesh, &triangles[0], 2, {1, 1, 1}, sim.pool), e.Status.Ok);
		shape: e.Shape_Handle;
		st: e.Status;
		shape, st = e.shape_import_mesh(&s.world, &mesh);
		testing.expect_value(t, st, e.Status.Ok);
		testing.expect_value(t, p.mesh_dispose(&mesh, sim.pool), e.Status.Ok);
		testing.expect_value(t, e.trigger_remove(&s.world, s.sensor_ref), e.Status.Ok);
		testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(shape, e.pose()), .None), e.Status.Ok);
		testing.expect_value(t, e.collider_parts_reserve(&s.world, {instance_capacity=8, part_capacity=16, pair_capacity=16, observations_per_worker=64, event_subscription=.Enabled}), e.Status.Ok);
		_, st=e.collider_part_set(&s.world, s.sensor_ref, 0, {role=.Trigger, options={.Stay}, user_id=81});
		testing.expect_value(t, st, e.Status.Ok);
		parent:[8]e.Trigger_Event;
		parts:[8]e.Collider_Part_Event;
		for pos in ([5]util.Vector3{{0, -0.5, 0}, {1, -0.5, 0}, {3.9, -0.5, 0}, {8, -0.5, 0}, {0, -0.5, 0}})
		{
			testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose(pos)), e.Status.Ok);
			if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
			{
				return;
			}
			n: int;
			status: e.Status;
			n, _, status = e.trigger_events_drain(&s.world, parent[:]);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, n, 1);
			m: int;
			status2: e.Status;
			m, _, status2 = e.trigger_part_events_drain(&s.world, parts[:]);
			testing.expect_value(t, status2, e.Status.Ok);
			testing.expect_value(t, m, n);
			if n>0 && m>0
			{
				testing.expect_value(t, parts[0].kind, parent[0].kind);
				testing.expect_value(t, parts[0].pair.a.child_index, i32(0));
				testing.expect_value(t, parts[0].pair.b.child_index, i32(0));
			}
			testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		}
	}
}

@(test)
triggers_mixed_big_compound_missing_child_route_preserves_history :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	big: e.Shape_Handle;
	status: e.Status;
	big, status = e.shape_import_big_compound(&s.world, []e.Compound_Child{e.compound_child(s.shape, e.pose()), e.compound_child(s.shape, e.pose({3, 0, 0}))});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(big, e.pose()), .None), e.Status.Ok);
	_, status=e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	route_index: int = p.collision_task_matrix_index(int(e.SHAPE_TYPE_SPHERE), int(e.SHAPE_TYPE_SPHERE));
	saved: p.Collision_Task_Reference = sim.collision_tasks.routes[route_index];
	defer sim.collision_tasks.routes[route_index]=saved;
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	for x in ([2]f32{3.5, 0.5})
	{
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({x, 0, 0})), e.Status.Ok);
		step: u64 = sim.step_index;
		retained: int = sim.triggers.previous_count;
		part_retained: int = sim.triggers.mixed.previous_count;
		sim.collision_tasks.routes[route_index].task_id=-1;
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Not_Found);
		testing.expect_value(t, sim.step_index, step);
		testing.expect_value(t, sim.triggers.previous_count, retained);
		testing.expect_value(t, sim.triggers.mixed.previous_count, part_retained);
		sim.collision_tasks.routes[route_index]=saved;
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		n: int;
		drain: e.Status;
		n, _, drain = e.trigger_events_drain(&s.world, parent[:]);
		testing.expect_value(t, drain, e.Status.Ok);
		testing.expect_value(t, n, 1);
		m: int;
		part_drain: e.Status;
		m, _, part_drain = e.trigger_part_events_drain(&s.world, parts[:]);
		testing.expect_value(t, part_drain, e.Status.Ok);
		testing.expect_value(t, m, 1);
		if n==1 && m==1
		{
			testing.expect_value(t, parent[0].kind, e.Trigger_Event_Kind.Enter if x>3 else e.Trigger_Event_Kind.Exit);
			testing.expect_value(t, parts[0].kind, parent[0].kind);
		}
	}
}

@(test)
triggers_mixed_last_sensor_returns_to_solid_workers :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	_, _, _=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, sim.triggers.mixed.trigger_count, 1);
	testing.expect_value(t, e.collider_part_remove(&s.world, s.sensor_ref, 1), e.Status.Ok);
	testing.expect_value(t, sim.triggers.mixed.trigger_count, 0);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	_, _, _=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect(t, sim.triggers.mixed.parts.count>0);
	// retained visitor lifetime is not a sensor
	sim.triggers.workers[0].status=.Not_Found;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	testing.expect_value(t, sim.triggers.workers[0].status, e.Status.Not_Found);
	// no trigger worker touched it
	st: e.Status;
	_, st = e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger});
	testing.expect_value(t, st, e.Status.Ok);
	testing.expect_value(t, sim.triggers.mixed.trigger_count, 1);
	testing.expect_value(t, e.static_remove(&s.world, s.sensor, .None), e.Status.Ok);
	testing.expect_value(t, sim.triggers.mixed.trigger_count, 0);
}

@(test)
triggers_mixed_contact_stepper_three_stream_retry :: proc(t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s: Trigger_Test_Scene;
		if mixed_test_scene(t, &s, workers) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1.5, 0, 0})), e.Status.Ok);
		settings_status: e.Status;
		_, settings_status = e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, options={.Stay}});
		testing.expect_value(t, settings_status, e.Status.Ok);
		tracker: e.Contact_Tracker;
		testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
		defer e.contact_tracker_destroy(&tracker);
		testing.expect_value(t, e.contact_tracker_bind(&tracker, &s.world), e.Status.Ok);
		stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
		parent: [8]e.Trigger_Event;
		parts: [8]e.Collider_Part_Event;
		contacts: [8]e.Contact_Event;
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		result: e.Collider_Part_Contact_Update_Result;
		status: e.Status;
		result, status = e.collider_part_stepper_update_with_contacts(
			&stepper, &s.world, 3.0/64, parent[:], parts[:], &tracker, nil,
		);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, result.completed_steps, i32(1));
		testing.expect_value(t, result.contact_required, i32(1));
		testing.expect_value(t, result.pending, e.Collider_Part_Pending_Streams{.Parent, .Part, .Contact});
		testing.expect_value(t, result.events_written + result.part_events_written + result.contact_events_written, i32(0));
		testing.expect_value(t, sim.step_index, u64(1));
		testing.expect_value(t, tracker.last_step_index, u64(0));
		testing.expect_value(t, stepper.accumulator, f32(2.0/64));
		for attempt in 0..<3
		{
			result, status = e.collider_part_stepper_update_with_contacts(
				&stepper, &s.world, 0, parent[:], parts[:], &tracker, nil,
			);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, result.completed_steps, i32(0));
			testing.expect_value(t, sim.step_index, u64(1));
			_ = attempt;
		}
		result, status = e.collider_part_stepper_update_with_contacts(
			&stepper, &s.world, 0, parent[:], nil, &tracker, contacts[:],
		);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, result.contact_events_written, i32(1));
		testing.expect_value(t, result.completed_steps, i32(0));
		testing.expect_value(t, result.pending, e.Collider_Part_Pending_Streams{.Parent, .Part});
		testing.expect_value(t, tracker.last_step_index, u64(1));
		testing.expect_value(t, contacts[0].kind, e.Contact_Event_Kind.Begin);
		result, status = e.collider_part_stepper_update_with_contacts(
			&stepper, &s.world, 0, parent[:], parts[:], &tracker, contacts[:],
		);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.completed_steps, i32(2));
		testing.expect_value(t, result.contact_events_written, i32(2));
		testing.expect_value(t, result.events_written, i32(3));
		testing.expect_value(t, result.part_events_written, i32(3));
		testing.expect_value(t, result.required + result.part_required + result.contact_required, i32(0));
		testing.expect_value(t, result.pending, e.Collider_Part_Pending_Streams{});
		testing.expect_value(t, sim.step_index, u64(3));
		testing.expect_value(t, tracker.last_step_index, u64(3));
		testing.expect_value(t, stepper.accumulator, f32(0));
		for index in 0..<3
		{
			testing.expect_value(t, parts[index].step, u64(index + 1));
			testing.expect_value(t, parts[index].step, parent[index].step);
		}
		result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 0, nil, nil, &tracker, nil);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.completed_steps + result.contact_events_written, i32(0));
	}
}

@(test)
triggers_mixed_contact_history_capacity_and_manual_drain :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1.5, 0, 0})), e.Status.Ok);
	tracker: e.Contact_Tracker;
	testing.expect_value(t, e.contact_tracker_init(&tracker, 0), e.Status.Ok);
	defer e.contact_tracker_destroy(&tracker);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &s.world), e.Status.Ok);
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	parent: [8]e.Trigger_Event;
	parts: [8]e.Collider_Part_Event;
	contacts: [8]e.Contact_Event;
	result: e.Collider_Part_Contact_Update_Result;
	status: e.Status;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 1.0/64, parent[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.contact_required, i32(1));
	testing.expect_value(t, tracker.last_step_index, u64(0));
	testing.expect_value(t, e.contact_tracker_ensure_capacity(&tracker, 8), e.Status.Ok);
	count: int;
	drain_status: e.Status;
	count, _, drain_status = e.contact_events_drain(&tracker, contacts[:]);
	testing.expect_value(t, drain_status, e.Status.Ok);
	testing.expect_value(t, count, 1);
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 0, parent[:], parts[:], &tracker, nil);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.contact_events_written + result.completed_steps, i32(0));
	testing.expect_value(t, result.events_written, i32(1));
	testing.expect_value(t, result.part_events_written, i32(1));
	testing.expect_value(t, result.contact_required, i32(0));
}

@(test)
triggers_mixed_contact_stepper_rejects_wrong_or_skipped_history :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	other: Trigger_Test_Scene;
	if mixed_test_scene(t, &s) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	if mixed_test_scene(t, &other) != .Ok
	{
		return;
	}
	defer e.world_destroy(&other.world);
	tracker: e.Contact_Tracker;
	testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
	defer e.contact_tracker_destroy(&tracker);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &other.world), e.Status.Ok);
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	result: e.Collider_Part_Contact_Update_Result;
	status: e.Status;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 1.0/64, nil, nil, &tracker, nil);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, result.completed_steps, i32(0));
	testing.expect_value(t, stepper.accumulator, f32(0));
	testing.expect_value(t, e.contact_tracker_unbind(&tracker), e.Status.Ok);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &s.world), e.Status.Ok);
	for _ in 0..<2
	{
		testing.expect_value(t, e.world_step(&s.world, 1.0/64), e.Status.Ok);
		testing.expect_value(t, e.trigger_events_discard(&s.world), e.Status.Ok);
		testing.expect_value(t, e.trigger_part_events_discard(&s.world), e.Status.Ok);
	}
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 1.0/64, nil, nil, &tracker, nil);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, tracker.last_step_index, u64(0));
	testing.expect_value(t, stepper.accumulator, f32(0));
}

@(test)
triggers_mixed_contact_stepper_failed_and_missing_samples :: proc(t: ^testing.T)
{
	s: Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({1.5, 0, 0})), e.Status.Ok);
	tracker: e.Contact_Tracker;
	testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
	defer e.contact_tracker_destroy(&tracker);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &s.world), e.Status.Ok);
	sim: ^p.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	custom: Trigger_Custom_Step = {mode=2};
	sim.timestepper = {step=trigger_test_step, user_context=&custom};
	parents: [8]e.Trigger_Event;
	parts: [8]e.Collider_Part_Event;
	contacts: [8]e.Contact_Event;
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	result: e.Collider_Part_Contact_Update_Result;
	status: e.Status;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 1.0/64, parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	testing.expect_value(t, result.completed_steps + result.events_written + result.part_events_written + result.contact_events_written, i32(0));
	testing.expect_value(t, stepper.accumulator, f32(1.0/64));
	testing.expect_value(t, sim.step_index, u64(0));
	// a failed stage does not publish history. a recovered custom step may sample
	// twice, but publishes and consumes the completed time exactly once
	custom.mode = 3;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 0, parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.events_written, i32(1));
	testing.expect_value(t, result.part_events_written, i32(1));
	testing.expect_value(t, result.contact_events_written, i32(1));
	testing.expect_value(t, tracker.last_step_index, u64(1));
	// the custom step reports completion without sampling. its successful time
	// is consumed, then notification failure blocks catch-up until explicit reset
	custom.mode = 1;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 2.0/64, parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, stepper.accumulator, f32(1.0/64));
	testing.expect_value(t, sim.step_index, u64(2));
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 0, parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, result.completed_steps, i32(0));
	testing.expect_value(t, sim.step_index, u64(2));
	testing.expect_value(t, e.trigger_reset_history(&s.world), e.Status.Ok);
	testing.expect_value(t, e.contact_tracker_clear(&tracker), e.Status.Ok);
	custom.mode = 0;
	result, status = e.collider_part_stepper_update_with_contacts(&stepper, &s.world, 0, parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, sim.step_index, u64(3));
	testing.expect_value(t, stepper.accumulator, f32(0));
}

@(test)
triggers_mixed_contact_stepper_warmed_all_owned :: proc(t: ^testing.T)
{
	allocations: allocation_test_tracker;
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	description.threading.worker_count = 2;
	description.allocator = allocation_test_allocator(&allocations);
	world: e.World;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=8, candidates_per_worker=32, child_capacity=64}), e.Status.Ok);
	testing.expect_value(t, e.collider_parts_reserve(&world, {instance_capacity=8, part_capacity=16, pair_capacity=8, observations_per_worker=32, event_subscription=.Enabled}), e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(&world, e.sphere(1));
	testing.expect_value(t, status, e.Status.Ok);
	children: [2]e.Compound_Child = {e.compound_child(shape, e.pose()), e.compound_child(shape, e.pose({3, 0, 0}))};
	compound: e.Shape_Handle;
	compound, status = e.shape_import_compound(&world, children[:]);
	testing.expect_value(t, status, e.Status.Ok);
	stat: e.Static_Handle;
	stat, status = e.static_add(&world, e.static_body(compound, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	reference: e.Collidable_Reference;
	reference, status = e.static_collidable_reference(&world, stat);
	testing.expect_value(t, status, e.Status.Ok);
	_, status = e.collider_part_set(&world, reference, 1, {role=.Trigger, options={.Stay}});
	testing.expect_value(t, status, e.Status.Ok);
	mass: e.Body_Inertia;
	mass, status = e.shape_inertia(e.sphere(1), 1);
	testing.expect_value(t, status, e.Status.Ok);
	body: e.Body_Handle;
	body, status = e.body_add(&world, e.body_dynamic(shape, mass, e.pose({1.5, 0, 0}), {}, e.body_activity(-1, 255)));
	testing.expect_value(t, status, e.Status.Ok);
	lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({1.5, 0, 0}));
	lock.linear_axes = {.X, .Y, .Z};
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	// the tracker keeps its independently selected allocator. its fixed arrays
	// are allocated at initialization, not through the world's All_Owned scope
	tracker: e.Contact_Tracker;
	testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &world), e.Status.Ok);
	parents: [4]e.Trigger_Event;
	parts: [4]e.Collider_Part_Event;
	contacts: [4]e.Contact_Event;
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	result: e.Collider_Part_Contact_Update_Result;
	for _ in 0..<8
	{
		result, status = e.collider_part_stepper_update_with_contacts(&stepper, &world, 1.0/64, parents[:], parts[:], &tracker, contacts[:]);
		testing.expect_value(t, status, e.Status.Ok);
	}
	first_entries: rawptr = tracker.tables[0].entries;
	second_entries: rawptr = tracker.tables[1].entries;
	requests: int = allocations.requests;
	allocations.fail_from = requests+1;
	for iteration in 0..<100
	{
		result, status = e.collider_part_stepper_update_with_contacts(&stepper, &world, 1.0/64, parents[:], parts[:], &tracker, contacts[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.completed_steps, i32(1));
		if result.events_written != 1 || result.part_events_written != 1 || result.contact_events_written != 1
		{
			state: e.Body_State;
			state, _ = e.body_get(&world, body);
			testing.expectf(t, false, "sample=%d position=%v velocity=%v result=%v", iteration, state.pose.position, state.velocity.linear, result);
			break;
		}
	}
	testing.expect_value(t, allocations.requests, requests);
	testing.expect_value(t, rawptr(tracker.tables[0].entries), first_entries);
	testing.expect_value(t, rawptr(tracker.tables[1].entries), second_entries);
	testing.expect_value(t, e.contact_tracker_destroy(&tracker), e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, allocations.requests, requests);
	allocation_test_empty(t, &allocations);
}

@(test)
triggers_nested_parts_preserve_response_filter_and_top_identity :: proc(t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		for outer_type in ([2]e.Shape_Type_ID{e.SHAPE_TYPE_COMPOUND, e.SHAPE_TYPE_BIG_COMPOUND})
		{
			s: Trigger_Test_Scene;
			if mixed_test_scene(t, &s, workers) != .Ok
			{
				return;
			}
			defer e.world_destroy(&s.world);
			inner: e.Shape_Handle;
			status: e.Status;
			inner, status = e.shape_import_big_compound(&s.world, []e.Compound_Child{
					e.compound_child(s.shape, e.pose()), e.compound_child(s.shape, e.pose({0, 0.2, 0}))});
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				return;
			}
			children: [2]e.Compound_Child = {e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({3, 0, 0}))};
			outer: e.Shape_Handle;
			if outer_type == e.SHAPE_TYPE_BIG_COMPOUND
			{
				outer, status = e.shape_import_big_compound(&s.world, children[:]);
			}
			else
			{
				outer, status = e.shape_import_compound(&s.world, children[:]);
			}
			if !testing.expect_value(t, status, e.Status.Ok)
			{
				return;
			}
			testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(outer, e.pose()), .None), e.Status.Ok);
			_, status = e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, user_id=91, options={.Stay}});
			testing.expect_value(t, status, e.Status.Ok);
			sim: ^e.Simulation;
			sim, _ = e.world_borrow_simulation(&s.world);
			parent: [8]e.Trigger_Event;
			parts: [8]e.Collider_Part_Event;
			for position in ([3]util.Vector3{{3.5, 0, 0}, {3.4, 0, 0}, {0.5, 0, 0}})
			{
				testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose(position)), e.Status.Ok);
				testing.expect_value(t, e.body_set_velocity(&s.world, s.body, {}), e.Status.Ok);
				if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
				{
					return;
				}
				n: int;
				st: e.Status;
				n, _, st = e.trigger_events_drain(&s.world, parent[:]);
				testing.expect_value(t, st, e.Status.Ok);
				testing.expect_value(t, n, 1);
				m: int;
				pst: e.Status;
				m, _, pst = e.trigger_part_events_drain(&s.world, parts[:]);
				testing.expect_value(t, pst, e.Status.Ok);
				testing.expect_value(t, m, 1);
				if m > 0
				{
					part: e.Collider_Part_Endpoint = parts[0].pair.a;
					if part.reference.packed != s.sensor_ref.packed
					{
						part = parts[0].pair.b;
					}
					testing.expect_value(t, part.child_index, i32(1));
					testing.expect_value(t, part.user_id, u64(91));
				}
				body: e.Body_State;
				body, _ = e.body_get(&s.world, s.body);
				if position.x > 3
				{
					testing.expect_value(t, body.velocity.linear, util.Vector3{});
					testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
				}
				else
				{
					testing.expect(t, body.velocity.linear.x > 0);
					testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 1);
				}
			}
		}
	}
}

@(test)
triggers_nested_mesh_child_queries_and_reduced_events :: proc(t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		s: Trigger_Test_Scene;
		if mixed_test_scene(t, &s, workers, .Kinematic) != .Ok
		{
			return;
		}
		defer e.world_destroy(&s.world);
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&s.world);
		triangles: [2]p.Triangle = {{{-4, 0, -4}, {-4, 0, 4}, {4, 0, 4}}, {{-4, 0, -4}, {4, 0, 4}, {4, 0, -4}}};
		mesh: p.Mesh;
		testing.expect_value(t, p.mesh_create(&mesh, &triangles[0], 2, {1, 1, 1}, sim.pool), e.Status.Ok);
		mesh_shape: e.Shape_Handle;
		status: e.Status;
		mesh_shape, status = e.shape_import_mesh(&s.world, &mesh);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, p.mesh_dispose(&mesh, sim.pool), e.Status.Ok);
		inner: e.Shape_Handle;
		status2: e.Status;
		inner, status2 = e.shape_import_big_compound(&s.world, []e.Compound_Child{e.compound_child(mesh_shape, e.pose())});
		if !testing.expect_value(t, status2, e.Status.Ok)
		{
			return;
		}
		outer: e.Shape_Handle;
		status3: e.Status;
		outer, status3 = e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(s.shape, e.pose({20, 0, 0})), e.compound_child(inner, e.pose())});
		if !testing.expect_value(t, status3, e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(outer, e.pose()), .None), e.Status.Ok);
		_, status = e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger, options={.Stay}});
		testing.expect_value(t, status, e.Status.Ok);
		point: e.Shape_Distance_Result;
		point_status: e.Status;
		point, point_status = e.shape_closest_point(&s.world, {0, -3, 0}, outer, e.pose());
		testing.expect_value(t, point_status, e.Status.Ok);
		testing.expect(t, abs(point.geometry.distance-3) < 1e-4);
		testing.expect_value(t, point.child_b, i32(1));
		filter: e.Query_Filter = {include=e.COLLIDABLE_STATIC};
		sweep: e.Sweep_Hit;
		sweep_status: e.Status;
		sweep, sweep_status = e.sweep_closest(&s.world, s.shape, e.pose({0, -3, 0}), e.velocity({0, 1, 0}), 5, filter);
		testing.expect_value(t, sweep_status, e.Status.Ok);
		testing.expect(t, abs(sweep.sweep.t1-2) < 0.02);
		testing.expect_value(t, sweep.sweep.child_b, i32(1));
		testing.expect_value(t, e.distance_query_reserve(&s.world), e.Status.Ok);
		correction: e.Shape_Correction_Result;
		correction_status: e.Status;
		correction, correction_status = e.shape_depenetrate(&s.world, s.shape, e.pose({0, -0.5, 0}), outer, e.pose());
		testing.expect_value(t, correction_status, e.Status.Ok);
		testing.expect(t, correction.translation.y < -0.45);
		parent: [8]e.Trigger_Event;
		parts: [8]e.Collider_Part_Event;
		for position in ([4]util.Vector3{{0, -0.5, 0}, {1, -0.5, 0}, {8, -0.5, 0}, {0, -0.5, 0}})
		{
			testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose(position)), e.Status.Ok);
			if !testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok)
			{
				return;
			}
			n: int;
			st: e.Status;
			n, _, st = e.trigger_events_drain(&s.world, parent[:]);
			testing.expect_value(t, st, e.Status.Ok);
			testing.expect_value(t, n, 1);
			m: int;
			pst: e.Status;
			m, _, pst = e.trigger_part_events_drain(&s.world, parts[:]);
			testing.expect_value(t, pst, e.Status.Ok);
			testing.expect_value(t, m, 1);
			if m > 0
			{
				testing.expect_value(t, parts[0].pair.a.child_index, i32(1));
			}
			testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		}
	}
}

@(test)
triggers_nested_shared_dag_remove_clear_and_depth_bound :: proc(t: ^testing.T)
{
	world: e.World;
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer e.world_destroy(&world);
	sphere: e.Shape_Handle;
	sphere, _ = e.shape_add(&world, e.sphere(1));
	inner: e.Shape_Handle;
	st: e.Status;
	inner, st = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(sphere, e.pose())});
	testing.expect_value(t, st, e.Status.Ok);
	a: e.Shape_Handle;
	sta: e.Status;
	a, sta = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(inner, e.pose())});
	b: e.Shape_Handle;
	stb: e.Status;
	b, stb = e.shape_import_big_compound(&world, []e.Compound_Child{e.compound_child(inner, e.pose())});
	testing.expect_value(t, sta, e.Status.Ok);
	testing.expect_value(t, stb, e.Status.Ok);
	root: e.Shape_Handle;
	str: e.Status;
	root, str = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(a, e.pose()), e.compound_child(b, e.pose())});
	testing.expect_value(t, str, e.Status.Ok);
	scratch: [32]e.Shape_Handle;
	removed: int;
	status: e.Status;
	removed, _, status = e.shape_remove_recursive(&world, root, scratch[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, removed, 5);
	sphere, _ = e.shape_add(&world, e.sphere(1));
	last: e.Shape_Handle = sphere;
	for _ in 0..=p.MAXIMUM_COMPOUND_HIERARCHY_DEPTH
	{
		last, status = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(last, e.pose())});
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
	}
	_, status = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(last, e.pose())});
	testing.expect_value(t, status, e.Status.Invalid_Description);
	testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
}

@(test)
triggers_nested_dense_chunks_and_part_capacity_retry :: proc(t:^testing.T)
{
	pair_count :: 1025;
	for workers in ([3]i32{1, 2, 4})
	{
		world:e.World;
		d: e.World_Description = small_world_description();
		d.gravity={};
		d.threading.worker_count=workers;
		d.capacity.statics=2*pair_count;
		if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		st: e.Status;
		shape, st = e.shape_add(&world, e.sphere(1));
		testing.expect_value(t, st, e.Status.Ok);
		inner: e.Shape_Handle;
		ins: e.Status;
		inner, ins = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(shape, e.pose())});
		testing.expect_value(t, ins, e.Status.Ok);
		compound: e.Shape_Handle;
		cs: e.Status;
		compound, cs = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({3, 0, 0}))});
		testing.expect_value(t, cs, e.Status.Ok);
		testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=pair_count, candidates_per_worker=2*pair_count, child_capacity=4096}), e.Status.Ok);
		configuration: e.Collider_Part_Configuration = e.Collider_Part_Configuration{instance_capacity=2*pair_count, part_capacity=2*pair_count, pair_capacity=pair_count-1, observations_per_worker=2*pair_count, event_subscription=.Enabled};
		testing.expect_value(t, e.collider_parts_reserve(&world, configuration), e.Status.Ok);
		for index in 0..<pair_count
		{
			a: e.Static_Handle;
			as: e.Status;
			a, as = e.static_add(&world, e.static_body(compound, e.pose({f32(index*8), 0, 0})), .None);
			testing.expect_value(t, as, e.Status.Ok);
			bs: e.Status;
			_, bs = e.static_add(&world, e.static_body(shape, e.pose({f32(index*8)+3.5, 0, 0})), .None);
			testing.expect_value(t, bs, e.Status.Ok);
			reference: e.Collidable_Reference;
			reference, _ = e.static_collidable_reference(&world, a);
			status: e.Status;
			_, status = e.collider_part_set(&world, reference, 1, {role=.Trigger, user_id=u64(index+1), options={.Static_Static, .Stay}});
			testing.expect_value(t, status, e.Status.Ok);
		}
		sim: ^e.Simulation;
		sim, _ = e.world_borrow_simulation(&world);
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Capacity_Missing);
		testing.expect_value(t, sim.step_index, u64(0));
		testing.expect_value(t, sim.triggers.event_count, 0);
		testing.expect_value(t, sim.triggers.mixed.event_count, 0);
		testing.expect_value(t, sim.triggers.previous_count, 0);
		testing.expect_value(t, sim.triggers.mixed.previous_count, 0);
		configuration.pair_capacity=pair_count;
		testing.expect_value(t, e.collider_parts_reserve(&world, configuration), e.Status.Ok);
		parent: []e.Trigger_Event = make([]e.Trigger_Event, pair_count);
		defer delete(parent);
		parts: []e.Collider_Part_Event = make([]e.Collider_Part_Event, pair_count);
		defer delete(parts);
		for iteration in 0..<3
		{
			if iteration==2
			{
				testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
			}
			if !testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok)
			{
				return;
			}
			n: int;
			required: int;
			status: e.Status;
			n, required, status = e.trigger_events_drain(&world, parent);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, n, pair_count);
			testing.expect_value(t, required, pair_count);
			n, required, status=e.trigger_part_events_drain(&world, parts[:pair_count-1]);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, n, 0);
			testing.expect_value(t, required, pair_count);
			n, required, status=e.trigger_part_events_drain(&world, parts);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, n, pair_count);
			for &event in parts
			{
				testing.expect_value(t, event.kind, e.Trigger_Event_Kind.Enter if iteration==0 else e.Trigger_Event_Kind.Stay);
			}
			testing.expect_value(t, sim.narrow_phase.pair_cache.mapping.count, 0);
		}
	}
}

@(test)
triggers_nested_allocation_failure_retains_children_and_warmed_queries :: proc(t: ^testing.T)
{
	for failure in 0..=24
	{
		tracker: allocation_test_tracker;
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.threading.worker_count = 2;
		description.allocator = allocation_test_allocator(&tracker);
		world: e.World;
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
		{
			return;
		}
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.sphere(1));
		inner: e.Shape_Handle;
		inner_status: e.Status;
		inner, inner_status = e.shape_import_big_compound(&world, []e.Compound_Child{e.compound_child(shape, e.pose())});
		if !testing.expect_value(t, inner_status, e.Status.Ok)
		{
			return;
		}
		before: e.Shape_Info;
		_, before, _ = e.shape_borrow_raw(&world, inner);
		if failure > 0
		{
			tracker.fail_from = tracker.requests+failure;
		}
		outer: e.Shape_Handle;
		status: e.Status;
		outer, status = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({3, 0, 0}))});
		if status != .Ok
		{
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			after: e.Shape_Info;
			_, after, _ = e.shape_borrow_raw(&world, inner);
			testing.expect_value(t, after.reference_count, before.reference_count);
		}
		else if failure == 0
		{
			stat: e.Static_Handle;
			static_status: e.Status;
			stat, static_status = e.static_add(&world, e.static_body(outer, e.pose()), .None);
			testing.expect_value(t, static_status, e.Status.Ok);
			body: e.Body_Handle;
			body, _ = e.body_add(&world, e.body_kinematic(shape, e.pose({3.5, 0, 0}), {}, e.body_activity(-1, 255)));
			testing.expect(t, body != e.body_handle_invalid());
			testing.expect_value(t, e.world_enable_triggers(&world, {pair_capacity=16, candidates_per_worker=32, child_capacity=256}), e.Status.Ok);
			testing.expect_value(t, e.collider_parts_reserve(&world, {instance_capacity=8, part_capacity=16, pair_capacity=16, observations_per_worker=64, event_subscription=.Enabled}), e.Status.Ok);
			reference: e.Collidable_Reference;
			reference, _ = e.static_collidable_reference(&world, stat);
			_, status = e.collider_part_set(&world, reference, 1, {role=.Trigger, options={.Stay}});
			testing.expect_value(t, status, e.Status.Ok);
			query: e.Query_Context;
			testing.expect_value(t, e.query_context_init(&query, &world), e.Status.Ok);
			parent: [4]e.Trigger_Event;
			parts: [4]e.Collider_Part_Event;
			for _ in 0..<3
			{
				testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
				_, _, _ = e.trigger_events_drain(&world, parent[:]);
				_, _, _ = e.trigger_part_events_drain(&world, parts[:]);
				_, status = e.collision_query_with_context(&query, shape, e.pose({3.5, 0, 0}), outer, e.pose());
				testing.expect_value(t, status, e.Status.Ok);
			}
			requests: int = tracker.requests;
			tracker.fail_from = requests+1;
			for _ in 0..<100
			{
				testing.expect_value(t, e.trigger_mark_filters_dirty(&world), e.Status.Ok);
				testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
				_, _, _ = e.trigger_events_drain(&world, parent[:]);
				_, _, _ = e.trigger_part_events_drain(&world, parts[:]);
				_, status = e.collision_query_with_context(&query, shape, e.pose({3.5, 0, 0}), outer, e.pose());
				testing.expect_value(t, status, e.Status.Ok);
			}
			testing.expect_value(t, tracker.requests, requests);
			testing.expect_value(t, e.query_context_destroy(&query), e.Status.Ok);
		}
		tracker.fail_from = tracker.requests+1;
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
}

@(test)
triggers_nested_missing_child_route_preserves_history :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	inner: e.Shape_Handle;
	st: e.Status;
	inner, st = e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(s.shape, e.pose())});
	testing.expect_value(t, st, e.Status.Ok);
	big: e.Shape_Handle;
	status: e.Status;
	big, status = e.shape_import_big_compound(&s.world, []e.Compound_Child{e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({3, 0, 0}))});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(big, e.pose()), .None), e.Status.Ok);
	_, status=e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger});
	testing.expect_value(t, status, e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	route_index: int = p.collision_task_matrix_index(int(e.SHAPE_TYPE_SPHERE), int(e.SHAPE_TYPE_SPHERE));
	saved: p.Collision_Task_Reference = sim.collision_tasks.routes[route_index];
	defer sim.collision_tasks.routes[route_index]=saved;
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	for x in ([2]f32{3.5, 0.5})
	{
		testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({x, 0, 0})), e.Status.Ok);
		step: u64 = sim.step_index;
		retained: int = sim.triggers.previous_count;
		part_retained: int = sim.triggers.mixed.previous_count;
		sim.collision_tasks.routes[route_index].task_id=-1;
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Not_Found);
		testing.expect_value(t, sim.step_index, step);
		testing.expect_value(t, sim.triggers.previous_count, retained);
		testing.expect_value(t, sim.triggers.mixed.previous_count, part_retained);
		sim.collision_tasks.routes[route_index]=saved;
		testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
		n: int;
		drain: e.Status;
		n, _, drain = e.trigger_events_drain(&s.world, parent[:]);
		testing.expect_value(t, drain, e.Status.Ok);
		testing.expect_value(t, n, 1);
		m: int;
		part_drain: e.Status;
		m, _, part_drain = e.trigger_part_events_drain(&s.world, parts[:]);
		testing.expect_value(t, part_drain, e.Status.Ok);
		testing.expect_value(t, m, 1);
		if n==1 && m==1
		{
			testing.expect_value(t, parent[0].kind, e.Trigger_Event_Kind.Enter if x>3 else e.Trigger_Event_Kind.Exit);
			testing.expect_value(t, parts[0].kind, parent[0].kind);
		}
	}
}

@(test)
triggers_nested_custom_routes_filters_and_failed_sample :: proc(t:^testing.T)
{
	s:Trigger_Test_Scene;
	if mixed_test_scene(t, &s, 2, .Kinematic) != .Ok
	{
		return;
	}
	defer e.world_destroy(&s.world);
	inner: e.Shape_Handle;
	inner_status: e.Status;
	inner, inner_status = e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(s.shape, e.pose())});
	testing.expect_value(t, inner_status, e.Status.Ok);
	outer: e.Shape_Handle;
	outer_status: e.Status;
	outer, outer_status = e.shape_import_compound(&s.world, []e.Compound_Child{e.compound_child(inner, e.pose()), e.compound_child(inner, e.pose({3, 0, 0}))});
	testing.expect_value(t, outer_status, e.Status.Ok);
	testing.expect_value(t, e.static_apply(&s.world, s.sensor, e.static_body(outer, e.pose()), .None), e.Status.Ok);
	bind_status: e.Status;
	_, bind_status = e.collider_part_set(&s.world, s.sensor_ref, 1, {role=.Trigger});
	testing.expect_value(t, bind_status, e.Status.Ok);
	type_id: e.Shape_Type_ID;
	ok: bool;
	type_id, ok = register_custom_sphere(t, &s.world);
	if !ok
	{
		return;
	}
	custom: e.Shape_Handle;
	st: e.Status;
	custom, st = e.custom_shape_add(&s.world, type_id, &Custom_Sphere{radius=1});
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.body_set_shape(&s.world, s.body, custom), e.Status.Ok);
	testing.expect_value(t, e.body_set_pose(&s.world, s.body, e.pose({3.5, 0, 0})), e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&s.world);
	// register the pre-existing compound traversal for the selected native type
	_, st=e.collision_task_register(&s.world, e.collision_task_compound(type_id, e.SHAPE_TYPE_COMPOUND, 16, .Convex_Compound, {.Subtask_Generator, .Child_Order}));
	if !testing.expect_value(t, st, e.Status.Ok)
	{
		return;
	}
	parent:[8]e.Trigger_Event;
	parts:[8]e.Collider_Part_Event;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n: int;
	status: e.Status;
	n, _, status = e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	committed: e.Collider_Part_Pair = parts[0].pair;
	policy: Trigger_Filter_Test = Trigger_Filter_Test{allow=.Allow, child=.Reject};
	saved: p.Narrow_Phase_Callbacks = sim.narrow_phase.callbacks;
	defer sim.narrow_phase.callbacks=saved;
	sim.narrow_phase.callbacks={user_context=&policy, allow=trigger_test_allow, allow_child=trigger_test_allow_child, configure=trigger_test_configure};
	testing.expect_value(t, e.trigger_mark_filters_dirty(&s.world), e.Status.Ok);
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, parts[0].reason, e.Trigger_Exit_Reason.Filter_Changed);
	policy.child=.Allow;
	custom_step: Trigger_Custom_Step = Trigger_Custom_Step{mode=2};
	saved_step: p.Timestepper = sim.timestepper;
	defer sim.timestepper=saved_step;
	sim.timestepper={user_context=&custom_step, step=trigger_test_step};
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Invalid_Description);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 0);
	custom_step.mode=3;
	testing.expect_value(t, e.world_step(&s.world, 0.01), e.Status.Ok);
	_, _, _=e.trigger_events_drain(&s.world, parent[:]);
	n, _, status=e.trigger_part_events_drain(&s.world, parts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, 1);
	testing.expect_value(t, parts[0].pair.a.serial, committed.a.serial);
	testing.expect_value(t, parts[0].pair.b.serial, committed.b.serial);
	testing.expect_value(t, policy.configured, 0);
}

@(test)
triggers_hierarchy_sweep_matches_flat_angular_reference :: proc(t: ^testing.T)
{
	world: e.World;
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer e.world_destroy(&world);
	sphere: e.Shape_Handle;
	sphere, _ = e.shape_add(&world, e.sphere(0.5));
	inner: e.Shape_Handle;
	inner_status: e.Status;
	inner, inner_status = e.shape_import_compound(&world, []e.Compound_Child{e.compound_child(sphere, e.pose({1, 0, 0}))});
	testing.expect_value(t, inner_status, e.Status.Ok);
	nested: e.Shape_Handle;
	ns: e.Status;
	nested, ns = e.shape_import_big_compound(&world, []e.Compound_Child{e.compound_child(inner, e.pose({2, 0, 0}))});
	flat: e.Shape_Handle;
	fs: e.Status;
	flat, fs = e.shape_import_big_compound(&world, []e.Compound_Child{e.compound_child(sphere, e.pose({3, 0, 0}))});
	testing.expect_value(t, ns, e.Status.Ok);
	testing.expect_value(t, fs, e.Status.Ok);
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&world);
	for offset in ([2]util.Vector3{{}, {100, -200, 30}})
	{
		for omega in ([3]f32{-1, 0, 1})
		{
			for order in ([2]p.Sweep_Task_Route_Order{.Expected, .Flipped})
			{
				pose_a: e.Rigid_Pose = e.pose(offset);
				pose_b: e.Rigid_Pose = e.pose(util.vector3_add(offset, {3, 2, 0}));
				va: e.Body_Velocity = e.velocity({0, 1, 0}, {0, 0, omega});
				vb: e.Body_Velocity = e.velocity({0, -0.25, 0}, {0, 0, 0.2});
				base: p.Sweep_Result;
				base_status: p.Physics_Status;
				base, base_status = p.sweep_task_registry_test(&sim.sweep_tasks,
					sphere if order == .Flipped else flat, flat if order == .Flipped else sphere,
					pose_b if order == .Flipped else pose_a, pose_a if order == .Flipped else pose_b,
					vb if order == .Flipped else va, va if order == .Flipped else vb, 2, 0.00001, 0.00001, 128,
					&sim.shapes, &sim.collision_tasks, pool=sim.pool);
				result: p.Sweep_Result;
				status: p.Physics_Status;
				result, status = p.sweep_task_registry_test(&sim.sweep_tasks,
					sphere if order == .Flipped else nested, nested if order == .Flipped else sphere,
					pose_b if order == .Flipped else pose_a, pose_a if order == .Flipped else pose_b,
					vb if order == .Flipped else va, va if order == .Flipped else vb, 2, 0.00001, 0.00001, 128,
					&sim.shapes, &sim.collision_tasks, pool=sim.pool);
				testing.expect_value(t, base_status, e.Status.Ok);
				testing.expect_value(t, status, base_status);
				testing.expect_value(t, result.state, base.state);
				if result.state == .Hit
				{
					testing.expect(t, abs(result.t1-base.t1)<0.0001);
					testing.expect(t, util.vector3_distance(result.normal, base.normal)<0.001);
					testing.expect_value(t, result.child_a, base.child_a);
					testing.expect_value(t, result.child_b, base.child_b);
				}
			}
		}
	}
}
