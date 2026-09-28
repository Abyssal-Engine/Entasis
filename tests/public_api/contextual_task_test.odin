package public_api_tests

import "base:runtime"
import "core:mem"
import "core:sync"
import "core:testing"
import "core:thread"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Contextual_Task_State :: struct
{
	tag: i32,
	scalar_calls, wide_calls, wide_lanes, sweep_calls, child_calls: i32,
	scalar_status, wide_status, sweep_status, child_status: e.Status,
}

contextual_test_scalar :: proc "contextless" (
	user_context: rawptr, a, b: rawptr, pose_a, pose_b: p.Rigid_Pose,
	margin: f32, shapes: ^p.Shape_Registry,
) -> (p.Convex_Contact_Manifold, e.Status)
{
	state: ^Contextual_Task_State = (^Contextual_Task_State)(user_context);
	sync.atomic_add_explicit(&state.scalar_calls, 1, .Relaxed);
	if state.scalar_status != .Ok
	{
		return {}, state.scalar_status;
	}
	result: p.Convex_Contact_Manifold;
	status: p.Physics_Status;
	result, status = p.sphere_pair_test(a, b, pose_a, pose_b, margin, shapes);
	for i in 0..<int(result.count)
	{
		result.contacts[i].feature_id = state.tag;
	}
	return result, status;
}

contextual_test_wide :: proc "contextless" (
	user_context: rawptr, bundle: ^p.Collision_Convex_Wide_Bundle,
	shapes: ^p.Shape_Registry, result: ^p.Collision_Wide_Manifold_Result,
) -> e.Status
{
	_ = shapes;
	state: ^Contextual_Task_State = (^Contextual_Task_State)(user_context);
	sync.atomic_add_explicit(&state.wide_calls, 1, .Relaxed);
	if state.wide_status != .Ok
	{
		return state.wide_status;
	}
	if bundle == nil || result == nil || bundle.count < 1 || bundle.count > 8
	{
		return .Invalid_Description;
	}
	sync.atomic_add_explicit(&state.wide_lanes, i32(bundle.count), .Relaxed);
	a, b: p.Sphere_Wide;
	for lane in 0..<bundle.count
	{
		if bundle.shape_a[lane] == nil || bundle.shape_b[lane] == nil
		{
			return .Invalid_Description;
		}
		_ = p.sphere_wide_write_slot(&a, lane, (^p.Sphere)(bundle.shape_a[lane])^);
		_ = p.sphere_wide_write_slot(&b, lane, (^p.Sphere)(bundle.shape_b[lane])^);
	}
	wide: p.Convex_1_Contact_Manifold_Wide;
	status: p.Physics_Status;
	wide, status = p.sphere_pair_test_wide(a, b, bundle.speculative_margin, bundle.offset_b, bundle.count);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

contextual_test_sweep :: proc "contextless" (
	user_context: rawptr, a, b: rawptr, type_a, type_b: int,
	pose_a, pose_b: p.Rigid_Pose, va, vb: p.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32, iterations: int,
	shapes: ^p.Shape_Registry, tasks: ^p.Collision_Task_Registry,
	filter: p.Collision_Child_Filter_Proc, filter_context: rawptr,
) -> (p.Sweep_Result, e.Status)
{
	state: ^Contextual_Task_State = (^Contextual_Task_State)(user_context);
	sync.atomic_add_explicit(&state.sweep_calls, 1, .Relaxed);
	if state.sweep_status != .Ok
	{
		return {}, state.sweep_status;
	}
	if filter != nil && filter(filter_context, 19, 3, 7) == .Reject
	{
		return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
	}
	result: p.Sweep_Result;
	status: p.Physics_Status;
	result, status = custom_sphere_sweep(a, b, type_a, type_b, pose_a, pose_b, va, vb,
		maximum_t, minimum_progression, convergence_threshold, iterations, shapes, tasks, nil, nil);
	result.child_a = 3;
	result.child_b = 7;
	return result, status;
}

contextual_test_child :: proc "contextless" (
	user_context: rawptr, a, b: rawptr, type_a, type_b: int,
	pa, pb, la, lb: p.Rigid_Pose, va, vb: p.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32, iterations: int,
	shapes: ^p.Shape_Registry, tasks: ^p.Collision_Task_Registry,
) -> (p.Sweep_Result, e.Status)
{
	state: ^Contextual_Task_State = (^Contextual_Task_State)(user_context);
	sync.atomic_add_explicit(&state.child_calls, 1, .Relaxed);
	if state.child_status != .Ok
	{
		return {}, state.child_status;
	}
	result: p.Sweep_Result;
	status: p.Physics_Status;
	result, status = custom_sphere_sweep_child(a, b, type_a, type_b, pa, pb, la, lb, va, vb,
		maximum_t, minimum_progression, convergence_threshold, iterations, shapes, tasks);
	result.child_a=3;
	result.child_b=7;
	return result, status;
}

Contextual_Task_Fixture :: struct
{
	world: e.World,
	type_id: e.Shape_Type_ID,
	custom, sphere: e.Shape_Handle,
	state: Contextual_Task_State,
}

contextual_shape_only :: proc(t: ^testing.T, f: ^Contextual_Task_Fixture, description: e.World_Description) -> e.Status
{
	status: e.Status = e.world_init(&f.world, description);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return status;
	}
	var: e.Custom_Shape_Registration = e.custom_shape_registration(Custom_Sphere, .Convex, custom_sphere_bounds, custom_sphere_inertia, custom_sphere_ray, custom_sphere_support);
	id: e.Shape_Type_ID;
	id, status = e.custom_shape_register(&f.world, var);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return status;
	}
	f.type_id=id;
	f.custom, status = e.custom_shape_add(&f.world, id, &Custom_Sphere{radius=1});
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return status;
	}
	f.sphere, status = e.shape_add(&f.world, e.sphere(0.5));
	testing.expect_value(t, status, e.Status.Ok);
	return status;
}

contextual_bind_fixture :: proc(t: ^testing.T, f: ^Contextual_Task_Fixture) -> e.Status
{
	for type_b in ([2]e.Shape_Type_ID{f.type_id, e.SHAPE_TYPE_SPHERE})
	{
		collision: e.Contextual_Collision_Task_Registration = e.Contextual_Collision_Task_Registration{
			shape_type_a=f.type_id, shape_type_b=type_b, batch_size=16, pair_type=.Standard,
			user_context=&f.state, test=contextual_test_scalar, wide_test=contextual_test_wide,
		};
		status: e.Status;
		_, status = e.collision_task_register_contextual(&f.world, collision);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return status;
		}
		// the registered callback table must not borrow this descriptor
		collision.test=nil;
		collision.wide_test=nil;
		collision.user_context=nil;
		sweep: e.Contextual_Sweep_Task_Registration = e.Contextual_Sweep_Task_Registration{
			shape_type_a=f.type_id, shape_type_b=type_b, user_context=&f.state,
			test=contextual_test_sweep, child_test=contextual_test_child,
		};
		_, status = e.sweep_task_register_contextual(&f.world, sweep);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return status;
		}
		sweep.test=nil;
		sweep.child_test=nil;
		sweep.user_context=nil;
	}
	return .Ok;
}

@(test)
contextual_tasks_copy_descriptors_and_keep_independent_world_bindings :: proc(t: ^testing.T)
{
	fixtures: [2]Contextual_Task_Fixture;
	defer for &f in fixtures
	{
		_=e.world_destroy(&f.world);
	}
	for &f, i in fixtures
	{
		f.state.tag=i32(123+i);
		if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
		{
			return;
		}
	}
	testing.expect_value(t, fixtures[0].type_id, fixtures[1].type_id);
	for &f in fixtures
	{
		forward: e.Manifold_Result;
		status: e.Status;
		forward, status = e.collision_query(&f.world, f.custom, e.pose(), f.sphere, e.pose({0, 1, 0}));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, forward.convex.count, i32(1));
		testing.expect_value(t, forward.convex.contacts[0].feature_id, f.state.tag);
		reverse: e.Manifold_Result;
		reverse_status: e.Status;
		reverse, reverse_status = e.collision_query(&f.world, f.sphere, e.pose({0, 1, 0}), f.custom, e.pose());
		testing.expect_value(t, reverse_status, e.Status.Ok);
		testing.expect_value(t, reverse.convex.normal, u.vector3_negate(forward.convex.normal));
		testing.expect_value(t, reverse.convex.contacts[0].feature_id, f.state.tag);
		testing.expect_value(t, f.state.scalar_calls, i32(2));
	}
}

@(test)
contextual_tasks_reject_invalid_registration_without_consuming_routes :: proc(t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	if contextual_shape_only(t, &f, small_world_description()) != .Ok
	{
		return;
	}
	collision: e.Contextual_Collision_Task_Registration = e.Contextual_Collision_Task_Registration{
		shape_type_a=f.type_id, shape_type_b=e.SHAPE_TYPE_SPHERE, batch_size=16,
		user_context=&f.state, test=contextual_test_scalar, wide_test=contextual_test_wide,
	};
	for invalid in 0..<5
	{
		bad: e.Contextual_Collision_Task_Registration = collision;
		switch invalid
		{
			case 0: bad.test=nil;
			case 1: bad.wide_test=nil;
			case 2: bad.batch_size=65537;
			case 3: bad.shape_type_a=127;
			case 4: bad.shape_type_a=e.SHAPE_TYPE_SPHERE;
		}
		status: e.Status;
		_, status = e.collision_task_register_contextual(&f.world, bad);
		testing.expect(t, status != .Ok);
	}
	status: e.Status;
	_, status = e.collision_task_register_contextual(&f.world, collision);
	testing.expect_value(t, status, e.Status.Ok);
	_, status=e.collision_task_register_contextual(&f.world, collision);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	sweep: e.Contextual_Sweep_Task_Registration = e.Contextual_Sweep_Task_Registration{
		shape_type_a=f.type_id, shape_type_b=e.SHAPE_TYPE_SPHERE, user_context=&f.state,
		test=contextual_test_sweep, child_test=contextual_test_child,
	};
	bad: e.Contextual_Sweep_Task_Registration = sweep;
	bad.child_test=nil;
	_, status=e.sweep_task_register_contextual(&f.world, bad);
	testing.expect(t, status != .Ok);
	bad=sweep;
	bad.shape_type_a=e.SHAPE_TYPE_SPHERE;
	_, status=e.sweep_task_register_contextual(&f.world, bad);
	testing.expect(t, status != .Ok);
	_, status=e.sweep_task_register_contextual(&f.world, sweep);
	testing.expect_value(t, status, e.Status.Ok);
	_, status=e.sweep_task_register_contextual(&f.world, sweep);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	testing.expect_value(t, e.world_step(&f.world, 1.0/60.0), e.Status.Ok);
	collision.shape_type_b=f.type_id;
	_, status=e.collision_task_register_contextual(&f.world, collision);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	sweep.shape_type_b=f.type_id;
	_, status=e.sweep_task_register_contextual(&f.world, sweep);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
}

@(test)
contextual_tasks_wide_batches_step_without_scalarizing_callbacks :: proc(t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.capacity.bodies=32;
	description.capacity.pairs=128;
	description.capacity.broad_phase_candidates=128;
	description.capacity.pending_pairs_per_worker=128;
	description.capacity.constraints=128;
	if contextual_shape_only(t, &f, description) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	// ten disjoint custom/sphere pairs force a full bundle and a partial bundle
	for i in 0..<10
	{
		status: e.Status;
		_, status = e.static_add(&f.world, e.static_body(f.custom, e.pose({f32(i)*5, 0, 0})), .None);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
		inertia: e.Body_Inertia;
		inertia_status: e.Status;
		inertia, inertia_status = e.shape_inertia(e.sphere(0.5), 1);
		if !testing.expect_value(t, inertia_status, e.Status.Ok)
		{
			return;
		}
		body: e.Body_Description = e.body_dynamic(f.sphere, inertia, e.pose({f32(i)*5, 1, 0}));
		body.activity.sleep_threshold=-1;
		_, status=e.body_add(&f.world, body);
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
	}
	testing.expect_value(t, e.world_step(&f.world, 1.0/60.0), e.Status.Ok);
	testing.expect(t, f.state.wide_calls >= 2);
	testing.expect(t, f.state.wide_lanes >= 10);
	testing.expect(t, f.state.wide_calls < f.state.wide_lanes);
	testing.expect_value(t, f.state.scalar_calls, i32(0));
}

Contextual_Filter_State :: struct
{
	pair, a, b, calls: i32, decision: p.Collision_Testing_State,
}

contextual_test_filter :: proc "contextless" (user_context: rawptr, pair, a, b:i32) -> p.Collision_Testing_State
{
	f: ^Contextual_Filter_State = (^Contextual_Filter_State)(user_context);
	f.pair=pair;
	f.a=a;
	f.b=b;
	f.calls+=1;
	if f.decision == .Reject
	{
		return .Reject;
	}
	return .Allow;
}

@(test)
contextual_tasks_sweep_filter_context_and_flipped_children_are_distinct :: proc(t: ^testing.T)
{
	f:Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	sim: ^e.Simulation;
	sim, _ = e.world_borrow_simulation(&f.world);
	shapes: ^p.Shape_Registry = p.simulation_shape_registry(sim);
	filter:Contextual_Filter_State = {decision=.Allow};
	forward: p.Sweep_Result;
	status: p.Physics_Status;
	forward, status = p.sweep_task_registry_test(&sim.sweep_tasks, f.custom, f.sphere, e.pose({-4, 0, 0}), e.pose(),
		e.velocity({1, 0, 0}), {}, 10, 1e-4, 1e-4, 20, shapes, &sim.collision_tasks, contextual_test_filter, &filter);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, forward.state, p.Sweep_Hit_State.Hit);
	testing.expect_value(t, filter.pair, i32(19));
	testing.expect_value(t, filter.a, i32(3));
	testing.expect_value(t, filter.b, i32(7));
	reverse: p.Sweep_Result;
	reverse_status: p.Physics_Status;
	reverse, reverse_status = p.sweep_task_registry_test(&sim.sweep_tasks, f.sphere, f.custom, e.pose(), e.pose({-4, 0, 0}),
		{}, e.velocity({1, 0, 0}), 10, 1e-4, 1e-4, 20, shapes, &sim.collision_tasks, contextual_test_filter, &filter);
	testing.expect_value(t, reverse_status, e.Status.Ok);
	testing.expect_value(t, reverse.t0, forward.t0);
	testing.expect_value(t, filter.a, i32(7));
	testing.expect_value(t, filter.b, i32(3));
	testing.expect_value(t, reverse.child_a, i32(7));
	testing.expect_value(t, reverse.child_b, i32(3));
	testing.expect_value(t, reverse.normal, u.vector3_negate(forward.normal));
	testing.expect_value(t, f.state.sweep_calls, i32(2));
	testing.expect_value(t, filter.calls, i32(2));
	filter.decision=.Reject;
	miss: p.Sweep_Result;
	miss_status: p.Physics_Status;
	miss, miss_status = p.sweep_task_registry_test(&sim.sweep_tasks, f.sphere, f.custom, e.pose(), e.pose({-4, 0, 0}),
		{}, e.velocity({1, 0, 0}), 10, 1e-4, 1e-4, 20, shapes, &sim.collision_tasks, contextual_test_filter, &filter);
	testing.expect_value(t, miss_status, e.Status.Ok);
	testing.expect_value(t, miss.state, p.Sweep_Hit_State.Miss);
}

@(test)
contextual_tasks_compound_subpairs_and_registered_child_sweeps_dispatch :: proc(t: ^testing.T)
{
	f:Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	description: e.World_Description = small_world_description();
	description.capacity.collision_child_pairs=256;
	if contextual_shape_only(t, &f, description) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	children: [2]e.Compound_Child = [2]e.Compound_Child{e.compound_child(f.custom, e.pose({-0.4, 0, 0})), e.compound_child(f.custom, e.pose({0.4, 0, 0}))};
	compound: e.Shape_Handle;
	status: e.Status;
	compound, status = e.shape_import_compound(&f.world, children[:]);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	manifold: e.Manifold_Result;
	collision_status: e.Status;
	manifold, collision_status = e.collision_query(&f.world, f.sphere, e.pose({0, 1, 0}), compound, e.pose());
	testing.expect_value(t, collision_status, e.Status.Ok);
	present: p.Reference_State;
	_, present = p.manifold_deepest_contact(&manifold);
	testing.expect_value(t, present, p.Reference_State.Present);
	testing.expect(t, f.state.wide_calls>0);
	_, status=e.static_add(&f.world, e.static_body(compound, e.pose()), .None);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	hit: e.Sweep_Hit;
	sweep_status: e.Status;
	hit, sweep_status = e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, sweep_status, e.Status.Ok);
	testing.expect(t, hit.sweep.t0 < 4);
	testing.expect(t, f.state.child_calls>0);
}

@(test)
contextual_tasks_propagate_callback_errors_without_losing_bindings :: proc(t: ^testing.T)
{
	f:Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	f.state.scalar_status=.Capacity_Missing;
	status: e.Status;
	_, status = e.collision_query(&f.world, f.custom, e.pose(), f.sphere, e.pose({0, 1, 0}));
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	f.state.scalar_status=.Ok;
	_, status=e.collision_query(&f.world, f.custom, e.pose(), f.sphere, e.pose({0, 1, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	_, status=e.static_add(&f.world, e.static_body(f.custom, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	f.state.sweep_status=.Invalid_Description;
	_, status=e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	f.state.sweep_status=.Ok;
	_, status=e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, status, e.Status.Ok);
}

@(test)
contextual_tasks_bindings_return_to_the_world_allocator :: proc(t: ^testing.T)
{
	tracker:mem.Tracking_Allocator;
	mem.tracking_allocator_init(&tracker, context.allocator);
	defer mem.tracking_allocator_destroy(&tracker);
	description: e.World_Description = small_world_description();
	description.allocator=mem.tracking_allocator(&tracker);
	f:Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, description) != .Ok
	{
		_=e.world_destroy(&f.world);
		return;
	}
	initial_bytes: i64 = tracker.current_memory_allocated;
	initial_count: i64 = tracker.total_allocation_count;
	if contextual_bind_fixture(t, &f) != .Ok
	{
		_=e.world_destroy(&f.world);
		return;
	}
	// four immutable records, one per successful registration. no per-shape data
	testing.expect_value(t, tracker.total_allocation_count-initial_count, i64(4));
	testing.expect_value(t, tracker.current_memory_allocated-initial_bytes, i64(4*32));
	testing.expect_value(t, e.world_clear(&f.world), e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&f.world), e.Status.Ok);
	testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	testing.expect_value(t, len(tracker.allocation_map), 0);
}

Contextual_Query_Call :: struct
{
	ctx:e.Query_Context,
	a, b:e.Shape_Handle,
	failures:int,
}

contextual_query_thread :: proc(host:^thread.Thread)
{
	context = runtime.default_context();
	call: ^Contextual_Query_Call = (^Contextual_Query_Call)(host.data);
	for _ in 0..<40
	{
		status: e.Status;
		_, status = e.collision_query_with_context(&call.ctx, call.a, e.pose(), call.b, e.pose({0, 1, 0}));
		if status != .Ok
		{
			call.failures+=1;
		}
	}
}

@(test)
contextual_tasks_support_independent_concurrent_query_contexts :: proc(t:^testing.T)
{
	f:Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	calls:[4]Contextual_Query_Call;
	hosts:[4]^thread.Thread;
	for &call, i in calls
	{
		call.a=f.custom;
		call.b=f.sphere;
		if !testing.expect_value(t, e.query_context_init(&call.ctx, &f.world), e.Status.Ok)
		{
			return;
		}
		hosts[i]=thread.create(contextual_query_thread);
		hosts[i].data=&call;
	}
	for host in hosts
	{
		thread.start(host);
	}
	for host in hosts
	{
		thread.join(host);
		thread.destroy(host);
	}
	for &call in calls
	{
		testing.expect_value(t, call.failures, 0);
		testing.expect_value(t, e.query_context_destroy(&call.ctx), e.Status.Ok);
	}
	testing.expect_value(t, f.state.scalar_calls, i32(160));
}

Contextual_Allocation_Policy :: enum u8
{
	Allow,
	Reject,
}

Contextual_Fallible_Allocator :: struct
{
	backing: mem.Allocator,
	failure: Contextual_Allocation_Policy,
	failed: int,
}

contextual_fallible_allocator :: proc (
	data: rawptr, mode: mem.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int, location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, mem.Allocator_Error)
{
	owner: ^Contextual_Fallible_Allocator = (^Contextual_Fallible_Allocator)(data);
	if owner.failure == .Reject && (mode == .Alloc || mode == .Alloc_Non_Zeroed ||
		mode == .Resize || mode == .Resize_Non_Zeroed)
	{
		owner.failed += 1;
		return nil, .Out_Of_Memory;
	}
	return owner.backing.procedure(owner.backing.data, mode, size, alignment,
		old_memory, old_size, location);
}

@(test)
contextual_tasks_failed_record_allocation_keeps_routes_and_cleans_up :: proc(t: ^testing.T)
{
	tracker: mem.Tracking_Allocator;
	mem.tracking_allocator_init(&tracker, context.allocator);
	defer mem.tracking_allocator_destroy(&tracker);
	owner: Contextual_Fallible_Allocator = Contextual_Fallible_Allocator{backing=mem.tracking_allocator(&tracker)};
	description: e.World_Description = small_world_description();
	description.allocator = {procedure=contextual_fallible_allocator, data=&owner};
	f: Contextual_Task_Fixture;
	if contextual_shape_only(t, &f, description) != .Ok
	{
		_=e.world_destroy(&f.world);
		return;
	}
	initial_bytes: i64 = tracker.current_memory_allocated;
	collision: e.Contextual_Collision_Task_Registration = e.Contextual_Collision_Task_Registration{
		shape_type_a=f.type_id, shape_type_b=e.SHAPE_TYPE_SPHERE, batch_size=16,
		user_context=&f.state, test=contextual_test_scalar, wide_test=contextual_test_wide,
	};
	sweep: e.Contextual_Sweep_Task_Registration = e.Contextual_Sweep_Task_Registration{
		shape_type_a=f.type_id, shape_type_b=e.SHAPE_TYPE_SPHERE,
		user_context=&f.state, test=contextual_test_sweep, child_test=contextual_test_child,
	};
	owner.failure = .Reject;
	collision_status: e.Status;
	_, collision_status = e.collision_task_register_contextual(&f.world, collision);
	sweep_status: e.Status;
	_, sweep_status = e.sweep_task_register_contextual(&f.world, sweep);
	testing.expect_value(t, collision_status, e.Status.Capacity_Missing);
	testing.expect_value(t, sweep_status, e.Status.Capacity_Missing);
	testing.expect_value(t, owner.failed, 2);
	testing.expect_value(t, tracker.current_memory_allocated, initial_bytes);
	owner.failure = .Allow;
	_, collision_status = e.collision_task_register_contextual(&f.world, collision);
	_, sweep_status = e.sweep_task_register_contextual(&f.world, sweep);
	testing.expect_value(t, collision_status, e.Status.Ok);
	testing.expect_value(t, sweep_status, e.Status.Ok);
	// destruction must release these records even if all new allocations fail
	owner.failure = .Reject;
	testing.expect_value(t, e.world_destroy(&f.world), e.Status.Ok);
	testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	testing.expect_value(t, len(tracker.allocation_map), 0);
	testing.expect_value(t, owner.failed, 2);
}

@(test)
contextual_tasks_wide_and_child_failures_propagate :: proc(t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	defer _=e.world_destroy(&f.world);
	description: e.World_Description = small_world_description();
	description.capacity.collision_child_pairs=256;
	if contextual_shape_only(t, &f, description) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	children: [2]e.Compound_Child = [2]e.Compound_Child{
		e.compound_child(f.custom, e.pose({-0.4, 0, 0})),
		e.compound_child(f.custom, e.pose({0.4, 0, 0})),
	};
	compound: e.Shape_Handle;
	status: e.Status;
	compound, status = e.shape_import_compound(&f.world, children[:]);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	f.state.wide_status=.Capacity_Missing;
	_, status=e.collision_query(&f.world, f.sphere, e.pose({0, 1, 0}), compound, e.pose());
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	f.state.wide_status=.Ok;
	_, status=e.collision_query(&f.world, f.sphere, e.pose({0, 1, 0}), compound, e.pose());
	testing.expect_value(t, status, e.Status.Ok);
	_, status=e.static_add(&f.world, e.static_body(compound, e.pose()), .None);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	f.state.child_status=.Capacity_Missing;
	_, status=e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	f.state.child_status=.Ok;
	_, status=e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, status, e.Status.Ok);
}

@(test)
contextual_tasks_preserve_native_record_layouts :: proc(t: ^testing.T)
{
	// these are the original x86-64-v3 task layouts, not public C ABI types
	collision: p.Collision_Task;
	sweep: p.Sweep_Task;
	testing.expect_value(t, size_of(p.Collision_Task), 40);
	testing.expect_value(t, align_of(p.Collision_Task), 8);
	testing.expect_value(t, uintptr(&collision.convex_test)-uintptr(&collision), uintptr(16));
	testing.expect_value(t, uintptr(&collision.convex_wide_test)-uintptr(&collision), uintptr(24));
	testing.expect_value(t, uintptr(&collision.convex_wide_test_into)-uintptr(&collision), uintptr(32));
	testing.expect_value(t, size_of(p.Sweep_Task), 32);
	testing.expect_value(t, align_of(p.Sweep_Task), 8);
	testing.expect_value(t, uintptr(&sweep.test)-uintptr(&sweep), uintptr(8));
	testing.expect_value(t, uintptr(&sweep.child_test)-uintptr(&sweep), uintptr(16));
	testing.expect_value(t, uintptr(&sweep.kind)-uintptr(&sweep), uintptr(24));
}

@(test)
contextual_sweep_any_preserves_pre_hit_capacity_failures :: proc(t: ^testing.T)
{
	f: Contextual_Task_Fixture;
	defer _ = e.world_destroy(&f.world);
	if contextual_shape_only(t, &f, small_world_description()) != .Ok || contextual_bind_fixture(t, &f) != .Ok
	{
		return;
	}
	status: e.Status;
	_, status = e.static_add(&f.world, e.static_body(f.custom, e.pose()), .None);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	query: e.Query_Context;
	if !testing.expect_value(t, e.query_context_init(&query, &f.world), e.Status.Ok)
	{
		return;
	}
	defer _ = e.query_context_destroy(&query);
	f.state.sweep_status = .Capacity_Missing;
	found: bool;
	direct_status: e.Status;
	found, direct_status = e.sweep_any(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, direct_status, e.Status.Capacity_Missing);
	testing.expect_value(t, found, false);
	context_found: e.Query_Hit_State;
	context_status: e.Status;
	context_found, context_status = e.sweep_any_with_context(&query, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, context_status, e.Status.Capacity_Missing);
	testing.expect_value(t, context_found, e.Query_Hit_State.Miss);
	f.state.sweep_status = .Ok;
	found, direct_status = e.sweep_any(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, direct_status, e.Status.Ok);
	testing.expect_value(t, found, true);
	context_found, context_status = e.sweep_any_with_context(&query, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
	testing.expect_value(t, context_status, e.Status.Ok);
	testing.expect_value(t, context_found, e.Query_Hit_State.Hit);
}
