package narrowphase_integration_tests

import "base:runtime"
import "core:testing"
import "core:mem"
import "core:simd"
import "core:sync"
import "core:thread"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Narrow_Callback_State :: struct
{
	allow_count:          i32,
	configure_count:      i32,
	allow_child_count:    i32,
	configure_child_count: i32,
	margin_before:        f32,
	margin_override:      f32,
	preserve_margin:      physics.Reference_State,
	rejected_child:       i32,
	allow_result:         physics.Collision_Testing_State,
	configure_result: physics.Collision_Testing_State,
	body_pair_allow_a:     physics.Collidable_Reference,
	body_pair_allow_b:     physics.Collidable_Reference,
	body_pair_configure_a: physics.Collidable_Reference,
	body_pair_configure_b: physics.Collidable_Reference,
	body_pair_allow_count: i32,
	body_pair_configure_count: i32,
}

Contact_View_Capture :: struct
{
	view:       physics.Contact_Constraint_Data_View,
	call_count: i32,
}

Phase3_Direct_Error_State :: struct
{
	simulation:       ^physics.Simulation,
	dispatcher:       ^util.Thread_Dispatcher_Boundary,
	entered:          sync.Futex,
	release:          sync.Futex,
	allow_calls:      [2]i32,
	published_observed: physics.Reference_State,
	failure_enabled:  physics.Reference_State,
	status:           physics.Physics_Status,
}

Phase3_Test_Dispatch_State :: struct
{
	pools: [2]^util.Buffer_Pool,
}

Phase3_Test_Worker_Entry :: struct
{
	worker:       util.Dispatcher_Worker_Proc,
	dispatcher:   ^util.Thread_Dispatcher_Boundary,
	worker_index: int,
}

Phase3_World_Outcome :: struct
{
	status:           physics.Physics_Status,
	route:            physics.Narrow_Phase_Collision_Route,
	pair_count:       int,
	constraint_count: i32,
	handle_state:     physics.Reference_State,
}

phase3_wait_for_futex :: proc "contextless" (value: ^sync.Futex, expected: sync.Futex)
{
	for
	{
		observed := sync.atomic_load_explicit(value, .Acquire);
		if observed == expected
		{
			return;
		}
		sync.futex_wait(value, u32(observed));
	}
}

phase3_direct_error_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int,
	a, b: physics.Collidable_Reference, speculative_margin: ^f32,
) -> physics.Collision_Testing_State
{
	_, _ = a, b;
	state := (^Phase3_Direct_Error_State)(user_context);
	if state.failure_enabled != .Present
	{
		return .Allow;
	}
	call_index := sync.atomic_add_explicit(
		&state.allow_calls[worker_index], i32(1), .Acq_Rel,
	);
	if call_index != 1
	{
		return .Allow;
	}
	if state.simulation.narrow_phase.batchers[worker_index].pair_count == 1
	{
		state.published_observed = .Present;
	}
	sync.atomic_store_explicit(&state.entered, sync.Futex(1), .Release);
	sync.futex_broadcast(&state.entered);
	phase3_wait_for_futex(&state.release, sync.Futex(1));
	speculative_margin^ = -1;
	return .Allow;
}

phase3_test_worker_entry :: proc(host_thread: ^thread.Thread)
{
	context = runtime.default_context();
	entry := (^Phase3_Test_Worker_Entry)(host_thread.data);
	entry.worker(entry.worker_index, entry.dispatcher);
}

phase3_test_dispatch :: proc "contextless" (
	dispatcher: ^util.Thread_Dispatcher_Boundary,
	worker: util.Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> util.Threading_Status
{
	context = runtime.default_context();
	if dispatcher == nil || worker == nil || maximum_worker_count <= 0 ||
		maximum_worker_count > 2
	{
		return .Invalid_Argument;
	}
	dispatcher.unmanaged_context = unmanaged_context;
	if maximum_worker_count == 1
	{
		worker(0, dispatcher);
		return .Ok;
	}
	entry := Phase3_Test_Worker_Entry{
		worker=worker, dispatcher=dispatcher, worker_index=1,
	};
	worker_thread := thread.create(phase3_test_worker_entry);
	if worker_thread == nil
	{
		return .Capacity_Missing;
	}
	worker_thread.data = &entry;
	thread.start(worker_thread);
	worker(0, dispatcher);
	thread.join(worker_thread);
	thread.destroy(worker_thread);
	return .Ok;
}

phase3_execute_controlled :: proc(host_thread: ^thread.Thread)
{
	context = runtime.default_context();
	state := (^Phase3_Direct_Error_State)(host_thread.data);
	state.status = physics.simulation_timestep(
		state.simulation, 1.0 / 60.0, state.dispatcher,
	);
}

capture_contact_view :: proc "contextless" (
	user_context: rawptr,
	view: ^physics.Contact_Constraint_Data_View,
)
{
	capture := (^Contact_View_Capture)(user_context);
	capture.view = view^;
	capture.call_count += 1;
}

expect_contact_view_impulses_match_solver :: proc(
	t: ^testing.T, simulation: ^physics.Simulation,
	handle: physics.Constraint_Handle,
	view: ^physics.Contact_Constraint_Data_View,
)
{
	location, resolve_status := physics.solver_resolve(
		&simulation.solver, handle,
	);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	batch := &simulation.solver.active_set.batches.memory[location.batch_index];
	type_batch := &batch.type_batches.memory[
		batch.type_id_to_batch_index[location.type_id]
	];
	impulse_vectors := ([^]util.F32x8)(
		physics.type_batch_impulse_bundle(
		type_batch, int(location.index_in_type_batch),
	),
	);
	lane := int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT;
	for impulse_index in 0 ..< int(view.impulse_count)
	{
		testing.expect_value(
			t, view.impulses[impulse_index],
			simd.extract(impulse_vectors[impulse_index], lane),
		);
	}
}

@(private)
test_simulation: physics.Simulation;

@(private)
test_fused_simulation: physics.Simulation;

@(private)
test_phase3_direct_simulation: physics.Simulation;
test_phase3_world_pools:       [2]util.Buffer_Pool;
test_phase3_world_simulations: [2]physics.Simulation;

@(private)
test_fused_flush_tasks: physics.Collision_Task_Registry;

test_pool_prepare :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 256), util.Memory_Status.Ok);
	for power in 0 ..= 22
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 524288, power), util.Memory_Status.Ok);
	}
}

test_narrow_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> physics.Physics_Status
{
	_ = simulation;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

test_narrow_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference, speculative_margin: ^f32,
) -> physics.Collision_Testing_State
{
	_ = worker_index;
	state := (^Narrow_Callback_State)(user_context);
	state.allow_count += 1;
	if physics.collidable_reference_mobility(a) != .Static &&
		physics.collidable_reference_mobility(b) != .Static
	{
		state.body_pair_allow_a = a;
		state.body_pair_allow_b = b;
		state.body_pair_allow_count += 1;
	}
	state.margin_before = speculative_margin^;
	if state.preserve_margin != .Present
	{
		speculative_margin^ = state.margin_override;
	}
	return state.allow_result;
}

test_narrow_allow_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference, child_a, child_b: int,
) -> physics.Collision_Testing_State
{
	_, _, _ = worker_index, a, b;
	state := (^Narrow_Callback_State)(user_context);
	state.allow_child_count += 1;
	if child_b == int(state.rejected_child)
	{
		return .Reject;
	}
	return .Allow;
}

test_narrow_configure :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	manifold: ^physics.Manifold_Result, material: ^physics.Contact_Material_Properties,
) -> physics.Collision_Testing_State
{
	_, _ = worker_index, manifold;
	state := (^Narrow_Callback_State)(user_context);
	state.configure_count += 1;
	if physics.collidable_reference_mobility(a) != .Static &&
		physics.collidable_reference_mobility(b) != .Static
	{
		state.body_pair_configure_a = a;
		state.body_pair_configure_b = b;
		state.body_pair_configure_count += 1;
	}
	material^ = {
		friction_coefficient=0.7,
		spring_settings={angular_frequency=30, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	return state.configure_result;
}

test_narrow_configure_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	child_a, child_b: int, manifold: ^physics.Convex_Contact_Manifold,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _ = worker_index, a, b, child_a, child_b;
	state := (^Narrow_Callback_State)(user_context);
	state.configure_child_count += 1;
	for contact_index in 0 ..< int(manifold.count)
	{
		manifold.contacts[contact_index].feature_id = 77;
	}
	return .Allow;
}

test_narrow_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}

test_select_custom_constraint :: proc "contextless" (
	user_context: rawptr, a, b: physics.Collidable_Reference,
	manifold: ^physics.Manifold_Result, default_type_id: i32,
) -> i32
{
	_, _, _, _, _ = user_context, a, b, manifold, default_type_id;
	return physics.FIRST_CALLER_CONSTRAINT_TYPE_ID;
}

test_custom_contact_validate :: proc "contextless" (type_id: i32, description: rawptr) -> physics.Physics_Status
{
	if type_id != physics.FIRST_CALLER_CONSTRAINT_TYPE_ID || description == nil
	{
		return .Invalid_Description;
	}
	return physics.constraint_description_validate((^physics.Contact_1)(description));
}

test_custom_contact_build :: proc "contextless" (
	manifold: ^physics.Manifold_Result, body_count: int, material: physics.Contact_Material_Properties,
	target: rawptr, feature_ids: ^[physics.MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
) -> physics.Physics_Status
{
	return physics.contact_constraint_builtin_build(manifold, body_count, material, target, feature_ids);
}

test_simulation_description :: proc(
	pool: ^util.Buffer_Pool, callbacks: physics.Narrow_Phase_Callbacks,
) -> physics.Simulation_Create_Description
{
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=32, statics=32, inactive_body_sets=8, shapes_per_type=8,
		constraints=64, constraint_batches=8, initial_constraints_per_type_batch=8,
		minimum_constraints_per_body=8, broad_phase_candidates=128, pairs=128,
		inactive_pairs=64, pending_pairs_per_worker=64, workers=1,
	};
	description.narrow_callbacks = callbacks;
	description.use_default_narrow = .Missing;
	description.default_pose_context.gravity = {};
	description.profiling = .Enabled;
	return description;
}

expect_narrow_pair_results_equal :: proc(
	t: ^testing.T,
	actual, expected: ^physics.Narrow_Phase_Pair_Result,
)
{
	testing.expect_value(t, actual.state, expected.state);
	if actual.state != .Accepted
	{
		return;
	}
	testing.expect_value(t, actual.material, expected.material);
	testing.expect_value(t, actual.manifold.kind, expected.manifold.kind);
	if actual.manifold.kind == .Convex
	{
		testing.expect_value(t, actual.manifold.convex, expected.manifold.convex);
	}
	else
	{
		testing.expect_value(t, actual.manifold.nonconvex, expected.manifold.nonconvex);
	}
}

verify_phase3_direct_route_completion :: proc(t: ^testing.T)
{
	direct_pool: util.Buffer_Pool;
	test_pool_prepare(t, &direct_pool);
	defer util.buffer_pool_dispose(&direct_pool);
	test_phase3_direct_simulation = {};
	direct_simulation := &test_phase3_direct_simulation;
	direct_description := test_simulation_description(
		&direct_pool, physics.narrow_phase_default_callbacks(),
	);
	direct_description.use_default_narrow = .Present;
	testing.expect_value(
		t, physics.simulation_create(direct_simulation, &direct_description).status,
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_bind_default_stored_completion(
		&direct_simulation.narrow_phase,
		{
			friction_coefficient=1,
			spring_settings=physics.spring_settings_create(30, 1),
			maximum_recovery_velocity=2,
		},
	),
		physics.Physics_Status.Ok,
	);
	direct_box := physics.Box{0.5, 0.5, 0.5};
	direct_box_shape, direct_box_status := physics.shape_registry_add(
		&direct_simulation.shapes, physics.BOX_TYPE_ID, &direct_box,
	);
	testing.expect_value(t, direct_box_status, physics.Physics_Status.Ok);
	direct_body_description := test_body_description(direct_box_shape, {0, 0.9, 0});
	direct_body_description.activity.sleep_threshold = -1;
	direct_body, direct_body_status := physics.simulation_add_body(
		direct_simulation, &direct_body_description,
	);
	testing.expect_value(t, direct_body_status, physics.Physics_Status.Ok);
	direct_static, direct_static_status := physics.simulation_add_static(
		direct_simulation,
		&physics.Static_Description{
			pose={orientation=util.quaternion_identity()}, shape=direct_box_shape,
		},
	);
	testing.expect_value(t, direct_static_status, physics.Physics_Status.Ok);
	direct_simulation.narrow_phase.results.memory[0].state = .Skipped;
	testing.expect_value(
		t, physics.simulation_timestep(direct_simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		&direct_simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Box_Direct,
	);
	testing.expect_value(
		t, direct_simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Skipped,
	);
	testing.expect_value(t, direct_simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, direct_simulation.solver.active_set.constraint_count, i32(1));
	// prepare traversal again after registration changes eligibility for the homogeneous route
	transition_sphere: physics.Sphere = {radius=0.125};
	transition_shape, transition_status := physics.shape_registry_add(
		&direct_simulation.shapes, physics.SPHERE_TYPE_ID, &transition_sphere,
	);
	testing.expect_value(t, transition_status, physics.Physics_Status.Ok);
	transition_routes: [2]physics.Narrow_Phase_Collision_Route = {.Box_Sphere_Direct, .Box_Direct};
	for expected_route, transition_index in transition_routes
	{
		if transition_index == 1
		{
			testing.expect_value(t, physics.shape_registry_remove(&direct_simulation.shapes, transition_shape), physics.Physics_Status.Ok);
		}
		actual_route: physics.Narrow_Phase_Collision_Route = physics.narrow_phase_collision_route_state(&direct_simulation.narrow_phase, 1);
		if actual_route == .Predecessor
		{
			actual_route = physics.narrow_phase_box_sphere_route_state(&direct_simulation.narrow_phase, 1);
		}
		testing.expect_value(t, actual_route, expected_route);
		testing.expect_value(t, physics.simulation_apply_body_description(direct_simulation, direct_body, &direct_body_description), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.simulation_timestep(direct_simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
		testing.expect_value(t, direct_simulation.narrow_phase.pair_cache.mapping.count, 1);
		testing.expect_value(t, direct_simulation.solver.active_set.constraint_count, i32(1));
	}
	direct_capsule := physics.Capsule{radius=0.25, half_length=0.25};
	direct_capsule_shape, direct_capsule_status := physics.shape_registry_add(
		&direct_simulation.shapes, physics.CAPSULE_TYPE_ID, &direct_capsule,
	);
	testing.expect_value(t, direct_capsule_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		&direct_simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	testing.expect_value(
		t, physics.simulation_apply_body_description(
		direct_simulation, direct_body, &direct_body_description,
	),
		physics.Physics_Status.Ok,
	);
	direct_simulation.narrow_phase.results.memory[0].state = .Rejected;
	testing.expect_value(
		t, physics.simulation_timestep(direct_simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, direct_simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Skipped,
	);
	testing.expect_value(
		t, physics.shape_registry_remove(
		&direct_simulation.shapes, direct_capsule_shape,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		&direct_simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Box_Direct,
	);
	testing.expect_value(
		t, physics.simulation_apply_body_description(
		direct_simulation, direct_body, &direct_body_description,
	),
		physics.Physics_Status.Ok,
	);
	direct_simulation.narrow_phase.results.memory[0].state = .Rejected;
	testing.expect_value(
		t, physics.simulation_timestep(direct_simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, direct_simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Rejected,
	);
	testing.expect_value(t, direct_simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, direct_simulation.solver.active_set.constraint_count, i32(1));
	direct_narrow := &direct_simulation.narrow_phase;
	direct_records_storage := ([^]physics.Narrow_Phase_Convex_Direct_Record)(
		rawptr(direct_narrow.batchers[0].pairs.memory),
	);
	direct_static_reference, direct_static_reference_status :=
		physics.collidable_reference_static(direct_static);
	testing.expect_value(
		t, direct_static_reference_status, physics.Physics_Status.Ok,
	);
	leaf_pose_a := physics.rigid_pose_identity();
	leaf_pose_b := physics.rigid_pose_identity();
	leaf_pose_b.position = {0, 0.9, 0};
	leaf_manifold, leaf_manifold_status := physics.box_pair_test_source(
		direct_box, direct_box, leaf_pose_a, leaf_pose_b, 0.25,
	);
	testing.expect_value(t, leaf_manifold_status, physics.Physics_Status.Ok);
	testing.expect(t, leaf_manifold.count > 0);
	leaf_counts := [4]int{8, 9, 32, 33};
	for leaf_count in leaf_counts
	{
		testing.expect_value(
			t, physics.narrow_phase_reset_transaction_streams(direct_narrow, 1),
			physics.Physics_Status.Ok,
		);
		for leaf_index in 0 ..< leaf_count
		{
			dynamic_reference, dynamic_reference_status :=
				physics.collidable_reference_create(.Dynamic, 100 + leaf_index);
			testing.expect_value(
				t, dynamic_reference_status, physics.Physics_Status.Ok,
			);
			direct_narrow.candidates.memory[leaf_index] = {
				dynamic_reference, direct_static_reference,
			};
			direct_narrow.continuations.memory[leaf_index] = {};
			direct_narrow.results.memory[leaf_index] = {
				manifold={kind=.Nonconvex, nonconvex={count=4}},
				material={friction_coefficient=123},
				state=.Accepted,
			};
			direct_records_storage[leaf_index] = {
				pair_id=i32(leaf_index),
				speculative_margin=0.25,
				shape_data_a=&direct_box,
				shape_data_b=&direct_box,
				pose_a=leaf_pose_a,
				pose_b=leaf_pose_b,
			};
		}
		direct_narrow.batchers[0].pair_count = leaf_count;
		direct_narrow.transaction_streams[0].traversal_candidate_count = i32(leaf_count);
		testing.expect_value(
			t, physics.narrow_phase_flush_worker_chunk_convex_direct(
			direct_narrow, 0, leaf_count, 0, leaf_count, false,
		),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(t, direct_narrow.batchers[0].pair_count, 0);
		testing.expect_value(
			t, direct_narrow.transaction_streams[0].total_count, i32(leaf_count),
		);
		deferred_records := ([^]physics.Narrow_Phase_Convex_Deferred_Record)(
			rawptr(direct_narrow.results.memory),
		);
		for deferred_index in 0 ..< leaf_count
		{
			expected_record := physics.Narrow_Phase_Convex_Deferred_Record{
				pair=physics.collidable_pair_create(
					direct_narrow.candidates.memory[deferred_index].a,
					direct_narrow.candidates.memory[deferred_index].b,
				),
				mapping_index=-1,
				manifold={kind=.Convex, convex=leaf_manifold},
			};
			actual_bytes := transmute([size_of(physics.Narrow_Phase_Convex_Deferred_Record)]u8)deferred_records[deferred_index];
			expected_bytes := transmute([size_of(physics.Narrow_Phase_Convex_Deferred_Record)]u8)expected_record;
			testing.expect_value(t, actual_bytes, expected_bytes);
		}
	}
	compaction_expected_counts := [3]int{0, 4, 8};
	leaf_pose_separated := leaf_pose_b;
	leaf_pose_separated.position.y = 4;
	for compaction_case in 0 ..< len(compaction_expected_counts)
	{
		testing.expect_value(
			t, physics.narrow_phase_reset_transaction_streams(direct_narrow, 1),
			physics.Physics_Status.Ok,
		);
		for leaf_index in 0 ..< 8
		{
			dynamic_reference, dynamic_reference_status :=
				physics.collidable_reference_create(.Dynamic, 300 + leaf_index);
			testing.expect_value(
				t, dynamic_reference_status, physics.Physics_Status.Ok,
			);
			direct_narrow.candidates.memory[leaf_index] = {
				dynamic_reference, direct_static_reference,
			};
			direct_narrow.continuations.memory[leaf_index] = {};
			direct_narrow.results.memory[leaf_index] = {
				manifold={kind=.Nonconvex, nonconvex={count=4}},
				material={friction_coefficient=123},
				state=.Accepted,
			};
			deferred_lane := compaction_case == 2 ||
				(compaction_case == 1 && leaf_index % 2 == 0);
			pose_b := leaf_pose_separated;
			if deferred_lane
			{
				pose_b = leaf_pose_b;
			}
			direct_records_storage[leaf_index] = {
				pair_id=i32(leaf_index),
				speculative_margin=0.25,
				shape_data_a=&direct_box,
				shape_data_b=&direct_box,
				pose_a=leaf_pose_a,
				pose_b=pose_b,
			};
		}
		direct_narrow.batchers[0].pair_count = 8;
		direct_narrow.transaction_streams[0].traversal_candidate_count = 8;
		testing.expect_value(
			t, physics.narrow_phase_flush_worker_chunk_convex_direct(
			direct_narrow, 0, 8, 0, 8, false,
		),
			physics.Physics_Status.Ok,
		);
		expected_count := compaction_expected_counts[compaction_case];
		testing.expect_value(t, direct_narrow.batchers[0].pair_count, 0);
		testing.expect_value(
			t, direct_narrow.transaction_streams[0].total_count,
			i32(expected_count),
		);
		deferred_records := ([^]physics.Narrow_Phase_Convex_Deferred_Record)(
			rawptr(direct_narrow.results.memory),
		);
		for deferred_index in 0 ..< expected_count
		{
			candidate_index := deferred_index;
			if compaction_case == 1
			{
				candidate_index *= 2;
			}
			expected_record := physics.Narrow_Phase_Convex_Deferred_Record{
				pair=physics.collidable_pair_create(
					direct_narrow.candidates.memory[candidate_index].a,
					direct_narrow.candidates.memory[candidate_index].b,
				),
				mapping_index=-1,
				manifold={kind=.Convex, convex=leaf_manifold},
			};
			actual_bytes := transmute([size_of(physics.Narrow_Phase_Convex_Deferred_Record)]u8)deferred_records[deferred_index];
			expected_bytes := transmute([size_of(physics.Narrow_Phase_Convex_Deferred_Record)]u8)expected_record;
			testing.expect_value(t, actual_bytes, expected_bytes);
		}
	}
	invalid_effect_mapping_count := direct_narrow.pair_cache.mapping.count;
	invalid_effect_constraint_count := direct_simulation.solver.active_set.constraint_count;
	invalid_statuses := [5]physics.Physics_Status{
		.Invalid_Argument, .Invalid_Argument, .Invalid_Description,
		.Invalid_Description, .Invalid_Description,
	};
	for invalid_case in 0 ..< len(invalid_statuses)
	{
		testing.expect_value(
			t, physics.narrow_phase_reset_transaction_streams(direct_narrow, 1),
			physics.Physics_Status.Ok,
		);
		dynamic_reference, dynamic_reference_status :=
			physics.collidable_reference_create(.Dynamic, 200 + invalid_case);
		testing.expect_value(t, dynamic_reference_status, physics.Physics_Status.Ok);
		direct_narrow.candidates.memory[0] = {
			dynamic_reference, direct_static_reference,
		};
		direct_records_storage[0] = {
			pair_id=0,
			speculative_margin=0.25,
			shape_data_a=&direct_box,
			shape_data_b=&direct_box,
			pose_a=leaf_pose_a,
			pose_b=leaf_pose_b,
		};
		candidate_count := 1;
		candidate_start := 0;
		direct_count := 1;
		switch invalid_case
		{
			case 0:
				direct_count = 2;
			case 1:
				candidate_start = int(direct_narrow.candidates.length);
			case 2:
				direct_records_storage[0].pair_id = 1;
			case 3:
				direct_records_storage[0].shape_data_a = nil;
			case 4:
				direct_records_storage[0].speculative_margin = -1;
		}
		direct_narrow.batchers[0].pair_count = direct_count;
		direct_narrow.transaction_streams[0].traversal_candidate_count = i32(candidate_count);
		testing.expect_value(
			t, physics.narrow_phase_flush_worker_chunk_convex_direct(
			direct_narrow, 0, candidate_count, candidate_start, direct_count, false,
		),
			invalid_statuses[invalid_case],
		);
		testing.expect_value(t, direct_narrow.batchers[0].pair_count, 0);
		testing.expect_value(
			t, direct_narrow.transaction_streams[0].total_count, i32(0),
		);
		testing.expect_value(
			t, direct_narrow.pair_cache.mapping.count, invalid_effect_mapping_count,
		);
		testing.expect_value(
			t, direct_simulation.solver.active_set.constraint_count,
			invalid_effect_constraint_count,
		);
	}
	reserve_body_description := test_body_description(
		direct_box_shape, {8, 8, 8},
	);
	reserve_body_description.activity.sleep_threshold = -1;
	reserve_body, reserve_body_status := physics.simulation_add_body(
		direct_simulation, &reserve_body_description,
	);
	testing.expect_value(t, reserve_body_status, physics.Physics_Status.Ok);
	direct_body_reference, direct_body_reference_status :=
		physics.collidable_reference_create(.Dynamic, int(direct_body.value));
	reserve_body_reference, reserve_body_reference_status :=
		physics.collidable_reference_create(.Dynamic, int(reserve_body.value));
	testing.expect_value(
		t, direct_body_reference_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, reserve_body_reference_status, physics.Physics_Status.Ok,
	);
	existing_pair := physics.collidable_pair_create(
		direct_body_reference, direct_static_reference,
	);
	existing_mapping_index := physics.pair_cache_index_of(
		&direct_narrow.pair_cache, existing_pair,
	);
	testing.expect(t, existing_mapping_index >= 0);
	testing.expect_value(
		t, physics.narrow_phase_reset_transaction_streams(direct_narrow, 1),
		physics.Physics_Status.Ok,
	);
	direct_narrow.candidates.memory[0] = {
		direct_body_reference, direct_static_reference,
	};
	direct_narrow.candidates.memory[1] = {
		reserve_body_reference, direct_static_reference,
	};
	direct_narrow.continuations.memory[0] = {};
	direct_narrow.continuations.memory[1] = {};
	reserve_direct_pose := direct_body_description.pose;
	reserve_static_pose := physics.rigid_pose_identity();
	for record_index in 0 ..< 2
	{
		direct_records_storage[record_index] = {
			pair_id=i32(record_index),
			speculative_margin=0.25,
			shape_data_a=&direct_box,
			shape_data_b=&direct_box,
			pose_a=reserve_direct_pose,
			pose_b=reserve_static_pose,
		};
	}
	direct_narrow.batchers[0].pair_count = 2;
	direct_narrow.transaction_streams[0].traversal_candidate_count = 2;
	expected_freshness := direct_narrow.pair_cache.freshness_generation;
	direct_narrow.pair_cache.pair_freshness.memory[existing_mapping_index] =
		expected_freshness ~ u8(0xff);
	reserve_mapping_count := direct_narrow.pair_cache.mapping.count;
	reserve_constraint_count := direct_simulation.solver.active_set.constraint_count;
	saved_transaction_capacity := direct_narrow.current_transaction_capacity;
	direct_narrow.current_transaction_capacity = 0;
	testing.expect_value(
		t, physics.narrow_phase_flush_worker_chunk_convex_direct(
		direct_narrow, 0, 2, 0, 2, false,
	),
		physics.Physics_Status.Capacity_Missing,
	);
	testing.expect_value(t, direct_narrow.batchers[0].pair_count, 0);
	testing.expect_value(
		t, direct_narrow.pair_cache.pair_freshness.memory[existing_mapping_index],
		expected_freshness,
	);
	testing.expect_value(
		t, direct_narrow.pair_cache.mapping.count, reserve_mapping_count,
	);
	testing.expect_value(
		t, direct_simulation.solver.active_set.constraint_count,
		reserve_constraint_count,
	);
	testing.expect_value(
		t, direct_narrow.transaction_streams[0].total_count, i32(0),
	);
	direct_narrow.current_transaction_capacity = saved_transaction_capacity;
	direct_narrow.batchers[0].pair_count = 2;
	testing.expect_value(
		t, physics.narrow_phase_flush_worker_chunk_convex_direct(
		direct_narrow, 0, 2, 0, 2, false,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, direct_narrow.transaction_streams[0].total_count, i32(1),
	);
	testing.expect_value(
		t, physics.narrow_phase_reset_transaction_streams(direct_narrow, 1),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_destroy(direct_simulation), physics.Physics_Status.Ok,
	);
}

verify_stored_and_materialized_completion_basics :: proc(t: ^testing.T)
{
	callback_state := Narrow_Callback_State{
		allow_result=.Allow,
		configure_result=.Allow,
	};
	callbacks := physics.Narrow_Phase_Callbacks{
		initialize=test_narrow_initialize,
		allow=test_narrow_allow,
		allow_child=test_narrow_allow_child,
		configure=test_narrow_configure,
		configure_child=test_narrow_configure_child,
		dispose=test_narrow_dispose,
		user_context=&callback_state,
	};
	material := physics.Contact_Material_Properties{
		friction_coefficient=0.7,
		spring_settings={angular_frequency=30, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	materialized_manifolds: [4]physics.Manifold_Result;
	materialized_manifolds[0] = {
		kind=.Convex,
		convex={
			offset_b={0.5, 0.25, -0.75},
			count=2,
			normal={0, 1, 0},
			contacts={
				{offset={1, 2, 3}, depth=0.25, feature_id=11},
				{offset={-2, 1, 0.5}, depth=-0.125, feature_id=12},
				{},
				{},
			},
		},
	};
	materialized_manifolds[1] = {
		kind=.Nonconvex,
		nonconvex={
			offset_b={-0.25, 0.75, 1.25},
			count=2,
			contacts={
				{offset={0.5, 1, 2}, depth=0.5, normal={0, 1, 0}, feature_id=21},
				{offset={1.5, -1, 0}, depth=0.125, normal={1, 0, 0}, feature_id=22},
				{},
				{},
			},
		},
	};
	materialized_manifolds[2] = {kind=.Convex};
	materialized_manifolds[3] = {
		kind=.Convex,
		convex={
			offset_b={0.5, 0, 0},
			count=1,
			normal={0, 1, 0},
			contacts={
				{offset={1, 0, 0}, depth=0.75, feature_id=31}, {}, {}, {},
			},
		},
	};
	continuations := [4]physics.Narrow_Phase_CCD_Continuation{
		{}, {}, {},
		{
			relative_linear_velocity={0.5, -2, 1},
			angular_a={0, 0, 1},
			angular_b={0, 0, -0.5},
			t=0.25,
			kind=.Continuous,
		},
	};
	candidate_a, candidate_a_status := physics.collidable_reference_create(.Dynamic, 0);
	candidate_b, candidate_b_status := physics.collidable_reference_create(.Static, 0);
	testing.expect_value(t, candidate_a_status, physics.Physics_Status.Ok);
	testing.expect_value(t, candidate_b_status, physics.Physics_Status.Ok);
	accessors: physics.Contact_Constraint_Accessors;
	testing.expect_value(
		t, physics.contact_constraint_accessors_initialize(&accessors),
		physics.Physics_Status.Ok,
	);
	defer physics.contact_constraint_accessors_dispose(&accessors);
	for case_index in 0 ..< len(materialized_manifolds)
	{
		materialized_input := materialized_manifolds[case_index];
		stored_input := physics.Collision_Stored_Manifold{kind=materialized_input.kind};
		if materialized_input.kind == .Convex
		{
			stored_input.convex = materialized_input.convex;
		}
		else
		{
			stored_input.nonconvex = materialized_input.nonconvex;
		}
		materialized_results := [1]physics.Narrow_Phase_Pair_Result{
			{manifold={kind=.Convex, convex={count=4}}, material={friction_coefficient=123}},
		};
		stored_results := materialized_results;
		materialized_continuations := [1]physics.Narrow_Phase_CCD_Continuation{
			continuations[case_index],
		};
		stored_continuations := materialized_continuations;
		candidate := physics.Broad_Phase_Pair{candidate_a, candidate_b};
		candidates := [1]physics.Broad_Phase_Pair{candidate};
		materialized_narrow := physics.Narrow_Phase{
			callbacks=callbacks,
			candidates={memory=&candidates[0], length=1, id=util.BUFFER_CALLER_OWNED_ID},
			continuations={
				memory=&materialized_continuations[0], length=1,
				id=util.BUFFER_CALLER_OWNED_ID,
			},
			results={
				memory=&materialized_results[0], length=1,
				id=util.BUFFER_CALLER_OWNED_ID,
			},
			state=.Stepping,
		};
		stored_narrow := physics.Narrow_Phase{
			callbacks=callbacks,
			stored_completion_material=material,
			stored_completion_state=.Present,
			candidates={memory=&candidates[0], length=1, id=util.BUFFER_CALLER_OWNED_ID},
			continuations={
				memory=&stored_continuations[0], length=1,
				id=util.BUFFER_CALLER_OWNED_ID,
			},
			results={memory=&stored_results[0], length=1, id=util.BUFFER_CALLER_OWNED_ID},
			state=.Stepping,
		};
		materialized_worker := physics.Narrow_Phase_Worker_Context{
			narrow=&materialized_narrow,
		};
		stored_worker := physics.Narrow_Phase_Worker_Context{narrow=&stored_narrow};
		testing.expect_value(
			t,
			physics.narrow_phase_pair_completed(&materialized_worker, 0, &materialized_input),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(
			t,
			physics.narrow_phase_pair_completed_stored(&stored_worker, 0, &stored_input),
			physics.Physics_Status.Ok,
		);
		expect_narrow_pair_results_equal(t, &stored_results[0], &materialized_results[0]);
		testing.expect_value(
			t, physics.collision_stored_manifold_materialize(stored_input), materialized_input,
		);
		if stored_results[0].state == .Accepted
		{
			materialized_description: [physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8;
			stored_description: [physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8;
			materialized_type, materialized_features, materialized_build_status :=
				physics.contact_constraint_build_description(
				&accessors, &materialized_input, 1, material,
				&materialized_description[0],
			);
			stored_narrow.accessors = accessors;
			stored_type, stored_features, stored_build_status :=
				physics.narrow_phase_build_stored_description(
				&stored_narrow, &stored_results[0], 1,
				&stored_description[0],
			);
			testing.expect_value(
				t, stored_build_status, materialized_build_status,
			);
			testing.expect_value(t, stored_type, materialized_type);
			testing.expect_value(t, stored_features, materialized_features);
			testing.expect_value(t, stored_description, materialized_description);
			if stored_input.kind == .Convex
			{
				fused_header: physics.Narrow_Phase_Transaction_Header;
				fused_transaction: physics.Narrow_Phase_Contact_Transaction;
				fused_pair := physics.collidable_pair_create(candidate_a, candidate_b);
				physics.narrow_phase_write_fused_stored_description_trusted(
					&stored_narrow, fused_pair, &stored_input,
					&fused_header, &fused_transaction,
				);
				testing.expect_value(t, fused_header.type_id, i16(stored_type));
				testing.expect_value(t, fused_header.body_count, u8(1));
				testing.expect_value(
					t, fused_transaction.feature_ids, stored_features,
				);
				testing.expect_value(
					t, fused_transaction.description, stored_description,
				);
			}
		}
	}
	testing.expect_value(t, callback_state.configure_count, i32(3));
	fused_candidates: [4]physics.Broad_Phase_Pair;
	fused_continuations: [4]physics.Narrow_Phase_CCD_Continuation;
	fused_results: [4]physics.Narrow_Phase_Pair_Result;
	fused_results[0] = {
		manifold={kind=.Convex, convex={count=4}},
		material={friction_coefficient=123},
		state=.Accepted,
	};
	fused_continuations[0] = {t=0.75, kind=.Continuous};
	fused_narrow := physics.Narrow_Phase{
		stored_completion_state=.Present,
		pending_capacity_per_worker=4,
		active_worker_count=1,
		candidates={memory=&fused_candidates[0], length=4, id=util.BUFFER_CALLER_OWNED_ID},
		continuations={memory=&fused_continuations[0], length=4, id=util.BUFFER_CALLER_OWNED_ID},
		results={memory=&fused_results[0], length=4, id=util.BUFFER_CALLER_OWNED_ID},
		state=.Stepping,
	};
	fused_narrow.batchers[0].state = .Ready;
	fused_narrow.transaction_streams[0].traversal_candidate_count = 2;
	fused_context := physics.Narrow_Phase_Worker_Context{
		narrow=&fused_narrow, worker_index=0,
	};
	valid_fused := physics.Collision_Stored_Manifold{
		kind=.Convex,
		convex={count=1, normal={0, 1, 0}},
	};
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 0, &valid_fused,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0], physics.Physics_Status.Ok,
	);
	immediate_empty := physics.Collision_Stored_Manifold{kind=.Convex};
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 2, &immediate_empty,
	),
		physics.Physics_Status.Ok,
	);
	invalid_fused := physics.Collision_Stored_Manifold{kind=.Nonconvex};
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 0, &invalid_fused,
	),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0], physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 0, nil,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0], physics.Physics_Status.Ok,
	);
	fused_narrow.batchers[0].state = .Flushing;
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 2, &immediate_empty,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0],
		physics.Physics_Status.Invalid_Description,
	);
	fused_narrow.transaction_statuses[0] = .Ok;
	testing.expect_value(
		t, physics.narrow_phase_pair_completed_stored_fused(
		&fused_context, 1, &valid_fused,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0], physics.Physics_Status.Ok,
	);
	testing.expect_value(t, fused_results[0].state, physics.Narrow_Phase_Result_State.Accepted);
	testing.expect_value(t, fused_results[0].material.friction_coefficient, f32(123));
	testing.expect_value(t, fused_continuations[0].t, f32(0.75));
	flush_pairs: [2]physics.Collision_Batcher_Pair;
	flush_pairs[0] = {
		pair_id=0,
		result_state=.Complete,
		result={kind=.Nonconvex},
	};
	flush_pairs[1] = {
		pair_id=2,
		result_state=.Pending,
		result={kind=.Convex},
	};
	fused_narrow.batchers[0] = {
		pairs={memory=&flush_pairs[0], length=2, id=util.BUFFER_CALLER_OWNED_ID},
		pair_count=2,
		state=.Ready,
	};
	testing.expect_value(
		t, physics.narrow_phase_flush_worker_chunk_fused(&fused_narrow, 0, 2, 0, 2),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(
		t, fused_narrow.transaction_streams[0].total_count, i32(0),
	);
	fused_narrow.batchers[0].state = .Ready;
	fused_narrow.batchers[0].pair_count = 1;
	fused_narrow.transaction_streams[0].traversal_candidate_count = 1;
	direct_records := ([^]physics.Narrow_Phase_Convex_Direct_Record)(
		rawptr(&flush_pairs[0])
	);
	direct_records[0] = {pair_id=0, speculative_margin=0};
	flush_context := physics.Narrow_Phase_Overlap_Context{
		narrow=&fused_narrow,
		worker_count=1,
		fused_route=.Box_Direct,
	};
	testing.expect_value(
		t, physics.narrow_phase_flush_worker_chunk(&flush_context, 0, .Predecessor),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(t, fused_narrow.batchers[0].pair_count, 0);
	testing.expect_value(t, fused_narrow.transaction_streams[0].total_count, i32(0));
}

verify_phase3_dispatch_failure_recovery :: proc(t: ^testing.T)
{
	material := physics.Contact_Material_Properties{
		friction_coefficient=0.7,
		spring_settings={angular_frequency=30, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	phase3_worker_pool: util.Dispatcher_Worker_Pool_Proc = proc (
		dispatcher: ^util.Thread_Dispatcher_Boundary, worker_index: int,
	) -> (^util.Buffer_Pool, util.Threading_Status)
	{
		if dispatcher == nil || dispatcher.dispatcher == nil ||
			worker_index < 0 || worker_index >= 2
		{
			return nil, .Invalid_Argument;
		}
		state := (^Phase3_Test_Dispatch_State)(dispatcher.dispatcher);
		return state.pools[worker_index], .Ok;
	}
	for dispatch_case in 0 ..< 3
	{
		case_pools: [2]util.Buffer_Pool;
		for pool_index in 0 ..< len(case_pools)
		{
			test_pool_prepare(t, &case_pools[pool_index]);
		}
		dispatch_state := Phase3_Test_Dispatch_State{
			pools={&case_pools[0], &case_pools[1]},
		};
		custom_dispatcher := util.Thread_Dispatcher_Boundary{
			dispatcher=&dispatch_state,
			dispatch=phase3_test_dispatch,
			worker_pool=phase3_worker_pool,
			worker_count=2,
		};
		error_state := Phase3_Direct_Error_State{failure_enabled=.Present};
		error_callbacks := physics.narrow_phase_default_callbacks();
		error_callbacks.allow = phase3_direct_error_allow;
		error_callbacks.user_context = &error_state;
		error_description := test_simulation_description(
			&case_pools[0], error_callbacks,
		);
		error_description.allocation_sizes.workers = 2;
		error_memory, error_allocation := mem.alloc( // odin-contracts-allow: test-fixture rule=ODIN_HOT_ALLOCATE_OR_REALLOC owner=Phase3_Direct_Error_Fixture phase=test_setup reason=simulation_fixture_lifetime
			size_of(physics.Simulation), align_of(physics.Simulation),
		);
		if !testing.expect(t, error_allocation == nil && error_memory != nil)
		{
			return;
		}
		error_simulation := (^physics.Simulation)(error_memory);
		error_simulation^ = {};
		error_create_status := physics.simulation_create(
			error_simulation, &error_description,
		).status;
		if !testing.expect_value(
			t, error_create_status, physics.Physics_Status.Ok,
		)
		{
			_ = mem.free(error_simulation); // odin-contracts-allow: test-fixture rule=ODIN_HOT_DELETE_OR_FREE owner=Phase3_Direct_Error_Fixture phase=test_cleanup reason=failed_create_fixture_cleanup
			for pool_index in 0 ..< len(case_pools)
			{
				_ = util.buffer_pool_dispose(&case_pools[pool_index]);
			}
			return;
		}
		error_state.simulation = error_simulation;
		testing.expect_value(
			t, physics.narrow_phase_bind_default_stored_completion(
			&error_simulation.narrow_phase, material,
		),
			physics.Physics_Status.Ok,
		);
		error_box := physics.Box{0.5, 0.5, 0.5};
		error_shape, error_shape_status := physics.shape_registry_add(
			&error_simulation.shapes, physics.BOX_TYPE_ID, &error_box,
		);
		testing.expect_value(t, error_shape_status, physics.Physics_Status.Ok);
		error_body_shape: physics.Typed_Index = error_shape;
		if dispatch_case == 2
		{
			sphere: physics.Sphere = {radius=0.5};
			sphere_status: physics.Physics_Status;
			error_body_shape, sphere_status = physics.shape_registry_add(&error_simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
			testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
		}
		error_body_description := test_body_description(error_body_shape, {0, 0, 0});
		error_body_description.activity.sleep_threshold = -1;
		error_body, error_body_status := physics.simulation_add_body(
			error_simulation, &error_body_description,
		);
		testing.expect_value(t, error_body_status, physics.Physics_Status.Ok);
		for static_index in 0 ..< 4
		{
			_, error_static_status := physics.simulation_add_static(
				error_simulation,
				&physics.Static_Description{
					pose={
						orientation=util.quaternion_identity(),
						position={f32(static_index) * 0.05, 0, 0},
					},
					shape=error_shape,
				},
			);
			testing.expect_value(t, error_static_status, physics.Physics_Status.Ok);
		}
		if dispatch_case >= 1
		{
			error_state.dispatcher = &custom_dispatcher;
		}
		execute_thread := thread.create(phase3_execute_controlled);
		if !testing.expect(t, execute_thread != nil)
		{
			_ = physics.simulation_destroy(error_simulation);
			return;
		}
		execute_thread.data = &error_state;
		thread.start(execute_thread);
		phase3_wait_for_futex(&error_state.entered, sync.Futex(1));
		testing.expect_value(
			t, error_state.published_observed, physics.Reference_State.Present,
		);
		sync.atomic_store_explicit(&error_state.release, sync.Futex(1), .Release);
		sync.futex_broadcast(&error_state.release);
		thread.join(execute_thread);
		thread.destroy(execute_thread);
		testing.expect_value(
			t, error_state.status, physics.Physics_Status.Invalid_Argument,
		);
		for worker_index in 0 ..< 2
		{
			testing.expect_value(
				t, error_simulation.narrow_phase.batchers[worker_index].pair_count,
				0,
			);
		}
		error_state.failure_enabled = .Missing;
		testing.expect_value(
			t, physics.simulation_apply_body_description(
			error_simulation, error_body, &error_body_description,
		),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(
			t, physics.simulation_timestep(
				error_simulation, 1.0 / 60.0, error_state.dispatcher,
			),
			physics.Physics_Status.Ok,
		);
		unused_capsule := physics.Capsule{radius=0.125, half_length=0.125};
		_, unused_capsule_status := physics.shape_registry_add(
			&error_simulation.shapes, physics.CAPSULE_TYPE_ID, &unused_capsule,
		);
		testing.expect_value(t, unused_capsule_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, physics.simulation_apply_body_description(
			error_simulation, error_body, &error_body_description,
		),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(
			t, physics.simulation_timestep(
				error_simulation, 1.0 / 60.0, error_state.dispatcher,
			),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(
			t, physics.simulation_destroy(error_simulation),
			physics.Physics_Status.Ok,
		);
		testing.expect_value(t, mem.free(error_simulation), mem.Allocator_Error.None); // odin-contracts-allow: test-fixture rule=ODIN_HOT_DELETE_OR_FREE owner=Phase3_Direct_Error_Fixture phase=test_cleanup reason=simulation_fixture_lifetime
		for pool_index in 0 ..< len(case_pools)
		{
			testing.expect_value(
				t, util.buffer_pool_dispose(&case_pools[pool_index]),
				util.Memory_Status.Ok,
			);
		}
	}
}

verify_phase3_nonfinite_margin_routes :: proc(t: ^testing.T)
{
	material := physics.Contact_Material_Properties{
		friction_coefficient=0.7,
		spring_settings={angular_frequency=30, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	maximum_finite_margin := transmute(f32)u32(0x7f7f_ffff);
	nonfinite_margins := [3]f32{
		maximum_finite_margin + maximum_finite_margin,
		transmute(f32)u32(0x7fc0_0000),
		transmute(f32)u32(0x7f80_0000),
	};
	for margin_case in 0 ..< len(nonfinite_margins)
	{
		outcomes: [2]Phase3_World_Outcome;
		for route_case in 0 ..< 2
		{
			world_pool := &test_phase3_world_pools[route_case];
			world_pool^ = {};
			test_pool_prepare(t, world_pool);
			world := &test_phase3_world_simulations[route_case];
			world^ = {};
			margin_state := Narrow_Callback_State{
				margin_override=0.25,
				allow_result=.Allow,
				configure_result=.Allow,
			};
			margin_callbacks := physics.Narrow_Phase_Callbacks{
				initialize=test_narrow_initialize,
				allow=test_narrow_allow,
				allow_child=test_narrow_allow_child,
				configure=test_narrow_configure,
				configure_child=test_narrow_configure_child,
				dispose=test_narrow_dispose,
				user_context=&margin_state,
			};
			world_description := test_simulation_description(
				world_pool, margin_callbacks,
			);
			world_create_status := physics.simulation_create(
				world, &world_description,
			).status;
			if !testing.expect_value(
				t, world_create_status, physics.Physics_Status.Ok,
			)
			{
				_ = util.buffer_pool_dispose(world_pool);
				return;
			}
			testing.expect_value(
				t, physics.narrow_phase_bind_default_stored_completion(
				&world.narrow_phase, material,
			),
				physics.Physics_Status.Ok,
			);
			world_box := physics.Box{0.5, 0.5, 0.5};
			world_shape, world_shape_status := physics.shape_registry_add(
				&world.shapes, physics.BOX_TYPE_ID, &world_box,
			);
			testing.expect_value(t, world_shape_status, physics.Physics_Status.Ok);
			if route_case == 1
			{
				unused_capsule := physics.Capsule{radius=0.125, half_length=0.125};
				_, capsule_status := physics.shape_registry_add(
					&world.shapes, physics.CAPSULE_TYPE_ID, &unused_capsule,
				);
				testing.expect_value(t, capsule_status, physics.Physics_Status.Ok);
			}
			body_a_description := test_body_description(world_shape, {0, 0, 0});
			body_b_description := test_body_description(world_shape, {8, 0, 0});
			body_a_description.collidable.continuity.mode = .Passive;
			body_b_description.collidable.continuity.mode = .Passive;
			body_a_description.activity.sleep_threshold = -1;
			body_b_description.activity.sleep_threshold = -1;
			body_a, body_a_status := physics.simulation_add_body(
				world, &body_a_description,
			);
			body_b, body_b_status := physics.simulation_add_body(
				world, &body_b_description,
			);
			testing.expect_value(t, body_a_status, physics.Physics_Status.Ok);
			testing.expect_value(t, body_b_status, physics.Physics_Status.Ok);
			outcome := &outcomes[route_case];
			warm_status := physics.simulation_timestep(world, 1.0 / 60.0);
			testing.expect_value(t, warm_status, physics.Physics_Status.Ok);
			outcome.route = physics.narrow_phase_collision_route_state(
				&world.narrow_phase, 1,
			);
			if margin_case == 0
			{
				tracker := (^mem.Tracking_Allocator)(context.allocator.data); // odin-contracts-allow: test-fixture rule=ODIN_HOT_DEFAULT_ALLOCATOR owner=Phase3_World_Outcome_Fixture phase=allocation_assertion reason=test_allocator_measurement
				allocation_count_before := tracker.total_allocation_count;
				hot_status := physics.Physics_Status.Ok;
				for _ in 0 ..< 32
				{
					hot_status = physics.simulation_timestep(world, 1.0 / 60.0);
					if hot_status != .Ok
					{
						break;
					}
				}
				testing.expect_value(t, hot_status, physics.Physics_Status.Ok);
				testing.expect_value(
					t, tracker.total_allocation_count, allocation_count_before,
				);
			}
			body_b_description.pose.position = {0, 0.9, 0};
			testing.expect_value(
				t, physics.simulation_apply_body_description(
				world, body_a, &body_a_description,
			),
				physics.Physics_Status.Ok,
			);
			testing.expect_value(
				t, physics.simulation_apply_body_description(
				world, body_b, &body_b_description,
			),
				physics.Physics_Status.Ok,
			);
			continuation_sentinel := physics.Narrow_Phase_CCD_Continuation{
				t=0.375, worker_index=-77, kind=.Continuous,
			};
			world.narrow_phase.continuations.memory[0] = continuation_sentinel;
			margin_state.margin_override = nonfinite_margins[margin_case];
			outcome.status = physics.simulation_timestep(world, 1.0 / 60.0);
			outcome.pair_count = world.narrow_phase.pair_cache.mapping.count;
			outcome.constraint_count = world.solver.active_set.constraint_count;
			testing.expect(
				t, world.narrow_phase.continuations.memory[0].worker_index != -77,
			);
			testing.expect(t, margin_state.allow_count > 0);
			if outcome.pair_count > 0
			{
				outcome.handle_state = .Present;
				testing.expect_value(t, outcome.pair_count, 1);
				testing.expect_value(t, outcome.constraint_count, i32(1));
				testing.expect_value(
					t, world.narrow_phase.continuations.memory[0].pair,
					world.narrow_phase.pair_cache.mapping.keys.memory[0],
				);
			}
			else
			{
				testing.expect_value(t, outcome.constraint_count, i32(0));
			}
		}
		direct_world := &test_phase3_world_simulations[0];
		predecessor_world := &test_phase3_world_simulations[1];
		testing.expect_value(
			t, outcomes[0].route, physics.Narrow_Phase_Collision_Route.Box_Direct,
		);
		testing.expect_value(
			t, outcomes[1].route, physics.Narrow_Phase_Collision_Route.Predecessor,
		);
		testing.expect_value(t, outcomes[0].status, outcomes[1].status);
		testing.expect_value(t, outcomes[0].pair_count, outcomes[1].pair_count);
		testing.expect_value(
			t, outcomes[0].constraint_count, outcomes[1].constraint_count,
		);
		testing.expect_value(t, outcomes[0].handle_state, outcomes[1].handle_state);
		testing.expect_value(
			t, direct_world.narrow_phase.continuations.memory[0],
			predecessor_world.narrow_phase.continuations.memory[0],
		);
		testing.expect_value(
			t, direct_world.narrow_phase.last_stage,
			predecessor_world.narrow_phase.last_stage,
		);
		if outcomes[0].handle_state == .Present
		{
			testing.expect_value(
				t, direct_world.narrow_phase.pair_cache.mapping.keys.memory[0],
				predecessor_world.narrow_phase.pair_cache.mapping.keys.memory[0],
			);
			direct_cache := &direct_world.narrow_phase.pair_cache.mapping.values.memory[0];
			predecessor_cache :=
				&predecessor_world.narrow_phase.pair_cache.mapping.values.memory[0];
			testing.expect_value(
				t, direct_cache.constraint_handle, predecessor_cache.constraint_handle,
			);
			testing.expect_value(t, direct_cache.feature_ids, predecessor_cache.feature_ids);
			testing.expect_value(
				t, direct_world.narrow_phase.pair_cache.pair_freshness.memory[0],
				predecessor_world.narrow_phase.pair_cache.pair_freshness.memory[0],
			);
			direct_capture := (^Contact_View_Capture)(
				rawptr(direct_world.narrow_phase.results.memory),
			);
			predecessor_capture := (^Contact_View_Capture)(
				rawptr(predecessor_world.narrow_phase.results.memory),
			);
			direct_capture^ = {};
			predecessor_capture^ = {};
			testing.expect_value(
				t, physics.narrow_phase_try_extract_solver_contact_data(
				&direct_world.narrow_phase, direct_cache.constraint_handle,
				capture_contact_view, direct_capture,
			),
				physics.Physics_Status.Ok,
			);
			testing.expect_value(
				t, physics.narrow_phase_try_extract_solver_contact_data(
				&predecessor_world.narrow_phase, predecessor_cache.constraint_handle,
				capture_contact_view, predecessor_capture,
			),
				physics.Physics_Status.Ok,
			);
			testing.expect_value(t, direct_capture.call_count, i32(1));
			testing.expect_value(t, predecessor_capture.call_count, i32(1));
			testing.expect_value(
				t, direct_capture.view, predecessor_capture.view,
			);
		}
		for route_case in 0 ..< 2
		{
			testing.expect_value(
				t, physics.simulation_destroy(&test_phase3_world_simulations[route_case]),
				physics.Physics_Status.Ok,
			);
			testing.expect_value(
				t, util.buffer_pool_dispose(&test_phase3_world_pools[route_case]),
				util.Memory_Status.Ok,
			);
		}
	}
}

verify_queue_continuations_match_sweep_reference :: proc(t: ^testing.T)
{
	modes: [3]physics.Continuous_Detection_Mode = {.Discrete, .Passive, .Continuous};
	worker_index: int = 1;
	dt: f32 = 1.0 / 60.0;
	margin: f32 = 0.25;
	seed: physics.Narrow_Phase_CCD_Continuation = {
		pair={{0x4000_0011}, {0x8000_0013}},
		relative_linear_velocity={11, 12, 13}, angular_a={14, 15, 16},
		angular_b={17, 18, 19}, t=0.375, worker_index=-77, kind=.Continuous,
	};
	for route_case in 0 ..< 3
	{
		pool: ^util.Buffer_Pool = &test_phase3_world_pools[0];
		pool^ = {};
		test_pool_prepare(t, pool);
		world: ^physics.Simulation = &test_phase3_world_simulations[0];
		world^ = {};
		callback_state: Narrow_Callback_State = {
			margin_override=margin, allow_result=.Allow, configure_result=.Allow,
		};
		callbacks: physics.Narrow_Phase_Callbacks = {
			initialize=test_narrow_initialize, allow=test_narrow_allow,
			allow_child=test_narrow_allow_child, configure=test_narrow_configure,
			configure_child=test_narrow_configure_child, dispose=test_narrow_dispose,
			user_context=&callback_state,
		};
		description: physics.Simulation_Create_Description = test_simulation_description(pool, callbacks);
		description.allocation_sizes.workers = 2;
		if !testing.expect_value(t, physics.simulation_create(world, &description).status, physics.Physics_Status.Ok)
		{
			_ = util.buffer_pool_dispose(pool);
			return;
		}
		testing.expect_value(t, physics.narrow_phase_bind_default_stored_completion(&world.narrow_phase, {
			friction_coefficient=0.7, spring_settings={angular_frequency=30, twice_damping_ratio=2},
			maximum_recovery_velocity=2,
		}), physics.Physics_Status.Ok);
		box: physics.Box = {0.5, 0.5, 0.5};
		shape: physics.Typed_Index;
		status: physics.Physics_Status;
		shape, status = physics.shape_registry_add(&world.shapes, physics.BOX_TYPE_ID, &box);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		shapes: [3]physics.Typed_Index = {shape, shape, shape};
		if route_case == 1
		{
			unused_capsule: physics.Capsule = {radius=0.125, half_length=0.125};
			_, status = physics.shape_registry_add(&world.shapes, physics.CAPSULE_TYPE_ID, &unused_capsule);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		else if route_case == 2
		{
			sphere: physics.Sphere = {radius=0.5};
			shapes[1], status = physics.shape_registry_add(&world.shapes, physics.SPHERE_TYPE_ID, &sphere);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			shapes[2] = shapes[1];
		}
		body_descriptions: [2]physics.Body_Description = {
			test_body_description(shapes[0], {0, 0, 0}),
			test_body_description(shapes[1], {3, 0.125, 0}),
		};
		body_descriptions[0].velocity = {linear={240, 0, 0}, angular={0.1, 0.2, -0.15}};
		body_descriptions[1].velocity = {linear={-30, 0, 0}, angular={-0.2, 0.1, 0.05}};
		body_descriptions[1].pose.orientation = util.quaternion_from_axis_angle({0, 1, 0}, 0.2);
		body_handles: [2]physics.Body_Handle;
		body_locations: [2]physics.Body_Memory_Location;
		references: [3]physics.Collidable_Reference;
		for body_index in 0 ..< 2
		{
			body_handles[body_index], status = physics.simulation_add_body(world, &body_descriptions[body_index]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			body_locations[body_index], status = physics.bodies_resolve(&world.bodies, body_handles[body_index]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			references[body_index], status = physics.collidable_reference_body(.Dynamic, body_handles[body_index]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		static_description: physics.Static_Description = {
			pose={position={3, -0.125, 0}, orientation=util.quaternion_identity()}, shape=shapes[2],
		};
		static_handle: physics.Static_Handle;
		static_handle, status = physics.simulation_add_static(world, &static_description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		references[2], status = physics.collidable_reference_static(static_handle);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		static_index: int = int(world.statics.handle_to_index.memory[static_handle.value]);
		narrow: ^physics.Narrow_Phase = &world.narrow_phase;
		testing.expect_value(t, physics.narrow_phase_prepare_batchers(narrow, 2), physics.Physics_Status.Ok);
		route: physics.Narrow_Phase_Collision_Route = physics.narrow_phase_collision_route_state(narrow, 2);
		if route == .Predecessor
		{
			route = physics.narrow_phase_box_sphere_route_state(narrow, 2);
		}
		expected_route: physics.Narrow_Phase_Collision_Route = .Box_Direct;
		if route_case == 1
		{
			expected_route = .Predecessor;
		}
		else if route_case == 2
		{
			expected_route = .Box_Sphere_Direct;
		}
		testing.expect_value(t, route, expected_route);
		narrow.state = .Stepping;
		batcher: ^physics.Collision_Batcher = &narrow.batchers[worker_index];
		shape_data: [3]rawptr;
		for source in 0 ..< len(shapes)
		{
			shape_data[source], _, status = physics.shape_registry_resolve(&world.shapes, shapes[source]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		pair_sources: [4][2]int = {{0, 1}, {1, 0}, {0, 2}, {2, 0}};
		continuous_hits: int = 0;
		for pair_source in pair_sources
		{
			pair: physics.Broad_Phase_Pair = {references[pair_source[0]], references[pair_source[1]]};
			for mode_a in modes
			{
				for mode_b in modes
				{
					continuities: [2]physics.Continuous_Detection = {
						{mode=mode_a, minimum_sweep_timestep=0.001, sweep_convergence_threshold=0.001},
						{mode=mode_b, minimum_sweep_timestep=0.001, sweep_convergence_threshold=0.001},
					};
					poses: [2]physics.Rigid_Pose;
					velocities: [2]physics.Body_Velocity;
					for side in 0 ..< 2
					{
						source: int = pair_source[side];
						if source == 2
						{
							world.statics.statics.memory[static_index].continuity = continuities[side];
							poses[side] = static_description.pose;
						}
						else
						{
							location: physics.Body_Memory_Location = body_locations[source];
							world.bodies.sets.memory[location.set_index].collidables.memory[location.index].continuity = continuities[side];
							poses[side] = body_descriptions[source].pose;
							velocities[side] = body_descriptions[source].velocity;
						}
					}
					batcher.pair_count = 0;
					physics.collision_batcher_reset_task_lists(batcher);
					narrow.continuations.memory[1] = seed;
					status = physics.narrow_phase_prepare_continuation(
						narrow, 1, worker_index, pair, shapes[pair_source[0]], shapes[pair_source[1]], &poses[0], &poses[1],
						velocities[0], velocities[1], continuities[0], continuities[1], margin, dt,
					);
					if !testing.expect_value(t, status, physics.Physics_Status.Ok)
					{
						continue;
					}
					expected_continuation: physics.Narrow_Phase_CCD_Continuation = narrow.continuations.memory[1];
					if expected_continuation.kind == .Continuous
					{
						continuous_hits += 1;
					}
					if mode_a != .Continuous && mode_b != .Continuous
					{
						testing.expect_value(t, expected_continuation, physics.Narrow_Phase_CCD_Continuation{
							pair=physics.collidable_pair_create(pair.a, pair.b), worker_index=i32(worker_index), kind=.Discrete,
						});
					}
					narrow.candidates.memory[0] = pair;
					narrow.continuations.memory[0] = seed;
					if route == .Box_Sphere_Direct
					{
						status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Box_Sphere_Direct);
					}
					else
					{
						status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Predecessor);
					}
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					testing.expect_value(t, narrow.continuations.memory[0], expected_continuation);
					testing.expect_value(t, narrow.continuations.memory[0].pair, physics.collidable_pair_create(pair.a, pair.b));
					testing.expect_value(t, narrow.continuations.memory[0].worker_index, i32(worker_index));
					testing.expect_value(t, batcher.pair_count, 1);
					if route == .Predecessor
					{
						maximum_expansion: f32 = margin;
						if mode_a != .Discrete || mode_b != .Discrete
						{
							maximum_expansion = transmute(f32)u32(0x7f7f_ffff);
						}
						testing.expect_value(t, batcher.pairs.memory[0].pair_id, i32(0));
						testing.expect_value(t, batcher.pairs.memory[0].result_state, physics.Collision_Batcher_Result_State.Pending);
						input: ^physics.Collision_Batcher_Input = &batcher.pairs.memory[0].input;
						testing.expect_value(t, input.shape_a, shapes[pair_source[0]]);
						testing.expect_value(t, input.shape_b, shapes[pair_source[1]]);
						testing.expect_value(t, input.shape_data_a, shape_data[pair_source[0]]);
						testing.expect_value(t, input.shape_data_b, shape_data[pair_source[1]]);
						testing.expect_value(t, input.speculative_margin, margin);
						testing.expect_value(t, input.dt, dt);
						testing.expect_value(t, input.maximum_expansion, maximum_expansion);
						testing.expect_value(t, input.next_in_task, i32(-1));
						testing.expect_value(t, input.order, physics.Collision_Task_Route_Order.Expected);
						testing.expect_value(t, input.pose_a.position, poses[0].position);
						testing.expect_value(t, input.pose_a.orientation, poses[0].orientation);
						testing.expect_value(t, input.pose_b.position, poses[1].position);
						testing.expect_value(t, input.pose_b.orientation, poses[1].orientation);
						testing.expect_value(t, input.velocity_a.linear, velocities[0].linear);
						testing.expect_value(t, input.velocity_a.angular, velocities[0].angular);
						testing.expect_value(t, input.velocity_b.linear, velocities[1].linear);
						testing.expect_value(t, input.velocity_b.angular, velocities[1].angular);
					}
					else if route == .Box_Sphere_Direct
					{
						records: [^]physics.Narrow_Phase_Box_Sphere_Record = ([^]physics.Narrow_Phase_Box_Sphere_Record)(rawptr(batcher.pairs.memory));
						reference: physics.Collision_Task_Reference;
						_, reference, status = physics.collision_task_registry_lookup(
							narrow.tasks, int(physics.typed_index_type(shapes[pair_source[0]])),
							int(physics.typed_index_type(shapes[pair_source[1]])),
						);
						testing.expect_value(t, status, physics.Physics_Status.Ok);
						testing.expect_value(t, records[0].pair_id, i32(0));
						testing.expect_value(t, records[0].speculative_margin, margin);
						testing.expect_value(t, records[0].shape_data_a, shape_data[pair_source[0]]);
						testing.expect_value(t, records[0].shape_data_b, shape_data[pair_source[1]]);
						testing.expect_value(t, records[0].pose_a, poses[0]);
						testing.expect_value(t, records[0].pose_b, poses[1]);
						testing.expect_value(t, records[0].next_in_task, i32(-1));
						testing.expect_value(t, records[0].order, reference.order);
						testing.expect_value(t, batcher.task_heads[reference.task_id], i32(0));
						testing.expect_value(t, batcher.task_tails[reference.task_id], i32(0));
					}
					else
					{
						direct_records: [^]physics.Narrow_Phase_Convex_Direct_Record = ([^]physics.Narrow_Phase_Convex_Direct_Record)(rawptr(batcher.pairs.memory));
						testing.expect_value(t, direct_records[0].pair_id, i32(0));
						testing.expect_value(t, direct_records[0].speculative_margin, margin);
						testing.expect_value(t, direct_records[0].shape_data_a, shape_data[pair_source[0]]);
						testing.expect_value(t, direct_records[0].shape_data_b, shape_data[pair_source[1]]);
						testing.expect_value(t, direct_records[0].pose_a.position, poses[0].position);
						testing.expect_value(t, direct_records[0].pose_a.orientation, poses[0].orientation);
						testing.expect_value(t, direct_records[0].pose_b.position, poses[1].position);
						testing.expect_value(t, direct_records[0].pose_b.orientation, poses[1].orientation);
					}
				}
			}
		}
		testing.expect(t, continuous_hits > 0);
		batcher.pair_count = int(batcher.pairs.length);
		narrow.continuations.memory[0] = seed;
		expected_capacity_status: physics.Physics_Status = .Capacity_Missing;
		if route == .Predecessor
		{
			batcher.state = .Faulted;
			expected_capacity_status = .Invalid_Argument;
		}
		if route == .Box_Sphere_Direct
		{
			status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Box_Sphere_Direct);
		}
		else
		{
			status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Predecessor);
		}
		testing.expect_value(t, status, expected_capacity_status);
		testing.expect_value(t, narrow.continuations.memory[0], seed);
		batcher.pair_count = 0;
		batcher.state = .Ready;
		physics.collision_batcher_reset_task_lists(batcher);
		if route == .Box_Sphere_Direct
		{
			status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Box_Sphere_Direct);
		}
		else
		{
			status = physics.narrow_phase_queue_candidate(narrow, 0, worker_index, dt, route, .Predecessor);
		}
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, batcher.pair_count, 1);
		batcher.pair_count = 0;
		physics.collision_batcher_reset_task_lists(batcher);
		narrow.state = .Ready;
		testing.expect_value(t, physics.simulation_destroy(world), physics.Physics_Status.Ok);
		testing.expect_value(t, util.buffer_pool_dispose(pool), util.Memory_Status.Ok);
	}
}

test_body_description :: proc(shape: physics.Typed_Index, position: util.Vector3) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position=position},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=0.25},
		activity={sleep_threshold=0.001, minimum_timestep_count_under_threshold=255},
	};
}

@(test)
stored_and_materialized_completion_match_for_convex_nonconvex_empty_and_ccd :: proc(
	t: ^testing.T,
)
{
	verify_stored_and_materialized_completion_basics(t);
	verify_phase3_direct_route_completion(t);
	verify_phase3_dispatch_failure_recovery(t);
	verify_phase3_nonfinite_margin_routes(t);
	verify_queue_continuations_match_sweep_reference(t);
}

@(test)
pair_freshness_contact_update_and_removal :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	test_pool_prepare(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	callback_state := Narrow_Callback_State{
		margin_override=0, rejected_child=0,
		allow_result=.Allow, configure_result=.Allow,
	};
	callbacks := physics.Narrow_Phase_Callbacks{
		initialize=test_narrow_initialize,
		allow=test_narrow_allow,
		allow_child=test_narrow_allow_child,
		configure=test_narrow_configure,
		configure_child=test_narrow_configure_child,
		dispose=test_narrow_dispose,
		user_context=&callback_state,
	};
	description := test_simulation_description(&pool, callbacks);
	test_simulation = {};
	simulation := &test_simulation;
	result := physics.simulation_create(simulation, &description);
	testing.expect_value(t, result.status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	sphere := physics.Sphere{radius=1};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	children := [2]physics.Compound_Child{
		{local_position={-0.5, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=shape},
		{local_position={0.5, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=shape},
	};
	compound: physics.Big_Compound;
	testing.expect_value(
		t, physics.big_compound_create(&compound, &children[0], len(children), &simulation.shapes, &pool),
		physics.Physics_Status.Ok,
	);
	compound_shape, compound_status := physics.shape_registry_add(
		&simulation.shapes, physics.BIG_COMPOUND_TYPE_ID, &compound,
	);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	body_description := test_body_description(shape, {0, 1.947, 0});
	body_description.velocity.linear = {1, 0, 0};
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	static_description := physics.Static_Description{
		pose={orientation=util.quaternion_identity()}, shape=compound_shape,
	};
	_, static_status := physics.simulation_add_static(simulation, &static_description);
	testing.expect_value(t, static_status, physics.Physics_Status.Ok);
	last_status := physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect(t, callback_state.margin_before > callback_state.margin_override);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	body_description.pose.position = {0, 1.5, 0};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body, &body_description), physics.Physics_Status.Ok,
	);
	stale_result := physics.Narrow_Phase_Pair_Result{
		manifold={kind=.Convex, convex={count=4}},
		material={friction_coefficient=123},
		state=.Accepted,
	};
	simulation.narrow_phase.results.memory[0] = stale_result;
	forward_route_index := physics.collision_task_matrix_index(
		physics.SPHERE_TYPE_ID, physics.BIG_COMPOUND_TYPE_ID,
	);
	reverse_route_index := physics.collision_task_matrix_index(
		physics.BIG_COMPOUND_TYPE_ID, physics.SPHERE_TYPE_ID,
	);
	forward_route := simulation.collision_tasks.routes[forward_route_index];
	reverse_route := simulation.collision_tasks.routes[reverse_route_index];
	simulation.collision_tasks.routes[forward_route_index].task_id = -1;
	simulation.collision_tasks.routes[reverse_route_index].task_id = -1;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Rejected,
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].manifold.convex.count,
		i32(4),
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].material.friction_coefficient,
		f32(123),
	);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(0));
	simulation.collision_tasks.routes[forward_route_index] = forward_route;
	simulation.collision_tasks.routes[reverse_route_index] = reverse_route;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Accepted
	);
	testing.expect_value(t, simulation.narrow_phase.results.memory[0].manifold.kind, physics.Manifold_Kind.Nonconvex);
	testing.expect_value(t, simulation.narrow_phase.results.memory[0].manifold.nonconvex.count, i32(1));
	testing.expect_value(t, simulation.narrow_phase.last_stage, physics.Narrow_Phase_Stage.Changes_Flushed);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(1));
	testing.expect(t, callback_state.allow_child_count >= 2);
	testing.expect(t, callback_state.configure_child_count > 0);
	manifold_result := simulation.narrow_phase.results.memory[0].manifold;
	testing.expect_value(t, manifold_result.kind, physics.Manifold_Kind.Nonconvex);
	for contact_index in 0 ..< int(manifold_result.nonconvex.count)
	{
		feature := u32(manifold_result.nonconvex.contacts[contact_index].feature_id);
		testing.expect_value(t, feature & 0xff, u32(77));
		testing.expect(t, ((feature >> 8) & 0xff) != 0 || ((feature >> 16) & 0xff) != 0);
	}
	first_handle := simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
	contact_capture: Contact_View_Capture;
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, first_handle,
		capture_contact_view, &contact_capture,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, contact_capture.call_count, i32(1));
	testing.expect_value(
		t, contact_capture.view.type_id,
		i32(physics.CONTACT_1_ONE_BODY_TYPE_ID),
	);
	testing.expect_value(
		t, contact_capture.view.kind,
		physics.Contact_Constraint_Kind.Convex,
	);
	testing.expect_value(t, contact_capture.view.body_count, i32(1));
	testing.expect_value(t, contact_capture.view.contact_count, i32(1));
	testing.expect_value(t, contact_capture.view.body_handles[0], body);
	testing.expect_value(
		t, contact_capture.view.prestep_size,
		i32(size_of(physics.Contact_1_One_Body)),
	);
	builtin_prestep := (^physics.Contact_1_One_Body)(
		&contact_capture.view.prestep[0],
	);
	testing.expect_value(
		t, builtin_prestep.material.friction_coefficient, f32(0.7),
	);
	builtin_record, builtin_record_status :=
		physics.constraint_type_registry_lookup(
		&simulation.solver.registry,
		physics.CONTACT_1_ONE_BODY_TYPE_ID,
	);
	testing.expect_value(
		t, builtin_record_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, contact_capture.view.impulse_count,
		builtin_record.impulse_bundle_size /
		i32(size_of(util.F32x8)),
	);
	expect_contact_view_impulses_match_solver(
		t, simulation, first_handle, &contact_capture.view,
	);
	noncontact_description := physics.One_Body_Linear_Servo{
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
		servo_settings={maximum_speed=10, base_speed=0.1, maximum_force=100},
	};
	noncontact_handles := [4]physics.Body_Handle{body, {}, {}, {}};
	noncontact, noncontact_status := physics.simulation_add_constraint(
		simulation, &noncontact_handles, &noncontact_description,
	);
	testing.expect_value(t, noncontact_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, noncontact,
		capture_contact_view, &contact_capture,
	),
		physics.Physics_Status.Not_Found,
	);
	testing.expect_value(t, contact_capture.call_count, i32(1));
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase,
		physics.constraint_handle_invalid(),
		capture_contact_view, &contact_capture,
	),
		physics.Physics_Status.Not_Found,
	);
	testing.expect_value(t, contact_capture.call_count, i32(1));
	testing.expect_value(
		t, physics.simulation_remove_constraint(simulation, noncontact),
		physics.Physics_Status.Ok,
	);
	testing.expect(t, callback_state.allow_count > 0);
	testing.expect(t, callback_state.configure_count > 0);
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle,
		first_handle
	);
	simulation.narrow_phase.results.memory[0] = stale_result;
	callback_state.allow_result = .Reject;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Skipped,
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].manifold.convex.count,
		i32(4),
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].material.friction_coefficient,
		f32(123),
	);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(0));
	callback_state.allow_result = .Allow;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(1));
	simulation.narrow_phase.results.memory[0] = stale_result;
	callback_state.configure_result = .Reject;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Rejected,
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].manifold.convex.count,
		i32(4),
	);
	testing.expect_value(
		t, simulation.narrow_phase.results.memory[0].material.friction_coefficient,
		f32(123),
	);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(0));
	callback_state.configure_result = .Allow;
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(1));
	body_description.pose.position = {0, 8, 0};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body, &body_description), physics.Physics_Status.Ok,
	);
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(0));
	kinematic_description := test_body_description(shape, {20, 0, 0});
	kinematic_description.local_inertia = {};
	kinematic_description.activity.sleep_threshold = -1;
	kinematic, kinematic_status := physics.simulation_add_body(
		simulation, &kinematic_description,
	);
	testing.expect_value(t, kinematic_status, physics.Physics_Status.Ok);
	dynamic_description := test_body_description(shape, {20, 0, 0});
	dynamic_description.activity.sleep_threshold = -1;
	dynamic_handle, dynamic_status := physics.simulation_add_body(
		simulation, &dynamic_description,
	);
	testing.expect_value(t, dynamic_status, physics.Physics_Status.Ok);
	testing.expect(t, kinematic.value < dynamic_handle.value);
	expected_a, expected_a_status :=
		physics.collidable_reference_body(.Kinematic, kinematic);
	expected_b, expected_b_status :=
		physics.collidable_reference_body(.Dynamic, dynamic_handle);
	testing.expect_value(t, expected_a_status, physics.Physics_Status.Ok);
	testing.expect_value(t, expected_b_status, physics.Physics_Status.Ok);
	last_status = physics.simulation_timestep(simulation, 1.0 / 60.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect(t, callback_state.body_pair_allow_count > 0);
	testing.expect(t, callback_state.body_pair_configure_count > 0);
	testing.expect_value(t, callback_state.body_pair_allow_a, expected_a);
	testing.expect_value(t, callback_state.body_pair_allow_b, expected_b);
	testing.expect_value(t, callback_state.body_pair_configure_a, expected_a);
	testing.expect_value(t, callback_state.body_pair_configure_b, expected_b);
	body_pair := physics.collidable_pair_create(expected_a, expected_b);
	body_pair_index := physics.pair_cache_index_of(
		&simulation.narrow_phase.pair_cache, body_pair,
	);
	testing.expect(t, body_pair_index >= 0);
	testing.expect_value(
		t,
		simulation.narrow_phase.pair_cache.mapping.keys.memory[
		body_pair_index
	],
		body_pair,
	);
	body_pair_handle :=
		simulation.narrow_phase.pair_cache.mapping.values.memory[
		body_pair_index
	].constraint_handle;
	body_pair_capture: Contact_View_Capture;
	testing.expect_value(
		t, physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, body_pair_handle,
		capture_contact_view, &body_pair_capture,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, body_pair_capture.view.body_count, i32(2));
	testing.expect_value(
		t, body_pair_capture.view.body_handles[0], kinematic,
	);
	testing.expect_value(
		t, body_pair_capture.view.body_handles[1], dynamic_handle,
	);
	testing.expect_value(
		t, physics.simulation_remove_body(simulation, dynamic_handle),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.pair_cache_index_of(
		&simulation.narrow_phase.pair_cache, body_pair,
	),
		-1,
	);
	testing.expect_value(
		t, physics.solver_reference_state(
		&simulation.solver, body_pair_handle,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.simulation_remove_body(simulation, kinematic),
		physics.Physics_Status.Ok,
	);
	child_count_before_motion := callback_state.allow_child_count;
	callback_state.rejected_child = -1;
	passive_description := test_body_description(shape, {-4, 0, 0});
	passive_description.velocity.linear = {4, 0, 0};
	passive_description.collidable.continuity = physics.continuous_detection_passive();
	passive_description.collidable.maximum_speculative_margin = 0.25;
	_, passive_status := physics.simulation_add_body(simulation, &passive_description);
	testing.expect_value(t, passive_status, physics.Physics_Status.Ok);
	last_status = physics.simulation_timestep(simulation, 1.0);
	testing.expect_value(t, last_status, physics.Physics_Status.Ok);
	testing.expect(t, callback_state.allow_child_count > child_count_before_motion);

	test_fused_simulation = {};
	fused_simulation := &test_fused_simulation;
	fused_description := test_simulation_description(
		&pool, physics.narrow_phase_default_callbacks(),
	);
	fused_description.use_default_narrow = .Present;
	testing.expect_value(
		t, physics.simulation_create(fused_simulation, &fused_description).status,
		physics.Physics_Status.Ok,
	);
	defer physics.simulation_destroy(fused_simulation);
	testing.expect_value(
		t, physics.narrow_phase_bind_default_stored_completion(
		&fused_simulation.narrow_phase,
		{
			friction_coefficient=1,
			spring_settings=physics.spring_settings_create(30, 1),
			maximum_recovery_velocity=2,
		},
	),
		physics.Physics_Status.Ok,
	);
	box := physics.Box{0.5, 0.5, 0.5};
	box_shape, box_shape_status := physics.shape_registry_add(
		&fused_simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	testing.expect_value(t, box_shape_status, physics.Physics_Status.Ok);
	fused_body_description := test_body_description(box_shape, {0, 0.9, 0});
	fused_body_description.activity.sleep_threshold = -1;
	fused_body, fused_body_status := physics.simulation_add_body(
		fused_simulation, &fused_body_description,
	);
	testing.expect_value(t, fused_body_status, physics.Physics_Status.Ok);
	_, fused_static_status := physics.simulation_add_static(
		fused_simulation,
		&physics.Static_Description{
			pose={orientation=util.quaternion_identity()}, shape=box_shape,
		},
	);
	testing.expect_value(t, fused_static_status, physics.Physics_Status.Ok);
	fused_simulation.narrow_phase.results.memory[0].state = .Skipped;
	testing.expect_value(
		t, physics.simulation_timestep(fused_simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, fused_simulation.narrow_phase.pair_cache.mapping.count, 1,
	);
	testing.expect_value(
		t, fused_simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Skipped,
	);
	fused_narrow := &fused_simulation.narrow_phase;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Present,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Box_Direct,
	);
	testing.expect_value(t, size_of(physics.Narrow_Phase_Convex_Direct_Record), 88);
	testing.expect_value(t, size_of(physics.Narrow_Phase_Convex_Deferred_Record), 160);
	box_route_index := physics.collision_task_matrix_index(
		physics.BOX_TYPE_ID, physics.BOX_TYPE_ID,
	);
	box_route := fused_simulation.collision_tasks.routes[box_route_index];
	default_stored_completion := fused_narrow.batchers[0].stored_pair_completed;
	fused_simulation.collision_tasks.routes[box_route_index].task_id = -1;
	fused_narrow.batchers[0].stored_pair_completed =
		physics.narrow_phase_pair_completed_stored_fused;
	fused_narrow.batchers[0].state = .Ready;
	fused_narrow.state = .Stepping;
	fused_narrow.transaction_streams[0].traversal_candidate_count = 0;
	fused_narrow.transaction_statuses[0] = .Ok;
	missing_task_pose := physics.Rigid_Pose{
		orientation=util.quaternion_identity(),
	};
	testing.expect_value(
		t, physics.collision_batcher_add(
		&fused_narrow.batchers[0], box_shape, box_shape,
		missing_task_pose, missing_task_pose, 0, 0,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, fused_narrow.batchers[0].pair_count, 0);
	fused_narrow.stored_completion_state = .Missing;
	testing.expect_value(
		t, physics.collision_batcher_add(
		&fused_narrow.batchers[0], box_shape, box_shape,
		missing_task_pose, missing_task_pose, 0, 0,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(t, fused_narrow.batchers[0].pair_count, 0);
	testing.expect_value(
		t, fused_narrow.transaction_statuses[0], physics.Physics_Status.Ok,
	);
	fused_narrow.stored_completion_state = .Present;
	fused_narrow.state = .Ready;
	fused_narrow.batchers[0].stored_pair_completed = default_stored_completion;
	fused_simulation.collision_tasks.routes[box_route_index] = box_route;
	fused_narrow.stored_completion_state = .Missing;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.stored_completion_state = .Present;
	fused_narrow.callbacks.select_constraint = test_select_custom_constraint;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.callbacks.select_constraint = nil;
	stored_material := fused_narrow.stored_completion_material;
	fused_narrow.stored_completion_material.friction_coefficient = -1;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.stored_completion_material = stored_material;
	stored_completion := fused_narrow.batchers[0].stored_pair_completed;
	fused_narrow.batchers[0].stored_pair_completed = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.batchers[0].stored_pair_completed = stored_completion;
	box_task, _, box_task_status := physics.collision_task_registry_lookup(
		fused_narrow.tasks, physics.BOX_TYPE_ID, physics.BOX_TYPE_ID,
	);
	testing.expect_value(t, box_task_status, physics.Physics_Status.Ok);
	box_test := box_task.convex_test;
	box_task.convex_test = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	box_task.convex_test = box_test;
	box_wide := box_task.convex_wide_test;
	box_task.convex_wide_test = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	box_task.convex_wide_test = box_wide;
	box_wide_into := box_task.convex_wide_test_into;
	box_task.convex_wide_test_into = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	box_task.convex_wide_test_into = box_wide_into;
	stored_user_context := fused_narrow.batchers[0].user_context;
	fused_narrow.batchers[0].user_context = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.batchers[0].user_context = stored_user_context;
	stored_pair_completed := fused_narrow.batchers[0].procedures.pair_completed;
	fused_narrow.batchers[0].procedures.pair_completed = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.batchers[0].procedures.pair_completed = stored_pair_completed;
	stored_child_completed := fused_narrow.batchers[0].procedures.child_pair_completed;
	fused_narrow.batchers[0].procedures.child_pair_completed = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.batchers[0].procedures.child_pair_completed = stored_child_completed;
	stored_allow_child := fused_narrow.batchers[0].procedures.allow_child_pair;
	fused_narrow.batchers[0].procedures.allow_child_pair = nil;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	fused_narrow.batchers[0].procedures.allow_child_pair = stored_allow_child;
	accessor := &fused_narrow.accessors.records[physics.CONTACT_1_ONE_BODY_TYPE_ID];
	accessor_registration := accessor.registration;
	accessor.registration = .Missing;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	accessor.registration = accessor_registration;
	type_record := &fused_narrow.solver.registry.records[physics.CONTACT_1_ONE_BODY_TYPE_ID];
	type_registration := type_record.registration;
	type_record.registration = .Missing;
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	type_record.registration = type_registration;
	capsule_for_gate := physics.Capsule{radius=0.25, half_length=0.25};
	capsule_gate_shape, capsule_gate_status := physics.shape_registry_add(
		&fused_simulation.shapes, physics.CAPSULE_TYPE_ID, &capsule_for_gate,
	);
	testing.expect_value(t, capsule_gate_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Missing,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	testing.expect_value(
		t, physics.shape_registry_remove(&fused_simulation.shapes, capsule_gate_shape),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_fused_route_state(
		fused_narrow, 1,
	),
		physics.Reference_State.Present,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		fused_narrow, 1,
	),
		physics.Narrow_Phase_Collision_Route.Box_Direct,
	);
	testing.expect_value(
		t, physics.simulation_apply_body_description(
		fused_simulation, fused_body, &fused_body_description,
	),
		physics.Physics_Status.Ok,
	);
	fused_narrow.results.memory[0].state = .Rejected;
	testing.expect_value(
		t, physics.simulation_timestep(fused_simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, fused_narrow.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Rejected,
	);
	testing.expect_value(
		t, fused_narrow.pair_cache.mapping.count, 1,
	);
}

@(test)
caller_registered_contact_accessor_uses_dense_dispatch :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	test_pool_prepare(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	memory, allocation_error := mem.alloc(size_of(physics.Simulation), align_of(physics.Simulation));
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return;
	}
	defer testing.expect_value(t, mem.free(memory), mem.Allocator_Error.None);
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	callback_state := Narrow_Callback_State{
		margin_override=0.25, rejected_child=-1,
		allow_result=.Allow, configure_result=.Allow,
	};
	callbacks := physics.Narrow_Phase_Callbacks{
		initialize=test_narrow_initialize,
		allow=test_narrow_allow,
		allow_child=test_narrow_allow_child,
		configure=test_narrow_configure,
		configure_child=test_narrow_configure_child,
		dispose=test_narrow_dispose,
		select_constraint=test_select_custom_constraint,
		user_context=&callback_state,
	};
	description := test_simulation_description(&pool, callbacks);
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	builtin, lookup_status := physics.constraint_type_registry_lookup(
		&simulation.solver.registry,
		physics.CONTACT_1_TYPE_ID
	);
	testing.expect_value(t, lookup_status, physics.Physics_Status.Ok);
	custom_record := builtin^;
	custom_record.type_id = physics.FIRST_CALLER_CONSTRAINT_TYPE_ID;
	custom_record.validate_description = test_custom_contact_validate;
	custom_record.registration = .Missing;
	testing.expect_value(
		t, physics.solver_register_constraint_type(&simulation.solver, custom_record), physics.Physics_Status.Ok,
	);
	accessor := physics.Contact_Constraint_Accessor_Record{
		type_id=physics.FIRST_CALLER_CONSTRAINT_TYPE_ID,
		body_count=2,
		contact_count=1,
		kind=.Convex,
		build_description=test_custom_contact_build,
		impulse_offset=physics.contact_constraint_builtin_impulse_offset_convex,
	};
	testing.expect_value(
		t, physics.contact_constraint_accessors_register_custom(
		&simulation.narrow_phase.accessors, &simulation.solver, accessor,
	), physics.Physics_Status.Ok,
	);
	sphere := physics.Sphere{radius=1};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body_a_description := test_body_description(shape, {0, 0, 0});
	body_b_description := test_body_description(shape, {1.5, 0, 0});
	body_a, status_a := physics.simulation_add_body(simulation, &body_a_description);
	body_b, status_b := physics.simulation_add_body(simulation, &body_b_description);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 1);
	handle := simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
	location, resolve_status := physics.solver_resolve(&simulation.solver, handle);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.type_id, i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID));
	// the generated caller-owned contact still recomputes its own coefficients
	testing.expect_value(t, simulation.solver.contact_coefficient_offsets.memory, nil);
	testing.expect_value(t, simulation.solver.contact_coefficients.memory, nil);
	contact_capture: Contact_View_Capture;
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle,
		capture_contact_view, &contact_capture,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, contact_capture.call_count, i32(1));
	testing.expect_value(
		t, contact_capture.view.type_id,
		i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID),
	);
	testing.expect_value(
		t, contact_capture.view.kind,
		physics.Contact_Constraint_Kind.Convex,
	);
	testing.expect_value(t, contact_capture.view.body_count, i32(2));
	testing.expect_value(t, contact_capture.view.contact_count, i32(1));
	testing.expect_value(t, contact_capture.view.body_handles[0], body_a);
	testing.expect_value(t, contact_capture.view.body_handles[1], body_b);
	testing.expect_value(
		t, contact_capture.view.prestep_size,
		i32(size_of(physics.Contact_1)),
	);
	custom_prestep := (^physics.Contact_1)(
		&contact_capture.view.prestep[0],
	);
	testing.expect_value(
		t, custom_prestep.material.friction_coefficient, f32(0.7),
	);
	custom_type_record, custom_type_status :=
		physics.constraint_type_registry_lookup(
		&simulation.solver.registry,
		physics.FIRST_CALLER_CONSTRAINT_TYPE_ID,
	);
	testing.expect_value(t, custom_type_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, contact_capture.view.impulse_count,
		custom_type_record.impulse_bundle_size /
		i32(size_of(util.F32x8)),
	);
	expect_contact_view_impulses_match_solver(
		t, simulation, handle, &contact_capture.view,
	);
	active_impulses := contact_capture.view.impulses;
	active_impulse_count := contact_capture.view.impulse_count;
	body_a_sleep, body_a_sleep_status := physics.bodies_get_description(
		&simulation.bodies, body_a,
	);
	body_b_sleep, body_b_sleep_status := physics.bodies_get_description(
		&simulation.bodies, body_b,
	);
	testing.expect_value(t, body_a_sleep_status, physics.Physics_Status.Ok);
	testing.expect_value(t, body_b_sleep_status, physics.Physics_Status.Ok);
	body_a_sleep.velocity = {};
	body_b_sleep.velocity = {};
	body_a_sleep.activity = {
		sleep_threshold=100,
		minimum_timestep_count_under_threshold=1,
	};
	body_b_sleep.activity = body_a_sleep.activity;
	testing.expect_value(
		t,
		physics.simulation_apply_body_description(
		simulation, body_a, &body_a_sleep,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t,
		physics.simulation_apply_body_description(
		simulation, body_b, &body_b_sleep,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	active_location, active_status := physics.bodies_resolve(
		&simulation.bodies, body_a,
	);
	testing.expect_value(t, active_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, active_location.set_index,
		i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	pre_sleep_capture: Contact_View_Capture;
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle,
		capture_contact_view, &pre_sleep_capture,
	),
		physics.Physics_Status.Ok,
	);
	active_impulses = pre_sleep_capture.view.impulses;
	active_impulse_count = pre_sleep_capture.view.impulse_count;
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 1.0 / 60.0),
		physics.Physics_Status.Ok,
	);
	sleeping_location, sleeping_status := physics.bodies_resolve(
		&simulation.bodies, body_a,
	);
	testing.expect_value(t, sleeping_status, physics.Physics_Status.Ok);
	testing.expect(t, sleeping_location.set_index > 0);
	testing.expect_value(
		t,
		physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle,
		capture_contact_view, &contact_capture,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, contact_capture.call_count, i32(2));
	testing.expect_value(
		t, contact_capture.view.type_id,
		i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID),
	);
	testing.expect_value(
		t, contact_capture.view.impulse_count, active_impulse_count,
	);
	for impulse_index in 0 ..< int(active_impulse_count)
	{
		testing.expect_value(
			t, contact_capture.view.impulses[impulse_index],
			active_impulses[impulse_index],
		);
	}
	testing.expect_value(t, contact_capture.view.body_handles[0], body_a);
	testing.expect_value(t, contact_capture.view.body_handles[1], body_b);
	// inactive capture restores authored data and impulses before any new step
	sleeping_authored := (^physics.Contact_1)(&contact_capture.view.prestep[0])^;
	awake_body, awake_body_status := physics.bodies_get_description(&simulation.bodies, body_a);
	testing.expect_value(t, awake_body_status, physics.Physics_Status.Ok);
	awake_body.velocity = {linear={0.05, 0, 0}, angular={0.01, 0.02, 0.03}};
	awake_body.activity = {sleep_threshold=-1, minimum_timestep_count_under_threshold=32};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body_a, &awake_body),
		physics.Physics_Status.Ok,
	);
	awakened_handles := [2]physics.Body_Handle{body_a, body_b};
	for body_handle in awakened_handles
	{
		awake_location, awake_status := physics.bodies_resolve(&simulation.bodies, body_handle);
		testing.expect_value(t, awake_status, physics.Physics_Status.Ok);
		testing.expect_value(t, awake_location.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	}
	awake_authored: physics.Contact_1;
	testing.expect_value(
		t, physics.solver_get_description_raw(
		&simulation.solver, handle, physics.FIRST_CALLER_CONSTRAINT_TYPE_ID,
		&awake_authored, size_of(awake_authored),
	), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, awake_authored.contact_0, sleeping_authored.contact_0);
	testing.expect_value(t, awake_authored.offset_b, sleeping_authored.offset_b);
	testing.expect_value(t, awake_authored.normal, sleeping_authored.normal);
	testing.expect_value(t, awake_authored.material, sleeping_authored.material);
	awake_capture: Contact_View_Capture;
	testing.expect_value(
		t, physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle, capture_contact_view, &awake_capture,
	), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, awake_capture.call_count, i32(1));
	testing.expect_value(t, awake_capture.view.type_id, i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID));
	testing.expect_value(t, awake_capture.view.body_handles[0], body_a);
	testing.expect_value(t, awake_capture.view.body_handles[1], body_b);
	testing.expect_value(t, awake_capture.view.impulse_count, active_impulse_count);
	for impulse_index in 0 ..< int(active_impulse_count)
	{
		testing.expect_value(t, awake_capture.view.impulses[impulse_index], active_impulses[impulse_index]);
	}
	expect_contact_view_impulses_match_solver(t, simulation, handle, &awake_capture.view);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_reference_state(&simulation.solver, handle), physics.Reference_State.Present);
	testing.expect_value(
		t, physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle, capture_contact_view, &awake_capture,
	), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, awake_capture.call_count, i32(2));
	testing.expect_value(t, awake_capture.view.type_id, i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID));
	expect_contact_view_impulses_match_solver(t, simulation, handle, &awake_capture.view);
	for impulse_index in 0 ..< int(awake_capture.view.impulse_count)
	{
		testing.expect_value(t, util.math_check_f32(awake_capture.view.impulses[impulse_index]), util.Math_Check_Status.Ok);
	}
	// mix an authored builtin with the existing generated caller contact. the
	// builtin receives a cache while the caller record keeps its own layout
	authored_body_description: physics.Body_Description = test_body_description(shape, {50, 10, 0});
	authored_body, authored_body_status := physics.simulation_add_body(simulation, &authored_body_description);
	testing.expect_value(t, authored_body_status, physics.Physics_Status.Ok);
	authored_handles: [4]physics.Body_Handle = {authored_body, {}, {}, {}};
	authored_description: physics.Contact_1_One_Body = {
		contact_0={penetration_depth=0.05}, normal={0, 1, 0}, material=awake_authored.material,
	};
	authored_handle, authored_status := physics.simulation_add_constraint(simulation, &authored_handles, &authored_description);
	testing.expect_value(t, authored_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_solve_substep(&simulation.solver, 1.0 / 60.0, 4, .First), physics.Physics_Status.Ok);
	testing.expect(t, simulation.solver.contact_coefficients.length > 0);
	authored_readback: physics.Contact_1_One_Body;
	testing.expect_value(t, physics.simulation_get_constraint_description(simulation, authored_handle, &authored_readback), physics.Physics_Status.Ok);
	testing.expect_value(t, authored_readback, authored_description);
	for batch_index in 0 ..< int(simulation.solver.active_set.batch_count)
	{
		batch: ^physics.Constraint_Batch = &simulation.solver.active_set.batches.memory[batch_index];
		for block_index in int(batch.work_block_start) ..< int(batch.work_block_start + batch.work_block_count)
		{
			block: ^physics.Solver_Work_Block = &simulation.solver.work_blocks.memory[block_index];
			type_id: i32 = batch.type_batches.memory[block.type_batch_index].type_id;
			offset: i32 = simulation.solver.contact_coefficient_offsets.memory[block_index];
			if type_id == physics.FIRST_CALLER_CONSTRAINT_TYPE_ID
			{
				testing.expect_value(t, offset, i32(-1));
			}
			else
			{
				testing.expect_value(t, type_id, i32(physics.CONTACT_1_ONE_BODY_TYPE_ID));
				testing.expect(t, offset >= 0 && offset % 2 == 0);
			}
		}
	}
	testing.expect_value(t, physics.solver_reference_state(&simulation.solver, handle), physics.Reference_State.Present);
}

transaction_stream_write_reservation :: proc(
	t: ^testing.T, narrow: ^physics.Narrow_Phase,
	reservation: physics.Narrow_Phase_Transaction_Reservation,
	write_count: int, sequence: ^int, expected: ^[128]i32,
	expected_count: ^int,
)
{
	for offset in 0 ..< write_count
	{
		transaction_index, index_status :=
			physics.narrow_phase_transaction_reservation_index(
			reservation, offset,
		);
		if !testing.expect_value(t, index_status, physics.Physics_Status.Ok)
		{
			return;
		}
		if !testing.expect(t, transaction_index >= 0 && transaction_index < int(narrow.transaction_headers.length))
		{
			return;
		}
		pair_a := physics.Collidable_Reference{packed=u32(4096 - sequence^)};
		pair_b := physics.Collidable_Reference{packed=u32(8192 + sequence^)};
		narrow.transaction_headers.memory[transaction_index] = {
			pair={a=pair_a, b=pair_b},
			payload_index=i32(transaction_index),
			kind=.Pending,
		};
		expected[expected_count^] = i32(transaction_index);
		expected_count^ += 1;
		sequence^ += 1;
	}
}

@(test)
worker_local_transaction_streams_preserve_mode_order_and_reuse_overflow_tail :: proc(
	t: ^testing.T,
)
{
	pool: util.Buffer_Pool;
	test_pool_prepare(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	callbacks := physics.narrow_phase_default_callbacks();
	description := test_simulation_description(&pool, callbacks);
	description.use_default_narrow = .Present;
	description.allocation_sizes.workers = 3;
	description.allocation_sizes.collision_child_pairs = 3;
	memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return;
	}
	defer testing.expect_value(t, mem.free(memory), mem.Allocator_Error.None);
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(simulation);
	narrow := &simulation.narrow_phase;
	testing.expect_value(
		t, size_of(physics.Narrow_Phase_Fused_Deferred_Record), 8,
	);
	worker_slice_end := narrow.pending_capacity_per_worker;
	testing.expect(t, worker_slice_end < int(narrow.results.length));
	narrow.results.memory[worker_slice_end].state = .Accepted;
	deferred_records := ([^]physics.Narrow_Phase_Fused_Deferred_Record)(
		rawptr(&narrow.results.memory[0])
	);
	for record_index in 0 ..< narrow.pending_capacity_per_worker
	{
		deferred_records[record_index] = {
			pair_slot=i32(record_index), mapping_index=i32(record_index - 1),
		};
	}
	for record_index in 0 ..< narrow.pending_capacity_per_worker
	{
		testing.expect_value(
			t, deferred_records[record_index].pair_slot, i32(record_index),
		);
		testing.expect_value(
			t, deferred_records[record_index].mapping_index, i32(record_index - 1),
		);
	}
	testing.expect_value(
		t, narrow.results.memory[worker_slice_end].state,
		physics.Narrow_Phase_Result_State.Accepted,
	);
	deferred_records[0] = {pair_slot=7, mapping_index=-1};
	testing.expect_value(t, deferred_records[0].pair_slot, i32(7));
	testing.expect_value(t, deferred_records[0].mapping_index, i32(-1));
	pending_prepared_header := physics.Narrow_Phase_Transaction_Header{
		kind=.Pending_Prepared,
	};
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.None), 0);
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.Pending), 1);
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.Update), 2);
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.Add), 3);
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.Replace), 4);
	testing.expect_value(t, int(physics.Narrow_Phase_Transaction_Kind.Remove), 5);
	testing.expect_value(
		t, int(physics.Narrow_Phase_Transaction_Kind.Pending_Prepared), 6,
	);
	pending_prepared_counts: physics.Narrow_Phase_Preflight_Worker_Counts;
	testing.expect_value(
		t, physics.narrow_phase_count_prepared_transaction(
		&pending_prepared_header, &pending_prepared_counts,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(
		t, narrow.transaction_stream_primary_capacity,
		physics.NARROW_PHASE_TRANSACTION_STREAM_MIN_PAGE_CAPACITY,
	);
	testing.expect_value(
		t, physics.narrow_phase_reset_transaction_streams(narrow, 3),
		physics.Physics_Status.Ok,
	);
	expected: [128]i32;
	expected_count := 0;
	sequence := 0;
	reservation_a, status_a := physics.narrow_phase_transaction_stream_reserve(
		narrow, 0, 20,
	);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	transaction_stream_write_reservation(
		t, narrow, reservation_a, 20, &sequence, &expected, &expected_count,
	);
	testing.expect_value(t, narrow.transaction_overflow_page_cursor, i32(1));
	reservation_b, status_b := physics.narrow_phase_transaction_stream_reserve(
		narrow, 0, 5,
	);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	transaction_stream_write_reservation(
		t, narrow, reservation_b, 5, &sequence, &expected, &expected_count,
	);
	testing.expect_value(t, narrow.transaction_overflow_page_cursor, i32(1));
	reservation_c, status_c := physics.narrow_phase_transaction_stream_reserve(
		narrow, 0, 10,
	);
	testing.expect_value(t, status_c, physics.Physics_Status.Ok);
	transaction_stream_write_reservation(
		t, narrow, reservation_c, 10, &sequence, &expected, &expected_count,
	);
	testing.expect_value(t, narrow.transaction_overflow_page_cursor, i32(2));
	reservation_d, status_d := physics.narrow_phase_transaction_stream_reserve(
		narrow, 1, 18,
	);
	testing.expect_value(t, status_d, physics.Physics_Status.Ok);
	transaction_stream_write_reservation(
		t, narrow, reservation_d, 18, &sequence, &expected, &expected_count,
	);
	testing.expect_value(t, narrow.transaction_overflow_page_cursor, i32(3));
	reservation_e, status_e := physics.narrow_phase_transaction_stream_reserve(
		narrow, 2, 7,
	);
	testing.expect_value(t, status_e, physics.Physics_Status.Ok);
	transaction_stream_write_reservation(
		t, narrow, reservation_e, 7, &sequence, &expected, &expected_count,
	);
	transaction_count, order_status := physics.narrow_phase_prepare_transaction_order(
		narrow, 3,
	);
	testing.expect_value(t, order_status, physics.Physics_Status.Ok);
	testing.expect_value(t, transaction_count, expected_count);
	for index in 0 ..< expected_count
	{
		testing.expect_value(
			t, narrow.transaction_order.memory[index], expected[index],
		);
	}
}

Box_Sphere_Allow_Trace :: struct
{
	pairs:    [64]physics.Broad_Phase_Pair,
	workers:  [64]int,
	count:    int,
	rejected: physics.Collidable_Reference,
}
box_sphere_oracle_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	speculative_margin: ^f32,
) -> physics.Collision_Testing_State
{
	trace := (^Box_Sphere_Allow_Trace)(user_context);
	if trace.count < len(trace.pairs)
	{
		trace.pairs[trace.count] = {a, b};
		trace.workers[trace.count] = worker_index;
	}
	trace.count += 1;
	speculative_margin^ = 0;
	if a == trace.rejected || b == trace.rejected
	{
		return .Reject;
	}
	return .Allow;
}
verify_box_sphere_fixture :: proc(
	t: ^testing.T, main_kind, main_count: int,
	continuity_a, continuity_b: physics.Continuous_Detection_Mode,
	fast_ccd: physics.Reference_State, pending_capacity: int = 64,
)
{
	world_pools := new([2]util.Buffer_Pool);
	world_simulations := new([2]physics.Simulation);
	defer free(world_pools);
	defer free(world_simulations);
	created_worlds, prepared_pools := 0, 0;
	defer
	{
		for route_case in 0 ..< created_worlds
		{
			testing.expect_value(t, physics.simulation_destroy(&world_simulations[route_case]), physics.Physics_Status.Ok);
		}
		for route_case in 0 ..< prepared_pools
		{
			testing.expect_value(t, util.buffer_pool_dispose(&world_pools[route_case]), util.Memory_Status.Ok);
		}
	}
	traces: [2]Box_Sphere_Allow_Trace;
	handles: [2][64]physics.Body_Handle;
	pair_count := main_count + 4;
	for route_case in 0 ..< 2
	{
		pool := &world_pools[route_case];
		pool^ = {};
		test_pool_prepare(t, pool);
		prepared_pools += 1;
		world := &world_simulations[route_case];
		world^ = {};
		callbacks := physics.narrow_phase_default_callbacks();
		callbacks.allow = box_sphere_oracle_allow;
		callbacks.user_context = &traces[route_case];
		description := test_simulation_description(pool, callbacks);
		description.allocation_sizes.bodies = 64;
		description.allocation_sizes.statics = 64;
		description.allocation_sizes.pending_pairs_per_worker = i32(pending_capacity);
		if !testing.expect_value(t, physics.simulation_create(world, &description).status, physics.Physics_Status.Ok)
		{
			return;
		}
		created_worlds += 1;
		testing.expect_value(t, physics.narrow_phase_bind_default_stored_completion(&world.narrow_phase, {
			friction_coefficient=0.7, spring_settings=physics.spring_settings_create(30, 1), maximum_recovery_velocity=2,
		}), physics.Physics_Status.Ok);
		box := physics.Box{0.5, 0.5, 0.5};
		sphere := physics.Sphere{radius=0.5};
		box_shape, box_status := physics.shape_registry_add(&world.shapes, physics.BOX_TYPE_ID, &box);
		sphere_shape, sphere_status := physics.shape_registry_add(&world.shapes, physics.SPHERE_TYPE_ID, &sphere);
		testing.expect_value(t, box_status, physics.Physics_Status.Ok);
		testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
		if route_case == 1
		{
			unused_capsule := physics.Capsule{radius=0.125, half_length=0.125};
			_, status := physics.shape_registry_add(&world.shapes, physics.CAPSULE_TYPE_ID, &unused_capsule);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		for pair_index in 0 ..< pair_count
		{
			kind := main_kind;
			x := f32(pair_index * 8);
			if pair_index >= main_count
			{
				extra := pair_index - main_count;
				x = f32(4 + extra * 8);
				kind = (main_kind + extra + 1) % 3;
				if extra == 2
				{
					kind = 1;
					if main_kind == 1
					{
						kind = 2;
					}
				}
			}
			shape_a, shape_b := box_shape, box_shape;
			if kind == 1
			{
				shape_a, shape_b = sphere_shape, sphere_shape;
			}
			else if kind == 2
			{
				shape_a, shape_b = sphere_shape, box_shape;
				if (pair_index + main_count) % 2 != 0
				{
					shape_a, shape_b = shape_b, shape_a;
				}
			}
			body := test_body_description(shape_a, {x, 0.9, 0});
			body.activity.sleep_threshold = -1;
			body.pose.orientation = util.quaternion_from_axis_angle({0, 0, 1}, f32(pair_index % 3) * 0.1);
			body.collidable.continuity = {
				mode=continuity_a, minimum_sweep_timestep=1e-4, sweep_convergence_threshold=1e-4,
			};
			if pair_index == main_count + 2
			{
				body.pose.position = {x + 0.95, 0.95, 0};
			}
			if fast_ccd == .Present && pair_index == 0
			{
				body.pose.position.y = 3;
				body.velocity.linear = {0, -240, 0};
			}
			handle, body_status := physics.simulation_add_body(world, &body);
			testing.expect_value(t, body_status, physics.Physics_Status.Ok);
			handles[route_case][pair_index] = handle;
			if pair_index == main_count + 3
			{
				rejected, status := physics.collidable_reference_create(.Dynamic, int(handle.value));
				testing.expect_value(t, status, physics.Physics_Status.Ok);
				traces[route_case].rejected = rejected;
			}
			if pair_index == 0
			{
				other_body := test_body_description(shape_b, {x, 0, 0});
				other_body.activity.sleep_threshold = -1;
				other_body.collidable.continuity = {mode=continuity_b, minimum_sweep_timestep=1e-4, sweep_convergence_threshold=1e-4};
				other_handle, other_status := physics.simulation_add_body(world, &other_body);
				testing.expect_value(t, other_status, physics.Physics_Status.Ok);
				handles[route_case][pair_count] = other_handle;
			}
			else
			{
				_, static_status := physics.simulation_add_static(world, &physics.Static_Description{
					pose={position={x, 0, 0}, orientation=util.quaternion_identity()},
					shape=shape_b,
					continuity={mode=continuity_b, minimum_sweep_timestep=1e-4, sweep_convergence_threshold=1e-4},
				});
				testing.expect_value(t, static_status, physics.Physics_Status.Ok);
			}
		}
	}
	a, b := &world_simulations[0], &world_simulations[1];
	transition_shape: physics.Typed_Index;
	for step_index in 0 ..< 3
	{
		expected_route: physics.Narrow_Phase_Collision_Route = .Box_Sphere_Direct;
		if main_kind == 0 && main_count == 1 && continuity_a == .Discrete &&
			continuity_b == .Discrete && fast_ccd == .Missing && step_index > 0
		{
			// keep the independent world on its predecessor while the tested world
			// changes route at the next preparation boundary and then changes back
			if step_index == 1
			{
				capsule: physics.Capsule = {radius=0.125, half_length=0.125};
				shape_status: physics.Physics_Status;
				transition_shape, shape_status = physics.shape_registry_add(&a.shapes, physics.CAPSULE_TYPE_ID, &capsule);
				testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
				expected_route = .Predecessor;
			}
			else
			{
				testing.expect_value(t, physics.shape_registry_remove(&a.shapes, transition_shape), physics.Physics_Status.Ok);
			}
		}
		for route_case in 0 ..< 2
		{
			world := &world_simulations[route_case];
			traces[route_case].count = 0;
			for index in 0 ..< world.narrow_phase.pending_capacity_per_worker
			{
				world.narrow_phase.continuations.memory[index] = {
					t=0.375, worker_index=-77, kind=.Continuous,
					relative_linear_velocity={1, 2, 3}, angular_a={4, 5, 6}, angular_b={7, 8, 9},
				};
			}
			testing.expect_value(t, physics.simulation_timestep(world, 1.0 / 60.0), physics.Physics_Status.Ok);
		}
		actual_route: physics.Narrow_Phase_Collision_Route = physics.narrow_phase_collision_route_state(&a.narrow_phase, 1);
		if actual_route == .Predecessor
		{
			actual_route = physics.narrow_phase_box_sphere_route_state(&a.narrow_phase, 1);
		}
		reference_route: physics.Narrow_Phase_Collision_Route = physics.narrow_phase_collision_route_state(&b.narrow_phase, 1);
		if reference_route == .Predecessor
		{
			reference_route = physics.narrow_phase_box_sphere_route_state(&b.narrow_phase, 1);
		}
		testing.expect_value(t, actual_route, expected_route);
		testing.expect_value(t, reference_route, physics.Narrow_Phase_Collision_Route.Predecessor);
		testing.expect_value(t, traces[0].count, traces[1].count);
		if step_index == 0
		{
			testing.expect_value(t, traces[0].count, pair_count);
		}
		testing.expect(t, traces[0].count <= len(traces[0].pairs));
		for index in 0 ..< min(traces[0].count, len(traces[0].pairs))
		{
			testing.expect_value(t, traces[0].pairs[index], traces[1].pairs[index]);
			testing.expect_value(t, traces[0].workers[index], traces[1].workers[index]);
		}
		for index in 0 ..< min(traces[0].count, pending_capacity)
		{
			testing.expect_value(t, a.narrow_phase.continuations.memory[index], b.narrow_phase.continuations.memory[index]);
		}
		if fast_ccd == .Present && step_index == 0
		{
			continuous_count := 0;
			for index in 0 ..< min(traces[0].count, a.narrow_phase.pending_capacity_per_worker)
			{
				if a.narrow_phase.continuations.memory[index].kind == .Continuous &&
					a.narrow_phase.continuations.memory[index].worker_index != -77
				{
					continuous_count += 1;
				}
			}
			testing.expect(t, continuous_count > 0);
		}
		testing.expect_value(t, a.narrow_phase.last_stage, b.narrow_phase.last_stage);
		testing.expect_value(t, a.narrow_phase.pair_cache.mapping.count, b.narrow_phase.pair_cache.mapping.count);
		testing.expect_value(t, a.solver.active_set.constraint_count, b.solver.active_set.constraint_count);
		for pair_index in 0 ..< a.narrow_phase.pair_cache.mapping.count
		{
			key := a.narrow_phase.pair_cache.mapping.keys.memory[pair_index];
			other_index := physics.pair_cache_index_of(&b.narrow_phase.pair_cache, key);
			if !testing.expect(t, other_index >= 0)
			{
				continue;
			}
			cache_a := &a.narrow_phase.pair_cache.mapping.values.memory[pair_index];
			cache_b := &b.narrow_phase.pair_cache.mapping.values.memory[other_index];
			testing.expect_value(t, cache_a.constraint_handle, cache_b.constraint_handle);
			testing.expect_value(t, cache_a.feature_ids, cache_b.feature_ids);
			testing.expect_value(t, a.narrow_phase.pair_cache.pair_freshness.memory[pair_index], b.narrow_phase.pair_cache.pair_freshness.memory[other_index]);
			capture_a, capture_b: Contact_View_Capture;
			testing.expect_value(t, physics.narrow_phase_try_extract_solver_contact_data(&a.narrow_phase, cache_a.constraint_handle, capture_contact_view, &capture_a), physics.Physics_Status.Ok);
			testing.expect_value(t, physics.narrow_phase_try_extract_solver_contact_data(&b.narrow_phase, cache_b.constraint_handle, capture_contact_view, &capture_b), physics.Physics_Status.Ok);
			testing.expect_value(t, capture_a.call_count, i32(1));
			testing.expect_value(t, capture_b.call_count, i32(1));
			testing.expect_value(t, capture_a.view, capture_b.view);
		}
		for body_index in 0 ..< pair_count + 1
		{
			location_a := a.bodies.handle_to_location.memory[handles[0][body_index].value];
			location_b := b.bodies.handle_to_location.memory[handles[1][body_index].value];
			motion_a := &a.bodies.sets.memory[location_a.set_index].dynamics_state.memory[location_a.index].motion;
			motion_b := &b.bodies.sets.memory[location_b.set_index].dynamics_state.memory[location_b.index].motion;
			testing.expect_value(t, motion_a.pose.position, motion_b.pose.position);
			testing.expect_value(t, motion_a.pose.orientation, motion_b.pose.orientation);
			testing.expect_value(t, motion_a.velocity.linear, motion_b.velocity.linear);
			testing.expect_value(t, motion_a.velocity.angular, motion_b.velocity.angular);
		}
	}
	if main_kind == 0 && main_count == 1 && continuity_a == .Discrete && continuity_b == .Discrete && fast_ccd == .Missing
	{
		verify_box_sphere_admission_guards(t, a, &handles[0], pair_count + 1);
	}
}
@(test)
box_sphere_compact_contacts_match_predecessor :: proc(t: ^testing.T)
{
	counts := [?]int{1, 2, 3, 4, 5, 6, 7, 8, 32, 33};
	for task_case in 0 ..< 3
	{
		for count in counts
		{
			verify_box_sphere_fixture(t, task_case, count, .Discrete, .Discrete, .Missing);
		}
	}
	modes := [?]physics.Continuous_Detection_Mode{.Discrete, .Passive, .Continuous};
	for mode_a in modes
	{
		for mode_b in modes
		{
			verify_box_sphere_fixture(t, 2, 2, mode_a, mode_b, .Missing);
		}
	}
	verify_box_sphere_fixture(t, 2, 1, .Continuous, .Discrete, .Present);
	verify_box_sphere_fixture(t, 0, 33, .Discrete, .Discrete, .Missing, 16);
}

box_sphere_override_calls: i32;
box_sphere_override_wide_into :: proc "contextless" (
	bundle: ^physics.Collision_Convex_Wide_Bundle, shapes: ^physics.Shape_Registry,
	result: ^physics.Collision_Wide_Manifold_Result,
) -> physics.Physics_Status
{
	_, _, _ = bundle, shapes, result;
	box_sphere_override_calls += 1;
	return .Invalid_Description;
}
verify_box_sphere_admission_guards :: proc(
	t: ^testing.T, world: ^physics.Simulation, handles: ^[64]physics.Body_Handle, body_count: int,
)
{
	narrow := &world.narrow_phase;
	for task_case in 0 ..< 3
	{
		type_a, type_b := physics.SPHERE_TYPE_ID, physics.SPHERE_TYPE_ID;
		if task_case == 1
		{
			type_b = physics.BOX_TYPE_ID;
		}
		else if task_case == 2
		{
			type_a, type_b = physics.BOX_TYPE_ID, physics.BOX_TYPE_ID;
		}
		route_index := physics.collision_task_matrix_index(type_a, type_b);
		reference := world.collision_tasks.routes[route_index];
		task := &world.collision_tasks.tasks[reference.task_id];
		original_task := task^;
		for guard_case in 0 ..< 8
		{
			switch guard_case
			{
				case 0: task.convex_test = nil;
				case 1: task.convex_wide_test = nil;
				case 2: task.convex_wide_test_into = nil;
				case 3: task.batch_size = 16;
				case 4: task.pair_type = .Standard;
				case 5: task.kind = .Convex_Compound;
				case 6: task.capabilities = {.Convex_Result};
				case 7: task.shape_type_a = physics.CAPSULE_TYPE_ID;
			}
			testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
			task^ = original_task;
			testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Box_Sphere_Direct);
		}
		world.collision_tasks.routes[route_index].order = .Flipped;
		testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
		world.collision_tasks.routes[route_index] = reference;
		world.collision_tasks.routes[route_index].batch_size = 16;
		testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
		world.collision_tasks.routes[route_index] = reference;
		if type_a != type_b
		{
			reverse_index := physics.collision_task_matrix_index(type_b, type_a);
			reverse := world.collision_tasks.routes[reverse_index];
			world.collision_tasks.routes[reverse_index].order = .Expected;
			testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
			world.collision_tasks.routes[reverse_index] = reverse;
			world.collision_tasks.routes[reverse_index].expected_first_type_id = physics.BOX_TYPE_ID;
			testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
			world.collision_tasks.routes[reverse_index] = reverse;
		}
	}
	original_stored_state := narrow.stored_completion_state;
	narrow.stored_completion_state = .Missing;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	narrow.stored_completion_state = original_stored_state;
	original_select := narrow.callbacks.select_constraint;
	narrow.callbacks.select_constraint = test_select_custom_constraint;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	narrow.callbacks.select_constraint = original_select;
	original_material := narrow.stored_completion_material;
	narrow.stored_completion_material.friction_coefficient = -1;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	narrow.stored_completion_material = original_material;
	for bodies in 1 ..= 2
	{
		for contacts in 1 ..= 4
		{
			type_id := physics.contact_constraint_type_id(bodies, contacts, .Convex);
			accessor := &narrow.accessors.records[type_id];
			original_offset := accessor.impulse_offset;
			accessor.impulse_offset = nil;
			testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
			accessor.impulse_offset = original_offset;
		}
	}
	batcher := &narrow.batchers[0];
	original_completion := batcher.stored_pair_completed;
	batcher.stored_pair_completed = physics.narrow_phase_pair_completed_stored_fused;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	batcher.stored_pair_completed = original_completion;
	original_child := batcher.procedures.allow_child_pair;
	batcher.procedures.allow_child_pair = nil;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	batcher.procedures.allow_child_pair = original_child;
	original_user_context := batcher.user_context;
	batcher.user_context = nil;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	batcher.user_context = original_user_context;
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Box_Sphere_Direct);
	for task_case in 0 ..< 3
	{
		for body_index in 0 ..< body_count
		{
			location := world.bodies.handle_to_location.memory[handles[body_index].value];
			set := &world.bodies.sets.memory[location.set_index];
			motion := &set.dynamics_state.memory[location.index].motion;
			position := util.Vector3{motion.pose.position.x, 0.9, 0};
			if body_index == body_count - 1
			{
				position.y = 0;
			}
			body := test_body_description(set.collidables.memory[location.index].shape, position);
			body.activity.sleep_threshold = -1;
			testing.expect_value(t, physics.simulation_apply_body_description(world, handles[body_index], &body), physics.Physics_Status.Ok);
		}
		type_a, type_b := physics.SPHERE_TYPE_ID, physics.SPHERE_TYPE_ID;
		if task_case == 1
		{
			type_b = physics.BOX_TYPE_ID;
		}
		else if task_case == 2
		{
			type_a, type_b = physics.BOX_TYPE_ID, physics.BOX_TYPE_ID;
		}
		reference := world.collision_tasks.routes[physics.collision_task_matrix_index(type_a, type_b)];
		task := &world.collision_tasks.tasks[reference.task_id];
		original_into := task.convex_wide_test_into;
		task.convex_wide_test_into = box_sphere_override_wide_into;
		box_sphere_override_calls = 0;
		testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
		testing.expect_value(t, physics.simulation_timestep(world, 1.0 / 60.0), physics.Physics_Status.Invalid_Description);
		testing.expect(t, box_sphere_override_calls > 0);
		task.convex_wide_test_into = original_into;
		testing.expect_value(t, physics.simulation_timestep(world, 1.0 / 60.0), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Box_Sphere_Direct);
	}
	testing.expect_value(t, physics.narrow_phase_box_sphere_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Box_Sphere_Direct);
}
