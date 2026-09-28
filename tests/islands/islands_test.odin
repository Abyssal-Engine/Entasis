package island_tests

import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 256), util.Memory_Status.Ok);
	for power in 0 ..= 22
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 524288, power), util.Memory_Status.Ok);
	}
}

allocate_simulation :: proc(t: ^testing.T) -> ^physics.Simulation
{
	memory, allocation_error := mem.alloc(size_of(physics.Simulation), align_of(physics.Simulation));
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return nil;
	}
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	return simulation;
}

small_description :: proc(pool: ^util.Buffer_Pool) -> physics.Simulation_Create_Description
{
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=32, statics=16, inactive_body_sets=8, shapes_per_type=8,
		constraints=64, constraint_batches=8, initial_constraints_per_type_batch=8,
		minimum_constraints_per_body=8, broad_phase_candidates=128, pairs=128,
		inactive_pairs=64, pending_pairs_per_worker=64, workers=1,
	};
	description.default_pose_context.gravity = {};
	return description;
}

sleeping_body :: proc(shape: physics.Typed_Index, position: util.Vector3) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position=position},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=0.1},
		activity={sleep_threshold=1, minimum_timestep_count_under_threshold=1},
	};
}

Island_Dispatch_State :: struct
{
	inner:                   ^util.Thread_Dispatcher_Boundary,
	traversal_maximum:       int,
	constraint_gather_maximum: int,
	body_gather_maximum:      int,
	awakener_phase_one_maximum: int,
	awakener_phase_two_maximum: int,
}

island_recording_dispatch :: proc "contextless" (
	dispatcher: ^util.Thread_Dispatcher_Boundary, worker: util.Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> util.Threading_Status
{
	if dispatcher == nil || dispatcher.dispatcher == nil
	{
		return .Invalid_Argument;
	}
	state := (^Island_Dispatch_State)(dispatcher.dispatcher);
	if worker == physics.island_sleeper_traversal_worker
	{
		state.traversal_maximum = max(state.traversal_maximum, maximum_worker_count);
	}
	else if worker == physics.island_sleeper_constraint_gather_worker
	{
		state.constraint_gather_maximum = max(state.constraint_gather_maximum, maximum_worker_count);
	}
	else if worker == physics.island_sleeper_body_gather_worker
	{
		state.body_gather_maximum = max(state.body_gather_maximum, maximum_worker_count);
	}
	else if worker == physics.island_awakener_phase_one_worker
	{
		state.awakener_phase_one_maximum = max(state.awakener_phase_one_maximum, maximum_worker_count);
	}
	else if worker == physics.island_awakener_phase_two_worker
	{
		state.awakener_phase_two_maximum = max(state.awakener_phase_two_maximum, maximum_worker_count);
	}
	return state.inner.dispatch(state.inner, worker, maximum_worker_count, unmanaged_context);
}

island_recording_worker_pool :: proc (
	dispatcher: ^util.Thread_Dispatcher_Boundary, worker_index: int,
) -> (^util.Buffer_Pool, util.Threading_Status)
{
	if dispatcher == nil || dispatcher.dispatcher == nil
	{
		return nil, .Invalid_Argument;
	}
	state := (^Island_Dispatch_State)(dispatcher.dispatcher);
	return state.inner.worker_pool(state.inner, worker_index);
}

@(test)
filtered_sleep_candidate_selection_preserves_order_and_schedule :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher, 2, 65536), util.Threading_Status.Ok,
	);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	simulation: ^physics.Simulation = allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
	description: physics.Simulation_Create_Description = small_description(&pool);
	description.allocation_sizes.workers = 2;
	testing.expect_value(
		t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok,
	);
	defer physics.simulation_destroy(simulation);
	simulation.sleeper.schedule_offset = 9;
	empty_count, empty_status := physics.island_sleeper_update_filtered_candidates(
		&simulation.sleeper,
	);
	testing.expect_value(t, empty_status, physics.Physics_Status.Ok);
	testing.expect_value(t, empty_count, 0);
	testing.expect_value(t, simulation.sleeper.schedule_offset, 9);

	sphere: physics.Sphere = {radius=0.25};
	shape, shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	handles: [8]physics.Body_Handle;
	for body_index in 0 ..< len(handles)
	{
		body_description: physics.Body_Description = sleeping_body(
			shape, {f32(body_index) * 2, 0, 0},
		);
		handle, status := physics.simulation_add_body(simulation, &body_description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		handles[body_index] = handle;
	}
	active: ^physics.Body_Set = &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	Selection_Case :: struct
	{
		fraction:         f32,
		offset:           int,
		candidate_mask:   u8,
		expected_indices: [8]int,
		expected_count:   int,
		expected_offset:  int,
	}
	cases: [9]Selection_Case = {
		{fraction=0.5, offset=2, candidate_mask=0, expected_count=0, expected_offset=3},
		{
			fraction=0.5, offset=1, candidate_mask=0x28,
			expected_indices={3, 5, 0, 0, 0, 0, 0, 0}, expected_count=2, expected_offset=2,
		},
		{
			fraction=0.5, offset=0, candidate_mask=0xff,
			expected_indices={0, 2, 4, 6, 0, 0, 0, 0}, expected_count=4, expected_offset=1,
		},
		{
			fraction=0.5, offset=8, candidate_mask=0xff,
			expected_indices={0, 2, 4, 6, 0, 0, 0, 0}, expected_count=4, expected_offset=9,
		},
		{
			fraction=0.5, offset=9, candidate_mask=0xff,
			expected_indices={0, 2, 4, 6, 0, 0, 0, 0}, expected_count=4, expected_offset=1,
		},
		{
			fraction=0.5, offset=7, candidate_mask=0xff,
			expected_indices={7, 1, 3, 5, 0, 0, 0, 0}, expected_count=4, expected_offset=8,
		},
		{
			fraction=0.01, offset=3, candidate_mask=0xff,
			expected_indices={3, 0, 0, 0, 0, 0, 0, 0}, expected_count=1, expected_offset=4,
		},
		{
			fraction=1, offset=7, candidate_mask=0xff,
			expected_indices={7, 0, 1, 2, 3, 4, 5, 6}, expected_count=8, expected_offset=8,
		},
		{
			fraction=0.4, offset=7, candidate_mask=0xff,
			expected_indices={7, 1, 3, 0, 0, 0, 0, 0}, expected_count=3, expected_offset=8,
		},
	};
	for selection in cases
	{
		for body_index in 0 ..< len(handles)
		{
			active.activity.memory[body_index].sleep_candidate = .Not_Candidate;
			if selection.candidate_mask & (u8(1) << uint(body_index)) != 0
			{
				active.activity.memory[body_index].sleep_candidate = .Candidate;
			}
		}
		simulation.sleeper.tested_fraction_per_frame = selection.fraction;
		simulation.sleeper.schedule_offset = selection.offset;
		count, status := physics.island_sleeper_update_filtered_candidates(&simulation.sleeper);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, count, selection.expected_count);
		testing.expect_value(t, simulation.sleeper.schedule_offset, selection.expected_offset);
		for candidate_index in 0 ..< selection.expected_count
		{
			testing.expect_value(
				t, simulation.sleeper.scaffold.stack.memory[candidate_index],
				handles[selection.expected_indices[candidate_index]],
			);
		}
	}

	// zero fraction keeps the outer unfiltered route, including its one sampled handle
	for body_index in 0 ..< len(handles)
	{
		active.activity.memory[body_index].sleep_candidate = .Not_Candidate;
	}
	simulation.sleeper.tested_fraction_per_frame = 0;
	simulation.sleeper.target_slept_fraction = 0;
	simulation.sleeper.target_traversed_fraction = 0;
	simulation.sleeper.schedule_offset = 2;
	testing.expect_value(
		t, physics.island_sleeper_update(&simulation.sleeper, boundary), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, simulation.sleeper.schedule_offset, 3);
	testing.expect_value(t, simulation.sleeper.scaffold.stack.memory[0], handles[2]);
	testing.expect_value(t, active.count, len(handles));
}

@(test)
connected_island_sleeps_and_restores_handles :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	inner_boundary := util.thread_dispatcher_boundary(&dispatcher);
	dispatch_state := Island_Dispatch_State{inner=inner_boundary};
	boundary := &util.Thread_Dispatcher_Boundary{
		dispatcher=&dispatch_state,
		dispatch=island_recording_dispatch,
		worker_pool=island_recording_worker_pool,
		worker_count=inner_boundary.worker_count,
	};
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
	description := small_description(&pool);
	description.allocation_sizes.workers = 4;
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	testing.expect_value(
		t, simulation.sleeper.tested_fraction_per_frame, f32(0.01),
	);
	testing.expect_value(
		t, simulation.sleeper.target_slept_fraction, f32(0.005),
	);
	testing.expect_value(
		t, simulation.sleeper.target_traversed_fraction, f32(0.01),
	);
	simulation.sleeper.tested_fraction_per_frame = 0.4;
	simulation.sleeper.target_slept_fraction = 0.4;
	simulation.sleeper.target_traversed_fraction = 0.8;
	sphere := physics.Sphere{radius=0.25};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body_a_description := sleeping_body(shape, {-1, 0, 0});
	body_b_description := sleeping_body(shape, {1, 0, 0});
	body_b_description.local_inertia = {};
	body_a, status_a := physics.simulation_add_body(simulation, &body_a_description);
	body_b, status_b := physics.simulation_add_body(simulation, &body_b_description);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	handles := [4]physics.Body_Handle{body_a, body_b, {}, {}};
	constraint_description := physics.Ball_Socket{
		local_offset_a={1, 0, 0}, local_offset_b={-1, 0, 0},
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
	};
	constraint, constraint_status := physics.simulation_add_constraint(simulation, &handles, &constraint_description);
	testing.expect_value(t, constraint_status, physics.Physics_Status.Ok);
	second_constraint, second_constraint_status := physics.simulation_add_constraint(
		simulation, &handles, &constraint_description,
	);
	testing.expect_value(t, second_constraint_status, physics.Physics_Status.Ok);
	constraint_total :: 40;
	constraints: [constraint_total]physics.Constraint_Handle;
	constraints[0] = constraint;
	constraints[1] = second_constraint;
	for constraint_index in 2 ..< constraint_total
	{
		additional, additional_constraint_status := physics.simulation_add_constraint(
			simulation, &handles, &constraint_description,
		);
		testing.expect_value(t, additional_constraint_status, physics.Physics_Status.Ok);
		constraints[constraint_index] = additional;
	}
	secondary_a_description := sleeping_body(shape, {100, 0, 0});
	secondary_b_description := sleeping_body(shape, {102, 0, 0});
	secondary_a, secondary_a_status := physics.simulation_add_body(
		simulation, &secondary_a_description,
	);
	secondary_b, secondary_b_status := physics.simulation_add_body(
		simulation, &secondary_b_description,
	);
	testing.expect_value(t, secondary_a_status, physics.Physics_Status.Ok);
	testing.expect_value(t, secondary_b_status, physics.Physics_Status.Ok);
	secondary_handles := [4]physics.Body_Handle{
		secondary_a, secondary_b, {}, {},
	};
	secondary_constraint, secondary_constraint_status :=
		physics.simulation_add_constraint(
		simulation, &secondary_handles, &constraint_description,
	);
	testing.expect_value(
		t, secondary_constraint_status, physics.Physics_Status.Ok,
	);
	kinematic_description := sleeping_body(shape, {200, 0, 0});
	kinematic_description.local_inertia = {};
	kinematic, kinematic_status := physics.simulation_add_body(
		simulation, &kinematic_description,
	);
	testing.expect_value(t, kinematic_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0, boundary), physics.Physics_Status.Ok);
	active_after_prediction := [5]physics.Body_Handle{
		body_a, body_b, secondary_a, secondary_b, kinematic,
	};
	for active_handle in active_after_prediction
	{
		location, resolve_status := physics.bodies_resolve(
			&simulation.bodies, active_handle,
		);
		testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, location.set_index,
			i32(physics.BODIES_ACTIVE_SET_INDEX),
		);
		activity := simulation.bodies.sets.memory[
			location.set_index
		].activity.memory[location.index];
		testing.expect_value(
			t, activity.timesteps_under_threshold_count, u8(1),
		);
		testing.expect_value(
			t, activity.sleep_candidate,
			physics.Sleep_Candidate_State.Candidate,
		);
	}
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0, boundary), physics.Physics_Status.Ok);
	primary_after_target, primary_after_target_status :=
		physics.bodies_resolve(&simulation.bodies, body_a);
	secondary_after_target, secondary_after_target_status :=
		physics.bodies_resolve(&simulation.bodies, secondary_a);
	kinematic_after_target, kinematic_after_target_status :=
		physics.bodies_resolve(&simulation.bodies, kinematic);
	testing.expect_value(
		t, primary_after_target_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, secondary_after_target_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, kinematic_after_target_status, physics.Physics_Status.Ok,
	);
	testing.expect(t, primary_after_target.set_index > 0);
	testing.expect_value(
		t, secondary_after_target.set_index,
		i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	testing.expect_value(
		t, kinematic_after_target.set_index,
		i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	testing.expect_value(
		t, simulation.sleeper.scaffold.body_marks.memory[body_a.value],
		u8(1),
	);
	testing.expect_value(
		t, simulation.sleeper.scaffold.body_marks.memory[body_b.value],
		u8(1),
	);
	testing.expect_value(
		t, simulation.sleeper.scaffold.body_marks.memory[secondary_a.value],
		u8(1),
	);
	testing.expect_value(
		t, simulation.sleeper.scaffold.body_marks.memory[secondary_b.value],
		u8(1),
	);
	testing.expect_value(
		t, simulation.sleeper.scaffold.body_marks.memory[kinematic.value],
		u8(0),
	);
	simulation.sleeper.tested_fraction_per_frame = 1;
	simulation.sleeper.target_slept_fraction = 1;
	simulation.sleeper.target_traversed_fraction = 1;
	testing.expect_value(
		t, physics.simulation_timestep(
		simulation, 1.0 / 60.0, boundary,
	), physics.Physics_Status.Ok,
	);
	location_a, resolve_a := physics.bodies_resolve(&simulation.bodies, body_a);
	location_b, resolve_b := physics.bodies_resolve(&simulation.bodies, body_b);
	testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
	testing.expect_value(t, resolve_b, physics.Physics_Status.Ok);
	testing.expect(t, location_a.set_index > 0);
	testing.expect_value(t, location_b.set_index, location_a.set_index);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(0));
	testing.expect_value(
		t,
		physics.solver_reference_state(&simulation.solver, constraint),
		physics.Reference_State.Present
	);
	testing.expect_value(
		t,
		physics.solver_reference_state(&simulation.solver, second_constraint),
		physics.Reference_State.Present
	);
	testing.expect_value(t, simulation.sleeper.inactive_constraint_count, constraint_total + 1);
	for sleeping_constraint in constraints
	{
		location := simulation.solver.handle_to_constraint.memory[
			sleeping_constraint.value
		];
		testing.expect(t, location.set_index > 0);
		testing.expect(
			t, int(location.index_in_type_batch) <
			simulation.sleeper.inactive_constraint_count,
		);
		record := &simulation.sleeper.inactive_constraints.memory[
			location.index_in_type_batch
		];
		testing.expect_value(t, record.handle, sleeping_constraint);
	}
	sleeping_readback: physics.Ball_Socket;
	testing.expect_value(
		t,
		physics.simulation_get_constraint_description(
		simulation, constraint, &sleeping_readback,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, sleeping_readback.local_offset_a,
		constraint_description.local_offset_a,
	);
	invalid_constraint_description := constraint_description;
	invalid_constraint_description.spring_settings.angular_frequency = 0;
	sleeping_set_index := location_a.set_index;
	testing.expect_value(
		t,
		physics.simulation_apply_constraint_description(
		simulation, constraint, &invalid_constraint_description,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
	testing.expect_value(t, location_a.set_index, sleeping_set_index);
	testing.expect_value(
		t, simulation.sleeper.inactive_constraint_count,
		constraint_total + 1,
	);
	updated_constraint_description := constraint_description;
	updated_constraint_description.local_offset_a.x = 1.25;
	testing.expect_value(
		t,
		physics.simulation_apply_constraint_description(
		simulation, constraint, &updated_constraint_description,
	),
		physics.Physics_Status.Ok,
	);
	location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
	testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
	testing.expect_value(
		t, location_a.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	testing.expect_value(t, simulation.sleeper.inactive_constraint_count, 1);
	sleeping_readback = {};
	testing.expect_value(
		t,
		physics.simulation_get_constraint_description(
		simulation, constraint, &sleeping_readback,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, sleeping_readback.local_offset_a,
		updated_constraint_description.local_offset_a,
	);
	secondary_readback: physics.Ball_Socket;
	testing.expect_value(
		t,
		physics.simulation_get_constraint_description(
		simulation, secondary_constraint, &secondary_readback,
	),
		physics.Physics_Status.Ok,
	);
	secondary_location :=
		simulation.solver.handle_to_constraint.memory[
		secondary_constraint.value
	];
	testing.expect_value(
		t,
		simulation.sleeper.inactive_constraints.memory[
		secondary_location.index_in_type_batch
	].handle,
		secondary_constraint,
	);
	testing.expect_value(
		t, physics.simulation_timestep(
		simulation, 1.0 / 60.0, boundary,
	), physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_timestep(
		simulation, 1.0 / 60.0, boundary,
	), physics.Physics_Status.Ok,
	);
	location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
	testing.expect(t, location_a.set_index > 0);
	testing.expect_value(
		t, simulation.sleeper.inactive_constraint_count,
		constraint_total + 1,
	);
	primary_sleeping_set_index := location_a.set_index;
	secondary_a_location, secondary_a_resolve := physics.bodies_resolve(
		&simulation.bodies, secondary_a,
	);
	secondary_b_location, secondary_b_resolve := physics.bodies_resolve(
		&simulation.bodies, secondary_b,
	);
	testing.expect_value(t, secondary_a_resolve, physics.Physics_Status.Ok);
	testing.expect_value(t, secondary_b_resolve, physics.Physics_Status.Ok);
	testing.expect(t, secondary_a_location.set_index > 0);
	testing.expect_value(
		t, secondary_b_location.set_index, secondary_a_location.set_index,
	);
	testing.expect(
		t, secondary_a_location.set_index != primary_sleeping_set_index,
	);
	testing.expect_value(
		t,
		physics.simulation_remove_constraint(
		simulation, secondary_constraint,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t,
		physics.solver_reference_state(
		&simulation.solver, secondary_constraint,
	),
		physics.Reference_State.Missing,
	);
	location_a, resolve_a = physics.bodies_resolve(
		&simulation.bodies, body_a,
	);
	secondary_a_location, secondary_a_resolve = physics.bodies_resolve(
		&simulation.bodies, secondary_a,
	);
	secondary_b_location, secondary_b_resolve = physics.bodies_resolve(
		&simulation.bodies, secondary_b,
	);
	testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
	testing.expect_value(
		t, location_a.set_index, primary_sleeping_set_index,
	);
	testing.expect_value(t, secondary_a_resolve, physics.Physics_Status.Ok);
	testing.expect_value(t, secondary_b_resolve, physics.Physics_Status.Ok);
	testing.expect_value(
		t, secondary_a_location.set_index,
		i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	testing.expect_value(
		t, secondary_b_location.set_index,
		i32(physics.BODIES_ACTIVE_SET_INDEX),
	);
	testing.expect_value(
		t, simulation.sleeper.inactive_constraint_count,
		constraint_total,
	);
	for sleeping_constraint in constraints
	{
		location := simulation.solver.handle_to_constraint.memory[
			sleeping_constraint.value
		];
		testing.expect_value(
			t, location.set_index, primary_sleeping_set_index,
		);
		testing.expect(
			t, int(location.index_in_type_batch) <
			simulation.sleeper.inactive_constraint_count,
		);
		record := &simulation.sleeper.inactive_constraints.memory[
			location.index_in_type_batch
		];
		testing.expect_value(t, record.handle, sleeping_constraint);
	}
	testing.expect(t, dispatch_state.traversal_maximum > 1);
	testing.expect(t, dispatch_state.constraint_gather_maximum > 1);
	testing.expect(t, dispatch_state.body_gather_maximum > 1);
	intruder_description := sleeping_body(shape, {-1, 0, 0});
	intruder_description.activity.sleep_threshold = -1;
	intruder, intruder_status := physics.simulation_add_body(simulation, &intruder_description);
	testing.expect_value(t, intruder_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0, boundary), physics.Physics_Status.Ok);
	location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
	location_b, resolve_b = physics.bodies_resolve(&simulation.bodies, body_b);
	testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
	testing.expect_value(t, resolve_b, physics.Physics_Status.Ok);
	testing.expect_value(t, location_a.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	testing.expect_value(t, location_b.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	testing.expect(t, simulation.solver.active_set.constraint_count >= i32(constraint_total + 1));
	testing.expect(t, simulation.narrow_phase.pair_cache.mapping.count > 0);
	testing.expect(t, dispatch_state.awakener_phase_one_maximum > 1);
	testing.expect(t, dispatch_state.awakener_phase_two_maximum > 1);
	testing.expect_value(t, physics.simulation_remove_body(simulation, intruder), physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(constraint_total));
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	body_description, body_description_status := physics.bodies_get_description(&simulation.bodies, body_a);
	testing.expect_value(t, body_description_status, physics.Physics_Status.Ok);
	body_description.velocity = {};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body_a, &body_description), physics.Physics_Status.Ok,
	);
	body_b_reset, body_b_reset_status := physics.bodies_get_description(&simulation.bodies, body_b);
	testing.expect_value(t, body_b_reset_status, physics.Physics_Status.Ok);
	body_b_reset.velocity = {};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body_b, &body_b_reset), physics.Physics_Status.Ok,
	);
	location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
	location_b, resolve_b = physics.bodies_resolve(&simulation.bodies, body_b);
	testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
	testing.expect_value(t, resolve_b, physics.Physics_Status.Ok);
	testing.expect_value(t, location_a.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	testing.expect_value(t, location_b.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(constraint_total));
	constraint_location, constraint_resolve := physics.solver_resolve(&simulation.solver, constraint);
	testing.expect_value(t, constraint_resolve, physics.Physics_Status.Ok);
	testing.expect_value(t, constraint_location.set_index, i32(0));
	readback: physics.Ball_Socket;
	testing.expect_value(
		t, physics.solver_get_description(&simulation.solver, constraint, &readback), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, readback.local_offset_a, updated_constraint_description.local_offset_a);
	testing.expect_value(t, readback.local_offset_b, constraint_description.local_offset_b);
	testing.expect_value(t, simulation.sleeper.inactive_constraint_count, 0);
	for cycle in 0 ..< 12
	{
		_ = cycle;
		testing.expect_value(
			t,
			physics.simulation_timestep(simulation, 1.0 / 60.0, boundary),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.simulation_timestep(simulation, 1.0 / 60.0, boundary),
			physics.Physics_Status.Ok
		);
		location_a, resolve_a = physics.bodies_resolve(&simulation.bodies, body_a);
		testing.expect_value(t, resolve_a, physics.Physics_Status.Ok);
		testing.expect(t, location_a.set_index > 0);
		body_description, body_description_status = physics.bodies_get_description(&simulation.bodies, body_a);
		testing.expect_value(t, body_description_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, physics.simulation_apply_body_description(
			simulation,
			body_a,
			&body_description
		), physics.Physics_Status.Ok,
		);
		testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(constraint_total));
	}
}

@(test)
static_changes_awaken_overlapping_sleeping_body :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
	description := small_description(&pool);
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	sphere := physics.Sphere{radius=0.5};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body_description := sleeping_body(shape, {});
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	location, resolve_status := physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect(t, location.set_index > 0);
	kinematic_description := sleeping_body(shape, {1.25, 0, 0});
	kinematic, kinematic_status := physics.simulation_add_body(simulation, &kinematic_description);
	testing.expect_value(t, kinematic_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	kinematic_location, kinematic_resolve_status := physics.bodies_resolve(&simulation.bodies, kinematic);
	testing.expect_value(t, kinematic_resolve_status, physics.Physics_Status.Ok);
	testing.expect(t, kinematic_location.set_index > 0);
	kinematic_set := &simulation.bodies.sets.memory[kinematic_location.set_index];
	kinematic_set.dynamics_state.memory[kinematic_location.index].inertia.local = {};
	kinematic_set.dynamics_state.memory[kinematic_location.index].inertia.world = {};
	testing.expect_value(
		t, physics.broad_phase_refresh_body_membership(&simulation.broad_phase, kinematic), physics.Physics_Status.Ok,
	);
	static_description := physics.Static_Description{
		pose={orientation=util.quaternion_identity(), position={0.625, 0, 0}},
		shape=shape,
	};
	static_handle, static_status := physics.simulation_add_static(simulation, &static_description);
	testing.expect_value(t, static_status, physics.Physics_Status.Ok);
	location, resolve_status = physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	kinematic_location, kinematic_resolve_status = physics.bodies_resolve(&simulation.bodies, kinematic);
	testing.expect_value(t, kinematic_resolve_status, physics.Physics_Status.Ok);
	testing.expect(t, kinematic_location.set_index > 0);
	static_description.pose.position.x = 0.75;
	testing.expect_value(
		t,
		physics.simulation_apply_static_description(simulation, static_handle, &static_description),
		physics.Physics_Status.Ok,
	);
	kinematic_location, kinematic_resolve_status = physics.bodies_resolve(&simulation.bodies, kinematic);
	testing.expect_value(t, kinematic_resolve_status, physics.Physics_Status.Ok);
	testing.expect(t, kinematic_location.set_index > 0);
	testing.expect_value(t, physics.simulation_remove_static(simulation, static_handle), physics.Physics_Status.Ok);
	kinematic_location, kinematic_resolve_status = physics.bodies_resolve(&simulation.bodies, kinematic);
	testing.expect_value(t, kinematic_resolve_status, physics.Physics_Status.Ok);
	testing.expect(t, kinematic_location.set_index > 0);
}

@(test)
static_changes_can_skip_awakening_overlapping_sleeping_body :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
	description := small_description(&pool);
	if !testing.expect_value(
		t, physics.simulation_create(simulation, &description).status,
		physics.Physics_Status.Ok,
	)
	{
		return;
	}
	defer physics.simulation_destroy(simulation);

	sphere := physics.Sphere{radius=0.5};
	sphere_shape, sphere_status := physics.shape_registry_add(
		&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	if !testing.expect_value(t, sphere_status, physics.Physics_Status.Ok)
	{
		return;
	}
	box := physics.Box{1, 1, 1};
	box_shape, box_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	if !testing.expect_value(t, box_status, physics.Physics_Status.Ok)
	{
		return;
	}

	body_description := sleeping_body(sphere_shape, {});
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	if !testing.expect_value(t, body_status, physics.Physics_Status.Ok)
	{
		return;
	}
	for _ in 0 ..< 2
	{
		if !testing.expect_value(
			t, physics.simulation_timestep(simulation, 1.0 / 60.0),
			physics.Physics_Status.Ok,
		)
		{
			return;
		}
	}
	location, resolve_status := physics.bodies_resolve(&simulation.bodies, body);
	if !testing.expect_value(t, resolve_status, physics.Physics_Status.Ok)
	{
		return;
	}
	if !testing.expect(t, location.set_index > physics.BODIES_ACTIVE_SET_INDEX)
	{
		return;
	}
	sleeping_set_index := location.set_index;

	static_description := physics.Static_Description{
		pose={orientation=util.quaternion_identity()},
		shape=sphere_shape,
	};
	static_handle, static_status :=
		physics.simulation_add_static_without_awakening_bodies(
		simulation, &static_description,
	);
	if !testing.expect_value(t, static_status, physics.Physics_Status.Ok)
	{
		return;
	}
	location, resolve_status = physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.set_index, sleeping_set_index);

	static_description.pose.position = {0.25, 0, 0};
	if !testing.expect_value(
		t,
		physics.simulation_apply_static_description_without_awakening_bodies(
		simulation, static_handle, &static_description,
	),
		physics.Physics_Status.Ok,
	)
	{
		return;
	}
	location, resolve_status = physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.set_index, sleeping_set_index);

	if !testing.expect_value(
		t,
		physics.simulation_set_static_shape_without_awakening_bodies(
		simulation, static_handle, box_shape,
	),
		physics.Physics_Status.Ok,
	)
	{
		return;
	}
	location, resolve_status = physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.set_index, sleeping_set_index);

	if !testing.expect_value(
		t,
		physics.simulation_remove_static_without_awakening_bodies(
		simulation, static_handle,
	),
		physics.Physics_Status.Ok,
	)
	{
		return;
	}
	location, resolve_status = physics.bodies_resolve(&simulation.bodies, body);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, location.set_index, sleeping_set_index);
}
