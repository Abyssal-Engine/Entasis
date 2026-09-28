package tests

import "core:math"
import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

create_small_simulation :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool,
) -> ^physics.Simulation
{
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=2,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=2,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=2,
		pairs=2,
		collision_child_pairs=2,
		inactive_pairs=1,
		pending_pairs_per_worker=2,
		workers=1,
	};
	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && simulation_memory != nil)
	{
		return nil;
	}
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		_ = mem.free(simulation_memory);
		return nil;
	}
	return simulation;
}

release_small_simulation :: proc(simulation: ^physics.Simulation)
{
	if simulation == nil
	{
		return;
	}
	if simulation.state == .Ready
	{
		_ = physics.simulation_destroy(simulation);
	}
	_ = mem.free(simulation);
}

@(test)
simulation_contact_storage_grows_from_small_capacities :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	simulation := create_small_simulation(t, &pool);
	if simulation == nil
	{
		return;
	}
	defer release_small_simulation(simulation);

	body_shape_value := physics.Box{0.45, 0.45, 0.45};
	floor_shape_value := physics.Box{64, 0.5, 2};
	body_shape, body_shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &body_shape_value,
	);
	if !testing.expect_value(t, body_shape_status, physics.Physics_Status.Ok)
	{
		return;
	}
	floor_shape, floor_shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &floor_shape_value,
	);
	if !testing.expect_value(t, floor_shape_status, physics.Physics_Status.Ok)
	{
		return;
	}

	floor_description := physics.Static_Description{
		pose={orientation=util.quaternion_identity(), position={0, -0.5, 0}},
		shape=floor_shape,
	};
	_, static_status := physics.simulation_add_static(simulation, &floor_description);
	if !testing.expect_value(t, static_status, physics.Physics_Status.Ok)
	{
		return;
	}

	inertia, inertia_status := physics.box_inertia(body_shape_value, 1);
	if !testing.expect_value(t, inertia_status, physics.Physics_Status.Ok)
	{
		return;
	}
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			shape=body_shape,
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=32},
	};
	for index in 0 ..< 32
	{
		body_description.pose.position = {f32(index) * 1.25 - 19.375, 0.44, 0};
		_, body_status := physics.simulation_add_body(simulation, &body_description);
		if !testing.expectf(t, body_status == .Ok, "body %d failed with %v", index, body_status)
		{
			return;
		}
	}

	dt := f32(1.0 / 60.0);
	stage_status := physics.simulation_sleep(simulation);
	if !testing.expectf(t, stage_status == .Ok, "sleep failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.simulation_predict_bounding_boxes(simulation, dt);
	if !testing.expectf(t, stage_status == .Ok, "predict failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.broad_phase_update(&simulation.broad_phase, nil);
	if !testing.expectf(t, stage_status == .Ok, "broad phase update failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.narrow_phase_execute(
		&simulation.narrow_phase, nil, dt,
	);
	transaction_count := 0;
	for worker_index in 0 ..< simulation.narrow_phase.active_worker_count
	{
		transaction_count += int(
			simulation.narrow_phase.transaction_streams[worker_index].total_count,
		);
	}
	if !testing.expectf(
		t, stage_status == .Ok,
		"narrow phase failed with %v stage=%v transactions=%d candidates=%d",
		stage_status, simulation.narrow_phase.last_stage,
		transaction_count, simulation.narrow_phase.candidate_counts[0],
	)
	{
		return;
	}
	stage_status = physics.simulation_solve(simulation, dt);
	if !testing.expectf(t, stage_status == .Ok, "solve failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.simulation_incrementally_optimize_data_structures(simulation);
	if !testing.expectf(t, stage_status == .Ok, "optimize failed with %v", stage_status)
	{
		return;
	}
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
		32,
	);
	testing.expect(t, simulation.solver.active_set.constraint_count >= 24);
	testing.expect(t, simulation.narrow_phase.pair_cache.mapping.count >= 24);
}

@(test)
simulation_sleeping_storage_grows_from_small_capacities :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	description := physics.simulation_create_description_default(&pool);
	description.default_pose_context.gravity = {};
	description.default_pose_context.linear_damping = 0;
	description.default_pose_context.angular_damping = 0;
	description.allocation_sizes = {
		bodies=2,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=2,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=2,
		pairs=2,
		collision_child_pairs=2,
		inactive_pairs=1,
		pending_pairs_per_worker=2,
		workers=1,
	};
	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && simulation_memory != nil)
	{
		return;
	}
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		_ = mem.free(simulation_memory);
		return;
	}
	defer release_small_simulation(simulation);

	box := physics.Box{0.5, 0.5, 0.5};
	shape, shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
	{
		return;
	}
	inertia, inertia_status := physics.box_inertia(box, 1);
	if !testing.expect_value(t, inertia_status, physics.Physics_Status.Ok)
	{
		return;
	}
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			shape=shape,
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={sleep_threshold=1, minimum_timestep_count_under_threshold=1},
	};
	body_count :: 32;
	for body_index in 0 ..< body_count
	{
		body_description.pose.position = {f32(body_index) * 4, 0, 0};
		_, body_status := physics.simulation_add_body(simulation, &body_description);
		if !testing.expectf(
			t, body_status == .Ok, "sleeping body %d failed with %v",
			body_index, body_status,
		)
		{
			return;
		}
	}

	active_for_candidacy := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< active_for_candidacy.count
	{
		physics.island_sleeper_update_candidacy(
			&active_for_candidacy.activity.memory[body_index],
			active_for_candidacy.dynamics_state.memory[body_index].motion.velocity,
		);
	}

	initial_set_capacity := int(simulation.bodies.sets.length);
	initial_inactive_capacity := simulation.bodies.inactive_arena.capacity;
	for frame_index in 0 ..< body_count * 8
	{
		if simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count == 0
		{
			break;
		}
		status := physics.island_sleeper_update(
			&simulation.sleeper, nil,
		);
		if !testing.expectf(
			t, status == .Ok,
			"sleep update %d failed with %v active=%d sets=%d arena=%d static_leaves=%d",
			frame_index, status,
			simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
			simulation.bodies.sets.length, simulation.bodies.inactive_arena.capacity,
			simulation.broad_phase.static_tree.leaf_count,
		)
		{
			return;
		}
	}

	active_count :=
		simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count;
	testing.expect_value(t, active_count, 0);
	inactive_body_count := 0;
	inactive_set_count := 0;
	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		set := &simulation.bodies.sets.memory[set_index];
		inactive_body_count += set.count;
		if set.count > 0
		{
			inactive_set_count += 1;
		}
	}
	testing.expect_value(t, inactive_body_count, body_count);
	testing.expect_value(t, inactive_set_count, body_count);
	testing.expect(t, int(simulation.bodies.sets.length) > initial_set_capacity);
	testing.expect(t, simulation.bodies.inactive_arena.capacity > initial_inactive_capacity);
	testing.expect(t, simulation.bodies.inactive_arena.capacity >= body_count);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 0);
	testing.expect_value(t, simulation.broad_phase.static_tree.leaf_count, body_count);

	active := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	resize_status := physics.body_set_resize_capacity(active, 1, 1, &pool);
	if !testing.expect_value(t, resize_status, physics.Physics_Status.Ok)
	{
		return;
	}
	testing.expect(t, physics.body_set_capacity(active) < body_count);
	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		if simulation.bodies.sets.memory[set_index].count == 0
		{
			continue;
		}
		awaken_status := physics.island_awakener_awaken_set(
			&simulation.awakener, set_index, nil, .Present,
		);
		if !testing.expectf(
			t, awaken_status == .Ok,
			"awaken set %d failed with %v active=%d capacity=%d",
			set_index, awaken_status, active.count, physics.body_set_capacity(active),
		)
		{
			return;
		}
	}
	testing.expect_value(t, active.count, body_count);
	testing.expect(t, physics.body_set_capacity(active) >= body_count);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, body_count);
	testing.expect_value(t, simulation.broad_phase.static_tree.leaf_count, 0);
}

@(test)
simulation_direct_constraints_grow_from_small_capacities :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	simulation := create_small_simulation(t, &pool);
	if simulation == nil
	{
		return;
	}
	defer release_small_simulation(simulation);

	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia={
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=32},
	};
	constraint_count :: 64;
	constraints: [constraint_count]physics.Constraint_Handle;
	spring := physics.spring_settings_create(30, 1);
	constraint_description := physics.Ball_Socket{
		local_offset_a={0.5, 0, 0},
		local_offset_b={-0.5, 0, 0},
		spring_settings=spring,
	};
	for constraint_index in 0 ..< constraint_count
	{
		handles: [4]physics.Body_Handle;
		for body_index in 0 ..< 2
		{
			body_description.pose.position = {
				f32(constraint_index) * 4 + f32(body_index),
				0,
				0,
			};
			handle, body_status := physics.simulation_add_body(
				simulation, &body_description,
			);
			if !testing.expectf(
				t, body_status == .Ok,
				"constraint body %d:%d failed with %v",
				constraint_index, body_index, body_status,
			)
			{
				return;
			}
			handles[body_index] = handle;
		}
		constraint, constraint_status := physics.simulation_add_constraint(
			simulation, &handles, &constraint_description,
		);
		if !testing.expectf(
			t, constraint_status == .Ok,
			"constraint %d failed with %v",
			constraint_index, constraint_status,
		)
		{
			return;
		}
		constraints[constraint_index] = constraint;
	}
	testing.expect_value(
		t, simulation.solver.active_set.constraint_count, constraint_count,
	);
	for constraint in constraints
	{
		remove_status := physics.simulation_remove_constraint(simulation, constraint);
		if !testing.expect_value(t, remove_status, physics.Physics_Status.Ok)
		{
			return;
		}
	}
	testing.expect_value(t, simulation.solver.active_set.constraint_count, 0);
	testing.expect(
		t, simulation.solver.handle_pool.available_id_count >= constraint_count,
	);
}

run_contact_sleep_awaken_growth :: proc(t: ^testing.T, worker_count: int)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	description := physics.simulation_create_description_default(&pool);
	description.default_pose_context.gravity = {};
	description.default_pose_context.linear_damping = 0;
	description.default_pose_context.angular_damping = 0;
	description.allocation_sizes = {
		bodies=2,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=2,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=2,
		pairs=2,
		collision_child_pairs=16,
		inactive_pairs=1,
		pending_pairs_per_worker=2,
		workers=i32(worker_count),
	};
	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && simulation_memory != nil)
	{
		return;
	}
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		_ = mem.free(simulation_memory);
		return;
	}

	dispatcher: util.Thread_Dispatcher;
	dispatcher_ready := false;
	if worker_count > 1
	{
		dispatch_status := util.thread_dispatcher_initialize(&dispatcher, worker_count, 65536);
		if !testing.expect_value(t, dispatch_status, util.Threading_Status.Ok)
		{
			_ = physics.simulation_destroy(simulation);
			_ = mem.free(simulation_memory);
			return;
		}
		dispatcher_ready = true;
	}
	defer
	{
		if dispatcher_ready
		{
			_ = util.thread_dispatcher_shutdown(&dispatcher);
		}
		if simulation.state == .Ready
		{
			_ = physics.simulation_destroy(simulation);
		}
		_ = mem.free(simulation_memory);
	}
	boundary: ^util.Thread_Dispatcher_Boundary;
	if dispatcher_ready
	{
		boundary = util.thread_dispatcher_boundary(&dispatcher);
	}

	box := physics.Box{0.5, 0.5, 0.5};
	shape, shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
	{
		return;
	}
	inertia, inertia_status := physics.box_inertia(box, 1);
	if !testing.expect_value(t, inertia_status, physics.Physics_Status.Ok)
	{
		return;
	}

	pair_count :: 16;
	body_count :: pair_count * 2;
	body_handles: [body_count]physics.Body_Handle;
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			shape=shape,
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=32},
	};
	for pair_index in 0 ..< pair_count
	{
		base_x := f32(pair_index) * 4;
		for local_index in 0 ..< 2
		{
			body_index := pair_index * 2 + local_index;
			body_description.pose.position = {
				base_x + f32(local_index) * 0.9, 0, 0,
			};
			handle, body_status := physics.simulation_add_body(
				simulation, &body_description,
			);
			if !testing.expectf(
				t, body_status == .Ok,
				"contact body %d failed with %v", body_index, body_status,
			)
			{
				return;
			}
			body_handles[body_index] = handle;
		}
	}

	dt := f32(1.0 / 60.0);
	status := physics.simulation_predict_bounding_boxes(simulation, dt, boundary);
	if !testing.expect_value(t, status, physics.Physics_Status.Ok)
	{
		return;
	}
	status = physics.simulation_collision_detection(simulation, dt, boundary);
	if !testing.expectf(t, status == .Ok, "collision detection failed with %v", status)
	{
		return;
	}
	status = physics.simulation_solve(simulation, dt, boundary);
	if !testing.expectf(t, status == .Ok, "initial solve failed with %v", status)
	{
		return;
	}
	status = physics.simulation_incrementally_optimize_data_structures(simulation, boundary);
	if !testing.expect_value(t, status, physics.Physics_Status.Ok)
	{
		return;
	}

	if !testing.expectf(
		t, simulation.narrow_phase.pair_cache.mapping.count >= pair_count,
		"expected at least %d active contact pairs, got %d",
		pair_count, simulation.narrow_phase.pair_cache.mapping.count,
	)
	{
		return;
	}
	if !testing.expectf(
		t, simulation.solver.active_set.constraint_count >= pair_count,
		"expected at least %d contact constraints, got %d",
		pair_count, simulation.solver.active_set.constraint_count,
	)
	{
		return;
	}

	active := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< active.count
	{
		active.dynamics_state.memory[body_index].motion.velocity = {};
		active.activity.memory[body_index] = {
			sleep_threshold=1,
			minimum_timesteps_under_threshold=1,
		};
	}

	active_for_candidacy := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< active_for_candidacy.count
	{
		physics.island_sleeper_update_candidacy(
			&active_for_candidacy.activity.memory[body_index],
			active_for_candidacy.dynamics_state.memory[body_index].motion.velocity,
		);
	}

	initial_set_capacity := int(simulation.bodies.sets.length);
	initial_inactive_pair_capacity :=
		int(simulation.narrow_phase.pair_cache.inactive_entries.length);
	for iteration in 0 ..< body_count * 8
	{
		active_count := simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count;
		if active_count == 0
		{
			break;
		}
		status = physics.island_sleeper_update(
			&simulation.sleeper, boundary,
		);
		if !testing.expectf(
			t, status == .Ok,
			"contact sleep %d failed with %v active=%d inactive_constraints=%d inactive_pairs=%d",
			iteration, status,
			simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
			simulation.sleeper.inactive_constraint_count,
			simulation.narrow_phase.pair_cache.inactive_count,
		)
		{
			return;
		}
	}

	active = &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	if !testing.expect_value(t, active.count, 0)
	{
		return;
	}
	if !testing.expectf(
		t, simulation.sleeper.inactive_constraint_count >= pair_count,
		"expected at least %d inactive constraints, got %d",
		pair_count, simulation.sleeper.inactive_constraint_count,
	)
	{
		return;
	}
	if !testing.expectf(
		t, simulation.narrow_phase.pair_cache.inactive_count >= pair_count,
		"expected at least %d inactive pairs, got %d",
		pair_count, simulation.narrow_phase.pair_cache.inactive_count,
	)
	{
		return;
	}
	if !testing.expect(t, int(simulation.bodies.sets.length) > initial_set_capacity)
	{
		return;
	}
	if !testing.expect(
		t, int(simulation.narrow_phase.pair_cache.inactive_entries.length) >
		initial_inactive_pair_capacity,
	)
	{
		return;
	}

	resize_status := physics.body_set_resize_capacity(active, 1, 1, &pool);
	if !testing.expect_value(t, resize_status, physics.Physics_Status.Ok)
	{
		return;
	}
	if !testing.expect(t, physics.body_set_capacity(active) < body_count)
	{
		return;
	}

	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		if simulation.bodies.sets.memory[set_index].count == 0
		{
			continue;
		}
		status = physics.island_awakener_awaken_set(
			&simulation.awakener, set_index, boundary, .Present,
		);
		if !testing.expectf(
			t, status == .Ok,
			"contact awaken set %d failed with %v active=%d capacity=%d inactive_constraints=%d inactive_pairs=%d",
			set_index, status, active.count, physics.body_set_capacity(active),
			simulation.sleeper.inactive_constraint_count,
			simulation.narrow_phase.pair_cache.inactive_count,
		)
		{
			return;
		}
	}

	if !testing.expect_value(t, active.count, body_count)
	{
		return;
	}
	if !testing.expect(t, physics.body_set_capacity(active) >= body_count)
	{
		return;
	}
	if !testing.expectf(
		t, simulation.solver.active_set.constraint_count >= pair_count,
		"expected restored constraints, got %d",
		simulation.solver.active_set.constraint_count,
	)
	{
		return;
	}
	if !testing.expectf(
		t, simulation.narrow_phase.pair_cache.mapping.count >= pair_count,
		"expected restored pairs, got %d",
		simulation.narrow_phase.pair_cache.mapping.count,
	)
	{
		return;
	}
	if !testing.expect_value(t, simulation.sleeper.inactive_constraint_count, 0)
	{
		return;
	}
	if !testing.expect_value(t, simulation.narrow_phase.pair_cache.inactive_count, 0)
	{
		return;
	}

	status = physics.simulation_timestep(simulation, dt, boundary);
	testing.expectf(t, status == .Ok, "post-awaken timestep failed with %v", status);
}

@(test)
simulation_contact_sleep_awaken_growth_single_threaded :: proc(t: ^testing.T)
{
	run_contact_sleep_awaken_growth(t, 1);
}

@(test)
simulation_contact_sleep_awaken_growth_four_workers :: proc(t: ^testing.T)
{
	run_contact_sleep_awaken_growth(t, 4);
}

@(test)
simulation_constrained_sleep_and_awaken_grows_all_destinations :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	description := physics.simulation_create_description_default(&pool);
	description.default_pose_context.gravity = {};
	description.default_pose_context.linear_damping = 0;
	description.default_pose_context.angular_damping = 0;
	description.allocation_sizes = {
		bodies=2,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=2,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=2,
		pairs=2,
		collision_child_pairs=2,
		inactive_pairs=1,
		pending_pairs_per_worker=2,
		workers=1,
	};
	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expect(t, allocation_error == nil && simulation_memory != nil)
	{
		return;
	}
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expect_value(t, create_result.status, physics.Physics_Status.Ok)
	{
		_ = mem.free(simulation_memory);
		return;
	}
	defer release_small_simulation(simulation);

	box := physics.Box{0.5, 0.5, 0.5};
	shape, shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
	{
		return;
	}
	inertia, inertia_status := physics.box_inertia(box, 1);
	if !testing.expect_value(t, inertia_status, physics.Physics_Status.Ok)
	{
		return;
	}
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			shape=shape,
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={
			sleep_threshold=f32(math.F32_MAX),
			minimum_timestep_count_under_threshold=1,
		},
	};

	island_count :: 8;
	body_count :: island_count * 2;
	body_handles: [body_count]physics.Body_Handle;
	spring := physics.spring_settings_create(30, 1);
	constraint_description := physics.Ball_Socket{
		local_offset_a={},
		local_offset_b={},
		spring_settings=spring,
	};
	for island_index in 0 ..< island_count
	{
		pair_handles: [4]physics.Body_Handle;
		base_x := f32(island_index) * 4;
		for body_index in 0 ..< 2
		{
			body_description.pose.position = {
				base_x + f32(body_index) * 0.45,
				0,
				0,
			};
			handle, body_status := physics.simulation_add_body(
				simulation, &body_description,
			);
			if !testing.expectf(
				t, body_status == .Ok,
				"island %d body %d failed with %v",
				island_index, body_index, body_status,
			)
			{
				return;
			}
			pair_handles[body_index] = handle;
			body_handles[island_index * 2 + body_index] = handle;
		}
		_, constraint_status := physics.simulation_add_constraint(
			simulation, &pair_handles, &constraint_description,
		);
		if !testing.expectf(
			t, constraint_status == .Ok,
			"island %d constraint failed with %v",
			island_index, constraint_status,
		)
		{
			return;
		}
	}

	dt := f32(1.0 / 60.0);
	stage_status := physics.simulation_predict_bounding_boxes(simulation, dt);
	if !testing.expectf(t, stage_status == .Ok, "predict failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.broad_phase_update(&simulation.broad_phase, nil);
	if !testing.expectf(t, stage_status == .Ok, "broad phase update failed with %v", stage_status)
	{
		return;
	}
	stage_status = physics.narrow_phase_execute(
		&simulation.narrow_phase, nil, dt,
	);
	if !testing.expectf(t, stage_status == .Ok, "narrow phase failed with %v", stage_status)
	{
		return;
	}

	active_constraint_count := simulation.solver.active_set.constraint_count;
	active_pair_count := simulation.narrow_phase.pair_cache.mapping.count;
	if !testing.expect(t, active_constraint_count >= island_count * 2)
	{
		return;
	}
	if !testing.expect(t, active_pair_count >= island_count)
	{
		return;
	}

	for frame_index in 0 ..< body_count * 8
	{
		if simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count == 0
		{
			break;
		}
		sleep_status := physics.island_sleeper_update(
			&simulation.sleeper, nil,
		);
		if !testing.expectf(
			t, sleep_status == .Ok,
			"constrained sleep update %d failed with %v",
			frame_index, sleep_status,
		)
		{
			return;
		}
	}

	active := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	testing.expect_value(t, active.count, 0);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, 0);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 0);
	testing.expect_value(
		t, simulation.narrow_phase.pair_cache.inactive_count, active_pair_count,
	);
	testing.expect_value(
		t, simulation.sleeper.inactive_constraint_count, int(active_constraint_count),
	);
	testing.expect(
		t, int(simulation.narrow_phase.pair_cache.inactive_entries.length) >
		int(description.allocation_sizes.inactive_pairs),
	);

	resize_status := physics.body_set_resize_capacity(active, 1, 1, &pool);
	if !testing.expect_value(t, resize_status, physics.Physics_Status.Ok)
	{
		return;
	}
	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		set := &simulation.bodies.sets.memory[set_index];
		if set.count == 0
		{
			continue;
		}
		for body_index in 0 ..< set.count
		{
			list := &set.constraints.memory[body_index];
			list_resize_status := physics.physics_resize_buffer_capacity(
				&pool, &list.span, 1, list.count,
			);
			if !testing.expect_value(
				t, list_resize_status, physics.Physics_Status.Ok,
			)
			{
				return;
			}
		}
	}
	for batch_index in 0 ..< int(simulation.solver.active_set.batches.length)
	{
		batch := &simulation.solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			if type_batch.state != .Allocated
			{
				continue;
			}
			type_resize_status := physics.type_batch_resize(type_batch, 1, &pool);
			if !testing.expect_value(
				t, type_resize_status, physics.Physics_Status.Ok,
			)
			{
				return;
			}
		}
	}

	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		if simulation.bodies.sets.memory[set_index].count == 0
		{
			continue;
		}
		awaken_status := physics.island_awakener_awaken_set(
			&simulation.awakener, set_index, nil, .Present,
		);
		if !testing.expectf(
			t, awaken_status == .Ok,
			"constrained awaken set %d failed with %v",
			set_index, awaken_status,
		)
		{
			return;
		}
	}

	testing.expect_value(t, active.count, body_count);
	testing.expect_value(
		t, simulation.solver.active_set.constraint_count, active_constraint_count,
	);
	testing.expect_value(
		t, simulation.narrow_phase.pair_cache.mapping.count, active_pair_count,
	);
	testing.expect_value(t, simulation.narrow_phase.pair_cache.inactive_count, 0);
	testing.expect_value(t, simulation.sleeper.inactive_constraint_count, 0);
	for handle in body_handles
	{
		location, resolve_status := physics.bodies_resolve(&simulation.bodies, handle);
		if !testing.expect_value(t, resolve_status, physics.Physics_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, location.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
		list := &active.constraints.memory[location.index];
		testing.expect(t, list.count >= 2);
	}

	stage_status = physics.simulation_timestep(simulation, dt);
	testing.expect_value(t, stage_status, physics.Physics_Status.Ok);
}

run_reused_pool_contact_awakening_iteration :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool, worker_count, iteration: int,
) -> bool
{
	description := physics.simulation_create_description_default(pool);
	description.default_pose_context.gravity = {};
	description.default_pose_context.linear_damping = 0;
	description.default_pose_context.angular_damping = 0;
	description.solve_description = {
		velocity_iteration_count=2,
		substep_count=1,
		fallback_batch_threshold=1,
	};
	description.allocation_sizes = {
		bodies=2,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=1,
		constraints=2,
		constraint_batches=2,
		initial_constraints_per_type_batch=1,
		minimum_constraints_per_body=1,
		broad_phase_candidates=2,
		pairs=2,
		collision_child_pairs=i32(max(worker_count, 2)),
		inactive_pairs=1,
		pending_pairs_per_worker=2,
		workers=i32(worker_count),
	};
	simulation_memory, allocation_error := mem.alloc(
		size_of(physics.Simulation), align_of(physics.Simulation),
	);
	if !testing.expectf(
		t, allocation_error == nil && simulation_memory != nil,
		"reuse iteration %d failed to allocate simulation", iteration,
	)
	{
		return false;
	}
	simulation := (^physics.Simulation)(simulation_memory);
	simulation^ = {};
	create_result := physics.simulation_create(simulation, &description);
	if !testing.expectf(
		t, create_result.status == .Ok,
		"reuse iteration %d create failed with %v", iteration, create_result.status,
	)
	{
		_ = mem.free(simulation_memory);
		return false;
	}

	dispatcher: util.Thread_Dispatcher;
	dispatcher_ready := false;
	if worker_count > 1
	{
		dispatch_status := util.thread_dispatcher_initialize(
			&dispatcher, worker_count, 65536,
		);
		if !testing.expectf(
			t, dispatch_status == .Ok,
			"reuse iteration %d dispatcher failed with %v",
			iteration, dispatch_status,
		)
		{
			_ = physics.simulation_destroy(simulation);
			_ = mem.free(simulation_memory);
			return false;
		}
		dispatcher_ready = true;
	}
	defer
	{
		if dispatcher_ready
		{
			_ = util.thread_dispatcher_shutdown(&dispatcher);
		}
		if simulation.state == .Ready
		{
			_ = physics.simulation_destroy(simulation);
		}
		_ = mem.free(simulation_memory);
	}
	boundary: ^util.Thread_Dispatcher_Boundary;
	if dispatcher_ready
	{
		boundary = util.thread_dispatcher_boundary(&dispatcher);
	}

	box := physics.Box{0.45, 0.45, 0.45};
	shape, shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	if !testing.expectf(
		t, shape_status == .Ok,
		"reuse iteration %d shape failed with %v", iteration, shape_status,
	)
	{
		return false;
	}
	inertia, inertia_status := physics.box_inertia(box, 1);
	if !testing.expectf(
		t, inertia_status == .Ok,
		"reuse iteration %d inertia failed with %v", iteration, inertia_status,
	)
	{
		return false;
	}

	SET_COUNT :: 2;
	BODIES_PER_SET :: 4;
	SLEEPING_BODY_COUNT :: SET_COUNT * BODIES_PER_SET;
	TOTAL_BODY_COUNT :: SLEEPING_BODY_COUNT * 2;
	sleeping_handles: [SLEEPING_BODY_COUNT]physics.Body_Handle;
	body_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity()},
		local_inertia=inertia,
		collidable={
			shape=shape,
			continuity=physics.continuous_detection_passive(),
			maximum_speculative_margin=f32(math.F32_MAX),
		},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=32},
	};
	for body_index in 0 ..< SLEEPING_BODY_COUNT
	{
		body_description.pose.position = {f32(body_index) * 4, 0, 0};
		handle, body_status := physics.simulation_add_body(
			simulation, &body_description,
		);
		if !testing.expectf(
			t, body_status == .Ok,
			"reuse iteration %d sleeping body %d failed with %v",
			iteration, body_index, body_status,
		)
		{
			return false;
		}
		sleeping_handles[body_index] = handle;
	}
	for body_index in 0 ..< SLEEPING_BODY_COUNT
	{
		body_description.pose.position = {f32(body_index) * 4, 0, 0};
		_, body_status := physics.simulation_add_body(
			simulation, &body_description,
		);
		if !testing.expectf(
			t, body_status == .Ok,
			"reuse iteration %d active body %d failed with %v",
			iteration, body_index, body_status,
		)
		{
			return false;
		}
	}

	capacity_status := physics.island_sleeper_ensure_capacity(
		&simulation.sleeper,
		max(int(simulation.bodies.handle_to_location.length), SLEEPING_BODY_COUNT),
		max(int(simulation.solver.handle_to_constraint.length), 1),
	);
	if !testing.expectf(
		t, capacity_status == .Ok,
		"reuse iteration %d sleeper growth failed with %v",
		iteration, capacity_status,
	)
	{
		return false;
	}
	scaffold := &simulation.sleeper.scaffold;
	for set_index in 0 ..< SET_COUNT
	{
		for body_index in 0 ..< BODIES_PER_SET
		{
			scaffold.island_bodies.memory[body_index] =
				sleeping_handles[set_index * BODIES_PER_SET + body_index];
		}
		sleep_status := physics.island_sleeper_sleep_island(
			&simulation.sleeper, scaffold, BODIES_PER_SET, 0, boundary,
		);
		if !testing.expectf(
			t, sleep_status == .Ok,
			"reuse iteration %d sleep set %d failed with %v",
			iteration, set_index, sleep_status,
		)
		{
			return false;
		}
	}
	active := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	if !testing.expectf(
		t, active.count == SLEEPING_BODY_COUNT,
		"reuse iteration %d expected %d active bodies before contact, got %d",
		iteration, SLEEPING_BODY_COUNT, active.count,
	)
	{
		return false;
	}

	step_status := physics.simulation_timestep(
		simulation, f32(1.0 / 60.0), boundary,
	);
	if !testing.expectf(
		t, step_status == .Ok,
		"reuse iteration %d timestep failed with %v stage=%v",
		iteration, step_status, simulation.narrow_phase.last_stage,
	)
	{
		return false;
	}
	if !testing.expectf(
		t, active.count == TOTAL_BODY_COUNT,
		"reuse iteration %d expected %d awakened bodies, got %d",
		iteration, TOTAL_BODY_COUNT, active.count,
	)
	{
		return false;
	}
	inactive_set_count := 0;
	for set_index in 1 ..< int(simulation.bodies.sets.length)
	{
		set := &simulation.bodies.sets.memory[set_index];
		if set.state == .Allocated && set.count > 0
		{
			inactive_set_count += 1;
		}
	}
	if !testing.expectf(
		t, inactive_set_count == 0,
		"reuse iteration %d left %d inactive sets",
		iteration, inactive_set_count,
	)
	{
		return false;
	}
	if !testing.expectf(
		t, simulation.narrow_phase.pair_cache.mapping.count >= SLEEPING_BODY_COUNT,
		"reuse iteration %d expected at least %d pairs, got %d",
		iteration, SLEEPING_BODY_COUNT,
		simulation.narrow_phase.pair_cache.mapping.count,
	)
	{
		return false;
	}
	return true;
}

@(test)
simulation_pool_reuse_preserves_contact_awakening_growth :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&pool, 4096, 2), util.Memory_Status.Ok,
	)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);

	worker_counts := [2]int{1, 4};
	for worker_count in worker_counts
	{
		for iteration in 0 ..< 3
		{
			if !run_reused_pool_contact_awakening_iteration(
				t, &pool, worker_count, iteration,
			)
			{
				return;
			}
		}
	}
}
