package simulation_tests

import "core:math"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

contact_execution_description :: proc($T: typeid, changed: physics.Reference_State = .Missing) -> T
{
	c0: physics.Constraint_Contact_Data = {{0.25, -0.3, 0.1}, 0.03};
	c1: physics.Constraint_Contact_Data = {{-0.2, -0.25, 0.35}, -0.01};
	c2: physics.Constraint_Contact_Data = {{0.4, -0.2, -0.15}, 0.05};
	c3: physics.Constraint_Contact_Data = {{-0.35, -0.15, -0.25}, 0.015};
	normal: util.Vector3 = {0, 1, 0};
	offset_b: util.Vector3 = {0.4, -0.1, 0.2};
	material: physics.Contact_Material_Properties = {
		friction_coefficient=0.65,
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	if changed == .Present
	{
		c0 = {{-0.15, 0.25, 0.3}, -0.02};
		c1 = {{0.35, -0.15, -0.2}, 0.04};
		c2 = {{-0.25, -0.3, 0.1}, -0.03};
		c3 = {{0.2, 0.15, -0.4}, 0.025};
		normal = {0.6, 0.8, 0};
		offset_b = {-0.25, 0.15, -0.1};
		material = {
			friction_coefficient=0.4,
			spring_settings={angular_frequency=14, twice_damping_ratio=1.5},
			maximum_recovery_velocity=1.25,
		};
	}
	when T == physics.Contact_1_One_Body
	{
		return {c0, normal, material};
	}
	else when T == physics.Contact_2_One_Body
	{
		return {c0, c1, normal, material};
	}
	else when T == physics.Contact_3_One_Body
	{
		return {c0, c1, c2, normal, material};
	}
	else when T == physics.Contact_4_One_Body
	{
		return {c0, c1, c2, c3, normal, material};
	}
	else when T == physics.Contact_1
	{
		return {c0, offset_b, normal, material};
	}
	else when T == physics.Contact_2
	{
		return {c0, c1, offset_b, normal, material};
	}
	else when T == physics.Contact_3
	{
		return {c0, c1, c2, offset_b, normal, material};
	}
	else
	{
		#assert(T == physics.Contact_4);
		return {c0, c1, c2, c3, offset_b, normal, material};
	}
}

contact_execution_populate :: proc(
	t: ^testing.T, world: ^physics.Simulation, description: ^$T,
	count: int, mode: physics.Solver_Integration_Mode,
) -> physics.Physics_Status
{
	type_id: int = int(physics.constraint_description_type_id(T));
	record: ^physics.Constraint_Type_Record = &world.solver.registry.records[type_id];
	body_count: int = int(record.body_count);
	handles: [22]physics.Body_Handle;
	targets: [11]physics.Constraint_Handle;
	for lane in 0 ..< count
	{
		for role in 0 ..< body_count
		{
			body: physics.Body_Description = dynamic_body();
			body.pose.position = {f32(lane) * 3 + f32(role) * 0.4, f32(role) * 0.5, f32(lane) * 0.1};
			body.pose.orientation = {0.6, 0, 0, 0.8};
			if role == 1
			{
				body.pose.orientation = {0, 0.6, 0, 0.8};
			}
			body.velocity = {
				linear={0.2 + f32(role) * 0.1, -0.15 + f32(role) * 0.2, 0.12},
				angular={0.1, 0.2, -0.15},
			};
			body.local_inertia = {
				inverse_mass=0.5 + f32(role) * 0.2,
				inverse_inertia_tensor={0.7, 0, 1.3, 0, 0, 1.9},
			};
			if role == 1 && lane % 4 == 3
			{
				body.local_inertia = {};
			}
			body.activity.sleep_threshold = -1;
			status: physics.Physics_Status;
			handles[lane * body_count + role], status = physics.simulation_add_body(world, &body);
			if status != .Ok
			{
				return status;
			}
		}
	}
	// previous contacts assign first-touch ownership to batch zero. moving the
	// untouched target lanes into batch one also exercises one-body partial masks
	if mode != .Always
	{
		for lane in 0 ..< count
		{
			if mode == .Conditional && lane % 3 != 0
			{
				continue;
			}
			connected: [4]physics.Body_Handle;
			for role in 0 ..< body_count
			{
				connected[role] = handles[lane * body_count + role];
			}
			_, status := physics.simulation_add_constraint(world, &connected, description);
			if status != .Ok
			{
				return status;
			}
		}
	}
	for lane in 0 ..< count
	{
		connected: [4]physics.Body_Handle;
		for role in 0 ..< body_count
		{
			connected[role] = handles[lane * body_count + role];
		}
		status: physics.Physics_Status;
		targets[lane], status = physics.simulation_add_constraint(world, &connected, description);
		if status != .Ok
		{
			return status;
		}
	}
	batch_index: int = 0;
	if mode != .Always
	{
		batch_index = 1;
		for lane in 0 ..< count
		{
			location, status := physics.solver_resolve(&world.solver, targets[lane]);
			if status != .Ok
			{
				return status;
			}
			if location.batch_index != 1
			{
				status = physics.solver_move_constraint(&world.solver, targets[lane], 1);
				if status != .Ok
				{
					return status;
				}
			}
		}
	}
	status: physics.Physics_Status = physics.solver_prepare_integration_responsibilities_serial(&world.solver);
	if status != .Ok
	{
		return status;
	}
	batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[batch_index];
	type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
	testing.expect_value(t, type_batch.count, i32(count));
	if mode == .Never
	{
		testing.expect_value(t, type_batch.has_integration_responsibilities, physics.Reference_State.Missing);
	}
	else
	{
		testing.expect_value(t, type_batch.has_integration_responsibilities, physics.Reference_State.Present);
	}
	if mode == .Conditional
	{
		bundle_mode, mask := physics.solver_integration_mask(&world.solver, type_batch, 0, 0, util.I32x8(-1));
		testing.expect_value(t, bundle_mode, physics.Solver_Bundle_Integration_Mode.Partial);
		lanes: [8]i32 = transmute([8]i32)mask;
		integrated: int;
		for lane in lanes
		{
			if lane != 0
			{
				integrated += 1;
			}
		}
		testing.expect(t, integrated > 0 && integrated < 8);
	}
	// include nonzero tangent, penetration and twist values, including unused lanes.
	// the existing Prestep must mask the tail before Warmstart and every Solve
	for bundle_index in 0 ..< (count + 7) / 8
	{
		impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, bundle_index * 8));
		for field in 0 ..< int(record.impulse_bundle_size) / size_of(util.F32x8)
		{
			impulses[field] = util.F32x8(0.025 + f32(field) * 0.01);
		}
		impulses[1] = util.F32x8(-0.018);
	}
	return .Ok;
}

contact_execution_expect_data :: proc(
	t: ^testing.T, actual, expected: ^physics.Simulation,
	block: ^physics.Solver_Work_Block, description: ^$T,
	stage: physics.Solver_Execution_Stage,
)
{
	actual_batch: ^physics.Type_Batch = &actual.solver.active_set.batches.memory[block.batch_index].type_batches.memory[block.type_batch_index];
	expected_batch: ^physics.Type_Batch = &expected.solver.active_set.batches.memory[block.batch_index].type_batches.memory[block.type_batch_index];
	type_id: int = int(actual_batch.type_id);
	testing.expect_value(t, actual_batch.count, expected_batch.count);
	record: ^physics.Constraint_Type_Record = &expected.solver.registry.records[type_id];
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		a: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(actual_batch, bundle_index * 8));
		b: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(expected_batch, bundle_index * 8));
		for field in 0 ..< int(record.impulse_bundle_size) / size_of(util.F32x8)
		{
			av: [8]f32 = transmute([8]f32)a[field];
			bv: [8]f32 = transmute([8]f32)b[field];
			for lane in 0 ..< 8
			{
				if !testing.expectf(t, math.abs(av[lane] - bv[lane]) <= 0.0002,
					"type %d stage %v bundle %d field %d lane %d impulse %g versus %g",
					type_id, stage, bundle_index, field, lane, av[lane], bv[lane])
				{
					return;
				}
				if bundle_index * 8 + lane >= int(actual_batch.count) && (stage == .Warmstart || stage == .Solve)
				{
					testing.expect_value(t, av[lane], f32(0));
				}
			}
		}
	}
	for index in 0 ..< int(actual_batch.count)
	{
		a_handle: physics.Constraint_Handle = {value=actual_batch.index_to_handle.memory[index]};
		b_handle: physics.Constraint_Handle = {value=expected_batch.index_to_handle.memory[index]};
		testing.expect_value(t, a_handle, b_handle);
		a_location, a_status := physics.solver_resolve(&actual.solver, a_handle);
		b_location, b_status := physics.solver_resolve(&expected.solver, b_handle);
		testing.expect_value(t, a_status, physics.Physics_Status.Ok);
		testing.expect_value(t, b_status, physics.Physics_Status.Ok);
		testing.expect_value(t, a_location, b_location);
		a_description, b_description: T;
		testing.expect_value(t, physics.solver_get_description_raw(
			&actual.solver, a_handle, i32(type_id), &a_description, size_of(T),
		), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.solver_get_description_raw(
			&expected.solver, b_handle, i32(type_id), &b_description, size_of(T),
		), physics.Physics_Status.Ok);
		// these eight authored descriptions consist solely of defined f32 fields.
		// do not read or compare any derived tail or storage padding
		a_fields: [^]f32 = ([^]f32)(&a_description);
		b_fields: [^]f32 = ([^]f32)(&b_description);
		for field in 0 ..< size_of(T) / size_of(f32)
		{
			testing.expectf(t, math.abs(a_fields[field] - b_fields[field]) <= 0.0002,
				"type %d stage %v handle %d description field %d", type_id, stage, a_handle.value, field);
		}
	}
}

contact_execution_substep :: proc(
	t: ^testing.T, actual, expected: ^physics.Simulation,
	actual_callbacks, expected_callbacks: ^Ball_Socket_Callback_State,
	description: ^$T, mode: physics.Solver_Integration_Mode,
	dt: f32, phase: physics.Solver_Substep_Phase,
	velocity_iterations: int = 4,
)
{
	actual_callbacks^ = {};
	expected_callbacks^ = {};
	worlds: [2]^physics.Simulation = {actual, expected};
	type_id: int = int(physics.constraint_description_type_id(T));
	batch_index: int = 0;
	if mode != .Always
	{
		batch_index = 1;
	}
	blocks: [2]physics.Solver_Work_Block;
	batches: [2]^physics.Type_Batch;
	for world, index in worlds
	{
		testing.expect_value(t, physics.solver_prepare_integration_responsibilities_serial(&world.solver), physics.Physics_Status.Ok);
		batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[batch_index];
		type_index: i16 = batch.type_id_to_batch_index[type_id];
		batches[index] = &batch.type_batches.memory[type_index];
		blocks[index] = {
			batch_index=i16(batch_index), type_batch_index=type_index,
			start_bundle=0, end_bundle=(batches[index].count + 7) / 8,
		};
	}
	testing.expect_value(t, blocks[0], blocks[1]);
	record: ^physics.Constraint_Type_Record = &expected.solver.registry.records[type_id];
	prepare_status, _ := physics.solver_prepare_work_blocks(&actual.solver, 4);
	if !testing.expect_value(t, prepare_status, physics.Physics_Status.Ok)
	{
		return;
	}
	job: physics.Solver_Substep_Job = {
		solver=&actual.solver, dt=dt, inverse_dt=1 / dt, phase=phase,
		velocity_iteration_count=velocity_iterations,
	};
	if !testing.expect_value(t, physics.solver_prepare_contact_coefficient_cache(&job), physics.Physics_Status.Ok)
	{
		return;
	}
	// both paths run the earlier batch stages before testing a later batch
	if mode != .Always
	{
		for world in worlds
		{
			prior_batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
			prior_type: ^physics.Type_Batch = &prior_batch.type_batches.memory[prior_batch.type_id_to_batch_index[type_id]];
			prior_stages: [3]physics.Solver_Execution_Stage = {.Incremental_Update, .Integrate_Constrained_Kinematics, .Warmstart};
			for stage in prior_stages
			{
				for bundle_index in 0 ..< (int(prior_type.count) + 7) / 8
				{
					testing.expect_value(t, physics.solver_execute_type_bundle(
						&world.solver, prior_type, record, 0, bundle_index,
						dt, job.inverse_dt, phase, stage, 3,
					), physics.Physics_Status.Ok);
				}
			}
		}
		ball_socket_expect_state(t, actual, expected, actual_callbacks, expected_callbacks);
		actual_callbacks^ = {};
		expected_callbacks^ = {};
	}
	observed_reference, reference_status := physics.type_batch_read_reference(batches[0], &actual.bodies, 0);
	testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
	observed_handle: physics.Body_Handle = observed_reference.body_handles[0];
	initial_body, initial_body_status := physics.bodies_get_description(&actual.bodies, observed_handle);
	testing.expect_value(t, initial_body_status, physics.Physics_Status.Ok);
	initial_velocity: physics.Body_Velocity = initial_body.velocity;
	initial_stages: [3]physics.Solver_Execution_Stage = {
		.Incremental_Update, .Integrate_Constrained_Kinematics, .Warmstart,
	};
	for stage_index in 0 ..< len(initial_stages) + velocity_iterations
	{
		stage: physics.Solver_Execution_Stage = .Solve;
		if stage_index < len(initial_stages)
		{
			stage = initial_stages[stage_index];
		}
		actual_batch: ^physics.Constraint_Batch = &actual.solver.active_set.batches.memory[batch_index];
		for block_index in int(actual_batch.work_block_start) ..<
			int(actual_batch.work_block_start + actual_batch.work_block_count)
		{
			// the cached route resolves its offset from this normal block
			owned_block: ^physics.Solver_Work_Block = &actual.solver.work_blocks.memory[block_index];
			testing.expect_value(t, physics.solver_execute_work_block(&job, owned_block, stage, 3), physics.Physics_Status.Ok);
		}
		for bundle_index in 0 ..< int(blocks[1].end_bundle)
		{
			testing.expect_value(t, physics.solver_execute_type_bundle(
				&expected.solver, batches[1], record, batch_index, bundle_index,
				dt, job.inverse_dt, phase, stage, 3,
			), physics.Physics_Status.Ok);
		}
		contact_execution_expect_data(t, actual, expected, &blocks[0], description, stage);
		ball_socket_expect_state(t, actual, expected, actual_callbacks, expected_callbacks);
		if stage == .Incremental_Update || mode == .Never
		{
			testing.expect_value(t, actual_callbacks.count, 0);
		}
	}
	if mode != .Never
	{
		testing.expect(t, actual_callbacks.count > 0);
	}
	final_body, final_body_status := physics.bodies_get_description(&actual.bodies, observed_handle);
	testing.expect_value(t, final_body_status, physics.Physics_Status.Ok);
	final_velocity: physics.Body_Velocity = final_body.velocity;
	testing.expect(t, initial_velocity.linear != final_velocity.linear || initial_velocity.angular != final_velocity.angular);
}

contact_execution_compare_case :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool, description: ^$T, count: int,
	mode: physics.Solver_Integration_Mode, phase: physics.Solver_Substep_Phase,
	angular_mode: physics.Angular_Integration_Mode,
)
{
	actual: ^physics.Simulation = allocate_simulation(t);
	if actual == nil
	{
		return;
	}
	defer free_simulation(t, actual);
	expected: ^physics.Simulation = allocate_simulation(t);
	if expected == nil
	{
		return;
	}
	defer free_simulation(t, expected);
	creation: physics.Simulation_Create_Description = small_description(pool);
	creation.allocation_sizes.bodies = 64;
	creation.allocation_sizes.workers = 8;
	creation.solve_description.substep_count = 1;
	if !testing.expect_value(t, physics.simulation_create(actual, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(actual);
	if !testing.expect_value(t, physics.simulation_create(expected, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(expected);
	actual_callbacks, expected_callbacks: Ball_Socket_Callback_State;
	worlds: [2]^physics.Simulation = {actual, expected};
	callbacks: [2]^Ball_Socket_Callback_State = {&actual_callbacks, &expected_callbacks};
	for world, index in worlds
	{
		world.integrator.callbacks.integrate_velocity = ball_socket_test_velocity;
		world.integrator.callbacks.user_context = callbacks[index];
		world.integrator.callbacks.angular_mode = angular_mode;
		world.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
		if !testing.expect_value(t, contact_execution_populate(t, world, description, count, mode), physics.Physics_Status.Ok)
		{
			return;
		}
	}
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, description, mode, 1.0 / 60.0, phase);
}

contact_execution_type_cases :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, $T: typeid)
{
	description: T = contact_execution_description(T);
	counts: [2]int = {8, 11};
	modes: [3]physics.Solver_Integration_Mode = {.Always, .Conditional, .Never};
	phases: [2]physics.Solver_Substep_Phase = {.First, .Continuation};
	angular_modes: [3]physics.Angular_Integration_Mode = {
		.Nonconserving, .Conserve_Momentum, .Conserve_Momentum_With_Gyroscopic_Torque,
	};
	for count in counts
	{
		for mode in modes
		{
			for phase in phases
			{
				for angular_mode in angular_modes
				{
					contact_execution_compare_case(t, pool, &description, count, mode, phase, angular_mode);
				}
			}
		}
	}
}

@(test)
convex_contact_staged_execution_matches_registry_and_preserves_integration :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	contact_execution_type_cases(t, &pool, physics.Contact_1_One_Body);
	contact_execution_type_cases(t, &pool, physics.Contact_2_One_Body);
	contact_execution_type_cases(t, &pool, physics.Contact_3_One_Body);
	contact_execution_type_cases(t, &pool, physics.Contact_4_One_Body);
	contact_execution_type_cases(t, &pool, physics.Contact_1);
	contact_execution_type_cases(t, &pool, physics.Contact_2);
	contact_execution_type_cases(t, &pool, physics.Contact_3);
	contact_execution_type_cases(t, &pool, physics.Contact_4);
}

contact_execution_sleep_and_awaken :: proc(t: ^testing.T, world: ^physics.Simulation, description: ^$T)
{
	type_id: int = int(physics.constraint_description_type_id(T));
	batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
	type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
	handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[0]};
	reference, reference_status := physics.type_batch_read_reference(type_batch, &world.bodies, 0);
	if !testing.expect_value(t, reference_status, physics.Physics_Status.Ok)
	{
		return;
	}
	saved_description: T;
	testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &saved_description), physics.Physics_Status.Ok);
	saved_impulses: [7]f32;
	impulse_count: int = int(type_batch.impulse_bundle_size) / size_of(util.F32x8);
	if !testing.expect_value(t, impulse_count, len(saved_impulses))
	{
		return;
	}
	impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, 0));
	for field in 0 ..< impulse_count
	{
		lanes: [8]f32 = transmute([8]f32)impulses[field];
		saved_impulses[field] = lanes[0];
	}
	// only this connected component becomes eligible for sleep. other bodies have
	// sleeping disabled. the ordinary sleeper handles traversal and both migrations
	for body_index in 0 ..< int(reference.body_count)
	{
		body, status := physics.bodies_get_description(&world.bodies, reference.body_handles[body_index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		body.activity = {sleep_threshold=1000000, minimum_timestep_count_under_threshold=1};
		testing.expect_value(t, physics.simulation_apply_body_description(world, reference.body_handles[body_index], &body), physics.Physics_Status.Ok);
		location, resolve_status := physics.bodies_resolve(&world.bodies, reference.body_handles[body_index]);
		testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
		active: ^physics.Body_Set = &world.bodies.sets.memory[location.set_index];
		// broad-phase prediction normally updates sleep candidacy here.
		// invoke it directly so the expected world never runs a cached Solve
		physics.island_sleeper_update_candidacy(
			&active.activity.memory[location.index], active.dynamics_state.memory[location.index].motion.velocity,
		);
	}
	world.sleeper.tested_fraction_per_frame = 1;
	world.sleeper.target_slept_fraction = 1;
	world.sleeper.target_traversed_fraction = 1;
	if !testing.expect_value(t, physics.island_sleeper_update(&world.sleeper), physics.Physics_Status.Ok)
	{
		return;
	}
	for body_index in 0 ..< int(reference.body_count)
	{
		location, status := physics.bodies_resolve(&world.bodies, reference.body_handles[body_index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect(t, location.set_index > 0);
	}
	inactive_index, inactive_status := physics.island_sleeper_resolve_inactive_constraint_index(&world.sleeper, handle);
	if !testing.expect_value(t, inactive_status, physics.Physics_Status.Ok)
	{
		return;
	}
	inactive: ^physics.Inactive_Constraint_Record = &world.sleeper.inactive_constraints.memory[inactive_index];
	testing.expect_value(t, inactive.type_id, i32(type_id));
	testing.expect_value(t, inactive.impulse_count, i32(impulse_count));
	for field in 0 ..< impulse_count
	{
		testing.expect_value(t, inactive.impulses[field], saved_impulses[field]);
	}
	sleeping_description: T;
	testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &sleeping_description), physics.Physics_Status.Ok);
	testing.expect_value(t, sleeping_description, saved_description);
	awake_body, awake_status := physics.bodies_get_description(&world.bodies, reference.body_handles[0]);
	testing.expect_value(t, awake_status, physics.Physics_Status.Ok);
	awake_body.activity = {sleep_threshold=-1, minimum_timestep_count_under_threshold=32};
	if !testing.expect_value(t, physics.simulation_apply_body_description(world, reference.body_handles[0], &awake_body), physics.Physics_Status.Ok)
	{
		return;
	}
	for body_index in 0 ..< int(reference.body_count)
	{
		location, status := physics.bodies_resolve(&world.bodies, reference.body_handles[body_index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, location.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
	}
	awake_description: T;
	testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &awake_description), physics.Physics_Status.Ok);
	testing.expect_value(t, awake_description, saved_description);
	location, status := physics.solver_resolve(&world.solver, handle);
	if !testing.expect_value(t, status, physics.Physics_Status.Ok)
	{
		return;
	}
	testing.expect_value(t, location.type_id, i32(type_id));
	testing.expect_value(t, location.batch_index, i32(0));
	// resolve storage again: sleep removal and restoration may change lane order
	batch = &world.solver.active_set.batches.memory[location.batch_index];
	type_batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
	impulses = ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, int(location.index_in_type_batch)));
	for field in 0 ..< impulse_count
	{
		lanes: [8]f32 = transmute([8]f32)impulses[field];
		testing.expect_value(t, lanes[int(location.index_in_type_batch) % 8], saved_impulses[field]);
	}
	restored, restored_status := physics.type_batch_read_reference(type_batch, &world.bodies, int(location.index_in_type_batch));
	testing.expect_value(t, restored_status, physics.Physics_Status.Ok);
	for body_index in 0 ..< int(reference.body_count)
	{
		testing.expect_value(t, restored.body_handles[body_index], reference.body_handles[body_index]);
	}
}

contact_execution_lifetime_case :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, $T: typeid)
{
	actual: ^physics.Simulation = allocate_simulation(t);
	if actual == nil
	{
		return;
	}
	defer free_simulation(t, actual);
	expected: ^physics.Simulation = allocate_simulation(t);
	if expected == nil
	{
		return;
	}
	defer free_simulation(t, expected);
	creation: physics.Simulation_Create_Description = small_description(pool);
	creation.allocation_sizes.bodies = 64;
	creation.allocation_sizes.workers = 8;
	if !testing.expect_value(t, physics.simulation_create(actual, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(actual);
	if !testing.expect_value(t, physics.simulation_create(expected, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(expected);
	description: T = contact_execution_description(T);
	type_id: int = int(physics.constraint_description_type_id(T));
	actual_callbacks, expected_callbacks: Ball_Socket_Callback_State;
	worlds: [2]^physics.Simulation = {actual, expected};
	callbacks: [2]^Ball_Socket_Callback_State = {&actual_callbacks, &expected_callbacks};
	for world, index in worlds
	{
		world.integrator.callbacks.integrate_velocity = ball_socket_test_velocity;
		world.integrator.callbacks.user_context = callbacks[index];
		world.integrator.callbacks.angular_mode = .Conserve_Momentum_With_Gyroscopic_Torque;
		world.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
		if !testing.expect_value(t, contact_execution_populate(t, world, &description, 11, .Always), physics.Physics_Status.Ok)
		{
			return;
		}
	}
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 60.0, .First, 1);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, nil);
	testing.expect_value(t, actual.solver.contact_coefficients.memory, nil);
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 90.0, .Continuation);
	testing.expect(t, actual.solver.contact_coefficient_offsets.length > 0);
	testing.expect(t, actual.solver.contact_coefficients.length > 0);
	coefficient_storage: [^]util.F32x8 = actual.solver.contact_coefficients.memory;
	offset_storage: [^]i32 = actual.solver.contact_coefficient_offsets.memory;
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 100.0, .Continuation, 1);
	testing.expect_value(t, actual.solver.contact_coefficients.memory, coefficient_storage);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, offset_storage);

	// change every coefficient input before the next Warmstart. sign-crossing
	// depths also alter friction-center weights. body application refreshes World
	description = contact_execution_description(T, .Present);
	for world in worlds
	{
		batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
		type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
		for index in 0 ..< int(type_batch.count)
		{
			handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[index]};
			testing.expect_value(t, physics.simulation_apply_constraint_description(world, handle, &description), physics.Physics_Status.Ok);
			readback: T;
			testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &readback), physics.Physics_Status.Ok);
			testing.expect_value(t, readback, description);
		}
		active: ^physics.Body_Set = &world.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
		for index in 0 ..< int(active.count)
		{
			handle: physics.Body_Handle = active.index_to_handle.memory[index];
			body, status := physics.bodies_get_description(&world.bodies, handle);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			body.pose.position.y += 0.12;
			body.pose.orientation = {0, 0, 0.6, 0.8};
			if body.local_inertia.inverse_mass > 0
			{
				body.local_inertia.inverse_mass = 0.9;
				body.local_inertia.inverse_inertia_tensor = {1.2, 0, 0.8, 0, 0, 1.6};
			}
			testing.expect_value(t, physics.simulation_apply_body_description(world, handle, &body), physics.Physics_Status.Ok);
		}
	}
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 75.0, .First);

	// all strictly negative depths select the uniform friction-center fallback.
	// an exactly zero depth has weight one and must select contact_0 instead.
	// later phases cross the basis sign and bias-clipping boundaries while the
	// existing impulses remain live across each geometry/material replacement
	for phase_index in 0 ..< 4
	{
		phase_dt: f32 = 1.0 / 75.0;
		description.contact_0.penetration_depth = -0.01;
		description.contact_1.penetration_depth = -0.02;
		description.contact_2.penetration_depth = -0.03;
		description.contact_3.penetration_depth = -0.04;
		if phase_index == 1
		{
			description.contact_0.penetration_depth = 0;
		}
		else if phase_index == 2
		{
			description.normal = {0.36, 0.48, -0.8};
			description.contact_0.penetration_depth = 1.5;
			description.contact_1.penetration_depth = 0.02;
			description.contact_3.penetration_depth = 0;
			description.material.maximum_recovery_velocity = 0.125;
			phase_dt = 1.0 / 30.0;
		}
		else if phase_index == 3
		{
			// negative zero must use the same basis branch as positive zero
			description.normal = {0.6, 0.8, transmute(f32)u32(0x80000000)};
			description.contact_0.penetration_depth = 0.015;
			description.contact_2.penetration_depth = 0;
			description.contact_3.penetration_depth = 0.03;
			description.material.maximum_recovery_velocity = 4;
			phase_dt = 1.0 / 120.0;
		}
		for world in worlds
		{
			batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
			type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
			for index in 0 ..< int(type_batch.count)
			{
				handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[index]};
				testing.expect_value(t, physics.simulation_apply_constraint_description(world, handle, &description), physics.Physics_Status.Ok);
				readback: T;
				testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &readback), physics.Physics_Status.Ok);
				testing.expect_value(t, readback, description);
				reference, reference_status := physics.type_batch_read_reference(type_batch, &world.bodies, index);
				if !testing.expect_value(t, reference_status, physics.Physics_Status.Ok)
				{
					return;
				}
				for body_index in 0 ..< int(reference.body_count)
				{
					body_handle: physics.Body_Handle = reference.body_handles[body_index];
					body, body_status := physics.bodies_get_description(&world.bodies, body_handle);
					if !testing.expect_value(t, body_status, physics.Physics_Status.Ok)
					{
						return;
					}
					if physics.body_inertia_mobility(body.local_inertia) == .Kinematic
					{
						continue;
					}
					// closing speed exceeds the speculative separation bias. tangential
					// and angular motion make friction observable in every depth phase
					body.velocity.linear = util.vector3_add(
						util.vector3_scale(description.normal, -8), util.Vector3{0, 0, 2},
					);
					body.velocity.angular = {0.4, -0.3, 1.2};
					if body_index == 1
					{
						body.velocity.linear = util.vector3_add(
							util.vector3_scale(description.normal, 8), util.Vector3{0, 0, -1},
						);
						body.velocity.angular = {-0.2, 0.5, -0.8};
					}
					testing.expect_value(t, physics.simulation_apply_body_description(world, body_handle, &body), physics.Physics_Status.Ok);
				}
			}
		}
		contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, phase_dt, .First);
		for world in worlds
		{
			batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
			type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
			impulses: ^physics.Contact_4_Accumulated_Impulses = (^physics.Contact_4_Accumulated_Impulses)(physics.type_batch_impulse_bundle(type_batch, 0));
			total_penetration: f32 = 0;
			for contact_index in 0 ..< 4
			{
				lanes: [8]f32 = transmute([8]f32)impulses.penetration[contact_index];
				total_penetration += lanes[0];
			}
			tangent_x: [8]f32 = transmute([8]f32)impulses.tangent.x;
			tangent_y: [8]f32 = transmute([8]f32)impulses.tangent.y;
			testing.expectf(t, total_penetration > 0.0002,
				"type %d depth phase %d must produce contact impulse", type_id, phase_index);
			testing.expectf(t, math.abs(tangent_x[0]) + math.abs(tangent_y[0]) > 0.0002,
				"type %d depth phase %d must produce friction impulse", type_id, phase_index);
		}
	}

	remaining: int = 11;
	target_counts: [2]int = {8, 7};
	for target_count in target_counts
	{
		for remaining > target_count
		{
			for world in worlds
			{
				batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
				type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
				moved_handle: i32 = type_batch.index_to_handle.memory[remaining - 1];
				handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[1]};
				testing.expect_value(t, physics.simulation_remove_constraint(world, handle), physics.Physics_Status.Ok);
				testing.expect_value(t, type_batch.count, i32(remaining - 1));
				testing.expect_value(t, type_batch.index_to_handle.memory[1], moved_handle);
			}
			remaining -= 1;
		}
		contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 120.0, .Continuation);
	}
	for world in worlds
	{
		contact_execution_sleep_and_awaken(t, world, &description);
	}
	// restored builtin contacts must refresh coefficients before their next sweeps.
	// the expected world still executes only the registered, recomputing route
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 80.0, .First);
	contact_execution_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, &description, .Always, 1.0 / 100.0, .Continuation);
}

Contact_Cache_Iteration_Schedule :: struct
{
	iterations: [3]i32,
	observed: [3]i32,
	count: int,
}

contact_cache_iteration_schedule :: proc "contextless" (user_context: rawptr, substep_index: int) -> i32
{
	schedule: ^Contact_Cache_Iteration_Schedule = (^Contact_Cache_Iteration_Schedule)(user_context);
	schedule.observed[substep_index] = i32(substep_index);
	schedule.count += 1;
	return schedule.iterations[substep_index];
}

contact_cache_reference_substeps :: proc(
	t: ^testing.T, world: ^physics.Simulation, dt: f32, iterations: []i32,
)
{
	testing.expect_value(t, physics.solver_prepare_integration_responsibilities_serial(&world.solver), physics.Physics_Status.Ok);
	initial_stages: [3]physics.Solver_Execution_Stage = {
		.Incremental_Update, .Integrate_Constrained_Kinematics, .Warmstart,
	};
	for iteration_count, substep_index in iterations
	{
		phase: physics.Solver_Substep_Phase = .First;
		if substep_index > 0
		{
			phase = .Continuation;
		}
		for stage_index in 0 ..< len(initial_stages) + int(iteration_count)
		{
			stage: physics.Solver_Execution_Stage = .Solve;
			if stage_index < len(initial_stages)
			{
				stage = initial_stages[stage_index];
			}
			for batch_index in 0 ..< int(world.solver.active_set.batch_count)
			{
				batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[batch_index];
				for type_index in 0 ..< int(batch.type_batch_count)
				{
					type_batch: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
					record: ^physics.Constraint_Type_Record = &world.solver.registry.records[type_batch.type_id];
					for bundle_index in 0 ..< (int(type_batch.count) + 7) / 8
					{
						// this registered kernel recomputes coefficients on every Solve
						testing.expect_value(t, physics.solver_execute_type_bundle(
							&world.solver, type_batch, record, batch_index, bundle_index,
							dt, 1 / dt, phase, stage, 0,
						), physics.Physics_Status.Ok);
					}
				}
			}
		}
	}
}

contact_cache_validate_caller :: proc "contextless" (
	type_id: i32, description: rawptr,
) -> physics.Physics_Status
{
	_ = type_id;
	return physics.constraint_description_validate((^physics.Contact_4_One_Body)(description));
}

contact_cache_populate_dispatch :: proc(t: ^testing.T, world: ^physics.Simulation, start, end: int)
{
	description: physics.Contact_4_One_Body = contact_execution_description(physics.Contact_4_One_Body);
	for index in start ..< end
	{
		body: physics.Body_Description = dynamic_body();
		body.pose.position = {f32(index) * 3, 0.2, 0};
		body.pose.orientation = {0.6, 0, 0, 0.8};
		body.velocity = {linear={0.2, -0.1, 0.05}, angular={0.1, 0.2, -0.15}};
		body.local_inertia = {inverse_mass=0.5, inverse_inertia_tensor={0.7, 0, 1.3, 0, 0, 1.9}};
		body.activity.sleep_threshold = -1;
		handle, body_status := physics.simulation_add_body(world, &body);
		testing.expect_value(t, body_status, physics.Physics_Status.Ok);
		handles: [4]physics.Body_Handle = {handle, {}, {}, {}};
		_, contact_status := physics.simulation_add_constraint(world, &handles, &description);
		testing.expect_value(t, contact_status, physics.Physics_Status.Ok);
		if index % 3 == 0
		{
			_, prior_status := physics.simulation_add_constraint(world, &handles, &description);
			testing.expect_value(t, prior_status, physics.Physics_Status.Ok);
		}
		if index == 0
		{
			_, custom_status := physics.solver_add_raw(
				&world.solver, physics.FIRST_CALLER_CONSTRAINT_TYPE_ID, &handles, &description,
			);
			testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
		}
	}
}

contact_cache_expect_dispatch :: proc(t: ^testing.T, actual, expected: ^physics.Simulation)
{
	description: physics.Contact_4_One_Body;
	testing.expect_value(t, actual.solver.active_set.batch_count, expected.solver.active_set.batch_count);
	for batch_index in 0 ..< int(actual.solver.active_set.batch_count)
	{
		batch: ^physics.Constraint_Batch = &actual.solver.active_set.batches.memory[batch_index];
		testing.expect_value(t, batch.type_batch_count, expected.solver.active_set.batches.memory[batch_index].type_batch_count);
		for type_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
			block: physics.Solver_Work_Block = {
				batch_index=i16(batch_index), type_batch_index=i16(type_index),
				start_bundle=0, end_bundle=(type_batch.count + 7) / 8,
			};
			contact_execution_expect_data(t, actual, expected, &block, &description, .Solve);
		}
	}
	actual_callbacks, expected_callbacks: Ball_Socket_Callback_State;
	ball_socket_expect_state(t, actual, expected, &actual_callbacks, &expected_callbacks);
}

contact_cache_dispatch_lifetime_case :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, worker_count: int)
{
	actual: ^physics.Simulation = allocate_simulation(t);
	if actual == nil
	{
		return;
	}
	defer free_simulation(t, actual);
	expected: ^physics.Simulation = allocate_simulation(t);
	if expected == nil
	{
		return;
	}
	defer free_simulation(t, expected);
	creation: physics.Simulation_Create_Description = small_description(pool);
	creation.allocation_sizes.bodies = 512;
	creation.allocation_sizes.constraints = 1024;
	creation.allocation_sizes.workers = i32(worker_count);
	if !testing.expect_value(t, physics.simulation_create(actual, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer
	{
		testing.expect_value(t, physics.simulation_destroy(actual), physics.Physics_Status.Ok);
		testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, nil);
		testing.expect_value(t, actual.solver.contact_coefficients.memory, nil);
	}
	if !testing.expect_value(t, physics.simulation_create(expected, &creation).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(expected);
	worlds: [2]^physics.Simulation = {actual, expected};
	for world in worlds
	{
		world.integrator.callbacks.angular_mode = .Conserve_Momentum_With_Gyroscopic_Torque;
		world.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
		cloned_record: physics.Constraint_Type_Record = world.solver.registry.records[physics.CONTACT_4_ONE_BODY_TYPE_ID];
		cloned_record.type_id = physics.FIRST_CALLER_CONSTRAINT_TYPE_ID;
		cloned_record.validate_description = contact_cache_validate_caller;
		cloned_record.registration = .Missing;
		testing.expect_value(t, physics.solver_register_constraint_type(&world.solver, cloned_record), physics.Physics_Status.Ok);
		contact_cache_populate_dispatch(t, world, 0, 9);
	}
	dt: f32 = 1.0 / 180.0;
	one_iteration: [1]i32 = {1};
	four_iterations: [1]i32 = {4};
	testing.expect_value(t, physics.solver_solve_substep(&actual.solver, dt, 1, .First), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, one_iteration[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect_value(t, actual.solver.contact_coefficients.memory, nil);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, nil);
	testing.expect_value(t, physics.solver_solve_substep(&actual.solver, dt, 4, .First), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, four_iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	small_capacity: int = int(actual.solver.contact_coefficients.length);
	testing.expect(t, small_capacity > 0);
	for world in worlds
	{
		contact_cache_populate_dispatch(t, world, 9, 257);
	}
	dispatcher: util.Thread_Dispatcher;
	if !testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, worker_count), util.Threading_Status.Ok)
	{
		return;
	}
	defer testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	schedule: Contact_Cache_Iteration_Schedule = {iterations={1, 4, 1}, observed={-1, -1, -1}};
	solve_description: physics.Solve_Description = {
		velocity_iteration_count=4, substep_count=3, fallback_batch_threshold=4,
		velocity_iteration_scheduler=contact_cache_iteration_schedule, scheduler_context=&schedule,
	};
	job: physics.Solver_Substep_Job = {
		solver=&actual.solver, solve_description=&solve_description,
		dt=dt, inverse_dt=1 / dt, substep_count=3, velocity_iteration_count=4,
	};
	testing.expect_value(t, physics.solver_dispatch_staged(&job, boundary), physics.Physics_Status.Ok);
	testing.expect_value(t, job.dispatched_worker_count, worker_count);
	testing.expect_value(t, schedule.count, 3);
	testing.expect_value(t, schedule.observed, [3]i32{0, 1, 2});
	contact_cache_reference_substeps(t, expected, dt, schedule.iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect(t, int(actual.solver.contact_coefficients.length) > small_capacity);
	coefficient_storage: [^]util.F32x8 = actual.solver.contact_coefficients.memory;
	offset_storage: [^]i32 = actual.solver.contact_coefficient_offsets.memory;
	for batch_index in 0 ..< int(actual.solver.active_set.batch_count)
	{
		batch: ^physics.Constraint_Batch = &actual.solver.active_set.batches.memory[batch_index];
		for block_index in int(batch.work_block_start) ..< int(batch.work_block_start + batch.work_block_count)
		{
			block: ^physics.Solver_Work_Block = &actual.solver.work_blocks.memory[block_index];
			type_id: i32 = batch.type_batches.memory[block.type_batch_index].type_id;
			offset: i32 = actual.solver.contact_coefficient_offsets.memory[block_index];
			if type_id == physics.FIRST_CALLER_CONSTRAINT_TYPE_ID
			{
				testing.expect_value(t, offset, i32(-1));
			}
			else
			{
				testing.expect(t, offset >= 0 && offset % 2 == 0);
			}
		}
	}
	schedule.count = 0;
	job = {
		solver=&actual.solver, solve_description=&solve_description,
		dt=dt, inverse_dt=1 / dt, substep_count=3, velocity_iteration_count=4,
	};
	testing.expect_value(t, physics.solver_dispatch_staged(&job, boundary), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, schedule.iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect_value(t, actual.solver.contact_coefficients.memory, coefficient_storage);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, offset_storage);

	// exercise normal-block reconstruction after preparing integration regions,
	// without the large fixture needed to reach the automatic-admission threshold
	layout_status, _ := physics.solver_prepare_integration_parallel_layout_and_owners(&actual.solver);
	testing.expect_value(t, layout_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_prepare_exact_integration_responsibilities(&actual.solver), physics.Physics_Status.Ok);
	region_status, region_count := physics.solver_prepare_integration_region_blocks(&actual.solver);
	testing.expect_value(t, region_status, physics.Physics_Status.Ok);
	testing.expect(t, region_count > 0);
	job = {
		solver=&actual.solver, dt=dt, inverse_dt=1 / dt, substep_count=1,
		velocity_iteration_count=4, worker_count=worker_count, dispatched_worker_count=worker_count,
		integration_work_count=region_count, integration_responsibility_stage=.Present,
	};
	testing.expect_value(t, boundary.dispatch(boundary, physics.solver_solve_worker, worker_count, &job), util.Threading_Status.Ok);
	testing.expect_value(t, physics.Physics_Status(job.status), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, four_iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);

	// synthetic metadata exceeds the pool byte limit and overflows u32 sizing
	// without allocating a large buffer or dispatching work that uses coefficients
	first_block: ^physics.Solver_Work_Block = &actual.solver.work_blocks.memory[0];
	saved_end: i32 = first_block.end_bundle;
	pool_byte_limit: i64 = i64(1) << uint(util.MAXIMUM_SPAN_SIZE_POWER);
	bundle_byte_count: i64 = size_of(physics.Contact_4_Solve_Data);
	rejected_bundle_counts: [3]i32 = {
		i32(pool_byte_limit / bundle_byte_count + 1),
		i32((i64(1) << 32) / bundle_byte_count + 1),
		max(i32),
	};
	retained_coefficients: util.Buffer(util.F32x8) = actual.solver.contact_coefficients;
	retained_offsets: util.Buffer(i32) = actual.solver.contact_coefficient_offsets;
	retained_first_offset: i32 = retained_offsets.memory[0];
	for bundle_count in rejected_bundle_counts
	{
		first_block.end_bundle = bundle_count;
		prepare_status: physics.Physics_Status = physics.solver_prepare_contact_coefficient_cache(&job);
		first_block.end_bundle = saved_end;
		testing.expect_value(t, prepare_status, physics.Physics_Status.Capacity_Missing);
		testing.expect_value(t, actual.solver.contact_coefficients, retained_coefficients);
		testing.expect_value(t, actual.solver.contact_coefficient_offsets, retained_offsets);
		testing.expect_value(t, retained_offsets.memory[0], retained_first_offset);
	}
	// the map's byte limit must reject the request before traversing the synthetic block range
	first_batch: ^physics.Constraint_Batch = &actual.solver.active_set.batches.memory[0];
	saved_work_block_count: i32 = first_batch.work_block_count;
	first_batch.work_block_count = i32(pool_byte_limit / size_of(i32) + 1);
	map_status: physics.Physics_Status = physics.solver_prepare_contact_coefficient_cache(&job);
	first_batch.work_block_count = saved_work_block_count;
	testing.expect_value(t, map_status, physics.Physics_Status.Capacity_Missing);
	testing.expect_value(t, actual.solver.contact_coefficients, retained_coefficients);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets, retained_offsets);
	testing.expect_value(t, retained_offsets.memory[0], retained_first_offset);
	// a failed solver job must stop every dispatched worker without executing later substeps
	job = {
		solver=&actual.solver, solve_description=&solve_description,
		dt=dt, inverse_dt=1 / dt, substep_count=3, velocity_iteration_count=4,
		worker_count=worker_count, dispatched_worker_count=worker_count,
		status=u32(physics.Physics_Status.Capacity_Missing),
	};
	testing.expect_value(t, boundary.dispatch(boundary, physics.solver_solve_worker, worker_count, &job), util.Threading_Status.Ok);
	testing.expect_value(t, physics.Physics_Status(job.status), physics.Physics_Status.Capacity_Missing);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect_value(t, physics.solver_solve_substep(&actual.solver, dt, 4, .First, boundary), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, four_iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	for world in worlds
	{
		testing.expect_value(t, physics.simulation_clear(world), physics.Physics_Status.Ok);
	}
	for index in 0 ..< int(actual.solver.contact_coefficient_offsets.length)
	{
		testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory[index], i32(-1));
	}
	for world in worlds
	{
		contact_cache_populate_dispatch(t, world, 0, 7);
	}
	testing.expect_value(t, physics.solver_solve_substep(&actual.solver, dt, 1, .First, boundary), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, one_iteration[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect_value(t, physics.solver_solve_substep(&actual.solver, dt, 4, .First, boundary), physics.Physics_Status.Ok);
	contact_cache_reference_substeps(t, expected, dt, four_iterations[:]);
	contact_cache_expect_dispatch(t, actual, expected);
	testing.expect_value(t, actual.solver.contact_coefficients.memory, coefficient_storage);
	testing.expect_value(t, actual.solver.contact_coefficient_offsets.memory, offset_storage);
}

@(test)
convex_contact_cached_sweeps_preserve_substep_and_description_lifetimes :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	contact_execution_lifetime_case(t, &pool, physics.Contact_4_One_Body);
	contact_execution_lifetime_case(t, &pool, physics.Contact_4);
	contact_cache_dispatch_lifetime_case(t, &pool, 4);
	contact_cache_dispatch_lifetime_case(t, &pool, 10);
}
