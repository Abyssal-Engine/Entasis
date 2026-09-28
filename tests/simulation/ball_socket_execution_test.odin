package simulation_tests

import "core:math"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Ball_Socket_Callback_Event :: struct
{
	indices: [8]i32,
	mask: [8]i32,
	worker: int,
}
Ball_Socket_Callback_State :: struct
{
	events: [16]Ball_Socket_Callback_Event,
	count: int,
	status: physics.Physics_Status,
}
ball_socket_test_velocity :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	inertia: physics.Body_Inertia_Wide, integration_mask: util.I32x8,
	worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	state: ^Ball_Socket_Callback_State = (^Ball_Socket_Callback_State)(user_context);
	if state.count >= len(state.events)
	{
		state.status = .Capacity_Missing;
		return;
	}
	state.events[state.count] = {
		indices=transmute([8]i32)body_indices,
		mask=transmute([8]i32)integration_mask,
		worker=worker_index,
	};
	state.count += 1;
	// deliberately write masked lanes too. integration must preserve them
	velocity.linear.x += dt * util.F32x8(0.3 + f32(worker_index) * 0.01) +
		position.x * util.F32x8(0.001);
	velocity.linear.y += orientation.y * util.F32x8(0.002);
	velocity.angular.z += inertia.inverse_mass * dt * util.F32x8(0.05);
}
ball_socket_body_components :: proc(body: physics.Body_Dynamics) -> [27]f32
{
	return {
		body.motion.pose.position.x, body.motion.pose.position.y, body.motion.pose.position.z,
		body.motion.pose.orientation.x, body.motion.pose.orientation.y,
		body.motion.pose.orientation.z, body.motion.pose.orientation.w,
		body.motion.velocity.linear.x, body.motion.velocity.linear.y, body.motion.velocity.linear.z,
		body.motion.velocity.angular.x, body.motion.velocity.angular.y, body.motion.velocity.angular.z,
		body.inertia.local.inverse_mass,
		body.inertia.local.inverse_inertia_tensor.xx, body.inertia.local.inverse_inertia_tensor.yx,
		body.inertia.local.inverse_inertia_tensor.yy, body.inertia.local.inverse_inertia_tensor.zx,
		body.inertia.local.inverse_inertia_tensor.zy, body.inertia.local.inverse_inertia_tensor.zz,
		body.inertia.world.inverse_mass,
		body.inertia.world.inverse_inertia_tensor.xx, body.inertia.world.inverse_inertia_tensor.yx,
		body.inertia.world.inverse_inertia_tensor.yy, body.inertia.world.inverse_inertia_tensor.zx,
		body.inertia.world.inverse_inertia_tensor.zy, body.inertia.world.inverse_inertia_tensor.zz,
	};
}
ball_socket_expect_state :: proc(
	t: ^testing.T, actual, expected: ^physics.Simulation,
	actual_callbacks, expected_callbacks: ^Ball_Socket_Callback_State,
)
{
	actual_set: ^physics.Body_Set = &actual.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	expected_set: ^physics.Body_Set = &expected.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	testing.expect_value(t, actual_set.count, expected_set.count);
	for index in 0 ..< int(actual_set.count)
	{
		a: [27]f32 = ball_socket_body_components(actual_set.dynamics_state.memory[index]);
		b: [27]f32 = ball_socket_body_components(expected_set.dynamics_state.memory[index]);
		for component in 0 ..< len(a)
		{
			testing.expect(t, math.abs(a[component] - b[component]) <= 0.0002);
		}
	}
	testing.expect_value(t, actual_callbacks.status, physics.Physics_Status.Ok);
	testing.expect_value(t, expected_callbacks.status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual_callbacks.count, expected_callbacks.count);
	for event_index in 0 ..< actual_callbacks.count
	{
		testing.expect_value(t, actual_callbacks.events[event_index], expected_callbacks.events[event_index]);
		testing.expect_value(t, actual_callbacks.events[event_index].worker, 3);
	}
}
ball_socket_populate_case :: proc(
	t: ^testing.T, simulation: ^physics.Simulation, count: int,
	mode: physics.Solver_Integration_Mode,
) -> (physics.Solver_Work_Block, physics.Physics_Status)
{
	handles: [33]physics.Body_Handle;
	for index in 0 ..< count * 3
	{
		role: int = index % 3;
		lane: int = index / 3;
		body: physics.Body_Description = dynamic_body();
		body.pose.position = {f32(lane) * 3 + f32(role) * 0.4, f32(role) * 0.5, f32(lane) * 0.1};
		body.pose.orientation = {0.6, 0, 0, 0.8};
		if role == 1
		{
			body.pose.orientation = {0, 0.6, 0, 0.8};
		}
		body.velocity = {linear={0.2, -0.1, f32(role) * 0.05}, angular={0.1, 0.2, -0.15}};
		body.local_inertia = {inverse_mass=0.5 + f32(role) * 0.2, inverse_inertia_tensor={0.7, 0, 1.3, 0, 0, 1.9}};
		if role == 1 && lane % 4 == 3
		{
			body.local_inertia = {};
		}
		body.activity.sleep_threshold = -1;
		status: physics.Physics_Status;
		handles[index], status = physics.simulation_add_body(simulation, &body);
		if status != .Ok
		{
			return {}, status;
		}
	}
	joint: physics.Ball_Socket = {
		local_offset_a={0.3, -0.2, 0.1}, local_offset_b={-0.4, 0.1, 0.25},
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
	};
	if mode != .Always
	{
		for lane in 0 ..< count
		{
			other_index: int = lane * 3 + 1;
			if mode == .Conditional && lane % 3 != 0
			{
				other_index = lane * 3 + 2;
			}
			connected: [4]physics.Body_Handle = {handles[lane * 3], handles[other_index], {}, {}};
			status: physics.Physics_Status;
			_, status = physics.simulation_add_constraint(simulation, &connected, &joint);
			if status != .Ok
			{
				return {}, status;
			}
		}
	}
	for lane in 0 ..< count
	{
		connected: [4]physics.Body_Handle = {handles[lane * 3], handles[lane * 3 + 1], {}, {}};
		status: physics.Physics_Status;
		_, status = physics.simulation_add_constraint(simulation, &connected, &joint);
		if status != .Ok
		{
			return {}, status;
		}
	}
	status: physics.Physics_Status = physics.solver_prepare_integration_responsibilities_serial(&simulation.solver);
	if status != .Ok
	{
		return {}, status;
	}
	batch_index: int = 0;
	if mode != .Always
	{
		batch_index = 1;
	}
	batch: ^physics.Constraint_Batch = &simulation.solver.active_set.batches.memory[batch_index];
	type_index: i16 = batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID];
	type_batch: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
	testing.expect_value(t, type_batch.count, i32(count));
	if mode == .Never
	{
		testing.expect_value(t, type_batch.has_integration_responsibilities, physics.Reference_State.Missing);
	}
	else
	{
		testing.expect_value(t, type_batch.has_integration_responsibilities, physics.Reference_State.Present);
	}
	bundle_count: int = (count + 7) / 8;
	for bundle_index in 0 ..< bundle_count
	{
		impulses: ^util.Vector3_Wide = (^util.Vector3_Wide)(physics.type_batch_impulse_bundle(type_batch, bundle_index * 8));
		impulses^ = {util.F32x8(0.01), util.F32x8(-0.02), util.F32x8(0.03)};
	}
	return {
		batch_index=i16(batch_index), type_batch_index=type_index,
		start_bundle=0, end_bundle=i32(bundle_count),
	}, .Ok;
}
ball_socket_compare_execution_case :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool, count: int,
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
	description: physics.Simulation_Create_Description = small_description(pool);
	description.allocation_sizes.bodies = 64;
	description.allocation_sizes.workers = 8;
	description.solve_description.substep_count = 1;
	if !testing.expect_value(t, physics.simulation_create(actual, &description).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(actual);
	if !testing.expect_value(t, physics.simulation_create(expected, &description).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(expected);
	actual_callbacks: Ball_Socket_Callback_State;
	expected_callbacks: Ball_Socket_Callback_State;
	actual.integrator.callbacks.integrate_velocity = ball_socket_test_velocity;
	expected.integrator.callbacks.integrate_velocity = ball_socket_test_velocity;
	actual.integrator.callbacks.user_context = &actual_callbacks;
	expected.integrator.callbacks.user_context = &expected_callbacks;
	actual.integrator.callbacks.angular_mode = angular_mode;
	expected.integrator.callbacks.angular_mode = angular_mode;
	actual.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
	expected.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
	actual_block: physics.Solver_Work_Block;
	expected_block: physics.Solver_Work_Block;
	status: physics.Physics_Status;
	actual_block, status = ball_socket_populate_case(t, actual, count, mode);
	if !testing.expect_value(t, status, physics.Physics_Status.Ok)
	{
		return;
	}
	expected_block, status = ball_socket_populate_case(t, expected, count, mode);
	if !testing.expect_value(t, status, physics.Physics_Status.Ok)
	{
		return;
	}
	testing.expect_value(t, actual_block, expected_block);
	actual_batch: ^physics.Type_Batch = &actual.solver.active_set.batches.memory[actual_block.batch_index].type_batches.memory[actual_block.type_batch_index];
	expected_batch: ^physics.Type_Batch = &expected.solver.active_set.batches.memory[expected_block.batch_index].type_batches.memory[expected_block.type_batch_index];
	record: ^physics.Constraint_Type_Record = &expected.solver.registry.records[physics.BALL_SOCKET_TYPE_ID];
	job: physics.Solver_Substep_Job = {solver=&actual.solver, dt=1.0 / 60.0, inverse_dt=60, phase=phase};
	// later batches rely on world inertia and velocity already integrated by batch zero
	if mode != .Always
	{
		simulations: [2]^physics.Simulation = {actual, expected};
		for simulation in simulations
		{
			prior_batch: ^physics.Constraint_Batch = &simulation.solver.active_set.batches.memory[0];
			prior_type: ^physics.Type_Batch = &prior_batch.type_batches.memory[prior_batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID]];
			prior_stages: [2]physics.Solver_Execution_Stage = {.Integrate_Constrained_Kinematics, .Warmstart};
			for prior_stage in prior_stages
			{
				for bundle_index in 0 ..< (count + 7) / 8
				{
					status = physics.solver_execute_type_bundle(
						&simulation.solver, prior_type, record, 0, bundle_index,
						job.dt, job.inverse_dt, phase, prior_stage, 3,
					);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
				}
			}
		}
		ball_socket_expect_state(t, actual, expected, &actual_callbacks, &expected_callbacks);
		actual_callbacks = {};
		expected_callbacks = {};
	}
	stages: [7]physics.Solver_Execution_Stage = {
		.Incremental_Update, .Integrate_Constrained_Kinematics, .Warmstart,
		.Solve, .Solve, .Solve, .Solve,
	};
	ball_socket_expect_state(t, actual, expected, &actual_callbacks, &expected_callbacks);
	for stage in stages
	{
		status = physics.solver_execute_work_block(&job, &actual_block, stage, 3);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		for bundle_index in int(expected_block.start_bundle) ..< int(expected_block.end_bundle)
		{
			status = physics.solver_execute_type_bundle(
				&expected.solver, expected_batch, record, int(expected_block.batch_index), bundle_index,
				job.dt, job.inverse_dt, phase, stage, 3,
			);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			actual_impulses: ^util.Vector3_Wide = (^util.Vector3_Wide)(physics.type_batch_impulse_bundle(actual_batch, bundle_index * 8));
			expected_impulses: ^util.Vector3_Wide = (^util.Vector3_Wide)(physics.type_batch_impulse_bundle(expected_batch, bundle_index * 8));
			for lane in 0 ..< 8
			{
				a: util.Vector3 = util.vector3_wide_read_slot(actual_impulses^, lane);
				b: util.Vector3 = util.vector3_wide_read_slot(expected_impulses^, lane);
				if !testing.expectf(t, math.abs(a.x - b.x) <= 0.0002 && math.abs(a.y - b.y) <= 0.0002 && math.abs(a.z - b.z) <= 0.0002,
					"count %d mode %v phase %v angular %v stage %v bundle %d lane %d impulses %v versus %v",
					count, mode, phase, angular_mode, stage, bundle_index, lane, a, b)
				{
					return;
				}
				if bundle_index * 8 + lane >= count && (stage == .Warmstart || stage == .Solve)
				{
					testing.expect_value(t, a, util.Vector3{});
				}
			}
		}
		ball_socket_expect_state(t, actual, expected, &actual_callbacks, &expected_callbacks);
		if stage == .Incremental_Update || mode == .Never
		{
			testing.expect_value(t, actual_callbacks.count, 0);
		}
	}
	if mode != .Never
	{
		testing.expect(t, actual_callbacks.count > 0);
	}
}

@(test)
ball_socket_staged_execution_matches_registry_and_preserves_integration :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
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
					ball_socket_compare_execution_case(t, &pool, count, mode, phase, angular_mode);
				}
			}
		}
	}
}

ball_socket_compare_cached_substep :: proc(
	t: ^testing.T, actual, expected: ^physics.Simulation,
	actual_callbacks, expected_callbacks: ^Ball_Socket_Callback_State,
	dt: f32, phase: physics.Solver_Substep_Phase,
)
{
	actual_callbacks^ = {};
	expected_callbacks^ = {};
	worlds: [2]^physics.Simulation = {actual, expected};
	blocks: [2]physics.Solver_Work_Block;
	batches: [2]^physics.Type_Batch;
	for world, index in worlds
	{
		testing.expect_value(t, physics.solver_prepare_integration_responsibilities_serial(&world.solver), physics.Physics_Status.Ok);
		batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
		type_index: i16 = batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID];
		batches[index] = &batch.type_batches.memory[type_index];
		blocks[index] = {
			batch_index=0, type_batch_index=type_index,
			start_bundle=0, end_bundle=(batches[index].count + 7) / 8,
		};
	}
	testing.expect_value(t, blocks[0], blocks[1]);
	record: ^physics.Constraint_Type_Record = &expected.solver.registry.records[physics.BALL_SOCKET_TYPE_ID];
	job: physics.Solver_Substep_Job = {solver=&actual.solver, dt=dt, inverse_dt=1 / dt, phase=phase};
	stages: [7]physics.Solver_Execution_Stage = {
		.Incremental_Update, .Integrate_Constrained_Kinematics, .Warmstart,
		.Solve, .Solve, .Solve, .Solve,
	};
	for stage, stage_index in stages
	{
		testing.expect_value(t, physics.solver_execute_work_block(&job, &blocks[0], stage, 3), physics.Physics_Status.Ok);
		for bundle_index in 0 ..< int(blocks[1].end_bundle)
		{
			testing.expect_value(t, physics.solver_execute_type_bundle(
				&expected.solver, batches[1], record, 0, bundle_index, dt, job.inverse_dt, phase, stage, 3,
			), physics.Physics_Status.Ok);
			impulse_a: ^util.Vector3_Wide = (^util.Vector3_Wide)(physics.type_batch_impulse_bundle(batches[0], bundle_index * 8));
			impulse_b: ^util.Vector3_Wide = (^util.Vector3_Wide)(physics.type_batch_impulse_bundle(batches[1], bundle_index * 8));
			for lane in 0 ..< 8
			{
				a: util.Vector3 = util.vector3_wide_read_slot(impulse_a^, lane);
				b: util.Vector3 = util.vector3_wide_read_slot(impulse_b^, lane);
				testing.expectf(t, math.abs(a.x - b.x) <= 0.0002 && math.abs(a.y - b.y) <= 0.0002 && math.abs(a.z - b.z) <= 0.0002,
					"dt %g phase %v stage %d bundle %d lane %d impulses %v versus %v", dt, phase, stage_index, bundle_index, lane, a, b);
			}
		}
		ball_socket_expect_state(t, actual, expected, actual_callbacks, expected_callbacks);
	}
	for index in 0 ..< int(batches[0].count)
	{
		handle_a: physics.Constraint_Handle = {value=batches[0].index_to_handle.memory[index]};
		handle_b: physics.Constraint_Handle = {value=batches[1].index_to_handle.memory[index]};
		a, b: physics.Ball_Socket;
		testing.expect_value(t, handle_a, handle_b);
		testing.expect_value(t, physics.simulation_get_constraint_description(actual, handle_a, &a), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.simulation_get_constraint_description(expected, handle_b, &b), physics.Physics_Status.Ok);
		testing.expect_value(t, a, b);
	}
}

ball_socket_cached_lifetime_case :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, count: int)
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
	description: physics.Simulation_Create_Description = small_description(pool);
	description.allocation_sizes.bodies = 64;
	description.allocation_sizes.workers = 8;
	if !testing.expect_value(t, physics.simulation_create(actual, &description).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(actual);
	if !testing.expect_value(t, physics.simulation_create(expected, &description).status, physics.Physics_Status.Ok)
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
		world.integrator.callbacks.integrate_kinematic_velocity = .Enabled;
		status: physics.Physics_Status;
		_, status = ball_socket_populate_case(t, world, count, .Always);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	ball_socket_compare_cached_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, 1.0 / 60.0, .First);
	ball_socket_compare_cached_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, 1.0 / 90.0, .Continuation);

	// change authored data and both pose and inertia inputs before the next Warmstart
	joint: physics.Ball_Socket = {
		local_offset_a={-0.2, 0.3, 0.15}, local_offset_b={0.1, -0.4, 0.2},
		spring_settings={angular_frequency=14, twice_damping_ratio=1.5},
	};
	for world in worlds
	{
		batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
		type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID]];
		for index in 0 ..< count
		{
			handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[index]};
			testing.expect_value(t, physics.simulation_apply_constraint_description(world, handle, &joint), physics.Physics_Status.Ok);
			readback: physics.Ball_Socket;
			testing.expect_value(t, physics.simulation_get_constraint_description(world, handle, &readback), physics.Physics_Status.Ok);
			testing.expect_value(t, readback, joint);
		}
		for index in 0 ..< count
		{
			handle: physics.Body_Handle = world.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].index_to_handle.memory[index * 3];
			body: physics.Body_Description;
			status: physics.Physics_Status;
			body, status = physics.bodies_get_description(&world.bodies, handle);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			body.pose.position.y += 0.12;
			body.pose.orientation = {0, 0, 0.6, 0.8};
			body.local_inertia.inverse_inertia_tensor = {1.2, 0, 0.8, 0, 0, 1.6};
			body.local_inertia.inverse_mass = 0.9;
			testing.expect_value(t, physics.simulation_apply_body_description(world, handle, &body), physics.Physics_Status.Ok);
		}
	}
	ball_socket_compare_cached_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, 1.0 / 75.0, .First);

	// removing a first-bundle lane moves the tail lane, including its active storage.
	// eleven constraints shrink through eight to seven, covering the bundle boundary
	remaining: int = count;
	for remaining > 7
	{
		for world in worlds
		{
			batch: ^physics.Constraint_Batch = &world.solver.active_set.batches.memory[0];
			type_batch: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID]];
			moved_handle: i32 = type_batch.index_to_handle.memory[remaining - 1];
			handle: physics.Constraint_Handle = {value=type_batch.index_to_handle.memory[1]};
			testing.expect_value(t, physics.simulation_remove_constraint(world, handle), physics.Physics_Status.Ok);
			testing.expect_value(t, type_batch.count, i32(remaining - 1));
			testing.expect_value(t, type_batch.index_to_handle.memory[1], moved_handle);
		}
		remaining -= 1;
		ball_socket_compare_cached_substep(t, actual, expected, &actual_callbacks, &expected_callbacks, 1.0 / 120.0, .Continuation);
	}
}

@(test)
ball_socket_cached_sweeps_preserve_substep_and_description_lifetimes :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	// expanding storage must not change the authored-description layout requirements
	prestep: physics.Ball_Socket_Prestep;
	readback: physics.Ball_Socket;
	invalid_layouts: [3][2]int = {
		{size_of(physics.Ball_Socket) - size_of(f32), size_of(physics.Ball_Socket_Prestep)},
		{size_of(physics.Ball_Socket), size_of(physics.Ball_Socket) * 8},
		{size_of(physics.Ball_Socket), size_of(physics.Ball_Socket_Prestep) + size_of(util.F32x8)},
	};
	for layout in invalid_layouts
	{
		testing.expect_value(t, physics.constraint_description_build(
			&prestep, &readback, int(physics.BALL_SOCKET_TYPE_ID), layout[0], layout[1], 0,
		), physics.Physics_Status.Invalid_Argument);
	}
	motor: physics.Ball_Socket_Motor;
	testing.expect_value(t, physics.constraint_description_build(
		&prestep, &motor, int(physics.BALL_SOCKET_MOTOR_TYPE_ID), size_of(physics.Ball_Socket_Motor),
		size_of(physics.Ball_Socket_Prestep), 0,
	), physics.Physics_Status.Invalid_Argument);
	counts: [2]int = {8, 11};
	for count in counts
	{
		ball_socket_cached_lifetime_case(t, &pool, count);
	}
}
