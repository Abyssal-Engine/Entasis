package entasis_physics

import util "entasis:entasis_utilities"
import "core:sync"
import "base:runtime"

solver_prepare_contact_coefficient_cache :: proc "contextless" (
	job: ^Solver_Substep_Job,
) -> Physics_Status
{
	if job.velocity_iteration_count <= 1
	{
		return .Ok;
	}
	solver: ^Solver = job.solver;
	synchronized_batch_count: int = min(
		int(solver.active_set.batch_count), int(solver.fallback_batch_index),
	);
	work_count: int = 0;
	vector_count: i64 = 0;
	for batch_index in 0 ..< synchronized_batch_count
	{
		batch: ^Constraint_Batch = &solver.active_set.batches.memory[batch_index];
		work_count = int(batch.work_block_start) + int(batch.work_block_count);
		if work_count > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(i32)
		{
			return .Capacity_Missing;
		}
		for block_index in int(batch.work_block_start) ..< work_count
		{
			block: ^Solver_Work_Block = &solver.work_blocks.memory[block_index];
			type_batch: ^Type_Batch = &batch.type_batches.memory[block.type_batch_index];
			if type_batch.type_id < CONTACT_1_ONE_BODY_TYPE_ID ||
			type_batch.type_id > CONTACT_4_TYPE_ID
			{
				continue;
			}
			contact_count: i64 = i64(type_batch.type_id % 4) + 1;
			vectors_per_bundle: i64 = 2 * contact_count + 5;
			if contact_count == 4
			{
				vectors_per_bundle = size_of(Contact_4_Solve_Data) / size_of(util.F32x8);
			}
			vector_count = (vector_count + 1) & ~i64(1);
			vector_count += i64(block.end_bundle - block.start_bundle) * vectors_per_bundle;
			if vector_count > i64((1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(util.F32x8))
			{
				return .Capacity_Missing;
			}
		}
	}
	if vector_count == 0
	{
		return .Ok;
	}
	// normal blocks stay unchanged until all sweeps finish. discard old coefficients
	// during preparation. Warmstart completely fills each disjoint range
	context = runtime.default_context();
	status: Physics_Status = physics_ensure_buffer_capacity(
		solver.pool, &solver.contact_coefficient_offsets, work_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		solver.pool, &solver.contact_coefficients, int(vector_count), 0,
	);
	if status != .Ok
	{
		return status;
	}
	vector_offset: int = 0;
	for block_index in 0 ..< work_count
	{
		block: ^Solver_Work_Block = &solver.work_blocks.memory[block_index];
		batch: ^Constraint_Batch = &solver.active_set.batches.memory[block.batch_index];
		type_batch: ^Type_Batch = &batch.type_batches.memory[block.type_batch_index];
		if type_batch.type_id < CONTACT_1_ONE_BODY_TYPE_ID ||
		type_batch.type_id > CONTACT_4_TYPE_ID
		{
			solver.contact_coefficient_offsets.memory[block_index] = -1;
			continue;
		}
		contact_count: int = int(type_batch.type_id % 4) + 1;
		vectors_per_bundle: int = 2 * contact_count + 5;
		if contact_count == 4
		{
			vectors_per_bundle = size_of(Contact_4_Solve_Data) / size_of(util.F32x8);
		}
		vector_offset = (vector_offset + 1) & ~int(1);
		solver.contact_coefficient_offsets.memory[block_index] = i32(vector_offset);
		vector_offset += int(block.end_bundle - block.start_bundle) * vectors_per_bundle;
	}
	return .Ok;
}

solver_execute_fallback_response :: proc "contextless" (
	job: ^Solver_Substep_Job, stage: Solver_Execution_Stage,
	$response: Solver_Response_Mode,
)
{
	if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
	{
		return;
	}
	batch := &job.solver.active_set.batches.memory[job.solver.fallback_batch_index];
	for type_batch_index in 0 ..< int(batch.type_batch_count)
	{
		when response == .Restitution
		{
			solver_job_set_status(job, restitution_execute_fallback_type(job, &batch.type_batches.memory[type_batch_index], stage));
			if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
			{
				return;
			}
			continue;
		}
		status: Physics_Status = solver_execute_type_batch_integration(
			job.solver, &batch.type_batches.memory[type_batch_index],
			int(job.solver.fallback_batch_index), job.dt, job.inverse_dt,
			job.phase, stage, .Present, .Conditional,
		);
		if status != .Ok
		{
			solver_job_set_status(job, status);
			return;
		}
	}
}

solver_solve_main_response :: proc "contextless" (job: ^Solver_Substep_Job,
	$response: Solver_Response_Mode, $capture: Reference_State,
)
{
	synchronized_batch_count := min(
		int(job.solver.active_set.batch_count), int(job.solver.fallback_batch_index),
	);
	fallback_exists := int(job.solver.fallback_batch_index) < int(job.solver.active_set.batch_count);
	generation := u32(1);
	if job.integration_responsibility_stage == .Present
	{
		solver_execute_main_integration_stage(job, generation);
		generation += 1;
		if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
		{
			solver_finalize_integration_responsibilities(job.solver, job.integration_work_count);
			prepare_status, _ := solver_prepare_work_blocks(
				job.solver, job.worker_count, generation - 1,
			);
			solver_job_set_status(job, prepare_status);
		}
	}
	for substep_index in 0 ..< job.substep_count
	{
		if job.solve_description != nil
		{
			job.phase = .First;
			if substep_index > 0
			{
				job.phase = .Continuation;
			}
			job.velocity_iteration_count = solve_description_velocity_iterations(
				job.solve_description, substep_index,
			);
		}
		phase := job.phase;
		if phase == .Continuation
		{
			for batch_index in 0 ..< synchronized_batch_count
			{
				solver_execute_main_stage_response(job, batch_index, generation, .Incremental_Update, response);
				generation += 1;
			}
			if fallback_exists
			{
				solver_execute_fallback_response(job, .Incremental_Update, response);
			}
		}
		if phase == .Continuation ||
		job.solver.integrator.callbacks.integrate_kinematic_velocity == .Enabled
		{
			for batch_index in 0 ..< synchronized_batch_count
			{
				solver_execute_main_stage_response(
					job, batch_index, generation, .Integrate_Constrained_Kinematics, response,
				);
				generation += 1;
			}
			if fallback_exists
			{
				solver_execute_fallback_response(job, .Integrate_Constrained_Kinematics, response);
			}
		}
		when response == .Restitution
		{
			for batch_index in 0 ..< synchronized_batch_count
			{
				solver_execute_main_stage_response(job, batch_index, generation, .Integrate_Constrained_Dynamics, response);
				generation += 1;
			}
			if fallback_exists
			{
				solver_execute_fallback_response(job, .Integrate_Constrained_Dynamics, response);
			}
			// every integration job has joined before any impact target is captured
			for batch_index in 0 ..< synchronized_batch_count
			{
				solver_execute_main_stage_response(job, batch_index, generation, .Capture_Restitution, response);
				generation += 1;
			}
			if fallback_exists
			{
				solver_execute_fallback_response(job, .Capture_Restitution, response);
			}
		}
		if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
		{
			prepare_status: Physics_Status = solver_prepare_contact_coefficient_cache(job);
			solver_job_set_status(job, prepare_status);
		}
		for batch_index in 0 ..< synchronized_batch_count
		{
			solver_execute_main_stage_response(job, batch_index, generation, .Warmstart, response);
			generation += 1;
		}
		if fallback_exists
		{
			solver_execute_fallback_response(job, .Warmstart, response);
		}
		for _ in 0 ..< job.velocity_iteration_count
		{
			for batch_index in 0 ..< synchronized_batch_count
			{
				solver_execute_main_stage_response(job, batch_index, generation, .Solve, response);
				generation += 1;
			}
			if fallback_exists
			{
				solver_execute_fallback_response(job, .Solve, response);
			}
		}
		when capture == .Present
		{
			if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
			{
				solver_job_set_status(job, joint_break_capture_substep(job.solver.joint_breaks, job.dt));
			}
		}
	}
	if job.post_solve_integration == .Present
	{
		solver_execute_main_post_solve_stage(
			job, generation, .Integrate_Constrained_Poses,
		);
		generation += 1;
		if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
		{
			prepare_status := pose_integrator_prepare(
				job.solver.integrator, job.post_solve_callback_dt,
			);
			solver_job_set_status(job, prepare_status);
		}
		solver_execute_main_post_solve_stage(
			job, generation, .Integrate_Unconstrained_After_Substepping,
		);
		generation += 1;
	}
	// wake workers that missed every stage and terminate all background callbacks
	solver_publish_stage(job, 0, generation, 0, 0, .Solve, .Present);
}

solver_solve_substep :: proc (
	solver: ^Solver, dt: f32, velocity_iteration_count: int, phase: Solver_Substep_Phase,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || solver.integrator == nil ||
	dt <= 0 || velocity_iteration_count <= 0
	{
		return .Invalid_Argument;
	}
	job := Solver_Substep_Job{
		solver=solver,
		dt=dt,
		inverse_dt=1 / dt,
		substep_count=1,
		velocity_iteration_count=velocity_iteration_count,
		phase=phase,
		status=u32(Physics_Status.Ok),
	};
	return solver_dispatch_staged(&job, dispatcher);
}

solver_solve :: proc (
	solver: ^Solver, total_dt: f32, solve_description: ^Solve_Description,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	return solver_solve_internal_response(
		solver, total_dt, solve_description, .Missing, dispatcher, nil, .Default,
	);
}

solver_solve_and_integrate :: proc (
	solver: ^Solver, total_dt: f32, solve_description: ^Solve_Description,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	return solver_solve_internal_response(
		solver, total_dt, solve_description, .Present, dispatcher, nil, .Default,
	);
}

solver_solve_internal_response :: proc (
	solver: ^Solver, total_dt: f32, solve_description: ^Solve_Description,
	post_solve_integration: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	restitution: ^Restitution_Storage = nil,
	$response: Solver_Response_Mode,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || solver.integrator == nil ||
	solve_description == nil || total_dt <= 0 ||
	solve_description_validate(solve_description^) != .Ok
	{
		return .Invalid_Argument;
	}
	substep_dt := total_dt / f32(solve_description.substep_count);
	job := Solver_Substep_Job{
		solver=solver,
		restitution=restitution,
		solve_description=solve_description,
		dt=substep_dt,
		inverse_dt=1 / substep_dt,
		substep_count=int(solve_description.substep_count),
		velocity_iteration_count=int(solve_description.velocity_iteration_count),
		status=u32(Physics_Status.Ok),
		post_solve_integration=post_solve_integration,
	};
	if post_solve_integration == .Present
	{
		active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		job.post_solve_bundle_count =
		(active.count + util.PRODUCTION_LANE_COUNT - 1) /
		util.PRODUCTION_LANE_COUNT;
		job.post_solve_callback_dt = total_dt;
		job.post_solve_callback_substep_count = 1;
		if solver.integrator.callbacks.allow_substeps_for_unconstrained == .Enabled
		{
			job.post_solve_callback_dt = substep_dt;
			job.post_solve_callback_substep_count = int(solve_description.substep_count);
		}
	}
	return solver_dispatch_staged_response(&job, dispatcher, response, .Missing);
}

solver_execute_fallback :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, stage: Solver_Execution_Stage,
)
{
	solver_execute_fallback_response(job, stage, .Default);
}

solver_solve_main :: #force_inline proc "contextless" (job: ^Solver_Substep_Job)
{
	solver_solve_main_response(job, .Default, .Missing);
}

solver_solve_internal :: #force_inline proc (
	solver: ^Solver, total_dt: f32, solve_description: ^Solve_Description,
	post_solve_integration: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	return solver_solve_internal_response(solver, total_dt, solve_description, post_solve_integration, dispatcher, nil, .Default);
}
