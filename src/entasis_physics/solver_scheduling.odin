package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:sync"
import "core:thread"

MAXIMUM_SOLVER_WORKER_COUNT :: 64;
MAXIMUM_POST_SOLVE_WORKER_COUNT :: 8;
SOLVER_WAIT_YIELD_THRESHOLD :: u32(3);
solver_worker_wait_once :: #force_inline proc "contextless" (wait_count: ^u32)
{
	if wait_count^ >= SOLVER_WAIT_YIELD_THRESHOLD
	{
		context = runtime.default_context();
		thread.yield();
		return;
	}
	pause_count := u32(1) << wait_count^;
	for _ in u32(0) ..< pause_count
	{
		sync.cpu_relax();
	}
	wait_count^ += 1;
}

Solver_Substep_Phase :: enum u8
{
	First,
	Continuation,
}

Solver_Execution_Stage :: enum u8
{
	Prepare_Integration_Responsibilities,
	Incremental_Update,
	Integrate_Constrained_Kinematics,
	Warmstart,
	Solve,
	Integrate_Constrained_Poses,
	Integrate_Unconstrained_After_Substepping,
	Integrate_Constrained_Dynamics,
	Capture_Restitution,
}

Solver_Integration_Mode :: enum u8
{
	Always,
	Conditional,
	Never,
}

Solver_Bundle_Integration_Mode :: enum u8
{
	None,
	Partial,
	All,
}

Solver_Work_Block :: struct
{
	batch_index:      i16,
	type_batch_index: i16,
	start_bundle:     i32,
	end_bundle:       i32,
	claim_generation: u32,
}
#assert(size_of(Solver_Work_Block) == 16);
solver_prepare_work_blocks :: proc "contextless" (
	solver: ^Solver, worker_count: int, claim_generation: u32 = 0,
) -> (Physics_Status, int)
{
	if solver == nil || solver.state != .Ready ||
	worker_count <= 0 || worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument, 0;
	}
	context = runtime.default_context();
	target_blocks_per_batch := worker_count * 4;
	synchronized_batch_count := min(
		int(solver.active_set.batch_count), int(solver.fallback_batch_index),
	);
	required_work_count := 0;
	for batch_index in 0 ..< synchronized_batch_count
	{
		batch := &solver.active_set.batches.memory[batch_index];
		batch_bundle_count := 0;
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			batch_bundle_count +=
			(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
			util.PRODUCTION_LANE_COUNT;
		}
		if batch_bundle_count == 0
		{
			continue;
		}
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			bundle_count :=
			(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
			util.PRODUCTION_LANE_COUNT;
			if bundle_count == 0
			{
				continue;
			}
			minimum_block_count := (bundle_count + 1023) / 1024;
			proportional_block_count :=
			target_blocks_per_batch * bundle_count / batch_bundle_count;
			type_batch_block_count := clamp(
				max(1, max(minimum_block_count, proportional_block_count)),
				1, bundle_count,
			);
			if required_work_count > max(int) - type_batch_block_count
			{
				return .Capacity_Missing, 0;
			}
			required_work_count += type_batch_block_count;
		}
	}
	if required_work_count > int(solver.work_blocks.length)
	{
		capacity_status := physics_ensure_buffer_capacity(
			solver.pool, &solver.work_blocks, required_work_count, 0,
		);
		if capacity_status != .Ok
		{
			return capacity_status, 0;
		}
	}
	work_count := 0;
	highest_work_count := 0;
	for batch_index in 0 ..< int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		batch.work_block_start = i32(work_count);
		batch.work_block_count = 0;
		if batch_index >= synchronized_batch_count
		{
			continue;
		}
		batch_bundle_count := 0;
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			batch_bundle_count +=
			(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
			util.PRODUCTION_LANE_COUNT;
		}
		if batch_bundle_count == 0
		{
			continue;
		}
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			bundle_count := (int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
			if bundle_count == 0
			{
				continue;
			}
			minimum_block_count := (bundle_count + 1023) / 1024;
			proportional_block_count :=
			target_blocks_per_batch * bundle_count / batch_bundle_count;
			type_batch_block_count := clamp(
				max(1, max(minimum_block_count, proportional_block_count)),
				1, bundle_count,
			);
			base_bundle_count := bundle_count / type_batch_block_count;
			remainder := bundle_count - base_bundle_count * type_batch_block_count;
			start_bundle := 0;
			for block_index in 0 ..< type_batch_block_count
			{
				if work_count >= int(solver.work_blocks.length)
				{
					return .Capacity_Missing, 0;
				}
				block_bundle_count := base_bundle_count;
				if block_index < remainder
				{
					block_bundle_count += 1;
				}
				solver.work_blocks.memory[work_count] = {
					batch_index=i16(batch_index),
					type_batch_index=i16(type_batch_index),
					start_bundle=i32(start_bundle),
					end_bundle=i32(start_bundle + block_bundle_count),
					claim_generation=claim_generation,
				};
				start_bundle += block_bundle_count;
				work_count += 1;
			}
		}
		batch.work_block_count = i32(work_count - int(batch.work_block_start));
		highest_work_count = max(highest_work_count, int(batch.work_block_count));
	}
	return .Ok, highest_work_count;
}

Solver_Stage_Publication :: struct #align(128)
{
	sequence:            u32,
	work_start:          i32,
	work_count:          i32,
	previous_generation: u32,
	stage:               u32,
	stop:                u32,
}

Solver_Completion_Counter :: struct #align(128)
{
	value: i32,
}
#assert(size_of(Solver_Stage_Publication) == 128);
#assert(size_of(Solver_Completion_Counter) == 128);
Solver_Substep_Job :: struct
{
	solver:                   ^Solver,
	restitution:              ^Restitution_Storage,
	solve_description:        ^Solve_Description,
	dt:                       f32,
	inverse_dt:               f32,
	substep_count:            int,
	velocity_iteration_count: int,
	worker_count:              int,
	dispatched_worker_count:   int,
	integration_work_count:    int,
	integration_responsibility_stage: Reference_State,
	phase:                     Solver_Substep_Phase,
	status:                    u32,
	post_solve_integration:    Reference_State,
	post_solve_callback_dt:    f32,
	post_solve_callback_substep_count: int,
	post_solve_bundle_count:   int,
	post_solve_worker_count:   int,
	post_solve_job_count:      int,
	publication:               Solver_Stage_Publication,
	completion:                Solver_Completion_Counter,
	post_solve_cursor:         Solver_Completion_Counter,
}

solver_job_set_status :: proc "contextless" (job: ^Solver_Substep_Job, status: Physics_Status)
{
	if status == .Ok
	{
		return;
	}
	_, _ = util.atomic_compare_exchange_u32_status(
		&job.status, u32(Physics_Status.Ok), u32(status),
	);
}

solver_publish_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, previous_generation, generation: u32,
	work_start, work_count: int, stage: Solver_Execution_Stage,
	stop: Reference_State = .Missing,
)
{
	stable_sequence := generation << 1;
	// odd means the descriptor is being rewritten. even means it is stable
	sync.atomic_store_explicit(&job.publication.sequence, stable_sequence - 1, .Seq_Cst);
	sync.atomic_store_explicit(&job.publication.work_start, i32(work_start), .Relaxed);
	sync.atomic_store_explicit(&job.publication.work_count, i32(work_count), .Relaxed);
	sync.atomic_store_explicit(
		&job.publication.previous_generation, previous_generation, .Relaxed,
	);
	sync.atomic_store_explicit(&job.publication.stage, u32(stage), .Relaxed);
	sync.atomic_store_explicit(&job.publication.stop, u32(stop), .Relaxed);
	sync.atomic_store_explicit(&job.publication.sequence, stable_sequence, .Release);
}

solver_read_published_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, latest_sequence: u32,
) -> (
	sequence, previous_generation, generation: u32,
	work_start, work_count: int,
	stage: Solver_Execution_Stage,
	stop: Reference_State,
	available: Reference_State,
)
{
	sequence_before := sync.atomic_load_explicit(&job.publication.sequence, .Acquire);
	if sequence_before == latest_sequence || (sequence_before & 1) != 0
	{
		return;
	}
	published_work_start := sync.atomic_load_explicit(&job.publication.work_start, .Relaxed);
	published_work_count := sync.atomic_load_explicit(&job.publication.work_count, .Relaxed);
	published_previous_generation := sync.atomic_load_explicit(
		&job.publication.previous_generation, .Relaxed,
	);
	published_stage := sync.atomic_load_explicit(&job.publication.stage, .Relaxed);
	published_stop := sync.atomic_load_explicit(&job.publication.stop, .Relaxed);
	sequence_after := sync.atomic_load_explicit(&job.publication.sequence, .Acquire);
	if sequence_before != sequence_after || (sequence_after & 1) != 0
	{
		return;
	}
	sequence = sequence_after;
	previous_generation = published_previous_generation;
	generation = sequence_after >> 1;
	work_start = int(published_work_start);
	work_count = int(published_work_count);
	stage = Solver_Execution_Stage(published_stage);
	stop = Reference_State(published_stop);
	available = .Present;
	return;
}

solver_execute_work_block_response :: proc "contextless" (
	job: ^Solver_Substep_Job, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$response: Solver_Response_Mode,
) -> Physics_Status
{
	solver := job.solver;
	if stage == .Prepare_Integration_Responsibilities
	{
		return solver_prepare_integration_region_block(solver, block);
	}
	batch := &solver.active_set.batches.memory[block.batch_index];
	type_batch := &batch.type_batches.memory[block.type_batch_index];
	when response == .Restitution
	{
		if stage == .Integrate_Constrained_Dynamics
		{
			return restitution_integrate_work_block(job, type_batch, block, worker_index);
		}
		if stage == .Capture_Restitution
		{
			return restitution_capture_work_block(job, type_batch, block);
		}
		if stage == .Warmstart || stage == .Solve
		{
			index: int = int((uintptr(block)-uintptr(solver.work_blocks.memory))/size_of(Solver_Work_Block));
			if job.restitution.blocks.memory[index].target_offset >= 0
			{
				return restitution_execute_work_block(job, type_batch, block, stage, worker_index, index);
			}
		}
	}
	integration_mode :: Solver_Integration_Mode.Never when response == .Restitution else Solver_Integration_Mode.Conditional;
	switch type_batch.type_id
	{
		case CONTACT_1_ONE_BODY_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 1, 1, velocity_integration=integration_mode);
		case CONTACT_2_ONE_BODY_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 1, 2, velocity_integration=integration_mode);
		case CONTACT_3_ONE_BODY_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 1, 3, velocity_integration=integration_mode);
		case CONTACT_4_ONE_BODY_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 1, 4, velocity_integration=integration_mode);
		case CONTACT_1_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 2, 1, velocity_integration=integration_mode);
		case CONTACT_2_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 2, 2, velocity_integration=integration_mode);
		case CONTACT_3_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 2, 3, velocity_integration=integration_mode);
		case CONTACT_4_TYPE_ID:
		return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, 2, 4, velocity_integration=integration_mode);
		case BALL_SOCKET_TYPE_ID:
		return solver_execute_ball_socket_work_block_integration(
			job, type_batch, block, stage, worker_index, velocity_integration=integration_mode,
		);
		case ANGULAR_HINGE_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_HINGE_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ANGULAR_SWIVEL_HINGE_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_SWIVEL_HINGE_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case SWING_LIMIT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			SWING_LIMIT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case TWIST_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			TWIST_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case TWIST_LIMIT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			TWIST_LIMIT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case TWIST_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			TWIST_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ANGULAR_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ANGULAR_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case WELD_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			WELD_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case VOLUME_CONSTRAINT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			VOLUME_CONSTRAINT_TYPE_ID,
			4, velocity_integration=integration_mode,
		);
		case DISTANCE_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			DISTANCE_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case DISTANCE_LIMIT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			DISTANCE_LIMIT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case CENTER_DISTANCE_CONSTRAINT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			CENTER_DISTANCE_CONSTRAINT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case AREA_CONSTRAINT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			AREA_CONSTRAINT_TYPE_ID,
			3, velocity_integration=integration_mode,
		);
		case POINT_ON_LINE_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			POINT_ON_LINE_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case LINEAR_AXIS_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			LINEAR_AXIS_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case LINEAR_AXIS_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			LINEAR_AXIS_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case LINEAR_AXIS_LIMIT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			LINEAR_AXIS_LIMIT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ANGULAR_AXIS_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_AXIS_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ONE_BODY_ANGULAR_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ONE_BODY_ANGULAR_SERVO_TYPE_ID,
			1, velocity_integration=integration_mode,
		);
		case ONE_BODY_ANGULAR_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ONE_BODY_ANGULAR_MOTOR_TYPE_ID,
			1, velocity_integration=integration_mode,
		);
		case ONE_BODY_LINEAR_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ONE_BODY_LINEAR_SERVO_TYPE_ID,
			1, velocity_integration=integration_mode,
		);
		case ONE_BODY_LINEAR_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ONE_BODY_LINEAR_MOTOR_TYPE_ID,
			1, velocity_integration=integration_mode,
		);
		case SWIVEL_HINGE_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			SWIVEL_HINGE_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case HINGE_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			HINGE_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case BALL_SOCKET_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			BALL_SOCKET_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case BALL_SOCKET_SERVO_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			BALL_SOCKET_SERVO_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case CENTER_DISTANCE_LIMIT_TYPE_ID:
		return solver_execute_direct_noncontact_work_block_integration(
			job,
			type_batch,
			block,
			stage,
			worker_index,
			CENTER_DISTANCE_LIMIT_TYPE_ID,
			2, velocity_integration=integration_mode,
		);
		case:
	}
	record := &solver.registry.records[type_batch.type_id];
	if record.dispatch == .Contextual
	{
		return solver_execute_contextual_work_block_integration(job, type_batch, record, block, stage, worker_index, velocity_integration=integration_mode);
	}
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		status: Physics_Status = solver_execute_type_bundle_integration(
			solver, type_batch, record, int(block.batch_index), bundle_index,
			job.dt, job.inverse_dt, job.phase, stage, worker_index, velocity_integration=integration_mode,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_try_work_block_response :: proc "contextless" (
	job: ^Solver_Substep_Job, work_index: int,
	previous_generation, generation: u32,
	stage: Solver_Execution_Stage, worker_index: int,
	$response: Solver_Response_Mode,
) -> Reference_State
{
	block := &job.solver.work_blocks.memory[work_index];
	// exact predecessor matching prevents delayed workers from claiming an older stage
	_, exchange_status := util.atomic_compare_exchange_u32_status(
		&block.claim_generation, previous_generation, generation,
	);
	if exchange_status != .Succeeded
	{
		return .Missing;
	}
	// continue claiming after an error so the orchestrator can finish the current stage safely
	if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
	{
		status: Physics_Status = solver_execute_work_block_response(job, block, stage, worker_index, response);
		solver_job_set_status(job, status);
	}
	return .Present;
}

solver_uniform_work_start :: #force_inline proc "contextless" (
	worker_index, work_count, worker_count: int,
) -> int
{
	if work_count <= worker_count
	{
		if worker_index < work_count
		{
			return worker_index;
		}
		return -1;
	}
	blocks_per_worker := work_count / worker_count;
	remainder := work_count - blocks_per_worker * worker_count;
	return blocks_per_worker * worker_index + min(remainder, worker_index);
}

solver_execute_worker_stage_response :: proc "contextless" (
	job: ^Solver_Substep_Job, work_start, work_count: int,
	previous_generation, generation: u32,
	stage: Solver_Execution_Stage, worker_index: int,
	$response: Solver_Response_Mode,
)
{
	if work_count <= 0
	{
		return;
	}
	if stage == .Integrate_Constrained_Poses ||
	stage == .Integrate_Unconstrained_After_Substepping
	{
		solver_execute_post_solve_worker_stage(job, stage, worker_index);
		return;
	}
	local_index := solver_uniform_work_start(worker_index, work_count, job.worker_count);
	if local_index < 0
	{
		return;
	}
	completed_count := 0;
	work_index := local_index;
	for solver_try_work_block_response(
		job, work_start + work_index,
		previous_generation, generation, stage, worker_index, response,
	) == .Present
	{
		completed_count += 1;
		work_index += 1;
		if work_index == work_count
		{
			work_index = 0;
		}
	}
	work_index = local_index - 1;
	if work_index < 0
	{
		work_index = work_count - 1;
	}
	for solver_try_work_block_response(
		job, work_start + work_index,
		previous_generation, generation, stage, worker_index, response,
	) == .Present
	{
		completed_count += 1;
		work_index -= 1;
		if work_index < 0
		{
			work_index = work_count - 1;
		}
	}
	if completed_count > 0
	{
		_ = sync.atomic_add_explicit(&job.completion.value, i32(completed_count), .Release);
	}
}

solver_execute_main_post_solve_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, generation: u32, stage: Solver_Execution_Stage,
)
{
	if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok ||
	job.post_solve_job_count <= 0
	{
		return;
	}
	if job.dispatched_worker_count <= 1 || job.post_solve_job_count == 1
	{
		for job_index in 0 ..< job.post_solve_job_count
		{
			if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
			{
				continue;
			}
			status := solver_execute_post_solve_job(job, job_index, stage, 0);
			solver_job_set_status(job, status);
		}
		return;
	}
	sync.atomic_store_explicit(&job.post_solve_cursor.value, i32(0), .Relaxed);
	sync.atomic_store_explicit(&job.completion.value, i32(0), .Release);
	solver_publish_stage(
		job, 0, generation, 0, job.post_solve_job_count, stage,
	);
	solver_execute_post_solve_worker_stage(job, stage, 0);
	expected_completion_count :=
	job.post_solve_job_count + job.post_solve_worker_count;
	for sync.atomic_load_explicit(&job.completion.value, .Acquire) !=
	i32(expected_completion_count)
	{
		sync.cpu_relax();
	}
}

solver_execute_main_integration_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, generation: u32,
)
{
	work_count := job.integration_work_count;
	if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok ||
	work_count <= 0
	{
		return;
	}
	previous_generation := sync.atomic_load_explicit(
		&job.solver.work_blocks.memory[0].claim_generation, .Acquire,
	);
	if job.dispatched_worker_count <= 1 || work_count == 1
	{
		for work_index in 0 ..< work_count
		{
			block := &job.solver.work_blocks.memory[work_index];
			sync.atomic_store_explicit(&block.claim_generation, generation, .Release);
			if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
			{
				continue;
			}
			status := solver_prepare_integration_region_block(job.solver, block);
			solver_job_set_status(job, status);
		}
		return;
	}
	sync.atomic_store_explicit(&job.completion.value, i32(0), .Release);
	solver_publish_stage(
		job, previous_generation, generation, 0, work_count,
		.Prepare_Integration_Responsibilities,
	);
	solver_execute_worker_stage_response(
		job, 0, work_count, previous_generation, generation,
		.Prepare_Integration_Responsibilities, 0, .Default,
	);
	for sync.atomic_load_explicit(&job.completion.value, .Acquire) != i32(work_count)
	{
		sync.cpu_relax();
	}
}

solver_execute_main_stage_response :: proc "contextless" (
	job: ^Solver_Substep_Job, batch_index: int, generation: u32,
	stage: Solver_Execution_Stage,
	$response: Solver_Response_Mode,
)
{
	if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
	{
		return;
	}
	batch := &job.solver.active_set.batches.memory[batch_index];
	work_start := int(batch.work_block_start);
	work_count := int(batch.work_block_count);
	if work_count <= 0
	{
		return;
	}
	previous_generation := sync.atomic_load_explicit(
		&job.solver.work_blocks.memory[work_start].claim_generation, .Acquire,
	);
	// avoid publication and atomic claims when there is only one executor or one block
	if job.dispatched_worker_count <= 1 || work_count == 1
	{
		for work_index in 0 ..< work_count
		{
			block := &job.solver.work_blocks.memory[work_start + work_index];
			sync.atomic_store_explicit(&block.claim_generation, generation, .Release);
			if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) != .Ok
			{
				continue;
			}
			status: Physics_Status = solver_execute_work_block_response(job, block, stage, 0, response);
			solver_job_set_status(job, status);
		}
		return;
	}
	sync.atomic_store_explicit(&job.completion.value, i32(0), .Release);
	solver_publish_stage(
		job, previous_generation, generation, work_start, work_count, stage,
	);
	solver_execute_worker_stage_response(
		job, work_start, work_count, previous_generation, generation, stage, 0, response,
	);
	// worker zero remains runnable and guarantees progress even when other workers are delayed
	for sync.atomic_load_explicit(&job.completion.value, .Acquire) != i32(work_count)
	{
		sync.cpu_relax();
	}
}

solver_solve_background_response :: proc "contextless" (
	job: ^Solver_Substep_Job, worker_index: int,
	$response: Solver_Response_Mode,
)
{
	latest_sequence := u32(0);
	wait_count := u32(0);
	for
	{
		sequence, previous_generation, generation, work_start, work_count,
		stage, stop, available := solver_read_published_stage(job, latest_sequence);
		if available == .Missing
		{
			solver_worker_wait_once(&wait_count);
			continue;
		}
		latest_sequence = sequence;
		wait_count = 0;
		if stop == .Present
		{
			return;
		}
		if (stage == .Integrate_Constrained_Poses ||
			stage == .Integrate_Unconstrained_After_Substepping) &&
		worker_index >= job.post_solve_worker_count
		{
			return;
		}
		solver_execute_worker_stage_response(
			job, work_start, work_count,
			previous_generation, generation, stage, worker_index, response,
		);
	}
}

solver_solve_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Solver_Substep_Job)(dispatcher.unmanaged_context);
	if worker_index == 0
	{
		solver_solve_main(job);
		return;
	}
	solver_solve_background(job, worker_index);
}

solver_dispatch_staged_response :: proc (
	job: ^Solver_Substep_Job,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	$response: Solver_Response_Mode, $capture: Reference_State,
) -> Physics_Status
{
	if job == nil || job.solver == nil || job.solver.state != .Ready ||
	job.solver.integrator == nil || job.dt <= 0 || job.substep_count <= 0 ||
	job.velocity_iteration_count <= 0
	{
		return .Invalid_Argument;
	}
	when capture == .Missing
	{
		owner: ^Joint_Break_System = job.solver.joint_breaks;
		if owner != nil && owner.watches.count > 0 && owner.phase == .Sampling
		{
			status: Physics_Status = joint_break_prepare_solve(owner);
			if status != .Ok
			{
				return status;
			}
			if owner.selection_count > 0
			{
				return solver_dispatch_staged_response(job, dispatcher, response, .Present);
			}
		}
	}
	job.worker_count = 1;
	job.dispatched_worker_count = 1;
	job.integration_work_count = 0;
	job.integration_responsibility_stage = .Missing;
	if dispatcher != nil
	{
		job.worker_count = min(dispatcher.worker_count, MAXIMUM_SOLVER_WORKER_COUNT);
		if job.worker_count <= 0
		{
			return .Invalid_Argument;
		}
	}
	highest_work_count := 0;
	use_parallel_integration := false;
	if dispatcher != nil && job.worker_count > 1
	{
		threshold := SOLVER_INTEGRATION_PARALLEL_BASE_THRESHOLD +
		job.worker_count * SOLVER_INTEGRATION_PARALLEL_PER_WORKER_THRESHOLD;
		parallel_constraint_count :=
		solver_integration_parallel_constraint_count(job.solver);
		if parallel_constraint_count > threshold
		{
			layout_status, _ :=
			solver_prepare_integration_parallel_layout_and_owners(job.solver);
			if layout_status == .Ok
			{
				exact_status := solver_prepare_exact_integration_responsibilities(job.solver);
				if exact_status != .Ok
				{
					return exact_status;
				}
				prepare_status, normal_highest_work_count := solver_prepare_work_blocks(
					job.solver, job.worker_count,
				);
				if prepare_status != .Ok
				{
					return prepare_status;
				}
				when response == .Restitution
				{
					restitution_status: Physics_Status = restitution_prepare_blocks(job.restitution);
					if restitution_status != .Ok
					{
						return restitution_status;
					}
				}
				region_status, region_work_count :=
				solver_prepare_integration_region_blocks(job.solver);
				if region_status == .Ok && region_work_count > 1
				{
					job.integration_work_count = region_work_count;
					job.integration_responsibility_stage = .Present;
					highest_work_count = max(
						normal_highest_work_count, region_work_count,
					);
					use_parallel_integration = true;
				}
				else if region_status != .Ok && region_status != .Capacity_Missing
				{
					return region_status;
				}
			}
			else if layout_status != .Capacity_Missing
			{
				return layout_status;
			}
		}
	}
	if !use_parallel_integration
	{
		status := solver_prepare_integration_responsibilities_serial(job.solver);
		if status != .Ok
		{
			return status;
		}
		prepare_status, prepared_highest_work_count := solver_prepare_work_blocks(
			job.solver, job.worker_count,
		);
		if prepare_status != .Ok
		{
			return prepare_status;
		}
		when response == .Restitution
		{
			restitution_status: Physics_Status = restitution_prepare_blocks(job.restitution);
			if restitution_status != .Ok
			{
				return restitution_status;
			}
		}
		highest_work_count = prepared_highest_work_count;
	}
	if job.post_solve_integration == .Present
	{
		highest_work_count = max(
			highest_work_count,
			min(job.post_solve_bundle_count, MAXIMUM_POST_SOLVE_WORKER_COUNT),
		);
	}
	if dispatcher != nil && job.worker_count > 1 && highest_work_count > 1
	{
		job.dispatched_worker_count = min(job.worker_count, highest_work_count);
	}
	job.post_solve_worker_count = min(
		job.dispatched_worker_count,
		MAXIMUM_POST_SOLVE_WORKER_COUNT,
		max(job.post_solve_bundle_count, 1),
	);
	job.post_solve_job_count = min(
		job.post_solve_bundle_count,
		job.post_solve_worker_count * 4,
	);
	if dispatcher != nil && job.dispatched_worker_count > 1
	{
		worker: util.Dispatcher_Worker_Proc = solver_solve_worker;
		when capture == .Present
		{
			worker = solver_solve_joint_break_worker(response);
		}
		else when response == .Restitution
		{
			worker = solver_solve_restitution_worker;
		}
		dispatch_status := dispatcher.dispatch(
			dispatcher, worker, job.dispatched_worker_count, job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		return Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire));
	}
	boundary := util.Thread_Dispatcher_Boundary{unmanaged_context=job};
	when capture == .Present
	{
		worker: util.Dispatcher_Worker_Proc = solver_solve_joint_break_worker(response);
		worker(0, &boundary);
	}
	else when response == .Restitution
	{
		solver_solve_restitution_worker(0, &boundary);
	}
	else
	{
		solver_solve_worker(0, &boundary);
	}
	return Physics_Status(job.status);
}

solver_execute_work_block :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
) -> Physics_Status
{
	return solver_execute_work_block_response(job, block, stage, worker_index, .Default);
}

solver_try_work_block :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, work_index: int,
	previous_generation, generation: u32,
	stage: Solver_Execution_Stage, worker_index: int,
) -> Reference_State
{
	return solver_try_work_block_response(job, work_index, previous_generation, generation, stage, worker_index, .Default);
}

solver_execute_worker_stage :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, work_start, work_count: int,
	previous_generation, generation: u32,
	stage: Solver_Execution_Stage, worker_index: int,
)
{
	solver_execute_worker_stage_response(job, work_start, work_count, previous_generation, generation, stage, worker_index, .Default);
}

solver_execute_main_stage :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, batch_index: int, generation: u32,
	stage: Solver_Execution_Stage,
)
{
	solver_execute_main_stage_response(job, batch_index, generation, stage, .Default);
}

solver_solve_background :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, worker_index: int,
)
{
	solver_solve_background_response(job, worker_index, .Default);
}

solver_dispatch_staged :: #force_inline proc (
	job: ^Solver_Substep_Job,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	return solver_dispatch_staged_response(job, dispatcher, .Default, .Missing);
}

// selected once at dispatch. background kernels and synchronization stay unchanged
solver_solve_joint_break_worker :: proc "contextless" ($response: Solver_Response_Mode) -> util.Dispatcher_Worker_Proc
{
	return proc "contextless" (worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
	{
		context = runtime.default_context();
		job: ^Solver_Substep_Job = (^Solver_Substep_Job)(dispatcher.unmanaged_context);
		if worker_index == 0
		{
			solver_solve_main_response(job, response, .Present);
		}
		else
		{
			solver_solve_background_response(job, worker_index, response);
		}
	};
}
