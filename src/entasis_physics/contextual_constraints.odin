// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Constraint_Callback_Dispatch :: enum u8
{
	Native,
	Contextual,
}

Contextual_Constraint_Validate_Proc :: #type proc "contextless" (
	user_context: rawptr, type_id: i32, description: rawptr,
) -> Physics_Status;
// scalar/pointer arguments only. SIMD views and the active mask are borrowed for
// this invocation. kernels obey declared body access and leave inactive lanes
// unchanged, exactly like native custom kernels. no foreign unwind is permitted
Contextual_Constraint_Kernel_Proc :: #type proc "contextless" (
	user_context, prestep: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32, impulses: rawptr, active_mask: ^util.I32x8,
	phase: Constraint_Kernel_Phase,
);
// immutable while registered. low-level users own the binding until solver
// disposal. the public facade copies it into one world-owned allocation
Contextual_Constraint_Binding :: struct
{
	user_context: rawptr,
	validate_description: Contextual_Constraint_Validate_Proc,
	kernel: Contextual_Constraint_Kernel_Proc,
	incremental_kernel: Contextual_Constraint_Kernel_Proc,
}

constraint_type_callbacks_status :: #force_inline proc "contextless" (
	record: Constraint_Type_Record,
) -> Physics_Status
{
	switch record.dispatch
	{
		case .Native:
		return .Ok if record.validate_description != nil && record.prestep_warmstart_solve != nil &&
		record.incrementally_update != nil else .Invalid_Description;
		case .Contextual:
		return .Ok if record.type_id >= FIRST_CALLER_CONSTRAINT_TYPE_ID && record.contextual != nil &&
		record.contextual.validate_description != nil && record.contextual.kernel != nil &&
		record.contextual.incremental_kernel != nil else .Invalid_Description;
	}
	return .Invalid_Description;
}

constraint_type_validate_description :: #force_inline proc "contextless" (
	record: ^Constraint_Type_Record, type_id: i32, description: rawptr,
) -> Physics_Status
{
	if record.dispatch == .Contextual
	{
		return record.contextual.validate_description(record.contextual.user_context, type_id, description);
	}
	return record.validate_description(type_id, description);
}

// used only by contextual specializations. native callback instructions remain
// in the original compile-time branch, with no binding lookup inside its loops
constraint_contextual_execute_kernel :: #force_inline proc "contextless" (
	binding: ^Contextual_Constraint_Binding, prestep: rawptr,
	bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, active_mask: ^util.I32x8,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage,
) -> Physics_Status
{
	switch stage
	{
		case .Incremental_Update:
		if phase == .First
		{
			return .Ok;
		}
		binding.incremental_kernel(binding.user_context, prestep, bodies, dt, inverse_dt,
			impulses, active_mask, .Incremental_Update);
		case .Warmstart:
		binding.kernel(binding.user_context, prestep, bodies, dt, inverse_dt, impulses, active_mask, .Prestep);
		binding.kernel(binding.user_context, prestep, bodies, dt, inverse_dt, impulses, active_mask, .Warmstart);
		case .Solve:
		binding.kernel(binding.user_context, prestep, bodies, dt, inverse_dt, impulses, active_mask, .Solve);
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities, .Integrate_Constrained_Kinematics,
		.Integrate_Constrained_Poses, .Integrate_Unconstrained_After_Substepping:
		return .Invalid_Description;
	}
	return .Ok;
}

solver_execute_contextual_work_block_integration :: #force_no_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	block: ^Solver_Work_Block, stage: Solver_Execution_Stage, worker_index: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		status: Physics_Status = solver_execute_type_bundle_dispatch_integration(job.solver, type_batch, record,
			int(block.batch_index), bundle_index, job.dt, job.inverse_dt, job.phase, stage, worker_index, .Contextual, velocity_integration);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_contextual_type_batch_integration :: #force_no_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, sequential: Reference_State,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	if sequential == .Missing
	{
		bundle_count: int = (int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
		for bundle_index in 0 ..< bundle_count
		{
			status: Physics_Status = solver_execute_type_bundle_dispatch_integration(solver, type_batch, record, batch_index,
				bundle_index, dt, inverse_dt, phase, stage, 0, .Contextual, velocity_integration);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	// preserve source-order gather/solve/scatter when fallback lanes share bodies
	for constraint_index in 0 ..< int(type_batch.count)
	{
		status: Physics_Status = solver_execute_type_lane_dispatch_integration(solver, type_batch, record, batch_index,
			constraint_index, dt, inverse_dt, phase, stage, 0, .Contextual, velocity_integration);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

// preserve the existing low-level native call signatures. only the selected
// contextual work-block/type-batch entry points instantiate contextual code
solver_execute_type_bundle_integration :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int = 0,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	return solver_execute_type_bundle_dispatch_integration(solver, type_batch, record, batch_index,
		bundle_index, dt, inverse_dt, phase, stage, worker_index, .Native, velocity_integration);
}

solver_execute_type_lane_integration :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage, worker_index: int = 0,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	return solver_execute_type_lane_dispatch_integration(solver, type_batch, record, batch_index,
		constraint_index, dt, inverse_dt, phase, stage, worker_index, .Native, velocity_integration);
}

solver_execute_contextual_work_block :: #force_no_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	block: ^Solver_Work_Block, stage: Solver_Execution_Stage, worker_index: int,
) -> Physics_Status
{
	return solver_execute_contextual_work_block_integration(job, type_batch, record, block, stage, worker_index, .Conditional);
}

solver_execute_contextual_type_batch :: #force_no_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, sequential: Reference_State,
) -> Physics_Status
{
	return solver_execute_contextual_type_batch_integration(solver, type_batch, record, batch_index, dt, inverse_dt, phase, stage, sequential, .Conditional);
}

solver_execute_type_bundle :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int = 0,
) -> Physics_Status
{
	return solver_execute_type_bundle_integration(solver, type_batch, record, batch_index, bundle_index, dt, inverse_dt, phase, stage, worker_index, .Conditional);
}

solver_execute_type_lane :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage, worker_index: int = 0,
) -> Physics_Status
{
	return solver_execute_type_lane_integration(solver, type_batch, record, batch_index, constraint_index, dt, inverse_dt, phase, stage, worker_index, .Conditional);
}
