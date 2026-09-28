package entasis

import "base:intrinsics"
import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Contextual_Constraint_Validate_Proc :: physics.Contextual_Constraint_Validate_Proc;
Contextual_Constraint_Kernel_Proc :: physics.Contextual_Constraint_Kernel_Proc;
Contextual_Custom_Constraint_Registration :: struct
{
	type_id: Constraint_Type_ID,
	body_count: i32,
	description_size: i32,
	prestep_bundle_size: i32,
	impulse_bundle_size: i32,
	initial_access: [4]Body_Access_Mask,
	solve_access: [4]Body_Access_Mask,
	user_context: rawptr,
	validate_description: Contextual_Constraint_Validate_Proc,
	kernel: Contextual_Constraint_Kernel_Proc,
	incremental_kernel: Contextual_Constraint_Kernel_Proc,
}

// description is tightly packed f32 data with one F32x8 per scalar in Prestep
custom_constraint_registration_contextual :: #force_inline proc "contextless" (
	$Description: typeid, $Prestep_Wide: typeid, $Accumulated_Impulses_Wide: typeid,
	type_id: Constraint_Type_ID, body_count: int,
	initial_access, solve_access: [4]Body_Access_Mask, user_context: rawptr,
	validate_description: Contextual_Constraint_Validate_Proc,
	kernel: Contextual_Constraint_Kernel_Proc,
	incremental_kernel: Contextual_Constraint_Kernel_Proc = nil,
) -> Contextual_Custom_Constraint_Registration
{
	resolved_incremental: Contextual_Constraint_Kernel_Proc = incremental_kernel;
	if resolved_incremental == nil
	{
		resolved_incremental = kernel;
	}
	return {
		type_id=type_id, body_count=i32(body_count), description_size=i32(size_of(Description)),
		prestep_bundle_size=i32(size_of(Prestep_Wide)), impulse_bundle_size=i32(size_of(Accumulated_Impulses_Wide)),
		initial_access=initial_access, solve_access=solve_access, user_context=user_context,
		validate_description=validate_description, kernel=kernel, incremental_kernel=resolved_incremental,
	};
}

@(private)
contextual_constraint_record :: struct
{
	next: ^contextual_constraint_record,
	binding: physics.Contextual_Constraint_Binding,
}

@(private)
contextual_constraint_records_release :: proc (data: ^world_data) -> Status
{
	result: Status = Status.Ok;
	record: ^contextual_constraint_record = data.contextual_constraints;
	data.contextual_constraints = nil;
	for record != nil
	{
		next: ^contextual_constraint_record = record.next;
		if util.allocation_free(record, size_of(contextual_constraint_record), align_of(contextual_constraint_record), data.allocator, data.allocation_scope) != .None
		{
			result = .Invalid_Argument;
		}
		record = next;
	}
	return result;
}

// copies callbacks once. user data remains borrowed through world destruction.
// register before any constraints or first step. existing native callbacks and
// scalar add/get/apply operations retain their signatures and storage semantics
custom_constraint_register_contextual :: proc (
	world: ^World, registration: Contextual_Custom_Constraint_Registration,
) -> Status
{
	data: ^world_data;
	simulation: ^physics.Simulation;
	status: Status;
	data, simulation, status = extension_world_ready(world);
	if status != .Ok
	{
		return status;
	}
	if simulation.step_index != 0 || simulation.solver.active_set.constraint_count != 0
	{
		return .Invalid_Argument;
	}
	type_id: int = int(registration.type_id);
	if type_id < physics.FIRST_CALLER_CONSTRAINT_TYPE_ID || type_id >= physics.CONSTRAINT_TYPE_ID_CAPACITY ||
	registration.body_count < 1 || registration.body_count > 4 || registration.description_size <= 0 ||
	registration.description_size > physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES ||
	registration.description_size % size_of(f32) != 0 ||
	registration.prestep_bundle_size != registration.description_size / size_of(f32) * size_of(util.F32x8) ||
	registration.impulse_bundle_size <= 0 || registration.impulse_bundle_size % size_of(util.F32x8) != 0 ||
	registration.impulse_bundle_size / size_of(util.F32x8) > MAXIMUM_INACTIVE_IMPULSE_SCALARS ||
	registration.validate_description == nil || registration.kernel == nil || registration.incremental_kernel == nil
	{
		return .Invalid_Description;
	}
	for body_index in 0 ..< 4
	{
		if (transmute(u8)registration.initial_access[body_index]) & ~u8(BODY_ACCESS_ALL) != 0 ||
		(transmute(u8)registration.solve_access[body_index]) & ~u8(BODY_ACCESS_ALL) != 0 ||
		(body_index >= int(registration.body_count) &&
			(registration.initial_access[body_index] != {} || registration.solve_access[body_index] != {}))
		{
			return .Invalid_Description;
		}
	}
	if simulation.solver.registry.records[type_id].registration == .Present
	{
		return .Invalid_Argument;
	}
	// snapshot before the application allocator runs. it may mutate caller-owned
	// descriptor storage, but that cannot change the registration being installed
	binding: physics.Contextual_Constraint_Binding = physics.Contextual_Constraint_Binding{
		user_context=registration.user_context, validate_description=registration.validate_description,
		kernel=registration.kernel, incremental_kernel=registration.incremental_kernel,
	};
	record: physics.Constraint_Type_Record = physics.Constraint_Type_Record{
		type_id=i32(type_id), body_count=registration.body_count, description_size=registration.description_size,
		prestep_bundle_size=registration.prestep_bundle_size, impulse_bundle_size=registration.impulse_bundle_size,
		initial_access=registration.initial_access, solve_access=registration.solve_access,
		apply_description=physics.constraint_description_transfer, build_description=physics.constraint_description_build,
		move_record=physics.constraint_storage_move_lane, remove_record=physics.constraint_storage_remove_lane,
		dispatch=.Contextual,
	};
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(contextual_constraint_record), align_of(contextual_constraint_record), data.allocator);
	if error != .None || memory == nil
	{
		return .Capacity_Missing;
	}
	owner: ^contextual_constraint_record = (^contextual_constraint_record)(memory);
	intrinsics.mem_zero(owner, size_of(contextual_constraint_record));
	owner.binding = binding;
	record.contextual = &owner.binding;
	status = physics.solver_register_constraint_type(&simulation.solver, record);
	if status != .Ok
	{
		_ = util.allocation_free(owner, size_of(contextual_constraint_record), align_of(contextual_constraint_record), data.allocator, data.allocation_scope);
		return status;
	}
	owner.next = data.contextual_constraints;
	data.contextual_constraints = owner;
	return .Ok;
}
