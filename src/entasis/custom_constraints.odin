package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// FIRST_CUSTOM_CONSTRAINT_TYPE_ID is the first type ID available to caller-defined constraints
FIRST_CUSTOM_CONSTRAINT_TYPE_ID :: Constraint_Type_ID(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID);
// MAXIMUM_CONSTRAINT_TYPE_COUNT is the fixed constraint type registry capacity
MAXIMUM_CONSTRAINT_TYPE_COUNT   :: physics.CONSTRAINT_TYPE_ID_CAPACITY;
// MAXIMUM_CUSTOM_DESCRIPTION_BYTES is the largest scalar custom constraint description
MAXIMUM_CUSTOM_DESCRIPTION_BYTES :: physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES;
// MAXIMUM_INACTIVE_IMPULSE_SCALARS limits stored impulse scalars for inactive custom constraints
MAXIMUM_INACTIVE_IMPULSE_SCALARS :: 32;

// Body_Access identifies one body field requested by a custom constraint kernel
Body_Access      :: physics.Body_Access;
// Body_Access_Mask combines body fields requested by a custom constraint kernel
Body_Access_Mask :: physics.Body_Access_Mask;

// BODY_ACCESS_ALL requests pose, inertia, and velocity data
BODY_ACCESS_ALL                         :: physics.BODY_ACCESS_ALL;
// BODY_ACCESS_NO_POSE requests inertia and velocity without pose data
BODY_ACCESS_NO_POSE                     :: physics.BODY_ACCESS_NO_POSE;
// BODY_ACCESS_NO_POSITION requests orientation, inertia, and velocity
BODY_ACCESS_NO_POSITION                 :: physics.BODY_ACCESS_NO_POSITION;
// BODY_ACCESS_NO_ORIENTATION requests position, inertia, and velocity
BODY_ACCESS_NO_ORIENTATION              :: physics.BODY_ACCESS_NO_ORIENTATION;
// BODY_ACCESS_ONLY_VELOCITY requests linear and angular velocity only
BODY_ACCESS_ONLY_VELOCITY               :: physics.BODY_ACCESS_ONLY_VELOCITY;
// BODY_ACCESS_ONLY_ANGULAR requests orientation, inertia, and angular velocity
BODY_ACCESS_ONLY_ANGULAR                :: physics.BODY_ACCESS_ONLY_ANGULAR;
// BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE requests inertia and angular velocity only
BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE   :: physics.BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
// BODY_ACCESS_ONLY_LINEAR requests position, inverse mass, and linear velocity
BODY_ACCESS_ONLY_LINEAR                 :: physics.BODY_ACCESS_ONLY_LINEAR;

// Constraint_Kernel_Phase identifies the custom kernel execution phase
Constraint_Kernel_Phase     :: physics.Constraint_Kernel_Phase;
// Constraint_Kernel_Body_Wide stores one SIMD body bundle supplied to a custom kernel
Constraint_Kernel_Body_Wide :: physics.Constraint_Kernel_Body_Wide;
// Constraint_Kernel_Proc is the SIMD solve callback for a custom constraint type
Constraint_Kernel_Proc      :: physics.Constraint_Kernel_Proc;

// F32x8 is the eight-lane f32 SIMD value used by custom constraint kernels
F32x8        :: util.F32x8;
// I32x8 is the eight-lane i32 SIMD value used by custom constraint kernels
I32x8        :: util.I32x8;
// Vector2_Wide stores eight Vector2 values in structure-of-arrays form
Vector2_Wide :: util.Vector2_Wide;
// Vector3_Wide stores eight Vector3 values in structure-of-arrays form
Vector3_Wide :: util.Vector3_Wide;
// Quaternion_Wide stores eight Quaternion values in structure-of-arrays form
Quaternion_Wide :: util.Quaternion_Wide;
// Symmetric3x3_Wide stores eight symmetric matrices in structure-of-arrays form
Symmetric3x3_Wide :: util.Symmetric3x3_Wide;

// Constraint_Description_Validate_Proc validates one scalar custom constraint description
Constraint_Description_Validate_Proc :: physics.Constraint_Description_Validate_Proc;

// Custom_Constraint_Registration describes one caller-defined constraint type and its SIMD kernels
Custom_Constraint_Registration :: struct
{
	type_id:              Constraint_Type_ID,
	body_count:           i32,
	description_size:     i32,
	prestep_bundle_size:  i32,
	impulse_bundle_size:  i32,
	initial_access:       [4]Body_Access_Mask,
	solve_access:         [4]Body_Access_Mask,
	validate_description: Constraint_Description_Validate_Proc,
	kernel:               Constraint_Kernel_Proc,
	incremental_kernel:   Constraint_Kernel_Proc,
}

// custom_constraint_registration infers the scalar description, wide prestep,
// and accumulated-impulse storage sizes. description must be a tightly packed
// sequence of f32 fields whose wide equivalent has one F32x8 per scalar field
custom_constraint_registration :: #force_inline proc "contextless" (
	$Description: typeid,
	$Prestep_Wide: typeid,
	$Accumulated_Impulses_Wide: typeid,
	type_id: Constraint_Type_ID,
	body_count: int,
	initial_access: [4]Body_Access_Mask,
	solve_access: [4]Body_Access_Mask,
	validate_description: Constraint_Description_Validate_Proc,
	kernel: Constraint_Kernel_Proc,
	incremental_kernel: Constraint_Kernel_Proc = nil,
) -> Custom_Constraint_Registration
{
	resolved_incremental_kernel := incremental_kernel;
	if resolved_incremental_kernel == nil
	{
		resolved_incremental_kernel = kernel;
	}
	return {
		type_id=type_id,
		body_count=i32(body_count),
		description_size=i32(size_of(Description)),
		prestep_bundle_size=i32(size_of(Prestep_Wide)),
		impulse_bundle_size=i32(size_of(Accumulated_Impulses_Wide)),
		initial_access=initial_access,
		solve_access=solve_access,
		validate_description=validate_description,
		kernel=kernel,
		incremental_kernel=resolved_incremental_kernel,
	};
}

// custom_constraint_next_type_id returns the first unused caller constraint type ID
custom_constraint_next_type_id :: proc "contextless" (
	world: ^World,
) -> (Constraint_Type_ID, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return CONSTRAINT_TYPE_INVALID, status;
	}
	for type_id in physics.FIRST_CALLER_CONSTRAINT_TYPE_ID ..< physics.CONSTRAINT_TYPE_ID_CAPACITY
	{
		if simulation.solver.registry.records[type_id].registration != .Present
		{
			return Constraint_Type_ID(type_id), .Ok;
		}
	}
	return CONSTRAINT_TYPE_INVALID, .Capacity_Missing;
}

// custom_constraint_register installs one caller-defined constraint type before simulation starts
custom_constraint_register :: proc "contextless" (
	world: ^World,
	registration: Custom_Constraint_Registration,
) -> Status
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return status;
	}
	if simulation.step_index != 0 || simulation.solver.active_set.constraint_count != 0
	{
		return .Invalid_Argument;
	}
	type_id := int(registration.type_id);
	if type_id < physics.FIRST_CALLER_CONSTRAINT_TYPE_ID ||
		type_id >= physics.CONSTRAINT_TYPE_ID_CAPACITY ||
		registration.body_count < 1 || registration.body_count > 4 ||
		registration.description_size <= 0 ||
		registration.description_size > physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES ||
		registration.description_size % size_of(f32) != 0 ||
		registration.prestep_bundle_size !=
		registration.description_size / size_of(f32) * size_of(util.F32x8) ||
		registration.impulse_bundle_size <= 0 ||
		registration.impulse_bundle_size % size_of(util.F32x8) != 0 ||
		registration.impulse_bundle_size / size_of(util.F32x8) > MAXIMUM_INACTIVE_IMPULSE_SCALARS ||
		registration.validate_description == nil || registration.kernel == nil ||
		registration.incremental_kernel == nil
	{
		return .Invalid_Description;
	}
	record := physics.Constraint_Type_Record{
		type_id=i32(type_id),
		body_count=registration.body_count,
		description_size=registration.description_size,
		prestep_bundle_size=registration.prestep_bundle_size,
		impulse_bundle_size=registration.impulse_bundle_size,
		initial_access=registration.initial_access,
		solve_access=registration.solve_access,
		apply_description=physics.constraint_description_transfer,
		build_description=physics.constraint_description_build,
		validate_description=registration.validate_description,
		prestep_warmstart_solve=registration.kernel,
		incrementally_update=registration.incremental_kernel,
		move_record=physics.constraint_storage_move_lane,
		remove_record=physics.constraint_storage_remove_lane,
	};
	return physics.solver_register_constraint_type(&simulation.solver, record);
}

@(private)
custom_constraint_record :: #force_inline proc "contextless" (
	simulation: ^physics.Simulation,
	type_id: Constraint_Type_ID,
) -> (^physics.Constraint_Type_Record, Status)
{
	if simulation == nil
	{
		return nil, .Invalid_Argument;
	}
	if int(type_id) < physics.FIRST_CALLER_CONSTRAINT_TYPE_ID ||
		int(type_id) >= physics.CONSTRAINT_TYPE_ID_CAPACITY
	{
		return nil, .Invalid_Argument;
	}
	return physics.constraint_type_registry_lookup(
		&simulation.solver.registry, i32(type_id),
	);
}

// custom_constraint_add creates one registered custom constraint from raw description bytes
custom_constraint_add :: proc (
	world: ^World,
	bodies: []Body_Handle,
	type_id: Constraint_Type_ID,
	description: rawptr,
	description_size: int,
) -> (Constraint_Handle, Status)
{
	data, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return constraint_handle_invalid(), status;
	}
	record, record_status := custom_constraint_record(simulation, type_id);
	if record_status != .Ok
	{
		return constraint_handle_invalid(), record_status;
	}
	if description == nil || description_size != int(record.description_size) ||
		len(bodies) != int(record.body_count)
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	handles := constraint_invalid_bodies();
	for index in 0 ..< len(bodies)
	{
		handles[index] = bodies[index];
		wake_status := physics.island_awakener_awaken_body(
			&simulation.awakener, handles[index],
		);
		if wake_status != .Ok
		{
			return constraint_handle_invalid(), wake_status;
		}
	}
	world_invalidate_views_data(data);
	return physics.solver_add_raw(
		&simulation.solver, i32(type_id), &handles, description,
	);
}

// custom_constraint_add_typed creates one registered custom constraint from a typed description
custom_constraint_add_typed :: #force_inline proc (
	world: ^World,
	bodies: []Body_Handle,
	type_id: Constraint_Type_ID,
	description: ^$T,
) -> (Constraint_Handle, Status)
{
	if description == nil
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	return custom_constraint_add(world, bodies, type_id, description, size_of(T));
}

// custom_constraint_get copies one custom constraint description into caller-owned storage
custom_constraint_get :: proc "contextless" (
	world: ^World,
	handle: Constraint_Handle,
	type_id: Constraint_Type_ID,
	target: rawptr,
	target_size: int,
) -> Status
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return status;
	}
	record, record_status := custom_constraint_record(simulation, type_id);
	if record_status != .Ok
	{
		return record_status;
	}
	if target == nil || target_size != int(record.description_size)
	{
		return .Invalid_Argument;
	}
	return physics.simulation_get_constraint_description_raw(
		simulation, handle, i32(type_id), target, target_size,
	);
}

// custom_constraint_get_typed returns one typed custom constraint description
custom_constraint_get_typed :: #force_inline proc "contextless" (
	world: ^World,
	handle: Constraint_Handle,
	type_id: Constraint_Type_ID,
	target: ^$T,
) -> Status
{
	if target == nil
	{
		return .Invalid_Argument;
	}
	return custom_constraint_get(world, handle, type_id, target, size_of(T));
}

// custom_constraint_apply replaces one custom constraint description from raw bytes
custom_constraint_apply :: proc (
	world: ^World,
	handle: Constraint_Handle,
	type_id: Constraint_Type_ID,
	description: rawptr,
	description_size: int,
) -> Status
{
	data, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return status;
	}
	record, record_status := custom_constraint_record(simulation, type_id);
	if record_status != .Ok
	{
		return record_status;
	}
	if description == nil || description_size != int(record.description_size)
	{
		return .Invalid_Argument;
	}
	world_invalidate_views_data(data);
	return physics.simulation_apply_constraint_description_raw(
		simulation, handle, i32(type_id), description,
	);
}

// custom_constraint_apply_typed replaces one custom constraint with a typed description
custom_constraint_apply_typed :: #force_inline proc (
	world: ^World,
	handle: Constraint_Handle,
	type_id: Constraint_Type_ID,
	description: ^$T,
) -> Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	return custom_constraint_apply(
		world, handle, type_id, description, size_of(T),
	);
}
