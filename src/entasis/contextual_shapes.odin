package entasis

import "base:intrinsics"
import "core:mem"
import util "entasis:entasis_utilities"
import physics "entasis:entasis_physics"

// Shape_Access is a borrowed callback-local, read-only child access scope.
// it exposes no simulation storage, collector, scratch pool or mutation API
Shape_Access :: physics.Shape_Access;
Shape_Access_Value :: physics.Shape_Access_Value;
shape_access_resolve :: physics.shape_access_resolve;
shape_access_compute_bounds :: physics.shape_access_compute_bounds;
shape_access_compute_inertia :: physics.shape_access_compute_inertia;
shape_access_ray_test :: physics.shape_access_ray_test;
shape_access_support :: physics.shape_access_support;
shape_access_sweep_support :: physics.shape_access_sweep_support;
Contextual_Shape_Bounds_Proc :: physics.Contextual_Shape_Bounds_Proc;
Contextual_Shape_Inertia_Proc :: physics.Contextual_Shape_Inertia_Proc;
Contextual_Shape_Ray_Proc :: physics.Contextual_Shape_Ray_Proc;
Contextual_Shape_Support_Proc :: physics.Contextual_Shape_Support_Proc;
Contextual_Shape_Dispose_Proc :: physics.Contextual_Shape_Dispose_Proc;
Contextual_Custom_Shape_Registration :: struct
{
	expected_type_id: Shape_Type_ID,
	size, alignment: int,
	batch_type: Shape_Batch_Type,
	user_context: rawptr,
	bounds: Contextual_Shape_Bounds_Proc,
	inertia: Contextual_Shape_Inertia_Proc,
	ray: Contextual_Shape_Ray_Proc,
	support: Contextual_Shape_Support_Proc,
	sweep_support: Contextual_Shape_Support_Proc,
	dispose: Contextual_Shape_Dispose_Proc,
}

custom_shape_registration_contextual :: #force_inline proc "contextless" (
	$T: typeid, batch_type: Shape_Batch_Type, user_context: rawptr,
	bounds: Contextual_Shape_Bounds_Proc, inertia: Contextual_Shape_Inertia_Proc,
	ray: Contextual_Shape_Ray_Proc, support: Contextual_Shape_Support_Proc,
	sweep_support: Contextual_Shape_Support_Proc = nil,
	dispose: Contextual_Shape_Dispose_Proc = physics.shape_contextual_no_dispose,
	expected_type_id: Shape_Type_ID = SHAPE_TYPE_INVALID,
) -> Contextual_Custom_Shape_Registration
{
	resolved_sweep: Contextual_Shape_Support_Proc = sweep_support;
	if resolved_sweep == nil
	{
		resolved_sweep = support;
	}
	return {
		expected_type_id=expected_type_id, size=size_of(T), alignment=align_of(T),
		batch_type=batch_type, user_context=user_context, bounds=bounds,
		inertia=inertia, ray=ray, support=support, sweep_support=resolved_sweep, dispose=dispose,
	};
}

@(private)
contextual_shape_record :: struct
{
	next: ^contextual_shape_record,
	binding: physics.Contextual_Shape_Binding,
}

@(private)
contextual_shape_records_release :: proc (data: ^world_data) -> Status
{
	result: Status = Status.Ok;
	record: ^contextual_shape_record = data.contextual_shapes;
	data.contextual_shapes = nil;
	for record != nil
	{
		next: ^contextual_shape_record = record.next;
		if util.allocation_free(record, size_of(contextual_shape_record), align_of(contextual_shape_record), data.allocator, data.allocation_scope) != .None
		{
			result = .Invalid_Argument;
		}
		record = next;
	}
	return result;
}

// copies the table into stable world-owned storage. user_context stays borrowed
// through destruction. register before the first step. payloads are unmodified,
// with supported alignment up to the pool's 128-byte guarantee. native shapes
// retain their existing registration structures and callback signatures
custom_shape_register_contextual :: proc (
	world: ^World, registration: Contextual_Custom_Shape_Registration,
) -> (Shape_Type_ID, Status)
{
	data: ^world_data;
	simulation: ^physics.Simulation;
	status: Status;
	data, simulation, status = extension_world_ready(world);
	if status != .Ok
	{
		return SHAPE_TYPE_INVALID, status;
	}
	if simulation.step_index != 0
	{
		return SHAPE_TYPE_INVALID, .Invalid_Argument;
	}
	registry: ^physics.Shape_Registry = physics.simulation_shape_registry(simulation);
	if registry == nil
	{
		return SHAPE_TYPE_INVALID, .Disposed;
	}
	if registration.expected_type_id != SHAPE_TYPE_INVALID &&
	registration.expected_type_id != Shape_Type_ID(registry.registered_type_count)
	{
		return SHAPE_TYPE_INVALID, .Invalid_Description;
	}
	binding: physics.Contextual_Shape_Binding = physics.Contextual_Shape_Binding{
		user_context=registration.user_context, bounds=registration.bounds,
		inertia=registration.inertia, ray=registration.ray, support=registration.support,
		sweep_support=registration.sweep_support, dispose=registration.dispose,
	};
	metadata: physics.Contextual_Shape_Type_Metadata = physics.Contextual_Shape_Type_Metadata{
		size=registration.size, alignment=registration.alignment,
		batch_type=registration.batch_type, binding=&binding,
	};
	status = physics.shape_registry_validate_contextual(registry, metadata);
	if status != .Ok
	{
		return SHAPE_TYPE_INVALID, status;
	}
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(contextual_shape_record), align_of(contextual_shape_record), data.allocator);
	if error != .None || memory == nil
	{
		return SHAPE_TYPE_INVALID, .Capacity_Missing;
	}
	record: ^contextual_shape_record = (^contextual_shape_record)(memory);
	intrinsics.mem_zero(record, size_of(contextual_shape_record));
	record.binding = binding;
	metadata.binding = &record.binding;
	type_id: i32;
	register_status: physics.Physics_Status;
	type_id, register_status = physics.shape_registry_register_contextual(registry, metadata);
	if register_status != .Ok
	{
		_ = util.allocation_free(record, size_of(contextual_shape_record), align_of(contextual_shape_record), data.allocator, data.allocation_scope);
		return SHAPE_TYPE_INVALID, register_status;
	}
	record.next = data.contextual_shapes;
	data.contextual_shapes = record;
	return Shape_Type_ID(type_id), .Ok;
}
