package entasis_c

import "base:intrinsics"
import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

#assert(int(ENTASIS_MAXIMUM_SHAPE_TYPE_COUNT) == entasis.MAXIMUM_SHAPE_TYPE_COUNT);
#assert(int(ENTASIS_BUILT_IN_SHAPE_TYPE_COUNT) == entasis.BUILT_IN_SHAPE_TYPE_COUNT);
#assert(int(ENTASIS_MAXIMUM_CUSTOM_SHAPE_ALIGNMENT) == util.BUFFER_POOL_BLOCK_ALIGNMENT);
#assert(0 == u32(entasis.Shape_Batch_Type.Convex));
#assert(1 == u32(entasis.Shape_Batch_Type.Compound));
#assert(2 == u32(entasis.Shape_Batch_Type.Homogeneous_Compound));
// allocated only when C custom callbacks are selected, never per shape instance.
// both this copied C table and the native contextual binding survive world_clear
abi_custom_shape_binding :: struct
{
	next: ^abi_custom_shape_binding,
	description: Entasis_Custom_Shape_Registration,
}

// this is a callback-stack record, not the native registry exposed as a C handle.
// each invocation has its own immutable scope. no per-world/TLS current callback
abi_shape_access_scope :: struct
{
	native: entasis.Shape_Access,
}

abi_shape_access_native :: #force_inline proc "contextless" (
	access: Entasis_Shape_Access,
) -> (entasis.Shape_Access, entasis.Status)
{
	if access.opaque == nil
	{
		return nil, .Invalid_Argument;
	}
	scope := (^abi_shape_access_scope)(access.opaque);
	if scope.native == nil
	{
		return nil, .Invalid_Argument;
	}
	return scope.native, .Ok;
}

abi_custom_callback_status :: #force_inline proc "contextless" (value: Entasis_Status) -> entasis.Status
{
	if value > abi_status(.Disposed)
	{
		return .Invalid_Argument;
	}
	return entasis.Status(value);
}

abi_shape_bounds_to_core :: #force_inline proc "contextless" (value: Entasis_Shape_Bounds) -> entasis.Shape_Bounds
{
	return {min=abi_vector3_to_core(value.min), max=abi_vector3_to_core(value.max),
		maximum_radius=value.maximum_radius, maximum_angular_expansion=value.maximum_angular_expansion};
}

abi_shape_bounds_from_core :: #force_inline proc "contextless" (value: entasis.Shape_Bounds) -> Entasis_Shape_Bounds
{
	return {min=abi_vector3_from_core(value.min), max=abi_vector3_from_core(value.max),
		maximum_radius=value.maximum_radius, maximum_angular_expansion=value.maximum_angular_expansion};
}

abi_shape_ray_from_core :: #force_inline proc "contextless" (value: entasis.Shape_Ray_Hit) -> Entasis_Shape_Ray_Hit
{
	if value.state != .Present
	{
		return {child_index=-1};
	}
	return {t=value.t, normal=abi_vector3_from_core(value.normal), child_index=value.child_index, hit=ENTASIS_TRUE};
}

abi_custom_shape_bounds_bridge :: proc "contextless" (
	user_context, shape: rawptr, orientation: ^util.Quaternion,
	access: entasis.Shape_Access, result: ^entasis.Shape_Bounds,
) -> entasis.Status
{
	binding := (^abi_custom_shape_binding)(user_context);
	scope := abi_shape_access_scope{access};
	q := Entasis_Quaternion{orientation.x, orientation.y, orientation.z, orientation.w};
	output: Entasis_Shape_Bounds;
	status := abi_custom_callback_status(binding.description.bounds(
			binding.description.user_context, shape, &q, {opaque=&scope}, &output));
	if status == .Ok
	{
		result^ = abi_shape_bounds_to_core(output);
	}
	return status;
}

abi_custom_shape_inertia_bridge :: proc "contextless" (
	user_context, shape: rawptr, mass: f32,
	access: entasis.Shape_Access, result: ^entasis.Body_Inertia,
) -> entasis.Status
{
	binding := (^abi_custom_shape_binding)(user_context);
	scope := abi_shape_access_scope{access};
	output: Entasis_Body_Inertia;
	status := abi_custom_callback_status(binding.description.inertia(
			binding.description.user_context, shape, mass, {opaque=&scope}, &output));
	if status == .Ok
	{
		result^ = abi_inertia_to_core(output);
	}
	return status;
}

abi_custom_shape_ray_bridge :: proc "contextless" (
	user_context, shape: rawptr, pose: ^entasis.Rigid_Pose, ray: ^physics.Tree_Ray,
	access: entasis.Shape_Access, result: ^entasis.Shape_Ray_Hit,
) -> entasis.Status
{
	binding := (^abi_custom_shape_binding)(user_context);
	scope := abi_shape_access_scope{access};
	p := abi_pose_from_core(pose^);
	r := Entasis_Ray{abi_vector3_from_core(ray.origin), abi_vector3_from_core(ray.direction), ray.maximum_t};
	output := Entasis_Shape_Ray_Hit{child_index=-1};
	status := abi_custom_callback_status(binding.description.ray(
			binding.description.user_context, shape, &p, &r, {opaque=&scope}, &output));
	if status != .Ok
	{
		return status;
	}
	if output.hit > ENTASIS_TRUE
	{
		return .Invalid_Description;
	}
	result^ = physics.shape_ray_miss();
	if output.hit != 0
	{
		// out-of-range/NaN hit parameters are not valid callback output
		if !(output.t >= 0 && output.t <= ray.maximum_t)
		{
			return .Invalid_Description;
		}
		result^ = {t=output.t, normal=abi_vector3_to_core(output.normal), child_index=output.child_index, state=.Present};
	}
	return .Ok;
}

abi_Support_Mode :: enum u8
{
	Support,
	Sweep,
}

abi_custom_shape_support_bridge_internal :: #force_inline proc "contextless" (
	user_context, shape: rawptr, direction: ^util.Vector3,
	access: entasis.Shape_Access, result: ^util.Vector3, $sweep: abi_Support_Mode,
) -> entasis.Status
{
	binding := (^abi_custom_shape_binding)(user_context);
	scope := abi_shape_access_scope{access};
	d := abi_vector3_from_core(direction^);
	output: Entasis_Vector3;
	callback := binding.description.support;
	when sweep == .Sweep
	{
		callback = binding.description.sweep_support;
	}
	status := abi_custom_callback_status(callback(binding.description.user_context, shape, &d, {opaque=&scope}, &output));
	if status == .Ok
	{
		result^ = abi_vector3_to_core(output);
	}
	return status;
}

abi_custom_shape_support_bridge :: proc "contextless" (
	user_context, shape: rawptr, direction: ^util.Vector3,
	access: entasis.Shape_Access, result: ^util.Vector3,
) -> entasis.Status
{
	return abi_custom_shape_support_bridge_internal(user_context, shape, direction, access, result, .Support);
}

abi_custom_shape_sweep_support_bridge :: proc "contextless" (
	user_context, shape: rawptr, direction: ^util.Vector3,
	access: entasis.Shape_Access, result: ^util.Vector3,
) -> entasis.Status
{
	return abi_custom_shape_support_bridge_internal(user_context, shape, direction, access, result, .Sweep);
}

abi_custom_shape_dispose_bridge :: proc "contextless" (
	user_context, shape: rawptr, access: entasis.Shape_Access, pool: ^util.Buffer_Pool,
) -> entasis.Status
{
	_ = pool;
	// the application disposes its own payload resources, not this pool
	binding := (^abi_custom_shape_binding)(user_context);
	if binding.description.dispose == nil
	{
		return .Ok;
	}
	scope := abi_shape_access_scope{access};
	return abi_custom_callback_status(binding.description.dispose(binding.description.user_context, shape, {opaque=&scope}));
}

abi_custom_shape_bindings_release :: proc "contextless" (resource: ^abi_world_resource)
{
	binding := resource.custom_shapes;
	resource.custom_shapes = nil;
	for binding != nil
	{
		next := binding.next;
		abi_resource_free(binding, size_of(abi_custom_shape_binding), align_of(abi_custom_shape_binding), &resource.allocator);
		binding = next;
	}
}

abi_custom_shape_registration_default :: proc "contextless" () -> Entasis_Custom_Shape_Registration
{
	return {struct_size=u32(size_of(Entasis_Custom_Shape_Registration)), struct_version=ENTASIS_STRUCT_VERSION,
		expected_type_id=Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID), alignment=1};
}

abi_custom_shape_next_type_id :: proc "contextless" (
	world: ^Entasis_World, out_type_id: ^Entasis_Shape_Type_ID, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_type_id == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	out_type_id^ = Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	id, status := entasis.custom_shape_next_type_id(&resource.world);
	if status == .Ok
	{
		out_type_id^ = Entasis_Shape_Type_ID(id);
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_custom_shape_register :: proc "contextless" (
	world: ^Entasis_World, registration: ^Entasis_Custom_Shape_Registration,
	out_type_id: ^Entasis_Shape_Type_ID, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if registration == nil || out_type_id == nil
	{
		if out_type_id != nil
		{
			out_type_id^ = Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
		}
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		out_type_id^ = Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
		return ready;
	}
	// read only the version header until its extent is known. snapshot before
	// touching an aliased output or invoking application allocation callbacks
	if registration.struct_size < u32(size_of(Entasis_Custom_Shape_Registration)) ||
	registration.struct_version != ENTASIS_STRUCT_VERSION
	{
		out_type_id^ = Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	selected := registration^;
	out_type_id^ = Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
	// neither validation nor allocation failure consumes an ID
	if selected.size == 0 || !abi_count_valid(selected.size) ||
	selected.alignment == 0 || selected.alignment > u64(ENTASIS_MAXIMUM_CUSTOM_SHAPE_ALIGNMENT) ||
	selected.alignment & (selected.alignment - 1) != 0 ||
	selected.size > u64(max(int)) - (selected.alignment - 1) ||
	selected.batch_type > 2 || selected.bounds == nil || selected.inertia == nil ||
	selected.ray == nil || selected.support == nil
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	stats, stats_status := entasis.world_stats(&resource.world);
	if stats_status != .Ok
	{
		return abi_status_finish(stats_status, diagnostic, .Shape_Register);
	}
	if stats.step_index != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	next_id, next_status := entasis.custom_shape_next_type_id(&resource.world);
	if next_status != .Ok
	{
		return abi_status_finish(next_status, diagnostic, .Shape_Register);
	}
	if selected.expected_type_id != Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID) &&
	selected.expected_type_id != Entasis_Shape_Type_ID(next_id)
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	memory, _, allocation_status := abi_resource_allocate_owned(size_of(abi_custom_shape_binding), align_of(abi_custom_shape_binding), &resource.allocator);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .Shape_Register);
	}
	binding := (^abi_custom_shape_binding)(memory);
	binding.description = selected;
	if binding.description.sweep_support == nil
	{
		binding.description.sweep_support = binding.description.support;
	}
	context = runtime.default_context();
	id, status := entasis.custom_shape_register_contextual(&resource.world, {
			expected_type_id=entasis.Shape_Type_ID(selected.expected_type_id),
			size=int(selected.size), alignment=int(selected.alignment),
			batch_type=entasis.Shape_Batch_Type(selected.batch_type), user_context=binding,
			bounds=abi_custom_shape_bounds_bridge, inertia=abi_custom_shape_inertia_bridge,
			ray=abi_custom_shape_ray_bridge, support=abi_custom_shape_support_bridge,
			sweep_support=abi_custom_shape_sweep_support_bridge, dispose=abi_custom_shape_dispose_bridge,
	});
	if status != .Ok
	{
		abi_resource_free(binding, size_of(abi_custom_shape_binding), align_of(abi_custom_shape_binding), &resource.allocator);
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	binding.next = resource.custom_shapes;
	resource.custom_shapes = binding;
	out_type_id^ = Entasis_Shape_Type_ID(id);
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_custom_shape_add :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Shape_Type_ID, shape: rawptr, size: u64,
	out_handle: ^Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_shape_handle_invalid();
	}
	if shape == nil || out_handle == nil || size == 0 || !abi_count_valid(size)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	handle, status := entasis.custom_shape_add_raw(&resource.world, entasis.Shape_Type_ID(type_id), shape, int(size));
	if status == .Ok
	{
		out_handle^ = abi_shape_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_custom_shape_get :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle, output: rawptr, capacity: u64,
	out_required: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || (capacity > 0 && output == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.custom_shape_data(&resource.world, abi_shape_handle_to_core(handle));
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	if out_required != nil
	{
		out_required^ = u64(value.size);
	}
	if capacity < u64(value.size)
	{
		return abi_status_finish(.Capacity_Missing, diagnostic, .Shape_Register);
	}
	intrinsics.mem_copy(output, value.data, value.size);
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_custom_shape_inertia :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle, mass: f32,
	out_inertia: ^Entasis_Body_Inertia, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_inertia == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	out_inertia^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	inertia, status := entasis.custom_shape_inertia(&resource.world, abi_shape_handle_to_core(handle), mass);
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_shape_access_custom_data :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle, out_view: ^Entasis_Custom_Shape_View,
) -> Entasis_Status
{
	if out_view == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_view^ = {};
	native, status := abi_shape_access_native(access);
	if status != .Ok
	{
		return abi_status(status);
	}
	value, resolve_status := entasis.shape_access_resolve(native, abi_shape_handle_to_core(handle));
	if resolve_status != .Ok
	{
		return abi_status(resolve_status);
	}
	if value.type_id < entasis.BUILT_IN_SHAPE_TYPE_COUNT
	{
		return abi_status(.Invalid_Argument);
	}
	out_view^ = {data=value.data, size=u64(value.size), alignment=u64(value.alignment), type_id=Entasis_Shape_Type_ID(value.type_id)};
	return abi_status(.Ok);
}

abi_shape_access_bounds :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle, orientation: ^Entasis_Quaternion,
	out_bounds: ^Entasis_Shape_Bounds,
) -> Entasis_Status
{
	if out_bounds == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_bounds^ = {};
	native, status := abi_shape_access_native(access);
	if status != .Ok || orientation == nil
	{
		return abi_status(.Invalid_Argument);
	}
	q := util.Quaternion{orientation.x, orientation.y, orientation.z, orientation.w};
	value, compute_status := entasis.shape_access_compute_bounds(native, abi_shape_handle_to_core(handle), q);
	if compute_status == .Ok
	{
		out_bounds^ = abi_shape_bounds_from_core(value);
	}
	return abi_status(compute_status);
}

abi_shape_access_inertia :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle, mass: f32,
	out_inertia: ^Entasis_Body_Inertia,
) -> Entasis_Status
{
	if out_inertia == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_inertia^ = {};
	native, status := abi_shape_access_native(access);
	if status != .Ok
	{
		return abi_status(status);
	}
	value, compute_status := entasis.shape_access_compute_inertia(native, abi_shape_handle_to_core(handle), mass);
	if compute_status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(value);
	}
	return abi_status(compute_status);
}

abi_shape_access_ray :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle,
	pose: ^Entasis_Rigid_Pose, ray: ^Entasis_Ray, out_hit: ^Entasis_Shape_Ray_Hit,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_hit^ = {child_index=-1};
	native, status := abi_shape_access_native(access);
	if status != .Ok || pose == nil || ray == nil
	{
		return abi_status(.Invalid_Argument);
	}
	r := physics.Tree_Ray{abi_vector3_to_core(ray.origin), abi_vector3_to_core(ray.direction), ray.maximum_t};
	value, ray_status := entasis.shape_access_ray_test(native, abi_shape_handle_to_core(handle), abi_pose_to_core(pose^), r);
	if ray_status == .Ok
	{
		out_hit^ = abi_shape_ray_from_core(value);
	}
	return abi_status(ray_status);
}

abi_shape_access_support_internal :: #force_inline proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle,
	direction: ^Entasis_Vector3, out_support: ^Entasis_Vector3, $sweep: abi_Support_Mode,
) -> Entasis_Status
{
	if out_support == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_support^ = {};
	native, status := abi_shape_access_native(access);
	if status != .Ok || direction == nil
	{
		return abi_status(.Invalid_Argument);
	}
	value: util.Vector3;
	when sweep == .Sweep
	{
		value, status = entasis.shape_access_sweep_support(native, abi_shape_handle_to_core(handle), abi_vector3_to_core(direction^));
	}
	else
	{
		value, status = entasis.shape_access_support(native, abi_shape_handle_to_core(handle), abi_vector3_to_core(direction^));
	}
	if status == .Ok
	{
		out_support^ = abi_vector3_from_core(value);
	}
	return abi_status(status);
}

abi_shape_access_support :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle,
	direction: ^Entasis_Vector3, out_support: ^Entasis_Vector3,
) -> Entasis_Status
{
	return abi_shape_access_support_internal(access, handle, direction, out_support, .Support);
}

abi_shape_access_sweep_support :: proc "contextless" (
	access: Entasis_Shape_Access, handle: Entasis_Shape_Handle,
	direction: ^Entasis_Vector3, out_support: ^Entasis_Vector3,
) -> Entasis_Status
{
	return abi_shape_access_support_internal(access, handle, direction, out_support, .Sweep);
}
