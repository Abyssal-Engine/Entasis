package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

#assert(int(ENTASIS_FIRST_CUSTOM_CONSTRAINT_TYPE_ID) == int(entasis.FIRST_CUSTOM_CONSTRAINT_TYPE_ID));
#assert(int(ENTASIS_MAXIMUM_CONSTRAINT_TYPE_COUNT) == entasis.MAXIMUM_CONSTRAINT_TYPE_COUNT);
#assert(int(ENTASIS_MAXIMUM_CUSTOM_DESCRIPTION_BYTES) == entasis.MAXIMUM_CUSTOM_DESCRIPTION_BYTES);
#assert(u8(entasis.BODY_ACCESS_ALL) == u8(u32(63)));
#assert(u32(entasis.Constraint_Kernel_Phase.Prestep) == u32(0));
#assert(u32(entasis.Constraint_Kernel_Phase.Warmstart) == u32(1));
#assert(u32(entasis.Constraint_Kernel_Phase.Solve) == u32(2));
#assert(u32(entasis.Constraint_Kernel_Phase.Incremental_Update) == u32(3));
// at most eight immutable C registrations per world. one boundary lookup per
// public call or batch. the solver invokes its explicit binding directly
abi_custom_constraint_binding :: struct
{
	next: ^abi_custom_constraint_binding,
	description: Entasis_Custom_Constraint_Registration,
	reaction_provider: Entasis_Joint_Reaction_Provider,
}

abi_custom_constraint_find :: proc "contextless" (
	resource: ^abi_world_resource, type_id: Entasis_Constraint_Type_ID,
) -> ^abi_custom_constraint_binding
{
	for binding := resource.custom_constraints; binding != nil; binding = binding.next
	{
		if binding.description.type_id == type_id
		{
			return binding;
		}
	}
	return nil;
}

abi_custom_constraint_bindings_release :: proc "contextless" (resource: ^abi_world_resource)
{
	binding := resource.custom_constraints;
	resource.custom_constraints = nil;
	for binding != nil
	{
		next := binding.next;
		abi_resource_free(binding, size_of(abi_custom_constraint_binding), align_of(abi_custom_constraint_binding), &resource.allocator);
		binding = next;
	}
}

abi_custom_constraint_validate_bridge :: proc "contextless" (
	user_context: rawptr, type_id: i32, description: rawptr,
) -> entasis.Status
{
	binding := (^abi_custom_constraint_binding)(user_context);
	return abi_custom_callback_status(binding.description.validate(binding.description.user_context,
			Entasis_Constraint_Type_ID(type_id), description, binding.description.description_size));
}

abi_custom_constraint_kernel_bridge :: proc "contextless" (
	user_context, prestep: rawptr, bodies: ^[4]entasis.Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32, impulses: rawptr, active_mask: ^util.I32x8,
	phase: entasis.Constraint_Kernel_Phase,
)
{
	binding := (^abi_custom_constraint_binding)(user_context);
	views: [4]Entasis_Constraint_Body_View;
	for body_index in 0 ..< int(binding.description.body_count)
	{
		mask := binding.description.initial_access[body_index];
		if phase == .Solve
		{
			mask = binding.description.solve_access[body_index];
		}
		body := &bodies[body_index];
		view := &views[body_index];
		// each field's layout, size and 32-byte alignment is asserted by the
		// shared callback owner. never reinterpret the whole native body struct
		if mask & u32(1) != 0
		{
			view.position = (^Entasis_Vector3_Wide)(&body.position);
		}
		if mask & u32(2) != 0
		{
			view.orientation = (^Entasis_Quaternion_Wide)(&body.orientation);
		}
		if mask & u32(4) != 0
		{
			view.inverse_mass = (^Entasis_F32x8)(&body.inverse_mass);
		}
		if mask & u32(8) != 0
		{
			view.inverse_inertia = (^Entasis_Symmetric3x3_Wide)(&body.inverse_inertia);
		}
		if mask & u32(16) != 0
		{
			view.linear_velocity = (^Entasis_Vector3_Wide)(&body.linear_velocity);
		}
		if mask & u32(32) != 0
		{
			view.angular_velocity = (^Entasis_Vector3_Wide)(&body.angular_velocity);
		}
	}
	view := Entasis_Constraint_Kernel_View{
		bodies=raw_data(views[:]), body_count=binding.description.body_count,
		phase=Entasis_Constraint_Kernel_Phase(phase), prestep=([^]Entasis_F32x8)(prestep),
		prestep_field_count=binding.description.prestep_bundle_size / u32(size_of(util.F32x8)),
		impulse_field_count=binding.description.impulse_bundle_size / u32(size_of(util.F32x8)),
		impulses=([^]Entasis_F32x8)(impulses), active_mask=(^Entasis_I32x8)(active_mask),
		dt=dt, inverse_dt=inverse_dt,
	};
	callback := binding.description.kernel;
	if phase == .Incremental_Update
	{
		callback = binding.description.incremental_kernel;
	}
	callback(binding.description.user_context, &view);
}

abi_custom_constraint_registration_default :: proc "contextless" () -> Entasis_Custom_Constraint_Registration
{
	return {struct_size=u32(size_of(Entasis_Custom_Constraint_Registration)), struct_version=ENTASIS_STRUCT_VERSION,
		type_id=Entasis_Constraint_Type_ID(entasis.CONSTRAINT_TYPE_INVALID)};
}

abi_custom_constraint_next_type_id :: proc "contextless" (
	world: ^Entasis_World, out_type_id: ^Entasis_Constraint_Type_ID, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_type_id == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_type_id^ = Entasis_Constraint_Type_ID(entasis.CONSTRAINT_TYPE_INVALID);
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	type_id, status := entasis.custom_constraint_next_type_id(&resource.world);
	out_type_id^ = Entasis_Constraint_Type_ID(type_id);
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_custom_constraint_register :: proc "contextless" (
	world: ^Entasis_World, registration: ^Entasis_Custom_Constraint_Registration, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if registration == nil || registration.struct_size != u32(size_of(Entasis_Custom_Constraint_Registration)) ||
	registration.struct_version != ENTASIS_STRUCT_VERSION
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	// caller-owned descriptor memory may change during application allocation
	selected := registration^;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	if selected.type_id < Entasis_Constraint_Type_ID(ENTASIS_FIRST_CUSTOM_CONSTRAINT_TYPE_ID) ||
	selected.type_id >= Entasis_Constraint_Type_ID(ENTASIS_MAXIMUM_CONSTRAINT_TYPE_COUNT) ||
	selected.body_count < 1 || selected.body_count > 4 || selected.description_size == 0 ||
	selected.description_size > ENTASIS_MAXIMUM_CUSTOM_DESCRIPTION_BYTES || selected.description_size % 4 != 0 ||
	selected.prestep_bundle_size != selected.description_size * 8 || selected.impulse_bundle_size == 0 ||
	selected.impulse_bundle_size > ENTASIS_MAXIMUM_INACTIVE_IMPULSE_SCALARS * 32 || selected.impulse_bundle_size % 32 != 0 ||
	selected._reserved0 != 0 || selected.validate == nil || selected.kernel == nil
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Constraint_Register);
	}
	initial, solve: [4]entasis.Body_Access_Mask;
	for index in 0 ..< 4
	{
		if selected.initial_access[index] & ~u32(63) != 0 || selected.solve_access[index] & ~u32(63) != 0 ||
		(index >= int(selected.body_count) && (selected.initial_access[index] != 0 || selected.solve_access[index] != 0))
		{
			return abi_status_finish(.Invalid_Description, diagnostic, .Constraint_Register);
		}
		initial[index] = transmute(entasis.Body_Access_Mask)u8(selected.initial_access[index]);
		solve[index] = transmute(entasis.Body_Access_Mask)u8(selected.solve_access[index]);
	}
	stats, status := entasis.world_stats(&resource.world);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Constraint_Register);
	}
	if stats.step_index != 0 || stats.active_constraints != 0 || stats.sleeping_constraints != 0 ||
	abi_custom_constraint_find(resource, selected.type_id) != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	if selected.incremental_kernel == nil
	{
		selected.incremental_kernel = selected.kernel;
	}
	memory, _, allocation_status := abi_resource_allocate_owned(size_of(abi_custom_constraint_binding), align_of(abi_custom_constraint_binding), &resource.allocator);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .Constraint_Register);
	}
	binding := (^abi_custom_constraint_binding)(memory);
	binding^ = {description=selected};
	context = runtime.default_context();
	status = entasis.custom_constraint_register_contextual(&resource.world, {
			type_id=entasis.Constraint_Type_ID(selected.type_id), body_count=i32(selected.body_count),
			description_size=i32(selected.description_size), prestep_bundle_size=i32(selected.prestep_bundle_size),
			impulse_bundle_size=i32(selected.impulse_bundle_size), initial_access=initial, solve_access=solve,
			user_context=binding, validate_description=abi_custom_constraint_validate_bridge,
			kernel=abi_custom_constraint_kernel_bridge, incremental_kernel=abi_custom_constraint_kernel_bridge,
	});
	if status != .Ok
	{
		abi_resource_free(binding, size_of(abi_custom_constraint_binding), align_of(abi_custom_constraint_binding), &resource.allocator);
		return abi_status_finish(status, diagnostic, .Constraint_Register);
	}
	binding.next = resource.custom_constraints;
	resource.custom_constraints = binding;
	return abi_status_finish(.Ok, diagnostic, .Constraint_Register);
}

abi_custom_constraint_add :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID, bodies: [^]Entasis_Body_Handle, body_count: u32,
	description: rawptr, description_size: u32, out_handle: ^Entasis_Constraint_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = {value=-1};
	}
	if out_handle == nil || !abi_constraint_pointer_aligned(bodies, align_of(Entasis_Body_Handle)) ||
	!abi_constraint_pointer_aligned(description, align_of(f32))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	binding := abi_custom_constraint_find(resource, type_id);
	if binding == nil
	{
		return abi_status_finish(.Not_Found, diagnostic, .Constraint_Register);
	}
	if body_count != binding.description.body_count || description_size != binding.description.description_size
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	context = runtime.default_context();
	handle, status := entasis.custom_constraint_add(&resource.world, (([^]entasis.Body_Handle)(bodies))[:int(body_count)],
		entasis.Constraint_Type_ID(type_id), description, int(description_size));
	if status == .Ok
	{
		out_handle^ = abi_constraint_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_Constraint_Transfer :: enum u8
{
	Read,
	Apply,
}

abi_custom_constraint_transfer :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Constraint_Handle, type_id: Entasis_Constraint_Type_ID,
	description: rawptr, description_size: u32, diagnostic: ^Entasis_Diagnostic, $apply: abi_Constraint_Transfer,
) -> Entasis_Status
{
	if !abi_constraint_pointer_aligned(description, align_of(f32))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=(.Read_Only if apply == .Read else .Exclusive));
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	binding := abi_custom_constraint_find(resource, type_id);
	if binding == nil
	{
		return abi_status_finish(.Not_Found, diagnostic, .Constraint_Register);
	}
	if description_size != binding.description.description_size
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	context = runtime.default_context();
	status: entasis.Status;
	when apply == .Apply
	{
		status = entasis.custom_constraint_apply(&resource.world, abi_constraint_handle_to_core(handle),
			entasis.Constraint_Type_ID(type_id), description, int(description_size));
	}
	else
	{
		status = entasis.custom_constraint_get(&resource.world, abi_constraint_handle_to_core(handle),
			entasis.Constraint_Type_ID(type_id), description, int(description_size));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_custom_constraint_get :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Constraint_Handle, type_id: Entasis_Constraint_Type_ID,
	description: rawptr, description_size: u32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_custom_constraint_transfer(world, handle, type_id, description, description_size, diagnostic, .Read);
}

abi_custom_constraint_apply :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Constraint_Handle, type_id: Entasis_Constraint_Type_ID,
	description: rawptr, description_size: u32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_custom_constraint_transfer(world, handle, type_id, description, description_size, diagnostic, .Apply);
}

// check byte spans before pointer arithmetic, without dereferencing caller data.
// nonoverlap between input/output arrays is the same caller contract as the
// existing batch operations. empty batches do not dereference any buffer
abi_custom_constraint_span_status :: proc "contextless" (
	pointer: rawptr, count, stride, element_size: u64, alignment: int,
) -> entasis.Status
{
	if count == 0
	{
		return .Ok;
	}
	if !abi_count_valid(count) || !abi_count_valid(stride) || stride < element_size ||
	stride % u64(alignment) != 0 || !abi_constraint_pointer_aligned(pointer, alignment) ||
	count - 1 > (u64(max(int)) - element_size) / stride
	{
		return .Invalid_Argument;
	}
	span := (count - 1) * stride + element_size;
	return .Ok if uintptr(pointer) <= max(uintptr) - uintptr(span) else .Invalid_Argument;
}

abi_custom_constraint_add_batch :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID, bodies: [^]Entasis_Body_Handle, body_handle_count: u64,
	descriptions: rawptr, description_stride, count: u64, out_handles: [^]Entasis_Constraint_Handle,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if out_completed == nil || !abi_count_valid(count)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	binding := abi_custom_constraint_find(resource, type_id);
	if binding == nil
	{
		return abi_status_finish(.Not_Found, diagnostic, .Constraint_Register);
	}
	arity := u64(binding.description.body_count);
	if count > u64(max(int)) / arity || body_handle_count != count * arity ||
	(abi_custom_constraint_span_status(bodies, body_handle_count, size_of(Entasis_Body_Handle), size_of(Entasis_Body_Handle), align_of(Entasis_Body_Handle)) != .Ok) ||
	(abi_custom_constraint_span_status(out_handles, count, size_of(Entasis_Constraint_Handle), size_of(Entasis_Constraint_Handle), align_of(Entasis_Constraint_Handle)) != .Ok) ||
	(abi_custom_constraint_span_status(descriptions, count, description_stride, u64(binding.description.description_size), align_of(f32)) != .Ok)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	context = runtime.default_context();
	for index in 0 ..< count
	{
		out_handles[index] = {value=-1};
		input := rawptr(uintptr(descriptions) + uintptr(index * description_stride));
		body_slice := (([^]entasis.Body_Handle)(bodies))[int(index * arity):int((index + 1) * arity)];
		handle, status := entasis.custom_constraint_add(&resource.world, body_slice, entasis.Constraint_Type_ID(type_id),
			input, int(binding.description.description_size));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .Constraint_Register, i32(min(index, u64(max(i32)))));
		}
		out_handles[index] = abi_constraint_handle_from_core(handle);
		out_completed^ = index + 1;
	}
	return abi_status_finish(.Ok, diagnostic, .Constraint_Register, i32(min(count, u64(max(i32)))));
}

abi_custom_constraint_transfer_batch :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID, handles: [^]Entasis_Constraint_Handle, count: u64,
	descriptions: rawptr, description_stride: u64, out_completed: ^u64, diagnostic: ^Entasis_Diagnostic, $apply: abi_Constraint_Transfer,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if out_completed == nil || !abi_count_valid(count)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=(.Read_Only if apply == .Read else .Exclusive));
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	binding := abi_custom_constraint_find(resource, type_id);
	if binding == nil
	{
		return abi_status_finish(.Not_Found, diagnostic, .Constraint_Register);
	}
	if (abi_custom_constraint_span_status(handles, count, size_of(Entasis_Constraint_Handle), size_of(Entasis_Constraint_Handle), align_of(Entasis_Constraint_Handle)) != .Ok) ||
	(abi_custom_constraint_span_status(descriptions, count, description_stride, u64(binding.description.description_size), align_of(f32)) != .Ok)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	context = runtime.default_context();
	for index in 0 ..< count
	{
		input := rawptr(uintptr(descriptions) + uintptr(index * description_stride));
		status: entasis.Status;
		when apply == .Apply
		{
			status = entasis.custom_constraint_apply(&resource.world, abi_constraint_handle_to_core(handles[index]),
				entasis.Constraint_Type_ID(type_id), input, int(binding.description.description_size));
		}
		else
		{
			status = entasis.custom_constraint_get(&resource.world, abi_constraint_handle_to_core(handles[index]),
				entasis.Constraint_Type_ID(type_id), input, int(binding.description.description_size));
		}
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .Constraint_Register, i32(min(index, u64(max(i32)))));
		}
		out_completed^ = index + 1;
	}
	return abi_status_finish(.Ok, diagnostic, .Constraint_Register, i32(min(count, u64(max(i32)))));
}

abi_custom_constraint_get_batch :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID, handles: [^]Entasis_Constraint_Handle, count: u64,
	descriptions: rawptr, description_stride: u64, out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_custom_constraint_transfer_batch(world, type_id, handles, count, descriptions, description_stride, out_completed, diagnostic, .Read);
}

abi_custom_constraint_apply_batch :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID, handles: [^]Entasis_Constraint_Handle, count: u64,
	descriptions: rawptr, description_stride: u64, out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_custom_constraint_transfer_batch(world, type_id, handles, count, descriptions, description_stride, out_completed, diagnostic, .Apply);
}
