package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_constraint_type_valid :: #force_inline proc "contextless" (
	type_id: Entasis_Constraint_Type_ID,
) -> bool
{
	switch type_id
	{
		case 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
		42, 43, 44, 45, 46, 47, 52, 53, 54, 55:
		return true;
	}
	return false;
}

abi_constraint_type_id :: #force_inline proc "contextless" (
	candidate: Entasis_Constraint_Type_ID,
) -> Entasis_Constraint_Type_ID
{
	if abi_constraint_type_valid(candidate)
	{
		return candidate;
	}
	return Entasis_Constraint_Type_ID(ENTASIS_CONSTRAINT_TYPE_INVALID);
}

abi_constraint_body_count :: #force_inline proc "contextless" (
	type_id: Entasis_Constraint_Type_ID,
) -> u32
{
	switch type_id
	{
		case 42, 43, 44, 45:
		return 1;
		case 36:
		return 3;
		case 32:
		return 4;
	}
	if abi_constraint_type_valid(type_id)
	{
		return 2;
	}
	return 0;
}

abi_servo_settings :: #force_inline proc "contextless" (
	maximum_speed, base_speed, maximum_force: f32,
) -> Entasis_Servo_Settings
{
	return Entasis_Servo_Settings(entasis.servo_settings(
			maximum_speed, base_speed, maximum_force,
	));
}

abi_motor_settings :: #force_inline proc "contextless" (
	maximum_force, softness: f32,
) -> Entasis_Motor_Settings
{
	return Entasis_Motor_Settings(entasis.motor_settings(
			maximum_force, softness,
	));
}

abi_constraint_pointer_aligned :: #force_inline proc "contextless" (
	pointer: rawptr, alignment: int,
) -> bool
{
	return pointer != nil && uintptr(pointer) % uintptr(alignment) == 0;
}

abi_constraint_add_typed :: proc "contextless" (
	resource: ^abi_world_resource,
	bodies: [^]Entasis_Body_Handle,
	body_count: u32,
	description: rawptr,
	description_size: u32,
	out_handle: ^Entasis_Constraint_Handle,
	diagnostic: ^Entasis_Diagnostic,
	$T: typeid,
) -> Entasis_Status
{
	if body_count != u32(entasis.constraint_body_count(T)) ||
	description_size != u32(size_of(T)) ||
	!abi_constraint_pointer_aligned(description, align_of(T))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	core_bodies := (cast([^]entasis.Body_Handle)bodies)[:int(body_count)];
	context = runtime.default_context();
	handle, status := entasis.constraint_add(
		&resource.world, core_bodies, (^T)(description)^,
	);
	if status == .Ok
	{
		out_handle^ = abi_constraint_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_add :: proc "contextless" (
	world: ^Entasis_World,
	type_id: Entasis_Constraint_Type_ID,
	bodies: [^]Entasis_Body_Handle,
	body_count: u32,
	description: rawptr,
	description_size: u32,
	out_handle: ^Entasis_Constraint_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = {value=-1};
	}
	if out_handle == nil || description == nil || body_count == 0 || bodies == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	switch type_id
	{
		case 22:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Ball_Socket
		);
		case 23:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Hinge
		);
		case 24:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Swivel_Hinge
		);
		case 25:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Swing_Limit
		);
		case 26:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Twist_Servo
		);
		case 27:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Twist_Limit
		);
		case 28:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Twist_Motor
		);
		case 29:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Servo
		);
		case 30:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Motor
		);
		case 31:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Weld
		);
		case 32:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Volume_Constraint
		);
		case 33:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Distance_Servo
		);
		case 34:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Distance_Limit
		);
		case 35:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Center_Distance_Constraint
		);
		case 36:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Area_Constraint
		);
		case 37:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Point_On_Line_Servo
		);
		case 38:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Linear_Axis_Servo
		);
		case 39:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Linear_Axis_Motor
		);
		case 40:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Linear_Axis_Limit
		);
		case 41:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Axis_Motor
		);
		case 42:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.One_Body_Angular_Servo
		);
		case 43:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.One_Body_Angular_Motor
		);
		case 44:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.One_Body_Linear_Servo
		);
		case 45:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.One_Body_Linear_Motor
		);
		case 46:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Swivel_Hinge
		);
		case 47:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Hinge
		);
		case 52:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Ball_Socket_Motor
		);
		case 53:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Ball_Socket_Servo
		);
		case 54:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Angular_Axis_Gear_Motor
		);
		case 55:
		return abi_constraint_add_typed(
			resource,
			bodies,
			body_count,
			description,
			description_size,
			out_handle,
			diagnostic,
			entasis.Center_Distance_Limit
		);
	}
	return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
}

abi_constraint_get_typed :: proc "contextless" (
	resource: ^abi_world_resource,
	handle: Entasis_Constraint_Handle,
	out_description: rawptr,
	description_size: u32,
	diagnostic: ^Entasis_Diagnostic,
	$T: typeid,
) -> Entasis_Status
{
	if description_size != u32(size_of(T)) ||
	!abi_constraint_pointer_aligned(out_description, align_of(T))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	status := entasis.constraint_get(
		&resource.world, abi_constraint_handle_to_core(handle), (^T)(out_description),
	);
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_get :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	type_id: Entasis_Constraint_Type_ID,
	out_description: rawptr,
	description_size: u32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	switch type_id
	{
		case 22:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Ball_Socket
		);
		case 23:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Hinge
		);
		case 24:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Swivel_Hinge
		);
		case 25:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Swing_Limit
		);
		case 26:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Twist_Servo
		);
		case 27:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Twist_Limit
		);
		case 28:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Twist_Motor
		);
		case 29:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Servo
		);
		case 30:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Motor
		);
		case 31:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Weld
		);
		case 32:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Volume_Constraint
		);
		case 33:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Distance_Servo
		);
		case 34:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Distance_Limit
		);
		case 35:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Center_Distance_Constraint
		);
		case 36:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Area_Constraint
		);
		case 37:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Point_On_Line_Servo
		);
		case 38:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Servo
		);
		case 39:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Motor
		);
		case 40:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Limit
		);
		case 41:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Axis_Motor
		);
		case 42:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.One_Body_Angular_Servo
		);
		case 43:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.One_Body_Angular_Motor
		);
		case 44:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.One_Body_Linear_Servo
		);
		case 45:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.One_Body_Linear_Motor
		);
		case 46:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Swivel_Hinge
		);
		case 47:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Hinge
		);
		case 52:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Ball_Socket_Motor
		);
		case 53:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Ball_Socket_Servo
		);
		case 54:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Angular_Axis_Gear_Motor
		);
		case 55:
		return abi_constraint_get_typed(
			resource,
			handle,
			out_description,
			description_size,
			diagnostic,
			entasis.Center_Distance_Limit
		);
	}
	return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
}

abi_constraint_apply_typed :: proc "contextless" (
	resource: ^abi_world_resource,
	handle: Entasis_Constraint_Handle,
	description: rawptr,
	description_size: u32,
	diagnostic: ^Entasis_Diagnostic,
	$T: typeid,
) -> Entasis_Status
{
	if description_size != u32(size_of(T)) ||
	!abi_constraint_pointer_aligned(description, align_of(T))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	context = runtime.default_context();
	status := entasis.constraint_apply(
		&resource.world, abi_constraint_handle_to_core(handle), (^T)(description)^,
	);
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_apply :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	type_id: Entasis_Constraint_Type_ID,
	description: rawptr,
	description_size: u32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	switch type_id
	{
		case 22:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Ball_Socket
		);
		case 23:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Hinge
		);
		case 24:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Swivel_Hinge
		);
		case 25:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Swing_Limit
		);
		case 26:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Twist_Servo
		);
		case 27:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Twist_Limit
		);
		case 28:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Twist_Motor
		);
		case 29:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Servo
		);
		case 30:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Motor
		);
		case 31:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Weld
		);
		case 32:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Volume_Constraint
		);
		case 33:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Distance_Servo
		);
		case 34:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Distance_Limit
		);
		case 35:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Center_Distance_Constraint
		);
		case 36:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Area_Constraint
		);
		case 37:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Point_On_Line_Servo
		);
		case 38:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Servo
		);
		case 39:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Motor
		);
		case 40:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Linear_Axis_Limit
		);
		case 41:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Axis_Motor
		);
		case 42:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.One_Body_Angular_Servo
		);
		case 43:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.One_Body_Angular_Motor
		);
		case 44:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.One_Body_Linear_Servo
		);
		case 45:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.One_Body_Linear_Motor
		);
		case 46:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Swivel_Hinge
		);
		case 47:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Hinge
		);
		case 52:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Ball_Socket_Motor
		);
		case 53:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Ball_Socket_Servo
		);
		case 54:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Angular_Axis_Gear_Motor
		);
		case 55:
		return abi_constraint_apply_typed(
			resource,
			handle,
			description,
			description_size,
			diagnostic,
			entasis.Center_Distance_Limit
		);
	}
	return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
}

abi_constraint_remove :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.constraint_remove(
			&resource.world, abi_constraint_handle_to_core(handle),
		), diagnostic, .Constraint_Register);
}

abi_constraint_add_batch_typed :: proc "contextless" (
	resource: ^abi_world_resource,
	bodies: [^]Entasis_Body_Handle,
	body_handle_count: u64,
	descriptions: rawptr,
	description_stride: u32,
	count: u64,
	out_handles: [^]Entasis_Constraint_Handle,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
	$T: typeid,
) -> Entasis_Status
{
	if description_stride != u32(size_of(T)) ||
	!abi_constraint_pointer_aligned(descriptions, align_of(T))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	core_descriptions := (cast([^]T)descriptions)[:int(count)];
	core_handles := (cast([^]entasis.Constraint_Handle)out_handles)[:int(count)];
	written := 0;
	status := entasis.Status.Invalid_Argument;
	context = runtime.default_context();
	when T == entasis.One_Body_Angular_Servo || T == entasis.One_Body_Angular_Motor ||
	T == entasis.One_Body_Linear_Servo || T == entasis.One_Body_Linear_Motor
	{
		core_bodies := (cast([^]entasis.Body_Handle)bodies)[:int(count)];
		written, status = entasis.constraint_add_batch_1(
			&resource.world, core_bodies, core_descriptions, core_handles,
		);
	}
	else when T == entasis.Area_Constraint
	{
		core_bodies := (cast([^][3]entasis.Body_Handle)bodies)[:int(count)];
		written, status = entasis.constraint_add_batch_3(
			&resource.world, core_bodies, core_descriptions, core_handles,
		);
	}
	else when T == entasis.Volume_Constraint
	{
		core_bodies := (cast([^][4]entasis.Body_Handle)bodies)[:int(count)];
		written, status = entasis.constraint_add_batch_4(
			&resource.world, core_bodies, core_descriptions, core_handles,
		);
	}
	else
	{
		core_bodies := (cast([^][2]entasis.Body_Handle)bodies)[:int(count)];
		written, status = entasis.constraint_add_batch_2(
			&resource.world, core_bodies, core_descriptions, core_handles,
		);
	}
	if out_completed != nil
	{
		out_completed^ = u64(max(written, 0));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register, i32(written));
}

abi_constraint_add_batch :: proc "contextless" (
	world: ^Entasis_World,
	type_id: Entasis_Constraint_Type_ID,
	bodies: [^]Entasis_Body_Handle,
	body_handle_count: u64,
	descriptions: rawptr,
	description_stride: u32,
	count: u64,
	out_handles: [^]Entasis_Constraint_Handle,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || !abi_count_valid(body_handle_count)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	arity := u64(abi_constraint_body_count(type_id));
	if arity == 0 || count > max(u64) / arity || body_handle_count != count * arity ||
	(count > 0 && (bodies == nil || descriptions == nil || out_handles == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	if count == 0
	{
		return abi_status_finish(.Ok, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	switch type_id
	{
		case 22:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Ball_Socket
		);
		case 23:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Hinge
		);
		case 24:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Swivel_Hinge
		);
		case 25:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Swing_Limit
		);
		case 26:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Twist_Servo
		);
		case 27:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Twist_Limit
		);
		case 28:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Twist_Motor
		);
		case 29:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Servo
		);
		case 30:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Motor
		);
		case 31:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Weld
		);
		case 32:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Volume_Constraint
		);
		case 33:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Distance_Servo
		);
		case 34:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Distance_Limit
		);
		case 35:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Center_Distance_Constraint
		);
		case 36:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Area_Constraint
		);
		case 37:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Point_On_Line_Servo
		);
		case 38:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Servo
		);
		case 39:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Motor
		);
		case 40:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Limit
		);
		case 41:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Axis_Motor
		);
		case 42:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.One_Body_Angular_Servo
		);
		case 43:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.One_Body_Angular_Motor
		);
		case 44:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.One_Body_Linear_Servo
		);
		case 45:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.One_Body_Linear_Motor
		);
		case 46:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Swivel_Hinge
		);
		case 47:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Hinge
		);
		case 52:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Ball_Socket_Motor
		);
		case 53:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Ball_Socket_Servo
		);
		case 54:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Angular_Axis_Gear_Motor
		);
		case 55:
		return abi_constraint_add_batch_typed(
			resource,
			bodies,
			body_handle_count,
			descriptions,
			description_stride,
			count,
			out_handles,
			out_completed,
			diagnostic,
			entasis.Center_Distance_Limit
		);
	}
	return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
}

abi_constraint_apply_batch_typed :: proc "contextless" (
	resource: ^abi_world_resource,
	handles: [^]Entasis_Constraint_Handle,
	descriptions: rawptr,
	description_stride: u32,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
	$T: typeid,
) -> Entasis_Status
{
	if description_stride != u32(size_of(T)) ||
	!abi_constraint_pointer_aligned(descriptions, align_of(T))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	core_handles := (cast([^]entasis.Constraint_Handle)handles)[:int(count)];
	core_descriptions := (cast([^]T)descriptions)[:int(count)];
	context = runtime.default_context();
	applied, status := entasis.constraint_apply_batch(
		&resource.world, core_handles, core_descriptions,
	);
	if out_completed != nil
	{
		out_completed^ = u64(max(applied, 0));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register, i32(applied));
}

abi_constraint_apply_batch :: proc "contextless" (
	world: ^Entasis_World,
	type_id: Entasis_Constraint_Type_ID,
	handles: [^]Entasis_Constraint_Handle,
	descriptions: rawptr,
	description_stride: u32,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || !abi_constraint_type_valid(type_id) ||
	(count > 0 && (handles == nil || descriptions == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	if count == 0
	{
		return abi_status_finish(.Ok, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	switch type_id
	{
		case 22:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Ball_Socket
		);
		case 23:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Hinge
		);
		case 24:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Swivel_Hinge
		);
		case 25:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Swing_Limit
		);
		case 26:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Twist_Servo
		);
		case 27:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Twist_Limit
		);
		case 28:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Twist_Motor
		);
		case 29:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Servo
		);
		case 30:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Motor
		);
		case 31:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Weld
		);
		case 32:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Volume_Constraint
		);
		case 33:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Distance_Servo
		);
		case 34:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Distance_Limit
		);
		case 35:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Center_Distance_Constraint
		);
		case 36:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Area_Constraint
		);
		case 37:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Point_On_Line_Servo
		);
		case 38:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Servo
		);
		case 39:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Motor
		);
		case 40:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Linear_Axis_Limit
		);
		case 41:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Axis_Motor
		);
		case 42:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.One_Body_Angular_Servo
		);
		case 43:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.One_Body_Angular_Motor
		);
		case 44:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.One_Body_Linear_Servo
		);
		case 45:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.One_Body_Linear_Motor
		);
		case 46:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Swivel_Hinge
		);
		case 47:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Hinge
		);
		case 52:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Ball_Socket_Motor
		);
		case 53:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Ball_Socket_Servo
		);
		case 54:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Angular_Axis_Gear_Motor
		);
		case 55:
		return abi_constraint_apply_batch_typed(
			resource,
			handles,
			descriptions,
			description_stride,
			count,
			out_completed,
			diagnostic,
			entasis.Center_Distance_Limit
		);
	}
	return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
}

abi_constraint_remove_batch :: proc "contextless" (
	world: ^Entasis_World,
	handles: [^]Entasis_Constraint_Handle,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && handles == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	if count == 0
	{
		return abi_status_finish(.Ok, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_handles := (cast([^]entasis.Constraint_Handle)handles)[:int(count)];
	context = runtime.default_context();
	removed, status := entasis.constraint_remove_batch(&resource.world, core_handles);
	if out_completed != nil
	{
		out_completed^ = u64(max(removed, 0));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register, i32(removed));
}

abi_constraint_info_from_core :: #force_inline proc "contextless" (
	value: entasis.Constraint_Info,
) -> Entasis_Constraint_Info
{
	result := Entasis_Constraint_Info{
		handle=abi_constraint_handle_from_core(value.handle),
		type_id=Entasis_Constraint_Type_ID(value.type_id),
		body_count=value.body_count,
		state=Entasis_Constraint_State(value.state),
	};
	for index in 0 ..< 4
	{
		result.bodies[index] = abi_body_handle_from_core(value.bodies[index]);
	}
	return result;
}

abi_constraint_inspect :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	out_info: ^Entasis_Constraint_Info,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_info == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_info^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	info, status := entasis.constraint_inspect(
		&resource.world, abi_constraint_handle_to_core(handle),
	);
	if status == .Ok
	{
		out_info^ = abi_constraint_info_from_core(info);
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_count :: proc "contextless" (
	world: ^Entasis_World,
	out_count: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_count == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_count^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	count, status := entasis.constraint_count(&resource.world);
	if status == .Ok
	{
		out_count^ = u64(max(count, 0));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_enumerate :: proc "contextless" (
	world: ^Entasis_World,
	out_infos: [^]Entasis_Constraint_Info,
	capacity: u64,
	out_written: ^u64,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || out_written == nil || out_required == nil ||
	(capacity > 0 && out_infos == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	// assertions verify matching core and ABI layouts. the core fills only the caller's capacity
	output := (cast([^]entasis.Constraint_Info)out_infos)[:int(capacity)];
	written, required, status := entasis.constraint_enumerate(&resource.world, output);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	// clear ABI padding deterministically and normalize exact handle representations
	for index in 0 ..< written
	{
		out_infos[index] = abi_constraint_info_from_core(output[index]);
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_body_constraint_count :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_count: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_count == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_count^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	count, status := entasis.body_constraint_count(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_count^ = u64(max(count, 0));
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_body_constraints :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	output: [^]Entasis_Constraint_Handle,
	capacity: u64,
	out_written: ^u64,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || out_written == nil || out_required == nil ||
	(capacity > 0 && output == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_output := (cast([^]entasis.Constraint_Handle)output)[:int(capacity)];
	written, required, status := entasis.body_constraints(
		&resource.world, abi_body_handle_to_core(handle), core_output,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_body_connected_bodies :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	output: [^]Entasis_Body_Handle,
	capacity: u64,
	out_written: ^u64,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || out_written == nil || out_required == nil ||
	(capacity > 0 && output == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_output := (cast([^]entasis.Body_Handle)output)[:int(capacity)];
	written, required, status := entasis.body_connected_bodies(
		&resource.world, abi_body_handle_to_core(handle), core_output,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_connected_bodies :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	output: [^]Entasis_Body_Handle,
	capacity: u64,
	out_written: ^u64,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || out_written == nil || out_required == nil ||
	(capacity > 0 && output == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_output := (cast([^]entasis.Body_Handle)output)[:int(capacity)];
	written, required, status := entasis.constraint_connected_bodies(
		&resource.world, abi_constraint_handle_to_core(handle), core_output,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_body_activation_state :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_state: ^Entasis_Body_Activation_State,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_state == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_state^ = Entasis_Body_Activation_State(0);
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	state, status := entasis.body_activation_state(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_state^ = Entasis_Body_Activation_State(state);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_is_active :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_value: ^Entasis_Bool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.body_is_active(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_value^ = abi_bool(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_is_sleeping :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_value: ^Entasis_Bool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.body_is_sleeping(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_value^ = abi_bool(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_awaken :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_awaken(
			&resource.world, abi_body_handle_to_core(handle),
		), diagnostic, .None);
}

abi_body_apply_linear_impulse :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	impulse: Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.body_apply_linear_impulse(
			&resource.world, abi_body_handle_to_core(handle), abi_vector3_to_core(impulse),
		), diagnostic, .None);
}

abi_body_apply_angular_impulse :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	impulse: Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.body_apply_angular_impulse(
			&resource.world, abi_body_handle_to_core(handle), abi_vector3_to_core(impulse),
		), diagnostic, .None);
}

abi_body_apply_impulse :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	impulse: Entasis_Vector3,
	offset: Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.body_apply_impulse(
			&resource.world, abi_body_handle_to_core(handle),
			abi_vector3_to_core(impulse), abi_vector3_to_core(offset),
		), diagnostic, .None);
}

abi_body_velocity_at_offset :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	offset: Entasis_Vector3,
	out_velocity: ^Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_velocity == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_velocity^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.body_velocity_at_offset(
		&resource.world, abi_body_handle_to_core(handle), abi_vector3_to_core(offset),
	);
	if status == .Ok
	{
		out_velocity^ = abi_vector3_from_core(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_bounds :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_bounds: ^Entasis_Bounding_Box,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_bounds == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_bounds^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.body_bounds(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_bounds^ = abi_bounds_from_core(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_update_bounds :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.body_update_bounds(
			&resource.world, abi_body_handle_to_core(handle),
		), diagnostic, .None);
}

abi_static_bounds :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Static_Handle,
	out_bounds: ^Entasis_Bounding_Box,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_bounds == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_bounds^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.static_bounds(&resource.world, abi_static_handle_to_core(handle));
	if status == .Ok
	{
		out_bounds^ = abi_bounds_from_core(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_update_bounds :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Static_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.static_update_bounds(
			&resource.world, abi_static_handle_to_core(handle),
		), diagnostic, .None);
}

abi_constraint_accumulated_impulses :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	output: [^]f32,
	capacity: u64,
	out_written: ^u64,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(capacity) || out_written == nil || out_required == nil ||
	(capacity > 0 && output == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	written, required, status := entasis.constraint_accumulated_impulses(
		&resource.world, abi_constraint_handle_to_core(handle), output[:int(capacity)],
	);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_accumulated_impulse_magnitude :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	out_value: ^f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_value^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.constraint_accumulated_impulse_magnitude(
		&resource.world, abi_constraint_handle_to_core(handle),
	);
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_constraint_accumulated_impulse_magnitude_squared :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	out_value: ^f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_value^ = 0;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	value, status := entasis.constraint_accumulated_impulse_magnitude_squared(
		&resource.world, abi_constraint_handle_to_core(handle),
	);
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_world_scale_active_accumulated_impulses :: proc "contextless" (
	world: ^Entasis_World,
	scale: f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.world_scale_active_accumulated_impulses(
			&resource.world, scale,
		), diagnostic, .Constraint_Register);
}

abi_world_scale_accumulated_impulses :: proc "contextless" (
	world: ^Entasis_World,
	scale: f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	return abi_status_finish(entasis.world_scale_accumulated_impulses(
			&resource.world, scale,
		), diagnostic, .Constraint_Register);
}

abi_solver_contact_data_from_core :: #force_inline proc "contextless" (
	value: entasis.Solver_Contact_Data,
) -> Entasis_Solver_Contact_Data
{
	result := Entasis_Solver_Contact_Data{
		constraint=abi_constraint_handle_from_core(value.constraint),
		pair_a={packed=value.pair_a.packed},
		pair_b={packed=value.pair_b.packed},
		kind=Entasis_Contact_Constraint_Kind(value.kind),
		body_count=value.body_count,
		contact_count=value.contact_count,
		impulse_count=value.impulse_count,
		offset_b=abi_vector3_from_core(value.offset_b),
		normal=abi_vector3_from_core(value.normal),
		material=abi_material_from_core(value.material),
	};
	for index in 0 ..< 4
	{
		result.body_handles[index] = abi_body_handle_from_core(value.body_handles[index]);
		contact := value.contacts[index];
		result.contacts[index] = {
			offset_a=abi_vector3_from_core(contact.offset_a),
			normal=abi_vector3_from_core(contact.normal),
			depth=contact.depth,
			feature_id=contact.feature_id,
			normal_impulse=contact.normal_impulse,
		};
	}
	for index in 0 ..< 32
	{
		result.impulses[index] = value.impulses[index];
	}
	return result;
}

abi_solver_contact_data :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Constraint_Handle,
	out_data: ^Entasis_Solver_Contact_Data,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_data == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Constraint_Register);
	}
	out_data^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Constraint_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_handle := abi_constraint_handle_to_core(handle);
	value, status := entasis.solver_contact_data(&resource.world, core_handle);
	if status == .Ok
	{
		out_data^ = abi_solver_contact_data_from_core(value);
	}
	else if status == .Not_Found
	{
		// a live user-authored constraint is not a solver contact. distinguish that
		// domain error from a stale or unknown handle without exposing solver IDs
		_, inspect_status := entasis.constraint_inspect(&resource.world, core_handle);
		if inspect_status == .Ok
		{
			status = .Invalid_Argument;
		}
		else if inspect_status != .Not_Found
		{
			status = inspect_status;
		}
	}
	return abi_status_finish(status, diagnostic, .Constraint_Register);
}

abi_fixed_stepper :: #force_inline proc "contextless" (
	timestep: f32,
	maximum_steps: u8,
) -> Entasis_Fixed_Stepper
{
	return transmute(Entasis_Fixed_Stepper)entasis.fixed_stepper(timestep, maximum_steps);
}

abi_fixed_stepper_reset :: #force_inline proc "contextless" (
	stepper: ^Entasis_Fixed_Stepper,
) -> Entasis_Status
{
	return abi_status(entasis.fixed_stepper_reset((^entasis.Fixed_Stepper)(stepper)));
}

abi_fixed_stepper_update :: proc "contextless" (
	stepper: ^Entasis_Fixed_Stepper,
	world: ^Entasis_World,
	elapsed: f32,
	out_steps: ^u32,
	out_alpha: ^f32,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_steps != nil
	{
		out_steps^ = 0;
	}
	if out_alpha != nil
	{
		out_alpha^ = 0;
	}
	if stepper == nil || out_steps == nil || out_alpha == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .World_Initialize);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	steps, alpha, status := entasis.fixed_stepper_update(
		(^entasis.Fixed_Stepper)(stepper), &resource.world, elapsed,
	);
	out_steps^ = u32(max(steps, 0));
	out_alpha^ = alpha;
	return abi_status_finish(status, diagnostic, .World_Initialize);
}

abi_solve_description_substeps :: #force_inline proc "contextless" (
	substeps, velocity_iterations, fallback_batch_threshold: i32,
) -> Entasis_Solve_Description
{
	value := entasis.solve_description_substeps(
		int(substeps), int(velocity_iterations), int(fallback_batch_threshold),
	);
	return {
		struct_size=u32(size_of(Entasis_Solve_Description)),
		struct_version=ENTASIS_STRUCT_VERSION,
		velocity_iterations=value.velocity_iterations,
		substeps=value.substeps,
		fallback_batch_threshold=value.fallback_batch_threshold,
	};
}
#assert(size_of(Entasis_Constraint_Handle) == size_of(entasis.Constraint_Handle));
#assert(align_of(Entasis_Constraint_Handle) == align_of(entasis.Constraint_Handle));
#assert(size_of(Entasis_Body_Handle) == size_of(entasis.Body_Handle));
#assert(align_of(Entasis_Body_Handle) == align_of(entasis.Body_Handle));
#assert(size_of(Entasis_Static_Handle) == size_of(entasis.Static_Handle));
#assert(align_of(Entasis_Static_Handle) == align_of(entasis.Static_Handle));
#assert(size_of(Entasis_Motor_Settings) == size_of(entasis.Motor_Settings));
#assert(align_of(Entasis_Motor_Settings) == align_of(entasis.Motor_Settings));
#assert(size_of(Entasis_Servo_Settings) == size_of(entasis.Servo_Settings));
#assert(align_of(Entasis_Servo_Settings) == align_of(entasis.Servo_Settings));
#assert(size_of(Entasis_Ball_Socket) == size_of(entasis.Ball_Socket));
#assert(align_of(Entasis_Ball_Socket) == align_of(entasis.Ball_Socket));
#assert(size_of(Entasis_Angular_Hinge) == size_of(entasis.Angular_Hinge));
#assert(align_of(Entasis_Angular_Hinge) == align_of(entasis.Angular_Hinge));
#assert(size_of(Entasis_Angular_Swivel_Hinge) == size_of(entasis.Angular_Swivel_Hinge));
#assert(align_of(Entasis_Angular_Swivel_Hinge) == align_of(entasis.Angular_Swivel_Hinge));
#assert(size_of(Entasis_Swing_Limit) == size_of(entasis.Swing_Limit));
#assert(align_of(Entasis_Swing_Limit) == align_of(entasis.Swing_Limit));
#assert(size_of(Entasis_Twist_Servo) == size_of(entasis.Twist_Servo));
#assert(align_of(Entasis_Twist_Servo) == align_of(entasis.Twist_Servo));
#assert(size_of(Entasis_Twist_Limit) == size_of(entasis.Twist_Limit));
#assert(align_of(Entasis_Twist_Limit) == align_of(entasis.Twist_Limit));
#assert(size_of(Entasis_Twist_Motor) == size_of(entasis.Twist_Motor));
#assert(align_of(Entasis_Twist_Motor) == align_of(entasis.Twist_Motor));
#assert(size_of(Entasis_Angular_Servo) == size_of(entasis.Angular_Servo));
#assert(align_of(Entasis_Angular_Servo) == align_of(entasis.Angular_Servo));
#assert(size_of(Entasis_Angular_Motor) == size_of(entasis.Angular_Motor));
#assert(align_of(Entasis_Angular_Motor) == align_of(entasis.Angular_Motor));
#assert(size_of(Entasis_Weld) == size_of(entasis.Weld));
#assert(align_of(Entasis_Weld) == align_of(entasis.Weld));
#assert(size_of(Entasis_Volume_Constraint) == size_of(entasis.Volume_Constraint));
#assert(align_of(Entasis_Volume_Constraint) == align_of(entasis.Volume_Constraint));
#assert(size_of(Entasis_Distance_Servo) == size_of(entasis.Distance_Servo));
#assert(align_of(Entasis_Distance_Servo) == align_of(entasis.Distance_Servo));
#assert(size_of(Entasis_Distance_Limit) == size_of(entasis.Distance_Limit));
#assert(align_of(Entasis_Distance_Limit) == align_of(entasis.Distance_Limit));
#assert(size_of(Entasis_Center_Distance_Constraint) == size_of(entasis.Center_Distance_Constraint));
#assert(align_of(Entasis_Center_Distance_Constraint) == align_of(entasis.Center_Distance_Constraint));
#assert(size_of(Entasis_Area_Constraint) == size_of(entasis.Area_Constraint));
#assert(align_of(Entasis_Area_Constraint) == align_of(entasis.Area_Constraint));
#assert(size_of(Entasis_Point_On_Line_Servo) == size_of(entasis.Point_On_Line_Servo));
#assert(align_of(Entasis_Point_On_Line_Servo) == align_of(entasis.Point_On_Line_Servo));
#assert(size_of(Entasis_Linear_Axis_Servo) == size_of(entasis.Linear_Axis_Servo));
#assert(align_of(Entasis_Linear_Axis_Servo) == align_of(entasis.Linear_Axis_Servo));
#assert(size_of(Entasis_Linear_Axis_Motor) == size_of(entasis.Linear_Axis_Motor));
#assert(align_of(Entasis_Linear_Axis_Motor) == align_of(entasis.Linear_Axis_Motor));
#assert(size_of(Entasis_Linear_Axis_Limit) == size_of(entasis.Linear_Axis_Limit));
#assert(align_of(Entasis_Linear_Axis_Limit) == align_of(entasis.Linear_Axis_Limit));
#assert(size_of(Entasis_Angular_Axis_Motor) == size_of(entasis.Angular_Axis_Motor));
#assert(align_of(Entasis_Angular_Axis_Motor) == align_of(entasis.Angular_Axis_Motor));
#assert(size_of(Entasis_One_Body_Angular_Servo) == size_of(entasis.One_Body_Angular_Servo));
#assert(align_of(Entasis_One_Body_Angular_Servo) == align_of(entasis.One_Body_Angular_Servo));
#assert(size_of(Entasis_One_Body_Angular_Motor) == size_of(entasis.One_Body_Angular_Motor));
#assert(align_of(Entasis_One_Body_Angular_Motor) == align_of(entasis.One_Body_Angular_Motor));
#assert(size_of(Entasis_One_Body_Linear_Servo) == size_of(entasis.One_Body_Linear_Servo));
#assert(align_of(Entasis_One_Body_Linear_Servo) == align_of(entasis.One_Body_Linear_Servo));
#assert(size_of(Entasis_One_Body_Linear_Motor) == size_of(entasis.One_Body_Linear_Motor));
#assert(align_of(Entasis_One_Body_Linear_Motor) == align_of(entasis.One_Body_Linear_Motor));
#assert(size_of(Entasis_Swivel_Hinge) == size_of(entasis.Swivel_Hinge));
#assert(align_of(Entasis_Swivel_Hinge) == align_of(entasis.Swivel_Hinge));
#assert(size_of(Entasis_Hinge) == size_of(entasis.Hinge));
#assert(align_of(Entasis_Hinge) == align_of(entasis.Hinge));
#assert(size_of(Entasis_Ball_Socket_Motor) == size_of(entasis.Ball_Socket_Motor));
#assert(align_of(Entasis_Ball_Socket_Motor) == align_of(entasis.Ball_Socket_Motor));
#assert(size_of(Entasis_Ball_Socket_Servo) == size_of(entasis.Ball_Socket_Servo));
#assert(align_of(Entasis_Ball_Socket_Servo) == align_of(entasis.Ball_Socket_Servo));
#assert(size_of(Entasis_Angular_Axis_Gear_Motor) == size_of(entasis.Angular_Axis_Gear_Motor));
#assert(align_of(Entasis_Angular_Axis_Gear_Motor) == align_of(entasis.Angular_Axis_Gear_Motor));
#assert(size_of(Entasis_Center_Distance_Limit) == size_of(entasis.Center_Distance_Limit));
#assert(align_of(Entasis_Center_Distance_Limit) == align_of(entasis.Center_Distance_Limit));
#assert(size_of(Entasis_Constraint_Info) == size_of(entasis.Constraint_Info));
#assert(align_of(Entasis_Constraint_Info) == align_of(entasis.Constraint_Info));
#assert(size_of(Entasis_Fixed_Stepper) == size_of(entasis.Fixed_Stepper));
#assert(align_of(Entasis_Fixed_Stepper) == align_of(entasis.Fixed_Stepper));
#assert(size_of(Entasis_Solver_Contact_Point) == size_of(entasis.Solver_Contact_Point));
#assert(align_of(Entasis_Solver_Contact_Point) == align_of(entasis.Solver_Contact_Point));
#assert(size_of(Entasis_Solver_Contact_Data) == size_of(entasis.Solver_Contact_Data));
#assert(align_of(Entasis_Solver_Contact_Data) == align_of(entasis.Solver_Contact_Data));
