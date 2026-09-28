package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_body_control_configuration_default :: proc "contextless" () -> Entasis_Body_Control_Configuration
{
	value: entasis.Body_Control_Configuration = entasis.body_control_configuration_default();
	return {struct_size=u32(size_of(Entasis_Body_Control_Configuration)), struct_version=ENTASIS_STRUCT_VERSION,
		input_capacity=value.input_capacity, settings_capacity=value.settings_capacity};
}

abi_world_enable_body_control :: proc "contextless" (
	world: ^Entasis_World, configuration: ^Entasis_Body_Control_Configuration, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	selected: Entasis_Body_Control_Configuration = abi_body_control_configuration_default();
	if configuration != nil
	{
		selected = configuration^;
	}
	if selected.struct_size != size_of(Entasis_Body_Control_Configuration) || selected.struct_version != ENTASIS_STRUCT_VERSION
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_enable_body_control(&resource.world,
			{input_capacity=selected.input_capacity, settings_capacity=selected.settings_capacity}), diagnostic, .None);
}

abi_world_disable_body_control :: proc "contextless" (
	world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_disable_body_control(&resource.world), diagnostic, .None);
}

abi_body_control_reserve :: proc "contextless" (
	world: ^Entasis_World, input_capacity, settings_capacity: i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_control_reserve(&resource.world, input_capacity, settings_capacity), diagnostic, .None);
}

abi_body_add_force :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, force: Entasis_Vector3, mode: Entasis_Body_Input_Mode, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if mode > 1 || wake > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_add_force(&resource.world, abi_body_handle_to_core(body), abi_vector3_to_core(force), entasis.Body_Input_Mode(mode), entasis.Body_Control_Wake(wake)), diagnostic, .None);
}

abi_body_add_torque :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, torque: Entasis_Vector3, mode: Entasis_Body_Input_Mode, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if mode > 1 || wake > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_add_torque(&resource.world, abi_body_handle_to_core(body), abi_vector3_to_core(torque), entasis.Body_Input_Mode(mode), entasis.Body_Control_Wake(wake)), diagnostic, .None);
}

abi_body_add_force_at_position :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, force, position: Entasis_Vector3, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if wake > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_add_force_at_position(&resource.world, abi_body_handle_to_core(body), abi_vector3_to_core(force), abi_vector3_to_core(position), entasis.Body_Control_Wake(wake)), diagnostic, .None);
}

abi_body_clear_inputs :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_clear_inputs(&resource.world, abi_body_handle_to_core(body)), diagnostic, .None);
}

abi_body_clear_kinematic_target :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_clear_kinematic_target(&resource.world, abi_body_handle_to_core(body)), diagnostic, .None);
}

abi_body_set_damping :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, settings: ^Entasis_Body_Damping, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if settings == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	selected: Entasis_Body_Damping = settings^;
	if selected.mode > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_set_damping(&resource.world, abi_body_handle_to_core(body), {linear=selected.linear, angular=selected.angular, mode=entasis.Body_Damping_Mode(selected.mode)}), diagnostic, .None);
}

abi_body_get_damping :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, output: ^Entasis_Body_Damping, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if output == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value, status := entasis.body_get_damping(&resource.world, abi_body_handle_to_core(body));
	if status == .Ok
	{
		output^ = {linear=value.linear, angular=value.angular, mode=u32(value.mode)};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_set_kinematic_target :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, target: ^Entasis_Rigid_Pose, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if target == nil || wake > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	selected: entasis.Rigid_Pose = abi_pose_to_core(target^);
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_set_kinematic_target(&resource.world, abi_body_handle_to_core(body), selected, entasis.Body_Control_Wake(wake)), diagnostic, .None);
}

abi_body_get_kinematic_target :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, output: ^Entasis_Rigid_Pose, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if output == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value, status := entasis.body_get_kinematic_target(&resource.world, abi_body_handle_to_core(body));
	if status == .Ok
	{
		output^ = abi_pose_from_core(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

@(private)
abi_body_axis_lock_from_core :: proc "contextless" (value: entasis.Body_Axis_Lock) -> Entasis_Body_Axis_Lock
{
	return {struct_size=u32(size_of(Entasis_Body_Axis_Lock)), struct_version=ENTASIS_STRUCT_VERSION,
		reference=abi_pose_from_core(value.reference), linear_axes=u32(transmute(u8)value.linear_axes),
		angular_axes=u32(transmute(u8)value.angular_axes), spring_settings=abi_spring_from_core(value.spring_settings)};
}

abi_body_axis_lock_default :: proc "contextless" (reference: Entasis_Rigid_Pose) -> Entasis_Body_Axis_Lock
{
	return abi_body_axis_lock_from_core(entasis.body_axis_lock_default(abi_pose_to_core(reference)));
}

abi_body_set_axis_lock :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, description: ^Entasis_Body_Axis_Lock,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	selected: Entasis_Body_Axis_Lock = description^;
	if selected.struct_size != size_of(Entasis_Body_Axis_Lock) || selected.struct_version != ENTASIS_STRUCT_VERSION ||
	selected.linear_axes > 7 || selected.angular_axes > 7
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_set_axis_lock(&resource.world, abi_body_handle_to_core(body), {
				reference=abi_pose_to_core(selected.reference),
				linear_axes=transmute(entasis.Body_Lock_Axes)u8(selected.linear_axes),
				angular_axes=transmute(entasis.Body_Lock_Axes)u8(selected.angular_axes),
				spring_settings=abi_spring_to_core(selected.spring_settings),
		}), diagnostic, .None);
}

abi_body_get_axis_lock :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, output: ^Entasis_Body_Axis_Lock,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if output == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value, status := entasis.body_get_axis_lock(&resource.world, abi_body_handle_to_core(body));
	if status == .Ok
	{
		output^ = abi_body_axis_lock_from_core(value);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_clear_axis_lock :: proc "contextless" (
	world: ^Entasis_World, body: Entasis_Body_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_clear_axis_lock(&resource.world, abi_body_handle_to_core(body)), diagnostic, .None);
}
