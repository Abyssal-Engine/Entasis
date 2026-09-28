package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_pose :: #force_inline proc "contextless" (
	position: Entasis_Vector3,
	orientation: Entasis_Quaternion,
) -> Entasis_Rigid_Pose
{
	return abi_pose_from_core(entasis.pose(
			abi_vector3_to_core(position), cast(entasis.Quaternion)orientation,
	));
}

abi_velocity :: #force_inline proc "contextless" (
	linear, angular: Entasis_Vector3,
) -> Entasis_Body_Velocity
{
	return abi_velocity_from_core(entasis.velocity(
			abi_vector3_to_core(linear), abi_vector3_to_core(angular),
	));
}

abi_body_activity :: #force_inline proc "contextless" (
	sleep_threshold: f32,
	minimum_timesteps_under_threshold: u8,
) -> Entasis_Activity_Description
{
	return abi_activity_from_core(entasis.body_activity(
			sleep_threshold, minimum_timesteps_under_threshold,
	));
}

abi_body_activity_default :: #force_inline proc "contextless" () -> Entasis_Activity_Description
{
	return abi_activity_from_core(entasis.body_activity_default());
}

abi_ccd_discrete :: #force_inline proc "contextless" () -> Entasis_Continuous_Detection
{
	return abi_continuity_from_core(entasis.ccd_discrete());
}

abi_ccd_passive :: #force_inline proc "contextless" () -> Entasis_Continuous_Detection
{
	return abi_continuity_from_core(entasis.ccd_passive());
}

abi_ccd_continuous :: #force_inline proc "contextless" (
	minimum_sweep_timestep, convergence_threshold: f32,
) -> Entasis_Continuous_Detection
{
	return abi_continuity_from_core(entasis.ccd_continuous(
			minimum_sweep_timestep, convergence_threshold,
	));
}

abi_collidable :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	continuity: Entasis_Continuous_Detection,
	minimum_speculative_margin, maximum_speculative_margin: f32,
) -> Entasis_Collidable_Description
{
	return abi_collidable_from_core(entasis.collidable(
			abi_shape_handle_to_core(shape),
			abi_continuity_to_core(continuity),
			minimum_speculative_margin,
			maximum_speculative_margin,
	));
}

abi_body_dynamic :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	inertia: Entasis_Body_Inertia,
	pose: Entasis_Rigid_Pose,
	velocity: Entasis_Body_Velocity,
	activity: Entasis_Activity_Description,
) -> Entasis_Body_Description
{
	return abi_body_description_from_core(entasis.body_dynamic(
			abi_shape_handle_to_core(shape),
			abi_inertia_to_core(inertia),
			abi_pose_to_core(pose),
			abi_velocity_to_core(velocity),
			abi_activity_to_core(activity),
	));
}

abi_body_kinematic :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	velocity: Entasis_Body_Velocity,
	activity: Entasis_Activity_Description,
) -> Entasis_Body_Description
{
	return abi_body_description_from_core(entasis.body_kinematic(
			abi_shape_handle_to_core(shape),
			abi_pose_to_core(pose),
			abi_velocity_to_core(velocity),
			abi_activity_to_core(activity),
	));
}

abi_body_shapeless :: #force_inline proc "contextless" (
	inertia: Entasis_Body_Inertia,
	pose: Entasis_Rigid_Pose,
	velocity: Entasis_Body_Velocity,
	activity: Entasis_Activity_Description,
) -> Entasis_Body_Description
{
	return abi_body_description_from_core(entasis.body_shapeless(
			abi_inertia_to_core(inertia),
			abi_pose_to_core(pose),
			abi_velocity_to_core(velocity),
			abi_activity_to_core(activity),
	));
}

abi_body_add :: proc "contextless" (
	world: ^Entasis_World,
	description: ^Entasis_Body_Description,
	out_handle: ^Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_body_handle_invalid();
	}
	if description == nil || out_handle == nil
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
	handle, status := entasis.body_add(
		&resource.world, abi_body_description_to_core(description^),
	);
	if status == .Ok
	{
		out_handle^ = abi_body_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_get :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	out_state: ^Entasis_Body_State,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_state == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_state^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	state, status := entasis.body_get(&resource.world, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_state^ = abi_body_description_from_core(state);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_apply :: proc "contextless" (
	world: ^Entasis_World,
	handle: Entasis_Body_Handle,
	description: ^Entasis_Body_Description,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if description == nil
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
	return abi_status_finish(entasis.body_apply(
			&resource.world,
			abi_body_handle_to_core(handle),
			abi_body_description_to_core(description^),
		), diagnostic, .None);
}

abi_body_remove :: proc "contextless" (
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
	return abi_status_finish(entasis.body_remove(
			&resource.world, abi_body_handle_to_core(handle),
		), diagnostic, .None);
}

abi_body_set_pose :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	value: ^Entasis_Rigid_Pose, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
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
	return abi_status_finish(entasis.body_set_pose(
			&resource.world, abi_body_handle_to_core(handle), abi_pose_to_core(value^),
		), diagnostic, .None);
}

abi_body_set_velocity :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	value: ^Entasis_Body_Velocity, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
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
	return abi_status_finish(entasis.body_set_velocity(
			&resource.world, abi_body_handle_to_core(handle), abi_velocity_to_core(value^),
		), diagnostic, .None);
}

abi_body_set_inertia :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	value: ^Entasis_Body_Inertia, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
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
	return abi_status_finish(entasis.body_set_inertia(
			&resource.world, abi_body_handle_to_core(handle), abi_inertia_to_core(value^),
		), diagnostic, .None);
}

abi_body_set_activity :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	value: ^Entasis_Activity_Description, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
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
	return abi_status_finish(entasis.body_set_activity(
			&resource.world, abi_body_handle_to_core(handle), abi_activity_to_core(value^),
		), diagnostic, .None);
}

abi_body_set_collidable :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	value: ^Entasis_Collidable_Description, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
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
	return abi_status_finish(entasis.body_set_collidable(
			&resource.world, abi_body_handle_to_core(handle), abi_collidable_to_core(value^),
		), diagnostic, .None);
}

abi_body_set_shape :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Body_Handle,
	shape: Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.body_set_shape(
			&resource.world, abi_body_handle_to_core(handle), abi_shape_handle_to_core(shape),
		), diagnostic, .None);
}

abi_body_add_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, descriptions: [^]Entasis_Body_Description,
	count: u64, out_handles: [^]Entasis_Body_Handle,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (descriptions == nil || out_handles == nil))
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
	core_descriptions := (cast([^]entasis.Body_Description)descriptions)[:int(count)];
	core_handles := (cast([^]entasis.Body_Handle)out_handles)[:int(count)];
	completed, status := entasis.body_add_batch(
		&resource.world, core_descriptions, core_handles,
	);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_body_get_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	count: u64, out_states: [^]Entasis_Body_State,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || out_states == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	for index in 0 ..< int(count)
	{
		state, status := entasis.body_get(&resource.world, abi_body_handle_to_core(handles[index]));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		out_states[index] = abi_body_description_from_core(state);
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_apply_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	descriptions: [^]Entasis_Body_Description, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || descriptions == nil))
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
	core_handles := (cast([^]entasis.Body_Handle)handles)[:int(count)];
	core_descriptions := (cast([^]entasis.Body_Description)descriptions)[:int(count)];
	completed, status := entasis.body_apply_batch(
		&resource.world, core_handles, core_descriptions,
	);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_body_remove_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	count: u64, out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && handles == nil)
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
	core_handles := (cast([^]entasis.Body_Handle)handles)[:int(count)];
	completed, status := entasis.body_remove_batch(&resource.world, core_handles);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_body_set_pose_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Rigid_Pose, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_pose(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_pose_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_set_velocity_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Body_Velocity, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_velocity(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_velocity_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_set_inertia_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Body_Inertia, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_inertia(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_inertia_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_set_activity_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Activity_Description, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_activity(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_activity_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_set_collidable_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Collidable_Description, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_collidable(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_collidable_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_set_shape_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
	values: [^]Entasis_Shape_Handle, count: u64,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
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
	for index in 0 ..< int(count)
	{
		status := entasis.body_set_shape(
			&resource.world,
			abi_body_handle_to_core(handles[index]),
			abi_shape_handle_to_core(values[index])
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_body :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	continuity: Entasis_Continuous_Detection,
) -> Entasis_Static_Description
{
	return abi_static_description_from_core(entasis.static_body(
			abi_shape_handle_to_core(shape), abi_pose_to_core(pose), abi_continuity_to_core(continuity),
	));
}

abi_awakening_to_core :: #force_inline proc "contextless" (
	awakening: Entasis_Awakening_Policy,
) -> (entasis.Awakening_Policy, entasis.Status)
{
	if awakening > 1
	{
		return {}, .Invalid_Argument;
	}
	return entasis.Awakening_Policy(awakening), .Ok;
}

abi_static_add :: proc "contextless" (
	world: ^Entasis_World, description: ^Entasis_Static_Description,
	awakening: Entasis_Awakening_Policy, out_handle: ^Entasis_Static_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_static_handle_invalid();
	}
	if description == nil || out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	handle, status := entasis.static_add(&resource.world, abi_static_description_to_core(description^), wake);
	if status == .Ok
	{
		out_handle^ = abi_static_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_get :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	out_state: ^Entasis_Static_State, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_state == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_state^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	state, status := entasis.static_get(&resource.world, abi_static_handle_to_core(handle));
	if status == .Ok
	{
		out_state^ = abi_static_description_from_core(state);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_apply :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	description: ^Entasis_Static_Description, awakening: Entasis_Awakening_Policy,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_apply(
			&resource.world, abi_static_handle_to_core(handle), abi_static_description_to_core(description^), wake,
		), diagnostic, .None);
}

abi_static_remove :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	awakening: Entasis_Awakening_Policy, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_remove(
			&resource.world, abi_static_handle_to_core(handle), wake,
		), diagnostic, .None);
}

abi_static_set_pose :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	value: ^Entasis_Rigid_Pose, awakening: Entasis_Awakening_Policy,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_set_pose(
			&resource.world, abi_static_handle_to_core(handle), abi_pose_to_core(value^), wake,
		), diagnostic, .None);
}

abi_static_set_shape :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	shape: Entasis_Shape_Handle, awakening: Entasis_Awakening_Policy,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_set_shape(
			&resource.world, abi_static_handle_to_core(handle), abi_shape_handle_to_core(shape), wake,
		), diagnostic, .None);
}

abi_static_set_continuity :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	value: ^Entasis_Continuous_Detection, awakening: Entasis_Awakening_Policy,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_set_continuity(
			&resource.world, abi_static_handle_to_core(handle), abi_continuity_to_core(value^), wake,
		), diagnostic, .None);
}

abi_static_add_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, descriptions: [^]Entasis_Static_Description,
	count: u64, awakening: Entasis_Awakening_Policy,
	out_handles: [^]Entasis_Static_Handle, out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (descriptions == nil || out_handles == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_descriptions := (cast([^]entasis.Static_Description)descriptions)[:int(count)];
	core_handles := (cast([^]entasis.Static_Handle)out_handles)[:int(count)];
	completed, status := entasis.static_add_batch(
		&resource.world, core_descriptions, core_handles, wake,
	);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_static_get_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Static_Handle,
	count: u64, out_states: [^]Entasis_Static_State,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || out_states == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	for index in 0 ..< int(count)
	{
		state, status := entasis.static_get(&resource.world, abi_static_handle_to_core(handles[index]));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		out_states[index] = abi_static_description_from_core(state);
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_apply_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Static_Handle,
	descriptions: [^]Entasis_Static_Description, count: u64,
	awakening: Entasis_Awakening_Policy, out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || descriptions == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_handles := (cast([^]entasis.Static_Handle)handles)[:int(count)];
	core_descriptions := (cast([^]entasis.Static_Description)descriptions)[:int(count)];
	completed, status := entasis.static_apply_batch(
		&resource.world, core_handles, core_descriptions, wake,
	);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_static_remove_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Static_Handle,
	count: u64, awakening: Entasis_Awakening_Policy,
	out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && handles == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_handles := (cast([^]entasis.Static_Handle)handles)[:int(count)];
	completed, status := entasis.static_remove_batch(&resource.world, core_handles, wake);
	if out_completed != nil
	{
		out_completed^ = u64(completed);
	}
	return abi_status_finish(status, diagnostic, .None, i32(completed));
}

abi_static_set_pose_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Static_Handle,
	values: [^]Entasis_Rigid_Pose, count: u64,
	awakening: Entasis_Awakening_Policy, out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (handles == nil || values == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	wake, wake_status := abi_awakening_to_core(awakening);
	if wake_status != .Ok
	{
		return abi_status_finish(wake_status, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	for index in 0 ..< int(count)
	{
		status := entasis.static_set_pose(
			&resource.world,
			abi_static_handle_to_core(handles[index]),
			abi_pose_to_core(values[index]),
			wake
		);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_awakening_bridge :: struct
{
	callback: Entasis_Static_Awakening_Filter_Proc,
	user_context: rawptr,
}

abi_static_awakening_filter :: proc "contextless" (raw: rawptr, body: entasis.Body_Handle) -> bool
{
	bridge := cast(^abi_static_awakening_bridge)raw;
	return bridge.callback(bridge.user_context, abi_body_handle_from_core(body)) != 0;
}

abi_static_add_filtered :: proc "contextless" (
	world: ^Entasis_World, description: ^Entasis_Static_Description,
	filter: Entasis_Static_Awakening_Filter_Proc, user_context: rawptr,
	out_handle: ^Entasis_Static_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_static_handle_invalid();
	}
	if description == nil || out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	bridge := abi_static_awakening_bridge{callback=filter, user_context=user_context};
	core_filter: entasis.Static_Awakening_Filter_Proc;
	if filter != nil
	{
		core_filter = abi_static_awakening_filter;
	}
	context = runtime.default_context();
	handle, status := entasis.static_add_filtered(&resource.world, abi_static_description_to_core(description^), core_filter, &bridge);
	if status == .Ok
	{
		out_handle^ = abi_static_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_remove_filtered :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle,
	filter: Entasis_Static_Awakening_Filter_Proc, user_context: rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	bridge := abi_static_awakening_bridge{callback=filter, user_context=user_context};
	core_filter: entasis.Static_Awakening_Filter_Proc;
	if filter != nil
	{
		core_filter = abi_static_awakening_filter;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_remove_filtered(&resource.world, abi_static_handle_to_core(handle), core_filter, &bridge), diagnostic, .None);
}

abi_static_apply_filtered :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Static_Handle, description: ^Entasis_Static_Description,
	filter: Entasis_Static_Awakening_Filter_Proc, user_context: rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if description == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	bridge := abi_static_awakening_bridge{callback=filter, user_context=user_context};
	core_filter: entasis.Static_Awakening_Filter_Proc;
	if filter != nil
	{
		core_filter = abi_static_awakening_filter;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.static_apply_filtered(&resource.world, abi_static_handle_to_core(handle), abi_static_description_to_core(description^), core_filter, &bridge), diagnostic, .None);
}
