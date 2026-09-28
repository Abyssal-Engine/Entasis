package entasis_c

import entasis "entasis:entasis"

ENTASIS_STRUCT_VERSION :: u32(1);
abi_status :: #force_inline proc "contextless" (status: entasis.Status) -> Entasis_Status
{
	return Entasis_Status(status);
}

abi_status_finish :: #force_inline proc "contextless" (
	status: entasis.Status,
	diagnostic: ^Entasis_Diagnostic,
	operation: entasis.Diagnostic_Operation,
	detail: i32 = 0,
) -> Entasis_Status
{
	result := abi_status(status);
	if status == .Ok
	{
		abi_diagnostic_clear(diagnostic);
	}
	else
	{
		abi_diagnostic_record(diagnostic, result, Entasis_Diagnostic_Operation(operation), detail);
	}
	return result;
}

abi_dispatch_status :: #force_inline proc "contextless" (
	status: entasis.Dispatcher_Status,
) -> Entasis_Dispatcher_Status
{
	return Entasis_Dispatcher_Status(status);
}

abi_vector3_to_core :: #force_inline proc "contextless" (value: Entasis_Vector3) -> entasis.Vector3
{
	return {value.x, value.y, value.z};
}

abi_vector3_from_core :: #force_inline proc "contextless" (value: entasis.Vector3) -> Entasis_Vector3
{
	return {value.x, value.y, value.z};
}

abi_spring_to_core :: #force_inline proc "contextless" (
	value: Entasis_Spring_Settings,
) -> entasis.Spring_Settings
{
	return {
		angular_frequency=value.angular_frequency,
		twice_damping_ratio=value.twice_damping_ratio,
	};
}

abi_spring_from_core :: #force_inline proc "contextless" (
	value: entasis.Spring_Settings,
) -> Entasis_Spring_Settings
{
	return {
		angular_frequency=value.angular_frequency,
		twice_damping_ratio=value.twice_damping_ratio,
	};
}

abi_material_to_core :: #force_inline proc "contextless" (
	value: Entasis_Contact_Material,
) -> entasis.Contact_Material
{
	return {
		friction_coefficient=value.friction_coefficient,
		spring_settings=abi_spring_to_core(value.spring_settings),
		maximum_recovery_velocity=value.maximum_recovery_velocity,
	};
}

abi_material_from_core :: #force_inline proc "contextless" (
	value: entasis.Contact_Material,
) -> Entasis_Contact_Material
{
	return {
		friction_coefficient=value.friction_coefficient,
		spring_settings=abi_spring_from_core(value.spring_settings),
		maximum_recovery_velocity=value.maximum_recovery_velocity,
	};
}

abi_capacity_to_core :: #force_inline proc "contextless" (
	value: Entasis_Capacity_Hints,
) -> entasis.Capacity_Hints
{
	return {
		bodies=value.bodies,
		statics=value.statics,
		inactive_body_sets=value.inactive_body_sets,
		shapes_per_type=value.shapes_per_type,
		constraints=value.constraints,
		initial_constraints_per_type_batch=value.initial_constraints_per_type_batch,
		minimum_constraints_per_body=value.minimum_constraints_per_body,
		broad_phase_candidates=value.broad_phase_candidates,
		pairs=value.pairs,
		collision_child_pairs=value.collision_child_pairs,
		inactive_pairs=value.inactive_pairs,
		pending_pairs_per_worker=value.pending_pairs_per_worker,
	};
}

abi_capacity_from_core :: #force_inline proc "contextless" (
	value: entasis.Capacity_Hints,
) -> Entasis_Capacity_Hints
{
	return {
		bodies=value.bodies,
		statics=value.statics,
		inactive_body_sets=value.inactive_body_sets,
		shapes_per_type=value.shapes_per_type,
		constraints=value.constraints,
		initial_constraints_per_type_batch=value.initial_constraints_per_type_batch,
		minimum_constraints_per_body=value.minimum_constraints_per_body,
		broad_phase_candidates=value.broad_phase_candidates,
		pairs=value.pairs,
		collision_child_pairs=value.collision_child_pairs,
		inactive_pairs=value.inactive_pairs,
		pending_pairs_per_worker=value.pending_pairs_per_worker,
	};
}

abi_shape_handle_to_core :: #force_inline proc "contextless" (value: Entasis_Shape_Handle) -> entasis.Shape_Handle
{
	return {packed=value.packed};
}

abi_shape_handle_from_core :: #force_inline proc "contextless" (value: entasis.Shape_Handle) -> Entasis_Shape_Handle
{
	return {packed=value.packed};
}

abi_body_handle_to_core :: #force_inline proc "contextless" (value: Entasis_Body_Handle) -> entasis.Body_Handle
{
	return {value=value.value};
}

abi_body_handle_from_core :: #force_inline proc "contextless" (value: entasis.Body_Handle) -> Entasis_Body_Handle
{
	return {value=value.value};
}

abi_static_handle_to_core :: #force_inline proc "contextless" (value: Entasis_Static_Handle) -> entasis.Static_Handle
{
	return {value=value.value};
}

abi_static_handle_from_core :: #force_inline proc "contextless" (value: entasis.Static_Handle) -> Entasis_Static_Handle
{
	return {value=value.value};
}

abi_pose_to_core :: #force_inline proc "contextless" (value: Entasis_Rigid_Pose) -> entasis.Rigid_Pose
{
	return transmute(entasis.Rigid_Pose)value;
}

abi_pose_from_core :: #force_inline proc "contextless" (value: entasis.Rigid_Pose) -> Entasis_Rigid_Pose
{
	return transmute(Entasis_Rigid_Pose)value;
}

abi_velocity_to_core :: #force_inline proc "contextless" (value: Entasis_Body_Velocity) -> entasis.Body_Velocity
{
	return transmute(entasis.Body_Velocity)value;
}

abi_velocity_from_core :: #force_inline proc "contextless" (value: entasis.Body_Velocity) -> Entasis_Body_Velocity
{
	return transmute(Entasis_Body_Velocity)value;
}

abi_inertia_to_core :: #force_inline proc "contextless" (value: Entasis_Body_Inertia) -> entasis.Body_Inertia
{
	return transmute(entasis.Body_Inertia)value;
}

abi_inertia_from_core :: #force_inline proc "contextless" (value: entasis.Body_Inertia) -> Entasis_Body_Inertia
{
	return transmute(Entasis_Body_Inertia)value;
}

abi_continuity_to_core :: #force_inline proc "contextless" (value: Entasis_Continuous_Detection) -> entasis.Continuous_Detection
{
	return transmute(entasis.Continuous_Detection)value;
}

abi_continuity_from_core :: #force_inline proc "contextless" (value: entasis.Continuous_Detection) -> Entasis_Continuous_Detection
{
	return transmute(Entasis_Continuous_Detection)value;
}

abi_activity_to_core :: #force_inline proc "contextless" (value: Entasis_Activity_Description) -> entasis.Activity_Description
{
	return transmute(entasis.Activity_Description)value;
}

abi_activity_from_core :: #force_inline proc "contextless" (value: entasis.Activity_Description) -> Entasis_Activity_Description
{
	return transmute(Entasis_Activity_Description)value;
}

abi_collidable_to_core :: #force_inline proc "contextless" (value: Entasis_Collidable_Description) -> entasis.Collidable_Description
{
	return transmute(entasis.Collidable_Description)value;
}

abi_collidable_from_core :: #force_inline proc "contextless" (value: entasis.Collidable_Description) -> Entasis_Collidable_Description
{
	return transmute(Entasis_Collidable_Description)value;
}

abi_body_description_to_core :: #force_inline proc "contextless" (value: Entasis_Body_Description) -> entasis.Body_Description
{
	return transmute(entasis.Body_Description)value;
}

abi_body_description_from_core :: #force_inline proc "contextless" (value: entasis.Body_Description) -> Entasis_Body_Description
{
	return transmute(Entasis_Body_Description)value;
}

abi_static_description_to_core :: #force_inline proc "contextless" (value: Entasis_Static_Description) -> entasis.Static_Description
{
	return transmute(entasis.Static_Description)value;
}

abi_static_description_from_core :: #force_inline proc "contextless" (value: entasis.Static_Description) -> Entasis_Static_Description
{
	return transmute(Entasis_Static_Description)value;
}

abi_ray_to_core :: #force_inline proc "contextless" (value: Entasis_Ray) -> entasis.Ray
{
	return transmute(entasis.Ray)value;
}

// acquire once at the public bridge boundary, never inside native kernels
abi_Access_Mode :: enum u8
{
	Exclusive,
	Read_Only,
}

abi_Execution_State :: enum u8
{
	Idle,
	Executing,
}

abi_Attachment_State :: enum u8
{
	Detached,
	Attached,
}

abi_Initialization_State :: enum u8
{
	Pending,
	Committed,
}

abi_Scope_State :: enum u8
{
	Inactive,
	Live,
}

abi_world_resource_for_domain :: #force_inline proc "contextless" (
	world: ^Entasis_World,
	diagnostic: ^Entasis_Diagnostic,
	operation: entasis.Diagnostic_Operation,
	access_mode: abi_Access_Mode = .Exclusive,
) -> (^abi_world_resource, Entasis_Status)
{
	resource := abi_world_get(world);
	if resource == nil
	{
		return nil, abi_status_finish(.Disposed, diagnostic, operation);
	}
	if resource.header.access == .Read_Phase && access_mode == .Read_Only
	{
		return resource, abi_status(.Ok);
	}
	if resource.header.access != .Ready
	{
		return nil, abi_status_finish(.Invalid_Argument, diagnostic, operation);
	}
	resource.header.access = .Exclusive;
	return resource, abi_status(.Ok);
}

abi_world_release :: #force_inline proc "contextless" (resource: ^abi_world_resource)
{
	if resource != nil && resource.header.access == .Exclusive
	{
		resource.header.access = .Ready;
	}
}

abi_count_valid :: #force_inline proc "contextless" (count: u64) -> bool
{
	return count <= u64(max(int));
}

abi_batch_begin :: #force_inline proc "contextless" (out_completed: ^u64)
{
	if out_completed != nil
	{
		out_completed^ = 0;
	}
}

// this C bridge is out of line. direct Odin users do not execute these wrappers.
// hot batch paths reinterpret verified POD arrays without allocating or copying
// per element

abi_constraint_handle_to_core :: #force_inline proc "contextless" (
	value: Entasis_Constraint_Handle,
) -> entasis.Constraint_Handle
{
	return {value=value.value};
}

abi_constraint_handle_from_core :: #force_inline proc "contextless" (
	value: entasis.Constraint_Handle,
) -> Entasis_Constraint_Handle
{
	return {value=value.value};
}

abi_bounds_from_core :: #force_inline proc "contextless" (
	value: entasis.Bounding_Box,
) -> Entasis_Bounding_Box
{
	return {
		min=abi_vector3_from_core(value.min),
		max=abi_vector3_from_core(value.max),
	};
}
