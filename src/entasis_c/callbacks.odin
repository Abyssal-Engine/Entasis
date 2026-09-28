package entasis_c

import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

#assert(size_of(Entasis_F32x8) == size_of(util.F32x8));
#assert(align_of(Entasis_F32x8) == align_of(util.F32x8));
#assert(size_of(Entasis_I32x8) == size_of(util.I32x8));
#assert(align_of(Entasis_I32x8) == align_of(util.I32x8));
#assert(size_of(Entasis_Vector3_Wide) == size_of(util.Vector3_Wide));
#assert(align_of(Entasis_Vector3_Wide) == align_of(util.Vector3_Wide));
#assert(offset_of(Entasis_Vector3_Wide, y) == offset_of(util.Vector3_Wide, y));
#assert(offset_of(Entasis_Vector3_Wide, z) == offset_of(util.Vector3_Wide, z));
#assert(size_of(Entasis_Quaternion_Wide) == size_of(util.Quaternion_Wide));
#assert(align_of(Entasis_Quaternion_Wide) == align_of(util.Quaternion_Wide));
#assert(offset_of(Entasis_Quaternion_Wide, w) == offset_of(util.Quaternion_Wide, w));
#assert(size_of(Entasis_Symmetric3x3_Wide) == size_of(util.Symmetric3x3_Wide));
#assert(align_of(Entasis_Symmetric3x3_Wide) == align_of(util.Symmetric3x3_Wide));
#assert(offset_of(Entasis_Symmetric3x3_Wide, yx) == offset_of(util.Symmetric3x3_Wide, yx));
#assert(offset_of(Entasis_Symmetric3x3_Wide, yy) == offset_of(util.Symmetric3x3_Wide, yy));
#assert(offset_of(Entasis_Symmetric3x3_Wide, zx) == offset_of(util.Symmetric3x3_Wide, zx));
#assert(offset_of(Entasis_Symmetric3x3_Wide, zy) == offset_of(util.Symmetric3x3_Wide, zy));
#assert(offset_of(Entasis_Symmetric3x3_Wide, zz) == offset_of(util.Symmetric3x3_Wide, zz));
#assert(size_of(Entasis_Body_Inertia_Wide) == size_of(physics.Body_Inertia_Wide));
#assert(align_of(Entasis_Body_Inertia_Wide) == align_of(physics.Body_Inertia_Wide));
#assert(offset_of(Entasis_Body_Inertia_Wide, inverse_mass) == offset_of(physics.Body_Inertia_Wide, inverse_mass));
#assert(size_of(Entasis_Body_Velocity_Wide) == size_of(physics.Body_Velocity_Wide));
#assert(align_of(Entasis_Body_Velocity_Wide) == align_of(physics.Body_Velocity_Wide));
#assert(offset_of(Entasis_Body_Velocity_Wide, angular) == offset_of(physics.Body_Velocity_Wide, angular));
#assert(size_of(Entasis_Convex_Contact_Manifold) == size_of(physics.Convex_Contact_Manifold));
#assert(align_of(Entasis_Convex_Contact_Manifold) == align_of(physics.Convex_Contact_Manifold));
#assert(offset_of(Entasis_Convex_Contact_Manifold, count) == offset_of(physics.Convex_Contact_Manifold, count));
#assert(offset_of(Entasis_Convex_Contact_Manifold, normal) == offset_of(physics.Convex_Contact_Manifold, normal));
#assert(offset_of(Entasis_Convex_Contact_Manifold, contacts) == offset_of(physics.Convex_Contact_Manifold, contacts));
#assert(offset_of(Entasis_Contact_Manifold, kind) == offset_of(physics.Manifold_Result, kind));
#assert(offset_of(Entasis_Contact_Manifold, convex) == offset_of(physics.Manifold_Result, convex));
#assert(offset_of(Entasis_Contact_Manifold, nonconvex) == offset_of(physics.Manifold_Result, nonconvex));
#assert(offset_of(Entasis_Vector3_Wide, x) == offset_of(util.Vector3_Wide, x));
#assert(offset_of(Entasis_Quaternion_Wide, x) == offset_of(util.Quaternion_Wide, x));
#assert(offset_of(Entasis_Quaternion_Wide, y) == offset_of(util.Quaternion_Wide, y));
#assert(offset_of(Entasis_Quaternion_Wide, z) == offset_of(util.Quaternion_Wide, z));
#assert(offset_of(Entasis_Symmetric3x3_Wide, xx) == offset_of(util.Symmetric3x3_Wide, xx));
#assert(offset_of(Entasis_Body_Inertia_Wide, inverse_inertia_tensor) == offset_of(physics.Body_Inertia_Wide, inverse_inertia_tensor));
#assert(offset_of(Entasis_Body_Velocity_Wide, linear) == offset_of(physics.Body_Velocity_Wide, linear));
abi_velocity_callbacks_validate :: proc "contextless" (value: ^Entasis_Velocity_Callbacks) -> entasis.Status
{
	if value == nil
	{
		return .Ok;
	}
	if value.struct_size < u32(size_of(Entasis_Velocity_Callbacks)) || value.struct_version != ENTASIS_STRUCT_VERSION ||
	value.initialize == nil || value.prepare == nil || value.integrate == nil || value.dispose == nil ||
	value.angular_mode > 2 || value.allow_substeps_for_unconstrained > 1 || value.integrate_kinematic_velocity > 1
	{
		return .Invalid_Description;
	}
	return .Ok;
}

abi_contact_callbacks_validate :: proc "contextless" (value: ^Entasis_Contact_Callbacks) -> entasis.Status
{
	if value == nil
	{
		return .Ok;
	}
	if value.struct_size < u32(size_of(Entasis_Contact_Callbacks)) || value.struct_version != ENTASIS_STRUCT_VERSION ||
	value.initialize == nil || value.allow == nil || value.allow_child == nil || value.configure == nil ||
	value.configure_child == nil || value.dispose == nil
	{
		return .Invalid_Description;
	}
	return .Ok;
}

abi_velocity_initialize_bridge :: proc "contextless" (user_context: rawptr, simulation: ^physics.Simulation) -> entasis.Status
{
	resource := (^abi_world_resource)(user_context);
	status := entasis.Status(resource.velocity_callbacks.initialize(resource.velocity_callbacks.user_context, {opaque=&resource.header}));
	if status == .Ok
	{
		resource.pose_simulation = simulation;
	}
	return status;
}

abi_velocity_prepare_bridge :: proc "contextless" (user_context: rawptr, dt: f32) -> entasis.Status
{
	resource := (^abi_world_resource)(user_context);
	return entasis.Status(resource.velocity_callbacks.prepare(resource.velocity_callbacks.user_context, dt));
}

abi_velocity_dispose_bridge :: proc "contextless" (user_context: rawptr)
{
	resource := (^abi_world_resource)(user_context);
	resource.pose_simulation = nil;
	resource.velocity_callbacks.dispose(resource.velocity_callbacks.user_context);
}

abi_velocity_integrate_bridge :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8, position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	resource := (^abi_world_resource)(user_context);
	mask := integration_mask;
	position_value := position;
	orientation_value := orientation;
	inertia_value := inertia;
	dt_value := dt;
	original := velocity^;
	index_values := transmute([8]i32)body_indices;
	mask_values := transmute([8]i32)integration_mask;
	handles: [8]Entasis_Body_Handle;
	active := &resource.pose_simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< 8
	{
		handles[lane] = abi_body_handle_invalid();
		index := int(index_values[lane]);
		if mask_values[lane] != 0 && index >= 0 && index < active.count
		{
			handles[lane] = abi_body_handle_from_core(active.index_to_handle.memory[index]);
		}
	}
	view := Entasis_Pose_Integration_View{
		body_handles=raw_data(handles[:]),
		active_mask=cast(^Entasis_I32x8)&mask,
		position=cast(^Entasis_Vector3_Wide)&position_value,
		orientation=cast(^Entasis_Quaternion_Wide)&orientation_value,
		inertia=cast(^Entasis_Body_Inertia_Wide)&inertia_value,
		dt=cast(^Entasis_F32x8)&dt_value,
		velocity=cast(^Entasis_Body_Velocity_Wide)velocity,
		worker_index=i32(worker_index),
	};
	resource.velocity_callbacks.integrate(resource.velocity_callbacks.user_context, &view);
	// the foreign writer cannot corrupt inactive lanes consumed by a later pass
	velocity.linear.x = util.wide_select_f32(integration_mask, velocity.linear.x, original.linear.x);
	velocity.linear.y = util.wide_select_f32(integration_mask, velocity.linear.y, original.linear.y);
	velocity.linear.z = util.wide_select_f32(integration_mask, velocity.linear.z, original.linear.z);
	velocity.angular.x = util.wide_select_f32(integration_mask, velocity.angular.x, original.angular.x);
	velocity.angular.y = util.wide_select_f32(integration_mask, velocity.angular.y, original.angular.y);
	velocity.angular.z = util.wide_select_f32(integration_mask, velocity.angular.z, original.angular.z);
}

abi_contact_initialize_bridge :: proc "contextless" (user_context: rawptr, simulation: ^physics.Simulation) -> entasis.Status
{
	_ = simulation;
	resource := (^abi_world_resource)(user_context);
	return entasis.Status(resource.contact_callbacks.initialize(resource.contact_callbacks.user_context, {opaque=&resource.header}));
}

abi_contact_dispose_bridge :: proc "contextless" (user_context: rawptr)
{
	resource := (^abi_world_resource)(user_context);
	resource.contact_callbacks.dispose(resource.contact_callbacks.user_context);
}

abi_contact_allow_bridge :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: entasis.Collidable_Reference, margin: ^f32,
) -> physics.Collision_Testing_State
{
	resource := (^abi_world_resource)(user_context);
	if resource.contact_callbacks.allow(resource.contact_callbacks.user_context, i32(worker_index), Entasis_Collidable_Reference(a), Entasis_Collidable_Reference(b), margin) != 0
	{
		return .Allow;
	}
	return .Reject;
}

abi_contact_allow_child_bridge :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: entasis.Collidable_Reference, child_a, child_b: int,
) -> physics.Collision_Testing_State
{
	resource := (^abi_world_resource)(user_context);
	if resource.contact_callbacks.allow_child(resource.contact_callbacks.user_context, i32(worker_index), Entasis_Collidable_Reference(a), Entasis_Collidable_Reference(b), i32(child_a), i32(child_b)) != 0
	{
		return .Allow;
	}
	return .Reject;
}

abi_contact_configure_bridge :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: entasis.Collidable_Reference,
	manifold: ^physics.Manifold_Result, material: ^physics.Contact_Material_Properties,
) -> physics.Collision_Testing_State
{
	resource := (^abi_world_resource)(user_context);
	c_material := abi_material_from_core(material^);
	accepted := resource.contact_callbacks.configure(resource.contact_callbacks.user_context,
		i32(worker_index), Entasis_Collidable_Reference(a), Entasis_Collidable_Reference(b),
		cast(^Entasis_Contact_Manifold)manifold, &c_material);
	material^ = abi_material_to_core(c_material);
	if accepted != 0
	{
		return .Allow;
	}
	return .Reject;
}

abi_contact_configure_child_bridge :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: entasis.Collidable_Reference,
	child_a, child_b: int, manifold: ^physics.Convex_Contact_Manifold,
) -> physics.Collision_Testing_State
{
	resource := (^abi_world_resource)(user_context);
	if resource.contact_callbacks.configure_child(resource.contact_callbacks.user_context,
		i32(worker_index), Entasis_Collidable_Reference(a), Entasis_Collidable_Reference(b),
		i32(child_a), i32(child_b), cast(^Entasis_Convex_Contact_Manifold)manifold) != 0
	{
		return .Allow;
	}
	return .Reject;
}

abi_contact_select_constraint_bridge :: proc "contextless" (
	user_context: rawptr, a, b: entasis.Collidable_Reference, manifold: ^physics.Manifold_Result, default_type_id: i32,
) -> i32
{
	resource := (^abi_world_resource)(user_context);
	return resource.contact_callbacks.select_constraint(resource.contact_callbacks.user_context,
		Entasis_Collidable_Reference(a), Entasis_Collidable_Reference(b), cast(^Entasis_Contact_Manifold)manifold, default_type_id);
}

abi_extension_callbacks_bind :: proc "contextless" (
	resource: ^abi_world_resource, description: ^entasis.World_Description, extensions: ^Entasis_World_Extensions,
)
{
	if extensions.velocity_callbacks != nil
	{
		resource.velocity_callbacks = extensions.velocity_callbacks^;
		value := &resource.velocity_callbacks;
		description.pose_callbacks = {
			initialize=abi_velocity_initialize_bridge, prepare_for_integration=abi_velocity_prepare_bridge,
			integrate_velocity=abi_velocity_integrate_bridge, dispose=abi_velocity_dispose_bridge,
			angular_mode=physics.Angular_Integration_Mode(value.angular_mode),
			allow_substeps_for_unconstrained=physics.Velocity_Integration_State(value.allow_substeps_for_unconstrained),
			integrate_kinematic_velocity=physics.Velocity_Integration_State(value.integrate_kinematic_velocity), user_context=resource,
		};
	}
	if extensions.contact_callbacks != nil
	{
		resource.contact_callbacks = extensions.contact_callbacks^;
		description.narrow_callbacks = {
			initialize=abi_contact_initialize_bridge, allow=abi_contact_allow_bridge,
			allow_child=abi_contact_allow_child_bridge, configure=abi_contact_configure_bridge,
			configure_child=abi_contact_configure_child_bridge, dispose=abi_contact_dispose_bridge, user_context=resource,
		};
		if resource.contact_callbacks.select_constraint != nil
		{
			description.narrow_callbacks.select_constraint = abi_contact_select_constraint_bridge;
		}
	}
}
