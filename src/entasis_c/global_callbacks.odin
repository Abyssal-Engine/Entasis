#+vet !unused-imports

package entasis_c

import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_global_velocity_callbacks_validate :: proc "contextless" (value: ^Entasis_Global_Velocity_Callbacks) -> entasis.Status
	{
		if value == nil
		{
			return .Ok;
		}
		if value.struct_size < u32(size_of(Entasis_Global_Velocity_Callbacks)) || value.struct_version != ENTASIS_STRUCT_VERSION ||
		value.initialize == nil || value.prepare == nil || value.integrate == nil || value.dispose == nil ||
		value.angular_mode > 2 || value.allow_substeps_for_unconstrained > 1 || value.integrate_kinematic_velocity > 1
		{
			return .Invalid_Description;
		}
		return .Ok;
	}
	
	abi_global_velocity_initialize_bridge :: proc "contextless" (user_context: rawptr, simulation: ^physics.Simulation) -> entasis.Status
	{
		resource: ^abi_world_resource = (^abi_world_resource)(user_context);
		status: entasis.Status = entasis.Status(resource.global_velocity_callbacks.initialize(resource.global_velocity_callbacks.user_context, {opaque=&resource.header}));
		if status == .Ok
		{
			resource.pose_simulation = simulation;
		}
		return status;
	}
	
	abi_global_velocity_prepare_bridge :: proc "contextless" (user_context: rawptr, dt: f32) -> entasis.Status
	{
		resource: ^abi_world_resource = (^abi_world_resource)(user_context);
		return entasis.Status(resource.global_velocity_callbacks.prepare(resource.global_velocity_callbacks.user_context, dt));
	}
	
	abi_global_velocity_dispose_bridge :: proc "contextless" (user_context: rawptr)
	{
		resource: ^abi_world_resource = (^abi_world_resource)(user_context);
		resource.pose_simulation = nil;
		resource.global_velocity_callbacks.dispose(resource.global_velocity_callbacks.user_context);
	}
	
	abi_global_velocity_integrate_bridge :: proc "contextless" (
		user_context: rawptr, body_indices: util.I32x8, position: ^physics.Global_Position_Bundle,
		orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
		integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
	)
	{
		resource: ^abi_world_resource = (^abi_world_resource)(user_context);
		mask: util.I32x8 = integration_mask;
		
		orientation_value: util.Quaternion_Wide = orientation;
		inertia_value: physics.Body_Inertia_Wide = inertia;
		dt_value: util.F32x8 = dt;
		original: physics.Body_Velocity_Wide = velocity^;
		index_values: [8]i32 = transmute([8]i32)body_indices;
		mask_values: [8]i32 = transmute([8]i32)integration_mask;
		handles: [8]Entasis_Body_Handle;
		active: ^physics.Body_Set = &resource.pose_simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
		for lane in 0 ..< 8
		{
			handles[lane] = abi_body_handle_invalid();
			index: int = int(index_values[lane]);
			if mask_values[lane] != 0 && index >= 0 && index < active.count
			{
				handles[lane] = abi_body_handle_from_core(active.index_to_handle.memory[index]);
			}
		}
		view: Entasis_Global_Pose_Integration_View = {
			body_handles=raw_data(handles[:]),
			active_mask=cast(^Entasis_I32x8)&mask,
			position=cast([^]Entasis_Global_Position)position,
			orientation=cast(^Entasis_Quaternion_Wide)&orientation_value,
			inertia=cast(^Entasis_Body_Inertia_Wide)&inertia_value,
			dt=cast(^Entasis_F32x8)&dt_value,
			velocity=cast(^Entasis_Body_Velocity_Wide)velocity,
			worker_index=i32(worker_index),
		};
		resource.global_velocity_callbacks.integrate(resource.global_velocity_callbacks.user_context, &view);
		// the foreign writer cannot corrupt inactive lanes consumed by a later pass
		velocity.linear.x = util.wide_select_f32(integration_mask, velocity.linear.x, original.linear.x);
		velocity.linear.y = util.wide_select_f32(integration_mask, velocity.linear.y, original.linear.y);
		velocity.linear.z = util.wide_select_f32(integration_mask, velocity.linear.z, original.linear.z);
		velocity.angular.x = util.wide_select_f32(integration_mask, velocity.angular.x, original.angular.x);
		velocity.angular.y = util.wide_select_f32(integration_mask, velocity.angular.y, original.angular.y);
		velocity.angular.z = util.wide_select_f32(integration_mask, velocity.angular.z, original.angular.z);
	}
	
	abi_global_velocity_callbacks_bind :: proc "contextless" (
		resource: ^abi_world_resource, description: ^entasis.World_Description, value: ^Entasis_Global_Velocity_Callbacks,
	)
	{
		resource.global_velocity_callbacks = value^;
		description.pose_callbacks = {
			initialize=abi_global_velocity_initialize_bridge, prepare_for_integration=abi_global_velocity_prepare_bridge,
			integrate_velocity_global=abi_global_velocity_integrate_bridge, dispose=abi_global_velocity_dispose_bridge,
			angular_mode=physics.Angular_Integration_Mode(value.angular_mode),
			allow_substeps_for_unconstrained=physics.Velocity_Integration_State(value.allow_substeps_for_unconstrained),
			integrate_kinematic_velocity=physics.Velocity_Integration_State(value.integrate_kinematic_velocity), user_context=resource,
		};
	}
	
	abi_world_init_global :: proc "contextless" (
		world: ^Entasis_World, description: ^Entasis_World_Description, extensions: ^Entasis_World_Extensions,
		pool: ^Entasis_Buffer_Pool, callbacks: ^Entasis_Global_Velocity_Callbacks, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if callbacks == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
		}
		return abi_world_init_internal(world, description, pool, diagnostic, extensions, callbacks);
	}
}
