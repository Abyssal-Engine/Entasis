#+vet !unused-imports

package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_global_pose_to_core :: proc "contextless" (value: Entasis_Global_Pose) -> entasis.Global_Pose
	{
		return {position=cast(entasis.Global_Position)value.position, orientation=cast(entasis.Quaternion)value.orientation};
	}
	
	abi_global_pose_from_core :: proc "contextless" (value: entasis.Global_Pose) -> Entasis_Global_Pose
	{
		return {position=cast(Entasis_Global_Position)value.position, orientation=cast(Entasis_Quaternion)value.orientation};
	}
	
	
	abi_body_add_global :: proc "contextless" (
		world: ^Entasis_World,
		description: ^Entasis_Body_Description,
		position: ^Entasis_Global_Position,
		out_handle: ^Entasis_Body_Handle,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_handle != nil
		{
			out_handle^ = abi_body_handle_invalid();
		}
		if position == nil || description == nil || out_handle == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		handle: entasis.Body_Handle;
		status: entasis.Status;
		handle, status = entasis.body_add_global(
			&resource.world, abi_body_description_to_core(description^), cast(entasis.Global_Position)position^,
		);
		if status == .Ok
		{
			out_handle^ = abi_body_handle_from_core(handle);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_body_apply_global :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		description: ^Entasis_Body_Description,
		position: ^Entasis_Global_Position,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if position == nil || description == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.body_apply_global(
				&resource.world,
				abi_body_handle_to_core(handle),
				abi_body_description_to_core(description^), cast(entasis.Global_Position)position^,
			), diagnostic, .None);
	}
	
	abi_body_set_global_pose :: proc "contextless" (
		world: ^Entasis_World, handle: Entasis_Body_Handle,
		value: ^Entasis_Global_Pose, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if value == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.body_set_global_pose(
				&resource.world, abi_body_handle_to_core(handle), abi_global_pose_to_core(value^),
			), diagnostic, .None);
	}
	
	abi_body_get_global_pose :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		out_state: ^Entasis_Global_Pose,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_state == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_state^ = {};
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		state: entasis.Global_Pose;
		status: entasis.Status;
		state, status = entasis.body_get_global_pose(&resource.world, abi_body_handle_to_core(handle));
		if status == .Ok
		{
			out_state^ = abi_global_pose_from_core(state);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_add_global :: proc "contextless" (
		world: ^Entasis_World, description: ^Entasis_Static_Description,
		position: ^Entasis_Global_Position,
		awakening: Entasis_Awakening_Policy, out_handle: ^Entasis_Static_Handle,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_handle != nil
		{
			out_handle^ = abi_static_handle_invalid();
		}
		if position == nil || description == nil || out_handle == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		wake: entasis.Awakening_Policy;
		wake_status: entasis.Status;
		wake, wake_status = abi_awakening_to_core(awakening);
		if wake_status != .Ok
		{
			return abi_status_finish(wake_status, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		handle: entasis.Static_Handle;
		status: entasis.Status;
		handle, status = entasis.static_add_global(&resource.world, abi_static_description_to_core(description^), cast(entasis.Global_Position)position^, wake);
		if status == .Ok
		{
			out_handle^ = abi_static_handle_from_core(handle);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_apply_global :: proc "contextless" (
		world: ^Entasis_World, handle: Entasis_Static_Handle,
		description: ^Entasis_Static_Description,
		position: ^Entasis_Global_Position, awakening: Entasis_Awakening_Policy,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if position == nil || description == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		wake: entasis.Awakening_Policy;
		wake_status: entasis.Status;
		wake, wake_status = abi_awakening_to_core(awakening);
		if wake_status != .Ok
		{
			return abi_status_finish(wake_status, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.static_apply_global(
				&resource.world, abi_static_handle_to_core(handle), abi_static_description_to_core(description^), cast(entasis.Global_Position)position^, wake,
			), diagnostic, .None);
	}
	
	abi_static_set_global_pose :: proc "contextless" (
		world: ^Entasis_World, handle: Entasis_Static_Handle,
		value: ^Entasis_Global_Pose, awakening: Entasis_Awakening_Policy,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if value == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		wake: entasis.Awakening_Policy;
		wake_status: entasis.Status;
		wake, wake_status = abi_awakening_to_core(awakening);
		if wake_status != .Ok
		{
			return abi_status_finish(wake_status, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.static_set_global_pose(
				&resource.world, abi_static_handle_to_core(handle), abi_global_pose_to_core(value^), wake,
			), diagnostic, .None);
	}
	
	abi_static_get_global_pose :: proc "contextless" (
		world: ^Entasis_World, handle: Entasis_Static_Handle,
		out_state: ^Entasis_Global_Pose, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_state == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_state^ = {};
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		state: entasis.Global_Pose;
		status: entasis.Status;
		state, status = entasis.static_get_global_pose(&resource.world, abi_static_handle_to_core(handle));
		if status == .Ok
		{
			out_state^ = abi_global_pose_from_core(state);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_add_global_filtered :: proc "contextless" (
		world: ^Entasis_World, description: ^Entasis_Static_Description, position: ^Entasis_Global_Position,
		filter: Entasis_Static_Awakening_Filter_Proc, user_context: rawptr,
		out_handle: ^Entasis_Static_Handle, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_handle != nil
		{
			out_handle^ = abi_static_handle_invalid();
		}
		if position == nil || description == nil || out_handle == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		bridge: abi_static_awakening_bridge = abi_static_awakening_bridge{callback=filter, user_context=user_context};
		core_filter: entasis.Static_Awakening_Filter_Proc;
		if filter != nil
		{
			core_filter = abi_static_awakening_filter;
		}
		context = runtime.default_context();
		handle: entasis.Static_Handle;
		status: entasis.Status;
		handle, status = entasis.static_add_global_filtered(&resource.world, abi_static_description_to_core(description^), cast(entasis.Global_Position)position^, core_filter, &bridge);
		if status == .Ok
		{
			out_handle^ = abi_static_handle_from_core(handle);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_apply_global_filtered :: proc "contextless" (
		world: ^Entasis_World, handle: Entasis_Static_Handle, description: ^Entasis_Static_Description, position: ^Entasis_Global_Position,
		filter: Entasis_Static_Awakening_Filter_Proc, user_context: rawptr,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if position == nil || description == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		bridge: abi_static_awakening_bridge = abi_static_awakening_bridge{callback=filter, user_context=user_context};
		core_filter: entasis.Static_Awakening_Filter_Proc;
		if filter != nil
		{
			core_filter = abi_static_awakening_filter;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.static_apply_global_filtered(&resource.world, abi_static_handle_to_core(handle), abi_static_description_to_core(description^), cast(entasis.Global_Position)position^, core_filter, &bridge), diagnostic, .None);
	}
	
	abi_body_set_global_kinematic_target :: proc "contextless" (
		world: ^Entasis_World, body: Entasis_Body_Handle, target: ^Entasis_Global_Pose, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if target == nil || wake > 1
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		selected: entasis.Global_Pose = abi_global_pose_to_core(target^);
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.body_set_global_kinematic_target(&resource.world, abi_body_handle_to_core(body), selected, entasis.Body_Control_Wake(wake)), diagnostic, .None);
	}
	
	abi_body_get_global_kinematic_target :: proc "contextless" (
		world: ^Entasis_World, body: Entasis_Body_Handle, output: ^Entasis_Global_Pose, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if output == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		value: entasis.Global_Pose;
		status: entasis.Status;
		value, status = entasis.body_get_global_kinematic_target(&resource.world, abi_body_handle_to_core(body));
		if status == .Ok
		{
			output^ = abi_global_pose_from_core(value);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_body_add_force_at_global_position :: proc "contextless" (
		world: ^Entasis_World, body: Entasis_Body_Handle, force: Entasis_Vector3, position: ^Entasis_Global_Position, wake: Entasis_Body_Control_Wake, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if position == nil || wake > 1
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		resource: ^abi_world_resource;
		ready: Entasis_Status;
		resource, ready = abi_world_resource_for_domain(world, diagnostic, .None);
		defer abi_world_release(resource);
		if resource == nil
		{
			return ready;
		}
		context = runtime.default_context();
		return abi_status_finish(entasis.body_add_force_at_global_position(&resource.world, abi_body_handle_to_core(body), abi_vector3_to_core(force), cast(entasis.Global_Position)position^, entasis.Body_Control_Wake(wake)), diagnostic, .None);
	}
}
