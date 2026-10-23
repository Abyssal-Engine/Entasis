#+vet !unused-imports

package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	#assert(size_of(Entasis_Global_Body_State) == size_of(entasis.Global_Body_State));
	#assert(align_of(Entasis_Global_Body_State) == align_of(entasis.Global_Body_State));
	#assert(offset_of(Entasis_Global_Body_State, pose) == offset_of(entasis.Global_Body_State, pose));
	#assert(offset_of(Entasis_Global_Body_State, velocity) == offset_of(entasis.Global_Body_State, velocity));
	#assert(offset_of(Entasis_Global_Body_State, local_inertia) == offset_of(entasis.Global_Body_State, local_inertia));
	#assert(offset_of(Entasis_Global_Body_State, collidable) == offset_of(entasis.Global_Body_State, collidable));
	#assert(offset_of(Entasis_Global_Body_State, activity) == offset_of(entasis.Global_Body_State, activity));
	#assert(size_of(Entasis_Global_Static_State) == size_of(entasis.Global_Static_State));
	#assert(align_of(Entasis_Global_Static_State) == align_of(entasis.Global_Static_State));
	#assert(offset_of(Entasis_Global_Static_State, pose) == offset_of(entasis.Global_Static_State, pose));
	#assert(offset_of(Entasis_Global_Static_State, shape) == offset_of(entasis.Global_Static_State, shape));
	#assert(offset_of(Entasis_Global_Static_State, continuity) == offset_of(entasis.Global_Static_State, continuity));
	#assert(size_of(Entasis_Global_Constraint_Target_State) == size_of(entasis.Global_Constraint_Target_State));
	#assert(align_of(Entasis_Global_Constraint_Target_State) == align_of(entasis.Global_Constraint_Target_State));
	#assert(offset_of(Entasis_Global_Constraint_Target_State, position) == offset_of(entasis.Global_Constraint_Target_State, position));
	#assert(offset_of(Entasis_Global_Constraint_Target_State, kind) == offset_of(entasis.Global_Constraint_Target_State, kind));
	#assert(size_of(Entasis_Global_Kinematic_Target_State) == size_of(entasis.Global_Kinematic_Target_State));
	#assert(align_of(Entasis_Global_Kinematic_Target_State) == align_of(entasis.Global_Kinematic_Target_State));
	#assert(offset_of(Entasis_Global_Kinematic_Target_State, pose) == offset_of(entasis.Global_Kinematic_Target_State, pose));
	#assert(offset_of(Entasis_Global_Kinematic_Target_State, presence) == offset_of(entasis.Global_Kinematic_Target_State, presence));
	abi_body_get_global_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		output: ^Entasis_Global_Body_State,
		diagnostic: ^Entasis_Diagnostic,
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
		value: entasis.Global_Body_State;
		status: entasis.Status;
		value, status = entasis.body_get_global_state(&resource.world, abi_body_handle_to_core(handle));
		if status == .Ok
		{
			output^ = transmute(Entasis_Global_Body_State)value;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_body_apply_global_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		state: ^Entasis_Global_Body_State,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if state == nil
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
		return abi_status_finish(entasis.body_apply_global_state(&resource.world, abi_body_handle_to_core(handle), transmute(entasis.Global_Body_State)state^), diagnostic, .None);
	}
	
	abi_static_get_global_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Static_Handle,
		output: ^Entasis_Global_Static_State,
		diagnostic: ^Entasis_Diagnostic,
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
		value: entasis.Global_Static_State;
		status: entasis.Status;
		value, status = entasis.static_get_global_state(&resource.world, abi_static_handle_to_core(handle));
		if status == .Ok
		{
			output^ = transmute(Entasis_Global_Static_State)value;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_apply_global_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Static_Handle,
		state: ^Entasis_Global_Static_State,
		awakening: Entasis_Awakening_Policy,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if state == nil
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
		return abi_status_finish(entasis.static_apply_global_state(&resource.world, abi_static_handle_to_core(handle), transmute(entasis.Global_Static_State)state^, wake), diagnostic, .None);
	}
	
	abi_constraint_get_global_target_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Constraint_Handle,
		output: ^Entasis_Global_Constraint_Target_State,
		diagnostic: ^Entasis_Diagnostic,
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
		value: entasis.Global_Constraint_Target_State;
		status: entasis.Status;
		value, status = entasis.constraint_get_global_target_state(&resource.world, abi_constraint_handle_to_core(handle));
		if status == .Ok
		{
			output^ = transmute(Entasis_Global_Constraint_Target_State)value;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_constraint_apply_global_target_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Constraint_Handle,
		state: ^Entasis_Global_Constraint_Target_State,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if state == nil
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
		return abi_status_finish(entasis.constraint_apply_global_target_state(&resource.world, abi_constraint_handle_to_core(handle), transmute(entasis.Global_Constraint_Target_State)state^), diagnostic, .None);
	}
	
	abi_body_get_global_target_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		output: ^Entasis_Global_Kinematic_Target_State,
		diagnostic: ^Entasis_Diagnostic,
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
		value: entasis.Global_Kinematic_Target_State;
		status: entasis.Status;
		value, status = entasis.body_get_global_target_state(&resource.world, abi_body_handle_to_core(handle));
		if status == .Ok
		{
			output^ = transmute(Entasis_Global_Kinematic_Target_State)value;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_body_apply_global_target_state :: proc "contextless" (
		world: ^Entasis_World,
		handle: Entasis_Body_Handle,
		state: ^Entasis_Global_Kinematic_Target_State,
		wake: Entasis_Body_Control_Wake,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if state == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if wake > 1
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
		return abi_status_finish(entasis.body_apply_global_target_state(&resource.world, abi_body_handle_to_core(handle), transmute(entasis.Global_Kinematic_Target_State)state^, entasis.Body_Control_Wake(wake)), diagnostic, .None);
	}
	
	abi_body_add_global_state :: proc "contextless" (
		world: ^Entasis_World,
		state: ^Entasis_Global_Body_State,
		output: ^Entasis_Body_Handle,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if output != nil
		{
			output^ = abi_body_handle_invalid();
		}
		if state == nil || output == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		status: entasis.Status;
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
		handle, status = entasis.body_add_global_state(&resource.world, transmute(entasis.Global_Body_State)state^);
		if status == .Ok
		{
			output^ = abi_body_handle_from_core(handle);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_static_add_global_state :: proc "contextless" (
		world: ^Entasis_World,
		state: ^Entasis_Global_Static_State,
		awakening: Entasis_Awakening_Policy,
		output: ^Entasis_Static_Handle,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if output != nil
		{
			output^ = abi_static_handle_invalid();
		}
		if state == nil || output == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		wake: entasis.Awakening_Policy;
		status: entasis.Status;
		wake, status = abi_awakening_to_core(awakening);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None);
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
		handle, status = entasis.static_add_global_state(&resource.world, transmute(entasis.Global_Static_State)state^, wake);
		if status == .Ok
		{
			output^ = abi_static_handle_from_core(handle);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
}
