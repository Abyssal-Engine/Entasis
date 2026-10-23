#+vet !unused-imports

package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	#assert(size_of(Entasis_Global_Pose) == size_of(entasis.Global_Pose));
	#assert(align_of(Entasis_Global_Pose) == align_of(entasis.Global_Pose));
	#assert(offset_of(Entasis_Global_Pose, orientation) == offset_of(entasis.Global_Pose, orientation));
	abi_body_add_batch_global :: #force_inline proc "contextless" (
		world: ^Entasis_World, descriptions: [^]Entasis_Body_Description,
		count: u64,
		positions: [^]Entasis_Global_Position, position_count: u64, out_handles: [^]Entasis_Body_Handle,
		out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		abi_batch_begin(out_completed);
		if position_count != count || !abi_count_valid(position_count) ||
			(position_count > 0 && positions == nil) || !abi_count_valid(count) || (count > 0 && (descriptions == nil || out_handles == nil))
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
		core_descriptions: []entasis.Body_Description = (cast([^]entasis.Body_Description)descriptions)[:int(count)];
		core_handles: []entasis.Body_Handle = (cast([^]entasis.Body_Handle)out_handles)[:int(count)];
		core_positions: []entasis.Global_Position = (cast([^]entasis.Global_Position)positions)[:int(count)];
		completed: int;
		status: entasis.Status;
		completed, status = entasis.body_add_batch_global(
			&resource.world, core_descriptions, core_handles, core_positions,
		);
		if out_completed != nil
		{
			out_completed^ = u64(completed);
		}
		return abi_status_finish(status, diagnostic, .None, i32(completed));
	}
	
	abi_body_apply_batch_global :: #force_inline proc "contextless" (
		world: ^Entasis_World, handles: [^]Entasis_Body_Handle,
		descriptions: [^]Entasis_Body_Description, count: u64,
		positions: [^]Entasis_Global_Position, position_count: u64,
		out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		abi_batch_begin(out_completed);
		if position_count != count || !abi_count_valid(position_count) ||
			(position_count > 0 && positions == nil) || !abi_count_valid(count) || (count > 0 && (handles == nil || descriptions == nil))
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
		core_handles: []entasis.Body_Handle = (cast([^]entasis.Body_Handle)handles)[:int(count)];
		core_descriptions: []entasis.Body_Description = (cast([^]entasis.Body_Description)descriptions)[:int(count)];
		core_positions: []entasis.Global_Position = (cast([^]entasis.Global_Position)positions)[:int(count)];
		completed: int;
		status: entasis.Status;
		completed, status = entasis.body_apply_batch_global(
			&resource.world, core_handles, core_descriptions, core_positions,
		);
		if out_completed != nil
		{
			out_completed^ = u64(completed);
		}
		return abi_status_finish(status, diagnostic, .None, i32(completed));
	}
	
	abi_static_add_batch_global :: #force_inline proc "contextless" (
		world: ^Entasis_World, descriptions: [^]Entasis_Static_Description,
		count: u64,
		positions: [^]Entasis_Global_Position, position_count: u64, awakening: Entasis_Awakening_Policy,
		out_handles: [^]Entasis_Static_Handle, out_completed: ^u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		abi_batch_begin(out_completed);
		if position_count != count || !abi_count_valid(position_count) ||
			(position_count > 0 && positions == nil) || !abi_count_valid(count) || (count > 0 && (descriptions == nil || out_handles == nil))
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
		core_descriptions: []entasis.Static_Description = (cast([^]entasis.Static_Description)descriptions)[:int(count)];
		core_handles: []entasis.Static_Handle = (cast([^]entasis.Static_Handle)out_handles)[:int(count)];
		core_positions: []entasis.Global_Position = (cast([^]entasis.Global_Position)positions)[:int(count)];
		completed: int;
		status: entasis.Status;
		completed, status = entasis.static_add_batch_global(
			&resource.world, core_descriptions, core_handles, core_positions, wake,
		);
		if out_completed != nil
		{
			out_completed^ = u64(completed);
		}
		return abi_status_finish(status, diagnostic, .None, i32(completed));
	}
	
	abi_static_apply_batch_global :: #force_inline proc "contextless" (
		world: ^Entasis_World, handles: [^]Entasis_Static_Handle,
		descriptions: [^]Entasis_Static_Description, count: u64,
		positions: [^]Entasis_Global_Position, position_count: u64,
		awakening: Entasis_Awakening_Policy, out_completed: ^u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		abi_batch_begin(out_completed);
		if position_count != count || !abi_count_valid(position_count) ||
			(position_count > 0 && positions == nil) || !abi_count_valid(count) || (count > 0 && (handles == nil || descriptions == nil))
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
		core_handles: []entasis.Static_Handle = (cast([^]entasis.Static_Handle)handles)[:int(count)];
		core_descriptions: []entasis.Static_Description = (cast([^]entasis.Static_Description)descriptions)[:int(count)];
		core_positions: []entasis.Global_Position = (cast([^]entasis.Global_Position)positions)[:int(count)];
		completed: int;
		status: entasis.Status;
		completed, status = entasis.static_apply_batch_global(
			&resource.world, core_handles, core_descriptions, core_positions, wake,
		);
		if out_completed != nil
		{
			out_completed^ = u64(completed);
		}
		return abi_status_finish(status, diagnostic, .None, i32(completed));
	}
	
	abi_world_apply_commands_global :: proc "contextless" (
		world: ^Entasis_World,
		buffer: ^Entasis_Command_Buffer,
		positions: [^]Entasis_Global_Position, position_count: u64,
		out_processed: ^u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_processed != nil
		{
			out_processed^ = 0;
		}
		if buffer == nil || out_processed == nil || position_count != buffer.command_count ||
			!abi_count_valid(position_count) || (position_count > 0 && positions == nil) ||
		!abi_count_valid(buffer.command_count) ||
		!abi_count_valid(buffer.status_capacity) ||
		!abi_count_valid(buffer.body_result_capacity) ||
		!abi_count_valid(buffer.static_result_capacity) ||
		(buffer.command_count > 0 && buffer.commands == nil) ||
		(buffer.status_capacity > 0 && buffer.statuses == nil) ||
		(buffer.body_result_capacity > 0 && buffer.body_results == nil) ||
		(buffer.static_result_capacity > 0 && buffer.static_results == nil)
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
		core_buffer: entasis.Command_Buffer = entasis.command_buffer(
			(cast([^]entasis.Command)buffer.commands)[:int(buffer.command_count)],
			(cast([^]entasis.Status)buffer.statuses)[:int(buffer.status_capacity)],
			(cast([^]entasis.Body_Handle)buffer.body_results)[:int(buffer.body_result_capacity)],
			(cast([^]entasis.Static_Handle)buffer.static_results)[:int(buffer.static_result_capacity)],
		);
		context = runtime.default_context();
		processed: int;
		status: entasis.Status;
		processed, status = entasis.world_apply_commands_global(&resource.world, &core_buffer,
			(cast([^]entasis.Global_Position)positions)[:int(position_count)]);
		out_processed^ = u64(max(processed, 0));
		return abi_status_finish(status, diagnostic, .None, i32(processed));
	}
	
	abi_world_export_global_poses :: proc "contextless" (
		world: ^Entasis_World, references: [^]Entasis_Collidable_Reference, count: u64,
		output: [^]Entasis_Global_Pose, capacity: u64, frame: ^Entasis_Origin_Frame,
		out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		abi_batch_begin(out_completed);
		if !abi_count_valid(count) || !abi_count_valid(capacity) || capacity < count ||
			(count > 0 && (references == nil || output == nil))
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
		selected_frame: entasis.Origin_Frame;
		if frame != nil
		{
			selected_frame = transmute(entasis.Origin_Frame)frame^;
		}
		completed: int;
		status: entasis.Status;
		completed, status = entasis.world_export_global_poses(&resource.world,
			(cast([^]entasis.Collidable_Reference)references)[:int(count)],
			(cast([^]entasis.Global_Pose)output)[:int(capacity)], selected_frame);
		if out_completed != nil
		{
			out_completed^ = u64(completed);
		}
		return abi_status_finish(status, diagnostic, .None, i32(completed));
	}
}
