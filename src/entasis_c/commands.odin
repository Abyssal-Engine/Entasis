package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_command_body_add :: #force_inline proc "contextless" (
	description: Entasis_Body_Description,
	result_index: i32,
) -> Entasis_Command
{
	return transmute(Entasis_Command)entasis.command_body_add(
		abi_body_description_to_core(description), int(result_index),
	);
}

abi_command_body_apply :: #force_inline proc "contextless" (
	handle: Entasis_Body_Handle,
	description: Entasis_Body_Description,
) -> Entasis_Command
{
	return transmute(Entasis_Command)entasis.command_body_apply(
		abi_body_handle_to_core(handle), abi_body_description_to_core(description),
	);
}

abi_command_body_remove :: #force_inline proc "contextless" (
	handle: Entasis_Body_Handle,
) -> Entasis_Command
{
	return transmute(Entasis_Command)entasis.command_body_remove(
		abi_body_handle_to_core(handle),
	);
}

abi_command_static_add :: #force_inline proc "contextless" (
	description: Entasis_Static_Description,
	result_index: i32,
	awakening: Entasis_Awakening_Policy,
) -> Entasis_Command
{
	wake, status := abi_awakening_to_core(awakening);
	if status != .Ok
	{
		return {kind=Entasis_Command_Kind(255), result_index=-1};
	}
	return transmute(Entasis_Command)entasis.command_static_add(
		abi_static_description_to_core(description), int(result_index), wake,
	);
}

abi_command_static_apply :: #force_inline proc "contextless" (
	handle: Entasis_Static_Handle,
	description: Entasis_Static_Description,
	awakening: Entasis_Awakening_Policy,
) -> Entasis_Command
{
	wake, status := abi_awakening_to_core(awakening);
	if status != .Ok
	{
		return {kind=Entasis_Command_Kind(255), result_index=-1};
	}
	return transmute(Entasis_Command)entasis.command_static_apply(
		abi_static_handle_to_core(handle), abi_static_description_to_core(description), wake,
	);
}

abi_command_static_remove :: #force_inline proc "contextless" (
	handle: Entasis_Static_Handle,
	awakening: Entasis_Awakening_Policy,
) -> Entasis_Command
{
	wake, status := abi_awakening_to_core(awakening);
	if status != .Ok
	{
		return {kind=Entasis_Command_Kind(255), result_index=-1};
	}
	return transmute(Entasis_Command)entasis.command_static_remove(
		abi_static_handle_to_core(handle), wake,
	);
}

abi_command_buffer :: #force_inline proc "contextless" (
	commands: [^]Entasis_Command,
	command_count: u64,
	statuses: [^]Entasis_Status,
	status_capacity: u64,
	body_results: [^]Entasis_Body_Handle,
	body_result_capacity: u64,
	static_results: [^]Entasis_Static_Handle,
	static_result_capacity: u64,
) -> Entasis_Command_Buffer
{
	return {
		commands=commands,
		command_count=command_count,
		statuses=statuses,
		status_capacity=status_capacity,
		body_results=body_results,
		body_result_capacity=body_result_capacity,
		static_results=static_results,
		static_result_capacity=static_result_capacity,
	};
}

abi_world_apply_commands :: proc "contextless" (
	world: ^Entasis_World,
	buffer: ^Entasis_Command_Buffer,
	out_processed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_processed != nil
	{
		out_processed^ = 0;
	}
	if buffer == nil || out_processed == nil ||
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
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	core_buffer := entasis.command_buffer(
		(cast([^]entasis.Command)buffer.commands)[:int(buffer.command_count)],
		(cast([^]entasis.Status)buffer.statuses)[:int(buffer.status_capacity)],
		(cast([^]entasis.Body_Handle)buffer.body_results)[:int(buffer.body_result_capacity)],
		(cast([^]entasis.Static_Handle)buffer.static_results)[:int(buffer.static_result_capacity)],
	);
	context = runtime.default_context();
	processed, status := entasis.world_apply_commands(&resource.world, &core_buffer);
	out_processed^ = u64(max(processed, 0));
	return abi_status_finish(status, diagnostic, .None, i32(processed));
}
#assert(size_of(Entasis_Command) == size_of(entasis.Command));
#assert(align_of(Entasis_Command) == align_of(entasis.Command));
#assert(size_of(Entasis_Command_Buffer) == size_of(entasis.Command_Buffer));
#assert(align_of(Entasis_Command_Buffer) == align_of(entasis.Command_Buffer));
