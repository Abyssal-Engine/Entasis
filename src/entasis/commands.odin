package entasis

import physics "entasis:entasis_physics"

// Command_Kind identifies one facade structural operation. commands are applied
// in array order. completion order never defines public structure
Command_Kind :: enum u8
{
	Body_Add,
	Body_Apply,
	Body_Remove,
	Static_Add,
	Static_Apply,
	Static_Remove,
}

@(private)
body_add_command_data :: struct
{
	description: Body_Description,
}

@(private)
body_apply_command_data :: struct
{
	handle:      Body_Handle,
	description: Body_Description,
}

@(private)
body_remove_command_data :: struct
{
	handle: Body_Handle,
}

@(private)
static_add_command_data :: struct
{
	description: Static_Description,
	awakening:   Awakening_Policy,
}

@(private)
static_apply_command_data :: struct
{
	handle:      Static_Handle,
	description: Static_Description,
	awakening:   Awakening_Policy,
}

@(private)
static_remove_command_data :: struct
{
	handle:    Static_Handle,
	awakening: Awakening_Policy,
}

// Command is a caller-owned tagged structural command. construct it only with
// the command_* helpers so payload and result routing remain valid
Command :: struct
{
	kind:         Command_Kind,
	result_index: i32,
	using payload: struct #raw_union
	{
		body_add_data:      body_add_command_data,
		body_apply_data:    body_apply_command_data,
		body_remove_data:   body_remove_command_data,
		static_add_data:    static_add_command_data,
		static_apply_data:  static_apply_command_data,
		static_remove_data: static_remove_command_data,
	},
}

// Command_Buffer binds caller-owned commands, optional per-command statuses,
// and add-result arrays. only statuses through the successful prefix and the
// first failure entry are written
Command_Buffer :: struct
{
	commands:       []Command,
	statuses:       []Status,
	body_results:   []Body_Handle,
	static_results: []Static_Handle,
}

// command_buffer constructs a non-owning command-buffer view. no allocation is
// performed and every supplied slice must outlive world_apply_commands
command_buffer :: #force_inline proc "contextless" (
	commands: []Command,
	statuses: []Status = nil,
	body_results: []Body_Handle = nil,
	static_results: []Static_Handle = nil,
) -> Command_Buffer
{
	return {
		commands=commands,
		statuses=statuses,
		body_results=body_results,
		static_results=static_results,
	};
}

// command_body_add constructs a body-add command with caller-selected result slot
command_body_add :: #force_inline proc "contextless" (
	description: Body_Description,
	result_index: int,
) -> Command
{
	return {
		kind=.Body_Add,
		result_index=i32(result_index),
		body_add_data={description=description},
	};
}

// command_body_apply constructs a body-apply command
command_body_apply :: #force_inline proc "contextless" (
	handle: Body_Handle,
	description: Body_Description,
) -> Command
{
	return {
		kind=.Body_Apply,
		result_index=-1,
		body_apply_data={handle=handle, description=description},
	};
}

// command_body_remove constructs a body-remove command
command_body_remove :: #force_inline proc "contextless" (
	handle: Body_Handle,
) -> Command
{
	return {
		kind=.Body_Remove,
		result_index=-1,
		body_remove_data={handle=handle},
	};
}

// command_static_add constructs a static-add command with result slot and awakening
// policy
command_static_add :: #force_inline proc "contextless" (
	description: Static_Description,
	result_index: int,
	awakening: Awakening_Policy = .Overlaps,
) -> Command
{
	return {
		kind=.Static_Add,
		result_index=i32(result_index),
		static_add_data={description=description, awakening=awakening},
	};
}

// command_static_apply constructs a static-apply command with awakening policy
command_static_apply :: #force_inline proc "contextless" (
	handle: Static_Handle,
	description: Static_Description,
	awakening: Awakening_Policy = .Overlaps,
) -> Command
{
	return {
		kind=.Static_Apply,
		result_index=-1,
		static_apply_data={
			handle=handle,
			description=description,
			awakening=awakening,
		},
	};
}

// command_static_remove constructs a static-remove command with awakening policy
command_static_remove :: #force_inline proc "contextless" (
	handle: Static_Handle,
	awakening: Awakening_Policy = .Overlaps,
) -> Command
{
	return {
		kind=.Static_Remove,
		result_index=-1,
		static_remove_data={handle=handle, awakening=awakening},
	};
}

@(private)
command_status_write :: #force_inline proc "contextless" (
	buffer: ^Command_Buffer,
	index: int,
	status: Status,
)
{
	if len(buffer.statuses) > 0
	{
		buffer.statuses[index] = status;
	}
}

@(private)
command_results_reset :: #force_inline proc "contextless" (
	buffer: ^Command_Buffer,
)
{
	for index in 0 ..< len(buffer.body_results)
	{
		buffer.body_results[index] = body_handle_invalid();
	}
	for index in 0 ..< len(buffer.static_results)
	{
		buffer.static_results[index] = static_handle_invalid();
	}
}

@(private)
command_validate_and_count :: proc "contextless" (
	buffer: ^Command_Buffer,
) -> (requirements: batch_add_requirements, status: Status)
{
	if buffer == nil
	{
		return {}, .Invalid_Argument;
	}
	if len(buffer.statuses) > 0 && len(buffer.statuses) < len(buffer.commands)
	{
		return {}, .Invalid_Argument;
	}
	command_results_reset(buffer);
	claimed_body := Body_Handle{-2};
	claimed_static := Static_Handle{-2};
	for command_index in 0 ..< len(buffer.commands)
	{
		command := &buffer.commands[command_index];
		switch command.kind
		{
			case .Body_Add:
				if requirements.body_count == max(int)
				{
					command_results_reset(buffer);
					return {}, .Capacity_Missing;
				}
				result_index := int(command.result_index);
				if result_index < 0 || result_index >= len(buffer.body_results) ||
					buffer.body_results[result_index].value == claimed_body.value
				{
					command_results_reset(buffer);
					return {}, .Invalid_Argument;
				}
				buffer.body_results[result_index] = claimed_body;
				requirements.body_count += 1;
				if physics.typed_index_state(command.body_add_data.description.collidable.shape) == .Present
				{
					requirements.body_collidable_count += 1;
				}
			case .Body_Apply, .Body_Remove:
			case .Static_Add:
				if requirements.static_count == max(int)
				{
					command_results_reset(buffer);
					return {}, .Capacity_Missing;
				}
				if batch_validate_awakening(command.static_add_data.awakening) != .Ok
				{
					command_results_reset(buffer);
					return {}, .Invalid_Argument;
				}
				result_index := int(command.result_index);
				if result_index < 0 || result_index >= len(buffer.static_results) ||
					buffer.static_results[result_index].value == claimed_static.value
				{
					command_results_reset(buffer);
					return {}, .Invalid_Argument;
				}
				buffer.static_results[result_index] = claimed_static;
				requirements.static_count += 1;
			case .Static_Apply:
				if batch_validate_awakening(command.static_apply_data.awakening) != .Ok
				{
					command_results_reset(buffer);
					return {}, .Invalid_Argument;
				}
			case .Static_Remove:
				if batch_validate_awakening(command.static_remove_data.awakening) != .Ok
				{
					command_results_reset(buffer);
					return {}, .Invalid_Argument;
				}
			case:
				command_results_reset(buffer);
				return {}, .Invalid_Argument;
		}
	}
	command_results_reset(buffer);
	return requirements, .Ok;
}

// world_apply_commands applies caller-owned structural commands in exact input
// order. one combined capacity check covers all body and static additions.
// the operation commits a successful prefix and stops at the first failure.
// processed is the first failing command index. add commands write handles into
// caller-selected result slots. commands cannot reference handles created by
// another command in the same buffer
// allocation: the initial capacity check may grow world storage
// ownership: owner thread only while the world is idle
world_apply_commands :: proc (
	world: ^World,
	buffer: ^Command_Buffer,
) -> (processed: int, status: Status)
{
	requirements, validation_status := command_validate_and_count(buffer);
	if validation_status != .Ok
	{
		return 0, validation_status;
	}
	if len(buffer.commands) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	preflight_status := batch_preflight_adds(data, requirements);
	if preflight_status != .Ok
	{
		return 0, preflight_status;
	}
	for command_index in 0 ..< len(buffer.commands)
	{
		command := &buffer.commands[command_index];
		command_status := Status.Ok;
		switch command.kind
		{
			case .Body_Add:
				description := command.body_add_data.description;
				handle, add_status := physics.simulation_add_body(
					simulation, &description,
				);
				command_status = add_status;
				if add_status == .Ok
				{
					buffer.body_results[command.result_index] = handle;
				}
			case .Body_Apply:
				description := command.body_apply_data.description;
				command_status = physics.simulation_apply_body_description(
					simulation, command.body_apply_data.handle, &description,
				);
			case .Body_Remove:
				command_status = physics.simulation_remove_body(
					simulation, command.body_remove_data.handle,
				);
			case .Static_Add:
				description := command.static_add_data.description;
				handle, add_status := batch_add_static_low_level(
					simulation, &description, command.static_add_data.awakening,
				);
				command_status = add_status;
				if add_status == .Ok
				{
					buffer.static_results[command.result_index] = handle;
				}
			case .Static_Apply:
				description := command.static_apply_data.description;
				command_status = batch_apply_static_low_level(
					simulation,
					command.static_apply_data.handle,
					&description,
					command.static_apply_data.awakening,
				);
			case .Static_Remove:
				command_status = batch_remove_static_low_level(
					simulation,
					command.static_remove_data.handle,
					command.static_remove_data.awakening,
				);
			case:
				command_status = .Invalid_Argument;
		}
		command_status_write(buffer, command_index, command_status);
		if command_status != .Ok
		{
			return command_index, command_status;
		}
	}
	return len(buffer.commands), .Ok;
}
