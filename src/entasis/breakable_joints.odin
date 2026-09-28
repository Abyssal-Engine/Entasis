package entasis

import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Joint_Break_Metric :: physics.Joint_Break_Metric;
Joint_Break_Metrics :: physics.Joint_Break_Metrics;
Joint_Break_Limits :: physics.Joint_Break_Limits;
Joint_Break_Event :: physics.Joint_Break_Event;
Joint_Reaction :: physics.Joint_Reaction;
Joint_Reaction_State :: physics.Joint_Reaction_State;
Joint_Reaction_Provider :: physics.Constraint_Reaction_Provider;
Joint_Reaction_Provider_Input :: physics.Constraint_Reaction_Provider_Input;
Joint_Impulse_Wrench :: physics.Constraint_Impulse_Wrench;
world_enable_joint_breaks :: proc(world:^World, watch_capacity:i32=64) -> Status
{
	data: ^world_data = world_data_get(world);
	if data==nil
	{
		return .Disposed;
	}
	return physics.joint_break_enable(&data.simulation, int(watch_capacity), data.allocator, data.allocation_scope);
}

@(private)
joint_break_owner :: proc "contextless" (world:^World) -> (^physics.Joint_Break_System, Status)
{
	data: ^world_data = world_data_get(world);
	if data==nil
	{
		return nil, .Disposed;
	}
	if data.simulation.state!=.Ready
	{
		return nil, .Invalid_Argument;
	}
	if data.simulation.solver.joint_breaks==nil
	{
		return nil, .Not_Found;
	}
	return data.simulation.solver.joint_breaks, .Ok;
}

world_disable_joint_breaks :: proc(world:^World) -> Status
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if owner.phase!=.Idle
	{
		return .Invalid_Argument;
	}
	owner.simulation.solver.joint_breaks=nil;
	physics.joint_break_release(owner);
	return .Ok;
}

joint_break_reserve :: proc(world:^World, watch_capacity:i32) -> Status
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return status;
	}
	owner.simulation.state = .Stepping;
	defer owner.simulation.state = .Ready;
	return physics.joint_break_reserve_storage(owner, int(watch_capacity));
}

// the native provider is copied. user data is borrowed until replaced/disabled.
// no changes are allowed while that type is watched
constraint_set_reaction_provider :: proc(world:^World, type_id:Constraint_Type_ID, provider:Joint_Reaction_Provider) -> Status
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return status;
	}
	id: i32 = i32(type_id);
	if owner.phase!=.Idle || id<physics.FIRST_CALLER_CONSTRAINT_TYPE_ID || id>=physics.CONSTRAINT_TYPE_ID_CAPACITY
	{
		return .Invalid_Argument;
	}
	record: ^physics.Constraint_Type_Record;
	lookup: Status;
	record, lookup = physics.constraint_type_registry_lookup(&owner.simulation.solver.registry, id);
	if lookup!=.Ok
	{
		return lookup;
	}
	if record.impulse_bundle_size%size_of(util.F32x8)!=0 || record.impulse_bundle_size/size_of(util.F32x8)>32
	{
		return .Invalid_Argument;
	}
	for index in 0..<owner.watches.count
	{
		if owner.watches.values.memory[index].type_id==id
		{
			return .Invalid_Argument;
		}
	}
	owner.providers[id-physics.FIRST_CALLER_CONSTRAINT_TYPE_ID]=provider;
	return .Ok;
}

constraint_set_break_limits :: proc(world:^World, handle:Constraint_Handle, limits:Joint_Break_Limits) -> Status
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return status;
	}
	return physics.joint_break_set(owner, handle, limits);
}

constraint_get_break_limits :: proc(world:^World, handle:Constraint_Handle) -> (Joint_Break_Limits, Status)
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return {}, status;
	}
	key: i32 = handle.value;
	watch: ^physics.Joint_Break_Watch;
	presence: util.Presence_Status;
	watch, presence = util.quick_dictionary_try_get(&owner.watches, &key);
	if presence!=.Present
	{
		return {}, .Not_Found;
	};
	return watch.limits, .Ok;
}

constraint_clear_break_limits :: proc(world:^World, handle:Constraint_Handle) -> Status
{
	return constraint_set_break_limits(world, handle, {});
}

constraint_reaction :: proc(world:^World, handle:Constraint_Handle) -> (Joint_Reaction, Status)
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return {}, status;
	}
	key: i32 = handle.value;
	watch: ^physics.Joint_Break_Watch;
	presence: util.Presence_Status;
	watch, presence = util.quick_dictionary_try_get(&owner.watches, &key);
	if presence!=.Present
	{
		return {}, .Not_Found;
	};
	return watch.committed, .Ok;
}

constraint_break_events_drain :: proc(world:^World, output:[]Joint_Break_Event) -> (written, required:int, status:Status)
{
	owner: ^physics.Joint_Break_System;
	resolve: Status;
	owner, resolve = joint_break_owner(world);
	if resolve!=.Ok
	{
		return 0, 0, resolve;
	}
	if owner.phase == .Commit_Pending
	{
		world_invalidate_views(world);
	}
	return physics.joint_break_drain(owner, output);
}

constraint_break_events_discard :: proc(world:^World) -> Status
{
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if owner.phase == .Commit_Pending
	{
		world_invalidate_views(world);
	}
	status=physics.joint_break_finish(owner);
	if status!=.Ok
	{
		return status;
	}
	owner.event_count=0;
	owner.phase=.Idle;
	return .Ok;
}

Joint_Break_Pending_Stream :: enum u8
{
	Parent,
	Part,
	Contact,
	Break,
}
Joint_Break_Pending_Streams :: bit_set[Joint_Break_Pending_Stream; u8];
Joint_Break_Update_Result :: struct
{
	completed_steps, events_written, part_events_written, contact_events_written, break_events_written: i32,
	required, part_required, contact_required, break_required: i32,
	alpha: f32,
	pending: Joint_Break_Pending_Streams,
}

// completes physics once, then delivers the selected streams before another step
// enabled parent/part subscriptions participate even when their output is empty
// consume returned prefixes once and retry backpressure with elapsed=0
joint_break_stepper_update :: proc (
	stepper: ^Fixed_Stepper, world: ^World, elapsed: f32,
	break_output: []Joint_Break_Event,
	output: []Trigger_Event = nil, part_output: []Collider_Part_Event = nil,
	tracker: ^Contact_Tracker = nil, contact_output: []Contact_Event = nil,
	dispatcher: ^Dispatcher = nil,
) -> (Joint_Break_Update_Result, Status)
{
	result: Joint_Break_Update_Result;
	owner: ^physics.Joint_Break_System;
	status: Status;
	owner, status = joint_break_owner(world);
	if status != .Ok
	{
		return result, status;
	}
	if stepper == nil || stepper.timestep <= 0 || stepper.maximum_steps == 0 ||
		math.is_nan(stepper.timestep) || math.is_inf(stepper.timestep, 0) || stepper.accumulator < 0 ||
		math.is_nan(stepper.accumulator) || math.is_inf(stepper.accumulator, 0) || elapsed < 0 ||
		math.is_nan(elapsed) || math.is_inf(elapsed, 0) ||
		len(output) > int(max(i32)) || len(part_output) > int(max(i32)) ||
		len(contact_output) > int(max(i32)) || len(break_output) > int(max(i32))
	{
		return result, .Invalid_Argument;
	}
	selected: Joint_Break_Pending_Streams = {.Break};
	triggers: ^physics.Trigger_System = owner.simulation.triggers;
	if triggers != nil
	{
		selected += {.Parent};
		if triggers.mixed != nil && triggers.mixed.configuration.event_subscription == .Enabled
		{
			selected += {.Part};
		}
	}
	if tracker != nil
	{
		if tracker.lifecycle != .Ready
		{
			return result, .Disposed;
		}
		if tracker.simulation != owner.simulation || tracker.last_step_index > owner.simulation.step_index ||
			owner.simulation.step_index-tracker.last_step_index > 1
		{
			return result, .Invalid_Argument;
		}
		selected += {.Contact};
	}
	if (.Parent not_in selected && len(output) != 0) ||
		(.Part not_in selected && len(part_output) != 0) ||
		(.Contact not_in selected && len(contact_output) != 0)
	{
		return result, .Invalid_Argument;
	}
	accumulator: f32 = stepper.accumulator+elapsed;
	if math.is_inf(accumulator, 0)
	{
		return result, .Invalid_Argument;
	}
	stepper.accumulator = accumulator;
	for
	{
		// allocation-failed publication belongs to already completed physics
		// finish it before observing contact storage or acknowledging any stream
		if owner.phase == .Commit_Pending
		{
			world_invalidate_views(world);
			status = physics.joint_break_finish(owner);
			if status != .Ok
			{
				result.pending += {.Break};
				if .Contact in selected && tracker.last_step_index != owner.simulation.step_index
				{
					result.pending += {.Contact};
				}
				break;
			}
		}
		if .Parent in selected && triggers.notification_status != .Ok
		{
			status = triggers.notification_status;
			result.pending += {.Parent};
			if .Contact in selected && tracker.last_step_index != owner.simulation.step_index
			{
				result.pending += {.Contact};
			}
			break;
		}
		if .Contact in selected && tracker.last_step_index != owner.simulation.step_index
		{
			written, required: int;
			written, required, status = contact_events_drain(tracker, contact_output[int(result.contact_events_written):]);
			if status != .Ok
			{
				result.contact_required = i32(required);
				result.pending += {.Contact};
				break;
			}
			result.contact_events_written += i32(written);
		}
		if .Parent in selected && triggers.event_count > len(output)-int(result.events_written)
		{
			result.pending += {.Parent};
		}
		if .Part in selected && triggers.mixed.event_count > len(part_output)-int(result.part_events_written)
		{
			result.pending += {.Part};
		}
		if owner.event_count > len(break_output)-int(result.break_events_written)
		{
			result.pending += {.Break};
		}
		if result.pending != {}
		{
			status = .Capacity_Missing;
			break;
		}
		written, required: int;
		if .Parent in selected
		{
			written, required, status = trigger_events_drain(world, output[int(result.events_written):]);
			if status != .Ok
			{
				result.required = i32(required);
				result.pending += {.Parent};
				break;
			}
			result.events_written += i32(written);
		}
		if .Part in selected
		{
			written, required, status = trigger_part_events_drain(world, part_output[int(result.part_events_written):]);
			if status != .Ok
			{
				result.part_required = i32(required);
				result.pending += {.Part};
				break;
			}
			result.part_events_written += i32(written);
		}
		written, required, status = constraint_break_events_drain(world, break_output[int(result.break_events_written):]);
		if status != .Ok
		{
			result.break_required = i32(required);
			result.pending += {.Break};
			break;
		}
		result.break_events_written += i32(written);
		if result.completed_steps >= i32(stepper.maximum_steps) || stepper.accumulator < stepper.timestep
		{
			break;
		}
		status = world_step(world, stepper.timestep, dispatcher);
		if status != .Ok
		{
			break;
		}
		stepper.accumulator = max(0, stepper.accumulator-stepper.timestep);
		result.completed_steps += 1;
	}
	// report every retained batch, including streams blocked by another stream
	if .Parent in selected && triggers.event_count != 0
	{
		result.required = i32(triggers.event_count);
		result.pending += {.Parent};
	}
	if .Part in selected && triggers.mixed.event_count != 0
	{
		result.part_required = i32(triggers.mixed.event_count);
		result.pending += {.Part};
	}
	if owner.event_count != 0
	{
		result.break_required = i32(owner.event_count);
		result.pending += {.Break};
	}
	result.alpha = clamp(stepper.accumulator/stepper.timestep, 0, 1);
	return result, status;
}
