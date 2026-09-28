package entasis

import "core:math"
import physics "entasis:entasis_physics"

Collider_Part_Role :: physics.Collider_Part_Role;
Collider_Part_Option :: physics.Collider_Part_Option;
Collider_Part_Options :: physics.Collider_Part_Options;
Collider_Part_Settings :: physics.Collider_Part_Settings;
Collider_Part_Configuration :: physics.Collider_Part_Configuration;
Collider_Part_Event_Subscription :: physics.Collider_Part_Event_Subscription;
Collider_Part_Identity :: physics.Collider_Part_Identity;
Collider_Part_Info :: physics.Collider_Part_Info;
Collider_Part_Endpoint :: physics.Collider_Part_Endpoint;
Collider_Part_Pair :: physics.Collider_Part_Pair;
Collider_Part_Event :: physics.Collider_Part_Event;
collider_part_configuration_default :: proc "contextless" () -> Collider_Part_Configuration
{
	return {instance_capacity=128, part_capacity=256, pair_capacity=256,
		observations_per_worker=512, event_subscription=.Disabled};
}

// trigger storage must be enabled first. event subscription is fixed for the
// binding lifetime. reservation grows capacity without rebuilding history
collider_parts_reserve :: proc(world:^World, configuration:Collider_Part_Configuration) -> Status
{
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if owner.mixed==nil
	{
		return physics.mixed_colliders_initialize(owner, configuration);
	}
	return physics.mixed_colliders_reserve(owner.mixed, configuration);
}

collider_part_set :: proc(world:^World, reference:Collidable_Reference, child_index:i32, settings:Collider_Part_Settings) -> (Collider_Part_Info, Status)
{
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return {}, status;
	}
	if owner.mixed==nil
	{
		return {}, .Not_Found;
	}
	return physics.mixed_colliders_part_set(owner.mixed, reference, child_index, settings);
}

collider_part_get :: proc(world:^World, reference:Collidable_Reference, child_index:i32) -> (Collider_Part_Info, Status)
{
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return {}, status;
	}
	if owner.mixed==nil
	{
		return {}, .Not_Found;
	}
	return physics.mixed_colliders_part_get(owner.mixed, reference, child_index);
}

collider_part_remove :: proc(world:^World, reference:Collidable_Reference, child_index:i32) -> Status
{
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if owner.mixed==nil
	{
		return .Not_Found;
	}
	return physics.mixed_colliders_part_remove(owner.mixed, reference, child_index);
}

// returns configured overrides, ordered by top-level child index. unspecified
// children are solid and do not acquire a record merely because they are read
collider_parts :: proc(world:^World, reference:Collidable_Reference, output:[]Collider_Part_Info) -> (written, required:int, status:Status)
{
	owner: ^physics.Trigger_System;
	st: Status;
	owner, st = trigger_owner(world);
	if st!=.Ok
	{
		return 0, 0, st;
	}
	if owner.mixed==nil
	{
		return 0, 0, .Not_Found;
	}
	target: physics.Shape_Query_Target;
	resolve: Status;
	target, resolve = physics.simulation_query_target(owner.simulation, reference);
	if resolve!=.Ok
	{
		return 0, 0, resolve;
	}
	count: i32;
	cs: Status;
	count, cs = physics.collider_part_count(physics.simulation_shape_registry(owner.simulation), target.shape);
	if cs!=.Ok
	{
		return 0, 0, cs;
	}
	for index:i32=0;index<count;index+=1
	{
		lookup: Status;
		_, lookup = physics.mixed_colliders_part_get(owner.mixed, reference, index);
		if lookup==.Ok
		{
			required+=1;
		}
		else if lookup!=.Not_Found
		{
			return 0, 0, lookup;
		}
	}
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	for index:i32=0;index<count;index+=1
	{
		info: Collider_Part_Info;
		lookup: Status;
		info, lookup = physics.mixed_colliders_part_get(owner.mixed, reference, index);
		if lookup==.Ok
		{
			output[written]=info;
			written+=1;
		}
	}
	return written, required, .Ok;
}

trigger_part_events_drain :: proc "contextless" (world:^World, output:[]Collider_Part_Event) -> (written, required:int, status:Status)
{
	owner: ^physics.Trigger_System;
	st: Status;
	owner, st = trigger_owner(world);
	if st!=.Ok
	{
		return 0, 0, st;
	}
	if owner.notification_status!=.Ok
	{
		return 0, 0, owner.notification_status;
	}
	if owner.mixed==nil || owner.mixed.configuration.event_subscription!=.Enabled
	{
		return 0, 0, .Not_Found;
	}
	required=owner.mixed.event_count;
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	copy(output, owner.mixed.events.memory[:required]);
	owner.mixed.event_count=0;
	return required, required, .Ok;
}

trigger_part_events_discard :: proc "contextless" (world:^World) -> Status
{
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if owner.notification_status!=.Ok
	{
		return owner.notification_status;
	}
	if owner.mixed==nil || owner.mixed.configuration.event_subscription!=.Enabled
	{
		return .Not_Found;
	}
	owner.mixed.event_count=0;
	return .Ok;
}

trigger_part_overlaps :: proc "contextless" (world:^World, output:[]Collider_Part_Pair) -> (written, required:int, status:Status)
{
	owner: ^physics.Trigger_System;
	st: Status;
	owner, st = trigger_owner(world);
	if st!=.Ok
	{
		return 0, 0, st;
	}
	if owner.mixed==nil || owner.mixed.configuration.event_subscription!=.Enabled
	{
		return 0, 0, .Not_Found;
	}
	required=owner.mixed.previous_count;
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	copy(output, owner.mixed.previous.memory[:required]);
	return required, required, .Ok;
}

Collider_Part_Pending_Stream :: enum u8
{
	Parent,
	Part,
	Contact,
}

Collider_Part_Pending_Streams :: bit_set[Collider_Part_Pending_Stream; u8];
Collider_Part_Update_Result :: struct
{
	completed_steps, events_written, part_events_written, required, part_required: i32,
	alpha: f32,
	pending: Collider_Part_Pending_Streams,
}

// additive result: the existing two-stream ABI layout is unchanged. a contact
// history-capacity failure reports the existing tracker's required pair capacity
Collider_Part_Contact_Update_Result :: struct
{
	completed_steps, events_written, part_events_written, contact_events_written: i32,
	required, part_required, contact_required: i32,
	alpha: f32,
	pending: Collider_Part_Pending_Streams,
}

// both streams are drained before further stepping. backpressure consumes the
// successful physics step exactly once. a retry supplies elapsed=0
collider_part_stepper_update :: proc(
	stepper:^Fixed_Stepper, world:^World, elapsed:f32,
	output:[]Trigger_Event, part_output:[]Collider_Part_Event, dispatcher:^Dispatcher=nil,
) -> (Collider_Part_Update_Result, Status)
{
	result:Collider_Part_Update_Result;
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status!=.Ok
	{
		return result, status;
	}
	if owner.mixed==nil || owner.mixed.configuration.event_subscription!=.Enabled
	{
		return result, .Not_Found;
	}
	if stepper==nil || stepper.timestep<=0 || stepper.maximum_steps==0 ||
	math.is_nan(stepper.timestep) || math.is_inf(stepper.timestep, 0) || stepper.accumulator<0 ||
	math.is_nan(stepper.accumulator) || math.is_inf(stepper.accumulator, 0) || elapsed<0 ||
	math.is_nan(elapsed) || math.is_inf(elapsed, 0)
	{
		return result, .Invalid_Argument;
	}
	accumulator:f32=stepper.accumulator+elapsed;
	if math.is_inf(accumulator, 0)
	{
		return result, .Invalid_Argument;
	}
	stepper.accumulator=accumulator;
	for
	{
		if owner.notification_status!=.Ok
		{
			status=owner.notification_status;
			break;
		}
		if owner.event_count>len(output)-int(result.events_written)
		{
			result.required=i32(owner.event_count);
			result.pending+= {.Parent};
		}
		if owner.mixed.event_count>len(part_output)-int(result.part_events_written)
		{
			result.part_required=i32(owner.mixed.event_count);
			result.pending+= {.Part};
		}
		if result.pending!={}
		{
			status=.Capacity_Missing;
			break;
		}
		count: int;
		drain: Status;
		count, _, drain = trigger_events_drain(world, output[int(result.events_written):]);
		if drain!=.Ok
		{
			status=drain;
			break;
		}
		result.events_written+=i32(count);
		count, _, drain=trigger_part_events_drain(world, part_output[int(result.part_events_written):]);
		if drain!=.Ok
		{
			status=drain;
			break;
		}
		result.part_events_written+=i32(count);
		if result.completed_steps>=i32(stepper.maximum_steps)||stepper.accumulator<stepper.timestep
		{
			break;
		}
		status=world_step(world, stepper.timestep, dispatcher);
		if status!=.Ok
		{
			break;
		}
		stepper.accumulator=max(0, stepper.accumulator-stepper.timestep);
		result.completed_steps+=1;
	}
	result.alpha=clamp(stepper.accumulator/stepper.timestep, 0, 1);
	return result, status;
}

// explicitly selects contact delivery without routing existing two-stream
// consumers through a wider result or adding tracker checks to their path.
// consume returned prefixes once and retry elapsed=0 after backpressure
collider_part_stepper_update_with_contacts :: proc (
	stepper: ^Fixed_Stepper, world: ^World, elapsed: f32,
	output: []Trigger_Event, part_output: []Collider_Part_Event,
	tracker: ^Contact_Tracker, contact_output: []Contact_Event,
	dispatcher: ^Dispatcher = nil,
) -> (Collider_Part_Contact_Update_Result, Status)
{
	result: Collider_Part_Contact_Update_Result;
	owner: ^physics.Trigger_System;
	status: Status;
	owner, status = trigger_owner(world);
	if status != .Ok
	{
		return result, status;
	}
	if owner.mixed == nil || owner.mixed.configuration.event_subscription != .Enabled
	{
		return result, .Not_Found;
	}
	if stepper == nil || stepper.timestep <= 0 || stepper.maximum_steps == 0 ||
	math.is_nan(stepper.timestep) || math.is_inf(stepper.timestep, 0) || stepper.accumulator < 0 ||
	math.is_nan(stepper.accumulator) || math.is_inf(stepper.accumulator, 0) || elapsed < 0 ||
	math.is_nan(elapsed) || math.is_inf(elapsed, 0)
	{
		return result, .Invalid_Argument;
	}
	if len(output) > int(max(i32)) || len(part_output) > int(max(i32)) ||
	len(contact_output) > int(max(i32))
	{
		return result, .Invalid_Argument;
	}
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return result, .Disposed;
	}
	if tracker.simulation != owner.simulation || tracker.last_step_index > owner.simulation.step_index ||
	owner.simulation.step_index - tracker.last_step_index > 1
	{
		// a different world or a skipped history sample cannot be reconstructed.
		// reject before accepting elapsed time or acknowledging trigger output
		return result, .Invalid_Argument;
	}
	accumulator: f32 = stepper.accumulator + elapsed;
	if math.is_inf(accumulator, 0)
	{
		return result, .Invalid_Argument;
	}
	stepper.accumulator = accumulator;
	for
	{
		if owner.notification_status != .Ok
		{
			status = owner.notification_status;
			break;
		}
		if tracker.last_step_index != owner.simulation.step_index
		{
			count, required: int;
			drain: Status;
			count, required, drain = contact_events_drain(tracker, contact_output[int(result.contact_events_written):]);
			if drain != .Ok
			{
				result.contact_required = i32(required);
				result.pending += {.Contact};
				if owner.event_count != 0
				{
					result.required = i32(owner.event_count);
					result.pending += {.Parent};
				}
				if owner.mixed.event_count != 0
				{
					result.part_required = i32(owner.mixed.event_count);
					result.pending += {.Part};
				}
				status = drain;
				break;
			}
			result.contact_events_written += i32(count);
		}
		if owner.event_count > len(output) - int(result.events_written)
		{
			result.required = i32(owner.event_count);
			result.pending += {.Parent};
		}
		if owner.mixed.event_count > len(part_output) - int(result.part_events_written)
		{
			result.part_required = i32(owner.mixed.event_count);
			result.pending += {.Part};
		}
		if result.pending != {}
		{
			// the two trigger streams are acknowledged together, so both
			// nonempty batches remain pending even if only one is short
			if owner.event_count != 0
			{
				result.required = i32(owner.event_count);
				result.pending += {.Parent};
			}
			if owner.mixed.event_count != 0
			{
				result.part_required = i32(owner.mixed.event_count);
				result.pending += {.Part};
			}
			status = .Capacity_Missing;
			break;
		}
		count: int;
		drain: Status;
		count, _, drain = trigger_events_drain(world, output[int(result.events_written):]);
		if drain != .Ok
		{
			status = drain;
			break;
		}
		result.events_written += i32(count);
		count, _, drain = trigger_part_events_drain(world, part_output[int(result.part_events_written):]);
		if drain != .Ok
		{
			status = drain;
			break;
		}
		result.part_events_written += i32(count);
		if result.completed_steps >= i32(stepper.maximum_steps) || stepper.accumulator < stepper.timestep
		{
			break;
		}
		status = world_step(world, stepper.timestep, dispatcher);
		if status != .Ok
		{
			break;
		}
		stepper.accumulator = max(0, stepper.accumulator - stepper.timestep);
		result.completed_steps += 1;
	}
	result.alpha = clamp(stepper.accumulator / stepper.timestep, 0, 1);
	return result, status;
}
