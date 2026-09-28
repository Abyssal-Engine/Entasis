package entasis
import "core:math"
import physics "entasis:entasis_physics"
Trigger_Configuration :: physics.Trigger_Configuration;
Trigger_Settings :: physics.Trigger_Settings;
Trigger_Selection :: physics.Trigger_Selection;
Trigger_Notification_State :: enum u8
{
	Ready, Pending
}

Trigger_Pair :: physics.Trigger_Pair;
Trigger_Event :: physics.Trigger_Event;
Trigger_Event_Kind :: physics.Trigger_Event_Kind;
Trigger_Exit_Reason :: physics.Trigger_Exit_Reason;
trigger_configuration_default :: physics.trigger_configuration_default;
@(private)
trigger_owner :: proc "contextless" (world:^World) -> (^physics.Trigger_System, Status)
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
	if data.simulation.triggers==nil
	{
		return nil, .Not_Found;
	}
	return data.simulation.triggers, .Ok;
}

// enables optional, non-solving collidables. no trigger worker runs until enabled
world_enable_triggers :: proc(world:^World, configuration:Trigger_Configuration={pair_capacity=1024, candidates_per_worker=1024, child_capacity=4096}) -> Status
{
	data: ^world_data = world_data_get(world);
	if data==nil
	{
		return .Disposed;
	}
	return physics.trigger_system_enable(&data.simulation, configuration, data.allocator, data.allocation_scope);
}

world_disable_triggers :: proc(world:^World) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if s.event_count!=0 || (s.mixed != nil && s.mixed.event_count != 0)
	{
		return .Invalid_Argument;
	}
	physics.trigger_system_destroy(s.simulation);
	return .Ok;
}

trigger_reserve :: proc(world:^World, configuration:Trigger_Configuration) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	return physics.trigger_system_reserve(s, configuration);
}

// mode is per instance, not per shared shape. explicit joints remain physical
trigger_set :: proc(world:^World, reference:Collidable_Reference, settings:Trigger_Settings={}) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	target: physics.Shape_Query_Target;
	resolve: Status;
	target, resolve = physics.simulation_query_target(s.simulation, reference);
	if resolve!=.Ok
	{
		return resolve;
	}
	if physics.typed_index_state(target.shape)!=.Present
	{
		return .Invalid_Argument;
	}
	if s.mixed != nil && physics.mixed_colliders_membership(s.mixed, reference) == .Present
	{
		return .Invalid_Argument;
	}
	status=physics.trigger_system_reserve(s, s.configuration);
	if status!=.Ok
	{
		return status;
	}
	slot: ^physics.Trigger_Slot = physics.trigger_slot(s, reference);
	status=physics.trigger_token(s, slot);
	if status!=.Ok
	{
		return status;
	}
	if slot.enabled == .Disabled
	{
		status=physics.trigger_retire_contacts(s, reference);
		if status!=.Ok
		{
			return status;
		}
		s.enabled_count += 1;
	}
	slot.enabled = .Enabled;
	slot.settings=settings;
	physics.trigger_changed(s, .Static if physics.collidable_reference_mobility(reference)==.Static else .Body, physics.collidable_reference_raw_handle(reference), .Modified);
	return .Ok;
}

trigger_get :: proc "contextless" (world:^World, reference:Collidable_Reference) -> (Trigger_Settings, Status)
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return {}, status;
	}
	slot: ^physics.Trigger_Slot = physics.trigger_slot(s, reference);
	if slot==nil||slot.enabled == .Disabled
	{
		return {}, .Not_Found;
	}
	return slot.settings, .Ok;
}

trigger_remove :: proc "contextless" (world:^World, reference:Collidable_Reference) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	slot: ^physics.Trigger_Slot = physics.trigger_slot(s, reference);
	if slot==nil||slot.enabled == .Disabled
	{
		return .Not_Found;
	}
	slot.enabled = .Disabled;
	s.enabled_count-=1;
	physics.trigger_changed(s, .Static if physics.collidable_reference_mobility(reference)==.Static else .Body, physics.collidable_reference_raw_handle(reference), .Modified);
	return .Ok;
}

trigger_mark_geometry_dirty :: proc "contextless" (world:^World, reference:Collidable_Reference) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	resolve: Status;
	_, resolve = physics.simulation_query_target(s.simulation, reference);
	if resolve!=.Ok
	{
		return resolve;
	}
	physics.trigger_changed(s, .Static if physics.collidable_reference_mobility(reference)==.Static else .Body, physics.collidable_reference_raw_handle(reference), .Modified);
	return .Ok;
}

trigger_mark_filters_dirty :: proc "contextless" (world:^World) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	s.all_dirty = .Present;
	s.filters_dirty = .Present;
	return .Ok;
}

// explicit recovery for a failed custom step or unannounced low-level mutation.
// keeps sensor membership and storage, discards published/history data
trigger_reset_history :: proc "contextless" (world:^World) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if s.epoch==max(u64)
	{
		return .Capacity_Missing;
	}
	s.epoch+=1;
	s.simulation.trigger_epoch=s.epoch;
	if s.mixed != nil
	{
		s.mixed.previous_count = 0;
		s.mixed.current_count = 0;
		s.mixed.event_count = 0;
	}
	s.previous_count=0;
	s.current_count=0;
	s.event_count=0;
	s.sample_valid = .Missing;
	s.notification_status=.Ok;
	s.all_dirty = .Present;
	s.filters_dirty = .Missing;
	return .Ok;
}

// all-or-nothing drain: insufficient storage does not acknowledge the batch
trigger_events_drain :: proc "contextless" (world:^World, output:[]Trigger_Event) -> (written, required:int, status:Status)
{
	s: ^physics.Trigger_System;
	st: Status;
	s, st = trigger_owner(world);
	if st!=.Ok
	{
		return 0, 0, st;
	}
	if s.notification_status!=.Ok
	{
		return 0, 0, s.notification_status;
	}
	required=s.event_count;
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	copy(output, s.events[:required]);
	s.event_count=0;
	return required, required, .Ok;
}

trigger_events_discard :: proc "contextless" (world:^World) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	if s.notification_status!=.Ok
	{
		return s.notification_status;
	}
	s.event_count=0;
	return .Ok;
}

trigger_overlaps :: proc "contextless" (world:^World, output:[]Trigger_Pair) -> (written, required:int, status:Status)
{
	s: ^physics.Trigger_System;
	st: Status;
	s, st = trigger_owner(world);
	if st!=.Ok
	{
		return 0, 0, st;
	}
	required=s.previous_count;
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	copy(output, s.previous[:required]);
	return required, required, .Ok;
}

Trigger_Update_Result :: struct
{
	completed_steps:i32,
	events_written:i32,
	required:i32,
	alpha:f32,
	notification_pending:Trigger_Notification_State,
}

// retains backlog on event backpressure. a completed physics step is consumed
// once, even when its immutable event batch must be drained by a later call
trigger_stepper_update :: proc(stepper:^Fixed_Stepper, world:^World, elapsed:f32, output:[]Trigger_Event, dispatcher:^Dispatcher=nil) -> (Trigger_Update_Result, Status)
{
	result:Trigger_Update_Result;
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return result, status;
	}
	if stepper==nil||stepper.timestep<=0||stepper.maximum_steps==0||(math.is_nan(stepper.timestep)||math.is_inf(stepper.timestep, 0))||stepper.accumulator<0||(math.is_nan(stepper.accumulator)||math.is_inf(stepper.accumulator, 0))||elapsed<0||(math.is_nan(elapsed)||math.is_inf(elapsed, 0))
	{
		return result, .Invalid_Argument;
	}
	accumulator: f32 = stepper.accumulator+elapsed;
	if (math.is_nan(accumulator)||math.is_inf(accumulator, 0))
	{
		return result, .Invalid_Argument;
	}
	stepper.accumulator=accumulator;
	for
	{
		if s.notification_status!=.Ok
		{
			status=s.notification_status;
			break;
		}
		if s.event_count>0
		{
			count, required: int;
			drain: Status;
			count, required, drain = trigger_events_drain(world, output[int(result.events_written):]);
			if drain!=.Ok
			{
				result.required=i32(required);
				status=drain;
				result.notification_pending=.Pending;
				break;
			}
			result.events_written+=i32(count);
		}
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
	if s.notification_status != .Ok
	{
		result.notification_pending = .Pending;
	}
	return result, status;
}

// captures an application ID for either endpoint without enabling sensor mode
trigger_set_user_id :: proc(world:^World, reference:Collidable_Reference, user_id:u64) -> Status
{
	s: ^physics.Trigger_System;
	status: Status;
	s, status = trigger_owner(world);
	if status!=.Ok
	{
		return status;
	}
	resolve: Status;
	_, resolve = physics.simulation_query_target(s.simulation, reference);
	if resolve!=.Ok
	{
		return resolve;
	}
	status=physics.trigger_system_reserve(s, s.configuration);
	if status!=.Ok
	{
		return status;
	}
	slot: ^physics.Trigger_Slot = physics.trigger_slot(s, reference);
	status=physics.trigger_token(s, slot);
	if status!=.Ok
	{
		return status;
	}
	slot.settings.user_id=user_id;
	physics.trigger_changed(s, .Static if physics.collidable_reference_mobility(reference)==.Static else .Body, physics.collidable_reference_raw_handle(reference), .Modified);
	return .Ok;
}
