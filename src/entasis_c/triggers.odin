package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_trigger_configuration_default :: proc "contextless" () -> Entasis_Trigger_Configuration
{
	c:=entasis.trigger_configuration_default();
	return {struct_size=u32(size_of(Entasis_Trigger_Configuration)), struct_version=1, pair_capacity=c.pair_capacity, candidates_per_worker=c.candidates_per_worker, child_capacity=c.child_capacity};
}

abi_trigger_configuration :: proc "contextless" (input:^Entasis_Trigger_Configuration) -> (entasis.Trigger_Configuration, entasis.Status)
{
	if input==nil
	{
		return entasis.trigger_configuration_default(), .Ok;
	}
	c:=input^;
	if c.struct_size!=size_of(Entasis_Trigger_Configuration)||c.struct_version!=1
	{
		return {}, .Invalid_Argument;
	}
	return {c.pair_capacity, c.candidates_per_worker, c.child_capacity}, .Ok;
}

abi_world_enable_triggers :: proc "contextless" (world:^Entasis_World, configuration:^Entasis_Trigger_Configuration, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	c, status:=abi_trigger_configuration(configuration);
	if status!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.world_enable_triggers(&owner.world, c), diagnostic, .None);
}

abi_trigger_reserve :: proc "contextless" (world:^Entasis_World, configuration:^Entasis_Trigger_Configuration, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	c, status:=abi_trigger_configuration(configuration);
	if status!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_reserve(&owner.world, c), diagnostic, .None);
}

abi_trigger_set :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, settings:^Entasis_Trigger_Settings, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	value:Entasis_Trigger_Settings;
	if settings!=nil
	{
		value=settings^;
	}
	if value.stay>1||value.static_static>1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	for v in value.reserved
	{
		if v!=0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_set(&owner.world, {reference.packed}, {user_id=value.user_id, stay=(.Enabled if value.stay!=0 else .Disabled), static_static=(.Enabled if value.static_static!=0 else .Disabled)}), diagnostic, .None);
}

abi_trigger_get :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, out_settings:^Entasis_Trigger_Settings, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_settings==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	settings, status:=entasis.trigger_get(&owner.world, {reference.packed});
	if status==.Ok
	{
		out_settings^={user_id=settings.user_id, stay=abi_bool(settings.stay == .Enabled), static_static=abi_bool(settings.static_static == .Enabled)};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_world_disable_triggers :: proc "contextless" (world:^Entasis_World, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.world_disable_triggers(&owner.world), diagnostic, .None);
}

abi_trigger_remove :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_remove(&owner.world, {reference.packed}), diagnostic, .None);
}

abi_trigger_mark_geometry_dirty :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_mark_geometry_dirty(&owner.world, {reference.packed}), diagnostic, .None);
}

abi_trigger_mark_filters_dirty :: proc "contextless" (world:^Entasis_World, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_mark_filters_dirty(&owner.world), diagnostic, .None);
}

abi_trigger_reset_history :: proc "contextless" (world:^Entasis_World, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_reset_history(&owner.world), diagnostic, .None);
}

abi_trigger_events_discard :: proc "contextless" (world:^Entasis_World, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_events_discard(&owner.world), diagnostic, .None);
}

abi_trigger_events_drain :: proc "contextless" (world:^Entasis_World, output:[^]Entasis_Trigger_Event, capacity:u64, out_written, out_required:^u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_written==nil||out_required==nil||capacity>u64(max(int)/size_of(Entasis_Trigger_Event))||(capacity>0&&output==nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^=0;
	out_required^=0;
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None, .Exclusive);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	written, required, status:=entasis.trigger_events_drain(&owner.world, ([^]entasis.Trigger_Event)(output)[:int(capacity)]);
	out_written^=u64(written);
	out_required^=u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_trigger_overlaps :: proc "contextless" (world:^Entasis_World, output:[^]Entasis_Trigger_Pair, capacity:u64, out_written, out_required:^u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_written==nil||out_required==nil||capacity>u64(max(int)/size_of(Entasis_Trigger_Pair))||(capacity>0&&output==nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^=0;
	out_required^=0;
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	written, required, status:=entasis.trigger_overlaps(&owner.world, ([^]entasis.Trigger_Pair)(output)[:int(capacity)]);
	out_written^=u64(written);
	out_required^=u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_trigger_stepper_update :: proc "contextless" (stepper:^Entasis_Fixed_Stepper, world:^Entasis_World, elapsed:f32, output:[^]Entasis_Trigger_Event, capacity:u64, out_result:^Entasis_Trigger_Update_Result, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if stepper==nil||out_result==nil||capacity>u64(max(i32)/size_of(Entasis_Trigger_Event))||(capacity>0&&output==nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	result, status:=entasis.trigger_stepper_update((^entasis.Fixed_Stepper)(stepper), &owner.world, elapsed, ([^]entasis.Trigger_Event)(output)[:int(capacity)]);
	out_result^={completed_steps=result.completed_steps, events_written=result.events_written, required=result.required, alpha=result.alpha, notification_pending=abi_bool(result.notification_pending == .Pending)};
	return abi_status_finish(status, diagnostic, .None);
}
#assert(size_of(Entasis_Trigger_Pair)==size_of(entasis.Trigger_Pair));
#assert(align_of(Entasis_Trigger_Pair)==align_of(entasis.Trigger_Pair));
#assert(offset_of(Entasis_Trigger_Pair, a)==offset_of(entasis.Trigger_Pair, a));
#assert(offset_of(Entasis_Trigger_Pair, b)==offset_of(entasis.Trigger_Pair, b));
#assert(offset_of(Entasis_Trigger_Pair, token_a)==offset_of(entasis.Trigger_Pair, token_a));
#assert(offset_of(Entasis_Trigger_Pair, token_b)==offset_of(entasis.Trigger_Pair, token_b));
#assert(offset_of(Entasis_Trigger_Pair, user_a)==offset_of(entasis.Trigger_Pair, user_a));
#assert(offset_of(Entasis_Trigger_Pair, user_b)==offset_of(entasis.Trigger_Pair, user_b));
#assert(offset_of(Entasis_Trigger_Pair, flags)==offset_of(entasis.Trigger_Pair, flags));
#assert(size_of(Entasis_Trigger_Event)==size_of(entasis.Trigger_Event));
#assert(align_of(Entasis_Trigger_Event)==align_of(entasis.Trigger_Event));
#assert(offset_of(Entasis_Trigger_Event, pair)==offset_of(entasis.Trigger_Event, pair));
#assert(offset_of(Entasis_Trigger_Event, step)==offset_of(entasis.Trigger_Event, step));
#assert(offset_of(Entasis_Trigger_Event, epoch)==offset_of(entasis.Trigger_Event, epoch));
#assert(offset_of(Entasis_Trigger_Event, kind)==offset_of(entasis.Trigger_Event, kind));
#assert(offset_of(Entasis_Trigger_Event, reason)==offset_of(entasis.Trigger_Event, reason));
#assert(offset_of(Entasis_Trigger_Pair, reserved)==offset_of(entasis.Trigger_Pair, reserved));
abi_trigger_set_user_id :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, user_id:u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.trigger_set_user_id(&owner.world, {reference.packed}, user_id), diagnostic, .None);
}
