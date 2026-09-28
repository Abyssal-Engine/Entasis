package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

// pointer-based arrays cross the ABI only after validating size, alignment,
// offsets and disjoint count/output spans
abi_part_array_valid :: proc "contextless" (output:rawptr, capacity:u64, size, alignment:uintptr) -> entasis.Status
{
	if capacity>u64(max(int))/u64(size) || (capacity>0 && output==nil)
	{
		return .Invalid_Argument;
	}
	address:uintptr=uintptr(output);
	bytes:uintptr=uintptr(capacity)*size;
	if address%alignment!=0 || address>max(uintptr)-bytes
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

abi_part_spans_disjoint :: proc "contextless" (a:rawptr, sa:uintptr, b:rawptr, sb:uintptr) -> entasis.Status
{
	if sa==0 || sb==0
	{
		return .Ok;
	}
	x:uintptr=uintptr(a);
	y:uintptr=uintptr(b);
	if x>max(uintptr)-sa || y>max(uintptr)-sb || (x<y+sb && y<x+sa)
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

abi_part_output_valid :: proc "contextless" (output:rawptr, capacity:u64, size, alignment:uintptr, written, required:^u64, world:^Entasis_World) -> entasis.Status
{
	if written==nil || required==nil || uintptr(written)%8!=0 || uintptr(required)%8!=0
	{
		return .Invalid_Argument;
	}
	status:entasis.Status=abi_part_array_valid(output, capacity, size, alignment);
	if status!=.Ok
	{
		return status;
	}
	bytes:uintptr=uintptr(capacity)*size;
	if abi_part_spans_disjoint(written, 8, required, 8)!=.Ok || abi_part_spans_disjoint(output, bytes, written, 8)!=.Ok || abi_part_spans_disjoint(output, bytes, required, 8)!=.Ok ||
	abi_part_spans_disjoint(output, bytes, world, size_of(Entasis_World))!=.Ok || abi_part_spans_disjoint(written, 8, world, size_of(Entasis_World))!=.Ok || abi_part_spans_disjoint(required, 8, world, size_of(Entasis_World))!=.Ok
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

abi_collider_part_configuration_default :: proc "contextless" () -> Entasis_Collider_Part_Configuration
{
	value: entasis.Collider_Part_Configuration = entasis.collider_part_configuration_default();
	return {struct_size=size_of(Entasis_Collider_Part_Configuration), struct_version=1, instance_capacity=value.instance_capacity,
		part_capacity=value.part_capacity, pair_capacity=value.pair_capacity, observations_per_worker=value.observations_per_worker, event_subscription=u8(value.event_subscription)};
}

abi_collider_parts_reserve :: proc "contextless" (world:^Entasis_World, configuration:^Entasis_Collider_Part_Configuration, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if configuration==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	value:Entasis_Collider_Part_Configuration=configuration^;
	if value.struct_size!=size_of(Entasis_Collider_Part_Configuration)||value.struct_version!=1||value.event_subscription>1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	for byte in value.reserved
	{
		if byte!=0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.collider_parts_reserve(&owner.world, {instance_capacity=value.instance_capacity,
				part_capacity=value.part_capacity, pair_capacity=value.pair_capacity, observations_per_worker=value.observations_per_worker,
				event_subscription=cast(entasis.Collider_Part_Event_Subscription)value.event_subscription}), diagnostic, .None);
}

abi_collider_part_info :: proc "contextless" (value:entasis.Collider_Part_Info) -> Entasis_Collider_Part_Info
{
	return {identity={instance=value.identity.instance, incarnation=value.identity.incarnation, serial=value.identity.serial, child_index=value.identity.child_index},
		settings={user_id=value.settings.user_id, role=u8(value.settings.role), options=transmute(u8)value.settings.options}};
}

abi_collider_part_set :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, child_index:i32, settings:^Entasis_Collider_Part_Settings, out_info:^Entasis_Collider_Part_Info, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if settings==nil || out_info==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	value:Entasis_Collider_Part_Settings=settings^;
	for byte in value.reserved
	{
		if byte!=0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	info: entasis.Collider_Part_Info;
	status: entasis.Status;
	info, status=entasis.collider_part_set(&owner.world, {reference.packed}, child_index, {user_id=value.user_id,
			role=cast(entasis.Collider_Part_Role)value.role, options=transmute(entasis.Collider_Part_Options)value.options});
	if status==.Ok
	{
		out_info^=abi_collider_part_info(info);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collider_part_get :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, child_index:i32, out_info:^Entasis_Collider_Part_Info, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_info==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	info: entasis.Collider_Part_Info;
	status: entasis.Status;
	info, status=entasis.collider_part_get(&owner.world, {reference.packed}, child_index);
	if status==.Ok
	{
		out_info^=abi_collider_part_info(info);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collider_part_remove :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, child_index:i32, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	return abi_status_finish(entasis.collider_part_remove(&owner.world, {reference.packed}, child_index), diagnostic, .None);
}

abi_collider_parts :: proc "contextless" (world:^Entasis_World, reference:Entasis_Collidable_Reference, output:[^]Entasis_Collider_Part_Info, capacity:u64, out_written, out_required:^u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if abi_part_output_valid(output, capacity, size_of(Entasis_Collider_Part_Info), align_of(Entasis_Collider_Part_Info), out_written, out_required, world)!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^=0;
	out_required^=0;
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	written, required: int;
	status: entasis.Status;
	written, required, status=entasis.collider_parts(&owner.world, {reference.packed}, ([^]entasis.Collider_Part_Info)(output)[:int(capacity)]);
	// public reserved bytes are zero even when native structure padding is not
	for index in 0..<written
	{
		output[index].identity.reserved=0;
		output[index].settings.reserved={};
	}
	out_written^=u64(written);
	out_required^=u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_trigger_part_overlaps :: proc "contextless" (world:^Entasis_World, output:[^]Entasis_Collider_Part_Pair, capacity:u64, out_written, out_required:^u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if abi_part_output_valid(output, capacity, size_of(Entasis_Collider_Part_Pair), align_of(Entasis_Collider_Part_Pair), out_written, out_required, world)!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^=0;
	out_required^=0;
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	written, required: int;
	status: entasis.Status;
	written, required, status=entasis.trigger_part_overlaps(&owner.world, ([^]entasis.Collider_Part_Pair)(output)[:int(capacity)]);
	out_written^=u64(written);
	out_required^=u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_trigger_part_events_drain :: proc "contextless" (world:^Entasis_World, output:[^]Entasis_Collider_Part_Event, capacity:u64, out_written, out_required:^u64, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if abi_part_output_valid(output, capacity, size_of(Entasis_Collider_Part_Event), align_of(Entasis_Collider_Part_Event), out_written, out_required, world)!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^=0;
	out_required^=0;
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	written, required: int;
	status: entasis.Status;
	written, required, status=entasis.trigger_part_events_drain(&owner.world, ([^]entasis.Collider_Part_Event)(output)[:int(capacity)]);
	out_written^=u64(written);
	out_required^=u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_trigger_part_events_discard :: proc "contextless" (world:^Entasis_World, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	return abi_status_finish(entasis.trigger_part_events_discard(&owner.world), diagnostic, .None);
}

abi_collider_part_stepper_update :: proc "contextless" (stepper:^Entasis_Fixed_Stepper, world:^Entasis_World, elapsed:f32,
	output:[^]Entasis_Trigger_Event, capacity:u64, part_output:[^]Entasis_Collider_Part_Event, part_capacity:u64,
	out_result:^Entasis_Collider_Part_Update_Result, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if stepper==nil || out_result==nil || capacity>u64(max(i32)) || part_capacity>u64(max(i32)) ||
	uintptr(stepper)%align_of(Entasis_Fixed_Stepper)!=0 || uintptr(out_result)%align_of(Entasis_Collider_Part_Update_Result)!=0 ||
	abi_part_array_valid(output, capacity, size_of(Entasis_Trigger_Event), align_of(Entasis_Trigger_Event))!=.Ok ||
	abi_part_array_valid(part_output, part_capacity, size_of(Entasis_Collider_Part_Event), align_of(Entasis_Collider_Part_Event))!=.Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	addresses: [5]rawptr = {output, part_output, stepper, out_result, world};
	sizes: [5]uintptr = {uintptr(capacity)*size_of(Entasis_Trigger_Event),
		uintptr(part_capacity)*size_of(Entasis_Collider_Part_Event), size_of(Entasis_Fixed_Stepper),
		size_of(Entasis_Collider_Part_Update_Result), size_of(Entasis_World)};
	for i: int = 0; i < len(addresses); i += 1
	{
		for j: int = 0; j < i; j += 1
		{
			if abi_part_spans_disjoint(addresses[i], sizes[i], addresses[j], sizes[j]) != .Ok
			{
				return abi_status_finish(.Invalid_Argument, diagnostic, .None);
			}
		}
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready=abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	context=runtime.default_context();
	result: entasis.Collider_Part_Update_Result;
	status: entasis.Status;
	result, status=entasis.collider_part_stepper_update((^entasis.Fixed_Stepper)(stepper), &owner.world, elapsed,
		([^]entasis.Trigger_Event)(output)[:int(capacity)], ([^]entasis.Collider_Part_Event)(part_output)[:int(part_capacity)]);
	out_result^={completed_steps=result.completed_steps, events_written=result.events_written, part_events_written=result.part_events_written,
		required=result.required, part_required=result.part_required, alpha=result.alpha, pending=transmute(u8)result.pending};
	return abi_status_finish(status, diagnostic, .None);
}
#assert(size_of(Entasis_Collider_Part_Settings)==size_of(entasis.Collider_Part_Settings));
#assert(align_of(Entasis_Collider_Part_Settings)==align_of(entasis.Collider_Part_Settings));
#assert(offset_of(Entasis_Collider_Part_Settings, user_id)==offset_of(entasis.Collider_Part_Settings, user_id));
#assert(offset_of(Entasis_Collider_Part_Settings, role)==offset_of(entasis.Collider_Part_Settings, role));
#assert(offset_of(Entasis_Collider_Part_Settings, options)==offset_of(entasis.Collider_Part_Settings, options));
#assert(size_of(Entasis_Collider_Part_Identity)==size_of(entasis.Collider_Part_Identity));
#assert(align_of(Entasis_Collider_Part_Identity)==align_of(entasis.Collider_Part_Identity));
#assert(offset_of(Entasis_Collider_Part_Identity, instance)==offset_of(entasis.Collider_Part_Identity, instance));
#assert(offset_of(Entasis_Collider_Part_Identity, incarnation)==offset_of(entasis.Collider_Part_Identity, incarnation));
#assert(offset_of(Entasis_Collider_Part_Identity, serial)==offset_of(entasis.Collider_Part_Identity, serial));
#assert(offset_of(Entasis_Collider_Part_Identity, child_index)==offset_of(entasis.Collider_Part_Identity, child_index));
#assert(size_of(Entasis_Collider_Part_Info)==size_of(entasis.Collider_Part_Info));
#assert(align_of(Entasis_Collider_Part_Info)==align_of(entasis.Collider_Part_Info));
#assert(offset_of(Entasis_Collider_Part_Info, identity)==offset_of(entasis.Collider_Part_Info, identity));
#assert(offset_of(Entasis_Collider_Part_Info, settings)==offset_of(entasis.Collider_Part_Info, settings));
#assert(size_of(Entasis_Collider_Part_Endpoint)==size_of(entasis.Collider_Part_Endpoint));
#assert(align_of(Entasis_Collider_Part_Endpoint)==align_of(entasis.Collider_Part_Endpoint));
#assert(offset_of(Entasis_Collider_Part_Endpoint, reference)==offset_of(entasis.Collider_Part_Endpoint, reference));
#assert(offset_of(Entasis_Collider_Part_Endpoint, child_index)==offset_of(entasis.Collider_Part_Endpoint, child_index));
#assert(offset_of(Entasis_Collider_Part_Endpoint, instance)==offset_of(entasis.Collider_Part_Endpoint, instance));
#assert(offset_of(Entasis_Collider_Part_Endpoint, incarnation)==offset_of(entasis.Collider_Part_Endpoint, incarnation));
#assert(offset_of(Entasis_Collider_Part_Endpoint, serial)==offset_of(entasis.Collider_Part_Endpoint, serial));
#assert(offset_of(Entasis_Collider_Part_Endpoint, user_id)==offset_of(entasis.Collider_Part_Endpoint, user_id));
#assert(size_of(Entasis_Collider_Part_Pair)==size_of(entasis.Collider_Part_Pair));
#assert(align_of(Entasis_Collider_Part_Pair)==align_of(entasis.Collider_Part_Pair));
#assert(offset_of(Entasis_Collider_Part_Pair, a)==offset_of(entasis.Collider_Part_Pair, a));
#assert(offset_of(Entasis_Collider_Part_Pair, b)==offset_of(entasis.Collider_Part_Pair, b));
#assert(offset_of(Entasis_Collider_Part_Pair, flags)==offset_of(entasis.Collider_Part_Pair, flags));
#assert(offset_of(Entasis_Collider_Part_Pair, reserved)==offset_of(entasis.Collider_Part_Pair, reserved));
#assert(size_of(Entasis_Collider_Part_Event)==size_of(entasis.Collider_Part_Event));
#assert(align_of(Entasis_Collider_Part_Event)==align_of(entasis.Collider_Part_Event));
#assert(offset_of(Entasis_Collider_Part_Event, pair)==offset_of(entasis.Collider_Part_Event, pair));
#assert(offset_of(Entasis_Collider_Part_Event, step)==offset_of(entasis.Collider_Part_Event, step));
#assert(offset_of(Entasis_Collider_Part_Event, epoch)==offset_of(entasis.Collider_Part_Event, epoch));
#assert(offset_of(Entasis_Collider_Part_Event, kind)==offset_of(entasis.Collider_Part_Event, kind));
#assert(offset_of(Entasis_Collider_Part_Event, reason)==offset_of(entasis.Collider_Part_Event, reason));
abi_collider_part_stepper_update_with_contacts :: proc "contextless" (
	stepper: ^Entasis_Fixed_Stepper, world: ^Entasis_World, elapsed: f32,
	output: [^]Entasis_Trigger_Event, capacity: u64,
	part_output: [^]Entasis_Collider_Part_Event, part_capacity: u64,
	tracker: ^Entasis_Contact_Tracker, contact_output: [^]Entasis_Contact_Event, contact_capacity: u64,
	out_result: ^Entasis_Collider_Part_Contact_Update_Result, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if stepper == nil || out_result == nil || capacity > u64(max(i32)) ||
	part_capacity > u64(max(i32)) || contact_capacity > u64(max(i32)) ||
	uintptr(stepper) % align_of(Entasis_Fixed_Stepper) != 0 ||
	uintptr(out_result) % align_of(Entasis_Collider_Part_Contact_Update_Result) != 0 ||
	abi_part_array_valid(output, capacity, size_of(Entasis_Trigger_Event), align_of(Entasis_Trigger_Event)) != .Ok ||
	abi_part_array_valid(part_output, part_capacity, size_of(Entasis_Collider_Part_Event), align_of(Entasis_Collider_Part_Event)) != .Ok ||
	abi_part_array_valid(contact_output, contact_capacity, size_of(Entasis_Contact_Event), align_of(Entasis_Contact_Event)) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	// this boundary writes all three arrays and the accumulator/result. handles
	// must survive those writes and release. reject overlapping caller storage
	addresses: [7]rawptr = {output, part_output, contact_output, stepper, out_result, world, tracker};
	sizes: [7]uintptr = {uintptr(capacity)*size_of(Entasis_Trigger_Event),
		uintptr(part_capacity)*size_of(Entasis_Collider_Part_Event), uintptr(contact_capacity)*size_of(Entasis_Contact_Event),
		size_of(Entasis_Fixed_Stepper), size_of(Entasis_Collider_Part_Contact_Update_Result),
		size_of(Entasis_World), size_of(Entasis_Contact_Tracker)};
	for i: int = 0; i < len(addresses); i += 1
	{
		for j: int = 0; j < i; j += 1
		{
			if abi_part_spans_disjoint(addresses[i], sizes[i], addresses[j], sizes[j]) != .Ok
			{
				return abi_status_finish(.Invalid_Argument, diagnostic, .None);
			}
		}
	}
	resource: ^abi_contact_tracker_resource = abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	if resource.bound_world != owner
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	context = runtime.default_context();
	result: entasis.Collider_Part_Contact_Update_Result;
	status: entasis.Status;
	result, status = entasis.collider_part_stepper_update_with_contacts(
		(^entasis.Fixed_Stepper)(stepper), &owner.world, elapsed,
		([^]entasis.Trigger_Event)(output)[:int(capacity)],
		([^]entasis.Collider_Part_Event)(part_output)[:int(part_capacity)],
		&resource.tracker, ([^]entasis.Contact_Event)(contact_output)[:int(contact_capacity)],
	);
	out_result^ = {completed_steps=result.completed_steps, events_written=result.events_written,
		part_events_written=result.part_events_written, contact_events_written=result.contact_events_written,
		required=result.required, part_required=result.part_required, contact_required=result.contact_required,
		alpha=result.alpha, pending=transmute(u8)result.pending};
	return abi_status_finish(status, diagnostic, .None);
}
