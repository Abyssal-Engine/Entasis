package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

abi_joint_reaction_provider_bridge :: proc "contextless" (
	user_context: rawptr, input: ^entasis.Joint_Reaction_Provider_Input,
	output: ^[4]entasis.Joint_Impulse_Wrench,
) -> entasis.Status
{
	binding: ^abi_custom_constraint_binding = (^abi_custom_constraint_binding)(user_context);
	view: Entasis_Joint_Reaction_Provider_Input = {
		type_id=Entasis_Constraint_Type_ID(input.type_id), body_count=u32(input.body_count),
		description=input.description, description_size=u32(input.description_size),
		impulse_count=u32(input.impulse_count), substep_duration=input.substep_duration,
	};
	for index: i32 = 0; index < input.body_count; index += 1
	{
		view.poses[index] = abi_pose_from_core(input.poses[index]);
	}
	for index: i32 = 0; index < input.impulse_count; index += 1
	{
		view.impulses[index] = input.impulses[index];
	}
	wrenches: [4]Entasis_Joint_Impulse_Wrench;
	status: entasis.Status = abi_custom_callback_status(binding.reaction_provider.read(
			binding.reaction_provider.user_context, &view, &wrenches[0],
	));
	if status != .Ok
	{
		return status;
	}
	for index: i32 = 0; index < input.body_count; index += 1
	{
		output[index] = {
			linear=abi_vector3_to_core(wrenches[index].linear),
			angular=abi_vector3_to_core(wrenches[index].angular),
		};
	}
	return .Ok;
}

abi_world_enable_joint_breaks :: proc "contextless" (
	world: ^Entasis_World, watch_capacity: i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_enable_joint_breaks(&owner.world, watch_capacity), diagnostic, .None);
}

abi_world_disable_joint_breaks :: proc "contextless" (
	world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	status: entasis.Status = entasis.world_disable_joint_breaks(&owner.world);
	if status == .Ok
	{
		for binding: ^abi_custom_constraint_binding = owner.custom_constraints; binding != nil; binding = binding.next
		{
			binding.reaction_provider = {};
		}
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_joint_break_reserve :: proc "contextless" (
	world: ^Entasis_World, watch_capacity: i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.joint_break_reserve(&owner.world, watch_capacity), diagnostic, .None);
}

abi_constraint_set_reaction_provider :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Constraint_Type_ID,
	provider: ^Entasis_Joint_Reaction_Provider, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	selected: Entasis_Joint_Reaction_Provider;
	if provider != nil
	{
		if uintptr(provider) % align_of(Entasis_Joint_Reaction_Provider) != 0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		selected = provider^;
		if selected.struct_size != size_of(Entasis_Joint_Reaction_Provider) ||
		selected.struct_version != ENTASIS_STRUCT_VERSION || selected.read == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	binding: ^abi_custom_constraint_binding = abi_custom_constraint_find(owner, type_id);
	if binding == nil
	{
		return abi_status_finish(.Not_Found, diagnostic, .None);
	}
	core: entasis.Joint_Reaction_Provider;
	if provider != nil
	{
		core = {read=abi_joint_reaction_provider_bridge, user_context=binding};
	}
	context = runtime.default_context();
	status: entasis.Status = entasis.constraint_set_reaction_provider(&owner.world, entasis.Constraint_Type_ID(type_id), core);
	if status == .Ok
	{
		binding.reaction_provider = selected;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_constraint_set_break_limits :: proc "contextless" (
	world: ^Entasis_World, constraint: Entasis_Constraint_Handle,
	limits: ^Entasis_Joint_Break_Limits, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if limits == nil || uintptr(limits) % align_of(Entasis_Joint_Break_Limits) != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	selected: Entasis_Joint_Break_Limits = limits^;
	if selected.reserved != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.constraint_set_break_limits(&owner.world, abi_constraint_handle_to_core(constraint), {
				metrics=transmute(entasis.Joint_Break_Metrics)selected.metrics,
				force=selected.force, torque=selected.torque, user_id=selected.user_id,
		}), diagnostic, .None);
}

abi_constraint_get_break_limits :: proc "contextless" (
	world: ^Entasis_World, constraint: Entasis_Constraint_Handle,
	out_limits: ^Entasis_Joint_Break_Limits, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_limits == nil || uintptr(out_limits) % align_of(Entasis_Joint_Break_Limits) != 0 ||
	abi_part_spans_disjoint(out_limits, size_of(Entasis_Joint_Break_Limits), world, size_of(Entasis_World)) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value: entasis.Joint_Break_Limits;
	status: entasis.Status;
	value, status = entasis.constraint_get_break_limits(&owner.world, abi_constraint_handle_to_core(constraint));
	if status == .Ok
	{
		out_limits^ = {metrics=transmute(u32)value.metrics, force=value.force, torque=value.torque, user_id=value.user_id};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_constraint_clear_break_limits :: proc "contextless" (
	world: ^Entasis_World, constraint: Entasis_Constraint_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.constraint_clear_break_limits(&owner.world, abi_constraint_handle_to_core(constraint)), diagnostic, .None);
}

abi_constraint_reaction :: proc "contextless" (
	world: ^Entasis_World, constraint: Entasis_Constraint_Handle,
	out_reaction: ^Entasis_Joint_Reaction, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_reaction == nil || uintptr(out_reaction) % align_of(Entasis_Joint_Reaction) != 0 ||
	abi_part_spans_disjoint(out_reaction, size_of(Entasis_Joint_Reaction), world, size_of(Entasis_World)) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value: entasis.Joint_Reaction;
	status: entasis.Status;
	value, status = entasis.constraint_reaction(&owner.world, abi_constraint_handle_to_core(constraint));
	if status == .Ok
	{
		result: Entasis_Joint_Reaction = {
			step=value.step, substep=value.substep, state=u32(value.state),
			maximum_force=value.maximum_force, maximum_torque=value.maximum_torque,
			sample={body_count=u32(value.sample.body_count), substep_duration=value.sample.substep_duration,
				maximum_force=value.sample.maximum_force, maximum_torque=value.sample.maximum_torque},
		};
		for index: i32 = 0; index < value.sample.body_count; index += 1
		{
			result.sample.bodies[index] = abi_body_handle_from_core(value.sample.bodies[index]);
			result.sample.forces[index] = {
				linear=abi_vector3_from_core(value.sample.forces[index].linear),
				angular=abi_vector3_from_core(value.sample.forces[index].angular),
			};
		}
		out_reaction^ = result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_constraint_break_events_drain :: proc "contextless" (
	world: ^Entasis_World, output: [^]Entasis_Joint_Break_Event, capacity: u64,
	out_written, out_required: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if abi_part_output_valid(output, capacity, size_of(Entasis_Joint_Break_Event),
		align_of(Entasis_Joint_Break_Event), out_written, out_required, world) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_written^ = 0;
	out_required^ = 0;
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	written, required: int;
	status: entasis.Status;
	written, required, status = entasis.constraint_break_events_drain(&owner.world,
		([^]entasis.Joint_Break_Event)(output)[:int(capacity)]);
	for index: int = 0; index < written; index += 1
	{
		output[index].reserved = 0;
	}
	out_written^ = u64(written);
	out_required^ = u64(required);
	return abi_status_finish(status, diagnostic, .None);
}

abi_constraint_break_events_discard :: proc "contextless" (
	world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.constraint_break_events_discard(&owner.world), diagnostic, .None);
}

abi_joint_break_stepper_update :: proc "contextless" (
	stepper: ^Entasis_Fixed_Stepper, world: ^Entasis_World, elapsed: f32,
	break_output: [^]Entasis_Joint_Break_Event, break_capacity: u64,
	output: [^]Entasis_Trigger_Event, capacity: u64,
	part_output: [^]Entasis_Collider_Part_Event, part_capacity: u64,
	tracker: ^Entasis_Contact_Tracker, contact_output: [^]Entasis_Contact_Event, contact_capacity: u64,
	out_result: ^Entasis_Joint_Break_Update_Result, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if stepper == nil || out_result == nil || capacity > u64(max(i32)) || part_capacity > u64(max(i32)) ||
	contact_capacity > u64(max(i32)) || break_capacity > u64(max(i32)) ||
	uintptr(stepper) % align_of(Entasis_Fixed_Stepper) != 0 ||
	uintptr(out_result) % align_of(Entasis_Joint_Break_Update_Result) != 0 ||
	abi_part_array_valid(output, capacity, size_of(Entasis_Trigger_Event), align_of(Entasis_Trigger_Event)) != .Ok ||
	abi_part_array_valid(part_output, part_capacity, size_of(Entasis_Collider_Part_Event), align_of(Entasis_Collider_Part_Event)) != .Ok ||
	abi_part_array_valid(contact_output, contact_capacity, size_of(Entasis_Contact_Event), align_of(Entasis_Contact_Event)) != .Ok ||
	abi_part_array_valid(break_output, break_capacity, size_of(Entasis_Joint_Break_Event), align_of(Entasis_Joint_Break_Event)) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	addresses: [8]rawptr = {output, part_output, contact_output, break_output, stepper, out_result, world, tracker};
	sizes: [8]uintptr = {uintptr(capacity)*size_of(Entasis_Trigger_Event),
		uintptr(part_capacity)*size_of(Entasis_Collider_Part_Event), uintptr(contact_capacity)*size_of(Entasis_Contact_Event),
		uintptr(break_capacity)*size_of(Entasis_Joint_Break_Event), size_of(Entasis_Fixed_Stepper),
		size_of(Entasis_Joint_Break_Update_Result), size_of(Entasis_World),
		size_of(Entasis_Contact_Tracker) if tracker != nil else 0};
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
	contact: ^abi_contact_tracker_resource;
	core_tracker: ^entasis.Contact_Tracker;
	if tracker != nil
	{
		contact = abi_contact_tracker_get(tracker);
		if contact == nil
		{
			return abi_status_finish(.Disposed, diagnostic, .None);
		}
		core_tracker = &contact.tracker;
	}
	owner: ^abi_world_resource;
	ready: Entasis_Status;
	owner, ready = abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	if contact != nil && contact.bound_world != owner
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	context = runtime.default_context();
	result: entasis.Joint_Break_Update_Result;
	status: entasis.Status;
	result, status = entasis.joint_break_stepper_update((^entasis.Fixed_Stepper)(stepper), &owner.world, elapsed,
		([^]entasis.Joint_Break_Event)(break_output)[:int(break_capacity)],
		([^]entasis.Trigger_Event)(output)[:int(capacity)],
		([^]entasis.Collider_Part_Event)(part_output)[:int(part_capacity)], core_tracker,
		([^]entasis.Contact_Event)(contact_output)[:int(contact_capacity)]);
	for index: i32 = 0; index < result.break_events_written; index += 1
	{
		break_output[index].reserved = 0;
	}
	out_result^ = {completed_steps=result.completed_steps, events_written=result.events_written,
		part_events_written=result.part_events_written, contact_events_written=result.contact_events_written,
		break_events_written=result.break_events_written, required=result.required, part_required=result.part_required,
		contact_required=result.contact_required, break_required=result.break_required, alpha=result.alpha,
		pending=transmute(u8)result.pending};
	return abi_status_finish(status, diagnostic, .None);
}
#assert(size_of(Entasis_Joint_Force_Wrench) == size_of(physics.Constraint_Force_Wrench));
#assert(align_of(Entasis_Joint_Force_Wrench) == align_of(physics.Constraint_Force_Wrench));
#assert(offset_of(Entasis_Joint_Force_Wrench, linear) == offset_of(physics.Constraint_Force_Wrench, linear));
#assert(offset_of(Entasis_Joint_Force_Wrench, angular) == offset_of(physics.Constraint_Force_Wrench, angular));
#assert(size_of(Entasis_Joint_Reaction_Sample) == size_of(physics.Constraint_Reaction_Sample));
#assert(align_of(Entasis_Joint_Reaction_Sample) == align_of(physics.Constraint_Reaction_Sample));
#assert(offset_of(Entasis_Joint_Reaction_Sample, body_count) == offset_of(physics.Constraint_Reaction_Sample, body_count));
#assert(offset_of(Entasis_Joint_Reaction_Sample, bodies) == offset_of(physics.Constraint_Reaction_Sample, bodies));
#assert(offset_of(Entasis_Joint_Reaction_Sample, forces) == offset_of(physics.Constraint_Reaction_Sample, forces));
#assert(offset_of(Entasis_Joint_Reaction_Sample, substep_duration) == offset_of(physics.Constraint_Reaction_Sample, substep_duration));
#assert(offset_of(Entasis_Joint_Reaction_Sample, maximum_force) == offset_of(physics.Constraint_Reaction_Sample, maximum_force));
#assert(offset_of(Entasis_Joint_Reaction_Sample, maximum_torque) == offset_of(physics.Constraint_Reaction_Sample, maximum_torque));
#assert(size_of(Entasis_Joint_Reaction) == size_of(entasis.Joint_Reaction));
#assert(align_of(Entasis_Joint_Reaction) == align_of(entasis.Joint_Reaction));
#assert(offset_of(Entasis_Joint_Reaction, sample) == offset_of(entasis.Joint_Reaction, sample));
#assert(offset_of(Entasis_Joint_Reaction, step) == offset_of(entasis.Joint_Reaction, step));
#assert(offset_of(Entasis_Joint_Reaction, substep) == offset_of(entasis.Joint_Reaction, substep));
#assert(offset_of(Entasis_Joint_Reaction, state) == offset_of(entasis.Joint_Reaction, state));
#assert(offset_of(Entasis_Joint_Reaction, maximum_force) == offset_of(entasis.Joint_Reaction, maximum_force));
#assert(offset_of(Entasis_Joint_Reaction, maximum_torque) == offset_of(entasis.Joint_Reaction, maximum_torque));
#assert(size_of(Entasis_Joint_Break_Event) == size_of(entasis.Joint_Break_Event));
#assert(align_of(Entasis_Joint_Break_Event) == align_of(entasis.Joint_Break_Event));
#assert(offset_of(Entasis_Joint_Break_Event, constraint) == offset_of(entasis.Joint_Break_Event, constraint));
#assert(offset_of(Entasis_Joint_Break_Event, type_id) == offset_of(entasis.Joint_Break_Event, type_id));
#assert(offset_of(Entasis_Joint_Break_Event, lifetime) == offset_of(entasis.Joint_Break_Event, lifetime));
#assert(offset_of(Entasis_Joint_Break_Event, user_id) == offset_of(entasis.Joint_Break_Event, user_id));
#assert(offset_of(Entasis_Joint_Break_Event, exceeded) == offset_of(entasis.Joint_Break_Event, exceeded));
#assert(offset_of(Entasis_Joint_Break_Event, reaction) == offset_of(entasis.Joint_Break_Event, reaction));
