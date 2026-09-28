package physics_observation

import entasis "entasis:entasis"
import scene "../physics_scene"

observation_trigger :: proc(o: ^Observation, a, b: entasis.Collidable_Reference, kind: entasis.Trigger_Event_Kind, part_a, part_b: i32)
{
	id_a: u32 = observation_collidable(o, a);
	id_b: u32 = observation_collidable(o, b);
	style: scene.Overlay_Kind;
	switch kind
	{
	case .Enter: style = .Enter;
	case .Stay: style = .Stay;
	case .Exit: style = .Exit;
	}
	append(&o.packet.frame.overlays, scene.Overlay{kind=style, entity_a=id_a, entity_b=id_b,
		part_a=part_a, part_b=part_b, a=observation_position(o, id_a), b=observation_position(o, id_b)});
}

observation_position :: proc(o: ^Observation, id: u32) -> scene.Vec3
{
	for value, index in o.packet.frame.ids
	{
		if value == id
		{
			return o.packet.frame.poses[index].position;
		}
	}
	return {};
}

observation_contacts :: proc(o: ^Observation, world: ^entasis.World, events: []entasis.Contact_Event) -> scene.Status
{
	for &event in events
	{
		a: u32 = observation_collidable(o, event.a);
		b: u32 = observation_collidable(o, event.b);
		kind: scene.Overlay_Kind;
		switch event.kind
		{
		case .Begin: kind = .Enter;
		case .Persist: kind = .Stay;
		case .End: kind = .Exit;
		}
		append(&o.packet.frame.overlays, scene.Overlay{kind=kind, entity_a=a, entity_b=b,
			part_a=-1, part_b=-1, a=observation_position(o, a), b=observation_position(o, b), value=f32(event.contact_count)});
		if .Solver_Data_Available in event.flags && event.kind != .End
		{
			body: entasis.Body_State;
			status: entasis.Status;
			body, status = entasis.body_get(world, event.contact_data.body_handles[0]);
			if status != .Ok
			{
				return .Invalid_Data;
			}
			for contact in event.contact_data.contacts[:event.contact_data.contact_count]
			{
				point: scene.Vec3 = copy_vector(body.pose.position)+copy_vector(contact.offset_a);
				normal: scene.Vec3 = copy_vector(contact.normal);
				append(&o.packet.frame.overlays,
					scene.Overlay{kind=.Contact, entity_a=a, entity_b=b, part_a=-1, part_b=-1,
						a=point, b=point+normal, value=contact.depth},
					scene.Overlay{kind=.Contact_Impulse, entity_a=a, entity_b=b, part_a=-1, part_b=-1,
						a=point, b=point+normal*contact.normal_impulse, value=contact.normal_impulse});
			}
		}
	}
	return .Ok;
}

observation_collidable :: proc(o: ^Observation, reference: entasis.Collidable_Reference) -> u32
{
	kind: scene.Object_Kind;
	handle: i32;
	status: entasis.Status;
	if entasis.collidable_mobility(reference) == .Static
	{
		value: entasis.Static_Handle;
		value, status = entasis.collidable_static_handle(reference);
		kind, handle = .Static, value.value;
	}
	else
	{
		value: entasis.Body_Handle;
		value, status = entasis.collidable_body_handle(reference);
		kind, handle = .Body, value.value;
	}
	if status != .Ok
	{
		return scene.NO_ENTITY;
	}
	for entity, index in o.packet.entities
	{
		if entity.kind == kind && entity.source_handle == handle && o.lifetimes[index] == .Available
		{
			return u32(index);
		}
	}
	return scene.NO_ENTITY;
}

