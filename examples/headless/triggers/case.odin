package triggers

import e "entasis:entasis"

STEPS :: 182;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: e.World,
	description: e.World_Description,
	body: e.Body_Handle,
	sensor: e.Static_Handle,
	tracker: e.Contact_Tracker,
	stepper: e.Fixed_Stepper,
	events: [16]e.Trigger_Event,
	parts: [4]e.Collider_Part_Event,
	contacts: [4]e.Contact_Event,
	result: e.Collider_Part_Contact_Update_Result,
	written, enters, exits, cycle, tick: int,
	solid_velocity: f32,
}

case_create :: proc(c: ^Case) -> e.Status
{
	description: e.World_Description = e.world_description_default();
	description.gravity, description.damping = {}, {};
	c.description = description;
	status: e.Status = e.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	status = e.world_enable_triggers(&c.world);
	if status != .Ok
	{
		return status;
	}
	if c.cycle == 1
	{
		return case_mixed_create(c);
	}
	volume, visitor: e.Shape_Handle;
	volume, status = e.shape_add(&c.world, e.box(2, 2, 2));
	if status != .Ok
	{
		return status;
	}
	c.sensor, status = e.static_add(&c.world, e.static_body(volume, e.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	reference: e.Collidable_Reference;
	reference, status = e.static_collidable_reference(&c.world, c.sensor);
	if status != .Ok
	{
		return status;
	}
	status = e.trigger_set(&c.world, reference, {user_id=1001});
	if status != .Ok
	{
		return status;
	}
	visitor, status = e.shape_add(&c.world, e.sphere(0.25));
	if status != .Ok
	{
		return status;
	}
	c.body, status = e.body_add(&c.world, e.body_kinematic(visitor, e.pose({-2, 0, 0}), e.velocity({2, 0, 0}), e.body_activity(-1, 255)));
	return status;
}

case_mixed_create :: proc(c: ^Case) -> e.Status
{
	configuration: e.Collider_Part_Configuration = e.collider_part_configuration_default();
	configuration.event_subscription = .Enabled;
	status: e.Status = e.collider_parts_reserve(&c.world, configuration);
	if status != .Ok
	{
		return status;
	}
	sphere, subtree, compound: e.Shape_Handle;
	sphere, status = e.shape_add(&c.world, e.sphere(1));
	if status != .Ok
	{
		return status;
	}
	subtree, status = e.shape_import_big_compound(&c.world, []e.Compound_Child{
		e.compound_child(sphere, e.pose()), e.compound_child(sphere, e.pose({0, 0.2, 0})),
	});
	if status != .Ok
	{
		return status;
	}
	children: [2]e.Compound_Child = {e.compound_child(sphere, e.pose()), e.compound_child(subtree, e.pose({3, 0, 0}))};
	compound, status = e.shape_import_compound(&c.world, children[:]);
	if status != .Ok
	{
		return status;
	}
	inertia: e.Body_Inertia;
	inertia, status = e.shape_inertia(e.sphere(1), 1);
	if status != .Ok
	{
		return status;
	}
	c.body, status = e.body_add(&c.world, e.body_dynamic(compound, inertia, e.pose(), {}, e.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	reference: e.Collidable_Reference;
	reference, status = e.body_collidable_reference(&c.world, c.body);
	if status != .Ok
	{
		return status;
	}
	_, status = e.collider_part_set(&c.world, reference, 1, {role=.Trigger, user_id=2001});
	if status != .Ok
	{
		return status;
	}
	for position in ([2]f32{0.5, 3.5})
	{
		_, status = e.static_add(&c.world, e.static_body(sphere, e.pose({position, 0, 0})), .None);
		if status != .Ok
		{
			return status;
		}
	}
	status = e.contact_tracker_init(&c.tracker, 8);
	if status != .Ok
	{
		return status;
	}
	status = e.contact_tracker_bind(&c.tracker, &c.world);
	c.stepper = e.fixed_stepper(TIMESTEP, 4);
	return status;
}

case_step :: proc(c: ^Case) -> e.Status
{
	c.written = 0;
	if c.cycle == 0 && c.tick == 180
	{
		if c.enters != 1 || c.exits != 1
		{
			return .Invalid_Argument;
		}
		e.world_destroy(&c.world);
		c.cycle, c.tick = 1, 0;
		return case_create(c);
	}
	if c.cycle == 0
	{
		status: e.Status = e.world_step(&c.world, TIMESTEP);
		if status != .Ok
		{
			return status;
		}
		c.written, _, status = e.trigger_events_drain(&c.world, c.events[:]);
		if status != .Ok
		{
			return status;
		}
		for event in c.events[:c.written]
		{
			if event.kind == .Enter
			{
				c.enters += 1;
			}
			if event.kind == .Exit
			{
				c.exits += 1;
			}
		}
	}
	else if c.tick == 0
	{
		status: e.Status;
		c.result, status = e.collider_part_stepper_update_with_contacts(&c.stepper, &c.world, TIMESTEP,
			c.events[:4], c.parts[:], &c.tracker, c.contacts[:]);
		if status != .Ok || c.result.completed_steps != 1 || c.result.events_written != 1 ||
			c.result.part_events_written != 1 || c.result.contact_events_written != 1
		{
			return .Invalid_Argument;
		}
		c.written = int(c.result.events_written);
		state: e.Body_State;
		state, status = e.body_get(&c.world, c.body);
		if status != .Ok || state.velocity.linear.x == 0
		{
			return .Invalid_Argument;
		}
		c.solid_velocity = state.velocity.linear.x;
	}
	else
	{
		return .Invalid_Argument;
	}
	c.tick += 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	e.contact_tracker_destroy(&c.tracker);
	e.world_destroy(&c.world);
	c^ = {};
}
