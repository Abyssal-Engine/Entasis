package impulses_solver_contacts

import entasis "entasis:entasis"

STEPS :: 2;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	tracker: entasis.Contact_Tracker,
	events: [4]entasis.Contact_Event,
	written, tick: int,
	impulse: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	shape: entasis.Shape_Handle;
	shape, status = entasis.shape_add(&c.world, entasis.sphere(1));
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_inertia(entasis.sphere(1), 1);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_dynamic(shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(shape, entasis.pose({1.5, 0, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	status = entasis.contact_tracker_init(&c.tracker, 8);
	if status != .Ok
	{
		return status;
	}
	return entasis.contact_tracker_bind(&c.tracker, &c.world);
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status;
	switch c.tick
	{
	case 0:
		status = entasis.world_step(&c.world, TIMESTEP);
		if status != .Ok
		{
			return status;
		}
		c.written, _, status = entasis.contact_events_drain(&c.tracker, c.events[:]);
		if status != .Ok || c.written != 1 || c.events[0].kind != .Begin || .Solver_Data_Available not_in c.events[0].flags
		{
			return .Invalid_Argument;
		}
		contact: ^entasis.Solver_Contact_Data = &c.events[0].contact_data;
		if contact.contact_count == 0 || contact.contacts[0].feature_id < 0 || contact.contacts[0].depth <= 0
		{
			return .Invalid_Argument;
		}
		constraint_count: int;
		constraint_count, status = entasis.body_constraint_count(&c.world, c.body);
		if status != .Ok || constraint_count == 0
		{
			return .Invalid_Argument;
		}
		c.impulse, status = entasis.constraint_accumulated_impulse_magnitude(&c.world, c.events[0].constraint);
		if status != .Ok || c.impulse <= 0
		{
			return .Invalid_Argument;
		}
	case 1:
		status = entasis.body_apply_impulse(&c.world, c.body, {0, 2, 0}, {1, 0, 0});
		if status != .Ok
		{
			return status;
		}
		state: entasis.Body_State;
		state, status = entasis.body_get(&c.world, c.body);
		if status != .Ok || state.velocity.linear.y <= 0 || state.velocity.angular.z <= 0
		{
			return .Invalid_Argument;
		}
	case:
		return .Invalid_Argument;
	}
	c.tick += 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.contact_tracker_destroy(&c.tracker);
	entasis.world_destroy(&c.world);
	c^ = {};
}
