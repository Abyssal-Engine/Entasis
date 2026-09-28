package overlap_and_events

import entasis "entasis:entasis"

STEPS :: 3;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	query_shape: entasis.Shape_Handle,
	tracker: entasis.Contact_Tracker,
	overlaps: [8]entasis.Overlap_Hit,
	events: [4]entasis.Contact_Event,
	overlap_count, written, tick: int,
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
	c.query_shape = shape;
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, shape, 1);
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
	c.overlap_count, status = entasis.overlap_all(&c.world, shape, entasis.pose(), c.overlaps[:]);
	if status != .Ok || c.overlap_count < 1
	{
		return .Invalid_Argument;
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
	if c.tick >= STEPS
	{
		return .Invalid_Argument;
	}
	if c.tick == 2
	{
		status: entasis.Status = entasis.body_set_pose(&c.world, c.body, entasis.pose({10, 0, 0}));
		if status != .Ok
		{
			return status;
		}
	}
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status != .Ok
	{
		return status;
	}
	c.written, _, status = entasis.contact_events_drain(&c.tracker, c.events[:]);
	kinds: [3]entasis.Contact_Event_Kind = {.Begin, .Persist, .End};
	if status != .Ok || c.written != 1 || c.events[0].kind != kinds[c.tick]
	{
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
