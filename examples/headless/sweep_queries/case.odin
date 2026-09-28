package sweep_queries

import entasis "entasis:entasis"

STEPS :: 1;
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	moving_shape: entasis.Shape_Handle,
	hit: entasis.Sweep_Hit,
	tick: int,
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
	c.moving_shape, status = entasis.shape_add(&c.world, entasis.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	target_shape: entasis.Shape_Handle;
	target_shape, status = entasis.shape_add(&c.world, entasis.box(1, 4, 4));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(target_shape, entasis.pose({5, 0, 0})), .None);
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	if c.tick != 0
	{
		return .Invalid_Argument;
	}
	status: entasis.Status;
	c.hit, status = entasis.sweep_closest(&c.world, c.moving_shape, entasis.pose(), entasis.velocity({10, 0, 0}), 1);
	if status != .Ok || c.hit.sweep.state != .Hit ||
		c.hit.sweep.t0 <= 0 || c.hit.sweep.t0 >= 1 || c.hit.sweep.normal.x >= 0
	{
		return .Invalid_Argument;
	}
	c.tick = 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
