package ray_queries

import entasis "entasis:entasis"

STEPS :: 3;
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	handles: [3]entasis.Static_Handle,
	ray: entasis.Ray,
	hits: [8]entasis.Ray_Hit,
	count, tick: int,
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
	shape, status = entasis.shape_add(&c.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	for &handle, index in c.handles
	{
		handle, status = entasis.static_add(&c.world,
			entasis.static_body(shape, entasis.pose({f32(index*3), 0, 0})), .None);
		if status != .Ok
		{
			return status;
		}
	}
	c.ray = entasis.ray({-5, 0, 0}, {1, 0, 0}, 20);
	return .Ok;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	switch c.tick
	{
	case 0:
		any: bool;
		status: entasis.Status;
		any, status = entasis.ray_cast_any(&c.world, c.ray);
		if status != .Ok || !any
		{
			return .Invalid_Argument;
		}
	case 1:
		status: entasis.Status;
		c.hits[0], status = entasis.ray_cast_closest(&c.world, c.ray);
		if status != .Ok
		{
			return status;
		}
		closest: entasis.Static_Handle;
		closest, status = entasis.collidable_static_handle(c.hits[0].collidable);
		if status != .Ok || closest != c.handles[0]
		{
			return .Invalid_Argument;
		}
		c.count = 1;
	case 2:
		status: entasis.Status;
		c.count, status = entasis.ray_cast_all(&c.world, c.ray, c.hits[:]);
		if status != .Ok || c.count != 3 || c.hits[0].t > c.hits[1].t || c.hits[1].t > c.hits[2].t
		{
			return .Invalid_Argument;
		}
	case: return .Invalid_Argument;
	}
	c.tick += 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
