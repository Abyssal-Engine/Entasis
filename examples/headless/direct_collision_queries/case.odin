package direct_collision_queries

import entasis "entasis:entasis"

STEPS :: 3;
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	sphere, box: entasis.Shape_Handle,
	static: entasis.Static_Handle,
	pose: entasis.Rigid_Pose,
	manifold: entasis.Manifold_Result,
	hits: [8]entasis.Sweep_Hit,
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
	c.sphere, status = entasis.shape_add(&c.world, entasis.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	c.box, status = entasis.shape_add(&c.world, entasis.box(2, 1, 2));
	c.pose = entasis.pose({0, 0.6, 0});
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status;
	switch c.tick
	{
	case 0:
		c.manifold, status = entasis.collision_query(&c.world, c.sphere, c.pose, c.box, entasis.pose());
		if status != .Ok || c.manifold.kind != .Convex || c.manifold.convex.count == 0
		{
			return .Invalid_Argument;
		}
	case 1:
		c.pose = entasis.pose({0, 5, 0});
		_, status = entasis.collision_query(&c.world, c.sphere, c.pose, c.box, entasis.pose());
		if status != .Not_Found
		{
			return .Invalid_Argument;
		}
	case 2:
		c.static, status = entasis.static_add(&c.world, entasis.static_body(c.box, entasis.pose()), .None);
		if status != .Ok
		{
			return status;
		}
		c.pose = entasis.pose({0, 3, 0});
		found: bool;
		found, status = entasis.sweep_any(&c.world, c.sphere, c.pose, entasis.velocity({0, -5, 0}), 1);
		if status != .Ok || !found
		{
			return .Invalid_Argument;
		}
		c.count, status = entasis.sweep_all(&c.world, c.sphere, c.pose, entasis.velocity({0, -5, 0}), 1, c.hits[:]);
		if status != .Ok || c.count == 0
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
	entasis.world_destroy(&c.world);
	c^ = {};
}
