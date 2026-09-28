package convex_hulls

import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"

STEPS :: 1;
Case :: struct
{
	ctx: cooking.Cooking_Context,
	cooked: cooking.Cooked_Hull,
	world: entasis.World,
	description: entasis.World_Description,
	shape: entasis.Shape_Handle,
	tick: int,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = cooking.cooking_context_init(&c.ctx);
	if status != .Ok
	{
		return status;
	}
	points: [8]entasis.Vector3 = {
		{-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1},
	};
	c.cooked, status = cooking.cook_hull(&c.ctx, points[:]);
	if status != .Ok
	{
		return status;
	}
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	c.description = description;
	return entasis.world_init(&c.world, c.description);
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	if c.tick != 0
	{
		return .Invalid_Argument;
	}
	status: entasis.Status;
	c.shape, status = cooking.cooked_hull_import(&c.world, &c.cooked);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, c.shape, 1);
	if status != .Ok || inertia.inverse_mass != 1
	{
		return .Invalid_Argument;
	}
	c.tick = 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	cooking.cooked_hull_destroy(&c.cooked);
	cooking.cooking_context_destroy(&c.ctx);
	c^ = {};
}
