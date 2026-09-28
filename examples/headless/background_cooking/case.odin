package background_cooking

import "core:thread"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"

STEPS :: 1;
Cook_Job :: struct
{
	points: []entasis.Vector3,
	ctx: cooking.Cooking_Context,
	cooked: cooking.Cooked_Hull,
	status: entasis.Status,
}
Case :: struct
{
	points: [8]entasis.Vector3,
	job: Cook_Job,
	world: entasis.World,
	description: entasis.World_Description,
	static: entasis.Static_Handle,
	ray: entasis.Ray,
	hit: entasis.Ray_Hit,
	independent_checksum: u64,
	tick: int,
}

cook_worker :: proc(data: rawptr)
{
	job: ^Cook_Job = (^Cook_Job)(data);
	job.status = cooking.cooking_context_init(&job.ctx);
	if job.status != .Ok
	{
		return;
	}
	job.cooked, job.status = cooking.cook_hull(&job.ctx, job.points);
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.points = {
		{-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {-1, 1, 1}, {1, 1, 1},
	};
	c.job.points = c.points[:];
	worker: ^thread.Thread = thread.create_and_start_with_data(&c.job, cook_worker);
	if worker == nil
	{
		return .Capacity_Missing;
	}
	for value in u64(0) ..< 100000
	{
		c.independent_checksum = c.independent_checksum*6364136223846793005+value;
	}
	thread.join(worker);
	thread.destroy(worker);
	if c.job.status != .Ok
	{
		return c.job.status;
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
	shape: entasis.Shape_Handle;
	status: entasis.Status;
	shape, status = cooking.cooked_hull_import(&c.world, &c.job.cooked);
	if status != .Ok
	{
		return status;
	}
	c.static, status = entasis.static_add(&c.world, entasis.static_body(shape, entasis.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	c.ray = entasis.ray({-3, 0, 0}, {1, 0, 0}, 6);
	c.hit, status = entasis.ray_cast_closest(&c.world, c.ray);
	if status != .Ok || c.hit.t < 1.9 || c.hit.t > 2.1
	{
		return .Invalid_Argument;
	}
	c.tick = 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	cooking.cooked_hull_destroy(&c.job.cooked);
	cooking.cooking_context_destroy(&c.job.ctx);
	c^ = {};
}
