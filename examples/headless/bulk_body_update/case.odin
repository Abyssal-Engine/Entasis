package bulk_body_update

import entasis "entasis:entasis"

STEPS :: 1;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [128]entasis.Body_Handle,
	tick: int,
	checksum: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	description.capacity.bodies = 128;
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor = {xx = 1, yy = 1, zz = 1}, inverse_mass = 1};
	for index in 0 ..< len(c.bodies)
	{
		c.bodies[index], status = entasis.body_add(&c.world,
			entasis.body_shapeless(inertia, entasis.pose({0, f32(index), 0}), {}, entasis.body_activity(-1, 255)));
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	view, fresh: entasis.Active_Body_View;
	status: entasis.Status;
	view, status = entasis.active_body_view(&c.world);
	if status != .Ok || view.count != 128 || !entasis.body_view_valid(&c.world, view)
	{
		return .Invalid_Argument;
	}
	for index in 0 ..< view.count
	{
		view.dynamics[index].motion.velocity.linear.x = f32(index + 1) * 0.01;
	}
	status = entasis.world_step(&c.world, TIMESTEP);
	if status != .Ok
	{
		return status;
	}
	if entasis.body_view_valid(&c.world, view)
	{
		return .Invalid_Argument;
	}
	fresh, status = entasis.active_body_view(&c.world);
	if status != .Ok
	{
		return status;
	}
	c.checksum = 0;
	for index in 0 ..< fresh.count
	{
		c.checksum += fresh.dynamics[index].motion.pose.position.x;
	}
	if c.checksum <= 1
	{
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
