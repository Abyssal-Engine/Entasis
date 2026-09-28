package kinematic_platform

import entasis "entasis:entasis"

STEPS :: 60;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [2]entasis.Body_Handle,
	tick: int,
	platform_y, passenger_y: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.description = entasis.world_description_default();
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	platform_shape, passenger_shape: entasis.Shape_Handle;
	platform_shape, status = entasis.shape_add(&c.world, entasis.box(4, 0.5, 4));
	if status != .Ok
	{
		return status;
	}
	passenger_shape, status = entasis.shape_add(&c.world, entasis.box(0.75, 0.75, 0.75));
	if status != .Ok
	{
		return status;
	}
	c.bodies[0], status = entasis.body_add(&c.world, entasis.body_kinematic(platform_shape,
		entasis.pose({0, 0, 0}), entasis.velocity({0, 1, 0}), entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, passenger_shape, 1);
	if status != .Ok
	{
		return status;
	}
	c.bodies[1], status = entasis.body_add(&c.world, entasis.body_dynamic(passenger_shape, inertia,
		entasis.pose({0, 0.65, 0}), {}, entasis.body_activity(-1, 255)));
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status == .Ok
	{
		c.tick += 1;
	}
	return status;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	platform, passenger: entasis.Body_State;
	status: entasis.Status;
	platform, status = entasis.body_get(&c.world, c.bodies[0]);
	if status != .Ok
	{
		return status;
	}
	passenger, status = entasis.body_get(&c.world, c.bodies[1]);
	if status != .Ok
	{
		return status;
	}
	c.platform_y, c.passenger_y = platform.pose.position.y, passenger.pose.position.y;
	return .Ok if c.tick == STEPS && c.platform_y >= 0.9 && c.passenger_y >= 1.1 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
