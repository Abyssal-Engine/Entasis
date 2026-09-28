package constraints

import "core:math"
import entasis "entasis:entasis"

STEPS :: 120;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [2]entasis.Body_Handle,
	tick: int,
	error: f32,
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
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
	c.bodies[0], status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose({0, 0, 0}), entasis.velocity({-2, 0, 0}), entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	c.bodies[1], status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose({4, 0, 0}), entasis.velocity({2, 0, 0}), entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.constraint_add_2(&c.world, c.bodies[0], c.bodies[1],
		entasis.Center_Distance_Constraint{target_distance=2, spring_settings=entasis.spring_settings(30, 1)});
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
	a, b: entasis.Body_State;
	status: entasis.Status;
	a, status = entasis.body_get(&c.world, c.bodies[0]);
	if status != .Ok
	{
		return status;
	}
	b, status = entasis.body_get(&c.world, c.bodies[1]);
	if status != .Ok
	{
		return status;
	}
	c.error = math.abs(math.abs(b.pose.position.x - a.pose.position.x) - 2);
	return .Ok if c.tick == STEPS && c.error <= 0.05 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
