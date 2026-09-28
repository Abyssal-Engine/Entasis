package substepping

import "core:math"
import entasis "entasis:entasis"

STEPS_PER_CASE :: 30;
STEPS :: 2*STEPS_PER_CASE+1;
TIMESTEP :: f32(1.0 / 30.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [2]entasis.Body_Handle,
	cycle, tick: int,
	errors: [2]f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	description.solve = entasis.solve_description_substeps(1 if c.cycle == 0 else 4, 1, 64);
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
	c.bodies[0], status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose({0, 0, 0}), entasis.velocity({-15, 0, 0}), entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	c.bodies[1], status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose({3, 0, 0}), entasis.velocity({15, 0, 0}), entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.constraint_add_2(&c.world, c.bodies[0], c.bodies[1],
		entasis.Center_Distance_Constraint{target_distance=1, spring_settings=entasis.spring_settings(8, 0.7)});
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	if c.tick == STEPS_PER_CASE
	{
		if c.cycle != 0
		{
			return .Invalid_Argument;
		}
		entasis.world_destroy(&c.world);
		c.cycle, c.tick = 1, 0;
		return case_create(c);
	}
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status != .Ok
	{
		return status;
	}
	c.tick += 1;
	if c.tick == STEPS_PER_CASE
	{
		a, b: entasis.Body_State;
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
		c.errors[c.cycle] = math.abs(math.abs(b.pose.position.x - a.pose.position.x) - 1);
	}
	return .Ok;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	return .Ok if c.cycle == 1 && c.tick == STEPS_PER_CASE && c.errors[1] <= c.errors[0]+0.01 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
