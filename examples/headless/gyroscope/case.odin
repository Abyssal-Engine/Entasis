package gyroscope

import "core:math"
import entasis "entasis:entasis"

STEPS_PER_CASE :: 120;
STEPS :: 2*STEPS_PER_CASE+1;
TIMESTEP :: f32(1.0 / 240.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	narrow_policy: entasis.Default_Narrow_Policy,
	pose_policy: entasis.Uniform_Gravity_Policy,
	body: entasis.Body_Handle,
	cycle, tick: int,
	velocities: [2]entasis.Body_Velocity,
	delta: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.narrow_policy = entasis.default_narrow_policy();
	c.pose_policy = entasis.uniform_gravity_policy({}, 0, 0);
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	mode: entasis.Angular_Integration_Mode = .Nonconserving if c.cycle == 0 else .Conserve_Momentum_With_Gyroscopic_Torque;
	status: entasis.Status = entasis.world_description_set_callbacks(&description,
		entasis.narrow_policy_default(&c.narrow_policy), entasis.pose_policy_uniform_mode(&c.pose_policy, mode));
	if status != .Ok
	{
		return status;
	}
	c.description = description;
	status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_inertia(entasis.box(1, 2, 4), 1);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose(), entasis.velocity({}, {0.7, 1.1, 2.3}), entasis.body_activity(-1, 255)));
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
		state: entasis.Body_State;
		state, status = entasis.body_get(&c.world, c.body);
		c.velocities[c.cycle] = state.velocity;
	}
	return status;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	c.delta = abs(c.velocities[1].angular.x-c.velocities[0].angular.x) +
		abs(c.velocities[1].angular.y-c.velocities[0].angular.y) +
		abs(c.velocities[1].angular.z-c.velocities[0].angular.z);
	return .Ok if c.cycle == 1 && c.tick == STEPS_PER_CASE && c.delta >= 0.01 && !math.is_nan(f64(c.delta)) else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
