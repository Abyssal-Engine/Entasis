package continuous_collision

import entasis "entasis:entasis"

STEPS :: 3;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	cycle, tick: int,
	positions: [2]f32,
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
	projectile_shape, wall_shape: entasis.Shape_Handle;
	projectile_shape, status = entasis.shape_add(&c.world, entasis.sphere(0.25));
	if status != .Ok
	{
		return status;
	}
	wall_shape, status = entasis.shape_add(&c.world, entasis.box(0.1, 4, 4));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(wall_shape, entasis.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, projectile_shape, 1);
	if status != .Ok
	{
		return status;
	}
	body_description: entasis.Body_Description = entasis.body_dynamic(projectile_shape, inertia,
		entasis.pose({-5, 0, 0}), entasis.velocity({600, 0, 0}), entasis.body_activity(-1, 255));
	continuity: entasis.Continuous_Detection = entasis.ccd_discrete() if c.cycle == 0 else entasis.ccd_continuous();
	// both cases retain contacts at the sweep pose with the same positive margin
	body_description.collidable = entasis.collidable(projectile_shape, continuity, 0, 0.1);
	c.body, status = entasis.body_add(&c.world, body_description);
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	if c.tick == 1
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
	state: entasis.Body_State;
	state, status = entasis.body_get(&c.world, c.body);
	c.positions[c.cycle] = state.pose.position.x;
	return status;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	return .Ok if c.cycle == 1 && c.tick == 1 && c.positions[0] > 1 && c.positions[1] < 1 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
