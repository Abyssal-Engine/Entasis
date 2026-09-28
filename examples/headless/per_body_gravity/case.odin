package per_body_gravity

import entasis "entasis:entasis"

STEPS :: 60;
TIMESTEP :: f32(1.0 / 60.0);
GRAVITIES :: [2]entasis.Vector3{{0, -2, 0}, {0, -10, 0}};
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	gravities: entasis.Body_Property_Table(entasis.Vector3),
	narrow_policy: entasis.Default_Narrow_Policy,
	gravity_policy: entasis.Per_Body_Gravity_Policy,
	bodies: [2]entasis.Body_Handle,
	tick: int,
	slow_y, fast_y: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.body_property_init(&c.gravities, 2);
	if status != .Ok
	{
		return status;
	}
	c.narrow_policy = entasis.default_narrow_policy();
	c.gravity_policy = entasis.per_body_gravity_policy(&c.gravities);
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	status = entasis.world_description_set_callbacks(&description,
		entasis.narrow_policy_default(&c.narrow_policy), entasis.pose_policy_per_body(&c.gravity_policy));
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
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
	gravities: [2]entasis.Vector3 = GRAVITIES;
	for &body, index in c.bodies
	{
		body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
			entasis.pose({f32(index), 10, 0}), {}, entasis.body_activity(-1, 255)));
		if status != .Ok
		{
			return status;
		}
		status = entasis.body_property_set(&c.gravities, body, gravities[index]);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
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
	slow, fast: entasis.Body_State;
	status: entasis.Status;
	slow, status = entasis.body_get(&c.world, c.bodies[0]);
	if status != .Ok
	{
		return status;
	}
	fast, status = entasis.body_get(&c.world, c.bodies[1]);
	if status != .Ok
	{
		return status;
	}
	c.slow_y, c.fast_y = slow.pose.position.y, fast.pose.position.y;
	return .Ok if c.tick == STEPS && c.fast_y < c.slow_y - 2 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	entasis.body_property_destroy(&c.gravities);
	c^ = {};
}
