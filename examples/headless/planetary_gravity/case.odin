package planetary_gravity

import entasis "entasis:entasis"

STEPS :: 1;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	narrow_policy: entasis.Default_Narrow_Policy,
	gravity_policy: entasis.Planetary_Gravity_Policy,
	body: entasis.Body_Handle,
	tick: int,
	final_velocity: entasis.Vector3,
	initial_velocity: entasis.Vector3,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.narrow_policy = entasis.default_narrow_policy();
	c.gravity_policy = entasis.planetary_gravity_policy({}, 1200);
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	status: entasis.Status = entasis.world_description_set_callbacks(&description,
		entasis.narrow_policy_default(&c.narrow_policy), entasis.pose_policy_planetary(&c.gravity_policy));
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
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor = {xx = 1, yy = 1, zz = 1}, inverse_mass = 1};
	c.initial_velocity = {0, 7, 0};
	c.body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
		entasis.pose({20, 0, 0}), entasis.velocity(c.initial_velocity), entasis.body_activity(-1, 255)));
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
	state: entasis.Body_State;
	status: entasis.Status;
	state, status = entasis.body_get(&c.world, c.body);
	if status != .Ok || c.tick != STEPS || state.velocity.linear.x >= 0 || state.velocity.linear.y <= 0
	{
		return .Invalid_Argument;
	}
	c.final_velocity = state.velocity.linear;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
