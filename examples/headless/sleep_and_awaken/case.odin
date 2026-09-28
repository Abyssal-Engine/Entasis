package sleep_and_awaken

import entasis "entasis:entasis"

STEPS :: 9;
TIMESTEP :: f32(1.0/60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	tick: int,
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
	c.body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia, entasis.pose(), {}, entasis.body_activity(1, 1)));
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status;
	if c.tick<8
	{
		status = entasis.world_step(&c.world, TIMESTEP);
	}
	else
	{
		status = entasis.body_awaken(&c.world, c.body);
	}
	if status != .Ok
	{
		return status;
	}
	c.tick += 1;
	if c.tick==8 || c.tick==9
	{
		state: entasis.Body_Activation_State;
		state, status = entasis.body_activation_state(&c.world, c.body);
		if status != .Ok
		{
			return status;
		}
		expected: entasis.Body_Activation_State = .Sleeping if c.tick==8 else .Active;
		if state!=expected
		{
			return .Invalid_Argument;
		}
	}
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
