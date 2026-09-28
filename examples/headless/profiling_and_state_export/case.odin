package profiling_and_state_export

import entasis "entasis:entasis"

STEPS :: 10;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [64]entasis.Body_Handle,
	profile: entasis.Profile_Snapshot,
	tick: int,
	checksum: f64,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	description.profiling = true;
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1};
	for &body, index in c.bodies
	{
		body, status = entasis.body_add(&c.world, entasis.body_shapeless(inertia,
			entasis.pose({f32(index), 0, 0}), entasis.velocity({0.01*f32(index+1), 0, 0}), entasis.body_activity(-1, 255)));
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
	status: entasis.Status;
	c.profile, status = entasis.world_profile_snapshot(&c.world);
	if status != .Ok || c.profile.step_index != 10 || c.profile.trace_count == 0 ||
		c.profile.stage_counts[entasis.Profile_Stage.Timestep] == 0 ||
		c.profile.stage_durations_nanoseconds[entasis.Profile_Stage.Timestep] <= 0
	{
		return .Invalid_Argument;
	}
	c.checksum = 0;
	for body in c.bodies
	{
		state: entasis.Body_State;
		state, status = entasis.body_get(&c.world, body);
		if status != .Ok
		{
			return status;
		}
		c.checksum += f64(state.pose.position.x)*f64(body.value+1);
	}
	return .Ok if c.tick == STEPS else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
