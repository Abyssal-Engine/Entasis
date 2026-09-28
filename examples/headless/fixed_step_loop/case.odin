package fixed_step_loop

import "core:math"
import entasis "entasis:entasis"

STEPS :: 4;
TIMESTEP :: f32(1.0/60.0);
FRAME_TIMES :: [4]f32{1.0/120.0, 1.0/120.0, 1.0/30.0, 1.0/60.0};
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	stepper: entasis.Fixed_Stepper,
	tick, total_steps, last_steps: int,
	alpha: f32,
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
	c.body, status = entasis.body_add(&c.world, entasis.body_shapeless(
		{inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1}, entasis.pose(),
		entasis.velocity({3, 0, 0}), entasis.body_activity(-1, 255)));
	c.stepper = entasis.fixed_stepper(TIMESTEP, 4);
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	frame_times: [4]f32 = FRAME_TIMES;
	status: entasis.Status;
	c.last_steps, c.alpha, status = entasis.fixed_stepper_update(&c.stepper, &c.world, frame_times[c.tick]);
	if status != .Ok
	{
		return status;
	}
	c.total_steps += c.last_steps;
	c.tick += 1;
	return .Ok;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	state: entasis.Body_State;
	status: entasis.Status;
	state, status = entasis.body_get(&c.world, c.body);
	if status != .Ok
	{
		return status;
	}
	if c.total_steps!=4 || math.abs(c.alpha)>1e-4 || math.abs(state.pose.position.x-0.2)>0.01
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
