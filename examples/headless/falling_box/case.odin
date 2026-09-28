package falling_box

import "core:math"
import entasis "entasis:entasis"

STEPS :: 180;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	tick: int,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.description = entasis.world_description_default();
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	box_shape: entasis.Shape_Handle;
	box_shape, status = entasis.shape_add(&c.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	floor_shape: entasis.Shape_Handle;
	floor_shape, status = entasis.shape_add(&c.world, entasis.box(20, 1, 20));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&c.world, box_shape, 1);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world,
		entasis.body_dynamic(box_shape, inertia, entasis.pose({0, 5, 0}), {}, entasis.body_activity(-1, 255)));
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
	if status != .Ok
	{
		return status;
	}
	if c.tick != STEPS || math.is_nan(state.pose.position.y) || state.pose.position.y < 0.25 || state.pose.position.y > 1.25
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
