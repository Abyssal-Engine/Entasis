package box_pile_batch

import "core:math"
import entasis "entasis:entasis"

STEPS :: 180;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [64]entasis.Body_Handle,
	tick: int,
	minimum_y: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.capacity.bodies = 64;
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	box_shape, floor_shape: entasis.Shape_Handle;
	box_shape, status = entasis.shape_add(&c.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	floor_shape, status = entasis.shape_add(&c.world, entasis.box(16, 1, 16));
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
	descriptions: [64]entasis.Body_Description;
	index: int = 0;
	for y in 0 ..< 4
	{
		for z in 0 ..< 4
		{
			for x in 0 ..< 4
			{
				descriptions[index] = entasis.body_dynamic(box_shape, inertia,
					entasis.pose({f32(x) - 1.5, f32(y) * 1.05 + 0.55, f32(z) - 1.5}),
					{}, entasis.body_activity(-1, 255));
				index += 1;
			}
		}
	}
	written: int;
	written, status = entasis.body_add_batch(&c.world, descriptions[:], c.bodies[:]);
	if status != .Ok
	{
		return status;
	}
	return .Ok if written == len(c.bodies) else .Invalid_Argument;
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
	c.minimum_y = math.F32_MAX;
	for body in c.bodies
	{
		state: entasis.Body_State;
		status: entasis.Status;
		state, status = entasis.body_get(&c.world, body);
		if status != .Ok || math.is_nan(state.pose.position.y)
		{
			return .Invalid_Argument;
		}
		c.minimum_y = min(c.minimum_y, state.pose.position.y);
	}
	return .Ok if c.tick == STEPS && c.minimum_y >= 0.2 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
