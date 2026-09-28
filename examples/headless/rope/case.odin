package rope

import "core:math"
import entasis "entasis:entasis"

SEGMENT_COUNT :: 16;
SEGMENT_LENGTH :: f32(0.45);
STEPS :: 300;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [SEGMENT_COUNT]entasis.Body_Handle,
	tick, constraint_count: int,
	maximum_error, end_y: f32,
}

body_distance :: #force_inline proc "contextless" (a, b: entasis.Body_State) -> f32
{
	dx: f32 = b.pose.position.x-a.pose.position.x;
	dy: f32 = b.pose.position.y-a.pose.position.y;
	dz: f32 = b.pose.position.z-a.pose.position.z;
	return math.sqrt(dx*dx+dy*dy+dz*dz);
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.solve = entasis.solve_description_substeps(4, 2, 64);
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=8, yy=8, zz=8}, inverse_mass=3};
	for index in 0 ..< SEGMENT_COUNT
	{
		position: entasis.Vector3 = {0, 8-f32(index)*SEGMENT_LENGTH, 0};
		body_description: entasis.Body_Description;
		if index == 0
		{
			body_description = {pose=entasis.pose(position), local_inertia={}, activity=entasis.body_activity(-1, 255)};
		}
		else
		{
			body_description = entasis.body_shapeless(inertia, entasis.pose(position), {}, entasis.body_activity(-1, 255));
		}
		c.bodies[index], status = entasis.body_add(&c.world, body_description);
		if status != .Ok
		{
			return status;
		}
		if index > 0
		{
			_, status = entasis.constraint_add_2(&c.world, c.bodies[index-1], c.bodies[index],
				entasis.Center_Distance_Constraint{target_distance=SEGMENT_LENGTH, spring_settings=entasis.spring_settings(55, 1)});
			if status != .Ok
			{
				return status;
			}
			_, status = entasis.constraint_add_2(&c.world, c.bodies[index-1], c.bodies[index],
				entasis.Twist_Limit{local_basis_a={w=1}, local_basis_b={w=1}, minimum_angle=-0.25, maximum_angle=0.25,
					spring_settings=entasis.spring_settings(35, 1)});
			if status != .Ok
			{
				return status;
			}
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
	c.maximum_error = 0;
	for index in 1 ..< SEGMENT_COUNT
	{
		previous, current: entasis.Body_State;
		previous_status, current_status: entasis.Status;
		previous, previous_status = entasis.body_get(&c.world, c.bodies[index-1]);
		current, current_status = entasis.body_get(&c.world, c.bodies[index]);
		if previous_status != .Ok || current_status != .Ok
		{
			return .Invalid_Argument;
		}
		c.maximum_error = max(c.maximum_error, math.abs(body_distance(previous, current)-SEGMENT_LENGTH));
	}
	anchor, end: entasis.Body_State;
	anchor, _ = entasis.body_get(&c.world, c.bodies[0]);
	end, _ = entasis.body_get(&c.world, c.bodies[SEGMENT_COUNT-1]);
	c.end_y = end.pose.position.y;
	infos: [64]entasis.Constraint_Info;
	required: int;
	status: entasis.Status;
	c.constraint_count, required, status = entasis.constraint_enumerate(&c.world, infos[:]);
	if math.abs(anchor.pose.position.y-8) > 1e-5 || c.end_y >= anchor.pose.position.y-5 || c.maximum_error > 0.05 ||
		status != .Ok || c.constraint_count < (SEGMENT_COUNT-1)*2 || required < (SEGMENT_COUNT-1)*2
	{
		return .Invalid_Argument;
	}
	return .Ok if c.tick == STEPS else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
