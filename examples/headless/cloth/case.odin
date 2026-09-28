package cloth

import "core:math"
import entasis "entasis:entasis"

WIDTH :: 6;
HEIGHT :: 6;
SPACING :: f32(0.5);
STEPS :: 240;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [WIDTH*HEIGHT]entasis.Body_Handle,
	links: [WIDTH*(HEIGHT-1)+(WIDTH-1)*HEIGHT][2]int,
	tick: int,
	maximum_error, bottom_y: f32,
}

index_of :: #force_inline proc "contextless" (x, y: int) -> int
{
	return y*WIDTH+x;
}

distance :: #force_inline proc "contextless" (a, b: entasis.Vector3) -> f32
{
	dx: f32 = b.x-a.x;
	dy: f32 = b.y-a.y;
	dz: f32 = b.z-a.z;
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
	inertia: entasis.Body_Inertia = {inverse_inertia_tensor={xx=10, yy=10, zz=10}, inverse_mass=4};
	for y in 0 ..< HEIGHT
	{
		for x in 0 ..< WIDTH
		{
			position: entasis.Vector3 = {f32(x)*SPACING-f32(WIDTH-1)*SPACING*0.5, 5-f32(y)*SPACING, 0};
			body_description: entasis.Body_Description;
			if y == 0
			{
				body_description = {pose=entasis.pose(position), local_inertia={}, activity=entasis.body_activity(-1, 255)};
			}
			else
			{
				body_description = entasis.body_shapeless(inertia, entasis.pose(position), {}, entasis.body_activity(-1, 255));
			}
			c.bodies[index_of(x, y)], status = entasis.body_add(&c.world, body_description);
			if status != .Ok
			{
				return status;
			}
		}
	}
	settings: entasis.Spring_Settings = entasis.spring_settings(70, 1);
	link: int;
	for y in 0 ..< HEIGHT
	{
		for x in 0 ..< WIDTH
		{
			if x+1 < WIDTH
			{
				c.links[link] = {index_of(x, y), index_of(x+1, y)};
				_, status = entasis.constraint_add_2(&c.world, c.bodies[c.links[link][0]], c.bodies[c.links[link][1]],
					entasis.Center_Distance_Constraint{target_distance=SPACING, spring_settings=settings});
				if status != .Ok
				{
					return status;
				}
				link += 1;
			}
			if y+1 < HEIGHT
			{
				c.links[link] = {index_of(x, y), index_of(x, y+1)};
				_, status = entasis.constraint_add_2(&c.world, c.bodies[c.links[link][0]], c.bodies[c.links[link][1]],
					entasis.Center_Distance_Constraint{target_distance=SPACING, spring_settings=settings});
				if status != .Ok
				{
					return status;
				}
				link += 1;
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
	for y in 0 ..< HEIGHT
	{
		for x in 0 ..< WIDTH
		{
			state: entasis.Body_State;
			status: entasis.Status;
			state, status = entasis.body_get(&c.world, c.bodies[index_of(x, y)]);
			if status != .Ok || (y == 0 && math.abs(state.pose.position.y-5) > 1e-5)
			{
				return .Invalid_Argument;
			}
			if x+1 < WIDTH
			{
				right: entasis.Body_State;
				right, _ = entasis.body_get(&c.world, c.bodies[index_of(x+1, y)]);
				c.maximum_error = max(c.maximum_error, math.abs(distance(state.pose.position, right.pose.position)-SPACING));
			}
			if y+1 < HEIGHT
			{
				below: entasis.Body_State;
				below, _ = entasis.body_get(&c.world, c.bodies[index_of(x, y+1)]);
				c.maximum_error = max(c.maximum_error, math.abs(distance(state.pose.position, below.pose.position)-SPACING));
			}
		}
	}
	bottom: entasis.Body_State;
	bottom, _ = entasis.body_get(&c.world, c.bodies[index_of(WIDTH/2, HEIGHT-1)]);
	c.bottom_y = bottom.pose.position.y;
	return .Ok if c.tick == STEPS && c.bottom_y < 3.5 && c.maximum_error <= 0.08 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
