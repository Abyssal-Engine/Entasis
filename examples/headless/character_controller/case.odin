package character_controller

import "core:math"
import entasis "entasis:entasis"

STEPS :: 40;
TIMESTEP :: f32(1.0 / 60.0);
Control :: enum
{
	Authored, Controlled,
}
Obstruction :: enum
{
	Clear, Blocked,
}
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	capsule: entasis.Shape_Handle,
	query_pose: entasis.Rigid_Pose,
	ground_hit, horizontal_hit: entasis.Sweep_Hit,
	ground_status, horizontal_status: entasis.Status,
	query_direction: entasis.Vector3,
	query_distance: f32,
	tick, grounded_frames: int,
	control: Control,
	obstruction: Obstruction,
	final_x: f32,
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
	c.capsule, status = entasis.shape_add(&c.world, entasis.capsule(0.4, 1.2));
	if status != .Ok
	{
		return status;
	}
	floor_shape, wall_shape: entasis.Shape_Handle;
	floor_shape, status = entasis.shape_add(&c.world, entasis.box(12, 1, 8));
	if status != .Ok
	{
		return status;
	}
	wall_shape, status = entasis.shape_add(&c.world, entasis.box(1, 4, 8));
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(wall_shape, entasis.pose({3, 1.5, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_kinematic(c.capsule,
		entasis.pose({0, 1.01, 0}), {}, entasis.body_activity(-1, 255)));
	return status;
}

case_step :: proc(c: ^Case, movement: entasis.Vector2 = {1, 0}) -> entasis.Status
{
	if movement.x != 1 || movement.y != 0
	{
		c.control = .Controlled;
	}
	state: entasis.Body_State;
	status: entasis.Status;
	state, status = entasis.body_get(&c.world, c.body);
	if status != .Ok
	{
		return status;
	}
	c.query_pose = state.pose;
	filter: entasis.Query_Filter = entasis.query_filter_mobility(entasis.COLLIDABLE_STATIC);
	c.ground_hit, c.ground_status = entasis.sweep_closest(&c.world, c.capsule, state.pose,
		entasis.velocity({0, -1, 0}), 0.08, filter);
	if c.ground_status == .Ok
	{
		c.grounded_frames += 1;
	}
	magnitude: f32 = math.sqrt(movement.x*movement.x+movement.y*movement.y);
	direction: entasis.Vector3 = {1, 0, 0};
	if magnitude > 0
	{
		direction = {movement.x/magnitude, 0, movement.y/magnitude};
	}
	travel: f32 = 0.1*min(magnitude, 1);
	c.query_direction, c.query_distance = direction, travel;
	c.horizontal_status = .Not_Found;
	c.horizontal_hit = {};
	if travel > 0
	{
		c.horizontal_hit, c.horizontal_status = entasis.sweep_closest(&c.world, c.capsule, state.pose,
			entasis.velocity(direction), travel, filter);
		if c.horizontal_status == .Ok
		{
			travel = max(c.horizontal_hit.sweep.t0-0.002, 0);
			c.obstruction = .Blocked;
		}
		else if c.horizontal_status != .Not_Found
		{
			return c.horizontal_status;
		}
	}
	state.pose.position.x += direction.x*travel;
	state.pose.position.z += direction.z*travel;
	status = entasis.body_set_pose(&c.world, c.body, state.pose);
	if status != .Ok
	{
		return status;
	}
	status = entasis.world_step(&c.world, TIMESTEP);
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
	c.final_x = state.pose.position.x;
	if c.control == .Authored && (c.grounded_frames < 30 || c.obstruction != .Blocked || c.final_x < 1.8 || c.final_x > 2.2)
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
