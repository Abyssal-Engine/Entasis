package main

import "core:fmt"
import e "entasis:entasis"

main :: proc()
{
	frame: e.Origin_Frame;
	status: e.Status;
	frame, status = e.origin_frame_at({1e9, -1e9, 5e8});
	if status != .Ok
	{
		panic("precision operation");
	}
	application: e.Global_Position = {1e9 + 0.002, -1e9 + 2, 5e8};
	local: e.Vector3;
	local, status = e.origin_frame_to_local(frame, application);
	if status != .Ok
	{
		panic("precision operation");
	}
	world: e.World;
	description: e.World_Description = e.world_description_default();
	description.gravity = {};
	description.threading.worker_count = 1;
	status = e.world_init(&world, description);
	if status != .Ok
	{
		panic("world initialization");
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, status = e.shape_add(&world, e.sphere(0.5));
	if status != .Ok
	{
		panic("precision operation");
	}
	body: e.Body_Handle;
	body, status = e.body_add(&world, e.body_kinematic(shape, e.pose(local), {}, e.body_activity(-1, 255)));
	if status != .Ok
	{
		panic("precision operation");
	}
	query: e.Query_Context;
	status = e.query_context_init(&query, &world);
	if status != .Ok
	{
		panic("query initialization");
	}
	defer e.query_context_destroy(&query);
	status = e.world_shift_origin(&world, &frame, {256, 0, 0});
	if status != .Ok
	{
		panic("origin shift");
	}
	state: e.Body_State;
	state, status = e.body_get(&world, body);
	if status != .Ok
	{
		panic("precision operation");
	}
	global: e.Global_Position;
	global, status = e.origin_frame_to_global(frame, state.pose.position);
	if status != .Ok
	{
		panic("precision operation");
	}
	hit: e.Ray_Hit;
	hit, status = e.ray_cast_closest_with_context(&query, e.ray({state.pose.position.x - 2, state.pose.position.y, state.pose.position.z}, {1, 0, 0}, 4));
	if status != .Ok
	{
		panic("precision operation");
	}
	handle: e.Body_Handle;
	handle, status = e.collidable_body_handle(hit.collidable);
	if status != .Ok
	{
		panic("precision operation");
	}
	fmt.printf("ORIGIN_PRECISION_VALUES x=%.9f y=%.9f origin=%.9f handle=%d\n", global.x, global.y, frame.origin.x, handle.value);
	fmt.println("ORIGIN_PRECISION_OK");
}
