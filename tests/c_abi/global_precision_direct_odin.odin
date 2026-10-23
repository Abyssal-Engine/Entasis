package main

import "core:fmt"
import e "entasis:entasis"

main :: proc()
{
	world: e.World;
	description: e.World_Description = e.world_description_default();
	description.gravity = {};
	description.threading.worker_count = 1;
	if e.world_init(&world, description) != .Ok
	{
		panic("world initialization");
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(&world, e.sphere(0.5));
	if status != .Ok
	{
		panic("precision operation");
	}
	positions: [2]e.Global_Position = {{1e9 + 0.002, 3, 0}, {1e9 + 0.004, 3, 0}};
	bodies: [2]e.Body_Handle;
	for index in 0 ..< len(bodies)
	{
		bodies[index], status = e.body_add_global(&world, e.body_kinematic(shape, e.pose(), {}, e.body_activity(-1, 255)), positions[index]);
		if status != .Ok
		{
			panic("precision operation");
		}
	}
	first: e.Global_Pose;
	second: e.Global_Pose;
	first, status = e.body_get_global_pose(&world, bodies[0]);
	if status != .Ok
	{
		panic("precision operation");
	}
	second, status = e.body_get_global_pose(&world, bodies[1]);
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
	stack: [128]i32;
	hit: e.Global_Ray_Hit;
	hit, status = e.ray_cast_global_closest_with_context(&query, {{positions[0].x - 2, 3, 0}, {positions[0].x + 2, 3, 0}}, stack[:]);
	if status != .Ok
	{
		panic("precision operation");
	}
	distance: e.Global_Shape_Distance_Result;
	distance, status = e.shape_distance_global_with_context(&query, shape, {position=positions[0], orientation={w=1}},
		shape, {position={positions[0].x + 2.002, 3, 0}, orientation={w=1}});
	if status != .Ok
	{
		panic("precision operation");
	}
	fmt.printf("GLOBAL_PRECISION_VALUES body=%.9f gap=%.9f hit=%.9f a=%.9f b=%.9f distance=%.9f\n", first.position.x, second.position.x - first.position.x,
		hit.location.x, distance.geometry.point_a.x, distance.geometry.point_b.x, f64(distance.geometry.distance));
	fmt.println("GLOBAL_PRECISION_OK");
}
