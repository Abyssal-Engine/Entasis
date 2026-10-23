package large_world_precision

import "core:fmt"
import e "entasis:entasis"

// the application owns the frame and shifts only between completed operations
main :: proc()
{
	world: e.World;
	description: e.World_Description = e.world_description_default();
	description.gravity = {};
	description.capacity = {bodies=8, statics=4, inactive_body_sets=4, shapes_per_type=4,
		constraints=16, initial_constraints_per_type_batch=4, minimum_constraints_per_body=4,
		broad_phase_candidates=32, pairs=32, inactive_pairs=16, pending_pairs_per_worker=8};
	if e.world_init(&world, description) != .Ok
	{
		panic("world initialization failed");
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(&world, e.sphere(0.5));
	if status != .Ok
	{
		panic("shape creation failed");
	}
	application: e.Global_Position = {1e9 + 0.002, -1e9, 1e9};
	frame: e.Origin_Frame;
	body: e.Body_Handle;
	shift: e.Vector3;
	when e.PRECISION_MODE == .Double_Coordinates
	{
		body, status = e.body_add_global(&world, e.body_kinematic(shape, e.pose()), application);
		shift = {1e9, -1e9, 1e9};
	}
	else
	{
		frame, status = e.origin_frame_at({1e9, -1e9, 1e9});
		if status != .Ok
		{
			panic("frame creation failed");
		}
		local: e.Vector3;
		local, status = e.origin_frame_to_local(frame, application);
		if status != .Ok
		{
			panic("frame conversion failed");
		}
		body, status = e.body_add(&world, e.body_kinematic(shape, e.pose(local)));
		shift = {128, 0, 0};
	}
	if status != .Ok
	{
		panic("body creation failed");
	}
	when e.PRECISION_MODE == .Double_Coordinates
	{
		query: e.Query_Context;
		if e.query_context_init(&query, &world) != .Ok
		{
			panic("query context creation failed");
		}
		stack: [64]i32;
		hit: e.Global_Ray_Hit;
		hit, status = e.ray_cast_global_closest_with_context(&query,
			{start={application.x - 2, application.y, application.z},
			 end={application.x + 2, application.y, application.z}}, stack[:]);
		if status != .Ok || abs(hit.location.x - (application.x - 0.5)) > 0.001
		{
			panic("high-coordinate query changed local geometry");
		}
		references: [1]e.Collidable_Reference = {hit.collidable};
		exported: [1]e.Global_Pose;
		completed: int;
		completed, status = e.world_export_global_poses(&world, references[:], exported[:]);
		if status != .Ok || completed != 1 || exported[0].position != application
		{
			panic("ordered high pose export changed the authored position");
		}
		if e.query_context_destroy(&query) != .Ok
		{
			panic("query context release failed");
		}
	}
	if e.world_shift_origin(&world, &frame, shift) != .Ok
	{
		panic("origin shift failed");
	}
	if e.world_step(&world, 1.0 / 60) != .Ok
	{
		panic("world step failed");
	}
	position: e.Global_Position;
	when e.PRECISION_MODE == .Double_Coordinates
	{
		pose: e.Global_Pose;
		pose, status = e.body_get_global_pose(&world, body);
		position = {frame.origin.x + pose.position.x, frame.origin.y + pose.position.y, frame.origin.z + pose.position.z};
	}
	else
	{
		state: e.Body_State;
		state, status = e.body_get(&world, body);
		if status == .Ok
		{
			position, status = e.origin_frame_to_global(frame, state.pose.position);
		}
	}
	if status != .Ok || abs(position.x - application.x) > 0.001 ||
		abs(position.y - application.y) > 0.001 || abs(position.z - application.z) > 0.001
	{
		panic("application position changed during maintenance");
	}
	fmt.printf("LARGE_WORLD_PRECISION_OK precision=%s application_position=%v origin=%v\n",
		e.PRECISION_NAME, position, frame.origin);
}
