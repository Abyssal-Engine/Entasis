package precision_queries_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_distance_target_copies_survive_sleep_compaction_shift_and_reuse :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	positions: [4]e.Global_Position = {
		{1e9 + 0.125, -1e9 + 0.25, 1e9 + 0.375},
		{1e9 + 4.125, -1e9 + 0.25, 1e9 + 0.375},
		{1e9 + 8.125, -1e9 + 0.25, 1e9 + 0.375},
		{1e9 + 12.125, -1e9 + 0.25, 1e9 + 0.375},
	};
	removed: p.Body_Handle = add_body(t, &scene, shape, {1e9 - 4, -1e9, 1e9});
	bodies: [2]p.Body_Handle = {add_body(t, &scene, shape, positions[0]), add_body(t, &scene, shape, positions[1], .Kinematic)};
	stationary: p.Static_Handle = add_static(t, &scene, shape, positions[2]);
	shapeless: p.Body_Handle = add_body(t, &scene, {}, positions[3]);
	references: [4]e.Collidable_Reference;
	references[0], _ = p.collidable_reference_create(.Dynamic, int(bodies[0].value));
	references[1], _ = p.collidable_reference_create(.Kinematic, int(bodies[1].value));
	references[2], _ = p.collidable_reference_create(.Static, int(stationary.value));
	references[3], _ = p.collidable_reference_create(.Dynamic, int(shapeless.value));
	copies: [4]e.Global_Pose;
	shapes: [4]e.Shape_Handle;
	for ref, i in references
	{
		status: e.Status;
		shapes[i], copies[i], status = e.collidable_shape_global_pose_with_context(&scene.query, ref);
		testing.expect_value(t, status, e.Status.Ok);
		expect_position(t, copies[i].position, positions[i], 0);
		testing.expect_value(t, shapes[i], shape if i < 3 else e.Shape_Handle{});
	}
	testing.expect_value(t, e.body_remove(&scene.world, removed), e.Status.Ok);
	sleep_body(t, &scene, bodies[0]);
	frame: e.Origin_Frame;
	for phase in 0 ..< 3
	{
		if phase == 1
		{
			testing.expect_value(t, e.world_shift_origin(&scene.world, &frame, {1e9, -1e9, 1e9}), e.Status.Ok);
		}
		if phase == 2
		{
			for index in 0 ..< 64
			{
				_ = add_body(t, &scene, shape, {128 + 4*f64(index), 32, 0});
			}
		}
		before: p.Body_Memory_Location;
		before, _ = p.bodies_resolve(&scene.simulation.bodies, bodies[0]);
		testing.expect(t, before.set_index > 0);
		for ref, i in references
		{
			target_shape: e.Shape_Handle;
			pose: e.Global_Pose;
			status: e.Status;
			target_shape, pose, status = e.collidable_shape_global_pose_with_context(&scene.query, ref);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, target_shape, shapes[i]);
			expect_position(t, {pose.position.x + frame.origin.x, pose.position.y + frame.origin.y, pose.position.z + frame.origin.z}, positions[i], 0);
			// value copies made before the mutation must not change with storage
			expect_position(t, copies[i].position, positions[i], 0);
			if i < 3
			{
				point: e.Global_Position = pose.position;
				point.x += 0.502;
				result: e.Global_Shape_Distance_Result;
				result, status = e.shape_closest_point_global_with_context(&scene.query, point, target_shape, pose);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect(t, abs(result.geometry.distance - 0.002) < 0.000002);
				expect_position(t, result.geometry.point_b, {pose.position.x + 0.5, pose.position.y, pose.position.z});
			}
		}
		after: p.Body_Memory_Location;
		after, _ = p.bodies_resolve(&scene.simulation.bodies, bodies[0]);
		testing.expect_value(t, after, before);
	}
	// removed references fail until the original handle is actually reused
	testing.expect_value(t, e.body_remove(&scene.world, bodies[1]), e.Status.Ok);
	_, missing_pose, missing_status := e.collidable_shape_global_pose_with_context(&scene.query, references[1]);
	testing.expect(t, missing_status != .Ok);
	testing.expect_value(t, missing_pose, e.Global_Pose{});
	replacement: p.Body_Handle = add_body(t, &scene, shape, {27.125, 0.25, 0.375}, .Kinematic);
	testing.expect_value(t, replacement, bodies[1]);
	_, current, current_status := e.collidable_shape_global_pose_with_context(&scene.query, references[1]);
	testing.expect_value(t, current_status, e.Status.Ok);
	expect_position(t, current.position, {27.125, 0.25, 0.375}, 0);
	expect_position(t, copies[1].position, positions[1], 0);
	_, missing_pose, missing_status = e.collidable_shape_global_pose_with_context(&scene.query, {packed=0xC0000000});
	testing.expect_value(t, missing_status, e.Status.Invalid_Argument);
	testing.expect_value(t, missing_pose, e.Global_Pose{});
}
