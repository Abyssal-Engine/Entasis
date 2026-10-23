package precision_mutations_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"

@(test)
precision_global_force_uses_authoritative_center_and_explicit_point :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for substeps in ([2]i32{1, 4})
		{
			scene: Scene;
			scene_initialize(t, &scene, 32, substeps);
			position: e.Global_Position = {origin, origin + 0.001953125, -origin};
			body: e.Body_Handle = add_body(t, &scene, position, e.Shape_Handle{});
			testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
			point: e.Global_Position = {position.x, position.y + 0.001953125, position.z};
			testing.expect_value(t, e.body_add_force_at_global_position(&scene.world, body, {4, 0, 0}, point), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			state: e.Body_State;
			state, _ = e.body_get(&scene.world, body);
			testing.expect(t, abs(state.velocity.linear.x - 1) < 1e-6);
			testing.expect(t, abs(state.velocity.angular.z + 0.001953125) < 1e-7);
			// a legacy point is deliberately authored at the integer origin.
			// its lever must still subtract the high center, not the cache
			testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {position, {w=1}}), e.Status.Ok);
			testing.expect_value(t, e.body_set_velocity(&scene.world, body, {}), e.Status.Ok);
			testing.expect_value(t, e.body_add_force_at_position(&scene.world, body, {4, 0, 0}, {f32(origin), f32(origin), f32(-origin)}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			state, _ = e.body_get(&scene.world, body);
			testing.expect(t, abs(state.velocity.angular.z - 0.001953125) < 1e-7);
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_global_force_invalid_points_do_not_queue_or_awaken :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, e.Shape_Handle{});
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	sleep_body(t, &scene, body);
	set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
	for point in ([3]e.Global_Position{{math.nan_f64(), 0, 0}, {math.inf_f64(1), 0, 0}, {1e40, 0, 0}})
	{
		testing.expect_value(t, e.body_add_force_at_global_position(&scene.world, body, {1, 0, 0}, point), e.Status.Invalid_Argument);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
	}
	testing.expect_value(t, e.body_add_force_at_global_position(&scene.world, body, {0, 0, 0}, position), e.Status.Ok);
	testing.expect_value(t, e.body_set_velocity(&scene.world, body, {}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
	state: e.Body_State;
	state, _ = e.body_get(&scene.world, body);
	testing.expect_value(t, state.velocity, e.Body_Velocity{});
	expect_body_position(t, &scene, body, position);
	scene_dispose(t, &scene);
}

@(test)
precision_global_kinematic_target_motion_expiration_and_property_cancellation :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for substeps in ([2]i32{1, 4})
		{
			scene: Scene;
			scene_initialize(t, &scene, 32, substeps);
			position: e.Global_Position = {origin + 0.125, -origin + 0.25, origin};
			body: e.Body_Handle;
			status: e.Status;
			body, status = e.body_add_global(&scene.world, e.body_kinematic(e.Shape_Handle{}, e.pose(), {}, e.body_activity(-1, 255)), position);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
			target: e.Global_Pose = {{position.x + 0.001953125, position.y - 0.00390625, position.z + 0.0078125}, {w=1}};
			testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, body, target), e.Status.Ok);
			read: e.Global_Pose;
			read, status = e.body_get_global_kinematic_target(&scene.world, body);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, read, target);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			expect_body_position(t, &scene, body, target.position);
			_, status = e.body_get_global_kinematic_target(&scene.world, body);
			testing.expect_value(t, status, e.Status.Not_Found);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			expect_body_position(t, &scene, body, target.position);
			target.position.x += 0.5;
			testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, body, target), e.Status.Ok);
			testing.expect_value(t, e.body_set_activity(&scene.world, body, e.body_activity(-1, 255)), e.Status.Ok);
			_, status = e.body_get_global_kinematic_target(&scene.world, body);
			testing.expect_value(t, status, e.Status.Not_Found);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			target.position.x -= 0.5;
			expect_body_position(t, &scene, body, target.position);
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_global_target_validation_read_sleep_and_explicit_float_replacement :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9};
	body: e.Body_Handle;
	body, _ = e.body_add_global(&scene.world, e.body_kinematic(e.Shape_Handle{}, e.pose(), {}, e.body_activity(-1, 255)), position);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	sleep_body(t, &scene, body);
	target: e.Global_Pose = {{position.x + 0.001953125, position.y, position.z}, {w=1}};
	testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, body, target, .Preserve_Sleep), e.Status.Ok);
	set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
	testing.expect(t, set_index > 0);
	read: e.Global_Pose;
	status: e.Status;
	read, status = e.body_get_global_kinematic_target(&scene.world, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, read, target);
	testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
	testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, body, {{1e40, 0, 0}, {w=1}}), e.Status.Invalid_Argument);
	read, status = e.body_get_global_kinematic_target(&scene.world, body);
	testing.expect_value(t, read, target);
	testing.expect_value(t, e.body_set_kinematic_target(&scene.world, body, e.pose({1e9, -1e9, 1e9})), e.Status.Ok);
	read, status = e.body_get_global_kinematic_target(&scene.world, body);
	testing.expect_value(t, read.position, e.Global_Position{1e9, -1e9, 1e9});
	testing.expect_value(t, e.body_clear_kinematic_target(&scene.world, body), e.Status.Ok);
	_, status = e.body_get_global_kinematic_target(&scene.world, body);
	testing.expect_value(t, status, e.Status.Not_Found);
	expect_body_position(t, &scene, body, position);
	scene_dispose(t, &scene);
}
