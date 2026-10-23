package precision_callbacks_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_callbacks_follow_growth_sleep_compaction_handle_reuse_and_shift :: proc(t: ^testing.T)
{
	capture: Capture;
	scene: Scene;
	scene_initialize(t, &scene, capacity=4, workers=4, callbacks=capture_callbacks(&capture));
	defer scene_dispose(t, &scene);
	bodies: [65]e.Body_Handle;
	positions: [65]e.Global_Position;
	for index in 0 ..< 65
	{
		positions[index] = {1e9 + f64(index) * 4 + 0.001953125, -1e9 + 0.125, 1e9};
		bodies[index] = add_body(t, &scene, positions[index], scene.sphere);
	}
	for index := 2; index < 65; index += 2
	{
		sleep_body(t, &scene, bodies[index]);
	}
	testing.expect_value(t, e.body_remove(&scene.world, bodies[0]), e.Status.Ok);
	positions[0] = {1e9 + 1024.001953125, -1e9 + 0.25, 1e9 - 0.125};
	reused: e.Body_Handle = add_body(t, &scene, positions[0], scene.sphere);
	testing.expect_value(t, reused, bodies[0]);
	testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
	for index in 0 ..< 65
	{
		if index > 0 && index % 2 == 0
		{
			testing.expect_value(t, capture.counts[bodies[index].value], 0);
		}
		else
		{
			testing.expect_value(t, capture.counts[bodies[index].value], 2);
			testing.expect_value(t, capture.last[bodies[index].value], positions[index]);
		}
		testing.expect_value(t, e.body_set_velocity(&scene.world, bodies[index], {}), e.Status.Ok);
	}
	testing.expect_value(t, p.simulation_translate_origin(scene.simulation, {1e9, -1e9, 1e9}), e.Status.Ok);
	capture.counts = {};
	testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
	for index in 0 ..< 65
	{
		testing.expect_value(t, capture.counts[bodies[index].value], 2);
		testing.expect_value(t, capture.last[bodies[index].value],
			e.Global_Position{positions[index].x - 1e9, positions[index].y + 1e9, positions[index].z - 1e9});
	}
	for worker in 0 ..< 4
	{
		testing.expect_value(t, capture.invalid_lanes[worker], 0);
	}
}
