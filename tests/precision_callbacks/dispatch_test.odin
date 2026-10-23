package precision_callbacks_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_callback_masked_dispatch_preserves_two_millimeter_positions :: proc(t: ^testing.T)
{
	capture: Capture;
	scene: Scene;
	scene_initialize(t, &scene, callbacks=capture_callbacks(&capture));
	defer scene_dispose(t, &scene);
	bodies: [8]e.Body_Handle;
	for lane in 0 ..< 8
	{
		bodies[lane] = add_body(t, &scene, {1e9 + f64(lane + 1) / 512, -1e9 - f64(lane) / 1024, 1e9 + 0.125}, {});
	}
	for bits in 0 ..< 256
	{
		indices: [8]i32;
		masks: [8]i32;
		capture.counts = {};
		for lane in 0 ..< 8
		{
			indices[lane] = -2147483647;
			if bits & (1 << uint(lane)) != 0
			{
				indices[lane] = i32(lane);
				masks[lane] = -1;
			}
		}
		velocity: p.Body_Velocity_Wide;
		p.pose_integrator_invoke_velocity(&scene.simulation.integrator,
			transmute(util.I32x8)indices, {}, {}, {}, transmute(util.I32x8)masks, 0, util.F32x8(0.25), &velocity);
		for lane in 0 ..< 8
		{
			if masks[lane] == 0
			{
				testing.expect_value(t, capture.counts[bodies[lane].value], 0);
				continue;
			}
			testing.expect_value(t, capture.counts[bodies[lane].value], 1);
			testing.expect_value(t, capture.last[bodies[lane].value],
				e.Global_Position{1e9 + f64(lane + 1) / 512, -1e9 - f64(lane) / 1024, 1e9 + 0.125});
		}
	}
	testing.expect_value(t, capture.invalid_lanes[0], 0);
}

@(test)
global_and_legacy_callback_prediction_keep_distinct_position_contracts :: proc(t: ^testing.T)
{
	for variant in 0 ..< 2
	{
		capture: Capture = {origin={1e9, 0, 0}, mode=.Position_Acceleration};
		callbacks: e.Pose_Callbacks = capture_callbacks(&capture);
		if variant == 1
		{
			callbacks.integrate_velocity_global = nil;
			callbacks.integrate_velocity = capture_legacy;
		}
		scene: Scene;
		scene_initialize(t, &scene, callbacks=callbacks);
		body: e.Body_Handle = add_body(t, &scene, {1e9 + 0.001953125, -1e9 + 0.25, 1e9 - 0.125}, scene.sphere);
		velocity: e.Body_Velocity = {linear={1, 2, 3}};
		testing.expect_value(t, e.body_set_velocity(&scene.world, body, velocity), e.Status.Ok);
		testing.expect_value(t, p.pose_integrator_prepare(&scene.simulation.integrator, 0.125), e.Status.Ok);
		testing.expect_value(t, p.pose_integrator_predict_velocity(&scene.simulation.integrator, 0, 0.125, &velocity), e.Status.Ok);
		testing.expect_value(t, p.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125, &scene.simulation.integrator), e.Status.Ok);
		expected: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.25, 1e9 - 0.125};
		if variant == 1
		{
			expected = {1e9, -1e9, 1e9};
		}
		testing.expect_value(t, capture.counts[body.value], 2);
		testing.expect_value(t, capture.positions[body.value][0], expected);
		testing.expect_value(t, capture.positions[body.value][1], expected);
		expect_body_position(t, &scene, body, {1e9 + 0.001953125, -1e9 + 0.25, 1e9 - 0.125});
		state: e.Body_Description;
		status: e.Status;
		state, status = e.body_get(&scene.world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, state.velocity, e.Body_Velocity{linear={1, 2, 3}});
		testing.expect_value(t, velocity.linear.x, f32(1 + (expected.x - 1e9) * 0.125));
		scene_dispose(t, &scene);
	}
}

@(test)
global_callback_substeps_observe_current_then_advanced_poses :: proc(t: ^testing.T)
{
	for workers in ([2]i32{1, 4})
	{
		for count in ([5]i32{1, 7, 8, 9, 65})
		{
			capture: Capture;
			scene: Scene;
			scene_initialize(t, &scene, capacity=128, substeps=4, workers=workers, callbacks=capture_callbacks(&capture));
			bodies: [65]e.Body_Handle;
			for index in 0 ..< int(count)
			{
				bodies[index] = add_body(t, &scene, {1e9 + f64(index) * 4 + 0.001953125, 0, 0}, scene.sphere);
				testing.expect_value(t, e.body_set_velocity(&scene.world, bodies[index], {linear={0.125, 0, 0}}), e.Status.Ok);
			}
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			for index in 0 ..< int(count)
			{
				handle: i32 = bodies[index].value;
				testing.expect_value(t, capture.counts[handle], 5);
				for visit in 0 ..< 5
				{
					x: f64 = 1e9 + f64(index) * 4 + 0.001953125 + f64(max(0, visit - 1)) * 0.0078125;
					testing.expect_value(t, capture.positions[handle][visit], e.Global_Position{x, 0, 0});
				}
				expect_body_position(t, &scene, bodies[index], {1e9 + f64(index) * 4 + 0.001953125 + 0.03125, 0, 0});
			}
			// work stealing does not promise participation by every worker. the
			// per-body visit and exact-position assertions above prove full work.
			// this check admits only configured worker IDs, including caller zero
			calls: int;
			for worker_calls, worker in capture.worker_calls
			{
				if worker < int(workers)
				{
					calls += worker_calls;
				}
				else
				{
					testing.expect_value(t, worker_calls, 0);
				}
			}
			testing.expect(t, calls > 0);
			scene_dispose(t, &scene);
		}
	}
}
