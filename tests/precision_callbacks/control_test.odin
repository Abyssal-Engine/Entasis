package precision_callbacks_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_callback_controls_preserve_positions_and_suppress_target_lanes :: proc(t: ^testing.T)
{
	capture: Capture = {origin={1e9, 0, 0}, mode=.Position_Acceleration};
	scene: Scene;
	scene_initialize(t, &scene, callbacks=capture_callbacks(&capture));
	body: e.Body_Handle = add_body(t, &scene, {1e9 + 0.001953125, 0, 0}, {});
	kinematic: e.Body_Handle;
	status: e.Status;
	kinematic, status = e.body_add_global(&scene.world, e.body_kinematic({}, e.pose(), {}, e.body_activity(-1, 255)), {1e9 + 10, 0, 0});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	testing.expect_value(t, e.body_set_damping(&scene.world, body, {mode=.Override}), e.Status.Invalid_Argument);
	testing.expect_value(t, e.body_set_damping(&scene.world, body, {mode=.Additional}), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&scene.world, body, {1, 0, 0}, .Acceleration), e.Status.Ok);
	target: e.Global_Pose = {{1e9 + 10.001953125, 0, 0}, {w=1}};
	testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, kinematic, target), e.Status.Ok);
	testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
	testing.expect_value(t, capture.counts[kinematic.value], 0);
	testing.expect_value(t, capture.counts[body.value], 2);
	testing.expect_value(t, capture.positions[body.value][0], e.Global_Position{1e9 + 0.001953125, 0, 0});
	testing.expect_value(t, capture.positions[body.value][1], e.Global_Position{1e9 + 0.001953125, 0, 0});
	state: e.Body_Description;
	state, status = e.body_get(&scene.world, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.velocity.linear.x, f32(0.25048828125));
	expect_body_position(t, &scene, kinematic, target.position);
	testing.expect_value(t, e.world_disable_body_control(&scene.world), e.Status.Ok);
	testing.expect(t, scene.simulation.integrator.callbacks.integrate_velocity_global == capture_global);
	testing.expect(t, scene.simulation.integrator.callbacks.integrate_velocity == nil);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&scene.world, body, {}), e.Status.Ok);
	testing.expect_value(t, capture.initializations, 1);
	scene_dispose(t, &scene);
	testing.expect_value(t, capture.disposals, 1);
}

@(test)
global_planetary_controls_override_only_damping_not_gravity :: proc(t: ^testing.T)
{
	for mode in e.Body_Damping_Mode
	{
		policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global({1e9 + 0.125, 0, 0}, 4, 0.75, 0.5);
		scene: Scene;
		scene_initialize(t, &scene, callbacks=e.pose_policy_planetary_global(&policy));
		body: e.Body_Handle = add_body(t, &scene, {1e9 + 2.125, 0, 0}, {});
		testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
		testing.expect_value(t, e.body_set_damping(&scene.world, body, {linear=0.375, angular=0.25, mode=mode}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
		state: e.Body_Description;
		status: e.Status;
		state, status = e.body_get(&scene.world, body);
		testing.expect_value(t, status, e.Status.Ok);
		expected: f64 = -0.125 * math.exp(-0.375 * 0.125);
		if mode == .Additional
		{
			expected *= math.pow(f64(0.25), f64(0.125));
		}
		testing.expectf(t, abs(f64(state.velocity.linear.x) - expected) < 2e-7, "mode %v actual %v expected %v", mode, state.velocity.linear.x, expected);
		testing.expect_value(t, e.world_disable_body_control(&scene.world), e.Status.Ok);
		testing.expect(t, scene.simulation.integrator.callbacks.integrate_velocity_global == e.planetary_gravity_global_velocity);
		scene_dispose(t, &scene);
	}
}

@(test)
global_callbacks_prepared_control_steps_do_not_allocate :: proc(t: ^testing.T)
{
	policy: e.Global_Planetary_Gravity_Policy = e.planetary_gravity_policy_global({1e9 + 0.125, 0, 0}, 4, 0.5);
	scene: Scene;
	scene_initialize(t, &scene, substeps=4, callbacks=e.pose_policy_planetary_global(&policy));
	defer scene_dispose(t, &scene);
	body: e.Body_Handle = add_body(t, &scene, {1e9 + 2.126953125, 0, 0}, {});
	kinematic: e.Body_Handle;
	status: e.Status;
	kinematic, status = e.body_add_global(&scene.world, e.body_kinematic({}, e.pose(), {}, e.body_activity(-1, 255)), {1e9 + 10, 0, 0});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	testing.expect_value(t, e.body_set_damping(&scene.world, body, {linear=0.25, mode=.Override}), e.Status.Ok);
	allocations, bytes: i64;
	for iteration in 0 ..< 1028
	{
		if iteration == 4
		{
			allocations = scene.tracker.total_allocation_count;
			bytes = scene.tracker.current_memory_allocated;
		}
		testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {{1e9 + 2.126953125, 0, 0}, {w=1}}), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&scene.world, body, {}), e.Status.Ok);
		testing.expect_value(t, e.body_add_force(&scene.world, body, {0.5, 0, 0}, .Acceleration), e.Status.Ok);
		target: e.Global_Pose = {{1e9 + 10 + f64(iteration % 2) / 512, 0, 0}, {w=1}};
		testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, kinematic, target), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.015625), e.Status.Ok);
		expect_body_position(t, &scene, kinematic, target.position);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

@(test)
global_callback_prepare_failure_restores_controls_and_allows_retry :: proc(t: ^testing.T)
{
	capture: Capture;
	scene: Scene;
	scene_initialize(t, &scene, callbacks=capture_callbacks(&capture));
	defer scene_dispose(t, &scene);
	body: e.Body_Handle = add_body(t, &scene, {1e9 + 0.001953125, 0, 0}, {});
	kinematic: e.Body_Handle;
	status: e.Status;
	kinematic, status = e.body_add_global(&scene.world, e.body_kinematic({}, e.pose(), {}, e.body_activity(-1, 255)), {1e9 + 10, 0, 0});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&scene.world, body, {1, 0, 0}, .Acceleration), e.Status.Ok);
	target: e.Global_Pose = {{1e9 + 10.001953125, 0, 0}, {w=1}};
	testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, kinematic, target), e.Status.Ok);
	capture.prepare_status = .Capacity_Missing;
	testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Capacity_Missing);
	testing.expect_value(t, scene.simulation.state, p.Simulation_State.Ready);
	testing.expect_value(t, scene.simulation.body_control.phase, p.Body_Control_Phase.Idle);
	testing.expect_value(t, capture.counts[body.value], 0);
	expect_body_position(t, &scene, body, {1e9 + 0.001953125, 0, 0});
	expect_body_position(t, &scene, kinematic, {1e9 + 10, 0, 0});
	state: e.Body_Description;
	state, status = e.body_get(&scene.world, kinematic);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.velocity, e.Body_Velocity{});
	capture.prepare_status = .Ok;
	testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
	state, status = e.body_get(&scene.world, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.velocity.linear.x, f32(0.25));
	expect_body_position(t, &scene, kinematic, target.position);
}
