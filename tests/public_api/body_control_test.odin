package public_api_tests

import "base:runtime"
import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Body_Control_Target_Cancellation :: enum
{
	Clear,
	Disable,
	Clear_After_Velocity_Edit,
	Disable_After_Velocity_Edit,
	Sleeping_Clear,
	Sleeping_Disable,
}

@(test)
body_control_replaced_expired_target_cancellation :: proc (t: ^testing.T)
{
	for cancellation in Body_Control_Target_Cancellation
	{
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		world: e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		status: e.Status;
		shape, status = e.shape_add(&world, e.sphere(1));
		testing.expect_value(t, status, e.Status.Ok);
		body: e.Body_Handle;
		body, status = e.body_add(&world, e.body_kinematic(shape, e.pose(), {}, e.body_activity(-1, 255)));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		testing.expect_value(t, e.body_set_kinematic_target(&world, body, e.pose({2, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		if cancellation == .Sleeping_Clear || cancellation == .Sleeping_Disable
		{
			handles: [1]e.Body_Handle = {body};
			testing.expect_value(t, e.bodies_sleep_group(&world, handles[:]), e.Status.Ok);
		}
		expected_velocity: e.Body_Velocity;
		if cancellation == .Clear_After_Velocity_Edit || cancellation == .Disable_After_Velocity_Edit
		{
			expected_velocity = e.velocity({7, 0, 0});
			testing.expect_value(t, e.body_set_velocity(&world, body, expected_velocity), e.Status.Ok);
		}
		testing.expect_value(t, e.body_set_kinematic_target(&world, body, e.pose({4, 0, 0})), e.Status.Ok);
		if cancellation == .Clear || cancellation == .Clear_After_Velocity_Edit || cancellation == .Sleeping_Clear
		{
			testing.expect_value(t, e.body_clear_kinematic_target(&world, body), e.Status.Ok);
		}
		else
		{
			testing.expect_value(t, e.world_disable_body_control(&world), e.Status.Ok);
		}
		value: e.Body_Description;
		value, status = e.body_get(&world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, value.velocity, expected_velocity);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		value, status = e.body_get(&world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, value.velocity, expected_velocity);
		testing.expect(t, abs(value.pose.position.x - (2 + expected_velocity.linear.x * 0.5)) < 1e-5);
	}
}

@(test)
body_control_standalone_solve_commits_restitution :: proc (t: ^testing.T)
{
	scene: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	configuration: e.Restitution_Configuration = e.restitution_configuration_default();
	configuration.fallback = {0.5, 1};
	testing.expect_value(t, e.world_enable_restitution(&scene.world, configuration), e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	// the existing iterative box solver is the numerical reference. body-control
	// participation must preserve its bounce, including the second rearmed impact
	control: Restitution_Test_Scene;
	if !testing.expect_value(t, restitution_test_scene(&control), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&control.world);
	testing.expect_value(t, e.world_enable_restitution(&control.world, configuration), e.Status.Ok);
	testing.expect_value(t, e.body_set_pose(&control.world, control.body, e.pose({0, 2, 0})), e.Status.Ok);
	testing.expect_value(t, e.body_set_velocity(&control.world, control.body, e.velocity({0, -10, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_stage_predict_bounds(&control.world, 1.0/60), e.Status.Ok);
	testing.expect_value(t, e.world_stage_collision_detection(&control.world, 1.0/60), e.Status.Ok);
	testing.expect_value(t, e.world_stage_solve(&control.world, 1.0/60), e.Status.Ok);
	control_value: e.Body_Description;
	control_status: e.Status;
	control_value, control_status = e.body_get(&control.world, control.body);
	testing.expect_value(t, control_status, e.Status.Ok);
	testing.expect(t, control_value.velocity.linear.y > 0);
	for _ in 0 ..< 2
	{
		// separate the pair before rearming the next physical impact
		testing.expect_value(t, e.body_set_pose(&scene.world, scene.body, e.pose({0, 5, 0})), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&scene.world, scene.body, {}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 1.0/60), e.Status.Ok);
		testing.expect_value(t, e.body_set_pose(&scene.world, scene.body, e.pose({0, 2, 0})), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&scene.world, scene.body, e.velocity({0, -10, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_stage_predict_bounds(&scene.world, 1.0/60), e.Status.Ok);
		testing.expect_value(t, e.world_stage_collision_detection(&scene.world, 1.0/60), e.Status.Ok);
		stats: e.World_Stats;
		status: e.Status;
		stats, status = e.world_stats(&scene.world);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, stats.active_constraints > 0);
		testing.expect_value(t, e.world_stage_solve(&scene.world, 1.0/60), e.Status.Ok);
		testing.expect_value(t, e.restitution_set(&scene.world, scene.body_reference, {0.5, 1}), e.Status.Ok);
		testing.expect_value(t, e.restitution_reserve(&scene.world, 32, 128), e.Status.Ok);
		testing.expect_value(t, e.restitution_remove(&scene.world, scene.body_reference), e.Status.Ok);
		value: e.Body_Description;
		value, status = e.body_get(&scene.world, scene.body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, value.velocity.linear.y, control_value.velocity.linear.y);
	}
}

body_control_test_body :: proc (world: ^e.World, position: e.Vector3 = {}) -> (e.Body_Handle, e.Status)
{
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(world, e.sphere(1));
	if status != .Ok
	{
		return {}, status;
	}
	inertia: e.Body_Inertia;
	inertia, status = e.shape_inertia(e.sphere(1), 2);
	if status != .Ok
	{
		return {}, status;
	}
	return e.body_add(world, e.body_dynamic(shape, inertia, e.pose(position), {}, e.body_activity(-1, 255)));
}

@(test)
body_control_force_duration_modes_and_legacy_impulses :: proc (t: ^testing.T)
{
	for substeps in ([3]i32{1, 4, 8})
	{
		for workers in ([2]i32{1, 2})
		{
			description: e.World_Description = small_world_description();
			description.gravity = {};
			description.damping = {};
			description.solve.substeps = substeps;
			description.threading.worker_count = workers;
			world: e.World;
			if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
			{
				return;
			}
			defer e.world_destroy(&world);
			simulation: ^p.Simulation;
			simulation, _ = e.world_borrow_simulation(&world);
			original: p.Pose_Integrator_Callbacks = simulation.integrator.callbacks;
			testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
			testing.expect_value(t, simulation.integrator.callbacks.integrate_velocity, original.integrate_velocity);
			bodies: [13]e.Body_Handle;
			for index in 0 ..< len(bodies)
			{
				bodies[index], _ = body_control_test_body(&world, {f32(index)*10, 0, 0});
				testing.expect_value(t, e.body_add_force(&world, bodies[index], {4, 0, 0}), e.Status.Ok);
				testing.expect_value(t, e.body_add_force(&world, bodies[index], {2, 0, 0}, .Acceleration), e.Status.Ok);
				testing.expect_value(t, e.body_add_torque(&world, bodies[index], {0, 0, 2}), e.Status.Ok);
			}
			testing.expect_value(t, e.world_step(&world, 0.25), e.Status.Ok);
			for body in bodies
			{
				value: e.Body_Description;
				value, _ = e.body_get(&world, body);
				testing.expect(t, abs(value.velocity.linear.x-1) < 2e-5);
				testing.expect(t, abs(value.velocity.angular.z-0.625) < 2e-5);
			}
			testing.expect_value(t, simulation.body_control.inputs.count, 0);
			testing.expect_value(t, simulation.integrator.callbacks.integrate_velocity, original.integrate_velocity);
			testing.expect_value(t, e.world_step(&world, 0.25), e.Status.Ok);
			value: e.Body_Description;
			value, _ = e.body_get(&world, bodies[0]);
			testing.expect(t, abs(value.velocity.linear.x-1) < 2e-5);
			testing.expect_value(t, e.body_apply_linear_impulse(&world, bodies[0], {2, 0, 0}), e.Status.Ok);
			value, _ = e.body_get(&world, bodies[0]);
			testing.expect(t, abs(value.velocity.linear.x-2) < 2e-5);
		}
	}
}

@(test)
body_control_world_point_wrench_and_input_validation :: proc (t: ^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world, {10, 20, 30});
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	testing.expect_value(t, e.body_add_force_at_position(&world, body, {4, 0, 0}, {10, 21, 30}), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&world, body, {math.nan_f32(), 0, 0}), e.Status.Invalid_Argument);
	testing.expect_value(t, e.body_add_torque(&world, body, {0, 0, 0}, e.Body_Input_Mode(255)), e.Status.Invalid_Argument);
	testing.expect_value(t, e.world_step(&world, 0.25), e.Status.Ok);
	value: e.Body_Description;
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-0.5)<1e-5);
	testing.expect(t, abs(value.velocity.angular.z+1.25)<1e-5);
}

@(test)
body_control_sleep_pending_zero_inputs_and_handle_reuse :: proc (t: ^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	group: [1]e.Body_Handle = {body};
	sleep_status: e.Status = e.bodies_sleep_group(&world, group[:]);
	testing.expect_value(t, sleep_status, e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&world, body, {}), e.Status.Ok);
	state: bool;
	state, _ = e.body_is_sleeping(&world, body);
	testing.expect(t, state);
	testing.expect_value(t, e.body_add_force(&world, body, {8, 0, 0}, .Force, .Preserve_Sleep), e.Status.Ok);
	sleep_simulation: ^p.Simulation;
	sleep_simulation, _ = e.world_borrow_simulation(&world);
	testing.expect_value(t, sleep_simulation.body_control.activation, p.Reference_State.Missing);
	testing.expect_value(t, sleep_simulation.integrator.callbacks.integrate_velocity, sleep_simulation.body_control.callbacks.integrate_velocity);
	for step in 0 ..< 3
	{
		_ = step;
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
	}
	testing.expect_value(t, e.body_awaken(&world, body), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
	value: e.Body_Description;
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-2)<1e-5);
	testing.expect_value(t, e.body_add_force(&world, body, {100, 0, 0}), e.Status.Ok);
	testing.expect_value(t, e.body_set_damping(&world, body, {linear=2}), e.Status.Ok);
	testing.expect_value(t, e.body_remove(&world, body), e.Status.Ok);
	replacement: e.Body_Handle;
	replacement, _ = body_control_test_body(&world);
	testing.expect_value(t, replacement, body);
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
	value, _ = e.body_get(&world, replacement);
	testing.expect_value(t, value.velocity.linear, e.Vector3{});
	damping: e.Body_Damping;
	result: e.Status;
	damping, result = e.body_get_damping(&world, replacement);
	testing.expect_value(t, result, e.Status.Ok);
	testing.expect_value(t, damping, e.Body_Damping{});
}

@(test)
body_control_damping_additional_override_and_masks :: proc (t: ^testing.T)
{
	for mode in ([2]e.Body_Damping_Mode{.Additional, .Override})
	{
		for substeps in ([2]i32{1, 4})
		{
			description: e.World_Description = small_world_description();
			description.gravity = {};
			description.damping = {linear=0.5, angular=0.5};
			description.solve.substeps = substeps;
			world: e.World;
			if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
			{
				return;
			}
			defer e.world_destroy(&world);
			body: e.Body_Handle;
			body, _ = body_control_test_body(&world);
			other: e.Body_Handle;
			other, _ = body_control_test_body(&world, {10, 0, 0});
			testing.expect_value(t, e.body_set_velocity(&world, body, e.velocity({10, 0, 0}, {0, 0, 10})), e.Status.Ok);
			testing.expect_value(t, e.body_set_velocity(&world, other, e.velocity({10, 0, 0})), e.Status.Ok);
			testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
			testing.expect_value(t, e.body_set_damping(&world, body, {linear=2, angular=3, mode=mode}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&world, 1), e.Status.Ok);
			value: e.Body_Description;
			value, _ = e.body_get(&world, body);
			factor: f32 = 1;
			if mode == .Additional
			{
				factor = 0.5;
			}
			testing.expect(t, abs(value.velocity.linear.x-10*factor*f32(math.exp(-2.0)))<2e-5);
			testing.expect(t, abs(value.velocity.angular.z-10*factor*f32(math.exp(-3.0)))<2e-5);
			unchanged: e.Body_Description;
			unchanged, _ = e.body_get(&world, other);
			testing.expect(t, abs(unchanged.velocity.linear.x-5)<1e-5);
		}
	}
}

@(test)
body_control_kinematic_target_rotation_expiration_and_explicit_velocity :: proc (t: ^testing.T)
{
	for substeps in ([3]i32{1, 4, 8})
	{
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		description.solve.substeps = substeps;
		world: e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.box(2, 2, 2));
		body: e.Body_Handle;
		body, _ = e.body_add(&world, e.body_kinematic(shape, e.pose(), {}, e.body_activity(-1, 255)));
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		target: e.Rigid_Pose = e.pose({2, 3, 4}, {0, f32(math.sin(0.5)), 0, f32(math.cos(0.5))});
		testing.expect_value(t, e.body_add_force(&world, body, {1, 0, 0}), e.Status.Invalid_Argument);
		testing.expect_value(t, e.body_set_kinematic_target(&world, body, target), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		value: e.Body_Description;
		value, _ = e.body_get(&world, body);
		testing.expect(t, util.vector3_length(util.vector3_subtract(value.pose.position, target.position))<2e-4);
		testing.expect(t, abs(value.pose.orientation.y-target.orientation.y)<2e-4);
		testing.expect(t, abs(value.velocity.angular.y-2)<2e-4);
		result: e.Status;
		_, result = e.body_get_kinematic_target(&world, body);
		testing.expect_value(t, result, e.Status.Not_Found);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		value, _ = e.body_get(&world, body);
		testing.expect_value(t, value.velocity.linear, e.Vector3{});
		testing.expect_value(t, value.velocity.angular, e.Vector3{});
		testing.expect(t, util.vector3_length(util.vector3_subtract(value.pose.position, target.position))<2e-4);
		target.position.x += 1;
		testing.expect_value(t, e.body_set_kinematic_target(&world, body, target), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&world, body, e.velocity({7, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		value, _ = e.body_get(&world, body);
		testing.expect(t, abs(value.velocity.linear.x-7)<1e-5);
	}
}

Body_Control_Test_Step :: enum u8
{
	Before, After, Success, Two_Solves,
}

Body_Control_Test_Context :: struct
{
	mode: Body_Control_Test_Step,
}

body_control_test_step :: proc "contextless" (
	user_context: rawptr, simulation: ^p.Simulation, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> p.Physics_Status
{
	context = runtime.default_context();
	state: ^Body_Control_Test_Context = (^Body_Control_Test_Context)(user_context);
	if state.mode == .Before
	{
		return .Capacity_Missing;
	}
	h: f32 = dt;
	if state.mode == .Two_Solves
	{
		h = dt*0.5;
	}
	status: p.Physics_Status = p.simulation_solve(simulation, h, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if state.mode == .After
	{
		return .Capacity_Missing;
	}
	if state.mode == .Two_Solves
	{
		status = p.simulation_solve(simulation, h, dispatcher);
		if status != .Ok
		{
			return status;
		}
	}
	simulation.step_index += 1;
	return .Ok;
}

@(test)
body_control_custom_failure_no_replay_and_manual_stages :: proc (t: ^testing.T)
{
	state: Body_Control_Test_Context;
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	description.timestepper = e.timestepper(body_control_test_step, &state);
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&world, body, {8, 0, 0}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Capacity_Missing);
	value: e.Body_Description;
	value, _ = e.body_get(&world, body);
	testing.expect_value(t, value.velocity.linear, e.Vector3{});
	state.mode = .After;
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Capacity_Missing);
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-2)<1e-5);
	state.mode = .Success;
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-2)<1e-5);
	state.mode = .Two_Solves;
	testing.expect_value(t, e.body_add_force(&world, body, {8, 0, 0}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-4)<1e-5);
	testing.expect_value(t, e.body_add_force(&world, body, {8, 0, 0}), e.Status.Ok);
	testing.expect_value(t, e.world_stage_solve(&world, 0.5), e.Status.Ok);
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-6)<1e-5);
	testing.expect_value(t, e.world_stage_solve(&world, 0.5), e.Status.Ok);
	value, _ = e.body_get(&world, body);
	testing.expect(t, abs(value.velocity.linear.x-6)<1e-5);
}

@(test)
body_control_owned_reservation_failure_and_warm_steps :: proc (t: ^testing.T)
{
	tracker: allocation_test_tracker;
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	description.allocator = allocation_test_allocator(&tracker);
	world: e.World;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	before: int = tracker.requests;
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	requests: int = tracker.requests-before;
	testing.expect_value(t, e.body_add_force(&world, body, {1, 0, 0}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
	tracker.fail_from = tracker.requests+1;
	before = tracker.requests;
	for step in 0 ..< 100
	{
		_ = step;
		testing.expect_value(t, e.body_control_reserve(&world, 1, 1), e.Status.Ok);
		testing.expect_value(t, e.body_add_force(&world, body, {1, 0, 0}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
	}
	testing.expect_value(t, tracker.requests, before);
	testing.expect_value(t, e.body_set_damping(&world, body, {linear=2}), e.Status.Ok);
	testing.expect_value(t, e.body_control_reserve(&world, 4096, 4096), e.Status.Capacity_Missing);
	damping: e.Body_Damping;
	status: e.Status;
	damping, status = e.body_get_damping(&world, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, damping.linear, f32(2));
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
	for fail in 1 ..= requests
	{
		tracker = {};
		world = nil;
		if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
		{
			return;
		}
		tracker.fail_from = tracker.requests+fail;
		result: e.Status = e.world_enable_body_control(&world);
		testing.expect(t, result == .Ok || result == .Capacity_Missing);
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
}

@(test)
body_control_constrained_fallback_restitution_responsibilities :: proc (t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			for bounce in 0 ..< 2
			{
				description: e.World_Description = small_world_description();
				description.gravity = {};
				description.damping = {};
				description.threading.worker_count = workers;
				description.solve = {velocity_iterations=4, substeps=substeps, fallback_batch_threshold=1};
				world: e.World;
				if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
				{
					return;
				}
				defer e.world_destroy(&world);
				type_id: e.Constraint_Type_ID;
				valid: bool;
				type_id, valid = register_velocity_target_constraint(t, &world);
				if !valid
				{
					return;
				}
				target: Velocity_Target_Constraint;
				bodies: [17]e.Body_Handle;
				for &body, index in bodies
				{
					body, _ = body_control_test_body(&world, {f32(index)*5, 0, 0});
					for _ in 0 ..< 3
					{
						status: e.Status;
						_, status = e.custom_constraint_add_typed(&world, bodies[index:index+1], type_id, &target);
						testing.expect_value(t, status, e.Status.Ok);
					}
				}
				testing.expect_value(t, e.world_enable_body_control(&world, {32, 32}), e.Status.Ok);
				if bounce != 0
				{
					config: e.Restitution_Configuration = e.restitution_configuration_default();
					config.fallback = {0.5, 1};
					testing.expect_value(t, e.world_enable_restitution(&world, config), e.Status.Ok);
				}
				for body in bodies
				{
					testing.expect_value(t, e.body_add_force(&world, body, {0, 8, 0}), e.Status.Ok);
					testing.expect_value(t, e.body_add_torque(&world, body, {0, 0, 2}), e.Status.Ok);
				}
				testing.expect_value(t, e.world_step(&world, 0.25), e.Status.Ok);
				for body in bodies
				{
					value: e.Body_Description;
					status: e.Status;
					value, status = e.body_get(&world, body);
					testing.expect_value(t, status, e.Status.Ok);
					testing.expectf(t, abs(value.velocity.linear.y-1)<2e-5, "substeps=%d bounce=%d vy=%v", substeps, bounce, value.velocity.linear.y);
					testing.expect(t, abs(value.velocity.angular.z-0.625)<2e-5);
				}
				simulation: ^p.Simulation;
				simulation, _ = e.world_borrow_simulation(&world);
				testing.expect_value(t, simulation.body_control.inputs.count, 0);
				testing.expect(t, simulation.solver.active_set.batch_count>simulation.solver.fallback_batch_index);
				for body in bodies
				{
					testing.expect_value(t, e.body_set_damping(&world, body, {linear=2, angular=3}), e.Status.Ok);
				}
				testing.expect_value(t, e.world_step(&world, 0.25), e.Status.Ok);
				for body in bodies
				{
					value: e.Body_Description;
					value, _ = e.body_get(&world, body);
					testing.expect(t, abs(value.velocity.linear.y-f32(math.exp(-0.5)))<2e-5);
					testing.expect(t, abs(value.velocity.angular.z-0.625*f32(math.exp(-0.75)))<2e-5);
				}
			}
		}
	}
}

@(test)
body_control_manual_preview_target_contact_and_disable :: proc (t: ^testing.T)
{
	for substeps in ([2]i32{1, 4})
	{
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		description.solve.substeps = substeps;
		world: e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		shape: e.Shape_Handle;
		shape, _ = e.shape_add(&world, e.sphere(1));
		kinematic: e.Body_Handle;
		kinematic, _ = e.body_add(&world, e.body_kinematic(shape, e.pose(), {}, e.body_activity(-1, 255)));
		body: e.Body_Handle;
		body, _ = body_control_test_body(&world, {2, 0, 0});
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		target: e.Rigid_Pose = e.pose({1, 0, 0});
		testing.expect_value(t, e.body_set_kinematic_target(&world, kinematic, target), e.Status.Ok);
		testing.expect_value(t, e.world_stage_predict_bounds(&world, 0.5), e.Status.Ok);
		pending: e.Rigid_Pose;
		result: e.Status;
		pending, result = e.body_get_kinematic_target(&world, kinematic);
		testing.expect_value(t, result, e.Status.Ok);
		testing.expect_value(t, pending, target);
		before: e.Body_Description;
		before, _ = e.body_get(&world, kinematic);
		testing.expect_value(t, before.velocity.linear, e.Vector3{});
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		pushed: e.Body_Description;
		pushed, _ = e.body_get(&world, body);
		testing.expect(t, pushed.velocity.linear.x>1.5);
		moved: e.Body_Description;
		moved, _ = e.body_get(&world, kinematic);
		testing.expect(t, abs(moved.pose.position.x-1)<1e-5);
		testing.expect_value(t, e.world_disable_body_control(&world), e.Status.Ok);
		moved, _ = e.body_get(&world, kinematic);
		testing.expect_value(t, moved.velocity.linear, e.Vector3{});
		testing.expect_value(t, e.world_step(&world, 0.5), e.Status.Ok);
		moved, _ = e.body_get(&world, kinematic);
		testing.expect(t, abs(moved.pose.position.x-1)<1e-5);
	}
}

Body_Control_Callback_State :: struct
{
	world: ^e.World, body: e.Body_Handle, disposed, attempted: i32,
}

body_control_test_initialize :: proc "contextless" (raw: rawptr, simulation: ^p.Simulation) -> p.Physics_Status
{
	_, _ = raw, simulation;
	return .Ok;
}

body_control_test_prepare :: proc "contextless" (raw: rawptr, dt: f32) -> p.Physics_Status
{
	context = runtime.default_context();
	state: ^Body_Control_Callback_State = (^Body_Control_Callback_State)(raw);
	state.attempted += 1;
	if e.body_add_force(state.world, state.body, {1, 0, 0}) != .Invalid_Argument
	{
		return .Capacity_Missing;
	}
	return .Ok if dt>0 else .Invalid_Argument;
}

body_control_test_velocity :: proc "contextless" (
	raw: rawptr, indices: util.I32x8, position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	inertia: p.Body_Inertia_Wide, mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^p.Body_Velocity_Wide,
)
{
	_, _, _, _, _, _, _, _ = raw, indices, position, orientation, inertia, mask, worker, dt;
	velocity.linear = util.vector3_wide_scale(velocity.linear, util.F32x8(0.5));
}

body_control_test_dispose :: proc "contextless" (raw: rawptr)
{
	(^Body_Control_Callback_State)(raw).disposed += 1;
}

@(test)
body_control_custom_callback_composition_and_manual_reentry :: proc (t: ^testing.T)
{
	world: e.World;
	state: Body_Control_Callback_State = {world=&world};
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	description.pose_callbacks = {initialize=body_control_test_initialize, prepare_for_integration=body_control_test_prepare,
		integrate_velocity=body_control_test_velocity, dispose=body_control_test_dispose, user_context=&state};
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	state.body, _ = body_control_test_body(&world);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	testing.expect_value(t, e.body_set_velocity(&world, state.body, e.velocity({10, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.body_set_damping(&world, state.body, {linear=2, mode=.Override}), e.Status.Invalid_Argument);
	testing.expect_value(t, e.body_set_damping(&world, state.body, {linear=2}), e.Status.Ok);
	testing.expect_value(t, e.world_stage_predict_bounds(&world, 1), e.Status.Ok);
	testing.expect_value(t, e.world_stage_solve(&world, 1), e.Status.Ok);
	value: e.Body_Description;
	value, _ = e.body_get(&world, state.body);
	testing.expect(t, abs(value.velocity.linear.x-5*f32(math.exp(-2.0)))<2e-5);
	testing.expect(t, state.attempted>=2);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, state.disposed, i32(1));
}

@(test)
body_control_target_preflight_and_prepared_cancellation :: proc (t: ^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, _ = e.shape_add(&world, e.sphere(1));
	bodies: [2]e.Body_Handle;
	for &body, index in bodies
	{
		body, _=e.body_add(&world, e.body_kinematic(shape, e.pose({0, f32(index)*10, 0}), {}, e.body_activity(-1, 255)));
	}
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	testing.expect_value(t, e.body_set_kinematic_target(&world, bodies[0], e.pose({1, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.body_set_kinematic_target(&world, bodies[1], e.pose({2e38, 10, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 1e-20), e.Status.Invalid_Argument);
	for body in bodies
	{
		value: e.Body_Description;
		value, _ = e.body_get(&world, body);
		testing.expect_value(t, value.velocity.linear, e.Vector3{});
		status: e.Status;
		_, status = e.body_get_kinematic_target(&world, body);
		testing.expect_value(t, status, e.Status.Ok);
	}
	testing.expect_value(t, e.body_clear_kinematic_target(&world, bodies[1]), e.Status.Ok);
	testing.expect_value(t, e.world_stage_predict_bounds(&world, 0.5), e.Status.Ok);
	testing.expect_value(t, e.world_stage_collision_detection(&world, 0.5), e.Status.Ok);
	prepared: e.Body_Description;
	prepared, _ = e.body_get(&world, bodies[0]);
	testing.expect(t, abs(prepared.velocity.linear.x-2)<1e-5);
	testing.expect_value(t, e.body_clear_kinematic_target(&world, bodies[0]), e.Status.Ok);
	prepared, _=e.body_get(&world, bodies[0]);
	testing.expect_value(t, prepared.velocity.linear, e.Vector3{});
	testing.expect_value(t, e.world_stage_solve(&world, 0.5), e.Status.Ok);
	prepared, _=e.body_get(&world, bodies[0]);
	testing.expect_value(t, prepared.pose.position, e.Vector3{});
}

@(test)
body_control_rotated_anisotropic_torque_matches_world_impulse :: proc (t: ^testing.T)
{
	description: e.World_Description=small_world_description();
	description.gravity={};
	description.damping={};
	world:e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	shape, _ = e.shape_add(&world, e.box(2, 4, 6));
	inertia: e.Body_Inertia;
	inertia, _ = e.shape_inertia(e.box(2, 4, 6), 3);
	orientation:e.Quaternion={0, 0, f32(math.sin(0.4)), f32(math.cos(0.4))};
	first: e.Body_Handle;
	first, _ = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 0, 0}, orientation), {}, e.body_activity(-1, 255)));
	second: e.Body_Handle;
	second, _ = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({20, 0, 0}, orientation), {}, e.body_activity(-1, 255)));
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	testing.expect_value(t, e.body_add_torque(&world, first, {2, 3, 4}), e.Status.Ok);
	testing.expect_value(t, e.body_apply_angular_impulse(&world, second, {0.02, 0.03, 0.04}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.01), e.Status.Ok);
	a: e.Body_Description;
	a, _ = e.body_get(&world, first);
	b: e.Body_Description;
	b, _ = e.body_get(&world, second);
	testing.expect(t, util.vector3_length(util.vector3_subtract(a.velocity.angular, b.velocity.angular))<2e-6);
}

@(test)
body_axis_locks_all_masks_full_and_partial_bundles :: proc (t: ^testing.T)
{
	for workers in ([3]i32{1, 2, 4})
	{
		d: e.World_Description = small_world_description();
		d.gravity = {};
		d.damping = {};
		d.threading.worker_count = workers;
		d.solve.velocity_iterations = 8;
		world: e.World;
		if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		bodies: [63]e.Body_Handle;
		for mask in 1 ..< 64
		{
			body: e.Body_Description = e.body_shapeless({inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}},
				e.pose({f32(mask)*10, 0, 0}), e.velocity({1, 2, 3}, {0.2, 0.3, 0.4}), e.body_activity(-1, 255));
			bodies[mask-1], _ = e.body_add(&world, body);
			lock: e.Body_Axis_Lock = e.body_axis_lock_default(body.pose);
			lock.linear_axes = transmute(e.Body_Lock_Axes)u8(mask & 7);
			lock.angular_axes = transmute(e.Body_Lock_Axes)u8(mask >> 3);
			if !testing.expect_value(t, e.body_set_axis_lock(&world, bodies[mask-1], lock), e.Status.Ok)
			{
				return;
			}
			read: e.Body_Axis_Lock;
			status: e.Status;
			read, status = e.body_get_axis_lock(&world, bodies[mask-1]);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, read, lock);
		}
		dt: f32 = 1.0/64;
		testing.expect_value(t, e.world_step(&world, dt), e.Status.Ok);
		spring: e.Spring_Settings = e.spring_settings(30, 1);
		w_dt: f32 = spring.angular_frequency*dt;
		softness: f32 = 1/(1+w_dt*(w_dt+spring.twice_damping_ratio));
		for mask in 1 ..< 64
		{
			actual: e.Body_Description;
			status: e.Status;
			actual, status = e.body_get(&world, bodies[mask-1]);
			testing.expect_value(t, status, e.Status.Ok);
			linear: [3]f32 = {actual.velocity.linear.x, actual.velocity.linear.y, actual.velocity.linear.z};
			angular: [3]f32 = {actual.velocity.angular.x, actual.velocity.angular.y, actual.velocity.angular.z};
			for axis in 0 ..< 3
			{
				linear_expected: f32 = f32(axis+1)*(softness if mask & (1<<uint(axis)) != 0 else 1);
				angular_expected: f32 = (f32(axis)+2)*0.1*(softness if mask & (8<<uint(axis)) != 0 else 1);
				testing.expect(t, abs(linear[axis]-linear_expected)<3e-5);
				testing.expect(t, abs(angular[axis]-angular_expected)<3e-5);
			}
		}
	}
}

@(test)
body_axis_lock_lifetime_sleep_reuse_and_disable :: proc (t: ^testing.T)
{
	d: e.World_Description = small_world_description();
	d.gravity={};
	d.damping={};
	world:e.World;
	if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({}));
	lock.linear_axes={.X, .Y, .Z};
	lock.angular_axes={.X, .Y, .Z};
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	joint: e.Constraint_Handle;
	status: e.Status;
	joint, status = e.constraint_add_1(&world, body, e.One_Body_Angular_Motor{target_velocity={}, settings=e.motor_settings(10, 0)});
	testing.expect_value(t, status, e.Status.Ok);
	// caller-selected group sleeping is explicitly for disconnected bodies.
	// locks must not make an unsupported migration silently lose joints
	testing.expect_value(t, e.bodies_sleep_group(&world, []e.Body_Handle{body}), e.Status.Invalid_Argument);
	_, status=e.constraint_inspect(&world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	body_axis_lock_test_sleep(t, &world, body);
	read: e.Body_Axis_Lock;
	get_status: e.Status;
	read, get_status = e.body_get_axis_lock(&world, body);
	testing.expect_value(t, get_status, e.Status.Ok);
	testing.expect_value(t, read, lock);
	sleeping: bool;
	sleeping, _ = e.body_is_sleeping(&world, body);
	testing.expect(t, sleeping);
	testing.expect_value(t, e.body_clear_axis_lock(&world, body), e.Status.Ok);
	_, get_status=e.body_get_axis_lock(&world, body);
	testing.expect_value(t, get_status, e.Status.Not_Found);
	info: e.Constraint_Info;
	info_status: e.Status;
	info, info_status = e.constraint_inspect(&world, joint);
	_ = info;
	testing.expect_value(t, info_status, e.Status.Ok);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	body_axis_lock_test_sleep(t, &world, body);
	testing.expect_value(t, e.world_disable_body_control(&world), e.Status.Ok);
	_, info_status=e.constraint_inspect(&world, joint);
	testing.expect_value(t, info_status, e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	_, get_status=e.body_get_axis_lock(&world, body);
	testing.expect_value(t, get_status, e.Status.Not_Found);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	testing.expect_value(t, e.body_set_inertia(&world, body, {}), e.Status.Ok);
	_, get_status=e.body_get_axis_lock(&world, body);
	testing.expect_value(t, get_status, e.Status.Not_Found);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Invalid_Argument);
	inertia: e.Body_Inertia;
	inertia, _ = e.shape_inertia(e.sphere(1), 2);
	testing.expect_value(t, e.body_set_inertia(&world, body, inertia), e.Status.Ok);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	testing.expect_value(t, e.body_remove(&world, body), e.Status.Ok);
	reused: e.Body_Handle;
	reused, _ = body_control_test_body(&world);
	testing.expect_value(t, reused, body);
	_, get_status=e.body_get_axis_lock(&world, reused);
	testing.expect_value(t, get_status, e.Status.Not_Found);
	testing.expect_value(t, e.body_set_axis_lock(&world, reused, lock), e.Status.Ok);
	testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
}

@(test)
body_axis_lock_reference_frame_and_coupled_contacts :: proc (t: ^testing.T)
{
	for substeps in ([2]i32{1, 4})
	{
		d: e.World_Description = small_world_description();
		d.gravity={0, -9.81, 0};
		d.damping={};
		d.solve.substeps=substeps;
		d.solve.velocity_iterations=16;
		world:e.World;
		if !testing.expect_value(t, e.world_init(&world, d), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		body: e.Body_Handle;
		body, _ = body_control_test_body(&world, {0, 5, 0});
		orientation:e.Quaternion={0, 0, 0.70710677, 0.70710677};
		testing.expect_value(t, e.body_set_pose(&world, body, e.pose({0, 5, 0}, orientation)), e.Status.Ok);
		lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({0, 5, 0}, orientation));
		lock.linear_axes={.Y};
		lock.angular_axes={.X};
		testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
		// reference X rotates onto world Y. torque about Y must react, while
		// translation X remains unconstrained. contacts push from below
		visitor: e.Body_Handle;
		visitor, _ = body_control_test_body(&world, {0, 3.1, 0});
		testing.expect_value(t, e.body_set_velocity(&world, visitor, e.velocity({0, 3, 0}, {})), e.Status.Ok);
		for step in 0 ..< 120
		{
			_ = step;
			testing.expect_value(t, e.body_add_force(&world, body, {1, 0, 0}), e.Status.Ok);
			testing.expect_value(t, e.body_add_torque(&world, body, {0, 1, 0}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&world, 1.0/120), e.Status.Ok);
		}
		actual: e.Body_Description;
		status: e.Status;
		actual, status = e.body_get(&world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(actual.pose.position.y-5)<0.02);
		testing.expect(t, actual.pose.position.x>0.15);
		testing.expect(t, abs(actual.velocity.angular.y)<0.02);
		testing.expect_value(t, e.body_clear_axis_lock(&world, body), e.Status.Ok);
		for step in 0 ..< 30
		{
			_ = step;
			testing.expect_value(t, e.world_step(&world, 1.0/60), e.Status.Ok);
		}
		actual, status=e.body_get(&world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, actual.pose.position.y<4.5);
	}
}

body_axis_lock_test_sleep :: proc (t: ^testing.T, world: ^e.World, body: e.Body_Handle)
{
	testing.expect_value(t, e.body_set_activity(world, body, e.body_activity(0.1, 1)), e.Status.Ok);
	for _ in 0 ..< 512
	{
		testing.expect_value(t, e.world_step(world, 1.0/64), e.Status.Ok);
		sleeping: bool;
		status: e.Status;
		sleeping, status = e.body_is_sleeping(world, body);
		testing.expect_value(t, status, e.Status.Ok);
		if sleeping
		{
			return;
		}
	}
	testing.expect(t, false, "constrained body did not sleep");
}

@(test)
body_axis_lock_fallback_substeps_and_reanchor :: proc (t: ^testing.T)
{
	for substeps in ([2]i32{1, 4})
	{
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = 2;
		description.solve = {velocity_iterations=8, substeps=substeps, fallback_batch_threshold=1};
		world:e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		bodies:[9]e.Body_Handle;
		for index in 0..<len(bodies)
		{
			body: e.Body_Handle;
			status: e.Status;
			body, status = body_control_test_body(&world, {f32(index)*8, 0, 0});
			testing.expect_value(t, status, e.Status.Ok);
			bodies[index]=body;
			_, status=e.constraint_add_1(&world, body, e.One_Body_Angular_Motor{target_velocity={}, settings=e.motor_settings(1, 0)});
			testing.expect_value(t, status, e.Status.Ok);
			lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({f32(index)*8, 0, 0}));
			lock.linear_axes={.Y};
			testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
		}
		simulation: ^p.Simulation;
		simulation, _ = e.world_borrow_simulation(&world);
		testing.expect_value(t, simulation.solver.sequential_batch.constraint_count, i32(9));
		for _ in 0..<32
		{
			for body in bodies
			{
				testing.expect_value(t, e.body_add_force(&world, body, {2, 8, 0}, .Acceleration), e.Status.Ok);
			}
			testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
		}
		for body in bodies
		{
			value: e.Body_Description;
			value, _ = e.body_get(&world, body);
			testing.expect(t, abs(value.pose.position.y)<0.01);
			testing.expect(t, value.velocity.linear.x>0.99);
			lock: e.Body_Axis_Lock = e.body_axis_lock_default(value.pose);
			lock.linear_axes={.X};
			testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
			restored: e.Body_Axis_Lock;
			status: e.Status;
			restored, status = e.body_get_axis_lock(&world, body);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, restored, lock);
		}
		testing.expect_value(t, e.world_disable_body_control(&world), e.Status.Ok);
		testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(9));
		testing.expect_value(t, simulation.solver.sequential_batch.constraint_count, i32(0));
	}
}

@(test)
body_axis_lock_orientation_error_validation_and_zero_mask :: proc (t: ^testing.T)
{
	d: e.World_Description = small_world_description();
	d.gravity={};
	d.damping={};
	world:e.World;
	testing.expect_value(t, e.world_init(&world, d), e.Status.Ok);
	defer e.world_destroy(&world);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose());
	lock.angular_axes={.X};
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	bad: e.Body_Axis_Lock = lock;
	bad.linear_axes=transmute(e.Body_Lock_Axes)u8(128);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, bad), e.Status.Invalid_Argument);
	bad=lock;
	bad.reference.orientation={};
	testing.expect_value(t, e.body_set_axis_lock(&world, body, bad), e.Status.Invalid_Argument);
	bad=lock;
	bad.spring_settings.angular_frequency=-1;
	testing.expect_value(t, e.body_set_axis_lock(&world, body, bad), e.Status.Invalid_Argument);
	pose: e.Rigid_Pose = e.pose();
	pose.orientation={math.sin(f32(0.2)), 0, 0, math.cos(f32(0.2))};
	testing.expect_value(t, e.body_set_pose(&world, body, pose), e.Status.Ok);
	for _ in 0..<80
	{
		testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
	}
	value: e.Body_Description;
	status: e.Status;
	value, status = e.body_get(&world, body);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, abs(value.pose.orientation.x)<0.001);
	lock.angular_axes={};
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	_, status=e.body_get_axis_lock(&world, body);
	testing.expect_value(t, status, e.Status.Not_Found);
}

@(test)
body_axis_lock_all_owned_failures_and_allocation_free_steps :: proc (t: ^testing.T)
{
	tracker:allocation_test_tracker;
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	// guarantee genuine allocation boundaries instead of vacuous failure
	// coverage from small buffers that happen to fit pre-existing pool blocks
	description.capacity.initial_constraints_per_type_batch=4096;
	description.threading.worker_count=2;
	description.solve.substeps=4;
	description.allocator=allocation_test_allocator(&tracker);
	world:e.World;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	body: e.Body_Handle;
	body, _ = body_control_test_body(&world);
	lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose());
	lock.linear_axes={.Y};
	lock.angular_axes={.X, .Y, .Z};
	before: int = tracker.requests;
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	requests: int = tracker.requests-before;
	testing.expect(t, requests>0);
	testing.expect_value(t, e.body_add_force(&world, body, {1, 2, 0}), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
	tracker.fail_from=tracker.requests+1;
	before=tracker.requests;
	for _ in 0..<100
	{
		testing.expect_value(t, e.body_add_force(&world, body, {1, 2, 0}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
	}
	testing.expect_value(t, tracker.requests, before);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &tracker);
	for fail in 1..=requests
	{
		tracker={};
		world=nil;
		testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		body, _=body_control_test_body(&world);
		tracker.fail_from=tracker.requests+fail;
		status: e.Status = e.body_set_axis_lock(&world, body, lock);
		testing.expect(t, status==.Ok || status==.Capacity_Missing);
		get_status: e.Status;
		_, get_status = e.body_get_axis_lock(&world, body);
		testing.expect_value(t, get_status, e.Status.Ok if status==.Ok else e.Status.Not_Found);
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		allocation_test_empty(t, &tracker);
	}
	// destroying an already-sleeping lock owner must not awaken the island
	// (and allocate) merely to remove rows the solver itself will dispose
	tracker={};
	world=nil;
	testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
	body, _=body_control_test_body(&world);
	testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
	body_axis_lock_test_sleep(t, &world, body);
	tracker.fail_from=tracker.requests+1;
	before=tracker.requests;
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, tracker.requests, before);
	allocation_test_empty(t, &tracker);
}

@(test)
body_axis_lock_restitution_keeps_unlocked_normal_response :: proc (t: ^testing.T)
{
	for workers in ([2]i32{1, 2})
	{
		d: e.World_Description = small_world_description();
		d.gravity={};
		d.damping={};
		d.threading.worker_count=workers;
		world:e.World;
		testing.expect_value(t, e.world_init(&world, d), e.Status.Ok);
		defer e.world_destroy(&world);
		testing.expect_value(t, e.world_enable_body_control(&world), e.Status.Ok);
		configuration: e.Restitution_Configuration = e.restitution_configuration_default();
		configuration.fallback={0.75, 1};
		testing.expect_value(t, e.world_enable_restitution(&world, configuration), e.Status.Ok);
		shape: e.Shape_Handle;
		shape_status: e.Status;
		shape, shape_status = e.shape_add(&world, e.sphere(1));
		testing.expect_value(t, shape_status, e.Status.Ok);
		static_status: e.Status;
		_, static_status = e.static_add(&world, e.static_body(shape, e.pose()), .None);
		testing.expect_value(t, static_status, e.Status.Ok);
		inertia: e.Body_Inertia;
		inertia, _ = e.shape_inertia(e.sphere(1), 1);
		body: e.Body_Handle;
		status: e.Status;
		body, status = e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({0, 2, 0}), e.velocity({2, -10, 0}), e.body_activity(-1, 255)));
		testing.expect_value(t, status, e.Status.Ok);
		lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({0, 2, 0}));
		lock.linear_axes={.X};
		testing.expect_value(t, e.body_set_axis_lock(&world, body, lock), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 1.0/64), e.Status.Ok);
		value: e.Body_Description;
		value, _ = e.body_get(&world, body);
		testing.expect(t, abs(value.velocity.linear.y-7.5)<0.0005);
		testing.expect(t, abs(value.velocity.linear.x)<0.3);
	}
}
