package precision_snapshots_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_state_constraint_targets_preserve_bits_in_active_and_sleeping_storage :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
		position: e.Global_Position = {origin, -origin, origin};
		target: e.Global_Position = {origin + 0.001953125, -origin - 0.00390625, origin + 0.0078125};
		bodies: [2]e.Body_Handle = {add_body(t, &scene, position, {}), add_body(t, &scene, position, {})};
		handles: [2]e.Constraint_Handle = {add_servo(t, &scene, bodies[0], target), add_lock(t, &scene, bodies[1], target)};
		for handle, index in handles
		{
			state, status := e.constraint_get_global_target_state(&scene.world, handle);
			testing.expect_value(t, status, e.Status.Ok);
			expect_position_bits(t, state.position, target);
			testing.expect_value(t, state.kind, e.Global_Constraint_Target_Kind.Linear_Servo if index == 0 else .Axis_Lock);
			sleep_body(t, &scene, bodies[index]);
			set_index: i32 = scene.simulation.bodies.handle_to_location.memory[bodies[index].value].set_index;
			sleeping, sleeping_status := e.constraint_get_global_target_state(&scene.world, handle);
			testing.expect_value(t, sleeping_status, e.Status.Ok);
			testing.expect_value(t, sleeping, state);
			testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[bodies[index].value].set_index, set_index);
			testing.expect_value(t, p.simulation_set_constraint_global_target(scene.simulation, handle, position), e.Status.Ok);
			testing.expect_value(t, e.constraint_apply_global_target_state(&scene.world, handle, state), e.Status.Ok);
			read, read_status := e.constraint_get_global_target_state(&scene.world, handle);
			testing.expect_value(t, read_status, e.Status.Ok);
			testing.expect_value(t, read, state);
		}
		scene_dispose(t, &scene);
	}
}

@(test)
global_state_constraint_target_kind_rejection_does_not_wake_or_overwrite :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9, 0};
	body: e.Body_Handle = add_body(t, &scene, position, {});
	handle: e.Constraint_Handle = add_servo(t, &scene, body, position);
	sleep_body(t, &scene, body);
	set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
	for kind in ([3]e.Global_Constraint_Target_Kind{.Invalid, .Axis_Lock, e.Global_Constraint_Target_Kind(255)})
	{
		testing.expect_value(t, e.constraint_apply_global_target_state(&scene.world, handle, {position={42, 0, 0}, kind=kind}), e.Status.Invalid_Argument);
		read, status := e.constraint_get_global_target_state(&scene.world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		expect_position_bits(t, read.position, position);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
	}
	testing.expect_value(t, e.constraint_apply_global_target_state(&scene.world, handle,
		{position={math.nan_f64(), 0, 0}, kind=.Linear_Servo}), e.Status.Invalid_Argument);
	testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
	other: e.Body_Handle = add_body(t, &scene, position, {});
	distance, status := e.constraint_add_2(&scene.world, body, other,
		e.Center_Distance_Constraint{target_distance=1, spring_settings=e.spring_settings(10, 1)});
	testing.expect_value(t, status, e.Status.Ok);
	_, status = e.constraint_get_global_target_state(&scene.world, distance);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	_, status = e.constraint_get_global_target_state(&scene.world, {value=-1});
	testing.expect_value(t, status, e.Status.Not_Found);
	scene_dispose(t, &scene);
}

@(test)
global_state_kinematic_pending_absent_expired_roundtrip :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {origin, -origin, 0};
		body: e.Body_Handle = add_kinematic(t, &scene, position);
		absent, status := e.body_get_global_target_state(&scene.world, body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, absent.presence, e.Global_Target_Presence.Absent);
		testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, absent), e.Status.Ok);
		testing.expect(t, scene.simulation.body_control == nil);
		target: e.Global_Kinematic_Target_State = {pose={{origin + 0.001953125, -origin - 0.00390625, 0.0078125}, {w=1}}, presence=.Present};
		testing.expect(t, e.body_apply_global_target_state(&scene.world, body, target) != .Ok);
		testing.expect(t, scene.simulation.body_control == nil);
		testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
		sleep_body(t, &scene, body);
		set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
		testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, target, .Preserve_Sleep), e.Status.Ok);
		read, read_status := e.body_get_global_target_state(&scene.world, body);
		testing.expect_value(t, read_status, e.Status.Ok);
		testing.expect_value(t, read, target);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
		testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, absent), e.Status.Ok);
		read, read_status = e.body_get_global_target_state(&scene.world, body);
		testing.expect_value(t, read.presence, e.Global_Target_Presence.Absent);
		testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, target), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
		expect_body_position(t, &scene, body, target.pose.position);
		read, read_status = e.body_get_global_target_state(&scene.world, body);
		testing.expect_value(t, read_status, e.Status.Ok);
		testing.expect_value(t, read.presence, e.Global_Target_Presence.Absent);
		scene_dispose(t, &scene);
	}
}

@(test)
global_state_target_admission_rejects_wrong_mobility_invalid_presence_and_pose :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	position: e.Global_Position = {1e9 + 0.001953125, 0, 0};
	body: e.Body_Handle = add_kinematic(t, &scene, position);
	dynamic_body: e.Body_Handle = add_body(t, &scene, position, {});
	sleep_body(t, &scene, body);
	target: e.Global_Kinematic_Target_State = {pose={position, {w=1}}, presence=.Present};
	testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, target, .Preserve_Sleep), e.Status.Ok);
	set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
	_, status := e.body_get_global_target_state(&scene.world, dynamic_body);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, e.body_apply_global_target_state(&scene.world, dynamic_body, target), e.Status.Invalid_Argument);
	testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, {presence=e.Global_Target_Presence(255)}), e.Status.Invalid_Argument);
	target.pose.position.x = math.nan_f64();
	testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, target), e.Status.Invalid_Argument);
	target.pose.position = position;
	testing.expect_value(t, e.body_apply_global_target_state(&scene.world, body, target, e.Body_Control_Wake(255)), e.Status.Invalid_Argument);
	read, read_status := e.body_get_global_target_state(&scene.world, body);
	testing.expect_value(t, read_status, e.Status.Ok);
	expect_position_bits(t, read.pose.position, position);
	testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
	scene_dispose(t, &scene);
}
