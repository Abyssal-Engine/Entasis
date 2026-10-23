package precision_snapshots_tests

import "core:testing"
import e "entasis:entasis"

@(test)
global_state_copied_values_survive_growth_compaction_and_source_removal :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 4);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9 - 0.25};
	first: e.Body_Handle = add_body(t, &scene, {}, {});
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	static_first: e.Static_Handle = add_static(t, &scene, {});
	static_handle: e.Static_Handle = add_static(t, &scene, position);
	body_state, body_status := e.body_get_global_state(&scene.world, body);
	static_state, static_status := e.static_get_global_state(&scene.world, static_handle);
	testing.expect_value(t, body_status, e.Status.Ok);
	testing.expect_value(t, static_status, e.Status.Ok);
	for index in 0 ..< 40
	{
		add_body(t, &scene, {f64(index) * 4, 8, 0}, {});
		add_static(t, &scene, {f64(index) * 4, 8, 0});
	}
	testing.expect_value(t, e.body_remove(&scene.world, first), e.Status.Ok);
	testing.expect_value(t, e.static_remove(&scene.world, static_first, .None), e.Status.Ok);
	expect_body_position(t, &scene, body, position);
	expect_static_position(t, &scene, static_handle, position);
	testing.expect_value(t, e.body_remove(&scene.world, body), e.Status.Ok);
	testing.expect_value(t, e.static_remove(&scene.world, static_handle, .None), e.Status.Ok);
	replacement_body, status := e.body_add_global_state(&scene.world, body_state);
	testing.expect_value(t, status, e.Status.Ok);
	replacement_static, static_add_status := e.static_add_global_state(&scene.world, static_state, .None);
	testing.expect_value(t, static_add_status, e.Status.Ok);
	expect_body_position(t, &scene, replacement_body, position);
	expect_static_position(t, &scene, replacement_static, position);
	body_copy, _ := e.body_get_global_state(&scene.world, replacement_body);
	static_copy, _ := e.static_get_global_state(&scene.world, replacement_static);
	testing.expect_value(t, body_copy, body_state);
	testing.expect_value(t, static_copy, static_state);
	scene_dispose(t, &scene);
}

@(test)
global_state_origin_shift_requires_explicit_native_frame_conversion :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	original_bodies := fixture.body_states;
	original_statics := fixture.static_states;
	original_constraints := fixture.constraint_states;
	original_kinematics := fixture.kinematic_states;
	frame: e.Origin_Frame;
	delta: e.Vector3 = {1e9, -1e9, 1e9};
	testing.expect_value(t, e.world_shift_origin(&fixture.scene.world, &frame, delta), e.Status.Ok);
	// capture returns owned values. they do not silently change with the world
	testing.expect_value(t, fixture.body_states, original_bodies);
	testing.expect_value(t, fixture.static_states, original_statics);
	testing.expect_value(t, fixture.constraint_states, original_constraints);
	testing.expect_value(t, fixture.kinematic_states, original_kinematics);
	high_delta: e.Global_Position = {f64(delta.x), f64(delta.y), f64(delta.z)};
	for &value in fixture.body_states
	{
		value.pose.position.x -= high_delta.x;
		value.pose.position.y -= high_delta.y;
		value.pose.position.z -= high_delta.z;
	}
	for &value in fixture.static_states
	{
		value.pose.position.x -= high_delta.x;
		value.pose.position.y -= high_delta.y;
		value.pose.position.z -= high_delta.z;
	}
	for &value in fixture.constraint_states
	{
		value.position.x -= high_delta.x;
		value.position.y -= high_delta.y;
		value.position.z -= high_delta.z;
	}
	for &value in fixture.kinematic_states
	{
		value.pose.position.x -= high_delta.x;
		value.pose.position.y -= high_delta.y;
		value.pose.position.z -= high_delta.z;
	}
	counts, status := e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{3, 1, 2, 1});
	expected_bodies := fixture.body_states;
	expected_statics := fixture.static_states;
	expected_constraints := fixture.constraint_states;
	expected_kinematics := fixture.kinematic_states;
	snapshot_capture(t, &fixture);
	testing.expect_value(t, fixture.body_states, expected_bodies);
	testing.expect_value(t, fixture.static_states, expected_statics);
	testing.expect_value(t, fixture.constraint_states, expected_constraints);
	testing.expect_value(t, fixture.kinematic_states, expected_kinematics);
	scene_dispose(t, &fixture.scene);
}

@(test)
global_state_import_rejects_later_streams_with_exact_committed_counts :: proc(t: ^testing.T)
{
	for failing in 0 ..< 3
	{
		fixture: Snapshot_Scene;
		snapshot_scene_initialize(t, &fixture);
		snapshot_capture(t, &fixture);
		fixture.body_states[0].pose.position.x += 0.001953125;
		original_static := fixture.static_states[0];
		original_target := fixture.constraint_states[1];
		if failing == 0
		{
			fixture.static_states[0].shape = e.Shape_Handle{};
		}
		else if failing == 1
		{
			fixture.constraint_states[1].kind = .Linear_Servo;
		}
		else
		{
			fixture.kinematic_states[0].presence = e.Global_Target_Presence(255);
		}
		counts, status := e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), .None);
		testing.expect(t, status != .Ok);
		expected: e.Global_State_Counts;
		switch failing
		{
		case 0: expected = {3, 0, 0, 0};
		case 1: expected = {3, 1, 1, 0};
		case 2: expected = {3, 1, 2, 0};
		}
		testing.expect_value(t, counts, expected);
		expect_body_position(t, &fixture.scene, fixture.bodies[0], fixture.body_states[0].pose.position);
		actual_static, _ := e.static_get_global_state(&fixture.scene.world, fixture.statics[0]);
		testing.expect_value(t, actual_static, original_static);
		actual_target, _ := e.constraint_get_global_target_state(&fixture.scene.world, fixture.constraints[1]);
		testing.expect_value(t, actual_target, original_target);
		actual_kinematic, _ := e.body_get_global_target_state(&fixture.scene.world, fixture.kinematics[0]);
		testing.expect_value(t, actual_kinematic.presence, e.Global_Target_Presence.Absent);
		scene_dispose(t, &fixture.scene);
	}
}
