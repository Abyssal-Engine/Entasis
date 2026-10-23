package precision_snapshots_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_state_batch_restores_bodies_before_absolute_and_kinematic_targets :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	scene: ^Scene = &fixture.scene;
	for body in fixture.bodies
	{
		testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {{0, 0, 0}, {w=1}}), e.Status.Ok);
	}
	testing.expect_value(t, e.static_set_global_pose(&scene.world, fixture.statics[0], {{20, 0, 0}, {w=1}}, .None), e.Status.Ok);
	for handle in fixture.constraints
	{
		testing.expect_value(t, p.simulation_set_constraint_global_target(scene.simulation, handle, {}), e.Status.Ok);
	}
	counts, status := e.world_apply_global_states(&scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{3, 1, 2, 1});
	for body, index in fixture.bodies
	{
		state, read_status := e.body_get_global_state(&scene.world, body);
		testing.expect_value(t, read_status, e.Status.Ok);
		testing.expect_value(t, state, fixture.body_states[index]);
	}
	state, read_status := e.static_get_global_state(&scene.world, fixture.statics[0]);
	testing.expect_value(t, read_status, e.Status.Ok);
	testing.expect_value(t, state, fixture.static_states[0]);
	for handle, index in fixture.constraints
	{
		target, target_status := e.constraint_get_global_target_state(&scene.world, handle);
		testing.expect_value(t, target_status, e.Status.Ok);
		testing.expect_value(t, target, fixture.constraint_states[index]);
	}
	target, target_status := e.body_get_global_target_state(&scene.world, fixture.kinematics[0]);
	testing.expect_value(t, target_status, e.Status.Ok);
	testing.expect_value(t, target, fixture.kinematic_states[0]);
	// a body-only import intentionally cancels the pending target
	testing.expect_value(t, e.body_apply_global_state(&scene.world, fixture.bodies[2], fixture.body_states[2]), e.Status.Ok);
	target, target_status = e.body_get_global_target_state(&scene.world, fixture.kinematics[0]);
	testing.expect_value(t, target.presence, e.Global_Target_Presence.Absent);
	scene_dispose(t, scene);
}

@(test)
global_state_batch_capacity_preflight_writes_nothing_in_any_stream :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	fixture.body_states[0].pose.position.x = 42;
	fixture.static_states[0].pose.position.x = 43;
	fixture.constraint_states[0].position.x = 44;
	fixture.kinematic_states[0].pose.position.x = 45;
	for stream in 0 ..< 4
	{
		short: e.Global_State_Snapshot = snapshot_buffers(&fixture);
		switch stream
		{
		case 0: short.bodies = short.bodies[:0];
		case 1: short.statics = short.statics[:0];
		case 2: short.constraint_targets = short.constraint_targets[:0];
		case 3: short.kinematic_targets = short.kinematic_targets[:0];
		}
		counts, status := e.world_export_global_states(&fixture.scene.world, snapshot_selection(&fixture), short);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, counts, e.Global_State_Counts{});
		testing.expect_value(t, fixture.body_states[0].pose.position.x, f64(42));
		testing.expect_value(t, fixture.static_states[0].pose.position.x, f64(43));
		testing.expect_value(t, fixture.constraint_states[0].position.x, f64(44));
		testing.expect_value(t, fixture.kinematic_states[0].pose.position.x, f64(45));
		before, _ := e.body_get_global_state(&fixture.scene.world, fixture.bodies[0]);
		counts, status = e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), short);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, counts, e.Global_State_Counts{});
		after, _ := e.body_get_global_state(&fixture.scene.world, fixture.bodies[0]);
		testing.expect_value(t, after, before);
	}
	counts, status := e.world_export_global_states(&fixture.scene.world, {}, {});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	counts, status = e.world_apply_global_states(&fixture.scene.world, {}, {});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	scene_dispose(t, &fixture.scene);
}

@(test)
global_state_batch_export_stops_at_invalid_record_and_preserves_tail :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	selection: e.Global_State_Selection = snapshot_selection(&fixture);
	for stream in 0 ..< 4
	{
		original_body: e.Body_Handle = fixture.bodies[1];
		original_static: e.Static_Handle = fixture.statics[0];
		original_constraint: e.Constraint_Handle = fixture.constraints[1];
		original_kinematic: e.Body_Handle = fixture.kinematics[0];
		fixture.body_states = {};
		fixture.static_states = {};
		fixture.constraint_states = {};
		fixture.kinematic_states = {};
		switch stream
		{
		case 0: fixture.bodies[1] = {value=-1};
		case 1: fixture.statics[0] = {value=-1};
		case 2: fixture.constraints[1] = {value=-1};
		case 3: fixture.kinematics[0] = {value=-1};
		}
		counts, status := e.world_export_global_states(&fixture.scene.world, selection, snapshot_buffers(&fixture));
		testing.expect_value(t, status, e.Status.Not_Found);
		expected: [4]e.Global_State_Counts = {{1, 0, 0, 0}, {3, 0, 0, 0}, {3, 1, 1, 0}, {3, 1, 2, 0}};
		testing.expect_value(t, counts, expected[stream]);
		if stream == 0
		{
			testing.expect_value(t, fixture.body_states[1], e.Global_Body_State{});
			testing.expect_value(t, fixture.body_states[2], e.Global_Body_State{});
		}
		if stream <= 1
		{
			testing.expect_value(t, fixture.static_states[0], e.Global_Static_State{});
		}
		if stream <= 2
		{
			testing.expect_value(t, fixture.constraint_states[1], e.Global_Constraint_Target_State{});
		}
		testing.expect_value(t, fixture.kinematic_states[0], e.Global_Kinematic_Target_State{});
		fixture.bodies[1] = original_body;
		fixture.statics[0] = original_static;
		fixture.constraints[1] = original_constraint;
		fixture.kinematics[0] = original_kinematic;
	}
	scene_dispose(t, &fixture.scene);
}

@(test)
global_state_batch_import_reports_only_committed_prefix_and_can_resume :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	original: [3]e.Global_Body_State = fixture.body_states;
	for &state in fixture.body_states[:]
	{
		state.pose.position.x += 8;
	}
	fixture.body_states[1].pose.position.x = math.nan_f64();
	counts, status := e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), .None);
	testing.expect(t, status != .Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{1, 0, 0, 0});
	expect_body_position(t, &fixture.scene, fixture.bodies[0], fixture.body_states[0].pose.position);
	expect_body_position(t, &fixture.scene, fixture.bodies[1], original[1].pose.position);
	expect_body_position(t, &fixture.scene, fixture.bodies[2], original[2].pose.position);
	fixture.body_states[1].pose.position.x = original[1].pose.position.x + 8;
	selection: e.Global_State_Selection = snapshot_selection(&fixture);
	buffers: e.Global_State_Snapshot = snapshot_buffers(&fixture);
	selection.bodies = selection.bodies[1:];
	buffers.bodies = buffers.bodies[1:];
	counts, status = e.world_apply_global_states(&fixture.scene.world, selection, buffers, .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{2, 1, 2, 1});
	for body, index in fixture.bodies
	{
		expect_body_position(t, &fixture.scene, body, fixture.body_states[index].pose.position);
	}
	scene_dispose(t, &fixture.scene);
}

@(test)
global_state_batch_duplicate_handles_and_unused_output_capacity_are_explicit :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	body: e.Body_Handle = add_body(t, &scene, {1e9 + 0.001953125, 0, 0}, {});
	handles: [3]e.Body_Handle = {body, body, body};
	output: [4]e.Global_Body_State;
	output[3].pose.position = {42, 43, 44};
	counts, status := e.world_export_global_states(&scene.world, {bodies=handles[:]}, {bodies=output[:]});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts.bodies, 3);
	testing.expect_value(t, output[0], output[1]);
	testing.expect_value(t, output[1], output[2]);
	testing.expect_value(t, output[3].pose.position, e.Global_Position{42, 43, 44});
	for index in 0 ..< 3
	{
		output[index].pose.position.x += f64(index) * 0.001953125;
	}
	counts, status = e.world_apply_global_states(&scene.world, {bodies=handles[:]}, {bodies=output[:]});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts.bodies, 3);
	expect_body_position(t, &scene, body, output[2].pose.position);
	scene_dispose(t, &scene);
}

@(test)
global_state_batch_rejects_invalid_policy_and_stepping_world_before_work :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	before: e.Global_Body_State = fixture.body_states[0];
	fixture.body_states[0].pose.position.x += 8;
	counts, status := e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), e.Awakening_Policy(255));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	counts, status = e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture), .None, e.Body_Control_Wake(255));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	expect_body_position(t, &fixture.scene, fixture.bodies[0], before.pose.position);
	fixture.scene.simulation.state = .Stepping;
	counts, status = e.world_export_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	counts, status = e.world_apply_global_states(&fixture.scene.world, snapshot_selection(&fixture), snapshot_buffers(&fixture));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, counts, e.Global_State_Counts{});
	fixture.scene.simulation.state = .Ready;
	testing.expect_value(t, fixture.body_states[0].pose.position.x, before.pose.position.x + 8);
	scene_dispose(t, &fixture.scene);
}
