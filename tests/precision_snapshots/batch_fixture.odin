package precision_snapshots_tests

import "core:testing"
import e "entasis:entasis"

Snapshot_Scene :: struct
{
	scene: Scene,
	bodies: [3]e.Body_Handle,
	statics: [1]e.Static_Handle,
	constraints: [2]e.Constraint_Handle,
	kinematics: [1]e.Body_Handle,
	body_states: [3]e.Global_Body_State,
	static_states: [1]e.Global_Static_State,
	constraint_states: [2]e.Global_Constraint_Target_State,
	kinematic_states: [1]e.Global_Kinematic_Target_State,
}

snapshot_scene_initialize :: proc(t: ^testing.T, fixture: ^Snapshot_Scene)
{
	scene_initialize(t, &fixture.scene);
	scene: ^Scene = &fixture.scene;
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 - 0.00390625, 1e9 + 0.0078125};
	fixture.bodies[0] = add_body(t, scene, position, {});
	position.y += 4;
	fixture.bodies[1] = add_body(t, scene, position, {});
	position.y += 4;
	fixture.bodies[2] = add_kinematic(t, scene, position);
	fixture.kinematics[0] = fixture.bodies[2];
	position.y += 4;
	fixture.statics[0] = add_static(t, scene, position);
	fixture.constraints[0] = add_servo(t, scene, fixture.bodies[0], {1e9 + 0.125, -1e9, 1e9});
	fixture.constraints[1] = add_lock(t, scene, fixture.bodies[1], {1e9 + 0.25, -1e9 + 4, 1e9});
	position.y += 4;
	testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, fixture.kinematics[0], {position, {w=1}}), e.Status.Ok);
}

snapshot_selection :: proc "contextless" (fixture: ^Snapshot_Scene) -> e.Global_State_Selection
{
	return {fixture.bodies[:], fixture.statics[:], fixture.constraints[:], fixture.kinematics[:]};
}

snapshot_buffers :: proc "contextless" (fixture: ^Snapshot_Scene) -> e.Global_State_Snapshot
{
	return {fixture.body_states[:], fixture.static_states[:], fixture.constraint_states[:], fixture.kinematic_states[:]};
}

snapshot_capture :: proc(t: ^testing.T, fixture: ^Snapshot_Scene)
{
	counts, status := e.world_export_global_states(&fixture.scene.world, snapshot_selection(fixture), snapshot_buffers(fixture));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, counts, e.Global_State_Counts{3, 1, 2, 1});
}
