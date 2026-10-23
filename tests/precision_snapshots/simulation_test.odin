package precision_snapshots_tests

import "core:testing"
import e "entasis:entasis"

@(test)
global_state_imported_servo_and_kinematic_targets_drive_near_far_simulations :: proc(t: ^testing.T)
{
	for workers in ([2]i32{1, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			scenes: [2]Scene;
			bodies: [2][2]e.Body_Handle;
			targets: [2]e.Global_Position;
			for variant in 0 ..< 2
			{
				scene: ^Scene = &scenes[variant];
				scene_initialize(t, scene, 32, substeps, workers);
				testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
				base: f64 = f64(variant) * 1e9;
				position: e.Global_Position = {base, -base, base};
				bodies[variant][0] = add_body(t, scene, position, {});
				bodies[variant][1] = add_kinematic(t, scene, e.Global_Position{position.x, position.y + 4, position.z});
				servo: e.Constraint_Handle = add_servo(t, scene, bodies[variant][0], position);
				states: [2]e.Global_Body_State;
				constraint_targets: [1]e.Global_Constraint_Target_State;
				kinematic_targets: [1]e.Global_Kinematic_Target_State;
				constraints: [1]e.Constraint_Handle = {servo};
				selection: e.Global_State_Selection = {bodies=bodies[variant][:], constraint_targets=constraints[:], kinematic_targets=bodies[variant][1:]};
				snapshot: e.Global_State_Snapshot = {bodies=states[:], constraint_targets=constraint_targets[:], kinematic_targets=kinematic_targets[:]};
				counts, status := e.world_export_global_states(&scene.world, selection, snapshot);
				testing.expect_value(t, status, e.Status.Ok);
				constraint_targets[0].position.x += 0.001953125;
				targets[variant] = e.Global_Position{position.x + 0.001953125, position.y + 4, position.z};
				kinematic_targets[0] = {pose={position=targets[variant], orientation={w=1}}, presence=.Present};
				counts, status = e.world_apply_global_states(&scene.world, selection, snapshot, .None);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect_value(t, counts, e.Global_State_Counts{2, 0, 1, 1});
			}
			for frame in 0 ..< 32
			{
				for variant in 0 ..< 2
				{
					testing.expect_value(t, e.world_step(&scenes[variant].world, 1.0 / 64), e.Status.Ok);
				}
				near, near_status := e.body_get_global_state(&scenes[0].world, bodies[0][0]);
				far, far_status := e.body_get_global_state(&scenes[1].world, bodies[1][0]);
				testing.expect_value(t, near_status, e.Status.Ok);
				testing.expect_value(t, far_status, e.Status.Ok);
				testing.expect(t, abs(far.pose.position.x - 1e9 - near.pose.position.x) < 2e-6);
				testing.expect(t, abs(far.velocity.linear.x - near.velocity.linear.x) < 2e-5);
				if frame == 0
				{
					testing.expect(t, near.velocity.linear.x > 0 && far.velocity.linear.x > 0);
					for variant in 0 ..< 2
					{
						expect_body_position(t, &scenes[variant], bodies[variant][1], targets[variant]);
					}
				}
			}
			for variant in 0 ..< 2
			{
				scene_dispose(t, &scenes[variant]);
			}
		}
	}
}
