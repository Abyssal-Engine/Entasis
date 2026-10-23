package precision_constraints_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

exercise_integrated_joint :: proc(t: ^testing.T, description: ^$T, $body_count: int)
{
	for workers in ([2]int{1, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			dispatcher: util.Thread_Dispatcher;
			testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, workers, 65536), util.Threading_Status.Ok);
			boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
			scenes: [2]Scene;
			fixtures: [2]Joint_Fixture;
			for variant in 0 ..< 2
			{
				scene_initialize(t, &scenes[variant], 128, substeps);
				base: f64 = f64(variant) * 1e9;
				populate_joints(t, &scenes[variant], &fixtures[variant], {base, -base, base}, description, body_count);
			}
			for frame in 0 ..< 128
			{
				for variant in 0 ..< 2
				{
					testing.expect_value(t, physics.simulation_timestep(scenes[variant].simulation, 1.0 / 64, boundary), physics.Physics_Status.Ok);
				}
				for group in 0 ..< 17
				{
					for body in 0 ..< body_count
					{
						a: physics.Global_Position = body_position(t, &scenes[0], fixtures[0].bodies[group][body]);
						b: physics.Global_Position = body_position(t, &scenes[1], fixtures[1].bodies[group][body]);
						// fixed 0.1 mm position and 1 mm/s velocity budgets, below the 2 mm fixture
						testing.expectf(t, abs(b.x - 1e9 - a.x) < 1e-4 && abs(b.y + 1e9 - a.y) < 1e-4 && abs(b.z - 1e9 - a.z) < 1e-4,
							"%v workers %v substeps %v frame %v group %v body %v near %v far %v", typeid_of(T), workers, substeps, frame, group, body, a, b);
						va: physics.Body_Velocity = body_motion(t, &scenes[0], fixtures[0].bodies[group][body]).velocity;
						vb: physics.Body_Velocity = body_motion(t, &scenes[1], fixtures[1].bodies[group][body]).velocity;
						expect_vector(t, vb.linear, va.linear, 1e-3);
						expect_vector(t, vb.angular, va.angular, 1e-3);
					}
				}
			}
			for variant in 0 ..< 2
			{
				scene_dispose(t, &scenes[variant]);
			}
			testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
		}
	}
}

@(test)
precision_noncontact_full_simulation_substeps_and_workers :: proc(t: ^testing.T)
{
	spring: physics.Spring_Settings = {angular_frequency=12.5, twice_damping_ratio=1.5};
	ball: physics.Ball_Socket = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring};
	distance: physics.Center_Distance_Constraint = {2, spring};
	area: physics.Area_Constraint = {2, spring};
	volume: physics.Volume_Constraint = {2, spring};
	exercise_integrated_joint(t, &ball, 2);
	exercise_integrated_joint(t, &distance, 2);
	exercise_integrated_joint(t, &area, 3);
	exercise_integrated_joint(t, &volume, 4);
}

@(test)
precision_prepared_joint_solving_and_reactions_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	fixture: Joint_Fixture;
	description: physics.Ball_Socket = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, {angular_frequency=12.5, twice_damping_ratio=1.5}};
	populate_joints(t, &scene, &fixture, {1e9, -1e9, 1e9}, &description, 2);
	for _ in 0 ..< 16
	{
		testing.expect_value(t, physics.simulation_timestep(scene.simulation, 1.0 / 64), physics.Physics_Status.Ok);
	}
	calls: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		execute_stage(t, &scene, fixture.joints[0], .Cached_Ball_Socket, .Warmstart, physics.BALL_SOCKET_TYPE_ID, 2);
		execute_stage(t, &scene, fixture.joints[0], .Cached_Ball_Socket, .Solve, physics.BALL_SOCKET_TYPE_ID, 2);
		samples: [8]physics.Constraint_Reaction_Sample;
		testing.expect_value(t, physics.constraint_capture_reaction_bundle(&scene.simulation.solver, fixture.joints[0], 0xff, 1.0 / 64, {}, &samples), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, calls);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}
