package precision_targets_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_target_simulation_matches_near_world_with_substeps_and_workers :: proc(t: ^testing.T)
{
	for workers in ([2]int{1, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			dispatcher: util.Thread_Dispatcher;
			testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, workers, 65536), util.Threading_Status.Ok);
			boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
			scenes: [2]Scene;
			bodies: [2][33]physics.Body_Handle;
			for variant in 0 ..< 2
			{
				scene_initialize(t, &scenes[variant], 64, substeps, 1);
				base: f64 = f64(variant) * 1e9;
				for index in 0 ..< 33
				{
					body: physics.Body_Handle = add_body(t, &scenes[variant], {base + f64(index), -base + 0.25, base - 0.125});
					bodies[variant][index] = body;
					body_motion(t, &scenes[variant], body).velocity = {linear={0.125, -0.25, 0.0625}, angular={0.0625, -0.125, 0.25}};
					target: physics.Global_Position = {base + f64(index) + 0.001953125, -base + 0.28125, base - 0.15625};
					for kind in Target_Kind
					{
						handle: physics.Constraint_Handle = add_float_target(t, &scenes[variant], body, physics.world_precision_position_cache(target), kind, angular_axes=5, orientation={0, 0, 0.70710677, 0.70710677});
						testing.expect_value(t, physics.simulation_set_constraint_global_target(scenes[variant].simulation, handle, target), physics.Physics_Status.Ok);
					}
				}
				testing.expect(t, scenes[variant].simulation.solver.active_set.batches.memory[1].constraint_count > 0);
			}
			for frame in 0 ..< 128
			{
				for variant in 0 ..< 2
				{
					testing.expect_value(t, physics.simulation_timestep(scenes[variant].simulation, 1.0 / 64, boundary), physics.Physics_Status.Ok);
				}
				for index in 0 ..< 33
				{
					a: physics.Global_Position = body_position(t, &scenes[0], bodies[0][index]);
					b: physics.Global_Position = body_position(t, &scenes[1], bodies[1][index]);
					testing.expectf(t, abs(b.x - 1e9 - a.x) < 1e-4 && abs(b.y + 1e9 - a.y) < 1e-4 && abs(b.z - 1e9 - a.z) < 1e-4,
						"workers %v substeps %v frame %v body %v near %v far %v", workers, substeps, frame, index, a, b);
					expect_vector(t, body_motion(t, &scenes[1], bodies[1][index]).velocity.linear, body_motion(t, &scenes[0], bodies[0][index]).velocity.linear, 1e-3);
					expect_vector(t, body_motion(t, &scenes[1], bodies[1][index]).velocity.angular, body_motion(t, &scenes[0], bodies[0][index]).velocity.angular, 1e-3);
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
precision_target_prepared_solve_mutation_and_reaction_loop_does_not_allocate :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		scene: Scene;
		scene_initialize(t, &scene);
		body: physics.Body_Handle = add_body(t, &scene, {1e9, -1e9, 1e9});
		handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {1e9, -1e9, 1e9}, kind);
		target: physics.Global_Position = {1e9 + 0.001953125, -1e9, 1e9};
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
		testing.expect_value(t, execute_target_stage(t, &scene, handle, .Registered_Bundle, .Warmstart), physics.Physics_Status.Ok);
		allocations: i64 = scene.tracker.total_allocation_count;
		bytes: i64 = scene.tracker.current_memory_allocated;
		for _ in 0 ..< 1024
		{
			testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
			expect_target(t, &scene, handle, target);
			testing.expect_value(t, execute_target_stage(t, &scene, handle, .Registered_Bundle, .Solve), physics.Physics_Status.Ok);
			sample: physics.Constraint_Reaction_Sample;
			testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, handle, 1.0 / 64, {}, &sample), physics.Physics_Status.Ok);
		}
		testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
		testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
		scene_dispose(t, &scene);
	}
}
