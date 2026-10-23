package precision_constraints_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Chain_Fixture :: struct
{
	bodies: [24]physics.Body_Handle,
	joints: [23]physics.Constraint_Handle,
}

populate_chain :: proc(t: ^testing.T, scene: ^Scene, fixture: ^Chain_Fixture, origin: f64)
{
	for index in 0 ..< 24
	{
		fixture.bodies[index] = add_body(t, scene, {origin + f64(index) * 1.002, -origin + 0.125, origin + 0.03125});
		body_motion(t, scene, fixture.bodies[index]).velocity.linear = {0.125, 0.25, -0.0625};
		if index > 0
		{
			pair: [4]physics.Body_Handle = {fixture.bodies[index - 1], fixture.bodies[index], {}, {}};
			description: physics.Ball_Socket = {{1, 0, 0}, {}, {angular_frequency=12.5, twice_damping_ratio=1.5}};
			status: physics.Physics_Status;
			fixture.joints[index - 1], status = physics.solver_add(&scene.simulation.solver, &pair, &description);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
	}
}

@(test)
precision_connected_fallback_chain_matches_near_world :: proc(t: ^testing.T)
{
	for workers in ([2]int{1, 4})
	{
		for substeps in ([2]i32{1, 4})
		{
			dispatcher: util.Thread_Dispatcher;
			testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, workers, 65536), util.Threading_Status.Ok);
			boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
			scenes: [2]Scene;
			fixtures: [2]Chain_Fixture;
			for variant in 0 ..< 2
			{
				scene_initialize(t, &scenes[variant], 64, substeps, 1);
				populate_chain(t, &scenes[variant], &fixtures[variant], f64(variant) * 1e9);
				solver: ^physics.Solver = &scenes[variant].simulation.solver;
				testing.expect_value(t, solver.fallback_batch_index, i32(1));
				testing.expect(t, solver.active_set.batches.memory[solver.fallback_batch_index].constraint_count > 0);
			}
			for frame in 0 ..< 256
			{
				for variant in 0 ..< 2
				{
					testing.expect_value(t, physics.simulation_timestep(scenes[variant].simulation, 1.0 / 64, boundary), physics.Physics_Status.Ok);
				}
				for index in 0 ..< 24
				{
					a: physics.Global_Position = body_position(t, &scenes[0], fixtures[0].bodies[index]);
					b: physics.Global_Position = body_position(t, &scenes[1], fixtures[1].bodies[index]);
					testing.expectf(t, abs(b.x - 1e9 - a.x) < 1e-4 && abs(b.y + 1e9 - a.y) < 1e-4 && abs(b.z - 1e9 - a.z) < 1e-4,
						"workers %v substeps %v frame %v body %v near %v far %v", workers, substeps, frame, index, a, b);
					expect_vector(t, body_motion(t, &scenes[1], fixtures[1].bodies[index]).velocity.linear,
						body_motion(t, &scenes[0], fixtures[0].bodies[index]).velocity.linear, 1e-3);
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
precision_joint_gathers_follow_compaction_sleep_wake_and_origin_shift :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 32, 1, 1);
	defer scene_dispose(t, &scene);
	spacer: physics.Body_Handle = add_body(t, &scene, {-123, 0, 0});
	fixture: Chain_Fixture;
	populate_chain(t, &scene, &fixture, 1e9);
	for _ in 0 ..< 4
	{
		testing.expect_value(t, physics.simulation_timestep(scene.simulation, 1.0 / 64), physics.Physics_Status.Ok);
	}
	positions: [24]physics.Global_Position;
	reactions: [23]physics.Constraint_Reaction_Sample;
	for index in 0 ..< 24
	{
		positions[index] = body_position(t, &scene, fixture.bodies[index]);
	}
	for index in 0 ..< 23
	{
		testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[index], 1.0 / 64, {}, &reactions[index]), physics.Physics_Status.Ok);
	}
	// removing the first unconstrained body relocates a live constrained body
	testing.expect_value(t, physics.bodies_remove(&scene.simulation.bodies, spacer), physics.Physics_Status.Ok);
	for index in 0 ..< 24
	{
		testing.expect_value(t, body_position(t, &scene, fixture.bodies[index]), positions[index]);
		scene.simulation.bodies.sets.memory[0].activity.memory[index].sleep_candidate = .Candidate;
	}
	sleeper: ^physics.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, physics.island_scaffold_clear(&sleeper.scaffold), physics.Physics_Status.Ok);
	body_count, constraint_count: int;
	can_sleep: physics.Reference_State;
	status: physics.Physics_Status;
	body_count, constraint_count, can_sleep, status = physics.island_sleeper_build_island(sleeper, &sleeper.scaffold, fixture.bodies[0]);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, can_sleep, physics.Reference_State.Present);
	testing.expect_value(t, body_count, 24);
	testing.expect_value(t, constraint_count, 23);
	testing.expect_value(t, physics.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), physics.Physics_Status.Ok);
	testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 0);
	testing.expect_value(t, physics.bodies_ensure_capacity(&scene.simulation.bodies, 256, 32, 8), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, fixture.bodies[12]), physics.Physics_Status.Ok);
	shift: util.Vector3 = {512, -256, 128};
	testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, shift), physics.Physics_Status.Ok);
	for index in 0 ..< 24
	{
		expected: physics.Global_Position = {positions[index].x - 512, positions[index].y + 256, positions[index].z - 128};
		testing.expect_value(t, body_position(t, &scene, fixture.bodies[index]), expected);
	}
	for index in 0 ..< 23
	{
		actual: physics.Constraint_Reaction_Sample;
		testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[index], 1.0 / 64, {}, &actual), physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reactions[index]);
	}
	// handle reuse and type-batch tail moves must preserve the remaining joints
	testing.expect_value(t, physics.solver_remove(&scene.simulation.solver, fixture.joints[7]), physics.Physics_Status.Ok);
	pair: [4]physics.Body_Handle = {fixture.bodies[7], fixture.bodies[8], {}, {}};
	description: physics.Ball_Socket = {{1, 0, 0}, {}, {angular_frequency=12.5, twice_damping_ratio=1.5}};
	replacement: physics.Constraint_Handle;
	replacement, status = physics.solver_add(&scene.simulation.solver, &pair, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, replacement, fixture.joints[7]);
	for _ in 0 ..< 16
	{
		testing.expect_value(t, physics.simulation_timestep(scene.simulation, 1.0 / 64), physics.Physics_Status.Ok);
	}
}
