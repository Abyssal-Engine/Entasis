package precision_constraints_tests

import "core:testing"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

exercise_relative_custom_simulation :: proc(t: ^testing.T, $dispatch: p.Constraint_Callback_Dispatch)
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
			when dispatch == .Contextual
			{
				probe_context: Relative_Probe_Context = {marker=7};
				binding: p.Contextual_Constraint_Binding = relative_probe_binding(&probe_context);
			}
			for variant in 0 ..< 2
			{
				scene: ^Scene = &scenes[variant];
				scene_initialize(t, scene, 32, substeps, 1);
				record: p.Constraint_Type_Record = relative_probe_record(2, 3, 3);
				when dispatch == .Contextual
				{
					record.dispatch = .Contextual;
					record.contextual = &binding;
				}
				testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
				base: f64 = f64(variant) * 1e9;
				a: p.Body_Handle = add_body(t, scene, {base + 0.125, -base, base + 0.25});
				for group in 0 ..< 17
				{
					fixtures[variant].bodies[group][0] = a;
					fixtures[variant].bodies[group][1] = add_body(t, scene,
						{base + 0.125 + f64(group + 1) / 512, -base + f64(group + 1) / 256, base + 0.25});
					description: Relative_Probe_Description = {2, 3, 3};
					status: p.Physics_Status;
					fixtures[variant].joints[group], status = p.solver_add_raw(&scene.simulation.solver, record.type_id,
						&fixtures[variant].bodies[group], &description);
					testing.expect_value(t, status, p.Physics_Status.Ok);
				}
				testing.expect_value(t, scene.simulation.solver.fallback_batch_index, i32(1));
				testing.expect(t, scene.simulation.solver.active_set.batches.memory[1].constraint_count > 0);
			}
			for frame in 0 ..< 64
			{
				for variant in 0 ..< 2
				{
					testing.expect_value(t, p.simulation_timestep(scenes[variant].simulation, 1.0 / 64, boundary), p.Physics_Status.Ok);
				}
				for group in 0 ..< 17
				{
					for body in 0 ..< 2
					{
						a: p.Global_Position = body_position(t, &scenes[0], fixtures[0].bodies[group][body]);
						b: p.Global_Position = body_position(t, &scenes[1], fixtures[1].bodies[group][body]);
						testing.expectf(t, abs(b.x - 1e9 - a.x) < 1e-4 && abs(b.y + 1e9 - a.y) < 1e-4 && abs(b.z - 1e9 - a.z) < 1e-4,
							"dispatch %v workers %v substeps %v frame %v group %v body %v near %v far %v", dispatch, workers, substeps, frame, group, body, a, b);
						expect_vector(t, body_motion(t, &scenes[1], fixtures[1].bodies[group][body]).velocity.linear,
							body_motion(t, &scenes[0], fixtures[0].bodies[group][body]).velocity.linear, 1e-4);
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
precision_custom_native_relative_full_simulation_fallback_substeps_workers :: proc(t: ^testing.T)
{
	exercise_relative_custom_simulation(t, .Native);
}

@(test)
precision_custom_contextual_relative_full_simulation_fallback_substeps_workers :: proc(t: ^testing.T)
{
	exercise_relative_custom_simulation(t, .Contextual);
}

@(test)
precision_custom_relative_prepared_callbacks_and_reactions_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	record: p.Constraint_Type_Record = relative_probe_record(4, 15, 15);
	testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
	fixture: Joint_Fixture;
	relative_probe_populate(t, &scene, record, &fixture, 1e9);
	probe: Relative_Reaction_Probe;
	provider: p.Constraint_Reaction_Provider = {read=relative_reaction_provider, user_context=&probe, position_space=.Body_A_Relative};
	calls: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		relative_probe_dispatch(t, &scene, &record, fixture.joints[0], .Warmstart, .Registered_Bundle, .Native);
		relative_probe_dispatch(t, &scene, &record, fixture.joints[0], .Solve, .Registered_Bundle, .Native);
		output: [8]p.Constraint_Reaction_Sample;
		testing.expect_value(t, p.constraint_capture_reaction_bundle(&scene.simulation.solver, fixture.joints[0], 0xff, 1, provider, &output), p.Physics_Status.Ok);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, calls);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

@(test)
precision_custom_relative_lifecycle_compaction_sleep_wake_growth_and_shift :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 32);
	defer scene_dispose(t, &scene);
	spacer: p.Body_Handle = add_body(t, &scene, {-123, 0, 0});
	record: p.Constraint_Type_Record = relative_probe_record(4, 15, 15);
	testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
	fixture: Joint_Fixture;
	relative_probe_populate(t, &scene, record, &fixture, 1e9, 3);
	testing.expect_value(t, p.bodies_remove(&scene.simulation.bodies, spacer), p.Physics_Status.Ok);
	testing.expect_value(t, p.solver_remove(&scene.simulation.solver, fixture.joints[1]), p.Physics_Status.Ok);
	for index in 0 ..< scene.simulation.bodies.sets.memory[0].count
	{
		scene.simulation.bodies.sets.memory[0].activity.memory[index].sleep_candidate = .Candidate;
	}
	sleeper: ^p.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, p.island_scaffold_clear(&sleeper.scaffold), p.Physics_Status.Ok);
	body_count, constraint_count: int;
	can_sleep: p.Reference_State;
	status: p.Physics_Status;
	body_count, constraint_count, can_sleep, status = p.island_sleeper_build_island(sleeper, &sleeper.scaffold, fixture.bodies[2][0]);
	testing.expect_value(t, status, p.Physics_Status.Ok);
	testing.expect_value(t, can_sleep, p.Reference_State.Present);
	testing.expect_value(t, body_count, 4);
	testing.expect_value(t, constraint_count, 1);
	testing.expect_value(t, p.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), p.Physics_Status.Ok);
	probe: Relative_Reaction_Probe;
	provider: p.Constraint_Reaction_Provider = {read=relative_reaction_provider, user_context=&probe, position_space=.Body_A_Relative};
	output: p.Constraint_Reaction_Sample = {maximum_force=123};
	before: p.Constraint_Reaction_Sample = output;
	testing.expect_value(t, p.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[2], 1, provider, &output), p.Physics_Status.Not_Found);
	testing.expect_value(t, output, before);
	testing.expect_value(t, p.bodies_ensure_capacity(&scene.simulation.bodies, 256, 32, 8), p.Physics_Status.Ok);
	testing.expect_value(t, p.simulation_translate_origin(scene.simulation, {512, -256, 128}), p.Physics_Status.Ok);
	testing.expect_value(t, p.island_awakener_awaken_body(&scene.simulation.awakener, fixture.bodies[2][0]), p.Physics_Status.Ok);
	relative_probe_dispatch(t, &scene, &record, fixture.joints[2], .Solve, .Registered_Bundle, .Native);
	testing.expect_value(t, p.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[2], 1, provider, &output), p.Physics_Status.Ok);
	for body in 0 ..< 4
	{
		expected: util.Vector3 = {f32(body) / 512, -f32(body) / 256, f32(body) / 128};
		testing.expect_value(t, output.forces[body].linear, expected);
		testing.expect_value(t, body_motion(t, &scene, fixture.bodies[2][body]).velocity.linear, expected);
	}
}
