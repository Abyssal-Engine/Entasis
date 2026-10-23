package precision_collisions_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// borrowed until the next scene mutation
@(private="package")
body_collidable :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> ^physics.Collidable
{
	location: physics.Body_Memory_Location;
	status: physics.Physics_Status;
	location, status = physics.bodies_resolve(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return &scene.simulation.bodies.sets.memory[location.set_index].collidables.memory[location.index];
}

body_position :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle) -> physics.Global_Position
{
	position: physics.Global_Position;
	status: physics.Physics_Status;
	position, status = physics.bodies_get_global_position(&scene.simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return position;
}

@(test)
precision_continuous_contacts_match_near_frame_before_impact :: proc(t: ^testing.T)
{
	kinds: [4]Shape_Kind = {.Sphere, .Box, .Hull, .Capsule};
	for kind in kinds
	{
		for completion in Completion
		{
			scenes: [2]Scene;
			for variant in 0 ..< 2
			{
				scene: ^Scene = &scenes[variant];
				scene_initialize(t, scene, completion);
				base: f64 = f64(variant) * 1e9;
				shape: physics.Typed_Index = add_shape(t, scene, kind);
				obstacle: physics.Typed_Index = add_shape(t, scene, .Box);
				handle: physics.Body_Handle = add_body(t, scene, shape, {base + 0.03125, -base + 3, base + 0.125});
				_ = add_static(t, scene, obstacle, {base + 0.03125, -base, base + 0.125});
				body_motion(t, scene, handle).velocity.linear = {0, -240, 0};
				body_collidable(t, scene, handle).continuity = physics.continuous_detection_continuous(1e-4, 1e-4);
				body_collidable(t, scene, handle).minimum_speculative_margin = 0.0625;
				body_collidable(t, scene, handle).maximum_speculative_margin = 0.0625;
				testing.expect_value(t, physics.simulation_predict_bounding_boxes(scene.simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
				testing.expect_value(t, physics.simulation_collision_detection(scene.simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
				testing.expectf(t, scene.simulation.narrow_phase.pair_cache.mapping.count == 1, "kind %v completion %v variant %v contact count %v", kind, completion, variant, scene.simulation.narrow_phase.pair_cache.mapping.count);
				continuation: physics.Narrow_Phase_CCD_Continuation = scene.simulation.narrow_phase.continuations.memory[0];
				testing.expect_value(t, continuation.kind, physics.Narrow_Phase_Continuation_Type.Continuous);
				testing.expect(t, continuation.t > 0 && continuation.t < 1.0 / 60.0);
				// collision detection must not advance either authoritative pose
				testing.expect_value(t, body_position(t, scene, handle).y, -base + 3);
			}
			expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
			testing.expect_value(t, scenes[0].simulation.narrow_phase.continuations.memory[0], scenes[1].simulation.narrow_phase.continuations.memory[0]);
			for variant in 0 ..< 2
			{
				scene_dispose(t, &scenes[variant]);
			}
		}
	}
}

@(test)
precision_contact_solver_substeps_keep_far_body_supported :: proc(t: ^testing.T)
{
	substeps: [2]i32 = {1, 4};
	for substep_count in substeps
	{
		scenes: [2]Scene;
		handles: [2]physics.Body_Handle;
		for variant in 0 ..< 2
		{
			scene: ^Scene = &scenes[variant];
			scene_initialize(t, scene);
			scene.simulation.default_pose_context.gravity = {0, -10, 0};
			scene.simulation.solve_description.substep_count = substep_count;
			base: f64 = f64(variant) * 1e9;
			shape: physics.Typed_Index = add_shape(t, scene, .Sphere);
			floor: physics.Typed_Index = add_shape(t, scene, .Box);
			handles[variant] = add_body(t, scene, shape, {base + 0.03125, base + 1.0, -base + 0.125});
			_ = add_static(t, scene, floor, {base + 0.03125, base, -base + 0.125});
			collidable: ^physics.Collidable = body_collidable(t, scene, handles[variant]);
			collidable.minimum_speculative_margin = 0.0625;
			collidable.maximum_speculative_margin = 0.0625;
		}
		for frame in 0 ..< 256
		{
			for variant in 0 ..< 2
			{
				testing.expect_value(t, physics.simulation_timestep(scenes[variant].simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
				testing.expect_value(t, scenes[variant].simulation.narrow_phase.pair_cache.mapping.count, 1);
			}
			near: physics.Global_Position = body_position(t, &scenes[0], handles[0]);
			far: physics.Global_Position = body_position(t, &scenes[1], handles[1]);
			// fixed positional budget is 0.1 mm, below the motivating 2 mm gap
			testing.expect(t, abs(far.x - 1e9 - near.x) < 1e-4);
			testing.expectf(t, abs(far.y - 1e9 - near.y) < 1e-4, "substeps %v frame %v near %v far %v", substep_count, frame, near.y, far.y - 1e9);
			testing.expect(t, abs(far.z + 1e9 - near.z) < 1e-4);
			velocity_a: physics.Body_Velocity = body_motion(t, &scenes[0], handles[0]).velocity;
			velocity_b: physics.Body_Velocity = body_motion(t, &scenes[1], handles[1]).velocity;
			testing.expect(t, util.vector3_length(util.vector3_subtract(velocity_a.linear, velocity_b.linear)) < 1e-3);
			testing.expect(t, util.vector3_length(util.vector3_subtract(velocity_a.angular, velocity_b.angular)) < 1e-3);
			testing.expect(t, far.y - 1e9 > 0.9 && far.y - 1e9 < 1.1);
		}
		for variant in 0 ..< 2
		{
			view: physics.Contact_Constraint_Data_View = contact_view(t, &scenes[variant], 0);
			impulse: f32;
			for index in 0 ..< int(view.impulse_count)
			{
				impulse += abs(view.impulses[index]);
			}
			testing.expect(t, impulse > 0);
			scene_dispose(t, &scenes[variant]);
		}
	}
}

@(test)
precision_sleep_wake_and_origin_shift_preserve_contact_cache :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	handle: physics.Body_Handle = add_body(t, &scene, shape, {1e9 + 0.03125, -1e9 + 0.9375, 1e9 + 0.125});
	_ = add_static(t, &scene, shape, {1e9 + 0.03125, -1e9, 1e9 + 0.125});
	testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
	before: physics.Contact_Constraint_Data_View = contact_view(t, &scene, 0);
	constraint: physics.Constraint_Handle = scene.simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
	position: physics.Global_Position = body_position(t, &scene, handle);
	scene.simulation.bodies.sets.memory[0].activity.memory[0].sleep_candidate = .Candidate;
	sleeper: ^physics.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, physics.island_scaffold_clear(&sleeper.scaffold), physics.Physics_Status.Ok);
	bodies, constraints: int;
	state: physics.Reference_State;
	status: physics.Physics_Status;
	bodies, constraints, state, status = physics.island_sleeper_build_island(sleeper, &sleeper.scaffold, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, state, physics.Reference_State.Present);
	testing.expect_value(t, bodies, 1);
	testing.expect_value(t, constraints, 1);
	testing.expect_value(t, physics.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, bodies, constraints), physics.Physics_Status.Ok);
	testing.expect_value(t, body_position(t, &scene, handle), position);
	testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, handle), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
	testing.expect_value(t, contact_view(t, &scene, 0), before);
	testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle, constraint);
	shift: util.Vector3 = {512, -256, 128};
	testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, shift), physics.Physics_Status.Ok);
	testing.expect_value(t, body_position(t, &scene, handle), physics.Global_Position{position.x - 512, position.y + 256, position.z - 128});
	testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
	testing.expect_value(t, contact_view(t, &scene, 0), before);
	testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle, constraint);
}
