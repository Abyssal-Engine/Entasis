package precision_broadphase_tests

import "core:testing"
import p "entasis:entasis_physics"

@(test)
precision_pair_admission_distinguishes_closed_touch_and_two_mm_gaps :: proc(t: ^testing.T)
{
	offsets: [5]f64 = {0, 10000, 1000000, 1000000000, -1000000000};
	for offset in offsets
	{
		for static_side in 0 ..< 2
		{
			scene: Scene;
			scene_initialize(t, &scene, 16, 1);
			origin: p.Global_Position = {offset, offset, -offset};
			_ = add_body(t, &scene, scene.shapes[0], origin);
			body: p.Body_Handle;
			stationary: p.Static_Handle;
			if static_side == 0
			{
				body = add_body(t, &scene, scene.shapes[0], origin);
			}
			else
			{
				stationary = add_static(t, &scene, scene.shapes[0], origin);
			}
			for axis in 0 ..< 3
			{
				for gap in -1 ..= 1
				{
					position: p.Global_Position = origin;
					distance: f64 = 0.125 + f64(gap) / 512;
					switch axis
					{
					case 0: position.x += distance;
					case 1: position.y += distance;
					case 2: position.z += distance;
					}
					if static_side == 0
					{
						testing.expect_value(t, p.bodies_set_global_position(&scene.simulation.bodies, body, position), p.Physics_Status.Ok);
					}
					else
					{
						testing.expect_value(t, p.statics_set_global_position(&scene.simulation.statics, stationary, position), p.Physics_Status.Ok);
					}
					testing.expect_value(t, p.broad_phase_update_bounds(&scene.simulation.broad_phase), p.Physics_Status.Ok);
					testing.expect_value(t, pair_count(t, &scene), 0 if gap > 0 else 1);
				}
			}
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_pair_admission_preserves_directional_prediction_and_continuous_reach :: proc(t: ^testing.T)
{
	modes: [3]p.Continuous_Detection_Mode = {.Discrete, .Passive, .Continuous};
	for mode in modes
	{
		scene: Scene;
		scene_initialize(t, &scene, 16, 1);
		origin: p.Global_Position = {1e9, -1e9, 1e9};
		body: p.Body_Handle = add_body(t, &scene, scene.shapes[0], origin);
		active: ^p.Body_Set = &scene.simulation.bodies.sets.memory[0];
		active.dynamics_state.memory[0].motion.velocity.linear = {8, 0, 0};
		active.collidables.memory[0].continuity.mode = mode;
		reach: f64 = 0.0625 if mode == .Discrete else 1;
		_ = add_static(t, &scene, scene.shapes[0], {origin.x + 0.125 + reach, origin.y, origin.z});
		_ = add_static(t, &scene, scene.shapes[0], {origin.x + 0.125 + reach + 1.0/512, origin.y, origin.z});
		_ = add_static(t, &scene, scene.shapes[0], {origin.x - 0.125 - 1.0/512, origin.y, origin.z});
		_ = add_static(t, &scene, scene.shapes[0], {origin.x, origin.y + 0.125 + 1.0/512, origin.z});
		testing.expect_value(t, p.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), p.Physics_Status.Ok);
		testing.expect_value(t, pair_count(t, &scene), 1);
		expect_precise_exact(t, body_precise_bounds(t, &scene, body), origin,
			{min={-0.0625, -0.0625, -0.0625}, max={f32(0.0625 + reach), 0.0625, 0.0625}});
		scene_dispose(t, &scene);
	}
}

@(test)
precision_pair_admission_uses_callback_prediction_without_committing_velocity :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 16, 1);
	defer scene_dispose(t, &scene);
	origin: p.Global_Position = {1e9, -1e9, 1e9};
	body: p.Body_Handle = add_body(t, &scene, scene.shapes[0], origin);
	scene.simulation.bodies.sets.memory[0].collidables.memory[0].continuity.mode = .Continuous;
	_ = add_static(t, &scene, scene.shapes[0], {origin.x + 0.375, origin.y, origin.z});
	_ = add_static(t, &scene, scene.shapes[0], {origin.x, origin.y - 0.25, origin.z});
	_ = add_static(t, &scene, scene.shapes[0], {origin.x - 0.125 - 1.0/512, origin.y, origin.z});
	record: Callback_Record;
	scene.simulation.integrator.callbacks.integrate_velocity = predict_velocity;
	scene.simulation.integrator.callbacks.user_context = &record;
	testing.expect_value(t, p.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125, &scene.simulation.integrator), p.Physics_Status.Ok);
	testing.expect_value(t, record.visits[0], u32(1));
	testing.expect_value(t, pair_count(t, &scene), 2);
	testing.expect_value(t, scene.simulation.bodies.sets.memory[0].dynamics_state.memory[0].motion.velocity, p.Body_Velocity{});
	expect_precise_exact(t, body_precise_bounds(t, &scene, body), origin,
		{min={-0.0625, -0.1875, -0.0625}, max={0.3125, 0.0625, 0.125}});
}

@(test)
precision_pair_admission_survives_origin_shifts_without_importing_coarse_envelopes :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 16, 1);
	defer scene_dispose(t, &scene);
	origin: p.Global_Position = {1e9, -1e9, 1e9};
	body: p.Body_Handle = add_body(t, &scene, scene.shapes[0], origin);
	_ = add_static(t, &scene, scene.shapes[0], {origin.x + 0.125 + 1.0/512, origin.y, origin.z});
	testing.expect_value(t, pair_count(t, &scene), 0);
	before: p.Global_Bounds = body_precise_bounds(t, &scene, body);
	for _ in 0 ..< 4
	{
		testing.expect_value(t, p.simulation_translate_origin(scene.simulation, {1e9, -1e9, 1e9}), p.Physics_Status.Ok);
		testing.expect_value(t, pair_count(t, &scene), 0);
		testing.expect_value(t, p.simulation_translate_origin(scene.simulation, {-1e9, 1e9, -1e9}), p.Physics_Status.Ok);
		testing.expect_value(t, pair_count(t, &scene), 0);
		testing.expect_value(t, body_precise_bounds(t, &scene, body), before);
	}
}
