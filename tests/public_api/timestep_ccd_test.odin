package public_api_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

fixed_step_world_description :: proc(worker_count: i32 = 1) -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	description.threading.worker_count = worker_count;
	return description;
}

fixed_step_body :: proc() -> entasis.Body_Description
{
	return entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose({}),
		entasis.velocity({1, 0, 0}),
		entasis.body_activity(-1, 255),
	);
}

@(test)
ccd_factories_and_substep_description_map_exactly_to_low_level_values :: proc(t: ^testing.T)
{
	testing.expect_value(t, entasis.ccd_discrete(), physics.continuous_detection_discrete());
	testing.expect_value(t, entasis.ccd_passive(), physics.continuous_detection_passive());
	testing.expect_value(
		t,
		entasis.ccd_continuous(1e-4, 2e-4),
		physics.continuous_detection_continuous(1e-4, 2e-4),
	);

	solve := entasis.solve_description_substeps(4, 6, 32);
	testing.expect_value(t, solve.velocity_iterations, i32(6));
	testing.expect_value(t, solve.substeps, i32(4));
	testing.expect_value(t, solve.fallback_batch_threshold, i32(32));
}

@(test)
fixed_stepper_matches_direct_fixed_steps_and_preserves_fractional_alpha :: proc(t: ^testing.T)
{
	direct_world: entasis.World;
	fixed_world: entasis.World;
	description := fixed_step_world_description();
	if !testing.expect_value(t, entasis.world_init(&direct_world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&direct_world);
	if !testing.expect_value(t, entasis.world_init(&fixed_world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&fixed_world);

	direct_body, direct_status := entasis.body_add(&direct_world, fixed_step_body());
	fixed_body, fixed_status := entasis.body_add(&fixed_world, fixed_step_body());
	if !testing.expect_value(t, direct_status, entasis.Status.Ok) ||
		!testing.expect_value(t, fixed_status, entasis.Status.Ok)
	{
		return;
	}

	stepper := entasis.fixed_stepper(0.1, 4);
	steps, alpha, update_status := entasis.fixed_stepper_update(&stepper, &fixed_world, 0.25);
	testing.expect_value(t, update_status, entasis.Status.Ok);
	testing.expect_value(t, steps, 2);
	testing.expect(t, math.abs(alpha - 0.5) < 1e-5);
	for _ in 0 ..< 2
	{
		testing.expect_value(t, entasis.world_step(&direct_world, 0.1), entasis.Status.Ok);
	}
	direct_state, direct_get_status := entasis.body_get(&direct_world, direct_body);
	fixed_state, fixed_get_status := entasis.body_get(&fixed_world, fixed_body);
	testing.expect_value(t, direct_get_status, entasis.Status.Ok);
	testing.expect_value(t, fixed_get_status, entasis.Status.Ok);
	testing.expect_value(t, direct_state, fixed_state);

	steps, alpha, update_status = entasis.fixed_stepper_update(&stepper, &fixed_world, 0.05);
	testing.expect_value(t, update_status, entasis.Status.Ok);
	testing.expect_value(t, steps, 1);
	testing.expect(t, math.abs(alpha) < 1e-5);
	testing.expect_value(t, entasis.world_step(&direct_world, 0.1), entasis.Status.Ok);
	direct_state, direct_get_status = entasis.body_get(&direct_world, direct_body);
	fixed_state, fixed_get_status = entasis.body_get(&fixed_world, fixed_body);
	testing.expect_value(t, direct_get_status, entasis.Status.Ok);
	testing.expect_value(t, fixed_get_status, entasis.Status.Ok);
	testing.expect_value(t, direct_state, fixed_state);
}

@(test)
fixed_stepper_limits_backlog_and_rejects_invalid_policy :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, fixed_step_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	stepper := entasis.fixed_stepper(0.1, 2);
	steps, alpha, status := entasis.fixed_stepper_update(&stepper, &world, 0.55);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, steps, 2);
	testing.expect(t, math.abs(alpha - 0.5) < 1e-4);
	testing.expect(t, math.abs(stepper.accumulator - 0.05) < 1e-4);
	testing.expect_value(t, entasis.fixed_stepper_reset(&stepper), entasis.Status.Ok);
	testing.expect_value(t, stepper.accumulator, f32(0));

	invalid := entasis.fixed_stepper(0, 2);
	_, _, invalid_policy := entasis.fixed_stepper_update(&invalid, &world, 0.1);
	testing.expect_value(t, invalid_policy, entasis.Status.Invalid_Description);
	_, _, invalid_elapsed := entasis.fixed_stepper_update(&stepper, &world, -0.1);
	testing.expect_value(t, invalid_elapsed, entasis.Status.Invalid_Argument);
}

@(test)
fixed_step_sequence_matches_between_available_worker_counts :: proc(t: ^testing.T)
{
	one_worker: entasis.World;
	two_workers: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&one_worker, fixed_step_world_description(1)), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&one_worker);
	if !testing.expect_value(
		t, entasis.world_init(&two_workers, fixed_step_world_description(2)), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&two_workers);

	one_body, one_status := entasis.body_add(&one_worker, fixed_step_body());
	two_body, two_status := entasis.body_add(&two_workers, fixed_step_body());
	if !testing.expect_value(t, one_status, entasis.Status.Ok) ||
		!testing.expect_value(t, two_status, entasis.Status.Ok)
	{
		return;
	}

	one_stepper := entasis.fixed_stepper(1.0 / 60.0, 8);
	two_stepper := entasis.fixed_stepper(1.0 / 60.0, 8);
	elapsed_values := [8]f32{0.005, 0.012, 0.031, 0.009, 0.028, 0.017, 0.004, 0.046};
	for elapsed in elapsed_values
	{
		one_steps, one_alpha, one_step_status := entasis.fixed_stepper_update(
			&one_stepper, &one_worker, elapsed,
		);
		two_steps, two_alpha, two_step_status := entasis.fixed_stepper_update(
			&two_stepper, &two_workers, elapsed,
		);
		if !testing.expect_value(t, one_step_status, entasis.Status.Ok) ||
			!testing.expect_value(t, two_step_status, entasis.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, one_steps, two_steps);
		testing.expect_value(t, one_alpha, two_alpha);
	}

	one_state, one_get_status := entasis.body_get(&one_worker, one_body);
	two_state, two_get_status := entasis.body_get(&two_workers, two_body);
	testing.expect_value(t, one_get_status, entasis.Status.Ok);
	testing.expect_value(t, two_get_status, entasis.Status.Ok);
	testing.expect_value(t, one_state, two_state);
	testing.expect_value(t, one_stepper, two_stepper);
}

@(test)
continuous_facade_settings_drive_the_existing_sweep_continuation_path :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := fixed_step_world_description();
	description.solve = entasis.solve_description_substeps(1, 8, 8);
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	box_shape := entasis.box(1.5, 2, 0.7);
	shape, shape_status := entasis.shape_add(&world, box_shape);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	inertia, inertia_status := entasis.shape_inertia(box_shape, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	body_description := entasis.body_dynamic(
		shape,
		inertia,
		entasis.pose({-5, 0, 0}),
		entasis.velocity({20, 0, 0}, {0, 3, 0}),
		entasis.body_activity(-1, 255),
	);
	body_description.collidable.continuity = entasis.ccd_continuous(1e-4, 1e-4);
	body_description.collidable.maximum_speculative_margin = 0.1;
	body, body_status := entasis.body_add(&world, body_description);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}
	_, static_status := entasis.static_add(
		&world,
		entasis.static_body(shape, entasis.pose({})),
		.None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}

	testing.expect_value(t, entasis.world_step(&world, 0.5), entasis.Status.Ok);
	state, state_status := entasis.body_get(&world, body);
	testing.expect_value(t, state_status, entasis.Status.Ok);
	// without a successful sweep continuation this body would cross the target
	// during the half-second step. the existing CCD path keeps it on the
	// approach side and creates a contact constraint
	testing.expect(t, state.pose.position.x < 0);
	constraint_total, constraint_status := entasis.constraint_count(&world);
	testing.expect_value(t, constraint_status, entasis.Status.Ok);
	testing.expect_value(t, constraint_total, 1);
}
