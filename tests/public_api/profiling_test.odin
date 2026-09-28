package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

@(test)
profiling_is_opt_in_and_returns_existing_stage_snapshot :: proc(t: ^testing.T)
{
	description := small_world_description();
	testing.expect_value(t, description.profiling, false);
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	enabled, enabled_status := entasis.world_profile_enabled(&world);
	testing.expect_value(t, enabled_status, entasis.Status.Ok);
	testing.expect_value(t, enabled, false);
	_, snapshot_status := entasis.world_profile_snapshot(&world);
	testing.expect_value(t, snapshot_status, entasis.Status.Invalid_Description);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);

	description.profiling = true;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	snapshot, status := entasis.world_profile_snapshot(&world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, snapshot.step_index, u64(1));
	testing.expect(t, snapshot.trace_count > 0);
	testing.expect(t, snapshot.stage_counts[entasis.Profile_Stage.Timestep] > 0);
	testing.expect(t, snapshot.stage_durations_nanoseconds[entasis.Profile_Stage.Timestep] > 0);
	testing.expect_value(t, entasis.profile_stage_text(.Timestep), "timestep");
}

@(test)
world_stats_reports_pointer_free_storage_counts :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape, shape_status := entasis.shape_add(&world, entasis.sphere(0.5));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	_, static_status := entasis.static_add(
		&world,
		entasis.static_body(shape, entasis.pose({0, -2, 0})),
		.None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	inertia, inertia_status := entasis.shape_registered_inertia(&world, shape, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}
	a, a_status := entasis.body_add(
		&world,
		entasis.body_dynamic(shape, inertia, entasis.pose({-1, 0, 0}), {}, entasis.body_activity(-1, 255)),
	);
	b, b_status := entasis.body_add(
		&world,
		entasis.body_dynamic(shape, inertia, entasis.pose({1, 0, 0}), {}, entasis.body_activity(-1, 255)),
	);
	if !testing.expect_value(t, a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, b_status, entasis.Status.Ok)
	{
		return;
	}
	_, constraint_status := entasis.constraint_add_2(
		&world,
		a,
		b,
		entasis.Center_Distance_Constraint{
			target_distance=2,
			spring_settings=entasis.spring_settings(30, 1),
		},
	);
	if !testing.expect_value(t, constraint_status, entasis.Status.Ok)
	{
		return;
	}

	stats, status := entasis.world_stats(&world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, stats.step_index, u64(0));
	testing.expect_value(t, stats.active_bodies, 2);
	testing.expect_value(t, stats.sleeping_bodies, 0);
	testing.expect_value(t, stats.statics, 1);
	testing.expect_value(t, stats.active_constraints, 1);
	testing.expect_value(t, stats.sleeping_constraints, 0);
	testing.expect_value(t, stats.registered_shapes, 1);
	testing.expect(t, stats.registered_shape_types >= 5);
}
