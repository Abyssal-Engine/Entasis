package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

view_world_description :: proc() -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.capacity = {
		bodies=8,
		statics=4,
		inactive_body_sets=4,
		shapes_per_type=4,
		constraints=16,
		initial_constraints_per_type_batch=4,
		minimum_constraints_per_body=4,
		broad_phase_candidates=32,
		pairs=32,
		collision_child_pairs=32,
		inactive_pairs=16,
		pending_pairs_per_worker=8,
	};
	return description;
}

view_body_description :: proc(position: entasis.Vector3, sleep_threshold: f32 = -1) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		entasis.Body_Inertia{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose(position),
		{},
		entasis.body_activity(sleep_threshold, 1),
	);
}

view_low_level_simulation :: proc "contextless" (world: entasis.World) -> ^physics.Simulation
{
	return (^physics.Simulation)(rawptr(world));
}

@(test)
direct_views_match_dense_low_level_storage_without_copy :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, view_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	first, first_status := entasis.body_add(
		&world, view_body_description({1, 2, 3}),
	);
	if !testing.expect_value(t, first_status, entasis.Status.Ok)
	{
		return;
	}
	second, second_status := entasis.body_add(
		&world, view_body_description({4, 5, 6}),
	);
	if !testing.expect_value(t, second_status, entasis.Status.Ok)
	{
		return;
	}

	box_shape, shape_status := entasis.shape_add(&world, entasis.box(2, 2, 2));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	static_handle, static_status := entasis.static_add(
		&world, entasis.static_body(box_shape, entasis.pose({0, -1, 0})), .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}

	body_view, body_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}
	static_values, static_view_status := entasis.static_view(&world);
	if !testing.expect_value(t, static_view_status, entasis.Status.Ok)
	{
		return;
	}

	simulation := view_low_level_simulation(world);
	active := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	testing.expect(t, body_view.handles == active.index_to_handle.memory);
	testing.expect(t, body_view.dynamics == active.dynamics_state.memory);
	testing.expect(t, body_view.collidables == active.collidables.memory);
	testing.expect(t, body_view.activity == active.activity.memory);
	testing.expect_value(t, body_view.count, active.count);
	testing.expect(t, entasis.body_view_valid(&world, body_view));

	testing.expect(t, static_values.handles == simulation.statics.index_to_handle.memory);
	testing.expect(t, static_values.records == simulation.statics.statics.memory);
	testing.expect_value(t, static_values.count, simulation.statics.count);
	testing.expect(t, entasis.static_view_valid(&world, static_values));

	first_row, first_row_ok := entasis.active_body_row(body_view, 0);
	if !testing.expect(t, first_row_ok)
	{
		return;
	}
	testing.expect_value(t, first_row.handle, first);
	testing.expect(t, first_row.dynamics == &body_view.dynamics[0]);
	testing.expect(t, first_row.collidable == &body_view.collidables[0]);
	testing.expect(t, first_row.activity == &body_view.activity[0]);
	_, invalid_body_row := entasis.active_body_row(body_view, body_view.count);
	testing.expect(t, !invalid_body_row);

	second_index := 0;
	for index in 0 ..< body_view.count
	{
		if body_view.handles[index] == second
		{
			second_index = index;
			break;
		}
	}
	body_view.dynamics[second_index].motion.velocity = entasis.velocity(
		{7, 8, 9}, {1, 2, 3},
	);
	body_view.activity[second_index].sleep_threshold = 0.25;
	second_state, get_status := entasis.body_get(&world, second);
	if !testing.expect_value(t, get_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(
		t, second_state.velocity, entasis.velocity({7, 8, 9}, {1, 2, 3}),
	);
	testing.expect_value(t, second_state.activity.sleep_threshold, f32(0.25));
	testing.expect(t, entasis.body_view_valid(&world, body_view));

	static_row, static_row_ok := entasis.static_view_row(static_values, 0);
	if !testing.expect(t, static_row_ok)
	{
		return;
	}
	testing.expect_value(t, static_row.handle, static_handle);
	testing.expect(t, static_row.record == &static_values.records[0]);
	_, invalid_static_row := entasis.static_view_row(static_values, static_values.count);
	testing.expect(t, !invalid_static_row);
}

@(test)
view_epoch_invalidates_all_views_at_facade_mutation_boundaries :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := view_world_description();
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	first, first_status := entasis.body_add(&world, view_body_description({0, 1, 0}));
	if !testing.expect_value(t, first_status, entasis.Status.Ok)
	{
		return;
	}
	shape, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	ground, ground_status := entasis.static_add(
		&world, entasis.static_body(shape, entasis.pose({0, -1, 0})), .None,
	);
	if !testing.expect_value(t, ground_status, entasis.Status.Ok)
	{
		return;
	}

	body_view, _ := entasis.active_body_view(&world);
	static_values, _ := entasis.static_view(&world);
	_, add_status := entasis.body_add(&world, view_body_description({1, 1, 0}));
	testing.expect_value(t, add_status, entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	state, get_status := entasis.body_get(&world, first);
	if !testing.expect_value(t, get_status, entasis.Status.Ok)
	{
		return;
	}
	state.velocity = entasis.velocity({2, 0, 0});
	testing.expect_value(t, entasis.body_apply(&world, first, state), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	static_state, static_get_status := entasis.static_get(&world, ground);
	if !testing.expect_value(t, static_get_status, entasis.Status.Ok)
	{
		return;
	}
	static_state.pose = entasis.pose({0, -2, 0});
	testing.expect_value(
		t, entasis.static_apply(&world, ground, static_state, .None), entasis.Status.Ok,
	);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	grown := description.capacity;
	grown.bodies = 32;
	grown.statics = 16;
	testing.expect_value(t, entasis.world_ensure_capacity(&world, grown), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	testing.expect_value(t, entasis.body_remove(&world, first), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	testing.expect_value(t, entasis.static_remove(&world, ground, .None), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));

	body_view, _ = entasis.active_body_view(&world);
	static_values, _ = entasis.static_view(&world);
	testing.expect_value(t, entasis.world_clear(&world), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, body_view));
	testing.expect(t, !entasis.static_view_valid(&world, static_values));
}

@(test)
active_view_excludes_sleeping_bodies_and_reacquires_after_awakening :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, view_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	handle, add_status := entasis.body_add(
		&world, view_body_description({}, 100),
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	before_sleep, before_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, before_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, before_sleep.count, 1);

	for _ in 0 ..< 4
	{
		if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
		{
			return;
		}
	}
	testing.expect(t, !entasis.body_view_valid(&world, before_sleep));
	sleeping_view, sleeping_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, sleeping_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, sleeping_view.count, 0);

	testing.expect_value(
		t, entasis.body_set_velocity(&world, handle, entasis.velocity({1, 0, 0})),
		entasis.Status.Ok,
	);
	testing.expect(t, !entasis.body_view_valid(&world, sleeping_view));
	awake_view, awake_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, awake_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, awake_view.count, 1);
	testing.expect_value(t, awake_view.handles[0], handle);
}

@(test)
disposed_or_stepping_world_rejects_direct_view_acquisition :: proc(t: ^testing.T)
{
	world: entasis.World;
	_, disposed_body := entasis.active_body_view(&world);
	_, disposed_static := entasis.static_view(&world);
	testing.expect_value(t, disposed_body, entasis.Status.Disposed);
	testing.expect_value(t, disposed_static, entasis.Status.Disposed);

	if !testing.expect_value(
		t, entasis.world_init(&world, view_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	idle_view, idle_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, idle_status, entasis.Status.Ok)
	{
		return;
	}
	simulation := view_low_level_simulation(world);
	simulation.state = .Stepping;
	_, stepping_body := entasis.active_body_view(&world);
	_, stepping_static := entasis.static_view(&world);
	testing.expect_value(t, stepping_body, entasis.Status.Invalid_Argument);
	testing.expect_value(t, stepping_static, entasis.Status.Invalid_Argument);
	simulation.state = .Ready;
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	testing.expect(t, !entasis.body_view_valid(&world, idle_view));
}
