package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

sleeping_world_description :: proc(worker_count: i32 = 1) -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	description.threading.worker_count = worker_count;
	return description;
}

sleeping_body_description :: proc(shape: entasis.Shape_Handle = {}) -> entasis.Body_Description
{
	inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
	if entasis.shape_handle_is_valid(shape)
	{
		return entasis.body_dynamic(
			shape,
			inertia,
			entasis.pose({}),
			{},
			entasis.body_activity(100, 1),
		);
	}
	return entasis.body_shapeless(
		inertia,
		entasis.pose({}),
		{},
		entasis.body_activity(100, 1),
	);
}

sleep_body_automatically :: proc(t: ^testing.T, world: ^entasis.World, handle: entasis.Body_Handle) -> bool
{
	for _ in 0 ..< 8
	{
		if !testing.expect_value(t, entasis.world_step(world, 1.0 / 60.0), entasis.Status.Ok)
		{
			return false;
		}
		sleeping, status := entasis.body_is_sleeping(world, handle);
		if status == .Ok && sleeping
		{
			return true;
		}
	}
	return testing.expect(t, false, "body did not enter a sleeping island");
}

@(test)
body_activation_queries_and_explicit_awaken_use_existing_island_storage :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, sleeping_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	handle, add_status := entasis.body_add(&world, sleeping_body_description());
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}

	state, state_status := entasis.body_activation_state(&world, handle);
	testing.expect_value(t, state_status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Body_Activation_State.Active);
	active, active_status := entasis.body_is_active(&world, handle);
	testing.expect_value(t, active_status, entasis.Status.Ok);
	testing.expect(t, active);

	if !sleep_body_automatically(t, &world, handle)
	{
		return;
	}
	state, state_status = entasis.body_activation_state(&world, handle);
	testing.expect_value(t, state_status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Body_Activation_State.Sleeping);
	active, active_status = entasis.body_is_active(&world, handle);
	testing.expect_value(t, active_status, entasis.Status.Ok);
	testing.expect(t, !active);

	testing.expect_value(t, entasis.body_awaken(&world, handle), entasis.Status.Ok);
	active, active_status = entasis.body_is_active(&world, handle);
	testing.expect_value(t, active_status, entasis.Status.Ok);
	testing.expect(t, active);
	// already-active awakening is a successful no-op
	testing.expect_value(t, entasis.body_awaken(&world, handle), entasis.Status.Ok);

	_, missing_status := entasis.body_activation_state(&world, entasis.body_handle_invalid());
	testing.expect_value(t, missing_status, entasis.Status.Not_Found);
}

@(test)
static_mutation_awakening_policy_is_explicit_for_sleeping_bodies :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, sleeping_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	sphere_shape, sphere_status := entasis.shape_add(&world, entasis.sphere(0.5));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok)
	{
		return;
	}
	box_shape, box_status := entasis.shape_add(&world, entasis.box(2, 2, 2));
	if !testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}

	body, body_status := entasis.body_add(&world, sleeping_body_description(sphere_shape));
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}
	if !sleep_body_automatically(t, &world, body)
	{
		return;
	}

	static_handle, static_status := entasis.static_add(
		&world,
		entasis.static_body(box_shape, entasis.pose({10, 0, 0})),
		.None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	still_sleeping, query_status := entasis.body_is_sleeping(&world, body);
	testing.expect_value(t, query_status, entasis.Status.Ok);
	testing.expect(t, still_sleeping);

	// moving an overlapping static without awakening keeps the island asleep
	testing.expect_value(
		t,
		entasis.static_set_pose(&world, static_handle, entasis.pose({}), .None),
		entasis.Status.Ok,
	);
	still_sleeping, query_status = entasis.body_is_sleeping(&world, body);
	testing.expect_value(t, query_status, entasis.Status.Ok);
	testing.expect(t, still_sleeping);

	// reapplying the same bounds with Overlaps awakens the sleeping island
	testing.expect_value(
		t,
		entasis.static_set_pose(&world, static_handle, entasis.pose({}), .Overlaps),
		entasis.Status.Ok,
	);
	active, active_status := entasis.body_is_active(&world, body);
	testing.expect_value(t, active_status, entasis.Status.Ok);
	testing.expect(t, active);
}

@(test)
sleep_and_awaken_state_matches_between_available_worker_counts :: proc(t: ^testing.T)
{
	one_worker: entasis.World;
	two_workers: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&one_worker, sleeping_world_description(1)), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&one_worker);
	if !testing.expect_value(
		t, entasis.world_init(&two_workers, sleeping_world_description(2)), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&two_workers);

	one_body, one_status := entasis.body_add(&one_worker, sleeping_body_description());
	two_body, two_status := entasis.body_add(&two_workers, sleeping_body_description());
	if !testing.expect_value(t, one_status, entasis.Status.Ok) ||
		!testing.expect_value(t, two_status, entasis.Status.Ok)
	{
		return;
	}

	for _ in 0 ..< 8
	{
		if !testing.expect_value(t, entasis.world_step(&one_worker, 1.0 / 60.0), entasis.Status.Ok) ||
			!testing.expect_value(t, entasis.world_step(&two_workers, 1.0 / 60.0), entasis.Status.Ok)
		{
			return;
		}
	}
	one_state, one_state_status := entasis.body_activation_state(&one_worker, one_body);
	two_state, two_state_status := entasis.body_activation_state(&two_workers, two_body);
	testing.expect_value(t, one_state_status, entasis.Status.Ok);
	testing.expect_value(t, two_state_status, entasis.Status.Ok);
	testing.expect_value(t, one_state, two_state);

	testing.expect_value(t, entasis.body_awaken(&one_worker, one_body), entasis.Status.Ok);
	testing.expect_value(t, entasis.body_awaken(&two_workers, two_body), entasis.Status.Ok);
	one_snapshot, one_get_status := entasis.body_get(&one_worker, one_body);
	two_snapshot, two_get_status := entasis.body_get(&two_workers, two_body);
	testing.expect_value(t, one_get_status, entasis.Status.Ok);
	testing.expect_value(t, two_get_status, entasis.Status.Ok);
	testing.expect_value(t, one_snapshot, two_snapshot);
}
