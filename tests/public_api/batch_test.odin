package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

batch_world_description :: proc() -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	return description;
}

batch_body_description :: proc(index: int) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose({f32(index), 1, 0}),
		{},
		entasis.body_activity(-1, 255),
	);
}

@(test)
body_batch_preserves_input_order_and_invalidates_views_once :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	first, first_status := entasis.body_add(&world, batch_body_description(-1));
	if !testing.expect_value(t, first_status, entasis.Status.Ok)
	{
		return;
	}
	view, view_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, view_status, entasis.Status.Ok)
	{
		return;
	}

	descriptions: [32]entasis.Body_Description;
	handles: [32]entasis.Body_Handle;
	for index in 0 ..< len(descriptions)
	{
		descriptions[index] = batch_body_description(index);
	}
	written, status := entasis.body_add_batch(
		&world, descriptions[:], handles[:],
	);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, written, len(descriptions));
	testing.expect(t, !entasis.body_view_valid(&world, view));
	for index in 0 ..< written
	{
		state, get_status := entasis.body_get(&world, handles[index]);
		testing.expect_value(t, get_status, entasis.Status.Ok);
		testing.expect_value(t, state.pose.position.x, f32(index));
		if index > 0
		{
			testing.expect_value(
				t, handles[index].value, handles[index - 1].value + 1,
			);
		}
	}

	all_handles: [33]entasis.Body_Handle;
	all_handles[0] = first;
	for index in 0 ..< written
	{
		all_handles[index + 1] = handles[index];
	}
	removed, remove_status := entasis.body_remove_batch(&world, all_handles[:]);
	testing.expect_value(t, remove_status, entasis.Status.Ok);
	testing.expect_value(t, removed, len(all_handles));
}

@(test)
body_batch_reports_committed_prefix_on_failure :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	descriptions := [3]entasis.Body_Description{
		batch_body_description(0),
		batch_body_description(1),
		batch_body_description(2),
	};
	descriptions[1].pose.orientation = {};
	handles: [3]entasis.Body_Handle;
	written, status := entasis.body_add_batch(&world, descriptions[:], handles[:]);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	testing.expect(t, entasis.body_handle_is_valid(handles[0]));
	testing.expect(t, !entasis.body_handle_is_valid(handles[1]));
	testing.expect(t, !entasis.body_handle_is_valid(handles[2]));
	_, first_status := entasis.body_get(&world, handles[0]);
	testing.expect_value(t, first_status, entasis.Status.Ok);
}

@(test)
shape_and_static_batches_use_caller_owned_outputs :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shapes := [4]entasis.Box{
		entasis.box(1, 1, 1),
		entasis.box(2, 1, 1),
		entasis.box(3, 1, 1),
		entasis.box(4, 1, 1),
	};
	shape_handles: [4]entasis.Shape_Handle;
	shape_count, shape_status := entasis.shape_add_batch(
		&world, shapes[:], shape_handles[:],
	);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, shape_count, len(shapes));

	static_descriptions: [4]entasis.Static_Description;
	for index in 0 ..< len(static_descriptions)
	{
		static_descriptions[index] = entasis.static_body(
			shape_handles[index], entasis.pose({f32(index), 0, 0}),
		);
	}
	static_handles: [4]entasis.Static_Handle;
	static_count, static_status := entasis.static_add_batch(
		&world, static_descriptions[:], static_handles[:], .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, static_count, len(static_descriptions));

	for index in 0 ..< len(static_descriptions)
	{
		static_descriptions[index].pose.position.y = f32(index + 10);
	}
	applied, apply_status := entasis.static_apply_batch(
		&world, static_handles[:], static_descriptions[:], .None,
	);
	testing.expect_value(t, apply_status, entasis.Status.Ok);
	testing.expect_value(t, applied, len(static_handles));
	for index in 0 ..< len(static_handles)
	{
		state, get_status := entasis.static_get(&world, static_handles[index]);
		testing.expect_value(t, get_status, entasis.Status.Ok);
		testing.expect_value(t, state.pose.position.y, f32(index + 10));
	}

	removed, remove_status := entasis.static_remove_batch(
		&world, static_handles[:], .None,
	);
	testing.expect_value(t, remove_status, entasis.Status.Ok);
	testing.expect_value(t, removed, len(static_handles));
	shape_removed, shape_remove_status := entasis.shape_remove_batch(
		&world, shape_handles[:],
	);
	testing.expect_value(t, shape_remove_status, entasis.Status.Ok);
	testing.expect_value(t, shape_removed, len(shape_handles));
}

@(test)
command_buffer_applies_exact_input_order_and_routes_results :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape_handle, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	existing, existing_status := entasis.body_add(&world, batch_body_description(-1));
	if !testing.expect_value(t, existing_status, entasis.Status.Ok)
	{
		return;
	}

	updated := batch_body_description(100);
	added := batch_body_description(200);
	static_description := entasis.static_body(shape_handle, entasis.pose({3, 4, 5}));
	commands := [4]entasis.Command{
		entasis.command_body_apply(existing, updated),
		entasis.command_body_add(added, 0),
		entasis.command_static_add(static_description, 0, .None),
		entasis.command_body_remove(existing),
	};
	statuses: [4]entasis.Status;
	body_results: [1]entasis.Body_Handle;
	static_results: [1]entasis.Static_Handle;
	buffer := entasis.command_buffer(
		commands[:], statuses[:], body_results[:], static_results[:],
	);
	processed, status := entasis.world_apply_commands(&world, &buffer);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, processed, len(commands));
	for command_status in statuses
	{
		testing.expect_value(t, command_status, entasis.Status.Ok);
	}

	_, removed_status := entasis.body_get(&world, existing);
	testing.expect_value(t, removed_status, entasis.Status.Not_Found);
	added_state, added_status := entasis.body_get(&world, body_results[0]);
	testing.expect_value(t, added_status, entasis.Status.Ok);
	testing.expect_value(t, added_state.pose.position.x, f32(200));
	static_state, static_status := entasis.static_get(&world, static_results[0]);
	testing.expect_value(t, static_status, entasis.Status.Ok);
	testing.expect_value(t, static_state, static_description);
}

@(test)
command_buffer_stops_at_first_failure_and_leaves_suffix_unwritten :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	commands := [3]entasis.Command{
		entasis.command_body_add(batch_body_description(1), 0),
		entasis.command_body_remove(entasis.body_handle_invalid()),
		entasis.command_body_add(batch_body_description(2), 1),
	};
	statuses := [3]entasis.Status{.Disposed, .Disposed, .Disposed};
	body_results: [2]entasis.Body_Handle;
	buffer := entasis.command_buffer(commands[:], statuses[:], body_results[:]);
	processed, status := entasis.world_apply_commands(&world, &buffer);
	testing.expect_value(t, processed, 1);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, statuses[0], entasis.Status.Ok);
	testing.expect_value(t, statuses[1], entasis.Status.Not_Found);
	testing.expect_value(t, statuses[2], entasis.Status.Disposed);
	testing.expect(t, entasis.body_handle_is_valid(body_results[0]));
	testing.expect(t, !entasis.body_handle_is_valid(body_results[1]));
}

@(test)
batch_prefix_matches_scalar_low_level_order :: proc(t: ^testing.T)
{
	batch_world: entasis.World;
	scalar_world: entasis.World;
	description := batch_world_description();
	if !testing.expect_value(t, entasis.world_init(&batch_world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&batch_world);
	if !testing.expect_value(t, entasis.world_init(&scalar_world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&scalar_world);

	descriptions: [16]entasis.Body_Description;
	batch_handles: [16]entasis.Body_Handle;
	scalar_handles: [16]entasis.Body_Handle;
	for index in 0 ..< len(descriptions)
	{
		descriptions[index] = batch_body_description(index);
		handle, status := entasis.body_add(&scalar_world, descriptions[index]);
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return;
		}
		scalar_handles[index] = handle;
	}
	written, status := entasis.body_add_batch(
		&batch_world, descriptions[:], batch_handles[:],
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, written, len(descriptions));
	for index in 0 ..< len(descriptions)
	{
		testing.expect_value(t, batch_handles[index], scalar_handles[index]);
		batch_state, batch_status := entasis.body_get(&batch_world, batch_handles[index]);
		scalar_state, scalar_status := entasis.body_get(&scalar_world, scalar_handles[index]);
		testing.expect_value(t, batch_status, entasis.Status.Ok);
		testing.expect_value(t, scalar_status, entasis.Status.Ok);
		testing.expect_value(t, batch_state, scalar_state);
	}

}

@(test)
command_buffer_rejects_duplicate_result_slots_without_mutation :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	commands := [2]entasis.Command{
		entasis.command_body_add(batch_body_description(1), 0),
		entasis.command_body_add(batch_body_description(2), 0),
	};
	body_results: [1]entasis.Body_Handle;
	buffer := entasis.command_buffer(commands[:], nil, body_results[:]);
	processed, status := entasis.world_apply_commands(&world, &buffer);
	testing.expect_value(t, processed, 0);
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
	testing.expect(t, !entasis.body_handle_is_valid(body_results[0]));
	view, view_status := entasis.active_body_view(&world);
	testing.expect_value(t, view_status, entasis.Status.Ok);
	testing.expect_value(t, view.count, 0);
}
