package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

Property_Test_Value :: struct
{
	user_id: u64,
	gravity: entasis.Vector3,
}

property_test_world_description :: proc() -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	return description;
}

property_test_inertia :: proc "contextless" () -> entasis.Body_Inertia
{
	return {
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
}

@(test)
body_property_table_growth_and_handle_reuse_are_generation_protected :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, property_test_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	table: entasis.Body_Property_Table(Property_Test_Value);
	if !testing.expect_value(t, entasis.body_property_init(&table, 1), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.body_property_destroy(&table);

	first, add_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
		property_test_inertia(),
		entasis.pose({1, 0, 0}),
		{},
		entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	first_value := Property_Test_Value{user_id=17, gravity={0, -9.81, 0}};
	testing.expect_value(t, entasis.body_property_set(&table, first, first_value), entasis.Status.Ok);
	first_pointer, first_get_status := entasis.body_property_get(&table, first);
	if testing.expect_value(t, first_get_status, entasis.Status.Ok)
	{
		testing.expect_value(t, first_pointer^, first_value);
	}
	first_key, first_key_status := entasis.body_property_key(&table, first);
	if !testing.expect_value(t, first_key_status, entasis.Status.Ok)
	{
		return;
	}
	key_pointer, key_status := entasis.body_property_get_key(&table, first_key);
	if testing.expect_value(t, key_status, entasis.Status.Ok)
	{
		testing.expect_value(t, key_pointer^, first_value);
	}

	// updating an existing property preserves its lifetime key
	updated_value := Property_Test_Value{user_id=18, gravity={0, -4, 0}};
	testing.expect_value(t, entasis.body_property_set(&table, first, updated_value), entasis.Status.Ok);
	key_pointer, key_status = entasis.body_property_get_key(&table, first_key);
	if testing.expect_value(t, key_status, entasis.Status.Ok)
	{
		testing.expect_value(t, key_pointer^, updated_value);
	}

	// the property hook follows a successful structural removal. the numeric body
	// handle may then be reused, but the old property lifetime remains invalid
	testing.expect_value(t, entasis.body_remove(&world, first), entasis.Status.Ok);
	testing.expect_value(t, entasis.body_property_remove(&table, first), entasis.Status.Ok);
	_, removed_status := entasis.body_property_get(&table, first);
	testing.expect_value(t, removed_status, entasis.Status.Not_Found);
	_, stale_key_status := entasis.body_property_get_key(&table, first_key);
	testing.expect_value(t, stale_key_status, entasis.Status.Not_Found);

	second, second_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
		property_test_inertia(),
		entasis.pose({2, 0, 0}),
		{},
		entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, second_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, second, first);
	_, reused_before_set := entasis.body_property_get(&table, second);
	testing.expect_value(t, reused_before_set, entasis.Status.Not_Found);

	second_value := Property_Test_Value{user_id=99, gravity={0, -1, 0}};
	testing.expect_value(t, entasis.body_property_set(&table, second, second_value), entasis.Status.Ok);
	second_key, second_key_status := entasis.body_property_key(&table, second);
	if !testing.expect_value(t, second_key_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, second_key != first_key);
	_, stale_after_reuse := entasis.body_property_get_key(&table, first_key);
	testing.expect_value(t, stale_after_reuse, entasis.Status.Not_Found);
	second_pointer, second_get_status := entasis.body_property_get_key(&table, second_key);
	if testing.expect_value(t, second_get_status, entasis.Status.Ok)
	{
		testing.expect_value(t, second_pointer^, second_value);
	}

	// a sparse high handle forces geometric growth without losing prior entries
	high_handle := entasis.Body_Handle{value=127};
	high_value := Property_Test_Value{user_id=127, gravity={1, 2, 3}};
	testing.expect_value(t, entasis.body_property_set(&table, high_handle, high_value), entasis.Status.Ok);
	testing.expect(t, entasis.body_property_capacity(&table) >= 128);
	second_pointer, second_get_status = entasis.body_property_get(&table, second);
	if testing.expect_value(t, second_get_status, entasis.Status.Ok)
	{
		testing.expect_value(t, second_pointer^, second_value);
	}
}

@(test)
static_and_collidable_property_tables_keep_handle_spaces_separate :: proc(t: ^testing.T)
{
	body_table: entasis.Body_Property_Table(u32);
	static_table: entasis.Static_Property_Table(u32);
	collidable_table: entasis.Collidable_Property_Table(u32);
	if !testing.expect_value(t, entasis.body_property_init(&body_table), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.body_property_destroy(&body_table);
	if !testing.expect_value(t, entasis.static_property_init(&static_table), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.static_property_destroy(&static_table);
	if !testing.expect_value(t, entasis.collidable_property_init(&collidable_table), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.collidable_property_destroy(&collidable_table);

	testing.expect_value(t, entasis.body_property_ensure_capacity(&body_table, 8), entasis.Status.Ok);
	testing.expect_value(t, entasis.static_property_ensure_capacity(&static_table, 8), entasis.Status.Ok);
	testing.expect_value(
		t, entasis.collidable_property_ensure_capacity(&collidable_table, 8, 8), entasis.Status.Ok,
	);
	testing.expect(t, entasis.body_property_capacity(&body_table) >= 8);
	testing.expect(t, entasis.static_property_capacity(&static_table) >= 8);

	body := entasis.Body_Handle{value=3};
	static := entasis.Static_Handle{value=3};
	testing.expect_value(t, entasis.body_property_set(&body_table, body, 11), entasis.Status.Ok);
	testing.expect_value(t, entasis.static_property_set(&static_table, static, 22), entasis.Status.Ok);
	body_value, body_status := entasis.body_property_get(&body_table, body);
	static_value, static_status := entasis.static_property_get(&static_table, static);
	if testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		testing.expect_value(t, body_value^, u32(11));
	}
	if testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		testing.expect_value(t, static_value^, u32(22));
	}
	static_key, static_key_status := entasis.static_property_key(&static_table, static);
	if testing.expect_value(t, static_key_status, entasis.Status.Ok)
	{
		static_key_value, static_key_get_status := entasis.static_property_get_key(&static_table, static_key);
		if testing.expect_value(t, static_key_get_status, entasis.Status.Ok)
		{
			testing.expect_value(t, static_key_value^, u32(22));
		}
	}
	static_batch_handles := [2]entasis.Static_Handle{{value=4}, {value=5}};
	static_batch_values := [2]u32{44, 55};
	static_written, static_set_status := entasis.static_property_set_batch(
		&static_table, static_batch_handles[:], static_batch_values[:],
	);
	testing.expect_value(t, static_written, 2);
	testing.expect_value(t, static_set_status, entasis.Status.Ok);
	static_removed, static_remove_status := entasis.static_property_remove_batch(
		&static_table, static_batch_handles[:],
	);
	testing.expect_value(t, static_removed, 2);
	testing.expect_value(t, static_remove_status, entasis.Status.Ok);

	body_reference, body_reference_status := physics.collidable_reference_body(.Dynamic, body);
	static_reference, static_reference_status := physics.collidable_reference_static(static);
	if !testing.expect_value(t, body_reference_status, physics.Physics_Status.Ok)
	{
		return;
	}
	if !testing.expect_value(t, static_reference_status, physics.Physics_Status.Ok)
	{
		return;
	}
	testing.expect_value(
		t, entasis.collidable_property_set(&collidable_table, body_reference, 101), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.collidable_property_set(&collidable_table, static_reference, 202), entasis.Status.Ok,
	);
	body_collidable_value, body_collidable_status := entasis.collidable_property_get(
		&collidable_table, body_reference,
	);
	static_collidable_value, static_collidable_status := entasis.collidable_property_get(
		&collidable_table, static_reference,
	);
	if testing.expect_value(t, body_collidable_status, entasis.Status.Ok)
	{
		testing.expect_value(t, body_collidable_value^, u32(101));
	}
	if testing.expect_value(t, static_collidable_status, entasis.Status.Ok)
	{
		testing.expect_value(t, static_collidable_value^, u32(202));
	}
	collidable_values := [2]u32{303, 404};
	collidable_references := [2]entasis.Collidable_Reference{body_reference, static_reference};
	collidable_written, collidable_set_status := entasis.collidable_property_set_batch(
		&collidable_table, collidable_references[:], collidable_values[:],
	);
	testing.expect_value(t, collidable_written, 2);
	testing.expect_value(t, collidable_set_status, entasis.Status.Ok);

	body_key, body_key_status := entasis.collidable_property_key(&collidable_table, body_reference);
	if !testing.expect_value(t, body_key_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(
		t, entasis.collidable_property_remove(&collidable_table, body_reference), entasis.Status.Ok,
	);
	_, stale_status := entasis.collidable_property_get_key(&collidable_table, body_key);
	testing.expect_value(t, stale_status, entasis.Status.Not_Found);
	static_collidable_value, static_collidable_status = entasis.collidable_property_get(
		&collidable_table, static_reference,
	);
	if testing.expect_value(t, static_collidable_status, entasis.Status.Ok)
	{
		testing.expect_value(t, static_collidable_value^, u32(404));
	}
	removed_count, removed_status := entasis.collidable_property_remove_batch(
		&collidable_table, collidable_references[1:],
	);
	testing.expect_value(t, removed_count, 1);
	testing.expect_value(t, removed_status, entasis.Status.Ok);
	testing.expect_value(t, entasis.collidable_property_set(&collidable_table, body_reference, 505), entasis.Status.Ok);
	body_key, body_key_status = entasis.collidable_property_key(&collidable_table, body_reference);
	if !testing.expect_value(t, body_key_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.collidable_property_clear(&collidable_table), entasis.Status.Ok);
	_, cleared_collidable_status := entasis.collidable_property_get_key(&collidable_table, body_key);
	testing.expect_value(t, cleared_collidable_status, entasis.Status.Not_Found);
	testing.expect_value(t, entasis.static_property_clear(&static_table), entasis.Status.Ok);
	_, cleared_static_status := entasis.static_property_get_key(&static_table, static_key);
	testing.expect_value(t, cleared_static_status, entasis.Status.Not_Found);
}

@(test)
property_batches_clear_and_reinitialize_preserve_explicit_lifetime_contract :: proc(t: ^testing.T)
{
	table: entasis.Body_Property_Table(u64);
	if !testing.expect_value(t, entasis.body_property_init(&table, 2), entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.body_property_ensure_capacity(&table, 4), entasis.Status.Ok);

	handles := [3]entasis.Body_Handle{{value=0}, {value=4}, {value=9}};
	values := [3]u64{10, 40, 90};
	written, set_status := entasis.body_property_set_batch(&table, handles[:], values[:]);
	testing.expect_value(t, written, 3);
	testing.expect_value(t, set_status, entasis.Status.Ok);
	key, key_status := entasis.body_property_key(&table, handles[1]);
	if !testing.expect_value(t, key_status, entasis.Status.Ok)
	{
		return;
	}

	removed, remove_status := entasis.body_property_remove_batch(&table, handles[:2]);
	testing.expect_value(t, removed, 2);
	testing.expect_value(t, remove_status, entasis.Status.Ok);
	_, missing_status := entasis.body_property_get(&table, handles[0]);
	testing.expect_value(t, missing_status, entasis.Status.Not_Found);
	remaining, remaining_status := entasis.body_property_get(&table, handles[2]);
	if testing.expect_value(t, remaining_status, entasis.Status.Ok)
	{
		testing.expect_value(t, remaining^, u64(90));
	}

	testing.expect_value(t, entasis.body_property_clear(&table), entasis.Status.Ok);
	_, cleared_key_status := entasis.body_property_get_key(&table, key);
	testing.expect_value(t, cleared_key_status, entasis.Status.Not_Found);
	_, cleared_value_status := entasis.body_property_get(&table, handles[2]);
	testing.expect_value(t, cleared_value_status, entasis.Status.Not_Found);

	testing.expect_value(t, entasis.body_property_destroy(&table), entasis.Status.Ok);
	_, disposed_status := entasis.body_property_get(&table, handles[2]);
	testing.expect_value(t, disposed_status, entasis.Status.Disposed);
	testing.expect_value(t, entasis.body_property_init(&table, 1), entasis.Status.Ok);
	defer entasis.body_property_destroy(&table);
	testing.expect_value(t, entasis.body_property_set(&table, handles[1], 400), entasis.Status.Ok);
	_, old_epoch_status := entasis.body_property_get_key(&table, key);
	testing.expect_value(t, old_epoch_status, entasis.Status.Not_Found);
}
