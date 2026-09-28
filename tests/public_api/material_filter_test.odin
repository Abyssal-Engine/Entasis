package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

Filter_Callback_State :: struct
{
	pair_calls:  i32,
	child_calls: i32,
}

filter_test_pair :: proc "contextless" (
	user_context: rawptr,
	a, b: entasis.Collidable_Reference,
) -> bool
{
	_, _ = a, b;
	state := (^Filter_Callback_State)(user_context);
	if state == nil
	{
		return false;
	}
	state.pair_calls += 1;
	return true;
}

filter_test_child :: proc "contextless" (
	user_context: rawptr,
	a, b: entasis.Collidable_Reference,
	child_a, child_b: i32,
) -> bool
{
	_, _, _ = a, b, child_a;
	state := (^Filter_Callback_State)(user_context);
	if state == nil
	{
		return false;
	}
	state.child_calls += 1;
	return child_b != 7;
}

@(test)
layer_masks_and_matrix_are_symmetric_and_fail_closed :: proc(t: ^testing.T)
{
	layer_a, status_a := entasis.collision_layer(2);
	layer_b, status_b := entasis.collision_layer(9);
	testing.expect_value(t, status_a, entasis.Status.Ok);
	testing.expect_value(t, status_b, entasis.Status.Ok);
	_, invalid_status := entasis.collision_layer(64);
	testing.expect_value(t, invalid_status, entasis.Status.Invalid_Argument);

	mask := entasis.layer_mask_none();
	mask = entasis.layer_mask_add(mask, layer_b);
	testing.expect(t, entasis.layer_mask_contains(mask, layer_b));
	testing.expect(t, !entasis.layer_mask_contains(mask, layer_a));
	mask = entasis.layer_mask_remove(mask, layer_b);
	testing.expect(t, !entasis.layer_mask_contains(mask, layer_b));

	layer_rules := entasis.layer_matrix_none();
	testing.expect(t, !entasis.layer_matrix_allows(&layer_rules, layer_a, layer_b));
	testing.expect_value(
		t, entasis.layer_matrix_allow(&layer_rules, layer_a, layer_b), entasis.Status.Ok,
	);
	testing.expect(t, entasis.layer_matrix_allows(&layer_rules, layer_a, layer_b));
	testing.expect(t, entasis.layer_matrix_allows(&layer_rules, layer_b, layer_a));
	testing.expect_value(
		t, entasis.layer_matrix_deny(&layer_rules, layer_b, layer_a), entasis.Status.Ok,
	);
	testing.expect(t, !entasis.layer_matrix_allows(&layer_rules, layer_a, layer_b));

	filter_a := entasis.collision_filter(layer_a, entasis.layer_mask(layer_b));
	filter_b := entasis.collision_filter(layer_b, entasis.layer_mask(layer_a));
	all_rules := entasis.layer_matrix_all();
	testing.expect(t, entasis.collision_filter_allows(filter_a, filter_b, &all_rules));
	filter_b.mask = entasis.layer_mask_none();
	testing.expect(t, !entasis.collision_filter_allows(filter_a, filter_b, &all_rules));
}

@(test)
default_material_combination_is_commutative_and_table_backed :: proc(t: ^testing.T)
{
	soft := entasis.material(0.5, 2, entasis.spring_settings(10, 0.5));
	stiff := entasis.material(0.25, 4, entasis.spring_settings(30, 1));
	combined_ab := entasis.material_combine_default(soft, stiff);
	combined_ba := entasis.material_combine_default(stiff, soft);
	testing.expect_value(t, combined_ab, combined_ba);
	testing.expect_value(t, combined_ab.friction, f32(0.125));
	testing.expect_value(t, combined_ab.maximum_recovery_velocity, f32(4));
	testing.expect_value(t, combined_ab.spring, stiff.spring);
	testing.expect_value(t, entasis.material_validate(combined_ab), entasis.Status.Ok);

	values := [2]entasis.Material{soft, stiff};
	table := entasis.material_table(values[:]);
	first_id, first_status := entasis.material_id(0);
	second_id, second_status := entasis.material_id(1);
	testing.expect_value(t, first_status, entasis.Status.Ok);
	testing.expect_value(t, second_status, entasis.Status.Ok);
	first, first_get := entasis.material_table_get(table, first_id);
	second, second_get := entasis.material_table_get(table, second_id);
	if testing.expect_value(t, first_get, entasis.Status.Ok)
	{
		testing.expect_value(t, first^, soft);
	}
	if testing.expect_value(t, second_get, entasis.Status.Ok)
	{
		testing.expect_value(t, second^, stiff);
	}
	_, missing := entasis.material_table_get(table, entasis.material_id_invalid());
	testing.expect_value(t, missing, entasis.Status.Not_Found);
}

@(test)
layer_material_callbacks_use_one_dense_property_record_per_collidable :: proc(t: ^testing.T)
{
	properties: entasis.Collision_Property_Table;
	if !testing.expect_value(t, entasis.collision_property_init(&properties, 2, 0), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.collision_property_destroy(&properties);

	layer_a, _ := entasis.collision_layer(1);
	layer_b, _ := entasis.collision_layer(2);
	material_a, _ := entasis.material_id(0);
	material_b, _ := entasis.material_id(1);
	materials := [2]entasis.Material{
		entasis.material(0.5, 2, entasis.spring_settings(10, 0.5)),
		entasis.material(0.25, 4, entasis.spring_settings(30, 1)),
	};
	body_a := entasis.Body_Handle{value=0};
	body_b := entasis.Body_Handle{value=1};
	testing.expect_value(
		t,
		entasis.collision_property_set_body(
		&properties, body_a,
		entasis.collision_properties(
		entasis.collision_filter(layer_a, entasis.layer_mask(layer_b)), material_a,
	),
	),
		entasis.Status.Ok,
	);
	testing.expect_value(
		t,
		entasis.collision_property_set_body(
		&properties, body_b,
		entasis.collision_properties(
		entasis.collision_filter(layer_b, entasis.layer_mask(layer_a)), material_b,
	),
	),
		entasis.Status.Ok,
	);
	reference_a, _ := physics.collidable_reference_body(.Dynamic, body_a);
	reference_b, _ := physics.collidable_reference_body(.Dynamic, body_b);

	layer_rules := entasis.layer_matrix_all();
	callback_state: Filter_Callback_State;
	policy := entasis.layer_material_policy(
		&properties, &layer_rules, entasis.material_table(materials[:]),
	);
	policy.pair_filter = filter_test_pair;
	policy.child_filter = filter_test_child;
	policy.user_context = &callback_state;
	callbacks := entasis.narrow_policy_layers_materials(&policy);

	speculative_margin: f32;
	allow_result := callbacks.allow(
		callbacks.user_context, 0, reference_a, reference_b, &speculative_margin,
	);
	testing.expect_value(t, allow_result, entasis.Collision_Testing_State.Allow);
	testing.expect_value(t, callback_state.pair_calls, i32(1));

	child_allow := callbacks.allow_child(
		callbacks.user_context, 0, reference_a, reference_b, 3, 4,
	);
	child_reject := callbacks.allow_child(
		callbacks.user_context, 4, reference_a, reference_b, 3, 7,
	);
	testing.expect_value(t, child_allow, entasis.Collision_Testing_State.Allow);
	testing.expect_value(t, child_reject, entasis.Collision_Testing_State.Reject);
	testing.expect_value(t, callback_state.child_calls, i32(2));

	manifold: entasis.Manifold_Result;
	first_contact, second_contact: entasis.Contact_Material;
	first_result := callbacks.configure(
		callbacks.user_context, 0, reference_a, reference_b, &manifold, &first_contact,
	);
	second_result := callbacks.configure(
		callbacks.user_context, 4, reference_a, reference_b, &manifold, &second_contact,
	);
	testing.expect_value(t, first_result, entasis.Collision_Testing_State.Allow);
	testing.expect_value(t, second_result, entasis.Collision_Testing_State.Allow);
	testing.expect_value(t, first_contact, second_contact);
	testing.expect_value(
		t, first_contact,
		entasis.material_to_contact(entasis.material_combine_default(materials[0], materials[1])),
	);
}

material_filter_body :: proc(
	world: ^entasis.World,
	shape: entasis.Shape_Handle,
	inertia: entasis.Body_Inertia,
	position: entasis.Vector3,
) -> (entasis.Body_Handle, entasis.Status)
{
	return entasis.body_add(
		world,
		entasis.body_dynamic(
		shape, inertia, entasis.pose(position), {}, entasis.body_activity(-1, 255),
	),
	);
}

@(test)
layer_policy_filters_real_contacts_and_accepts_after_idle_policy_change :: proc(t: ^testing.T)
{
	properties: entasis.Collision_Property_Table;
	if !testing.expect_value(t, entasis.collision_property_init(&properties, 4, 0), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.collision_property_destroy(&properties);

	layer_a, _ := entasis.collision_layer(3);
	layer_b, _ := entasis.collision_layer(4);
	layer_rules := entasis.layer_matrix_all();
	if !testing.expect_value(
		t, entasis.layer_matrix_deny(&layer_rules, layer_a, layer_b), entasis.Status.Ok,
	)
	{
		return;
	}
	materials := [1]entasis.Material{entasis.material_default()};
	policy := entasis.layer_material_policy(
		&properties, &layer_rules, entasis.material_table(materials[:]),
	);
	pose_policy := entasis.uniform_gravity_policy({}, 0, 0);
	description := small_world_description();
	description.gravity = {};
	if !testing.expect_value(
		t,
		entasis.world_description_set_callbacks(
		&description,
		entasis.narrow_policy_layers_materials(&policy),
		entasis.pose_policy_uniform(&pose_policy),
	),
		entasis.Status.Ok,
	)
	{
		return;
	}

	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape_value := entasis.sphere(1);
	shape, shape_status := entasis.shape_add(&world, shape_value);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	inertia, inertia_status := entasis.shape_inertia(shape_value, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}
	body_a, body_a_status := material_filter_body(&world, shape, inertia, {0, 0, 0});
	body_b, body_b_status := material_filter_body(&world, shape, inertia, {1.5, 0, 0});
	if !testing.expect_value(t, body_a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, body_b_status, entasis.Status.Ok)
	{
		return;
	}
	material_zero, _ := entasis.material_id(0);
	testing.expect_value(
		t,
		entasis.collision_property_set_body(
		&properties, body_a,
		entasis.collision_properties(
		entasis.collision_filter(layer_a, entasis.layer_mask(layer_b)), material_zero,
	),
	),
		entasis.Status.Ok,
	);
	testing.expect_value(
		t,
		entasis.collision_property_set_body(
		&properties, body_b,
		entasis.collision_properties(
		entasis.collision_filter(layer_b, entasis.layer_mask(layer_a)), material_zero,
	),
	),
		entasis.Status.Ok,
	);

	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	count, count_status := entasis.constraint_count(&world);
	testing.expect_value(t, count_status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);

	// policies may be changed while the world is idle. the next step accepts the
	// same broad-phase candidate and creates the contact constraint
	testing.expect_value(
		t, entasis.layer_matrix_allow(&layer_rules, layer_a, layer_b), entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	count, count_status = entasis.constraint_count(&world);
	testing.expect_value(t, count_status, entasis.Status.Ok);
	testing.expect(t, count > 0);
}

@(test)
invalid_material_table_is_rejected_during_world_initialization :: proc(t: ^testing.T)
{
	invalid_materials := [1]entasis.Material{
		entasis.material(-1, 2, entasis.spring_settings(30, 1)),
	};
	policy := entasis.layer_material_policy(
		nil, nil, entasis.material_table(invalid_materials[:]),
	);
	pose_policy := entasis.uniform_gravity_policy();
	description := small_world_description();
	if !testing.expect_value(
		t,
		entasis.world_description_set_callbacks(
		&description,
		entasis.narrow_policy_layers_materials(&policy),
		entasis.pose_policy_uniform(&pose_policy),
	),
		entasis.Status.Ok,
	)
	{
		return;
	}
	world: entasis.World;
	testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Invalid_Description);
}

@(test)
collision_property_specialization_preserves_dense_table_contract :: proc(t: ^testing.T)
{
	table: entasis.Collision_Property_Table;
	if !testing.expect_value(t, entasis.collision_property_init(&table, 1, 1), entasis.Status.Ok)
	{
		return;
	}
	destroyed := false;
	defer if !destroyed
	{
		entasis.collision_property_destroy(&table);
	}
	testing.expect_value(
		t, entasis.collision_property_ensure_capacity(&table, 8, 8), entasis.Status.Ok,
	);

	layer, _ := entasis.collision_layer(5);
	value := entasis.collision_properties(
		entasis.collision_filter(layer, entasis.layer_mask_all()),
		entasis.material_id_invalid(),
	);
	body := entasis.Body_Handle{value=3};
	static := entasis.Static_Handle{value=3};
	body_reference, _ := physics.collidable_reference_body(.Dynamic, body);
	static_reference, _ := physics.collidable_reference_static(static);

	testing.expect_value(
		t, entasis.collision_property_set_body(&table, body, value), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.collision_property_set_static(&table, static, value), entasis.Status.Ok,
	);
	body_value, body_status := entasis.collision_property_get_body(&table, body);
	static_value, static_status := entasis.collision_property_get_static(&table, static);
	if testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		testing.expect_value(t, body_value^, value);
	}
	if testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		testing.expect_value(t, static_value^, value);
	}

	updated := value;
	updated.mask = entasis.layer_mask(layer);
	testing.expect_value(
		t, entasis.collision_property_set(&table, body_reference, updated), entasis.Status.Ok,
	);
	generic_value, generic_status := entasis.collision_property_get(&table, body_reference);
	if testing.expect_value(t, generic_status, entasis.Status.Ok)
	{
		testing.expect_value(t, generic_value^, updated);
	}

	collidables := [2]entasis.Collidable_Reference{body_reference, static_reference};
	values := [2]entasis.Collision_Properties{value, updated};
	written, set_status := entasis.collision_property_set_batch(
		&table, collidables[:], values[:],
	);
	testing.expect_value(t, written, 2);
	testing.expect_value(t, set_status, entasis.Status.Ok);
	removed, remove_status := entasis.collision_property_remove_batch(&table, collidables[:]);
	testing.expect_value(t, removed, 2);
	testing.expect_value(t, remove_status, entasis.Status.Ok);

	testing.expect_value(
		t, entasis.collision_property_set_body(&table, body, value), entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.collision_property_remove_body(&table, body), entasis.Status.Ok);
	testing.expect_value(
		t, entasis.collision_property_set_static(&table, static, value), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.collision_property_remove_static(&table, static), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.collision_property_set(&table, body_reference, value), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.collision_property_remove(&table, body_reference), entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.collision_property_clear(&table), entasis.Status.Ok);

	invalid := value;
	invalid.layer = entasis.Collision_Layer(64);
	testing.expect_value(
		t, entasis.collision_properties_validate(invalid), entasis.Status.Invalid_Description,
	);
	testing.expect_value(
		t, entasis.collision_property_set_body(&table, body, invalid),
		entasis.Status.Invalid_Description,
	);

	testing.expect_value(t, entasis.collision_property_destroy(&table), entasis.Status.Ok);
	destroyed = true;
}
