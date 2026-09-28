package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
shape_factories_use_full_dimensions_by_default :: proc(t: ^testing.T)
{
	testing.expect_value(t, entasis.sphere(2), physics.Sphere{radius=2});
	segmented_box := entasis.box(2, 4, 6);
	testing.expect_value(t, segmented_box, physics.Box{half_width=1, half_height=2, half_length=3});
	testing.expect_value(
		t, entasis.box_half_extents(1, 2, 3),
		physics.Box{half_width=1, half_height=2, half_length=3},
	);
	testing.expect_value(
		t, entasis.capsule(1, 4), physics.Capsule{radius=1, half_length=2},
	);
	testing.expect_value(
		t, entasis.capsule_half_length(1, 2),
		physics.Capsule{radius=1, half_length=2},
	);
	testing.expect_value(
		t, entasis.cylinder(1, 4), physics.Cylinder{radius=1, half_length=2},
	);
	testing.expect_value(
		t, entasis.cylinder_half_length(1, 2),
		physics.Cylinder{radius=1, half_length=2},
	);
	a, b, c := entasis.Vector3{0, 0, 0}, entasis.Vector3{1, 0, 0}, entasis.Vector3{0, 0, 1};
	testing.expect_value(t, entasis.triangle(a, b, c), physics.Triangle{a=a, b=b, c=c});
}

@(test)
shape_type_ids_are_compile_time_built_in_constants :: proc(t: ^testing.T)
{
	testing.expect_value(t, entasis.shape_type_id(entasis.Sphere), entasis.SHAPE_TYPE_SPHERE);
	testing.expect_value(t, entasis.shape_type_id(entasis.Capsule), entasis.SHAPE_TYPE_CAPSULE);
	testing.expect_value(t, entasis.shape_type_id(entasis.Box), entasis.SHAPE_TYPE_BOX);
	testing.expect_value(t, entasis.shape_type_id(entasis.Triangle), entasis.SHAPE_TYPE_TRIANGLE);
	testing.expect_value(t, entasis.shape_type_id(entasis.Cylinder), entasis.SHAPE_TYPE_CYLINDER);
	testing.expect_value(t, i32(entasis.SHAPE_TYPE_SPHERE), i32(physics.SPHERE_TYPE_ID));
	testing.expect_value(t, i32(entasis.SHAPE_TYPE_MESH), i32(physics.MESH_TYPE_ID));
}

@(test)
shape_validation_rejects_degenerate_public_values :: proc(t: ^testing.T)
{
	testing.expect_value(t, entasis.shape_validate(entasis.sphere(0)), entasis.Status.Invalid_Description);
	testing.expect_value(t, entasis.shape_validate(entasis.box(-1, 1, 1)), entasis.Status.Invalid_Description);
	testing.expect_value(t, entasis.shape_validate(entasis.capsule(1, 0)), entasis.Status.Invalid_Description);
	testing.expect_value(t, entasis.shape_validate(entasis.cylinder(0, 1)), entasis.Status.Invalid_Description);
	degenerate := entasis.triangle({0, 0, 0}, {1, 0, 0}, {2, 0, 0});
	testing.expect_value(t, entasis.shape_validate(degenerate), entasis.Status.Invalid_Description);
}

@(test)
shape_inertia_matches_low_level_built_in_kernels :: proc(t: ^testing.T)
{
	box_shape := entasis.box(2, 4, 6);
	facade_box, facade_box_status := entasis.shape_inertia(box_shape, 3);
	direct_box, direct_box_status := physics.box_inertia(box_shape, 3);
	testing.expect_value(t, facade_box_status, direct_box_status);
	testing.expect_value(t, facade_box, direct_box);

	capsule_shape := entasis.capsule(0.5, 2);
	facade_capsule, facade_capsule_status := entasis.shape_inertia(capsule_shape, 2);
	direct_capsule, direct_capsule_status := physics.capsule_inertia(capsule_shape, 2);
	testing.expect_value(t, facade_capsule_status, direct_capsule_status);
	testing.expect_value(t, facade_capsule, direct_capsule);

	_, invalid_mass_status := entasis.shape_inertia(box_shape, 0);
	testing.expect_value(t, invalid_mass_status, entasis.Status.Invalid_Argument);
	_, invalid_shape_status := entasis.shape_inertia(entasis.sphere(-1), 1);
	testing.expect_value(t, invalid_shape_status, entasis.Status.Invalid_Description);
}

@(test)
shape_registration_removal_and_registered_inertia_use_world_registry :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape_value := entasis.box(2, 4, 6);
	handle, add_status := entasis.shape_add(&world, shape_value);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, entasis.shape_handle_is_valid(handle));
	testing.expect_value(t, physics.typed_index_type(handle), i32(physics.BOX_TYPE_ID));

	direct_inertia, direct_status := entasis.shape_inertia(shape_value, 5);
	registered_inertia, registered_status := entasis.shape_registered_inertia(&world, handle, 5);
	testing.expect_value(t, registered_status, direct_status);
	testing.expect_value(t, registered_inertia, direct_inertia);

	testing.expect_value(t, entasis.shape_remove(&world, handle), entasis.Status.Ok);
	testing.expect_value(t, entasis.shape_remove(&world, handle), entasis.Status.Not_Found);
}

@(test)
shape_registration_validates_and_advanced_typed_add_is_explicit :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	invalid_handle, invalid_status := entasis.shape_add(&world, entasis.sphere(0));
	testing.expect_value(t, invalid_status, entasis.Status.Invalid_Description);
	testing.expect(t, !entasis.shape_handle_is_valid(invalid_handle));

	raw := physics.Sphere{radius=1};
	handle, status := entasis.shape_add_typed(&world, entasis.SHAPE_TYPE_SPHERE, &raw);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.shape_remove(&world, handle), entasis.Status.Ok);
}

@(test)
shared_shape_cannot_be_removed_while_bodies_reference_it :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape_value := entasis.box(1, 1, 1);
	shape_handle, shape_status := entasis.shape_add(&world, shape_value);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	inertia, inertia_status := entasis.shape_inertia(shape_value, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	description := entasis.Body_Description{
		pose={orientation=util.quaternion_identity(), position={0, 2, 0}},
		local_inertia=inertia,
		collidable={shape=shape_handle, maximum_speculative_margin=1},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
	for offset in 0 ..< 2
	{
		description.pose.position.x = f32(offset);
		_, body_status := entasis.body_add(&world, description);
		testing.expect_value(t, body_status, entasis.Status.Ok);
	}
	testing.expect_value(t, entasis.shape_remove(&world, shape_handle), entasis.Status.Shape_In_Use);
	testing.expect_value(t, entasis.world_clear(&world), entasis.Status.Ok);
}
