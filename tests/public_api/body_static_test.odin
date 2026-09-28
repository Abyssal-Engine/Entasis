package public_api_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

body_static_world_description :: proc() -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	return description;
}

@(test)
body_and_static_factories_match_low_level_descriptions :: proc(t: ^testing.T)
{
	shape, shape_status := physics.typed_index_create(physics.BOX_TYPE_ID, 7);
	if !testing.expect_value(t, shape_status, physics.Physics_Status.Ok)
	{
		return;
	}
	body_pose := entasis.pose({1, 2, 3});
	body_velocity := entasis.velocity({4, 5, 6}, {7, 8, 9});
	activity := entasis.body_activity(0.25, 12);
	inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=2, zz=3},
		inverse_mass=0.5,
	};
	body_collidable := entasis.collidable(shape);

	testing.expect_value(
		t, body_pose,
		physics.Rigid_Pose{orientation=util.quaternion_identity(), position={1, 2, 3}},
	);
	testing.expect_value(
		t, body_velocity,
		physics.Body_Velocity{linear={4, 5, 6}, angular={7, 8, 9}},
	);
	testing.expect_value(
		t, activity,
		physics.Body_Activity_Description{
			sleep_threshold=0.25,
			minimum_timestep_count_under_threshold=12,
		},
	);
	testing.expect_value(
		t, body_collidable,
		physics.Collidable_Description{
			shape=shape,
			continuity={mode=.Passive},
			minimum_speculative_margin=0,
			maximum_speculative_margin=f32(math.F32_MAX),
		},
	);

	dynamic_description := entasis.body_dynamic(shape, inertia, body_pose, body_velocity, activity);
	testing.expect_value(
		t, dynamic_description,
		physics.Body_Description{
			pose=body_pose,
			velocity=body_velocity,
			local_inertia=inertia,
			collidable=body_collidable,
			activity=activity,
		},
	);

	kinematic_description := entasis.body_kinematic(shape, body_pose, body_velocity, activity);
	testing.expect_value(
		t, kinematic_description,
		physics.Body_Description{
			pose=body_pose,
			velocity=body_velocity,
			local_inertia={},
			collidable=body_collidable,
			activity=activity,
		},
	);

	shapeless := entasis.body_shapeless(inertia, body_pose, body_velocity, activity);
	testing.expect_value(
		t, shapeless,
		physics.Body_Description{
			pose=body_pose,
			velocity=body_velocity,
			local_inertia=inertia,
			collidable={},
			activity=activity,
		},
	);

	static_description := entasis.static_body(shape, body_pose);
	testing.expect_value(
		t, static_description,
		physics.Static_Description{pose=body_pose, shape=shape, continuity={}},
	);
}

@(test)
body_creation_snapshots_mutation_and_removal_use_qualified_paths :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, body_static_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	box_a := entasis.box(1, 2, 3);
	shape_a, shape_a_status := entasis.shape_add(&world, box_a);
	if !testing.expect_value(t, shape_a_status, entasis.Status.Ok)
	{
		return;
	}
	box_b := entasis.box(2, 2, 2);
	shape_b, shape_b_status := entasis.shape_add(&world, box_b);
	if !testing.expect_value(t, shape_b_status, entasis.Status.Ok)
	{
		return;
	}
	inertia, inertia_status := entasis.shape_inertia(box_a, 2);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	description := entasis.body_dynamic(
		shape_a,
		inertia,
		entasis.pose({0, 3, 0}),
		entasis.velocity({1, 0, 0}),
		entasis.body_activity(-1, 255),
	);
	handle, add_status := entasis.body_add(&world, description);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}

	state, get_status := entasis.body_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state, description);

	updated_velocity := entasis.velocity({2, 3, 4}, {5, 6, 7});
	testing.expect_value(
		t, entasis.body_set_velocity(&world, handle, updated_velocity), entasis.Status.Ok,
	);
	state, get_status = entasis.body_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state.velocity, updated_velocity);

	updated_pose := entasis.pose({4, 5, 6}, {0, 0, 0, 1});
	testing.expect_value(t, entasis.body_set_pose(&world, handle, updated_pose), entasis.Status.Ok);
	updated_activity := entasis.body_activity(0.5, 7);
	testing.expect_value(
		t, entasis.body_set_activity(&world, handle, updated_activity), entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.body_set_shape(&world, handle, shape_b), entasis.Status.Ok);
	state, get_status = entasis.body_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state.pose, updated_pose);
	testing.expect_value(t, state.activity, updated_activity);
	testing.expect_value(t, state.collidable.shape, shape_b);

	// the old shape is released when the body changes shape. the new one remains
	// referenced until body removal
	testing.expect_value(t, entasis.shape_remove(&world, shape_a), entasis.Status.Ok);
	testing.expect_value(t, entasis.shape_remove(&world, shape_b), entasis.Status.Shape_In_Use);

	state.velocity = entasis.velocity({-1, -2, -3});
	testing.expect_value(t, entasis.body_apply(&world, handle, state), entasis.Status.Ok);
	state, get_status = entasis.body_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state.velocity.linear, entasis.Vector3{-1, -2, -3});

	testing.expect_value(t, entasis.body_remove(&world, handle), entasis.Status.Ok);
	_, removed_status := entasis.body_get(&world, handle);
	testing.expect_value(t, removed_status, entasis.Status.Not_Found);
	testing.expect_value(t, entasis.body_remove(&world, handle), entasis.Status.Not_Found);
	testing.expect_value(t, entasis.shape_remove(&world, shape_b), entasis.Status.Ok);
}

@(test)
numeric_body_handle_reuse_preserves_generation_contract :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, body_static_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
	first, first_status := entasis.body_add(
		&world,
		entasis.body_shapeless(inertia, entasis.pose({1, 0, 0}), {}, entasis.body_activity(-1, 255)),
	);
	if !testing.expect_value(t, first_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.body_remove(&world, first), entasis.Status.Ok);
	_, stale_status := entasis.body_get(&world, first);
	testing.expect_value(t, stale_status, entasis.Status.Not_Found);

	second, second_status := entasis.body_add(
		&world,
		entasis.body_shapeless(inertia, entasis.pose({2, 0, 0}), {}, entasis.body_activity(-1, 255)),
	);
	if !testing.expect_value(t, second_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, second, first);
	state, reused_status := entasis.body_get(&world, first);
	testing.expect_value(t, reused_status, entasis.Status.Ok);
	testing.expect_value(t, state.pose.position, entasis.Vector3{2, 0, 0});
}

@(test)
static_creation_mutation_and_removal_use_explicit_awakening_policy :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, body_static_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	shape_a, shape_a_status := entasis.shape_add(&world, entasis.box(4, 1, 4));
	if !testing.expect_value(t, shape_a_status, entasis.Status.Ok)
	{
		return;
	}
	shape_b, shape_b_status := entasis.shape_add(&world, entasis.sphere(1));
	if !testing.expect_value(t, shape_b_status, entasis.Status.Ok)
	{
		return;
	}

	description := entasis.static_body(shape_a, entasis.pose({0, -0.5, 0}));
	handle, add_status := entasis.static_add(&world, description, .None);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	state, get_status := entasis.static_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state, description);

	updated_pose := entasis.pose({1, 2, 3});
	testing.expect_value(
		t, entasis.static_set_pose(&world, handle, updated_pose, .None), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.static_set_shape(&world, handle, shape_b, .Overlaps), entasis.Status.Ok,
	);
	continuity := entasis.Continuous_Detection{mode=.Passive};
	testing.expect_value(
		t, entasis.static_set_continuity(&world, handle, continuity, .None), entasis.Status.Ok,
	);
	state, get_status = entasis.static_get(&world, handle);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, state.pose, updated_pose);
	testing.expect_value(t, state.shape, shape_b);
	testing.expect_value(t, state.continuity, continuity);

	testing.expect_value(t, entasis.shape_remove(&world, shape_a), entasis.Status.Ok);
	testing.expect_value(t, entasis.shape_remove(&world, shape_b), entasis.Status.Shape_In_Use);

	state.pose = entasis.pose({4, 5, 6});
	testing.expect_value(
		t, entasis.static_apply(&world, handle, state, .Overlaps), entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.static_remove(&world, handle, .None), entasis.Status.Ok);
	_, removed_status := entasis.static_get(&world, handle);
	testing.expect_value(t, removed_status, entasis.Status.Not_Found);
	testing.expect_value(t, entasis.shape_remove(&world, shape_b), entasis.Status.Ok);

	invalid_policy := entasis.Awakening_Policy(255);
	_, invalid_add := entasis.static_add(&world, description, invalid_policy);
	testing.expect_value(t, invalid_add, entasis.Status.Invalid_Argument);
}

@(test)
body_snapshot_and_mutation_remain_valid_after_sleep_transition :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, body_static_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
	description := entasis.body_shapeless(
		inertia,
		entasis.pose({0, 0, 0}),
		{},
		entasis.body_activity(100, 1),
	);
	handle, add_status := entasis.body_add(&world, description);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}

	// the timestep may move the body into an inactive set. handle-based snapshots
	// and complete body mutations must still resolve it
	for _ in 0 ..< 4
	{
		if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
		{
			return;
		}
	}
	state, get_status := entasis.body_get(&world, handle);
	if !testing.expect_value(t, get_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, state.pose.position, entasis.Vector3{});

	testing.expect_value(
		t,
		entasis.body_set_velocity(&world, handle, entasis.velocity({1, 0, 0})),
		entasis.Status.Ok,
	);
	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	state, get_status = entasis.body_get(&world, handle);
	if !testing.expect_value(t, get_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, state.pose.position.x > 0);
}

@(test)
disposed_world_rejects_body_and_static_lifecycle_calls :: proc(t: ^testing.T)
{
	world: entasis.World;
	body_description := entasis.body_shapeless(
		entasis.Body_Inertia{inverse_mass=1},
		entasis.pose(),
	);
	static_description := entasis.static_body(
		entasis.shape_handle_invalid(),
		entasis.pose(),
	);

	_, body_add_status := entasis.body_add(&world, body_description);
	testing.expect_value(t, body_add_status, entasis.Status.Disposed);
	_, body_get_status := entasis.body_get(&world, entasis.body_handle_invalid());
	testing.expect_value(t, body_get_status, entasis.Status.Disposed);
	testing.expect_value(
		t,
		entasis.body_apply(&world, entasis.body_handle_invalid(), body_description),
		entasis.Status.Disposed,
	);
	testing.expect_value(
		t,
		entasis.body_remove(&world, entasis.body_handle_invalid()),
		entasis.Status.Disposed,
	);

	_, static_add_status := entasis.static_add(&world, static_description, .None);
	testing.expect_value(t, static_add_status, entasis.Status.Disposed);
	_, static_get_status := entasis.static_get(&world, entasis.static_handle_invalid());
	testing.expect_value(t, static_get_status, entasis.Status.Disposed);
	testing.expect_value(
		t,
		entasis.static_apply(
		&world,
		entasis.static_handle_invalid(),
		static_description,
		.None,
	),
		entasis.Status.Disposed,
	);
	testing.expect_value(
		t,
		entasis.static_remove(&world, entasis.static_handle_invalid(), .None),
		entasis.Status.Disposed,
	);
}
