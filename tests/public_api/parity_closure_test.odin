package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

Parity_Stage_Counts :: struct
{
	counts: [4]int,
}

parity_stage_completed :: proc "contextless" (
	user_context: rawptr,
	stage: entasis.Timestep_Completion_Stage,
	dt: f32,
	dispatcher: ^entasis.Dispatcher,
) -> entasis.Status
{
	_, _ = dt, dispatcher;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	counts := (^Parity_Stage_Counts)(user_context);
	counts.counts[int(stage)] += 1;
	return .Ok;
}

@(test)
parity_world_interop_resize_and_stage_callbacks :: proc(t: ^testing.T)
{
	world: entasis.World;
	counts: Parity_Stage_Counts;
	description := small_world_description();
	description.timestep_callbacks = entasis.timestep_callbacks(
		parity_stage_completed, &counts,
	);
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	simulation, simulation_status := entasis.world_borrow_simulation(&world);
	pool, pool_status := entasis.world_borrow_pool(&world);
	_, dispatcher_status := entasis.world_borrow_dispatcher(&world);
	testing.expect_value(t, simulation_status, entasis.Status.Ok);
	testing.expect(t, simulation != nil);
	testing.expect_value(t, pool_status, entasis.Status.Ok);
	testing.expect(t, pool != nil);
	testing.expect_value(t, dispatcher_status, entasis.Status.Ok);

	hints := description.capacity;
	hints.bodies = 32;
	hints.statics = 16;
	testing.expect_value(t, entasis.world_resize(&world, hints), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	for count in counts.counts
	{
		testing.expect_value(t, count, 1);
	}
}

@(test)
parity_body_impulses_bounds_and_direct_bound_update :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
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
	body, body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 255)),
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}

	testing.expect_value(
		t, entasis.body_apply_linear_impulse(&world, body, {2, 0, 0}), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.body_apply_angular_impulse(&world, body, {0, 1, 0}), entasis.Status.Ok,
	);
	testing.expect_value(
		t, entasis.body_apply_impulse(&world, body, {1, 0, 0}, {0, 1, 0}), entasis.Status.Ok,
	);
	state, state_status := entasis.body_get(&world, body);
	if !testing.expect_value(t, state_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, state.velocity.linear.x, f32(3));
	testing.expect(t, state.velocity.angular.z != 0);
	point_velocity, point_status := entasis.body_velocity_at_offset(&world, body, {1, 0, 0});
	testing.expect_value(t, point_status, entasis.Status.Ok);
	testing.expect(t, point_velocity.x >= 3);

	bounds, bounds_status := entasis.body_bounds(&world, body);
	if !testing.expect_value(t, bounds_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, bounds.min.x < -0.9 && bounds.max.x > 0.9);

	view, view_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, view_status, entasis.Status.Ok)
	{
		return;
	}
	row, row_valid := entasis.active_body_row(view, 0);
	if !testing.expect(t, row_valid)
	{
		return;
	}
	row.dynamics.motion.pose.position.x = 10;
	testing.expect_value(t, entasis.body_update_bounds(&world, body), entasis.Status.Ok);
	moved, moved_status := entasis.body_bounds(&world, body);
	testing.expect_value(t, moved_status, entasis.Status.Ok);
	testing.expect(t, moved.min.x > 8.9 && moved.max.x > 10.9);
}

@(test)
parity_connectivity_and_accumulated_impulses :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, constraint_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	a, a_status := entasis.body_add(&world, constraint_body(0));
	b, b_status := entasis.body_add(&world, constraint_body(1));
	if !testing.expect_value(t, a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, b_status, entasis.Status.Ok)
	{
		return;
	}
	constraint, constraint_status := entasis.constraint_add_2(
		&world, a, b,
		entasis.Center_Distance_Constraint{
			target_distance=0,
			spring_settings=entasis.spring_settings(30, 1),
		},
	);
	if !testing.expect_value(t, constraint_status, entasis.Status.Ok)
	{
		return;
	}

	count, count_status := entasis.body_constraint_count(&world, a);
	testing.expect_value(t, count_status, entasis.Status.Ok);
	testing.expect_value(t, count, 1);
	constraints: [1]entasis.Constraint_Handle;
	written, required, list_status := entasis.body_constraints(&world, a, constraints[:]);
	testing.expect_value(t, list_status, entasis.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, constraints[0], constraint);
	connected: [1]entasis.Body_Handle;
	written, required, list_status = entasis.body_connected_bodies(&world, a, connected[:]);
	testing.expect_value(t, list_status, entasis.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, required, 1);
	testing.expect_value(t, connected[0], b);

	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	impulses: [8]f32;
	impulse_count, impulse_required, impulse_status := entasis.constraint_accumulated_impulses(
		&world, constraint, impulses[:],
	);
	testing.expect_value(t, impulse_status, entasis.Status.Ok);
	testing.expect(t, impulse_count > 0 && impulse_required == impulse_count);
	magnitude, magnitude_status := entasis.constraint_accumulated_impulse_magnitude(&world, constraint);
	testing.expect_value(t, magnitude_status, entasis.Status.Ok);
	testing.expect(t, magnitude > 0);
	testing.expect_value(t, entasis.world_scale_accumulated_impulses(&world, 0.5), entasis.Status.Ok);
	scaled, scaled_status := entasis.constraint_accumulated_impulse_magnitude(&world, constraint);
	testing.expect_value(t, scaled_status, entasis.Status.Ok);
	testing.expect(t, scaled > 0 && scaled < magnitude);
}

@(test)
parity_shape_inspection_compound_mass_and_recursive_removal :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	sphere_value := entasis.sphere(0.5);
	box_value := entasis.box(1, 1, 1);
	sphere_shape, sphere_status := entasis.shape_add(&world, sphere_value);
	box_shape, box_status := entasis.shape_add(&world, box_value);
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok) ||
		!testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}
	children := [3]entasis.Compound_Child{
		entasis.compound_child(sphere_shape, entasis.pose({-1, 0, 0})),
		entasis.compound_child(box_shape, entasis.pose()),
		entasis.compound_child(sphere_shape, entasis.pose({1, 0, 0})),
	};
	masses := [3]f32{1, 2, 1};
	result, build_status := entasis.compound_build_dynamic(
		&world, entasis.compound_builder(children[:], masses[:]), true,
	);
	if !testing.expect_value(t, build_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, entasis.shape_handle_is_valid(result.shape));
	testing.expect(t, result.inertia.inverse_mass > 0);
	info, info_status := entasis.shape_inspect(&world, result.shape);
	testing.expect_value(t, info_status, entasis.Status.Ok);
	testing.expect_value(t, info.type_id, entasis.SHAPE_TYPE_COMPOUND);
	borrowed, borrow_status := entasis.shape_borrow_typed(&world, result.shape, entasis.Compound);
	testing.expect_value(t, borrow_status, entasis.Status.Ok);
	testing.expect(t, borrowed != nil && borrowed.children.length == 3);
	_, bounds_status := entasis.shape_bounds(&world, result.shape);
	testing.expect_value(t, bounds_status, entasis.Status.Ok);

	scratch: [8]entasis.Shape_Handle;
	removed, required, remove_status := entasis.shape_remove_recursive(
		&world, result.shape, scratch[:],
	);
	testing.expect_value(t, remove_status, entasis.Status.Ok);
	testing.expect(t, required >= 4);
	testing.expect_value(t, removed, 3);
}

@(test)
parity_sweep_collectors_and_direct_collision_queries :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	sphere_shape, sphere_status := entasis.shape_add(&world, entasis.sphere(0.5));
	box_shape, box_status := entasis.shape_add(&world, entasis.box(4, 1, 4));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok) ||
		!testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}
	_, static_status := entasis.static_add(
		&world, entasis.static_body(box_shape, entasis.pose({0, 0, 0})), .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}

	found, any_status := entasis.sweep_any(
		&world, sphere_shape, entasis.pose({0, 3, 0}), entasis.velocity({0, -5, 0}), 1,
	);
	testing.expect_value(t, any_status, entasis.Status.Ok);
	testing.expect(t, found);
	hits: [8]entasis.Sweep_Hit;
	hit_count, all_status := entasis.sweep_all(
		&world, sphere_shape, entasis.pose({0, 3, 0}), entasis.velocity({0, -5, 0}), 1,
		hits[:],
	);
	testing.expect_value(t, all_status, entasis.Status.Ok);
	testing.expect(t, hit_count >= 1);

	other_sphere, other_status := entasis.shape_add(&world, entasis.sphere(0.75));
	if !testing.expect_value(t, other_status, entasis.Status.Ok)
	{
		return;
	}
	manifold, collision_status := entasis.collision_query(
		&world,
		sphere_shape, entasis.pose(),
		other_sphere, entasis.pose({0.9, 0, 0}),
	);
	testing.expect_value(t, collision_status, entasis.Status.Ok);
	_, contact_state := physics.manifold_deepest_contact(&manifold);
	testing.expect_value(t, contact_state, physics.Reference_State.Present);
}

@(test)
motor_settings_preserve_softness_semantics :: proc(t: ^testing.T)
{
	settings := entasis.motor_settings(5000, 1e-4);
	testing.expect(t, settings.damping > 9999 && settings.damping < 10001);
	raw := entasis.motor_settings_damping(5000, 1e-4);
	testing.expect_value(t, raw.damping, f32(1e-4));

	world: entasis.World;
	description := constraint_world_description();
	description.gravity = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	a, a_status := entasis.body_add(&world, constraint_body(0));
	b, b_status := entasis.body_add(&world, constraint_body(0));
	if !testing.expect_value(t, a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, b_status, entasis.Status.Ok)
	{
		return;
	}
	_, add_status := entasis.constraint_add_2(
		&world, a, b,
		entasis.Angular_Axis_Motor{
			local_axis_a={0, 0, 1},
			target_velocity=10,
			settings=entasis.motor_settings(1000, 1e-4),
		},
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	for _ in 0 ..< 10
	{
		if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
		{
			return;
		}
	}
	a_state, _ := entasis.body_get(&world, a);
	b_state, _ := entasis.body_get(&world, b);
	relative := a_state.velocity.angular.z - b_state.velocity.angular.z;
	testing.expect(t, relative > 5);
}
