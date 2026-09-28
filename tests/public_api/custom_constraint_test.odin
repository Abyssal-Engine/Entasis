package public_api_tests

import "core:testing"
import "core:simd"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

Velocity_Target_Constraint :: struct
{
	target_speed: f32,
}

Velocity_Target_Prestep :: struct
{
	target_speed: entasis.F32x8,
}

Velocity_Target_Impulses :: struct
{
	accumulated: entasis.F32x8,
}

velocity_target_validate :: proc "contextless" (
	type_id: i32, description: rawptr,
) -> entasis.Status
{
	if type_id < i32(entasis.FIRST_CUSTOM_CONSTRAINT_TYPE_ID) || description == nil
	{
		return .Invalid_Argument;
	}
	value := (^Velocity_Target_Constraint)(description).target_speed;
	if value != value || value < -10000 || value > 10000
	{
		return .Invalid_Description;
	}
	return .Ok;
}

velocity_target_kernel :: proc "contextless" (
	prestep_raw: rawptr,
	bodies: ^[4]entasis.Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32,
	impulses_raw: rawptr,
	active_mask: entasis.I32x8,
	phase: entasis.Constraint_Kernel_Phase,
)
{
	_, _ = dt, inverse_dt;
	prestep := (^Velocity_Target_Prestep)(prestep_raw);
	impulses := (^Velocity_Target_Impulses)(impulses_raw);
	body := &bodies[0];
	switch phase
	{
		case .Prestep, .Incremental_Update:
			return;
		case .Warmstart:
			updated := simd.add(
				body.linear_velocity.x,
				simd.mul(impulses.accumulated, body.inverse_mass),
			);
			body.linear_velocity.x = util.wide_select_f32(
				active_mask, updated, body.linear_velocity.x,
			);
		case .Solve:
			delta_velocity := simd.sub(prestep.target_speed, body.linear_velocity.x);
			impulse_delta := simd.div(delta_velocity, body.inverse_mass);
			updated_velocity := simd.add(
				body.linear_velocity.x,
				simd.mul(impulse_delta, body.inverse_mass),
			);
			body.linear_velocity.x = util.wide_select_f32(
				active_mask, updated_velocity, body.linear_velocity.x,
			);
			updated_impulse := simd.add(impulses.accumulated, impulse_delta);
			impulses.accumulated = util.wide_select_f32(
				active_mask, updated_impulse, impulses.accumulated,
			);
	}
}

register_velocity_target_constraint :: proc(
	t: ^testing.T, world: ^entasis.World,
) -> (entasis.Constraint_Type_ID, bool)
{
	type_id, next_status := entasis.custom_constraint_next_type_id(world);
	if !testing.expect_value(t, next_status, entasis.Status.Ok)
	{
		return {}, false;
	}
	access := [4]entasis.Body_Access_Mask{
		entasis.BODY_ACCESS_NO_POSE, {}, {}, {},
	};
	registration := entasis.custom_constraint_registration(
		Velocity_Target_Constraint,
		Velocity_Target_Prestep,
		Velocity_Target_Impulses,
		type_id,
		1,
		access,
		access,
		velocity_target_validate,
		velocity_target_kernel,
	);
	if !testing.expect_value(
		t, entasis.custom_constraint_register(world, registration), entasis.Status.Ok,
	)
	{
		return {}, false;
	}
	return type_id, true;
}

custom_constraint_body_description :: proc "contextless" (
	position: entasis.Vector3,
	activity: entasis.Activity_Description,
) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose(position),
		entasis.velocity(),
		activity,
	);
}

@(test)
custom_constraint_runs_full_and_tail_bundles_and_supports_get_apply :: proc(t: ^testing.T)
{
	description := small_world_description();
	description.gravity = {};
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	type_id, ok := register_velocity_target_constraint(t, &world);
	if !ok
	{
		return;
	}

	bodies: [9]entasis.Body_Handle;
	constraints: [9]entasis.Constraint_Handle;
	for index in 0 ..< len(bodies)
	{
		body, body_status := entasis.body_add(
			&world, custom_constraint_body_description({f32(index) * 2, 0, 0}, entasis.body_activity_default()),
		);
		if !testing.expect_value(t, body_status, entasis.Status.Ok)
		{
			return;
		}
		bodies[index] = body;
		target := Velocity_Target_Constraint{target_speed=f32(index + 1)};
		constraint, constraint_status := entasis.custom_constraint_add_typed(
			&world, bodies[index:index + 1], type_id, &target,
		);
		if !testing.expect_value(t, constraint_status, entasis.Status.Ok)
		{
			return;
		}
		constraints[index] = constraint;
	}
	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	for index in 0 ..< len(bodies)
	{
		state, state_status := entasis.body_get(&world, bodies[index]);
		if !testing.expect_value(t, state_status, entasis.Status.Ok)
		{
			return;
		}
		testing.expect(t, state.velocity.linear.x > f32(index + 1) - 1e-4);
		testing.expect(t, state.velocity.linear.x < f32(index + 1) + 1e-4);
	}

	stored: Velocity_Target_Constraint;
	testing.expect_value(
		t, entasis.custom_constraint_get_typed(
		&world, constraints[8], type_id, &stored,
	), entasis.Status.Ok,
	);
	testing.expect_value(t, stored.target_speed, f32(9));
	stored.target_speed = -3;
	testing.expect_value(
		t, entasis.custom_constraint_apply_typed(
		&world, constraints[8], type_id, &stored,
	), entasis.Status.Ok,
	);
	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	state, state_status := entasis.body_get(&world, bodies[8]);
	if !testing.expect_value(t, state_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, state.velocity.linear.x > -3.0001 && state.velocity.linear.x < -2.9999);
}

@(test)
custom_constraint_uses_fallback_batch_and_survives_sleeping_description_storage :: proc(t: ^testing.T)
{
	description := small_world_description();
	description.gravity = {};
	description.solve.fallback_batch_threshold = 1;
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	type_id, ok := register_velocity_target_constraint(t, &world);
	if !ok
	{
		return;
	}

	activity := entasis.body_activity(10, 1);
	body, body_status := entasis.body_add(
		&world, custom_constraint_body_description({}, activity),
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}
	constraints: [3]entasis.Constraint_Handle;
	for index in 0 ..< len(constraints)
	{
		target := Velocity_Target_Constraint{target_speed=0};
		constraint, status := entasis.custom_constraint_add_typed(
			&world, []entasis.Body_Handle{body}, type_id, &target,
		);
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return;
		}
		constraints[index] = constraint;
	}
	for _ in 0 ..< 4
	{
		if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
		{
			return;
		}
	}
	sleeping, sleeping_status := entasis.body_is_sleeping(&world, body);
	testing.expect_value(t, sleeping_status, entasis.Status.Ok);
	testing.expect(t, sleeping);
	stored: Velocity_Target_Constraint;
	testing.expect_value(
		t, entasis.custom_constraint_get_typed(&world, constraints[2], type_id, &stored),
		entasis.Status.Ok,
	);
	testing.expect_value(t, stored.target_speed, f32(0));
	stored.target_speed = 2;
	testing.expect_value(
		t, entasis.custom_constraint_apply_typed(&world, constraints[2], type_id, &stored),
		entasis.Status.Ok,
	);
	testing.expect_value(t, entasis.body_awaken(&world, body), entasis.Status.Ok);
	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	state, state_status := entasis.body_get(&world, body);
	if !testing.expect_value(t, state_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, state.velocity.linear.x > 1.999 && state.velocity.linear.x < 2.001);
}

@(test)
custom_constraint_registration_enforces_public_storage_limits :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	type_id, status := entasis.custom_constraint_next_type_id(&world);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}
	access := [4]entasis.Body_Access_Mask{entasis.BODY_ACCESS_NO_POSE, {}, {}, {}};
	registration := entasis.Custom_Constraint_Registration{
		type_id=type_id,
		body_count=1,
		description_size=entasis.MAXIMUM_CUSTOM_DESCRIPTION_BYTES + 4,
		prestep_bundle_size=32,
		impulse_bundle_size=32,
		initial_access=access,
		solve_access=access,
		validate_description=velocity_target_validate,
		kernel=velocity_target_kernel,
		incremental_kernel=velocity_target_kernel,
	};
	testing.expect_value(
		t, entasis.custom_constraint_register(&world, registration),
		entasis.Status.Invalid_Description,
	);
}
