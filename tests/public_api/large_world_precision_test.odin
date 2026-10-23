package public_api_tests

import util "entasis:entasis_utilities"

import "core:math"
import "core:testing"
import entasis "entasis:entasis"

PRECISION_POSITION_TOLERANCE :: 0.001;
PRECISION_LOCAL_GAP_TOLERANCE :: 0.000001;

precision_expect_close_f64 :: proc(t: ^testing.T, a, b, tolerance: f64)
{
	testing.expect(t, math.abs(a - b) <= tolerance);
}

precision_expect_global :: proc(
	t: ^testing.T,
	actual, expected: entasis.Global_Position,
	tolerance: f64,
)
{
	precision_expect_close_f64(t, actual.x, expected.x, tolerance);
	precision_expect_close_f64(t, actual.y, expected.y, tolerance);
	precision_expect_close_f64(t, actual.z, expected.z, tolerance);
}

@(test)
precision_pose_roundtrip_and_translation :: proc(t: ^testing.T)
{
	gap: f64 = 0.002;
	offsets := [4]f64{0, 10_000, 1_000_000, 1_000_000_000};
	for offset in offsets
	{
		origin := entasis.global_position(offset, -offset*0.25, offset*0.5);
		frame, frame_status := entasis.origin_frame_at(origin);
		if !testing.expect_value(t, frame_status, entasis.Status.Ok)
		{
			return;
		}
		first := entasis.global_position(origin.x+3.125, origin.y-7.5, origin.z+11.25);
		second := entasis.global_position(first.x+gap, first.y-gap, first.z+gap);
		local_first, first_status := entasis.origin_frame_to_local(frame, first);
		local_second, second_status := entasis.origin_frame_to_local(frame, second);
		if !testing.expect_value(t, first_status, entasis.Status.Ok) ||
			!testing.expect_value(t, second_status, entasis.Status.Ok)
		{
			return;
		}
		expected_delta := entasis.global_position(
			second.x-first.x,
			second.y-first.y,
			second.z-first.z,
		);
		precision_expect_close_f64(t, f64(local_second.x-local_first.x), expected_delta.x, PRECISION_LOCAL_GAP_TOLERANCE);
		precision_expect_close_f64(t, f64(local_second.y-local_first.y), expected_delta.y, PRECISION_LOCAL_GAP_TOLERANCE);
		precision_expect_close_f64(t, f64(local_second.z-local_first.z), expected_delta.z, PRECISION_LOCAL_GAP_TOLERANCE);

		reconstructed_first, reconstruct_first_status := entasis.origin_frame_to_global(frame, local_first);
		reconstructed_second, reconstruct_second_status := entasis.origin_frame_to_global(frame, local_second);
		testing.expect_value(t, reconstruct_first_status, entasis.Status.Ok);
		testing.expect_value(t, reconstruct_second_status, entasis.Status.Ok);
		precision_expect_global(t, reconstructed_first, first, PRECISION_POSITION_TOLERANCE);
		precision_expect_global(t, reconstructed_second, second, PRECISION_POSITION_TOLERANCE);
	}

	// the unchanged absolute-f32 baseline is expected to lose a 2 mm gap at
	// one billion metres, this is negative-control evidence rather than a candidate pass
	baseline_a, baseline_a_status := entasis.origin_frame_to_local({}, 
		entasis.global_position(1_000_000_000, 0, 0),
	);
	baseline_b, baseline_b_status := entasis.origin_frame_to_local({}, 
		entasis.global_position(1_000_000_000+gap, 0, 0),
	);
	testing.expect_value(t, baseline_a_status, entasis.Status.Ok);
	testing.expect_value(t, baseline_b_status, entasis.Status.Ok);
	testing.expect_value(t, baseline_a.x, baseline_b.x);

	// both power-of-two and non-binary caller-owned frame changes preserve the
	// global value when local coordinates are recomputed before simulation use
	frame, frame_status := entasis.origin_frame_at(entasis.global_position(1_000_000, -2_000_000, 3_000_000));
	if !testing.expect_value(t, frame_status, entasis.Status.Ok)
	{
		return;
	}
	global := entasis.global_position(1_000_123.458, -1_999_991.875, 2_999_744.002);
	for delta in ([2]entasis.Global_Position{
		entasis.global_position(256, -512, 1024),
		entasis.global_position(123.456, -78.125, 0.03125),
	})
	{
		if !testing.expect_value(t, entasis.origin_frame_translate(&frame, delta), entasis.Status.Ok)
		{
			return;
		}
		local, local_status := entasis.origin_frame_to_local(frame, global);
		reconstructed, global_status := entasis.origin_frame_to_global(frame, local);
		testing.expect_value(t, local_status, entasis.Status.Ok);
		testing.expect_value(t, global_status, entasis.Status.Ok);
		precision_expect_global(t, reconstructed, global, PRECISION_POSITION_TOLERANCE);
	}

	// exercise an actual entasis body path with the origin candidate, the world
	// receives only local f32 coordinates while application coordinates stay f64
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	shape_value := entasis.sphere(0.5);
	shape, shape_status := entasis.shape_add(&world, shape_value);
	inertia, inertia_status := entasis.shape_inertia(shape_value, 1);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}
	body_frame, body_frame_status := entasis.origin_frame_at(
		entasis.global_position(1_000_000_000, -1_000_000_000, 500_000_000),
	);
	body_global := entasis.global_position(1_000_000_003.125, -999_999_992.5, 500_000_011.25);
	body_local, body_local_status := entasis.origin_frame_to_local(body_frame, body_global);
	if !testing.expect_value(t, body_frame_status, entasis.Status.Ok) ||
		!testing.expect_value(t, body_local_status, entasis.Status.Ok)
	{
		return;
	}
	handle, body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			shape,
			inertia,
			entasis.pose(body_local),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return;
	}
	state, get_status := entasis.body_get(&world, handle);
	reconstructed, reconstruct_status := entasis.origin_frame_to_global(body_frame, state.pose.position);
	testing.expect_value(t, get_status, entasis.Status.Ok);
	testing.expect_value(t, reconstruct_status, entasis.Status.Ok);
	precision_expect_global(t, reconstructed, body_global, PRECISION_POSITION_TOLERANCE);
}

precision_expect_vector :: proc(
	t: ^testing.T,
	actual, expected: entasis.Vector3,
	tolerance: f32,
)
{
	testing.expect(t, math.abs(actual.x - expected.x) <= tolerance);
	testing.expect(t, math.abs(actual.y - expected.y) <= tolerance);
	testing.expect(t, math.abs(actual.z - expected.z) <= tolerance);
}

precision_subtract_vector :: #force_inline proc "contextless" (
	value, delta: entasis.Vector3,
) -> entasis.Vector3
{
	return {value.x - delta.x, value.y - delta.y, value.z - delta.z};
}

@(test)
precision_queries_preserve_hits_and_local_geometry :: proc(t: ^testing.T)
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
	sphere_shape, sphere_status := entasis.shape_add(&world, sphere_value);
	sphere_inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	box_shape, box_status := entasis.shape_add(&world, entasis.box(2, 2, 2));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok) ||
		!testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}

	frame_origin := entasis.global_position(1_000_000_000, -750_000_000, 250_000_000);
	frame, frame_status := entasis.origin_frame_at(frame_origin);
	if !testing.expect_value(t, frame_status, entasis.Status.Ok)
	{
		return;
	}
	body_global := entasis.global_position(
		frame_origin.x + 3.25,
		frame_origin.y + 0.5,
		frame_origin.z - 1.0,
	);
	static_global := entasis.global_position(
		frame_origin.x + 12.0,
		frame_origin.y + 0.5,
		frame_origin.z - 1.0,
	);
	body_local, body_local_status := entasis.origin_frame_to_local(frame, body_global);
	static_local, static_local_status := entasis.origin_frame_to_local(frame, static_global);
	if !testing.expect_value(t, body_local_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_local_status, entasis.Status.Ok)
	{
		return;
	}

	body, body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			sphere_shape,
			sphere_inertia,
			entasis.pose(body_local),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	static_handle, static_status := entasis.static_add(
		&world,
		entasis.static_body(box_shape, entasis.pose(static_local)),
		.None,
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}

	body_ray_origin := entasis.Vector3{body_local.x - 5, body_local.y, body_local.z};
	body_hit_before, body_hit_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(body_ray_origin, {1, 0, 0}, 10),
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	static_ray_origin := entasis.Vector3{static_local.x - 5, static_local.y, static_local.z};
	static_hit_before, static_hit_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(static_ray_origin, {1, 0, 0}, 10),
		{include=entasis.COLLIDABLE_STATIC},
	);
	if !testing.expect_value(t, body_hit_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_hit_status, entasis.Status.Ok)
	{
		return;
	}
	body_hit_handle, body_handle_status := entasis.collidable_body_handle(body_hit_before.collidable);
	static_hit_handle, static_handle_status := entasis.collidable_static_handle(static_hit_before.collidable);
	testing.expect_value(t, body_handle_status, entasis.Status.Ok);
	testing.expect_value(t, static_handle_status, entasis.Status.Ok);
	testing.expect_value(t, body_hit_handle, body);
	testing.expect_value(t, static_hit_handle, static_handle);

	shift := entasis.Vector3{1024.5, -512.25, 64.125};
	if !testing.expect_value(
		t,
		entasis.world_shift_origin(&world, &frame, shift),
		entasis.Status.Ok,
	)
	{
		return;
	}

	body_hit_after, body_after_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(precision_subtract_vector(body_ray_origin, shift), {1, 0, 0}, 10),
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	static_hit_after, static_after_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(precision_subtract_vector(static_ray_origin, shift), {1, 0, 0}, 10),
		{include=entasis.COLLIDABLE_STATIC},
	);
	if !testing.expect_value(t, body_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_after_status, entasis.Status.Ok)
	{
		return;
	}
	body_hit_handle, body_handle_status = entasis.collidable_body_handle(body_hit_after.collidable);
	static_hit_handle, static_handle_status = entasis.collidable_static_handle(static_hit_after.collidable);
	testing.expect_value(t, body_handle_status, entasis.Status.Ok);
	testing.expect_value(t, static_handle_status, entasis.Status.Ok);
	testing.expect_value(t, body_hit_handle, body);
	testing.expect_value(t, static_hit_handle, static_handle);
	testing.expect(t, math.abs(body_hit_after.t - body_hit_before.t) <= 0.00001);
	testing.expect(t, math.abs(static_hit_after.t - static_hit_before.t) <= 0.00001);
	precision_expect_vector(
		t,
		body_hit_after.location,
		precision_subtract_vector(body_hit_before.location, shift),
		0.0001,
	);
	precision_expect_vector(
		t,
		static_hit_after.location,
		precision_subtract_vector(static_hit_before.location, shift),
		0.0001,
	);
	precision_expect_vector(t, body_hit_after.normal, body_hit_before.normal, 0.00001);
	precision_expect_vector(t, static_hit_after.normal, static_hit_before.normal, 0.00001);

	body_state, body_get_status := entasis.body_get(&world, body);
	static_state, static_get_status := entasis.static_get(&world, static_handle);
	body_reconstructed, body_global_status := entasis.origin_frame_to_global(
		frame,
		body_state.pose.position,
	);
	static_reconstructed, static_global_status := entasis.origin_frame_to_global(
		frame,
		static_state.pose.position,
	);
	testing.expect_value(t, body_get_status, entasis.Status.Ok);
	testing.expect_value(t, static_get_status, entasis.Status.Ok);
	testing.expect_value(t, body_global_status, entasis.Status.Ok);
	testing.expect_value(t, static_global_status, entasis.Status.Ok);
	precision_expect_global(t, body_reconstructed, body_global, PRECISION_POSITION_TOLERANCE);
	precision_expect_global(t, static_reconstructed, static_global, PRECISION_POSITION_TOLERANCE);
}

@(test)
precision_sleep_and_static_bounds_survive_frame_changes :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t,
		entasis.world_init(&world, sleeping_world_description()),
		entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	sphere_value := entasis.sphere(0.5);
	sphere_shape, sphere_status := entasis.shape_add(&world, sphere_value);
	sphere_inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	box_shape, box_status := entasis.shape_add(&world, entasis.box(2, 2, 2));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok) ||
		!testing.expect_value(t, box_status, entasis.Status.Ok)
	{
		return;
	}

	origin := entasis.global_position(-1_000_000_000, 500_000_000, 750_000_000);
	frame, frame_status := entasis.origin_frame_at(origin);
	if !testing.expect_value(t, frame_status, entasis.Status.Ok)
	{
		return;
	}
	body_global := entasis.global_position(origin.x + 4, origin.y, origin.z);
	static_global := entasis.global_position(origin.x - 4, origin.y, origin.z);
	body_local, body_local_status := entasis.origin_frame_to_local(frame, body_global);
	static_local, static_local_status := entasis.origin_frame_to_local(frame, static_global);
	if !testing.expect_value(t, body_local_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_local_status, entasis.Status.Ok)
	{
		return;
	}

	body, body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			sphere_shape,
			sphere_inertia,
			entasis.pose(body_local),
			{},
			entasis.body_activity(100, 1),
		),
	);
	static_handle, static_status := entasis.static_add(
		&world,
		entasis.static_body(box_shape, entasis.pose(static_local)),
		.None,
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_status, entasis.Status.Ok) ||
		!sleep_body_automatically(t, &world, body)
	{
		return;
	}

	shift := entasis.Vector3{256.5, -32.25, 8};
	if !testing.expect_value(
		t,
		entasis.world_shift_origin(&world, &frame, shift),
		entasis.Status.Ok,
	)
	{
		return;
	}
	sleeping, sleeping_status := entasis.body_is_sleeping(&world, body);
	testing.expect_value(t, sleeping_status, entasis.Status.Ok);
	testing.expect(t, sleeping);

	body_state, body_get_status := entasis.body_get(&world, body);
	static_state, static_get_status := entasis.static_get(&world, static_handle);
	body_reconstructed, body_global_status := entasis.origin_frame_to_global(
		frame,
		body_state.pose.position,
	);
	static_reconstructed, static_global_status := entasis.origin_frame_to_global(
		frame,
		static_state.pose.position,
	);
	testing.expect_value(t, body_get_status, entasis.Status.Ok);
	testing.expect_value(t, static_get_status, entasis.Status.Ok);
	testing.expect_value(t, body_global_status, entasis.Status.Ok);
	testing.expect_value(t, static_global_status, entasis.Status.Ok);
	precision_expect_global(t, body_reconstructed, body_global, PRECISION_POSITION_TOLERANCE);
	precision_expect_global(t, static_reconstructed, static_global, PRECISION_POSITION_TOLERANCE);

	body_hit, body_hit_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(
			{body_state.pose.position.x - 5, body_state.pose.position.y, body_state.pose.position.z},
			{1, 0, 0},
			10,
		),
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	static_hit, static_hit_status := entasis.ray_cast_closest(
		&world,
		entasis.ray(
			{static_state.pose.position.x - 5, static_state.pose.position.y, static_state.pose.position.z},
			{1, 0, 0},
			10,
		),
		{include=entasis.COLLIDABLE_STATIC},
	);
	if !testing.expect_value(t, body_hit_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_hit_status, entasis.Status.Ok)
	{
		return;
	}
	body_hit_handle, body_handle_status := entasis.collidable_body_handle(body_hit.collidable);
	static_hit_handle, static_handle_status := entasis.collidable_static_handle(static_hit.collidable);
	testing.expect_value(t, body_handle_status, entasis.Status.Ok);
	testing.expect_value(t, static_handle_status, entasis.Status.Ok);
	testing.expect_value(t, body_hit_handle, body);
	testing.expect_value(t, static_hit_handle, static_handle);
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	sleeping, sleeping_status = entasis.body_is_sleeping(&world, body);
	testing.expect_value(t, sleeping_status, entasis.Status.Ok);
	testing.expect(t, sleeping);
}

@(test)
precision_targets_and_axis_references_follow_frame_changes :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	if !testing.expect_value(t, entasis.world_enable_body_control(&world), entasis.Status.Ok)
	{
		return;
	}

	shape, shape_status := entasis.shape_add(&world, entasis.sphere(0.5));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	target_body, target_body_status := entasis.body_add(
		&world,
		entasis.body_kinematic(
			shape,
			entasis.pose({-10, 1, 2}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	if !testing.expect_value(t, target_body_status, entasis.Status.Ok)
	{
		return;
	}

	constraint_body_description := entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose({10, 1, -2}),
		{},
		entasis.body_activity(-1, 255),
	);
	constraint_body, constraint_body_status := entasis.body_add(&world, constraint_body_description);
	if !testing.expect_value(t, constraint_body_status, entasis.Status.Ok)
	{
		return;
	}

	target := entasis.pose({-3, 2, 1});
	lock := entasis.body_axis_lock_default(entasis.pose({13, 1, -2}));
	lock.linear_axes = {.X};
	servo := entasis.One_Body_Linear_Servo{
		target={19, 3, -4},
		spring_settings=entasis.spring_settings(30, 1),
		servo_settings=entasis.servo_settings(20, 0, 100),
	};
	if !testing.expect_value(
		t,
		entasis.body_set_kinematic_target(&world, target_body, target),
		entasis.Status.Ok,
	) ||
		!testing.expect_value(
			t,
			entasis.body_set_axis_lock(&world, constraint_body, lock),
			entasis.Status.Ok,
		)
	{
		return;
	}
	servo_handle, servo_status := entasis.constraint_add_1(&world, constraint_body, servo);
	if !testing.expect_value(t, servo_status, entasis.Status.Ok)
	{
		return;
	}

	frame, frame_status := entasis.origin_frame_at(
		entasis.global_position(1_000_000_000, 1_000_000_000, -1_000_000_000),
	);
	shift := entasis.Vector3{4, -2, 1};
	if !testing.expect_value(t, frame_status, entasis.Status.Ok) ||
		!testing.expect_value(
			t,
			entasis.world_shift_origin(&world, &frame, shift),
			entasis.Status.Ok,
		)
	{
		return;
	}

	read_target, target_status := entasis.body_get_kinematic_target(&world, target_body);
	read_lock, lock_status := entasis.body_get_axis_lock(&world, constraint_body);
	read_servo: entasis.One_Body_Linear_Servo;
	read_servo_status := entasis.constraint_get(&world, servo_handle, &read_servo);
	testing.expect_value(t, target_status, entasis.Status.Ok);
	testing.expect_value(t, lock_status, entasis.Status.Ok);
	testing.expect_value(t, read_servo_status, entasis.Status.Ok);
	precision_expect_vector(t, read_target.position, precision_subtract_vector(target.position, shift), 0.00001);
	precision_expect_vector(t, read_lock.reference.position, precision_subtract_vector(lock.reference.position, shift), 0.00001);
	precision_expect_vector(t, read_servo.target, precision_subtract_vector(servo.target, shift), 0.00001);

	if !testing.expect_value(
		t,
		entasis.body_clear_kinematic_target(&world, target_body),
		entasis.Status.Ok,
	)
	{
		return;
	}
	constraint_body_state, body_get_status := entasis.body_get(&world, constraint_body);
	if !testing.expect_value(t, body_get_status, entasis.Status.Ok)
	{
		return;
	}
	lock = entasis.body_axis_lock_default(constraint_body_state.pose);
	lock.linear_axes = {.X, .Y, .Z};
	servo.target = constraint_body_state.pose.position;
	if !testing.expect_value(
		t,
		entasis.body_set_axis_lock(&world, constraint_body, lock),
		entasis.Status.Ok,
	) ||
		!testing.expect_value(
			t,
			entasis.constraint_apply(&world, servo_handle, servo),
			entasis.Status.Ok,
		) ||
		!testing.expect_value(
			t,
			entasis.body_set_activity(&world, constraint_body, entasis.body_activity(0.1, 1)),
			entasis.Status.Ok,
		)
	{
		return;
	}
	for _ in 0 ..< 512
	{
		if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 64.0), entasis.Status.Ok)
		{
			return;
		}
		sleeping, sleeping_status := entasis.body_is_sleeping(&world, constraint_body);
		if sleeping_status == .Ok && sleeping
		{
			break;
		}
	}
	sleeping, sleeping_status := entasis.body_is_sleeping(&world, constraint_body);
	if !testing.expect_value(t, sleeping_status, entasis.Status.Ok) || !testing.expect(t, sleeping)
	{
		return;
	}

	second_shift := entasis.Vector3{-8, 16, -0.5};
	if !testing.expect_value(
		t,
		entasis.world_shift_origin(&world, &frame, second_shift),
		entasis.Status.Ok,
	)
	{
		return;
	}
	read_lock, lock_status = entasis.body_get_axis_lock(&world, constraint_body);
	read_servo_status = entasis.constraint_get(&world, servo_handle, &read_servo);
	testing.expect_value(t, lock_status, entasis.Status.Ok);
	testing.expect_value(t, read_servo_status, entasis.Status.Ok);
	precision_expect_vector(
		t,
		read_lock.reference.position,
		precision_subtract_vector(lock.reference.position, second_shift),
		0.00001,
	);
	precision_expect_vector(
		t,
		read_servo.target,
		precision_subtract_vector(servo.target, second_shift),
		0.00001,
	);
	sleeping, sleeping_status = entasis.body_is_sleeping(&world, constraint_body);
	testing.expect_value(t, sleeping_status, entasis.Status.Ok);
	testing.expect(t, sleeping);
}

precision_expect_shifted_body_state :: proc(
	t: ^testing.T,
	world: ^entasis.World,
	handle: entasis.Body_Handle,
	high_before: entasis.Global_Position,
	before, after: entasis.Body_Description,
	shift: entasis.Vector3,
)
{
	expected := before;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		expected_high: entasis.Global_Position = {high_before.x - f64(shift.x),
			high_before.y - f64(shift.y), high_before.z - f64(shift.z)};
		actual_high: entasis.Global_Pose;
		status: entasis.Status;
		actual_high, status = entasis.body_get_global_pose(world, handle);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, actual_high.position, expected_high);
		testing.expect_value(t, before.pose.position, entasis.Vector3{f32(high_before.x), f32(high_before.y), f32(high_before.z)});
		expected.pose.position = {f32(expected_high.x), f32(expected_high.y), f32(expected_high.z)};
	}
	else
	{
		expected.pose.position = precision_subtract_vector(before.pose.position, shift);
	}
	testing.expect_value(t, after, expected);
}

precision_expect_shifted_static_state :: proc(
	t: ^testing.T,
	world: ^entasis.World,
	handle: entasis.Static_Handle,
	high_before: entasis.Global_Position,
	before, after: entasis.Static_Description,
	shift: entasis.Vector3,
)
{
	expected := before;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		expected_high: entasis.Global_Position = {high_before.x - f64(shift.x),
			high_before.y - f64(shift.y), high_before.z - f64(shift.z)};
		actual_high: entasis.Global_Pose;
		status: entasis.Status;
		actual_high, status = entasis.static_get_global_pose(world, handle);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, actual_high.position, expected_high);
		testing.expect_value(t, before.pose.position, entasis.Vector3{f32(high_before.x), f32(high_before.y), f32(high_before.z)});
		expected.pose.position = {f32(expected_high.x), f32(expected_high.y), f32(expected_high.z)};
	}
	else
	{
		expected.pose.position = precision_subtract_vector(before.pose.position, shift);
	}
	testing.expect_value(t, after, expected);
}

precision_expect_impulses_equal :: proc(
	t: ^testing.T,
	before, after: []f32,
)
{
	testing.expect_value(t, len(after), len(before));
	for index in 0 ..< min(len(before), len(after))
	{
		testing.expect_value(t, after[index], before[index]);
	}
}

@(test)
precision_contacts_and_joints_preserve_relative_motion :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	tracker: entasis.Contact_Tracker;
	if !testing.expect_value(t, entasis.contact_tracker_init(&tracker, 4), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.contact_tracker_destroy(&tracker);
	if !testing.expect_value(t, entasis.contact_tracker_bind(&tracker, &world), entasis.Status.Ok)
	{
		return;
	}

	sphere_value := entasis.sphere(1);
	sphere_shape, shape_status := entasis.shape_add(&world, sphere_value);
	sphere_inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	contact_body, contact_body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			sphere_shape,
			sphere_inertia,
			entasis.pose({}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	contact_static, contact_static_status := entasis.static_add(
		&world,
		entasis.static_body(sphere_shape, entasis.pose({1.5, 0, 0})),
		.None,
	);
	joint_inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
	joint_a, joint_a_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
			joint_inertia,
			entasis.pose({-10, 0, 0}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	joint_b, joint_b_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
			joint_inertia,
			entasis.pose({-7, 0, 0}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	if !testing.expect_value(t, contact_body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_status, entasis.Status.Ok)
	{
		return;
	}

	joint_description := entasis.Center_Distance_Constraint{
		target_distance=2,
		spring_settings=entasis.spring_settings(30, 1),
	};
	joint, joint_status := entasis.constraint_add_2(
		&world,
		joint_a,
		joint_b,
		joint_description,
	);
	if !testing.expect_value(t, joint_status, entasis.Status.Ok) ||
		!testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}

	contact_events: [2]entasis.Contact_Event;
	contact_written, contact_required, contact_status := entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1)
	{
		return;
	}
	begin := contact_events[0];
	testing.expect_value(t, begin.kind, entasis.Contact_Event_Kind.Begin);
	testing.expect(t, .Solver_Data_Available in begin.flags);

	contact_body_reference, contact_body_reference_status := entasis.body_collidable_reference(
		&world,
		contact_body,
	);
	contact_static_reference, contact_static_reference_status := entasis.static_collidable_reference(
		&world,
		contact_static,
	);
	if !testing.expect_value(t, contact_body_reference_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_reference_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(
		t,
		(begin.a == contact_body_reference && begin.b == contact_static_reference) ||
			(begin.a == contact_static_reference && begin.b == contact_body_reference),
	);

	contact_data_before, contact_data_status := entasis.solver_contact_data(
		&world,
		begin.constraint,
	);
	contact_body_before, contact_body_get_status := entasis.body_get(&world, contact_body);
	contact_body_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		contact_body_high_pose: entasis.Global_Pose;
		contact_body_high_status: entasis.Status;
		contact_body_high_pose, contact_body_high_status = entasis.body_get_global_pose(&world, contact_body);
		testing.expect_value(t, contact_body_high_status, entasis.Status.Ok);
		contact_body_high_before = contact_body_high_pose.position;
	}
	contact_static_before, contact_static_get_status := entasis.static_get(&world, contact_static);
	contact_static_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		contact_static_high_pose: entasis.Global_Pose;
		contact_static_high_status: entasis.Status;
		contact_static_high_pose, contact_static_high_status = entasis.static_get_global_pose(&world, contact_static);
		testing.expect_value(t, contact_static_high_status, entasis.Status.Ok);
		contact_static_high_before = contact_static_high_pose.position;
	}
	joint_a_before, joint_a_get_status := entasis.body_get(&world, joint_a);
	joint_a_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		joint_a_high_pose: entasis.Global_Pose;
		joint_a_high_status: entasis.Status;
		joint_a_high_pose, joint_a_high_status = entasis.body_get_global_pose(&world, joint_a);
		testing.expect_value(t, joint_a_high_status, entasis.Status.Ok);
		joint_a_high_before = joint_a_high_pose.position;
	}
	joint_b_before, joint_b_get_status := entasis.body_get(&world, joint_b);
	joint_b_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		joint_b_high_pose: entasis.Global_Pose;
		joint_b_high_status: entasis.Status;
		joint_b_high_pose, joint_b_high_status = entasis.body_get_global_pose(&world, joint_b);
		testing.expect_value(t, joint_b_high_status, entasis.Status.Ok);
		joint_b_high_before = joint_b_high_pose.position;
	}
	if !testing.expect_value(t, contact_data_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_body_get_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_get_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_get_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_get_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, contact_data_before, begin.contact_data);

	contact_impulses_before: [entasis.MAXIMUM_SOLVER_IMPULSE_COUNT]f32;
	joint_impulses_before: [8]f32;
	contact_impulse_written, contact_impulse_required, contact_impulse_status :=
		entasis.constraint_accumulated_impulses(
			&world,
			begin.constraint,
			contact_impulses_before[:],
		);
	joint_impulse_written, joint_impulse_required, joint_impulse_status :=
		entasis.constraint_accumulated_impulses(
			&world,
			joint,
			joint_impulses_before[:],
		);
	contact_magnitude_before, contact_magnitude_status :=
		entasis.constraint_accumulated_impulse_magnitude(&world, begin.constraint);
	joint_magnitude_before, joint_magnitude_status :=
		entasis.constraint_accumulated_impulse_magnitude(&world, joint);
	if !testing.expect_value(t, contact_impulse_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_impulse_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_magnitude_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_magnitude_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, contact_impulse_written, contact_impulse_required);
	testing.expect_value(t, joint_impulse_written, joint_impulse_required);
	testing.expect(t, contact_magnitude_before > 0);
	testing.expect(t, joint_magnitude_before > 0);

	frame, frame_status := entasis.origin_frame_at(
		entasis.global_position(1_000_000_000, -1_000_000_000, 500_000_000),
	);
	shift := entasis.Vector3{256.5, -64.25, 32.125};
	if !testing.expect_value(t, frame_status, entasis.Status.Ok) ||
		!testing.expect_value(
			t,
			entasis.world_shift_origin(&world, &frame, shift),
			entasis.Status.Ok,
		)
	{
		return;
	}

	contact_body_after, contact_body_after_status := entasis.body_get(&world, contact_body);
	contact_static_after, contact_static_after_status := entasis.static_get(&world, contact_static);
	joint_a_after, joint_a_after_status := entasis.body_get(&world, joint_a);
	joint_b_after, joint_b_after_status := entasis.body_get(&world, joint_b);
	if !testing.expect_value(t, contact_body_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_after_status, entasis.Status.Ok)
	{
		return;
	}
	precision_expect_shifted_body_state(t, &world, contact_body, contact_body_high_before, contact_body_before, contact_body_after, shift);
	precision_expect_shifted_static_state(t, &world, contact_static, contact_static_high_before, contact_static_before, contact_static_after, shift);
	precision_expect_shifted_body_state(t, &world, joint_a, joint_a_high_before, joint_a_before, joint_a_after, shift);
	precision_expect_shifted_body_state(t, &world, joint_b, joint_b_high_before, joint_b_before, joint_b_after, shift);
	precision_expect_vector(
		t,
		precision_subtract_vector(contact_body_after.pose.position, contact_static_after.pose.position),
		precision_subtract_vector(contact_body_before.pose.position, contact_static_before.pose.position),
		0.001,
	);
	precision_expect_vector(
		t,
		precision_subtract_vector(joint_b_after.pose.position, joint_a_after.pose.position),
		precision_subtract_vector(joint_b_before.pose.position, joint_a_before.pose.position),
		0.001,
	);

	contact_data_after, contact_data_after_status := entasis.solver_contact_data(
		&world,
		begin.constraint,
	);
	contact_impulses_after: [entasis.MAXIMUM_SOLVER_IMPULSE_COUNT]f32;
	joint_impulses_after: [8]f32;
	contact_after_written, contact_after_required, contact_after_impulse_status :=
		entasis.constraint_accumulated_impulses(
			&world,
			begin.constraint,
			contact_impulses_after[:],
		);
	joint_after_written, joint_after_required, joint_after_impulse_status :=
		entasis.constraint_accumulated_impulses(
			&world,
			joint,
			joint_impulses_after[:],
		);
	contact_magnitude_after, contact_after_magnitude_status :=
		entasis.constraint_accumulated_impulse_magnitude(&world, begin.constraint);
	joint_magnitude_after, joint_after_magnitude_status :=
		entasis.constraint_accumulated_impulse_magnitude(&world, joint);
	if !testing.expect_value(t, contact_data_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_after_impulse_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_after_impulse_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_after_magnitude_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_after_magnitude_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, contact_data_after, contact_data_before);
	testing.expect_value(t, contact_after_written, contact_impulse_written);
	testing.expect_value(t, contact_after_required, contact_impulse_required);
	testing.expect_value(t, joint_after_written, joint_impulse_written);
	testing.expect_value(t, joint_after_required, joint_impulse_required);
	precision_expect_impulses_equal(
		t,
		contact_impulses_before[:contact_impulse_written],
		contact_impulses_after[:contact_after_written],
	);
	precision_expect_impulses_equal(
		t,
		joint_impulses_before[:joint_impulse_written],
		joint_impulses_after[:joint_after_written],
	);
	testing.expect_value(t, contact_magnitude_after, contact_magnitude_before);
	testing.expect_value(t, joint_magnitude_after, joint_magnitude_before);
	joint_readback: entasis.Center_Distance_Constraint;
	testing.expect_value(
		t,
		entasis.constraint_get(&world, joint, &joint_readback),
		entasis.Status.Ok,
	);
	testing.expect_value(t, joint_readback, joint_description);

	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	contact_written, contact_required, contact_status = entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1)
	{
		return;
	}
	persist := contact_events[0];
	testing.expect_value(t, persist.kind, entasis.Contact_Event_Kind.Persist);
	testing.expect_value(t, persist.a, begin.a);
	testing.expect_value(t, persist.b, begin.b);
	testing.expect_value(t, persist.constraint, begin.constraint);
}

@(test)
precision_event_and_query_context_lifetimes :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.damping = {};
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	tracker: entasis.Contact_Tracker;
	if !testing.expect_value(t, entasis.contact_tracker_init(&tracker, 4), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.contact_tracker_destroy(&tracker);
	if !testing.expect_value(t, entasis.contact_tracker_bind(&tracker, &world), entasis.Status.Ok)
	{
		return;
	}

	query_context: entasis.Query_Context;
	if !testing.expect_value(
		t,
		entasis.query_context_init(&query_context, &world),
		entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.query_context_destroy(&query_context);

	contact_sphere_value := entasis.sphere(0.5);
	contact_sphere, contact_shape_status := entasis.shape_add(&world, contact_sphere_value);
	contact_inertia, contact_inertia_status := entasis.shape_inertia(contact_sphere_value, 1);
	floor_shape, floor_shape_status := entasis.shape_add(&world, entasis.box(4, 1, 4));
	trigger_shape, trigger_shape_status := entasis.shape_add(&world, entasis.sphere(1));
	if !testing.expect_value(t, contact_shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_inertia_status, entasis.Status.Ok) ||
		!testing.expect_value(t, floor_shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_shape_status, entasis.Status.Ok)
	{
		return;
	}

	contact_body, contact_body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			contact_sphere,
			contact_inertia,
			entasis.pose({0, 0.45, 0}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	floor, floor_status := entasis.static_add(
		&world,
		entasis.static_body(floor_shape, entasis.pose({0, -0.5, 0})),
		.None,
	);
	trigger_body, trigger_body_status := entasis.body_add(
		&world,
		entasis.body_kinematic(
			trigger_shape,
			entasis.pose({11.5, 4, 0}),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	trigger_static, trigger_static_status := entasis.static_add(
		&world,
		entasis.static_body(trigger_shape, entasis.pose({10, 4, 0})),
		.None,
	);
	if !testing.expect_value(t, contact_body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, floor_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_static_status, entasis.Status.Ok)
	{
		return;
	}

	contact_body_reference, contact_body_reference_status := entasis.body_collidable_reference(
		&world,
		contact_body,
	);
	floor_reference, floor_reference_status := entasis.static_collidable_reference(
		&world,
		floor,
	);
	trigger_body_reference, trigger_body_reference_status := entasis.body_collidable_reference(
		&world,
		trigger_body,
	);
	trigger_static_reference, trigger_static_reference_status := entasis.static_collidable_reference(
		&world,
		trigger_static,
	);
	if !testing.expect_value(t, contact_body_reference_status, entasis.Status.Ok) ||
		!testing.expect_value(t, floor_reference_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_body_reference_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_static_reference_status, entasis.Status.Ok)
	{
		return;
	}

	if !testing.expect_value(
		t,
		entasis.world_enable_triggers(
			&world,
			{pair_capacity=4, candidates_per_worker=16, child_capacity=32},
		),
		entasis.Status.Ok,
	) ||
		!testing.expect_value(
			t,
			entasis.trigger_set(
				&world,
				trigger_static_reference,
				{user_id=77, stay=.Enabled},
			),
			entasis.Status.Ok,
		) ||
		!testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}

	contact_events: [2]entasis.Contact_Event;
	contact_written, contact_required, contact_status := entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	trigger_events: [2]entasis.Trigger_Event;
	trigger_written, trigger_required, trigger_status := entasis.trigger_events_drain(
		&world,
		trigger_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1) ||
		!testing.expect_value(t, trigger_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_written, 1) ||
		!testing.expect_value(t, trigger_required, 1)
	{
		return;
	}

	contact_begin := contact_events[0];
	trigger_enter := trigger_events[0];
	testing.expect_value(t, contact_begin.kind, entasis.Contact_Event_Kind.Begin);
	testing.expect_value(t, trigger_enter.kind, entasis.Trigger_Event_Kind.Enter);
	testing.expect(
		t,
		(contact_begin.a == contact_body_reference && contact_begin.b == floor_reference) ||
			(contact_begin.a == floor_reference && contact_begin.b == contact_body_reference),
	);
	testing.expect(
		t,
		(trigger_enter.pair.a == trigger_body_reference &&
			trigger_enter.pair.b == trigger_static_reference) ||
			(trigger_enter.pair.a == trigger_static_reference &&
				trigger_enter.pair.b == trigger_body_reference),
	);
	testing.expect(t, .Solver_Data_Available in contact_begin.flags);

	contact_data_before, contact_data_status := entasis.solver_contact_data(
		&world,
		contact_begin.constraint,
	);
	contact_body_before, contact_body_get_status := entasis.body_get(&world, contact_body);
	contact_body_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		contact_body_high_pose: entasis.Global_Pose;
		contact_body_high_status: entasis.Status;
		contact_body_high_pose, contact_body_high_status = entasis.body_get_global_pose(&world, contact_body);
		testing.expect_value(t, contact_body_high_status, entasis.Status.Ok);
		contact_body_high_before = contact_body_high_pose.position;
	}
	if !testing.expect_value(t, contact_data_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_body_get_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, contact_data_before, contact_begin.contact_data);

	ray_before := entasis.ray(
		{
			contact_body_before.pose.position.x,
			contact_body_before.pose.position.y + 3,
			contact_body_before.pose.position.z,
		},
		{0, -1, 0},
		6,
	);
	hit_before, hit_before_status := entasis.ray_cast_closest_with_context(
		&query_context,
		ray_before,
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	if !testing.expect_value(t, hit_before_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, hit_before.collidable, contact_body_reference);

	view_before, view_before_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, view_before_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, entasis.body_view_valid(&world, view_before));

	frame, frame_status := entasis.origin_frame_at(
		entasis.global_position(-1_000_000_000, 750_000_000, 250_000_000),
	);
	shift := entasis.Vector3{-128.25, 64.5, 16.125};
	if !testing.expect_value(t, frame_status, entasis.Status.Ok) ||
		!testing.expect_value(
			t,
			entasis.world_shift_origin(&world, &frame, shift),
			entasis.Status.Ok,
		)
	{
		return;
	}
	testing.expect(t, !entasis.body_view_valid(&world, view_before));
	view_after, view_after_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, view_after_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, entasis.body_view_valid(&world, view_after));
	testing.expect_value(t, view_after.count, view_before.count);

	contact_body_after, contact_body_after_status := entasis.body_get(&world, contact_body);
	contact_data_after, contact_data_after_status := entasis.solver_contact_data(
		&world,
		contact_begin.constraint,
	);
	if !testing.expect_value(t, contact_body_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_data_after_status, entasis.Status.Ok)
	{
		return;
	}
	precision_expect_shifted_body_state(t, &world, contact_body, contact_body_high_before, contact_body_before, contact_body_after, shift);
	testing.expect_value(t, contact_data_after, contact_data_before);

	ray_after := ray_before;
	ray_after.origin = precision_subtract_vector(ray_before.origin, shift);
	hit_after, hit_after_status := entasis.ray_cast_closest_with_context(
		&query_context,
		ray_after,
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	if !testing.expect_value(t, hit_after_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, hit_after.collidable, hit_before.collidable);
	testing.expect_value(t, hit_after.child_index, hit_before.child_index);
	testing.expect_value(t, hit_after.ray_id, hit_before.ray_id);
	testing.expect(t, math.abs(hit_after.t - hit_before.t) <= 0.001);
	precision_expect_vector(t, hit_after.normal, hit_before.normal, 0.00001);
	precision_expect_vector(
		t,
		hit_after.location,
		precision_subtract_vector(hit_before.location, shift),
		0.001,
	);

	trigger_written, trigger_required, trigger_status = entasis.trigger_events_drain(
		&world,
		trigger_events[:],
	);
	testing.expect_value(t, trigger_status, entasis.Status.Ok);
	testing.expect_value(t, trigger_written, 0);
	testing.expect_value(t, trigger_required, 0);

	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	contact_written, contact_required, contact_status = entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	trigger_written, trigger_required, trigger_status = entasis.trigger_events_drain(
		&world,
		trigger_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1) ||
		!testing.expect_value(t, trigger_status, entasis.Status.Ok) ||
		!testing.expect_value(t, trigger_written, 1) ||
		!testing.expect_value(t, trigger_required, 1)
	{
		return;
	}
	contact_persist := contact_events[0];
	trigger_stay := trigger_events[0];
	testing.expect_value(t, contact_persist.kind, entasis.Contact_Event_Kind.Persist);
	testing.expect_value(t, contact_persist.a, contact_begin.a);
	testing.expect_value(t, contact_persist.b, contact_begin.b);
	testing.expect_value(t, contact_persist.constraint, contact_begin.constraint);
	testing.expect_value(t, trigger_stay.kind, entasis.Trigger_Event_Kind.Stay);
	testing.expect_value(t, trigger_stay.pair, trigger_enter.pair);
	testing.expect_value(t, trigger_stay.epoch, trigger_enter.epoch);

	contact_body_after_step, contact_body_after_step_status := entasis.body_get(
		&world,
		contact_body,
	);
	if !testing.expect_value(t, contact_body_after_step_status, entasis.Status.Ok)
	{
		return;
	}
	reuse_ray := entasis.ray(
		{
			contact_body_after_step.pose.position.x,
			contact_body_after_step.pose.position.y + 3,
			contact_body_after_step.pose.position.z,
		},
		{0, -1, 0},
		6,
	);
	reuse_hit, reuse_status := entasis.ray_cast_closest_with_context(
		&query_context,
		reuse_ray,
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	if !testing.expect_value(t, reuse_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, reuse_hit.collidable, contact_body_reference);
}

@(test)
precision_region_crossing_preserves_interactions :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	description.gravity = {};
	description.damping = {};
	description.capacity.bodies = 8;
	description.capacity.statics = 2;
	description.capacity.constraints = 32;
	description.capacity.broad_phase_candidates = 32;
	description.capacity.pairs = 32;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	tracker: entasis.Contact_Tracker;
	if !testing.expect_value(t, entasis.contact_tracker_init(&tracker, 4), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.contact_tracker_destroy(&tracker);
	if !testing.expect_value(t, entasis.contact_tracker_bind(&tracker, &world), entasis.Status.Ok)
	{
		return;
	}

	sphere_value := entasis.sphere(1);
	sphere_shape, shape_status := entasis.shape_add(&world, sphere_value);
	sphere_inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	frame_origin := entasis.global_position(1_000_000_000, -500_000_000, 250_000_000);
	frame, frame_status := entasis.origin_frame_at(frame_origin);
	if !testing.expect_value(t, frame_status, entasis.Status.Ok)
	{
		return;
	}
	contact_body_global := entasis.global_position(
		frame_origin.x + 511.25,
		frame_origin.y,
		frame_origin.z,
	);
	contact_static_global := entasis.global_position(
		frame_origin.x + 512.75,
		frame_origin.y,
		frame_origin.z,
	);
	joint_a_global := entasis.global_position(
		frame_origin.x + 511,
		frame_origin.y + 10,
		frame_origin.z,
	);
	joint_b_global := entasis.global_position(
		frame_origin.x + 514,
		frame_origin.y + 10,
		frame_origin.z,
	);
	contact_body_local, contact_body_local_status := entasis.origin_frame_to_local(
		frame,
		contact_body_global,
	);
	contact_static_local, contact_static_local_status := entasis.origin_frame_to_local(
		frame,
		contact_static_global,
	);
	joint_a_local, joint_a_local_status := entasis.origin_frame_to_local(frame, joint_a_global);
	joint_b_local, joint_b_local_status := entasis.origin_frame_to_local(frame, joint_b_global);
	if !testing.expect_value(t, contact_body_local_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_local_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_local_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_local_status, entasis.Status.Ok)
	{
		return;
	}

	contact_body, contact_body_status := entasis.body_add(
		&world,
		entasis.body_dynamic(
			sphere_shape,
			sphere_inertia,
			entasis.pose(contact_body_local),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	contact_static, contact_static_status := entasis.static_add(
		&world,
		entasis.static_body(sphere_shape, entasis.pose(contact_static_local)),
		.None,
	);
	joint_a, joint_a_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
			sphere_inertia,
			entasis.pose(joint_a_local),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	joint_b, joint_b_status := entasis.body_add(
		&world,
		entasis.body_shapeless(
			sphere_inertia,
			entasis.pose(joint_b_local),
			{},
			entasis.body_activity(-1, 255),
		),
	);
	if !testing.expect_value(t, contact_body_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_status, entasis.Status.Ok)
	{
		return;
	}

	joint_description := entasis.Center_Distance_Constraint{
		target_distance=2,
		spring_settings=entasis.spring_settings(30, 1),
	};
	joint, joint_status := entasis.constraint_add_2(
		&world,
		joint_a,
		joint_b,
		joint_description,
	);
	if !testing.expect_value(t, joint_status, entasis.Status.Ok) ||
		!testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}

	contact_events: [2]entasis.Contact_Event;
	contact_written, contact_required, contact_status := entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1)
	{
		return;
	}
	contact_begin := contact_events[0];
	testing.expect_value(t, contact_begin.kind, entasis.Contact_Event_Kind.Begin);

	contact_body_reference, body_reference_status := entasis.body_collidable_reference(
		&world,
		contact_body,
	);
	contact_static_reference, static_reference_status := entasis.static_collidable_reference(
		&world,
		contact_static,
	);
	if !testing.expect_value(t, body_reference_status, entasis.Status.Ok) ||
		!testing.expect_value(t, static_reference_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(
		t,
		(contact_begin.a == contact_body_reference && contact_begin.b == contact_static_reference) ||
			(contact_begin.a == contact_static_reference && contact_begin.b == contact_body_reference),
	);

	contact_body_before, contact_body_before_status := entasis.body_get(&world, contact_body);
	contact_body_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		contact_body_high_pose: entasis.Global_Pose;
		contact_body_high_status: entasis.Status;
		contact_body_high_pose, contact_body_high_status = entasis.body_get_global_pose(&world, contact_body);
		testing.expect_value(t, contact_body_high_status, entasis.Status.Ok);
		contact_body_high_before = contact_body_high_pose.position;
	}
	contact_static_before, contact_static_before_status := entasis.static_get(&world, contact_static);
	contact_static_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		contact_static_high_pose: entasis.Global_Pose;
		contact_static_high_status: entasis.Status;
		contact_static_high_pose, contact_static_high_status = entasis.static_get_global_pose(&world, contact_static);
		testing.expect_value(t, contact_static_high_status, entasis.Status.Ok);
		contact_static_high_before = contact_static_high_pose.position;
	}
	joint_a_before, joint_a_before_status := entasis.body_get(&world, joint_a);
	joint_a_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		joint_a_high_pose: entasis.Global_Pose;
		joint_a_high_status: entasis.Status;
		joint_a_high_pose, joint_a_high_status = entasis.body_get_global_pose(&world, joint_a);
		testing.expect_value(t, joint_a_high_status, entasis.Status.Ok);
		joint_a_high_before = joint_a_high_pose.position;
	}
	joint_b_before, joint_b_before_status := entasis.body_get(&world, joint_b);
	joint_b_high_before: entasis.Global_Position;
	when util.PRECISION_MODE == .Double_Coordinates
	{
		joint_b_high_pose: entasis.Global_Pose;
		joint_b_high_status: entasis.Status;
		joint_b_high_pose, joint_b_high_status = entasis.body_get_global_pose(&world, joint_b);
		testing.expect_value(t, joint_b_high_status, entasis.Status.Ok);
		joint_b_high_before = joint_b_high_pose.position;
	}
	if !testing.expect_value(t, contact_body_before_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_before_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_before_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_before_status, entasis.Status.Ok)
	{
		return;
	}
	contact_global_before, contact_global_before_status := entasis.origin_frame_to_global(
		frame,
		contact_body_before.pose.position,
	);
	joint_a_global_before, joint_a_global_before_status := entasis.origin_frame_to_global(
		frame,
		joint_a_before.pose.position,
	);
	joint_b_global_before, joint_b_global_before_status := entasis.origin_frame_to_global(
		frame,
		joint_b_before.pose.position,
	);
	if !testing.expect_value(t, contact_global_before_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_global_before_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_global_before_status, entasis.Status.Ok)
	{
		return;
	}

	ray_before := entasis.ray(
		{contact_body_before.pose.position.x - 4, contact_body_before.pose.position.y, contact_body_before.pose.position.z},
		{1, 0, 0},
		8,
	);
	hit_before, hit_before_status := entasis.ray_cast_closest(
		&world,
		ray_before,
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	if !testing.expect_value(t, hit_before_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, hit_before.collidable, contact_body_reference);

	shift := entasis.Vector3{512, 0, 0};
	if !testing.expect_value(
		t,
		entasis.world_shift_origin(&world, &frame, shift),
		entasis.Status.Ok,
	)
	{
		return;
	}

	contact_body_after, contact_body_after_status := entasis.body_get(&world, contact_body);
	contact_static_after, contact_static_after_status := entasis.static_get(&world, contact_static);
	joint_a_after, joint_a_after_status := entasis.body_get(&world, joint_a);
	joint_b_after, joint_b_after_status := entasis.body_get(&world, joint_b);
	if !testing.expect_value(t, contact_body_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_static_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_after_status, entasis.Status.Ok)
	{
		return;
	}
	precision_expect_shifted_body_state(t, &world, contact_body, contact_body_high_before, contact_body_before, contact_body_after, shift);
	precision_expect_shifted_static_state(t, &world, contact_static, contact_static_high_before, contact_static_before, contact_static_after, shift);
	precision_expect_shifted_body_state(t, &world, joint_a, joint_a_high_before, joint_a_before, joint_a_after, shift);
	precision_expect_shifted_body_state(t, &world, joint_b, joint_b_high_before, joint_b_before, joint_b_after, shift);

	contact_global_after, contact_global_after_status := entasis.origin_frame_to_global(
		frame,
		contact_body_after.pose.position,
	);
	joint_a_global_after, joint_a_global_after_status := entasis.origin_frame_to_global(
		frame,
		joint_a_after.pose.position,
	);
	joint_b_global_after, joint_b_global_after_status := entasis.origin_frame_to_global(
		frame,
		joint_b_after.pose.position,
	);
	if !testing.expect_value(t, contact_global_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_a_global_after_status, entasis.Status.Ok) ||
		!testing.expect_value(t, joint_b_global_after_status, entasis.Status.Ok)
	{
		return;
	}
	precision_expect_global(t, contact_global_after, contact_global_before, PRECISION_POSITION_TOLERANCE);
	precision_expect_global(t, joint_a_global_after, joint_a_global_before, PRECISION_POSITION_TOLERANCE);
	precision_expect_global(t, joint_b_global_after, joint_b_global_before, PRECISION_POSITION_TOLERANCE);
	precision_expect_vector(
		t,
		precision_subtract_vector(joint_b_after.pose.position, joint_a_after.pose.position),
		precision_subtract_vector(joint_b_before.pose.position, joint_a_before.pose.position),
		0.001,
	);

	joint_readback: entasis.Center_Distance_Constraint;
	testing.expect_value(
		t,
		entasis.constraint_get(&world, joint, &joint_readback),
		entasis.Status.Ok,
	);
	testing.expect_value(t, joint_readback, joint_description);

	ray_after := ray_before;
	ray_after.origin = precision_subtract_vector(ray_before.origin, shift);
	hit_after, hit_after_status := entasis.ray_cast_closest(
		&world,
		ray_after,
		{include=entasis.COLLIDABLE_DYNAMIC},
	);
	if !testing.expect_value(t, hit_after_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, hit_after.collidable, hit_before.collidable);
	testing.expect_value(t, hit_after.child_index, hit_before.child_index);
	testing.expect(t, math.abs(hit_after.t - hit_before.t) <= 0.001);
	precision_expect_vector(t, hit_after.normal, hit_before.normal, 0.0001);

	contact_written, contact_required, contact_status = entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	testing.expect_value(t, contact_status, entasis.Status.Invalid_Argument);
	testing.expect_value(t, contact_written, 0);
	testing.expect_value(t, contact_required, 0);

	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	contact_written, contact_required, contact_status = entasis.contact_events_drain(
		&tracker,
		contact_events[:],
	);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok) ||
		!testing.expect_value(t, contact_written, 1) ||
		!testing.expect_value(t, contact_required, 1)
	{
		return;
	}
	contact_persist := contact_events[0];
	testing.expect_value(t, contact_persist.kind, entasis.Contact_Event_Kind.Persist);
	testing.expect_value(t, contact_persist.a, contact_begin.a);
	testing.expect_value(t, contact_persist.b, contact_begin.b);
	testing.expect_value(t, contact_persist.constraint, contact_begin.constraint);
}

precision_long_run_world_description :: proc() -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity = {
		bodies=64,
		statics=1,
		inactive_body_sets=1,
		shapes_per_type=2,
		constraints=256,
		initial_constraints_per_type_batch=16,
		minimum_constraints_per_body=4,
		broad_phase_candidates=256,
		pairs=256,
		collision_child_pairs=0,
		inactive_pairs=1,
		pending_pairs_per_worker=64,
	};
	description.solve = {
		velocity_iterations=4,
		substeps=1,
		fallback_batch_threshold=64,
	};
	return description;
}

@(test)
precision_reserved_maintenance_and_long_run :: proc(t: ^testing.T)
{
	BODY_COUNT :: 64;
	STEP_COUNT :: 10_000;
	SHIFT_INTERVAL :: 10;
	EXPECTED_SHIFT_COUNT :: STEP_COUNT / SHIFT_INTERVAL;
	TIMESTEP :: f32(1.0 / 60.0);

	control_world: entasis.World;
	control_description := precision_long_run_world_description();
	if !testing.expect_value(
		t,
		entasis.world_init(&control_world, control_description),
		entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&control_world);

	allocation_tracker: allocation_test_tracker;
	shifted_world: entasis.World;
	shifted_description := precision_long_run_world_description();
	shifted_description.allocator = allocation_test_allocator(&allocation_tracker);
	if !testing.expect_value(
		t,
		entasis.world_init_with_allocation_scope(
			&shifted_world,
			shifted_description,
			.All_Owned,
		),
		entasis.Status.Ok,
	)
	{
		return;
	}
	defer
	{
		testing.expect_value(t, entasis.world_destroy(&shifted_world), entasis.Status.Ok);
		allocation_test_empty(t, &allocation_tracker);
	}

	sphere_value := entasis.sphere(0.25);
	control_shape, control_shape_status := entasis.shape_add(&control_world, sphere_value);
	shifted_shape, shifted_shape_status := entasis.shape_add(&shifted_world, sphere_value);
	inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	if !testing.expect_value(t, control_shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, shifted_shape_status, entasis.Status.Ok) ||
		!testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return;
	}

	initial_origin := entasis.global_position(1_000_000_000, -750_000_000, 500_000_000);
	frame, frame_status := entasis.origin_frame_at(initial_origin);
	if !testing.expect_value(t, frame_status, entasis.Status.Ok)
	{
		return;
	}
	control_handles: [BODY_COUNT]entasis.Body_Handle;
	shifted_handles: [BODY_COUNT]entasis.Body_Handle;
	body_velocity := entasis.velocity({
		(f32(1.0) / 256.0) / TIMESTEP,
		0,
		-(f32(1.0) / 512.0) / TIMESTEP,
	});
	for index in 0 ..< BODY_COUNT
	{
		x := f32(index % 8) * 4 - 14;
		z := f32(index / 8) * 4 - 14;
		local := entasis.Vector3{x, f32(index % 3), z};
		global := entasis.global_position(
			initial_origin.x + f64(local.x),
			initial_origin.y + f64(local.y),
			initial_origin.z + f64(local.z),
		);
		shifted_local, local_status := entasis.origin_frame_to_local(frame, global);
		if !testing.expect_value(t, local_status, entasis.Status.Ok)
		{
			return;
		}
		control_handles[index], control_shape_status = entasis.body_add(
			&control_world,
			entasis.body_dynamic(
				control_shape,
				inertia,
				entasis.pose(local),
				body_velocity,
				entasis.body_activity(-1, 255),
			),
		);
		shifted_handles[index], shifted_shape_status = entasis.body_add(
			&shifted_world,
			entasis.body_dynamic(
				shifted_shape,
				inertia,
				entasis.pose(shifted_local),
				body_velocity,
				entasis.body_activity(-1, 255),
			),
		);
		if !testing.expect_value(t, control_shape_status, entasis.Status.Ok) ||
			!testing.expect_value(t, shifted_shape_status, entasis.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, shifted_handles[index], control_handles[index]);
	}

	failed_frame := frame;
	failed_body_before, failed_body_before_status := entasis.body_get(
		&shifted_world,
		shifted_handles[0],
	);
	if !testing.expect_value(t, failed_body_before_status, entasis.Status.Ok)
	{
		return;
	}
	failed_requests_before := allocation_tracker.requests;
	failed_live_before := allocation_tracker.live;
	failed_live_bytes_before := allocation_tracker.live_bytes;
	testing.expect_value(
		t,
		entasis.world_shift_origin(
			&shifted_world,
			&frame,
			{math.QNAN_F32, 0, 0},
		),
		entasis.Status.Invalid_Argument,
	);
	failed_body_after, failed_body_after_status := entasis.body_get(
		&shifted_world,
		shifted_handles[0],
	);
	testing.expect_value(t, failed_body_after_status, entasis.Status.Ok);
	testing.expect_value(t, frame, failed_frame);
	testing.expect_value(t, failed_body_after, failed_body_before);
	testing.expect_value(t, allocation_tracker.requests, failed_requests_before);
	testing.expect_value(t, allocation_tracker.live, failed_live_before);
	testing.expect_value(t, allocation_tracker.live_bytes, failed_live_bytes_before);

	shift_count := 0;
	shift_allocation_violation_count := 0;
	for step_index in 0 ..< STEP_COUNT
	{
		if !testing.expect_value(
			t,
			entasis.world_step(&control_world, TIMESTEP),
			entasis.Status.Ok,
		) ||
			!testing.expect_value(
				t,
				entasis.world_step(&shifted_world, TIMESTEP),
				entasis.Status.Ok,
			)
		{
			return;
		}
		if (step_index+1) % SHIFT_INTERVAL != 0
		{
			continue;
		}
		shift := entasis.Vector3{256, -128, 64};
		if shift_count % 2 != 0
		{
			shift = {-shift.x, -shift.y, -shift.z};
		}
		requests_before := allocation_tracker.requests;
		live_before := allocation_tracker.live;
		live_bytes_before := allocation_tracker.live_bytes;
		if !testing.expect_value(
			t,
			entasis.world_shift_origin(&shifted_world, &frame, shift),
			entasis.Status.Ok,
		)
		{
			return;
		}
		if allocation_tracker.requests != requests_before ||
			allocation_tracker.live != live_before ||
			allocation_tracker.live_bytes != live_bytes_before
		{
			shift_allocation_violation_count += 1;
		}
		shift_count += 1;
	}
	testing.expect_value(t, shift_count, EXPECTED_SHIFT_COUNT);
	testing.expect_value(t, shift_allocation_violation_count, 0);
	precision_expect_global(t, frame.origin, initial_origin, 0);

	control_stats, control_stats_status := entasis.world_stats(&control_world);
	shifted_stats, shifted_stats_status := entasis.world_stats(&shifted_world);
	if !testing.expect_value(t, control_stats_status, entasis.Status.Ok) ||
		!testing.expect_value(t, shifted_stats_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, control_stats.step_index, STEP_COUNT);
	testing.expect_value(t, shifted_stats.step_index, STEP_COUNT);
	testing.expect_value(t, control_stats.active_bodies, BODY_COUNT);
	testing.expect_value(t, shifted_stats.active_bodies, BODY_COUNT);

	maximum_drift: f64;
	for index in 0 ..< BODY_COUNT
	{
		control_body, control_body_status := entasis.body_get(&control_world, control_handles[index]);
		shifted_body, shifted_body_status := entasis.body_get(&shifted_world, shifted_handles[index]);
		if !testing.expect_value(t, control_body_status, entasis.Status.Ok) ||
			!testing.expect_value(t, shifted_body_status, entasis.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, shifted_body.velocity, control_body.velocity);
		testing.expect_value(t, shifted_body.pose.orientation, control_body.pose.orientation);
		shifted_global, global_status := entasis.origin_frame_to_global(
			frame,
			shifted_body.pose.position,
		);
		if !testing.expect_value(t, global_status, entasis.Status.Ok)
		{
			return;
		}
		expected_global := entasis.global_position(
			initial_origin.x + f64(control_body.pose.position.x),
			initial_origin.y + f64(control_body.pose.position.y),
			initial_origin.z + f64(control_body.pose.position.z),
		);
		drift := max(
			math.abs(shifted_global.x - expected_global.x),
			math.abs(shifted_global.y - expected_global.y),
			math.abs(shifted_global.z - expected_global.z),
		);
		maximum_drift = max(maximum_drift, drift);
		testing.expect(t, !math.is_nan(shifted_body.pose.position.x));
		testing.expect(t, !math.is_inf(shifted_body.pose.position.x, 0));
		testing.expect(t, !math.is_nan(shifted_body.pose.position.y));
		testing.expect(t, !math.is_inf(shifted_body.pose.position.y, 0));
		testing.expect(t, !math.is_nan(shifted_body.pose.position.z));
		testing.expect(t, !math.is_inf(shifted_body.pose.position.z, 0));

		body_reference, body_reference_status := entasis.body_collidable_reference(
			&shifted_world,
			shifted_handles[index],
		);
		if !testing.expect_value(t, body_reference_status, entasis.Status.Ok)
		{
			return;
		}
		overlap_hits: [4]entasis.Overlap_Hit;
		overlap_count, overlap_status := entasis.overlap_all(
			&shifted_world,
			shifted_shape,
			shifted_body.pose,
			overlap_hits[:],
			{include=entasis.COLLIDABLE_DYNAMIC},
		);
		if !testing.expect_value(t, overlap_status, entasis.Status.Ok)
		{
			return;
		}
		matching_hit_count := 0;
		for hit in overlap_hits[:overlap_count]
		{
			if hit.collidable == body_reference
			{
				matching_hit_count += 1;
			}
		}
		testing.expect_value(t, matching_hit_count, 1);
	}
	testing.expectf(
		t,
		maximum_drift <= PRECISION_POSITION_TOLERANCE,
		"maximum repeated-shift drift exceeded tolerance: %.9f",
		maximum_drift,
	);
}
