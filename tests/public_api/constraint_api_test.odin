package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

constraint_world_description :: proc() -> entasis.World_Description
{
	description := small_world_description();
	description.gravity = {};
	description.threading.worker_count = 1;
	return description;
}

constraint_body :: proc(index: int, minimum_quiet_steps: u8 = 255) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose({f32(index), 0, 0}),
		{},
		entasis.body_activity(0.01, minimum_quiet_steps),
	);
}

expect_constraint_mapping :: proc(
	t: ^testing.T,
	$T: typeid,
	expected_body_count: int,
)
{
	testing.expect(
		t, entasis.constraint_type_id(T) != entasis.CONSTRAINT_TYPE_INVALID,
	);
	testing.expect_value(
		t, entasis.constraint_body_count(T), expected_body_count,
	);
}

@(test)
all_builtin_constraint_descriptions_are_publicly_mapped :: proc(t: ^testing.T)
{
	expect_constraint_mapping(t, entasis.Angular_Axis_Gear_Motor, 2);
	expect_constraint_mapping(t, entasis.Angular_Axis_Motor, 2);
	expect_constraint_mapping(t, entasis.Angular_Hinge, 2);
	expect_constraint_mapping(t, entasis.Angular_Motor, 2);
	expect_constraint_mapping(t, entasis.Angular_Servo, 2);
	expect_constraint_mapping(t, entasis.Angular_Swivel_Hinge, 2);
	expect_constraint_mapping(t, entasis.Area_Constraint, 3);
	expect_constraint_mapping(t, entasis.Ball_Socket, 2);
	expect_constraint_mapping(t, entasis.Ball_Socket_Motor, 2);
	expect_constraint_mapping(t, entasis.Ball_Socket_Servo, 2);
	expect_constraint_mapping(t, entasis.Center_Distance_Constraint, 2);
	expect_constraint_mapping(t, entasis.Center_Distance_Limit, 2);
	expect_constraint_mapping(t, entasis.Distance_Limit, 2);
	expect_constraint_mapping(t, entasis.Distance_Servo, 2);
	expect_constraint_mapping(t, entasis.Hinge, 2);
	expect_constraint_mapping(t, entasis.Linear_Axis_Limit, 2);
	expect_constraint_mapping(t, entasis.Linear_Axis_Motor, 2);
	expect_constraint_mapping(t, entasis.Linear_Axis_Servo, 2);
	expect_constraint_mapping(t, entasis.One_Body_Angular_Motor, 1);
	expect_constraint_mapping(t, entasis.One_Body_Angular_Servo, 1);
	expect_constraint_mapping(t, entasis.One_Body_Linear_Motor, 1);
	expect_constraint_mapping(t, entasis.One_Body_Linear_Servo, 1);
	expect_constraint_mapping(t, entasis.Point_On_Line_Servo, 2);
	expect_constraint_mapping(t, entasis.Swing_Limit, 2);
	expect_constraint_mapping(t, entasis.Swivel_Hinge, 2);
	expect_constraint_mapping(t, entasis.Twist_Limit, 2);
	expect_constraint_mapping(t, entasis.Twist_Motor, 2);
	expect_constraint_mapping(t, entasis.Twist_Servo, 2);
	expect_constraint_mapping(t, entasis.Volume_Constraint, 4);
	expect_constraint_mapping(t, entasis.Weld, 2);

	servo := entasis.servo_settings(10, 0.5, 100);
	motor := entasis.motor_settings(100, 1);
	testing.expect_value(t, servo.maximum_speed, f32(10));
	testing.expect_value(t, servo.base_speed, f32(0.5));
	testing.expect_value(t, servo.maximum_force, f32(100));
	testing.expect_value(t, motor.maximum_force, f32(100));
	testing.expect_value(t, motor.damping, f32(1));
}

@(test)
constraint_add_get_apply_inspect_enumerate_and_remove_round_trip :: proc(t: ^testing.T)
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

	view, view_status := entasis.active_body_view(&world);
	if !testing.expect_value(t, view_status, entasis.Status.Ok)
	{
		return;
	}

	description := entasis.Ball_Socket{
		local_offset_a={0.25, 0, 0},
		local_offset_b={-0.25, 0, 0},
		spring_settings=entasis.spring_settings(30, 1),
	};
	handle, add_status := entasis.constraint_add_2(&world, a, b, description);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, !entasis.body_view_valid(&world, view));

	readback: entasis.Ball_Socket;
	testing.expect_value(
		t, entasis.constraint_get(&world, handle, &readback), entasis.Status.Ok,
	);
	testing.expect_value(t, readback, description);

	info, inspect_status := entasis.constraint_inspect(&world, handle);
	testing.expect_value(t, inspect_status, entasis.Status.Ok);
	testing.expect_value(t, info.handle, handle);
	testing.expect_value(t, info.type_id, entasis.constraint_type_id(entasis.Ball_Socket));
	testing.expect_value(t, info.body_count, u8(2));
	testing.expect_value(t, info.state, entasis.Constraint_State.Active);
	testing.expect_value(t, info.bodies[0], a);
	testing.expect_value(t, info.bodies[1], b);

	description.local_offset_a.x = 1.25;
	testing.expect_value(
		t, entasis.constraint_apply(&world, handle, description), entasis.Status.Ok,
	);
	readback = {};
	testing.expect_value(
		t, entasis.constraint_get(&world, handle, &readback), entasis.Status.Ok,
	);
	testing.expect_value(t, readback, description);

	count, count_status := entasis.constraint_count(&world);
	testing.expect_value(t, count_status, entasis.Status.Ok);
	testing.expect_value(t, count, 1);
	infos: [1]entasis.Constraint_Info;
	written, total, enumerate_status := entasis.constraint_enumerate(
		&world, infos[:],
	);
	testing.expect_value(t, enumerate_status, entasis.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, total, 1);
	testing.expect_value(t, infos[0].handle, handle);

	written, total, enumerate_status = entasis.constraint_enumerate(
		&world, infos[:0],
	);
	testing.expect_value(t, enumerate_status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, total, 1);

	testing.expect_value(
		t, entasis.constraint_remove(&world, handle), entasis.Status.Ok,
	);
	_, missing_status := entasis.constraint_inspect(&world, handle);
	testing.expect_value(t, missing_status, entasis.Status.Not_Found);
}

@(test)
constraint_helpers_cover_one_three_and_four_body_descriptions :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, constraint_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	bodies: [4]entasis.Body_Handle;
	for index in 0 ..< len(bodies)
	{
		handle, status := entasis.body_add(&world, constraint_body(index));
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return;
		}
		bodies[index] = handle;
	}

	one, one_status := entasis.constraint_add_1(
		&world,
		bodies[0],
		entasis.One_Body_Linear_Motor{
			settings=entasis.motor_settings(100, 1),
		},
	);
	three, three_status := entasis.constraint_add_3(
		&world,
		bodies[0], bodies[1], bodies[2],
		entasis.Area_Constraint{
			target_scaled_area=1,
			spring_settings=entasis.spring_settings(30, 1),
		},
	);
	four, four_status := entasis.constraint_add_4(
		&world,
		bodies[0], bodies[1], bodies[2], bodies[3],
		entasis.Volume_Constraint{
			target_scaled_volume=1,
			spring_settings=entasis.spring_settings(30, 1),
		},
	);
	testing.expect_value(t, one_status, entasis.Status.Ok);
	testing.expect_value(t, three_status, entasis.Status.Ok);
	testing.expect_value(t, four_status, entasis.Status.Ok);

	one_info, _ := entasis.constraint_inspect(&world, one);
	three_info, _ := entasis.constraint_inspect(&world, three);
	four_info, _ := entasis.constraint_inspect(&world, four);
	testing.expect_value(t, one_info.body_count, u8(1));
	testing.expect_value(t, three_info.body_count, u8(3));
	testing.expect_value(t, four_info.body_count, u8(4));
}

@(test)
constraint_batches_preserve_order_and_prefix_failures :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, constraint_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	bodies: [8]entasis.Body_Handle;
	for index in 0 ..< len(bodies)
	{
		handle, status := entasis.body_add(&world, constraint_body(index));
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return;
		}
		bodies[index] = handle;
	}

	pairs: [4][2]entasis.Body_Handle;
	descriptions: [4]entasis.Ball_Socket;
	for index in 0 ..< len(pairs)
	{
		pairs[index] = {bodies[index * 2], bodies[index * 2 + 1]};
		descriptions[index] = {
			local_offset_a={f32(index), 0, 0},
			local_offset_b={-f32(index), 0, 0},
			spring_settings=entasis.spring_settings(30, 1),
		};
	}
	handles: [4]entasis.Constraint_Handle;
	written, add_status := entasis.constraint_add_batch_2(
		&world, pairs[:], descriptions[:], handles[:],
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, written, len(handles));
	for index in 0 ..< len(handles)
	{
		readback: entasis.Ball_Socket;
		testing.expect_value(
			t, entasis.constraint_get(&world, handles[index], &readback), entasis.Status.Ok,
		);
		testing.expect_value(t, readback, descriptions[index]);
		if index > 0
		{
			testing.expect_value(
				t, handles[index].value, handles[index - 1].value + 1,
			);
		}
		descriptions[index].local_offset_a.y = f32(index + 10);
	}
	applied, apply_status := entasis.constraint_apply_batch(
		&world, handles[:], descriptions[:],
	);
	testing.expect_value(t, apply_status, entasis.Status.Ok);
	testing.expect_value(t, applied, len(handles));
	removed, remove_status := entasis.constraint_remove_batch(&world, handles[:]);
	testing.expect_value(t, remove_status, entasis.Status.Ok);
	testing.expect_value(t, removed, len(handles));

	failing_descriptions := [3]entasis.Ball_Socket{
		{spring_settings=entasis.spring_settings(30, 1)},
		{spring_settings={}},
		{spring_settings=entasis.spring_settings(30, 1)},
	};
	failing_pairs := [3][2]entasis.Body_Handle{
		{bodies[0], bodies[1]},
		{bodies[2], bodies[3]},
		{bodies[4], bodies[5]},
	};
	failing_handles: [3]entasis.Constraint_Handle;
	written, add_status = entasis.constraint_add_batch_2(
		&world, failing_pairs[:], failing_descriptions[:], failing_handles[:],
	);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, add_status, entasis.Status.Invalid_Argument);
	testing.expect(t, entasis.constraint_handle_is_valid(failing_handles[0]));
	testing.expect(t, !entasis.constraint_handle_is_valid(failing_handles[1]));
	testing.expect(t, !entasis.constraint_handle_is_valid(failing_handles[2]));
}

@(test)
constraint_inspection_tracks_sleeping_storage_without_exposing_it :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, constraint_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);

	a, a_status := entasis.body_add(&world, constraint_body(0, 1));
	b, b_status := entasis.body_add(&world, constraint_body(1, 1));
	if !testing.expect_value(t, a_status, entasis.Status.Ok) ||
		!testing.expect_value(t, b_status, entasis.Status.Ok)
	{
		return;
	}
	handle, add_status := entasis.constraint_add_2(
		&world,
		a, b,
		entasis.Ball_Socket{spring_settings=entasis.spring_settings(30, 1)},
	);
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}

	inactive := false;
	for step_index in 0 ..< 512
	{
		_ = step_index;
		if !testing.expect_value(
			t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok,
		)
		{
			return;
		}
		info, inspect_status := entasis.constraint_inspect(&world, handle);
		if !testing.expect_value(t, inspect_status, entasis.Status.Ok)
		{
			return;
		}
		if info.state == .Inactive
		{
			inactive = true;
			break;
		}
	}
	testing.expect(t, inactive);
	if !inactive
	{
		return;
	}

	readback: entasis.Ball_Socket;
	testing.expect_value(
		t, entasis.constraint_get(&world, handle, &readback), entasis.Status.Ok,
	);
	testing.expect_value(t, readback.spring_settings, entasis.spring_settings(30, 1));

	infos: [1]entasis.Constraint_Info;
	written, total, enumerate_status := entasis.constraint_enumerate(&world, infos[:]);
	testing.expect_value(t, enumerate_status, entasis.Status.Ok);
	testing.expect_value(t, written, 1);
	testing.expect_value(t, total, 1);
	testing.expect_value(t, infos[0].state, entasis.Constraint_State.Inactive);
}
