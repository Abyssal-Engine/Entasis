package constraints_tests

import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Constraint_Description :: struct
{
	target: f32,
}

Compression_Dispatch_State :: struct
{
	inner:   ^util.Thread_Dispatcher_Boundary,
	visited: [physics.MAXIMUM_SOLVER_WORKER_COUNT]physics.Reference_State,
	maximum_worker_count: int,
}

Compression_Dispatch_Job :: struct
{
	state:             ^Compression_Dispatch_State,
	worker:            util.Dispatcher_Worker_Proc,
	unmanaged_context: rawptr,
}

compression_recording_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	job := (^Compression_Dispatch_Job)(dispatcher.unmanaged_context);
	job.state.visited[worker_index] = .Present;
	inner_boundary := dispatcher^;
	inner_boundary.unmanaged_context = job.unmanaged_context;
	job.worker(worker_index, &inner_boundary);
}

compression_recording_dispatch :: proc "contextless" (
	dispatcher: ^util.Thread_Dispatcher_Boundary,
	worker: util.Dispatcher_Worker_Proc,
	maximum_worker_count: int,
	unmanaged_context: rawptr,
) -> util.Threading_Status
{
	if dispatcher == nil || dispatcher.dispatcher == nil
	{
		return .Invalid_Argument;
	}
	state := (^Compression_Dispatch_State)(dispatcher.dispatcher);
	state.maximum_worker_count = max(
		state.maximum_worker_count, maximum_worker_count,
	);
	job := Compression_Dispatch_Job{
		state=state,
		worker=worker,
		unmanaged_context=unmanaged_context,
	};
	return state.inner.dispatch(
		state.inner, compression_recording_worker,
		maximum_worker_count, &job,
	);
}

compression_recording_worker_pool :: proc (
	dispatcher: ^util.Thread_Dispatcher_Boundary, worker_index: int,
) -> (^util.Buffer_Pool, util.Threading_Status)
{
	if dispatcher == nil || dispatcher.dispatcher == nil
	{
		return nil, .Invalid_Argument;
	}
	state := (^Compression_Dispatch_State)(dispatcher.dispatcher);
	return state.inner.worker_pool(state.inner, worker_index);
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 24
	{
		testing.expect_value(
			t,
			util.buffer_pool_ensure_capacity_for_power(pool, 1 << 20, power),
			util.Memory_Status.Ok
		);
	}
}

dynamic_description :: proc(index: int) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position={f32(index), f32(index * 2), f32(-index)}},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={continuity=physics.continuous_detection_discrete(), maximum_speculative_margin=1},
		activity={sleep_threshold=0.01, minimum_timestep_count_under_threshold=32},
	};
}

prepare_solver :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool, bodies: ^physics.Bodies, solver: ^physics.Solver,
	fallback_threshold: int = 3,
) -> [4]physics.Body_Handle
{
	testing.expect_value(t, physics.bodies_initialize(bodies, 16, 4, 8, pool), physics.Physics_Status.Ok);
	handles: [4]physics.Body_Handle;
	for index in 0 ..< 4
	{
		description := dynamic_description(index);
		handle, status := physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		handles[index] = handle;
	}
	testing.expect_value(
		t,
		physics.solver_initialize(solver, bodies, 128, fallback_threshold + 2, fallback_threshold, 4, pool),
		physics.Physics_Status.Ok
	);
	return handles;
}

valid_spring :: proc() -> physics.Spring_Settings
{
	return {angular_frequency=12.5, twice_damping_ratio=1.5};
}
valid_servo :: proc() -> physics.Servo_Settings
{
	return {maximum_speed=25, base_speed=0.5, maximum_force=1000};
}
valid_motor :: proc() -> physics.Motor_Settings
{
	return {maximum_force=750, damping=20};
}
unit_x :: proc() -> util.Vector3
{
	return {1, 0, 0};
}
unit_y :: proc() -> util.Vector3
{
	return {0, 1, 0};
}
unit_z :: proc() -> util.Vector3
{
	return {0, 0, 1};
}

valid_material :: proc() -> physics.Contact_Material_Properties
{
	return {friction_coefficient=0.7, spring_settings=valid_spring(), maximum_recovery_velocity=3};
}

valid_contact :: proc(index: int) -> physics.Constraint_Contact_Data
{
	return {offset_a={f32(index + 1), f32(index + 2), f32(index + 3)}, penetration_depth=f32(index + 1) * 0.1};
}

valid_nonconvex_contact :: proc(index: int) -> physics.Nonconvex_Constraint_Contact_Data
{
	return {
		offset_a={f32(index + 1), f32(index + 2), f32(index + 3)},
		normal=unit_y(),
		penetration_depth=f32(index + 1) * 0.1,
	};
}

description_bytes_match :: proc "contextless" (a, b: ^$T) -> physics.Reference_State
{
	left := ([^]u8)(a);
	right := ([^]u8)(b);
	for index in 0 ..< size_of(T)
	{
		if left[index] != right[index]
		{
			return .Missing;
		}
	}
	return .Present;
}

round_trip_description :: proc(
	t: ^testing.T, solver: ^physics.Solver, handles: ^[4]physics.Body_Handle, source, updated: ^$T,
)
{
	handle, add_status := physics.solver_add(solver, handles, source);
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	observe_type_batch_buffers(t, solver, handle);
	readback: T;
	testing.expect_value(t, physics.solver_get_description(solver, handle, &readback), physics.Physics_Status.Ok);
	testing.expect_value(t, description_bytes_match(source, &readback), physics.Reference_State.Present);
	testing.expect_value(t, physics.solver_apply_description(solver, handle, updated), physics.Physics_Status.Ok);
	readback = {};
	testing.expect_value(t, physics.solver_get_description(solver, handle, &readback), physics.Physics_Status.Ok);
	testing.expect_value(t, description_bytes_match(updated, &readback), physics.Reference_State.Present);
	testing.expect_value(t, physics.solver_remove(solver, handle), physics.Physics_Status.Ok);
}

custom_validate :: proc "contextless" (type_id: i32, description: rawptr) -> physics.Physics_Status
{
	if type_id < physics.FIRST_CALLER_CONSTRAINT_TYPE_ID ||
		type_id >= physics.CONSTRAINT_TYPE_ID_CAPACITY || description == nil ||
		(^Custom_Constraint_Description)(description).target < 0
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

custom_kernel :: proc "contextless" (
	prestep: rawptr, bodies: ^[4]physics.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, active_mask: util.I32x8, phase: physics.Constraint_Kernel_Phase,
)
{
	_ = prestep;
	_ = bodies;
	_ = dt;
	_ = inverse_dt;
	if phase == .Prestep
	{
		value := (^util.F32x8)(impulses);
		value^ = util.wide_select_f32(active_mask, value^, util.F32x8(0));
	}
}

custom_record :: proc() -> physics.Constraint_Type_Record
{
	all := [4]physics.Body_Access_Mask{physics.BODY_ACCESS_ALL, physics.BODY_ACCESS_ALL, {}, {}};
	return {
		type_id=physics.FIRST_CALLER_CONSTRAINT_TYPE_ID,
		body_count=2,
		description_size=size_of(Custom_Constraint_Description),
		prestep_bundle_size=size_of(util.F32x8),
		impulse_bundle_size=size_of(util.F32x8),
		initial_access=all,
		solve_access=all,
		apply_description=physics.constraint_description_transfer,
		build_description=physics.constraint_description_build,
		validate_description=custom_validate,
		prestep_warmstart_solve=custom_kernel,
		incrementally_update=custom_kernel,
		move_record=physics.constraint_storage_move_lane,
		remove_record=physics.constraint_storage_remove_lane,
	};
}

observe_type_batch_buffers :: proc(t: ^testing.T, solver: ^physics.Solver, handle: physics.Constraint_Handle)
{
	location, status := physics.solver_resolve(solver, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := batch.type_id_to_batch_index[location.type_id];
	testing.expect(t, type_batch_index >= 0);
	type_batch := &batch.type_batches.memory[type_batch_index];
	record, lookup_status := physics.constraint_type_registry_lookup(&solver.registry, location.type_id);
	testing.expect_value(t, lookup_status, physics.Physics_Status.Ok);
	testing.expect_value(t, type_batch.prestep_bundle_size, record.prestep_bundle_size);
	testing.expect_value(t, type_batch.impulse_bundle_size, record.impulse_bundle_size);
	testing.expect(t, type_batch.body_references.memory != nil);
	testing.expect(t, type_batch.prestep_data.memory != nil);
	testing.expect(t, type_batch.accumulated_impulses.memory != nil);
	testing.expect(t, type_batch.index_to_handle.memory != nil);
	testing.expect(t, rawptr(type_batch.body_references.memory) != rawptr(type_batch.prestep_data.memory));
	testing.expect(t, rawptr(type_batch.prestep_data.memory) != rawptr(type_batch.accumulated_impulses.memory));
	testing.expect(t, rawptr(type_batch.accumulated_impulses.memory) != rawptr(type_batch.index_to_handle.memory));
	testing.expect_value(t, type_batch.index_to_handle.memory[location.index_in_type_batch], handle.value);
	reference, reference_status := physics.type_batch_read_reference(
		type_batch,
		solver.bodies,
		int(location.index_in_type_batch)
	);
	testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
	for body_index in 0 ..< int(record.body_count)
	{
		body_location, body_status := physics.bodies_resolve(solver.bodies, reference.body_handles[body_index]);
		testing.expect_value(t, body_status, physics.Physics_Status.Ok);
		expected := u32(body_location.index);
		inertia := solver.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[body_location.index].inertia.local;
		if physics.body_inertia_mobility(inertia) == .Kinematic
		{
			expected |= physics.BODY_REFERENCE_KINEMATIC_MASK;
		}
		testing.expect_value(t, reference.encoded_body_references[body_index], i32(expected));
	}
}

@(test)
all_builtin_descriptions_round_trip_through_solver_storage :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	solver: physics.Solver;
	handles := prepare_solver(t, &pool, &bodies, &solver);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(t, solver.registry.registered_count, i32(physics.BUILT_IN_CONSTRAINT_TYPE_COUNT));

	c0, c1, c2, c3 := valid_contact(0), valid_contact(1), valid_contact(2), valid_contact(3);
	n0, n1, n2, n3 := valid_nonconvex_contact(0), valid_nonconvex_contact(1), valid_nonconvex_contact(2), valid_nonconvex_contact(3);
	material := valid_material();
	one_common := physics.Nonconvex_One_Body_Properties{
		material.friction_coefficient,
		material.spring_settings,
		material.maximum_recovery_velocity,
	};
	two_common := physics.Nonconvex_Two_Body_Properties{
		{4, 5, 6},
		material.friction_coefficient,
		material.spring_settings,
		material.maximum_recovery_velocity,
	};

	contact_1_one := physics.Contact_1_One_Body{c0, unit_y(), material};
	contact_1_one_b := contact_1_one;
	contact_1_one_b.contact_0.penetration_depth += 0.01;
	round_trip_description(t, &solver, &handles, &contact_1_one, &contact_1_one_b);
	contact_2_one := physics.Contact_2_One_Body{c0, c1, unit_y(), material};
	contact_2_one_b := contact_2_one;
	contact_2_one_b.contact_1.penetration_depth += 0.01;
	round_trip_description(t, &solver, &handles, &contact_2_one, &contact_2_one_b);
	contact_3_one := physics.Contact_3_One_Body{c0, c1, c2, unit_y(), material};
	contact_3_one_b := contact_3_one;
	contact_3_one_b.contact_2.penetration_depth += 0.01;
	round_trip_description(t, &solver, &handles, &contact_3_one, &contact_3_one_b);
	contact_4_one := physics.Contact_4_One_Body{c0, c1, c2, c3, unit_y(), material};
	contact_4_one_b := contact_4_one;
	contact_4_one_b.contact_3.penetration_depth += 0.01;
	round_trip_description(t, &solver, &handles, &contact_4_one, &contact_4_one_b);
	contact_1 := physics.Contact_1{c0, {4, 5, 6}, unit_y(), material};
	contact_1_b := contact_1;
	contact_1_b.offset_b.x += 1;
	round_trip_description(t, &solver, &handles, &contact_1, &contact_1_b);
	contact_2 := physics.Contact_2{c0, c1, {4, 5, 6}, unit_y(), material};
	contact_2_b := contact_2;
	contact_2_b.offset_b.x += 1;
	round_trip_description(t, &solver, &handles, &contact_2, &contact_2_b);
	contact_3 := physics.Contact_3{c0, c1, c2, {4, 5, 6}, unit_y(), material};
	contact_3_b := contact_3;
	contact_3_b.offset_b.x += 1;
	round_trip_description(t, &solver, &handles, &contact_3, &contact_3_b);
	contact_4 := physics.Contact_4{c0, c1, c2, c3, {4, 5, 6}, unit_y(), material};
	contact_4_b := contact_4;
	contact_4_b.offset_b.x += 1;
	round_trip_description(t, &solver, &handles, &contact_4, &contact_4_b);
	nc2o := physics.Contact_2_Nonconvex_One_Body{one_common, n0, n1};
	nc2ob:=nc2o;
	nc2ob.contact_1.penetration_depth+=0.01;
	round_trip_description(t, &solver, &handles, &nc2o, &nc2ob);
	nc3o := physics.Contact_3_Nonconvex_One_Body{one_common, n0, n1, n2};
	nc3ob:=nc3o;
	nc3ob.contact_2.penetration_depth+=0.01;
	round_trip_description(t, &solver, &handles, &nc3o, &nc3ob);
	nc4o := physics.Contact_4_Nonconvex_One_Body{one_common, n0, n1, n2, n3};
	nc4ob:=nc4o;
	nc4ob.contact_3.penetration_depth+=0.01;
	round_trip_description(t, &solver, &handles, &nc4o, &nc4ob);
	nc2 := physics.Contact_2_Nonconvex{two_common, n0, n1};
	nc2b:=nc2;
	nc2b.common.offset_b.x+=1;
	round_trip_description(t, &solver, &handles, &nc2, &nc2b);
	nc3 := physics.Contact_3_Nonconvex{two_common, n0, n1, n2};
	nc3b:=nc3;
	nc3b.common.offset_b.x+=1;
	round_trip_description(t, &solver, &handles, &nc3, &nc3b);
	nc4 := physics.Contact_4_Nonconvex{two_common, n0, n1, n2, n3};
	nc4b:=nc4;
	nc4b.common.offset_b.x+=1;
	round_trip_description(t, &solver, &handles, &nc4, &nc4b);

	spring, servo, motor := valid_spring(), valid_servo(), valid_motor();
	q := util.quaternion_identity();
	aa_gear := physics.Angular_Axis_Gear_Motor{unit_x(), 2, motor};
	aa_gear_b:=aa_gear;
	aa_gear_b.velocity_scale=3;
	round_trip_description(t, &solver, &handles, &aa_gear, &aa_gear_b);
	aa_motor := physics.Angular_Axis_Motor{unit_x(), 2, motor};
	aa_motor_b:=aa_motor;
	aa_motor_b.target_velocity=3;
	round_trip_description(t, &solver, &handles, &aa_motor, &aa_motor_b);
	angular_hinge:=physics.Angular_Hinge{unit_x(), unit_y(), spring};
	angular_hinge_b:=angular_hinge;
	angular_hinge_b.spring_settings.angular_frequency+=1;
	round_trip_description(t, &solver, &handles, &angular_hinge, &angular_hinge_b);
	angular_motor:=physics.Angular_Motor{{1, 2, 3}, motor};
	angular_motor_b:=angular_motor;
	angular_motor_b.target_velocity_local_a.x+=1;
	round_trip_description(t, &solver, &handles, &angular_motor, &angular_motor_b);
	angular_servo:=physics.Angular_Servo{q, spring, servo};
	angular_servo_b:=angular_servo;
	angular_servo_b.servo_settings.base_speed+=1;
	round_trip_description(t, &solver, &handles, &angular_servo, &angular_servo_b);
	angular_swivel:=physics.Angular_Swivel_Hinge{unit_x(), unit_y(), spring};
	angular_swivel_b:=angular_swivel;
	angular_swivel_b.spring_settings.angular_frequency+=1;
	round_trip_description(t, &solver, &handles, &angular_swivel, &angular_swivel_b);
	area:=physics.Area_Constraint{2, spring};
	area_b:=area;
	area_b.target_scaled_area=3;
	round_trip_description(t, &solver, &handles, &area, &area_b);
	ball:=physics.Ball_Socket{{1, 2, 3}, {4, 5, 6}, spring};
	ball_b:=ball;
	ball_b.local_offset_a.x+=1;
	round_trip_description(t, &solver, &handles, &ball, &ball_b);
	ball_motor:=physics.Ball_Socket_Motor{{4, 5, 6}, {1, 2, 3}, motor};
	ball_motor_b:=ball_motor;
	ball_motor_b.local_offset_b.x+=1;
	round_trip_description(t, &solver, &handles, &ball_motor, &ball_motor_b);
	ball_servo:=physics.Ball_Socket_Servo{{1, 2, 3}, {4, 5, 6}, spring, servo};
	ball_servo_b:=ball_servo;
	ball_servo_b.local_offset_a.x+=1;
	round_trip_description(t, &solver, &handles, &ball_servo, &ball_servo_b);
	center:=physics.Center_Distance_Constraint{2, spring};
	center_b:=center;
	center_b.target_distance=3;
	round_trip_description(t, &solver, &handles, &center, &center_b);
	center_limit:=physics.Center_Distance_Limit{1, 3, spring};
	center_limit_b:=center_limit;
	center_limit_b.maximum_distance=4;
	round_trip_description(t, &solver, &handles, &center_limit, &center_limit_b);
	distance_limit:=physics.Distance_Limit{{1, 2, 3}, {4, 5, 6}, 1, 3, spring};
	distance_limit_b:=distance_limit;
	distance_limit_b.maximum_distance=4;
	round_trip_description(t, &solver, &handles, &distance_limit, &distance_limit_b);
	distance_servo:=physics.Distance_Servo{{1, 2, 3}, {4, 5, 6}, 2, servo, spring};
	distance_servo_b:=distance_servo;
	distance_servo_b.target_distance=3;
	round_trip_description(t, &solver, &handles, &distance_servo, &distance_servo_b);
	hinge:=physics.Hinge{{1, 2, 3}, unit_x(), {4, 5, 6}, unit_y(), spring};
	hinge_b:=hinge;
	hinge_b.local_offset_a.x+=1;
	round_trip_description(t, &solver, &handles, &hinge, &hinge_b);
	axis_limit:=physics.Linear_Axis_Limit{{1, 2, 3}, {4, 5, 6}, unit_x(), -2, 3, spring};
	axis_limit_b:=axis_limit;
	axis_limit_b.maximum_offset=4;
	round_trip_description(t, &solver, &handles, &axis_limit, &axis_limit_b);
	axis_motor:=physics.Linear_Axis_Motor{{1, 2, 3}, {4, 5, 6}, unit_x(), 2, motor};
	axis_motor_b:=axis_motor;
	axis_motor_b.target_velocity=3;
	round_trip_description(t, &solver, &handles, &axis_motor, &axis_motor_b);
	axis_servo:=physics.Linear_Axis_Servo{{1, 2, 3}, {4, 5, 6}, unit_x(), 2, servo, spring};
	axis_servo_b:=axis_servo;
	axis_servo_b.target_offset=3;
	round_trip_description(t, &solver, &handles, &axis_servo, &axis_servo_b);
	one_angular_motor:=physics.One_Body_Angular_Motor{{1, 2, 3}, motor};
	one_angular_motor_b:=one_angular_motor;
	one_angular_motor_b.target_velocity.x+=1;
	round_trip_description(t, &solver, &handles, &one_angular_motor, &one_angular_motor_b);
	one_angular_servo:=physics.One_Body_Angular_Servo{q, spring, servo};
	one_angular_servo_b:=one_angular_servo;
	one_angular_servo_b.servo_settings.base_speed+=1;
	round_trip_description(t, &solver, &handles, &one_angular_servo, &one_angular_servo_b);
	one_linear_motor:=physics.One_Body_Linear_Motor{{1, 2, 3}, {4, 5, 6}, motor};
	one_linear_motor_b:=one_linear_motor;
	one_linear_motor_b.local_offset.x+=1;
	round_trip_description(t, &solver, &handles, &one_linear_motor, &one_linear_motor_b);
	one_linear_servo:=physics.One_Body_Linear_Servo{{1, 2, 3}, {4, 5, 6}, spring, servo};
	one_linear_servo_b:=one_linear_servo;
	one_linear_servo_b.target.x+=1;
	round_trip_description(t, &solver, &handles, &one_linear_servo, &one_linear_servo_b);
	point_line:=physics.Point_On_Line_Servo{{1, 2, 3}, {4, 5, 6}, unit_x(), servo, spring};
	point_line_b:=point_line;
	point_line_b.local_offset_a.x+=1;
	round_trip_description(t, &solver, &handles, &point_line, &point_line_b);
	swing:=physics.Swing_Limit{unit_x(), unit_y(), -0.5, spring};
	swing_b:=swing;
	swing_b.minimum_dot=-0.25;
	round_trip_description(t, &solver, &handles, &swing, &swing_b);
	swivel:=physics.Swivel_Hinge{{1, 2, 3}, unit_x(), {4, 5, 6}, unit_y(), spring};
	swivel_b:=swivel;
	swivel_b.local_offset_a.x+=1;
	round_trip_description(t, &solver, &handles, &swivel, &swivel_b);
	twist_limit:=physics.Twist_Limit{q, q, -1, 1, spring};
	twist_limit_b:=twist_limit;
	twist_limit_b.maximum_angle=2;
	round_trip_description(t, &solver, &handles, &twist_limit, &twist_limit_b);
	twist_motor:=physics.Twist_Motor{unit_x(), unit_y(), 2, motor};
	twist_motor_b:=twist_motor;
	twist_motor_b.target_velocity=3;
	round_trip_description(t, &solver, &handles, &twist_motor, &twist_motor_b);
	twist_servo:=physics.Twist_Servo{q, q, 0.5, spring, servo};
	twist_servo_b:=twist_servo;
	twist_servo_b.target_angle=0.75;
	round_trip_description(t, &solver, &handles, &twist_servo, &twist_servo_b);
	volume:=physics.Volume_Constraint{2, spring};
	volume_b:=volume;
	volume_b.target_scaled_volume=3;
	round_trip_description(t, &solver, &handles, &volume, &volume_b);
	weld:=physics.Weld{{1, 2, 3}, q, spring};
	weld_b:=weld;
	weld_b.local_offset.x+=1;
	round_trip_description(t, &solver, &handles, &weld, &weld_b);

	// caller records retain the builtin builder and current full storage stride.
	// their scalar prefixes are caller data, including nonunit/negative fields
	for builtin_type_id in physics.CONTACT_1_ONE_BODY_TYPE_ID ..= physics.CONTACT_4_TYPE_ID
	{
		builtin_record, builtin_status := physics.constraint_type_registry_lookup(
			&solver.registry, i32(builtin_type_id),
		);
		testing.expect_value(t, builtin_status, physics.Physics_Status.Ok);
		cloned_record := builtin_record^;
		cloned_record.type_id = i32(physics.FIRST_CALLER_CONSTRAINT_TYPE_ID + builtin_type_id);
		cloned_record.validate_description = custom_validate;
		cloned_record.registration = .Missing;
		testing.expect_value(
			t, physics.solver_register_constraint_type(&solver, cloned_record),
			physics.Physics_Status.Ok,
		);
		scalar_count := int(cloned_record.description_size) / size_of(f32);
		source_prefix: [physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES / size_of(f32)]f32;
		readback_prefix: [physics.CONSTRAINT_DESCRIPTION_STORAGE_BYTES / size_of(f32)]f32;
		for field in 0 ..< scalar_count
		{
			source_prefix[field] = -f32(field + 1) * 0.125;
		}
		source_prefix[0] = 1.25;
		cloned_handle, cloned_add_status := physics.solver_add_raw(
			&solver, cloned_record.type_id, &handles, &source_prefix[0],
		);
		testing.expect_value(t, cloned_add_status, physics.Physics_Status.Ok);
		observe_type_batch_buffers(t, &solver, cloned_handle);
		for revision in 0 ..< 2
		{
			if revision > 0
			{
				for field in 0 ..< scalar_count
				{
					source_prefix[field] += 0.25;
				}
				testing.expect_value(
					t, physics.solver_apply_description_raw(
					&solver, cloned_handle, cloned_record.type_id, &source_prefix[0],
				), physics.Physics_Status.Ok,
				);
			}
			for field in 0 ..< len(readback_prefix)
			{
				readback_prefix[field] = 123.5;
			}
			testing.expect_value(
				t, physics.solver_get_description_raw(
				&solver, cloned_handle, cloned_record.type_id, &readback_prefix[0],
				int(cloned_record.description_size),
			), physics.Physics_Status.Ok,
			);
			for field in 0 ..< scalar_count
			{
				testing.expect_value(t, readback_prefix[field], source_prefix[field]);
			}
			testing.expect_value(t, readback_prefix[scalar_count], f32(123.5));
		}
		cloned_location, cloned_resolve_status := physics.solver_resolve(&solver, cloned_handle);
		testing.expect_value(t, cloned_resolve_status, physics.Physics_Status.Ok);
		cloned_batch := &solver.active_set.batches.memory[cloned_location.batch_index];
		cloned_type_batch := &cloned_batch.type_batches.memory[
			cloned_batch.type_id_to_batch_index[cloned_location.type_id]
		];
		stored_prefix := physics.type_batch_prestep_bundle(
			cloned_type_batch, int(cloned_location.index_in_type_batch),
		);
		lane := int(cloned_location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT;
		flat_stride := scalar_count * size_of(util.F32x8);
		testing.expect_value(
			t, cloned_record.build_description(
			stored_prefix, &readback_prefix[0], int(cloned_record.type_id),
			int(cloned_record.description_size), flat_stride, lane,
		), physics.Physics_Status.Ok,
		);
		for field in 0 ..< scalar_count
		{
			testing.expect_value(t, readback_prefix[field], source_prefix[field]);
		}
		other_type_id := (builtin_type_id + 1) % 8;
		other_record, other_status := physics.constraint_type_registry_lookup(
			&solver.registry, i32(other_type_id),
		);
		testing.expect_value(t, other_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, cloned_record.build_description(
			stored_prefix, &readback_prefix[0], int(cloned_record.type_id),
			int(cloned_record.description_size), int(other_record.prestep_bundle_size), lane,
		), physics.Physics_Status.Invalid_Argument,
		);
		testing.expect_value(
			t, cloned_record.build_description(
			stored_prefix, &readback_prefix[0], int(cloned_record.type_id),
			int(cloned_record.description_size),
			int(cloned_record.prestep_bundle_size) + size_of(util.F32x8), lane,
		), physics.Physics_Status.Invalid_Argument,
		);
		// derived coefficients live outside authored records, including caller clones
		testing.expect_value(t, int(cloned_record.prestep_bundle_size), flat_stride);
		testing.expect_value(t, physics.solver_remove(&solver, cloned_handle), physics.Physics_Status.Ok);
	}
}

expect_invalid_transaction :: proc(
	t: ^testing.T, solver: ^physics.Solver, handles: ^[4]physics.Body_Handle, valid, invalid: ^$T,
)
{
	count_before := solver.active_set.constraint_count;
	invalid_handle, invalid_add := physics.solver_add(solver, handles, invalid);
	testing.expect_value(t, invalid_add, physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, invalid_handle.value, i32(-1));
	testing.expect_value(t, solver.active_set.constraint_count, count_before);
	handle, add_status := physics.solver_add(solver, handles, valid);
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	location_before, _ := physics.solver_resolve(solver, handle);
	testing.expect_value(
		t,
		physics.solver_apply_description(solver, handle, invalid),
		physics.Physics_Status.Invalid_Argument
	);
	location_after, _ := physics.solver_resolve(solver, handle);
	testing.expect_value(t, location_after, location_before);
	readback: T;
	testing.expect_value(t, physics.solver_get_description(solver, handle, &readback), physics.Physics_Status.Ok);
	testing.expect_value(t, description_bytes_match(valid, &readback), physics.Reference_State.Present);
	testing.expect_value(t, physics.solver_remove(solver, handle), physics.Physics_Status.Ok);
}

@(test)
invalid_descriptions_reject_without_mutating_solver_storage :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	solver: physics.Solver;
	handles:=prepare_solver(t, &pool, &bodies, &solver);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	valid_distance:=physics.Distance_Servo{{}, {}, 2, valid_servo(), valid_spring()};
	invalid0:=valid_distance;
	invalid0.spring_settings.angular_frequency=0;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid0);
	infinity:=transmute(f32)u32(0x7f80_0000);
	invalid1:=valid_distance;
	invalid1.spring_settings.angular_frequency=infinity;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid1);
	invalid2:=valid_distance;
	invalid2.spring_settings.twice_damping_ratio=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid2);
	invalid3:=valid_distance;
	invalid3.servo_settings.maximum_speed=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid3);
	invalid4:=valid_distance;
	invalid4.servo_settings.base_speed=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid4);
	invalid5:=valid_distance;
	invalid5.servo_settings.maximum_force=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_distance, &invalid5);
	valid_motor_description:=physics.Linear_Axis_Motor{{}, {}, unit_x(), 2, valid_motor()};
	invalid6:=valid_motor_description;
	invalid6.settings.maximum_force=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_motor_description, &invalid6);
	invalid7:=valid_motor_description;
	invalid7.settings.damping=-1;
	expect_invalid_transaction(t, &solver, &handles, &valid_motor_description, &invalid7);
	invalid8:=valid_motor_description;
	invalid8.local_axis={};
	expect_invalid_transaction(t, &solver, &handles, &valid_motor_description, &invalid8);
	invalid_axis:=valid_motor_description;
	invalid_axis.local_axis.x=1.00025;
	expect_invalid_transaction(t, &solver, &handles, &valid_motor_description, &invalid_axis);
	valid_angular:=physics.Angular_Servo{util.quaternion_identity(), valid_spring(), valid_servo()};
	invalid_basis:=valid_angular;
	invalid_basis.target_relative_rotation_local_a.w=1.00025;
	expect_invalid_transaction(t, &solver, &handles, &valid_angular, &invalid_basis);
	valid_limit:=physics.Distance_Limit{{}, {}, 1, 2, valid_spring()};
	invalid9:=valid_limit;
	invalid9.minimum_distance=3;
	expect_invalid_transaction(t, &solver, &handles, &valid_limit, &invalid9);
}

@(test)
caller_registered_constraint_uses_dense_dispatch :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	solver: physics.Solver;
	handles:=prepare_solver(t, &pool, &bodies, &solver);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(
		t,
		physics.solver_register_constraint_type(&solver, custom_record()),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, solver.registry.registered_count, i32(physics.BUILT_IN_CONSTRAINT_TYPE_COUNT+1));
	description:=Custom_Constraint_Description{7};
	handle, status:=physics.solver_add_raw(&solver, physics.FIRST_CALLER_CONSTRAINT_TYPE_ID, &handles, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	observe_type_batch_buffers(t, &solver, handle);
	readback:Custom_Constraint_Description;
	testing.expect_value(
		t,
		physics.solver_get_description_raw(
		&solver,
		handle,
		physics.FIRST_CALLER_CONSTRAINT_TYPE_ID,
		&readback,
		size_of(readback)
	),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, readback.target, f32(7));
	prepare_status, _ := physics.solver_prepare_work_blocks(&solver, 1);
	testing.expect_value(t, prepare_status, physics.Physics_Status.Ok);
	cache_job: physics.Solver_Substep_Job = {solver=&solver, velocity_iteration_count=4};
	testing.expect_value(t, physics.solver_prepare_contact_coefficient_cache(&cache_job), physics.Physics_Status.Ok);
	testing.expect_value(t, solver.contact_coefficient_offsets.memory, nil);
	testing.expect_value(t, solver.contact_coefficients.memory, nil);
}

@(test)
constraint_add_remove_move_and_handle_reuse_preserve_locations :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	solver: physics.Solver;
	handles:=prepare_solver(t, &pool, &bodies, &solver);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	description:=physics.Ball_Socket{{}, {}, valid_spring()};
	h0, s0:=physics.solver_add(&solver, &handles, &description);
	h1, s1:=physics.solver_add(&solver, &handles, &description);
	testing.expect_value(t, s0, physics.Physics_Status.Ok);
	testing.expect_value(t, s1, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_move_constraint(&solver, h1, 2), physics.Physics_Status.Ok);
	l1, _:=physics.solver_resolve(&solver, h1);
	testing.expect_value(t, l1.batch_index, i32(2));
	testing.expect_value(t, physics.solver_remove(&solver, h0), physics.Physics_Status.Ok);
	replacement, s2:=physics.solver_add(&solver, &handles, &description);
	testing.expect_value(t, s2, physics.Physics_Status.Ok);
	testing.expect_value(t, replacement.value, h0.value);
	lr, _:=physics.solver_resolve(&solver, replacement);
	testing.expect_value(t, lr.type_id, i32(physics.BALL_SOCKET_TYPE_ID));
	testing.expect_value(t, physics.solver_remove(&solver, replacement), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver, h1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_local_inertia(&bodies, handles[3], {}), physics.Physics_Status.Ok);
	moving_pair:=[4]physics.Body_Handle{handles[2], handles[3], {}, {}};
	moving_constraint, moving_status:=physics.solver_add(&solver, &moving_pair, &description);
	testing.expect_value(t, moving_status, physics.Physics_Status.Ok);
	moving_location, _:=physics.solver_resolve(&solver, moving_constraint);
	moving_batch:=&solver.active_set.batches.memory[moving_location.batch_index];
	moving_type_batch:=&moving_batch.type_batches.memory[moving_batch.type_id_to_batch_index[moving_location.type_id]];
	reference_before, _:=physics.type_batch_read_reference(
		moving_type_batch,
		&bodies,
		int(moving_location.index_in_type_batch)
	);
	testing.expect_value(t, reference_before.encoded_body_references[0], i32(2));
	testing.expect_value(
		t,
		reference_before.encoded_body_references[1],
		i32(u32(3)|physics.BODY_REFERENCE_KINEMATIC_MASK)
	);
	testing.expect_value(t, physics.bodies_remove(&bodies, handles[0]), physics.Physics_Status.Ok);
	reference_after, reference_after_status:=physics.type_batch_read_reference(
		moving_type_batch,
		&bodies,
		int(moving_location.index_in_type_batch)
	);
	testing.expect_value(t, reference_after_status, physics.Physics_Status.Ok);
	testing.expect_value(t, reference_after.body_handles[1], handles[3]);
	testing.expect_value(t, reference_after.encoded_body_references[0], i32(2));
	testing.expect_value(t, reference_after.encoded_body_references[1], i32(physics.BODY_REFERENCE_KINEMATIC_MASK));
	testing.expect_value(t, physics.solver_remove(&solver, moving_constraint), physics.Physics_Status.Ok);
	new_description:=dynamic_description(4);
	new_body, new_body_status:=physics.bodies_add(&bodies, &new_description);
	testing.expect_value(t, new_body_status, physics.Physics_Status.Ok);
	fallback_pair:=[4]physics.Body_Handle{handles[1], new_body, {}, {}};
	fallback_constraint, fallback_status:=physics.solver_add(&solver, &fallback_pair, &description);
	testing.expect_value(t, fallback_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_move_constraint(&solver, fallback_constraint, 3), physics.Physics_Status.Ok);
	fallback_counts:=solver.sequential_batch.dynamic_body_constraint_counts;
	testing.expect_value(t, fallback_counts.memory[1], i32(1));
	testing.expect_value(t, fallback_counts.memory[3], i32(1));
	testing.expect_value(t, physics.bodies_remove(&bodies, handles[3]), physics.Physics_Status.Ok);
	testing.expect_value(t, fallback_counts.memory[0], i32(1));
	testing.expect_value(t, fallback_counts.memory[1], i32(1));
	testing.expect_value(t, fallback_counts.memory[3], i32(0));
	fallback_location, _:=physics.solver_resolve(&solver, fallback_constraint);
	fallback_batch:=&solver.active_set.batches.memory[fallback_location.batch_index];
	fallback_type_batch:=&fallback_batch.type_batches.memory[fallback_batch.type_id_to_batch_index[fallback_location.type_id]];
	fallback_reference, fallback_reference_status:=physics.type_batch_read_reference(
		fallback_type_batch,
		&bodies,
		int(fallback_location.index_in_type_batch)
	);
	testing.expect_value(t, fallback_reference_status, physics.Physics_Status.Ok);
	testing.expect_value(t, fallback_reference.body_handles[1], new_body);
	testing.expect_value(t, fallback_reference.encoded_body_references[1], i32(0));
	testing.expect_value(t, physics.solver_remove(&solver, fallback_constraint), physics.Physics_Status.Ok);
}

@(test)
conflicting_constraints_enter_and_leave_sequential_fallback :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies:physics.Bodies;
	solver:physics.Solver;
	handles:=prepare_solver(t, &pool, &bodies, &solver, 2);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	description:=physics.Ball_Socket{{}, {}, valid_spring()};
	h0, _:=physics.solver_add(&solver, &handles, &description);
	h1, _:=physics.solver_add(&solver, &handles, &description);
	h2, _:=physics.solver_add(&solver, &handles, &description);
	l0, _:=physics.solver_resolve(&solver, h0);
	l1, _:=physics.solver_resolve(&solver, h1);
	l2, _:=physics.solver_resolve(&solver, h2);
	testing.expect_value(t, l0.batch_index, i32(0));
	testing.expect_value(t, l1.batch_index, i32(1));
	testing.expect_value(t, l2.batch_index, i32(2));
	testing.expect_value(t, solver.sequential_batch.constraint_count, i32(1));
	testing.expect_value(t, physics.solver_remove(&solver, h0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_compress(&solver), physics.Physics_Status.Ok);
	l2, _=physics.solver_resolve(&solver, h2);
	testing.expect(t, l2.batch_index<2);
	testing.expect_value(t, solver.sequential_batch.constraint_count, i32(0));
	testing.expect_value(t, physics.solver_remove(&solver, h1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver, h2), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_local_inertia(&bodies, handles[3], {}), physics.Physics_Status.Ok);
	shared_kinematic_a:=[4]physics.Body_Handle{handles[0], handles[3], {}, {}};
	shared_kinematic_b:=[4]physics.Body_Handle{handles[1], handles[3], {}, {}};
	kinematic_a, status_a:=physics.solver_add(&solver, &shared_kinematic_a, &description);
	kinematic_b, status_b:=physics.solver_add(&solver, &shared_kinematic_b, &description);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	location_a, _:=physics.solver_resolve(&solver, kinematic_a);
	location_b, _:=physics.solver_resolve(&solver, kinematic_b);
	testing.expect_value(t, location_a.batch_index, i32(0));
	testing.expect_value(t, location_b.batch_index, i32(0));
	batch:=&solver.active_set.batches.memory[0];
	testing.expect_value(t, batch.body_reference_counts.memory[handles[0].value], i32(1));
	testing.expect_value(t, batch.body_reference_counts.memory[handles[1].value], i32(1));
	testing.expect_value(t, batch.body_reference_counts.memory[handles[3].value], i32(0));
	body_description, body_status:=physics.bodies_get_description(&bodies, handles[3]);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	body_description.local_inertia=dynamic_description(3).local_inertia;
	testing.expect_value(
		t,
		physics.bodies_apply_description(&bodies, handles[3], &body_description),
		physics.Physics_Status.Ok
	);
	location_a, _=physics.solver_resolve(&solver, kinematic_a);
	location_b, _=physics.solver_resolve(&solver, kinematic_b);
	testing.expect(t, location_a.batch_index!=location_b.batch_index);
	kinematic_constraints:=[2]physics.Constraint_Handle{kinematic_a, kinematic_b};
	for constraint_handle in kinematic_constraints
	{
		location, _:=physics.solver_resolve(&solver, constraint_handle);
		constraint_batch:=&solver.active_set.batches.memory[location.batch_index];
		type_batch:=&constraint_batch.type_batches.memory[constraint_batch.type_id_to_batch_index[location.type_id]];
		reference, reference_status:=physics.type_batch_read_reference(
			type_batch,
			&bodies,
			int(location.index_in_type_batch)
		);
		testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
		testing.expect_value(t, reference.encoded_body_references[1], i32(3));
	}
	testing.expect_value(t, physics.bodies_set_local_inertia(&bodies, handles[3], {}), physics.Physics_Status.Ok);
	for constraint_handle in kinematic_constraints
	{
		location, _:=physics.solver_resolve(&solver, constraint_handle);
		constraint_batch:=&solver.active_set.batches.memory[location.batch_index];
		type_batch:=&constraint_batch.type_batches.memory[constraint_batch.type_id_to_batch_index[location.type_id]];
		reference, reference_status:=physics.type_batch_read_reference(
			type_batch,
			&bodies,
			int(location.index_in_type_batch)
		);
		testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
		testing.expect_value(t, reference.encoded_body_references[1], i32(u32(3)|physics.BODY_REFERENCE_KINEMATIC_MASK));
	}
	testing.expect_value(t, physics.solver_remove(&solver, kinematic_a), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver, kinematic_b), physics.Physics_Status.Ok);
	one_body_handles:=[4]physics.Body_Handle{handles[2], {}, {}, {}};
	one_body_description:=physics.One_Body_Linear_Servo{{}, {1, 2, 3}, valid_spring(), valid_servo()};
	one_body_constraint, one_body_status:=physics.solver_add(&solver, &one_body_handles, &one_body_description);
	testing.expect_value(t, one_body_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_local_inertia(&bodies, handles[2], {}), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_reference_state(&solver, one_body_constraint), physics.Reference_State.Missing);

	// body creation invalidates world inertia. normal integration prepares it
	// before Solve. supply that existing preparation without a Warmstart
	for body_index in 0 ..< 2
	{
		body_location, body_location_status := physics.bodies_resolve(&bodies, handles[body_index]);
		testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
		physics.pose_integrator_update_world_inertia(
			&bodies.sets.memory[body_location.set_index].dynamics_state.memory[body_location.index],
		);
	}
	// only the third contact enters fallback. execute its registered Solve
	// directly: no Warmstart may be required to initialize derived storage
	fallback_contact := physics.Contact_1{
		contact_0={penetration_depth=0.1}, normal=unit_y(), material=valid_material(),
	};
	fallback_contact.material.friction_coefficient = 0;
	contact_handles: [3]physics.Constraint_Handle;
	for index in 0 ..< len(contact_handles)
	{
		contact_handle, contact_add_status := physics.solver_add(&solver, &handles, &fallback_contact);
		testing.expect_value(t, contact_add_status, physics.Physics_Status.Ok);
		contact_handles[index] = contact_handle;
	}
	contact_location, contact_resolve_status := physics.solver_resolve(&solver, contact_handles[2]);
	testing.expect_value(t, contact_resolve_status, physics.Physics_Status.Ok);
	testing.expect_value(t, contact_location.batch_index, solver.fallback_batch_index);
	testing.expect_value(t, solver.sequential_batch.constraint_count, i32(1));
	contact_batch := &solver.active_set.batches.memory[contact_location.batch_index];
	contact_type_batch := &contact_batch.type_batches.memory[
		contact_batch.type_id_to_batch_index[contact_location.type_id]
	];
	testing.expect_value(t, contact_type_batch.prestep_bundle_size,
		i32(size_of(physics.Contact_1) / size_of(f32) * size_of(util.F32x8)));
	fallback_job := physics.Solver_Substep_Job{
		solver=&solver, dt=1.0 / 60.0, inverse_dt=60, phase=.First, velocity_iteration_count=4,
	};
	testing.expect_value(t, solver.contact_coefficient_offsets.memory, nil);
	testing.expect_value(t, solver.contact_coefficients.memory, nil);
	for _ in 0 ..< 4
	{
		physics.solver_execute_fallback(&fallback_job, .Solve);
		testing.expect_value(t, physics.Physics_Status(fallback_job.status), physics.Physics_Status.Ok);
		contact_impulses := (^physics.Contact_1_Accumulated_Impulses)(physics.type_batch_impulse_bundle(
			contact_type_batch, int(contact_location.index_in_type_batch),
		));
		penetrations := transmute([util.PRODUCTION_LANE_COUNT]f32)contact_impulses.penetration[0];
		penetration := penetrations[int(contact_location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT];
		testing.expect_value(t, util.math_check_f32(penetration), util.Math_Check_Status.Ok);
		testing.expect(t, penetration > 0);
		for body_index in 0 ..< 2
		{
			body_after, body_after_status := physics.bodies_get_description(&bodies, handles[body_index]);
			testing.expect_value(t, body_after_status, physics.Physics_Status.Ok);
			velocity_components := [6]f32{
				body_after.velocity.linear.x, body_after.velocity.linear.y, body_after.velocity.linear.z,
				body_after.velocity.angular.x, body_after.velocity.angular.y, body_after.velocity.angular.z,
			};
			for component in velocity_components
			{
				testing.expect_value(t, util.math_check_f32(component), util.Math_Check_Status.Ok);
			}
			if body_index == 0
			{
				testing.expect(t, body_after.velocity.linear.y > 0);
			}
			else
			{
				testing.expect(t, body_after.velocity.linear.y < 0);
			}
		}
	}
	for contact_handle in contact_handles
	{
		testing.expect_value(t, physics.solver_remove(&solver, contact_handle), physics.Physics_Status.Ok);
	}
}

@(test)
inactive_body_mobility_changes_reject_without_mutation :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	solver: physics.Solver;
	testing.expect_value(
		t, physics.bodies_initialize(&bodies, 16, 4, 8, &pool),
		physics.Physics_Status.Ok,
	);
	shapes: physics.Shape_Registry;
	testing.expect_value(
		t, physics.shape_registry_initialize(&shapes, 8, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.shape_registry_dispose(&shapes);
	testing.expect_value(
		t, physics.bodies_bind_shape_registry(&bodies, &shapes),
		physics.Physics_Status.Ok,
	);
	sphere := physics.Sphere{radius=0.5};
	shape, shape_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	replacement_sphere := physics.Sphere{radius=0.75};
	replacement_shape, replacement_shape_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &replacement_sphere,
	);
	testing.expect_value(
		t, replacement_shape_status, physics.Physics_Status.Ok,
	);
	handles: [4]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description := dynamic_description(index);
		description.collidable.shape = shape;
		handles[index], shape_status = physics.bodies_add(&bodies, &description);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	}
	testing.expect_value(
		t, physics.solver_initialize(&solver, &bodies, 128, 5, 3, 4, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(
		t, physics.bodies_move(&bodies, handles[3], 1),
		physics.Physics_Status.Ok,
	);
	location, location_status := physics.bodies_resolve(&bodies, handles[3]);
	testing.expect_value(t, location_status, physics.Physics_Status.Ok);
	testing.expect(t, location.set_index > physics.BODIES_ACTIVE_SET_INDEX);
	testing.expect(
		t,
		int(location.index) <
		bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count,
	);
	old_description, old_description_status := physics.bodies_get_description(
		&bodies, handles[3],
	);
	testing.expect_value(t, old_description_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.bodies_set_local_inertia(&bodies, handles[3], {}),
		physics.Physics_Status.Invalid_Argument,
	);
	failed_description, failed_description_status := physics.bodies_get_description(
		&bodies, handles[3],
	);
	testing.expect_value(t, failed_description_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, description_bytes_match(&failed_description, &old_description),
		physics.Reference_State.Present,
	);
	updated_description := old_description;
	updated_description.local_inertia = {};
	updated_description.collidable.shape = replacement_shape;
	shape_references := &shapes.batches[physics.SPHERE_TYPE_ID].references;
	old_shape_references := shape_references.memory[physics.typed_index_index(shape)];
	replacement_shape_index := physics.typed_index_index(replacement_shape);
	shape_references.memory[replacement_shape_index] = max(i32);
	old_replacement_shape_references := shape_references.memory[
		replacement_shape_index
	];
	old_constraint_count := solver.active_set.constraint_count;
	testing.expect_value(
		t,
		physics.bodies_apply_description(
		&bodies, handles[3], &updated_description,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	failed_description, failed_description_status = physics.bodies_get_description(
		&bodies, handles[3],
	);
	testing.expect_value(t, failed_description_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, description_bytes_match(&failed_description, &old_description),
		physics.Reference_State.Present,
	);
	testing.expect_value(
		t, shape_references.memory[physics.typed_index_index(shape)],
		old_shape_references,
	);
	testing.expect_value(
		t, shape_references.memory[replacement_shape_index],
		old_replacement_shape_references,
	);
	testing.expect_value(
		t, solver.active_set.constraint_count, old_constraint_count,
	);
}

@(test)
body_mobility_capacity_failure_preserves_body_solver_and_shape_state :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies: physics.Bodies;
	testing.expect_value(
		t, physics.bodies_initialize(&bodies, 16, 4, 8, &pool),
		physics.Physics_Status.Ok,
	);
	shapes: physics.Shape_Registry;
	testing.expect_value(
		t, physics.shape_registry_initialize(&shapes, 8, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.shape_registry_dispose(&shapes);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(
		t, physics.bodies_bind_shape_registry(&bodies, &shapes),
		physics.Physics_Status.Ok,
	);
	sphere := physics.Sphere{radius=0.5};
	shape, shape_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	replacement_sphere := physics.Sphere{radius=0.75};
	replacement_shape, replacement_shape_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &replacement_sphere,
	);
	testing.expect_value(
		t, replacement_shape_status, physics.Physics_Status.Ok,
	);
	handles: [6]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description := dynamic_description(index);
		description.collidable.shape = shape;
		handles[index], shape_status = physics.bodies_add(&bodies, &description);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	}
	solver: physics.Solver;
	testing.expect_value(
		t, physics.solver_initialize(&solver, &bodies, 128, 5, 3, 4, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.solver_dispose(&solver);
	testing.expect_value(
		t, physics.bodies_set_local_inertia(&bodies, handles[3], {}),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.bodies_set_local_inertia(&bodies, handles[4], {}),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.bodies_set_local_inertia(&bodies, handles[5], {}),
		physics.Physics_Status.Ok,
	);
	constraint_description := physics.Ball_Socket{{}, {}, valid_spring()};
	target_pairs := [2][4]physics.Body_Handle{
		{handles[0], handles[3], {}, {}},
		{handles[1], handles[3], {}, {}},
	};
	target_constraints: [2]physics.Constraint_Handle;
	for index in 0 ..< len(target_constraints)
	{
		target_constraints[index], shape_status = physics.solver_add(
			&solver, &target_pairs[index], &constraint_description,
		);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	}
	filler_pair := [4]physics.Body_Handle{handles[4], handles[5], {}, {}};
	filler_constraints: [4]physics.Constraint_Handle;
	for index in 0 ..< len(filler_constraints)
	{
		filler_constraints[index], shape_status = physics.solver_add(
			&solver, &filler_pair, &constraint_description,
		);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, physics.solver_move_constraint(&solver, filler_constraints[index], 1),
			physics.Physics_Status.Ok,
		);
	}
	target_batch := &solver.active_set.batches.memory[1];
	target_type_batch := &target_batch.type_batches.memory[
		target_batch.type_id_to_batch_index[physics.BALL_SOCKET_TYPE_ID]
	];
	testing.expect_value(
		t, target_type_batch.count, i32(target_type_batch.index_to_handle.length),
	);
	old_description, old_description_status := physics.bodies_get_description(
		&bodies, handles[3],
	);
	testing.expect_value(t, old_description_status, physics.Physics_Status.Ok);
	new_description := old_description;
	new_description.local_inertia = dynamic_description(3).local_inertia;
	new_description.collidable.shape = replacement_shape;
	old_shape_references := shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(shape)
	];
	old_replacement_shape_references :=
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(replacement_shape)
	];
	old_locations: [2]physics.Constraint_Location;
	old_references: [2]physics.Constraint_Reference;
	for index in 0 ..< len(target_constraints)
	{
		old_locations[index], shape_status = physics.solver_resolve(
			&solver, target_constraints[index],
		);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
		batch := &solver.active_set.batches.memory[old_locations[index].batch_index];
		type_batch := &batch.type_batches.memory[
			batch.type_id_to_batch_index[old_locations[index].type_id]
		];
		old_references[index], shape_status = physics.type_batch_read_reference(
			type_batch, &bodies, int(old_locations[index].index_in_type_batch),
		);
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	}
	old_batch_counts: [2]i32;
	old_body_counts: [2][6]i32;
	for batch_index in 0 ..< len(old_batch_counts)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		old_batch_counts[batch_index] = batch.constraint_count;
		for body_index in 0 ..< len(handles)
		{
			old_body_counts[batch_index][body_index] =
				batch.body_reference_counts.memory[handles[body_index].value];
		}
	}
	old_fallback_counts: [6]i32;
	for body_index in 0 ..< len(handles)
	{
		old_fallback_counts[body_index] =
			solver.sequential_batch.dynamic_body_constraint_counts.memory[body_index];
	}
	old_fallback_constraint_count := solver.sequential_batch.constraint_count;
	rejecting_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&rejecting_pool, 128),
		util.Memory_Status.Ok,
	);
	seed, seed_status := util.buffer_pool_take_at_least(
		&rejecting_pool, physics.Solver_Body_Mobility_Change_Entry,
		len(target_constraints),
	);
	testing.expect_value(t, seed_status, util.Memory_Status.Ok);
	seed_power := int(seed.id) >> util.BUFFER_POOL_ID_POWER_SHIFT;
	testing.expect_value(
		t, util.buffer_pool_return(&rejecting_pool, &seed),
		util.Memory_Status.Ok,
	);
	for power in 0 ..< util.BUFFER_POOL_POWER_COUNT
	{
		rejecting_pool.pools[power].next_slot = util.BUFFER_POOL_ID_SLOT_COUNT;
		if power != seed_power
		{
			rejecting_pool.pools[power].free_count = 0;
		}
	}
	original_solver_pool := solver.pool;
	solver.pool = &rejecting_pool;
	failure_status := physics.bodies_apply_description(
		&bodies, handles[3], &new_description,
	);
	solver.pool = original_solver_pool;
	testing.expect_value(
		t, failure_status, physics.Physics_Status.Capacity_Missing,
	);
	testing.expect_value(
		t, util.buffer_pool_dispose(&rejecting_pool), util.Memory_Status.Ok,
	);
	failed_description, failed_description_status := physics.bodies_get_description(
		&bodies, handles[3],
	);
	testing.expect_value(t, failed_description_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t, description_bytes_match(&failed_description, &old_description),
		physics.Reference_State.Present,
	);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(shape)
	],
		old_shape_references,
	);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(replacement_shape)
	],
		old_replacement_shape_references,
	);
	testing.expect_value(
		t, target_type_batch.count, i32(target_type_batch.index_to_handle.length),
	);
	for index in 0 ..< len(target_constraints)
	{
		location, location_status := physics.solver_resolve(
			&solver, target_constraints[index],
		);
		testing.expect_value(t, location_status, physics.Physics_Status.Ok);
		testing.expect_value(t, location, old_locations[index]);
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch := &batch.type_batches.memory[
			batch.type_id_to_batch_index[location.type_id]
		];
		reference, reference_status := physics.type_batch_read_reference(
			type_batch, &bodies, int(location.index_in_type_batch),
		);
		testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t, reference.encoded_body_references,
			old_references[index].encoded_body_references,
		);
	}
	for batch_index in 0 ..< len(old_batch_counts)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		testing.expect_value(
			t, batch.constraint_count, old_batch_counts[batch_index],
		);
		for body_index in 0 ..< len(handles)
		{
			testing.expect_value(
				t, batch.body_reference_counts.memory[handles[body_index].value],
				old_body_counts[batch_index][body_index],
			);
		}
	}
	testing.expect_value(
		t, solver.sequential_batch.constraint_count, old_fallback_constraint_count,
	);
	for body_index in 0 ..< len(handles)
	{
		testing.expect_value(
			t, solver.sequential_batch.dynamic_body_constraint_counts.memory[body_index],
			old_fallback_counts[body_index],
		);
	}
	testing.expect_value(
		t, physics.bodies_apply_description(&bodies, handles[3], &new_description),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(shape)
	],
		old_shape_references - 1,
	);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(replacement_shape)
	],
		old_replacement_shape_references + 1,
	);
	for constraint_handle in target_constraints
	{
		location, location_status := physics.solver_resolve(
			&solver, constraint_handle,
		);
		testing.expect_value(t, location_status, physics.Physics_Status.Ok);
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch := &batch.type_batches.memory[
			batch.type_id_to_batch_index[location.type_id]
		];
		reference, reference_status := physics.type_batch_read_reference(
			type_batch, &bodies, int(location.index_in_type_batch),
		);
		testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
		testing.expect_value(t, reference.encoded_body_references[1], i32(3));
	}
}

@(test)
batch_compression_preserves_handles_with_parallel_discovery :: proc(t: ^testing.T)
{
	pool:util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	dispatcher:util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 2, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	inner:=util.thread_dispatcher_boundary(&dispatcher);
	dispatch_state:=Compression_Dispatch_State{inner=inner};
	boundary:=&util.Thread_Dispatcher_Boundary{
		dispatcher=&dispatch_state,
		dispatch=compression_recording_dispatch,
		worker_pool=compression_recording_worker_pool,
		worker_count=inner.worker_count,
	};
	testing.expect_value(t, physics.solver_compression_budget_count(500, 200), 2);
	testing.expect_value(t, physics.solver_compression_budget_count(5000, 2000), 2);
	description:=physics.Ball_Socket{{}, {}, valid_spring()};
	bodies_a:physics.Bodies;
	solver_a:physics.Solver;
	handles_a:=prepare_solver(t, &pool, &bodies_a, &solver_a, 3);
	defer physics.solver_dispose(&solver_a);
	defer physics.bodies_dispose(&bodies_a);
	bodies_b:physics.Bodies;
	solver_b:physics.Solver;
	handles_b:=prepare_solver(t, &pool, &bodies_b, &solver_b, 3);
	defer physics.solver_dispose(&solver_b);
	defer physics.bodies_dispose(&bodies_b);
	pair_a_0:=[4]physics.Body_Handle{handles_a[0], handles_a[1], {}, {}};
	pair_a_1:=[4]physics.Body_Handle{handles_a[2], handles_a[3], {}, {}};
	pair_b_0:=[4]physics.Body_Handle{handles_b[0], handles_b[1], {}, {}};
	pair_b_1:=[4]physics.Body_Handle{handles_b[2], handles_b[3], {}, {}};
	blocker_a_0, status_a0:=physics.solver_add(&solver_a, &pair_a_0, &description);
	testing.expect_value(t, status_a0, physics.Physics_Status.Ok);
	blocker_a_1, status_a1:=physics.solver_add(&solver_a, &pair_a_1, &description);
	testing.expect_value(t, status_a1, physics.Physics_Status.Ok);
	candidate_a_0, status_a2:=physics.solver_add(&solver_a, &pair_a_0, &description);
	testing.expect_value(t, status_a2, physics.Physics_Status.Ok);
	candidate_a_1, status_a3:=physics.solver_add(&solver_a, &pair_a_1, &description);
	testing.expect_value(t, status_a3, physics.Physics_Status.Ok);
	blocker_b_0, status_b0:=physics.solver_add(&solver_b, &pair_b_0, &description);
	testing.expect_value(t, status_b0, physics.Physics_Status.Ok);
	blocker_b_1, status_b1:=physics.solver_add(&solver_b, &pair_b_1, &description);
	testing.expect_value(t, status_b1, physics.Physics_Status.Ok);
	candidate_b_0, status_b2:=physics.solver_add(&solver_b, &pair_b_0, &description);
	testing.expect_value(t, status_b2, physics.Physics_Status.Ok);
	candidate_b_1, status_b3:=physics.solver_add(&solver_b, &pair_b_1, &description);
	testing.expect_value(t, status_b3, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver_a, blocker_a_0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver_a, blocker_a_1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver_b, blocker_b_0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver_b, blocker_b_1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_move_constraint(&solver_b, candidate_b_0, 2), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_move_constraint(&solver_b, candidate_b_0, 1), physics.Physics_Status.Ok);
	solver_a.compression_batch_index=1;
	solver_a.compression_type_batch_index=0;
	solver_b.compression_batch_index=1;
	solver_b.compression_type_batch_index=0;
	testing.expect_value(t, physics.solver_compress(&solver_a, boundary), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_compress(&solver_b, boundary), physics.Physics_Status.Ok);
	testing.expect(t, dispatch_state.maximum_worker_count>1);
	testing.expect_value(t, dispatch_state.visited[1], physics.Reference_State.Present);
	location_a_0, resolve_a0:=physics.solver_resolve(&solver_a, candidate_a_0);
	testing.expect_value(t, resolve_a0, physics.Physics_Status.Ok);
	location_a_1, resolve_a1:=physics.solver_resolve(&solver_a, candidate_a_1);
	testing.expect_value(t, resolve_a1, physics.Physics_Status.Ok);
	location_b_0, resolve_b0:=physics.solver_resolve(&solver_b, candidate_b_0);
	testing.expect_value(t, resolve_b0, physics.Physics_Status.Ok);
	location_b_1, resolve_b1:=physics.solver_resolve(&solver_b, candidate_b_1);
	testing.expect_value(t, resolve_b1, physics.Physics_Status.Ok);
	testing.expect(t, location_a_0.batch_index>=0&&location_a_0.batch_index<=1);
	testing.expect(t, location_a_1.batch_index>=0&&location_a_1.batch_index<=1);
	testing.expect(t, location_b_0.batch_index>=0&&location_b_0.batch_index<=1);
	testing.expect(t, location_b_1.batch_index>=0&&location_b_1.batch_index<=1);
	testing.expect(t, location_a_0.batch_index!=location_a_1.batch_index);
	testing.expect(t, location_b_0.batch_index!=location_b_1.batch_index);

	fallback_bodies:physics.Bodies;
	fallback_solver:physics.Solver;
	fallback_handles:=prepare_solver(t, &pool, &fallback_bodies, &fallback_solver, 1);
	defer physics.solver_dispose(&fallback_solver);
	defer physics.bodies_dispose(&fallback_bodies);
	fallback_pair:=[4]physics.Body_Handle{fallback_handles[0], fallback_handles[1], {}, {}};
	fallback_constraints:[3001]physics.Constraint_Handle;
	for index in 0..<len(fallback_constraints)
	{
		fallback_status:physics.Physics_Status;
		fallback_constraints[index], fallback_status=physics.solver_add(&fallback_solver, &fallback_pair, &description);
		testing.expect_value(t, fallback_status, physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.solver_remove(&fallback_solver, fallback_constraints[0]), physics.Physics_Status.Ok);
	fallback_solver.compression_batch_index=1;
	fallback_solver.compression_type_batch_index=0;
	testing.expect_value(t, physics.solver_compress(&fallback_solver, boundary), physics.Physics_Status.Ok);
	testing.expect_value(t, fallback_solver.active_set.batches.memory[0].constraint_count, i32(1));
}

@(test)
solver_storage_reuses_preallocated_capacity_without_general_allocations :: proc(t: ^testing.T)
{
	pool:util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	bodies:physics.Bodies;
	solver:physics.Solver;
	handles:=prepare_solver(t, &pool, &bodies, &solver, 3);
	defer physics.solver_dispose(&solver);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(
		t,
		physics.solver_ensure_type_capacity(&solver, physics.BALL_SOCKET_TYPE_ID, 8),
		physics.Physics_Status.Ok
	);
	description:=physics.Ball_Socket{{}, {}, valid_spring()};
	retained_constraint, retained_status:=physics.solver_add(&solver, &handles, &description);
	testing.expect_value(t, retained_status, physics.Physics_Status.Ok);
	initial_body_capacity:=bodies.handle_to_location.length;
	post_growth_handle:=physics.body_handle_invalid();
	for index in 0..=initial_body_capacity
	{
		body_description:=dynamic_description(int(index)+4);
		body_status:physics.Physics_Status;
		post_growth_handle, body_status=physics.bodies_add(&bodies, &body_description);
		testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	}
	testing.expect(t, post_growth_handle.value>=i32(initial_body_capacity));
	for batch_index in 0..<solver.active_set.batches.length
	{
		testing.expect(
			t,
			solver.active_set.batches.memory[batch_index].body_reference_counts.length>=bodies.handle_to_location.length
		);
	}
	testing.expect_value(t, solver.active_set.batches.memory[0].body_reference_counts.memory[handles[0].value], i32(1));
	post_growth_pair:=[4]physics.Body_Handle{handles[0], post_growth_handle, {}, {}};
	post_growth_constraint, post_growth_status:=physics.solver_add(&solver, &post_growth_pair, &description);
	testing.expect_value(t, post_growth_status, physics.Physics_Status.Ok);
	post_growth_constraint_location, location_status:=physics.solver_resolve(&solver, post_growth_constraint);
	testing.expect_value(t, location_status, physics.Physics_Status.Ok);
	testing.expect(t, post_growth_constraint_location.batch_index<solver.fallback_batch_threshold);
	testing.expect_value(t, physics.solver_move_constraint(&solver, post_growth_constraint, 3), physics.Physics_Status.Ok);
	post_growth_location, _:=physics.bodies_resolve(&bodies, post_growth_handle);
	testing.expect_value(
		t,
		solver.sequential_batch.dynamic_body_constraint_counts.memory[post_growth_location.index],
		i32(1)
	);
	testing.expect_value(t, physics.solver_remove(&solver, post_growth_constraint), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_remove(&solver, retained_constraint), physics.Physics_Status.Ok);
	tracker:=(^mem.Tracking_Allocator)(context.allocator.data);
	before:=tracker.total_allocation_count;
	for _ in 0..<1024
	{
		handle, status:=physics.solver_add(&solver, &handles, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.solver_compress(&solver), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.solver_remove(&solver, handle), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, tracker.total_allocation_count, before);
}
