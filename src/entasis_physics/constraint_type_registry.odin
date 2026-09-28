// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

BUILT_IN_CONSTRAINT_TYPE_COUNT :: 44;
CONSTRAINT_TYPE_ID_CAPACITY :: 64;
FIRST_CALLER_CONSTRAINT_TYPE_ID :: 56;
CONSTRAINT_DESCRIPTION_STORAGE_BYTES :: 256;
Constraint_Registry_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

Constraint_Description_Transfer_Proc :: #type proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
);
Constraint_Description_Build_Proc :: #type proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
) -> Physics_Status;
Constraint_Description_Validate_Proc :: #type proc "contextless" (
	type_id: i32, description: rawptr,
) -> Physics_Status;
Constraint_Storage_Move_Proc :: #type proc "contextless" (
	source_prestep, source_impulses, target_prestep, target_impulses: rawptr,
	prestep_bundle_size, impulse_bundle_size, source_lane, target_lane: int,
);
Constraint_Storage_Remove_Proc :: #type proc "contextless" (
	prestep, impulses: rawptr, prestep_bundle_size, impulse_bundle_size, lane: int,
);
Constraint_Type_Record :: struct
{
	type_id:                 i32,
	body_count:              i32,
	description_size:        i32,
	prestep_bundle_size:     i32,
	impulse_bundle_size:     i32,
	initial_access:          [4]Body_Access_Mask,
	solve_access:            [4]Body_Access_Mask,
	apply_description:       Constraint_Description_Transfer_Proc,
	build_description:       Constraint_Description_Build_Proc,
	using validation: struct #raw_union
	{
		validate_description: Constraint_Description_Validate_Proc,
		contextual: ^Contextual_Constraint_Binding,
	},
	prestep_warmstart_solve: Constraint_Kernel_Proc,
	incrementally_update:    Constraint_Kernel_Proc,
	move_record:             Constraint_Storage_Move_Proc,
	remove_record:           Constraint_Storage_Remove_Proc,
	registration:            Reference_State,
	dispatch:                Constraint_Callback_Dispatch,
}

Constraint_Type_Registry :: struct
{
	records:          [CONSTRAINT_TYPE_ID_CAPACITY]Constraint_Type_Record,
	registered_count: i32,
	state:            Constraint_Registry_State,
}

constraint_description_nonconvex_shape :: proc "contextless" (type_id: int) -> (body_count, contact_count: int)
{
	switch type_id
	{
		case CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID:
		return 1, 2;
		case CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID:
		return 1, 3;
		case CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID:
		return 1, 4;
		case CONTACT_2_NONCONVEX_TYPE_ID:
		return 2, 2;
		case CONTACT_3_NONCONVEX_TYPE_ID:
		return 2, 3;
		case CONTACT_4_NONCONVEX_TYPE_ID:
		return 2, 4;
		case:
		return 0, 0;
	}
}

constraint_description_apply_nonconvex :: proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
)
{
	body_count, contact_count := constraint_description_nonconvex_shape(type_id);
	_, _ = description_size, prestep_bundle_size;
	source_scalars := ([^]f32)(source);
	target_scalars := ([^]f32)(target);
	source_material := 0;
	source_contacts := 4;
	target_offset := 4;
	target_contacts := 4;
	if body_count == 2
	{
		source_material = 3;
		source_contacts = 7;
		target_contacts = 7;
		for component in 0 ..< 3
		{
			target_scalars[(target_offset + component) * util.PRODUCTION_LANE_COUNT + lane] =
			source_scalars[component];
		}
	}
	for field in 0 ..< 4
	{
		target_scalars[field * util.PRODUCTION_LANE_COUNT + lane] =
		source_scalars[source_material + field];
	}
	for contact_index in 0 ..< contact_count
	{
		source_base := source_contacts + contact_index * 7;
		target_base := target_contacts + contact_index * 7;
		for component in 0 ..< 3
		{
			target_scalars[(target_base + component) * util.PRODUCTION_LANE_COUNT + lane] =
			source_scalars[source_base + component];
		}
		target_scalars[(target_base + 3) * util.PRODUCTION_LANE_COUNT + lane] =
		source_scalars[source_base + 6];
		for component in 0 ..< 3
		{
			target_scalars[(target_base + 4 + component) * util.PRODUCTION_LANE_COUNT + lane] =
			source_scalars[source_base + 3 + component];
		}
	}
}

constraint_description_build_nonconvex :: proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
) -> Physics_Status
{
	body_count, contact_count := constraint_description_nonconvex_shape(type_id);
	if body_count == 0
	{
		return .Invalid_Argument;
	}
	expected_scalar_count := 4 + contact_count * 7;
	if body_count == 2
	{
		expected_scalar_count += 3;
	}
	if description_size != expected_scalar_count * size_of(f32) ||
	prestep_bundle_size != expected_scalar_count * size_of(util.F32x8)
	{
		return .Invalid_Argument;
	}
	source_scalars := ([^]f32)(source);
	target_scalars := ([^]f32)(target);
	target_material := 0;
	target_contacts := 4;
	source_offset := 4;
	source_contacts := 4;
	if body_count == 2
	{
		target_material = 3;
		target_contacts = 7;
		source_contacts = 7;
		for component in 0 ..< 3
		{
			target_scalars[component] =
			source_scalars[(source_offset + component) * util.PRODUCTION_LANE_COUNT + lane];
		}
	}
	for field in 0 ..< 4
	{
		target_scalars[target_material + field] =
		source_scalars[field * util.PRODUCTION_LANE_COUNT + lane];
	}
	for contact_index in 0 ..< contact_count
	{
		target_base := target_contacts + contact_index * 7;
		source_base := source_contacts + contact_index * 7;
		for component in 0 ..< 3
		{
			target_scalars[target_base + component] =
			source_scalars[(source_base + component) * util.PRODUCTION_LANE_COUNT + lane];
		}
		for component in 0 ..< 3
		{
			target_scalars[target_base + 3 + component] =
			source_scalars[(source_base + 4 + component) * util.PRODUCTION_LANE_COUNT + lane];
		}
		target_scalars[target_base + 6] =
		source_scalars[(source_base + 3) * util.PRODUCTION_LANE_COUNT + lane];
	}
	return .Ok;
}

constraint_description_transfer :: proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
)
{
	body_count, _ := constraint_description_nonconvex_shape(type_id);
	if body_count != 0
	{
		constraint_description_apply_nonconvex(source, target, type_id, description_size, prestep_bundle_size, lane);
		return;
	}
	source_scalars := ([^]f32)(source);
	target_scalars := ([^]f32)(target);
	for field in 0 ..< description_size / size_of(f32)
	{
		target_scalars[field * util.PRODUCTION_LANE_COUNT + lane] = source_scalars[field];
	}
}

constraint_description_build :: proc "contextless" (
	source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
) -> Physics_Status
{
	if source == nil || target == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT ||
	description_size <= 0 ||
	description_size > CONSTRAINT_DESCRIPTION_STORAGE_BYTES ||
	description_size % size_of(f32) != 0
	{
		return .Invalid_Argument;
	}
	if type_id == BALL_SOCKET_TYPE_ID
	{
		if description_size != size_of(Ball_Socket) || prestep_bundle_size != size_of(Ball_Socket_Prestep)
		{
			return .Invalid_Argument;
		}
	}
	else if prestep_bundle_size != description_size / size_of(f32) * size_of(util.F32x8)
	{
		return .Invalid_Argument;
	}
	body_count, _ := constraint_description_nonconvex_shape(type_id);
	if body_count != 0
	{
		return constraint_description_build_nonconvex(
			source,
			target,
			type_id,
			description_size,
			prestep_bundle_size,
			lane
		);
	}
	source_scalars := ([^]f32)(source);
	target_scalars := ([^]f32)(target);
	for field in 0 ..< description_size / size_of(f32)
	{
		target_scalars[field] = source_scalars[field * util.PRODUCTION_LANE_COUNT + lane];
	}
	return .Ok;
}

constraint_storage_move_lane :: proc "contextless" (
	source_prestep, source_impulses, target_prestep, target_impulses: rawptr,
	prestep_bundle_size, impulse_bundle_size, source_lane, target_lane: int,
)
{
	source_prestep_scalars := ([^]f32)(source_prestep);
	target_prestep_scalars := ([^]f32)(target_prestep);
	for field in 0 ..< prestep_bundle_size / size_of(util.F32x8)
	{
		target_prestep_scalars[field * util.PRODUCTION_LANE_COUNT + target_lane] =
		source_prestep_scalars[field * util.PRODUCTION_LANE_COUNT + source_lane];
	}
	source_impulse_scalars := ([^]f32)(source_impulses);
	target_impulse_scalars := ([^]f32)(target_impulses);
	for field in 0 ..< impulse_bundle_size / size_of(util.F32x8)
	{
		target_impulse_scalars[field * util.PRODUCTION_LANE_COUNT + target_lane] =
		source_impulse_scalars[field * util.PRODUCTION_LANE_COUNT + source_lane];
	}
}

constraint_storage_remove_lane :: proc "contextless" (
	prestep, impulses: rawptr, prestep_bundle_size, impulse_bundle_size, lane: int,
)
{
	prestep_scalars := ([^]f32)(prestep);
	for field in 0 ..< prestep_bundle_size / size_of(util.F32x8)
	{
		prestep_scalars[field * util.PRODUCTION_LANE_COUNT + lane] = 0;
	}
	impulse_scalars := ([^]f32)(impulses);
	for field in 0 ..< impulse_bundle_size / size_of(util.F32x8)
	{
		impulse_scalars[field * util.PRODUCTION_LANE_COUNT + lane] = 0;
	}
}

constraint_registry_validate_description :: proc "contextless" (type_id: i32, description: rawptr) -> Physics_Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	switch type_id
	{
		case CONTACT_1_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_1_One_Body)(description));
		case CONTACT_2_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_2_One_Body)(description));
		case CONTACT_3_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_3_One_Body)(description));
		case CONTACT_4_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_4_One_Body)(description));
		case CONTACT_1_TYPE_ID:
		return constraint_description_validate((^Contact_1)(description));
		case CONTACT_2_TYPE_ID:
		return constraint_description_validate((^Contact_2)(description));
		case CONTACT_3_TYPE_ID:
		return constraint_description_validate((^Contact_3)(description));
		case CONTACT_4_TYPE_ID:
		return constraint_description_validate((^Contact_4)(description));
		case CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_2_Nonconvex_One_Body)(description));
		case CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_3_Nonconvex_One_Body)(description));
		case CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID:
		return constraint_description_validate((^Contact_4_Nonconvex_One_Body)(description));
		case CONTACT_2_NONCONVEX_TYPE_ID:
		return constraint_description_validate((^Contact_2_Nonconvex)(description));
		case CONTACT_3_NONCONVEX_TYPE_ID:
		return constraint_description_validate((^Contact_3_Nonconvex)(description));
		case CONTACT_4_NONCONVEX_TYPE_ID:
		return constraint_description_validate((^Contact_4_Nonconvex)(description));
		case BALL_SOCKET_TYPE_ID:
		return constraint_description_validate((^Ball_Socket)(description));
		case ANGULAR_HINGE_TYPE_ID:
		return constraint_description_validate((^Angular_Hinge)(description));
		case ANGULAR_SWIVEL_HINGE_TYPE_ID:
		return constraint_description_validate((^Angular_Swivel_Hinge)(description));
		case SWING_LIMIT_TYPE_ID:
		return constraint_description_validate((^Swing_Limit)(description));
		case TWIST_SERVO_TYPE_ID:
		return constraint_description_validate((^Twist_Servo)(description));
		case TWIST_LIMIT_TYPE_ID:
		return constraint_description_validate((^Twist_Limit)(description));
		case TWIST_MOTOR_TYPE_ID:
		return constraint_description_validate((^Twist_Motor)(description));
		case ANGULAR_SERVO_TYPE_ID:
		return constraint_description_validate((^Angular_Servo)(description));
		case ANGULAR_MOTOR_TYPE_ID:
		return constraint_description_validate((^Angular_Motor)(description));
		case WELD_TYPE_ID:
		return constraint_description_validate((^Weld)(description));
		case VOLUME_CONSTRAINT_TYPE_ID:
		return constraint_description_validate((^Volume_Constraint)(description));
		case DISTANCE_SERVO_TYPE_ID:
		return constraint_description_validate((^Distance_Servo)(description));
		case DISTANCE_LIMIT_TYPE_ID:
		return constraint_description_validate((^Distance_Limit)(description));
		case CENTER_DISTANCE_CONSTRAINT_TYPE_ID:
		return constraint_description_validate((^Center_Distance_Constraint)(description));
		case AREA_CONSTRAINT_TYPE_ID:
		return constraint_description_validate((^Area_Constraint)(description));
		case POINT_ON_LINE_SERVO_TYPE_ID:
		return constraint_description_validate((^Point_On_Line_Servo)(description));
		case LINEAR_AXIS_SERVO_TYPE_ID:
		return constraint_description_validate((^Linear_Axis_Servo)(description));
		case LINEAR_AXIS_MOTOR_TYPE_ID:
		return constraint_description_validate((^Linear_Axis_Motor)(description));
		case LINEAR_AXIS_LIMIT_TYPE_ID:
		return constraint_description_validate((^Linear_Axis_Limit)(description));
		case ANGULAR_AXIS_MOTOR_TYPE_ID:
		return constraint_description_validate((^Angular_Axis_Motor)(description));
		case ONE_BODY_ANGULAR_SERVO_TYPE_ID:
		return constraint_description_validate((^One_Body_Angular_Servo)(description));
		case ONE_BODY_ANGULAR_MOTOR_TYPE_ID:
		return constraint_description_validate((^One_Body_Angular_Motor)(description));
		case ONE_BODY_LINEAR_SERVO_TYPE_ID:
		return constraint_description_validate((^One_Body_Linear_Servo)(description));
		case ONE_BODY_LINEAR_MOTOR_TYPE_ID:
		return constraint_description_validate((^One_Body_Linear_Motor)(description));
		case SWIVEL_HINGE_TYPE_ID:
		return constraint_description_validate((^Swivel_Hinge)(description));
		case HINGE_TYPE_ID:
		return constraint_description_validate((^Hinge)(description));
		case BALL_SOCKET_MOTOR_TYPE_ID:
		return constraint_description_validate((^Ball_Socket_Motor)(description));
		case BALL_SOCKET_SERVO_TYPE_ID:
		return constraint_description_validate((^Ball_Socket_Servo)(description));
		case ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID:
		return constraint_description_validate((^Angular_Axis_Gear_Motor)(description));
		case CENTER_DISTANCE_LIMIT_TYPE_ID:
		return constraint_description_validate((^Center_Distance_Limit)(description));
		case:
		return .Invalid_Argument;
	}
}

constraint_type_registry_register :: proc "contextless" (
	registry: ^Constraint_Type_Registry, record: Constraint_Type_Record,
) -> Physics_Status
{
	if registry == nil ||
	registry.state != .Ready ||
	record.type_id < 0 ||
	record.type_id >= CONSTRAINT_TYPE_ID_CAPACITY ||
	record.body_count < 1 || record.body_count > 4 || record.description_size <= 0 ||
	record.description_size > CONSTRAINT_DESCRIPTION_STORAGE_BYTES || record.prestep_bundle_size <= 0 ||
	record.impulse_bundle_size <= 0 || record.apply_description == nil || record.build_description == nil ||
	constraint_type_callbacks_status(record) != .Ok ||
	record.move_record == nil || record.remove_record == nil
	{
		return .Invalid_Argument;
	}
	if registry.records[record.type_id].registration == .Present
	{
		return .Invalid_Argument;
	}
	registry.records[record.type_id] = record;
	registry.records[record.type_id].registration = .Present;
	registry.registered_count += 1;
	return .Ok;
}

constraint_type_layout :: proc "contextless" ($T: typeid) -> (prestep_bundle_size, impulse_bundle_size: int)
{
	when T == Contact_1_One_Body
	{
		return size_of(Contact_1_One_Body_Prestep), size_of(Contact_1_Accumulated_Impulses);
	}
	else when T == Contact_2_One_Body
	{
		return size_of(Contact_2_One_Body_Prestep), size_of(Contact_2_Accumulated_Impulses);
	}
	else when T == Contact_3_One_Body
	{
		return size_of(Contact_3_One_Body_Prestep), size_of(Contact_3_Accumulated_Impulses);
	}
	else when T == Contact_4_One_Body
	{
		return size_of(Contact_4_One_Body_Prestep), size_of(Contact_4_Accumulated_Impulses);
	}
	else when T == Contact_1
	{
		return size_of(Contact_1_Prestep), size_of(Contact_1_Accumulated_Impulses);
	}
	else when T == Contact_2
	{
		return size_of(Contact_2_Prestep), size_of(Contact_2_Accumulated_Impulses);
	}
	else when T == Contact_3
	{
		return size_of(Contact_3_Prestep), size_of(Contact_3_Accumulated_Impulses);
	}
	else when T == Contact_4
	{
		return size_of(Contact_4_Prestep), size_of(Contact_4_Accumulated_Impulses);
	}
	else when T == Contact_2_Nonconvex_One_Body || T == Contact_2_Nonconvex
	{
		when T == Contact_2_Nonconvex_One_Body
		{
			prestep_bundle_size = size_of(Contact_2_Nonconvex_One_Body_Prestep);
		}
		else
		{
			prestep_bundle_size = size_of(Contact_2_Nonconvex_Prestep);
		}
		return prestep_bundle_size, size_of(Contact_2_Nonconvex_Accumulated_Impulses);
	}
	else when T == Contact_3_Nonconvex_One_Body || T == Contact_3_Nonconvex
	{
		when T == Contact_3_Nonconvex_One_Body
		{
			prestep_bundle_size = size_of(Contact_3_Nonconvex_One_Body_Prestep);
		}
		else
		{
			prestep_bundle_size = size_of(Contact_3_Nonconvex_Prestep);
		}
		return prestep_bundle_size, size_of(Contact_3_Nonconvex_Accumulated_Impulses);
	}
	else when T == Contact_4_Nonconvex_One_Body || T == Contact_4_Nonconvex
	{
		when T == Contact_4_Nonconvex_One_Body
		{
			prestep_bundle_size = size_of(Contact_4_Nonconvex_One_Body_Prestep);
		}
		else
		{
			prestep_bundle_size = size_of(Contact_4_Nonconvex_Prestep);
		}
		return prestep_bundle_size, size_of(Contact_4_Nonconvex_Accumulated_Impulses);
	}
	else when T == Angular_Axis_Gear_Motor
	{
		return size_of(Angular_Axis_Gear_Motor_Prestep), size_of(util.F32x8);
	}
	else when T == Angular_Axis_Motor
	{
		return size_of(Angular_Axis_Motor_Prestep), size_of(util.F32x8);
	}
	else when T == Angular_Hinge
	{
		return size_of(Angular_Hinge_Prestep), size_of(util.Vector2_Wide);
	}
	else when T == Angular_Motor
	{
		return size_of(Angular_Motor_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Angular_Servo
	{
		return size_of(Angular_Servo_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Angular_Swivel_Hinge
	{
		return size_of(Angular_Swivel_Hinge_Prestep), size_of(util.F32x8);
	}
	else when T == Area_Constraint
	{
		return size_of(Area_Constraint_Prestep), size_of(util.F32x8);
	}
	else when T == Ball_Socket
	{
		return size_of(Ball_Socket_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Ball_Socket_Motor
	{
		return size_of(Ball_Socket_Motor_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Ball_Socket_Servo
	{
		return size_of(Ball_Socket_Servo_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Center_Distance_Constraint
	{
		return size_of(Center_Distance_Constraint_Prestep), size_of(util.F32x8);
	}
	else when T == Center_Distance_Limit
	{
		return size_of(Center_Distance_Limit_Prestep), size_of(util.F32x8);
	}
	else when T == Distance_Limit
	{
		return size_of(Distance_Limit_Prestep), size_of(util.F32x8);
	}
	else when T == Distance_Servo
	{
		return size_of(Distance_Servo_Prestep), size_of(util.F32x8);
	}
	else when T == Hinge
	{
		return size_of(Hinge_Prestep), size_of(Hinge_Accumulated_Impulses);
	}
	else when T == Linear_Axis_Limit
	{
		return size_of(Linear_Axis_Limit_Prestep), size_of(util.F32x8);
	}
	else when T == Linear_Axis_Motor
	{
		return size_of(Linear_Axis_Motor_Prestep), size_of(util.F32x8);
	}
	else when T == Linear_Axis_Servo
	{
		return size_of(Linear_Axis_Servo_Prestep), size_of(util.F32x8);
	}
	else when T == One_Body_Angular_Motor
	{
		return size_of(One_Body_Angular_Motor_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == One_Body_Angular_Servo
	{
		return size_of(One_Body_Angular_Servo_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == One_Body_Linear_Motor
	{
		return size_of(One_Body_Linear_Motor_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == One_Body_Linear_Servo
	{
		return size_of(One_Body_Linear_Servo_Prestep), size_of(util.Vector3_Wide);
	}
	else when T == Point_On_Line_Servo
	{
		return size_of(Point_On_Line_Servo_Prestep), size_of(util.Vector2_Wide);
	}
	else when T == Swing_Limit
	{
		return size_of(Swing_Limit_Prestep), size_of(util.F32x8);
	}
	else when T == Swivel_Hinge
	{
		return size_of(Swivel_Hinge_Prestep), size_of(util.Vector4_Wide);
	}
	else when T == Twist_Limit
	{
		return size_of(Twist_Limit_Prestep), size_of(util.F32x8);
	}
	else when T == Twist_Motor
	{
		return size_of(Twist_Motor_Prestep), size_of(util.F32x8);
	}
	else when T == Twist_Servo
	{
		return size_of(Twist_Servo_Prestep), size_of(util.F32x8);
	}
	else when T == Volume_Constraint
	{
		return size_of(Volume_Constraint_Prestep), size_of(util.F32x8);
	}
	else when T == Weld
	{
		return size_of(Weld_Prestep), size_of(Weld_Accumulated_Impulses);
	}
	return 0, 0;
}

constraint_type_registry_add_builtin :: proc "contextless" (
	registry: ^Constraint_Type_Registry, $T: typeid, kernel: Constraint_Kernel_Proc,
	initial_access, solve_access: [4]Body_Access_Mask,
) -> Physics_Status
{
	prestep_bundle_size, impulse_bundle_size := constraint_type_layout(T);
	return constraint_type_registry_register(registry, {
			type_id=constraint_description_type_id(T),
			body_count=constraint_description_body_count(T),
			description_size=size_of(T),
			prestep_bundle_size=i32(prestep_bundle_size),
			impulse_bundle_size=i32(impulse_bundle_size),
			initial_access=initial_access,
			solve_access=solve_access,
			apply_description=constraint_description_transfer,
			build_description=constraint_description_build,
			validate_description=constraint_registry_validate_description,
			prestep_warmstart_solve=kernel,
			incrementally_update=kernel,
			move_record=constraint_storage_move_lane,
			remove_record=constraint_storage_remove_lane,
	});
}

constraint_type_registry_add_contact_builtins :: proc "contextless" (registry: ^Constraint_Type_Registry) -> Physics_Status
{
	no_pose := [4]Body_Access_Mask{BODY_ACCESS_NO_POSE, BODY_ACCESS_NO_POSE, {}, {}};
	if constraint_type_registry_add_builtin(
		registry,
		Contact_1_One_Body,
		contact_1_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_2_One_Body,
		contact_2_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_3_One_Body,
		contact_3_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_4_One_Body,
		contact_4_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Contact_1, contact_1_kernel, no_pose, no_pose) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Contact_2, contact_2_kernel, no_pose, no_pose) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Contact_3, contact_3_kernel, no_pose, no_pose) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Contact_4, contact_4_kernel, no_pose, no_pose) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_2_Nonconvex_One_Body,
		contact_2_nonconvex_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_3_Nonconvex_One_Body,
		contact_3_nonconvex_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_4_Nonconvex_One_Body,
		contact_4_nonconvex_one_body_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_2_Nonconvex,
		contact_2_nonconvex_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_3_Nonconvex,
		contact_3_nonconvex_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Contact_4_Nonconvex,
		contact_4_nonconvex_kernel,
		no_pose,
		no_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

constraint_type_registry_initialize :: proc "contextless" (registry: ^Constraint_Type_Registry) -> Physics_Status
{
	if registry == nil || registry.state != .Uninitialized
	{
		return .Invalid_Argument;
	}
	registry^ = {};
	registry.state = .Ready;
	if constraint_type_registry_add_contact_builtins(registry) != .Ok
	{
		registry^ = {};
		return .Invalid_Argument;
	}
	// each noncontact call stays explicit so the compiler creates the matching layout specialization
	all := [4]Body_Access_Mask{BODY_ACCESS_ALL, BODY_ACCESS_ALL, BODY_ACCESS_ALL, BODY_ACCESS_ALL};
	angular := [4]Body_Access_Mask{BODY_ACCESS_ONLY_ANGULAR, BODY_ACCESS_ONLY_ANGULAR, {}, {}};
	angular_without_pose := [4]Body_Access_Mask{
		BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE,
		BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE,
		{},
		{},
	};
	linear := [4]Body_Access_Mask{
		BODY_ACCESS_ONLY_LINEAR,
		BODY_ACCESS_ONLY_LINEAR,
		BODY_ACCESS_ONLY_LINEAR,
		BODY_ACCESS_ONLY_LINEAR,
	};
	no_position := [4]Body_Access_Mask{BODY_ACCESS_NO_POSITION, BODY_ACCESS_NO_POSITION, {}, {}};
	angular_a_pose := [4]Body_Access_Mask{BODY_ACCESS_ONLY_ANGULAR, BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE, {}, {}};
	weld_initial := [4]Body_Access_Mask{BODY_ACCESS_NO_POSITION, BODY_ACCESS_NO_POSE, {}, {}};
	ball_socket_motor_initial := [4]Body_Access_Mask{BODY_ACCESS_NO_ORIENTATION, BODY_ACCESS_ALL, {}, {}};
	if constraint_type_registry_add_builtin(registry, Ball_Socket, ball_socket_kernel, no_position, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Hinge,
		angular_hinge_kernel,
		angular_a_pose,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Swivel_Hinge,
		angular_swivel_hinge_kernel,
		angular,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Swing_Limit, swing_limit_kernel, angular, angular) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Twist_Servo, twist_servo_kernel, angular, angular) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Twist_Limit, twist_limit_kernel, angular, angular) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Twist_Motor, twist_motor_kernel, angular, angular) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Servo,
		angular_servo_kernel,
		angular_without_pose,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Motor,
		angular_motor_kernel,
		angular_without_pose,
		angular_a_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Weld, weld_kernel, weld_initial, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Volume_Constraint,
		volume_constraint_kernel,
		linear,
		linear
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Distance_Servo, distance_servo_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Distance_Limit, distance_limit_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Center_Distance_Constraint,
		center_distance_constraint_kernel,
		linear,
		linear
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Area_Constraint, area_constraint_kernel, linear, linear) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Point_On_Line_Servo, point_on_line_servo_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Linear_Axis_Servo, linear_axis_servo_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Linear_Axis_Motor, linear_axis_motor_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Linear_Axis_Limit, linear_axis_limit_kernel, all, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Axis_Motor,
		angular_axis_motor_kernel,
		angular_a_pose,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		One_Body_Angular_Servo,
		one_body_angular_servo_kernel,
		angular,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		One_Body_Angular_Motor,
		one_body_angular_motor_kernel,
		angular_without_pose,
		angular
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		One_Body_Linear_Servo,
		one_body_linear_servo_kernel,
		all,
		all
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		One_Body_Linear_Motor,
		one_body_linear_motor_kernel,
		no_position,
		no_position
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Swivel_Hinge, swivel_hinge_kernel, no_position, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(registry, Hinge, hinge_kernel, no_position, all) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Ball_Socket_Motor,
		ball_socket_motor_kernel,
		ball_socket_motor_initial,
		all
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Ball_Socket_Servo,
		ball_socket_servo_kernel,
		no_position,
		all
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Angular_Axis_Gear_Motor,
		angular_axis_gear_motor_kernel,
		angular_a_pose,
		angular_a_pose
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if constraint_type_registry_add_builtin(
		registry,
		Center_Distance_Limit,
		center_distance_limit_kernel,
		linear,
		linear
	) != .Ok
	{
		return .Invalid_Argument;
	}
	if registry.registered_count != BUILT_IN_CONSTRAINT_TYPE_COUNT
	{
		registry^ = {};
		return .Invalid_Argument;
	}
	return .Ok;
}

constraint_type_registry_lookup :: proc "contextless" (
	registry: ^Constraint_Type_Registry, type_id: i32,
) -> (^Constraint_Type_Record, Physics_Status)
{
	if registry == nil || registry.state != .Ready || type_id < 0 || type_id >= CONSTRAINT_TYPE_ID_CAPACITY
	{
		return nil, .Not_Found;
	}
	record := &registry.records[type_id];
	if record.registration == .Missing
	{
		return nil, .Not_Found;
	}
	return record, .Ok;
}

constraint_type_registry_dispose :: proc "contextless" (registry: ^Constraint_Type_Registry) -> Physics_Status
{
	if registry == nil || registry.state != .Ready
	{
		return .Disposed;
	}
	registry^ = {};
	registry.state = .Disposed;
	return .Ok;
}
#assert(size_of(Contact_4) <= CONSTRAINT_DESCRIPTION_STORAGE_BYTES);
#assert(size_of(Contact_4_Nonconvex) <= CONSTRAINT_DESCRIPTION_STORAGE_BYTES);
#assert(size_of(Weld) <= CONSTRAINT_DESCRIPTION_STORAGE_BYTES);
