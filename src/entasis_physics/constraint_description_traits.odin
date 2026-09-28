// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

constraint_description_type_id :: proc "contextless" ($T: typeid) -> i32
{
	when T == Contact_1_One_Body
	{
		return CONTACT_1_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_2_One_Body
	{
		return CONTACT_2_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_3_One_Body
	{
		return CONTACT_3_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_4_One_Body
	{
		return CONTACT_4_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_1
	{
		return CONTACT_1_TYPE_ID;
	}
	else when T == Contact_2
	{
		return CONTACT_2_TYPE_ID;
	}
	else when T == Contact_3
	{
		return CONTACT_3_TYPE_ID;
	}
	else when T == Contact_4
	{
		return CONTACT_4_TYPE_ID;
	}
	else when T == Contact_2_Nonconvex_One_Body
	{
		return CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_3_Nonconvex_One_Body
	{
		return CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_4_Nonconvex_One_Body
	{
		return CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID;
	}
	else when T == Contact_2_Nonconvex
	{
		return CONTACT_2_NONCONVEX_TYPE_ID;
	}
	else when T == Contact_3_Nonconvex
	{
		return CONTACT_3_NONCONVEX_TYPE_ID;
	}
	else when T == Contact_4_Nonconvex
	{
		return CONTACT_4_NONCONVEX_TYPE_ID;
	}
	else when T == Ball_Socket
	{
		return BALL_SOCKET_TYPE_ID;
	}
	else when T == Angular_Hinge
	{
		return ANGULAR_HINGE_TYPE_ID;
	}
	else when T == Angular_Swivel_Hinge
	{
		return ANGULAR_SWIVEL_HINGE_TYPE_ID;
	}
	else when T == Swing_Limit
	{
		return SWING_LIMIT_TYPE_ID;
	}
	else when T == Twist_Servo
	{
		return TWIST_SERVO_TYPE_ID;
	}
	else when T == Twist_Limit
	{
		return TWIST_LIMIT_TYPE_ID;
	}
	else when T == Twist_Motor
	{
		return TWIST_MOTOR_TYPE_ID;
	}
	else when T == Angular_Servo
	{
		return ANGULAR_SERVO_TYPE_ID;
	}
	else when T == Angular_Motor
	{
		return ANGULAR_MOTOR_TYPE_ID;
	}
	else when T == Weld
	{
		return WELD_TYPE_ID;
	}
	else when T == Volume_Constraint
	{
		return VOLUME_CONSTRAINT_TYPE_ID;
	}
	else when T == Distance_Servo
	{
		return DISTANCE_SERVO_TYPE_ID;
	}
	else when T == Distance_Limit
	{
		return DISTANCE_LIMIT_TYPE_ID;
	}
	else when T == Center_Distance_Constraint
	{
		return CENTER_DISTANCE_CONSTRAINT_TYPE_ID;
	}
	else when T == Area_Constraint
	{
		return AREA_CONSTRAINT_TYPE_ID;
	}
	else when T == Point_On_Line_Servo
	{
		return POINT_ON_LINE_SERVO_TYPE_ID;
	}
	else when T == Linear_Axis_Servo
	{
		return LINEAR_AXIS_SERVO_TYPE_ID;
	}
	else when T == Linear_Axis_Motor
	{
		return LINEAR_AXIS_MOTOR_TYPE_ID;
	}
	else when T == Linear_Axis_Limit
	{
		return LINEAR_AXIS_LIMIT_TYPE_ID;
	}
	else when T == Angular_Axis_Motor
	{
		return ANGULAR_AXIS_MOTOR_TYPE_ID;
	}
	else when T == One_Body_Angular_Servo
	{
		return ONE_BODY_ANGULAR_SERVO_TYPE_ID;
	}
	else when T == One_Body_Angular_Motor
	{
		return ONE_BODY_ANGULAR_MOTOR_TYPE_ID;
	}
	else when T == One_Body_Linear_Servo
	{
		return ONE_BODY_LINEAR_SERVO_TYPE_ID;
	}
	else when T == One_Body_Linear_Motor
	{
		return ONE_BODY_LINEAR_MOTOR_TYPE_ID;
	}
	else when T == Swivel_Hinge
	{
		return SWIVEL_HINGE_TYPE_ID;
	}
	else when T == Hinge
	{
		return HINGE_TYPE_ID;
	}
	else when T == Ball_Socket_Motor
	{
		return BALL_SOCKET_MOTOR_TYPE_ID;
	}
	else when T == Ball_Socket_Servo
	{
		return BALL_SOCKET_SERVO_TYPE_ID;
	}
	else when T == Angular_Axis_Gear_Motor
	{
		return ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID;
	}
	else when T == Center_Distance_Limit
	{
		return CENTER_DISTANCE_LIMIT_TYPE_ID;
	}
	else
	{
		return -1;
	}
}

constraint_description_body_count :: proc "contextless" ($T: typeid) -> i32
{
	when T == Contact_1_One_Body || T == Contact_2_One_Body || T == Contact_3_One_Body ||
		T == Contact_4_One_Body || T == Contact_2_Nonconvex_One_Body ||
		T == Contact_3_Nonconvex_One_Body || T == Contact_4_Nonconvex_One_Body ||
		T == One_Body_Angular_Servo || T == One_Body_Angular_Motor ||
		T == One_Body_Linear_Servo || T == One_Body_Linear_Motor
	{
		return 1;
	}
	else when T == Area_Constraint
	{
		return 3;
	}
	else when T == Volume_Constraint
	{
		return 4;
	}
	else
	{
		return 2;
	}
}

constraint_contact_material_validate :: proc "contextless" (material: Contact_Material_Properties) -> Physics_Status
{
	if constraint_nonnegative_finite(material.friction_coefficient) == .Missing ||
		constraint_nonnegative_finite(material.maximum_recovery_velocity) == .Missing
	{
		return .Invalid_Argument;
	}
	return spring_settings_validate(material.spring_settings);
}

constraint_nonconvex_one_body_material_validate :: proc "contextless" (material: Nonconvex_One_Body_Properties) -> Physics_Status
{
	return constraint_contact_material_validate({
			friction_coefficient=material.friction_coefficient,
			spring_settings=material.spring_settings,
			maximum_recovery_velocity=material.maximum_recovery_velocity,
		});
}

constraint_nonconvex_two_body_material_validate :: proc "contextless" (material: Nonconvex_Two_Body_Properties) -> Physics_Status
{
	if constraint_vector3_finite(material.offset_b) == .Missing
	{
		return .Invalid_Argument;
	}
	return constraint_contact_material_validate({
			friction_coefficient=material.friction_coefficient,
			spring_settings=material.spring_settings,
			maximum_recovery_velocity=material.maximum_recovery_velocity,
		});
}

constraint_convex_contact_validate :: proc "contextless" (contact: Constraint_Contact_Data) -> Physics_Status
{
	if constraint_vector3_finite(contact.offset_a) == .Missing ||
		constraint_finite(contact.penetration_depth) == .Missing
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

constraint_nonconvex_contact_validate :: proc "contextless" (contact: Nonconvex_Constraint_Contact_Data) -> Physics_Status
{
	if constraint_vector3_finite(contact.offset_a) == .Missing || constraint_unit_vector3(contact.normal) == .Missing ||
		constraint_finite(contact.penetration_depth) == .Missing
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

constraint_description_validate :: proc "contextless" (description: ^$T) -> Physics_Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	when T == Angular_Axis_Gear_Motor
	{
		if constraint_unit_vector3(description.local_axis_a) == .Missing ||
			constraint_finite(description.velocity_scale) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Angular_Axis_Motor
	{
		if constraint_unit_vector3(description.local_axis_a) == .Missing ||
			constraint_finite(description.target_velocity) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Angular_Hinge
	{
		if constraint_unit_vector3(description.local_hinge_axis_a) == .Missing ||
			constraint_unit_vector3(description.local_hinge_axis_b) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Angular_Motor
	{
		if constraint_vector3_finite(description.target_velocity_local_a) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Angular_Servo
	{
		if constraint_unit_quaternion(description.target_relative_rotation_local_a) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Angular_Swivel_Hinge
	{
		if constraint_unit_vector3(description.local_swivel_axis_a) == .Missing ||
			constraint_unit_vector3(description.local_hinge_axis_b) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Area_Constraint
	{
		if constraint_nonnegative_finite(description.target_scaled_area) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Ball_Socket
	{
		if constraint_vector3_finite(description.local_offset_a) == .Missing ||
			constraint_vector3_finite(description.local_offset_b) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Ball_Socket_Motor
	{
		if constraint_vector3_finite(description.local_offset_b) == .Missing ||
			constraint_vector3_finite(description.target_velocity_local_a) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Ball_Socket_Servo
	{
		if constraint_vector3_finite(description.local_offset_a) == .Missing ||
			constraint_vector3_finite(description.local_offset_b) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Center_Distance_Constraint
	{
		if constraint_nonnegative_finite(description.target_distance) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Center_Distance_Limit
	{
		if constraint_nonnegative_finite(description.minimum_distance) == .Missing ||
			constraint_nonnegative_finite(description.maximum_distance) == .Missing ||
			description.maximum_distance < description.minimum_distance
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Distance_Limit
	{
		if constraint_nonnegative_finite(description.minimum_distance) == .Missing ||
			constraint_nonnegative_finite(description.maximum_distance) == .Missing ||
			description.maximum_distance < description.minimum_distance
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Distance_Servo
	{
		if constraint_nonnegative_finite(description.target_distance) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Hinge
	{
		if constraint_unit_vector3(description.local_hinge_axis_a) == .Missing ||
			constraint_unit_vector3(description.local_hinge_axis_b) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Linear_Axis_Limit
	{
		if constraint_unit_vector3(description.local_axis) == .Missing ||
			description.maximum_offset < description.minimum_offset
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Linear_Axis_Motor
	{
		if constraint_unit_vector3(description.local_axis) == .Missing ||
			constraint_finite(description.target_velocity) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Linear_Axis_Servo
	{
		if constraint_unit_vector3(description.local_plane_normal) == .Missing ||
			constraint_finite(description.target_offset) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == One_Body_Angular_Motor
	{
		if constraint_vector3_finite(description.target_velocity) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == One_Body_Angular_Servo
	{
		if constraint_unit_quaternion(description.target_orientation) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == One_Body_Linear_Motor
	{
		if constraint_vector3_finite(description.local_offset) == .Missing ||
			constraint_vector3_finite(description.target_velocity) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == One_Body_Linear_Servo
	{
		if constraint_vector3_finite(description.local_offset) == .Missing ||
			constraint_vector3_finite(description.target) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Point_On_Line_Servo
	{
		if constraint_unit_vector3(description.local_direction) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Swing_Limit
	{
		if constraint_unit_vector3(description.axis_local_a) == .Missing ||
			constraint_unit_vector3(description.axis_local_b) == .Missing ||
			description.minimum_dot < -1 ||
			description.minimum_dot > 1
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Swivel_Hinge
	{
		if constraint_unit_vector3(description.local_swivel_axis_a) == .Missing ||
			constraint_unit_vector3(description.local_hinge_axis_b) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Twist_Limit
	{
		if constraint_unit_quaternion(description.local_basis_a) == .Missing ||
			constraint_unit_quaternion(description.local_basis_b) == .Missing ||
			description.maximum_angle < description.minimum_angle
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Twist_Motor
	{
		if constraint_unit_vector3(description.local_axis_a) == .Missing ||
			constraint_unit_vector3(description.local_axis_b) == .Missing ||
			constraint_finite(description.target_velocity) == .Missing
		{
			return .Invalid_Argument;
		}
		return motor_settings_validate(description.settings);
	}
	else when T == Twist_Servo
	{
		if constraint_unit_quaternion(description.local_basis_a) == .Missing ||
			constraint_unit_quaternion(description.local_basis_b) == .Missing ||
			constraint_finite(description.target_angle) == .Missing
		{
			return .Invalid_Argument;
		}
		if spring_settings_validate(description.spring_settings) != .Ok
		{
			return .Invalid_Argument;
		}
		return servo_settings_validate(description.servo_settings);
	}
	else when T == Volume_Constraint
	{
		if constraint_finite(description.target_scaled_volume) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Weld
	{
		if constraint_unit_quaternion(description.local_orientation) == .Missing
		{
			return .Invalid_Argument;
		}
		return spring_settings_validate(description.spring_settings);
	}
	else when T == Contact_1_One_Body ||
		T == Contact_2_One_Body ||
		T == Contact_3_One_Body ||
		T == Contact_4_One_Body ||
		T == Contact_1 || T == Contact_2 || T == Contact_3 || T == Contact_4
	{
		if constraint_unit_vector3(description.normal) == .Missing ||
			constraint_contact_material_validate(description.material) != .Ok
		{
			return .Invalid_Argument;
		}
		when T == Contact_1 || T == Contact_2 || T == Contact_3 || T == Contact_4
		{
			if constraint_vector3_finite(description.offset_b) == .Missing
			{
				return .Invalid_Argument;
			}
		}
		if constraint_convex_contact_validate(description.contact_0) != .Ok
		{
			return .Invalid_Argument;
		}
		when T != Contact_1_One_Body && T != Contact_1
		{
			if constraint_convex_contact_validate(description.contact_1) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		when T == Contact_3_One_Body || T == Contact_4_One_Body || T == Contact_3 || T == Contact_4
		{
			if constraint_convex_contact_validate(description.contact_2) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		when T == Contact_4_One_Body || T == Contact_4
		{
			if constraint_convex_contact_validate(description.contact_3) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		return .Ok;
	}
	else when T == Contact_2_Nonconvex || T == Contact_3_Nonconvex || T == Contact_4_Nonconvex
	{
		if constraint_nonconvex_two_body_material_validate(description.common) != .Ok
		{
			return .Invalid_Argument;
		}
		if constraint_nonconvex_contact_validate(description.contact_0) != .Ok ||
			constraint_nonconvex_contact_validate(description.contact_1) != .Ok
		{
			return .Invalid_Argument;
		}
		when T == Contact_3_Nonconvex || T == Contact_4_Nonconvex
		{
			if constraint_nonconvex_contact_validate(description.contact_2) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		when T == Contact_4_Nonconvex
		{
			if constraint_nonconvex_contact_validate(description.contact_3) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		return .Ok;
	}
	else when T == Contact_2_Nonconvex_One_Body ||
		T == Contact_3_Nonconvex_One_Body ||
		T == Contact_4_Nonconvex_One_Body
	{
		if constraint_nonconvex_one_body_material_validate(description.common) != .Ok
		{
			return .Invalid_Argument;
		}
		if constraint_nonconvex_contact_validate(description.contact_0) != .Ok ||
			constraint_nonconvex_contact_validate(description.contact_1) != .Ok
		{
			return .Invalid_Argument;
		}
		when T == Contact_3_Nonconvex_One_Body || T == Contact_4_Nonconvex_One_Body
		{
			if constraint_nonconvex_contact_validate(description.contact_2) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		when T == Contact_4_Nonconvex_One_Body
		{
			if constraint_nonconvex_contact_validate(description.contact_3) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		return .Ok;
	}
	else
	{
		return .Invalid_Argument;
	}
}
