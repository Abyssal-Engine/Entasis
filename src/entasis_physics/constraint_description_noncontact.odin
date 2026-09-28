// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

BALL_SOCKET_TYPE_ID :: 22;
ANGULAR_HINGE_TYPE_ID :: 23;
ANGULAR_SWIVEL_HINGE_TYPE_ID :: 24;
SWING_LIMIT_TYPE_ID :: 25;
TWIST_SERVO_TYPE_ID :: 26;
TWIST_LIMIT_TYPE_ID :: 27;
TWIST_MOTOR_TYPE_ID :: 28;
ANGULAR_SERVO_TYPE_ID :: 29;
ANGULAR_MOTOR_TYPE_ID :: 30;
WELD_TYPE_ID :: 31;
VOLUME_CONSTRAINT_TYPE_ID :: 32;
DISTANCE_SERVO_TYPE_ID :: 33;
DISTANCE_LIMIT_TYPE_ID :: 34;
CENTER_DISTANCE_CONSTRAINT_TYPE_ID :: 35;
AREA_CONSTRAINT_TYPE_ID :: 36;
POINT_ON_LINE_SERVO_TYPE_ID :: 37;
LINEAR_AXIS_SERVO_TYPE_ID :: 38;
LINEAR_AXIS_MOTOR_TYPE_ID :: 39;
LINEAR_AXIS_LIMIT_TYPE_ID :: 40;
ANGULAR_AXIS_MOTOR_TYPE_ID :: 41;
ONE_BODY_ANGULAR_SERVO_TYPE_ID :: 42;
ONE_BODY_ANGULAR_MOTOR_TYPE_ID :: 43;
ONE_BODY_LINEAR_SERVO_TYPE_ID :: 44;
ONE_BODY_LINEAR_MOTOR_TYPE_ID :: 45;
SWIVEL_HINGE_TYPE_ID :: 46;
HINGE_TYPE_ID :: 47;
BALL_SOCKET_MOTOR_TYPE_ID :: 52;
BALL_SOCKET_SERVO_TYPE_ID :: 53;
ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID :: 54;
CENTER_DISTANCE_LIMIT_TYPE_ID :: 55;

Angular_Axis_Gear_Motor :: struct
{
	local_axis_a:   util.Vector3,
	velocity_scale: f32,
	settings:       Motor_Settings,
}

Angular_Axis_Motor :: struct
{
	local_axis_a:   util.Vector3,
	target_velocity: f32,
	settings:       Motor_Settings,
}

Angular_Hinge :: struct
{
	local_hinge_axis_a: util.Vector3,
	local_hinge_axis_b: util.Vector3,
	spring_settings:    Spring_Settings,
}

Angular_Motor :: struct
{
	target_velocity_local_a: util.Vector3,
	settings:                Motor_Settings,
}

Angular_Servo :: struct
{
	target_relative_rotation_local_a: util.Quaternion,
	spring_settings:                  Spring_Settings,
	servo_settings:                   Servo_Settings,
}

Angular_Swivel_Hinge :: struct
{
	local_swivel_axis_a: util.Vector3,
	local_hinge_axis_b:  util.Vector3,
	spring_settings:     Spring_Settings,
}

Area_Constraint :: struct
{
	target_scaled_area: f32,
	spring_settings:    Spring_Settings,
}

Ball_Socket :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	spring_settings: Spring_Settings,
}

Ball_Socket_Motor :: struct
{
	local_offset_b:          util.Vector3,
	target_velocity_local_a: util.Vector3,
	settings:                Motor_Settings,
}

Ball_Socket_Servo :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	spring_settings: Spring_Settings,
	servo_settings:  Servo_Settings,
}

Center_Distance_Constraint :: struct
{
	target_distance: f32,
	spring_settings: Spring_Settings,
}

Center_Distance_Limit :: struct
{
	minimum_distance: f32,
	maximum_distance: f32,
	spring_settings:  Spring_Settings,
}

Distance_Limit :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	minimum_distance: f32,
	maximum_distance: f32,
	spring_settings: Spring_Settings,
}

Distance_Servo :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	target_distance: f32,
	servo_settings:  Servo_Settings,
	spring_settings: Spring_Settings,
}

Hinge :: struct
{
	local_offset_a:     util.Vector3,
	local_hinge_axis_a: util.Vector3,
	local_offset_b:     util.Vector3,
	local_hinge_axis_b: util.Vector3,
	spring_settings:    Spring_Settings,
}

Linear_Axis_Limit :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	local_axis:      util.Vector3,
	minimum_offset:  f32,
	maximum_offset:  f32,
	spring_settings: Spring_Settings,
}

Linear_Axis_Motor :: struct
{
	local_offset_a:   util.Vector3,
	local_offset_b:   util.Vector3,
	local_axis:       util.Vector3,
	target_velocity:  f32,
	settings:         Motor_Settings,
}

Linear_Axis_Servo :: struct
{
	local_offset_a:     util.Vector3,
	local_offset_b:     util.Vector3,
	local_plane_normal: util.Vector3,
	target_offset:      f32,
	servo_settings:     Servo_Settings,
	spring_settings:    Spring_Settings,
}

One_Body_Angular_Motor :: struct
{
	target_velocity: util.Vector3,
	settings:        Motor_Settings,
}

One_Body_Angular_Servo :: struct
{
	target_orientation: util.Quaternion,
	spring_settings:    Spring_Settings,
	servo_settings:     Servo_Settings,
}

One_Body_Linear_Motor :: struct
{
	local_offset:    util.Vector3,
	target_velocity: util.Vector3,
	settings:        Motor_Settings,
}

One_Body_Linear_Servo :: struct
{
	local_offset:    util.Vector3,
	target:          util.Vector3,
	spring_settings: Spring_Settings,
	servo_settings:  Servo_Settings,
}

Point_On_Line_Servo :: struct
{
	local_offset_a:  util.Vector3,
	local_offset_b:  util.Vector3,
	local_direction: util.Vector3,
	servo_settings:  Servo_Settings,
	spring_settings: Spring_Settings,
}

Swing_Limit :: struct
{
	axis_local_a:    util.Vector3,
	axis_local_b:    util.Vector3,
	minimum_dot:     f32,
	spring_settings: Spring_Settings,
}

Swivel_Hinge :: struct
{
	local_offset_a:      util.Vector3,
	local_swivel_axis_a: util.Vector3,
	local_offset_b:      util.Vector3,
	local_hinge_axis_b:  util.Vector3,
	spring_settings:     Spring_Settings,
}

Twist_Limit :: struct
{
	local_basis_a:   util.Quaternion,
	local_basis_b:   util.Quaternion,
	minimum_angle:   f32,
	maximum_angle:   f32,
	spring_settings: Spring_Settings,
}

Twist_Motor :: struct
{
	local_axis_a:   util.Vector3,
	local_axis_b:   util.Vector3,
	target_velocity: f32,
	settings:       Motor_Settings,
}

Twist_Servo :: struct
{
	local_basis_a:   util.Quaternion,
	local_basis_b:   util.Quaternion,
	target_angle:    f32,
	spring_settings: Spring_Settings,
	servo_settings:  Servo_Settings,
}

Volume_Constraint :: struct
{
	target_scaled_volume: f32,
	spring_settings:      Spring_Settings,
}

Weld :: struct
{
	local_offset:      util.Vector3,
	local_orientation: util.Quaternion,
	spring_settings:   Spring_Settings,
}
