// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Spring_Settings :: struct
{
	angular_frequency:   f32,
	twice_damping_ratio: f32,
}

Servo_Settings :: struct
{
	maximum_speed: f32,
	base_speed:    f32,
	maximum_force: f32,
}

Motor_Settings :: struct
{
	maximum_force: f32,
	damping:       f32,
}

Spring_Settings_Wide :: struct
{
	angular_frequency:   util.F32x8,
	twice_damping_ratio: util.F32x8,
}

spring_settings_create :: #force_inline proc "contextless" (
	frequency, damping_ratio: f32,
) -> Spring_Settings
{
	return {
		angular_frequency=frequency * f32(2 * math.PI),
		twice_damping_ratio=damping_ratio * 2,
	};
}

Servo_Settings_Wide :: struct
{
	maximum_speed: util.F32x8,
	base_speed:    util.F32x8,
	maximum_force: util.F32x8,
}

Motor_Settings_Wide :: struct
{
	maximum_force: util.F32x8,
	damping:       util.F32x8,
}

BODY_ACCESS_NO_POSE :: Body_Access_Mask{.Mass, .Inertia_Tensor, .Linear_Velocity, .Angular_Velocity};
BODY_ACCESS_NO_POSITION :: Body_Access_Mask{.Orientation, .Mass, .Inertia_Tensor, .Linear_Velocity, .Angular_Velocity};
BODY_ACCESS_NO_ORIENTATION :: Body_Access_Mask{.Position, .Mass, .Inertia_Tensor, .Linear_Velocity, .Angular_Velocity};
BODY_ACCESS_ONLY_VELOCITY :: Body_Access_Mask{.Linear_Velocity, .Angular_Velocity};
BODY_ACCESS_ONLY_ANGULAR :: Body_Access_Mask{.Orientation, .Inertia_Tensor, .Angular_Velocity};
BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE :: Body_Access_Mask{.Inertia_Tensor, .Angular_Velocity};
BODY_ACCESS_ONLY_LINEAR :: Body_Access_Mask{.Position, .Mass, .Linear_Velocity};

Batch_Integration_Mode :: enum u8
{
	Always,
	Never,
	Conditional,
}

constraint_positive_finite :: proc "contextless" (value: f32) -> Reference_State
{
	if value > 0 && value <= f32(math.F32_MAX)
	{
		return .Present;
	}
	return .Missing;
}

constraint_nonnegative_finite :: proc "contextless" (value: f32) -> Reference_State
{
	if value >= 0 && value <= f32(math.F32_MAX)
	{
		return .Present;
	}
	return .Missing;
}

constraint_finite :: proc "contextless" (value: f32) -> Reference_State
{
	if value >= -f32(math.F32_MAX) && value <= f32(math.F32_MAX)
	{
		return .Present;
	}
	return .Missing;
}

constraint_vector3_finite :: proc "contextless" (value: util.Vector3) -> Reference_State
{
	if constraint_finite(value.x) == .Present && constraint_finite(value.y) == .Present &&
		constraint_finite(value.z) == .Present
	{
		return .Present;
	}
	return .Missing;
}

constraint_unit_vector3 :: proc "contextless" (value: util.Vector3) -> Reference_State
{
	if constraint_vector3_finite(value) == .Missing
	{
		return .Missing;
	}
	if math.abs(util.vector3_length_squared(value) - 1) <= 1e-5
	{
		return .Present;
	}
	return .Missing;
}

constraint_unit_quaternion :: proc "contextless" (value: util.Quaternion) -> Reference_State
{
	if constraint_finite(value.x) == .Missing || constraint_finite(value.y) == .Missing ||
		constraint_finite(value.z) == .Missing || constraint_finite(value.w) == .Missing
	{
		return .Missing;
	}
	if math.abs(util.quaternion_length_squared(value) - 1) <= 1e-5
	{
		return .Present;
	}
	return .Missing;
}

spring_settings_validate :: proc "contextless" (settings: Spring_Settings) -> Physics_Status
{
	if constraint_positive_finite(settings.angular_frequency) == .Missing ||
		constraint_nonnegative_finite(settings.twice_damping_ratio) == .Missing
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

servo_settings_validate :: proc "contextless" (settings: Servo_Settings) -> Physics_Status
{
	if constraint_nonnegative_finite(settings.maximum_speed) == .Missing ||
		constraint_nonnegative_finite(settings.base_speed) == .Missing ||
		constraint_nonnegative_finite(settings.maximum_force) == .Missing
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

motor_settings_validate :: proc "contextless" (settings: Motor_Settings) -> Physics_Status
{
	if constraint_nonnegative_finite(settings.maximum_force) == .Missing ||
		constraint_nonnegative_finite(settings.damping) == .Missing
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

spring_settings_wide_compute :: #force_inline proc "contextless" (
	settings: Spring_Settings_Wide, dt: f32,
) -> (position_error_to_velocity, effective_mass_cfm_scale, softness_impulse_scale: util.F32x8)
{
	angular_frequency_dt := simd.mul(settings.angular_frequency, util.F32x8(dt));
	denominator := simd.add(angular_frequency_dt, settings.twice_damping_ratio);
	position_error_to_velocity = simd.div(settings.angular_frequency, denominator);
	extra := simd.div(util.F32x8(1), simd.mul(angular_frequency_dt, denominator));
	effective_mass_cfm_scale = simd.div(util.F32x8(1), simd.add(util.F32x8(1), extra));
	softness_impulse_scale = simd.mul(extra, effective_mass_cfm_scale);
	return;
}

servo_settings_wide_clamp_impulse :: #force_inline proc "contextless" (
	maximum_impulse: util.F32x8, accumulated_impulse: ^util.F32x8, csi: ^util.F32x8,
)
{
	previous := accumulated_impulse^;
	accumulated_impulse^ = simd.max(simd.neg(maximum_impulse), simd.min(maximum_impulse, simd.add(previous, csi^)));
	csi^ = simd.sub(accumulated_impulse^, previous);
}

servo_settings_wide_compute_bias_scalar :: #force_inline proc "contextless" (
	error, position_error_to_velocity: util.F32x8, settings: Servo_Settings_Wide, dt, inverse_dt: f32,
) -> (bias_velocity, maximum_impulse: util.F32x8)
{
	base_speed := simd.min(settings.base_speed, simd.mul(simd.abs(error), util.F32x8(inverse_dt)));
	unclamped := simd.mul(error, position_error_to_velocity);
	negative := transmute(util.I32x8)simd.lanes_lt(unclamped, util.F32x8(0));
	negative_bias := simd.max(simd.neg(settings.maximum_speed), simd.min(simd.neg(base_speed), unclamped));
	positive_bias := simd.min(settings.maximum_speed, simd.max(base_speed, unclamped));
	bias_velocity = util.wide_select_f32(negative, negative_bias, positive_bias);
	maximum_impulse = simd.mul(settings.maximum_force, util.F32x8(dt));
	return;
}

servo_settings_wide_compute_bias_vector2 :: #force_inline proc "contextless" (
	error: util.Vector2_Wide, position_error_to_velocity: util.F32x8,
	settings: Servo_Settings_Wide, dt, inverse_dt: f32,
) -> (bias_velocity: util.Vector2_Wide, maximum_impulse: util.F32x8)
{
	length := util.vector2_wide_length(error);
	axis := util.vector2_wide_scale(error, simd.div(util.F32x8(1), length));
	use_zero_axis := transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-10));
	axis = util.vector2_wide_select(use_zero_axis, {}, axis);
	base_speed := simd.min(settings.base_speed, simd.mul(length, util.F32x8(inverse_dt)));
	unclamped_speed := simd.mul(length, position_error_to_velocity);
	target_speed := simd.max(base_speed, unclamped_speed);
	scale := simd.min(util.F32x8(1), simd.div(settings.maximum_speed, target_speed));
	use_unscaled_speed := transmute(util.I32x8)simd.lanes_lt(target_speed, util.F32x8(1e-10));
	scale = util.wide_select_f32(use_unscaled_speed, util.F32x8(1), scale);
	bias_velocity = util.vector2_wide_scale(axis, simd.mul(scale, unclamped_speed));
	maximum_impulse = simd.mul(settings.maximum_force, util.F32x8(dt));
	return;
}

servo_settings_wide_compute_bias_vector3 :: #force_inline proc "contextless" (
	error: util.Vector3_Wide, position_error_to_velocity: util.F32x8,
	settings: Servo_Settings_Wide, dt, inverse_dt: f32,
) -> (bias_velocity: util.Vector3_Wide, maximum_impulse: util.F32x8)
{
	length := util.vector3_wide_length(error);
	axis := util.vector3_wide_scale(error, simd.div(util.F32x8(1), length));
	use_zero_axis := transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-10));
	axis = util.vector3_wide_select(use_zero_axis, {}, axis);
	base_speed := simd.min(settings.base_speed, simd.mul(length, util.F32x8(inverse_dt)));
	unclamped_speed := simd.mul(length, position_error_to_velocity);
	target_speed := simd.max(base_speed, unclamped_speed);
	scale := simd.min(util.F32x8(1), simd.div(settings.maximum_speed, target_speed));
	use_unscaled_speed := transmute(util.I32x8)simd.lanes_lt(target_speed, util.F32x8(1e-10));
	scale = util.wide_select_f32(use_unscaled_speed, util.F32x8(1), scale);
	bias_velocity = util.vector3_wide_scale(axis, simd.mul(scale, unclamped_speed));
	maximum_impulse = simd.mul(settings.maximum_force, util.F32x8(dt));
	return;
}

servo_settings_wide_clamp_impulse_vector2 :: #force_inline proc "contextless" (
	maximum_impulse: util.F32x8, accumulated_impulse, csi: ^util.Vector2_Wide,
)
{
	previous := accumulated_impulse^;
	unclamped := util.vector2_wide_add(previous, csi^);
	magnitude := util.vector2_wide_length(unclamped);
	scale := simd.min(util.F32x8(1), simd.div(maximum_impulse, magnitude));
	use_unscaled_impulse := transmute(util.I32x8)simd.lanes_lt(simd.abs(magnitude), util.F32x8(1e-10));
	scale = util.wide_select_f32(use_unscaled_impulse, util.F32x8(1), scale);
	accumulated_impulse^ = util.vector2_wide_scale(unclamped, scale);
	csi^ = util.vector2_wide_subtract(accumulated_impulse^, previous);
}

servo_settings_wide_clamp_impulse_vector3 :: #force_inline proc "contextless" (
	maximum_impulse: util.F32x8, accumulated_impulse, csi: ^util.Vector3_Wide,
)
{
	previous := accumulated_impulse^;
	unclamped := util.vector3_wide_add(previous, csi^);
	magnitude := util.vector3_wide_length(unclamped);
	scale := simd.min(util.F32x8(1), simd.div(maximum_impulse, magnitude));
	use_unscaled_impulse := transmute(util.I32x8)simd.lanes_lt(simd.abs(magnitude), util.F32x8(1e-10));
	scale = util.wide_select_f32(use_unscaled_impulse, util.F32x8(1), scale);
	accumulated_impulse^ = util.vector3_wide_scale(unclamped, scale);
	csi^ = util.vector3_wide_subtract(accumulated_impulse^, previous);
}

constraint_inequality_bias_velocity :: #force_inline proc "contextless" (
	error, position_error_to_velocity: util.F32x8, inverse_dt: f32,
) -> util.F32x8
{
	return simd.min(simd.mul(error, position_error_to_velocity), simd.mul(error, util.F32x8(inverse_dt)));
}

constraint_clamp_positive_impulse :: #force_inline proc "contextless" (
	accumulated_impulse, csi: ^util.F32x8,
)
{
	previous := accumulated_impulse^;
	accumulated_impulse^ = simd.max(util.F32x8(0), simd.add(previous, csi^));
	csi^ = simd.sub(accumulated_impulse^, previous);
}

motor_settings_wide_compute :: #force_inline proc "contextless" (
	settings: Motor_Settings_Wide, dt: f32,
) -> (effective_mass_cfm_scale, softness_impulse_scale, maximum_impulse: util.F32x8)
{
	damping_dt := simd.mul(settings.damping, util.F32x8(dt));
	effective_mass_cfm_scale = simd.div(damping_dt, simd.add(damping_dt, util.F32x8(1)));
	softness_impulse_scale = simd.sub(util.F32x8(1), effective_mass_cfm_scale);
	maximum_impulse = simd.mul(settings.maximum_force, util.F32x8(dt));
	return;
}
