#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Position_Bundle :: physics.Global_Position_Bundle;
	Global_Pose_Velocity_Proc :: physics.Pose_Integrator_Global_Velocity_Proc;

	// caller-owned, native-frame inverse-square gravity. center and rate fields are
	// immutable while stepping. after an origin shift, the caller translates center
	// by the same shift before the next step, the world does not own callback data
	// the existing one-meter denominator floor is retained. double radial arithmetic
	// avoids overflow of squared/cubed distances across the admitted position range
	Global_Planetary_Gravity_Policy :: struct
	{
		center: Global_Position,
		gravity: f32,
		linear_damping, angular_damping: f32,
		gravity_dt: f64,
		linear_damping_dt, angular_damping_dt: util.F32x8,
	}

	planetary_gravity_policy_global :: proc "contextless" (
		center: Global_Position = {}, gravity: f32 = 100000,
		linear_damping: f32 = 0, angular_damping: f32 = 0,
	) -> Global_Planetary_Gravity_Policy
	{
		return {center=center, gravity=gravity, linear_damping=linear_damping, angular_damping=angular_damping};
	}

	planetary_gravity_global_initialize :: proc "contextless" (
		user_context: rawptr, simulation: ^physics.Simulation,
	) -> Status
	{
		_ = simulation;
		if user_context == nil
		{
			return .Invalid_Argument;
		}
		policy: ^Global_Planetary_Gravity_Policy = (^Global_Planetary_Gravity_Policy)(user_context);
		if physics.world_precision_position_validate(policy.center) != .Ok ||
			!(policy.gravity >= 0 && policy.gravity <= f32(math.F32_MAX)) ||
			!(policy.linear_damping >= 0 && policy.linear_damping <= 1) ||
			!(policy.angular_damping >= 0 && policy.angular_damping <= 1)
		{
			return .Invalid_Description;
		}
		return .Ok;
	}

	planetary_gravity_global_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> Status
	{
		if user_context == nil || !(dt > 0 && dt <= f32(math.F32_MAX))
		{
			return .Invalid_Argument;
		}
		policy: ^Global_Planetary_Gravity_Policy = (^Global_Planetary_Gravity_Policy)(user_context);
		policy.gravity_dt = f64(policy.gravity) * f64(dt);
		policy.linear_damping_dt = util.F32x8(f32(math.pow(f64(1 - policy.linear_damping), f64(dt))));
		policy.angular_damping_dt = util.F32x8(f32(math.pow(f64(1 - policy.angular_damping), f64(dt))));
		return .Ok;
	}

	planetary_gravity_global_delta :: proc "contextless" (
		position, center: Global_Position, gravity_dt: f64,
	) -> Vector3
	{
		x: f64 = position.x - center.x;
		y: f64 = position.y - center.y;
		z: f64 = position.z - center.z;
		distance_squared: f64 = x * x + y * y + z * z;
		scale: f64 = -gravity_dt / max(f64(1), distance_squared * math.sqrt(distance_squared));
		return {f32(x * scale), f32(y * scale), f32(z * scale)};
	}

	planetary_gravity_global_undamped :: proc "contextless" (
		user_context: rawptr, body_indices: util.I32x8,
		position: ^Global_Position_Bundle, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
		integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
	)
	{
		_, _, _, _, _ = body_indices, orientation, inertia, worker_index, dt;
		policy: ^Global_Planetary_Gravity_Policy = (^Global_Planetary_Gravity_Policy)(user_context);
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if simd.extract(integration_mask, lane) == 0
			{
				continue;
			}
			delta: Vector3 = planetary_gravity_global_delta(position[lane], policy.center, policy.gravity_dt);
			linear: Vector3 = util.vector3_wide_read_slot(velocity.linear, lane);
			util.vector3_wide_write_slot(&velocity.linear, lane, util.vector3_add(linear, delta));
		}
	}

	planetary_gravity_global_velocity :: proc "contextless" (
		user_context: rawptr, body_indices: util.I32x8,
		position: ^Global_Position_Bundle, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
		integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
	)
	{
		policy: ^Global_Planetary_Gravity_Policy = (^Global_Planetary_Gravity_Policy)(user_context);
		prior: physics.Body_Velocity_Wide = velocity^;
		planetary_gravity_global_undamped(user_context, body_indices, position, orientation,
			inertia, integration_mask, worker_index, dt, velocity);
		velocity.linear = util.vector3_wide_select(integration_mask,
			util.vector3_wide_scale(velocity.linear, policy.linear_damping_dt), prior.linear);
		velocity.angular = util.vector3_wide_select(integration_mask,
			util.vector3_wide_scale(velocity.angular, policy.angular_damping_dt), prior.angular);
	}

	planetary_gravity_global_dispose :: proc "contextless" (user_context: rawptr)
	{
		_ = user_context;
	}

	pose_policy_planetary_global :: proc "contextless" (policy: ^Global_Planetary_Gravity_Policy) -> Pose_Callbacks
	{
		return {
			initialize=planetary_gravity_global_initialize,
			prepare_for_integration=planetary_gravity_global_prepare,
			integrate_velocity_global=planetary_gravity_global_velocity,
			dispose=planetary_gravity_global_dispose,
			user_context=policy,
		};
	}
}
