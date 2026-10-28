package entasis_physics

import "core:simd"
import util "entasis:entasis_utilities"

Contact_Warmstart_Application :: enum u8
{
	Applied,
	Sequential_Required,
}

contact_response_bias :: #force_inline proc "contextless" (
	depth, position_error_to_velocity, maximum_recovery_velocity: util.F32x8,
	inverse_dt: f32,
) -> util.F32x8
{
	return simd.min(
		simd.mul(depth, util.F32x8(inverse_dt)),
		simd.min(simd.mul(depth, position_error_to_velocity), maximum_recovery_velocity),
	);
}

contact_apply_velocity_change :: #force_inline proc "contextless" (
	body: ^$Body,
	linear_impulse, angular_velocity_change: util.Vector3_Wide,
	active_mask: util.I32x8,
)
{
	body.linear_velocity = constraint_kernel_select_vector3(
		active_mask,
		util.vector3_wide_add(
			body.linear_velocity,
			util.vector3_wide_scale(linear_impulse, body.inverse_mass),
		),
		body.linear_velocity,
	);
	body.angular_velocity = constraint_kernel_select_vector3(
		active_mask,
		util.vector3_wide_add(body.angular_velocity, angular_velocity_change),
		body.angular_velocity,
	);
}

constraint_kernel_velocity_nonfinite_mask :: #force_inline proc "contextless" (
	linear_velocity, angular_velocity: util.Vector3_Wide,
) -> util.I32x8
{
	// infinities and NaNs have every exponent bit set
	exponent_mask: util.I32x8 = util.I32x8(0x7f800000);
	return transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)linear_velocity.x & exponent_mask, exponent_mask) |
		transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)linear_velocity.y & exponent_mask, exponent_mask) |
		transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)linear_velocity.z & exponent_mask, exponent_mask) |
		transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)angular_velocity.x & exponent_mask, exponent_mask) |
		transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)angular_velocity.y & exponent_mask, exponent_mask) |
		transmute(util.I32x8)simd.lanes_eq(transmute(util.I32x8)angular_velocity.z & exponent_mask, exponent_mask);
}

// all active velocity proposals must qualify before either body is published
contact_response_commit_velocities :: #force_inline proc "contextless" (
	bodies, proposed_bodies: [^]$Body,
	active_mask: util.I32x8,
	$body_count: int,
) -> Reference_State
{
	nonfinite_mask: util.I32x8;
	for body_index: int = 0; body_index < body_count; body_index += 1
	{
		nonfinite_mask |= constraint_kernel_velocity_nonfinite_mask(
			proposed_bodies[body_index].linear_velocity,
			proposed_bodies[body_index].angular_velocity,
		);
	}
	if solver_mask_has_lanes(active_mask & nonfinite_mask) == .Present
	{
		return .Missing;
	}

	for body_index: int = 0; body_index < body_count; body_index += 1
	{
		bodies[body_index].linear_velocity = proposed_bodies[body_index].linear_velocity;
		bodies[body_index].angular_velocity = proposed_bodies[body_index].angular_velocity;
	}
	return .Present;
}

contact_convex_warmstart_fast :: #force_inline proc "contextless" (
	bodies: [^]$Body,
	contacts: [^]Convex_Contact_Wide,
	normal, offset_b: util.Vector3_Wide,
	tangent_x, tangent_y, friction_center_a: util.Vector3_Wide,
	tangent_impulse: util.Vector2_Wide,
	normal_impulses: [^]util.F32x8,
	twist_impulse: util.F32x8,
	active_mask: util.I32x8,
	$body_count, $contact_count: int,
) -> Contact_Warmstart_Application
{
	// carried warmstart impulses are fixed, so accumulate the normal impulse
	// and its first moment before applying one wrench per body
	total_normal_impulse: util.F32x8 = normal_impulses[0];
	normal_impulse_moment: util.Vector3_Wide = util.vector3_wide_scale(contacts[0].offset_a, normal_impulses[0]);
	for contact_index: int = 1; contact_index < contact_count; contact_index += 1
	{
		total_normal_impulse = simd.add(total_normal_impulse, normal_impulses[contact_index]);
		normal_impulse_moment = util.vector3_wide_add(
			normal_impulse_moment,
			util.vector3_wide_scale(contacts[contact_index].offset_a, normal_impulses[contact_index]),
		);
	}

	world_tangent_impulse: util.Vector3_Wide = util.vector3_wide_add(
		util.vector3_wide_scale(tangent_x, tangent_impulse.x),
		util.vector3_wide_scale(tangent_y, tangent_impulse.y),
	);
	linear_impulse: util.Vector3_Wide = util.vector3_wide_add(
		world_tangent_impulse,
		util.vector3_wide_scale(normal, total_normal_impulse),
	);
	angular_impulse_a: util.Vector3_Wide = util.vector3_wide_add(
		util.vector3_wide_cross(friction_center_a, world_tangent_impulse),
		util.vector3_wide_add(
			util.vector3_wide_cross(normal_impulse_moment, normal),
			util.vector3_wide_scale(normal, twist_impulse),
		),
	);

	// keep updates local until both active velocity proposals are finite
	proposed_body_a: Body = bodies[0];
	constraint_kernel_apply_world_impulses(&proposed_body_a, linear_impulse, angular_impulse_a, active_mask);
	nonfinite_mask: util.I32x8 = constraint_kernel_velocity_nonfinite_mask(
		proposed_body_a.linear_velocity,
		proposed_body_a.angular_velocity,
	);
	proposed_body_b: Body;
	_ = proposed_body_b;
	when body_count == 2
	{
		angular_impulse_b: util.Vector3_Wide = util.vector3_wide_subtract(
			util.vector3_wide_cross(offset_b, linear_impulse),
			angular_impulse_a,
		);
		proposed_body_b = bodies[1];
		constraint_kernel_apply_world_impulses(
			&proposed_body_b,
			util.vector3_wide_negate(linear_impulse),
			angular_impulse_b,
			active_mask,
		);
		nonfinite_mask |= constraint_kernel_velocity_nonfinite_mask(
			proposed_body_b.linear_velocity,
			proposed_body_b.angular_velocity,
		);
	}
	if solver_mask_has_lanes(active_mask & nonfinite_mask) == .Present
	{
		return .Sequential_Required;
	}

	bodies[0].linear_velocity = proposed_body_a.linear_velocity;
	bodies[0].angular_velocity = proposed_body_a.angular_velocity;
	when body_count == 2
	{
		bodies[1].linear_velocity = proposed_body_b.linear_velocity;
		bodies[1].angular_velocity = proposed_body_b.angular_velocity;
	}
	return .Applied;
}
