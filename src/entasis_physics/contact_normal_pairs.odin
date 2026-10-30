package entasis_physics

import "core:simd"
import util "entasis:entasis_utilities"

Contact_Normal_Pair_Mode :: enum u8
{
	Serial,
	Parallel,
}

contact_normal_lcp2_scaled :: #force_inline proc "contextless" (
	a00, a01, a11, b0, b1: util.F32x8,
	active_mask: util.I32x8,
	$pair_mode: Contact_Normal_Pair_Mode,
) -> (x0, x1: util.F32x8, accepted_mask: util.I32x8)
{
	zero: util.F32x8 = util.F32x8(0);
	one: util.F32x8 = util.F32x8(1);
	limit: util.F32x8 = util.F32x8(3.4e38);
	magnitude_mask: util.I32x8 = util.I32x8(0x7fffffff);
	magnitude_a01: util.F32x8 = transmute(util.F32x8)(transmute(util.I32x8)a01 & magnitude_mask);
	magnitude_b0: util.F32x8 = transmute(util.F32x8)(transmute(util.I32x8)b0 & magnitude_mask);
	magnitude_b1: util.F32x8 = transmute(util.F32x8)(transmute(util.I32x8)b1 & magnitude_mask);
	valid_mask: util.I32x8 = active_mask & transmute(util.I32x8)simd.lanes_gt(a00, zero) &
		transmute(util.I32x8)simd.lanes_gt(a11, zero) &
		transmute(util.I32x8)simd.lanes_lt(a00, limit) &
		transmute(util.I32x8)simd.lanes_lt(a11, limit) &
		transmute(util.I32x8)simd.lanes_lt(magnitude_a01, limit) &
		transmute(util.I32x8)simd.lanes_lt(magnitude_b0, limit) &
		transmute(util.I32x8)simd.lanes_lt(magnitude_b1, limit);

	// scale rows by their diagonal. avoid forming a00*a11, which can
	// overflow even when the physical solution is small and finite
	inverse_diagonal0: util.F32x8 = simd.div(one, util.wide_select_f32(valid_mask, a00, one));
	inverse_diagonal1: util.F32x8 = simd.div(one, util.wide_select_f32(valid_mask, a11, one));
	c01: util.F32x8 = simd.mul(util.wide_select_f32(valid_mask, a01, zero), inverse_diagonal0);
	c10: util.F32x8 = simd.mul(util.wide_select_f32(valid_mask, a01, zero), inverse_diagonal1);
	q0: util.F32x8 = simd.mul(util.wide_select_f32(valid_mask, b0, zero), inverse_diagonal0);
	q1: util.F32x8 = simd.mul(util.wide_select_f32(valid_mask, b1, zero), inverse_diagonal1);

	determinant: util.F32x8 = simd.sub(one, simd.mul(c01, c10));
	valid_mask &= transmute(util.I32x8)simd.lanes_gt(determinant, util.F32x8(1e-5));
	both0, both1: util.F32x8;
	both_active_mask: util.I32x8;
	when pair_mode == .Serial
	{
		inverse_determinant: util.F32x8 = simd.div(one, util.wide_select_f32(valid_mask, determinant, one));

		both0 = simd.mul(simd.sub(simd.mul(c01, q1), q0), inverse_determinant);
		both1 = simd.mul(simd.sub(simd.mul(c10, q0), q1), inverse_determinant);
		both_active_mask = valid_mask & transmute(util.I32x8)simd.lanes_ge(both0, zero) &
			transmute(util.I32x8)simd.lanes_ge(both1, zero) &
			transmute(util.I32x8)simd.lanes_lt(both0, limit) &
			transmute(util.I32x8)simd.lanes_lt(both1, limit);
	}
	else
	{
		numerator0: util.F32x8 = simd.sub(simd.mul(c01, q1), q0);
		numerator1: util.F32x8 = simd.sub(simd.mul(c10, q0), q1);
		both_candidate_mask: util.I32x8 = valid_mask & transmute(util.I32x8)simd.lanes_ge(numerator0, zero) &
			transmute(util.I32x8)simd.lanes_ge(numerator1, zero);
		// an admitted determinant is at most one, so scaling cannot make a negative numerator admissible
		if solver_mask_has_lanes(both_candidate_mask) == .Present
		{
			inverse_determinant: util.F32x8 = simd.div(one, util.wide_select_f32(valid_mask, determinant, one));
			both0 = simd.mul(numerator0, inverse_determinant);
			both1 = simd.mul(numerator1, inverse_determinant);
			both_active_mask = valid_mask & transmute(util.I32x8)simd.lanes_ge(both0, zero) &
				transmute(util.I32x8)simd.lanes_ge(both1, zero) &
				transmute(util.I32x8)simd.lanes_lt(both0, limit) &
				transmute(util.I32x8)simd.lanes_lt(both1, limit);
		}
	}

	if solver_mask_has_lanes(active_mask & ~both_active_mask) == .Missing
	{
		x0 = util.wide_select_f32(both_active_mask, both0, zero);
		x1 = util.wide_select_f32(both_active_mask, both1, zero);
		accepted_mask = both_active_mask;
		return;
	}

	only0: util.F32x8 = simd.neg(q0);
	only0_mask: util.I32x8 = valid_mask & ~both_active_mask & transmute(util.I32x8)simd.lanes_ge(only0, zero) &
		transmute(util.I32x8)simd.lanes_ge(simd.add(simd.mul(c10, only0), q1), zero) &
		transmute(util.I32x8)simd.lanes_lt(only0, limit);

	only1: util.F32x8 = simd.neg(q1);
	only1_mask: util.I32x8 = valid_mask & ~(both_active_mask | only0_mask) & transmute(util.I32x8)simd.lanes_ge(only1, zero) &
		transmute(util.I32x8)simd.lanes_ge(simd.add(simd.mul(c01, only1), q0), zero) &
		transmute(util.I32x8)simd.lanes_lt(only1, limit);

	zero_impulse_mask: util.I32x8 = valid_mask & transmute(util.I32x8)simd.lanes_ge(q0, zero) &
		transmute(util.I32x8)simd.lanes_ge(q1, zero);

	accepted_mask = both_active_mask | only0_mask | only1_mask | zero_impulse_mask;
	x0 = util.wide_select_f32(both_active_mask, both0, util.wide_select_f32(only0_mask, only0, zero));
	x1 = util.wide_select_f32(both_active_mask, both1, util.wide_select_f32(only1_mask, only1, zero));
	return;
}

contact_normal_pair_solve_fast :: #force_inline proc "contextless" (
	bodies: [^]$Body,
	normal, offset_b: util.Vector3_Wide,
	contact0, contact1: Convex_Contact_Wide,
	position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity, softness: util.F32x8,
	inverse_dt: f32,
	accumulated0, accumulated1: ^util.F32x8,
	active_mask: util.I32x8,
	$body_count: int,
	$pair_mode: Contact_Normal_Pair_Mode,
)
{
	eligible_mask: util.I32x8 = active_mask;
	angular_jacobian_a0: util.Vector3_Wide = util.vector3_wide_cross(contact0.offset_a, normal);
	angular_jacobian_a1: util.Vector3_Wide = util.vector3_wide_cross(contact1.offset_a, normal);
	angular_response_a0: util.Vector3_Wide = util.symmetric3x3_wide_transform(angular_jacobian_a0, bodies[0].inverse_inertia);
	angular_response_a1: util.Vector3_Wide = util.symmetric3x3_wide_transform(angular_jacobian_a1, bodies[0].inverse_inertia);
	angular_response_b0, angular_response_b1: util.Vector3_Wide;
	_ = angular_response_b0;
	_ = angular_response_b1;
	w00: util.F32x8 = simd.add(bodies[0].inverse_mass, util.vector3_wide_dot(angular_response_a0, angular_jacobian_a0));
	w11: util.F32x8 = simd.add(bodies[0].inverse_mass, util.vector3_wide_dot(angular_response_a1, angular_jacobian_a1));
	w01: util.F32x8 = simd.add(bodies[0].inverse_mass, util.vector3_wide_dot(angular_response_a0, angular_jacobian_a1));
	linear_normal_velocity: util.F32x8 = util.vector3_wide_dot(bodies[0].linear_velocity, normal);
	normal_velocity0: util.F32x8 = simd.add(
		linear_normal_velocity,
		util.vector3_wide_dot(bodies[0].angular_velocity, angular_jacobian_a0),
	);
	normal_velocity1: util.F32x8 = simd.add(
		linear_normal_velocity,
		util.vector3_wide_dot(bodies[0].angular_velocity, angular_jacobian_a1),
	);
	when body_count == 2
	{
		angular_jacobian_b0: util.Vector3_Wide = util.vector3_wide_cross(
			normal, util.vector3_wide_subtract(contact0.offset_a, offset_b),
		);
		angular_jacobian_b1: util.Vector3_Wide = util.vector3_wide_cross(
			normal, util.vector3_wide_subtract(contact1.offset_a, offset_b),
		);
		angular_response_b0 = util.symmetric3x3_wide_transform(angular_jacobian_b0, bodies[1].inverse_inertia);
		angular_response_b1 = util.symmetric3x3_wide_transform(angular_jacobian_b1, bodies[1].inverse_inertia);
		w00 = simd.add(w00, simd.add(bodies[1].inverse_mass, util.vector3_wide_dot(angular_response_b0, angular_jacobian_b0)));
		w11 = simd.add(w11, simd.add(bodies[1].inverse_mass, util.vector3_wide_dot(angular_response_b1, angular_jacobian_b1)));
		w01 = simd.add(w01, simd.add(bodies[1].inverse_mass, util.vector3_wide_dot(angular_response_b0, angular_jacobian_b1)));
		linear_normal_velocity_b: util.F32x8 = util.vector3_wide_dot(bodies[1].linear_velocity, normal);
		normal_velocity0 = simd.add(
			simd.sub(normal_velocity0, linear_normal_velocity_b),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_jacobian_b0),
		);
		normal_velocity1 = simd.add(
			simd.sub(normal_velocity1, linear_normal_velocity_b),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_jacobian_b1),
		);
	}

	bias0: util.F32x8 = contact_response_bias(contact0.depth, position_error_to_velocity, maximum_recovery_velocity, inverse_dt);
	bias1: util.F32x8 = contact_response_bias(contact1.depth, position_error_to_velocity, maximum_recovery_velocity, inverse_dt);
	previous_impulse0: util.F32x8 = accumulated0^;
	previous_impulse1: util.F32x8 = accumulated1^;
	// a = w + diag(softness * w_ii / effective_mass_scale)
	// existing velocity includes w*old, but not the compliance contribution
	diagonal_scale: util.F32x8 = simd.add(util.F32x8(1), simd.div(softness, effective_mass_scale));
	b0: util.F32x8 = simd.sub(
		simd.sub(normal_velocity0, bias0),
		simd.add(simd.mul(w00, previous_impulse0), simd.mul(w01, previous_impulse1)),
	);
	b1: util.F32x8 = simd.sub(
		simd.sub(normal_velocity1, bias1),
		simd.add(simd.mul(w01, previous_impulse0), simd.mul(w11, previous_impulse1)),
	);
	next_impulse0, next_impulse1: util.F32x8;
	accepted_mask: util.I32x8;
	next_impulse0, next_impulse1, accepted_mask = contact_normal_lcp2_scaled(
		simd.mul(w00, diagonal_scale), w01, simd.mul(w11, diagonal_scale), b0, b1, eligible_mask, pair_mode,
	);
	impulse_delta0: util.F32x8 = constraint_kernel_mask_scalar(simd.sub(next_impulse0, previous_impulse0), accepted_mask);
	impulse_delta1: util.F32x8 = constraint_kernel_mask_scalar(simd.sub(next_impulse1, previous_impulse1), accepted_mask);
	linear_impulse: util.Vector3_Wide = util.vector3_wide_scale(normal, simd.add(impulse_delta0, impulse_delta1));

	proposed_body_a: Body = bodies[0];
	proposed_body_b: Body;
	_ = proposed_body_b;
	contact_apply_velocity_change(
		&proposed_body_a, linear_impulse,
		util.vector3_wide_add(
			util.vector3_wide_scale(angular_response_a0, impulse_delta0),
			util.vector3_wide_scale(angular_response_a1, impulse_delta1),
		),
		accepted_mask,
	);
	nonfinite_mask: util.I32x8 = constraint_kernel_velocity_nonfinite_mask(
		proposed_body_a.linear_velocity, proposed_body_a.angular_velocity,
	);
	when body_count == 2
	{
		proposed_body_b = bodies[1];
		contact_apply_velocity_change(
			&proposed_body_b, util.vector3_wide_negate(linear_impulse),
			util.vector3_wide_add(
				util.vector3_wide_scale(angular_response_b0, impulse_delta0),
				util.vector3_wide_scale(angular_response_b1, impulse_delta1),
			),
			accepted_mask,
		);
		nonfinite_mask |= constraint_kernel_velocity_nonfinite_mask(
			proposed_body_b.linear_velocity, proposed_body_b.angular_velocity,
		);
	}

	// commit neither body until the complete proposal for both bodies is finite
	accepted_mask &= ~nonfinite_mask;
	bodies[0].linear_velocity = constraint_kernel_select_vector3(
		accepted_mask, proposed_body_a.linear_velocity, bodies[0].linear_velocity,
	);
	bodies[0].angular_velocity = constraint_kernel_select_vector3(
		accepted_mask, proposed_body_a.angular_velocity, bodies[0].angular_velocity,
	);
	when body_count == 2
	{
		bodies[1].linear_velocity = constraint_kernel_select_vector3(
			accepted_mask, proposed_body_b.linear_velocity, bodies[1].linear_velocity,
		);
		bodies[1].angular_velocity = constraint_kernel_select_vector3(
			accepted_mask, proposed_body_b.angular_velocity, bodies[1].angular_velocity,
		);
	}
	accumulated0^ = constraint_kernel_mask_scalar(util.wide_select_f32(accepted_mask, next_impulse0, previous_impulse0), active_mask);
	accumulated1^ = constraint_kernel_mask_scalar(util.wide_select_f32(accepted_mask, next_impulse1, previous_impulse1), active_mask);

	fallback_mask: util.I32x8 = active_mask & ~accepted_mask;
	if solver_mask_has_lanes(fallback_mask) == .Present
	{
		fallback_impulse0, fallback_impulse1: util.F32x8 = previous_impulse0, previous_impulse1;
		constraint_contact_penetration_solve(
			bodies, normal, contact0.offset_a, util.vector3_wide_subtract(contact0.offset_a, offset_b), contact0.depth,
			position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity, softness,
			0, inverse_dt, &fallback_impulse0, fallback_mask, body_count,
		);
		constraint_contact_penetration_solve(
			bodies, normal, contact1.offset_a, util.vector3_wide_subtract(contact1.offset_a, offset_b), contact1.depth,
			position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity, softness,
			0, inverse_dt, &fallback_impulse1, fallback_mask, body_count,
		);
		accumulated0^ = util.wide_select_f32(fallback_mask, fallback_impulse0, accumulated0^);
		accumulated1^ = util.wide_select_f32(fallback_mask, fallback_impulse1, accumulated1^);
	}
}
