package solver_contacts

import "core:math"
import "core:slice"
import physics "entasis:entasis_physics"

Quality_Status :: enum u8
{
	Pass,
	Fail,
}

Quality_Result :: struct
{
	status: Quality_Status,
	expected_rows, observed_rows, residual_rows: int,
	finite_failures, invalid_coefficients, feasibility_failures: int,
	residual_max_mps, residual_rms_mps, residual_p95_mps: f64,
	negative_impulse_max, friction_violation_max, twist_violation_max: f64,
}

Quality_Body :: struct
{
	linear, angular: [3]f64,
	inverse_mass: f64,
	inverse_inertia: [6]f64,
}

Quality_Manifold :: struct
{
	bodies: [2]Quality_Body,
	body_count, contact_count: int,
	offsets: [4][3]f64,
	depths, normal_impulses: [4]f64,
	normal, offset_b: [3]f64,
	tangent_impulse: [2]f64,
	twist_impulse, friction, angular_frequency, twice_damping_ratio: f64,
	maximum_recovery_velocity, dt: f64,
}

quality_finite :: proc(value: f64) -> physics.Reference_State
{
	if transmute(u64)value & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000
	{
		return .Missing;
	}
	return .Present;
}

quality_dot :: proc(a, b: [3]f64) -> f64
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

quality_cross :: proc(a, b: [3]f64) -> [3]f64
{
	return {a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]};
}

quality_inertia_transform :: proc(tensor: [6]f64, v: [3]f64) -> [3]f64
{
	return {
		tensor[0]*v[0] + tensor[1]*v[1] + tensor[3]*v[2],
		tensor[1]*v[0] + tensor[2]*v[1] + tensor[4]*v[2],
		tensor[3]*v[0] + tensor[4]*v[1] + tensor[5]*v[2],
	};
}

quality_body :: proc(body: ^physics.Body_Dynamics) -> Quality_Body
{
	v := body.motion.velocity;
	i := body.inertia.world;
	return {
		linear={f64(v.linear.x), f64(v.linear.y), f64(v.linear.z)},
		angular={f64(v.angular.x), f64(v.angular.y), f64(v.angular.z)},
		inverse_mass=f64(i.inverse_mass),
		inverse_inertia={f64(i.inverse_inertia_tensor.xx), f64(i.inverse_inertia_tensor.yx), f64(i.inverse_inertia_tensor.yy),
			f64(i.inverse_inertia_tensor.zx), f64(i.inverse_inertia_tensor.zy), f64(i.inverse_inertia_tensor.zz)},
	};
}

// all rows read the final shared body state, including changes from later manifolds
quality_manifold_observe :: proc(m: ^Quality_Manifold, result: ^Quality_Result, scratch: []f64)
{
	result.observed_rows += m.contact_count;
	coefficients: [8]f64 = {m.friction, m.angular_frequency, m.twice_damping_ratio,
		m.maximum_recovery_velocity, m.dt, m.normal[0], m.normal[1], m.normal[2]};
	valid: physics.Reference_State = .Present;
	for value in coefficients
	{
		if quality_finite(value) == .Missing
		{
			valid = .Missing;
		}
	}
	for value in m.offset_b
	{
		if quality_finite(value) == .Missing
		{
			valid = .Missing;
		}
	}
	if valid == .Missing || m.dt <= 0 || m.angular_frequency <= 0 || m.twice_damping_ratio < 0 ||
		m.friction < 0 || m.maximum_recovery_velocity < 0
	{
		result.invalid_coefficients += m.contact_count;
		return;
	}
	angular_frequency_dt := m.angular_frequency*m.dt;
	denominator := angular_frequency_dt + m.twice_damping_ratio;
	position_scale := m.angular_frequency/denominator;
	extra := 1/(angular_frequency_dt*denominator);
	effective_mass_scale := 1/(1 + extra);
	softness := extra*effective_mass_scale;
	if quality_finite(effective_mass_scale) == .Missing || effective_mass_scale <= 0 ||
		quality_finite(softness) == .Missing || quality_finite(position_scale) == .Missing
	{
		result.invalid_coefficients += m.contact_count;
		return;
	}

	total_impulse, center_weight: f64;
	center: [3]f64;
	for point in 0 ..< m.contact_count
	{
		if m.depths[point] >= 0
		{
			center_weight += 1;
			for axis in 0 ..< 3
			{
				center[axis] += m.offsets[point][axis];
			}
		}
	}
	if center_weight == 0
	{
		center_weight = f64(m.contact_count);
		for point in 0 ..< m.contact_count
		{
			for axis in 0 ..< 3
			{
				center[axis] += m.offsets[point][axis];
			}
		}
	}
	for axis in 0 ..< 3
	{
		center[axis] /= center_weight;
	}

	twist_radius_impulse: f64;
	for point in 0 ..< m.contact_count
	{
		lambda := m.normal_impulses[point];
		point_valid: physics.Reference_State = .Present;
		for value in m.offsets[point]
		{
			if quality_finite(value) == .Missing
			{
				point_valid = .Missing;
			}
		}
		if quality_finite(m.depths[point]) == .Missing
		{
			point_valid = .Missing;
		}
		if point_valid == .Missing
		{
			result.invalid_coefficients += 1;
			continue;
		}
		if quality_finite(lambda) == .Missing
		{
			result.finite_failures += 1;
			continue;
		}
		negative := max(0, -lambda);
		result.negative_impulse_max = max(result.negative_impulse_max, negative);
		if negative > 64*f64(1.1920928955078125e-7)*max(1, math.abs(lambda))
		{
			result.feasibility_failures += 1;
		}
		total_impulse += lambda;
		if m.contact_count == 1
		{
			twist_radius_impulse += lambda*max(0, m.depths[point]);
		}
		else
		{
			delta := m.offsets[point] - center;
			twist_radius_impulse += lambda*math.sqrt(quality_dot(delta, delta));
		}

		jacobian_a := quality_cross(m.offsets[point], m.normal);
		normal_length_squared := quality_dot(m.normal, m.normal);
		w := m.bodies[0].inverse_mass*normal_length_squared + quality_dot(jacobian_a, quality_inertia_transform(m.bodies[0].inverse_inertia, jacobian_a));
		velocity := quality_dot(m.normal, m.bodies[0].linear) + quality_dot(jacobian_a, m.bodies[0].angular);
		if m.body_count == 2
		{
			jacobian_b := quality_cross(m.normal, m.offsets[point] - m.offset_b);
			w += m.bodies[1].inverse_mass*normal_length_squared + quality_dot(jacobian_b, quality_inertia_transform(m.bodies[1].inverse_inertia, jacobian_b));
			velocity += -quality_dot(m.normal, m.bodies[1].linear) + quality_dot(jacobian_b, m.bodies[1].angular);
		}
		if quality_finite(w) == .Missing || w <= 0
		{
			result.invalid_coefficients += 1;
			continue;
		}
		bias := min(m.depths[point]/m.dt, min(m.depths[point]*position_scale, m.maximum_recovery_velocity));
		gamma := w*softness/effective_mass_scale;
		u := velocity - bias + gamma*lambda;
		eta := math.abs(min(w/effective_mass_scale*lambda, u));
		if quality_finite(u) == .Missing || quality_finite(eta) == .Missing
		{
			result.finite_failures += 1;
			continue;
		}
		scratch[result.residual_rows] = eta;
		result.residual_rows += 1;
	}

	if quality_finite(m.tangent_impulse[0]) == .Missing || quality_finite(m.tangent_impulse[1]) == .Missing ||
		quality_finite(m.twist_impulse) == .Missing
	{
		result.finite_failures += 1;
		return;
	}
	tangent_magnitude := math.sqrt(m.tangent_impulse[0]*m.tangent_impulse[0] + m.tangent_impulse[1]*m.tangent_impulse[1]);
	tangent_limit := m.friction*total_impulse;
	twist_limit := m.friction*twist_radius_impulse;
	tangent_violation := max(0, tangent_magnitude - tangent_limit);
	twist_violation := max(0, math.abs(m.twist_impulse) - twist_limit);
	result.friction_violation_max = max(result.friction_violation_max, tangent_violation);
	result.twist_violation_max = max(result.twist_violation_max, twist_violation);
	if tangent_violation > 64*f64(1.1920928955078125e-7)*max(1, max(tangent_magnitude, math.abs(tangent_limit)))
	{
		result.feasibility_failures += 1;
	}
	if twist_violation > 64*f64(1.1920928955078125e-7)*max(1, max(math.abs(m.twist_impulse), math.abs(twist_limit)))
	{
		result.feasibility_failures += 1;
	}
}

quality_finish :: proc(result: ^Quality_Result, scratch: []f64)
{
	if result.residual_rows > 0
	{
		values := scratch[:result.residual_rows];
		slice.sort(values);
		result.residual_max_mps = values[len(values) - 1];
		if result.residual_max_mps > 0
		{
			scaled_squared_sum: f64;
			for value in values
			{
				scaled := value/result.residual_max_mps;
				scaled_squared_sum += scaled*scaled;
			}
			result.residual_rms_mps = result.residual_max_mps*math.sqrt(scaled_squared_sum/f64(len(values)));
		}
		rank := int(math.ceil(0.95*f64(len(values)))) - 1;
		result.residual_p95_mps = values[rank];
	}
	if result.finite_failures != 0 || result.invalid_coefficients != 0 || result.feasibility_failures != 0 ||
		result.observed_rows != result.expected_rows || result.residual_rows != result.expected_rows
	{
		result.status = .Fail;
	}
}

quality_observe :: proc(replay: ^Replay, scratch: []f64) -> Quality_Result
{
	result: Quality_Result = {expected_rows=int(input_counts(replay.input).points)};
	active := &replay.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for index in 0 ..< active.count
	{
		body := &active.dynamics_state.memory[index];
		pose := body.motion.pose;
		velocity := body.motion.velocity;
		inertia := body.inertia.world;
		values: [20]f32 = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y,
			pose.orientation.z, pose.orientation.w, velocity.linear.x, velocity.linear.y, velocity.linear.z,
			velocity.angular.x, velocity.angular.y, velocity.angular.z, inertia.inverse_mass,
			inertia.inverse_inertia_tensor.xx, inertia.inverse_inertia_tensor.yx, inertia.inverse_inertia_tensor.yy,
			inertia.inverse_inertia_tensor.zx, inertia.inverse_inertia_tensor.zy, inertia.inverse_inertia_tensor.zz};
		for value in values
		{
			if quality_finite(f64(value)) == .Missing
			{
				result.finite_failures += 1;
			}
		}
	}
	for batch_index in 0 ..< replay.solver.active_set.batch_count
	{
		batch := &replay.solver.active_set.batches.memory[batch_index];
		for type_index in 0 ..< batch.type_batch_count
		{
			kind := &batch.type_batches.memory[type_index];
			for index in 0 ..< kind.count
			{
				contact := contact_read(kind, int(index));
				m: Quality_Manifold = {
					body_count=int(kind.body_count), contact_count=int(kind.type_id%4 + 1),
					normal={f64(contact.normal.x), f64(contact.normal.y), f64(contact.normal.z)},
					offset_b={f64(contact.offset_b.x), f64(contact.offset_b.y), f64(contact.offset_b.z)},
					tangent_impulse={f64(contact.tangent_impulse.x), f64(contact.tangent_impulse.y)},
					twist_impulse=f64(contact.twist_impulse), friction=f64(contact.material.friction_coefficient),
					angular_frequency=f64(contact.material.spring_settings.angular_frequency),
					twice_damping_ratio=f64(contact.material.spring_settings.twice_damping_ratio),
					maximum_recovery_velocity=f64(contact.material.maximum_recovery_velocity),
					dt=f64(replay.input.dt/f32(replay.input.solve.substep_count)),
				};
				for body in 0 ..< m.body_count
				{
					body_index := int(u32(contact.bodies[body]) & physics.BODY_REFERENCE_INDEX_MASK);
					m.bodies[body] = quality_body(&active.dynamics_state.memory[body_index]);
				}
				for point in 0 ..< m.contact_count
				{
					offset := contact.offsets[point];
					m.offsets[point] = {f64(offset.x), f64(offset.y), f64(offset.z)};
					m.depths[point] = f64(contact.depths[point]);
					m.normal_impulses[point] = f64(contact.normal_impulses[point]);
				}
				quality_manifold_observe(&m, &result, scratch);
			}
		}
	}
	quality_finish(&result, scratch);
	return result;
}
