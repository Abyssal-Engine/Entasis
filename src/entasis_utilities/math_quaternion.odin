// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:math"

quaternion_identity :: proc "contextless" () -> Quaternion
{
	return {0, 0, 0, 1};
}
quaternion_add :: proc "contextless" (a, b: Quaternion) -> Quaternion
{
	return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
quaternion_scale :: proc "contextless" (q: Quaternion, scale: f32) -> Quaternion
{
	return {q.x * scale, q.y * scale, q.z * scale, q.w * scale};
}
quaternion_negate :: proc "contextless" (q: Quaternion) -> Quaternion
{
	return {-q.x, -q.y, -q.z, -q.w};
}
quaternion_conjugate :: proc "contextless" (q: Quaternion) -> Quaternion
{
	return {-q.x, -q.y, -q.z, q.w};
}

quaternion_concatenate :: proc "contextless" (a, b: Quaternion) -> Quaternion
{
	return {
		a.w * b.x + a.x * b.w + a.z * b.y - a.y * b.z,
		a.w * b.y + a.y * b.w + a.x * b.z - a.z * b.x,
		a.w * b.z + a.z * b.w + a.y * b.x - a.x * b.y,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
	};
}

quaternion_length_squared :: proc "contextless" (q: Quaternion) -> f32
{
	return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}
quaternion_length :: proc "contextless" (q: Quaternion) -> f32
{
	return math.sqrt(quaternion_length_squared(q));
}
quaternion_normalize :: proc "contextless" (q: Quaternion) -> Quaternion
{
	return quaternion_scale(q, 1.0 / quaternion_length(q));
}

quaternion_from_rotation_matrix :: proc "contextless" (r: Matrix3x3) -> Quaternion
{
	q: Quaternion;
	t: f32;
	if r.z.z < 0
	{
		if r.x.x > r.y.y
		{
			t = 1 + r.x.x - r.y.y - r.z.z;
			q = {t, r.x.y + r.y.x, r.z.x + r.x.z, r.y.z - r.z.y};
		}
		else
		{
			t = 1 - r.x.x + r.y.y - r.z.z;
			q = {r.x.y + r.y.x, t, r.y.z + r.z.y, r.z.x - r.x.z};
		}
	}
	else
	{
		if r.x.x < -r.y.y
		{
			t = 1 - r.x.x - r.y.y + r.z.z;
			q = {r.z.x + r.x.z, r.y.z + r.z.y, t, r.x.y - r.y.x};
		}
		else
		{
			t = 1 + r.x.x + r.y.y + r.z.z;
			q = {r.y.z - r.z.y, r.z.x - r.x.z, r.x.y - r.y.x, t};
		}
	}
	return quaternion_scale(q, 0.5 / math.sqrt(t));
}

quaternion_from_matrix :: proc "contextless" (m: Matrix) -> Quaternion
{
	return quaternion_from_rotation_matrix(matrix_to_matrix3x3(m));
}

quaternion_slerp :: proc "contextless" (start, end_input: Quaternion, interpolation_amount: f32) -> Quaternion
{
	end := end_input;
	cos_half_theta := f64(start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z);
	if cos_half_theta < 0
	{
		end = quaternion_negate(end);
		cos_half_theta = -cos_half_theta;
	}
	if cos_half_theta > 1.0 - 1e-12
	{
		return start;
	}
	half_theta := math.acos(cos_half_theta);
	sin_half_theta := math.sqrt(1.0 - cos_half_theta * cos_half_theta);
	a_fraction := math.sin((1.0 - f64(interpolation_amount)) * half_theta) / sin_half_theta;
	b_fraction := math.sin(f64(interpolation_amount) * half_theta) / sin_half_theta;
	return {
		f32(f64(start.x) * a_fraction + f64(end.x) * b_fraction),
		f32(f64(start.y) * a_fraction + f64(end.y) * b_fraction),
		f32(f64(start.z) * a_fraction + f64(end.z) * b_fraction),
		f32(f64(start.w) * a_fraction + f64(end.w) * b_fraction),
	};
}

quaternion_inverse :: proc "contextless" (quaternion: Quaternion) -> Quaternion
{
	inverse_squared_norm := quaternion_length_squared(quaternion);
	return {
		-quaternion.x * inverse_squared_norm,
		-quaternion.y * inverse_squared_norm,
		-quaternion.z * inverse_squared_norm,
		quaternion.w * inverse_squared_norm,
	};
}

quaternion_transform :: proc "contextless" (v: Vector3, rotation: Quaternion) -> Vector3
{
	x2 := rotation.x + rotation.x;
	y2 := rotation.y + rotation.y;
	z2 := rotation.z + rotation.z;
	xx2 := rotation.x * x2;
	xy2 := rotation.x * y2;
	xz2 := rotation.x * z2;
	yy2 := rotation.y * y2;
	yz2 := rotation.y * z2;
	zz2 := rotation.z * z2;
	wx2 := rotation.w * x2;
	wy2 := rotation.w * y2;
	wz2 := rotation.w * z2;
	return {
		v.x * (1 - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
		v.x * (xy2 + wz2) + v.y * (1 - xx2 - zz2) + v.z * (yz2 - wx2),
		v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1 - xx2 - yy2),
	};
}

quaternion_transform_unit_x :: proc "contextless" (rotation: Quaternion) -> Vector3
{
	y2 := rotation.y + rotation.y;
	z2 := rotation.z + rotation.z;
	return {
		1 - rotation.y * y2 - rotation.z * z2,
		rotation.x * y2 + rotation.w * z2,
		rotation.x * z2 - rotation.w * y2,
	};
}

quaternion_transform_unit_y :: proc "contextless" (rotation: Quaternion) -> Vector3
{
	x2 := rotation.x + rotation.x;
	y2 := rotation.y + rotation.y;
	z2 := rotation.z + rotation.z;
	return {
		rotation.x * y2 - rotation.w * z2,
		1 - rotation.x * x2 - rotation.z * z2,
		rotation.y * z2 + rotation.w * x2,
	};
}

quaternion_transform_unit_z :: proc "contextless" (rotation: Quaternion) -> Vector3
{
	x2 := rotation.x + rotation.x;
	y2 := rotation.y + rotation.y;
	return {
		rotation.x * (rotation.z + rotation.z) + rotation.w * y2,
		rotation.y * (rotation.z + rotation.z) - rotation.w * x2,
		1 - rotation.x * x2 - rotation.y * y2,
	};
}

quaternion_from_axis_angle :: proc "contextless" (axis: Vector3, angle: f32) -> Quaternion
{
	half_angle := f64(angle) * 0.5;
	s := math.sin(half_angle);
	return {f32(f64(axis.x) * s), f32(f64(axis.y) * s), f32(f64(axis.z) * s), f32(math.cos(half_angle))};
}

quaternion_from_yaw_pitch_roll :: proc "contextless" (yaw, pitch, roll: f32) -> Quaternion
{
	half_roll := f64(roll) * 0.5;
	half_pitch := f64(pitch) * 0.5;
	half_yaw := f64(yaw) * 0.5;
	sin_roll := math.sin(half_roll);
	sin_pitch := math.sin(half_pitch);
	sin_yaw := math.sin(half_yaw);
	cos_roll := math.cos(half_roll);
	cos_pitch := math.cos(half_pitch);
	cos_yaw := math.cos(half_yaw);
	cos_yaw_cos_pitch := cos_yaw * cos_pitch;
	cos_yaw_sin_pitch := cos_yaw * sin_pitch;
	sin_yaw_cos_pitch := sin_yaw * cos_pitch;
	sin_yaw_sin_pitch := sin_yaw * sin_pitch;
	return {
		f32(cos_yaw_sin_pitch * cos_roll + sin_yaw_cos_pitch * sin_roll),
		f32(sin_yaw_cos_pitch * cos_roll - cos_yaw_sin_pitch * sin_roll),
		f32(cos_yaw_cos_pitch * sin_roll - sin_yaw_sin_pitch * cos_roll),
		f32(cos_yaw_cos_pitch * cos_roll + sin_yaw_sin_pitch * sin_roll),
	};
}

quaternion_angle :: proc "contextless" (q: Quaternion) -> f32
{
	qw := abs(q.w);
	if qw > 1
	{
		return 0;
	}
	return 2 * f32(math.acos(f64(qw)));
}

quaternion_axis_angle :: proc "contextless" (q: Quaternion) -> (axis: Vector3, angle: f32)
{
	qw := q.w;
	if qw > 0
	{
		axis = {q.x, q.y, q.z};
	}
	else
	{
		axis = {-q.x, -q.y, -q.z};
		qw = -qw;
	}
	length_squared := vector3_length_squared(axis);
	if length_squared > 1e-14
	{
		axis = vector3_scale(axis, 1.0 / math.sqrt(length_squared));
		angle = 2 * f32(math.acos(f64(clamp(qw, -1, 1))));
	}
	else
	{
		axis = {0, 1, 0};
		angle = 0;
	}
	return;
}

quaternion_between_normalized_vectors :: proc "contextless" (v1, v2: Vector3) -> Quaternion
{
	dot := vector3_dot(v1, v2);
	q: Quaternion;
	if dot < -0.9999
	{
		abs_x := abs(v1.x);
		abs_y := abs(v1.y);
		abs_z := abs(v1.z);
		if abs_x < abs_y && abs_x < abs_z
		{
			q = {0, -v1.z, v1.y, 0};
		}
		else if abs_y < abs_z
		{
			q = {-v1.z, 0, v1.x, 0};
		}
		else
		{
			q = {-v1.y, v1.x, 0, 0};
		}
	}
	else
	{
		axis := vector3_cross(v1, v2);
		q = {axis.x, axis.y, axis.z, dot + 1};
	}
	return quaternion_normalize(q);
}

quaternion_relative_rotation :: proc "contextless" (start, end: Quaternion) -> Quaternion
{
	return quaternion_concatenate(quaternion_conjugate(start), end);
}

quaternion_local_rotation :: proc "contextless" (rotation, target_basis: Quaternion) -> Quaternion
{
	return quaternion_concatenate(rotation, quaternion_conjugate(target_basis));
}
