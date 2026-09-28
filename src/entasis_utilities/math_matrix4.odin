// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:math"

matrix_identity :: proc "contextless" () -> Matrix
{
	return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
}

matrix_translation :: proc "contextless" (m: Matrix) -> Vector3
{
	return {m.w.x, m.w.y, m.w.z};
}

matrix_set_translation :: proc "contextless" (m: ^Matrix, translation: Vector3)
{
	m.w.x = translation.x;
	m.w.y = translation.y;
	m.w.z = translation.z;
}

matrix_transpose :: proc "contextless" (m: Matrix) -> Matrix
{
	return {
		{m.x.x, m.y.x, m.z.x, m.w.x},
		{m.x.y, m.y.y, m.z.y, m.w.y},
		{m.x.z, m.y.z, m.z.z, m.w.z},
		{m.x.w, m.y.w, m.z.w, m.w.w},
	};
}

matrix_transform_transpose :: proc "contextless" (v: Vector4, m: Matrix) -> Vector4
{
	return {vector4_dot(v, m.x), vector4_dot(v, m.y), vector4_dot(v, m.z), vector4_dot(v, m.w)};
}

matrix_transform :: proc "contextless" (v: Vector4, m: Matrix) -> Vector4
{
	return vector4_add(
		vector4_add(vector4_scale(m.x, v.x), vector4_scale(m.y, v.y)),
		vector4_add(vector4_scale(m.z, v.z), vector4_scale(m.w, v.w)),
	);
}

matrix_transform_vector3 :: proc "contextless" (v: Vector3, m: Matrix) -> Vector4
{
	return vector4_add(vector4_add(vector4_scale(m.x, v.x), vector4_scale(m.y, v.y)), vector4_scale(m.z, v.z));
}

matrix_multiply :: proc "contextless" (a, b: Matrix) -> Matrix
{
	return {matrix_transform(a.x, b), matrix_transform(a.y, b), matrix_transform(a.z, b), matrix_transform(a.w, b)};
}

matrix_from_axis_angle :: proc "contextless" (axis: Vector3, angle: f32) -> Matrix
{
	r := matrix3x3_from_axis_angle(axis, angle);
	return matrix_from_matrix3x3(r);
}

matrix_from_quaternion :: proc "contextless" (q: Quaternion) -> Matrix
{
	return matrix_from_matrix3x3(matrix3x3_from_quaternion(q));
}

matrix_from_matrix3x3 :: proc "contextless" (m: Matrix3x3) -> Matrix
{
	return {{m.x.x, m.x.y, m.x.z, 0}, {m.y.x, m.y.y, m.y.z, 0}, {m.z.x, m.z.y, m.z.z, 0}, {0, 0, 0, 1}};
}

matrix_to_matrix3x3 :: proc "contextless" (m: Matrix) -> Matrix3x3
{
	return {{m.x.x, m.x.y, m.x.z}, {m.y.x, m.y.y, m.y.z}, {m.z.x, m.z.y, m.z.z}};
}

matrix_create_perspective_field_of_view :: proc "contextless" (
	field_of_view,
	aspect_ratio,
	near_clip,
	far_clip: f32
) -> Matrix
{
	h := 1.0 / math.tan(field_of_view * 0.5);
	w := h / aspect_ratio;
	m33 := far_clip / (near_clip - far_clip);
	return {{w, 0, 0, 0}, {0, h, 0, 0}, {0, 0, m33, -1}, {0, 0, near_clip * m33, 0}};
}

matrix_create_perspective_field_of_view_lh :: proc "contextless" (
	field_of_view,
	aspect_ratio,
	near_clip,
	far_clip: f32
) -> Matrix
{
	h := 1.0 / math.tan(field_of_view * 0.5);
	w := h / aspect_ratio;
	m33 := far_clip / (far_clip - near_clip);
	return {{w, 0, 0, 0}, {0, h, 0, 0}, {0, 0, m33, 1}, {0, 0, -near_clip * m33, 0}};
}

matrix_create_perspective_from_field_of_views :: proc "contextless" (
	vertical_field_of_view,
	horizontal_field_of_view,
	near_clip,
	far_clip: f32
) -> Matrix
{
	h := 1.0 / math.tan(vertical_field_of_view * 0.5);
	w := 1.0 / math.tan(horizontal_field_of_view * 0.5);
	m33 := far_clip / (near_clip - far_clip);
	return {{w, 0, 0, 0}, {0, h, 0, 0}, {0, 0, m33, -1}, {0, 0, near_clip * m33, 0}};
}

matrix_create_orthographic :: proc "contextless" (left, right, bottom, top, z_near, z_far: f32) -> Matrix
{
	width := right - left;
	height := top - bottom;
	depth := z_far - z_near;
	return {
		{2 / width, 0, 0, 0},
		{0, 2 / height, 0, 0},
		{0, 0, -1 / depth, 0},
		{(left + right) / -width, (top + bottom) / -height, z_near / -depth, 1},
	};
}

matrix_invert :: proc "contextless" (m: Matrix) -> Matrix
{
	s0 := m.x.x * m.y.y - m.y.x * m.x.y;
	s1 := m.x.x * m.y.z - m.y.x * m.x.z;
	s2 := m.x.x * m.y.w - m.y.x * m.x.w;
	s3 := m.x.y * m.y.z - m.y.y * m.x.z;
	s4 := m.x.y * m.y.w - m.y.y * m.x.w;
	s5 := m.x.z * m.y.w - m.y.z * m.x.w;
	c5 := m.z.z * m.w.w - m.w.z * m.z.w;
	c4 := m.z.y * m.w.w - m.w.y * m.z.w;
	c3 := m.z.y * m.w.z - m.w.y * m.z.z;
	c2 := m.z.x * m.w.w - m.w.x * m.z.w;
	c1 := m.z.x * m.w.z - m.w.x * m.z.z;
	c0 := m.z.x * m.w.y - m.w.x * m.z.y;
	id := 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
	return {
		vector4_scale(
			{
				m.y.y * c5 - m.y.z * c4 + m.y.w * c3,
				-m.x.y * c5 + m.x.z * c4 - m.x.w * c3,
				m.w.y * s5 - m.w.z * s4 + m.w.w * s3,
				-m.z.y * s5 + m.z.z * s4 - m.z.w * s3
			},
			id
		),
		vector4_scale(
			{
				-m.y.x * c5 + m.y.z * c2 - m.y.w * c1,
				m.x.x * c5 - m.x.z * c2 + m.x.w * c1,
				-m.w.x * s5 + m.w.z * s2 - m.w.w * s1,
				m.z.x * s5 - m.z.z * s2 + m.z.w * s1
			},
			id
		),
		vector4_scale(
			{
				m.y.x * c4 - m.y.y * c2 + m.y.w * c0,
				-m.x.x * c4 + m.x.y * c2 - m.x.w * c0,
				m.w.x * s4 - m.w.y * s2 + m.w.w * s0,
				-m.z.x * s4 + m.z.y * s2 - m.z.w * s0
			},
			id
		),
		vector4_scale(
			{
				-m.y.x * c3 + m.y.y * c1 - m.y.z * c0,
				m.x.x * c3 - m.x.y * c1 + m.x.z * c0,
				-m.w.x * s3 + m.w.y * s1 - m.w.z * s0,
				m.z.x * s3 - m.z.y * s1 + m.z.z * s0
			},
			id
		),
	};
}

matrix_create_view :: proc "contextless" (position, forward, up_vector: Vector3) -> Matrix
{
	z := vector3_scale(forward, -1.0 / vector3_length(forward));
	x := vector3_normalize(vector3_cross(up_vector, z));
	y := vector3_cross(z, x);
	return {
		{x.x, y.x, z.x, 0},
		{x.y, y.y, z.y, 0},
		{x.z, y.z, z.z, 0},
		{-vector3_dot(x, position), -vector3_dot(y, position), -vector3_dot(z, position), 1},
	};
}

matrix_create_look_at :: proc "contextless" (position, target, up_vector: Vector3) -> Matrix
{
	return matrix_create_view(position, vector3_subtract(target, position), up_vector);
}

matrix_create_rigid :: proc "contextless" (rotation: Matrix3x3, position: Vector3) -> Matrix
{
	return {
		{rotation.x.x, rotation.x.y, rotation.x.z, 0},
		{rotation.y.x, rotation.y.y, rotation.y.z, 0},
		{rotation.z.x, rotation.z.y, rotation.z.z, 0},
		{position.x, position.y, position.z, 1},
	};
}
