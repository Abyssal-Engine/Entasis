// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:math"

matrix3x3_identity :: proc "contextless" () -> Matrix3x3
{
	return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}

matrix3x3_add :: proc "contextless" (a, b: Matrix3x3) -> Matrix3x3
{
	return {vector3_add(a.x, b.x), vector3_add(a.y, b.y), vector3_add(a.z, b.z)};
}

matrix3x3_subtract :: proc "contextless" (a, b: Matrix3x3) -> Matrix3x3
{
	return {vector3_subtract(a.x, b.x), vector3_subtract(a.y, b.y), vector3_subtract(a.z, b.z)};
}

matrix3x3_scale :: proc "contextless" (value: Matrix3x3, scale: f32) -> Matrix3x3
{
	return {vector3_scale(value.x, scale), vector3_scale(value.y, scale), vector3_scale(value.z, scale)};
}

matrix3x3_transpose :: proc "contextless" (m: Matrix3x3) -> Matrix3x3
{
	xy := m.x.y;
	xz := m.x.z;
	yz := m.y.z;
	return {{m.x.x, m.y.x, m.z.x}, {xy, m.y.y, m.z.y}, {xz, yz, m.z.z}};
}

matrix3x3_determinant :: proc "contextless" (m: Matrix3x3) -> f32
{
	return vector3_dot(m.x, vector3_cross(m.y, m.z));
}

matrix3x3_invert :: proc "contextless" (m: Matrix3x3) -> Matrix3x3
{
	yz := vector3_cross(m.y, m.z);
	zx := vector3_cross(m.z, m.x);
	xy := vector3_cross(m.x, m.y);
	inverse_determinant := 1.0 / vector3_dot(m.x, yz);
	return matrix3x3_transpose({
			vector3_scale(yz, inverse_determinant),
			vector3_scale(zx, inverse_determinant),
			vector3_scale(xy, inverse_determinant),
		});
}

matrix3x3_transform :: proc "contextless" (v: Vector3, m: Matrix3x3) -> Vector3
{
	return vector3_add(vector3_add(vector3_scale(m.x, v.x), vector3_scale(m.y, v.y)), vector3_scale(m.z, v.z));
}

matrix3x3_transform_transpose :: proc "contextless" (v: Vector3, m: Matrix3x3) -> Vector3
{
	return {vector3_dot(v, m.x), vector3_dot(v, m.y), vector3_dot(v, m.z)};
}

matrix3x3_multiply :: proc "contextless" (a, b: Matrix3x3) -> Matrix3x3
{
	result: Matrix3x3;
	result.x = vector3_add(
		vector3_add(vector3_scale(b.x, a.x.x), vector3_scale(b.y, a.x.y)),
		vector3_scale(b.z, a.x.z)
	);
	result.y = vector3_add(
		vector3_add(vector3_scale(b.x, a.y.x), vector3_scale(b.y, a.y.y)),
		vector3_scale(b.z, a.y.z)
	);
	result.z = vector3_add(
		vector3_add(vector3_scale(b.x, a.z.x), vector3_scale(b.y, a.z.y)),
		vector3_scale(b.z, a.z.z)
	);
	return result;
}

matrix3x3_multiply_transposed :: proc "contextless" (a, b: Matrix3x3) -> Matrix3x3
{
	return matrix3x3_multiply(matrix3x3_transpose(a), b);
}

matrix3x3_from_quaternion :: proc "contextless" (quaternion: Quaternion) -> Matrix3x3
{
	qx2 := quaternion.x + quaternion.x;
	qy2 := quaternion.y + quaternion.y;
	qz2 := quaternion.z + quaternion.z;
	xx := qx2 * quaternion.x;
	yy := qy2 * quaternion.y;
	zz := qz2 * quaternion.z;
	xy := qx2 * quaternion.y;
	xz := qx2 * quaternion.z;
	xw := qx2 * quaternion.w;
	yz := qy2 * quaternion.z;
	yw := qy2 * quaternion.w;
	zw := qz2 * quaternion.w;
	return {
		{1 - yy - zz, xy + zw, xz - yw},
		{xy - zw, 1 - xx - zz, yz + xw},
		{xz + yw, yz - xw, 1 - xx - yy},
	};
}

matrix3x3_create_scale :: proc "contextless" (scale: Vector3) -> Matrix3x3
{
	return {{scale.x, 0, 0}, {0, scale.y, 0}, {0, 0, scale.z}};
}

matrix3x3_from_axis_angle :: proc "contextless" (axis: Vector3, angle: f32) -> Matrix3x3
{
	xx := axis.x * axis.x;
	yy := axis.y * axis.y;
	zz := axis.z * axis.z;
	xy := axis.x * axis.y;
	xz := axis.x * axis.z;
	yz := axis.y * axis.z;
	sin_angle := math.sin(angle);
	one_minus_cos_angle := 1 - math.cos(angle);
	return {
		{
			1 + one_minus_cos_angle * (xx - 1),
			axis.z * sin_angle + one_minus_cos_angle * xy,
			-axis.y * sin_angle + one_minus_cos_angle * xz,
		},
		{
			-axis.z * sin_angle + one_minus_cos_angle * xy,
			1 + one_minus_cos_angle * (yy - 1),
			axis.x * sin_angle + one_minus_cos_angle * yz,
		},
		{
			axis.y * sin_angle + one_minus_cos_angle * xz,
			-axis.x * sin_angle + one_minus_cos_angle * yz,
			1 + one_minus_cos_angle * (zz - 1),
		},
	};
}

matrix3x3_create_cross_product :: proc "contextless" (v: Vector3) -> Matrix3x3
{
	return {{0, -v.z, v.y}, {v.z, 0, -v.x}, {-v.y, v.x, 0}};
}
