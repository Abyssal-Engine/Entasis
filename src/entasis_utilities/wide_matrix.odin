// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

wide_mul_add2 :: proc "contextless" (a, b, c, d: F32x8) -> F32x8
{
	return simd.add(simd.mul(a, b), simd.mul(c, d));
}
wide_mul_add3 :: proc "contextless" (a, b, c, d, e, f: F32x8) -> F32x8
{
	return simd.add(wide_mul_add2(a, b, c, d), simd.mul(e, f));
}
wide_mul_add4 :: proc "contextless" (a, b, c, d, e, f, g, h: F32x8) -> F32x8
{
	return simd.add(wide_mul_add2(a, b, c, d), wide_mul_add2(e, f, g, h));
}

matrix2x2_wide_multiply_by_transpose :: proc "contextless" (a, b: Matrix2x2_Wide) -> Matrix2x2_Wide
{
	return {
		{wide_mul_add2(a.x.x, b.x.x, a.x.y, b.x.y), wide_mul_add2(a.x.x, b.y.x, a.x.y, b.y.y)},
		{wide_mul_add2(a.y.x, b.x.x, a.y.y, b.x.y), wide_mul_add2(a.y.x, b.y.x, a.y.y, b.y.y)},
	};
}

matrix2x2_wide_transform :: proc "contextless" (v: Vector2_Wide, m: Matrix2x2_Wide) -> Vector2_Wide
{
	return {wide_mul_add2(v.x, m.x.x, v.y, m.y.x), wide_mul_add2(v.x, m.x.y, v.y, m.y.y)};
}

matrix2x2_wide_scale :: proc "contextless" (m: Matrix2x2_Wide, scale: F32x8) -> Matrix2x2_Wide
{
	return {vector2_wide_scale(m.x, scale), vector2_wide_scale(m.y, scale)};
}
matrix2x2_wide_add :: proc "contextless" (a, b: Matrix2x2_Wide) -> Matrix2x2_Wide
{
	return {vector2_wide_add(a.x, b.x), vector2_wide_add(a.y, b.y)};
}
matrix2x2_wide_subtract :: proc "contextless" (a, b: Matrix2x2_Wide) -> Matrix2x2_Wide
{
	return {vector2_wide_subtract(a.x, b.x), vector2_wide_subtract(a.y, b.y)};
}

matrix2x2_wide_invert :: proc "contextless" (m: Matrix2x2_Wide) -> Matrix2x2_Wide
{
	id := simd.div(F32x8(1), simd.sub(simd.mul(m.x.x, m.y.y), simd.mul(m.x.y, m.y.x)));
	return {{simd.mul(m.y.y, id), simd.mul(simd.neg(m.x.y), id)}, {simd.mul(simd.neg(m.y.x), id), simd.mul(m.x.x, id)}};
}

matrix2x3_wide_multiply_matrix3x3 :: proc "contextless" (a: Matrix2x3_Wide, b: Matrix3x3_Wide) -> Matrix2x3_Wide
{
	return {
		{
			wide_mul_add3(a.x.x, b.x.x, a.x.y, b.y.x, a.x.z, b.z.x),
			wide_mul_add3(a.x.x, b.x.y, a.x.y, b.y.y, a.x.z, b.z.y),
			wide_mul_add3(a.x.x, b.x.z, a.x.y, b.y.z, a.x.z, b.z.z),
		},
		{
			wide_mul_add3(a.y.x, b.x.x, a.y.y, b.y.x, a.y.z, b.z.x),
			wide_mul_add3(a.y.x, b.x.y, a.y.y, b.y.y, a.y.z, b.z.y),
			wide_mul_add3(a.y.x, b.x.z, a.y.y, b.y.z, a.y.z, b.z.z),
		},
	};
}

matrix2x2_wide_multiply_matrix2x3 :: proc "contextless" (a: Matrix2x2_Wide, b: Matrix2x3_Wide) -> Matrix2x3_Wide
{
	return {
		{
			wide_mul_add2(a.x.x, b.x.x, a.x.y, b.y.x),
			wide_mul_add2(a.x.x, b.x.y, a.x.y, b.y.y),
			wide_mul_add2(a.x.x, b.x.z, a.x.y, b.y.z),
		},
		{
			wide_mul_add2(a.y.x, b.x.x, a.y.y, b.y.x),
			wide_mul_add2(a.y.x, b.x.y, a.y.y, b.y.y),
			wide_mul_add2(a.y.x, b.x.z, a.y.y, b.y.z),
		},
	};
}

matrix2x2_wide_transpose_multiply_matrix2x3 :: proc "contextless" (
	a: Matrix2x2_Wide,
	b: Matrix2x3_Wide
) -> Matrix2x3_Wide
{
	return {
		{
			wide_mul_add2(a.x.x, b.x.x, a.y.x, b.y.x),
			wide_mul_add2(a.x.x, b.x.y, a.y.x, b.y.y),
			wide_mul_add2(a.x.x, b.x.z, a.y.x, b.y.z),
		},
		{
			wide_mul_add2(a.x.y, b.x.x, a.y.y, b.y.x),
			wide_mul_add2(a.x.y, b.x.y, a.y.y, b.y.y),
			wide_mul_add2(a.x.y, b.x.z, a.y.y, b.y.z),
		},
	};
}

matrix2x3_wide_multiply_by_transpose :: proc "contextless" (a, b: Matrix2x3_Wide) -> Matrix2x2_Wide
{
	return {
		{
			wide_mul_add3(a.x.x, b.x.x, a.x.y, b.x.y, a.x.z, b.x.z),
			wide_mul_add3(a.x.x, b.y.x, a.x.y, b.y.y, a.x.z, b.y.z),
		},
		{
			wide_mul_add3(a.y.x, b.x.x, a.y.y, b.x.y, a.y.z, b.x.z),
			wide_mul_add3(a.y.x, b.y.x, a.y.y, b.y.y, a.y.z, b.y.z),
		},
	};
}

matrix2x3_wide_transform_by_transpose :: proc "contextless" (v: Vector3_Wide, m: Matrix2x3_Wide) -> Vector2_Wide
{
	return {wide_mul_add3(v.x, m.x.x, v.y, m.x.y, v.z, m.x.z), wide_mul_add3(v.x, m.y.x, v.y, m.y.y, v.z, m.y.z)};
}

matrix2x3_wide_transform :: proc "contextless" (v: Vector2_Wide, m: Matrix2x3_Wide) -> Vector3_Wide
{
	return {
		wide_mul_add2(v.x, m.x.x, v.y, m.y.x),
		wide_mul_add2(v.x, m.x.y, v.y, m.y.y),
		wide_mul_add2(v.x, m.x.z, v.y, m.y.z),
	};
}

matrix2x3_wide_negate :: proc "contextless" (m: Matrix2x3_Wide) -> Matrix2x3_Wide
{
	return {vector3_wide_negate(m.x), vector3_wide_negate(m.y)};
}
matrix2x3_wide_scale :: proc "contextless" (m: Matrix2x3_Wide, scale: F32x8) -> Matrix2x3_Wide
{
	return {vector3_wide_scale(m.x, scale), vector3_wide_scale(m.y, scale)};
}
matrix2x3_wide_add :: proc "contextless" (a, b: Matrix2x3_Wide) -> Matrix2x3_Wide
{
	return {vector3_wide_add(a.x, b.x), vector3_wide_add(a.y, b.y)};
}

matrix3x3_wide_broadcast :: proc "contextless" (source: Matrix3x3) -> Matrix3x3_Wide
{
	return {vector3_wide_broadcast(source.x), vector3_wide_broadcast(source.y), vector3_wide_broadcast(source.z)};
}

matrix3x3_wide_identity :: proc "contextless" () -> Matrix3x3_Wide
{
	return matrix3x3_wide_broadcast(matrix3x3_identity());
}

matrix3x3_wide_multiply :: proc "contextless" (a, b: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return {
		{
			wide_mul_add3(a.x.x, b.x.x, a.x.y, b.y.x, a.x.z, b.z.x),
			wide_mul_add3(a.x.x, b.x.y, a.x.y, b.y.y, a.x.z, b.z.y),
			wide_mul_add3(a.x.x, b.x.z, a.x.y, b.y.z, a.x.z, b.z.z),
		},
		{
			wide_mul_add3(a.y.x, b.x.x, a.y.y, b.y.x, a.y.z, b.z.x),
			wide_mul_add3(a.y.x, b.x.y, a.y.y, b.y.y, a.y.z, b.z.y),
			wide_mul_add3(a.y.x, b.x.z, a.y.y, b.y.z, a.y.z, b.z.z),
		},
		{
			wide_mul_add3(a.z.x, b.x.x, a.z.y, b.y.x, a.z.z, b.z.x),
			wide_mul_add3(a.z.x, b.x.y, a.z.y, b.y.y, a.z.z, b.z.y),
			wide_mul_add3(a.z.x, b.x.z, a.z.y, b.y.z, a.z.z, b.z.z),
		},
	};
}

matrix3x3_wide_transpose :: proc "contextless" (m: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return {{m.x.x, m.y.x, m.z.x}, {m.x.y, m.y.y, m.z.y}, {m.x.z, m.y.z, m.z.z}};
}

matrix3x3_wide_multiply_transposed :: proc "contextless" (a, b: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return matrix3x3_wide_multiply(matrix3x3_wide_transpose(a), b);
}
matrix3x3_wide_multiply_by_transpose :: proc "contextless" (a, b: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return matrix3x3_wide_multiply(a, matrix3x3_wide_transpose(b));
}

matrix3x3_wide_transform :: proc "contextless" (v: Vector3_Wide, m: Matrix3x3_Wide) -> Vector3_Wide
{
	return {
		wide_mul_add3(v.x, m.x.x, v.y, m.y.x, v.z, m.z.x),
		wide_mul_add3(v.x, m.x.y, v.y, m.y.y, v.z, m.z.y),
		wide_mul_add3(v.x, m.x.z, v.y, m.y.z, v.z, m.z.z),
	};
}

matrix3x3_wide_transform_transposed :: proc "contextless" (v: Vector3_Wide, m: Matrix3x3_Wide) -> Vector3_Wide
{
	return {vector3_wide_dot(v, m.x), vector3_wide_dot(v, m.y), vector3_wide_dot(v, m.z)};
}

matrix3x3_wide_invert :: proc "contextless" (m: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	yz := vector3_wide_cross(m.y, m.z);
	zx := vector3_wide_cross(m.z, m.x);
	xy := vector3_wide_cross(m.x, m.y);
	id := simd.div(F32x8(1), vector3_wide_dot(m.x, yz));
	return matrix3x3_wide_transpose({
			vector3_wide_scale(yz, id),
			vector3_wide_scale(zx, id),
			vector3_wide_scale(xy, id)
		});
}

matrix3x3_wide_create_cross_product :: proc "contextless" (v: Vector3_Wide) -> Matrix3x3_Wide
{
	zero := F32x8(0);
	return {{zero, simd.neg(v.z), v.y}, {v.z, zero, simd.neg(v.x)}, {simd.neg(v.y), v.x, zero}};
}

matrix3x3_wide_negate :: proc "contextless" (m: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return {vector3_wide_negate(m.x), vector3_wide_negate(m.y), vector3_wide_negate(m.z)};
}
matrix3x3_wide_scale :: proc "contextless" (m: Matrix3x3_Wide, scale: F32x8) -> Matrix3x3_Wide
{
	return {vector3_wide_scale(m.x, scale), vector3_wide_scale(m.y, scale), vector3_wide_scale(m.z, scale)};
}
matrix3x3_wide_subtract :: proc "contextless" (a, b: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return {vector3_wide_subtract(a.x, b.x), vector3_wide_subtract(a.y, b.y), vector3_wide_subtract(a.z, b.z)};
}

matrix3x3_wide_from_quaternion :: proc "contextless" (q: Quaternion_Wide) -> Matrix3x3_Wide
{
	return {
		quaternion_wide_transform_unit_x(q),
		quaternion_wide_transform_unit_y(q),
		quaternion_wide_transform_unit_z(q),
	};
}

matrix3x3_wide_read_slot :: proc "contextless" (wide: Matrix3x3_Wide, lane: int) -> Matrix3x3
{
	return {
		vector3_wide_read_slot(wide.x, lane),
		vector3_wide_read_slot(wide.y, lane),
		vector3_wide_read_slot(wide.z, lane),
	};
}
