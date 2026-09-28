// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

symmetric2x2_wide_scale :: proc "contextless" (m: Symmetric2x2_Wide, scale: F32x8) -> Symmetric2x2_Wide
{
	return {simd.mul(m.xx, scale), simd.mul(m.yx, scale), simd.mul(m.yy, scale)};
}
symmetric2x2_wide_add :: proc "contextless" (a, b: Symmetric2x2_Wide) -> Symmetric2x2_Wide
{
	return {simd.add(a.xx, b.xx), simd.add(a.yx, b.yx), simd.add(a.yy, b.yy)};
}
symmetric2x2_wide_subtract :: proc "contextless" (a, b: Symmetric2x2_Wide) -> Symmetric2x2_Wide
{
	return {simd.sub(a.xx, b.xx), simd.sub(a.yx, b.yx), simd.sub(a.yy, b.yy)};
}

symmetric2x2_wide_invert :: proc "contextless" (m: Symmetric2x2_Wide) -> Symmetric2x2_Wide
{
	denominator := simd.div(F32x8(1), simd.sub(simd.mul(m.yx, m.yx), simd.mul(m.xx, m.yy)));
	return {simd.mul(simd.neg(m.yy), denominator), simd.mul(m.yx, denominator), simd.mul(simd.neg(m.xx), denominator)};
}

symmetric2x2_wide_transform :: proc "contextless" (v: Vector2_Wide, m: Symmetric2x2_Wide) -> Vector2_Wide
{
	return {wide_mul_add2(v.x, m.xx, v.y, m.yx), wide_mul_add2(v.x, m.yx, v.y, m.yy)};
}

symmetric2x2_wide_sandwich_scale :: proc "contextless" (m: Matrix2x3_Wide, scale: F32x8) -> Symmetric2x2_Wide
{
	return {
		simd.mul(scale, vector3_wide_dot(m.x, m.x)),
		simd.mul(scale, vector3_wide_dot(m.y, m.x)),
		simd.mul(scale, vector3_wide_dot(m.y, m.y)),
	};
}

symmetric2x2_wide_multiply_transposed :: proc "contextless" (a: Matrix2x3_Wide, b: Symmetric2x2_Wide) -> Matrix2x3_Wide
{
	return {
		{
			wide_mul_add2(a.x.x, b.xx, a.y.x, b.yx),
			wide_mul_add2(a.x.y, b.xx, a.y.y, b.yx),
			wide_mul_add2(a.x.z, b.xx, a.y.z, b.yx),
		},
		{
			wide_mul_add2(a.x.x, b.yx, a.y.x, b.yy),
			wide_mul_add2(a.x.y, b.yx, a.y.y, b.yy),
			wide_mul_add2(a.x.z, b.yx, a.y.z, b.yy),
		},
	};
}

symmetric2x2_wide_complete_matrix_sandwich :: proc "contextless" (a, b: Matrix2x3_Wide) -> Symmetric2x2_Wide
{
	return {vector3_wide_dot(a.x, b.x), vector3_wide_dot(a.y, b.x), vector3_wide_dot(a.y, b.y)};
}

symmetric3x3_wide_invert :: proc "contextless" (m: Symmetric3x3_Wide) -> Symmetric3x3_Wide
{
	m11 := simd.sub(simd.mul(m.yy, m.zz), simd.mul(m.zy, m.zy));
	m21 := simd.sub(simd.mul(m.zy, m.zx), simd.mul(m.zz, m.yx));
	m31 := simd.sub(simd.mul(m.yx, m.zy), simd.mul(m.zx, m.yy));
	id := simd.div(F32x8(1), wide_mul_add3(m11, m.xx, m21, m.yx, m31, m.zx));
	m22 := simd.sub(simd.mul(m.zz, m.xx), simd.mul(m.zx, m.zx));
	m32 := simd.sub(simd.mul(m.zx, m.yx), simd.mul(m.xx, m.zy));
	m33 := simd.sub(simd.mul(m.xx, m.yy), simd.mul(m.yx, m.yx));
	return {
		simd.mul(m11, id),
		simd.mul(m21, id),
		simd.mul(m22, id),
		simd.mul(m31, id),
		simd.mul(m32, id),
		simd.mul(m33, id),
	};
}

symmetric3x3_wide_add :: proc "contextless" (a, b: Symmetric3x3_Wide) -> Symmetric3x3_Wide
{
	return {
		simd.add(a.xx, b.xx),
		simd.add(a.yx, b.yx),
		simd.add(a.yy, b.yy),
		simd.add(a.zx, b.zx),
		simd.add(a.zy, b.zy),
		simd.add(a.zz, b.zz),
	};
}

symmetric3x3_wide_subtract :: proc "contextless" (a, b: Symmetric3x3_Wide) -> Symmetric3x3_Wide
{
	return {
		simd.sub(a.xx, b.xx),
		simd.sub(a.yx, b.yx),
		simd.sub(a.yy, b.yy),
		simd.sub(a.zx, b.zx),
		simd.sub(a.zy, b.zy),
		simd.sub(a.zz, b.zz),
	};
}

symmetric3x3_wide_scale :: proc "contextless" (m: Symmetric3x3_Wide, scale: F32x8) -> Symmetric3x3_Wide
{
	return {
		simd.mul(m.xx, scale),
		simd.mul(m.yx, scale),
		simd.mul(m.yy, scale),
		simd.mul(m.zx, scale),
		simd.mul(m.zy, scale),
		simd.mul(m.zz, scale),
	};
}

symmetric3x3_wide_as_matrix :: proc "contextless" (m: Symmetric3x3_Wide) -> Matrix3x3_Wide
{
	return {{m.xx, m.yx, m.zx}, {m.yx, m.yy, m.zy}, {m.zx, m.zy, m.zz}};
}

matrix3x3_wide_add_symmetric3x3 :: proc "contextless" (a: Matrix3x3_Wide, b: Symmetric3x3_Wide) -> Matrix3x3_Wide
{
	return {
		vector3_wide_add(a.x, {b.xx, b.yx, b.zx}),
		vector3_wide_add(a.y, {b.yx, b.yy, b.zy}),
		vector3_wide_add(a.z, {b.zx, b.zy, b.zz}),
	};
}

symmetric3x3_wide_from_lower :: proc "contextless" (m: Matrix3x3_Wide) -> Symmetric3x3_Wide
{
	return {m.x.x, m.y.x, m.y.y, m.z.x, m.z.y, m.z.z};
}

symmetric3x3_wide_skew_sandwich :: proc "contextless" (v: Vector3_Wide, m: Symmetric3x3_Wide) -> Symmetric3x3_Wide
{
	xzy := simd.mul(v.x, m.zy);
	yzx := simd.mul(v.y, m.zx);
	zyx := simd.mul(v.z, m.yx);
	ixy := simd.sub(simd.mul(v.y, m.zy), simd.mul(v.z, m.yy));
	ixz := simd.sub(simd.mul(v.y, m.zz), simd.mul(v.z, m.zy));
	iyx := simd.sub(simd.mul(v.z, m.xx), simd.mul(v.x, m.zx));
	iyy := simd.sub(zyx, xzy);
	iyz := simd.sub(simd.mul(v.z, m.zx), simd.mul(v.x, m.zz));
	izx := simd.sub(simd.mul(v.x, m.yx), simd.mul(v.y, m.xx));
	izy := simd.sub(simd.mul(v.x, m.yy), simd.mul(v.y, m.yx));
	izz := simd.sub(xzy, yzx);
	return {
		simd.sub(simd.mul(v.y, ixz), simd.mul(v.z, ixy)),
		simd.sub(simd.mul(v.y, iyz), simd.mul(v.z, iyy)),
		simd.sub(simd.mul(v.z, iyx), simd.mul(v.x, iyz)),
		simd.sub(simd.mul(v.y, izz), simd.mul(v.z, izy)),
		simd.sub(simd.mul(v.z, izx), simd.mul(v.x, izz)),
		simd.sub(simd.mul(v.x, izy), simd.mul(v.y, izx)),
	};
}

symmetric3x3_wide_transform :: proc "contextless" (v: Vector3_Wide, m: Symmetric3x3_Wide) -> Vector3_Wide
{
	return {
		wide_mul_add3(v.x, m.xx, v.y, m.yx, v.z, m.zx),
		wide_mul_add3(v.x, m.yx, v.y, m.yy, v.z, m.zy),
		wide_mul_add3(v.x, m.zx, v.y, m.zy, v.z, m.zz),
	};
}

symmetric3x3_wide_vector_sandwich :: proc "contextless" (v: Vector3_Wide, m: Symmetric3x3_Wide) -> F32x8
{
	return vector3_wide_dot(symmetric3x3_wide_transform(v, m), v);
}

symmetric3x3_wide_rotation_sandwich :: proc "contextless" (r: Matrix3x3_Wide, m: Symmetric3x3_Wide) -> Symmetric3x3_Wide
{
	intermediate := matrix3x3_wide_multiply_transposed(r, symmetric3x3_wide_as_matrix(m));
	return symmetric3x3_wide_from_lower(matrix3x3_wide_multiply(intermediate, r));
}

matrix2x3_wide_multiply_symmetric3x3 :: proc "contextless" (a: Matrix2x3_Wide, b: Symmetric3x3_Wide) -> Matrix2x3_Wide
{
	return matrix2x3_wide_multiply_matrix3x3(a, symmetric3x3_wide_as_matrix(b));
}

matrix3x3_wide_multiply_symmetric3x3 :: proc "contextless" (a: Matrix3x3_Wide, b: Symmetric3x3_Wide) -> Matrix3x3_Wide
{
	return matrix3x3_wide_multiply(a, symmetric3x3_wide_as_matrix(b));
}

symmetric3x3_wide_multiply_matrix3x3 :: proc "contextless" (a: Symmetric3x3_Wide, b: Matrix3x3_Wide) -> Matrix3x3_Wide
{
	return matrix3x3_wide_multiply(symmetric3x3_wide_as_matrix(a), b);
}

symmetric3x3_wide_multiply_by_transposed_matrix3x3 :: proc "contextless" (
	a: Symmetric3x3_Wide,
	b: Matrix3x3_Wide
) -> Matrix3x3_Wide
{
	return matrix3x3_wide_multiply(symmetric3x3_wide_as_matrix(a), matrix3x3_wide_transpose(b));
}

symmetric3x3_wide_multiply_by_transposed_matrix2x3 :: proc "contextless" (
	a: Symmetric3x3_Wide,
	b: Matrix2x3_Wide
) -> Matrix2x3_Wide
{
	return {
		{
			wide_mul_add3(a.xx, b.x.x, a.yx, b.x.y, a.zx, b.x.z),
			wide_mul_add3(a.yx, b.x.x, a.yy, b.x.y, a.zy, b.x.z),
			wide_mul_add3(a.zx, b.x.x, a.zy, b.x.y, a.zz, b.x.z),
		},
		{
			wide_mul_add3(a.xx, b.y.x, a.yx, b.y.y, a.zx, b.y.z),
			wide_mul_add3(a.yx, b.y.x, a.yy, b.y.y, a.zy, b.y.z),
			wide_mul_add3(a.zx, b.y.x, a.zy, b.y.y, a.zz, b.y.z),
		},
	};
}

symmetric3x3_wide_matrix_sandwich :: proc "contextless" (m: Matrix2x3_Wide, t: Symmetric3x3_Wide) -> Symmetric2x2_Wide
{
	intermediate := matrix2x3_wide_multiply_symmetric3x3(m, t);
	return symmetric2x2_wide_complete_matrix_sandwich(intermediate, m);
}

symmetric3x3_wide_complete_matrix_sandwich :: proc "contextless" (a, b: Matrix3x3_Wide) -> Symmetric3x3_Wide
{
	return {
		vector3_wide_dot(a.x, Vector3_Wide{x=b.x.x, y=b.y.x, z=b.z.x}),
		vector3_wide_dot(a.y, Vector3_Wide{x=b.x.x, y=b.y.x, z=b.z.x}),
		vector3_wide_dot(a.y, Vector3_Wide{x=b.x.y, y=b.y.y, z=b.z.y}),
		vector3_wide_dot(a.z, Vector3_Wide{x=b.x.x, y=b.y.x, z=b.z.x}),
		vector3_wide_dot(a.z, Vector3_Wide{x=b.x.y, y=b.y.y, z=b.z.y}),
		vector3_wide_dot(a.z, Vector3_Wide{x=b.x.z, y=b.y.z, z=b.z.z}),
	};
}

symmetric3x3_wide_complete_matrix_sandwich_2x3 :: proc "contextless" (a, b: Matrix2x3_Wide) -> Symmetric3x3_Wide
{
	return {
		wide_mul_add2(a.x.x, b.x.x, a.y.x, b.y.x),
		wide_mul_add2(a.x.y, b.x.x, a.y.y, b.y.x),
		wide_mul_add2(a.x.y, b.x.y, a.y.y, b.y.y),
		wide_mul_add2(a.x.z, b.x.x, a.y.z, b.y.x),
		wide_mul_add2(a.x.z, b.x.y, a.y.z, b.y.y),
		wide_mul_add2(a.x.z, b.x.z, a.y.z, b.y.z),
	};
}

symmetric3x3_wide_complete_sandwich_by_transpose :: proc "contextless" (a, b: Matrix3x3_Wide) -> Symmetric3x3_Wide
{
	return symmetric3x3_wide_from_lower(matrix3x3_wide_multiply_by_transpose(a, b));
}

symmetric3x3_wide_complete_sandwich_transpose :: proc "contextless" (a, b: Matrix3x3_Wide) -> Symmetric3x3_Wide
{
	return symmetric3x3_wide_from_lower(matrix3x3_wide_multiply_transposed(a, b));
}

symmetric3x3_wide_write_slot :: proc "contextless" (wide: ^Symmetric3x3_Wide, lane: int, source: Symmetric3x3)
{
	wide.xx = simd.replace(wide.xx, lane, source.xx);
	wide.yx = simd.replace(wide.yx, lane, source.yx);
	wide.yy = simd.replace(wide.yy, lane, source.yy);
	wide.zx = simd.replace(wide.zx, lane, source.zx);
	wide.zy = simd.replace(wide.zy, lane, source.zy);
	wide.zz = simd.replace(wide.zz, lane, source.zz);
}
