// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

symmetric4x4_wide_scale :: proc "contextless" (m: Symmetric4x4_Wide, scale: F32x8) -> Symmetric4x4_Wide
{
	return {
		simd.mul(
			m.xx,
			scale
		), simd.mul(m.yx, scale), simd.mul(m.yy, scale), simd.mul(m.zx, scale), simd.mul(m.zy, scale),
		simd.mul(
			m.zz,
			scale
		), simd.mul(m.wx, scale), simd.mul(m.wy, scale), simd.mul(m.wz, scale), simd.mul(m.ww, scale),
	};
}

symmetric4x4_wide_invert :: proc "contextless" (m: Symmetric4x4_Wide) -> Symmetric4x4_Wide
{
	s0 := simd.sub(simd.mul(m.xx, m.yy), simd.mul(m.yx, m.yx));
	s1 := simd.sub(simd.mul(m.xx, m.zy), simd.mul(m.yx, m.zx));
	s2 := simd.sub(simd.mul(m.xx, m.wy), simd.mul(m.yx, m.wx));
	s3 := simd.sub(simd.mul(m.yx, m.zy), simd.mul(m.yy, m.zx));
	s4 := simd.sub(simd.mul(m.yx, m.wy), simd.mul(m.yy, m.wx));
	s5 := simd.sub(simd.mul(m.zx, m.wy), simd.mul(m.zy, m.wx));
	c5 := simd.sub(simd.mul(m.zz, m.ww), simd.mul(m.wz, m.wz));
	c4 := simd.sub(simd.mul(m.zy, m.ww), simd.mul(m.wy, m.wz));
	c3 := simd.sub(simd.mul(m.zy, m.wz), simd.mul(m.wy, m.zz));
	c2 := simd.sub(simd.mul(m.zx, m.ww), simd.mul(m.wx, m.wz));
	c1 := simd.sub(simd.mul(m.zx, m.wz), simd.mul(m.wx, m.zz));
	determinant := simd.add(
		simd.sub(simd.add(simd.mul(s0, c5), simd.mul(s2, c3)), simd.mul(s1, c4)),
		simd.add(simd.sub(simd.mul(s3, c2), simd.mul(s4, c1)), simd.mul(s5, s5)),
	);
	id := simd.div(F32x8(1), determinant);
	return {
		simd.mul(wide_mul_add3(m.yy, c5, simd.neg(m.zy), c4, m.wy, c3), id),
		simd.mul(wide_mul_add3(simd.neg(m.yx), c5, m.zy, c2, simd.neg(m.wy), c1), id),
		simd.mul(wide_mul_add3(m.xx, c5, simd.neg(m.zx), c2, m.wx, c1), id),
		simd.mul(wide_mul_add3(m.yx, c4, simd.neg(m.yy), c2, m.wy, s5), id),
		simd.mul(wide_mul_add3(simd.neg(m.xx), c4, m.yx, c2, simd.neg(m.wx), s5), id),
		simd.mul(wide_mul_add3(m.wx, s4, simd.neg(m.wy), s2, m.ww, s0), id),
		simd.mul(wide_mul_add3(simd.neg(m.yx), c3, m.yy, c1, simd.neg(m.zy), s5), id),
		simd.mul(wide_mul_add3(m.xx, c3, simd.neg(m.yx), c1, m.zx, s5), id),
		simd.mul(wide_mul_add3(simd.neg(m.wx), s3, m.wy, s1, simd.neg(m.wz), s0), id),
		simd.mul(wide_mul_add3(m.zx, s3, simd.neg(m.zy), s1, m.zz, s0), id),
	};
}

symmetric4x4_wide_transform :: proc "contextless" (v: Vector4_Wide, m: Symmetric4x4_Wide) -> Vector4_Wide
{
	return {
		wide_mul_add4(v.x, m.xx, v.y, m.yx, v.z, m.zx, v.w, m.wx),
		wide_mul_add4(v.x, m.yx, v.y, m.yy, v.z, m.zy, v.w, m.wy),
		wide_mul_add4(v.x, m.zx, v.y, m.zy, v.z, m.zz, v.w, m.wz),
		wide_mul_add4(v.x, m.wx, v.y, m.wy, v.z, m.wz, v.w, m.ww),
	};
}

symmetric5x5_wide_scale :: proc "contextless" (m: Symmetric5x5_Wide, scale: F32x8) -> Symmetric5x5_Wide
{
	return {symmetric3x3_wide_scale(m.a, scale), matrix2x3_wide_scale(m.b, scale), symmetric2x2_wide_scale(m.d, scale)};
}

symmetric5x5_wide_invert_blocks :: proc "contextless" (
	a: Symmetric3x3_Wide,
	b: Matrix2x3_Wide,
	d: Symmetric2x2_Wide
) -> Symmetric5x5_Wide
{
	inv_d := symmetric2x2_wide_invert(d);
	b_t_inv_d := symmetric2x2_wide_multiply_transposed(b, inv_d);
	b_t_inv_d_b := symmetric3x3_wide_complete_matrix_sandwich_2x3(b_t_inv_d, b);
	result_a := symmetric3x3_wide_invert(symmetric3x3_wide_subtract(a, b_t_inv_d_b));
	negated_result_b_t := symmetric3x3_wide_multiply_by_transposed_matrix2x3(result_a, b_t_inv_d);
	result_b := matrix2x3_wide_negate(negated_result_b_t);
	result_d := symmetric2x2_wide_add(symmetric2x2_wide_complete_matrix_sandwich(b_t_inv_d, negated_result_b_t), inv_d);
	return {result_a, result_b, result_d};
}

symmetric5x5_wide_invert :: proc "contextless" (m: Symmetric5x5_Wide) -> Symmetric5x5_Wide
{
	return symmetric5x5_wide_invert_blocks(m.a, m.b, m.d);
}

symmetric5x5_wide_transform :: proc "contextless" (
	v0: Vector3_Wide,
	v1: Vector2_Wide,
	m: Symmetric5x5_Wide
) -> (result0: Vector3_Wide, result1: Vector2_Wide)
{
	result0 = vector3_wide_add(symmetric3x3_wide_transform(v0, m.a), matrix2x3_wide_transform(v1, m.b));
	result1 = vector2_wide_add(matrix2x3_wide_transform_by_transpose(v0, m.b), symmetric2x2_wide_transform(v1, m.d));
	return;
}

symmetric6x6_wide_scale :: proc "contextless" (m: Symmetric6x6_Wide, scale: F32x8) -> Symmetric6x6_Wide
{
	return {symmetric3x3_wide_scale(m.a, scale), matrix3x3_wide_scale(m.b, scale), symmetric3x3_wide_scale(m.d, scale)};
}

symmetric6x6_wide_invert_blocks :: proc "contextless" (
	a: Symmetric3x3_Wide,
	b: Matrix3x3_Wide,
	d: Symmetric3x3_Wide
) -> Symmetric6x6_Wide
{
	inv_d := symmetric3x3_wide_invert(d);
	b_inv_d := matrix3x3_wide_multiply_symmetric3x3(b, inv_d);
	b_inv_d_b_t := symmetric3x3_wide_complete_sandwich_by_transpose(b_inv_d, b);
	result_a := symmetric3x3_wide_invert(symmetric3x3_wide_subtract(a, b_inv_d_b_t));
	negated_result_b := symmetric3x3_wide_multiply_matrix3x3(result_a, b_inv_d);
	result_b := matrix3x3_wide_negate(negated_result_b);
	result_d := symmetric3x3_wide_add(symmetric3x3_wide_complete_sandwich_transpose(b_inv_d, negated_result_b), inv_d);
	return {result_a, result_b, result_d};
}

symmetric6x6_wide_invert :: proc "contextless" (m: Symmetric6x6_Wide) -> Symmetric6x6_Wide
{
	return symmetric6x6_wide_invert_blocks(m.a, m.b, m.d);
}

symmetric6x6_wide_transform :: proc "contextless" (
	v0,
	v1: Vector3_Wide,
	m: Symmetric6x6_Wide
) -> (result0, result1: Vector3_Wide)
{
	result0 = vector3_wide_add(symmetric3x3_wide_transform(v0, m.a), matrix3x3_wide_transform_transposed(v1, m.b));
	result1 = vector3_wide_add(matrix3x3_wide_transform(v0, m.b), symmetric3x3_wide_transform(v1, m.d));
	return;
}

symmetric6x6_wide_ldlt_solve :: proc "contextless" (
	v0, v1: Vector3_Wide, a: Symmetric3x3_Wide, b: Matrix3x3_Wide, d: Symmetric3x3_Wide,
) -> (result0, result1: Vector3_Wide)
{
	d1 := a.xx;
	inverse_d1 := F32x8(1) / d1;
	l21 := inverse_d1 * a.yx;
	l31 := inverse_d1 * a.zx;
	l41 := inverse_d1 * b.x.x;
	l51 := inverse_d1 * b.x.y;
	l61 := inverse_d1 * b.x.z;
	d2 := a.yy - l21 * l21 * d1;
	inverse_d2 := F32x8(1) / d2;
	l32 := inverse_d2 * (a.zy - l31 * l21 * d1);
	l42 := inverse_d2 * (b.y.x - l41 * l21 * d1);
	l52 := inverse_d2 * (b.y.y - l51 * l21 * d1);
	l62 := inverse_d2 * (b.y.z - l61 * l21 * d1);
	d3 := a.zz - l31 * l31 * d1 - l32 * l32 * d2;
	inverse_d3 := F32x8(1) / d3;
	l43 := inverse_d3 * (b.z.x - l41 * l31 * d1 - l42 * l32 * d2);
	l53 := inverse_d3 * (b.z.y - l51 * l31 * d1 - l52 * l32 * d2);
	l63 := inverse_d3 * (b.z.z - l61 * l31 * d1 - l62 * l32 * d2);
	d4 := d.xx - l41 * l41 * d1 - l42 * l42 * d2 - l43 * l43 * d3;
	inverse_d4 := F32x8(1) / d4;
	l54 := inverse_d4 * (d.yx - l51 * l41 * d1 - l52 * l42 * d2 - l53 * l43 * d3);
	l64 := inverse_d4 * (d.zx - l61 * l41 * d1 - l62 * l42 * d2 - l63 * l43 * d3);
	d5 := d.yy - l51 * l51 * d1 - l52 * l52 * d2 - l53 * l53 * d3 - l54 * l54 * d4;
	inverse_d5 := F32x8(1) / d5;
	l65 := inverse_d5 * (d.zy - l61 * l51 * d1 - l62 * l52 * d2 - l63 * l53 * d3 - l64 * l54 * d4);
	d6 := d.zz - l61 * l61 * d1 - l62 * l62 * d2 - l63 * l63 * d3 - l64 * l64 * d4 - l65 * l65 * d5;
	inverse_d6 := F32x8(1) / d6;
	result0.x = v0.x;
	result0.y = v0.y - l21 * result0.x;
	result0.z = v0.z - l31 * result0.x - l32 * result0.y;
	result1.x = v1.x - l41 * result0.x - l42 * result0.y - l43 * result0.z;
	result1.y = v1.y - l51 * result0.x - l52 * result0.y - l53 * result0.z - l54 * result1.x;
	result1.z = v1.z - l61 * result0.x - l62 * result0.y - l63 * result0.z - l64 * result1.x - l65 * result1.y;
	result1.z = result1.z * inverse_d6;
	result1.y = result1.y * inverse_d5 - l65 * result1.z;
	result1.x = result1.x * inverse_d4 - l64 * result1.z - l54 * result1.y;
	result0.z = result0.z * inverse_d3 - l63 * result1.z - l53 * result1.y - l43 * result1.x;
	result0.y = result0.y * inverse_d2 - l62 * result1.z - l52 * result1.y - l42 * result1.x - l32 * result0.z;
	result0.x = result0.x * inverse_d1 - l61 * result1.z - l51 * result1.y - l41 * result1.x - l31 * result0.z - l21 * result0.y;
	return;
}
