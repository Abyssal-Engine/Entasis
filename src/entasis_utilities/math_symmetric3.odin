// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

symmetric3x3_rotation_sandwich :: proc "contextless" (r: Matrix3x3, m: Symmetric3x3) -> Symmetric3x3
{
	i11 := r.x.x * m.xx + r.y.x * m.yx + r.z.x * m.zx;
	i12 := r.x.x * m.yx + r.y.x * m.yy + r.z.x * m.zy;
	i13 := r.x.x * m.zx + r.y.x * m.zy + r.z.x * m.zz;
	i21 := r.x.y * m.xx + r.y.y * m.yx + r.z.y * m.zx;
	i22 := r.x.y * m.yx + r.y.y * m.yy + r.z.y * m.zy;
	i23 := r.x.y * m.zx + r.y.y * m.zy + r.z.y * m.zz;
	i31 := r.x.z * m.xx + r.y.z * m.yx + r.z.z * m.zx;
	i32 := r.x.z * m.yx + r.y.z * m.yy + r.z.z * m.zy;
	i33 := r.x.z * m.zx + r.y.z * m.zy + r.z.z * m.zz;
	return {
		i11 * r.x.x + i12 * r.y.x + i13 * r.z.x,
		i21 * r.x.x + i22 * r.y.x + i23 * r.z.x,
		i21 * r.x.y + i22 * r.y.y + i23 * r.z.y,
		i31 * r.x.x + i32 * r.y.x + i33 * r.z.x,
		i31 * r.x.y + i32 * r.y.y + i33 * r.z.y,
		i31 * r.x.z + i32 * r.y.z + i33 * r.z.z,
	};
}

symmetric3x3_determinant :: proc "contextless" (m: Symmetric3x3) -> f32
{
	m11 := m.yy * m.zz - m.zy * m.zy;
	m21 := m.zy * m.zx - m.zz * m.yx;
	m31 := m.yx * m.zy - m.zx * m.yy;
	return m11 * m.xx + m21 * m.yx + m31 * m.zx;
}

symmetric3x3_invert :: proc "contextless" (m: Symmetric3x3) -> Symmetric3x3
{
	m11 := m.yy * m.zz - m.zy * m.zy;
	m21 := m.zy * m.zx - m.zz * m.yx;
	m31 := m.yx * m.zy - m.zx * m.yy;
	determinant_inverse := 1.0 / (m11 * m.xx + m21 * m.yx + m31 * m.zx);
	m22 := m.zz * m.xx - m.zx * m.zx;
	m32 := m.zx * m.yx - m.xx * m.zy;
	m33 := m.xx * m.yy - m.yx * m.yx;
	return {
		m11 * determinant_inverse,
		m21 * determinant_inverse,
		m22 * determinant_inverse,
		m31 * determinant_inverse,
		m32 * determinant_inverse,
		m33 * determinant_inverse,
	};
}

symmetric3x3_add :: proc "contextless" (a, b: Symmetric3x3) -> Symmetric3x3
{
	return {a.xx + b.xx, a.yx + b.yx, a.yy + b.yy, a.zx + b.zx, a.zy + b.zy, a.zz + b.zz};
}

symmetric3x3_subtract :: proc "contextless" (a, b: Symmetric3x3) -> Symmetric3x3
{
	return {a.xx - b.xx, a.yx - b.yx, a.yy - b.yy, a.zx - b.zx, a.zy - b.zy, a.zz - b.zz};
}

matrix3x3_add_symmetric :: proc "contextless" (a: Matrix3x3, b: Symmetric3x3) -> Matrix3x3
{
	return matrix3x3_add(a, symmetric3x3_as_matrix(b));
}

matrix3x3_subtract_symmetric :: proc "contextless" (a: Matrix3x3, b: Symmetric3x3) -> Matrix3x3
{
	return matrix3x3_subtract(a, symmetric3x3_as_matrix(b));
}

symmetric3x3_subtract_matrix :: proc "contextless" (a: Symmetric3x3, b: Matrix3x3) -> Matrix3x3
{
	return matrix3x3_subtract(symmetric3x3_as_matrix(a), b);
}

symmetric3x3_scale :: proc "contextless" (m: Symmetric3x3, scale: f32) -> Symmetric3x3
{
	return {m.xx * scale, m.yx * scale, m.yy * scale, m.zx * scale, m.zy * scale, m.zz * scale};
}

symmetric3x3_multiply :: proc "contextless" (a, b: Symmetric3x3) -> Symmetric3x3
{
	ayxbyx := a.yx * b.yx;
	azxbzx := a.zx * b.zx;
	azybzy := a.zy * b.zy;
	return {
		a.xx * b.xx + ayxbyx + azxbzx,
		a.yx * b.xx + a.yy * b.yx + a.zy * b.zx,
		ayxbyx + a.yy * b.yy + azybzy,
		a.zx * b.xx + a.zy * b.yx + a.zz * b.zx,
		a.zx * b.yx + a.zy * b.yy + a.zz * b.zy,
		azxbzx + azybzy + a.zz * b.zz,
	};
}

symmetric3x3_as_matrix :: proc "contextless" (m: Symmetric3x3) -> Matrix3x3
{
	return {{m.xx, m.yx, m.zx}, {m.yx, m.yy, m.zy}, {m.zx, m.zy, m.zz}};
}

matrix3x3_multiply_symmetric :: proc "contextless" (a: Matrix3x3, b: Symmetric3x3) -> Matrix3x3
{
	return matrix3x3_multiply(a, symmetric3x3_as_matrix(b));
}

symmetric3x3_multiply_matrix :: proc "contextless" (a: Symmetric3x3, b: Matrix3x3) -> Matrix3x3
{
	return matrix3x3_multiply(symmetric3x3_as_matrix(a), b);
}

symmetric3x3_transform :: proc "contextless" (v: Vector3, m: Symmetric3x3) -> Vector3
{
	return {
		v.x * m.xx + v.y * m.yx + v.z * m.zx,
		v.x * m.yx + v.y * m.yy + v.z * m.zy,
		v.x * m.zx + v.y * m.zy + v.z * m.zz,
	};
}
