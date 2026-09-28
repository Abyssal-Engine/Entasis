// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

quaternion_wide_broadcast :: proc "contextless" (source: Quaternion) -> Quaternion_Wide
{
	return {F32x8(source.x), F32x8(source.y), F32x8(source.z), F32x8(source.w)};
}

quaternion_wide_read_slot :: proc "contextless" (wide: Quaternion_Wide, lane: int) -> Quaternion
{
	return {
		simd.extract(wide.x, lane),
		simd.extract(wide.y, lane),
		simd.extract(wide.z, lane),
		simd.extract(wide.w, lane),
	};
}

quaternion_wide_write_slot :: proc "contextless" (wide: ^Quaternion_Wide, lane: int, source: Quaternion)
{
	wide.x = simd.replace(wide.x, lane, source.x);
	wide.y = simd.replace(wide.y, lane, source.y);
	wide.z = simd.replace(wide.z, lane, source.z);
	wide.w = simd.replace(wide.w, lane, source.w);
}

quaternion_wide_rebroadcast :: proc "contextless" (source: Quaternion_Wide, lane: int) -> Quaternion_Wide
{
	return quaternion_wide_broadcast(quaternion_wide_read_slot(source, lane));
}

quaternion_wide_add :: proc "contextless" (a, b: Quaternion_Wide) -> Quaternion_Wide
{
	return {simd.add(a.x, b.x), simd.add(a.y, b.y), simd.add(a.z, b.z), simd.add(a.w, b.w)};
}

quaternion_wide_scale :: proc "contextless" (q: Quaternion_Wide, scale: F32x8) -> Quaternion_Wide
{
	return {simd.mul(q.x, scale), simd.mul(q.y, scale), simd.mul(q.z, scale), simd.mul(q.w, scale)};
}

quaternion_wide_length_squared :: proc "contextless" (q: Quaternion_Wide) -> F32x8
{
	return simd.add(simd.add(simd.mul(q.x, q.x), simd.mul(q.y, q.y)), simd.add(simd.mul(q.z, q.z), simd.mul(q.w, q.w)));
}

quaternion_wide_length :: proc "contextless" (q: Quaternion_Wide) -> F32x8
{
	return simd.sqrt(quaternion_wide_length_squared(q));
}

quaternion_wide_normalize :: proc "contextless" (q: Quaternion_Wide) -> Quaternion_Wide
{
	return quaternion_wide_scale(q, simd.div(F32x8(1), quaternion_wide_length(q)));
}

quaternion_wide_negate :: proc "contextless" (q: Quaternion_Wide) -> Quaternion_Wide
{
	return {simd.neg(q.x), simd.neg(q.y), simd.neg(q.z), simd.neg(q.w)};
}

quaternion_wide_from_rotation_matrix :: proc "contextless" (r: Matrix3x3_Wide) -> Quaternion_Wide
{
	one_add_x := simd.add(F32x8(1), r.x.x);
	one_sub_x := simd.sub(F32x8(1), r.x.x);
	y_add_z := simd.add(r.y.y, r.z.z);
	y_sub_z := simd.sub(r.y.y, r.z.z);
	tx := simd.sub(one_add_x, y_add_z);
	ty := simd.add(one_sub_x, y_sub_z);
	tz := simd.sub(one_sub_x, y_sub_z);
	tw := simd.add(one_add_x, y_add_z);
	use_upper := transmute(I32x8)simd.lanes_lt(r.z.z, F32x8(0));
	use_upper_upper := transmute(I32x8)simd.lanes_gt(r.x.x, r.y.y);
	use_lower_upper := transmute(I32x8)simd.lanes_lt(r.x.x, simd.neg(r.y.y));
	t := wide_select_f32(use_upper, wide_select_f32(use_upper_upper, tx, ty), wide_select_f32(use_lower_upper, tz, tw));
	xy_add_yx := simd.add(r.x.y, r.y.x);
	yz_sub_zy := simd.sub(r.y.z, r.z.y);
	zx_add_xz := simd.add(r.z.x, r.x.z);
	yz_add_zy := simd.add(r.y.z, r.z.y);
	zx_sub_xz := simd.sub(r.z.x, r.x.z);
	xy_sub_yx := simd.sub(r.x.y, r.y.x);
	q := Quaternion_Wide{
		x=wide_select_f32(
			use_upper,
			wide_select_f32(use_upper_upper, tx, xy_add_yx),
			wide_select_f32(use_lower_upper, zx_add_xz, yz_sub_zy)
		),
		y=wide_select_f32(
			use_upper,
			wide_select_f32(use_upper_upper, xy_add_yx, ty),
			wide_select_f32(use_lower_upper, yz_add_zy, zx_sub_xz)
		),
		z=wide_select_f32(
			use_upper,
			wide_select_f32(use_upper_upper, zx_add_xz, yz_add_zy),
			wide_select_f32(use_lower_upper, tz, xy_sub_yx)
		),
		w=wide_select_f32(
			use_upper,
			wide_select_f32(use_upper_upper, yz_sub_zy, zx_sub_xz),
			wide_select_f32(use_lower_upper, xy_sub_yx, tw)
		),
	};
	return quaternion_wide_scale(q, simd.div(F32x8(0.5), simd.sqrt(t)));
}

quaternion_wide_between_normalized_vectors :: proc "contextless" (v1, v2: Vector3_Wide) -> Quaternion_Wide
{
	dot := vector3_wide_dot(v1, v2);
	cross := vector3_wide_cross(v1, v2);
	use_normal := transmute(I32x8)simd.lanes_gt(dot, F32x8(-0.999999));
	abs_x := simd.abs(v1.x);
	abs_y := simd.abs(v1.y);
	abs_z := simd.abs(v1.z);
	x_smallest_mask := simd.bit_and(simd.lanes_lt(abs_x, abs_y), simd.lanes_lt(abs_x, abs_z));
	x_smallest := transmute(I32x8)x_smallest_mask;
	y_smaller := transmute(I32x8)simd.lanes_lt(abs_y, abs_z);
	q := Quaternion_Wide{
		x=wide_select_f32(
			use_normal,
			cross.x,
			wide_select_f32(x_smallest, F32x8(0), wide_select_f32(y_smaller, simd.neg(v1.z), simd.neg(v1.y)))
		),
		y=wide_select_f32(
			use_normal,
			cross.y,
			wide_select_f32(x_smallest, simd.neg(v1.z), wide_select_f32(y_smaller, F32x8(0), v1.x))
		),
		z=wide_select_f32(
			use_normal,
			cross.z,
			wide_select_f32(x_smallest, v1.y, wide_select_f32(y_smaller, v1.x, F32x8(0)))
		),
		w=wide_select_f32(use_normal, simd.add(dot, F32x8(1)), F32x8(0)),
	};
	return quaternion_wide_normalize(q);
}

quaternion_wide_axis_angle :: proc "contextless" (q: Quaternion_Wide) -> (axis: Vector3_Wide, angle: F32x8)
{
	should_negate := transmute(I32x8)simd.lanes_lt(q.w, F32x8(0));
	axis = {
		wide_select_f32(should_negate, simd.neg(q.x), q.x),
		wide_select_f32(should_negate, simd.neg(q.y), q.y),
		wide_select_f32(should_negate, simd.neg(q.z), q.z),
	};
	qw := wide_select_f32(should_negate, simd.neg(q.w), q.w);
	axis_length := vector3_wide_length(axis);
	axis = vector3_wide_divide(axis, axis_length);
	use_fallback := transmute(I32x8)simd.lanes_lt(axis_length, F32x8(1e-14));
	axis.x = wide_select_f32(use_fallback, F32x8(1), axis.x);
	axis.y = wide_select_f32(use_fallback, F32x8(0), axis.y);
	axis.z = wide_select_f32(use_fallback, F32x8(0), axis.z);
	angle = simd.mul(F32x8(2), acos_approx_wide(qw));
	return;
}

quaternion_wide_transform :: proc "contextless" (v: Vector3_Wide, rotation: Quaternion_Wide) -> Vector3_Wide
{
	x2 := simd.add(rotation.x, rotation.x);
	y2 := simd.add(rotation.y, rotation.y);
	z2 := simd.add(rotation.z, rotation.z);
	xx2 := simd.mul(rotation.x, x2);
	xy2 := simd.mul(rotation.x, y2);
	xz2 := simd.mul(rotation.x, z2);
	yy2 := simd.mul(rotation.y, y2);
	yz2 := simd.mul(rotation.y, z2);
	zz2 := simd.mul(rotation.z, z2);
	wx2 := simd.mul(rotation.w, x2);
	wy2 := simd.mul(rotation.w, y2);
	wz2 := simd.mul(rotation.w, z2);
	return {
		simd.add(
			simd.add(simd.mul(v.x, simd.sub(simd.sub(F32x8(1), yy2), zz2)), simd.mul(v.y, simd.sub(xy2, wz2))),
			simd.mul(v.z, simd.add(xz2, wy2))
		),
		simd.add(
			simd.add(simd.mul(v.x, simd.add(xy2, wz2)), simd.mul(v.y, simd.sub(simd.sub(F32x8(1), xx2), zz2))),
			simd.mul(v.z, simd.sub(yz2, wx2))
		),
		simd.add(
			simd.add(simd.mul(v.x, simd.sub(xz2, wy2)), simd.mul(v.y, simd.add(yz2, wx2))),
			simd.mul(v.z, simd.sub(simd.sub(F32x8(1), xx2), yy2))
		),
	};
}

quaternion_wide_transform_by_conjugate :: proc "contextless" (
	v: Vector3_Wide,
	rotation: Quaternion_Wide
) -> Vector3_Wide
{
	conjugate_form := rotation;
	conjugate_form.w = simd.neg(rotation.w);
	return quaternion_wide_transform(v, conjugate_form);
}

quaternion_wide_transform_unit_x :: proc "contextless" (rotation: Quaternion_Wide) -> Vector3_Wide
{
	y2 := simd.add(rotation.y, rotation.y);
	z2 := simd.add(rotation.z, rotation.z);
	return {
		simd.sub(simd.sub(F32x8(1), simd.mul(rotation.y, y2)), simd.mul(rotation.z, z2)),
		simd.add(simd.mul(rotation.x, y2), simd.mul(rotation.w, z2)),
		simd.sub(simd.mul(rotation.x, z2), simd.mul(rotation.w, y2)),
	};
}

quaternion_wide_transform_unit_y :: proc "contextless" (rotation: Quaternion_Wide) -> Vector3_Wide
{
	x2 := simd.add(rotation.x, rotation.x);
	y2 := simd.add(rotation.y, rotation.y);
	z2 := simd.add(rotation.z, rotation.z);
	return {
		simd.sub(simd.mul(rotation.x, y2), simd.mul(rotation.w, z2)),
		simd.sub(simd.sub(F32x8(1), simd.mul(rotation.x, x2)), simd.mul(rotation.z, z2)),
		simd.add(simd.mul(rotation.y, z2), simd.mul(rotation.w, x2)),
	};
}

quaternion_wide_transform_unit_z :: proc "contextless" (rotation: Quaternion_Wide) -> Vector3_Wide
{
	x2 := simd.add(rotation.x, rotation.x);
	y2 := simd.add(rotation.y, rotation.y);
	z2 := simd.add(rotation.z, rotation.z);
	return {
		simd.add(simd.mul(rotation.x, z2), simd.mul(rotation.w, y2)),
		simd.sub(simd.mul(rotation.y, z2), simd.mul(rotation.w, x2)),
		simd.sub(simd.sub(F32x8(1), simd.mul(rotation.x, x2)), simd.mul(rotation.y, y2)),
	};
}

quaternion_wide_transform_unit_xy :: proc "contextless" (rotation: Quaternion_Wide) -> (x, y: Vector3_Wide)
{
	x2 := simd.add(rotation.x, rotation.x);
	y2 := simd.add(rotation.y, rotation.y);
	z2 := simd.add(rotation.z, rotation.z);
	xx2 := simd.mul(rotation.x, x2);
	xy2 := simd.mul(rotation.x, y2);
	xz2 := simd.mul(rotation.x, z2);
	yy2 := simd.mul(rotation.y, y2);
	yz2 := simd.mul(rotation.y, z2);
	zz2 := simd.mul(rotation.z, z2);
	wx2 := simd.mul(rotation.w, x2);
	wy2 := simd.mul(rotation.w, y2);
	wz2 := simd.mul(rotation.w, z2);
	x = {simd.sub(simd.sub(F32x8(1), yy2), zz2), simd.add(xy2, wz2), simd.sub(xz2, wy2)};
	y = {simd.sub(xy2, wz2), simd.sub(simd.sub(F32x8(1), xx2), zz2), simd.add(yz2, wx2)};
	return;
}

quaternion_wide_transform_unit_xz :: proc "contextless" (rotation: Quaternion_Wide) -> (x, z: Vector3_Wide)
{
	qx2 := simd.add(rotation.x, rotation.x);
	qy2 := simd.add(rotation.y, rotation.y);
	qz2 := simd.add(rotation.z, rotation.z);
	yy := simd.mul(qy2, rotation.y);
	zz := simd.mul(qz2, rotation.z);
	xy := simd.mul(qx2, rotation.y);
	zw := simd.mul(qz2, rotation.w);
	xz := simd.mul(qx2, rotation.z);
	yw := simd.mul(qy2, rotation.w);
	xx := simd.mul(qx2, rotation.x);
	xw := simd.mul(qx2, rotation.w);
	yz := simd.mul(qy2, rotation.z);
	x = {simd.sub(simd.sub(F32x8(1), yy), zz), simd.add(xy, zw), simd.sub(xz, yw)};
	z = {simd.add(xz, yw), simd.sub(yz, xw), simd.sub(simd.sub(F32x8(1), xx), yy)};
	return;
}

quaternion_wide_concatenate :: proc "contextless" (a, b: Quaternion_Wide) -> Quaternion_Wide
{
	return {
		simd.sub(simd.add(simd.add(simd.mul(a.w, b.x), simd.mul(a.x, b.w)), simd.mul(a.z, b.y)), simd.mul(a.y, b.z)),
		simd.sub(simd.add(simd.add(simd.mul(a.w, b.y), simd.mul(a.y, b.w)), simd.mul(a.x, b.z)), simd.mul(a.z, b.x)),
		simd.sub(simd.add(simd.add(simd.mul(a.w, b.z), simd.mul(a.z, b.w)), simd.mul(a.y, b.x)), simd.mul(a.x, b.y)),
		simd.sub(simd.sub(simd.sub(simd.mul(a.w, b.w), simd.mul(a.x, b.x)), simd.mul(a.y, b.y)), simd.mul(a.z, b.z)),
	};
}

quaternion_wide_conjugate :: proc "contextless" (q: Quaternion_Wide) -> Quaternion_Wide
{
	return {q.x, q.y, q.z, simd.neg(q.w)};
}

quaternion_wide_select :: proc "contextless" (condition: I32x8, left, right: Quaternion_Wide) -> Quaternion_Wide
{
	return {
		wide_select_f32(condition, left.x, right.x),
		wide_select_f32(condition, left.y, right.y),
		wide_select_f32(condition, left.z, right.z),
		wide_select_f32(condition, left.w, right.w),
	};
}
