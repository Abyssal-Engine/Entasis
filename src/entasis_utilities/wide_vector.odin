// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

wide_select_f32 :: proc "contextless" (condition: I32x8, left, right: F32x8) -> F32x8
{
	return simd.select(transmute(Mask32x8)condition, left, right);
}

vector2_wide_add :: proc "contextless" (a, b: Vector2_Wide) -> Vector2_Wide
{
	return {simd.add(a.x, b.x), simd.add(a.y, b.y)};
}
vector2_wide_add_scalar :: proc "contextless" (v: Vector2_Wide, scalar: F32x8) -> Vector2_Wide
{
	return {simd.add(v.x, scalar), simd.add(v.y, scalar)};
}
vector2_wide_subtract :: proc "contextless" (a, b: Vector2_Wide) -> Vector2_Wide
{
	return {simd.sub(a.x, b.x), simd.sub(a.y, b.y)};
}
vector2_wide_dot :: proc "contextless" (a, b: Vector2_Wide) -> F32x8
{
	return simd.add(simd.mul(a.x, b.x), simd.mul(a.y, b.y));
}
vector2_wide_scale :: proc "contextless" (v: Vector2_Wide, scale: F32x8) -> Vector2_Wide
{
	return {simd.mul(v.x, scale), simd.mul(v.y, scale)};
}
vector2_wide_negate :: proc "contextless" (v: Vector2_Wide) -> Vector2_Wide
{
	return {simd.neg(v.x), simd.neg(v.y)};
}
vector2_wide_conditional_negate :: proc "contextless" (condition: I32x8, v: Vector2_Wide) -> Vector2_Wide
{
	return {wide_select_f32(condition, simd.neg(v.x), v.x), wide_select_f32(condition, simd.neg(v.y), v.y)};
}
vector2_wide_select :: proc "contextless" (condition: I32x8, left, right: Vector2_Wide) -> Vector2_Wide
{
	return {wide_select_f32(condition, left.x, right.x), wide_select_f32(condition, left.y, right.y)};
}
vector2_wide_length_squared :: proc "contextless" (v: Vector2_Wide) -> F32x8
{
	return vector2_wide_dot(v, v);
}
vector2_wide_length :: proc "contextless" (v: Vector2_Wide) -> F32x8
{
	return simd.sqrt(vector2_wide_length_squared(v));
}
vector2_wide_perp_dot :: proc "contextless" (a, b: Vector2_Wide) -> F32x8
{
	return simd.sub(simd.mul(a.x, b.y), simd.mul(a.y, b.x));
}
vector2_wide_broadcast :: proc "contextless" (source: Vector2) -> Vector2_Wide
{
	return {F32x8(source.x), F32x8(source.y)};
}
vector2_wide_read_slot :: proc "contextless" (wide: Vector2_Wide, lane: int) -> Vector2
{
	return {simd.extract(wide.x, lane), simd.extract(wide.y, lane)};
}
vector2_wide_write_slot :: proc "contextless" (wide: ^Vector2_Wide, lane: int, source: Vector2)
{
	wide.x = simd.replace(wide.x, lane, source.x);
	wide.y = simd.replace(wide.y, lane, source.y);
}

vector3_wide_add :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {simd.add(a.x, b.x), simd.add(a.y, b.y), simd.add(a.z, b.z)};
}
vector3_wide_add_scalar :: proc "contextless" (v: Vector3_Wide, scalar: F32x8) -> Vector3_Wide
{
	return {simd.add(v.x, scalar), simd.add(v.y, scalar), simd.add(v.z, scalar)};
}
vector3_wide_subtract :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {simd.sub(a.x, b.x), simd.sub(a.y, b.y), simd.sub(a.z, b.z)};
}
vector3_wide_subtract_scalar :: proc "contextless" (v: Vector3_Wide, scalar: F32x8) -> Vector3_Wide
{
	return {simd.sub(v.x, scalar), simd.sub(v.y, scalar), simd.sub(v.z, scalar)};
}
vector3_wide_scalar_subtract :: proc "contextless" (scalar: F32x8, v: Vector3_Wide) -> Vector3_Wide
{
	return {simd.sub(scalar, v.x), simd.sub(scalar, v.y), simd.sub(scalar, v.z)};
}
vector3_wide_dot :: proc "contextless" (a, b: Vector3_Wide) -> F32x8
{
	return simd.add(simd.add(simd.mul(a.x, b.x), simd.mul(a.y, b.y)), simd.mul(a.z, b.z));
}
vector3_wide_min :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {simd.min(a.x, b.x), simd.min(a.y, b.y), simd.min(a.z, b.z)};
}
vector3_wide_max :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {simd.max(a.x, b.x), simd.max(a.y, b.y), simd.max(a.z, b.z)};
}
vector3_wide_min_scalar :: proc "contextless" (scalar: F32x8, v: Vector3_Wide) -> Vector3_Wide
{
	return {simd.min(scalar, v.x), simd.min(scalar, v.y), simd.min(scalar, v.z)};
}
vector3_wide_max_scalar :: proc "contextless" (scalar: F32x8, v: Vector3_Wide) -> Vector3_Wide
{
	return {simd.max(scalar, v.x), simd.max(scalar, v.y), simd.max(scalar, v.z)};
}
vector3_wide_scale :: proc "contextless" (v: Vector3_Wide, scale: F32x8) -> Vector3_Wide
{
	return {simd.mul(v.x, scale), simd.mul(v.y, scale), simd.mul(v.z, scale)};
}
vector3_wide_divide :: proc "contextless" (v: Vector3_Wide, divisor: F32x8) -> Vector3_Wide
{
	return {simd.div(v.x, divisor), simd.div(v.y, divisor), simd.div(v.z, divisor)};
}
vector3_wide_multiply :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {simd.mul(a.x, b.x), simd.mul(a.y, b.y), simd.mul(a.z, b.z)};
}
vector3_wide_abs :: proc "contextless" (v: Vector3_Wide) -> Vector3_Wide
{
	return {simd.abs(v.x), simd.abs(v.y), simd.abs(v.z)};
}
vector3_wide_negate :: proc "contextless" (v: Vector3_Wide) -> Vector3_Wide
{
	return {simd.neg(v.x), simd.neg(v.y), simd.neg(v.z)};
}
vector3_wide_conditional_negate :: proc "contextless" (condition: I32x8, v: Vector3_Wide) -> Vector3_Wide
{
	return {
		wide_select_f32(condition, simd.neg(v.x), v.x),
		wide_select_f32(condition, simd.neg(v.y), v.y),
		wide_select_f32(condition, simd.neg(v.z), v.z),
	};
}
vector3_wide_cross :: proc "contextless" (a, b: Vector3_Wide) -> Vector3_Wide
{
	return {
		simd.sub(simd.mul(a.y, b.z), simd.mul(a.z, b.y)),
		simd.sub(simd.mul(a.z, b.x), simd.mul(a.x, b.z)),
		simd.sub(simd.mul(a.x, b.y), simd.mul(a.y, b.x)),
	};
}
vector3_wide_length_squared :: proc "contextless" (v: Vector3_Wide) -> F32x8
{
	return vector3_wide_dot(v, v);
}
vector3_wide_length :: proc "contextless" (v: Vector3_Wide) -> F32x8
{
	return simd.sqrt(vector3_wide_length_squared(v));
}
vector3_wide_distance_squared :: proc "contextless" (a, b: Vector3_Wide) -> F32x8
{
	return vector3_wide_length_squared(vector3_wide_subtract(a, b));
}
vector3_wide_distance :: proc "contextless" (a, b: Vector3_Wide) -> F32x8
{
	return simd.sqrt(vector3_wide_distance_squared(a, b));
}
vector3_wide_normalize :: proc "contextless" (v: Vector3_Wide) -> Vector3_Wide
{
	return vector3_wide_divide(v, vector3_wide_length(v));
}
vector3_wide_select :: proc "contextless" (condition: I32x8, left, right: Vector3_Wide) -> Vector3_Wide
{
	return {
		wide_select_f32(condition, left.x, right.x),
		wide_select_f32(condition, left.y, right.y),
		wide_select_f32(condition, left.z, right.z),
	};
}
vector3_wide_broadcast :: proc "contextless" (source: Vector3) -> Vector3_Wide
{
	return {F32x8(source.x), F32x8(source.y), F32x8(source.z)};
}
vector3_wide_read_slot :: proc "contextless" (wide: Vector3_Wide, lane: int) -> Vector3
{
	return {simd.extract(wide.x, lane), simd.extract(wide.y, lane), simd.extract(wide.z, lane)};
}
vector3_wide_write_slot :: proc "contextless" (wide: ^Vector3_Wide, lane: int, source: Vector3)
{
	wide.x = simd.replace(wide.x, lane, source.x);
	wide.y = simd.replace(wide.y, lane, source.y);
	wide.z = simd.replace(wide.z, lane, source.z);
}
vector3_wide_rebroadcast :: proc "contextless" (source: Vector3_Wide, lane: int) -> Vector3_Wide
{
	return vector3_wide_broadcast(vector3_wide_read_slot(source, lane));
}
vector3_wide_copy_slot :: proc "contextless" (
	source: Vector3_Wide,
	source_lane: int,
	target: ^Vector3_Wide,
	target_lane: int
)
{
	vector3_wide_write_slot(target, target_lane, vector3_wide_read_slot(source, source_lane));
}

vector4_wide_add :: proc "contextless" (a, b: Vector4_Wide) -> Vector4_Wide
{
	return {simd.add(a.x, b.x), simd.add(a.y, b.y), simd.add(a.z, b.z), simd.add(a.w, b.w)};
}
vector4_wide_add_scalar :: proc "contextless" (v: Vector4_Wide, scalar: F32x8) -> Vector4_Wide
{
	return {simd.add(v.x, scalar), simd.add(v.y, scalar), simd.add(v.z, scalar), simd.add(v.w, scalar)};
}
vector4_wide_subtract :: proc "contextless" (a, b: Vector4_Wide) -> Vector4_Wide
{
	return {simd.sub(a.x, b.x), simd.sub(a.y, b.y), simd.sub(a.z, b.z), simd.sub(a.w, b.w)};
}
vector4_wide_dot :: proc "contextless" (a, b: Vector4_Wide) -> F32x8
{
	return simd.add(simd.add(simd.mul(a.x, b.x), simd.mul(a.y, b.y)), simd.add(simd.mul(a.z, b.z), simd.mul(a.w, b.w)));
}
vector4_wide_min :: proc "contextless" (a, b: Vector4_Wide) -> Vector4_Wide
{
	return {simd.min(a.x, b.x), simd.min(a.y, b.y), simd.min(a.z, b.z), simd.min(a.w, b.w)};
}
vector4_wide_max :: proc "contextless" (a, b: Vector4_Wide) -> Vector4_Wide
{
	return {simd.max(a.x, b.x), simd.max(a.y, b.y), simd.max(a.z, b.z), simd.max(a.w, b.w)};
}
vector4_wide_scale :: proc "contextless" (v: Vector4_Wide, scale: F32x8) -> Vector4_Wide
{
	return {simd.mul(v.x, scale), simd.mul(v.y, scale), simd.mul(v.z, scale), simd.mul(v.w, scale)};
}
vector4_wide_abs :: proc "contextless" (v: Vector4_Wide) -> Vector4_Wide
{
	return {simd.abs(v.x), simd.abs(v.y), simd.abs(v.z), simd.abs(v.w)};
}
vector4_wide_negate :: proc "contextless" (v: Vector4_Wide) -> Vector4_Wide
{
	return {simd.neg(v.x), simd.neg(v.y), simd.neg(v.z), simd.neg(v.w)};
}
vector4_wide_length_squared :: proc "contextless" (v: Vector4_Wide) -> F32x8
{
	return vector4_wide_dot(v, v);
}
vector4_wide_length :: proc "contextless" (v: Vector4_Wide) -> F32x8
{
	return simd.sqrt(vector4_wide_length_squared(v));
}
vector4_wide_distance :: proc "contextless" (a, b: Vector4_Wide) -> F32x8
{
	return vector4_wide_length(vector4_wide_subtract(a, b));
}
vector4_wide_normalize :: proc "contextless" (v: Vector4_Wide) -> Vector4_Wide
{
	return vector4_wide_scale(v, simd.div(F32x8(1), vector4_wide_length(v)));
}
vector4_wide_select :: proc "contextless" (condition: I32x8, left, right: Vector4_Wide) -> Vector4_Wide
{
	return {
		wide_select_f32(condition, left.x, right.x),
		wide_select_f32(condition, left.y, right.y),
		wide_select_f32(condition, left.z, right.z),
		wide_select_f32(condition, left.w, right.w),
	};
}
vector4_wide_broadcast :: proc "contextless" (source: Vector4) -> Vector4_Wide
{
	return {F32x8(source.x), F32x8(source.y), F32x8(source.z), F32x8(source.w)};
}
vector4_wide_read_slot :: proc "contextless" (wide: Vector4_Wide, lane: int) -> Vector4
{
	return {
		simd.extract(wide.x, lane),
		simd.extract(wide.y, lane),
		simd.extract(wide.z, lane),
		simd.extract(wide.w, lane),
	};
}
vector4_wide_write_slot :: proc "contextless" (wide: ^Vector4_Wide, lane: int, source: Vector4)
{
	wide.x = simd.replace(wide.x, lane, source.x);
	wide.y = simd.replace(wide.y, lane, source.y);
	wide.z = simd.replace(wide.z, lane, source.z);
	wide.w = simd.replace(wide.w, lane, source.w);
}
