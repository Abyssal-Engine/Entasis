// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:math"

int2_hash :: proc "contextless" (value: Int2) -> i32
{
	P1 :: u64(961748927);
	P2 :: u64(899809343);
	hash64 := u64(value.x) * (P1 * P2) + u64(value.y) * P2;
	return i32(u32(hash64 ~ (hash64 >> 32)));
}

int3_hash :: proc "contextless" (value: Int3) -> i32
{
	P1 :: u64(961748927);
	P2 :: u64(899809343);
	P3 :: u64(715225741);
	p123 := u64(P1);
	p123 *= P2;
	p123 *= P3;
	p23 := u64(P2);
	p23 *= P3;
	hash64 := u64(value.x) * p123 + u64(value.y) * p23 + u64(value.z) * P3;
	return i32(u32(hash64 ~ (hash64 >> 32)));
}

int4_hash :: proc "contextless" (value: Int4) -> i32
{
	P1 :: u64(961748927);
	P2 :: u64(899809343);
	P3 :: u64(715225741);
	P4 :: u64(472882027);
	p123 := u64(P1);
	p123 *= P2;
	p123 *= P3;
	p23 := u64(P2);
	p23 *= P3;
	hash64 := u64(value.x) * p123 + u64(value.y) * p23 + u64(value.z) * P3 + u64(value.w) * P4;
	return i32(u32(hash64 ~ (hash64 >> 32)));
}

vector2_add :: proc "contextless" (a, b: Vector2) -> Vector2
{
	return {a.x + b.x, a.y + b.y};
}
vector2_subtract :: proc "contextless" (a, b: Vector2) -> Vector2
{
	return {a.x - b.x, a.y - b.y};
}
vector2_scale :: proc "contextless" (v: Vector2, scale: f32) -> Vector2
{
	return {v.x * scale, v.y * scale};
}
vector2_dot :: proc "contextless" (a, b: Vector2) -> f32
{
	return a.x * b.x + a.y * b.y;
}
vector2_length_squared :: proc "contextless" (v: Vector2) -> f32
{
	return vector2_dot(v, v);
}
vector2_length :: proc "contextless" (v: Vector2) -> f32
{
	return math.sqrt(vector2_length_squared(v));
}

vector3_add :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}
vector3_subtract :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}
vector3_multiply :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {a.x * b.x, a.y * b.y, a.z * b.z};
}
vector3_scale :: proc "contextless" (v: Vector3, scale: f32) -> Vector3
{
	return {v.x * scale, v.y * scale, v.z * scale};
}
vector3_negate :: proc "contextless" (v: Vector3) -> Vector3
{
	return {-v.x, -v.y, -v.z};
}
vector3_abs :: proc "contextless" (v: Vector3) -> Vector3
{
	return {abs(v.x), abs(v.y), abs(v.z)};
}
vector3_min :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)};
}
vector3_max :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)};
}
vector3_dot :: proc "contextless" (a, b: Vector3) -> f32
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
vector3_cross :: proc "contextless" (a, b: Vector3) -> Vector3
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
vector3_length_squared :: proc "contextless" (v: Vector3) -> f32
{
	return vector3_dot(v, v);
}
vector3_length :: proc "contextless" (v: Vector3) -> f32
{
	return math.sqrt(vector3_length_squared(v));
}
vector3_normalize :: proc "contextless" (v: Vector3) -> Vector3
{
	return vector3_scale(v, 1.0 / vector3_length(v));
}
vector3_distance_squared :: proc "contextless" (a, b: Vector3) -> f32
{
	return vector3_length_squared(vector3_subtract(a, b));
}
vector3_distance :: proc "contextless" (a, b: Vector3) -> f32
{
	return math.sqrt(vector3_distance_squared(a, b));
}

vector4_add :: proc "contextless" (a, b: Vector4) -> Vector4
{
	return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
vector4_subtract :: proc "contextless" (a, b: Vector4) -> Vector4
{
	return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}
vector4_scale :: proc "contextless" (v: Vector4, scale: f32) -> Vector4
{
	return {v.x * scale, v.y * scale, v.z * scale, v.w * scale};
}
vector4_dot :: proc "contextless" (a, b: Vector4) -> f32
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
vector4_length_squared :: proc "contextless" (v: Vector4) -> f32
{
	return vector4_dot(v, v);
}
vector4_length :: proc "contextless" (v: Vector4) -> f32
{
	return math.sqrt(vector4_length_squared(v));
}
vector4_normalize :: proc "contextless" (v: Vector4) -> Vector4
{
	return vector4_scale(v, 1.0 / vector4_length(v));
}
