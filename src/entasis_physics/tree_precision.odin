package entasis_physics

import "base:intrinsics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Tree_Scalar :: f64;
	Tree_Bits :: u64;
	Tree_Position :: Global_Position;
	Tree_Bounds :: Global_Bounds;
	TREE_MINIMUM_NORMAL :: f64(2.2250738585072014e-308);
	TREE_ORIGIN_MARGIN :: f64(36_028_797_018_963_968);
	TREE_SIGN_BIT :: u64(0x8000_0000_0000_0000);
	TREE_NEGATIVE_INFINITY :: u64(0xfff0_0000_0000_0000);
}
else
{
	Tree_Scalar :: f32;
	Tree_Bits :: u32;
	Tree_Position :: util.Vector3;
	Tree_Bounds :: util.Bounding_Box;
	TREE_MINIMUM_NORMAL :: f32(1.1754943508222875e-38);
	TREE_ORIGIN_MARGIN :: f32(67_108_864);
	TREE_SIGN_BIT :: u32(0x8000_0000);
	TREE_NEGATIVE_INFINITY :: u32(0xff80_0000);
}

// shape kernels continue to use f32 rays in both profiles
Tree_Ray :: struct
{
	origin: util.Vector3,
	direction: util.Vector3,
	maximum_t: f32,
}

tree_position_add :: #force_inline proc "contextless" (a, b: Tree_Position) -> Tree_Position
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

tree_position_subtract :: #force_inline proc "contextless" (a, b: Tree_Position) -> Tree_Position
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}

tree_position_scale :: #force_inline proc "contextless" (value: Tree_Position, scale: Tree_Scalar) -> Tree_Position
{
	return {value.x * scale, value.y * scale, value.z * scale};
}

tree_position_min :: #force_inline proc "contextless" (a, b: Tree_Position) -> Tree_Position
{
	return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)};
}

tree_position_max :: #force_inline proc "contextless" (a, b: Tree_Position) -> Tree_Position
{
	return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)};
}

tree_position_from_local :: #force_inline proc "contextless" (value: util.Vector3) -> Tree_Position
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		return {f64(value.x), f64(value.y), f64(value.z)};
	}
	else
	{
		return value;
	}
}

tree_bounds_from_local :: #force_inline proc "contextless" (value: util.Bounding_Box) -> Tree_Bounds
{
	return {min=tree_position_from_local(value.min), max=tree_position_from_local(value.max)};
}

tree_translate_bounds :: proc "contextless" (
	bounds: Tree_Bounds, delta: util.Vector3,
) -> (Tree_Bounds, Physics_Status)
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		translated: Global_Bounds = world_precision_translate_global_bounds(bounds, delta);
		if global_position_finite(translated.min) == .Missing || global_position_finite(translated.max) == .Missing
		{
			return {}, .Invalid_Argument;
		}
		return translated, .Ok;
	}
	else
	{
		return world_precision_translate_bounds(bounds, delta);
	}
}

tree_ray_from_local :: #force_inline proc "contextless" (ray: Tree_Ray) -> Tree_Coordinate_Ray
{
	return {origin=tree_position_from_local(ray.origin), direction=tree_position_from_local(ray.direction), maximum_t=Tree_Scalar(ray.maximum_t)};
}

tree_validate_scalar :: #force_inline proc "contextless" (value: Tree_Scalar)
{
	when util.CHECKMATH != 0
	{
		if !(abs(value) <= max(Tree_Scalar))
		{
			intrinsics.debug_trap();
		}
	}
}

tree_validate_position :: #force_inline proc "contextless" (value: Tree_Position)
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		tree_validate_scalar(value.x);
		tree_validate_scalar(value.y);
		tree_validate_scalar(value.z);
	}
	else
	{
		util.math_validate_vector3(value);
	}
}

tree_validate_bounds :: #force_inline proc "contextless" (value: Tree_Bounds)
{
	tree_validate_position(value.min);
	tree_validate_position(value.max);
}

tree_local_bounds_distance_squared :: #force_inline proc "contextless" (
	a: util.Bounding_Box, b: Tree_Bounds,
) -> f64
{
	dx: f64 = max(f64(b.min.x) - f64(a.max.x), f64(a.min.x) - f64(b.max.x), 0);
	dy: f64 = max(f64(b.min.y) - f64(a.max.y), f64(a.min.y) - f64(b.max.y), 0);
	dz: f64 = max(f64(b.min.z) - f64(a.max.z), f64(a.min.z) - f64(b.max.z), 0);
	return dx * dx + dy * dy + dz * dz;
}

tree_bounds_intersection_state :: #force_inline proc "contextless" (a, b: Tree_Bounds) -> util.Intersection_State
{
	return .Intersecting if a.min.x <= b.max.x && a.max.x >= b.min.x &&
		a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z else .Separate;
}
