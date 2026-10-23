#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// authoritative mixed64 bounds surround the unchanged local f32 geometry
	// no absolute endpoint is narrowed before enclosure is known
	Global_Bounds :: struct
	{
		min: Global_Position,
		max: Global_Position,
	}

	// two-sum retains the rounding residue at a large position, including a local
	// bound too small to change its f64 sum. ordinary addition followed by an f32
	// cast cannot enclose that case when the rounded sum is exactly representable
	// requires IEEE round-to-nearest operations without fast-math reassociation
	world_precision_sum_residue :: #force_inline proc "contextless" (
		a, b: f64,
	) -> (sum: f64, residue: f64)
	{
		sum = a + b;
		b_virtual := sum - a;
		a_virtual := sum - b_virtual;
		residue = (a - a_virtual) + (b - b_virtual);
		return;
	}

	// input comes from application coordinates and an existing conservative local
	// AABB (including any motion/speculative expansion). validation is transactional
	world_precision_bounds_from_local :: proc "contextless" (
		position: Global_Position,
		local: util.Bounding_Box,
	) -> (Global_Bounds, Physics_Status)
	{
		if !(abs(position.x) <= math.F64_MAX && abs(position.y) <= math.F64_MAX && abs(position.z) <= math.F64_MAX) ||
			!(local.min.x <= local.max.x && local.min.y <= local.max.y && local.min.z <= local.max.z) ||
			!(abs(local.min.x) <= math.F32_MAX && abs(local.min.y) <= math.F32_MAX && abs(local.min.z) <= math.F32_MAX &&
			abs(local.max.x) <= math.F32_MAX && abs(local.max.y) <= math.F32_MAX && abs(local.max.z) <= math.F32_MAX)
		{
			return {}, .Invalid_Argument;
		}
		min_x, min_x_residue := world_precision_sum_residue(position.x, f64(local.min.x));
		min_y, min_y_residue := world_precision_sum_residue(position.y, f64(local.min.y));
		min_z, min_z_residue := world_precision_sum_residue(position.z, f64(local.min.z));
		max_x, max_x_residue := world_precision_sum_residue(position.x, f64(local.max.x));
		max_y, max_y_residue := world_precision_sum_residue(position.y, f64(local.max.y));
		max_z, max_z_residue := world_precision_sum_residue(position.z, f64(local.max.z));
		if min_x_residue < 0
		{
			min_x = math.nextafter_f64(min_x, math.inf_f64(-1));
		}
		if min_y_residue < 0
		{
			min_y = math.nextafter_f64(min_y, math.inf_f64(-1));
		}
		if min_z_residue < 0
		{
			min_z = math.nextafter_f64(min_z, math.inf_f64(-1));
		}
		if max_x_residue > 0
		{
			max_x = math.nextafter_f64(max_x, math.inf_f64(1));
		}
		if max_y_residue > 0
		{
			max_y = math.nextafter_f64(max_y, math.inf_f64(1));
		}
		if max_z_residue > 0
		{
			max_z = math.nextafter_f64(max_z, math.inf_f64(1));
		}
		result := Global_Bounds{min={min_x, min_y, min_z}, max={max_x, max_y, max_z}};
		if !(abs(result.min.x) <= math.F64_MAX && abs(result.min.y) <= math.F64_MAX && abs(result.min.z) <= math.F64_MAX &&
			abs(result.max.x) <= math.F64_MAX && abs(result.max.y) <= math.F64_MAX && abs(result.max.z) <= math.F64_MAX)
		{
			return {}, .Invalid_Argument;
		}
		return result, .Ok;
	}

	world_precision_min_to_float :: #force_inline proc "contextless" (endpoint: f64) -> f32
	{
		result := f32(endpoint);
		if f64(result) > endpoint
		{
			result = math.nextafter_f32(result, -math.F32_MAX);
		}
		return result;
	}

	world_precision_max_to_float :: #force_inline proc "contextless" (endpoint: f64) -> f32
	{
		result := f32(endpoint);
		if f64(result) < endpoint
		{
			result = math.nextafter_f32(result, math.F32_MAX);
		}
		return result;
	}

	// reject before publishing any narrowed endpoint. the finite f32 tree contract
	// does not permit using infinities as overflow or unsupported-world sentinels
	world_precision_bounds_to_float :: proc "contextless" (
		bounds: Global_Bounds,
	) -> (util.Bounding_Box, Physics_Status)
	{
		limit := f64(f32(math.F32_MAX));
		if !(bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y && bounds.min.z <= bounds.max.z) ||
			!(bounds.min.x >= -limit && bounds.min.y >= -limit && bounds.min.z >= -limit &&
			bounds.max.x <= limit && bounds.max.y <= limit && bounds.max.z <= limit)
		{
			return {}, .Invalid_Argument;
		}
		return {
			min={
				world_precision_min_to_float(bounds.min.x),
				world_precision_min_to_float(bounds.min.y),
				world_precision_min_to_float(bounds.min.z),
			},
			max={
				world_precision_max_to_float(bounds.max.x),
				world_precision_max_to_float(bounds.max.y),
				world_precision_max_to_float(bounds.max.z),
			},
		}, .Ok;
	}

	world_precision_translate_global_bounds :: proc "contextless" (
		bounds: Global_Bounds, delta: util.Vector3,
	) -> Global_Bounds
	{
		minimum: [3]f64 = {bounds.min.x, bounds.min.y, bounds.min.z};
		maximum: [3]f64 = {bounds.max.x, bounds.max.y, bounds.max.z};
		shift: [3]f64 = {-f64(delta.x), -f64(delta.y), -f64(delta.z)};
		for axis in 0 ..< 3
		{
			lower: f64;
			upper: f64;
			lower_residue: f64;
			upper_residue: f64;
			lower, lower_residue = world_precision_sum_residue(minimum[axis], shift[axis]);
			upper, upper_residue = world_precision_sum_residue(maximum[axis], shift[axis]);
			if lower_residue < 0
			{
				lower = math.nextafter_f64(lower, math.inf_f64(-1));
			}
			if upper_residue > 0
			{
				upper = math.nextafter_f64(upper, math.inf_f64(1));
			}
			minimum[axis] = lower;
			maximum[axis] = upper;
		}
		return {min={minimum[0], minimum[1], minimum[2]}, max={maximum[0], maximum[1], maximum[2]}};
	}

}
