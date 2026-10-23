package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Rounding_Scalar :: physics.Tree_Scalar;
Rounding_Bounds :: physics.Tree_Bounds;
Rounding_Vector :: physics.Tree_Position;
Rounding_Visits :: struct
{
	count: int,
}

rounding_visit :: proc "contextless" (
	raw: rawptr, index: int, maximum_t: ^Rounding_Scalar,
) -> physics.Physics_Status
{
	visits := (^Rounding_Visits)(raw);
	visits.count += 1;
	return .Ok;
}

rounding_check_routes :: proc (
	t: ^testing.T, bounds: Rounding_Bounds, ray: physics.Tree_Coordinate_Ray, expected: int,
)
{
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	_, added := physics.tree_add(&tree, bounds);
	testing.expect_value(t, added, physics.Physics_Status.Ok);
	for route in 0 ..< 4
	{
		visits: Rounding_Visits;
		status: physics.Physics_Status;
		switch route
		{
		case 0:
			_, status = physics.tree_ray_traverse(&tree, ray, rounding_visit, &visits);
		case 1:
			_, status = physics.tree_ray_traverse_closest(&tree, ray, rounding_visit, &visits);
		case 2:
			_, status = physics.tree_sweep_traverse(&tree,
				Rounding_Bounds{min=ray.origin, max=ray.origin}, ray.direction, ray.maximum_t,
				rounding_visit, &visits);
		case 3:
			_, status = physics.tree_sweep_traverse_closest(&tree,
				Rounding_Bounds{min=ray.origin, max=ray.origin}, ray.direction, ray.maximum_t,
				rounding_visit, &visits);
		}
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, visits.count, expected);
	}
	ids: [2]i32 = {-1, -1};
	output := util.Buffer(i32){memory=&ids[0], length=2, id=-1};
	count, query_status := physics.tree_ray_query(&tree, ray, output);
	testing.expect_value(t, query_status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, expected);
	count, query_status = physics.tree_sweep_query(&tree,
		Rounding_Bounds{min=ray.origin, max=ray.origin}, ray.direction, ray.maximum_t, output);
	testing.expect_value(t, query_status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, expected);
	_, scalar_hit := physics.tree_ray_intersection(bounds, ray);
	_, prepared_hit := physics.tree_ray_intersection_prepared(bounds,
		physics.tree_ray_traversal_data(ray), ray.maximum_t);
	expected_state := physics.Reference_State.Missing;
	if expected != 0
	{
		expected_state = .Present;
	}
	testing.expect_value(t, scalar_hit, expected_state);
	testing.expect_value(t, prepared_hit, expected_state);
}

@(test)
closed_corner_tangencies_survive_independent_reciprocal_rounding :: proc(t: ^testing.T)
{
	// the two clipping planes meet at exactly the same rational parameter:
	// binary32: 5/3 == 15/9, binary64: 3/5 == 9/15. no float oracle
	numerator: Rounding_Scalar = 5;
	denominator: Rounding_Scalar = 3;
	for axis in 0 ..< 3
	{
		other := (axis + 1) % 3;
		for sign in ([2]Rounding_Scalar{-1, 1})
		{
			for origin in ([3]Rounding_Scalar{0, 16, 1048576})
			{
				for gap in 0 ..< 2
				{
					lo, hi, direction: [3]Rounding_Scalar;
					lo = {origin - 1, origin - 1, origin - 1};
					hi = {origin + 1, origin + 1, origin + 1};
					lo[axis] = origin + numerator;
					hi[axis] = origin + numerator + 2;
					hi[other] = origin + 3 * numerator - Rounding_Scalar(gap);
					direction[axis] = denominator;
					direction[other] = 3 * denominator;
					if sign < 0
					{
						for i in 0 ..< 3
						{
							lo[i], hi[i] = 2 * origin - hi[i], 2 * origin - lo[i];
							direction[i] = -direction[i];
						}
					}
					rounding_check_routes(t,
						{min={lo[0], lo[1], lo[2]}, max={hi[0], hi[1], hi[2]}},
						{origin={origin, origin, origin}, direction={direction[0], direction[1], direction[2]}, maximum_t=4},
						1 - gap);
				}
			}
		}
	}
}

@(test)
finite_opposite_endpoints_do_not_overflow_before_ray_scaling :: proc(t: ^testing.T)
{
	when util.CHECKMATH == 0
	{
		// separation is twice scale, but the exact intersection parameter is 2.
		// each input is finite. no infinitely large shape or interval is authored
		scale: Rounding_Scalar = 3e38;
		for sign in ([2]Rounding_Scalar{-1, 1})
		{
			for axis in 0 ..< 3
			{
				origin, target, direction: [3]Rounding_Scalar;
				origin[axis] = -sign * scale;
				target[axis] = sign * scale;
				direction[axis] = sign * scale;
				for limit in ([2]Rounding_Scalar{1.5, 2})
				{
					position := Rounding_Vector{target[0], target[1], target[2]};
					expected: int;
					if limit == 2
					{
						expected = 1;
					}
					rounding_check_routes(t, {min=position, max=position},
						{origin={origin[0], origin[1], origin[2]}, direction={direction[0], direction[1], direction[2]}, maximum_t=limit},
						expected);
				}
			}
		}
	}
}

// integer clipping compares rational parameters by cross multiplication. all
// fixture coordinates are exactly representable. it shares no float slab math
rounding_integer_interval :: proc "contextless" (
	lo, hi, origin, direction: [3]i64,
) -> (i64, i64, physics.Reference_State)
{
	near_n: i64 = 0;
	near_d: i64 = 1;
	far_n: i64 = 8;
	far_d: i64 = 1;
	for axis in 0 ..< 3
	{
		d := direction[axis];
		if d == 0
		{
			if origin[axis] < lo[axis] || origin[axis] > hi[axis]
			{
				return 0, 1, .Missing;
			}
			continue;
		}
		n0 := lo[axis] - origin[axis];
		n1 := hi[axis] - origin[axis];
		if d < 0
		{
			n0, n1, d = -n1, -n0, -d;
		}
		if n0 * near_d > near_n * d
		{
			near_n, near_d = n0, d;
		}
		if n1 * far_d < far_n * d
		{
			far_n, far_d = n1, d;
		}
		if near_n * far_d > far_n * near_d
		{
			return 0, 1, .Missing;
		}
	}
	return near_n, near_d, .Present;
}

rounding_entry_admission :: proc "contextless" (
	value: Rounding_Scalar, numerator, denominator: i64,
) -> physics.Reference_State
{
	if value <= 0
	{
		return .Present;
	}
	// decode the returned parameter and compare its exact binary rational to
	// the integer oracle, without rounding a division or converting the oracle
	bits: physics.Tree_Bits = transmute(physics.Tree_Bits)value;
	fraction: physics.Tree_Bits = bits & ((physics.Tree_Bits(1) << ROUNDING_FRACTION_BITS) - 1);
	exponent: int = int((bits >> ROUNDING_FRACTION_BITS) & ROUNDING_EXPONENT_MASK) - ROUNDING_EXPONENT_BIAS - ROUNDING_FRACTION_BITS;
	mantissa: i128 = i128(fraction | (physics.Tree_Bits(1) << ROUNDING_FRACTION_BITS));
	lhs := mantissa * i128(denominator);
	rhs := i128(numerator);
	if exponent >= 0
	{
		lhs <<= uint(exponent);
	}
	else
	{
		rhs <<= uint(-exponent);
	}
	return .Present if lhs <= rhs else .Missing;
}

@(test)
integer_slab_oracle_bounds_candidates_and_deferred_entry_parameters :: proc(t: ^testing.T)
{
	seed: u32 = 0x32aabb;
	hits: int;
	for _ in 0 ..< 32768
	{
		lo, hi, origin, direction: [3]i64;
		for axis in 0 ..< 3
		{
			seed = seed * 1664525 + 1013904223;
			lo[axis] = i64(seed % 41) - 20;
			seed = seed * 1664525 + 1013904223;
			hi[axis] = lo[axis] + i64(seed % 13);
			seed = seed * 1664525 + 1013904223;
			origin[axis] = i64(seed % 41) - 20;
			seed = seed * 1664525 + 1013904223;
			direction[axis] = i64(seed % 33) - 16;
		}
		n, d, expected := rounding_integer_interval(lo, hi, origin, direction);
		bounds: Rounding_Bounds = {
			min={Rounding_Scalar(lo[0]), Rounding_Scalar(lo[1]), Rounding_Scalar(lo[2])},
			max={Rounding_Scalar(hi[0]), Rounding_Scalar(hi[1]), Rounding_Scalar(hi[2])},
		};
		ray: physics.Tree_Coordinate_Ray = {
			origin={Rounding_Scalar(origin[0]), Rounding_Scalar(origin[1]), Rounding_Scalar(origin[2])},
			direction={Rounding_Scalar(direction[0]), Rounding_Scalar(direction[1]), Rounding_Scalar(direction[2])},
			maximum_t=8,
		};
		entry, state := physics.tree_ray_intersection(bounds, ray);
		prepared_entry, prepared_state := physics.tree_ray_intersection_prepared(
			bounds, physics.tree_ray_traversal_data(ray), ray.maximum_t);
		testing.expect_value(t, state, expected);
		testing.expect_value(t, prepared_state, expected);
		if state == .Present && expected == .Present
		{
			hits += 1;
			testing.expect_value(t, rounding_entry_admission(entry, n, d), physics.Reference_State.Present);
			testing.expect_value(t, rounding_entry_admission(prepared_entry, n, d), physics.Reference_State.Present);
		}
	}
	testing.expect(t, hits > 100);
}

@(test)
buffered_sweeps_preserve_subnormal_point_axes :: proc(t: ^testing.T)
{
	when util.CHECKMATH == 0
	{
		pool: util.Buffer_Pool;
		testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
		defer util.buffer_pool_dispose(&pool);
		tiny := transmute(physics.Tree_Scalar)physics.Tree_Bits(1);
		for extent in ([2]Rounding_Scalar{1, 3e38})
		{
			tree: physics.Tree;
			testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
			target := Rounding_Bounds{min={extent, tiny, 0}, max={extent, tiny, 0}};
			_, status := physics.tree_add(&tree, target);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			ids: [2]i32;
			output := util.Buffer(i32){memory=&ids[0], length=2, id=-1};
			count: int;
			count, status = physics.tree_sweep_query(&tree,
				Rounding_Bounds{min={-extent, tiny, 0}, max={-extent, tiny, 0}},
				Rounding_Vector{extent, 0, 0}, 2, output);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, count, 1);
			physics.tree_dispose(&tree);
		}
	}
}

when util.PRECISION_MODE == .Double_Coordinates
{
	ROUNDING_FRACTION_BITS :: 52;
	ROUNDING_EXPONENT_MASK :: 2047;
	ROUNDING_EXPONENT_BIAS :: 1023;
}
else
{
	ROUNDING_FRACTION_BITS :: 23;
	ROUNDING_EXPONENT_MASK :: 255;
	ROUNDING_EXPONENT_BIAS :: 127;
}
