package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
streaming_sweeps_preserve_minimum_subnormal_point_axes :: proc(t: ^testing.T)
{
	when util.CHECKMATH == 0
	{
		tiny: Rounding_Scalar;
		when size_of(Rounding_Scalar) == 4
		{
			tiny = transmute(f32)u32(1);
		}
		else
		{
			tiny = transmute(f64)u64(1);
		}
		for sign in ([2]Rounding_Scalar{-1, 1})
		{
			for axis in 0 ..< 3
			{
				parallel_axis := (axis + 1) % 3;
				origin, target, direction: [3]Rounding_Scalar;
				origin[parallel_axis] = sign * tiny;
				target[parallel_axis] = sign * tiny;
				target[axis] = 1;
				direction[axis] = 1;
				position := Rounding_Vector{target[0], target[1], target[2]};
				rounding_check_routes(t, {min=position, max=position},
					{origin={origin[0], origin[1], origin[2]},
					 direction={direction[0], direction[1], direction[2]}, maximum_t=2}, 1);
			}
		}
	}
}

@(test)
streaming_sweeps_preserve_small_closed_intervals :: proc(t: ^testing.T)
{
	when util.CHECKMATH == 0
	{
		tiny: Rounding_Scalar;
		when size_of(Rounding_Scalar) == 4
		{
			tiny = transmute(f32)u32(1);
		}
		else
		{
			tiny = transmute(f64)u64(1);
		}
		pool: util.Buffer_Pool;
		testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
		defer util.buffer_pool_dispose(&pool);
		for endpoint in ([2]Rounding_Scalar{-tiny, tiny})
		{
			tree: physics.Tree;
			testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
			_, added := physics.tree_add(&tree, Rounding_Bounds{min={1, endpoint, 0}, max={1, endpoint, 0}});
			testing.expect_value(t, added, physics.Physics_Status.Ok);
			for route in 0 ..< 2
			{
				visits: Rounding_Visits;
				status: physics.Physics_Status;
				bounds := Rounding_Bounds{min={0, -tiny, 0}, max={0, tiny, 0}};
				if route == 0
				{
					_, status = physics.tree_sweep_traverse(&tree, bounds, Rounding_Vector{1, 0, 0}, 2, rounding_visit, &visits);
				}
				else
				{
					_, status = physics.tree_sweep_traverse_closest(&tree, bounds, Rounding_Vector{1, 0, 0}, 2, rounding_visit, &visits);
				}
				testing.expect_value(t, status, physics.Physics_Status.Ok);
				testing.expect_value(t, visits.count, 1);
			}
			physics.tree_dispose(&tree);
		}
	}
}

Sweep_Recenter_Selection :: struct
{
	mask: u32,
}

sweep_recenter_collect :: proc "contextless" (
	raw: rawptr, index: int, maximum_t: ^Rounding_Scalar,
) -> physics.Physics_Status
{
	selection := (^Sweep_Recenter_Selection)(raw);
	selection.mask |= u32(1) << uint(index);
	return .Ok;
}

sweep_recenter_lattice_value :: proc "contextless" (bits: u64) -> Rounding_Scalar
{
	when size_of(Rounding_Scalar) == 4
	{
		return transmute(f32)u32(bits);
	}
	else
	{
		return transmute(f64)bits;
	}
}

@(test)
streaming_sweeps_enclose_original_endpoints_across_numeric_ranges :: proc(t: ^testing.T)
{
	when util.CHECKMATH == 0
	{
		// the oracle compares original endpoints, never midpoint/radius or slab
		// arithmetic. every target is a point at x=1. all arithmetic error can
		// only affect the stationary y axis, whose exact overlap is inclusive
		bases: [5]u64;
		when size_of(Rounding_Scalar) == 4
		{
			bases = {1, 0x007f_fff8, 0x0080_0010, 0x4e6e_6b20, 0x7f7f_ffe0};
		}
		else
		{
			bases = {1, 0x000f_ffff_ffff_fff8, 0x0010_0000_0000_0010,
				0x41cd_cd65_0000_0000, 0x7fef_ffff_ffff_ffe0};
		}
		pool: util.Buffer_Pool;
		testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
		defer util.buffer_pool_dispose(&pool);
		for base in bases
		{
			for sign in ([2]Rounding_Scalar{-1, 1})
			{
				tree: physics.Tree;
				testing.expect_value(t, physics.tree_initialize(&tree, 32, &pool), physics.Physics_Status.Ok);
				values: [17]Rounding_Scalar;
				for index in 0 ..< len(values)
				{
					bits := base + u64(index);
					if sign < 0
					{
						bits = base + u64(len(values) - 1 - index);
					}
					values[index] = sign * sweep_recenter_lattice_value(bits);
					position := Rounding_Vector{1, values[index], 0};
					leaf, added := physics.tree_add(&tree, Rounding_Bounds{min=position, max=position});
					testing.expect_value(t, added, physics.Physics_Status.Ok);
					testing.expect_value(t, leaf, index);
				}
				for lower in 0 ..< len(values)
				{
					for upper in lower ..< len(values)
					{
						expected: u32;
						for index in lower ..= upper
						{
							expected |= u32(1) << uint(index);
						}
						bounds := Rounding_Bounds{min={0, values[lower], 0}, max={0, values[upper], 0}};
						for route in 0 ..< 2
						{
							selection: Sweep_Recenter_Selection;
							status: physics.Physics_Status;
							if route == 0
							{
								_, status = physics.tree_sweep_traverse(&tree, bounds, Rounding_Vector{1, 0, 0}, 2,
									sweep_recenter_collect, &selection);
							}
							else
							{
								_, status = physics.tree_sweep_traverse_closest(&tree, bounds, Rounding_Vector{1, 0, 0}, 2,
									sweep_recenter_collect, &selection);
							}
							testing.expect_value(t, status, physics.Physics_Status.Ok);
							testing.expect_value(t, selection.mask & expected, expected);
						}
					}
				}
				physics.tree_dispose(&tree);
			}
		}
	}
}

@(test)
streaming_sweeps_do_not_delay_normal_endpoint_contacts :: proc(t: ^testing.T)
{
	// three consecutive representable coordinates have an exact one-step gap.
	// translation by b-a touches c at t=1, independently of any midpoint or
	// reciprocal implementation. the t=0.5 control is still separated
	bases: [3]u64;
	when size_of(Rounding_Scalar) == 4
	{
		bases = {0x3f80_0000, 0x4e6e_6b20, 0x7f7f_ffe0};
	}
	else
	{
		bases = {0x3ff0_0000_0000_0000, 0x41cd_cd65_0000_0000, 0x7fef_ffff_ffff_ffe0};
	}
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	for base in bases
	{
		when util.CHECKMATH != 0
		{
			// the optional vector diagnostic squares inputs. keep its ordinary
			// coordinate contract. the last range runs in the other profiles
			if base == bases[len(bases) - 1]
			{
				continue;
			}
		}
		for parity in 0 ..< 2
		{
			a := sweep_recenter_lattice_value(base + u64(parity));
			b := sweep_recenter_lattice_value(base + u64(parity) + 1);
			c := sweep_recenter_lattice_value(base + u64(parity) + 2);
			for sign in ([2]Rounding_Scalar{-1, 1})
			{
				for axis in 0 ..< 3
				{
					lo, hi, target, direction: [3]Rounding_Scalar;
					lo[axis] = min(sign * a, sign * b);
					hi[axis] = max(sign * a, sign * b);
					target[axis] = sign * c;
					direction[axis] = sign * (b - a);
					point := Rounding_Vector{target[0], target[1], target[2]};
					bounds := Rounding_Bounds{min={lo[0], lo[1], lo[2]}, max={hi[0], hi[1], hi[2]}};
					velocity := Rounding_Vector{direction[0], direction[1], direction[2]};
					tree: physics.Tree;
					testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
					_, added := physics.tree_add(&tree, Rounding_Bounds{min=point, max=point});
					testing.expect_value(t, added, physics.Physics_Status.Ok);
					for limit in ([2]Rounding_Scalar{0.5, 1})
					{
						expected: int;
						if limit == 1
						{
							expected = 1;
						}
						for route in 0 ..< 2
						{
							visits: Rounding_Visits;
							status: physics.Physics_Status;
							if route == 0
							{
								_, status = physics.tree_sweep_traverse(&tree, bounds, velocity, limit, rounding_visit, &visits);
							}
							else
							{
								_, status = physics.tree_sweep_traverse_closest(&tree, bounds, velocity, limit, rounding_visit, &visits);
							}
							testing.expect_value(t, status, physics.Physics_Status.Ok);
							testing.expect_value(t, visits.count, expected);
						}
						ids: [2]i32 = {-1, -1};
						count, buffered := physics.tree_sweep_query(&tree, bounds, velocity, limit,
							util.Buffer(i32){memory=&ids[0], length=2, id=-1});
						testing.expect_value(t, buffered, physics.Physics_Status.Ok);
						testing.expect_value(t, count, expected);
					}
					physics.tree_dispose(&tree);
				}
			}
		}
	}
}

@(test)
tree_inverse_diagnostics_accept_finite_reciprocal_components :: proc(t: ^testing.T)
{
	// the reciprocal fits, but its square does not. none of these queries
	// require a vector norm. all paths describe the finite segment 0..2
	scale: Rounding_Scalar;
	when size_of(Rounding_Scalar) == 4
	{
		scale = 1e-21;
	}
	else
	{
		scale = 1e-200;
	}
	for sign in ([2]Rounding_Scalar{-1, 1})
	{
		for axis in 0 ..< 3
		{
			lo, hi, direction: [3]Rounding_Scalar;
			lo = {-1, -1, -1};
			hi = {1, 1, 1};
			lo[axis] = min(sign, 1.5 * sign);
			hi[axis] = max(sign, 1.5 * sign);
			direction[axis] = sign * scale;
			for distance in ([2]Rounding_Scalar{0.5, 2})
			{
				expected: int;
				if distance == 2
				{
					expected = 1;
				}
				rounding_check_routes(t,
					{min={lo[0], lo[1], lo[2]}, max={hi[0], hi[1], hi[2]}},
					{origin={}, direction={direction[0], direction[1], direction[2]}, maximum_t=distance / scale}, expected);
			}
		}
	}
}
