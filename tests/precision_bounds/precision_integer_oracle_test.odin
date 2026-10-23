package precision_bounds_tests

import "core:fmt"
import "core:math"
import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// integer slab fractions compare by exact cross multiplication. the inputs
// are small enough that no product can overflow i64. this oracle never calls
// the runtime's f64 clipping or bounds producer to decide hit membership
integer_segment_reference :: proc "contextless" (
	start, end, minimum, maximum: [3]i64,
) -> (entry_n, entry_d, exit_n, exit_d: i64, state: physics.Reference_State)
{
	entry_n = 0;
	entry_d = 1;
	exit_n = 1;
	exit_d = 1;
	for axis in 0 ..< 3
	{
		direction := end[axis] - start[axis];
		if direction == 0
		{
			if start[axis] < minimum[axis] || start[axis] > maximum[axis]
			{
				return 0, 1, 0, 1, .Missing;
			}
			continue;
		}
		a := minimum[axis] - start[axis];
		b := maximum[axis] - start[axis];
		if direction < 0
		{
			direction = -direction;
			a, b = -b, -a;
		}
		if a * entry_d > entry_n * direction
		{
			entry_n = a;
			entry_d = direction;
		}
		if b * exit_d < exit_n * direction
		{
			exit_n = b;
			exit_d = direction;
		}
		if entry_n * exit_d > exit_n * entry_d
		{
			return 0, 1, 0, 1, .Missing;
		}
	}
	return entry_n, entry_d, exit_n, exit_d, .Present;
}

@(test)
precision_global_ray_matches_exact_integer_oracle :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 1, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	_, add_status := physics.tree_add(&tree, {});
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	output: [1]physics.World_Precision_Ray_Leaf;
	stack: [1]i32;
	state := u32(0x613db247);
	comparison_count, exact_hit_count, exact_miss_count: int;
	// power-of-two scaling preserves the source integers exactly in both
	// float formats, including a very small and very large finite tree cell
	exponents := [?]i32{-120, -24, 0, 24, 100};
	for exponent in exponents
	{
		scale := math.ldexp_f64(1, int(exponent));
		for index in 0 ..< 4096
		{
			minimum, maximum, start, end: [3]i64;
			for axis in 0 ..< 3
			{
				state = state * 1664525 + 1013904223;
				minimum[axis] = i64(state & 31) - 16;
				maximum[axis] = minimum[axis] + i64((state >> 8) & 15);
				state = state * 1664525 + 1013904223;
				start[axis] = i64((state >> 16) & 127) - 64;
				state = state * 1664525 + 1013904223;
				end[axis] = i64((state >> 16) & 127) - 64;
			}
			// retain zero-width boxes, endpoint touches, stationary points and
			// multi-axis ties as well as nonintersecting random segments
			switch index % 4
			{
			case 0:
				end = minimum;
			case 1:
				start = minimum;
				end = maximum;
			case 2:
				start = maximum;
				end = maximum;
			}
			bounds := util.Bounding_Box{
				min={f32(f64(minimum[0]) * scale), f32(f64(minimum[1]) * scale), f32(f64(minimum[2]) * scale)},
				max={f32(f64(maximum[0]) * scale), f32(f64(maximum[1]) * scale), f32(f64(maximum[2]) * scale)},
			};
			segment := physics.World_Precision_Segment{
				start={f64(start[0]) * scale, f64(start[1]) * scale, f64(start[2]) * scale},
				end={f64(end[0]) * scale, f64(end[1]) * scale, f64(end[2]) * scale},
			};
			testing.expect_value(t, physics.tree_update_bounds(&tree, 0, physics.tree_bounds_from_local(bounds)), physics.Physics_Status.Ok);
			entry_n, entry_d, exit_n, exit_d, expected := integer_segment_reference(start, end, minimum, maximum);
			count, status := physics.world_precision_tree_ray_query(&tree, segment, output[:], stack[:]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			if expected == .Present
			{
				exact_hit_count += 1;
				testing.expect_value(t, count, 1);
				testing.expect_value(t, output[0].leaf_index, i32(0));
				testing.expect(t, output[0].min_t <= f64(entry_n) / f64(entry_d));
				testing.expect(t, output[0].max_t >= f64(exit_n) / f64(exit_d));
			}
			else
			{
				exact_miss_count += 1;
				testing.expect_value(t, count, 0);
			}
			comparison_count += 1;
		}
	}
	testing.expect(t, exact_hit_count > 0 && exact_miss_count > 0);
	fmt.printfln("mixed32-ray integer-oracle comparisons=%d exact-hits=%d exact-misses=%d", comparison_count, exact_hit_count, exact_miss_count);
}
