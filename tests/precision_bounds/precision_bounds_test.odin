package precision_bounds_tests

import "core:fmt"
import "core:math"
import "core:mem"
import "core:os"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

LEAF_COUNT :: 64;
PAIR_CAPACITY :: LEAF_COUNT * (LEAF_COUNT - 1) / 2;

OFFSETS :: [5]physics.Global_Position{
	{}, {1e4, -1e4, 1e4}, {-1e6, 1e6, -1e6},
	{1e9, -1e9, 1e9}, {-1e9, 1e9, -1e9},
};

expect_enclosure :: proc(t: ^testing.T, global: physics.Global_Bounds, local: $Bounds)
{
	testing.expect(t, f64(local.min.x) <= global.min.x && f64(local.max.x) >= global.max.x);
	testing.expect(t, f64(local.min.y) <= global.min.y && f64(local.max.y) >= global.max.y);
	testing.expect(t, f64(local.min.z) <= global.min.z && f64(local.max.z) >= global.max.z);
}

bounds_overlap :: proc "contextless" (a, b: physics.Global_Bounds) -> physics.Reference_State
{
	if a.min.x <= b.max.x && b.min.x <= a.max.x &&
		a.min.y <= b.max.y && b.min.y <= a.max.y &&
		a.min.z <= b.max.z && b.min.z <= a.max.z
	{
		return .Present;
	}
	return .Missing;
}

fixture_bounds :: proc(
	t: ^testing.T,
	offset: physics.Global_Position,
	index, frame: int,
) -> (physics.Global_Bounds, util.Bounding_Box)
{
	// alternate 2 mm gaps and overlaps. use the actual box geometry owner,
	// placement and motion are generated in f64, never from far float positions
	shape := physics.box_bounds({0.5, 0.5, 0.5}, util.quaternion_identity());
	position := physics.Global_Position{
		x=offset.x + f64(index % 8) * 1.002 - f64(index % 2) * 0.004 + f64(frame) * 0.001,
		y=offset.y + f64(index / 8) * 1.002 + f64(frame) * 0.125,
		z=offset.z - f64(frame) * 0.03125,
	};
	global, status := physics.world_precision_bounds_from_local(position, {min=shape.min, max=shape.max});
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	bounds, narrow_status := physics.world_precision_bounds_to_float(global);
	testing.expect_value(t, narrow_status, physics.Physics_Status.Ok);
	expect_enclosure(t, global, bounds);
	return global, bounds;
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, tracker: ^mem.Tracking_Allocator)
{
	// own tracking explicitly so these checks remain valid when the runner's
	// optional context allocator tracking is disabled
	mem.tracking_allocator_init(tracker, os.heap_allocator(), os.heap_allocator());
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(pool, mem.tracking_allocator(tracker), 4096), util.Memory_Status.Ok);
}

validate_pairs :: proc(
	t: ^testing.T,
	tree: ^physics.Tree,
	global: []physics.Global_Bounds,
) -> (candidate_count, exact_count: int)
{
	pairs: [PAIR_CAPACITY]physics.Tree_Leaf_Pair;
	count, status := physics.tree_self_query(tree, {memory=&pairs[0], length=PAIR_CAPACITY, id=-1});
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	seen: [LEAF_COUNT * LEAF_COUNT]u8;
	for pair in pairs[:count]
	{
		a := min(int(pair.a), int(pair.b));
		b := max(int(pair.a), int(pair.b));
		testing.expect(t, a >= 0 && b < len(global) && a < b);
		key := a * LEAF_COUNT + b;
		testing.expect_value(t, seen[key], u8(0));
		seen[key] = 1;
	}
	for a in 0 ..< len(global)
	{
		actual, get_status := physics.tree_get_leaf_bounds(tree, a);
		testing.expect_value(t, get_status, physics.Physics_Status.Ok);
		expect_enclosure(t, global[a], actual);
		for b in a + 1 ..< len(global)
		{
			if bounds_overlap(global[a], global[b]) == .Present
			{
				exact_count += 1;
				testing.expect_value(t, seen[a * LEAF_COUNT + b], u8(1));
			}
		}
	}
	testing.expect(t, exact_count > 0 && count >= exact_count);
	testing.expect_value(t, physics.tree_validate(tree), physics.Physics_Status.Ok);
	return count, exact_count;
}

// independent slab oracle over high-precision source geometry, rather than the
// rounded tree bounds or production clipping helpers
reference_ray :: proc "contextless" (
	segment: physics.World_Precision_Segment,
	bounds: physics.Global_Bounds,
) -> (entry, exit: f64, state: physics.Reference_State)
{
	start := [3]f64{segment.start.x, segment.start.y, segment.start.z};
	end := [3]f64{segment.end.x, segment.end.y, segment.end.z};
	minimum := [3]f64{bounds.min.x, bounds.min.y, bounds.min.z};
	maximum := [3]f64{bounds.max.x, bounds.max.y, bounds.max.z};
	entry = 0;
	exit = 1;
	for axis in 0 ..< 3
	{
		direction := end[axis] - start[axis];
		if direction == 0
		{
			if start[axis] < minimum[axis] || start[axis] > maximum[axis]
			{
				return 0, 0, .Missing;
			}
			continue;
		}
		a := (minimum[axis] - start[axis]) / direction;
		b := (maximum[axis] - start[axis]) / direction;
		entry = max(entry, min(a, b));
		exit = min(exit, max(a, b));
		if entry > exit
		{
			return 0, 0, .Missing;
		}
	}
	return entry, exit, .Present;
}

validate_ray :: proc(
	t: ^testing.T,
	tree: ^physics.Tree,
	global: []physics.Global_Bounds,
	segment: physics.World_Precision_Segment,
) -> (candidate_count, exact_count: int)
{
	output: [LEAF_COUNT]physics.World_Precision_Ray_Leaf;
	stack: [LEAF_COUNT]i32;
	count, status := physics.world_precision_tree_ray_query(tree, segment, output[:], stack[:]);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	seen: [LEAF_COUNT]u8;
	intervals: [LEAF_COUNT][2]f64;
	for leaf in output[:count]
	{
		index := int(leaf.leaf_index);
		testing.expect(t, index >= 0 && index < len(global));
		testing.expect_value(t, seen[index], u8(0));
		testing.expect(t, leaf.min_t >= 0 && leaf.max_t <= 1 && leaf.min_t <= leaf.max_t);
		seen[index] = 1;
		intervals[index] = {leaf.min_t, leaf.max_t};
	}
	for bounds, index in global
	{
		entry, exit, state := reference_ray(segment, bounds);
		if state == .Present
		{
			exact_count += 1;
			testing.expect_value(t, seen[index], u8(1));
			testing.expect(t, intervals[index][0] <= entry && intervals[index][1] >= exit);
		}
	}
	testing.expect(t, count >= exact_count);
	return count, exact_count;
}

@(test)
precision_bounds_round_outward_tightly :: proc(t: ^testing.T)
{
	// walk finite binary32 exponent/sign/mantissa combinations and probe between
	// adjacent values in f64. exact endpoints must not be widened unnecessarily
	state := u32(8191);
	for index in 0 ..< 16384
	{
		state = state * 1664525 + 1013904223;
		bits := state & 0xfeffffff;
		a := transmute(f32)bits;
		b := math.nextafter_f32(a, math.F32_MAX);
		value := f64(a) + (f64(b) - f64(a)) * f64(index % 5) * 0.25;
		global := physics.Global_Bounds{min={value, -value, value}, max={value, -value, value}};
		bounds, status := physics.world_precision_bounds_to_float(global);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		expect_enclosure(t, global, bounds);
		if f64(bounds.min.x) != value
		{
			testing.expect(t, f64(math.nextafter_f32(bounds.min.x, math.F32_MAX)) > value);
		}
		if f64(bounds.max.x) != value
		{
			testing.expect(t, f64(math.nextafter_f32(bounds.max.x, -math.F32_MAX)) < value);
		}
	}
	values := [?]f64{0, -0.0, 1e-320, -1e-320, 1e-46, -1e-46, 1e4 + 0.001, -1e6 - 0.001, 1e9 + 0.002, f64(f32(math.F32_MAX)), -f64(f32(math.F32_MAX))};
	for value in values
	{
		global := physics.Global_Bounds{min={value, value, value}, max={value, value, value}};
		bounds, status := physics.world_precision_bounds_to_float(global);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		expect_enclosure(t, global, bounds);
	}
}

@(test)
precision_bounds_preserve_addition_residue :: proc(t: ^testing.T)
{
	for position in OFFSETS
	{
		zero, zero_status := physics.world_precision_bounds_from_local(position, {});
		testing.expect_value(t, zero_status, physics.Physics_Status.Ok);
		testing.expect_value(t, zero.min, position);
		testing.expect_value(t, zero.max, position);
		global, status := physics.world_precision_bounds_from_local(position, {
			min={-1e-20, -1e-20, -1e-20}, max={1e-20, 1e-20, 1e-20},
		});
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect(t, global.min.x < position.x && global.max.x > position.x);
		testing.expect(t, global.min.y < position.y && global.max.y > position.y);
		testing.expect(t, global.min.z < position.z && global.max.z > position.z);
		bounds, narrow_status := physics.world_precision_bounds_to_float(global);
		testing.expect_value(t, narrow_status, physics.Physics_Status.Ok);
		expect_enclosure(t, global, bounds);
	}
	// exact f64 addition rounds back to F32_MAX here, but its positive residue
	// means no finite float maximum can enclose the source endpoint
	global, status := physics.world_precision_bounds_from_local({f64(f32(math.F32_MAX)), 0, 0}, {max={1, 0, 0}});
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	_, narrow_status := physics.world_precision_bounds_to_float(global);
	testing.expect_value(t, narrow_status, physics.Physics_Status.Invalid_Argument);
	_, overflow_status := physics.world_precision_bounds_from_local({math.F64_MAX, 0, 0}, {max={1, 0, 0}});
	testing.expect_value(t, overflow_status, physics.Physics_Status.Invalid_Argument);
}

@(test)
precision_bounds_reject_invalid_before_publication :: proc(t: ^testing.T)
{
	invalid := [?]physics.Global_Bounds{
		{min={1, 0, 0}, max={0, 0, 0}},
		{min={0, 1, 0}, max={0, 0, 0}},
		{min={0, 0, 1}, max={0, 0, 0}},
		{min={math.nan_f64(), 0, 0}},
		{max={0, math.inf_f64(1), 0}},
		{min={0, 0, math.inf_f64(-1)}},
		{max={math.nextafter_f64(f64(f32(math.F32_MAX)), math.inf_f64(1)), 0, 0}},
		{min={-math.nextafter_f64(f64(f32(math.F32_MAX)), math.inf_f64(1)), 0, 0}},
	};
	for global in invalid
	{
		bounds, status := physics.world_precision_bounds_to_float(global);
		testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, bounds, util.Bounding_Box{});
	}
	invalid_local := [?]util.Bounding_Box{
		{min={1, 0, 0}}, {max={0, math.inf_f32(1), 0}}, {min={0, 0, math.nan_f32()}},
	};
	for local in invalid_local
	{
		global, status := physics.world_precision_bounds_from_local({}, local);
		testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, global, physics.Global_Bounds{});
	}
	_, status := physics.world_precision_bounds_from_local({math.nan_f64(), 0, 0}, {});
	testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
}

@(test)
precision_float_tree_pairs_survive_maintenance :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	for offset in OFFSETS
	{
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, LEAF_COUNT, &pool), physics.Physics_Status.Ok);
		global: [LEAF_COUNT]physics.Global_Bounds;
		for index in 0 ..< LEAF_COUNT
		{
			bounds: util.Bounding_Box;
			global[index], bounds = fixture_bounds(t, offset, index, 0);
			leaf, status := physics.tree_add(&tree, physics.tree_bounds_from_local(bounds));
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, index);
		}
		candidates, exact := validate_pairs(t, &tree, global[:]);
		fmt.printfln("mixed32-bounds offset=%.0f leaves=%d candidates=%d exact=%d false-positive=%d", offset.x, LEAF_COUNT, candidates, exact, candidates - exact);
		for frame in 1 ..= 4
		{
			for index in 0 ..< LEAF_COUNT
			{
				bounds: util.Bounding_Box;
				global[index], bounds = fixture_bounds(t, offset, index, frame);
				testing.expect_value(t, physics.tree_update_bounds(&tree, index, physics.tree_bounds_from_local(bounds)), physics.Physics_Status.Ok);
			}
			testing.expect_value(t, physics.tree_refit(&tree), physics.Physics_Status.Ok);
			testing.expect_value(t, physics.tree_refine_at_frame(&tree, u32(frame)), physics.Physics_Status.Ok);
			validate_pairs(t, &tree, global[:]);
		}
		removed, remove_status := physics.tree_remove_at(&tree, 17);
		testing.expect_value(t, remove_status, physics.Physics_Status.Ok);
		testing.expect_value(t, removed.moved_leaf_original_index, i32(LEAF_COUNT - 1));
		global[17] = global[LEAF_COUNT - 1];
		validate_pairs(t, &tree, global[:LEAF_COUNT - 1]);
		bounds: util.Bounding_Box;
		global[LEAF_COUNT - 1], bounds = fixture_bounds(t, offset, 17, 4);
		_, add_status := physics.tree_add(&tree, physics.tree_bounds_from_local(bounds));
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok);
		validate_pairs(t, &tree, global[:]);
		testing.expect_value(t, physics.tree_dispose(&tree), physics.Physics_Status.Ok);
	}
}

@(test)
precision_float_tree_intertree_and_volume_enclosure :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	for offset in OFFSETS
	{
		tree_a, tree_b: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree_a, LEAF_COUNT / 2, &pool), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_initialize(&tree_b, LEAF_COUNT / 2, &pool), physics.Physics_Status.Ok);
		global: [LEAF_COUNT]physics.Global_Bounds;
		for index in 0 ..< LEAF_COUNT
		{
			bounds: util.Bounding_Box;
			global[index], bounds = fixture_bounds(t, offset, index, 0);
			tree := &tree_a;
			if index % 2 == 1
			{
				tree = &tree_b;
			}
			_, status := physics.tree_add(tree, physics.tree_bounds_from_local(bounds));
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		pairs: [LEAF_COUNT * LEAF_COUNT / 4]physics.Tree_Leaf_Pair;
		count, status := physics.tree_intertree_query(&tree_a, &tree_b, {memory=&pairs[0], length=i32(len(pairs)), id=-1});
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		seen: [LEAF_COUNT * LEAF_COUNT / 4]u8;
		for pair in pairs[:count]
		{
			key := int(pair.a) * (LEAF_COUNT / 2) + int(pair.b);
			testing.expect_value(t, seen[key], u8(0));
			seen[key] = 1;
		}
		for a in 0 ..< LEAF_COUNT / 2
		{
			for b in 0 ..< LEAF_COUNT / 2
			{
				if bounds_overlap(global[a * 2], global[b * 2 + 1]) == .Present
				{
					testing.expect_value(t, seen[a * (LEAF_COUNT / 2) + b], u8(1));
				}
			}
			query, query_status := physics.world_precision_bounds_to_float(global[a * 2]);
			testing.expect_value(t, query_status, physics.Physics_Status.Ok);
			hits: [LEAF_COUNT / 2]i32;
			hit_count, volume_status := physics.tree_query_overlaps(&tree_b, physics.tree_bounds_from_local(query), {memory=&hits[0], length=i32(len(hits)), id=-1});
			testing.expect_value(t, volume_status, physics.Physics_Status.Ok);
			volume_seen: [LEAF_COUNT / 2]u8;
			for hit in hits[:hit_count]
			{
				volume_seen[hit] = 1;
			}
			for b in 0 ..< LEAF_COUNT / 2
			{
				if bounds_overlap(global[a * 2], global[b * 2 + 1]) == .Present
				{
					testing.expect_value(t, volume_seen[b], u8(1));
				}
			}
		}
		testing.expect_value(t, physics.tree_dispose(&tree_a), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_dispose(&tree_b), physics.Physics_Status.Ok);
	}
}

@(test)
precision_global_ray_preserves_candidates_and_intervals :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	for offset in OFFSETS
	{
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, LEAF_COUNT, &pool), physics.Physics_Status.Ok);
		global: [LEAF_COUNT]physics.Global_Bounds;
		for index in 0 ..< LEAF_COUNT
		{
			bounds: util.Bounding_Box;
			global[index], bounds = fixture_bounds(t, offset, index, 0);
			_, status := physics.tree_add(&tree, physics.tree_bounds_from_local(bounds));
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		candidate_total, exact_total: int;
		for row in 0 ..< 7
		{
			for family in 0 ..< 4
			{
				y := offset.y + f64(row) * 1.002;
				if family >= 2
				{
					y += 0.501;
				}
				segment := physics.World_Precision_Segment{
					start={offset.x - 2, y, offset.z}, end={offset.x + 10, y, offset.z},
				};
				if family % 2 == 1
				{
					segment.start, segment.end = segment.end, segment.start;
				}
				candidates, exact := validate_ray(t, &tree, global[:], segment);
				candidate_total += candidates;
				exact_total += exact;
				if family < 2
				{
					testing.expect_value(t, exact, 8);
				}
				else
				{
					testing.expect_value(t, exact, 0);
				}
			}
		}
		fmt.printfln("mixed32-ray offset=%.0f rays=28 candidates=%d exact=%d false-positive=%d", offset.x, candidate_total, exact_total, candidate_total - exact_total);
		// exact segment endpoints, origin inside a body, stationary point and
		// diagonal corner crossings all retain their original interval
		segments := [?]physics.World_Precision_Segment{
			{start=global[0].min, end=global[0].max},
			{start=global[0].min, end=global[0].min},
			{start=offset, end={offset.x + 4, offset.y, offset.z}},
			{start={offset.x - 2, offset.y - 2, offset.z - 2}, end={offset.x + 9, offset.y + 9, offset.z + 9}},
			{start={offset.x - 2, offset.y, offset.z}, end=global[0].min},
		};
		for segment in segments
		{
			validate_ray(t, &tree, global[:], segment);
		}
		testing.expect_value(t, physics.tree_dispose(&tree), physics.Physics_Status.Ok);
	}
}

@(test)
precision_global_ray_admission_and_capacities :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 2, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	segment := physics.World_Precision_Segment{start={-2, 0, 0}, end={2, 0, 0}};
	count, status := physics.world_precision_tree_ray_query(&tree, segment, nil, nil);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	_, add_status := physics.tree_add(&tree, {min={-1, -1, -1}, max={1, 1, 1}});
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	output: [2]physics.World_Precision_Ray_Leaf;
	stack: [2]i32;
	output[0] = {leaf_index=-7, min_t=-13, max_t=-17};
	sentinel := output;
	invalid := [?]physics.World_Precision_Segment{
		{start={math.nan_f64(), 0, 0}},
		{end={0, math.inf_f64(1), 0}},
		{start={-math.F64_MAX, 0, 0}, end={math.F64_MAX, 0, 0}},
		{start={-1, 0, 0}, end={math.F64_MAX, 0, 0}},
		{start={math.F64_MAX, 0, 0}, end={-1, 0, 0}},
	};
	for value in invalid
	{
		count, status = physics.world_precision_tree_ray_query(&tree, value, output[:], stack[:]);
		testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, output, sentinel);
	}
	count, status = physics.world_precision_tree_ray_query(&tree, segment, output[:], nil);
	testing.expect_value(t, status, physics.Physics_Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, output, sentinel);
	count, status = physics.world_precision_tree_ray_query(&tree, segment, nil, stack[:]);
	testing.expect_value(t, status, physics.Physics_Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	count, status = physics.world_precision_tree_ray_query(&tree, segment, output[:], stack[:]);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 1);
	testing.expect(t, output[0].min_t <= 0.25 && output[0].max_t >= 0.75);
	_, add_status = physics.tree_add(&tree, {min={-0.5, -1, -1}, max={0.5, 1, 1}});
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	count, status = physics.world_precision_tree_ray_query(&tree, segment, output[:1], stack[:]);
	testing.expect_value(t, status, physics.Physics_Status.Capacity_Missing);
	testing.expect_value(t, count, 1);
	_, status = physics.world_precision_tree_ray_query(nil, segment, output[:], stack[:]);
	testing.expect_value(t, status, physics.Physics_Status.Disposed);
}

@(test)
precision_global_ray_and_bounds_do_not_allocate :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	tracker: mem.Tracking_Allocator;
	prepare_pool(t, &pool, &tracker);
	defer mem.tracking_allocator_destroy(&tracker);
	defer testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, LEAF_COUNT, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	for index in 0 ..< LEAF_COUNT
	{
		_, bounds := fixture_bounds(t, OFFSETS[3], index, 0);
		_, status := physics.tree_add(&tree, physics.tree_bounds_from_local(bounds));
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	output: [LEAF_COUNT]physics.World_Precision_Ray_Leaf;
	stack: [LEAF_COUNT]i32;
	before := tracker.total_allocation_count;
	pool_before := util.buffer_pool_total_allocated_byte_count(&pool);
	hot_status := physics.Physics_Status.Ok;
	checksum: f64;
	for index in 0 ..< 1024
	{
		position := physics.Global_Position{1e9 + f64(index) * 0.001, -1e9, 1e9};
		global: physics.Global_Bounds;
		global, hot_status = physics.world_precision_bounds_from_local(position, {min={-0.5, -0.5, -0.5}, max={0.5, 0.5, 0.5}});
		if hot_status != .Ok
		{
			break;
		}
		bounds: util.Bounding_Box;
		bounds, hot_status = physics.world_precision_bounds_to_float(global);
		if hot_status != .Ok
		{
			break;
		}
		count: int;
		count, hot_status = physics.world_precision_tree_ray_query(&tree, {
			start={1e9 - 2, -1e9, 1e9}, end={1e9 + 10, -1e9, 1e9},
		}, output[:], stack[:]);
		if hot_status != .Ok
		{
			break;
		}
		checksum += f64(bounds.max.x) - f64(bounds.min.x) + f64(count);
	}
	after := tracker.total_allocation_count;
	pool_after := util.buffer_pool_total_allocated_byte_count(&pool);
	testing.expect_value(t, hot_status, physics.Physics_Status.Ok);
	testing.expect(t, before > 0);
	testing.expect_value(t, after, before);
	testing.expect_value(t, pool_after, pool_before);
	testing.expect(t, checksum > 0);
}
