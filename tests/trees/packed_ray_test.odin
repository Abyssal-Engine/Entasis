package trees_tests

import "core:testing"
import "core:fmt"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

R09_Traversal_Result :: struct
{
	bounds: []util.Bounding_Box,
	ray: physics.Tree_Ray,
	expansion: util.Vector3,
	closest: bool,
	ids: [128]int,
	count: int,
}

r09_record_leaf :: proc "contextless" (
	user: rawptr, leaf: int, maximum_t: ^f32,
) -> physics.Physics_Status
{
	result := (^R09_Traversal_Result)(user);
	result.ids[result.count] = leaf;
	result.count += 1;
	if result.closest
	{
		bounds := result.bounds[leaf];
		bounds.min = util.vector3_subtract(bounds.min, result.expansion);
		bounds.max = util.vector3_add(bounds.max, result.expansion);
		t, hit := physics.tree_ray_intersection_prepared(
			bounds, physics.tree_ray_traversal_data(result.ray), maximum_t^,
		);
		if hit == .Present
		{
			maximum_t^ = t;
		}
	}
	return .Ok;
}

// independent recursive walk using the unchanged public scalar slab oracle.
// it preserves near/far ordering, including B-first ties and deferred clipping
r09_reference_node :: proc "contextless" (
	tree: ^physics.Tree, index: i32, result: ^R09_Traversal_Result,
	maximum_t: ^f32,
)
{
	if index < 0
	{
		_ = r09_record_leaf(result, physics.tree_decode_leaf(index), maximum_t);
		return;
	}
	node := &tree.nodes.memory[index];
	a := util.Bounding_Box{min=util.vector3_subtract(node.a.min, result.expansion), max=util.vector3_add(node.a.max, result.expansion)};
	b := util.Bounding_Box{min=util.vector3_subtract(node.b.min, result.expansion), max=util.vector3_add(node.b.max, result.expansion)};
	prepared := physics.tree_ray_traversal_data(result.ray);
	ta, ha := physics.tree_ray_intersection_prepared(a, prepared, maximum_t^);
	tb, hb := physics.tree_ray_intersection_prepared(b, prepared, maximum_t^);
	if ha == .Present && hb == .Present
	{
		near, far := node.b.index, node.a.index;
		far_t := ta;
		if ta < tb
		{
			near, far = node.a.index, node.b.index;
			far_t = tb;
		}
		r09_reference_node(tree, near, result, maximum_t);
		if !result.closest || far_t <= maximum_t^
		{
			r09_reference_node(tree, far, result, maximum_t);
		}
	}
	else if ha == .Present
	{
		r09_reference_node(tree, node.a.index, result, maximum_t);
	}
	else if hb == .Present
	{
		r09_reference_node(tree, node.b.index, result, maximum_t);
	}
}

// CHECKMATH validates squared vector lengths, not each component. its existing
// diagnostic domain excludes large finite coordinates and reciprocal vectors.
// the complete finite edge matrix still runs in Development, Release and ASan
r09_checkmath_accepts :: proc "contextless" (
	ray: physics.Tree_Ray, bounds: []util.Bounding_Box,
) -> bool
{
	when util.CHECKMATH != 0
	{
		if util.math_check_f32(util.vector3_length_squared(ray.origin)) != .Ok ||
			util.math_check_f32(util.vector3_length_squared(ray.direction)) != .Ok
		{
			return false;
		}
		prepared := physics.tree_ray_traversal_data(ray);
		if util.math_check_f32(util.vector3_length_squared(prepared.inverse_direction)) != .Ok
		{
			return false;
		}
		for bound in bounds
		{
			if util.math_check_f32(util.vector3_length_squared(bound.min)) != .Ok ||
				util.math_check_f32(util.vector3_length_squared(bound.max)) != .Ok
			{
				return false;
			}
		}
	}
	return true;
}

r09_compare_traversal :: proc(
	t: ^testing.T, tree: ^physics.Tree, bounds: []util.Bounding_Box,
	ray: physics.Tree_Ray, sweep: bool, closest: bool,
) -> bool
{
	if !r09_checkmath_accepts(ray, bounds)
	{
		return false;
	}
	sweep_bounds := util.Bounding_Box{min=util.vector3_subtract(ray.origin, util.Vector3{0.25, 0.5, 0.75}), max=util.vector3_add(ray.origin, util.Vector3{0.25, 0.5, 0.75})};
	oracle_ray := ray;
	expansion := util.Vector3{};
	if sweep
	{
		// match the production sweep's original half-min/half-max arithmetic
		half_min := util.vector3_scale(sweep_bounds.min, 0.5);
		half_max := util.vector3_scale(sweep_bounds.max, 0.5);
		oracle_ray.origin = util.vector3_add(half_max, half_min);
		expansion = util.vector3_subtract(half_max, half_min);
	}
	expected := R09_Traversal_Result{bounds=bounds, ray=oracle_ray, expansion=expansion, closest=closest};
	actual := expected;
	expected_t := ray.maximum_t;
	if tree.leaf_count == 1
	{
		target := util.Bounding_Box{min=util.vector3_subtract(bounds[0].min, expansion), max=util.vector3_add(bounds[0].max, expansion)};
		_, hit := physics.tree_ray_intersection_prepared(target, physics.tree_ray_traversal_data(oracle_ray), expected_t);
		if hit == .Present
		{
			_ = r09_record_leaf(&expected, 0, &expected_t);
		}
	}
	else if tree.leaf_count > 1
	{
		r09_reference_node(tree, 0, &expected, &expected_t);
	}
	actual_t: f32;
	status: physics.Physics_Status;
	if sweep
	{
		if closest
		{
			actual_t, status = physics.tree_sweep_traverse_closest(tree, sweep_bounds, ray.direction, ray.maximum_t, r09_record_leaf, &actual);
		}
		else
		{
			actual_t, status = physics.tree_sweep_traverse(tree, sweep_bounds, ray.direction, ray.maximum_t, r09_record_leaf, &actual);
		}
	}
	else if closest
	{
		actual_t, status = physics.tree_ray_traverse_closest(tree, ray, r09_record_leaf, &actual);
	}
	else
	{
		actual_t, status = physics.tree_ray_traverse(tree, ray, r09_record_leaf, &actual);
	}
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual.count, expected.count);
	testing.expect_value(t, transmute(u32)actual_t, transmute(u32)expected_t);
	for i in 0 ..< min(actual.count, expected.count)
	{
		testing.expect_value(t, actual.ids[i], expected.ids[i]);
	}
	return true;
}

@(test)
packed_ray_and_sweep_traversal_match_scalar_order_and_clipping :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 16
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(&pool, 131072, power), util.Memory_Status.Ok);
	}
	defer util.buffer_pool_dispose(&pool);
	seed := u32(0x09ca_fe42);
	compared := 0;
	for count in ([4]int{0, 1, 8, 64})
	{
		bounds: [64]util.Bounding_Box;
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, 64, &pool), physics.Physics_Status.Ok);
		for i in 0 ..< count
		{
			seed = seed * 1664525 + 1013904223;
			center := util.Vector3{f32(i % 4) * 2, f32((i / 4) % 4) * 2, f32(i / 16) * 2};
			extent := f32(seed % 16 + 1) / 16;
			bounds[i] = {min=util.vector3_subtract(center, util.Vector3{extent, extent, extent}), max=util.vector3_add(center, util.Vector3{extent, extent, extent})};
			leaf, status := physics.tree_add_without_refinement(&tree, bounds[i]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, i);
		}
		for i in 0 ..< 512
		{
			seed = seed * 1664525 + 1013904223;
			x := f32(i32(seed & 1023) - 256) / 64;
			seed = seed * 1664525 + 1013904223;
			y := f32(i32(seed & 1023) - 256) / 64;
			seed = seed * 1664525 + 1013904223;
			z := f32(i32(seed & 1023) - 256) / 64;
			directions := [12]util.Vector3{
				{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
				{1, 1e-20, -1e-20}, {-1.25, 1.01e-20, -1.01e-20},
				{1e-21, -1e-21, 0}, {0.37, -0.29, 0.83}, {-0.4, 0.9, -0.7}, {0, 0, 0},
			};
			ray := physics.Tree_Ray{origin={x, y, z}, direction=directions[i % len(directions)], maximum_t=f32(i % 23)};
			for sweep in ([2]bool{false, true})
			{
				for closest in ([2]bool{false, true})
				{
					compared += int(r09_compare_traversal(t, &tree, bounds[:count], ray, sweep, closest));
				}
			}
		}
		testing.expect_value(t, physics.tree_dispose(&tree), physics.Physics_Status.Ok);
	}
	when util.CHECKMATH == 0
	{
		testing.expect_value(t, compared, 8192);
	}
	else
	{
		testing.expect_value(t, compared, 7504);
	}
	fmt.printfln("R09 regular traversal comparisons: %d", compared);
}

@(test)
packed_slab_parallel_tangency_metadata_and_finite_extremes :: proc(t: ^testing.T)
{
	// single-leaf construction avoids area arithmetic on extreme finite bounds.
	// every native load is within a complete child, including the fourth metadata lane
	node: physics.Tree_Node;
	tree := physics.Tree{nodes={memory=&node, length=1, id=-1}, node_count=1, leaf_count=1, state=.Ready};
	limit := f32(3.402823466e+38);
	neg_zero := transmute(f32)u32(0x80000000);
	boxes := [4]util.Bounding_Box{
		{min={-1, -1, -1}, max={1, 1, 1}},
		{min={0, 0, 0}, max={0, 0, 0}},
		{min={-limit, -limit, -1}, max={limit, limit, 1}},
		{min={0, 0, 0}, max={limit, limit, limit}},
	};
	rays := [12]physics.Tree_Ray{
		{origin={1, 1, 1}, direction={0, neg_zero, 0}, maximum_t=0},
		{origin={1, 1, 1}, direction={-1, 0, 0}, maximum_t=1},
		{origin={-2, 1, 1}, direction={1, neg_zero, 0}, maximum_t=1},
		{origin={-2, 1.0001, 1}, direction={1, 0, 0}, maximum_t=10},
		{origin={0, 0, 0}, direction={0, 0, 0}, maximum_t=limit},
		{origin={limit, -limit, 0}, direction={0, neg_zero, 1}, maximum_t=2},
		{origin={limit, limit, 0}, direction={1, 0, 0}, maximum_t=limit},
		{origin={0, 0, 0}, direction={1e-20, -1e-20, 0}, maximum_t=limit},
		{origin={0, 0, 0}, direction={1.01e-20, -1.01e-20, 0}, maximum_t=limit},
		{origin={0, 0, 0}, direction={limit, -limit, 1}, maximum_t=1},
		{origin={-1, -1, -1}, direction={-0.5, 0.25, 0.75}, maximum_t=0},
		{origin={0, 0, 0}, direction={neg_zero, 0, 1}, maximum_t=0.5},
	};
	compared := 0;
	for bounds in boxes
	{
		node.a = {min=bounds.min, max=bounds.max, index=-1, leaf_count=1};
		for ray in rays
		{
			for closest in ([2]bool{false, true})
			{
				compared += int(r09_compare_traversal(t, &tree, {bounds}, ray, false, closest));
			}
		}
	}
	when util.CHECKMATH == 0
	{
		testing.expect_value(t, compared, 96);
	}
	else
	{
		testing.expect_value(t, compared, 32);
	}
	fmt.printfln("R09 boundary traversal comparisons: %d", compared);
}
