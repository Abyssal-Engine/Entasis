package trees_tests

import "core:testing"
import "core:fmt"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

R09_Query_Kind :: enum u8
{
	Ray,
	Sweep,
}

R09_Traversal_Mode :: enum u8
{
	All,
	Closest,
}

R09_Traversal_Result :: struct
{
	bounds: []physics.Tree_Bounds,
	ray: physics.Tree_Coordinate_Ray,
	expansion: physics.Tree_Position,
	closest: R09_Traversal_Mode,
	ids: [128]int,
	count: int,
}

r09_record_leaf :: proc "contextless" (
	user: rawptr, leaf: int, maximum_t: ^physics.Tree_Scalar,
) -> physics.Physics_Status
{
	result := (^R09_Traversal_Result)(user);
	result.ids[result.count] = leaf;
	result.count += 1;
	if result.closest == .Closest
	{
		bounds := result.bounds[leaf];
		bounds.min = physics.tree_position_subtract(bounds.min, result.expansion);
		bounds.max = physics.tree_position_add(bounds.max, result.expansion);
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
	maximum_t: ^physics.Tree_Scalar,
)
{
	if index < 0
	{
		_ = r09_record_leaf(result, physics.tree_decode_leaf(index), maximum_t);
		return;
	}
	node := &tree.nodes.memory[index];
	a := physics.Tree_Bounds{min=physics.tree_position_subtract(node.a.min, result.expansion), max=physics.tree_position_add(node.a.max, result.expansion)};
	b := physics.Tree_Bounds{min=physics.tree_position_subtract(node.b.min, result.expansion), max=physics.tree_position_add(node.b.max, result.expansion)};
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
		if result.closest == .All || far_t <= maximum_t^
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

// ray inputs and bounds retain the existing squared-length diagnostic domain
// reciprocal components use the production slab admission individually
r09_checkmath_admission :: proc "contextless" (
	ray: physics.Tree_Coordinate_Ray, bounds: []physics.Tree_Bounds,
) -> physics.Reference_State
{
	when util.CHECKMATH != 0
	{
		if tree_test_math_check(tree_test_position_length_squared(ray.origin)) != .Ok ||
			tree_test_math_check(tree_test_position_length_squared(ray.direction)) != .Ok
		{
			return .Missing;
		}
		prepared := physics.tree_ray_traversal_data(ray);
		if tree_test_math_check(prepared.inverse_direction.x) != .Ok ||
			tree_test_math_check(prepared.inverse_direction.y) != .Ok ||
			tree_test_math_check(prepared.inverse_direction.z) != .Ok
		{
			return .Missing;
		}
		for bound in bounds
		{
			if tree_test_math_check(tree_test_position_length_squared(bound.min)) != .Ok ||
				tree_test_math_check(tree_test_position_length_squared(bound.max)) != .Ok
			{
				return .Missing;
			}
		}
	}
	return .Present;
}

r09_compare_traversal :: proc(
	t: ^testing.T, tree: ^physics.Tree, bounds: []physics.Tree_Bounds,
	ray: physics.Tree_Coordinate_Ray, sweep: R09_Query_Kind, closest: R09_Traversal_Mode,
) -> physics.Reference_State
{
	if r09_checkmath_admission(ray, bounds) == .Missing
	{
		return .Missing;
	}
	sweep_bounds := physics.Tree_Bounds{min=physics.tree_position_subtract(ray.origin, physics.Tree_Position{0.25, 0.5, 0.75}), max=physics.tree_position_add(ray.origin, physics.Tree_Position{0.25, 0.5, 0.75})};
	oracle_ray := ray;
	expansion := physics.Tree_Position{};
	if sweep == .Sweep
	{
		// match the production sweep's original half-min/half-max arithmetic
		half_min := physics.tree_position_scale(sweep_bounds.min, 0.5);
		half_max := physics.tree_position_scale(sweep_bounds.max, 0.5);
		oracle_ray.origin = physics.tree_position_add(half_max, half_min);
		expansion = physics.tree_position_subtract(half_max, half_min);
	}
	expected := R09_Traversal_Result{bounds=bounds, ray=oracle_ray, expansion=expansion, closest=closest};
	actual := expected;
	expected_t := ray.maximum_t;
	if tree.leaf_count == 1
	{
		target := physics.Tree_Bounds{min=physics.tree_position_subtract(bounds[0].min, expansion), max=physics.tree_position_add(bounds[0].max, expansion)};
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
	actual_t: physics.Tree_Scalar;
	status: physics.Physics_Status;
	if sweep == .Sweep
	{
		if closest == .Closest
		{
			actual_t, status = physics.tree_sweep_traverse_closest(tree, sweep_bounds, ray.direction, ray.maximum_t, r09_record_leaf, &actual);
		}
		else
		{
			actual_t, status = physics.tree_sweep_traverse(tree, sweep_bounds, ray.direction, ray.maximum_t, r09_record_leaf, &actual);
		}
	}
	else if closest == .Closest
	{
		actual_t, status = physics.tree_ray_traverse_closest(tree, ray, r09_record_leaf, &actual);
	}
	else
	{
		actual_t, status = physics.tree_ray_traverse(tree, ray, r09_record_leaf, &actual);
	}
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual.count, expected.count);
	testing.expect_value(t, transmute(physics.Tree_Bits)actual_t, transmute(physics.Tree_Bits)expected_t);
	for i in 0 ..< min(actual.count, expected.count)
	{
		testing.expect_value(t, actual.ids[i], expected.ids[i]);
	}
	return .Present;
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
		bounds: [64]physics.Tree_Bounds;
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, 64, &pool), physics.Physics_Status.Ok);
		for i in 0 ..< count
		{
			seed = seed * 1664525 + 1013904223;
			center := physics.Tree_Position{physics.Tree_Scalar(i % 4) * 2, physics.Tree_Scalar((i / 4) % 4) * 2, physics.Tree_Scalar(i / 16) * 2};
			extent := physics.Tree_Scalar(seed % 16 + 1) / 16;
			bounds[i] = {min=physics.tree_position_subtract(center, physics.Tree_Position{extent, extent, extent}), max=physics.tree_position_add(center, physics.Tree_Position{extent, extent, extent})};
			leaf, status := physics.tree_add_without_refinement(&tree, bounds[i]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, i);
		}
		for i in 0 ..< 512
		{
			seed = seed * 1664525 + 1013904223;
			x := physics.Tree_Scalar(i32(seed & 1023) - 256) / 64;
			seed = seed * 1664525 + 1013904223;
			y := physics.Tree_Scalar(i32(seed & 1023) - 256) / 64;
			seed = seed * 1664525 + 1013904223;
			z := physics.Tree_Scalar(i32(seed & 1023) - 256) / 64;
			directions := [12]physics.Tree_Position{
				{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
				{1, 1e-20, -1e-20}, {-1.25, 1.01e-20, -1.01e-20},
				{1e-21, -1e-21, 0}, {0.37, -0.29, 0.83}, {-0.4, 0.9, -0.7}, {0, 0, 0},
			};
			ray := physics.Tree_Coordinate_Ray{origin={x, y, z}, direction=directions[i % len(directions)], maximum_t=physics.Tree_Scalar(i % 23)};
			for sweep in ([2]R09_Query_Kind{.Ray, .Sweep})
			{
				for closest in ([2]R09_Traversal_Mode{.All, .Closest})
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
		testing.expect_value(t, compared, 8192);
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
	limit := physics.Tree_Scalar(3.402823466e+38);
	neg_zero: physics.Tree_Scalar = -physics.Tree_Scalar(0);
	boxes := [4]physics.Tree_Bounds{
		{min={-1, -1, -1}, max={1, 1, 1}},
		{min={0, 0, 0}, max={0, 0, 0}},
		{min={-limit, -limit, -1}, max={limit, limit, 1}},
		{min={0, 0, 0}, max={limit, limit, limit}},
	};
	rays := [12]physics.Tree_Coordinate_Ray{
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
			for closest in ([2]R09_Traversal_Mode{.All, .Closest})
			{
				compared += int(r09_compare_traversal(t, &tree, {bounds}, ray, .Ray, closest));
			}
		}
	}
	when util.CHECKMATH == 0 || util.PRECISION_MODE == .Double_Coordinates
	{
		testing.expect_value(t, compared, 96);
	}
	else
	{
		testing.expect_value(t, compared, 36);
	}
	fmt.printfln("R09 boundary traversal comparisons: %d", compared);
}

tree_test_position_length_squared :: proc "contextless" (value: physics.Tree_Position) -> physics.Tree_Scalar
{
	return value.x * value.x + value.y * value.y + value.z * value.z;
}

tree_test_math_check :: proc "contextless" (value: physics.Tree_Scalar) -> util.Math_Check_Status
{
	return .Ok if abs(value) <= max(physics.Tree_Scalar) else .Non_Finite;
}
