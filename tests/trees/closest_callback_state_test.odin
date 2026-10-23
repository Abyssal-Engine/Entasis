package trees_tests

import "core:testing"
import "core:fmt"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

R18_Callback_State :: struct
{
	ids: [512]int,
	limits: [512]physics.Tree_Bits,
	count: int,
	policy: int,
	fail_after: int,
}

r18_stateful_leaf :: proc "contextless" (
	user: rawptr, leaf: int, maximum_t: ^physics.Tree_Scalar,
) -> physics.Physics_Status
{
	state := (^R18_Callback_State)(user);
	state.ids[state.count] = leaf;
	state.limits[state.count] = transmute(physics.Tree_Bits)maximum_t^;
	state.count += 1;
	switch state.policy
	{
	case 1:
		maximum_t^ *= 0.75;
	case 2:
		maximum_t^ = 0;
	case 3:
		maximum_t^ = min(maximum_t^, physics.Tree_Scalar(leaf % 5));
	}
	if state.count == state.fail_after
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

// recursive scalar oracle: each callback's limit update is visible to deferred
// children. errors stop the traversal immediately, with the updated limit
r18_reference_closest :: proc "contextless" (
	tree: ^physics.Tree, index: i32, prepared: physics.Tree_Ray_Traversal_Data,
	expansion: physics.Tree_Position, maximum_t: ^physics.Tree_Scalar, state: ^R18_Callback_State,
) -> physics.Physics_Status
{
	if index < 0
	{
		return r18_stateful_leaf(state, physics.tree_decode_leaf(index), maximum_t);
	}
	node := &tree.nodes.memory[index];
	a := physics.Tree_Bounds{min=physics.tree_position_subtract(node.a.min, expansion), max=physics.tree_position_add(node.a.max, expansion)};
	b := physics.Tree_Bounds{min=physics.tree_position_subtract(node.b.min, expansion), max=physics.tree_position_add(node.b.max, expansion)};
	ta, ha := physics.tree_ray_intersection_prepared(a, prepared, maximum_t^);
	tb, hb := physics.tree_ray_intersection_prepared(b, prepared, maximum_t^);
	if ha == .Present && hb == .Present
	{
		near, far, far_t := node.b.index, node.a.index, ta;
		if ta < tb
		{
			near, far, far_t = node.a.index, node.b.index, tb;
		}
		status := r18_reference_closest(tree, near, prepared, expansion, maximum_t, state);
		if status != .Ok
		{
			return status;
		}
		if far_t <= maximum_t^
		{
			return r18_reference_closest(tree, far, prepared, expansion, maximum_t, state);
		}
	}
	else if ha == .Present
	{
		return r18_reference_closest(tree, node.a.index, prepared, expansion, maximum_t, state);
	}
	else if hb == .Present
	{
		return r18_reference_closest(tree, node.b.index, prepared, expansion, maximum_t, state);
	}
	return .Ok;
}

@(test)
closest_traversal_refreshes_callback_limits_and_propagates_errors :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 16
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(&pool, 131072, power), util.Memory_Status.Ok);
	}
	defer util.buffer_pool_dispose(&pool);
	comparisons, callbacks := 0, 0;
	for count in ([6]int{0, 1, 2, 8, 32, 64})
	{
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, 64, &pool), physics.Physics_Status.Ok);
		for i in 0 ..< count
		{
			center := physics.Tree_Position{physics.Tree_Scalar(i % 8) * 0.75, physics.Tree_Scalar((i / 8) % 4) * 0.75, physics.Tree_Scalar(i / 32) * 0.75};
			extent := physics.Tree_Position{0.625, 0.625, 0.625};
			_, status := physics.tree_add_without_refinement(&tree, {min=physics.tree_position_subtract(center, extent), max=physics.tree_position_add(center, extent)});
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		for sample in 0 ..< 64
		{
			directions := [8]physics.Tree_Position{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {0.8, 0.3, -0.5}, {-0.5, 0.7, 0.4}};
			ray := physics.Tree_Coordinate_Ray{origin={physics.Tree_Scalar(sample % 8) * 0.75 - 0.5, physics.Tree_Scalar((sample / 8) % 4) * 0.75, physics.Tree_Scalar(sample / 32) * 0.75}, direction=directions[sample % len(directions)], maximum_t=16};
			for sweep_index in 0 ..< 2
			{
				expansion := physics.Tree_Position{};
				if sweep_index != 0
				{
					expansion = {0.25, 0.5, 0.75};
				}
				sweep_bounds := physics.Tree_Bounds{min=physics.tree_position_subtract(ray.origin, expansion), max=physics.tree_position_add(ray.origin, expansion)};
				for policy in 0 ..< 4
				{
					for fail_after in ([3]int{0, 1, 3})
					{
						expected := R18_Callback_State{policy=policy, fail_after=fail_after};
						actual := expected;
						limit := ray.maximum_t;
						expected_status := physics.Physics_Status.Ok;
						prepared := physics.tree_ray_traversal_data(ray);
						if count == 1
						{
							a := tree.nodes.memory[0].a;
							_, hit := physics.tree_ray_intersection_prepared({min=physics.tree_position_subtract(a.min, expansion), max=physics.tree_position_add(a.max, expansion)}, prepared, limit);
							if hit == .Present
							{
								expected_status = r18_stateful_leaf(&expected, 0, &limit);
							}
						}
						else if count > 1
						{
							expected_status = r18_reference_closest(&tree, 0, prepared, expansion, &limit, &expected);
						}
						actual_limit: physics.Tree_Scalar;
						status: physics.Physics_Status;
						if sweep_index == 0
						{
							actual_limit, status = physics.tree_ray_traverse_closest(&tree, ray, r18_stateful_leaf, &actual, &pool);
						}
						else
						{
							actual_limit, status = physics.tree_sweep_traverse_closest(&tree, sweep_bounds, ray.direction, ray.maximum_t, r18_stateful_leaf, &actual, &pool);
						}
						testing.expect_value(t, status, expected_status);
						testing.expect_value(t, transmute(physics.Tree_Bits)actual_limit, transmute(physics.Tree_Bits)limit);
						testing.expect_value(t, actual.count, expected.count);
						for i in 0 ..< min(actual.count, expected.count)
						{
							testing.expect_value(t, actual.ids[i], expected.ids[i]);
							testing.expect_value(t, actual.limits[i], expected.limits[i]);
						}
						comparisons += 1;
						callbacks += actual.count;
					}
				}
			}
		}
		physics.tree_dispose(&tree);
	}
	fmt.printf("R18 callback-state comparisons: %d; callbacks: %d\n", comparisons, callbacks);
}

@(test)
closest_ray_subtree_continuation_preserves_order_and_live_limits :: proc(t: ^testing.T)
{
	nodes: [300]physics.Tree_Node;
	metas: [300]physics.Tree_Metanode;
	leaves: [301]physics.Tree_Leaf;
	for &node, i in nodes
	{
		node.a = {min={1, -1, -1}, max={2, 1, 1}, index=physics.tree_encode_leaf(i), leaf_count=1};
		node.b = {min={1, -1, -1}, max={2, 1, 1}, index=i32(i+1), leaf_count=i32(300-i)};
		metas[i] = {parent=i32(i-1), index_in_parent=1};
		leaves[i] = physics.tree_leaf_create(i, 0);
	}
	nodes[299].b.index = physics.tree_encode_leaf(300);
	leaves[300] = physics.tree_leaf_create(299, 1);
	tree := physics.Tree{
		nodes={memory=&nodes[0], length=300, id=-1}, metanodes={memory=&metas[0], length=300, id=-1},
		leaves={memory=&leaves[0], length=301, id=-1}, node_count=300, leaf_count=301, state=.Ready,
	};
	ray := physics.Tree_Coordinate_Ray{origin={}, direction={1, 0, 0}, maximum_t=10};
	for policy in 0 ..< 4
	{
		for fail_after in ([4]int{0, 1, 2, 270})
		{
			expected := R18_Callback_State{policy=policy, fail_after=fail_after};
			actual := expected;
			limit := ray.maximum_t;
			expected_status := r18_reference_closest(&tree, 0, physics.tree_ray_traversal_data(ray), {}, &limit, &expected);
			actual_limit, status := physics.tree_ray_traverse_closest(&tree, ray, r18_stateful_leaf, &actual, nil);
			testing.expect_value(t, status, expected_status);
			testing.expect_value(t, actual_limit, limit);
			testing.expect_value(t, actual, expected);
		}
	}
}
