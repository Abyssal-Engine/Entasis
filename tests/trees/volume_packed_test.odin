package trees_tests

import "core:fmt"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// independent recursive scalar traversal. it preserves A-first order and the
// exact six inclusion comparisons, including inclusive contact and unordered axes
r18_volume_reference :: proc "contextless" (
	tree: ^physics.Tree, index: i32, query: util.Bounding_Box,
) -> int
{
	if index < 0
	{
		return physics.tree_decode_leaf(index);
	}
	node := tree.nodes.memory[index];
	for child in ([2]physics.Tree_Node_Child{node.a, node.b})
	{
		if child.max.x >= query.min.x && child.min.x <= query.max.x &&
			child.max.y >= query.min.y && child.min.y <= query.max.y &&
			child.max.z >= query.min.z && child.min.z <= query.max.z
		{
			leaf := r18_volume_reference(tree, child.index, query);
			if leaf >= 0
			{
				return leaf;
			}
		}
	}
	return -1;
}

@(test)
any_volume_packed_bounds_preserve_scalar_first_leaf_and_edges :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	seed := u32(0x18_aabb);
	comparisons := 0;
	for count in ([7]int{0, 1, 2, 3, 17, 64, 257})
	{
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, 512, &pool), physics.Physics_Status.Ok);
		bounds: [257]util.Bounding_Box;
		for i in 0 ..< count
		{
			bounds[i] = random_bounds(&seed, i);
			_, status := physics.tree_add_without_refinement(&tree, bounds[i]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		for layout in 0 ..< 2
		{
			if layout == 1
			{
				testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok);
			}
			for sample in 0 ..< 512
			{
				query := random_bounds(&seed, sample);
				if count > 0 && sample < count
				{
					query = bounds[sample];
					// point queries on min/max faces retain inclusive contact
					if sample % 2 == 0
					{
						query.max = query.min;
					}
					else
					{
						query.min = query.max;
					}
				}
				if sample == 500
				{
					query = {min={0, 0, 0}, max={-1, 1, 1}};
				}
				if sample == 501
				{
					query = {min={-0.0, -0.0, -0.0}, max={0, 0, 0}};
				}
				if sample >= 502
				{
					// preserve the existing difference between the one-leaf helper
					// and multi-leaf loop for unordered axes. these tests verify behavior,
					// not support for nonfinite production physics scenes
					inf := transmute(f32)u32(0x7f800000);
					nan := transmute(f32)u32(0x7fc01234);
					query = {min={-inf, -inf, -inf}, max={inf, inf, inf}};
					if sample % 2 == 0
					{
						query.min.x = nan;
					}
					if sample % 3 == 0
					{
						query.max.y = nan;
					}
					if sample % 5 == 0
					{
						query.min.z = nan;
					}
				}
				expected := -1;
				expected_status := physics.Physics_Status.Ok;
				if query.min.x > query.max.x || query.min.y > query.max.y || query.min.z > query.max.z
				{
					expected_status = .Invalid_Argument;
				}
				else if count == 1
				{
					b := bounds[0];
					if !(b.min.x > query.max.x || b.max.x < query.min.x ||
						b.min.y > query.max.y || b.max.y < query.min.y ||
						b.min.z > query.max.z || b.max.z < query.min.z)
					{
						expected = 0;
					}
				}
				else if count > 1
				{
					expected = r18_volume_reference(&tree, 0, query);
				}
				leaf, hit, status := physics.tree_volume_any(&tree, query);
				testing.expect_value(t, status, expected_status);
				testing.expect_value(t, leaf, expected);
				testing.expect_value(t, hit == .Present, expected >= 0);
				comparisons += 1;
			}
		}
		physics.tree_dispose(&tree);
	}
	fmt.printf("R18 any-volume scalar-reference comparisons: %d\n", comparisons);
}
