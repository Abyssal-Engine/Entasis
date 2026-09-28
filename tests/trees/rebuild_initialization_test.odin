package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
rebuild_initializes_live_nodes_without_touching_unused_capacity :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 4096, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	bounds: [257]util.Bounding_Box;
	seed := u32(91827);
	for &value, index in bounds
	{
		value = random_bounds(&seed, index);
	}
	poison_node := physics.Tree_Node{
		a={min={51, 52, 53}, max={54, 55, 56}, index=2147483647, leaf_count=-77},
		b={min={61, 62, 63}, max={64, 65, 66}, index=2147483647, leaf_count=-88},
	};
	poison_meta := physics.Tree_Metanode{parent=-999, index_in_parent=-888, refine_or_cost=0xf1f2f3f4};
	for count in ([9]int{0, 1, 2, 3, 63, 64, 65, 256, 257})
	{
		for index in 0 ..< int(tree.nodes.length)
		{
			tree.nodes.memory[index] = poison_node;
		}
		for index in 0 ..< int(tree.metanodes.length)
		{
			tree.metanodes.memory[index] = poison_meta;
		}
		testing.expect_value(t, physics.tree_build(&tree, &bounds[0], count), physics.Physics_Status.Ok);
		testing.expect_value(t, tree.node_count, max(1, count - 1));
		rotation_check_leaves(t, &tree, bounds[:count]);
		testing.expect_value(t, tree.metanodes.memory[0], physics.Tree_Metanode{parent=-1, index_in_parent=-1});
		for index in 0 ..< tree.node_count
		{
			testing.expect_value(t, tree.metanodes.memory[index].refine_or_cost, u32(0));
		}
		for index in tree.node_count ..< int(tree.nodes.length)
		{
			testing.expect_value(t, tree.nodes.memory[index], poison_node);
		}
		for index in tree.node_count ..< int(tree.metanodes.length)
		{
			testing.expect_value(t, tree.metanodes.memory[index], poison_meta);
		}
		if count <= 1
		{
			testing.expect_value(t, tree.nodes.memory[0].b, physics.Tree_Node_Child{});
			if count == 0
			{
				testing.expect_value(t, tree.nodes.memory[0].a, physics.Tree_Node_Child{});
			}
		}
		// insertion must fully initialize recycled poison slots without a bulk clear
		for index in count ..< len(bounds)
		{
			leaf, status := physics.tree_add(&tree, bounds[index]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, index);
		}
		rotation_check_leaves(t, &tree, bounds[:]);
		for _ in 0 ..< len(bounds)
		{
			_, status := physics.tree_remove_at(&tree, 0);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
		testing.expect_value(t, tree.nodes.memory[0], physics.Tree_Node{});
	}
}

@(test)
rebuild_growth_shrink_and_repeated_public_build_preserve_live_bounds :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 2, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	bounds: [257]util.Bounding_Box;
	seed := u32(102311);
	for &value, index in bounds
	{
		value = random_bounds(&seed, index);
	}
	for count in ([10]int{257, 1, 0, 65, 2, 256, 3, 0, 1, 257})
	{
		testing.expect_value(t, physics.tree_build(&tree, &bounds[0], count), physics.Physics_Status.Ok);
		rotation_check_leaves(t, &tree, bounds[:count]);
		testing.expect_value(t, physics.tree_resize(&tree, max(count, 1)), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok);
		rotation_check_leaves(t, &tree, bounds[:count]);
	}
}

@(test)
rounded_leaf_storage_never_outgrows_rebuild_scratch :: proc(t: ^testing.T)
{
	Build_Path :: enum u8
	{
		Incremental, Direct
	}
	for path in ([2]Build_Path{.Incremental, .Direct})
	{
		pool: util.Buffer_Pool;
		prepare_pool(t, &pool);
		defer util.buffer_pool_dispose(&pool);
		tree: physics.Tree;
		if !testing.expect_value(t, physics.tree_initialize(&tree, 48, &pool), physics.Physics_Status.Ok)
		{
			return;
		}
		defer physics.tree_dispose(&tree);
		bounds: [1025]util.Bounding_Box;
		for &value, i in bounds
		{
			value = {min={f32(i)*6, 0, 0}, max={f32(i)*6+2, 2, 2}};
		}
		if path == .Direct
		{
			// rounded leaf storage is larger than the separately rounded rebuild
			// records. public build must grow all of them before writing its input
			count: int = min(int(tree.leaves.length), len(bounds));
			if !testing.expect_value(t, physics.tree_build(&tree, &bounds[0], count), physics.Physics_Status.Ok)
			{
				return;
			}
			rotation_check_leaves(t, &tree, bounds[:count]);
		}
		else
		{
			for value, i in bounds
			{
				status: physics.Physics_Status;
				_, status = physics.tree_add(&tree, value);
				if !testing.expect_value(t, status, physics.Physics_Status.Ok)
				{
					return;
				}
				if !testing.expect(t, int(tree.build_references.length) >= i+1 && int(tree.build_scratch.length) >= i+1)
				{
					return;
				}
			}
			rotation_check_leaves(t, &tree, bounds[:]);
		}
		if !testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok)
		{
			return;
		}
		rotation_check_leaves(t, &tree, bounds[:tree.leaf_count]);
		if !testing.expect_value(t, physics.tree_resize(&tree, tree.leaf_count), physics.Physics_Status.Ok)
		{
			return;
		}
		rotation_check_leaves(t, &tree, bounds[:tree.leaf_count]);
	}
}
