package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

rotation_check_leaves :: proc(t: ^testing.T, tree: ^physics.Tree, expected: []util.Bounding_Box)
{
	testing.expect_value(t, physics.tree_validate(tree), physics.Physics_Status.Ok);
	testing.expect_value(t, tree.leaf_count, len(expected));
	for bounds, index in expected
	{
		actual, status := physics.tree_get_leaf_bounds(tree, index);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, bounds);
	}
}

@(test)
rotation_add_remove_move_rebuild_preserves_bounds_and_query_sets :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	COUNT :: 128;
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, COUNT, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	bounds: [COUNT]util.Bounding_Box;
	state := u32(0xa03bb469);
	for index in 0 ..< COUNT
	{
		bounds[index] = random_bounds(&state, index);
		if index & 1 == 0
		{
			leaf, status := physics.tree_add(&tree, bounds[index]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, index);
		}
		else
		{
			testing.expect_value(t, physics.tree_add_trusted(&tree, bounds[index]), index);
		}
	}
	for iteration in 0 ..< 256
	{
		index := int(next_random(&state) % COUNT);
		_, status := physics.tree_remove_at(&tree, index);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		bounds[index] = bounds[COUNT - 1];
		bounds[COUNT - 1] = random_bounds(&state, iteration);
		leaf, add_status := physics.tree_add(&tree, bounds[COUNT - 1]);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, leaf, COUNT - 1);
		index = int(next_random(&state) % COUNT);
		bounds[index] = random_bounds(&state, iteration + COUNT);
		testing.expect_value(t, physics.tree_update_bounds(&tree, index, bounds[index]), physics.Physics_Status.Ok);
		if iteration % 32 == 0
		{
			testing.expect_value(t, physics.tree_rebuild_binned(&tree), physics.Physics_Status.Ok);
		}
		rotation_check_leaves(t, &tree, bounds[:]);
		query := random_bounds(&state, iteration);
		storage: [COUNT]i32;
		output := util.Buffer(i32){memory=&storage[0], length=COUNT, id=util.BUFFER_CALLER_OWNED_ID};
		count, query_status := physics.tree_query_overlaps(&tree, query, output);
		testing.expect_value(t, query_status, physics.Physics_Status.Ok);
		seen: [COUNT]bool;
		for result in storage[:count]
		{
			testing.expect(t, !seen[result]);
			seen[result] = true;
		}
		for value, model_index in bounds
		{
			testing.expect_value(t, seen[model_index], physics.tree_bounds_intersect(value, query) == .Present);
		}
	}
}

@(test)
rotation_coincident_and_sorted_insertions_do_not_form_chains :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	COUNT :: 512;
	for layout in 0 ..< 3
	{
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, COUNT, &pool), physics.Physics_Status.Ok);
		for index in 0 ..< COUNT
		{
			bounds := util.Bounding_Box{min={-1, -1, -1}, max={1, 1, 1}};
			if layout == 1
			{
				bounds.min.x += f32(index) * 3;
				bounds.max.x += f32(index) * 3;
			}
			else if layout == 2
			{
				bounds = {};
			}
			leaf, status := physics.tree_add(&tree, bounds);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, leaf, index);
		}
		testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
		for index in 0 ..< COUNT
		{
			depth := 1;
			node := physics.tree_leaf_node_index(tree.leaves.memory[index]);
			for tree.metanodes.memory[node].parent >= 0 && depth <= COUNT
			{
				depth += 1;
				node = int(tree.metanodes.memory[node].parent);
			}
			testing.expect(t, depth <= 32);
		}
		testing.expect_value(t, physics.tree_dispose(&tree), physics.Physics_Status.Ok);
	}
}
