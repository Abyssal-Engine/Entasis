// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

tree_add_without_refinement_trusted :: proc "contextless" (tree: ^Tree, bounds: util.Bounding_Box) -> int
{
	leaf_index := tree.leaf_count;
	if tree.leaf_count < 2
	{
		child_index := tree.leaf_count;
		child := tree_child(&tree.nodes.memory[0], child_index);
		child^ = tree_child_from_bounds(bounds, tree_encode_leaf(leaf_index), 1);
		tree.leaves.memory[leaf_index] = tree_leaf_create(0, child_index);
		tree.leaf_count += 1;
		return leaf_index;
	}
	new_node_index := tree.node_count;
	tree.node_count += 1;
	tree.leaf_count += 1;
	node_index := 0;
	for
	{
		node := &tree.nodes.memory[node_index];
		merged_a := tree_bounds_merge(bounds, tree_child_bounds(node.a));
		merged_b := tree_bounds_merge(bounds, tree_child_bounds(node.b));
		cost_a := tree_bounds_metric(merged_a) * f32(node.a.leaf_count + 1) -
		tree_bounds_metric(tree_child_bounds(node.a)) * f32(node.a.leaf_count);
		cost_b := tree_bounds_metric(merged_b) * f32(node.b.leaf_count + 1) -
		tree_bounds_metric(tree_child_bounds(node.b)) * f32(node.b.leaf_count);
		chosen_index := 0;
		if cost_b < cost_a || cost_b == cost_a && node.b.leaf_count < node.a.leaf_count
		{
			chosen_index = 1;
		}
		chosen := tree_child(node, chosen_index);
		merged := merged_a;
		if chosen_index == 1
		{
			merged = merged_b;
		}
		if chosen.leaf_count == 1
		{
			old_child := chosen^;
			new_node := &tree.nodes.memory[new_node_index];
			new_node.a = tree_child_from_bounds(bounds, tree_encode_leaf(leaf_index), 1);
			new_node.b = old_child;
			tree.metanodes.memory[new_node_index] = {parent=i32(node_index), index_in_parent=i32(chosen_index)};
			tree.leaves.memory[leaf_index] = tree_leaf_create(new_node_index, 0);
			old_leaf_index := tree_decode_leaf(old_child.index);
			tree.leaves.memory[old_leaf_index] = tree_leaf_create(new_node_index, 1);
			chosen^ = tree_child_from_bounds(merged, i32(new_node_index), 2);
			break;
		}
		child_node_index := int(chosen.index);
		chosen.min = merged.min;
		chosen.max = merged.max;
		chosen.leaf_count += 1;
		node_index = child_node_index;
	}
	return leaf_index;
}

tree_add_without_refinement :: proc (tree: ^Tree, bounds: util.Bounding_Box) -> (int, Physics_Status)
{
	if tree == nil || tree.state != .Ready ||
	bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z
	{
		return -1, .Invalid_Argument;
	}
	// pool rounding differs between leaf and rebuild records. admit a new
	// leaf only when every live/rebuild buffer can hold it. otherwise a later
	// rebuild could write beyond scratch, or growth copy beyond its capacity
	if tree.leaf_count >= int(tree.leaves.length) ||
	tree.leaf_count >= int(tree.build_references.length) ||
	tree.leaf_count >= int(tree.build_scratch.length) ||
	tree.node_count >= int(tree.nodes.length) ||
	tree.node_count >= int(tree.metanodes.length)
	{
		capacity_status := tree_ensure_capacity(tree, max(int(tree.leaves.length) * 2, tree.leaf_count + 1));
		if capacity_status != .Ok
		{
			return -1, capacity_status;
		}
	}
	return tree_add_without_refinement_trusted(tree, bounds), .Ok;
}

tree_set_child_location :: proc "contextless" (tree: ^Tree, child: Tree_Node_Child, node_index, child_index: int)
{
	if child.index < 0
	{
		tree.leaves.memory[tree_decode_leaf(child.index)] = tree_leaf_create(node_index, child_index);
	}
	else
	{
		tree.metanodes.memory[child.index] = {parent=i32(node_index), index_in_parent=i32(child_index)};
	}
}

tree_try_rotate_node :: proc "contextless" (tree: ^Tree, rotation_root_index: int)
{
	root := &tree.nodes.memory[rotation_root_index];
	cost_a := tree_bounds_metric(tree_child_bounds(root.a)) * f32(root.a.leaf_count);
	cost_b := tree_bounds_metric(tree_child_bounds(root.b)) * f32(root.b.leaf_count);
	original_cost := cost_a + cost_b;
	left_rotation_cost_change: f32;
	left_uses_a := Reference_State.Missing;
	right_rotation_cost_change: f32;
	right_uses_a := Reference_State.Missing;
	if root.a.index >= 0
	{
		a := &tree.nodes.memory[root.a.index];
		aa_b := tree_bounds_merge(tree_child_bounds(a.a), tree_child_bounds(root.b));
		ab_b := tree_bounds_merge(tree_child_bounds(a.b), tree_child_bounds(root.b));
		cost_aa := tree_bounds_metric(tree_child_bounds(a.a)) * f32(a.a.leaf_count);
		cost_ab := tree_bounds_metric(tree_child_bounds(a.b)) * f32(a.b.leaf_count);
		cost_aab := tree_bounds_metric(aa_b) * f32(a.a.leaf_count + root.b.leaf_count) + cost_ab;
		cost_abb := tree_bounds_metric(ab_b) * f32(a.b.leaf_count + root.b.leaf_count) + cost_aa;
		if cost_aab < cost_abb
		{
			right_uses_a = .Present;
		}
		right_rotation_cost_change = min(cost_aab, cost_abb) - original_cost;
	}
	if root.b.index >= 0
	{
		b := &tree.nodes.memory[root.b.index];
		ba_a := tree_bounds_merge(tree_child_bounds(b.a), tree_child_bounds(root.a));
		bb_a := tree_bounds_merge(tree_child_bounds(b.b), tree_child_bounds(root.a));
		cost_ba := tree_bounds_metric(tree_child_bounds(b.a)) * f32(b.a.leaf_count);
		cost_bb := tree_bounds_metric(tree_child_bounds(b.b)) * f32(b.b.leaf_count);
		cost_baa := tree_bounds_metric(ba_a) * f32(b.a.leaf_count + root.a.leaf_count) + cost_bb;
		cost_bba := tree_bounds_metric(bb_a) * f32(b.b.leaf_count + root.a.leaf_count) + cost_ba;
		if cost_baa < cost_bba
		{
			left_uses_a = .Present;
		}
		left_rotation_cost_change = min(cost_baa, cost_bba) - original_cost;
	}
	if min(left_rotation_cost_change, right_rotation_cost_change) >= 0
	{
		return;
	}
	if left_rotation_cost_change < right_rotation_cost_change
	{
		node_index_to_replace := int(root.b.index);
		node_to_replace := &tree.nodes.memory[node_index_to_replace];
		child_to_shift_up := node_to_replace.a;
		child_to_shift_left := node_to_replace.b;
		if left_uses_a == .Present
		{
			child_to_shift_up = node_to_replace.b;
			child_to_shift_left = node_to_replace.a;
		}
		node_to_replace.a = root.a;
		node_to_replace.b = child_to_shift_left;
		merged := tree_bounds_merge(tree_child_bounds(node_to_replace.a), tree_child_bounds(node_to_replace.b));
		root.a = tree_child_from_bounds(
			merged, i32(node_index_to_replace), node_to_replace.a.leaf_count + node_to_replace.b.leaf_count,
		);
		root.b = child_to_shift_up;
		tree.metanodes.memory[node_index_to_replace] = {parent=i32(rotation_root_index), index_in_parent=0};
		tree_set_child_location(tree, child_to_shift_up, rotation_root_index, 1);
		tree_set_child_location(tree, node_to_replace.a, node_index_to_replace, 0);
		tree_set_child_location(tree, node_to_replace.b, node_index_to_replace, 1);
	}
	else
	{
		node_index_to_replace := int(root.a.index);
		node_to_replace := &tree.nodes.memory[node_index_to_replace];
		child_to_shift_up := node_to_replace.a;
		child_to_shift_right := node_to_replace.b;
		if right_uses_a == .Present
		{
			child_to_shift_up = node_to_replace.b;
			child_to_shift_right = node_to_replace.a;
		}
		node_to_replace.a = child_to_shift_right;
		node_to_replace.b = root.b;
		merged := tree_bounds_merge(tree_child_bounds(node_to_replace.a), tree_child_bounds(node_to_replace.b));
		root.b = tree_child_from_bounds(
			merged, i32(node_index_to_replace), node_to_replace.a.leaf_count + node_to_replace.b.leaf_count,
		);
		root.a = child_to_shift_up;
		tree.metanodes.memory[node_index_to_replace] = {parent=i32(rotation_root_index), index_in_parent=1};
		tree_set_child_location(tree, child_to_shift_up, rotation_root_index, 0);
		tree_set_child_location(tree, node_to_replace.a, node_index_to_replace, 0);
		tree_set_child_location(tree, node_to_replace.b, node_index_to_replace, 1);
	}
}

tree_add_trusted :: proc "contextless" (tree: ^Tree, bounds: util.Bounding_Box) -> int
{
	leaf_index := tree_add_without_refinement_trusted(tree, bounds);
	parent_index := tree_leaf_node_index(tree.leaves.memory[leaf_index]);
	for parent_index >= 0
	{
		tree_try_rotate_node(tree, parent_index);
		parent_index = int(tree.metanodes.memory[parent_index].parent);
	}
	return leaf_index;
}

tree_add :: proc (tree: ^Tree, bounds: util.Bounding_Box) -> (int, Physics_Status)
{
	leaf_index, status := tree_add_without_refinement(tree, bounds);
	if status != .Ok
	{
		return leaf_index, status;
	}
	parent_index := tree_leaf_node_index(tree.leaves.memory[leaf_index]);
	for parent_index >= 0
	{
		tree_try_rotate_node(tree, parent_index);
		parent_index = int(tree.metanodes.memory[parent_index].parent);
	}
	return leaf_index, .Ok;
}

tree_refit_for_node_bounds_change :: proc "contextless" (tree: ^Tree, node_index: int)
{
	current_index := node_index;
	for tree.metanodes.memory[current_index].parent >= 0
	{
		meta := tree.metanodes.memory[current_index];
		node := &tree.nodes.memory[current_index];
		parent := &tree.nodes.memory[meta.parent];
		child_in_parent := tree_child(parent, int(meta.index_in_parent));
		merged := tree_bounds_merge(tree_child_bounds(node.a), tree_child_bounds(node.b));
		child_in_parent.min = merged.min;
		child_in_parent.max = merged.max;
		child_in_parent.leaf_count = node.a.leaf_count + node.b.leaf_count;
		current_index = int(meta.parent);
	}
}

tree_update_bounds :: proc "contextless" (tree: ^Tree, leaf_index: int, bounds: util.Bounding_Box) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || leaf_index < 0 || leaf_index >= tree.leaf_count ||
	bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z
	{
		return .Invalid_Argument;
	}
	leaf := tree.leaves.memory[leaf_index];
	node_index := tree_leaf_node_index(leaf);
	child := tree_child(&tree.nodes.memory[node_index], tree_leaf_child_index(leaf));
	child.min = bounds.min;
	child.max = bounds.max;
	tree_refit_for_node_bounds_change(tree, node_index);
	return .Ok;
}

tree_refit_subtree :: proc "contextless" (
	tree: ^Tree, node_index: int,
) -> (util.Bounding_Box, i32)
{
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index >= 0
		{
			bounds, count := tree_refit_subtree(tree, int(child.index));
			child.min = bounds.min;
			child.max = bounds.max;
			child.leaf_count = count;
		}
	}
	return tree_bounds_merge(
		tree_child_bounds(node.a),
		tree_child_bounds(node.b)
	), node.a.leaf_count + node.b.leaf_count;
}

tree_refit :: proc "contextless" (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready
	{
		return .Disposed;
	}
	if tree.leaf_count > 1
	{
		_, _ = tree_refit_subtree(tree, 0);
	}
	return .Ok;
}

tree_remove_node_at :: proc "contextless" (tree: ^Tree, node_index: int)
{
	tree.node_count -= 1;
	if node_index >= tree.node_count
	{
		return;
	}
	tree.nodes.memory[node_index] = tree.nodes.memory[tree.node_count];
	tree.metanodes.memory[node_index] = tree.metanodes.memory[tree.node_count];
	meta := &tree.metanodes.memory[node_index];
	parent_child := tree_child(&tree.nodes.memory[meta.parent], int(meta.index_in_parent));
	parent_child.index = i32(node_index);
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index >= 0
		{
			tree.metanodes.memory[child.index].parent = i32(node_index);
		}
		else
		{
			tree.leaves.memory[tree_decode_leaf(child.index)] = tree_leaf_create(node_index, child_index);
		}
	}
}

tree_refit_for_removal :: proc "contextless" (tree: ^Tree, node_index: int)
{
	current_index := node_index;
	for tree.metanodes.memory[current_index].parent >= 0
	{
		meta := tree.metanodes.memory[current_index];
		node := &tree.nodes.memory[current_index];
		parent := &tree.nodes.memory[meta.parent];
		child_in_parent := tree_child(parent, int(meta.index_in_parent));
		merged := tree_bounds_merge(tree_child_bounds(node.a), tree_child_bounds(node.b));
		child_in_parent.min = merged.min;
		child_in_parent.max = merged.max;
		child_in_parent.leaf_count = node.a.leaf_count + node.b.leaf_count;
		current_index = int(meta.parent);
	}
}

tree_remove_at_trusted :: proc "contextless" (tree: ^Tree, leaf_index: int) -> Tree_Remove_Result
{
	result := Tree_Remove_Result{-1, -1};
	leaf := tree.leaves.memory[leaf_index];
	tree.leaf_count -= 1;
	if leaf_index < tree.leaf_count
	{
		last_leaf := tree.leaves.memory[tree.leaf_count];
		tree.leaves.memory[leaf_index] = last_leaf;
		moved_child := tree_child(
			&tree.nodes.memory[tree_leaf_node_index(last_leaf)], tree_leaf_child_index(last_leaf),
		);
		moved_child.index = tree_encode_leaf(leaf_index);
		result = {i32(tree.leaf_count), i32(leaf_index)};
	}
	tree.leaves.memory[tree.leaf_count] = {};
	node_index := tree_leaf_node_index(leaf);
	child_index := tree_leaf_child_index(leaf);
	node := &tree.nodes.memory[node_index];
	meta := tree.metanodes.memory[node_index];
	surviving_index := child_index ~ 1;
	surviving := tree_child(node, surviving_index)^;
	if meta.parent >= 0
	{
		child_in_parent := tree_child(&tree.nodes.memory[meta.parent], int(meta.index_in_parent));
		child_in_parent^ = surviving;
		if surviving.index < 0
		{
			tree.leaves.memory[tree_decode_leaf(surviving.index)] = tree_leaf_create(
				int(meta.parent),
				int(meta.index_in_parent)
			);
		}
		else
		{
			tree.metanodes.memory[surviving.index].parent = meta.parent;
			tree.metanodes.memory[surviving.index].index_in_parent = meta.index_in_parent;
		}
		tree_refit_for_removal(tree, int(meta.parent));
		tree_remove_node_at(tree, node_index);
	}
	else if tree.leaf_count > 0
	{
		if surviving.index >= 0
		{
			pulled_index := int(surviving.index);
			tree.nodes.memory[0] = tree.nodes.memory[pulled_index];
			tree.metanodes.memory[0] = {parent=-1, index_in_parent=-1};
			for root_child_index in 0 ..< 2
			{
				root_child := tree_child(&tree.nodes.memory[0], root_child_index);
				if root_child.index >= 0
				{
					tree.metanodes.memory[root_child.index].parent = 0;
				}
				else
				{
					tree.leaves.memory[tree_decode_leaf(root_child.index)] = tree_leaf_create(0, root_child_index);
				}
			}
			tree_remove_node_at(tree, pulled_index);
		}
		else
		{
			tree.nodes.memory[0].a = surviving;
			tree.nodes.memory[0].b = {};
			tree.leaves.memory[tree_decode_leaf(surviving.index)] = tree_leaf_create(0, 0);
		}
	}
	else
	{
		tree_initialize_root(tree);
	}
	return result;
}

tree_remove_at :: proc "contextless" (tree: ^Tree, leaf_index: int) -> (Tree_Remove_Result, Physics_Status)
{
	if tree == nil || tree.state != .Ready || leaf_index < 0 || leaf_index >= tree.leaf_count
	{
		return {-1, -1}, .Not_Found;
	}
	return tree_remove_at_trusted(tree, leaf_index), .Ok;
}
