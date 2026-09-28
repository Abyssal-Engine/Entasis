// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "base:runtime"

Tree_Bin :: struct
{
	bounds:            util.Bounding_Box,
	centroid_bounds:   util.Bounding_Box,
	count:             i32,
	leaf_count:        i32,
}

tree_empty_bounds :: proc "contextless" () -> util.Bounding_Box
{
	return {
		min={f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)},
		max={-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)},
	};
}

tree_bounds_centroid :: proc "contextless" (bounds: util.Bounding_Box) -> util.Vector3
{
	return util.vector3_scale(util.vector3_add(bounds.min, bounds.max), 0.5);
}

tree_bounds_centroid_sum :: proc "contextless" (bounds: util.Bounding_Box) -> util.Vector3
{
	return util.vector3_add(bounds.min, bounds.max);
}

tree_reference_bounds :: proc "contextless" (
	references: util.Buffer(Tree_Build_Reference), start, count: int,
) -> util.Bounding_Box
{
	bounds := references.memory[start].bounds;
	for index in start + 1 ..< start + count
	{
		bounds = tree_bounds_merge(bounds, references.memory[index].bounds);
	}
	return bounds;
}

tree_reference_axis_value :: proc "contextless" (reference: Tree_Build_Reference, axis: int) -> f32
{
	centroid := tree_bounds_centroid_sum(reference.bounds);
	if axis == 0
	{
		return centroid.x;
	}
	if axis == 1
	{
		return centroid.y;
	}
	return centroid.z;
}

tree_binned_bin_count :: proc "contextless" (subtree_count: int) -> int
{
	return min(TREE_BIN_MAX_COUNT, max(int(f32(subtree_count) * TREE_BIN_LEAF_MULTIPLIER), TREE_BIN_MIN_COUNT));
}

tree_binned_axis :: proc "contextless" (
	centroid_min,
	centroid_max: util.Vector3
) -> (axis: int, axis_min, axis_span: f32)
{
	span := util.vector3_subtract(centroid_max, centroid_min);
	if span.x > span.y && span.x > span.z
	{
		return 0, centroid_min.x, span.x;
	}
	if span.y > span.z
	{
		return 1, centroid_min.y, span.y;
	}
	return 2, centroid_min.z, span.z;
}

tree_binned_partition_references :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Build_Reference), start, count: int,
) -> int
{
	centroid_min := util.Vector3{f32(math.F32_MAX), f32(math.F32_MAX), f32(math.F32_MAX)};
	centroid_max := util.Vector3{-f32(math.F32_MAX), -f32(math.F32_MAX), -f32(math.F32_MAX)};
	for index in start ..< start + count
	{
		centroid := tree_bounds_centroid_sum(references.memory[index].bounds);
		centroid_min = util.vector3_min(centroid_min, centroid);
		centroid_max = util.vector3_max(centroid_max, centroid);
	}
	span := util.vector3_subtract(centroid_max, centroid_min);
	if span.x <= 1e-12 && span.y <= 1e-12 && span.z <= 1e-12
	{
		return count / 2;
	}
	axis, axis_min, axis_span := tree_binned_axis(centroid_min, centroid_max);
	if count <= TREE_MICROSWEEP_THRESHOLD
	{
		for index in start + 1 ..< start + count
		{
			reference := references.memory[index];
			value := tree_reference_axis_value(reference, axis);
			target := index;
			for target > start && tree_reference_axis_value(references.memory[target - 1], axis) > value
			{
				references.memory[target] = references.memory[target - 1];
				target -= 1;
			}
			references.memory[target] = reference;
		}
		prefix_bounds: [TREE_MICROSWEEP_THRESHOLD]util.Bounding_Box;
		prefix_bounds[0] = references.memory[start].bounds;
		for index in 1 ..< count
		{
			prefix_bounds[index] = tree_bounds_merge(prefix_bounds[index - 1], references.memory[start + index].bounds);
		}
		best_cost := f32(math.F32_MAX);
		best_split := 1;
		best_right_count := 0;
		right_bounds := references.memory[start + count - 1].bounds;
		right_count := 1;
		for candidate := count - 1; candidate >= 1; candidate -= 1
		{
			cost := tree_bounds_metric(prefix_bounds[candidate - 1]) * f32(count - right_count) +
			tree_bounds_metric(right_bounds) * f32(right_count);
			if cost < best_cost
			{
				best_cost = cost;
				best_split = candidate;
				best_right_count = right_count;
			}
			right_bounds = tree_bounds_merge(references.memory[start + candidate - 1].bounds, right_bounds);
			right_count += 1;
		}
		if best_right_count == 0 || best_right_count == count || best_cost == f32(math.F32_MAX) ||
		math.is_nan(best_cost) || math.is_inf(best_cost, 0)
		{
			return count / 2;
		}
		return best_split;
	}
	bin_count := tree_binned_bin_count(count);
	bins: [TREE_BIN_MAX_COUNT]Tree_Bin;
	for bin_index in 0 ..< bin_count
	{
		bins[bin_index].bounds = tree_empty_bounds();
	}
	scale := f32(bin_count) / axis_span;
	for index in start ..< start + count
	{
		reference := references.memory[index];
		bin_index := clamp(int((tree_reference_axis_value(reference, axis) - axis_min) * scale), 0, bin_count - 1);
		bin := &bins[bin_index];
		if bin.count == 0
		{
			bin.bounds = reference.bounds;
		}
		else
		{
			bin.bounds = tree_bounds_merge(bin.bounds, reference.bounds);
		}
		bin.count += 1;
	}
	prefix_bounds: [TREE_BIN_MAX_COUNT]util.Bounding_Box;
	prefix_counts: [TREE_BIN_MAX_COUNT]i32;
	accumulated_bounds := tree_empty_bounds();
	accumulated_count: i32;
	for bin_index in 0 ..< bin_count
	{
		if bins[bin_index].count > 0
		{
			if accumulated_count == 0
			{
				accumulated_bounds = bins[bin_index].bounds;
			}
			else
			{
				accumulated_bounds = tree_bounds_merge(accumulated_bounds, bins[bin_index].bounds);
			}
			accumulated_count += bins[bin_index].count;
		}
		prefix_bounds[bin_index] = accumulated_bounds;
		prefix_counts[bin_index] = accumulated_count;
	}
	best_cost := f32(math.F32_MAX);
	best_split := 1;
	best_right_count := 0;
	right_bounds := bins[bin_count - 1].bounds;
	right_count := bins[bin_count - 1].count;
	for candidate := bin_count - 1; candidate >= 1; candidate -= 1
	{
		cost := tree_bounds_metric(prefix_bounds[candidate - 1]) * f32(count - int(right_count)) +
		tree_bounds_metric(right_bounds) * f32(right_count);
		if cost < best_cost
		{
			best_cost = cost;
			best_split = candidate;
			best_right_count = int(right_count);
		}
		previous := candidate - 1;
		if bins[previous].count > 0
		{
			right_bounds = tree_bounds_merge(bins[previous].bounds, right_bounds);
		}
		right_count += bins[previous].count;
	}
	if best_right_count == 0 || best_right_count == count || best_cost == f32(math.F32_MAX) ||
	math.is_nan(best_cost) || math.is_inf(best_cost, 0)
	{
		return count / 2;
	}
	left_count, partitioned_right_count := 0, 0;
	for index in start ..< start + count
	{
		reference := references.memory[index];
		bin_index := clamp(int((tree_reference_axis_value(reference, axis) - axis_min) * scale), 0, bin_count - 1);
		if bin_index < best_split
		{
			scratch.memory[start + left_count] = reference;
			left_count += 1;
		}
		else
		{
			partitioned_right_count += 1;
			scratch.memory[start + count - partitioned_right_count] = reference;
		}
	}
	for index in start ..< start + count
	{
		references.memory[index] = scratch.memory[index];
	}
	return left_count;
}

tree_build_child :: proc "contextless" (
	tree: ^Tree, start, count, parent_node, index_in_parent: int,
) -> Tree_Node_Child
{
	bounds := tree_reference_bounds(tree.build_references, start, count);
	if count == 1
	{
		leaf_index := int(tree.build_references.memory[start].leaf_index);
		tree.leaves.memory[leaf_index] = tree_leaf_create(parent_node, index_in_parent);
		return tree_child_from_bounds(bounds, tree_encode_leaf(leaf_index), 1);
	}
	node_index := tree.node_count;
	tree.node_count += 1;
	tree.metanodes.memory[node_index] = {parent=i32(parent_node), index_in_parent=i32(index_in_parent)};
	tree_build_subtree(tree, start, count, node_index);
	return tree_child_from_bounds(bounds, i32(node_index), i32(count));
}

tree_build_subtree :: proc "contextless" (tree: ^Tree, start, count, node_index: int)
{
	left_count := tree_binned_partition_references(tree.build_references, tree.build_scratch, start, count);
	right_count := count - left_count;
	node := &tree.nodes.memory[node_index];
	node.a = tree_build_child(tree, start, left_count, node_index, 0);
	node.b = tree_build_child(tree, start + left_count, right_count, node_index, 1);
}

tree_build_from_references :: proc "contextless" (tree: ^Tree, count: int) -> Physics_Status
{
	// the builder writes both children and complete metadata for every live
	// non-root node. only the root needs clearing for empty/single-leaf trees.
	// reserved capacity is not live state and is initialized on later insertion
	tree_initialize_root(tree);
	tree.leaf_count = count;
	if count == 0
	{
		return .Ok;
	}
	if count == 1
	{
		reference := tree.build_references.memory[0];
		tree.nodes.memory[0].a = tree_child_from_bounds(
			reference.bounds,
			tree_encode_leaf(int(reference.leaf_index)),
			1
		);
		tree.leaves.memory[reference.leaf_index] = tree_leaf_create(0, 0);
		return .Ok;
	}
	tree_build_subtree(tree, 0, count, 0);
	return .Ok;
}

tree_build :: proc (tree: ^Tree, bounds: [^]util.Bounding_Box, count: int) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || bounds == nil || count < 0
	{
		return .Invalid_Argument;
	}
	if count > int(tree.leaves.length) || count > int(tree.build_references.length) ||
	count > int(tree.build_scratch.length) || max(count-1, 1) > int(tree.nodes.length) ||
	max(count-1, 1) > int(tree.metanodes.length)
	{
		capacity_status: Physics_Status = tree_ensure_capacity(tree, max(count, 1));
		if capacity_status != .Ok
		{
			return capacity_status;
		}
	}
	for index in 0 ..< count
	{
		if bounds[index].min.x > bounds[index].max.x ||
		bounds[index].min.y > bounds[index].max.y ||
		bounds[index].min.z > bounds[index].max.z
		{
			return .Invalid_Argument;
		}
		tree.build_references.memory[index] = {bounds=bounds[index], leaf_index=i32(index)};
	}
	return tree_build_from_references(tree, count);
}

tree_rebuild_binned :: proc "contextless" (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready
	{
		return .Disposed;
	}
	count := tree.leaf_count;
	for leaf_index in 0 ..< count
	{
		bounds, status := tree_get_leaf_bounds(tree, leaf_index);
		if status != .Ok
		{
			return status;
		}
		tree.build_references.memory[leaf_index] = {bounds=bounds, leaf_index=i32(leaf_index)};
	}
	return tree_build_from_references(tree, count);
}

TREE_DEFAULT_TREELET_SIZE :: 16;
TREE_REFINEMENT_SUBTREE_FLAG :: i32(1 << 30);
tree_node_child_reference_bounds :: proc "contextless" (
	references: util.Buffer(Tree_Node_Child), start, count: int,
) -> util.Bounding_Box
{
	bounds := tree_child_bounds(references.memory[start]);
	for index in start + 1 ..< start + count
	{
		bounds = tree_bounds_merge(bounds, tree_child_bounds(references.memory[index]));
	}
	return bounds;
}

tree_node_child_reference_leaf_count :: proc "contextless" (
	references: util.Buffer(Tree_Node_Child), start, count: int,
) -> i32
{
	leaf_count: i32;
	for index in start ..< start + count
	{
		leaf_count += references.memory[index].leaf_count;
	}
	return leaf_count;
}

tree_node_child_axis_value :: proc "contextless" (reference: Tree_Node_Child, axis: int) -> f32
{
	centroid := tree_bounds_centroid_sum(tree_child_bounds(reference));
	if axis == 0
	{
		return centroid.x;
	}
	if axis == 1
	{
		return centroid.y;
	}
	return centroid.z;
}

tree_node_child_centroid_bounds :: proc "contextless" (
	references: util.Buffer(Tree_Node_Child), start, count: int,
) -> util.Bounding_Box
{
	centroid := tree_bounds_centroid_sum(tree_child_bounds(references.memory[start]));
	bounds := util.Bounding_Box{min=centroid, max=centroid};
	for index in start + 1 ..< start + count
	{
		centroid = tree_bounds_centroid_sum(tree_child_bounds(references.memory[index]));
		bounds.min = util.vector3_min(bounds.min, centroid);
		bounds.max = util.vector3_max(bounds.max, centroid);
	}
	return bounds;
}

tree_binned_node_child_split_from_bins :: proc "contextless" (
	bins: ^[TREE_BIN_MAX_COUNT]Tree_Bin, bin_count, total_leaf_count: int,
) -> (
	split_index, right_leaf_count: int,
	left_centroid_bounds, right_centroid_bounds: util.Bounding_Box,
	status: Physics_Status,
)
{
	prefix_bounds: [TREE_BIN_MAX_COUNT]util.Bounding_Box;
	prefix_centroid_bounds: [TREE_BIN_MAX_COUNT]util.Bounding_Box;
	accumulated_bounds := tree_empty_bounds();
	accumulated_centroid_bounds := tree_empty_bounds();
	accumulated_leaf_count: i32;
	for bin_index in 0 ..< bin_count
	{
		bin := &bins[bin_index];
		if bin.leaf_count > 0
		{
			if accumulated_leaf_count == 0
			{
				accumulated_bounds = bin.bounds;
				accumulated_centroid_bounds = bin.centroid_bounds;
			}
			else
			{
				accumulated_bounds = tree_bounds_merge(accumulated_bounds, bin.bounds);
				accumulated_centroid_bounds = tree_bounds_merge(accumulated_centroid_bounds, bin.centroid_bounds);
			}
			accumulated_leaf_count += bin.leaf_count;
		}
		prefix_bounds[bin_index] = accumulated_bounds;
		prefix_centroid_bounds[bin_index] = accumulated_centroid_bounds;
	}
	best_cost := f32(math.F32_MAX);
	split_index = 1;
	right_bounds := bins[bin_count - 1].bounds;
	accumulated_right_centroid_bounds := bins[bin_count - 1].centroid_bounds;
	accumulated_right_leaf_count := bins[bin_count - 1].leaf_count;
	for candidate := bin_count - 1; candidate >= 1; candidate -= 1
	{
		cost := tree_bounds_metric(prefix_bounds[candidate - 1]) * f32(total_leaf_count - int(accumulated_right_leaf_count)) +
		tree_bounds_metric(right_bounds) * f32(accumulated_right_leaf_count);
		if cost < best_cost
		{
			best_cost = cost;
			split_index = candidate;
			right_leaf_count = int(accumulated_right_leaf_count);
			right_centroid_bounds = accumulated_right_centroid_bounds;
		}
		previous := candidate - 1;
		if bins[previous].leaf_count > 0
		{
			right_bounds = tree_bounds_merge(bins[previous].bounds, right_bounds);
			accumulated_right_centroid_bounds = tree_bounds_merge(
				bins[previous].centroid_bounds, accumulated_right_centroid_bounds,
			);
		}
		accumulated_right_leaf_count += bins[previous].leaf_count;
	}
	if right_leaf_count == 0 || right_leaf_count == total_leaf_count || best_cost == f32(math.F32_MAX) ||
	math.is_nan(best_cost) || math.is_inf(best_cost, 0)
	{
		return 0, 0, {}, {}, .Invalid_Description;
	}
	left_centroid_bounds = prefix_centroid_bounds[split_index - 1];
	status = .Ok;
	return;
}

tree_binned_partition_node_children_by_split :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Node_Child), start, count, axis: int,
	axis_min, axis_span: f32, bin_count, split_index: int,
) -> int
{
	scale := f32(bin_count) / axis_span;
	left_count, right_count := 0, 0;
	for index in start ..< start + count
	{
		reference := references.memory[index];
		bin_index := clamp(int((tree_node_child_axis_value(reference, axis) - axis_min) * scale), 0, bin_count - 1);
		if bin_index < split_index
		{
			scratch.memory[start + left_count] = reference;
			left_count += 1;
		}
		else
		{
			right_count += 1;
			scratch.memory[start + count - right_count] = reference;
		}
	}
	for index in start ..< start + count
	{
		references.memory[index] = scratch.memory[index];
	}
	return left_count;
}

tree_binned_partition_node_children :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Node_Child), start, count: int,
) -> int
{
	centroid_bounds := tree_node_child_centroid_bounds(references, start, count);
	span := util.vector3_subtract(centroid_bounds.max, centroid_bounds.min);
	if span.x <= 1e-12 && span.y <= 1e-12 && span.z <= 1e-12
	{
		return count / 2;
	}
	axis, axis_min, axis_span := tree_binned_axis(centroid_bounds.min, centroid_bounds.max);
	total_leaf_count := int(tree_node_child_reference_leaf_count(references, start, count));
	if count <= TREE_MICROSWEEP_THRESHOLD
	{
		for index in start + 1 ..< start + count
		{
			reference := references.memory[index];
			value := tree_node_child_axis_value(reference, axis);
			target := index;
			for target > start && tree_node_child_axis_value(references.memory[target - 1], axis) > value
			{
				references.memory[target] = references.memory[target - 1];
				target -= 1;
			}
			references.memory[target] = reference;
		}
		prefix_bounds: [TREE_MICROSWEEP_THRESHOLD]util.Bounding_Box;
		prefix_leaf_counts: [TREE_MICROSWEEP_THRESHOLD]i32;
		prefix_bounds[0] = tree_child_bounds(references.memory[start]);
		prefix_leaf_counts[0] = references.memory[start].leaf_count;
		for index in 1 ..< count
		{
			prefix_bounds[index] = tree_bounds_merge(
				prefix_bounds[index - 1],
				tree_child_bounds(references.memory[start + index])
			);
			prefix_leaf_counts[index] = prefix_leaf_counts[index - 1] + references.memory[start + index].leaf_count;
		}
		best_cost := f32(math.F32_MAX);
		best_split := 1;
		best_right_leaf_count := 0;
		right_bounds := tree_child_bounds(references.memory[start + count - 1]);
		right_leaf_count := references.memory[start + count - 1].leaf_count;
		for candidate := count - 1; candidate >= 1; candidate -= 1
		{
			cost := tree_bounds_metric(prefix_bounds[candidate - 1]) * f32(total_leaf_count - int(right_leaf_count)) +
			tree_bounds_metric(right_bounds) * f32(right_leaf_count);
			if cost < best_cost
			{
				best_cost = cost;
				best_split = candidate;
				best_right_leaf_count = int(right_leaf_count);
			}
			right_bounds = tree_bounds_merge(tree_child_bounds(references.memory[start + candidate - 1]), right_bounds);
			right_leaf_count += references.memory[start + candidate - 1].leaf_count;
		}
		if best_right_leaf_count == 0 || best_right_leaf_count == total_leaf_count || best_cost == f32(math.F32_MAX) ||
		math.is_nan(best_cost) || math.is_inf(best_cost, 0)
		{
			return count / 2;
		}
		return best_split;
	}
	bin_count := tree_binned_bin_count(count);
	bins: [TREE_BIN_MAX_COUNT]Tree_Bin;
	for bin_index in 0 ..< bin_count
	{
		bins[bin_index].bounds = tree_empty_bounds();
		bins[bin_index].centroid_bounds = tree_empty_bounds();
	}
	scale := f32(bin_count) / axis_span;
	for index in start ..< start + count
	{
		reference := references.memory[index];
		bin_index := clamp(int((tree_node_child_axis_value(reference, axis) - axis_min) * scale), 0, bin_count - 1);
		bin := &bins[bin_index];
		bounds := tree_child_bounds(reference);
		centroid := tree_bounds_centroid_sum(bounds);
		if bin.count == 0
		{
			bin.bounds = bounds;
			bin.centroid_bounds = {min=centroid, max=centroid};
		}
		else
		{
			bin.bounds = tree_bounds_merge(bin.bounds, bounds);
			bin.centroid_bounds.min = util.vector3_min(bin.centroid_bounds.min, centroid);
			bin.centroid_bounds.max = util.vector3_max(bin.centroid_bounds.max, centroid);
		}
		bin.count += 1;
		bin.leaf_count += reference.leaf_count;
	}
	best_split, _, _, _, split_status := tree_binned_node_child_split_from_bins(&bins, bin_count, total_leaf_count);
	if split_status != .Ok
	{
		return count / 2;
	}
	return tree_binned_partition_node_children_by_split(
		references, scratch, start, count, axis, axis_min, axis_span, bin_count, best_split,
	);
}

tree_temporary_write_node_children :: proc "contextless" (
	references: util.Buffer(Tree_Node_Child), nodes: util.Buffer(Tree_Node),
	node_index, start, count, left_count: int,
)
{
	counts := [2]int{left_count, count - left_count};
	starts := [2]int{start, start + left_count};
	for child_index in 0 ..< 2
	{
		child_count := counts[child_index];
		child_start := starts[child_index];
		child := tree_child(&nodes.memory[node_index], child_index);
		if child_count == 1
		{
			child^ = references.memory[child_start];
		}
		else
		{
			child_node_index := node_index + 1;
			if child_index == 1
			{
				child_node_index = node_index + left_count;
			}
			child^ = tree_child_from_bounds(
				tree_node_child_reference_bounds(references, child_start, child_count),
				i32(child_node_index), tree_node_child_reference_leaf_count(references, child_start, child_count),
			);
		}
	}
}

tree_temporary_partition_node_children :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Node_Child), nodes: util.Buffer(Tree_Node),
	node_index, start, count: int,
) -> int
{
	left_count := tree_binned_partition_node_children(references, scratch, start, count);
	tree_temporary_write_node_children(references, nodes, node_index, start, count, left_count);
	return left_count;
}

tree_temporary_build_node_children :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Node_Child), nodes: util.Buffer(Tree_Node),
	node_index, start, count: int,
)
{
	left_count := tree_temporary_partition_node_children(references, scratch, nodes, node_index, start, count);
	if left_count > 1
	{
		tree_temporary_build_node_children(references, scratch, nodes, node_index + 1, start, left_count);
	}
	right_count := count - left_count;
	if right_count > 1
	{
		tree_temporary_build_node_children(
			references,
			scratch,
			nodes,
			node_index + left_count,
			start + left_count,
			right_count
		);
	}
}

tree_reify_binned_refinement_child :: proc "contextless" (
	tree: ^Tree, child: ^Tree_Node_Child, node_indices: util.Buffer(i32),
	real_node_index, child_index: int,
)
{
	if child.index < 0
	{
		tree.leaves.memory[tree_decode_leaf(child.index)] = tree_leaf_create(real_node_index, child_index);
		return;
	}
	if child.index & TREE_REFINEMENT_SUBTREE_FLAG != 0
	{
		child.index &= ~TREE_REFINEMENT_SUBTREE_FLAG;
	}
	else
	{
		child.index = node_indices.memory[child.index];
	}
	child_meta := &tree.metanodes.memory[child.index];
	child_meta.parent = i32(real_node_index);
	child_meta.index_in_parent = i32(child_index);
}

tree_reify_binned_refinement_range :: proc "contextless" (
	tree: ^Tree, temporary_nodes: util.Buffer(Tree_Node), node_indices: util.Buffer(i32),
	start_index, end_index: int,
)
{
	for temporary_index in start_index ..< end_index
	{
		real_node_index := int(node_indices.memory[temporary_index]);
		refined := temporary_nodes.memory[temporary_index];
		tree_reify_binned_refinement_child(tree, &refined.a, node_indices, real_node_index, 0);
		tree_reify_binned_refinement_child(tree, &refined.b, node_indices, real_node_index, 1);
		tree.nodes.memory[real_node_index] = refined;
	}
}

tree_binned_refine_subtrees :: proc "contextless" (
	tree: ^Tree, subtrees, scratch: util.Buffer(Tree_Node_Child), subtree_count: int,
	temporary_nodes: util.Buffer(Tree_Node), node_indices: util.Buffer(i32), node_count: int,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || subtree_count <= 2 || node_count != subtree_count - 1 ||
	subtree_count > int(subtrees.length) ||
	subtree_count > int(scratch.length) ||
	node_count > int(temporary_nodes.length) ||
	node_count > int(node_indices.length)
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	_ = util.buffer_clear(temporary_nodes, 0, node_count);
	tree_temporary_build_node_children(subtrees, scratch, temporary_nodes, 0, 0, subtree_count);
	tree_reify_binned_refinement_range(tree, temporary_nodes, node_indices, 0, node_count);
	return .Ok;
}

tree_temporary_build_node :: proc "contextless" (
	references, scratch: util.Buffer(Tree_Build_Reference), nodes: util.Buffer(Tree_Node),
	node_index, start, count: int, next_node: ^int,
)
{
	left_count := tree_binned_partition_references(references, scratch, start, count);
	counts := [2]int{left_count, count - left_count};
	starts := [2]int{start, start + left_count};
	for child_index in 0 ..< 2
	{
		child_count := counts[child_index];
		child_start := starts[child_index];
		bounds := tree_reference_bounds(references, child_start, child_count);
		child := tree_child(&nodes.memory[node_index], child_index);
		if child_count == 1
		{
			child^ = tree_child_from_bounds(
				bounds,
				tree_encode_leaf(int(references.memory[child_start].leaf_index)),
				1
			);
		}
		else
		{
			child_node_index := next_node^;
			next_node^ += 1;
			child^ = tree_child_from_bounds(bounds, i32(child_node_index), i32(child_count));
			tree_temporary_build_node(
				references,
				scratch,
				nodes,
				child_node_index,
				child_start,
				child_count,
				next_node
			);
		}
	}
}

tree_collect_treelet :: proc "contextless" (
	tree: ^Tree, node_index: int, references: util.Buffer(Tree_Build_Reference), reference_count: ^int,
	node_indices: util.Buffer(i32), node_count: ^int,
) -> Physics_Status
{
	if reference_count^ >= int(references.length) || node_count^ >= int(node_indices.length)
	{
		return .Capacity_Missing;
	}
	node_indices.memory[node_count^] = i32(node_index);
	node_count^ += 1;
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index < 0
		{
			if reference_count^ >= int(references.length)
			{
				return .Capacity_Missing;
			}
			references.memory[reference_count^] = {
				bounds=tree_child_bounds(child^),
				leaf_index=i32(tree_decode_leaf(child.index)),
			};
			reference_count^ += 1;
		}
		else
		{
			status := tree_collect_treelet(
				tree,
				int(child.index),
				references,
				reference_count,
				node_indices,
				node_count
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

tree_binned_refine_treelet :: proc (
	tree: ^Tree, root_node_index: int,
	references, scratch: util.Buffer(Tree_Build_Reference), temporary_nodes: util.Buffer(Tree_Node),
	node_indices: util.Buffer(i32),
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || root_node_index < 0 || root_node_index >= tree.node_count
	{
		return .Invalid_Argument;
	}
	reference_count, node_count := 0, 0;
	status := tree_collect_treelet(tree, root_node_index, references, &reference_count, node_indices, &node_count);
	if status != .Ok
	{
		return status;
	}
	if reference_count <= 2
	{
		return .Ok;
	}
	if node_count != reference_count - 1 || node_count > int(temporary_nodes.length)
	{
		return .Invalid_Description;
	}
	_ = util.buffer_clear(temporary_nodes, 0, node_count);
	next_node := 1;
	tree_temporary_build_node(references, scratch, temporary_nodes, 0, 0, reference_count, &next_node);
	for temporary_index in 0 ..< node_count
	{
		real_node_index := int(node_indices.memory[temporary_index]);
		refined := temporary_nodes.memory[temporary_index];
		for child_index in 0 ..< 2
		{
			child := tree_child(&refined, child_index);
			if child.index >= 0
			{
				child.index = node_indices.memory[child.index];
			}
			tree_set_child_location(tree, child^, real_node_index, child_index);
		}
		tree.nodes.memory[real_node_index] = refined;
	}
	tree_refit_for_node_bounds_change(tree, root_node_index);
	return .Ok;
}

tree_refinement_candidate_count_node :: proc "contextless" (tree: ^Tree, node_index, treelet_size: int) -> int
{
	node := &tree.nodes.memory[node_index];
	count := 0;
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index < 0 || child.leaf_count <= 2
		{
			continue;
		}
		if child.leaf_count <= i32(treelet_size)
		{
			count += 1;
		}
		else
		{
			count += tree_refinement_candidate_count_node(tree, int(child.index), treelet_size);
		}
	}
	return count;
}

tree_refinement_candidate_at_node :: proc "contextless" (
	tree: ^Tree, node_index, treelet_size: int, target: ^int,
) -> int
{
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index < 0 || child.leaf_count <= 2
		{
			continue;
		}
		if child.leaf_count <= i32(treelet_size)
		{
			if target^ == 0
			{
				return int(child.index);
			}
			target^ -= 1;
		}
		else
		{
			found := tree_refinement_candidate_at_node(tree, int(child.index), treelet_size, target);
			if found >= 0
			{
				return found;
			}
		}
	}
	return -1;
}

tree_refinement_candidate_count :: proc "contextless" (tree: ^Tree, treelet_size: int) -> int
{
	if tree.leaf_count <= 2
	{
		return 0;
	}
	if tree.leaf_count <= treelet_size
	{
		return 1;
	}
	return tree_refinement_candidate_count_node(tree, 0, treelet_size);
}

tree_refinement_candidate_at :: proc "contextless" (tree: ^Tree, treelet_size, candidate_index: int) -> int
{
	if tree.leaf_count <= treelet_size
	{
		return 0;
	}
	index := candidate_index;
	return tree_refinement_candidate_at_node(tree, 0, treelet_size, &index);
}

tree_refinement_offset_at_frame :: proc "contextless" (
	tree: ^Tree, frame_index: u32, treelet_size: int = TREE_DEFAULT_TREELET_SIZE,
) -> i32
{
	if tree == nil || tree.state != .Ready || treelet_size <= 2
	{
		return -1;
	}
	candidate_count := tree_refinement_candidate_count(tree, treelet_size);
	if candidate_count == 0
	{
		return -1;
	}
	return i32((u64(frame_index) * 236887691 + 104395303) % u64(candidate_count));
}

tree_refine_at_frame :: proc (
	tree: ^Tree,
	frame_index: u32,
	treelet_size: int = TREE_DEFAULT_TREELET_SIZE
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || treelet_size <= 2
	{
		return .Invalid_Argument;
	}
	candidate_count := tree_refinement_candidate_count(tree, treelet_size);
	if candidate_count == 0
	{
		return .Ok;
	}
	offset := int(tree_refinement_offset_at_frame(tree, frame_index, treelet_size));
	target := tree_refinement_candidate_at(tree, treelet_size, offset);
	if target < 0
	{
		return .Invalid_Description;
	}
	capacity := min(treelet_size, tree.leaf_count);
	references, references_status := util.buffer_pool_take_at_least(tree.pool, Tree_Build_Reference, capacity);
	if references_status != .Ok
	{
		return physics_memory_status(references_status);
	}
	scratch, scratch_status := util.buffer_pool_take_at_least(tree.pool, Tree_Build_Reference, capacity);
	if scratch_status != .Ok
	{
		physics_return_buffer(tree.pool, &references);
		return physics_memory_status(scratch_status);
	}
	temporary_nodes, nodes_status := util.buffer_pool_take_at_least(tree.pool, Tree_Node, max(capacity - 1, 1));
	if nodes_status != .Ok
	{
		physics_return_buffer(tree.pool, &scratch);
		physics_return_buffer(tree.pool, &references);
		return physics_memory_status(nodes_status);
	}
	node_indices, indices_status := util.buffer_pool_take_at_least(tree.pool, i32, max(capacity - 1, 1));
	if indices_status != .Ok
	{
		physics_return_buffer(tree.pool, &temporary_nodes);
		physics_return_buffer(tree.pool, &scratch);
		physics_return_buffer(tree.pool, &references);
		return physics_memory_status(indices_status);
	}
	status := tree_binned_refine_treelet(tree, target, references, scratch, temporary_nodes, node_indices);
	physics_return_buffer(tree.pool, &node_indices);
	physics_return_buffer(tree.pool, &temporary_nodes);
	physics_return_buffer(tree.pool, &scratch);
	physics_return_buffer(tree.pool, &references);
	return status;
}

tree_refine :: proc "contextless" (tree: ^Tree) -> Physics_Status
{
	if tree == nil
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	status := tree_refine_at_frame(tree, tree.refinement_frame);
	if status == .Ok
	{
		tree.refinement_frame += 1;
	}
	return status;
}

tree_cache_copy_node :: proc "contextless" (
	tree: ^Tree, source_nodes: util.Buffer(Tree_Node), target_nodes: util.Buffer(Tree_Node),
	target_metanodes: util.Buffer(Tree_Metanode), source_index, target_index, parent_index, slot: int,
	next_target: ^int,
)
{
	source := source_nodes.memory[source_index];
	target := Tree_Node{};
	target_metanodes.memory[target_index] = {parent=i32(parent_index), index_in_parent=i32(slot)};
	for child_index in 0 ..< 2
	{
		source_child := tree_child(&source, child_index);
		target_child := tree_child(&target, child_index);
		target_child^ = source_child^;
		if source_child.index >= 0
		{
			child_target := next_target^;
			next_target^ += 1;
			target_child.index = i32(child_target);
			tree_cache_copy_node(
				tree, source_nodes, target_nodes, target_metanodes,
				int(source_child.index), child_target, target_index, child_index, next_target,
			);
		}
		else
		{
			tree.leaves.memory[tree_decode_leaf(source_child.index)] = tree_leaf_create(target_index, child_index);
		}
	}
	target_nodes.memory[target_index] = target;
}

tree_optimize_cache :: proc (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready
	{
		return .Disposed;
	}
	if tree.node_count <= 1
	{
		return .Ok;
	}
	new_nodes, nodes_status := util.buffer_pool_take_at_least(tree.pool, Tree_Node, int(tree.nodes.length));
	if nodes_status != .Ok
	{
		return physics_memory_status(nodes_status);
	}
	new_metanodes, metanodes_status := util.buffer_pool_take_at_least(
		tree.pool,
		Tree_Metanode,
		int(tree.metanodes.length)
	);
	if metanodes_status != .Ok
	{
		physics_return_buffer(tree.pool, &new_nodes);
		return physics_memory_status(metanodes_status);
	}
	_ = util.buffer_clear(new_nodes, 0, int(new_nodes.length));
	_ = util.buffer_clear(new_metanodes, 0, int(new_metanodes.length));
	next_target := 1;
	tree_cache_copy_node(tree, tree.nodes, new_nodes, new_metanodes, 0, 0, -1, -1, &next_target);
	physics_return_buffer(tree.pool, &tree.metanodes);
	physics_return_buffer(tree.pool, &tree.nodes);
	tree.nodes = new_nodes;
	tree.metanodes = new_metanodes;
	return .Ok;
}
