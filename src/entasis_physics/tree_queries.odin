// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Tree_Ray :: struct
{
	origin:    util.Vector3,
	direction: util.Vector3,
	maximum_t: f32,
}

Tree_Pair_Order :: enum u8
{
	Canonical,
	Preserve_Tree_Order,
}

Tree_Pair_Visitor_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index, leaf_a, leaf_b: int,
) -> Physics_Status;

tree_append_leaf :: proc "contextless" (
	output: util.Buffer(i32), output_count: ^int, leaf_index: int,
) -> Physics_Status
{
	if output_count^ >= int(output.length)
	{
		return .Capacity_Missing;
	}
	output.memory[output_count^] = i32(leaf_index);
	output_count^ += 1;
	return .Ok;
}

tree_append_pair :: proc "contextless" (
	output: util.Buffer(Tree_Leaf_Pair), output_count: ^int, a, b: int, order: Tree_Pair_Order,
) -> Physics_Status
{
	if output_count^ >= int(output.length)
	{
		return .Capacity_Missing;
	}
	if order == .Preserve_Tree_Order || a < b
	{
		output.memory[output_count^] = {i32(a), i32(b)};
	}
	else
	{
		output.memory[output_count^] = {i32(b), i32(a)};
	}
	output_count^ += 1;
	return .Ok;
}

tree_query_child :: proc "contextless" (
	tree: ^Tree, child: ^Tree_Node_Child, query: util.Bounding_Box,
	output: util.Buffer(i32), output_count: ^int,
) -> Physics_Status
{
	if tree_bounds_intersect(tree_child_bounds(child^), query) != .Present
	{
		return .Ok;
	}
	if child.index < 0
	{
		return tree_append_leaf(output, output_count, tree_decode_leaf(child.index));
	}
	node := &tree.nodes.memory[child.index];
	status := tree_query_child(tree, &node.a, query, output, output_count);
	if status != .Ok
	{
		return status;
	}
	return tree_query_child(tree, &node.b, query, output, output_count);
}

tree_query_overlaps :: proc "contextless" (
	tree: ^Tree, query: util.Bounding_Box, output: util.Buffer(i32),
) -> (int, Physics_Status)
{
	if tree == nil || tree.state != .Ready || output.memory == nil ||
		query.min.x > query.max.x || query.min.y > query.max.y || query.min.z > query.max.z
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	if tree.leaf_count == 0
	{
		return 0, .Ok;
	}
	root := &tree.nodes.memory[0];
	status := tree_query_child(tree, &root.a, query, output, &output_count);
	if status != .Ok
	{
		return output_count, status;
	}
	if tree.leaf_count > 1
	{
		status = tree_query_child(tree, &root.b, query, output, &output_count);
	}
	return output_count, status;
}

tree_ray_intersection :: proc "contextless" (bounds: util.Bounding_Box, ray: Tree_Ray) -> (f32, Reference_State)
{
	util.math_validate_bounding_box(bounds);
	util.math_validate_vector3(ray.origin);
	util.math_validate_vector3(ray.direction);
	util.math_validate_f32(ray.maximum_t);
	t_min := f32(0);
	t_max := ray.maximum_t;
	for axis in 0 ..< 3
	{
		origin: f32;
		direction: f32;
		minimum: f32;
		maximum: f32;
		if axis == 0
		{
			origin=ray.origin.x;
			direction=ray.direction.x;
			minimum=bounds.min.x;
			maximum=bounds.max.x;
		}
		else if axis == 1
		{
			origin=ray.origin.y;
			direction=ray.direction.y;
			minimum=bounds.min.y;
			maximum=bounds.max.y;
		}
		else
		{
			origin=ray.origin.z;
			direction=ray.direction.z;
			minimum=bounds.min.z;
			maximum=bounds.max.z;
		}
		if abs(direction) <= 1e-20
		{
			if origin < minimum || origin > maximum
			{
				return 0, .Missing;
			}
		}
		else
		{
			inverse := 1 / direction;
			entry := (minimum - origin) * inverse;
			exit := (maximum - origin) * inverse;
			if entry > exit
			{
				entry, exit = exit, entry;
			}
			t_min = tree_ordered_max(t_min, entry);
			t_max = tree_ordered_min(t_max, exit);
			if t_min > t_max
			{
				return 0, .Missing;
			}
		}
	}
	return t_min, .Present;
}

tree_ray_query_child :: proc "contextless" (
	tree: ^Tree, child: ^Tree_Node_Child, ray: Tree_Ray,
	output: util.Buffer(i32), output_count: ^int,
) -> Physics_Status
{
	_, hit := tree_ray_intersection(tree_child_bounds(child^), ray);
	if hit != .Present
	{
		return .Ok;
	}
	if child.index < 0
	{
		return tree_append_leaf(output, output_count, tree_decode_leaf(child.index));
	}
	node := &tree.nodes.memory[child.index];
	status := tree_ray_query_child(tree, &node.a, ray, output, output_count);
	if status != .Ok
	{
		return status;
	}
	return tree_ray_query_child(tree, &node.b, ray, output, output_count);
}

tree_ray_query :: proc "contextless" (
	tree: ^Tree, ray: Tree_Ray, output: util.Buffer(i32),
) -> (int, Physics_Status)
{
	if tree == nil || tree.state != .Ready || output.memory == nil || ray.maximum_t < 0
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	if tree.leaf_count == 0
	{
		return 0, .Ok;
	}
	root := &tree.nodes.memory[0];
	status := tree_ray_query_child(tree, &root.a, ray, output, &output_count);
	if status != .Ok
	{
		return output_count, status;
	}
	if tree.leaf_count > 1
	{
		status = tree_ray_query_child(tree, &root.b, ray, output, &output_count);
	}
	return output_count, status;
}

tree_sweep_query :: proc "contextless" (
	tree: ^Tree, bounds: util.Bounding_Box, sweep: util.Vector3, maximum_t: f32, output: util.Buffer(i32),
) -> (int, Physics_Status)
{
	if bounds.min.x > bounds.max.x || bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z
	{
		return 0, .Invalid_Argument;
	}
	// a moving AABB intersects a target when the sweep point enters the target's Minkowski expansion
	query_ray := Tree_Ray{direction=sweep, maximum_t=maximum_t};
	return tree_sweep_query_node(tree, 0, bounds, query_ray, output);
}

tree_sweep_query_child :: proc "contextless" (
	tree: ^Tree, child: ^Tree_Node_Child, bounds: util.Bounding_Box, ray: Tree_Ray,
	output: util.Buffer(i32), output_count: ^int,
) -> Physics_Status
{
	expanded := util.Bounding_Box{
		min=util.vector3_subtract(child.min, bounds.max),
		max=util.vector3_subtract(child.max, bounds.min),
	};
	_, hit := tree_ray_intersection(expanded, ray);
	if hit != .Present
	{
		return .Ok;
	}
	if child.index < 0
	{
		return tree_append_leaf(output, output_count, tree_decode_leaf(child.index));
	}
	node := &tree.nodes.memory[child.index];
	status := tree_sweep_query_child(tree, &node.a, bounds, ray, output, output_count);
	if status != .Ok
	{
		return status;
	}
	return tree_sweep_query_child(tree, &node.b, bounds, ray, output, output_count);
}

tree_sweep_query_node :: proc "contextless" (
	tree: ^Tree, node_index: int, bounds: util.Bounding_Box, ray: Tree_Ray, output: util.Buffer(i32),
) -> (int, Physics_Status)
{
	if tree == nil || tree.state != .Ready || output.memory == nil || ray.maximum_t < 0
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	if tree.leaf_count == 0
	{
		return 0, .Ok;
	}
	node := &tree.nodes.memory[node_index];
	status := tree_sweep_query_child(tree, &node.a, bounds, ray, output, &output_count);
	if status != .Ok
	{
		return output_count, status;
	}
	if tree.leaf_count > 1
	{
		status = tree_sweep_query_child(tree, &node.b, bounds, ray, output, &output_count);
	}
	return output_count, status;
}

tree_visit_pair :: #force_inline proc "contextless" (
	visitor: Tree_Pair_Visitor_Proc, user_context: rawptr, worker_index, a, b: int,
	order: Tree_Pair_Order,
) -> Physics_Status
{
	leaf_a, leaf_b := a, b;
	if order == .Canonical && leaf_b < leaf_a
	{
		leaf_a, leaf_b = leaf_b, leaf_a;
	}
	return visitor(user_context, worker_index, leaf_a, leaf_b);
}

tree_visit_child_pair :: proc "contextless" (
	tree_a: ^Tree, child_a: ^Tree_Node_Child,
	tree_b: ^Tree, child_b: ^Tree_Node_Child,
	visitor: Tree_Pair_Visitor_Proc, user_context: rawptr, worker_index: int,
	order: Tree_Pair_Order,
) -> Physics_Status
{
	if tree_bounds_intersect(tree_child_bounds(child_a^), tree_child_bounds(child_b^)) != .Present
	{
		return .Ok;
	}
	if child_a.index < 0 && child_b.index < 0
	{
		return tree_visit_pair(
			visitor, user_context, worker_index,
			tree_decode_leaf(child_a.index), tree_decode_leaf(child_b.index), order,
		);
	}
	if child_b.index < 0 || child_a.index >= 0 && child_a.leaf_count >= child_b.leaf_count
	{
		node := &tree_a.nodes.memory[child_a.index];
		status := tree_visit_child_pair(
			tree_a, &node.a, tree_b, child_b,
			visitor, user_context, worker_index, order,
		);
		if status != .Ok
		{
			return status;
		}
		return tree_visit_child_pair(
			tree_a, &node.b, tree_b, child_b,
			visitor, user_context, worker_index, order,
		);
	}
	node := &tree_b.nodes.memory[child_b.index];
	status := tree_visit_child_pair(
		tree_a, child_a, tree_b, &node.a,
		visitor, user_context, worker_index, order,
	);
	if status != .Ok
	{
		return status;
	}
	return tree_visit_child_pair(
		tree_a, child_a, tree_b, &node.b,
		visitor, user_context, worker_index, order,
	);
}

tree_self_visit_node :: proc "contextless" (
	tree: ^Tree, node_index: int, visitor: Tree_Pair_Visitor_Proc,
	user_context: rawptr, worker_index: int,
) -> Physics_Status
{
	node := &tree.nodes.memory[node_index];
	if node.a.index >= 0
	{
		status := tree_self_visit_node(
			tree, int(node.a.index), visitor, user_context, worker_index,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if node.b.index >= 0
	{
		status := tree_self_visit_node(
			tree, int(node.b.index), visitor, user_context, worker_index,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return tree_visit_child_pair(
		tree, &node.a, tree, &node.b, visitor, user_context, worker_index, .Canonical,
	);
}

tree_self_visit :: proc "contextless" (
	tree: ^Tree, visitor: Tree_Pair_Visitor_Proc, user_context: rawptr,
	worker_index: int = 0,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || visitor == nil || worker_index < 0
	{
		return .Invalid_Argument;
	}
	if tree.leaf_count < 2
	{
		return .Ok;
	}
	return tree_self_visit_node(tree, 0, visitor, user_context, worker_index);
}

tree_intertree_visit :: proc "contextless" (
	tree_a, tree_b: ^Tree, visitor: Tree_Pair_Visitor_Proc, user_context: rawptr,
	worker_index: int = 0,
) -> Physics_Status
{
	if tree_a == nil || tree_b == nil || tree_a.state != .Ready ||
		tree_b.state != .Ready || visitor == nil || worker_index < 0
	{
		return .Invalid_Argument;
	}
	if tree_a.leaf_count == 0 || tree_b.leaf_count == 0
	{
		return .Ok;
	}
	root_a := &tree_a.nodes.memory[0];
	root_b := &tree_b.nodes.memory[0];
	a_child_count := 1;
	b_child_count := 1;
	if tree_a.leaf_count > 1
	{
		a_child_count = 2;
	}
	if tree_b.leaf_count > 1
	{
		b_child_count = 2;
	}
	for a_child_index in 0 ..< a_child_count
	{
		for b_child_index in 0 ..< b_child_count
		{
			status := tree_visit_child_pair(
				tree_a, tree_child(root_a, a_child_index), tree_b, tree_child(root_b, b_child_index),
				visitor, user_context, worker_index, .Preserve_Tree_Order,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

Tree_Pair_Buffer_Visitor_Context :: struct
{
	output: util.Buffer(Tree_Leaf_Pair),
	count:  int,
}

tree_pair_buffer_visitor :: proc "contextless" (
	user_context: rawptr, worker_index, leaf_a, leaf_b: int,
) -> Physics_Status
{
	_ = worker_index;
	ctx := (^Tree_Pair_Buffer_Visitor_Context)(user_context);
	return tree_append_pair(ctx.output, &ctx.count, leaf_a, leaf_b, .Preserve_Tree_Order);
}

tree_self_query_node :: proc "contextless" (
	tree: ^Tree, node_index: int, output: util.Buffer(Tree_Leaf_Pair), output_count: ^int,
) -> Physics_Status
{
	ctx := Tree_Pair_Buffer_Visitor_Context{output=output, count=output_count^};
	status := tree_self_visit_node(tree, node_index, tree_pair_buffer_visitor, &ctx, 0);
	output_count^ = ctx.count;
	return status;
}

tree_self_query :: proc "contextless" (
	tree: ^Tree, output: util.Buffer(Tree_Leaf_Pair),
) -> (int, Physics_Status)
{
	if output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	ctx := Tree_Pair_Buffer_Visitor_Context{output=output};
	status := tree_self_visit(tree, tree_pair_buffer_visitor, &ctx);
	return ctx.count, status;
}

tree_intertree_query :: proc "contextless" (
	tree_a, tree_b: ^Tree, output: util.Buffer(Tree_Leaf_Pair),
) -> (int, Physics_Status)
{
	if output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	ctx := Tree_Pair_Buffer_Visitor_Context{output=output};
	status := tree_intertree_visit(tree_a, tree_b, tree_pair_buffer_visitor, &ctx);
	return ctx.count, status;
}

tree_validate_node :: proc "contextless" (
	tree: ^Tree,
	node_index,
	expected_parent,
	expected_slot: int
) -> (i32, Physics_Status)
{
	meta := tree.metanodes.memory[node_index];
	if meta.parent != i32(expected_parent) || meta.index_in_parent != i32(expected_slot)
	{
		return 0, .Invalid_Description;
	}
	node := &tree.nodes.memory[node_index];
	count: i32;
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.leaf_count <= 0
		{
			return 0, .Invalid_Description;
		}
		if child.index < 0
		{
			leaf_index := tree_decode_leaf(child.index);
			if leaf_index < 0 || leaf_index >= tree.leaf_count
			{
				return 0, .Invalid_Description;
			}
			leaf := tree.leaves.memory[leaf_index];
			if tree_leaf_node_index(leaf) != node_index ||
				tree_leaf_child_index(leaf) != child_index ||
				child.leaf_count != 1
			{
				return 0, .Invalid_Description;
			}
		}
		else
		{
			child_count, status := tree_validate_node(tree, int(child.index), node_index, child_index);
			if status != .Ok || child_count != child.leaf_count
			{
				return 0, .Invalid_Description;
			}
			child_node := tree.nodes.memory[child.index];
			merged := tree_bounds_merge(tree_child_bounds(child_node.a), tree_child_bounds(child_node.b));
			if merged.min != child.min || merged.max != child.max
			{
				return 0, .Invalid_Description;
			}
		}
		count += child.leaf_count;
	}
	return count, .Ok;
}

tree_validate :: proc "contextless" (tree: ^Tree) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || tree.node_count < 1 || tree.leaf_count < 0
	{
		return .Invalid_Description;
	}
	if tree.leaf_count == 0
	{
		if tree.node_count != 1
		{
			return .Invalid_Description;
		}
		return .Ok;
	}
	if tree.leaf_count == 1
	{
		leaf := tree.leaves.memory[0];
		if tree_leaf_node_index(leaf) != 0 ||
			tree_leaf_child_index(leaf) != 0 ||
			tree.nodes.memory[0].a.index != tree_encode_leaf(0)
		{
			return .Invalid_Description;
		}
		return .Ok;
	}
	count, status := tree_validate_node(tree, 0, -1, -1);
	if status != .Ok || int(count) != tree.leaf_count || tree.node_count != tree.leaf_count - 1
	{
		return .Invalid_Description;
	}
	return .Ok;
}
