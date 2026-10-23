#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// clip the moving high AABB against the conservative selected-width child
	// velocity and time keep the local sweep kernel's f32 contract. subtraction
	// and slab division are f64, rounded outward, without an origin-scaled epsilon
	world_precision_sweep_clip_child :: proc "contextless" (
		bounds: ^Global_Bounds, velocity: util.Vector3, child: ^Tree_Node_Child, maximum_t: f32,
	) -> Reference_State
	{
		query_min: [3]f64 = {bounds.min.x, bounds.min.y, bounds.min.z};
		query_max: [3]f64 = {bounds.max.x, bounds.max.y, bounds.max.z};
		child_min: [3]f64 = {f64(child.min.x), f64(child.min.y), f64(child.min.z)};
		child_max: [3]f64 = {f64(child.max.x), f64(child.max.y), f64(child.max.z)};
		motion: [3]f64 = {f64(velocity.x), f64(velocity.y), f64(velocity.z)};
		entry: f64 = 0;
		exit: f64 = f64(maximum_t);
		for axis in 0 ..< 3
		{
			speed: f64 = motion[axis];
			if speed == 0
			{
				if query_min[axis] > child_max[axis] || query_max[axis] < child_min[axis]
				{
					return .Missing;
				}
				continue;
			}
			lower, lower_residue, upper, upper_residue: f64;
			if speed > 0
			{
				lower, lower_residue = world_precision_sum_residue(child_min[axis], -query_max[axis]);
				upper, upper_residue = world_precision_sum_residue(child_max[axis], -query_min[axis]);
			}
			else
			{
				speed = -speed;
				lower, lower_residue = world_precision_sum_residue(query_min[axis], -child_max[axis]);
				upper, upper_residue = world_precision_sum_residue(query_max[axis], -child_min[axis]);
			}
			if lower_residue < 0
			{
				lower = math.nextafter_f64(lower, math.inf_f64(-1));
			}
			if upper_residue > 0
			{
				upper = math.nextafter_f64(upper, math.inf_f64(1));
			}
			entry = max(entry, math.nextafter_f64(lower / speed, math.inf_f64(-1)));
			exit = min(exit, math.nextafter_f64(upper / speed, math.inf_f64(1)));
			if entry > exit
			{
				return .Missing;
			}
		}
		return .Present;
	}

	// the owning query preflights both trees before any callbacks. the stack has
	// at least tree.node_count entries. topology/bounds stay immutable throughout,
	// callbacks may only decrease maximum_t. all-hit callbacks retain it unchanged
	world_precision_tree_sweep_traverse :: proc "contextless" (
		tree: ^Tree, bounds: ^Global_Bounds, velocity: util.Vector3,
		maximum_t: ^f32, traversal_stack: []i32, leaf_proc: Tree_Ray_Leaf_Proc, user_context: rawptr,
	) -> Physics_Status
	{
		if tree.leaf_count == 0
		{
			return .Ok;
		}
		traversal_stack[0] = 0;
		stack_count: int = 1;
		child_count: int = 2;
		if tree.leaf_count == 1
		{
			child_count = 1;
		}
		for stack_count > 0
		{
			stack_count -= 1;
			node: ^Tree_Node = &tree.nodes.memory[traversal_stack[stack_count]];
			for child_index in 0 ..< child_count
			{
				child: ^Tree_Node_Child = tree_child(node, child_index);
				if world_precision_sweep_clip_child(bounds, velocity, child, maximum_t^) != .Present
				{
					continue;
				}
				if child.index >= 0
				{
					traversal_stack[stack_count] = child.index;
					stack_count += 1;
					continue;
				}
				status: Physics_Status = leaf_proc(user_context, tree_decode_leaf(child.index), maximum_t);
				if status != .Ok
				{
					return status;
				}
			}
		}
		return .Ok;
	}
}
