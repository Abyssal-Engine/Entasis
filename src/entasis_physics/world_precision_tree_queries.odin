#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	World_Precision_Segment :: struct
	{
		start: Global_Position,
		end: Global_Position,
	}

	World_Precision_Ray_Leaf :: struct
	{
		leaf_index: i32,
		min_t: f64,
		max_t: f64,
	}

	World_Precision_Prepared_Segment :: struct
	{
		start: [3]f64,
		end: [3]f64,
		distance_min: [3]f64,
		distance_max: [3]f64,
	}

	world_precision_segment_prepare :: proc "contextless" (
		segment: World_Precision_Segment,
	) -> (World_Precision_Prepared_Segment, Physics_Status)
	{
		if !(abs(segment.start.x) <= math.F64_MAX && abs(segment.start.y) <= math.F64_MAX && abs(segment.start.z) <= math.F64_MAX &&
			abs(segment.end.x) <= math.F64_MAX && abs(segment.end.y) <= math.F64_MAX && abs(segment.end.z) <= math.F64_MAX)
		{
			return {}, .Invalid_Argument;
		}
		result := World_Precision_Prepared_Segment{
			start={segment.start.x, segment.start.y, segment.start.z},
			end={segment.end.x, segment.end.y, segment.end.z},
		};
		for axis in 0 ..< 3
		{
			a := max(result.start[axis], result.end[axis]);
			b := min(result.start[axis], result.end[axis]);
			distance, residue := world_precision_sum_residue(a, -b);
			if !(distance <= math.F64_MAX)
			{
				return {}, .Invalid_Argument;
			}
			result.distance_min[axis] = distance;
			result.distance_max[axis] = distance;
			if residue < 0
			{
				result.distance_min[axis] = math.nextafter_f64(distance, math.inf_f64(-1));
			}
			if residue > 0
			{
				result.distance_max[axis] = math.nextafter_f64(distance, math.inf_f64(1));
			}
			if result.distance_max[axis] > math.F64_MAX
			{
				return {}, .Invalid_Argument;
			}
		}
		return result, .Ok;
	}

	// clip only planes actually crossed by the segment. every division therefore
	// uses a nonnegative numerator and positive denominator, with a result in or
	// immediately around [0, 1]. two-sum and directed division protect tangencies
	// without an epsilon scaled by the world's absolute coordinate
	world_precision_segment_clip_child :: proc "contextless" (
		segment: ^World_Precision_Prepared_Segment,
		child: Tree_Node_Child,
	) -> (min_t, max_t: f64, state: Reference_State)
	{
		minimum := [3]f64{f64(child.min.x), f64(child.min.y), f64(child.min.z)};
		maximum := [3]f64{f64(child.max.x), f64(child.max.y), f64(child.max.z)};
		min_t = 0;
		max_t = 1;
		for axis in 0 ..< 3
		{
			start := segment.start[axis];
			end := segment.end[axis];
			lo := minimum[axis];
			hi := maximum[axis];
			if max(start, end) < lo || min(start, end) > hi
			{
				return 0, 0, .Missing;
			}
			if start == end
			{
				continue;
			}
			entry_sum, entry_residue: f64;
			exit_sum, exit_residue: f64;
			entry_state := Reference_State.Missing;
			exit_state := Reference_State.Missing;
			if start < end
			{
				if start < lo
				{
					entry_sum, entry_residue = world_precision_sum_residue(lo, -start);
					entry_state = .Present;
				}
				if end > hi
				{
					exit_sum, exit_residue = world_precision_sum_residue(hi, -start);
					exit_state = .Present;
				}
			}
			else
			{
				if start > hi
				{
					entry_sum, entry_residue = world_precision_sum_residue(start, -hi);
					entry_state = .Present;
				}
				if end < lo
				{
					exit_sum, exit_residue = world_precision_sum_residue(start, -lo);
					exit_state = .Present;
				}
			}
			if entry_state == .Present
			{
				if entry_residue < 0
				{
					entry_sum = math.nextafter_f64(entry_sum, math.inf_f64(-1));
				}
				entry := math.nextafter_f64(entry_sum / segment.distance_max[axis], math.inf_f64(-1));
				min_t = max(min_t, entry);
			}
			if exit_state == .Present
			{
				if exit_residue > 0
				{
					exit_sum = math.nextafter_f64(exit_sum, math.inf_f64(1));
				}
				exit := math.nextafter_f64(exit_sum / segment.distance_min[axis], math.inf_f64(1));
				max_t = min(max_t, exit);
			}
			if min_t > max_t
			{
				return 0, 0, .Missing;
			}
		}
		return min_t, max_t, .Present;
	}

	// query the selected-width tree without rounding the global segment. each
	// output is a conservative candidate interval, not a narrow-phase shape hit
	// reserve traversal_stack for tree.node_count and results for tree.leaf_count
	// outside the query. no allocation, tree mutation, sorting or callback is hidden
	world_precision_tree_ray_query :: proc "contextless" (
		tree: ^Tree,
		segment: World_Precision_Segment,
		output: []World_Precision_Ray_Leaf,
		traversal_stack: []i32,
	) -> (int, Physics_Status)
	{
		if tree == nil || tree.state != .Ready
		{
			return 0, .Disposed;
		}
		prepared, status := world_precision_segment_prepare(segment);
		if status != .Ok
		{
			return 0, status;
		}
		if tree.leaf_count == 0
		{
			return 0, .Ok;
		}
		if len(traversal_stack) < tree.node_count
		{
			return 0, .Capacity_Missing;
		}
		traversal_stack[0] = 0;
		stack_count := 1;
		output_count := 0;
		for stack_count > 0
		{
			stack_count -= 1;
			node_index := int(traversal_stack[stack_count]);
			node := tree.nodes.memory[node_index];
			children := [2]Tree_Node_Child{node.a, node.b};
			child_count := 2;
			if tree.leaf_count == 1
			{
				child_count = 1;
			}
			for child in children[:child_count]
			{
				min_t, max_t, state := world_precision_segment_clip_child(&prepared, child);
				if state != .Present
				{
					continue;
				}
				if child.index >= 0
				{
					traversal_stack[stack_count] = child.index;
					stack_count += 1;
					continue;
				}
				if output_count == len(output)
				{
					return output_count, .Capacity_Missing;
				}
				output[output_count] = {
					leaf_index=i32(tree_decode_leaf(child.index)),
					min_t=min_t,
					max_t=max_t,
				};
				output_count += 1;
			}
		}
		return output_count, .Ok;
	}

	// bounds and scratch are admitted once by the owning global query. leaves and
	// node topology remain immutable through callbacks. traversal reports callback
	// status verbatim, including the any-overlap owner's paired early-out sentinel
	world_precision_tree_volume_traverse :: proc "contextless" (
		tree: ^Tree, bounds: Global_Bounds, traversal_stack: []i32,
		leaf_proc: Tree_Volume_Leaf_Proc, user_context: rawptr,
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
				// immutable topology permits direct child reads through callbacks
				child: ^Tree_Node_Child = tree_child(node, child_index);
				if bounds.min.x > f64(child.max.x) || bounds.max.x < f64(child.min.x) ||
					bounds.min.y > f64(child.max.y) || bounds.max.y < f64(child.min.y) ||
					bounds.min.z > f64(child.max.z) || bounds.max.z < f64(child.min.z)
				{
					continue;
				}
				if child.index >= 0
				{
					traversal_stack[stack_count] = child.index;
					stack_count += 1;
					continue;
				}
				status: Physics_Status = leaf_proc(user_context, tree_decode_leaf(child.index));
				if status != .Ok
				{
					return status;
				}
			}
		}
		return .Ok;
	}
}
