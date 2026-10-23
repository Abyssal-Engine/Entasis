#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Volume_Query_Context :: struct
	{
		leaves: util.Buffer(Collidable_Reference),
		collector: ^Volume_Query_Collector,
	}

	global_volume_query_leaf :: proc "contextless" (
		user_context: rawptr, leaf_index: int,
	) -> Physics_Status
	{
		query: ^Global_Volume_Query_Context = (^Global_Volume_Query_Context)(user_context);
		target_id: i32 = i32(query.leaves.memory[leaf_index].packed);
		collector: ^Volume_Query_Collector = query.collector;
		if query_filter_target(collector.filter, collector.user_context, target_id) != .Allow
		{
			return .Ok;
		}
		if collector.count == int(collector.hits.length)
		{
			return .Capacity_Missing;
		}
		collector.hits.memory[collector.count] = {target_id};
		collector.count += 1;
		return .Ok;
	}

	// volumes query broad-phase bounds with motion/speculative expansion, so
	// an overlap need not intersect shape geometry. query endpoints stay f64,
	// even outside the admitted binary32 coordinate range. no shape resolution or
	// temporary candidate list is necessary to return the existing leaf identity
	simulation_global_volume_query :: proc "contextless" (
		simulation: ^Simulation, bounds: Global_Bounds,
		collector: ^Volume_Query_Collector, traversal_stack: []i32,
	) -> Physics_Status
	{
		if simulation == nil || simulation.state != .Ready
		{
			return .Disposed;
		}
		if collector == nil || collector.count != 0 || collector.hits.length < 0 ||
			collector.hits.length > 0 && collector.hits.memory == nil
		{
			return .Invalid_Argument;
		}
		if !(bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y && bounds.min.z <= bounds.max.z &&
			bounds.min.x >= -math.F64_MAX && bounds.min.y >= -math.F64_MAX && bounds.min.z >= -math.F64_MAX &&
			bounds.max.x <= math.F64_MAX && bounds.max.y <= math.F64_MAX && bounds.max.z <= math.F64_MAX)
		{
			return .Invalid_Argument;
		}
		broad: ^Broad_Phase = &simulation.broad_phase;
		if len(traversal_stack) < world_precision_ray_stack_requirement(broad)
		{
			return .Capacity_Missing;
		}
		query: Global_Volume_Query_Context = {leaves=broad.active_leaves, collector=collector};
		status: Physics_Status = world_precision_tree_volume_traverse(
			&broad.active_tree, bounds, traversal_stack, global_volume_query_leaf, &query,
		);
		if status == .Ok
		{
			query.leaves = broad.static_leaves;
			status = world_precision_tree_volume_traverse(
				&broad.static_tree, bounds, traversal_stack, global_volume_query_leaf, &query,
			);
		}
		return status;
	}
}
