#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Bounds :: physics.Global_Bounds;

	// reserve i32 scratch outside recurring overlap/volume queries. the existing
	// global ray requirement is identical. structural mutations can increase it
	// empty worlds need zero entries even when they retain allocated tree roots
	global_bounds_stack_requirement :: proc "contextless" (world: ^World) -> (int, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return 0, status;
		}
		return physics.world_precision_ray_stack_requirement(&data.simulation.broad_phase), .Ok;
	}

	// return broad-phase bounds overlaps, including motion/speculative expansion
	// results need not intersect shape geometry. endpoints remain f64 and the boundary is closed
	// caller owns disjoint output/stack storage. insufficient stack is rejected
	// before any callback/output. later errors return the valid written prefix
	// empty output is valid for a miss and reports Capacity_Missing for a real hit
	volume_global_all_with_context :: proc "contextless" (
		query_context: ^Query_Context, bounds: Global_Bounds,
		hits: []Volume_Hit, traversal_stack: []i32, filter: Query_Filter = {},
	) -> (int, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return 0, status;
		}
		defer data.execution = .Idle;
		return query_context_global_volume_execute(data, bounds, hits, traversal_stack, filter);
	}

	query_context_global_volume_execute :: proc "contextless" (
		data: ^query_context_data, bounds: Global_Bounds,
		hits: []Volume_Hit, traversal_stack: []i32, filter: Query_Filter,
	) -> (int, Status)
	{
		filter_context: query_filter_context = {filter=filter};
		collector: physics.Volume_Query_Collector = {
			hits=query_volume_buffer(hits), filter=query_filter_low_level(filter), user_context=&filter_context,
		};
		status: Status = physics.simulation_global_volume_query(&data.world.simulation, bounds, &collector, traversal_stack);
		return collector.count, status;
	}
}
