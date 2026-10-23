#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// query-centered local f32 geometry with authoritative f64 positioning
	// manifold offsets remain relative to the supplied high query pose, not the
	// world origin. filters and custom geometry callbacks are read-only. custom
	// collision callbacks explicitly receive the query-centered frame
	// caller output and traversal slices must be disjoint. an error returns the
	// valid written prefix and leaves every element beyond count untouched
	// reserve traversal with global_bounds_stack_requirement before recurring work
	// prepared child traversal uses only this context's private pool/batcher
	// the existing float entry points and their callback contracts are unchanged
	overlap_global_all_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape: Shape_Handle, pose: Global_Pose,
		hits: []Overlap_Hit, traversal_stack: []i32, filter: Query_Filter = {},
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
		return query_context_global_overlap_execute(data, shape, pose, hits, traversal_stack, filter);
	}

	// stop only after a permitted nonnegative-depth contact. no all-hit list is
	// constructed. errors discard partial hit state and leave the context reusable
	overlap_global_any_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape: Shape_Handle, pose: Global_Pose,
		traversal_stack: []i32, filter: Query_Filter = {},
	) -> (Overlap_State, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return .Separated, status;
		}
		defer data.execution = .Idle;
		return query_context_global_overlap_any_execute(data, shape, pose, traversal_stack, filter);
	}

	query_context_global_overlap_execute :: proc "contextless" (
		data: ^query_context_data, shape: Shape_Handle, pose: Global_Pose,
		hits: []Overlap_Hit, traversal_stack: []i32, filter: Query_Filter,
	) -> (int, Status)
	{
		filter_context: query_filter_context = {filter=filter};
		collector: physics.Overlap_Query_Collector = {
			hits=query_overlap_buffer(hits), filter=query_filter_low_level(filter), user_context=&filter_context,
		};
		status: Status = physics.simulation_global_overlap_query(
			&data.world.simulation, shape, pose, &collector, traversal_stack, &data.overlap, data.pool,
		);
		return collector.count, status;
	}

	query_context_global_overlap_any_execute :: proc "contextless" (
		data: ^query_context_data, shape: Shape_Handle, pose: Global_Pose,
		traversal_stack: []i32, filter: Query_Filter,
	) -> (Overlap_State, Status)
	{
		filter_context: query_filter_context = {filter=filter};
		return physics.simulation_global_overlap_any_query(
			&data.world.simulation, shape, pose, query_filter_low_level(filter), &filter_context,
			traversal_stack, &data.overlap, data.pool,
		);
	}
}
