#+vet !unused-imports

package entasis_c

import "base:intrinsics"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	abi_query_context_query_batch_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context, queries: [^]Entasis_Global_Query, query_count: u64,
		results: [^]Entasis_Global_Query_Result, result_capacity: u64, scratch: ^Entasis_Global_Query_Scratch,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if !abi_count_valid(query_count) || !abi_count_valid(result_capacity) || result_capacity < query_count || scratch == nil ||
			(query_count > 0 && (queries == nil || results == nil))
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(scratch.ray_capacity) || (scratch.ray_capacity > 0 && scratch.ray_hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(scratch.sweep_capacity) || (scratch.sweep_capacity > 0 && scratch.sweep_hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(scratch.overlap_capacity) || (scratch.overlap_capacity > 0 && scratch.overlap_hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(scratch.volume_capacity) || (scratch.volume_capacity > 0 && scratch.volume_hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if !abi_count_valid(scratch.stack_capacity) || (scratch.stack_capacity > 0 && scratch.traversal_stack == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		core_scratch: entasis.Global_Query_Scratch = {
			ray_hits=(cast([^]entasis.Global_Ray_Hit)scratch.ray_hits)[:int(scratch.ray_capacity)],
			sweep_hits=(cast([^]entasis.Global_Sweep_Hit)scratch.sweep_hits)[:int(scratch.sweep_capacity)],
			overlap_hits=(cast([^]entasis.Overlap_Hit)scratch.overlap_hits)[:int(scratch.overlap_capacity)],
			volume_hits=(cast([^]entasis.Volume_Hit)scratch.volume_hits)[:int(scratch.volume_capacity)],
			traversal_stack=scratch.traversal_stack[:int(scratch.stack_capacity)],
		};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		overall: entasis.Status = .Ok;
		for index in 0 ..< int(query_count)
		{
			input: ^Entasis_Global_Query = &queries[index];
			output: ^Entasis_Global_Query_Result = &results[index];
			output^ = {kind=input.kind};
			native_queries: [1]entasis.Global_Query;
			native_results: [1]entasis.Global_Query_Result;
			bridge: abi_query_filter_bridge;
			query: ^entasis.Global_Query = &native_queries[0];
			query.kind = entasis.Global_Query_Kind(input.kind);
			switch input.kind
			{
				case 0, 1, 2:
					payload: ^abi_global_query_ray_payload = cast(^abi_global_query_ray_payload)&input.payload[0];
					query.ray_data = {segment=transmute(entasis.World_Precision_Segment)payload.segment,
						filter=abi_query_filter_to_core(payload.filter, &bridge), output=cast(entasis.Query_Output)payload.output};
				case 3, 4, 5:
					payload: ^abi_global_query_sweep_payload = cast(^abi_global_query_sweep_payload)&input.payload[0];
					query.sweep_data = {shape=abi_shape_handle_to_core(payload.shape), pose=abi_global_pose_to_core(payload.pose),
						velocity=abi_velocity_to_core(payload.velocity), maximum_t=payload.maximum_t,
						settings=abi_sweep_settings_to_core(payload.settings), filter=abi_query_filter_to_core(payload.filter, &bridge),
						output=cast(entasis.Query_Output)payload.output};
				case 6, 7:
					payload: ^abi_global_query_overlap_payload = cast(^abi_global_query_overlap_payload)&input.payload[0];
					query.overlap_data = {shape=abi_shape_handle_to_core(payload.shape), pose=abi_global_pose_to_core(payload.pose),
						filter=abi_query_filter_to_core(payload.filter, &bridge), output=cast(entasis.Query_Output)payload.output};
				case 8:
					payload: ^abi_global_query_volume_payload = cast(^abi_global_query_volume_payload)&input.payload[0];
					query.volume_data = {bounds=transmute(entasis.Global_Bounds)payload.bounds,
						filter=abi_query_filter_to_core(payload.filter, &bridge), output=cast(entasis.Query_Output)payload.output};
				case:
					output.status = abi_status(.Invalid_Argument);
					if overall == .Ok
					{
						overall = .Invalid_Argument;
					}
					continue;
			}
			status: entasis.Status = entasis.query_batch_global_with_context(&resource.native,
				native_queries[:], native_results[:], core_scratch);
			native: entasis.Global_Query_Result = native_results[0];
			output.status = abi_status(native.status);
			output.count = native.count;
			output.hit = abi_bool(native.hit == .Hit);
			if native.status == .Ok && native.hit == .Hit
			{
				if input.kind == 1
				{
					value: Entasis_Global_Ray_Hit = transmute(Entasis_Global_Ray_Hit)native.ray_hit;
					intrinsics.mem_copy_non_overlapping(&output.payload[0], &value, size_of(value));
				}
				else if input.kind == 4
				{
					value: Entasis_Global_Sweep_Hit = transmute(Entasis_Global_Sweep_Hit)native.sweep_hit;
					intrinsics.mem_copy_non_overlapping(&output.payload[0], &value, size_of(value));
				}
			}
			if overall == .Ok && status != .Ok
			{
				overall = status;
			}
		}
		return abi_status_finish(overall, diagnostic, .None);
	}
}
