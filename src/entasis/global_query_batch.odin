#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Query_Kind :: enum u8
	{
		Ray_Any,
		Ray_Closest,
		Ray_All,
		Sweep_Any,
		Sweep_Closest,
		Sweep_All,
		Overlap_Any,
		Overlap_All,
		Volume_All,
	}

	Global_Ray_Query :: struct
	{
		segment: World_Precision_Segment,
		filter: Query_Filter,
		output: Query_Output,
	}

	Global_Sweep_Query :: struct
	{
		shape: Shape_Handle,
		pose: Global_Pose,
		velocity: Body_Velocity,
		maximum_t: f32,
		settings: Sweep_Settings,
		filter: Query_Filter,
		output: Query_Output,
	}

	Global_Overlap_Query :: struct
	{
		shape: Shape_Handle,
		pose: Global_Pose,
		filter: Query_Filter,
		output: Query_Output,
	}

	Global_Volume_Query :: struct
	{
		bounds: Global_Bounds,
		filter: Query_Filter,
		output: Query_Output,
	}

	// only the payload selected by kind is read. distance operations retain their
	// own dense input/output spans instead of enlarging every spatial record
	Global_Query :: struct
	{
		kind: Global_Query_Kind,
		using payload: struct #raw_union
		{
			ray_data: Global_Ray_Query,
			sweep_data: Global_Sweep_Query,
			overlap_data: Global_Overlap_Query,
			volume_data: Global_Volume_Query,
		},
	}

	// all-hit counts include the valid prefix on failure. Any/closest failures
	// publish no hit. Closest misses return Not_Found, while any/all misses are Ok
	// only successful closest operations have a defined inline payload
	Global_Query_Result :: struct
	{
		status: Status,
		count: i32,
		hit: Query_Hit_State,
		using payload: struct #raw_union
		{
			ray_hit: Global_Ray_Hit,
			sweep_hit: Global_Sweep_Hit,
		},
	}

	// caller-owned output ranges and one reused traversal stack per batch owner
	// reserve global_bounds_stack_requirement entries after structural growth
	// each query selects its output subrange through the existing Query_Output
	Global_Query_Scratch :: struct
	{
		ray_hits: []Global_Ray_Hit,
		sweep_hits: []Global_Sweep_Hit,
		overlap_hits: []Overlap_Hit,
		volume_hits: []Volume_Hit,
		traversal_stack: []i32,
	}

	global_batch_ray_execute :: proc "contextless" (
		data: ^query_context_data, kind: Global_Query_Kind,
		query: ^Global_Ray_Query, scratch: Global_Query_Scratch,
	) -> Global_Query_Result
	{
		result: Global_Query_Result;
		count: int;
		if kind == .Ray_All
		{
			if !query_output_range_valid(query.output, len(scratch.ray_hits))
			{
				return {status=.Invalid_Argument};
			}
			start: int = int(query.output.start);
			hits: []Global_Ray_Hit = scratch.ray_hits[start:start + int(query.output.capacity)];
			count, result.status = query_context_global_ray_execute(data, query.segment,
				hits, scratch.traversal_stack, .All, query.filter);
		}
		else
		{
			hits: [1]Global_Ray_Hit;
			mode: physics.World_Precision_Ray_Mode = .Any if kind == .Ray_Any else .Closest;
			count, result.status = query_context_global_ray_execute(data, query.segment,
				hits[:], scratch.traversal_stack, mode, query.filter);
			if kind == .Ray_Closest
			{
				result.ray_hit = {child_index=-1};
			}
			if result.status != .Ok
			{
				return result;
			}
			if kind == .Ray_Closest
			{
				if count > 0
				{
					result.ray_hit = hits[0];
				}
				else
				{
					result.status = .Not_Found;
				}
			}
		}
		result.count = i32(count);
		result.hit = .Hit if count > 0 else .Miss;
		return result;
	}

	global_batch_sweep_execute :: proc "contextless" (
		data: ^query_context_data, kind: Global_Query_Kind,
		query: ^Global_Sweep_Query, scratch: Global_Query_Scratch,
	) -> Global_Query_Result
	{
		result: Global_Query_Result;
		count: int;
		if kind == .Sweep_All
		{
			if !query_output_range_valid(query.output, len(scratch.sweep_hits))
			{
				return {status=.Invalid_Argument};
			}
			start: int = int(query.output.start);
			hits: []Global_Sweep_Hit = scratch.sweep_hits[start:start + int(query.output.capacity)];
			count, result.status = query_context_global_sweep_execute(data, query.shape, query.pose,
				query.velocity, query.maximum_t, hits, scratch.traversal_stack, .All, query.filter, query.settings);
		}
		else
		{
			hits: [1]Global_Sweep_Hit;
			mode: physics.World_Precision_Sweep_Mode = .Any if kind == .Sweep_Any else .Closest;
			count, result.status = query_context_global_sweep_execute(data, query.shape, query.pose,
				query.velocity, query.maximum_t, hits[:], scratch.traversal_stack, mode, query.filter, query.settings);
			if result.status != .Ok
			{
				return result;
			}
			if kind == .Sweep_Closest
			{
				if count > 0
				{
					result.sweep_hit = hits[0];
				}
				else
				{
					result.status = .Not_Found;
				}
			}
		}
		result.count = i32(count);
		result.hit = .Hit if count > 0 else .Miss;
		return result;
	}

	global_batch_overlap_execute :: proc "contextless" (
		data: ^query_context_data, kind: Global_Query_Kind,
		query: ^Global_Overlap_Query, scratch: Global_Query_Scratch,
	) -> Global_Query_Result
	{
		result: Global_Query_Result;
		count: int;
		if kind == .Overlap_All
		{
			if !query_output_range_valid(query.output, len(scratch.overlap_hits))
			{
				return {status=.Invalid_Argument};
			}
			start: int = int(query.output.start);
			hits: []Overlap_Hit = scratch.overlap_hits[start:start + int(query.output.capacity)];
			count, result.status = query_context_global_overlap_execute(data, query.shape, query.pose,
				hits, scratch.traversal_stack, query.filter);
		}
		else
		{
			state: Overlap_State;
			state, result.status = query_context_global_overlap_any_execute(data, query.shape, query.pose,
				scratch.traversal_stack, query.filter);
			if result.status == .Ok && state == .Intersecting
			{
				count = 1;
			}
		}
		result.count = i32(count);
		result.hit = .Hit if count > 0 else .Miss;
		return result;
	}

	// input order is callback order. every input is attempted, including after a
	// per-item error, return the first non-Ok status (including Not_Found). result
	// capacity/context admission failures write nothing. invalid kinds/ranges fail
	// only that item, without invoking shape/filter callbacks for it. all-hit
	// failures retain a valid prefix, the suffix outside that prefix is untouched
	//
	// input, result, output and traversal memory must be disjoint, and output
	// ranges must not overlap when their results must coexist. no sorting, output
	// growth, hidden scheduling or world scratch borrowing occurs in this layer
	// existing geometry/context reservation policies still apply. keep topology,
	// poses, shapes and registrations immutable, including during callbacks
	// parallel callers partition spans and use distinct contexts, pools and stacks
	query_batch_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, queries: []Global_Query,
		results: []Global_Query_Result, scratch: Global_Query_Scratch = {},
	) -> Status
	{
		if len(results) < len(queries)
		{
			return .Invalid_Argument;
		}
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return status;
		}
		defer data.execution = .Idle;
		first: Status = .Ok;
		for &query, index in queries
		{
			result: Global_Query_Result;
			switch query.kind
			{
			case .Ray_Any, .Ray_Closest, .Ray_All:
				result = global_batch_ray_execute(data, query.kind, &query.ray_data, scratch);
			case .Sweep_Any, .Sweep_Closest, .Sweep_All:
				result = global_batch_sweep_execute(data, query.kind, &query.sweep_data, scratch);
			case .Overlap_Any, .Overlap_All:
				result = global_batch_overlap_execute(data, query.kind, &query.overlap_data, scratch);
			case .Volume_All:
				input: ^Global_Volume_Query = &query.volume_data;
				if !query_output_range_valid(input.output, len(scratch.volume_hits))
				{
					result.status = .Invalid_Argument;
					break;
				}
				start: int = int(input.output.start);
				hits: []Volume_Hit = scratch.volume_hits[start:start + int(input.output.capacity)];
				count: int;
				count, result.status = query_context_global_volume_execute(data, input.bounds,
					hits, scratch.traversal_stack, input.filter);
				result.count = i32(count);
				result.hit = .Hit if count > 0 else .Miss;
			case:
				result.status = .Invalid_Argument;
			}
			results[index] = result;
			if first == .Ok && result.status != .Ok
			{
				first = result.status;
			}
		}
		return first;
	}
}
