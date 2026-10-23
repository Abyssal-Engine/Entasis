#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	World_Precision_Segment :: physics.World_Precision_Segment;

	// fraction along the original closed segment, current-frame f64 hit position,
	// and the unchanged local-float normal/child identity
	Global_Ray_Hit :: struct
	{
		t: f64,
		location: Global_Position,
		normal: Vector3,
		collidable: Collidable_Reference,
		child_index: i32,
	}

	#assert(size_of(Global_Ray_Hit) == size_of(physics.Global_Ray_Query_Hit));
	#assert(align_of(Global_Ray_Hit) == align_of(physics.Global_Ray_Query_Hit));
	#assert(offset_of(Global_Ray_Hit, collidable) == offset_of(physics.Global_Ray_Query_Hit, target_id));
	#assert(offset_of(Global_Ray_Hit, child_index) == offset_of(physics.Global_Ray_Query_Hit, child_index));

	// reserve this many i32 entries outside recurring work. a structural world
	// mutation may increase the requirement. queries never resize caller storage
	global_ray_stack_requirement :: proc "contextless" (world: ^World) -> (int, Status)
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

	query_context_global_ray_execute :: proc "contextless" (
		data: ^query_context_data, segment: World_Precision_Segment,
		hits: []Global_Ray_Hit, traversal_stack: []i32,
		mode: physics.World_Precision_Ray_Mode, filter: Query_Filter,
	) -> (int, Status)
	{
		filter_context: query_filter_context = {filter=filter};
		storage: []physics.Global_Ray_Query_Hit;
		if len(hits) > 0
		{
			storage = ([^]physics.Global_Ray_Query_Hit)(rawptr(&hits[0]))[:len(hits)];
		}
		return physics.simulation_global_ray_query(
			&data.world.simulation, segment, mode, storage, traversal_stack, data.pool,
			query_filter_low_level(filter), &filter_context,
		);
	}

	// these high-coordinate entry points use a caller-owned Query_Context and
	// traversal stack. independent contexts/stacks may read the same immutable
	// world concurrently. filters are read-only. custom shape ray callbacks receive
	// a target-centered float frame explicitly, legacy float queries do not change
	ray_cast_global_closest_with_context :: proc "contextless" (
		query_context: ^Query_Context, segment: World_Precision_Segment,
		traversal_stack: []i32, filter: Query_Filter = {},
	) -> (Global_Ray_Hit, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return {child_index=-1}, status;
		}
		defer data.execution = .Idle;
		hits: [1]Global_Ray_Hit;
		count: int;
		count, status = query_context_global_ray_execute(data, segment, hits[:], traversal_stack, .Closest, filter);
		if status != .Ok
		{
			return {child_index=-1}, status;
		}
		if count == 0
		{
			return {child_index=-1}, .Not_Found;
		}
		return hits[0], .Ok;
	}

	ray_cast_global_any_with_context :: proc "contextless" (
		query_context: ^Query_Context, segment: World_Precision_Segment,
		traversal_stack: []i32, filter: Query_Filter = {},
	) -> (Query_Hit_State, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return .Miss, status;
		}
		defer data.execution = .Idle;
		hits: [1]Global_Ray_Hit;
		count: int;
		count, status = query_context_global_ray_execute(data, segment, hits[:], traversal_stack, .Any, filter);
		if status != .Ok
		{
			return .Miss, status;
		}
		return .Hit if count != 0 else .Miss, .Ok;
	}

	// empty output is valid. a real hit then reports Capacity_Missing, whereas a
	// miss returns zero/Ok. on a traversal/filter/shape/capacity failure, count is
	// the valid prefix already written. entries beyond count remain untouched
	ray_cast_global_all_with_context :: proc "contextless" (
		query_context: ^Query_Context, segment: World_Precision_Segment,
		hits: []Global_Ray_Hit, traversal_stack: []i32, filter: Query_Filter = {},
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
		return query_context_global_ray_execute(data, segment, hits, traversal_stack, .All, filter);
	}
}
