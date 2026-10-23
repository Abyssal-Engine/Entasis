#+vet !unused-imports

package entasis

import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Sweep_Result :: physics.Global_Sweep_Result;
	Global_Sweep_Hit :: struct
	{
		collidable: Collidable_Reference,
		sweep: Global_Sweep_Result,
	}

	#assert(size_of(Global_Sweep_Hit) == size_of(physics.Global_Sweep_Query_Hit));
	#assert(align_of(Global_Sweep_Hit) == align_of(physics.Global_Sweep_Query_Hit));
	#assert(offset_of(Global_Sweep_Hit, collidable) == offset_of(physics.Global_Sweep_Query_Hit, target_id));
	#assert(offset_of(Global_Sweep_Hit, sweep) == offset_of(physics.Global_Sweep_Query_Hit, sweep));

	query_context_global_sweep_execute :: proc "contextless" (
		data: ^query_context_data, shape: Shape_Handle, pose: Global_Pose,
		velocity: Body_Velocity, maximum_t: f32, hits: []Global_Sweep_Hit, traversal_stack: []i32,
		mode: physics.World_Precision_Sweep_Mode, filter: Query_Filter, settings: Sweep_Settings,
	) -> (int, Status)
	{
		status: Status = physics.world_precision_sweep_admit(&data.world.simulation, pose, velocity, maximum_t, traversal_stack);
		if status != .Ok
		{
			return 0, status;
		}
		// unlike a finite maximum_t check alone, these reject nonfinite settings
		// before a registered bounds callback can observe the request
		if !(settings.minimum_progression >= 0 && settings.minimum_progression <= math.F32_MAX &&
			settings.convergence_threshold >= 0 && settings.convergence_threshold <= math.F32_MAX) ||
			settings.maximum_iteration_count < 0
		{
			return 0, .Invalid_Description;
		}
		resolved: Sweep_Settings;
		resolved, status = sweep_settings_resolve(data.world, shape, velocity, maximum_t, settings);
		if status != .Ok
		{
			return 0, status;
		}
		filter_context: query_filter_context = {filter=filter};
		storage: []physics.Global_Sweep_Query_Hit;
		if len(hits) > 0
		{
			storage = ([^]physics.Global_Sweep_Query_Hit)(rawptr(&hits[0]))[:len(hits)];
		}
		return physics.simulation_global_sweep_query(&data.world.simulation, shape, pose, velocity,
			maximum_t, resolved.minimum_progression, resolved.convergence_threshold, int(resolved.maximum_iteration_count),
			mode, storage, traversal_stack, data.pool, query_filter_low_level(filter), &filter_context);
	}

	// local sweeps at high coordinates: subtract authoritative positions in f64,
	// then reuse f32 shape/motion kernels in a query-centered frame. times retain
	// their original units/precision, location is reconstructed in f64. targets are
	// stationary as in the original API. registered tasks receive the local frame
	// reserve global_bounds_stack_requirement entries before recurring work. the
	// context/pool, stack and output have exclusive owners, the world and callback
	// registries stay immutable during a query. f64 positions do not increase the
	// motion kernel's accuracy for long rotating sweeps
	sweep_global_closest_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape: Shape_Handle, pose: Global_Pose,
		velocity: Body_Velocity, maximum_t: f32, traversal_stack: []i32,
		filter: Query_Filter = {}, settings: Sweep_Settings = {},
	) -> (Global_Sweep_Hit, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return {}, status;
		}
		defer data.execution = .Idle;
		hits: [1]Global_Sweep_Hit;
		count: int;
		count, status = query_context_global_sweep_execute(data, shape, pose, velocity, maximum_t,
			hits[:], traversal_stack, .Closest, filter, settings);
		if status != .Ok
		{
			return {}, status;
		}
		if count == 0
		{
			return {}, .Not_Found;
		}
		return hits[0], .Ok;
	}

	sweep_global_any_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape: Shape_Handle, pose: Global_Pose,
		velocity: Body_Velocity, maximum_t: f32, traversal_stack: []i32,
		filter: Query_Filter = {}, settings: Sweep_Settings = {},
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
		hits: [1]Global_Sweep_Hit;
		count: int;
		count, status = query_context_global_sweep_execute(data, shape, pose, velocity, maximum_t,
			hits[:], traversal_stack, .Any, filter, settings);
		if status != .Ok
		{
			return .Miss, status;
		}
		return .Hit if count != 0 else .Miss, .Ok;
	}

	// all means the first impact per permitted collidable, matching the existing
	// API, not every surface/child along its motion. order is unspecified. errors
	// preserve the valid written prefix and leave all storage beyond count intact
	sweep_global_all_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape: Shape_Handle, pose: Global_Pose,
		velocity: Body_Velocity, maximum_t: f32, hits: []Global_Sweep_Hit, traversal_stack: []i32,
		filter: Query_Filter = {}, settings: Sweep_Settings = {},
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
		return query_context_global_sweep_execute(data, shape, pose, velocity, maximum_t,
			hits, traversal_stack, .All, filter, settings);
	}
}
