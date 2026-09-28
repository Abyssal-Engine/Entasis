package entasis

import physics "entasis:entasis_physics"

// Profile_Stage identifies one low-level timestep stage. values and
// ordering match the low-level profiler exactly
Profile_Stage :: physics.Simulation_Stage;

// Profile_Snapshot is a pointer-free copy of the most recently completed step's
// stage timings plus cumulative stage invocation counts. durations are in
// nanoseconds. profiling must be enabled in World_Description before world_init
Profile_Snapshot :: struct
{
	trace:                       [64]Profile_Stage,
	stage_counts:                [len(Profile_Stage)]u64,
	stage_durations_nanoseconds: [len(Profile_Stage)]i64,
	trace_count:                 int,
	step_index:                  u64,
}

// World_Stats is a pointer-free snapshot of world storage counts. it allocates
// nothing and exposes no internal set, batch, or pool pointers
World_Stats :: struct
{
	step_index:             u64,
	active_bodies:          int,
	sleeping_bodies:        int,
	sleeping_islands:       int,
	statics:                int,
	active_constraints:     int,
	sleeping_constraints:   int,
	active_pairs:           int,
	inactive_pairs:         int,
	registered_shapes:      int,
	registered_shape_types: int,
}

// profile_stage_text returns one allocation-free static stage name
profile_stage_text :: proc "contextless" (stage: Profile_Stage) -> string
{
	switch stage
	{
		case .Timestep:
			return "timestep";
		case .Sleep:
			return "sleep";
		case .Slept_Callback:
			return "slept_callback";
		case .Predict_Bounding_Boxes:
			return "predict_bounding_boxes";
		case .Before_Collision_Callback:
			return "before_collision_callback";
		case .Broad_Phase:
			return "broad_phase";
		case .Collision_Detection:
			return "collision_detection";
		case .Collisions_Detected_Callback:
			return "collisions_detected_callback";
		case .Solve:
			return "solve";
		case .Constraints_Solved_Callback:
			return "constraints_solved_callback";
		case .Incrementally_Optimize:
			return "incrementally_optimize";
		case .Cleanup:
			return "cleanup";
	}
	return "unknown";
}

// world_profile_enabled reports whether profiling was enabled at world creation
world_profile_enabled :: proc "contextless" (world: ^World) -> (bool, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return false, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return false, .Invalid_Argument;
	}
	return data.simulation.profiler.state == .Enabled, .Ok;
}

// world_profile_snapshot copies the most recently completed profile. it
// allocates nothing and is owner-thread only while the world is idle
world_profile_snapshot :: proc "contextless" (
	world: ^World,
) -> (Profile_Snapshot, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	profiler := &data.simulation.profiler;
	if profiler.state != .Enabled
	{
		return {}, .Invalid_Description;
	}
	result := Profile_Snapshot{
		trace_count=min(profiler.trace_count, len(profiler.trace)),
		step_index=data.simulation.step_index,
	};
	result.trace = profiler.trace;
	result.stage_counts = profiler.stage_counts;
	result.stage_durations_nanoseconds = profiler.stage_durations_nanoseconds;
	return result, .Ok;
}

// world_stats returns stable read-only counts while the world is idle
// allocation: none. thread safety: owner thread only
world_stats :: proc "contextless" (world: ^World) -> (World_Stats, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	simulation := &data.simulation;
	active_count := simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].count;
	sleeping_count := 0;
	sleeping_island_count := 0;
	for set_index in 1 ..< simulation.bodies.sets.length
	{
		set := &simulation.bodies.sets.memory[set_index];
		if set.state == .Allocated
		{
			sleeping_count += set.count;
			if set.count > 0
			{
				sleeping_island_count += 1;
			}
		}
	}
	shape_count := 0;
	registered_shape_types := 0;
	registry := physics.simulation_shape_registry(simulation);
	if registry != nil && registry.state == .Allocated
	{
		registered_shape_types = registry.registered_type_count;
		for type_index in 0 ..< registry.registered_type_count
		{
			batch := &registry.batches[type_index];
			if batch.state == .Registered
			{
				shape_count += batch.active_count;
			}
		}
	}
	return {
		step_index=simulation.step_index,
		active_bodies=active_count,
		sleeping_bodies=sleeping_count,
		sleeping_islands=sleeping_island_count,
		statics=simulation.statics.count,
		active_constraints=int(simulation.solver.active_set.constraint_count),
		sleeping_constraints=simulation.sleeper.inactive_constraint_count,
		active_pairs=simulation.narrow_phase.pair_cache.mapping.count,
		inactive_pairs=simulation.narrow_phase.pair_cache.inactive_count,
		registered_shapes=shape_count,
		registered_shape_types=registered_shape_types,
	}, .Ok;
}

// World_Solver_Stats is a pointer-free snapshot of active solver batch counts.
// it is intended for diagnostics and benchmark fixture validation outside timed regions
World_Solver_Stats :: struct
{
	active_batches:       int,
	fallback_batch_index: int,
	fallback_constraints: int,
}

// world_solver_stats returns read-only solver layout counts while the world is idle
// allocation: none. thread safety: owner thread only
world_solver_stats :: #force_inline proc "contextless" (
	world: ^World,
) -> (World_Solver_Stats, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	solver := &data.simulation.solver;
	fallback_index := int(solver.fallback_batch_index);
	fallback_count := 0;
	if fallback_index >= 0 && fallback_index < int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[fallback_index];
		if batch.state == .Allocated
		{
			fallback_count = int(batch.constraint_count);
		}
	}
	return {
		active_batches=int(solver.active_set.batch_count),
		fallback_batch_index=fallback_index,
		fallback_constraints=fallback_count,
	}, .Ok;
}
