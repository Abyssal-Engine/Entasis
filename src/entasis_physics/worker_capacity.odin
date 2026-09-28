// grow worker scratch only when a dispatcher exceeds the simulation's current
// initial-capacity hint. these cold paths are kept outside hot stage files
package entasis_physics

import util "entasis:entasis_utilities"

broad_phase_ensure_worker_capacity :: #force_no_inline proc (
	broad_phase: ^Broad_Phase, worker_count, candidate_capacity: int,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || worker_count <= 0 ||
		worker_count > MAXIMUM_SOLVER_WORKER_COUNT || candidate_capacity <= 0
	{
		return .Invalid_Argument;
	}
	if worker_count <= broad_phase.query_workspace.worker_count &&
		worker_count <= broad_phase.maintenance_workspace.worker_count
	{
		return .Ok;
	}
	target_worker_count := max(
		worker_count,
		max(
		broad_phase.query_workspace.worker_count,
		broad_phase.maintenance_workspace.worker_count,
	),
	);
	maximum_leaves := max(
		3,
		max(
		broad_phase.maintenance_workspace.maximum_leaves,
		max(
		broad_phase.active_tree.leaf_count,
		broad_phase.static_tree.leaf_count,
	),
	),
	);
	new_query_workspace: Tree_Parallel_Query_Workspace;
	status := tree_parallel_query_workspace_initialize(
		&new_query_workspace, target_worker_count, maximum_leaves,
		candidate_capacity, broad_phase.pool,
	);
	if status != .Ok
	{
		return status;
	}
	new_maintenance_workspace: Tree_Refit_Refine_Workspace;
	status = tree_refit_refine_workspace_initialize(
		&new_maintenance_workspace, target_worker_count, maximum_leaves,
		broad_phase.pool, broad_phase.maintenance_workspace.treelet_size,
	);
	if status != .Ok
	{
		_ = tree_parallel_query_workspace_dispose(&new_query_workspace);
		return status;
	}
	old_query_workspace := broad_phase.query_workspace;
	old_maintenance_workspace := broad_phase.maintenance_workspace;
	broad_phase.query_workspace = new_query_workspace;
	broad_phase.maintenance_workspace = new_maintenance_workspace;
	_ = tree_parallel_query_workspace_dispose(&old_query_workspace);
	_ = tree_refit_refine_workspace_dispose(&old_maintenance_workspace);
	return .Ok;
}

narrow_phase_ensure_worker_capacity :: #force_no_inline proc (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || worker_count <= 0 ||
		worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	if worker_count <= narrow.active_worker_count
	{
		return .Ok;
	}
	if narrow.pending_capacity_per_worker <= 0 ||
		narrow.pending_capacity_per_worker > max(int) / worker_count
	{
		return .Capacity_Missing;
	}
	pending_total_capacity := narrow.pending_capacity_per_worker * worker_count;
	stream_status := narrow_phase_ensure_transaction_stream_capacity(
		narrow, max(narrow.current_transaction_capacity, 1),
		max(int(narrow.pair_cache.mapping.keys.length), 1), worker_count,
	);
	if stream_status != .Ok
	{
		return stream_status;
	}
	target_child_capacity := max(narrow.collision_child_capacity, worker_count);
	new_candidates, candidate_status := util.buffer_pool_take_at_least(
		narrow.pool, Broad_Phase_Pair, pending_total_capacity,
	);
	if candidate_status != .Ok
	{
		return physics_memory_status(candidate_status);
	}
	new_collision_storage, storage_status := util.buffer_pool_take_at_least(
		narrow.pool, Collision_Batcher_Pair, pending_total_capacity,
	);
	if storage_status != .Ok
	{
		physics_return_buffer(narrow.pool, &new_candidates);
		return physics_memory_status(storage_status);
	}
	new_continuations, continuation_status := util.buffer_pool_take_at_least(
		narrow.pool, Narrow_Phase_CCD_Continuation, pending_total_capacity,
	);
	if continuation_status != .Ok
	{
		physics_return_buffer(narrow.pool, &new_collision_storage);
		physics_return_buffer(narrow.pool, &new_candidates);
		return physics_memory_status(continuation_status);
	}
	new_results, result_status := util.buffer_pool_take_at_least(
		narrow.pool, Narrow_Phase_Pair_Result, pending_total_capacity,
	);
	if result_status != .Ok
	{
		physics_return_buffer(narrow.pool, &new_continuations);
		physics_return_buffer(narrow.pool, &new_collision_storage);
		physics_return_buffer(narrow.pool, &new_candidates);
		return physics_memory_status(result_status);
	}
	new_scratch, scratch_status := collision_batcher_take_scratch(
		narrow.pool, pending_total_capacity, target_child_capacity,
	);
	if scratch_status != .Ok
	{
		physics_return_buffer(narrow.pool, &new_results);
		physics_return_buffer(narrow.pool, &new_continuations);
		physics_return_buffer(narrow.pool, &new_collision_storage);
		physics_return_buffer(narrow.pool, &new_candidates);
		return scratch_status;
	}
	old_candidates := narrow.candidates;
	old_collision_storage := narrow.collision_storage;
	old_continuations := narrow.continuations;
	old_results := narrow.results;
	old_scratch := narrow.collision_scratch;
	old_worker_count := narrow.active_worker_count;
	old_child_capacity := narrow.collision_child_capacity;
	narrow.candidates = new_candidates;
	narrow.collision_storage = new_collision_storage;
	narrow.continuations = new_continuations;
	narrow.results = new_results;
	narrow.collision_scratch = new_scratch;
	narrow.collision_child_capacity = target_child_capacity;
	narrow.active_worker_count = worker_count;
	for worker_index in old_worker_count ..< worker_count
	{
		narrow.worker_contexts[worker_index] = {
			narrow=narrow,
			worker_index=i32(worker_index),
		};
	}
	status := narrow_phase_prepare_batchers(narrow, worker_count);
	if status != .Ok
	{
		narrow.active_worker_count = old_worker_count;
		narrow.collision_child_capacity = old_child_capacity;
		narrow.collision_scratch = old_scratch;
		narrow.results = old_results;
		narrow.continuations = old_continuations;
		narrow.collision_storage = old_collision_storage;
		narrow.candidates = old_candidates;
		_ = narrow_phase_prepare_batchers(narrow, old_worker_count);
		collision_batcher_return_scratch(narrow.pool, &new_scratch);
		physics_return_buffer(narrow.pool, &new_results);
		physics_return_buffer(narrow.pool, &new_continuations);
		physics_return_buffer(narrow.pool, &new_collision_storage);
		physics_return_buffer(narrow.pool, &new_candidates);
		return status;
	}
	collision_batcher_return_scratch(narrow.pool, &old_scratch);
	physics_return_buffer(narrow.pool, &old_results);
	physics_return_buffer(narrow.pool, &old_continuations);
	physics_return_buffer(narrow.pool, &old_collision_storage);
	physics_return_buffer(narrow.pool, &old_candidates);
	return .Ok;
}

island_sleeper_ensure_worker_capacity :: #force_no_inline proc (
	sleeper: ^Island_Sleeper, worker_count: int,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready || worker_count <= 0 ||
		worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	if worker_count <= sleeper.worker_count
	{
		return .Ok;
	}
	body_capacity := max(int(sleeper.scaffold.body_marks.length), 1);
	constraint_capacity := max(
		int(sleeper.scaffold.constraint_marks.length), 1,
	);
	initialized_start := sleeper.worker_count;
	for worker_index in initialized_start ..< worker_count
	{
		status := island_scaffold_initialize(
			&sleeper.worker_scaffolds[worker_index], body_capacity,
			constraint_capacity, sleeper.pool,
		);
		if status != .Ok
		{
			for release_index in initialized_start ..< worker_index
			{
				_ = island_scaffold_dispose(
					&sleeper.worker_scaffolds[release_index],
				);
			}
			return status;
		}
	}
	sleeper.worker_count = worker_count;
	return .Ok;
}

simulation_ensure_worker_capacity :: #force_no_inline proc (
	simulation: ^Simulation, worker_count: int,
) -> Physics_Status
{
	if simulation == nil ||
		(simulation.state != .Ready && simulation.state != .Stepping)
	{
		return .Disposed;
	}
	if worker_count <= 0 || worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	if worker_count <= int(simulation.allocation_sizes.workers)
	{
		return .Ok;
	}
	status := broad_phase_ensure_worker_capacity(
		&simulation.broad_phase, worker_count,
		max(int(simulation.allocation_sizes.broad_phase_candidates), 1),
	);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_ensure_worker_capacity(
		&simulation.narrow_phase, worker_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = island_sleeper_ensure_worker_capacity(
		&simulation.sleeper, worker_count,
	);
	if status != .Ok
	{
		return status;
	}
	simulation.allocation_sizes.workers = i32(worker_count);
	simulation.allocation_sizes.collision_child_pairs = max(
		simulation.allocation_sizes.collision_child_pairs,
		i32(worker_count),
	);
	return .Ok;
}

simulation_stage_validate_worker_capacity_cold :: #force_no_inline proc (
	simulation: ^Simulation, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	if simulation == nil || dispatcher == nil || dispatcher.dispatch == nil ||
		dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	if simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	return simulation_ensure_worker_capacity(
		simulation, dispatcher.worker_count,
	);
}
