package entasis_physics

import util "entasis:entasis_utilities"

broad_phase_initialize :: proc (
	broad_phase: ^Broad_Phase, active_capacity, static_capacity, candidate_capacity, worker_count: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if broad_phase == nil || pool == nil || active_capacity <= 0 || static_capacity <= 0 ||
	candidate_capacity <= 0 || worker_count <= 0
	{
		return .Invalid_Argument;
	}
	broad_phase^ = {pool=pool};
	status := tree_initialize(&broad_phase.active_tree, active_capacity, pool);
	if status != .Ok
	{
		broad_phase^ = {};
		return status;
	}
	status = tree_initialize(&broad_phase.static_tree, static_capacity, pool);
	if status != .Ok
	{
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return status;
	}
	active_leaves, active_status := util.buffer_pool_take_at_least(pool, Collidable_Reference, active_capacity);
	if active_status != .Ok
	{
		_ = tree_dispose(&broad_phase.static_tree);
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return physics_memory_status(active_status);
	}
	broad_phase.active_leaves = active_leaves;
	static_leaves, static_status := util.buffer_pool_take_at_least(pool, Collidable_Reference, static_capacity);
	if static_status != .Ok
	{
		physics_return_buffer(pool, &broad_phase.active_leaves);
		_ = tree_dispose(&broad_phase.static_tree);
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return physics_memory_status(static_status);
	}
	broad_phase.static_leaves = static_leaves;
	maximum_leaves := max(active_capacity, static_capacity);
	leaves, leaf_status := util.buffer_pool_take_at_least(pool, i32, maximum_leaves);
	if leaf_status != .Ok
	{
		physics_return_buffer(pool, &broad_phase.static_leaves);
		physics_return_buffer(pool, &broad_phase.active_leaves);
		_ = tree_dispose(&broad_phase.static_tree);
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return physics_memory_status(leaf_status);
	}
	broad_phase.leaf_scratch = leaves;
	workspace_status := tree_parallel_query_workspace_initialize(
		&broad_phase.query_workspace, worker_count, maximum_leaves, candidate_capacity, pool,
	);
	if workspace_status != .Ok
	{
		physics_return_buffer(pool, &broad_phase.leaf_scratch);
		physics_return_buffer(pool, &broad_phase.static_leaves);
		physics_return_buffer(pool, &broad_phase.active_leaves);
		_ = tree_dispose(&broad_phase.static_tree);
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return workspace_status;
	}
	maintenance_status := tree_refit_refine_workspace_initialize(
		&broad_phase.maintenance_workspace, worker_count, maximum_leaves, pool,
	);
	if maintenance_status != .Ok
	{
		_ = tree_parallel_query_workspace_dispose(&broad_phase.query_workspace);
		physics_return_buffer(pool, &broad_phase.leaf_scratch);
		physics_return_buffer(pool, &broad_phase.static_leaves);
		physics_return_buffer(pool, &broad_phase.active_leaves);
		_ = tree_dispose(&broad_phase.static_tree);
		_ = tree_dispose(&broad_phase.active_tree);
		broad_phase^ = {};
		return maintenance_status;
	}
	broad_phase.state = .Ready;
	return .Ok;
}

broad_phase_bind_owners :: proc "contextless" (
	broad_phase: ^Broad_Phase, shapes: ^Shape_Registry, bodies: ^Bodies, statics: ^Statics,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.active_tree.leaf_count != 0 ||
	broad_phase.static_tree.leaf_count != 0 || shapes == nil || shapes.state != .Allocated ||
	bodies == nil || bodies.state != .Allocated || statics == nil || statics.state != .Allocated ||
	bodies.shapes != shapes || statics.shapes != shapes
	{
		return .Invalid_Argument;
	}
	broad_phase.shapes = shapes;
	broad_phase.bodies = bodies;
	broad_phase.statics = statics;
	return .Ok;
}

broad_phase_ensure_capacity :: proc (
	broad_phase: ^Broad_Phase, active_capacity, static_capacity,
	candidate_capacity: int,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready ||
	active_capacity <= 0 || static_capacity <= 0 ||
	candidate_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := tree_ensure_capacity(&broad_phase.active_tree, active_capacity);
	if status != .Ok
	{
		return status;
	}
	status = tree_ensure_capacity(&broad_phase.static_tree, static_capacity);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		broad_phase.pool, &broad_phase.active_leaves, active_capacity,
		broad_phase.active_tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		broad_phase.pool, &broad_phase.static_leaves, static_capacity,
		broad_phase.static_tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	maximum_leaves := max(active_capacity, static_capacity);
	status = physics_ensure_buffer_capacity(
		broad_phase.pool, &broad_phase.leaf_scratch, maximum_leaves, 0,
	);
	if status != .Ok
	{
		return status;
	}
	if maximum_leaves > broad_phase.query_workspace.maximum_jobs / 2
	{
		new_query_workspace: Tree_Parallel_Query_Workspace;
		status = tree_parallel_query_workspace_initialize(
			&new_query_workspace, broad_phase.query_workspace.worker_count,
			maximum_leaves, candidate_capacity, broad_phase.pool,
		);
		if status != .Ok
		{
			return status;
		}
		old_query_workspace := broad_phase.query_workspace;
		broad_phase.query_workspace = new_query_workspace;
		_ = tree_parallel_query_workspace_dispose(&old_query_workspace);
	}
	if maximum_leaves > broad_phase.maintenance_workspace.maximum_leaves
	{
		new_maintenance_workspace: Tree_Refit_Refine_Workspace;
		status = tree_refit_refine_workspace_initialize(
			&new_maintenance_workspace,
			broad_phase.maintenance_workspace.worker_count,
			maximum_leaves, broad_phase.pool,
			broad_phase.maintenance_workspace.treelet_size,
		);
		if status != .Ok
		{
			return status;
		}
		old_maintenance_workspace := broad_phase.maintenance_workspace;
		broad_phase.maintenance_workspace = new_maintenance_workspace;
		// the procedure table is embedded in the workspace. rebind after moving the
		// initialized temporary so later threaded updates never borrow its stack
		broad_phase.maintenance_workspace.task_stack.procedures.memory =
		&broad_phase.maintenance_workspace.task_procedures[0];
		_ = tree_refit_refine_workspace_dispose(&old_maintenance_workspace);
	}
	return .Ok;
}

broad_phase_resize :: proc (
	broad_phase: ^Broad_Phase, active_capacity, static_capacity,
	candidate_capacity: int,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready ||
	active_capacity <= 0 || static_capacity <= 0 ||
	candidate_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := broad_phase_ensure_capacity(
		broad_phase, active_capacity, static_capacity, candidate_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = tree_resize(&broad_phase.active_tree, active_capacity);
	if status != .Ok
	{
		return status;
	}
	status = tree_resize(&broad_phase.static_tree, static_capacity);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		broad_phase.pool, &broad_phase.active_leaves,
		max(active_capacity, broad_phase.active_tree.leaf_count),
		broad_phase.active_tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		broad_phase.pool, &broad_phase.static_leaves,
		max(static_capacity, broad_phase.static_tree.leaf_count),
		broad_phase.static_tree.leaf_count,
	);
	if status != .Ok
	{
		return status;
	}
	maximum_leaves := max(
		max(active_capacity, broad_phase.active_tree.leaf_count),
		max(static_capacity, broad_phase.static_tree.leaf_count),
	);
	status = physics_resize_buffer_capacity(
		broad_phase.pool, &broad_phase.leaf_scratch, maximum_leaves, 0,
	);
	if status != .Ok
	{
		return status;
	}
	new_query_workspace: Tree_Parallel_Query_Workspace;
	status = tree_parallel_query_workspace_initialize(
		&new_query_workspace, broad_phase.query_workspace.worker_count,
		maximum_leaves, candidate_capacity, broad_phase.pool,
	);
	if status != .Ok
	{
		return status;
	}
	new_maintenance_workspace: Tree_Refit_Refine_Workspace;
	status = tree_refit_refine_workspace_initialize(
		&new_maintenance_workspace,
		broad_phase.maintenance_workspace.worker_count,
		maximum_leaves, broad_phase.pool,
		broad_phase.maintenance_workspace.treelet_size,
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
	// the procedure table is embedded in the workspace. rebind after moving the
	// initialized temporary so later threaded updates never borrow its stack
	broad_phase.maintenance_workspace.task_stack.procedures.memory =
	&broad_phase.maintenance_workspace.task_procedures[0];
	_ = tree_parallel_query_workspace_dispose(&old_query_workspace);
	_ = tree_refit_refine_workspace_dispose(&old_maintenance_workspace);
	return .Ok;
}

broad_phase_clear :: proc (broad_phase: ^Broad_Phase) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready
	{
		return .Disposed;
	}
	status := tree_clear(&broad_phase.active_tree);
	if status != .Ok
	{
		return status;
	}
	status = tree_clear(&broad_phase.static_tree);
	if status != .Ok
	{
		return status;
	}
	_ = util.buffer_clear(
		broad_phase.active_leaves, 0, int(broad_phase.active_leaves.length),
	);
	_ = util.buffer_clear(
		broad_phase.static_leaves, 0, int(broad_phase.static_leaves.length),
	);
	broad_phase.frame_index = 0;
	broad_phase.active_subtree_refinement_start_index = 0;
	broad_phase.static_subtree_refinement_start_index = 0;
	broad_phase.bounds_state = .Current;
	return .Ok;
}

broad_phase_dispose :: proc (broad_phase: ^Broad_Phase) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || broad_phase.pool == nil
	{
		return .Disposed;
	}
	pool := broad_phase.pool;
	_ = tree_refit_refine_workspace_dispose(&broad_phase.maintenance_workspace);
	_ = tree_parallel_query_workspace_dispose(&broad_phase.query_workspace);
	physics_return_buffer(pool, &broad_phase.leaf_scratch);
	physics_return_buffer(pool, &broad_phase.static_leaves);
	physics_return_buffer(pool, &broad_phase.active_leaves);
	_ = tree_dispose(&broad_phase.static_tree);
	_ = tree_dispose(&broad_phase.active_tree);
	broad_phase^ = {state=.Disposed};
	return .Ok;
}
