package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"
import "core:sync"

Tree_Maintenance_Task_Kind :: enum u8
{
	Refit,
	Refine,
	Root_Refine,
	Subtree_Refine,
	Cache_Copy,
}
Tree_Maintenance_Tree :: enum u8
{
	Active,
	Static,
}
Tree_Task_Procedure :: enum u16
{
	Update2_Entry,
	Refinement,
	Binned_Node,
	Binned_Centroid,
	Binned_Bin,
	Binned_Partition,
	Reify,
	Cache_Copy,
	Request_Stop,
}
TREE_TASK_PROCEDURE_COUNT :: int(Tree_Task_Procedure.Request_Stop) + 1;
TREE_BINNED_MIN_SUBTREES_PER_NODE_TASK :: 256;
TREE_BINNED_NODE_TASK_MULTIPLIER :: 8;
TREE_BINNED_MIN_SUBTREES_PER_INNER_TASK :: 1024;
Tree_Refinement_Schedule :: struct
{
	root_refinement_size:    int,
	subtree_refinement_count: int,
	subtree_refinement_size: int,
	use_priority_queue:      Reference_State,
}
Tree_Maintenance_Task :: struct
{
	node_index:        i32,
	target_node_index: i32,
	parent_node_index: i32,
	index_in_parent:   i32,
	kind:              Tree_Maintenance_Task_Kind,
	tree_kind:         Tree_Maintenance_Tree,
}
Tree_Refinement_Root_Stack_Entry :: struct
{
	node_index:     i32,
	subtree_budget: i32,
}
Tree_Refinement_Heap_Entry :: struct
{
	node_index: i32,
	cost:       f32,
}
Tree_Refit_Refine_Workspace :: struct
{
	refit_nodes:                 util.Buffer(i32),
	refinement_candidates:       util.Buffer(i32),
	refinement_targets:          util.Buffer(i32),
	static_refinement_targets:   util.Buffer(i32),
	task_records:                util.Buffer(Tree_Maintenance_Task),
	static_task_records:         util.Buffer(Tree_Maintenance_Task),
	binned_centroid_bounds:      util.Buffer(util.Bounding_Box),
	binned_bins:                 util.Buffer(Tree_Bin),
	binned_bin_indices:          util.Buffer(u8),
	references:                  util.Buffer(Tree_Build_Reference),
	scratch:                     util.Buffer(Tree_Build_Reference),
	refinement_subtrees:         util.Buffer(Tree_Node_Child),
	refinement_subtree_scratch: util.Buffer(Tree_Node_Child),
	temporary_nodes:             util.Buffer(Tree_Node),
	node_indices:                util.Buffer(i32),
	root_stack_entries:          util.Buffer(Tree_Refinement_Root_Stack_Entry),
	heap_entries:                util.Buffer(Tree_Refinement_Heap_Entry),
	cache_nodes:                 util.Buffer(Tree_Node),
	cache_metanodes:             util.Buffer(Tree_Metanode),
	statuses:                    util.Buffer(Physics_Status),
	worker_hits:                 util.Buffer(i32),
	task_stack:                  util.Task_Stack,
	task_procedures:             [TREE_TASK_PROCEDURE_COUNT]util.Task_Procedure,
	worker_count:                int,
	maximum_leaves:              int,
	treelet_size:                int,
	refinement_capacity:         int,
	binned_inner_slot_count:     int,
	last_refinement_offset:      i32,
	last_refinement_target_count: i32,
	pool:                        ^util.Buffer_Pool,
	state:                       Tree_State,
}
tree_refit_refine_workspace_return_buffers :: proc (workspace: ^Tree_Refit_Refine_Workspace, pool: ^util.Buffer_Pool)
{
	if workspace.task_stack.state == .Ready
	{
		_ = util.task_stack_dispose(&workspace.task_stack, pool);
	}
	physics_return_buffer(pool, &workspace.worker_hits);
	physics_return_buffer(pool, &workspace.statuses);
	physics_return_buffer(pool, &workspace.cache_metanodes);
	physics_return_buffer(pool, &workspace.cache_nodes);
	physics_return_buffer(pool, &workspace.heap_entries);
	physics_return_buffer(pool, &workspace.root_stack_entries);
	physics_return_buffer(pool, &workspace.node_indices);
	physics_return_buffer(pool, &workspace.temporary_nodes);
	physics_return_buffer(pool, &workspace.refinement_subtree_scratch);
	physics_return_buffer(pool, &workspace.refinement_subtrees);
	physics_return_buffer(pool, &workspace.scratch);
	physics_return_buffer(pool, &workspace.references);
	physics_return_buffer(pool, &workspace.binned_bin_indices);
	physics_return_buffer(pool, &workspace.binned_bins);
	physics_return_buffer(pool, &workspace.binned_centroid_bounds);
	physics_return_buffer(pool, &workspace.static_task_records);
	physics_return_buffer(pool, &workspace.task_records);
	physics_return_buffer(pool, &workspace.static_refinement_targets);
	physics_return_buffer(pool, &workspace.refinement_targets);
	physics_return_buffer(pool, &workspace.refinement_candidates);
	physics_return_buffer(pool, &workspace.refit_nodes);
}
tree_refit_refine_workspace_initialize :: proc (
	workspace: ^Tree_Refit_Refine_Workspace, worker_count, maximum_leaves: int,
	pool: ^util.Buffer_Pool, treelet_size: int = TREE_DEFAULT_TREELET_SIZE,
) -> Physics_Status
{
	if workspace == nil || pool == nil || worker_count <= 0 || maximum_leaves <= 2 || treelet_size <= 2
	{
		return .Invalid_Argument;
	}
	refinement_capacity := max(treelet_size, min(maximum_leaves, int(math.ceil(math.sqrt(f32(maximum_leaves)) * 4))));
	binned_inner_slot_count := worker_count + 1;
	workspace^ = {
		worker_count=worker_count, maximum_leaves=maximum_leaves, treelet_size=treelet_size,
		refinement_capacity=refinement_capacity, binned_inner_slot_count=binned_inner_slot_count, pool=pool,
	};
	status: util.Memory_Status;
	workspace.refit_nodes, status = util.buffer_pool_take_at_least(pool, i32, maximum_leaves);
	if status != .Ok
	{
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.refinement_candidates, status = util.buffer_pool_take_at_least(pool, i32, maximum_leaves);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.refinement_targets, status = util.buffer_pool_take_at_least(pool, i32, maximum_leaves);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.static_refinement_targets, status = util.buffer_pool_take_at_least(pool, i32, maximum_leaves);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.task_records, status = util.buffer_pool_take_at_least(pool, Tree_Maintenance_Task, maximum_leaves);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.static_task_records, status = util.buffer_pool_take_at_least(pool, Tree_Maintenance_Task, maximum_leaves);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.binned_centroid_bounds, status = util.buffer_pool_take_at_least(
		pool, util.Bounding_Box, worker_count * binned_inner_slot_count,
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.binned_bins, status = util.buffer_pool_take_at_least(
		pool, Tree_Bin, worker_count * binned_inner_slot_count * TREE_BIN_MAX_COUNT,
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.binned_bin_indices, status = util.buffer_pool_take_at_least(
		pool, u8, worker_count * refinement_capacity,
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.references, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Build_Reference,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.scratch, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Build_Reference,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.refinement_subtrees, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Node_Child,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.refinement_subtree_scratch, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Node_Child,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.temporary_nodes, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Node,
		worker_count * (refinement_capacity - 1)
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.node_indices, status = util.buffer_pool_take_at_least(
		pool,
		i32,
		worker_count * (refinement_capacity - 1)
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.root_stack_entries, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Refinement_Root_Stack_Entry,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.heap_entries, status = util.buffer_pool_take_at_least(
		pool,
		Tree_Refinement_Heap_Entry,
		worker_count * refinement_capacity
	);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.cache_nodes, status = util.buffer_pool_take_at_least(pool, Tree_Node, max(maximum_leaves - 1, 1));
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.cache_metanodes, status = util.buffer_pool_take_at_least(pool, Tree_Metanode, max(maximum_leaves - 1, 1));
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.statuses, status = util.buffer_pool_take_at_least(pool, Physics_Status, worker_count);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.worker_hits, status = util.buffer_pool_take_at_least(pool, i32, worker_count);
	if status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return physics_memory_status(status);
	}
	workspace.task_procedures = {
		tree_update2_entry_task,
		tree_refinement_task,
		tree_binned_node_task,
		tree_binned_centroid_task,
		tree_binned_bin_task,
		tree_binned_partition_task,
		tree_reify_task,
		tree_cache_copy_task,
		tree_task_request_stop,
	};
	procedure_buffer := util.Buffer(util.Task_Procedure){
		memory=&workspace.task_procedures[0],
		length=i32(TREE_TASK_PROCEDURE_COUNT),
		id=-1,
	};
	task_status := util.task_stack_initialize(
		&workspace.task_stack, worker_count, procedure_buffer, pool,
	);
	if task_status != .Ok
	{
		tree_refit_refine_workspace_return_buffers(workspace, pool);
		workspace^ = {};
		return .Capacity_Missing;
	}
	workspace.state = .Ready;
	return .Ok;
}
tree_refit_refine_workspace_dispose :: proc (workspace: ^Tree_Refit_Refine_Workspace) -> Physics_Status
{
	if workspace == nil || workspace.state != .Ready || workspace.pool == nil
	{
		return .Disposed;
	}
	pool := workspace.pool;
	tree_refit_refine_workspace_return_buffers(workspace, pool);
	workspace^ = {state=.Disposed};
	return .Ok;
}
tree_collect_refit_wavefront :: proc "contextless" (
	tree: ^Tree, node_index, leaf_threshold: int, output: util.Buffer(i32), count: ^int,
) -> Physics_Status
{
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index < 0
		{
			continue;
		}
		if child.leaf_count <= i32(leaf_threshold)
		{
			if count^ >= int(output.length)
			{
				return .Capacity_Missing;
			}
			output.memory[count^] = child.index;
			count^ += 1;
		}
		else
		{
			status := tree_collect_refit_wavefront(tree, int(child.index), leaf_threshold, output, count);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
tree_collect_refinement_candidates :: proc "contextless" (
	tree: ^Tree, node_index, treelet_size: int, output: util.Buffer(i32), count: ^int,
) -> Physics_Status
{
	node := &tree.nodes.memory[node_index];
	for child_index in 0 ..< 2
	{
		child := tree_child(node, child_index);
		if child.index < 0 || child.leaf_count <= 2
		{
			continue;
		}
		if child.leaf_count <= i32(treelet_size)
		{
			if count^ >= int(output.length)
			{
				return .Capacity_Missing;
			}
			output.memory[count^] = child.index;
			count^ += 1;
		}
		else
		{
			status := tree_collect_refinement_candidates(tree, int(child.index), treelet_size, output, count);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
Tree_Refit_Refine_Context :: struct
{
	tree:         ^Tree,
	workspace:    ^Tree_Refit_Refine_Workspace,
	task_count:   int,
	worker_count: int,
	next_task:    i32,
}
tree_execute_maintenance_task :: proc "contextless" (ctx: ^Tree_Refit_Refine_Context, worker_index, task_index: int)
{
	workspace := ctx.workspace;
	if workspace.statuses.memory[worker_index] != .Ok
	{
		return;
	}
	task := workspace.task_records.memory[task_index];
	if task.kind == .Refit
	{
		_, _ = tree_refit_subtree(ctx.tree, int(task.node_index));
	}
	else
	{
		context = runtime.default_context();
		treelet_size := workspace.treelet_size;
		worker_stride := workspace.refinement_capacity;
		node_stride := worker_stride - 1;
		node_capacity := treelet_size - 1;
		references := util.Buffer(Tree_Build_Reference){
			memory=&workspace.references.memory[worker_index * worker_stride], length=i32(treelet_size), id=-1,
		};
		scratch := util.Buffer(Tree_Build_Reference){
			memory=&workspace.scratch.memory[worker_index * worker_stride], length=i32(treelet_size), id=-1,
		};
		temporary_nodes := util.Buffer(Tree_Node){
			memory=&workspace.temporary_nodes.memory[worker_index * node_stride], length=i32(node_capacity), id=-1,
		};
		node_indices := util.Buffer(i32){
			memory=&workspace.node_indices.memory[worker_index * node_stride], length=i32(node_capacity), id=-1,
		};
		workspace.statuses.memory[worker_index] = tree_binned_refine_treelet(
			ctx.tree, int(task.node_index), references, scratch, temporary_nodes, node_indices,
		);
	}
	workspace.worker_hits.memory[worker_index] += 1;
}
tree_refit_refine_worker :: proc "contextless" (worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	ctx := (^Tree_Refit_Refine_Context)(dispatcher.unmanaged_context);
	if worker_index < ctx.task_count
	{
		tree_execute_maintenance_task(ctx, worker_index, worker_index);
	}
	for
	{
		task_index := int(sync.atomic_add_explicit(&ctx.next_task, i32(1), .Seq_Cst));
		if task_index >= ctx.task_count
		{
			break;
		}
		tree_execute_maintenance_task(ctx, worker_index, task_index);
	}
}
tree_dispatch_maintenance_tasks :: proc "contextless" (
	tree: ^Tree, workspace: ^Tree_Refit_Refine_Workspace,
	dispatcher: ^util.Thread_Dispatcher_Boundary, task_count: int,
) -> Physics_Status
{
	if task_count <= 0
	{
		return .Ok;
	}
	worker_count := min(dispatcher.worker_count, task_count);
	ctx := Tree_Refit_Refine_Context{
		tree=tree,
		workspace=workspace,
		task_count=task_count,
		worker_count=worker_count,
		next_task=i32(worker_count),
	};
	dispatch_status := dispatcher.dispatch(dispatcher, tree_refit_refine_worker, worker_count, &ctx);
	if dispatch_status != .Ok
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< worker_count
	{
		if workspace.statuses.memory[worker_index] != .Ok
		{
			return workspace.statuses.memory[worker_index];
		}
	}
	return .Ok;
}
tree_refit_refine_single :: proc "contextless" (
	tree: ^Tree, frame_index: u32, workspace: ^Tree_Refit_Refine_Workspace,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || workspace == nil || workspace.state != .Ready ||
		tree.leaf_count > workspace.maximum_leaves
	{
		return .Invalid_Argument;
	}
	workspace.last_refinement_offset = -1;
	workspace.last_refinement_target_count = 0;
	status := tree_refit(tree);
	if status != .Ok || tree.leaf_count <= 2
	{
		tree.refinement_frame = frame_index + 1;
		return status;
	}
	candidate_count := 0;
	if tree.leaf_count <= workspace.treelet_size
	{
		workspace.refinement_candidates.memory[0] = 0;
		candidate_count = 1;
	}
	else
	{
		status = tree_collect_refinement_candidates(
			tree, 0, workspace.treelet_size, workspace.refinement_candidates, &candidate_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if candidate_count > 0
	{
		offset := int((u64(frame_index) * 236887691 + 104395303) % u64(candidate_count));
		workspace.last_refinement_offset = i32(offset);
		workspace.last_refinement_target_count = 1;
		treelet_size := workspace.treelet_size;
		node_capacity := treelet_size - 1;
		references := util.Buffer(Tree_Build_Reference){
			memory=workspace.references.memory, length=i32(treelet_size), id=-1,
		};
		scratch := util.Buffer(Tree_Build_Reference){
			memory=workspace.scratch.memory, length=i32(treelet_size), id=-1,
		};
		temporary_nodes := util.Buffer(Tree_Node){
			memory=workspace.temporary_nodes.memory, length=i32(node_capacity), id=-1,
		};
		node_indices := util.Buffer(i32){
			memory=workspace.node_indices.memory, length=i32(node_capacity), id=-1,
		};
		context = runtime.default_context();
		status = tree_binned_refine_treelet(
			tree, int(workspace.refinement_candidates.memory[offset]),
			references, scratch, temporary_nodes, node_indices,
		);
	}
	if status == .Ok
	{
		tree.refinement_frame = frame_index + 1;
	}
	return status;
}
tree_refit_refine_parallel :: proc "contextless" (
	tree: ^Tree, frame_index: u32, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Refit_Refine_Workspace,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || dispatcher == nil || dispatcher.dispatch == nil ||
		workspace == nil || workspace.state != .Ready || tree.leaf_count > workspace.maximum_leaves ||
		dispatcher.worker_count <= 0 || dispatcher.worker_count > workspace.worker_count
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< workspace.worker_count
	{
		workspace.statuses.memory[worker_index] = .Ok;
		workspace.worker_hits.memory[worker_index] = 0;
	}
	workspace.last_refinement_offset = -1;
	workspace.last_refinement_target_count = 0;
	if tree.leaf_count <= 2
	{
		tree.refinement_frame = frame_index + 1;
		return .Ok;
	}
	refit_count := 0;
	leaf_threshold := max(2, (tree.leaf_count + dispatcher.worker_count * 2 - 1) / (dispatcher.worker_count * 2));
	status := tree_collect_refit_wavefront(tree, 0, leaf_threshold, workspace.refit_nodes, &refit_count);
	if status != .Ok
	{
		return status;
	}
	if refit_count == 0
	{
		workspace.refit_nodes.memory[0] = 0;
		refit_count = 1;
	}
	for index in 0 ..< refit_count
	{
		workspace.task_records.memory[index] = {node_index=workspace.refit_nodes.memory[index], kind=.Refit};
	}
	status = tree_dispatch_maintenance_tasks(tree, workspace, dispatcher, refit_count);
	if status != .Ok
	{
		return status;
	}
	for index in 0 ..< refit_count
	{
		tree_refit_for_node_bounds_change(tree, int(workspace.refit_nodes.memory[index]));
	}

	candidate_count := 0;
	if tree.leaf_count <= workspace.treelet_size
	{
		workspace.refinement_candidates.memory[0] = 0;
		candidate_count = 1;
	}
	else
	{
		status = tree_collect_refinement_candidates(
			tree, 0, workspace.treelet_size, workspace.refinement_candidates, &candidate_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if candidate_count > 0
	{
		target_count := min(candidate_count, dispatcher.worker_count);
		offset := int((u64(frame_index) * 236887691 + 104395303) % u64(candidate_count));
		period := max(1, candidate_count / target_count);
		workspace.last_refinement_offset = i32(offset);
		workspace.last_refinement_target_count = i32(target_count);
		for index in 0 ..< target_count
		{
			candidate_index := (offset + index * period) % candidate_count;
			target := workspace.refinement_candidates.memory[candidate_index];
			workspace.refinement_targets.memory[index] = target;
			workspace.task_records.memory[index] = {node_index=target, kind=.Refine};
		}
		status = tree_dispatch_maintenance_tasks(tree, workspace, dispatcher, target_count);
		if status != .Ok
		{
			return status;
		}
	}
	tree.refinement_frame = frame_index + 1;
	return .Ok;
}
tree_find_subtree_refinement_targets_node :: proc "contextless" (
	tree: ^Tree, node_index, left_leaf_count, subtree_refinement_size, target_count: int,
	start_index: ^int, end_index: int, output: util.Buffer(i32), output_count: ^int,
) -> Physics_Status
{
	if start_index^ >= end_index || output_count^ == target_count
	{
		return .Ok;
	}
	node := &tree.nodes.memory[node_index];
	midpoint := left_leaf_count + int(node.a.leaf_count);
	if start_index^ < midpoint
	{
		if node.a.leaf_count <= i32(subtree_refinement_size)
		{
			if node.a.leaf_count > 2
			{
				if output_count^ >= int(output.length)
				{
					return .Capacity_Missing;
				}
				output.memory[output_count^] = node.a.index;
				output_count^ += 1;
			}
			start_index^ += int(node.a.leaf_count);
		}
		else
		{
			status := tree_find_subtree_refinement_targets_node(
				tree, int(node.a.index), left_leaf_count, subtree_refinement_size, target_count,
				start_index, end_index, output, output_count,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	if start_index^ >= end_index || output_count^ == target_count
	{
		return .Ok;
	}
	if start_index^ >= midpoint
	{
		if node.b.leaf_count <= i32(subtree_refinement_size)
		{
			if node.b.leaf_count > 2
			{
				if output_count^ >= int(output.length)
				{
					return .Capacity_Missing;
				}
				output.memory[output_count^] = node.b.index;
				output_count^ += 1;
			}
			start_index^ += int(node.b.leaf_count);
		}
		else
		{
			status := tree_find_subtree_refinement_targets_node(
				tree, int(node.b.index), midpoint, subtree_refinement_size, target_count,
				start_index, end_index, output, output_count,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
tree_find_subtree_refinement_targets :: proc "contextless" (
	tree: ^Tree, subtree_refinement_size, target_count: int, start_index: ^int,
	output: util.Buffer(i32), output_count: ^int,
) -> Physics_Status
{
	output_count^ = 0;
	if target_count <= 0 || subtree_refinement_size <= 0 || tree.leaf_count <= 2
	{
		return .Ok;
	}
	if start_index^ >= tree.leaf_count || start_index^ < 0
	{
		start_index^ = 0;
	}
	initial_start := start_index^;
	status := tree_find_subtree_refinement_targets_node(
		tree, 0, 0, subtree_refinement_size, target_count, start_index, tree.leaf_count, output, output_count,
	);
	if status != .Ok
	{
		return status;
	}
	if start_index^ >= tree.leaf_count && output_count^ < target_count
	{
		start_index^ = 0;
		return tree_find_subtree_refinement_targets_node(
			tree, 0, 0, subtree_refinement_size, target_count, start_index, initial_start, output, output_count,
		);
	}
	return .Ok;
}
tree_refinement_target_state :: proc "contextless" (
	child: Tree_Node_Child, parent_leaf_count, subtree_refinement_size: int,
	targets: util.Buffer(i32), target_count: int,
) -> Reference_State
{
	if child.leaf_count > i32(subtree_refinement_size) || parent_leaf_count <= subtree_refinement_size
	{
		return .Missing;
	}
	for index in 0 ..< target_count
	{
		if targets.memory[index] == child.index
		{
			return .Present;
		}
	}
	return .Missing;
}
tree_append_root_refinement_subtree :: proc "contextless" (
	child: Tree_Node_Child, output: util.Buffer(Tree_Node_Child), count: ^int,
) -> Physics_Status
{
	if count^ >= int(output.length)
	{
		return .Capacity_Missing;
	}
	output.memory[count^] = child;
	output.memory[count^].index |= TREE_REFINEMENT_SUBTREE_FLAG;
	count^ += 1;
	return .Ok;
}
tree_collect_root_balanced_child :: proc "contextless" (
	child: Tree_Node_Child, subtree_budget, parent_leaf_count, subtree_refinement_size: int,
	targets: util.Buffer(i32), target_count: int,
	stack: util.Buffer(Tree_Refinement_Root_Stack_Entry), stack_count: ^int,
	subtrees: util.Buffer(Tree_Node_Child), subtree_count: ^int,
) -> Physics_Status
{
	if subtree_budget == 1 || tree_refinement_target_state(
		child, parent_leaf_count, subtree_refinement_size, targets, target_count,
	) == .Present
	{
		return tree_append_root_refinement_subtree(child, subtrees, subtree_count);
	}
	if child.index < 0
	{
		return .Invalid_Description;
	}
	if stack_count^ >= int(stack.length)
	{
		return .Capacity_Missing;
	}
	stack.memory[stack_count^] = {node_index=child.index, subtree_budget=i32(subtree_budget)};
	stack_count^ += 1;
	return .Ok;
}
tree_collect_root_refinement_balanced :: proc "contextless" (
	tree: ^Tree, root_refinement_size, subtree_refinement_size: int,
	targets: util.Buffer(i32), target_count: int,
	stack: util.Buffer(Tree_Refinement_Root_Stack_Entry),
	node_indices: util.Buffer(i32), node_count: ^int,
	subtrees: util.Buffer(Tree_Node_Child), subtree_count: ^int,
) -> Physics_Status
{
	node_count^ = 0;
	subtree_count^ = 0;
	stack_count := 1;
	stack.memory[0] = {node_index=0, subtree_budget=i32(root_refinement_size)};
	for stack_count > 0
	{
		stack_count -= 1;
		entry := stack.memory[stack_count];
		if node_count^ >= int(node_indices.length)
		{
			return .Capacity_Missing;
		}
		node_indices.memory[node_count^] = entry.node_index;
		node_count^ += 1;
		node := &tree.nodes.memory[entry.node_index];
		total_leaf_count := int(node.a.leaf_count + node.b.leaf_count);
		lower_budget := min((int(entry.subtree_budget) + 1) / 2, min(int(node.a.leaf_count), int(node.b.leaf_count)));
		higher_budget := int(entry.subtree_budget) - lower_budget;
		a_budget, b_budget := higher_budget, lower_budget;
		if lower_budget == int(node.a.leaf_count)
		{
			a_budget, b_budget = lower_budget, higher_budget;
		}
		status := tree_collect_root_balanced_child(
			node.b, b_budget, total_leaf_count, subtree_refinement_size, targets, target_count,
			stack, &stack_count, subtrees, subtree_count,
		);
		if status != .Ok
		{
			return status;
		}
		status = tree_collect_root_balanced_child(
			node.a, a_budget, total_leaf_count, subtree_refinement_size, targets, target_count,
			stack, &stack_count, subtrees, subtree_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
tree_refinement_heap_insert :: proc "contextless" (
	heap: util.Buffer(Tree_Refinement_Heap_Entry), count: ^int, entry: Tree_Refinement_Heap_Entry,
) -> Physics_Status
{
	if count^ >= int(heap.length)
	{
		return .Capacity_Missing;
	}
	index := count^;
	count^ += 1;
	for index > 0
	{
		parent_index := (index - 1) >> 1;
		parent := heap.memory[parent_index];
		if parent.cost >= entry.cost
		{
			break;
		}
		heap.memory[index] = parent;
		index = parent_index;
	}
	heap.memory[index] = entry;
	return .Ok;
}
tree_refinement_heap_pop :: proc "contextless" (
	heap: util.Buffer(Tree_Refinement_Heap_Entry), count: ^int,
) -> Tree_Refinement_Heap_Entry
{
	result := heap.memory[0];
	count^ -= 1;
	last := heap.memory[count^];
	index := 0;
	for
	{
		child_a := index * 2 + 1;
		if child_a >= count^
		{
			break;
		}
		child_b := child_a + 1;
		larger := child_a;
		if child_b < count^ && heap.memory[child_b].cost >= heap.memory[child_a].cost
		{
			larger = child_b;
		}
		if last.cost > heap.memory[larger].cost
		{
			break;
		}
		heap.memory[index] = heap.memory[larger];
		index = larger;
	}
	heap.memory[index] = last;
	return result;
}
tree_collect_root_priority_child :: proc "contextless" (
	child: Tree_Node_Child, parent_leaf_count, subtree_refinement_size: int,
	targets: util.Buffer(i32), target_count: int,
	heap: util.Buffer(Tree_Refinement_Heap_Entry), heap_count: ^int,
	subtrees: util.Buffer(Tree_Node_Child), subtree_count: ^int,
) -> Physics_Status
{
	if child.index < 0
	{
		if subtree_count^ >= int(subtrees.length)
		{
			return .Capacity_Missing;
		}
		subtrees.memory[subtree_count^] = child;
		subtree_count^ += 1;
		return .Ok;
	}
	if tree_refinement_target_state(
		child,
		parent_leaf_count,
		subtree_refinement_size,
		targets,
		target_count
	) == .Present
	{
		return tree_append_root_refinement_subtree(child, subtrees, subtree_count);
	}
	return tree_refinement_heap_insert(
		heap, heap_count, {
			node_index=child.index,
			cost=tree_bounds_metric(tree_child_bounds(child)) * f32(child.leaf_count)
		},
	);
}
tree_collect_root_refinement_priority :: proc "contextless" (
	tree: ^Tree, root_refinement_size, subtree_refinement_size: int,
	targets: util.Buffer(i32), target_count: int,
	heap: util.Buffer(Tree_Refinement_Heap_Entry),
	node_indices: util.Buffer(i32), node_count: ^int,
	subtrees: util.Buffer(Tree_Node_Child), subtree_count: ^int,
) -> Physics_Status
{
	node_count^ = 0;
	subtree_count^ = 0;
	heap_count := 0;
	status := tree_refinement_heap_insert(heap, &heap_count, {node_index=0});
	if status != .Ok
	{
		return status;
	}
	for heap_count > 0 && heap_count + subtree_count^ < root_refinement_size
	{
		entry := tree_refinement_heap_pop(heap, &heap_count);
		if node_count^ >= int(node_indices.length)
		{
			return .Capacity_Missing;
		}
		node_indices.memory[node_count^] = entry.node_index;
		node_count^ += 1;
		node := &tree.nodes.memory[entry.node_index];
		total_leaf_count := int(node.a.leaf_count + node.b.leaf_count);
		status = tree_collect_root_priority_child(
			node.b, total_leaf_count, subtree_refinement_size, targets, target_count,
			heap, &heap_count, subtrees, subtree_count,
		);
		if status != .Ok
		{
			return status;
		}
		status = tree_collect_root_priority_child(
			node.a, total_leaf_count, subtree_refinement_size, targets, target_count,
			heap, &heap_count, subtrees, subtree_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	for index in 0 ..< heap_count
	{
		node_index := int(heap.memory[index].node_index);
		meta := tree.metanodes.memory[node_index];
		child := tree_child(&tree.nodes.memory[meta.parent], int(meta.index_in_parent))^;
		status = tree_append_root_refinement_subtree(child, subtrees, subtree_count);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
tree_collect_subtree_refinement :: proc "contextless" (
	tree: ^Tree, root_node_index: int, stack: util.Buffer(Tree_Refinement_Root_Stack_Entry),
	node_indices: util.Buffer(i32), node_count: ^int,
	subtrees: util.Buffer(Tree_Node_Child), subtree_count: ^int,
) -> Physics_Status
{
	node_count^ = 0;
	subtree_count^ = 0;
	stack_count := 1;
	stack.memory[0] = {node_index=i32(root_node_index)};
	for stack_count > 0
	{
		stack_count -= 1;
		node_index := int(stack.memory[stack_count].node_index);
		if node_count^ >= int(node_indices.length)
		{
			return .Capacity_Missing;
		}
		node_indices.memory[node_count^] = i32(node_index);
		node_count^ += 1;
		node := &tree.nodes.memory[node_index];
		if node.b.index >= 0
		{
			if stack_count >= int(stack.length)
			{
				return .Capacity_Missing;
			}
			stack.memory[stack_count] = {node_index=node.b.index};
			stack_count += 1;
		}
		else
		{
			if subtree_count^ >= int(subtrees.length)
			{
				return .Capacity_Missing;
			}
			subtrees.memory[subtree_count^] = node.b;
			subtree_count^ += 1;
		}
		if node.a.index >= 0
		{
			if stack_count >= int(stack.length)
			{
				return .Capacity_Missing;
			}
			stack.memory[stack_count] = {node_index=node.a.index};
			stack_count += 1;
		}
		else
		{
			if subtree_count^ >= int(subtrees.length)
			{
				return .Capacity_Missing;
			}
			subtrees.memory[subtree_count^] = node.a;
			subtree_count^ += 1;
		}
	}
	return .Ok;
}
