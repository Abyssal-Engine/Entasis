package entasis_physics

import util "entasis:entasis_utilities"
import "core:sync"

Tree_Parallel_Query_Mode :: enum u8
{
	Self,
	Intertree,
}
Tree_Node_Pair_Job_Kind :: enum u8
{
	Self_Node,
	Child_Pair,
}
Tree_Node_Pair_Job :: struct
{
	a:          Tree_Node_Child,
	b:          Tree_Node_Child,
	node_index: i32,
	kind:       Tree_Node_Pair_Job_Kind,
	mode:       Tree_Parallel_Query_Mode,
}
Tree_Parallel_Query_Workspace :: struct
{
	jobs:                     util.Buffer(Tree_Node_Pair_Job),
	statuses:                 util.Buffer(Physics_Status),
	worker_count:             int,
	maximum_jobs:             int,
	pool:                     ^util.Buffer_Pool,
	state:                    Tree_State,
	claim_cursor:             util.Buffer(i32),
}
Tree_Parallel_Worker_Finalize_Proc :: #type proc "contextless" (
	user_context: rawptr, worker_index: int,
) -> Physics_Status;
Tree_Parallel_Query_Context :: struct
{
	tree_a:       ^Tree,
	tree_b:       ^Tree,
	workspace:    ^Tree_Parallel_Query_Workspace,
	job_count:              int,
	next_job:               ^i32,
	visitor:                Tree_Pair_Visitor_Proc,
	self_user_context:      rawptr,
	intertree_user_context: rawptr,
	worker_finalize:        Tree_Parallel_Worker_Finalize_Proc,
	worker_finalize_context: rawptr,
}
tree_parallel_query_workspace_initialize :: proc (
	workspace: ^Tree_Parallel_Query_Workspace, worker_count, maximum_leaves, pair_capacity_per_worker: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if workspace == nil || pool == nil || worker_count <= 0 || maximum_leaves <= 0 || pair_capacity_per_worker <= 0
	{
		return .Invalid_Argument;
	}
	_ = pair_capacity_per_worker;
	if maximum_leaves > max(int) / 2 || worker_count > max(int) / 16
	{
		return .Capacity_Missing;
	}
	maximum_jobs := max(maximum_leaves * 2, worker_count * 16);
	jobs, job_status := util.buffer_pool_take_at_least(pool, Tree_Node_Pair_Job, maximum_jobs);
	if job_status != .Ok
	{
		return physics_memory_status(job_status);
	}
	statuses, status_status := util.buffer_pool_take_at_least(pool, Physics_Status, worker_count);
	if status_status != .Ok
	{
		physics_return_buffer(pool, &jobs);
		return physics_memory_status(status_status);
	}
	claim_cursor: util.Buffer(i32);
	cursor_status: util.Memory_Status;
	claim_cursor, cursor_status = util.buffer_pool_take_at_least(pool, i32, 16);
	if cursor_status != .Ok
	{
		physics_return_buffer(pool, &statuses);
		physics_return_buffer(pool, &jobs);
		return physics_memory_status(cursor_status);
	}
	workspace^ = {
		jobs=jobs,
		statuses=statuses,
		worker_count=worker_count,
		maximum_jobs=maximum_jobs,
		pool=pool,
		state=.Ready,
		claim_cursor=claim_cursor,
	};
	return .Ok;
}
tree_parallel_query_workspace_dispose :: proc (workspace: ^Tree_Parallel_Query_Workspace) -> Physics_Status
{
	if workspace == nil || workspace.state != .Ready || workspace.pool == nil
	{
		return .Disposed;
	}
	pool := workspace.pool;
	physics_return_buffer(pool, &workspace.claim_cursor);
	physics_return_buffer(pool, &workspace.statuses);
	physics_return_buffer(pool, &workspace.jobs);
	workspace^ = {state=.Disposed};
	return .Ok;
}
tree_parallel_add_query_job :: proc "contextless" (
	workspace: ^Tree_Parallel_Query_Workspace, job_count: ^int, job: Tree_Node_Pair_Job,
) -> Physics_Status
{
	if job_count^ >= workspace.maximum_jobs
	{
		return .Capacity_Missing;
	}
	workspace.jobs.memory[job_count^] = job;
	job_count^ += 1;
	return .Ok;
}
tree_parallel_collect_pair_jobs :: proc "contextless" (
	tree_a: ^Tree, child_a: Tree_Node_Child, tree_b: ^Tree, child_b: Tree_Node_Child,
	mode: Tree_Parallel_Query_Mode, leaf_threshold: int,
	workspace: ^Tree_Parallel_Query_Workspace, job_count: ^int,
) -> Physics_Status
{
	if tree_bounds_intersect(tree_child_bounds(child_a), tree_child_bounds(child_b)) != .Present
	{
		return .Ok;
	}
	if child_a.index < 0 && child_b.index < 0 || int(child_a.leaf_count + child_b.leaf_count) <= leaf_threshold
	{
		return tree_parallel_add_query_job(
			workspace, job_count, {a=child_a, b=child_b, kind=.Child_Pair, mode=mode},
		);
	}
	if child_b.index < 0 || child_a.index >= 0 && child_a.leaf_count >= child_b.leaf_count
	{
		node := &tree_a.nodes.memory[child_a.index];
		status := tree_parallel_collect_pair_jobs(
			tree_a, node.a, tree_b, child_b, mode, leaf_threshold, workspace, job_count,
		);
		if status != .Ok
		{
			return status;
		}
		return tree_parallel_collect_pair_jobs(
			tree_a, node.b, tree_b, child_b, mode, leaf_threshold, workspace, job_count,
		);
	}
	node := &tree_b.nodes.memory[child_b.index];
	status := tree_parallel_collect_pair_jobs(
		tree_a, child_a, tree_b, node.a, mode, leaf_threshold, workspace, job_count,
	);
	if status != .Ok
	{
		return status;
	}
	return tree_parallel_collect_pair_jobs(
		tree_a, child_a, tree_b, node.b, mode, leaf_threshold, workspace, job_count,
	);
}
tree_parallel_collect_self_jobs :: proc "contextless" (
	tree: ^Tree, node_index, leaf_threshold: int,
	workspace: ^Tree_Parallel_Query_Workspace, job_count: ^int,
) -> Physics_Status
{
	node := &tree.nodes.memory[node_index];
	leaf_count := int(node.a.leaf_count + node.b.leaf_count);
	if leaf_count <= leaf_threshold
	{
		return tree_parallel_add_query_job(
			workspace, job_count, {node_index=i32(node_index), kind=.Self_Node, mode=.Self},
		);
	}
	if node.a.index >= 0
	{
		status := tree_parallel_collect_self_jobs(tree, int(node.a.index), leaf_threshold, workspace, job_count);
		if status != .Ok
		{
			return status;
		}
	}
	if node.b.index >= 0
	{
		status := tree_parallel_collect_self_jobs(tree, int(node.b.index), leaf_threshold, workspace, job_count);
		if status != .Ok
		{
			return status;
		}
	}
	return tree_parallel_collect_pair_jobs(
		tree, node.a, tree, node.b, .Self, leaf_threshold, workspace, job_count,
	);
}
tree_parallel_execute_query_job :: proc "contextless" (
	ctx: ^Tree_Parallel_Query_Context, worker_index, job_index: int,
)
{
	workspace := ctx.workspace;
	if workspace.statuses.memory[worker_index] != .Ok
	{
		return;
	}
	job := workspace.jobs.memory[job_index];
	status: Physics_Status;
	if job.kind == .Self_Node
	{
		status = tree_self_visit_node(
			ctx.tree_a, int(job.node_index), ctx.visitor,
			ctx.self_user_context, worker_index,
		);
	}
	else if job.mode == .Self
	{
		status = tree_visit_child_pair(
			ctx.tree_a, &job.a, ctx.tree_a, &job.b,
			ctx.visitor, ctx.self_user_context, worker_index, .Canonical,
		);
	}
	else
	{
		status = tree_visit_child_pair(
			ctx.tree_a, &job.a, ctx.tree_b, &job.b,
			ctx.visitor, ctx.intertree_user_context, worker_index, .Preserve_Tree_Order,
		);
	}
	if status != .Ok
	{
		workspace.statuses.memory[worker_index] = status;
	}
}
tree_parallel_query_worker :: proc "contextless" (worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	ctx := (^Tree_Parallel_Query_Context)(dispatcher.unmanaged_context);
	next_job: ^i32 = ctx.next_job;
	if worker_index < ctx.job_count
	{
		tree_parallel_execute_query_job(ctx, worker_index, worker_index);
	}
	if ctx.workspace.statuses.memory[worker_index] == .Ok
	{
		for
		{
			job_index := int(sync.atomic_add_explicit(next_job, i32(1), .Relaxed));
			if job_index >= ctx.job_count
			{
				break;
			}
			tree_parallel_execute_query_job(ctx, worker_index, job_index);
			if ctx.workspace.statuses.memory[worker_index] != .Ok
			{
				break;
			}
		}
	}
	if ctx.workspace.statuses.memory[worker_index] == .Ok && ctx.worker_finalize != nil
	{
		ctx.workspace.statuses.memory[worker_index] =
			ctx.worker_finalize(ctx.worker_finalize_context, worker_index);
	}
}
tree_leaf_pair_compare :: proc "contextless" (a, b, comparer_context: rawptr) -> int
{
	_ = comparer_context;
	left := (^Tree_Leaf_Pair)(a);
	right := (^Tree_Leaf_Pair)(b);
	if left.a < right.a
	{
		return -1;
	}
	if left.a > right.a
	{
		return 1;
	}
	if left.b < right.b
	{
		return -1;
	}
	if left.b > right.b
	{
		return 1;
	}
	return 0;
}
tree_parallel_visit :: proc "contextless" (
	tree_a, tree_b: ^Tree, mode: Tree_Parallel_Query_Mode,
	dispatcher: ^util.Thread_Dispatcher_Boundary, workspace: ^Tree_Parallel_Query_Workspace,
	visitor: Tree_Pair_Visitor_Proc, user_context: rawptr,
) -> Physics_Status
{
	if tree_a == nil || tree_b == nil || tree_a.state != .Ready || tree_b.state != .Ready ||
		dispatcher == nil || dispatcher.dispatch == nil || workspace == nil || workspace.state != .Ready ||
		visitor == nil || dispatcher.worker_count <= 0 || dispatcher.worker_count > workspace.worker_count ||
		mode == .Self && tree_a != tree_b
	{
		return .Invalid_Argument;
	}
	if mode == .Self &&
		tree_a.leaf_count < 2 ||
		mode == .Intertree &&
		(tree_a.leaf_count == 0 || tree_b.leaf_count == 0)
	{
		return .Ok;
	}
	worker_count := dispatcher.worker_count;
	job_count := 0;
	leaf_threshold := max(2, max(tree_a.leaf_count, tree_b.leaf_count) / (worker_count * 8));
	status := Physics_Status.Ok;
	if mode == .Self
	{
		status = tree_parallel_collect_self_jobs(tree_a, 0, leaf_threshold, workspace, &job_count);
	}
	else
	{
		root_a := &tree_a.nodes.memory[0];
		root_b := &tree_b.nodes.memory[0];
		a_count, b_count := 1, 1;
		if tree_a.leaf_count > 1
		{
			a_count = 2;
		}
		if tree_b.leaf_count > 1
		{
			b_count = 2;
		}
		for a_index in 0 ..< a_count
		{
			for b_index in 0 ..< b_count
			{
				status = tree_parallel_collect_pair_jobs(
					tree_a, tree_child(root_a, a_index)^, tree_b, tree_child(root_b, b_index)^,
					.Intertree, leaf_threshold, workspace, &job_count,
				);
				if status != .Ok
				{
					return status;
				}
			}
		}
	}
	if status != .Ok || job_count == 0
	{
		return status;
	}
	active_workers := min(worker_count, job_count);
	for worker_index in 0 ..< active_workers
	{
		workspace.statuses.memory[worker_index] = .Ok;
	}
	next_job: ^i32 = &workspace.claim_cursor.memory[0];
	next_job^ = i32(active_workers);
	ctx := Tree_Parallel_Query_Context{
		tree_a=tree_a,
		tree_b=tree_b,
		workspace=workspace,
		job_count=job_count,
		next_job=next_job,
		visitor=visitor,
		self_user_context=user_context,
		intertree_user_context=user_context,
	};
	dispatch_status := dispatcher.dispatch(dispatcher, tree_parallel_query_worker, active_workers, &ctx);
	if dispatch_status != .Ok
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< active_workers
	{
		if workspace.statuses.memory[worker_index] != .Ok
		{
			return workspace.statuses.memory[worker_index];
		}
	}
	return .Ok;
}
tree_self_intertree_visit_parallel :: proc "contextless" (
	tree_a, tree_b: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, visitor: Tree_Pair_Visitor_Proc,
	self_user_context, intertree_user_context: rawptr,
	worker_finalize: Tree_Parallel_Worker_Finalize_Proc = nil,
	worker_finalize_context: rawptr = nil,
) -> Physics_Status
{
	if tree_a == nil || tree_b == nil || tree_a.state != .Ready || tree_b.state != .Ready ||
		dispatcher == nil || dispatcher.dispatch == nil || workspace == nil || workspace.state != .Ready ||
		visitor == nil || dispatcher.worker_count <= 0 || dispatcher.worker_count > workspace.worker_count
	{
		return .Invalid_Argument;
	}
	worker_count := dispatcher.worker_count;
	job_count := 0;
	if tree_a.leaf_count >= 2
	{
		self_leaf_threshold := max(2, tree_a.leaf_count / (worker_count * 8));
		status := tree_parallel_collect_self_jobs(
			tree_a, 0, self_leaf_threshold, workspace, &job_count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if tree_a.leaf_count > 0 && tree_b.leaf_count > 0
	{
		intertree_leaf_threshold := max(
			2, max(tree_a.leaf_count, tree_b.leaf_count) / (worker_count * 8),
		);
		root_a := &tree_a.nodes.memory[0];
		root_b := &tree_b.nodes.memory[0];
		a_count, b_count := 1, 1;
		if tree_a.leaf_count > 1
		{
			a_count = 2;
		}
		if tree_b.leaf_count > 1
		{
			b_count = 2;
		}
		for a_index in 0 ..< a_count
		{
			for b_index in 0 ..< b_count
			{
				status := tree_parallel_collect_pair_jobs(
					tree_a, tree_child(root_a, a_index)^,
					tree_b, tree_child(root_b, b_index)^,
					.Intertree, intertree_leaf_threshold, workspace, &job_count,
				);
				if status != .Ok
				{
					return status;
				}
			}
		}
	}
	if job_count == 0
	{
		return .Ok;
	}
	active_workers := min(worker_count, job_count);
	for worker_index in 0 ..< active_workers
	{
		workspace.statuses.memory[worker_index] = .Ok;
	}
	next_job: ^i32 = &workspace.claim_cursor.memory[0];
	next_job^ = i32(active_workers);
	ctx := Tree_Parallel_Query_Context{
		tree_a=tree_a,
		tree_b=tree_b,
		workspace=workspace,
		job_count=job_count,
		next_job=next_job,
		visitor=visitor,
		self_user_context=self_user_context,
		intertree_user_context=intertree_user_context,
		worker_finalize=worker_finalize,
		worker_finalize_context=worker_finalize_context,
	};
	dispatch_status := dispatcher.dispatch(dispatcher, tree_parallel_query_worker, active_workers, &ctx);
	if dispatch_status != .Ok
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< active_workers
	{
		if workspace.statuses.memory[worker_index] != .Ok
		{
			return workspace.statuses.memory[worker_index];
		}
	}
	return .Ok;
}
tree_self_visit_parallel :: proc "contextless" (
	tree: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, visitor: Tree_Pair_Visitor_Proc,
	user_context: rawptr,
) -> Physics_Status
{
	return tree_parallel_visit(
		tree, tree, .Self, dispatcher, workspace, visitor, user_context,
	);
}
tree_intertree_visit_parallel :: proc "contextless" (
	tree_a, tree_b: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, visitor: Tree_Pair_Visitor_Proc,
	user_context: rawptr,
) -> Physics_Status
{
	return tree_parallel_visit(
		tree_a, tree_b, .Intertree, dispatcher, workspace, visitor, user_context,
	);
}
Tree_Parallel_Buffer_Visitor_Context :: struct
{
	output: util.Buffer(Tree_Leaf_Pair),
	cursor: u32,
}
tree_parallel_buffer_visitor :: proc "contextless" (
	user_context: rawptr, worker_index, leaf_a, leaf_b: int,
) -> Physics_Status
{
	_ = worker_index;
	ctx := (^Tree_Parallel_Buffer_Visitor_Context)(user_context);
	for
	{
		cursor := sync.atomic_load_explicit(&ctx.cursor, .Acquire);
		if int(cursor) >= int(ctx.output.length)
		{
			return .Capacity_Missing;
		}
		_, exchange_status := util.atomic_compare_exchange_u32_status(
			&ctx.cursor, cursor, cursor + 1,
		);
		if exchange_status != .Succeeded
		{
			continue;
		}
		ctx.output.memory[cursor] = {i32(leaf_a), i32(leaf_b)};
		return .Ok;
	}
}
tree_parallel_query_to_buffer :: proc "contextless" (
	tree_a, tree_b: ^Tree, mode: Tree_Parallel_Query_Mode,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, output: util.Buffer(Tree_Leaf_Pair),
) -> (int, Physics_Status)
{
	if output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	ctx := Tree_Parallel_Buffer_Visitor_Context{output=output};
	status := tree_parallel_visit(
		tree_a, tree_b, mode, dispatcher, workspace,
		tree_parallel_buffer_visitor, &ctx,
	);
	output_count := int(sync.atomic_load_explicit(&ctx.cursor, .Acquire));
	if status != .Ok
	{
		return output_count, status;
	}
	if output_count > 1
	{
		sort_status := util.quick_sort_keys(
			output.memory, 0, output_count - 1, {compare=tree_leaf_pair_compare},
		);
		if sort_status != .Ok
		{
			return output_count, .Invalid_Argument;
		}
	}
	return output_count, .Ok;
}
tree_self_query_parallel :: proc "contextless" (
	tree: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, output: util.Buffer(Tree_Leaf_Pair),
) -> (int, Physics_Status)
{
	return tree_parallel_query_to_buffer(
		tree, tree, .Self, dispatcher, workspace, output,
	);
}
tree_intertree_query_parallel :: proc "contextless" (
	tree_a, tree_b: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary,
	workspace: ^Tree_Parallel_Query_Workspace, output: util.Buffer(Tree_Leaf_Pair),
) -> (int, Physics_Status)
{
	return tree_parallel_query_to_buffer(
		tree_a, tree_b, .Intertree, dispatcher, workspace, output,
	);
}
