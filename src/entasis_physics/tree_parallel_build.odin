package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"
import "core:sync"

Tree_Update2_Context :: struct
{
	active_tree:         ^Tree,
	static_tree:         ^Tree,
	workspace:           ^Tree_Refit_Refine_Workspace,
	dispatcher:          ^util.Thread_Dispatcher_Boundary,
	task_stack:          ^util.Task_Stack,
	active_schedule:     Tree_Refinement_Schedule,
	static_schedule:     Tree_Refinement_Schedule,
	active_target_count: int,
	static_target_count: int,
	active_task_count:   int,
	static_task_count:   int,
	active_total_subtree_leaf_count: int,
	static_total_subtree_leaf_count: int,
	target_active_task_count: int,
	target_static_task_count: int,
	target_total_task_count: int,
	cache_active_tree:   Reference_State,
	status:              u32,
}
Tree_Binned_Build_Context :: struct
{
	references:             util.Buffer(Tree_Node_Child),
	scratch:                util.Buffer(Tree_Node_Child),
	temporary_nodes:        util.Buffer(Tree_Node),
	workspace:              ^Tree_Refit_Refine_Workspace,
	task_stack:             ^util.Task_Stack,
	original_subtree_count: int,
	target_task_count:      int,
	status:                 u32,
}
Tree_Binned_Node_Task_Context :: struct
{
	build:      ^Tree_Binned_Build_Context,
	node_index: int,
	start:      int,
	count:      int,
	centroid_bounds: util.Bounding_Box,
}
Tree_Reify_Context :: struct
{
	tree:            ^Tree,
	temporary_nodes: util.Buffer(Tree_Node),
	node_indices:    util.Buffer(i32),
	workspace:       ^Tree_Refit_Refine_Workspace,
}
Tree_Binned_Inner_Task_Data :: struct
{
	start:          int,
	count:          int,
	task_count:     int,
	slots_per_task: int,
	remainder:      int,
}
Tree_Binned_Centroid_Task_Context :: struct
{
	build:        ^Tree_Binned_Build_Context,
	task_data:    Tree_Binned_Inner_Task_Data,
	owner_worker: int,
}
Tree_Binned_Bin_Task_Context :: struct
{
	build:        ^Tree_Binned_Build_Context,
	task_data:    Tree_Binned_Inner_Task_Data,
	owner_worker: int,
	axis:         int,
	axis_min:     f32,
	axis_span:    f32,
	bin_count:    int,
}
Tree_Binned_Partition_Counters :: struct #align(128)
{
	left_count:  i32,
	right_count: i32,
}
Tree_Binned_Partition_Task_Context :: struct
{
	build:        ^Tree_Binned_Build_Context,
	task_data:    Tree_Binned_Inner_Task_Data,
	owner_worker: int,
	split_index:  int,
	counters:     Tree_Binned_Partition_Counters,
}
tree_task_scheduling_status :: proc "contextless" (status: util.Task_Scheduling_Status) -> Physics_Status
{
	switch status
	{
		case .Ok:
			return .Ok;
		case .Capacity_Missing:
			return .Capacity_Missing;
		case .Invalid_Argument, .Procedure_Missing, .Empty, .Stop, .Disposed:
			return .Invalid_Argument;
	}
	return .Invalid_Argument;
}
tree_task_record_failure :: proc "contextless" (target: ^u32, status: Physics_Status)
{
	if status == .Ok
	{
		return;
	}
	_, _ = util.atomic_compare_exchange_u32_status(target, u32(Physics_Status.Ok), u32(status));
}
tree_task_filter :: proc "contextless" (tag: u64, filter_context: rawptr) -> util.Presence_Status
{
	if filter_context != nil && tag == (^u64)(filter_context)^
	{
		return .Present;
	}
	return .Missing;
}
tree_task_wait_filtered :: proc "contextless" (
	stack: ^util.Task_Stack, continuation: util.Task_Continuation_Handle, worker_index: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary, tag: u64,
) -> Physics_Status
{
	filter_tag := tag;
	wait_iteration: u32;
	for util.task_stack_continuation_state(stack, continuation) != .Completed
	{
		pop_status := util.task_stack_try_pop_and_run(stack, worker_index, dispatcher, tree_task_filter, &filter_tag);
		if pop_status == .Stop
		{
			return .Invalid_Argument;
		}
		if pop_status == .Success
		{
			wait_iteration = 0;
		}
		else
		{
			util.task_stack_wait_once(&wait_iteration);
		}
	}
	return .Ok;
}
tree_binned_inner_target_task_count :: proc "contextless" (build: ^Tree_Binned_Build_Context, count: int) -> int
{
	return int(math.ceil(f32(build.target_task_count * count) / f32(build.original_subtree_count)));
}
tree_binned_inner_task_data :: proc "contextless" (
	start, count, target_task_count: int,
) -> Tree_Binned_Inner_Task_Data
{
	task_size := max(TREE_BINNED_MIN_SUBTREES_PER_INNER_TASK, count / max(target_task_count, 1));
	task_count := (count + task_size - 1) / task_size;
	return {
		start=start,
		count=count,
		task_count=task_count,
		slots_per_task=count / task_count,
		remainder=count - task_count * (count / task_count),
	};
}
tree_binned_inner_task_interval :: proc "contextless" (
	data: Tree_Binned_Inner_Task_Data, task_id: int,
) -> (start, count: int)
{
	remaindered_count := min(data.remainder, task_id);
	start = data.start + (data.slots_per_task + 1) * remaindered_count +
		data.slots_per_task * (task_id - remaindered_count);
	count = data.slots_per_task;
	if task_id < data.remainder
	{
		count += 1;
	}
	return;
}
tree_binned_push_inner_tasks :: proc "contextless" (
	build: ^Tree_Binned_Build_Context, task_context: rawptr, task_data: Tree_Binned_Inner_Task_Data,
	procedure: Tree_Task_Procedure, worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary, tag: u64,
) -> Physics_Status
{
	context = runtime.default_context();
	reservation, reserve_status := util.task_stack_reserve_batch(
		build.task_stack, dispatcher, worker_index, task_data.task_count, tag, .Present,
	);
	if reserve_status != .Ok
	{
		return tree_task_scheduling_status(reserve_status);
	}
	for task_id in 0 ..< task_data.task_count
	{
		reservation.tasks[task_id] = {
			task_context=task_context,
			task_id=i64(task_id),
			procedure_index=u16(procedure),
			procedure_state=.Present,
		};
	}
	commit_status := util.task_stack_commit_batch(build.task_stack, dispatcher, &reservation);
	if commit_status != .Ok
	{
		return tree_task_scheduling_status(commit_status);
	}
	return tree_task_wait_filtered(
		build.task_stack, reservation.continuation_handle, worker_index, dispatcher, tag,
	);
}
tree_binned_centroid_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	_ = dispatcher;
	ctx := (^Tree_Binned_Centroid_Task_Context)(task_context);
	start, count := tree_binned_inner_task_interval(ctx.task_data, int(task_id));
	result_index := ctx.owner_worker * ctx.build.workspace.binned_inner_slot_count + int(task_id);
	ctx.build.workspace.binned_centroid_bounds.memory[result_index] = tree_node_child_centroid_bounds(
		ctx.build.references, start, count,
	);
	ctx.build.workspace.worker_hits.memory[worker_index] += 1;
}
tree_binned_bin_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	_ = dispatcher;
	ctx := (^Tree_Binned_Bin_Task_Context)(task_context);
	workspace := ctx.build.workspace;
	slot := ctx.owner_worker * workspace.binned_inner_slot_count + int(task_id);
	bin_base := slot * TREE_BIN_MAX_COUNT;
	for bin_index in 0 ..< ctx.bin_count
	{
		workspace.binned_bins.memory[bin_base + bin_index] = {
			bounds=tree_empty_bounds(), centroid_bounds=tree_empty_bounds(),
		};
	}
	start, count := tree_binned_inner_task_interval(ctx.task_data, int(task_id));
	bin_index_base := ctx.owner_worker * workspace.refinement_capacity;
	scale := f32(ctx.bin_count) / ctx.axis_span;
	for index in start ..< start + count
	{
		reference := ctx.build.references.memory[index];
		bin_index := clamp(
			int((tree_node_child_axis_value(reference, ctx.axis) - ctx.axis_min) * scale), 0, ctx.bin_count - 1,
		);
		workspace.binned_bin_indices.memory[bin_index_base + index] = u8(bin_index);
		bin := &workspace.binned_bins.memory[bin_base + bin_index];
		bounds := tree_child_bounds(reference);
		centroid := tree_bounds_centroid_sum(bounds);
		if bin.count == 0
		{
			bin.bounds = bounds;
			bin.centroid_bounds = {min=centroid, max=centroid};
		}
		else
		{
			bin.bounds = tree_bounds_merge(bin.bounds, bounds);
			bin.centroid_bounds.min = util.vector3_min(bin.centroid_bounds.min, centroid);
			bin.centroid_bounds.max = util.vector3_max(bin.centroid_bounds.max, centroid);
		}
		bin.count += 1;
		bin.leaf_count += reference.leaf_count;
	}
	workspace.worker_hits.memory[worker_index] += 1;
}
tree_binned_partition_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	_ = dispatcher;
	ctx := (^Tree_Binned_Partition_Task_Context)(task_context);
	start, count := tree_binned_inner_task_interval(ctx.task_data, int(task_id));
	bin_index_base := ctx.owner_worker * ctx.build.workspace.refinement_capacity;
	local_left_count := 0;
	for index in start ..< start + count
	{
		if int(ctx.build.workspace.binned_bin_indices.memory[bin_index_base + index]) < ctx.split_index
		{
			local_left_count += 1;
		}
	}
	local_right_count := count - local_left_count;
	left_start := int(sync.atomic_add_explicit(&ctx.counters.left_count, i32(local_left_count), .Seq_Cst));
	right_end := ctx.task_data.count -
		int(sync.atomic_add_explicit(&ctx.counters.right_count, i32(local_right_count), .Seq_Cst));
	left_offset, right_offset := 0, 0;
	for index in start ..< start + count
	{
		reference := ctx.build.references.memory[index];
		if int(ctx.build.workspace.binned_bin_indices.memory[bin_index_base + index]) < ctx.split_index
		{
			ctx.build.scratch.memory[ctx.task_data.start + left_start + left_offset] = reference;
			left_offset += 1;
		}
		else
		{
			ctx.build.scratch.memory[ctx.task_data.start + right_end - local_right_count + right_offset] = reference;
			right_offset += 1;
		}
	}
	ctx.build.workspace.worker_hits.memory[worker_index] += 1;
}
tree_binned_partition_node_children_tasked :: proc "contextless" (
	build: ^Tree_Binned_Build_Context, node_index, start, count, worker_index: int,
	centroid_bounds: util.Bounding_Box, centroid_state: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> (left_count: int, left_centroid_bounds, right_centroid_bounds: util.Bounding_Box, status: Physics_Status)
{
	node_centroid_bounds := centroid_bounds;
	target_inner_task_count := tree_binned_inner_target_task_count(build, count);
	inner_tasks := tree_binned_inner_task_data(start, count, target_inner_task_count);
	if centroid_state == .Missing
	{
		if inner_tasks.task_count > 1
		{
			centroid_context := Tree_Binned_Centroid_Task_Context{
				build=build, task_data=inner_tasks, owner_worker=worker_index,
			};
			tag := u64(uintptr(&centroid_context)) ~ 0xb0a1_bf32_1000_0000;
			status = tree_binned_push_inner_tasks(
				build, &centroid_context, inner_tasks, .Binned_Centroid, worker_index, dispatcher, tag,
			);
			if status != .Ok
			{
				return;
			}
			node_centroid_bounds = build.workspace.binned_centroid_bounds.memory[
				worker_index * build.workspace.binned_inner_slot_count
			];
			for task_index in 1 ..< inner_tasks.task_count
			{
				bounds := build.workspace.binned_centroid_bounds.memory[
					worker_index * build.workspace.binned_inner_slot_count + task_index
				];
				node_centroid_bounds = tree_bounds_merge(node_centroid_bounds, bounds);
			}
		}
		else
		{
			node_centroid_bounds = tree_node_child_centroid_bounds(build.references, start, count);
		}
	}
	span := util.vector3_subtract(node_centroid_bounds.max, node_centroid_bounds.min);
	if span.x <= 1e-12 && span.y <= 1e-12 && span.z <= 1e-12
	{
		left_count = count / 2;
		tree_temporary_write_node_children(
			build.references,
			build.temporary_nodes,
			node_index,
			start,
			count,
			left_count
		);
		left_centroid_bounds = node_centroid_bounds;
		right_centroid_bounds = node_centroid_bounds;
		status = .Ok;
		return;
	}
	if count <= TREE_MICROSWEEP_THRESHOLD || inner_tasks.task_count <= 1
	{
		left_count = tree_binned_partition_node_children(build.references, build.scratch, start, count);
		tree_temporary_write_node_children(
			build.references,
			build.temporary_nodes,
			node_index,
			start,
			count,
			left_count
		);
		left_centroid_bounds = tree_node_child_centroid_bounds(build.references, start, left_count);
		right_centroid_bounds = tree_node_child_centroid_bounds(
			build.references,
			start + left_count,
			count - left_count
		);
		status = .Ok;
		return;
	}

	axis, axis_min, axis_span := tree_binned_axis(node_centroid_bounds.min, node_centroid_bounds.max);
	bin_count := tree_binned_bin_count(count);
	bin_context := Tree_Binned_Bin_Task_Context{
		build=build, task_data=inner_tasks, owner_worker=worker_index,
		axis=axis, axis_min=axis_min, axis_span=axis_span, bin_count=bin_count,
	};
	tag := u64(uintptr(&bin_context)) ~ 0xb0a1_bf32_2000_0000;
	status = tree_binned_push_inner_tasks(build, &bin_context, inner_tasks, .Binned_Bin, worker_index, dispatcher, tag);
	if status != .Ok
	{
		return;
	}
	merged_bins: [TREE_BIN_MAX_COUNT]Tree_Bin;
	for bin_index in 0 ..< bin_count
	{
		merged_bins[bin_index] = {bounds=tree_empty_bounds(), centroid_bounds=tree_empty_bounds()};
	}
	for task_index in 0 ..< inner_tasks.task_count
	{
		bin_base := (worker_index * build.workspace.binned_inner_slot_count + task_index) * TREE_BIN_MAX_COUNT;
		for bin_index in 0 ..< bin_count
		{
			source := build.workspace.binned_bins.memory[bin_base + bin_index];
			if source.count == 0
			{
				continue;
			}
			target := &merged_bins[bin_index];
			if target.count == 0
			{
				target^ = source;
			}
			else
			{
				target.bounds = tree_bounds_merge(target.bounds, source.bounds);
				target.centroid_bounds = tree_bounds_merge(target.centroid_bounds, source.centroid_bounds);
				target.count += source.count;
				target.leaf_count += source.leaf_count;
			}
		}
	}
	total_leaf_count := int(tree_node_child_reference_leaf_count(build.references, start, count));
	split_index, _, binned_left_centroid_bounds, binned_right_centroid_bounds, split_status :=
		tree_binned_node_child_split_from_bins(&merged_bins, bin_count, total_leaf_count);
	if split_status != .Ok
	{
		left_count = count / 2;
		left_centroid_bounds = node_centroid_bounds;
		right_centroid_bounds = node_centroid_bounds;
	}
	else
	{
		left_centroid_bounds = binned_left_centroid_bounds;
		right_centroid_bounds = binned_right_centroid_bounds;
		partition_context := Tree_Binned_Partition_Task_Context{
			build=build, task_data=inner_tasks, owner_worker=worker_index, split_index=split_index,
		};
		partition_tag := u64(uintptr(&partition_context)) ~ 0xb0a1_bf32_3000_0000;
		status = tree_binned_push_inner_tasks(
			build, &partition_context, inner_tasks, .Binned_Partition, worker_index, dispatcher, partition_tag,
		);
		if status != .Ok
		{
			return;
		}
		left_count = int(sync.atomic_load_explicit(&partition_context.counters.left_count, .Acquire));
		for index in start ..< start + count
		{
			build.references.memory[index] = build.scratch.memory[index];
		}
	}
	tree_temporary_write_node_children(build.references, build.temporary_nodes, node_index, start, count, left_count);
	status = .Ok;
	return;
}
tree_binned_build_node_tasked :: proc "contextless" (
	build: ^Tree_Binned_Build_Context, node_index, start, count, worker_index: int,
	centroid_bounds: util.Bounding_Box, centroid_state: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	context = runtime.default_context();
	if Physics_Status(sync.atomic_load_explicit(&build.status, .Acquire)) != .Ok
	{
		return .Invalid_Argument;
	}
	left_count, left_centroid_bounds, right_centroid_bounds, partition_status := tree_binned_partition_node_children_tasked(
		build, node_index, start, count, worker_index, centroid_bounds, centroid_state, dispatcher,
	);
	if partition_status != .Ok
	{
		return partition_status;
	}
	right_count := count - left_count;
	target_node_task_count := int(math.ceil(
		f32(TREE_BINNED_NODE_TASK_MULTIPLIER * build.target_task_count * count) /
		f32(build.original_subtree_count),
	));
	if target_node_task_count > 1 &&
		left_count >= TREE_BINNED_MIN_SUBTREES_PER_NODE_TASK &&
		right_count >= TREE_BINNED_MIN_SUBTREES_PER_NODE_TASK
	{
		right_context := Tree_Binned_Node_Task_Context{
			build=build, node_index=node_index + left_count, start=start + left_count, count=right_count,
			centroid_bounds=right_centroid_bounds,
		};
		tag := u64(uintptr(&right_context)) ~ 0xb0a1_bf32_0000_0000;
		reservation, reserve_status := util.task_stack_reserve_batch(
			build.task_stack, dispatcher, worker_index, 1, tag, .Present,
		);
		if reserve_status != .Ok
		{
			return tree_task_scheduling_status(reserve_status);
		}
		reservation.tasks[0] = {
			task_context=&right_context,
			procedure_index=u16(Tree_Task_Procedure.Binned_Node),
			procedure_state=.Present,
		};
		commit_status := util.task_stack_commit_batch(build.task_stack, dispatcher, &reservation);
		if commit_status != .Ok
		{
			return tree_task_scheduling_status(commit_status);
		}
		left_status := Physics_Status.Ok;
		if left_count > 1
		{
			left_status = tree_binned_build_node_tasked(
				build, node_index + 1, start, left_count, worker_index, left_centroid_bounds, .Present, dispatcher,
			);
		}
		wait_status := tree_task_wait_filtered(
			build.task_stack, reservation.continuation_handle, worker_index, dispatcher, tag,
		);
		if left_status != .Ok
		{
			return left_status;
		}
		if wait_status != .Ok
		{
			return wait_status;
		}
		return Physics_Status(sync.atomic_load_explicit(&build.status, .Acquire));
	}
	if left_count > 1
	{
		status := tree_binned_build_node_tasked(
			build, node_index + 1, start, left_count, worker_index, left_centroid_bounds, .Present, dispatcher,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if right_count > 1
	{
		return tree_binned_build_node_tasked(
			build, node_index + left_count, start + left_count, right_count, worker_index,
			right_centroid_bounds, .Present, dispatcher,
		);
	}
	return .Ok;
}
tree_binned_node_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = task_id;
	_ = range_start;
	_ = range_end;
	ctx := (^Tree_Binned_Node_Task_Context)(task_context);
	status := tree_binned_build_node_tasked(
		ctx.build, ctx.node_index, ctx.start, ctx.count, worker_index, ctx.centroid_bounds, .Present, dispatcher,
	);
	tree_task_record_failure(&ctx.build.status, status);
	ctx.build.workspace.worker_hits.memory[worker_index] += 1;
}
tree_reify_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = task_id;
	_ = dispatcher;
	ctx := (^Tree_Reify_Context)(task_context);
	tree_reify_binned_refinement_range(
		ctx.tree, ctx.temporary_nodes, ctx.node_indices, int(range_start), int(range_end),
	);
	ctx.workspace.worker_hits.memory[worker_index] += 1;
}
tree_binned_refine_subtrees_tasked :: proc "contextless" (
	tree: ^Tree, subtrees, scratch: util.Buffer(Tree_Node_Child), subtree_count: int,
	temporary_nodes: util.Buffer(Tree_Node), node_indices: util.Buffer(i32), node_count: int,
	target_task_count, worker_index: int, workspace: ^Tree_Refit_Refine_Workspace,
	task_stack: ^util.Task_Stack, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || subtree_count <= 2 || node_count != subtree_count - 1 ||
		subtree_count > int(subtrees.length) ||
		subtree_count > int(scratch.length) ||
		node_count > int(temporary_nodes.length) ||
		node_count > int(node_indices.length) || target_task_count <= 1
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	_ = util.buffer_clear(temporary_nodes, 0, node_count);
	build := Tree_Binned_Build_Context{
		references=subtrees,
		scratch=scratch,
		temporary_nodes=temporary_nodes,
		workspace=workspace,
		task_stack=task_stack,
		original_subtree_count=subtree_count,
		target_task_count=target_task_count,
		status=u32(Physics_Status.Ok),
	};
	status := tree_binned_build_node_tasked(&build, 0, 0, subtree_count, worker_index, {}, .Missing, dispatcher);
	if status != .Ok
	{
		return status;
	}
	status = Physics_Status(sync.atomic_load_explicit(&build.status, .Acquire));
	if status != .Ok
	{
		return status;
	}
	reify := Tree_Reify_Context{
		tree=tree,
		temporary_nodes=temporary_nodes,
		node_indices=node_indices,
		workspace=workspace,
	};
	tag := u64(uintptr(&reify)) ~ 0x72e1_0000_0000_0000;
	reservation, reserve_status := util.task_stack_reserve_batch(
		task_stack, dispatcher, worker_index, target_task_count, tag, .Present,
	);
	if reserve_status != .Ok
	{
		return tree_task_scheduling_status(reserve_status);
	}
	nodes_per_task := node_count / target_task_count;
	remainder := node_count - target_task_count * nodes_per_task;
	start_index := 0;
	for index in 0 ..< target_task_count
	{
		count := nodes_per_task;
		if index < remainder
		{
			count += 1;
		}
		reservation.tasks[index] = {
			task_context=&reify,
			task_id=i64(index),
			range_start=i64(start_index),
			range_end=i64(start_index + count),
			procedure_index=u16(Tree_Task_Procedure.Reify),
			procedure_state=.Present,
		};
		start_index += count;
	}
	commit_status := util.task_stack_commit_batch(task_stack, dispatcher, &reservation);
	if commit_status != .Ok
	{
		return tree_task_scheduling_status(commit_status);
	}
	return tree_task_wait_filtered(
		task_stack, reservation.continuation_handle, worker_index, dispatcher, tag,
	);
}
tree_execute_refinement_task :: proc "contextless" (
	ctx: ^Tree_Update2_Context, worker_index: int, task: Tree_Maintenance_Task,
) -> Physics_Status
{
	workspace := ctx.workspace;
	stride := workspace.refinement_capacity;
	node_stride := stride - 1;
	subtrees := util.Buffer(Tree_Node_Child){
		memory=&workspace.refinement_subtrees.memory[worker_index * stride], length=i32(stride), id=-1,
	};
	scratch := util.Buffer(Tree_Node_Child){
		memory=&workspace.refinement_subtree_scratch.memory[worker_index * stride], length=i32(stride), id=-1,
	};
	temporary_nodes := util.Buffer(Tree_Node){
		memory=&workspace.temporary_nodes.memory[worker_index * node_stride], length=i32(node_stride), id=-1,
	};
	node_indices := util.Buffer(i32){
		memory=&workspace.node_indices.memory[worker_index * node_stride], length=i32(node_stride), id=-1,
	};
	stack := util.Buffer(Tree_Refinement_Root_Stack_Entry){
		memory=&workspace.root_stack_entries.memory[worker_index * stride], length=i32(stride), id=-1,
	};
	heap := util.Buffer(Tree_Refinement_Heap_Entry){
		memory=&workspace.heap_entries.memory[worker_index * stride], length=i32(stride), id=-1,
	};
	tree := ctx.active_tree;
	schedule := ctx.active_schedule;
	targets := workspace.refinement_targets;
	target_count := ctx.active_target_count;
	total_subtree_leaf_count := ctx.active_total_subtree_leaf_count;
	target_task_budget := ctx.target_active_task_count;
	if task.tree_kind == .Static
	{
		tree = ctx.static_tree;
		schedule = ctx.static_schedule;
		targets = workspace.static_refinement_targets;
		target_count = ctx.static_target_count;
		total_subtree_leaf_count = ctx.static_total_subtree_leaf_count;
		target_task_budget = ctx.target_static_task_count;
	}
	node_count, subtree_count := 0, 0;
	refinement_leaf_count := 0;
	status := Physics_Status.Ok;
	if task.kind == .Root_Refine
	{
		root_size := min(schedule.root_refinement_size, tree.leaf_count);
		refinement_leaf_count = root_size;
		if schedule.use_priority_queue == .Present
		{
			status = tree_collect_root_refinement_priority(
				tree, root_size, schedule.subtree_refinement_size, targets, target_count,
				heap, node_indices, &node_count, subtrees, &subtree_count,
			);
		}
		else
		{
			status = tree_collect_root_refinement_balanced(
				tree, root_size, schedule.subtree_refinement_size, targets, target_count,
				stack, node_indices, &node_count, subtrees, &subtree_count,
			);
		}
	}
	else
	{
		node := &tree.nodes.memory[task.node_index];
		refinement_leaf_count = int(node.a.leaf_count + node.b.leaf_count);
		status = tree_collect_subtree_refinement(
			tree, int(task.node_index), stack, node_indices, &node_count, subtrees, &subtree_count,
		);
	}
	if status != .Ok
	{
		return status;
	}
	if subtree_count <= 2
	{
		return .Ok;
	}
	denominator := max(1, min(schedule.root_refinement_size, tree.leaf_count) + total_subtree_leaf_count);
	refinement_task_count := int(math.ceil(
		f32(target_task_budget * refinement_leaf_count) / f32(denominator),
	));
	if refinement_task_count > 1
	{
		return tree_binned_refine_subtrees_tasked(
			tree, subtrees, scratch, subtree_count, temporary_nodes, node_indices, node_count,
			refinement_task_count, worker_index, workspace, ctx.task_stack, ctx.dispatcher,
		);
	}
	return tree_binned_refine_subtrees(
		tree,
		subtrees,
		scratch,
		subtree_count,
		temporary_nodes,
		node_indices,
		node_count
	);
}
tree_cache_copy_subtree :: proc "contextless" (
	source_nodes: util.Buffer(Tree_Node), target_nodes: util.Buffer(Tree_Node), target_metanodes: util.Buffer(Tree_Metanode),
	source_node_index, target_node_index, parent_node_index, index_in_parent: int,
)
{
	source := source_nodes.memory[source_node_index];
	target_metanodes.memory[target_node_index] = {parent=i32(parent_node_index), index_in_parent=i32(index_in_parent)};
	target := source;
	if source.a.index >= 0
	{
		target.a.index = i32(target_node_index + 1);
		tree_cache_copy_subtree(
			source_nodes, target_nodes, target_metanodes, int(source.a.index), int(target.a.index), target_node_index, 0,
		);
	}
	if source.b.index >= 0
	{
		target.b.index = i32(target_node_index + int(source.a.leaf_count));
		tree_cache_copy_subtree(
			source_nodes, target_nodes, target_metanodes, int(source.b.index), int(target.b.index), target_node_index, 1,
		);
	}
	target_nodes.memory[target_node_index] = target;
}
tree_cache_prepare_tasks :: proc "contextless" (
	source_nodes: util.Buffer(Tree_Node), target_nodes: util.Buffer(Tree_Node), target_metanodes: util.Buffer(Tree_Metanode),
	source_node_index, target_node_index, parent_node_index, index_in_parent, leaf_count_per_task: int,
	workspace: ^Tree_Refit_Refine_Workspace, task_count: ^int,
) -> Physics_Status
{
	source := source_nodes.memory[source_node_index];
	target_metanodes.memory[target_node_index] = {parent=i32(parent_node_index), index_in_parent=i32(index_in_parent)};
	target := source;
	children := [2]Tree_Node_Child{source.a, source.b};
	for child_index in 0 ..< 2
	{
		child := children[child_index];
		if child.index < 0
		{
			continue;
		}
		child_target_index := target_node_index + 1;
		if child_index == 1
		{
			child_target_index = target_node_index + int(source.a.leaf_count);
		}
		target_child := tree_child(&target, child_index);
		target_child.index = i32(child_target_index);
		if child.leaf_count <= i32(leaf_count_per_task)
		{
			if task_count^ >= int(workspace.task_records.length)
			{
				return .Capacity_Missing;
			}
			workspace.task_records.memory[task_count^] = {
				node_index=child.index, target_node_index=i32(child_target_index),
				parent_node_index=i32(target_node_index), index_in_parent=i32(child_index),
				kind=.Cache_Copy, tree_kind=.Active,
			};
			task_count^ += 1;
		}
		else
		{
			status := tree_cache_prepare_tasks(
				source_nodes, target_nodes, target_metanodes, int(child.index), child_target_index,
				target_node_index, child_index, leaf_count_per_task, workspace, task_count,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	target_nodes.memory[target_node_index] = target;
	return .Ok;
}
tree_execute_cache_task :: proc "contextless" (
	ctx: ^Tree_Update2_Context, task: Tree_Maintenance_Task,
)
{
	tree_cache_copy_subtree(
		ctx.active_tree.nodes, ctx.workspace.cache_nodes, ctx.workspace.cache_metanodes,
		int(task.node_index), int(task.target_node_index), int(task.parent_node_index), int(task.index_in_parent),
	);
}
tree_refinement_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	ctx := (^Tree_Update2_Context)(task_context);
	task_index := int(task_id);
	task := ctx.workspace.task_records.memory[task_index];
	if Tree_Maintenance_Tree(range_start) == .Static
	{
		task = ctx.workspace.static_task_records.memory[task_index];
	}
	status := tree_execute_refinement_task(ctx, worker_index, task);
	tree_task_record_failure(&ctx.status, status);
	ctx.workspace.worker_hits.memory[worker_index] += 1;
}
tree_run_refinement_tasks :: proc "contextless" (
	ctx: ^Tree_Update2_Context, tree_kind: Tree_Maintenance_Tree, worker_index: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	context = runtime.default_context();
	task_count := ctx.active_task_count;
	if tree_kind == .Static
	{
		task_count = ctx.static_task_count;
	}
	if task_count <= 0
	{
		return .Ok;
	}
	tag := u64(uintptr(ctx)) ~ (u64(tree_kind) << 56) ~ u64(worker_index + 1);
	reservation, reserve_status := util.task_stack_reserve_batch(
		ctx.task_stack, dispatcher, worker_index, task_count, tag, .Present,
	);
	if reserve_status != .Ok
	{
		return tree_task_scheduling_status(reserve_status);
	}
	for task_index in 0 ..< task_count
	{
		reservation.tasks[task_index] = {
			task_context=ctx,
			task_id=i64(task_index),
			range_start=i64(tree_kind),
			procedure_index=u16(Tree_Task_Procedure.Refinement),
			procedure_state=.Present,
		};
	}
	commit_status := util.task_stack_commit_batch(ctx.task_stack, dispatcher, &reservation);
	if commit_status != .Ok
	{
		return tree_task_scheduling_status(commit_status);
	}
	return tree_task_wait_filtered(
		ctx.task_stack, reservation.continuation_handle, worker_index, dispatcher, tag,
	);
}
tree_prepare_refinement_tasks :: proc "contextless" (
	tree: ^Tree, schedule: Tree_Refinement_Schedule, start_index: ^int,
	targets: util.Buffer(i32), tree_kind: Tree_Maintenance_Tree,
	task_records: util.Buffer(Tree_Maintenance_Task),
) -> (target_count, task_count, total_subtree_leaf_count: int, status: Physics_Status)
{
	status = tree_find_subtree_refinement_targets(
		tree, min(schedule.subtree_refinement_size, tree.leaf_count), schedule.subtree_refinement_count,
		start_index, targets, &target_count,
	);
	if status != .Ok
	{
		return;
	}
	for index in 0 ..< target_count
	{
		if task_count >= int(task_records.length)
		{
			status = .Capacity_Missing;
			return;
		}
		target := targets.memory[index];
		node := &tree.nodes.memory[target];
		total_subtree_leaf_count += int(node.a.leaf_count + node.b.leaf_count);
		task_records.memory[task_count] = {
			node_index=target, kind=.Subtree_Refine, tree_kind=tree_kind,
		};
		task_count += 1;
	}
	if schedule.root_refinement_size > 0 && tree.leaf_count > 2
	{
		if task_count >= int(task_records.length)
		{
			status = .Capacity_Missing;
			return;
		}
		task_records.memory[task_count] = {kind=.Root_Refine, tree_kind=tree_kind};
		task_count += 1;
	}
	status = .Ok;
	return;
}
tree_refine2_single :: proc "contextless" (
	tree: ^Tree, schedule: Tree_Refinement_Schedule, start_index: ^int,
	workspace: ^Tree_Refit_Refine_Workspace, tree_kind: Tree_Maintenance_Tree,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || workspace == nil || workspace.state != .Ready ||
		tree.leaf_count > workspace.maximum_leaves
	{
		return .Invalid_Argument;
	}
	targets := workspace.refinement_targets;
	task_records := workspace.task_records;
	if tree_kind == .Static
	{
		targets = workspace.static_refinement_targets;
		task_records = workspace.static_task_records;
	}
	target_count, task_count, total_subtree_leaf_count, status := tree_prepare_refinement_tasks(
		tree, schedule, start_index, targets, tree_kind, task_records,
	);
	if status != .Ok
	{
		return status;
	}
	ctx := Tree_Update2_Context{workspace=workspace, target_active_task_count=1, target_static_task_count=1};
	if tree_kind == .Active
	{
		ctx.active_tree = tree;
		ctx.active_schedule = schedule;
		ctx.active_target_count = target_count;
		ctx.active_total_subtree_leaf_count = total_subtree_leaf_count;
	}
	else
	{
		ctx.static_tree = tree;
		ctx.static_schedule = schedule;
		ctx.static_target_count = target_count;
		ctx.static_total_subtree_leaf_count = total_subtree_leaf_count;
	}
	for task_index in 0 ..< task_count
	{
		status = tree_execute_refinement_task(&ctx, 0, task_records.memory[task_index]);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
tree_task_request_stop :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = task_id;
	_ = range_start;
	_ = range_end;
	_ = worker_index;
	_ = dispatcher;
	util.task_stack_request_stop((^util.Task_Stack)(task_context));
}
tree_cache_copy_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	_ = worker_index;
	_ = dispatcher;
	ctx := (^Tree_Update2_Context)(task_context);
	tree_execute_cache_task(ctx, ctx.workspace.task_records.memory[int(task_id)]);
}
tree_update2_entry_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = range_start;
	_ = range_end;
	ctx := (^Tree_Update2_Context)(task_context);
	tree_kind := Tree_Maintenance_Tree(task_id);
	status := tree_run_refinement_tasks(ctx, tree_kind, worker_index, dispatcher);
	if status == .Ok && tree_kind == .Active && ctx.cache_active_tree == .Present
	{
		status = tree_refit2_with_cache_optimization_parallel(
			ctx.active_tree, dispatcher, ctx.target_total_task_count, ctx, worker_index,
		);
	}
	tree_task_record_failure(&ctx.status, status);
}
tree_dispatch_update2_entries :: proc "contextless" (
	ctx: ^Tree_Update2_Context, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	context = runtime.default_context();
	if ctx == nil || ctx.workspace == nil || ctx.workspace.state != .Ready || dispatcher == nil ||
		dispatcher.dispatch == nil || dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > ctx.workspace.worker_count
	{
		return .Invalid_Argument;
	}
	stack := &ctx.workspace.task_stack;
	begin_status := util.task_stack_begin_epoch(stack, dispatcher);
	if begin_status != .Ok
	{
		return tree_task_scheduling_status(begin_status);
	}
	ctx.dispatcher = dispatcher;
	ctx.task_stack = stack;
	ctx.status = u32(Physics_Status.Ok);
	entry_count := 1;
	if ctx.static_tree != nil
	{
		entry_count = 2;
	}
	completion := util.Task_Record{
		task_context=stack,
		procedure_index=u16(Tree_Task_Procedure.Request_Stop),
		procedure_state=.Present,
	};
	reservation, reserve_status := util.task_stack_reserve_batch(
		stack, dispatcher, 0, entry_count, 0x7570_6461_7465_3200, .Present, completion,
	);
	if reserve_status != .Ok
	{
		end_status := util.task_stack_end_epoch(stack, dispatcher);
		if end_status != .Ok
		{
			return tree_task_scheduling_status(end_status);
		}
		return tree_task_scheduling_status(reserve_status);
	}
	reservation.tasks[0] = {
		task_context=ctx,
		task_id=i64(Tree_Maintenance_Tree.Active),
		procedure_index=u16(Tree_Task_Procedure.Update2_Entry),
		procedure_state=.Present,
	};
	if entry_count == 2
	{
		reservation.tasks[1] = {
			task_context=ctx,
			task_id=i64(Tree_Maintenance_Tree.Static),
			procedure_index=u16(Tree_Task_Procedure.Update2_Entry),
			procedure_state=.Present,
		};
	}
	commit_status := util.task_stack_commit_batch(stack, dispatcher, &reservation);
	if commit_status != .Ok
	{
		end_status := util.task_stack_end_epoch(stack, dispatcher);
		if end_status != .Ok
		{
			return tree_task_scheduling_status(end_status);
		}
		return tree_task_scheduling_status(commit_status);
	}
	dispatch_status := dispatcher.dispatch(
		dispatcher, util.task_stack_dispatch_worker, dispatcher.worker_count, stack,
	);
	result := Physics_Status(sync.atomic_load_explicit(&ctx.status, .Acquire));
	end_status := util.task_stack_end_epoch(stack, dispatcher);
	if end_status != .Ok
	{
		return tree_task_scheduling_status(end_status);
	}
	if dispatch_status != .Ok
	{
		return .Invalid_Argument;
	}
	return result;
}
tree_refine2_parallel :: proc "contextless" (
	tree: ^Tree, schedule: Tree_Refinement_Schedule, start_index: ^int,
	dispatcher: ^util.Thread_Dispatcher_Boundary, target_task_count: int,
	workspace: ^Tree_Refit_Refine_Workspace,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || start_index == nil || dispatcher == nil || dispatcher.dispatch == nil ||
		target_task_count <= 0 || target_task_count > dispatcher.worker_count || workspace == nil ||
		workspace.state != .Ready || tree.leaf_count > workspace.maximum_leaves
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< workspace.worker_count
	{
		workspace.statuses.memory[worker_index] = .Ok;
		workspace.worker_hits.memory[worker_index] = 0;
	}
	target_count, task_count, total_subtree_leaf_count, status := tree_prepare_refinement_tasks(
		tree, schedule, start_index, workspace.refinement_targets, .Active, workspace.task_records,
	);
	if status != .Ok
	{
		return status;
	}
	ctx := Tree_Update2_Context{
		active_tree=tree,
		workspace=workspace,
		active_schedule=schedule,
		active_target_count=target_count,
		active_task_count=task_count,
		active_total_subtree_leaf_count=total_subtree_leaf_count,
		target_active_task_count=target_task_count,
		target_total_task_count=target_task_count,
		cache_active_tree=.Missing,
	};
	return tree_dispatch_update2_entries(&ctx, dispatcher);
}
tree_rebuild_leaf_locations :: proc "contextless" (tree: ^Tree)
{
	for node_index in 0 ..< tree.node_count
	{
		node := &tree.nodes.memory[node_index];
		if node.a.index < 0
		{
			tree.leaves.memory[tree_decode_leaf(node.a.index)] = tree_leaf_create(node_index, 0);
		}
		if node.b.index < 0
		{
			tree.leaves.memory[tree_decode_leaf(node.b.index)] = tree_leaf_create(node_index, 1);
		}
	}
}
tree_finish_cache_optimization :: proc "contextless" (
	tree: ^Tree, workspace: ^Tree_Refit_Refine_Workspace,
) -> Physics_Status
{
	target_tree := tree^;
	target_tree.nodes = workspace.cache_nodes;
	target_tree.metanodes = workspace.cache_metanodes;
	_, _ = tree_refit_subtree(&target_tree, 0);
	old_nodes := tree.nodes;
	old_metanodes := tree.metanodes;
	tree.nodes = workspace.cache_nodes;
	tree.metanodes = workspace.cache_metanodes;
	workspace.cache_nodes = old_nodes;
	workspace.cache_metanodes = old_metanodes;
	tree_rebuild_leaf_locations(tree);
	return .Ok;
}
tree_refit2_with_cache_optimization_single :: proc "contextless" (
	tree: ^Tree, workspace: ^Tree_Refit_Refine_Workspace,
) -> Physics_Status
{
	if tree == nil || tree.state != .Ready || workspace == nil || workspace.state != .Ready ||
		tree.node_count > int(workspace.cache_nodes.length)
	{
		return .Invalid_Argument;
	}
	if tree.leaf_count <= 2
	{
		return .Ok;
	}
	tree_cache_copy_subtree(tree.nodes, workspace.cache_nodes, workspace.cache_metanodes, 0, 0, -1, -1);
	return tree_finish_cache_optimization(tree, workspace);
}
tree_refit2_with_cache_optimization_parallel :: proc "contextless" (
	tree: ^Tree, dispatcher: ^util.Thread_Dispatcher_Boundary, target_task_count: int,
	ctx: ^Tree_Update2_Context, worker_index: int,
) -> Physics_Status
{
	context = runtime.default_context();
	workspace := ctx.workspace;
	if tree == nil || tree.state != .Ready || dispatcher == nil || dispatcher.dispatch == nil ||
		workspace == nil || workspace.state != .Ready || tree.node_count > int(workspace.cache_nodes.length) ||
		target_task_count <= 0 || worker_index < 0 || worker_index >= workspace.worker_count ||
		ctx.task_stack == nil
	{
		return .Invalid_Argument;
	}
	if tree.leaf_count <= 2
	{
		return .Ok;
	}
	leaf_count_per_task := max(32, int(math.ceil(f32(tree.leaf_count) / f32(target_task_count))));
	task_count := 0;
	status := tree_cache_prepare_tasks(
		tree.nodes, workspace.cache_nodes, workspace.cache_metanodes, 0, 0, -1, -1,
		leaf_count_per_task, workspace, &task_count,
	);
	if status != .Ok
	{
		return status;
	}
	if task_count > 0
	{
		tag := u64(uintptr(ctx)) ~ 0xca43_0000_0000_0000 ~ u64(worker_index + 1);
		reservation, reserve_status := util.task_stack_reserve_batch(
			ctx.task_stack, dispatcher, worker_index, task_count, tag, .Present,
		);
		if reserve_status != .Ok
		{
			return tree_task_scheduling_status(reserve_status);
		}
		for task_index in 0 ..< task_count
		{
			reservation.tasks[task_index] = {
				task_context=ctx,
				task_id=i64(task_index),
				procedure_index=u16(Tree_Task_Procedure.Cache_Copy),
				procedure_state=.Present,
			};
		}
		commit_status := util.task_stack_commit_batch(ctx.task_stack, dispatcher, &reservation);
		if commit_status != .Ok
		{
			return tree_task_scheduling_status(commit_status);
		}
		status = tree_task_wait_filtered(
			ctx.task_stack, reservation.continuation_handle, worker_index, dispatcher, tag,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return tree_finish_cache_optimization(tree, workspace);
}
