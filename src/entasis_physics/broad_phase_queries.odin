package entasis_physics

import util "entasis:entasis_utilities"
import "core:sync"

Broad_Phase_Tree_Visitor_Context :: struct
{
	broad_phase: ^Broad_Phase,
	visitor:     Broad_Phase_Pair_Visitor_Proc,
	user_context: rawptr,
	tree_kind:   Broad_Phase_Tree,
}
broad_phase_tree_pair_visitor :: proc "contextless" (
	user_context: rawptr, worker_index, leaf_a, leaf_b: int,
) -> Physics_Status
{
	ctx := (^Broad_Phase_Tree_Visitor_Context)(user_context);
	if ctx == nil || ctx.broad_phase == nil || ctx.visitor == nil ||
		leaf_a < 0 || leaf_a >= ctx.broad_phase.active_tree.leaf_count
	{
		return .Invalid_Argument;
	}
	b := Collidable_Reference{};
	if ctx.tree_kind == .Active
	{
		if leaf_b < 0 || leaf_b >= ctx.broad_phase.active_tree.leaf_count
		{
			return .Invalid_Argument;
		}
		b = ctx.broad_phase.active_leaves.memory[leaf_b];
	}
	else
	{
		if leaf_b < 0 || leaf_b >= ctx.broad_phase.static_tree.leaf_count
		{
			return .Invalid_Argument;
		}
		b = ctx.broad_phase.static_leaves.memory[leaf_b];
	}
	a := ctx.broad_phase.active_leaves.memory[leaf_a];
	if ctx.tree_kind == .Active && b.packed < a.packed
	{
		a, b = b, a;
	}
	return ctx.visitor(ctx.user_context, worker_index, {a, b});
}
broad_phase_visit_pairs :: proc "contextless" (
	broad_phase: ^Broad_Phase, visitor: Broad_Phase_Pair_Visitor_Proc,
	user_context: rawptr, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	worker_finalize: Tree_Parallel_Worker_Finalize_Proc = nil,
	worker_finalize_context: rawptr = nil,
) -> Physics_Status
{
	if broad_phase == nil || broad_phase.state != .Ready || visitor == nil
	{
		return .Invalid_Argument;
	}
	active_context := Broad_Phase_Tree_Visitor_Context{
		broad_phase=broad_phase, visitor=visitor, user_context=user_context,
		tree_kind=.Active,
	};
	static_context := Broad_Phase_Tree_Visitor_Context{
		broad_phase=broad_phase, visitor=visitor, user_context=user_context,
		tree_kind=.Static,
	};
	if dispatcher != nil && dispatcher.worker_count > 1
	{
		return tree_self_intertree_visit_parallel(
			&broad_phase.active_tree, &broad_phase.static_tree, dispatcher,
			&broad_phase.query_workspace, broad_phase_tree_pair_visitor,
			&active_context, &static_context,
			worker_finalize, worker_finalize_context,
		);
	}
	status := tree_self_visit(
		&broad_phase.active_tree, broad_phase_tree_pair_visitor, &active_context,
	);
	if status != .Ok
	{
		return status;
	}
	status = tree_intertree_visit(
		&broad_phase.active_tree, &broad_phase.static_tree,
		broad_phase_tree_pair_visitor, &static_context,
	);
	if status != .Ok || worker_finalize == nil
	{
		return status;
	}
	return worker_finalize(worker_finalize_context, 0);
}
Broad_Phase_Output_Visitor_Context :: struct
{
	output: util.Buffer(Broad_Phase_Pair),
	cursor: u32,
}
broad_phase_output_visitor :: proc "contextless" (
	user_context: rawptr, worker_index: int, pair: Broad_Phase_Pair,
) -> Physics_Status
{
	_ = worker_index;
	ctx := (^Broad_Phase_Output_Visitor_Context)(user_context);
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
		ctx.output.memory[cursor] = pair;
		return .Ok;
	}
}
broad_phase_find_pairs :: proc "contextless" (
	broad_phase: ^Broad_Phase, output: util.Buffer(Broad_Phase_Pair),
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> (int, Physics_Status)
{
	if output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	ctx := Broad_Phase_Output_Visitor_Context{output=output};
	status := broad_phase_visit_pairs(
		broad_phase, broad_phase_output_visitor, &ctx, dispatcher,
	);
	output_count := int(sync.atomic_load_explicit(&ctx.cursor, .Acquire));
	if status != .Ok
	{
		return output_count, status;
	}
	return output_count, .Ok;
}
broad_phase_append_query_leaves :: proc "contextless" (
	leaves: util.Buffer(Collidable_Reference), indices: util.Buffer(i32), index_count: int,
	output: util.Buffer(Collidable_Reference), output_count: ^int,
) -> Physics_Status
{
	if output_count^ > int(output.length) - index_count
	{
		return .Capacity_Missing;
	}
	for index in 0 ..< index_count
	{
		leaf_index := int(indices.memory[index]);
		if leaf_index < 0 || leaf_index >= int(leaves.length)
		{
			return .Invalid_Description;
		}
		output.memory[output_count^] = leaves.memory[leaf_index];
		output_count^ += 1;
	}
	return .Ok;
}
broad_phase_volume_any_query :: proc "contextless" (
	broad_phase: ^Broad_Phase, bounds: util.Bounding_Box,
) -> (Collidable_Reference, Reference_State, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready
	{
		return {}, .Missing, .Invalid_Argument;
	}
	leaf_index, state, status := tree_volume_any(&broad_phase.active_tree, bounds);
	if status != .Ok
	{
		return {}, .Missing, status;
	}
	if state == .Present
	{
		if leaf_index < 0 || leaf_index >= broad_phase.active_tree.leaf_count ||
			leaf_index >= int(broad_phase.active_leaves.length)
		{
			return {}, .Missing, .Invalid_Description;
		}
		return broad_phase.active_leaves.memory[leaf_index], .Present, .Ok;
	}
	leaf_index, state, status = tree_volume_any(&broad_phase.static_tree, bounds);
	if status != .Ok
	{
		return {}, .Missing, status;
	}
	if state == .Present
	{
		if leaf_index < 0 || leaf_index >= broad_phase.static_tree.leaf_count ||
			leaf_index >= int(broad_phase.static_leaves.length)
		{
			return {}, .Missing, .Invalid_Description;
		}
		return broad_phase.static_leaves.memory[leaf_index], .Present, .Ok;
	}
	return {}, .Missing, .Ok;
}

broad_phase_volume_query :: proc "contextless" (
	broad_phase: ^Broad_Phase, bounds: util.Bounding_Box,
	output: util.Buffer(Collidable_Reference),
) -> (int, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready || output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	count, status := tree_query_overlaps(&broad_phase.active_tree, bounds, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return 0, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.active_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	count, status = tree_query_overlaps(&broad_phase.static_tree, bounds, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return output_count, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.static_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	return output_count, .Ok;
}
broad_phase_ray_query :: proc "contextless" (
	broad_phase: ^Broad_Phase, ray: Tree_Ray,
	output: util.Buffer(Collidable_Reference),
) -> (int, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready || output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	count, status := tree_ray_query(&broad_phase.active_tree, ray, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return 0, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.active_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	count, status = tree_ray_query(&broad_phase.static_tree, ray, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return output_count, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.static_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	return output_count, .Ok;
}
broad_phase_sweep_query :: proc "contextless" (
	broad_phase: ^Broad_Phase, bounds: util.Bounding_Box, sweep: util.Vector3, maximum_t: f32,
	output: util.Buffer(Collidable_Reference),
) -> (int, Physics_Status)
{
	if broad_phase == nil || broad_phase.state != .Ready || output.memory == nil
	{
		return 0, .Invalid_Argument;
	}
	output_count := 0;
	count, status := tree_sweep_query(&broad_phase.active_tree, bounds, sweep, maximum_t, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return 0, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.active_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	count, status = tree_sweep_query(&broad_phase.static_tree, bounds, sweep, maximum_t, broad_phase.leaf_scratch);
	if status != .Ok
	{
		return output_count, status;
	}
	status = broad_phase_append_query_leaves(
		broad_phase.static_leaves,
		broad_phase.leaf_scratch,
		count,
		output,
		&output_count
	);
	if status != .Ok
	{
		return output_count, status;
	}
	return output_count, .Ok;
}
