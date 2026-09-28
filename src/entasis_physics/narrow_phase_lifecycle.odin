package entasis_physics

import util "entasis:entasis_utilities"
import "core:sync"

narrow_phase_default_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^Simulation,
) -> Physics_Status
{
	_, _ = user_context, simulation;
	return .Ok;
}
narrow_phase_bind_awakener :: proc "contextless" (
	narrow: ^Narrow_Phase, awakener: ^Island_Awakener,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || awakener == nil || awakener.sleeper == nil
	{
		return .Invalid_Argument;
	}
	narrow.awakener = awakener;
	return .Ok;
}
narrow_phase_try_extract_solver_contact_data :: proc "contextless" (
	narrow: ^Narrow_Phase, handle: Constraint_Handle,
	visitor: Contact_Constraint_Data_Visitor_Proc,
	user_context: rawptr = nil,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready ||
		narrow.solver == nil || narrow.solver.state != .Ready
	{
		return .Disposed;
	}
	if visitor == nil
	{
		return .Invalid_Argument;
	}
	if handle.value < 0 ||
		int(handle.value) >= int(narrow.solver.handle_to_constraint.length)
	{
		return .Not_Found;
	}
	location := narrow.solver.handle_to_constraint.memory[handle.value];
	view: Contact_Constraint_Data_View;
	status := Physics_Status.Not_Found;
	if location.set_index == 0
	{
		status = contact_constraint_extract_active_data(
			narrow.solver, &narrow.accessors, handle, &view,
		);
	}
	else if location.set_index > 0 && narrow.awakener != nil
	{
		record_index, resolve_status :=
			island_sleeper_resolve_inactive_constraint_index(
			narrow.awakener.sleeper, handle,
		);
		if resolve_status != .Ok
		{
			return resolve_status;
		}
		source := &narrow.awakener.sleeper.inactive_constraints.memory[
			record_index
		];
		status = contact_constraint_extract_inactive_data(
			narrow.solver, &narrow.accessors, source, &view,
		);
	}
	if status != .Ok
	{
		return status;
	}
	visitor(user_context, &view);
	return .Ok;
}
narrow_phase_transaction_stream_layout :: proc "contextless" (
	candidate_capacity, pair_capacity, worker_count: int,
) -> (Narrow_Phase_Transaction_Stream_Layout, Physics_Status)
{
	if candidate_capacity <= 0 || pair_capacity <= 0 || worker_count <= 0 ||
		worker_count > MAXIMUM_SOLVER_WORKER_COUNT || worker_count > max(int) / 8
	{
		return {}, .Invalid_Argument;
	}
	divisor := worker_count * 8;
	if candidate_capacity > max(int) - (divisor - 1)
	{
		return {}, .Capacity_Missing;
	}
	page_capacity := (candidate_capacity + divisor - 1) / divisor;
	page_capacity = max(page_capacity, NARROW_PHASE_TRANSACTION_STREAM_MIN_PAGE_CAPACITY);
	page_capacity = min(page_capacity, NARROW_PHASE_TRANSACTION_STREAM_MAX_PAGE_CAPACITY);
	if page_capacity > max(int) / worker_count
	{
		return {}, .Capacity_Missing;
	}
	primary_capacity := page_capacity * worker_count;
	if page_capacity - 1 > max(int) / worker_count
	{
		return {}, .Capacity_Missing;
	}
	fragmentation_slack := worker_count * (page_capacity - 1);
	if candidate_capacity > max(int) - fragmentation_slack
	{
		return {}, .Capacity_Missing;
	}
	overflow_required := candidate_capacity + fragmentation_slack;
	if overflow_required > max(int) - (page_capacity - 1)
	{
		return {}, .Capacity_Missing;
	}
	overflow_page_count := max(
		(overflow_required + page_capacity - 1) / page_capacity, 1,
	);
	if overflow_page_count > max(int) / page_capacity
	{
		return {}, .Capacity_Missing;
	}
	overflow_capacity := overflow_page_count * page_capacity;
	if primary_capacity > max(int) - overflow_capacity
	{
		return {}, .Capacity_Missing;
	}
	storage_capacity := primary_capacity + overflow_capacity;
	if storage_capacity > max(int) - pair_capacity ||
		candidate_capacity > max(int) - pair_capacity
	{
		return {}, .Capacity_Missing;
	}
	header_capacity := storage_capacity + pair_capacity;
	order_capacity := candidate_capacity + pair_capacity;
	if storage_capacity > int(max(i32)) || header_capacity > int(max(i32)) ||
		order_capacity > int(max(i32)) || overflow_page_count > int(max(i32))
	{
		return {}, .Capacity_Missing;
	}
	return {
		primary_capacity=page_capacity,
		page_capacity=page_capacity,
		overflow_start=primary_capacity,
		overflow_page_count=overflow_page_count,
		storage_capacity=storage_capacity,
		header_capacity=header_capacity,
		order_capacity=order_capacity,
	}, .Ok;
}
narrow_phase_apply_transaction_stream_layout :: proc "contextless" (
	narrow: ^Narrow_Phase, layout: Narrow_Phase_Transaction_Stream_Layout,
	candidate_capacity: int,
)
{
	narrow.transaction_stream_primary_capacity = layout.primary_capacity;
	narrow.transaction_stream_page_capacity = layout.page_capacity;
	narrow.transaction_stream_overflow_start = layout.overflow_start;
	narrow.transaction_stream_overflow_page_count = layout.overflow_page_count;
	narrow.transaction_stream_storage_capacity = layout.storage_capacity;
	narrow.current_transaction_capacity = candidate_capacity;
}
narrow_phase_ensure_transaction_stream_capacity :: proc (
	narrow: ^Narrow_Phase, candidate_capacity, pair_capacity, worker_count: int,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || narrow.pool == nil
	{
		return .Disposed;
	}
	layout, layout_status := narrow_phase_transaction_stream_layout(
		candidate_capacity, pair_capacity, worker_count,
	);
	if layout_status != .Ok
	{
		return layout_status;
	}
	status := physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_headers, layout.header_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.contact_transactions, layout.storage_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_overflow_page_next,
		layout.overflow_page_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_overflow_page_counts,
		layout.overflow_page_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_order, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.remove_order, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.remove_sort_scratch, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	narrow_phase_apply_transaction_stream_layout(narrow, layout, candidate_capacity);
	return .Ok;
}
narrow_phase_resize_transaction_stream_capacity :: proc (
	narrow: ^Narrow_Phase, candidate_capacity, pair_capacity, worker_count: int,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || narrow.pool == nil
	{
		return .Disposed;
	}
	layout, layout_status := narrow_phase_transaction_stream_layout(
		candidate_capacity, pair_capacity, worker_count,
	);
	if layout_status != .Ok
	{
		return layout_status;
	}
	status := physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_headers, layout.header_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.contact_transactions, layout.storage_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_overflow_page_next,
		layout.overflow_page_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_overflow_page_counts,
		layout.overflow_page_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_order, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.remove_order, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.remove_sort_scratch, layout.order_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	narrow_phase_apply_transaction_stream_layout(narrow, layout, candidate_capacity);
	return .Ok;
}
narrow_phase_reset_transaction_streams :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Physics_Status
{
	if narrow == nil || worker_count <= 0 ||
		worker_count > narrow.active_worker_count ||
		narrow.transaction_stream_primary_capacity <= 0 ||
		narrow.transaction_stream_page_capacity <= 0 ||
		narrow.transaction_stream_overflow_page_count <= 0
	{
		return .Invalid_Argument;
	}
	sync.atomic_store_explicit(
		&narrow.transaction_overflow_page_cursor, i32(0), .Relaxed,
	);
	for worker_index in 0 ..< worker_count
	{
		narrow.transaction_streams[worker_index] = {
			first_overflow_page=-1,
			last_overflow_page=-1,
		};
	}
	return .Ok;
}
narrow_phase_ensure_capacity :: proc (
	narrow: ^Narrow_Phase, broad_phase_candidate_capacity, pair_capacity,
	constraint_capacity,
	inactive_pair_capacity, pending_capacity, collision_child_capacity: int,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || pair_capacity <= 0 ||
		broad_phase_candidate_capacity <= 0 || constraint_capacity <= 0 ||
		inactive_pair_capacity <= 0 || pending_capacity <= 0 ||
		pending_capacity > max(int) / narrow.active_worker_count ||
		collision_child_capacity < narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	transaction_capacity, transaction_capacity_status :=
		simulation_transaction_capacity(
		broad_phase_candidate_capacity, pair_capacity,
	);
	if transaction_capacity_status != .Ok
	{
		return .Invalid_Argument;
	}
	status := pair_cache_ensure_capacity(
		&narrow.pair_cache, pair_capacity, constraint_capacity,
		inactive_pair_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	pending_total_capacity := pending_capacity * narrow.active_worker_count;
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.candidates, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.collision_storage, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.continuations, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.results, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_headers, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.contact_transactions, broad_phase_candidate_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.remove_transactions, pair_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.transaction_order, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.remove_order, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.remove_sort_scratch, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_ensure_transaction_stream_capacity(
		narrow, broad_phase_candidate_capacity, pair_capacity,
		narrow.active_worker_count,
	);
	if status != .Ok
	{
		return status;
	}
	body_capacity := int(narrow.bodies.handle_to_location.length);
	status = physics_ensure_zeroed_buffer_capacity(
		narrow.pool, &narrow.body_claims, body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	body_batch_claim_capacity :=
		max(util.index_set_bundle_capacity(body_capacity), 1) *
		int(narrow.solver.fallback_batch_threshold);
	status = physics_ensure_zeroed_buffer_capacity(
		narrow.pool, &narrow.body_batch_claims,
		body_batch_claim_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_zeroed_buffer_capacity(
		narrow.pool, &narrow.type_claims,
		int(narrow.solver.active_set.batches.length) *
		CONSTRAINT_TYPE_ID_CAPACITY,
	);
	if status != .Ok
	{
		return status;
	}
	touched_claim_capacity := max(
		int(narrow.body_claims.length) + int(narrow.type_claims.length),
		int(narrow.bodies.sets.length),
	);
	status = physics_ensure_buffer_capacity(
		narrow.pool, &narrow.touched_claim_indices, touched_claim_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_zeroed_buffer_capacity(
		narrow.pool, &narrow.pair_table_claim_generations,
		max(
		int(narrow.pair_cache.mapping.table.length),
		int(narrow.bodies.sets.length),
	),
	);
	if status != .Ok
	{
		return status;
	}
	if pending_total_capacity > int(narrow.collision_scratch.continuations.length) ||
		collision_child_capacity > int(narrow.collision_scratch.children.length)
	{
		new_scratch, scratch_status := collision_batcher_take_scratch(
			narrow.pool, pending_total_capacity, collision_child_capacity,
		);
		if scratch_status != .Ok
		{
			return scratch_status;
		}
		old_scratch := narrow.collision_scratch;
		old_child_capacity := narrow.collision_child_capacity;
		old_pending_capacity := narrow.pending_capacity_per_worker;
		narrow.collision_scratch = new_scratch;
		narrow.collision_child_capacity = collision_child_capacity;
		narrow.pending_capacity_per_worker = pending_capacity;
		status = narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
		if status != .Ok
		{
			narrow.collision_scratch = old_scratch;
			narrow.collision_child_capacity = old_child_capacity;
			narrow.pending_capacity_per_worker = old_pending_capacity;
			_ = narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
			collision_batcher_return_scratch(narrow.pool, &new_scratch);
			return status;
		}
		collision_batcher_return_scratch(narrow.pool, &old_scratch);
		narrow.current_transaction_capacity = broad_phase_candidate_capacity;
		return .Ok;
	}
	old_child_capacity := narrow.collision_child_capacity;
	old_pending_capacity := narrow.pending_capacity_per_worker;
	narrow.collision_child_capacity = collision_child_capacity;
	narrow.pending_capacity_per_worker = pending_capacity;
	status = narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
	if status != .Ok
	{
		narrow.collision_child_capacity = old_child_capacity;
		narrow.pending_capacity_per_worker = old_pending_capacity;
		_ = narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
		return status;
	}
	narrow.current_transaction_capacity = broad_phase_candidate_capacity;
	return .Ok;
}
narrow_phase_ensure_runtime_contact_capacity :: proc(
	narrow: ^Narrow_Phase,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || narrow.broad_phase == nil
	{
		return .Disposed;
	}
	active_leaf_count := narrow.broad_phase.active_tree.leaf_count;
	static_leaf_count := narrow.broad_phase.static_tree.leaf_count;
	if active_leaf_count < 0 || static_leaf_count < 0 ||
		active_leaf_count > (max(int) - static_leaf_count) / 4
	{
		return .Capacity_Missing;
	}
	leaf_envelope := active_leaf_count * 4 + static_leaf_count;
	current_pair_capacity := int(narrow.pair_cache.mapping.keys.length);
	current_constraint_capacity := int(narrow.pair_cache.constraint_handle_to_pair.length);
	current_inactive_capacity := int(narrow.pair_cache.inactive_entries.length);
	if narrow.pair_cache.mapping.count > max(int) / 2
	{
		return .Capacity_Missing;
	}
	target_pair_capacity := max(
		current_pair_capacity,
		max(leaf_envelope, narrow.pair_cache.mapping.count * 2),
	);
	target_candidate_capacity := max(
		narrow.current_transaction_capacity, leaf_envelope,
	);
	target_constraint_capacity := max(
		current_constraint_capacity,
		max(target_pair_capacity, int(narrow.solver.handle_pool.next_index)),
	);
	pending_capacity := max(narrow.pending_capacity_per_worker, 1);
	if pending_capacity > max(int) / narrow.active_worker_count
	{
		return .Capacity_Missing;
	}
	pending_total_capacity := pending_capacity * narrow.active_worker_count;
	transaction_capacity, transaction_capacity_status :=
		simulation_transaction_capacity(
		target_candidate_capacity, target_pair_capacity,
	);
	if transaction_capacity_status != .Ok
	{
		return .Capacity_Missing;
	}
	body_capacity := int(narrow.bodies.handle_to_location.length);
	body_batch_claim_capacity :=
		max(util.index_set_bundle_capacity(body_capacity), 1) *
		int(narrow.solver.fallback_batch_threshold);
	type_claim_capacity :=
		int(narrow.solver.active_set.batches.length) *
		CONSTRAINT_TYPE_ID_CAPACITY;
	touched_claim_capacity := max(
		body_capacity + type_claim_capacity,
		int(narrow.bodies.sets.length),
	);
	pair_table_claim_capacity := max(
		int(narrow.pair_cache.mapping.table.length),
		int(narrow.bodies.sets.length),
	);
	collision_child_capacity := max(
		narrow.collision_child_capacity, narrow.active_worker_count,
	);
	if target_pair_capacity <= int(narrow.pair_cache.mapping.keys.length) &&
		target_pair_capacity <= int(narrow.pair_cache.pair_freshness.length) &&
		target_constraint_capacity <= int(narrow.pair_cache.constraint_handle_to_pair.length) &&
		current_inactive_capacity <= int(narrow.pair_cache.inactive_entries.length) &&
		pending_total_capacity <= int(narrow.candidates.length) &&
		pending_total_capacity <= int(narrow.collision_storage.length) &&
		pending_total_capacity <= int(narrow.continuations.length) &&
		pending_total_capacity <= int(narrow.results.length) &&
		transaction_capacity <= int(narrow.transaction_headers.length) &&
		target_candidate_capacity <= int(narrow.contact_transactions.length) &&
		target_pair_capacity <= int(narrow.remove_transactions.length) &&
		transaction_capacity <= int(narrow.transaction_order.length) &&
		transaction_capacity <= int(narrow.remove_order.length) &&
		transaction_capacity <= int(narrow.remove_sort_scratch.length) &&
		body_capacity <= int(narrow.body_claims.length) &&
		body_batch_claim_capacity <= int(narrow.body_batch_claims.length) &&
		type_claim_capacity <= int(narrow.type_claims.length) &&
		touched_claim_capacity <= int(narrow.touched_claim_indices.length) &&
		pair_table_claim_capacity <= int(narrow.pair_table_claim_generations.length) &&
		pending_total_capacity <= int(narrow.collision_scratch.continuations.length) &&
		collision_child_capacity <= int(narrow.collision_scratch.children.length) &&
		target_candidate_capacity <= narrow.current_transaction_capacity
	{
		return .Ok;
	}
	return narrow_phase_ensure_capacity(
		narrow, max(target_candidate_capacity, 1), max(target_pair_capacity, 1),
		max(target_constraint_capacity, 1), max(current_inactive_capacity, 1),
		pending_capacity, collision_child_capacity,
	);
}
narrow_phase_resize :: proc (
	narrow: ^Narrow_Phase, broad_phase_candidate_capacity, pair_capacity,
	constraint_capacity,
	inactive_pair_capacity, pending_capacity, collision_child_capacity: int,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || pair_capacity <= 0 ||
		broad_phase_candidate_capacity <= 0 || constraint_capacity <= 0 ||
		inactive_pair_capacity <= 0 || pending_capacity <= 0 ||
		pending_capacity > max(int) / narrow.active_worker_count ||
		collision_child_capacity < narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	transaction_capacity, transaction_capacity_status :=
		simulation_transaction_capacity(
		broad_phase_candidate_capacity, pair_capacity,
	);
	if transaction_capacity_status != .Ok
	{
		return .Invalid_Argument;
	}
	required_constraint_capacity := max(
		constraint_capacity, int(narrow.solver.handle_pool.next_index),
	);
	status := pair_cache_resize(
		&narrow.pair_cache, pair_capacity,
		required_constraint_capacity, inactive_pair_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	pending_total_capacity := pending_capacity * narrow.active_worker_count;
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.candidates, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.collision_storage, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.continuations, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.results, pending_total_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_headers, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.contact_transactions, broad_phase_candidate_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.remove_transactions, pair_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.transaction_order, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.remove_order, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.remove_sort_scratch, transaction_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_resize_transaction_stream_capacity(
		narrow, broad_phase_candidate_capacity, pair_capacity,
		narrow.active_worker_count,
	);
	if status != .Ok
	{
		return status;
	}
	required_body_capacity := max(
		int(narrow.bodies.handle_pool.next_index), 1,
	);
	status = physics_resize_zeroed_buffer_capacity(
		narrow.pool, &narrow.body_claims, required_body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	body_batch_claim_capacity :=
		max(
		util.index_set_bundle_capacity(required_body_capacity), 1,
	) * int(narrow.solver.fallback_batch_threshold);
	status = physics_resize_zeroed_buffer_capacity(
		narrow.pool, &narrow.body_batch_claims,
		body_batch_claim_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_zeroed_buffer_capacity(
		narrow.pool, &narrow.type_claims,
		int(narrow.solver.active_set.batches.length) *
		CONSTRAINT_TYPE_ID_CAPACITY,
	);
	if status != .Ok
	{
		return status;
	}
	touched_claim_capacity := max(
		int(narrow.body_claims.length) + int(narrow.type_claims.length),
		int(narrow.bodies.sets.length),
	);
	status = physics_resize_buffer_capacity(
		narrow.pool, &narrow.touched_claim_indices, touched_claim_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_zeroed_buffer_capacity(
		narrow.pool, &narrow.pair_table_claim_generations,
		max(
		int(narrow.pair_cache.mapping.table.length),
		int(narrow.bodies.sets.length),
	),
	);
	if status != .Ok
	{
		return status;
	}
	new_scratch, scratch_status := collision_batcher_take_scratch(
		narrow.pool, pending_total_capacity, collision_child_capacity,
	);
	if scratch_status != .Ok
	{
		return scratch_status;
	}
	old_scratch := narrow.collision_scratch;
	old_child_capacity := narrow.collision_child_capacity;
	old_pending_capacity := narrow.pending_capacity_per_worker;
	narrow.collision_scratch = new_scratch;
	narrow.collision_child_capacity = collision_child_capacity;
	narrow.pending_capacity_per_worker = pending_capacity;
	status = narrow_phase_prepare_batchers(
		narrow, narrow.active_worker_count,
	);
	if status != .Ok
	{
		narrow.collision_scratch = old_scratch;
		narrow.collision_child_capacity = old_child_capacity;
		narrow.pending_capacity_per_worker = old_pending_capacity;
		_ = narrow_phase_prepare_batchers(narrow, narrow.active_worker_count);
		collision_batcher_return_scratch(narrow.pool, &new_scratch);
		return status;
	}
	collision_batcher_return_scratch(narrow.pool, &old_scratch);
	narrow.current_transaction_capacity = broad_phase_candidate_capacity;
	return .Ok;
}
narrow_phase_default_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference, speculative_margin: ^f32,
) -> Collision_Testing_State
{
	_, _, _, _, _ = user_context, worker_index, a, b, speculative_margin;
	return .Allow;
}
narrow_phase_default_allow_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference, child_a, child_b: int,
) -> Collision_Testing_State
{
	_, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b;
	return .Allow;
}
narrow_phase_default_configure :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference,
	manifold: ^Manifold_Result, material: ^Contact_Material_Properties,
) -> Collision_Testing_State
{
	_, _, _, _, _ = user_context, worker_index, a, b, manifold;
	material^ = {friction_coefficient=1, spring_settings=spring_settings_create(30, 1), maximum_recovery_velocity=2};
	return .Allow;
}
narrow_phase_default_configure_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: Collidable_Reference,
	child_a, child_b: int, manifold: ^Convex_Contact_Manifold,
) -> Collision_Testing_State
{
	_, _, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b, manifold;
	return .Allow;
}
narrow_phase_default_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}
narrow_phase_default_callbacks :: proc "contextless" () -> Narrow_Phase_Callbacks
{
	return {
		initialize=narrow_phase_default_initialize,
		allow=narrow_phase_default_allow,
		allow_child=narrow_phase_default_allow_child,
		configure=narrow_phase_default_configure,
		configure_child=narrow_phase_default_configure_child,
		dispose=narrow_phase_default_dispose,
	};
}
narrow_phase_callbacks_validate :: proc "contextless" (callbacks: Narrow_Phase_Callbacks) -> Physics_Status
{
	if callbacks.initialize == nil || callbacks.allow == nil || callbacks.allow_child == nil ||
		callbacks.configure == nil || callbacks.configure_child == nil || callbacks.dispose == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}
narrow_phase_initialize :: proc (
	narrow: ^Narrow_Phase, broad_phase: ^Broad_Phase, bodies: ^Bodies, statics: ^Statics,
	shapes: ^Shape_Registry, solver: ^Solver, tasks: ^Collision_Task_Registry,
	sweep_tasks: ^Sweep_Task_Registry,
	callbacks: Narrow_Phase_Callbacks, broad_phase_candidate_capacity,
	pair_capacity, constraint_capacity,
	inactive_pair_capacity, worker_count, pending_capacity, collision_child_capacity: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	transaction_capacity, transaction_capacity_status :=
		simulation_transaction_capacity(
		broad_phase_candidate_capacity, pair_capacity,
	);
	if narrow == nil || narrow.state != .Uninitialized || broad_phase == nil || broad_phase.state != .Ready ||
		bodies == nil || bodies.state != .Allocated || statics == nil || statics.state != .Allocated ||
		shapes == nil || shapes.state != .Allocated || solver == nil || solver.state != .Ready ||
		tasks == nil || tasks.state != .Ready || sweep_tasks == nil || sweep_tasks.state != .Ready ||
		pool == nil || transaction_capacity_status != .Ok || worker_count <= 0 ||
		pending_capacity <= 0 || pending_capacity > max(int) / worker_count ||
		collision_child_capacity < worker_count ||
		narrow_phase_callbacks_validate(callbacks) != .Ok ||
		int(solver.fallback_batch_threshold) > max(int) /
		max(util.index_set_bundle_capacity(int(bodies.handle_to_location.length)), 1)
	{
		return .Invalid_Argument;
	}
	pair_status := pair_cache_initialize(
		&narrow.pair_cache, pair_capacity, constraint_capacity, inactive_pair_capacity,
		pool,
	);
	if pair_status != .Ok
	{
		return pair_status;
	}
	accessor_status := contact_constraint_accessors_initialize(&narrow.accessors);
	if accessor_status != .Ok
	{
		_ = pair_cache_dispose(&narrow.pair_cache);
		return accessor_status;
	}
	pending_total_capacity := pending_capacity * worker_count;
	candidates, candidate_status := util.buffer_pool_take_at_least(
		pool, Broad_Phase_Pair, pending_total_capacity,
	);
	if candidate_status != .Ok
	{
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(candidate_status);
	}
	storage, storage_status := util.buffer_pool_take_at_least(
		pool, Collision_Batcher_Pair, pending_total_capacity,
	);
	if storage_status != .Ok
	{
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(storage_status);
	}
	storage.length = i32(pending_total_capacity);
	continuations, continuation_status := util.buffer_pool_take_at_least(
		pool, Narrow_Phase_CCD_Continuation, pending_total_capacity,
	);
	if continuation_status != .Ok
	{
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(continuation_status);
	}
	results, results_status := util.buffer_pool_take_at_least(
		pool, Narrow_Phase_Pair_Result, pending_total_capacity,
	);
	if results_status != .Ok
	{
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(results_status);
	}
	transaction_headers, transaction_header_status := util.buffer_pool_take_at_least(
		pool, Narrow_Phase_Transaction_Header, transaction_capacity,
	);
	if transaction_header_status != .Ok
	{
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(transaction_header_status);
	}
	contact_transactions, contact_transaction_status := util.buffer_pool_take_at_least(
		pool, Narrow_Phase_Contact_Transaction, broad_phase_candidate_capacity,
	);
	if contact_transaction_status != .Ok
	{
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(contact_transaction_status);
	}
	remove_transactions, remove_transaction_status := util.buffer_pool_take_at_least(
		pool, Narrow_Phase_Remove_Transaction, pair_capacity,
	);
	if remove_transaction_status != .Ok
	{
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(remove_transaction_status);
	}
	transaction_order, order_status := util.buffer_pool_take_at_least(
		pool, i32, transaction_capacity,
	);
	if order_status != .Ok
	{
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(order_status);
	}
	remove_order, remove_order_status := util.buffer_pool_take_at_least(
		pool, Solver_Remove_Order_Entry, int(transaction_headers.length),
	);
	if remove_order_status != .Ok
	{
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(remove_order_status);
	}
	remove_sort_scratch, remove_scratch_status := util.buffer_pool_take_at_least(
		pool, Solver_Remove_Order_Entry, int(transaction_headers.length),
	);
	if remove_scratch_status != .Ok
	{
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(remove_scratch_status);
	}
	body_claims, body_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Body_Claim, int(bodies.handle_to_location.length),
	);
	if body_claim_status != .Ok
	{
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(body_claim_status);
	}
	body_batch_claims, body_batch_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Body_Batch_Claim,
		max(util.index_set_bundle_capacity(int(bodies.handle_to_location.length)), 1) *
		int(solver.fallback_batch_threshold),
	);
	if body_batch_claim_status != .Ok
	{
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(body_batch_claim_status);
	}
	type_claims, type_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Count_Claim,
		int(solver.active_set.batches.length) * CONSTRAINT_TYPE_ID_CAPACITY,
	);
	if type_claim_status != .Ok
	{
		physics_return_buffer(pool, &body_batch_claims);
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(type_claim_status);
	}
	pair_table_claim_generations, pair_claim_status := util.buffer_pool_take_at_least(
		pool, u32, max(
		int(narrow.pair_cache.mapping.table.length),
		int(bodies.sets.length),
	),
	);
	if pair_claim_status != .Ok
	{
		physics_return_buffer(pool, &type_claims);
		physics_return_buffer(pool, &body_batch_claims);
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(pair_claim_status);
	}
	collision_scratch, collision_scratch_status := collision_batcher_take_scratch(
		pool, int(storage.length), collision_child_capacity,
	);
	if collision_scratch_status != .Ok
	{
		physics_return_buffer(pool, &pair_table_claim_generations);
		physics_return_buffer(pool, &type_claims);
		physics_return_buffer(pool, &body_batch_claims);
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return collision_scratch_status;
	}
	touched_claim_indices, touched_claim_status := util.buffer_pool_take_at_least(
		pool, i32, max(
		int(body_claims.length) + int(type_claims.length),
		int(bodies.sets.length),
	),
	);
	if touched_claim_status != .Ok
	{
		collision_batcher_return_scratch(pool, &collision_scratch);
		physics_return_buffer(pool, &pair_table_claim_generations);
		physics_return_buffer(pool, &type_claims);
		physics_return_buffer(pool, &body_batch_claims);
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &remove_sort_scratch);
		physics_return_buffer(pool, &remove_order);
		physics_return_buffer(pool, &transaction_order);
		physics_return_buffer(pool, &remove_transactions);
		physics_return_buffer(pool, &contact_transactions);
		physics_return_buffer(pool, &transaction_headers);
		physics_return_buffer(pool, &results);
		physics_return_buffer(pool, &continuations);
		physics_return_buffer(pool, &storage);
		physics_return_buffer(pool, &candidates);
		_ = contact_constraint_accessors_dispose(&narrow.accessors);
		_ = pair_cache_dispose(&narrow.pair_cache);
		return physics_memory_status(touched_claim_status);
	}
	_ = util.buffer_clear(body_claims, 0, int(body_claims.length));
	_ = util.buffer_clear(body_batch_claims, 0, int(body_batch_claims.length));
	_ = util.buffer_clear(type_claims, 0, int(type_claims.length));
	_ = util.buffer_clear(pair_table_claim_generations, 0, int(pair_table_claim_generations.length));
	narrow.callbacks = callbacks;
	narrow.candidates = candidates;
	narrow.collision_storage = storage;
	narrow.collision_scratch = collision_scratch;
	narrow.continuations = continuations;
	narrow.results = results;
	narrow.transaction_headers = transaction_headers;
	narrow.contact_transactions = contact_transactions;
	narrow.remove_transactions = remove_transactions;
	narrow.transaction_order = transaction_order;
	narrow.remove_order = remove_order;
	narrow.remove_sort_scratch = remove_sort_scratch;
	narrow.body_claims = body_claims;
	narrow.body_batch_claims = body_batch_claims;
	narrow.type_claims = type_claims;
	narrow.touched_claim_indices = touched_claim_indices;
	narrow.pair_table_claim_generations = pair_table_claim_generations;
	narrow.broad_phase = broad_phase;
	narrow.bodies = bodies;
	narrow.statics = statics;
	narrow.shapes = shapes;
	narrow.solver = solver;
	narrow.tasks = tasks;
	narrow.sweep_tasks = sweep_tasks;
	narrow.pool = pool;
	narrow.active_worker_count = worker_count;
	narrow.collision_child_capacity = collision_child_capacity;
	narrow.pending_capacity_per_worker = pending_capacity;
	narrow.current_transaction_capacity = broad_phase_candidate_capacity;
	narrow.state = .Ready;
	stream_status := narrow_phase_ensure_transaction_stream_capacity(
		narrow, broad_phase_candidate_capacity, pair_capacity, worker_count,
	);
	if stream_status != .Ok
	{
		_ = narrow_phase_dispose(narrow);
		return stream_status;
	}
	reset_status := narrow_phase_reset_transaction_streams(narrow, worker_count);
	if reset_status != .Ok
	{
		_ = narrow_phase_dispose(narrow);
		return reset_status;
	}
	for worker_index in 0 ..< worker_count
	{
		narrow.worker_contexts[worker_index] = {narrow=narrow, worker_index=i32(worker_index)};
	}
	return .Ok;
}
narrow_phase_activate :: proc "contextless" (
	narrow: ^Narrow_Phase, simulation: ^Simulation,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready ||
		narrow.callback_activation != .Missing || narrow.awakener == nil ||
		simulation == nil || simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	status := narrow.callbacks.initialize(
		narrow.callbacks.user_context, simulation,
	);
	if status != .Ok
	{
		return status;
	}
	narrow.callback_activation = .Present;
	return .Ok;
}
narrow_phase_bind_default_stored_completion :: proc "contextless" (
	narrow: ^Narrow_Phase, material: Contact_Material_Properties,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready ||
		narrow.callback_activation != .Present ||
		narrow.stored_completion_state != .Missing
	{
		return .Invalid_Argument;
	}
	if constraint_contact_material_validate(material) != .Ok
	{
		return .Invalid_Description;
	}
	narrow.stored_completion_material = material;
	narrow.stored_completion_state = .Present;
	return .Ok;
}
narrow_phase_clear :: proc (narrow: ^Narrow_Phase) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready
	{
		return .Disposed;
	}
	status := pair_cache_clear(&narrow.pair_cache);
	if status != .Ok
	{
		return status;
	}
	narrow.remove_count = 0;
	narrow.touched_body_count = 0;
	narrow.touched_type_count = 0;
	narrow.prepared_pair_add_count = 0;
	narrow.remove_handle_return_start = 0;
	narrow.claim_generation = 0;
	narrow.last_stage = .Idle;
	reset_status := narrow_phase_reset_transaction_streams(
		narrow, narrow.active_worker_count,
	);
	if reset_status != .Ok
	{
		return reset_status;
	}
	for worker_index in 0 ..< narrow.active_worker_count
	{
		narrow.candidate_counts[worker_index] = 0;
		narrow.transaction_statuses[worker_index] = .Ok;
		narrow.transaction_ranges[worker_index] = {};
		if narrow.batchers[worker_index].state == .Ready
		{
			narrow.batchers[worker_index].pair_count = 0;
		}
	}
	return .Ok;
}
narrow_phase_dispose :: proc (narrow: ^Narrow_Phase) -> Physics_Status
{
	if narrow == nil || narrow.state == .Disposed || narrow.pool == nil
	{
		return .Disposed;
	}
	pool := narrow.pool;
	if narrow.callback_activation == .Present
	{
		narrow.callbacks.dispose(narrow.callbacks.user_context);
	}
	physics_return_buffer(pool, &narrow.pair_table_claim_generations);
	physics_return_buffer(pool, &narrow.touched_claim_indices);
	physics_return_buffer(pool, &narrow.type_claims);
	physics_return_buffer(pool, &narrow.body_batch_claims);
	physics_return_buffer(pool, &narrow.body_claims);
	physics_return_buffer(pool, &narrow.remove_sort_scratch);
	physics_return_buffer(pool, &narrow.remove_order);
	physics_return_buffer(pool, &narrow.transaction_order);
	physics_return_buffer(pool, &narrow.transaction_overflow_page_counts);
	physics_return_buffer(pool, &narrow.transaction_overflow_page_next);
	physics_return_buffer(pool, &narrow.remove_transactions);
	physics_return_buffer(pool, &narrow.contact_transactions);
	physics_return_buffer(pool, &narrow.transaction_headers);
	physics_return_buffer(pool, &narrow.results);
	physics_return_buffer(pool, &narrow.continuations);
	collision_batcher_return_scratch(pool, &narrow.collision_scratch);
	physics_return_buffer(pool, &narrow.collision_storage);
	physics_return_buffer(pool, &narrow.candidates);
	_ = contact_constraint_accessors_dispose(&narrow.accessors);
	_ = pair_cache_dispose(&narrow.pair_cache);
	narrow^ = {state=.Disposed};
	return .Ok;
}
