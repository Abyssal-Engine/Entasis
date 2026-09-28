package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"
import "base:runtime"
import "core:sync"

narrow_phase_prepare_pair_transaction :: proc (
	narrow: ^Narrow_Phase, header: ^Narrow_Phase_Transaction_Header,
	transaction: ^Narrow_Phase_Contact_Transaction,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Stepping || header == nil || transaction == nil ||
		header.kind != .Pending
	{
		return .Invalid_Argument;
	}
	result: Narrow_Phase_Pair_Result;
	intrinsics.mem_copy(
		&result, &transaction.description[0], size_of(Narrow_Phase_Pair_Result),
	);
	pair := header.pair;
	prepared, description_status := narrow_phase_build_pair_description(
		narrow, pair, &result, &transaction.description[0],
	);
	if description_status != .Ok
	{
		return description_status;
	}
	transaction.body_handles = prepared.body_handles;
	transaction.feature_ids = prepared.feature_ids;
	transaction.accessor = prepared.accessor;
	header.type_id = i16(prepared.type_id);
	header.body_count = u8(prepared.body_count);
	index := pair_cache_index_of(&narrow.pair_cache, pair);
	if index >= 0
	{
		old_cache := narrow.pair_cache.mapping.values.memory[index];
		location, location_status := solver_resolve(
			narrow.solver, old_cache.constraint_handle,
		);
		if location_status != .Ok
		{
			return location_status;
		}
		batch := &narrow.solver.active_set.batches.memory[location.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
		if type_batch_index < 0 ||
			type_batch_index >= int(batch.type_batches.length)
		{
			return .Invalid_Argument;
		}
		old_accessor, accessor_status := contact_constraint_accessor_lookup(
			&narrow.accessors, location.type_id,
		);
		if accessor_status != .Ok
		{
			return accessor_status;
		}
		transaction.old_cache = old_cache;
		header.pair_mapping_index = i32(index);
		transaction.old_impulses = contact_constraint_gather_impulses_trusted(
			&batch.type_batches.memory[type_batch_index], old_accessor,
			int(location.index_in_type_batch),
		);
		transaction.old_contact_count = old_accessor.contact_count;
		if location.type_id == prepared.type_id
		{
			transaction.prepared.update = {
				type_record=prepared.type_record,
				type_batch=&batch.type_batches.memory[type_batch_index],
				batch_index=location.batch_index,
				record_index=location.index_in_type_batch,
			};
			transaction.accessor = old_accessor;
			header.kind = .Update;
		}
		else
		{
			transaction.prepared.replace.solver_add.type_record = prepared.type_record;
			header.kind = .Replace;
		}
		return pair_cache_mark_fresh(&narrow.pair_cache, index);
	}
	transaction.old_cache = {};
	transaction.old_impulses = {};
	transaction.old_contact_count = 0;
	transaction.prepared.add.solver_add.type_record = prepared.type_record;
	header.pair_mapping_index = -1;
	header.kind = .Add;
	return .Ok;
}
narrow_phase_update_builtin_convex_existing :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
	manifold: ^Convex_Contact_Manifold, material: Contact_Material_Properties,
	mapping_index: int,
	$body_count, $contact_count: int,
) -> Reference_State
{
	cache := &narrow.pair_cache.mapping.values.memory[mapping_index];
	location := narrow.solver.handle_to_constraint.memory[cache.constraint_handle.value];
	desired_type_id := i32(CONTACT_1_TYPE_ID + contact_count - 1);
	when body_count == 1
	{
		desired_type_id = i32(CONTACT_1_ONE_BODY_TYPE_ID + contact_count - 1);
	}
	if location.type_id != desired_type_id
	{
		return .Missing;
	}
	batch := &narrow.solver.active_set.batches.memory[location.batch_index];
	type_batch := &batch.type_batches.memory[
		batch.type_id_to_batch_index[desired_type_id]
	];
	record_index := int(location.index_in_type_batch);
	contact_constraint_write_convex_prestep_lane(
		type_batch, record_index, manifold, material,
		body_count, contact_count,
	);
	if contact_constraint_convex_features_equal(
		&cache.feature_ids, manifold, contact_count,
	) != .Equal
	{
		old_features := cache.feature_ids;
		contact_constraint_redistribute_convex_impulses_trusted(
			type_batch, record_index, old_features, manifold, contact_count,
		);
		for contact_index in 0 ..< contact_count
		{
			cache.feature_ids[contact_index] =
				manifold.contacts[contact_index].feature_id;
		}
	}
	narrow.pair_cache.pair_freshness.memory[mapping_index] =
		narrow.pair_cache.freshness_generation;
	_ = pair;
	return .Present;
}
narrow_phase_try_update_builtin_convex_existing :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
	result: ^Narrow_Phase_Pair_Result, mapping_index: int,
) -> Reference_State
{
	if result.manifold.kind != .Convex || narrow.callbacks.select_constraint != nil
	{
		return .Missing;
	}
	body_count := 2;
	if collidable_reference_mobility(pair.a) == .Static ||
		collidable_reference_mobility(pair.b) == .Static
	{
		body_count = 1;
	}
	switch result.manifold.convex.count
	{
		case 1:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, &result.manifold.convex, result.material, mapping_index, 1, 1,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, &result.manifold.convex, result.material, mapping_index, 2, 1,
			);
		case 2:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, &result.manifold.convex, result.material, mapping_index, 1, 2,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, &result.manifold.convex, result.material, mapping_index, 2, 2,
			);
		case 3:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, &result.manifold.convex, result.material, mapping_index, 1, 3,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, &result.manifold.convex, result.material, mapping_index, 2, 3,
			);
		case 4:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, &result.manifold.convex, result.material, mapping_index, 1, 4,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, &result.manifold.convex, result.material, mapping_index, 2, 4,
			);
		case:
			return .Missing;
	}
}
narrow_phase_prepare_existing_pair_transaction :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair, result: ^Narrow_Phase_Pair_Result,
	mapping_index: int, header: ^Narrow_Phase_Transaction_Header,
	deferred: ^Narrow_Phase_Contact_Transaction,
) -> (Reference_State, Physics_Status)
{
	description: [CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8;
	prepared, description_status := narrow_phase_build_pair_description(
		narrow, pair, result, &description[0],
	);
	if description_status != .Ok
	{
		return .Missing, description_status;
	}
	old_cache := narrow.pair_cache.mapping.values.memory[mapping_index];
	location, location_status := solver_resolve(
		narrow.solver, old_cache.constraint_handle,
	);
	if location_status != .Ok
	{
		return .Missing, location_status;
	}
	batch := &narrow.solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	if type_batch_index < 0 || type_batch_index >= int(batch.type_batches.length)
	{
		return .Missing, .Invalid_Argument;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	old_accessor, accessor_status := contact_constraint_accessor_lookup(
		&narrow.accessors, location.type_id,
	);
	if accessor_status != .Ok
	{
		return .Missing, accessor_status;
	}
	old_impulses := contact_constraint_gather_impulses_trusted(
		type_batch, old_accessor, int(location.index_in_type_batch),
	);
	payload_index := header.payload_index;
	header^ = {
		pair=pair,
		payload_index=payload_index,
		pair_mapping_index=i32(mapping_index),
		type_id=i16(prepared.type_id),
		body_count=u8(prepared.body_count),
	};
	deferred.body_handles = prepared.body_handles;
	deferred.old_cache = old_cache;
	deferred.feature_ids = prepared.feature_ids;
	deferred.old_impulses = old_impulses;
	deferred.old_contact_count = old_accessor.contact_count;
	if location.type_id == prepared.type_id
	{
		deferred.prepared.update = {
			prepared.type_record, type_batch, location.batch_index,
			location.index_in_type_batch,
		};
		deferred.accessor = old_accessor;
		header.kind = .Update;
	}
	else
	{
		deferred.prepared.replace.solver_add.type_record = prepared.type_record;
		deferred.accessor = prepared.accessor;
		header.kind = .Replace;
	}
	intrinsics.mem_copy(
		&deferred.description[0], &description[0],
		int(prepared.type_record.description_size),
	);
	fresh_status := pair_cache_mark_fresh(&narrow.pair_cache, mapping_index);
	return .Present, fresh_status;
}
narrow_phase_fused_awakening_set_index :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
) -> i32
{
	if narrow.inactive_sets_present != .Present ||
		collidable_reference_mobility(pair.a) == .Static ||
		collidable_reference_mobility(pair.b) == .Static
	{
		return -1;
	}
	handle_a := collidable_reference_raw_handle(pair.a);
	handle_b := collidable_reference_raw_handle(pair.b);
	location_a := narrow.bodies.handle_to_location.memory[handle_a];
	location_b := narrow.bodies.handle_to_location.memory[handle_b];
	if location_a.set_index == location_b.set_index
	{
		return -1;
	}
	set_index := location_a.set_index;
	if set_index == BODIES_ACTIVE_SET_INDEX
	{
		set_index = location_b.set_index;
	}
	if set_index <= BODIES_ACTIVE_SET_INDEX
	{
		return -1;
	}
	return set_index;
}
narrow_phase_write_fused_stored_description_trusted :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
	manifold: ^Collision_Stored_Manifold,
	header: ^Narrow_Phase_Transaction_Header,
	transaction: ^Narrow_Phase_Contact_Transaction,
)
{
	body_count := 2;
	transaction.body_handles[0] = {
		collidable_reference_raw_handle(pair.a),
	};
	if collidable_reference_mobility(pair.a) == .Static
	{
		body_count = 1;
		transaction.body_handles[0] = {
			collidable_reference_raw_handle(pair.b),
		};
	}
	else if collidable_reference_mobility(pair.b) == .Static
	{
		body_count = 1;
	}
	else
	{
		transaction.body_handles[1] = {
			collidable_reference_raw_handle(pair.b),
		};
	}
	contact_count := int(manifold.convex.count);
	type_id := contact_constraint_type_id(body_count, contact_count, .Convex);
	header.type_id = i16(type_id);
	header.body_count = u8(body_count);
	transaction.accessor = &narrow.accessors.records[type_id];
	for contact_index in 0 ..< MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		transaction.feature_ids[contact_index] = -1;
	}
	contacts := ([^]Constraint_Contact_Data)(&transaction.description[0]);
	for contact_index in 0 ..< contact_count
	{
		contact := manifold.convex.contacts[contact_index];
		contacts[contact_index] = {contact.offset, contact.depth};
		transaction.feature_ids[contact_index] = contact.feature_id;
	}
	offset := contact_count * size_of(Constraint_Contact_Data);
	if body_count == 2
	{
		(^util.Vector3)(rawptr(uintptr(&transaction.description[0]) + uintptr(offset)))^ =
			manifold.convex.offset_b;
		offset += size_of(util.Vector3);
	}
	(^util.Vector3)(rawptr(uintptr(&transaction.description[0]) + uintptr(offset)))^ =
		manifold.convex.normal;
	offset += size_of(util.Vector3);
	(^Contact_Material_Properties)(
		rawptr(uintptr(&transaction.description[0]) + uintptr(offset))
	)^ = narrow.stored_completion_material;
}
narrow_phase_prepare_fused_existing_mapping :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, mapping_index: int,
	header: ^Narrow_Phase_Transaction_Header,
	transaction: ^Narrow_Phase_Contact_Transaction,
) -> Physics_Status
{
	old_cache := narrow.pair_cache.mapping.values.memory[mapping_index];
	location, location_status := solver_resolve(
		narrow.solver, old_cache.constraint_handle,
	);
	if location_status != .Ok
	{
		return location_status;
	}
	batch := &narrow.solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	if type_batch_index < 0 || type_batch_index >= int(batch.type_batches.length)
	{
		return .Invalid_Argument;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	old_accessor, accessor_status := contact_constraint_accessor_lookup(
		&narrow.accessors, location.type_id,
	);
	if accessor_status != .Ok
	{
		return accessor_status;
	}
	transaction.old_cache = old_cache;
	header.pair_mapping_index = i32(mapping_index);
	transaction.old_impulses = contact_constraint_gather_impulses_trusted(
		type_batch, old_accessor, int(location.index_in_type_batch),
	);
	transaction.old_contact_count = old_accessor.contact_count;
	if location.type_id == i32(header.type_id)
	{
		transaction.prepared.update = {
			type_record=&narrow.solver.registry.records[int(header.type_id)],
			type_batch=type_batch,
			batch_index=location.batch_index,
			record_index=location.index_in_type_batch,
		};
		transaction.accessor = old_accessor;
		header.kind = .Update;
	}
	else
	{
		transaction.prepared.replace.solver_add.type_record =
			&narrow.solver.registry.records[int(header.type_id)];
		header.kind = .Replace;
	}
	return pair_cache_mark_fresh(&narrow.pair_cache, mapping_index);
}
narrow_phase_finalize_fused_prepared_transaction :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, header: ^Narrow_Phase_Transaction_Header,
	transaction: ^Narrow_Phase_Contact_Transaction,
) -> Physics_Status
{
	if header.kind != .Pending_Prepared
	{
		return .Invalid_Argument;
	}
	mapping_index := pair_cache_index_of(&narrow.pair_cache, header.pair);
	if mapping_index >= 0
	{
		return narrow_phase_prepare_fused_existing_mapping(
			narrow, mapping_index, header, transaction,
		);
	}
	transaction.old_cache = {};
	transaction.old_impulses = {};
	transaction.old_contact_count = 0;
	transaction.prepared.add.solver_add.type_record =
		&narrow.solver.registry.records[int(header.type_id)];
	header.pair_mapping_index = -1;
	header.kind = .Add;
	return .Ok;
}
narrow_phase_try_update_fused_builtin_convex_existing :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
	manifold: ^Convex_Contact_Manifold, mapping_index: int,
) -> Reference_State
{
	body_count := 2;
	if collidable_reference_mobility(pair.a) == .Static ||
		collidable_reference_mobility(pair.b) == .Static
	{
		body_count = 1;
	}
	switch manifold.count
	{
		case 1:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, manifold, narrow.stored_completion_material,
					mapping_index, 1, 1,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, manifold, narrow.stored_completion_material,
				mapping_index, 2, 1,
			);
		case 2:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, manifold, narrow.stored_completion_material,
					mapping_index, 1, 2,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, manifold, narrow.stored_completion_material,
				mapping_index, 2, 2,
			);
		case 3:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, manifold, narrow.stored_completion_material,
					mapping_index, 1, 3,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, manifold, narrow.stored_completion_material,
				mapping_index, 2, 3,
			);
		case 4:
			if body_count == 1
			{
				return narrow_phase_update_builtin_convex_existing(
					narrow, pair, manifold, narrow.stored_completion_material,
					mapping_index, 1, 4,
				);
			}
			return narrow_phase_update_builtin_convex_existing(
				narrow, pair, manifold, narrow.stored_completion_material,
				mapping_index, 2, 4,
			);
		case:
			return .Missing;
	}
}
narrow_phase_update_existing_pair_single :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair, result: ^Narrow_Phase_Pair_Result,
	mapping_index: int, header: ^Narrow_Phase_Transaction_Header,
	deferred: ^Narrow_Phase_Contact_Transaction,
) -> (Reference_State, Physics_Status)
{
	if narrow_phase_try_update_builtin_convex_existing(
		narrow, pair, result, mapping_index,
	) == .Present
	{
		return .Missing, .Ok;
	}
	return narrow_phase_prepare_existing_pair_transaction(
		narrow, pair, result, mapping_index, header, deferred,
	);
}
NARROW_PHASE_STALE_SCAN_MAXIMUM_WORKERS :: 4;
Narrow_Phase_Transaction_Prepare_Job :: struct
{
	narrow:             ^Narrow_Phase,
	transaction_count:  int,
	worker_count:       int,
	stale_worker_count: int,
	preflight_counts:   ^[MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Preflight_Worker_Counts,
}
narrow_phase_freshness_region :: #force_inline proc "contextless" (
	mapping_count, worker_count, worker_index: int,
) -> (start, end: int)
{
	wide_count := mapping_count >> 3;
	start = (wide_count * worker_index / worker_count) << 3;
	end = (wide_count * (worker_index + 1) / worker_count) << 3;
	if worker_index == worker_count - 1
	{
		end = mapping_count;
	}
	return;
}
narrow_phase_collect_stale_pair_indices :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, worker_count, worker_index: int,
) -> Physics_Status
{
	mapping_count := narrow.pair_cache.mapping.count;
	start, end := narrow_phase_freshness_region(
		mapping_count, worker_count, worker_index,
	);
	write_index := start;
	freshness := narrow.pair_cache.freshness_generation;
	freshness_pattern := u64(freshness) * u64(0x0101_0101_0101_0101);
	wide_end := end - (end & 7);
	for mapping_index := start; mapping_index < wide_end; mapping_index += 8
	{
		freshness_batch := (^u64)(&narrow.pair_cache.pair_freshness.memory[mapping_index])^;
		if freshness_batch == freshness_pattern
		{
			continue;
		}
		for lane in 0 ..< 8
		{
			index := mapping_index + lane;
			if narrow.pair_cache.pair_freshness.memory[index] == freshness
			{
				continue;
			}
			transaction := &narrow.remove_transactions.memory[index];
			transaction.constraint_handle =
				narrow.pair_cache.mapping.values.memory[index].constraint_handle;
			status := narrow_phase_prepare_remove_transaction(
				narrow, transaction.constraint_handle, &transaction.solver_remove,
			);
			if status != .Ok
			{
				return status;
			}
			narrow.remove_order.memory[write_index].record_index = i32(index);
			write_index += 1;
		}
	}
	for mapping_index := wide_end; mapping_index < end; mapping_index += 1
	{
		if narrow.pair_cache.pair_freshness.memory[mapping_index] == freshness
		{
			continue;
		}
		transaction := &narrow.remove_transactions.memory[mapping_index];
		transaction.constraint_handle =
			narrow.pair_cache.mapping.values.memory[mapping_index].constraint_handle;
		status := narrow_phase_prepare_remove_transaction(
			narrow, transaction.constraint_handle, &transaction.solver_remove,
		);
		if status != .Ok
		{
			return status;
		}
		narrow.remove_order.memory[write_index].record_index = i32(mapping_index);
		write_index += 1;
	}
	narrow.candidate_counts[worker_index] = i32(write_index - start);
	return .Ok;
}
narrow_phase_count_prepared_transaction :: #force_inline proc "contextless" (
	header: ^Narrow_Phase_Transaction_Header,
	counts: ^Narrow_Phase_Preflight_Worker_Counts,
) -> Physics_Status
{
	if header == nil || counts == nil
	{
		return .Invalid_Argument;
	}
	switch header.kind
	{
		case .None:
			return .Ok;
		case .Update:
			counts.updates += 1;
		case .Add:
			counts.adds += 1;
		case .Replace:
			counts.replacements += 1;
		case .Remove:
			counts.removals += 1;
		case .Pending, .Pending_Prepared:
			return .Invalid_Argument;
	}
	return .Ok;
}
narrow_phase_transaction_prepare_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Narrow_Phase_Transaction_Prepare_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	counts: Narrow_Phase_Preflight_Worker_Counts;
	start := job.transaction_count * worker_index / job.worker_count;
	end := job.transaction_count * (worker_index + 1) / job.worker_count;
	for order_index in start ..< end
	{
		transaction_index := int(job.narrow.transaction_order.memory[order_index]);
		header := &job.narrow.transaction_headers.memory[transaction_index];
		if header.kind == .Pending
		{
			payload_index := int(header.payload_index);
			if payload_index < 0 || payload_index >= int(job.narrow.contact_transactions.length)
			{
				status = .Capacity_Missing;
				break;
			}
			status = narrow_phase_prepare_pair_transaction(
				job.narrow, header, &job.narrow.contact_transactions.memory[payload_index],
			);
			if status != .Ok
			{
				break;
			}
		}
		else if header.kind == .Pending_Prepared
		{
			payload_index := int(header.payload_index);
			if payload_index < 0 || payload_index >= int(job.narrow.contact_transactions.length)
			{
				status = .Capacity_Missing;
				break;
			}
			status = narrow_phase_finalize_fused_prepared_transaction(
				job.narrow, header,
				&job.narrow.contact_transactions.memory[payload_index],
			);
			if status != .Ok
			{
				break;
			}
		}
		status = narrow_phase_count_prepared_transaction(header, &counts);
		if status != .Ok
		{
			break;
		}
	}
	if status == .Ok && worker_index < job.stale_worker_count
	{
		status = narrow_phase_collect_stale_pair_indices(
			job.narrow, job.stale_worker_count, worker_index,
		);
		if status == .Ok
		{
			stale_count := job.narrow.candidate_counts[worker_index];
			if counts.removals > max(i32) - stale_count
			{
				status = .Capacity_Missing;
			}
			else
			{
				counts.removals += stale_count;
			}
		}
	}
	job.preflight_counts[worker_index] = counts;
	job.narrow.transaction_statuses[worker_index] = status;
}
narrow_phase_preflight_summary :: #force_inline proc "contextless" (
	counts: ^[MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Preflight_Worker_Counts,
	worker_count: int,
) -> (Narrow_Phase_Preflight_Summary, Physics_Status)
{
	summary := Narrow_Phase_Preflight_Summary{worker_counts_state=.Present};
	for worker_index in 0 ..< worker_count
	{
		worker := counts[worker_index];
		new_constraints := int(worker.adds) + int(worker.replacements);
		removals := int(worker.removals) + int(worker.replacements);
		writes := int(worker.updates) + new_constraints;
		if summary.new_constraint_count > max(int) - new_constraints ||
			summary.pair_add_count > max(int) - int(worker.adds) ||
			summary.remove_count > max(int) - removals ||
			summary.write_count > max(int) - writes
		{
			return {}, .Capacity_Missing;
		}
		summary.new_constraint_count += new_constraints;
		summary.pair_add_count += int(worker.adds);
		summary.remove_count += removals;
		summary.write_count += writes;
	}
	return summary, .Ok;
}
narrow_phase_prepare_transactions :: proc (
	narrow: ^Narrow_Phase, current_transaction_count, append_transaction_index,
	worker_count: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
	preflight_counts: ^[MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Preflight_Worker_Counts,
) -> (int, Narrow_Phase_Preflight_Summary, Physics_Status)
{
	if current_transaction_count < 0 || append_transaction_index < 0 ||
		current_transaction_count > int(narrow.transaction_order.length) ||
		append_transaction_index > int(narrow.transaction_headers.length) ||
		narrow.pair_cache.mapping.count > int(narrow.remove_transactions.length) ||
		narrow.pair_cache.mapping.count > int(narrow.remove_order.length) ||
		preflight_counts == nil
	{
		return 0, {}, .Capacity_Missing;
	}
	for worker_index in 0 ..< worker_count
	{
		narrow.candidate_counts[worker_index] = 0;
		preflight_counts[worker_index] = {};
	}
	stale_worker_count := min(
		worker_count, NARROW_PHASE_STALE_SCAN_MAXIMUM_WORKERS,
	);
	if dispatcher != nil && worker_count > 1
	{
		job := Narrow_Phase_Transaction_Prepare_Job{
			narrow=narrow,
			transaction_count=current_transaction_count,
			worker_count=worker_count,
			stale_worker_count=stale_worker_count,
			preflight_counts=preflight_counts,
		};
		dispatch_status := dispatcher.dispatch(dispatcher, narrow_phase_transaction_prepare_worker, worker_count, &job);
		if dispatch_status != .Ok
		{
			return 0, {}, .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if narrow.transaction_statuses[worker_index] != .Ok
			{
				return 0, {}, narrow.transaction_statuses[worker_index];
			}
		}
	}
	else
	{
		for order_index in 0 ..< current_transaction_count
		{
			transaction_index := int(narrow.transaction_order.memory[order_index]);
			header := &narrow.transaction_headers.memory[transaction_index];
			if header.kind != .Pending && header.kind != .Pending_Prepared
			{
				continue;
			}
			payload_index := int(header.payload_index);
			if payload_index < 0 || payload_index >= int(narrow.contact_transactions.length)
			{
				return 0, {}, .Capacity_Missing;
			}
			status := Physics_Status.Ok;
			if header.kind == .Pending
			{
				status = narrow_phase_prepare_pair_transaction(
					narrow, header, &narrow.contact_transactions.memory[payload_index],
				);
			}
			else
			{
				status = narrow_phase_finalize_fused_prepared_transaction(
					narrow, header, &narrow.contact_transactions.memory[payload_index],
				);
			}
			if status != .Ok
			{
				return 0, {}, status;
			}
		}
		status := narrow_phase_collect_stale_pair_indices(narrow, 1, 0);
		if status != .Ok
		{
			return 0, {}, status;
		}
	}
	stale_count := 0;
	for worker_index in 0 ..< stale_worker_count
	{
		stale_count += int(narrow.candidate_counts[worker_index]);
	}
	if append_transaction_index + stale_count > int(narrow.transaction_headers.length) ||
		current_transaction_count + stale_count > int(narrow.transaction_order.length)
	{
		return 0, {}, .Capacity_Missing;
	}
	transaction_count := current_transaction_count;
	transaction_index := append_transaction_index;
	for worker_index in 0 ..< stale_worker_count
	{
		region_start, _ := narrow_phase_freshness_region(
			narrow.pair_cache.mapping.count, stale_worker_count, worker_index,
		);
		worker_stale_count := int(narrow.candidate_counts[worker_index]);
		for stale_index in 0 ..< worker_stale_count
		{
			payload_index := region_start + stale_index;
			mapping_index := int(
				narrow.remove_order.memory[payload_index].record_index,
			);
			narrow.transaction_headers.memory[transaction_index] = {
				pair=narrow.pair_cache.mapping.keys.memory[mapping_index],
				payload_index=i32(mapping_index),
				pair_mapping_index=i32(mapping_index),
				kind=.Remove,
			};
			narrow.transaction_order.memory[transaction_count] = i32(transaction_index);
			transaction_index += 1;
			transaction_count += 1;
		}
	}
	summary: Narrow_Phase_Preflight_Summary;
	if dispatcher != nil && worker_count > 1
	{
		summary_status: Physics_Status;
		summary, summary_status = narrow_phase_preflight_summary(preflight_counts, worker_count);
		if summary_status != .Ok
		{
			return 0, {}, summary_status;
		}
	}
	return transaction_count, summary, .Ok;
}
narrow_phase_prepare_remove_transaction :: proc "contextless" (
	narrow: ^Narrow_Phase, constraint_handle: Constraint_Handle,
	prepared: ^Solver_Constraint_Remove_Transaction,
) -> Physics_Status
{
	if prepared == nil
	{
		return .Invalid_Argument;
	}
	location, location_status := solver_resolve(narrow.solver, constraint_handle);
	if location_status != .Ok
	{
		return location_status;
	}
	batch := &narrow.solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	type_batch := &batch.type_batches.memory[type_batch_index];
	reference, reference_status := type_batch_read_reference(
		type_batch, narrow.bodies, int(location.index_in_type_batch),
	);
	if reference_status != .Ok
	{
		return reference_status;
	}
	type_record, type_status := constraint_type_registry_lookup(&narrow.solver.registry, location.type_id);
	if type_status != .Ok
	{
		return type_status;
	}
	prepared.type_record = type_record;
	prepared.batch = batch;
	prepared.type_batch = type_batch;
	prepared.reference = reference;
	prepared.batch_index = location.batch_index;
	prepared.type_batch_index = i32(type_batch_index);
	prepared.record_index = location.index_in_type_batch;
	for body_index in 0 ..< int(reference.body_count)
	{
		body_location, body_status := bodies_resolve(narrow.bodies, reference.body_handles[body_index]);
		if body_status != .Ok || body_location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return .Invalid_Argument;
		}
		prepared.body_lists[body_index] =
			&narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[body_location.index];
	}
	return .Ok;
}
narrow_phase_batch_accepts_transaction :: proc "contextless" (
	narrow: ^Narrow_Phase, batch_index: int, reference: ^Constraint_Reference,
) -> Reference_State
{
	batch := &narrow.solver.active_set.batches.memory[batch_index];
	claim_word_count := max(util.index_set_bundle_capacity(int(narrow.body_claims.length)), 1);
	claim_start := batch_index * claim_word_count;
	for body_index in 0 ..< int(reference.body_count)
	{
		if u32(reference.encoded_body_references[body_index]) >= BODY_REFERENCE_KINEMATIC_MASK
		{
			continue;
		}
		handle := reference.body_handles[body_index];
		if batch.body_reference_counts.memory[handle.value] != 0
		{
			return .Missing;
		}
		word := &narrow.body_batch_claims.memory[
			claim_start + (int(handle.value) >> util.INDEX_SET_SHIFT)
		];
		mask := u64(1) << uint(int(handle.value) & util.INDEX_SET_MASK);
		if word.generation == narrow.claim_generation && (word.bits & mask) != 0
		{
			return .Missing;
		}
	}
	return .Present;
}
narrow_phase_prepare_add_transaction :: proc "contextless" (
	narrow: ^Narrow_Phase, header: ^Narrow_Phase_Transaction_Header,
	transaction: ^Narrow_Phase_Contact_Transaction,
	prepared: ^Solver_Constraint_Add_Transaction, handle: Constraint_Handle,
) -> Physics_Status
{
	if header == nil || transaction == nil || prepared == nil
	{
		return .Invalid_Argument;
	}
	reference := Constraint_Reference{
		handle=handle,
		body_handles=transaction.body_handles,
		body_count=i32(header.body_count),
	};
	active_set := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< int(reference.body_count)
	{
		body_handle := reference.body_handles[body_index];
		body_location := narrow.bodies.handle_to_location.memory[body_handle.value];
		encoded := u32(body_location.index);
		inertia := active_set.dynamics_state.memory[body_location.index].inertia.local;
		if body_inertia_mobility(inertia) == .Kinematic
		{
			encoded |= BODY_REFERENCE_KINEMATIC_MASK;
		}
		reference.encoded_body_references[body_index] = i32(encoded);
		prepared.body_lists[body_index] = &active_set.constraints.memory[body_location.index];
	}
	batch_index := int(narrow.solver.fallback_batch_index);
	for candidate_index in 0 ..< int(narrow.solver.fallback_batch_index)
	{
		if narrow_phase_batch_accepts_transaction(narrow, candidate_index, &reference) == .Present
		{
			batch_index = candidate_index;
			break;
		}
	}
	if batch_index < int(narrow.solver.fallback_batch_index)
	{
		claim_word_count := max(util.index_set_bundle_capacity(int(narrow.body_claims.length)), 1);
		claim_start := batch_index * claim_word_count;
		for body_index in 0 ..< int(reference.body_count)
		{
			if u32(reference.encoded_body_references[body_index]) >= BODY_REFERENCE_KINEMATIC_MASK
			{
				continue;
			}
			body_handle := reference.body_handles[body_index];
			word := &narrow.body_batch_claims.memory[
				claim_start + (int(body_handle.value) >> util.INDEX_SET_SHIFT)
			];
			if word.generation != narrow.claim_generation
			{
				word^ = {generation=narrow.claim_generation};
			}
			word.bits |= u64(1) << uint(int(body_handle.value) & util.INDEX_SET_MASK);
		}
	}
	batch := &narrow.solver.active_set.batches.memory[batch_index];
	type_id := int(header.type_id);
	type_batch_index := int(batch.type_id_to_batch_index[type_id]);
	type_batch := &batch.type_batches.memory[type_batch_index];
	type_claim_index := batch_index * CONSTRAINT_TYPE_ID_CAPACITY + type_id;
	type_claim := &narrow.type_claims.memory[type_claim_index];
	if type_claim.generation != narrow.claim_generation
	{
		touched_index := int(narrow.body_claims.length) + narrow.touched_type_count;
		type_claim^ = {generation=narrow.claim_generation};
		narrow.touched_claim_indices.memory[touched_index] = i32(type_claim_index);
		narrow.touched_type_count += 1;
	}
	record_index := int(type_batch.count) + int(type_claim.count);
	type_claim.count += 1;
	prepared.batch = batch;
	prepared.type_batch = type_batch;
	prepared.reference = reference;
	prepared.batch_index = i32(batch_index);
	prepared.type_batch_index = i32(type_batch_index);
	prepared.record_index = i32(record_index);
	for body_index in 0 ..< int(reference.body_count)
	{
		body_handle := reference.body_handles[body_index];
		list := prepared.body_lists[body_index];
		claim := &narrow.body_claims.memory[body_handle.value];
		if claim.generation != narrow.claim_generation
		{
			claim^ = {generation=narrow.claim_generation};
			narrow.touched_claim_indices.memory[narrow.touched_body_count] = body_handle.value;
			narrow.touched_body_count += 1;
		}
		prepared.body_list_indices[body_index] = i32(list.count + int(claim.list_count));
		claim.list_count += 1;
	}
	return .Ok;
}
narrow_phase_prepare_pair_add_transaction :: proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair,
	pair_add: ^Pair_Cache_Add_Transaction, claimed_pair_count: ^int,
) -> Physics_Status
{
	if pair_add == nil
	{
		return .Invalid_Argument;
	}
	mapping_index := narrow.pair_cache.mapping.count + claimed_pair_count^;
	if mapping_index >= int(narrow.pair_cache.mapping.keys.length)
	{
		return .Capacity_Missing;
	}
	pair_value := pair;
	table_index := int(u32(util.hash_rehash(collidable_pair_hash(&pair_value)))) &
		narrow.pair_cache.mapping.table_mask;
	for probe_count in 0 ..< int(narrow.pair_cache.mapping.table.length)
	{
		occupied := Reference_State.Missing;
		if narrow.pair_cache.mapping.table.memory[table_index] != 0 ||
			narrow.pair_table_claim_generations.memory[table_index] == narrow.claim_generation
		{
			occupied = .Present;
		}
		if occupied == .Missing
		{
			pair_add^ = {i32(mapping_index), i32(table_index)};
			narrow.pair_table_claim_generations.memory[table_index] = narrow.claim_generation;
			claimed_pair_count^ += 1;
			return .Ok;
		}
		table_index = (table_index + 1) & narrow.pair_cache.mapping.table_mask;
		_ = probe_count;
	}
	return .Capacity_Missing;
}
narrow_phase_body_list_target_capacity :: proc "contextless" (
	list: ^util.Quick_List(Body_Constraint_Reference), claimed_count: int,
) -> (int, Physics_Status)
{
	if list == nil || list.span.memory == nil || claimed_count <= 0 ||
		list.count < 0 || list.count > int(list.span.length)
	{
		return 0, .Invalid_Argument;
	}
	if list.count > max(int) - claimed_count
	{
		return 0, .Capacity_Missing;
	}
	required_count := list.count + claimed_count;
	if required_count <= int(list.span.length)
	{
		return int(list.span.length), .Ok;
	}
	if list.count > max(int) / 2
	{
		return 0, .Capacity_Missing;
	}
	return max(max(list.count * 2, 1), required_count), .Ok;
}
narrow_phase_validate_constraint_storage_capacity :: proc (
	narrow: ^Narrow_Phase,
) -> Physics_Status
{
	requirements: [util.BUFFER_POOL_POWER_COUNT]int;
	type_claim_offset := int(narrow.body_claims.length);
	for touched_index in 0 ..< narrow.touched_type_count
	{
		claim_index := int(narrow.touched_claim_indices.memory[type_claim_offset + touched_index]);
		batch_index := claim_index / CONSTRAINT_TYPE_ID_CAPACITY;
		type_id := claim_index - batch_index * CONSTRAINT_TYPE_ID_CAPACITY;
		claim := &narrow.type_claims.memory[claim_index];
		batch := &narrow.solver.active_set.batches.memory[batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[type_id]);
		if type_batch_index < 0
		{
			return .Capacity_Missing;
		}
		type_batch := &batch.type_batches.memory[type_batch_index];
		if int(type_batch.count) > max(int) - int(claim.count)
		{
			return .Capacity_Missing;
		}
		target, target_status := type_batch_target_capacity(
			type_batch, int(type_batch.count) + int(claim.count),
		);
		if target_status != .Ok
		{
			return target_status;
		}
		requirement_status := type_batch_accumulate_resize_slot_requirements(
			type_batch, target, &requirements,
		);
		if requirement_status != .Ok
		{
			return requirement_status;
		}
	}
	for touched_index in 0 ..< narrow.touched_body_count
	{
		handle_index := int(narrow.touched_claim_indices.memory[touched_index]);
		claim := &narrow.body_claims.memory[handle_index];
		location := narrow.bodies.handle_to_location.memory[handle_index];
		list := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[location.index];
		target, target_status := narrow_phase_body_list_target_capacity(
			list, int(claim.list_count),
		);
		if target_status != .Ok
		{
			return target_status;
		}
		if target <= int(list.span.length)
		{
			continue;
		}
		power, power_status := util.buffer_pool_power_for_count(
			Body_Constraint_Reference, target,
		);
		if power_status != .Ok
		{
			return physics_memory_status(power_status);
		}
		if requirements[power] == max(int)
		{
			return .Capacity_Missing;
		}
		requirements[power] += 1;
	}
	for power in 0 ..< len(requirements)
	{
		if requirements[power] == 0
		{
			continue;
		}
		ensure_status := util.buffer_pool_ensure_available_slot_count(
			narrow.solver.pool, power, requirements[power],
		);
		if ensure_status != .Ok
		{
			return physics_memory_status(ensure_status);
		}
	}
	return .Ok;
}
narrow_phase_refresh_add_destinations :: proc "contextless" (
	narrow: ^Narrow_Phase, prepared: ^Solver_Constraint_Add_Transaction,
) -> Physics_Status
{
	if prepared == nil
	{
		return .Invalid_Argument;
	}
	batch_index := int(prepared.batch_index);
	type_batch_index := int(prepared.type_batch_index);
	if batch_index < 0 || batch_index >= int(narrow.solver.active_set.batches.length)
	{
		return .Invalid_Argument;
	}
	batch := &narrow.solver.active_set.batches.memory[batch_index];
	if type_batch_index < 0 || type_batch_index >= int(batch.type_batches.length)
	{
		return .Invalid_Argument;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	if prepared.record_index < 0 ||
		int(prepared.record_index) >= int(type_batch.index_to_handle.length)
	{
		return .Capacity_Missing;
	}
	prepared.batch = batch;
	prepared.type_batch = type_batch;
	for body_index in 0 ..< int(prepared.reference.body_count)
	{
		location, location_status := bodies_resolve(
			narrow.bodies, prepared.reference.body_handles[body_index],
		);
		if location_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return .Invalid_Argument;
		}
		list := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[location.index];
		if prepared.body_list_indices[body_index] < 0 ||
			int(prepared.body_list_indices[body_index]) >= int(list.span.length)
		{
			return .Capacity_Missing;
		}
		prepared.body_lists[body_index] = list;
	}
	return .Ok;
}
narrow_phase_apply_constraint_storage_capacity :: proc (
	narrow: ^Narrow_Phase, transaction_count: int,
) -> Physics_Status
{
	type_claim_offset := int(narrow.body_claims.length);
	for touched_index in 0 ..< narrow.touched_type_count
	{
		claim_index := int(narrow.touched_claim_indices.memory[type_claim_offset + touched_index]);
		batch_index := claim_index / CONSTRAINT_TYPE_ID_CAPACITY;
		type_id := claim_index - batch_index * CONSTRAINT_TYPE_ID_CAPACITY;
		claim := &narrow.type_claims.memory[claim_index];
		batch := &narrow.solver.active_set.batches.memory[batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[type_id]);
		if type_batch_index < 0
		{
			return .Capacity_Missing;
		}
		type_batch := &batch.type_batches.memory[type_batch_index];
		target, target_status := type_batch_target_capacity(
			type_batch, int(type_batch.count) + int(claim.count),
		);
		if target_status != .Ok
		{
			return target_status;
		}
		resize_status := type_batch_ensure_capacity(
			type_batch, target, narrow.solver.pool,
		);
		if resize_status != .Ok
		{
			return resize_status;
		}
	}
	for touched_index in 0 ..< narrow.touched_body_count
	{
		handle_index := int(narrow.touched_claim_indices.memory[touched_index]);
		claim := &narrow.body_claims.memory[handle_index];
		location := narrow.bodies.handle_to_location.memory[handle_index];
		list := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[location.index];
		target, target_status := narrow_phase_body_list_target_capacity(
			list, int(claim.list_count),
		);
		if target_status != .Ok
		{
			return target_status;
		}
		resize_status := util.quick_list_ensure_capacity(
			list, target, narrow.solver.pool,
		);
		if resize_status != .Ok
		{
			return physics_collection_status(resize_status);
		}
	}
	for order_index in 0 ..< transaction_count
	{
		transaction_index := int(narrow.transaction_order.memory[order_index]);
		header := &narrow.transaction_headers.memory[transaction_index];
		if header.kind != .Add && header.kind != .Replace
		{
			continue;
		}
		contact := &narrow.contact_transactions.memory[header.payload_index];
		prepared := &contact.prepared.add.solver_add;
		if header.kind == .Replace
		{
			prepared = &contact.prepared.replace.solver_add;
		}
		refresh_status := narrow_phase_refresh_add_destinations(narrow, prepared);
		if refresh_status != .Ok
		{
			return refresh_status;
		}
	}
	return .Ok;
}
narrow_phase_transaction_worker_index :: #force_inline proc "contextless" (
	batch_index, type_id, record_index, worker_count: int,
) -> int
{
	if worker_count <= 1
	{
		return 0;
	}
	bundle_index := record_index / util.PRODUCTION_LANE_COUNT;
	hash := u32(i32(batch_index) * 19349663 ~ i32(type_id) * 73856093 ~ i32(bundle_index) * 83492791);
	return int(hash % u32(worker_count));
}
narrow_phase_type_batch_worker_index :: #force_inline proc "contextless" (
	batch_index, type_id, worker_count: int, fallback_batch_index: i32,
) -> int
{
	if worker_count <= 1 || i32(batch_index) == fallback_batch_index
	{
		return 0;
	}
	hash := u32(i32(batch_index) * 19349663 ~ i32(type_id) * 73856093);
	return int(hash % u32(worker_count));
}
NARROW_PHASE_PARALLEL_REMOVE_MINIMUM :: 32;
NARROW_PHASE_PARALLEL_REMOVE_MAXIMUM_WORKERS :: 8;
narrow_phase_parallel_remove_worker_count :: #force_inline proc "contextless" (
	worker_count, remove_count: int,
) -> int
{
	if worker_count <= 1 || remove_count < NARROW_PHASE_PARALLEL_REMOVE_MINIMUM
	{
		return 0;
	}
	return min(worker_count, NARROW_PHASE_PARALLEL_REMOVE_MAXIMUM_WORKERS);
}
narrow_phase_advance_claim_generation :: proc "contextless" (
	narrow: ^Narrow_Phase,
) -> Physics_Status
{
	context = runtime.default_context();
	if narrow.claim_generation == max(u32)
	{
		status := util.buffer_clear(narrow.body_claims, 0, int(narrow.body_claims.length));
		if status != .Ok
		{
			return physics_memory_status(status);
		}
		status = util.buffer_clear(narrow.body_batch_claims, 0, int(narrow.body_batch_claims.length));
		if status != .Ok
		{
			return physics_memory_status(status);
		}
		status = util.buffer_clear(narrow.type_claims, 0, int(narrow.type_claims.length));
		if status != .Ok
		{
			return physics_memory_status(status);
		}
		status = util.buffer_clear(
			narrow.pair_table_claim_generations, 0, int(narrow.pair_table_claim_generations.length),
		);
		if status != .Ok
		{
			return physics_memory_status(status);
		}
		narrow.claim_generation = 1;
	}
	else
	{
		narrow.claim_generation += 1;
	}
	return .Ok;
}
narrow_phase_preflight_transactions :: proc (
	narrow: ^Narrow_Phase, transaction_count, worker_count: int,
	summary: Narrow_Phase_Preflight_Summary,
) -> Physics_Status
{
	generation_status := narrow_phase_advance_claim_generation(narrow);
	if generation_status != .Ok
	{
		return generation_status;
	}
	narrow.touched_body_count = 0;
	narrow.touched_type_count = 0;
	narrow.prepared_pair_add_count = 0;
	narrow.remove_handle_return_start = 0;
	new_constraint_count := summary.new_constraint_count;
	pair_add_count := summary.pair_add_count;
	if summary.worker_counts_state != .Present
	{
		for order_index in 0 ..< transaction_count
		{
			transaction_index := int(narrow.transaction_order.memory[order_index]);
			header := &narrow.transaction_headers.memory[transaction_index];
			if header.kind == .Add || header.kind == .Replace
			{
				if new_constraint_count == max(int)
				{
					return .Capacity_Missing;
				}
				new_constraint_count += 1;
			}
			if header.kind == .Add
			{
				if pair_add_count == max(int)
				{
					return .Capacity_Missing;
				}
				pair_add_count += 1;
			}
		}
	}
	if int(narrow.solver.handle_pool.next_index) > max(int) - new_constraint_count ||
		narrow.pair_cache.mapping.count > max(int) - pair_add_count
	{
		return .Capacity_Missing;
	}
	target_constraint_capacity :=
		int(narrow.solver.handle_pool.next_index) + new_constraint_count;
	if target_constraint_capacity > int(narrow.solver.handle_to_constraint.length)
	{
		capacity_status := solver_ensure_handle_capacity(
			narrow.solver, max(target_constraint_capacity, 1),
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
	}
	target_pair_capacity := narrow.pair_cache.mapping.count + pair_add_count;
	capacity_status := pair_cache_ensure_active_capacity(
		&narrow.pair_cache, max(target_pair_capacity, 1),
		max(target_constraint_capacity, 1),
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	capacity_status = physics_ensure_zeroed_buffer_capacity(
		narrow.pool, &narrow.pair_table_claim_generations,
		max(
		int(narrow.pair_cache.mapping.table.length),
		int(narrow.bodies.sets.length),
	),
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	available_cursor := narrow.solver.handle_pool.available_id_count;
	next_handle := narrow.solver.handle_pool.next_index;
	remove_count := 0;
	claimed_pair_count := 0;
	for worker_index in 0 ..< worker_count
	{
		narrow.transaction_ranges[worker_index] = {};
	}
	for order_index in 0 ..< transaction_count
	{
		transaction_index := int(narrow.transaction_order.memory[order_index]);
		header := &narrow.transaction_headers.memory[transaction_index];
		switch header.kind
		{
			case .Update:
				payload_index := int(header.payload_index);
				if payload_index < 0 || payload_index >= int(narrow.contact_transactions.length)
				{
					return .Capacity_Missing;
				}
				transaction := &narrow.contact_transactions.memory[payload_index];
				update := &transaction.prepared.update;
				if summary.worker_counts_state != .Present
				{
					location, location_status := solver_resolve(
						narrow.solver, transaction.old_cache.constraint_handle,
					);
					if location_status != .Ok || location.type_id != i32(header.type_id)
					{
						return .Invalid_Argument;
					}
					batch := &narrow.solver.active_set.batches.memory[location.batch_index];
					type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
					type_record, type_status := constraint_type_registry_lookup(
						&narrow.solver.registry, location.type_id,
					);
					if type_status != .Ok
					{
						return type_status;
					}
					accessor, accessor_status := contact_constraint_accessor_lookup(
						&narrow.accessors, location.type_id,
					);
					if accessor_status != .Ok
					{
						return accessor_status;
					}
					update^ = {
						type_record, type_batch, location.batch_index, location.index_in_type_batch,
					};
					transaction.accessor = accessor;
				}
				route := narrow_phase_transaction_worker_index(
					int(update.batch_index), int(header.type_id),
					int(update.record_index), worker_count,
				);
				header.worker_index = i32(route);
				narrow.transaction_ranges[route].count += 1;
			case .Add, .Replace:
				payload_index := int(header.payload_index);
				if payload_index < 0 || payload_index >= int(narrow.contact_transactions.length)
				{
					return .Capacity_Missing;
				}
				transaction := &narrow.contact_transactions.memory[payload_index];
				solver_add := &transaction.prepared.add.solver_add;
				if header.kind == .Replace
				{
					solver_add = &transaction.prepared.replace.solver_add;
				}
				handle_value := next_handle;
				if available_cursor > 0
				{
					available_cursor -= 1;
					handle_value = narrow.solver.handle_pool.available_ids.memory[available_cursor];
				}
				else
				{
					next_handle += 1;
				}
				if handle_value < 0 || int(handle_value) >= int(narrow.solver.handle_to_constraint.length) ||
					int(handle_value) >= int(narrow.pair_cache.constraint_handle_to_pair.length)
				{
					return .Capacity_Missing;
				}
				status := narrow_phase_prepare_add_transaction(
					narrow, header, transaction, solver_add, {handle_value},
				);
				if status != .Ok
				{
					return status;
				}
				if header.kind == .Add
				{
					status = narrow_phase_prepare_pair_add_transaction(
						narrow, header.pair, &transaction.prepared.add.pair_add,
						&claimed_pair_count,
					);
					if status != .Ok
					{
						return status;
					}
				}
				else
				{
					solver_remove := &transaction.prepared.replace.solver_remove;
					status = narrow_phase_prepare_remove_transaction(
						narrow, transaction.old_cache.constraint_handle, solver_remove,
					);
					if status != .Ok
					{
						return status;
					}
					if remove_count >= int(narrow.remove_order.length)
					{
						return .Capacity_Missing;
					}
					narrow.remove_order.memory[remove_count] = {
						key=solver_remove_order_key(solver_remove),
						record_index=i32(transaction_index),
					};
					remove_count += 1;
				}
				if worker_count > 1
				{
					route := 0;
					if solver_add.batch_index != narrow.solver.fallback_batch_index
					{
						route = narrow_phase_transaction_worker_index(
							int(solver_add.batch_index), int(header.type_id),
							int(solver_add.record_index), worker_count,
						);
					}
					header.worker_index = i32(route);
					narrow.transaction_ranges[route].count += 1;
				}
			case .Remove:
				payload_index := int(header.payload_index);
				if payload_index < 0 || payload_index >= int(narrow.remove_transactions.length)
				{
					return .Capacity_Missing;
				}
				transaction := &narrow.remove_transactions.memory[payload_index];
				if remove_count >= int(narrow.remove_order.length)
				{
					return .Capacity_Missing;
				}
				narrow.remove_order.memory[remove_count] = {
					key=solver_remove_order_key(&transaction.solver_remove),
					record_index=i32(transaction_index),
				};
				remove_count += 1;
			case .None, .Pending, .Pending_Prepared:
				return .Invalid_Argument;
		}
	}
	if summary.worker_counts_state == .Present
	{
		if remove_count != summary.remove_count || claimed_pair_count != summary.pair_add_count
		{
			return .Invalid_Argument;
		}
	}
	else if claimed_pair_count != pair_add_count
	{
		return .Invalid_Argument;
	}
	if available_cursor + remove_count > int(narrow.solver.handle_pool.available_ids.length)
	{
		return .Capacity_Missing;
	}
	capacity_status = narrow_phase_validate_constraint_storage_capacity(narrow);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	capacity_status = narrow_phase_apply_constraint_storage_capacity(
		narrow, transaction_count,
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	sort_status := solver_sort_remove_order_descending(
		narrow.remove_order, narrow.remove_sort_scratch, remove_count,
	);
	if sort_status != .Ok
	{
		return sort_status;
	}
	parallel_remove_worker_count := narrow_phase_parallel_remove_worker_count(
		worker_count, remove_count,
	);
	parallel_remove_compaction := Reference_State.Missing;
	if parallel_remove_worker_count > 0
	{
		write_transaction_count := 0;
		for worker_index in 0 ..< worker_count
		{
			write_transaction_count += int(narrow.transaction_ranges[worker_index].count);
		}
		if write_transaction_count > 0
		{
			parallel_remove_compaction = .Present;
		}
	}
	if parallel_remove_compaction == .Present
	{
		narrow.remove_handle_return_start = available_cursor;
		for worker_index in 0 ..< worker_count
		{
			narrow.transaction_ranges[worker_index] = {};
		}
		for order_index in 0 ..< transaction_count
		{
			transaction_index := int(narrow.transaction_order.memory[order_index]);
			header := &narrow.transaction_headers.memory[transaction_index];
			route := -1;
			switch header.kind
			{
				case .Update:
					update := &narrow.contact_transactions.memory[header.payload_index].prepared.update;
					route = narrow_phase_type_batch_worker_index(
						int(update.batch_index), int(header.type_id), parallel_remove_worker_count,
						narrow.solver.fallback_batch_index,
					);
				case .Add, .Replace:
					transaction := &narrow.contact_transactions.memory[header.payload_index];
					solver_add := &transaction.prepared.add.solver_add;
					if header.kind == .Replace
					{
						solver_add = &transaction.prepared.replace.solver_add;
					}
					route = narrow_phase_type_batch_worker_index(
						int(solver_add.batch_index), int(header.type_id), parallel_remove_worker_count,
						narrow.solver.fallback_batch_index,
					);
				case .None, .Pending, .Pending_Prepared, .Remove:
					continue;
			}
			header.worker_index = i32(route);
			narrow.transaction_ranges[route].count += 1;
		}
	}
	offset := 0;
	for worker_index in 0 ..< worker_count
	{
		narrow.transaction_ranges[worker_index].start = i32(offset);
		offset += int(narrow.transaction_ranges[worker_index].count);
	}
	cursors: [MAXIMUM_SOLVER_WORKER_COUNT]int;
	for worker_index in 0 ..< worker_count
	{
		cursors[worker_index] = int(narrow.transaction_ranges[worker_index].start);
	}
	for order_index in 0 ..< transaction_count
	{
		transaction_index := int(narrow.transaction_order.memory[order_index]);
		header := &narrow.transaction_headers.memory[transaction_index];
		if header.kind != .Update &&
			!(worker_count > 1 && (header.kind == .Add || header.kind == .Replace))
		{
			continue;
		}
		worker_index := int(header.worker_index);
		narrow.remove_sort_scratch.memory[cursors[worker_index]] = {
			record_index=i32(transaction_index),
		};
		cursors[worker_index] += 1;
	}
	narrow.remove_count = remove_count;
	narrow.prepared_pair_add_count = claimed_pair_count;
	return .Ok;
}
Narrow_Phase_Transaction_Commit_Job :: struct
{
	narrow: ^Narrow_Phase,
}
Narrow_Phase_Parallel_Remove_Commit_Job :: struct
{
	narrow:                ^Narrow_Phase,
	worker_count:          int,
	returned_handle_start: int,
}
narrow_phase_commit_write_range :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, worker_index: int,
)
{
	range := narrow.transaction_ranges[worker_index];
	for range_index in 0 ..< int(range.count)
	{
		transaction_index := narrow.remove_sort_scratch.memory[
			int(range.start) + range_index
		].record_index;
		header := &narrow.transaction_headers.memory[transaction_index];
		transaction := &narrow.contact_transactions.memory[header.payload_index];
		switch header.kind
		{
			case .Update:
				update := &transaction.prepared.update;
				solver_commit_update_transaction(update, &transaction.description[0]);
				contact_constraint_scatter_impulses_trusted(
					update.type_batch, transaction.accessor,
					int(update.record_index), transaction.feature_ids,
					transaction.old_cache.feature_ids, transaction.old_impulses,
					int(transaction.old_contact_count),
				);
				pair_cache_commit_update(
					&narrow.pair_cache, int(header.pair_mapping_index), header.pair,
					{transaction.old_cache.constraint_handle, transaction.feature_ids},
				);
			case .Add, .Replace:
				solver_add := &transaction.prepared.add.solver_add;
				if header.kind == .Replace
				{
					solver_add = &transaction.prepared.replace.solver_add;
				}
				solver_write_add_transaction_trusted(
					narrow.solver, solver_add, &transaction.description[0],
				);
				contact_constraint_scatter_impulses_trusted(
					solver_add.type_batch, transaction.accessor,
					int(solver_add.record_index), transaction.feature_ids,
					transaction.old_cache.feature_ids, transaction.old_impulses,
					int(transaction.old_contact_count),
				);
				cache := Constraint_Cache{
					solver_add.reference.handle, transaction.feature_ids,
				};
				if header.kind == .Add
				{
					pair_cache_write_add_reserved(
						&narrow.pair_cache, transaction.prepared.add.pair_add,
						header.pair, cache,
					);
				}
				else
				{
					pair_cache_commit_update(
						&narrow.pair_cache, int(header.pair_mapping_index),
						header.pair, cache,
					);
				}
			case .None, .Pending, .Pending_Prepared, .Remove:
		}
	}
}
narrow_phase_remove_transaction :: #force_inline proc "contextless" (
	narrow: ^Narrow_Phase, remove_index: int,
) -> ^Solver_Constraint_Remove_Transaction
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Narrow_Phase phase=constraint_finalize reason=checked_transaction_reference lifetime=until_transaction_buffers_mutate_or_phase_end
	transaction_index := int(narrow.remove_order.memory[remove_index].record_index);
	header := &narrow.transaction_headers.memory[transaction_index];
	if header.kind == .Replace
	{
		return &narrow.contact_transactions.memory[
			header.payload_index
		].prepared.replace.solver_remove;
	}
	return &narrow.remove_transactions.memory[header.payload_index].solver_remove;
}
narrow_phase_finalize_type_adds_worker :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, worker_count: int,
)
{
	type_claim_offset := int(narrow.body_claims.length);
	for touched_index in 0 ..< narrow.touched_type_count
	{
		claim_index := int(narrow.touched_claim_indices.memory[type_claim_offset + touched_index]);
		batch_index := claim_index / CONSTRAINT_TYPE_ID_CAPACITY;
		type_id := claim_index - batch_index * CONSTRAINT_TYPE_ID_CAPACITY;
		owner := narrow_phase_type_batch_worker_index(
			batch_index, type_id, worker_count, narrow.solver.fallback_batch_index,
		);
		if owner != worker_index
		{
			continue;
		}
		claim := &narrow.type_claims.memory[claim_index];
		batch := &narrow.solver.active_set.batches.memory[batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[type_id]);
		batch.type_batches.memory[type_batch_index].count += claim.count;
	}
}
narrow_phase_compact_remove_groups_worker :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, worker_count, returned_handle_start: int,
)
{
	remove_index := 0;
	for remove_index < narrow.remove_count
	{
		group_start := remove_index;
		group_key := narrow.remove_order.memory[group_start].key >> 32;
		remove_index += 1;
		for remove_index < narrow.remove_count &&
				narrow.remove_order.memory[remove_index].key >> 32 == group_key
		{
			remove_index += 1;
		}
		first := narrow_phase_remove_transaction(narrow, group_start);
		owner := narrow_phase_type_batch_worker_index(
			int(first.batch_index), int(first.type_record.type_id), worker_count,
			narrow.solver.fallback_batch_index,
		);
		if owner != worker_index
		{
			continue;
		}
		for group_index in group_start ..< remove_index
		{
			solver_compact_remove_transaction_owned(
				narrow.solver,
				narrow_phase_remove_transaction(narrow, group_index),
				returned_handle_start + group_index,
			);
		}
	}
}
narrow_phase_commit_update_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Narrow_Phase_Transaction_Commit_Job)(dispatcher.unmanaged_context);
	narrow_phase_commit_write_range(job.narrow, worker_index);
}
narrow_phase_commit_update_remove_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Narrow_Phase_Parallel_Remove_Commit_Job)(dispatcher.unmanaged_context);
	narrow_phase_commit_write_range(job.narrow, worker_index);
	narrow_phase_finalize_type_adds_worker(
		job.narrow, worker_index, job.worker_count,
	);
	narrow_phase_compact_remove_groups_worker(
		job.narrow, worker_index, job.worker_count, job.returned_handle_start,
	);
}
narrow_phase_commit_remove_side_effects :: proc "contextless" (narrow: ^Narrow_Phase)
{
	for remove_index in 0 ..< narrow.remove_count
	{
		solver_commit_remove_transaction_side_effects(
			narrow.solver, narrow_phase_remove_transaction(narrow, remove_index),
		);
	}
}
narrow_phase_commit_pair_removals_compact :: proc "contextless" (narrow: ^Narrow_Phase)
{
	for remove_index in 0 ..< narrow.remove_count
	{
		transaction_index := int(narrow.remove_order.memory[remove_index].record_index);
		header := &narrow.transaction_headers.memory[transaction_index];
		if header.kind == .Remove
		{
			pair_cache_commit_remove(&narrow.pair_cache, header.pair);
		}
	}
}
narrow_phase_commit_parallel_remove_tail :: proc "contextless" (
	narrow: ^Narrow_Phase, returned_handle_start: int,
)
{
	batch := (^Constraint_Batch)(nil);
	batch_remove_count := 0;
	fallback_remove_count := 0;
	for remove_index in 0 ..< narrow.remove_count
	{
		transaction_index := int(narrow.remove_order.memory[remove_index].record_index);
		header := &narrow.transaction_headers.memory[transaction_index];
		transaction := narrow_phase_remove_transaction(narrow, remove_index);
		if transaction.batch != batch
		{
			if batch != nil
			{
				batch.constraint_count -= i32(batch_remove_count);
			}
			batch = transaction.batch;
			batch_remove_count = 0;
		}
		batch_remove_count += 1;
		if transaction.batch_index == narrow.solver.fallback_batch_index
		{
			sequential_fallback_remove_body_references(
				&narrow.solver.sequential_batch, &transaction.reference,
			);
			fallback_remove_count += 1;
		}
		else
		{
			constraint_batch_remove_body_references(
				transaction.batch, &transaction.reference,
			);
		}
		for body_index in 0 ..< int(transaction.reference.body_count)
		{
			list := transaction.body_lists[body_index];
			for reference_index in 0 ..< list.count
			{
				if list.span.memory[reference_index].connecting_constraint_handle.value !=
					transaction.reference.handle.value
				{
					continue;
				}
				list.count -= 1;
				if reference_index < list.count
				{
					list.span.memory[reference_index] = list.span.memory[list.count];
				}
				list.span.memory[list.count] = {};
				break;
			}
		}
		if header.kind == .Remove
		{
			pair_cache_commit_remove(&narrow.pair_cache, header.pair);
		}
	}
	if batch != nil
	{
		batch.constraint_count -= i32(batch_remove_count);
	}
	narrow.solver.active_set.constraint_count -= i32(narrow.remove_count);
	narrow.solver.sequential_batch.constraint_count -= i32(fallback_remove_count);
	narrow.solver.handle_pool.available_id_count =
		returned_handle_start + narrow.remove_count;
	solver_trim_empty_trailing_batches(narrow.solver);
}
narrow_phase_finalize_parallel_adds :: proc "contextless" (
	narrow: ^Narrow_Phase,
)
{
	add_count := 0;
	fallback_add_count := 0;
	maximum_batch_count := narrow.solver.active_set.batch_count;
	type_claim_offset := int(narrow.body_claims.length);
	for touched_index in 0 ..< narrow.touched_type_count
	{
		claim_index := int(narrow.touched_claim_indices.memory[type_claim_offset + touched_index]);
		batch_index := claim_index / CONSTRAINT_TYPE_ID_CAPACITY;
		type_id := claim_index - batch_index * CONSTRAINT_TYPE_ID_CAPACITY;
		claim := &narrow.type_claims.memory[claim_index];
		batch := &narrow.solver.active_set.batches.memory[batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[type_id]);
		type_batch := &batch.type_batches.memory[type_batch_index];
		type_batch.count += claim.count;
		batch.constraint_count += claim.count;
		add_count += int(claim.count);
		if i32(batch_index) == narrow.solver.fallback_batch_index
		{
			fallback_add_count += int(claim.count);
		}
		if i32(batch_index + 1) > maximum_batch_count
		{
			maximum_batch_count = i32(batch_index + 1);
		}
	}
	if add_count == 0
	{
		return;
	}
	available_count := min(add_count, narrow.solver.handle_pool.available_id_count);
	narrow.solver.handle_pool.available_id_count -= available_count;
	narrow.solver.handle_pool.next_index += i32(add_count - available_count);
	narrow.pair_cache.mapping.count += narrow.prepared_pair_add_count;
	for touched_index in 0 ..< narrow.touched_body_count
	{
		handle_index := int(narrow.touched_claim_indices.memory[touched_index]);
		claim := narrow.body_claims.memory[handle_index];
		location := narrow.bodies.handle_to_location.memory[handle_index];
		list := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[location.index];
		list.count += int(claim.list_count);
	}
	narrow.solver.active_set.constraint_count += i32(add_count);
	narrow.solver.active_set.batch_count = maximum_batch_count;
	narrow.solver.sequential_batch.constraint_count += i32(fallback_add_count);
}
narrow_phase_finalize_parallel_adds_after_owned_type_counts :: proc "contextless" (
	narrow: ^Narrow_Phase,
)
{
	add_count := 0;
	fallback_add_count := 0;
	maximum_batch_count := narrow.solver.active_set.batch_count;
	type_claim_offset := int(narrow.body_claims.length);
	for touched_index in 0 ..< narrow.touched_type_count
	{
		claim_index := int(narrow.touched_claim_indices.memory[type_claim_offset + touched_index]);
		batch_index := claim_index / CONSTRAINT_TYPE_ID_CAPACITY;
		claim := &narrow.type_claims.memory[claim_index];
		batch := &narrow.solver.active_set.batches.memory[batch_index];
		batch.constraint_count += claim.count;
		add_count += int(claim.count);
		if i32(batch_index) == narrow.solver.fallback_batch_index
		{
			fallback_add_count += int(claim.count);
		}
		if i32(batch_index + 1) > maximum_batch_count
		{
			maximum_batch_count = i32(batch_index + 1);
		}
	}
	if add_count == 0
	{
		return;
	}
	available_count := min(add_count, narrow.solver.handle_pool.available_id_count);
	narrow.solver.handle_pool.available_id_count -= available_count;
	narrow.solver.handle_pool.next_index += i32(add_count - available_count);
	narrow.pair_cache.mapping.count += narrow.prepared_pair_add_count;
	for touched_index in 0 ..< narrow.touched_body_count
	{
		handle_index := int(narrow.touched_claim_indices.memory[touched_index]);
		claim := narrow.body_claims.memory[handle_index];
		location := narrow.bodies.handle_to_location.memory[handle_index];
		list := &narrow.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[location.index];
		list.count += int(claim.list_count);
	}
	narrow.solver.active_set.constraint_count += i32(add_count);
	narrow.solver.active_set.batch_count = maximum_batch_count;
	narrow.solver.sequential_batch.constraint_count += i32(fallback_add_count);
}
narrow_phase_commit_transactions :: proc (
	narrow: ^Narrow_Phase, transaction_count, worker_count: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	write_count := 0;
	for worker_index in 0 ..< worker_count
	{
		write_count += int(narrow.transaction_ranges[worker_index].count);
	}
	parallel_remove_worker_count := narrow_phase_parallel_remove_worker_count(
		worker_count, narrow.remove_count,
	);
	parallel_remove_compaction := Reference_State.Missing;
	if dispatcher != nil && parallel_remove_worker_count > 0 && write_count > 0
	{
		parallel_remove_compaction = .Present;
	}
	returned_handle_start := narrow.remove_handle_return_start;
	if write_count > 0
	{
		if dispatcher != nil && worker_count > 1
		{
			if parallel_remove_compaction == .Present
			{
				job := Narrow_Phase_Parallel_Remove_Commit_Job{
					narrow=narrow,
					worker_count=parallel_remove_worker_count,
					returned_handle_start=returned_handle_start,
				};
				dispatch_status := dispatcher.dispatch(
					dispatcher, narrow_phase_commit_update_remove_worker,
					parallel_remove_worker_count, &job,
				);
				if dispatch_status != .Ok
				{
					return .Invalid_Argument;
				}
			}
			else
			{
				job := Narrow_Phase_Transaction_Commit_Job{narrow};
				dispatch_status := dispatcher.dispatch(
					dispatcher, narrow_phase_commit_update_worker, worker_count, &job,
				);
				if dispatch_status != .Ok
				{
					return .Invalid_Argument;
				}
			}
		}
		else
		{
			boundary := util.Thread_Dispatcher_Boundary{unmanaged_context=nil};
			job := Narrow_Phase_Transaction_Commit_Job{narrow};
			boundary.unmanaged_context = &job;
			narrow_phase_commit_update_worker(0, &boundary);
		}
	}

	if worker_count > 1
	{
		if parallel_remove_compaction == .Present
		{
			narrow_phase_finalize_parallel_adds_after_owned_type_counts(narrow);
		}
		else
		{
			narrow_phase_finalize_parallel_adds(narrow);
		}
	}
	else
	{
		for order_index in 0 ..< transaction_count
		{
			transaction_index := int(narrow.transaction_order.memory[order_index]);
			header := &narrow.transaction_headers.memory[transaction_index];
			if header.kind != .Add && header.kind != .Replace
			{
				continue;
			}
			transaction := &narrow.contact_transactions.memory[header.payload_index];
			solver_add := &transaction.prepared.add.solver_add;
			if header.kind == .Replace
			{
				solver_add = &transaction.prepared.replace.solver_add;
			}
			solver_commit_add_transaction(
				narrow.solver, solver_add, &transaction.description[0],
			);
			contact_constraint_scatter_impulses_trusted(
				solver_add.type_batch, transaction.accessor,
				int(solver_add.record_index), transaction.feature_ids,
				transaction.old_cache.feature_ids, transaction.old_impulses,
				int(transaction.old_contact_count),
			);
			cache := Constraint_Cache{
				solver_add.reference.handle, transaction.feature_ids,
			};
			if header.kind == .Add
			{
				pair_cache_commit_add(
					&narrow.pair_cache, transaction.prepared.add.pair_add,
					header.pair, cache,
				);
			}
			else
			{
				pair_cache_commit_update(
					&narrow.pair_cache, int(header.pair_mapping_index),
					header.pair, cache,
				);
			}
		}
	}

	if parallel_remove_compaction == .Present
	{
		narrow_phase_commit_parallel_remove_tail(narrow, returned_handle_start);
	}
	else
	{
		for remove_index in 0 ..< narrow.remove_count
		{
			transaction_index := int(narrow.remove_order.memory[remove_index].record_index);
			header := &narrow.transaction_headers.memory[transaction_index];
			switch header.kind
			{
				case .Replace:
					transaction := &narrow.contact_transactions.memory[header.payload_index];
					solver_commit_remove_transaction(
						narrow.solver, &transaction.prepared.replace.solver_remove,
					);
				case .Remove:
					transaction := &narrow.remove_transactions.memory[header.payload_index];
					solver_commit_remove_transaction(narrow.solver, &transaction.solver_remove);
				case .None, .Pending, .Pending_Prepared, .Update, .Add:
			}
		}
		for order_index in 0 ..< transaction_count
		{
			transaction_index := int(narrow.transaction_order.memory[order_index]);
			header := &narrow.transaction_headers.memory[transaction_index];
			if header.kind == .Remove
			{
				pair_cache_commit_remove(&narrow.pair_cache, header.pair);
			}
		}
	}
	narrow.pair_cache.state = .Ready;
	return .Ok;
}
narrow_phase_transaction_stream_reserve :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, count: int,
) -> (Narrow_Phase_Transaction_Reservation, Physics_Status)
{
	reservation: Narrow_Phase_Transaction_Reservation;
	if narrow == nil || worker_index < 0 ||
		worker_index >= narrow.active_worker_count || count < 0 ||
		narrow.transaction_stream_primary_capacity <= 0 ||
		narrow.transaction_stream_page_capacity <= 0 ||
		narrow.transaction_stream_overflow_page_count <= 0 ||
		narrow.transaction_stream_storage_capacity <= 0
	{
		return reservation, .Invalid_Argument;
	}
	if count == 0
	{
		return reservation, .Ok;
	}
	stream := &narrow.transaction_streams[worker_index];
	if stream.total_count < 0 || count > narrow.current_transaction_capacity - int(stream.total_count)
	{
		return reservation, .Capacity_Missing;
	}
	primary_used := int(stream.primary_count);
	if primary_used < 0 || primary_used > narrow.transaction_stream_primary_capacity
	{
		return reservation, .Invalid_Argument;
	}
	primary_available := narrow.transaction_stream_primary_capacity - primary_used;
	primary_count := min(count, primary_available);
	remaining := count - primary_count;
	reservation.primary_start = i32(
		worker_index * narrow.transaction_stream_primary_capacity + primary_used,
	);
	reservation.primary_count = i32(primary_count);

	tail_page := int(stream.last_overflow_page);
	tail_count := 0;
	tail_used := 0;
	if remaining > 0 && tail_page >= 0
	{
		if tail_page >= narrow.transaction_stream_overflow_page_count
		{
			return {}, .Invalid_Argument;
		}
		tail_used = int(narrow.transaction_overflow_page_counts.memory[tail_page]);
		if tail_used < 0 || tail_used > narrow.transaction_stream_page_capacity
		{
			return {}, .Invalid_Argument;
		}
		tail_count = min(
			remaining, narrow.transaction_stream_page_capacity - tail_used,
		);
		remaining -= tail_count;
		reservation.tail_start = i32(
			narrow.transaction_stream_overflow_start +
			tail_page * narrow.transaction_stream_page_capacity + tail_used,
		);
		reservation.tail_count = i32(tail_count);
	}

	new_page_count := 0;
	first_new_page := -1;
	if remaining > 0
	{
		if remaining > max(int) - (narrow.transaction_stream_page_capacity - 1)
		{
			return {}, .Capacity_Missing;
		}
		new_page_count = (
			remaining + narrow.transaction_stream_page_capacity - 1
		) / narrow.transaction_stream_page_capacity;
		first_new_page = int(sync.atomic_add_explicit(
			&narrow.transaction_overflow_page_cursor,
			i32(new_page_count), .Relaxed,
		));
		if first_new_page < 0 ||
			new_page_count > narrow.transaction_stream_overflow_page_count - first_new_page
		{
			return {}, .Capacity_Missing;
		}
		reservation.overflow_start = i32(
			narrow.transaction_stream_overflow_start +
			first_new_page * narrow.transaction_stream_page_capacity,
		);
		reservation.overflow_count = i32(remaining);
	}

	stream.primary_count += i32(primary_count);
	if tail_count > 0
	{
		narrow.transaction_overflow_page_counts.memory[tail_page] = i32(
			tail_used + tail_count,
		);
	}
	if new_page_count > 0
	{
		if tail_page >= 0
		{
			narrow.transaction_overflow_page_next.memory[tail_page] = i32(first_new_page);
		}
		else
		{
			stream.first_overflow_page = i32(first_new_page);
		}
		remaining_to_publish := remaining;
		for page_offset in 0 ..< new_page_count
		{
			page_index := first_new_page + page_offset;
			page_count := min(
				remaining_to_publish, narrow.transaction_stream_page_capacity,
			);
			narrow.transaction_overflow_page_counts.memory[page_index] = i32(page_count);
			next_page := -1;
			if page_offset + 1 < new_page_count
			{
				next_page = page_index + 1;
			}
			narrow.transaction_overflow_page_next.memory[page_index] = i32(next_page);
			remaining_to_publish -= page_count;
		}
		stream.last_overflow_page = i32(first_new_page + new_page_count - 1);
	}
	stream.total_count += i32(count);
	return reservation, .Ok;
}
narrow_phase_transaction_reservation_index :: proc "contextless" (
	reservation: Narrow_Phase_Transaction_Reservation, offset: int,
) -> (int, Physics_Status)
{
	if offset < 0
	{
		return 0, .Invalid_Argument;
	}
	remaining_offset := offset;
	primary_count := int(reservation.primary_count);
	if remaining_offset < primary_count
	{
		return int(reservation.primary_start) + remaining_offset, .Ok;
	}
	remaining_offset -= primary_count;
	tail_count := int(reservation.tail_count);
	if remaining_offset < tail_count
	{
		return int(reservation.tail_start) + remaining_offset, .Ok;
	}
	remaining_offset -= tail_count;
	overflow_count := int(reservation.overflow_count);
	if remaining_offset < overflow_count
	{
		return int(reservation.overflow_start) + remaining_offset, .Ok;
	}
	return 0, .Invalid_Argument;
}
narrow_phase_prepare_transaction_order :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> (int, Physics_Status)
{
	if narrow == nil || worker_count <= 0 ||
		worker_count > narrow.active_worker_count
	{
		return 0, .Invalid_Argument;
	}
	transaction_count := 0;
	for worker_index in 0 ..< worker_count
	{
		stream := &narrow.transaction_streams[worker_index];
		if stream.primary_count < 0 || stream.total_count < stream.primary_count ||
			int(stream.primary_count) > narrow.transaction_stream_primary_capacity
		{
			return 0, .Invalid_Argument;
		}
		primary_start := worker_index * narrow.transaction_stream_primary_capacity;
		for local_index in 0 ..< int(stream.primary_count)
		{
			if transaction_count >= int(narrow.transaction_order.length)
			{
				return 0, .Capacity_Missing;
			}
			narrow.transaction_order.memory[transaction_count] = i32(
				primary_start + local_index,
			);
			transaction_count += 1;
		}
		page_index := int(stream.first_overflow_page);
		overflow_count := 0;
		visited_page_count := 0;
		for page_index >= 0
		{
			if page_index >= narrow.transaction_stream_overflow_page_count ||
				visited_page_count >= narrow.transaction_stream_overflow_page_count
			{
				return 0, .Invalid_Argument;
			}
			page_count := int(
				narrow.transaction_overflow_page_counts.memory[page_index],
			);
			if page_count <= 0 || page_count > narrow.transaction_stream_page_capacity
			{
				return 0, .Invalid_Argument;
			}
			page_start := narrow.transaction_stream_overflow_start +
				page_index * narrow.transaction_stream_page_capacity;
			for local_index in 0 ..< page_count
			{
				if transaction_count >= int(narrow.transaction_order.length)
				{
					return 0, .Capacity_Missing;
				}
				narrow.transaction_order.memory[transaction_count] = i32(
					page_start + local_index,
				);
				transaction_count += 1;
			}
			overflow_count += page_count;
			visited_page_count += 1;
			page_index = int(narrow.transaction_overflow_page_next.memory[page_index]);
		}
		if int(stream.primary_count) + overflow_count != int(stream.total_count)
		{
			return 0, .Invalid_Argument;
		}
	}
	return transaction_count, .Ok;
}
