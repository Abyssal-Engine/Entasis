package entasis_physics

import util "entasis:entasis_utilities"

solver_remove_order_key :: proc "contextless" (
	transaction: ^Solver_Constraint_Remove_Transaction,
) -> u64
{
	return u64(u32(transaction.batch_index)) << 40 |
		u64(u32(transaction.type_record.type_id)) << 32 |
		u64(u32(transaction.record_index));
}
solver_sort_remove_order_descending :: proc "contextless" (
	entries, scratch: util.Buffer(Solver_Remove_Order_Entry), count: int,
) -> Physics_Status
{
	if count < 0 || count > int(entries.length) || count > int(scratch.length)
	{
		return .Capacity_Missing;
	}
	if count < 2
	{
		return .Ok;
	}
	source := entries.memory;
	target := scratch.memory;
	for pass in 0 ..< 8
	{
		counts: [256]int;
		shift := pass * 8;
		for index in 0 ..< count
		{
			bucket := int((~source[index].key >> u32(shift)) & 0xff);
			counts[bucket] += 1;
		}
		offset := 0;
		for bucket in 0 ..< len(counts)
		{
			bucket_count := counts[bucket];
			counts[bucket] = offset;
			offset += bucket_count;
		}
		for index in 0 ..< count
		{
			bucket := int((~source[index].key >> u32(shift)) & 0xff);
			target[counts[bucket]] = source[index];
			counts[bucket] += 1;
		}
		source, target = target, source;
	}
	return .Ok;
}
solver_abort_body_mobility_change :: proc (
	solver: ^Solver, change: ^Solver_Body_Mobility_Change,
)
{
	if solver != nil && change != nil && change.entries.memory != nil
	{
		physics_return_buffer(solver.pool, &change.entries);
	}
	if change != nil
	{
		change^ = {};
	}
}
solver_body_mobility_change_count_delta :: proc "contextless" (
	change: ^Solver_Body_Mobility_Change, batch_index: int, body_handle: Body_Handle,
) -> int
{
	delta := 0;
	for entry_index in 0 ..< int(change.entry_count)
	{
		entry := &change.entries.memory[entry_index];
		if int(entry.source_batch_index) == batch_index &&
			body_handle.value == change.body_handle.value
		{
			delta += 1;
		}
		if entry.action != .Move
		{
			continue;
		}
		for connected_index in 0 ..< int(entry.reference.body_count)
		{
			if body_reference_mobility(entry.reference.encoded_body_references[connected_index]) != .Dynamic ||
				entry.reference.body_handles[connected_index].value != body_handle.value
			{
				continue;
			}
			if int(entry.source_batch_index) == batch_index
			{
				delta -= 1;
			}
			if int(entry.target_batch_index) == batch_index
			{
				delta += 1;
			}
		}
	}
	return delta;
}
solver_body_mobility_change_batch_fit :: proc "contextless" (
	solver: ^Solver, change: ^Solver_Body_Mobility_Change,
	reference: ^Constraint_Reference, source_batch_index: int,
) -> int
{
	for batch_index in 0 ..< int(solver.fallback_batch_index)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		fit := Reference_State.Present;
		for connected_index in 0 ..< int(reference.body_count)
		{
			if body_reference_mobility(reference.encoded_body_references[connected_index]) != .Dynamic
			{
				continue;
			}
			handle := reference.body_handles[connected_index];
			count := batch.body_reference_counts.memory[handle.value] +
				i32(solver_body_mobility_change_count_delta(change, batch_index, handle));
			if batch_index == source_batch_index &&
				handle.value == change.body_handle.value
			{
				count += 1;
			}
			if count != 0
			{
				fit = .Missing;
				break;
			}
		}
		if fit == .Present
		{
			return batch_index;
		}
	}
	return int(solver.fallback_batch_index);
}
solver_body_mobility_change_target_add_count :: proc "contextless" (
	change: ^Solver_Body_Mobility_Change, target_batch_index: int, type_id: i32,
) -> int
{
	count := 0;
	for entry_index in 0 ..< int(change.entry_count)
	{
		entry := &change.entries.memory[entry_index];
		if entry.action == .Move &&
			int(entry.target_batch_index) == target_batch_index &&
			entry.type_id == type_id
		{
			count += 1;
		}
	}
	return count;
}
solver_prepare_body_mobility_change :: proc (
	solver: ^Solver, body_handle: Body_Handle, body_index: int,
	previous, current: Body_Mobility, change: ^Solver_Body_Mobility_Change,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || change == nil ||
		change.state != .Unprepared || solver.bodies == nil ||
		body_handle.value < 0 || body_index < 0 ||
		previous == current || previous == .Static || current == .Static
	{
		return .Invalid_Argument;
	}
	active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if body_index >= active.count ||
		active.index_to_handle.memory[body_index].value != body_handle.value
	{
		return .Invalid_Argument;
	}
	constraints := &active.constraints.memory[body_index];
	change^ = {
		body_handle=body_handle,
		body_index=i32(body_index),
		previous=previous,
		current=current,
		state=.Prepared,
	};
	if constraints.count == 0
	{
		return .Ok;
	}
	entries, entries_status := util.buffer_pool_take_at_least(
		solver.pool, Solver_Body_Mobility_Change_Entry, constraints.count,
	);
	if entries_status != .Ok
	{
		change^ = {};
		return physics_memory_status(entries_status);
	}
	change.entries = entries;
	if current == .Kinematic
	{
		removal_count := 0;
		for reference_index := constraints.count - 1; reference_index >= 0; reference_index -= 1
		{
			body_reference := constraints.span.memory[reference_index];
			location, resolve_status := solver_resolve(
				solver, body_reference.connecting_constraint_handle,
			);
			if resolve_status != .Ok
			{
				solver_abort_body_mobility_change(solver, change);
				return resolve_status;
			}
			batch := &solver.active_set.batches.memory[location.batch_index];
			type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
			if type_batch_index < 0
			{
				solver_abort_body_mobility_change(solver, change);
				return .Invalid_Argument;
			}
			type_batch := &batch.type_batches.memory[type_batch_index];
			reference, read_status := type_batch_read_reference(
				type_batch, solver.bodies, int(location.index_in_type_batch),
			);
			if read_status != .Ok ||
				reference.body_handles[body_reference.body_index_in_constraint].value != body_handle.value ||
				body_reference_mobility(
				reference.encoded_body_references[body_reference.body_index_in_constraint],
			) != .Dynamic
			{
				solver_abort_body_mobility_change(solver, change);
				if read_status != .Ok
				{
					return read_status;
				}
				return .Invalid_Argument;
			}
			dynamic_count := 0;
			for connected_index in 0 ..< int(reference.body_count)
			{
				connected_location, connected_status := bodies_resolve(
					solver.bodies, reference.body_handles[connected_index],
				);
				if connected_status != .Ok ||
					connected_location.set_index != BODIES_ACTIVE_SET_INDEX
				{
					solver_abort_body_mobility_change(solver, change);
					return .Invalid_Argument;
				}
				if body_reference_mobility(reference.encoded_body_references[connected_index]) == .Dynamic
				{
					dynamic_count += 1;
				}
			}
			action := Solver_Body_Mobility_Change_Action.Update;
			if dynamic_count == 1
			{
				action = .Remove;
				removal_count += 1;
			}
			change.entries.memory[change.entry_count] = {
				reference=reference,
				handle=body_reference.connecting_constraint_handle,
				source_batch_index=location.batch_index,
				target_batch_index=location.batch_index,
				type_id=location.type_id,
				body_index_in_constraint=body_reference.body_index_in_constraint,
				action=action,
			};
			change.entry_count += 1;
		}
		if removal_count > int(solver.handle_pool.available_ids.length) -
			solver.handle_pool.available_id_count
		{
			solver_abort_body_mobility_change(solver, change);
			return .Capacity_Missing;
		}
		return .Ok;
	}
	for reference_index in 0 ..< constraints.count
	{
		body_reference := constraints.span.memory[reference_index];
		location, resolve_status := solver_resolve(
			solver, body_reference.connecting_constraint_handle,
		);
		if resolve_status != .Ok
		{
			solver_abort_body_mobility_change(solver, change);
			return resolve_status;
		}
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
		if type_batch_index < 0
		{
			solver_abort_body_mobility_change(solver, change);
			return .Invalid_Argument;
		}
		type_batch := &batch.type_batches.memory[type_batch_index];
		reference, read_status := type_batch_read_reference(
			type_batch, solver.bodies, int(location.index_in_type_batch),
		);
		if read_status != .Ok ||
			reference.body_handles[body_reference.body_index_in_constraint].value != body_handle.value ||
			body_reference_mobility(
			reference.encoded_body_references[body_reference.body_index_in_constraint],
		) != .Kinematic
		{
			solver_abort_body_mobility_change(solver, change);
			if read_status != .Ok
			{
				return read_status;
			}
			return .Invalid_Argument;
		}
		reference.encoded_body_references[body_reference.body_index_in_constraint] =
			i32(u32(reference.encoded_body_references[body_reference.body_index_in_constraint]) &
			BODY_REFERENCE_INDEX_MASK);
		target_batch_index := solver_body_mobility_change_batch_fit(
			solver, change, &reference, int(location.batch_index),
		);
		action := Solver_Body_Mobility_Change_Action.Update;
		if target_batch_index != int(location.batch_index)
		{
			action = .Move;
			type_record := &solver.registry.records[location.type_id];
			target_batch := &solver.active_set.batches.memory[target_batch_index];
			target_type_batch, target_status := constraint_batch_get_or_create_type_batch(
				target_batch, type_record, int(solver.initial_type_batch_capacity), solver.pool,
			);
			if target_status != .Ok
			{
				solver_abort_body_mobility_change(solver, change);
				return target_status;
			}
			required_capacity := int(target_type_batch.count) +
				solver_body_mobility_change_target_add_count(
				change, target_batch_index, location.type_id,
			) + 1;
			target_capacity, capacity_status := type_batch_target_capacity(
				target_type_batch, required_capacity,
			);
			if capacity_status == .Ok
			{
				capacity_status = type_batch_ensure_capacity(
					target_type_batch, target_capacity, solver.pool,
				);
			}
			if capacity_status != .Ok
			{
				solver_abort_body_mobility_change(solver, change);
				return capacity_status;
			}
		}
		else if target_batch_index != int(solver.fallback_batch_index)
		{
			solver_abort_body_mobility_change(solver, change);
			return .Invalid_Argument;
		}
		change.entries.memory[change.entry_count] = {
			reference=reference,
			handle=body_reference.connecting_constraint_handle,
			source_batch_index=location.batch_index,
			target_batch_index=i32(target_batch_index),
			type_id=location.type_id,
			body_index_in_constraint=body_reference.body_index_in_constraint,
			action=action,
		};
		change.entry_count += 1;
	}
	return .Ok;
}
solver_prepare_remove_transaction_trusted :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle,
) -> Solver_Constraint_Remove_Transaction
{
	location := solver.handle_to_constraint.memory[handle.value];
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	type_batch := &batch.type_batches.memory[type_batch_index];
	reference := type_batch_read_reference_trusted(
		type_batch, solver.bodies, int(location.index_in_type_batch),
	);
	transaction := Solver_Constraint_Remove_Transaction{
		type_record=&solver.registry.records[location.type_id],
		batch=batch,
		type_batch=type_batch,
		reference=reference,
		batch_index=location.batch_index,
		type_batch_index=i32(type_batch_index),
		record_index=location.index_in_type_batch,
	};
	for connected_index in 0 ..< int(reference.body_count)
	{
		body_location := solver.bodies.handle_to_location.memory[
			reference.body_handles[connected_index].value
		];
		transaction.body_lists[connected_index] =
			&solver.bodies.sets.memory[body_location.set_index].constraints.memory[body_location.index];
	}
	return transaction;
}
solver_commit_body_mobility_change :: proc (
	solver: ^Solver, change: ^Solver_Body_Mobility_Change,
)
{
	for entry_index in 0 ..< int(change.entry_count)
	{
		entry := &change.entries.memory[entry_index];
		location := solver.handle_to_constraint.memory[entry.handle.value];
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch := &batch.type_batches.memory[
			batch.type_id_to_batch_index[location.type_id]
		];
		switch entry.action
		{
			case .Remove:
				transaction := solver_prepare_remove_transaction_trusted(solver, entry.handle);
				solver_commit_remove_transaction(solver, &transaction);
			case .Update:
				_ = type_batch_update_body_mobility(
					type_batch, int(location.index_in_type_batch),
					int(entry.body_index_in_constraint), change.current,
				);
				if location.batch_index == solver.fallback_batch_index
				{
					if change.current == .Dynamic
					{
						solver.sequential_batch.dynamic_body_constraint_counts.memory[change.body_index] += 1;
					}
					else
					{
						solver.sequential_batch.dynamic_body_constraint_counts.memory[change.body_index] -= 1;
					}
				}
				else if change.current == .Dynamic
				{
					batch.body_reference_counts.memory[change.body_handle.value] += 1;
				}
				else
				{
					batch.body_reference_counts.memory[change.body_handle.value] -= 1;
				}
			case .Move:
				_ = type_batch_update_body_mobility(
					type_batch, int(location.index_in_type_batch),
					int(entry.body_index_in_constraint), .Dynamic,
				);
				if location.batch_index == solver.fallback_batch_index
				{
					solver.sequential_batch.dynamic_body_constraint_counts.memory[change.body_index] += 1;
				}
				else
				{
					batch.body_reference_counts.memory[change.body_handle.value] += 1;
				}
				solver_move_to_batch_commit_trusted(
					solver, entry.handle, int(entry.target_batch_index),
				);
		}
	}
	solver_abort_body_mobility_change(solver, change);
}
solver_write_add_transaction_trusted :: #force_inline proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Add_Transaction, description: rawptr,
)
{
	type_record := transaction.type_record;
	type_batch := transaction.type_batch;
	index := int(transaction.record_index);
	prestep_bundle := type_batch_prestep_bundle(type_batch, index);
	impulse_bundle := type_batch_impulse_bundle(type_batch, index);
	lane := index % util.PRODUCTION_LANE_COUNT;
	type_record.remove_record(
		prestep_bundle, impulse_bundle, int(type_record.prestep_bundle_size),
		int(type_record.impulse_bundle_size), lane,
	);
	type_record.apply_description(
		description, prestep_bundle, int(type_record.type_id), int(type_record.description_size),
		int(type_record.prestep_bundle_size), lane,
	);
	type_batch_write_reference_trusted(type_batch, index, transaction.reference);
	if transaction.batch_index == solver.fallback_batch_index
	{
		sequential_fallback_add_body_references(&solver.sequential_batch, &transaction.reference);
	}
	else
	{
		constraint_batch_add_body_references(transaction.batch, &transaction.reference);
	}
	for body_index in 0 ..< int(transaction.reference.body_count)
	{
		list := transaction.body_lists[body_index];
		list.span.memory[transaction.body_list_indices[body_index]] = {
			transaction.reference.handle, i32(body_index),
		};
	}
	handle := transaction.reference.handle;
	solver.handle_to_constraint.memory[handle.value] = {
		0, transaction.batch_index, type_record.type_id, transaction.record_index,
	};
}
solver_commit_add_transaction :: proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Add_Transaction, description: rawptr,
)
{
	if solver.handle_pool.available_id_count > 0
	{
		solver.handle_pool.available_id_count -= 1;
	}
	else
	{
		solver.handle_pool.next_index += 1;
	}
	solver_write_add_transaction_trusted(solver, transaction, description);
	transaction.type_batch.count += 1;
	transaction.batch.constraint_count += 1;
	for body_index in 0 ..< int(transaction.reference.body_count)
	{
		transaction.body_lists[body_index].count += 1;
	}
	solver.active_set.constraint_count += 1;
	if transaction.batch_index + 1 > solver.active_set.batch_count
	{
		solver.active_set.batch_count = transaction.batch_index + 1;
	}
	if transaction.batch_index == solver.fallback_batch_index
	{
		solver.sequential_batch.constraint_count += 1;
	}
}
solver_commit_update_transaction :: #force_inline proc "contextless" (
	transaction: ^Solver_Constraint_Update_Transaction, description: rawptr,
)
{
	index := int(transaction.record_index);
	transaction.type_record.apply_description(
		description, type_batch_prestep_bundle(transaction.type_batch, index),
		int(transaction.type_record.type_id), int(transaction.type_record.description_size),
		int(transaction.type_record.prestep_bundle_size), index % util.PRODUCTION_LANE_COUNT,
	);
}
solver_commit_remove_transaction_side_effects :: #force_inline proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Remove_Transaction,
	release_handle: Reference_State = .Present,
)
{
	batch := transaction.batch;
	if transaction.batch_index == solver.fallback_batch_index
	{
		sequential_fallback_remove_body_references(&solver.sequential_batch, &transaction.reference);
		solver.sequential_batch.constraint_count -= 1;
	}
	else
	{
		constraint_batch_remove_body_references(batch, &transaction.reference);
	}
	for body_index in 0 ..< int(transaction.reference.body_count)
	{
		list := transaction.body_lists[body_index];
		for reference_index in 0 ..< list.count
		{
			if list.span.memory[reference_index].connecting_constraint_handle.value == transaction.reference.handle.value
			{
				list.count -= 1;
				if reference_index < list.count
				{
					list.span.memory[reference_index] = list.span.memory[list.count];
				}
				list.span.memory[list.count] = {};
				break;
			}
		}
	}
	batch.constraint_count -= 1;
	solver.active_set.constraint_count -= 1;
	solver.handle_to_constraint.memory[transaction.reference.handle.value] = constraint_location_missing();
	if release_handle == .Present
	{
		if solver.joint_breaks != nil && transaction.type_record.type_id > CONTACT_4_NONCONVEX_TYPE_ID
		{
			joint_break_retire(solver.joint_breaks, transaction.reference.handle);
		}
		solver.handle_pool.available_ids.memory[solver.handle_pool.available_id_count] = transaction.reference.handle.value;
		solver.handle_pool.available_id_count += 1;
	}
}
solver_compact_remove_transaction :: #force_inline proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Remove_Transaction,
)
{
	type_batch := transaction.type_batch;
	record_index := int(transaction.record_index);
	last_index := int(type_batch.count) - 1;
	if record_index != last_index
	{
		moved := type_batch_read_reference_trusted(type_batch, solver.bodies, last_index);
		transaction.type_record.move_record(
			type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
			type_batch_prestep_bundle(type_batch, record_index), type_batch_impulse_bundle(type_batch, record_index),
			int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size),
			last_index % util.PRODUCTION_LANE_COUNT, record_index % util.PRODUCTION_LANE_COUNT,
		);
		type_batch_write_reference_trusted(type_batch, record_index, moved);
		solver.handle_to_constraint.memory[moved.handle.value].index_in_type_batch = i32(record_index);
	}
	transaction.type_record.remove_record(
		type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
		int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size),
		last_index % util.PRODUCTION_LANE_COUNT,
	);
	type_batch_clear_reference(type_batch, last_index);
	type_batch.count -= 1;
}
solver_compact_remove_transaction_owned :: #force_inline proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Remove_Transaction,
	returned_handle_index: int,
)
{
	solver_compact_remove_transaction(solver, transaction);
	handle_value := transaction.reference.handle.value;
	solver.handle_to_constraint.memory[handle_value] = constraint_location_missing();
	solver.handle_pool.available_ids.memory[returned_handle_index] = handle_value;
}
solver_trim_empty_trailing_batches :: #force_inline proc "contextless" (solver: ^Solver)
{
	for solver.active_set.batch_count > 1 &&
			solver.active_set.batches.memory[solver.active_set.batch_count - 1].constraint_count == 0
	{
		solver.active_set.batch_count -= 1;
	}
}
solver_commit_remove_transaction :: proc "contextless" (
	solver: ^Solver, transaction: ^Solver_Constraint_Remove_Transaction,
	release_handle: Reference_State = .Present,
)
{
	type_batch := transaction.type_batch;
	batch := transaction.batch;
	record_index := int(transaction.record_index);
	last_index := int(type_batch.count) - 1;
	if transaction.batch_index == solver.fallback_batch_index
	{
		sequential_fallback_remove_body_references(&solver.sequential_batch, &transaction.reference);
		solver.sequential_batch.constraint_count -= 1;
	}
	else
	{
		constraint_batch_remove_body_references(batch, &transaction.reference);
	}
	for body_index in 0 ..< int(transaction.reference.body_count)
	{
		list := transaction.body_lists[body_index];
		for reference_index in 0 ..< list.count
		{
			if list.span.memory[reference_index].connecting_constraint_handle.value == transaction.reference.handle.value
			{
				list.count -= 1;
				if reference_index < list.count
				{
					list.span.memory[reference_index] = list.span.memory[list.count];
				}
				list.span.memory[list.count] = {};
				break;
			}
		}
	}
	if record_index != last_index
	{
		moved := type_batch_read_reference_trusted(type_batch, solver.bodies, last_index);
		transaction.type_record.move_record(
			type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
			type_batch_prestep_bundle(type_batch, record_index), type_batch_impulse_bundle(type_batch, record_index),
			int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size),
			last_index % util.PRODUCTION_LANE_COUNT, record_index % util.PRODUCTION_LANE_COUNT,
		);
		type_batch_write_reference_trusted(type_batch, record_index, moved);
		solver.handle_to_constraint.memory[moved.handle.value].index_in_type_batch = i32(record_index);
	}
	transaction.type_record.remove_record(
		type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
		int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size),
		last_index % util.PRODUCTION_LANE_COUNT,
	);
	type_batch_clear_reference(type_batch, last_index);
	type_batch.count -= 1;
	batch.constraint_count -= 1;
	solver.active_set.constraint_count -= 1;
	solver.handle_to_constraint.memory[transaction.reference.handle.value] = constraint_location_missing();
	if release_handle == .Present
	{
		if solver.joint_breaks != nil && transaction.type_record.type_id > CONTACT_4_NONCONVEX_TYPE_ID
		{
			joint_break_retire(solver.joint_breaks, transaction.reference.handle);
		}
		solver.handle_pool.available_ids.memory[solver.handle_pool.available_id_count] = transaction.reference.handle.value;
		solver.handle_pool.available_id_count += 1;
	}
	for solver.active_set.batch_count > 1 &&
			solver.active_set.batches.memory[solver.active_set.batch_count - 1].constraint_count == 0
	{
		solver.active_set.batch_count -= 1;
	}
}
