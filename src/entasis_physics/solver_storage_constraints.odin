package entasis_physics

import util "entasis:entasis_utilities"

solver_validate_body_handles :: proc "contextless" (
	solver: ^Solver, body_handles: ^[4]Body_Handle, body_count: int,
) -> Physics_Status
{
	if body_handles == nil || body_count < 1 || body_count > 4
	{
		return .Invalid_Argument;
	}
	for body_index in 0 ..< body_count
	{
		location, status := bodies_resolve(solver.bodies, body_handles[body_index]);
		if status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return .Invalid_Argument;
		}
		for previous_index in 0 ..< body_index
		{
			if body_handles[previous_index].value == body_handles[body_index].value
			{
				return .Invalid_Argument;
			}
		}
	}
	return .Ok;
}

solver_prepare_body_references :: proc (
	solver: ^Solver, body_handles: ^[4]Body_Handle, body_count: int,
) -> Physics_Status
{
	for body_index in 0 ..< body_count
	{
		location, _ := bodies_resolve(solver.bodies, body_handles[body_index]);
		list := &solver.bodies.sets.memory[location.set_index].constraints.memory[location.index];
		status := util.quick_list_ensure_capacity(list, list.count + 1, solver.pool);
		if status != .Ok
		{
			return physics_collection_status(status);
		}
	}
	return .Ok;
}

solver_add_body_references :: proc (
	solver: ^Solver, reference: Constraint_Reference,
)
{
	for body_index in 0 ..< int(reference.body_count)
	{
		location, _ := bodies_resolve(solver.bodies, reference.body_handles[body_index]);
		list := &solver.bodies.sets.memory[location.set_index].constraints.memory[location.index];
		_ = util.quick_list_add_unsafely(list, Body_Constraint_Reference{reference.handle, i32(body_index)});
	}
}

solver_remove_body_references :: proc (
	solver: ^Solver, reference: Constraint_Reference,
)
{
	for body_index in 0 ..< int(reference.body_count)
	{
		location, status := bodies_resolve(solver.bodies, reference.body_handles[body_index]);
		if status != .Ok
		{
			continue;
		}
		list := &solver.bodies.sets.memory[location.set_index].constraints.memory[location.index];
		for index in 0 ..< list.count
		{
			if list.span.memory[index].connecting_constraint_handle.value == reference.handle.value
			{
				_ = util.quick_list_fast_remove_at(list, index);
				break;
			}
		}
	}
}

solver_find_batch :: proc "contextless" (
	solver: ^Solver, reference: ^Constraint_Reference,
) -> int
{
	for batch_index in 0 ..< int(solver.fallback_batch_index)
	{
		if constraint_batch_body_fit(&solver.active_set.batches.memory[batch_index], reference) == .Present
		{
			return batch_index;
		}
	}
	return int(solver.fallback_batch_index);
}

solver_build_constraint_reference :: proc "contextless" (
	solver: ^Solver, body_handles: ^[4]Body_Handle, body_count: int, handle: Constraint_Handle,
) -> (Constraint_Reference, Physics_Status)
{
	reference := Constraint_Reference{handle=handle, body_handles=body_handles^, body_count=i32(body_count)};
	for body_index in 0 ..< body_count
	{
		location, status := bodies_resolve(solver.bodies, body_handles[body_index]);
		if status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return {}, .Invalid_Argument;
		}
		encoded := u32(location.index);
		inertia := solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[location.index].inertia.local;
		if body_inertia_mobility(inertia) == .Kinematic
		{
			encoded |= BODY_REFERENCE_KINEMATIC_MASK;
		}
		reference.encoded_body_references[body_index] = i32(encoded);
	}
	return reference, .Ok;
}

solver_update_for_body_memory_move :: proc "contextless" (
	solver: ^Solver, active_set: ^Body_Set, original_body_index, new_body_index: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || active_set == nil || active_set.state != .Allocated ||
	original_body_index < 0 || new_body_index < 0 || new_body_index >= active_set.count
	{
		return .Invalid_Argument;
	}
	constraints := &active_set.constraints.memory[new_body_index];
	fallback_updated := Reference_State.Missing;
	for reference_index in 0 ..< constraints.count
	{
		body_reference := constraints.span.memory[reference_index];
		location, resolve_status := solver_resolve(solver, body_reference.connecting_constraint_handle);
		if resolve_status != .Ok
		{
			return resolve_status;
		}
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		mobility, update_status := type_batch_update_body_memory_move(
			type_batch, int(location.index_in_type_batch), int(body_reference.body_index_in_constraint),
			original_body_index, new_body_index,
		);
		if update_status != .Ok
		{
			return update_status;
		}
		if mobility == .Dynamic && location.batch_index == solver.fallback_batch_index && fallback_updated == .Missing
		{
			sequential_fallback_update_body_memory_move(&solver.sequential_batch, original_body_index, new_body_index);
			fallback_updated = .Present;
		}
	}
	return .Ok;
}

solver_add_raw :: proc (
	solver: ^Solver, type_id: i32, body_handles: ^[4]Body_Handle, description: rawptr,
) -> (Constraint_Handle, Physics_Status)
{
	if solver == nil || solver.state != .Ready
	{
		return constraint_handle_invalid(), .Disposed;
	}
	type_record, lookup_status := constraint_type_registry_lookup(&solver.registry, type_id);
	if lookup_status != .Ok
	{
		return constraint_handle_invalid(), lookup_status;
	}
	if solver_validate_body_handles(solver, body_handles, int(type_record.body_count)) != .Ok
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	validation: Physics_Status = constraint_type_validate_description(type_record, type_id, description);
	if validation != .Ok
	{
		return constraint_handle_invalid(), validation;
	}
	prepare_status := solver_prepare_body_references(solver, body_handles, int(type_record.body_count));
	if prepare_status != .Ok
	{
		return constraint_handle_invalid(), prepare_status;
	}
	handle_value, handle_status := util.id_pool_take(&solver.handle_pool);
	if handle_status != .Ok
	{
		return constraint_handle_invalid(), physics_memory_status(handle_status);
	}
	if int(handle_value) >= int(solver.handle_to_constraint.length)
	{
		capacity_status := solver_ensure_handle_capacity(
			solver,
			max(int(solver.handle_to_constraint.length) * 2, int(handle_value) + 1)
		);
		if capacity_status != .Ok
		{
			_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
			return constraint_handle_invalid(), capacity_status;
		}
	}
	reference, reference_status := solver_build_constraint_reference(
		solver, body_handles, int(type_record.body_count), {handle_value},
	);
	if reference_status != .Ok
	{
		_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
		return constraint_handle_invalid(), reference_status;
	}
	batch_index := solver_find_batch(solver, &reference);
	if batch_index < 0
	{
		_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
		return constraint_handle_invalid(), .Capacity_Missing;
	}
	batch := &solver.active_set.batches.memory[batch_index];
	type_batch, type_batch_status := constraint_batch_get_or_create_type_batch(
		batch,
		type_record,
		int(solver.initial_type_batch_capacity),
		solver.pool
	);
	if type_batch_status != .Ok
	{
		_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
		return constraint_handle_invalid(), type_batch_status;
	}
	if int(type_batch.count) >= int(type_batch.index_to_handle.length)
	{
		capacity_status := type_batch_ensure_capacity(
			type_batch,
			max(int(type_batch.index_to_handle.length) * 2, 1),
			solver.pool
		);
		if capacity_status != .Ok
		{
			_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
			return constraint_handle_invalid(), capacity_status;
		}
	}
	handle := Constraint_Handle{handle_value};
	index := int(type_batch.count);
	reference.handle = handle;
	prestep_bundle := type_batch_prestep_bundle(type_batch, index);
	impulse_bundle := type_batch_impulse_bundle(type_batch, index);
	lane := index % util.PRODUCTION_LANE_COUNT;
	type_record.remove_record(
		prestep_bundle, impulse_bundle, int(type_record.prestep_bundle_size),
		int(type_record.impulse_bundle_size), lane,
	);
	type_record.apply_description(
		description, prestep_bundle, int(type_id), int(type_record.description_size),
		int(type_record.prestep_bundle_size), lane,
	);
	reference_status = type_batch_write_reference(type_batch, index, reference);
	if reference_status != .Ok
	{
		_ = util.id_pool_return(&solver.handle_pool, handle_value, solver.pool);
		return constraint_handle_invalid(), reference_status;
	}
	type_batch.count += 1;
	batch.constraint_count += 1;
	if batch_index == int(solver.fallback_batch_index)
	{
		sequential_fallback_add_body_references(&solver.sequential_batch, &reference);
	}
	else
	{
		constraint_batch_add_body_references(batch, &reference);
	}
	solver.handle_to_constraint.memory[handle_value] = {0, i32(batch_index), type_id, i32(index)};
	solver.active_set.constraint_count += 1;
	if batch_index + 1 > int(solver.active_set.batch_count)
	{
		solver.active_set.batch_count = i32(batch_index + 1);
	}
	if batch_index == int(solver.fallback_batch_index)
	{
		solver.sequential_batch.constraint_count += 1;
	}
	solver_add_body_references(solver, reference);
	return handle, .Ok;
}

solver_update_for_body_memory_move_trusted :: proc "contextless" (
	solver: ^Solver, active_set: ^Body_Set, original_body_index, new_body_index: int,
)
{
	constraints := &active_set.constraints.memory[new_body_index];
	fallback_updated := Reference_State.Missing;
	for reference_index in 0 ..< constraints.count
	{
		body_reference := constraints.span.memory[reference_index];
		location := solver.handle_to_constraint.memory[body_reference.connecting_constraint_handle.value];
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		mobility, _ := type_batch_update_body_memory_move(
			type_batch, int(location.index_in_type_batch), int(body_reference.body_index_in_constraint),
			original_body_index, new_body_index,
		);
		if mobility == .Dynamic && location.batch_index == solver.fallback_batch_index &&
		fallback_updated == .Missing
		{
			sequential_fallback_update_body_memory_move(
				&solver.sequential_batch, original_body_index, new_body_index,
			);
			fallback_updated = .Present;
		}
	}
}

solver_add :: proc (
	solver: ^Solver, body_handles: ^[4]Body_Handle, description: ^$T,
) -> (Constraint_Handle, Physics_Status)
{
	type_id := constraint_description_type_id(T);
	if type_id < 0
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	return solver_add_raw(solver, type_id, body_handles, description);
}

solver_get_description_raw :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, type_id: i32, target: rawptr, target_size: int,
) -> Physics_Status
{
	location, resolve_status := solver_resolve(solver, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if location.type_id != type_id
	{
		return .Invalid_Argument;
	}
	type_record, _ := constraint_type_registry_lookup(&solver.registry, type_id);
	if target_size != int(type_record.description_size)
	{
		return .Invalid_Argument;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
	index := int(location.index_in_type_batch);
	return type_record.build_description(
		type_batch_prestep_bundle(type_batch, index), target, int(type_id), target_size,
		int(type_record.prestep_bundle_size), index % util.PRODUCTION_LANE_COUNT,
	);
}

solver_get_description :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, target: ^$T,
) -> Physics_Status
{
	return solver_get_description_raw(solver, handle, constraint_description_type_id(T), target, size_of(T));
}

solver_apply_description_raw :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, type_id: i32, description: rawptr,
) -> Physics_Status
{
	location, resolve_status := solver_resolve(solver, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if location.type_id != type_id
	{
		return .Invalid_Argument;
	}
	type_record, _ := constraint_type_registry_lookup(&solver.registry, type_id);
	validation: Physics_Status = constraint_type_validate_description(type_record, type_id, description);
	if validation != .Ok
	{
		return validation;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[type_id]];
	index := int(location.index_in_type_batch);
	type_record.apply_description(
		description, type_batch_prestep_bundle(type_batch, index), int(type_id), int(type_record.description_size),
		int(type_record.prestep_bundle_size), index % util.PRODUCTION_LANE_COUNT,
	);
	return .Ok;
}

solver_apply_description :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, description: ^$T,
) -> Physics_Status
{
	return solver_apply_description_raw(solver, handle, constraint_description_type_id(T), description);
}

solver_remove_record_at :: proc (
	solver: ^Solver, batch_index, type_batch_index, record_index: int, release_handle: Reference_State,
) -> Physics_Status
{
	batch := &solver.active_set.batches.memory[batch_index];
	type_batch := &batch.type_batches.memory[type_batch_index];
	removed, reference_status := type_batch_read_reference(type_batch, solver.bodies, record_index);
	if reference_status != .Ok
	{
		return reference_status;
	}
	type_record, lookup_status := constraint_type_registry_lookup(&solver.registry, type_batch.type_id);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	last_index := int(type_batch.count) - 1;
	if batch_index == int(solver.fallback_batch_index)
	{
		sequential_fallback_remove_body_references(&solver.sequential_batch, &removed);
	}
	else
	{
		constraint_batch_remove_body_references(batch, &removed);
	}
	solver_remove_body_references(solver, removed);
	if record_index != last_index
	{
		moved, moved_status := type_batch_read_reference(type_batch, solver.bodies, last_index);
		if moved_status != .Ok
		{
			return moved_status;
		}
		type_record.move_record(
			type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
			type_batch_prestep_bundle(type_batch, record_index), type_batch_impulse_bundle(type_batch, record_index),
			int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size),
			last_index % util.PRODUCTION_LANE_COUNT, record_index % util.PRODUCTION_LANE_COUNT,
		);
		type_batch_write_reference_trusted(type_batch, record_index, moved);
		solver.handle_to_constraint.memory[moved.handle.value].index_in_type_batch = i32(record_index);
	}
	type_record.remove_record(
		type_batch_prestep_bundle(type_batch, last_index), type_batch_impulse_bundle(type_batch, last_index),
		int(type_batch.prestep_bundle_size), int(type_batch.impulse_bundle_size), last_index % util.PRODUCTION_LANE_COUNT,
	);
	type_batch_clear_reference(type_batch, last_index);
	type_batch.count -= 1;
	batch.constraint_count -= 1;
	solver.active_set.constraint_count -= 1;
	if batch_index == int(solver.fallback_batch_index)
	{
		solver.sequential_batch.constraint_count -= 1;
	}
	solver.handle_to_constraint.memory[removed.handle.value] = constraint_location_missing();
	if release_handle == .Present
	{
		if solver.joint_breaks != nil && type_record.type_id > CONTACT_4_NONCONVEX_TYPE_ID
		{
			joint_break_retire(solver.joint_breaks, removed.handle);
		}
		_ = util.id_pool_return(&solver.handle_pool, removed.handle.value, solver.pool);
	}
	for solver.active_set.batch_count > 1 &&
	solver.active_set.batches.memory[solver.active_set.batch_count - 1].constraint_count == 0
	{
		solver.active_set.batch_count -= 1;
	}
	return .Ok;
}

solver_remove :: proc (solver: ^Solver, handle: Constraint_Handle) -> Physics_Status
{
	location, resolve_status := solver_resolve(solver, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	return solver_remove_record_at(
		solver,
		int(location.batch_index),
		int(batch.type_id_to_batch_index[location.type_id]),
		int(location.index_in_type_batch),
		.Present
	);
}

solver_move_to_batch_commit_trusted :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, target_batch_index: int,
)
{
	location := solver.handle_to_constraint.memory[handle.value];
	source_batch := &solver.active_set.batches.memory[location.batch_index];
	source_type_batch := &source_batch.type_batches.memory[
		source_batch.type_id_to_batch_index[location.type_id]
	];
	reference := type_batch_read_reference_trusted(
		source_type_batch, solver.bodies, int(location.index_in_type_batch),
	);
	type_record := &solver.registry.records[location.type_id];
	target_batch := &solver.active_set.batches.memory[target_batch_index];
	target_type_batch := &target_batch.type_batches.memory[
		target_batch.type_id_to_batch_index[location.type_id]
	];
	target_index := int(target_type_batch.count);
	type_record.move_record(
		type_batch_prestep_bundle(source_type_batch, int(location.index_in_type_batch)),
		type_batch_impulse_bundle(source_type_batch, int(location.index_in_type_batch)),
		type_batch_prestep_bundle(target_type_batch, target_index),
		type_batch_impulse_bundle(target_type_batch, target_index),
		int(type_record.prestep_bundle_size), int(type_record.impulse_bundle_size),
		int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT,
		target_index % util.PRODUCTION_LANE_COUNT,
	);
	type_batch_write_reference_trusted(target_type_batch, target_index, reference);
	target_type_batch.count += 1;
	target_batch.constraint_count += 1;
	if target_batch_index == int(solver.fallback_batch_index)
	{
		sequential_fallback_add_body_references(&solver.sequential_batch, &reference);
	}
	else
	{
		constraint_batch_add_body_references(target_batch, &reference);
	}
	// the body reference lists keep the same handle and do not move
	if int(location.batch_index) == int(solver.fallback_batch_index)
	{
		sequential_fallback_remove_body_references(&solver.sequential_batch, &reference);
	}
	else
	{
		constraint_batch_remove_body_references(source_batch, &reference);
	}
	last_index := int(source_type_batch.count) - 1;
	if int(location.index_in_type_batch) != last_index
	{
		moved := type_batch_read_reference_trusted(
			source_type_batch, solver.bodies, last_index,
		);
		type_record.move_record(
			type_batch_prestep_bundle(source_type_batch, last_index),
			type_batch_impulse_bundle(source_type_batch, last_index),
			type_batch_prestep_bundle(source_type_batch, int(location.index_in_type_batch)),
			type_batch_impulse_bundle(source_type_batch, int(location.index_in_type_batch)),
			int(type_record.prestep_bundle_size), int(type_record.impulse_bundle_size),
			last_index % util.PRODUCTION_LANE_COUNT,
			int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT,
		);
		type_batch_write_reference_trusted(
			source_type_batch, int(location.index_in_type_batch), moved,
		);
		solver.handle_to_constraint.memory[moved.handle.value].index_in_type_batch =
		location.index_in_type_batch;
	}
	type_record.remove_record(
		type_batch_prestep_bundle(source_type_batch, last_index),
		type_batch_impulse_bundle(source_type_batch, last_index),
		int(type_record.prestep_bundle_size), int(type_record.impulse_bundle_size),
		last_index % util.PRODUCTION_LANE_COUNT,
	);
	type_batch_clear_reference(source_type_batch, last_index);
	source_type_batch.count -= 1;
	source_batch.constraint_count -= 1;
	if int(location.batch_index) == int(solver.fallback_batch_index)
	{
		solver.sequential_batch.constraint_count -= 1;
	}
	if target_batch_index == int(solver.fallback_batch_index)
	{
		solver.sequential_batch.constraint_count += 1;
	}
	solver.handle_to_constraint.memory[handle.value] = {
		0, i32(target_batch_index), location.type_id, i32(target_index),
	};
	if target_batch_index + 1 > int(solver.active_set.batch_count)
	{
		solver.active_set.batch_count = i32(target_batch_index + 1);
	}
	solver_trim_empty_trailing_batches(solver);
}

solver_move_to_batch_internal :: proc (
	solver: ^Solver, handle: Constraint_Handle, target_batch_index: int,
) -> Physics_Status
{
	location, resolve_status := solver_resolve(solver, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if target_batch_index < 0 ||
	target_batch_index >= int(solver.active_set.batches.length) ||
	target_batch_index == int(location.batch_index)
	{
		return .Invalid_Argument;
	}
	source_batch := &solver.active_set.batches.memory[location.batch_index];
	source_type_batch_index := int(source_batch.type_id_to_batch_index[location.type_id]);
	source_type_batch := &source_batch.type_batches.memory[source_type_batch_index];
	reference, reference_status := type_batch_read_reference(
		source_type_batch,
		solver.bodies,
		int(location.index_in_type_batch)
	);
	if reference_status != .Ok
	{
		return reference_status;
	}
	type_record, lookup_status := constraint_type_registry_lookup(&solver.registry, location.type_id);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	target_batch := &solver.active_set.batches.memory[target_batch_index];
	if target_batch_index != int(solver.fallback_batch_index) &&
	constraint_batch_body_fit(target_batch, &reference) == .Missing
	{
		return .Invalid_Argument;
	}
	target_type_batch, target_status := constraint_batch_get_or_create_type_batch(
		target_batch,
		type_record,
		int(solver.initial_type_batch_capacity),
		solver.pool
	);
	if target_status != .Ok
	{
		return target_status;
	}
	if int(target_type_batch.count) >= int(target_type_batch.index_to_handle.length)
	{
		capacity_status := type_batch_ensure_capacity(
			target_type_batch,
			max(int(target_type_batch.index_to_handle.length) * 2, 1),
			solver.pool
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
	}
	solver_move_to_batch_commit_trusted(solver, handle, target_batch_index);
	return .Ok;
}

solver_move_constraint :: proc (
	solver: ^Solver, handle: Constraint_Handle, target_batch_index: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready
	{
		return .Disposed;
	}
	return solver_move_to_batch_internal(solver, handle, target_batch_index);
}
