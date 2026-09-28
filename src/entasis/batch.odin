package entasis

import physics "entasis:entasis_physics"

@(private)
batch_world_ready :: #force_inline proc "contextless" (
	world: ^World,
) -> (^world_data, ^physics.Simulation, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, .Invalid_Argument;
	}
	return data, &data.simulation, .Ok;
}

@(private)
batch_add_requirements :: struct
{
	body_count:            int,
	body_collidable_count: int,
	static_count:          int,
}

@(private)
batch_body_requirements :: #force_inline proc "contextless" (
	descriptions: []Body_Description,
) -> batch_add_requirements
{
	requirements := batch_add_requirements{body_count=len(descriptions)};
	for index in 0 ..< len(descriptions)
	{
		if physics.typed_index_state(descriptions[index].collidable.shape) == .Present
		{
			requirements.body_collidable_count += 1;
		}
	}
	return requirements;
}

@(private)
batch_preflight_adds :: proc (
	data: ^world_data,
	requirements: batch_add_requirements,
) -> Status
{
	if data == nil
	{
		return .Disposed;
	}
	if requirements.body_count < 0 ||
		requirements.body_collidable_count < 0 ||
		requirements.body_collidable_count > requirements.body_count ||
		requirements.static_count < 0
	{
		return .Invalid_Argument;
	}
	if requirements.body_count == 0 && requirements.static_count == 0
	{
		return .Ok;
	}

	simulation := &data.simulation;
	pool := simulation.pool;
	if pool == nil
	{
		return .Disposed;
	}

	if requirements.body_count > 0
	{
		available := simulation.bodies.handle_pool.available_id_count;
		new_ids := max(requirements.body_count - available, 0);
		next_index := int(simulation.bodies.handle_pool.next_index);
		active_set := &simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
		if next_index > max(int) - new_ids ||
			active_set.count > max(int) - requirements.body_count
		{
			return .Capacity_Missing;
		}
		handle_capacity := max(next_index + new_ids, 1);
		active_capacity := max(active_set.count + requirements.body_count, 1);
		status := physics.bodies_ensure_handle_capacity(
			&simulation.bodies, handle_capacity,
		);
		if status != .Ok
		{
			return status;
		}
		status = physics.body_set_ensure_capacity(active_set, active_capacity, pool);
		if status != .Ok
		{
			return status;
		}
	}

	if requirements.static_count > 0
	{
		available := simulation.statics.handle_pool.available_id_count;
		new_ids := max(requirements.static_count - available, 0);
		next_index := int(simulation.statics.handle_pool.next_index);
		if next_index > max(int) - new_ids ||
			simulation.statics.count > max(int) - requirements.static_count
		{
			return .Capacity_Missing;
		}
		static_capacity := max(
			next_index + new_ids,
			simulation.statics.count + requirements.static_count,
		);
		status := physics.statics_ensure_capacity(
			&simulation.statics, max(static_capacity, 1),
		);
		if status != .Ok
		{
			return status;
		}
	}

	if requirements.body_collidable_count > 0 || requirements.static_count > 0
	{
		active_count := simulation.broad_phase.active_tree.leaf_count;
		static_count := simulation.broad_phase.static_tree.leaf_count;
		if active_count > max(int) - requirements.body_collidable_count ||
			static_count > max(int) - requirements.static_count
		{
			return .Capacity_Missing;
		}
		status := physics.broad_phase_ensure_capacity(
			&simulation.broad_phase,
			max(active_count + requirements.body_collidable_count, 1),
			max(static_count + requirements.static_count, 1),
			max(int(simulation.allocation_sizes.broad_phase_candidates), 1),
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

@(private)
batch_validate_awakening :: #force_inline proc "contextless" (
	awakening: Awakening_Policy,
) -> Status
{
	switch awakening
	{
		case .Overlaps, .None:
			return .Ok;
		case:
			return .Invalid_Argument;
	}
}

@(private)
batch_add_static_low_level :: #force_inline proc (
	simulation: ^physics.Simulation,
	description: ^Static_Description,
	awakening: Awakening_Policy,
) -> (Static_Handle, Status)
{
	switch awakening
	{
		case .Overlaps:
			return physics.simulation_add_static(simulation, description);
		case .None:
			return physics.simulation_add_static_without_awakening_bodies(
				simulation, description,
			);
		case:
			return static_handle_invalid(), .Invalid_Argument;
	}
}

@(private)
batch_apply_static_low_level :: #force_inline proc (
	simulation: ^physics.Simulation,
	handle: Static_Handle,
	description: ^Static_Description,
	awakening: Awakening_Policy,
) -> Status
{
	switch awakening
	{
		case .Overlaps:
			return physics.simulation_apply_static_description(
				simulation, handle, description,
			);
		case .None:
			return physics.simulation_apply_static_description_without_awakening_bodies(
				simulation, handle, description,
			);
		case:
			return .Invalid_Argument;
	}
}

@(private)
batch_remove_static_low_level :: #force_inline proc (
	simulation: ^physics.Simulation,
	handle: Static_Handle,
	awakening: Awakening_Policy,
) -> Status
{
	switch awakening
	{
		case .Overlaps:
			return physics.simulation_remove_static(simulation, handle);
		case .None:
			return physics.simulation_remove_static_without_awakening_bodies(
				simulation, handle,
			);
		case:
			return .Invalid_Argument;
	}
}

// body_add_batch adds a prefix of descriptions in input order after one capacity
// check. handles must contain at least len(descriptions) entries. on failure,
// written is the number of successfully committed prefix entries and handles at
// and after written remain invalid
// allocation: the initial capacity check may grow world storage
// ownership: owner thread only while the world is idle
body_add_batch :: proc (
	world: ^World,
	descriptions: []Body_Description,
	handles: []Body_Handle,
) -> (written: int, status: Status)
{
	if len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = body_handle_invalid();
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	preflight_status := batch_preflight_adds(
		data, batch_body_requirements(descriptions),
	);
	if preflight_status != .Ok
	{
		return 0, preflight_status;
	}
	for index in 0 ..< len(descriptions)
	{
		stored_description := descriptions[index];
		handle, add_status := physics.simulation_add_body(
			simulation, &stored_description,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(descriptions), .Ok;
}

// body_apply_batch applies matching handles and descriptions in input order.
// a failure stops the batch. applied identifies the first failing index
// allocation: may occur only through the existing body mutation path
// ownership: owner thread only while the world is idle
body_apply_batch :: proc (
	world: ^World,
	handles: []Body_Handle,
	descriptions: []Body_Description,
) -> (applied: int, status: Status)
{
	if len(handles) != len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		stored_description := descriptions[index];
		apply_status := physics.simulation_apply_body_description(
			simulation, handles[index], &stored_description,
		);
		if apply_status != .Ok
		{
			return index, apply_status;
		}
	}
	return len(handles), .Ok;
}

// body_remove_batch removes handles in input order. a failure stops the batch.
// removed identifies the first failing index
// ownership: owner thread only while the world is idle
body_remove_batch :: proc (
	world: ^World,
	handles: []Body_Handle,
) -> (removed: int, status: Status)
{
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		remove_status := physics.simulation_remove_body(simulation, handles[index]);
		if remove_status != .Ok
		{
			return index, remove_status;
		}
	}
	return len(handles), .Ok;
}

// the empty static tree contains no sleeping collidables. under exclusive
// ownership, stage bounds in existing build storage instead of constructing
// an incremental tree that would immediately be discarded by the bulk build
@(private)
batch_add_statics_to_empty_tree :: proc (
	simulation: ^physics.Simulation,
	descriptions: []Static_Description,
	handles: []Static_Handle,
	awakening: Awakening_Policy,
) -> (written: int, status: Status)
{
	tree := &simulation.broad_phase.static_tree;
	status = .Ok;
	for index in 0 ..< len(descriptions)
	{
		description := descriptions[index];
		status = physics.static_description_validate(&description);
		if status != .Ok
		{
			break;
		}
		if awakening == .Overlaps
		{
			// preserve bounds callback order before shape retention. no sleeping
			// collidable can appear while this owner-thread batch is executing
			wake_bounds, bounds_status := physics.shape_registry_compute_world_bounds(
				physics.simulation_shape_registry(simulation), description.shape, description.pose,
			);
			status = bounds_status;
			if status == .Ok && (wake_bounds.min.x > wake_bounds.max.x ||
				wake_bounds.min.y > wake_bounds.max.y || wake_bounds.min.z > wake_bounds.max.z)
			{
				status = .Invalid_Argument;
			}
			if status != .Ok
			{
				break;
			}
		}
		handle, add_status := physics.statics_add(&simulation.statics, &description);
		status = add_status;
		if status != .Ok
		{
			break;
		}
		bounds, bounds_status := physics.shape_registry_compute_world_bounds(
			simulation.broad_phase.shapes, description.shape, description.pose,
		);
		status = bounds_status;
		reference, reference_status := physics.collidable_reference_static(handle);
		if status == .Ok
		{
			status = reference_status;
			if status == .Ok && (bounds.min.x > bounds.max.x ||
				bounds.min.y > bounds.max.y || bounds.min.z > bounds.max.z)
			{
				status = .Invalid_Argument;
			}
		}
		if status != .Ok
		{
			_ = physics.statics_remove(&simulation.statics, handle);
			break;
		}
		tree.build_references.memory[index] = {bounds=bounds, leaf_index=i32(index)};
		simulation.broad_phase.static_leaves.memory[index] = reference;
		simulation.statics.statics.memory[simulation.statics.count - 1].broad_phase_index = i32(index);
		handles[index] = handle;
		written += 1;
	}
	if status != .Ok
	{
		// a failed prefix keeps the original incremental topology and IDs
		for index in 0 ..< written
		{
			_ = physics.tree_add_trusted(tree, tree.build_references.memory[index].bounds);
		}
		return written, status;
	}
	return written, physics.tree_build_from_references(tree, written);
}

// static_add_batch adds a prefix of descriptions in input order after one
// capacity check. the awakening policy is applied to every item
// allocation: the initial capacity check may grow world storage
// ownership: owner thread only while the world is idle
static_add_batch :: proc (
	world: ^World,
	descriptions: []Static_Description,
	handles: []Static_Handle,
	awakening: Awakening_Policy = .Overlaps,
) -> (written: int, status: Status)
{
	if len(handles) < len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if batch_validate_awakening(awakening) != .Ok
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(descriptions)
	{
		handles[index] = static_handle_invalid();
	}
	if len(descriptions) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	preflight_status := batch_preflight_adds(
		data, {static_count=len(descriptions)},
	);
	if preflight_status != .Ok
	{
		return 0, preflight_status;
	}
	initial_tree_count := simulation.broad_phase.static_tree.leaf_count;
	if initial_tree_count == 0 && len(descriptions) > 1
	{
		return batch_add_statics_to_empty_tree(simulation, descriptions, handles, awakening);
	}
	for index in 0 ..< len(descriptions)
	{
		stored_description := descriptions[index];
		handle, add_status := batch_add_static_low_level(
			simulation, &stored_description, awakening,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	// a population-sized mutation batch triggers one quality rebuild here,
	// while the world is exclusively owned. small edits stay incremental.
	// rebuilding preserves leaf indices, including any sleeping bodies
	if len(descriptions) >= initial_tree_count && simulation.broad_phase.static_tree.leaf_count > 1
	{
		return len(descriptions), physics.tree_rebuild_binned(&simulation.broad_phase.static_tree);
	}
	return len(descriptions), .Ok;
}

// static_apply_batch applies matching handles and descriptions in input order.
// a failure stops the batch. applied identifies the first failing index
// ownership: owner thread only while the world is idle
static_apply_batch :: proc (
	world: ^World,
	handles: []Static_Handle,
	descriptions: []Static_Description,
	awakening: Awakening_Policy = .Overlaps,
) -> (applied: int, status: Status)
{
	if len(handles) != len(descriptions)
	{
		return 0, .Invalid_Argument;
	}
	if batch_validate_awakening(awakening) != .Ok
	{
		return 0, .Invalid_Argument;
	}
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		stored_description := descriptions[index];
		apply_status := batch_apply_static_low_level(
			simulation, handles[index], &stored_description, awakening,
		);
		if apply_status != .Ok
		{
			return index, apply_status;
		}
	}
	return len(handles), .Ok;
}

// static_remove_batch removes handles in input order using one awakening policy.
// a failure stops the batch. removed identifies the first failing index
// ownership: owner thread only while the world is idle
static_remove_batch :: proc (
	world: ^World,
	handles: []Static_Handle,
	awakening: Awakening_Policy = .Overlaps,
) -> (removed: int, status: Status)
{
	if batch_validate_awakening(awakening) != .Ok
	{
		return 0, .Invalid_Argument;
	}
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	data, simulation, ready_status := batch_world_ready(world);
	if ready_status != .Ok
	{
		return 0, ready_status;
	}
	world_invalidate_views_data(data);
	for index in 0 ..< len(handles)
	{
		remove_status := batch_remove_static_low_level(
			simulation, handles[index], awakening,
		);
		if remove_status != .Ok
		{
			return index, remove_status;
		}
	}
	return len(handles), .Ok;
}

@(private)
shape_batch_preflight :: proc (
	world: ^World,
	type_id: Shape_Type_ID,
	shape_count: int,
) -> (^physics.Shape_Registry, Status)
{
	if shape_count < 0 || type_id == SHAPE_TYPE_INVALID
	{
		return nil, .Invalid_Argument;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return nil, registry_status;
	}
	low_type_id := int(type_id);
	if low_type_id < 0 || low_type_id >= registry.registered_type_count
	{
		return nil, .Invalid_Argument;
	}
	if shape_count == 0
	{
		return registry, .Ok;
	}
	batch := &registry.batches[low_type_id];
	available := batch.ids.available_id_count;
	new_ids := max(shape_count - available, 0);
	next_index := int(batch.ids.next_index);
	if next_index > int(max(i32)) - new_ids || batch.active_count > int(max(i32)) - shape_count
	{
		return nil, .Capacity_Missing;
	}
	required := max(
		next_index + new_ids,
		batch.active_count + shape_count,
	);
	if required <= physics.shape_batch_capacity(batch)
	{
		return registry, .Ok;
	}
	status := physics.shape_batch_ensure_capacity(registry, low_type_id, required);
	return registry, status;
}

// shape_add_batch validates and registers one homogeneous batch of built-in
// primitive shapes. input order defines handle output order
// allocation: the initial shape-batch capacity check may grow registry storage
// ownership: owner thread only while the world is idle
shape_add_batch :: proc (
	world: ^World,
	shapes: []$T,
	handles: []Shape_Handle,
) -> (written: int, status: Status)
{
	if len(handles) < len(shapes)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(shapes)
	{
		handles[index] = shape_handle_invalid();
	}
	type_id := shape_type_id(T);
	when T == Sphere || T == Capsule || T == Box || T == Triangle || T == Cylinder
	{
		for index in 0 ..< len(shapes)
		{
			validation := shape_validate(shapes[index]);
			if validation != .Ok
			{
				return 0, validation;
			}
		}
	}
	else
	{
		return 0, .Invalid_Argument;
	}
	registry, preflight_status := shape_batch_preflight(
		world, type_id, len(shapes),
	);
	if preflight_status != .Ok
	{
		return 0, preflight_status;
	}
	for index in 0 ..< len(shapes)
	{
		stored_shape := shapes[index];
		handle, add_status := physics.shape_registry_add(
			registry, int(type_id), &stored_shape,
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(shapes), .Ok;
}

// shape_add_batch_typed is the advanced homogeneous batch path for cooked or
// custom registered shape types. successful prefix entries transfer their
// internal ownership to the registry. unprocessed suffix entries remain caller
// owned
shape_add_batch_typed :: proc (
	world: ^World,
	type_id: Shape_Type_ID,
	shapes: []$T,
	handles: []Shape_Handle,
) -> (written: int, status: Status)
{
	if len(handles) < len(shapes)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(shapes)
	{
		handles[index] = shape_handle_invalid();
	}
	registry, preflight_status := shape_batch_preflight(
		world, type_id, len(shapes),
	);
	if preflight_status != .Ok
	{
		return 0, preflight_status;
	}
	for index in 0 ..< len(shapes)
	{
		handle, add_status := physics.shape_registry_add(
			registry, int(type_id), &shapes[index],
		);
		if add_status != .Ok
		{
			return index, add_status;
		}
		handles[index] = handle;
	}
	return len(shapes), .Ok;
}

// shape_remove_batch removes unreferenced shapes in input order. a failure
// stops the batch. removed identifies the first failing index
// ownership: owner thread only while the world is idle
shape_remove_batch :: proc (
	world: ^World,
	handles: []Shape_Handle,
) -> (removed: int, status: Status)
{
	if len(handles) == 0
	{
		return 0, .Ok;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return 0, registry_status;
	}
	for index in 0 ..< len(handles)
	{
		remove_status := physics.shape_registry_remove(registry, handles[index]);
		if remove_status != .Ok
		{
			return index, remove_status;
		}
	}
	return len(handles), .Ok;
}
