// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

BODIES_ACTIVE_SET_INDEX :: 0;
Bodies :: struct
{
	sets:                                 util.Buffer(Body_Set),
	inactive_arena:                       Body_Set_Arena,
	handle_to_location:                   util.Buffer(Body_Memory_Location),
	handle_pool:                          util.Id_Pool,
	pool:                                 ^util.Buffer_Pool,
	shapes:                               ^Shape_Registry,
	solver:                               ^Solver,
	minimum_constraint_capacity_per_body: int,
	initial_set_capacity:                 int,
	state:                                Body_Set_State,
	triggers: ^Trigger_System,
	restitution: ^Restitution_Storage,
	body_control: ^Body_Control_Storage,
}

bodies_bind_shape_registry :: proc "contextless" (bodies: ^Bodies, shapes: ^Shape_Registry) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || shapes == nil || shapes.state != .Allocated ||
	bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].count != 0
	{
		return .Invalid_Argument;
	}
	bodies.shapes = shapes;
	return .Ok;
}

bodies_initialize :: proc (
	bodies: ^Bodies, body_capacity, set_capacity, minimum_constraint_capacity_per_body: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if bodies == nil ||
	pool == nil ||
	body_capacity <= 0 ||
	set_capacity <= 0 ||
	minimum_constraint_capacity_per_body <= 0
	{
		return .Invalid_Argument;
	}
	sets, sets_status := util.buffer_pool_take_at_least(pool, Body_Set, set_capacity);
	if sets_status != .Ok
	{
		return physics_memory_status(sets_status);
	}
	_ = util.buffer_clear(sets, 0, int(sets.length));
	locations, locations_status := util.buffer_pool_take_at_least(pool, Body_Memory_Location, body_capacity);
	if locations_status != .Ok
	{
		physics_return_buffer(pool, &sets);
		return physics_memory_status(locations_status);
	}
	for index in 0 ..< locations.length
	{
		locations.memory[index] = {-1, -1};
	}
	handle_pool: util.Id_Pool;
	id_status := util.id_pool_initialize(&handle_pool, body_capacity, pool);
	if id_status != .Ok
	{
		physics_return_buffer(pool, &locations);
		physics_return_buffer(pool, &sets);
		return physics_memory_status(id_status);
	}
	active_status := body_set_initialize(&sets.memory[BODIES_ACTIVE_SET_INDEX], body_capacity, pool);
	if active_status != .Ok
	{
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		physics_return_buffer(pool, &sets);
		return active_status;
	}
	inactive_arena: Body_Set_Arena;
	arena_status := body_set_arena_initialize(&inactive_arena, body_capacity, pool);
	if arena_status != .Ok
	{
		_ = body_set_dispose(&sets.memory[BODIES_ACTIVE_SET_INDEX], pool);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		physics_return_buffer(pool, &sets);
		return arena_status;
	}
	for set_index in 1 ..< sets.length
	{
		bind_status := body_set_bind_arena_range(&sets.memory[set_index], &inactive_arena, 0, 0);
		if bind_status != .Ok
		{
			_ = body_set_arena_dispose(&inactive_arena, pool);
			_ = body_set_dispose(&sets.memory[BODIES_ACTIVE_SET_INDEX], pool);
			_ = util.id_pool_dispose(&handle_pool, pool);
			physics_return_buffer(pool, &locations);
			physics_return_buffer(pool, &sets);
			return bind_status;
		}
	}
	bodies^ = {
		sets=sets,
		inactive_arena=inactive_arena,
		handle_to_location=locations,
		handle_pool=handle_pool,
		pool=pool,
		minimum_constraint_capacity_per_body=minimum_constraint_capacity_per_body,
		initial_set_capacity=body_capacity,
		state=.Allocated,
	};
	return .Ok;
}

bodies_prepare_inactive_set_capacity :: proc "contextless" (
	bodies: ^Bodies, set_index, capacity: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || set_index <= BODIES_ACTIVE_SET_INDEX ||
	set_index >= int(bodies.sets.length) || capacity <= 0 ||
	bodies.inactive_arena.state != .Allocated
	{
		return .Invalid_Argument;
	}
	target := &bodies.sets.memory[set_index];
	if target.state != .Allocated || target.storage != .Shared_Arena
	{
		return .Invalid_Description;
	}
	if target.count > 0
	{
		if capacity <= body_set_capacity(target)
		{
			return .Ok;
		}
		return .Capacity_Missing;
	}
	cursor := 0;
	previous_source_start := -1;
	for
	{
		owner_index := -1;
		owner_source_start := max(int);
		for candidate_index in 1 ..< int(bodies.sets.length)
		{
			candidate := &bodies.sets.memory[candidate_index];
			if candidate.count <= 0
			{
				continue;
			}
			source_start := int(candidate.arena_start);
			if source_start > previous_source_start && source_start < owner_source_start
			{
				owner_index = candidate_index;
				owner_source_start = source_start;
			}
		}
		if owner_index < 0
		{
			break;
		}
		owner := &bodies.sets.memory[owner_index];
		status := body_set_arena_move_left(owner, &bodies.inactive_arena, cursor);
		if status != .Ok
		{
			return status;
		}
		cursor += owner.count;
		previous_source_start = owner_source_start;
	}
	if cursor > bodies.inactive_arena.capacity - capacity
	{
		return .Capacity_Missing;
	}
	return body_set_bind_arena_range(target, &bodies.inactive_arena, cursor, capacity);
}

bodies_ensure_handle_capacity :: proc (bodies: ^Bodies, capacity: int) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || capacity <= 0
	{
		return .Invalid_Argument;
	}
	if capacity <= int(bodies.handle_to_location.length)
	{
		return .Ok;
	}
	old_length := int(bodies.handle_to_location.length);
	new_locations, locations_status := util.buffer_pool_take_at_least(bodies.pool, Body_Memory_Location, capacity);
	if locations_status != .Ok
	{
		return physics_memory_status(locations_status);
	}
	new_available_ids, ids_status := util.buffer_pool_take_at_least(bodies.pool, i32, capacity);
	if ids_status != .Ok
	{
		physics_return_buffer(bodies.pool, &new_locations);
		return physics_memory_status(ids_status);
	}
	for index in 0 ..< new_locations.length
	{
		new_locations.memory[index] = {-1, -1};
	}
	copy_status := util.buffer_copy(
		util.buffer_view(bodies.handle_to_location), 0, util.buffer_view(new_locations), 0, old_length,
	);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(bodies.handle_pool.available_ids), 0,
			util.buffer_view(new_available_ids), 0,
			bodies.handle_pool.available_id_count,
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(bodies.pool, &new_available_ids);
		physics_return_buffer(bodies.pool, &new_locations);
		return physics_memory_status(copy_status);
	}
	if bodies.solver != nil
	{
		solver_status := solver_ensure_body_capacity(bodies.solver, int(new_locations.length));
		if solver_status != .Ok
		{
			physics_return_buffer(bodies.pool, &new_available_ids);
			physics_return_buffer(bodies.pool, &new_locations);
			return solver_status;
		}
	}
	physics_return_buffer(bodies.pool, &bodies.handle_pool.available_ids);
	physics_return_buffer(bodies.pool, &bodies.handle_to_location);
	bodies.handle_to_location = new_locations;
	bodies.handle_pool.available_ids = new_available_ids;
	return .Ok;
}

bodies_ensure_set_capacity :: proc (
	bodies: ^Bodies, set_capacity: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || set_capacity <= 0
	{
		return .Invalid_Argument;
	}
	if set_capacity <= int(bodies.sets.length)
	{
		return .Ok;
	}
	new_sets, take_status := util.buffer_pool_take_at_least(
		bodies.pool, Body_Set, set_capacity,
	);
	if take_status != .Ok
	{
		return physics_memory_status(take_status);
	}
	_ = util.buffer_clear(new_sets, 0, int(new_sets.length));
	copy_status := util.buffer_copy(
		util.buffer_view(bodies.sets), 0, util.buffer_view(new_sets), 0,
		int(bodies.sets.length),
	);
	if copy_status != .Ok
	{
		physics_return_buffer(bodies.pool, &new_sets);
		return physics_memory_status(copy_status);
	}
	for set_index in int(bodies.sets.length) ..< int(new_sets.length)
	{
		status := body_set_bind_arena_range(
			&new_sets.memory[set_index], &bodies.inactive_arena, 0, 0,
		);
		if status != .Ok
		{
			physics_return_buffer(bodies.pool, &new_sets);
			return status;
		}
	}
	physics_return_buffer(bodies.pool, &bodies.sets);
	bodies.sets = new_sets;
	return .Ok;
}

bodies_ensure_inactive_arena_capacity :: proc (
	bodies: ^Bodies, capacity: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || capacity <= 0
	{
		return .Invalid_Argument;
	}
	if capacity <= bodies.inactive_arena.capacity
	{
		return .Ok;
	}
	new_arena: Body_Set_Arena;
	status := body_set_arena_initialize(&new_arena, capacity, bodies.pool);
	if status != .Ok
	{
		return status;
	}
	cursor := 0;
	for set_index in 1 ..< bodies.sets.length
	{
		set := &bodies.sets.memory[set_index];
		if cursor > new_arena.capacity - set.count
		{
			_ = body_set_arena_dispose(&new_arena, bodies.pool);
			return .Capacity_Missing;
		}
		for body_index in 0 ..< set.count
		{
			target_index := cursor + body_index;
			new_arena.dynamics_state.memory[target_index] =
			set.dynamics_state.memory[body_index];
			new_arena.index_to_handle.memory[target_index] =
			set.index_to_handle.memory[body_index];
			new_arena.collidables.memory[target_index] =
			set.collidables.memory[body_index];
			new_arena.activity.memory[target_index] =
			set.activity.memory[body_index];
			new_arena.constraints.memory[target_index] =
			set.constraints.memory[body_index];
		}
		status = body_set_bind_arena_range(
			set, &new_arena, cursor, set.count,
		);
		if status != .Ok
		{
			_ = body_set_arena_dispose(&new_arena, bodies.pool);
			return status;
		}
		cursor += set.count;
	}
	old_arena := bodies.inactive_arena;
	bodies.inactive_arena = new_arena;
	_ = body_set_arena_dispose(&old_arena, bodies.pool);
	return .Ok;
}

bodies_ensure_capacity :: proc (
	bodies: ^Bodies, body_capacity, set_capacity,
	minimum_constraint_capacity_per_body: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || body_capacity <= 0 ||
	set_capacity <= 0 || minimum_constraint_capacity_per_body <= 0
	{
		return .Invalid_Argument;
	}
	status := bodies_ensure_set_capacity(bodies, set_capacity);
	if status != .Ok
	{
		return status;
	}
	status = bodies_ensure_handle_capacity(bodies, body_capacity);
	if status != .Ok
	{
		return status;
	}
	status = body_set_ensure_capacity(
		&bodies.sets.memory[BODIES_ACTIVE_SET_INDEX], body_capacity, bodies.pool,
	);
	if status != .Ok
	{
		return status;
	}
	status = bodies_ensure_inactive_arena_capacity(bodies, body_capacity);
	if status != .Ok
	{
		return status;
	}
	for set_index in 0 ..< bodies.sets.length
	{
		status = body_set_ensure_constraint_capacities(
			&bodies.sets.memory[set_index],
			minimum_constraint_capacity_per_body, bodies.pool,
		);
		if status != .Ok
		{
			return status;
		}
	}
	bodies.minimum_constraint_capacity_per_body =
	max(
		bodies.minimum_constraint_capacity_per_body,
		minimum_constraint_capacity_per_body,
	);
	bodies.initial_set_capacity = max(bodies.initial_set_capacity, body_capacity);
	return .Ok;
}

bodies_rebuild_inactive_arena :: proc (
	bodies: ^Bodies, capacity: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || capacity <= 0
	{
		return .Invalid_Argument;
	}
	total_count := 0;
	for set_index in 1 ..< bodies.sets.length
	{
		total_count += bodies.sets.memory[set_index].count;
	}
	target := max(capacity, total_count);
	if target == bodies.inactive_arena.capacity
	{
		return .Ok;
	}
	new_arena: Body_Set_Arena;
	status := body_set_arena_initialize(&new_arena, target, bodies.pool);
	if status != .Ok
	{
		return status;
	}
	cursor := 0;
	for set_index in 1 ..< bodies.sets.length
	{
		set := &bodies.sets.memory[set_index];
		for body_index in 0 ..< set.count
		{
			target_index := cursor + body_index;
			new_arena.dynamics_state.memory[target_index] =
			set.dynamics_state.memory[body_index];
			new_arena.index_to_handle.memory[target_index] =
			set.index_to_handle.memory[body_index];
			new_arena.collidables.memory[target_index] =
			set.collidables.memory[body_index];
			new_arena.activity.memory[target_index] =
			set.activity.memory[body_index];
			new_arena.constraints.memory[target_index] =
			set.constraints.memory[body_index];
		}
		cursor += set.count;
	}
	cursor = 0;
	for set_index in 1 ..< bodies.sets.length
	{
		set := &bodies.sets.memory[set_index];
		status = body_set_bind_arena_range(
			set, &new_arena, cursor, set.count,
		);
		if status != .Ok
		{
			_ = body_set_arena_dispose(&new_arena, bodies.pool);
			return status;
		}
		cursor += set.count;
	}
	old_arena := bodies.inactive_arena;
	bodies.inactive_arena = new_arena;
	_ = body_set_arena_dispose(&old_arena, bodies.pool);
	return .Ok;
}

bodies_resize :: proc (
	bodies: ^Bodies, body_capacity, set_capacity,
	minimum_constraint_capacity_per_body: int,
) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || body_capacity <= 0 ||
	set_capacity <= 0 || minimum_constraint_capacity_per_body <= 0
	{
		return .Invalid_Argument;
	}
	status := bodies_ensure_capacity(
		bodies, body_capacity, set_capacity,
		minimum_constraint_capacity_per_body,
	);
	if status != .Ok
	{
		return status;
	}
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	status = body_set_resize_capacity(
		active, body_capacity, minimum_constraint_capacity_per_body,
		bodies.pool,
	);
	if status != .Ok
	{
		return status;
	}
	status = bodies_rebuild_inactive_arena(bodies, body_capacity);
	if status != .Ok
	{
		return status;
	}
	required_handle_capacity := max(
		body_capacity, int(bodies.handle_pool.next_index),
	);
	status = physics_resize_buffer_capacity(
		bodies.pool, &bodies.handle_to_location,
		required_handle_capacity, int(bodies.handle_pool.next_index),
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		bodies.pool, &bodies.handle_pool.available_ids,
		max(required_handle_capacity, bodies.handle_pool.available_id_count),
		bodies.handle_pool.available_id_count,
	);
	if status != .Ok
	{
		return status;
	}
	for index in bodies.handle_pool.next_index ..< bodies.handle_to_location.length
	{
		bodies.handle_to_location.memory[index] = {-1, -1};
	}
	required_set_capacity := max(set_capacity, 1);
	for set_index in required_set_capacity ..< int(bodies.sets.length)
	{
		if bodies.sets.memory[set_index].count > 0
		{
			required_set_capacity = set_index + 1;
		}
	}
	status = physics_resize_buffer_capacity(
		bodies.pool, &bodies.sets, required_set_capacity,
		required_set_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	for set_index in required_set_capacity ..< int(bodies.sets.length)
	{
		bodies.sets.memory[set_index] = {};
		status = body_set_bind_arena_range(
			&bodies.sets.memory[set_index], &bodies.inactive_arena, 0, 0,
		);
		if status != .Ok
		{
			return status;
		}
	}
	bodies.minimum_constraint_capacity_per_body =
	minimum_constraint_capacity_per_body;
	bodies.initial_set_capacity = body_capacity;
	return .Ok;
}

bodies_resolve :: proc "contextless" (bodies: ^Bodies, handle: Body_Handle) -> (Body_Memory_Location, Physics_Status)
{
	if bodies == nil ||
	bodies.state != .Allocated ||
	handle.value < 0 ||
	int(handle.value) >= int(bodies.handle_to_location.length)
	{
		return {}, .Not_Found;
	}
	location := bodies.handle_to_location.memory[handle.value];
	if location.set_index < 0 || int(location.set_index) >= int(bodies.sets.length)
	{
		return {}, .Not_Found;
	}
	set := &bodies.sets.memory[location.set_index];
	if set.state != .Allocated || location.index < 0 || int(location.index) >= set.count ||
	set.index_to_handle.memory[location.index].value != handle.value
	{
		return {}, .Not_Found;
	}
	return location, .Ok;
}

bodies_reference_state :: proc "contextless" (bodies: ^Bodies, handle: Body_Handle) -> Reference_State
{
	_, status := bodies_resolve(bodies, handle);
	if status == .Ok
	{
		return .Present;
	}
	return .Missing;
}

bodies_add :: proc (bodies: ^Bodies, description: ^Body_Description) -> (Body_Handle, Physics_Status)
{
	if bodies == nil || bodies.state != .Allocated
	{
		return body_handle_invalid(), .Disposed;
	}
	validation := body_description_validate(description);
	if validation != .Ok
	{
		return body_handle_invalid(), validation;
	}
	shape_retained := Reference_State.Missing;
	if typed_index_state(description.collidable.shape) == .Present
	{
		if bodies.shapes == nil
		{
			return body_handle_invalid(), .Invalid_Argument;
		}
		retain_status := shape_registry_retain(bodies.shapes, description.collidable.shape);
		if retain_status != .Ok
		{
			return body_handle_invalid(), retain_status;
		}
		shape_retained = .Present;
	}
	handle_index, id_status := util.id_pool_take(&bodies.handle_pool);
	if id_status != .Ok
	{
		if shape_retained == .Present
		{
			_ = shape_registry_release(bodies.shapes, description.collidable.shape);
		}
		return body_handle_invalid(), physics_memory_status(id_status);
	}
	if int(handle_index) >= int(bodies.handle_to_location.length)
	{
		capacity_status := bodies_ensure_handle_capacity(
			bodies,
			max(int(bodies.handle_to_location.length) * 2, int(handle_index) + 1)
		);
		if capacity_status != .Ok
		{
			_ = util.id_pool_return(&bodies.handle_pool, handle_index, bodies.pool);
			if shape_retained == .Present
			{
				_ = shape_registry_release(bodies.shapes, description.collidable.shape);
			}
			return body_handle_invalid(), capacity_status;
		}
	}
	handle := Body_Handle{handle_index};
	set := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	index, add_status := body_set_add(
		set,
		description,
		handle,
		bodies.minimum_constraint_capacity_per_body,
		bodies.pool
	);
	if add_status != .Ok
	{
		_ = util.id_pool_return(&bodies.handle_pool, handle_index, bodies.pool);
		if shape_retained == .Present
		{
			_ = shape_registry_release(bodies.shapes, description.collidable.shape);
		}
		return body_handle_invalid(), add_status;
	}
	bodies.handle_to_location.memory[handle_index] = {BODIES_ACTIVE_SET_INDEX, i32(index)};
	if bodies.triggers != nil
	{
		trigger_changed(bodies.triggers, .Body, handle.value, .Modified);
	}
	return handle, .Ok;
}

bodies_remove :: proc (bodies: ^Bodies, handle: Body_Handle) -> Physics_Status
{
	location, resolve_status := bodies_resolve(bodies, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	set := &bodies.sets.memory[location.set_index];
	constraints := &set.constraints.memory[location.index];
	if constraints.count > 0 && bodies.solver == nil
	{
		return .Invalid_Argument;
	}
	for constraints.count > 0
	{
		constraint_handle := constraints.span.memory[constraints.count - 1].connecting_constraint_handle;
		remove_status := solver_remove(bodies.solver, constraint_handle);
		if remove_status != .Ok
		{
			return remove_status;
		}
	}
	removed_shape := set.collidables.memory[location.index].shape;
	removed_handle, moved_handle, removed_constraints, moved, remove_status := body_set_remove_at(
		set,
		int(location.index)
	);
	if remove_status != .Ok
	{
		return remove_status;
	}
	if removed_constraints.span.memory != nil
	{
		_ = util.quick_list_dispose(&removed_constraints, bodies.pool);
	}
	if typed_index_state(removed_shape) == .Present && bodies.shapes != nil
	{
		_ = shape_registry_release(bodies.shapes, removed_shape);
	}
	if moved == .Present
	{
		if bodies.solver != nil
		{
			update_status := solver_update_for_body_memory_move(bodies.solver, set, set.count, int(location.index));
			if update_status != .Ok
			{
				return update_status;
			}
		}
		bodies.handle_to_location.memory[moved_handle.value] = {location.set_index, location.index};
	}
	bodies.handle_to_location.memory[removed_handle.value] = {-1, -1};
	if bodies.triggers != nil
	{
		trigger_changed(bodies.triggers, .Body, removed_handle.value, .Removed);
	}
	if bodies.body_control != nil
	{
		body_control_retire(bodies.body_control, removed_handle);
	}
	if bodies.restitution != nil
	{
		reference: Collidable_Reference;
		reference, _ = collidable_reference_body(.Dynamic, removed_handle);
		restitution_storage_retire(bodies.restitution, reference);
	}
	return physics_memory_status(util.id_pool_return(&bodies.handle_pool, removed_handle.value, bodies.pool));
}

bodies_get_description :: proc "contextless" (
	bodies: ^Bodies,
	handle: Body_Handle
) -> (Body_Description, Physics_Status)
{
	location, resolve_status := bodies_resolve(bodies, handle);
	if resolve_status != .Ok
	{
		return {}, resolve_status;
	}
	return body_set_get_description(&bodies.sets.memory[location.set_index], int(location.index));
}

bodies_apply_description :: proc (
	bodies: ^Bodies, handle: Body_Handle, description: ^Body_Description,
) -> Physics_Status
{
	location, resolve_status := bodies_resolve(bodies, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	validation := body_description_validate(description);
	if validation != .Ok
	{
		return validation;
	}
	set := &bodies.sets.memory[location.set_index];
	old_shape := set.collidables.memory[location.index].shape;
	broad_phase_index := set.collidables.memory[location.index].broad_phase_index;
	new_shape := description.collidable.shape;
	previous_mobility := body_inertia_mobility(set.dynamics_state.memory[location.index].inertia.local);
	current_mobility := body_inertia_mobility(description.local_inertia);
	mobility_change: Solver_Body_Mobility_Change;
	if bodies.solver != nil && previous_mobility != current_mobility
	{
		mobility_status := solver_prepare_body_mobility_change(
			bodies.solver, handle, int(location.index),
			previous_mobility, current_mobility, &mobility_change,
		);
		if mobility_status != .Ok
		{
			return mobility_status;
		}
	}
	if old_shape.packed != new_shape.packed && typed_index_state(new_shape) == .Present
	{
		if bodies.shapes == nil
		{
			if mobility_change.state == .Prepared
			{
				solver_abort_body_mobility_change(bodies.solver, &mobility_change);
			}
			return .Invalid_Argument;
		}
		retain_status := shape_registry_retain(bodies.shapes, new_shape);
		if retain_status != .Ok
		{
			if mobility_change.state == .Prepared
			{
				solver_abort_body_mobility_change(bodies.solver, &mobility_change);
			}
			return retain_status;
		}
	}
	apply_status := body_set_apply_description(set, int(location.index), description);
	if apply_status != .Ok
	{
		if mobility_change.state == .Prepared
		{
			solver_abort_body_mobility_change(bodies.solver, &mobility_change);
		}
		if old_shape.packed != new_shape.packed && typed_index_state(new_shape) == .Present
		{
			_ = shape_registry_release(bodies.shapes, new_shape);
		}
		return apply_status;
	}
	set.collidables.memory[location.index].broad_phase_index = broad_phase_index;
	if mobility_change.state == .Prepared
	{
		solver_commit_body_mobility_change(bodies.solver, &mobility_change);
	}
	if old_shape.packed != new_shape.packed && typed_index_state(old_shape) == .Present
	{
		_ = shape_registry_release(bodies.shapes, old_shape);
	}
	if bodies.body_control != nil
	{
		body_control_body_applied(bodies.body_control, handle, previous_mobility, current_mobility);
	}
	if bodies.triggers != nil
	{
		trigger_changed(bodies.triggers, .Body, handle.value, .Modified);
	}
	return .Ok;
}

bodies_move :: proc (bodies: ^Bodies, handle: Body_Handle, target_set_index: int) -> Physics_Status
{
	location, resolve_status := bodies_resolve(bodies, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if target_set_index < 0 || target_set_index >= int(bodies.sets.length)
	{
		return .Invalid_Argument;
	}
	if target_set_index == int(location.set_index)
	{
		return .Ok;
	}
	source := &bodies.sets.memory[location.set_index];
	target := &bodies.sets.memory[target_set_index];
	if target.state != .Allocated && target_set_index == BODIES_ACTIVE_SET_INDEX
	{
		initialize_status := body_set_initialize(target, bodies.initial_set_capacity, bodies.pool);
		if initialize_status != .Ok
		{
			return initialize_status;
		}
	}
	if target.count >= body_set_capacity(target)
	{
		capacity_status := Physics_Status.Ok;
		if target_set_index == BODIES_ACTIVE_SET_INDEX
		{
			capacity_status = body_set_ensure_capacity(
				target, max(body_set_capacity(target) * 2, 1), bodies.pool,
			);
		}
		else
		{
			capacity_status = bodies_prepare_inactive_set_capacity(
				bodies, target_set_index, target.count + 1,
			);
		}
		if capacity_status != .Ok
		{
			return capacity_status;
		}
	}
	source_index := int(location.index);
	target_index := target.count;
	target.count += 1;
	target.dynamics_state.memory[target_index] = source.dynamics_state.memory[source_index];
	target.index_to_handle.memory[target_index] = handle;
	target.collidables.memory[target_index] = source.collidables.memory[source_index];
	target.activity.memory[target_index] = source.activity.memory[source_index];
	target.constraints.memory[target_index] = source.constraints.memory[source_index];
	_, moved_handle, _, moved, remove_status := body_set_remove_at(source, source_index);
	if remove_status != .Ok
	{
		return remove_status;
	}
	if moved == .Present
	{
		if location.set_index == BODIES_ACTIVE_SET_INDEX && bodies.solver != nil
		{
			update_status := solver_update_for_body_memory_move(bodies.solver, source, source.count, source_index);
			if update_status != .Ok
			{
				return update_status;
			}
		}
		bodies.handle_to_location.memory[moved_handle.value] = {location.set_index, location.index};
	}
	bodies.handle_to_location.memory[handle.value] = {i32(target_set_index), i32(target_index)};
	return .Ok;
}

bodies_set_local_inertia :: proc (bodies: ^Bodies, handle: Body_Handle, inertia: Body_Inertia) -> Physics_Status
{
	location, status := bodies_resolve(bodies, handle);
	if status != .Ok
	{
		return status;
	}
	state := &bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
	previous_mobility := body_inertia_mobility(state.inertia.local);
	current_mobility := body_inertia_mobility(inertia);
	mobility_change: Solver_Body_Mobility_Change;
	if bodies.solver != nil && previous_mobility != current_mobility
	{
		status = solver_prepare_body_mobility_change(
			bodies.solver, handle, int(location.index),
			previous_mobility, current_mobility, &mobility_change,
		);
		if status != .Ok
		{
			return status;
		}
	}
	state.inertia.local = inertia;
	state.inertia.world = {};
	if mobility_change.state == .Prepared
	{
		solver_commit_body_mobility_change(bodies.solver, &mobility_change);
	}
	return .Ok;
}

bodies_set_shape :: proc "contextless" (bodies: ^Bodies, handle: Body_Handle, shape: Typed_Index) -> Physics_Status
{
	location, status := bodies_resolve(bodies, handle);
	if status != .Ok
	{
		return status;
	}
	collidable := &bodies.sets.memory[location.set_index].collidables.memory[location.index];
	old_shape := collidable.shape;
	if old_shape.packed == shape.packed
	{
		return .Ok;
	}
	if typed_index_state(shape) == .Present
	{
		if bodies.shapes == nil
		{
			return .Invalid_Argument;
		}
		retain_status := shape_registry_retain(bodies.shapes, shape);
		if retain_status != .Ok
		{
			return retain_status;
		}
	}
	collidable.shape = shape;
	if typed_index_state(old_shape) == .Present
	{
		_ = shape_registry_release(bodies.shapes, old_shape);
	}
	if bodies.triggers != nil
	{
		trigger_changed(bodies.triggers, .Body, handle.value, .Modified);
	}
	return .Ok;
}

bodies_clear :: proc (bodies: ^Bodies) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || bodies.pool == nil
	{
		return .Disposed;
	}
	if bodies.triggers != nil
	{
		trigger_reset(bodies.triggers);
	}
	for set_index in 0 ..< bodies.sets.length
	{
		set := &bodies.sets.memory[set_index];
		for body_index in 0 ..< set.count
		{
			if bodies.shapes != nil
			{
				shape := set.collidables.memory[body_index].shape;
				if typed_index_state(shape) == .Present
				{
					status := shape_registry_release(bodies.shapes, shape);
					if status != .Ok
					{
						return status;
					}
				}
			}
			if set.constraints.memory[body_index].span.memory != nil
			{
				status := util.quick_list_dispose(
					&set.constraints.memory[body_index], bodies.pool,
				);
				if status != .Ok
				{
					return physics_collection_status(status);
				}
			}
		}
		if set.storage == .Owned
		{
			_ = util.buffer_clear(set.dynamics_state, 0, set.count);
			_ = util.buffer_clear(set.index_to_handle, 0, set.count);
			_ = util.buffer_clear(set.collidables, 0, set.count);
			_ = util.buffer_clear(set.activity, 0, set.count);
			_ = util.buffer_clear(set.constraints, 0, set.count);
			set.count = 0;
		}
		else
		{
			set.count = 0;
			status := body_set_bind_arena_range(
				set, &bodies.inactive_arena, 0, 0,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	_ = util.buffer_clear(
		bodies.inactive_arena.dynamics_state, 0,
		int(bodies.inactive_arena.dynamics_state.length),
	);
	_ = util.buffer_clear(
		bodies.inactive_arena.index_to_handle, 0,
		int(bodies.inactive_arena.index_to_handle.length),
	);
	_ = util.buffer_clear(
		bodies.inactive_arena.collidables, 0,
		int(bodies.inactive_arena.collidables.length),
	);
	_ = util.buffer_clear(
		bodies.inactive_arena.activity, 0,
		int(bodies.inactive_arena.activity.length),
	);
	_ = util.buffer_clear(
		bodies.inactive_arena.constraints, 0,
		int(bodies.inactive_arena.constraints.length),
	);
	for index in 0 ..< bodies.handle_to_location.length
	{
		bodies.handle_to_location.memory[index] = {-1, -1};
	}
	if bodies.restitution != nil
	{
		restitution_storage_clear_kind(bodies.restitution, .Dynamic);
	}
	if bodies.body_control != nil
	{
		body_control_clear_storage(bodies.body_control);
	}
	util.id_pool_clear(&bodies.handle_pool);
	return .Ok;
}

bodies_dispose :: proc (bodies: ^Bodies) -> Physics_Status
{
	if bodies == nil || bodies.state != .Allocated || bodies.pool == nil
	{
		return .Disposed;
	}
	pool := bodies.pool;
	for set_index in 0 ..< bodies.sets.length
	{
		set := &bodies.sets.memory[set_index];
		if set.state == .Allocated
		{
			if bodies.shapes != nil
			{
				for body_index in 0 ..< set.count
				{
					shape := set.collidables.memory[body_index].shape;
					if typed_index_state(shape) == .Present
					{
						_ = shape_registry_release(bodies.shapes, shape);
					}
				}
			}
			_ = body_set_dispose(set, pool);
		}
	}
	if bodies.inactive_arena.state == .Allocated
	{
		_ = body_set_arena_dispose(&bodies.inactive_arena, pool);
	}
	_ = util.id_pool_dispose(&bodies.handle_pool, pool);
	physics_return_buffer(pool, &bodies.handle_to_location);
	physics_return_buffer(pool, &bodies.sets);
	bodies^ = {};
	return .Ok;
}
