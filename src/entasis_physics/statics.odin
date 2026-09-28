// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Statics :: struct
{
	statics:         util.Buffer(Static),
	handle_to_index: util.Buffer(i32),
	index_to_handle: util.Buffer(Static_Handle),
	handle_pool:     util.Id_Pool,
	pool:            ^util.Buffer_Pool,
	shapes:          ^Shape_Registry,
	count:           int,
	state:           Body_Set_State,
	triggers: ^Trigger_System,
	restitution: ^Restitution_Storage,
}

statics_capacity :: proc "contextless" (statics: ^Statics) -> int
{
	if statics == nil || statics.state != .Allocated
	{
		return 0;
	}
	return min(
		int(statics.statics.length),
		min(
			int(statics.handle_to_index.length),
			int(statics.index_to_handle.length),
		),
	);
}

statics_bind_shape_registry :: proc "contextless" (statics: ^Statics, shapes: ^Shape_Registry) -> Physics_Status
{
	if statics == nil || statics.state != .Allocated || statics.count != 0 ||
	shapes == nil || shapes.state != .Allocated
	{
		return .Invalid_Argument;
	}
	statics.shapes = shapes;
	return .Ok;
}

statics_initialize :: proc (statics: ^Statics, capacity: int, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if statics == nil || pool == nil || capacity <= 0
	{
		return .Invalid_Argument;
	}
	static_buffer, static_status := util.buffer_pool_take_at_least(pool, Static, capacity);
	if static_status != .Ok
	{
		return physics_memory_status(static_status);
	}
	storage_capacity := int(static_buffer.length);
	handle_to_index, map_status := util.buffer_pool_take_at_least(
		pool, i32, storage_capacity,
	);
	if map_status != .Ok
	{
		physics_return_buffer(pool, &static_buffer);
		return physics_memory_status(map_status);
	}
	index_to_handle, reverse_status := util.buffer_pool_take_at_least(
		pool, Static_Handle, storage_capacity,
	);
	if reverse_status != .Ok
	{
		physics_return_buffer(pool, &handle_to_index);
		physics_return_buffer(pool, &static_buffer);
		return physics_memory_status(reverse_status);
	}
	_ = util.buffer_clear(static_buffer, 0, int(static_buffer.length));
	_ = util.buffer_clear(index_to_handle, 0, int(index_to_handle.length));
	for index in 0 ..< handle_to_index.length
	{
		handle_to_index.memory[index] = -1;
	}
	handle_pool: util.Id_Pool;
	id_status := util.id_pool_initialize(&handle_pool, storage_capacity, pool);
	if id_status != .Ok
	{
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &handle_to_index);
		physics_return_buffer(pool, &static_buffer);
		return physics_memory_status(id_status);
	}
	statics^ = {
		statics=static_buffer,
		handle_to_index=handle_to_index,
		index_to_handle=index_to_handle,
		handle_pool=handle_pool,
		pool=pool,
		state=.Allocated,
	};
	return .Ok;
}

statics_ensure_capacity :: proc (statics: ^Statics, capacity: int) -> Physics_Status
{
	if statics == nil || statics.state != .Allocated || capacity <= 0
	{
		return .Invalid_Argument;
	}
	if capacity <= statics_capacity(statics)
	{
		return .Ok;
	}
	old_length := int(statics.handle_to_index.length);
	new_statics, statics_status := util.buffer_pool_take_at_least(statics.pool, Static, capacity);
	if statics_status != .Ok
	{
		return physics_memory_status(statics_status);
	}
	storage_capacity := int(new_statics.length);
	new_reverse, reverse_status := util.buffer_pool_take_at_least(
		statics.pool, Static_Handle, storage_capacity,
	);
	if reverse_status != .Ok
	{
		physics_return_buffer(statics.pool, &new_statics);
		return physics_memory_status(reverse_status);
	}
	new_map, map_status := util.buffer_pool_take_at_least(
		statics.pool, i32, storage_capacity,
	);
	if map_status != .Ok
	{
		physics_return_buffer(statics.pool, &new_reverse);
		physics_return_buffer(statics.pool, &new_statics);
		return physics_memory_status(map_status);
	}
	new_available_ids, ids_status := util.buffer_pool_take_at_least(
		statics.pool, i32, storage_capacity,
	);
	if ids_status != .Ok
	{
		physics_return_buffer(statics.pool, &new_map);
		physics_return_buffer(statics.pool, &new_reverse);
		physics_return_buffer(statics.pool, &new_statics);
		return physics_memory_status(ids_status);
	}
	_ = util.buffer_clear(new_statics, 0, int(new_statics.length));
	_ = util.buffer_clear(new_reverse, 0, int(new_reverse.length));
	for index in 0 ..< new_map.length
	{
		new_map.memory[index] = -1;
	}
	copy_status := util.buffer_copy(
		util.buffer_view(statics.statics),
		0,
		util.buffer_view(new_statics),
		0,
		statics.count
	);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(statics.index_to_handle),
			0,
			util.buffer_view(new_reverse),
			0,
			statics.count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(statics.handle_to_index),
			0,
			util.buffer_view(new_map),
			0,
			old_length
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(statics.handle_pool.available_ids), 0,
			util.buffer_view(new_available_ids), 0,
			statics.handle_pool.available_id_count,
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(statics.pool, &new_available_ids);
		physics_return_buffer(statics.pool, &new_map);
		physics_return_buffer(statics.pool, &new_reverse);
		physics_return_buffer(statics.pool, &new_statics);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(statics.pool, &statics.handle_pool.available_ids);
	physics_return_buffer(statics.pool, &statics.handle_to_index);
	physics_return_buffer(statics.pool, &statics.index_to_handle);
	physics_return_buffer(statics.pool, &statics.statics);
	statics.statics = new_statics;
	statics.index_to_handle = new_reverse;
	statics.handle_to_index = new_map;
	statics.handle_pool.available_ids = new_available_ids;
	return .Ok;
}

statics_resize :: proc (statics: ^Statics, capacity: int) -> Physics_Status
{
	if statics == nil || statics.state != .Allocated || capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := statics_ensure_capacity(statics, capacity);
	if status != .Ok
	{
		return status;
	}
	required_capacity := max(
		capacity,
		max(statics.count, int(statics.handle_pool.next_index)),
	);
	status = physics_resize_buffer_capacity(
		statics.pool, &statics.statics, required_capacity, statics.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		statics.pool, &statics.index_to_handle,
		required_capacity, statics.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		statics.pool, &statics.handle_to_index,
		required_capacity, int(statics.handle_pool.next_index),
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		statics.pool, &statics.handle_pool.available_ids,
		max(required_capacity, statics.handle_pool.available_id_count),
		statics.handle_pool.available_id_count,
	);
	if status != .Ok
	{
		return status;
	}
	for index in statics.handle_pool.next_index ..< statics.handle_to_index.length
	{
		statics.handle_to_index.memory[index] = -1;
	}
	return .Ok;
}

statics_resolve :: proc "contextless" (statics: ^Statics, handle: Static_Handle) -> (int, Physics_Status)
{
	if statics == nil ||
	statics.state != .Allocated ||
	handle.value < 0 ||
	int(handle.value) >= int(statics.handle_to_index.length)
	{
		return -1, .Not_Found;
	}
	index := int(statics.handle_to_index.memory[handle.value]);
	if index < 0 || index >= statics.count || statics.index_to_handle.memory[index].value != handle.value
	{
		return -1, .Not_Found;
	}
	return index, .Ok;
}

statics_reference_state :: proc "contextless" (statics: ^Statics, handle: Static_Handle) -> Reference_State
{
	_, status := statics_resolve(statics, handle);
	if status == .Ok
	{
		return .Present;
	}
	return .Missing;
}

statics_add :: proc (statics: ^Statics, description: ^Static_Description) -> (Static_Handle, Physics_Status)
{
	if statics == nil || statics.state != .Allocated
	{
		return static_handle_invalid(), .Disposed;
	}
	validation := static_description_validate(description);
	if validation != .Ok
	{
		return static_handle_invalid(), validation;
	}
	if statics.shapes == nil
	{
		return static_handle_invalid(), .Invalid_Argument;
	}
	retain_status := shape_registry_retain(statics.shapes, description.shape);
	if retain_status != .Ok
	{
		return static_handle_invalid(), retain_status;
	}
	handle_index, id_status := util.id_pool_take(&statics.handle_pool);
	if id_status != .Ok
	{
		_ = shape_registry_release(statics.shapes, description.shape);
		return static_handle_invalid(), physics_memory_status(id_status);
	}
	if statics.count >= statics_capacity(statics) || int(handle_index) >= int(statics.handle_to_index.length)
	{
		capacity_status := statics_ensure_capacity(
			statics, max(statics_capacity(statics) * 2, int(handle_index) + 1),
		);
		if capacity_status != .Ok
		{
			_ = util.id_pool_return(&statics.handle_pool, handle_index, statics.pool);
			_ = shape_registry_release(statics.shapes, description.shape);
			return static_handle_invalid(), capacity_status;
		}
	}
	index := statics.count;
	statics.count += 1;
	handle := Static_Handle{handle_index};
	statics.handle_to_index.memory[handle_index] = i32(index);
	statics.index_to_handle.memory[index] = handle;
	statics.statics.memory[index] = {
		pose=description.pose,
		shape=description.shape,
		continuity=description.continuity,
		broad_phase_index=-1,
	};
	if statics.triggers != nil
	{
		trigger_changed(statics.triggers, .Static, handle.value, .Modified);
	}
	return handle, .Ok;
}

statics_remove :: proc (statics: ^Statics, handle: Static_Handle) -> Physics_Status
{
	index, resolve_status := statics_resolve(statics, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	removed_shape := statics.statics.memory[index].shape;
	statics.count -= 1;
	if index < statics.count
	{
		statics.statics.memory[index] = statics.statics.memory[statics.count];
		moved_handle := statics.index_to_handle.memory[statics.count];
		statics.index_to_handle.memory[index] = moved_handle;
		statics.handle_to_index.memory[moved_handle.value] = i32(index);
	}
	statics.statics.memory[statics.count] = {};
	statics.index_to_handle.memory[statics.count] = {};
	statics.handle_to_index.memory[handle.value] = -1;
	if statics.triggers != nil
	{
		trigger_changed(statics.triggers, .Static, handle.value, .Removed);
	}
	if statics.restitution != nil
	{
		reference: Collidable_Reference;
		reference, _ = collidable_reference_static(handle);
		restitution_storage_retire(statics.restitution, reference);
	}
	_ = shape_registry_release(statics.shapes, removed_shape);
	return physics_memory_status(util.id_pool_return(&statics.handle_pool, handle.value, statics.pool));
}

statics_get_description :: proc "contextless" (
	statics: ^Statics,
	handle: Static_Handle
) -> (Static_Description, Physics_Status)
{
	index, resolve_status := statics_resolve(statics, handle);
	if resolve_status != .Ok
	{
		return {}, resolve_status;
	}
	static := &statics.statics.memory[index];
	return {pose=static.pose, shape=static.shape, continuity=static.continuity}, .Ok;
}

statics_apply_description :: proc "contextless" (
	statics: ^Statics, handle: Static_Handle, description: ^Static_Description,
) -> Physics_Status
{
	validation := static_description_validate(description);
	if validation != .Ok
	{
		return validation;
	}
	index, resolve_status := statics_resolve(statics, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	old_shape := statics.statics.memory[index].shape;
	if old_shape.packed != description.shape.packed
	{
		if statics.shapes == nil
		{
			return .Invalid_Argument;
		}
		retain_status := shape_registry_retain(statics.shapes, description.shape);
		if retain_status != .Ok
		{
			return retain_status;
		}
	}
	broad_phase_index := statics.statics.memory[index].broad_phase_index;
	statics.statics.memory[index] = {
		pose=description.pose,
		shape=description.shape,
		continuity=description.continuity,
		broad_phase_index=broad_phase_index,
	};
	if old_shape.packed != description.shape.packed
	{
		_ = shape_registry_release(statics.shapes, old_shape);
	}
	if statics.triggers != nil
	{
		trigger_changed(statics.triggers, .Static, handle.value, .Modified);
	}
	return .Ok;
}

statics_set_shape :: proc "contextless" (statics: ^Statics, handle: Static_Handle, shape: Typed_Index) -> Physics_Status
{
	if typed_index_state(shape) != .Present
	{
		return .Invalid_Description;
	}
	index, resolve_status := statics_resolve(statics, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	old_shape := statics.statics.memory[index].shape;
	if old_shape.packed == shape.packed
	{
		return .Ok;
	}
	if statics.shapes == nil
	{
		return .Invalid_Argument;
	}
	retain_status := shape_registry_retain(statics.shapes, shape);
	if retain_status != .Ok
	{
		return retain_status;
	}
	statics.statics.memory[index].shape = shape;
	_ = shape_registry_release(statics.shapes, old_shape);
	if statics.triggers != nil
	{
		trigger_changed(statics.triggers, .Static, handle.value, .Modified);
	}
	return .Ok;
}

statics_clear :: proc (statics: ^Statics) -> Physics_Status
{
	if statics == nil || statics.state != .Allocated
	{
		return .Disposed;
	}
	if statics.shapes != nil
	{
		for index in 0 ..< statics.count
		{
			status := shape_registry_release(
				statics.shapes, statics.statics.memory[index].shape,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	_ = util.buffer_clear(statics.statics, 0, statics.count);
	_ = util.buffer_clear(statics.index_to_handle, 0, statics.count);
	if statics.triggers != nil
	{
		trigger_reset(statics.triggers);
	}
	if statics.restitution != nil
	{
		restitution_storage_clear_kind(statics.restitution, .Static);
	}
	for index in 0 ..< statics.handle_to_index.length
	{
		statics.handle_to_index.memory[index] = -1;
	}
	statics.count = 0;
	util.id_pool_clear(&statics.handle_pool);
	return .Ok;
}

statics_dispose :: proc (statics: ^Statics) -> Physics_Status
{
	if statics == nil || statics.state != .Allocated || statics.pool == nil
	{
		return .Disposed;
	}
	pool := statics.pool;
	if statics.shapes != nil
	{
		for index in 0 ..< statics.count
		{
			_ = shape_registry_release(statics.shapes, statics.statics.memory[index].shape);
		}
	}
	_ = util.id_pool_dispose(&statics.handle_pool, pool);
	physics_return_buffer(pool, &statics.index_to_handle);
	physics_return_buffer(pool, &statics.handle_to_index);
	physics_return_buffer(pool, &statics.statics);
	statics^ = {};
	return .Ok;
}
