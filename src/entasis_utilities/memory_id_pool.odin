// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Id_Pool :: struct
{
	next_index:         i32,
	available_id_count: int,
	available_ids:      Buffer(i32),
}

id_pool_initialize :: proc (id_pool: ^Id_Pool, initial_capacity: int, pool: ^Buffer_Pool) -> Memory_Status
{
	if id_pool == nil || initial_capacity < 0
	{
		return .Invalid_Count;
	}
	available_ids, status := buffer_pool_take_at_least(pool, i32, initial_capacity);
	if status != .Ok
	{
		return status;
	}
	id_pool^ = Id_Pool{available_ids=available_ids};
	return .Ok;
}

id_pool_highest_possibly_claimed_id :: proc (id_pool: ^Id_Pool) -> i32
{
	return id_pool.next_index - 1;
}

id_pool_take :: proc (id_pool: ^Id_Pool) -> (id: i32, status: Memory_Status)
{
	if id_pool == nil || buffer_allocation_state(id_pool.available_ids) != .Allocated
	{
		return 0, .Invalid_Buffer;
	}
	if id_pool.available_id_count > 0
	{
		id_pool.available_id_count -= 1;
		return id_pool.available_ids.memory[id_pool.available_id_count], .Ok;
	}
	id = id_pool.next_index;
	id_pool.next_index += 1;
	return id, .Ok;
}

id_pool_return_unsafely :: proc(id_pool: ^Id_Pool, id: i32) -> Memory_Status
{
	if id_pool == nil || buffer_allocation_state(id_pool.available_ids) != .Allocated ||
		id_pool.available_id_count < 0 ||
		id_pool.available_id_count >= int(id_pool.available_ids.length)
	{
		return .Invalid_Buffer;
	}
	id_pool.available_ids.memory[id_pool.available_id_count] = id;
	id_pool.available_id_count += 1;
	return .Ok;
}

id_pool_return :: proc(
	id_pool: ^Id_Pool, id: i32, pool: ^Buffer_Pool,
) -> Memory_Status
{
	if id_pool == nil || pool == nil ||
		buffer_allocation_state(id_pool.available_ids) != .Allocated
	{
		return .Invalid_Buffer;
	}
	if id_pool.available_id_count == int(id_pool.available_ids.length)
	{
		if id_pool.available_id_count > max(int) / 2
		{
			return .Overflow;
		}
		target_count := max(
			id_pool.available_id_count * 2, int(id_pool.available_ids.length),
		);
		resize_status := id_pool_resize(id_pool, max(target_count, 1), pool);
		if resize_status != .Ok
		{
			return resize_status;
		}
	}
	return id_pool_return_unsafely(id_pool, id);
}

id_pool_clear :: proc (id_pool: ^Id_Pool)
{
	id_pool.next_index = 0;
	id_pool.available_id_count = 0;
}

id_pool_resize :: proc (id_pool: ^Id_Pool, count: int, pool: ^Buffer_Pool) -> Memory_Status
{
	if id_pool == nil || count < 0
	{
		return .Invalid_Count;
	}
	target_count := max(count, id_pool.available_id_count);
	if id_pool.available_ids.memory == nil
	{
		return id_pool_initialize(id_pool, target_count, pool);
	}
	return buffer_pool_resize_to_at_least(pool, &id_pool.available_ids, target_count, id_pool.available_id_count);
}

id_pool_ensure_capacity :: proc (id_pool: ^Id_Pool, count: int, pool: ^Buffer_Pool) -> Memory_Status
{
	if id_pool == nil || count < 0
	{
		return .Invalid_Count;
	}
	if id_pool.available_ids.memory == nil
	{
		return id_pool_resize(id_pool, count, pool);
	}
	capacity, capacity_status := buffer_capacity(id_pool.available_ids);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	if capacity < count
	{
		return id_pool_resize(id_pool, count, pool);
	}
	return .Ok;
}

id_pool_compact :: proc (id_pool: ^Id_Pool, minimum_count: int, pool: ^Buffer_Pool) -> Memory_Status
{
	if id_pool == nil || minimum_count < 0 || id_pool.available_ids.memory == nil
	{
		return .Invalid_Count;
	}
	target_count := max(minimum_count, id_pool.available_id_count);
	target_capacity, status := buffer_pool_capacity_for_count(i32, target_count);
	if status != .Ok
	{
		return status;
	}
	current_capacity, capacity_status := buffer_capacity(id_pool.available_ids);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	if target_capacity >= current_capacity
	{
		return status;
	}
	new_buffer, take_status := buffer_pool_take_at_least(pool, i32, target_count);
	if take_status != .Ok
	{
		return take_status;
	}
	copy_status := buffer_copy(
		buffer_view(id_pool.available_ids),
		0,
		buffer_view(new_buffer),
		0,
		id_pool.available_id_count
	);
	if copy_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_buffer);
		return copy_status;
	}
	return_status := buffer_pool_return(pool, &id_pool.available_ids);
	if return_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_buffer);
		return return_status;
	}
	id_pool.available_ids = new_buffer;
	return .Ok;
}

id_pool_dispose :: proc (id_pool: ^Id_Pool, pool: ^Buffer_Pool) -> Memory_Status
{
	if id_pool == nil
	{
		return .Invalid_Buffer;
	}
	status := buffer_pool_return(pool, &id_pool.available_ids);
	if status == .Ok
	{
		id_pool^ = {};
	}
	return status;
}
