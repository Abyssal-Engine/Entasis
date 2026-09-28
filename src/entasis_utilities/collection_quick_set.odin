// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Quick_Set :: struct($T: typeid)
{
	count:              int,
	table_mask:         int,
	table_power_offset: int,
	table:              Buffer(i32),
	elements:           Buffer(T),
	comparer:           Hash_Equal_Procs,
}

quick_set_initialize :: proc (
	set: ^Quick_Set($T), initial_capacity: int, pool: ^Buffer_Pool,
	comparer: Hash_Equal_Procs, table_power_offset: int = 2,
) -> Collection_Status
{
	if set == nil || pool == nil || initial_capacity <= 0 || comparer.hash == nil || comparer.equal == nil
	{
		return .Invalid_Argument;
	}
	elements, element_status := buffer_pool_take_at_least(pool, T, initial_capacity);
	if element_status != .Ok
	{
		return memory_to_collection_status(element_status);
	}
	table, table_status := buffer_pool_take_at_least(pool, i32, int(elements.length) << uint(table_power_offset));
	if table_status != .Ok
	{
		_ = buffer_pool_return(pool, &elements);
		return memory_to_collection_status(table_status);
	}
	_ = buffer_clear(table, 0, int(table.length));
	set^ = Quick_Set(T){
		table_mask=int(table.length) - 1,
		table_power_offset=table_power_offset,
		table=table,
		elements=elements,
		comparer=comparer,
	};
	return .Ok;
}

quick_set_get_table_indices :: proc (set: ^Quick_Set($T), element: ^T) -> (Presence_Status, int, int)
{
	if set == nil || element == nil
	{
		return .Missing, -1, -1;
	}
	table_index := int(u32(hash_rehash(set.comparer.hash(element)))) & set.table_mask;
	for
	{
		encoded_index := int(set.table.memory[table_index]);
		if encoded_index <= 0
		{
			return .Missing, table_index, -1;
		}
		element_index := encoded_index - 1;
		if set.comparer.equal(&set.elements.memory[element_index], element) == .Equal
		{
			return .Present, table_index, element_index;
		}
		table_index = (table_index + 1) & set.table_mask;
	}
}

quick_set_index_of :: proc (set: ^Quick_Set($T), element: ^T) -> int
{
	presence, _, index := quick_set_get_table_indices(set, element);
	if presence == .Present
	{
		return index;
	}
	return -1;
}

quick_set_resize :: proc (set: ^Quick_Set($T), new_size: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || pool == nil || new_size <= 0
	{
		return .Invalid_Argument;
	}
	new_elements, element_status := buffer_pool_take_at_least(pool, T, new_size);
	if element_status != .Ok
	{
		return memory_to_collection_status(element_status);
	}
	new_table, table_status := buffer_pool_take_at_least(
		pool,
		i32,
		int(new_elements.length) << uint(set.table_power_offset)
	);
	if table_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_elements);
		return memory_to_collection_status(table_status);
	}
	_ = buffer_clear(new_table, 0, int(new_table.length));
	new_count := min(set.count, int(new_elements.length));
	new_mask := int(new_table.length) - 1;
	for index in 0 ..< new_count
	{
		new_elements.memory[index] = set.elements.memory[index];
		table_index := int(u32(hash_rehash(set.comparer.hash(&new_elements.memory[index])))) & new_mask;
		for new_table.memory[table_index] != 0
		{
			table_index = (table_index + 1) & new_mask;
		}
		new_table.memory[table_index] = i32(index + 1);
	}
	old_elements := set.elements;
	old_table := set.table;
	set.elements = new_elements;
	set.table = new_table;
	set.table_mask = new_mask;
	set.count = new_count;
	element_return := buffer_pool_return(pool, &old_elements);
	table_return := buffer_pool_return(pool, &old_table);
	if element_return != .Ok || table_return != .Ok
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

quick_set_ensure_capacity :: proc (set: ^Quick_Set($T), count: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || count < 0
	{
		return .Invalid_Argument;
	}
	if count <= int(set.elements.length)
	{
		return .Ok;
	}
	return quick_set_resize(set, count, pool);
}

quick_set_add_unsafely :: proc (set: ^Quick_Set($T), element: T) -> Collection_Status
{
	if set == nil || set.count >= int(set.elements.length)
	{
		return .Capacity_Missing;
	}
	element_copy := element;
	presence, table_index, _ := quick_set_get_table_indices(set, &element_copy);
	if presence == .Present
	{
		return .Duplicate;
	}
	set.elements.memory[set.count] = element;
	set.count += 1;
	set.table.memory[table_index] = i32(set.count);
	return .Ok;
}

quick_set_add :: proc (set: ^Quick_Set($T), element: T, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil
	{
		return .Invalid_Argument;
	}
	if set.count == int(set.elements.length)
	{
		status := quick_set_resize(set, set.count * 2, pool);
		if status != .Ok
		{
			return status;
		}
	}
	return quick_set_add_unsafely(set, element);
}

quick_set_add_and_replace :: proc (
	set: ^Quick_Set($T),
	element: T,
	pool: ^Buffer_Pool
) -> (Replacement_Status, Collection_Status)
{
	if set == nil
	{
		return .Added, .Invalid_Argument;
	}
	if set.count == int(set.elements.length)
	{
		status := quick_set_resize(set, set.count * 2, pool);
		if status != .Ok
		{
			return .Added, status;
		}
	}
	element_copy := element;
	presence, table_index, element_index := quick_set_get_table_indices(set, &element_copy);
	if presence == .Present
	{
		set.elements.memory[element_index] = element;
		return .Replaced, .Ok;
	}
	set.elements.memory[set.count] = element;
	set.count += 1;
	set.table.memory[table_index] = i32(set.count);
	return .Added, .Ok;
}

quick_set_fast_remove_at :: proc (set: ^Quick_Set($T), table_index, element_index: int) -> Collection_Status
{
	if set == nil || element_index < 0 || element_index >= set.count
	{
		return .Not_Found;
	}
	table_cursor := table_index;
	gap_index := table_cursor;
	for
	{
		table_cursor = (table_cursor + 1) & set.table_mask;
		move_candidate := int(set.table.memory[table_cursor]);
		if move_candidate <= 0
		{
			break;
		}
		move_candidate -= 1;
		desired_index := int(u32(hash_rehash(set.comparer.hash(&set.elements.memory[move_candidate])))) & set.table_mask;
		if ((table_cursor - gap_index) & set.table_mask) <= ((table_cursor - desired_index) & set.table_mask)
		{
			set.table.memory[gap_index] = set.table.memory[table_cursor];
			gap_index = table_cursor;
		}
	}
	set.table.memory[gap_index] = 0;
	set.count -= 1;
	if element_index < set.count
	{
		set.elements.memory[element_index] = set.elements.memory[set.count];
		_, swapped_table_index, _ := quick_set_get_table_indices(set, &set.elements.memory[element_index]);
		set.table.memory[swapped_table_index] = i32(element_index + 1);
	}
	set.elements.memory[set.count] = {};
	return .Ok;
}

quick_set_fast_remove :: proc (set: ^Quick_Set($T), element: ^T) -> Collection_Status
{
	presence, table_index, element_index := quick_set_get_table_indices(set, element);
	if presence == .Missing
	{
		return .Not_Found;
	}
	return quick_set_fast_remove_at(set, table_index, element_index);
}

quick_set_clear :: proc (set: ^Quick_Set($T))
{
	if set == nil
	{
		return;
	}
	_ = buffer_clear(set.table, 0, int(set.table.length));
	_ = buffer_clear(set.elements, 0, set.count);
	set.count = 0;
}

quick_set_fast_clear :: proc (set: ^Quick_Set($T))
{
	if set == nil
	{
		return;
	}
	_ = buffer_clear(set.table, 0, int(set.table.length));
	set.count = 0;
}

quick_set_dispose :: proc (set: ^Quick_Set($T), pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || pool == nil || set.elements.memory == nil
	{
		return .Disposed;
	}
	element_status := buffer_pool_return(pool, &set.elements);
	table_status := buffer_pool_return(pool, &set.table);
	if element_status != .Ok || table_status != .Ok
	{
		return .Invalid_Argument;
	}
	set^ = {};
	return .Ok;
}
