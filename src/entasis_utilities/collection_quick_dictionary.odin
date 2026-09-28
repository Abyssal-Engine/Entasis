// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Quick_Dictionary :: struct($K, $V: typeid)
{
	count:              int,
	table_mask:         int,
	table_power_offset: int,
	table:              Buffer(i32),
	keys:               Buffer(K),
	values:             Buffer(V),
	comparer:           Hash_Equal_Procs,
}

quick_dictionary_initialize_from_buffers :: proc (
	dictionary: ^Quick_Dictionary($K, $V), keys: Buffer(K), values: Buffer(V), table: Buffer(i32),
	comparer: Hash_Equal_Procs, table_power_offset: int = 2,
) -> Collection_Status
{
	if dictionary == nil || keys.memory == nil || values.memory == nil || table.memory == nil ||
		keys.length <= 0 || values.length < keys.length || table.length < keys.length ||
		power_of_two_state(int(table.length)) != .Allocated || comparer.hash == nil || comparer.equal == nil ||
		table_power_offset < 0
	{
		return .Invalid_Argument;
	}
	_ = buffer_clear(table, 0, int(table.length));
	dictionary^ = Quick_Dictionary(K, V){
		table_mask=int(table.length) - 1,
		table_power_offset=table_power_offset,
		table=table,
		keys=keys,
		values=values,
		comparer=comparer,
	};
	return .Ok;
}

quick_dictionary_initialize :: proc (
	dictionary: ^Quick_Dictionary($K, $V), initial_capacity: int, pool: ^Buffer_Pool,
	comparer: Hash_Equal_Procs, table_power_offset: int = 2,
) -> Collection_Status
{
	if dictionary == nil || pool == nil || initial_capacity <= 0 || table_power_offset < 0
	{
		return .Invalid_Argument;
	}
	keys, key_status := buffer_pool_take_at_least(pool, K, initial_capacity);
	if key_status != .Ok
	{
		return memory_to_collection_status(key_status);
	}
	values, value_status := buffer_pool_take_at_least(pool, V, int(keys.length));
	if value_status != .Ok
	{
		_ = buffer_pool_return(pool, &keys);
		return memory_to_collection_status(value_status);
	}
	table, table_status := buffer_pool_take_at_least(pool, i32, int(keys.length) << uint(table_power_offset));
	if table_status != .Ok
	{
		_ = buffer_pool_return(pool, &values);
		_ = buffer_pool_return(pool, &keys);
		return memory_to_collection_status(table_status);
	}
	return quick_dictionary_initialize_from_buffers(dictionary, keys, values, table, comparer, table_power_offset);
}

quick_dictionary_get_table_indices :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: ^K,
) -> (Presence_Status, int, int)
{
	if dictionary == nil || key == nil || dictionary.comparer.hash == nil
	{
		return .Missing, -1, -1;
	}
	table_index := int(u32(hash_rehash(dictionary.comparer.hash(key)))) & dictionary.table_mask;
	for
	{
		encoded_index := int(dictionary.table.memory[table_index]);
		if encoded_index <= 0
		{
			return .Missing, table_index, -1;
		}
		element_index := encoded_index - 1;
		if dictionary.comparer.equal(&dictionary.keys.memory[element_index], key) == .Equal
		{
			return .Present, table_index, element_index;
		}
		table_index = (table_index + 1) & dictionary.table_mask;
	}
}

quick_dictionary_index_of :: proc (dictionary: ^Quick_Dictionary($K, $V), key: ^K) -> int
{
	presence, _, element_index := quick_dictionary_get_table_indices(dictionary, key);
	if presence == .Present
	{
		return element_index;
	}
	return -1;
}

quick_dictionary_try_get :: proc (dictionary: ^Quick_Dictionary($K, $V), key: ^K) -> (^V, Presence_Status)
{
	index := quick_dictionary_index_of(dictionary, key);
	if index < 0
	{
		return nil, .Missing;
	}
	return &dictionary.values.memory[index], .Present;
}

quick_dictionary_resize :: proc (
	dictionary: ^Quick_Dictionary($K, $V),
	new_size: int,
	pool: ^Buffer_Pool
) -> Collection_Status
{
	if dictionary == nil || pool == nil || new_size <= 0
	{
		return .Invalid_Argument;
	}
	new_keys, key_status := buffer_pool_take_at_least(pool, K, new_size);
	if key_status != .Ok
	{
		return memory_to_collection_status(key_status);
	}
	new_values, value_status := buffer_pool_take_at_least(pool, V, int(new_keys.length));
	if value_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_keys);
		return memory_to_collection_status(value_status);
	}
	new_table, table_status := buffer_pool_take_at_least(
		pool,
		i32,
		int(new_keys.length) << uint(dictionary.table_power_offset)
	);
	if table_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_values);
		_ = buffer_pool_return(pool, &new_keys);
		return memory_to_collection_status(table_status);
	}
	_ = buffer_clear(new_table, 0, int(new_table.length));
	new_count := min(dictionary.count, int(new_keys.length));
	new_mask := int(new_table.length) - 1;
	for index in 0 ..< new_count
	{
		new_keys.memory[index] = dictionary.keys.memory[index];
		new_values.memory[index] = dictionary.values.memory[index];
		table_index := int(u32(hash_rehash(dictionary.comparer.hash(&new_keys.memory[index])))) & new_mask;
		for new_table.memory[table_index] != 0
		{
			table_index = (table_index + 1) & new_mask;
		}
		new_table.memory[table_index] = i32(index + 1);
	}
	old_keys := dictionary.keys;
	old_values := dictionary.values;
	old_table := dictionary.table;
	dictionary.keys = new_keys;
	dictionary.values = new_values;
	dictionary.table = new_table;
	dictionary.table_mask = new_mask;
	dictionary.count = new_count;
	key_return := buffer_pool_return(pool, &old_keys);
	value_return := buffer_pool_return(pool, &old_values);
	table_return := buffer_pool_return(pool, &old_table);
	if key_return != .Ok || value_return != .Ok || table_return != .Ok
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

quick_dictionary_ensure_capacity :: proc (
	dictionary: ^Quick_Dictionary($K, $V),
	count: int,
	pool: ^Buffer_Pool
) -> Collection_Status
{
	if dictionary == nil || count < 0
	{
		return .Invalid_Argument;
	}
	if count <= int(dictionary.keys.length)
	{
		return .Ok;
	}
	return quick_dictionary_resize(dictionary, count, pool);
}

quick_dictionary_compact :: proc (dictionary: ^Quick_Dictionary($K, $V), pool: ^Buffer_Pool) -> Collection_Status
{
	target, status := buffer_pool_capacity_for_count(K, max(dictionary.count, 1));
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	if target == int(dictionary.keys.length)
	{
		return .Ok;
	}
	return quick_dictionary_resize(dictionary, target, pool);
}

quick_dictionary_find_or_allocate_unsafely :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: K,
) -> (int, Presence_Status, Collection_Status)
{
	if dictionary == nil || dictionary.count >= int(dictionary.keys.length)
	{
		return -1, .Missing, .Capacity_Missing;
	}
	key_copy := key;
	presence, table_index, element_index := quick_dictionary_get_table_indices(dictionary, &key_copy);
	if presence == .Present
	{
		return element_index, .Present, .Ok;
	}
	element_index = dictionary.count;
	dictionary.count += 1;
	dictionary.keys.memory[element_index] = key;
	dictionary.table.memory[table_index] = i32(element_index + 1);
	return element_index, .Missing, .Ok;
}

quick_dictionary_find_or_allocate :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: K, pool: ^Buffer_Pool,
) -> (int, Presence_Status, Collection_Status)
{
	if dictionary == nil
	{
		return -1, .Missing, .Invalid_Argument;
	}
	if dictionary.count == int(dictionary.keys.length)
	{
		status := quick_dictionary_resize(dictionary, dictionary.count * 2, pool);
		if status != .Ok
		{
			return -1, .Missing, status;
		}
	}
	return quick_dictionary_find_or_allocate_unsafely(dictionary, key);
}

quick_dictionary_add_unsafely :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: K, value: V,
) -> Collection_Status
{
	index, presence, status := quick_dictionary_find_or_allocate_unsafely(dictionary, key);
	if status != .Ok
	{
		return status;
	}
	if presence == .Present
	{
		return .Duplicate;
	}
	dictionary.values.memory[index] = value;
	return .Ok;
}

quick_dictionary_add :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: K, value: V, pool: ^Buffer_Pool,
) -> Collection_Status
{
	if dictionary == nil
	{
		return .Invalid_Argument;
	}
	if dictionary.count == int(dictionary.keys.length)
	{
		status := quick_dictionary_resize(dictionary, dictionary.count * 2, pool);
		if status != .Ok
		{
			return status;
		}
	}
	return quick_dictionary_add_unsafely(dictionary, key, value);
}

quick_dictionary_add_and_replace :: proc (
	dictionary: ^Quick_Dictionary($K, $V), key: K, value: V, pool: ^Buffer_Pool,
) -> (Replacement_Status, Collection_Status)
{
	if dictionary == nil
	{
		return .Added, .Invalid_Argument;
	}
	if dictionary.count == int(dictionary.keys.length)
	{
		status := quick_dictionary_resize(dictionary, dictionary.count * 2, pool);
		if status != .Ok
		{
			return .Added, status;
		}
	}
	key_copy := key;
	presence, table_index, element_index := quick_dictionary_get_table_indices(dictionary, &key_copy);
	if presence == .Present
	{
		dictionary.keys.memory[element_index] = key;
		dictionary.values.memory[element_index] = value;
		return .Replaced, .Ok;
	}
	element_index = dictionary.count;
	dictionary.count += 1;
	dictionary.keys.memory[element_index] = key;
	dictionary.values.memory[element_index] = value;
	dictionary.table.memory[table_index] = i32(element_index + 1);
	return .Added, .Ok;
}

quick_dictionary_fast_remove_at :: proc (
	dictionary: ^Quick_Dictionary($K, $V), table_index, element_index: int,
) -> Collection_Status
{
	if dictionary == nil || table_index < 0 || element_index < 0 || element_index >= dictionary.count
	{
		return .Not_Found;
	}
	table_cursor := table_index;
	gap_index := table_cursor;
	for
	{
		table_cursor = (table_cursor + 1) & dictionary.table_mask;
		move_candidate := int(dictionary.table.memory[table_cursor]);
		if move_candidate <= 0
		{
			break;
		}
		move_candidate -= 1;
		desired_index := int(u32(hash_rehash(dictionary.comparer.hash(&dictionary.keys.memory[move_candidate])))) & dictionary.table_mask;
		distance_from_gap := (table_cursor - gap_index) & dictionary.table_mask;
		distance_from_ideal := (table_cursor - desired_index) & dictionary.table_mask;
		if distance_from_gap <= distance_from_ideal
		{
			dictionary.table.memory[gap_index] = dictionary.table.memory[table_cursor];
			gap_index = table_cursor;
		}
	}
	dictionary.table.memory[gap_index] = 0;
	dictionary.count -= 1;
	if element_index < dictionary.count
	{
		dictionary.keys.memory[element_index] = dictionary.keys.memory[dictionary.count];
		dictionary.values.memory[element_index] = dictionary.values.memory[dictionary.count];
		_, swapped_table_index, _ := quick_dictionary_get_table_indices(
			dictionary,
			&dictionary.keys.memory[element_index]
		);
		dictionary.table.memory[swapped_table_index] = i32(element_index + 1);
	}
	dictionary.keys.memory[dictionary.count] = {};
	dictionary.values.memory[dictionary.count] = {};
	return .Ok;
}

quick_dictionary_fast_remove :: proc (dictionary: ^Quick_Dictionary($K, $V), key: ^K) -> Collection_Status
{
	presence, table_index, element_index := quick_dictionary_get_table_indices(dictionary, key);
	if presence == .Missing
	{
		return .Not_Found;
	}
	return quick_dictionary_fast_remove_at(dictionary, table_index, element_index);
}

quick_dictionary_clear :: proc (dictionary: ^Quick_Dictionary($K, $V))
{
	if dictionary == nil
	{
		return;
	}
	_ = buffer_clear(dictionary.table, 0, int(dictionary.table.length));
	_ = buffer_clear(dictionary.keys, 0, dictionary.count);
	_ = buffer_clear(dictionary.values, 0, dictionary.count);
	dictionary.count = 0;
}

quick_dictionary_fast_clear :: proc (dictionary: ^Quick_Dictionary($K, $V))
{
	if dictionary == nil
	{
		return;
	}
	_ = buffer_clear(dictionary.table, 0, int(dictionary.table.length));
	dictionary.count = 0;
}

quick_dictionary_dispose :: proc (dictionary: ^Quick_Dictionary($K, $V), pool: ^Buffer_Pool) -> Collection_Status
{
	if dictionary == nil || pool == nil || dictionary.keys.memory == nil
	{
		return .Disposed;
	}
	key_status := buffer_pool_return(pool, &dictionary.keys);
	value_status := buffer_pool_return(pool, &dictionary.values);
	table_status := buffer_pool_return(pool, &dictionary.table);
	if key_status != .Ok || value_status != .Ok || table_status != .Ok
	{
		return .Invalid_Argument;
	}
	dictionary^ = {};
	return .Ok;
}
