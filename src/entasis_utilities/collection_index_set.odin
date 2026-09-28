// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Index_Set :: struct
{
	flags: Buffer(u64),
}

INDEX_SET_SHIFT :: 6;
INDEX_SET_MASK :: 63;

index_set_bundle_capacity :: proc "contextless" (count: int) -> int
{
	return (count + INDEX_SET_MASK) >> INDEX_SET_SHIFT;
}

index_set_initialize :: proc (set: ^Index_Set, initial_capacity: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || pool == nil || initial_capacity < 0
	{
		return .Invalid_Argument;
	}
	bundle_count := max(index_set_bundle_capacity(initial_capacity), 1);
	flags, status := buffer_pool_take_at_least(pool, u64, bundle_count);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	_ = buffer_clear(flags, 0, int(flags.length));
	set.flags = flags;
	return .Ok;
}

index_set_contains :: proc "contextless" (set: ^Index_Set, index: int) -> Presence_Status
{
	if set == nil || index < 0
	{
		return .Missing;
	}
	packed_index := index >> INDEX_SET_SHIFT;
	if packed_index < int(set.flags.length) &&
		(set.flags.memory[packed_index] & (u64(1) << uint(index & INDEX_SET_MASK))) != 0
	{
		return .Present;
	}
	return .Missing;
}

index_set_resize_for_bundle_count :: proc (set: ^Index_Set, bundle_count: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || bundle_count <= 0
	{
		return .Invalid_Argument;
	}
	old_length := int(set.flags.length);
	copy_count := min(bundle_count, old_length);
	status := buffer_pool_resize_to_at_least(pool, &set.flags, bundle_count, copy_count);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	if int(set.flags.length) > copy_count
	{
		_ = buffer_clear(set.flags, copy_count, int(set.flags.length) - copy_count);
	}
	return .Ok;
}

index_set_ensure_capacity :: proc (set: ^Index_Set, index_capacity: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || index_capacity < 0
	{
		return .Invalid_Argument;
	}
	required := max(index_set_bundle_capacity(index_capacity), 1);
	if required <= int(set.flags.length)
	{
		return .Ok;
	}
	target, status := buffer_pool_capacity_for_count(u64, required);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	return index_set_resize_for_bundle_count(set, target, pool);
}

index_set_set_unsafely :: proc (set: ^Index_Set, index: int) -> Collection_Status
{
	if set == nil || index < 0 || (index >> INDEX_SET_SHIFT) >= int(set.flags.length)
	{
		return .Capacity_Missing;
	}
	set.flags.memory[index >> INDEX_SET_SHIFT] |= u64(1) << uint(index & INDEX_SET_MASK);
	return .Ok;
}

index_set_set :: proc (set: ^Index_Set, index: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if index < 0
	{
		return .Invalid_Argument;
	}
	status := index_set_ensure_capacity(set, index + 1, pool);
	if status != .Ok
	{
		return status;
	}
	return index_set_set_unsafely(set, index);
}

index_set_add :: proc (set: ^Index_Set, index: int, pool: ^Buffer_Pool) -> Collection_Status
{
	status := index_set_ensure_capacity(set, index + 1, pool);
	if status != .Ok
	{
		return status;
	}
	if index_set_contains(set, index) == .Present
	{
		return .Duplicate;
	}
	return index_set_set_unsafely(set, index);
}

index_set_unset :: proc (set: ^Index_Set, index: int) -> Collection_Status
{
	if index_set_contains(set, index) == .Missing
	{
		return .Not_Found;
	}
	set.flags.memory[index >> INDEX_SET_SHIFT] &= ~(u64(1) << uint(index & INDEX_SET_MASK));
	return .Ok;
}

index_set_can_fit :: proc "contextless" (set: ^Index_Set, indices: [^]i32, count: int) -> Presence_Status
{
	if set == nil || indices == nil || count < 0
	{
		return .Missing;
	}
	for index in 0 ..< count
	{
		if index_set_contains(set, int(indices[index])) == .Present
		{
			return .Missing;
		}
	}
	return .Present;
}

index_set_clear :: proc (set: ^Index_Set)
{
	if set == nil
	{
		return;
	}
	_ = buffer_clear(set.flags, 0, int(set.flags.length));
}

index_set_compact :: proc (set: ^Index_Set, index_capacity: int, pool: ^Buffer_Pool) -> Collection_Status
{
	desired, status := buffer_pool_capacity_for_count(u64, max(index_set_bundle_capacity(index_capacity), 1));
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	if int(set.flags.length) <= desired
	{
		return .Ok;
	}
	return index_set_resize_for_bundle_count(set, desired, pool);
}

index_set_dispose :: proc (set: ^Index_Set, pool: ^Buffer_Pool) -> Collection_Status
{
	if set == nil || set.flags.memory == nil
	{
		return .Disposed;
	}
	status := buffer_pool_return(pool, &set.flags);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	set^ = {};
	return .Ok;
}
