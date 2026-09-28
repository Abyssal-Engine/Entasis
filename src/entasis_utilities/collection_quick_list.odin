// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"

Quick_List :: struct($T: typeid)
{
	span:  Buffer(T),
	count: int,
}

quick_list_initialize :: proc (
	list: ^Quick_List($T),
	minimum_initial_count: int,
	pool: ^Buffer_Pool
) -> Collection_Status
{
	if list == nil || minimum_initial_count <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	span, status := buffer_pool_take_at_least(pool, T, minimum_initial_count);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	list^ = Quick_List(T){span=span};
	return .Ok;
}

quick_list_initialize_from_buffer :: proc (list: ^Quick_List($T), span: Buffer(T)) -> Collection_Status
{
	if list == nil || span.memory == nil || span.length <= 0
	{
		return .Invalid_Argument;
	}
	list^ = Quick_List(T){span=span};
	return .Ok;
}

quick_list_at :: proc (list: ^Quick_List($T), index: int) -> (^T, Collection_Status)
{
	if list == nil || index < 0 || index >= list.count
	{
		return nil, .Invalid_Argument;
	}
	return &list.span.memory[index], .Ok;
}

quick_list_ensure_capacity :: proc (list: ^Quick_List($T), count: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if list == nil || pool == nil || count < 0
	{
		return .Invalid_Argument;
	}
	if list.span.memory == nil
	{
		span, status := buffer_pool_take_at_least(pool, T, max(count, 1));
		if status != .Ok
		{
			return memory_to_collection_status(status);
		}
		list.span = span;
		return .Ok;
	}
	if count <= int(list.span.length)
	{
		return .Ok;
	}
	status := buffer_pool_resize_to_at_least(pool, &list.span, count, list.count);
	return memory_to_collection_status(status);
}

quick_list_resize :: proc (list: ^Quick_List($T), new_size: int, pool: ^Buffer_Pool) -> Collection_Status
{
	if list == nil || new_size <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	copy_count := min(list.count, new_size);
	status := buffer_pool_resize_to_at_least(pool, &list.span, new_size, copy_count);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	list.count = copy_count;
	return .Ok;
}

quick_list_compact :: proc (list: ^Quick_List($T), pool: ^Buffer_Pool) -> Collection_Status
{
	if list == nil || list.span.memory == nil
	{
		return .Invalid_Argument;
	}
	target, status := buffer_pool_capacity_for_count(T, max(list.count, 1));
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	if target == int(list.span.length)
	{
		return .Ok;
	}
	return quick_list_resize(list, target, pool);
}

quick_list_allocate_unsafely :: proc (list: ^Quick_List($T), count: int = 1) -> (^T, Collection_Status)
{
	if list == nil || count <= 0 || list.count > int(list.span.length) - count
	{
		return nil, .Capacity_Missing;
	}
	start := list.count;
	list.count += count;
	return &list.span.memory[start], .Ok;
}

quick_list_allocate :: proc (list: ^Quick_List($T), count: int, pool: ^Buffer_Pool) -> (^T, Collection_Status)
{
	if list == nil || count <= 0
	{
		return nil, .Invalid_Argument;
	}
	new_count := list.count + count;
	if new_count > int(list.span.length)
	{
		status := quick_list_ensure_capacity(list, max(max(list.count * 2, 1), new_count), pool);
		if status != .Ok
		{
			return nil, status;
		}
	}
	return quick_list_allocate_unsafely(list, count);
}

quick_list_add_unsafely :: proc (list: ^Quick_List($T), value: T) -> Collection_Status
{
	target, status := quick_list_allocate_unsafely(list);
	if status != .Ok
	{
		return status;
	}
	target^ = value;
	return .Ok;
}

quick_list_add :: proc (list: ^Quick_List($T), value: T, pool: ^Buffer_Pool) -> Collection_Status
{
	target, status := quick_list_allocate(list, 1, pool);
	if status != .Ok
	{
		return status;
	}
	target^ = value;
	return .Ok;
}

quick_list_add_range_unsafely :: proc (list: ^Quick_List($T), source: [^]T, count: int) -> Collection_Status
{
	if list == nil || source == nil || count < 0 || list.count > int(list.span.length) - count
	{
		return .Capacity_Missing;
	}
	if count > 0
	{
		intrinsics.mem_copy(&list.span.memory[list.count], source, count * size_of(T));
	}
	list.count += count;
	return .Ok;
}

quick_list_index_of :: proc (list: ^Quick_List($T), value: ^T, equal: Equal_Proc) -> int
{
	if list == nil || value == nil || equal == nil
	{
		return -1;
	}
	for index in 0 ..< list.count
	{
		if equal(&list.span.memory[index], value) == .Equal
		{
			return index;
		}
	}
	return -1;
}

quick_list_index_of_predicate :: proc (
	list: ^Quick_List($T), predicate: Predicate_Proc, predicate_context: rawptr,
) -> int
{
	if list == nil || predicate == nil
	{
		return -1;
	}
	for index in 0 ..< list.count
	{
		if predicate(&list.span.memory[index], predicate_context) == .Equal
		{
			return index;
		}
	}
	return -1;
}

quick_list_remove :: proc (list: ^Quick_List($T), value: ^T, equal: Equal_Proc) -> Collection_Status
{
	index := quick_list_index_of(list, value, equal);
	if index < 0
	{
		return .Not_Found;
	}
	return quick_list_remove_at(list, index);
}

quick_list_fast_remove :: proc (list: ^Quick_List($T), value: ^T, equal: Equal_Proc) -> Collection_Status
{
	index := quick_list_index_of(list, value, equal);
	if index < 0
	{
		return .Not_Found;
	}
	return quick_list_fast_remove_at(list, index);
}

quick_list_for_each_ref :: proc (
	list: ^Quick_List($T), loop_body: For_Each_Ref_Proc, loop_context: rawptr,
) -> Collection_Status
{
	if list == nil || loop_body == nil
	{
		return .Invalid_Argument;
	}
	for index in 0 ..< list.count
	{
		loop_body(&list.span.memory[index], loop_context);
	}
	return .Ok;
}

quick_list_for_each_ref_breakable :: proc (
	list: ^Quick_List($T), loop_body: Breakable_For_Each_Ref_Proc, loop_context: rawptr,
) -> (int, Collection_Status)
{
	if list == nil || loop_body == nil
	{
		return -1, .Invalid_Argument;
	}
	for index in 0 ..< list.count
	{
		if loop_body(&list.span.memory[index], loop_context) == .Break
		{
			return index, .Ok;
		}
	}
	return list.count, .Ok;
}

quick_list_remove_at :: proc (list: ^Quick_List($T), index: int) -> Collection_Status
{
	if list == nil || index < 0 || index >= list.count
	{
		return .Not_Found;
	}
	move_count := list.count - index - 1;
	if move_count > 0
	{
		for move_index in 0 ..< move_count
		{
			list.span.memory[index + move_index] = list.span.memory[index + move_index + 1];
		}
	}
	list.count -= 1;
	list.span.memory[list.count] = {};
	return .Ok;
}

quick_list_fast_remove_at :: proc (list: ^Quick_List($T), index: int) -> Collection_Status
{
	if list == nil || index < 0 || index >= list.count
	{
		return .Not_Found;
	}
	list.count -= 1;
	if index < list.count
	{
		list.span.memory[index] = list.span.memory[list.count];
	}
	list.span.memory[list.count] = {};
	return .Ok;
}

quick_list_pop :: proc (list: ^Quick_List($T)) -> (T, Collection_Status)
{
	if list == nil || list.count <= 0
	{
		return {}, .Empty;
	}
	list.count -= 1;
	value := list.span.memory[list.count];
	list.span.memory[list.count] = {};
	return value, .Ok;
}

quick_list_clear :: proc (list: ^Quick_List($T))
{
	if list == nil
	{
		return;
	}
	if list.count > 0
	{
		intrinsics.mem_zero(list.span.memory, list.count * size_of(T));
	}
	list.count = 0;
}

quick_list_dispose :: proc (list: ^Quick_List($T), pool: ^Buffer_Pool) -> Collection_Status
{
	if list == nil || pool == nil || list.span.memory == nil
	{
		return .Disposed;
	}
	status := buffer_pool_return(pool, &list.span);
	if status != .Ok
	{
		return memory_to_collection_status(status);
	}
	list^ = {};
	return .Ok;
}
