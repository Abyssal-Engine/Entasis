// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Allocator_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Duplicate_Id,
	Not_Found,
	Cannot_Fit,
	Capacity_Missing,
	Disposed,
}

Allocator_Fit_Status :: enum u8
{
	Cannot_Fit,
	Can_Fit,
}

Memory_Allocation :: struct
{
	start:    i64,
	end:      i64,
	previous: u64,
	next:     u64,
}

Memory_Allocator :: struct
{
	pool:               ^Buffer_Pool,
	capacity:           i64,
	search_start_index: int,
	allocations:        Quick_Dictionary(u64, Memory_Allocation),
}

allocator_collection_status :: proc "contextless" (status: Collection_Status) -> Allocator_Status
{
	switch status
	{
		case .Ok:
			return .Ok;
		case .Capacity_Missing, .Out_Of_Memory:
			return .Capacity_Missing;
		case .Disposed:
			return .Disposed;
		case .Duplicate:
			return .Duplicate_Id;
		case .Not_Found:
			return .Not_Found;
		case .Invalid_Argument, .Empty:
			return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

memory_allocator_initialize :: proc (
	allocator: ^Memory_Allocator, capacity: i64, initial_allocation_capacity: int,
	pool: ^Buffer_Pool,
) -> Allocator_Status
{
	if allocator == nil || pool == nil || capacity < 0 || initial_allocation_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := quick_dictionary_initialize(
		&allocator.allocations,
		initial_allocation_capacity,
		pool,
		primitive_u64_hash_equal()
	);
	if status != .Ok
	{
		return allocator_collection_status(status);
	}
	allocator.pool = pool;
	allocator.capacity = capacity;
	return .Ok;
}

memory_allocator_set_capacity :: proc (allocator: ^Memory_Allocator, capacity: i64) -> Allocator_Status
{
	if allocator == nil || capacity < 0
	{
		return .Invalid_Argument;
	}
	if capacity < allocator.capacity
	{
		for index in 0 ..< allocator.allocations.count
		{
			if capacity < allocator.allocations.values.memory[index].end
			{
				return .Cannot_Fit;
			}
		}
	}
	allocator.capacity = capacity;
	return .Ok;
}

memory_allocator_contains :: proc (allocator: ^Memory_Allocator, id: u64) -> Presence_Status
{
	if allocator == nil
	{
		return .Missing;
	}
	id_copy := id;
	if quick_dictionary_index_of(&allocator.allocations, &id_copy) >= 0
	{
		return .Present;
	}
	return .Missing;
}

memory_allocator_try_get :: proc (allocator: ^Memory_Allocator, id: u64) -> (^Memory_Allocation, Presence_Status)
{
	if allocator == nil
	{
		return nil, .Missing;
	}
	id_copy := id;
	return quick_dictionary_try_get(&allocator.allocations, &id_copy);
}

memory_allocator_can_fit :: proc (
	allocator: ^Memory_Allocator, size: i64, ignored_ids: Predicate_Proc = nil, predicate_context: rawptr = nil,
) -> Allocator_Fit_Status
{
	if allocator == nil || size < 0
	{
		return .Cannot_Fit;
	}
	if allocator.allocations.count == 0
	{
		if size <= allocator.capacity
		{
			return .Can_Fit;
		}
		return .Cannot_Fit;
	}
	allocation_index := allocator.search_start_index;
	initial_id := allocator.allocations.keys.memory[allocation_index];
	for
	{
		allocation := allocator.allocations.values.memory[allocation_index];
		next_id := allocation.next;
		next_index: int;
		next_allocation: Memory_Allocation;
		for
		{
			next_index = quick_dictionary_index_of(&allocator.allocations, &next_id);
			next_allocation = allocator.allocations.values.memory[next_index];
			next_id = next_allocation.next;
			if ignored_ids == nil || ignored_ids(&next_id, predicate_context) != .Equal
			{
				break;
			}
		}
		if next_allocation.start < allocation.end
		{
			if allocator.capacity - allocation.end >= size || next_allocation.start >= size
			{
				return .Can_Fit;
			}
		}
		else if next_allocation.start - allocation.end >= size
		{
			return .Can_Fit;
		}
		allocation_index = next_index;
		if allocator.allocations.keys.memory[allocation_index] == initial_id
		{
			return .Cannot_Fit;
		}
	}
}

memory_allocator_add_allocation :: proc (
	allocator: ^Memory_Allocator, id: u64, start, end: i64, allocation_index, next_index: int,
) -> Allocator_Status
{
	allocation := &allocator.allocations.values.memory[allocation_index];
	next_allocation := &allocator.allocations.values.memory[next_index];
	new_allocation := Memory_Allocation{
		start=start,
		end=end,
		next=allocation.next,
		previous=next_allocation.previous,
	};
	allocation.next = id;
	next_allocation.previous = id;
	allocator.search_start_index = allocator.allocations.count;
	status := quick_dictionary_add(&allocator.allocations, id, new_allocation, allocator.pool);
	return allocator_collection_status(status);
}

memory_allocator_allocate :: proc (allocator: ^Memory_Allocator, id: u64, size: i64) -> (i64, Allocator_Status)
{
	if allocator == nil || size < 0
	{
		return 0, .Invalid_Argument;
	}
	if memory_allocator_contains(allocator, id) == .Present
	{
		return 0, .Duplicate_Id;
	}
	capacity_status := quick_dictionary_ensure_capacity(
		&allocator.allocations,
		allocator.allocations.count + 1,
		allocator.pool
	);
	if capacity_status != .Ok
	{
		return 0, allocator_collection_status(capacity_status);
	}
	if allocator.allocations.count == 0
	{
		if size > allocator.capacity
		{
			return 0, .Cannot_Fit;
		}
		allocation := Memory_Allocation{start=0, end=size, previous=id, next=id};
		status := quick_dictionary_add(&allocator.allocations, id, allocation, allocator.pool);
		if status != .Ok
		{
			return 0, allocator_collection_status(status);
		}
		allocator.search_start_index = 0;
		return 0, .Ok;
	}
	allocation_index := allocator.search_start_index;
	initial_id := allocator.allocations.keys.memory[allocation_index];
	for
	{
		allocation := allocator.allocations.values.memory[allocation_index];
		next_index := quick_dictionary_index_of(&allocator.allocations, &allocation.next);
		next_allocation := allocator.allocations.values.memory[next_index];
		if next_allocation.start < allocation.end
		{
			if allocator.capacity - allocation.end >= size
			{
				start := allocation.end;
				return start, memory_allocator_add_allocation(
					allocator,
					id,
					start,
					start + size,
					allocation_index,
					next_index
				);
			}
			if next_allocation.start >= size
			{
				return 0, memory_allocator_add_allocation(allocator, id, 0, size, allocation_index, next_index);
			}
		}
		else if next_allocation.start - allocation.end >= size
		{
			start := allocation.end;
			return start, memory_allocator_add_allocation(
				allocator,
				id,
				start,
				start + size,
				allocation_index,
				next_index
			);
		}
		allocation_index = next_index;
		if allocator.allocations.keys.memory[allocation_index] == initial_id
		{
			return 0, .Cannot_Fit;
		}
	}
}

memory_allocator_deallocate :: proc (allocator: ^Memory_Allocator, id: u64) -> Allocator_Status
{
	if allocator == nil
	{
		return .Invalid_Argument;
	}
	id_copy := id;
	allocation_index := quick_dictionary_index_of(&allocator.allocations, &id_copy);
	if allocation_index < 0
	{
		return .Not_Found;
	}
	allocation := allocator.allocations.values.memory[allocation_index];
	if allocation.previous != id
	{
		previous_index := quick_dictionary_index_of(&allocator.allocations, &allocation.previous);
		next_index := quick_dictionary_index_of(&allocator.allocations, &allocation.next);
		allocator.allocations.values.memory[previous_index].next = allocation.next;
		allocator.allocations.values.memory[next_index].previous = allocation.previous;
	}
	status := quick_dictionary_fast_remove(&allocator.allocations, &id_copy);
	if status != .Ok
	{
		return allocator_collection_status(status);
	}
	allocator.search_start_index = quick_dictionary_index_of(&allocator.allocations, &allocation.previous);
	return .Ok;
}

memory_allocator_largest_contiguous :: proc (allocator: ^Memory_Allocator) -> (largest, total: i64)
{
	if allocator == nil
	{
		return;
	}
	if allocator.allocations.count == 0
	{
		return allocator.capacity, allocator.capacity;
	}
	for index in 0 ..< allocator.allocations.count
	{
		allocation := allocator.allocations.values.memory[index];
		next_index := quick_dictionary_index_of(&allocator.allocations, &allocation.next);
		next_allocation := allocator.allocations.values.memory[next_index];
		to_next := next_allocation.start - allocation.end;
		if to_next < 0
		{
			adjacent := allocator.capacity - allocation.end;
			wrapped := next_allocation.start;
			largest = max(largest, max(adjacent, wrapped));
			total += adjacent + wrapped;
		}
		else
		{
			largest = max(largest, to_next);
			total += to_next;
		}
	}
	return;
}

memory_allocator_incremental_compact :: proc (
	allocator: ^Memory_Allocator,
) -> (id: u64, size, old_start, new_start: i64, status: Allocator_Status)
{
	if allocator == nil || allocator.allocations.count == 0
	{
		return 0, 0, 0, 0, .Not_Found;
	}
	for index in 0 ..< allocator.allocations.count
	{
		allocation := allocator.allocations.values.memory[index];
		previous_index := quick_dictionary_index_of(&allocator.allocations, &allocation.previous);
		if allocator.allocations.values.memory[previous_index].end > allocation.start
		{
			walk_index := index;
			previous_end: i64 = 0;
			for _ in 0 ..< allocator.allocations.count
			{
				allocator.search_start_index = walk_index;
				current := &allocator.allocations.values.memory[walk_index];
				if current.start > previous_end
				{
					id = allocator.allocations.keys.memory[walk_index];
					size = current.end - current.start;
					old_start = current.start;
					new_start = previous_end;
					current.start = new_start;
					current.end = new_start + size;
					return id, size, old_start, new_start, .Ok;
				}
				previous_end = current.end;
				walk_index = quick_dictionary_index_of(&allocator.allocations, &current.next);
			}
			break;
		}
	}
	return 0, 0, 0, 0, .Not_Found;
}

memory_allocator_resize :: proc (
	allocator: ^Memory_Allocator,
	id: u64,
	size: i64
) -> (old_start, new_start: i64, status: Allocator_Status)
{
	if allocator == nil || size < 0
	{
		return 0, 0, .Invalid_Argument;
	}
	id_copy := id;
	index := quick_dictionary_index_of(&allocator.allocations, &id_copy);
	if index < 0
	{
		return 0, 0, .Not_Found;
	}
	allocation := allocator.allocations.values.memory[index];
	old_start = allocation.start;
	current_size := allocation.end - allocation.start;
	if size == current_size
	{
		return old_start, old_start, .Invalid_Argument;
	}
	if size < current_size
	{
		allocator.allocations.values.memory[index].end = allocation.start + size;
		return old_start, allocation.start, .Ok;
	}
	deallocate_status := memory_allocator_deallocate(allocator, id);
	if deallocate_status != .Ok
	{
		return old_start, old_start, deallocate_status;
	}
	allocated_start, allocate_status := memory_allocator_allocate(allocator, id, size);
	if allocate_status != .Ok
	{
		restored_start, restore_status := memory_allocator_allocate(allocator, id, current_size);
		if restore_status != .Ok
		{
			return old_start, restored_start, restore_status;
		}
		return old_start, restored_start, .Cannot_Fit;
	}
	return old_start, allocated_start, .Ok;
}

memory_allocator_dispose :: proc (allocator: ^Memory_Allocator) -> Allocator_Status
{
	if allocator == nil || allocator.pool == nil
	{
		return .Disposed;
	}
	status := quick_dictionary_dispose(&allocator.allocations, allocator.pool);
	if status != .Ok
	{
		return allocator_collection_status(status);
	}
	allocator^ = {};
	return .Ok;
}
