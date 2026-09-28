// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"
import "base:runtime"
import "core:mem"

BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT :: 16;
BUFFER_POOL_ID_SLOT_COUNT :: 1 << BUFFER_POOL_ID_POWER_SHIFT;
when ODIN_DEBUG
{
	Power_Pool_Debug_State :: struct
	{
		outstanding_words: [^]u64,
		word_capacity:     int,
	}
}
else
{
	Power_Pool_Debug_State :: struct
	{
	}
}

Power_Pool :: struct
{
	blocks:                           [^]rawptr,
	block_capacity:                   int,
	block_count:                      int,
	free_slots:                       [^]i32,
	slot_capacity:                    int,
	free_count:                       int,
	next_slot:                        int,
	suballocations_per_block:         int,
	suballocations_per_block_shift:   int,
	suballocations_per_block_mask:    int,
	power:                            int,
	suballocation_size:               int,
	block_size:                       int,
	debug:                            Power_Pool_Debug_State,
}

Buffer_Pool :: struct
{
	pools:              [BUFFER_POOL_POWER_COUNT]Power_Pool,
	minimum_block_size: int,
	state:              Pool_State,
	allocation_scope:   Allocation_Scope,
	allocator:          mem.Allocator,
}

Worker_Buffer_Pools :: struct
{
	pools:                  [^]Buffer_Pool,
	worker_count:           int,
	default_block_capacity: int,
	state:                  Pool_State,
	allocation_scope:       Allocation_Scope,
	allocator:              mem.Allocator,
}

power_pool_resize_blocks :: proc(pool: ^Buffer_Pool, power_pool: ^Power_Pool, new_capacity: int) -> Memory_Status
{
	if power_pool == nil || new_capacity <= power_pool.block_capacity ||
	new_capacity > max(int) / size_of(rawptr)
	{
		if power_pool != nil && new_capacity <= power_pool.block_capacity
		{
			return .Ok;
		}
		return .Invalid_Count;
	}
	memory, allocation_error := mem.alloc(
		size_of(rawptr) * new_capacity, align_of(rawptr), pool.allocator,
	); // odin-contracts-allow: setup rule=ODIN_HOT_GENERAL_ALLOC_CALL owner=Buffer_Pool phase=metadata_growth reason=entasis_power_pool_block_reference_growth
	if allocation_error != nil || memory == nil
	{
		return .Out_Of_Memory;
	}
	new_blocks := ([^]rawptr)(memory);
	if power_pool.block_count > 0
	{
		intrinsics.mem_copy(
			new_blocks, power_pool.blocks, size_of(rawptr) * power_pool.block_count,
		);
	}
	if power_pool.blocks != nil
	{
		_ = allocation_free(power_pool.blocks, size_of(rawptr) * power_pool.block_capacity, align_of(rawptr), pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=metadata_growth reason=replace_block_reference_storage
	}
	power_pool.blocks = new_blocks;
	power_pool.block_capacity = new_capacity;
	return .Ok;
}

power_pool_resize_free_slots :: proc(pool: ^Buffer_Pool, power_pool: ^Power_Pool, new_capacity: int) -> Memory_Status
{
	if power_pool == nil || new_capacity <= power_pool.slot_capacity ||
	new_capacity > max(int) / size_of(i32)
	{
		if power_pool != nil && new_capacity <= power_pool.slot_capacity
		{
			return .Ok;
		}
		return .Invalid_Count;
	}
	memory, allocation_error := mem.alloc(
		size_of(i32) * new_capacity, align_of(i32), pool.allocator,
	); // odin-contracts-allow: setup rule=ODIN_HOT_GENERAL_ALLOC_CALL owner=Buffer_Pool phase=metadata_growth reason=entasis_managed_id_pool_growth
	if allocation_error != nil || memory == nil
	{
		return .Out_Of_Memory;
	}
	new_slots := ([^]i32)(memory);
	if power_pool.free_count > 0
	{
		intrinsics.mem_copy(
			new_slots, power_pool.free_slots, size_of(i32) * power_pool.free_count,
		);
	}
	if power_pool.free_slots != nil
	{
		_ = allocation_free(power_pool.free_slots, size_of(i32) * power_pool.slot_capacity, align_of(i32), pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=metadata_growth reason=replace_returned_id_storage
	}
	power_pool.free_slots = new_slots;
	power_pool.slot_capacity = new_capacity;
	return .Ok;
}

power_pool_geometric_capacity :: proc(
	current_capacity,
	required_capacity,
	minimum_capacity: int
) -> (int, Memory_Status)
{
	if required_capacity <= 0 || minimum_capacity <= 0
	{
		return 0, .Invalid_Count;
	}
	target := max(required_capacity, minimum_capacity);
	if current_capacity > 0
	{
		if current_capacity > max(int) / 2
		{
			return 0, .Overflow;
		}
		target = max(target, current_capacity * 2);
	}
	power, status := containing_power_of_two(target);
	if status != .Ok
	{
		return 0, status;
	}
	return 1 << uint(power), .Ok;
}

when ODIN_DEBUG
{
	power_pool_debug_ensure_slot_capacity :: proc(
		pool: ^Buffer_Pool,
		power_pool: ^Power_Pool, slot: int,
	) -> Memory_Status
	{
		if power_pool == nil || slot < 0
		{
			return .Invalid_Count;
		}
		required_word_capacity := (slot >> 6) + 1;
		if required_word_capacity <= power_pool.debug.word_capacity
		{
			return .Ok;
		}
		new_capacity, capacity_status := power_pool_geometric_capacity(
			power_pool.debug.word_capacity, required_word_capacity, 1,
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
		if new_capacity > max(int) / size_of(u64)
		{
			return .Overflow;
		}
		memory, allocation_error := mem.alloc( // odin-contracts-allow: cold-metadata rule=ODIN_HOT_ALLOCATE_OR_REALLOC owner=Buffer_Pool phase=debug_take reason=debug_only_outstanding_slot_tracking
			size_of(u64) * new_capacity, align_of(u64), pool.allocator,
		);
		if allocation_error != nil || memory == nil
		{
			return .Out_Of_Memory;
		}
		new_words := ([^]u64)(memory);
		if power_pool.debug.word_capacity > 0
		{
			intrinsics.mem_copy(
				new_words, power_pool.debug.outstanding_words,
				size_of(u64) * power_pool.debug.word_capacity,
			);
		}
		if power_pool.debug.outstanding_words != nil
		{
			_ = allocation_free(power_pool.debug.outstanding_words, size_of(u64) * power_pool.debug.word_capacity, align_of(u64), pool.allocator, pool.allocation_scope); // odin-contracts-allow: cold-metadata rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=debug_metadata_growth reason=replace_outstanding_slot_tracking
		}
		power_pool.debug.outstanding_words = new_words;
		power_pool.debug.word_capacity = new_capacity;
		return .Ok;
	}
	power_pool_debug_slot_state :: proc(
		power_pool: ^Power_Pool, slot: int,
	) -> Allocation_State
	{
		if power_pool == nil || slot < 0
		{
			return .Unallocated;
		}
		word_index := slot >> 6;
		if word_index >= power_pool.debug.word_capacity
		{
			return .Unallocated;
		}
		bit := u64(1) << uint(slot & 63);
		if power_pool.debug.outstanding_words[word_index] & bit != 0
		{
			return .Allocated;
		}
		return .Unallocated;
	}
	power_pool_debug_set_slot_state :: proc(
		power_pool: ^Power_Pool, slot: int, state: Allocation_State,
	)
	{
		word_index := slot >> 6;
		bit := u64(1) << uint(slot & 63);
		switch state
		{
			case .Allocated:
			power_pool.debug.outstanding_words[word_index] |= bit;
			case .Unallocated:
			power_pool.debug.outstanding_words[word_index] &~= bit;
		}
	}
}

power_pool_initialize_metadata :: proc(
	pool: ^Buffer_Pool,
	power_pool: ^Power_Pool, expected_pooled_resource_count: int,
) -> Memory_Status
{
	blocks_status: Memory_Status = power_pool_resize_blocks(pool, power_pool, 1);
	if blocks_status != .Ok
	{
		return blocks_status;
	}
	free_status: Memory_Status = power_pool_resize_free_slots(pool, power_pool, expected_pooled_resource_count);
	if free_status != .Ok
	{
		_ = allocation_free(power_pool.blocks, size_of(rawptr) * power_pool.block_capacity, align_of(rawptr), pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=initialize_failure reason=release_block_reference_storage
		power_pool.blocks = nil;
		power_pool.block_capacity = 0;
		return free_status;
	}
	return .Ok;
}

power_pool_release_blocks :: proc(pool: ^Buffer_Pool, power_pool: ^Power_Pool)
{
	if power_pool == nil
	{
		return;
	}
	for block_index in 0 ..< power_pool.block_count
	{
		if power_pool.blocks[block_index] != nil
		{
			_ = allocation_free(power_pool.blocks[block_index], power_pool.block_size, BUFFER_POOL_BLOCK_ALIGNMENT, pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=clear reason=release_native_block
			power_pool.blocks[block_index] = nil;
		}
	}
	power_pool.block_count = 0;
	power_pool.free_count = 0;
	power_pool.next_slot = 0;
	when ODIN_DEBUG
	{
		if power_pool.debug.outstanding_words != nil && power_pool.debug.word_capacity > 0
		{
			intrinsics.mem_zero(
				power_pool.debug.outstanding_words,
				size_of(u64) * power_pool.debug.word_capacity,
			);
		}
	}
}

power_pool_release_metadata :: proc(pool: ^Buffer_Pool, power_pool: ^Power_Pool)
{
	if power_pool == nil
	{
		return;
	}
	if power_pool.blocks != nil
	{
		_ = allocation_free(power_pool.blocks, size_of(rawptr) * power_pool.block_capacity, align_of(rawptr), pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=dispose reason=release_block_reference_storage
	}
	if power_pool.free_slots != nil
	{
		_ = allocation_free(power_pool.free_slots, size_of(i32) * power_pool.slot_capacity, align_of(i32), pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=dispose reason=release_returned_id_storage
	}
	when ODIN_DEBUG
	{
		if power_pool.debug.outstanding_words != nil
		{
			_ = allocation_free(power_pool.debug.outstanding_words, size_of(u64) * power_pool.debug.word_capacity, align_of(u64), pool.allocator, pool.allocation_scope); // odin-contracts-allow: cold-metadata rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=dispose reason=release_outstanding_slot_tracking
		}
		power_pool.debug = {};
	}
	power_pool.blocks = nil;
	power_pool.free_slots = nil;
	power_pool.block_capacity = 0;
	power_pool.block_count = 0;
	power_pool.slot_capacity = 0;
	power_pool.free_count = 0;
	power_pool.next_slot = 0;
}

power_pool_ensure_block_pointer_capacity :: proc(
	pool: ^Buffer_Pool,
	power_pool: ^Power_Pool, required_capacity: int, geometric: bool,
) -> Memory_Status
{
	if power_pool == nil || required_capacity <= 0
	{
		return .Invalid_Count;
	}
	if required_capacity <= power_pool.block_capacity
	{
		return .Ok;
	}
	target_capacity := required_capacity;
	if geometric
	{
		capacity, capacity_status := power_pool_geometric_capacity(
			power_pool.block_capacity, required_capacity, 1,
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
		target_capacity = capacity;
	}
	return power_pool_resize_blocks(pool, power_pool, target_capacity);
}

power_pool_ensure_block_count :: proc(
	pool: ^Buffer_Pool,
	power_pool: ^Power_Pool, needed_block_count: int, geometric_metadata: bool,
) -> Memory_Status
{
	if power_pool == nil || needed_block_count < 0
	{
		return .Invalid_Count;
	}
	if needed_block_count <= power_pool.block_count
	{
		return .Ok;
	}
	maximum_block_count :=
	(BUFFER_POOL_ID_SLOT_COUNT + power_pool.suballocations_per_block - 1) /
	power_pool.suballocations_per_block;
	if needed_block_count > maximum_block_count
	{
		return .Overflow;
	}
	metadata_status: Memory_Status = power_pool_ensure_block_pointer_capacity(pool,
		power_pool, needed_block_count, geometric_metadata,
	);
	if metadata_status != .Ok
	{
		return metadata_status;
	}
	if pool.allocation_scope == .All_Owned
	{
		if power_pool.suballocations_per_block > max(int) / needed_block_count
		{
			return .Overflow;
		}
		slot_count: int = min(BUFFER_POOL_ID_SLOT_COUNT,
			needed_block_count * power_pool.suballocations_per_block);
		if slot_count > power_pool.slot_capacity
		{
			capacity: int;
			status: Memory_Status;
			capacity, status = power_pool_geometric_capacity(
				power_pool.slot_capacity, slot_count, BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT,
			);
			if status != .Ok
			{
				return status;
			}
			status = power_pool_resize_free_slots(pool, power_pool, capacity);
			if status != .Ok
			{
				return status;
			}
		}
		when ODIN_DEBUG
		{
			status: Memory_Status = power_pool_debug_ensure_slot_capacity(pool, power_pool, slot_count - 1);
			if status != .Ok
			{
				return status;
			}
		}
	}
	original_block_count := power_pool.block_count;
	for block_index in original_block_count ..< needed_block_count
	{
		memory, allocation_error := mem.alloc(
			power_pool.block_size, BUFFER_POOL_BLOCK_ALIGNMENT, pool.allocator,
		);
		allocation_status: Memory_Status = Memory_Status.Ok;
		if allocation_error != nil || memory == nil
		{
			allocation_status = .Out_Of_Memory;
		} // odin-contracts-allow: setup rule=ODIN_HOT_GENERAL_ALLOC_CALL owner=Buffer_Pool phase=take reason=entasis_on_demand_native_block_growth
		if allocation_status != .Ok
		{
			for release_index in original_block_count ..< block_index
			{
				_ = allocation_free(power_pool.blocks[release_index], power_pool.block_size, BUFFER_POOL_BLOCK_ALIGNMENT, pool.allocator, pool.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Buffer_Pool phase=growth_failure reason=release_partial_native_blocks
				power_pool.blocks[release_index] = nil;
			}
			return allocation_status;
		}
		power_pool.blocks[block_index] = memory;
	}
	power_pool.block_count = needed_block_count;
	return .Ok;
}

buffer_pool_initialize :: proc(
	pool: ^Buffer_Pool, minimum_block_size: int = 131072,
	expected_pooled_resource_count: int = BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT,
) -> Memory_Status
{
	return buffer_pool_initialize_internal(pool, minimum_block_size,
		expected_pooled_resource_count, runtime.heap_allocator(), .Legacy);
}

buffer_pool_initialize_with_allocator :: proc(
	pool: ^Buffer_Pool, allocator: mem.Allocator,
	minimum_block_size: int = 131072,
	expected_pooled_resource_count: int = BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT,
) -> Memory_Status
{
	return buffer_pool_initialize_internal(pool, minimum_block_size,
		expected_pooled_resource_count, allocation_allocator(allocator), .All_Owned);
}

buffer_pool_initialize_internal :: proc(
	pool: ^Buffer_Pool, minimum_block_size, expected_pooled_resource_count: int,
	allocator: mem.Allocator, scope: Allocation_Scope,
) -> Memory_Status
{
	if pool == nil || power_of_two_state(minimum_block_size) != .Allocated
	{
		return .Invalid_Alignment;
	}
	if expected_pooled_resource_count <= 0
	{
		return .Invalid_Count;
	}
	if pool.state != .Uninitialized
	{
		return .Invalid_Buffer;
	}
	if minimum_block_size > 1 << uint(MAXIMUM_SPAN_SIZE_POWER)
	{
		return .Overflow;
	}
	pool^ = {};
	pool.minimum_block_size = minimum_block_size;
	pool.allocator = allocator;
	pool.allocation_scope = scope;
	for power in 0 ..= MAXIMUM_SPAN_SIZE_POWER
	{
		suballocation_size := 1 << uint(power);
		block_size := max(suballocation_size, minimum_block_size);
		suballocations_per_block := block_size / suballocation_size;
		shift, shift_status := containing_power_of_two(suballocations_per_block);
		if shift_status != .Ok
		{
			for release_power in 0 ..< power
			{
				power_pool_release_metadata(pool, &pool.pools[release_power]);
			}
			pool^ = {};
			return shift_status;
		}
		power_pool := &pool.pools[power];
		power_pool.power = power;
		power_pool.suballocation_size = suballocation_size;
		power_pool.block_size = block_size;
		power_pool.suballocations_per_block = suballocations_per_block;
		power_pool.suballocations_per_block_shift = shift;
		power_pool.suballocations_per_block_mask = suballocations_per_block - 1;
		metadata_status: Memory_Status = power_pool_initialize_metadata(pool,
			power_pool, expected_pooled_resource_count,
		);
		if metadata_status != .Ok
		{
			for release_power in 0 ..= power
			{
				power_pool_release_metadata(pool, &pool.pools[release_power]);
			}
			pool^ = {};
			return metadata_status;
		}
	}
	pool.state = .Ready;
	return .Ok;
}

buffer_pool_ensure_capacity_for_power :: proc(
	pool: ^Buffer_Pool, byte_count, power: int,
) -> Memory_Status
{
	if pool == nil || pool.state != .Ready
	{
		return .Pool_Disposed;
	}
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return .Invalid_Power;
	}
	if byte_count < 0
	{
		return .Invalid_Count;
	}
	power_pool := &pool.pools[power];
	needed_block_count := 0;
	if byte_count > 0
	{
		needed_block_count = 1 + (byte_count - 1) / power_pool.block_size;
	}
	return power_pool_ensure_block_count(pool, power_pool, needed_block_count, false);
}

buffer_pool_take_for_power :: proc(
	pool: ^Buffer_Pool, power: int,
) -> (buffer: Buffer(u8), status: Memory_Status)
{
	if pool == nil || pool.state != .Ready
	{
		return {}, .Pool_Disposed;
	}
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return {}, .Invalid_Power;
	}
	power_pool := &pool.pools[power];
	slot := 0;
	if power_pool.free_count > 0
	{
		when ODIN_DEBUG
		{
			slot = int(power_pool.free_slots[power_pool.free_count - 1]);
			if slot < 0 || slot >= power_pool.next_slot ||
			slot >> 6 >= power_pool.debug.word_capacity
			{
				return {}, .Invalid_Buffer;
			}
			if power_pool_debug_slot_state(power_pool, slot) == .Allocated
			{
				return {}, .Invalid_Buffer;
			}
			power_pool.free_count -= 1;
		}
		else
		{
			power_pool.free_count -= 1;
			slot = int(power_pool.free_slots[power_pool.free_count]);
			if slot < 0 || slot >= power_pool.next_slot
			{
				return {}, .Invalid_Buffer;
			}
		}
	}
	else
	{
		if power_pool.next_slot >= BUFFER_POOL_ID_SLOT_COUNT
		{
			return {}, .Overflow;
		}
		slot = power_pool.next_slot;
		when ODIN_DEBUG
		{
			debug_capacity_status: Memory_Status = power_pool_debug_ensure_slot_capacity(pool, power_pool, slot);
			if debug_capacity_status != .Ok
			{
				return {}, debug_capacity_status;
			}
			if power_pool_debug_slot_state(power_pool, slot) == .Allocated
			{
				return {}, .Invalid_Buffer;
			}
		}
		block_index := slot >> uint(power_pool.suballocations_per_block_shift);
		block_status: Memory_Status = power_pool_ensure_block_count(pool, power_pool, block_index + 1, true);
		if block_status != .Ok
		{
			return {}, block_status;
		}
		power_pool.next_slot += 1;
	}
	block_index := slot >> uint(power_pool.suballocations_per_block_shift);
	if block_index < 0 || block_index >= power_pool.block_count
	{
		return {}, .Invalid_Buffer;
	}
	index_in_block := slot & power_pool.suballocations_per_block_mask;
	memory := ([^]u8)(
		uintptr(power_pool.blocks[block_index]) +
		uintptr(index_in_block * power_pool.suballocation_size),
	);
	buffer = {
		memory=memory,
		length=i32(power_pool.suballocation_size),
		id=i32((power << BUFFER_POOL_ID_POWER_SHIFT) | slot),
	};
	when ODIN_DEBUG
	{
		power_pool_debug_set_slot_state(power_pool, slot, .Allocated);
	}
	return buffer, .Ok;
}

buffer_pool_power_for_count :: proc(
	$T: typeid, count: int,
) -> (power: int, status: Memory_Status)
{
	if count < 0 || count > max(int) / size_of(T)
	{
		return 0, .Invalid_Count;
	}
	requested_count := max(count, 1);
	return containing_power_of_two(requested_count * size_of(T));
}

buffer_pool_capacity_for_count :: proc(
	$T: typeid, count: int,
) -> (capacity: int, status: Memory_Status)
{
	power, power_status := buffer_pool_power_for_count(T, count);
	if power_status != .Ok
	{
		return 0, power_status;
	}
	return (1 << uint(power)) / size_of(T), .Ok;
}

buffer_pool_available_slot_count :: proc "contextless"(
	pool: ^Buffer_Pool, power: int,
) -> (int, Memory_Status)
{
	if pool == nil || pool.state != .Ready
	{
		return 0, .Pool_Disposed;
	}
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return 0, .Invalid_Power;
	}
	power_pool := &pool.pools[power];
	allocated_slot_count := power_pool.block_count * power_pool.suballocations_per_block;
	if power_pool.next_slot < 0 || power_pool.next_slot > allocated_slot_count ||
	power_pool.free_count < 0 || power_pool.free_count > power_pool.slot_capacity
	{
		return 0, .Invalid_Buffer;
	}
	return power_pool.free_count + allocated_slot_count - power_pool.next_slot, .Ok;
}

buffer_pool_ensure_available_slot_count :: proc(
	pool: ^Buffer_Pool, power, required_count: int,
) -> Memory_Status
{
	if pool == nil || pool.state != .Ready
	{
		return .Pool_Disposed;
	}
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return .Invalid_Power;
	}
	if required_count < 0
	{
		return .Invalid_Count;
	}
	if required_count == 0
	{
		return .Ok;
	}
	power_pool := &pool.pools[power];
	available, available_status := buffer_pool_available_slot_count(pool, power);
	if available_status != .Ok
	{
		return available_status;
	}
	if required_count <= available
	{
		return .Ok;
	}
	new_slot_count := required_count - power_pool.free_count;
	if new_slot_count <= 0
	{
		return .Ok;
	}
	if power_pool.next_slot > BUFFER_POOL_ID_SLOT_COUNT - new_slot_count
	{
		return .Overflow;
	}
	target_next_slot := power_pool.next_slot + new_slot_count;
	needed_block_count :=
	(target_next_slot + power_pool.suballocations_per_block - 1) /
	power_pool.suballocations_per_block;
	return power_pool_ensure_block_count(pool, power_pool, needed_block_count, true);
}

buffer_pool_take_at_least :: proc(
	pool: ^Buffer_Pool, $T: typeid, count: int,
) -> (buffer: Buffer(T), status: Memory_Status)
{
	capacity, capacity_status := buffer_pool_capacity_for_count(T, count);
	if capacity_status != .Ok
	{
		return {}, capacity_status;
	}
	power, power_status := buffer_pool_power_for_count(T, count);
	if power_status != .Ok
	{
		return {}, power_status;
	}
	raw_buffer, take_status := buffer_pool_take_for_power(pool, power);
	if take_status != .Ok
	{
		return {}, take_status;
	}
	return {
		memory=([^]T)(raw_buffer.memory),
		length=i32(capacity),
		id=raw_buffer.id,
	}, .Ok;
}

buffer_pool_take :: proc(
	pool: ^Buffer_Pool, $T: typeid, count: int,
) -> (buffer: Buffer(T), status: Memory_Status)
{
	buffer, status = buffer_pool_take_at_least(pool, T, count);
	if status == .Ok
	{
		buffer.length = i32(count);
	}
	return;
}

when ODIN_DEBUG
{
	buffer_pool_debug_validate_return :: proc(
		pool: ^Buffer_Pool, buffer: ^Buffer($T),
	) -> Memory_Status
	{
		if pool == nil || pool.state != .Ready
		{
			return .Pool_Disposed;
		}
		if buffer == nil || buffer.memory == nil || buffer.id < 0
		{
			return .Invalid_Buffer;
		}
		power := int(buffer.id) >> BUFFER_POOL_ID_POWER_SHIFT;
		slot := int(buffer.id) & BUFFER_POOL_ID_SLOT_MASK;
		if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
		{
			return .Invalid_Buffer;
		}
		power_pool := &pool.pools[power];
		if slot < 0 || slot >= power_pool.next_slot
		{
			return .Invalid_Buffer;
		}
		if power_pool_debug_slot_state(power_pool, slot) != .Allocated
		{
			return .Invalid_Buffer;
		}
		block_index := slot >> uint(power_pool.suballocations_per_block_shift);
		if block_index < 0 || block_index >= power_pool.block_count
		{
			return .Invalid_Buffer;
		}
		index_in_block := slot & power_pool.suballocations_per_block_mask;
		expected_address :=
		uintptr(power_pool.blocks[block_index]) +
		uintptr(index_in_block * power_pool.suballocation_size);
		if uintptr(buffer.memory) != expected_address
		{
			return .Invalid_Buffer;
		}
		length := int(buffer.length);
		if length < 0 || length > power_pool.suballocation_size / size_of(T)
		{
			return .Invalid_Buffer;
		}
		return .Ok;
	}
}

buffer_pool_return_unsafely :: proc(pool: ^Buffer_Pool, id: i32) -> Memory_Status
{
	if pool == nil || pool.state != .Ready
	{
		return .Pool_Disposed;
	}
	if id < 0
	{
		return .Invalid_Buffer;
	}
	power := int(id) >> BUFFER_POOL_ID_POWER_SHIFT;
	slot := int(id) & BUFFER_POOL_ID_SLOT_MASK;
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return .Invalid_Buffer;
	}
	power_pool := &pool.pools[power];
	if slot < 0 || slot >= power_pool.next_slot
	{
		return .Invalid_Buffer;
	}
	when ODIN_DEBUG
	{
		if power_pool_debug_slot_state(power_pool, slot) != .Allocated
		{
			return .Invalid_Buffer;
		}
	}
	if power_pool.free_count == power_pool.slot_capacity
	{
		new_capacity, capacity_status := power_pool_geometric_capacity(
			power_pool.slot_capacity, power_pool.free_count + 1,
			BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT,
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
		resize_status: Memory_Status = power_pool_resize_free_slots(pool, power_pool, new_capacity);
		if resize_status != .Ok
		{
			return resize_status;
		}
	}
	when ODIN_DEBUG
	{
		power_pool_debug_set_slot_state(power_pool, slot, .Unallocated);
	}
	power_pool.free_slots[power_pool.free_count] = i32(slot);
	power_pool.free_count += 1;
	return .Ok;
}

buffer_pool_return :: proc(pool: ^Buffer_Pool, buffer: ^Buffer($T)) -> Memory_Status
{
	if buffer == nil || buffer.memory == nil
	{
		return .Invalid_Buffer;
	}
	when ODIN_DEBUG
	{
		validation_status := buffer_pool_debug_validate_return(pool, buffer);
		if validation_status != .Ok
		{
			return validation_status;
		}
	}
	status := buffer_pool_return_unsafely(pool, buffer.id);
	if status == .Ok
	{
		buffer^ = {};
	}
	return status;
}

buffer_pool_resize_to_at_least :: proc(
	pool: ^Buffer_Pool, buffer: ^Buffer($T), target_size, copy_count: int,
) -> Memory_Status
{
	if buffer == nil || target_size < 0 || copy_count < 0 ||
	copy_count > target_size || copy_count > int(buffer.length)
	{
		return .Invalid_Count;
	}
	capacity, capacity_status := buffer_pool_capacity_for_count(T, target_size);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	if buffer.memory == nil
	{
		new_buffer, take_status := buffer_pool_take_at_least(pool, T, target_size);
		if take_status == .Ok
		{
			buffer^ = new_buffer;
		}
		return take_status;
	}
	current_capacity, current_capacity_status := buffer_capacity(buffer^);
	if current_capacity_status != .Ok
	{
		return current_capacity_status;
	}
	if capacity <= current_capacity
	{
		buffer.length = i32(current_capacity);
		return .Ok;
	}
	new_buffer, take_status := buffer_pool_take_at_least(pool, T, target_size);
	if take_status != .Ok
	{
		return take_status;
	}
	copy_status := buffer_copy(
		buffer_view(buffer^), 0, buffer_view(new_buffer), 0, copy_count,
	);
	if copy_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_buffer);
		return copy_status;
	}
	return_status := buffer_pool_return(pool, buffer);
	if return_status != .Ok
	{
		_ = buffer_pool_return(pool, &new_buffer);
		return return_status;
	}
	buffer^ = new_buffer;
	return .Ok;
}

buffer_pool_resize :: proc(
	pool: ^Buffer_Pool, buffer: ^Buffer($T), target_size, copy_count: int,
) -> Memory_Status
{
	status := buffer_pool_resize_to_at_least(pool, buffer, target_size, copy_count);
	if status == .Ok
	{
		buffer.length = i32(target_size);
	}
	return status;
}

buffer_pool_capacity_for_power :: proc(
	pool: ^Buffer_Pool, power: int,
) -> (int, Memory_Status)
{
	if pool == nil || pool.state != .Ready
	{
		return 0, .Pool_Disposed;
	}
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return 0, .Invalid_Power;
	}
	power_pool := &pool.pools[power];
	return power_pool.block_count * power_pool.block_size, .Ok;
}

buffer_pool_total_allocated_byte_count :: proc(pool: ^Buffer_Pool) -> u64
{
	if pool == nil || pool.state != .Ready
	{
		return 0;
	}
	total := u64(0);
	for power_pool in pool.pools
	{
		total += u64(power_pool.block_count) * u64(power_pool.block_size);
	}
	return total;
}

buffer_pool_clear :: proc(pool: ^Buffer_Pool) -> Memory_Status
{
	if pool == nil || pool.state != .Ready
	{
		return .Invalid_Buffer;
	}
	for power in 0 ..= MAXIMUM_SPAN_SIZE_POWER
	{
		power_pool_release_blocks(pool, &pool.pools[power]);
	}
	return .Ok;
}

buffer_pool_dispose :: proc(pool: ^Buffer_Pool) -> Memory_Status
{
	if pool == nil || pool.state != .Ready
	{
		return .Pool_Disposed;
	}
	_ = buffer_pool_clear(pool);
	for power in 0 ..= MAXIMUM_SPAN_SIZE_POWER
	{
		power_pool_release_metadata(pool, &pool.pools[power]);
	}
	pool.state = .Disposed;
	return .Ok;
}

worker_buffer_pools_initialize :: proc(
	workers: ^Worker_Buffer_Pools, worker_count: int,
	default_block_capacity: int = 16384,
) -> Memory_Status
{
	return worker_buffer_pools_initialize_internal(workers, worker_count,
		default_block_capacity, runtime.heap_allocator(), .Legacy);
}

worker_buffer_pools_initialize_with_allocator :: proc(
	workers: ^Worker_Buffer_Pools, worker_count: int, allocator: mem.Allocator,
	default_block_capacity: int = 16384,
) -> Memory_Status
{
	return worker_buffer_pools_initialize_internal(workers, worker_count,
		default_block_capacity, allocation_allocator(allocator), .All_Owned);
}

worker_buffer_pools_initialize_internal :: proc(
	workers: ^Worker_Buffer_Pools, worker_count, default_block_capacity: int,
	allocator: mem.Allocator, scope: Allocation_Scope,
) -> Memory_Status
{
	if workers == nil || worker_count <= 0 || worker_count > max(int) / size_of(Buffer_Pool) || workers.state != .Uninitialized
	{
		return .Invalid_Count;
	}
	if power_of_two_state(default_block_capacity) != .Allocated ||
	default_block_capacity > 1 << uint(MAXIMUM_SPAN_SIZE_POWER)
	{
		return .Invalid_Alignment;
	}
	memory, allocator_error := mem.alloc(
		size_of(Buffer_Pool) * worker_count, align_of(Buffer_Pool),
		allocator,
	); // odin-contracts-allow: setup rule=ODIN_HOT_GENERAL_ALLOC_CALL owner=Worker_Buffer_Pools phase=initialize reason=explicit_worker_capacity
	if allocator_error != nil || memory == nil
	{
		return .Out_Of_Memory;
	}
	workers^ = {
		pools=([^]Buffer_Pool)(memory),
		worker_count=worker_count,
		default_block_capacity=default_block_capacity,
		state=.Ready,
		allocator=allocator, allocation_scope=scope,
	};
	for worker_index in 0 ..< worker_count
	{
		status: Memory_Status = buffer_pool_initialize_internal(
			&workers.pools[worker_index], default_block_capacity,
			BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT, allocator, scope,
		);
		if status != .Ok
		{
			for initialized_index in 0 ..< worker_index
			{
				_ = buffer_pool_dispose(&workers.pools[initialized_index]);
			}
			_ = allocation_free(workers.pools, size_of(Buffer_Pool) * workers.worker_count, align_of(Buffer_Pool), workers.allocator, workers.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Worker_Buffer_Pools phase=initialize_failure reason=release_partial_worker_pools
			workers^ = {};
			return status;
		}
	}
	return .Ok;
}

worker_buffer_pool_get :: proc(
	workers: ^Worker_Buffer_Pools, worker_index: int,
) -> (^Buffer_Pool, Memory_Status)
{
	if workers == nil || workers.state != .Ready ||
	worker_index < 0 || worker_index >= workers.worker_count
	{
		return nil, .Invalid_Count;
	}
	return &workers.pools[worker_index], .Ok;
}

worker_buffer_pools_total_allocated_byte_count :: proc(
	workers: ^Worker_Buffer_Pools,
) -> u64
{
	if workers == nil || workers.state != .Ready
	{
		return 0;
	}
	total := u64(0);
	for worker_index in 0 ..< workers.worker_count
	{
		total += buffer_pool_total_allocated_byte_count(&workers.pools[worker_index]);
	}
	return total;
}

worker_buffer_pools_clear :: proc(workers: ^Worker_Buffer_Pools) -> Memory_Status
{
	if workers == nil || workers.state != .Ready
	{
		return .Pool_Disposed;
	}
	for worker_index in 0 ..< workers.worker_count
	{
		status := buffer_pool_clear(&workers.pools[worker_index]);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

worker_buffer_pools_dispose :: proc(workers: ^Worker_Buffer_Pools) -> Memory_Status
{
	if workers == nil || workers.state != .Ready
	{
		return .Pool_Disposed;
	}
	for worker_index in 0 ..< workers.worker_count
	{
		_ = buffer_pool_dispose(&workers.pools[worker_index]);
	}
	_ = allocation_free(workers.pools, size_of(Buffer_Pool) * workers.worker_count, align_of(Buffer_Pool), workers.allocator, workers.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Worker_Buffer_Pools phase=dispose reason=explicit_worker_pool_teardown
	workers^ = Worker_Buffer_Pools{state=.Disposed};
	return .Ok;
}
