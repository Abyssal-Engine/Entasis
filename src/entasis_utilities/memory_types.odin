// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"
import "base:runtime"
import "core:mem"

MAXIMUM_SPAN_SIZE_POWER :: 30;
BUFFER_POOL_POWER_COUNT :: MAXIMUM_SPAN_SIZE_POWER + 1;
BUFFER_POOL_ID_POWER_SHIFT :: 26;
BUFFER_POOL_ID_SLOT_MASK :: (1 << BUFFER_POOL_ID_POWER_SHIFT) - 1;
BUFFER_POOL_BLOCK_ALIGNMENT :: 128;
Memory_Status :: enum u8
{
	Ok,
	Invalid_Count,
	Invalid_Alignment,
	Invalid_Power,
	Invalid_Buffer,
	Capacity_Missing,
	Out_Of_Memory,
	Overflow,
	Pool_Disposed,
}

Allocation_State :: enum u8
{
	Unallocated,
	Allocated,
}

Pool_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

// Legacy preserves lazy pool metadata and existing allocator callback semantics.
// All_Owned retains each resource's allocator and makes valid buffer returns allocation-free
Allocation_Scope :: enum u8
{
	Legacy,
	All_Owned,
}

allocation_allocator :: #force_inline proc (allocator: mem.Allocator) -> mem.Allocator
{
	if allocator.procedure == nil
	{
		return runtime.heap_allocator();
	}
	return allocator;
}

// cold owner release. free in All_Owned retains the allocation's exact request.
// mem.free and mem.free_with_size discard its alignment
allocation_free :: proc (
	memory: rawptr, byte_count, alignment: int,
	allocator: mem.Allocator, scope: Allocation_Scope,
	location: runtime.Source_Code_Location = #caller_location,
) -> mem.Allocator_Error
{
	if memory == nil
	{
		return nil;
	}
	if scope == .Legacy
	{
		return mem.free(memory, allocator);
	}
	error: mem.Allocator_Error;
	_, error = allocator.procedure(
		allocator.data, .Free, 0, alignment, memory, byte_count, location,
	);
	return error;
}

Aligned_Allocation :: struct
{
	memory:     rawptr,
	byte_count: int,
	alignment:  int,
}

Buffer :: struct($T: typeid)
{
	memory: [^]T,
	length: i32,
	id:     i32,
}

BUFFER_CALLER_OWNED_ID :: i32(-1);
Buffer_View :: struct($T: typeid)
{
	memory:   [^]T,
	length:   int,
	capacity: int,
}

containing_power_of_two :: proc (value: int) -> (power: int, status: Memory_Status)
{
	if value < 0
	{
		return 0, .Invalid_Count;
	}
	unsigned := u32(1);
	if value > 0
	{
		unsigned = u32(value);
	}
	power = 32 - int(intrinsics.count_leading_zeros(unsigned - 1));
	if power > MAXIMUM_SPAN_SIZE_POWER
	{
		return 0, .Overflow;
	}
	return power, .Ok;
}

power_of_two_state :: proc (value: int) -> Allocation_State
{
	if value > 0 && (value & (value - 1)) == 0
	{
		return .Allocated;
	}
	return .Unallocated;
}

aligned_allocate :: proc (byte_count, alignment: int) -> (allocation: Aligned_Allocation, status: Memory_Status)
{
	if byte_count <= 0
	{
		return {}, .Invalid_Count;
	}
	if power_of_two_state(alignment) != .Allocated || alignment < int(align_of(rawptr))
	{
		return {}, .Invalid_Alignment;
	}
	memory, allocator_error := mem.alloc(byte_count, alignment, runtime.heap_allocator());
	if allocator_error != nil || memory == nil
	{
		return {}, .Out_Of_Memory;
	}
	allocation = Aligned_Allocation{memory=memory, byte_count=byte_count, alignment=alignment};
	return allocation, .Ok;
}

aligned_release :: proc (allocation: ^Aligned_Allocation) -> Memory_Status
{
	if allocation == nil || allocation.memory == nil
	{
		return .Invalid_Buffer;
	}
	allocator_error: mem.Allocator_Error = mem.free(allocation.memory, runtime.heap_allocator()); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Aligned_Allocation phase=release reason=explicit_allocation_teardown
	if allocator_error != nil
	{
		return .Invalid_Buffer;
	}
	allocation^ = {};
	return .Ok;
}

buffer_allocation_state :: proc (buffer: Buffer($T)) -> Allocation_State
{
	if buffer.memory != nil
	{
		return .Allocated;
	}
	return .Unallocated;
}

buffer_capacity :: proc (buffer: Buffer($T)) -> (capacity: int, status: Memory_Status)
{
	if buffer.memory == nil || buffer.length < 0 || buffer.id < BUFFER_CALLER_OWNED_ID
	{
		return 0, .Invalid_Buffer;
	}
	if buffer.id == BUFFER_CALLER_OWNED_ID
	{
		return int(buffer.length), .Ok;
	}
	power := int(buffer.id) >> BUFFER_POOL_ID_POWER_SHIFT;
	if power < 0 || power > MAXIMUM_SPAN_SIZE_POWER
	{
		return 0, .Invalid_Buffer;
	}
	return (1 << uint(power)) / size_of(T), .Ok;
}

buffer_view :: proc (buffer: Buffer($T)) -> Buffer_View(T)
{
	length := int(buffer.length);
	return Buffer_View(T){memory=buffer.memory, length=length, capacity=length};
}

buffer_slice :: proc (buffer: Buffer($T), start, count: int) -> (view: Buffer_View(T), status: Memory_Status)
{
	length := int(buffer.length);
	if start < 0 || count < 0 || start > length || count > length - start
	{
		return {}, .Invalid_Count;
	}
	return Buffer_View(T){memory=&buffer.memory[start], length=count, capacity=length - start}, .Ok;
}

buffer_clear :: proc (buffer: Buffer($T), start, count: int) -> Memory_Status
{
	length := int(buffer.length);
	if start < 0 || count < 0 || start > length || count > length - start
	{
		return .Invalid_Count;
	}
	if count > 0
	{
		intrinsics.mem_zero(&buffer.memory[start], size_of(T) * count);
	}
	return .Ok;
}

buffer_copy :: proc (
	source: Buffer_View($T), source_index: int,
	target: Buffer_View(T), target_index, count: int,
) -> Memory_Status
{
	if source_index < 0 || target_index < 0 || count < 0 ||
	source_index > source.length || count > source.length - source_index ||
	target_index > target.length || count > target.length - target_index
	{
		return .Invalid_Count;
	}
	if count > 0
	{
		intrinsics.mem_copy(&target.memory[target_index], &source.memory[source_index], size_of(T) * count);
	}
	return .Ok;
}
#assert(size_of(Aligned_Allocation) == 24);
#assert(align_of(Aligned_Allocation) == 8);
#assert(size_of(Buffer(u32)) == 16);
#assert(align_of(Buffer(u32)) == 8);
#assert(offset_of(Buffer(u32), length) == 8);
#assert(offset_of(Buffer(u32), id) == 12);
