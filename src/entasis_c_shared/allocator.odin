package entasis_c_shared

import "base:intrinsics"
import "base:runtime"
import "core:mem"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

Allocate_Proc :: #type proc "c" (user_context: rawptr, size, alignment: u64) -> rawptr;
Reallocate_Proc :: #type proc "c" (user_context, memory: rawptr, old_size, new_size, alignment: u64) -> rawptr;
Deallocate_Proc :: #type proc "c" (user_context, memory: rawptr, size, alignment: u64);
// copied once into each stable resource. the public descriptor is never borrowed
// by mem.allocator.data. its scope occupies the old descriptor-header space
Allocator :: struct
{
	scope: util.Allocation_Scope,
	user_context: rawptr,
	allocate: Allocate_Proc,
	reallocate: Reallocate_Proc,
	deallocate: Deallocate_Proc,
}

Allocator_To_Core :: proc (allocator: ^Allocator) -> mem.Allocator
{
	if allocator == nil || allocator.allocate == nil
	{
		return runtime.heap_allocator();
	}
	return {procedure=Allocator_Proc, data=allocator};
}

Resource_Allocate :: proc (
	size, alignment: int, allocator: ^Allocator,
) -> (rawptr, Allocator, entasis.Status)
{
	copy: Allocator;
	if allocator != nil
	{
		copy = allocator^;
	}
	memory, error := mem.alloc(size, alignment, Allocator_To_Core(&copy));
	if error != nil || memory == nil
	{
		return nil, copy, .Capacity_Missing;
	}
	return memory, copy, .Ok;
}

Resource_Free :: proc (
	memory: rawptr, size, alignment: int, allocator: ^Allocator,
)
{
	if memory == nil
	{
		return;
	}
	// rebind a final root free to the saved copy, never to the cleared/freed owner
	copy := allocator^;
	_ = util.allocation_free(memory, size, alignment, Allocator_To_Core(&copy), .All_Owned);
}

Allocator_Proc :: proc (
	allocator_data: rawptr,
	mode: runtime.Allocator_Mode,
	size, alignment: int,
	old_memory: rawptr,
	old_size: int,
	location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, runtime.Allocator_Error)
{
	_ = location;
	if allocator_data == nil
	{
		return nil, .Invalid_Argument;
	}
	allocator := (^Allocator)(allocator_data);
	if allocator.allocate == nil || allocator.deallocate == nil
	{
		return nil, .Invalid_Argument;
	}
	if size < 0 || old_size < 0 || alignment < 0
	{
		return nil, .Invalid_Argument;
	}
	switch mode
	{
		case .Alloc, .Alloc_Non_Zeroed:
		if size == 0
		{
			return nil, nil;
		}
		memory := allocator.allocate(
			allocator.user_context,
			u64(size),
			u64(max(alignment, 1)),
		);
		if memory == nil
		{
			return nil, .Out_Of_Memory;
		}
		if mode == .Alloc
		{
			intrinsics.mem_zero(memory, size);
		}
		return (cast([^]byte)memory)[:size], nil;
		case .Free:
		if old_memory != nil && allocator.scope == .All_Owned && (old_size <= 0 || alignment <= 0)
		{
			return nil, .Invalid_Argument;
		}
		if old_memory != nil
		{
			allocator.deallocate(
				allocator.user_context,
				old_memory,
				u64(max(old_size, 0)),
				u64(max(alignment, 0)),
			);
		}
		return nil, nil;
		case .Resize, .Resize_Non_Zeroed:
		if old_memory != nil && allocator.scope == .All_Owned && (old_size <= 0 || alignment <= 0)
		{
			return nil, .Invalid_Argument;
		}
		if old_memory == nil
		{
			if size == 0
			{
				return nil, nil;
			}
			memory := allocator.allocate(
				allocator.user_context, u64(size), u64(max(alignment, 1)),
			);
			if memory == nil
			{
				return nil, .Out_Of_Memory;
			}
			if mode == .Resize
			{
				intrinsics.mem_zero(memory, size);
			}
			return (cast([^]byte)memory)[:size], nil;
		}
		if size == 0
		{
			allocator.deallocate(
				allocator.user_context, old_memory, u64(max(old_size, 0)), u64(max(alignment, 0)),
			);
			return nil, nil;
		}
		memory: rawptr;
		if allocator.reallocate != nil
		{
			memory = allocator.reallocate(
				allocator.user_context,
				old_memory,
				u64(max(old_size, 0)),
				u64(size),
				u64(max(alignment, 1)),
			);
		}
		else
		{
			memory = allocator.allocate(
				allocator.user_context, u64(size), u64(max(alignment, 1)),
			);
			if memory != nil
			{
				copy_size := min(old_size, size);
				if copy_size > 0
				{
					intrinsics.mem_copy_non_overlapping(memory, old_memory, copy_size);
				}
				allocator.deallocate(
					allocator.user_context,
					old_memory,
					u64(max(old_size, 0)),
					u64(max(alignment, 0)),
				);
			}
		}
		if memory == nil
		{
			return nil, .Out_Of_Memory;
		}
		if mode == .Resize && size > old_size
		{
			intrinsics.mem_zero(rawptr(uintptr(memory) + uintptr(old_size)), size - old_size);
		}
		return (cast([^]byte)memory)[:size], nil;
		case .Query_Features:
		set := (^runtime.Allocator_Mode_Set)(old_memory);
		if set != nil
		{
			set^ = {
				.Alloc, .Alloc_Non_Zeroed, .Free, .Resize,
				.Resize_Non_Zeroed, .Query_Features,
			};
		}
		return nil, nil;
		case .Free_All, .Query_Info:
		return nil, .Mode_Not_Implemented;
	}
	return nil, .Mode_Not_Implemented;
}
