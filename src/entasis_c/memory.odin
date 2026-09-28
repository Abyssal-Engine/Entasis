package entasis_c

import "base:runtime"
import "core:mem"
import shared "entasis:entasis_c_shared"
import util "entasis:entasis_utilities"
import entasis "entasis:entasis"

ENTASIS_POOL_MAGIC        :: u64(0x454E54504F4F4C31);
abi_allocator_is_default :: #force_inline proc "contextless" (allocator: ^Entasis_Allocator) -> bool
{
	return allocator == nil ||
	(allocator.allocate == nil && allocator.reallocate == nil && allocator.deallocate == nil);
}

abi_allocator_validate :: #force_inline proc "contextless" (allocator: ^Entasis_Allocator) -> entasis.Status
{
	if abi_allocator_is_default(allocator)
	{
		return .Ok;
	}
	if allocator.struct_size < u32(size_of(Entasis_Allocator)) ||
	allocator.struct_version != ENTASIS_STRUCT_VERSION ||
	allocator.allocate == nil || allocator.deallocate == nil
	{
		return .Invalid_Description;
	}
	return .Ok;
}

abi_allocator_resolve :: proc "contextless" (
	allocator: ^Entasis_Allocator, scope: util.Allocation_Scope = .Legacy,
) -> (shared.Allocator, entasis.Status)
{
	status := abi_allocator_validate(allocator);
	if status != .Ok
	{
		return {}, status;
	}
	copy := shared.Allocator{scope=scope};
	if allocator != nil
	{
		copy.user_context = allocator.user_context;
		copy.allocate = allocator.allocate;
		copy.reallocate = allocator.reallocate;
		copy.deallocate = allocator.deallocate;
	}
	return copy, .Ok;
}

abi_allocator_to_core :: proc "contextless" (
	allocator: ^shared.Allocator,
) -> (mem.Allocator, entasis.Status)
{
	context = runtime.default_context();
	return shared.Allocator_To_Core(allocator), .Ok;
}

abi_resource_allocate :: proc "contextless" (
	size, alignment: int, allocator: ^Entasis_Allocator,
	scope: util.Allocation_Scope = .Legacy,
) -> (rawptr, shared.Allocator, entasis.Status)
{
	copy, status := abi_allocator_resolve(allocator, scope);
	if status != .Ok
	{
		return nil, {}, status;
	}
	context = runtime.default_context();
	return shared.Resource_Allocate(size, alignment, &copy);
}

abi_resource_allocate_owned :: proc "contextless" (
	size, alignment: int, allocator: ^shared.Allocator,
) -> (rawptr, shared.Allocator, entasis.Status)
{
	context = runtime.default_context();
	return shared.Resource_Allocate(size, alignment, allocator);
}

abi_resource_free :: proc "contextless" (
	memory: rawptr, size, alignment: int, allocator: ^shared.Allocator,
)
{
	context = runtime.default_context();
	shared.Resource_Free(memory, size, alignment, allocator);
}

abi_buffer_pool_header :: struct
{
	magic:       u64,
	pool:        ^entasis.Buffer_Pool,
	owner:       rawptr,
	attached:    i32,
	borrowed:    bool,
	query_context_attached: abi_Attachment_State,
	reserved:    [2]u8,
}

abi_buffer_pool_resource :: struct
{
	header:    abi_buffer_pool_header,
	allocator: shared.Allocator,
	storage:   entasis.Buffer_Pool,
}

abi_buffer_pool_header_get :: #force_inline proc "contextless" (
	handle: ^Entasis_Buffer_Pool,
) -> ^abi_buffer_pool_header
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	header := (^abi_buffer_pool_header)(handle.opaque);
	if header.magic != ENTASIS_POOL_MAGIC || header.pool == nil
	{
		return nil;
	}
	return header;
}

abi_buffer_pool_init :: proc "contextless" (
	pool: ^Entasis_Buffer_Pool,
	minimum_block_size, expected_resource_count: i32,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_buffer_pool_init_extended(pool, minimum_block_size, expected_resource_count, allocator, 0, diagnostic);
}

abi_buffer_pool_init_extended :: proc "contextless" (
	pool: ^Entasis_Buffer_Pool,
	minimum_block_size, expected_resource_count: i32,
	allocator: ^Entasis_Allocator,
	allocation_scope: Entasis_Allocation_Scope,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if pool == nil || pool.opaque != nil || minimum_block_size <= 0 || expected_resource_count <= 0 || allocation_scope > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	memory, allocator_copy, allocation_status := abi_resource_allocate(
		size_of(abi_buffer_pool_resource), align_of(abi_buffer_pool_resource), allocator, entasis.Allocation_Scope(allocation_scope),
	);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .World_Initialize);
	}
	resource := (^abi_buffer_pool_resource)(memory);
	resource.allocator = allocator_copy;
	resource.header = {magic=ENTASIS_POOL_MAGIC, pool=&resource.storage};
	context = runtime.default_context();
	status := entasis.Status.Ok;
	if allocation_scope == 1
	{
		status = entasis.buffer_pool_init_with_allocator(&resource.storage,
			shared.Allocator_To_Core(&resource.allocator), int(minimum_block_size), int(expected_resource_count));
	}
	else
	{
		status = entasis.buffer_pool_init(&resource.storage, int(minimum_block_size), int(expected_resource_count));
	}
	if status != .Ok
	{
		resource.header.magic = 0;
		abi_resource_free(
			resource, size_of(abi_buffer_pool_resource), align_of(abi_buffer_pool_resource),
			&resource.allocator,
		);
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	pool.opaque = &resource.header;
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}

abi_buffer_pool_clear :: proc "contextless" (
	pool: ^Entasis_Buffer_Pool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	header := abi_buffer_pool_header_get(pool);
	if header == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if header.borrowed || header.attached != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.buffer_pool_clear(header.pool), diagnostic, .World_Initialize);
}

abi_buffer_pool_destroy :: proc "contextless" (
	pool: ^Entasis_Buffer_Pool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	header := abi_buffer_pool_header_get(pool);
	if header == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .World_Initialize);
	}
	if header.borrowed || header.attached != 0
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	resource := (^abi_buffer_pool_resource)(header);
	context = runtime.default_context();
	status := entasis.buffer_pool_destroy(header.pool);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	allocator := resource.allocator;
	header.magic = 0;
	pool.opaque = nil;
	abi_resource_free(
		resource, size_of(abi_buffer_pool_resource), align_of(abi_buffer_pool_resource), &allocator,
	);
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}
