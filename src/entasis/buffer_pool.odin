package entasis

import "core:mem"
import util "entasis:entasis_utilities"

// buffer_pool_init initializes a caller-owned pool for world_init_with_pool
buffer_pool_init :: proc(
	pool: ^Buffer_Pool,
	minimum_block_size: int = 131072,
	expected_resource_count: int = 16,
) -> Status
{
	return world_memory_status(util.buffer_pool_initialize(
			pool,
			minimum_block_size,
			expected_resource_count,
	));
}

// buffer_pool_clear releases native blocks while retaining ready pool metadata
buffer_pool_clear :: proc(pool: ^Buffer_Pool) -> Status
{
	return world_memory_status(util.buffer_pool_clear(pool));
}

// buffer_pool_destroy releases a caller-owned pool after all attached worlds are destroyed
buffer_pool_destroy :: proc(pool: ^Buffer_Pool) -> Status
{
	return world_memory_status(util.buffer_pool_dispose(pool));
}

// owns metadata and native blocks with the supplied allocator. clear preserves
// it. destruction and valid buffer returns never allocate in this mode
buffer_pool_init_with_allocator :: proc (
	pool: ^Buffer_Pool, allocator: mem.Allocator,
	minimum_block_size: int = 131072, expected_resource_count: int = 16,
) -> Status
{
	return world_memory_status(util.buffer_pool_initialize_with_allocator(
			pool, allocator, minimum_block_size, expected_resource_count,
	));
}
