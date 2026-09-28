package utilities_memory_tests

import "core:testing"
import entasis "entasis:entasis_utilities"

next_random_u32 :: proc(state: ^u32) -> u32
{
	state^ = state^ * 1664525 + 1013904223;
	return state^;
}

@(test)
aligned_allocation_boundaries :: proc(t: ^testing.T)
{
	_, invalid_alignment := entasis.aligned_allocate(128, 24);
	testing.expect_value(t, invalid_alignment, entasis.Memory_Status.Invalid_Alignment);
	_, invalid_count := entasis.aligned_allocate(0, 32);
	testing.expect_value(t, invalid_count, entasis.Memory_Status.Invalid_Count);

	allocation, allocation_status := entasis.aligned_allocate(257, 128);
	if !testing.expect_value(t, allocation_status, entasis.Memory_Status.Ok)
	{
		return;
	}
	testing.expect_value(t, allocation.byte_count, 257);
	testing.expect_value(t, allocation.alignment, 128);
	testing.expect_value(t, uintptr(allocation.memory) & 127, uintptr(0));
	testing.expect_value(t, entasis.aligned_release(&allocation), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.aligned_release(&allocation), entasis.Memory_Status.Invalid_Buffer);
}

@(test)
buffer_pool_bucket_ownership_and_churn :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	testing.expect_value(t, entasis.buffer_pool_initialize(&pool, 128), entasis.Memory_Status.Ok);
	defer entasis.buffer_pool_dispose(&pool);

	capacity, capacity_status := entasis.buffer_pool_capacity_for_count(u32, 3);
	testing.expect_value(t, capacity_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, capacity, 4);
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(&pool, 128, 4), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_total_allocated_byte_count(&pool), u64(128));
	caller_storage: [4]u32;
	caller_buffer := entasis.Buffer(u32){memory=&caller_storage[0], length=i32(len(caller_storage)), id=-1};
	caller_capacity, caller_capacity_status := entasis.buffer_capacity(caller_buffer);
	testing.expect_value(t, caller_capacity_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, caller_capacity, len(caller_storage));

	for iteration in 0 ..< 512
	{
		buffer, take_status := entasis.buffer_pool_take(&pool, u32, 3);
		if !testing.expect_value(t, take_status, entasis.Memory_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, buffer.length, 3);
		buffer_capacity, buffer_capacity_status := entasis.buffer_capacity(buffer);
		testing.expect_value(t, buffer_capacity_status, entasis.Memory_Status.Ok);
		testing.expect_value(t, buffer_capacity, 4);
		testing.expect_value(t, uintptr(buffer.memory) & 15, uintptr(0));
		buffer.memory[0] = u32(iteration);
		testing.expect_value(t, entasis.buffer_pool_return(&pool, &buffer), entasis.Memory_Status.Ok);
	}

	testing.expect_value(t, entasis.buffer_pool_clear(&pool), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_total_allocated_byte_count(&pool), u64(0));
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(&pool, 128, 4), entasis.Memory_Status.Ok);
	reused, reused_status := entasis.buffer_pool_take(&pool, u32, 3);
	testing.expect_value(t, reused_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_return(&pool, &reused), entasis.Memory_Status.Ok);
}

when ODIN_DEBUG
{
	@(test)
	buffer_pool_debug_rejects_invalid_returns_without_corrupting_ownership :: proc(t: ^testing.T)
	{
		pool_a: entasis.Buffer_Pool;
		pool_b: entasis.Buffer_Pool;
		testing.expect_value(t, entasis.buffer_pool_initialize(&pool_a, 128), entasis.Memory_Status.Ok);
		defer entasis.buffer_pool_dispose(&pool_a);
		testing.expect_value(t, entasis.buffer_pool_initialize(&pool_b, 128), entasis.Memory_Status.Ok);
		defer entasis.buffer_pool_dispose(&pool_b);

		duplicate, duplicate_status := entasis.buffer_pool_take(&pool_a, u32, 4);
		if !testing.expect_value(t, duplicate_status, entasis.Memory_Status.Ok)
		{
			return;
		}
		stale_duplicate := duplicate;
		stale_duplicate_memory := stale_duplicate.memory;
		testing.expect_value(t, entasis.buffer_pool_return(&pool_a, &duplicate), entasis.Memory_Status.Ok);
		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_a, &stale_duplicate),
			entasis.Memory_Status.Invalid_Buffer,
		);
		testing.expect_value(t, stale_duplicate.memory, stale_duplicate_memory);

		pool_a_buffer, pool_a_status := entasis.buffer_pool_take(&pool_a, u32, 4);
		pool_b_buffer, pool_b_status := entasis.buffer_pool_take(&pool_b, u32, 4);
		if !testing.expect_value(t, pool_a_status, entasis.Memory_Status.Ok) ||
			!testing.expect_value(t, pool_b_status, entasis.Memory_Status.Ok)
		{
			return;
		}
		testing.expect_value(t, pool_a_buffer.id, pool_b_buffer.id);
		wrong_pool := pool_a_buffer;
		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_b, &wrong_pool),
			entasis.Memory_Status.Invalid_Buffer,
		);
		testing.expect_value(t, wrong_pool.memory, pool_a_buffer.memory);

		wrong_address := pool_a_buffer;
		wrong_address.memory = &wrong_address.memory[1];
		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_a, &wrong_address),
			entasis.Memory_Status.Invalid_Buffer,
		);
		testing.expect_value(t, wrong_address.memory, &pool_a_buffer.memory[1]);

		over_extent := pool_a_buffer;
		over_extent.length += 1;
		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_a, &over_extent),
			entasis.Memory_Status.Invalid_Buffer,
		);
		testing.expect_value(t, over_extent.length, pool_a_buffer.length + 1);

		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_a, &pool_a_buffer),
			entasis.Memory_Status.Ok,
		);
		testing.expect_value(
			t, entasis.buffer_pool_return(&pool_b, &pool_b_buffer),
			entasis.Memory_Status.Ok,
		);
	}
}

@(test)
buffer_growth_is_automatic_and_resize_preserves_requested_elements :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	testing.expect_value(t, entasis.buffer_pool_initialize(&pool, 128), entasis.Memory_Status.Ok);
	defer entasis.buffer_pool_dispose(&pool);
	buffer, take_status := entasis.buffer_pool_take(&pool, u32, 3);
	if !testing.expect_value(t, take_status, entasis.Memory_Status.Ok)
	{
		return;
	}
	buffer.memory[0] = 11;
	buffer.memory[1] = 22;
	buffer.memory[2] = 33;
	testing.expect_value(t, entasis.buffer_pool_resize(&pool, &buffer, 8, 3), entasis.Memory_Status.Ok);
	testing.expect_value(t, buffer.length, 8);
	capacity, capacity_status := entasis.buffer_capacity(buffer);
	testing.expect_value(t, capacity_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, capacity, 8);
	testing.expect_value(t, buffer.memory[0], u32(11));
	testing.expect_value(t, buffer.memory[1], u32(22));
	testing.expect_value(t, buffer.memory[2], u32(33));
	testing.expect_value(t, entasis.buffer_pool_return(&pool, &buffer), entasis.Memory_Status.Ok);
}

@(test)
buffer_views_do_not_own_storage :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	testing.expect_value(t, entasis.buffer_pool_initialize(&pool, 128), entasis.Memory_Status.Ok);
	defer entasis.buffer_pool_dispose(&pool);
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(&pool, 128, 5), entasis.Memory_Status.Ok);
	buffer, take_status := entasis.buffer_pool_take(&pool, u32, 8);
	if !testing.expect_value(t, take_status, entasis.Memory_Status.Ok)
	{
		return;
	}
	for index in 0 ..< buffer.length
	{
		buffer.memory[index] = u32(index + 10);
	}
	view, slice_status := entasis.buffer_slice(buffer, 2, 3);
	testing.expect_value(t, slice_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, view.memory[0], u32(12));
	testing.expect_value(t, view.memory[2], u32(14));
	testing.expect_value(t, entasis.buffer_pool_return(&pool, &buffer), entasis.Memory_Status.Ok);
}

@(test)
id_pool_reuses_returned_ids_lifo :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	testing.expect_value(t, entasis.buffer_pool_initialize(&pool, 128), entasis.Memory_Status.Ok);
	defer entasis.buffer_pool_dispose(&pool);
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(&pool, 128, 4), entasis.Memory_Status.Ok);
	id_pool: entasis.Id_Pool;
	testing.expect_value(t, entasis.id_pool_initialize(&id_pool, 4, &pool), entasis.Memory_Status.Ok);
	defer entasis.id_pool_dispose(&id_pool, &pool);

	first, first_status := entasis.id_pool_take(&id_pool);
	second, second_status := entasis.id_pool_take(&id_pool);
	testing.expect_value(t, first_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, second_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, first, i32(0));
	testing.expect_value(t, second, i32(1));
	testing.expect_value(t, entasis.id_pool_return(&id_pool, first, &pool), entasis.Memory_Status.Ok);
	reused, reused_status := entasis.id_pool_take(&id_pool);
	testing.expect_value(t, reused_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, reused, first);
}

@(test)
id_pool_capacity_maintenance_and_randomized_valid_churn :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	testing.expect_value(t, entasis.buffer_pool_initialize(&pool, 128), entasis.Memory_Status.Ok);
	defer entasis.buffer_pool_dispose(&pool);
	for power in 4 ..= 7
	{
		testing.expect_value(
			t,
			entasis.buffer_pool_ensure_capacity_for_power(&pool, 128, power),
			entasis.Memory_Status.Ok
		);
	}
	id_pool: entasis.Id_Pool;
	testing.expect_value(t, entasis.id_pool_initialize(&id_pool, 4, &pool), entasis.Memory_Status.Ok);
	defer entasis.id_pool_dispose(&id_pool, &pool);

	claimed: [5]i32;
	for index in 0 ..< len(claimed)
	{
		id, status := entasis.id_pool_take(&id_pool);
		testing.expect_value(t, status, entasis.Memory_Status.Ok);
		testing.expect_value(t, id, i32(index));
		claimed[index] = id;
	}
	for index in 0 ..< 4
	{
		testing.expect_value(t, entasis.id_pool_return(&id_pool, claimed[index], &pool), entasis.Memory_Status.Ok);
	}
	testing.expect_value(t, entasis.id_pool_return(&id_pool, claimed[4], &pool), entasis.Memory_Status.Ok);
	for expected in 0 ..< len(claimed)
	{
		id, status := entasis.id_pool_take(&id_pool);
		testing.expect_value(t, status, entasis.Memory_Status.Ok);
		testing.expect_value(t, id, i32(len(claimed) - 1 - expected));
	}

	entasis.id_pool_clear(&id_pool);
	testing.expect_value(t, entasis.id_pool_resize(&id_pool, 32, &pool), entasis.Memory_Status.Ok);
	active: [32]i32;
	model_available: [32]i32;
	active_count := 0;
	model_available_count := 0;
	next_id := i32(0);
	random_state := u32(0x7b51_29d3);
	for _ in 0 ..< 1024
	{
		random_value := next_random_u32(&random_state);
		if active_count == 0 || (active_count < len(active) && random_value & 1 == 0)
		{
			actual, status := entasis.id_pool_take(&id_pool);
			testing.expect_value(t, status, entasis.Memory_Status.Ok);
			expected := next_id;
			if model_available_count > 0
			{
				model_available_count -= 1;
				expected = model_available[model_available_count];
			}
			else
			{
				next_id += 1;
			}
			testing.expect_value(t, actual, expected);
			active[active_count] = actual;
			active_count += 1;
		}
		else
		{
			active_index := int(random_value % u32(active_count));
			returned := active[active_index];
			testing.expect_value(t, entasis.id_pool_return(&id_pool, returned, &pool), entasis.Memory_Status.Ok);
			model_available[model_available_count] = returned;
			model_available_count += 1;
			active_count -= 1;
			active[active_index] = active[active_count];
		}
	}
	for active_index in 0 ..< active_count
	{
		testing.expect_value(
			t,
			entasis.id_pool_return(&id_pool, active[active_index], &pool),
			entasis.Memory_Status.Ok
		);
	}
	entasis.id_pool_clear(&id_pool);
	testing.expect_value(t, entasis.id_pool_compact(&id_pool, 4, &pool), entasis.Memory_Status.Ok);
	available_capacity, available_capacity_status := entasis.buffer_capacity(id_pool.available_ids);
	testing.expect_value(t, available_capacity_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, available_capacity, 4);
	first_after_clear, first_after_clear_status := entasis.id_pool_take(&id_pool);
	testing.expect_value(t, first_after_clear_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, first_after_clear, i32(0));
}

@(test)
worker_pools_keep_ownership_separate :: proc(t: ^testing.T)
{
	workers: entasis.Worker_Buffer_Pools;
	testing.expect_value(t, entasis.worker_buffer_pools_initialize(&workers, 2, 128), entasis.Memory_Status.Ok);
	defer entasis.worker_buffer_pools_dispose(&workers);
	worker0, status0 := entasis.worker_buffer_pool_get(&workers, 0);
	worker1, status1 := entasis.worker_buffer_pool_get(&workers, 1);
	testing.expect_value(t, status0, entasis.Memory_Status.Ok);
	testing.expect_value(t, status1, entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(worker0, 128, 4), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_ensure_capacity_for_power(worker1, 128, 4), entasis.Memory_Status.Ok);
	buffer0, take_status0 := entasis.buffer_pool_take(worker0, u32, 4);
	buffer1, take_status1 := entasis.buffer_pool_take(worker1, u32, 4);
	testing.expect_value(t, take_status0, entasis.Memory_Status.Ok);
	testing.expect_value(t, take_status1, entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_return(worker0, &buffer0), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.buffer_pool_return(worker1, &buffer1), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.worker_buffer_pools_total_allocated_byte_count(&workers), u64(256));
}
