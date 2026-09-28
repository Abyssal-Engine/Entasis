package public_api_tests

import "base:runtime"
import "core:mem"
import "core:sync"
import "core:testing"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import shared "entasis:entasis_c_shared"
import util "entasis:entasis_utilities"

allocation_test_record :: struct
{
	memory: rawptr, size, alignment: int,
}

Allocation_Test_Fill :: enum u8
{
	Preserve,
	Poison,
}

Allocation_Test_Pool_Operation :: enum u8
{
	Acquire,
	Release,
}

allocation_test_tracker :: struct
{
	lock: sync.Mutex,
	records: [4096]allocation_test_record,
	requests, fail_from, live, live_bytes, metadata_errors: int,
	fill: Allocation_Test_Fill,
}

allocation_test_proc :: proc (
	data: rawptr, mode: runtime.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int,
	location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, runtime.Allocator_Error)
{
	state: ^allocation_test_tracker = (^allocation_test_tracker)(data);
	sync.mutex_lock(&state.lock);
	defer sync.mutex_unlock(&state.lock);
	if mode == .Query_Features
	{
		if old_memory != nil
		{
			(cast(^runtime.Allocator_Mode_Set)old_memory)^ = {.Alloc, .Alloc_Non_Zeroed, .Free, .Resize, .Resize_Non_Zeroed, .Query_Features};
		}
		return nil, nil;
	}
	index: int = -1;
	if old_memory != nil
	{
		for record, i in state.records
		{
			if record.memory == old_memory
			{
				index = i;
				break;
			}
		}
		if index < 0
		{
			state.metadata_errors += 1;
			return nil, .Invalid_Argument;
		}
		record: allocation_test_record = state.records[index];
		if record.size != old_size || record.alignment != alignment
		{
			state.metadata_errors += 1;
		}
	}
	if mode == .Free
	{
		if old_memory == nil
		{
			return nil, nil;
		}
		record: allocation_test_record = state.records[index];
		allocator: mem.Allocator = runtime.heap_allocator();
		error: runtime.Allocator_Error;
		_, error = allocator.procedure(allocator.data, .Free, 0, record.alignment, old_memory, record.size, location);
		state.records[index] = {};
		state.live -= 1;
		state.live_bytes -= record.size;
		return nil, error;
	}
	if mode != .Alloc && mode != .Alloc_Non_Zeroed && mode != .Resize && mode != .Resize_Non_Zeroed
	{
		return nil, .Mode_Not_Implemented;
	}
	state.requests += 1;
	if state.fail_from > 0 && state.requests >= state.fail_from
	{
		return nil, .Out_Of_Memory;
	}
	if index < 0
	{
		for record, i in state.records
		{
			if record.memory == nil
			{
				index = i;
				break;
			}
		}
	}
	if index < 0
	{
		return nil, .Out_Of_Memory;
	}
	allocator: mem.Allocator = runtime.heap_allocator();
	memory: []byte;
	error: runtime.Allocator_Error;
	memory, error = allocator.procedure(allocator.data, mode, size, alignment, old_memory, old_size, location);
	if error != nil || len(memory) == 0
	{
		return memory, error;
	}
	if old_memory == nil
	{
		state.live += 1;
	}
	state.live_bytes += size - state.records[index].size;
	state.records[index] = {raw_data(memory), size, alignment};
	if state.fill == .Poison
	{
		for &value in memory[min(old_size, len(memory)):]
		{
			value = 0xcd;
		}
	}
	return memory, nil;
}

allocation_test_allocator :: proc "contextless" (tracker: ^allocation_test_tracker) -> mem.Allocator
{
	return {procedure=allocation_test_proc, data=tracker};
}

allocation_test_empty :: proc (t: ^testing.T, state: ^allocation_test_tracker)
{
	testing.expect_value(t, state.metadata_errors, 0);
	testing.expect_value(t, state.live, 0);
	testing.expect_value(t, state.live_bytes, 0);
}

@(test)
all_owned_pool_commit_and_allocation_free_returns :: proc (t: ^testing.T)
{
	state: allocation_test_tracker;
	pool: util.Buffer_Pool;
	testing.expect(t, util.buffer_pool_initialize_with_allocator(&pool, allocation_test_allocator(&state)) == .Ok);
	value: util.Buffer(u8);
	status: util.Memory_Status;
	value, status = util.buffer_pool_take(&pool, u8, 1);
	if !testing.expect(t, status == .Ok)
	{
		return;
	}
	value.memory[0] = 73;
	testing.expect(t, pool.pools[0].slot_capacity >= 131072);
	when ODIN_DEBUG
	{
		testing.expect(t, pool.pools[0].debug.word_capacity >= 2048);
	}
	blocks: int = pool.pools[0].block_count;
	slots: int = pool.pools[0].next_slot;
	state.fail_from = state.requests + 1;
	// force another committed block. no existing block or live buffer may change
	testing.expect(t, util.buffer_pool_ensure_capacity_for_power(&pool, 262144, 0) == .Out_Of_Memory);
	testing.expect_value(t, pool.pools[0].block_count, blocks);
	testing.expect_value(t, pool.pools[0].next_slot, slots);
	testing.expect_value(t, value.memory[0], u8(73));
	requests: int = state.requests;
	testing.expect(t, util.buffer_pool_return(&pool, &value) == .Ok);
	testing.expect(t, util.buffer_pool_clear(&pool) == .Ok);
	testing.expect(t, pool.allocation_scope == .All_Owned);
	testing.expect_value(t, state.requests, requests);
	testing.expect(t, util.buffer_pool_dispose(&pool) == .Ok);
	testing.expect_value(t, state.requests, requests);
	allocation_test_empty(t, &state);
	// every metadata-allocation failure leaves an uninitialized reusable owner
	for fail in 1 ..= 63
	{
		state = {fail_from=fail};
		pool = {};
		status = util.buffer_pool_initialize_with_allocator(&pool, allocation_test_allocator(&state), 128, 1);
		if status == .Ok
		{
			_ = util.buffer_pool_dispose(&pool);
		}
		else
		{
			testing.expect(t, status == .Out_Of_Memory);
			testing.expect(t, pool.state == .Uninitialized);
		}
		allocation_test_empty(t, &state);
	}
}

@(test)
all_owned_world_and_thread_partial_initialization :: proc (t: ^testing.T)
{
	state: allocation_test_tracker;
	description: entasis.World_Description = small_world_description();
	description.allocator = allocation_test_allocator(&state);
	description.threading.worker_count = 2;
	description.threading.worker_pool_block_size = 128;
	world: entasis.World;
	if !testing.expect(t, entasis.world_init_with_allocation_scope(&world, description, .All_Owned) == .Ok)
	{
		return;
	}
	requests: int = state.requests;
	state.fail_from = requests + 1;
	testing.expect(t, entasis.world_destroy(&world) == .Ok);
	testing.expect_value(t, state.requests, requests);
	allocation_test_empty(t, &state);
	for fail in 1 ..= requests
	{
		state = {fail_from=fail};
		world = nil;
		status: entasis.Status = entasis.world_init_with_allocation_scope(&world, description, .All_Owned);
		if status == .Ok
		{
			_ = entasis.world_destroy(&world);
		}
		else
		{
			testing.expect(t, status == .Capacity_Missing);
			testing.expect(t, world == nil);
		}
		allocation_test_empty(t, &state);
	}
	// the ten-worker path uses the existing padded combined backing allocation
	state = {};
	pool: entasis.Thread_Pool;
	if testing.expect(t, entasis.thread_pool_init_with_allocator(&pool, 10, allocation_test_allocator(&state), 128) == .Ok)
	{
		requests = state.requests;
		state.fail_from = requests + 1;
		testing.expect(t, entasis.thread_pool_destroy(&pool) == .Ok);
		testing.expect_value(t, state.requests, requests);
	}
	allocation_test_empty(t, &state);
}

@(test)
all_owned_query_cooking_and_borrowed_owners :: proc (t: ^testing.T)
{
	world_state, cooking_state: allocation_test_tracker;
	description: entasis.World_Description = small_world_description();
	description.allocator = allocation_test_allocator(&world_state);
	world: entasis.World;
	if !testing.expect(t, entasis.world_init_with_allocation_scope(&world, description, .All_Owned) == .Ok)
	{
		return;
	}
	ctx: cooking.Cooking_Context;
	if !testing.expect(t, cooking.cooking_context_init_with_allocator(&ctx, allocation_test_allocator(&cooking_state), 128) == .Ok)
	{
		return;
	}
	triangles: [1]entasis.Triangle = [1]entasis.Triangle{{a={-1, 0, -1}, b={1, 0, -1}, c={0, 0, 1}}};
	asset: cooking.Cooked_Mesh;
	status: entasis.Status;
	asset, status = cooking.cook_mesh(&ctx, triangles[:]);
	testing.expect(t, status == .Ok);
	shape: entasis.Shape_Handle;
	import_status: entasis.Status;
	shape, import_status = cooking.cooked_mesh_import(&world, &asset);
	testing.expect(t, import_status == .Ok);
	query: entasis.Query_Context;
	testing.expect(t, entasis.query_context_init(&query, &world) == .Ok);
	world_requests: int = world_state.requests;
	cooking_requests: int = cooking_state.requests;
	world_state.fail_from = world_requests + 1;
	cooking_state.fail_from = cooking_requests + 1;
	testing.expect(t, cooking.cooked_mesh_destroy(&asset) == .Ok);
	testing.expect(t, cooking.cooking_context_clear(&ctx) == .Ok);
	testing.expect(t, cooking.cooking_context_destroy(&ctx) == .Ok);
	testing.expect(t, entasis.query_context_destroy(&query) == .Ok);
	testing.expect(t, entasis.shape_remove(&world, shape) == .Ok);
	testing.expect(t, entasis.world_destroy(&world) == .Ok);
	testing.expect_value(t, world_state.requests, world_requests);
	testing.expect_value(t, cooking_state.requests, cooking_requests);
	allocation_test_empty(t, &world_state);
	allocation_test_empty(t, &cooking_state);
	// borrowed pools retain their allocator and scope, including on world failure
	world_state = {};
	cooking_state = {};
	pool: entasis.Buffer_Pool;
	testing.expect(t, entasis.buffer_pool_init_with_allocator(&pool, allocation_test_allocator(&cooking_state), 128) == .Ok);
	for fail in 1 ..= 20
	{
		world_state.fail_from = world_state.requests + fail;
		status = entasis.world_init_with_pool_and_allocation_scope(&world, description, &pool, .All_Owned);
		if status == .Ok
		{
			testing.expect(t, entasis.world_destroy(&world) == .Ok);
		}
		testing.expect(t, pool.state == .Ready);
		allocation_test_empty(t, &world_state);
		for bucket in pool.pools
		{
			testing.expect_value(t, bucket.next_slot - bucket.free_count, 0);
		}
	}
	cooking_state.fail_from = cooking_state.requests + 1;
	testing.expect(t, entasis.buffer_pool_destroy(&pool) == .Ok);
	allocation_test_empty(t, &cooking_state);
}

allocation_test_c_allocate :: proc "c" (data: rawptr, size, alignment: u64) -> rawptr
{
	context = runtime.default_context();
	bytes: []byte;
	error: runtime.Allocator_Error;
	bytes, error = allocation_test_proc(data, .Alloc_Non_Zeroed, int(size), int(alignment), nil, 0);
	if error != nil
	{
		return nil;
	}
	return raw_data(bytes);
}

allocation_test_c_free :: proc "c" (data, memory: rawptr, size, alignment: u64)
{
	context = runtime.default_context();
	_, _ = allocation_test_proc(data, .Free, 0, int(alignment), memory, int(size));
}

allocation_test_c_resize :: proc "c" (data, memory: rawptr, old_size, new_size, alignment: u64) -> rawptr
{
	context = runtime.default_context();
	bytes: []byte;
	error: runtime.Allocator_Error;
	bytes, error = allocation_test_proc(data, .Resize_Non_Zeroed, int(new_size), int(alignment), memory, int(old_size));
	if error != nil
	{
		return nil;
	}
	return raw_data(bytes);
}

@(test)
all_owned_shared_allocator_resize_and_zeroing :: proc (t: ^testing.T)
{
	for optional in 0 ..< 2
	{
		state: allocation_test_tracker = allocation_test_tracker{fill=.Poison};
		callback: shared.Allocator = shared.Allocator{scope=.All_Owned, user_context=&state,
			allocate=allocation_test_c_allocate, deallocate=allocation_test_c_free};
		if optional != 0
		{
			callback.reallocate = allocation_test_c_resize;
		}
		allocator: mem.Allocator = shared.Allocator_To_Core(&callback);
		memory: []byte;
		error: runtime.Allocator_Error;
		memory, error = allocator.procedure(allocator.data, .Alloc, 32, 32, nil, 0, #location());
		if !testing.expect(t, error == nil && len(memory) == 32)
		{
			return;
		}
		for value in memory
		{
			testing.expect_value(t, value, byte(0));
		}
		for &value in memory
		{
			value = 91;
		}
		state.fail_from = state.requests + 1;
		_, error = allocator.procedure(allocator.data, .Resize, 64, 32, raw_data(memory), 32, #location());
		testing.expect(t, error == .Out_Of_Memory);
		for value in memory
		{
			testing.expect_value(t, value, byte(91));
		}
		state.fail_from = 0;
		grown: []byte;
		growth_error: runtime.Allocator_Error;
		grown, growth_error = allocator.procedure(allocator.data, .Resize, 64, 32, raw_data(memory), 32, #location());
		if !testing.expect(t, growth_error == nil && len(grown) == 64)
		{
			return;
		}
		for value in grown[:32]
		{
			testing.expect_value(t, value, byte(91));
		}
		for value in grown[32:]
		{
			testing.expect_value(t, value, byte(0));
		}
		requests: int = state.requests;
		_, error = allocator.procedure(allocator.data, .Resize, 0, 32, raw_data(grown), 64, #location());
		testing.expect(t, error == nil);
		testing.expect_value(t, state.requests, requests);
		allocation_test_empty(t, &state);
	}
}

allocation_worker_test_state :: struct
{
	buffers: [3][8]util.Buffer(u8),
	errors: [3]int,
	operation: Allocation_Test_Pool_Operation,
}

allocation_worker_test :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^allocation_worker_test_state = (^allocation_worker_test_state)(boundary.unmanaged_context);
	pool: ^util.Buffer_Pool;
	status: util.Threading_Status;
	pool, status = boundary.worker_pool(boundary, worker);
	if status != .Ok
	{
		state.errors[worker] += 1;
		return;
	}
	if state.operation == .Acquire
	{
		for &buffer in state.buffers[worker]
		{
			allocated: util.Buffer(u8);
			allocation_status: util.Memory_Status;
			allocated, allocation_status = util.buffer_pool_take_at_least(pool, u8, 32768);
			buffer = allocated;
			if allocation_status != .Ok
			{
				state.errors[worker] += 1;
			}
		}
	}
	else
	{
		failed: util.Memory_Status;
		_, failed = util.buffer_pool_take_at_least(pool, u8, 1<<20);
		if failed == .Ok
		{
			state.errors[worker] += 1;
		}
		for &buffer in state.buffers[worker]
		{
			if util.buffer_pool_return(pool, &buffer) != .Ok
			{
				state.errors[worker] += 1;
			}
		}
	}
}

@(test)
all_owned_worker_growth_and_returns_under_sustained_refusal :: proc(t: ^testing.T)
{
	tracker: allocation_test_tracker;
	dispatcher: util.Thread_Dispatcher;
	if !testing.expect(t, util.thread_dispatcher_initialize_with_allocator(&dispatcher, 3,
			allocation_test_allocator(&tracker), 128) == .Ok)
	{
		return;
	}
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	state: allocation_worker_test_state;
	testing.expect(t, boundary.dispatch(boundary, allocation_worker_test, 3, &state) == .Ok);
	tracker.fail_from = tracker.requests + 1;
	state.operation = .Release;
	testing.expect(t, boundary.dispatch(boundary, allocation_worker_test, 3, &state) == .Ok);
	for error in state.errors
	{
		testing.expect_value(t, error, 0);
	}
	testing.expect(t, util.thread_dispatcher_shutdown(&dispatcher) == .Ok);
	allocation_test_empty(t, &tracker);
}
