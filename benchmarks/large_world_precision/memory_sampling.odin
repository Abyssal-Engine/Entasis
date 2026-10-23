package large_world_precision

import "core:fmt"
import "core:mem"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

Memory_Frame :: struct
{
	allocations, live_bytes: i64,
	pool_bytes, worker_pool_bytes: u64,
	active_pairs: i32,
	maintenance_allocations: i64,
}

Precision_Memory :: struct
{
	tracker: mem.Tracking_Allocator,
	allocator: mem.Allocator,
	frames: [MEASURED_STEPS]Memory_Frame,
	initial: Memory_Frame,
	setup_peak_bytes: i64,
	application_reserved_bytes: int,
	count: int,
}

precision_memory_initialize :: proc(memory: ^Precision_Memory)
{
	mem.tracking_allocator_init(&memory.tracker, context.allocator, context.allocator);
	memory.allocator = mem.tracking_allocator(&memory.tracker);
}

precision_pool_initialize :: proc(pool: ^entasis.Buffer_Pool, memory: ^Precision_Memory) -> entasis.Status
{
	if memory != nil
	{
		return entasis.buffer_pool_init_with_allocator(pool, memory.allocator, POOL_MINIMUM_BLOCK_SIZE);
	}
	return entasis.buffer_pool_init(pool, POOL_MINIMUM_BLOCK_SIZE);
}

precision_workers_initialize :: proc(pool: ^entasis.Thread_Pool, workers: int, memory: ^Precision_Memory) -> entasis.Dispatcher_Status
{
	if memory != nil
	{
		return entasis.thread_pool_init_with_allocator(pool, workers, memory.allocator, POOL_MINIMUM_BLOCK_SIZE);
	}
	return entasis.thread_pool_init(pool, workers, POOL_MINIMUM_BLOCK_SIZE);
}

precision_world_initialize :: proc(
	world: ^entasis.World, description: entasis.World_Description,
	pool: ^entasis.Buffer_Pool, memory: ^Precision_Memory,
) -> entasis.Status
{
	if memory != nil
	{
		tracked_description := description;
		tracked_description.allocator = memory.allocator;
		return entasis.world_init_with_pool_and_allocation_scope(world, tracked_description, pool, .All_Owned);
	}
	return entasis.world_init_with_pool(world, description, pool);
}

// all readers run after world_step has joined its workers. pool bytes measure
// native blocks, not rss. allocation counters include metadata and worker owners
precision_memory_world :: proc(world: ^entasis.World, frame: ^Memory_Frame) -> entasis.Status
{
	pool, status := entasis.world_borrow_pool(world);
	if status != .Ok
	{
		return status;
	}
	frame.pool_bytes = util.buffer_pool_total_allocated_byte_count(pool);
	dispatcher, dispatcher_status := entasis.world_borrow_dispatcher(world);
	if dispatcher_status != .Ok
	{
		return dispatcher_status;
	}
	if dispatcher != nil
	{
		for index in 0 ..< dispatcher.worker_count
		{
			worker, worker_status := dispatcher.worker_pool(dispatcher, index);
			if worker_status != .Ok
			{
				return .Invalid_Argument;
			}
			frame.worker_pool_bytes += util.buffer_pool_total_allocated_byte_count(worker);
		}
	}
	stats, stats_status := entasis.world_stats(world);
	frame.active_pairs = i32(stats.active_pairs);
	return stats_status;
}

precision_memory_begin :: proc(memory: ^Precision_Memory)
{
	memory.initial.allocations = memory.tracker.total_allocation_count;
	memory.initial.live_bytes = memory.tracker.current_memory_allocated;
	memory.setup_peak_bytes = memory.tracker.peak_memory_allocated;
	memory.tracker.peak_memory_allocated = memory.tracker.current_memory_allocated;
}

precision_memory_record :: proc(memory: ^Precision_Memory, frame: Memory_Frame)
{
	value := frame;
	value.allocations = memory.tracker.total_allocation_count;
	value.live_bytes = memory.tracker.current_memory_allocated;
	memory.frames[memory.count] = value;
	memory.count += 1;
}

precision_memory_emit :: proc(memory: ^Precision_Memory, application_bytes: int)
{
	memory.application_reserved_bytes = application_bytes;
	fmt.printfln("precision_memory_summary setup_peak_bytes=%d measurement_peak_bytes=%d initial_live_bytes=%d initial_allocations=%d initial_pool_bytes=%d initial_worker_pool_bytes=%d application_reserved_bytes=%d samples=%d",
		memory.setup_peak_bytes, memory.tracker.peak_memory_allocated,
		memory.initial.live_bytes, memory.initial.allocations,
		memory.initial.pool_bytes, memory.initial.worker_pool_bytes, application_bytes, memory.count);
	previous := memory.initial;
	for frame, index in memory.frames[:memory.count]
	{
		fmt.printfln("precision_memory_frame frame=%d allocations=%d live_bytes=%d live_growth_bytes=%d pool_bytes=%d worker_pool_bytes=%d active_pairs=%d maintenance_allocations=%d",
			index+1, frame.allocations-previous.allocations, frame.live_bytes,
			frame.live_bytes-previous.live_bytes, frame.pool_bytes, frame.worker_pool_bytes,
			frame.active_pairs, frame.maintenance_allocations);
		previous = frame;
	}
}

precision_memory_finish :: proc(memory: ^Precision_Memory)
{
	fmt.printfln("precision_memory_release remaining_live_bytes=%d allocations=%d frees=%d",
		memory.tracker.current_memory_allocated, memory.tracker.total_allocation_count,
		memory.tracker.total_free_count);
	mem.tracking_allocator_destroy(&memory.tracker);
}


MEMORY_HEADER :: ",setup_peak_bytes,measurement_peak_bytes,initial_owned_bytes,final_owned_bytes,total_allocations,measured_allocations,initial_pool_bytes,final_pool_bytes,initial_worker_pool_bytes,final_worker_pool_bytes,application_reserved_bytes,shift_allocations,remaining_owned_bytes";

precision_memory_columns :: proc(memory: ^Precision_Memory, precision_case: Precision_Case) -> (string, Precision_Admission)
{
    expected: int = QUERY_PASS_COUNT if precision_case == .Queries_Local || precision_case == .Queries_Spanning else MEASURED_STEPS;
    if memory.count != expected || memory.tracker.current_memory_allocated != 0
    {
        return "", .Rejected;
    }
    last: Memory_Frame = memory.frames[memory.count-1];
    shift_allocations: i64;
    for frame in memory.frames[:memory.count]
    {
        shift_allocations += frame.maintenance_allocations;
    }
    measured_allocations: i64 = last.allocations-memory.initial.allocations;
    if shift_allocations != 0 || ((precision_case == .Queries_Local || precision_case == .Queries_Spanning) && measured_allocations != 0)
    {
        return "", .Rejected;
    }
    return fmt.tprintf(",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        memory.setup_peak_bytes, memory.tracker.peak_memory_allocated,
        memory.initial.live_bytes, last.live_bytes, memory.tracker.total_allocation_count,
        measured_allocations, memory.initial.pool_bytes, last.pool_bytes,
        memory.initial.worker_pool_bytes, last.worker_pool_bytes,
        memory.application_reserved_bytes, shift_allocations, memory.tracker.current_memory_allocated), .Accepted;
}
