package tasking_multi_tests

import "core:sync"
import "core:testing"
import entasis "entasis:entasis_utilities"

TASK_COUNT :: 128;
WORKER_COUNT :: 4;

Multi_Context :: struct
{
	stack:            ^entasis.Task_Stack,
	counts:           [TASK_COUNT]i32,
	values:           [TASK_COUNT]u64,
	worker_hits:      [WORKER_COUNT]i32,
	worker_entered:   [WORKER_COUNT]i32,
	entered_count:    i32,
	completion_count: i32,
}

multi_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^entasis.Thread_Dispatcher_Boundary,
)
{
	_ = dispatcher;
	state := (^Multi_Context)(task_context);
	index := int(task_id);
	previous_entry := sync.atomic_exchange_explicit(
		&state.worker_entered[worker_index], i32(1), .Seq_Cst,
	);
	if previous_entry == 0
	{
		_ = sync.atomic_add_explicit(&state.entered_count, i32(1), .Seq_Cst);
	}
	for sync.atomic_load_explicit(&state.entered_count, .Seq_Cst) < WORKER_COUNT
	{
		sync.cpu_relax();
	}
	_ = sync.atomic_add_explicit(&state.counts[index], i32(1), .Seq_Cst);
	_ = sync.atomic_add_explicit(&state.worker_hits[worker_index], i32(1), .Seq_Cst);
	value := u64(task_id + 1) * 0x9e37_79b9;
	value ~= u64(range_start) << 17;
	value ~= u64(range_end) << 33;
	state.values[index] = value;
}

multi_completion_and_stop :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^entasis.Thread_Dispatcher_Boundary,
)
{
	_ = task_id;
	_ = range_start;
	_ = range_end;
	_ = worker_index;
	_ = dispatcher;
	state := (^Multi_Context)(task_context);
	_ = sync.atomic_add_explicit(&state.completion_count, i32(1), .Seq_Cst);
	entasis.task_stack_request_stop(state.stack);
}

prepare_task_pool :: proc(t: ^testing.T, pool: ^entasis.Buffer_Pool)
{
	testing.expect_value(t, entasis.buffer_pool_initialize(pool, 128), entasis.Memory_Status.Ok);
	for power in 0 ..= 16
	{
		testing.expect_value(
			t, entasis.buffer_pool_ensure_capacity_for_power(pool, 65536, power),
			entasis.Memory_Status.Ok,
		);
	}
}

result_hash :: proc "contextless" (state: ^Multi_Context) -> u64
{
	hash := u64(1469598103934665603);
	for index in 0 ..< TASK_COUNT
	{
		hash ~= state.values[index];
		hash *= 1099511628211;
	}
	return hash;
}

worker_pool_outstanding_slot_count :: proc "contextless" (
	dispatcher: ^entasis.Thread_Dispatcher,
) -> int
{
	count := 0;
	for worker_index in 0 ..< dispatcher.worker_pools.worker_count
	{
		pool := &dispatcher.worker_pools.pools[worker_index];
		for power in 0 ..< entasis.BUFFER_POOL_POWER_COUNT
		{
			power_pool := &pool.pools[power];
			count += power_pool.next_slot - power_pool.free_count;
		}
	}
	return count;
}

worker_pool_block_count :: proc "contextless" (
	dispatcher: ^entasis.Thread_Dispatcher,
) -> int
{
	count := 0;
	for worker_index in 0 ..< dispatcher.worker_pools.worker_count
	{
		pool := &dispatcher.worker_pools.pools[worker_index];
		for power in 0 ..< entasis.BUFFER_POOL_POWER_COUNT
		{
			count += pool.pools[power].block_count;
		}
	}
	return count;
}

@(test)
persistent_workers_execute_each_task_once_publish_completion_and_shutdown_cleanly :: proc(
	t: ^testing.T,
)
{
	pool: entasis.Buffer_Pool;
	prepare_task_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);
	procedures := [2]entasis.Task_Procedure{multi_task, multi_completion_and_stop};
	procedure_buffer := entasis.Buffer(entasis.Task_Procedure){
		memory=&procedures[0],
		length=2,
		id=-1,
	};
	stack: entasis.Task_Stack;
	testing.expect_value(
		t, entasis.task_stack_initialize(&stack, WORKER_COUNT, procedure_buffer, &pool),
		entasis.Task_Scheduling_Status.Ok,
	);
	defer entasis.task_stack_dispose(&stack, &pool);

	dispatcher: entasis.Thread_Dispatcher;
	testing.expect_value(
		t, entasis.thread_dispatcher_initialize(&dispatcher, WORKER_COUNT),
		entasis.Threading_Status.Ok,
	);
	boundary := entasis.thread_dispatcher_boundary(&dispatcher);
	allocations_after_startup := dispatcher.general_allocator_call_count;
	initial_outstanding := worker_pool_outstanding_slot_count(&dispatcher);
	expected_hash: u64;
	warmed_block_count := 0;
	warmed_allocated_bytes := u64(0);
	for iteration in 0 ..< 64
	{
		testing.expect_value(
			t, entasis.task_stack_begin_epoch(&stack, boundary),
			entasis.Task_Scheduling_Status.Ok,
		);
		state := Multi_Context{stack=&stack};
		completion := entasis.Task_Record{
			task_context=&state,
			procedure_index=1,
			procedure_state=.Present,
		};
		reservation, reserve_status := entasis.task_stack_reserve_batch(
			&stack, boundary, 0, TASK_COUNT, u64(iteration), .Present, completion,
		);
		testing.expect_value(t, reserve_status, entasis.Task_Scheduling_Status.Ok);
		for index in 0 ..< TASK_COUNT
		{
			reservation.tasks[index] = {
				task_context=&state,
				task_id=i64(index),
				range_start=i64(index * 3),
				range_end=i64(index * 3 + 3),
				procedure_state=.Present,
			};
		}
		testing.expect_value(
			t, entasis.task_stack_commit_batch(&stack, boundary, &reservation),
			entasis.Task_Scheduling_Status.Ok,
		);
		testing.expect_value(t, reservation.state, entasis.Task_Batch_State.Published);
		job := (^entasis.Task_Job)(stack.workers.memory[0].job_head);
		testing.expect(t, job != nil);
		testing.expect_value(t, job.ownership_previous, uintptr(0));
		continuation := reservation.continuation_handle;
		testing.expect_value(
			t, boundary.dispatch(
			boundary, entasis.task_stack_dispatch_worker, WORKER_COUNT, &stack,
		),
			entasis.Threading_Status.Ok,
		);
		for index in 0 ..< TASK_COUNT
		{
			testing.expect_value(t, state.counts[index], i32(1));
		}
		testing.expect_value(t, state.entered_count, i32(WORKER_COUNT));
		for worker_index in 0 ..< WORKER_COUNT
		{
			testing.expect(t, state.worker_hits[worker_index] > 0);
		}
		testing.expect_value(t, state.completion_count, i32(1));
		testing.expect_value(
			t, entasis.task_stack_continuation_state(&stack, continuation),
			entasis.Continuation_State.Completed,
		);
		hash := result_hash(&state);
		if iteration == 0
		{
			expected_hash = hash;
		}
		testing.expect_value(t, hash, expected_hash);
		testing.expect_value(
			t, entasis.task_stack_end_epoch(&stack, boundary),
			entasis.Task_Scheduling_Status.Ok,
		);
		testing.expect_value(
			t, worker_pool_outstanding_slot_count(&dispatcher), initial_outstanding,
		);
		if iteration == 0
		{
			warmed_block_count = worker_pool_block_count(&dispatcher);
			warmed_allocated_bytes = entasis.worker_buffer_pools_total_allocated_byte_count(
				&dispatcher.worker_pools,
			);
		}
		else
		{
			testing.expect_value(
				t, worker_pool_block_count(&dispatcher), warmed_block_count,
			);
			testing.expect_value(
				t, entasis.worker_buffer_pools_total_allocated_byte_count(
				&dispatcher.worker_pools,
			),
				warmed_allocated_bytes,
			);
		}
		testing.expect_value(
			t, dispatcher.general_allocator_call_count, allocations_after_startup,
		);
	}
	testing.expect_value(
		t, entasis.thread_dispatcher_shutdown(&dispatcher),
		entasis.Threading_Status.Ok,
	);
	testing.expect_value(
		t, entasis.thread_dispatcher_shutdown(&dispatcher),
		entasis.Threading_Status.Disposed,
	);
}
