package tasking_single_tests

import "core:testing"
import entasis "entasis:entasis_utilities"

Single_Context :: struct
{
	order:            [16]i64,
	order_count:      int,
	range_sum:        i64,
	completion_count: int,
}

record_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^entasis.Thread_Dispatcher_Boundary,
)
{
	_ = worker_index;
	_ = dispatcher;
	state := (^Single_Context)(task_context);
	state.order[state.order_count] = task_id;
	state.order_count += 1;
	for index := range_start; index < range_end; index += 1
	{
		state.range_sum += index;
	}
}

completion_task :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^entasis.Thread_Dispatcher_Boundary,
)
{
	_ = task_id;
	_ = range_start;
	_ = range_end;
	_ = worker_index;
	_ = dispatcher;
	state := (^Single_Context)(task_context);
	state.completion_count += 1;
}

Tag_Filter_Context :: struct
{
	allowed_tag: u64,
}

allow_tag :: proc "contextless" (
	tag: u64, filter_context: rawptr,
) -> entasis.Presence_Status
{
	ctx := (^Tag_Filter_Context)(filter_context);
	if tag == ctx.allowed_tag
	{
		return .Present;
	}
	return .Missing;
}

serial_dispatch :: proc "contextless" (
	dispatcher: ^entasis.Thread_Dispatcher_Boundary, worker: entasis.Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> entasis.Threading_Status
{
	if dispatcher == nil || worker == nil || maximum_worker_count <= 0
	{
		return .Invalid_Argument;
	}
	dispatcher.unmanaged_context = unmanaged_context;
	worker_count := min(maximum_worker_count, dispatcher.worker_count);
	for worker_index in 0 ..< worker_count
	{
		worker(worker_index, dispatcher);
	}
	dispatcher.unmanaged_context = nil;
	return .Ok;
}

serial_worker_pool :: proc (
	dispatcher: ^entasis.Thread_Dispatcher_Boundary, worker_index: int,
) -> (^entasis.Buffer_Pool, entasis.Threading_Status)
{
	if dispatcher == nil || dispatcher.dispatcher == nil || worker_index != 0
	{
		return nil, .Invalid_Argument;
	}
	return (^entasis.Buffer_Pool)(dispatcher.dispatcher), .Ok;
}

prepare_task_pool :: proc(t: ^testing.T, pool: ^entasis.Buffer_Pool)
{
	testing.expect_value(t, entasis.buffer_pool_initialize(pool, 128), entasis.Memory_Status.Ok);
	for power in 0 ..= 14
	{
		testing.expect_value(
			t, entasis.buffer_pool_ensure_capacity_for_power(pool, 32768, power),
			entasis.Memory_Status.Ok,
		);
	}
}

pool_outstanding_slot_count :: proc "contextless" (pool: ^entasis.Buffer_Pool) -> int
{
	count := 0;
	for power in 0 ..< entasis.BUFFER_POOL_POWER_COUNT
	{
		power_pool := &pool.pools[power];
		count += power_pool.next_slot - power_pool.free_count;
	}
	return count;
}

@(test)
caller_supplied_dispatcher_runs_ranges_dependencies_and_continuation_in_source_order :: proc(
	t: ^testing.T,
)
{
	pool: entasis.Buffer_Pool;
	prepare_task_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);
	procedures := [2]entasis.Task_Procedure{record_task, completion_task};
	procedure_buffer := entasis.Buffer(entasis.Task_Procedure){
		memory=&procedures[0],
		length=2,
		id=-1,
	};
	stack: entasis.Task_Stack;
	testing.expect_value(
		t, entasis.task_stack_initialize(&stack, 1, procedure_buffer, &pool),
		entasis.Task_Scheduling_Status.Ok,
	);
	defer entasis.task_stack_dispose(&stack, &pool);
	boundary := entasis.Thread_Dispatcher_Boundary{
		dispatcher=&pool,
		dispatch=serial_dispatch,
		worker_pool=serial_worker_pool,
		worker_count=1,
	};
	testing.expect_value(
		t, entasis.task_stack_begin_epoch(&stack, &boundary),
		entasis.Task_Scheduling_Status.Ok,
	);

	state: Single_Context;
	completion := entasis.Task_Record{
		task_context=&state,
		procedure_index=1,
		procedure_state=.Present,
	};
	reservation, reserve_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, 4, 0, .Present, completion,
	);
	testing.expect_value(t, reserve_status, entasis.Task_Scheduling_Status.Ok);
	for index in 0 ..< reservation.task_count
	{
		reservation.tasks[index] = {
			task_context=&state,
			task_id=i64(index),
			range_start=i64(index * 2),
			range_end=i64(index * 2 + 2),
			procedure_index=0,
			procedure_state=.Present,
		};
	}
	testing.expect_value(
		t, entasis.task_stack_commit_batch(&stack, &boundary, &reservation),
		entasis.Task_Scheduling_Status.Ok,
	);
	continuation := reservation.continuation_handle;
	caller_pool, caller_pool_status := boundary.worker_pool(&boundary, 0);
	testing.expect_value(t, caller_pool_status, entasis.Threading_Status.Ok);
	testing.expect(t, caller_pool == &pool);
	testing.expect_value(
		t, boundary.dispatch(&boundary, entasis.task_stack_drain_worker, 1, &stack),
		entasis.Threading_Status.Ok,
	);
	expected_order := [4]i64{3, 2, 1, 0};
	for value, index in expected_order
	{
		testing.expect_value(t, state.order[index], value);
	}
	testing.expect_value(t, state.range_sum, i64(28));
	testing.expect_value(t, state.completion_count, 1);
	testing.expect_value(
		t, entasis.task_stack_continuation_state(&stack, continuation),
		entasis.Continuation_State.Completed,
	);

	filtered_state: Single_Context;
	task_ids := [2]i64{10, 20};
	tags := [2]u64{10, 22};
	for task_id, index in task_ids
	{
		filtered_reservation, filtered_status := entasis.task_stack_reserve_batch(
			&stack, &boundary, 0, 1, tags[index], .Missing,
		);
		testing.expect_value(t, filtered_status, entasis.Task_Scheduling_Status.Ok);
		filtered_reservation.tasks[0] = {
			task_context=&filtered_state,
			task_id=task_id,
			procedure_index=0,
			procedure_state=.Present,
		};
		testing.expect_value(
			t, entasis.task_stack_commit_batch(&stack, &boundary, &filtered_reservation),
			entasis.Task_Scheduling_Status.Ok,
		);
	}
	filter_context := Tag_Filter_Context{allowed_tag=10};
	testing.expect_value(
		t, entasis.task_stack_try_pop_and_run(&stack, 0, &boundary, allow_tag, &filter_context),
		entasis.Task_Pop_Status.Success,
	);
	testing.expect_value(t, filtered_state.order_count, 1);
	testing.expect_value(t, filtered_state.order[0], i64(10));
	testing.expect_value(
		t, entasis.task_stack_try_pop_and_run(&stack, 0, &boundary),
		entasis.Task_Pop_Status.Success,
	);
	testing.expect_value(t, filtered_state.order_count, 2);
	testing.expect_value(t, filtered_state.order[1], i64(20));
	testing.expect_value(
		t, entasis.task_stack_end_epoch(&stack, &boundary),
		entasis.Task_Scheduling_Status.Ok,
	);
}

@(test)
batch_admission_failure_is_transactional :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_task_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);
	procedures := [1]entasis.Task_Procedure{record_task};
	procedure_buffer := entasis.Buffer(entasis.Task_Procedure){
		memory=&procedures[0],
		length=1,
		id=-1,
	};
	stack: entasis.Task_Stack;
	testing.expect_value(
		t, entasis.task_stack_initialize(&stack, 1, procedure_buffer, &pool),
		entasis.Task_Scheduling_Status.Ok,
	);
	defer entasis.task_stack_dispose(&stack, &pool);
	boundary := entasis.Thread_Dispatcher_Boundary{
		dispatcher=&pool,
		dispatch=serial_dispatch,
		worker_pool=serial_worker_pool,
		worker_count=1,
	};
	testing.expect_value(
		t, entasis.task_stack_begin_epoch(&stack, &boundary),
		entasis.Task_Scheduling_Status.Ok,
	);
	outstanding_before := pool_outstanding_slot_count(&pool);
	state: Single_Context;

	failed, failed_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, max(int), 7, .Present,
	);
	testing.expect_value(
		t, failed_status, entasis.Task_Scheduling_Status.Capacity_Missing,
	);
	testing.expect_value(t, failed.state, entasis.Task_Batch_State.Aborted);
	testing.expect_value(
		t, failed.continuation_handle, entasis.Task_Continuation_Handle(0),
	);
	testing.expect_value(t, stack.head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].job_head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].continuation_head, uintptr(0));
	testing.expect_value(t, pool_outstanding_slot_count(&pool), outstanding_before);
	testing.expect_value(t, state.order_count, 0);

	invalid_completion := entasis.Task_Record{
		procedure_index=1,
		procedure_state=.Present,
	};
	invalid, invalid_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, 1, 8, .Present, invalid_completion,
	);
	testing.expect_value(
		t, invalid_status, entasis.Task_Scheduling_Status.Procedure_Missing,
	);
	testing.expect_value(t, invalid.state, entasis.Task_Batch_State.Aborted);
	testing.expect_value(t, pool_outstanding_slot_count(&pool), outstanding_before);

	job_byte_count := entasis.TASK_JOB_TASK_OFFSET + size_of(entasis.Task_Record);
	job_power, job_power_status := entasis.buffer_pool_power_for_count(
		u8, job_byte_count,
	);
	continuation_power, continuation_power_status :=
		entasis.buffer_pool_power_for_count(
		u8, size_of(entasis.Task_Continuation_Block),
	);
	testing.expect_value(t, job_power_status, entasis.Memory_Status.Ok);
	testing.expect_value(
		t, continuation_power_status, entasis.Memory_Status.Ok,
	);
	testing.expect(t, job_power != continuation_power);
	job_pool := &pool.pools[job_power];
	continuation_pool := &pool.pools[continuation_power];
	testing.expect_value(t, job_pool.next_slot, 0);
	testing.expect_value(t, job_pool.free_count, 0);

	saved_continuation_next_slot := continuation_pool.next_slot;
	saved_continuation_free_count := continuation_pool.free_count;
	continuation_pool.next_slot = entasis.BUFFER_POOL_ID_SLOT_COUNT;
	continuation_pool.free_count = 0;
	partial, partial_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, 1, 9, .Present,
	);
	continuation_pool.next_slot = saved_continuation_next_slot;
	continuation_pool.free_count = saved_continuation_free_count;
	testing.expect_value(
		t, partial_status, entasis.Task_Scheduling_Status.Capacity_Missing,
	);
	testing.expect_value(t, partial.state, entasis.Task_Batch_State.Aborted);
	testing.expect_value(
		t, partial.continuation_handle, entasis.Task_Continuation_Handle(0),
	);
	testing.expect_value(t, job_pool.next_slot, 1);
	testing.expect_value(t, job_pool.free_count, 1);
	testing.expect_value(
		t, continuation_pool.next_slot, saved_continuation_next_slot,
	);
	testing.expect_value(
		t, continuation_pool.free_count, saved_continuation_free_count,
	);
	testing.expect_value(t, pool_outstanding_slot_count(&pool), outstanding_before);
	testing.expect_value(t, stack.head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].job_head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].continuation_head, uintptr(0));
	testing.expect_value(t, state.order_count, 0);

	commit_failure, commit_failure_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, 1, 10, .Present,
	);
	testing.expect_value(
		t, commit_failure_status, entasis.Task_Scheduling_Status.Ok,
	);
	commit_failure.tasks[0] = {
		task_context=&state,
		procedure_index=1,
		procedure_state=.Present,
	};
	testing.expect_value(
		t, entasis.task_stack_commit_batch(
		&stack, &boundary, &commit_failure,
	),
		entasis.Task_Scheduling_Status.Procedure_Missing,
	);
	testing.expect_value(
		t, commit_failure.state, entasis.Task_Batch_State.Aborted,
	);
	testing.expect_value(
		t, commit_failure.continuation_handle,
		entasis.Task_Continuation_Handle(0),
	);
	testing.expect_value(t, pool_outstanding_slot_count(&pool), outstanding_before);
	testing.expect_value(t, stack.head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].job_head, uintptr(0));
	testing.expect_value(t, stack.workers.memory[0].continuation_head, uintptr(0));
	testing.expect_value(t, state.order_count, 0);

	reservation, reserve_status := entasis.task_stack_reserve_batch(
		&stack, &boundary, 0, 4, 11, .Missing,
	);
	testing.expect_value(t, reserve_status, entasis.Task_Scheduling_Status.Ok);
	for index in 0 ..< reservation.task_count
	{
		reservation.tasks[index] = {
			task_context=&state,
			task_id=i64(index),
			procedure_index=0,
			procedure_state=.Present,
		};
	}
	testing.expect_value(
		t, entasis.task_stack_commit_batch(&stack, &boundary, &reservation),
		entasis.Task_Scheduling_Status.Ok,
	);
	testing.expect_value(t, reservation.state, entasis.Task_Batch_State.Published);
	testing.expect(t, stack.head != 0);
	testing.expect(t, stack.workers.memory[0].job_head != 0);
	testing.expect_value(
		t, boundary.dispatch(&boundary, entasis.task_stack_drain_worker, 1, &stack),
		entasis.Threading_Status.Ok,
	);
	testing.expect_value(t, state.order_count, 4);
	testing.expect_value(
		t, entasis.task_stack_end_epoch(&stack, &boundary),
		entasis.Task_Scheduling_Status.Ok,
	);
	testing.expect_value(t, pool_outstanding_slot_count(&pool), outstanding_before);
}
