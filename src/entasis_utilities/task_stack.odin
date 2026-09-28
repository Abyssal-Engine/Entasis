// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:runtime"
import "core:sync"
import "core:thread"

TASK_STACK_SPIN_LIMIT :: 32;
TASK_CONTINUATION_BLOCK_CAPACITY :: 128;

Task_Scheduling_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Capacity_Missing,
	Procedure_Missing,
	Empty,
	Stop,
	Disposed,
}

Task_Pop_Status :: enum u8
{
	Success,
	Empty,
	Stop,
}

Task_Stop_State :: enum u32
{
	Running,
	Requested,
}

Task_Epoch_State :: enum u8
{
	Inactive,
	Active,
}

Task_Batch_State :: enum u8
{
	Uninitialized,
	Reserved,
	Published,
	Aborted,
}

Continuation_State :: enum u8
{
	Uninitialized,
	Pending,
	Completed,
}

Task_Procedure_State :: enum u8
{
	Missing,
	Present,
}

Task_Continuation_Handle :: distinct uintptr;

Task_Procedure :: #type proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
);

Job_Filter_Proc :: #type proc "contextless" (tag: u64, filter_context: rawptr) -> Presence_Status;

Task_Record :: struct
{
	task_context:       rawptr,
	task_id:            i64,
	range_start:        i64,
	range_end:          i64,
	continuation:       Task_Continuation_Handle,
	procedure_index:    u16,
	procedure_state:    Task_Procedure_State,
}

Task_Continuation :: struct
{
	on_completed:      Task_Record,
	remaining_counter: i32,
	state:             Continuation_State,
}

Task_Continuation_Block :: struct
{
	ownership_previous: uintptr,
	allocation_length:  i32,
	allocation_id:      i32,
	count:              i32,
	continuations:      [TASK_CONTINUATION_BLOCK_CAPACITY]Task_Continuation,
}

Task_Job :: struct
{
	tasks:                [^]Task_Record,
	task_count:           i32,
	remaining_counter:    i32,
	allocation_length:    i32,
	allocation_id:        i32,
	publication_previous: uintptr,
	ownership_previous:   uintptr,
	tag:                  u64,
}

TASK_JOB_TASK_OFFSET :: (
	(size_of(Task_Job) + align_of(Task_Record) - 1) / align_of(Task_Record)
) * align_of(Task_Record);

Task_Worker :: struct
{
	job_head:          uintptr,
	continuation_head: uintptr,
}

Task_Batch_Reservation :: struct
{
	job_block:                 Buffer(u8),
	continuation_block_buffer: Buffer(u8),
	job:                       ^Task_Job,
	tasks:                     [^]Task_Record,
	continuation_block:        ^Task_Continuation_Block,
	continuation_slot:         ^Task_Continuation,
	continuation_handle:       Task_Continuation_Handle,
	on_completed:              Task_Record,
	task_count:                int,
	worker_index:              int,
	tag:                       u64,
	continuation_state:        Presence_Status,
	new_continuation_block:    Presence_Status,
	state:                     Task_Batch_State,
}

Task_Stop_Pad :: struct
{
	leading:  [128]u8,
	value:    u32,
	trailing: [140]u8,
}

#assert(size_of(Task_Stop_Pad) == 272);
#assert(offset_of(Task_Stop_Pad, value) == 128);
#assert(TASK_JOB_TASK_OFFSET % align_of(Task_Record) == 0);

Task_Stack :: struct
{
	workers:    Buffer(Task_Worker),
	procedures: Buffer(Task_Procedure),
	stop:       Task_Stop_Pad,
	head:       uintptr,
	epoch:      Task_Epoch_State,
	state:      Pool_State,
}

task_stack_wait_once :: proc "contextless" (iteration: ^u32)
{
	if iteration^ < TASK_STACK_SPIN_LIMIT
	{
		for _ in 0 ..< min(iteration^ + 1, u32(16))
		{
			sync.cpu_relax();
		}
		iteration^ += 1;
		return;
	}
	context = runtime.default_context();
	thread.yield();
}

task_stack_compare_exchange_head :: proc "contextless" (
	target: ^uintptr, expected, desired: uintptr,
) -> (uintptr, Atomic_Exchange_Status)
{
	old, boundary_success := sync.atomic_compare_exchange_strong_explicit(
		target, expected, desired, .Seq_Cst, .Seq_Cst,
	);
	if boundary_success
	{
		return old, .Succeeded;
	}
	return old, .Failed;
}

task_stack_return_initialization_buffer :: proc (stack: ^Task_Stack, pool: ^Buffer_Pool)
{
	if stack.workers.memory != nil
	{
		_ = buffer_pool_return(pool, &stack.workers);
	}
}

task_stack_initialize :: proc (
	stack: ^Task_Stack, worker_count: int, procedures: Buffer(Task_Procedure), pool: ^Buffer_Pool,
) -> Task_Scheduling_Status
{
	if stack == nil || pool == nil || worker_count <= 0 ||
		procedures.memory == nil || procedures.length <= 0
	{
		return .Invalid_Argument;
	}
	stack^ = {};
	workers, worker_status := buffer_pool_take(pool, Task_Worker, worker_count);
	if worker_status != .Ok
	{
		return .Capacity_Missing;
	}
	stack.workers = workers;
	stack.procedures = procedures;
	for worker_index in 0 ..< worker_count
	{
		stack.workers.memory[worker_index] = {};
	}
	sync.atomic_store_explicit(&stack.head, uintptr(0), .Release);
	sync.atomic_store_explicit(&stack.stop.value, u32(Task_Stop_State.Running), .Release);
	stack.epoch = .Inactive;
	stack.state = .Ready;
	return .Ok;
}

task_stack_begin_epoch :: proc (
	stack: ^Task_Stack, dispatcher: ^Thread_Dispatcher_Boundary,
) -> Task_Scheduling_Status
{
	if stack == nil || stack.state != .Ready
	{
		return .Disposed;
	}
	if stack.epoch != .Inactive || dispatcher == nil || dispatcher.dispatch == nil ||
		dispatcher.worker_pool == nil || dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > int(stack.workers.length)
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< dispatcher.worker_count
	{
		worker := &stack.workers.memory[worker_index];
		if worker.job_head != 0 || worker.continuation_head != 0
		{
			return .Invalid_Argument;
		}
		pool, pool_status := dispatcher.worker_pool(dispatcher, worker_index);
		if pool_status != .Ok || pool == nil || pool.state != .Ready
		{
			return .Invalid_Argument;
		}
	}
	sync.atomic_store_explicit(&stack.head, uintptr(0), .Release);
	sync.atomic_store_explicit(&stack.stop.value, u32(Task_Stop_State.Running), .Release);
	stack.epoch = .Active;
	return .Ok;
}

task_stack_return_job_block :: proc (
	pool: ^Buffer_Pool, job: ^Task_Job,
) -> Memory_Status
{
	allocation := Buffer(u8){
		memory=([^]u8)(job),
		length=job.allocation_length,
		id=job.allocation_id,
	};
	return buffer_pool_return(pool, &allocation);
}

task_stack_return_continuation_block :: proc (
	pool: ^Buffer_Pool, block: ^Task_Continuation_Block,
) -> Memory_Status
{
	allocation := Buffer(u8){
		memory=([^]u8)(block),
		length=block.allocation_length,
		id=block.allocation_id,
	};
	return buffer_pool_return(pool, &allocation);
}

task_stack_end_epoch :: proc (
	stack: ^Task_Stack, dispatcher: ^Thread_Dispatcher_Boundary,
) -> Task_Scheduling_Status
{
	if stack == nil || stack.state != .Ready
	{
		return .Disposed;
	}
	if stack.epoch != .Active || dispatcher == nil || dispatcher.worker_pool == nil ||
		dispatcher.worker_count <= 0 || dispatcher.worker_count > int(stack.workers.length)
	{
		return .Invalid_Argument;
	}
	for worker_index in 0 ..< dispatcher.worker_count
	{
		pool, pool_status := dispatcher.worker_pool(dispatcher, worker_index);
		if pool_status != .Ok || pool == nil || pool.state != .Ready
		{
			return .Invalid_Argument;
		}
	}
	return_status := Task_Scheduling_Status.Ok;
	for worker_index in 0 ..< dispatcher.worker_count
	{
		pool, _ := dispatcher.worker_pool(dispatcher, worker_index);
		worker := &stack.workers.memory[worker_index];
		job_address := worker.job_head;
		for job_address != 0
		{
			job := (^Task_Job)(job_address);
			ownership_previous := job.ownership_previous;
			if task_stack_return_job_block(pool, job) != .Ok
			{
				return_status = .Invalid_Argument;
			}
			job_address = ownership_previous;
		}
		continuation_address := worker.continuation_head;
		for continuation_address != 0
		{
			block := (^Task_Continuation_Block)(continuation_address);
			ownership_previous := block.ownership_previous;
			if task_stack_return_continuation_block(pool, block) != .Ok
			{
				return_status = .Invalid_Argument;
			}
			continuation_address = ownership_previous;
		}
		worker^ = {};
	}
	sync.atomic_store_explicit(&stack.head, uintptr(0), .Release);
	sync.atomic_store_explicit(&stack.stop.value, u32(Task_Stop_State.Running), .Release);
	stack.epoch = .Inactive;
	return return_status;
}

task_stack_validate_completion :: proc "contextless" (
	stack: ^Task_Stack, continuation_state: Presence_Status, on_completed: Task_Record,
) -> Task_Scheduling_Status
{
	if continuation_state == .Missing
	{
		if on_completed.procedure_state == .Present
		{
			return .Invalid_Argument;
		}
		return .Ok;
	}
	if on_completed.continuation != Task_Continuation_Handle(0)
	{
		return .Invalid_Argument;
	}
	if on_completed.procedure_state == .Present &&
		(int(on_completed.procedure_index) >= int(stack.procedures.length) ||
		stack.procedures.memory[on_completed.procedure_index] == nil)
	{
		return .Procedure_Missing;
	}
	return .Ok;
}

task_stack_reserve_batch :: proc (
	stack: ^Task_Stack, dispatcher: ^Thread_Dispatcher_Boundary,
	worker_index, task_count: int, tag: u64, continuation_state: Presence_Status,
	on_completed: Task_Record = {},
) -> (reservation: Task_Batch_Reservation, status: Task_Scheduling_Status)
{
	reservation.state = .Aborted;
	if stack == nil || stack.state != .Ready || stack.epoch != .Active ||
		dispatcher == nil || dispatcher.worker_pool == nil || task_count <= 0 ||
		worker_index < 0 || worker_index >= dispatcher.worker_count ||
		worker_index >= int(stack.workers.length)
	{
		return reservation, .Invalid_Argument;
	}
	completion_status := task_stack_validate_completion(stack, continuation_state, on_completed);
	if completion_status != .Ok
	{
		return reservation, completion_status;
	}
	if task_count > int(max(i32)) ||
		task_count > (max(int) - TASK_JOB_TASK_OFFSET) / size_of(Task_Record)
	{
		return reservation, .Capacity_Missing;
	}
	pool, pool_status := dispatcher.worker_pool(dispatcher, worker_index);
	if pool_status != .Ok || pool == nil || pool.state != .Ready
	{
		return reservation, .Invalid_Argument;
	}
	job_byte_count := TASK_JOB_TASK_OFFSET + task_count * size_of(Task_Record);
	job_block, job_status := buffer_pool_take(pool, u8, job_byte_count);
	if job_status != .Ok
	{
		return reservation, .Capacity_Missing;
	}
	job := (^Task_Job)(job_block.memory);
	task_address := uintptr(job_block.memory) + uintptr(TASK_JOB_TASK_OFFSET);
	if task_address < uintptr(job_block.memory) ||
		task_address % uintptr(align_of(Task_Record)) != 0
	{
		_ = buffer_pool_return(pool, &job_block);
		return reservation, .Capacity_Missing;
	}
	tasks := ([^]Task_Record)(task_address);
	job^ = {
		tasks=tasks,
		task_count=i32(task_count),
		remaining_counter=i32(task_count),
		allocation_length=job_block.length,
		allocation_id=job_block.id,
		tag=tag,
	};
	reservation = {
		job_block=job_block,
		job=job,
		tasks=tasks,
		on_completed=on_completed,
		task_count=task_count,
		worker_index=worker_index,
		tag=tag,
		continuation_state=continuation_state,
		state=.Reserved,
	};
	if continuation_state == .Present
	{
		worker := &stack.workers.memory[worker_index];
		block := (^Task_Continuation_Block)(worker.continuation_head);
		if block == nil || block.count >= TASK_CONTINUATION_BLOCK_CAPACITY
		{
			block_buffer, block_status := buffer_pool_take(
				pool, u8, size_of(Task_Continuation_Block),
			);
			if block_status != .Ok
			{
				_ = buffer_pool_return(pool, &reservation.job_block);
				reservation = {state=.Aborted};
				return reservation, .Capacity_Missing;
			}
			block = (^Task_Continuation_Block)(block_buffer.memory);
			block^ = {
				allocation_length=block_buffer.length,
				allocation_id=block_buffer.id,
			};
			reservation.continuation_block_buffer = block_buffer;
			reservation.new_continuation_block = .Present;
		}
		if block.count < 0 || block.count >= TASK_CONTINUATION_BLOCK_CAPACITY
		{
			if reservation.new_continuation_block == .Present
			{
				_ = buffer_pool_return(pool, &reservation.continuation_block_buffer);
			}
			_ = buffer_pool_return(pool, &reservation.job_block);
			reservation = {state=.Aborted};
			return reservation, .Capacity_Missing;
		}
		reservation.continuation_block = block;
		reservation.continuation_slot = &block.continuations[block.count];
		reservation.continuation_handle = Task_Continuation_Handle(uintptr(reservation.continuation_slot));
	}
	return reservation, .Ok;
}

task_stack_abort_batch :: proc (
	stack: ^Task_Stack, dispatcher: ^Thread_Dispatcher_Boundary,
	reservation: ^Task_Batch_Reservation,
) -> Task_Scheduling_Status
{
	if stack == nil || stack.state != .Ready || stack.epoch != .Active ||
		dispatcher == nil || dispatcher.worker_pool == nil || reservation == nil ||
		reservation.state != .Reserved ||
		reservation.worker_index < 0 || reservation.worker_index >= dispatcher.worker_count
	{
		return .Invalid_Argument;
	}
	pool, pool_status := dispatcher.worker_pool(dispatcher, reservation.worker_index);
	if pool_status != .Ok || pool == nil || pool.state != .Ready
	{
		return .Invalid_Argument;
	}
	return_status := Task_Scheduling_Status.Ok;
	if reservation.job_block.memory != nil &&
		buffer_pool_return(pool, &reservation.job_block) != .Ok
	{
		return_status = .Invalid_Argument;
	}
	if reservation.new_continuation_block == .Present &&
		reservation.continuation_block_buffer.memory != nil &&
		buffer_pool_return(pool, &reservation.continuation_block_buffer) != .Ok
	{
		return_status = .Invalid_Argument;
	}
	reservation.job = nil;
	reservation.tasks = nil;
	reservation.continuation_block = nil;
	reservation.continuation_slot = nil;
	reservation.continuation_handle = Task_Continuation_Handle(0);
	reservation.state = .Aborted;
	return return_status;
}

task_stack_commit_batch :: proc (
	stack: ^Task_Stack, dispatcher: ^Thread_Dispatcher_Boundary,
	reservation: ^Task_Batch_Reservation,
) -> Task_Scheduling_Status
{
	if stack == nil || stack.state != .Ready || stack.epoch != .Active ||
		dispatcher == nil || reservation == nil || reservation.state != .Reserved ||
		reservation.task_count <= 0 || reservation.job == nil || reservation.tasks == nil ||
		reservation.job_block.memory == nil ||
		reservation.worker_index < 0 || reservation.worker_index >= int(stack.workers.length) ||
		reservation.worker_index >= dispatcher.worker_count ||
		reservation.job != (^Task_Job)(reservation.job_block.memory)
	{
		if reservation != nil && reservation.state == .Reserved
		{
			_ = task_stack_abort_batch(stack, dispatcher, reservation);
		}
		return .Invalid_Argument;
	}
	if reservation.continuation_state == .Present
	{
		if reservation.continuation_block == nil || reservation.continuation_slot == nil ||
			reservation.continuation_handle == Task_Continuation_Handle(0)
		{
			_ = task_stack_abort_batch(stack, dispatcher, reservation);
			return .Invalid_Argument;
		}
	}
	else if reservation.continuation_handle != Task_Continuation_Handle(0)
	{
		_ = task_stack_abort_batch(stack, dispatcher, reservation);
		return .Invalid_Argument;
	}
	for index in 0 ..< reservation.task_count
	{
		task := &reservation.tasks[index];
		if task.procedure_state != .Present ||
			int(task.procedure_index) >= int(stack.procedures.length) ||
			stack.procedures.memory[task.procedure_index] == nil
		{
			_ = task_stack_abort_batch(stack, dispatcher, reservation);
			return .Procedure_Missing;
		}
		if task.continuation != Task_Continuation_Handle(0)
		{
			_ = task_stack_abort_batch(stack, dispatcher, reservation);
			return .Invalid_Argument;
		}
	}
	worker := &stack.workers.memory[reservation.worker_index];
	continuation_handle := Task_Continuation_Handle(0);
	if reservation.continuation_state == .Present
	{
		block := reservation.continuation_block;
		slot := reservation.continuation_slot;
		slot^ = {
			on_completed=reservation.on_completed,
			remaining_counter=i32(reservation.task_count),
			state=.Pending,
		};
		block.count += 1;
		if reservation.new_continuation_block == .Present
		{
			block.ownership_previous = worker.continuation_head;
			worker.continuation_head = uintptr(block);
		}
		continuation_handle = reservation.continuation_handle;
	}
	for index in 0 ..< reservation.task_count
	{
		reservation.tasks[index].continuation = continuation_handle;
	}
	job := reservation.job;
	job.ownership_previous = worker.job_head;
	worker.job_head = uintptr(job);
	for
	{
		head := sync.atomic_load_explicit(&stack.head, .Acquire);
		job.publication_previous = head;
		_, exchange_status := task_stack_compare_exchange_head(&stack.head, head, uintptr(job));
		if exchange_status == .Succeeded
		{
			break;
		}
	}
	reservation.state = .Published;
	return .Ok;
}

task_stack_continuation_state :: proc "contextless" (
	stack: ^Task_Stack, continuation_handle: Task_Continuation_Handle,
) -> Continuation_State
{
	if stack == nil || continuation_handle == Task_Continuation_Handle(0)
	{
		return .Uninitialized;
	}
	continuation := (^Task_Continuation)(uintptr(continuation_handle));
	remaining := sync.atomic_load_explicit(&continuation.remaining_counter, .Acquire);
	if remaining <= 0
	{
		return .Completed;
	}
	return continuation.state;
}

task_stack_run_task :: proc "contextless" (
	stack: ^Task_Stack, task: Task_Record, worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
) -> Task_Scheduling_Status
{
	if stack == nil || stack.epoch != .Active
	{
		return .Invalid_Argument;
	}
	if task.procedure_state != .Present ||
		int(task.procedure_index) >= int(stack.procedures.length)
	{
		return .Procedure_Missing;
	}
	if task.continuation != Task_Continuation_Handle(0) &&
		task_stack_continuation_state(stack, task.continuation) != .Pending
	{
		return .Invalid_Argument;
	}
	procedure := stack.procedures.memory[task.procedure_index];
	if procedure == nil
	{
		return .Procedure_Missing;
	}
	procedure(task.task_id, task.task_context, task.range_start, task.range_end, worker_index, dispatcher);
	if task.continuation != Task_Continuation_Handle(0)
	{
		continuation := (^Task_Continuation)(uintptr(task.continuation));
		old_remaining := sync.atomic_sub_explicit(&continuation.remaining_counter, i32(1), .Seq_Cst);
		if old_remaining == 1
		{
			continuation.state = .Completed;
			completion := continuation.on_completed;
			if completion.procedure_state == .Present &&
				int(completion.procedure_index) < int(stack.procedures.length)
			{
				completion_procedure := stack.procedures.memory[completion.procedure_index];
				if completion_procedure != nil
				{
					completion_procedure(
						completion.task_id, completion.task_context,
						completion.range_start, completion.range_end,
						worker_index, dispatcher,
					);
				}
			}
		}
	}
	return .Ok;
}

task_stack_try_pop :: proc "contextless" (
	stack: ^Task_Stack, filter: Job_Filter_Proc = nil, filter_context: rawptr = nil,
) -> (Task_Record, Task_Pop_Status)
{
	if stack == nil || stack.epoch != .Active
	{
		return {}, .Stop;
	}
	job_address := sync.atomic_load_explicit(&stack.head, .Acquire);
	for
	{
		if job_address == 0
		{
			if Task_Stop_State(sync.atomic_load_explicit(&stack.stop.value, .Acquire)) == .Requested
			{
				return {}, .Stop;
			}
			return {}, .Empty;
		}
		job := (^Task_Job)(job_address);
		if filter != nil && filter(job.tag, filter_context) == .Missing
		{
			job_address = job.publication_previous;
			continue;
		}
		old_remaining := sync.atomic_sub_explicit(&job.remaining_counter, i32(1), .Seq_Cst);
		new_remaining := old_remaining - 1;
		if new_remaining >= 0
		{
			return job.tasks[int(new_remaining)], .Success;
		}
		current_head := sync.atomic_load_explicit(&stack.head, .Acquire);
		if current_head == job_address
		{
			_, _ = task_stack_compare_exchange_head(
				&stack.head, job_address, job.publication_previous,
			);
		}
		job_address = sync.atomic_load_explicit(&stack.head, .Acquire);
	}
}

task_stack_try_pop_and_run :: proc "contextless" (
	stack: ^Task_Stack, worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
	filter: Job_Filter_Proc = nil, filter_context: rawptr = nil,
) -> Task_Pop_Status
{
	task, pop_status := task_stack_try_pop(stack, filter, filter_context);
	if pop_status == .Success
	{
		_ = task_stack_run_task(stack, task, worker_index, dispatcher);
	}
	return pop_status;
}

task_stack_wait_for_completion :: proc "contextless" (
	stack: ^Task_Stack, continuation_handle: Task_Continuation_Handle,
	worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
) -> Task_Scheduling_Status
{
	if task_stack_continuation_state(stack, continuation_handle) == .Uninitialized
	{
		return .Invalid_Argument;
	}
	wait_iteration: u32;
	for task_stack_continuation_state(stack, continuation_handle) != .Completed
	{
		status := task_stack_try_pop_and_run(stack, worker_index, dispatcher);
		if status == .Stop
		{
			return .Stop;
		}
		if status == .Success
		{
			wait_iteration = 0;
		}
		else if status == .Empty
		{
			task_stack_wait_once(&wait_iteration);
		}
	}
	return .Ok;
}

task_stack_request_stop :: proc "contextless" (stack: ^Task_Stack)
{
	if stack == nil
	{
		return;
	}
	sync.atomic_store_explicit(&stack.stop.value, u32(Task_Stop_State.Requested), .Release);
}

task_stack_dispatch_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
)
{
	stack := (^Task_Stack)(dispatcher.unmanaged_context);
	wait_iteration: u32;
	for
	{
		status := task_stack_try_pop_and_run(stack, worker_index, dispatcher);
		if status == .Stop
		{
			return;
		}
		if status == .Success
		{
			wait_iteration = 0;
		}
		else if status == .Empty
		{
			task_stack_wait_once(&wait_iteration);
		}
	}
}

task_stack_drain_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary,
)
{
	stack := (^Task_Stack)(dispatcher.unmanaged_context);
	for
	{
		status := task_stack_try_pop_and_run(stack, worker_index, dispatcher);
		if status != .Success
		{
			return;
		}
	}
}

task_stack_dispose :: proc (
	stack: ^Task_Stack, pool: ^Buffer_Pool,
) -> Task_Scheduling_Status
{
	if stack == nil || pool == nil || stack.state != .Ready
	{
		return .Disposed;
	}
	if stack.epoch != .Inactive
	{
		return .Invalid_Argument;
	}
	task_stack_return_initialization_buffer(stack, pool);
	stack^ = {};
	stack.epoch = .Inactive;
	stack.state = .Disposed;
	return .Ok;
}
