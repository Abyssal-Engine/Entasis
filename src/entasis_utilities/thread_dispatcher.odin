// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:runtime"
import "core:mem"
import "core:sync"
import "core:thread"
import "core:time"

Threading_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Capacity_Missing,
	Already_Running,
	Not_Running,
	Shutting_Down,
	Disposed,
}

Dispatcher_Lifecycle :: enum u32
{
	Uninitialized,
	Ready,
	Shutting_Down,
	Disposed,
}

Dispatch_State :: enum u32
{
	Idle,
	Running,
}

Atomic_Exchange_Status :: enum u8
{
	Failed,
	Succeeded,
}

Dispatcher_Worker_Proc :: #type proc "contextless" (worker_index: int, dispatcher: ^Thread_Dispatcher_Boundary);
// a non-Ok dispatch result guarantees that no worker invocation began
Dispatcher_Dispatch_Proc :: #type proc "contextless" (
	dispatcher: ^Thread_Dispatcher_Boundary, worker: Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> Threading_Status;
Dispatcher_Worker_Pool_Proc :: #type proc (
	dispatcher: ^Thread_Dispatcher_Boundary, worker_index: int,
) -> (^Buffer_Pool, Threading_Status);
Thread_Dispatcher_Boundary :: struct
{
	dispatcher:          rawptr,
	dispatch:            Dispatcher_Dispatch_Proc,
	worker_pool:         Dispatcher_Worker_Pool_Proc,
	worker_count:        int,
	unmanaged_context:   rawptr,
}

Thread_Dispatcher_Worker :: struct
{
	thread_handle: ^thread.Thread,
	wake:          sync.Atomic_Sema,
	dispatcher:    ^Thread_Dispatcher,
	worker_index:  int,
}

Thread_Dispatcher_Counter :: struct #align(128)
{
	leading:  [128]u8,
	value:    i32,
	trailing: [124]u8,
}
#assert(size_of(Thread_Dispatcher_Counter) == 256);
#assert(offset_of(Thread_Dispatcher_Counter, value) == 128);
Thread_Dispatcher_Session_Lifecycle :: enum u32
{
	Inactive,
	Active,
	Stopping,
}

Thread_Dispatcher_Session_Publication :: enum u32
{
	Idle,
	Phase,
	Stop,
	Invalid,
}

THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS :: 10;
THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS :: 64;
THREAD_DISPATCHER_SESSION_COUNT_MASK :: u32(0x3f);
THREAD_DISPATCHER_SESSION_STATE_SHIFT :: 6;
THREAD_DISPATCHER_SESSION_GENERATION_SHIFT :: 8;
THREAD_DISPATCHER_SESSION_STOP_GENERATION :: u32(0x00ff_ffff);
THREAD_DISPATCHER_SESSION_LAST_PHASE_GENERATION :: THREAD_DISPATCHER_SESSION_STOP_GENERATION - 1;
Thread_Dispatcher_Session :: struct #align(128)
{
	publication:       sync.Futex,
	lifecycle:         u32,
	owner_thread_id:   int,
	publication_pad:   [112]u8,
	outer_worker:      Dispatcher_Worker_Proc,
	outer_context:     rawptr,
	phase_worker:      Dispatcher_Worker_Proc,
	phase_context:     rawptr,
	payload_pad:       [96]u8,
	remaining:         sync.Futex,
	remaining_pad:     [124]u8,
}
#assert(size_of(Thread_Dispatcher_Session) == 384);
#assert(align_of(Thread_Dispatcher_Session) == 128);
#assert(offset_of(Thread_Dispatcher_Session, publication) == 0);
#assert(offset_of(Thread_Dispatcher_Session, lifecycle) == 4);
#assert(offset_of(Thread_Dispatcher_Session, owner_thread_id) == 8);
#assert(offset_of(Thread_Dispatcher_Session, outer_worker) == 128);
#assert(offset_of(Thread_Dispatcher_Session, phase_worker) == 144);
#assert(offset_of(Thread_Dispatcher_Session, remaining) == 256);
Thread_Dispatcher_Session_Outer_Request :: struct
{
	worker:        Dispatcher_Worker_Proc,
	outer_context: rawptr,
}

Thread_Dispatcher :: struct
{
	boundary:           Thread_Dispatcher_Boundary,
	background_workers: [^]Thread_Dispatcher_Worker,
	background_count:   int,
	worker_pools:       Worker_Buffer_Pools,
	finished:           sync.Sema,
	worker_body:        Dispatcher_Worker_Proc,
	remaining:          Thread_Dispatcher_Counter,
	lifecycle:          u32,
	dispatch_state:     u32,
	general_allocator_call_count: u64,
	allocation_scope: Allocation_Scope,
	allocator: mem.Allocator,
}

atomic_compare_exchange_u32_status :: proc "contextless" (
	target: ^u32, expected, desired: u32,
) -> (u32, Atomic_Exchange_Status)
{
	old, boundary_success := sync.atomic_compare_exchange_strong_explicit(
		target,
		expected,
		desired,
		.Seq_Cst,
		.Seq_Cst
	);
	if boundary_success
	{
		return old, .Succeeded;
	}
	return old, .Failed;
}

thread_dispatcher_session_pointer :: proc "contextless" (
	dispatcher: ^Thread_Dispatcher,
) -> ^Thread_Dispatcher_Session
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Thread_Dispatcher phase=session reason=derived_from_combined_startup_allocation lifetime=until_dispatcher_shutdown
	if dispatcher == nil || dispatcher.background_workers == nil
	{
		return nil;
	}
	worker_count := dispatcher.background_count + 1;
	if worker_count < THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS ||
	worker_count > THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS
	{
		return nil;
	}
	worker_bytes := uintptr(size_of(Thread_Dispatcher_Worker) * dispatcher.background_count);
	session_address := (uintptr(dispatcher.background_workers) + worker_bytes + 127) & ~uintptr(127);
	return (^Thread_Dispatcher_Session)(rawptr(session_address));
}

thread_dispatcher_session_publication :: proc "contextless" (
	state: Thread_Dispatcher_Session_Publication, generation: u32,
	admitted_count: int,
) -> u32
{
	count_bits := u32(0);
	if admitted_count > 0
	{
		count_bits = u32(admitted_count - 1) & THREAD_DISPATCHER_SESSION_COUNT_MASK;
	}
	return (generation << THREAD_DISPATCHER_SESSION_GENERATION_SHIFT) |
	(u32(state) << THREAD_DISPATCHER_SESSION_STATE_SHIFT) | count_bits;
}

thread_dispatcher_session_publication_state :: proc "contextless" (
	publication: u32,
) -> Thread_Dispatcher_Session_Publication
{
	return Thread_Dispatcher_Session_Publication((publication >> THREAD_DISPATCHER_SESSION_STATE_SHIFT) & 3);
}

thread_dispatcher_session_publication_generation :: proc "contextless" (publication: u32) -> u32
{
	return publication >> THREAD_DISPATCHER_SESSION_GENERATION_SHIFT;
}

thread_dispatcher_session_publication_count :: proc "contextless" (publication: u32) -> int
{
	return int(publication & THREAD_DISPATCHER_SESSION_COUNT_MASK) + 1;
}

thread_dispatcher_session_finish_worker :: proc "contextless" (session: ^Thread_Dispatcher_Session)
{
	previous_remaining := sync.atomic_sub_explicit(
		&session.remaining, sync.Futex(1), .Acq_Rel,
	);
	if previous_remaining == 1
	{
		sync.futex_signal(&session.remaining);
	}
}

thread_dispatcher_session_execute_worker :: proc "contextless" (
	session: ^Thread_Dispatcher_Session, worker_index: int,
	boundary: ^Thread_Dispatcher_Boundary,
)
{
	worker := session.phase_worker;
	worker(worker_index, boundary);
	thread_dispatcher_session_finish_worker(session);
}

@(private)
thread_dispatcher_session_wait :: #force_inline proc "contextless" (
	publication: ^sync.Futex, expected: u32,
)
{
	// give a short phase handoff time to complete before entering the OS wait
	for _ in 0 ..< 128
	{
		if u32(sync.atomic_load_explicit(publication, .Acquire)) != expected
		{
			return;
		}
		sync.cpu_relax();
	}
	sync.futex_wait(publication, expected);
}

thread_dispatcher_session_resident_loop :: proc "contextless" (
	session: ^Thread_Dispatcher_Session, worker_index: int,
	boundary: ^Thread_Dispatcher_Boundary,
)
{
	observed_generation := u32(0);
	for
	{
		publication := u32(sync.atomic_load_explicit(&session.publication, .Acquire));
		state := thread_dispatcher_session_publication_state(publication);
		generation := thread_dispatcher_session_publication_generation(publication);
		if state == .Stop
		{
			return;
		}
		if state == .Phase && generation != observed_generation
		{
			observed_generation = generation;
			if worker_index < thread_dispatcher_session_publication_count(publication)
			{
				thread_dispatcher_session_execute_worker(session, worker_index, boundary);
			}
			continue;
		}
		thread_dispatcher_session_wait(&session.publication, publication);
	}
}

thread_dispatcher_session_publish_stop :: proc "contextless" (session: ^Thread_Dispatcher_Session)
{
	sync.atomic_store_explicit(
		&session.lifecycle, u32(Thread_Dispatcher_Session_Lifecycle.Stopping), .Release,
	);
	stop := thread_dispatcher_session_publication(
		.Stop, THREAD_DISPATCHER_SESSION_STOP_GENERATION, 1,
	);
	sync.atomic_store_explicit(&session.publication, sync.Futex(stop), .Release);
	sync.futex_broadcast(&session.publication);
}

thread_dispatcher_session_outer_worker :: proc "contextless" (
	worker_index: int, boundary: ^Thread_Dispatcher_Boundary,
)
{
	dispatcher := (^Thread_Dispatcher)(boundary.dispatcher);
	session := thread_dispatcher_session_pointer(dispatcher);
	if worker_index == 0
	{
		outer_worker := session.outer_worker;
		boundary.unmanaged_context = session.outer_context;
		outer_worker(0, boundary);
		publication := u32(sync.atomic_load_explicit(&session.publication, .Acquire));
		if thread_dispatcher_session_publication_state(publication) != .Stop
		{
			thread_dispatcher_session_publish_stop(session);
		}
		return;
	}
	thread_dispatcher_session_resident_loop(session, worker_index, boundary);
}

thread_dispatcher_dispatch_session :: proc "contextless" (
	boundary: ^Thread_Dispatcher_Boundary, worker_body: Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> Threading_Status
{
	dispatcher := (^Thread_Dispatcher)(boundary.dispatcher);
	session := thread_dispatcher_session_pointer(dispatcher);
	if session == nil || boundary != &dispatcher.boundary ||
	boundary.dispatch != thread_dispatcher_dispatch_boundary ||
	boundary.worker_pool != thread_dispatcher_worker_pool_boundary ||
	dispatcher.background_count + 1 != boundary.worker_count
	{
		return .Already_Running;
	}
	if Thread_Dispatcher_Session_Lifecycle(
		sync.atomic_load_explicit(&session.lifecycle, .Acquire),
	) != .Active ||
	sync.atomic_load_explicit(&session.owner_thread_id, .Acquire) != sync.current_thread_id()
	{
		return .Already_Running;
	}
	publication := u32(sync.atomic_load_explicit(&session.publication, .Acquire));
	if thread_dispatcher_session_publication_state(publication) != .Idle
	{
		return .Already_Running;
	}
	generation := thread_dispatcher_session_publication_generation(publication);
	if generation >= THREAD_DISPATCHER_SESSION_LAST_PHASE_GENERATION
	{
		thread_dispatcher_session_publish_stop(session);
		return .Capacity_Missing;
	}
	admitted_count := min(maximum_worker_count, boundary.worker_count);
	if admitted_count <= 0 || admitted_count > THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS
	{
		return .Invalid_Argument;
	}
	next_generation := generation + 1;
	session.phase_worker = worker_body;
	session.phase_context = unmanaged_context;
	boundary.unmanaged_context = unmanaged_context;
	sync.atomic_store_explicit(&session.remaining, sync.Futex(admitted_count), .Release);
	phase := thread_dispatcher_session_publication(.Phase, next_generation, admitted_count);
	sync.atomic_store_explicit(&session.publication, sync.Futex(phase), .Release);
	if admitted_count > 1
	{
		sync.futex_broadcast(&session.publication);
	}
	thread_dispatcher_session_execute_worker(session, 0, boundary);
	for
	{
		remaining := u32(sync.atomic_load_explicit(&session.remaining, .Acquire));
		if remaining == 0
		{
			break;
		}
		thread_dispatcher_session_wait(&session.remaining, remaining);
	}
	session.phase_worker = nil;
	session.phase_context = nil;
	boundary.unmanaged_context = nil;
	idle := thread_dispatcher_session_publication(.Idle, next_generation, 1);
	sync.atomic_store_explicit(&session.publication, sync.Futex(idle), .Release);
	return .Ok;
}

thread_dispatcher_session_entry :: proc "contextless" (
	boundary: ^Thread_Dispatcher_Boundary, outer_worker: Dispatcher_Worker_Proc,
	maximum_worker_count: int, outer_context: rawptr,
) -> Threading_Status
{
	if boundary == nil || boundary.dispatcher == nil || outer_worker == nil ||
	maximum_worker_count <= 0 || outer_context == nil
	{
		return .Invalid_Argument;
	}
	dispatcher := (^Thread_Dispatcher)(boundary.dispatcher);
	if thread_dispatcher_session_pointer(dispatcher) == nil ||
	maximum_worker_count != boundary.worker_count
	{
		return .Invalid_Argument;
	}
	request := Thread_Dispatcher_Session_Outer_Request{outer_worker, outer_context};
	return thread_dispatcher_dispatch_boundary(
		boundary, thread_dispatcher_session_outer_worker,
		maximum_worker_count, &request,
	);
}

thread_dispatcher_dispatch_thread :: proc "contextless" (dispatcher: ^Thread_Dispatcher, worker_index: int)
{
	worker_body := dispatcher.worker_body;
	if worker_body != nil
	{
		worker_body(worker_index, &dispatcher.boundary);
	}
	old_remaining := sync.atomic_sub_explicit(&dispatcher.remaining.value, i32(1), .Seq_Cst);
	if old_remaining == 0
	{
		sync.sema_post(&dispatcher.finished);
	}
}

// reuse an available permit before starting the bounded cooperative wait.
// yield between short polling groups so ready callbacks and the caller can run
THREAD_DISPATCHER_WAIT_BUDGET :: time.Duration(750 * time.Microsecond);
thread_dispatcher_wait_for_work :: proc (worker: ^Thread_Dispatcher_Worker)
{
	// a posted permit needs no timed wait or clock conversion
	initial_count: sync.Futex = sync.atomic_load_explicit(&worker.wake.count, .Relaxed);
	if initial_count != 0 && initial_count == sync.atomic_compare_exchange_strong_explicit(
		&worker.wake.count, initial_count, initial_count - 1, .Acquire, .Acquire,
	)
	{
		return;
	}
	started: time.Tick = time.tick_now();
	for
	{
		for _ in 0 ..< 16
		{
			count: sync.Futex = sync.atomic_load_explicit(&worker.wake.count, .Relaxed);
			if count != 0 && count == sync.atomic_compare_exchange_strong_explicit(
				&worker.wake.count, count, count - 1, .Acquire, .Acquire,
			)
			{
				return;
			}
			sync.cpu_relax();
		}
		if time.tick_since(started) >= THREAD_DISPATCHER_WAIT_BUDGET
		{
			break;
		}
		// let ready callbacks and the caller run while waiting for the next
		// dispatch. yield is not a completion condition
		thread.yield();
	}
	// a permit posted during yield or fallback remains available here
	sync.atomic_sema_wait(&worker.wake);
}

thread_dispatcher_worker_loop :: proc (host_thread: ^thread.Thread)
{
	worker := (^Thread_Dispatcher_Worker)(host_thread.data);
	for
	{
		thread_dispatcher_wait_for_work(worker);
		lifecycle := Dispatcher_Lifecycle(sync.atomic_load_explicit(&worker.dispatcher.lifecycle, .Acquire));
		if lifecycle != .Ready
		{
			return;
		}
		thread_dispatcher_dispatch_thread(worker.dispatcher, worker.worker_index);
	}
}

thread_dispatcher_worker_pool_boundary :: proc (
	boundary: ^Thread_Dispatcher_Boundary, worker_index: int,
) -> (^Buffer_Pool, Threading_Status)
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Thread_Dispatcher phase=dispatch reason=checked_worker_pool_reference lifetime=until_dispatcher_shutdown
	if boundary == nil || boundary.dispatcher == nil
	{
		return nil, .Invalid_Argument;
	}
	dispatcher := (^Thread_Dispatcher)(boundary.dispatcher);
	pool, status := worker_buffer_pool_get(&dispatcher.worker_pools, worker_index);
	if status != .Ok
	{
		return nil, .Invalid_Argument;
	}
	return pool, .Ok;
}

thread_dispatcher_dispatch_boundary :: proc "contextless" (
	boundary: ^Thread_Dispatcher_Boundary, worker_body: Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> Threading_Status
{
	if boundary == nil || boundary.dispatcher == nil || worker_body == nil || maximum_worker_count <= 0
	{
		return .Invalid_Argument;
	}
	dispatcher := (^Thread_Dispatcher)(boundary.dispatcher);
	if Dispatcher_Lifecycle(sync.atomic_load_explicit(&dispatcher.lifecycle, .Acquire)) != .Ready
	{
		return .Shutting_Down;
	}
	_, exchange_status := atomic_compare_exchange_u32_status(
		&dispatcher.dispatch_state,
		u32(Dispatch_State.Idle),
		u32(Dispatch_State.Running)
	);
	if exchange_status != .Succeeded
	{
		if worker_body == thread_dispatcher_session_outer_worker
		{
			return .Already_Running;
		}
		return thread_dispatcher_dispatch_session(
			boundary, worker_body, maximum_worker_count, unmanaged_context,
		);
	}
	session: ^Thread_Dispatcher_Session;
	if worker_body == thread_dispatcher_session_outer_worker
	{
		session = thread_dispatcher_session_pointer(dispatcher);
		request := (^Thread_Dispatcher_Session_Outer_Request)(unmanaged_context);
		if session == nil || request == nil || request.worker == nil || request.outer_context == nil ||
		boundary != &dispatcher.boundary || boundary.dispatch != thread_dispatcher_dispatch_boundary ||
		boundary.worker_pool != thread_dispatcher_worker_pool_boundary ||
		dispatcher.background_count + 1 != boundary.worker_count
		{
			sync.atomic_store_explicit(&dispatcher.dispatch_state, u32(Dispatch_State.Idle), .Release);
			return .Invalid_Argument;
		}
		session.outer_worker = request.worker;
		session.outer_context = request.outer_context;
		session.phase_worker = nil;
		session.phase_context = nil;
		sync.atomic_store_explicit(&session.publication, sync.Futex(0), .Relaxed);
		sync.atomic_store_explicit(&session.remaining, sync.Futex(0), .Relaxed);
		sync.atomic_store_explicit(&session.owner_thread_id, sync.current_thread_id(), .Relaxed);
		sync.atomic_store_explicit(
			&session.lifecycle, u32(Thread_Dispatcher_Session_Lifecycle.Active), .Release,
		);
	}
	worker_count := min(maximum_worker_count, boundary.worker_count);
	boundary.unmanaged_context = unmanaged_context;
	dispatcher.worker_body = worker_body;
	if worker_count == 1
	{
		worker_body(0, boundary);
	}
	else
	{
		background_to_signal := worker_count - 1;
		sync.atomic_store_explicit(&dispatcher.remaining.value, i32(background_to_signal), .Seq_Cst);
		for index in 0 ..< background_to_signal
		{
			sync.atomic_sema_post(&dispatcher.background_workers[index].wake);
		}
		thread_dispatcher_dispatch_thread(dispatcher, 0);
		sync.sema_wait(&dispatcher.finished);
	}
	dispatcher.worker_body = nil;
	boundary.unmanaged_context = nil;
	if session != nil
	{
		sync.atomic_store_explicit(
			&session.lifecycle, u32(Thread_Dispatcher_Session_Lifecycle.Inactive), .Release,
		);
		sync.atomic_store_explicit(&session.owner_thread_id, 0, .Release);
		session.outer_worker = nil;
		session.outer_context = nil;
		session.phase_worker = nil;
		session.phase_context = nil;
		sync.atomic_store_explicit(&session.remaining, sync.Futex(0), .Release);
		sync.atomic_store_explicit(&session.publication, sync.Futex(0), .Release);
	}
	sync.atomic_store_explicit(&dispatcher.dispatch_state, u32(Dispatch_State.Idle), .Release);
	return .Ok;
}

// core:thread allocates one fixed Thread record and frees it without size or
// alignment. this adapter is installed only for create. worker init_context is
// unchanged. its data is the stable dispatcher and outlives join/destroy
thread_dispatcher_record_allocator :: proc (
	allocator_data: rawptr, mode: runtime.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int,
	location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, runtime.Allocator_Error)
{
	dispatcher: ^Thread_Dispatcher = (^Thread_Dispatcher)(allocator_data);
	if dispatcher == nil
	{
		return nil, .Invalid_Argument;
	}
	allocator: mem.Allocator = dispatcher.allocator;
	switch mode
	{
		case .Alloc, .Alloc_Non_Zeroed:
		if size != size_of(thread.Thread) || alignment != align_of(thread.Thread)
		{
			return nil, .Invalid_Argument;
		}
		return allocator.procedure(allocator.data, mode, size, alignment, nil, 0, location);
		case .Free:
		if old_memory == nil
		{
			return nil, nil;
		}
		return allocator.procedure(allocator.data, .Free, 0,
			align_of(thread.Thread), old_memory, size_of(thread.Thread), location);
		case .Query_Features:
		if old_memory != nil
		{
			(cast(^runtime.Allocator_Mode_Set)old_memory)^ = {.Alloc, .Alloc_Non_Zeroed, .Free, .Query_Features};
		}
		return nil, nil;
		case .Free_All, .Resize, .Resize_Non_Zeroed, .Query_Info:
		return nil, .Mode_Not_Implemented;
	}
	return nil, .Mode_Not_Implemented;
}

thread_dispatcher_release_workers :: proc (dispatcher: ^Thread_Dispatcher)
{
	if dispatcher.background_workers == nil
	{
		return;
	}
	byte_count: int = size_of(Thread_Dispatcher_Worker) * dispatcher.background_count;
	alignment: int = int(align_of(Thread_Dispatcher_Worker));
	worker_count := dispatcher.background_count + 1;
	if worker_count >= THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS &&
	worker_count <= THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS
	{
		byte_count += 511;
		alignment = 128;
	}
	_ = allocation_free(dispatcher.background_workers, byte_count, alignment,
		dispatcher.allocator, dispatcher.allocation_scope);
	dispatcher.background_workers = nil;
}

thread_dispatcher_initialize :: proc (
	dispatcher: ^Thread_Dispatcher, worker_count: int, worker_pool_block_size: int = 16384,
) -> Threading_Status
{
	return thread_dispatcher_initialize_internal(dispatcher, worker_count,
		worker_pool_block_size, runtime.heap_allocator(), .Legacy);
}

thread_dispatcher_initialize_with_allocator :: proc (
	dispatcher: ^Thread_Dispatcher, worker_count: int, allocator: mem.Allocator,
	worker_pool_block_size: int = 16384,
) -> Threading_Status
{
	return thread_dispatcher_initialize_internal(dispatcher, worker_count,
		worker_pool_block_size, allocation_allocator(allocator), .All_Owned);
}

thread_dispatcher_initialize_internal :: proc (
	dispatcher: ^Thread_Dispatcher, worker_count, worker_pool_block_size: int,
	allocator: mem.Allocator, scope: Allocation_Scope,
) -> Threading_Status
{
	if dispatcher == nil || worker_count <= 0 || worker_count > max(int) / size_of(Thread_Dispatcher_Worker) || worker_pool_block_size <= 0
	{
		return .Invalid_Argument;
	}
	dispatcher^ = {};
	dispatcher.allocator = allocator;
	dispatcher.allocation_scope = scope;
	dispatcher.background_count = worker_count - 1;
	if dispatcher.background_count > 0
	{
		worker_record_bytes := size_of(Thread_Dispatcher_Worker) * dispatcher.background_count;
		allocation_bytes := worker_record_bytes;
		allocation_alignment := align_of(Thread_Dispatcher_Worker);
		if worker_count >= THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS &&
		worker_count <= THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS
		{
			if worker_record_bytes > max(int) - 511
			{
				return .Capacity_Missing;
			}
			allocation_bytes = worker_record_bytes + 511;
			allocation_alignment = 128;
		}
		memory, allocation_error := mem.alloc( // odin-contracts-allow: setup rule=ODIN_HOT_ALLOCATE_OR_REALLOC owner=Thread_Dispatcher phase=startup reason=persistent_worker_records
			allocation_bytes, allocation_alignment, allocator,
		);
		if allocation_error != nil || memory == nil
		{
			return .Capacity_Missing;
		}
		dispatcher.background_workers = ([^]Thread_Dispatcher_Worker)(memory);
		dispatcher.general_allocator_call_count += 1;
		for index in 0 ..< dispatcher.background_count
		{
			dispatcher.background_workers[index] = {};
		}
		session := thread_dispatcher_session_pointer(dispatcher);
		if session != nil
		{
			session^ = {};
		}
	}
	pool_status: Memory_Status = worker_buffer_pools_initialize_internal(&dispatcher.worker_pools, worker_count, worker_pool_block_size, allocator, scope);
	if pool_status != .Ok
	{
		if dispatcher.background_workers != nil
		{
			thread_dispatcher_release_workers(dispatcher); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Thread_Dispatcher phase=startup_failure reason=release_worker_records
		}
		dispatcher^ = {};
		return .Capacity_Missing;
	}
	dispatcher.general_allocator_call_count += 1;
	dispatcher.boundary = Thread_Dispatcher_Boundary{
		dispatcher=dispatcher,
		dispatch=thread_dispatcher_dispatch_boundary,
		worker_pool=thread_dispatcher_worker_pool_boundary,
		worker_count=worker_count,
	};
	sync.atomic_store_explicit(&dispatcher.lifecycle, u32(Dispatcher_Lifecycle.Ready), .Release);
	sync.atomic_store_explicit(&dispatcher.dispatch_state, u32(Dispatch_State.Idle), .Release);
	for index in 0 ..< dispatcher.background_count
	{
		worker := &dispatcher.background_workers[index];
		worker.dispatcher = dispatcher;
		worker.worker_index = index + 1;
		if scope == .All_Owned
		{
			context.allocator = {procedure=thread_dispatcher_record_allocator, data=dispatcher};
			worker.thread_handle = thread.create(thread_dispatcher_worker_loop);
		}
		else
		{
			worker.thread_handle = thread.create(thread_dispatcher_worker_loop);
		}
		if worker.thread_handle == nil
		{
			sync.atomic_store_explicit(&dispatcher.lifecycle, u32(Dispatcher_Lifecycle.Shutting_Down), .Release);
			for started in 0 ..< index
			{
				sync.atomic_sema_post(&dispatcher.background_workers[started].wake);
			}
			for started in 0 ..< index
			{
				thread.join(dispatcher.background_workers[started].thread_handle);
				thread.destroy(dispatcher.background_workers[started].thread_handle);
			}
			_ = worker_buffer_pools_dispose(&dispatcher.worker_pools);
			thread_dispatcher_release_workers(dispatcher); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Thread_Dispatcher phase=thread_start_failure reason=release_worker_records
			dispatcher^ = {};
			return .Capacity_Missing;
		}
		worker.thread_handle.data = worker;
		thread.start(worker.thread_handle);
	}
	return .Ok;
}

thread_dispatcher_boundary :: proc "contextless" (dispatcher: ^Thread_Dispatcher) -> ^Thread_Dispatcher_Boundary
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Thread_Dispatcher phase=dispatch reason=stable_explicit_dispatch_boundary lifetime=until_dispatcher_shutdown
	if dispatcher == nil
	{
		return nil;
	}
	return &dispatcher.boundary;
}

thread_dispatcher_shutdown :: proc (dispatcher: ^Thread_Dispatcher) -> Threading_Status
{
	if dispatcher == nil
	{
		return .Invalid_Argument;
	}
	lifecycle := Dispatcher_Lifecycle(sync.atomic_load_explicit(&dispatcher.lifecycle, .Acquire));
	if lifecycle == .Disposed || lifecycle == .Uninitialized
	{
		return .Disposed;
	}
	if Dispatch_State(sync.atomic_load_explicit(&dispatcher.dispatch_state, .Acquire)) != .Idle
	{
		return .Already_Running;
	}
	sync.atomic_store_explicit(&dispatcher.lifecycle, u32(Dispatcher_Lifecycle.Shutting_Down), .Release);
	for index in 0 ..< dispatcher.background_count
	{
		sync.atomic_sema_post(&dispatcher.background_workers[index].wake);
	}
	for index in 0 ..< dispatcher.background_count
	{
		thread.join(dispatcher.background_workers[index].thread_handle);
		thread.destroy(dispatcher.background_workers[index].thread_handle);
	}
	pool_status := worker_buffer_pools_dispose(&dispatcher.worker_pools);
	if dispatcher.background_workers != nil
	{
		thread_dispatcher_release_workers(dispatcher); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=Thread_Dispatcher phase=shutdown reason=release_persistent_worker_records
	}
	dispatcher.background_workers = nil;
	dispatcher.background_count = 0;
	dispatcher.boundary = {};
	sync.atomic_store_explicit(&dispatcher.lifecycle, u32(Dispatcher_Lifecycle.Disposed), .Release);
	if pool_status != .Ok
	{
		return .Invalid_Argument;
	}
	return .Ok;
}
