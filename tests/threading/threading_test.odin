package simulation_threading_tests

import "core:mem"
import "base:runtime"
import "core:simd"
import "core:sync"
import "core:testing"
import "core:thread"
import "core:time"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 256), util.Memory_Status.Ok);
	for power in 0 ..= 22
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 524288, power), util.Memory_Status.Ok);
	}
}

pool_block_count :: proc "contextless" (pool: ^util.Buffer_Pool) -> int
{
	count := 0;
	for power in 0 ..< util.BUFFER_POOL_POWER_COUNT
	{
		count += pool.pools[power].block_count;
	}
	return count;
}

worker_pool_block_count :: proc "contextless" (dispatcher: ^util.Thread_Dispatcher) -> int
{
	count := 0;
	for worker_index in 0 ..< dispatcher.worker_pools.worker_count
	{
		count += pool_block_count(&dispatcher.worker_pools.pools[worker_index]);
	}
	return count;
}

pool_outstanding_slot_count :: proc "contextless" (pool: ^util.Buffer_Pool) -> int
{
	count := 0;
	for power in 0 ..< util.BUFFER_POOL_POWER_COUNT
	{
		power_pool := &pool.pools[power];
		count += power_pool.next_slot - power_pool.free_count;
	}
	return count;
}

worker_pool_outstanding_slot_count :: proc "contextless" (
	dispatcher: ^util.Thread_Dispatcher,
) -> int
{
	count := 0;
	for worker_index in 0 ..< dispatcher.worker_pools.worker_count
	{
		count += pool_outstanding_slot_count(
			&dispatcher.worker_pools.pools[worker_index],
		);
	}
	return count;
}

tasking_probe :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_, _, _, _, _, _ = task_id, task_context, range_start, range_end, worker_index, dispatcher;
}

tasking_stop :: proc "contextless" (
	task_id: i64, task_context: rawptr, range_start, range_end: i64,
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_, _, _, _, _ = task_id, range_start, range_end, worker_index, dispatcher;
	util.task_stack_request_stop((^util.Task_Stack)(task_context));
}

Parallel_Callback_State :: struct
{
	overlap_calls:     [64]i32,
	integration_calls: [64]i32,
	payload_invalid:   [64]i32,
}

Fast_Dispatch_Session_State :: struct
{
	phase_calls:             [5][24]i32,
	phase_statuses:          [5]util.Threading_Status,
	recursive_owner_status:  util.Threading_Status,
	recursive_worker_status: util.Threading_Status,
	recursive_session_status: util.Threading_Status,
	live_shutdown_status:    util.Threading_Status,
	generation_limit_status: util.Threading_Status,
	context_clear_failures:  i32,
	unexpected_calls:        i32,
}

Fast_Dispatch_Session_Phase :: struct
{
	state:       ^Fast_Dispatch_Session_State,
	phase_index: int,
}

Fast_Dispatch_Race_State :: struct
{
	boundary:      ^util.Thread_Dispatcher_Boundary,
	entered:       sync.Futex,
	release:       sync.Futex,
	status:        util.Threading_Status,
	worker_calls:  i32,
}

fast_dispatch_wait_for_futex :: proc "contextless" (value: ^sync.Futex, expected: sync.Futex)
{
	for
	{
		observed := sync.atomic_load_explicit(value, .Acquire);
		if observed == expected
		{
			return;
		}
		sync.futex_wait(value, u32(observed));
	}
}

fast_dispatch_release_race :: proc "contextless" (state: ^Fast_Dispatch_Race_State)
{
	sync.atomic_store_explicit(&state.release, sync.Futex(1), .Release);
	sync.futex_broadcast(&state.release);
}

fast_dispatch_blocking_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = worker_index;
	state := (^Fast_Dispatch_Race_State)(dispatcher.unmanaged_context);
	sync.atomic_add_explicit(&state.worker_calls, i32(1), .Relaxed);
	sync.atomic_store_explicit(&state.entered, sync.Futex(1), .Release);
	sync.futex_broadcast(&state.entered);
	fast_dispatch_wait_for_futex(&state.release, sync.Futex(1));
}

fast_dispatch_ordinary_contender :: proc(host_thread: ^thread.Thread)
{
	context = runtime.default_context();
	state := (^Fast_Dispatch_Race_State)(host_thread.data);
	state.status = state.boundary.dispatch(
		state.boundary, fast_dispatch_blocking_worker, 1, state,
	);
}

fast_dispatch_session_contender :: proc(host_thread: ^thread.Thread)
{
	context = runtime.default_context();
	state := (^Fast_Dispatch_Race_State)(host_thread.data);
	state.status = util.thread_dispatcher_session_entry(
		state.boundary, fast_dispatch_blocking_worker,
		state.boundary.worker_count, state,
	);
}

fast_dispatch_session_unexpected_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = worker_index;
	if dispatcher != nil && dispatcher.unmanaged_context != nil
	{
		state := (^Fast_Dispatch_Session_State)(dispatcher.unmanaged_context);
		sync.atomic_add_explicit(&state.unexpected_calls, i32(1), .Relaxed);
	}
}

fast_dispatch_session_phase_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	if dispatcher == nil || dispatcher.unmanaged_context == nil
	{
		return;
	}
	phase := (^Fast_Dispatch_Session_Phase)(dispatcher.unmanaged_context);
	if phase.phase_index < 0 || phase.phase_index >= len(phase.state.phase_calls) ||
		worker_index < 0 || worker_index >= len(phase.state.phase_calls[0])
	{
		return;
	}
	phase.state.phase_calls[phase.phase_index][worker_index] += 1;
	if phase.phase_index == 1 && worker_index == 0
	{
		phase.state.recursive_owner_status = dispatcher.dispatch(
			dispatcher, fast_dispatch_session_unexpected_worker, 1, phase.state,
		);
	}
	if phase.phase_index == 2 && worker_index == 1
	{
		phase.state.recursive_worker_status = dispatcher.dispatch(
			dispatcher, fast_dispatch_session_unexpected_worker, 1, phase.state,
		);
	}
}

fast_dispatch_session_outer_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	if worker_index != 0 || dispatcher == nil || dispatcher.unmanaged_context == nil
	{
		return;
	}
	state := (^Fast_Dispatch_Session_State)(dispatcher.unmanaged_context);
	dispatcher.unmanaged_context = nil;
	included := (^util.Thread_Dispatcher)(dispatcher.dispatcher);
	context = runtime.default_context();
	state.recursive_session_status = util.thread_dispatcher_session_entry(
		dispatcher, fast_dispatch_session_unexpected_worker,
		dispatcher.worker_count, state,
	);
	state.live_shutdown_status = util.thread_dispatcher_shutdown(included);
	session := util.thread_dispatcher_session_pointer(included);
	if session != nil
	{
		sync.futex_broadcast(&session.publication);
	}
	participant_counts := [5]int{1, 2, 8, 10, 24};
	for phase_index in 0 ..< len(participant_counts)
	{
		phase := Fast_Dispatch_Session_Phase{state, phase_index};
		state.phase_statuses[phase_index] = dispatcher.dispatch(
			dispatcher, fast_dispatch_session_phase_worker,
			participant_counts[phase_index], &phase,
		);
		if dispatcher.unmanaged_context != nil
		{
			state.context_clear_failures += 1;
		}
	}
	if session != nil
	{
		limit := util.thread_dispatcher_session_publication(
			.Idle, util.THREAD_DISPATCHER_SESSION_LAST_PHASE_GENERATION, 1,
		);
		sync.atomic_store_explicit(&session.publication, sync.Futex(limit), .Release);
		state.generation_limit_status = dispatcher.dispatch(
			dispatcher, fast_dispatch_session_unexpected_worker, 1, state,
		);
	}
}

fast_dispatch_session_ordinary_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	if dispatcher == nil || dispatcher.unmanaged_context == nil
	{
		return;
	}
	counts := (^[64]i32)(dispatcher.unmanaged_context);
	if worker_index >= 0 && worker_index < len(counts^)
	{
		counts[worker_index] += 1;
	}
}

Ordinary_Dispatch_Wake_State :: struct
{
	calls:            [24]i32,
	observed_payload: [24]i32,
	payload:          i32,
	caller_thread:    int,
	caller_failures:  i32,
}

ordinary_dispatch_wake_worker :: proc "contextless" (
	worker_index: int, boundary: ^util.Thread_Dispatcher_Boundary,
)
{
	state: ^Ordinary_Dispatch_Wake_State = (^Ordinary_Dispatch_Wake_State)(boundary.unmanaged_context);
	sync.atomic_add_explicit(&state.calls[worker_index], i32(1), .Relaxed);
	sync.atomic_store_explicit(&state.observed_payload[worker_index], state.payload, .Relaxed);
	if worker_index == 0 && sync.current_thread_id() != state.caller_thread
	{
		state.caller_failures += 1;
	}
}

@(test)
ordinary_dispatch_preserves_wake_permits_context_and_shutdown :: proc(t: ^testing.T)
{
	participant_counts: [8]int = {24, 1, 10, 2, 24, 8, 16, 3};
	shutdown_idles: [2]time.Duration = {0, 2 * time.Millisecond};
	for shutdown_idle in shutdown_idles
	{
		dispatcher: util.Thread_Dispatcher;
		if !testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 24), util.Threading_Status.Ok)
		{
			return;
		}
		boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
		state: Ordinary_Dispatch_Wake_State = {caller_thread=sync.current_thread_id()};
		expected_calls, expected_payload: [24]i32;
		for iteration in 0 ..< 96
		{
			// consecutive calls and deliberate idle opportunities exercise wake reuse.
			// this does not identify an exact polling-to-park race or OS wait state
			if iteration % 16 == 15
			{
				time.sleep(2 * time.Millisecond);
			}
			participants: int = participant_counts[iteration % len(participant_counts)];
			state.payload = i32(iteration + 1);
			testing.expect_value(
				t, boundary.dispatch(boundary, ordinary_dispatch_wake_worker, participants, &state),
				util.Threading_Status.Ok,
			);
			for index in 0 ..< participants
			{
				expected_calls[index] += 1;
				expected_payload[index] = state.payload;
			}
			for index in 0 ..< len(expected_calls)
			{
				testing.expect_value(t, sync.atomic_load_explicit(&state.calls[index], .Relaxed), expected_calls[index]);
				testing.expect_value(t, sync.atomic_load_explicit(&state.observed_payload[index], .Relaxed), expected_payload[index]);
			}
			testing.expect_value(t, state.caller_failures, i32(0));
			testing.expect_value(t, boundary.unmanaged_context, rawptr(nil));
		}
		if shutdown_idle > 0
		{
			time.sleep(shutdown_idle);
		}
		testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
		for index in 0 ..< len(expected_calls)
		{
			testing.expect_value(t, state.calls[index], expected_calls[index]);
		}
	}
}

@(test)
fast_dispatch_session_preserves_phase_order_varying_participants_errors_and_reentry :: proc(t: ^testing.T)
{
	allocation_worker_counts := [4]int{1, 8, 10, 64};
	for worker_count in allocation_worker_counts
	{
		dispatcher: util.Thread_Dispatcher;
		if !testing.expect_value(
			t, util.thread_dispatcher_initialize(&dispatcher, worker_count),
			util.Threading_Status.Ok,
		)
		{
			return;
		}
		expected_allocator_calls := u64(2);
		if worker_count == 1
		{
			expected_allocator_calls = 1;
		}
		testing.expect_value(t, dispatcher.general_allocator_call_count, expected_allocator_calls);
		session := util.thread_dispatcher_session_pointer(&dispatcher);
		if worker_count < util.THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS
		{
			testing.expect_value(t, session, (^util.Thread_Dispatcher_Session)(nil));
		}
		else
		{
			testing.expect(t, session != nil);
			if session != nil
			{
				testing.expect_value(t, uintptr(session) & 127, uintptr(0));
			}
		}
		ordinary_counts: [64]i32;
		boundary := util.thread_dispatcher_boundary(&dispatcher);
		testing.expect_value(
			t, boundary.dispatch(
			boundary, fast_dispatch_session_ordinary_worker,
			worker_count, &ordinary_counts,
		),
			util.Threading_Status.Ok,
		);
		for worker_index in 0 ..< worker_count
		{
			testing.expect_value(t, ordinary_counts[worker_index], i32(1));
		}
		testing.expect_value(
			t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok,
		);
	}

	dispatcher: util.Thread_Dispatcher;
	if !testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher, 24), util.Threading_Status.Ok,
	)
	{
		return;
	}
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	general_allocations := dispatcher.general_allocator_call_count;
	state: Fast_Dispatch_Session_State;
	state.recursive_owner_status = .Ok;
	state.recursive_worker_status = .Ok;
	state.recursive_session_status = .Ok;
	state.live_shutdown_status = .Ok;
	state.generation_limit_status = .Ok;
	testing.expect_value(
		t, util.thread_dispatcher_session_entry(
		boundary, fast_dispatch_session_unexpected_worker, 23, &state,
	),
		util.Threading_Status.Invalid_Argument,
	);
	testing.expect_value(t, state.unexpected_calls, i32(0));
	copied_boundary := boundary^;
	testing.expect_value(
		t, util.thread_dispatcher_session_entry(
		&copied_boundary, fast_dispatch_session_outer_worker, 24, &state,
	),
		util.Threading_Status.Invalid_Argument,
	);
	testing.expect_value(t, state.phase_calls, [5][24]i32{});
	ordinary_race := Fast_Dispatch_Race_State{boundary=boundary, status=.Ok};
	ordinary_contender := thread.create(fast_dispatch_ordinary_contender);
	if !testing.expect(t, ordinary_contender != nil)
	{
		return;
	}
	ordinary_contender.data = &ordinary_race;
	thread.start(ordinary_contender);
	fast_dispatch_wait_for_futex(&ordinary_race.entered, sync.Futex(1));
	testing.expect_value(
		t, util.thread_dispatcher_session_entry(
		boundary, fast_dispatch_session_unexpected_worker, 24, &state,
	),
		util.Threading_Status.Already_Running,
	);
	testing.expect_value(t, state.unexpected_calls, i32(0));
	fast_dispatch_release_race(&ordinary_race);
	thread.join(ordinary_contender);
	thread.destroy(ordinary_contender);
	testing.expect_value(t, ordinary_race.status, util.Threading_Status.Ok);
	testing.expect_value(t, ordinary_race.worker_calls, i32(1));

	session_race := Fast_Dispatch_Race_State{boundary=boundary, status=.Ok};
	session_contender := thread.create(fast_dispatch_session_contender);
	if !testing.expect(t, session_contender != nil)
	{
		return;
	}
	session_contender.data = &session_race;
	thread.start(session_contender);
	fast_dispatch_wait_for_futex(&session_race.entered, sync.Futex(1));
	losing_counts: [24]i32;
	testing.expect_value(
		t, boundary.dispatch(
		boundary, fast_dispatch_session_ordinary_worker, 1, &losing_counts,
	),
		util.Threading_Status.Already_Running,
	);
	testing.expect_value(t, losing_counts, [24]i32{});
	fast_dispatch_release_race(&session_race);
	thread.join(session_contender);
	thread.destroy(session_contender);
	testing.expect_value(t, session_race.status, util.Threading_Status.Ok);
	testing.expect_value(t, session_race.worker_calls, i32(1));

	testing.expect_value(
		t, util.thread_dispatcher_session_entry(
		boundary, fast_dispatch_session_outer_worker, 24, &state,
	),
		util.Threading_Status.Ok,
	);
	participant_counts := [5]int{1, 2, 8, 10, 24};
	for phase_index in 0 ..< len(participant_counts)
	{
		testing.expect_value(t, state.phase_statuses[phase_index], util.Threading_Status.Ok);
		for worker_index in 0 ..< 24
		{
			expected := i32(0);
			if worker_index < participant_counts[phase_index]
			{
				expected = 1;
			}
			testing.expect_value(t, state.phase_calls[phase_index][worker_index], expected);
		}
	}
	testing.expect_value(t, state.recursive_owner_status, util.Threading_Status.Already_Running);
	testing.expect_value(t, state.recursive_worker_status, util.Threading_Status.Already_Running);
	testing.expect_value(t, state.recursive_session_status, util.Threading_Status.Already_Running);
	testing.expect_value(t, state.live_shutdown_status, util.Threading_Status.Already_Running);
	testing.expect_value(t, state.generation_limit_status, util.Threading_Status.Capacity_Missing);
	testing.expect_value(t, state.context_clear_failures, i32(0));
	testing.expect_value(t, state.unexpected_calls, i32(0));
	testing.expect_value(t, boundary.unmanaged_context, rawptr(nil));
	testing.expect_value(t, dispatcher.general_allocator_call_count, general_allocations);
	ordinary_counts: [24]i32;
	testing.expect_value(
		t, boundary.dispatch(boundary, fast_dispatch_session_ordinary_worker, 24, &ordinary_counts),
		util.Threading_Status.Ok,
	);
	for worker_index in 0 ..< len(ordinary_counts)
	{
		testing.expect_value(t, ordinary_counts[worker_index], i32(1));
	}
	testing.expect_value(
		t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok,
	);
}

parallel_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> physics.Physics_Status
{
	_ = simulation;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

parallel_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> physics.Physics_Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

parallel_integrate_velocity :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _ = position, orientation, inertia, velocity;
	state := (^Parallel_Callback_State)(user_context);
	if worker_index < 0 || worker_index >= len(state.integration_calls)
	{
		return;
	}
	state.integration_calls[worker_index] += 1;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		index := simd.extract(body_indices, lane);
		mask := simd.extract(integration_mask, lane);
		lane_dt := simd.extract(dt, lane);
		if (mask == 0 && index != -1) || (mask != 0 && (index < 0 || lane_dt <= 0))
		{
			state.payload_invalid[worker_index] += 1;
		}
	}
}

parallel_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}

parallel_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference, speculative_margin: ^f32,
) -> physics.Collision_Testing_State
{
	_, _, _ = a, b, speculative_margin;
	state := (^Parallel_Callback_State)(user_context);
	if worker_index >= 0 && worker_index < len(state.overlap_calls)
	{
		state.overlap_calls[worker_index] += 1;
	}
	return .Allow;
}

parallel_allow_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference, child_a, child_b: int,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b;
	return .Allow;
}

parallel_configure :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	manifold: ^physics.Manifold_Result, material: ^physics.Contact_Material_Properties,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _ = user_context, worker_index, a, b, manifold;
	material^ = {
		friction_coefficient=0.5,
		spring_settings={angular_frequency=30, twice_damping_ratio=2},
		maximum_recovery_velocity=2,
	};
	return .Allow;
}

parallel_configure_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	child_a, child_b: int, manifold: ^physics.Convex_Contact_Manifold,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b, manifold;
	return .Allow;
}

@(test)
task_stack_initialization_allocates_only_worker_metadata :: proc(t: ^testing.T)
{
	worker_counts := [2]int{1, 5};
	for worker_count in worker_counts
	{
		pool: util.Buffer_Pool;
		testing.expect_value(
			t, util.buffer_pool_initialize(&pool, 256), util.Memory_Status.Ok,
		);
		procedures := [2]util.Task_Procedure{tasking_probe, tasking_stop};
		procedure_buffer := util.Buffer(util.Task_Procedure){
			memory=&procedures[0],
			length=2,
			id=-1,
		};
		stack: util.Task_Stack;
		testing.expect_value(
			t, util.task_stack_initialize(
			&stack, worker_count, procedure_buffer, &pool,
		),
			util.Task_Scheduling_Status.Ok,
		);
		testing.expect_value(t, int(stack.workers.length), worker_count);
		testing.expect_value(t, stack.epoch, util.Task_Epoch_State.Inactive);
		testing.expect_value(t, stack.head, uintptr(0));
		for worker_index in 0 ..< worker_count
		{
			testing.expect_value(
				t, stack.workers.memory[worker_index].job_head, uintptr(0),
			);
			testing.expect_value(
				t, stack.workers.memory[worker_index].continuation_head, uintptr(0),
			);
		}
		testing.expect_value(t, pool_outstanding_slot_count(&pool), 1);
		metadata_bytes := util.buffer_pool_total_allocated_byte_count(&pool);
		metadata_byte_limit := max(
			256, worker_count * size_of(util.Task_Worker) * 2,
		);
		testing.expect(t, metadata_bytes > 0);
		testing.expect(t, metadata_bytes <= u64(metadata_byte_limit));

		dispatcher: util.Thread_Dispatcher;
		testing.expect_value(
			t, util.thread_dispatcher_initialize(&dispatcher, worker_count),
			util.Threading_Status.Ok,
		);
		boundary := util.thread_dispatcher_boundary(&dispatcher);
		initial_outstanding := worker_pool_outstanding_slot_count(&dispatcher);
		testing.expect_value(
			t, util.worker_buffer_pools_total_allocated_byte_count(
			&dispatcher.worker_pools,
		),
			u64(0),
		);
		general_allocations := dispatcher.general_allocator_call_count;
		warmed_block_count := 0;
		warmed_allocated_bytes := u64(0);
		for epoch_index in 0 ..< 2
		{
			testing.expect_value(
				t, util.task_stack_begin_epoch(&stack, boundary),
				util.Task_Scheduling_Status.Ok,
			);
			completion := util.Task_Record{
				task_context=&stack,
				procedure_index=1,
				procedure_state=.Present,
			};
			task_count := worker_count * 2 + 1;
			reservation, reserve_status := util.task_stack_reserve_batch(
				&stack, boundary, 0, task_count, u64(epoch_index),
				.Present, completion,
			);
			testing.expect_value(
				t, reserve_status, util.Task_Scheduling_Status.Ok,
			);
			for task_index in 0 ..< task_count
			{
				reservation.tasks[task_index] = {
					task_id=i64(task_index),
					procedure_state=.Present,
				};
			}
			testing.expect_value(
				t, util.task_stack_commit_batch(
				&stack, boundary, &reservation,
			),
				util.Task_Scheduling_Status.Ok,
			);
			continuation := reservation.continuation_handle;
			testing.expect_value(
				t, boundary.dispatch(
				boundary, util.task_stack_dispatch_worker,
				worker_count, &stack,
			),
				util.Threading_Status.Ok,
			);
			testing.expect_value(
				t, util.task_stack_continuation_state(&stack, continuation),
				util.Continuation_State.Completed,
			);
			testing.expect_value(
				t, util.task_stack_end_epoch(&stack, boundary),
				util.Task_Scheduling_Status.Ok,
			);
			testing.expect_value(
				t, worker_pool_outstanding_slot_count(&dispatcher),
				initial_outstanding,
			);
			if epoch_index == 0
			{
				warmed_block_count = worker_pool_block_count(&dispatcher);
				warmed_allocated_bytes =
					util.worker_buffer_pools_total_allocated_byte_count(
					&dispatcher.worker_pools,
				);
			}
			else
			{
				testing.expect_value(
					t, worker_pool_block_count(&dispatcher),
					warmed_block_count,
				);
				testing.expect_value(
					t, util.worker_buffer_pools_total_allocated_byte_count(
					&dispatcher.worker_pools,
				),
					warmed_allocated_bytes,
				);
			}
			testing.expect_value(
				t, dispatcher.general_allocator_call_count,
				general_allocations,
			);
		}
		testing.expect_value(
			t, util.thread_dispatcher_shutdown(&dispatcher),
			util.Threading_Status.Ok,
		);
		testing.expect_value(
			t, util.task_stack_dispose(&stack, &pool),
			util.Task_Scheduling_Status.Ok,
		);
		testing.expect_value(t, pool_outstanding_slot_count(&pool), 0);
		testing.expect_value(
			t, util.buffer_pool_dispose(&pool), util.Memory_Status.Ok,
		);
	}
}

@(test)
repeated_parallel_timesteps_use_only_startup_storage :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 10, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	memory, allocation_error := mem.alloc(size_of(physics.Simulation), align_of(physics.Simulation));
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return;
	}
	defer testing.expect_value(t, mem.free(memory), mem.Allocator_Error.None);
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	description := physics.simulation_create_description_default(&pool);
	description.allocation_sizes = {
		bodies=32, statics=16, inactive_body_sets=8, shapes_per_type=8,
		constraints=128, constraint_batches=8, initial_constraints_per_type_batch=16,
		minimum_constraints_per_body=8, broad_phase_candidates=256, pairs=256,
		inactive_pairs=128, pending_pairs_per_worker=128, workers=10,
	};
	description.default_pose_context.gravity = {};
	callback_state: Parallel_Callback_State;
	description.narrow_callbacks = {
		initialize=parallel_initialize,
		allow=parallel_allow,
		allow_child=parallel_allow_child,
		configure=parallel_configure,
		configure_child=parallel_configure_child,
		dispose=parallel_dispose,
		user_context=&callback_state,
	};
	description.use_default_narrow = .Missing;
	description.pose_callbacks = {
		initialize=parallel_initialize,
		prepare_for_integration=parallel_prepare,
		integrate_velocity=parallel_integrate_velocity,
		dispose=parallel_dispose,
		angular_mode=.Nonconserving,
		allow_substeps_for_unconstrained=.Enabled,
		integrate_kinematic_velocity=.Disabled,
		user_context=&callback_state,
	};
	description.use_default_pose = .Missing;
	if !testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.simulation_destroy(simulation);
	sphere := physics.Sphere{radius=0.25};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	handles: [24]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		body_description := physics.Body_Description{
			pose={orientation=util.quaternion_identity(), position={f32(index / 2) * 2, f32(index & 1) * 0.3, 0}},
			local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
			collidable={shape=shape, maximum_speculative_margin=0.1},
			activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
		};
		status: physics.Physics_Status;
		handles[index], status = physics.simulation_add_body(simulation, &body_description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	constraint_bodies := [4]physics.Body_Handle{handles[0], {}, {}, {}};
	linear_motor := physics.One_Body_Linear_Motor{settings={maximum_force=10, damping=1}};
	for index in 0 ..< 16
	{
		constraint_bodies[0] = handles[index];
		_, status := physics.simulation_add_constraint(simulation, &constraint_bodies, &linear_motor);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	sleeper_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity(), position={-1, 50, 0}},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=0.1},
		activity={sleep_threshold=1, minimum_timestep_count_under_threshold=1},
	};
	sleeper_a, sleeper_a_status := physics.simulation_add_body(simulation, &sleeper_description);
	testing.expect_value(t, sleeper_a_status, physics.Physics_Status.Ok);
	sleeper_description.pose.position = {1, 50, 0};
	sleeper_b, sleeper_b_status := physics.simulation_add_body(simulation, &sleeper_description);
	testing.expect_value(t, sleeper_b_status, physics.Physics_Status.Ok);
	sleeper_handles := [4]physics.Body_Handle{sleeper_a, sleeper_b, {}, {}};
	sleeper_constraint := physics.Ball_Socket{
		local_offset_a={1, 0, 0}, local_offset_b={-1, 0, 0},
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
	};
	for _ in 0 ..< 2
	{
		_, sleeper_constraint_status := physics.simulation_add_constraint(
			simulation, &sleeper_handles, &sleeper_constraint,
		);
		testing.expect_value(t, sleeper_constraint_status, physics.Physics_Status.Ok);
	}
	intruder_description := physics.Body_Description{
		pose={orientation=util.quaternion_identity(), position={-1, 60, 0}},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=0.1},
		activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
	};
	intruder, intruder_status := physics.simulation_add_body(simulation, &intruder_description);
	testing.expect_value(t, intruder_status, physics.Physics_Status.Ok);
	batch := &simulation.solver.active_set.batches.memory[0];
	type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[physics.ONE_BODY_LINEAR_MOTOR_TYPE_ID]];
	testing.expect_value(t, type_batch.count, i32(16));
	main_blocks := 0;
	worker_blocks := 0;
	general_allocations := u64(0);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	simulation.sleeper.tested_fraction_per_frame = 1;
	simulation.sleeper.target_slept_fraction = 1;
	simulation.sleeper.target_traversed_fraction = 1;
	// this fixture verifies allocation-free dispatch after startup checks.
	// the contact cache can otherwise grow when the intruder creates contacts.
	// reserve its existing constraint budget: at most one block per constraint,
	// thirteen coefficient vectors plus one alignment vector per block
	testing.expect_value(t, physics.physics_ensure_buffer_capacity(
		&pool, &simulation.solver.contact_coefficient_offsets,
		int(description.allocation_sizes.constraints), 0,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.physics_ensure_buffer_capacity(
		&pool, &simulation.solver.contact_coefficients,
		int(description.allocation_sizes.constraints) * 14, 0,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 120.0, boundary), physics.Physics_Status.Ok);
	for step_index in 0 ..< 16
	{
		testing.expect_value(
			t,
			physics.simulation_timestep(simulation, 1.0 / 120.0, boundary),
			physics.Physics_Status.Ok
		);
		if step_index == 0
		{
			main_blocks = pool_block_count(&pool);
			worker_blocks = worker_pool_block_count(&dispatcher);
			general_allocations = dispatcher.general_allocator_call_count;
		}
		else
		{
			testing.expect_value(t, pool_block_count(&pool), main_blocks);
			testing.expect_value(t, worker_pool_block_count(&dispatcher), worker_blocks);
			testing.expect_value(t, dispatcher.general_allocator_call_count, general_allocations);
		}
		if step_index == 0
		{
			location, resolve_status := physics.bodies_resolve(&simulation.bodies, sleeper_a);
			testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
			testing.expect(t, location.set_index > 0);
			intruder_description.pose.position = {-1, 50, 0};
			testing.expect_value(
				t, physics.simulation_apply_body_description(simulation, intruder, &intruder_description),
				physics.Physics_Status.Ok,
			);
		}
		else if step_index == 1
		{
			location, resolve_status := physics.bodies_resolve(&simulation.bodies, sleeper_a);
			testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
			testing.expect_value(t, location.set_index, i32(physics.BODIES_ACTIVE_SET_INDEX));
			intruder_description.pose.position = {-1, 60, 0};
			testing.expect_value(
				t, physics.simulation_apply_body_description(simulation, intruder, &intruder_description),
				physics.Physics_Status.Ok,
			);
			for sleeper in sleeper_handles[:2]
			{
				reset_description, reset_status := physics.bodies_get_description(&simulation.bodies, sleeper);
				testing.expect_value(t, reset_status, physics.Physics_Status.Ok);
				reset_description.velocity = {};
				testing.expect_value(
					t, physics.simulation_apply_body_description(simulation, sleeper, &reset_description),
					physics.Physics_Status.Ok,
				);
			}
		}
		else if step_index == 3
		{
			location, resolve_status := physics.bodies_resolve(&simulation.bodies, sleeper_a);
			testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
			testing.expect(t, location.set_index > 0);
		}
	}
	integration_call_count := i32(0);
	for worker_index in 0 ..< 10
	{
		testing.expect(t, callback_state.overlap_calls[worker_index] > 0);
		testing.expect_value(t, callback_state.payload_invalid[worker_index], i32(0));
		integration_call_count += callback_state.integration_calls[worker_index];
	}
	testing.expect(t, integration_call_count > 0);
}
