package public_api_tests

import "core:os"
import "core:testing"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

external_dispatch_available_worker_count :: proc() -> int
{
	return clamp(os.get_processor_core_count(), 1, 64);
}

external_dispatch_test_body :: proc(index: int) -> entasis.Body_Description
{
	return entasis.body_shapeless(
		{
			inverse_inertia_tensor={xx=1, yy=1, zz=1},
			inverse_mass=1,
		},
		entasis.pose({f32(index % 8), f32(index / 8 + 1), 0}),
		entasis.velocity({f32(index % 3) * 0.1, f32(index % 5) * 0.05, 0}),
		entasis.body_activity(-1, 255),
	);
}

external_dispatch_run :: proc(
	t: ^testing.T,
	world: ^entasis.World,
	interface: ^entasis.Dispatcher_Interface,
	states: []entasis.Body_State,
) -> bool
{
	description := small_world_description();
	description.gravity = {0, -9.81, 0};
	description.capacity.bodies = i32(len(states));
	if interface == nil
	{
		description.threading.worker_count = 1;
	}
	else
	{
		description.threading.worker_count = i32(interface.worker_count);
	}
	if !testing.expect_value(t, entasis.world_init(world, description), entasis.Status.Ok)
	{
		return false;
	}
	handles := make([]entasis.Body_Handle, len(states));
	defer delete(handles);
	for index in 0 ..< len(states)
	{
		handle, status := entasis.body_add(world, external_dispatch_test_body(index));
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return false;
		}
		handles[index] = handle;
	}
	for _ in 0 ..< 12
	{
		status := entasis.Status.Ok;
		if interface == nil
		{
			status = entasis.world_step(world, 1.0 / 60.0);
		}
		else
		{
			status = entasis.world_step_external(world, 1.0 / 60.0, interface^);
		}
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return false;
		}
	}
	for index in 0 ..< len(states)
	{
		state, status := entasis.body_get(world, handles[index]);
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			return false;
		}
		states[index] = state;
	}
	return true;
}

@(test)
external_thread_pool_matches_included_dispatcher_state :: proc(t: ^testing.T)
{
	pool: entasis.Thread_Pool;
	if !testing.expect_value(
		t, entasis.thread_pool_init(
		&pool,
		external_dispatch_available_worker_count(),
		65536
	), entasis.Dispatcher_Status.Ok,
	)
	{
		return;
	}
	defer entasis.thread_pool_destroy(&pool);
	interface := entasis.dispatcher_from_thread_pool(&pool);
	if !testing.expect_value(
		t, entasis.dispatcher_interface_validate(interface), entasis.Dispatcher_Status.Ok,
	)
	{
		return;
	}

	included_world: entasis.World;
	external_world: entasis.World;
	defer entasis.world_destroy(&included_world);
	defer entasis.world_destroy(&external_world);
	included_states: [32]entasis.Body_State;
	external_states: [32]entasis.Body_State;
	if !external_dispatch_run(t, &included_world, nil, included_states[:])
	{
		return;
	}
	if !external_dispatch_run(t, &external_world, &interface, external_states[:])
	{
		return;
	}
	for index in 0 ..< len(included_states)
	{
		testing.expect_value(t, external_states[index], included_states[index]);
	}
}

Inline_Dispatch_Test_Context :: struct
{
	pools:          [10]util.Buffer_Pool,
	dispatch_calls: int,
	pool_calls:     [10]i32,
}

inline_dispatch_test_proc :: proc "contextless" (
	user_context: rawptr,
	work: entasis.Dispatcher_Work_Proc,
	work_context: rawptr,
	worker_count: int,
) -> entasis.Dispatcher_Status
{
	ctx := (^Inline_Dispatch_Test_Context)(user_context);
	ctx.dispatch_calls += 1;
	for worker_index in 0 ..< worker_count
	{
		work(worker_index, work_context);
	}
	return .Ok;
}

inline_dispatch_test_pool :: proc "contextless" (
	user_context: rawptr,
	worker_index: int,
) -> (^entasis.Buffer_Pool, entasis.Dispatcher_Status)
{
	ctx := (^Inline_Dispatch_Test_Context)(user_context);
	if worker_index < 0 || worker_index >= len(ctx.pools)
	{
		return nil, .Invalid_Argument;
	}
	ctx.pool_calls[worker_index] += 1;
	return &ctx.pools[worker_index], .Ok;
}

@(test)
one_worker_external_dispatch_bypasses_background_dispatch :: proc(t: ^testing.T)
{
	ctx: Inline_Dispatch_Test_Context;
	initialized_pool_count := 0;
	defer
	{
		for pool_index in 0 ..< initialized_pool_count
		{
			_ = util.buffer_pool_dispose(&ctx.pools[pool_index]);
		}
	}
	for pool_index in 0 ..< len(ctx.pools)
	{
		if !testing.expect_value(
			t, util.buffer_pool_initialize(&ctx.pools[pool_index], 4096, 1),
			util.Memory_Status.Ok,
		)
		{
			return;
		}
		initialized_pool_count += 1;
	}
	interface := entasis.Dispatcher_Interface{
		user_context=&ctx,
		worker_count=1,
		dispatch=inline_dispatch_test_proc,
		worker_pool=inline_dispatch_test_pool,
	};
	world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	_, add_status := entasis.body_add(&world, external_dispatch_test_body(0));
	if !testing.expect_value(t, add_status, entasis.Status.Ok)
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.world_step_external(&world, 1.0 / 60.0, interface), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect_value(t, ctx.dispatch_calls, 0);
	testing.expect(t, ctx.pool_calls[0] > 0);

	dispatch_calls_before := ctx.dispatch_calls;
	description := small_world_description();
	description.threading.worker_count = 10;
	resident_world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&resident_world, description), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&resident_world);
	_, resident_add_status := entasis.body_add(&resident_world, external_dispatch_test_body(1));
	if !testing.expect_value(t, resident_add_status, entasis.Status.Ok)
	{
		return;
	}
	interface.worker_count = 10;
	if !testing.expect_value(
		t, entasis.world_step_external(&resident_world, 1.0 / 60.0, interface), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect(t, ctx.dispatch_calls > dispatch_calls_before);
	for worker_index in 0 ..< interface.worker_count
	{
		testing.expect(t, ctx.pool_calls[worker_index] > 0);
	}
}

@(test)
dispatcher_interface_rejects_incomplete_contracts :: proc(t: ^testing.T)
{
	testing.expect_value(
		t,
		entasis.dispatcher_interface_validate({}),
		entasis.Dispatcher_Status.Invalid_Argument,
	);
	ctx: Inline_Dispatch_Test_Context;
	interface := entasis.Dispatcher_Interface{
		user_context=&ctx,
		worker_count=2,
		worker_pool=inline_dispatch_test_pool,
	};
	testing.expect_value(
		t,
		entasis.dispatcher_interface_validate(interface),
		entasis.Dispatcher_Status.Invalid_Argument,
	);
}
