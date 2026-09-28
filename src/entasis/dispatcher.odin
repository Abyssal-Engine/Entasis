package entasis

import "base:runtime"
import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Dispatcher_Status describes external worker-pool failures without conflating
// scheduler state with physics operation status
Dispatcher_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Capacity_Missing,
	Busy,
	Shutting_Down,
	Disposed,
}

// Dispatcher_Work_Proc is invoked exactly once for each worker index in the
// half-open range [0, worker_count). work_context belongs to Entasis and is valid
// only until the blocking dispatch call returns
Dispatcher_Work_Proc :: #type proc "contextless" (
	worker_index: int,
	work_context: rawptr,
);
// Dispatcher_Dispatch_Proc must synchronously invoke work once for every worker
// index in [0, worker_count), then return only after every invocation completes.
// a non-Ok return guarantees that no worker invocation began
Dispatcher_Dispatch_Proc :: #type proc "contextless" (
	user_context: rawptr,
	work: Dispatcher_Work_Proc,
	work_context: rawptr,
	worker_count: int,
) -> Dispatcher_Status;
// Dispatcher_Worker_Pool_Proc returns one stable, exclusively worker-owned
// Buffer_Pool for the requested index. the same index must always map to the
// same pool while the interface is in use
Dispatcher_Worker_Pool_Proc :: #type proc "contextless" (
	user_context: rawptr,
	worker_index: int,
) -> (^Buffer_Pool, Dispatcher_Status);
// Dispatcher_Interface is the stable engine integration boundary. the engine
// owns user_context, workers, synchronization, and worker pools. Entasis owns its
// internal phase and work-block scheduling
Dispatcher_Interface :: struct
{
	user_context: rawptr,
	worker_count: int,
	dispatch:      Dispatcher_Dispatch_Proc,
	worker_pool:   Dispatcher_Worker_Pool_Proc,
}

// Thread_Pool is the included caller-owned worker pool implementation. ordinary
// worlds can own this internally. engines may also own one and expose it through
// dispatcher_from_thread_pool
Thread_Pool :: util.Thread_Dispatcher;
@(private)
dispatcher_status_from_threading :: #force_inline proc "contextless" (
	status: util.Threading_Status,
) -> Dispatcher_Status
{
	switch status
	{
		case .Ok:
		return .Ok;
		case .Capacity_Missing:
		return .Capacity_Missing;
		case .Already_Running:
		return .Busy;
		case .Shutting_Down:
		return .Shutting_Down;
		case .Disposed:
		return .Disposed;
		case .Invalid_Argument, .Not_Running:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

@(private)
dispatcher_status_to_threading :: #force_inline proc "contextless" (
	status: Dispatcher_Status,
) -> util.Threading_Status
{
	switch status
	{
		case .Ok:
		return .Ok;
		case .Capacity_Missing:
		return .Capacity_Missing;
		case .Busy:
		return .Already_Running;
		case .Shutting_Down:
		return .Shutting_Down;
		case .Disposed:
		return .Disposed;
		case .Invalid_Argument:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

// dispatcher_interface_validate verifies the stable blocking-dispatch contract
dispatcher_interface_validate :: #force_inline proc "contextless" (
	interface: Dispatcher_Interface,
) -> Dispatcher_Status
{
	if interface.worker_count <= 0 ||
	interface.worker_count > physics.MAXIMUM_SOLVER_WORKER_COUNT ||
	interface.worker_pool == nil ||
	(interface.worker_count > 1 && interface.dispatch == nil)
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

// thread_pool_init initializes the included caller-owned worker pool
thread_pool_init :: proc (
	pool: ^Thread_Pool,
	worker_count: int,
	worker_pool_block_size: int = 16384,
) -> Dispatcher_Status
{
	return dispatcher_status_from_threading(util.thread_dispatcher_initialize(
			pool,
			worker_count,
			worker_pool_block_size,
	));
}

// thread_pool_destroy stops and releases an included caller-owned worker pool
thread_pool_destroy :: proc (pool: ^Thread_Pool) -> Dispatcher_Status
{
	return dispatcher_status_from_threading(util.thread_dispatcher_shutdown(pool));
}

@(private)
thread_pool_public_dispatch_context :: struct
{
	work:         Dispatcher_Work_Proc,
	work_context: rawptr,
}

@(private)
thread_pool_public_worker :: proc "contextless" (
	worker_index: int,
	boundary: ^util.Thread_Dispatcher_Boundary,
)
{
	if boundary == nil || boundary.unmanaged_context == nil
	{
		return;
	}
	ctx := (^thread_pool_public_dispatch_context)(boundary.unmanaged_context);
	if ctx.work != nil
	{
		ctx.work(worker_index, ctx.work_context);
	}
}

@(private)
thread_pool_public_dispatch :: proc "contextless" (
	user_context: rawptr,
	work: Dispatcher_Work_Proc,
	work_context: rawptr,
	worker_count: int,
) -> Dispatcher_Status
{
	if user_context == nil || work == nil || worker_count <= 0
	{
		return .Invalid_Argument;
	}
	pool := (^Thread_Pool)(user_context);
	boundary := util.thread_dispatcher_boundary(pool);
	if boundary == nil || boundary.dispatch == nil
	{
		return .Disposed;
	}
	ctx := thread_pool_public_dispatch_context{
		work=work,
		work_context=work_context,
	};
	return dispatcher_status_from_threading(boundary.dispatch(
			boundary,
			thread_pool_public_worker,
			worker_count,
			&ctx,
	));
}

@(private)
thread_pool_public_worker_pool :: proc "contextless" (
	user_context: rawptr,
	worker_index: int,
) -> (^Buffer_Pool, Dispatcher_Status)
{
	if user_context == nil
	{
		return nil, .Invalid_Argument;
	}
	pool := (^Thread_Pool)(user_context);
	boundary := util.thread_dispatcher_boundary(pool);
	if boundary == nil || boundary.worker_pool == nil
	{
		return nil, .Disposed;
	}
	context = runtime.default_context();
	worker_pool, status := boundary.worker_pool(boundary, worker_index);
	return worker_pool, dispatcher_status_from_threading(status);
}

// dispatcher_from_thread_pool exposes an included caller-owned Thread_Pool
// through the stable external interface
dispatcher_from_thread_pool :: #force_inline proc "contextless" (
	pool: ^Thread_Pool,
) -> Dispatcher_Interface
{
	if pool == nil
	{
		return {};
	}
	boundary := util.thread_dispatcher_boundary(pool);
	if boundary == nil
	{
		return {};
	}
	return {
		user_context=pool,
		worker_count=boundary.worker_count,
		dispatch=thread_pool_public_dispatch,
		worker_pool=thread_pool_public_worker_pool,
	};
}

@(private)
external_dispatch_bridge :: struct
{
	interface:     Dispatcher_Interface,
	boundary:      Dispatcher,
	active_worker: util.Dispatcher_Worker_Proc,
	active_count:  int,
	active:        bool,
}

@(private)
external_dispatch_worker :: proc "contextless" (
	worker_index: int,
	work_context: rawptr,
)
{
	if work_context == nil
	{
		return;
	}
	bridge := (^external_dispatch_bridge)(work_context);
	if !bridge.active || bridge.active_worker == nil ||
	worker_index < 0 || worker_index >= bridge.active_count
	{
		return;
	}
	bridge.active_worker(worker_index, &bridge.boundary);
}

@(private)
external_dispatch_boundary_worker_pool :: proc (
	boundary: ^util.Thread_Dispatcher_Boundary,
	worker_index: int,
) -> (^util.Buffer_Pool, util.Threading_Status)
{
	if boundary == nil || boundary.dispatcher == nil
	{
		return nil, .Invalid_Argument;
	}
	bridge := (^external_dispatch_bridge)(boundary.dispatcher);
	pool, status := bridge.interface.worker_pool(
		bridge.interface.user_context,
		worker_index,
	);
	return pool, dispatcher_status_to_threading(status);
}

@(private)
external_dispatch_boundary_dispatch :: proc "contextless" (
	boundary: ^util.Thread_Dispatcher_Boundary,
	worker: util.Dispatcher_Worker_Proc,
	maximum_worker_count: int,
	unmanaged_context: rawptr,
) -> util.Threading_Status
{
	if boundary == nil || boundary.dispatcher == nil || worker == nil ||
	maximum_worker_count <= 0
	{
		return .Invalid_Argument;
	}
	bridge := (^external_dispatch_bridge)(boundary.dispatcher);
	if bridge.active
	{
		return .Already_Running;
	}
	validation := dispatcher_interface_validate(bridge.interface);
	if validation != .Ok
	{
		return dispatcher_status_to_threading(validation);
	}
	worker_count := min(maximum_worker_count, bridge.interface.worker_count);
	if worker_count <= 0
	{
		return .Invalid_Argument;
	}
	boundary.unmanaged_context = unmanaged_context;
	bridge.active_worker = worker;
	bridge.active_count = worker_count;
	bridge.active = true;
	status := Dispatcher_Status.Ok;
	if worker_count == 1
	{
		worker(0, boundary);
	}
	else
	{
		status = bridge.interface.dispatch(
			bridge.interface.user_context,
			external_dispatch_worker,
			bridge,
			worker_count,
		);
	}
	bridge.active = false;
	bridge.active_worker = nil;
	bridge.active_count = 0;
	boundary.unmanaged_context = nil;
	return dispatcher_status_to_threading(status);
}

// world_step_external advances one timestep through a caller-owned blocking
// dispatcher interface. the interface and all worker pools remain caller owned.
// the interface is used only for this call and may be stack allocated
world_step_external :: proc (
	world: ^World,
	dt: f32,
	interface: Dispatcher_Interface,
) -> Status
{
	validation := dispatcher_interface_validate(interface);
	if validation != .Ok
	{
		if validation == .Capacity_Missing
		{
			return .Capacity_Missing;
		}
		if validation == .Disposed || validation == .Shutting_Down
		{
			return .Disposed;
		}
		return .Invalid_Argument;
	}
	bridge := external_dispatch_bridge{interface=interface};
	bridge.boundary = {
		dispatcher=&bridge,
		dispatch=external_dispatch_boundary_dispatch,
		worker_pool=external_dispatch_boundary_worker_pool,
		worker_count=interface.worker_count,
	};
	return world_step(world, dt, &bridge.boundary);
}

// includes worker backing storage, fixed core thread records and every worker
// pool. allocator/user data must outlive shutdown and all thread joins
thread_pool_init_with_allocator :: proc (
	pool: ^Thread_Pool, worker_count: int, allocator: mem.Allocator,
	worker_pool_block_size: int = 16384,
) -> Dispatcher_Status
{
	if worker_count <= 0 || worker_count > physics.MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	return dispatcher_status_from_threading(util.thread_dispatcher_initialize_with_allocator(
			pool, worker_count, allocator, worker_pool_block_size,
	));
}
