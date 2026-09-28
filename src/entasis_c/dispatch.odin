package entasis_c

import shared "entasis:entasis_c_shared"
import "base:runtime"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

ENTASIS_THREAD_POOL_MAGIC :: u64(0x454E545448524431);
ENTASIS_DISPATCH_OK               :: Entasis_Dispatcher_Status(0);
ENTASIS_DISPATCH_INVALID_ARGUMENT :: Entasis_Dispatcher_Status(1);
ENTASIS_DISPATCH_CAPACITY_MISSING :: Entasis_Dispatcher_Status(2);
ENTASIS_DISPATCH_BUSY             :: Entasis_Dispatcher_Status(3);
ENTASIS_DISPATCH_SHUTTING_DOWN    :: Entasis_Dispatcher_Status(4);
ENTASIS_DISPATCH_DISPOSED         :: Entasis_Dispatcher_Status(5);
abi_thread_pool_resource :: struct
{
	magic:             u64,
	allocator:         shared.Allocator,
	pool:              entasis.Thread_Pool,
	attached_worlds:   i32,
	active_dispatches: i32,
	worker_count:      i32,
	reserved:          i32,
	borrowed_headers:  [entasis.MAXIMUM_WORKER_COUNT]abi_buffer_pool_header,
	borrowed_handles:  [entasis.MAXIMUM_WORKER_COUNT]Entasis_Buffer_Pool,
}

abi_external_dispatch_bridge :: struct
{
	interface:     Entasis_Dispatcher_Interface,
	boundary:      entasis.Dispatcher,
	active_worker: util.Dispatcher_Worker_Proc,
	active_count:  int,
	active:        bool,
}

abi_thread_pool_get :: #force_inline proc "contextless" (
	handle: ^Entasis_Thread_Pool,
) -> ^abi_thread_pool_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_thread_pool_resource)(handle.opaque);
	if resource.magic != ENTASIS_THREAD_POOL_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_dispatch_to_threading :: #force_inline proc "contextless" (
	status: Entasis_Dispatcher_Status,
) -> util.Threading_Status
{
	switch status
	{
		case ENTASIS_DISPATCH_OK:
		return .Ok;
		case ENTASIS_DISPATCH_CAPACITY_MISSING:
		return .Capacity_Missing;
		case ENTASIS_DISPATCH_BUSY:
		return .Already_Running;
		case ENTASIS_DISPATCH_SHUTTING_DOWN:
		return .Shutting_Down;
		case ENTASIS_DISPATCH_DISPOSED:
		return .Disposed;
		case ENTASIS_DISPATCH_INVALID_ARGUMENT:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

abi_dispatch_from_threading :: #force_inline proc "contextless" (
	status: util.Threading_Status,
) -> Entasis_Dispatcher_Status
{
	switch status
	{
		case .Ok:
		return ENTASIS_DISPATCH_OK;
		case .Capacity_Missing:
		return ENTASIS_DISPATCH_CAPACITY_MISSING;
		case .Already_Running:
		return ENTASIS_DISPATCH_BUSY;
		case .Shutting_Down:
		return ENTASIS_DISPATCH_SHUTTING_DOWN;
		case .Disposed:
		return ENTASIS_DISPATCH_DISPOSED;
		case .Invalid_Argument, .Not_Running:
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	return ENTASIS_DISPATCH_INVALID_ARGUMENT;
}

abi_dispatcher_interface_validate :: proc "contextless" (
	interface: ^Entasis_Dispatcher_Interface,
) -> Entasis_Dispatcher_Status
{
	if interface == nil ||
	interface.struct_size < u32(size_of(Entasis_Dispatcher_Interface)) ||
	interface.struct_version != ENTASIS_STRUCT_VERSION ||
	interface.worker_count == 0 ||
	interface.worker_count > u32(entasis.MAXIMUM_WORKER_COUNT) ||
	interface.worker_pool == nil ||
	(interface.worker_count > 1 && interface.dispatch == nil)
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	return ENTASIS_DISPATCH_OK;
}

abi_external_worker :: proc "c" (worker_index: u32, work_context: rawptr)
{
	if work_context == nil
	{
		return;
	}
	bridge := (^abi_external_dispatch_bridge)(work_context);
	if !bridge.active || bridge.active_worker == nil ||
	worker_index >= u32(bridge.active_count)
	{
		return;
	}
	bridge.active_worker(int(worker_index), &bridge.boundary);
}

abi_external_boundary_worker_pool :: proc (
	boundary: ^util.Thread_Dispatcher_Boundary,
	worker_index: int,
) -> (^util.Buffer_Pool, util.Threading_Status)
{
	context = runtime.default_context();
	if boundary == nil || boundary.dispatcher == nil || worker_index < 0
	{
		return nil, .Invalid_Argument;
	}
	bridge := (^abi_external_dispatch_bridge)(boundary.dispatcher);
	if u32(worker_index) >= bridge.interface.worker_count
	{
		return nil, .Invalid_Argument;
	}
	handle: Entasis_Buffer_Pool;
	status := bridge.interface.worker_pool(
		bridge.interface.user_context, u32(worker_index), &handle,
	);
	if status != ENTASIS_DISPATCH_OK
	{
		return nil, abi_dispatch_to_threading(status);
	}
	header := abi_buffer_pool_header_get(&handle);
	if header == nil
	{
		return nil, .Invalid_Argument;
	}
	return header.pool, .Ok;
}

abi_external_boundary_dispatch :: proc "contextless" (
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
	bridge := (^abi_external_dispatch_bridge)(boundary.dispatcher);
	if bridge.active
	{
		return .Already_Running;
	}
	validation := abi_dispatcher_interface_validate(&bridge.interface);
	if validation != ENTASIS_DISPATCH_OK
	{
		return abi_dispatch_to_threading(validation);
	}
	worker_count := min(maximum_worker_count, int(bridge.interface.worker_count));
	if worker_count <= 0
	{
		return .Invalid_Argument;
	}
	boundary.unmanaged_context = unmanaged_context;
	bridge.active_worker = worker;
	bridge.active_count = worker_count;
	bridge.active = true;
	status := ENTASIS_DISPATCH_OK;
	if worker_count == 1
	{
		worker(0, boundary);
	}
	else
	{
		status = bridge.interface.dispatch(
			bridge.interface.user_context,
			abi_external_worker,
			bridge,
			u32(worker_count),
		);
	}
	bridge.active = false;
	bridge.active_worker = nil;
	bridge.active_count = 0;
	boundary.unmanaged_context = nil;
	return abi_dispatch_to_threading(status);
}

abi_external_bridge_init :: #force_inline proc "contextless" (
	bridge: ^abi_external_dispatch_bridge,
	interface: ^Entasis_Dispatcher_Interface,
) -> Entasis_Dispatcher_Status
{
	status := abi_dispatcher_interface_validate(interface);
	if status != ENTASIS_DISPATCH_OK
	{
		return status;
	}
	bridge^ = {};
	bridge.interface = interface^;
	bridge.boundary = {
		dispatcher=bridge,
		dispatch=abi_external_boundary_dispatch,
		worker_pool=abi_external_boundary_worker_pool,
		worker_count=int(interface.worker_count),
	};
	return ENTASIS_DISPATCH_OK;
}

abi_thread_pool_work_context :: struct
{
	work:         Entasis_Dispatch_Work_Proc,
	work_context: rawptr,
}

abi_thread_pool_core_worker :: proc "contextless" (
	worker_index: int,
	work_context: rawptr,
)
{
	if work_context == nil || worker_index < 0
	{
		return;
	}
	ctx := (^abi_thread_pool_work_context)(work_context);
	if ctx.work != nil
	{
		ctx.work(u32(worker_index), ctx.work_context);
	}
}

abi_thread_pool_dispatch :: proc "c" (
	user_context: rawptr,
	work: Entasis_Dispatch_Work_Proc,
	work_context: rawptr,
	worker_count: u32,
) -> Entasis_Dispatcher_Status
{
	if user_context == nil || work == nil || worker_count == 0
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	resource := (^abi_thread_pool_resource)(user_context);
	if resource.magic != ENTASIS_THREAD_POOL_MAGIC
	{
		return ENTASIS_DISPATCH_DISPOSED;
	}
	if worker_count > u32(resource.worker_count)
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	if resource.active_dispatches != 0
	{
		return ENTASIS_DISPATCH_BUSY;
	}
	context = runtime.default_context();
	interface := entasis.dispatcher_from_thread_pool(&resource.pool);
	if interface.dispatch == nil
	{
		return ENTASIS_DISPATCH_DISPOSED;
	}
	ctx := abi_thread_pool_work_context{work=work, work_context=work_context};
	resource.active_dispatches += 1;
	status := interface.dispatch(
		interface.user_context,
		abi_thread_pool_core_worker,
		&ctx,
		int(worker_count),
	);
	resource.active_dispatches -= 1;
	return abi_dispatch_status(status);
}

abi_thread_pool_worker_pool :: proc "c" (
	user_context: rawptr,
	worker_index: u32,
	out_pool: ^Entasis_Buffer_Pool,
) -> Entasis_Dispatcher_Status
{
	if user_context == nil || out_pool == nil
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	out_pool^ = {};
	resource := (^abi_thread_pool_resource)(user_context);
	if resource.magic != ENTASIS_THREAD_POOL_MAGIC
	{
		return ENTASIS_DISPATCH_DISPOSED;
	}
	if worker_index >= u32(resource.worker_count)
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	context = runtime.default_context();
	interface := entasis.dispatcher_from_thread_pool(&resource.pool);
	low_pool, status := interface.worker_pool(interface.user_context, int(worker_index));
	if status != .Ok || low_pool == nil
	{
		return abi_dispatch_status(status);
	}
	header := &resource.borrowed_headers[worker_index];
	header^ = {
		magic=ENTASIS_POOL_MAGIC,
		pool=low_pool,
		owner=resource,
		borrowed=true,
	};
	handle := &resource.borrowed_handles[worker_index];
	handle.opaque = header;
	out_pool^ = handle^;
	return ENTASIS_DISPATCH_OK;
}

abi_dispatcher_from_thread_pool :: proc "contextless" (
	pool: ^Entasis_Thread_Pool,
	out_interface: ^Entasis_Dispatcher_Interface,
) -> Entasis_Dispatcher_Status
{
	if out_interface == nil
	{
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	out_interface^ = {};
	resource := abi_thread_pool_get(pool);
	if resource == nil
	{
		return ENTASIS_DISPATCH_DISPOSED;
	}
	out_interface^ = {
		struct_size=u32(size_of(Entasis_Dispatcher_Interface)),
		struct_version=ENTASIS_STRUCT_VERSION,
		user_context=resource,
		worker_count=u32(resource.worker_count),
		dispatch=abi_thread_pool_dispatch,
		worker_pool=abi_thread_pool_worker_pool,
	};
	return ENTASIS_DISPATCH_OK;
}

abi_thread_pool_from_interface :: #force_inline proc "contextless" (
	interface: ^Entasis_Dispatcher_Interface,
) -> ^abi_thread_pool_resource
{
	if interface == nil || interface.user_context == nil ||
	interface.dispatch != abi_thread_pool_dispatch ||
	interface.worker_pool != abi_thread_pool_worker_pool
	{
		return nil;
	}
	resource := (^abi_thread_pool_resource)(interface.user_context);
	if resource.magic != ENTASIS_THREAD_POOL_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_thread_pool_init :: proc "contextless" (
	pool: ^Entasis_Thread_Pool,
	worker_count, worker_pool_block_size: u32,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Dispatcher_Status
{
	return abi_thread_pool_init_extended(pool, worker_count, worker_pool_block_size, allocator, 0, diagnostic);
}

abi_thread_pool_init_extended :: proc "contextless" (
	pool: ^Entasis_Thread_Pool,
	worker_count, worker_pool_block_size: u32,
	allocator: ^Entasis_Allocator,
	allocation_scope: Entasis_Allocation_Scope,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Dispatcher_Status
{
	if pool == nil || pool.opaque != nil || worker_count == 0 ||
	worker_count > u32(entasis.MAXIMUM_WORKER_COUNT) || worker_pool_block_size == 0 || allocation_scope > 1
	{
		abi_diagnostic_record(
			diagnostic, abi_status(.Invalid_Argument),
			Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Dispatcher_Initialize), 0,
		);
		return ENTASIS_DISPATCH_INVALID_ARGUMENT;
	}
	memory, allocator_copy, allocation_status := abi_resource_allocate(
		size_of(abi_thread_pool_resource), align_of(abi_thread_pool_resource), allocator, entasis.Allocation_Scope(allocation_scope),
	);
	if allocation_status != .Ok
	{
		abi_diagnostic_record(
			diagnostic, abi_status(allocation_status),
			Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Dispatcher_Initialize), 0,
		);
		return ENTASIS_DISPATCH_CAPACITY_MISSING;
	}
	resource := (^abi_thread_pool_resource)(memory);
	resource.magic = ENTASIS_THREAD_POOL_MAGIC;
	resource.allocator = allocator_copy;
	resource.worker_count = i32(worker_count);
	context = runtime.default_context();
	status := entasis.Dispatcher_Status.Ok;
	if allocation_scope == 1
	{
		status = entasis.thread_pool_init_with_allocator(&resource.pool, int(worker_count),
			shared.Allocator_To_Core(&resource.allocator), int(worker_pool_block_size));
	}
	else
	{
		status = entasis.thread_pool_init(&resource.pool, int(worker_count), int(worker_pool_block_size));
	}
	if status != .Ok
	{
		allocator_copy = resource.allocator;
		resource.magic = 0;
		abi_resource_free(
			resource, size_of(abi_thread_pool_resource), align_of(abi_thread_pool_resource),
			&allocator_copy,
		);
		abi_diagnostic_record(
			diagnostic, abi_status(.Capacity_Missing),
			Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Dispatcher_Initialize), i32(status),
		);
		return abi_dispatch_status(status);
	}
	pool.opaque = resource;
	abi_diagnostic_clear(diagnostic);
	return ENTASIS_DISPATCH_OK;
}

abi_thread_pool_destroy :: proc "contextless" (
	pool: ^Entasis_Thread_Pool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Dispatcher_Status
{
	resource := abi_thread_pool_get(pool);
	if resource == nil
	{
		abi_diagnostic_record(
			diagnostic, abi_status(.Disposed),
			Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Dispatcher_Initialize), 0,
		);
		return ENTASIS_DISPATCH_DISPOSED;
	}
	if resource.attached_worlds != 0 || resource.active_dispatches != 0
	{
		abi_diagnostic_record(
			diagnostic, abi_status(.Invalid_Argument),
			Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Dispatcher_Initialize), 0,
		);
		return ENTASIS_DISPATCH_BUSY;
	}
	for worker_index in 0 ..< resource.worker_count
	{
		if resource.borrowed_headers[worker_index].attached != 0
		{
			return ENTASIS_DISPATCH_BUSY;
		}
	}
	context = runtime.default_context();
	status := entasis.thread_pool_destroy(&resource.pool);
	if status != .Ok
	{
		return abi_dispatch_status(status);
	}
	allocator := resource.allocator;
	resource.magic = 0;
	pool.opaque = nil;
	abi_resource_free(
		resource, size_of(abi_thread_pool_resource), align_of(abi_thread_pool_resource), &allocator,
	);
	abi_diagnostic_clear(diagnostic);
	return ENTASIS_DISPATCH_OK;
}
#assert(int(entasis.Dispatcher_Status.Ok) == int(ENTASIS_DISPATCH_OK));
#assert(int(entasis.Dispatcher_Status.Disposed) == int(ENTASIS_DISPATCH_DISPOSED));
