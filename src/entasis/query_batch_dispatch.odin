package entasis

import "base:runtime"
import "core:sync"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(private)
QUERY_BATCH_DISPATCH_SUBMISSION_SIZE :: 256;

@(private)
Query_Batch_Dispatch_State :: enum u8
{
	Serial,
	Completed,
}

@(private)
Query_Batch_Dispatch_Request :: struct
{
	world: ^World,
	data: ^world_data,
	queries: []Query,
	results: []Query_Result,
	worker_count: int,
	pools: [util.THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS]^Buffer_Pool,
	statuses: [util.THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS]Status,
	phase_status: util.Threading_Status,
}

@(private)
query_batch_dispatch_worker :: proc "contextless" (
	worker_index: int, boundary: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	request: ^Query_Batch_Dispatch_Request = cast(^Query_Batch_Dispatch_Request)boundary.unmanaged_context;
	begin: int = worker_index * len(request.queries) / request.worker_count;
	end: int = (worker_index + 1) * len(request.queries) / request.worker_count;
	overall: Status = .Ok;
	for block: int = begin; block < end; block += QUERY_BATCH_DISPATCH_SUBMISSION_SIZE
	{
		count: int = min(QUERY_BATCH_DISPATCH_SUBMISSION_SIZE, end - block);
		status: Status = query_batch_execute(
			request.world, request.data, request.queries[block:block+count],
			request.results[block:block+count], nil, request.pools[worker_index],
		);
		if overall == .Ok && status != .Ok
		{
			overall = status;
		}
	}
	request.statuses[worker_index] = overall;
}

@(private)
query_batch_dispatch_session_worker :: proc "contextless" (
	worker_index: int, boundary: ^util.Thread_Dispatcher_Boundary,
)
{
	request: ^Query_Batch_Dispatch_Request = cast(^Query_Batch_Dispatch_Request)boundary.unmanaged_context;
	request.phase_status = boundary.dispatch(
		boundary, query_batch_dispatch_worker, request.worker_count, request,
	);
}

@(private)
query_batch_try_dispatch :: proc "contextless" (
	world: ^World, data: ^world_data, queries: []Query, results: []Query_Result,
) -> (Status, Query_Batch_Dispatch_State)
{
	// the caller selects its included pool and at least one submission per worker
	dispatcher: ^util.Thread_Dispatcher = &data.owned_dispatcher;
	worker_count: int = dispatcher.boundary.worker_count;
	if util.Dispatch_State(sync.atomic_load_explicit(&dispatcher.dispatch_state, .Acquire)) != .Idle
	{
		return .Ok, .Serial;
	}
	if query_batch_ray_run_eligibility(data) != .Present
	{
		return .Ok, .Serial;
	}
	for query in queries
	{
		if query.kind != .Ray_Closest || query.ray_data.filter.allow != nil ||
		query.ray_data.filter.allow_child != nil || physics.shape_ray_valid(query.ray_data.ray) != .Ok
		{
			return .Ok, .Serial;
		}
	}
	request: Query_Batch_Dispatch_Request = {
		world=world, data=data, queries=queries, results=results, worker_count=worker_count,
	};
	context = runtime.default_context();
	for worker in 0 ..< worker_count
	{
		pool: ^Buffer_Pool;
		pool_status: util.Threading_Status;
		pool, pool_status = util.thread_dispatcher_worker_pool_boundary(&dispatcher.boundary, worker);
		if pool_status != .Ok || pool.state != .Ready
		{
			return .Ok, .Serial;
		}
		request.pools[worker] = pool;
	}
	entry_status: util.Threading_Status = util.thread_dispatcher_session_entry(
		&dispatcher.boundary, query_batch_dispatch_session_worker, worker_count, &request,
	);
	if entry_status != .Ok
	{
		// rejected entry starts no callbacks, so the ordinary path still owns all work
		return .Ok, .Serial;
	}
	if request.phase_status != .Ok
	{
		return world_threading_status(request.phase_status), .Completed;
	}
	for status in request.statuses[:worker_count]
	{
		if status != .Ok
		{
			return status, .Completed;
		}
	}
	return .Ok, .Completed;
}
