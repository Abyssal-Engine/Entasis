package spatial_query_batch

import "base:runtime"
import "core:testing"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"
import trace "../spatial_query_trace"

Busy_Check :: struct
{
	job: ^Job,
	status: trace.Case_Status,
}

busy_mailbox_worker :: proc "contextless" (index: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Busy_Check = cast(^Busy_Check)boundary.unmanaged_context;
	_, _, state.status = execute(state.job, .Mailbox);
}

@(test)
spatial_query_dispatch_modes_preserve_results_and_frame_checksums :: proc(t: ^testing.T)
{
	for workers in ([2]int{1, 3})
	{
		pool: entasis.Buffer_Pool;
		testing.expect_value(t, entasis.buffer_pool_init(&pool, trace.POOL_MINIMUM_BLOCK_SIZE), entasis.Status.Ok);
		defer entasis.buffer_pool_destroy(&pool);
		owner: trace.Benchmark_Owner;
		testing.expect_value(t, trace.benchmark_owner_create(&owner, &pool, workers, .Oblique), trace.Case_Status.Ok);
		defer trace.benchmark_owner_destroy(&owner);
		queries: []entasis.Query = make([]entasis.Query, trace.RAY_QUERY_COUNT);
		defer delete(queries);
		results: []entasis.Query_Result = make([]entasis.Query_Result, len(queries));
		defer delete(results);
		for &query, index in queries
		{
			query = entasis.query_ray_closest(owner.ray_inputs[index].ray);
		}
		job: Job = {owner=&owner, queries=queries, results=results, mode=.Validate};
		hits, checksum: u64;
		status: trace.Case_Status;
		hits, checksum, status = execute(&job, .Caller);
		testing.expect_value(t, status, trace.Case_Status.Ok);
		testing.expect_value(t, hits, trace.EXPECTED_RAY_HITS);
		for &result in results
		{
			result.status = .Disposed;
		}
		job.mode = .Timed;
		if workers > 1
		{
			state: Busy_Check = {job=&job};
			testing.expect_value(t, owner.dispatcher.dispatch(owner.dispatcher, busy_mailbox_worker, 1, &state), util.Threading_Status.Ok);
			testing.expect_value(t, state.status, trace.Case_Status.Query_Failed);
			testing.expect_value(t, results[0].status, entasis.Status.Disposed);
		}
		actual_hits, actual_checksum: u64;
		actual_status: trace.Case_Status;
		actual_hits, actual_checksum, actual_status = execute(&job, .Mailbox);
		testing.expect_value(t, frame_result_status(actual_hits, actual_checksum, actual_status, hits, checksum), trace.Case_Status.Ok);
		if workers == 1
		{
			testing.expect_value(t, frame_result_status(actual_hits, actual_checksum ~ 1, actual_status, hits, checksum), trace.Case_Status.Validation_Failed);
		}
		else
		{
			job.mode = .Validate;
			_, _, status = consume_query_results(&job);
			testing.expect_value(t, status, trace.Case_Status.Ok);
			job.mode = .Timed;
			queries[0] = entasis.query_ray_closest(entasis.ray({}, {}, 6));
			_, _, status = execute(&job, .Mailbox);
			testing.expect_value(t, status, trace.Case_Status.Query_Failed);
		}
	}
}
