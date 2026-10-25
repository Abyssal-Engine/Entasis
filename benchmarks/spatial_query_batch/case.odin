package spatial_query_batch

import "base:runtime"
import "core:fmt"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import trace "../spatial_query_trace"

QUERY_EXECUTION :: #config(ENTASIS_QUERY_EXECUTION, "caller");
#assert(QUERY_EXECUTION == "caller" || QUERY_EXECUTION == "mailbox");
BATCH_SIZE :: 256;

Batch_Execution :: enum
{
	Caller, Mailbox,
}
SELECTED_EXECUTION :: Batch_Execution.Mailbox when QUERY_EXECUTION == "mailbox" else Batch_Execution.Caller;

Job :: struct
{
	owner: ^trace.Benchmark_Owner,
	queries: []entasis.Query,
	results: []entasis.Query_Result,
	mode: trace.Execution_Mode,
	dispatch_status: util.Threading_Status,
}

close :: proc "contextless" (a, b: f32) -> util.Comparison_Status
{
	if math.abs(a - b) <= 0.0002 * max(f32(1), math.abs(a), math.abs(b))
	{
		return .Equal;
	}
	return .Different;
}

execute_lane :: proc "contextless" (job: ^Job, index: int)
{
	owner: ^trace.Benchmark_Owner = job.owner;
	begin: int = len(job.queries) * index / owner.worker_count;
	end: int = len(job.queries) * (index + 1) / owner.worker_count;
	for block: int = begin; block < end; block += BATCH_SIZE
	{
		count: int = min(BATCH_SIZE, end - block);
		// misses remain per-item results and all submissions are attempted
		_ = entasis.query_batch(&owner.world, job.queries[block:block+count], job.results[block:block+count]);
	}
}

worker_proc :: proc "contextless" (index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	job := (^Job)(dispatcher.unmanaged_context);
	execute_lane(job, index);
}

session_outer_worker :: proc "contextless" (index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	job: ^Job = cast(^Job)dispatcher.unmanaged_context;
	job.dispatch_status = dispatcher.dispatch(dispatcher, worker_proc, job.owner.worker_count, job);
}

frame_result_status :: proc "contextless" (
	hits, checksum: u64, status: trace.Case_Status, expected_hits, expected_checksum: u64,
) -> trace.Case_Status
{
	if status != .Ok
	{
		return status;
	}
	if hits != expected_hits || checksum != expected_checksum
	{
		return .Validation_Failed;
	}
	return .Ok;
}

execute :: proc(job: ^Job, execution: Batch_Execution = SELECTED_EXECUTION) -> (u64, u64, trace.Case_Status)
{
	owner: ^trace.Benchmark_Owner = job.owner;
	if owner.worker_count == 1
	{
		execute_lane(job, 0);
	}
	else if execution == .Mailbox
	{
		job.dispatch_status = .Ok;
		outer_status: util.Threading_Status = util.thread_dispatcher_session_entry(
			owner.dispatcher, session_outer_worker, owner.worker_count, job);
		if outer_status != .Ok || job.dispatch_status != .Ok
		{
			return 0, 0, .Query_Failed;
		}
	}
	else if owner.dispatcher.dispatch(owner.dispatcher, worker_proc, owner.worker_count, job) != .Ok
	{
		return 0, 0, .Query_Failed;
	}
	// identical post-join consumption is included in both timers
	return consume_query_results(job);
}

consume_query_results :: #force_no_inline proc(job: ^Job) -> (u64, u64, trace.Case_Status)
{
	owner: ^trace.Benchmark_Owner = job.owner;
	hits, checksum: u64;
	for result, index in job.results
	{
		if result.status != .Ok && result.status != .Not_Found
		{
			fmt.eprintfln("query status=%v", result.status);
			return 0, 0, .Query_Failed;
		}
		value: physics.Ray_Query_Hit = {
			t=result.ray_hit.t, location=result.ray_hit.location, normal=result.ray_hit.normal,
			target_id=i32(result.ray_hit.collidable.packed), child_index=result.ray_hit.child_index,
		};
		if result.hit
		{
			hits += 1;
		}
		if job.mode == .Validate
		{
			if len(owner.observations.rays) > 0
			{
				owner.observations.rays[index] = {state=.Hit if result.hit else .Miss, hit=result.ray_hit};
			}
			reference_hit, reference, status := trace.ray_execute(owner.simulation, owner.owner_pool, owner.ray_inputs[index]);
			if status != .Ok || reference_hit != result.hit || (reference_hit &&
				(reference.target_id != value.target_id || reference.child_index != value.child_index ||
				close(reference.t, value.t) != .Equal ||
				close(reference.location.x, value.location.x) != .Equal || close(reference.location.y, value.location.y) != .Equal || close(reference.location.z, value.location.z) != .Equal ||
				close(reference.normal.x, value.normal.x) != .Equal || close(reference.normal.y, value.normal.y) != .Equal || close(reference.normal.z, value.normal.z) != .Equal))
			{
				return 0, 0, .Validation_Failed;
			}
		}
		token: u64 = u64(index + 1) * 0x9e37_79b9_7f4a_7c15;
		if result.hit
		{
			token ~= u64(u32(value.target_id)) << 32;
			token ~= u64(transmute(u32)value.t);
			token ~= 0x7261_795f_6869_7401;
		}
		else
		{
			token ~= 0x7261_795f_6d69_7373;
		}
		checksum ~= trace.checksum_mix(token);
	}
	if hits != trace.EXPECTED_RAY_HITS
	{
		return hits, checksum, .Validation_Failed;
	}
	return hits, checksum, .Ok;
}
