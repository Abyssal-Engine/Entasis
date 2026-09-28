package spatial_query_batch

import "base:runtime"
import "core:fmt"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import trace "../spatial_query_trace"

// public closest-hit batch queries, including timed submission and dispatch
BATCH_SIZE :: 256;

Output :: struct
{
	hit: physics.Reference_State,
	value: physics.Ray_Query_Hit,
}

Worker :: struct #align(128)
{
	queries: [BATCH_SIZE]entasis.Query,
	results: [BATCH_SIZE]entasis.Query_Result,
	output: [BATCH_SIZE]Output,
	hits: u64,
	checksum: u64,
	status: physics.Physics_Status,
}

Job :: struct
{
	owner: ^trace.Benchmark_Owner,
	workers: []Worker,
	mode: trace.Execution_Mode,
}

execute_block :: proc "contextless" (
	owner: ^trace.Benchmark_Owner, worker: ^Worker,
	begin, count: int,
) -> physics.Physics_Status
{
	for lane in 0 ..< count
	{
		worker.output[lane] = {};
	}
	for lane in 0 ..< count
	{
		worker.queries[lane] = entasis.query_ray_closest(owner.ray_inputs[begin + lane].ray);
	}
	// Not_Found is a per-query miss, not a failed frame
	_ = entasis.query_batch(&owner.world, worker.queries[:count], worker.results[:count]);
	for lane in 0 ..< count
	{
		result := &worker.results[lane];
		if result.status != .Ok && result.status != .Not_Found
		{
			return result.status;
		}
		hit := result.ray_hit;
		worker.output[lane] = {hit=.Missing, value={
			t=hit.t, location=hit.location, normal=hit.normal,
			target_id=i32(hit.collidable.packed), child_index=hit.child_index,
		}};
		if result.hit
		{
			worker.output[lane].hit = .Present;
		}
	}
	return .Ok;
}

close :: proc "contextless" (a, b: f32) -> util.Comparison_Status
{
	if math.abs(a - b) <= 0.0002 * max(f32(1), math.abs(a), math.abs(b))
	{
		return .Equal;
	}
	return .Different;
}

execute_lane :: proc "contextless" (job: ^Job, index: int, pool: ^util.Buffer_Pool)
{
	owner := job.owner;
	worker := &job.workers[index];
	worker.hits = 0;
	worker.checksum = 0;
	worker.status = .Ok;
	begin := len(owner.ray_inputs) * index / owner.worker_count;
	end := len(owner.ray_inputs) * (index + 1) / owner.worker_count;
	for block := begin; block < end; block += BATCH_SIZE
	{
		count := min(BATCH_SIZE, end - block);
		worker.status = execute_block(owner, worker, block, count);
		if worker.status != .Ok
		{
			return;
		}
		for lane in 0 ..< count
		{
			output := worker.output[lane];
			if output.hit == .Present
			{
				worker.hits += 1;
			}
			if job.mode == .Validate
			{
				if len(owner.observations.rays) > 0
				{
					owner.observations.rays[block+lane] = {state=.Hit if output.hit == .Present else .Miss,
						hit=transmute(entasis.Ray_Hit)output.value};
				}
				hit, value, status := trace.ray_execute(owner.simulation, pool, owner.ray_inputs[block + lane]);
				if status != .Ok || hit != (output.hit == .Present)
				{
					worker.status = .Invalid_Description;
					return;
				}
				token := u64(block + lane + 1) * 0x9e37_79b9_7f4a_7c15;
				if hit
				{
					if value.target_id != output.value.target_id || value.child_index != output.value.child_index ||
						close(value.t, output.value.t) != .Equal ||
						close(value.location.x, output.value.location.x) != .Equal || close(value.location.y, output.value.location.y) != .Equal || close(value.location.z, output.value.location.z) != .Equal ||
						close(value.normal.x, output.value.normal.x) != .Equal || close(value.normal.y, output.value.normal.y) != .Equal || close(value.normal.z, output.value.normal.z) != .Equal
					{
						worker.status = .Invalid_Description;
						return;
					}
					token ~= u64(u32(output.value.target_id)) << 32;
					token ~= u64(transmute(u32)output.value.t);
					token ~= 0x7261_795f_6869_7401;
				}
				else
				{
					token ~= 0x7261_795f_6d69_7373;
				}
				worker.checksum ~= trace.checksum_mix(token);
			}
		}
	}
}

worker_proc :: proc "contextless" (index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	job := (^Job)(dispatcher.unmanaged_context);
	pool, status := dispatcher.worker_pool(dispatcher, index);
	if status != .Ok
	{
		job.workers[index].status = .Invalid_Argument;
		return;
	}
	execute_lane(job, index, pool);
}

execute :: proc(job: ^Job) -> (u64, u64, trace.Case_Status)
{
	owner := job.owner;
	if owner.worker_count == 1
	{
		execute_lane(job, 0, owner.owner_pool);
	}
	else
	{
		if owner.dispatcher.dispatch(owner.dispatcher, worker_proc, owner.worker_count, job) != .Ok
		{
			return 0, 0, .Query_Failed;
		}
	}
	hits, checksum := u64(0), u64(0);
	for &worker in job.workers
	{
		if worker.status != .Ok
		{
			fmt.eprintfln("query status=%v", worker.status);
			return 0, 0, .Query_Failed;
		}
		hits += worker.hits;
		checksum ~= worker.checksum;
	}
	if hits != trace.EXPECTED_RAY_HITS
	{
		return hits, checksum, .Validation_Failed;
	}
	return hits, checksum, .Ok;
}
