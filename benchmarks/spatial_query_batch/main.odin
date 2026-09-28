package spatial_query_batch

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"
import trace "../spatial_query_trace"

main :: proc()
{
	args: trace.Arguments;
	status: trace.Argument_Status;
	measured_batches := trace.MEASURED_BATCH_COUNT;
	args, status = trace.parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintfln("invalid ray benchmark arguments: %v", os.args);
		os.exit(2);
	}
	pool: entasis.Buffer_Pool;
	pool_status := entasis.buffer_pool_init(&pool, trace.POOL_MINIMUM_BLOCK_SIZE);
	if pool_status != .Ok
	{
		fmt.eprintfln("ray benchmark pool initialization failed: %v", pool_status);
		os.exit(2);
	}
	defer entasis.buffer_pool_destroy(&pool);
	owner: trace.Benchmark_Owner;
	owner_status := trace.benchmark_owner_create(&owner, &pool, args.worker_count);
	if owner_status != .Ok
	{
		fmt.eprintfln("ray benchmark world creation failed: %v", owner_status);
		os.exit(2);
	}
	defer trace.benchmark_owner_destroy(&owner);
	workers := make([]Worker, args.worker_count);
	defer delete(workers);
	job := Job{owner=&owner, workers=workers, mode=.Validate};
	hits, checksum, validation_status := execute(&job);
	if validation_status != .Ok
	{
		fmt.eprintln("scalar/batch result validation failed");
		os.exit(2);
	}
	job.mode = .Timed;
	for _ in 0 ..< trace.WARMUP_BATCH_COUNT
	{
		_, _, frame_status := execute(&job);
		if frame_status != .Ok
		{
			os.exit(2);
		}
	}
	elapsed := time.Duration(0);
	for _ in 0 ..< measured_batches
	{
		start := time.tick_now();
		_, _, frame_status := execute(&job);
		elapsed += time.tick_since(start);
		if frame_status != .Ok
		{
			os.exit(2);
		}
	}
	job.mode = .Validate;
	final_hits, final_checksum, final_status := execute(&job);
	if final_status != .Ok || hits != final_hits || checksum != final_checksum
	{
		os.exit(2);
	}
	_, err := os.stat(args.output_path, context.temp_allocator);
	file, open_error := os.open(args.output_path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil
	{
		os.exit(2);
	}
	defer os.close(file);
	if err != nil
	{
		fmt.fprintln(file, "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status");
	}
	fmt.fprintfln(file, "%d,%d,%d,%d,%d,%d,%d,%.9f,ok", args.worker_count, BATCH_SIZE,
		trace.RAY_QUERY_COUNT, trace.WARMUP_BATCH_COUNT, measured_batches, hits, checksum, f64(elapsed) / 1e6);
}
