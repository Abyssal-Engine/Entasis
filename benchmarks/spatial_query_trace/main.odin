package spatial_query_trace

import "core:fmt"
import "core:math"
import "core:os"
import entasis "entasis:entasis"

CSV_HEADER :: "worker_count,static_body_count,shape_count,query_count,warmup_batches,measured_batches,ray_query_count,sphere_cast_query_count,overlap_query_count,ray_hit_count,sphere_cast_hit_count,overlap_hit_count,ray_checksum,sphere_cast_checksum,overlap_checksum,ray_elapsed_ms,sphere_cast_elapsed_ms,overlap_elapsed_ms,queries_per_second,ray_queries_per_second,sphere_cast_queries_per_second,overlap_queries_per_second,case_status,metric_status,physics_elapsed_ms";

Argument_Status :: enum u8
{
	Ok,
	Invalid,
}

Arguments :: struct
{
	worker_count: int,
	output_path:  string,
}

argument_value :: proc "contextless" (argument, prefix: string) -> (string, Argument_Status)
{
	if len(argument) <= len(prefix) || argument[:len(prefix)] != prefix
	{
		return "", .Invalid;
	}
	return argument[len(prefix):], .Ok;
}

parse_positive_integer :: proc "contextless" (value: string) -> (int, Argument_Status)
{
	if len(value) == 0
	{
		return 0, .Invalid;
	}
	result := 0;
	for character in value
	{
		if character < '0' || character > '9'
		{
			return 0, .Invalid;
		}
		digit := int(character - '0');
		if result > (max(int) - digit) / 10
		{
			return 0, .Invalid;
		}
		result = result * 10 + digit;
	}
	if result <= 0
	{
		return 0, .Invalid;
	}
	return result, .Ok;
}

parse_arguments :: proc "contextless" (arguments: []string) -> (Arguments, Argument_Status)
{
	if len(arguments) != 3
	{
		return {}, .Invalid;
	}
	result: Arguments;
	worker_seen, output_seen := false, false;
	for argument in arguments[1:]
	{
		value, status := argument_value(argument, "--worker-count=");
		if status == .Ok
		{
			if worker_seen
			{
				return {}, .Invalid;
			}
			result.worker_count, status = parse_positive_integer(value);
			if status != .Ok || result.worker_count > MAXIMUM_WORKER_COUNT
			{
				return {}, .Invalid;
			}
			worker_seen = true;
			continue;
		}
		value, status = argument_value(argument, "--output=");
		if status == .Ok
		{
			if output_seen || len(value) == 0
			{
				return {}, .Invalid;
			}
			result.output_path = value;
			output_seen = true;
			continue;
		}
		return {}, .Invalid;
	}
	if !worker_seen || !output_seen
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

finite_positive :: #force_inline proc "contextless" (value: f64) -> bool
{
	return value > 0 && !math.is_nan(value) && !math.is_inf(value);
}

write_result :: proc(
	path: string,
	worker_count: int,
	result: Benchmark_Result,
) -> Case_Status
{
	_, stat_error := os.stat(path, context.temp_allocator);
	write_header := stat_error != nil;
	file, open_error := os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil || file == nil
	{
		return .Validation_Failed;
	}
	if write_header && fmt.fprintfln(file, CSV_HEADER) <= 0
	{
		_ = os.close(file);
		return .Validation_Failed;
	}
	ray_ms := f64(result.ray_nanoseconds) / 1_000_000;
	sphere_ms := f64(result.sphere_nanoseconds) / 1_000_000;
	overlap_ms := f64(result.overlap_nanoseconds) / 1_000_000;
	physics_ms := ray_ms + sphere_ms + overlap_ms;
	measured_batches := f64(MEASURED_BATCH_COUNT);
	ray_rate := f64(RAY_QUERY_COUNT) * measured_batches * 1_000_000_000 / f64(result.ray_nanoseconds);
	sphere_rate := f64(SPHERE_CAST_QUERY_COUNT) * measured_batches * 1_000_000_000 / f64(result.sphere_nanoseconds);
	overlap_rate := f64(OVERLAP_QUERY_COUNT) * measured_batches * 1_000_000_000 / f64(result.overlap_nanoseconds);
	query_rate := f64(QUERY_COUNT_PER_BATCH) * measured_batches * 1000 / physics_ms;
	if !finite_positive(ray_ms) || !finite_positive(sphere_ms) || !finite_positive(overlap_ms) ||
		!finite_positive(physics_ms) || !finite_positive(query_rate) || !finite_positive(ray_rate) ||
		!finite_positive(sphere_rate) || !finite_positive(overlap_rate)
	{
		_ = os.close(file);
		return .Validation_Failed;
	}
	written := fmt.fprintfln(
		file,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,ok,ok,%.9f",
		worker_count,
		STATIC_BODY_COUNT,
		2,
		QUERY_COUNT_PER_BATCH,
		WARMUP_BATCH_COUNT,
		MEASURED_BATCH_COUNT,
		RAY_QUERY_COUNT,
		SPHERE_CAST_QUERY_COUNT,
		OVERLAP_QUERY_COUNT,
		result.ray_hits,
		result.sphere_cast_hits,
		result.overlap_hits,
		result.ray_checksum,
		result.sphere_cast_checksum,
		result.overlap_checksum,
		ray_ms,
		sphere_ms,
		overlap_ms,
		query_rate,
		ray_rate,
		sphere_rate,
		overlap_rate,
		physics_ms,
	);
	if written <= 0 || os.close(file) != nil
	{
		return .Validation_Failed;
	}
	return .Ok;
}

run_benchmark :: proc(arguments: Arguments) -> Benchmark_Result
{
	pool: entasis.Buffer_Pool;
	if entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE) != .Ok
	{
		return {status=.Allocation_Failed};
	}
	defer
	{
		_ = entasis.buffer_pool_destroy(&pool);
	}
	owner: Benchmark_Owner;
	owner_status := benchmark_owner_create(&owner, &pool, arguments.worker_count);
	if owner_status != .Ok
	{
		return {status=owner_status};
	}
	result := benchmark_execute(&owner);
	if benchmark_owner_destroy(&owner) != .Ok
	{
		return {status=.Release_Failed};
	}
	if result.status != .Ok
	{
		return result;
	}
	result.status = write_result(arguments.output_path, arguments.worker_count, result);
	return result;
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis_spatial_query_trace --worker-count=<positive count up to engine maximum> --output=<path>");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v",
			result.status,
			result.physics_status,
		);
		os.exit(2);
	}
	os.exit(0);
}
