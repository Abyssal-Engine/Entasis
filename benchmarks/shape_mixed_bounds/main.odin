package shape_mixed_bounds

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"

CSV_HEADER :: "body_count,shape_type_count,invalid_bounds_count,mismatched_bounds_count,case_status,metric_status,physics_elapsed_ms,bounds_clustered_ms,precision,worker_count,measured_passes,warmup_passes,benchmark_parameters";

Argument_Status :: enum u8
{
	Ok, Invalid,
}
Arguments :: struct
{
	worker_count: int, body_count: int, output_path: string,
}

Argument_Presence :: enum u8
{
	Missing,
	Present,
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
		digit: int = int(character - '0');
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
	if len(arguments) < 3 || len(arguments) > 4
	{
		return {}, .Invalid;
	}
	result: Arguments = {body_count=BODY_COUNT};
	worker_set, output_set, body_set: Argument_Presence;
	for argument in arguments[1:]
	{
		value, status := argument_value(argument, "--worker-count=");
		if status == .Ok
		{
			if worker_set == .Present
			{
				return {}, .Invalid;
			}
			result.worker_count, status = parse_positive_integer(value);
			if status != .Ok || result.worker_count <= 0 || result.worker_count > entasis.MAXIMUM_WORKER_COUNT
			{
				return {}, .Invalid;
			}
			worker_set = .Present;
			continue;
		}
		value, status = argument_value(argument, "--body-count=");
		if status == .Ok
		{
			if body_set == .Present
			{
				return {}, .Invalid;
			}
			result.body_count, status = parse_positive_integer(value);
			if status != .Ok || result.body_count < 8 || result.body_count > 1_000_000
			{
				return {}, .Invalid;
			}
			body_set = .Present;
			continue;
		}
		value, status = argument_value(argument, "--output=");
		if status == .Ok
		{
			if output_set == .Present || len(value) == 0
			{
				return {}, .Invalid;
			}
			result.output_path = value;
			output_set = .Present;
			continue;
		}
		return {}, .Invalid;
	}
	if worker_set != .Present || output_set != .Present
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

write_result :: proc(arguments: Arguments, counters: Bounds_Counters, elapsed_ms, clustered_ms: f64) -> Case_Status
{
	_, stat_error := os.stat(arguments.output_path, context.temp_allocator);
	write_header := stat_error != nil;
	file, open_error := os.open(arguments.output_path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil || file == nil
	{
		return .Validation_Failed;
	}
	defer os.close(file);
	if write_header && fmt.fprintfln(file, CSV_HEADER) <= 0
	{
		return .Validation_Failed;
	}
	if fmt.fprintfln(
		file, "%d,%d,%d,%d,ok,ok,%.9f,%.9f,%s,%d,%d,%d,\"workload=shape_mixed_bounds;region=predict-bounds;layouts=round-robin-and-clustered;bodies=%d;measured-passes=%d;warmup-passes=%d;warmup-cycle=same-owner-predict\"",
		counters.body_count, SHAPE_TYPE_COUNT, counters.invalid_bounds_count,
		counters.mismatched_bounds_count, elapsed_ms, clustered_ms, entasis.PRECISION_NAME,
		arguments.worker_count, MEASURED_STEP_COUNT, WARMUP_STEP_COUNT, counters.body_count, MEASURED_STEP_COUNT, WARMUP_STEP_COUNT,
	) <= 0
	{
		return .Validation_Failed;
	}
	return .Ok;
}

benchmark_run_layout :: proc(
	pool: ^entasis.Buffer_Pool, worker_count: int, layout: Bounds_Layout, body_count: int = BODY_COUNT,
) -> (Benchmark_Run_Result, Bounds_Counters, f64)
{
	owner: Benchmark_Simulation_Owner;
	status: Case_Status = benchmark_owner_create(&owner, pool, worker_count, layout, body_count);
	if status != .Ok
	{
		return {status=status, layout=layout}, {}, 0;
	}
	result: Benchmark_Run_Result = benchmark_owner_predict(&owner, WARMUP_STEP_COUNT, .Warmup);
	if result.status != .Ok
	{
		_ = benchmark_owner_destroy(&owner);
		result.layout = layout;
		return result, {}, 0;
	}
	start := time.tick_now();
	result = benchmark_owner_predict(&owner, MEASURED_STEP_COUNT, .Measured);
	end := time.tick_now();
	if result.status != .Ok
	{
		_ = benchmark_owner_destroy(&owner);
		result.layout = layout;
		return result, {}, 0;
	}
	counters: Bounds_Counters;
	status = benchmark_validate_bounds(&owner, &counters);
	if status != .Ok
	{
		_ = benchmark_owner_destroy(&owner);
		return {status=status, layout=layout}, counters, 0;
	}
	elapsed_ms := f64(i64(time.tick_diff(start, end))) / 1_000_000;
	if elapsed_ms <= 0
	{
		_ = benchmark_owner_destroy(&owner);
		return {status=.Validation_Failed, layout=layout}, counters, elapsed_ms;
	}
	if benchmark_owner_destroy(&owner) != .Ok
	{
		return {status=.Release_Failed, layout=layout}, counters, elapsed_ms;
	}
	return {status=.Ok, layout=layout}, counters, elapsed_ms;
}

run_benchmark :: proc(arguments: Arguments) -> Benchmark_Run_Result
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
	result, counters, elapsed_ms := benchmark_run_layout(&pool, arguments.worker_count, .Round_Robin, arguments.body_count);
	if result.status != .Ok
	{
		return result;
	}
	clustered_result, _, clustered_ms := benchmark_run_layout(&pool, arguments.worker_count, .Clustered, arguments.body_count);
	if clustered_result.status != .Ok
	{
		return clustered_result;
	}
	return {status=write_result(arguments, counters, elapsed_ms, clustered_ms)};
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis_shape_mixed_bounds --worker-count=<positive count up to engine maximum> --output=<path> [--body-count=8..1000000]");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v phase=%v failed_step=%d layout=%v",
			result.status,
			result.physics_status,
			result.step_phase,
			result.failed_step,
			result.layout,
		);
		os.exit(2);
	}
}
