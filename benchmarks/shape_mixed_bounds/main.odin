package shape_mixed_bounds

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"

CSV_HEADER :: "body_count,shape_type_count,invalid_bounds_count,mismatched_bounds_count,case_status,metric_status,physics_elapsed_ms";

Argument_Status :: enum u8
{
	Ok, Invalid,
}
Arguments :: struct
{
	worker_count: int, output_path: string,
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
		result = result * 10 + int(character - '0');
	}
	if result <= 0
	{
		return 0, .Invalid;
	}
	return result, .Ok;
}

worker_count_valid :: proc "contextless" (worker_count: int) -> bool
{
	return worker_count > 0 && worker_count <= entasis.MAXIMUM_WORKER_COUNT;
}

parse_arguments :: proc "contextless" (arguments: []string) -> (Arguments, Argument_Status)
{
	if len(arguments) != 3
	{
		return {}, .Invalid;
	}
	result: Arguments;
	worker_set, output_set := false, false;
	for argument in arguments[1:]
	{
		value, status := argument_value(argument, "--worker-count=");
		if status == .Ok
		{
			if worker_set
			{
				return {}, .Invalid;
			}
			result.worker_count, status = parse_positive_integer(value);
			if status != .Ok || !worker_count_valid(result.worker_count)
			{
				return {}, .Invalid;
			}
			worker_set = true;
			continue;
		}
		value, status = argument_value(argument, "--output=");
		if status == .Ok
		{
			if output_set || len(value) == 0
			{
				return {}, .Invalid;
			}
			result.output_path = value;
			output_set = true;
			continue;
		}
		return {}, .Invalid;
	}
	if !worker_set || !output_set
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

write_result :: proc(path: string, counters: Bounds_Counters, elapsed_ms: f64) -> Case_Status
{
	_, stat_error := os.stat(path, context.temp_allocator);
	write_header := stat_error != nil;
	file, open_error := os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
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
		file, "%d,%d,%d,%d,ok,ok,%.9f",
		counters.body_count, SHAPE_TYPE_COUNT, counters.invalid_bounds_count,
		counters.mismatched_bounds_count, elapsed_ms,
	) <= 0
	{
		return .Validation_Failed;
	}
	return .Ok;
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
	status := Case_Status.Ok;

	warmup: Benchmark_Simulation_Owner;
	status = benchmark_owner_create(&warmup, &pool, arguments.worker_count);
	if status != .Ok
	{
		return {status=status};
	}
	result := benchmark_owner_predict(&warmup, WARMUP_STEP_COUNT, .Warmup);
	if result.status != .Ok
	{
		_ = benchmark_owner_destroy(&warmup);
		return result;
	}
	if benchmark_owner_destroy(&warmup) != .Ok
	{
		return {status=.Release_Failed};
	}

	measured: Benchmark_Simulation_Owner;
	status = benchmark_owner_create(&measured, &pool, arguments.worker_count);
	if status != .Ok
	{
		return {status=status};
	}
	start := time.tick_now();
	result = benchmark_owner_predict(&measured, MEASURED_STEP_COUNT, .Measured);
	end := time.tick_now();
	if result.status != .Ok
	{
		_ = benchmark_owner_destroy(&measured);
		return result;
	}
	counters: Bounds_Counters;
	status = benchmark_validate_bounds(&measured, &counters);
	if status != .Ok
	{
		_ = benchmark_owner_destroy(&measured);
		return {status=status};
	}
	elapsed_ms := f64(i64(time.tick_diff(start, end))) / 1_000_000;
	if elapsed_ms <= 0
	{
		_ = benchmark_owner_destroy(&measured);
		return {status=.Validation_Failed};
	}
	if benchmark_owner_destroy(&measured) != .Ok
	{
		return {status=.Release_Failed};
	}
	return {status=write_result(arguments.output_path, counters, elapsed_ms)};
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis_shape_mixed_bounds --worker-count=<positive count up to engine maximum> --output=<path>");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v phase=%v failed_step=%d",
			result.status,
			result.physics_status,
			result.step_phase,
			result.failed_step
		);
		os.exit(2);
	}
}
