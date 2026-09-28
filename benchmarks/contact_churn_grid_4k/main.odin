package contact_churn_grid_4k

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"

CSV_HEADER :: "body_count,shape_count,invalid_transform_count,below_floor_count,out_of_bounds_count,case_status,metric_status,physics_elapsed_ms";

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

worker_count_status :: proc "contextless" (worker_count: int) -> Argument_Status
{
	if worker_count > 0 && worker_count <= entasis.MAXIMUM_WORKER_COUNT
	{
		return .Ok;
	}
	return .Invalid;
}

parse_arguments :: proc "contextless" (arguments: []string) -> (Arguments, Argument_Status)
{
	if len(arguments) != 3
	{
		return {}, .Invalid;
	}
	result: Arguments;
	worker_state := Argument_Status.Invalid;
	output_state := Argument_Status.Invalid;
	for argument in arguments[1:]
	{
		value, status := argument_value(argument, "--worker-count=");
		if status == .Ok
		{
			if worker_state == .Ok
			{
				return {}, .Invalid;
			}
			result.worker_count, status = parse_positive_integer(value);
			if status != .Ok || worker_count_status(result.worker_count) != .Ok
			{
				return {}, .Invalid;
			}
			worker_state = .Ok;
			continue;
		}
		value, status = argument_value(argument, "--output=");
		if status == .Ok
		{
			if output_state == .Ok || len(value) == 0
			{
				return {}, .Invalid;
			}
			result.output_path = value;
			output_state = .Ok;
			continue;
		}
		return {}, .Invalid;
	}
	if worker_state != .Ok || output_state != .Ok
	{
		return {}, .Invalid;
	}
	return result, .Ok;
}

f64_finite_state :: proc "contextless" (value: f64) -> Argument_Status
{
	if transmute(u64)value & 0x7ff0_0000_0000_0000 != 0x7ff0_0000_0000_0000
	{
		return .Ok;
	}
	return .Invalid;
}

write_result :: proc(
	path: string, counters: Stability_Counters, elapsed_milliseconds: f64,
) -> Case_Status
{
	_, stat_error := os.stat(path, context.temp_allocator);
	header_state := Argument_Status.Invalid;
	if stat_error != nil
	{
		header_state = .Ok;
	}
	file, open_error := os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil || file == nil
	{
		return .Validation_Failed;
	}
	if header_state == .Ok
	{
		if fmt.fprintfln(file, CSV_HEADER) <= 0
		{
			_ = os.close(file);
			return .Validation_Failed;
		}
	}
	if fmt.fprintfln(
		file, "%d,%d,%d,%d,%d,ok,ok,%.9f",
		PROTOCOL_BODY_COUNT, PROTOCOL_SHAPE_COUNT,
		counters.invalid_transform_count, counters.below_floor_count,
		counters.out_of_bounds_count, elapsed_milliseconds,
	) <= 0
	{
		_ = os.close(file);
		return .Validation_Failed;
	}
	if os.close(file) != nil
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
	step_result := benchmark_owner_step_verified(&warmup, WARMUP_STEP_COUNT, .Warmup);
	if step_result.status != .Ok
	{
		_ = benchmark_owner_destroy(&warmup);
		return step_result;
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
	step_result = benchmark_owner_step(&measured, MEASURED_STEP_COUNT, .Measured);
	end := time.tick_now();
	if step_result.status != .Ok
	{
		_ = benchmark_owner_destroy(&measured);
		return step_result;
	}

	counters: Stability_Counters;
	status = benchmark_count_stability(&measured, &counters);
	if status != .Ok
	{
		_ = benchmark_owner_destroy(&measured);
		return {status=status};
	}
	elapsed_nanoseconds := i64(time.tick_diff(start, end));
	elapsed_milliseconds := f64(elapsed_nanoseconds) / 1_000_000;
	milliseconds_per_step := elapsed_milliseconds / MEASURED_STEP_COUNT;
	steps_per_second := f64(MEASURED_STEP_COUNT) * 1000 / elapsed_milliseconds;
	if elapsed_milliseconds <= 0 || milliseconds_per_step <= 0 || steps_per_second <= 0 ||
		f64_finite_state(elapsed_milliseconds) != .Ok ||
		f64_finite_state(milliseconds_per_step) != .Ok ||
		f64_finite_state(steps_per_second) != .Ok
	{
		_ = benchmark_owner_destroy(&measured);
		return {status=.Validation_Failed};
	}
	if benchmark_owner_destroy(&measured) != .Ok
	{
		return {status=.Release_Failed};
	}
	return {status=write_result(arguments.output_path, counters, elapsed_milliseconds)};
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis-contact-churn-grid-4k --worker-count=<positive count up to engine maximum> --output=<path>");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v step_phase=%v failed_step=%d exhausted_pool_power_mask=%d",
			result.status, result.physics_status, result.step_phase, result.failed_step,
			result.exhausted_pool_power_mask,
		);
		os.exit(2);
	}
	os.exit(0);
}
