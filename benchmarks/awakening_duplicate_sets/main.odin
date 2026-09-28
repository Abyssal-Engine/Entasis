package awakening_duplicate_sets

import "core:fmt"
import "core:os"
import "core:time"
import entasis "entasis:entasis"

CSV_HEADER :: "sleeping_body_count,active_body_count,awakened_body_count,inactive_set_count,expected_pair_count,pair_count,invalid_transform_count,case_status,metric_status,physics_elapsed_ms";

parse_worker_count :: proc "contextless" (value: string) -> (int, bool)
{
	if len(value) == 0
	{
		return 0, false;
	}
	result := 0;
	for character in value
	{
		if character < '0' || character > '9'
		{
			return 0, false;
		}
		digit := int(character - '0');
		if result > (max(int) - digit) / 10
		{
			return 0, false;
		}
		result = result * 10 + digit;
	}
	if result <= 0 || result > entasis.MAXIMUM_WORKER_COUNT
	{
		return 0, false;
	}
	return result, true;
}

write_result :: proc(path: string, result: Run_Result) -> bool
{
	_, stat_error := os.stat(path, context.temp_allocator);
	write_header := stat_error != nil;
	file, open_error := os.open(path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil || file == nil
	{
		return false;
	}
	if write_header && fmt.fprintfln(file, CSV_HEADER) <= 0
	{
		_ = os.close(file);
		return false;
	}
	ok := fmt.fprintfln(
		file, "%d,%d,%d,%d,%d,%d,%d,ok,ok,%.9f",
		SLEEPING_BODY_COUNT, ACTIVE_BODY_COUNT, SLEEPING_BODY_COUNT,
		result.inactive_set_count, EXPECTED_PAIR_COUNT, result.pair_count,
		result.invalid_transform_count, result.elapsed_ms,
	) > 0;
	if os.close(file) != nil
	{
		return false;
	}
	return ok;
}

main :: proc()
{
	if len(os.args) != 3 || len(os.args[1]) <= len("--worker-count=") ||
		os.args[1][:len("--worker-count=")] != "--worker-count=" ||
		len(os.args[2]) <= len("--output=") || os.args[2][:len("--output=")] != "--output="
	{
		fmt.eprintln("usage: entasis_awakening_duplicate_sets --worker-count=<positive count up to engine maximum> --output=<path>");
		os.exit(2);
	}
	worker_count, worker_ok := parse_worker_count(os.args[1][len("--worker-count="):]);
	output_path := os.args[2][len("--output="):];
	if !worker_ok || len(output_path) == 0
	{
		os.exit(2);
	}

	pool: entasis.Buffer_Pool;
	if entasis.buffer_pool_init(&pool, WORKER_POOL_BLOCK_SIZE) != .Ok
	{
		os.exit(2);
	}
	defer
	{
		_ = entasis.buffer_pool_destroy(&pool);
	}

	warmup: Owner;
	warmup_status, warmup_physics := owner_create(&warmup, &pool, worker_count);
	if warmup_status != .Ok
	{
		fmt.eprintfln("benchmark_failed status=%v physics_status=%v", warmup_status, warmup_physics);
		os.exit(2);
	}
	warmup_step_status := owner_step(&warmup);
	warmup_result := Run_Result{status=.Ok, physics_status=warmup_step_status};
	if warmup_step_status != .Ok
	{
		warmup_result.status = .Step_Failed;
	}
	else
	{
		warmup_result = owner_validate(&warmup);
	}
	if owner_destroy(&warmup) != .Ok || warmup_result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v",
			warmup_result.status,
			warmup_result.physics_status
		);
		os.exit(2);
	}

	measured: Owner;
	measured_status, measured_physics := owner_create(&measured, &pool, worker_count);
	if measured_status != .Ok
	{
		fmt.eprintfln("benchmark_failed status=%v physics_status=%v", measured_status, measured_physics);
		os.exit(2);
	}
	start := time.tick_now();
	step_status := owner_step(&measured);
	end := time.tick_now();
	result := Run_Result{status=.Ok, physics_status=step_status};
	if step_status != .Ok
	{
		result.status = .Step_Failed;
	}
	else
	{
		result = owner_validate(&measured);
	}
	result.elapsed_ms = f64(i64(time.tick_diff(start, end))) / 1_000_000;
	if owner_destroy(&measured) != .Ok || result.status != .Ok || result.elapsed_ms <= 0
	{
		fmt.eprintfln(
			"benchmark_failed status=%v physics_status=%v active=%d inactive_sets=%d pairs=%d invalid=%d",
			result.status, result.physics_status, result.active_body_count,
			result.inactive_set_count, result.pair_count, result.invalid_transform_count,
		);
		os.exit(2);
	}
	if !write_result(output_path, result)
	{
		os.exit(2);
	}
}
