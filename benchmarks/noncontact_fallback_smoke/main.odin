package noncontact_fallback_smoke

import "core:fmt"
import support "../benchmark_support"
import "core:os"
import "core:time"
import observation "../../tools/physics_observation"
import replay "../../tools/physics_replay"
import scene "../../tools/physics_scene"
import entasis "entasis:entasis"

CSV_HEADER :: "body_count,constraint_count,invalid_state_count,case_status,metric_status,physics_elapsed_ms";

Argument_Status :: enum u8
{
	Ok, Invalid,
}
Arguments :: struct
{
	recording: support.Recording_Options,
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

parse_arguments :: proc(arguments: []string) -> (Arguments, Argument_Status)
{
	recording: support.Recording_Options;
	remaining: []string;
	admission: support.Admission;
	recording, remaining, admission = support.recording_arguments("noncontact_fallback_smoke", arguments);
	if admission != .Ok
	{
		return {}, .Invalid;
	}

	if len(remaining) != 3
	{
		return {}, .Invalid;
	}
	result: Arguments = {recording=recording};
	worker_state := Argument_Status.Invalid;
	output_state := Argument_Status.Invalid;
	for argument in remaining[1:]
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
	if recording.mode == .On && os.dir(recording.output) != os.dir(result.output_path)
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

write_result :: proc(path: string, counters: Stability_Counters, elapsed_milliseconds: f64, recording: support.Recording_Options) -> Case_Status
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
	if header_state == .Ok && fmt.fprintfln(file, CSV_HEADER+support.RECORDING_HEADER) <= 0
	{
		_ = os.close(file);
		return .Validation_Failed;
	}
	if fmt.fprintfln(
		file, "%d,%d,%d,ok,ok,%.9f%s",
		counters.dynamic_body_count, counters.constraint_count,
		counters.invalid_state_count, elapsed_milliseconds, support.recording_columns(recording, "whole_loop"),
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
	step_result := benchmark_owner_step(&warmup, WARMUP_STEP_COUNT, .Warmup);
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
	elapsed_nanoseconds: i64;
	when support.RECORDING_ENABLED
	{
		step_result, elapsed_nanoseconds = measure_recorded_world(&measured, arguments.recording, arguments.worker_count);
	}
	else
	{
		start := time.tick_now();
		step_result = benchmark_owner_step(&measured, MEASURED_STEP_COUNT, .Measured);
		end := time.tick_now();
		elapsed_nanoseconds = i64(time.tick_diff(start, end));
	}
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
	return {status=write_result(arguments.output_path, counters, elapsed_milliseconds, arguments.recording)};
}

main :: proc()
{
	arguments, status := parse_arguments(os.args);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis_noncontact_fallback_smoke --worker-count=<positive count up to engine maximum> --output=<path>");
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

Measured_Observation :: observation.Observation;
Measured_Writer :: replay.Writer;
Recording_Status :: scene.Status;

when support.RECORDING_ENABLED
{
measure_recorded_world :: proc(owner: ^Benchmark_Simulation_Owner, recording: support.Recording_Options, workers: int) -> (Benchmark_Run_Result, i64)
{
	settings: string = fmt.tprintf("fixture=benchmark/noncontact_fallback_smoke;workers=%d;steps=%d;timestep=%.9g;warmup=%d", workers, MEASURED_STEP_COUNT, TIMESTEP_DURATION, WARMUP_STEP_COUNT);
	o: Measured_Observation = {budget=recording.budget};
	defer observation.observation_destroy(&o);
	w: Measured_Writer;
	defer replay.writer_close(&w);
	status: Recording_Status = observation.observation_create(&o, &owner.world, owner.recording_bodies[:]);
	if status != .Ok
	{
		return {status=.Validation_Failed}, 0;
	}
	status = observation.observation_read(&o, &owner.world);
	if status != .Ok
	{
		return {status=.Validation_Failed}, 0;
	}
	metadata: scene.Metadata = {scenario="benchmark/noncontact_fallback_smoke", source="measured benchmark", revision=support.SOURCE_REVISION,
		compiler=ODIN_VERSION, host=fmt.tprintf("%v", ODIN_OS), configuration=support.BUILD_CONFIGURATION, components=support.BENCHMARK_COMPONENTS,
		settings=settings, settings_state=.Available, axis=.Physics, timestep=TIMESTEP_DURATION};
	status = replay.writer_open(&w, recording.output, metadata, replay.Compression(recording.compression), recording.budget-observation.observation_bytes(&o));
	if status != .Ok || replay.writer_frame(&w, &o.packet) != .Ok
	{
		return {status=.Validation_Failed}, 0;
	}
	elapsed: i64;
	for step_index in 0 ..< MEASURED_STEP_COUNT
	{
		start: time.Tick = time.tick_now();
		physics_status: entasis.Status;
		physics_status = entasis.world_step(&owner.world, TIMESTEP_DURATION);
		end: time.Tick = time.tick_now();
		if physics_status != .Ok
		{
			return {status=.Validation_Failed}, 0;
		}
		elapsed += i64(time.tick_diff(start, end));
		o.budget = recording.budget-replay.writer_bytes(&w);
		status = observation.observation_read(&o, &owner.world);
		o.packet.frame.tick = u64(step_index+1);
		o.packet.frame.phase = .Completed if step_index+1 == MEASURED_STEP_COUNT else .Active;
		w.budget = recording.budget-observation.observation_bytes(&o);
		if status != .Ok || replay.writer_frame(&w, &o.packet) != .Ok
		{
			return {status=.Validation_Failed}, 0;
		}
	}
	if replay.writer_finish(&w, .Completed) != .Ok
	{
		return {status=.Validation_Failed}, 0;
	}
	return {status=.Ok}, elapsed;
}
}
