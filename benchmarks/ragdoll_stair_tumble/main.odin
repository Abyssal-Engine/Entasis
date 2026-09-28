package ragdoll_stair_tumble

import "core:fmt"
import support "../benchmark_support"
import "core:os"
import "core:time"
import observation "../../tools/physics_observation"
import replay "../../tools/physics_replay"
import scene "../../tools/physics_scene"
import entasis "entasis:entasis"

CSV_HEADER :: "worker_count,step_index,active_constraints,active_pairs,body_count,shape_count,constraint_count,sleeping_body_count,invalid_transform_count,case_status,metric_status,physics_elapsed_ms";

Argument_Status :: enum u8
{
	Ok,
	Invalid,
}

Arguments :: struct
{
	recording: support.Recording_Options,
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

parse_arguments :: proc(arguments: []string) -> (Arguments, Argument_Status)
{
	recording: support.Recording_Options;
	remaining: []string;
	admission: support.Admission;
	recording, remaining, admission = support.recording_arguments("ragdoll_stair_tumble", arguments);
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

write_result :: proc(arguments: Arguments, sample: Benchmark_Sample) -> Case_Status
{
	_, stat_error := os.stat(arguments.output_path, context.temp_allocator);
	if stat_error != nil && stat_error != os.Error(os.General_Error.Not_Exist)
	{
		return .Output_Failed;
	}
	file, open_error := os.open(arguments.output_path, {.Write, .Append, .Create}, os.Permissions_Default_File);
	if open_error != nil
	{
		return .Output_Failed;
	}
	if stat_error == os.Error(os.General_Error.Not_Exist)
	{
		header := CSV_HEADER + support.RECORDING_HEADER + "\n";
		written, write_error := os.write(file, transmute([]byte)header);
		if write_error != nil || written != len(header)
		{
			_ = os.close(file);
			return .Output_Failed;
		}
	}
	// nine 64-bit integers, status text, and finite f64 at %.9f fit in 1 KiB
	row_buffer: [1024]byte;
	row := fmt.bprintfln(
		row_buffer[:], "%d,%d,%d,%d,%d,%d,%d,%d,%d,ok,ok,%.9f%s",
		arguments.worker_count, MEASURED_STEP_COUNT,
		sample.stats.active_constraints, sample.stats.active_pairs,
		sample.stats.active_bodies + sample.stats.sleeping_bodies + sample.stats.statics,
		PROTOCOL_SHAPE_COUNT, AUTHORED_CONSTRAINT_COUNT,
		sample.stats.sleeping_bodies, sample.invalid_transform_count, sample.elapsed_ms,
		support.recording_columns(arguments.recording, "native_step_sum"),
	);
	written, write_error := os.write(file, transmute([]byte)row);
	close_error := os.close(file);
	if write_error != nil || written != len(row) || close_error != nil
	{
		return .Output_Failed;
	}
	return .Ok;
}

measure_world :: proc(owner: ^Benchmark_Owner, recording: support.Recording_Options, workers: int) -> (Benchmark_Result, Benchmark_Sample)
{
	when support.RECORDING_ENABLED
	{
		return measure_recorded_world(owner, recording, workers);
	}
	else
	{
	// fixed 300/600-step storage is at most 4.8 KiB, owned before timing begins
	durations: [MEASURED_STEP_COUNT]i64;
	for step_index in 0 ..< MEASURED_STEP_COUNT
	{
		start := time.tick_now();
		status := entasis.world_step(&owner.world, TIMESTEP_DURATION);
		end := time.tick_now();
		if status != .Ok
		{
			return {status=.Step_Failed, physics_status=status, phase=.Measured, failed_step=step_index + 1}, {};
		}
		durations[step_index] = i64(time.tick_diff(start, end));
	}
	sample: Benchmark_Sample;
	for duration in durations
	{
		if duration <= 0
		{
			return {status=.Validation_Failed, phase=.Measured}, {};
		}
		sample.elapsed_ms += f64(duration) / 1_000_000;
	}
	if sample.elapsed_ms <= 0 ||
		transmute(u64)sample.elapsed_ms & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000
	{
		return {status=.Validation_Failed, phase=.Measured}, {};
	}
	result := benchmark_validate(owner, &sample);
	return result, sample;
	}
}

run_warmup :: proc(worker_count: int) -> Benchmark_Result
{
	pool: entasis.Buffer_Pool;
	pool_status := entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE);
	if pool_status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=pool_status, phase=.Warmup};
	}
	owner: Benchmark_Owner;
	result := benchmark_owner_create(&owner, &pool, worker_count);
	result.phase = .Warmup;
	if result.status == .Ok
	{
		for step_index in 0 ..< WARMUP_STEP_COUNT
		{
			status := entasis.world_step(&owner.world, TIMESTEP_DURATION);
			if status != .Ok
			{
				result = {status=.Step_Failed, physics_status=status, phase=.Warmup, failed_step=step_index + 1};
				break;
			}
		}
	}
	// release the world before its pool, including on construction/step failure
	release_status := benchmark_owner_destroy(&owner);
	pool_status = entasis.buffer_pool_destroy(&pool);
	if release_status == .Ok
	{
		release_status = pool_status;
	}
	if release_status != .Ok
	{
		result.release_status = release_status;
		if result.status == .Ok
		{
			result.status = .Release_Failed;
		}
	}
	return result;
}

execute_case :: proc(
	owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, worker_count: int, recording: support.Recording_Options,
) -> (Benchmark_Result, Benchmark_Sample)
{
	result := benchmark_owner_create(owner, pool, worker_count);
	if result.status != .Ok
	{
		return result, {};
	}
	return measure_world(owner, recording, worker_count);
}

run_benchmark :: proc(arguments: Arguments) -> Benchmark_Result
{
	warmup_result := run_warmup(arguments.worker_count);
	if warmup_result.status != .Ok
	{
		return warmup_result;
	}
	// Bepu creates the measured world in a new pool after releasing warmup
	pool: entasis.Buffer_Pool;
	pool_status := entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE);
	if pool_status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=pool_status};
	}
	owner: Benchmark_Owner;
	result, sample := execute_case(&owner, &pool, arguments.worker_count, arguments.recording);
	release_status := benchmark_owner_destroy(&owner);
	pool_status = entasis.buffer_pool_destroy(&pool);
	if release_status == .Ok
	{
		release_status = pool_status;
	}
	if release_status != .Ok
	{
		result.release_status = release_status;
		if result.status == .Ok
		{
			result.status = .Release_Failed;
		}
	}
	if result.status == .Ok
	{
		result.status = write_result(arguments, sample);
	}
	return result;
}

main :: proc()
{
	input_arguments := os.args;
	arguments, status := parse_arguments(input_arguments);
	if status != .Ok
	{
		fmt.eprintln("usage: entasis-ragdoll-stair-tumble --worker-count=<positive count up to engine maximum> --output=<path>");
		os.exit(2);
	}
	result := run_benchmark(arguments);
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed case=%s status=%v physics_status=%v phase=%v failed_step=%d release_status=%v",
			CASE_ID, result.status, result.physics_status, result.phase, result.failed_step, result.release_status,
		);
		os.exit(2);
	}
}

Measured_Observation :: observation.Observation;
Measured_Writer :: replay.Writer;
Recording_Status :: scene.Status;

when support.RECORDING_ENABLED
{
measure_recorded_world :: proc(owner: ^Benchmark_Owner, recording: support.Recording_Options, workers: int) -> (Benchmark_Result, Benchmark_Sample)
{
	settings: string = fmt.tprintf("fixture=benchmark/ragdoll_stair_tumble;workers=%d;steps=%d;timestep=%.9g;warmup=%d", workers, MEASURED_STEP_COUNT, TIMESTEP_DURATION, WARMUP_STEP_COUNT);
	o: Measured_Observation = {budget=recording.budget};
	defer observation.observation_destroy(&o);
	w: Measured_Writer;
	defer replay.writer_close(&w);
	status: Recording_Status = observation.observation_create(&o, &owner.world, owner.bodies);
	if status != .Ok
	{
		return {status=.Validation_Failed}, {};
	}
	status = observation.observation_read(&o, &owner.world);
	if status != .Ok
	{
		return {status=.Validation_Failed}, {};
	}
	metadata: scene.Metadata = {scenario="benchmark/ragdoll_stair_tumble", source="measured benchmark", revision=support.SOURCE_REVISION,
		compiler=ODIN_VERSION, host=fmt.tprintf("%v", ODIN_OS), configuration=support.BUILD_CONFIGURATION, components=support.BENCHMARK_COMPONENTS,
		settings=settings, settings_state=.Available, axis=.Physics, timestep=TIMESTEP_DURATION};
	status = replay.writer_open(&w, recording.output, metadata, replay.Compression(recording.compression), recording.budget-observation.observation_bytes(&o));
	if status != .Ok || replay.writer_frame(&w, &o.packet) != .Ok
	{
		return {status=.Validation_Failed}, {};
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
			return {status=.Validation_Failed}, {};
		}
		elapsed += i64(time.tick_diff(start, end));
		o.budget = recording.budget-replay.writer_bytes(&w);
		status = observation.observation_read(&o, &owner.world);
		o.packet.frame.tick = u64(step_index+1);
		o.packet.frame.phase = .Completed if step_index+1 == MEASURED_STEP_COUNT else .Active;
		w.budget = recording.budget-observation.observation_bytes(&o);
		if status != .Ok || replay.writer_frame(&w, &o.packet) != .Ok
		{
			return {status=.Validation_Failed}, {};
		}
	}
	sample: Benchmark_Sample = {elapsed_ms=f64(elapsed)/1e6};
	result: Benchmark_Result = benchmark_validate(owner, &sample);
	if result.status != .Ok || replay.writer_finish(&w, .Completed) != .Ok
	{
		return {status=.Validation_Failed}, {};
	}
	return result, sample;
}
}
