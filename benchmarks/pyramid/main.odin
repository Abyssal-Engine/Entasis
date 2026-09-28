package pyramid

import "core:fmt"
import "core:os"
import "core:time"
import observation "../../tools/physics_observation"
import replay "../../tools/physics_replay"
import scene "../../tools/physics_scene"
import entasis "entasis:entasis"
import support "../benchmark_support"

measure_world :: proc(owner: ^Benchmark_Owner) -> (Benchmark_Result, Benchmark_Sample)
{
	when support.RECORDING_ENABLED
	{
		return measure_recorded_world(owner);
	}
	else
	{
	durations, allocation_error := make([]i64, owner.options.steps);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed}, {};
	}
	defer delete(durations);
	timestep := 1 / f32(owner.options.timestep_hz);
	launch_step := owner.options.launch_step;
	projectile_count := owner.options.projectile_count;
	for step_index in 0 ..< owner.options.steps
	{
		start := time.tick_now();
		status := entasis.Status.Ok;
		if projectile_count > 0 && step_index == launch_step
		{
			status = benchmark_launch_projectiles(owner);
		}
		if status == .Ok
		{
			status = entasis.world_step(&owner.world, timestep);
		}
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

execute_case :: proc(
owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, options: support.Options,
) -> (Benchmark_Result, Benchmark_Sample)
{
	result := benchmark_owner_create(owner, pool, options);
	if result.status != .Ok
	{
		return result, {};
	}
	return measure_world(owner);
}

run_benchmark :: proc(options: support.Options, parameters: string) -> Benchmark_Result
{
	pool: entasis.Buffer_Pool;
	pool_status := entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE);
	if pool_status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=pool_status};
	}
	owner: Benchmark_Owner;
	result, sample := execute_case(&owner, &pool, options);
	release_status := benchmark_owner_destroy(&owner);
	pool_status = entasis.buffer_pool_destroy(&pool);
	if release_status == .Ok
	{
		release_status = pool_status;
	}
	if release_status != .Ok
	{
		if result.release_status == .Ok
		{
			result.release_status = release_status;
		}
		if result.status == .Ok
		{
			result.status = .Release_Failed;
		}
	}
	if result.status == .Ok
	{
		result.status = support.write_result(options, parameters, sample);
	}
	return result;
}


main :: proc()
{
	options, diagnostic := support.parse_options(.Pyramid, os.args[1:]);
	if len(diagnostic) != 0
	{
		fmt.eprintfln("invalid benchmark option: %s", diagnostic);
		os.exit(2);
	}
	parameter_buffer: [support.PARAMETER_CAPACITY]byte;
	parameters, status := support.canonical_parameters(options, parameter_buffer[:]);
	if status != .Ok
	{
		fmt.eprintln("cannot format benchmark parameters");
		os.exit(2);
	}
	result := run_benchmark(options, parameters);
	if result.status != .Ok
	{
		fmt.eprintfln("benchmark_failed status=%v physics_status=%v phase=%v failed_step=%d release_status=%v",
		result.status, result.physics_status, result.phase, result.failed_step, result.release_status);
		os.exit(2);
	}
}

Measured_Observation :: observation.Observation;
Measured_Writer :: replay.Writer;
Recording_Status :: scene.Status;

when support.RECORDING_ENABLED
{
measure_recorded_world :: proc(owner: ^Benchmark_Owner) -> (Benchmark_Result, Benchmark_Sample)
{
	recording: support.Recording_Options = owner.options.recording;
	parameter_buffer: [support.PARAMETER_CAPACITY]u8;
	parameters, admission := support.canonical_parameters(owner.options, parameter_buffer[:]);
	if admission != .Ok
	{
		return {status=.Validation_Failed}, {};
	}
	settings: string = fmt.tprintf("%s;workers=%d", parameters, owner.options.worker_count);
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
	metadata: scene.Metadata = {scenario="benchmark/pyramid", source="measured benchmark", revision=support.SOURCE_REVISION,
		compiler=ODIN_VERSION, host=fmt.tprintf("%v", ODIN_OS), configuration=support.BUILD_CONFIGURATION, components=support.BENCHMARK_COMPONENTS,
		settings=settings, settings_state=.Available, axis=.Physics, timestep=1/f32(owner.options.timestep_hz)};
	status = replay.writer_open(&w, recording.output, metadata, replay.Compression(recording.compression), recording.budget-observation.observation_bytes(&o));
	if status != .Ok || replay.writer_frame(&w, &o.packet) != .Ok
	{
		return {status=.Validation_Failed}, {};
	}
	elapsed: i64;
	for step_index in 0 ..< owner.options.steps
	{
		start: time.Tick = time.tick_now();
		physics_status: entasis.Status;
		if owner.options.projectile_count > 0 && step_index == owner.options.launch_step
		{
			physics_status = benchmark_launch_projectiles(owner);
		}
		if physics_status == .Ok
		{
			physics_status = entasis.world_step(&owner.world, 1/f32(owner.options.timestep_hz));
		}
		end: time.Tick = time.tick_now();
		if physics_status != .Ok
		{
			return {status=.Validation_Failed}, {};
		}
		elapsed += i64(time.tick_diff(start, end));
		o.budget = recording.budget-replay.writer_bytes(&w);
		status = observation.observation_read(&o, &owner.world);
		o.packet.frame.tick = u64(step_index+1);
		o.packet.frame.phase = .Completed if step_index+1 == owner.options.steps else .Active;
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
