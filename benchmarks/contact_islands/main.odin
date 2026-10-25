package contact_islands

import "core:fmt"
import "core:os"
@(require) import "core:slice"
import "core:time"
import observation "../../tools/physics_observation"
import replay "../../tools/physics_replay"
import scene "../../tools/physics_scene"
import entasis "entasis:entasis"
import support "../benchmark_support"
import observations "../scene_observation"

Step_Latency :: struct
{
	p50_ms:     f64,
	p95_ms:     f64,
	p99_ms:     f64,
	maximum_ms: f64,
}

measure_world :: proc(owner: ^Benchmark_Owner, q: ^observations.Qualification) -> (Benchmark_Result, Benchmark_Sample, Step_Latency)
{
	when support.RECORDING_ENABLED
	{
		result: Benchmark_Result;
		sample: Benchmark_Sample;
		result, sample = measure_recorded_world(owner, q);
		return result, sample, {};
	}
	else
	{
	durations, allocation_error := make([]i64, owner.options.steps);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed}, {}, {};
	}
	defer delete(durations);
	timestep := 1 / f32(owner.options.timestep_hz);
	for step_index in 0 ..< owner.options.steps
	{
		start := time.tick_now();
		status := entasis.world_step(&owner.world, timestep);
		end := time.tick_now();
		if status != .Ok
		{
			return {status=.Step_Failed, physics_status=status, phase=.Measured, failed_step=step_index + 1}, {}, {};
		}
		durations[step_index] = i64(time.tick_diff(start, end));
		if benchmark_assess(owner, q, .Measured, step_index+1) != .Ok
		{
			return {status=.Validation_Failed}, {}, {}
		}
	}
	sample: Benchmark_Sample;
	for duration, step_index in durations
	{
		if duration <= 0
		{
			return {status=.Validation_Failed, phase=.Measured}, {}, {};
		}
		sample.elapsed_ms += f64(duration) / 1_000_000;
		fmt.printfln("STEP_TIMING step=%d elapsed_ns=%d", step_index + 1, duration);
	}
	if sample.elapsed_ms <= 0 ||
	transmute(u64)sample.elapsed_ms & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000
	{
		return {status=.Validation_Failed, phase=.Measured}, {}, {};
	}
	slice.sort(durations);
	latency: Step_Latency = {
		p50_ms=f64(durations[(50 * len(durations) + 99) / 100 - 1]) / 1_000_000,
		p95_ms=f64(durations[(95 * len(durations) + 99) / 100 - 1]) / 1_000_000,
		p99_ms=f64(durations[(99 * len(durations) + 99) / 100 - 1]) / 1_000_000,
		maximum_ms=f64(durations[len(durations) - 1]) / 1_000_000,
	};
	result := benchmark_validate(owner, &sample);
	return result, sample, latency;
	}
}

execute_case :: proc(
	owner: ^Benchmark_Owner, pool: ^entasis.Buffer_Pool, options: support.Options, q: ^observations.Qualification,
) -> (Benchmark_Result, Benchmark_Sample, Step_Latency)
{
	result := benchmark_owner_create(owner, pool, options);
	if result.status != .Ok
	{
		return result, {}, {};
	}
	if benchmark_assess(owner, q, .Construction, 0) != .Ok
	{
		return {status=.Validation_Failed}, {}, {}
	}
	timestep := 1 / f32(owner.options.timestep_hz);
	for step_index in 0 ..< owner.options.warmup_steps
	{
		status := entasis.world_step(&owner.world, timestep);
		if status != .Ok
		{
			return {status=.Step_Failed, physics_status=status, phase=.Warmup, failed_step=step_index + 1}, {}, {};
		}
		if benchmark_assess(owner, q, .Warmup, step_index+1) != .Ok
		{
			return {status=.Validation_Failed}, {}, {}
		}
	}
	// contact islands measures the already-warmed world
	return measure_world(owner, q);
}

run_benchmark :: proc(options: support.Options, parameters: string) -> Benchmark_Result
{
	q: observations.Qualification
	defer observations.qualification_destroy(&q)
	if observations.qualification_create(&q, benchmark_qualification_configuration(options), options.verify) != nil
	{
		return {status=.Allocation_Failed}
	}
	pool: entasis.Buffer_Pool;
	pool_status := entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE);
	if pool_status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=pool_status};
	}
	owner: Benchmark_Owner;
	result: Benchmark_Result;
	sample: Benchmark_Sample;
	latency: Step_Latency;
	result, sample, latency = execute_case(&owner, &pool, options, &q);
	if result.status == .Ok
	{
		observations.qualification_complete(&q)
		sample.physical = q.result
	}
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
		latency_header :: ",step_p50_ms,step_p95_ms,step_p99_ms,step_max_ms";
		latency_values: string = ",unavailable,unavailable,unavailable,unavailable";
		when !support.RECORDING_ENABLED
		{
			latency_buffer: [256]byte;
			latency_values = fmt.bprintf(latency_buffer[:], ",%.9f,%.9f,%.9f,%.9f",
				latency.p50_ms, latency.p95_ms, latency.p99_ms, latency.maximum_ms);
		}
		when support.BENCHMARK_COMPONENTS == "all"
		{
			components: Restitution_Components;
			result, components = measure_restitution_components(options.worker_count);
			if result.status != .Ok
			{
				return result;
			}
			trigger_components: [3]Trigger_Component = measure_trigger_components(options.worker_count);
			mixed_components: [5]Mixed_Collider_Component = measure_mixed_collider_components(options.worker_count);
			values_buffer: [2048]byte;
			values: string = fmt.bprintf(values_buffer[:],
				",%s,%v,%d,%d,%.9f,%d,%d,%d,%.9f,%d,%d,%d,%.9f,%d,%d,%d",
				support.BENCHMARK_COMPONENTS, components.state, RESTITUTION_BODY_COUNT, RESTITUTION_MEASURED_COUNT,
				components.elapsed_ms[0], components.bounces[0], components.pairs[0], components.pool_bytes[0],
				components.elapsed_ms[1], components.bounces[1], components.pairs[1], components.pool_bytes[1],
				components.elapsed_ms[2], components.bounces[2], components.pairs[2], components.pool_bytes[2]);
			count: int = len(values);
			count += len(fmt.bprintf(values_buffer[count:], ",Available,%d,%d", TRIGGER_COLLIDABLE_COUNT, TRIGGER_MEASURED_TRANSITIONS));
			for component in trigger_components
			{
				count += len(fmt.bprintf(values_buffer[count:], ",%.9f,%.9f,%d,%d,%d,%d,%d,%d", component.step_ms, component.consume_ms,
						component.sensors, component.enters, component.exits, component.final_constraints, component.world_pool_bytes, component.worker_pool_bytes));
			}
			count += len(fmt.bprintf(values_buffer[count:], ",Available,%d,%d", TRIGGER_COLLIDABLE_COUNT, TRIGGER_MEASURED_TRANSITIONS));
			for component in mixed_components
			{
				count += len(fmt.bprintf(values_buffer[count:], ",%.9f,%.9f,%.9f,%.9f,%d,%d,%d,%d,%d,%d",
						component.step_ms, component.parent_drain_ms, component.part_drain_ms, component.dormant_ms,
						component.parent_enters, component.parent_exits, component.part_enters, component.part_exits,
						component.physical_responses, component.world_pool_bytes));
			}
			count += len(fmt.bprintf(values_buffer[count:], "%s", latency_values));
			values = string(values_buffer[:count]);
			result.status = support.write_result(options, parameters, sample,
				",benchmark_components,restitution_state,restitution_bodies,restitution_measured_steps,restitution_0_ms,restitution_0_bounces,restitution_0_pairs,restitution_0_pool_bytes,restitution_10_ms,restitution_10_bounces,restitution_10_pairs,restitution_10_pool_bytes,restitution_100_ms,restitution_100_bounces,restitution_100_pairs,restitution_100_pool_bytes,tr01_state,tr01_collidables,tr01_measured_transitions,tr01_0_step_ms,tr01_0_consume_ms,tr01_0_sensors,tr01_0_enters,tr01_0_exits,tr01_0_final_constraints,tr01_0_world_pool_bytes,tr01_0_worker_pool_bytes,tr01_10_step_ms,tr01_10_consume_ms,tr01_10_sensors,tr01_10_enters,tr01_10_exits,tr01_10_final_constraints,tr01_10_world_pool_bytes,tr01_10_worker_pool_bytes,tr01_100_step_ms,tr01_100_consume_ms,tr01_100_sensors,tr01_100_enters,tr01_100_exits,tr01_100_final_constraints,tr01_100_world_pool_bytes,tr01_100_worker_pool_bytes" + MIXED_COLLIDER_HEADERS + latency_header,
				values);
		}
		else
		{
			values_buffer: [512]byte;
			values: string = fmt.bprintf(values_buffer[:], ",common,Unavailable%s", latency_values);
			result.status = support.write_result(options, parameters, sample,
				",benchmark_components,restitution_state" + latency_header, values);
		}
	}
	if result.status == .Ok && q.result.status == .Fail
	{
		fmt.eprintfln("physical_failed reason=%s phase=%s step=%d body=%d value_m=%.9f", observations.physical_reason_text(q.result.reason),
			observations.assessment_phase_text(q.result.first.phase), q.result.first.step, q.result.first.body, q.result.first.value)
		result.status = .Physical_Failed
	}
	return result;
}

main :: proc()
{
	options, diagnostic := support.parse_options(.Contact_Islands, os.args[1:]);
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
measure_recorded_world :: proc(owner: ^Benchmark_Owner, q: ^observations.Qualification) -> (Benchmark_Result, Benchmark_Sample)
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
	metadata: scene.Metadata = {scenario="benchmark/contact_islands", source="measured benchmark", revision=support.SOURCE_REVISION,
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
		physics_status = entasis.world_step(&owner.world, 1/f32(owner.options.timestep_hz));
		end: time.Tick = time.tick_now();
		if physics_status != .Ok
		{
			return {status=.Validation_Failed}, {};
		}
		elapsed += i64(time.tick_diff(start, end));
		if benchmark_assess(owner, q, .Measured, step_index+1) != .Ok
		{
			return {status=.Validation_Failed}, {}
		}
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
