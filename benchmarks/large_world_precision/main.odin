package large_world_precision

import "core:fmt"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:time"
import entasis "entasis:entasis"
import support "../benchmark_support"

Precision_Arguments :: struct
{
	worker_count: int,
	output: string,
	precision_case: Precision_Case,
	case_admission: Precision_Admission,
	sampling_mode: Sampling_Mode,
}

precision_arguments :: proc(args: []string) -> (Precision_Arguments, Precision_Admission)
{
	result: Precision_Arguments;
	if len(args) != 4 && len(args) != 5
	{
		return {}, .Rejected;
	}
	sampling_admission: Precision_Admission;
	for arg in args[1:]
	{
		if strings.has_prefix(arg, "--worker-count=")
		{
			if result.worker_count != 0
			{
				return {}, .Rejected;
			}
			worker_count, parsed := strconv.parse_int(arg[len("--worker-count="):]);
			if !parsed || worker_count <= 0 || worker_count > entasis.MAXIMUM_WORKER_COUNT
			{
				return {}, .Rejected;
			}
			result.worker_count = worker_count;
		}
		else if strings.has_prefix(arg, "--output=")
		{
			if len(result.output) != 0
			{
				return {}, .Rejected;
			}
			result.output = arg[len("--output="):];
			if len(result.output) == 0
			{
				return {}, .Rejected;
			}
		}
		else if strings.has_prefix(arg, "--sampling-mode=")
		{
			if sampling_admission == .Accepted
			{
				return {}, .Rejected;
			}
			result.sampling_mode, sampling_admission = sampling_mode_parse(arg[len("--sampling-mode="):]);
			if sampling_admission != .Accepted
			{
				return {}, .Rejected;
			}
		}
		else if strings.has_prefix(arg, "--precision-case=")
		{
			if result.case_admission == .Accepted
			{
				return {}, .Rejected;
			}
			value := arg[len("--precision-case="):];
			result.precision_case, result.case_admission = precision_case_parse(value);
			if result.case_admission != .Accepted
			{
				return {}, .Rejected;
			}
		}
		else
		{
			return {}, .Rejected;
		}
	}
	if result.worker_count == 0 || len(result.output) == 0 ||
		result.case_admission != .Accepted
	{
		return {}, .Rejected;
	}
	return result, .Accepted;
}

precision_parameters :: proc(
	options: support.Options,
	precision_case: Precision_Case,
	buffer: []byte,
	mode: Sampling_Mode = .Instrumented,
) -> (string, Precision_Admission)
{
	profile := precision_profile(precision_case);
	if profile.fixture == .Distributed || profile.fixture == .Query_Spanning
	{
		return distributed_parameters(options, precision_case, buffer, mode);
	}
	parameters: string;
	if profile.fixture == .Query_Local
	{
		parameters = fmt.bprintf(
			buffer,
			"workload=large_world_precision;fixture=%s;precision-candidate=%s;precision-case=%s;queries-per-family=%d;families=4;passes=%d;warmup-queries-per-family=%d;population=%d;bodies=%d;statics=%d;routes=primitive,compound,mesh;timing=serial-query-passes-with-global-conversion;sampling=%s",
			precision_fixture_text(precision_case),
			PRECISION_CANDIDATE,
			precision_case_text(precision_case),
			QUERY_COUNT,
			QUERY_PASS_COUNT,
			QUERY_WARMUP_COUNT,
			options.population_count,
			options.body_count,
			options.static_count,
			sampling_mode_text(mode),
		);
	}
	else
	{
		parameters = fmt.bprintf(
			buffer,
			"workload=large_world_precision;fixture=%s;precision-candidate=%s;precision-case=%s;steps=%d;timestep-hz=%d;warmup-steps=%d;velocity-iterations=%d;substeps=%d;population=%d;bodies=%d;statics=%d;grid=%d,%d,%d;spacing=%.9g,%.9g,%.9g;spawn-height=%.9g;container-size=%.9g,%.9g,%.9g;timing=%s;sampling=%s",
			precision_fixture_text(precision_case),
			PRECISION_CANDIDATE,
			precision_case_text(precision_case),
			options.steps,
			options.timestep_hz,
			options.warmup_steps,
			options.velocity_iterations,
			options.substeps,
			options.population_count,
			options.body_count,
			options.static_count,
			options.grid[0],
			options.grid[1],
			options.grid[2],
			options.spacing[0],
			options.spacing[1],
			options.spacing[2],
			options.spawn_height,
			options.container_size[0],
			options.container_size[1],
			options.container_size[2],
			"world-step-sum-excluding-maintenance-and-export" if mode == .Engine_Only else "whole-loop-with-global-export",
			sampling_mode_text(mode),
		);
	}
	if len(parameters) > 0 && len(parameters) < len(buffer)
	{
		return parameters, .Accepted;
	}
	return parameters, .Rejected;
}

run_benchmark :: proc(
	options: support.Options,
	precision_case: Precision_Case,
	parameters: string,
	mode: Sampling_Mode = .Instrumented,
	memory: ^Precision_Memory = nil,
) -> support.Benchmark_Result
{
	
	when PRECISION_NATIVE_GLOBAL
	{
		admission := mixed64_candidate_admission(options.worker_count);
		if admission.status != .Ok
		{
			return admission;
		}
	}
	pool: entasis.Buffer_Pool;
	setup_nanoseconds: i64;
	destruction_nanoseconds: i64;
	setup_start: time.Tick = time.tick_now();
	pool_status := precision_pool_initialize(&pool, memory);
	setup_nanoseconds += i64(time.tick_diff(setup_start, time.tick_now()));
	if pool_status != .Ok
	{
		return {status=.Allocation_Failed, physics_status=pool_status};
	}
	owner: Benchmark_Owner;
	// query creation includes preparation and query warmup in setup time
	setup_start = time.tick_now();
	result := benchmark_owner_create(&owner, &pool, options, precision_case, memory);
	setup_nanoseconds += i64(time.tick_diff(setup_start, time.tick_now()));
	if result.status == .Ok && precision_profile(precision_case).fixture != .Query_Local &&
		precision_profile(precision_case).fixture != .Query_Spanning
	{
		result = step_world(&owner, WARMUP_STEPS, .Warmup);
	}
	destruction_start: time.Tick = time.tick_now();
	release_status := benchmark_owner_destroy(&owner);
	destruction_nanoseconds += i64(time.tick_diff(destruction_start, time.tick_now()));
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
		setup_start = time.tick_now();
		result = benchmark_owner_create(&owner, &pool, options, precision_case, memory);
		setup_nanoseconds += i64(time.tick_diff(setup_start, time.tick_now()));
	}
	owner.sampling_mode = mode;
	sample: support.Benchmark_Sample;
	if result.status == .Ok
	{
		if precision_profile(precision_case).fixture == .Query_Local
		{
			result, sample = query_local_measure(&owner);
		}
		else if precision_profile(precision_case).fixture == .Query_Spanning
		{
			when PRECISION_NATIVE_GLOBAL
			{
				result, sample = global_spanning_measure(&owner);
			}
			else
			{
				unreachable();
			}
		}
		else
		{
			result, sample = measure_world(&owner);
		}
	}
	frame_median_ms, frame_max_ms, maintenance_max_ms, export_checksum: f64;
	maintenance_count, sampling_count: int;
	if result.status == .Ok
	{
		frame_median_ms = owner.frame_median_ms;
		frame_max_ms = owner.frame_max_ms;
		maintenance_count = owner.maintenance_count;
		maintenance_max_ms = owner.maintenance_max_ms;
		export_checksum = owner.export_checksum;
		sampling_count = MEASURED_STEPS;
		if precision_profile(precision_case).fixture == .Query_Local || precision_profile(precision_case).fixture == .Query_Spanning
		{
			sampling_count = QUERY_PASS_COUNT;
		}
	}
	destruction_start = time.tick_now();
	release_status = benchmark_owner_destroy(&owner);
	pool_status = entasis.buffer_pool_destroy(&pool);
	destruction_nanoseconds += i64(time.tick_diff(destruction_start, time.tick_now()));
	if release_status == .Ok
	{
		release_status = pool_status;
	}
	if result.status == .Ok && release_status == .Ok
	{
		output_header: string = ",precision_case,precision_candidate,sampling_mode,sampling_count,frame_median_ms,frame_max_ms,maintenance_count,maintenance_max_ms,export_checksum,setup_elapsed_ms,destruction_elapsed_ms";
		output_values: string = sampling_columns(mode, precision_case, sampling_count,
			frame_median_ms, frame_max_ms, maintenance_count, maintenance_max_ms,
			export_checksum, f64(setup_nanoseconds)/1e6, f64(destruction_nanoseconds)/1e6);
		if mode == .Memory
		{
			memory_values: string;
			memory_admission: Precision_Admission;
			memory_values, memory_admission = precision_memory_columns(memory, precision_case);
			if memory_admission != .Accepted
			{
				return {status=.Validation_Failed};
			}
			output_header = fmt.tprintf("%s%s", output_header, MEMORY_HEADER);
			output_values = fmt.tprintf("%s%s", output_values, memory_values);
		}
		result.status = support.write_result(
			options,
			parameters,
			sample,
			output_header,
			output_values, sampling_timing_method(mode), .Absent,
		);
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
	return result;
}

main :: proc()
{
	arguments, admission := precision_arguments(os.args);
	if admission != .Accepted
	{
		fmt.eprintln("invalid benchmark options: require one each of --worker-count, --output and --precision-case");
		os.exit(2);
	}
	if (arguments.precision_case == .Queries_Local || arguments.precision_case == .Queries_Spanning) &&
		arguments.worker_count != 1
	{
		fmt.eprintln("precision query workloads require --worker-count=1");
		os.exit(2);
	}
	if precision_case_supported(arguments.precision_case) != .Accepted
	{
		fmt.eprintfln(
			"precision_case_unsupported candidate=%s case=%s",
			PRECISION_CANDIDATE,
			precision_case_text(arguments.precision_case),
		);
		os.exit(2);
	}
	options := precision_options(
		arguments.worker_count,
		arguments.output,
		arguments.precision_case,
	);
	parameter_buffer: [support.PARAMETER_CAPACITY]byte;
	parameters, format_admission := precision_parameters(
		options,
		arguments.precision_case,
		parameter_buffer[:],
		arguments.sampling_mode,
	);
	if format_admission != .Accepted
	{
		fmt.eprintln("cannot format benchmark parameters");
		os.exit(2);
	}
	if arguments.sampling_mode == .Engine_Only &&
		(arguments.precision_case == .Queries_Local || arguments.precision_case == .Queries_Spanning)
	{
		fmt.eprintln("engine-only sampling supports simulation cells, not query-conversion isolation");
		os.exit(2);
	}
	memory: Precision_Memory;
	memory_pointer: ^Precision_Memory;
	if arguments.sampling_mode == .Memory
	{
		precision_memory_initialize(&memory);
		memory_pointer = &memory;
	}
	result := run_benchmark(options, arguments.precision_case, parameters, arguments.sampling_mode, memory_pointer);
	if memory_pointer != nil
	{
		precision_memory_finish(memory_pointer);
	}
	if result.status != .Ok
	{
		fmt.eprintfln(
			"benchmark_failed candidate=%s case=%s status=%v physics_status=%v phase=%v failed_step=%d release_status=%v",
			PRECISION_CANDIDATE,
			precision_case_text(arguments.precision_case),
			result.status,
			result.physics_status,
			result.phase,
			result.failed_step,
			result.release_status,
		);
		os.exit(2);
	}
}
