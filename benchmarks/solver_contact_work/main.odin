package solver_contact_work

import "core:fmt"
import "core:math"
import "core:os"
import "core:slice"
import "core:time"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"
import contacts "../solver_contacts"

#assert(util.PRECISION_MODE == .Float_Coordinates);
MEASURED_PASSES :: 100;
WARMUP_PASSES :: 10;

Measurement :: struct
{
	times: [MEASURED_PASSES]f64,
	quality: [MEASURED_PASSES]contacts.Quality_Result,
	elapsed_ms: f64,
	completed, quality_failures: int,
	pool_before, pool_after: u64,
	worker_pool_before, worker_pool_after: u64,
	solve_status: physics.Physics_Status,
}

measure :: proc(replay: ^contacts.Replay, dispatcher: ^util.Thread_Dispatcher, scratch: []f64) -> Measurement
{
	result: Measurement;
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(dispatcher);
	for pass: int = -WARMUP_PASSES; pass < MEASURED_PASSES; pass += 1
	{
		if pass == 0
		{
			result.pool_before = util.buffer_pool_total_allocated_byte_count(replay.pool);
			result.worker_pool_before = util.worker_buffer_pools_total_allocated_byte_count(&dispatcher.worker_pools);
		}
		contacts.replay_reset(replay);
		start: time.Tick = time.tick_now();
		result.solve_status = contacts.replay_solve(replay, boundary);
		elapsed: time.Duration = time.tick_diff(start, time.tick_now());
		if result.solve_status != .Ok
		{
			if pass < 0
			{
				return result;
			}
			break;
		}
		quality: contacts.Quality_Result = contacts.quality_observe(replay, scratch);
		if pass < 0
		{
			continue;
		}
		result.times[pass] = f64(elapsed)/1e6;
		result.elapsed_ms += result.times[pass];
		result.quality[pass] = quality;
		result.completed += 1;
		if result.quality[pass].status != .Pass
		{
			result.quality_failures += 1;
		}
	}
	result.pool_after = util.buffer_pool_total_allocated_byte_count(replay.pool);
	result.worker_pool_after = util.worker_buffer_pools_total_allocated_byte_count(&dispatcher.worker_pools);
	return result;
}

write_measurement :: proc(o: Options, input: ^contacts.Input, counts: contacts.Counts, m: ^Measurement) -> int
{
	worst: contacts.Quality_Result;
	maxima, rms, percentiles: [MEASURED_PASSES]f64;
	for pass in 0 ..< m.completed
	{
		q: contacts.Quality_Result = m.quality[pass];
		worst.residual_max_mps = max(worst.residual_max_mps, q.residual_max_mps);
		worst.residual_rms_mps = max(worst.residual_rms_mps, q.residual_rms_mps);
		worst.residual_p95_mps = max(worst.residual_p95_mps, q.residual_p95_mps);
		worst.negative_impulse_max = max(worst.negative_impulse_max, q.negative_impulse_max);
		worst.friction_violation_max = max(worst.friction_violation_max, q.friction_violation_max);
		worst.twist_violation_max = max(worst.twist_violation_max, q.twist_violation_max);
		maxima[pass] = q.residual_max_mps;
		rms[pass] = q.residual_rms_mps;
		percentiles[pass] = q.residual_p95_mps;
		fmt.printfln("CONTACT_CALL sample=%d elapsed_ms=%.9f quality=%v rows=%d/%d residual_rows=%d residual_max_mps=%.9g residual_rms_mps=%.9g residual_p95_mps=%.9g finite_failures=%d invalid_coefficients=%d feasibility_failures=%d",
			pass+1, m.times[pass], q.status, q.observed_rows, q.expected_rows, q.residual_rows,
			q.residual_max_mps, q.residual_rms_mps, q.residual_p95_mps, q.finite_failures, q.invalid_coefficients, q.feasibility_failures);
	}
	if m.completed > 0
	{
		slice.sort(maxima[:m.completed]);
		slice.sort(rms[:m.completed]);
		slice.sort(percentiles[:m.completed]);
		lower: int = (m.completed-1)/2;
		upper: int = m.completed/2;
		fmt.printfln("CONTACT_QUALITY_SUMMARY metric=residual_max_mps min=%.9g median=%.9g max=%.9g", maxima[0], (maxima[lower]+maxima[upper])/2, maxima[m.completed-1]);
		fmt.printfln("CONTACT_QUALITY_SUMMARY metric=residual_rms_mps min=%.9g median=%.9g max=%.9g", rms[0], (rms[lower]+rms[upper])/2, rms[m.completed-1]);
		fmt.printfln("CONTACT_QUALITY_SUMMARY metric=residual_p95_mps min=%.9g median=%.9g max=%.9g", percentiles[0], (percentiles[lower]+percentiles[upper])/2, percentiles[m.completed-1]);
	}
	times: [MEASURED_PASSES]f64 = m.times;
	slice.sort(times[:m.completed]);
	p50, p95, maximum: f64;
	if m.completed > 0
	{
		p50 = times[(50*m.completed+99)/100-1];
		p95 = times[(95*m.completed+99)/100-1];
		maximum = times[m.completed-1];
	}
	case_status, metric_status, physics_status: string = "ok", "ok", "pass";
	exit_code: int;
	if m.completed != MEASURED_PASSES || m.solve_status != .Ok || m.elapsed_ms <= 0 || math.is_nan(m.elapsed_ms) || math.is_inf(m.elapsed_ms)
	{
		case_status = "failed";
		metric_status = "failed";
		exit_code = 3;
	}
	if m.pool_after != m.pool_before || m.worker_pool_after != m.worker_pool_before
	{
		case_status = "failed";
		metric_status = "failed";
		exit_code = 3;
		fmt.eprintln("CONTACT_WORK_MEASUREMENT_FAILED reason=pool-capacity-changed-after-warmup");
	}
	if m.quality_failures != 0 || m.completed != MEASURED_PASSES
	{
		physics_status = "fail";
		if exit_code == 0
		{
			exit_code = 2;
		}
	}
	origin: string = "accepted" if input.origin == .Accepted else "r02-pairs";
	parameters_buffer: [support.PARAMETER_CAPACITY]u8;
	parameters: string = fmt.bprintf(parameters_buffer[:], "workload=solver_contact_work;origin=%s;capture-step=%d;region=prepare-and-solver-without-final-pose;velocity-iterations=1;substeps=4;timestep-hz=60;measured-passes=100;warmup-passes=10;warmup-cycle=reset-solve-observe;cache=reset-warmed;observer-summary=maximum-over-calls;type-counts=%d|%d|%d|%d|%d|%d|%d|%d",
		origin, input.step, counts.type_counts[0], counts.type_counts[1], counts.type_counts[2], counts.type_counts[3],
		counts.type_counts[4], counts.type_counts[5], counts.type_counts[6], counts.type_counts[7]);
	file: ^os.File;
	error: os.Error;
	file, error = os.open(o.output, {.Write, .Append, .Create});
	if error != nil
	{
		fmt.eprintfln("CONTACT_WORK_OUTPUT_FAILED path=%s error=%v", o.output, error);
		return 1;
	}
	defer os.close(file);
	size: i64;
	size, error = os.file_size(file);
	if error != nil
	{
		return 1;
	}
	if size == 0
	{
		fmt.fprintln(file, "case_status,metric_status,worker_count,processor_affinity,bodies,constraints,batches,fallback_manifolds,contact_input,contact_points,point_updates,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,call_p50_ms,call_p95_ms,call_max_ms,ms_per_1000_point_updates,physics_status,quality_coverage,quality_failures,residual_max_mps,residual_rms_mps,residual_p95_mps,negative_impulse_max,friction_violation_max,twist_violation_max,pool_capacity_before,pool_capacity_after,worker_pool_capacity_before,worker_pool_capacity_after,precision,benchmark_parameters"+support.RECORDING_HEADER);
	}
	updates: i64 = contacts.logical_point_updates(input, MEASURED_PASSES);
	fmt.fprintfln(file, "%s,%s,%d,%d,%d,%d,%d,%d,\"%s\",%d,%d,%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%s,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%s,\"%s\",,,off,native_step_sum",
		case_status, metric_status, o.workers, o.affinity, counts.bodies, counts.manifolds, counts.batches, counts.fallback_manifolds,
		o.input, counts.points, updates, MEASURED_PASSES, WARMUP_PASSES, m.elapsed_ms, m.elapsed_ms/MEASURED_PASSES,
		p50, p95, maximum, 1000*m.elapsed_ms/f64(updates), physics_status, m.completed, m.quality_failures,
		worst.residual_max_mps, worst.residual_rms_mps, worst.residual_p95_mps, worst.negative_impulse_max,
		worst.friction_violation_max, worst.twist_violation_max, m.pool_before, m.pool_after,
		m.worker_pool_before, m.worker_pool_after, util.PRECISION_NAME, parameters);
	fmt.printfln("CONTACT_WORK_COMPLETE samples=%d/%d solve=%v quality_failures=%d pool_before=%d pool_after=%d worker_pool_before=%d worker_pool_after=%d exit_code=%d",
		m.completed, MEASURED_PASSES, m.solve_status, m.quality_failures, m.pool_before, m.pool_after, m.worker_pool_before, m.worker_pool_after, exit_code);
	return exit_code;
}

work_run :: proc(o: Options) -> int
{
	input: contacts.Input;
	defer contacts.input_destroy(&input);
	status: contacts.Status = contacts.input_read(o.input, &input);
	if status != .Ok
	{
		fmt.eprintfln("CONTACT_WORK_INPUT_REJECT path=%s status=%v", o.input, status);
		return 2;
	}
	counts: contacts.Counts = contacts.input_counts(&input);
	scratch: []f64 = make([]f64, int(counts.points));
	defer delete(scratch);
	pool: util.Buffer_Pool;
	if util.buffer_pool_initialize(&pool, 65_536) != .Ok
	{
		return 3;
	}
	defer util.buffer_pool_dispose(&pool);
	replay: contacts.Replay;
	defer contacts.replay_destroy(&replay);
	if contacts.replay_create(&replay, &input, &pool) != .Ok
	{
		fmt.eprintln("CONTACT_WORK_REPLAY_CREATE_FAILED");
		return 3;
	}
	dispatcher: util.Thread_Dispatcher;
	if util.thread_dispatcher_initialize(&dispatcher, o.workers, 65_536) != .Ok
	{
		return 3;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	m: Measurement = measure(&replay, &dispatcher, scratch);
	return write_measurement(o, &input, counts, &m);
}

main :: proc()
{
	o: Options;
	status: support.Admission;
	o, status = parse_options(os.args[1:]);
	if status != .Ok
	{
		fmt.eprintln("CONTACT_WORK_REJECT requires --input=PATH --output=CSV --worker-count=N --processor-affinity=MASK");
		os.exit(2);
	}
	os.exit(work_run(o));
}
