package custom_extensions

import "core:fmt"
import "core:os"
import "core:time"
import e "entasis:entasis"
import support "../benchmark_support"

run_mode :: proc(workers, mode: int) -> Result
{
	// a separate disposable scene warms code, not the measured world's state
	warmup: Owner;
	support.extension_require(owner_init(&warmup, workers, mode), "custom warmup create");
	for _ in 0..<WARMUP_STEPS
	{
		support.extension_require(e.world_step(&warmup.world, DT), "custom warmup step");
	}
	_, warmup_status := validate_owner(&warmup, mode);
	support.extension_require(warmup_status, "custom warmup validation");
	support.extension_require(e.world_destroy(&warmup.world), "custom warmup destroy");
	owner: Owner;
	support.extension_require(owner_init(&owner, workers, mode), "custom measured create");
	statuses: [MEASURED_STEPS]e.Status;
	start: time.Tick = time.tick_now();
	for &status in statuses
	{
		status = e.world_step(&owner.world, DT);
	}
	elapsed: f64 = f64(time.tick_diff(start, time.tick_now())) / 1e6;
	for status in statuses
	{
		support.extension_require(status, "custom measured step");
	}
	result: Result;
	validation_status: e.Status;
	result, validation_status = validate_owner(&owner, mode);
	support.extension_require(validation_status, "custom measured validation");
	result.elapsed = elapsed;
	support.extension_require(e.world_destroy(&owner.world), "custom measured destroy");
	if elapsed <= 0
	{
		panic("invalid custom timing");
	}
	return result;
}

main :: proc()
{
	args: support.Extension_Arguments;
	admission: support.Admission;
	args, admission = support.extension_arguments(os.args);
	if admission != .Ok
	{
		fmt.eprintln("usage: custom_extensions --worker-count=<n> --output=<path>");
		os.exit(2);
	}
	results: [5]Result;
	total: f64;
	for &result, mode in results
	{
		result = run_mode(args.workers, mode);
		total += result.elapsed;
	}
	when support.BENCHMARK_COMPONENTS == "all"
	{
		body_control_results: [5]Body_Control_Result = measure_body_control_components(args.workers);
		joint_break_results: [5]Joint_Break_Component = measure_joint_break_components(args.workers);
	}
	file: ^os.File;
	csv_state: support.Extension_CSV_State;
	file, csv_state = support.extension_csv_open(args.output);
	if csv_state == .Empty
	{
		if fmt.fprint(file, "worker_count,body_count,measured_steps,case_status,metric_status,physics_elapsed_ms,benchmark_components") <= 0
		{
			panic("CSV header");
		}
		for mode in MODES
		{
			if fmt.fprintf(file, ",%s_ms,%s_active_bodies,%s_constraints,%s_fallback,%s_contacts", mode, mode, mode, mode, mode) <= 0
			{
				panic("CSV header");
			}
		}
		when support.BENCHMARK_COMPONENTS == "all"
		{
			if fmt.fprint(file, ",bc01_state,bc01_bodies,bc01_measured_steps") <= 0
			{
				panic("CSV header");
			}
			for name in BODY_CONTROL_NAMES
			{
				if fmt.fprintf(file, ",bc01_%s_submission_ms,bc01_%s_step_ms,bc01_%s_submissions,bc01_%s_active_bodies,bc01_%s_constraints,bc01_%s_position_checksum,bc01_%s_velocity_checksum,bc01_%s_world_pool_bytes,bc01_%s_worker_pool_bytes",
					name, name, name, name, name, name, name, name, name) <= 0
				{
					panic("CSV header");
				}
			}
			if fmt.fprint(file, ",bj01_state,bj01_pairs,bj01_measured_steps") <= 0
			{
				panic("CSV header");
			}
			for name in JOINT_BREAK_NAMES
			{
				if fmt.fprintf(file, ",bj01_%s_step_ms,bj01_%s_drain_ms,bj01_%s_rebuild_ms,bj01_%s_watches,bj01_%s_events,bj01_%s_constraints,bj01_%s_maximum_force,bj01_%s_world_pool_bytes,bj01_%s_worker_pool_bytes",
					name, name, name, name, name, name, name, name, name) <= 0
				{
					panic("CSV header");
				}
			}
		}
		if fmt.fprintln(file) <= 0
		{
			panic("CSV header");
		}
	}
	if fmt.fprintf(file, "%d,%d,%d,ok,ok,%.9f,%s", args.workers, BODY_COUNT, MEASURED_STEPS, total, support.BENCHMARK_COMPONENTS) <= 0
	{
		panic("CSV row");
	}
	for result in results
	{
		if fmt.fprintf(file, ",%.9f,%d,%d,%d,%d", result.elapsed, result.active_bodies, result.custom_constraints, result.fallback_constraints, result.contact_pairs) <= 0
		{
			panic("CSV row");
		}
	}
	when support.BENCHMARK_COMPONENTS == "all"
	{
		if fmt.fprintf(file, ",Available,%d,%d", BODY_COUNT, MEASURED_STEPS) <= 0
		{
			panic("CSV row");
		}
		for result in body_control_results
		{
			if fmt.fprintf(file, ",%.9f,%.9f,%d,%d,%d,%.9f,%.9f,%d,%d", result.submission_ms, result.step_ms,
				result.submissions, result.active_bodies, result.active_constraints, result.position_checksum, result.velocity_checksum,
				result.world_pool_bytes, result.worker_pool_bytes) <= 0
			{
				panic("CSV row");
			}
		}
		if fmt.fprintf(file, ",Available,%d,%d", JOINT_BREAK_PAIR_COUNT, MEASURED_STEPS) <= 0
		{
			panic("CSV row");
		}
		for result in joint_break_results
		{
			if fmt.fprintf(file, ",%.9f,%.9f,%.9f,%d,%d,%d,%.9f,%d,%d", result.step_ms, result.drain_ms, result.rebuild_ms,
				result.watches, result.events, result.remaining_constraints, result.maximum_force, result.world_pool_bytes, result.worker_pool_bytes) <= 0
			{
				panic("CSV row");
			}
		}
	}
	if fmt.fprintln(file) <= 0
	{
		panic("CSV row");
	}
	support.extension_csv_close(file);
}
