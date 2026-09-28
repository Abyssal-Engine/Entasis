package world_lifecycle

import "core:fmt"
import "core:os"
import "core:time"
import e "entasis:entasis"
import support "../benchmark_support"

cycle :: proc(workers: int, geometry: ^Geometry) -> Result
{
	owner: Owner;
	result: Result;
	desc: e.World_Description = world_description(workers);
	initialization_statuses: [2]e.Status;
	population_statuses: [BODY_COUNT*2+2]e.Status;
	cooking_statuses: [6]e.Status;
	destruction_statuses: [5]e.Status;
	clear_status: e.Status;
	start: time.Tick = time.tick_now();
	initialize(&owner, desc, &initialization_statuses);
	result.elapsed[0] = f64(time.tick_diff(start, time.tick_now()))/1e6;
	for status in initialization_statuses
	{
		support.extension_require(status, "lifecycle initialization");
	}

	start = time.tick_now();
	populate(&owner, &population_statuses);
	result.elapsed[1] = f64(time.tick_diff(start, time.tick_now()))/1e6;
	for status in population_statuses
	{
		support.extension_require(status, "lifecycle growth");
	}
	support.extension_require(validate_world(&owner), "lifecycle growth/reuse coverage");

	start = time.tick_now();
	import_cooked(&owner, geometry, &cooking_statuses);
	result.elapsed[2] = f64(time.tick_diff(start, time.tick_now()))/1e6;
	for status in cooking_statuses
	{
		support.extension_require(status, "lifecycle cooking/import");
	}
	measure_pools(&owner, &result);
	if owner.cooking.live_assets != 3
	{
		panic("cooking coverage");
	}

	start = time.tick_now();
	clear_status = clear_repopulate(&owner, &population_statuses);
	result.elapsed[3] = f64(time.tick_diff(start, time.tick_now()))/1e6;
	support.extension_require(clear_status, "lifecycle clear");
	for status in population_statuses
	{
		support.extension_require(status, "lifecycle reuse");
	}
	support.extension_require(validate_world(&owner), "lifecycle growth/reuse coverage");
	measure_pools(&owner, &result);

	start = time.tick_now();
	destroy(&owner, &destruction_statuses);
	result.elapsed[4] = f64(time.tick_diff(start, time.tick_now()))/1e6;
	for status in destruction_statuses
	{
		support.extension_require(status, "lifecycle destruction");
	}
	if owner.world != nil || owner.cooking.live_assets != 0 || owner.cooking.state != .Disposed
	{
		panic("lifecycle teardown state");
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
		fmt.eprintln("usage: world_lifecycle --worker-count=<n> --output=<path>");
		os.exit(2);
	}
	// geometry preparation is not part of the lifecycle timings
	geometry: Geometry = fixture_geometry();
	_ = cycle(args.workers, &geometry);
	samples: [CYCLES]Result;
	for &sample in samples
	{
		sample = cycle(args.workers, &geometry);
	}
	totals: [5]f64;
	total: f64;
	peaks: [3]u64;
	for sample in samples
	{
		for elapsed, i in sample.elapsed
		{
			if elapsed <= 0
			{
				panic("invalid lifecycle timing");
			}
			totals[i] += elapsed;
			total += elapsed;
		}
		peaks[0] = max(peaks[0], sample.world_native_bytes);
		peaks[1] = max(peaks[1], sample.worker_native_bytes);
		peaks[2] = max(peaks[2], sample.cooking_native_bytes);
	}
	file: ^os.File;
	csv_state: support.Extension_CSV_State;
	file, csv_state = support.extension_csv_open(args.output);
	if csv_state == .Empty
	{
		if fmt.fprint(file, "worker_count,cycles,body_count,case_status,metric_status,physics_elapsed_ms,world_native_bytes,worker_native_bytes,cooking_native_bytes") <= 0
		{
			panic("CSV header");
		}
		for name in COMPONENTS
		{
			if fmt.fprintf(file, ",%s_ms", name) <= 0
			{
				panic("CSV header");
			}
		}
		// retain bounded per-cycle samples, not only their aggregate
		for i in 0..<CYCLES
		{
			for name in COMPONENTS
			{
				if fmt.fprintf(file, ",cycle%d_%s_ms", i, name) <= 0
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
	if fmt.fprintf(file, "%d,%d,%d,ok,ok,%.9f,%d,%d,%d", args.workers, CYCLES, BODY_COUNT, total, peaks[0], peaks[1], peaks[2]) <= 0
	{
		panic("CSV row");
	}
	for value in totals
	{
		if fmt.fprintf(file, ",%.9f", value) <= 0
		{
			panic("CSV row");
		}
	}
	for sample in samples
	{
		for value in sample.elapsed
		{
			if fmt.fprintf(file, ",%.9f", value) <= 0
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
