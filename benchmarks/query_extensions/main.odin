package query_extensions

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:time"
import e "entasis:entasis"
import support "../benchmark_support"

main :: proc()
{
	args: support.Extension_Arguments;
	admission: support.Admission;
	args, admission = support.extension_arguments(os.args);
	if admission != .Ok
	{
		fmt.eprintln("usage: query_extensions --worker-count=<n> --output=<path>");
		os.exit(2);
	}
	samples: []Sample;
	err: runtime.Allocator_Error;
	samples, err = make([]Sample, QUERY_COUNT);
	if err != nil
	{
		panic("sample storage");
	}
	defer delete(samples);
	elapsed: [4][8]f64;
	hit_counts: [4][8]u64;
	checksums: [4][8]f64;
	total: f64;
	for route in 0..<len(ROUTES)
	{
		scene: Scene;
		support.extension_require(scene_init(&scene, args.workers, route), "query scene create");
		for op in 0..<len(OPERATIONS)
		{
			run_operation(&scene, op, samples[:WARMUP_COUNT], .Discard);
			_, _, validation := validate(samples[:WARMUP_COUNT], op, route);
			support.extension_require(validation, "query warmup coverage/result");
			start: time.Tick = time.tick_now();
			run_operation(&scene, op, samples, .Discard);
			elapsed[route][op] = f64(time.tick_diff(start, time.tick_now())) / 1e6;
			hit_counts[route][op], checksums[route][op], validation = validate(samples, op, route);
			support.extension_require(validation, "query coverage/result");
			if elapsed[route][op] <= 0
			{
				panic("invalid query timing");
			}
			total += elapsed[route][op];
		}
		stats: e.World_Stats;
		status: e.Status;
		stats, status = e.world_stats(&scene.world);
		support.extension_require(status, "query stats");
		if stats.statics != 1 || stats.active_bodies != 0
		{
			panic("query world changed");
		}
		support.extension_require(e.world_destroy(&scene.world), "query world destroy");
	}
	when support.BENCHMARK_COMPONENTS == "all"
	{
		distance_elapsed: [4][4]f64;
		distance_counts: [4][4]u64;
		distance_checksums: [4][4]f64;
		distance_total: f64;
		for route in 0..<len(ROUTES)
		{
			scene: Scene;
			support.extension_require(scene_init(&scene, args.workers, route), "query scene create");
			support.extension_require(e.distance_query_reserve(&scene.world, {512, 1024, 1536}), "reserve distance scratch");
			for op in 0..<len(DISTANCE_OPERATIONS)
			{
				run_distance_operation(&scene, op, samples[:WARMUP_COUNT], .Discard);
				_, _, validation := validate_distance(samples[:WARMUP_COUNT], op, route);
				support.extension_require(validation, "distance warmup coverage/result");
				start: time.Tick = time.tick_now();
				run_distance_operation(&scene, op, samples, .Discard);
				distance_elapsed[route][op] = f64(time.tick_diff(start, time.tick_now()))/1e6;
				distance_counts[route][op], distance_checksums[route][op], validation = validate_distance(samples, op, route);
				support.extension_require(validation, "distance coverage/result");
				if distance_elapsed[route][op] <= 0
				{
					panic("invalid distance timing");
				}
				distance_total += distance_elapsed[route][op];
			}
			support.extension_require(e.world_destroy(&scene.world), "distance world destroy");
		}
	}
	file: ^os.File;
	csv_state: support.Extension_CSV_State;
	file, csv_state = support.extension_csv_open(args.output);
	if csv_state == .Empty
	{
		if fmt.fprint(file, "worker_count,query_count_per_operation,case_status,metric_status,physics_elapsed_ms,benchmark_components") <= 0
		{
			panic("CSV header");
		}
		for route in ROUTES
		{
			for op in OPERATIONS
			{
				if fmt.fprintf(file, ",%s_%s_ms,%s_%s_hits,%s_%s_checksum", route, op, route, op, route, op) <= 0
				{
					panic("CSV header");
				}
			}
		}
		if fmt.fprint(file, ",cq01_state") <= 0
		{
			panic("CSV header");
		}
		when support.BENCHMARK_COMPONENTS == "all"
		{
			if fmt.fprint(file, ",cq01_elapsed_ms") <= 0
			{
				panic("CSV header");
			}
			for route in ROUTES
			{
				for op in DISTANCE_OPERATIONS
				{
					if fmt.fprintf(file, ",%s_%s_ms,%s_%s_hits,%s_%s_checksum", route, op, route, op, route, op) <= 0
					{
						panic("CSV header");
					}
				}
			}
		}
		if fmt.fprintln(file) <= 0
		{
			panic("CSV header");
		}
	}
	if fmt.fprintf(file, "%d,%d,ok,ok,%.9f,%s", args.workers, QUERY_COUNT, total, support.BENCHMARK_COMPONENTS) <= 0
	{
		panic("CSV row");
	}
	for route in 0..<4
	{
		for op in 0..<8
		{
			if fmt.fprintf(file, ",%.9f,%d,%.9f", elapsed[route][op], hit_counts[route][op], checksums[route][op]) <= 0
			{
				panic("CSV row");
			}
		}
	}
	when support.BENCHMARK_COMPONENTS == "all"
	{
		if fmt.fprint(file, ",Available") <= 0
		{
			panic("CSV row");
		}
		if fmt.fprintf(file, ",%.9f", distance_total) <= 0
		{
			panic("CSV row");
		}
		for route in 0..<4
		{
			for op in 0..<4
			{
				if fmt.fprintf(file, ",%.9f,%d,%.9f", distance_elapsed[route][op], distance_counts[route][op], distance_checksums[route][op]) <= 0
				{
					panic("CSV row");
				}
			}
		}
	}
	else
	{
		if fmt.fprint(file, ",Unavailable") <= 0
		{
			panic("CSV row");
		}
	}
	if fmt.fprintln(file) <= 0
	{
		panic("CSV row");
	}
	support.extension_csv_close(file);
}
