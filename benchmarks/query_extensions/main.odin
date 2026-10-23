package query_extensions

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:time"
import e "entasis:entasis"
import support "../benchmark_support"

main :: proc()
{
	args: Query_Arguments;
	admission: support.Admission;
	args, admission = query_arguments(os.args);
	if admission != .Ok
	{
		fmt.eprintln("usage: query_extensions --worker-count=<n> --output=<path>");
		os.exit(2);
	}
	if args.query_case != "control"
	{
		if args.query_case == "collision-batch"
		{
			support.extension_require(run_collision_batch_case(args), "collision batch case");
		}
		else if args.query_operation == "closest-point" || args.query_operation == "distance" || args.query_operation == "depenetration" ||
			args.query_operation == "sweep-any" || args.query_operation == "sweep-closest" || args.query_operation == "sweep-all"
		{
			support.extension_require(run_hierarchy_geometry_case(args), "hierarchy geometry case");
		}
		else if args.query_case == "hierarchy"
		{
			support.extension_require(run_hierarchy_case(args), "hierarchy case");
		}
		else
		{
			when e.PRECISION_NAME == "mixed64"
			{
				support.extension_require(run_global_case(args), "global ray case");
			}
		}
		return;
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
		if len(args.query_group) > 0
		{
			break;
		}
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
	selected_hits: u64;
	selected_checksum: f64;
	if len(args.query_group) > 0
	{
		scene: Scene;
		support.extension_require(scene_init(&scene, args.workers, args.route), "query scene create");
		if args.operation >= len(OPERATIONS)
		{
			support.extension_require(e.distance_query_reserve(&scene.world, {512, 1024, 1536}), "reserve distance scratch");
		}
		for chunk in 0..<args.warmup_repetitions+args.repetitions
		{
			count: int = WARMUP_COUNT if chunk < args.warmup_repetitions else QUERY_COUNT;
			duration: f64;
			hits: u64;
			checksum: f64;
			validation: e.Status;
			if args.operation < len(OPERATIONS)
			{
				start: time.Tick = time.tick_now();
				run_operation(&scene, args.operation, samples[:count], .Discard);
				duration = f64(time.tick_diff(start, time.tick_now()))/1e6;
				hits, checksum, validation = validate(samples[:count], args.operation, args.route);
			}
			else
			{
				when support.BENCHMARK_COMPONENTS == "all"
				{
					start: time.Tick = time.tick_now();
					run_distance_operation(&scene, args.operation-len(OPERATIONS), samples[:count], .Discard);
					duration = f64(time.tick_diff(start, time.tick_now()))/1e6;
					hits, checksum, validation = validate_distance(samples[:count], args.operation-len(OPERATIONS), args.route);
				}
			}
			support.extension_require(validation, "selected query chunk coverage/result");
			if chunk >= args.warmup_repetitions
			{
				if duration <= 0
				{
					panic("invalid selected query timing");
				}
				total += duration;
				selected_hits += hits;
				selected_checksum += checksum;
			}
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
			if len(args.query_group) > 0
			{
				break;
			}
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
		if len(args.query_group) > 0 && fmt.fprint(file, ",precision,query_group,query_repetitions,query_warmup_repetitions") <= 0
		{
			panic("CSV header");
		}
		if len(args.query_group) == 0 && fmt.fprint(file, ",precision") <= 0
		{
			panic("CSV header");
		}
		if fmt.fprintln(file) <= 0
		{
			panic("CSV header");
		}
	}
	if fmt.fprintf(file, "%d,%d,ok,ok,%.9f,%s", args.workers, u64(QUERY_COUNT)*u64(args.repetitions), total, support.BENCHMARK_COMPONENTS) <= 0
	{
		panic("CSV row");
	}
	if len(args.query_group) > 0
	{
		for route in 0..<len(ROUTES)
		{
			for operation in 0..<len(OPERATIONS)
			{
				if route == args.route && operation == args.operation
				{
					if fmt.fprintf(file, ",%.9f,%d,%.9f", total, selected_hits, selected_checksum) <= 0
					{
						panic("CSV row");
					}
				}
				else if fmt.fprint(file, ",excluded,excluded,excluded") <= 0
				{
					panic("CSV row");
				}
			}
		}
		if fmt.fprint(file, ",Excluded") <= 0
		{
			panic("CSV row");
		}
		when support.BENCHMARK_COMPONENTS == "all"
		{
			if fmt.fprint(file, ",excluded") <= 0
			{
				panic("CSV row");
			}
			for route in 0..<len(ROUTES)
			{
				for operation in 0..<len(DISTANCE_OPERATIONS)
				{
					if route == args.route && operation+len(OPERATIONS) == args.operation
					{
						if fmt.fprintf(file, ",%.9f,%d,%.9f", total, selected_hits, selected_checksum) <= 0
						{
							panic("CSV row");
						}
					}
					else if fmt.fprint(file, ",excluded,excluded,excluded") <= 0
					{
						panic("CSV row");
					}
				}
			}
		}
		if fmt.fprintf(file, ",%s,%s,%d,%d\n", e.PRECISION_NAME, args.query_group, args.repetitions, args.warmup_repetitions) <= 0
		{
			panic("CSV row");
		}
	}
	else
	{
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
	}
	if len(args.query_group) == 0 && fmt.fprintf(file, ",%s\n", e.PRECISION_NAME) <= 0
	{
		panic("CSV row");
	}
	support.extension_csv_close(file);
}
