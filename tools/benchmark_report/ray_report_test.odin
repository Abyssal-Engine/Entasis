package benchmark_report

import "core:fmt"
import "core:mem"
import "core:strings"
import "core:testing"

@(test)
ray_report_uses_native_timing_and_rejects_mixed_workloads :: proc(t: ^testing.T)
{
	test_allocator := context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	path, directory_status := test_directory("entasis-benchmark-report-ray")
	header :: "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status\n"
	first_write: Report_Status = write_lane(path, 1, header + "1,256,50000,10,100,25000,42,100,ok\n1,256,50000,10,100,25000,42,300,ok\n")
	second_write: Report_Status = write_lane(path, 2, header + "2,256,50000,10,100,25000,42,80,ok\n2,256,50000,10,100,25000,42,120,ok\n")
	request: Report_Request = test_request(path, []int{1, 2}, 2)
	request.package_name = "spatial_query_batch"
	result: Report_Result = collect_summaries(request)
	median: f64
	speedup: f64
	metric_label: Test_Flag
	if result.status == .Ok
	{
		median = result.summaries[1].median
		speedup = result.summaries[1].speedup
		markdown: string = render_markdown(request, result.summaries[:])
		if strings.contains(markdown, "## Closest-ray time summary") && strings.contains(markdown, "ms/frame (50,000 rays)") &&
			strings.contains(markdown, "| 2 | 2 | 1.000000 | 0.800000 | 1.200000 | 2.000000x |") &&
			strings.contains(markdown, "Public query_batch, batch size 256")
		{
			metric_label = .Enabled
		}
	}

	invalid_rows: [9]string = {
		"1,256,50000,10,100,25000,42,80,ok", // wrong worker lane
		"2,256,49999,10,100,25000,42,80,ok", // wrong ray count
		"2,128,50000,10,100,25000,42,80,ok", // wrong batch size
		"2,256,50000,10,99,25000,42,80,ok", // wrong measured count
		"2,256,50000,10,100,24999,42,80,ok", // missing hit
		"2,256,50000,10,100,25000,43,80,ok", // changed checksum within lane
		"2,256,50000,10,100,25000,42,NaN,ok",
		"2,256,50000,10,100,25000,42,0,ok",
		"2,256,50000,10,100,25000,42,80,failed",
	}
	rejected: int
	for row in invalid_rows
	{
		_ = write_lane(path, 2, fmt.aprintf("%s2,256,50000,10,100,25000,42,120,ok\n%s\n", header, row))
		if collect_summaries(request).status != .Ok
		{
			rejected += 1
		}
	}
	_ = write_lane(path, 2, header + "2,256,50000,10,100,25000,43,80,ok\n2,256,50000,10,100,25000,43,120,ok\n")
	cross_lane_status: Report_Status = collect_summaries(request).status
	_ = write_lane(path, 2, "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status,extra\n2,256,50000,10,100,25000,42,80,ok,0\n2,256,50000,10,100,25000,42,120,ok,0\n")
	extra_column_status: Report_Status = collect_summaries(request).status
	_ = write_lane(path, 2, "case_status,metric_status,physics_elapsed_ms\nok,ok,80\nok,ok,120\n")
	header_status: Report_Status = collect_summaries(request).status
	_ = write_lane(path, 1, header+"1,256,50000,2,3,25000,42,6,ok\n1,256,50000,2,3,25000,42,12,ok\n");
	_ = write_lane(path, 2, header+"2,256,50000,2,3,25000,42,3,ok\n2,256,50000,2,3,25000,42,6,ok\n");
	configured: Report_Result = collect_summaries(request);
	testing.expect_value(t, configured.status, Report_Status.Ok);
	if configured.status == .Ok
	{
		testing.expect_value(t, configured.summaries[0].query.measured_batches, 3);
		testing.expect(t, strings.contains(render_markdown(request, configured.summaries[:]), "Warmup batches: 2 | Measured batches: 3"));
	}
	_ = write_lane(path, 2, header+"2,256,50000,3,3,25000,42,3,ok\n2,256,50000,3,3,25000,42,6,ok\n");
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);

	context.allocator = test_allocator
	test_directory_remove(path)
	mem.dynamic_arena_destroy(&arena)
	testing.expect_value(t, directory_status, Report_Status.Ok)
	testing.expect_value(t, first_write, Report_Status.Ok)
	testing.expect_value(t, second_write, Report_Status.Ok)
	testing.expect_value(t, result.status, Report_Status.Ok)
	testing.expect_value(t, median, f64(100))
	testing.expect_value(t, speedup, f64(2))
	testing.expect_value(t, metric_label, Test_Flag.Enabled)
	testing.expect_value(t, rejected, len(invalid_rows))
	testing.expect_value(t, cross_lane_status, Report_Status.Invalid_Measurement)
	testing.expect_value(t, header_status, Report_Status.Invalid_Header)
	testing.expect_value(t, extra_column_status, Report_Status.Invalid_Header)
}

@(test)
spatial_query_reports_reject_mixed_ray_patterns :: proc(t: ^testing.T)
{
	test_allocator := context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	path, directory_status := test_directory("entasis-benchmark-report-ray-pattern");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	for package_name in ([]string{"spatial_query_batch", "spatial_query_trace"})
	{
		header: string = "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status";
		row: string = "256,50000,10,100,25000,42,100,ok";
		if package_name == "spatial_query_trace"
		{
			header = "worker_count,query_count,warmup_batches,measured_batches,physics_elapsed_ms,case_status,metric_status";
			row = "100000,10,100,100,ok,ok";
		}
		historical_header, historical_row: string = header, row;
		header = fmt.aprintf("%s,precision", header);
		row = fmt.aprintf("%s,float32", row);
		request: Report_Request = test_request(path, []int{1, 2}, 2);
		request.package_name = package_name;
		dataset: Result_Dataset;
		for pattern in ([]string{"axial", "oblique"})
		{
			for worker in request.workers
			{
				_ = write_lane(path, worker, fmt.aprintf("%s,ray_pattern\n%d,%s,%s\n%d,%s,%s\n", header, worker, row, pattern, worker, row, pattern));
			}
			result: Report_Result = collect_summaries(request);
			testing.expect_value(t, result.status, Report_Status.Ok);
			if result.status == .Ok
			{
				testing.expect(t, strings.contains(render_markdown(request, result.summaries[:]), fmt.tprintf("Ray pattern: %s", pattern)));
			}
			testing.expect_value(t, result_dataset_open(&dataset, path, package_name).status, Report_Status.Ok);
			result_dataset_close(&dataset);
		}
		// use identical checksums so distribution admission owns these failures
		for patterns in ([][2]string{{"axial", "oblique"}, {"", ""}, {"diagonal", "diagonal"}})
		{
			_ = write_lane(path, 2, fmt.aprintf("%s,ray_pattern\n2,%s,%s\n2,%s,%s\n", header, row, patterns[0], row, patterns[1]));
			testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
			testing.expect_value(t, result_dataset_open(&dataset, path, package_name).status, Report_Status.Invalid_Measurement);
			result_dataset_close(&dataset);
		}
		_ = write_lane(path, 2, fmt.aprintf("%s,ray_pattern\n2,%s,axial\n2,%s,axial\n", header, row, row));
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
		testing.expect_value(t, result_dataset_open(&dataset, path, package_name).status, Report_Status.Invalid_Measurement);
		result_dataset_close(&dataset);
		for worker in request.workers
		{
			_ = write_lane(path, worker, fmt.aprintf("%s\n%d,%s\n%d,%s\n", historical_header, worker, historical_row, worker, historical_row));
		}
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Ok);
		testing.expect_value(t, result_dataset_open(&dataset, path, package_name).status, Report_Status.Ok);
		result_dataset_close(&dataset);
		if package_name == "spatial_query_batch"
		{
			_ = write_lane(path, 2, fmt.aprintf("%s,ray_pattern,extra\n2,%s,axial,0\n2,%s,axial,0\n", header, row, row));
			testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Header);
			testing.expect_value(t, result_dataset_open(&dataset, path, package_name).status, Report_Status.Invalid_Header);
			result_dataset_close(&dataset);
		}
	}
	selected_header: string = "worker_count,query_count,warmup_batches,measured_batches,case_status,metric_status,physics_elapsed_ms,query_family,ray_query_count,sphere_cast_query_count,overlap_query_count,ray_hit_count,sphere_cast_hit_count,overlap_hit_count,ray_checksum,sphere_cast_checksum,overlap_checksum,ray_elapsed_ms,sphere_cast_elapsed_ms,overlap_elapsed_ms,ray_queries_per_second,sphere_cast_queries_per_second,overlap_queries_per_second";
	selected_row: string = "25000,2,3,ok,ok,6,aabb-presence,excluded,excluded,25000,excluded,excluded,12500,excluded,excluded,42,excluded,excluded,6,excluded,excluded,12500000";
	for worker in 1..=2
	{
		_ = write_lane(path, worker, fmt.aprintf("%s\n%d,%s\n", selected_header, worker, selected_row));
	}
	selected_request: Report_Request = test_request(path, {1, 2}, 1);
	selected_request.package_name = "spatial_query_trace";
	selected_request.query = {family="aabb-presence", warmup_batches=2, measured_batches=3};
	selected: Report_Result = collect_summaries(selected_request);
	testing.expect_value(t, selected.status, Report_Status.Ok);
	dataset: Result_Dataset;
	testing.expect_value(t, result_dataset_open(&dataset, path, "spatial_query_trace").status, Report_Status.Ok);
	result_dataset_close(&dataset);
	if selected.status == .Ok
	{
		testing.expect_value(t, selected.summaries[0].query_count, 25000);
		testing.expect(t, strings.contains(render_markdown(selected_request, selected.summaries[:]), "Query family: aabb-presence"));
	}
	for pair in ([][2]string{{"aabb-presence", "all"}, {"excluded,excluded,25000", "0,excluded,25000"}, {"25000,2,3", "25000,3,3"}, {"25000,2,3", "25000,2,4"}, {"ok,ok,6", "ok,ok,5"}})
	{
		changed: string;
		changed, _ = strings.replace_all(selected_row, pair[0], pair[1]);
		_ = write_lane(path, 2, fmt.aprintf("%s\n2,%s\n", selected_header, changed));
		testing.expect(t, collect_summaries(selected_request).status != .Ok);
		testing.expect(t, result_dataset_open(&dataset, path, "spatial_query_trace").status != .Ok);
		result_dataset_close(&dataset);
	}

	context.allocator = test_allocator;
	test_directory_remove(path);
	mem.dynamic_arena_destroy(&arena);
}

@(test)
spatial_query_reports_reject_mixed_execution_modes :: proc(t: ^testing.T)
{
	original: mem.Allocator = context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	path, status := test_directory("entasis-query-execution");
	testing.expect_value(t, status, Report_Status.Ok);
	request: Report_Request = test_request(path, {1, 2}, 1);
	request.package_name = "spatial_query_batch";
	header :: "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status";
	dataset: Result_Dataset;
	for mode in ([]string{"caller", "mailbox", "legacy"})
	{
		for worker in request.workers
		{
			identity: string = ",query_execution";
			value: string = fmt.aprintf(",%s", mode);
			if mode == "legacy"
			{
				identity = "";
				value = "";
			}
			_ = write_lane(path, worker, fmt.aprintf("%s%s\n%d,%d,50000,10,100,25000,42,100,ok%s\n", header, identity,
				worker, 256, value));
		}
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Ok);
		testing.expect_value(t, result_dataset_open(&dataset, path, request.package_name).status, Report_Status.Ok);
		result_dataset_close(&dataset);
	}
	// historical caller and prepared caller have distinct timer identities
	_ = write_lane(path, 2, header+",query_execution\n2,256,50000,10,100,25000,42,100,ok,caller\n");
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	testing.expect_value(t, result_dataset_open(&dataset, path, request.package_name).status, Report_Status.Invalid_Measurement);
	result_dataset_close(&dataset);
	_ = write_lane(path, 1, header+",query_execution\n1,256,50000,10,100,25000,42,100,ok,caller\n");
	for execution in ([]string{"mailbox", "invalid", ""})
	{
		_ = write_lane(path, 2, fmt.aprintf("%s,query_execution\n2,256,50000,10,100,25000,42,100,ok,%s\n", header, execution));
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
		testing.expect_value(t, result_dataset_open(&dataset, path, request.package_name).status, Report_Status.Invalid_Measurement);
		result_dataset_close(&dataset);
	}
	_ = write_lane(path, 2, header+",query_execution\n2,256,50000,10,100,25000,42,100,ok,caller\n");
	request.query.execution = "mailbox";
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	args: []string = {"report", "--source", path, "--package", "spatial_query_batch", "--configuration", "release",
		"--runs", "1", "--workers", "1,2", "--compiler-version", "test", "--operating-system", "test", "--processor", "test",
		"--logical-processors", "2", "--command", "test", "--query-execution", "mailbox"};
	_, cli_status := parse_cli(args);
	testing.expect_value(t, cli_status, Report_Status.Ok);
	args[len(args)-1] = "bad";
	_, cli_status = parse_cli(args);
	testing.expect_value(t, cli_status, Report_Status.Invalid_Arguments);
	context.allocator = original;
	test_directory_remove(path);
	mem.dynamic_arena_destroy(&arena);
}
