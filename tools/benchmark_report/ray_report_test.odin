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
		if strings.contains(markdown, "## ray_elapsed_ms summary") && strings.contains(markdown, "divide by 100") &&
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
