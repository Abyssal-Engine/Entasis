package benchmark_report

import "core:fmt"
import "core:mem"
import "core:os"
import "core:path/filepath"
import "core:strings"
import "core:testing"

Test_Flag :: enum
{
	Disabled,
	Enabled,
}

test_directory_remove :: proc(path: string)
{
	known_files := []string{"workers-1.csv", "workers-2.csv", "workers-3.csv", "README.md", "README.md.tmp"}
	for name in known_files
	{
		file_path, join_error := filepath.join([]string{path, name})
		if join_error == nil
		{
			_ = os.remove(file_path)
			delete(file_path)
		}
	}
	_ = os.remove(path)
}

test_directory :: proc(name: string) -> (string, Report_Status)
{
	temporary_root, temp_error := os.temp_directory(context.allocator)
	if temp_error != nil
	{
		return "", .Report_Write_Failed
	}
	path, join_error := filepath.join([]string{temporary_root, name})
	if join_error != nil
	{
		return "", .Report_Write_Failed
	}
	test_directory_remove(path)
	if os.make_directory_all(path) != nil
	{
		return "", .Report_Write_Failed
	}
	return path, .Ok
}

test_request :: proc(path: string, workers: []int, run_count: int) -> Report_Request
{
	return Report_Request{
		source_directory = path,
		package_name = "box_container_pile_10k",
		configuration = "release",
		run_count = run_count,
		workers = workers,
		compiler_version = "dev-test",
		operating_system = "Test OS",
		processor = "Test CPU",
		logical_processors = 8,
		command = "./scripts/run_benchmark.sh --package box_container_pile_10k --runs 2 --workers 1,2 --configuration release",
		run_date = "2026-09-06",
	}
}

write_lane :: proc(path: string, worker_count: int, rows: string) -> Report_Status
{
	lane_path, join_error := filepath.join([]string{path, worker_file_name(worker_count)})
	if join_error != nil || os.write_entire_file(lane_path, rows) != nil
	{
		return .Report_Write_Failed
	}
	return .Ok
}

@(test)
valid_multi_lane_aggregation_and_deterministic_markdown :: proc(t: ^testing.T)
{
	test_allocator := context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	path, directory_status := test_directory("entasis-benchmark-report-valid")
	header := "case_status,metric_status,physics_elapsed_ms,package_value\n"
	first_write_status := write_lane(path, 1, fmt.aprintf("%sok,ok,10,a\nok,ok,30,b\n", header))
	second_write_status := write_lane(path, 2, fmt.aprintf("%sok,ok,8,c\nok,ok,22,d\n", header))
	workers := []int{1, 2}
	request := test_request(path, workers, 2)
	result := collect_summaries(request)
	summary_count := len(result.summaries)
	first_median, second_median := f64(0), f64(0)
	deterministic, expected_row := Test_Flag.Disabled, Test_Flag.Disabled
	if result.status == .Ok && summary_count == 2
	{
		first_median = result.summaries[0].median
		second_median = result.summaries[1].median
		first := render_markdown(request, result.summaries[:])
		second := render_markdown(request, result.summaries[:])
		if first == second
		{
			deterministic = .Enabled
		}
		if strings.contains(first, "| 2 | 2 | 15.000000 | 8.000000 | 22.000000 | 1.333333x |")
		{
			expected_row = .Enabled
		}
	}
	context.allocator = test_allocator
	test_directory_remove(path)
	mem.dynamic_arena_destroy(&arena)
	testing.expect_value(t, directory_status, Report_Status.Ok)
	testing.expect_value(t, first_write_status, Report_Status.Ok)
	testing.expect_value(t, second_write_status, Report_Status.Ok)
	testing.expect_value(t, result.status, Report_Status.Ok)
	testing.expect_value(t, summary_count, 2)
	testing.expect_value(t, first_median, f64(20))
	testing.expect_value(t, second_median, f64(15))
	testing.expect_value(t, deterministic, Test_Flag.Enabled)
	testing.expect_value(t, expected_row, Test_Flag.Enabled)
}

@(test)
compiler_metadata_omits_executable_paths :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	versions: [3]string = {
		`C:\Build Tools\Odin\odin.exe version dev-test:123abc`,
		"/opt/compiler/odin version dev-test:123abc",
		"dev-test:123abc",
	}
	for version in versions
	{
		request: Report_Request = test_request("", nil, 1)
		request.compiler_version = version
		markdown: string = render_markdown(request, {{worker_count=1, sample_count=1, median=1, minimum=1, maximum=1}})
		testing.expect(t, strings.contains(markdown, "- Odin compiler: `dev-test:123abc`\n"))
		testing.expect(t, !strings.contains(markdown, `C:\Build Tools`))
		testing.expect(t, !strings.contains(markdown, "/opt/compiler"))
	}
}

@(test)
even_and_odd_sample_median_and_range :: proc(t: ^testing.T)
{
	test_allocator := context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	path, directory_status := test_directory("entasis-benchmark-report-median")
	header := "case_status,metric_status,physics_elapsed_ms\n"
	even_write_status := write_lane(path, 1, fmt.aprintf("%sok,ok,4\nok,ok,1\nok,ok,3\nok,ok,2\n", header))
	lane_path, _ := filepath.join([]string{path, "workers-1.csv"})
	even_summary, even_result := summarize_csv(lane_path, 4, 1)
	odd_write_status := write_lane(path, 1, fmt.aprintf("%sok,ok,9\nok,ok,2\nok,ok,5\n", header))
	odd_summary, odd_result := summarize_csv(lane_path, 3, 1)
	context.allocator = test_allocator
	test_directory_remove(path)
	mem.dynamic_arena_destroy(&arena)
	testing.expect_value(t, directory_status, Report_Status.Ok)
	testing.expect_value(t, even_write_status, Report_Status.Ok)
	testing.expect_value(t, even_result.status, Report_Status.Ok)
	testing.expect_value(t, even_summary.median, f64(2.5))
	testing.expect_value(t, even_summary.minimum, f64(1))
	testing.expect_value(t, even_summary.maximum, f64(4))
	testing.expect_value(t, odd_write_status, Report_Status.Ok)
	testing.expect_value(t, odd_result.status, Report_Status.Ok)
	testing.expect_value(t, odd_summary.median, f64(5))
	testing.expect_value(t, odd_summary.minimum, f64(2))
	testing.expect_value(t, odd_summary.maximum, f64(9))
}

@(test)
missing_duplicate_unexpected_lanes_and_wrong_rows_are_rejected :: proc(t: ^testing.T)
{
	test_allocator := context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	missing_path, missing_directory_status := test_directory("entasis-benchmark-report-missing")
	unexpected_path, unexpected_directory_status := test_directory("entasis-benchmark-report-unexpected")
	rows_path, rows_directory_status := test_directory("entasis-benchmark-report-rows")
	header := "case_status,metric_status,physics_elapsed_ms\n"
	workers := []int{1, 2}
	missing_write_status := write_lane(missing_path, 1, fmt.aprintf("%sok,ok,1\n", header))
	missing_request := test_request(missing_path, workers, 1)
	missing_status := collect_summaries(missing_request).status
	_ = write_lane(unexpected_path, 1, fmt.aprintf("%sok,ok,1\n", header))
	unexpected_write_status := write_lane(unexpected_path, 3, fmt.aprintf("%sok,ok,1\n", header))
	unexpected_request := test_request(unexpected_path, workers, 1)
	unexpected_status := collect_summaries(unexpected_request).status
	_ = write_lane(rows_path, 1, fmt.aprintf("%sok,ok,1\n", header))
	row_write_status := write_lane(rows_path, 2, fmt.aprintf("%sok,ok,1\nok,ok,2\n", header))
	rows_request := test_request(rows_path, workers, 1)
	row_count_status := collect_summaries(rows_request).status
	duplicate_workers := []int{1, 1}
	duplicate_request := test_request(rows_path, duplicate_workers, 1)
	duplicate_status := collect_summaries(duplicate_request).status
	context.allocator = test_allocator
	test_directory_remove(missing_path)
	test_directory_remove(unexpected_path)
	test_directory_remove(rows_path)
	mem.dynamic_arena_destroy(&arena)
	testing.expect_value(t, missing_directory_status, Report_Status.Ok)
	testing.expect_value(t, unexpected_directory_status, Report_Status.Ok)
	testing.expect_value(t, rows_directory_status, Report_Status.Ok)
	testing.expect_value(t, missing_write_status, Report_Status.Ok)
	testing.expect_value(t, missing_status, Report_Status.Missing_Lane)
	testing.expect_value(t, unexpected_write_status, Report_Status.Ok)
	testing.expect_value(t, unexpected_status, Report_Status.Unexpected_Lane)
	testing.expect_value(t, row_write_status, Report_Status.Ok)
	testing.expect_value(t, row_count_status, Report_Status.Invalid_Row_Count)
	testing.expect_value(t, duplicate_status, Report_Status.Invalid_Arguments)
}

@(test)
required_columns_statuses_and_measurements_are_rejected :: proc(t: ^testing.T)
{
	test_allocator := context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	path, directory_status := test_directory("entasis-benchmark-report-values")
	workers := []int{1}
	request := test_request(path, workers, 1)
	missing_write_status := write_lane(path, 1, "case_status,metric_status\nok,ok\n")
	missing_header_status := collect_summaries(request).status
	duplicate_write_status := write_lane(path, 1, "case_status,case_status,metric_status,physics_elapsed_ms\nok,ok,ok,1\n")
	duplicate_header_status := collect_summaries(request).status
	status_write_status := write_lane(path, 1, "case_status,metric_status,physics_elapsed_ms\nfailed,ok,1\n")
	invalid_status := collect_summaries(request).status
	invalid_values := []string{"nan", "inf", "0", "-1", "invalid"}
	measurement_statuses: [5]Report_Status
	for invalid_value, index in invalid_values
	{
		rows := fmt.aprintf("case_status,metric_status,physics_elapsed_ms\nok,ok,%s\n", invalid_value)
		_ = write_lane(path, 1, rows)
		measurement_statuses[index] = collect_summaries(request).status
	}
	context.allocator = test_allocator
	test_directory_remove(path)
	mem.dynamic_arena_destroy(&arena)
	testing.expect_value(t, directory_status, Report_Status.Ok)
	testing.expect_value(t, missing_write_status, Report_Status.Ok)
	testing.expect_value(t, missing_header_status, Report_Status.Invalid_Header)
	testing.expect_value(t, duplicate_write_status, Report_Status.Ok)
	testing.expect_value(t, duplicate_header_status, Report_Status.Invalid_Header)
	testing.expect_value(t, status_write_status, Report_Status.Ok)
	testing.expect_value(t, invalid_status, Report_Status.Invalid_Status)
	for measurement_status in measurement_statuses
	{
		testing.expect_value(t, measurement_status, Report_Status.Invalid_Measurement)
	}
}

@(test)
required_cli_values_and_command_count_are_rejected :: proc(t: ^testing.T)
{
	blank_provenance_args := []string{
		"benchmark_report",
		"--source", "missing-source",
		"--package", "box_container_pile_10k",
		"--configuration", "release",
		"--runs", "1",
		"--workers", "1",
		"--compiler-version", "dev-test",
		"--operating-system", "",
		"--processor", "Test CPU",
		"--logical-processors", "8",
		"--command", "./scripts/run_benchmark.sh",
	}
	_, blank_status := parse_cli(blank_provenance_args)
	empty_first_command_args := []string{
		"benchmark_report",
		"--source", "missing-source",
		"--package", "box_container_pile_10k",
		"--configuration", "release",
		"--runs", "1",
		"--workers", "1",
		"--compiler-version", "dev-test",
		"--operating-system", "Test OS",
		"--processor", "Test CPU",
		"--logical-processors", "8",
		"--command", "",
		"--command", "./scripts/run_benchmark.sh",
	}
	_, duplicate_status := parse_cli(empty_first_command_args)
	testing.expect_value(t, blank_status, Report_Status.Invalid_Arguments)
	testing.expect_value(t, duplicate_status, Report_Status.Invalid_Arguments)
}

@(test)
parameterized_rows_require_matching_configuration :: proc(t: ^testing.T)
{
 arena: mem.Dynamic_Arena
 mem.dynamic_arena_init(&arena)
 defer mem.dynamic_arena_destroy(&arena)
 context.allocator = mem.dynamic_arena_allocator(&arena)
 path, _ := test_directory("entasis-parameter-rows")
 defer test_directory_remove(path)
 header := "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters\n"
 for rows in ([]string{
  "ok,ok,10,1,10,a\nok,ok,20,1,10,b\n",
  "ok,ok,10,1,10,a\nok,ok,20,1,11,a\n",
  "ok,ok,10,1,10,a\nok,ok,20,2,10,a\n",
  "ok,ok,10,1,10,\nok,ok,20,1,10,\n",
 })
 {
  testing.expect_value(t, write_lane(path, 1, fmt.aprintf("%s%s", header, rows)), Report_Status.Ok)
  request := test_request(path, {1}, 2)
  request.package_name = "container"
  request.variant = "box-default"
  result := collect_summaries(request)
  testing.expect_value(t, result.status, Report_Status.Invalid_Measurement)
 }
 testing.expect_value(t, write_lane(path, 1, "case_status,metric_status,physics_elapsed_ms\nok,ok,10\n"), Report_Status.Ok)
 request := test_request(path, {1}, 1)
 request.package_name = "container"
 request.variant = "box-default"
 result := collect_summaries(request)
 testing.expect_value(t, result.status, Report_Status.Invalid_Header)
}

@(test)
parameterized_lanes_require_matching_configuration :: proc(t: ^testing.T)
{
 arena: mem.Dynamic_Arena
 mem.dynamic_arena_init(&arena)
 defer mem.dynamic_arena_destroy(&arena)
 context.allocator = mem.dynamic_arena_allocator(&arena)
 path, _ := test_directory("entasis-parameter-lanes")
 defer test_directory_remove(path)
 header := "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters\n"
 testing.expect_value(t, write_lane(path, 1, fmt.aprintf("%sok,ok,10,1,10,a\n", header)), Report_Status.Ok)
 for row in ([]string{"ok,ok,5,2,10,b\n", "ok,ok,5,2,11,a\n"})
 {
  testing.expect_value(t, write_lane(path, 2, fmt.aprintf("%s%s", header, row)), Report_Status.Ok)
  request := test_request(path, {1, 2}, 1)
  request.package_name = "pyramid"
  request.variant = "spheres"
  result := collect_summaries(request)
  testing.expect_value(t, result.status, Report_Status.Invalid_Measurement)
 }
}

@(test)
parameterized_report_records_variant_settings_and_step_mean :: proc(t: ^testing.T)
{
 arena: mem.Dynamic_Arena
 mem.dynamic_arena_init(&arena)
 defer mem.dynamic_arena_destroy(&arena)
 context.allocator = mem.dynamic_arena_allocator(&arena)
 path, _ := test_directory("entasis-parameter-report")
 defer test_directory_remove(path)
 header := "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters\n"
 testing.expect_value(t, write_lane(path, 1, fmt.aprintf("%sok,ok,10,1,10,shape=sphere;steps=10\nok,ok,30,1,10,shape=sphere;steps=10\n", header)), Report_Status.Ok)
 request := test_request(path, {1}, 2)
 request.package_name = "contact_islands"
 request.variant = "sphere-small"
 request.command = "run_benchmark.sh --package contact_islands --shape sphere --variant sphere-small"
 result := generate_report(request)
 testing.expect_value(t, result.status, Report_Status.Ok)
 testing.expect(t, strings.contains(result.markdown, "Variant: `sphere-small`"))
 testing.expect(t, strings.contains(result.markdown, "shape=sphere;steps=10"))
 testing.expect(t, strings.contains(result.markdown, "| 1 | 2.000000 | 1.000000 | 3.000000 |"))
 testing.expect(t, strings.contains(result.markdown, request.command))
 testing.expect(t, strings.contains(result.markdown, "not the median of individual step times"))
}

PHYSICAL_TEST_HEADER :: "worker_count,step_index,case_status,metric_status,physics_elapsed_ms,benchmark_parameters,verification_mode,physical_criterion,physical_status,physical_coverage,physical_reason,physical_sample_count,physical_shape_limit_m,physical_first_phase,physical_first_step,physical_first_body,physical_first_value_m,physical_max_slot_excess_m,physical_max_slot_excess_phase,physical_max_slot_excess_step,physical_max_slot_excess_body,physical_max_neighbor_compression_m,physical_max_neighbor_compression_phase,physical_max_neighbor_compression_step,physical_max_neighbor_compression_body,physical_warmup_samples,physical_measured_samples,physical_terminal_samples,physical_impact_step\n"
PHYSICAL_TEST_PASS :: "1,61,ok,ok,100,workload=contact_islands;steps=61;timestep-hz=60;warmup-steps=0;verify=on,on,island_boxes,pass,complete,none,62,0.1,,,,,0,measured,1,0,0,measured,1,0,0,61,61,\n"
PHYSICAL_TEST_FAIL :: "1,61,ok,ok,200,workload=contact_islands;steps=61;timestep-hz=60;warmup-steps=0;verify=on,on,island_boxes,fail,complete,slot_excess,62,0.1,measured,1,0,0.11,0.11,measured,1,0,0,measured,1,0,0,61,61,\n"
PHYSICAL_TEST_OFF :: "1,61,ok,ok,200,workload=contact_islands;steps=61;timestep-hz=60;warmup-steps=0;verify=off,off,island_boxes,unassessed,incomplete,verification_off,0,0.1,,,,,,,,,,,,,0,0,0,\n"

@(test)
physical_outcomes_preserve_timing_and_coverage :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	defer context.allocator = owned_allocator
	path: string
	status: Report_Status
	path, status = test_directory("entasis-physical-report")
	testing.expect_value(t, status, Report_Status.Ok)
	defer test_directory_remove(path)
	request: Report_Request = test_request(path, {1}, 2)
	request.package_name = "contact_islands"
	request.variant = "physical-test"
	cases: [3]string = {PHYSICAL_TEST_PASS+PHYSICAL_TEST_PASS, PHYSICAL_TEST_PASS+PHYSICAL_TEST_FAIL, PHYSICAL_TEST_OFF+PHYSICAL_TEST_OFF}
	outcomes: [3]Result_Physical = {.Pass, .Fail, .Unassessed}
	for rows, index in cases
	{
		testing.expect_value(t, write_lane(path, 1, fmt.aprintf("%s%s", PHYSICAL_TEST_HEADER, rows)), Report_Status.Ok)
		result: Report_Result = collect_summaries(request)
		testing.expect_value(t, result.status, Report_Status.Ok)
		if result.status != .Ok
		{
			continue
		}
		testing.expect_value(t, result.summaries[0].physical.status, outcomes[index])
		testing.expect_value(t, result.summaries[0].median, f64(100 if index == 0 else 150 if index == 1 else 200))
		markdown: string = render_markdown(request, result.summaries[:])
		testing.expect(t, strings.contains(markdown, "Physical outcome"))
		testing.expect_value(t, result.summaries[0].sample_count, 2)
	}
	unassessed_rows: [2]string = {
		"1,61,ok,ok,100,workload=contact_islands;steps=61;timestep-hz=60;warmup-steps=0;verify=on,on,island_boxes,unassessed,incomplete,incomplete_coverage,31,0.1,,,,,0,measured,1,0,0,measured,1,0,0,30,30,\n",
		"1,61,ok,ok,100,workload=contact_islands;steps=61;timestep-hz=60;warmup-steps=0;verify=on,on,island_boxes,unassessed,complete,unsupported_fixture,62,0.1,,,,,,,,,,,,,0,61,0,\n",
	}
	for row in unassessed_rows
	{
		_ = write_lane(path, 1, fmt.aprintf("%s%s%s", PHYSICAL_TEST_HEADER, row, row))
		result: Report_Result = collect_summaries(request)
		testing.expect_value(t, result.status, Report_Status.Ok)
		if result.status == .Ok
		{
			testing.expect_value(t, result.summaries[0].physical.status, Result_Physical.Unassessed)
		}
	}
	_ = write_lane(path, 1, PHYSICAL_TEST_HEADER+PHYSICAL_TEST_PASS+PHYSICAL_TEST_OFF)
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)
	for replacement in ([][2]string{
		{",62,0.1,", ",61,0.1,"},
		{",0,61,61,", ",0,61,60,"},
		{",pass,complete,", ",pass,incomplete,"},
		{",0.1,", ",NaN,"},
		{",0,measured,1,0,", ",0.11,measured,1,0,"},
	})
	{
		invalid, _ := strings.replace_all(PHYSICAL_TEST_PASS, replacement[0], replacement[1])
		_ = write_lane(path, 1, fmt.aprintf("%s%s%s", PHYSICAL_TEST_HEADER, invalid, invalid))
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)
	}
	_ = write_lane(path, 1, PHYSICAL_TEST_HEADER+PHYSICAL_TEST_PASS)
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Row_Count)
	request.package_name = "pyramid"
	request.run_count = 1
	one_body: string = "1,61,ok,ok,100,workload=pyramid;rows=1;steps=61;timestep-hz=60;warmup-steps=0;verify=on,on,pyramid_boxes,pass,complete,none,62,0.1,,,,,0,construction,0,0,,,,,0,61,0,\n"
	_ = write_lane(path, 1, fmt.aprintf("%s%s", PHYSICAL_TEST_HEADER, one_body))
	one_body_result: Report_Result = collect_summaries(request)
	testing.expect_value(t, one_body_result.status, Report_Status.Ok)
	if one_body_result.status == .Ok
	{
		testing.expect_value(t, one_body_result.summaries[0].physical.status, Result_Physical.Pass)
	}
	for impact_step: int = 0; impact_step <= 2; impact_step += 1
	{
		row: string = fmt.aprintf("%s%d\n", strings.trim_suffix(one_body, "\n"), impact_step)
		_ = write_lane(path, 1, fmt.aprintf("%s%s", PHYSICAL_TEST_HEADER, row))
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement if impact_step <= 1 else Report_Status.Ok)
	}
	unassessed_impact, _ := strings.replace_all(one_body, ",pass,complete,none,", ",unassessed,complete,impact_before_assessment,")
	unassessed_impact = fmt.aprintf("%s1\n", strings.trim_suffix(unassessed_impact, "\n"))
	_ = write_lane(path, 1, fmt.aprintf("%s%s", PHYSICAL_TEST_HEADER, unassessed_impact))
	impact_result: Report_Result = collect_summaries(request)
	testing.expect_value(t, impact_result.status, Report_Status.Ok)
	if impact_result.status == .Ok
	{
		testing.expect_value(t, impact_result.summaries[0].physical.status, Result_Physical.Unassessed)
		testing.expect_value(t, impact_result.summaries[0].median, f64(100))
	}
}

@(test)
mixed_query_report_normalizes_native_totals_per_batch :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator
	arena: mem.Dynamic_Arena
	mem.dynamic_arena_init(&arena)
	defer mem.dynamic_arena_destroy(&arena)
	context.allocator = mem.dynamic_arena_allocator(&arena)
	defer context.allocator = owned_allocator
	path: string
	status: Report_Status
	path, status = test_directory("entasis-mixed-query-report")
	testing.expect_value(t, status, Report_Status.Ok)
	defer test_directory_remove(path)
	header :: "worker_count,query_count,measured_batches,case_status,metric_status,physics_elapsed_ms\n"
	_ = write_lane(path, 1, header+"1,100000,100,ok,ok,200\n1,100000,100,ok,ok,400\n")
	request: Report_Request = test_request(path, {1}, 2)
	request.package_name = "spatial_query_trace"
	result: Report_Result = collect_summaries(request)
	testing.expect_value(t, result.status, Report_Status.Ok)
	if result.status == .Ok
	{
		testing.expect_value(t, result.summaries[0].median, f64(300))
		markdown: string = render_markdown(request, result.summaries[:])
		testing.expect(t, strings.contains(markdown, "ms/batch (100,000 queries)"))
		testing.expect(t, strings.contains(markdown, "| 1 | 2 | 3.000000 | 2.000000 | 4.000000 |"))
		testing.expect(t, strings.contains(markdown, "37.500000"))
	}
	for count in ([]string{"0", "-1", "99", "NaN"})
	{
		_ = write_lane(path, 1, fmt.aprintf("%s1,100000,100,ok,ok,200\n1,100000,%s,ok,ok,400\n", header, count))
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)
	}
	_ = write_lane(path, 1, header+"1,99999,100,ok,ok,200\n1,99999,100,ok,ok,400\n")
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)

}
