package benchmark_report

import "core:fmt"
import "core:mem"
import "core:os"
import "core:path/filepath"
import "core:strings"
import "core:testing"

PRECISION_TEST_HEADER :: [20]string{
	"precision_case", "precision_candidate", "sampling_mode", "sampling_count", "frame_median_ms",
	"frame_max_ms", "maintenance_count", "maintenance_max_ms", "export_checksum", "worker_count",
	"step_index", "case_status", "metric_status", "physics_elapsed_ms", "timing_method", "precision", "benchmark_parameters", "recording_path", "recording_component", "recording_mode",
};

precision_test_row :: proc() -> [20]string
{
	return {"compact-near", "mixed64", "batch", "512", "", "", "0", "", "123", "1", "512", "ok", "ok", "2", "whole_loop", "mixed64", "workload=large_world_precision;steps=512", "", "", "off"};
}

@(test)
precision_report_rejects_candidate_case_mode_worker_and_profile_mismatch :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = owned_allocator;
	header: [20]string = PRECISION_TEST_HEADER;
	row: [20]string = precision_test_row();
	records: [2][]string = {header[:], row[:]};
	for field in ([4]int{1, 9, 15, 2})
	{
		row = precision_test_row();
		row[field] = "float32" if field == 15 else "origin" if field == 1 else "4" if field == 9 else "unknown";
		result: Report_Result;
		_, result = summarize_records(records[:], "precision-row", 1, 1, "large_world_precision");
		testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
	}
	row = precision_test_row();
	path: string;
	path_status: Report_Status;
	path, path_status = test_directory("entasis-compiled-precision-selection");
	if !testing.expect_value(t, path_status, Report_Status.Ok)
	{
		return;
	}
	defer test_directory_remove(path);
	defer delete(path);
	file: string;
	file, _ = filepath.join([]string{path, "workers-1.csv"});
	defer delete(file);
	header_text: string = strings.join(header[:], ",");
	defer delete(header_text);
	row_text: string = strings.join(row[:], ",");
	defer delete(row_text);
	csv: string = fmt.aprintf("%s\n%s\n", header_text, row_text);
	defer delete(csv);
	testing.expect_value(t, os.write_entire_file(file, csv), os.Error(nil));
	request: Report_Request = {source_directory=path, package_name="large_world_precision", run_count=1,
		workers=([]int{1}), precision="float32", precision_candidate="mixed64", precision_case="compact-near", sampling_mode="batch"};
	result: Report_Result = collect_summaries(request);
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
}

@(test)
precision_report_preserves_batch_and_instrumented_metric_availability :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = owned_allocator;
	header: [20]string = PRECISION_TEST_HEADER;
	row: [20]string = precision_test_row();
	records: [2][]string = {header[:], row[:]};
	summary: Lane_Summary;
	result: Report_Result;
	summary, result = summarize_records(records[:], "batch-row", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Ok);
	testing.expect_value(t, summary.precision.frame_median_ms, f64(0));
	row[4] = "0";
	_, result = summarize_records(records[:], "invented-batch-latency", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
	row = precision_test_row();
	row[2], row[4], row[5] = "instrumented", "0.1", "0.3";
	_, result = summarize_records(records[:], "instrumented-row", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Ok);
	row[5] = "0.01";
	_, result = summarize_records(records[:], "reversed-latencies", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
}

@(test)
precision_report_memory_requires_counts_release_and_reserved_allocation_contracts :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = owned_allocator;
	header, row: [33]string;
	for name, index in PRECISION_TEST_HEADER
	{
		header[index] = name;
	}
	for name, index in PRECISION_MEMORY_NAMES
	{
		header[index+20] = name;
	}
	base: [20]string = precision_test_row();
	for value, index in base
	{
		row[index] = value;
	}
	row[2], row[13], row[14] = "memory", "", "allocator_instrumented";
	for index in 20 ..< len(row)
	{
		row[index] = "0";
	}
	records: [2][]string = {header[:], row[:]};
	result: Report_Result;
	_, result = summarize_records(records[:], "memory-row", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Ok);
	for field in ([3]int{3, 31, 32})
	{
		previous: string = row[field];
		row[field] = "1";
		_, result = summarize_records(records[:], "memory-contract-failure", 1, 1, "large_world_precision");
		testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
		row[field] = previous;
	}
	row[0], row[3], row[10], row[25] = "queries-local", "8", "8", "1";
	_, result = summarize_records(records[:], "reserved-query-allocation", 1, 1, "large_world_precision");
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
}

@(test)
precision_report_rejects_nonfinite_checksum_and_timing :: proc(t: ^testing.T)
{
	owned_allocator: mem.Allocator = context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = owned_allocator;
	header: [20]string = PRECISION_TEST_HEADER;
	row: [20]string = precision_test_row();
	records: [2][]string = {header[:], row[:]};
	for field in ([2]int{8, 13})
	{
		for text in ([3]string{"NaN", "+Inf", "-Inf"})
		{
			row = precision_test_row();
			row[field] = text;
			result: Report_Result;
			_, result = summarize_records(records[:], "nonfinite-row", 1, 1, "large_world_precision");
			testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
		}
	}
}

@(test)
precision_report_rejects_unsupported_cases_query_workers_and_engine_only :: proc(t: ^testing.T)
{
	testing.expect_value(t, precision_case_status("origin", "distributed", "batch", 1), Report_Status.Invalid_Arguments);
	testing.expect_value(t, precision_case_status("origin", "queries-spanning", "batch", 1), Report_Status.Invalid_Arguments);
	testing.expect_value(t, precision_case_status("mixed64", "queries-local", "engine-only", 1), Report_Status.Invalid_Arguments);
	testing.expect_value(t, precision_case_status("mixed64", "queries-spanning", "memory", 4), Report_Status.Invalid_Arguments);
	testing.expect_value(t, precision_case_status("mixed64", "queries-spanning", "memory", 1), Report_Status.Ok);
}

@(test)
precision_report_cli_rejects_unknown_duplicate_and_inapplicable_selection :: proc(t: ^testing.T)
{
	args: []string = {"report", "--source", "unused", "--package", "large_world_precision", "--configuration", "release",
		"--runs", "1", "--workers", "1", "--compiler-version", "fixture", "--operating-system", "fixture", "--processor", "fixture",
		"--logical-processors", "1", "--command", "fixture", "--precision", "mixed64", "--precision-candidate", "mixed64", "--precision-case", "compact-near", "--sampling-mode", "batch"};
	status: Report_Status;
	_, status = parse_cli(args);
	testing.expect_value(t, status, Report_Status.Ok);
	args[22] = "mixed32";
	_, status = parse_cli(args);
	testing.expect_value(t, status, Report_Status.Invalid_Arguments);
	args[22] = "mixed64";
	duplicate: [31]string;
	for value, index in args
	{
		duplicate[index] = value;
	}
	duplicate[29], duplicate[30] = "--precision", "mixed64";
	_, status = parse_cli(duplicate[:]);
	testing.expect_value(t, status, Report_Status.Invalid_Arguments);
}
