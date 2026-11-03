package benchmark_report

import "base:runtime"
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

@(test)
paired_gain_interval_is_reproducible :: proc(t: ^testing.T)
{
	baseline: [5]f64 = {100, 200, 300, 400, 500};
	candidate: [5]f64 = {90, 220, 240, 380, 500};
	a, b: Paired_Summary;
	status: Report_Status;
	a, status = paired_gain_interval(baseline[:], candidate[:]);
	testing.expect_value(t, status, Report_Status.Ok);
	b, status = paired_gain_interval(baseline[:], candidate[:]);
	testing.expect_value(t, status, Report_Status.Ok);
	testing.expect_value(t, a, b);
	testing.expect_value(t, a.gain, f64(0.05));
	testing.expect_value(t, a.lower, f64(-0.1));
	testing.expect_value(t, a.upper, f64(0.2));
	_, status = paired_gain_interval({1}, {});
	testing.expect_value(t, status, Report_Status.Invalid_Row_Count);
	_, status = paired_gain_interval({0}, {1});
	testing.expect_value(t, status, Report_Status.Invalid_Measurement);
}

@(test)
paired_comparison_rejects_mismatched_or_incomplete_arms :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline_path: string;
	candidate_path: string;
	directory_status: Report_Status;
	baseline_path, directory_status = test_directory("entasis-paired-baseline");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(baseline_path);
	candidate_path, directory_status = test_directory("entasis-paired-candidate");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(candidate_path);
	header: string = "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters,benchmark_components\n";
	baseline_rows: string = strings.concatenate({header, "ok,ok,100,1,300,shape=box,common\nok,ok,200,1,300,shape=box,common\n"});
	candidate_rows: string = strings.concatenate({header, "ok,ok,90,1,300,shape=box,common\nok,ok,220,1,300,shape=box,common\n"});
	testing.expect_value(t, write_lane(baseline_path, 1, baseline_rows), Report_Status.Ok);
	testing.expect_value(t, write_lane(candidate_path, 1, candidate_rows), Report_Status.Ok);
	request: Report_Request = test_request(candidate_path, {1}, 2);
	request.package_name = "container";
	request.variant = "fixture";
	request.baseline_source = baseline_path;
	result: Report_Result = collect_summaries(request);
	testing.expect_value(t, result.status, Report_Status.Ok);
	testing.expect_value(t, len(result.paired_summaries), 1);
	if result.status == .Ok && len(result.paired_summaries) == 1
	{
		testing.expect_value(t, result.paired_summaries[0].gain, f64(0));
	}
	mismatched_rows: [3]string = {
		strings.concatenate({header, "ok,ok,90,1,300,shape=sphere,common\nok,ok,220,1,300,shape=sphere,common\n"}),
		strings.concatenate({header, "ok,ok,90,1,301,shape=box,common\nok,ok,220,1,301,shape=box,common\n"}),
		strings.concatenate({header, "ok,ok,90,1,300,shape=box,all\nok,ok,220,1,300,shape=box,all\n"}),
	};
	for rows in mismatched_rows
	{
		testing.expect_value(t, write_lane(candidate_path, 1, rows), Report_Status.Ok);
		single: Report_Request = request;
		single.baseline_source = "";
		testing.expect_value(t, collect_summaries(single).status, Report_Status.Ok);
		single.source_directory = baseline_path;
		testing.expect_value(t, collect_summaries(single).status, Report_Status.Ok);
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	}
	testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({header, "ok,ok,90,1,300,shape=box,common\n"})), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Row_Count);
	testing.expect_value(t, write_lane(candidate_path, 1, candidate_rows), Report_Status.Ok);
	baseline_file: string;
	join_error: runtime.Allocator_Error;
	baseline_file, join_error = filepath.join({baseline_path, "workers-1.csv"});
	testing.expect_value(t, join_error, runtime.Allocator_Error(nil));
	testing.expect_value(t, os.remove(baseline_file), os.Error(nil));
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Missing_Lane);
}

@(test)
paired_report_preserves_append_order_and_baseline_worker_contract :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline_path: string;
	candidate_path: string;
	directory_status: Report_Status;
	baseline_path, directory_status = test_directory("entasis-paired-workers-baseline");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(baseline_path);
	candidate_path, directory_status = test_directory("entasis-paired-workers-candidate");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(candidate_path);
	header: string = "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters\n";
	for worker in ([2]int{1, 2})
	{
		testing.expect_value(t, write_lane(baseline_path, worker, fmt.aprintf("%sok,ok,100,%d,300,shape=box\nok,ok,200,%d,300,shape=box\n", header, worker, worker)), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, worker, fmt.aprintf("%sok,ok,190,%d,300,shape=box\nok,ok,110,%d,300,shape=box\n", header, worker, worker)), Report_Status.Ok);
	}
	request: Report_Request = test_request(candidate_path, {1, 2}, 2);
	request.package_name = "container";
	request.variant = "fixture";
	request.baseline_source = baseline_path;
	result: Report_Result = generate_report(request);
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status == .Ok
	{
		testing.expect(t, strings.contains(result.markdown, "| 1 | 2 | -22.500000 |"));
		testing.expect(t, strings.contains(result.markdown, "| 2 | 2 | -22.500000 |"));
	}
	testing.expect_value(t, write_lane(baseline_path, 2, strings.concatenate({header, "ok,ok,100,2,300,shape=sphere\nok,ok,200,2,300,shape=sphere\n"})), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	testing.expect_value(t, write_lane(baseline_path, 2, strings.concatenate({header, "ok,ok,100,2,300,shape=box\nok,ok,200,2,300,shape=box\n"})), Report_Status.Ok);
	testing.expect_value(t, write_lane(baseline_path, 3, strings.concatenate({header, "ok,ok,100,3,300,shape=box\nok,ok,200,3,300,shape=box\n"})), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Unexpected_Lane);
}

@(test)
paired_reports_support_explicit_solver_workloads :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline_path: string;
	candidate_path: string;
	directory_status: Report_Status;
	baseline_path, directory_status = test_directory("entasis-paired-solver-baseline");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(baseline_path);
	candidate_path, directory_status = test_directory("entasis-paired-solver-candidate");
	testing.expect_value(t, directory_status, Report_Status.Ok);
	defer test_directory_remove(candidate_path);
	request: Report_Request = test_request(candidate_path, {1}, 2);
	request.baseline_source = baseline_path;
	for package_name in ([2]string{"solver_joint_angular", "solver_fixed_work"})
	{
		request.package_name = package_name;
		header: string = "case_status,metric_status,physics_elapsed_ms,worker_count,step_index,benchmark_parameters\n" if package_name == "solver_joint_angular" else "case_status,metric_status,fixed_elapsed_ms,worker_count,measured_passes,benchmark_parameters\n";
		testing.expect_value(t, write_lane(baseline_path, 1, strings.concatenate({header, "ok,ok,100,1,300,fixture\nok,ok,200,1,300,fixture\n"})), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({header, "ok,ok,90,1,300,fixture\nok,ok,220,1,300,fixture\n"})), Report_Status.Ok);
		result: Report_Result = collect_summaries(request);
		testing.expect_value(t, result.status, Report_Status.Ok);
		if result.status == .Ok && package_name == "solver_fixed_work"
		{
			testing.expect_value(t, result.summaries[0].measured_steps, 0);
			testing.expect_value(t, result.summaries[0].measured_passes, 300);
			testing.expect_value(t, result.summaries[0].median/f64(result.summaries[0].measured_passes), f64(155)/300);
		}
	}
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

contact_report_fixture :: proc(
	input: string, elapsed: []f64, status: string = "pass", failures: int = 0,
	latencies: [][3]f64 = {{0.5, 1.5, 2}, {0.75, 1.75, 2.5}},
) -> string
{
	builder: strings.Builder;
	strings.builder_init(&builder);
	strings.write_string(&builder, "case_status,metric_status,worker_count,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,contact_input,contact_points,point_updates,physics_status,quality_coverage,quality_failures,residual_max_mps,residual_rms_mps,residual_p95_mps,negative_impulse_max,friction_violation_max,twist_violation_max,ms_per_1000_point_updates,processor_affinity,call_p50_ms,call_p95_ms,call_max_ms,precision,benchmark_parameters\n");
	for value, index in elapsed
	{
		fmt.sbprintf(&builder, "ok,ok,1,100,10,%.9f,%.9f,\"%s\",1000,400000,%s,100,%d,0.2,0.1,0.15,0,0,0,%.9f,16777216,%.9f,%.9f,%.9f,float32,workload=solver_contact_work;region=solver-stage;substeps=4;velocity-iterations=1;timestep-hz=60\n",
			value, value/100, input, status, failures, value/400, latencies[index][0], latencies[index][1], latencies[index][2]);
	}
	return strings.to_string(builder);
}

@(test)
paired_headlines_use_average_cost_gain :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline, candidate: string;
	bs, cs: Report_Status;
	baseline, bs = test_directory("entasis-average-gain-baseline");
	candidate, cs = test_directory("entasis-average-gain-candidate");
	testing.expect_value(t, bs, Report_Status.Ok);
	testing.expect_value(t, cs, Report_Status.Ok);
	defer test_directory_remove(baseline);
	defer test_directory_remove(candidate);
	totals: [2][5]f64 = {{100, 100, 100, 100, 1000}, {110, 110, 110, 110, 110}};
	latencies: [5][3]f64 = {{0.5, 12, 20}, {0.5, 12, 20}, {0.5, 12, 20}, {0.5, 12, 20}, {0.5, 12, 20}};
	for package_name in ([2]string{"solver_contact_work", "solver_fixed_work"})
	{
		for arm: int = 0; arm < len(totals); arm += 1
		{
			elapsed: [5]f64 = totals[arm];
			rows: string;
			if package_name == "solver_contact_work"
			{
				rows = contact_report_fixture("C:/common/accepted.step300.bin", elapsed[:], latencies=latencies[:]);
			}
			else
			{
				builder: strings.Builder;
				strings.builder_init(&builder);
				strings.write_string(&builder, "case_status,metric_status,worker_count,case,bodies,admitted_units,work_units,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,residual_max_mps,residual_rms_mps,residual_p95_mps,precision,benchmark_parameters\n");
				for value in elapsed
				{
					fmt.sbprintf(&builder, "ok,ok,1,convex,10000,20000,8000000,100,10,%.9f,%.9f,pass,100,0,0.1,0.05,0.08,float32,workload=solver_fixed_work;region=prepare-and-native-solver;substeps=4\n", value, value/100);
				}
				rows = strings.to_string(builder);
			}
			testing.expect_value(t, write_lane(baseline if arm == 0 else candidate, 1, rows), Report_Status.Ok);
		}
		request: Report_Request = test_request(candidate, {1}, 5);
		request.package_name, request.baseline_source = package_name, baseline;
		result: Report_Result = generate_report(request);
		testing.expect(t, result.status == .Ok, result.diagnostic);
		if result.status != .Ok
		{
			continue;
		}
		paired: Paired_Summary = result.paired_summaries[0];
		testing.expect(t, abs(result.summaries[0].mean/f64(100)-1.1) < 1e-12);
		testing.expect_value(t, paired.gain, f64(-0.1));
		if package_name == "solver_contact_work"
		{
			testing.expect(t, abs(paired.baseline_ms_per_call-2.8) < 1e-12);
			testing.expect(t, abs(paired.candidate_ms_per_call-1.1) < 1e-12);
		}
		else
		{
			testing.expect_value(t, paired.metric_count, 1);
			testing.expect_value(t, paired.metrics[0].unit, "ms/solve");
			testing.expect(t, abs(paired.metrics[0].baseline-2.8) < 1e-12);
			testing.expect(t, abs(paired.metrics[0].candidate-1.1) < 1e-12);
			testing.expect(t, abs(paired.metrics[0].gain+0.1) < 1e-12);
		}
		comparison: string = result.markdown;
		testing.expect_value(t, strings.count(comparison, "## Paired comparison"), 1);
		testing.expect_value(t, strings.count(comparison, "| Threads | Baseline ms/solve | Candidate ms/solve | Gain % |"), 1);
		testing.expect(t, !strings.contains(comparison, "| Workers | Samples | Average"));
		testing.expect(t, !strings.contains(comparison, "| Threads | Metric |"));
		testing.expect_value(t, strings.count(comparison, "Measured passes per process: 100"), 1);
		boundary: int = strings.index(comparison, "<details><summary>Paired variation and intervals</summary>");
		if !testing.expect(t, boundary >= 0)
		{
			continue;
		}
		testing.expect(t, strings.contains(comparison[:boundary], "| Threads | Baseline ms/solve | Candidate ms/solve | Gain % |"));
		testing.expect(t, strings.contains(comparison[:boundary], "| 1 | 2.800000000 | 1.100000000 | 60.714286 |"));
		testing.expect(t, !strings.contains(comparison[:boundary], "-10.000000"));
		testing.expect(t, strings.contains(comparison[boundary:], "Median paired gain %"));
		testing.expect(t, strings.contains(comparison[boundary:], "different statistic from the average-cost headline gain"));
		testing.expect(t, strings.contains(comparison[boundary:], "| 1 | 5 | -10.000000 |"));
		if package_name == "solver_contact_work"
		{
			testing.expect(t, strings.contains(comparison, "| 1 | 0.700000000 | 0.275000000 | 60.714286 |"));
		}
		report_path: string;
		join_error: runtime.Allocator_Error;
		report_path, join_error = filepath.join({candidate, "README.md"});
		testing.expect_value(t, join_error, runtime.Allocator_Error(nil));
		testing.expect_value(t, os.remove(report_path), os.Error(nil));
	}
}

@(test)
contact_work_report_uses_fixed_timing_and_quality :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline, candidate: string;
	bs, cs: Report_Status;
	baseline, bs = test_directory("entasis-contact-report-baseline");
	candidate, cs = test_directory("entasis-contact-report-candidate");
	testing.expect_value(t, bs, Report_Status.Ok);
	testing.expect_value(t, cs, Report_Status.Ok);
	defer test_directory_remove(baseline);
	defer test_directory_remove(candidate);
	input: string = "C:/common/contact; trial/accepted.step300.bin";
	testing.expect_value(t, write_lane(baseline, 1, contact_report_fixture(input, {100, 120}, latencies={{0.5, 1.5, 2.5}, {0.75, 2, 3}})), Report_Status.Ok);
	testing.expect_value(t, write_lane(candidate, 1, contact_report_fixture(input, {80, 96}, latencies={{0.25, 1, 1.5}, {0.5, 1.5, 2}})), Report_Status.Ok);
	request: Report_Request = test_request(candidate, {1}, 2);
	request.package_name, request.baseline_source = "solver_contact_work", baseline;
	result: Report_Result = collect_summaries(request);
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status != .Ok
	{
		return;
	}
	summary: Lane_Summary = result.summaries[0];
	testing.expect_value(t, summary.measured_passes, 100);
	testing.expect_value(t, summary.median/f64(summary.measured_passes), f64(0.88));
	testing.expect_value(t, summary.contact.updates, i64(400000));
	testing.expect_value(t, summary.contact.covered, 200);
	testing.expect_value(t, summary.contact.residual_max, f64(0.2));
	testing.expect_value(t, summary.contact.input, input);
	testing.expect_value(t, summary.contact.call_p50_median_ms, f64(0.375));
	testing.expect_value(t, summary.contact.call_p95_median_ms, f64(1.25));
	testing.expect_value(t, summary.contact.call_max_ms, f64(2));
	testing.expect_value(t, result.paired_summaries[0].gain, f64(0.2));
	markdown: string = render_markdown(request, result.summaries[:]);
	testing.expect(t, strings.contains(markdown, "ms/1000 point updates"));
	testing.expect(t, strings.contains(markdown, "0.220000000"));
	testing.expect(t, strings.contains(markdown, "Worst call RMS m/s"));
	testing.expect(t, strings.contains(markdown, "own final-substep equations"));
	testing.expect(t, strings.contains(markdown, "Median process call p95 ms"));
	testing.expect(t, strings.contains(markdown, "| 1 | 0.375000000 | 1.250000000 | 2.000000000 |"));
	testing.expect(t, strings.contains(markdown, "not pooled-call quantiles"));
	comparison: string = render_paired_comparison(request, result.paired_summaries[:]);
	testing.expect(t, strings.contains(comparison, "| Threads | Baseline ms/solve | Candidate ms/solve | Gain % |"));
	testing.expect(t, strings.contains(comparison, "| 1 | 1.100000000 | 0.880000000 | 20.000000 |"));
	testing.expect(t, strings.contains(comparison, "| 1 | 0.275000000 | 0.220000000 | 20.000000 |"));
	testing.expect(t, strings.contains(comparison, "| 1 | baseline | 200 | 0 |"));
	testing.expect(t, strings.contains(comparison, "| 1 | candidate | 200 | 0 |"));
	testing.expect(t, strings.contains(comparison, "| Negative impulse N s |"));
	testing.expect(t, strings.contains(comparison, "| 1 | baseline | 0.625000000 | 1.750000000 | 3.000000000 |"));
	testing.expect(t, strings.contains(comparison, "| 1 | candidate | 0.375000000 | 1.250000000 | 2.000000000 |"));
}

@(test)
contact_work_report_rejects_mismatched_or_incomplete_inputs :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	baseline, candidate: string;
	bs, cs: Report_Status;
	baseline, bs = test_directory("entasis-contact-reject-baseline");
	candidate, cs = test_directory("entasis-contact-reject-candidate");
	testing.expect_value(t, bs, Report_Status.Ok);
	testing.expect_value(t, cs, Report_Status.Ok);
	defer test_directory_remove(baseline);
	defer test_directory_remove(candidate);
	input: string = "C:/common/contact; trial/accepted.step300.bin";
	base: string = contact_report_fixture(input, {100, 120});
	testing.expect_value(t, write_lane(baseline, 1, base), Report_Status.Ok);
	request: Report_Request = test_request(candidate, {1}, 2);
	request.package_name, request.baseline_source = "solver_contact_work", baseline;
	for change in ([5][2]string{{"accepted.step300", "r02-pairs.step300"}, {"400000", "400001"}, {"substeps=4", "substeps=3"}, {"16777216", "4294967295"}, {",100,0,", ",99,0,"}})
	{
		changed: string;
		changed, _ = strings.replace_all(base, change[0], change[1]);
		testing.expect_value(t, write_lane(candidate, 1, changed), Report_Status.Ok);
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	}
	missing: string;
	missing, _ = strings.replace_all(base, "call_p95_ms", "missing_call_p95");
	testing.expect_value(t, write_lane(candidate, 1, missing), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Header);
	for change in ([5][2]string{{"0.500000000", "nan"}, {"1.500000000", "inf"}, {"0.500000000", "-0.500000000"}, {"0.500000000", "1.600000000"}, {"2.000000000", "1.000000000"}})
	{
		changed: string;
		changed, _ = strings.replace_all(base, change[0], change[1]);
		testing.expect_value(t, write_lane(candidate, 1, changed), Report_Status.Ok);
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	}
	testing.expect_value(t, write_lane(candidate, 1, contact_report_fixture(input, {100, 120}, latencies={{0.2, 0.4, 0.8}, {0.3, 0.5, 0.9}})), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	separator: int = strings.index_byte(base, '\n');
	testing.expect_value(t, write_lane(candidate, 1, base[:separator+1]), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Row_Count);
	testing.expect_value(t, write_lane(candidate, 1, contact_report_fixture(input, {80, 96}, "fail", 1)), Report_Status.Ok);
	failed: Report_Result = collect_summaries(request);
	testing.expect_value(t, failed.status, Report_Status.Ok);
	if failed.status == .Ok
	{
		testing.expect_value(t, failed.summaries[0].contact.failed, 2);
		testing.expect(t, strings.contains(render_markdown(request, failed.summaries[:]), "Hard-failed calls"));
	}
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
		markdown: string = render_markdown(request, {{worker_count=1, sample_count=1, median=1, minimum=1, maximum=1,
			primary_unit="ms/process total", primary_divisor=1}})
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
	valid: []string = {"benchmark_report", "--source", "unused", "--package", "query_extensions", "--configuration", "release", "--runs", "1", "--workers", "1", "--compiler-version", "test", "--operating-system", "test", "--processor", "test", "--logical-processors", "8", "--command", "test"};
	for options in ([][]string{
		{"--query-group", "convex.ray_all", "--query-repetitions", "2", "--query-warmup-repetitions", "3"},
		{"--query-group", "mesh.shape_distance", "--components", "all"},
	})
	{
		args: [dynamic]string;
		append(&args, ..valid);
		append(&args, ..options);
		_, status := parse_cli(args[:]);
		testing.expect_value(t, status, Report_Status.Ok);
		delete(args);
	}
	for options in ([][]string{
		{"--query-group"}, {"--query-group", ""},
		{"--query-group", "convex.ray_all", "--query-group", "convex.ray_all"},
		{"--query-group", "convex.ray_all", "--query-repetitions", "0"},
		{"--query-group", "convex.ray_all", "--query-repetitions", "99999999999999999999999"},
		{"--query-group", "convex.ray_all", "--query-warmup-repetitions", "+2"},
		{"--query-group", "convex.unknown"}, {"--query-group", "mesh.shape_distance", "--components", "common"},
		{"--query-repetitions", "2"}, {"--query-family", "ray"}, {"--measured-batches", "2"},
	})
	{
		args: [dynamic]string;
		append(&args, ..valid);
		append(&args, ..options);
		_, status := parse_cli(args[:]);
		testing.expect(t, status != .Ok);
		delete(args);
	}

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
 testing.expect(t, strings.contains(result.markdown, "| 1 | 2 | 2.000000 | 1.000000 | 3.000000 | 1.000000x |"))
 testing.expect(t, strings.contains(result.markdown, request.command))
 testing.expect(t, strings.contains(result.markdown, "Average is the arithmetic mean across process samples"))
 testing.expect_value(t, strings.count(result.markdown, "| Workers | Samples | Average ms/step |"), 1)
 testing.expect(t, !strings.contains(result.markdown, "| Workers | Average ms/step |"))
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
		paired_request: Report_Request = request;
		paired_request.baseline_source = path;
		paired_result: Report_Result = collect_summaries(paired_request);
		testing.expect_value(t, paired_result.status, Report_Status.Ok);
		if paired_result.status == .Ok
		{
			paired_markdown: string = render_paired_comparison(paired_request, paired_result.paired_summaries[:]);
			physical: Physical_Summary = result.summaries[0].physical;
			outcome: string = "Pass" if physical.status == .Pass else "Fail" if physical.status == .Fail else "Unassessed";
			for arm in ([2]string{"baseline", "candidate"})
			{
				testing.expect(t, strings.contains(paired_markdown, fmt.tprintf("| 1 | %s | 2 | %s | %d | %d | %d |",
					arm, outcome, physical.passed, physical.failed, physical.unassessed)));
			}
		}
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
	header :: "worker_count,query_count,warmup_batches,measured_batches,case_status,metric_status,physics_elapsed_ms\n"
	_ = write_lane(path, 1, header+"1,100000,10,100,ok,ok,200\n1,100000,10,100,ok,ok,400\n")
	request: Report_Request = test_request(path, {1}, 2)
	request.package_name = "spatial_query_trace"
	result: Report_Result = collect_summaries(request)
	testing.expect_value(t, result.status, Report_Status.Ok)
	if result.status == .Ok
	{
		testing.expect_value(t, result.summaries[0].median, f64(300))
		markdown: string = render_markdown(request, result.summaries[:])
		testing.expect(t, strings.contains(markdown, "| Workers | Samples | Average ms/batch |"))
		testing.expect(t, strings.contains(markdown, "Queries/batch: 100000"))
		testing.expect(t, strings.contains(markdown, "| 1 | 2 | 3.000000 | 2.000000 | 4.000000 |"))
		testing.expect(t, strings.contains(markdown, "37.500000"))
	}
	for count in ([]string{"0", "-1", "99", "NaN"})
	{
		_ = write_lane(path, 1, fmt.aprintf("%s1,100000,10,100,ok,ok,200\n1,100000,10,%s,ok,ok,400\n", header, count))
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)
	}
	_ = write_lane(path, 1, header+"1,99999,10,100,ok,ok,200\n1,99999,10,100,ok,ok,400\n")
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement)

	_ = write_lane(path, 1, "worker_count,query_count,warmup_batches,measured_batches,case_status,metric_status,physics_elapsed_ms\n1,100000,2,3,ok,ok,6\n1,100000,2,3,ok,ok,12\n");
	result = collect_summaries(request);
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status == .Ok
	{
		testing.expect_value(t, result.summaries[0].query.warmup_batches, 2);
		testing.expect(t, strings.contains(render_markdown(request, result.summaries[:]), "| 1 | 2 | 3.000000 | 2.000000 | 4.000000 |"));
	}

	_ = write_lane(path, 1, "worker_count,query_count,measured_batches,case_status,metric_status,physics_elapsed_ms\n1,100000,100,ok,ok,6\n1,100000,100,ok,ok,12\n");
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Header);

}


@(test)
domain_reports_preserve_region_units_and_quality :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	previous: mem.Allocator = context.allocator;
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = previous;
	Fixture :: struct
	{
		package_name, header, baseline, candidate, unit: string,
		metrics: int,
		baseline_cost, candidate_cost: f64,
	}
	fixtures: [6]Fixture = {
		{
			"simulation_stage_work",
			"case_status,metric_status,worker_count,stage,fixture,motion,bodies,statics,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,pair_count,manifold_count,contact_points,precision,benchmark_parameters\n",
			"ok,ok,1,tree-update,container,steady,10000,5,300,30,600,2,pass,300,0,9000,0,0,float32,workload=simulation_stage_work;region=tree-update\n",
			"ok,ok,1,tree-update,container,steady,10000,5,300,30,480,1.6,pass,300,0,9000,0,0,float32,workload=simulation_stage_work;region=tree-update\n",
			"ms/stage call", 1, 2, 1.6,
		},
		{
			"solver_fixed_work",
			"case_status,metric_status,worker_count,case,bodies,admitted_units,work_units,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,residual_max_mps,residual_rms_mps,residual_p95_mps,precision,benchmark_parameters\n",
			"ok,ok,1,convex,10000,20000,8000000,100,10,400,4,pass,100,0,0.1,0.05,0.08,float32,workload=solver_fixed_work;region=prepare-and-native-solver;substeps=4\n",
			"ok,ok,1,convex,10000,20000,8000000,100,10,360,3.6,fail,100,1,0.2,0.1,0.16,float32,workload=solver_fixed_work;region=prepare-and-native-solver;substeps=4\n",
			"ms/solve", 1, 4, 3.6,
		},
		{
			"shape_mixed_bounds",
			"worker_count,body_count,shape_type_count,case_status,metric_status,physics_elapsed_ms,bounds_clustered_ms,measured_passes,warmup_passes,precision,benchmark_parameters\n",
			"1,10000,9,ok,ok,3000,6000,3000,30,float32,workload=shape_mixed_bounds;region=predict-bounds\n",
			"1,10000,9,ok,ok,2700,6600,3000,30,float32,workload=shape_mixed_bounds;region=predict-bounds\n",
			"ms/bounds call", 2, 1, 0.9,
		},
		{
			"world_lifecycle",
			"worker_count,cycles,body_count,case_status,metric_status,physics_elapsed_ms,initialization_ms,growth_ms,cooking_import_ms,clear_reuse_ms,destruction_ms,precision,benchmark_parameters\n",
			"1,20,2048,ok,ok,100,10,20,30,30,10,float32,workload=world_lifecycle;region=five-lifecycle-phases\n",
			"1,20,2048,ok,ok,90,9,18,27,27,9,float32,workload=world_lifecycle;region=five-lifecycle-phases\n",
			"ms/cycle", 6, 5, 4.5,
		},
		{
			"query_extensions",
			"worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries,benchmark_parameters\n",
			"1,ok,ok,2,all,float32,collision-batch,collision,box-native-256,24576,1,24576,256,12288,12288,24576,workload=query_extensions;region=dispatch-query-results\n",
			"1,ok,ok,1.8,all,float32,collision-batch,collision,box-native-256,24576,1,24576,256,12288,12288,24576,workload=query_extensions;region=dispatch-query-results\n",
			"ns/pair", 1, 81.38020833333333, 73.2421875,
		},
		{
			"spatial_query_batch",
			"worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status,precision,ray_pattern,query_execution,benchmark_parameters\n",
			"1,256,50000,10,100,25000,1234,200,ok,float32,oblique,caller,workload=spatial_query_batch;region=prepared-post-join\n",
			"1,256,50000,10,100,25000,1234,180,ok,float32,oblique,caller,workload=spatial_query_batch;region=prepared-post-join\n",
			"ms/batch", 1, 2, 1.8,
		},
	};
	for fixture in fixtures
	{
		baseline_path, candidate_path: string;
		status: Report_Status;
		baseline_path, status = test_directory(fmt.aprintf("entasis-domain-baseline-%s", fixture.package_name));
		testing.expect_value(t, status, Report_Status.Ok);
		defer test_directory_remove(baseline_path);
		candidate_path, status = test_directory(fmt.aprintf("entasis-domain-candidate-%s", fixture.package_name));
		testing.expect_value(t, status, Report_Status.Ok);
		defer test_directory_remove(candidate_path);
		testing.expect_value(t, write_lane(baseline_path, 1, strings.concatenate({fixture.header, fixture.baseline, fixture.baseline})), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({fixture.header, fixture.candidate, fixture.candidate})), Report_Status.Ok);
		request: Report_Request = test_request(candidate_path, {1}, 2);
		request.package_name, request.baseline_source = fixture.package_name, baseline_path;
		result: Report_Result = generate_report(request);
		testing.expect(t, result.status == .Ok, result.diagnostic);
		if result.status != .Ok
		{
			continue;
		}
		paired: Paired_Summary = result.paired_summaries[0];
		testing.expect_value(t, paired.metric_count, fixture.metrics);
		testing.expect_value(t, paired.metrics[0].unit, fixture.unit);
		testing.expect(t, abs(paired.metrics[0].baseline-fixture.baseline_cost) < 1e-10);
		testing.expect(t, abs(paired.metrics[0].candidate-fixture.candidate_cost) < 1e-10);
		testing.expect_value(t, result.summaries[0].primary_unit, fixture.unit);
		testing.expect(t, abs(result.summaries[0].median/result.summaries[0].primary_divisor-fixture.candidate_cost) < 1e-10);
		testing.expect(t, paired.metrics[0].gain > 0);
		markdown: string = result.markdown;
		testing.expect(t, !strings.contains(markdown, "| Workers | Samples | Average"));
		testing.expect(t, !strings.contains(markdown, "| 1 | fixed_elapsed_ms |"));
		testing.expect(t, !strings.contains(markdown, "| 1 | physics_elapsed_ms |"));
		testing.expect(t, !strings.contains(markdown, "| 1 | ray_elapsed_ms |"));
		unpaired: string = render_markdown(request, result.summaries[:]);
		testing.expect_value(t, strings.count(unpaired, "| Workers | Samples | Average"), 1);
		testing.expect(t, !strings.contains(unpaired, "| Workers | Passes | Average ms/pass |"));
		testing.expect(t, !strings.contains(unpaired, "| Workers | Average ms/step |"));
		testing.expect(t, strings.contains(markdown, fixture.unit));
		testing.expect(t, strings.contains(markdown, fmt.tprintf("| Threads | Baseline %s | Candidate %s | Gain %% |", fixture.unit, fixture.unit)));
		testing.expect(t, !strings.contains(markdown, "Time change"));
		testing.expect(t, strings.contains(markdown, "<details><summary>Paired variation and intervals</summary>"));
		testing.expect(t, strings.contains(markdown, "Report generated (UTC)"));
		testing.expect(t, !strings.contains(markdown, "Run date (UTC)"));
		if fixture.package_name == "simulation_stage_work"
		{
			testing.expect(t, strings.contains(markdown, "capacity growth across the measured-frame observation window, including untimed prerequisites and observation"));
			testing.expect(t, strings.contains(markdown, "cannot be attributed solely to the timed stage"));
			testing.expect(t, strings.contains(unpaired, "capacity growth across the measured-frame observation window, including untimed prerequisites and observation"));
		}
		if fixture.package_name == "shape_mixed_bounds"
		{
			testing.expect_value(t, paired.metrics[1].name, "bounds_clustered_ms");
			testing.expect(t, paired.metrics[1].gain < 0);
			changed_count: string;
			changed_count, _ = strings.replace_all(fixture.candidate, ",3000,30,", ",1500,30,");
			testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({fixture.header, fixture.candidate, changed_count})), Report_Status.Ok);
			request.baseline_source = "";
			testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
		}
		if fixture.package_name == "world_lifecycle"
		{
			testing.expect_value(t, paired.metrics[5].name, "destruction_ms");
		}
		if fixture.package_name == "query_extensions"
		{
			testing.expect_value(t, strings.count(markdown, "Selected query: collision-batch/collision/box-native-256 | Inputs: 24576 | Passes: 1 | Pairs: 24576"), 1);
		}
		if fixture.package_name == "spatial_query_batch"
		{
			testing.expect_value(t, strings.count(markdown, "Ray pattern: oblique"), 1);
			testing.expect_value(t, strings.count(markdown, "Query family: all | Queries/batch: 50000 | Warmup batches: 10 | Measured batches: 100"), 1);
			testing.expect_value(t, strings.count(markdown, "Execution: caller | Timer: prepared-post-join"), 1);
		}
		if fixture.package_name == "solver_fixed_work"
		{
			testing.expect_value(t, paired.candidate_domain.covered, 200);
			testing.expect_value(t, paired.candidate_domain.failed, 2);
			testing.expect_value(t, paired.candidate_domain.residuals[0], f64(0.2));
			testing.expect(t, strings.contains(markdown, "prepare-and-native-solver"));
		}
	}
	applicability_header: string = "case_status,metric_status,worker_count,case,bodies,admitted_units,work_units,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,contact_points,residual_max_mps,residual_rms_mps,residual_p95_mps,position_error_m,orientation_norm_error,precision,benchmark_parameters\n";
	baseline_path, candidate_path: string;
	status: Report_Status;
	baseline_path, status = test_directory("entasis-domain-applicability-baseline");
	testing.expect_value(t, status, Report_Status.Ok);
	defer test_directory_remove(baseline_path);
	candidate_path, status = test_directory("entasis-domain-applicability-candidate");
	testing.expect_value(t, status, Report_Status.Ok);
	defer test_directory_remove(candidate_path);
	for case_name in ([9]string{"convex", "cached-convex", "nonconvex", "joints", "nonconserving", "pose", "constrained-final", "free-final", "kinematic"})
	{
		expected_mask: u8 = u8(1)<<4;
		points: int;
		position_text: string = "not assessed";
		residual_text: string = "not assessed | not assessed | not assessed";
		switch case_name
		{
		case "convex", "cached-convex", "nonconvex", "constrained-final":
			expected_mask |= 7;
			points = 1;
			residual_text = "0 | 0 | 0";
		case "joints":
			expected_mask |= u8(1)<<3;
			position_text = "0";
		}
		row: string = fmt.aprintf("ok,ok,1,%s,10000,10000,1000000,100,10,400,4,pass,100,0,%d,0,0,0,0,0,float32,workload=solver_fixed_work;case=%s;region=applicability-fixture\n", case_name, points, case_name);
		rows: string = strings.concatenate({applicability_header, row, row});
		testing.expect_value(t, write_lane(baseline_path, 1, rows), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, 1, rows), Report_Status.Ok);
		request: Report_Request = test_request(candidate_path, {1}, 2);
		request.package_name, request.baseline_source = "solver_fixed_work", baseline_path;
		result: Report_Result = collect_summaries(request);
		testing.expect(t, result.status == .Ok, result.diagnostic);
		if result.status == .Ok
		{
			testing.expect_value(t, result.summaries[0].domain.observed_metrics, expected_mask);
			markdown: string = render_paired_comparison(request, result.paired_summaries[:]);
			expected_row: string = fmt.aprintf("| 1 | candidate | 0 | 0 | %d | %s | %s | 0 | not assessed |", points, residual_text, position_text);
			testing.expect(t, strings.contains(markdown, expected_row), case_name);
		}
		current_header, current_row: string;
		current_header, _ = strings.replace_all(applicability_header, ",precision,", ",joint_angle_error_rad,precision,");
		current_row, _ = strings.replace_all(row, ",float32,", ",0.25,float32,");
		current_rows: string = strings.concatenate({current_header, current_row, current_row});
		testing.expect_value(t, write_lane(baseline_path, 1, current_rows), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, 1, current_rows), Report_Status.Ok);
		current: Report_Result = collect_summaries(request);
		testing.expect(t, current.status == .Ok, current.diagnostic);
		if current.status == .Ok
		{
			if case_name == "joints"
			{
				expected_mask |= u8(1)<<5;
				testing.expect_value(t, current.summaries[0].domain.residuals[5], f64(0.25));
			}
			testing.expect_value(t, current.summaries[0].domain.observed_metrics, expected_mask);
			markdown: string = render_paired_comparison(request, current.paired_summaries[:]);
			angle_text: string = "0.25" if case_name == "joints" else "not assessed";
			expected_row: string = fmt.aprintf("| 1 | candidate | 0 | 0 | %d | %s | %s | 0 | %s |", points, residual_text, position_text, angle_text);
			testing.expect(t, strings.contains(markdown, "Joint angle error rad"));
			testing.expect(t, strings.contains(markdown, expected_row), case_name);
		}
	}
}

@(test)
domain_reports_reject_mismatched_or_incomplete_work :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	previous: mem.Allocator = context.allocator;
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = previous;
	baseline_path, candidate_path: string;
	status: Report_Status;
	baseline_path, status = test_directory("entasis-domain-mismatch-baseline");
	testing.expect_value(t, status, Report_Status.Ok);
	defer test_directory_remove(baseline_path);
	candidate_path, status = test_directory("entasis-domain-mismatch-candidate");
	testing.expect_value(t, status, Report_Status.Ok);
	defer test_directory_remove(candidate_path);
	header: string = "case_status,metric_status,worker_count,stage,fixture,motion,bodies,statics,measured_passes,warmup_passes,fixed_elapsed_ms,pass_ms,physics_status,quality_coverage,quality_failures,precision,benchmark_parameters\n";
	row: string = "ok,ok,1,tree-update,container,steady,10000,5,300,30,600,2,pass,300,0,float32,workload=simulation_stage_work;region=tree-update\n";
	valid: string = strings.concatenate({header, row, row});
	testing.expect_value(t, write_lane(baseline_path, 1, valid), Report_Status.Ok);
	request: Report_Request = test_request(candidate_path, {1}, 2);
	request.package_name, request.baseline_source = "simulation_stage_work", baseline_path;
	for mutation in ([7][2]string{
		{",tree-update,", ",pair-traversal,"},
		{",container,", ",islands,"},
		{"region=tree-update", "region=collision"},
		{",10000,5,", ",9999,5,"},
		{",300,30,", ",299,30,"},
		{",pass,300,0,", ",pass,299,0,"},
		{",float32,", ",mixed64,"},
	})
	{
		invalid: string;
		invalid, _ = strings.replace_all(row, mutation[0], mutation[1]);
		testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({header, invalid, invalid})), Report_Status.Ok);
		testing.expect(t, collect_summaries(request).status != .Ok);
	}
	testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({header, row})), Report_Status.Ok);
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Row_Count);
	duplicate_header: string;
	duplicate_header, _ = strings.replace_all(header, "quality_failures", "quality_coverage");
	testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({duplicate_header, row, row})), Report_Status.Ok);
	testing.expect(t, collect_summaries(request).status != .Ok);
	failed: string;
	failed, _ = strings.replace_all(row, ",pass,300,0,", ",fail,300,1,");
	testing.expect_value(t, write_lane(candidate_path, 1, strings.concatenate({header, failed, failed})), Report_Status.Ok);
	result: Report_Result = collect_summaries(request);
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		testing.expect_value(t, result.paired_summaries[0].candidate_domain.failed, 2);
	}
	request.package_name = "shape_mixed_bounds";
	bounds_header: string = "worker_count,body_count,shape_type_count,case_status,metric_status,physics_elapsed_ms,bounds_clustered_ms,measured_passes,warmup_passes,precision,benchmark_parameters\n";
	for worker in ([2]int{8, 1})
	{
		bounds_row: string = fmt.aprintf("%d,10000,9,ok,ok,3000,6000,3000,30,float32,workload=shape_mixed_bounds;region=predict-bounds\n", worker);
		rows: string = strings.concatenate({bounds_header, bounds_row, bounds_row});
		testing.expect_value(t, write_lane(baseline_path, 1, rows), Report_Status.Ok);
		testing.expect_value(t, write_lane(candidate_path, 1, rows), Report_Status.Ok);
		testing.expect_value(t, collect_summaries(request).status, Report_Status.Ok if worker == 1 else Report_Status.Invalid_Measurement);
		if worker == 1
		{
			historical: string;
			historical, _ = strings.replace_all(rows, "worker_count,", "");
			historical, _ = strings.replace_all(historical, "1,10000,", "10000,");
			testing.expect_value(t, write_lane(baseline_path, 1, historical), Report_Status.Ok);
			testing.expect_value(t, write_lane(candidate_path, 1, historical), Report_Status.Ok);
			testing.expect_value(t, collect_summaries(request).status, Report_Status.Ok);
		}
	}
}
