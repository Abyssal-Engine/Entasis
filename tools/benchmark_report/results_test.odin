package benchmark_report

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strings"
import "core:testing"

@(test)
visual_result_components_preserve_units_and_absence :: proc(t: ^testing.T)
{
	owned_allocator := context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	path: string;
	status: Report_Status;
	path, status = test_directory("entasis-visual-result-components");
	testing.expect_value(t, status, Report_Status.Ok);
	defer delete(path);
	defer test_directory_remove(path);
	dataset: Result_Dataset;
	defer result_dataset_close(&dataset);
	component_modes: [2]string = {"common", "all"};
	for components in component_modes
	{
		row: string = fmt.aprintf("worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,extra_elapsed_ms\n1,ok,ok,12,%s,%s\n1,ok,ok,18,%s,%s\n",
			components, "excluded" if components == "common" else "2.5", components, "excluded" if components == "common" else "3.5");
		testing.expect_value(t, write_lane(path, 1, row), Report_Status.Ok);
		delete(row);
		result: Report_Result = result_dataset_open(&dataset, path, "custom_extensions");
		testing.expect_value(t, result.status, Report_Status.Ok);
		if result.status != .Ok
		{
			return;
		}
		lane: Result_Lane = dataset.lanes[0];
		testing.expect_value(t, lane.summary.median, f64(15));
		testing.expect_value(t, lane.columns[3].unit, Result_Unit.Milliseconds);
		testing.expect_value(t, lane.values[5].availability,
			Result_Availability.Unavailable if components == "common" else Result_Availability.Available);
		if components == "all"
		{
			testing.expect_value(t, lane.values[5].value, f64(2.5));
			testing.expect_value(t, lane.columns[5].summary, Result_Summary{2, 3, 2.5, 3.5});
		}
		else
		{
			testing.expect_value(t, lane.columns[5].summary.sample_count, 0);
		}
		testing.expect_value(t, lane.columns[3].summary, Result_Summary{2, 15, 12, 18});
	}
	mixed: string = "worker_count,case_status,metric_status,physics_elapsed_ms,extra_elapsed_ms,throughput_per_second\n1,ok,ok,12,excluded,0\n1,ok,ok,18,4,9\n1,ok,ok,15,2,unavailable\n";
	testing.expect_value(t, write_lane(path, 1, mixed), Report_Status.Ok);
	mixed_result: Report_Result = result_dataset_open(&dataset, path, "custom_extensions");
	testing.expect_value(t, mixed_result.status, Report_Status.Ok);
	if mixed_result.status == .Ok
	{
		lane: ^Result_Lane = &dataset.lanes[0];
		testing.expect_value(t, lane.columns[4].summary, Result_Summary{2, 3, 2, 4});
		testing.expect_value(t, lane.columns[5].summary, Result_Summary{2, 4.5, 0, 9});
		testing.expect_value(t, lane.columns[5].unit, Result_Unit.Per_Second);
		testing.expect_value(t, lane.values[4].availability, Result_Availability.Unavailable);
		before: Result_Summary = lane.columns[4].summary;
		required: u64 = result_lane_bytes(lane)+3*size_of(f64);
		bounded: Report_Result = result_lane_summaries(lane, required-1);
		testing.expect_value(t, bounded.status, Report_Status.Budget_Exceeded);
		testing.expect_value(t, bounded.required_bytes, required);
		testing.expect_value(t, lane.columns[4].summary, before);
		bounded = result_lane_summaries(lane, required);
		testing.expect_value(t, bounded.status, Report_Status.Ok);
		testing.expect_value(t, lane.columns[4].summary, before);
	}
	ray: string = "worker_count,batch_size,ray_query_count,warmup_batches,measured_batches,ray_hit_count,ray_checksum,ray_elapsed_ms,case_status\n1,256,50000,10,100,25000,42,250,ok\n";
	testing.expect_value(t, write_lane(path, 1, ray), Report_Status.Ok);
	result: Report_Result = result_dataset_open(&dataset, path, "spatial_query_batch");
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status == .Ok
	{
		testing.expect_value(t, dataset.lanes[0].columns[7].unit, Result_Unit.Milliseconds);
		testing.expect_value(t, dataset.lanes[0].summary.median, f64(250));
		testing.expect_value(t, dataset.lanes[0].records[1][6], "42");
	}
	common: string = "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,cq01_state\n1,ok,ok,12,common,Unavailable\n";
	testing.expect_value(t, write_lane(path, 1, common), Report_Status.Ok);
	result = result_dataset_open(&dataset, path, "query_extensions");
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status == .Ok
	{
		_, absent_status := header_index(dataset.lanes[0].records[0], "cq01_elapsed_ms");
		testing.expect_value(t, absent_status, Report_Status.Invalid_Header);
		testing.expect_value(t, dataset.lanes[0].records[1][5], "Unavailable");
	}

	associated: string = "worker_count,case_status,metric_status,physics_elapsed_ms,extra_elapsed_ms,benchmark_parameters,step_index,recording_path,recording_component,recording_mode,timing_method\n1,ok,ok,12,2.5,workload=contact_islands,1,sample-1.epr,physics_elapsed_ms,on,native_step_sum\n1,ok,ok,18,3.5,workload=contact_islands,1,sample-2.epr,physics_elapsed_ms,on,native_step_sum\n";
	testing.expect_value(t, write_lane(path, 1, associated), Report_Status.Ok);
	result = result_dataset_open(&dataset, path, "contact_islands");
	testing.expect_value(t, result.status, Report_Status.Ok);
	if result.status == .Ok
	{
		lane: ^Result_Lane = &dataset.lanes[0];
		testing.expect_value(t, result_sample_recording(lane, 0, "physics_elapsed_ms"), "sample-1.epr");
		testing.expect_value(t, result_sample_recording(lane, 1, "physics_elapsed_ms"), "sample-2.epr");
		testing.expect_value(t, result_sample_recording(lane, 0, "extra_elapsed_ms"), "");
		testing.expect_value(t, lane.summary.recording_mode, Result_Recording.On);
	}
	overhead: string = "step_10k,1000000,10000,42.000000000\nstep_10k,3000000,10000,42.000000000\nquery_batch,5000,256,8.000000000\n";
	testing.expect_value(t, write_lane(path, 1, overhead), Report_Status.Ok);
	lane_path: string = fmt.aprintf("%s/workers-1.csv", path);
	defer delete(lane_path);
	identities: [3]string = {"c_abi_overhead/native", "c_abi_overhead/c", "c_abi_overhead"};
	for identity in identities
	{
		result = result_dataset_open(&dataset, lane_path, identity);
		testing.expect_value(t, result.status, Report_Status.Ok);
		if result.status == .Ok
		{
			testing.expect_value(t, len(dataset.lanes), 2);
			testing.expect_value(t, dataset.lanes[0].unit, Result_Unit.Nanoseconds);
			testing.expect_value(t, dataset.lanes[0].summary.median, f64(2_000_000));
			testing.expect_value(t, dataset.lanes[0].columns[1].summary, Result_Summary{2, 2_000_000, 1_000_000, 3_000_000});
			testing.expect(t, strings.contains(dataset.lanes[0].label, "step_10k"));
			testing.expect(t, strings.has_prefix(dataset.lanes[0].label,
				"Native | " if identity == "c_abi_overhead/native" else "C | " if identity == "c_abi_overhead/c" else "Lane unavailable | "));
			testing.expect_value(t, dataset.lanes[0].records[1][2], "10000");
			testing.expect_value(t, dataset.lanes[1].summary.sample_count, 1);
		}
	}
	{
		context.allocator = owned_allocator;
		bounded: Result_Dataset;
		defer result_dataset_close(&bounded);
		result = result_dataset_open(&bounded, lane_path, "c_abi_overhead/native", 1);
		testing.expect_value(t, result.status, Report_Status.Budget_Exceeded);
		testing.expect(t, result.required_bytes > result.available_bytes);
		testing.expect_value(t, result_dataset_bytes(&bounded), u64(0));
		result = result_dataset_open(&bounded, lane_path, "c_abi_overhead/native", 65536);
		testing.expect_value(t, result.status, Report_Status.Ok);
		testing.expect(t, result_dataset_bytes(&bounded) < 65536);
		result_dataset_close(&bounded);
		testing.expect_value(t, result_dataset_bytes(&bounded), u64(0));
		long_field: string = strings.repeat("quoted, field\n", 700);
		defer delete(long_field);
		rows: string = fmt.aprintf("worker_count,case_status,metric_status,physics_elapsed_ms,notes\n1,ok,ok,12,\"%s\"\n", long_field);
		defer delete(rows);
		testing.expect_value(t, os.write_entire_file(lane_path, rows), os.Error(nil));
		result = result_dataset_open(&bounded, path, "custom_extensions", 1024*1024);
		testing.expect_value(t, result.status, Report_Status.Ok);
		if result.status == .Ok
		{
			testing.expect_value(t, bounded.lanes[0].records[1][4], long_field);
			testing.expect_value(t, bounded.lanes[0].summary.median, f64(12));
			required: u64 = result_dataset_bytes(&bounded);
			result = result_dataset_open(&bounded, path, "custom_extensions", required-1);
			testing.expect_value(t, result.status, Report_Status.Budget_Exceeded);
			testing.expect(t, result.required_bytes > result.available_bytes);
		}
	}
}

@(test)
visual_result_inputs_reject_mixed_or_invalid_rows :: proc(t: ^testing.T)
{
	owned_allocator := context.allocator;
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	context.allocator = mem.dynamic_arena_allocator(&arena);
	path: string;
	status: Report_Status;
	path, status = test_directory("entasis-visual-result-invalid");
	testing.expect_value(t, status, Report_Status.Ok);
	defer delete(path);
	defer test_directory_remove(path);
	dataset: Result_Dataset;
	defer result_dataset_close(&dataset);
	invalid_rows: [5]string = {
		"worker_count,case_status,metric_status,physics_elapsed_ms\n2,ok,ok,10\n",
		"worker_count,case_status,metric_status,physics_elapsed_ms\n1,ok,ok,NaN\n",
		"worker_count,case_status,metric_status,physics_elapsed_ms,extra_elapsed_ms\n1,ok,ok,10,-1\n",
		"worker_count,case_status,metric_status,physics_elapsed_ms,physics_elapsed_ms\n1,ok,ok,10,12\n",
		"worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components\n1,ok,ok,10,common\n1,ok,ok,12,all\n",
	}
	for rows in invalid_rows
	{
		testing.expect_value(t, write_lane(path, 1, rows), Report_Status.Ok);
		result: Report_Result = result_dataset_open(&dataset, path, "custom_extensions");
		testing.expect(t, result.status != .Ok);
	}

	for association in ([]string{"../sample.epr,physics_elapsed_ms,on,native_step_sum", "sample.epr.partial,physics_elapsed_ms,on,native_step_sum", "sample.epr,extra_elapsed_ms,on,native_step_sum", "sample.epr,physics_elapsed_ms,off,whole_loop", "sample.epr,physics_elapsed_ms,on,whole_loop"})
	{
		rows: string = fmt.tprintf("worker_count,case_status,metric_status,physics_elapsed_ms,recording_path,recording_component,recording_mode,timing_method\n1,ok,ok,10,%s\n", association);
		testing.expect_value(t, write_lane(path, 1, rows), Report_Status.Ok);
		result: Report_Result = result_dataset_open(&dataset, path, "contact_islands");
		testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
		cli: Report_Result = collect_summaries({source_directory=path, package_name="contact_islands", variant="fixture", run_count=1, workers={1}});
		testing.expect_value(t, cli.status, Report_Status.Invalid_Measurement);
	}
	for worker in 1 ..= 2
	{
		rows: string = fmt.aprintf("worker_count,step_index,case_status,metric_status,physics_elapsed_ms,benchmark_parameters\n%d,60,ok,ok,10,workload=container;substeps=%d\n", worker, worker);
		testing.expect_value(t, write_lane(path, worker, rows), Report_Status.Ok);
		delete(rows);
	}
	result: Report_Result = result_dataset_open(&dataset, path, "container");
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
	testing.expect_value(t, write_lane(path, 1, "query_batch,5000,256,8\nquery_batch,6000,512,8\n"), Report_Status.Ok);
	lane_path: string = fmt.aprintf("%s/workers-1.csv", path);
	defer delete(lane_path);
	result = result_dataset_open(&dataset, lane_path, "c_abi_overhead/native");
	testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
	{
		context.allocator = owned_allocator;
		bounded: Result_Dataset;
		defer result_dataset_close(&bounded);
		result = result_dataset_open(&bounded, lane_path, "c_abi_overhead/native");
		testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
		result = result_dataset_open(&bounded, path, "container");
		testing.expect(t, result.status != .Ok);
		result_dataset_close(&bounded);
		testing.expect_value(t, result_dataset_bytes(&bounded), u64(0));
	}
}
