package benchmark_report

import "core:encoding/csv"
import "core:fmt"
import "core:mem"
import "core:os"
import "core:strings"
import "core:testing"

@(test)
query_case_identity_counts_and_control_history_are_preserved :: proc(t: ^testing.T)
{
	arena: mem.Dynamic_Arena;
	mem.dynamic_arena_init(&arena);
	defer mem.dynamic_arena_destroy(&arena);
	previous: mem.Allocator = context.allocator;
	context.allocator = mem.dynamic_arena_allocator(&arena);
	defer context.allocator = previous;
	header :: "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries\n";
	row :: "1,ok,ok,2,all,float32,hierarchy,ray-direct,mesh-8192,1536,8,12288,128,6144,6144,1536\n";
	records: [][]string;
	error: csv.Error;
	records, error = csv.read_all_from_string(header+row);
	testing.expect(t, error == nil);
	summary: Lane_Summary;
	result: Report_Result;
	summary, result = summarize_records(records, "selected", 1, 1, "query_extensions");
	testing.expect(t, result.status == .Ok, result.diagnostic);
	testing.expect_value(t, summary.query.case_name, "hierarchy");
	testing.expect_value(t, summary.query.cell, "mesh-8192");
	testing.expect_value(t, summary.query_count, 12288);
	testing.expect_value(t, summary.median_throughput, f64(6.144));
	for mutation in ([14][2]string{
		{"hierarchy", "global"}, {"ray-direct", "distance"}, {"mesh-8192", "hull-unknown"},
		{"all", "common"}, {"float32", "unknown"}, {",1536,8,", ",1024,8,"},
		{",8,12288,", ",7,12288,"}, {",12288,128,", ",12287,128,"},
		{",128,6144,", ",127,6144,"}, {",6144,6144,", ",6143,6144,"},
		{",6144,1536", ",6143,1536"}, {",1536\n", ",1535\n"},
		{",ok,ok,2,", ",ok,ok,NaN,"}, {",ok,ok,2,", ",ok,ok,0,"},
	})
	{
		invalid: string;
		invalid, _ = strings.replace_all(row, mutation[0], mutation[1]);
		bad_records: [][]string;
		bad_records, _ = csv.read_all_from_string(fmt.aprintf("%s%s", header, invalid));
		_, result = summarize_records(bad_records, "invalid", 1, 1, "query_extensions");
		testing.expect(t, result.status != .Ok);
	}
	for replacement in ([3]string{"nested-mesh-8192", "mesh-128", "box-1"})
	{
		second: string;
		second, _ = strings.replace_all(row, "mesh-8192", replacement);
		mixed: [][]string;
		mixed, _ = csv.read_all_from_string(fmt.aprintf("%s%s%s", header, row, second));
		_, result = summarize_records(mixed, "mixed", 2, 1, "query_extensions");
		testing.expect(t, result.status != .Ok);
	}
	for selected_profile in ([2]string{"float32", "mixed64"})
	{
		selected_row: string;
		selected_row, _ = strings.replace_all(row, "float32", selected_profile);
		valid: [][]string;
		valid, _ = csv.read_all_from_string(fmt.aprintf("%s%s", header, selected_row));
		_, result = summarize_records(valid, "selected precision", 1, 1, "query_extensions");
		testing.expect(t, result.status == .Ok, result.diagnostic);
	}
	for cell in ([2]string{"hull-32", "hull-256"})
	{
		for operation in ([4]string{"ray-direct", "ray-any", "ray-closest", "ray-all"})
		{
			for selected_profile in ([2]string{"float32", "mixed64"})
			{
				selected: [][]string;
				selected, error = csv.read_all_from_string(fmt.aprintf("%s%d,ok,ok,2,all,%s,hierarchy,%s,%s,1536,8,12288,%d,6144,6144,1536\n", header, 1, selected_profile, operation, cell, 128));
				testing.expect(t, error == nil);
				_, result = summarize_records(selected, "hull cell", 1, 1, "query_extensions");
				testing.expect_value(t, result.status, Report_Status.Ok);
			}
		}
	}
	historical: [][]string;
	for cell in ([5]string{"compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192"})
	{
		for operation in ([4]string{"ray-direct", "ray-any", "ray-closest", "ray-all"})
		{
			expected: Report_Status = .Ok;
			if (cell == "compound-8" || cell == "big-128" || cell == "big-8192") && operation != "ray-direct"
			{
				expected = .Invalid_Measurement;
			}
			hits: int = 12288 if operation == "ray-all" && strings.has_prefix(cell, "nested-") else 6144;
			selected: [][]string;
			selected, error = csv.read_all_from_string(fmt.aprintf("%s%d,ok,ok,2,all,mixed64,hierarchy,%s,%s,1536,8,12288,%d,%d,%d,1536\n", header, 1, operation, cell, 128, hits, hits));
			testing.expect(t, error == nil);
			_, result = summarize_records(selected, "compound cell", 1, 1, "query_extensions");
			testing.expect_value(t, result.status, expected);
			testing.expect_value(t, query_case_admit("global", operation, cell), Report_Status.Invalid_Measurement);
		}
	}
	for cell in ([6]string{"mesh-8192-near", "mesh-8192-far", "mesh-8192-local", "box-near", "box-far", "box-local"})
	{
		for operation in ([4]string{"ray-any", "ray-closest", "ray-all", "ray-direct"})
		{
			for selected_profile in ([2]string{"mixed64", "float32"})
			{
				selected: [][]string;
				selected, error = csv.read_all_from_string(fmt.aprintf("%s1,ok,ok,2,all,%s,global,%s,%s,1536,8,12288,128,6144,6144,1536\n", header, selected_profile, operation, cell));
				testing.expect(t, error == nil);
				_, result = summarize_records(selected, "global ray cell", 1, 1, "query_extensions");
				expected: Report_Status = .Ok if selected_profile == "mixed64" && operation != "ray-direct" else .Invalid_Measurement;
				testing.expect_value(t, result.status, expected);
			}
		}
	}
	historical, _ = csv.read_all_from_string("case_status,metric_status,physics_elapsed_ms,benchmark_components,query_count_per_operation\nok,ok,2,common,8192\n");
	summary, result = summarize_records(historical, "historical control", 1, 1, "query_extensions");
	testing.expect(t, result.status == .Ok, result.diagnostic);
	testing.expect_value(t, summary.query.case_name, "control");
	trace: [][]string;
	trace, _ = csv.read_all_from_string("case_status,metric_status,physics_elapsed_ms,warmup_batches,measured_batches\nok,ok,2,10,100\n");
	_, result = query_measurement_read(trace, "spatial_query_trace", 1);
	testing.expect(t, result.status == .Ok, result.diagnostic);
	path: string;
	directory_status: Report_Status;
	path, directory_status = test_directory("entasis-query-cells");
	if !testing.expect_value(t, directory_status, Report_Status.Ok)
	{
		return;
	}
	defer test_directory_remove(path);
	defer os.remove(fmt.tprintf("%s/workers-16.csv", path));
	testing.expect_value(t, write_lane(path, 16, fmt.aprintf("%s16,ok,ok,2,all,float32,hierarchy,ray-direct,mesh-8192,1536,8,12288,%d,6144,6144,1536\n", header, 128*16)), Report_Status.Ok);
	request: Report_Request = test_request(path, []int{16}, 1);
	request.package_name = "query_extensions";
	request.query = {case_name="hierarchy", operation="ray-direct", cell="mesh-8192", components="all"};
	result = collect_summaries(request);
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		testing.expect(t, strings.contains(render_markdown(request, result.summaries[:]), "unmeasured"));
	}
	dataset: Result_Dataset;
	result = result_dataset_open(&dataset, path, "");
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		testing.expect_value(t, dataset.lanes[0].summary.query.cell, "mesh-8192");
		testing.expect_value(t, dataset.lanes[0].summary.speedup, f64(0));
	}
	result_dataset_close(&dataset);
	request.query.cell = "mesh-128";
	testing.expect_value(t, collect_summaries(request).status, Report_Status.Invalid_Measurement);
	testing.expect_value(t, write_lane(path, 16, fmt.aprintf("%s16,ok,ok,2,all,mixed64,global,ray-closest,mesh-8192-far,1536,8,12288,%d,6144,6144,1536\n", header, 128*16)), Report_Status.Ok);
	request.query = {case_name="global", operation="ray-closest", cell="mesh-8192-far", components="all"};
	request.precision = "mixed64";
	result = collect_summaries(request);
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		testing.expect_value(t, result.summaries[0].query_count, 12288);
		testing.expect_value(t, result.summaries[0].median_throughput, f64(6.144));
		testing.expect(t, strings.contains(render_markdown(request, result.summaries[:]), "unmeasured"));
		testing.expect(t, strings.contains(render_markdown(request, result.summaries[:]), "global/ray-closest/mesh-8192-far"));
	}

	for operation in ([6]string{"closest-point", "distance", "depenetration", "sweep-any", "sweep-closest", "sweep-all"})
	{
		for cell in ([6]string{"nested-query-64", "query-box-1", "compound4-nested64", "compound4-mesh128", "compound4-nested64-angular", "compound4-mesh128-angular"})
		{
			for case_name in ([2]string{"hierarchy", "global"})
			{
				for selected_profile in ([2]string{"float32", "mixed64"})
				{
					hits: int = 12288 if operation == "closest-point" || operation == "distance" || operation == "depenetration" else 6144;
					selected: [][]string;
					selected, error = csv.read_all_from_string(fmt.aprintf("%s%d,ok,ok,2,all,%s,%s,%s,%s,1536,8,12288,%d,%d,%d,1536\n", header, 1, selected_profile, case_name, operation, cell, 128, hits, hits));
					testing.expect(t, error == nil);
					_, result = summarize_records(selected, "hierarchy geometry", 1, 1, "query_extensions");
					expected: Report_Status = .Ok if ((hits == 12288 && (cell == "nested-query-64" || cell == "query-box-1")) ||
						(hits == 6144 && case_name == "hierarchy" && cell != "nested-query-64")) &&
						(case_name != "global" || selected_profile == "mixed64") else .Invalid_Measurement;
					testing.expect_value(t, result.status, expected);
					if expected == .Ok
					{
						selected[1][13] = "6144" if hits == 12288 else "12288";
						_, result = summarize_records(selected, "wrong result count", 1, 1, "query_extensions");
						testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
					}
				}
			}
		}
	}

	Collision_Cell :: struct
	{
		name: string,
		batch_size: int,
	}
	collision_cells: [16]Collision_Cell = {
		{"box-native-1", 1}, {"box-native-8", 8}, {"box-native-32", 32}, {"box-native-256", 256},
		{"box-context-1", 1}, {"box-context-8", 8}, {"box-context-32", 32}, {"box-context-256", 256},
		{"box-native-scalar", 256}, {"box-context-scalar", 256},
		{"sphere-native-256", 256}, {"sphere-context-256", 256},
		{"sphere-native-scalar", 256}, {"sphere-context-scalar", 256},
		{"box-triangle-scalar", 256}, {"box-hull-scalar", 256},
	};
	for cell in collision_cells
	{
		for selected_profile in ([2]string{"float32", "mixed64"})
		{
			selected: [][]string;
			selected, error = csv.read_all_from_string(fmt.aprintf("%s1,ok,ok,2,all,%s,collision-batch,collision,%s,24576,1,24576,%d,12288,12288,24576\n", header, selected_profile, cell.name, cell.batch_size));
			testing.expect(t, error == nil);
			summary, result = summarize_records(selected, "collision batch", 1, 1, "query_extensions");
			testing.expect_value(t, result.status, Report_Status.Ok);
			testing.expect_value(t, summary.query_count, 24576);
			testing.expect_value(t, summary.median_throughput, f64(12.288));
		}
	}
	for worker in ([5]int{1, 8, 16, 24, 32})
	{
		selected: [][]string;
		selected, error = csv.read_all_from_string(fmt.aprintf("%s%d,ok,ok,2,all,float32,hierarchy,ray-direct,mesh-8192,1536,8,12288,%d,6144,6144,1536\n", header, worker, 128*worker));
		testing.expect(t, error == nil);
		_, result = summarize_records(selected, "hierarchy worker warmup", 1, worker, "query_extensions");
		testing.expect_value(t, result.status, Report_Status.Ok);
		for cell in collision_cells[:4]
		{
			selected, error = csv.read_all_from_string(fmt.aprintf("%s%d,ok,ok,2,all,float32,collision-batch,collision,%s,24576,1,24576,%d,12288,12288,24576\n", header, worker, cell.name, cell.batch_size*worker));
			testing.expect(t, error == nil);
			_, result = summarize_records(selected, "collision worker warmup", 1, worker, "query_extensions");
			testing.expect_value(t, result.status, Report_Status.Ok);
		}
	}
	invalid_collision: [][]string;
	invalid_collision, error = csv.read_all_from_string(header+"1,ok,ok,2,all,float32,collision-batch,collision,box-context-256,24576,1,24576,256,12288,12288,24576\n");
	testing.expect(t, error == nil);
	for column in ([7]int{9, 10, 11, 12, 13, 14, 15})
	{
		original: string = invalid_collision[1][column];
		invalid_collision[1][column] = "2";
		_, result = summarize_records(invalid_collision, "invalid collision count", 1, 1, "query_extensions");
		testing.expect_value(t, result.status, Report_Status.Invalid_Measurement);
		invalid_collision[1][column] = original;
	}
	testing.expect_value(t, query_case_admit("collision-batch", "collision", "box-native-7"), Report_Status.Invalid_Measurement);
	testing.expect_value(t, query_case_admit("collision-batch", "distance", "box-native-8"), Report_Status.Invalid_Measurement);
	testing.expect_value(t, write_lane(path, 16, fmt.aprintf("%s16,ok,ok,2,all,float32,collision-batch,collision,box-context-256,24576,1,24576,4096,12288,12288,24576\n", header)), Report_Status.Ok);
	request.query = {case_name="collision-batch", operation="collision", cell="box-context-256", components="all"};
	request.precision = "float32";
	result = collect_summaries(request);
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		rendered: string = render_markdown(request, result.summaries[:]);
		testing.expect(t, strings.contains(rendered, "24,576 pairs"));
		testing.expect(t, strings.contains(rendered, "unmeasured"));
		testing.expect_value(t, result.summaries[0].speedup, f64(0));
	}
	result = result_dataset_open(&dataset, path, "");
	testing.expect(t, result.status == .Ok, result.diagnostic);
	if result.status == .Ok
	{
		testing.expect_value(t, dataset.lanes[0].summary.speedup, f64(0));
		testing.expect_value(t, dataset.lanes[0].summary.query_count, 24576);
	}
	result_dataset_close(&dataset);

}
