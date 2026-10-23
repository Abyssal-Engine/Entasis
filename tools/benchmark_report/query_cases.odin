package benchmark_report

import "core:strings"

query_case_admit :: proc "contextless" (case_name, operation, cell: string) -> Report_Status
{
	if case_name == "collision-batch" && operation == "collision" && collision_cell_batch_size(cell) > 0
	{
		return .Ok;
	}

	if (case_name == "hierarchy" || case_name == "global") &&
		(operation == "closest-point" || operation == "distance" || operation == "depenetration") &&
		(cell == "nested-query-64" || cell == "query-box-1")
	{
		return .Ok;
	}
	if case_name == "hierarchy" && (operation == "sweep-any" || operation == "sweep-closest" || operation == "sweep-all") &&
		(cell == "compound4-nested64" || cell == "compound4-mesh128" || cell == "compound4-nested64-angular" || cell == "compound4-mesh128-angular" || cell == "query-box-1")
	{
		return .Ok;
	}
	if case_name == "global"
	{
		if operation != "ray-any" && operation != "ray-closest" && operation != "ray-all"
		{
			return .Invalid_Measurement;
		}
		switch cell
		{
		case "mesh-8192-near", "mesh-8192-far", "mesh-8192-local", "box-near", "box-far", "box-local":
			return .Ok;
		}
		return .Invalid_Measurement;
	}
	if case_name != "hierarchy" ||
		(operation != "ray-direct" && operation != "ray-any" && operation != "ray-closest" && operation != "ray-all")
	{
		return .Invalid_Measurement;
	}
	switch cell
	{
	case "mesh-128", "mesh-8192", "nested-mesh-128", "nested-mesh-8192", "box-1", "hull-32", "hull-256":
		return .Ok;
	case "compound-8", "big-128", "big-8192":
		return .Ok if operation == "ray-direct" else .Invalid_Measurement;
	case "nested-compound-8", "nested-big-8192":
		return .Ok;
	}
	return .Invalid_Measurement;
}

collision_cell_batch_size :: proc "contextless" (cell: string) -> int
{
	switch cell
	{
	case "box-native-1", "box-context-1":
		return 1;
	case "box-native-8", "box-context-8":
		return 8;
	case "box-native-32", "box-context-32":
		return 32;
	case "box-native-256", "box-context-256", "sphere-native-256", "sphere-context-256", "box-native-scalar", "box-context-scalar", "sphere-native-scalar", "sphere-context-scalar", "box-triangle-scalar", "box-hull-scalar":
		return 256;
	}
	return 0;
}

query_case_read :: proc(records: [][]string, worker: int) -> (Query_Measurement, Report_Result)
{
	header: []string = records[0];
	names: [14]string = {"query_case", "query_operation", "query_cell", "benchmark_components", "precision", "worker_count", "input_count", "passes", "query_count", "warmup_queries", "hit_count", "checksum", "physics_elapsed_ms", "oracle_queries"};
	columns: [14]int;
	for name, index in names
	{
		status: Report_Status;
		columns[index], status = header_index(header, name);
		if status != .Ok
		{
			return {}, report_error(.Invalid_Header, "missing query cell column: %s", name);
		}
	}
	for name in ([3]string{"query_group", "query_repetitions", "query_warmup_repetitions"})
	{
		status: Report_Status;
		_, status = header_index(header, name);
		if status == .Ok
		{
			return {}, report_error(.Invalid_Header, "query cell excludes group metadata");
		}
	}
	settings: Query_Measurement;
	profile: string;
	for row, row_index in records[1:]
	{
		if len(row) != len(header)
		{
			return {}, report_error(.Csv_Invalid, "query cell column count mismatch");
		}
		current: Query_Measurement = {case_name=row[columns[0]], operation=row[columns[1]], cell=row[columns[2]], components=row[columns[3]], inputs=1536, passes=8, queries=12288};
		if query_case_admit(current.case_name, current.operation, current.cell) != .Ok || current.components != "all" ||
			(row[columns[4]] != "float32" && row[columns[4]] != "mixed64") ||
			(current.case_name == "global" && row[columns[4]] != "mixed64") ||
			(worker != 1 && worker != 8 && worker != 16 && worker != 24 && worker != 32)
		{
			return {}, report_error(.Invalid_Measurement, "invalid query case/operation/cell/components/profile/worker");
		}
		expected_hits: int = 12288 if current.operation == "ray-all" && strings.has_prefix(current.cell, "nested-") else 6144;
		if current.operation == "closest-point" || current.operation == "distance" || current.operation == "depenetration"
		{
			expected_hits = 12288;
		}
		warmup: int = 128*worker;
		if current.case_name == "collision-batch"
		{
			current.inputs, current.passes, current.queries = 24576, 1, 24576;
			expected_hits = 12288;
			warmup = collision_cell_batch_size(current.cell)*worker;
		}
		expected: [7]int = {worker, current.inputs, current.passes, current.queries, warmup, expected_hits, expected_hits};
		for value, index in expected
		{
			parsed: int;
			status: Report_Status;
			parsed, status = query_positive_count(row[columns[index+5]], 1_000_000);
			if status != .Ok || parsed != value
			{
				return {}, report_error(.Invalid_Measurement, "query cell count/oracle mismatch: %s", names[index+5]);
			}
		}
		oracle_count: int;
		oracle_status: Report_Status;
		oracle_count, oracle_status = query_positive_count(row[columns[13]]);
		if oracle_status != .Ok || oracle_count != current.inputs ||
			(row_index > 0 && (settings != current || profile != row[columns[4]]))
		{
			return {}, report_error(.Invalid_Measurement, "invalid or mixed query cell timing/profile/identity");
		}
		settings, profile = current, row[columns[4]];
	}
	return settings, {status=.Ok};
}
