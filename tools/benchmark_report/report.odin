package benchmark_report

import "core:encoding/csv"
import "core:fmt"
import "core:math"
import "core:os"
import "core:path/filepath"
import "core:strconv"
import "core:strings"

Report_Status :: enum
{
	Ok,
	Invalid_Arguments,
	Directory_Read_Failed,
	Unexpected_Lane,
	Missing_Lane,
	File_Read_Failed,
	Csv_Invalid,
	Invalid_Header,
	Invalid_Row_Count,
	Invalid_Status,
	Invalid_Measurement,
	Report_Write_Failed,
	Budget_Exceeded,
	Allocation_Failed,
}

Query_Measurement :: struct
{
	execution, timer: string,
	group, family, components: string,
	case_name, operation, cell: string,
	inputs, passes, queries: int,
	repetitions, warmup_repetitions, warmup_batches, measured_batches: int,
}

Report_Request :: struct
{
	query: Query_Measurement,
	ray_pattern: string,
	precision, precision_candidate, precision_case, sampling_mode: string,
	variant: string,
	source_directory:  string,
	package_name:      string,
	configuration:     string,
	run_count:         int,
	workers:           []int,
	compiler_version:  string,
	operating_system:  string,
	processor:         string,
	logical_processors: int,
	command:           string,
	run_date:          string,
}

Lane_Summary :: struct
{
	query: Query_Measurement,
	profile: string,
	precision: Precision_Result,
	recording_mode: Result_Recording,
	timing_method: Result_Timing,
	benchmark_parameters: string,
	measured_steps: int,
	worker_count: int,
	sample_count: int,
	median:       f64,
	minimum:      f64,
	maximum:      f64,
	speedup:      f64,
	ray_checksum: u64,
	ray_pattern: string,
	measured_units: int,
	query_count: int,
	median_throughput: f64,
	physical: Physical_Summary,
}

Report_Result :: struct
{
	status:    Report_Status,
	diagnostic: string,
	summaries: [dynamic]Lane_Summary,
	markdown:  string,
	required_bytes, available_bytes: u64,
}

report_line :: proc(builder: ^strings.Builder, line := "")
{
	strings.write_string(builder, line)
	strings.write_byte(builder, '\n')
}

report_format_line :: proc(builder: ^strings.Builder, format: string, args: ..any)
{
	fmt.sbprintf(builder, format, ..args)
	strings.write_byte(builder, '\n')
}

report_error :: proc(status: Report_Status, format: string, args: ..any) -> Report_Result
{
	return Report_Result{status = status, diagnostic = fmt.aprintf(format, ..args)}
}

worker_file_name :: proc(worker_count: int) -> string
{
	return fmt.aprintf("workers-%d.csv", worker_count)
}

worker_index :: proc(workers: []int, worker_count: int) -> int
{
	for candidate, index in workers
	{
		if candidate == worker_count
		{
			return index
		}
	}
	return -1
}

validate_worker_contract :: proc(workers: []int, run_count: int) -> Report_Result
{
	if run_count < 1 || len(workers) == 0
	{
		return report_error(.Invalid_Arguments, "runs and workers must be positive")
	}
	for worker, index in workers
	{
		if worker < 1
		{
			return report_error(.Invalid_Arguments, "invalid worker count: %d", worker)
		}
		for prior_index in 0..<index
		{
			if workers[prior_index] == worker
			{
				return report_error(.Invalid_Arguments, "duplicate worker count: %d", worker)
			}
		}
	}
	return Report_Result{status = .Ok}
}

header_index :: proc(header: []string, required_name: string) -> (int, Report_Status)
{
	found_index := -1
	found_count := 0
	for name, index in header
	{
		if name == required_name
		{
			found_index = index
			found_count += 1
		}
	}
	if found_count != 1
	{
		return -1, .Invalid_Header
	}
	return found_index, .Ok
}

sort_measurements :: proc(values: []f64)
{
	for index in 1..<len(values)
	{
		value := values[index]
		cursor := index
		for cursor > 0 && values[cursor - 1] > value
		{
			values[cursor] = values[cursor - 1]
			cursor -= 1
		}
		values[cursor] = value
	}
}

summarize_csv :: proc(path: string, expected_rows, worker_count: int, package_name: string = "") -> (Lane_Summary, Report_Result)
{
	data, read_error := os.read_entire_file(path, context.allocator)
	if read_error != nil
	{
		return {}, report_error(.File_Read_Failed, "failed to read lane file: %s", path)
	}
	records, csv_error := csv.read_all_from_string(string(data), context.allocator, context.allocator)
	if csv_error != nil || len(records) == 0
	{
		return {}, report_error(.Csv_Invalid, "invalid CSV: %s", path)
	}
	return summarize_records(records, path, expected_rows, worker_count, package_name)
}

query_positive_count :: proc "contextless" (value: string, limit: u64 = 1_000_000) -> (int, Report_Status)
{
	if len(value) == 0
	{
		return 0, .Invalid_Measurement;
	}
	count: u64;
	for character in value
	{
		if character < '0' || character > '9' || count > (limit-u64(character-'0'))/10
		{
			return 0, .Invalid_Measurement;
		}
		count = count*10+u64(character-'0');
	}
	return int(count), .Ok if count > 0 && count <= limit else .Invalid_Measurement;
}

query_group_column :: proc(group, components: string) -> (string, Report_Status)
{
	routes: [4]string = {"convex", "compound", "mesh", "native_custom"};
	operations: [12]string = {"ray_closest", "ray_filtered", "ray_all", "sweep_closest", "sweep_all", "geometric_overlap", "direct_collision", "collision_batch", "closest_point", "shape_distance", "depenetration", "overlap_any"};
	for route in routes
	{
		for operation, index in operations
		{
			if index >= 8 && components != "all"
			{
				continue;
			}
			if group == fmt.tprintf("%s.%s", route, operation)
			{
				return fmt.aprintf("%s_%s", route, operation), .Ok;
			}
		}
	}
	return "", .Invalid_Measurement;
}

query_measurement_read :: proc(records: [][]string, package_name: string, worker: int) -> (Query_Measurement, Report_Result)
{
	settings: Query_Measurement;
	if package_name != "query_extensions" && package_name != "spatial_query_trace" && package_name != "spatial_query_batch"
	{
		return settings, {status=.Ok};
	}
	header: []string = records[0];
	for name, index in header
	{
		for prior in header[:index]
		{
			if name == prior
			{
				return {}, report_error(.Invalid_Header, "duplicate query header: %s", name);
			}
		}
	}
	if _, status := header_index(header, "query_case"); status == .Ok
	{
		if package_name != "query_extensions"
		{
			return {}, report_error(.Invalid_Header, "query case requires query_extensions");
		}
		return query_case_read(records, worker);
	}
	for name in ([2]string{"query_operation", "query_cell"})
	{
		if _, status := header_index(header, name); status == .Ok
		{
			return {}, report_error(.Invalid_Header, "partial query case metadata");
		}
	}
	group_column, group_status := header_index(header, "query_group");
	repetitions_column, repetitions_status := header_index(header, "query_repetitions");
	warmup_column, warmup_status := header_index(header, "query_warmup_repetitions");
	family_column, family_status := header_index(header, "query_family");
	metadata_count: int = (1 if group_status == .Ok else 0)+(1 if repetitions_status == .Ok else 0)+(1 if warmup_status == .Ok else 0);
	if (metadata_count != 0 && metadata_count != 3) || (metadata_count > 0 && package_name != "query_extensions") ||
		(family_status == .Ok && package_name != "spatial_query_trace")
	{
		return {}, report_error(.Invalid_Header, "partial or inapplicable query selection metadata");
	}
	for row, row_index in records[1:]
	{
		if len(row) != len(header)
		{
			return {}, report_error(.Csv_Invalid, "query column count mismatch");
		}
		current: Query_Measurement;
		if package_name == "spatial_query_batch"
		{
			current.execution = "caller";
			current.timer = "legacy-worker-construction";
			if column, status := header_index(header, "query_execution"); status == .Ok
			{
				current.execution = row[column];
				current.timer = "prepared-post-join";
				if current.execution != "caller" && current.execution != "mailbox"
				{
					return {}, report_error(.Invalid_Measurement, "invalid query execution");
				}
			}
		}
		if package_name == "query_extensions"
		{
			current.case_name = "control";
		}
		if column, status := header_index(header, "worker_count"); status == .Ok
		{
			value, parsed := strconv.parse_i64(row[column]);
			if !parsed || value != i64(worker)
			{
				return {}, report_error(.Invalid_Measurement, "query worker mismatch");
			}
		}
		if column, status := header_index(header, "benchmark_components"); status == .Ok
		{
			current.components = row[column];
			if current.components != "common" && current.components != "all"
			{
				return {}, report_error(.Invalid_Measurement, "invalid query components");
			}
		}
		if metadata_count == 3
		{
			current.group = row[group_column];
			selected, selection_status := query_group_column(current.group, current.components);
			defer delete(selected);
			var_status: Report_Status;
			current.repetitions, var_status = query_positive_count(row[repetitions_column]);
			current.warmup_repetitions, warmup_status = query_positive_count(row[warmup_column]);
			count_column, count_status := header_index(header, "query_count_per_operation");
			if selection_status != .Ok || var_status != .Ok || warmup_status != .Ok || count_status != .Ok
			{
				return {}, report_error(.Invalid_Measurement, "invalid focused query selection or counts");
			}
			calls, calls_status := query_positive_count(row[count_column], 8192*1_000_000);
			if calls_status != .Ok || u64(calls) != 8192*u64(current.repetitions)
			{
				return {}, report_error(.Invalid_Measurement, "focused query call accounting mismatch");
			}
			state_column, state_status := header_index(header, "cq01_state");
			if state_status != .Ok || row[state_column] != "Excluded"
			{
				return {}, report_error(.Invalid_Measurement, "focused query total must be excluded");
			}
			if column, status := header_index(header, "cq01_elapsed_ms"); status == .Ok && row[column] != "excluded"
			{
				return {}, report_error(.Invalid_Measurement, "focused additional total must be excluded");
			}
			selected_columns, group_columns: int;
			for name, column in header
			{
				if name == "physics_elapsed_ms" || name == "cq01_elapsed_ms"
				{
					continue;
				}
				if !strings.has_suffix(name, "_ms") && !strings.has_suffix(name, "_hits") && !strings.has_suffix(name, "_checksum")
				{
					continue;
				}
				group_columns += 1;
				if name == fmt.tprintf("%s_ms", selected) || name == fmt.tprintf("%s_hits", selected) || name == fmt.tprintf("%s_checksum", selected)
				{
					value, parsed := strconv.parse_f64(row[column]);
					if !parsed || math.is_nan(value) || math.is_inf(value) || value < 0 || (strings.has_suffix(name, "_ms") && value <= 0)
					{
						return {}, report_error(.Invalid_Measurement, "invalid executed focused group");
					}
					if strings.has_suffix(name, "_ms")
					{
						total_column, total_status := header_index(header, "physics_elapsed_ms");
						if total_status != .Ok || row[column] != row[total_column]
						{
							return {}, report_error(.Invalid_Measurement, "focused elapsed total differs from selected group");
						}
					}
					selected_columns += 1;
				}
				else if row[column] != "excluded"
				{
					return {}, report_error(.Invalid_Measurement, "unselected query group must be excluded");
				}
			}
			if selected_columns != 3 || group_columns != (144 if current.components == "all" else 96)
			{
				return {}, report_error(.Invalid_Header, "missing focused group columns");
			}
		}
		if package_name == "query_extensions" && metadata_count == 0
		{
			if column, status := header_index(header, "query_count_per_operation"); status == .Ok
			{
				count, count_status := query_positive_count(row[column], 8192);
				if count_status != .Ok || count != 8192
				{
					return {}, report_error(.Invalid_Measurement, "complete extension workload requires 8192 calls per operation");
				}
			}
			for name, column in header
			{
				if name != "cq01_elapsed_ms" && (strings.has_suffix(name, "_ms") || strings.has_suffix(name, "_hits") || strings.has_suffix(name, "_checksum")) && row[column] == "excluded"
				{
					return {}, report_error(.Invalid_Measurement, "excluded query group requires focused metadata");
				}
			}
		}
		if package_name == "spatial_query_trace" || package_name == "spatial_query_batch"
		{
			current.family = "all";
			current.warmup_batches = 10;
			if family_status == .Ok
			{
				current.family = row[family_column];
			}
			if current.family != "all" && current.family != "ray" && current.family != "sphere-cast" && current.family != "aabb-presence"
			{
				return {}, report_error(.Invalid_Measurement, "invalid query family");
			}
			for name, index in ([2]string{"warmup_batches", "measured_batches"})
			{
				column, status := header_index(header, name);
				if status != .Ok
				{
					return {}, report_error(.Invalid_Header, "missing spatial work count");
				}
				count, count_status := query_positive_count(row[column]);
				if count_status != .Ok
				{
					return {}, report_error(.Invalid_Measurement, "invalid spatial work count");
				}
				if index == 0
				{
					current.warmup_batches = count;
				}
				else
				{
					current.measured_batches = count;
				}
			}
			if current.family != "all"
			{
				if column, status := header_index(header, "ray_pattern"); status == .Ok && row[column] == "oblique" && current.family != "ray"
				{
					return {}, report_error(.Invalid_Measurement, "oblique requires rays");
				}
			}
			if package_name == "spatial_query_trace"
			{
				total: f64;
				for family, index in ([3]string{"ray", "sphere_cast", "overlap"})
				{
					family_names: [3]string = {"ray", "sphere-cast", "aabb-presence"};
					family_name: string = family_names[index];
					for suffix in ([5]string{"_query_count", "_hit_count", "_checksum", "_elapsed_ms", "_queries_per_second"})
					{
						name: string = fmt.tprintf("%s%s", family, suffix);
						column, status := header_index(header, name);
						if status != .Ok && family_status != .Ok
						{
							continue;
						}
						if status != .Ok
						{
							return {}, report_error(.Invalid_Header, "missing selected spatial family column: %s", name);
						}
						if current.family != "all" && current.family != family_name
						{
							if row[column] != "excluded"
							{
								return {}, report_error(.Invalid_Measurement, "skipped spatial family must be excluded");
							}
							continue;
						}
						value, parsed := strconv.parse_f64(row[column]);
						if !parsed || math.is_nan(value) || math.is_inf(value) || value <= 0 ||
							(suffix == "_query_count" && value != f64(50_000 if index == 0 else 25_000)) ||
							(suffix == "_hit_count" && value != f64(25_000 if index == 0 else 12_500))
						{
							return {}, report_error(.Invalid_Measurement, "invalid executed spatial family");
						}
						if suffix == "_elapsed_ms"
						{
							total += value;
						}
					}
				}
				if total > 0
				{
					column, status := header_index(header, "physics_elapsed_ms");
					if status != .Ok
					{
						return {}, report_error(.Invalid_Header, "missing spatial total");
					}
					value, parsed := strconv.parse_f64(row[column]);
					if !parsed || abs(value-total) > 0.00000001
					{
						return {}, report_error(.Invalid_Measurement, "spatial total differs from executed families");
					}
				}
			}
		}
		if row_index > 0 && current != settings
		{
			return {}, report_error(.Invalid_Measurement, "query selection, counts or components differ within lane");
		}
		settings = current;
	}
	return settings, {status=.Ok};
}

summarize_records :: proc(records: [][]string, path: string, expected_rows, worker_count: int, package_name: string) -> (Lane_Summary, Report_Result)
{
	if len(records) != expected_rows + 1
	{
		return {}, report_error(
			.Invalid_Row_Count,
			"wrong row count in %s: expected %d, got %d",
			path,
			expected_rows,
			len(records) - 1,
		)
	}
	query, query_status := query_measurement_read(records, package_name, worker_count);
	if query_status.status != .Ok
	{
		return {}, query_status;
	}
	profile: string;
	profile_status: Report_Result;
	profile, profile_status = result_precision_profile(records);
	if profile_status.status != .Ok
	{
		return {}, profile_status;
	}
	precision: Precision_Result;
	if package_name == "large_world_precision"
	{
		precision_status: Report_Result;
		precision, precision_status = precision_result_summary(records, worker_count);
		if precision_status.status != .Ok
		{
			return {}, precision_status;
		}
		if precision.mode == "memory"
		{
			return {profile=profile, precision=precision, worker_count=worker_count, sample_count=expected_rows, recording_mode=.Off, timing_method=.Allocator_Instrumented}, {status=.Ok};
		}
	}
	recording_mode, timing_method, recording_status := result_recording_conditions(records);
	if recording_status.status != .Ok
	{
		return {}, recording_status;
	}
	physical: Physical_Summary
	physical_result: Report_Result
	physical, physical_result = physical_result_summary(records, package_name)
	if physical_result.status != .Ok
	{
		return {}, physical_result
	}
	case_status_index, case_header_status := header_index(records[0], "case_status")
	measurement_name: string = "physics_elapsed_ms"
	metric_status_name: string = "metric_status"
	if package_name == "spatial_query_batch"
	{
		measurement_name = "ray_elapsed_ms"
		metric_status_name = "case_status"
	}
	metric_status_index, metric_header_status := header_index(records[0], metric_status_name)
	measurement_index, measurement_header_status := header_index(records[0], measurement_name)
	if case_header_status != .Ok || metric_header_status != .Ok || measurement_header_status != .Ok
	{
		return {}, report_error(.Invalid_Header, "required header missing or duplicated in %s", path)
	}
	measurements := make([]f64, expected_rows)
	defer delete(measurements)
	throughputs: []f64
	if package_name == "spatial_query_batch" || package_name == "spatial_query_trace"
	{
		throughputs = make([]f64, expected_rows)
	}
	defer delete(throughputs)
	ray_checksum: u64
	parameters: string
	measured_steps := 0
	parameter_columns: [3]int
	if workload_kind(package_name) == .Parameterized
	{
		names := [3]string{"benchmark_parameters", "step_index", "worker_count"}
		for name, index in names
		{
			column, status := header_index(records[0], name)
			if status != .Ok
			{
				return {}, report_error(.Invalid_Header, "required configuration header missing or duplicated: %s", name)
			}
			parameter_columns[index] = column
		}
	}
	ray_pattern: string;
	ray_pattern_column: int = -1;
	if package_name == "spatial_query_batch" || package_name == "spatial_query_trace"
	{
		ray_pattern = "axial";
		for name, index in records[0]
		{
			if name == "ray_pattern"
			{
				if ray_pattern_column >= 0
				{
					return {}, report_error(.Invalid_Header, "duplicated ray_pattern in %s", path);
				}
				ray_pattern_column = index;
			}
		}
	}
	ray_columns: [7]int
	if package_name == "spatial_query_batch"
	{
		names: [7]string = {"worker_count", "batch_size", "ray_query_count", "warmup_batches", "measured_batches", "ray_hit_count", "ray_checksum"}
		if len(records[0]) != len(names) + 2 + (1 if len(profile) > 0 else 0) + (1 if ray_pattern_column >= 0 else 0) + (1 if query.timer == "prepared-post-join" else 0)
		{
			return {}, report_error(.Invalid_Header, "unexpected ray CSV columns in %s", path)
		}
		for name, index in names
		{
			column, status := header_index(records[0], name)
			if status != .Ok
			{
				return {}, report_error(.Invalid_Header, "required ray header missing or duplicated: %s in %s", name, path)
			}
			ray_columns[index] = column
		}
	}
	unit_column, query_column: int
	measured_units, query_count: int
	if package_name == "spatial_query_batch" || package_name == "spatial_query_trace"
	{
		unit_status, query_status: Report_Status
		unit_column, unit_status = header_index(records[0], "measured_batches")
		query_column, query_status = header_index(records[0], "ray_query_count" if package_name == "spatial_query_batch" else "query_count")
		if unit_status != .Ok || query_status != .Ok
		{
			return {}, report_error(.Invalid_Header, "query normalization requires measured batch and query counts")
		}
	}
	for row_index in 0..<expected_rows
	{
		record := records[row_index + 1]
		if len(record) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "column count mismatch in %s row %d", path, row_index + 2)
		}
		if ray_pattern_column >= 0
		{
			pattern: string = record[ray_pattern_column];
			if (pattern != "axial" && pattern != "oblique") || (row_index > 0 && pattern != ray_pattern)
			{
				return {}, report_error(.Invalid_Measurement, "invalid or mixed ray pattern in %s row %d", path, row_index+2);
			}
			ray_pattern = pattern;
		}
		if record[case_status_index] != "ok" || record[metric_status_index] != "ok"
		{
			return {}, report_error(.Invalid_Status, "non-ok status in %s row %d", path, row_index + 2)
		}
		if package_name == "spatial_query_batch"
		{
			expected: [6]u64 = {u64(worker_count), 256, 50_000, u64(query.warmup_batches), u64(query.measured_batches), 25_000}
			for column, index in ray_columns
			{
				value, parsed := strconv.parse_u64(record[column])
				if !parsed || (index < len(expected) && value != expected[index])
				{
					return {}, report_error(.Invalid_Measurement, "ray workload mismatch in %s row %d", path, row_index + 2)
				}
				if index == len(expected)
				{
					if row_index > 0 && value != ray_checksum
					{
						return {}, report_error(.Invalid_Measurement, "ray checksum mismatch in %s row %d", path, row_index + 2)
					}
					ray_checksum = value
				}
			}
		}
		if workload_kind(package_name) == .Parameterized
		{
			row_parameters := record[parameter_columns[0]]
			steps, steps_parsed := strconv.parse_i64(record[parameter_columns[1]])
			worker, worker_parsed := strconv.parse_i64(record[parameter_columns[2]])
			if len(row_parameters) == 0 || !steps_parsed || steps <= 0 || steps > 1_000_000 || !worker_parsed || worker != i64(worker_count) ||
			(row_index > 0 && (parameters != row_parameters || measured_steps != int(steps)))
			{
				return {}, report_error(.Invalid_Measurement, "configuration, measured steps or worker mismatch in %s row %d", path, row_index+2)
			}
			parameters = row_parameters
			measured_steps = int(steps)
		}
		if package_name == "spatial_query_batch" || package_name == "spatial_query_trace"
		{
			units, units_parsed := strconv.parse_i64(record[unit_column])
			queries, queries_parsed := strconv.parse_i64(record[query_column])
			if !units_parsed || units <= 0 || units > 1_000_000 || !queries_parsed || queries <= 0 || queries > i64(max(i32)) ||
				(package_name == "spatial_query_trace" && queries != i64(100_000 if query.family == "all" else 50_000 if query.family == "ray" else 25_000)) ||
				(row_index > 0 && (units != i64(measured_units) || queries != i64(query_count)))
			{
					return {}, report_error(.Invalid_Measurement, "query measured counts differ or are nonpositive")
			}
			measured_units, query_count = int(units), int(queries)
		}
		measurement, parsed := strconv.parse_f64(record[measurement_index])
		if !parsed || math.is_nan(measurement) || math.is_inf(measurement) || measurement <= 0
		{
			return {}, report_error(.Invalid_Measurement, "invalid %s in %s row %d", measurement_name, path, row_index + 2)
		}
		measurements[row_index] = measurement
		if len(throughputs) > 0
		{
			throughputs[row_index] = f64(query_count)*f64(measured_units)/(measurement*1000)
		}
	}
	sort_measurements(measurements)
	middle := expected_rows / 2
	median := measurements[middle]
	if expected_rows % 2 == 0
	{
		median = (measurements[middle - 1] + measurements[middle]) / 2
	}
	median_throughput: f64
	if len(throughputs) > 0
	{
		sort_measurements(throughputs)
		median_throughput = throughputs[middle]
		if expected_rows%2 == 0
		{
			median_throughput = (throughputs[middle-1]+throughputs[middle])/2
		}
	}
	if query.case_name == "hierarchy" || query.case_name == "global" || query.case_name == "collision-batch"
	{
		query_count, measured_units = query.queries, 1;
		median_throughput = f64(query.queries)*0.001/median;
	}
	return Lane_Summary{
		query=query,
		profile=profile, precision=precision,
		recording_mode=recording_mode, timing_method=timing_method,
		worker_count = worker_count,
		sample_count = expected_rows,
		median = median,
		minimum = measurements[0],
		maximum = measurements[len(measurements) - 1],
		ray_checksum = ray_checksum,
		ray_pattern = ray_pattern,
		measured_units=measured_units, query_count=query_count, median_throughput=median_throughput, physical=physical,
		benchmark_parameters = parameters,
		measured_steps = measured_steps,
	}, Report_Result{status = .Ok}
}

collect_summaries :: proc(request: Report_Request) -> Report_Result
{
	if (workload_kind(request.package_name) == .Parameterized && request.package_name != "large_world_precision" && variant_status(request.variant) != .Ok) ||
	(workload_kind(request.package_name) == .Legacy && len(request.variant) != 0)
	{
		return report_error(.Invalid_Arguments, "variant must identify a configurable workload")
	}
	contract_result := validate_worker_contract(request.workers, request.run_count)
	if contract_result.status != .Ok
	{
		return contract_result
	}
	entries, directory_error := os.read_all_directory_by_path(request.source_directory, context.allocator)
	if directory_error != nil
	{
		return report_error(.Directory_Read_Failed, "failed to read source directory: %s", request.source_directory)
	}
	found := make([]u8, len(request.workers))
	for entry in entries
	{
		if entry.type != .Regular || !strings.has_suffix(entry.name, ".csv")
		{
			continue
		}
		matched_index := -1
		for worker, worker_position in request.workers
		{
			if entry.name == worker_file_name(worker)
			{
				matched_index = worker_position
				break
			}
		}
		if matched_index < 0
		{
			return report_error(.Unexpected_Lane, "unexpected CSV lane: %s", entry.name)
		}
		if found[matched_index] != 0
		{
			return report_error(.Unexpected_Lane, "duplicate CSV lane: %s", entry.name)
		}
		found[matched_index] = 1
	}
	for marker, index in found
	{
		if marker == 0
		{
			return report_error(.Missing_Lane, "missing CSV lane: %s", worker_file_name(request.workers[index]))
		}
	}
	summaries := make([dynamic]Lane_Summary, 0, len(request.workers))
	for worker in request.workers
	{
		path, join_error := filepath.join([]string{request.source_directory, worker_file_name(worker)})
		if join_error != nil
		{
			return report_error(.Invalid_Arguments, "invalid lane path for worker %d", worker)
		}
		summary, summary_result := summarize_csv(path, request.run_count, worker, request.package_name)
		if summary_result.status != .Ok
		{
			return summary_result
		}
		if len(request.precision) > 0 && summary.profile != request.precision
		{
			return report_error(.Invalid_Measurement, "compiled precision differs from prepared artifact");
		}
		if request.package_name == "large_world_precision" &&
			(summary.precision.candidate != request.precision_candidate || summary.precision.precision_case != request.precision_case || summary.precision.mode != request.sampling_mode)
		{
				return report_error(.Invalid_Measurement, "precision output differs from requested cell");
		}
		if (request.query.case_name != "" && request.query.case_name != summary.query.case_name) ||
			(request.query.operation != "" && request.query.operation != summary.query.operation) ||
			(request.query.cell != "" && request.query.cell != summary.query.cell) ||
			((request.query.group != "" || request.query.components != "") && request.query.group != summary.query.group) ||
			(request.query.family != "" && request.query.family != summary.query.family) ||
			(request.query.components != "" && request.query.components != summary.query.components) ||
			(request.query.repetitions > 0 && request.query.repetitions != summary.query.repetitions) ||
			(request.query.warmup_repetitions > 0 && request.query.warmup_repetitions != summary.query.warmup_repetitions) ||
			(request.query.warmup_batches > 0 && request.query.warmup_batches != summary.query.warmup_batches) ||
			(request.query.measured_batches > 0 && request.query.measured_batches != summary.query.measured_batches) ||
			(request.ray_pattern != "" && request.ray_pattern != summary.ray_pattern) ||
			(request.query.execution != "" && request.query.execution != summary.query.execution)
		{
			return report_error(.Invalid_Measurement, "emitted query settings differ from requested Run");
		}
		if len(summaries) > 0 && (summary.query != summaries[0].query || summary.profile != summaries[0].profile)
		{
			return report_error(.Invalid_Measurement, "query selection, counts, components or precision differ across workers");
		}
		if len(summaries) > 0 && summary.ray_pattern != summaries[0].ray_pattern
		{
			return report_error(.Invalid_Measurement, "ray pattern differs across worker counts in %s", path);
		}
		if request.package_name == "spatial_query_batch" && len(summaries) > 0 &&
			summary.ray_checksum != summaries[0].ray_checksum
		{
				return report_error(.Invalid_Measurement, "ray checksum differs across worker counts in %s", path)
		}
		if workload_kind(request.package_name) == .Parameterized && len(summaries) > 0 &&
		(summary.benchmark_parameters != summaries[0].benchmark_parameters || summary.measured_steps != summaries[0].measured_steps)
		{
			return report_error(.Invalid_Measurement, "configuration differs across worker counts in %s", path)
		}
		if len(summaries) > 0 && (summary.recording_mode != summaries[0].recording_mode || summary.timing_method != summaries[0].timing_method)
		{
			return report_error(.Invalid_Measurement, "mixed recording or timing conditions");
		}
		if len(summaries) > 0 && (summary.physical.mode != summaries[0].physical.mode || summary.physical.criterion != summaries[0].physical.criterion ||
			summary.physical.shape_limit != summaries[0].physical.shape_limit || summary.measured_units != summaries[0].measured_units || summary.query_count != summaries[0].query_count)
		{
				return report_error(.Invalid_Measurement, "incompatible physical verification or query counts across workers")
		}
		append(&summaries, summary)
	}
	smallest_worker_index := 0
	for summary, index in summaries
	{
		if summary.worker_count < summaries[smallest_worker_index].worker_count
		{
			smallest_worker_index = index
		}
	}
	baseline := summaries[smallest_worker_index].median
	for &summary in summaries
	{
		if summary.precision.mode != "memory"
		{
			summary.speedup = baseline / summary.median;
			if (summary.query.case_name == "hierarchy" || summary.query.case_name == "global" || summary.query.case_name == "collision-batch") && summaries[smallest_worker_index].worker_count != 1
			{
				summary.speedup = 0;
			}
		}
	}
	return Report_Result{status = .Ok, summaries = summaries}
}

render_markdown :: proc(request: Report_Request, summaries: []Lane_Summary) -> string
{
	builder := strings.builder_make_len_cap(0, 4096)
	compiler_version: string = request.compiler_version
	version_start: int = strings.last_index(compiler_version, " version ")
	if version_start >= 0
	{
		compiler_version = compiler_version[version_start + len(" version "):]
	}
	if summaries[0].query.group != ""
	{
		report_format_line(&builder, "# Benchmark result: %s (%s)", request.package_name, summaries[0].query.group);
	}
	else
	{
		report_format_line(&builder, "# Benchmark result: %s", request.package_name);
	}
	report_line(&builder)
	report_format_line(&builder, "- Configuration: `%s`", request.configuration)
	report_format_line(&builder, "- Recording: %v | Timing: %v", summaries[0].recording_mode, summaries[0].timing_method)
	report_format_line(&builder, "- Run date (UTC): `%s`", request.run_date)
	report_format_line(&builder, "- Odin compiler: `%s`", compiler_version)
	report_format_line(&builder, "- Operating system: `%s`", request.operating_system)
	report_format_line(&builder, "- Processor: `%s`", request.processor)
	report_format_line(&builder, "- Logical processors: `%d`", request.logical_processors)
	report_line(&builder)
	if request.package_name == "large_world_precision"
	{
		report_format_line(&builder, "Precision: %s | Candidate: %s | Case: %s | Mode: %s", request.precision, request.precision_candidate, request.precision_case, request.sampling_mode);
		if request.sampling_mode == "memory"
		{
			report_line(&builder, "Memory observations from separate allocator-instrumented processes");
			report_line(&builder, "| Workers | Field | Maximum observed value |");
			report_line(&builder, "| --- | --- | --- |");
			for summary in summaries
			{
				for name, index in PRECISION_MEMORY_NAMES
				{
					report_format_line(&builder, "| %d | %s | %d |", summary.worker_count, name, summary.precision.memory[index]);
				}
			}
			return strings.to_string(builder);
		}
		report_line(&builder, "| Workers | Observations | Frame median ms, maximum across processes | Frame maximum ms | Maintenance events | Maintenance maximum ms |");
		report_line(&builder, "| --- | --- | --- | --- | --- | --- |");
		for summary in summaries
		{
			if request.sampling_mode == "batch"
			{
				report_format_line(&builder, "| %d | %d | unavailable | unavailable | %d | unavailable |", summary.worker_count, summary.precision.count, summary.precision.maintenance_count);
			}
			else
			{
				report_format_line(&builder, "| %d | %d | %.9f | %.9f | %d | %.9f |", summary.worker_count, summary.precision.count, summary.precision.frame_median_ms, summary.precision.frame_max_ms, summary.precision.maintenance_count, summary.precision.maintenance_max_ms);
			}
		}
	}
	if workload_kind(request.package_name) == .Parameterized
	{
		report_format_line(&builder, "Variant: `%s`", request.variant)
		report_line(&builder)
		report_line(&builder, "Effective benchmark parameters:")
		report_line(&builder)
		report_line(&builder, "```text")
		report_line(&builder, summaries[0].benchmark_parameters)
		report_line(&builder, "```")
		report_line(&builder)
	}
	report_line(&builder, "## Reproduction")
	report_line(&builder)
	report_line(&builder, "```text")
	report_line(&builder, request.command)
	report_line(&builder, "```")
	report_line(&builder)
	if request.package_name == "spatial_query_batch"
	{
		report_line(&builder, "## Closest-ray time summary")
		report_line(&builder)
		if summaries[0].query.timer == "prepared-post-join"
		{
			report_format_line(&builder, "Execution: %s | Timer: prepared-post-join. Prepared inputs and reservations are outside timing. Dispatch/session, query result writes and common caller validation/checksum are timed. Scalar reference validation runs before and after timing", summaries[0].query.execution);
		}
		else
		{
			report_line(&builder, "Public query_batch, batch size 256. Times are native totals divided by measured frames, in ms/frame (50,000 rays). Lower is better. Work counts are stated below. Scalar output validation runs before and after timing. Worker counts include the caller. Windows controls placement without affinity pinning")
		}
	}
	else if request.package_name == "spatial_query_trace"
	{
		report_line(&builder, "## Mixed-query time summary")
		report_line(&builder)
		report_line(&builder, "Times are native totals divided by measured batches, in ms/batch (100,000 queries) for the default all-family workload. Selected-family work counts are stated below. Lower is better")
	}
	else
	{
		report_line(&builder, "## physics_elapsed_ms summary")
	}
	report_line(&builder)
	if request.package_name == "query_extensions" && (summaries[0].query.case_name == "hierarchy" || summaries[0].query.case_name == "global" || summaries[0].query.case_name == "collision-batch")
	{
		query: Query_Measurement = summaries[0].query;
		if query.case_name == "collision-batch"
		{
			report_format_line(&builder, "Selected query: %s/%s/%s | 24,576 inputs x 1 pass = 24,576 pairs", query.case_name, query.operation, query.cell);
		}
		else
		{
			report_format_line(&builder, "Selected query: %s/%s/%s | 1,536 inputs x 8 passes = 12,288 queries", query.case_name, query.operation, query.cell);
		}
		report_line(&builder, "The timer includes dispatch, completion, the production query and required output writes. Setup, warmup, oracle checking and checksum reduction are outside timing. Each worker owns a context and result range. WN includes the caller. Scaling requires a measured W1 lane in this same dataset");
		report_line(&builder);
		report_line(&builder, "| Workers | Median M queries/s |");
		report_line(&builder, "| ------: | -----------------: |");
		for summary in summaries
		{
			report_format_line(&builder, "| %d | %.6f |", summary.worker_count, summary.median_throughput);
		}
		report_line(&builder);
	}
	else if request.package_name == "query_extensions" && summaries[0].query.group != ""
	{
		query: Query_Measurement = summaries[0].query;
		report_format_line(&builder, "Selected query: %s | %d measured chunks of 8,192 API calls | %d warmup chunks of 128 calls", query.group, query.repetitions, query.warmup_repetitions);
		report_line(&builder, "Times sum validated measured chunks. Other groups and complete Common/All totals are excluded. Calls execute serially, and workers configure world resources. Timing ratios compare resource counts and do not measure parallel speedup or a per-call latency distribution");
		if strings.has_suffix(query.group, ".collision_batch")
		{
			report_format_line(&builder, "Each API call performs two pair tests: %d calls and %d pair tests", u64(query.repetitions)*8192, u64(query.repetitions)*16384);
		}
		report_line(&builder, "| World resources | Mean ns/API call from median process sum |");
		report_line(&builder, "| --- | --- |");
		for summary in summaries
		{
			report_format_line(&builder, "| %d | %.6f |", summary.worker_count, summary.median*1e6/(8192*f64(query.repetitions)));
		}
		report_line(&builder);
	}
	else if request.package_name == "query_extensions"
	{
		report_line(&builder, "Common totals sum 8,192 calls for each of eight operations across four shape routes, after 128 warmup calls per operation. Calls execute serially, including the two-pair direct collision batch. Worker counts configure world resources. The speedup column is a timing ratio across those configurations and does not measure parallel query scaling. Enabled CQ01 timings are separate CSV columns")
		report_line(&builder)
	}
	else if request.package_name == "world_lifecycle"
	{
		report_line(&builder, "Totals sum five lifecycle phases across 20 measured cycles after one disposable warmup cycle. No simulation step or query dispatch is measured. Worker counts change dispatcher setup, storage and destruction costs. The speedup column is a timing ratio across those configurations")
		report_line(&builder)
	}
	report_line(&builder, "Median is the middle sample for odd counts and the arithmetic mean of the two middle samples for even counts. Speedup is relative to the smallest requested worker count")
	report_line(&builder)
	if request.package_name == "spatial_query_batch" || request.package_name == "spatial_query_trace"
	{
		report_format_line(&builder, "Ray pattern: %s", summaries[0].ray_pattern);
		report_format_line(&builder, "Query family: %s | Queries/batch: %d | Warmup batches: %d | Measured batches: %d", summaries[0].query.family, summaries[0].query_count, summaries[0].query.warmup_batches, summaries[0].query.measured_batches);
		report_line(&builder, "Per-process aggregate timings do not supply p95/p99 query latency");
		report_line(&builder);
		unit: string = "ms/frame" if request.package_name == "spatial_query_batch" else "ms/batch"
		report_format_line(&builder, "| Workers | Samples | Median %s | Minimum %s | Maximum %s | Speedup |", unit, unit, unit)
	}
	else
	{
		report_line(&builder, "| Workers | Samples | Median | Minimum | Maximum | Speedup |")
	}
	report_line(&builder, "| ------: | ------: | -----: | ------: | ------: | ------: |")
	for summary in summaries
	{
		divisor: f64 = f64(summary.measured_units) if request.package_name == "spatial_query_batch" || request.package_name == "spatial_query_trace" else 1
		if (summary.query.case_name == "hierarchy" || summary.query.case_name == "global" || summary.query.case_name == "collision-batch") && summary.speedup == 0
		{
			report_format_line(&builder, "| %d | %d | %.6f | %.6f | %.6f | unmeasured |", summary.worker_count, summary.sample_count, summary.median, summary.minimum, summary.maximum);
			continue;
		}
		report_format_line(
			&builder,
			"| %d | %d | %.6f | %.6f | %.6f | %.6fx |",
			summary.worker_count,
			summary.sample_count,
			summary.median/divisor,
			summary.minimum/divisor,
			summary.maximum/divisor,
			summary.speedup,
		)
	}
	report_line(&builder)
	if request.package_name == "spatial_query_batch" || request.package_name == "spatial_query_trace"
	{
		report_line(&builder, "Secondary throughput, in millions of queries per second. The time table uses per-process mean time, not individual query latency")
		report_line(&builder)
		report_line(&builder, "| Workers | Median M queries/s |")
		report_line(&builder, "| ------: | -----------------: |")
		for summary in summaries
		{
			report_format_line(&builder, "| %d | %.6f |", summary.worker_count, summary.median_throughput)
		}
		report_line(&builder)
	}
	if workload_kind(request.package_name) == .Parameterized
	{
		report_line(&builder, "Physical qualification uses authored box slots and assigned-neighbor compression. These fixed-axis measures do not describe contact-manifold penetration. Verification can affect cache and wall time outside the native timer")
		report_line(&builder)
		report_line(&builder, "| Workers | Measured | Physical outcome | Passed | Failed | Unassessed |")
		report_line(&builder, "| ------: | -------: | :--------------- | -----: | -----: | ---------: |")
		for summary in summaries
		{
			names: [Result_Physical]string = {.Unassessed="Unassessed", .Pass="Pass", .Fail="Fail"}
			report_format_line(&builder, "| %d | %d | %s | %d | %d | %d |", summary.worker_count, summary.sample_count,
				names[summary.physical.status], summary.physical.passed, summary.physical.failed, summary.physical.unassessed)
		}
		report_line(&builder)
	}
	if workload_kind(request.package_name) == .Parameterized
	{
		report_line(&builder, "Mean milliseconds per measured step, derived from each process total. The median of process means is not the median of individual step times")
		report_line(&builder)
		report_line(&builder, "| Workers | Median mean ms/step | Minimum mean ms/step | Maximum mean ms/step |")
		report_line(&builder, "| ------: | ------------------: | -------------------: | -------------------: |")
		for summary in summaries
		{
			report_format_line(&builder, "| %d | %.6f | %.6f | %.6f |", summary.worker_count, summary.median/f64(summary.measured_steps), summary.minimum/f64(summary.measured_steps), summary.maximum/f64(summary.measured_steps))
		}
		report_line(&builder)
	}
	report_line(&builder, "Compare results only when the machine, compiler, configuration, workload, and timed region are unchanged")
	return strings.to_string(builder)
}

generate_report :: proc(request: Report_Request) -> Report_Result
{
	result := collect_summaries(request)
	if result.status != .Ok
	{
		return result
	}
	result.markdown = render_markdown(request, result.summaries[:])
	temporary_path, temporary_error := filepath.join([]string{request.source_directory, "README.md.tmp"})
	output_path, output_error := filepath.join([]string{request.source_directory, "README.md"})
	if temporary_error != nil || output_error != nil || os.exists(output_path)
	{
		return report_error(.Report_Write_Failed, "report destination is not fresh: %s", request.source_directory)
	}
	if os.write_entire_file(temporary_path, result.markdown) != nil
	{
		return report_error(.Report_Write_Failed, "failed to write temporary report: %s", temporary_path)
	}
	if os.rename(temporary_path, output_path) != nil
	{
		_ = os.remove(temporary_path)
		return report_error(.Report_Write_Failed, "failed to publish report: %s", output_path)
	}
	return result
}

Workload_Kind :: enum u8
{
	Legacy,
	Parameterized,
}

workload_kind :: proc(package_name: string) -> Workload_Kind
{
	switch package_name
	{
		case "container", "contact_islands", "pyramid", "large_world_precision": return .Parameterized
	}
	return .Legacy
}

variant_status :: proc(value: string) -> Report_Status
{
	if len(value) == 0
	{
		return .Invalid_Arguments
	}
	for c, index in value
	{
		if (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')
		{
			continue
		}
		if index == 0 || (c != '.' && c != '_' && c != '-')
		{
			return .Invalid_Arguments
		}
	}
	return .Ok
}
