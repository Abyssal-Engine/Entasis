package benchmark_report

import "base:runtime"
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

Contact_Work_Summary :: struct
{
	input, affinity: string,
	points, updates: i64,
	covered, failed: int,
	residual_max, residual_rms, residual_p95: f64,
	negative_impulse, tangent_violation, twist_violation: f64,
	call_p50_median_ms, call_p95_median_ms, call_max_ms: f64,
}

Domain_Work_Summary :: struct
{
	case_name, region, unit: string,
	bodies, admitted_units, work_units: i64,
	contact_counts: [3]u64,
	residuals: [6]f64,
	observed_metrics: u8,
	covered, failed: int,
	pool_growth, worker_pool_growth: u64,
}

Report_Request :: struct
{
	query: Query_Measurement,
	ray_pattern: string,
	precision, precision_candidate, precision_case, sampling_mode: string,
	variant: string,
	source_directory:  string,
	baseline_source: string,
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
	domain: Domain_Work_Summary,
	contact: Contact_Work_Summary,
	query: Query_Measurement,
	profile: string,
	precision: Precision_Result,
	recording_mode: Result_Recording,
	timing_method: Result_Timing,
	benchmark_parameters: string,
	measured_steps: int,
	measured_passes: int,
	worker_count: int,
	sample_count: int,
	mean:         f64,
	median:       f64,
	minimum:      f64,
	maximum:      f64,
	speedup:      f64,
	ray_checksum: u64,
	ray_pattern: string,
	measured_units: int,
	query_count: int,
	primary_unit: string,
	primary_divisor: f64,
	median_throughput: f64,
	physical: Physical_Summary,
}

Report_Result :: struct
{
	status:    Report_Status,
	diagnostic: string,
	summaries: [dynamic]Lane_Summary,
	paired_summaries: [dynamic]Paired_Summary,
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

summarize_csv :: proc(path: string, expected_rows, worker_count: int, package_name: string = "", records_out: ^[][]string = nil) -> (Lane_Summary, Report_Result)
{
	if records_out != nil
	{
		parsed: Result_CSV;
		parsed_result: Report_Result;
		parsed, parsed_result = result_csv_read(path, 512*1024*1024);
		if parsed_result.status != .Ok
		{
			return {}, parsed_result;
		}
		records_out^ = parsed.records;
		return summarize_records(parsed.records, path, expected_rows, worker_count, package_name);
	}
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

contact_work_summary :: proc(records: [][]string) -> (Contact_Work_Summary, Report_Result)
{
	names: [20]string = {"contact_input", "contact_points", "point_updates", "measured_passes", "warmup_passes",
		"physics_status", "quality_coverage", "quality_failures", "residual_max_mps", "residual_rms_mps", "residual_p95_mps",
		"negative_impulse_max", "friction_violation_max", "twist_violation_max", "fixed_elapsed_ms", "ms_per_1000_point_updates", "processor_affinity",
		"call_p50_ms", "call_p95_ms", "call_max_ms"};
	columns: [len(names)]int;
	for name, index in names
	{
		status: Report_Status;
		columns[index], status = header_index(records[0], name);
		if status != .Ok
		{
			return {}, report_error(.Invalid_Header, "required contact-work column missing or duplicated: %s", name);
		}
	}
	summary: Contact_Work_Summary;
	call_p50: []f64 = make([]f64, len(records)-1);
	defer delete(call_p50);
	call_p95: []f64 = make([]f64, len(records)-1);
	defer delete(call_p95);
	for record, row in records[1:]
	{
		if len(record) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "contact-work column count mismatch");
		}
		input: string = record[columns[0]];
		affinity: string = record[columns[16]];
		mask: u64;
		mask_parsed: bool;
		mask, mask_parsed = strconv.parse_u64(affinity);
		if !mask_parsed || mask == 0 || (row > 0 && affinity != summary.affinity)
		{
			return {}, report_error(.Invalid_Measurement, "invalid or differing contact-work processor affinity");
		}
		counts: [6]i64;
		for field, index in ([6]int{1, 2, 3, 4, 6, 7})
		{
			parsed: bool;
			counts[index], parsed = strconv.parse_i64(record[columns[field]]);
			if !parsed
			{
				return {}, report_error(.Invalid_Measurement, "invalid contact-work count: %s", names[field]);
			}
		}
		if len(input) == 0 || counts[0] <= 0 || counts[0] > 1_000_000 || counts[1] != counts[0]*400 ||
			counts[2] != 100 || counts[3] != 10 || counts[4] != counts[2] || counts[5] < 0 || counts[5] > counts[4] ||
			(row > 0 && (input != summary.input || counts[0] != summary.points || counts[1] != summary.updates))
		{
			return {}, report_error(.Invalid_Measurement, "contact input, work or observation coverage differs");
		}
		status: string = record[columns[5]];
		if (status != "pass" && status != "fail") || (status == "pass" && counts[5] != 0) || (status == "fail" && counts[5] == 0)
		{
			return {}, report_error(.Invalid_Status, "contact hard-check status disagrees with failed calls");
		}
		metrics: [8]f64;
		for &value, index in metrics
		{
			parsed: bool;
			value, parsed = strconv.parse_f64(record[columns[index+8]]);
			if !parsed || math.is_nan(value) || math.is_inf(value) || value < 0
			{
				return {}, report_error(.Invalid_Measurement, "nonfinite or negative contact-work metric: %s", names[index+8]);
			}
		}
		normalized: f64 = 1000*metrics[6]/f64(counts[1]);
		if metrics[6] <= 0 || math.abs(metrics[7]-normalized) > 1e-6*max(1, normalized)
		{
			return {}, report_error(.Invalid_Measurement, "contact point-work normalization differs from native total");
		}
		latency: [3]f64;
		for &value, index in latency
		{
			parsed: bool;
			value, parsed = strconv.parse_f64(record[columns[17+index]]);
			if !parsed || math.is_nan(value) || math.is_inf(value) || value <= 0
			{
				return {}, report_error(.Invalid_Measurement, "invalid contact call latency: %s", names[17+index]);
			}
		}
		// native millisecond columns round to nine decimals
		if latency[0] > latency[1] || latency[1] > latency[2] || metrics[6]/f64(counts[2]) > latency[2]+1e-9
		{
			return {}, report_error(.Invalid_Measurement, "contact call latency disagrees with percentile order or process mean");
		}
		call_p50[row], call_p95[row] = latency[0], latency[1];
		summary.call_max_ms = max(summary.call_max_ms, latency[2]);
		summary.input, summary.affinity, summary.points, summary.updates = input, affinity, counts[0], counts[1];
		summary.covered += int(counts[4]);
		summary.failed += int(counts[5]);
		summary.residual_max = max(summary.residual_max, metrics[0]);
		summary.residual_rms = max(summary.residual_rms, metrics[1]);
		summary.residual_p95 = max(summary.residual_p95, metrics[2]);
		summary.negative_impulse = max(summary.negative_impulse, metrics[3]);
		summary.tangent_violation = max(summary.tangent_violation, metrics[4]);
		summary.twist_violation = max(summary.twist_violation, metrics[5]);
	}
	summary.call_p50_median_ms = paired_median(call_p50);
	summary.call_p95_median_ms = paired_median(call_p95);
	return summary, {status=.Ok};
}

domain_work_summary :: proc(records: [][]string, package_name: string) -> (Domain_Work_Summary, Report_Result)
{
	if package_name != "simulation_stage_work" && package_name != "solver_fixed_work"
	{
		return {}, {status=.Ok};
	}
	case_column: int;
	case_status: Report_Status;
	case_column, case_status = header_index(records[0], "stage" if package_name == "simulation_stage_work" else "case");
	if case_status != .Ok
	{
		// historical fixed-work rows do not contain the new observation fields
		if package_name == "solver_fixed_work"
		{
			return {}, {status=.Ok};
		}
		return {}, report_error(.Invalid_Header, "stage identity is missing");
	}
	coverage_column: int;
	coverage_status: Report_Status;
	coverage_column, coverage_status = header_index(records[0], "quality_coverage");
	if coverage_status != .Ok && package_name == "solver_fixed_work" && records[1][case_column] == "conserving"
	{
		return {}, {status=.Ok};
	}
	names: [6]string = {"bodies", "measured_passes", "quality_coverage", "quality_failures", "physics_status", "benchmark_parameters"};
	columns: [len(names)]int;
	for name, index in names
	{
		column: int;
		status: Report_Status;
		column, status = header_index(records[0], name);
		if status != .Ok
		{
			return {}, report_error(.Invalid_Header, "domain work requires %s", name);
		}
		columns[index] = column;
	}
	summary: Domain_Work_Summary;
	for row, index in records[1:]
	{
		if len(row) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "domain work column count mismatch");
		}
		bodies: i64;
		bodies_ok: bool;
		passes: i64;
		passes_ok: bool;
		coverage: i64;
		coverage_ok: bool;
		failures: i64;
		failures_ok: bool;
		bodies, bodies_ok = strconv.parse_i64(row[columns[0]]);
		passes, passes_ok = strconv.parse_i64(row[columns[1]]);
		coverage, coverage_ok = strconv.parse_i64(row[columns[2]]);
		failures, failures_ok = strconv.parse_i64(row[columns[3]]);
		if !bodies_ok || bodies <= 0 || !passes_ok || passes <= 0 || !coverage_ok || coverage != passes ||
			!failures_ok || failures < 0 || failures > coverage ||
			(row[columns[4]] != "pass" && row[columns[4]] != "fail") ||
			(row[columns[4]] == "pass") != (failures == 0)
		{
			return {}, report_error(.Invalid_Measurement, "domain work has incomplete or inconsistent observation");
		}
		case_name: string = row[case_column];
		if case_name == "" || (index > 0 && (case_name != summary.case_name || bodies != summary.bodies))
		{
			return {}, report_error(.Invalid_Measurement, "domain work case or population differs");
		}
		summary.case_name, summary.bodies = case_name, bodies;
		summary.unit = "ms/stage call" if package_name == "simulation_stage_work" else "ms/pass";
		if package_name == "solver_fixed_work" && (case_name == "convex" || case_name == "cached-convex" || case_name == "nonconvex" || case_name == "joints")
		{
			summary.unit = "ms/solve";
		}
		if column, status := header_index(records[0], "region"); status == .Ok
		{
			if row[column] == "" || (index > 0 && row[column] != summary.region)
			{
				return {}, report_error(.Invalid_Measurement, "domain measured region differs");
			}
			summary.region = row[column];
		}
		else
		{
			summary.region = row[columns[5]];
			for parameter in strings.split(row[columns[5]], ";")
			{
				if strings.has_prefix(parameter, "region=")
				{
					summary.region = parameter[len("region="):];
				}
			}
		}
		for name, metric in ([3]string{"pair_count", "manifold_count", "contact_points"})
		{
			if column, status := header_index(records[0], name); status == .Ok
			{
				value, parsed := strconv.parse_u64(row[column]);
				if !parsed
				{
					return {}, report_error(.Invalid_Measurement, "invalid produced domain count: %s", name);
				}
				summary.contact_counts[metric] = max(summary.contact_counts[metric], value);
			}
		}
		applicable_metrics: u8;
		if package_name == "solver_fixed_work"
		{
			// every new fixed case checks quaternion norms; only contact fixtures
			// evaluate normal residuals, and only joints construct observed edges
			applicable_metrics = u8(1)<<4;
			switch case_name
			{
			case "convex", "cached-convex", "nonconvex", "constrained-final":
				applicable_metrics |= 7;
			case "joints":
				applicable_metrics |= (u8(1)<<3) | (u8(1)<<5);
			}
		}
		for name, metric in ([6]string{"residual_max_mps", "residual_rms_mps", "residual_p95_mps", "position_error_m", "orientation_norm_error", "joint_angle_error_rad"})
		{
			if applicable_metrics & (u8(1)<<uint(metric)) == 0
			{
				continue;
			}
			if column, status := header_index(records[0], name); status == .Ok
			{
				value, parsed := strconv.parse_f64(row[column]);
				if !parsed || value < 0 || math.is_nan(value) || math.is_inf(value)
				{
					return {}, report_error(.Invalid_Measurement, "invalid domain quality metric: %s", name);
				}
				summary.residuals[metric] = max(summary.residuals[metric], value);
				summary.observed_metrics |= u8(1)<<uint(metric);
			}
		}
		for name, target in ([2]string{"admitted_units", "work_units"})
		{
			if column, status := header_index(records[0], name); status == .Ok
			{
				value, parsed := strconv.parse_i64(row[column]);
				if !parsed || value <= 0 || (index > 0 && value != (summary.admitted_units if target == 0 else summary.work_units))
				{
					return {}, report_error(.Invalid_Measurement, "domain %s is nonpositive or inconsistent", name);
				}
				if target == 0
				{
					summary.admitted_units = value;
				}
				else
				{
					summary.work_units = value;
				}
			}
		}
		for prefix, target in ([2]string{"pool_capacity_", "worker_pool_capacity_"})
		{
			before_column: int;
			before_status: Report_Status;
			after_column: int;
			after_status: Report_Status;
			before_column, before_status = header_index(records[0], strings.concatenate({prefix, "before"}));
			after_column, after_status = header_index(records[0], strings.concatenate({prefix, "after"}));
			if before_status != after_status
			{
				return {}, report_error(.Invalid_Header, "partial domain capacity observation");
			}
			if before_status == .Ok
			{
				before: u64;
				before_ok: bool;
				after: u64;
				after_ok: bool;
				before, before_ok = strconv.parse_u64(row[before_column]);
				after, after_ok = strconv.parse_u64(row[after_column]);
				if !before_ok || !after_ok
				{
					return {}, report_error(.Invalid_Measurement, "invalid domain capacity observation");
				}
				growth: u64 = after-min(after, before);
				if target == 0
				{
					summary.pool_growth = max(summary.pool_growth, growth);
				}
				else
				{
					summary.worker_pool_growth = max(summary.worker_pool_growth, growth);
				}
			}
		}
		summary.covered += int(coverage);
		summary.failed += int(failures);
	}
	return summary, {status=.Ok};
}

arithmetic_mean :: proc(values: []f64) -> f64
{
	total: f64;
	for value in values
	{
		total += value;
	}
	return total/f64(len(values));
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
	worker_column: int = -1;
	for name, column in records[0]
	{
		if name == "worker_count"
		{
			if worker_column >= 0
			{
				return {}, report_error(.Invalid_Header, "duplicated worker_count in %s", path);
			}
			worker_column = column;
		}
	}
	if worker_column < 0 && workload_kind(package_name) != .Legacy
	{
		return {}, report_error(.Invalid_Header, "required worker_count missing in %s", path);
	}
	for record, row_index in records[1:]
	{
		if len(record) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "column count mismatch in %s row %d", path, row_index+2);
		}
		if worker_column >= 0
		{
			worker: i64;
			parsed: bool;
			worker, parsed = strconv.parse_i64(record[worker_column]);
			if !parsed || worker != i64(worker_count)
			{
				return {}, report_error(.Invalid_Measurement, "worker mismatch in %s row %d: expected %d", path, row_index+2, worker_count);
			}
		}
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
	if package_name == "solver_contact_work" && profile != "float32"
	{
		return {}, report_error(.Invalid_Measurement, "contact-work requires explicit Float32 precision");
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
	contact: Contact_Work_Summary;
	if package_name == "solver_contact_work"
	{
		contact_result: Report_Result;
		contact, contact_result = contact_work_summary(records);
		if contact_result.status != .Ok
		{
			return {}, contact_result;
		}
	}
	domain: Domain_Work_Summary;
	domain_result: Report_Result;
	domain, domain_result = domain_work_summary(records, package_name);
	if domain_result.status != .Ok
	{
		return {}, domain_result;
	}
	case_status_index, case_header_status := header_index(records[0], "case_status")
	measurement_name: string = "fixed_elapsed_ms" if workload_kind(package_name) == .Fixed else "physics_elapsed_ms"
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
	parameter_columns: [2]int
	if workload_kind(package_name) != .Legacy
	{
		names: [2]string = {"benchmark_parameters", "measured_passes" if workload_kind(package_name) == .Fixed else "step_index"}
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
		parameter_column: int;
		parameter_status: Report_Status;
		parameter_column, parameter_status = header_index(records[0], "benchmark_parameters");
		_ = parameter_column;
		if len(records[0]) != len(names) + 2 + (1 if len(profile) > 0 else 0) + (1 if ray_pattern_column >= 0 else 0) + (1 if query.timer == "prepared-post-join" else 0) + (1 if parameter_status == .Ok else 0)
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
		if workload_kind(package_name) != .Legacy
		{
			row_parameters := record[parameter_columns[0]]
			steps, steps_parsed := strconv.parse_i64(record[parameter_columns[1]])
			if len(row_parameters) == 0 || !steps_parsed || steps <= 0 || steps > 1_000_000 ||
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
	mean: f64 = arithmetic_mean(measurements);
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
	summary: Lane_Summary = {
		domain=domain,
		contact=contact,
		query=query,
		profile=profile, precision=precision,
		recording_mode=recording_mode, timing_method=timing_method,
		worker_count = worker_count,
		sample_count = expected_rows,
		mean = mean,
		median = median,
		minimum = measurements[0],
		maximum = measurements[len(measurements) - 1],
		ray_checksum = ray_checksum,
		ray_pattern = ray_pattern,
		measured_units=measured_units, query_count=query_count, median_throughput=median_throughput, physical=physical,
		benchmark_parameters = parameters,
		measured_steps = measured_steps if workload_kind(package_name) != .Fixed else 0,
		measured_passes = measured_steps if workload_kind(package_name) == .Fixed else 0,
	};
	normalization: Report_Result;
	summary.primary_unit, summary.primary_divisor, normalization = metric_normalization(package_name, measurement_name, summary, records);
	if normalization.status != .Ok
	{
		return {}, normalization;
	}
	return summary, {status=.Ok};
}

validate_lane_files :: proc(source_directory: string, workers: []int) -> Report_Result
{
	entries, directory_error := os.read_all_directory_by_path(source_directory, context.allocator)
	if directory_error != nil
	{
		return report_error(.Directory_Read_Failed, "failed to read source directory: %s", source_directory)
	}
	found := make([]u8, len(workers))
	for entry in entries
	{
		if entry.type != .Regular || !strings.has_suffix(entry.name, ".csv")
		{
			continue
		}
		matched_index := -1
		for worker, worker_position in workers
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
			return report_error(.Missing_Lane, "missing CSV lane: %s", worker_file_name(workers[index]))
		}
	}
	return {status=.Ok};
}

validate_lane_summary :: proc(request: Report_Request, summary: Lane_Summary, previous: []Lane_Summary, path: string) -> Report_Result
{
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
	if len(previous) > 0 && (summary.query != previous[0].query || summary.profile != previous[0].profile)
	{
		return report_error(.Invalid_Measurement, "query selection, counts, components or precision differ across workers");
	}
	if len(previous) > 0 && summary.ray_pattern != previous[0].ray_pattern
	{
		return report_error(.Invalid_Measurement, "ray pattern differs across worker counts in %s", path);
	}
	if request.package_name == "spatial_query_batch" && len(previous) > 0 &&
		summary.ray_checksum != previous[0].ray_checksum
	{
			return report_error(.Invalid_Measurement, "ray checksum differs across worker counts in %s", path)
	}
	if workload_kind(request.package_name) != .Legacy && len(previous) > 0 &&
	(summary.benchmark_parameters != previous[0].benchmark_parameters || summary.measured_steps != previous[0].measured_steps || summary.measured_passes != previous[0].measured_passes)
	{
		return report_error(.Invalid_Measurement, "configuration differs across worker counts in %s", path)
	}
	if len(previous) > 0 && (summary.recording_mode != previous[0].recording_mode || summary.timing_method != previous[0].timing_method)
	{
		return report_error(.Invalid_Measurement, "mixed recording or timing conditions");
	}
	if len(previous) > 0 && (summary.physical.mode != previous[0].physical.mode || summary.physical.criterion != previous[0].physical.criterion ||
		summary.physical.shape_limit != previous[0].physical.shape_limit || summary.measured_units != previous[0].measured_units || summary.query_count != previous[0].query_count)
	{
			return report_error(.Invalid_Measurement, "incompatible physical verification or query counts across workers")
	}
	return {status=.Ok};
}

collect_summaries :: proc(request: Report_Request) -> Report_Result
{
	if (workload_kind(request.package_name) == .Parameterized && request.package_name != "large_world_precision" && variant_status(request.variant) != .Ok) ||
	(workload_kind(request.package_name) == .Legacy && len(request.variant) != 0)
	{
		return report_error(.Invalid_Arguments, "variant must identify a configurable workload");
	}
	contract_result: Report_Result = validate_worker_contract(request.workers, request.run_count);
	if contract_result.status != .Ok
	{
		return contract_result;
	}
	for source_directory, source_position in ([2]string{request.source_directory, request.baseline_source})
	{
		if source_position == 1 && source_directory == ""
		{
			continue;
		}
		directory_result: Report_Result = validate_lane_files(source_directory, request.workers);
		if directory_result.status != .Ok
		{
			return directory_result;
		}
	}
	summaries: [dynamic]Lane_Summary = make([dynamic]Lane_Summary, 0, len(request.workers));
	paired_summaries: [dynamic]Paired_Summary;
	baseline_first: [1]Lane_Summary;
	if request.baseline_source != ""
	{
		paired_summaries = make([dynamic]Paired_Summary, 0, len(request.workers));
	}
	for worker, worker_position in request.workers
	{
		path: string;
		join_error: runtime.Allocator_Error;
		path, join_error = filepath.join([]string{request.source_directory, worker_file_name(worker)});
		if join_error != nil
		{
			return report_error(.Invalid_Arguments, "invalid lane path for worker %d", worker);
		}
		records: [][]string;
		summary: Lane_Summary;
		summary_result: Report_Result;
		if request.baseline_source == ""
		{
			summary, summary_result = summarize_csv(path, request.run_count, worker, request.package_name);
		}
		else
		{
			summary, summary_result = summarize_csv(path, request.run_count, worker, request.package_name, &records);
		}
		if summary_result.status != .Ok
		{
			return summary_result;
		}
		summary_result = validate_lane_summary(request, summary, summaries[:], path);
		if summary_result.status != .Ok
		{
			return summary_result;
		}
		if request.baseline_source != ""
		{
			baseline_path: string;
			baseline_path, join_error = filepath.join([]string{request.baseline_source, worker_file_name(worker)});
			if join_error != nil
			{
				return report_error(.Invalid_Arguments, "invalid baseline lane path for worker %d", worker);
			}
			baseline_records: [][]string;
			baseline_summary: Lane_Summary;
			baseline_summary, summary_result = summarize_csv(baseline_path, request.run_count, worker, request.package_name, &baseline_records);
			if summary_result.status != .Ok
			{
				return summary_result;
			}
			summary_result = validate_lane_summary(request, baseline_summary, baseline_first[:min(worker_position, 1)], baseline_path);
			if summary_result.status != .Ok
			{
				return summary_result;
			}
			if worker_position == 0
			{
				baseline_first[0] = baseline_summary;
			}
			paired: Paired_Summary;
			paired, summary_result = paired_compare_records(baseline_records, records, baseline_summary, summary, request.package_name);
			if summary_result.status != .Ok
			{
				return summary_result;
			}
			append(&paired_summaries, paired);
		}
		append(&summaries, summary);
	}
	smallest_worker_index := 0
	for summary, index in summaries
	{
		if summary.worker_count < summaries[smallest_worker_index].worker_count
		{
			smallest_worker_index = index
		}
	}
	baseline := summaries[smallest_worker_index].mean
	for &summary in summaries
	{
		if summary.precision.mode != "memory"
		{
			summary.speedup = baseline / summary.mean;
			if (summary.query.case_name == "hierarchy" || summary.query.case_name == "global" || summary.query.case_name == "collision-batch") && summaries[smallest_worker_index].worker_count != 1
			{
				summary.speedup = 0;
			}
		}
	}
	return {status=.Ok, summaries=summaries, paired_summaries=paired_summaries};
}

render_report_metadata :: proc(request: Report_Request, summaries: []Lane_Summary) -> string
{
	builder: strings.Builder = strings.builder_make_len_cap(0, 4096);
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
	report_format_line(&builder, "- Report generated (UTC): `%s`", request.run_date)
	report_format_line(&builder, "- Odin compiler: `%s`", compiler_version)
	report_format_line(&builder, "- Operating system: `%s`", request.operating_system)
	report_format_line(&builder, "- Processor: `%s`", request.processor)
	report_format_line(&builder, "- Logical processors: `%d`", request.logical_processors)
	report_line(&builder)
	kind: Workload_Kind = workload_kind(request.package_name);
	if kind == .Parameterized || kind == .Domain || kind == .Fixed
	{
		if kind == .Parameterized
		{
			report_format_line(&builder, "Variant: `%s`", request.variant);
			report_line(&builder);
		}
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
	if workload_kind(request.package_name) == .Fixed
	{
		report_format_line(&builder, "Measured passes per process: %d", summaries[0].measured_passes);
	}
	if summaries[0].domain.case_name != ""
	{
		report_format_line(&builder, "Domain: %s | Unit: %s | Measured region: %s", summaries[0].domain.case_name, summaries[0].domain.unit, summaries[0].domain.region);
	}
	query: Query_Measurement = summaries[0].query;
	if request.package_name == "spatial_query_batch" || request.package_name == "spatial_query_trace"
	{
		report_format_line(&builder, "Ray pattern: %s", summaries[0].ray_pattern);
		report_format_line(&builder, "Query family: %s | Queries/batch: %d | Warmup batches: %d | Measured batches: %d", query.family, summaries[0].query_count, query.warmup_batches, query.measured_batches);
		report_line(&builder, "Per-process aggregate timings do not supply p95/p99 query latency");
		if request.package_name == "spatial_query_batch"
		{
			report_format_line(&builder, "Execution: %s | Timer: %s", query.execution, query.timer);
			if query.timer == "prepared-post-join"
			{
				report_line(&builder, "Prepared inputs and reservations are outside timing. Dispatch/session, query result writes and common caller validation/checksum are timed. Scalar reference validation runs before and after timing");
			}
			else
			{
				report_line(&builder, "Public query_batch, batch size 256. Times are native totals divided by measured frames, in ms/frame (50,000 rays). Lower is better. Scalar output validation runs before and after timing. Worker counts include the caller");
			}
		}
		report_line(&builder);
	}
	if request.package_name == "query_extensions"
	{
		if query.case_name == "hierarchy" || query.case_name == "global" || query.case_name == "collision-batch"
		{
			report_format_line(&builder, "Selected query: %s/%s/%s | Inputs: %d | Passes: %d | %s: %d", query.case_name, query.operation, query.cell,
				query.inputs, query.passes, "Pairs" if query.case_name == "collision-batch" else "Queries", query.queries);
			report_line(&builder, "The timer includes dispatch, completion, the production query and required output writes. Setup, warmup, oracle checking and checksum reduction are outside timing. Each worker owns a context and result range. WN includes the caller. Scaling requires a measured W1 lane in this same dataset");
		}
		else if query.group != ""
		{
			report_format_line(&builder, "Selected query: %s | %d measured chunks of 8,192 API calls | %d warmup chunks of 128 calls", query.group, query.repetitions, query.warmup_repetitions);
			report_line(&builder, "Times sum validated measured chunks. Other groups and complete Common/All totals are excluded. Calls execute serially, and workers configure world resources. Timing ratios compare resource counts and do not measure parallel speedup or a per-call latency distribution");
			if strings.has_suffix(query.group, ".collision_batch")
			{
				report_format_line(&builder, "Each API call performs two pair tests: %d calls and %d pair tests", u64(query.repetitions)*8192, u64(query.repetitions)*16384);
			}
		}
		else
		{
			report_line(&builder, "Common totals sum 8,192 calls for each of eight operations across four shape routes, after 128 warmup calls per operation. Calls execute serially, including the two-pair direct collision batch. Worker counts configure world resources and do not measure parallel query scaling. Enabled CQ01 timings are separate CSV columns");
		}
		report_line(&builder);
	}
	return strings.to_string(builder);
}

render_markdown :: proc(request: Report_Request, summaries: []Lane_Summary) -> string
{
	builder: strings.Builder = strings.builder_make_len_cap(0, 4096);
	strings.write_string(&builder, render_report_metadata(request, summaries));
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
	if request.package_name == "spatial_query_batch"
	{
		report_line(&builder, "## Closest-ray time summary")
		report_line(&builder)
	}
	else if request.package_name == "spatial_query_trace"
	{
		report_line(&builder, "## Mixed-query time summary")
		report_line(&builder)
	}
	else
	{
		report_line(&builder, "## Domain cost summary" if summaries[0].primary_unit != "ms/process total" else "## physics_elapsed_ms summary")
	}
	report_line(&builder)
	if request.package_name == "query_extensions" && (summaries[0].query.case_name == "hierarchy" || summaries[0].query.case_name == "global" || summaries[0].query.case_name == "collision-batch")
	{
		report_line(&builder, "| Workers | Median M queries/s |");
		report_line(&builder, "| ------: | -----------------: |");
		for summary in summaries
		{
			report_format_line(&builder, "| %d | %.6f |", summary.worker_count, summary.median_throughput);
		}
		report_line(&builder);
	}
	if request.package_name == "world_lifecycle"
	{
		report_line(&builder, "Costs divide the sum of five lifecycle phases by the recorded measured cycles after one disposable warmup cycle. No simulation step or query dispatch is measured. Worker counts change dispatcher setup, storage and destruction costs. The speedup column is a timing ratio across those configurations")
		report_line(&builder)
	}
	report_line(&builder, "Average is the arithmetic mean across process samples in the declared unit. Speedup uses the same averages relative to the smallest requested worker count")
	report_line(&builder)
	if request.package_name == "spatial_query_batch" || request.package_name == "spatial_query_trace"
	{
		unit: string = "ms/frame" if request.package_name == "spatial_query_batch" else "ms/batch"
		report_format_line(&builder, "| Workers | Samples | Average %s | Minimum %s | Maximum %s | Speedup |", unit, unit, unit)
	}
	else
	{
		report_format_line(&builder, "| Workers | Samples | Average %s | Minimum %s | Maximum %s | Speedup |",
			summaries[0].primary_unit, summaries[0].primary_unit, summaries[0].primary_unit);
	}
	report_line(&builder, "| ------: | ------: | -----: | ------: | ------: | ------: |")
	for summary in summaries
	{
		divisor: f64 = summary.primary_divisor;
		if (summary.query.case_name == "hierarchy" || summary.query.case_name == "global" || summary.query.case_name == "collision-batch") && summary.speedup == 0
		{
			report_format_line(&builder, "| %d | %d | %.6f | %.6f | %.6f | unmeasured |", summary.worker_count, summary.sample_count,
				summary.mean/divisor, summary.minimum/divisor, summary.maximum/divisor);
			continue;
		}
		report_format_line(
			&builder,
			"| %d | %d | %.6f | %.6f | %.6f | %.6fx |",
			summary.worker_count,
			summary.sample_count,
			summary.mean/divisor,
			summary.minimum/divisor,
			summary.maximum/divisor,
			summary.speedup,
		)
	}
	report_line(&builder)
	if request.package_name == "solver_contact_work"
	{
		report_line(&builder, "## Contact work and endpoint quality");
		report_line(&builder);
		report_line(&builder, "Production solver-stage replay restores identical entry state and a fixed contact graph. Preparation and the native 1x4 solve are timed. Reset, observation and final pose/unconstrained integration are excluded. Logical point updates count live normal rows, including zero impulses, across all measured sweeps");
		report_line(&builder);
		report_line(&builder, "Residuals describe each version's own final-substep equations because substep geometry can diverge. They are reported convergence measures, with no universal residual Pass threshold. Hard checks cover finite state and impulse feasibility. A positive gain makes no keep/reject decision");
		report_line(&builder);
		report_line(&builder, "| Workers | ms/1000 point updates | Observed calls | Hard-failed calls | Max residual m/s | Worst call RMS m/s | Worst call p95 m/s | Negative impulse N s | Tangent excess N s | Twist excess N m s |");
		report_line(&builder, "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
		for summary in summaries
		{
			c: Contact_Work_Summary = summary.contact;
			report_format_line(&builder, "| %d | %.9f | %d | %d | %.9f | %.9f | %.9f | %.9f | %.9f | %.9f |", summary.worker_count,
				1000*summary.mean/f64(c.updates), c.covered, c.failed,
				c.residual_max, c.residual_rms, c.residual_p95, c.negative_impulse, c.tangent_violation, c.twist_violation);
		}
		report_line(&builder);
		report_line(&builder, "Call latency aggregates process-level percentiles: median of each process's call p50, median of each process's call p95, and maximum of each process's call maximum. These are not pooled-call quantiles. Process mean ms/call remains the primary timing statistic");
		report_line(&builder);
		report_line(&builder, "| Workers | Median process call p50 ms | Median process call p95 ms | Maximum call ms |");
		report_line(&builder, "| ---: | ---: | ---: | ---: |");
		for summary in summaries
		{
			c: Contact_Work_Summary = summary.contact;
			report_format_line(&builder, "| %d | %.9f | %.9f | %.9f |", summary.worker_count,
				c.call_p50_median_ms, c.call_p95_median_ms, c.call_max_ms);
		}
		report_line(&builder);
	}
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
	if summaries[0].domain.case_name != ""
	{
		report_line(&builder);
		report_line(&builder, "Domain observations remain outside the timer. Hard checks and complete coverage do not establish equal convergence or evolving-scene accuracy");
		if request.package_name == "simulation_stage_work"
		{
			report_line(&builder, "Stage pool columns report capacity growth across the measured-frame observation window, including untimed prerequisites and observation. They cannot be attributed solely to the timed stage");
		}
		report_line(&builder, "| Workers | Bodies | Admitted units | Work units | Observed calls | Failed calls | Maximum pool growth bytes | Maximum worker-pool growth bytes |");
		report_line(&builder, "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
		for summary in summaries
		{
			d: Domain_Work_Summary = summary.domain;
			report_format_line(&builder, "| %d | %d | %d | %d | %d | %d | %d | %d |", summary.worker_count, d.bodies,
				d.admitted_units, d.work_units, d.covered, d.failed, d.pool_growth, d.worker_pool_growth);
		}
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
	if request.baseline_source == ""
	{
		result.markdown = render_markdown(request, result.summaries[:]);
	}
	else
	{
		metadata: string = render_report_metadata(request, result.summaries[:]);
		comparison: string = render_paired_comparison(request, result.paired_summaries[:]);
		result.markdown = strings.concatenate({metadata, comparison});
	}
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
	Domain,
	Fixed,
}

workload_kind :: proc(package_name: string) -> Workload_Kind
{
	switch package_name
	{
		case "container", "contact_islands", "pyramid", "large_world_precision": return .Parameterized
		case "solver_joint_angular": return .Domain
		case "solver_fixed_work", "solver_contact_work", "simulation_stage_work": return .Fixed
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
