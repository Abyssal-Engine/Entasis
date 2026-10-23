package benchmark_report

import "core:math"
import "core:strconv"

Precision_Result :: struct
{
	profile, candidate, precision_case, mode: string,
	count, maintenance_count: int,
	frame_median_ms, frame_max_ms, maintenance_max_ms, checksum: f64,
	memory: [13]u64,
}

PRECISION_MEMORY_NAMES :: [13]string{
	"setup_peak_bytes", "measurement_peak_bytes", "initial_owned_bytes", "final_owned_bytes",
	"total_allocations", "measured_allocations", "initial_pool_bytes", "final_pool_bytes",
	"initial_worker_pool_bytes", "final_worker_pool_bytes", "application_reserved_bytes",
	"shift_allocations", "remaining_owned_bytes",
};

precision_case_status :: proc(candidate, precision_case, mode: string, worker: int) -> Report_Status
{
	if candidate != "baseline" && candidate != "origin" && candidate != "mixed64"
	{
		return .Invalid_Arguments;
	}
	if mode != "batch" && mode != "instrumented" && mode != "engine-only" && mode != "memory"
	{
		return .Invalid_Arguments;
	}
	switch precision_case
	{
		case "compact-near", "compact-10km", "compact-1000km", "compact-1gm":
		case "shift-sparse", "shift-stress", "crossing", "holdout-small", "holdout-mixed":
			if candidate == "baseline"
			{
				return .Invalid_Arguments;
			}
		case "distributed":
			if candidate != "mixed64"
			{
				return .Invalid_Arguments;
			}
		case "queries-local", "queries-spanning":
			if candidate == "baseline" || worker != 1 || mode == "engine-only" ||
				(precision_case == "queries-spanning" && candidate != "mixed64")
			{
				return .Invalid_Arguments;
			}
		case: return .Invalid_Arguments;
	}
	return .Ok;
}

result_precision_profile :: proc(records: [][]string) -> (string, Report_Result)
{
	column: int = -1;
	for name, index in records[0]
	{
		if name == "precision"
		{
			if column >= 0
			{
				return "", report_error(.Invalid_Header, "duplicate precision column");
			}
			column = index;
		}
	}
	if column < 0
	{
		return "", {status=.Ok};
	}
	profile: string;
	for row, index in records[1:]
	{
		if len(row) != len(records[0]) || (row[column] != "float32" && row[column] != "mixed64") ||
			(index > 0 && profile != row[column])
		{
			return "", report_error(.Invalid_Measurement, "invalid or mixed compiled precision");
		}
		profile = row[column];
	}
	return profile, {status=.Ok};
}

precision_result_summary :: proc(records: [][]string, worker: int) -> (Precision_Result, Report_Result)
{
	columns: [15]int;
	names: [15]string = {"precision_case", "precision_candidate", "sampling_mode", "sampling_count",
		"frame_median_ms", "frame_max_ms", "maintenance_count", "maintenance_max_ms", "export_checksum",
		"worker_count", "step_index", "case_status", "metric_status", "physics_elapsed_ms", "timing_method"};
	for name, index in names
	{
		column: int;
		status: Report_Status;
		column, status = header_index(records[0], name);
		if status != .Ok
		{
			return {}, report_error(.Invalid_Header, "missing or duplicate precision field: %s", name);
		}
		columns[index] = column;
	}
	result: Precision_Result;
	profile_result: Report_Result;
	result.profile, profile_result = result_precision_profile(records);
	if profile_result.status != .Ok
	{
		return {}, profile_result;
	}
	if len(result.profile) == 0
	{
		return {}, report_error(.Invalid_Header, "precision producer requires its compiled profile");
	}
	for row, index in records[1:]
	{
		if len(row) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "precision column count mismatch");
		}
		candidate: string = row[columns[1]];
		precision_case: string = row[columns[0]];
		mode: string = row[columns[2]];
		if precision_case_status(candidate, precision_case, mode, worker) != .Ok ||
			(result.profile == "mixed64") != (candidate == "mixed64") ||
			(index > 0 && (result.candidate != candidate || result.precision_case != precision_case || result.mode != mode))
		{
			return {}, report_error(.Invalid_Measurement, "precision candidate, case, mode or profile mismatch");
		}
		if candidate == "baseline" && (precision_case == "compact-1000km" || precision_case == "compact-1gm")
		{
			return {}, report_error(.Invalid_Measurement, "expected accuracy failures cannot be successful benchmark rows");
		}
		result.candidate, result.precision_case, result.mode = candidate, precision_case, mode;
		expected: i64 = 8 if precision_case == "queries-local" || precision_case == "queries-spanning" else 512;
		for field in ([3]int{3, 9, 10})
		{
			value, parsed := strconv.parse_i64(row[columns[field]]);
			if !parsed || value != (i64(worker) if field == 9 else expected)
			{
				return {}, report_error(.Invalid_Measurement, "precision worker or observation count mismatch");
			}
		}
		result.count = int(expected);
		if row[columns[11]] != "ok" || row[columns[12]] != "ok"
		{
			return {}, report_error(.Invalid_Status, "non-ok precision status");
		}
		maintenance, maintenance_parsed := strconv.parse_i64(row[columns[6]]);
		expected_maintenance: i64;
		if candidate == "origin" && precision_case == "shift-sparse"
		{
			expected_maintenance = 2;
		}
		if candidate == "origin" && precision_case == "shift-stress"
		{
			expected_maintenance = 32;
		}
		if !maintenance_parsed || maintenance != expected_maintenance
		{
			return {}, report_error(.Invalid_Measurement, "precision maintenance count mismatch");
		}
		result.maintenance_count = int(maintenance);
		checksum, checksum_parsed := strconv.parse_f64(row[columns[8]]);
		if !checksum_parsed || math.is_nan(checksum) || math.is_inf(checksum)
		{
			return {}, report_error(.Invalid_Measurement, "nonfinite precision checksum");
		}
		result.checksum = checksum;
		for field in ([3]int{4, 5, 7})
		{
			text: string = row[columns[field]];
			required: Result_Availability = .Unavailable;
			if (field != 7 && (mode == "instrumented" || mode == "engine-only")) ||
				(field == 7 && mode == "instrumented" && maintenance > 0)
			{
				required = .Available;
			}
			if required == .Unavailable
			{
				if len(text) != 0
				{
					return {}, report_error(.Invalid_Measurement, "unavailable precision latency must be empty");
				}
				continue;
			}
			value, parsed := strconv.parse_f64(text);
			if !parsed || value <= 0 || math.is_nan(value) || math.is_inf(value)
			{
				return {}, report_error(.Invalid_Measurement, "invalid precision latency");
			}
			switch field
			{
				case 4: result.frame_median_ms = max(result.frame_median_ms, value);
				case 5: result.frame_max_ms = max(result.frame_max_ms, value);
				case 7: result.maintenance_max_ms = max(result.maintenance_max_ms, value);
			}
		}
		if result.frame_max_ms < result.frame_median_ms
		{
			return {}, report_error(.Invalid_Measurement, "frame maximum is below median");
		}
		method: string = "allocator_instrumented" if mode == "memory" else "native_step_sum" if mode == "engine-only" else "whole_loop";
		if row[columns[14]] != method
		{
			return {}, report_error(.Invalid_Measurement, "precision timing method mismatch");
		}
		if mode == "memory"
		{
			if len(row[columns[13]]) != 0
			{
				return {}, report_error(.Invalid_Measurement, "memory samples cannot contain timing");
			}
			for name, field in PRECISION_MEMORY_NAMES
			{
				column: int;
				status: Report_Status;
				column, status = header_index(records[0], name);
				if status != .Ok
				{
					return {}, report_error(.Invalid_Header, "missing or duplicate memory field: %s", name);
				}
				value, parsed := strconv.parse_u64(row[column]);
				if !parsed || ((field == 11 || field == 12 || (field == 5 && expected == 8)) && value != 0)
				{
					return {}, report_error(.Invalid_Measurement, "memory leak or reserved-operation allocation");
				}
				result.memory[field] = max(result.memory[field], value);
			}
		}
	}
	return result, {status=.Ok};
}
