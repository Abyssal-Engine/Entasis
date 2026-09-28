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

Report_Request :: struct
{
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
	recording_mode, timing_method, recording_status := result_recording_conditions(records);
	if recording_status.status != .Ok
	{
		return {}, recording_status;
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
	ray_columns: [7]int
	if package_name == "spatial_query_batch"
	{
		names: [7]string = {"worker_count", "batch_size", "ray_query_count", "warmup_batches", "measured_batches", "ray_hit_count", "ray_checksum"}
		if len(records[0]) != len(names) + 2
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
	for row_index in 0..<expected_rows
	{
		record := records[row_index + 1]
		if len(record) != len(records[0])
		{
			return {}, report_error(.Csv_Invalid, "column count mismatch in %s row %d", path, row_index + 2)
		}
		if record[case_status_index] != "ok" || record[metric_status_index] != "ok"
		{
			return {}, report_error(.Invalid_Status, "non-ok status in %s row %d", path, row_index + 2)
		}
		if package_name == "spatial_query_batch"
		{
			expected: [6]u64 = {u64(worker_count), 256, 50_000, 10, 100, 25_000}
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
		measurement, parsed := strconv.parse_f64(record[measurement_index])
		if !parsed || math.is_nan(measurement) || math.is_inf(measurement) || measurement <= 0
		{
			return {}, report_error(.Invalid_Measurement, "invalid %s in %s row %d", measurement_name, path, row_index + 2)
		}
		measurements[row_index] = measurement
	}
	sort_measurements(measurements)
	middle := expected_rows / 2
	median := measurements[middle]
	if expected_rows % 2 == 0
	{
		median = (measurements[middle - 1] + measurements[middle]) / 2
	}
	return Lane_Summary{
		recording_mode=recording_mode, timing_method=timing_method,
		worker_count = worker_count,
		sample_count = expected_rows,
		median = median,
		minimum = measurements[0],
		maximum = measurements[len(measurements) - 1],
		ray_checksum = ray_checksum,
		benchmark_parameters = parameters,
		measured_steps = measured_steps,
	}, Report_Result{status = .Ok}
}

collect_summaries :: proc(request: Report_Request) -> Report_Result
{
	if (workload_kind(request.package_name) == .Parameterized && variant_status(request.variant) != .Ok) ||
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
		summary.speedup = baseline / summary.median
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
	report_format_line(&builder, "# Benchmark result: %s", request.package_name)
	report_line(&builder)
	report_format_line(&builder, "- Configuration: `%s`", request.configuration)
	report_format_line(&builder, "- Recording: %v | Timing: %v", summaries[0].recording_mode, summaries[0].timing_method)
	report_format_line(&builder, "- Run date (UTC): `%s`", request.run_date)
	report_format_line(&builder, "- Odin compiler: `%s`", compiler_version)
	report_format_line(&builder, "- Operating system: `%s`", request.operating_system)
	report_format_line(&builder, "- Processor: `%s`", request.processor)
	report_format_line(&builder, "- Logical processors: `%d`", request.logical_processors)
	report_line(&builder)
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
		report_line(&builder, "## ray_elapsed_ms summary")
		report_line(&builder)
		report_line(&builder, "Public query_batch, batch size 256. Each run measures 100 frames of 50,000 closest rays after 10 warmup frames. Times below are totals in milliseconds: divide by 100 for ms/frame, then calculate million rays/s = 50 / ms/frame. Scalar output validation runs before and after timing. Worker counts include the caller. Windows controls placement without affinity pinning")
	}
	else
	{
		report_line(&builder, "## physics_elapsed_ms summary")
	}
	report_line(&builder)
	if request.package_name == "query_extensions"
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
	report_line(&builder, "| Workers | Samples | Median | Minimum | Maximum | Speedup |")
	report_line(&builder, "| ------: | ------: | -----: | ------: | ------: | ------: |")
	for summary in summaries
	{
		report_format_line(
			&builder,
			"| %d | %d | %.6f | %.6f | %.6f | %.6fx |",
			summary.worker_count,
			summary.sample_count,
			summary.median,
			summary.minimum,
			summary.maximum,
			summary.speedup,
		)
	}
	report_line(&builder)
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
		case "container", "contact_islands", "pyramid": return .Parameterized
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
