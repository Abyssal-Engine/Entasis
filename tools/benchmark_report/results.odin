package benchmark_report

import "base:runtime"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"

Result_State :: enum
{
	Empty, Loading, Ready,
}
Result_Unit :: enum
{
	Text, Milliseconds, Nanoseconds, Per_Second,
}
Result_Availability :: enum
{
	Unavailable, Available,
}
Result_Value :: struct
{
	value: f64,
	availability: Result_Availability,
}
Result_Column :: struct
{
	name: string,
	unit: Result_Unit,
	summary: Result_Summary,
}
Result_Summary :: struct
{
	sample_count: int,
	median, minimum, maximum: f64,
}
Result_Lane :: struct
{
	path: string,
	label: string,
	measurement: string,
	unit: Result_Unit,
	records: [][]string,
	columns: []Result_Column,
	values: []Result_Value,
	summary: Lane_Summary,
	csv: Result_CSV,
}
Result_Dataset :: struct
{
	budget: u64,
	csv: Result_CSV,
	diagnostic: string,
	state: Result_State,
	path, package_name: string,
	lanes: [dynamic]Result_Lane,
}

result_lane_close :: proc(lane: ^Result_Lane)
{
	if lane.csv.storage == nil && len(lane.records) > 0
	{
		delete(lane.records[0]);
		delete(lane.records);
	}
	delete(lane.csv.storage);
	delete(lane.columns);
	delete(lane.values);
	delete(lane.path);
	delete(lane.label);
	lane^ = {};
}

result_lane_bytes :: proc(lane: ^Result_Lane) -> u64
{
	bytes: u64 = u64(len(lane.csv.storage)+len(lane.path)+len(lane.label))+
		u64(len(lane.columns))*size_of(Result_Column)+u64(len(lane.values))*size_of(Result_Value);
	if lane.csv.storage == nil && len(lane.records) > 0
	{
		bytes += u64(len(lane.records))*size_of([]string)+u64(len(lane.records[0]))*size_of(string);
	}
	return bytes;
}

result_dataset_bytes :: proc(dataset: ^Result_Dataset) -> u64
{
	bytes: u64 = u64(len(dataset.path)+len(dataset.package_name)+len(dataset.diagnostic)+len(dataset.csv.storage))+
		u64(cap(dataset.lanes))*size_of(Result_Lane);
	for &lane in dataset.lanes
	{
		bytes += result_lane_bytes(&lane);
	}
	return bytes;
}

result_dataset_close :: proc(dataset: ^Result_Dataset)
{
	for &lane in dataset.lanes
	{
		result_lane_close(&lane);
	}
	delete(dataset.lanes);
	delete(dataset.csv.storage);
	delete(dataset.path);
	delete(dataset.package_name);
	delete(dataset.diagnostic);
	dataset^ = {};
}

result_reserve_lanes :: proc(dataset: ^Result_Dataset, count: int) -> Report_Result
{
	if count <= cap(dataset.lanes)
	{
		return {status=.Ok};
	}
	capacity: int = max(count, max(4, cap(dataset.lanes)*2));
	required: u64 = result_dataset_bytes(dataset)+u64(capacity)*size_of(Result_Lane);
	if required > dataset.budget
	{
		return result_budget_error(required, dataset.budget);
	}
	if reserve(&dataset.lanes, capacity) != nil
	{
		return {status=.Allocation_Failed};
	}
	return {status=.Ok};
}

result_column_unit :: proc(name: string) -> Result_Unit
{
	if strings.has_suffix(name, "_ms")
	{
		return .Milliseconds;
	}
	if strings.has_suffix(name, "_ns") || name == "nanoseconds"
	{
		return .Nanoseconds;
	}
	if strings.has_suffix(name, "_per_second")
	{
		return .Per_Second;
	}
	return .Text;
}

result_lane_summaries :: proc(lane: ^Result_Lane, budget: u64) -> Report_Result
{
	samples: int = len(lane.records)-1;
	required: u64 = result_lane_bytes(lane)+u64(samples)*size_of(f64);
	if required > budget
	{
		return result_budget_error(required, budget);
	}
	measurements: []f64;
	allocation_error: runtime.Allocator_Error;
	measurements, allocation_error = make([]f64, samples);
	if allocation_error != nil
	{
		return {status=.Allocation_Failed};
	}
	defer delete(measurements);
	for &column, column_index in lane.columns
	{
		if column.unit == .Text
		{
			continue;
		}
		count: int;
		for row in 0 ..< samples
		{
			value: Result_Value = lane.values[row*len(lane.columns)+column_index];
			if value.availability == .Available
			{
				measurements[count] = value.value;
				count += 1;
			}
		}
		if count == 0
		{
			column.summary = {};
			continue;
		}
		sort_measurements(measurements[:count]);
		middle: int = count/2;
		median: f64 = measurements[middle];
		if count%2 == 0
		{
			median = (measurements[middle-1]+median)/2;
		}
		column.summary = {count, median, measurements[0], measurements[count-1]};
	}
	return {status=.Ok};
}

result_lane_read :: proc(path: string, worker: int, package_name: string, budget: u64) -> (output: Result_Lane, result: Report_Result)
{
	lane: Result_Lane;
	defer if result.status != .Ok
	{
		result_lane_close(&lane);
	}
	if u64(len(path)) > budget
	{
		return {}, result_budget_error(u64(len(path)), budget);
	}
	lane.csv, result = result_csv_read(path, budget-u64(len(path)));
	if result.status != .Ok
	{
		return {}, result;
	}
	records: [][]string = lane.csv.records;
	if len(records) < 2 || len(records[0]) == 0
	{
		return {}, report_error(.Csv_Invalid, "empty or invalid CSV: %s", path);
	}
	base_bytes: u64 = u64(len(lane.csv.storage)+len(path))+u64(len(records[0]))*size_of(Result_Column);
	row_bytes: u64 = u64(len(records[0]))*size_of(Result_Value)+size_of(f64);
	if u64(len(records)-1) > (max(u64)-base_bytes)/row_bytes
	{
		return {}, result_budget_error(max(u64), budget);
	}
	required: u64 = base_bytes+u64(len(records)-1)*row_bytes;
	if required > budget || required > u64(max(int))
	{
		return {}, result_budget_error(required, budget);
	}
	lane.path, lane.records = strings.clone(path), records;
	for name, index in records[0]
	{
		if len(name) == 0
		{
			return {}, report_error(.Invalid_Header, "empty header in %s", path);
		}
		for prior in records[0][:index]
		{
			if name == prior
			{
				return {}, report_error(.Invalid_Header, "duplicate %s in %s", name, path);
			}
		}
	}
	effective_package: string = package_name;
	if len(effective_package) == 0
	{
		column, status := header_index(records[0], "benchmark_parameters");
		if status == .Ok && len(records[1]) == len(records[0])
		{
			names: [3]string = {"container", "contact_islands", "pyramid"};
			for name in names
			{
				prefix: string = strings.concatenate({"workload=", name, ";"});
				defer delete(prefix);
				if strings.has_prefix(records[1][column], prefix)
				{
					effective_package = name;
					break;
				}
			}
			if len(effective_package) == 0
			{
				return {}, report_error(.Invalid_Measurement, "unknown parameterized workload in %s", path);
			}
		}
		else if _, status = header_index(records[0], "batch_size"); status == .Ok
		{
			effective_package = "spatial_query_batch";
		}
	}
	lane.summary, result = summarize_records(records, path, len(records)-1, worker, effective_package);
	if result.status != .Ok
	{
		return {}, result;
	}
	lane.measurement = "ray_elapsed_ms" if effective_package == "spatial_query_batch" else "physics_elapsed_ms";
	lane.unit = .Milliseconds;
	lane.columns = make([]Result_Column, len(records[0]));
	lane.values = make([]Result_Value, (len(records)-1)*len(lane.columns));
	for name, column in records[0]
	{
		lane.columns[column] = {name=name, unit=result_column_unit(name)};
		for record, row in records[1:]
		{
			if name == "worker_count"
			{
				count, parsed := strconv.parse_i64(record[column]);
				if !parsed || count != i64(worker)
				{
					return {}, report_error(.Invalid_Measurement, "worker count mismatch in %s", path);
				}
			}
			if lane.columns[column].unit == .Text
			{
				if name == "benchmark_components" && record[column] != records[1][column]
				{
					return {}, report_error(.Invalid_Measurement, "mixed component selections in %s", path);
				}
				continue;
			}
			text: string = record[column];
			if text == "" || text == "unavailable" || text == "excluded" || text == "n/a"
			{
				continue;
			}
			value, parsed := strconv.parse_f64(text);
			if !parsed || math.is_nan(value) || math.is_inf(value) || value < 0
			{
				return {}, report_error(.Invalid_Measurement, "invalid %s in %s row %d", name, path, row+2);
			}
			lane.values[row*len(lane.columns)+column] = {value, .Available};
		}
	}
	result = result_lane_summaries(&lane, budget);
	if result.status != .Ok
	{
		return {}, result;
	}
	return lane, result;
}

result_dataset_open :: proc(dataset: ^Result_Dataset, path, package_name: string, budget: u64 = 512*1024*1024) -> (result: Report_Result)
{
	result_dataset_close(dataset);
	dataset.budget, dataset.state = budget, .Loading;
	defer dataset.diagnostic = result.diagnostic;
	if u64(len(path))+u64(len(package_name)) > budget
	{
		return result_budget_error(u64(len(path))+u64(len(package_name)), budget);
	}
	dataset.path, dataset.package_name = strings.clone(path), strings.clone(package_name);
	if !os.is_dir(path)
	{
		return result_overhead_read(dataset, path, package_name);
	}
	directory, directory_error := os.open(path);
	if directory_error != nil
	{
		return report_error(.Directory_Read_Failed, "failed to read result directory: %s", path);
	}
	defer os.close(directory);
	iterator: os.Read_Directory_Iterator = os.read_directory_iterator_create(directory);
	defer os.read_directory_iterator_destroy(&iterator);
	for entry in os.read_directory_iterator(&iterator)
	{
		if entry.type != .Regular || !strings.has_suffix(entry.name, ".csv")
		{
			continue;
		}
		if !strings.has_prefix(entry.name, "workers-")
		{
			return report_error(.Unexpected_Lane, "unexpected CSV lane: %s", entry.name);
		}
		worker, parsed := strconv.parse_i64(entry.name[8:len(entry.name)-4]);
		if !parsed || worker < 1 || worker > 2147483647
		{
			return report_error(.Unexpected_Lane, "invalid worker lane: %s", entry.name);
		}
		lane: Result_Lane;
		result = result_reserve_lanes(dataset, len(dataset.lanes)+1);
		if result.status != .Ok
		{
			return result;
		}
		lane, result = result_lane_read(entry.fullpath, int(worker), package_name, budget-result_dataset_bytes(dataset));
		if result.status != .Ok
		{
			return result;
		}
		defer if result.status != .Ok
		{
			result_lane_close(&lane);
		}
		if len(dataset.lanes) > 0
		{
			first: Result_Lane = dataset.lanes[0];
			if lane.summary.sample_count != first.summary.sample_count ||
				lane.summary.benchmark_parameters != first.summary.benchmark_parameters ||
				lane.summary.measured_steps != first.summary.measured_steps ||
				lane.summary.recording_mode != first.summary.recording_mode || lane.summary.timing_method != first.summary.timing_method ||
				lane.summary.ray_checksum != first.summary.ray_checksum || len(lane.columns) != len(first.columns)
			{
				return report_error(.Invalid_Measurement, "inconsistent workload or samples in %s", entry.fullpath);
			}
			for column, index in lane.columns
			{
				if column.name != first.columns[index].name || column.unit != first.columns[index].unit ||
					(column.name == "benchmark_components" && lane.records[1][index] != first.records[1][index])
				{
					return report_error(.Invalid_Header, "inconsistent component columns in %s", entry.fullpath);
				}
			}
		}
		for previous in dataset.lanes
		{
			if previous.summary.worker_count == int(worker)
			{
				return report_error(.Unexpected_Lane, "duplicate worker lane: %s", entry.name);
			}
		}
		append(&dataset.lanes, lane);
		lane = {};
	}
	if _, error := os.read_directory_iterator_error(&iterator); error != nil
	{
		return report_error(.Directory_Read_Failed, "failed to enumerate %s", path);
	}
	if len(dataset.lanes) == 0
	{
		return report_error(.Missing_Lane, "no worker CSV files in %s", path);
	}
	for index in 1 ..< len(dataset.lanes)
	{
		lane: Result_Lane = dataset.lanes[index];
		cursor: int = index;
		for cursor > 0 && dataset.lanes[cursor-1].summary.worker_count > lane.summary.worker_count
		{
			dataset.lanes[cursor] = dataset.lanes[cursor-1];
			cursor -= 1;
		}
		dataset.lanes[cursor] = lane;
	}
	baseline: f64 = dataset.lanes[0].summary.median;
	for &lane in dataset.lanes
	{
		lane.summary.speedup = baseline/lane.summary.median;
	}
	dataset.state = .Ready;
	return {status=.Ok};
}

Result_Recording :: enum
{
	Legacy, Off, On,
}
Result_Timing :: enum
{
	Unavailable, Whole_Loop, Native_Step_Sum,
}

result_recording_conditions :: proc(records: [][]string) -> (Result_Recording, Result_Timing, Report_Result)
{
	columns: [4]int = {-1, -1, -1, -1};
	names: [4]string = {"recording_path", "recording_component", "recording_mode", "timing_method"};
	found: int;
	for name, index in names
	{
		for column, field in records[0]
		{
			if column == name
			{
				if columns[index] != -1
				{
					return {}, {}, report_error(.Invalid_Header, "duplicate recording column");
				}
				columns[index] = field;
				found += 1;
			}
		}
	}
	if found == 0
	{
		return .Legacy, .Unavailable, {status=.Ok};
	}
	if found != 4
	{
		return {}, {}, report_error(.Invalid_Header, "incomplete recording association");
	}
	mode: Result_Recording;
	timing: Result_Timing;
	for row, index in records[1:]
	{
		if len(row) != len(records[0])
		{
			return {}, {}, report_error(.Csv_Invalid, "invalid recording row width");
		}
		path, component, row_mode, method := row[columns[0]], row[columns[1]], row[columns[2]], row[columns[3]];
		if (row_mode != "off" && row_mode != "on") || (method != "whole_loop" && method != "native_step_sum") ||
			(row_mode == "off" && (len(path) != 0 || len(component) != 0)) ||
			(row_mode == "on" && (len(path) == 0 || component != "physics_elapsed_ms" || method != "native_step_sum"))
		{
			return {}, {}, report_error(.Invalid_Measurement, "invalid recording or timing conditions");
		}
		if row_mode == "on"
		{
			if path == "." || path == ".." || strings.has_suffix(path, ".partial")
			{
				return {}, {}, report_error(.Invalid_Measurement, "invalid recording path");
			}
			for c in path
			{
				if !(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') && c != '_' && c != '-' && c != '.'
				{
					return {}, {}, report_error(.Invalid_Measurement, "recording path must name a file within the run");
				}
			}
			for previous in records[1:index+1]
			{
				if previous[columns[0]] == path
				{
					return {}, {}, report_error(.Invalid_Measurement, "recording reused by different samples");
				}
			}
		}
		current_mode: Result_Recording = .On if row_mode == "on" else .Off;
		current_timing: Result_Timing = .Native_Step_Sum if method == "native_step_sum" else .Whole_Loop;
		if index > 0 && (current_mode != mode || current_timing != timing)
		{
			return {}, {}, report_error(.Invalid_Measurement, "mixed recording or timing conditions");
		}
		mode, timing = current_mode, current_timing;
	}
	return mode, timing, {status=.Ok};
}

result_sample_recording :: proc(lane: ^Result_Lane, sample: int, component: string) -> string
{
	if lane.summary.recording_mode != .On
	{
		return "";
	}
	path_column, _ := header_index(lane.records[0], "recording_path");
	component_column, _ := header_index(lane.records[0], "recording_component");
	if lane.records[sample+1][component_column] != component
	{
		return "";
	}
	return lane.records[sample+1][path_column];
}
