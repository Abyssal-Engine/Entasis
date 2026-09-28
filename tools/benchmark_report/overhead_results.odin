package benchmark_report

import "core:math"
import "core:strconv"
import "core:strings"

result_overhead_read :: proc(dataset: ^Result_Dataset, path, package_name: string) -> Report_Result
{
	identity: string;
	switch package_name
	{
	case "c_abi_overhead/native": identity = "Native";
	case "c_abi_overhead/c": identity = "C";
	case "c_abi_overhead": identity = "Lane unavailable";
	case: return report_error(.Invalid_Arguments, "raw overhead rows require an explicit c_abi_overhead package selection");
	}
	result: Report_Result;
	dataset.csv, result = result_csv_read(path, dataset.budget-result_dataset_bytes(dataset));
	if result.status != .Ok
	{
		return result;
	}
	records: [][]string = dataset.csv.records;
	if len(records) == 0
	{
		return report_error(.Csv_Invalid, "empty overhead rows in %s", path);
	}
	names: [5]string = {"step_10k", "create_10k", "mutate_10k", "query_batch", "event_drain"};
	counts, first_rows: [5]int;
	for record, row in records
	{
		if len(record) != 4
		{
			return report_error(.Csv_Invalid, "overhead row %d requires scenario, nanoseconds, operations and checksum", row+1);
		}
		group: int = -1;
		for name, index in names
		{
			if record[0] == name
			{
				group = index;
				break;
			}
		}
		nanoseconds, time_ok := strconv.parse_i64(record[1]);
		operations, count_ok := strconv.parse_i64(record[2]);
		checksum, checksum_ok := strconv.parse_f64(record[3]);
		if group < 0 || !time_ok || nanoseconds <= 0 || !count_ok || operations <= 0 ||
			!checksum_ok || math.is_nan(checksum) || math.is_inf(checksum)
		{
			return report_error(.Invalid_Measurement, "invalid overhead row %d in %s", row+1, path);
		}
		if counts[group] > 0 &&
			(record[2] != records[first_rows[group]][2] || record[3] != records[first_rows[group]][3])
		{
			return report_error(.Invalid_Measurement, "mixed operation counts or checksums for %s", record[0]);
		}
		if counts[group] == 0
		{
			first_rows[group] = row;
		}
		counts[group] += 1;
	}
	for count, index in counts
	{
		if count == 0
		{
			continue;
		}
		result = result_reserve_lanes(dataset, len(dataset.lanes)+1);
		if result.status != .Ok
		{
			return result;
		}
		required: u64 = result_dataset_bytes(dataset)+u64(len(path)+len(identity)+len(names[index])+3)+
			4*size_of(Result_Column)+u64(count+1)*size_of([]string)+4*size_of(string)+
			u64(count)*(4*size_of(Result_Value)+size_of(f64));
		if required > dataset.budget
		{
			return result_budget_error(required, dataset.budget);
		}
		lane: Result_Lane = {path=strings.clone(path), label=strings.concatenate({identity, " | ", names[index]}),
			measurement="nanoseconds", unit=.Nanoseconds};
		lane.columns = make([]Result_Column, 4);
		copy(lane.columns, []Result_Column{{name="scenario", unit=.Text}, {name="nanoseconds", unit=.Nanoseconds}, {name="operations", unit=.Text}, {name="checksum", unit=.Text}});
		lane.records = make([][]string, count+1);
		lane.records[0] = make([]string, 4);
		copy(lane.records[0], []string{"scenario", "nanoseconds", "operations", "checksum"});
		lane.values = make([]Result_Value, count*4);
		measurements: []f64 = make([]f64, count);
		defer delete(measurements);
		row: int;
		for record in records
		{
			if record[0] != names[index]
			{
				continue;
			}
			lane.records[row+1] = record;
			nanoseconds, _ := strconv.parse_i64(record[1]);
			measurements[row] = f64(nanoseconds);
			lane.values[row*4+1] = {f64(nanoseconds), .Available};
			row += 1;
		}
		sort_measurements(measurements);
		middle: int = len(measurements)/2;
		median: f64 = measurements[middle];
		if len(measurements)%2 == 0
		{
			median = (median+measurements[middle-1])/2;
		}
		lane.summary = {sample_count=count, median=median,
			minimum=measurements[0], maximum=measurements[len(measurements)-1]};
		lane.columns[1].summary = {count, median, lane.summary.minimum, lane.summary.maximum};
		append(&dataset.lanes, lane);
	}
	dataset.state = .Ready;
	return {status=.Ok};
}
