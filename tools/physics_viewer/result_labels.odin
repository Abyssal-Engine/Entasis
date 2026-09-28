package physics_viewer

import "base:runtime"
import "core:fmt"
import "core:strings"
import report "../benchmark_report"
import scene "../physics_scene"

Result_Labels :: struct
{
	storage: []u8,
	names: []cstring,
	lanes, samples, metrics: []cstring,
	columns: []int,
	owned_bytes: u64,
}

result_labels_close :: proc(labels: ^Result_Labels)
{
	delete(labels.storage);
	delete(labels.names);
	delete(labels.columns);
	labels^ = {};
}

result_metric_name :: proc(name: string) -> string
{
	switch name
	{
	case "physics_elapsed_ms": return "Total physics time";
	case "ray_elapsed_ms": return "Total ray query time";
	case "elapsed_ms": return "Total elapsed time";
	case "nanoseconds": return "Time per operation";
	}
	if strings.has_suffix(name, "_ms") || strings.has_suffix(name, "_ns")
	{
		return name[:len(name)-3];
	}
	return name;
}

result_labels_open :: proc(labels: ^Result_Labels, dataset: ^report.Result_Dataset, budget: u64) -> (status: scene.Status)
{
	defer if status != .Ok
	{
		result_labels_close(labels);
	}
	lanes: int = len(dataset.lanes);
	samples: int = len(dataset.lanes[0].records)-1;
	metrics, length: int;
	for column in dataset.lanes[0].columns
	{
		if column.unit != .Text
		{
			metrics += 1;
			length += len(result_metric_name(column.name))+1;
		}
	}
	// numeric labels contain at most two decimal int values and fixed text
	length += (lanes+samples)*64;
	bytes: u64 = u64(length)+u64(lanes+samples+metrics)*size_of(cstring)+u64(metrics)*size_of(int);
	if bytes > budget
	{
		return .Budget_Exceeded;
	}
	error: runtime.Allocator_Error;
	labels.storage, error = make([]u8, length);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	labels.names, error = make([]cstring, lanes+samples+metrics);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	labels.columns, error = make([]int, metrics);
	if error != nil
	{
		return .Out_Of_Memory;
	}
	labels.owned_bytes = bytes;
	labels.lanes = labels.names[:lanes];
	labels.samples = labels.names[lanes:lanes+samples];
	labels.metrics = labels.names[lanes+samples:];
	offset: int;
	for lane, index in dataset.lanes
	{
		labels.lanes[index] = cast(cstring)&labels.storage[offset];
		fmt.bprintf(labels.storage[offset:offset+63], "%d workers", lane.summary.worker_count);
		offset += 64;
	}
	for _, index in labels.samples
	{
		labels.samples[index] = cast(cstring)&labels.storage[offset];
		fmt.bprintf(labels.storage[offset:offset+63], "Sample %d of %d", index+1, samples);
		offset += 64;
	}
	metric: int;
	for column, index in dataset.lanes[0].columns
	{
		if column.unit == .Text
		{
			continue;
		}
		name: string = result_metric_name(column.name);
		labels.metrics[metric] = cast(cstring)&labels.storage[offset];
		ui_name(name, labels.storage[offset:offset+len(name)]);
		labels.columns[metric] = index;
		offset += len(name)+1;
		metric += 1;
	}
	return .Ok;
}
