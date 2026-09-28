package physics_viewer

import "core:strings"
import "core:os"
import report "../benchmark_report"
import scene "../physics_scene"

Recording_State :: enum
{
	Missing, Present, Rejected, Failed,
}
Recording_File :: struct
{
	state: Recording_State,
	bytes: u64,
	error: os.Error,
}
Recording_Filter :: enum
{
	All, With, Without,
}
Recording_Target :: struct
{
	name: string,
	worker, sample: int,
	state: Recording_State,
	bytes: u64,
}
Recording_Inventory :: struct
{
	entries: []Recording_Target,
	owned_bytes: u64,
	present: int,
	state: Recording_State,
}
Recording_Path_Kind :: enum
{
	File, Directory,
}

recording_directory_inspect :: proc(path: string) -> Recording_File
{
	current: string = path;
	for
	{
		file: Recording_File = result_recording_file(current, .Directory);
		if file.state != .Present
		{
			return file;
		}
		parent: string = os.dir(current);
		if parent == current || len(parent) == 0
		{
			return file;
		}
		current = parent;
	}
}

recording_inspect :: proc(run, name: string) -> Recording_File
{
	if len(name) == 0
	{
		return {};
	}
	if !strings.has_suffix(name, ".epr")
	{
		return {state=.Rejected};
	}
	buffer: File_Path_Buffer;
	path: string;
	status: scene.Status;
	path, status = recording_join_path(buffer[:], run, name);
	if status != .Ok
	{
		return {state=.Failed, error=os.General_Error.Invalid_Path};
	}
	return result_recording_file(path);
}

recording_inventory_close :: proc(inventory: ^Recording_Inventory)
{
	for entry in inventory.entries
	{
		delete(entry.name);
	}
	delete(inventory.entries);
	inventory^ = {};
}

recording_inventory_load :: proc(inventory: ^Recording_Inventory, dataset: ^report.Result_Dataset, budget: u64) -> scene.Status
{
	recording_inventory_close(inventory);
	count: int;
	bytes: u64;
	for &lane in dataset.lanes
	{
		count += len(lane.records)-1;
		for sample in 0 ..< len(lane.records)-1
		{
			bytes += u64(len(report.result_sample_recording(&lane, sample, "physics_elapsed_ms")));
		}
	}
	bytes += u64(count)*size_of(Recording_Target);
	if bytes > budget
	{
		inventory.state = .Failed;
		return .Budget_Exceeded;
	}
	inventory.entries = make([]Recording_Target, count);
	inventory.owned_bytes = bytes;
	index: int;
	for &lane in dataset.lanes
	{
		for sample in 0 ..< len(lane.records)-1
		{
			name: string = report.result_sample_recording(&lane, sample, "physics_elapsed_ms");
			file: Recording_File = recording_inspect(dataset.path, name);
			inventory.entries[index] = {strings.clone(name), lane.summary.worker_count, sample, file.state, file.bytes};
			if file.state == .Present
			{
				inventory.present += 1;
			}
			if file.state == .Failed
			{
				inventory.state = .Failed;
			}
			index += 1;
		}
	}
	if inventory.present > 0
	{
		inventory.state = .Present;
	}
	return .Ok;
}

recording_presence :: proc(path, package_name: string, budget: u64) -> (Recording_State, scene.Status)
{
	dataset: report.Result_Dataset;
	defer report.result_dataset_close(&dataset);
	result: report.Report_Result = report.result_dataset_open(&dataset, path, package_name, budget);
	if result.status != .Ok
	{
		return .Failed, .Budget_Exceeded if result.status == .Budget_Exceeded else (.Out_Of_Memory if result.status == .Allocation_Failed else .Ok);
	}
	state: Recording_State = .Missing;
	for &lane in dataset.lanes
	{
		for sample in 0 ..< len(lane.records)-1
		{
			file: Recording_File = recording_inspect(path, report.result_sample_recording(&lane, sample, "physics_elapsed_ms"));
			if file.state == .Present
			{
				return .Present, .Ok;
			}
			if file.state == .Failed
			{
				state = .Failed;
			}
		}
	}
	return state, .Ok;
}

recording_current_index :: proc(v: ^Viewer) -> int
{
	return int(v.ui.result_lane)*(len(v.results.lanes[0].records)-1)+int(v.ui.result_sample);
}

recording_refresh_current :: proc(v: ^Viewer)
{
	recording_inventory_close(&v.ui.recording_inventory);
	v.ui.status = recording_inventory_load(&v.ui.recording_inventory, &v.results, v.budget-min(v.budget, viewer_content_bytes(v)));
	for &entry in v.ui.runs.entries
	{
		if entry.path == v.results.path
		{
			entry.recording = v.ui.recording_inventory.state;
		}
	}
	results_filter(v);
	recordings_close(&v.ui.recordings);
}


// windows paths contain at most 32767 utf-16 units, or three utf-8 bytes per unit
// linux pathname syscalls accept at most 4095 bytes plus the terminator
File_Path_Buffer :: [32768*3]u8 when ODIN_OS == .Windows else [4096]u8;

recording_join_path :: proc(buffer: []u8, root, leaf: string) -> (string, scene.Status)
{
	length: int = len(root)+1+len(leaf);
	if length >= len(buffer)
	{
		return "", .File_Error;
	}
	copy(buffer, root);
	buffer[len(root)] = '/';
	copy(buffer[len(root)+1:], leaf);
	buffer[length] = 0;
	return string(buffer[:length]), .Ok;
}
