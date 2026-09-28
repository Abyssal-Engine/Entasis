package physics_viewer

import "base:runtime"
import "core:fmt"
import "core:strings"
import rl "vendor:raylib"
import scene "../physics_scene"
import report "../benchmark_report"

when ODIN_OS != .Windows
{
	viewer_open_file :: proc(filename: string, leaf: string = "")
	{
		path_buffer: File_Path_Buffer;
		path: string = filename;
		status: scene.Status;
		if len(leaf) > 0
		{
			path, status = recording_join_path(path_buffer[:], filename, leaf);
			if status != .Ok
			{
				return;
			}
		}
		path, status = recording_absolute_path(path, path_buffer[:]);
		if status != .Ok
		{
			fmt.eprintfln("Open file: %v", status);
			return;
		}
		// percent encoding expands each pathname byte to at most three bytes
		encoded: [3*size_of(File_Path_Buffer)+9]u8;
		prefix: string = "file://" if strings.has_prefix(path, "/") else "file:///";
		count: int = copy(encoded[:], prefix);
		hex: string = "0123456789ABCDEF";
		for c in transmute([]u8)path
		{
			if c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' ||
				c == '/' || c == ':' || c == '-' || c == '_' || c == '.' || c == '~'
			{
				encoded[count] = c;
				count += 1;
			}
			else
			{
				encoded[count], encoded[count+1], encoded[count+2] = '%', hex[c>>4], hex[c&15];
				count += 3;
			}
		}
		rl.OpenURL(cast(cstring)raw_data(encoded[:]));
	}
}

Selection_Action :: enum
{
	None, Compare,
}
Recording_Entry :: struct
{
	sample: Result_Sample,
	label: cstring,
}
Recording_List :: struct
{
	entries: [dynamic]Recording_Entry,
	names: []cstring,
	owned_bytes: u64,
	case_id: string,
	selected, scroll, focus: i32,
}

recordings_close :: proc(list: ^Recording_List)
{
	for entry in list.entries
	{
		delete(entry.sample.run);
		delete(entry.sample.lane);
		delete(entry.sample.package_name);
		delete(entry.sample.component);
		delete(entry.label);
	}
	delete(list.entries);
	delete(list.names);
	delete(list.case_id);
	list^ = {selected=-1, focus=-1};
}

recordings_refresh :: proc(v: ^Viewer, case_id: string) -> (status: scene.Status)
{
	defer if status != .Ok
	{
		recordings_close(&v.ui.recordings);
	}
	held: u64 = viewer_content_bytes(v)-v.ui.recordings.owned_bytes;
	if u64(len(case_id)) > v.budget-min(v.budget, held)
	{
		return .Budget_Exceeded;
	}
	id: string;
	id_error: runtime.Allocator_Error;
	id, id_error = strings.clone(case_id);
	if id_error != nil
	{
		return .Out_Of_Memory;
	}
	recordings_close(&v.ui.recordings);
	list: ^Recording_List = &v.ui.recordings;
	list.case_id = id;
	list.owned_bytes = u64(len(id));
	package_name: string;
	for definition in BENCHMARKS
	{
		if strings.has_prefix(id, "benchmark/") && id[len("benchmark/"):] == definition.package_name
		{
			package_name = definition.package_name;
			break;
		}
	}
	if len(package_name) == 0
	{
		return .Unsupported;
	}
	runs: Run_List;
	defer results_paths_close(&runs);
	status = results_scan_case(&runs, package_name, v.budget-min(v.budget, viewer_content_bytes(v)));
	if status != .Ok
	{
		return status;
	}
	for entry in runs.entries
	{
		dataset: report.Result_Dataset;
		result: report.Report_Result = report.result_dataset_open(&dataset, entry.path, package_name, v.budget-min(v.budget, viewer_content_bytes(v)+runs.owned_bytes));
		if result.status == .Budget_Exceeded || result.status == .Allocation_Failed
		{
			report.result_dataset_close(&dataset);
			return .Budget_Exceeded if result.status == .Budget_Exceeded else .Out_Of_Memory;
		}
		if result.status == .Ok
		{
			for &lane in dataset.lanes
			{
				for sample in 0 ..< len(lane.records)-1
				{
					filename: string = report.result_sample_recording(&lane, sample, lane.measurement);
					if recording_inspect(entry.path, filename).state == .Present
					{
						number_buffer: [64]u8;
						numbers: string = fmt.bprintf(number_buffer[:], "%d workers | Sample %d", lane.summary.worker_count, sample+1);
						capacity: int = cap(list.entries);
						if len(list.entries) == capacity
						{
							capacity = max(8, 2*capacity);
						}
						label_length: int = len(entry.date)+len(entry.title)+len(numbers)+11;
						bytes: u64 = u64(capacity-cap(list.entries))*size_of(Recording_Entry)+size_of(cstring)+
							u64(label_length+len(entry.path)+len(lane.path)+len(package_name)+len(lane.measurement));
						if bytes > v.budget-min(v.budget, viewer_content_bytes(v)+runs.owned_bytes+report.result_dataset_bytes(&dataset))
						{
							report.result_dataset_close(&dataset);
							return .Budget_Exceeded;
						}
						label_storage: []u8;
						allocation_error: runtime.Allocator_Error;
						label_storage, allocation_error = make([]u8, label_length);
						if allocation_error != nil
						{
							report.result_dataset_close(&dataset);
							return .Out_Of_Memory;
						}
						if reserve(&list.entries, capacity) != nil
						{
							delete(label_storage);
							report.result_dataset_close(&dataset);
							return .Out_Of_Memory;
						}
						fmt.bprintf(label_storage[:label_length-1], "%s UTC | %s | %s", entry.date, entry.title, numbers);
						label: cstring = cstring(raw_data(label_storage));
						locator: Result_Sample = {strings.clone(entry.path), strings.clone(lane.path), strings.clone(package_name), strings.clone(lane.measurement), sample};
						append(&list.entries, Recording_Entry{locator, label});
						list.owned_bytes += bytes;
					}
				}
			}
		}
		report.result_dataset_close(&dataset);
	}
	error: runtime.Allocator_Error;
	list.names, error = make([]cstring, len(list.entries));
	if error != nil
	{
		return .Out_Of_Memory;
	}
	for entry, index in list.entries
	{
		list.names[index] = entry.label;
	}
	return .Ok;
}

ui_recordings :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	list: ^Recording_List = &v.ui.recordings;
	d: f32 = v.ui.dpi;
	if len(list.case_id) == 0
	{
		ui_text(bounds, "Choose a benchmark recording");
		return;
	}
	ui_text({bounds.x, bounds.y, bounds.width-100*d, 28*d}, "%s | saved recordings", list.case_id);
	if rl.GuiButton({bounds.x+bounds.width-96*d, bounds.y, 96*d, 28*d}, "Refresh")
	{
		v.ui.status = recordings_refresh(v, list.case_id);
	}
	if len(list.entries) == 0
	{
		ui_text({bounds.x, bounds.y+48*d, bounds.width, 30*d}, "No saved recordings for this case");
		return;
	}
	rl.GuiListViewEx({bounds.x, bounds.y+40*d, bounds.width, max(36*d, bounds.height-84*d)}, raw_data(list.names), i32(len(list.names)),
		&list.scroll, &list.selected, &list.focus);
	if list.selected >= 0
	{
		v.ui.status = result_open_recording(v, list.entries[list.selected].sample, v.ui.selection_action);
		list.selected = -1;
		if v.ui.status == .Ok
		{
			v.ui.selection_action = .None;
		}
	}
}

ui_recording_dialog :: proc(v: ^Viewer)
{
	if v.ui.selection_action == .None
	{
		return;
	}
	d: f32 = v.ui.dpi;
	width: f32 = min(900*d, f32(rl.GetScreenWidth())-24*d);
	x: f32 = (f32(rl.GetScreenWidth())-width)/2;
	bounds: rl.Rectangle = {x, 50*d, width, f32(rl.GetScreenHeight())-100*d};
	rl.DrawRectangleRec(bounds, BACKGROUND);
	ui_recordings(v, {x+12*d, 62*d, width-24*d, bounds.height-54*d});
	if rl.GuiButton({x+12*d, bounds.y+bounds.height-36*d, 100*d, 28*d}, "Cancel") || rl.IsKeyPressed(.ESCAPE)
	{
		v.ui.selection_action = .None;
	}
	if v.ui.status != .Ok
	{
		ui_text({x+124*d, bounds.y+bounds.height-36*d, width-136*d, 28*d}, "%v", v.ui.status);
	}
}
