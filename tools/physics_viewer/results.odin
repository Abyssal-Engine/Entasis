package physics_viewer

import "base:runtime"
import "core:strings"
import "core:fmt"
import "core:os"
import "core:time"
import support "../../benchmarks/benchmark_support"
import report "../benchmark_report"
import replay "../physics_replay"
import scene "../physics_scene"
import rl "vendor:raylib"

Result_Sample :: struct
{
	run, lane, package_name, component: string,
	index: int,
}

open_results :: proc(v: ^Viewer, path, package_name: string) -> scene.Status
{
	v.cursor.playback, v.cursor.accumulator = .Paused, 0;
	held: u64 = viewer_content_bytes(v);
	if held > v.budget
	{
		return viewer_budget_failure(v, held, v.budget);
	}
	candidate: report.Result_Dataset;
	result: report.Report_Result = report.result_dataset_open(&candidate, path, package_name, v.budget-held);
	v.ui.results = .Available;
	v.ui.result_status = result.status;
	v.ui.result_diagnostic = {};
	if result.status != .Ok
	{
		if result.status == .Budget_Exceeded
		{
			fmt.bprintf(v.ui.result_diagnostic[:511], "Required %d bytes | available %d", result.required_bytes, result.available_bytes);
		}
		else
		{
			copy(v.ui.result_diagnostic[:511], result.diagnostic);
		}
		report.result_dataset_close(&candidate);
		if result.status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, result.required_bytes, result.available_bytes);
		}
		return .Invalid_Data;
	}
	labels: Result_Labels;
	status: scene.Status = result_labels_open(&labels, &candidate, v.budget-min(v.budget, held+report.result_dataset_bytes(&candidate)));
	if status != .Ok
	{
		report.result_dataset_close(&candidate);
		v.ui.result_status = .Budget_Exceeded if status == .Budget_Exceeded else .Allocation_Failed;
		fmt.bprintf(v.ui.result_diagnostic[:511], "Cannot prepare result labels: %v", status);
		return status;
	}
	v.ui.dropdown, v.ui.choice = 0, {};
	result_labels_close(&v.ui.result_labels);
	v.ui.result_labels = labels;
	report.result_dataset_close(&v.results);
	v.results = candidate;
	viewer_replay_budget(v);
	v.ui.result_lane, v.ui.result_sample = 0, 0;
	for column, index in v.results.lanes[0].columns
	{
		if column.name == v.results.lanes[0].measurement
		{
			v.ui.result_column = i32(index);
			break;
		}
	}
	v.ui.recording_message = {};
	recording_refresh_current(v);
	return v.ui.status;
}

result_unit_label :: proc(unit: report.Result_Unit) -> string
{
	switch unit
	{
	case .Milliseconds: return "ms";
	case .Nanoseconds: return "ns";
	case .Per_Second: return "/s";
	case .Text: return "";
	}
	unreachable();
}

ui_results :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	d: f32 = v.ui.dpi;
	x, y: f32 = bounds.x, bounds.y;
	// reserve room for both optional summaries and the deletion outcome
	gap: f32 = 4*d if f32(rl.GetScreenHeight()) < bounds.y+488*d else 12*d;
	if rl.GuiButton({x, y, 112*d, 28*d}, "< Saved runs")
	{
		v.ui.results = .Unavailable;
	}
	if v.results.state != .Ready || v.ui.result_status != .Ok
	{
		ui_text({x, y+40*d, bounds.width, 30*d}, "%v: %s", v.ui.result_status, strings.string_from_null_terminated_ptr(raw_data(v.ui.result_diagnostic[:]), 512));
		return;
	}
	y += 30*d+gap;
	ui_heading(&v.ui, {x, y, bounds.width, 32*d}, benchmark_label(result_package(v)));
	y += 36*d;
	for entry in v.ui.runs.entries
	{
		if entry.path == v.results.path
		{
			ui_text({x, y, bounds.width, 28*d}, "%s UTC  |  %s  |  %s", entry.date, entry.title, entry.configuration);
			break;
		}
	}
	y += 26*d+gap;
	labels: ^Result_Labels = &v.ui.result_labels;
	ui_choice(&v.ui, 10, {x, y, 156*d, 28*d}, labels.lanes, &v.ui.result_lane);
	lane: ^report.Result_Lane = &v.results.lanes[v.ui.result_lane];
	v.ui.result_sample = clamp(v.ui.result_sample, 0, i32(len(lane.records)-2));
	ui_choice(&v.ui, 12, {x+168*d, y, 168*d, 28*d}, labels.samples, &v.ui.result_sample);
	y += 30*d+gap;
	selected: i32;
	for column, index in labels.columns
	{
		if i32(column) == v.ui.result_column
		{
			selected = i32(index);
		}
	}
	if len(labels.columns) == 0
	{
		ui_text({x, y, bounds.width, 28*d}, "No timing measurements in this run");
		return;
	}
	ui_text({x, y, 116*d, 28*d}, "Measurement");
	ui_choice(&v.ui, 11, {x+120*d, y, min(360*d, bounds.width-120*d), 28*d}, labels.metrics, &selected);
	v.ui.result_column = i32(labels.columns[selected]);
	column: report.Result_Column = lane.columns[v.ui.result_column];
	component: string = column.name;
	y += 32*d+gap;
	width: f32 = min(bounds.width, 720*d);
	card: rl.Rectangle = {x, y, width, 100*d};
	rl.DrawRectangleRec(card, {39, 43, 49, 255});
	value: report.Result_Value = lane.values[int(v.ui.result_sample)*len(lane.columns)+int(v.ui.result_column)];
	ui_text({x+16*d, y+10*d, width-32*d, 24*d}, "%s", string(labels.metrics[selected]));
	if value.availability == .Available
	{
		ui_heading(&v.ui, {x+16*d, y+40*d, width-32*d, 42*d}, "%.3f %s", value.value, result_unit_label(column.unit));
	}
	else
	{
		ui_heading(&v.ui, {x+16*d, y+40*d, width-32*d, 42*d}, "Unavailable");
	}
	if component == "physics_elapsed_ms" && lane.summary.measured_steps > 0 && value.availability == .Available
	{
		ui_text({x+width*0.55, y+10*d, width*0.45-16*d, 24*d}, "Mean per step");
		ui_heading(&v.ui, {x+width*0.55, y+40*d, width*0.45-16*d, 42*d}, "%.3f ms", value.value/f64(lane.summary.measured_steps));
	}
	y += 100*d+gap;
	ui_cell(&v.ui, {x, y, bounds.width, 28*d}, "Recording: %s | Timing: %v", "Unavailable" if lane.summary.recording_mode == .Legacy else ("On" if lane.summary.recording_mode == .On else "Off"), lane.summary.timing_method);
	y += 32*d;
	if lane.summary.measured_steps > 0
	{
		ui_text({x, y, bounds.width, 28*d}, "Measured across %d steps. Mean = total time / measured steps", lane.summary.measured_steps);
		y += 32*d;
	}
	if column.summary.sample_count > 1
	{
		ui_text({x, y, bounds.width, 28*d}, "Across %d samples: median %.3f %s  |  min %.3f  |  max %.3f",
			column.summary.sample_count, column.summary.median, result_unit_label(column.unit), column.summary.minimum, column.summary.maximum);
		y += 32*d;
	}
	filename: string = report.result_sample_recording(lane, int(v.ui.result_sample), component);
	index: int = recording_current_index(v);
	file_state: Recording_State = v.ui.recording_inventory.entries[index].state if index < len(v.ui.recording_inventory.entries) else .Failed;
	if len(filename) > 0 && file_state == .Present
	{
		if rl.GuiButton({x, y, 148*d, 32*d}, "Play recording")
		{
			v.ui.status = result_open_recording(v, {v.results.path, lane.path, result_package(v), component, int(v.ui.result_sample)});
		}
	}
	else
	{
		label: string = "Replay unavailable";
		if len(filename) > 0
		{
			label = "Recording missing" if file_state == .Missing else "Recording unavailable";
		}
		else if lane.summary.recording_mode == .Off
		{
			label = "Recording off" if support.recording_supported(result_package(v)) == .Ok else "Replay unsupported";
		}
		else if lane.summary.recording_mode == .On
		{
			label = "Not recorded";
		}
		ui_cell(&v.ui, {x, y, 148*d, 32*d}, label);
	}
	if rl.GuiButton({x+160*d, y, 112*d, 32*d}, "Run again")
	{
		result_run_again(v, lane);
	}
	if rl.GuiButton({x+284*d, y, 112*d, 32*d}, "Open report")
	{
		viewer_open_file(v.results.path, "README.md");
	}
	if v.ui.recording_inventory.present == 0
	{
		rl.GuiDisable();
	}
	if rl.GuiButton({x+408*d, y, 180*d, 32*d}, "Delete recordings...")
	{
		v.ui.recording_message = {};
		recording_delete_preview(v, .Sample if len(filename) > 0 && file_state == .Present else .Run);
	}
	rl.GuiEnable();
	ui_cell(&v.ui, {x, y+38*d, bounds.width, 28*d}, strings.string_from_null_terminated_ptr(raw_data(v.ui.recording_message[:]), 512));
}

Run_Entry :: struct
{
	path, date, title, configuration: string,
	modified: i64,
	recording: Recording_State,
}
Run_List :: struct
{
	entries: [dynamic]Run_Entry,
	owned_bytes: u64,
}

results_paths_close :: proc(runs: ^Run_List)
{
	for entry in runs.entries
	{
		delete(entry.path);
		delete(entry.date);
		delete(entry.title);
		delete(entry.configuration);
	}
	delete(runs.entries);
	runs^ = {};
}

Result_Discovery_Kind :: enum
{
	Runs, Cases,
}
Result_Discovery :: struct
{
	run: Run_Entry,
	files: [dynamic]os.File_Info,
	data: []u8,
	owned_bytes: u64,
	error: os.Error,
	status: scene.Status,
	diagnostic: string,
}

results_discover :: proc(root: string, depth: int, budget: u64, kind: Result_Discovery_Kind = .Runs) -> Result_Discovery
{
	if strings.has_suffix(root, ".pending") || strings.has_suffix(root, ".failed")
	{
		return {};
	}
	directory: Recording_File = recording_directory_inspect(root);
	if directory.state == .Missing
	{
		return {};
	}
	if directory.state != .Present
	{
		return {status=.File_Error, error=directory.error, diagnostic="Linked or invalid directory"};
	}
	folder: ^os.File;
	error: os.Error;
	folder, error = os.open(root);
	if error != nil
	{
		return {status=.File_Error, error=error, diagnostic="Cannot read saved report"};
	}
	defer os.close(folder);
	iterator: os.Read_Directory_Iterator = os.read_directory_iterator_create(folder);
	defer os.read_directory_iterator_destroy(&iterator);
	result: Result_Discovery;
	for file in os.read_directory_iterator(&iterator)
	{
		_, error = os.read_directory_iterator_error(&iterator);
		if error != nil
		{
			break;
		}
		capacity: int = cap(result.files);
		growth: u64;
		if len(result.files) == capacity
		{
			if capacity > max(int)/size_of(os.File_Info)/2
			{
				result.status, result.diagnostic = .Budget_Exceeded, "Saved directory exceeds remaining memory budget";
				return result;
			}
			capacity = max(8, capacity*2);
			// admit the full replacement while the old array can still be live
			growth = u64(capacity)*size_of(os.File_Info);
		}
		path_bytes: u64 = u64(len(file.fullpath));
		if growth+path_bytes > budget-result.owned_bytes
		{
			result.status, result.diagnostic = .Budget_Exceeded, "Saved directory exceeds remaining memory budget";
			return result;
		}
		old_capacity: int = cap(result.files);
		allocation_error: runtime.Allocator_Error = reserve(&result.files, capacity);
		if allocation_error != nil
		{
			result.status, result.error, result.diagnostic = .File_Error, allocation_error, "Cannot read saved report";
			return result;
		}
		result.owned_bytes += u64(cap(result.files)-old_capacity)*size_of(os.File_Info);
		owned: os.File_Info;
		owned, allocation_error = os.file_info_clone(file, context.allocator);
		if allocation_error != nil
		{
			result.status, result.error, result.diagnostic = .File_Error, allocation_error, "Cannot read saved report";
			return result;
		}
		append(&result.files, owned);
		result.owned_bytes += path_bytes;
	}
	_, error = os.read_directory_iterator_error(&iterator);
	if error != nil
	{
		result.status, result.error, result.diagnostic = .File_Error, error, "Cannot read saved report";
		return result;
	}
	if kind == .Runs
	{
		for file in result.files
		{
			if file.type != .Regular || file.name != "README.md"
			{
				continue;
			}
			if file.size < 0 || u64(file.size) > budget-result.owned_bytes
			{
				result.status, result.diagnostic = .Budget_Exceeded, "Saved report exceeds remaining memory budget";
				return result;
			}
			data: []u8;
			data, error = os.read_entire_file(file.fullpath, context.allocator);
			result.data = data;
			result.owned_bytes += u64(len(data));
			if error != nil
			{
				result.status, result.error, result.diagnostic = .File_Error, error, "Cannot read saved report";
				return result;
			}
			metadata: string = string(data);
			date: string = run_report_field(metadata, "- Run date (UTC): `");
			leaf: string = os.base(root);
			if len(leaf) >= 16 && leaf[8] == 'T' && strings.contains(leaf, "Z.")
			{
				date = leaf;
			}
			variant: string = run_report_field(metadata, "Variant: `");
			configuration: string = run_report_field(metadata, "- Configuration: `");
			result.run = {path=root, modified=time.to_unix_seconds(file.modification_time),
				date=date if len(date) > 0 else "Date unavailable",
				title="Interactive run" if variant == "viewer" else (variant if len(variant) > 0 else "Saved run"),
				configuration=configuration if len(configuration) > 0 else "Unknown"};
			return result;
		}
	}
	return result;
}

results_discovery_close :: proc(discovery: ^Result_Discovery)
{
	for file in discovery.files
	{
		os.file_info_delete(file, context.allocator);
	}
	delete(discovery.files);
	delete(discovery.data);
	discovery^ = {};
}

results_scan :: proc(runs: ^Run_List, root: string, depth: int, budget: u64) -> (status: scene.Status)
{
	defer if status != .Ok
	{
		results_paths_close(runs);
	}
	discovery: Result_Discovery = results_discover(root, depth, budget-min(budget, runs.owned_bytes));
	defer results_discovery_close(&discovery);
	if discovery.status != .Ok
	{
		return discovery.status;
	}
	if len(discovery.run.path) > 0
	{
		date: string = discovery.run.date;
		leaf: string = os.base(discovery.run.path);
		date_buffer: [19]u8;
		if len(leaf) >= 16 && leaf[8] == 'T' && strings.contains(leaf, "Z.")
		{
			date = fmt.bprintf(date_buffer[:], "%s-%s-%s %s:%s:%s", leaf[:4], leaf[4:6], leaf[6:8], leaf[9:11], leaf[11:13], leaf[13:15]);
		}
		capacity: int = cap(runs.entries);
		growth: u64;
		if len(runs.entries) == capacity
		{
			if capacity > max(int)/2
			{
				return .Budget_Exceeded;
			}
			capacity = max(8, capacity*2);
			// the replacement and old array may coexist during resize
			growth = u64(capacity)*size_of(Run_Entry);
		}
		text_bytes: u64 = u64(len(discovery.run.path)+len(date)+len(discovery.run.title)+len(discovery.run.configuration));
		if growth+text_bytes > budget-min(budget, runs.owned_bytes+discovery.owned_bytes)
		{
			return .Budget_Exceeded;
		}
		old_capacity: int = cap(runs.entries);
		if reserve(&runs.entries, capacity) != nil
		{
			return .Out_Of_Memory;
		}
		runs.owned_bytes += u64(cap(runs.entries)-old_capacity)*size_of(Run_Entry);
		entry: Run_Entry = {modified=discovery.run.modified};
		defer
		{
			delete(entry.path);
			delete(entry.date);
			delete(entry.title);
			delete(entry.configuration);
		}
		error: runtime.Allocator_Error;
		entry.path, error = strings.clone(discovery.run.path);
		if error != nil
		{
			return .Out_Of_Memory;
		}
		entry.date, error = strings.clone(date);
		if error != nil
		{
			return .Out_Of_Memory;
		}
		entry.title, error = strings.clone(discovery.run.title);
		if error != nil
		{
			return .Out_Of_Memory;
		}
		entry.configuration, error = strings.clone(discovery.run.configuration);
		if error != nil
		{
			return .Out_Of_Memory;
		}
		ui_name(entry.title, transmute([]u8)entry.title);
		ui_name(entry.configuration, transmute([]u8)entry.configuration);
		append(&runs.entries, entry);
		runs.owned_bytes += text_bytes;
		entry = {};
	}
	for file in discovery.files
	{
		if len(discovery.run.path) == 0 && depth > 0 && (file.type == .Directory || file.type == .Symlink)
		{
			status = results_scan(runs, file.fullpath, depth-1, budget-discovery.owned_bytes);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

run_report_field :: proc(text, prefix: string) -> string
{
	start: int = strings.index(text, prefix);
	if start < 0
	{
		return "";
	}
	value: string = text[start+len(prefix):];
	end: int = strings.index(value, "`");
	return value[:end] if end >= 0 else "";
}

results_scan_case :: proc(runs: ^Run_List, package_name: string, budget: u64) -> (status: scene.Status)
{
	results_paths_close(runs);
	defer if status != .Ok
	{
		results_paths_close(runs);
	}
	host: string = "windows_amd64" when ODIN_OS == .Windows else "linux_amd64";
	// package names come from the fixed benchmark catalog
	buffer: [128]u8;
	status = results_scan(runs, fmt.bprintf(buffer[:], "build/benchmark-results/%s/%s", host, package_name), 3, budget);
	if status == .Ok
	{
		status = results_scan(runs, fmt.bprintf(buffer[:], "results/%s", package_name), 2, budget);
	}
	if status != .Ok
	{
		return status;
	}
	for index in 1 ..< len(runs.entries)
	{
		for cursor: int = index; cursor > 0 && runs.entries[cursor].modified > runs.entries[cursor-1].modified; cursor -= 1
		{
			runs.entries[cursor], runs.entries[cursor-1] = runs.entries[cursor-1], runs.entries[cursor];
		}
	}
	for &entry in runs.entries
	{
		entry.recording, status = recording_presence(entry.path, package_name, budget-runs.owned_bytes);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

results_filter :: proc(v: ^Viewer)
{
	ui: ^UI = &v.ui;
	ui.run_pressed = 0;
	delete(ui.visible_runs);
	ui.visible_runs = nil;
	ui.run_focus, ui.run_scroll = -1, 0;
	count: int;
	for entry in ui.runs.entries
	{
		if ui.recording_filter == .All || (ui.recording_filter == .With && entry.recording == .Present) ||
			(ui.recording_filter == .Without && entry.recording == .Missing)
		{
			count += 1;
		}
	}
	if u64(count)*size_of(int) > v.budget-min(v.budget, viewer_content_bytes(v))
	{
		ui.status = .Budget_Exceeded;
		ui.run_selected = -1;
		return;
	}
	allocation_error: runtime.Allocator_Error;
	ui.visible_runs, allocation_error = make([]int, count);
	if allocation_error != nil
	{
		ui.status, ui.run_selected = .Out_Of_Memory, -1;
		return;
	}
	cursor: int;
	selected: i32 = -1;
	for entry, index in ui.runs.entries
	{
		if ui.recording_filter == .All || (ui.recording_filter == .With && entry.recording == .Present) ||
			(ui.recording_filter == .Without && entry.recording == .Missing)
		{
			ui.visible_runs[cursor] = index;
			if i32(index) == ui.run_selected
			{
				selected = i32(cursor);
			}
			cursor += 1;
		}
	}
	ui.run_focus = selected;
	if selected < 0
	{
		ui.run_selected = -1;
	}
	ui.run_scroll = 0;
}

results_refresh :: proc(v: ^Viewer)
{
	selected: string;
	selected_buffer: File_Path_Buffer;
	if v.ui.run_selected >= 0 && int(v.ui.run_selected) < len(v.ui.runs.entries)
	{
		path: string = v.ui.runs.entries[v.ui.run_selected].path;
		copy(selected_buffer[:], path);
		selected = string(selected_buffer[:len(path)]);
	}
	results_paths_close(&v.ui.runs);
	delete(v.ui.visible_runs);
	v.ui.visible_runs = nil;
	v.ui.run_selected, v.ui.run_focus, v.ui.run_scroll, v.ui.run_pressed = -1, -1, 0, 0;
	v.ui.status = results_scan_case(&v.ui.runs, benchmark_package(v.launch.package_index), v.budget-min(v.budget, viewer_content_bytes(v)));
	if v.ui.status != .Ok
	{
		return;
	}
	for entry, index in v.ui.runs.entries
	{
		if entry.path == selected
		{
			v.ui.run_selected = i32(index);
		}
	}
	results_filter(v);
}

ui_saved_runs :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	d: f32 = v.ui.dpi;
	if v.ui.category != 0
	{
		v.ui.run_pressed = 0;
		ui_text(bounds, "Select an example to open it");
		return;
	}
	ui_heading(&v.ui, {bounds.x, bounds.y, bounds.width-104*d, 32*d}, benchmark_label(benchmark_package(v.launch.package_index)));
	if rl.GuiButton({bounds.x+bounds.width-96*d, bounds.y, 96*d, 28*d}, "Refresh")
	{
		results_refresh(v);
	}
	filter: i32 = i32(v.ui.recording_filter);
	rl.GuiToggleGroup({bounds.x, bounds.y+38*d, min(180*d, (bounds.width-12*d)/3), 28*d}, "All;With recordings;Without recordings", &filter);
	if filter != i32(v.ui.recording_filter)
	{
		v.ui.recording_filter = Recording_Filter(filter);
		results_filter(v);
	}
	ui_text({bounds.x, bounds.y+72*d, bounds.width, 26*d}, "Saved runs  |  %d of %d shown", len(v.ui.visible_runs), len(v.ui.runs.entries));
	ui_text({bounds.x, bounds.y+bounds.height-30*d, bounds.width, 28*d}, "%s",
		strings.string_from_null_terminated_ptr(raw_data(v.launch.diagnostic[:]), 256));
	if len(v.ui.visible_runs) == 0
	{
		v.ui.run_pressed = 0;
		ui_text({bounds.x, bounds.y+114*d, bounds.width, 30*d}, "No saved runs yet. Choose Run new to start" if len(v.ui.runs.entries) == 0 else "No runs match this filter. Select All to show every run");
		return;
	}
	row_height: f32 = 44*d;
	top: f32 = bounds.y+158*d;
	rows: i32 = max(1, i32((bounds.height-198*d)/row_height));
	maximum_scroll: i32 = max(0, i32(len(v.ui.visible_runs))-rows);
	list_bounds: rl.Rectangle = {bounds.x, top, bounds.width, f32(rows)*row_height};
	active: scene.Availability = .Available if v.ui.input == .List && !rl.GuiIsLocked() && rl.IsWindowFocused() else .Unavailable;
	if active == .Unavailable
	{
		v.ui.run_pressed = 0;
	}
	open_index: i32 = -1;
	if active == .Available && rl.CheckCollisionPointRec(rl.GetMousePosition(), list_bounds)
	{
		v.ui.run_scroll = clamp(v.ui.run_scroll-i32(rl.GetMouseWheelMove()*3), 0, maximum_scroll);
	}
	if active == .Available && v.ui.run_focus >= 0
	{
		if rl.IsKeyPressed(.DOWN) || rl.IsKeyPressedRepeat(.DOWN)
		{
			v.ui.run_focus = min(i32(len(v.ui.visible_runs)-1), v.ui.run_focus+1);
		}
		if rl.IsKeyPressed(.UP) || rl.IsKeyPressedRepeat(.UP)
		{
			v.ui.run_focus = max(0, v.ui.run_focus-1);
		}
		if rl.IsKeyPressed(.ENTER)
		{
			open_index = v.ui.run_focus;
		}
		if rl.IsKeyPressed(.DOWN) || rl.IsKeyPressed(.UP)
		{
			v.ui.run_scroll = clamp(v.ui.run_scroll, max(0, v.ui.run_focus-rows+1), v.ui.run_focus);
		}
	}
	date_width: f32 = 200*d;
	build_width: f32 = 128*d;
	run_x: f32 = bounds.x+date_width;
	build_x: f32 = bounds.x+bounds.width-build_width-24*d;
	rl.DrawRectangleRec({bounds.x, top-36*d, bounds.width, 32*d}, {43, 47, 54, 255});
	ui_text({bounds.x+12*d, top-36*d, date_width-12*d, 32*d}, "Date (UTC)");
	ui_text({run_x+12*d, top-36*d, build_x-run_x-24*d, 32*d}, "Run");
	ui_text({build_x+12*d, top-36*d, build_width-12*d, 32*d}, "Build");
	v.ui.run_scroll = clamp(v.ui.run_scroll, 0, maximum_scroll);
	for row in 0 ..< rows
	{
		index: i32 = v.ui.run_scroll+row;
		if int(index) >= len(v.ui.visible_runs)
		{
			break;
		}
		entry: Run_Entry = v.ui.runs.entries[v.ui.visible_runs[index]];
		area: rl.Rectangle = {bounds.x, top+f32(row)*row_height, bounds.width-24*d, row_height-2*d};
		color: rl.Color = {36, 39, 44, 255} if row%2 == 0 else {31, 34, 39, 255};
		if i32(v.ui.visible_runs[index]) == v.ui.run_selected || index == v.ui.run_focus
		{
			color = {51, 69, 87, 255};
		}
		if active == .Available && rl.CheckCollisionPointRec(rl.GetMousePosition(), area)
		{
			color = {64, 78, 95, 255};
			if rl.IsMouseButtonPressed(.LEFT)
			{
				v.ui.run_pressed = v.ui.visible_runs[index]+1;
			}
			if rl.IsMouseButtonReleased(.LEFT) && v.ui.run_pressed == v.ui.visible_runs[index]+1
			{
				open_index, v.ui.run_focus = index, index;
			}
		}
		rl.DrawRectangleRec(area, color);
		ui_cell(&v.ui, {bounds.x+12*d, area.y, date_width-24*d, area.height}, entry.date);
		ui_cell(&v.ui, {run_x+12*d, area.y, build_x-run_x-24*d, area.height}, "%s%s", entry.title, " | recording status unavailable" if entry.recording == .Failed else "");
		ui_cell(&v.ui, {build_x+12*d, area.y, build_width-24*d, area.height}, entry.configuration);
	}
	if maximum_scroll > 0
	{
		track: rl.Rectangle = {bounds.x+bounds.width-18*d, top, 16*d, f32(rows)*row_height};
		thumb_height: f32 = max(24*d, track.height*f32(rows)/f32(len(v.ui.visible_runs)));
		if active == .Available && rl.IsMouseButtonDown(.LEFT) && rl.CheckCollisionPointRec(rl.GetMousePosition(), track)
		{
			fraction: f32 = clamp((rl.GetMousePosition().y-track.y-thumb_height/2)/(track.height-thumb_height), 0, 1);
			v.ui.run_scroll = i32(fraction*f32(maximum_scroll)+0.5);
		}
		rl.DrawRectangleRec(track, {43, 47, 54, 255});
		rl.DrawRectangleRec({track.x, track.y+(track.height-thumb_height)*f32(v.ui.run_scroll)/f32(maximum_scroll), track.width, thumb_height}, {91, 105, 123, 255});
	}
	if active == .Available && v.ui.run_focus < 0 && (rl.IsKeyPressed(.DOWN) || rl.IsKeyPressed(.TAB))
	{
		v.ui.run_focus = v.ui.run_scroll;
	}
	if rl.IsMouseButtonReleased(.LEFT) || open_index >= 0
	{
		v.ui.run_pressed = 0;
	}
	if open_index >= 0
	{
		v.ui.run_selected = i32(v.ui.visible_runs[open_index]);
		v.ui.status = open_results(v, v.ui.runs.entries[v.ui.run_selected].path, benchmark_package(v.launch.package_index));
	}
}

result_open_recording :: proc(v: ^Viewer, sample: Result_Sample, action: Selection_Action = .None) -> scene.Status
{
	available: u64 = v.budget-min(v.budget, viewer_content_bytes(v));
	dataset: report.Result_Dataset;
	defer report.result_dataset_close(&dataset);
	result: report.Report_Result = report.result_dataset_open(&dataset, sample.run, sample.package_name, available);
	if result.status != .Ok
	{
		return viewer_budget_failure(v, result.required_bytes, available) if result.status == .Budget_Exceeded else .Invalid_Data;
	}
	lane: ^report.Result_Lane;
	for &candidate in dataset.lanes
	{
		if candidate.path == sample.lane
		{
			lane = &candidate;
			break;
		}
	}
	if lane == nil || sample.index < 0 || sample.index >= len(lane.records)-1
	{
		return .Invalid_Data;
	}
	filename: string = report.result_sample_recording(lane, sample.index, sample.component);
	if len(filename) == 0
	{
		return .Unsupported;
	}
	path_buffer: File_Path_Buffer;
	path: string;
	path_status: scene.Status;
	path, path_status = recording_join_path(path_buffer[:], sample.run, filename);
	if path_status != .Ok
	{
		return path_status;
	}
	if recording_inspect(sample.run, filename).state != .Present
	{
		return .File_Error;
	}
	reader: replay.Reader;
	defer replay.reader_close(&reader);
	status: scene.Status = replay.reader_open(&reader, path, available-report.result_dataset_bytes(&dataset));
	if status != .Ok
	{
		return status;
	}
	metadata: scene.Metadata = reader.metadata;
	worker: int;
	settings: string = metadata.settings;
	for field in strings.split_by_byte_iterator(&settings, ';')
	{
		key, value: string;
		key, value, _ = support.split_once(field, '=');
		if key == "workers"
		{
			worker, _ = support.parse_integer(value);
		}
	}
	if !strings.has_prefix(metadata.scenario, "benchmark/") || metadata.scenario[len("benchmark/"):] != sample.package_name ||
		metadata.completion != .Completed || worker != lane.summary.worker_count
	{
		return .Invalid_Data;
	}
	steps: int = lane.summary.measured_steps;
	if steps == 0
	{
		settings = metadata.settings;
		for field in strings.split_by_byte_iterator(&settings, ';')
		{
			key, value: string;
			key, value, _ = support.split_once(field, '=');
			if key == "steps"
			{
				steps, _ = support.parse_integer(value);
			}
		}
	}
	if steps <= 0 || len(reader.index) != steps+1 || reader.index[0].tick != 0 || reader.index[len(reader.index)-1].tick != u64(steps)
	{
		return .Invalid_Data;
	}
	parameters: string = lane.summary.benchmark_parameters;
	if len(parameters) > 0 && (!strings.has_prefix(metadata.settings, parameters) || len(metadata.settings) <= len(parameters) || metadata.settings[len(parameters)] != ';')
	{
		return .Invalid_Data;
	}
	replay.reader_close(&reader);
	report.result_dataset_close(&dataset);
	return open_comparison(v, path) if action == .Compare else open_replays(v, path, "");
}

result_run_again :: proc(v: ^Viewer, lane: ^report.Result_Lane)
{
	package_name: string = result_package(v);
	found: scene.Availability;
	for definition, index in BENCHMARKS
	{
		if definition.package_name == package_name
		{
			v.launch.package_index = i32(index);
			found = .Available;
			break;
		}
	}
	if found != .Available
	{
		v.ui.status = .Unsupported;
		return;
	}
	v.launch.workers, v.launch.repeats, v.launch.fields, v.launch.variant, v.launch.diagnostic = {}, {}, {}, {}, {};
	if lane.summary.worker_count >= 1 && lane.summary.worker_count <= min(64, launch_available_workers())
	{
		v.launch.workers = {lane.summary.worker_count};
	}
	fmt.bprintf(v.launch.repeats[:15], "%d", lane.summary.sample_count);
	launch_open(v);
	v.launch.fields = {};
	v.launch.record = .On if lane.summary.recording_mode == .On else .Off;
	if package_name == "container" || package_name == "contact_islands" || package_name == "pyramid"
	{
		workload: support.Workload = .Container if package_name == "container" else .Contact_Islands if package_name == "contact_islands" else .Pyramid;
		options: support.Options;
		status: support.Admission;
		options, status = support.options_from_saved_parameters(workload, lane.summary.benchmark_parameters, lane.summary.worker_count);
		if status != .Ok
		{
			v.launch.missing_settings = .Available;
			copy(v.launch.diagnostic[:255], "Incomplete saved settings: explicitly fill missing values from current defaults");
		}
		else
		{
			v.launch.shape = i32(options.shape);
		}
		fields: string = lane.summary.benchmark_parameters;
		for field in strings.split_by_byte_iterator(&fields, ';')
		{
			key, value: string;
			key, value, _ = support.split_once(field, '=');
			if key == "shape"
			{
				shape: support.Shape;
				admitted: support.Admission;
				shape, admitted = support.parse_shape(value);
				if admitted == .Ok
				{
					v.launch.shape = i32(shape);
				}
			}
			option_names: [len(support.OPTION_NAMES)]string = support.OPTION_NAMES;
			for name, index in option_names[:26]
			{
				if key == name && index != 0
				{
					launch_set_field(&v.launch, index, value);
				}
			}
		}
		if status == .Ok
		{
			copy(v.launch.diagnostic[:255], "Saved workload restored");
		}
	}
	for name, index in lane.records[0]
	{
		if name == "benchmark_components"
		{
			v.launch.components = 1 if lane.records[1][index] == "common" else 0;
		}
	}
	if worker_count(v.launch.workers) == 0
	{
		v.launch.diagnostic = {};
		fmt.bprintf(v.launch.diagnostic[:255], "Saved thread count %d is unavailable on this host. Select a valid count", lane.summary.worker_count);
	}
}

result_package :: proc(v: ^Viewer) -> string
{
	if len(v.results.package_name) > 0
	{
		return v.results.package_name;
	}
	if len(v.results.lanes) > 0
	{
		parameters: string = v.results.lanes[0].summary.benchmark_parameters;
		for name in ([]string{"container", "contact_islands", "pyramid"})
		{
			if strings.has_prefix(parameters, "workload=") && strings.has_prefix(parameters[len("workload="):], name) &&
				len(parameters) > len("workload=")+len(name) && parameters[len("workload=")+len(name)] == ';'
			{
				return name;
			}
		}
		for name, column in v.results.lanes[0].records[0]
		{
			if name == "benchmark"
			{
				return v.results.lanes[0].records[1][column];
			}
		}
	}
	return "";
}
