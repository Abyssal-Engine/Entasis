package physics_viewer

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:strings"
import rl "vendor:raylib"
import report "../benchmark_report"
import scene "../physics_scene"

Recording_Delete_Scope :: enum
{
	Sample, Run, Workspace,
}
Recording_Delete_Phase :: enum
{
	Closed, Scanning, Ready, Rechecking, Removing, Complete, Failed,
}
Recording_Delete_Outcome :: enum
{
	Pending, Deleted, Absent, Failed,
}
Recording_Delete_Directory :: struct
{
	path, package_name: string,
	depth: int,
}
Recording_Delete_Run :: struct
{
	path, package_name: string,
	inventory: Recording_Inventory,
}
Recording_Delete_Target :: struct
{
	run, entry: int,
	outcome: Recording_Delete_Outcome,
	diagnostic: [96]u8,
}
Recording_Delete :: struct
{
	phase: Recording_Delete_Phase,
	scope: Recording_Delete_Scope,
	opened: scene.Availability,
	directories: [dynamic]Recording_Delete_Directory,
	runs: [dynamic]Recording_Delete_Run,
	targets: []Recording_Delete_Target,
	exclusions: [dynamic]string,
	owned_bytes, bytes: u64,
	cursor, focus, affected_runs: int,
	detail_scroll: rl.Vector2,
	detail_width: f32,
	worker, sample: int,
	deleted, absent, failed: int,
	playback: scene.Playback,
	message: [512]u8,
}

recording_delete_close :: proc(p: ^Recording_Delete)
{
	for directory in p.directories
	{
		delete(directory.path);
		delete(directory.package_name);
	}
	for &run in p.runs
	{
		delete(run.path);
		delete(run.package_name);
		recording_inventory_close(&run.inventory);
	}
	for reason in p.exclusions
	{
		delete(reason);
	}
	delete(p.directories);
	delete(p.runs);
	delete(p.targets);
	delete(p.exclusions);
	p^ = {};
}

recording_delete_error :: proc(p: ^Recording_Delete, message: string, args: ..any)
{
	p.phase = .Failed;
	p.message = {};
	if len(args) == 0
	{
		copy(p.message[:511], message);
	}
	else
	{
		fmt.bprintf(p.message[:511], message, ..args);
	}
}

recording_delete_queue :: proc(v: ^Viewer, path, package_name: string, depth: int) -> scene.Status
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	capacity: int = cap(p.directories);
	if len(p.directories) == capacity
	{
		capacity = max(8, capacity*2);
	}
	bytes: u64 = u64(capacity-cap(p.directories))*size_of(Recording_Delete_Directory)+u64(len(path)+len(package_name));
	if bytes > v.budget-min(v.budget, viewer_content_bytes(v))
	{
		return .Budget_Exceeded;
	}
	if reserve(&p.directories, capacity) != nil
	{
		return .Out_Of_Memory;
	}
	append(&p.directories, Recording_Delete_Directory{strings.clone(path), strings.clone(package_name), depth});
	p.owned_bytes += bytes;
	return .Ok;
}

recording_delete_exclude :: proc(v: ^Viewer, path, reason: string) -> scene.Status
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	capacity: int = cap(p.exclusions);
	if len(p.exclusions) == capacity
	{
		capacity = max(4, capacity*2);
	}
	length: int = len(reason)+3+len(path);
	bytes: u64 = u64(capacity-cap(p.exclusions))*size_of(string)+u64(length);
	if bytes > v.budget-min(v.budget, viewer_content_bytes(v))
	{
		return .Budget_Exceeded;
	}
	storage: []u8;
	allocation_error: runtime.Allocator_Error;
	storage, allocation_error = make([]u8, length);
	if allocation_error != nil
	{
		return .Out_Of_Memory;
	}
	if reserve(&p.exclusions, capacity) != nil
	{
		delete(storage);
		return .Out_Of_Memory;
	}
	fmt.bprintf(storage, "%s | %s", reason, path);
	append(&p.exclusions, string(storage));
	p.owned_bytes += bytes;
	return .Ok;
}

recording_delete_preview :: proc(v: ^Viewer, scope: Recording_Delete_Scope)
{
	path_buffer: File_Path_Buffer;
	p: ^Recording_Delete = &v.ui.recording_delete;
	playback: scene.Playback = v.cursor.playback if p.phase == .Closed else p.playback;
	recording_delete_close(p);
	p.phase, p.scope, p.opened, p.playback = .Scanning, scope, .Available, playback;
	v.ui.text_field, v.ui.dropdown = 0, 0;
	v.ui.edit, v.ui.choice = {}, {};
	v.cursor.playback = .Paused;
	status: scene.Status;
	if scope == .Workspace
	{
		for root, index in ([3]string{"results", "build/benchmark-results/windows_amd64", "build/benchmark-results/linux_amd64"})
		{
			path: string;
			path, status = recording_absolute_path(root, path_buffer[:]);
			if status == .Ok
			{
				status = recording_delete_queue(v, path, "", 3 if index == 0 else 4);
			}
			if status != .Ok
			{
				break;
			}
		}
	}
	else
	{
		p.worker = v.results.lanes[v.ui.result_lane].summary.worker_count;
		p.sample = int(v.ui.result_sample);
		path: string;
		path, status = recording_absolute_path(v.results.path, path_buffer[:]);
		if status == .Ok
		{
			status = recording_delete_queue(v, path, result_package(v), 0);
		}
	}
	if status != .Ok
	{
		recording_delete_error(p, "Cannot prepare cleanup: %v", status);
	}
}

recording_delete_load_run :: proc(v: ^Viewer, directory: Recording_Delete_Directory) -> scene.Status
{
	path_buffer: File_Path_Buffer;
	p: ^Recording_Delete = &v.ui.recording_delete;
	for run in p.runs
	{
		if run.path == directory.path
		{
			return .Ok;
		}
	}
	dataset: report.Result_Dataset;
	defer report.result_dataset_close(&dataset);
	result: report.Report_Result = report.result_dataset_open(&dataset, directory.path, directory.package_name, v.budget-min(v.budget, viewer_content_bytes(v)));
	if result.status == .Budget_Exceeded
	{
		return .Budget_Exceeded;
	}
	if result.status != .Ok
	{
		return recording_delete_exclude(v, directory.path, result.diagnostic);
	}
	capacity: int = cap(p.runs);
	if len(p.runs) == capacity
	{
		capacity = max(8, capacity*2);
	}
	bytes: u64 = u64(capacity-cap(p.runs))*size_of(Recording_Delete_Run)+u64(len(directory.path)+len(directory.package_name));
	held: u64 = viewer_content_bytes(v)+report.result_dataset_bytes(&dataset);
	if bytes > v.budget-min(v.budget, held)
	{
		return .Budget_Exceeded;
	}
	run: Recording_Delete_Run;
	status: scene.Status = recording_inventory_load(&run.inventory, &dataset, v.budget-held-bytes);
	if status != .Ok
	{
		recording_inventory_close(&run.inventory);
		return status;
	}
	if reserve(&p.runs, capacity) != nil
	{
		recording_inventory_close(&run.inventory);
		return .Out_Of_Memory;
	}
	run.path, run.package_name = strings.clone(directory.path), strings.clone(directory.package_name);
	append(&p.runs, run);
	p.owned_bytes += bytes+run.inventory.owned_bytes;
	report.result_dataset_close(&dataset);
	for entry in run.inventory.entries
	{
		if entry.state == .Failed || entry.state == .Rejected
		{
			path: string;
			path, status = recording_join_path(path_buffer[:], run.path, entry.name);
			if status == .Ok
			{
				status = recording_delete_exclude(v, path, "Unavailable or unsafe recording");
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

recording_delete_targets :: proc(v: ^Viewer) -> scene.Status
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	retained: int;
	for run in p.runs
	{
		if run.inventory.present == 0
		{
			p.owned_bytes -= u64(len(run.path)+len(run.package_name))+run.inventory.owned_bytes;
			delete(run.path);
			delete(run.package_name);
			inventory: Recording_Inventory = run.inventory;
			recording_inventory_close(&inventory);
		}
		else
		{
			p.runs[retained] = run;
			retained += 1;
		}
	}
	resize(&p.runs, retained);
	for index in 1 ..< len(p.runs)
	{
		for cursor: int = index; cursor > 0 && p.runs[cursor].path < p.runs[cursor-1].path; cursor -= 1
		{
			p.runs[cursor], p.runs[cursor-1] = p.runs[cursor-1], p.runs[cursor];
		}
	}
	count: int;
	for run in p.runs
	{
		count += run.inventory.present;
	}
	bytes: u64 = u64(count)*size_of(Recording_Delete_Target);
	if bytes > v.budget-min(v.budget, viewer_content_bytes(v))
	{
		return .Budget_Exceeded;
	}
	p.targets = make([]Recording_Delete_Target, count);
	p.owned_bytes += bytes;
	count = 0;
	for run, run_index in p.runs
	{
		start: int = count;
		for entry, entry_index in run.inventory.entries
		{
			if entry.state != .Present || (p.scope == .Sample && (entry.worker != p.worker || entry.sample != p.sample))
			{
				continue;
			}
			duplicate: scene.Availability;
			for previous in p.targets[start:count]
			{
				previous_name: string = run.inventory.entries[previous.entry].name;
				if (strings.equal_fold(previous_name, entry.name) when ODIN_OS == .Windows else previous_name == entry.name)
				{
					duplicate = .Available;
					break;
				}
			}
			if duplicate == .Unavailable
			{
				p.targets[count] = {run=run_index, entry=entry_index};
				count += 1;
				p.bytes += entry.bytes;
			}
		}
		if count > start
		{
			p.affected_runs += 1;
		}
	}
	p.targets = p.targets[:count];
	p.phase = .Ready;
	return .Ok;
}

recording_delete_scan :: proc(v: ^Viewer) -> scene.Status
{
	path_buffer: File_Path_Buffer;
	p: ^Recording_Delete = &v.ui.recording_delete;
	if len(p.directories) == 0
	{
		return recording_delete_targets(v);
	}
	directory: Recording_Delete_Directory = pop(&p.directories);
	defer
	{
		p.owned_bytes -= u64(len(directory.path)+len(directory.package_name));
		delete(directory.path);
		delete(directory.package_name);
	}
	if p.scope != .Workspace
	{
		file: Recording_File = recording_directory_inspect(directory.path);
		if file.state != .Present
		{
			error_buffer: [128]u8;
			return recording_delete_exclude(v, directory.path, fmt.bprintf(error_buffer[:], "%v", file.error) if file.state == .Failed else "Run directory missing or unsafe");
		}
		return recording_delete_load_run(v, directory);
	}
	discovery: Result_Discovery = results_discover(directory.path, directory.depth, v.budget-min(v.budget, viewer_content_bytes(v)),
		.Cases if len(directory.package_name) == 0 else .Runs);
	defer results_discovery_close(&discovery);
	if discovery.status == .Budget_Exceeded
	{
		return .Budget_Exceeded;
	}
	if discovery.status != .Ok
	{
		return recording_delete_exclude(v, directory.path, discovery.diagnostic);
	}
	if len(discovery.run.path) > 0
	{
		return recording_delete_load_run(v, directory);
	}
	for file in discovery.files
	{
		if directory.depth <= 0 || (file.type != .Directory && file.type != .Symlink)
		{
			continue;
		}
		child: string = file.fullpath;
		path: string;
		status: scene.Status;
		path, status = recording_absolute_path(child, path_buffer[:]);
		if status == .Ok
		{
			status = recording_delete_queue(v, path, directory.package_name if len(directory.package_name) > 0 else os.base(path), directory.depth-1);
		}
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

recording_delete_recheck :: proc(v: ^Viewer)
{
	path_buffer: File_Path_Buffer;
	p: ^Recording_Delete = &v.ui.recording_delete;
	if p.cursor == len(p.runs)
	{
		for side in 0 ..< (v.count if v.mode == .Replay else 0)
		{
			path: string;
			status: scene.Status;
			path, status = recording_absolute_path(os.name(v.panes[side].reader.file), path_buffer[:]);
			if status != .Ok
			{
				recording_delete_error(p, "Cannot identify the open recording");
				return;
			}
			for target in p.targets
			{
				run: ^Recording_Delete_Run = &p.runs[target.run];
				name: string = run.inventory.entries[target.entry].name;
				if len(path) == len(run.path)+1+len(name) && path[len(run.path)] == '/' &&
					(strings.equal_fold(path[:len(run.path)], run.path) && strings.equal_fold(path[len(run.path)+1:], name) when ODIN_OS == .Windows else
					path[:len(run.path)] == run.path && path[len(run.path)+1:] == name)
				{
					close_content(v);
					break;
				}
			}
			if v.count == 0
			{
				break;
			}
		}
		p.phase, p.cursor = .Removing, 0;
		return;
	}
	run: ^Recording_Delete_Run = &p.runs[p.cursor];
	dataset: report.Result_Dataset;
	defer report.result_dataset_close(&dataset);
	result: report.Report_Result = report.result_dataset_open(&dataset, run.path, run.package_name, v.budget-min(v.budget, viewer_content_bytes(v)));
	if result.status != .Ok
	{
		recording_delete_error(p, "Cannot recheck %s: %s", run.path, result.diagnostic);
		p.phase = .Ready;
		return;
	}
	index: int;
	changed: scene.Availability;
	for &lane in dataset.lanes
	{
		for sample in 0 ..< len(lane.records)-1
		{
			if index >= len(run.inventory.entries)
			{
				changed = .Available;
				break;
			}
			previous: Recording_Target = run.inventory.entries[index];
			if previous.worker != lane.summary.worker_count || previous.sample != sample ||
				previous.name != report.result_sample_recording(&lane, sample, "physics_elapsed_ms")
			{
				changed = .Available;
			}
			index += 1;
		}
	}
	if changed == .Available || index != len(run.inventory.entries)
	{
		current_path: string;
		current_status: scene.Status;
		if v.results.state == .Ready
		{
			current_path, current_status = recording_absolute_path(v.results.path, path_buffer[:]);
		}
		if v.results.state == .Ready && current_status == .Ok && current_path == run.path
		{
			labels: Result_Labels;
			status: scene.Status = result_labels_open(&labels, &dataset, v.budget-min(v.budget, viewer_content_bytes(v)+report.result_dataset_bytes(&dataset)));
			if status != .Ok
			{
				recording_delete_error(p, "Cannot refresh saved result labels: %v", status);
				return;
			}
			v.ui.dropdown, v.ui.choice = 0, {};
			result_labels_close(&v.ui.result_labels);
			v.ui.result_labels = labels;
			report.result_dataset_close(&v.results);
			v.results, dataset = dataset, {};
			v.ui.result_lane = clamp(v.ui.result_lane, 0, i32(len(v.results.lanes)-1));
			v.ui.result_sample = clamp(v.ui.result_sample, 0, i32(len(v.results.lanes[v.ui.result_lane].records)-2));
			recording_refresh_current(v);
		}
		recording_delete_preview(v, p.scope);
		copy(p.message[:], "Saved associations changed. Review the updated files and confirm again");
		return;
	}
	p.cursor += 1;
}

recording_delete_finish :: proc(v: ^Viewer)
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	p.phase = .Complete;
	p.detail_scroll = {};
	p.detail_width = 0;
	fmt.bprintf(v.ui.recording_message[:511], "Deleted %d | Already absent %d | Failed %d | Unattempted %d",
		p.deleted, p.absent, p.failed, len(p.targets)-p.cursor);
	if v.results.state == .Ready
	{
		recording_refresh_current(v);
	}
	results_refresh(v);
	recordings_close(&v.ui.recordings);
	if p.scope != .Workspace && p.failed == 0
	{
		recording_delete_close(p);
	}
}

recording_delete_step :: proc(v: ^Viewer)
{
	path_buffer: File_Path_Buffer;
	p: ^Recording_Delete = &v.ui.recording_delete;
	switch p.phase
	{
	case .Scanning:
		status: scene.Status = recording_delete_scan(v);
		if status != .Ok
		{
			recording_delete_error(p, "Cannot finish cleanup preview: %v", status);
		}
	case .Rechecking:
		recording_delete_recheck(v);
	case .Removing:
		end: int = min(len(p.targets), p.cursor+8);
		for p.cursor < end
		{
			target: ^Recording_Delete_Target = &p.targets[p.cursor];
			run: ^Recording_Delete_Run = &p.runs[target.run];
			path: string;
			status: scene.Status;
			path, status = recording_join_path(path_buffer[:], run.path, run.inventory.entries[target.entry].name);
			file: Recording_File = {state=.Failed, error=os.General_Error.Invalid_Path};
			if status == .Ok
			{
				file = recording_remove(path);
			}
			switch file.state
			{
			case .Present:
				target.outcome = .Deleted;
				p.deleted += 1;
			case .Missing:
				target.outcome = .Absent;
				p.absent += 1;
			case .Rejected, .Failed:
				target.outcome = .Failed;
				p.failed += 1;
				if file.state == .Failed
				{
					fmt.bprintf(target.diagnostic[:95], "%v", file.error);
				}
				else
				{
					copy(target.diagnostic[:95], "Not a regular recording file");
				}
			}
			p.cursor += 1;
		}
		if p.cursor == len(p.targets)
		{
			recording_delete_finish(v);
		}
	case .Closed, .Ready, .Complete, .Failed:
	}
}

ui_recording_delete_rows :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	d: f32 = v.ui.dpi;
	count: int = len(p.targets)+len(p.exclusions);
	view: rl.Rectangle;
	rl.GuiScrollPanel(bounds, nil, {0, 0, max(bounds.width-20*d, p.detail_width), f32(count)*28*d}, &p.detail_scroll, &view);
	first: int = max(0, int(-p.detail_scroll.y/(28*d)));
	last: int = min(count, first+int(view.height/(28*d))+2);
	rl.BeginScissorMode(i32(view.x), i32(view.y), i32(view.width), i32(view.height));
	for index in first ..< last
	{
		out: Text_Output = {font=v.ui.font, size=16*d, spacing=d, mode=.Draw, color={223, 227, 233, 255},
			position={view.x+4*d+p.detail_scroll.x, view.y+f32(index)*28*d+p.detail_scroll.y+5*d}};
		out.origin = out.position;
		if index >= len(p.targets)
		{
			fmt.wprintf({procedure=ui_text_write, data=&out}, "Excluded | %s", p.exclusions[index-len(p.targets)]);
		}
		else
		{
			target: ^Recording_Delete_Target = &p.targets[index];
			run: ^Recording_Delete_Run = &p.runs[target.run];
			entry: Recording_Target = run.inventory.entries[target.entry];
			if p.phase == .Complete
			{
				fmt.wprintf({procedure=ui_text_write, data=&out}, "%v | %s%s%s / %s / %s | %s/%s", target.outcome,
					strings.string_from_null_terminated_ptr(raw_data(target.diagnostic[:]), 96), " | " if target.outcome == .Failed else "",
					benchmark_label(run.package_name), os.base(run.path), entry.name, run.path, entry.name);
			}
			else
			{
				fmt.wprintf({procedure=ui_text_write, data=&out}, "%s / %s | %d thread%s | Sample %d | %s | %s/%s", benchmark_label(run.package_name), os.base(run.path),
					entry.worker, "" if entry.worker == 1 else "s", entry.sample+1, entry.name, run.path, entry.name);
			}
		}
		p.detail_width = max(p.detail_width, out.width+16*d);
	}
	rl.EndScissorMode();
}

ui_recording_delete :: proc(v: ^Viewer)
{
	p: ^Recording_Delete = &v.ui.recording_delete;
	if p.phase == .Closed
	{
		return;
	}
	d: f32 = v.ui.dpi;
	width: f32 = min((800*d if p.scope == .Workspace else 720*d), f32(rl.GetScreenWidth())-32*d);
	height: f32 = min((600*d if p.scope == .Workspace else 520*d), f32(rl.GetScreenHeight())-32*d);
	area: rl.Rectangle = {(f32(rl.GetScreenWidth())-width)/2, (f32(rl.GetScreenHeight())-height)/2, width, height};
	rl.DrawRectangle(0, 0, rl.GetScreenWidth(), rl.GetScreenHeight(), {0, 0, 0, 96});
	rl.DrawRectangleRec(area, BACKGROUND);
	rl.DrawRectangleLinesEx(area, d, {113, 173, 230, 255});
	x, y: f32 = area.x+16*d, area.y+12*d;
	ui_heading(&v.ui, {x, y, width-32*d, 32*d}, "Delete all recordings in this workspace?" if p.scope == .Workspace else "Delete recordings");
	if p.scope == .Workspace
	{
		ui_cell(&v.ui, {x, y+36*d, width-32*d, 24*d}, "results/ and build/benchmark-results/{windows_amd64,linux_amd64}/");
		ui_cell(&v.ui, {x, y+64*d, width-32*d, 24*d}, "Saved runs only | Excludes pending, failed, partial and unassociated files");
	}
	else
	{
		ui_cell(&v.ui, {x, y+36*d, width-32*d, 28*d}, v.results.path);
		scope: i32 = i32(p.scope);
		if p.phase != .Ready
		{
			rl.GuiDisable();
		}
		rl.GuiToggleGroup({x, y+72*d, (width-40*d)/2, 28*d}, "Selected sample;All recordings in this run", &scope);
		rl.GuiEnable();
		if scope != i32(p.scope)
		{
			recording_delete_preview(v, Recording_Delete_Scope(scope));
		}
	}
	ui_text({x, y+108*d, width-32*d, 28*d}, "%d recording%s | %d run%s | %.2f MiB | %d excluded", len(p.targets),
		"" if len(p.targets) == 1 else "s", p.affected_runs, "" if p.affected_runs == 1 else "s", f64(p.bytes)/(1024*1024), len(p.exclusions));
	top: f32 = y+144*d;
	footer: f32 = area.y+height-(146*d if p.scope == .Workspace else 130*d);
	button_y: f32 = footer+(100*d if p.scope == .Workspace else 72*d);
	ui_recording_delete_rows(v, {x, top, width-32*d, max(28*d, footer-top-8*d)});
	ui_text({x, footer, width-32*d, 24*d}, "Permanent deletion. Measurements and reports are retained");
	message_buffer: [128]u8;
	message: string = strings.string_from_null_terminated_ptr(raw_data(p.message[:]), 512);
	switch p.phase
	{
	case .Scanning: message = fmt.bprintf(message_buffer[:], "Scanning saved runs... %d inspected", len(p.runs));
	case .Rechecking: message = fmt.bprintf(message_buffer[:], "Rechecking saved associations... %d of %d", p.cursor, len(p.runs));
	case .Removing: message = fmt.bprintf(message_buffer[:], "Deleting recordings... %d of %d | %d failed", p.cursor, len(p.targets), p.failed);
	case .Complete: message = strings.string_from_null_terminated_ptr(raw_data(v.ui.recording_message[:]), 512);
	case .Ready:
		if len(p.targets) == 0
		{
			message = "No eligible recordings to delete";
		}
	case .Closed, .Failed:
	}
	ui_cell(&v.ui, {x, footer+28*d, width-32*d, 26*d}, message);
	if p.scope == .Workspace && len(p.exclusions) > 0
	{
		ui_text({x, footer+56*d, width-32*d, 24*d}, "Excluded locations were not cleaned. See details above");
	}
	if p.opened == .Unavailable && rl.IsKeyPressed(.TAB)
	{
		p.focus = 1-p.focus;
	}
	left: cstring = "Cancel";
	if p.phase == .Removing
	{
		left = "Stop";
	}
	else if p.phase == .Complete || p.phase == .Failed || (p.phase == .Ready && len(p.targets) == 0)
	{
		left = "Close";
	}
	if rl.GuiButton({x, button_y, 100*d, 30*d}, left) ||
		(p.opened == .Unavailable && (rl.IsKeyPressed(.ESCAPE) || (p.focus == 0 && rl.IsKeyPressed(.ENTER))))
	{
		if p.phase == .Removing
		{
			recording_delete_finish(v);
		}
		else
		{
			v.cursor.playback = p.playback if v.count > 0 else v.cursor.playback;
			recording_delete_close(p);
		}
		return;
	}
	if p.phase == .Ready && len(p.targets) > 0
	{
		button: rl.Rectangle = {x+width-280*d, button_y, 248*d, 30*d};
		if p.focus == 1
		{
			rl.DrawRectangleLinesEx(button, 2*d, {113, 173, 230, 255});
		}
		label: [64]u8;
		fmt.bprintf(label[:63], "Delete %d recording%s", len(p.targets), "" if len(p.targets) == 1 else "s");
		clicked: scene.Availability = .Available if rl.GuiButton(button, cast(cstring)raw_data(label[:])) else .Unavailable;
		if p.opened == .Unavailable && (clicked == .Available || (p.focus == 1 && rl.IsKeyPressed(.SPACE)))
		{
			p.phase, p.cursor = .Rechecking, 0;
			return;
		}
	}
	p.opened = .Unavailable;
	recording_delete_step(v);
}
