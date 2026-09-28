package physics_viewer

import "base:runtime"
import "core:fmt"
import replay "../physics_replay"
import scene "../physics_scene"
import scenarios "../physics_scenarios"
import report "../benchmark_report"

viewer_budget_failure :: proc(v: ^Viewer, required, available: u64) -> scene.Status
{
	v.required_bytes, v.available_bytes = required, available;
	fmt.eprintfln("VISUAL_BUDGET_EXCEEDED required_bytes=%d available_bytes=%d", required, available);
	return .Budget_Exceeded;
}

pane_close :: proc(pane: ^Pane)
{
	renderer_destroy(&pane.renderer);
	replay.reader_close(&pane.reader);
	viewport_release(pane);
	pane^ = {};
}

close_content :: proc(v: ^Viewer)
{
	for &pane in v.panes
	{
		pane_close(&pane);
	}
	if v.live != nil
	{
		scenarios.session_destroy(v.live);
		free(v.live);
		v.live = nil;
	}
	delete(v.pairs);
	v.pairs = nil;
	v.synchronization = .Independent;
	v.camera_link = .Independent;
	v.active_pane = 0;
	v.cursor = {speed=3};
	v.ui.drag = .None;
	v.ui.dropdown, v.ui.text_field = 0, 0;
	v.ui.choice, v.ui.edit = {}, {};
	v.ui.focused_pane = -1;
	v.ui.fit = .Available;
	v.ui.filter = {};
	v.count = 0;
	v.mode = .Browser;
}

viewer_content_bytes :: proc(v: ^Viewer) -> u64
{
	bytes: u64 = u64(len(v.pairs))*size_of([2]int)+report.result_dataset_bytes(&v.results)+
		v.ui.runs.owned_bytes+v.ui.recordings.owned_bytes+v.ui.result_labels.owned_bytes+v.ui.recording_inventory.owned_bytes+v.ui.recording_delete.owned_bytes+u64(len(v.ui.visible_runs))*size_of(int);
	if v.live != nil
	{
		bytes += scenarios.session_bytes(v.live);
	}
	for &pane in v.panes
	{
		bytes += pane.renderer.bytes+pane.reader.owned_bytes;
	}
	return bytes;
}

viewer_replay_budget :: proc(v: ^Viewer)
{
	if v.mode != .Replay
	{
		return;
	}
	used: u64 = viewer_content_bytes(v);
	spare: u64 = (v.budget-min(v.budget, used))/u64(v.count);
	for side in 0 ..< v.count
	{
		v.panes[side].reader.budget = v.panes[side].reader.owned_bytes+spare;
	}
}

open_live :: proc(v: ^Viewer, recipe: scenarios.Recipe) -> scene.Status
{
	held: u64 = viewer_content_bytes(v);
	if held > v.budget
	{
		return viewer_budget_failure(v, held, v.budget);
	}
	candidate: ^scenarios.Session;
	allocation_error: runtime.Allocator_Error;
	candidate, allocation_error = new(scenarios.Session);
	if allocation_error != nil
	{
		return .Out_Of_Memory;
	}
	renderer: Renderer;
	status: scene.Status = scenarios.session_create(candidate, recipe, v.budget-held);
	defer if status != .Ok
	{
		renderer_destroy(&renderer);
		scenarios.session_destroy(candidate);
		free(candidate);
	}
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, candidate.required_bytes, v.budget-held);
		}
		return status;
	}
	used: u64 = scenarios.session_bytes(candidate);
	if used > v.budget-held
	{
		status = viewer_budget_failure(v, used, v.budget-held);
		return status;
	}
	status = renderer_create(&renderer, &candidate.observation.packet, v.budget-held-used);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, renderer.required_bytes, v.budget-held-used);
		}
		return status;
	}
	camera: Camera = camera_fit(&candidate.observation.packet, &renderer, 1);
	close_content(v);
	v.live = candidate;
	v.panes[0].renderer, v.panes[0].camera = renderer, camera;
	v.mode, v.count = .Live, 1;
	v.ui.category = 1;
	catalog: [scenarios.Scenario]scenarios.Descriptor = scenarios.CATALOG;
	v.ui.example_kind = 1 if catalog[candidate.recipe.scenario].presentation == .Diagnostic else 0;
	visible_index: i32;
	for descriptor, id in catalog
	{
		if descriptor.presentation == catalog[candidate.recipe.scenario].presentation
		{
			if id == candidate.recipe.scenario
			{
				v.ui.catalog_selected[1] = visible_index;
				break;
			}
			visible_index += 1;
		}
	}
	v.ui.status = .Ok;
	v.cursor = {count=candidate.extent+1, speed=3};
	return .Ok;
}

open_replays :: proc(v: ^Viewer, path, compare: string) -> scene.Status
{
	held: u64 = viewer_content_bytes(v);
	if held > v.budget
	{
		return viewer_budget_failure(v, held, v.budget);
	}
	candidate: Viewer = {budget=v.budget-held};
	status: scene.Status = prepare_replays(&candidate, path, compare);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			v.required_bytes, v.available_bytes = candidate.required_bytes, candidate.available_bytes;
		}
		close_content(&candidate);
		return status;
	}
	close_content(v);
	v.mode, v.count = candidate.mode, candidate.count;
	v.cursor, v.pairs = candidate.cursor, candidate.pairs;
	v.synchronization, v.active_pane = candidate.synchronization, candidate.active_pane;
	v.camera_link = candidate.camera_link;
	for side in 0 ..< v.count
	{
		v.panes[side].reader = candidate.panes[side].reader;
		v.panes[side].renderer = candidate.panes[side].renderer;
		v.panes[side].camera = candidate.panes[side].camera;
	}
	viewer_replay_budget(v);
	return .Ok;
}

open_comparison :: proc(v: ^Viewer, path: string) -> (status: scene.Status)
{
	held: u64 = viewer_content_bytes(v);
	if held > v.budget
	{
		return viewer_budget_failure(v, held, v.budget);
	}
	available: u64 = v.budget-held;
	candidate: Pane;
	pairs: [][2]int;
	defer if status != .Ok
	{
		pane_close(&candidate);
		delete(pairs);
	}
	status = replay.reader_open(&candidate.reader, path, available);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, candidate.reader.required_bytes, available);
		}
		return status;
	}
	pairs, status = replay.comparison_index(&v.panes[0].reader, &candidate.reader, available-candidate.reader.owned_bytes);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, v.panes[0].reader.required_bytes, available-candidate.reader.owned_bytes);
		}
		return status;
	}
	paired_index: int = -1;
	for pair, index in pairs
	{
		if pair[0] == v.panes[0].reader.selected
		{
			paired_index = index;
			break;
		}
	}
	pair_bytes: u64 = u64(len(pairs))*size_of([2]int);
	candidate.reader.budget = available-pair_bytes;
	status = replay.reader_seek(&candidate.reader, pairs[paired_index][1] if paired_index >= 0 else 0);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, candidate.reader.required_bytes, candidate.reader.budget);
		}
		return status;
	}
	available -= pair_bytes+candidate.reader.owned_bytes;
	status = renderer_create(&candidate.renderer, &candidate.reader.packet, available,
		.Outline if candidate.reader.metadata.scenario == "benchmark/container" else .Filled);
	if status != .Ok
	{
		if status == .Budget_Exceeded
		{
			return viewer_budget_failure(v, candidate.renderer.required_bytes, available);
		}
		return status;
	}
	candidate.camera = v.panes[0].camera;
	pane_close(&v.panes[1]);
	delete(v.pairs);
	v.panes[1], v.pairs = candidate, pairs;
	v.count, v.active_pane = 2, 0;
	v.camera_link = .Linked;
	v.synchronization = .Linked if paired_index >= 0 else .Independent;
	v.cursor.index = paired_index if paired_index >= 0 else v.panes[0].reader.selected;
	v.cursor.count = len(pairs) if paired_index >= 0 else len(v.panes[0].reader.index);
	v.cursor.playback, v.cursor.accumulator = .Paused, 0;
	viewer_replay_budget(v);
	return .Ok;
}

close_comparison :: proc(v: ^Viewer)
{
	pane_close(&v.panes[1]);
	delete(v.pairs);
	v.pairs = nil;
	v.count, v.active_pane = 1, 0;
	v.synchronization, v.camera_link = .Independent, .Independent;
	v.cursor.index, v.cursor.count = v.panes[0].reader.selected, len(v.panes[0].reader.index);
	v.cursor.playback, v.cursor.accumulator = .Paused, 0;
	viewer_replay_budget(v);
}

prepare_replays :: proc(v: ^Viewer, path, compare: string) -> scene.Status
{
	v.mode, v.count = .Replay, 1;
	v.ui.results = .Unavailable;
	if len(compare)>0
	{
		v.count=2;
	}
	paths: [2]string = {path, compare};
	v.cursor = {speed=3, count=max(int)};
	for side in 0 ..< v.count
	{
		pane: ^Pane = &v.panes[side];
		status: scene.Status = replay.reader_open(&pane.reader, paths[side], v.budget/u64(v.count));
		if status != .Ok
		{
			if status == .Budget_Exceeded
			{
				return viewer_budget_failure(v, pane.reader.required_bytes, pane.reader.budget);
			}
			return status;
		}
		status = replay.reader_seek(&pane.reader, 0);
		if status != .Ok
		{
			if status == .Budget_Exceeded
			{
				return viewer_budget_failure(v, pane.reader.required_bytes, pane.reader.budget);
			}
			return status;
		}
		status = renderer_create(&pane.renderer, &pane.reader.packet, v.budget/u64(v.count)-pane.reader.owned_bytes,
			.Outline if pane.reader.metadata.scenario == "benchmark/container" else .Filled);
		if status != .Ok
		{
			if status == .Budget_Exceeded
			{
				return viewer_budget_failure(v, pane.renderer.required_bytes, v.budget/u64(v.count)-pane.reader.owned_bytes);
			}
			return status;
		}
		pane.reader.budget = v.budget/u64(v.count)-pane.renderer.bytes;
		pane.camera = camera_fit(&pane.reader.packet, &pane.renderer, v.count);
		v.cursor.count = min(v.cursor.count, len(pane.reader.index));
	}
	if v.count==2
	{
		v.camera_link = .Linked;
		v.panes[1].camera = v.panes[0].camera;
		used: u64;
		for side in 0 ..< v.count
		{
			used+=v.panes[side].reader.owned_bytes+v.panes[side].renderer.bytes;
		}
		if used>v.budget
		{
			return viewer_budget_failure(v, used, v.budget);
		}
		status: scene.Status;
		v.pairs, status = replay.comparison_index(&v.panes[0].reader, &v.panes[1].reader, v.budget-used);
		if status!=.Ok
		{
			if status == .Budget_Exceeded
			{
				return viewer_budget_failure(v, v.panes[0].reader.required_bytes, v.budget-used);
			}
			return status;
		}
		if status==.Ok
		{
			v.synchronization = .Linked;
			v.cursor.count=len(v.pairs);
			for side in 0 ..< v.count
			{
				v.panes[side].reader.budget-=u64(len(v.pairs))*size_of([2]int)/2;
			}
			return select_frame(v, 0);
		}
		v.cursor.count=len(v.panes[0].reader.index);
	}
	return .Ok;
}

@(private)
packet :: proc(v: ^Viewer, side: int) -> ^scene.Scene_Packet
{
	if v.mode == .Live
	{
		return &v.live.observation.packet;
	}
	return &v.panes[side].reader.packet;
}
