package physics_viewer

import "core:strings"
import observation "../physics_observation"
import rl "vendor:raylib"
import scene "../physics_scene"
import scenarios "../physics_scenarios"
import replay "../physics_replay"
import report "../benchmark_report"

playback_interval :: proc(v: ^Viewer) -> f64
{
	if v.mode == .Live
	{
		return 0.5 if v.live.axis == .Operation else f64(v.live.timestep);
	}
	if v.mode == .Replay
	{
		side: int = v.active_pane if v.synchronization == .Independent else 0;
		metadata: scene.Metadata = v.panes[side].reader.metadata;
		return 0.5 if metadata.axis == .Operation else f64(metadata.timestep);
	}
	return 0.5;
}

transport_toggle_link :: proc(v: ^Viewer) -> scene.Status
{
	v.cursor.playback, v.cursor.accumulator = .Paused, 0;
	if v.synchronization == .Linked
	{
		v.synchronization = .Independent;
		v.cursor.count = len(v.panes[v.active_pane].reader.index);
		v.cursor.index = v.panes[v.active_pane].reader.selected;
		return .Ok;
	}
	if len(v.pairs) == 0
	{
		return .Unsupported;
	}
	previous: scene.Cursor = v.cursor;
	v.synchronization, v.cursor.count = .Linked, len(v.pairs);
	status: scene.Status = select_frame(v, 0);
	if status != .Ok
	{
		v.synchronization, v.cursor = .Independent, previous;
	}
	return status;
}

Synchronization :: enum
{
	Independent, Linked,
}

select_frame :: proc(v: ^Viewer, index: int) -> scene.Status
{
	target: int = index;
	if target < 0 || target >= v.cursor.count
	{
		return .Invalid_Data;
	}
	if v.mode == .Replay
	{
		if v.synchronization == .Linked
		{
			for side in 0 ..< v.count
			{
				status: scene.Status = replay.reader_prepare(&v.panes[side].reader, v.pairs[target][side]);
				if status != .Ok
				{
					if status == .Budget_Exceeded
					{
						return viewer_budget_failure(v, v.panes[side].reader.required_bytes, v.panes[side].reader.budget);
					}
					return status;
				}
			}
			for side in 0 ..< v.count
			{
				replay.reader_commit(&v.panes[side].reader);
			}
		}
		else
		{
			status: scene.Status = replay.reader_seek(&v.panes[v.active_pane].reader, target);
			if status != .Ok
			{
				if status == .Budget_Exceeded
				{
					return viewer_budget_failure(v, v.panes[v.active_pane].reader.required_bytes, v.panes[v.active_pane].reader.budget);
				}
				return status;
			}
		}
	}
	else
	{
		if target < v.live.tick
		{
			return .Unsupported;
		}
		for v.live.tick < target
		{
			occupied: u64 = v.panes[0].renderer.bytes+scenarios.session_bytes(v.live)-observation.observation_bytes(&v.live.observation)+report.result_dataset_bytes(&v.results);
			if occupied > v.budget
			{
				return viewer_budget_failure(v, occupied, v.budget);
			}
			v.live.observation.budget = v.budget-occupied;
			status: scene.Status = scenarios.session_step(v.live, viewer_input(v));
			if status != .Ok
			{
				if status == .Budget_Exceeded
				{
					return viewer_budget_failure(v, v.live.observation.required_bytes, v.live.observation.budget);
				}
				return status;
			}
			if v.live.phase == .Reset
			{
				target = v.live.tick;
				v.cursor.playback, v.cursor.accumulator = .Paused, 0;
			}
			target = min(target, v.live.extent);
			v.cursor.count = v.live.extent+1;
			used: u64 = scenarios.session_bytes(v.live)+report.result_dataset_bytes(&v.results);
			if used > v.budget
			{
				status = viewer_budget_failure(v, used, v.budget);
			}
			else
			{
				status = renderer_reconcile(&v.panes[0].renderer, &v.live.observation.packet, v.budget-used);
			}
			if status != .Ok
			{
				if status == .Budget_Exceeded && used <= v.budget
				{
					viewer_budget_failure(v, v.panes[0].renderer.required_bytes, v.budget-used);
				}
				v.live.state = .Failed;
				return status;
			}
		}
	}
	for side in 0 ..< v.count
	{
		if v.mode != .Replay || v.synchronization == .Linked || side == v.active_pane
		{
			v.panes[side].data_dirty = .Available;
		}
	}
	v.cursor.index=target;
	return .Ok;
}

transport_advance :: proc(v: ^Viewer, elapsed: f64, previous_status: scene.Status) -> scene.Status
{
	status: scene.Status = previous_status;
	if live_diagnostic(v) == .Available
	{
		v.cursor.playback = .Paused;
	}
	interval: f64 = playback_interval(v);
	next: scene.Cursor=v.cursor;
	previous_index: int = v.cursor.index;
	scene.cursor_advance(&next, elapsed, interval, 8);
	if next.index!=v.cursor.index
	{
		status=select_frame(v, next.index);
	}
	if status==.Ok
	{
		if v.mode == .Live
		{
			next.count = v.live.extent+1;
			next.index = v.live.tick;
			if v.cursor.playback == .Paused && next.index != previous_index
			{
				next.playback, next.accumulator = .Paused, 0;
			}
			if v.live.state == .Completed
			{
				next.playback, next.accumulator = .Paused, 0;
			}
		}
		v.cursor=next;
	}
	if status!=.Ok
	{
		v.cursor.playback=.Paused;
		v.cursor.accumulator=0;
	}
	return status;
}

ui_transport :: proc(v: ^Viewer, bounds: rl.Rectangle) -> scene.Status
{
	status: scene.Status = v.ui.status;
	d: f32 = v.ui.dpi;
	x, y: f32 = bounds.x, bounds.y;
	if v.mode == .Replay
	{
		index: f32 = f32(v.cursor.index);
		rl.GuiSliderBar({x, y, bounds.width, 18*d}, nil, nil, &index, 0, f32(v.cursor.count-1));
		if int(index) != v.cursor.index
		{
			v.cursor.playback, v.cursor.accumulator = .Paused, 0;
			status = select_frame(v, int(index));
		}
		y += 26*d;
	}
	diagnostic: scene.Availability = live_diagnostic(v);
	if diagnostic == .Unavailable && rl.GuiButton({x, y, 72*d, 28*d}, "Pause" if v.cursor.playback == .Running else "Play")
	{
		status = transport_play_pause(v);
	}
	if rl.GuiButton({x+80*d, y, 130*d, 28*d}, "Next operation" if diagnostic == .Available else "Step")
	{
		v.cursor.playback, v.cursor.accumulator = .Paused, 0;
		status = select_frame(v, min(v.cursor.index+1, v.cursor.count-1));
	}
	if rl.GuiButton({x+218*d, y, 64*d, 28*d}, "Reset")
	{
		if v.mode == .Live
		{
			status = open_live(v, v.live.recipe);
		}
		else
		{
			v.cursor.playback, v.cursor.accumulator = .Paused, 0;
			status = select_frame(v, 0);
		}
	}
	if diagnostic == .Unavailable
	{
		speed: i32 = i32(v.cursor.speed);
		ui_choice_static(&v.ui, 20, {x+290*d, y, 80*d, 28*d}, []cstring{"0.1x", "0.25x", "0.5x", "1x", "2x", "4x"}, &speed);
		v.cursor.speed = int(speed);
	}
	if v.mode == .Live
	{
		catalog: [scenarios.Scenario]scenarios.Descriptor = scenarios.CATALOG;
		if .Input in catalog[v.live.recipe.scenario].capabilities
		{
			ui_text({x, y+34*d, 76*d, 28*d}, "Controls");
			selected: i32 = 1 if v.live.recipe.input == .Live else 0;
			rl.GuiToggleGroup({x+84*d, y+34*d, 84*d, 28*d}, "Scripted;WASD", &selected);
			input: scene.Input_Mode = .Live if selected == 1 else .Scripted;
			if input != v.live.recipe.input
			{
				recipe: scenarios.Recipe = v.live.recipe;
				recipe.input = input;
				status = open_live(v, recipe);
			}
		}
	}
	else if v.mode == .Replay
	{
		ui_text({x+382*d, y, 44*d, 28*d}, "Loop");
		loop: i32 = 1 if v.cursor.loop == .On else 0;
		rl.GuiToggleGroup({x+430*d, y, 48*d, 28*d}, "Off;On", &loop);
		v.cursor.loop = .On if loop == 1 else .Off;
	}
	ui_text({x, bounds.y-28*d, bounds.width, 24*d}, "%s %d / %d  |  %s",
		"Operation" if diagnostic == .Available else "Frame", v.cursor.index+1, v.cursor.count,
		"Finished - Space to replay" if v.mode == .Replay && v.cursor.index == v.cursor.count-1 else
		("Playing" if v.cursor.playback == .Running else "Paused"));
	return status;
}

live_diagnostic :: proc(v: ^Viewer) -> scene.Availability
{
	if v.mode != .Live
	{
		return .Unavailable;
	}
	catalog: [scenarios.Scenario]scenarios.Descriptor = scenarios.CATALOG;
	return .Available if catalog[v.live.recipe.scenario].presentation == .Diagnostic else .Unavailable;
}

transport_play_pause :: proc(v: ^Viewer) -> scene.Status
{
	if v.cursor.playback != .Running && v.mode == .Replay && v.cursor.index == v.cursor.count-1
	{
		status: scene.Status = select_frame(v, 0);
		if status != .Ok
		{
			return status;
		}
	}
	v.cursor.playback = .Paused if v.cursor.playback == .Running else .Running;
	v.cursor.accumulator = 0;
	return .Ok;
}

ui_comparison :: proc(v: ^Viewer)
{
	d: f32 = v.ui.dpi;
	if rl.GuiButton({488*d, 52*d, 144*d, 28*d}, "Unlink playback" if v.synchronization == .Linked else "Link playback")
	{
		v.ui.status = transport_toggle_link(v);
	}
	if rl.GuiButton({640*d, 52*d, 144*d, 28*d}, "Unlink cameras" if v.camera_link == .Linked else "Link cameras")
	{
		v.camera_link = .Independent if v.camera_link == .Linked else .Linked;
		if v.camera_link == .Linked
		{
			v.panes[1-v.active_pane].camera = v.panes[v.active_pane].camera;
		}
	}
	if rl.GuiButton({792*d, 52*d, 156*d, 28*d}, "Close comparison")
	{
		close_comparison(v);
	}
}

ui_comparison_metadata :: proc(v: ^Viewer)
{
	d: f32 = v.ui.dpi;
	a, b: scene.Metadata = v.panes[0].reader.metadata, v.panes[1].reader.metadata;
	settings: scene.Configuration_Comparison = scene.comparison_configuration(a, b);
	source: scene.Configuration_Comparison = .Matching;
	for value in ([8]string{a.source, a.revision, a.compiler, a.host, b.source, b.revision, b.compiler, b.host})
	{
		if len(value) == 0 || strings.equal_fold(value, "Unknown")
		{
			source = .Unavailable;
			break;
		}
	}
	if source != .Unavailable && (a.source != b.source || a.revision != b.revision || a.compiler != b.compiler || a.host != b.host)
	{
		source = .Different;
	}
	hovered: int = -1;
	for side in 0 ..< 2
	{
		pane: rl.Rectangle = v.ui.images[side];
		bounds: rl.Rectangle = {pane.x, v.ui.body.y, pane.width, 28*d};
		ui_cell(&v.ui, bounds, "%s: %v", "Settings" if side == 0 else "Source", settings if side == 0 else source);
		ui_cell(&v.ui, {pane.x, bounds.y+28*d, pane.width, 24*d}, "%s | %s", "A" if side == 0 else "B", v.panes[side].reader.metadata.scenario);
		if rl.CheckCollisionPointRec(rl.GetMousePosition(), bounds)
		{
			hovered = side;
		}
	}
	if hovered < 0
	{
		v.ui.comparison_scroll = 0;
		return;
	}
	metadata: [2]scene.Metadata = {a, b};
	counts: [2]int = {4, 4};
	if hovered == 0
	{
		for value, side in metadata
		{
			counts[side] = 3;
			if value.settings_state == .Available
			{
				counts[side] = 2+strings.count(value.settings, ";")+1;
			}
		}
	}
	rows: int = max(counts[0], counts[1]);
	lines: int = 2 if v.ui.body.width < 1000*d else 1;
	visible: int = min(rows, max(1, int((v.ui.body.height-84*d)/(24*d*f32(lines)))));
	v.ui.comparison_scroll = clamp(v.ui.comparison_scroll-int(rl.GetMouseWheelMove()), 0, rows-visible);
	bounds: rl.Rectangle = {v.ui.body.x, v.ui.body.y+28*d, v.ui.body.width, f32(visible*lines+1)*24*d};
	rl.DrawRectangleRec(bounds, {28, 31, 36, 255});
	rl.DrawRectangleLinesEx(bounds, d, {104, 114, 125, 255});
	for side in 0 ..< 2
	{
		x: f32 = v.ui.images[side].x+8*d;
		width: f32 = v.ui.images[side].width-16*d;
		if lines == 2
		{
			x, width = bounds.x+8*d, bounds.width-16*d;
		}
		if lines == 1 || side == 0
		{
			ui_cell(&v.ui, {x, bounds.y, width, 24*d}, "%s | %d-%d / %d", "A / B" if lines == 2 else ("A" if side == 0 else "B"), v.ui.comparison_scroll+1, v.ui.comparison_scroll+visible, rows);
		}
		for row in 0 ..< visible
		{
			index: int = row+v.ui.comparison_scroll;
			if index < counts[side]
			{
				line: int = row*lines+1+(side if lines == 2 else 0);
				name, value: string;
				m: scene.Metadata = metadata[side];
				if hovered == 1
				{
					names: [4]string = {"Source=", "Revision=", "Compiler=", "Host="};
					values: [4]string = {m.source, m.revision, m.compiler, m.host};
					name, value = names[index], values[index];
				}
				else if index < 2
				{
					name = "Build=" if index == 0 else "Components=";
					value = m.configuration if index == 0 else m.components;
				}
				else if m.settings_state != .Available
				{
					value = "Settings=unavailable";
				}
				else
				{
					remaining: string = m.settings;
					for _ in 0 ..= index-2
					{
						value, _ = strings.split_by_byte_iterator(&remaining, ';');
					}
				}
				ui_cell(&v.ui, {x, bounds.y+f32(line)*24*d, width, 24*d}, "%s%s%s", ("A | " if side == 0 else "B | ") if lines == 2 else "", name, value);
			}
		}
	}
}
