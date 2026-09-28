package physics_viewer

import "core:fmt"
import rl "vendor:raylib"
import scene "../physics_scene"

Worker_Counts :: bit_set[1..=64; u64];
WORKER_DROPDOWN :: -1;

Worker_Chooser :: struct
{
	bounds: rl.Rectangle,
	focus, scroll, visible_rows: int,
	opened, closing: scene.Availability,
}

worker_count :: proc(counts: Worker_Counts) -> int
{
	count: int;
	for value in 1 ..= 64
	{
		if value in counts
		{
			count += 1;
		}
	}
	return count;
}

worker_arguments :: proc(counts: Worker_Counts, buffer: []u8) -> string
{
	written: int;
	for value in 1 ..= 64
	{
		if value in counts
		{
			written += len(fmt.bprintf(buffer[written:], "%s%d", "," if written > 0 else "", value));
		}
	}
	return string(buffer[:written]);
}

worker_summary :: proc(counts: Worker_Counts, buffer: []u8) -> string
{
	written: int;
	for value: int = 1; value <= 64; value += 1
	{
		if value not_in counts
		{
			continue;
		}
		first: int = value;
		for value < 64 && (value+1 in counts)
		{
			value += 1;
		}
		written += len(fmt.bprintf(buffer[written:], "%s%d", ", " if written > 0 else "", first));
		if first != value
		{
			written += len(fmt.bprintf(buffer[written:], "-%d", value));
		}
	}
	return string(buffer[:written]);
}

ui_workers :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	ui: ^UI = &v.ui;
	count: int = worker_count(v.launch.workers);
	summary: [256]u8;
	text: string = worker_summary(v.launch.workers, summary[:]);
	limit: int = max(1, int(bounds.width/(9*ui.dpi))-18);
	suffix: string;
	if len(text) > limit
	{
		text, suffix = text[:limit], "...";
	}
	label: [288]u8;
	if count > 0
	{
		fmt.bprintf(label[:287], "%s%s (%d selected)  v", text, suffix, count);
	}
	else
	{
		copy(label[:], "None selected  v");
	}
	if ui.text_field == 10
	{
		ui.edit.bounds = bounds;
		rl.DrawRectangleLinesEx({bounds.x-2, bounds.y-2, bounds.width+4, bounds.height+4}, 2, {113, 173, 230, 255});
	}
	if rl.GuiButton(bounds, cstring(raw_data(label[:]))) ||
		(!rl.GuiIsLocked() && ui.text_field == 10 && (rl.IsKeyPressed(.ENTER) || rl.IsKeyPressed(.SPACE)))
	{
		ui.dropdown = WORKER_DROPDOWN;
		ui.text_field, ui.edit.active = 0, 0;
		ui.workers = {bounds=bounds, focus=1, opened=.Available};
		for value in 1 ..= 64
		{
			if value in v.launch.workers
			{
				ui.workers.focus = value;
				break;
			}
		}
		rl.GuiLock();
	}
}

ui_worker_popup :: proc(v: ^Viewer)
{
	ui: ^UI = &v.ui;
	if ui.dropdown != WORKER_DROPDOWN
	{
		return;
	}
	p: ^Worker_Chooser = &ui.workers;
	d: f32 = ui.dpi;
	available: int = v.launch.available_workers;
	columns: int = max(1, min(8, available));
	rows: int = (available+columns-1)/columns;
	width: f32 = min(520*d, f32(rl.GetScreenWidth())-24*d);
	height: f32 = min(f32(rows)*38*d+150*d, f32(rl.GetScreenHeight())-32*d);
	area: rl.Rectangle = {min(p.bounds.x, f32(rl.GetScreenWidth())-width-12*d), min(p.bounds.y+p.bounds.height+4*d, f32(rl.GetScreenHeight())-height-12*d), width, height};
	if p.opened == .Unavailable && (rl.IsKeyPressed(.ESCAPE) ||
		(rl.IsMouseButtonPressed(.LEFT) && !rl.CheckCollisionPointRec(rl.GetMousePosition(), area)))
	{
		p.closing = .Available;
	}
	if p.closing == .Available
	{
		// underlying controls remain locked through the release frame
		if !rl.IsMouseButtonDown(.LEFT)
		{
			ui.dropdown = 0;
			ui.text_field = 10;
		}
		return;
	}
	if p.opened == .Available
	{
		rl.GuiLock();
	}
	rl.DrawRectangleRec(area, BACKGROUND);
	rl.DrawRectangleLinesEx(area, d, {113, 173, 230, 255});
	ui_text({area.x+12*d, area.y+8*d, width-24*d, 26*d}, "Thread counts  |  %d of %d selected", worker_count(v.launch.workers), available);
	previous_focus: int = p.focus;
	if p.opened == .Unavailable && rl.IsKeyPressed(.TAB)
	{
		direction: int = -1 if rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT) else 1;
		p.focus = 1+(p.focus-1+direction+available+3)%(available+3);
	}
	if p.focus <= available
	{
		move: int;
		if rl.IsKeyPressed(.LEFT) || rl.IsKeyPressedRepeat(.LEFT)
		{
			move = -1;
		}
		if rl.IsKeyPressed(.RIGHT) || rl.IsKeyPressedRepeat(.RIGHT)
		{
			move = 1;
		}
		if rl.IsKeyPressed(.UP) || rl.IsKeyPressedRepeat(.UP)
		{
			move = -columns;
		}
		if rl.IsKeyPressed(.DOWN) || rl.IsKeyPressedRepeat(.DOWN)
		{
			move = columns;
		}
		p.focus = clamp(p.focus+move, 1, max(1, available));
	}
	activate: int = p.focus if p.opened == .Unavailable && (rl.IsKeyPressed(.SPACE) || rl.IsKeyPressed(.ENTER)) else 0;
	for index in 0 ..< 2
	{
		button: rl.Rectangle = {area.x+12*d+f32(index)*140*d, area.y+40*d, 128*d, 28*d};
		if rl.GuiButton(button, "Select all" if index == 0 else "Select none")
		{
			activate = available+1+index;
			p.focus = activate;
		}
		if p.focus == available+1+index
		{
			rl.DrawRectangleLinesEx(button, 2*d, {113, 173, 230, 255});
		}
	}
	if activate == available+1 || activate == available+2
	{
		v.launch.workers = {};
		if activate == available+1
		{
			for value in 1 ..= available
			{
				v.launch.workers += {value};
			}
		}
	}
	else if activate >= 1 && activate <= available
	{
		v.launch.workers ~= {activate};
	}
	visible_rows: int = max(1, int((height-150*d)/(38*d)));
	if p.opened == .Available || p.focus != previous_focus || p.visible_rows != visible_rows || activate >= 1 && activate <= available
	{
		if p.focus <= available
		{
			p.scroll = clamp(p.scroll, (p.focus-1)/columns-visible_rows+1, (p.focus-1)/columns);
		}
	}
	p.visible_rows = visible_rows;
	p.scroll = clamp(p.scroll-int(rl.GetMouseWheelMove()), 0, max(0, rows-visible_rows));
	cell: f32 = (width-24*d)/f32(columns);
	for row in 0 ..< visible_rows
	{
		for column in 0 ..< columns
		{
			value: int = (row+p.scroll)*columns+column+1;
			if value > available
			{
				break;
			}
			button: rl.Rectangle = {area.x+12*d+f32(column)*cell, area.y+78*d+f32(row)*38*d, cell-4*d, 30*d};
			selected: bool = value in v.launch.workers;
			label: [8]u8;
			fmt.bprintf(label[:7], "%s %d", "x" if selected else " ", value);
			rl.GuiToggle(button, cstring(raw_data(label[:])), &selected);
			if selected
			{
				v.launch.workers += {value};
			}
			else
			{
				v.launch.workers -= {value};
			}
			if rl.IsMouseButtonPressed(.LEFT) && rl.CheckCollisionPointRec(rl.GetMousePosition(), button)
			{
				p.focus = value;
			}
			if p.focus == value
			{
				rl.DrawRectangleLinesEx(button, 2*d, {113, 173, 230, 255});
			}
		}
	}
	ui_text({area.x+12*d, area.y+height-64*d, width-24*d, 24*d}, "Each selected count runs separately");
	done: rl.Rectangle = {area.x+width-112*d, area.y+height-34*d, 100*d, 26*d};
	if p.focus == available+3
	{
		rl.DrawRectangleLinesEx(done, 2*d, {113, 173, 230, 255});
	}
	if rl.GuiButton(done, "Done") || activate == available+3
	{
		ui.dropdown = 0;
		ui.text_field = 10;
	}
	p.opened = .Unavailable;
	rl.GuiUnlock();
}
