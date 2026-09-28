package physics_viewer

import "core:strings"
import rl "vendor:raylib"
import scene "../physics_scene"

Choice_Popup :: struct
{
	bounds: rl.Rectangle,
	labels: []cstring,
	fixed_labels: [6]cstring,
	selected, focus, scroll: i32,
	result_id, result: i32,
	opened: scene.Availability,
}
Edit_Modifier :: enum
{
	Shift, Control,
}
Text_Edit :: struct
{
	active, cursor, anchor: i32,
	bounds: rl.Rectangle,
	original: [128]u8,
	offset: f32,
	selecting: scene.Availability,
	last_click: f64,
}

ui_choice_static :: proc(ui: ^UI, id: i32, bounds: rl.Rectangle, labels: []cstring, selected: ^i32)
{
	ui_choice(ui, id, bounds, labels, selected);
	if ui.dropdown == id
	{
		assert(len(labels) <= len(ui.choice.fixed_labels));
		copy(ui.choice.fixed_labels[:], labels);
		ui.choice.labels = ui.choice.fixed_labels[:len(labels)];
	}
}

ui_choice :: proc(ui: ^UI, id: i32, bounds: rl.Rectangle, labels: []cstring, selected: ^i32)
{
	if ui.choice.result_id == id
	{
		selected^ = ui.choice.result;
		ui.choice.result_id = 0;
	}
	text: cstring = labels[selected^];
	if rl.GuiButton(bounds, text)
	{
		ui.dropdown = id;
		ui.text_field = 0;
		ui.choice.selected, ui.choice.focus = -1, selected^;
		ui.choice.scroll = max(0, selected^-3);
		ui.choice.opened = .Available;
		rl.GuiLock();
	}
	ui_text({bounds.x+bounds.width-20*ui.dpi, bounds.y, 16*ui.dpi, bounds.height}, "v");
	if ui.dropdown == id
	{
		ui.choice.bounds, ui.choice.labels = bounds, labels;
	}
}

ui_choice_popup :: proc(ui: ^UI)
{
	if ui.dropdown == 0 || len(ui.choice.labels) == 0
	{
		return;
	}
	p: ^Choice_Popup = &ui.choice;
	d: f32 = ui.dpi;
	height: f32 = min(f32(len(p.labels))*30*d+4*d, f32(rl.GetScreenHeight())*0.55);
	bounds: rl.Rectangle = {p.bounds.x, p.bounds.y+p.bounds.height+2*d, p.bounds.width, height};
	if bounds.y+height > f32(rl.GetScreenHeight())-8*d
	{
		bounds.y = max(8*d, p.bounds.y-height-2*d);
	}
	if rl.IsKeyPressed(.ESCAPE) || (p.opened == .Unavailable && rl.IsMouseButtonPressed(.LEFT) && !rl.CheckCollisionPointRec(rl.GetMousePosition(), bounds))
	{
		ui.dropdown = 0;
		return;
	}
	if rl.IsKeyPressed(.DOWN)
	{
		p.focus = min(i32(len(p.labels)-1), p.focus+1);
	}
	if rl.IsKeyPressed(.UP)
	{
		p.focus = max(0, p.focus-1);
	}
	if rl.IsKeyPressed(.DOWN) || rl.IsKeyPressed(.UP)
	{
		p.scroll = clamp(p.scroll, p.focus-i32(height/(30*d))+2, p.focus);
		p.scroll = max(0, p.scroll);
	}
	rl.DrawRectangleRec(bounds, BACKGROUND);
	rl.GuiListViewEx(bounds, raw_data(p.labels), i32(len(p.labels)), &p.scroll, &p.selected, &p.focus);
	if rl.IsKeyPressed(.ENTER)
	{
		p.selected = p.focus;
	}
	if p.selected >= 0
	{
		p.result_id, p.result = ui.dropdown, p.selected;
		ui.dropdown = 0;
	}
	p.opened = .Unavailable;
}

ui_edit_replace :: proc(edit: ^Text_Edit, buffer: []u8, text: string)
{
	length: int = len(strings.string_from_null_terminated_ptr(raw_data(buffer), len(buffer)));
	first, last: int = int(min(edit.cursor, edit.anchor)), int(max(edit.cursor, edit.anchor));
	amount: int = min(len(text), len(buffer)-1-(length-(last-first)));
	copy(buffer[first+amount:], buffer[last:length+1]);
	copy(buffer[first:first+amount], text[:amount]);
	edit.cursor, edit.anchor = i32(first+amount), i32(first+amount);
}

ui_edit :: proc(ui: ^UI, id: i32, bounds: rl.Rectangle, buffer: []u8)
{
	edit: ^Text_Edit = &ui.edit;
	text: string = strings.string_from_null_terminated_ptr(raw_data(buffer), len(buffer));
	size: f32 = 16*ui.dpi;
	mouse: rl.Vector2 = rl.GetMousePosition();
	if !rl.GuiIsLocked() && ui.dropdown == 0 && rl.IsMouseButtonPressed(.LEFT) && rl.CheckCollisionPointRec(mouse, bounds)
	{
		ui.text_field = id;
	}
	if !rl.GuiIsLocked() && ui.text_field == id && edit.active != id
	{
		edit.active = id;
		edit.cursor, edit.anchor = i32(len(text)), 0;
		edit.original = {};
		copy(edit.original[:], buffer);
		edit.offset = 0;
	}
	if !rl.GuiIsLocked() && ui.text_field == id
	{
		edit.bounds = bounds;
		modifiers: bit_set[Edit_Modifier; u8];
		if rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT)
		{
			modifiers += {.Shift};
		}
		if rl.IsKeyDown(.LEFT_CONTROL) || rl.IsKeyDown(.RIGHT_CONTROL)
		{
			modifiers += {.Control};
		}
		if rl.IsMouseButtonPressed(.LEFT) && rl.CheckCollisionPointRec(mouse, bounds)
		{
			edit.selecting = .Available;
			if rl.GetTime()-edit.last_click < 0.3
			{
				edit.anchor, edit.cursor = 0, i32(len(text));
				edit.selecting = .Unavailable;
			}
			edit.last_click = rl.GetTime();
		}
		if !rl.IsMouseButtonDown(.LEFT)
		{
			edit.selecting = .Unavailable;
		}
		if edit.selecting == .Available
		{
			edit.cursor = 0;
			for index in 1 ..= len(text)
			{
				width: f32 = ui_measure_text(ui.font, text[:index], size, 1);
				if width > mouse.x-bounds.x-6*ui.dpi+edit.offset
				{
					break;
				}
				edit.cursor = i32(index);
			}
			if rl.IsMouseButtonPressed(.LEFT) && .Shift not_in modifiers
			{
				edit.anchor = edit.cursor;
			}
		}
		if .Control in modifiers && rl.IsKeyPressed(.A)
		{
			edit.anchor, edit.cursor = 0, i32(len(text));
		}
		if .Control in modifiers && (rl.IsKeyPressed(.C) || rl.IsKeyPressed(.X)) && edit.cursor != edit.anchor
		{
			selection: string = text[min(edit.cursor, edit.anchor):max(edit.cursor, edit.anchor)];
			clipboard: [128]u8;
			copy(clipboard[:127], selection);
			rl.SetClipboardText(cast(cstring)raw_data(clipboard[:]));
			if rl.IsKeyPressed(.X)
			{
				ui_edit_replace(edit, buffer, "");
			}
		}
		if .Control in modifiers && rl.IsKeyPressed(.V)
		{
			paste: string = string(rl.GetClipboardText());
			for c in paste
			{
				if c >= 32 && c <= 126
				{
					character: [1]u8 = {u8(c)};
					ui_edit_replace(edit, buffer, string(character[:]));
				}
			}
		}
		text = strings.string_from_null_terminated_ptr(raw_data(buffer), len(buffer));
		if rl.IsKeyPressed(.LEFT) || rl.IsKeyPressedRepeat(.LEFT)
		{
			edit.cursor = min(edit.cursor, edit.anchor) if .Shift not_in modifiers && edit.cursor != edit.anchor else max(0, edit.cursor-1);
			if .Shift not_in modifiers
			{
				edit.anchor = edit.cursor;
			}
		}
		if rl.IsKeyPressed(.RIGHT) || rl.IsKeyPressedRepeat(.RIGHT)
		{
			edit.cursor = max(edit.cursor, edit.anchor) if .Shift not_in modifiers && edit.cursor != edit.anchor else min(i32(len(text)), edit.cursor+1);
			if .Shift not_in modifiers
			{
				edit.anchor = edit.cursor;
			}
		}
		if rl.IsKeyPressed(.HOME) || rl.IsKeyPressed(.END)
		{
			edit.cursor = 0 if rl.IsKeyPressed(.HOME) else i32(len(text));
			if .Shift not_in modifiers
			{
				edit.anchor = edit.cursor;
			}
		}
		if rl.IsKeyPressed(.BACKSPACE) || rl.IsKeyPressedRepeat(.BACKSPACE)
		{
			if edit.cursor == edit.anchor
			{
				edit.anchor = max(0, edit.cursor-1);
			}
			ui_edit_replace(edit, buffer, "");
		}
		if rl.IsKeyPressed(.DELETE) || rl.IsKeyPressedRepeat(.DELETE)
		{
			if edit.cursor == edit.anchor
			{
				edit.anchor = min(i32(len(text)), edit.cursor+1);
			}
			ui_edit_replace(edit, buffer, "");
		}
		for c: rune = rl.GetCharPressed(); c != 0; c = rl.GetCharPressed()
		{
			if .Control not_in modifiers && c >= 32 && c <= 126
			{
				character: [1]u8 = {u8(c)};
				ui_edit_replace(edit, buffer, string(character[:]));
			}
		}
		if .Control in modifiers && rl.IsKeyPressed(.Z)
		{
			copy(buffer, edit.original[:]);
			text = strings.string_from_null_terminated_ptr(raw_data(buffer), len(buffer));
			edit.cursor, edit.anchor = i32(len(text)), 0;
		}
		if rl.IsKeyPressed(.ENTER) || rl.IsKeyPressed(.ESCAPE)
		{
			if rl.IsKeyPressed(.ESCAPE)
			{
				copy(buffer, edit.original[:]);
			}
			ui.text_field, edit.active = 0, 0;
		}
	}
	text = strings.string_from_null_terminated_ptr(raw_data(buffer), len(buffer));
	rl.GuiTextBox(bounds, "", 1, false);
	first, last: i32 = min(edit.cursor, edit.anchor), max(edit.cursor, edit.anchor);
	offset: f32;
	if ui.text_field == id
	{
		cursor: f32 = ui_measure_text(ui.font, text[:edit.cursor], size, 1);
		edit.offset = max(0, clamp(edit.offset, cursor-bounds.width+18*ui.dpi, cursor));
		offset = edit.offset;
		rl.DrawRectangleLinesEx(bounds, 2*ui.dpi, {113, 173, 230, 255});
	}
	rl.BeginScissorMode(i32(bounds.x+4*ui.dpi), i32(bounds.y+2*ui.dpi), i32(bounds.width-8*ui.dpi), i32(bounds.height-4*ui.dpi));
	origin: rl.Vector2 = {bounds.x+6*ui.dpi-offset, bounds.y+(bounds.height-size)/2};
	if ui.text_field == id
	{
		left: f32 = ui_measure_text(ui.font, text[:first], size, 1);
		right: f32 = ui_measure_text(ui.font, text[:last], size, 1);
		rl.DrawRectangleRec({origin.x+left, origin.y, right-left, size}, {61, 102, 144, 255});
		cursor: f32 = ui_measure_text(ui.font, text[:edit.cursor], size, 1);
		rl.DrawLineEx({origin.x+cursor, origin.y}, {origin.x+cursor, origin.y+size}, ui.dpi, rl.WHITE);
	}
	ui_draw_text(ui.font, text, origin, size, 1, {223, 227, 233, 255});
	rl.EndScissorMode();
}
