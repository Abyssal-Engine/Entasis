package physics_viewer

import "core:fmt"
import rl "vendor:raylib"
import scene "../physics_scene"
import scenarios "../physics_scenarios"
import report "../benchmark_report"

Scene_Drag :: enum
{
	None, Look, Orbit, Pan,
}
Input_Owner :: enum
{
	Scene, List, Form, Text, Dropdown, Modal,
}
UI :: struct
{
	selection_action: Selection_Action,
	recordings: Recording_List,
	result_labels: Result_Labels,
	category, example_kind: i32,
	catalog_selected, catalog_scroll, catalog_focus: [2]i32,
	status: scene.Status,
	results: scene.Availability,
	result_lane, result_sample, result_column: i32,
	result_status: report.Report_Status,
	result_diagnostic: [512]u8,
	runs: Run_List,
	visible_runs: []int,
	recording_filter: Recording_Filter,
	recording_inventory: Recording_Inventory,
	recording_delete: Recording_Delete,
	recording_message: [512]u8,
	run_selected, run_scroll, run_focus: i32,
	run_pressed: int,
	filter: View_Filter,
	dpi: f32,
	font: rl.Font,
	body: rl.Rectangle,
	images: [2]rl.Rectangle,
	image_count: int,
	comparison_scroll: int,
	hovered_pane, focused_pane, drag_pane: int,
	drag: Scene_Drag,
	orbit_pivot: rl.Vector3,
	input: Input_Owner,
	text_field, dropdown: i32,
	choice: Choice_Popup,
	workers: Worker_Chooser,
	edit: Text_Edit,
	drag_button: rl.MouseButton,
	keyboard, fit: scene.Availability,
}

ui_destroy :: proc(ui: ^UI)
{
	recordings_close(&ui.recordings);
	result_labels_close(&ui.result_labels);
	recording_inventory_close(&ui.recording_inventory);
	recording_delete_close(&ui.recording_delete);
	delete(ui.visible_runs);
	results_paths_close(&ui.runs);
	if ui.font.texture.id != 0
	{
		rl.GuiSetFont(rl.GetFontDefault());
		rl.UnloadFont(ui.font);
	}
	ui^ = {};
}

ui_style :: proc(ui: ^UI) -> scene.Status
{
	dpi: f32 = rl.GetWindowScaleDPI().x;
	if ui.dpi == dpi
	{
		return .Ok;
	}
	font: rl.Font = rl.LoadFontEx("tools/physics_viewer/assets/DejaVuSans.ttf", i32(18*dpi), nil, 0);
	if font.texture.id == 0
	{
		return .File_Error;
	}
	rl.GuiSetFont(rl.GetFontDefault());
	if ui.font.texture.id != 0
	{
		rl.UnloadFont(ui.font);
	}
	ui.font, ui.dpi = font, dpi;
	rl.SetTextureFilter(font.texture, .BILINEAR);
	rl.GuiLoadStyleDefault();
	rl.GuiSetFont(font);
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiDefaultProperty.TEXT_SIZE), i32(16*dpi));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.BASE_COLOR_NORMAL), i32(0x343941ff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.TEXT_COLOR_NORMAL), transmute(i32)u32(0xdfe3e9ff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.BORDER_COLOR_NORMAL), i32(0x68727dff));
	rl.GuiSetStyle(.LISTVIEW, i32(rl.GuiListViewProperty.LIST_ITEMS_HEIGHT), i32(28*dpi));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiDefaultProperty.BACKGROUND_COLOR), i32(0x27292dff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.BASE_COLOR_FOCUSED), i32(0x465362ff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.TEXT_COLOR_FOCUSED), transmute(i32)u32(0xf5f6f7ff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.BASE_COLOR_PRESSED), i32(0x526980ff));
	rl.GuiSetStyle(.DEFAULT, i32(rl.GuiControlProperty.TEXT_COLOR_PRESSED), transmute(i32)u32(0xffffffff));
	rl.GuiSetStyle(.LISTVIEW, i32(rl.GuiControlProperty.TEXT_ALIGNMENT), i32(rl.GuiTextAlignment.TEXT_ALIGN_LEFT));

	return .Ok;
}

ui_layout :: proc(v: ^Viewer)
{
	ui: ^UI = &v.ui;
	style_status: scene.Status = ui_style(ui);
	if style_status != .Ok
	{
		ui.status = style_status;
	}
	d: f32 = ui.dpi;
	ui.body = {244*d, 90*d, max(1, f32(rl.GetScreenWidth())-256*d), max(1, f32(rl.GetScreenHeight())-192*d)};
	ui.image_count, ui.hovered_pane = 0, -1;
	if rl.IsMouseButtonPressed(.LEFT) && ui.text_field != 0 && !rl.CheckCollisionPointRec(rl.GetMousePosition(), ui.edit.bounds)
	{
		ui.text_field, ui.edit.active = 0, 0;
	}
	ui.input = .List;
	if ui.recording_delete.phase != .Closed || ui.selection_action != .None
	{
		ui.input = .Modal;
	}
	else if v.launch.state != .Closed && v.launch.state != .Editing
	{
		ui.input = .Modal;
	}
	else if ui.text_field != 0
	{
		ui.input = .Text;
	}
	else if ui.dropdown != 0
	{
		ui.input = .Dropdown;
	}
	else if v.launch.state == .Editing
	{
		ui.input = .Form;
	}
	if v.count > 0
	{
		viewport_layout(v);
	}
	if rl.IsMouseButtonPressed(.LEFT) || ui.input != .List || ui.category != 0 ||
		ui.results == .Available || v.count != 0 || !rl.IsWindowFocused() ||
		(!rl.IsMouseButtonDown(.LEFT) && !rl.IsMouseButtonReleased(.LEFT))
	{
		ui.run_pressed = 0;
	}
	ui.keyboard = .Available if rl.IsWindowFocused() && v.count > 0 &&
		(ui.input == .Scene || (ui.input == .List && rl.GetMousePosition().x >= ui.body.x)) else .Unavailable;
}

ui_catalog :: proc(v: ^Viewer)
{
	d: f32 = v.ui.dpi;
	tab: int = int(v.ui.category);
	ui_text({12*d, 52*d, 216*d, 28*d}, "Cases");
	if tab == 0
	{
		names: [len(BENCHMARKS)]cstring;
		indices: [len(BENCHMARKS)]i32;
		count: i32;
		selected: i32 = -1;
		for definition, index in BENCHMARKS
		{
			names[count] = definition.label;
			indices[count] = i32(index);
			if i32(index) == v.launch.package_index
			{
				selected = count;
			}
			count += 1;
		}
		previous: i32 = v.launch.package_index;
		bounds: rl.Rectangle = {12*d, 90*d, 216*d, max(40*d, f32(rl.GetScreenHeight())-156*d)};
		rl.BeginScissorMode(i32(bounds.x), i32(bounds.y), i32(bounds.width), i32(bounds.height));
		rl.GuiListViewEx(bounds, raw_data(names[:]), count,
			&v.ui.catalog_scroll[0], &selected, &v.ui.catalog_focus[0]);
		rl.EndScissorMode();
		if v.launch.state == .Editing && !rl.GuiIsLocked() && rl.IsWindowFocused() && v.ui.text_field == 0 &&
			rl.CheckCollisionPointRec(rl.GetMousePosition(), bounds)
		{
			direction: i32;
			if rl.IsKeyPressed(.UP) || rl.IsKeyPressedRepeat(.UP)
			{
				direction = -1;
			}
			if rl.IsKeyPressed(.DOWN) || rl.IsKeyPressedRepeat(.DOWN)
			{
				direction = 1;
			}
			if direction != 0
			{
				selected = clamp(selected+direction, 0, count-1);
				v.ui.catalog_focus[0] = selected;
				row_height: i32 = rl.GuiGetStyle(.LISTVIEW, i32(rl.GuiListViewProperty.LIST_ITEMS_HEIGHT))+
					rl.GuiGetStyle(.LISTVIEW, i32(rl.GuiListViewProperty.LIST_ITEMS_SPACING));
				rows: i32 = max(1, i32(bounds.height)/row_height);
				v.ui.catalog_scroll[0] = clamp(v.ui.catalog_scroll[0], max(0, selected-rows+1), selected);
			}
		}
		if selected >= 0 && selected < count && previous != indices[selected]
		{
			launch_select_case(v, indices[selected]);
		}
		if rl.GuiButton({12*d, f32(rl.GetScreenHeight())-54*d, 216*d, 30*d}, "Run new") && v.launch.state != .Editing
		{
			launch_open(v);
		}
		return;
	}
	catalog: [scenarios.Scenario]scenarios.Descriptor = scenarios.CATALOG;
	names: [len(scenarios.CATALOG)]cstring;
	name_storage: [len(scenarios.CATALOG)][64]u8;
	matching: [len(scenarios.CATALOG)]scenarios.Scenario;
	count: i32;
	for entry, index in catalog
	{
		if tab == 1 && (entry.presentation == .Correctness || i32(entry.presentation) != v.ui.example_kind)
		{
			continue;
		}
		assert(len(entry.title) < len(name_storage[count]));
		copy(name_storage[count][:], entry.title);
		names[count] = cast(cstring)raw_data(name_storage[count][:]);
		matching[count] = scenarios.Scenario(index);
		count += 1;
	}
	list_y: f32 = 90*d;
	if tab == 1
	{
		previous_kind: i32 = v.ui.example_kind;
		ui_choice_static(&v.ui, 1, {12*d, list_y, 216*d, 28*d}, []cstring{"Motion", "Diagnostic stepping"}, &v.ui.example_kind);
		if previous_kind != v.ui.example_kind
		{
			close_content(v);
			v.ui.catalog_selected[tab] = -1;
			return;
		}
		list_y += 38*d;
	}
	previous: i32 = v.ui.catalog_selected[tab];
	rl.BeginScissorMode(i32(12*d), i32(list_y), i32(216*d), i32(max(40*d, f32(rl.GetScreenHeight())-list_y-66*d)));
	rl.GuiListViewEx({12*d, list_y, 216*d, max(40*d, f32(rl.GetScreenHeight())-list_y-66*d)}, raw_data(names[:]), count,
		&v.ui.catalog_scroll[tab], &v.ui.catalog_selected[tab], &v.ui.catalog_focus[tab]);
	rl.EndScissorMode();
	if v.ui.catalog_selected[tab] < 0
	{
		v.ui.catalog_selected[tab] = previous;
	}
	selected: i32 = v.ui.catalog_selected[tab];
	if selected >= 0 && selected < count && (selected != previous || v.count == 0)
	{
		recipe: scenarios.Recipe;
		recipe, v.ui.status = scenarios.recipe_admit(matching[selected], nil);
		if v.ui.status == .Ok
		{
			v.ui.status = open_live(v, recipe);
		}
	}

}

ui_workspace :: proc(v: ^Viewer)
{
	ui: ^UI = &v.ui;
	d: f32 = ui.dpi;
	if v.launch.state == .Preparing || v.launch.state == .Running || v.launch.state == .Finished
	{
		ui_benchmark_run(v, {12*d, 10*d, f32(rl.GetScreenWidth())-24*d, f32(rl.GetScreenHeight())-20*d});
		return;
	}
	if ui.recording_delete.phase != .Closed || ui.selection_action != .None || ui.dropdown != 0
	{
		rl.GuiLock();
	}
	previous: i32 = ui.category;
	tabs: [2]cstring = {"Benchmarks", "Examples"};
	for tab, index in tabs
	{
		selected: bool = ui.category == i32(index);
		rl.GuiToggle({12*d+f32(index)*168*d, 10*d, 160*d, 30*d}, tab, &selected);
		if selected
		{
			ui.category = i32(index);
		}
	}
	if previous != ui.category
	{
		if v.launch.state == .Editing
		{
			v.launch.diagnostic = {};
			ui.status = .Ok;
		}
		close_content(v);
		v.launch.state = .Closed;
		ui.results = .Unavailable;
	}
	if ui.category == 0 && rl.GuiButton({f32(rl.GetScreenWidth())-256*d, 10*d, 244*d, 30*d}, "Delete all recordings...")
	{
		recording_delete_preview(v, .Workspace);
		rl.GuiLock();
	}
	ui_catalog(v);
	if v.launch.state == .Editing
	{
		ui_launch(v, {ui.body.x, 52*d, ui.body.width, f32(rl.GetScreenHeight())-80*d});
	}
	else if v.count == 0
	{
		if ui.results == .Available
		{
			ui_results(v, ui.body);
		}
		else
		{
			ui_saved_runs(v, ui.body);
		}
	}
	else
	{
		if rl.GuiButton({244*d, 52*d, 64*d, 28*d}, "Back")
		{
			close_content(v);
			ui.catalog_selected[1] = -1;
		}
		if rl.GuiButton({316*d, 52*d, 64*d, 28*d}, "Fit")
		{
			ui.fit = .Available;
		}
		if v.mode == .Replay && rl.GuiButton({388*d, 52*d, 92*d, 28*d}, "Compare")
		{
			v.ui.status = recordings_refresh(v, v.panes[v.active_pane].reader.metadata.scenario);
			ui.selection_action = .Compare;
			v.cursor.playback, v.cursor.accumulator = .Paused, 0;
		}
		if v.count == 2
		{
			ui_comparison(v);
		}
		if v.count > 0
		{
			ui.status = ui_transport(v, {244*d, f32(rl.GetScreenHeight())-94*d, ui.body.width, 64*d});

		}
	}
	if v.count > 0 && ui.status == .Ok
	{
		ui_text({244*d, f32(rl.GetScreenHeight())-28*d, ui.body.width, 24*d}, "WASD/QE move | RMB look | Alt+LMB orbit | MMB pan | Wheel zoom | F fit | Space play/pause" if ui.body.width >= 900*d else "RMB look | Alt+LMB orbit | MMB pan | WASD/QE move");
	}
	if ui.status != .Ok
	{
		ui_text({244*d, f32(rl.GetScreenHeight())-28*d, ui.body.width, 24*d}, "%v", ui.status);
	}
	rl.GuiUnlock();
	ui_recording_dialog(v);
	ui_recording_delete(v);
	rl.GuiUnlock();
	if ui.dropdown > 0
	{
		ui_choice_popup(ui);
	}
	ui_worker_popup(v);
	if v.count == 2 && ui.selection_action == .None && ui.recording_delete.phase == .Closed && ui.dropdown == 0 && v.launch.state != .Editing
	{
		ui_comparison_metadata(v);
	}
}

ui_name :: proc(name: string, value: []u8) -> string
{
	assert(len(value) >= len(name));
	for byte, index in transmute([]u8)name
	{
		value[index] = ' ' if byte == '_' || byte == '-' else byte;
	}
	if len(value) > 0 && value[0] >= 'a' && value[0] <= 'z'
	{
		value[0] -= 'a'-'A';
	}
	return string(value[:len(name)]);
}

ui_heading :: proc(ui: ^UI, bounds: rl.Rectangle, text: string, args: ..any)
{
	rl.BeginScissorMode(i32(bounds.x), i32(bounds.y), i32(bounds.width), i32(bounds.height));
	out: Text_Output = {font=ui.font, position={bounds.x, bounds.y+2*ui.dpi}, origin={bounds.x, bounds.y+2*ui.dpi}, size=24*ui.dpi, spacing=ui.dpi, color={235, 239, 244, 255}, mode=.Draw};
	if len(args) == 0
	{
		fmt.wprintf({procedure=ui_text_write, data=&out}, "%s", text);
	}
	else
	{
		fmt.wprintf({procedure=ui_text_write, data=&out}, text, ..args);
	}
	rl.EndScissorMode();
}

ui_cell :: proc(ui: ^UI, bounds: rl.Rectangle, text: string, args: ..any)
{
	rl.BeginScissorMode(i32(bounds.x), i32(bounds.y), i32(max(1, bounds.width)), i32(bounds.height));
	if len(args) == 0
	{
		ui_text(bounds, "%s", text);
	}
	else
	{
		ui_text(bounds, text, ..args);
	}
	rl.EndScissorMode();
}
