package physics_viewer

import "core:fmt"
import "core:strings"
import "core:strconv"
import rl "vendor:raylib"
import support "../../benchmarks/benchmark_support"
import scene "../physics_scene"

POWERSHELL :: #config(ENTASIS_VIEWER_POWERSHELL, "pwsh");
Benchmark_Capability :: enum
{
	Workload_Settings,
}
Benchmark_Definition :: struct
{
	package_name: string,
	label: cstring,
	capabilities: bit_set[Benchmark_Capability; u8],
}
when ODIN_OS == .Windows
{
	BENCHMARKS :: [?]Benchmark_Definition{
		{"awakening_duplicate_sets", "Awakening duplicate sets", {}},
		{"container", "Container", {.Workload_Settings}},
		{"contact_churn_grid_4k", "Contact churn grid 4k", {}},
		{"noncontact_constraint_mix", "Noncontact constraint mix", {}},
		{"noncontact_fallback_smoke", "Noncontact fallback smoke", {}},
		{"shape_mixed_bounds", "Shape mixed bounds", {}},
		{"spatial_query_trace", "Spatial query trace", {}},
		{"spatial_query_batch", "Spatial query batch", {}},
		{"custom_extensions", "Custom extensions", {}},
		{"query_extensions", "Query extensions", {}},
		{"world_lifecycle", "World lifecycle", {}},
		{"contact_islands", "Contact islands", {.Workload_Settings}},
		{"ragdoll_stair_tumble", "Ragdoll stair tumble", {}},
		{"pyramid", "Pyramid", {.Workload_Settings}},
	};
}
else
{
	BENCHMARKS :: [?]Benchmark_Definition{
		{"awakening_duplicate_sets", "Awakening duplicate sets", {}},
		{"container", "Container", {.Workload_Settings}},
		{"contact_churn_grid_4k", "Contact churn grid 4k", {}},
		{"noncontact_constraint_mix", "Noncontact constraint mix", {}},
		{"noncontact_fallback_smoke", "Noncontact fallback smoke", {}},
		{"shape_mixed_bounds", "Shape mixed bounds", {}},
		{"spatial_query_trace", "Spatial query trace", {}},
		{"spatial_query_batch", "Spatial query batch", {}},
		{"contact_islands", "Contact islands", {.Workload_Settings}},
		{"ragdoll_stair_tumble", "Ragdoll stair tumble", {}},
		{"pyramid", "Pyramid", {.Workload_Settings}},
	};
}
Launch_State :: enum
{
	Closed, Editing, Requested, Preparing, Running, Finished,
}
Benchmark_Launch :: struct
{
	state: Launch_State,
	package_index: i32,
	workers: Worker_Counts,
	available_workers: int,
	repeats: [16]u8,
	variant: [128]u8,
	shape: i32,
	record: support.Recording_Mode,
	components: i32,
	options: support.Options,
	scroll: rl.Vector2,
	fields: [26][128]u8,
	diagnostic: [256]u8,
	missing_settings: scene.Availability,
}
Launch_Operation :: enum
{
	Benchmark, Build_And_Benchmark,
}
Launch_Phase :: enum
{
	Run, Build,
}
Launch_Request :: struct
{
	arguments, run_arguments: [dynamic]string,
	operation: Launch_Operation,
	phase: Launch_Phase,
	package_index: i32,
}

launch_open :: proc(v: ^Viewer)
{
	form: ^Benchmark_Launch = &v.launch;
	package_name: string = benchmark_package(form.package_index);
	form.state = .Editing;
	if form.variant[0] == 0
	{
		copy(form.variant[:], "viewer");
	}
	form.missing_settings = .Unavailable;
	form.record = .Off;
	form.available_workers = min(64, launch_available_workers());
	if form.repeats[0] == 0
	{
		form.workers = {1};
		copy(form.repeats[:], "1");

	}
	if form.fields[5][0] == 0 && (package_name == "container" || package_name == "contact_islands" || package_name == "pyramid")
	{
		v.ui.status = launch_fill_missing(form);
		form.diagnostic = {};
	}
	form.scroll = {};
	v.ui.text_field, v.ui.edit.active, v.ui.dropdown = 0, 0, 0;
	v.cursor.playback, v.cursor.accumulator = .Paused, 0;
}

launch_admit :: proc(form: ^Benchmark_Launch, operation: Launch_Operation) -> (Launch_Request, scene.Status)
{
	if form.missing_settings == .Available
	{
		return {}, .Invalid_Data;
	}
	worker_buffer: [192]u8;
	workers: string = worker_arguments(form.workers, worker_buffer[:]);
	repeats_text: string = strings.string_from_null_terminated_ptr(raw_data(form.repeats[:]), 16);
	repeats: int;
	repeats_status: support.Admission;
	repeats, repeats_status = support.parse_integer(repeats_text);
	if repeats_status != .Ok || repeats < 1 || repeats > 2147483647
	{
		return {}, .Invalid_Data;
	}
	available_workers: int = launch_available_workers();
	if available_workers < 1
	{
		copy(form.diagnostic[:255], "Cannot determine available CPUs");
		return {}, .Invalid_Data;
	}
	available_workers = min(available_workers, 64);
	if worker_count(form.workers) == 0
	{
		copy(form.diagnostic[:255], "Select at least one thread count");
		return {}, .Invalid_Data;
	}
	first_worker: int;
	for worker in 1 ..= 64
	{
		if worker not_in form.workers
		{
			continue;
		}
		if worker > available_workers
		{
			fmt.bprintf(form.diagnostic[:255], "Thread counts must be between 1 and %d available CPUs", available_workers);
			return {}, .Invalid_Data;
		}
		if first_worker == 0
		{
			first_worker = worker;
		}
	}
	package_name: string = benchmark_package(form.package_index);
	request: Launch_Request = {package_index=form.package_index, operation=operation};
	status: scene.Status = .Invalid_Data;
	defer if status != .Ok
	{
		launch_request_close(&request);
	}
	when ODIN_OS == .Windows
	{
		append(&request.arguments, strings.clone(POWERSHELL), strings.clone("-NoProfile"), strings.clone("-ExecutionPolicy"), strings.clone("Bypass"),
			strings.clone("-File"), strings.clone("scripts/windows/benchmarks/run_benchmark.ps1"));
	}
	else
	{
		append(&request.arguments, strings.clone("bash"), strings.clone("scripts/linux/benchmarks/run_benchmark.sh"));
	}
	launch_argument(&request, "Package", "package", package_name);
	launch_argument(&request, "Configuration", "configuration", "release");
	launch_argument(&request, "Record", "record", "on" if form.record == .On && support.recording_supported(package_name) == .Ok else "off");
	when ODIN_OS == .Windows
	{
		if package_name == "contact_islands" || package_name == "query_extensions" || package_name == "custom_extensions"
		{
			launch_argument(&request, "Components", "components", "all" if form.components == 0 else "common");
		}
	}
	if operation == .Build_And_Benchmark
	{
		for argument, index in request.arguments
		{
			value: string = argument;
			when ODIN_OS == .Windows
			{
				if index == 5
				{
					value = "scripts/windows/benchmarks/build_benchmarks.ps1";
				}
			}
			else
			{
				if index == 1
				{
					value = "scripts/linux/benchmarks/build_benchmarks.sh";
				}
			}
			append(&request.run_arguments, strings.clone(value));
		}
	}
	launch_argument(&request, "Workers", "workers", workers);
	launch_argument(&request, "Runs", "runs", repeats_text);
	if package_name == "container" || package_name == "contact_islands" || package_name == "pyramid"
	{
		variant: string = strings.string_from_null_terminated_ptr(raw_data(form.variant[:]), 128);
		if len(variant) == 0 || variant == "." || variant == ".."
		{
			return {}, status;
		}
		for c in variant
		{
			if !(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') && c != '_' && c != '-' && c != '.'
			{
				return {}, status;
			}
		}
		shapes: [5]string = {"box", "sphere", "capsule", "cylinder", "hull"};
		launch_argument(&request, "Shape", "shape", shapes[form.shape]);
		launch_argument(&request, "Variant", "variant", variant);
		argument_storage: [29][160]u8;
		arguments: [29]string;
		arguments[0] = fmt.bprintf(argument_storage[0][:], "--shape=%s", shapes[form.shape]);
		arguments[1] = fmt.bprintf(argument_storage[1][:], "--worker-count=%d", first_worker);
		arguments[2] = "--output=preview.csv";
		argument_count: int = 3;
		option_names: [len(support.OPTION_NAMES)]string = support.OPTION_NAMES;
		for &field, index in form.fields
		{
			value: string = strings.string_from_null_terminated_ptr(raw_data(field[:]), 128);
			if len(value) == 0 || index == 0
			{
				continue;
			}
			name: string = option_names[index];
			arguments[argument_count] = fmt.bprintf(argument_storage[argument_count][:], "--%s=%s", name, value);
			argument_count += 1;
			windows_name: [32]u8;
			name_count: int;
			for byte in transmute([]u8)name
			{
				if byte != '-'
				{
					windows_name[name_count] = byte;
					name_count += 1;
				}
			}
			launch_argument(&request, string(windows_name[:name_count]), name, value);
		}
		workload: support.Workload = .Container if package_name == "container" else .Contact_Islands if package_name == "contact_islands" else .Pyramid;
		diagnostic: string;
		form.options, diagnostic = support.parse_workload_options(workload, arguments[:argument_count]);
		if len(diagnostic) > 0
		{
			copy(form.diagnostic[:255], diagnostic);
			return {}, status;
		}
	}
	if operation == .Build_And_Benchmark
	{
		request.arguments, request.run_arguments = request.run_arguments, request.arguments;
		request.phase = .Build;
	}
	status = .Ok;
	return request, status;
}

launch_argument :: proc(request: ^Launch_Request, windows_name, linux_name, value: string)
{
	when ODIN_OS == .Windows
	{
		append(&request.arguments, fmt.aprintf("-%s", windows_name));
	}
	else
	{
		append(&request.arguments, fmt.aprintf("--%s", linux_name));
	}
	append(&request.arguments, strings.clone(value));
}

launch_request_close :: proc(request: ^Launch_Request)
{
	for argument in request.arguments
	{
		delete(argument);
	}
	delete(request.arguments);
	for argument in request.run_arguments
	{
		delete(argument);
	}
	delete(request.run_arguments);
	request^ = {};
}

launch_defaults :: proc(v: ^Viewer)
{
	index: i32 = v.launch.package_index;
	v.ui.status = .Ok;
	v.launch = {package_index=index};
	launch_open(v);
}

launch_select_case :: proc(v: ^Viewer, index: i32)
{
	previous: Benchmark_Launch = v.launch;
	close_content(v);
	v.launch = {package_index=index};
	v.ui.results = .Unavailable;
	v.ui.status = .Ok;
	if previous.state == .Editing
	{
		launch_open(v);
		v.launch.workers, v.launch.repeats = previous.workers, previous.repeats;
		v.launch.record, v.launch.variant = previous.record, previous.variant;
	}
	results_refresh(v);
}

ui_launch :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	form: ^Benchmark_Launch = &v.launch;
	editing_at_start: i32 = v.ui.text_field;
	d: f32 = v.ui.dpi;
	x: f32 = bounds.x+12*d;
	width: f32 = bounds.width-24*d;
	y: f32 = bounds.y+12*d;
	package_name: string = benchmark_package(form.package_index);
	definitions: [len(BENCHMARKS)]Benchmark_Definition = BENCHMARKS;
	capabilities: bit_set[Benchmark_Capability; u8] = definitions[form.package_index].capabilities;
	rl.DrawRectangleRec(bounds, BACKGROUND);
	if v.ui.dropdown != 0
	{
		rl.GuiLock();
	}
	if !rl.GuiIsLocked() && .Workload_Settings not_in capabilities && rl.IsKeyPressed(.TAB) && v.ui.dropdown == 0
	{
		v.ui.text_field = 11 if v.ui.text_field == 10 else 10;
		v.ui.edit.active = 0;
	}
	ui_text({x, y, width-170*d, 28*d}, "Run %s", package_name);
	if rl.GuiButton({x+width-156*d, y, 156*d, 28*d}, "Reset defaults")
	{
		launch_defaults(v);
	}
	y += 40*d;
	column: f32 = (width-24*d)/3;
	ui_text({x, y, column, 22*d}, "Thread counts");
	ui_workers(v, {x, y+24*d, column, 28*d});
	ui_text({x+column+12*d, y, column, 22*d}, "Repeats");
	ui_edit(&v.ui, 11, {x+column+12*d, y+24*d, column, 28*d}, form.repeats[:]);
	if .Workload_Settings in capabilities
	{
		ui_text({x+2*(column+12*d), y, column, 22*d}, "Shape");
		previous: i32 = form.shape;
		ui_choice_static(&v.ui, 50, {x+2*(column+12*d), y+24*d, column, 28*d}, []cstring{"Box", "Sphere", "Capsule", "Cylinder", "Hull"}, &form.shape);
		if previous != form.shape
		{
			form.fields[2] = {};
			v.ui.status = launch_fill_missing(form);
			form.diagnostic = {};
		}
	}
	y += 64*d;
	if support.recording_supported(package_name) == .Ok
	{
		ui_text({x, y, 172*d, 28*d}, "%s", "Save baseline replay" if package_name == "contact_islands" else "Save replay");
		record: i32 = 1 if form.record == .On else 0;
		rl.GuiToggleGroup({x+180*d, y, 52*d, 28*d}, "Off;On", &record);
		form.record = .On if record == 1 else .Off;
		ui_text({x, y+32*d, width, 24*d}, "Saving replay can affect measured timings");
	}
	else
	{
		ui_text({x, y, width, 28*d}, "Numerical results only");
	}
	when ODIN_OS == .Windows
	{
		if package_name == "contact_islands" || package_name == "query_extensions" || package_name == "custom_extensions"
		{
			ui_choice_static(&v.ui, 51, {x+width-220*d, y, 220*d, 28*d}, []cstring{"All components", "Common components"}, &form.components);
		}
	}
	y += 64*d;
	footer: f32 = bounds.y+bounds.height-100*d;
	if .Workload_Settings in capabilities
	{
		ui_launch_fields(v, {x, y, width, max(36*d, footer-y-36*d)});
	}
	else
	{
		ui_text({x, y, width, 28*d}, "This case uses its authored workload defaults");
	}
	if form.missing_settings == .Available
	{
		if rl.GuiButton({x, footer, 320*d, 26*d}, "Use defaults for missing saved values")
		{
			v.ui.status = launch_fill_missing(form);
		}
	}
	else
	{
		ui_text({x, footer, width, 26*d}, "Benchmark uses existing binaries; Build and benchmark compiles source first");
	}
	selected_count: int = worker_count(form.workers);
	if selected_count == 0 || form.available_workers < 1
	{
		ui_text({x, footer-28*d, width, 24*d}, "%s", "Cannot determine available CPUs" if form.available_workers < 1 else "Select at least one thread count");
		rl.GuiDisable();
	}
	else
	{
		repeats: int;
		admission: support.Admission;
		repeats, admission = support.parse_integer(strings.string_from_null_terminated_ptr(raw_data(form.repeats[:]), 16));
		if admission == .Ok && repeats > 0
		{
			ui_text({x, footer-28*d, width, 24*d}, "%d thread count%s, %d repeat%s each", selected_count, "" if selected_count == 1 else "s", repeats, "" if repeats == 1 else "s");
		}
	}
	operation: Launch_Operation = .Benchmark;
	if rl.GuiButton({x, footer+32*d, 188*d, 32*d}, "Build and benchmark")
	{
		operation = .Build_And_Benchmark;
	}
	if rl.GuiButton({x+200*d, footer+32*d, 124*d, 32*d}, "Benchmark") || operation == .Build_And_Benchmark
	{
		form.diagnostic = {};
		request: Launch_Request;
		status: scene.Status;
		request, status = launch_admit(form, operation);
		if status == .Ok
		{
			v.request = request;
			form.state = .Requested;
		}
		else if form.diagnostic[0] == 0
		{
			copy(form.diagnostic[:255], "Check workers (1-64), repeats and workload values");
		}
		v.ui.status = status;
	}
	rl.GuiEnable();
	if rl.GuiButton({x+336*d, footer+32*d, 100*d, 32*d}, "Cancel") ||
		(!rl.GuiIsLocked() && editing_at_start == 0 && v.ui.dropdown == 0 && rl.IsKeyPressed(.ESCAPE))
	{
		form.state = .Closed;
		form.diagnostic = {};
		v.ui.status = .Ok;
		v.ui.text_field, v.ui.dropdown = 0, 0;
	}
	ui_text({x, footer+70*d, width, 26*d}, "%s", strings.string_from_null_terminated_ptr(raw_data(form.diagnostic[:]), 256));
	rl.GuiUnlock();
}

ui_launch_fields :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	form: ^Benchmark_Launch = &v.launch;
	d: f32 = v.ui.dpi;
	labels: [26]string = {"Shape", "Static shape", "Shape size (X, Y, Z)", "Density", "Layout scale", "Steps", "Timestep (Hz)", "Warmup steps", "Velocity iterations", "Substeps", "Sleep", "Grid (X, Y, Z)", "Spacing (X, Y, Z)", "Spawn height", "Container size (X, Y, Z)", "Island grid (X, Z)", "Island spacing (X, Z)", "Floor size (X, Y, Z)", "Rows", "Projectile count", "Launch step", "Projectile radius", "Projectile density", "Projectile center (X, Y, Z)", "Projectile spacing (X, Y, Z)", "Projectile velocity (X, Y, Z)"};
	indices: [25]int;
	count: int;
	package_name: string = benchmark_package(form.package_index);
	for index in 1 ..< 26
	{
		if index <= 10 || (package_name == "container" && index >= 11 && index <= 14) ||
			(package_name == "contact_islands" && (index >= 11 && index <= 13 || index >= 15 && index <= 17)) ||
			(package_name == "pyramid" && index >= 17)
		{
			indices[count] = index;
			count += 1;
		}
	}
	columns: int = 2 if bounds.width >= 600*d else 1;
	column_width: f32 = (bounds.width-24*d)/f32(columns);
	row_height: f32 = 62*d;
	if !rl.GuiIsLocked() && rl.IsKeyPressed(.TAB) && v.ui.dropdown == 0
	{
		order: [28]i32;
		order[0], order[1] = 10, 11;
		order_count: int = 2;
		for index in indices[:count]
		{
			if index != 1 && index != 10
			{
				order[order_count] = i32(100+index);
				order_count += 1;
			}
		}
		current: int = -1;
		for id, index in order[:order_count]
		{
			if id == v.ui.text_field
			{
				current = index;
			}
		}
		direction: int = -1 if rl.IsKeyDown(.LEFT_SHIFT) || rl.IsKeyDown(.RIGHT_SHIFT) else 1;
		v.ui.text_field = order[(current+direction+order_count)%order_count];
		v.ui.edit.active = 0;
		for index, ordinal in indices[:count]
		{
			if i32(100+index) == v.ui.text_field
			{
				form.scroll.y = -f32(ordinal/columns)*row_height;
			}
		}
	}
	view: rl.Rectangle;
	rl.GuiScrollPanel(bounds, nil, {0, 0, bounds.width-18*d, f32((count+columns-1)/columns)*row_height+8*d}, &form.scroll, &view);
	for index, ordinal in indices[:count]
	{
		x: f32 = bounds.x+8*d+f32(ordinal%columns)*column_width;
		y: f32 = bounds.y+8*d+f32(ordinal/columns)*row_height+form.scroll.y;
		if y < view.y || y+52*d > view.y+view.height
		{
			continue;
		}
		ui_text({x, y, column_width-16*d, 22*d}, "%s", labels[index]);
		field: ^[128]u8 = &form.fields[index];
		field_bounds: rl.Rectangle = {x, y+24*d, column_width-16*d, 28*d};
		if index == 1
		{
			shape: support.Shape;
			shape, _ = support.parse_shape(strings.string_from_null_terminated_ptr(raw_data(field[:]), 128));
			selected: i32 = 1 if shape == .Hull else 0;
			ui_choice_static(&v.ui, 52, field_bounds, []cstring{"Box", "Hull"}, &selected);
			names: [2]string = {"box", "hull"};
			field^ = {};
			copy(field[:], names[selected]);
		}
		else if index == 10
		{
			selected: i32 = 1 if field[0] == 'e' else 0;
			ui_choice_static(&v.ui, 53, field_bounds, []cstring{"Disabled", "Enabled"}, &selected);
			field^ = {};
			copy(field[:], "enabled" if selected == 1 else "disabled");
		}
		else
		{
			ui_edit(&v.ui, i32(100+index), field_bounds, field[:]);
		}
	}
}

benchmark_package :: proc(index: i32) -> string
{
	packages: [len(BENCHMARKS)]Benchmark_Definition = BENCHMARKS;
	return packages[index].package_name;
}

launch_fill_missing :: proc(form: ^Benchmark_Launch) -> scene.Status
{
	package_name: string = benchmark_package(form.package_index);
	workload: support.Workload = .Container if package_name == "container" else .Contact_Islands if package_name == "contact_islands" else .Pyramid;
	shapes: [5]string = {"box", "sphere", "capsule", "cylinder", "hull"};
	argument_storage: [28][160]u8;
	arguments: [28]string;
	arguments[0] = fmt.bprintf(argument_storage[0][:], "--shape=%s", shapes[form.shape]);
	arguments[1], arguments[2] = "--worker-count=1", "--output=preview.csv";
	argument_count: int = 3;
	names: [len(support.OPTION_NAMES)]string = support.OPTION_NAMES;
	for &field, index in form.fields
	{
		value: string = strings.string_from_null_terminated_ptr(raw_data(field[:]), 128);
		if index > 0 && len(value) > 0
		{
			arguments[argument_count] = fmt.bprintf(argument_storage[argument_count][:], "--%s=%s", names[index], value);
			argument_count += 1;
		}
	}
	options: support.Options;
	diagnostic: string;
	options, diagnostic = support.parse_workload_options(workload, arguments[:argument_count]);
	if len(diagnostic) > 0
	{
		copy(form.diagnostic[:255], diagnostic);
		return .Invalid_Data;
	}
	buffer: [support.PARAMETER_CAPACITY]u8;
	parameters: string;
	status: support.Admission;
	parameters, status = support.canonical_parameters(options, buffer[:]);
	if status != .Ok
	{
		return .Invalid_Data;
	}
	for field in strings.split_by_byte_iterator(&parameters, ';')
	{
		key, value: string;
		key, value, _ = support.split_once(field, '=');
		for name, index in names[:26]
		{
			if key == name && index > 0 && form.fields[index][0] == 0
			{
				launch_set_field(form, index, value);
			}
		}
	}
	form.missing_settings = .Unavailable;
	form.diagnostic = {};
	copy(form.diagnostic[:255], "Missing values filled from current defaults; review workload fields before running");
	return .Ok;
}

launch_set_field :: proc(form: ^Benchmark_Launch, index: int, value: string)
{
	form.fields[index] = {};
	written: int;
	remaining: string = value;
	for part in strings.split_by_byte_iterator(&remaining, ',')
	{
		if written > 0
		{
			written += copy(form.fields[index][written:127], ",");
		}
		formatted: string = part;
		buffer: [64]u8;
		if strings.contains_any(part, ".eE")
		{
			number: f32;
			admission: support.Admission;
			number, admission = support.parse_number(part);
			if admission == .Ok
			{
				formatted = strconv.write_float(buffer[:], f64(number), 'g', -1, 32);
				if len(formatted) > 0 && formatted[0] == '+'
				{
					formatted = formatted[1:];
				}
			}
		}
		written += copy(form.fields[index][written:127], formatted);
	}
}

benchmark_label :: proc(package_name: string) -> string
{
	definitions: [len(BENCHMARKS)]Benchmark_Definition = BENCHMARKS;
	for definition in definitions
	{
		if definition.package_name == package_name
		{
			return string(definition.label);
		}
	}
	return package_name;
}
