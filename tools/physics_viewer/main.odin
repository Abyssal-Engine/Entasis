package physics_viewer

import "core:fmt"
import "core:os"
import rl "vendor:raylib"
import replay "../physics_replay"
import scene "../physics_scene"
import scenarios "../physics_scenarios"
import report "../benchmark_report"

CONFIGURATION :: #config(ENTASIS_VISUAL_CONFIGURATION, "development");
COMPONENTS :: #config(ENTASIS_BENCHMARK_COMPONENTS, "all");
BACKGROUND :: rl.Color{30, 31, 34, 255};
Pane :: struct
{
	reader: replay.Reader,
	renderer: Renderer,
	camera: Camera,
	texture: rl.RenderTexture,
	data_dirty, cached: scene.Availability,
	cached_camera: Camera,
	cached_filter: View_Filter,
}
Viewer :: struct
{
	panes: [2]Pane,
	live: ^scenarios.Session,
	mode: Mode,
	count: int,
	cursor: scene.Cursor,
	budget, required_bytes, available_bytes: u64,
	ui: UI,
	pairs: [][2]int,
	synchronization: Synchronization,
	camera_link: Camera_Link,
	active_pane: int,
	results: report.Result_Dataset,
	launch: Benchmark_Launch,
	request: Launch_Request,
	benchmark: Benchmark_Run,
}
Mode :: enum
{
	Browser, Live, Replay,
}

run :: proc(options: Options, root: string) -> int
{
	r: scenarios.Recipe;
	status: scene.Status;
	if len(options.scenario)>0 || len(options.recipe)>0
	{
		if len(options.recipe)>0
		{
			bytes: []u8;
			error: os.Error;
			bytes, error = os.read_entire_file(options.recipe, context.allocator);
			if error != nil
			{
				return 2;
			}
			r, status = scenarios.recipe_parse(bytes);
			delete(bytes);
		}
		else
		{
			id: scenarios.Scenario;
			if len(options.scenario)>0
			{
				id, status = scenarios.resolve(options.scenario);
			}
			if status == .Ok
			{
				r, status = scenarios.recipe_admit(id, nil);
			}
		}
		if status != .Ok
		{
			fmt.eprintfln("scenario admission: %v", status);
			return 2;
		}
	}
	rl.SetTraceLogLevel(.WARNING);
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT, .MSAA_4X_HINT});
	rl.InitWindow(options.width, options.height, "Entasis | Examples and benchmark replay");
	if !rl.IsWindowReady()
	{
		return 1;
	}
	rl.SetWindowMinSize(640, 480);
	defer rl.CloseWindow();
	rl.SetExitKey(rl.KeyboardKey(0));
	rl.SetTargetFPS(60);
	v: Viewer = {budget=options.budget};
	v.ui.catalog_selected = {-1, -1};
	v.ui.hovered_pane, v.ui.focused_pane, v.ui.drag_pane = -1, -1, -1;
	v.launch = {package_index=1};
	defer benchmark_run_close(&v.benchmark);
	defer launch_request_close(&v.request);
	defer report.result_dataset_close(&v.results);
	defer ui_destroy(&v.ui);
	defer close_content(&v);
	status = ui_style(&v.ui);
	if status != .Ok
	{
		fmt.eprintfln("UI font: %v", status);
		return 1;
	}
	if len(options.replay)>0
	{
		status=open_replays(&v, options.replay, options.compare);
	}
	else if len(options.scenario)>0 || len(options.recipe)>0
	{
		status=open_live(&v, r);
	}
	else if len(options.results) > 0
	{
		status = open_results(&v, options.results, options.package_name);
	}
	if status != .Ok
	{
		fmt.eprintfln("open: %v", status);
		return 1;
	}
	if v.count>0
	{
		status=select_frame(&v, options.frame);
		if len(options.screenshot) > 0 && status == .Ok
		{
			v.ui.fit = .Available;
		}
	}
	if status != .Ok
	{
		fmt.eprintfln("frame: %v", status);
		return 2;
	}
	results_refresh(&v);
	rendered_frames: int;
	for
	{
		if rl.WindowShouldClose()
		{
			if v.launch.state == .Running || v.launch.state == .Preparing
			{
				benchmark_run_cancel(&v);
			}
			break;
		}
		free_all(context.temp_allocator);
		if v.launch.state == .Running
		{
			benchmark_run_poll(&v);
		}
		rl.BeginDrawing();
		rl.ClearBackground(BACKGROUND);
		ui_layout(&v);
		if v.ui.fit == .Available && v.ui.image_count > 0
		{
			viewer_camera_fit(&v);
			v.ui.fit = .Unavailable;
		}
		if v.launch.state != .Preparing && v.launch.state != .Running && v.launch.state != .Finished
		{
			status = viewer_shortcuts(&v, v.ui.status);
			viewer_camera_update(&v);
			status = transport_advance(&v, f64(rl.GetFrameTime()), status);
			v.ui.status = status;
			render_status: scene.Status = viewport_render(&v);
			if render_status != .Ok
			{
				fmt.eprintfln("viewport render target: %v", render_status);
				return 1;
			}
			viewport_composite(&v);
		}
		ui_workspace(&v);
		rl.EndDrawing();
		if v.launch.state == .Preparing
		{
			benchmark_run_start(&v, root);
		}
		else if v.launch.state == .Requested
		{
			benchmark_run_prepare(&v, root);
		}
		rendered_frames += 1;
		if len(options.screenshot)>0 && rendered_frames >= 3
		{
			status=save_screenshot(options.screenshot);
			if status!=.Ok
			{
				return 1;
			}
			fmt.printfln("SCREENSHOT_OK path=%s", options.screenshot);
			return 0;
		}
	}
	return 0;
}

main :: proc()
{
	o: Options;
	status: scene.Status;
	o, status=parse_options(os.args[1:]);
	if status!=.Ok
	{
		fmt.eprintfln("viewer arguments: %v", status);
		os.exit(2);
	}
	root: string;
	root_error: os.Error;
	root, root_error = os.get_absolute_path(".", context.allocator);
	if root_error != nil
	{
		os.exit(1);
	}
	result: int = run(o, root);
	delete(root);
	os.exit(result);
}
