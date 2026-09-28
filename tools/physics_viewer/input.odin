package physics_viewer

import "core:os"
import "core:math"
import support "../../benchmarks/benchmark_support"
import replay_format "../physics_replay"
import scene "../physics_scene"
import rl "vendor:raylib"

viewer_input :: proc(v: ^Viewer) -> scene.Input
{
	input: scene.Input = {mode=v.live.recipe.input};
	if input.mode != .Live || v.ui.keyboard == .Unavailable || v.cursor.playback != .Running ||
		!rl.IsWindowFocused() || v.ui.input != .Scene
	{
		return input;
	}
	forward, turn: f32;
	if rl.IsKeyDown(.W)
	{
		forward += 1;
	}
	if rl.IsKeyDown(.S)
	{
		forward -= 1;
	}
	if rl.IsKeyDown(.D)
	{
		turn += 1;
	}
	if rl.IsKeyDown(.A)
	{
		turn -= 1;
	}
	#partial switch v.live.recipe.scenario
	{
	case .Character_Controller:
		yaw: f32 = v.panes[0].camera.yaw;
		length: f32 = max(1, math.sqrt(forward*forward+turn*turn));
		input.x = (-math.cos(yaw)*forward+math.sin(yaw)*turn)/length;
		input.y = (-math.sin(yaw)*forward-math.cos(yaw)*turn)/length;
	case .Vehicle:
		// the authored vehicle steers its -X axle, so positive steering turns the +X chassis front right
		input.x, input.y = forward, turn;
	case .Tank_Controller:
		input.x, input.y = clamp(forward+turn, -1, 1), clamp(forward-turn, -1, 1);
	}
	return input;
}

Options :: struct
{
	scenario, recipe, replay, compare, screenshot, results, package_name: string,
	frame: int,
	budget: u64,
	width, height: i32,
}

parse_options :: proc(arguments: []string) -> (Options, scene.Status)
{
	o: Options = {budget=replay_format.DEFAULT_BUDGET, width=1600, height=900};
	seen: bit_set[0..<14];
	for argument in arguments
	{
		key, value: string;
		found: support.Admission;
		key, value, found = support.split_once(argument, '=');
		if found != .Ok || len(value) == 0
		{
			return {}, .Invalid_Data;
		}
		index: int;
		switch key
		{
		case "--scenario": index=0;
		o.scenario=value;
		case "--recipe": index=1;
		o.recipe=value;
		case "--replay": index=2;
		o.replay=value;
		case "--compare": index=3;
		o.compare=value;
		case "--screenshot": index=4;
		o.screenshot=value;
		case "--frame", "--memory-mib":
			number: int;
			admission: support.Admission;
			number, admission = support.parse_integer(value);
			if admission != .Ok
			{
				return {}, .Invalid_Data;
			}
			if key == "--frame"
			{
				index=5;
				if number < 0 || number > 1_000_000
				{
					return {}, .Invalid_Data;
				}
				o.frame=number;
			}
			else
			{
				index=6;
				if number < 64 || number > 16384
				{
					return {}, .Invalid_Data;
				}
				o.budget=u64(number)*1024*1024;
			}
		case "--window-size":
			index=7;
			dimensions: [2]int;
			if support.parse_grid(value, dimensions[:]) != .Ok || dimensions.x < 640 || dimensions.x > 7680 || dimensions.y < 480 || dimensions.y > 4320
			{
				return {}, .Invalid_Data;
			}
			o.width, o.height = i32(dimensions.x), i32(dimensions.y);
		case "--results": index=12;
		o.results=value;
		case "--package": index=13;
		o.package_name=value;
		case: return {}, .Invalid_Data;
		}
		if index in seen
		{
			return {}, .Invalid_Data;
		}
		seen += {index};
	}
	selections: int;
	selection_options: [4]int = {0, 1, 2, 12};
	for index in selection_options
	{
		if index in seen
		{
			selections+=1;
		}
	}
	if selections>1 || (3 in seen && 2 not_in seen) || (13 in seen && 12 not_in seen) ||
		(5 in seen && 2 not_in seen && (4 not_in seen || (0 not_in seen && 1 not_in seen)))
	{
		return {}, .Invalid_Data;
	}
	if len(o.screenshot)>0 && (os.exists(o.screenshot) || !os.is_dir(os.dir(o.screenshot)))
	{
		return {}, .File_Error;
	}
	return o, .Ok;
}

viewer_shortcuts :: proc(v: ^Viewer, previous_status: scene.Status) -> scene.Status
{
	status: scene.Status = previous_status;
	if rl.IsWindowFocused() && v.ui.selection_action == .None && v.ui.recording_delete.phase == .Closed &&
		v.launch.state == .Closed && v.ui.dropdown == 0 && v.ui.text_field == 0 && rl.IsKeyPressed(.ESCAPE)
	{
		close_content(v);
		v.ui.catalog_selected[1] = -1;
		return status;
	}
	if v.ui.keyboard == .Available
	{
		if v.count>0 && rl.IsKeyPressed(.SPACE)
		{
			status = .Ok;
			if live_diagnostic(v) == .Available
			{
				v.cursor.playback = .Paused;
				status = select_frame(v, min(v.cursor.index+1, v.cursor.count-1));
			}
			else
			{
				status = transport_play_pause(v);
			}
			v.cursor.accumulator=0;
		}
		if rl.IsKeyPressed(.RIGHT)
		{
			v.cursor.playback=.Paused;
			status=select_frame(v, min(v.cursor.index+1, v.cursor.count-1));
		}
		if rl.IsKeyPressed(.LEFT) && v.mode==.Replay
		{
			v.cursor.playback=.Paused;
			status=select_frame(v, max(0, v.cursor.index-1));
		}
		if rl.IsKeyPressed(.END) && v.mode == .Replay
		{
			v.cursor.playback, v.cursor.accumulator = .Paused, 0;
			status = select_frame(v, v.cursor.count-1);
		}
		if rl.IsKeyPressed(.F)
		{
			viewer_camera_fit(v);
		}
		if rl.IsKeyPressed(.R) || rl.IsKeyPressed(.HOME)
		{
			v.cursor.playback, v.cursor.accumulator = .Paused, 0;
			status = open_live(v, v.live.recipe) if v.mode == .Live else select_frame(v, 0);
		}
	}
	return status;
}
