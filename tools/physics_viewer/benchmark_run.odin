package physics_viewer

import "core:fmt"
import "core:io"
import "core:os"
import "core:strings"
import "core:time"
import rl "vendor:raylib"
import scene "../physics_scene"

Benchmark_Stage :: enum
{
	Preparing, Building, Measuring, Reporting,
}
Benchmark_Run :: struct
{
	process: Launch_Process,
	log, reader: ^os.File,
	path, result: string,
	tail: []u8,
	tail_count, parsed: int,
	offset: i64,
	started, last_read: time.Tick,
	build_complete: scene.Availability,
	elapsed: time.Duration,
	stage: Benchmark_Stage,
	exit_code: int,
	error: os.Error,
	detail: [512]u8,
	scroll: rl.Vector2,
}

benchmark_run_close :: proc(run: ^Benchmark_Run)
{
	launch_process_close(&run.process);
	if run.reader != nil
	{
		os.close(run.reader);
	}
	if run.log != nil
	{
		os.close(run.log);
	}
	delete(run.tail);
	delete(run.path);
	delete(run.result);
	run^ = {};
}

Benchmark_Log_Output :: struct
{
	file: ^os.File,
	error: os.Error,
}

benchmark_log_output :: proc(data: rawptr, mode: io.Stream_Mode, bytes: []u8, offset: i64, whence: io.Seek_From) -> (i64, io.Error)
{
	output: ^Benchmark_Log_Output = cast(^Benchmark_Log_Output)data;
	if mode == .Flush
	{
		return 0, .None;
	}
	if mode != .Write
	{
		return 0, .Unsupported;
	}
	written: int;
	for written < len(bytes) && output.error == nil
	{
		count: int;
		count, output.error = os.write(output.file, bytes[written:]);
		written += count;
		if count == 0 && output.error == nil
		{
			output.error = io.Error.Short_Write;
		}
	}
	return i64(written), .None if output.error == nil else .Short_Write;
}

benchmark_log_write :: proc(run: ^Benchmark_Run, format: string, args: ..any) -> os.Error
{
	output: Benchmark_Log_Output = {file=run.log};
	fmt.wprintf({procedure=benchmark_log_output, data=&output}, format, ..args);
	return output.error;
}

benchmark_run_prepare :: proc(v: ^Viewer, root: string)
{
	run: ^Benchmark_Run = &v.benchmark;
	benchmark_run_close(run);
	run.started = time.tick_now();
	run.last_read = run.started;
	run.path = fmt.aprintf("%s/build/physics-viewer/runs/%d/launch.log", root, time.to_unix_nanoseconds(time.now()));
	directory: string = os.dir(run.path);
	run.error = os.make_directory_all(directory);
	if run.error == nil
	{
		run.log, run.error = os.open(run.path, {.Write, .Append, .Create, .Excl, .Inheritable});
	}
	if run.error == nil
	{
		run.reader, run.error = os.open(run.path);
	}
	if run.error == nil
	{
		run.tail = make([]u8, 64*1024);
		run.error = benchmark_log_write(run, "BENCHMARK_LAUNCH time=%v cwd=%s operation=%v\n", time.now(), root, v.request.operation);
	}
	if run.error != nil
	{
		fmt.bprintf(run.detail[:511], "Cannot create launch log: %v", run.error);
		run.exit_code = 1;
		v.launch.state = .Finished;
		launch_request_close(&v.request);
		return;
	}
	fmt.printfln("BENCHMARK_LOG path=%s", run.path);
	close_content(v);
	v.ui.selection_action = .None;
	v.launch.state = .Preparing;
}

benchmark_log_line :: proc(run: ^Benchmark_Run, line: string)
{
	if strings.has_prefix(line, "BENCHMARK_")
	{
		fmt.println(line);
	}
	if line == "BUILD_BENCHMARKS_OK"
	{
		run.build_complete = .Available;
	}
	if strings.has_prefix(line, "[build]") || strings.has_prefix(line, "BENCHMARK_STAGE stage=build")
	{
		run.stage = .Building;
	}
	else if strings.has_prefix(line, "[benchmark]")
	{
		run.stage = .Measuring;
	}
	else if strings.has_prefix(line, "BENCHMARK_STAGE stage=report")
	{
		run.stage = .Reporting;
	}
	else if strings.has_prefix(line, "BENCHMARK_OK result=")
	{
		value: string = line[len("BENCHMARK_OK result="):];
		separator: int = strings.index(value, " report=");
		if separator > 0
		{
			delete(run.result);
			run.result = strings.clone(value[:separator]);
		}
	}
	if len(line) > 0
	{
		run.detail = {};
		copy(run.detail[:511], line);
	}
}

benchmark_log_read :: proc(run: ^Benchmark_Run) -> int
{
	buffer: [16*1024]u8;
	count: int;
	error: os.Error;
	count, error = os.read_at(run.reader, buffer[:], run.offset);
	if count == 0
	{
		if error != nil && error != io.Error.EOF
		{
			run.error = error;
		}
		return 0;
	}
	run.offset += i64(count);
	remove: int = max(0, run.tail_count+count-len(run.tail));
	copy(run.tail, run.tail[remove:run.tail_count]);
	run.tail_count -= remove;
	run.parsed = max(0, run.parsed-remove);
	copy(run.tail[run.tail_count:], buffer[:count]);
	run.tail_count += count;
	for run.parsed < run.tail_count
	{
		remaining: string = string(run.tail[run.parsed:run.tail_count]);
		end: int = strings.index_byte(remaining, '\n');
		if end < 0
		{
			break;
		}
		benchmark_log_line(run, strings.trim_right(remaining[:end], "\r"));
		run.parsed += end+1;
	}
	return count;
}

benchmark_run_drain :: proc(run: ^Benchmark_Run)
{
	for benchmark_log_read(run) > 0
	{
	}
	if run.parsed < run.tail_count
	{
		benchmark_log_line(run, string(run.tail[run.parsed:run.tail_count]));
		run.parsed = run.tail_count;
	}
}

benchmark_run_finish :: proc(v: ^Viewer, code: int, error: os.Error)
{
	run: ^Benchmark_Run = &v.benchmark;
	launch_process_close(&run.process);
	run.exit_code = code;
	if run.error == nil
	{
		run.error = error;
	}
	run.elapsed = time.tick_since(run.started);
	benchmark_run_drain(run);
	launch_request_close(&v.request);
	v.launch.state = .Finished;
	if code == 0 && run.error == nil
	{
		if len(run.result) == 0
		{
			run.exit_code = 1;
			run.detail = {};
			copy(run.detail[:], "Runner exited without publishing a result");
		}
		else
		{
			results_refresh(v);
			selected: string = run.result;
			path_buffer, result_buffer: File_Path_Buffer;
			result_path: string;
			result_path, _ = recording_absolute_path(run.result, result_buffer[:]);
			for entry, index in v.ui.runs.entries
			{
				absolute: string;
				absolute, _ = recording_absolute_path(entry.path, path_buffer[:]);
				if absolute == result_path && len(absolute) > 0
				{
					selected = entry.path;
					v.ui.run_selected = i32(index);
					break;
				}
			}
			v.ui.status = open_results(v, selected, benchmark_package(v.launch.package_index));
			if v.ui.status == .Ok
			{
				v.launch.state = .Closed;
			}
			else
			{
				run.exit_code = 1;
				run.detail = {};
				fmt.bprintf(run.detail[:511], "Published result could not be opened: %v", v.ui.status);
			}
		}
	}
	if run.exit_code == 0 && run.error != nil
	{
		run.exit_code = 1;
	}
	log_error: os.Error = benchmark_log_write(run, "\nBENCHMARK_EXIT code=%d error=%v elapsed_ms=%d child_code=%d detail=%s\n",
		run.exit_code, run.error, i64(run.elapsed/time.Millisecond), code, strings.string_from_null_terminated_ptr(raw_data(run.detail[:]), 512));
	if log_error != nil
	{
		if run.error == nil
		{
			run.error = log_error;
		}
		if run.exit_code == 0
		{
			run.exit_code = 1;
		}
		v.launch.state = .Finished;
	}
	os.close(run.log);
	os.close(run.reader);
	run.log, run.reader = nil, nil;
	fmt.printfln("BENCHMARK_FINISHED code=%d error=%v elapsed_ms=%d", run.exit_code, run.error, i64(run.elapsed/time.Millisecond));
}

benchmark_run_start :: proc(v: ^Viewer, root: string)
{
	run: ^Benchmark_Run = &v.benchmark;
	run.stage = .Building if v.request.phase == .Build else .Preparing;
	error: os.Error = benchmark_log_write(run, "BENCHMARK_STAGE stage=preparing phase=%v frame_ms=%d\nargv=%q\n", v.request.phase, i64(time.tick_since(run.started)/time.Millisecond), v.request.arguments[:]);
	if error == nil
	{
		error = launch_process_start(&run.process, &v.request, root, run.log);
	}
	if error != nil
	{
		benchmark_run_finish(v, 1, error);
		return;
	}
	error = benchmark_log_write(run, "BENCHMARK_CHILD pid=%d cancellable=yes\n", run.process.pid);
	if error != nil
	{
		benchmark_run_finish(v, 1, error);
		return;
	}
	v.launch.state = .Running;
}

benchmark_run_poll :: proc(v: ^Viewer)
{
	run: ^Benchmark_Run = &v.benchmark;
	if time.tick_since(run.last_read) >= 100*time.Millisecond
	{
		benchmark_log_read(run);
		run.last_read = time.tick_now();
	}
	if run.error != nil
	{
		benchmark_run_finish(v, 1, run.error);
		return;
	}
	code: int;
	error: os.Error;
	code, error = launch_process_poll(&run.process);
	if error == os.General_Error.Timeout
	{
		return;
	}
	if v.request.phase == .Build
	{
		launch_process_close(&run.process);
		benchmark_run_drain(run);
		log_error: os.Error = benchmark_log_write(run, "\nBENCHMARK_BUILD_EXIT code=%d error=%v marker=%v\n", code, error, run.build_complete);
		if run.error == nil
		{
			run.error = log_error;
		}
		if code == 0 && error == nil && run.error == nil && run.build_complete == .Available
		{
			for argument in v.request.arguments
			{
				delete(argument);
			}
			delete(v.request.arguments);
			v.request.arguments, v.request.run_arguments = v.request.run_arguments, {};
			v.request.phase = .Run;
			run.stage = .Preparing;
			v.launch.state = .Preparing;
			return;
		}
		if code == 0
		{
			code = 1;
			benchmark_log_write(run, "Build did not complete successfully; benchmark was not started\n");
		}
	}
	benchmark_run_finish(v, code, error);
}

benchmark_run_cancel :: proc(v: ^Viewer)
{
	error: os.Error;
	if v.launch.state == .Running
	{
		error = launch_process_cancel(&v.benchmark.process);
	}
	benchmark_run_finish(v, 130 if error == nil else 1, error);
}

ui_benchmark_run :: proc(v: ^Viewer, bounds: rl.Rectangle)
{
	run: ^Benchmark_Run = &v.benchmark;
	d: f32 = v.ui.dpi;
	x, y, width: f32 = bounds.x+12*d, bounds.y+12*d, bounds.width-24*d;
	rl.DrawRectangleRec(bounds, BACKGROUND);
	ui_heading(&v.ui, {x, y, width, 32*d}, benchmark_label(benchmark_package(v.launch.package_index)));
	y += 42*d;
	stages: [Benchmark_Stage]string = {.Preparing="Preparing", .Building="Building", .Measuring="Running", .Reporting="Reporting"};
	elapsed: time.Duration = run.elapsed if v.launch.state == .Finished else time.tick_since(run.started);
	if v.launch.state == .Finished
	{
		if run.exit_code == 130
		{
			ui_text({x, y, width, 28*d}, "Cancelled");
		}
		else
		{
			ui_text({x, y, width, 28*d}, "Failed | exit %d | %s", run.exit_code, stages[run.stage]);
		}
	}
	else
	{
		ui_text({x, y, width, 28*d}, "%s | Total elapsed %.1f s", stages[run.stage], f64(elapsed)/f64(time.Second));
	}
	y += 36*d;
	if run.error != nil
	{
		ui_text({x, y, width, 28*d}, "%v", run.error);
		y += 32*d;
	}
	ui_cell(&v.ui, {x, y, width, 28*d}, strings.string_from_null_terminated_ptr(raw_data(run.detail[:]), 512));
	y += 32*d;
	if v.launch.state == .Finished
	{
		log_bounds: rl.Rectangle = {x, y, width, max(40*d, bounds.y+bounds.height-y-64*d)};
		view: rl.Rectangle;
		remaining: string = string(run.tail[:run.tail_count]);
		line_width: f32;
		count: int;
		for line in strings.split_by_byte_iterator(&remaining, '\n')
		{
			line_width = max(line_width, ui_measure_text(v.ui.font, line, 16*d, 0));
			count += 1;
		}
		rl.GuiScrollPanel(log_bounds, nil, {0, 0, max(width-20*d, line_width+16*d), f32(count)*22*d+8*d}, &run.scroll, &view);
		rl.BeginScissorMode(i32(view.x), i32(view.y), i32(view.width), i32(view.height));
		remaining = string(run.tail[:run.tail_count]);
		index: int;
		for line in strings.split_by_byte_iterator(&remaining, '\n')
		{
			line_y: f32 = log_bounds.y+4*d+run.scroll.y+f32(index)*22*d;
			if line_y+22*d >= view.y && line_y < view.y+view.height
			{
				ui_draw_text(v.ui.font, strings.trim_right(line, "\r"), {log_bounds.x+8*d+run.scroll.x, line_y}, 16*d, 0, {223, 227, 233, 255});
			}
			index += 1;
		}
		rl.EndScissorMode();
	}
	y = bounds.y+bounds.height-44*d;
	if v.launch.state == .Finished
	{
		if rl.GuiButton({x, y, 144*d, 32*d}, "Back to settings") || rl.IsKeyPressed(.ESCAPE)
		{
			v.launch.state = .Editing;
			v.launch.diagnostic = {};
			copy(v.launch.diagnostic[:255], "For missing binaries, choose Build and benchmark");
		}
	}
	else if rl.GuiButton({x, y, 144*d, 32*d}, "Cancel run") || rl.IsKeyPressed(.ESCAPE)
	{
		benchmark_run_cancel(v);
	}
	if len(run.tail) > 0 && rl.GuiButton({x+156*d, y, 112*d, 32*d}, "Open log")
	{
		viewer_open_file(run.path);
	}
}
