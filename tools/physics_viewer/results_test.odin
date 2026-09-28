package physics_viewer

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strings"
import "core:sync"
import "core:testing"
import scene "../physics_scene"

// these fixtures temporarily own cwd because discovery uses workspace roots
saved_run_test_cwd: sync.Mutex;

SAVED_RUN_TEST_CSV :: "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_parameters,step_index,recording_path,recording_component,recording_mode,timing_method\n1,ok,ok,12,workload=container,1,sample.epr,physics_elapsed_ms,on,native_step_sum\n";
SAVED_RUN_TEST_RECORDING :: "isolated associated recording";

saved_run_fixture :: proc(t: ^testing.T, path, title: string)
{
	testing.expect_value(t, os.make_directory_all(path), nil);
	metadata: string = fmt.aprintf("- Run date (UTC): `2026-09-28`\n- Configuration: `release`\nVariant: `%s`\n", title);
	defer delete(metadata);
	for name, index in ([3]string{"README.md", "workers-1.csv", "sample.epr"})
	{
		filename: string = fmt.aprintf("%s/%s", path, name);
		defer delete(filename);
		text: string = ([3]string{metadata, SAVED_RUN_TEST_CSV, SAVED_RUN_TEST_RECORDING})[index];
		testing.expect_value(t, os.write_entire_file(filename, text), nil);
	}
}

saved_run_fixture_preserved :: proc(t: ^testing.T, path, title: string)
{
	metadata: string = fmt.aprintf("- Run date (UTC): `2026-09-28`\n- Configuration: `release`\nVariant: `%s`\n", title);
	defer delete(metadata);
	for name, index in ([2]string{"README.md", "workers-1.csv"})
	{
		filename: string = fmt.aprintf("%s/%s", path, name);
		defer delete(filename);
		data: []u8;
		error: os.Error;
		data, error = os.read_entire_file(filename, context.allocator);
		defer delete(data);
		testing.expect_value(t, error, nil);
		testing.expect_value(t, string(data), metadata if index == 0 else SAVED_RUN_TEST_CSV);
	}
}

@(test)
saved_directory_budget :: proc(t: ^testing.T)
{
	sync.mutex_lock(&saved_run_test_cwd);
	defer sync.mutex_unlock(&saved_run_test_cwd);
	previous: string;
	root: string;
	error: os.Error;
	previous, error = os.getwd(context.allocator);
	if !testing.expect_value(t, error, nil)
	{
		return;
	}
	defer delete(previous);
	root, error = os.make_directory_temp("", "entasis-directory-", context.allocator);
	if !testing.expect_value(t, error, nil)
	{
		return;
	}
	defer delete(root);
	defer os.remove_all(root);
	if !testing.expect_value(t, os.chdir(root), nil)
	{
		return;
	}
	defer os.chdir(previous);
	testing.expect_value(t, os.make_directory("populated"), nil);
	testing.expect_value(t, os.make_directory("empty"), nil);
	for index in 0 ..< 25
	{
		buffer: [32]u8;
		path: string = fmt.bprintf(buffer[:], "populated/entry-%02d", index);
		testing.expect_value(t, os.write_entire_file(path, "fixture"), nil);
	}
	// fixtures and assertions stay outside the observed allocator context
	for path, index in ([3]string{"populated", "empty", "missing"})
	{
		status: scene.Status;
		owned_bytes: u64;
		{
			context.allocator = mem.panic_allocator();
			discovery: Result_Discovery = results_discover(path, 0, 0, .Cases);
			status, error, owned_bytes = discovery.status, discovery.error, discovery.owned_bytes;
			results_discovery_close(&discovery);
		}
		testing.expect_value(t, status, scene.Status.Budget_Exceeded if index == 0 else scene.Status.Ok);
		testing.expect_value(t, error, nil);
		testing.expect_value(t, owned_bytes, u64(0));
	}
	tracking: mem.Tracking_Allocator;
	mem.tracking_allocator_init(&tracking, context.allocator);
	defer mem.tracking_allocator_destroy(&tracking);
	// find both sides of admission while exercising several capacity increases
	low, high: u64 = 0, 1024*1024;
	budget: u64 = high;
	for
	{
		mem.tracking_allocator_reset(&tracking);
		status: scene.Status;
		count: int;
		owned_bytes: u64;
		retained: i64;
		{
			context.allocator = mem.tracking_allocator(&tracking);
			discovery: Result_Discovery = results_discover("populated", 0, budget, .Cases);
			status, error = discovery.status, discovery.error;
			count, owned_bytes = len(discovery.files), discovery.owned_bytes;
			retained = tracking.current_memory_allocated;
			results_discovery_close(&discovery);
		}
		testing.expect_value(t, error, nil);
		testing.expect_value(t, u64(retained), owned_bytes);
		testing.expect(t, u64(tracking.peak_memory_allocated) <= budget);
		testing.expect_value(t, tracking.current_memory_allocated, i64(0));
		testing.expect_value(t, len(tracking.allocation_map), 0);
		if status == .Ok
		{
			testing.expect_value(t, count, 25);
			testing.expect(t, tracking.total_allocation_count > 25);
			high = budget;
		}
		else
		{
			testing.expect_value(t, status, scene.Status.Budget_Exceeded);
			testing.expect(t, count < 25);
			if !testing.expect(t, budget < high)
			{
				return;
			}
			low = budget;
		}
		if high-low == 1
		{
			break;
		}
		budget = (low+high)/2;
	}
	// fixed backing forces allocation failure before and after snapshot retention
	backing: []u8 = make_aligned([]u8, 4096, align_of(os.File_Info));
	defer delete(backing);
	for available in ([3]int{128, 2048, len(backing)})
	{
		buddy: mem.Buddy_Allocator;
		mem.buddy_allocator_init(&buddy, backing[:available], align_of(os.File_Info));
		mem.tracking_allocator_reset(&tracking);
		tracking.backing = mem.buddy_allocator(&buddy);
		status: scene.Status;
		count: int;
		{
			context.allocator = mem.tracking_allocator(&tracking);
			discovery: Result_Discovery = results_discover("populated", 0, 1024*1024, .Cases);
			status, error, count = discovery.status, discovery.error, len(discovery.files);
			results_discovery_close(&discovery);
		}
		testing.expect_value(t, status, scene.Status.File_Error);
		testing.expect_value(t, error, os.Error(mem.Allocator_Error.Out_Of_Memory));
		testing.expect_value(t, tracking.current_memory_allocated, i64(0));
		testing.expect_value(t, len(tracking.allocation_map), 0);
		if available == len(backing)
		{
			testing.expect(t, count > 0 && count < 25);
		}
	}
}

@(test)
saved_run_catalog_budget :: proc(t: ^testing.T)
{
	sync.mutex_lock(&saved_run_test_cwd);
	defer sync.mutex_unlock(&saved_run_test_cwd);
	previous: string;
	root: string;
	error: os.Error;
	previous, error = os.getwd(context.allocator);
	if !testing.expect_value(t, error, nil)
	{
		return;
	}
	defer delete(previous);
	root, error = os.make_directory_temp("", "entasis-catalog-", context.allocator);
	if !testing.expect_value(t, error, nil)
	{
		return;
	}
	defer delete(root);
	defer os.remove_all(root);
	if !testing.expect_value(t, os.chdir(root), nil)
	{
		return;
	}
	defer os.chdir(previous);
	label: [2048]u8;
	for &byte in label
	{
		byte = 'x';
	}
	for index in 0 ..< 9
	{
		path: string = fmt.aprintf("results/container/run-%d", index);
		defer delete(path);
		saved_run_fixture(t, path, string(label[:]));
	}
	runs: Run_List;
	defer results_paths_close(&runs);
	if !testing.expect_value(t, results_scan_case(&runs, "container", 1024*1024), scene.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, len(runs.entries), 9);
	testing.expect(t, cap(runs.entries) > len(runs.entries));
	retained: u64 = u64(cap(runs.entries))*size_of(Run_Entry);
	for entry in runs.entries
	{
		retained += u64(len(entry.path)+len(entry.date)+len(entry.title)+len(entry.configuration));
		testing.expect_value(t, len(entry.title), len(label));
		testing.expect_value(t, entry.recording, Recording_State.Present);
	}
	testing.expect_value(t, runs.owned_bytes, retained);
	// find the actual admission boundary including live discovery and presence data
	low, high: u64 = 0, 1024*1024;
	for high-low > 1
	{
		middle: u64 = (low+high)/2;
		status: scene.Status = results_scan_case(&runs, "container", middle);
		if status == .Ok
		{
			high = middle;
		}
		else
		{
			testing.expect_value(t, status, scene.Status.Budget_Exceeded);
			testing.expect_value(t, runs.owned_bytes, u64(0));
			testing.expect_value(t, cap(runs.entries), 0);
			low = middle;
		}
	}
	testing.expect(t, high > retained);
	testing.expect_value(t, results_scan_case(&runs, "container", high), scene.Status.Ok);
	testing.expect_value(t, len(runs.entries), 9);
	testing.expect_value(t, results_scan_case(&runs, "container", high-1), scene.Status.Budget_Exceeded);
	testing.expect_value(t, runs.owned_bytes, u64(0));
	testing.expect_value(t, len(runs.entries), 0);
	v: Viewer = {budget=high};
	defer results_paths_close(&v.ui.runs);
	defer delete(v.ui.visible_runs);
	defer recordings_close(&v.ui.recordings);
	for definition, index in BENCHMARKS
	{
		if definition.package_name == "container"
		{
			v.launch.package_index = i32(index);
		}
	}
	results_refresh(&v);
	if !testing.expect_value(t, v.ui.status, scene.Status.Ok) || !testing.expect_value(t, len(v.ui.visible_runs), 9)
	{
		return;
	}
	testing.expect_value(t, viewer_content_bytes(&v), retained+9*size_of(int));
	v.ui.run_selected = 3;
	selected: string = strings.clone(v.ui.runs.entries[3].path);
	defer delete(selected);
	results_refresh(&v);
	testing.expect_value(t, v.ui.status, scene.Status.Ok);
	if testing.expect(t, v.ui.run_selected >= 0)
	{
		testing.expect_value(t, v.ui.runs.entries[v.ui.run_selected].path, selected);
	}
	// the UI catalog remains live while the comparison catalog is scanned
	testing.expect_value(t, recordings_refresh(&v, "benchmark/container"), scene.Status.Budget_Exceeded);
	testing.expect_value(t, v.ui.recordings.owned_bytes, u64(0));
	v.budget = high-1;
	results_refresh(&v);
	testing.expect_value(t, v.ui.status, scene.Status.Budget_Exceeded);
	testing.expect_value(t, v.ui.run_selected, i32(-1));
	testing.expect_value(t, v.ui.run_focus, i32(-1));
	testing.expect_value(t, len(v.ui.visible_runs), 0);
	testing.expect_value(t, viewer_content_bytes(&v), u64(0));
	// without a UI catalog, the local catalog still coexists with comparison labels
	v.budget = high+u64(len("benchmark/container"));
	testing.expect_value(t, recordings_refresh(&v, "benchmark/container"), scene.Status.Budget_Exceeded);
	testing.expect_value(t, v.ui.recordings.owned_bytes, u64(0));
	v.budget = 0;
	testing.expect_value(t, recordings_refresh(&v, "benchmark/container"), scene.Status.Budget_Exceeded);
	testing.expect_value(t, viewer_content_bytes(&v), u64(0));
	v.budget = 1024*1024;
	testing.expect_value(t, recordings_refresh(&v, "benchmark/container"), scene.Status.Ok);
	testing.expect_value(t, len(v.ui.recordings.entries), 9);
	testing.expect(t, v.ui.recordings.owned_bytes > 9*size_of(Recording_Entry));
	testing.expect(t, viewer_content_bytes(&v) <= v.budget);
	recordings_close(&v.ui.recordings);
	testing.expect_value(t, viewer_content_bytes(&v), u64(0));
}
