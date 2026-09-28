#+build linux

package physics_viewer

import "core:fmt"
import "core:os"
import "core:sync"
import "core:testing"
import scene "../physics_scene"

@(test)
recording_cleanup_missing_roots :: proc(t: ^testing.T)
{
	when ODIN_OS == .Linux
	{
		sync.mutex_lock(&saved_run_test_cwd);
		defer sync.mutex_unlock(&saved_run_test_cwd);
		previous, error := os.getwd(context.allocator);
		if !testing.expect_value(t, error, nil)
		{
			return;
		}
		defer delete(previous);
		root: string;
		root, error = os.make_directory_temp("", "entasis-cleanup-", context.allocator);
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
		v: Viewer = {budget=1024*1024};
		defer recording_delete_close(&v.ui.recording_delete);
		defer results_paths_close(&v.ui.runs);
		defer delete(v.ui.visible_runs);
		for location in ([4]string{"", "results/container/run", "build/benchmark-results/linux_amd64/container/run", "build/benchmark-results/windows_amd64/container/run"})
		{
			if len(location) > 0
			{
				saved_run_fixture(t, location, "ordinary");
			}
			recording_delete_preview(&v, .Workspace);
			for step: int = 0; step < 32 && v.ui.recording_delete.phase == .Scanning; step += 1
			{
				recording_delete_step(&v);
			}
			if !testing.expect_value(t, v.ui.recording_delete.phase, Recording_Delete_Phase.Ready)
			{
				return;
			}
			testing.expect_value(t, len(v.ui.recording_delete.targets), 0 if len(location) == 0 else 1);
			v.ui.recording_delete.phase = .Rechecking;
			for step: int = 0; step < 32 && v.ui.recording_delete.phase != .Complete; step += 1
			{
				recording_delete_step(&v);
			}
			testing.expect_value(t, v.ui.recording_delete.phase, Recording_Delete_Phase.Complete);
			testing.expect_value(t, v.ui.recording_delete.deleted, 0 if len(location) == 0 else 1);
			if len(location) > 0
			{
				filename: string = fmt.aprintf("%s/sample.epr", location);
				defer delete(filename);
				testing.expect_value(t, result_recording_file(filename).state, Recording_State.Missing);
				saved_run_fixture_preserved(t, location, "ordinary");
			}
			recording_delete_close(&v.ui.recording_delete);
			os.remove_all("results");
			os.remove_all("build");
		}
		buffer: File_Path_Buffer;
		path, status := recording_absolute_path("absent/path", buffer[:]);
		testing.expect_value(t, status, scene.Status.Ok);
		testing.expect_value(t, recording_directory_inspect(path).state, Recording_State.Missing);
		_, status = recording_absolute_path("oversized", buffer[:2]);
		testing.expect_value(t, status, scene.Status.File_Error);
		copy(buffer[:], "alias/path");
		path, status = recording_absolute_path(string(buffer[:10]), buffer[:]);
		testing.expect_value(t, status, scene.Status.Ok);
		expected: string = fmt.aprintf("%s/alias/path", root);
		defer delete(expected);
		testing.expect_value(t, path, expected);
	}
}

@(test)
recording_cleanup_rejects_linked_runs :: proc(t: ^testing.T)
{
	when ODIN_OS == .Linux
	{
		sync.mutex_lock(&saved_run_test_cwd);
		defer sync.mutex_unlock(&saved_run_test_cwd);
		previous, error := os.getwd(context.allocator);
		if !testing.expect_value(t, error, nil)
		{
			return;
		}
		defer delete(previous);
		root: string;
		root, error = os.make_directory_temp("", "entasis-linked-cleanup-", context.allocator);
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
		saved_run_fixture(t, "outside/run", "external");
		testing.expect_value(t, os.make_directory_all("workspace/results/container"), nil);
		testing.expect_value(t, os.make_directory_all("outside/child"), nil);
		testing.expect_value(t, os.symlink("../../../outside/run", "workspace/results/container/linked"), nil);
		testing.expect_value(t, os.symlink("../outside", "workspace/parent"), nil);
		testing.expect_value(t, os.symlink("../outside/child", "workspace/traversal"), nil);
		testing.expect_value(t, os.chdir("workspace"), nil);
		v: Viewer = {budget=1024*1024};
		defer recording_delete_close(&v.ui.recording_delete);
		recording_delete_preview(&v, .Workspace);
		for step: int = 0; step < 32 && v.ui.recording_delete.phase == .Scanning; step += 1
		{
			recording_delete_step(&v);
		}
		testing.expect_value(t, v.ui.recording_delete.phase, Recording_Delete_Phase.Ready);
		testing.expect_value(t, len(v.ui.recording_delete.targets), 0);
		testing.expect(t, len(v.ui.recording_delete.exclusions) > 0);
		for spelling in ([3]string{"results/container/linked", "parent/run", "traversal/../run"})
		{
			buffer: File_Path_Buffer;
			path, status := recording_absolute_path(spelling, buffer[:]);
			testing.expect_value(t, status, scene.Status.Ok);
			testing.expect_value(t, recording_directory_inspect(path).state, Recording_State.Rejected);
			recording_delete_close(&v.ui.recording_delete);
			v.ui.recording_delete.scope, v.ui.recording_delete.phase = .Run, .Scanning;
			testing.expect_value(t, recording_delete_queue(&v, path, "container", 0), scene.Status.Ok);
			for step: int = 0; step < 4 && v.ui.recording_delete.phase == .Scanning; step += 1
			{
				recording_delete_step(&v);
			}
			testing.expect_value(t, v.ui.recording_delete.phase, Recording_Delete_Phase.Ready);
			testing.expect_value(t, len(v.ui.recording_delete.targets), 0);
			filename: string = fmt.aprintf("%s/sample.epr", path);
			defer delete(filename);
			testing.expect_value(t, recording_remove(filename).state, Recording_State.Rejected);
		}
		saved_run_fixture_preserved(t, "../outside/run", "external");
		data: []u8;
		data, error = os.read_entire_file("../outside/run/sample.epr", context.allocator);
		defer delete(data);
		testing.expect_value(t, error, nil);
		testing.expect_value(t, string(data), SAVED_RUN_TEST_RECORDING);
	}
}
