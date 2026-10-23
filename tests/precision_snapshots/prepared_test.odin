package precision_snapshots_tests

import "base:runtime"
import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

@(test)
global_state_prepared_capture_apply_and_capacity_recovery_allocate_nothing :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	snapshot_capture(t, &fixture);
	selection: e.Global_State_Selection = snapshot_selection(&fixture);
	buffers: e.Global_State_Snapshot = snapshot_buffers(&fixture);
	allocations, bytes: i64;
	for iteration in 0 ..< 1028
	{
		if iteration == 4
		{
			allocations = fixture.scene.tracker.total_allocation_count;
			bytes = fixture.scene.tracker.current_memory_allocated;
		}
		counts, status := e.world_apply_global_states(&fixture.scene.world, selection, buffers, .None);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, counts, e.Global_State_Counts{3, 1, 2, 1});
		view, view_status := e.active_body_view(&fixture.scene.world);
		testing.expect_value(t, view_status, e.Status.Ok);
		counts, status = e.world_export_global_states(&fixture.scene.world, selection, buffers);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, counts, e.Global_State_Counts{3, 1, 2, 1});
		testing.expect(t, e.body_view_valid(&fixture.scene.world, view));
		short: e.Global_State_Snapshot = buffers;
		short.constraint_targets = short.constraint_targets[:0];
		counts, status = e.world_export_global_states(&fixture.scene.world, selection, short);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, counts, e.Global_State_Counts{});
	}
	testing.expect_value(t, fixture.scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, fixture.scene.tracker.current_memory_allocated, bytes);
	scene_dispose(t, &fixture.scene);
}

Snapshot_Parallel_State :: struct
{
	fixture: ^Snapshot_Scene,
	body_outputs: [4][3]e.Global_Body_State,
	static_outputs: [4][1]e.Global_Static_State,
	constraint_outputs: [4][2]e.Global_Constraint_Target_State,
	kinematic_outputs: [4][1]e.Global_Kinematic_Target_State,
	errors, completed: [4]int,
}

snapshot_parallel_worker :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Snapshot_Parallel_State = (^Snapshot_Parallel_State)(boundary.unmanaged_context);
	fixture: ^Snapshot_Scene = state.fixture;
	output: e.Global_State_Snapshot = {state.body_outputs[worker][:], state.static_outputs[worker][:],
		state.constraint_outputs[worker][:], state.kinematic_outputs[worker][:]};
	selection: e.Global_State_Selection = snapshot_selection(fixture);
	errors: int;
	for _ in 0 ..< 1024
	{
		counts, status := e.world_export_global_states(&fixture.scene.world, selection, output);
		if status != .Ok || counts != (e.Global_State_Counts{3, 1, 2, 1})
		{
			errors += 1;
		}
		for value, index in output.bodies
		{
			if value != fixture.body_states[index]
			{
				errors += 1;
			}
		}
		if output.statics[0] != fixture.static_states[0] ||
		output.constraint_targets[0] != fixture.constraint_states[0] ||
		output.constraint_targets[1] != fixture.constraint_states[1] ||
		output.kinematic_targets[0] != fixture.kinematic_states[0]
		{
			errors += 1;
		}
	}
	state.errors[worker] = errors;
	state.completed[worker] = 1024;
}

@(test)
global_state_parallel_capture_uses_disjoint_caller_buffers :: proc(t: ^testing.T)
{
	fixture: Snapshot_Scene;
	snapshot_scene_initialize(t, &fixture);
	sleep_body(t, &fixture.scene, fixture.bodies[0]);
	snapshot_capture(t, &fixture);
	state: Snapshot_Parallel_State = {fixture=&fixture};
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, snapshot_parallel_worker, 4, &state), util.Threading_Status.Ok);
	for errors in state.errors
	{
		testing.expect_value(t, errors, 0);
	}
	for completed in state.completed
	{
		testing.expect_value(t, completed, 1024);
	}
	testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	scene_dispose(t, &fixture.scene);
}
