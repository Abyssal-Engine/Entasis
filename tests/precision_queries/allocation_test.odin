package precision_queries_tests

import "base:runtime"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_query_and_pose_export_prepared_loop_has_no_allocations :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	references: [16]entasis.Collidable_Reference;
	for &reference, index in references
	{
		handle: physics.Static_Handle = add_static(t, &scene, shape, {1e9 + f64(index * 4), 0, 0});
		reference, _ = physics.collidable_reference_create(.Static, int(handle.value));
	}
	stack: [128]i32;
	hits: [64]entasis.Global_Ray_Hit;
	poses: [16]entasis.Global_Pose;
	segment: physics.World_Precision_Segment = {start={1e9 - 4, 0, 0}, end={1e9 + 70, 0, 0}};
	_, warm_status := entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:], stack[:]);
	testing.expect_value(t, warm_status, entasis.Status.Ok);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		count, status := entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 32);
		_, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		_, status = entasis.ray_cast_global_any_with_context(&scene.query, segment, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		count, status = entasis.world_export_global_poses(&scene.world, references[:], poses[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

Parallel_Reads :: struct
{
	queries: [4]entasis.Query_Context,
	stacks: [4][128]i32,
	errors: [4]int,
}
parallel_read :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Parallel_Reads = (^Parallel_Reads)(boundary.unmanaged_context);
	segment: physics.World_Precision_Segment = {start={1e9 - 2, 0.25, 0}, end={1e9 + 2, 0.25, 0}};
	for _ in 0 ..< 1024
	{
		hit, status := entasis.ray_cast_global_closest_with_context(&state.queries[worker], segment, state.stacks[worker][:]);
		if status != .Ok || hit.location.x != 1e9 - 0.5
		{
			state.errors[worker] += 1;
		}
	}
}

@(test)
global_independent_query_contexts_read_concurrently :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	_ = add_body(t, &scene, shape, {1e9, 0, 0});
	state: Parallel_Reads;
	for &query in state.queries
	{
		testing.expect_value(t, entasis.query_context_init(&query, &scene.world), entasis.Status.Ok);
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, parallel_read, 4, &state), util.Threading_Status.Ok);
	for error in state.errors
	{
		testing.expect_value(t, error, 0);
	}
	testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	for &query in state.queries
	{
		testing.expect_value(t, entasis.query_context_destroy(&query), entasis.Status.Ok);
	}
}
