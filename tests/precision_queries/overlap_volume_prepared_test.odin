package precision_queries_tests

import "base:runtime"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_overlap_volume_prepared_compound_queries_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	query_shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	for index in 0 ..< 16
	{
		if index < 8
		{
			_ = add_body(t, &scene, shape, {1e9, f64(index) * 0.01, 0});
		}
		else
		{
			_ = add_static(t, &scene, shape, {1e9, f64(index) * 0.01, 0});
		}
	}
	stack: [128]i32;
	hits: [16]entasis.Overlap_Hit;
	volumes: [16]entasis.Volume_Hit;
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	bounds: entasis.Global_Bounds = {min={1e9 - 2, -2, -2}, max={1e9 + 2, 2, 2}};
	_, warm_status := entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, hits[:], stack[:]);
	testing.expect_value(t, warm_status, entasis.Status.Ok);
	_, warm_status = entasis.overlap_global_any_with_context(&scene.query, query_shape, pose, stack[:]);
	testing.expect_value(t, warm_status, entasis.Status.Ok);
	_, warm_status = entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:], stack[:]);
	testing.expect_value(t, warm_status, entasis.Status.Ok);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		// force compound collector exhaustion and then use every owner again.
		// fault/callback state must not survive into the next prepared operation
		count, status := entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, nil, stack[:]);
		testing.expect_value(t, status, entasis.Status.Capacity_Missing);
		testing.expect_value(t, count, 0);
		count, status = entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
		state: entasis.Overlap_State;
		state, status = entasis.overlap_global_any_with_context(&scene.query, query_shape, pose, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, state, entasis.Overlap_State.Intersecting);
		count, status = entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
		// the unchanged float API still uses its original absolute-float frame
		count, status = entasis.overlap_all_with_context(&scene.query, query_shape,
			{position={1e9, 0, 0}, orientation=util.quaternion_identity()}, hits[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
		_, status = entasis.ray_cast_global_any_with_context(&scene.query,
			{start={1e9 - 2, 0, 0}, end={1e9 + 2, 0, 0}}, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

Overlap_Volume_Parallel :: struct
{
	queries: [4]entasis.Query_Context,
	stacks: [4][128]i32,
	hits: [4][16]entasis.Overlap_Hit,
	volumes: [4][16]entasis.Volume_Hit,
	shape: entasis.Shape_Handle,
	errors: [4]int,
}
overlap_volume_parallel_worker :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Overlap_Volume_Parallel = (^Overlap_Volume_Parallel)(boundary.unmanaged_context);
	pose: entasis.Global_Pose = {position={1e9, 0.25, 0}, orientation=util.quaternion_identity()};
	bounds: entasis.Global_Bounds = {min={1e9 - 2, -2, -2}, max={1e9 + 2, 2, 2}};
	for _ in 0 ..< 1024
	{
		count, status := entasis.overlap_global_all_with_context(&state.queries[worker], state.shape, pose, state.hits[worker][:], state.stacks[worker][:]);
		if status != .Ok || count != 16
		{
			state.errors[worker] += 1;
		}
		result: entasis.Overlap_State;
		result, status = entasis.overlap_global_any_with_context(&state.queries[worker], state.shape, pose, state.stacks[worker][:]);
		if status != .Ok || result != .Intersecting
		{
			state.errors[worker] += 1;
		}
		count, status = entasis.volume_global_all_with_context(&state.queries[worker], bounds, state.volumes[worker][:], state.stacks[worker][:]);
		if status != .Ok || count != 16
		{
			state.errors[worker] += 1;
		}
	}
}

@(test)
global_overlap_volume_independent_contexts_read_concurrently :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	for index in 0 ..< 16
	{
		_ = add_static(t, &scene, shape, {1e9, f64(index) * 0.01, 0});
	}
	state: Overlap_Volume_Parallel = {shape=add_shape(t, &scene, .Sphere)};
	for &query in state.queries
	{
		testing.expect_value(t, entasis.query_context_init(&query, &scene.world), entasis.Status.Ok);
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, overlap_volume_parallel_worker, 4, &state), util.Threading_Status.Ok);
	for errors in state.errors
	{
		testing.expect_value(t, errors, 0);
	}
	testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	for &query in state.queries
	{
		testing.expect_value(t, entasis.query_context_destroy(&query), entasis.Status.Ok);
	}
}
