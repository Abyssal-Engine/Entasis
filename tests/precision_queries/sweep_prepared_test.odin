package precision_queries_tests

import "base:runtime"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_sweep_prepared_compound_queries_do_not_allocate :: proc(t: ^testing.T)
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
			_ = add_body(t, &scene, shape, {1e9, f64(index)*0.01, 0});
		}
		else
		{
			_ = add_static(t, &scene, shape, {1e9, f64(index)*0.01, 0});
		}
	}
	stack: [128]i32;
	hits: [16]entasis.Global_Sweep_Hit;
	legacy: [16]entasis.Sweep_Hit;
	pose: entasis.Global_Pose = {position={1e9 - 3, 0.25, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	_, status := entasis.sweep_global_all_with_context(&scene.query, query_shape, pose, velocity, 4, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	_, status = entasis.sweep_global_any_with_context(&scene.query, query_shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	_, status = entasis.sweep_global_closest_with_context(&scene.query, query_shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	_, status = entasis.sweep_all_with_context(&scene.query, query_shape, {position={1e9, 0.25, 0}, orientation=util.quaternion_identity()}, velocity, 4, legacy[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		count: int;
		count, status = entasis.sweep_global_all_with_context(&scene.query, query_shape, pose, velocity, 4, nil, stack[:]);
		testing.expect_value(t, status, entasis.Status.Capacity_Missing);
		testing.expect_value(t, count, 0);
		count, status = entasis.sweep_global_all_with_context(&scene.query, query_shape, pose, velocity, 4, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
		result: entasis.Query_Hit_State;
		result, status = entasis.sweep_global_any_with_context(&scene.query, query_shape, pose, velocity, 4, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, result, entasis.Query_Hit_State.Hit);
		hit: entasis.Global_Sweep_Hit;
		hit, status = entasis.sweep_global_closest_with_context(&scene.query, query_shape, pose, velocity, 4, stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, hit.sweep.state, entasis.Sweep_Hit_State.Hit);
		count, status = entasis.sweep_all_with_context(&scene.query, query_shape,
			{position={1e9, 0.25, 0}, orientation=util.quaternion_identity()}, velocity, 4, legacy[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 16);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

Sweep_Parallel :: struct
{
	queries: [4]entasis.Query_Context,
	stacks: [4][128]i32,
	hits: [4][16]entasis.Global_Sweep_Hit,
	shape: entasis.Shape_Handle,
	errors: [4]int,
}
sweep_parallel_worker :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Sweep_Parallel = (^Sweep_Parallel)(boundary.unmanaged_context);
	pose: entasis.Global_Pose = {position={1e9 - 3, 0.25, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	for _ in 0 ..< 1024
	{
		count, status := entasis.sweep_global_all_with_context(&state.queries[worker], state.shape, pose, velocity, 4, state.hits[worker][:], state.stacks[worker][:]);
		if status != .Ok || count != 16
		{
			state.errors[worker] += 1;
		}
		result: entasis.Query_Hit_State;
		result, status = entasis.sweep_global_any_with_context(&state.queries[worker], state.shape, pose, velocity, 4, state.stacks[worker][:]);
		if status != .Ok || result != .Hit
		{
			state.errors[worker] += 1;
		}
		hit: entasis.Global_Sweep_Hit;
		hit, status = entasis.sweep_global_closest_with_context(&state.queries[worker], state.shape, pose, velocity, 4, state.stacks[worker][:]);
		if status != .Ok || hit.sweep.state != .Hit
		{
			state.errors[worker] += 1;
		}
	}
}

@(test)
global_sweep_independent_contexts_read_concurrently :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	for index in 0 ..< 16
	{
		_ = add_static(t, &scene, shape, {1e9, f64(index)*0.01, 0});
	}
	state: Sweep_Parallel = {shape=add_shape(t, &scene, .Sphere)};
	for &query in state.queries
	{
		testing.expect_value(t, entasis.query_context_init(&query, &scene.world), entasis.Status.Ok);
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, sweep_parallel_worker, 4, &state), util.Threading_Status.Ok);
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
