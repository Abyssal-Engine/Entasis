package precision_queries_tests

import "base:runtime"
import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

Batch_Output_Storage :: struct
{
	rays: [16]e.Global_Ray_Hit,
	sweeps: [16]e.Global_Sweep_Hit,
	overlaps: [16]e.Overlap_Hit,
	volumes: [16]e.Volume_Hit,
	stack: [128]i32,
}

batch_bind_output :: proc "contextless" (storage: ^Batch_Output_Storage) -> e.Global_Query_Scratch
{
	return {storage.rays[:], storage.sweeps[:], storage.overlaps[:], storage.volumes[:], storage.stack[:]};
}

@(test)
global_batches_prepared_compound_work_and_error_recovery_allocate_nothing :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	probe: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	hull: e.Shape_Handle = add_shape(t, &scene, .Hull);
	compound: e.Shape_Handle = add_shape(t, &scene, .Big_Compound);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	_ = add_static(t, &scene, compound, origin);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query, {512, 1024, 1024}), e.Status.Ok);
	queries: [9]e.Global_Query = batch_spatial_inputs(probe, origin);
	for index in 0 ..< 3
	{
		queries[index].ray_data.segment.start.x += 0.5;
		queries[index].ray_data.segment.end.x += 0.5;
		queries[index + 3].sweep_data.pose.position.x += 0.5;
	}
	queries[6].overlap_data.pose.position.x += 0.5;
	queries[7].overlap_data.pose.position.x += 0.5;
	distance_queries: [4]e.Global_Distance_Query = batch_distance_inputs(hull, origin);
	distance_queries[0].shape_b = compound;
	distance_queries[0].pose_b = {position=origin, orientation=util.quaternion_identity()};
	distance_queries[0].point.x = origin.x + 4;
	for index in ([2]int{1, 3})
	{
		distance_queries[index].shape_a = probe;
		distance_queries[index].shape_b = compound;
		distance_queries[index].pose_a.position.x = origin.x + 0.75;
		distance_queries[index].pose_b.position = origin;
	}
	output: Batch_Output_Storage;
	scratch: e.Global_Query_Scratch = batch_bind_output(&output);
	results: [9]e.Global_Query_Result;
	distance_results: [4]e.Global_Distance_Query_Result;
	// prepared owner: any one-time context pool growth happens before measurement
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Ok);
	testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, distance_queries[:], distance_results[:]), e.Status.Ok);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Ok);
		for result in results
		{
			testing.expect(t, result.count > 0);
		}
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, distance_queries[:], distance_results[:]), e.Status.Ok);
		testing.expect_value(t, distance_results[0].shape.geometry.distance, f32(3));
		testing.expect(t, distance_results[3].correction.translation.x > 0);
		queries[2].ray_data.output.capacity = 0;
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Capacity_Missing);
		testing.expect_value(t, results[2].count, i32(0));
		testing.expect_value(t, results[8].status, e.Status.Ok);
		queries[2].ray_data.output.capacity = 14;
		distance_queries[1].kind = e.Distance_Query_Kind(255);
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, distance_queries[:], distance_results[:]), e.Status.Invalid_Argument);
		testing.expect_value(t, distance_results[1].shape, e.Global_Shape_Distance_Result{});
		testing.expect_value(t, distance_results[3].status, e.Status.Ok);
		distance_queries[1].kind = .Distance;
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

Batch_Parallel_State :: struct
{
	contexts: [4]e.Query_Context,
	queries: [36]e.Global_Query,
	results: [36]e.Global_Query_Result,
	distance_queries: [16]e.Global_Distance_Query,
	distance_results: [16]e.Global_Distance_Query_Result,
	outputs: [4]Batch_Output_Storage,
	errors: [4]int,
}

batch_parallel_worker :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Batch_Parallel_State = (^Batch_Parallel_State)(boundary.unmanaged_context);
	query: ^e.Query_Context = &state.contexts[worker];
	start: int = worker * 9;
	distance_start: int = worker * 4;
	scratch: e.Global_Query_Scratch = batch_bind_output(&state.outputs[worker]);
	errors: int;
	for _ in 0 ..< 1024
	{
		if e.query_batch_global_with_context(query, state.queries[start:start + 9], state.results[start:start + 9], scratch) != .Ok
		{
			errors += 1;
		}
		for result in state.results[start:start + 9]
		{
			if result.status != .Ok || result.count != 1 || result.hit != .Hit
			{
				errors += 1;
			}
		}
		if e.distance_query_batch_global_with_context(query, state.distance_queries[distance_start:distance_start + 4],
			state.distance_results[distance_start:distance_start + 4]) != .Ok
		{
			errors += 1;
		}
		if abs(state.distance_results[distance_start + 1].shape.geometry.distance - 0.002) > 0.000002
		{
			errors += 1;
		}
	}
	// only one final store per worker, not a shared counter in the hot loop
	state.errors[worker] = errors;
}

@(test)
global_batches_parallel_partitioned_spans_and_borrowed_pool_ownership :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	_ = add_static(t, &scene, shape, origin);
	state: Batch_Parallel_State;
	borrowed: e.Buffer_Pool;
	testing.expect_value(t, e.buffer_pool_init(&borrowed, 16384), e.Status.Ok);
	for &query, worker in state.contexts
	{
		testing.expect_value(t, e.query_context_init(&query, &scene.world, borrowed_pool=&borrowed if worker == 0 else nil), e.Status.Ok);
		testing.expect_value(t, e.distance_query_reserve_with_context(&query), e.Status.Ok);
		inputs: [9]e.Global_Query = batch_spatial_inputs(shape, origin);
		for input, index in inputs
		{
			state.queries[worker * 9 + index] = input;
		}
		distance_inputs: [4]e.Global_Distance_Query = batch_distance_inputs(shape, {origin.x + 16*f64(worker), origin.y, origin.z});
		for input, index in distance_inputs
		{
			state.distance_queries[worker * 4 + index] = input;
		}
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, batch_parallel_worker, 4, &state), util.Threading_Status.Ok);
	for errors in state.errors
	{
		testing.expect_value(t, errors, 0);
	}
	testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	for &query in state.contexts
	{
		testing.expect_value(t, e.query_context_destroy(&query), e.Status.Ok);
	}
	testing.expect_value(t, borrowed.state, util.Pool_State.Ready);
	testing.expect_value(t, e.buffer_pool_destroy(&borrowed), e.Status.Ok);
}
