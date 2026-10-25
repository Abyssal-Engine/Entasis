package public_api_tests

import "base:runtime"
import "core:sync"
import "core:testing"
import "core:thread"
import e "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Query_Batch_Dispatch_Callback :: struct
{
	caller_thread, calls, wrong_thread: int,
}

query_batch_dispatch_reject :: proc "contextless" (
	raw: rawptr, _collidable: e.Collidable_Reference,
) -> bool
{
	state: ^Query_Batch_Dispatch_Callback = cast(^Query_Batch_Dispatch_Callback)raw;
	state.calls += 1;
	if sync.current_thread_id() != state.caller_thread
	{
		state.wrong_thread += 1;
	}
	return false;
}

Query_Batch_Dispatch_Busy :: struct
{
	world: ^e.World,
	queries: []e.Query,
	results: []e.Query_Result,
	status: e.Status,
}

query_batch_dispatch_while_busy :: proc "contextless" (
	_index: int, boundary: ^util.Thread_Dispatcher_Boundary,
)
{
	state: ^Query_Batch_Dispatch_Busy = cast(^Query_Batch_Dispatch_Busy)boundary.unmanaged_context;
	state.status = e.query_batch(state.world, state.queries, state.results);
}

query_batch_dispatch_external_failure :: proc "contextless" (
	boundary: ^util.Thread_Dispatcher_Boundary, _worker: util.Dispatcher_Worker_Proc,
	_count: int, _payload: rawptr,
) -> util.Threading_Status
{
	calls: ^int = cast(^int)boundary.dispatcher;
	calls^ += 1;
	return .Capacity_Missing;
}

query_batch_dispatch_inputs :: proc(queries: []e.Query)
{
	filters: [6]e.Query_Filter = {
		{},
		{include=e.COLLIDABLE_STATIC},
		{include=e.COLLIDABLE_BODIES},
		{include=e.COLLIDABLE_ALL, exclude=e.COLLIDABLE_DYNAMIC},
		{include=e.COLLIDABLE_KINEMATIC},
		{exclude=e.COLLIDABLE_BODIES},
	};
	for &query, index in queries
	{
		y: f32 = 100 if index % 11 == 3 else 0;
		query = e.query_ray_closest(e.ray({-10, y, 0}, {1, 0, 0}, 30), filters[(index / 37) % len(filters)]);
	}
}

query_batch_dispatch_expect_guard :: proc(t: ^testing.T, result, guard: e.Query_Result)
{
	testing.expect_value(t, result.status, guard.status);
	testing.expect_value(t, result.count, guard.count);
	testing.expect_value(t, result.hit, guard.hit);
	testing.expect_value(t, result.ray_hit.t, guard.ray_hit.t);
	testing.expect_value(t, result.ray_hit.child_index, guard.ray_hit.child_index);
}

Query_Batch_Dispatch_Concurrent :: struct
{
	world: ^e.World,
	queries: []e.Query,
	reference: []e.Query_Result,
	results: [785]e.Query_Result,
	start: ^sync.Barrier,
	failures: int,
}

query_batch_dispatch_concurrent :: proc(host: ^thread.Thread)
{
	context = runtime.default_context();
	state: ^Query_Batch_Dispatch_Concurrent = cast(^Query_Batch_Dispatch_Concurrent)host.data;
	for _ in 0 ..< 2
	{
		for &result in state.results
		{
			result.status = .Disposed;
		}
		_ = sync.barrier_wait(state.start);
		if e.query_batch(state.world, state.queries, state.results[:]) != .Not_Found
		{
			state.failures += 1;
		}
		for result, index in state.results
		{
			expected: e.Query_Result = state.reference[index];
			if result.status != expected.status || result.hit != expected.hit || result.count != expected.count ||
			result.ray_hit.collidable != expected.ray_hit.collidable ||
			result.ray_hit.child_index != expected.ray_hit.child_index || result.ray_hit.ray_id != expected.ray_hit.ray_id ||
			abs(result.ray_hit.t - expected.ray_hit.t) > 2e-4 * max(f32(1), abs(expected.ray_hit.t)) ||
			util.vector3_distance(result.ray_hit.location, expected.ray_hit.location) > 2e-4 * max(f32(1), util.vector3_length(expected.ray_hit.location)) ||
			util.vector3_distance(result.ray_hit.normal, expected.ray_hit.normal) > 2e-4
			{
				state.failures += 1;
			}
		}
	}
}

@(test)
query_batch_owned_dispatch_preserves_results_and_serial_fallbacks :: proc(t: ^testing.T)
{
	queries: [785]e.Query;
	results: [787]e.Query_Result;
	guard: e.Query_Result = {status=.Disposed, count=-91, ray_hit={t=-123, child_index=91}};
	for workers in ([2]int{1, 3})
	{
		scene: Query_Test_Scene;
		if !query_test_scene_init(t, &scene, workers)
		{
			return;
		}
		defer query_test_scene_destroy(&scene);
		query_batch_dispatch_inputs(queries[:]);
		for &result in results
		{
			result = guard;
		}
		testing.expect_value(t, e.query_batch(&scene.world, queries[:], results[1:786]), e.Status.Not_Found);
		for query, index in queries
		{
			query_expect_scalar(t, &scene.world, query, results[index + 1]);
		}
		query_batch_dispatch_expect_guard(t, results[0], guard);
		query_batch_dispatch_expect_guard(t, results[786], guard);
		if workers == 1
		{
			continue;
		}
		baseline: [785]e.Query_Result;
		for &result, index in baseline
		{
			result = results[index + 1];
		}

		callback: Query_Batch_Dispatch_Callback = {caller_thread=sync.current_thread_id()};
		callback_input: e.Query = queries[400];
		queries[400] = e.query_ray_closest(e.ray({-10, 0, 0}, {1, 0, 0}, 30),
			{allow=query_batch_dispatch_reject, user_context=&callback});
		results[401].status = .Disposed;
		results[785].status = .Disposed;
		testing.expect_value(t, e.query_batch(&scene.world, queries[:], results[1:786]), e.Status.Not_Found);
		testing.expect(t, callback.calls > 0);
		testing.expect_value(t, callback.wrong_thread, 0);
		testing.expect_value(t, results[401].status, e.Status.Not_Found);
		testing.expect_value(t, results[401].count, i32(0));
		testing.expect_value(t, results[785].status, baseline[784].status);
		queries[400] = callback_input;

		all_input: e.Query = queries[600];
		ray_hits, expected_hits: [3]e.Ray_Hit;
		ray: e.Ray = e.ray({-10, 0, 0}, {1, 0, 0}, 30);
		count: int;
		all_status: e.Status;
		count, all_status = e.ray_cast_all(&scene.world, ray, expected_hits[:]);
		testing.expect_value(t, all_status, e.Status.Ok);
		testing.expect_value(t, count, 3);
		queries[600] = e.query_ray_all(ray, e.query_output(0, 3));
		scratch: e.Query_Scratch = e.query_scratch(ray_hits[:], nil, nil);
		results[601].status = .Disposed;
		results[785].status = .Disposed;
		testing.expect_value(t, e.query_batch(&scene.world, queries[:], results[1:786], &scratch), e.Status.Not_Found);
		testing.expect_value(t, results[601].status, e.Status.Ok);
		testing.expect_value(t, results[601].count, i32(3));
		testing.expect_value(t, ray_hits, expected_hits);
		testing.expect_value(t, results[785].status, baseline[784].status);
		queries[600] = all_input;

		first: e.Query = queries[0];
		queries[0] = e.query_ray_closest(e.ray({}, {}, 30));
		queries[700] = e.query_ray_closest(e.ray({}, {}, 30));
		queries[784] = first;
		for attempt in 0 ..< 2
		{
			results[1].status = .Disposed;
			results[701].status = .Disposed;
			results[785].status = .Disposed;
			testing.expect_value(t, e.query_batch(&scene.world, queries[:], results[1:786]),
				e.Status.Invalid_Argument if attempt == 0 else e.Status.Not_Found);
			testing.expect_value(t, results[1].status, e.Status.Invalid_Argument if attempt == 0 else e.Status.Ok);
			testing.expect_value(t, results[701].status, e.Status.Invalid_Argument);
			testing.expect_value(t, results[785].status, e.Status.Ok);
			testing.expect_value(t, results[785].ray_hit, baseline[0].ray_hit);
			query_batch_dispatch_expect_guard(t, results[0], guard);
			query_batch_dispatch_expect_guard(t, results[786], guard);
			queries[0] = first;
		}

		query_batch_dispatch_inputs(queries[:]);
		boundary: ^e.Dispatcher;
		status: e.Status;
		boundary, status = e.world_borrow_dispatcher(&scene.world);
		testing.expect_value(t, status, e.Status.Ok);
		for &result in results[1:786]
		{
			result.status = .Disposed;
		}
		busy: Query_Batch_Dispatch_Busy = {world=&scene.world, queries=queries[:], results=results[1:786]};
		testing.expect_value(t, boundary.dispatch(boundary, query_batch_dispatch_while_busy, 1, &busy), util.Threading_Status.Ok);
		testing.expect_value(t, busy.status, e.Status.Not_Found);
		for result, index in results[1:786]
		{
			testing.expect_value(t, result.status, baseline[index].status);
			testing.expect_value(t, result.count, baseline[index].count);
			testing.expect_value(t, result.hit, baseline[index].hit);
			testing.expect_value(t, result.ray_hit, baseline[index].ray_hit);
		}
		start: sync.Barrier;
		sync.barrier_init(&start, 2);
		calls: [2]Query_Batch_Dispatch_Concurrent;
		hosts: [2]^thread.Thread;
		for &host, index in hosts
		{
			calls[index].world = &scene.world;
			calls[index].queries = queries[:];
			calls[index].reference = baseline[:];
			calls[index].start = &start;
			host = thread.create(query_batch_dispatch_concurrent);
			if !testing.expect(t, host != nil)
			{
				for created in hosts
				{
					if created != nil
					{
						thread.destroy(created);
					}
				}
				return;
			}
			host.data = &calls[index];
		}
		for host in hosts
		{
			thread.start(host);
		}
		for host in hosts
		{
			thread.join(host);
			thread.destroy(host);
		}
		for &call in calls
		{
			testing.expect_value(t, call.failures, 0);
		}

		testing.expect_value(t, e.static_remove(&scene.world, scene.static_body, .None), e.Status.Ok);
		for _ in 0 ..< 301
		{
			add_status: e.Status;
			_, add_status = e.static_add(&scene.world, e.static_body(scene.box_shape, e.pose({2, 0, 0})), .None);
			testing.expect_value(t, add_status, e.Status.Ok);
		}
		// fixture-only access creates a valid depth beyond the inline traversal stack.
		simulation: ^physics.Simulation = cast(^physics.Simulation)rawptr(scene.world);
		query_make_deep_tree(&simulation.broad_phase.static_tree);
		for &query, index in queries
		{
			y: f32 = 100 if index % 11 == 3 else 0;
			query = e.query_ray_closest(e.ray({0, y, 0}, {1, 0, 0}, 10), {include=e.COLLIDABLE_STATIC});
		}
		deep_hits: [2]e.Ray_Hit;
		deep_statuses: [2]e.Status;
		for query_index, index in ([2]int{0, 3})
		{
			deep_hits[index], deep_statuses[index] = e.ray_cast_closest(
				&scene.world, queries[query_index].ray_data.ray, queries[query_index].ray_data.filter,
			);
		}
		for _ in 0 ..< 2
		{
			for &result in results[1:786]
			{
				result.status = .Disposed;
			}
			testing.expect_value(t, e.query_batch(&scene.world, queries[:], results[1:786]), e.Status.Not_Found);
			for result, index in results[1:786]
			{
				reference: int = 1 if index % 11 == 3 else 0;
				testing.expect_value(t, result.status, deep_statuses[reference]);
				testing.expect_value(t, result.hit, deep_statuses[reference] == .Ok);
				testing.expect_value(t, result.count, i32(deep_statuses[reference] == .Ok));
				if result.status == .Ok
				{
					testing.expect_value(t, result.ray_hit.collidable, deep_hits[reference].collidable);
					testing.expect_value(t, result.ray_hit.child_index, deep_hits[reference].child_index);
					testing.expect(t, abs(result.ray_hit.t - deep_hits[reference].t) <= 2e-4);
					testing.expect(t, util.vector3_distance(result.ray_hit.location, deep_hits[reference].location) <= 2e-4);
					testing.expect(t, util.vector3_distance(result.ray_hit.normal, deep_hits[reference].normal) <= 2e-4);
				}
			}
		}
		query_batch_dispatch_expect_guard(t, results[0], guard);
		query_batch_dispatch_expect_guard(t, results[786], guard);
	}

	external_calls: int;
	external: e.Dispatcher = {
		dispatcher=&external_calls, dispatch=query_batch_dispatch_external_failure, worker_count=3,
	};
	world: e.World;
	description: e.World_Description = small_world_description();
	description.threading.external_dispatcher = &external;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(&world, e.box(2, 2, 2));
	testing.expect_value(t, status, e.Status.Ok);
	_, status = e.static_add(&world, e.static_body(shape, e.pose()), .None);
	testing.expect_value(t, status, e.Status.Ok);
	query_batch_dispatch_inputs(queries[:]);
	testing.expect_value(t, e.query_batch(&world, queries[:], results[1:786]), e.Status.Not_Found);
	testing.expect_value(t, external_calls, 0);
	for query, index in queries
	{
		query_expect_scalar(t, &world, query, results[index + 1]);
	}
	query_batch_dispatch_expect_guard(t, results[0], guard);
	query_batch_dispatch_expect_guard(t, results[786], guard);
}
