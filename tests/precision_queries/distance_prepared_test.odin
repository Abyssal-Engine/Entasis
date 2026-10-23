package precision_queries_tests

import "base:runtime"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_distance_prepared_queries_reuse_epa_without_allocation :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	hull: e.Shape_Handle = add_shape(t, &scene, .Hull);
	compound: e.Shape_Handle = add_shape(t, &scene, .Big_Compound);
	body: p.Body_Handle = add_body(t, &scene, compound, {1e9, -1e9, 1e9});
	ref: e.Collidable_Reference;
	ref, _ = p.collidable_reference_create(.Dynamic, int(body.value));
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query, {512, 1024, 1024}), e.Status.Ok);
	a: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 0.75;
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		result: e.Global_Shape_Distance_Result;
		status: e.Status;
		result, status = e.shape_closest_point_global_with_context(&scene.query, {a.position.x + 4, a.position.y, a.position.z}, compound, a);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.geometry.distance, f32(3));
		result, status = e.shape_distance_global_with_context(&scene.query, sphere, b, compound, a);
		testing.expect_value(t, status, e.Status.Ok);
		result, status = e.shape_penetration_global_with_context(&scene.query, hull, a, hull, b);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(result.geometry.depth - 0.25) < 1e-5);
		correction: e.Shape_Correction_Result;
		correction, status = e.shape_depenetrate_global_with_context(&scene.query, sphere, b, compound, a);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, correction.translation.x > 0);
		_, _, status = e.collidable_shape_global_pose_with_context(&scene.query, ref);
		testing.expect_value(t, status, e.Status.Ok);
		// a failed high request restores the same owner for both API families
		result, status = e.shape_penetration_global_with_context(&scene.query, compound, a, hull, b);
		testing.expect(t, status != .Ok);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		local: e.Shape_Distance_Result;
		local, status = e.shape_penetration_with_context(&scene.query, hull, e.pose(), hull, e.pose({0.75, 0, 0}));
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, abs(local.geometry.depth - 0.25) < 1e-5);
		testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query, {4, 4, 3}), e.Status.Ok);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

Distance_Parallel :: struct
{
	queries: [4]e.Query_Context,
	sphere, hull, compound: e.Shape_Handle,
	errors: [4]int,
}
distance_parallel_worker :: proc "contextless" (worker: int, boundary: ^util.Thread_Dispatcher_Boundary)
{
	context = runtime.default_context();
	state: ^Distance_Parallel = (^Distance_Parallel)(boundary.unmanaged_context);
	query: ^e.Query_Context = &state.queries[worker];
	a: e.Global_Pose = {position={1e9 + 0.125 + 16*f64(worker), -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 0.75;
	for _ in 0 ..< 1024
	{
		result: e.Global_Shape_Distance_Result;
		status: e.Status;
		result, status = e.shape_closest_point_global_with_context(query, {a.position.x + 4, a.position.y, a.position.z}, state.compound, a);
		if status != .Ok || result.geometry.distance != 3
		{
			state.errors[worker] += 1;
		}
		result, status = e.shape_distance_global_with_context(query, state.sphere, b, state.compound, a);
		if status != .Ok || result.geometry.state != .Intersecting
		{
			state.errors[worker] += 1;
		}
		result, status = e.shape_penetration_global_with_context(query, state.hull, a, state.hull, b);
		if status != .Ok || abs(result.geometry.depth - 0.25) > 1e-5
		{
			state.errors[worker] += 1;
		}
		correction: e.Shape_Correction_Result;
		correction, status = e.shape_depenetrate_global_with_context(query, state.sphere, b, state.compound, a);
		if status != .Ok || correction.translation.x <= 0
		{
			state.errors[worker] += 1;
		}
	}
}

@(test)
global_distance_independent_contexts_and_borrowed_scratch :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Distance_Parallel = {sphere=add_shape(t, &scene, .Sphere), hull=add_shape(t, &scene, .Hull), compound=add_shape(t, &scene, .Big_Compound)};
	borrowed: e.Buffer_Pool;
	testing.expect_value(t, e.buffer_pool_init(&borrowed, 16384), e.Status.Ok);
	for &query, index in state.queries
	{
		testing.expect_value(t, e.query_context_init(&query, &scene.world, borrowed_pool=&borrowed if index == 0 else nil), e.Status.Ok);
		testing.expect_value(t, e.distance_query_reserve_with_context(&query, {512, 1024, 1024}), e.Status.Ok);
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 4096), util.Threading_Status.Ok);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(boundary, distance_parallel_worker, 4, &state), util.Threading_Status.Ok);
	for errors in state.errors
	{
		testing.expect_value(t, errors, 0);
	}
	testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
	for &query in state.queries
	{
		testing.expect_value(t, e.query_context_destroy(&query), e.Status.Ok);
	}
	testing.expect_value(t, borrowed.state, util.Pool_State.Ready);
	testing.expect_value(t, e.buffer_pool_destroy(&borrowed), e.Status.Ok);
}
