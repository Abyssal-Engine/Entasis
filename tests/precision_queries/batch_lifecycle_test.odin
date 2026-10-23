package precision_queries_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_spatial_batch_growth_sleep_shift_reuse_and_stale_stack_admission :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, shape, origin);
	queries: [9]e.Global_Query = batch_spatial_inputs(shape, origin);
	results: [9]e.Global_Query_Result;
	storage: Batch_Output_Storage;
	scratch: e.Global_Query_Scratch = batch_bind_output(&storage);
	original_requirement: int;
	original_requirement, _ = e.global_bounds_stack_requirement(&scene.world);
	scratch.traversal_stack = storage.stack[:original_requirement];
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Ok);
	for index in 0 ..< 32
	{
		_ = add_static(t, &scene, shape, {origin.x + 128 + 16*f64(index), origin.y, origin.z});
	}
	grown_requirement: int;
	grown_requirement, _ = e.global_bounds_stack_requirement(&scene.world);
	testing.expect(t, grown_requirement > original_requirement);
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Capacity_Missing);
	for result in results
	{
		testing.expect_value(t, result.status, e.Status.Capacity_Missing);
		testing.expect_value(t, result.count, i32(0));
	}
	scratch.traversal_stack = storage.stack[:];
	frame: p.Origin_Frame;
	for phase in 0 ..< 4
	{
		if phase == 1
		{
			sleep_body(t, &scene, body);
		}
		if phase == 2
		{
			testing.expect_value(t, e.world_shift_origin(&scene.world, &frame, {1e9, -1e9, 1e9}), e.Status.Ok);
			queries = batch_spatial_inputs(shape, {0.125, 0, 0});
		}
		if phase == 3
		{
			testing.expect_value(t, e.body_remove(&scene.world, body), e.Status.Ok);
			replacement: e.Body_Handle = add_body(t, &scene, shape, {0.125, 0, 0});
			testing.expect_value(t, replacement, body);
		}
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Ok);
		for result in results
		{
			testing.expect_value(t, result.count, i32(1));
		}
		ref: e.Collidable_Reference;
		ref, _ = p.collidable_reference_create(.Dynamic, int(body.value));
		testing.expect_value(t, results[1].ray_hit.collidable, ref);
		if phase == 1 || phase == 2
		{
			location: p.Body_Memory_Location;
			location, _ = p.bodies_resolve(&scene.simulation.bodies, body);
			testing.expect(t, location.set_index > 0);
		}
	}
}

@(test)
global_batch_failures_restore_context_for_existing_float_batches :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	_ = add_static(t, &scene, shape, {});
	queries: [1]e.Global_Query = {{kind=e.Global_Query_Kind(255)}};
	results: [1]e.Global_Query_Result;
	distances: [1]e.Global_Distance_Query = {{kind=e.Distance_Query_Kind(255)}};
	distance_results: [1]e.Global_Distance_Query_Result;
	local_queries: [1]e.Query = {e.query_ray_closest(e.ray({-2, 0, 0}, {1, 0, 0}, 4))};
	local_results: [1]e.Query_Result;
	local_distances: [1]e.Distance_Query = {{kind=.Distance, shape_a=shape, shape_b=shape,
		pose_a=e.pose(), pose_b=e.pose({2, 0, 0}), settings={1e-5, 1e-5, 128}}};
	local_distance_results: [1]e.Distance_Query_Result;
	for _ in 0 ..< 32
	{
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:]), e.Status.Invalid_Argument);
		testing.expect_value(t, e.query_batch_with_context(&scene.query, local_queries[:], local_results[:]), e.Status.Ok);
		testing.expect_value(t, local_results[0].count, i32(1));
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, distances[:], distance_results[:]), e.Status.Invalid_Argument);
		testing.expect_value(t, e.distance_query_batch_with_context(&scene.query, local_distances[:], local_distance_results[:]), e.Status.Ok);
		testing.expect_value(t, local_distance_results[0].shape.geometry.distance, f32(1));
	}
}
