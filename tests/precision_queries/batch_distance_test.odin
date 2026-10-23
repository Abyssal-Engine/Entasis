package precision_queries_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

batch_distance_inputs :: proc "contextless" (
	shape: e.Shape_Handle, origin: e.Global_Position,
) -> [4]e.Global_Distance_Query
{
	a: e.Global_Pose = {position=origin, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 0.998;
	query: e.Global_Distance_Query = {shape_a=shape, shape_b=shape, pose_a=a, pose_b=b,
		point={origin.x + 2, origin.y, origin.z}, settings={1e-5, 1e-5, 128}};
	queries: [4]e.Global_Distance_Query;
	for &item, index in queries
	{
		item = query;
		item.kind = e.Distance_Query_Kind(index);
	}
	queries[1].pose_b.position.x = origin.x + 1.002;
	return queries;
}

@(test)
global_distance_batch_matches_single_queries_and_independent_millimeter_oracles :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query), e.Status.Ok);
	for origin in ([4]f64{0, 1e3, 1e6, 1e9})
	{
		queries: [4]e.Global_Distance_Query = batch_distance_inputs(shape, {origin + 0.125, -origin, origin});
		results: [5]e.Global_Distance_Query_Result;
		results[4].status = .No_Convergence;
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:]), e.Status.Ok);
		for &query, index in queries
		{
			shape_result: e.Global_Shape_Distance_Result;
			correction: e.Shape_Correction_Result;
			status: e.Status;
			switch query.kind
			{
			case .Closest_Point:
				shape_result, status = e.shape_closest_point_global_with_context(&scene.query, query.point, shape, query.pose_b, query.settings);
				testing.expect(t, abs(results[index].shape.geometry.distance - 0.502) < 0.000002);
			case .Distance:
				shape_result, status = e.shape_distance_global_with_context(&scene.query, shape, query.pose_a, shape, query.pose_b, query.settings);
				testing.expect(t, abs(results[index].shape.geometry.distance - 0.002) < 0.000002);
				expect_position(t, results[index].shape.geometry.point_a, {query.pose_a.position.x + 0.5, -origin, origin});
				expect_position(t, results[index].shape.geometry.point_b, {query.pose_b.position.x - 0.5, -origin, origin});
			case .Penetration:
				shape_result, status = e.shape_penetration_global_with_context(&scene.query, shape, query.pose_a, shape, query.pose_b, query.settings);
				testing.expect(t, abs(results[index].shape.geometry.depth - 0.002) < 0.000002);
			case .Depenetration:
				correction, status = e.shape_depenetrate_global_with_context(&scene.query, shape, query.pose_a, shape, query.pose_b, query.settings);
				testing.expect(t, results[index].correction.translation.x < -0.00199);
			}
			testing.expect_value(t, results[index].status, status);
			testing.expect_value(t, results[index].shape, shape_result);
			testing.expect_value(t, results[index].correction, correction);
		}
		testing.expect_value(t, results[4].status, e.Status.No_Convergence);
	}
}

@(test)
global_distance_batch_composite_routes_and_late_input_errors_remain_indexed :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	compound: e.Shape_Handle = add_shape(t, &scene, .Big_Compound);
	a: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 0.75;
	queries: [7]e.Global_Distance_Query;
	for &query in queries
	{
		query = {kind=.Distance, shape_a=sphere, shape_b=compound, pose_a=b, pose_b=a, settings={1e-5, 1e-5, 128}};
	}
	queries[1].kind = .Penetration;
	queries[2].kind = .Depenetration;
	queries[3].kind = e.Distance_Query_Kind(255);
	queries[4].pose_a.position.x = math.inf_f64(1);
	queries[5].pose_b.orientation = {};
	results: [8]e.Global_Distance_Query_Result;
	for &result in results
	{
		result.status = .No_Convergence;
	}
	testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:6]), e.Status.Invalid_Argument);
	testing.expect_value(t, e.distance_query_batch_global_with_context(nil, queries[:], results[:]), e.Status.Disposed);
	for result in results
	{
		testing.expect_value(t, result.status, e.Status.No_Convergence);
	}
	testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, nil, nil), e.Status.Ok);
	// no EPA storage has been reserved, but independent distance items work
	testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:]), e.Status.Capacity_Missing);
	testing.expect_value(t, results[0].status, e.Status.Ok);
	testing.expect_value(t, results[1].status, e.Status.Capacity_Missing);
	testing.expect_value(t, results[2].status, e.Status.Capacity_Missing);
	for index in 3 ..< 6
	{
		testing.expect_value(t, results[index].status, e.Status.Invalid_Argument);
	}
	testing.expect_value(t, results[6].status, e.Status.Ok);
	for index in 1 ..< 6
	{
		testing.expect_value(t, results[index].shape, e.Global_Shape_Distance_Result{});
		testing.expect_value(t, results[index].correction, e.Shape_Correction_Result{});
	}
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query), e.Status.Ok);
	status: e.Status = e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:]);
	testing.expect(t, status != .Ok);
	testing.expect_value(t, status, results[1].status);
	testing.expect_value(t, results[2].status, e.Status.Ok);
	testing.expect(t, results[2].correction.translation.x > 0);
	testing.expect_value(t, results[6].status, e.Status.Ok);
	testing.expect_value(t, results[7].status, e.Status.No_Convergence);
}

@(test)
global_distance_batch_callback_failures_reentry_and_recovery :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	state: Distance_Callback_State = {query=&scene.query, other=sphere, mode=.Reenter};
	custom: e.Shape_Handle = distance_add_custom(t, &scene, &state);
	base: [4]e.Global_Distance_Query = batch_distance_inputs(sphere, {1e9 + 0.125, -1e9, 1e9});
	queries: [3]e.Global_Distance_Query = {base[1], base[1], base[1]};
	queries[0].shape_a = custom;
	queries[2].shape_a = custom;
	results: [3]e.Global_Distance_Query_Result;
	for failure in ([4]e.Status{.Not_Found, .Capacity_Missing, .No_Convergence, .Invalid_Description})
	{
		state.calls = 0;
		state.fail_at = 1;
		state.failure = failure;
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:]), failure);
		testing.expect_value(t, results[0].status, failure);
		testing.expect_value(t, results[0].shape, e.Global_Shape_Distance_Result{});
		testing.expect_value(t, results[0].correction, e.Shape_Correction_Result{});
		testing.expect_value(t, results[1].status, e.Status.Ok);
		testing.expect_value(t, results[2].status, e.Status.Ok);
		testing.expect_value(t, state.errors, 0);
		state.fail_at = 0;
		testing.expect_value(t, e.distance_query_batch_global_with_context(&scene.query, queries[:], results[:]), e.Status.Ok);
	}
}
