package precision_queries_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

@(test)
global_distance_numeric_admission_errors_and_context_recovery :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	good: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	result: e.Global_Shape_Distance_Result;
	status: e.Status;
	for value in ([5]f64{math.inf_f64(1), -math.inf_f64(1), math.nan_f64(), math.F64_MAX, -math.F64_MAX})
	{
		for axis in 0 ..< 3
		{
			position: [3]f64 = {good.position.x, good.position.y, good.position.z};
			position[axis] = value;
			bad: e.Global_Pose = {position={position[0], position[1], position[2]}, orientation=good.orientation};
			result, status = e.shape_distance_global_with_context(&scene.query, shape, good, shape, bad);
			testing.expect_value(t, status, e.Status.Invalid_Argument);
			testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
			result, status = e.shape_distance_global_with_context(&scene.query, shape, bad, shape, good);
			testing.expect_value(t, status, e.Status.Invalid_Argument);
			testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
			result, status = e.shape_closest_point_global_with_context(&scene.query, bad.position, shape, good);
			testing.expect_value(t, status, e.Status.Invalid_Argument);
			testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
			result, status = e.shape_closest_point_global_with_context(&scene.query, good.position, shape, good);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Intersecting);
		}
	}
	for orientation in ([4]util.Quaternion{{}, {0, 0, 0, 2}, {math.nan_f32(), 0, 0, 1}, {0, math.inf_f32(1), 0, 1}})
	{
		bad: e.Global_Pose = good;
		bad.orientation = orientation;
		result, status = e.shape_distance_global_with_context(&scene.query, shape, good, shape, bad);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		result, status = e.shape_distance_global_with_context(&scene.query, shape, bad, shape, good);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		result, status = e.shape_closest_point_global_with_context(&scene.query, good.position, shape, bad);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
	}
	for settings in ([5]e.Distance_Query_Settings{{}, {math.nan_f32(), 1e-5, 128}, {1e-5, math.inf_f32(1), 128}, {1e-5, 1e-5, 0}, {1e-5, 1e-5, 4097}})
	{
		result, status = e.shape_distance_global_with_context(&scene.query, shape, good, shape, good, settings);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		result, status = e.shape_closest_point_global_with_context(&scene.query, good.position, shape, good, settings);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		result, status = e.shape_penetration_global_with_context(&scene.query, shape, good, shape, good, settings);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
	}
	result, status = e.shape_distance_global_with_context(&scene.query, {}, good, shape, good);
	testing.expect(t, status != .Ok);
	testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
	dead: e.Query_Context;
	result, status = e.shape_distance_global_with_context(&dead, shape, good, shape, good);
	testing.expect_value(t, status, e.Status.Disposed);
	result, status = e.shape_closest_point_global_with_context(&dead, good.position, shape, good);
	testing.expect_value(t, status, e.Status.Disposed);
	result, status = e.shape_penetration_global_with_context(&dead, shape, good, shape, good);
	testing.expect_value(t, status, e.Status.Disposed);
	correction: e.Shape_Correction_Result;
	correction, status = e.shape_depenetrate_global_with_context(&dead, shape, good, shape, good);
	testing.expect_value(t, status, e.Status.Disposed);
	testing.expect_value(t, correction, e.Shape_Correction_Result{});
	_, _, status = e.collidable_shape_global_pose_with_context(&dead, {});
	testing.expect_value(t, status, e.Status.Disposed);
	result, status = e.shape_distance_global_with_context(&scene.query, shape, good, shape, good);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Intersecting);
}

@(test)
global_distance_explicit_epa_capacity_and_local_contracts :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	hull: e.Shape_Handle = add_shape(t, &scene, .Hull);
	box: e.Shape_Handle = add_shape(t, &scene, .Box);
	compound: e.Shape_Handle = add_shape(t, &scene, .Big_Compound);
	a: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 0.75;
	result: e.Global_Shape_Distance_Result;
	status: e.Status;
	result, status = e.shape_penetration_global_with_context(&scene.query, hull, a, hull, b);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
	// reserving the world must not create an implicit shared context owner
	testing.expect_value(t, e.distance_query_reserve(&scene.world), e.Status.Ok);
	result, status = e.shape_penetration_global_with_context(&scene.query, hull, a, hull, b);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query, {4, 4, 3}), e.Status.Ok);
	result, status = e.shape_penetration_global_with_context(&scene.query, hull, a, hull, b);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query, {512, 1024, 1024}), e.Status.Ok);
	result, status = e.shape_penetration_global_with_context(&scene.query, hull, a, hull, b);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Penetrating);
	testing.expect(t, abs(result.geometry.depth - 0.25) < 1e-5);
	// same reserved scratch serves the old float API without changing its frame
	local: e.Shape_Distance_Result;
	local, status = e.shape_penetration_with_context(&scene.query, hull, e.pose(), hull, e.pose({0.75, 0, 0}));
	testing.expect_value(t, status, e.Status.Ok);
	distance_expect_local(t, result, local, a.position);
	// high APIs must not silently promote unsupported composite penetration
	_, expected := e.shape_penetration_with_context(&scene.query, box, e.pose(), compound, e.pose());
	result, status = e.shape_penetration_global_with_context(&scene.query, box, a, compound, a);
	testing.expect_value(t, status, expected);
	testing.expect(t, status != .Ok);
	testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
	correction: e.Shape_Correction_Result;
	_, expected = e.shape_depenetrate_with_context(&scene.query, compound, e.pose(), box, e.pose());
	correction, status = e.shape_depenetrate_global_with_context(&scene.query, compound, a, box, a);
	testing.expect_value(t, status, expected);
	testing.expect(t, status != .Ok);
	testing.expect_value(t, correction, e.Shape_Correction_Result{});
	result, status = e.shape_distance_global_with_context(&scene.query, hull, a, hull, b);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.geometry.state, e.Distance_Query_State.Intersecting);
}
