package precision_queries_tests

import "base:runtime"
import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Distance_Callback_Mode :: enum u8
{
	Normal,
	Reenter,
}
Distance_Callback_State :: struct
{
	query: ^e.Query_Context,
	other: e.Shape_Handle,
	mode: Distance_Callback_Mode,
	calls, errors, fail_at: int,
	failure: e.Status,
}
Distance_Custom_Sphere :: struct
{
	radius: f32,
	state: ^Distance_Callback_State,
}

distance_custom_bounds :: proc "contextless" (shape: rawptr, orientation: util.Quaternion, _shapes: ^p.Shape_Registry) -> (p.Shape_Bounds, p.Physics_Status)
{
	return p.sphere_bounds({(^Distance_Custom_Sphere)(shape).radius}, orientation), .Ok;
}
distance_custom_inertia :: proc "contextless" (shape: rawptr, _shapes: ^p.Shape_Registry, mass: f32) -> (p.Body_Inertia, p.Physics_Status)
{
	return p.sphere_inertia({(^Distance_Custom_Sphere)(shape).radius}, mass);
}
distance_custom_ray :: proc "contextless" (shape: rawptr, pose: p.Rigid_Pose, ray: p.Tree_Ray, _shapes: ^p.Shape_Registry) -> (p.Shape_Ray_Hit, p.Physics_Status)
{
	return p.sphere_ray_test({(^Distance_Custom_Sphere)(shape).radius}, pose, ray);
}
distance_custom_support :: proc "contextless" (shape: rawptr, direction: util.Vector3, _shapes: ^p.Shape_Registry) -> (util.Vector3, p.Physics_Status)
{
	context = runtime.default_context();
	value: ^Distance_Custom_Sphere = (^Distance_Custom_Sphere)(shape);
	state: ^Distance_Callback_State = value.state;
	state.calls += 1;
	if state.mode == .Reenter
	{
		pose: e.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
		_, status := e.shape_distance_global_with_context(state.query, state.other, pose, state.other, pose);
		if status != .Invalid_Argument
		{
			state.errors += 1;
		}
		_, status = e.shape_closest_point_global_with_context(state.query, pose.position, state.other, pose);
		if status != .Invalid_Argument
		{
			state.errors += 1;
		}
		_, _, status = e.collidable_shape_global_pose_with_context(state.query, {});
		if status != .Invalid_Argument
		{
			state.errors += 1;
		}
		if e.distance_query_reserve_with_context(state.query) != .Invalid_Argument || e.query_context_destroy(state.query) != .Invalid_Argument
		{
			state.errors += 1;
		}
	}
	if state.fail_at == state.calls
	{
		return {}, state.failure;
	}
	return p.sphere_support({value.radius}, direction);
}

distance_add_custom :: proc(t: ^testing.T, scene: ^Scene, state: ^Distance_Callback_State) -> e.Shape_Handle
{
	registration: e.Custom_Shape_Registration = e.custom_shape_registration(Distance_Custom_Sphere, .Convex,
		distance_custom_bounds, distance_custom_inertia, distance_custom_ray, distance_custom_support);
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register(&scene.world, registration);
	testing.expect_value(t, status, e.Status.Ok);
	value: Distance_Custom_Sphere = {radius=0.5, state=state};
	shape: e.Shape_Handle;
	shape, status = e.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, e.Status.Ok);
	return shape;
}

@(test)
global_distance_custom_support_reentry_and_numeric_preflight :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	state: Distance_Callback_State = {query=&scene.query, other=sphere, mode=.Reenter};
	shape: e.Shape_Handle = distance_add_custom(t, &scene, &state);
	a: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 1.002;
	result: e.Global_Shape_Distance_Result;
	status: e.Status;
	result, status = e.shape_distance_global_with_context(&scene.query, shape, a, sphere, b);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, state.calls > 0);
	testing.expect_value(t, state.errors, 0);
	testing.expect(t, abs(f64(result.geometry.distance) - (b.position.x - a.position.x - 1)) < 0.000002);
	expect_position(t, result.geometry.point_a, {a.position.x + 0.5, a.position.y, a.position.z});
	expect_position(t, result.geometry.point_b, {b.position.x - 0.5, b.position.y, b.position.z});
	calls: int = state.calls;
	for value in ([3]f64{math.inf_f64(1), math.nan_f64(), math.F64_MAX})
	{
		bad: e.Global_Pose = b;
		bad.position.y = value;
		result, status = e.shape_distance_global_with_context(&scene.query, shape, a, sphere, bad);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		testing.expect_value(t, state.calls, calls);
	}
	result, status = e.shape_distance_global_with_context(&scene.query, shape, a, sphere, b);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.errors, 0);
}

@(test)
global_distance_callback_failures_publish_no_partial_witnesses :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	sphere: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	state: Distance_Callback_State = {query=&scene.query, other=sphere};
	shape: e.Shape_Handle = distance_add_custom(t, &scene, &state);
	testing.expect_value(t, e.distance_query_reserve_with_context(&scene.query), e.Status.Ok);
	a: e.Global_Pose = {position={1e9 + 0.125, -1e9, 1e9}, orientation=util.quaternion_identity()};
	b: e.Global_Pose = a;
	b.position.x += 3;
	for failure in ([4]e.Status{.Not_Found, .Capacity_Missing, .No_Convergence, .Invalid_Description})
	{
		state.calls = 0;
		state.fail_at = 1;
		state.failure = failure;
		result: e.Global_Shape_Distance_Result;
		status: e.Status;
		result, status = e.shape_distance_global_with_context(&scene.query, shape, a, sphere, b);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		state.calls = 0;
		result, status = e.shape_closest_point_global_with_context(&scene.query, b.position, shape, a);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		// coincident convex supports enter the penetration expansion. the third
		// support is a late error, not a miss or an approximate successful depth
		state.calls = 0;
		state.fail_at = 3;
		result, status = e.shape_penetration_global_with_context(&scene.query, shape, a, sphere, a);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, state.calls, 3);
		testing.expect_value(t, result, e.Global_Shape_Distance_Result{});
		state.calls = 0;
		state.fail_at = 1;
		correction: e.Shape_Correction_Result;
		correction, status = e.shape_depenetrate_global_with_context(&scene.query, shape, a, sphere, a);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, correction, e.Shape_Correction_Result{});
		state.fail_at = 0;
		result, status = e.shape_distance_global_with_context(&scene.query, shape, a, sphere, b);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.geometry.distance, f32(2));
	}
}
