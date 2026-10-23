package precision_queries_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Batch_Callback_State :: struct
{
	query: ^e.Query_Context,
	events: [16]int,
	count, errors: int,
}
Batch_Custom_Sphere :: struct
{
	radius: f32,
	state: ^Batch_Callback_State,
}

batch_custom_bounds :: proc "contextless" (shape: rawptr, orientation: util.Quaternion, _shapes: ^p.Shape_Registry) -> (p.Shape_Bounds, p.Physics_Status)
{
	return p.sphere_bounds({(^Batch_Custom_Sphere)(shape).radius}, orientation), .Ok;
}
batch_custom_inertia :: proc "contextless" (shape: rawptr, _shapes: ^p.Shape_Registry, mass: f32) -> (p.Body_Inertia, p.Physics_Status)
{
	return p.sphere_inertia({(^Batch_Custom_Sphere)(shape).radius}, mass);
}
batch_custom_support :: proc "contextless" (shape: rawptr, direction: util.Vector3, _shapes: ^p.Shape_Registry) -> (util.Vector3, p.Physics_Status)
{
	return p.sphere_support({(^Batch_Custom_Sphere)(shape).radius}, direction);
}
batch_custom_ray :: proc "contextless" (shape: rawptr, pose: p.Rigid_Pose, ray: p.Tree_Ray, _shapes: ^p.Shape_Registry) -> (p.Shape_Ray_Hit, p.Physics_Status)
{
	value: ^Batch_Custom_Sphere = (^Batch_Custom_Sphere)(shape);
	state: ^Batch_Callback_State = value.state;
	label: int = int(ray.origin.y * 32 + 0.5);
	if state.count < len(state.events)
	{
		state.events[state.count] = label;
	}
	state.count += 1;
	queries: [1]e.Global_Query;
	results: [1]e.Global_Query_Result = {{count=77}};
	if e.query_batch_global_with_context(state.query, queries[:], results[:]) != .Invalid_Argument || results[0].count != 77
	{
		state.errors += 1;
	}
	distance_queries: [1]e.Global_Distance_Query;
	distance_results: [1]e.Global_Distance_Query_Result = {{status=.No_Convergence}};
	if e.distance_query_batch_global_with_context(state.query, distance_queries[:], distance_results[:]) != .Invalid_Argument ||
		distance_results[0].status != .No_Convergence
	{
		state.errors += 1;
	}
	if label == 3
	{
		return {}, .Invalid_Description;
	}
	return p.sphere_ray_test({value.radius}, pose, ray);
}

@(test)
global_spatial_batch_callback_order_reentry_and_error_continuation :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Batch_Callback_State = {query=&scene.query};
	registration: e.Custom_Shape_Registration = e.custom_shape_registration(Batch_Custom_Sphere, .Convex,
		batch_custom_bounds, batch_custom_inertia, batch_custom_ray, batch_custom_support);
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register(&scene.world, registration);
	testing.expect_value(t, status, e.Status.Ok);
	value: Batch_Custom_Sphere = {radius=0.5, state=&state};
	shape: e.Shape_Handle;
	shape, status = e.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, e.Status.Ok);
	_ = add_static(t, &scene, shape, {1e9 + 0.125, 0, 0});
	queries: [8]e.Global_Query;
	for &query, index in queries
	{
		query = {kind=.Ray_Any if index % 2 == 0 else .Ray_Closest,
			ray_data={segment={start={1e9 - 2, f64(index) / 32, 0}, end={1e9 + 2, f64(index) / 32, 0}}}};
	}
	queries[1].kind = .Ray_All;
	queries[1].ray_data.output = {-1, 1};
	results: [8]e.Global_Query_Result;
	stack: [128]i32;
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], {traversal_stack=stack[:]}), e.Status.Invalid_Argument);
	testing.expect_value(t, state.count, 7);
	for label, index in ([7]int{0, 2, 3, 4, 5, 6, 7})
	{
		testing.expect_value(t, state.events[index], label);
	}
	for result, index in results
	{
		if index == 1 || index == 3
		{
			testing.expect_value(t, result.status, e.Status.Invalid_Argument if index == 1 else e.Status.Invalid_Description);
			testing.expect_value(t, result.count, i32(0));
			testing.expect_value(t, result.hit, e.Query_Hit_State.Miss);
		}
		else
		{
			testing.expect_value(t, result.status, e.Status.Ok);
			testing.expect_value(t, result.hit, e.Query_Hit_State.Hit);
		}
	}
	testing.expect_value(t, state.errors, 0);
	// single-query admission also prevents callbacks from entering either batch
	_, status = e.ray_cast_global_closest_with_context(&scene.query, queries[0].ray_data.segment, stack[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, state.errors, 0);
}

@(test)
global_spatial_batch_preserves_each_mobility_filter :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	_ = add_body(t, &scene, shape, origin);
	_ = add_body(t, &scene, shape, origin, .Kinematic);
	_ = add_static(t, &scene, shape, origin);
	stack: [128]i32;
	rays: [16]e.Global_Ray_Hit;
	sweeps: [16]e.Global_Sweep_Hit;
	overlaps: [16]e.Overlap_Hit;
	volumes: [16]e.Volume_Hit;
	scratch: e.Global_Query_Scratch = {rays[:], sweeps[:], overlaps[:], volumes[:], stack[:]};
	for mobility in p.Body_Mobility
	{
		queries: [9]e.Global_Query = batch_spatial_inputs(shape, origin);
		for &query in queries
		{
			switch query.kind
			{
			case .Ray_Any, .Ray_Closest, .Ray_All:
				query.ray_data.filter.include = {mobility};
			case .Sweep_Any, .Sweep_Closest, .Sweep_All:
				query.sweep_data.filter.include = {mobility};
			case .Overlap_Any, .Overlap_All:
				query.overlap_data.filter.include = {mobility};
			case .Volume_All:
				query.volume_data.filter.include = {mobility};
			}
		}
		results: [9]e.Global_Query_Result;
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch), e.Status.Ok);
		for result in results
		{
			testing.expect_value(t, result.status, e.Status.Ok);
			testing.expect_value(t, result.count, i32(1));
			testing.expect_value(t, result.hit, e.Query_Hit_State.Hit);
		}
		testing.expect_value(t, p.collidable_reference_mobility(rays[1].collidable), mobility);
		testing.expect_value(t, p.collidable_reference_mobility(sweeps[1].collidable), mobility);
		testing.expect_value(t, p.collidable_reference_mobility(overlaps[1].collidable), mobility);
		testing.expect_value(t, p.collidable_reference_mobility(volumes[1].collidable), mobility);
		testing.expect_value(t, p.collidable_reference_mobility(results[1].ray_hit.collidable), mobility);
		testing.expect_value(t, p.collidable_reference_mobility(results[4].sweep_hit.collidable), mobility);
	}
}
