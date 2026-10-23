package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Callback_Mode :: enum u8
{
	Normal,
	Fail,
}
Custom_State :: struct
{
	query: ^entasis.Query_Context,
	calls, errors: int,
	mode: Callback_Mode,
}
Custom_Sphere :: struct
{
	radius: f32,
	state: ^Custom_State,
}

custom_bounds :: proc "contextless" (shape: rawptr, orientation: util.Quaternion, _registry: ^physics.Shape_Registry) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	return physics.sphere_bounds({(^Custom_Sphere)(shape).radius}, orientation), .Ok;
}
custom_inertia :: proc "contextless" (shape: rawptr, _registry: ^physics.Shape_Registry, mass: f32) -> (physics.Body_Inertia, physics.Physics_Status)
{
	return physics.sphere_inertia({(^Custom_Sphere)(shape).radius}, mass);
}
custom_support :: proc "contextless" (shape: rawptr, direction: util.Vector3, _registry: ^physics.Shape_Registry) -> (util.Vector3, physics.Physics_Status)
{
	return physics.sphere_support({(^Custom_Sphere)(shape).radius}, direction);
}
custom_ray :: proc "contextless" (shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, _registry: ^physics.Shape_Registry) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	value: ^Custom_Sphere = (^Custom_Sphere)(shape);
	state: ^Custom_State = value.state;
	state.calls += 1;
	if pose.position != {} || util.vector3_length(ray.origin) > 2 || ray.maximum_t > 2
	{
		state.errors += 1;
	}
	// the callback is read-only. a nested execution of its own context must
	// be rejected before touching any caller-owned traversal/result storage
	stack: [128]i32;
	_, status := entasis.ray_cast_global_any_with_context(state.query, {start={-1, 0, 0}, end={1, 0, 0}}, stack[:]);
	if status != .Invalid_Argument
	{
		state.errors += 1;
	}
	if state.mode == .Fail
	{
		return {}, .Invalid_Description;
	}
	return physics.sphere_ray_test({value.radius}, pose, ray);
}

@(test)
global_custom_local_frame_reentrancy_and_errors :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Custom_State = {query=&scene.query};
	registration: entasis.Custom_Shape_Registration = entasis.custom_shape_registration(
		Custom_Sphere, .Convex, custom_bounds, custom_inertia, custom_ray, custom_support);
	type_id: entasis.Shape_Type_ID;
	status: entasis.Status;
	type_id, status = entasis.custom_shape_register(&scene.world, registration);
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Custom_Sphere = {radius=0.5, state=&state};
	shape: physics.Typed_Index;
	shape, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	_ = add_static(t, &scene, shape, {1e9 + 0.125, 0, 0});
	segment: physics.World_Precision_Segment = {start={-1e9, 0, 0}, end={2e9, 0, 0}};
	stack: [128]i32;
	hit: entasis.Global_Ray_Hit;
	hit, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, hit.location.x, 1e9 - 0.375);
	testing.expect_value(t, state.calls, 1);
	testing.expect_value(t, state.errors, 0);
	state.mode = .Fail;
	hit, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	testing.expect_value(t, hit.child_index, i32(-1));
	testing.expect_value(t, hit.location, entasis.Global_Position{});
	state.mode = .Normal;
	_, status = entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state.errors, 0);
}

Filter_State :: struct
{
	targets, children: int,
	rejected: i32,
	accepted_child: i32,
}
allow_target :: proc "contextless" (user: rawptr, target: i32) -> physics.Query_Filter_Result
{
	state: ^Filter_State = (^Filter_State)(user);
	state.targets += 1;
	return .Reject if target == state.rejected else .Allow;
}
allow_child :: proc "contextless" (user: rawptr, _target, child: i32) -> physics.Query_Filter_Result
{
	state: ^Filter_State = (^Filter_State)(user);
	state.children += 1;
	return .Allow if child == state.accepted_child else .Reject;
}

@(test)
global_query_filters_preserve_mobility_and_child_identity :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	_ = add_body(t, &scene, shape, {1e9 - 4, 0, 0});
	_ = add_body(t, &scene, shape, {1e9, 0, 0}, .Kinematic);
	_ = add_static(t, &scene, shape, {1e9 + 4, 0, 0});
	segment: physics.World_Precision_Segment = {start={1e9 - 8, 0, 0}, end={1e9 + 8, 0, 0}};
	stack: [128]i32;
	hits: [16]entasis.Global_Ray_Hit;
	for mobility in physics.Body_Mobility
	{
		count, status := entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:], stack[:], {include={mobility}});
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 1);
		testing.expect_value(t, physics.collidable_reference_mobility(hits[0].collidable), mobility);
	}
	compound: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	handle: physics.Static_Handle = add_static(t, &scene, compound, {1e9, 4, 0});
	segment = {start={1e9 - 4, 4, 0}, end={1e9 + 4, 4, 0}};
	low_hits: [16]physics.Global_Ray_Query_Hit;
	state: Filter_State = {rejected=-1, accepted_child=1};
	count, status := physics.simulation_global_ray_query(scene.simulation, segment, .All, low_hits[:], stack[:], &scene.pool,
		{allow_target=allow_target, allow_child=allow_child}, &state);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, low_hits[0].child_index, i32(1));
	testing.expect(t, state.children >= 2);
	ref, _ := physics.collidable_reference_create(.Static, int(handle.value));
	state = {rejected=i32(ref.packed), accepted_child=1};
	count, status = physics.simulation_global_ray_query(scene.simulation, segment, .All, low_hits[:], stack[:], &scene.pool,
		{allow_target=allow_target, allow_child=allow_child}, &state);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 0);
	// rejecting the parent must prevent traversal of all its children
	testing.expect_value(t, state.children, 0);
}

// the explicit streaming byte occupies padding so collectors retain their
// storage size and callback offset
Original_Ray_Collector :: struct
{
	hits: util.Buffer(physics.Ray_Query_Hit),
	count: int,
	mode: physics.Query_Collection_Mode,
	callbacks: physics.Ray_Query_Callbacks,
}
#assert(size_of(Original_Ray_Collector) == size_of(physics.Ray_Query_Collector));
#assert(offset_of(Original_Ray_Collector, callbacks) == offset_of(physics.Ray_Query_Collector, callbacks));

stream_report :: proc "contextless" (user: rawptr, _ray: physics.Tree_Ray, _hit: ^physics.Ray_Query_Hit, maximum_t: ^f32) -> physics.Physics_Status
{
	(^int)(user)^ += 1;
	maximum_t^ = min(maximum_t^, 1);
	return .Ok;
}

@(test)
global_stream_delivery_is_explicit_and_preserves_stored_contract :: proc(t: ^testing.T)
{
	calls: int;
	collector: physics.Ray_Query_Collector = {mode=.All, callbacks={hit=stream_report, user_context=&calls}};
	maximum: f32 = 2;
	testing.expect_value(t, physics.ray_query_collector_report(&collector, {}, {}, &maximum), physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, calls, 0);
	testing.expect_value(t, maximum, f32(2));
	collector.delivery = .Callback;
	testing.expect_value(t, physics.ray_query_collector_report(&collector, {}, {}, &maximum), physics.Physics_Status.Ok);
	testing.expect_value(t, calls, 1);
	testing.expect_value(t, maximum, f32(1));
	storage: physics.Ray_Query_Hit;
	testing.expect_value(t, physics.ray_query_collector_initialize(&collector, {memory=&storage, length=1, id=-1}, .All,
		{hit=stream_report, user_context=&calls}), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.delivery, physics.Ray_Query_Delivery.Stored);
	testing.expect_value(t, physics.ray_query_collector_report(&collector, {}, {t=0.5}, &maximum), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, 1);
	testing.expect_value(t, storage.t, f32(0.5));
	testing.expect_value(t, physics.ray_query_collector_report(&collector, {}, {t=0.25}, &maximum), physics.Physics_Status.Capacity_Missing);
	testing.expect_value(t, calls, 2);
	testing.expect_value(t, storage.t, f32(0.5));
}
