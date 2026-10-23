package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Sweep_Callback_State :: struct
{
	query: ^entasis.Query_Context,
	calls, errors: int,
	status: physics.Physics_Status,
	expected_a, expected_b: util.Vector3,
	velocity_a, velocity_b: physics.Body_Velocity,
}
sweep_custom_task :: proc "contextless" (
	user: rawptr, _a, _b: rawptr, _type_a, _type_b: int,
	pose_a, pose_b: physics.Rigid_Pose, va, vb: physics.Body_Velocity,
	_maximum_t, _progression, _convergence: f32, _iterations: int,
	_shapes: ^physics.Shape_Registry, _tasks: ^physics.Collision_Task_Registry,
	_filter: physics.Collision_Child_Filter_Proc, _filter_context: rawptr,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	state: ^Sweep_Callback_State = (^Sweep_Callback_State)(user);
	state.calls += 1;
	if pose_a.position != state.expected_a || pose_b.position != state.expected_b ||
		va != state.velocity_a || vb != state.velocity_b
	{
		state.errors += 1;
	}
	_, status := entasis.volume_global_all_with_context(state.query, {}, nil, nil);
	if status != .Invalid_Argument
	{
		state.errors += 1;
	}
	return {state=.Hit, t0=0.125, t1=0.25, location={0.0625, -0.125, 0.25}, normal={1, 0, 0}, child_a=3, child_b=7}, state.status;
}
sweep_custom_child :: proc "contextless" (
	_user: rawptr, _a, _b: rawptr, _type_a, _type_b: int,
	_pa, _pb, _la, _lb: physics.Rigid_Pose, _va, _vb: physics.Body_Velocity,
	_maximum_t, _progression, _convergence: f32, _iterations: int,
	_shapes: ^physics.Shape_Registry, _tasks: ^physics.Collision_Task_Registry,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	return {}, .Not_Found;
}

@(test)
global_sweep_custom_frame_errors_reentrancy_and_preflight :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bounds_state: Overlap_Callback_State = {query=&scene.query};
	type_id, status := entasis.custom_shape_register(&scene.world, entasis.custom_shape_registration(
		Overlap_Custom_Sphere, .Convex, overlap_custom_bounds, overlap_custom_inertia, overlap_custom_ray, overlap_custom_support));
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Overlap_Custom_Sphere = {radius=0.5, state=&bounds_state};
	shape: physics.Typed_Index;
	shape, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	state: Sweep_Callback_State = {query=&scene.query, expected_b={0.125, 0, 0}, velocity_a={linear={1, 0, 0}}};
	_, status = entasis.sweep_task_register_contextual(&scene.world, {shape_type_a=type_id, shape_type_b=entasis.SHAPE_TYPE_SPHERE,
		user_context=&state, test=sweep_custom_task, child_test=sweep_custom_child});
	testing.expect_value(t, status, entasis.Status.Ok);
	sphere: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_static(t, &scene, sphere, {1e9 + 0.125, -1e9, 1e9});
	pose: entasis.Global_Pose = {position={1e9, -1e9, 1e9}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	stack: [128]i32;
	hits: [4]entasis.Global_Sweep_Hit;
	bounds_state.observation = .Inspect;
	count: int;
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 1, hits[:], nil);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, bounds_state.bounds_calls, 0);
	testing.expect_value(t, state.calls, 0);
	hit: entasis.Global_Sweep_Hit;
	hit, status = entasis.sweep_global_closest_with_context(&scene.query, shape, pose, velocity, 1, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	expect_position(t, hit.sweep.location, {1e9 + 0.0625, -1e9 - 0.125, 1e9 + 0.25}, 0);
	testing.expect_value(t, hit.sweep.t0, f32(0.125));
	testing.expect_value(t, hit.sweep.t1, f32(0.25));
	testing.expect_value(t, hit.sweep.child_a, i32(3));
	testing.expect_value(t, hit.sweep.child_b, i32(7));
	for failure in ([2]physics.Physics_Status{.Invalid_Description, .Capacity_Missing})
	{
		state.status = failure;
		sentinel: entasis.Global_Sweep_Hit = {collidable={packed=0xdeadbeef}};
		hits[0] = sentinel;
		count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 1, hits[:], stack[:]);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, hits[0], sentinel);
		result: entasis.Query_Hit_State;
		result, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 1, stack[:]);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, result, entasis.Query_Hit_State.Miss);
	}
	// existing sweep task Not_Found is a miss, unlike other callback errors
	state.status = .Not_Found;
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 1, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	state.status = .Ok;
	result, any_status := entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 1, stack[:]);
	testing.expect_value(t, any_status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Query_Hit_State.Hit);
	testing.expect_value(t, state.calls, 7);
	testing.expect_value(t, state.errors, 0);
	testing.expect_value(t, bounds_state.errors, 0);
}

@(test)
global_sweep_mobility_parent_and_child_filters :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_body(t, &scene, shape, {1e9, 0, 0});
	_ = add_body(t, &scene, shape, {1e9, 0.125, 0}, .Kinematic);
	_ = add_static(t, &scene, shape, {1e9, 0.25, 0});
	stack: [128]i32;
	hits: [16]entasis.Global_Sweep_Hit;
	pose: entasis.Global_Pose = {position={1e9 - 3, 0, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	for mobility in physics.Body_Mobility
	{
		count, status := entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 4, hits[:], stack[:], {include={mobility}});
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 1);
		testing.expect_value(t, physics.collidable_reference_mobility(hits[0].collidable), mobility);
	}
	compound: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	handle: physics.Static_Handle = add_static(t, &scene, compound, {1e9, 4, 0});
	pose.position = {1e9 - 3, 4, 0};
	low_hits: [16]physics.Global_Sweep_Query_Hit;
	state: Filter_State = {rejected=-1, accepted_child=1};
	pose.position = {1e9 - 3, 0, 0};
	for accepted in ([2]i32{-1, 0})
	{
		state = {rejected=-1, accepted_child=accepted};
		primitive_count, primitive_status := physics.simulation_global_sweep_query(scene.simulation, shape, pose, velocity, 4, 1e-5, 1e-5, 25,
			.All, low_hits[:], stack[:], &scene.pool, {allow_target=allow_target, allow_child=allow_child}, &state);
		testing.expect_value(t, primitive_status, physics.Physics_Status.Ok);
		testing.expect_value(t, primitive_count, 3 if accepted == 0 else 0);
		testing.expect_value(t, state.targets, 3);
		testing.expect_value(t, state.children, 3);
	}
	pose.position = {1e9 - 3, 4, 0};
	state = {rejected=-1, accepted_child=1};
	testing.expect_value(t, physics.world_precision_sweep_admit(scene.simulation, pose, velocity, 4, stack[:]), physics.Physics_Status.Ok);
	count, status := physics.simulation_global_sweep_query(scene.simulation, shape, pose, velocity, 4, 1e-5, 1e-5, 25,
		.All, low_hits[:], stack[:], &scene.pool, {allow_target=allow_target, allow_child=allow_child}, &state);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, low_hits[0].sweep.child_b, i32(1));
	testing.expect(t, abs(low_hits[0].sweep.t1 - 2.5) < 1e-4);
	testing.expect(t, state.children >= 2);
	reference, _ := physics.collidable_reference_create(.Static, int(handle.value));
	testing.expect_value(t, low_hits[0].target_id, i32(reference.packed));
	state = {rejected=i32(reference.packed), accepted_child=1};
	count, status = physics.simulation_global_sweep_query(scene.simulation, shape, pose, velocity, 4, 1e-5, 1e-5, 25,
		.All, low_hits[:], stack[:], &scene.pool, {allow_target=allow_target, allow_child=allow_child}, &state);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, state.children, 0);
	state = {rejected=-1, accepted_child=-1};
	count, status = physics.simulation_global_sweep_query(scene.simulation, shape, pose, velocity, 4, 1e-5, 1e-5, 25,
		.All, low_hits[:], stack[:], &scene.pool, {allow_target=allow_target, allow_child=allow_child}, &state);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, count, 0);
	testing.expect(t, state.children >= 2);
}

@(test)
global_sweep_callback_failure_preserves_prefix_and_reverse_route :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bounds_state: Overlap_Callback_State = {query=&scene.query};
	type_id, status := entasis.custom_shape_register(&scene.world, entasis.custom_shape_registration(
		Overlap_Custom_Sphere, .Convex, overlap_custom_bounds, overlap_custom_inertia, overlap_custom_ray, overlap_custom_support));
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Overlap_Custom_Sphere = {radius=0.5, state=&bounds_state};
	custom: physics.Typed_Index;
	custom, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	state: Sweep_Callback_State = {query=&scene.query, status=.Invalid_Description,
		expected_a={0.125, 0, 0}, velocity_b={linear={1, 0, 0}}};
	_, status = entasis.sweep_task_register_contextual(&scene.world, {shape_type_a=type_id, shape_type_b=entasis.SHAPE_TYPE_SPHERE,
		user_context=&state, test=sweep_custom_task, child_test=sweep_custom_child});
	testing.expect_value(t, status, entasis.Status.Ok);
	sphere: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	body: physics.Body_Handle = add_body(t, &scene, sphere, {1e9, 0, 0});
	_ = add_static(t, &scene, custom, {1e9 + 0.125, 0, 0});
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	stack: [128]i32;
	sentinel: entasis.Global_Sweep_Hit = {collidable={packed=0xdeadbeef}};
	hits: [4]entasis.Global_Sweep_Hit = {sentinel, sentinel, sentinel, sentinel};
	count: int;
	count, status = entasis.sweep_global_all_with_context(&scene.query, sphere, pose, velocity, 1, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	testing.expect_value(t, count, 1);
	reference, _ := physics.collidable_reference_create(.Dynamic, int(body.value));
	testing.expect_value(t, hits[0].collidable, reference);
	for hit in hits[1:]
	{
		testing.expect_value(t, hit, sentinel);
	}
	testing.expect_value(t, state.calls, 1);
	result: entasis.Query_Hit_State;
	result, status = entasis.sweep_global_any_with_context(&scene.query, sphere, pose, velocity, 1, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Query_Hit_State.Hit);
	testing.expect_value(t, state.calls, 1);
	result, status = entasis.sweep_global_any_with_context(&scene.query, sphere, pose, velocity, 1, stack[:], {include={.Static}});
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	testing.expect_value(t, result, entasis.Query_Hit_State.Miss);
	state.status = .Ok;
	hit: entasis.Global_Sweep_Hit;
	hit, status = entasis.sweep_global_closest_with_context(&scene.query, sphere, pose, velocity, 1, stack[:], {include={.Static}});
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, hit.sweep.child_a, i32(7));
	testing.expect_value(t, hit.sweep.child_b, i32(3));
	testing.expect_value(t, hit.sweep.normal, util.Vector3{-1, 0, 0});
	expect_position(t, hit.sweep.location, {1e9 + 0.0625, -0.125, 0.25}, 0);
	testing.expect_value(t, state.errors, 0);
}
