package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Overlap_Observation :: enum u8
{
	Idle,
	Inspect,
}
Overlap_Callback_State :: struct
{
	query: ^entasis.Query_Context,
	observation: Overlap_Observation,
	result: physics.Physics_Status,
	bounds_calls, pair_calls, errors: int,
	depth: f32,
	expected_a, expected_b: util.Vector3,
}
Overlap_Custom_Sphere :: struct
{
	radius: f32,
	state: ^Overlap_Callback_State,
}

overlap_custom_bounds :: proc "contextless" (shape: rawptr, orientation: util.Quaternion, _registry: ^physics.Shape_Registry) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	value: ^Overlap_Custom_Sphere = (^Overlap_Custom_Sphere)(shape);
	if value.state.observation == .Inspect
	{
		value.state.bounds_calls += 1;
		_, status := entasis.volume_global_all_with_context(value.state.query, {}, nil, nil);
		if status != .Invalid_Argument
		{
			value.state.errors += 1;
		}
	}
	return physics.sphere_bounds({value.radius}, orientation), .Ok;
}
overlap_custom_inertia :: proc "contextless" (shape: rawptr, _registry: ^physics.Shape_Registry, mass: f32) -> (physics.Body_Inertia, physics.Physics_Status)
{
	return physics.sphere_inertia({(^Overlap_Custom_Sphere)(shape).radius}, mass);
}
overlap_custom_support :: proc "contextless" (shape: rawptr, direction: util.Vector3, _registry: ^physics.Shape_Registry) -> (util.Vector3, physics.Physics_Status)
{
	return physics.sphere_support({(^Overlap_Custom_Sphere)(shape).radius}, direction);
}
overlap_custom_ray :: proc "contextless" (shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, _registry: ^physics.Shape_Registry) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	return physics.sphere_ray_test({(^Overlap_Custom_Sphere)(shape).radius}, pose, ray);
}
overlap_custom_pair :: proc "contextless" (shape_a, _shape_b: rawptr, pose_a, pose_b: physics.Rigid_Pose, _margin: f32,
	_shapes: ^physics.Shape_Registry) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	state: ^Overlap_Callback_State = (^Overlap_Custom_Sphere)(shape_a).state;
	state.pair_calls += 1;
	if state.observation == .Inspect
	{
		if pose_a.position != state.expected_a || pose_b.position != state.expected_b
		{
			state.errors += 1;
		}
		_, status := entasis.volume_global_all_with_context(state.query, {}, nil, nil);
		if status != .Invalid_Argument
		{
			state.errors += 1;
		}
	}
	// a custom route can distinguish a speculative negative-depth contact
	// from an actual overlap. all preserves it, while any must not report a hit
	return {count=1, offset_b=pose_b.position, normal={1, 0, 0}, contacts={
		0={offset={0.5, 0, 0}, depth=state.depth, feature_id=37},
	}}, state.result;
}
overlap_custom_pair_wide :: proc "contextless" (bundle: ^physics.Collision_Convex_Wide_Bundle, _registry: ^physics.Shape_Registry) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	a, b: physics.Sphere_Wide;
	for lane in 0 ..< bundle.count
	{
		physics.sphere_wide_write_slot_trusted(&a, lane, {(^Overlap_Custom_Sphere)(bundle.shape_a[lane]).radius});
		physics.sphere_wide_write_slot_trusted(&b, lane, (^physics.Sphere)(bundle.shape_b[lane])^);
	}
	wide, status := physics.sphere_pair_test_wide(a, b, bundle.speculative_margin, bundle.offset_b, bundle.count);
	return {kind=.One_Contact, one=wide}, status;
}

@(test)
global_overlap_custom_frame_callback_errors_reentrancy_and_preflight :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Overlap_Callback_State = {query=&scene.query, depth=-0.002, expected_b={0.125, 0, 0}};
	registration: entasis.Custom_Shape_Registration = entasis.custom_shape_registration(
		Overlap_Custom_Sphere, .Convex, overlap_custom_bounds, overlap_custom_inertia, overlap_custom_ray, overlap_custom_support);
	type_id, status := entasis.custom_shape_register(&scene.world, registration);
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Overlap_Custom_Sphere = {radius=0.5, state=&state};
	query_shape: physics.Typed_Index;
	query_shape, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	_, status = entasis.collision_task_register(&scene.world,
		entasis.collision_task_convex(type_id, entasis.SHAPE_TYPE_SPHERE, 16, .Sphere, overlap_custom_pair, overlap_custom_pair_wide));
	testing.expect_value(t, status, entasis.Status.Ok);
	sphere: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_static(t, &scene, sphere, {1e9 + 0.125, -1e9, 1e9});
	pose: entasis.Global_Pose = {position={1e9, -1e9, 1e9}, orientation=util.quaternion_identity()};
	stack: [128]i32;
	hits: [4]entasis.Overlap_Hit;
	state.observation = .Inspect;
	count: int;
	count, status = entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, hits[:], nil);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, state.bounds_calls, 0);
	testing.expect_value(t, state.pair_calls, 0);
	count, status = entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, hits[0].manifold.convex.contacts[0].depth, f32(-0.002));
	result: entasis.Overlap_State;
	result, status = entasis.overlap_global_any_with_context(&scene.query, query_shape, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Overlap_State.Separated);
	for failure in ([3]physics.Physics_Status{.Invalid_Description, .Not_Found, .Capacity_Missing})
	{
		state.result = failure;
		state.depth = 0.002;
		sentinel: entasis.Overlap_Hit = {collidable={packed=0xdeadbeef}};
		hits[0] = sentinel;
		count, status = entasis.overlap_global_all_with_context(&scene.query, query_shape, pose, hits[:], stack[:]);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, hits[0], sentinel);
		result, status = entasis.overlap_global_any_with_context(&scene.query, query_shape, pose, stack[:]);
		testing.expect_value(t, status, failure);
		testing.expect_value(t, result, entasis.Overlap_State.Separated);
	}
	state.result = .Ok;
	result, status = entasis.overlap_global_any_with_context(&scene.query, query_shape, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Overlap_State.Intersecting);
	testing.expect_value(t, state.errors, 0);
	testing.expect_value(t, state.pair_calls, 9);
	testing.expect_value(t, state.bounds_calls, 9);
}

@(test)
global_overlap_volume_filters_mobility_parent_and_child_identity :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_body(t, &scene, shape, {1e9, 0, 0});
	_ = add_body(t, &scene, shape, {1e9, 0.125, 0}, .Kinematic);
	_ = add_static(t, &scene, shape, {1e9, 0.25, 0});
	stack: [128]i32;
	hits: [16]entasis.Overlap_Hit;
	volumes: [16]entasis.Volume_Hit;
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	for mobility in physics.Body_Mobility
	{
		count, status := entasis.overlap_global_all_with_context(&scene.query, shape, pose, hits[:], stack[:], {include={mobility}});
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 1);
		testing.expect_value(t, physics.collidable_reference_mobility(hits[0].collidable), mobility);
		count, status = entasis.volume_global_all_with_context(&scene.query,
			{min={1e9 - 2, -2, -2}, max={1e9 + 2, 2, 2}}, volumes[:], stack[:], {include={mobility}});
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 1);
		testing.expect_value(t, physics.collidable_reference_mobility(volumes[0].collidable), mobility);
	}
	compound: physics.Typed_Index = add_shape(t, &scene, .Big_Compound);
	handle: physics.Static_Handle = add_static(t, &scene, compound, {1e9, 4, 0});
	pose.position = {1e9, 4, 0};
	batcher: physics.Query_Overlap_Batcher;
	testing.expect_value(t, physics.query_overlap_batcher_bind(&batcher, &scene.simulation.narrow_phase), physics.Physics_Status.Ok);
	low_hits: [16]physics.Overlap_Query_Hit;
	filter_state: Filter_State = {rejected=-1, accepted_child=1};
	collector: physics.Overlap_Query_Collector = {
		hits={memory=&low_hits[0], length=i32(len(low_hits)), id=-1},
		filter={allow_target=allow_target, allow_child=allow_child}, user_context=&filter_state,
	};
	pose.position = {1e9, 0, 0};
	for accepted in ([2]i32{-1, 0})
	{
		filter_state = {rejected=-1, accepted_child=accepted};
		collector.count = 0;
		testing.expect_value(t, physics.simulation_global_overlap_query(scene.simulation, shape, pose, &collector, stack[:], &batcher, &scene.pool), physics.Physics_Status.Ok);
		testing.expect_value(t, collector.count, 3 if accepted == 0 else 0);
		testing.expect_value(t, filter_state.targets, 3);
		testing.expect_value(t, filter_state.children, 3);
		filter_state = {rejected=-1, accepted_child=accepted};
		primitive_result, primitive_status := physics.simulation_global_overlap_any_query(scene.simulation, shape, pose,
			collector.filter, &filter_state, stack[:], &batcher, &scene.pool);
		testing.expect_value(t, primitive_status, physics.Physics_Status.Ok);
		testing.expect_value(t, primitive_result, physics.Overlap_Query_State.Intersecting if accepted == 0 else physics.Overlap_Query_State.Separated);
		testing.expect_value(t, filter_state.targets, 1 if accepted == 0 else 3);
		testing.expect_value(t, filter_state.children, 1 if accepted == 0 else 3);
	}
	pose.position = {1e9, 4, 0};
	filter_state = {rejected=-1, accepted_child=1};
	collector.count = 0;
	testing.expect_value(t, physics.simulation_global_overlap_query(scene.simulation, shape, pose, &collector, stack[:], &batcher, &scene.pool), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, 1);
	testing.expect(t, filter_state.children >= 2);
	// only the +X child can contribute when child index 1 is selected
	testing.expect_value(t, low_hits[0].manifold.kind, physics.Manifold_Kind.Nonconvex);
	testing.expect_value(t, low_hits[0].manifold.nonconvex.count, i32(1));
	testing.expect(t, low_hits[0].manifold.nonconvex.contacts[0].offset.x > 0);
	ref, _ := physics.collidable_reference_create(.Static, int(handle.value));
	testing.expect_value(t, low_hits[0].target_id, i32(ref.packed));
	filter_state = {rejected=i32(ref.packed), accepted_child=1};
	collector.count = 0;
	testing.expect_value(t, physics.simulation_global_overlap_query(scene.simulation, shape, pose, &collector, stack[:], &batcher, &scene.pool), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, 0);
	testing.expect_value(t, filter_state.children, 0);
	filter_state = {rejected=-1, accepted_child=-1};
	result, status := physics.simulation_global_overlap_any_query(scene.simulation, shape, pose,
		collector.filter, &filter_state, stack[:], &batcher, &scene.pool);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, result, physics.Overlap_Query_State.Separated);
	testing.expect(t, filter_state.children >= 2);
	filter_state = {rejected=-1, accepted_child=1};
	result, status = physics.simulation_global_overlap_any_query(scene.simulation, shape, pose,
		collector.filter, &filter_state, stack[:], &batcher, &scene.pool);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, result, physics.Overlap_Query_State.Intersecting);
	// the world-owned batcher is borrowed only by this explicit low-level test.
	// native high entry points use their own Query_Context batchers instead
	testing.expect_value(t, physics.query_overlap_batcher_dispose(&batcher), physics.Physics_Status.Ok);
}

@(test)
global_overlap_failure_preserves_active_prefix_and_any_early_exit :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Overlap_Callback_State = {query=&scene.query, result=.Not_Found, depth=0.002, expected_a={0.125, 0, 0}};
	type_id, status := entasis.custom_shape_register(&scene.world, entasis.custom_shape_registration(
		Overlap_Custom_Sphere, .Convex, overlap_custom_bounds, overlap_custom_inertia, overlap_custom_ray, overlap_custom_support));
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Overlap_Custom_Sphere = {radius=0.5, state=&state};
	custom: physics.Typed_Index;
	custom, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	_, status = entasis.collision_task_register(&scene.world, entasis.collision_task_convex(
		type_id, entasis.SHAPE_TYPE_SPHERE, 16, .Sphere, overlap_custom_pair, overlap_custom_pair_wide));
	testing.expect_value(t, status, entasis.Status.Ok);
	sphere: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	body: physics.Body_Handle = add_body(t, &scene, sphere, {1e9, 0, 0});
	_ = add_static(t, &scene, custom, {1e9 + 0.125, 0, 0});
	state.observation = .Inspect;
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	stack: [128]i32;
	sentinel: entasis.Overlap_Hit = {collidable={packed=0xdeadbeef}};
	hits: [4]entasis.Overlap_Hit = {sentinel, sentinel, sentinel, sentinel};
	count: int;
	count, status = entasis.overlap_global_all_with_context(&scene.query, sphere, pose, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, count, 1);
	reference, _ := physics.collidable_reference_create(.Dynamic, int(body.value));
	testing.expect_value(t, hits[0].collidable, reference);
	for hit in hits[1:]
	{
		testing.expect_value(t, hit, sentinel);
	}
	testing.expect_value(t, state.pair_calls, 1);
	result: entasis.Overlap_State;
	result, status = entasis.overlap_global_any_with_context(&scene.query, sphere, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Overlap_State.Intersecting);
	testing.expect_value(t, state.pair_calls, 1);
	result, status = entasis.overlap_global_any_with_context(&scene.query, sphere, pose, stack[:], {include={.Static}});
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, result, entasis.Overlap_State.Separated);
	state.result = .Ok;
	count, status = entasis.overlap_global_all_with_context(&scene.query, sphere, pose, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 2);
	testing.expect_value(t, state.errors, 0);
}
