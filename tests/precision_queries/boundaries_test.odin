package precision_queries_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

@(test)
global_ray_capacity_numeric_and_empty_world_contracts :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	segment: physics.World_Precision_Segment = {start={1e9 - 2, 0, 0}, end={1e9 + 2, 0, 0}};
	count: int;
	status: entasis.Status;
	count, status = entasis.ray_cast_global_all_with_context(&scene.query, segment, nil, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	_ = add_body(t, &scene, shape, {1e9, 0, 0});
	for index in 0 ..< 9
	{
		_ = add_static(t, &scene, shape, {1e9, f64(index) * 0.01, 0});
	}
	stack: [128]i32;
	required: int;
	required, status = entasis.global_ray_stack_requirement(&scene.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect(t, required > 1);
	sentinel: entasis.Global_Ray_Hit = {t=-123, child_index=-123};
	hits: [16]entasis.Global_Ray_Hit;
	for &hit in hits
	{
		hit = sentinel;
	}
	count, status = entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:], stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	for hit in hits
	{
		testing.expect_value(t, hit, sentinel);
	}
	count, status = entasis.ray_cast_global_all_with_context(&scene.query, segment, hits[:3], stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 3);
	for hit in hits[:3]
	{
		testing.expect(t, hit.t >= 0 && hit.t <= 1);
	}
	testing.expect_value(t, hits[3], sentinel);
	count, status = entasis.ray_cast_global_all_with_context(&scene.query, segment, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	invalid: [5]physics.World_Precision_Segment = {
		{}, {start={math.nan_f64(), 0, 0}, end={1, 0, 0}},
		{start={0, 0, 0}, end={math.inf_f64(1), 0, 0}},
		{start={-math.F64_MAX, 0, 0}, end={math.F64_MAX, 0, 0}},
		{start={1e9, 2, 3}, end={1e9, 2, 3}},
	};
	for ray in invalid
	{
		hits[0] = sentinel;
		count, status = entasis.ray_cast_global_all_with_context(&scene.query, ray, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, hits[0], sentinel);
	}
	// starting beyond the finite f32 range is valid: only target-local geometry
	// enters float kernels. the original f64 segment remains authoritative
	count, status = entasis.ray_cast_global_all_with_context(&scene.query,
		{start={-1e40, 0, 0}, end={1e40, 0, 0}}, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 10);
	for hit in hits[:count]
	{
		testing.expect_value(t, hit.t, f64(0.5));
		testing.expect(t, abs(hit.location.x - (1e9 - 0.5)) <= 0.000002);
	}
	dead: entasis.Query_Context;
	_, status = entasis.ray_cast_global_any_with_context(&dead, segment, stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
}

@(test)
global_pose_export_capacity_invalid_reference_and_frame_contracts :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	body: physics.Body_Handle = add_body(t, &scene, {}, {1e9 + 0.002, -1e9 + 0.125, 1e6 + 0.25});
	reference: physics.Collidable_Reference;
	reference, _ = physics.collidable_reference_create(.Dynamic, int(body.value));
	missing: physics.Collidable_Reference;
	missing, _ = physics.collidable_reference_create(.Dynamic, 99999);
	references: [3]physics.Collidable_Reference = {reference, missing, reference};
	sentinel: entasis.Global_Pose = {position={-123, -456, -789}};
	output: [3]entasis.Global_Pose = {sentinel, sentinel, sentinel};
	count: int;
	status: entasis.Status;
	count, status = entasis.world_export_global_poses(&scene.world, references[:], output[:2]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, output[0], sentinel);
	count, status = entasis.world_export_global_poses(&scene.world, references[:], output[:], {origin={math.nan_f64(), 0, 0}});
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
	testing.expect_value(t, count, 0);
	testing.expect_value(t, output[0], sentinel);
	count, status = entasis.world_export_global_poses(&scene.world, references[:], output[:], {origin={1e12, 0, 0}});
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, output[0].position.x, 1e12 + (1e9 + 0.002));
	testing.expect_value(t, output[1], sentinel);
	testing.expect_value(t, output[2], sentinel);
	_, status = entasis.body_get_global_pose(&scene.world, {-1});
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
}
