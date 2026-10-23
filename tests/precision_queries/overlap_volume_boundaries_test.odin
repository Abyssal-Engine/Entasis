package precision_queries_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_overlap_volume_empty_capacity_and_prefix_contracts :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	bounds: entasis.Global_Bounds = {min={1e9 - 1, -1, -1}, max={1e9 + 1, 1, 1}};
	required, status := entasis.global_bounds_stack_requirement(&scene.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, required, 0);
	count: int;
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, nil, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	state: entasis.Overlap_State;
	state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Overlap_State.Separated);
	count, status = entasis.volume_global_all_with_context(&scene.query, bounds, nil, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	_ = add_body(t, &scene, shape, pose.position);
	for index in 0 ..< 9
	{
		_ = add_static(t, &scene, shape, {1e9, f64(index) * 0.01, 0});
	}
	required, status = entasis.global_bounds_stack_requirement(&scene.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect(t, required > 1);
	stack: [128]i32;
	overlap_sentinel: entasis.Overlap_Hit = {collidable={packed=0xdeadbeef}, manifold={convex={count=-123}}};
	volume_sentinel: entasis.Volume_Hit = {collidable={packed=0xdeadbeef}};
	overlaps: [16]entasis.Overlap_Hit;
	volumes: [16]entasis.Volume_Hit;
	for &hit in overlaps
	{
		hit = overlap_sentinel;
	}
	for &hit in volumes
	{
		hit = volume_sentinel;
	}
	// the static tree needs more scratch than the active tree. no active-tree
	// prefix may escape when the complete operation has insufficient scratch
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, overlaps[:], stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, state, entasis.Overlap_State.Separated);
	count, status = entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:], stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	for index in 0 ..< len(overlaps)
	{
		testing.expect_value(t, overlaps[index], overlap_sentinel);
		testing.expect_value(t, volumes[index], volume_sentinel);
	}
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, overlaps[:3], stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 3);
	for hit in overlaps[:3]
	{
		testing.expect(t, hit.manifold.convex.count > 0);
	}
	count, status = entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:3], stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 3);
	for index in 3 ..< len(overlaps)
	{
		testing.expect_value(t, overlaps[index], overlap_sentinel);
		testing.expect_value(t, volumes[index], volume_sentinel);
	}
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	count, status = entasis.volume_global_all_with_context(&scene.query, bounds, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, overlaps[:], stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 10);
	count, status = entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:], stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 10);
	state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Overlap_State.Intersecting);
}

@(test)
global_overlap_volume_numeric_admission_and_context_reuse :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	_ = add_body(t, &scene, shape, {1e9, 0, 0});
	stack: [128]i32;
	sentinel: entasis.Overlap_Hit = {collidable={packed=0xdeadbeef}};
	hits: [4]entasis.Overlap_Hit;
	poses: [5]entasis.Global_Pose = {
		{position={math.nan_f64(), 0, 0}, orientation=util.quaternion_identity()},
		{position={0, math.inf_f64(1), 0}, orientation=util.quaternion_identity()},
		{position={1e9, 0, 0}},
		{position={1e9, 0, 0}, orientation={w=2}},
		{position={1e9, 0, 0}, orientation={x=math.nan_f32(), w=1}},
	};
	for pose in poses
	{
		hits[0] = sentinel;
		count, status := entasis.overlap_global_all_with_context(&scene.query, shape, pose, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, hits[0], sentinel);
		state: entasis.Overlap_State;
		state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, state, entasis.Overlap_State.Separated);
	}
	volumes: [4]entasis.Volume_Hit;
	volume_sentinel: entasis.Volume_Hit = {collidable={packed=0xdeadbeef}};
	invalid: [6]entasis.Global_Bounds = {
		{min={1, 0, 0}, max={0, 1, 1}}, {min={0, 1, 0}, max={1, 0, 1}},
		{min={0, 0, 1}, max={1, 1, 0}}, {min={math.nan_f64(), 0, 0}, max={1, 1, 1}},
		{min={0, 0, 0}, max={math.inf_f64(1), 1, 1}},
		{min={-math.inf_f64(1), 0, 0}, max={1, 1, 1}},
	};
	for bounds in invalid
	{
		volumes[0] = volume_sentinel;
		count, status := entasis.volume_global_all_with_context(&scene.query, bounds, volumes[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, volumes[0], volume_sentinel);
	}
	// f64 bounds/poses beyond f32 remain valid when no relative geometry is
	// needed. a wide volume encloses the entire float tree without narrowing
	count, status := entasis.volume_global_all_with_context(&scene.query,
		{min={-1e100, -1e100, -1e100}, max={1e100, 1e100, 1e100}}, volumes[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 1);
	count, status = entasis.volume_global_all_with_context(&scene.query,
		{min={1e100, 0, 0}, max={1e100, 0, 0}}, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	pose: entasis.Global_Pose = {position={1e100, 0, 0}, orientation=util.quaternion_identity()};
	count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	state: entasis.Overlap_State;
	state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Overlap_State.Separated);
	pose.position = {1e9, 0, 0};
	_, status = entasis.overlap_global_any_with_context(&scene.query, {}, pose, stack[:]);
	testing.expect(t, status != .Ok);
	state, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Overlap_State.Intersecting);
	dead: entasis.Query_Context;
	_, status = entasis.overlap_global_any_with_context(&dead, shape, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
	_, status = entasis.overlap_global_all_with_context(&dead, shape, pose, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
	_, status = entasis.volume_global_all_with_context(&dead, {}, volumes[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
}
