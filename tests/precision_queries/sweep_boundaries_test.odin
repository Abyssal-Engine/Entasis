package precision_queries_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_sweep_empty_preflight_capacity_and_prefix :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	required, status := entasis.global_bounds_stack_requirement(&scene.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, required, 0);
	count: int;
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, nil, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
	state: entasis.Query_Hit_State;
	state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 32, nil);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Query_Hit_State.Miss);
	_, status = entasis.sweep_global_closest_with_context(&scene.query, shape, pose, velocity, 32, nil);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	_ = add_body(t, &scene, shape, {1e9 + 24, 0, 0});
	closest_static: physics.Static_Handle;
	for index in 0 ..< 9
	{
		handle: physics.Static_Handle = add_static(t, &scene, shape, {1e9 + 2 + f64(index) * 2, 0, 0});
		if index == 0
		{
			closest_static = handle;
		}
	}
	required, status = entasis.global_bounds_stack_requirement(&scene.world);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect(t, required > 1);
	stack: [128]i32;
	sentinel: entasis.Global_Sweep_Hit = {collidable={packed=0xdeadbeef}, sweep={t1=-123, child_a=-37}};
	hits: [16]entasis.Global_Sweep_Hit;
	for &hit in hits
	{
		hit = sentinel;
	}
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, hits[:], stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 32, stack[:required - 1]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, state, entasis.Query_Hit_State.Miss);
	for hit in hits
	{
		testing.expect_value(t, hit, sentinel);
	}
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, hits[:3], stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 3);
	for hit in hits[3:]
	{
		testing.expect_value(t, hit, sentinel);
	}
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, count, 0);
	// all must not inherit the earliest time from the active tree or a prior
	// static leaf. every one of ten distinct impacts remains observable
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, hits[:], stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 10);
	for hit, index in hits[:count]
	{
		testing.expect_value(t, hit.sweep.state, physics.Sweep_Hit_State.Hit);
		for other in hits[:index]
		{
			testing.expect(t, other.collidable != hit.collidable);
		}
	}
	closest: entasis.Global_Sweep_Hit;
	closest, status = entasis.sweep_global_closest_with_context(&scene.query, shape, pose, velocity, 32, stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	reference: physics.Collidable_Reference;
	reference, _ = physics.collidable_reference_create(.Static, int(closest_static.value));
	testing.expect_value(t, closest.collidable, reference);
	testing.expect(t, abs(closest.sweep.t1 - 1) <= 0.00002);
	state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 32, stack[:required]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Query_Hit_State.Hit);
	pose.position.y = 4;
	count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 32, nil, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, count, 0);
}

@(test)
global_sweep_numeric_admission_and_recovery :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_static(t, &scene, shape, {1e9 + 3, 0, 0});
	stack: [128]i32;
	sentinel: entasis.Global_Sweep_Hit = {collidable={packed=0xdeadbeef}};
	hits: [2]entasis.Global_Sweep_Hit;
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	velocity: entasis.Body_Velocity = {linear={1, 0, 0}};
	invalid_poses: [5]entasis.Global_Pose = {
		{position={math.nan_f64(), 0, 0}, orientation=util.quaternion_identity()},
		{position={0, math.inf_f64(1), 0}, orientation=util.quaternion_identity()},
		{position={1e9, 0, 0}},
		{position={1e9, 0, 0}, orientation={w=2}},
		{position={1e9, 0, 0}, orientation={x=math.nan_f32(), w=1}},
	};
	for invalid in invalid_poses
	{
		hits[0] = sentinel;
		count, status := entasis.sweep_global_all_with_context(&scene.query, shape, invalid, velocity, 4, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, hits[0], sentinel);
	}
	for time in ([3]f32{-1, math.nan_f32(), math.inf_f32(1)})
	{
		state, status := entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, time, stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
		testing.expect_value(t, state, entasis.Query_Hit_State.Miss);
	}
	for invalid in ([3]entasis.Body_Velocity{{linear={math.nan_f32(), 0, 0}}, {angular={0, math.inf_f32(1), 0}}, {linear={0, 0, math.inf_f32(-1)}}})
	{
		_, status := entasis.sweep_global_any_with_context(&scene.query, shape, pose, invalid, 4, stack[:]);
		testing.expect_value(t, status, entasis.Status.Invalid_Argument);
	}
	for invalid in ([5]entasis.Sweep_Settings{
		{minimum_progression=math.nan_f32()}, {convergence_threshold=math.inf_f32(1)},
		{minimum_progression=-1}, {convergence_threshold=-1}, {maximum_iteration_count=-1},
	})
	{
		_, status := entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 4, stack[:], settings=invalid);
		testing.expect_value(t, status, entasis.Status.Invalid_Description);
	}
	_, status := entasis.sweep_global_any_with_context(&scene.query, {}, pose, velocity, 4, stack[:]);
	testing.expect(t, status != .Ok);
	state: entasis.Query_Hit_State;
	state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Query_Hit_State.Hit);
	pose.position = {1e100, 0, 0};
	state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, state, entasis.Query_Hit_State.Miss);
	dead: entasis.Query_Context;
	_, status = entasis.sweep_global_any_with_context(&dead, shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
	_, status = entasis.sweep_global_closest_with_context(&dead, shape, pose, velocity, 4, stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
	_, status = entasis.sweep_global_all_with_context(&dead, shape, pose, velocity, 4, hits[:], stack[:]);
	testing.expect_value(t, status, entasis.Status.Disposed);
}

@(test)
global_sweep_initial_overlap_zero_time_and_closed_end :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	_ = add_static(t, &scene, shape, {1e9 + 0.25, -1e9, 1e9});
	stack: [128]i32;
	pose: entasis.Global_Pose = {position={1e9, -1e9, 1e9}, orientation=util.quaternion_identity()};
	settings: entasis.Sweep_Settings = {minimum_progression=0.01, convergence_threshold=1e-5, maximum_iteration_count=32};
	for maximum in ([2]f32{0, 4})
	{
		for velocity in ([2]entasis.Body_Velocity{{}, {linear={1, 0, 0}}})
		{
			hit, status := entasis.sweep_global_closest_with_context(&scene.query, shape, pose, velocity, maximum, stack[:], settings=settings);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, hit.sweep.t1, f32(0));
			state: entasis.Query_Hit_State;
			state, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, maximum, stack[:], settings=settings);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, state, entasis.Query_Hit_State.Hit);
		}
	}
	pose.position.x = 1e9 - 2.75;
	hit, status := entasis.sweep_global_closest_with_context(&scene.query, shape, pose, {linear={1, 0, 0}}, 2, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect(t, abs(hit.sweep.t1 - 2) <= 0.00002);
}
