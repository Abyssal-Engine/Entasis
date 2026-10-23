package precision_queries_tests

import "core:testing"
import e "entasis:entasis"

batch_set_output :: proc "contextless" (query: ^e.Global_Query, output: e.Query_Output)
{
	switch query.kind
	{
	case .Ray_All:
		query.ray_data.output = output;
	case .Sweep_All:
		query.sweep_data.output = output;
	case .Overlap_All:
		query.overlap_data.output = output;
	case .Volume_All:
		query.volume_data.output = output;
	case .Ray_Any, .Ray_Closest, .Sweep_Any, .Sweep_Closest, .Overlap_Any:
		return;
	case:
		return;
	}
}

@(test)
global_spatial_batch_admission_and_invalid_ranges_write_no_output :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	_ = add_static(t, &scene, shape, origin);
	queries: [9]e.Global_Query = batch_spatial_inputs(shape, origin);
	results: [10]e.Global_Query_Result;
	for &result in results
	{
		result.count = 99;
		result.status = .No_Convergence;
	}
	stack: [128]i32;
	rays: [16]e.Global_Ray_Hit;
	sweeps: [16]e.Global_Sweep_Hit;
	overlaps: [16]e.Overlap_Hit;
	volumes: [16]e.Volume_Hit;
	scratch: e.Global_Query_Scratch = {rays[:], sweeps[:], overlaps[:], volumes[:], stack[:]};
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:8], scratch), e.Status.Invalid_Argument);
	testing.expect_value(t, e.query_batch_global_with_context(nil, queries[:], results[:], scratch), e.Status.Disposed);
	for result in results
	{
		testing.expect_value(t, result.count, i32(99));
		testing.expect_value(t, result.status, e.Status.No_Convergence);
	}
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, nil, nil), e.Status.Ok);
	for index in ([4]int{2, 5, 7, 8})
	{
		for output in ([5]e.Query_Output{{-1, 1}, {0, -1}, {17, 0}, {1, 2147483647}, {2147483647, 1}})
		{
			pair: [2]e.Global_Query = {queries[index], queries[1]};
			batch_set_output(&pair[0], output);
			testing.expect_value(t, e.query_batch_global_with_context(&scene.query, pair[:], results[:], scratch), e.Status.Invalid_Argument);
			testing.expect_value(t, results[0].status, e.Status.Invalid_Argument);
			testing.expect_value(t, results[0].count, i32(0));
			testing.expect_value(t, results[1].status, e.Status.Ok);
			testing.expect_value(t, results[1].count, i32(1));
		}
	}
	invalid: [2]e.Global_Query = {{kind=e.Global_Query_Kind(255)}, queries[1]};
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, invalid[:], results[:], scratch), e.Status.Invalid_Argument);
	testing.expect_value(t, results[0].status, e.Status.Invalid_Argument);
	testing.expect_value(t, results[1].status, e.Status.Ok);
	testing.expect_value(t, results[9].count, i32(99));
	for i in 0 ..< 16
	{
		testing.expect_value(t, rays[i], e.Global_Ray_Hit{});
		testing.expect_value(t, sweeps[i], e.Global_Sweep_Hit{});
		testing.expect_value(t, overlaps[i], e.Overlap_Hit{});
		testing.expect_value(t, volumes[i], e.Volume_Hit{});
	}
}

@(test)
global_spatial_batch_all_hit_failures_preserve_prefix_and_guard_slots :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	origin: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	_ = add_static(t, &scene, shape, origin);
	_ = add_static(t, &scene, shape, {origin.x, origin.y - 0.125, origin.z});
	queries: [9]e.Global_Query = batch_spatial_inputs(shape, origin);
	results: [1]e.Global_Query_Result;
	stack: [128]i32;
	rays: [3]e.Global_Ray_Hit;
	sweeps: [3]e.Global_Sweep_Hit;
	overlaps: [3]e.Overlap_Hit;
	volumes: [3]e.Volume_Hit;
	scratch: e.Global_Query_Scratch = {rays[:], sweeps[:], overlaps[:], volumes[:], stack[:]};
	for index in ([4]int{2, 5, 7, 8})
	{
		for i in 0 ..< 3
		{
			rays[i] = {child_index=99};
			sweeps[i] = {collidable={packed=0xdeadbeef}};
			overlaps[i] = {collidable={packed=0xdeadbeef}};
			volumes[i] = {collidable={packed=0xdeadbeef}};
		}
		query: [1]e.Global_Query = {queries[index]};
		batch_set_output(&query[0], {1, 1});
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, query[:], results[:], scratch), e.Status.Capacity_Missing);
		testing.expect_value(t, results[0].count, i32(1));
		testing.expect_value(t, results[0].hit, e.Query_Hit_State.Hit);
		for guard in ([2]int{0, 2})
		{
			testing.expect_value(t, rays[guard].child_index, i32(99));
			testing.expect_value(t, sweeps[guard].collidable.packed, u32(0xdeadbeef));
			testing.expect_value(t, overlaps[guard].collidable.packed, u32(0xdeadbeef));
			testing.expect_value(t, volumes[guard].collidable.packed, u32(0xdeadbeef));
		}
		batch_set_output(&query[0], {2, 0});
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, query[:], results[:], scratch), e.Status.Capacity_Missing);
		testing.expect_value(t, results[0].count, i32(0));
		testing.expect_value(t, results[0].hit, e.Query_Hit_State.Miss);
		scratch.traversal_stack = nil;
		batch_set_output(&query[0], {1, 1});
		testing.expect_value(t, e.query_batch_global_with_context(&scene.query, query[:], results[:], scratch), e.Status.Capacity_Missing);
		testing.expect_value(t, results[0].count, i32(0));
		scratch.traversal_stack = stack[:];
	}
}

@(test)
global_spatial_batch_empty_world_misses_need_no_traversal_or_output :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: e.Shape_Handle = add_shape(t, &scene, .Sphere);
	queries: [9]e.Global_Query = batch_spatial_inputs(shape, {1e9, 0, 0});
	for &query in queries
	{
		batch_set_output(&query, {});
	}
	results: [9]e.Global_Query_Result;
	testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:]), e.Status.Not_Found);
	for result, index in results
	{
		testing.expect_value(t, result.status, e.Status.Not_Found if index == 1 || index == 4 else e.Status.Ok);
		testing.expect_value(t, result.count, i32(0));
		testing.expect_value(t, result.hit, e.Query_Hit_State.Miss);
	}
}
