package precision_queries_tests

import "core:testing"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

// one query for each spatial route, with nonzero output offsets and guard slots
batch_spatial_inputs :: proc "contextless" (shape: e.Shape_Handle, origin: e.Global_Position) -> [9]e.Global_Query
{
	ray: e.Global_Ray_Query = {
		segment={start={origin.x, origin.y + 3, origin.z}, end={origin.x, origin.y - 3, origin.z}},
		output={1, 14},
	};
	sweep: e.Global_Sweep_Query = {
		shape=shape, pose={position=ray.segment.start, orientation=util.quaternion_identity()},
		velocity={linear={0, -6, 0}}, maximum_t=1, output={1, 14},
	};
	overlap: e.Global_Overlap_Query = {
		shape=shape, pose={position=origin, orientation=util.quaternion_identity()}, output={1, 14},
	};
	volume: e.Global_Volume_Query = {
		bounds={min={origin.x - 2, origin.y - 2, origin.z - 2}, max={origin.x + 2, origin.y + 2, origin.z + 2}},
		output={1, 14},
	};
	return {
		{kind=.Ray_Any, ray_data=ray}, {kind=.Ray_Closest, ray_data=ray}, {kind=.Ray_All, ray_data=ray},
		{kind=.Sweep_Any, sweep_data=sweep}, {kind=.Sweep_Closest, sweep_data=sweep}, {kind=.Sweep_All, sweep_data=sweep},
		{kind=.Overlap_Any, overlap_data=overlap}, {kind=.Overlap_All, overlap_data=overlap}, {kind=.Volume_All, volume_data=volume},
	};
}

@(test)
global_spatial_batch_all_modes_match_single_queries_across_shapes :: proc(t: ^testing.T)
{
	for kind in Shape_Kind
	{
		scene: Scene;
		scene_initialize(t, &scene);
		target: e.Shape_Handle = add_shape(t, &scene, kind);
		probe: e.Shape_Handle = add_shape(t, &scene, .Sphere);
		origin: e.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9 - 0.375};
		_ = add_static(t, &scene, target, origin);
		queries: [9]e.Global_Query = batch_spatial_inputs(probe, origin);
		results: [10]e.Global_Query_Result;
		results[9].count = 99;
		stack: [128]i32;
		rays, expected_rays: [16]e.Global_Ray_Hit;
		sweeps, expected_sweeps: [16]e.Global_Sweep_Hit;
		overlaps, expected_overlaps: [16]e.Overlap_Hit;
		volumes, expected_volumes: [16]e.Volume_Hit;
		rays[0].child_index = 91;
		rays[15].child_index = 92;
		scratch: e.Global_Query_Scratch = {rays[:], sweeps[:], overlaps[:], volumes[:], stack[:]};
		batch_status: e.Status = e.query_batch_global_with_context(&scene.query, queries[:], results[:], scratch);
		first: e.Status = .Ok;
		for &query, index in queries
		{
			result: ^e.Global_Query_Result = &results[index];
			status: e.Status;
			count: int;
			switch query.kind
			{
			case .Ray_Any:
				state: e.Query_Hit_State;
				state, status = e.ray_cast_global_any_with_context(&scene.query, query.ray_data.segment, stack[:]);
				count = 1 if state == .Hit else 0;
			case .Ray_Closest:
				hit: e.Global_Ray_Hit;
				hit, status = e.ray_cast_global_closest_with_context(&scene.query, query.ray_data.segment, stack[:]);
				count = 1 if status == .Ok else 0;
				testing.expect_value(t, result.ray_hit, hit);
			case .Ray_All:
				count, status = e.ray_cast_global_all_with_context(&scene.query, query.ray_data.segment, expected_rays[:], stack[:]);
				for i in 0 ..< count
				{
					testing.expect_value(t, rays[i + 1], expected_rays[i]);
				}
			case .Sweep_Any:
				state: e.Query_Hit_State;
				state, status = e.sweep_global_any_with_context(&scene.query, probe, query.sweep_data.pose, query.sweep_data.velocity, 1, stack[:]);
				count = 1 if state == .Hit else 0;
			case .Sweep_Closest:
				hit: e.Global_Sweep_Hit;
				hit, status = e.sweep_global_closest_with_context(&scene.query, probe, query.sweep_data.pose, query.sweep_data.velocity, 1, stack[:]);
				count = 1 if status == .Ok else 0;
				testing.expect_value(t, result.sweep_hit, hit);
			case .Sweep_All:
				count, status = e.sweep_global_all_with_context(&scene.query, probe, query.sweep_data.pose, query.sweep_data.velocity, 1, expected_sweeps[:], stack[:]);
				for i in 0 ..< count
				{
					testing.expect_value(t, sweeps[i + 1], expected_sweeps[i]);
				}
			case .Overlap_Any:
				state: e.Overlap_State;
				state, status = e.overlap_global_any_with_context(&scene.query, probe, query.overlap_data.pose, stack[:]);
				count = 1 if state == .Intersecting else 0;
			case .Overlap_All:
				count, status = e.overlap_global_all_with_context(&scene.query, probe, query.overlap_data.pose, expected_overlaps[:], stack[:]);
				for i in 0 ..< count
				{
					testing.expect_value(t, overlaps[i + 1], expected_overlaps[i]);
				}
			case .Volume_All:
				count, status = e.volume_global_all_with_context(&scene.query, query.volume_data.bounds, expected_volumes[:], stack[:]);
				for i in 0 ..< count
				{
					testing.expect_value(t, volumes[i + 1], expected_volumes[i]);
				}
			}
			testing.expect_value(t, result.status, status);
			testing.expect_value(t, result.count, i32(count));
			testing.expect_value(t, result.hit, e.Query_Hit_State.Hit if count > 0 else e.Query_Hit_State.Miss);
			if first == .Ok && status != .Ok
			{
				first = status;
			}
		}
		testing.expect_value(t, batch_status, first);
		testing.expect_value(t, results[9].count, i32(99));
		testing.expect_value(t, rays[0].child_index, i32(91));
		testing.expect_value(t, rays[15].child_index, i32(92));
		scene_dispose(t, &scene);
	}
}

@(test)
global_spatial_batch_preserves_millimeter_distinctions_and_indices :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e3, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		shape: e.Shape_Handle = add_shape(t, &scene, .Box);
		center: e.Global_Position = {origin + 0.125, -origin + 0.25, origin - 0.375};
		_ = add_body(t, &scene, shape, center);
		for axis in 0 ..< 3
		{
			for sign in ([2]f64{-1, 1})
			{
				queries: [257]e.Global_Query;
				results: [258]e.Global_Query_Result;
				results[257].count = 456;
				stack: [128]i32;
				for &query, index in queries
				{
					start: [3]f64 = {center.x, center.y, center.z};
					end: [3]f64 = start;
					start[axis] -= 2 * sign;
					end[axis] += 2 * sign;
					start[(axis + 1) % 3] += 0.498 if index % 2 == 0 else 0.502;
					end[(axis + 1) % 3] = start[(axis + 1) % 3];
					query = {kind=.Ray_Closest, ray_data={segment={start={start[0], start[1], start[2]}, end={end[0], end[1], end[2]}}}};
				}
				testing.expect_value(t, e.query_batch_global_with_context(&scene.query, queries[:], results[:], {traversal_stack=stack[:]}), e.Status.Not_Found);
				for &result, index in results[:257]
				{
					if index % 2 == 0
					{
						testing.expect_value(t, result.status, e.Status.Ok);
						testing.expect_value(t, result.count, i32(1));
						testing.expect_value(t, result.hit, e.Query_Hit_State.Hit);
						testing.expect(t, abs(result.ray_hit.t - 0.375) < 1e-6);
					}
					else
					{
						testing.expect_value(t, result.status, e.Status.Not_Found);
						testing.expect_value(t, result.count, i32(0));
						testing.expect_value(t, result.hit, e.Query_Hit_State.Miss);
						testing.expect_value(t, result.ray_hit.child_index, i32(-1));
					}
				}
				testing.expect_value(t, results[257].count, i32(456));
			}
		}
		scene_dispose(t, &scene);
	}
}
