package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_ray_all_builtin_shapes_match_local_geometry :: proc(t: ^testing.T)
{
	near: Scene;
	far: Scene;
	scene_initialize(t, &near);
	scene_initialize(t, &far);
	defer scene_dispose(t, &far);
	defer scene_dispose(t, &near);
	offset: physics.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9 - 0.375};
	near_hits: [16]entasis.Ray_Hit;
	far_hits: [16]entasis.Global_Ray_Hit;
	stack: [128]i32;
	for kind in Shape_Kind
	{
		shape_a: physics.Typed_Index = add_shape(t, &near, kind);
		shape_b: physics.Typed_Index = add_shape(t, &far, kind);
		position: physics.Global_Position = {f64(int(kind) * 8), 0, 0};
		_ = add_static(t, &near, shape_a, position);
		_ = add_static(t, &far, shape_b, {position.x + offset.x, offset.y, offset.z});
	}
	for kind in Shape_Kind
	{
		position: f64 = f64(int(kind) * 8);
		for x in ([3]f64{-0.25, 0, 0.25})
		{
			segment: physics.World_Precision_Segment = {start={position + x, 2, -0.25}, end={position + x, -2, -0.25}};
			local_ray: entasis.Ray = {origin={f32(segment.start.x), 2, -0.25}, direction={0, -1, 0}, maximum_t=4};
			near_count, far_count: int;
			status: entasis.Status;
			near_count, status = entasis.ray_cast_all_with_context(&near.query, local_ray, near_hits[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			far_count, status = entasis.ray_cast_global_all_with_context(&far.query, translated_segment(segment, offset), far_hits[:], stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expectf(t, far_count == near_count, "shape %v x %v near %v far %v", kind, x, near_count, far_count);
			if x == -0.25
			{
				testing.expectf(t, near_count > 0, "shape %v must exercise actual geometry", kind);
			}
			for far_hit in far_hits[:far_count]
			{
				matched: physics.Reference_State;
				for near_hit in near_hits[:near_count]
				{
					if far_hit.child_index != near_hit.child_index
					{
						continue;
					}
					matched = .Present;
					testing.expect_value(t, far_hit.collidable, near_hit.collidable);
					testing.expectf(t, abs(far_hit.t - f64(near_hit.t) / 4) <= 1e-7, "shape %v near %v far %v", kind, near_hit.t / 4, far_hit.t);
					expect_position(t, far_hit.location, {offset.x + f64(near_hit.location.x), offset.y + f64(near_hit.location.y), offset.z + f64(near_hit.location.z)});
					testing.expect(t, util.vector3_length(util.vector3_subtract(far_hit.normal, near_hit.normal)) <= 0.00001);
				}
				testing.expect_value(t, matched, physics.Reference_State.Present);
			}
		}
	}
}

@(test)
global_rotated_oblique_queries_match_local_kernels :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	center: physics.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9 - 0.375};
	handle: physics.Static_Handle = add_static(t, &scene, shape, center);
	index, _ := physics.statics_resolve(&scene.simulation.statics, handle);
	orientation: util.Quaternion = util.quaternion_from_yaw_pitch_roll(0.43, -0.72, 0.19);
	scene.simulation.statics.statics.memory[index].pose.orientation = orientation;
	testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
	stack: [128]i32;
	matched: int;
	for ray_index in 0 ..< 4096
	{
		// deterministic rational slopes, with endpoints narrowed only after
		// authoritative relative subtraction for the independent local oracle
		y: f64 = f64((ray_index * 37) % 2048 - 1024) / 512;
		z: f64 = f64((ray_index * 73) % 2048 - 1024) / 512;
		segment: physics.World_Precision_Segment = translated_segment({start={-3, y, z}, end={3, -0.37 * y, 0.19 * z}}, center);
		direction: util.Vector3 = {f32(segment.end.x - segment.start.x), f32(segment.end.y - segment.start.y), f32(segment.end.z - segment.start.z)};
		local: physics.Tree_Ray = {
			origin={f32(segment.start.x - center.x), f32(segment.start.y - center.y), f32(segment.start.z - center.z)},
			direction=direction, maximum_t=1,
		};
		expected, local_status := physics.box_ray_test({0.5, 0.5, 0.5}, {orientation=orientation}, local);
		testing.expect_value(t, local_status, physics.Physics_Status.Ok);
		hit, status := entasis.ray_cast_global_closest_with_context(&scene.query, segment, stack[:]);
		if expected.state == .Missing
		{
			testing.expect_value(t, status, entasis.Status.Not_Found);
		}
		else
		{
			matched += 1;
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expectf(t, abs(hit.t - f64(expected.t)) <= 0.000001, "ray %v actual %v expected %v", ray_index, hit.t, expected.t);
			expected_location: util.Vector3 = util.vector3_add(local.origin, util.vector3_scale(local.direction, expected.t));
			expect_position(t, hit.location, {center.x + f64(expected_location.x), center.y + f64(expected_location.y), center.z + f64(expected_location.z)});
			testing.expect(t, util.vector3_length(util.vector3_subtract(hit.normal, expected.normal)) <= 0.00001);
		}
	}
	testing.expect(t, matched > 256 && matched < 3840);
}
