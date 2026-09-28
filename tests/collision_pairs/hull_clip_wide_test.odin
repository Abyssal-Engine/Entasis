package collision_pairs_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

hull_test_random :: proc(seed: ^u32) -> f32
{
	seed^ = seed^ * 1664525 + 1013904223;
	return f32(seed^ >> 8) * (1.0 / 16777216.0);
}

hull_test_expect_manifold :: proc(t: ^testing.T, expected, actual: physics.Convex_Contact_Manifold)
{
	testing.expect_value(t, actual.count, expected.count);
	if expected.count > 0 && actual.count > 0
	{
		normal_delta := util.vector3_subtract(actual.normal, expected.normal);
		offset_delta := util.vector3_subtract(actual.offset_b, expected.offset_b);
		testing.expect(t, util.vector3_length_squared(normal_delta) < 1e-9);
		testing.expect(t, util.vector3_length_squared(offset_delta) < 1e-9);
	}
	for i in 0 ..< min(int(expected.count), int(actual.count))
	{
		testing.expect_value(t, actual.contacts[i].feature_id, expected.contacts[i].feature_id);
		testing.expect(t, abs(actual.contacts[i].depth - expected.contacts[i].depth) < 2e-5);
		delta := util.vector3_subtract(actual.contacts[i].offset, expected.contacts[i].offset);
		testing.expect(t, util.vector3_length_squared(delta) < 1e-9);
	}
}

@(test)
hull_small_candidate_reduction_preserves_scalar_order_and_features :: proc(t: ^testing.T)
{
	seed := u32(17391);
	for iteration in 0 ..< 256
	{
		events: [12]physics.Hull_Clip_Candidate_Wide;
		maximum_count, active: util.I32x8;
		center_a: util.Vector3_Wide;
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			maximum_count = simd.replace(maximum_count, lane, i32(3 + (iteration + lane) % 6));
			active = simd.replace(active, lane, i32(-int((iteration + lane) % 7 != 0)));
			util.vector3_wide_write_slot(&center_a, lane, {0, hull_test_random(&seed) * 0.6 - 0.3, 0});
			for event in 0 ..< 12
			{
				events[event].x = simd.replace(events[event].x, lane, hull_test_random(&seed) * 4 - 2);
				events[event].y = simd.replace(events[event].y, lane, hull_test_random(&seed) * 4 - 2);
				events[event].feature_id = simd.replace(events[event].feature_id, lane, i32(event * 31 + lane));
				events[event].exists = simd.replace(events[event].exists, lane, i32(-int(hull_test_random(&seed) > 0.25)));
			}
		}
		normal_a := util.Vector3{0.15, 1, -0.3};
		x := util.Vector3{1, 0, 0};
		y := util.Vector3{0, 0, 1};
		rotation := util.Matrix3x3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
		wide := physics.manifold_candidate_reduce_small_wide(&events, maximum_count, active,
			util.vector3_wide_broadcast(normal_a), util.F32x8(1), center_a, {},
			util.vector3_wide_broadcast(x), util.vector3_wide_broadcast(y), util.F32x8(1), util.F32x8(-0.1),
			util.matrix3x3_wide_broadcast(rotation), {}, util.vector3_wide_broadcast({0, 1, 0}));
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			candidates: [physics.MAXIMUM_MANIFOLD_CANDIDATE_COUNT]physics.Manifold_Candidate_Scalar = ---;
			count := 0;
			if simd.extract(active, lane) < 0
			{
				for event in events
				{
					if simd.extract(event.exists, lane) < 0 && count < int(simd.extract(maximum_count, lane))
					{
						candidates[count] = {simd.extract(event.x, lane), simd.extract(event.y, lane), simd.extract(event.feature_id, lane)};
						count += 1;
					}
				}
			}
			expected, status := physics.manifold_candidate_reduce(&candidates, count, normal_a, 1,
				util.vector3_wide_read_slot(center_a, lane), {}, x, y, 1, -0.1, rotation, {}, {0, 1, 0});
			actual, read_status := physics.convex_4_manifold_wide_read_lane(&wide, {}, lane);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, read_status, physics.Physics_Status.Ok);
			hull_test_expect_manifold(t, expected, actual);
		}
	}
}

@(test)
hull_small_face_clipping_matches_scalar_for_rotated_irregular_hulls :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	seed := u32(76321);
	for shape_kind in 0 ..< 4
	{
		points, starts, indices := cube_hull_data();
		for &point in points
		{
			if shape_kind == 1
			{
				point.x += 0.25 * point.y + 0.11 * point.z;
				point.y += 0.15 * point.z;
			}
		}
		hull: physics.Convex_Hull;
		if shape_kind == 2
		{
			tetra := [4]util.Vector3{{-1, -1, -1}, {1.2, -0.8, -1}, {0.2, 1.3, -0.7}, {0, 0, 1.4}};
			center: util.Vector3;
			testing.expect_value(t, physics.convex_hull_create_from_point_cloud(&hull, &center, &tetra[0], 4, &pool), physics.Physics_Status.Ok);
		}
		else if shape_kind == 3
		{
			phi := f32((1 + math.sqrt(5.0)) * 0.5);
			icosahedron := [12]util.Vector3{
				{0, -1, -phi}, {0, -1, phi}, {0, 1, -phi}, {0, 1, phi},
				{-1, -phi, 0}, {-1, phi, 0}, {1, -phi, 0}, {1, phi, 0},
				{-phi, 0, -1}, {-phi, 0, 1}, {phi, 0, -1}, {phi, 0, 1},
			};
			center: util.Vector3;
			testing.expect_value(t, physics.convex_hull_create_from_point_cloud(&hull, &center, &icosahedron[0], 12, &pool), physics.Physics_Status.Ok);
			testing.expect(t, hull.points.length > 1);
		}
		else
		{
			testing.expect_value(t, physics.convex_hull_create(&hull, &points[0], 8, &starts[0], 6, &indices[0], 24, &pool), physics.Physics_Status.Ok);
		}
		for iteration in 0 ..< 64
		{
			rotation_a, rotation_b: util.Matrix3x3_Wide;
			local_offset, local_normal, normal_a, normal_b: util.Vector3_Wide;
			indices_a, indices_b, active: util.I32x8;
			pair_count := iteration % 8 + 1;
			for lane in 0 ..< pair_count
			{
				angle := hull_test_random(&seed) * 1.2 - 0.6;
				s, c := math.sin(angle), math.cos(angle);
				rotation := util.Matrix3x3{{c, s, 0}, {-s, c, 0}, {0, 0, 1}};
				util.vector3_wide_write_slot(&rotation_a.x, lane, rotation.x);
				util.vector3_wide_write_slot(&rotation_a.y, lane, rotation.y);
				util.vector3_wide_write_slot(&rotation_a.z, lane, rotation.z);
				rotation_b = util.matrix3x3_wide_identity();
				face_b := (iteration + lane) % int(hull.face_start_indices.length);
				n_b, _ := physics.convex_hull_get_face_plane(&hull, face_b);
				best_dot := f32(2);
				face_a := 0;
				n_a: util.Vector3;
				for face in 0 ..< int(hull.face_start_indices.length)
				{
					n, _ := physics.convex_hull_get_face_plane(&hull, face);
					dot := util.vector3_dot(util.matrix3x3_transform(n, rotation), n_b);
					if dot < best_dot
					{
						best_dot, face_a, n_a = dot, face, n;
					}
				}
				util.vector3_wide_write_slot(&local_normal, lane, n_b);
				util.vector3_wide_write_slot(&normal_a, lane, n_a);
				util.vector3_wide_write_slot(&normal_b, lane, n_b);
				util.vector3_wide_write_slot(&local_offset, lane, util.vector3_scale(n_b, hull_test_random(&seed) * 2.6));
				indices_a = simd.replace(indices_a, lane, i32(face_a));
				indices_b = simd.replace(indices_b, lane, i32(face_b));
				active = simd.replace(active, lane, i32(-int((iteration + lane) % 5 != 0)));
			}
			offset_b := util.vector3_wide_negate(local_offset);
			wide, handled, status := physics.collision_hull_pair_clip_small_faces_wide(&hull, &hull, offset_b, rotation_a,
				local_offset, local_normal, normal_a, normal_b, indices_a, indices_b, util.F32x8(1), util.F32x8(-0.1), rotation_b, active, pair_count);
			testing.expect_value(t, handled, physics.Reference_State.Present);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			for lane in 0 ..< 8
			{
				actual, read_status := physics.convex_4_manifold_wide_read_lane(&wide, offset_b, lane);
				testing.expect_value(t, read_status, physics.Physics_Status.Ok);
				if lane >= pair_count || simd.extract(active, lane) >= 0
				{
					testing.expect_value(t, actual.count, i32(0));
					continue;
				}
				expected, scalar_status := physics.collision_hull_pair_build_manifold(&hull, &hull,
					util.vector3_wide_read_slot(offset_b, lane), util.matrix3x3_wide_read_slot(rotation_a, lane),
					util.vector3_wide_read_slot(local_offset, lane), util.vector3_wide_read_slot(local_normal, lane),
					util.vector3_wide_read_slot(normal_a, lane), util.vector3_wide_read_slot(normal_b, lane),
					int(simd.extract(indices_a, lane)), int(simd.extract(indices_b, lane)), 1, -0.1, util.matrix3x3_wide_read_slot(rotation_b, lane));
				testing.expect_value(t, scalar_status, physics.Physics_Status.Ok);
				hull_test_expect_manifold(t, expected, actual);
			}
		}
		testing.expect_value(t, physics.convex_hull_dispose(&hull, &pool), physics.Physics_Status.Ok);
	}
}

@(test)
hull_direct_groups_preserve_standard_wide_results_for_mixed_shapes_and_tails :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	points, starts, indices := cube_hull_data();
	cube, irregular: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(&cube, &points[0], 8, &starts[0], 6, &indices[0], 24, &pool), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&cube, &pool);
	points[0].x += 0.18;
	points[2].y -= 0.13;
	points[5].z -= 0.24;
	center: util.Vector3;
	testing.expect_value(t, physics.convex_hull_create_from_point_cloud(&irregular, &center, &points[0], 8, &pool), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&irregular, &pool);
	for mixed in 0 ..< 2
	{
		for count in 1 ..= 8
		{
			records: [10]physics.Narrow_Phase_Convex_Direct_Record;
			a, b: physics.Convex_Hull_Wide;
			offsets: util.Vector3_Wide;
			qa, qb: util.Quaternion_Wide;
			for lane in 0 ..< count
			{
				angle := f32(lane) * 0.13;
				shape := &irregular;
				if mixed == 1 && lane % 2 == 0
				{
					shape = &cube;
				}
				a.hulls[lane], b.hulls[lane] = shape, shape;
				pose_a := physics.Rigid_Pose{position={100, -20, 30}, orientation={0, math.sin(angle * 0.5), 0, math.cos(angle * 0.5)}};
				pose_b := physics.Rigid_Pose{position={101.65, -19.8 + f32(lane) * 0.08, 30.1}, orientation={math.sin(angle * 0.3), 0, 0, math.cos(angle * 0.3)}};
				records[lane + 2] = {pair_id=i32(lane), speculative_margin=0.1, shape_data_a=shape, shape_data_b=shape, pose_a=pose_a, pose_b=pose_b};
				util.vector3_wide_write_slot(&offsets, lane, util.vector3_subtract(pose_b.position, pose_a.position));
				util.quaternion_wide_write_slot(&qa, lane, pose_a.orientation);
				util.quaternion_wide_write_slot(&qb, lane, pose_b.orientation);
			}
			expected, status := physics.convex_hull_pair_test_wide(a, b, util.F32x8(0.1), offsets, qa, qb, count);
			actual: physics.Convex_4_Contact_Manifold_Wide;
			actual_offsets: util.Vector3_Wide;
			testing.expect_value(t, physics.narrow_phase_hull_direct_test_group(&records[0], 2, count, &actual_offsets, &actual), status);
			unpacked: [8]physics.Convex_Contact_Manifold;
			physics.narrow_phase_hull_direct_unpack(&actual, actual_offsets, &unpacked);
			for lane in 0 ..< 8
			{
				actual_lane, read_status := physics.convex_4_manifold_wide_read_lane(&actual, actual_offsets, lane);
				testing.expect_value(t, read_status, physics.Physics_Status.Ok);
				hull_test_expect_manifold(t, actual_lane, unpacked[lane]);
				if lane < count
				{
					expected_lane, expected_status := physics.convex_4_manifold_wide_read_lane(&expected, offsets, lane);
					testing.expect_value(t, expected_status, physics.Physics_Status.Ok);
					hull_test_expect_manifold(t, expected_lane, actual_lane);
				}
				else
				{
					testing.expect_value(t, actual_lane.count, i32(0));
				}
			}
		}
	}
}

@(test)
hull_small_face_kernel_defers_larger_polygons_to_general_clipping :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	points: [12]util.Vector3;
	for i in 0 ..< 6
	{
		angle := f32(i) * f32(math.PI / 3);
		points[i] = {math.cos(angle), -1, math.sin(angle)};
		points[i + 6] = {math.cos(angle), 1, math.sin(angle)};
	}
	hull: physics.Convex_Hull;
	center: util.Vector3;
	testing.expect_value(t, physics.convex_hull_create_from_point_cloud(&hull, &center, &points[0], 12, &pool), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	large_face := -1;
	for face in 0 ..< int(hull.face_start_indices.length)
	{
		start, end, status := physics.convex_hull_face_range(&hull, face);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		if end - start > 4
		{
			large_face = face;
			break;
		}
	}
	testing.expect(t, large_face >= 0);
	if large_face < 0
	{
		return;
	}
	_, handled, status := physics.collision_hull_pair_clip_small_faces_wide(&hull, &hull, {}, util.matrix3x3_wide_identity(), {},
		util.vector3_wide_broadcast({0, 1, 0}), util.vector3_wide_broadcast({0, -1, 0}), util.vector3_wide_broadcast({0, 1, 0}),
		util.I32x8(i32(large_face)), util.I32x8(i32(large_face)), util.F32x8(1), util.F32x8(-0.1), util.matrix3x3_wide_identity(), util.I32x8(-1), 8);
	testing.expect_value(t, handled, physics.Reference_State.Missing);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
}
