package collision_pairs_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
hull_shared_support_preserves_first_maximum_and_point_bits :: proc(t: ^testing.T)
{
	seed := u32(436711);
	points: util.Vector3_Wide;
	hull := physics.Convex_Hull{points={memory=&points, length=1}};
	for iteration in 0 ..< 512
	{
		for point_index in 0 ..< 8
		{
			point := util.Vector3{
				hull_test_random(&seed) * 4 - 2,
				hull_test_random(&seed) * 4 - 2,
				hull_test_random(&seed) * 4 - 2,
			};
			if point_index == 0 && iteration % 5 == 0
			{
				point.y = transmute(f32)u32(0x80000000);
			}
			if iteration % 4 == 0
			{
				point.x = f32(point_index / 2);
			}
			if point_index >= 4 && iteration % 3 == 0
			{
				point = util.vector3_wide_read_slot(points, 0);
			}
			if iteration % 17 == 0 && point_index == (iteration / 17) % 8
			{
				point.z = transmute(f32)u32(0x7fc00001);
			}
			util.vector3_wide_write_slot(&points, point_index, point);
		}
		directions: util.Vector3_Wide;
		for lane in 0 ..< 8
		{
			direction := util.Vector3{
				hull_test_random(&seed) * 2 - 1,
				hull_test_random(&seed) * 2 - 1,
				hull_test_random(&seed) * 2 - 1,
			};
			if lane == 0
			{
				direction = {};
			}
			else if lane == 1
			{
				direction = {1, 0, 0};
			}
			else if lane == 2
			{
				direction = {-1, 0, 0};
			}
			util.vector3_wide_write_slot(&directions, lane, direction);
		}
		actual := physics.hull_pair_support_shared_bundle_wide(&hull, directions);
		for lane in 0 ..< 8
		{
			direction := util.vector3_wide_read_slot(directions, lane);
			expected := util.vector3_wide_read_slot(points, 0);
			best_dot := util.vector3_dot(expected, direction);
			for point_index in 1 ..< 8
			{
				point := util.vector3_wide_read_slot(points, point_index);
				dot := util.vector3_dot(point, direction);
				if dot > best_dot
				{
					expected, best_dot = point, dot;
				}
			}
			selected := util.vector3_wide_read_slot(actual, lane);
			testing.expect_value(t, transmute([3]u32)selected, transmute([3]u32)expected);
		}
	}
}

@(test)
hull_shared_face_selection_preserves_ordered_error_ties :: proc(t: ^testing.T)
{
	seed := u32(160511);
	planes: physics.Hull_Bounding_Plane_Bundle;
	hull := physics.Convex_Hull{bounding_planes={memory=&planes, length=1}};
	for face_count in 1 ..= 8
	{
		hull.face_start_indices.length = i32(face_count);
		for iteration in 0 ..< 64
		{
			for face in 0 ..< face_count
			{
				normal := util.Vector3{
					hull_test_random(&seed) * 2 - 1,
					hull_test_random(&seed) * 2 - 1,
					hull_test_random(&seed) * 2 - 1,
				};
				normal = util.vector3_scale(normal, 1 / math.sqrt(util.vector3_length_squared(normal)));
				offset := hull_test_random(&seed) * 2;
				if face > 0 && iteration % 3 == 0
				{
					normal = util.vector3_wide_read_slot(planes.normal, 0);
					offset = simd.extract(planes.offset, 0);
				}
				util.vector3_wide_write_slot(&planes.normal, face, normal);
				planes.offset = simd.replace(planes.offset, face, offset);
			}
			normals, closest: util.Vector3_Wide;
			for lane in 0 ..< 8
			{
				util.vector3_wide_write_slot(&normals, lane, {
					hull_test_random(&seed), hull_test_random(&seed), hull_test_random(&seed),
				});
				util.vector3_wide_write_slot(&closest, lane, {
					hull_test_random(&seed), hull_test_random(&seed), hull_test_random(&seed),
				});
			}
			epsilon := f32(0.001);
			if iteration % 3 == 1
			{
				epsilon = 0;
			}
			else if iteration % 3 == 2
			{
				epsilon = 1e-7;
			}
			actual_normal, actual_index := physics.hull_pair_pick_representative_face_shared_bundle_wide(
				&hull, normals, closest, util.F32x8(epsilon),
			);
			for lane in 0 ..< 8
			{
				normal := util.vector3_wide_read_slot(normals, lane);
				point := util.vector3_wide_read_slot(closest, lane);
				selected := util.vector3_wide_read_slot(planes.normal, 0);
				best_dot := util.vector3_dot(selected, normal);
				best_error := abs(util.vector3_dot(selected, point) - simd.extract(planes.offset, 0));
				best_index := 0;
				for face in 1 ..< face_count
				{
					candidate := util.vector3_wide_read_slot(planes.normal, face);
					dot := util.vector3_dot(candidate, normal);
					error := abs(util.vector3_dot(candidate, point) - simd.extract(planes.offset, face));
					improvement := best_error - error;
					if improvement >= epsilon || improvement > -epsilon && dot > best_dot
					{
						selected, best_index, best_dot, best_error = candidate, face, dot, error;
					}
				}
				testing.expect_value(t, simd.extract(actual_index, lane), i32(best_index));
				actual := util.vector3_wide_read_slot(actual_normal, lane);
				testing.expect_value(t, transmute([3]u32)actual, transmute([3]u32)selected);
			}
		}
	}
}

@(test)
depth_refiner_full_and_mixed_slot_masks_preserve_values :: proc(t: ^testing.T)
{
	for mask in 0 ..< 256
	{
		for terminated_pattern in 0 ..< 4
		{
			exists, terminated: util.I32x8;
			for lane in 0 ..< 8
			{
				exists = simd.replace(exists, lane, -i32((mask >> u32(lane)) & 1));
				terminated = simd.replace(terminated, lane, -i32(((terminated_pattern * 85) >> u32(lane)) & 1));
			}
			vertex := physics.Depth_Refiner_Vertex_With_Witness_Wide{
				support={x=util.F32x8(1), y=util.F32x8(2), z=util.F32x8(3)},
				support_on_a={x=util.F32x8(4), y=util.F32x8(5), z=util.F32x8(6)},
				weight=util.F32x8(0.75), exists=exists,
			};
			support := util.vector3_wide_negate(vertex.support);
			witness := util.vector3_wide_negate(vertex.support_on_a);
			expected := vertex;
			dont_fill := exists | terminated;
			expected.support = util.vector3_wide_select(dont_fill, expected.support, support);
			expected.support_on_a = util.vector3_wide_select(dont_fill, expected.support_on_a, witness);
			expected.exists = physics.depth_refiner_select_i32(dont_fill, expected.exists, util.I32x8(-1));
			physics.depth_refiner_fill_slot_wide(&vertex, support, witness, terminated);
			for lane in 0 ..< 8
			{
				testing.expect_value(t, util.vector3_wide_read_slot(vertex.support, lane), util.vector3_wide_read_slot(expected.support, lane));
				testing.expect_value(t, util.vector3_wide_read_slot(vertex.support_on_a, lane), util.vector3_wide_read_slot(expected.support_on_a, lane));
				testing.expect_value(t, simd.extract(vertex.exists, lane), simd.extract(expected.exists, lane));
				testing.expect_value(t, simd.extract(vertex.weight, lane), simd.extract(expected.weight, lane));
			}
		}
	}
}
