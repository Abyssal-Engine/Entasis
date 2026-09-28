package collision_pairs_tests

import "core:mem"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Collision_Shape :: struct
{
	radius: f32,
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 20
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 131072, power), util.Memory_Status.Ok);
	}
}

cube_hull_data :: proc() -> (points: [8]util.Vector3, face_starts: [6]i32, face_indices: [24]i32)
{
	points = {
		{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
	};
	face_starts = {0, 4, 8, 12, 16, 20};
	face_indices = {
		0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3,
		1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2,
	};
	return;
}

custom_collision_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: physics.Rigid_Pose,
	speculative_margin: f32, shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	_ = shapes;
	a := physics.Sphere{(^Custom_Collision_Shape)(shape_a).radius};
	b := physics.Sphere{(^Custom_Collision_Shape)(shape_b).radius};
	return physics.sphere_pair_test(&a, &b, pose_a, pose_b, speculative_margin, nil);
}

@(test)
manifold_layout_continuation_and_dense_registry_contracts_match_the_source :: proc(t: ^testing.T)
{
	testing.expect_value(t, size_of(physics.Contact), 32);
	testing.expect_value(t, size_of(physics.Convex_Contact), 20);
	testing.expect_value(t, size_of(physics.Nonconvex_Contact_Manifold), 144);
	testing.expect_value(t, size_of(physics.Convex_Contact_Manifold), 108);
	testing.expect_value(t, size_of(physics.Convex_1_Contact_Manifold_Wide), 288);
	testing.expect_value(t, size_of(physics.Convex_2_Contact_Manifold_Wide), 480);
	testing.expect_value(t, size_of(physics.Convex_4_Contact_Manifold_Wide), 864);
	continuation, status := physics.ccd_continuation_create(1, 123456);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ccd_continuation_state(continuation), physics.Continuation_State.Present);
	testing.expect_value(t, physics.ccd_continuation_type(continuation), i32(1));
	testing.expect_value(t, physics.ccd_continuation_index(continuation), i32(123456));
	testing.expect_value(t, physics.ccd_continuation_state({}), physics.Continuation_State.Missing);

	registry: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&registry), physics.Physics_Status.Ok);
	testing.expect_value(t, registry.task_count, physics.BUILT_IN_COLLISION_TASK_COUNT);
	expected_convex_batch_sizes := [6][6]i16{
		{32, 32, 32, 32, 32, 16},
		{0, 32, 32, 32, 32, 16},
		{0, 0, 32, 32, 16, 16},
		{0, 0, 0, 32, 16, 16},
		{0, 0, 0, 0, 16, 16},
		{0, 0, 0, 0, 0, 16},
	};
	for type_a in 0 ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
	{
		for type_b in type_a ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
		{
			task, forward, lookup_status := physics.collision_task_registry_lookup(&registry, type_a, type_b);
			_, reverse, reverse_status := physics.collision_task_registry_lookup(&registry, type_b, type_a);
			testing.expect_value(t, lookup_status, physics.Physics_Status.Ok);
			testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
			testing.expect(t, task != nil);
			testing.expect_value(t, forward.task_id, reverse.task_id);
			if type_b <= physics.CONVEX_HULL_TYPE_ID
			{
				testing.expect_value(t, task.batch_size, expected_convex_batch_sizes[type_a][type_b]);
			}
			else
			{
				testing.expect_value(t, task.batch_size, i16(16));
			}
		}
	}
}

@(test)
wide_manifolds_preserve_source_contact_order_flip_and_inactive_tail_masks :: proc(t: ^testing.T)
{
	offset_b: util.Vector3_Wide;
	util.vector3_wide_write_slot(&offset_b, 0, {2, 3, 4});
	flip_mask := util.I32x8(0);
	flip_mask = simd.replace(flip_mask, 0, -1);

	one: physics.Convex_1_Contact_Manifold_Wide;
	util.vector3_wide_write_slot(&one.offset_a, 0, {1, 2, 3});
	util.vector3_wide_write_slot(&one.normal, 0, {0, 1, 0});
	one.depth = simd.replace(one.depth, 0, 0.25);
	one.feature_id = simd.replace(one.feature_id, 0, 17);
	one.contact_exists = simd.replace(one.contact_exists, 0, -1);
	one_offset_b := offset_b;
	testing.expect_value(
		t,
		physics.convex_1_manifold_wide_apply_flip_mask(&one, &one_offset_b, flip_mask),
		physics.Physics_Status.Ok
	);
	one_lane, one_status := physics.convex_1_manifold_wide_read_lane(&one, one_offset_b, 0);
	testing.expect_value(t, one_status, physics.Physics_Status.Ok);
	testing.expect_value(t, one_lane.count, i32(1));
	testing.expect_value(t, one_lane.offset_b, util.Vector3{-2, -3, -4});
	testing.expect_value(t, one_lane.normal, util.Vector3{0, -1, 0});
	testing.expect_value(t, one_lane.contacts[0].offset, util.Vector3{-1, -1, -1});
	testing.expect_value(t, one_lane.contacts[0].feature_id, i32(17));

	two: physics.Convex_2_Contact_Manifold_Wide;
	util.vector3_wide_write_slot(&two.offset_a_0, 0, {1, 0, 0});
	util.vector3_wide_write_slot(&two.offset_a_1, 0, {2, 0, 0});
	util.vector3_wide_write_slot(&two.normal, 0, {1, 0, 0});
	two.depth_0 = simd.replace(two.depth_0, 0, 0.5);
	two.depth_1 = simd.replace(two.depth_1, 0, 0.25);
	two.feature_id_0 = simd.replace(two.feature_id_0, 0, 3);
	two.feature_id_1 = simd.replace(two.feature_id_1, 0, 4);
	two.contact_0_exists = simd.replace(two.contact_0_exists, 0, -1);
	two.contact_1_exists = simd.replace(two.contact_1_exists, 0, -1);
	two_offset_b := offset_b;
	testing.expect_value(
		t,
		physics.convex_2_manifold_wide_apply_flip_mask(&two, &two_offset_b, flip_mask),
		physics.Physics_Status.Ok
	);
	two_lane, two_status := physics.convex_2_manifold_wide_read_lane(&two, two_offset_b, 0);
	testing.expect_value(t, two_status, physics.Physics_Status.Ok);
	testing.expect_value(t, two_lane.count, i32(2));
	testing.expect_value(t, two_lane.contacts[0].feature_id, i32(3));
	testing.expect_value(t, two_lane.contacts[1].feature_id, i32(4));
	testing.expect_value(t, two_lane.contacts[0].offset, util.Vector3{-1, -3, -4});
	testing.expect_value(t, two_lane.contacts[1].offset, util.Vector3{0, -3, -4});

	four: physics.Convex_4_Contact_Manifold_Wide;
	util.vector3_wide_write_slot(&four.offset_a_0, 0, {0, 0, 0});
	util.vector3_wide_write_slot(&four.offset_a_1, 0, {1, 0, 0});
	util.vector3_wide_write_slot(&four.offset_a_2, 0, {2, 0, 0});
	util.vector3_wide_write_slot(&four.offset_a_3, 0, {3, 0, 0});
	util.vector3_wide_write_slot(&four.normal, 0, {0, 0, 1});
	four.feature_id_0 = simd.replace(four.feature_id_0, 0, 10);
	four.feature_id_1 = simd.replace(four.feature_id_1, 0, 11);
	four.feature_id_2 = simd.replace(four.feature_id_2, 0, 12);
	four.feature_id_3 = simd.replace(four.feature_id_3, 0, 13);
	four.contact_0_exists = simd.replace(four.contact_0_exists, 0, -1);
	four.contact_1_exists = simd.replace(four.contact_1_exists, 0, -1);
	four.contact_2_exists = simd.replace(four.contact_2_exists, 0, -1);
	four.contact_3_exists = simd.replace(four.contact_3_exists, 0, -1);
	four.contact_0_exists = simd.replace(four.contact_0_exists, 7, -1);
	testing.expect_value(t, physics.convex_manifold_wide_clear_tail(&four, 1), physics.Physics_Status.Ok);
	four_lane, four_status := physics.convex_4_manifold_wide_read_lane(&four, offset_b, 0);
	testing.expect_value(t, four_status, physics.Physics_Status.Ok);
	testing.expect_value(t, four_lane.count, i32(4));
	for contact_index in 0 ..< 4
	{
		testing.expect_value(t, four_lane.contacts[contact_index].feature_id, i32(10 + contact_index));
	}
	tail_lane, tail_status := physics.convex_4_manifold_wide_read_lane(&four, offset_b, 7);
	testing.expect_value(t, tail_status, physics.Physics_Status.Ok);
	testing.expect_value(t, tail_lane.count, i32(0));
}

@(test)
sphere_pair_family_wide_kernels_match_scalar_source_routes_and_mask_tail_lanes :: proc(t: ^testing.T)
{
	sphere_wide: physics.Sphere_Wide;
	sphere_b_wide: physics.Sphere_Wide;
	capsule_wide: physics.Capsule_Wide;
	box_wide: physics.Box_Wide;
	triangle_wide: physics.Triangle_Wide;
	cylinder_wide: physics.Cylinder_Wide;
	offset_b_wide: util.Vector3_Wide;
	orientation_b_wide: util.Quaternion_Wide;
	speculative_margin := util.F32x8(0.1);
	sphere := physics.Sphere{0.75};
	sphere_b := physics.Sphere{1.25};
	capsule := physics.Capsule{0.5, 1};
	box := physics.Box{1, 0.75, 1.25};
	triangle := physics.Triangle{{-1, 0, -1}, {0, 0, 1}, {1, 0, -1}};
	cylinder := physics.Cylinder{1, 0.75};
	identity := physics.rigid_pose_identity();
	for lane in 0 ..< 5
	{
		testing.expect_value(t, physics.sphere_wide_write_slot(&sphere_wide, lane, sphere), physics.Physics_Status.Ok);
		testing.expect_value(
			t,
			physics.sphere_wide_write_slot(&sphere_b_wide, lane, sphere_b),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.capsule_wide_write_slot(&capsule_wide, lane, capsule),
			physics.Physics_Status.Ok
		);
		testing.expect_value(t, physics.box_wide_write_slot(&box_wide, lane, box), physics.Physics_Status.Ok);
		testing.expect_value(
			t,
			physics.triangle_wide_write_slot(&triangle_wide, lane, triangle),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.cylinder_wide_write_slot(&cylinder_wide, lane, cylinder),
			physics.Physics_Status.Ok
		);
		util.vector3_wide_write_slot(&offset_b_wide, lane, {0.25 + f32(lane) * 0.2, -0.4, 0.15});
		util.quaternion_wide_write_slot(&orientation_b_wide, lane, identity.orientation);
	}

	sphere_pair_wide, sphere_pair_status := physics.sphere_pair_test_wide(
		sphere_wide, sphere_b_wide, speculative_margin, offset_b_wide, 5,
	);
	sphere_capsule_wide, sphere_capsule_status := physics.sphere_capsule_test_wide(
		sphere_wide, capsule_wide, speculative_margin, offset_b_wide, orientation_b_wide, 5,
	);
	sphere_box_wide, sphere_box_status := physics.sphere_box_test_wide(
		sphere_wide, box_wide, speculative_margin, offset_b_wide, orientation_b_wide, 5,
	);
	sphere_triangle_wide, sphere_triangle_status := physics.sphere_triangle_test_wide(
		sphere_wide, triangle_wide, speculative_margin, offset_b_wide, orientation_b_wide, 5,
	);
	sphere_cylinder_wide, sphere_cylinder_status := physics.sphere_cylinder_test_wide(
		sphere_wide, cylinder_wide, speculative_margin, offset_b_wide, orientation_b_wide, 5,
	);
	testing.expect_value(t, sphere_pair_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_capsule_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_box_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_triangle_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_cylinder_status, physics.Physics_Status.Ok);

	for lane in 0 ..< 5
	{
		pose_b := identity;
		pose_b.position = util.vector3_wide_read_slot(offset_b_wide, lane);
		wide_manifolds := [5]^physics.Convex_1_Contact_Manifold_Wide{
			&sphere_pair_wide, &sphere_capsule_wide, &sphere_box_wide, &sphere_triangle_wide, &sphere_cylinder_wide,
		};
		scalar_manifolds := [5]physics.Convex_Contact_Manifold{};
		scalar_manifolds[0], _ = physics.sphere_pair_test(&sphere, &sphere_b, identity, pose_b, 0.1, nil);
		scalar_manifolds[1], _ = physics.sphere_capsule_test(&sphere, &capsule, identity, pose_b, 0.1, nil);
		scalar_manifolds[2], _ = physics.sphere_box_test(&sphere, &box, identity, pose_b, 0.1, nil);
		scalar_manifolds[3], _ = physics.sphere_triangle_test(&sphere, &triangle, identity, pose_b, 0.1, nil);
		scalar_manifolds[4], _ = physics.sphere_cylinder_test(&sphere, &cylinder, identity, pose_b, 0.1, nil);
		for pair_index in 0 ..< len(wide_manifolds)
		{
			wide_lane, read_status := physics.convex_1_manifold_wide_read_lane(
				wide_manifolds[pair_index],
				offset_b_wide,
				lane
			);
			testing.expect_value(t, read_status, physics.Physics_Status.Ok);
			testing.expect_value(t, wide_lane.count, scalar_manifolds[pair_index].count);
			if wide_lane.count > 0
			{
				testing.expect(t, util.vector3_distance(wide_lane.normal, scalar_manifolds[pair_index].normal) < 2e-5);
				testing.expect(
					t,
					util.vector3_distance(
					wide_lane.contacts[0].offset,
					scalar_manifolds[pair_index].contacts[0].offset
				) < 2e-5
				);
				testing.expect(
					t,
					abs(wide_lane.contacts[0].depth - scalar_manifolds[pair_index].contacts[0].depth) < 2e-5
				);
				testing.expect_value(
					t,
					wide_lane.contacts[0].feature_id,
					scalar_manifolds[pair_index].contacts[0].feature_id
				);
			}
		}
	}
	tail_lane, tail_status := physics.convex_1_manifold_wide_read_lane(&sphere_pair_wide, offset_b_wide, 7);
	testing.expect_value(t, tail_status, physics.Physics_Status.Ok);
	testing.expect_value(t, tail_lane.count, i32(0));
}

@(test)
capsule_pair_wide_kernel_preserves_the_source_two_contact_interval :: proc(t: ^testing.T)
{
	a := physics.Capsule{0.5, 1.25};
	b := physics.Capsule{0.75, 1};
	a_wide: physics.Capsule_Wide;
	b_wide: physics.Capsule_Wide;
	offset_b_wide: util.Vector3_Wide;
	orientation_a_wide: util.Quaternion_Wide;
	orientation_b_wide: util.Quaternion_Wide;
	identity := physics.rigid_pose_identity();
	for lane in 0 ..< 6
	{
		testing.expect_value(t, physics.capsule_wide_write_slot(&a_wide, lane, a), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.capsule_wide_write_slot(&b_wide, lane, b), physics.Physics_Status.Ok);
		util.vector3_wide_write_slot(&offset_b_wide, lane, {0.75 + f32(lane) * 0.03, 0.1, 0});
		util.quaternion_wide_write_slot(&orientation_a_wide, lane, identity.orientation);
		orientation_b := util.quaternion_from_axis_angle({0, 0, 1}, f32(lane) * 0.008);
		util.quaternion_wide_write_slot(&orientation_b_wide, lane, orientation_b);
	}
	wide, wide_status := physics.capsule_pair_test_wide(
		a_wide, b_wide, util.F32x8(0.1), offset_b_wide, orientation_a_wide, orientation_b_wide, 6,
	);
	testing.expect_value(t, wide_status, physics.Physics_Status.Ok);
	for lane in 0 ..< 6
	{
		pose_b := identity;
		pose_b.position = util.vector3_wide_read_slot(offset_b_wide, lane);
		pose_b.orientation = util.quaternion_wide_read_slot(orientation_b_wide, lane);
		scalar, scalar_status := physics.capsule_pair_test(&a, &b, identity, pose_b, 0.1, nil);
		lane_manifold, read_status := physics.convex_2_manifold_wide_read_lane(&wide, offset_b_wide, lane);
		testing.expect_value(t, scalar_status, physics.Physics_Status.Ok);
		testing.expect_value(t, read_status, physics.Physics_Status.Ok);
		testing.expect_value(t, lane_manifold.count, scalar.count);
		for contact_index in 0 ..< int(scalar.count)
		{
			testing.expect_value(
				t,
				lane_manifold.contacts[contact_index].feature_id,
				scalar.contacts[contact_index].feature_id
			);
			testing.expect(
				t,
				abs(lane_manifold.contacts[contact_index].depth - scalar.contacts[contact_index].depth) < 2e-5
			);
			testing.expect(
				t,
				util.vector3_distance(
				lane_manifold.contacts[contact_index].offset,
				scalar.contacts[contact_index].offset
			) < 2e-5
			);
		}
	}
	tail, tail_status := physics.convex_2_manifold_wide_read_lane(&wide, offset_b_wide, 7);
	testing.expect_value(t, tail_status, physics.Physics_Status.Ok);
	testing.expect_value(t, tail.count, i32(0));
}

@(test)
all_twenty_one_specialized_convex_pairs_execute_both_orders_repeatably :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);

	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.75, 1};
	box := physics.Box{1, 1, 1};
	triangle := physics.Triangle{{-1, 0, -1}, {0, 0, 1}, {1, 0, -1}};
	cylinder := physics.Cylinder{1, 1};
	points, face_starts, face_indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	shape_data := [6]rawptr{&sphere, &capsule, &box, &triangle, &cylinder, &hull};
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {0.25, 0.2, 0.15};
	for type_a in 0 ..= physics.CONVEX_HULL_TYPE_ID
	{
		for type_b in type_a ..= physics.CONVEX_HULL_TYPE_ID
		{
			forward, forward_status := physics.collision_task_registry_test_convex(
				&tasks, type_a, type_b, shape_data[type_a], shape_data[type_b], pose_a, pose_b, 0.1, &shapes,
			);
			repeated, repeated_status := physics.collision_task_registry_test_convex(
				&tasks, type_a, type_b, shape_data[type_a], shape_data[type_b], pose_a, pose_b, 0.1, &shapes,
			);
			reverse, reverse_status := physics.collision_task_registry_test_convex(
				&tasks, type_b, type_a, shape_data[type_b], shape_data[type_a], pose_b, pose_a, 0.1, &shapes,
			);
			testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
			testing.expect_value(t, repeated_status, physics.Physics_Status.Ok);
			testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
			testing.expect_value(t, forward, repeated);
			testing.expect_value(t, forward.count, reverse.count);
			if forward.count > 0
			{
				testing.expect(t, util.vector3_dot(forward.normal, reverse.normal) < -0.99);
				for contact_index in 0 ..< int(forward.count)
				{
					testing.expect(t, forward.contacts[contact_index].depth >= -0.1);
				}
			}

			// centered volumes overlap along X, clockwise triangle fronts point along -Y
			known_a: physics.Rigid_Pose = physics.rigid_pose_identity();
			known_b: physics.Rigid_Pose = physics.rigid_pose_identity();
			known_b.position = {1, 0, 0};
			expected_normal: util.Vector3 = {-1, 0, 0};
			if type_a == physics.TRIANGLE_TYPE_ID && type_b == physics.TRIANGLE_TYPE_ID
			{
				// transverse triangles intersect through their interiors on their front sides
				known_b.position = {-0.2, -0.2, 0};
				known_b.orientation = util.quaternion_from_axis_angle({0, 0, 1}, 1.57079633);
				expected_normal = {};
			}
			else if type_a == physics.TRIANGLE_TYPE_ID
			{
				known_b.position = {0, -0.75, 0};
				expected_normal = {0, 1, 0};
			}
			else if type_b == physics.TRIANGLE_TYPE_ID
			{
				known_a.position = {0, -1.5 if type_a == physics.CAPSULE_TYPE_ID else -0.75, 0};
				known_b.position = {};
				expected_normal = {0, -1, 0};
			}
			for separation in ([2]f32{0, 20})
			{
				separated_b: physics.Rigid_Pose = known_b;
				separated_b.position.x += separation;
				for order in 0 ..< 2
				{
					a: int = type_a if order == 0 else type_b;
					b: int = type_b if order == 0 else type_a;
					first: physics.Rigid_Pose = known_a if order == 0 else separated_b;
					second: physics.Rigid_Pose = separated_b if order == 0 else known_a;
					manifold: physics.Convex_Contact_Manifold;
					status: physics.Physics_Status;
					manifold, status = physics.collision_task_registry_test_convex(
						&tasks, a, b, shape_data[a], shape_data[b], first, second, 0.1, &shapes,
					);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					if separation == 0
					{
						testing.expectf(t, manifold.count > 0, "pair %d/%d: expected contact", a, b);
						testing.expect(t, manifold.count <= i32(len(manifold.contacts)));
						testing.expect(t, abs(util.vector3_dot(manifold.normal, manifold.normal)-1) < 1e-3);
						if expected_normal != (util.Vector3{})
						{
							direction: f32 = 1 if order == 0 else -1;
							testing.expectf(t, util.vector3_dot(manifold.normal, expected_normal)*direction > 0.99,
								"pair %d/%d: normal %v", a, b, manifold.normal);
						}
						for contact in manifold.contacts[:manifold.count]
						{
							testing.expect(t, contact.depth >= -0.1 && contact.depth < 4);
							for component in ([3]f32{contact.offset.x, contact.offset.y, contact.offset.z})
							{
								testing.expect(t, abs(component) < 10);
							}
						}
					}
					else
					{
						testing.expect_value(t, manifold.count, i32(0));
					}
				}
			}
		}
	}
}

@(test)
source_boundary_contacts_preserve_count_order_features_normals_and_margins :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	identity := physics.rigid_pose_identity();
	offset := identity;
	offset.position = {1.5, 0, 0};
	sphere := physics.Sphere{1};
	manifold, status := physics.collision_task_registry_test_convex(
		&tasks, 0, 0, &sphere, &sphere, identity, offset, 0, &shapes,
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, manifold.count, i32(1));
	testing.expect_value(t, manifold.normal, util.Vector3{-1, 0, 0});
	testing.expect_value(t, manifold.contacts[0].depth, f32(0.5));
	testing.expect_value(t, manifold.contacts[0].feature_id, i32(0));

	separated := offset;
	separated.position = {2.05, 0, 0};
	without_margin, _ := physics.collision_task_registry_test_convex(
		&tasks, 0, 0, &sphere, &sphere, identity, separated, 0, &shapes,
	);
	with_margin, _ := physics.collision_task_registry_test_convex(
		&tasks, 0, 0, &sphere, &sphere, identity, separated, 0.1, &shapes,
	);
	testing.expect_value(t, without_margin.count, i32(0));
	testing.expect_value(t, with_margin.count, i32(1));
	testing.expect(t, abs(with_margin.contacts[0].depth + 0.05) < 1e-5);

	box := physics.Box{1, 1, 1};
	box_manifold, box_status := physics.box_pair_test(&box, &box, identity, offset, 0, &shapes);
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_manifold.count, i32(4));
	expected_box_feature_ids := [4]i32{123, 91, 103, 111};
	for contact_index in 0 ..< 4
	{
		testing.expect_value(
			t,
			box_manifold.contacts[contact_index].feature_id,
			expected_box_feature_ids[contact_index]
		);
		testing.expect_value(t, box_manifold.contacts[contact_index].depth, f32(0.5));
	}
}

@(test)
box_wide_candidate_compaction_preserves_mixed_lane_order_and_features :: proc(t: ^testing.T)
{
	feature_bases := [8]i32{-1, -5, -17, -21, 64, 65, 123, 91};
	for pair_count in 1 ..= util.PRODUCTION_LANE_COUNT
	{
		candidates: [8]physics.Manifold_Candidate_Wide;
		candidate_count := util.I32x8(0);
		initialized_candidate_count := 0;
		for candidate_index in 0 ..< len(candidates)
		{
			candidate: physics.Manifold_Candidate_Wide;
			exists := util.I32x8(0);
			for lane in 0 ..< util.PRODUCTION_LANE_COUNT
			{
				candidate.x = simd.replace(
					candidate.x, lane, f32(candidate_index * 100 + lane) + 0.125,
				);
				candidate.y = simd.replace(
					candidate.y, lane, -f32(candidate_index * 100 + lane) - 0.25,
				);
				candidate.depth = simd.replace(
					candidate.depth, lane, f32(candidate_index * 10 + lane) + 0.5,
				);
				candidate.feature_id = simd.replace(
					candidate.feature_id, lane, feature_bases[candidate_index] + i32(lane * 256),
				);
				expected_count := (lane + pair_count) % 9;
				if lane < pair_count && candidate_index < expected_count
				{
					exists = simd.replace(exists, lane, -1);
				}
			}
			physics.collision_box_pair_add_candidate_wide(
				&candidates, &candidate_count, &initialized_candidate_count,
				candidate, exists, pair_count,
			);
		}
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			expected_count := 0;
			if lane < pair_count
			{
				expected_count = (lane + pair_count) % 9;
			}
			testing.expect_value(t, simd.extract(candidate_count, lane), i32(expected_count));
			for slot in 0 ..< expected_count
			{
				testing.expect_value(
					t, simd.extract(candidates[slot].x, lane), f32(slot * 100 + lane) + 0.125,
				);
				testing.expect_value(
					t, simd.extract(candidates[slot].y, lane), -f32(slot * 100 + lane) - 0.25,
				);
				testing.expect_value(
					t, simd.extract(candidates[slot].depth, lane), f32(slot * 10 + lane) + 0.5,
				);
				testing.expect_value(
					t, simd.extract(candidates[slot].feature_id, lane),
					feature_bases[slot] + i32(lane * 256),
				);
			}
		}
	}

	boxes_a: [util.PRODUCTION_LANE_COUNT]physics.Box;
	boxes_b: [util.PRODUCTION_LANE_COUNT]physics.Box;
	poses_a: [util.PRODUCTION_LANE_COUNT]physics.Rigid_Pose;
	poses_b: [util.PRODUCTION_LANE_COUNT]physics.Rigid_Pose;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		boxes_a[lane] = {1, 1, 1};
		boxes_b[lane] = {1 - f32(lane) * 0.025, 0.8 + f32(lane) * 0.02, 1.1};
		poses_a[lane] = physics.rigid_pose_identity();
		poses_b[lane] = physics.rigid_pose_identity();
	}
	poses_b[0].position = {1.5, 0, 0};
	poses_b[1].position = {3, 0, 0};
	poses_b[2].position = {1.65, 0.85, 0};
	poses_b[2].orientation = util.quaternion_from_axis_angle({0, 0, 1}, 0.7853982);
	poses_b[3].position = {1.45, 1.35, 1.25};
	poses_b[3].orientation = util.quaternion_from_axis_angle({0.57735026, 0.57735026, 0.57735026}, 0.6154797);
	poses_b[4].position = {0, 1.55, 0.2};
	poses_a[4].orientation = util.quaternion_from_axis_angle({0, 1, 0}, 0.2);
	poses_b[5].position = {0.9, 1.55, 0.75};
	poses_b[5].orientation = util.quaternion_from_axis_angle({1, 0, 0}, 0.7853982);
	poses_b[6].position = {1.35, 1.2, 1.45};
	poses_b[6].orientation = util.quaternion_from_axis_angle({0, 1, 0}, 0.7853982);
	poses_b[7].position = {-1.4, 0.35, 1.1};
	poses_b[7].orientation = util.quaternion_from_axis_angle({0, 0, 1}, -0.45);
	vertex_case_state := physics.Reference_State.Missing;
	vertex_seed := u32(0x6a09_e667);
	for _ in 0 ..< 512
	{
		vertex_seed ~= vertex_seed << 13;
		vertex_seed ~= vertex_seed >> 17;
		vertex_seed ~= vertex_seed << 5;
		vertex_pose := physics.rigid_pose_identity();
		vertex_pose.position = {
			0.6 + f32(vertex_seed & 0xff) / 255.0 * 1.8,
			0.6 + f32((vertex_seed >> 8) & 0xff) / 255.0 * 1.8,
			0.6 + f32((vertex_seed >> 16) & 0xff) / 255.0 * 1.8,
		};
		vertex_pose.orientation = util.quaternion_from_axis_angle(
			{0.26726124, 0.5345225, 0.8017837},
			0.1 + f32((vertex_seed >> 24) & 0xff) / 255.0 * 1.3,
		);
		vertex_manifold, vertex_status := physics.box_pair_test_source(
			boxes_a[3], boxes_b[3], poses_a[3], vertex_pose, 0.1,
		);
		testing.expect_value(t, vertex_status, physics.Physics_Status.Ok);
		if vertex_manifold.count == 1
		{
			poses_b[3] = vertex_pose;
			vertex_case_state = .Present;
			break;
		}
	}
	testing.expect_value(t, vertex_case_state, physics.Reference_State.Present);
	three_contact_state := physics.Reference_State.Missing;
	three_contact_seed := u32(0xbb67_ae85);
	three_contact_pose := physics.rigid_pose_identity();
	three_contact_expected: physics.Convex_Contact_Manifold;
	for _ in 0 ..< 2048
	{
		three_contact_seed ~= three_contact_seed << 13;
		three_contact_seed ~= three_contact_seed >> 17;
		three_contact_seed ~= three_contact_seed << 5;
		candidate_pose := physics.rigid_pose_identity();
		candidate_pose.position = {
			0.4 + f32(three_contact_seed & 0xff) / 255.0 * 1.8,
			0.4 + f32((three_contact_seed >> 8) & 0xff) / 255.0 * 1.8,
			0.4 + f32((three_contact_seed >> 16) & 0xff) / 255.0 * 1.8,
		};
		candidate_pose.orientation = util.quaternion_from_axis_angle(
			{0.37139067, 0.55708605, 0.74278134},
			0.05 + f32((three_contact_seed >> 24) & 0xff) / 255.0 * 1.45,
		);
		three_contact_manifold, three_contact_status := physics.box_pair_test_source(
			boxes_a[6], boxes_b[6], poses_a[6], candidate_pose, 0.1,
		);
		testing.expect_value(t, three_contact_status, physics.Physics_Status.Ok);
		if three_contact_manifold.count == 3
		{
			three_contact_pose = candidate_pose;
			three_contact_expected = three_contact_manifold;
			three_contact_state = .Present;
			break;
		}
	}
	testing.expect_value(t, three_contact_state, physics.Reference_State.Present);

	contact_count_seen: [physics.MAXIMUM_MANIFOLD_CONTACT_COUNT + 1]physics.Reference_State;
	for pair_count in 1 ..= util.PRODUCTION_LANE_COUNT
	{
		a_wide, b_wide: physics.Box_Wide;
		offset_b: util.Vector3_Wide;
		orientation_a, orientation_b: util.Quaternion_Wide;
		for lane in 0 ..< pair_count
		{
			testing.expect_value(
				t,
				physics.box_wide_write_slot(&a_wide, lane, boxes_a[lane]),
				physics.Physics_Status.Ok
			);
			testing.expect_value(
				t,
				physics.box_wide_write_slot(&b_wide, lane, boxes_b[lane]),
				physics.Physics_Status.Ok
			);
			util.vector3_wide_write_slot(
				&offset_b, lane, util.vector3_subtract(poses_b[lane].position, poses_a[lane].position),
			);
			util.quaternion_wide_write_slot(&orientation_a, lane, poses_a[lane].orientation);
			util.quaternion_wide_write_slot(&orientation_b, lane, poses_b[lane].orientation);
		}
		wide, wide_status := physics.box_pair_test_wide(
			a_wide, b_wide, util.F32x8(0.1), offset_b,
			orientation_a, orientation_b, pair_count,
		);
		testing.expect_value(t, wide_status, physics.Physics_Status.Ok);
		for lane in 0 ..< pair_count
		{
			expected, expected_status := physics.box_pair_test_source(
				boxes_a[lane], boxes_b[lane], poses_a[lane], poses_b[lane], 0.1,
			);
			actual, actual_status := physics.convex_4_manifold_wide_read_lane(&wide, offset_b, lane);
			testing.expect_value(t, expected_status, physics.Physics_Status.Ok);
			testing.expect_value(t, actual_status, physics.Physics_Status.Ok);
			testing.expect_value(t, actual.count, expected.count);
			if actual.count > 0
			{
				contact_count_seen[actual.count] = .Present;
				testing.expect_value(t, actual.normal, expected.normal);
				testing.expect_value(t, actual.offset_b, expected.offset_b);
				for contact_index in 0 ..< int(actual.count)
				{
					testing.expect_value(
						t, actual.contacts[contact_index], expected.contacts[contact_index],
					);
				}
			}
			else
			{
				contact_count_seen[0] = .Present;
			}
		}
		for lane in pair_count ..< util.PRODUCTION_LANE_COUNT
		{
			tail, tail_status := physics.convex_4_manifold_wide_read_lane(&wide, offset_b, lane);
			testing.expect_value(t, tail_status, physics.Physics_Status.Ok);
			testing.expect_value(t, tail.count, i32(0));
		}
	}
	three_a_wide, three_b_wide: physics.Box_Wide;
	three_offset_wide: util.Vector3_Wide;
	three_orientation_a_wide, three_orientation_b_wide: util.Quaternion_Wide;
	physics.box_wide_write_slot_trusted(&three_a_wide, 0, boxes_a[6]);
	physics.box_wide_write_slot_trusted(&three_b_wide, 0, boxes_b[6]);
	util.vector3_wide_write_slot(
		&three_offset_wide, 0,
		util.vector3_subtract(three_contact_pose.position, poses_a[6].position),
	);
	util.quaternion_wide_write_slot(
		&three_orientation_a_wide, 0, poses_a[6].orientation,
	);
	util.quaternion_wide_write_slot(
		&three_orientation_b_wide, 0, three_contact_pose.orientation,
	);
	three_wide, three_wide_status := physics.box_pair_test_wide(
		three_a_wide, three_b_wide, util.F32x8(0.1), three_offset_wide,
		three_orientation_a_wide, three_orientation_b_wide, 1,
	);
	three_actual, three_actual_status := physics.convex_4_manifold_wide_read_lane(
		&three_wide, three_offset_wide, 0,
	);
	testing.expect_value(t, three_wide_status, physics.Physics_Status.Ok);
	testing.expect_value(t, three_actual_status, physics.Physics_Status.Ok);
	testing.expect_value(t, three_actual, three_contact_expected);
	contact_count_seen[3] = .Present;
	benchmark_dimensions := [4]physics.Box{
		{14.5, 0.5, 14.5},
		{0.5, 12.25, 14.5},
		{14.5, 12.25, 0.5},
		{0.5, 0.5, 0.5},
	};
	for dimension_index in 0 ..< len(benchmark_dimensions)
	{
		benchmark_box := benchmark_dimensions[dimension_index];
		benchmark_pose_a := physics.rigid_pose_identity();
		benchmark_pose_b := physics.rigid_pose_identity();
		benchmark_pose_b.position = {
			benchmark_box.half_width * 0.75,
			benchmark_box.half_height * 0.25,
			benchmark_box.half_length * -0.5,
		};
		benchmark_pose_b.orientation = util.quaternion_from_axis_angle(
			{0.26726124, 0.5345225, 0.8017837}, 0.17,
		);
		expected, expected_status := physics.box_pair_test_source(
			benchmark_box, benchmark_box, benchmark_pose_a, benchmark_pose_b, 0.1,
		);
		a_wide, b_wide: physics.Box_Wide;
		offset_wide: util.Vector3_Wide;
		orientation_a_wide, orientation_b_wide: util.Quaternion_Wide;
		physics.box_wide_write_slot_trusted(&a_wide, 0, benchmark_box);
		physics.box_wide_write_slot_trusted(&b_wide, 0, benchmark_box);
		util.vector3_wide_write_slot(&offset_wide, 0, benchmark_pose_b.position);
		util.quaternion_wide_write_slot(
			&orientation_a_wide, 0, benchmark_pose_a.orientation,
		);
		util.quaternion_wide_write_slot(
			&orientation_b_wide, 0, benchmark_pose_b.orientation,
		);
		wide, wide_status := physics.box_pair_test_wide(
			a_wide, b_wide, util.F32x8(0.1), offset_wide,
			orientation_a_wide, orientation_b_wide, 1,
		);
		actual, actual_status := physics.convex_4_manifold_wide_read_lane(
			&wide, offset_wide, 0,
		);
		testing.expect_value(t, expected_status, physics.Physics_Status.Ok);
		testing.expect_value(t, wide_status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual_status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual.count, expected.count);
		if actual.count > 0
		{
			testing.expect_value(t, actual.offset_b, expected.offset_b);
			testing.expect_value(t, actual.normal, expected.normal);
			for contact_index in 0 ..< int(actual.count)
			{
				testing.expect_value(
					t, actual.contacts[contact_index], expected.contacts[contact_index],
				);
			}
		}
	}
	testing.expect_value(t, contact_count_seen[0], physics.Reference_State.Present);
	testing.expect_value(t, contact_count_seen[1], physics.Reference_State.Present);
	testing.expect_value(t, contact_count_seen[2], physics.Reference_State.Present);
	testing.expect_value(t, contact_count_seen[3], physics.Reference_State.Present);
	testing.expect_value(t, contact_count_seen[4], physics.Reference_State.Present);
}

@(test)
caller_registered_collision_task_uses_the_same_dense_public_route :: proc(t: ^testing.T)
{
	registry: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&registry), physics.Physics_Status.Ok);
	custom_type := physics.BUILT_IN_SHAPE_TYPE_COUNT;
	_, register_status := physics.collision_task_registry_register(&registry, {
			shape_type_a=i16(custom_type),
			shape_type_b=i16(custom_type),
			batch_size=8,
			pair_type=.Flipless,
			kind=.Convex,
			capabilities={.Convex_Result},
			convex_test=custom_collision_test,
		});
	testing.expect_value(t, register_status, physics.Physics_Status.Ok);
	_, duplicate_status := physics.collision_task_registry_register(&registry, {
			shape_type_a=i16(custom_type), shape_type_b=i16(custom_type), batch_size=8,
			pair_type=.Flipless, kind=.Convex, capabilities={.Convex_Result}, convex_test=custom_collision_test,
		});
	testing.expect_value(t, duplicate_status, physics.Physics_Status.Invalid_Description);
}

@(test)
randomized_sphere_pairs_match_the_source_distance_invariant_in_both_orders :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere_a := physics.Sphere{0.75};
	sphere_b := physics.Sphere{1.25};
	seed := u32(0x1234_5678);
	for _ in 0 ..< 256
	{
		seed ~= seed << 13;
		seed ~= seed >> 17;
		seed ~= seed << 5;
		x := (f32(seed & 0xffff) / 65535.0) * 6 - 3;
		pose_a := physics.rigid_pose_identity();
		pose_b := physics.rigid_pose_identity();
		pose_b.position = {x, f32(i32(seed >> 16) % 7) * 0.1, -0.2};
		forward, forward_status := physics.collision_task_registry_test_convex(
			&tasks, 0, 0, &sphere_a, &sphere_b, pose_a, pose_b, 0.15, &shapes,
		);
		reverse, reverse_status := physics.collision_task_registry_test_convex(
			&tasks, 0, 0, &sphere_b, &sphere_a, pose_b, pose_a, 0.15, &shapes,
		);
		testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
		testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
		distance := util.vector3_length(pose_b.position);
		expected_depth := sphere_a.radius + sphere_b.radius - distance;
		expected_count := i32(0);
		if expected_depth >= -0.15
		{
			expected_count = 1;
		}
		testing.expect_value(t, forward.count, expected_count);
		testing.expect_value(t, reverse.count, expected_count);
		if expected_count == 1
		{
			testing.expect(t, abs(forward.contacts[0].depth - expected_depth) < 1e-5);
			testing.expect_value(t, forward.contacts[0].depth, reverse.contacts[0].depth);
			testing.expect(t, util.vector3_dot(forward.normal, reverse.normal) < -0.999);
		}
	}
}

@(test)
randomized_complete_convex_matrix_preserves_repeat_and_reverse_invariants :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.75, 1};
	box := physics.Box{1, 0.8, 1.2};
	triangle := physics.Triangle{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}};
	cylinder := physics.Cylinder{1, 1};
	points, face_starts, face_indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	shape_data := [6]rawptr{&sphere, &capsule, &box, &triangle, &cylinder, &hull};
	seed := u32(0x9e37_79b9);
	for sample in 0 ..< 24
	{
		seed ~= seed << 13;
		seed ~= seed >> 17;
		seed ~= seed << 5;
		pose_a := physics.rigid_pose_identity();
		pose_b := physics.rigid_pose_identity();
		pose_a.orientation = util.quaternion_from_axis_angle({0, 1, 0}, f32(sample)*0.013);
		pose_b.orientation = util.quaternion_from_axis_angle({0.3, 0.7, 0.2}, f32(sample)*0.021);
		pose_b.position = {
			(f32(seed & 0xff)/255.0)*2.4-1.2,
			(f32((seed>>8) & 0xff)/255.0)*1.2+0.05,
			(f32((seed>>16) & 0xff)/255.0)*2.4-1.2,
		};
		for type_a in 0 ..= physics.CONVEX_HULL_TYPE_ID
		{
			for type_b in type_a ..= physics.CONVEX_HULL_TYPE_ID
			{
				forward, forward_status := physics.collision_task_registry_test_convex(
					&tasks, type_a, type_b, shape_data[type_a], shape_data[type_b], pose_a, pose_b, 0.15, &shapes,
				);
				repeated, repeated_status := physics.collision_task_registry_test_convex(
					&tasks, type_a, type_b, shape_data[type_a], shape_data[type_b], pose_a, pose_b, 0.15, &shapes,
				);
				reverse, reverse_status := physics.collision_task_registry_test_convex(
					&tasks, type_b, type_a, shape_data[type_b], shape_data[type_a], pose_b, pose_a, 0.15, &shapes,
				);
				testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
				testing.expect_value(t, repeated_status, physics.Physics_Status.Ok);
				testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
				testing.expect_value(t, forward, repeated);
				if type_a != type_b
				{
					testing.expect_value(t, forward.count, reverse.count);
				}
				testing.expect(t, forward.count >= 0 && forward.count <= physics.MAXIMUM_MANIFOLD_CONTACT_COUNT);
				if forward.count > 0
				{
					testing.expect(t, util.vector3_dot(forward.normal, reverse.normal) < -0.985);
					if type_a != physics.CYLINDER_TYPE_ID || type_b != physics.CONVEX_HULL_TYPE_ID
					{
						for contact_index in 0 ..< int(forward.count)
						{
							testing.expect(t, forward.contacts[contact_index].depth >= -0.1501);
						}
					}
				}
			}
		}
	}
}

@(test)
cylinder_specialized_routes_generate_stable_multicontact_features :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	identity := physics.rigid_pose_identity();
	side_pose := identity;
	side_pose.position = {1.5, 0, 0};
	box := physics.Box{1, 1, 1};
	cylinder := physics.Cylinder{1, 1};
	box_cylinder, box_status := physics.box_cylinder_test(&box, &cylinder, identity, side_pose, 0, &shapes);
	cylinder_pair, pair_status := physics.cylinder_pair_test(&cylinder, &cylinder, identity, side_pose, 0, &shapes);
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);
	testing.expect_value(t, pair_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_cylinder.count, i32(2));
	testing.expect_value(t, cylinder_pair.count, i32(2));
	for contact_index in 0 ..< 2
	{
		testing.expect_value(t, box_cylinder.contacts[contact_index].feature_id, i32(contact_index));
		testing.expect_value(t, cylinder_pair.contacts[contact_index].feature_id, i32(contact_index));
	}

	triangle := physics.Triangle{{-3, 0, -3}, {0, 0, 3}, {3, 0, -3}};
	cap_pose := identity;
	cap_pose.position = {0, -0.5, 0};
	triangle_cylinder, triangle_status := physics.triangle_cylinder_test(
		&triangle, &cylinder, identity, cap_pose, 0, &shapes,
	);
	testing.expect_value(t, triangle_status, physics.Physics_Status.Ok);
	testing.expect(t, triangle_cylinder.count >= 3);
	testing.expect(t, triangle_cylinder.contacts[0].feature_id & physics.MESH_REDUCTION_FACE_COLLISION_FLAG != 0);
}

@(test)
generated_depth_refiner_vertex_edge_face_and_degenerate_cases_are_executable :: proc(t: ^testing.T)
{
	testing.expect_value(t, physics.DEPTH_REFINER_INITIAL_MAXIMUM_ITERATIONS, 25);
	testing.expect_value(t, physics.DEPTH_REFINER_REUSED_SIMPLEX_MAXIMUM_ITERATIONS, 50);
	a := physics.Depth_Refiner_Vertex{support={0, 0, 0}, support_a={10, 0, 0}};
	b := physics.Depth_Refiner_Vertex{support={2, 0, 0}, support_a={12, 0, 0}};
	c := physics.Depth_Refiner_Vertex{support={0, 2, 0}, support_a={10, 2, 0}};
	vertex := physics.depth_refiner_closest_triangle(a, b, c, {-1, -1, 0});
	edge := physics.depth_refiner_closest_triangle(a, b, c, {1, -1, 0});
	face := physics.depth_refiner_closest_triangle(a, b, c, {0.5, 0.5, 1});
	testing.expect_value(t, vertex.mask, u8(1));
	testing.expect_value(t, edge.mask, u8(3));
	testing.expect_value(t, face.mask, u8(7));
	testing.expect(t, abs(face.witness_a.x - 10.5) < 1e-5);
	degenerate_c := physics.Depth_Refiner_Vertex{support={4, 0, 0}, support_a={14, 0, 0}};
	degenerate := physics.depth_refiner_closest_triangle(a, b, degenerate_c, {3, 1, 0});
	testing.expect(t, degenerate.mask != 0);
}

@(test)
convex_pair_hot_dispatch_makes_no_general_allocator_calls_after_startup :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere_a := physics.Sphere{0.75};
	sphere_b := physics.Sphere{1.25};
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {1.5, 0.25, -0.125};

	tracker := (^mem.Tracking_Allocator)(context.allocator.data);
	allocation_count_before := tracker.total_allocation_count;
	hot_status := physics.Physics_Status.Ok;
	for _ in 0 ..< 1024
	{
		_, hot_status = physics.collision_task_registry_test_convex(
			&tasks, physics.SPHERE_TYPE_ID, physics.SPHERE_TYPE_ID,
			&sphere_a, &sphere_b, pose_a, pose_b, 0.1, &shapes,
		);
		if hot_status != .Ok
		{
			break;
		}
	}
	allocation_count_after := tracker.total_allocation_count;

	testing.expect_value(t, hot_status, physics.Physics_Status.Ok);
	testing.expect_value(t, allocation_count_after, allocation_count_before);
}
