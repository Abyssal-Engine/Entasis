package sweeps_tests

import "core:testing"
import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// reference arithmetic without the production inactive-angular-motion exit
r18_integrate_orientation_reference :: proc "contextless" (
	start: util.Quaternion_Wide, angular_velocity: util.Vector3_Wide, half_times: util.F32x8,
) -> util.Quaternion_Wide
{
	speed := util.vector3_wide_length(angular_velocity);
	half_angle := simd.mul(speed, half_times);
	scale := simd.div(util.sin_approx_wide(half_angle), speed);
	delta := util.Quaternion_Wide{
		x=simd.mul(angular_velocity.x, scale),
		y=simd.mul(angular_velocity.y, scale),
		z=simd.mul(angular_velocity.z, scale),
		w=util.cos_approx_wide(half_angle),
	};
	integrated := util.quaternion_wide_normalize(util.quaternion_wide_concatenate(start, delta));
	return util.quaternion_wide_select(
		transmute(util.I32x8)simd.lanes_gt(speed, util.F32x8(1e-15)), integrated, start,
	);
}

r18_construct_samples_reference :: proc "contextless" (
	t0, t1: f32, initial_parent_offset_b, linear_b, angular_a, angular_b: util.Vector3_Wide,
	initial_parent_orientation_a, initial_parent_orientation_b: util.Quaternion_Wide,
	local_pose_a, local_pose_b: physics.Rigid_Pose,
) -> (samples: util.F32x8, offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide)
{
	samples = physics.sweep_sample_times_wide(t0, t1);
	offset_b = util.vector3_wide_add(initial_parent_offset_b, util.vector3_wide_scale(linear_b, samples));
	half_samples := simd.mul(samples, util.F32x8(0.5));
	parent_orientation_a := r18_integrate_orientation_reference(initial_parent_orientation_a, angular_a, half_samples);
	parent_orientation_b := r18_integrate_orientation_reference(initial_parent_orientation_b, angular_b, half_samples);
	orientation_a = util.quaternion_wide_concatenate(
		util.quaternion_wide_broadcast(local_pose_a.orientation), parent_orientation_a,
	);
	orientation_b = util.quaternion_wide_concatenate(
		util.quaternion_wide_broadcast(local_pose_b.orientation), parent_orientation_b,
	);
	child_position_a := util.quaternion_wide_transform(
		util.vector3_wide_broadcast(local_pose_a.position), parent_orientation_a,
	);
	child_position_b := util.quaternion_wide_transform(
		util.vector3_wide_broadcast(local_pose_b.position), parent_orientation_b,
	);
	offset_b = util.vector3_wide_add(offset_b, util.vector3_wide_subtract(child_position_b, child_position_a));
	return;
}


r19_expect_wide_bits :: proc(t: ^testing.T, actual, expected: util.F32x8)
{
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		a := simd.extract(actual, lane);
		b := simd.extract(expected, lane);
		testing.expect_value(t, transmute(u32)a, transmute(u32)b);
	}
}

@(test)
linear_sweep_refinement_preserves_generic_path_bits :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere: physics.Sphere = {0.7};
	target_sphere: physics.Sphere = {0.9};
	triangle: physics.Triangle = {{0, -3, -3}, {0, 3, -3}, {0, 0, 3}};
	box: physics.Box = {0.8, 1.2, 1.4};
	negative_zero: f32 = transmute(f32)u32(0x80000000);
	// nonzero components force the general route, but every angular speed
	// underflows to zero for these finite poses, preserving the reference motion
	reference_angular: util.Vector3 = {1e-30, -1e-30, 1e-30};
	testing.expect_value(t, util.vector3_length(reference_angular), f32(0));
	budgets: [3]int = {1, 2, 64};
	refined_count: int;
	for case_index in 0 ..< 16
	{
		x: f32 = f32(case_index) * 0.037;
		parent_a: physics.Rigid_Pose = {
			position={-3.5, x, negative_zero},
			orientation=util.quaternion_from_axis_angle(util.vector3_normalize({0.3, -0.8, 0.2}), x),
		};
		parent_b: physics.Rigid_Pose = {
			position={0.5, -x, 0},
			orientation=util.quaternion_from_axis_angle(util.vector3_normalize({-0.2, 0.7, 0.6}), -x),
		};
		local_a: physics.Rigid_Pose = {
			position={x, -0.3, 0.7}, orientation=util.quaternion_from_axis_angle({0, 1, 0}, 0.13),
		};
		local_b: physics.Rigid_Pose = {
			position={-0.8, x, 0.1}, orientation=util.quaternion_from_axis_angle({1, 0, 0}, -0.27),
		};
		if case_index == 0
		{
			local_a = physics.rigid_pose_identity();
			local_b = physics.rigid_pose_identity();
			local_a.position = {negative_zero, 0, negative_zero};
			local_b.orientation.x = negative_zero;
		}
		velocity_a: physics.Body_Velocity = {linear={2, 0.03, negative_zero}};
		velocity_b: physics.Body_Velocity = {linear={-0.5, -0.02, 0}};
		if case_index & 1 != 0
		{
			velocity_a.angular = {negative_zero, 0, negative_zero};
			velocity_b.angular = {0, negative_zero, 0};
		}
		reference_a: physics.Body_Velocity = velocity_a;
		reference_b: physics.Body_Velocity = velocity_b;
		reference_a.angular = reference_angular;
		reference_b.angular = reference_angular;
		for target in 0 ..< 3
		{
			shape_b: rawptr = &triangle;
			type_b: int = physics.TRIANGLE_TYPE_ID;
			if target == 1
			{
				shape_b = &box;
				type_b = physics.BOX_TYPE_ID;
			}
			else if target == 2
			{
				shape_b = &target_sphere;
				type_b = physics.SPHERE_TYPE_ID;
			}
			first: physics.Sweep_Result;
			for budget in budgets
			{
				actual, actual_status := physics.sweep_task_test_convex_distance_internal(
					&sphere, shape_b, physics.SPHERE_TYPE_ID, type_b,
					parent_a, parent_b, local_a, local_b, velocity_a, velocity_b,
					3, 1e-5, 1e-5, budget, &shapes,
				);
				expected, expected_status := physics.sweep_task_test_convex_distance_internal(
					&sphere, shape_b, physics.SPHERE_TYPE_ID, type_b,
					parent_a, parent_b, local_a, local_b, reference_a, reference_b,
					3, 1e-5, 1e-5, budget, &shapes,
				);
				testing.expect_value(t, actual_status, physics.Physics_Status.Ok);
				testing.expect_value(t, actual_status, expected_status);
				testing.expect_value(t, actual.state, expected.state);
				testing.expect_value(t, actual.child_a, expected.child_a);
				testing.expect_value(t, actual.child_b, expected.child_b);
				actual_values: [8]f32 = {actual.t0, actual.t1, actual.location.x, actual.location.y,
					actual.location.z, actual.normal.x, actual.normal.y, actual.normal.z};
				expected_values: [8]f32 = {expected.t0, expected.t1, expected.location.x, expected.location.y,
					expected.location.z, expected.normal.x, expected.normal.y, expected.normal.z};
				for value, i in actual_values
				{
					testing.expect_value(t, transmute(u32)value, transmute(u32)expected_values[i]);
				}
				if budget == 1
				{
					first = actual;
				}
				else if actual.t0 != first.t0 || actual.t1 != first.t1
				{
					refined_count += 1;
				}
			}
		}
	}
	testing.expect(t, refined_count > 0);
}

@(test)
mesh_sweep_preserves_linear_and_rotating_triangle_reference :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere: physics.Sphere = {0.5};
	triangle: physics.Triangle = {{-3, 0, -3}, {0, 0, 3}, {3, 0, -3}};
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
	defer physics.mesh_dispose(&mesh, &pool);
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	triangle_index, triangle_status := physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, triangle_status, physics.Physics_Status.Ok);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	angular_speeds: [4]f32 = {0, 1e-30, -0.4, 0.4};
	for speed in angular_speeds
	{
		for order in 0 ..< 2
		{
			pose_a: physics.Rigid_Pose = {position={0.1, 4, -0.2}, orientation=util.quaternion_identity()};
			pose_b: physics.Rigid_Pose = {position={0.2, 0.1, 0.3}, orientation=util.quaternion_from_axis_angle({0, 0, 1}, 0.13)};
			velocity_a: physics.Body_Velocity = {linear={0, -5, 0}, angular={speed, 0, 0}};
			velocity_b: physics.Body_Velocity = {angular={0, 0, speed}};
			expected, expected_status := physics.sweep_task_registry_test(
				&sweeps, sphere_index if order == 0 else triangle_index, triangle_index if order == 0 else sphere_index,
				pose_a if order == 0 else pose_b, pose_b if order == 0 else pose_a,
				velocity_a if order == 0 else velocity_b, velocity_b if order == 0 else velocity_a,
				1, 1e-5, 1e-5, 64, &shapes, &collisions,
			);
			actual, actual_status := physics.sweep_task_registry_test(
				&sweeps, sphere_index if order == 0 else mesh_index, mesh_index if order == 0 else sphere_index,
				pose_a if order == 0 else pose_b, pose_b if order == 0 else pose_a,
				velocity_a if order == 0 else velocity_b, velocity_b if order == 0 else velocity_a,
				1, 1e-5, 1e-5, 64, &shapes, &collisions, pool=&pool,
			);
			testing.expect_value(t, actual_status, physics.Physics_Status.Ok);
			testing.expect_value(t, expected_status, actual_status);
			testing.expect_value(t, actual.state, physics.Sweep_Hit_State.Hit);
			testing.expect_value(t, actual.state, expected.state);
			testing.expect(t, abs(actual.t1 - expected.t1) < 1e-4);
			testing.expect(t, util.vector3_distance(actual.normal, expected.normal) < 1e-3);
			testing.expect_value(t, actual.child_b if order == 0 else actual.child_a, i32(0));
		}
	}
}

r19_expect_quaternion_bits :: proc(t: ^testing.T, actual, expected: util.Quaternion_Wide)
{
	r19_expect_wide_bits(t, actual.x, expected.x);
	r19_expect_wide_bits(t, actual.y, expected.y);
	r19_expect_wide_bits(t, actual.z, expected.z);
	r19_expect_wide_bits(t, actual.w, expected.w);
}

@(test)
sweep_angular_integration_preserves_every_lane_mask_and_threshold :: proc(t: ^testing.T)
{
	times := util.F32x8{0, 1e-8, 0.01, 0.3, 1, 5, 10, 64};
	for round in 0 ..< 4
	{
		for mask in 0 ..< 256
		{
			start: util.Quaternion_Wide;
			angular: util.Vector3_Wide;
			for lane in 0 ..< util.PRODUCTION_LANE_COUNT
			{
				q := util.quaternion_from_axis_angle(
					util.vector3_normalize({0.2 + f32(lane), 0.8, -0.4}),
					f32(round * 8 + lane) * 0.19,
				);
				util.quaternion_wide_write_slot(&start, lane, q);
				v := util.Vector3{};
				if round & 1 != 0
				{
					v.x = 0.5e-15;
				}
				if mask & (1 << uint(lane)) != 0
				{
					v = util.vector3_scale(
						util.vector3_normalize({0.1 + f32(lane), -0.3, 0.8}),
						0.17 * f32(1 + round * 8 + lane),
					);
				}
				util.vector3_wide_write_slot(&angular, lane, v);
			}
			actual := physics.sweep_integrate_orientation_wide(start, angular, times);
			expected := r18_integrate_orientation_reference(start, angular, times);
			r19_expect_quaternion_bits(t, actual, expected);
		}
	}
	thresholds := [11]f32{0, -0.0, 1e-20, 0.5e-15, 1e-15, 1.000001e-15, 1e-6, 0.01, 0.3, 3, 30};
	for speed in thresholds
	{
		start := util.quaternion_wide_broadcast({0, transmute(f32)u32(0x80000000), 0, 1});
		angular := util.vector3_wide_broadcast({speed, 0, 0});
		r19_expect_quaternion_bits(t,
			physics.sweep_integrate_orientation_wide(start, angular, times),
			r18_integrate_orientation_reference(start, angular, times),
		);
	}
}

@(test)
sweep_sample_packets_preserve_parent_child_motion_and_full_float_bits :: proc(t: ^testing.T)
{
	for case_index in 0 ..< 64
	{
		for motion in 0 ..< 4
		{
			x := f32(case_index) * 0.017;
			t0 := x;
			t1 := t0 + f32(case_index % 7) * 0.23;
			parent_a := util.quaternion_wide_broadcast(util.quaternion_from_axis_angle(
				util.vector3_normalize({0.3, -0.8, 0.2}), x));
			parent_b := util.quaternion_wide_broadcast(util.quaternion_from_axis_angle(
				util.vector3_normalize({-0.2, 0.7, 0.6}), -x));
			angular_a, angular_b: util.Vector3_Wide;
			for lane in 0 ..< util.PRODUCTION_LANE_COUNT
			{
				if motion & 1 != 0
				{
					util.vector3_wide_write_slot(&angular_a, lane, {0.11 * f32(lane), -0.21, x});
				}
				if motion & 2 != 0
				{
					util.vector3_wide_write_slot(&angular_b, lane, {-0.07, x, 0.23 * f32(lane)});
				}
			}
			local_a := physics.Rigid_Pose{
				position={x, -0.3, 0.7},
				orientation=util.quaternion_from_axis_angle({0, 1, 0}, 0.13),
			};
			local_b := physics.Rigid_Pose{
				position={-0.8, x, 0.1},
				orientation=util.quaternion_from_axis_angle({1, 0, 0}, -0.27),
			};
			offset := util.vector3_wide_broadcast({2.7, x, -0.9});
			linear := util.vector3_wide_broadcast({-0.8, 0.2, x});
			a_times, a_offset, a_qa, a_qb, _ := physics.sweep_construct_samples_wide(
				t0, t1, offset, linear, angular_a, angular_b, parent_a, parent_b, local_a, local_b,
			);
			b_times, b_offset, b_qa, b_qb := r18_construct_samples_reference(
				t0, t1, offset, linear, angular_a, angular_b, parent_a, parent_b, local_a, local_b,
			);
			r19_expect_wide_bits(t, a_times, b_times);
			r19_expect_wide_bits(t, a_offset.x, b_offset.x);
			r19_expect_wide_bits(t, a_offset.y, b_offset.y);
			r19_expect_wide_bits(t, a_offset.z, b_offset.z);
			r19_expect_quaternion_bits(t, a_qa, b_qa);
			r19_expect_quaternion_bits(t, a_qb, b_qb);
		}
	}
}
