package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

collision_cylinder_pair_build_manifold :: proc "contextless" (
	a, b: Cylinder, pose_a, pose_b: Rigid_Pose,
	depth: f32, world_normal, world_closest_on_b: util.Vector3, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	r_a := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	local_offset_a := util.vector3_negate(local_offset_b);
	local_normal := util.matrix3x3_transform_transpose(world_normal, world_r_b);
	closest_on_b := util.matrix3x3_transform_transpose(
		util.vector3_subtract(world_closest_on_b, pose_b.position), world_r_b,
	);

	n_dot_a_y := util.vector3_dot(r_a.y, local_normal);
	inverse_n_dot_a_y := 1 / n_dot_a_y;
	inverse_normal_y := 1 / local_normal.y;
	cap_center_a_scale := a.half_length;
	if n_dot_a_y > 0
	{
		cap_center_a_scale = -cap_center_a_scale;
	}
	cap_center_a := util.vector3_add(local_offset_a, util.vector3_scale(r_a.y, cap_center_a_scale));
	cap_center_b_y := b.half_length;
	if local_normal.y < 0
	{
		cap_center_b_y = -cap_center_b_y;
	}
	use_cap_a := abs(n_dot_a_y) > 0.70710678118;
	use_cap_b := abs(local_normal.y) > 0.70710678118;

	extreme_a := util.vector3_subtract(closest_on_b, util.vector3_scale(local_normal, depth));
	extreme_a_horizontal := util.vector3_subtract(extreme_a, local_offset_a);
	extreme_a_horizontal = util.vector3_subtract(
		extreme_a_horizontal,
		util.vector3_scale(r_a.y, util.vector3_dot(extreme_a_horizontal, r_a.y)),
	);
	extreme_b := util.Vector2{closest_on_b.x, closest_on_b.z};
	cap_feature_normal_a := r_a.y;
	if n_dot_a_y > 0
	{
		cap_feature_normal_a = util.vector3_negate(cap_feature_normal_a);
	}
	feature_normal_a := cap_feature_normal_a;
	feature_position_a := cap_center_a;
	contacts: [4]util.Vector3;
	exists: [4]Reference_State;

	if use_cap_a && use_cap_b
	{
		parallel_threshold :: f32(0.9999);
		parallel_interpolation_max :: f32(0.99995);
		abs_a_dot := abs(n_dot_a_y);
		abs_b_dot := abs(local_normal.y);
		both_not_parallel := abs_a_dot < parallel_threshold && abs_b_dot < parallel_threshold;
		cap_contacts: [4]util.Vector2;
		cap_contacts[0] = extreme_b;
		if !both_not_parallel
		{
			cap_center_a_on_b := collision_project_onto_cap_b(
				cap_center_b_y, inverse_normal_y, local_normal, cap_center_a,
			);
			horizontal_length := util.vector2_length(cap_center_a_on_b);
			horizontal_direction := util.Vector2{1, 0};
			if horizontal_length >= 1e-14
			{
				horizontal_direction = util.vector2_scale(cap_center_a_on_b, 1 / horizontal_length);
			}
			initial_line_start := util.vector2_scale(horizontal_direction, b.radius);
			line_endpoint := util.vector2_scale(initial_line_start, -1);
			start_3 := util.Vector3{initial_line_start.x, cap_center_b_y, initial_line_start.y};
			end_3 := util.Vector3{line_endpoint.x, cap_center_b_y, line_endpoint.y};
			line_start_a := collision_project_onto_cap_a(
				cap_center_a, r_a, inverse_n_dot_a_y, local_normal, start_3,
			);
			line_end_a := collision_project_onto_cap_a(
				cap_center_a, r_a, inverse_n_dot_a_y, local_normal, end_3,
			);
			line_direction_a := util.vector2_subtract(line_end_a, line_start_a);
			line_direction_b := util.vector2_subtract(line_endpoint, initial_line_start);
			t_min_a, t_max_a, _ := collision_intersect_line_circle(line_start_a, line_direction_a, a.radius);
			first_t_min := max(f32(0), t_min_a);
			first_t_max := min(f32(1), t_max_a);
			cap_contacts[0] = util.vector2_add(initial_line_start, util.vector2_scale(line_direction_b, first_t_min));
			cap_contacts[1] = util.vector2_add(initial_line_start, util.vector2_scale(line_direction_b, first_t_max));

			circle_intersection_t := f32(0.5) *
				(horizontal_length + (b.radius * b.radius - a.radius * a.radius) / max(horizontal_length, f32(1e-30)));
			second_start_t := min(horizontal_length, max(f32(0), circle_intersection_t));
			second_start_b := util.vector2_scale(horizontal_direction, second_start_t);
			second_direction_b := util.Vector2{horizontal_direction.y, -horizontal_direction.x};
			second_end_b := util.vector2_add(second_start_b, second_direction_b);
			second_start_3 := util.Vector3{second_start_b.x, cap_center_b_y, second_start_b.y};
			second_end_3 := util.Vector3{second_end_b.x, cap_center_b_y, second_end_b.y};
			second_start_a := collision_project_onto_cap_a(
				cap_center_a, r_a, inverse_n_dot_a_y, local_normal, second_start_3,
			);
			second_end_a := collision_project_onto_cap_a(
				cap_center_a, r_a, inverse_n_dot_a_y, local_normal, second_end_3,
			);
			second_direction_a := util.vector2_subtract(second_end_a, second_start_a);
			second_t_min_a, second_t_max_a, _ := collision_intersect_line_circle(
				second_start_a, second_direction_a, a.radius,
			);
			second_t_min_b, second_t_max_b, _ := collision_intersect_line_circle(
				second_start_b, second_direction_b, b.radius,
			);
			second_t_min := max(second_t_min_a, second_t_min_b);
			second_t_max := min(second_t_max_a, second_t_max_b);
			cap_contacts[2] = util.vector2_add(second_start_b, util.vector2_scale(second_direction_b, second_t_min));
			cap_contacts[3] = util.vector2_add(second_start_b, util.vector2_scale(second_direction_b, second_t_max));

			weight_a_parallel := max(f32(0), min(f32(1), (abs_a_dot - parallel_threshold) /
				(parallel_interpolation_max - parallel_threshold)));
			weight_b_parallel := max(f32(0), min(f32(1), (abs_b_dot - parallel_threshold) /
				(parallel_interpolation_max - parallel_threshold)));
			parallel_weight := weight_a_parallel * weight_b_parallel;
			extreme_weight := 1 - parallel_weight;
			center_to_extreme := util.vector2_subtract(extreme_b, second_start_b);
			replace_dot_0 := util.vector2_dot(horizontal_direction, center_to_extreme);
			replace_dot_2 := util.vector2_dot(second_direction_b, center_to_extreme);
			replace_index := 3;
			if abs(replace_dot_0) > abs(replace_dot_2)
			{
				replace_index = 1;
				if replace_dot_0 > 0
				{
					replace_index = 0;
				}
			}
			else if replace_dot_2 < 0
			{
				replace_index = 2;
			}
			cap_contacts[replace_index] = util.vector2_add(
				util.vector2_scale(extreme_b, extreme_weight),
				util.vector2_scale(cap_contacts[replace_index], parallel_weight),
			);
			if first_t_max > first_t_min
			{
				exists[1] = .Present;
			}
			exists[2] = exists[1];
			if exists[1] == .Present && second_t_max > second_t_min
			{
				exists[3] = .Present;
			}
		}
		exists[0] = .Present;
		for contact_index in 0 ..< 4
		{
			contacts[contact_index] = {cap_contacts[contact_index].x, cap_center_b_y, cap_contacts[contact_index].y};
		}
	}
	else
	{
		ax := util.vector3_dot(r_a.x, local_normal);
		az := util.vector3_dot(r_a.z, local_normal);
		horizontal_a_length := math.sqrt(ax * ax + az * az);
		side_feature_normal_a := util.vector3_scale(
			util.vector3_add(util.vector3_scale(r_a.x, ax), util.vector3_scale(r_a.z, az)),
			1 / horizontal_a_length,
		);
		side_center_a := util.vector3_add(extreme_a_horizontal, local_offset_a);
		side_center_b := util.Vector3{extreme_b.x, 0, extreme_b.y};
		if use_cap_a != use_cap_b
		{
			side_line_end_a := util.vector3_add(side_center_a, r_a.y);
			side_line_end_b := util.vector3_add(side_center_b, {0, 1, 0});
			projected_start: util.Vector2;
			projected_end: util.Vector2;
			radius := b.radius;
			side_half_length := a.half_length;
			if use_cap_a
			{
				projected_start = collision_project_onto_cap_a(
					cap_center_a, r_a, inverse_n_dot_a_y, local_normal, side_center_b,
				);
				projected_end = collision_project_onto_cap_a(
					cap_center_a, r_a, inverse_n_dot_a_y, local_normal, side_line_end_b,
				);
				radius = a.radius;
				side_half_length = b.half_length;
			}
			else
			{
				projected_start = collision_project_onto_cap_b(
					cap_center_b_y, inverse_normal_y, local_normal, side_center_a,
				);
				projected_end = collision_project_onto_cap_b(
					cap_center_b_y, inverse_normal_y, local_normal, side_line_end_a,
				);
			}
			projected_direction := util.vector2_subtract(projected_end, projected_start);
			t_min, t_max, _ := collision_intersect_line_circle(projected_start, projected_direction, radius);
			t_min = min(side_half_length, max(-side_half_length, t_min));
			t_max = min(side_half_length, t_max);
			if use_cap_a
			{
				contacts[0] = {side_center_b.x, t_min, side_center_b.z};
				contacts[1] = {side_center_b.x, t_max, side_center_b.z};
				feature_normal_a = cap_feature_normal_a;
				feature_position_a = cap_center_a;
			}
			else
			{
				point_0 := util.vector2_add(projected_start, util.vector2_scale(projected_direction, t_min));
				point_1 := util.vector2_add(projected_start, util.vector2_scale(projected_direction, t_max));
				contacts[0] = {point_0.x, cap_center_b_y, point_0.y};
				contacts[1] = {point_1.x, cap_center_b_y, point_1.y};
				feature_normal_a = side_feature_normal_a;
				feature_position_a = side_center_a;
			}
			exists[0] = .Present;
			if t_max > t_min
			{
				exists[1] = .Present;
			}
		}
		else
		{
			horizontal_b_length_squared := local_normal.x * local_normal.x + local_normal.z * local_normal.z;
			contact_t_min, contact_t_max := capsule_cylinder_contact_interval(
				a.half_length, b.half_length, r_a.y, local_normal,
				1 / horizontal_b_length_squared, util.vector3_negate(side_center_a),
			);
			contacts[0] = {extreme_b.x, contact_t_min, extreme_b.y};
			contacts[1] = {extreme_b.x, contact_t_max, extreme_b.y};
			exists[0] = .Present;
			if contact_t_max > contact_t_min
			{
				exists[1] = .Present;
			}
			feature_normal_a = side_feature_normal_a;
			feature_position_a = side_center_a;
		}
	}

	manifold := Convex_Contact_Manifold{offset_b=offset_b, normal=world_normal};
	inverse_feature_dot := 1 / util.vector3_dot(feature_normal_a, local_normal);
	for contact_index in 0 ..< 4
	{
		if exists[contact_index] == .Missing
		{
			continue;
		}
		contact_depth := util.vector3_dot(
			util.vector3_subtract(contacts[contact_index], feature_position_a), feature_normal_a,
		) * inverse_feature_dot;
		if contact_depth < depth_threshold
		{
			continue;
		}
		add_status := collision_manifold_add_local_b(
			&manifold, contacts[contact_index], contact_depth, i32(contact_index), world_r_b,
		);
		if add_status != .Ok
		{
			return {}, add_status;
		}
	}
	return manifold, .Ok;
}
cylinder_pair_test_source :: proc "contextless" (
	a, b: Cylinder, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if cylinder_validate(a) != .Ok || cylinder_validate(b) != .Ok || speculative_margin < 0 || shapes == nil
	{
		return {}, .Invalid_Description;
	}
	shape_a := a;
	shape_b := b;
	epsilon_scale := min(max(a.radius, a.half_length), max(b.radius, b.half_length));
	depth, normal, closest_on_b, refine_status := collision_refine_pair_normal(
		&shape_a, CYLINDER_TYPE_ID, pose_a, &shape_b, CYLINDER_TYPE_ID, pose_b,
		epsilon_scale * 1e-6, 1e-20, speculative_margin, shapes,
	);
	if refine_status != .Ok
	{
		return {}, refine_status;
	}
	if depth < -speculative_margin
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	return collision_cylinder_pair_build_manifold(
		a, b, pose_a, pose_b, depth, normal, closest_on_b, -speculative_margin,
	);
}
collision_cylinder_support_local_wide :: proc "contextless" (
	cylinder: Cylinder_Wide, direction: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	horizontal_length := simd.sqrt(simd.add(
		simd.mul(direction.x, direction.x), simd.mul(direction.z, direction.z),
	));
	horizontal_scale := simd.div(cylinder.radius, horizontal_length);
	use_horizontal := transmute(util.I32x8)simd.lanes_gt(horizontal_length, util.F32x8(1e-8));
	return {
		x=util.wide_select_f32(use_horizontal, simd.mul(direction.x, horizontal_scale), util.F32x8(0)),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(direction.y, util.F32x8(0)),
			cylinder.half_length, simd.neg(cylinder.half_length),
		),
		z=util.wide_select_f32(use_horizontal, simd.mul(direction.z, horizontal_scale), util.F32x8(0)),
	};
}
collision_cylinder_support_world_wide :: proc "contextless" (
	cylinder: Cylinder_Wide, orientation: util.Matrix3x3_Wide, position, direction: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	local_direction := util.matrix3x3_wide_transform_transposed(direction, orientation);
	local_support := collision_cylinder_support_local_wide(cylinder, local_direction);
	return util.vector3_wide_add(position, util.matrix3x3_wide_transform(local_support, orientation));
}
Cylinder_Pair_Depth_Context_Wide :: struct
{
	a, b:                  Cylinder_Wide,
	orientation_a:         util.Matrix3x3_Wide,
	orientation_b:         util.Matrix3x3_Wide,
	offset_b:              util.Vector3_Wide,
}
cylinder_pair_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_b: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Cylinder_Pair_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	support_on_b = collision_cylinder_support_world_wide(
		depth_context.b, depth_context.orientation_b, depth_context.offset_b, direction,
	);
	support_on_a := collision_cylinder_support_world_wide(
		depth_context.a, depth_context.orientation_a, {}, util.vector3_wide_negate(direction),
	);
	return util.vector3_wide_subtract(support_on_b, support_on_a), support_on_b, .Ok;
}
collision_depth_initial_normal_wide :: proc "contextless" (
	offset_b: util.Vector3_Wide, alternate: util.Vector3_Wide, fallback_distance: f32,
) -> util.Vector3_Wide
{
	initial := util.vector3_wide_negate(offset_b);
	length := util.vector3_wide_length(initial);
	normalized := util.vector3_wide_scale(initial, simd.div(util.F32x8(1), length));
	return util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(fallback_distance)), alternate, normalized,
	);
}
cylinder_pair_test_wide :: proc "contextless" (
	a, b: Cylinder_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	orientation_matrix_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	orientation_matrix_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	epsilon_scale := simd.min(simd.max(a.radius, a.half_length), simd.max(b.radius, b.half_length));
	depth_context := Cylinder_Pair_Depth_Context_Wide{
		a=a, b=b, orientation_a=orientation_matrix_a, orientation_b=orientation_matrix_b, offset_b=offset_b,
	};
	depth, normal, closest_on_b, depth_status := depth_refiner_find_minimum_depth_wide(
		cylinder_pair_depth_support_wide, &depth_context,
		collision_depth_initial_normal_wide(offset_b, orientation_matrix_b.y, 1e-10), ~active,
		simd.mul(epsilon_scale, util.F32x8(1e-6)),
		simd.neg(speculative_margin), {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	inactive := ~active | transmute(util.I32x8)simd.lanes_lt(depth, simd.neg(speculative_margin));
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		cylinder_a := Cylinder{simd.extract(a.radius, lane), simd.extract(a.half_length, lane)};
		cylinder_b := Cylinder{simd.extract(b.radius, lane), simd.extract(b.half_length, lane)};
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		lane_normal := util.vector3_wide_read_slot(normal, lane);
		lane_closest := util.vector3_wide_read_slot(closest_on_b, lane);
		lane_manifold, lane_status := collision_cylinder_pair_build_manifold(
			cylinder_a, cylinder_b, pose_a, pose_b, simd.extract(depth, lane), lane_normal, lane_closest,
			-simd.extract(speculative_margin, lane),
		);
		if lane_status != .Ok
		{
			return {}, lane_status;
		}
		write_status := convex_4_manifold_wide_write_lane(&manifold, lane, lane_manifold);
		if write_status != .Ok
		{
			return {}, write_status;
		}
	}
	return manifold, .Ok;
}
