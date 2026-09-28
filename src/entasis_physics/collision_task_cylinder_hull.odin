package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

collision_project_hull_point_onto_cylinder_cap :: proc "contextless" (
	cap_center: util.Vector3, cylinder_orientation: util.Matrix3x3,
	inverse_normal_dot_axis: f32, local_normal, point: util.Vector3,
) -> util.Vector2
{
	t := util.vector3_dot(util.vector3_subtract(cap_center, point), cylinder_orientation.y) *
		inverse_normal_dot_axis;
	projected := util.vector3_subtract(point, util.vector3_scale(local_normal, t));
	from_center := util.vector3_subtract(projected, cap_center);
	return {
		util.vector3_dot(from_center, cylinder_orientation.x),
		util.vector3_dot(from_center, cylinder_orientation.z),
	};
}
collision_cylinder_hull_build_manifold :: proc "contextless" (
	a: Cylinder, b: ^Convex_Hull, pose_a, pose_b: Rigid_Pose,
	depth: f32, world_normal, world_closest_on_hull: util.Vector3,
	epsilon_scale, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	r_a := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	local_offset_a := util.vector3_negate(local_offset_b);
	local_normal := util.matrix3x3_transform_transpose(world_normal, world_r_b);
	closest_on_hull := util.matrix3x3_transform_transpose(
		util.vector3_subtract(world_closest_on_hull, pose_b.position), world_r_b,
	);
	closest_on_cylinder := util.vector3_subtract(closest_on_hull, util.vector3_scale(local_normal, depth));
	local_normal_in_a := util.matrix3x3_transform_transpose(local_normal, r_a);
	use_cap := abs(local_normal_in_a.y) > 0.70710678118;

	hull_face_normal, face_index, face_status := convex_hull_pick_representative_face(
		b, local_normal, closest_on_hull, epsilon_scale * 1e-3,
	);
	if face_status != .Ok
	{
		return {}, face_status;
	}
	face_start, face_end, range_status := convex_hull_face_range(b, face_index);
	if range_status != .Ok
	{
		return {}, range_status;
	}
	face_vertex_count := face_end - face_start;
	if face_vertex_count * 2 > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}
	previous_index := int(b.face_vertex_indices.memory[face_end - 1]);
	hull_face_origin := convex_hull_get_point(b, previous_index);
	manifold := Convex_Contact_Manifold{offset_b=offset_b, normal=world_normal};

	if use_cap
	{
		cap_scale := a.half_length;
		if local_normal_in_a.y > 0
		{
			cap_scale = -cap_scale;
		}
		cap_center := util.vector3_add(local_offset_a, util.vector3_scale(r_a.y, cap_scale));
		closest_cylinder_local := util.matrix3x3_transform_transpose(
			util.vector3_subtract(closest_on_cylinder, local_offset_a), r_a,
		);
		interior_points := collision_generate_cylinder_interior_points(a, local_normal_in_a, closest_cylinder_local);
		maximum_containment_dots: [4]f32;
		inverse_normal_dot_axis := 1 / local_normal_in_a.y;
		previous_vertex := hull_face_origin;
		previous_projected := collision_project_hull_point_onto_cylinder_cap(
			cap_center, r_a, inverse_normal_dot_axis, local_normal, previous_vertex,
		);
		candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
		candidate_count := 0;
		for face_vertex in 0 ..< face_vertex_count
		{
			point_index := int(b.face_vertex_indices.memory[face_start + face_vertex]);
			vertex := convex_hull_get_point(b, point_index);
			projected := collision_project_hull_point_onto_cylinder_cap(
				cap_center, r_a, inverse_normal_dot_axis, local_normal, vertex,
			);
			edge := util.vector2_subtract(projected, previous_projected);
			for interior_index in 0 ..< 4
			{
				point := interior_points[interior_index];
				containment := (point.x - previous_projected.x) * edge.y -
					(point.y - previous_projected.y) * edge.x;
				if inverse_normal_dot_axis > 0
				{
					containment = -containment;
				}
				maximum_containment_dots[interior_index] = max(
					maximum_containment_dots[interior_index], containment,
				);
			}
			t_min, t_max, intersected := collision_intersect_line_circle(previous_projected, edge, a.radius);
			if intersected == .Present
			{
				t_min = max(f32(0), t_min);
				t_max = min(f32(1), t_max);
				base_feature_id := i32((previous_index ~ point_index) << 8);
				if t_max >= t_min
				{
					point := util.vector2_add(previous_projected, util.vector2_scale(edge, t_max));
					candidates[candidate_count] = {
						x=point.x, y=point.y, feature_id=base_feature_id + i32(point_index),
					};
					candidate_count += 1;
				}
				if t_min < t_max && t_min > 0
				{
					point := util.vector2_add(previous_projected, util.vector2_scale(edge, t_min));
					candidates[candidate_count] = {
						x=point.x, y=point.y, feature_id=base_feature_id + i32(previous_index),
					};
					candidate_count += 1;
				}
			}
			previous_index = point_index;
			previous_vertex = vertex;
			previous_projected = projected;
		}
		for interior_index in 0 ..< 4
		{
			if maximum_containment_dots[interior_index] <= 0
			{
				point := interior_points[interior_index];
				candidates[candidate_count] = {x=point.x, y=point.y, feature_id=i32(interior_index)};
				candidate_count += 1;
			}
		}
		reduce_status: Physics_Status;
		manifold, reduce_status = manifold_candidate_reduce(
			&candidates, candidate_count, hull_face_normal,
			-1 / util.vector3_dot(local_normal, hull_face_normal),
			hull_face_origin, cap_center, r_a.x, r_a.z,
			epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
		);
		if reduce_status != .Ok
		{
			return {}, reduce_status;
		}
		collision_manifold_push_contacts(&manifold);
		return manifold, .Ok;
	}

	cylinder_to_closest := util.vector3_subtract(closest_on_cylinder, local_offset_a);
	side_edge_center := util.vector3_subtract(
		closest_on_cylinder, util.vector3_scale(r_a.y, util.vector3_dot(cylinder_to_closest, r_a.y)),
	);
	previous_vertex := hull_face_origin;
	latest_entry_numerator := f32(math.F32_MAX);
	latest_entry_denominator := f32(-1);
	earliest_exit_numerator := f32(math.F32_MAX);
	earliest_exit_denominator := f32(1);
	for face_vertex in 0 ..< face_vertex_count
	{
		point_index := int(b.face_vertex_indices.memory[face_start + face_vertex]);
		vertex := convex_hull_get_point(b, point_index);
		edge := util.vector3_subtract(vertex, previous_vertex);
		edge_plane_normal := util.vector3_cross(edge, local_normal);
		numerator := util.vector3_dot(util.vector3_subtract(previous_vertex, side_edge_center), edge_plane_normal);
		denominator := util.vector3_dot(edge_plane_normal, r_a.y);
		previous_vertex = vertex;
		plane_length_squared := util.vector3_length_squared(edge_plane_normal);
		denominator_squared := denominator * denominator;
		if denominator_squared > 1e-5 * plane_length_squared
		{
			if denominator_squared < 3e-4 * plane_length_squared
			{
				restrict_weight := max(f32(0), min(f32(1),
					(denominator_squared / plane_length_squared - 1e-5) / (3e-4 - 1e-5),
				));
				unrestricted_numerator := a.half_length * denominator;
				if denominator < 0
				{
					unrestricted_numerator = -unrestricted_numerator;
				}
				numerator = restrict_weight * numerator + (1 - restrict_weight) * unrestricted_numerator;
			}
			if denominator < 0
			{
				if numerator * latest_entry_denominator > latest_entry_numerator * denominator
				{
					latest_entry_numerator = numerator;
					latest_entry_denominator = denominator;
				}
			}
			else if numerator * earliest_exit_denominator < earliest_exit_numerator * denominator
			{
				earliest_exit_numerator = numerator;
				earliest_exit_denominator = denominator;
			}
		}
	}
	latest_entry := min(a.half_length, max(-a.half_length, latest_entry_numerator / latest_entry_denominator));
	earliest_exit := min(a.half_length, max(-a.half_length, earliest_exit_numerator / earliest_exit_denominator));
	inverse_depth_denominator := 1 / util.vector3_dot(hull_face_normal, local_normal);
	contact_ts := [2]f32{earliest_exit, latest_entry};
	for contact_index in 0 ..< 2
	{
		if contact_index == 1 && earliest_exit - latest_entry <= a.half_length * 1e-3
		{
			continue;
		}
		contact := util.vector3_add(side_edge_center, util.vector3_scale(r_a.y, contact_ts[contact_index]));
		contact_depth := util.vector3_dot(
			util.vector3_subtract(hull_face_origin, contact), hull_face_normal,
		) * inverse_depth_denominator;
		add_status := collision_manifold_add_local_b(
			&manifold, contact, contact_depth, i32(contact_index), world_r_b,
		);
		if add_status != .Ok
		{
			return {}, add_status;
		}
	}
	collision_manifold_push_contacts(&manifold);
	return manifold, .Ok;
}
cylinder_convex_hull_test_source :: proc "contextless" (
	a: Cylinder, b: ^Convex_Hull, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
	shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if cylinder_validate(a) != .Ok || convex_hull_validate(b) != .Ok || speculative_margin < 0 || shapes == nil
	{
		return {}, .Invalid_Description;
	}
	shape_a := a;
	hull_point := convex_hull_get_point(b, 0);
	hull_scale := (abs(hull_point.x) + abs(hull_point.y) + abs(hull_point.z)) / 3;
	epsilon_scale := min(max(a.radius, a.half_length), hull_scale);
	depth, normal, closest_on_b, refine_status := collision_refine_pair_normal(
		&shape_a, CYLINDER_TYPE_ID, pose_a, b, CONVEX_HULL_TYPE_ID, pose_b,
		epsilon_scale * 1e-5, 1e-16, speculative_margin, shapes,
	);
	if refine_status != .Ok
	{
		return {}, refine_status;
	}
	if depth < -speculative_margin
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	return collision_cylinder_hull_build_manifold(
		a, b, pose_a, pose_b, depth, normal, closest_on_b,
		epsilon_scale, -speculative_margin,
	);
}
collision_hull_support_world_wide :: proc "contextless" (
	hulls: Convex_Hull_Wide, orientation: util.Matrix3x3_Wide, position, direction: util.Vector3_Wide,
	terminated_lanes: util.I32x8,
) -> (util.Vector3_Wide, Physics_Status)
{
	support: util.Vector3_Wide;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(terminated_lanes, lane) < 0
		{
			continue;
		}
		local_direction := util.matrix3x3_transform_transpose(
			util.vector3_wide_read_slot(direction, lane), util.matrix3x3_wide_read_slot(orientation, lane),
		);
		local_support, support_status := convex_hull_support(hulls.hulls[lane], local_direction);
		if support_status != .Ok
		{
			return {}, support_status;
		}
		world_support := util.vector3_add(
			util.vector3_wide_read_slot(position, lane),
			util.matrix3x3_transform(local_support, util.matrix3x3_wide_read_slot(orientation, lane)),
		);
		util.vector3_wide_write_slot(&support, lane, world_support);
	}
	return support, .Ok;
}
Cylinder_Hull_Depth_Context_Wide :: struct
{
	cylinder:              Cylinder_Wide,
	hulls:                 Convex_Hull_Wide,
	cylinder_orientation:  util.Matrix3x3_Wide,
	hull_orientation:      util.Matrix3x3_Wide,
	offset_b:              util.Vector3_Wide,
}
cylinder_hull_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_b: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Cylinder_Hull_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	support_on_b, status = collision_hull_support_world_wide(
		depth_context.hulls, depth_context.hull_orientation, depth_context.offset_b, direction, terminated_lanes,
	);
	if status != .Ok
	{
		return {}, {}, status;
	}
	support_on_a := collision_cylinder_support_world_wide(
		depth_context.cylinder, depth_context.cylinder_orientation, {}, util.vector3_wide_negate(direction),
	);
	return util.vector3_wide_subtract(support_on_b, support_on_a), support_on_b, .Ok;
}
cylinder_convex_hull_test_wide :: proc "contextless" (
	a: Cylinder_Wide, b: Convex_Hull_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
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
	hull_scale: util.F32x8;
	for lane in 0 ..< pair_count
	{
		hull := b.hulls[lane];
		if convex_hull_validate(hull) != .Ok
		{
			return {}, .Invalid_Description;
		}
		point := convex_hull_get_point(hull, 0);
		hull_scale = simd.replace(hull_scale, lane, (abs(point.x) + abs(point.y) + abs(point.z)) / 3);
	}
	epsilon_scale := simd.min(simd.max(a.radius, a.half_length), hull_scale);
	depth_context := Cylinder_Hull_Depth_Context_Wide{
		cylinder=a, hulls=b, cylinder_orientation=orientation_matrix_a,
		hull_orientation=orientation_matrix_b, offset_b=offset_b,
	};
	depth, normal, closest_on_b, depth_status := depth_refiner_find_minimum_depth_wide(
		cylinder_hull_depth_support_wide, &depth_context,
		collision_depth_initial_normal_wide(offset_b, orientation_matrix_b.y, 1e-8), ~active,
		simd.mul(epsilon_scale, util.F32x8(1e-5)),
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
		cylinder := Cylinder{simd.extract(a.radius, lane), simd.extract(a.half_length, lane)};
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		lane_normal := util.vector3_wide_read_slot(normal, lane);
		lane_closest := util.vector3_wide_read_slot(closest_on_b, lane);
		lane_manifold, lane_status := collision_cylinder_hull_build_manifold(
			cylinder, b.hulls[lane], pose_a, pose_b, simd.extract(depth, lane), lane_normal, lane_closest,
			simd.extract(epsilon_scale, lane), -simd.extract(speculative_margin, lane),
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
