// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Box_Hull_Depth_Context_Wide :: struct
{
	boxes:             Box_Wide,
	hulls:             Convex_Hull_Wide,
	local_offset_a:    util.Vector3_Wide,
	hull_local_r_box:  util.Matrix3x3_Wide,
}

box_hull_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Box_Hull_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(terminated_lanes, lane) < 0
		{
			continue;
		}
		hull := depth_context.hulls.hulls[lane];
		if convex_hull_validate(hull) != .Ok
		{
			return {}, {}, .Invalid_Description;
		}
		direction_hull := util.vector3_wide_read_slot(direction, lane);
		r_box := util.matrix3x3_wide_read_slot(depth_context.hull_local_r_box, lane);
		direction_box := util.matrix3x3_transform_transpose(util.vector3_negate(direction_hull), r_box);
		hull_support, hull_status := convex_hull_support(hull, direction_hull);
		if hull_status != .Ok
		{
			return {}, {}, hull_status;
		}
		box := Box{
			simd.extract(depth_context.boxes.half_width, lane),
			simd.extract(depth_context.boxes.half_height, lane),
			simd.extract(depth_context.boxes.half_length, lane),
		};
		box_support, box_status := box_support(box, direction_box);
		if box_status != .Ok
		{
			return {}, {}, box_status;
		}
		box_support = util.vector3_add(
			util.matrix3x3_transform(box_support, r_box),
			util.vector3_wide_read_slot(depth_context.local_offset_a, lane),
		);
		util.vector3_wide_write_slot(&support_on_a, lane, hull_support);
		util.vector3_wide_write_slot(&support, lane, util.vector3_subtract(hull_support, box_support));
	}
	return support, support_on_a, .Ok;
}

box_convex_hull_test_source :: proc "contextless" (
	box: Box, hull: ^Convex_Hull, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if box_validate(box) != .Ok || convex_hull_validate(hull) != .Ok
	{
		return {}, .Invalid_Description;
	}
	box_orientation := util.matrix3x3_from_quaternion(pose_a.orientation);
	hull_orientation := util.matrix3x3_from_quaternion(pose_b.orientation);
	hull_local_box_orientation := util.matrix3x3_multiply(box_orientation, util.matrix3x3_transpose(hull_orientation));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, hull_orientation);
	local_offset_a := util.vector3_negate(local_offset_b);
	center_distance := util.vector3_length(local_offset_a);
	initial_normal := util.Vector3{0, 1, 0};
	if center_distance >= 1e-8
	{
		initial_normal = util.vector3_scale(local_offset_a, 1 / center_distance);
	}
	hull_point := convex_hull_get_point(hull, 0);
	hull_epsilon_scale := (abs(hull_point.x) + abs(hull_point.y) + abs(hull_point.z)) * (1.0 / 3.0);
	epsilon_scale := min(max(box.half_width, max(box.half_height, box.half_length)), hull_epsilon_scale);
	depth_threshold := -speculative_margin;

	depth_context: Box_Hull_Depth_Context_Wide;
	_ = box_wide_write_slot(&depth_context.boxes, 0, box);
	depth_context.hulls.hulls[0] = hull;
	util.vector3_wide_write_slot(&depth_context.local_offset_a, 0, local_offset_a);
	util.vector3_wide_write_slot(&depth_context.hull_local_r_box.x, 0, hull_local_box_orientation.x);
	util.vector3_wide_write_slot(&depth_context.hull_local_r_box.y, 0, hull_local_box_orientation.y);
	util.vector3_wide_write_slot(&depth_context.hull_local_r_box.z, 0, hull_local_box_orientation.z);
	initial_normal_wide: util.Vector3_Wide;
	util.vector3_wide_write_slot(&initial_normal_wide, 0, initial_normal);
	inactive := util.I32x8(-1);
	inactive = simd.replace(inactive, 0, 0);
	depth_wide, normal_wide, closest_on_hull_wide, depth_status := depth_refiner_find_minimum_depth_wide(
		box_hull_depth_support_wide, &depth_context, initial_normal_wide, inactive,
		util.F32x8(1e-5 * epsilon_scale), util.F32x8(depth_threshold), {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	if simd.extract(depth_wide, 0) < depth_threshold
	{
		return {offset_b=offset_b}, .Ok;
	}
	local_normal := util.vector3_wide_read_slot(normal_wide, 0);
	closest_on_hull := util.vector3_wide_read_slot(closest_on_hull_wide, 0);

	local_normal_in_a := util.matrix3x3_transform_transpose(local_normal, hull_local_box_orientation);
	abs_local_normal_in_a := util.vector3_abs(local_normal_in_a);
	face_axis := 2;
	if abs_local_normal_in_a.x > abs_local_normal_in_a.y && abs_local_normal_in_a.x > abs_local_normal_in_a.z
	{
		face_axis = 0;
	}
	else if abs_local_normal_in_a.y > abs_local_normal_in_a.z
	{
		face_axis = 1;
	}
	box_face_normal := hull_local_box_orientation.z;
	box_face_x := hull_local_box_orientation.x;
	box_face_y := hull_local_box_orientation.y;
	box_face_half_width := box.half_width;
	box_face_half_height := box.half_height;
	box_face_normal_offset := box.half_length;
	negate_face := local_normal_in_a.z > 0;
	if face_axis == 0
	{
		box_face_normal = hull_local_box_orientation.x;
		box_face_x = hull_local_box_orientation.y;
		box_face_y = hull_local_box_orientation.z;
		box_face_half_width = box.half_height;
		box_face_half_height = box.half_length;
		box_face_normal_offset = box.half_width;
		negate_face = local_normal_in_a.x > 0;
	}
	else if face_axis == 1
	{
		box_face_normal = hull_local_box_orientation.y;
		box_face_x = hull_local_box_orientation.z;
		box_face_y = hull_local_box_orientation.x;
		box_face_half_width = box.half_length;
		box_face_half_height = box.half_width;
		box_face_normal_offset = box.half_height;
		negate_face = local_normal_in_a.y > 0;
	}
	if negate_face
	{
		box_face_normal = util.vector3_negate(box_face_normal);
	}
	else
	{
		box_face_x = util.vector3_negate(box_face_x);
	}
	box_face_center := util.vector3_add(local_offset_a, util.vector3_scale(box_face_normal, box_face_normal_offset));
	box_face_x_offset := util.vector3_scale(box_face_x, box_face_half_width);
	box_face_y_offset := util.vector3_scale(box_face_y, box_face_half_height);
	v_0 := util.vector3_subtract(box_face_center, box_face_x_offset);
	v_1 := util.vector3_add(box_face_center, box_face_x_offset);
	box_vertices := [4]util.Vector3{
		util.vector3_subtract(v_0, box_face_y_offset),
		util.vector3_subtract(v_1, box_face_y_offset),
		util.vector3_add(v_1, box_face_y_offset),
		util.vector3_add(v_0, box_face_y_offset),
	};
	box_edge_directions := [4]util.Vector3{
		box_face_x, box_face_y, util.vector3_negate(box_face_x), util.vector3_negate(box_face_y),
	};
	box_edge_plane_normals: [4]util.Vector3;
	for edge_index in 0 ..< 4
	{
		box_edge_plane_normals[edge_index] = util.vector3_cross(box_edge_directions[edge_index], local_normal);
	}

	face_normal, face_index, face_status := convex_hull_pick_representative_face(
		hull, local_normal, closest_on_hull, 1e-3 * epsilon_scale,
	);
	if face_status != .Ok
	{
		return {}, face_status;
	}
	face_start, face_end, face_range_status := convex_hull_face_range(hull, face_index);
	if face_range_status != .Ok
	{
		return {}, face_range_status;
	}
	face_vertex_count := face_end - face_start;
	maximum_contact_count := max(8, face_vertex_count);
	if maximum_contact_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}
	face_x, face_y := collision_build_orthonormal_basis(face_normal);
	previous_point_index := int(hull.face_vertex_indices.memory[face_end - 1]);
	face_origin := convex_hull_get_point(hull, previous_point_index);
	previous_vertex := face_origin;
	maximum_vertex_containment_dots: [4]f32;
	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
	candidate_count := 0;
	for face_vertex in 0 ..< face_vertex_count
	{
		point_index := int(hull.face_vertex_indices.memory[face_start + face_vertex]);
		vertex := convex_hull_get_point(hull, point_index);
		hull_edge_offset := util.vector3_subtract(vertex, previous_vertex);
		hull_edge_plane_normal := util.vector3_cross(hull_edge_offset, local_normal);
		latest_entry := -f32(3.402823466e+38);
		earliest_exit := f32(3.402823466e+38);
		for box_edge in 0 ..< 4
		{
			hull_edge_to_box_start := util.vector3_subtract(box_vertices[box_edge], previous_vertex);
			containment_dot := util.vector3_dot(hull_edge_plane_normal, hull_edge_to_box_start);
			maximum_vertex_containment_dots[box_edge] = max(maximum_vertex_containment_dots[box_edge], containment_dot);
			numerator := util.vector3_dot(hull_edge_to_box_start, box_edge_plane_normals[box_edge]);
			denominator := util.vector3_dot(box_edge_plane_normals[box_edge], hull_edge_offset);
			if denominator < 0
			{
				latest_entry = max(latest_entry, numerator / denominator);
			}
			else if denominator > 0
			{
				earliest_exit = min(earliest_exit, numerator / denominator);
			}
			else if numerator < 0
			{
				earliest_exit = -f32(3.402823466e+38);
				latest_entry = f32(3.402823466e+38);
			}
		}
		latest_entry = max(f32(0), latest_entry);
		earliest_exit = min(f32(1), earliest_exit);
		base_feature_id := (previous_point_index ~ point_index) << 8;
		if earliest_exit >= latest_entry && candidate_count < maximum_contact_count
		{
			point := util.vector3_subtract(
				util.vector3_add(previous_vertex, util.vector3_scale(hull_edge_offset, earliest_exit)), face_origin,
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(point, face_x), y=util.vector3_dot(point, face_y),
				feature_id=i32(base_feature_id + point_index),
			};
			candidate_count += 1;
		}
		if latest_entry < earliest_exit && latest_entry > 0 && candidate_count < maximum_contact_count
		{
			point := util.vector3_subtract(
				util.vector3_add(previous_vertex, util.vector3_scale(hull_edge_offset, latest_entry)), face_origin,
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(point, face_x), y=util.vector3_dot(point, face_y),
				feature_id=i32(base_feature_id + previous_point_index),
			};
			candidate_count += 1;
		}
		previous_point_index = point_index;
		previous_vertex = vertex;
	}

	if candidate_count < maximum_contact_count
	{
		inverse_face_normal_dot_local_normal := 1 / util.vector3_dot(face_normal, local_normal);
		for vertex_index in 0 ..< 4
		{
			if maximum_vertex_containment_dots[vertex_index] <= 0 && candidate_count < maximum_contact_count
			{
				face_to_box_vertex := util.vector3_subtract(box_vertices[vertex_index], face_origin);
				projection_t := util.vector3_dot(
					face_to_box_vertex,
					face_normal
				) * inverse_face_normal_dot_local_normal;
				projected := util.vector3_subtract(face_to_box_vertex, util.vector3_scale(local_normal, projection_t));
				candidates[candidate_count] = {
					x=util.vector3_dot(projected, face_x),
					y=util.vector3_dot(projected, face_y),
					feature_id=i32(vertex_index),
				};
				candidate_count += 1;
			}
		}
	}
	world_normal := util.matrix3x3_transform(local_normal, hull_orientation);
	return manifold_candidate_reduce(
		&candidates, candidate_count, box_face_normal,
		1 / util.vector3_dot(box_face_normal, local_normal),
		box_face_center, face_origin, face_x, face_y,
		epsilon_scale, depth_threshold, hull_orientation, offset_b, world_normal,
	);
}

collision_box_hull_build_manifold :: proc "contextless" (
	box: Box, hull: ^Convex_Hull, pose_a, pose_b: Rigid_Pose,
	local_offset_a, local_normal, closest_on_hull: util.Vector3,
	hull_local_box_orientation, hull_orientation: util.Matrix3x3,
	epsilon_scale, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	local_normal_in_a := util.matrix3x3_transform_transpose(local_normal, hull_local_box_orientation);
	abs_local_normal_in_a := util.vector3_abs(local_normal_in_a);
	face_axis := 2;
	if abs_local_normal_in_a.x > abs_local_normal_in_a.y && abs_local_normal_in_a.x > abs_local_normal_in_a.z
	{
		face_axis = 0;
	}
	else if abs_local_normal_in_a.y > abs_local_normal_in_a.z
	{
		face_axis = 1;
	}
	box_face_normal := hull_local_box_orientation.z;
	box_face_x := hull_local_box_orientation.x;
	box_face_y := hull_local_box_orientation.y;
	box_face_half_width := box.half_width;
	box_face_half_height := box.half_height;
	box_face_normal_offset := box.half_length;
	negate_face := local_normal_in_a.z > 0;
	if face_axis == 0
	{
		box_face_normal = hull_local_box_orientation.x;
		box_face_x = hull_local_box_orientation.y;
		box_face_y = hull_local_box_orientation.z;
		box_face_half_width = box.half_height;
		box_face_half_height = box.half_length;
		box_face_normal_offset = box.half_width;
		negate_face = local_normal_in_a.x > 0;
	}
	else if face_axis == 1
	{
		box_face_normal = hull_local_box_orientation.y;
		box_face_x = hull_local_box_orientation.z;
		box_face_y = hull_local_box_orientation.x;
		box_face_half_width = box.half_length;
		box_face_half_height = box.half_width;
		box_face_normal_offset = box.half_height;
		negate_face = local_normal_in_a.y > 0;
	}
	if negate_face
	{
		box_face_normal = util.vector3_negate(box_face_normal);
	}
	else
	{
		box_face_x = util.vector3_negate(box_face_x);
	}
	box_face_center := util.vector3_add(local_offset_a, util.vector3_scale(box_face_normal, box_face_normal_offset));
	box_face_x_offset := util.vector3_scale(box_face_x, box_face_half_width);
	box_face_y_offset := util.vector3_scale(box_face_y, box_face_half_height);
	v_0 := util.vector3_subtract(box_face_center, box_face_x_offset);
	v_1 := util.vector3_add(box_face_center, box_face_x_offset);
	box_vertices := [4]util.Vector3{
		util.vector3_subtract(v_0, box_face_y_offset),
		util.vector3_subtract(v_1, box_face_y_offset),
		util.vector3_add(v_1, box_face_y_offset),
		util.vector3_add(v_0, box_face_y_offset),
	};
	box_edge_directions := [4]util.Vector3{
		box_face_x, box_face_y, util.vector3_negate(box_face_x), util.vector3_negate(box_face_y),
	};
	box_edge_plane_normals: [4]util.Vector3;
	for edge_index in 0 ..< 4
	{
		box_edge_plane_normals[edge_index] = util.vector3_cross(box_edge_directions[edge_index], local_normal);
	}
	face_normal, face_index, face_status := convex_hull_pick_representative_face(
		hull, local_normal, closest_on_hull, 1e-3 * epsilon_scale,
	);
	if face_status != .Ok
	{
		return {}, face_status;
	}
	face_start, face_end, face_range_status := convex_hull_face_range(hull, face_index);
	if face_range_status != .Ok
	{
		return {}, face_range_status;
	}
	face_vertex_count := face_end - face_start;
	maximum_contact_count := max(8, face_vertex_count);
	if maximum_contact_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}
	face_x, face_y := collision_build_orthonormal_basis(face_normal);
	previous_point_index := int(hull.face_vertex_indices.memory[face_end - 1]);
	face_origin := convex_hull_get_point(hull, previous_point_index);
	previous_vertex := face_origin;
	maximum_vertex_containment_dots: [4]f32;
	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
	candidate_count := 0;
	for face_vertex in 0 ..< face_vertex_count
	{
		point_index := int(hull.face_vertex_indices.memory[face_start + face_vertex]);
		vertex := convex_hull_get_point(hull, point_index);
		hull_edge_offset := util.vector3_subtract(vertex, previous_vertex);
		hull_edge_plane_normal := util.vector3_cross(hull_edge_offset, local_normal);
		latest_entry := -f32(3.402823466e+38);
		earliest_exit := f32(3.402823466e+38);
		for box_edge in 0 ..< 4
		{
			hull_edge_to_box_start := util.vector3_subtract(box_vertices[box_edge], previous_vertex);
			containment_dot := util.vector3_dot(hull_edge_plane_normal, hull_edge_to_box_start);
			maximum_vertex_containment_dots[box_edge] = max(maximum_vertex_containment_dots[box_edge], containment_dot);
			numerator := util.vector3_dot(hull_edge_to_box_start, box_edge_plane_normals[box_edge]);
			denominator := util.vector3_dot(box_edge_plane_normals[box_edge], hull_edge_offset);
			if denominator < 0
			{
				latest_entry = max(latest_entry, numerator / denominator);
			}
			else if denominator > 0
			{
				earliest_exit = min(earliest_exit, numerator / denominator);
			}
			else if numerator < 0
			{
				earliest_exit = -f32(3.402823466e+38);
				latest_entry = f32(3.402823466e+38);
			}
		}
		latest_entry = max(f32(0), latest_entry);
		earliest_exit = min(f32(1), earliest_exit);
		base_feature_id := (previous_point_index ~ point_index) << 8;
		if earliest_exit >= latest_entry && candidate_count < maximum_contact_count
		{
			point := util.vector3_subtract(
				util.vector3_add(previous_vertex, util.vector3_scale(hull_edge_offset, earliest_exit)), face_origin,
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(point, face_x), y=util.vector3_dot(point, face_y),
				feature_id=i32(base_feature_id + point_index),
			};
			candidate_count += 1;
		}
		if latest_entry < earliest_exit && latest_entry > 0 && candidate_count < maximum_contact_count
		{
			point := util.vector3_subtract(
				util.vector3_add(previous_vertex, util.vector3_scale(hull_edge_offset, latest_entry)), face_origin,
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(point, face_x), y=util.vector3_dot(point, face_y),
				feature_id=i32(base_feature_id + previous_point_index),
			};
			candidate_count += 1;
		}
		previous_point_index = point_index;
		previous_vertex = vertex;
	}
	if candidate_count < maximum_contact_count
	{
		inverse_face_normal_dot_local_normal := 1 / util.vector3_dot(face_normal, local_normal);
		for vertex_index in 0 ..< 4
		{
			if maximum_vertex_containment_dots[vertex_index] <= 0 && candidate_count < maximum_contact_count
			{
				face_to_box_vertex := util.vector3_subtract(box_vertices[vertex_index], face_origin);
				projection_t := util.vector3_dot(
					face_to_box_vertex,
					face_normal
				) * inverse_face_normal_dot_local_normal;
				projected := util.vector3_subtract(face_to_box_vertex, util.vector3_scale(local_normal, projection_t));
				candidates[candidate_count] = {
					x=util.vector3_dot(projected, face_x), y=util.vector3_dot(projected, face_y),
					feature_id=i32(vertex_index),
				};
				candidate_count += 1;
			}
		}
	}
	world_normal := util.matrix3x3_transform(local_normal, hull_orientation);
	return manifold_candidate_reduce(
		&candidates, candidate_count, box_face_normal,
		1 / util.vector3_dot(box_face_normal, local_normal),
		box_face_center, face_origin, face_x, face_y,
		epsilon_scale, depth_threshold, hull_orientation,
		util.vector3_subtract(pose_b.position, pose_a.position), world_normal,
	);
}

box_convex_hull_test_wide :: proc "contextless" (
	box: Box_Wide, hulls: Convex_Hull_Wide, speculative_margin: util.F32x8,
	offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	inactive := ~active;
	box_orientation := util.matrix3x3_wide_from_quaternion(orientation_a);
	hull_orientation := util.matrix3x3_wide_from_quaternion(orientation_b);
	hull_local_box_orientation := util.matrix3x3_wide_multiply_by_transpose(box_orientation, hull_orientation);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, hull_orientation);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	center_distance := util.vector3_wide_length(local_offset_a);
	initial_normal := util.vector3_wide_scale(local_offset_a, simd.div(util.F32x8(1), center_distance));
	initial_normal = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(center_distance, util.F32x8(1e-8)),
		{y=util.F32x8(1)}, initial_normal,
	);
	hull_epsilon_scale: util.F32x8;
	for lane in 0 ..< pair_count
	{
		hull := hulls.hulls[lane];
		if convex_hull_validate(hull) != .Ok
		{
			return {}, .Invalid_Description;
		}
		point := convex_hull_get_point(hull, 0);
		hull_epsilon_scale = simd.replace(
			hull_epsilon_scale, lane, (abs(point.x) + abs(point.y) + abs(point.z)) * (1.0 / 3.0),
		);
	}
	epsilon_scale := simd.min(
		simd.max(box.half_width, simd.max(box.half_height, box.half_length)), hull_epsilon_scale,
	);
	depth_threshold := simd.neg(speculative_margin);
	depth_context := Box_Hull_Depth_Context_Wide{
		boxes=box, hulls=hulls, local_offset_a=local_offset_a, hull_local_r_box=hull_local_box_orientation,
	};
	depth, local_normal, closest_on_hull, depth_status := depth_refiner_find_minimum_depth_wide(
		box_hull_depth_support_wide, &depth_context, initial_normal, inactive,
		simd.mul(util.F32x8(1e-5), epsilon_scale), depth_threshold, {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	inactive |= transmute(util.I32x8)simd.lanes_lt(depth, depth_threshold);
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		box_lane := Box{
			simd.extract(box.half_width, lane), simd.extract(
				box.half_height,
				lane
			), simd.extract(box.half_length, lane),
		};
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		lane_manifold, lane_status := collision_box_hull_build_manifold(
			box_lane, hulls.hulls[lane], pose_a, pose_b,
			util.vector3_wide_read_slot(local_offset_a, lane),
			util.vector3_wide_read_slot(local_normal, lane),
			util.vector3_wide_read_slot(closest_on_hull, lane),
			util.matrix3x3_wide_read_slot(hull_local_box_orientation, lane),
			util.matrix3x3_wide_read_slot(hull_orientation, lane),
			simd.extract(epsilon_scale, lane), simd.extract(depth_threshold, lane),
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
