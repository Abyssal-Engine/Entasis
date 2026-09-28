// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Collision_Clip_Vertex :: struct
{
	position:   util.Vector3,
	feature_id: i32,
}

Collision_Clip_Polygon :: struct
{
	vertices: [16]Collision_Clip_Vertex,
	count:    int,
}

collision_clip_polygon_against_plane :: proc "contextless" (
	polygon: ^Collision_Clip_Polygon, plane_normal: util.Vector3, plane_offset: f32,
) -> Physics_Status
{
	if polygon == nil || polygon.count < 0 || polygon.count > len(polygon.vertices)
	{
		return .Invalid_Argument;
	}
	if polygon.count == 0
	{
		return .Ok;
	}
	input := polygon^;
	polygon.count = 0;
	previous := input.vertices[input.count - 1];
	previous_distance := util.vector3_dot(previous.position, plane_normal) - plane_offset;
	previous_inside := previous_distance <= 0;
	for index in 0 ..< input.count
	{
		current := input.vertices[index];
		current_distance := util.vector3_dot(current.position, plane_normal) - plane_offset;
		current_inside := current_distance <= 0;
		if current_inside != previous_inside
		{
			denominator := previous_distance - current_distance;
			if abs(denominator) > 1e-20
			{
				t := previous_distance / denominator;
				if polygon.count >= len(polygon.vertices)
				{
					return .Capacity_Missing;
				}
				polygon.vertices[polygon.count] = {
					position=util.vector3_add(
						previous.position,
						util.vector3_scale(util.vector3_subtract(current.position, previous.position), t),
					),
					feature_id=(previous.feature_id ~ current.feature_id) | (i32(1) << 30),
				};
				polygon.count += 1;
			}
		}
		if current_inside
		{
			if polygon.count >= len(polygon.vertices)
			{
				return .Capacity_Missing;
			}
			polygon.vertices[polygon.count] = current;
			polygon.count += 1;
		}
		previous = current;
		previous_distance = current_distance;
		previous_inside = current_inside;
	}
	return .Ok;
}

collision_box_projection_radius :: proc "contextless" (
	box: Box, axes: [3]util.Vector3, axis: util.Vector3,
) -> f32
{
	return box.half_width * abs(util.vector3_dot(axes[0], axis)) +
		box.half_height * abs(util.vector3_dot(axes[1], axis)) +
		box.half_length * abs(util.vector3_dot(axes[2], axis));
}

collision_box_face_polygon :: proc "contextless" (
	box: Box, pose: Rigid_Pose, axes: [3]util.Vector3,
	face_axis: int, outward_normal: util.Vector3, feature_base: i32,
) -> Collision_Clip_Polygon
{
	extents := [3]f32{box.half_width, box.half_height, box.half_length};
	face_sign := f32(1);
	if util.vector3_dot(axes[face_axis], outward_normal) < 0
	{
		face_sign = -1;
	}
	face_center := util.vector3_add(pose.position, util.vector3_scale(axes[face_axis], face_sign * extents[face_axis]));
	side_0 := (face_axis + 1) % 3;
	side_1 := (face_axis + 2) % 3;
	polygon: Collision_Clip_Polygon;
	signs := [4][2]f32{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
	for index in 0 ..< 4
	{
		polygon.vertices[index] = {
			position=util.vector3_add(
				face_center,
				util.vector3_add(
				util.vector3_scale(axes[side_0], signs[index][0] * extents[side_0]),
				util.vector3_scale(axes[side_1], signs[index][1] * extents[side_1]),
			),
			),
			feature_id=feature_base | i32(index),
		};
	}
	polygon.count = 4;
	return polygon;
}

box_pair_test_source :: proc "contextless" (
	a, b: Box, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if box_validate(a) != .Ok || box_validate(b) != .Ok || speculative_margin < 0
	{
		return {}, .Invalid_Description;
	}
	a_wide, b_wide: Box_Wide;
	_ = box_wide_write_slot(&a_wide, 0, a);
	_ = box_wide_write_slot(&b_wide, 0, b);
	offset_b: util.Vector3_Wide;
	util.vector3_wide_write_slot(&offset_b, 0, util.vector3_subtract(pose_b.position, pose_a.position));
	orientation_a, orientation_b: util.Quaternion_Wide;
	util.quaternion_wide_write_slot(&orientation_a, 0, pose_a.orientation);
	util.quaternion_wide_write_slot(&orientation_b, 0, pose_b.orientation);
	wide, status := box_pair_test_wide(
		a_wide, b_wide, util.F32x8(speculative_margin), offset_b, orientation_a, orientation_b, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_4_manifold_wide_read_lane(&wide, offset_b, 0);
}

collision_box_projection_radius_wide :: proc "contextless" (
	box: Box_Wide, axes: [3]util.Vector3_Wide, axis: util.Vector3_Wide,
) -> util.F32x8
{
	return simd.add(
		simd.add(
		simd.mul(box.half_width, simd.abs(util.vector3_wide_dot(axes[0], axis))),
		simd.mul(box.half_height, simd.abs(util.vector3_wide_dot(axes[1], axis))),
	),
		simd.mul(box.half_length, simd.abs(util.vector3_wide_dot(axes[2], axis))),
	);
}

collision_box_pair_select_axis_wide :: proc "contextless" (
	best_depth: ^util.F32x8, best_normal: ^util.Vector3_Wide,
	depth: util.F32x8, normal: util.Vector3_Wide,
)
{
	use_candidate := transmute(util.I32x8)simd.lanes_lt(depth, best_depth^);
	best_depth^ = util.wide_select_f32(use_candidate, depth, best_depth^);
	best_normal^ = util.vector3_wide_select(use_candidate, normal, best_normal^);
}

collision_box_pair_test_edge_edge_wide :: proc "contextless" (
	a, b: Box_Wide, local_offset_b: util.Vector3_Wide,
	r_b: util.Matrix3x3_Wide, edge_b_direction: util.Vector3_Wide,
) -> (util.F32x8, util.Vector3_Wide)
{
	x_squared := simd.mul(edge_b_direction.x, edge_b_direction.x);
	y_squared := simd.mul(edge_b_direction.y, edge_b_direction.y);
	z_squared := simd.mul(edge_b_direction.z, edge_b_direction.z);
	maximum_depth := util.F32x8(3.402823466e+38);
	zero := util.F32x8(0);

	length := simd.sqrt(simd.add(y_squared, z_squared));
	inverse_length := simd.div(util.F32x8(1), length);
	normal := util.Vector3_Wide{
		y=simd.mul(edge_b_direction.z, inverse_length),
		z=simd.neg(simd.mul(edge_b_direction.y, inverse_length)),
	};
	extreme_a := simd.add(simd.mul(simd.abs(normal.y), a.half_height), simd.mul(simd.abs(normal.z), a.half_length));
	n_bx := simd.add(simd.mul(normal.y, r_b.x.y), simd.mul(normal.z, r_b.x.z));
	n_by := simd.add(simd.mul(normal.y, r_b.y.y), simd.mul(normal.z, r_b.y.z));
	n_bz := simd.add(simd.mul(normal.y, r_b.z.y), simd.mul(normal.z, r_b.z.z));
	extreme_b := simd.add(
		simd.add(simd.mul(simd.abs(n_bx), b.half_width), simd.mul(simd.abs(n_by), b.half_height)),
		simd.mul(simd.abs(n_bz), b.half_length),
	);
	depth := simd.sub(
		simd.add(extreme_a, extreme_b),
		simd.abs(simd.add(simd.mul(local_offset_b.y, normal.y), simd.mul(local_offset_b.z, normal.z))),
	);
	depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-7)), maximum_depth, depth,
	);

	length = simd.sqrt(simd.add(x_squared, z_squared));
	inverse_length = simd.div(util.F32x8(1), length);
	candidate_normal := util.Vector3_Wide{
		x=simd.mul(edge_b_direction.z, inverse_length),
		z=simd.neg(simd.mul(edge_b_direction.x, inverse_length)),
	};
	extreme_a = simd.add(
		simd.mul(simd.abs(candidate_normal.x), a.half_width),
		simd.mul(simd.abs(candidate_normal.z), a.half_length)
	);
	n_bx = simd.add(simd.mul(candidate_normal.x, r_b.x.x), simd.mul(candidate_normal.z, r_b.x.z));
	n_by = simd.add(simd.mul(candidate_normal.x, r_b.y.x), simd.mul(candidate_normal.z, r_b.y.z));
	n_bz = simd.add(simd.mul(candidate_normal.x, r_b.z.x), simd.mul(candidate_normal.z, r_b.z.z));
	extreme_b = simd.add(
		simd.add(simd.mul(simd.abs(n_bx), b.half_width), simd.mul(simd.abs(n_by), b.half_height)),
		simd.mul(simd.abs(n_bz), b.half_length),
	);
	candidate_depth := simd.sub(
		simd.add(extreme_a, extreme_b),
		simd.abs(simd.add(
		simd.mul(local_offset_b.x, candidate_normal.x),
		simd.mul(local_offset_b.z, candidate_normal.z)
	)),
	);
	candidate_depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-7)), maximum_depth, candidate_depth,
	);
	collision_box_pair_select_axis_wide(&depth, &normal, candidate_depth, candidate_normal);

	length = simd.sqrt(simd.add(x_squared, y_squared));
	inverse_length = simd.div(util.F32x8(1), length);
	candidate_normal = {
		x=simd.mul(edge_b_direction.y, inverse_length),
		y=simd.neg(simd.mul(edge_b_direction.x, inverse_length)),
		z=zero,
	};
	extreme_a = simd.add(
		simd.mul(simd.abs(candidate_normal.x), a.half_width),
		simd.mul(simd.abs(candidate_normal.y), a.half_height)
	);
	n_bx = simd.add(simd.mul(candidate_normal.x, r_b.x.x), simd.mul(candidate_normal.y, r_b.x.y));
	n_by = simd.add(simd.mul(candidate_normal.x, r_b.y.x), simd.mul(candidate_normal.y, r_b.y.y));
	n_bz = simd.add(simd.mul(candidate_normal.x, r_b.z.x), simd.mul(candidate_normal.y, r_b.z.y));
	extreme_b = simd.add(
		simd.add(simd.mul(simd.abs(n_bx), b.half_width), simd.mul(simd.abs(n_by), b.half_height)),
		simd.mul(simd.abs(n_bz), b.half_length),
	);
	candidate_depth = simd.sub(
		simd.add(extreme_a, extreme_b),
		simd.abs(simd.add(
		simd.mul(local_offset_b.x, candidate_normal.x),
		simd.mul(local_offset_b.y, candidate_normal.y)
	)),
	);
	candidate_depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-7)), maximum_depth, candidate_depth,
	);
	collision_box_pair_select_axis_wide(&depth, &normal, candidate_depth, candidate_normal);
	return depth, normal;
}

collision_box_pair_add_candidate_wide :: proc "contextless" (
	candidates: ^[8]Manifold_Candidate_Wide, count: ^util.I32x8, initialized_count: ^int,
	candidate: Manifold_Candidate_Wide, new_contact_exists: util.I32x8, pair_count: int,
)
{
	_ = pair_count;
	for slot in 0 ..< len(candidates)
	{
		slot_mask := new_contact_exists &
			transmute(util.I32x8)simd.lanes_eq(count^, util.I32x8(i32(slot)));
		if simd.reduce_or(slot_mask) == 0
		{
			continue;
		}
		if slot == initialized_count^
		{
			candidates[slot] = candidate;
			initialized_count^ += 1;
		}
		else
		{
			candidates[slot].x = util.wide_select_f32(
				slot_mask, candidate.x, candidates[slot].x,
			);
			candidates[slot].y = util.wide_select_f32(
				slot_mask, candidate.y, candidates[slot].y,
			);
			candidates[slot].depth = util.wide_select_f32(
				slot_mask, candidate.depth, candidates[slot].depth,
			);
			candidates[slot].feature_id = collision_wide_select_i32(
				slot_mask, candidate.feature_id, candidates[slot].feature_id,
			);
		}
	}
	count^ = collision_wide_select_i32(new_contact_exists, simd.add(count^, util.I32x8(1)), count^);
}

collision_box_pair_add_a_vertex_wide :: proc "contextless" (
	vertex: util.Vector3_Wide, feature_id: util.I32x8,
	face_normal_b, contact_normal: util.Vector3_Wide, inverse_contact_normal_dot_face_normal_b: util.F32x8,
	face_center_b, face_tangent_bx, face_tangent_by: util.Vector3_Wide,
	half_span_bx, half_span_by: util.F32x8,
	candidates: ^[8]Manifold_Candidate_Wide, candidate_count: ^util.I32x8,
	initialized_candidate_count: ^int, pair_count: int, allow_contacts: util.I32x8,
)
{
	point_on_b_to_vertex := util.vector3_wide_subtract(vertex, face_center_b);
	plane_distance := util.vector3_wide_dot(face_normal_b, point_on_b_to_vertex);
	offset := util.vector3_wide_scale(
		contact_normal,
		simd.mul(plane_distance, inverse_contact_normal_dot_face_normal_b)
	);
	vertex_on_b_face := util.vector3_wide_subtract(vertex, offset);
	vertex_offset_on_b_face := util.vector3_wide_subtract(vertex_on_b_face, face_center_b);
	candidate := Manifold_Candidate_Wide{
		x=util.vector3_wide_dot(vertex_offset_on_b_face, face_tangent_bx),
		y=util.vector3_wide_dot(vertex_offset_on_b_face, face_tangent_by),
		feature_id=feature_id,
	};
	contained := transmute(util.I32x8)simd.lanes_le(simd.abs(candidate.x), half_span_bx) &
		transmute(util.I32x8)simd.lanes_le(simd.abs(candidate.y), half_span_by);
	below_capacity := transmute(util.I32x8)simd.lanes_lt(candidate_count^, util.I32x8(8));
	collision_box_pair_add_candidate_wide(
		candidates, candidate_count, initialized_candidate_count, candidate,
		allow_contacts & contained & below_capacity, pair_count,
	);
}

collision_box_pair_add_a_vertices_wide :: proc "contextless" (
	face_center_b, face_tangent_bx, face_tangent_by: util.Vector3_Wide,
	half_span_bx, half_span_by: util.F32x8, face_normal_b, contact_normal: util.Vector3_Wide,
	vertices: [4]util.Vector3_Wide, feature_ids: [4]util.I32x8,
	candidates: ^[8]Manifold_Candidate_Wide, candidate_count: ^util.I32x8,
	initialized_candidate_count: ^int, pair_count: int, allow_contacts: util.I32x8,
)
{
	normal_dot := util.vector3_wide_dot(face_normal_b, contact_normal);
	inverse_dot := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(simd.abs(normal_dot), util.F32x8(1e-10)),
		simd.div(util.F32x8(1), normal_dot), util.F32x8(3.402823466e+38),
	);
	for vertex_index in 0 ..< 4
	{
		collision_box_pair_add_a_vertex_wide(
			vertices[vertex_index], feature_ids[vertex_index], face_normal_b, contact_normal, inverse_dot,
			face_center_b, face_tangent_bx, face_tangent_by, half_span_bx, half_span_by,
			candidates, candidate_count, initialized_candidate_count, pair_count, allow_contacts,
		);
	}
}

collision_box_pair_clip_edge_plane_wide :: proc "contextless" (
	edge_direction: util.Vector3_Wide,
	edge_start_b0_to_a00, edge_start_b0_to_a11, edge_start_b1_to_a00, edge_start_b1_to_a11: util.Vector3_Wide,
	box_edge_plane_normal: util.Vector3_Wide,
) -> (util.F32x8, util.F32x8, util.F32x8, util.F32x8)
{
	distance_00 := util.vector3_wide_dot(edge_start_b0_to_a00, box_edge_plane_normal);
	distance_01 := util.vector3_wide_dot(edge_start_b0_to_a11, box_edge_plane_normal);
	distance_10 := util.vector3_wide_dot(edge_start_b1_to_a00, box_edge_plane_normal);
	distance_11 := util.vector3_wide_dot(edge_start_b1_to_a11, box_edge_plane_normal);
	velocity := util.vector3_wide_dot(box_edge_plane_normal, edge_direction);
	inverse_velocity := simd.div(util.F32x8(1), velocity);
	inside_0 := transmute(util.I32x8)simd.lanes_le(simd.mul(distance_00, distance_01), util.F32x8(0));
	inside_1 := transmute(util.I32x8)simd.lanes_le(simd.mul(distance_10, distance_11), util.F32x8(0));
	nonparallel := transmute(util.I32x8)simd.lanes_gt(simd.abs(velocity), util.F32x8(1e-15));
	large_negative, large_positive := util.F32x8(-3.402823466e+38), util.F32x8(3.402823466e+38);
	t00, t01 := simd.mul(distance_00, inverse_velocity), simd.mul(distance_01, inverse_velocity);
	t10, t11 := simd.mul(distance_10, inverse_velocity), simd.mul(distance_11, inverse_velocity);
	min_0 := util.wide_select_f32(
		nonparallel,
		simd.min(t00, t01),
		util.wide_select_f32(inside_0, large_negative, large_positive)
	);
	max_0 := util.wide_select_f32(
		nonparallel,
		simd.max(t00, t01),
		util.wide_select_f32(inside_0, large_positive, large_negative)
	);
	min_1 := util.wide_select_f32(
		nonparallel,
		simd.min(t10, t11),
		util.wide_select_f32(inside_1, large_negative, large_positive)
	);
	max_1 := util.wide_select_f32(
		nonparallel,
		simd.max(t10, t11),
		util.wide_select_f32(inside_1, large_positive, large_negative)
	);
	return min_0, max_0, min_1, max_1;
}

collision_box_pair_clip_edges_wide :: proc "contextless" (
	edge_start_b0, edge_start_b1, edge_direction_b: util.Vector3_Wide, half_span_b: util.F32x8,
	vertex_a00, vertex_a11, edge_plane_normal_ax, edge_plane_normal_ay: util.Vector3_Wide,
) -> (util.F32x8, util.F32x8, util.F32x8, util.F32x8)
{
	b0_to_a00 := util.vector3_wide_subtract(vertex_a00, edge_start_b0);
	b0_to_a11 := util.vector3_wide_subtract(vertex_a11, edge_start_b0);
	b1_to_a00 := util.vector3_wide_subtract(vertex_a00, edge_start_b1);
	b1_to_a11 := util.vector3_wide_subtract(vertex_a11, edge_start_b1);
	min_x0, max_x0, min_x1, max_x1 := collision_box_pair_clip_edge_plane_wide(
		edge_direction_b, b0_to_a00, b0_to_a11, b1_to_a00, b1_to_a11, edge_plane_normal_ax,
	);
	min_y0, max_y0, min_y1, max_y1 := collision_box_pair_clip_edge_plane_wide(
		edge_direction_b, b0_to_a00, b0_to_a11, b1_to_a00, b1_to_a11, edge_plane_normal_ay,
	);
	negative_half_span := simd.neg(half_span_b);
	return simd.max(negative_half_span, simd.max(min_x0, min_y0)), simd.min(half_span_b, simd.min(max_x0, max_y0)),
	simd.max(negative_half_span, simd.max(min_x1, min_y1)), simd.min(half_span_b, simd.min(max_x1, max_y1));
}

collision_box_pair_add_edge_contacts_wide :: proc "contextless" (
	minimum: util.F32x8, minimum_candidate: Manifold_Candidate_Wide,
	maximum: util.F32x8, maximum_candidate: Manifold_Candidate_Wide,
	half_span_b, epsilon: util.F32x8,
	candidates: ^[8]Manifold_Candidate_Wide, candidate_count: ^util.I32x8,
	initialized_candidate_count: ^int, allow_contacts: util.I32x8, pair_count: int,
)
{
	minimum_exists := allow_contacts &
		transmute(util.I32x8)simd.lanes_gt(simd.sub(maximum, minimum), epsilon) &
		transmute(util.I32x8)simd.lanes_lt(simd.abs(minimum), half_span_b);
	collision_box_pair_add_candidate_wide(
		candidates, candidate_count, initialized_candidate_count, minimum_candidate, minimum_exists, pair_count,
	);
	maximum_exists := allow_contacts &
		transmute(util.I32x8)simd.lanes_ge(maximum, minimum) &
		transmute(util.I32x8)simd.lanes_le(simd.abs(maximum), half_span_b);
	collision_box_pair_add_candidate_wide(
		candidates, candidate_count, initialized_candidate_count, maximum_candidate, maximum_exists, pair_count,
	);
}

collision_box_pair_create_edge_contacts_wide :: proc "contextless" (
	face_center_b, face_tangent_bx, face_tangent_by: util.Vector3_Wide,
	half_span_bx, half_span_by: util.F32x8,
	vertex_a00, vertex_a11, face_tangent_ax, face_tangent_ay, contact_normal: util.Vector3_Wide,
	feature_id_x0, feature_id_x1, feature_id_y0, feature_id_y1: util.I32x8,
	epsilon_scale: util.F32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, initialized_candidate_count: ^int,
	pair_count: int, allow_contacts: util.I32x8,
)
{
	edge_plane_normal_ax := util.vector3_wide_cross(face_tangent_ay, contact_normal);
	edge_plane_normal_ay := util.vector3_wide_cross(face_tangent_ax, contact_normal);
	edge_offset_bx := util.vector3_wide_scale(face_tangent_by, half_span_by);
	edge_offset_by := util.vector3_wide_scale(face_tangent_bx, half_span_bx);
	edge_start_bx0 := util.vector3_wide_subtract(face_center_b, edge_offset_bx);
	edge_start_bx1 := util.vector3_wide_add(face_center_b, edge_offset_bx);
	min_x0, max_x0, unflipped_min_x1, unflipped_max_x1 := collision_box_pair_clip_edges_wide(
		edge_start_bx0, edge_start_bx1, face_tangent_bx, half_span_bx,
		vertex_a00, vertex_a11, edge_plane_normal_ax, edge_plane_normal_ay,
	);
	edge_start_by0 := util.vector3_wide_subtract(face_center_b, edge_offset_by);
	edge_start_by1 := util.vector3_wide_add(face_center_b, edge_offset_by);
	unflipped_min_y0, unflipped_max_y0, min_y1, max_y1 := collision_box_pair_clip_edges_wide(
		edge_start_by0, edge_start_by1, face_tangent_by, half_span_by,
		vertex_a00, vertex_a11, edge_plane_normal_ax, edge_plane_normal_ay,
	);
	min_x1, max_x1 := simd.neg(unflipped_max_x1), simd.neg(unflipped_min_x1);
	min_y0, max_y0 := simd.neg(unflipped_max_y0), simd.neg(unflipped_min_y0);
	epsilon := simd.mul(epsilon_scale, util.F32x8(1e-5));
	edge_feature_offset := util.I32x8(64);
	minimum, maximum: Manifold_Candidate_Wide;
	minimum = {x=min_x0, y=simd.neg(half_span_by), feature_id=feature_id_x0};
	maximum = {x=max_x0, y=simd.neg(half_span_by), feature_id=simd.add(feature_id_x0, edge_feature_offset)};
	collision_box_pair_add_edge_contacts_wide(
		min_x0,
		minimum,
		max_x0,
		maximum,
		half_span_bx,
		epsilon,
		candidates,
		candidate_count,
		initialized_candidate_count,
		allow_contacts,
		pair_count
	);
	minimum = {x=half_span_bx, y=min_y1, feature_id=feature_id_y1};
	maximum = {x=half_span_bx, y=max_y1, feature_id=simd.add(feature_id_y1, edge_feature_offset)};
	collision_box_pair_add_edge_contacts_wide(
		min_y1,
		minimum,
		max_y1,
		maximum,
		half_span_by,
		epsilon,
		candidates,
		candidate_count,
		initialized_candidate_count,
		allow_contacts,
		pair_count
	);
	minimum = {x=unflipped_max_x1, y=half_span_by, feature_id=feature_id_x1};
	maximum = {x=unflipped_min_x1, y=half_span_by, feature_id=simd.add(feature_id_x1, edge_feature_offset)};
	collision_box_pair_add_edge_contacts_wide(
		min_x1,
		minimum,
		max_x1,
		maximum,
		half_span_bx,
		epsilon,
		candidates,
		candidate_count,
		initialized_candidate_count,
		allow_contacts,
		pair_count
	);
	minimum = {x=simd.neg(half_span_bx), y=unflipped_max_y0, feature_id=feature_id_y0};
	maximum = {x=simd.neg(half_span_bx), y=unflipped_min_y0, feature_id=simd.add(feature_id_y0, edge_feature_offset)};
	collision_box_pair_add_edge_contacts_wide(
		min_y0,
		minimum,
		max_y0,
		maximum,
		half_span_by,
		epsilon,
		candidates,
		candidate_count,
		initialized_candidate_count,
		allow_contacts,
		pair_count
	);
}

box_pair_test_wide_core :: proc "contextless" (
	a, b: Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, active: util.I32x8, pair_count: int,
	manifold: ^Convex_4_Contact_Manifold_Wide,
)
{
	manifold.contact_0_exists = util.I32x8(0);
	manifold.contact_1_exists = util.I32x8(0);
	manifold.contact_2_exists = util.I32x8(0);
	manifold.contact_3_exists = util.I32x8(0);
	world_r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	world_r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	r_b := util.matrix3x3_wide_multiply_by_transpose(world_r_b, world_r_a);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_a);
	best_depth, local_normal := collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.x);
	candidate_depth, candidate_normal := collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.y);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, candidate_normal);
	candidate_depth, candidate_normal = collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.z);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, candidate_normal);

	abs_r_bx := util.vector3_wide_abs(r_b.x);
	abs_r_by := util.vector3_wide_abs(r_b.y);
	abs_r_bz := util.vector3_wide_abs(r_b.z);
	candidate_depth = simd.sub(
		simd.add(
		a.half_width,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.x),
		simd.add(simd.mul(b.half_height, abs_r_by.x), simd.mul(b.half_length, abs_r_bz.x))
	)
	),
		simd.abs(local_offset_b.x),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {x=util.F32x8(1)});
	candidate_depth = simd.sub(
		simd.add(
		a.half_height,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.y),
		simd.add(simd.mul(b.half_height, abs_r_by.y), simd.mul(b.half_length, abs_r_bz.y))
	)
	),
		simd.abs(local_offset_b.y),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {y=util.F32x8(1)});
	candidate_depth = simd.sub(
		simd.add(
		a.half_length,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.z),
		simd.add(simd.mul(b.half_height, abs_r_by.z), simd.mul(b.half_length, abs_r_bz.z))
	)
	),
		simd.abs(local_offset_b.z),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {z=util.F32x8(1)});

	b_local_offset_b := util.matrix3x3_wide_transform_transposed(local_offset_b, r_b);
	candidate_depth = simd.sub(
		simd.add(
		b.half_width,
		simd.add(
		simd.mul(a.half_width, abs_r_bx.x),
		simd.add(simd.mul(a.half_height, abs_r_bx.y), simd.mul(a.half_length, abs_r_bx.z))
	)
	),
		simd.abs(b_local_offset_b.x),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.x);
	candidate_depth = simd.sub(
		simd.add(
		b.half_height,
		simd.add(
		simd.mul(a.half_width, abs_r_by.x),
		simd.add(simd.mul(a.half_height, abs_r_by.y), simd.mul(a.half_length, abs_r_by.z))
	)
	),
		simd.abs(b_local_offset_b.y),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.y);
	candidate_depth = simd.sub(
		simd.add(
		b.half_length,
		simd.add(
		simd.mul(a.half_width, abs_r_bz.x),
		simd.add(simd.mul(a.half_height, abs_r_bz.y), simd.mul(a.half_length, abs_r_bz.z))
	)
	),
		simd.abs(b_local_offset_b.z),
	);
	collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.z);

	minimum_depth := simd.neg(speculative_margin);
	allow_contacts := active & transmute(util.I32x8)simd.lanes_ge(best_depth, minimum_depth);
	valid_lanes := transmute(util.I32x8)simd.lanes_lt(
		util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(pair_count)),
	);
	if simd.reduce_or(allow_contacts & valid_lanes) == 0
	{
		return;
	}
	local_normal = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(util.vector3_wide_dot(local_normal, local_offset_b), util.F32x8(0)),
		local_normal,
	);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, world_r_a);
	axes_a := [3]util.Vector3_Wide{world_r_a.x, world_r_a.y, world_r_a.z};
	axes_b := [3]util.Vector3_Wide{world_r_b.x, world_r_b.y, world_r_b.z};

	ax_dot := util.vector3_wide_dot(manifold.normal, axes_a[0]);
	ay_dot := util.vector3_wide_dot(manifold.normal, axes_a[1]);
	az_dot := util.vector3_wide_dot(manifold.normal, axes_a[2]);
	max_a_dot := simd.max(simd.abs(ax_dot), simd.max(simd.abs(ay_dot), simd.abs(az_dot)));
	use_ax := transmute(util.I32x8)simd.lanes_eq(max_a_dot, simd.abs(ax_dot));
	use_ay := transmute(util.I32x8)simd.lanes_eq(max_a_dot, simd.abs(ay_dot)) & ~use_ax;
	normal_a := util.vector3_wide_select(use_ay, axes_a[1], util.vector3_wide_select(use_ax, axes_a[0], axes_a[2]));
	tangent_ax := util.vector3_wide_select(use_ay, axes_a[0], util.vector3_wide_select(use_ax, axes_a[2], axes_a[1]));
	tangent_ay := util.vector3_wide_select(use_ay, axes_a[2], util.vector3_wide_select(use_ax, axes_a[1], axes_a[0]));
	half_span_ax := util.wide_select_f32(
		use_ax,
		a.half_length,
		util.wide_select_f32(use_ay, a.half_width, a.half_height)
	);
	half_span_ay := util.wide_select_f32(
		use_ax,
		a.half_height,
		util.wide_select_f32(use_ay, a.half_length, a.half_width)
	);
	half_span_az := util.wide_select_f32(
		use_ax,
		a.half_width,
		util.wide_select_f32(use_ay, a.half_height, a.half_length)
	);
	local_x_id, local_y_id, local_z_id := util.I32x8(1), util.I32x8(4), util.I32x8(16);
	axis_id_ax := collision_wide_select_i32(
		use_ax,
		local_z_id,
		collision_wide_select_i32(use_ay, local_x_id, local_y_id)
	);
	axis_id_ay := collision_wide_select_i32(
		use_ax,
		local_y_id,
		collision_wide_select_i32(use_ay, local_z_id, local_x_id)
	);
	axis_id_az := collision_wide_select_i32(
		use_ax,
		local_x_id,
		collision_wide_select_i32(use_ay, local_y_id, local_z_id)
	);

	bx_dot := util.vector3_wide_dot(manifold.normal, axes_b[0]);
	by_dot := util.vector3_wide_dot(manifold.normal, axes_b[1]);
	bz_dot := util.vector3_wide_dot(manifold.normal, axes_b[2]);
	max_b_dot := simd.max(simd.abs(bx_dot), simd.max(simd.abs(by_dot), simd.abs(bz_dot)));
	use_bx := transmute(util.I32x8)simd.lanes_eq(max_b_dot, simd.abs(bx_dot));
	use_by := transmute(util.I32x8)simd.lanes_eq(max_b_dot, simd.abs(by_dot)) & ~use_bx;
	normal_b := util.vector3_wide_select(use_by, axes_b[1], util.vector3_wide_select(use_bx, axes_b[0], axes_b[2]));
	tangent_bx := util.vector3_wide_select(use_by, axes_b[0], util.vector3_wide_select(use_bx, axes_b[2], axes_b[1]));
	tangent_by := util.vector3_wide_select(use_by, axes_b[2], util.vector3_wide_select(use_bx, axes_b[1], axes_b[0]));
	half_span_bx := util.wide_select_f32(
		use_bx,
		b.half_length,
		util.wide_select_f32(use_by, b.half_width, b.half_height)
	);
	half_span_by := util.wide_select_f32(
		use_bx,
		b.half_height,
		util.wide_select_f32(use_by, b.half_length, b.half_width)
	);
	half_span_bz := util.wide_select_f32(
		use_bx,
		b.half_width,
		util.wide_select_f32(use_by, b.half_height, b.half_length)
	);
	axis_id_bx := collision_wide_select_i32(
		use_bx,
		local_z_id,
		collision_wide_select_i32(use_by, local_x_id, local_y_id)
	);
	axis_id_by := collision_wide_select_i32(
		use_bx,
		local_y_id,
		collision_wide_select_i32(use_by, local_z_id, local_x_id)
	);
	axis_id_bz := collision_wide_select_i32(
		use_bx,
		local_x_id,
		collision_wide_select_i32(use_by, local_y_id, local_z_id)
	);

	calibration_dot_a := util.vector3_wide_dot(normal_a, manifold.normal);
	normal_a = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(calibration_dot_a, util.F32x8(0)), normal_a,
	);
	calibration_dot_b := util.vector3_wide_dot(normal_b, manifold.normal);
	normal_b = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_lt(calibration_dot_b, util.F32x8(0)), normal_b,
	);
	face_center_a := util.vector3_wide_scale(normal_a, half_span_az);
	face_center_b := util.vector3_wide_add(util.vector3_wide_scale(normal_b, half_span_bz), offset_b);
	face_center_b_to_face_center_a := util.vector3_wide_subtract(face_center_a, face_center_b);
	edge_offset_ax := util.vector3_wide_scale(tangent_ay, half_span_ay);
	edge_offset_ay := util.vector3_wide_scale(tangent_ax, half_span_ax);
	vertex_a0 := util.vector3_wide_subtract(face_center_a, edge_offset_ax);
	vertex_a00 := util.vector3_wide_subtract(vertex_a0, edge_offset_ay);
	vertex_a1 := util.vector3_wide_add(face_center_a, edge_offset_ax);
	vertex_a11 := util.vector3_wide_add(vertex_a1, edge_offset_ay);
	epsilon_scale := simd.min(
		simd.max(half_span_ax, simd.max(half_span_ay, half_span_az)),
		simd.max(half_span_bx, simd.max(half_span_by, half_span_bz)),
	);
	three := util.I32x8(3);
	axis_z_edge_id_contribution := simd.mul(axis_id_bz, three);
	edge_id_bx0 := simd.add(simd.mul(axis_id_bx, util.I32x8(2)), simd.add(axis_id_by, axis_z_edge_id_contribution));
	edge_id_bx1 := simd.add(
		simd.mul(axis_id_bx, util.I32x8(2)),
		simd.add(simd.mul(axis_id_by, three), axis_z_edge_id_contribution)
	);
	edge_id_by0 := simd.add(axis_id_bx, simd.add(simd.mul(axis_id_by, util.I32x8(2)), axis_z_edge_id_contribution));
	edge_id_by1 := simd.add(
		simd.mul(axis_id_bx, three),
		simd.add(simd.mul(axis_id_by, util.I32x8(2)), axis_z_edge_id_contribution)
	);
	candidates: [8]Manifold_Candidate_Wide = ---
		candidate_count := util.I32x8(0);
	initialized_candidate_count := 0;
	collision_box_pair_create_edge_contacts_wide(
		face_center_b, tangent_bx, tangent_by, half_span_bx, half_span_by,
		vertex_a00, vertex_a11, tangent_ax, tangent_ay, manifold.normal,
		edge_id_bx0, edge_id_bx1, edge_id_by0, edge_id_by1, epsilon_scale,
		&candidates, &candidate_count, &initialized_candidate_count, pair_count, allow_contacts,
	);
	vertex_a01 := util.vector3_wide_add(vertex_a0, edge_offset_ay);
	vertex_a10 := util.vector3_wide_subtract(vertex_a1, edge_offset_ay);
	vertices := [4]util.Vector3_Wide{vertex_a00, vertex_a01, vertex_a10, vertex_a11};
	feature_ids := [4]util.I32x8{
		simd.neg(axis_id_az), simd.neg(simd.add(axis_id_az, axis_id_ay)),
		simd.neg(simd.add(axis_id_az, axis_id_ax)),
		simd.neg(simd.add(axis_id_az, simd.add(axis_id_ax, axis_id_ay))),
	};
	collision_box_pair_add_a_vertices_wide(
		face_center_b, tangent_bx, tangent_by, half_span_bx, half_span_by, normal_b, manifold.normal,
		vertices, feature_ids, &candidates, &candidate_count, &initialized_candidate_count,
		pair_count, allow_contacts,
	);
	contacts, contact_exists := manifold_candidate_wide_reduce(
		&candidates, candidate_count, initialized_candidate_count, normal_a,
		simd.div(util.F32x8(-1), simd.abs(calibration_dot_a)),
		face_center_b_to_face_center_a, tangent_bx, tangent_by, epsilon_scale, minimum_depth, pair_count,
	);
	manifold.offset_a_0 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[0].x),
		util.vector3_wide_scale(tangent_by, contacts[0].y)
	)
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[1].x),
		util.vector3_wide_scale(tangent_by, contacts[1].y)
	)
	);
	manifold.offset_a_2 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[2].x),
		util.vector3_wide_scale(tangent_by, contacts[2].y)
	)
	);
	manifold.offset_a_3 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[3].x),
		util.vector3_wide_scale(tangent_by, contacts[3].y)
	)
	);
	manifold.depth_0, manifold.depth_1 = contacts[0].depth, contacts[1].depth;
	manifold.depth_2, manifold.depth_3 = contacts[2].depth, contacts[3].depth;
	manifold.feature_id_0, manifold.feature_id_1 = contacts[0].feature_id, contacts[1].feature_id;
	manifold.feature_id_2, manifold.feature_id_3 = contacts[2].feature_id, contacts[3].feature_id;
	manifold.contact_0_exists, manifold.contact_1_exists = contact_exists[0] & allow_contacts, contact_exists[1] & allow_contacts;
	manifold.contact_2_exists, manifold.contact_3_exists = contact_exists[2] & allow_contacts, contact_exists[3] & allow_contacts;
}

box_pair_test_wide_into :: proc "contextless" (
	a, b: Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
	manifold: ^Convex_4_Contact_Manifold_Wide,
) -> Physics_Status
{
	if manifold == nil
	{
		return .Invalid_Argument;
	}
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return status;
	}
	box_pair_test_wide_core(
		a, b, speculative_margin, offset_b, orientation_a, orientation_b,
		active, pair_count, manifold,
	);
	return .Ok;
}

box_pair_test_wide :: proc "contextless" (
	a, b: Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	manifold: Convex_4_Contact_Manifold_Wide = ---
		status := box_pair_test_wide_into(
		a, b, speculative_margin, offset_b, orientation_a, orientation_b, pair_count, &manifold,
	);
	return manifold, status;
}
