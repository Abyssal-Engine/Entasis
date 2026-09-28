// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

capsule_triangle_test_edge :: proc "contextless" (
	triangle: Triangle,
	triangle_normal, edge_start, edge_offset,
	capsule_center, capsule_axis: util.Vector3,
	capsule_half_length: f32,
) -> (
	edge_direction: util.Vector3,
	ta, tb, b_min, b_max, depth: f32,
	normal: util.Vector3,
)
{
	edge_length := util.vector3_length(edge_offset);
	edge_direction = util.vector3_scale(edge_offset, 1 / edge_length);
	offset_b := util.vector3_subtract(edge_start, capsule_center);
	da_offset_b := util.vector3_dot(capsule_axis, offset_b);
	db_offset_b := util.vector3_dot(edge_direction, offset_b);
	dadb := util.vector3_dot(capsule_axis, edge_direction);
	ta = (da_offset_b - db_offset_b * dadb) / max(f32(1e-15), 1 - dadb * dadb);
	tb = ta * dadb - db_offset_b;

	ta_0 := max(-capsule_half_length, min(capsule_half_length, da_offset_b));
	ta_1 := min(capsule_half_length, max(-capsule_half_length, da_offset_b + edge_length * dadb));
	a_min := min(ta_0, ta_1);
	a_max := max(ta_0, ta_1);
	a_onto_b_offset := capsule_half_length * abs(dadb);
	b_min = max(f32(0), min(edge_length, -a_onto_b_offset - db_offset_b));
	b_max = min(edge_length, max(f32(0), a_onto_b_offset - db_offset_b));
	ta = min(max(ta, a_min), a_max);
	tb = min(max(tb, b_min), b_max);

	closest_on_capsule := util.vector3_add(capsule_center, util.vector3_scale(capsule_axis, ta));
	closest_on_edge := util.vector3_add(edge_start, util.vector3_scale(edge_direction, tb));
	normal = util.vector3_subtract(closest_on_capsule, closest_on_edge);
	normal_length_squared := util.vector3_length_squared(normal);
	fallback_normal := util.vector3_cross(capsule_axis, edge_offset);
	if util.vector3_dot(fallback_normal, capsule_center) < 0
	{
		fallback_normal = util.vector3_negate(fallback_normal);
	}
	fallback_normal_length_squared := util.vector3_length_squared(fallback_normal);
	if normal_length_squared < 1e-13
	{
		normal = fallback_normal;
		normal_length_squared = fallback_normal_length_squared;
	}
	second_fallback_normal := util.vector3_cross(triangle_normal, edge_offset);
	second_fallback_normal_length_squared := util.vector3_length_squared(second_fallback_normal);
	if normal_length_squared < 1e-13
	{
		normal = second_fallback_normal;
		normal_length_squared = second_fallback_normal_length_squared;
	}
	normal = util.vector3_scale(normal, 1 / math.sqrt(normal_length_squared));

	n_axis := util.vector3_dot(capsule_axis, normal);
	n_capsule_center := util.vector3_dot(normal, capsule_center);
	extreme_on_capsule := n_capsule_center - abs(n_axis) * capsule_half_length;
	extreme_on_triangle := max(
		util.vector3_dot(triangle.a, normal),
		max(util.vector3_dot(triangle.b, normal), util.vector3_dot(triangle.c, normal)),
	);
	depth = extreme_on_triangle - extreme_on_capsule;
	return;
}

capsule_triangle_clip_edge_plane :: proc "contextless" (
	edge_start, edge_offset, face_normal, capsule_center, capsule_axis: util.Vector3,
) -> (entry, exit: f32)
{
	edge_plane_normal := util.vector3_cross(face_normal, edge_offset);
	edge_to_capsule := util.vector3_subtract(capsule_center, edge_start);
	distance := util.vector3_dot(edge_to_capsule, edge_plane_normal);
	velocity := util.vector3_dot(capsule_axis, edge_plane_normal);
	velocity_positive := velocity > 0;
	numerator := distance;
	if velocity_positive
	{
		numerator = -distance;
	}
	t := numerator / max(f32(1e-15), abs(velocity));
	entry = -f32(math.F32_MAX);
	exit = f32(math.F32_MAX);
	if velocity_positive
	{
		exit = t;
	}
	else
	{
		entry = t;
	}
	return;
}

capsule_triangle_test_source :: proc "contextless" (
	a: Capsule, b: Triangle, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if capsule_validate(a) != .Ok || triangle_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	local_triangle_center := util.vector3_scale(util.vector3_add(b.a, util.vector3_add(b.b, b.c)), 1.0 / 3.0);
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.vector3_add(util.matrix3x3_transform_transpose(offset_b, r_b), local_triangle_center);
	local_offset_a := util.vector3_negate(local_offset_b);
	triangle := Triangle{
		util.vector3_subtract(b.a, local_triangle_center),
		util.vector3_subtract(b.b, local_triangle_center),
		util.vector3_subtract(b.c, local_triangle_center),
	};
	world_capsule_axis := util.quaternion_transform_unit_y(pose_a.orientation);
	local_capsule_axis := util.matrix3x3_transform_transpose(world_capsule_axis, r_b);

	ac := util.vector3_subtract(b.c, b.a);
	ab := util.vector3_subtract(b.b, b.a);
	ac_cross_ab := util.vector3_cross(ac, ab);
	face_normal_length := util.vector3_length(ac_cross_ab);
	face_normal := util.vector3_scale(ac_cross_ab, 1 / face_normal_length);
	n_dot_axis := util.vector3_dot(face_normal, local_capsule_axis);
	capsule_offset_along_normal := util.vector3_dot(face_normal, local_offset_a);
	face_depth := a.half_length * abs(n_dot_axis) - capsule_offset_along_normal;

	edge_direction, ta, tb, b_min, b_max, edge_depth, edge_normal := capsule_triangle_test_edge(
		triangle, face_normal, triangle.a, ab, local_offset_a, local_capsule_axis, a.half_length,
	);
	candidate_direction, candidate_ta, candidate_tb, candidate_min, candidate_max, candidate_depth, candidate_normal :=
		capsule_triangle_test_edge(
		triangle, face_normal, triangle.a, ac, local_offset_a, local_capsule_axis, a.half_length,
	);
	if candidate_depth < edge_depth
	{
		edge_direction = candidate_direction;
		ta = candidate_ta;
		tb = candidate_tb;
		b_min = candidate_min;
		b_max = candidate_max;
		edge_depth = candidate_depth;
		edge_normal = candidate_normal;
	}
	edge_start := triangle.a;
	bc := util.vector3_subtract(b.c, b.b);
	candidate_direction, candidate_ta, candidate_tb, candidate_min, candidate_max, candidate_depth, candidate_normal =
		capsule_triangle_test_edge(
		triangle, face_normal, triangle.b, bc, local_offset_a, local_capsule_axis, a.half_length,
	);
	if candidate_depth < edge_depth
	{
		edge_start = triangle.b;
		edge_direction = candidate_direction;
		ta = candidate_ta;
		tb = candidate_tb;
		b_min = candidate_min;
		b_max = candidate_max;
		edge_depth = candidate_depth;
		edge_normal = candidate_normal;
	}

	depth := min(edge_depth, face_depth);
	use_edge := edge_depth < face_depth;
	local_normal := face_normal;
	if use_edge
	{
		local_normal = edge_normal;
	}
	local_normal_dot_face_normal := util.vector3_dot(local_normal, face_normal);
	negative_margin := -speculative_margin;
	if depth + a.radius < negative_margin ||
		local_normal_dot_face_normal < TRIANGLE_BACKFACE_REJECTION_THRESHOLD ||
		face_normal_length < 1e-7
	{
		return {offset_b=offset_b}, .Ok;
	}

	b_0, b_1: util.Vector3;
	contact_count := 0;
	if use_edge
	{
		plane_normal := util.vector3_cross(edge_direction, edge_normal);
		plane_normal_length_squared := util.vector3_length_squared(plane_normal);
		numerator_unsquared := util.vector3_dot(local_capsule_axis, plane_normal);
		squared_angle := f32(0);
		if plane_normal_length_squared >= 1e-10
		{
			squared_angle = numerator_unsquared * numerator_unsquared / plane_normal_length_squared;
		}
		lower_threshold := f32(0.01 * 0.01);
		upper_threshold := f32(0.05 * 0.05);
		interval_weight := max(
			f32(0),
			min(f32(1), (upper_threshold - squared_angle) / (upper_threshold - lower_threshold))
		);
		weighted_tb := tb - tb * interval_weight;
		b_min = interval_weight * b_min + weighted_tb;
		b_max = interval_weight * b_max + weighted_tb;
		b_0 = util.vector3_add(edge_start, util.vector3_scale(edge_direction, b_min));
		b_1 = util.vector3_add(edge_start, util.vector3_scale(edge_direction, b_max));
		contact_count = 1;
		if b_max > b_min
		{
			contact_count = 2;
		}
	}

	if contact_count <= 1
	{
		ab_entry, ab_exit := capsule_triangle_clip_edge_plane(
			triangle.a, ab, face_normal, local_offset_a, local_capsule_axis,
		);
		bc_entry, bc_exit := capsule_triangle_clip_edge_plane(
			triangle.b, bc, face_normal, local_offset_a, local_capsule_axis,
		);
		ca := util.vector3_negate(ac);
		ca_entry, ca_exit := capsule_triangle_clip_edge_plane(
			triangle.a, ca, face_normal, local_offset_a, local_capsule_axis,
		);
		triangle_interval_min := max(ab_entry, max(bc_entry, ca_entry));
		triangle_interval_max := min(ab_exit, min(bc_exit, ca_exit));
		overlap_interval_min := max(triangle_interval_min, -a.half_length);
		overlap_interval_max := min(triangle_interval_max, a.half_length);
		interval_valid_for_second := overlap_interval_max >= overlap_interval_min;
		overlap_interval_min = min(overlap_interval_min, a.half_length);
		overlap_interval_max = max(overlap_interval_max, -a.half_length);
		clipped_a_0 := util.vector3_add(local_offset_a, util.vector3_scale(local_capsule_axis, overlap_interval_min));
		distance_along_normal_a_0 := util.vector3_dot(clipped_a_0, face_normal);
		face_candidate_0 := util.vector3_subtract(
			clipped_a_0,
			util.vector3_scale(face_normal, distance_along_normal_a_0)
		);
		clipped_a_1 := util.vector3_add(local_offset_a, util.vector3_scale(local_capsule_axis, overlap_interval_max));
		distance_along_normal_a_1 := util.vector3_dot(clipped_a_1, face_normal);
		face_candidate_1 := util.vector3_subtract(
			clipped_a_1,
			util.vector3_scale(face_normal, distance_along_normal_a_1)
		);
		if contact_count == 0 && capsule_offset_along_normal >= 0
		{
			b_0 = face_candidate_0;
			b_1 = face_candidate_1;
			contact_count = 2;
		}
		if contact_count == 1
		{
			use_face_contact_1 := abs(overlap_interval_max - ta) > abs(overlap_interval_min - ta);
			second_contact := face_candidate_0;
			second_distance := distance_along_normal_a_0;
			if use_face_contact_1
			{
				second_contact = face_candidate_1;
				second_distance = distance_along_normal_a_1;
			}
			if interval_valid_for_second && second_distance > 0
			{
				b_1 = second_contact;
				contact_count = 2;
			}
		}
	}

	capsule_tangent := util.vector3_cross(local_normal, local_capsule_axis);
	face_normal_a := util.vector3_cross(capsule_tangent, local_capsule_axis);
	face_normal_a_dot_local_normal := util.vector3_dot(face_normal_a, local_normal);
	inverse_face_dot := 1 / face_normal_a_dot_local_normal;
	t_0 := util.vector3_dot(util.vector3_add(local_offset_b, b_0), face_normal_a) * inverse_face_dot;
	t_1 := util.vector3_dot(util.vector3_add(local_offset_b, b_1), face_normal_a) * inverse_face_dot;
	depth_0 := a.radius + t_0;
	depth_1 := a.radius + t_1;
	collapse := abs(face_normal_a_dot_local_normal) < 1e-7;
	if collapse
	{
		depth_0 = a.radius + depth;
	}

	local_offset_a_0 := util.vector3_subtract(b_0, local_offset_a);
	local_offset_a_1 := util.vector3_subtract(b_1, local_offset_a);
	ta_0 := util.vector3_dot(local_offset_a_0, local_capsule_axis);
	ta_1 := util.vector3_dot(local_offset_a_1, local_capsule_axis);
	feature_0, feature_1 := i32(0), i32(1);
	if ta_1 < ta_0
	{
		feature_0, feature_1 = 1, 0;
	}
	if local_normal_dot_face_normal >= MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION
	{
		feature_0 += MESH_REDUCTION_FACE_COLLISION_FLAG;
	}

	manifold := Convex_Contact_Manifold{
		offset_b=offset_b,
		normal=util.matrix3x3_transform(local_normal, r_b),
	};
	if contact_count > 0 && depth_0 > negative_margin
	{
		manifold.contacts[manifold.count] = {
			offset=util.matrix3x3_transform(local_offset_a_0, r_b), depth=depth_0, feature_id=feature_0,
		};
		manifold.count += 1;
	}
	if contact_count == 2 && !collapse && depth_1 > negative_margin
	{
		manifold.contacts[manifold.count] = {
			offset=util.matrix3x3_transform(local_offset_a_1, r_b), depth=depth_1, feature_id=feature_1,
		};
		manifold.count += 1;
	}
	return manifold, .Ok;
}
