// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

capsule_cylinder_bounce :: proc "contextless" (
	line_origin, line_direction: util.Vector3, t: f32, cylinder: Cylinder, radius_squared: f32,
) -> (point, clamped: util.Vector3)
{
	point = util.vector3_add(line_origin, util.vector3_scale(line_direction, t));
	horizontal_distance_squared := point.x * point.x + point.z * point.z;
	clamped = point;
	if horizontal_distance_squared > radius_squared
	{
		clamp_scale := cylinder.radius / math.sqrt(horizontal_distance_squared);
		clamped.x = clamp_scale * point.x;
		clamped.z = clamp_scale * point.z;
	}
	clamped.y = max(-cylinder.half_length, min(cylinder.half_length, point.y));
	return;
}

capsule_cylinder_closest_line_point :: proc "contextless" (
	line_origin, line_direction: util.Vector3, half_length: f32, cylinder: Cylinder,
) -> (t: f32, offset_from_cylinder: util.Vector3)
{
	minimum := -half_length;
	maximum := half_length;
	radius_squared := cylinder.radius * cylinder.radius;
	origin_dot := util.vector3_dot(line_direction, line_origin);
	epsilon := half_length * 1e-7;
	for _ in 0 ..< 12
	{
		_, clamped := capsule_cylinder_bounce(line_origin, line_direction, t, cylinder, radius_squared);
		conservative_new_t := max(minimum, min(maximum, util.vector3_dot(clamped, line_direction) - origin_dot));
		change := conservative_new_t - t;
		if abs(change) < epsilon
		{
			break;
		}
		if change > 0
		{
			minimum = conservative_new_t;
		}
		else
		{
			maximum = conservative_new_t;
		}
		t = 0.5 * (minimum + maximum);
	}
	point_on_line, clamped_to_cylinder := capsule_cylinder_bounce(
		line_origin, line_direction, t, cylinder, radius_squared,
	);
	offset_from_cylinder = util.vector3_subtract(point_on_line, clamped_to_cylinder);
	return;
}

capsule_cylinder_closest_segments :: proc "contextless" (
	axis_a, local_offset_b: util.Vector3, a_half_length, b_half_length: f32,
) -> (ta, ta_min, ta_max, tb, tb_min, tb_max: f32)
{
	da_offset_b := util.vector3_dot(axis_a, local_offset_b);
	db_offset_b := local_offset_b.y;
	dadb := axis_a.y;
	ta = (da_offset_b - db_offset_b * dadb) / max(f32(1e-15), 1 - dadb * dadb);
	tb = ta * dadb - db_offset_b;
	abs_dadb := abs(dadb);
	b_onto_a_offset := b_half_length * abs_dadb;
	a_onto_b_offset := a_half_length * abs_dadb;
	ta_min = max(-a_half_length, min(a_half_length, da_offset_b - b_onto_a_offset));
	ta_max = min(a_half_length, max(-a_half_length, da_offset_b + b_onto_a_offset));
	tb_min = max(-b_half_length, min(b_half_length, -a_onto_b_offset - db_offset_b));
	tb_max = min(b_half_length, max(-b_half_length, a_onto_b_offset - db_offset_b));
	ta = min(max(ta, ta_min), ta_max);
	tb = min(max(tb, tb_min), tb_max);
	return;
}

capsule_cylinder_contact_interval :: proc "contextless" (
	a_half_length, b_half_length: f32,
	axis_a, local_normal: util.Vector3,
	inverse_horizontal_normal_length_squared: f32,
	offset_b: util.Vector3,
) -> (contact_t_min, contact_t_max: f32)
{
	_, _, _, tb, tb_min, tb_max := capsule_cylinder_closest_segments(
		axis_a, offset_b, a_half_length, b_half_length,
	);
	dot := axis_a.x * local_normal.z - axis_a.z * local_normal.x;
	squared_angle := dot * dot * inverse_horizontal_normal_length_squared;
	lower_threshold := f32(0.02 * 0.02);
	upper_threshold := f32(0.15 * 0.15);
	interval_weight := max(
		f32(0),
		min(f32(1), (upper_threshold - squared_angle) / (upper_threshold - lower_threshold))
	);
	weighted_tb := tb - tb * interval_weight;
	contact_t_min = interval_weight * tb_min + weighted_tb;
	contact_t_max = interval_weight * tb_max + weighted_tb;
	return;
}

capsule_cylinder_test_source :: proc "contextless" (
	a: Capsule, b: Cylinder, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if capsule_validate(a) != .Ok || cylinder_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	world_capsule_axis := util.quaternion_transform_unit_y(pose_a.orientation);
	capsule_axis := util.matrix3x3_transform_transpose(world_capsule_axis, world_r_b);
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	local_offset_a := util.vector3_negate(local_offset_b);

	_, local_normal := capsule_cylinder_closest_line_point(local_offset_a, capsule_axis, a.half_length, b);
	distance_squared := util.vector3_length_squared(local_normal);
	internal_line_segment_intersected := distance_squared < 1e-12;
	distance := math.sqrt(distance_squared);
	depth := -distance;
	if internal_line_segment_intersected
	{
		depth = f32(math.F32_MAX);
	}
	else
	{
		local_normal = util.vector3_scale(local_normal, 1 / distance);
	}
	negative_margin := -speculative_margin;
	if !internal_line_segment_intersected && depth + a.radius < negative_margin
	{
		return {offset_b=offset_b}, .Ok;
	}
	if internal_line_segment_intersected
	{
		endpoint_cap_depth := b.half_length + abs(capsule_axis.y * a.half_length) - abs(local_offset_a.y);
		if endpoint_cap_depth < depth
		{
			depth = endpoint_cap_depth;
			local_normal = {0, -1, 0};
			if local_offset_a.y > 0
			{
				local_normal.y = 1;
			}
		}

		ta, _, _, tb, _, _ := capsule_cylinder_closest_segments(
			capsule_axis, local_offset_b, a.half_length, b.half_length,
		);
		closest_a := util.vector3_scale(capsule_axis, ta);
		internal_offset := util.vector3_subtract(closest_a, local_offset_b);
		internal_offset.y -= tb;
		internal_distance := util.vector3_length(internal_offset);
		internal_edge_normal := util.Vector3{1, 0, 0};
		if internal_distance >= 1e-7
		{
			internal_edge_normal = util.vector3_scale(internal_offset, 1 / internal_distance);
		}
		center_separation := util.vector3_dot(local_offset_a, internal_edge_normal);
		cylinder_contribution := abs(b.half_length * internal_edge_normal.y) +
			b.radius * math.sqrt(max(f32(0), 1 - internal_edge_normal.y * internal_edge_normal.y));
		capsule_contribution := abs(util.vector3_dot(capsule_axis, internal_edge_normal)) * a.half_length;
		internal_edge_depth := cylinder_contribution + capsule_contribution - center_separation;
		if internal_edge_depth < depth
		{
			depth = internal_edge_depth;
			local_normal = internal_edge_normal;
		}
	}
	depth += a.radius;
	if depth < negative_margin
	{
		return {offset_b=offset_b}, .Ok;
	}

	use_cap_contacts := abs(local_normal.y) > 0.70710678118;
	horizontal_normal_length_squared := local_normal.x * local_normal.x + local_normal.z * local_normal.z;
	contact_0, contact_1: util.Vector3;
	contact_count := 1;
	if !use_cap_contacts
	{
		inverse_horizontal_length_squared := 1 / horizontal_normal_length_squared;
		scale := b.radius * math.sqrt(inverse_horizontal_length_squared);
		cylinder_segment_offset_x := local_normal.x * scale;
		cylinder_segment_offset_z := local_normal.z * scale;
		a_to_side_segment_center := util.Vector3{
			local_offset_b.x + cylinder_segment_offset_x,
			local_offset_b.y,
			local_offset_b.z + cylinder_segment_offset_z,
		};
		contact_t_min, contact_t_max := capsule_cylinder_contact_interval(
			a.half_length, b.half_length, capsule_axis, local_normal,
			inverse_horizontal_length_squared, a_to_side_segment_center,
		);
		contact_0 = {cylinder_segment_offset_x, contact_t_min, cylinder_segment_offset_z};
		contact_1 = {cylinder_segment_offset_x, contact_t_max, cylinder_segment_offset_z};
		if abs(contact_t_max - contact_t_min) >= b.half_length * 1e-5
		{
			contact_count = 2;
		}
	}
	else
	{
		cap_height := -b.half_length;
		if local_normal.y > 0
		{
			cap_height = b.half_length;
		}
		inverse_normal_y := 1 / local_normal.y;
		endpoint_offset := util.vector3_scale(capsule_axis, a.half_length);
		positive := util.vector3_add(local_offset_a, endpoint_offset);
		positive.y -= cap_height;
		negative := util.vector3_subtract(local_offset_a, endpoint_offset);
		negative.y -= cap_height;
		t_negative := negative.y * inverse_normal_y;
		t_positive := positive.y * inverse_normal_y;
		projected_negative := util.Vector2{
			negative.x - local_normal.x * t_negative,
			negative.z - local_normal.z * t_negative,
		};
		projected_positive := util.Vector2{
			positive.x - local_normal.x * t_positive,
			positive.z - local_normal.z * t_positive,
		};
		projected_offset := util.vector2_subtract(projected_positive, projected_negative);
		coefficient_c := util.vector2_dot(projected_negative, projected_negative) - b.radius * b.radius;
		coefficient_b := util.vector2_dot(projected_negative, projected_offset);
		coefficient_a := util.vector2_dot(projected_offset, projected_offset);
		t_min, t_max := f32(0), f32(0);
		if abs(coefficient_a) >= 1e-12
		{
			inverse_a := 1 / coefficient_a;
			t_offset := math.sqrt(max(
				f32(0),
				coefficient_b * coefficient_b - coefficient_a * coefficient_c
			)) * inverse_a;
			t_base := -coefficient_b * inverse_a;
			t_min = max(f32(0), min(f32(1), t_base - t_offset));
			t_max = max(f32(0), min(f32(1), t_base + t_offset));
		}
		contact_0 = {
			t_min * projected_offset.x + projected_negative.x,
			cap_height,
			t_min * projected_offset.y + projected_negative.y,
		};
		contact_1 = {
			t_max * projected_offset.x + projected_negative.x,
			cap_height,
			t_max * projected_offset.y + projected_negative.y,
		};
		if t_max - t_min > 1e-5
		{
			contact_count = 2;
		}
	}

	capsule_tangent := util.vector3_cross(local_normal, capsule_axis);
	face_normal_a := util.vector3_cross(capsule_tangent, capsule_axis);
	face_normal_a_dot_local_normal := util.vector3_dot(face_normal_a, local_normal);
	depth_0 := a.radius + util.vector3_dot(util.vector3_add(local_offset_b, contact_0), face_normal_a) /
		face_normal_a_dot_local_normal;
	depth_1 := a.radius + util.vector3_dot(util.vector3_add(local_offset_b, contact_1), face_normal_a) /
		face_normal_a_dot_local_normal;
	collapse := abs(face_normal_a_dot_local_normal) < 1e-7;
	if collapse
	{
		depth_0 = depth;
	}
	manifold := Convex_Contact_Manifold{
		offset_b=offset_b,
		normal=util.matrix3x3_transform(local_normal, world_r_b),
	};
	if depth_0 >= negative_margin
	{
		manifold.contacts[manifold.count] = {
			offset=util.vector3_add(util.matrix3x3_transform(contact_0, world_r_b), offset_b),
			depth=depth_0,
			feature_id=0,
		};
		manifold.count += 1;
	}
	if contact_count == 2 && !collapse && depth_1 >= negative_margin
	{
		manifold.contacts[manifold.count] = {
			offset=util.vector3_add(util.matrix3x3_transform(contact_1, world_r_b), offset_b),
			depth=depth_1,
			feature_id=1,
		};
		manifold.count += 1;
	}
	return manifold, .Ok;
}
