// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

capsule_box_edge_test :: proc "contextless" (
	local_offset_a, capsule_axis: util.Vector3,
	capsule_half_length: f32,
	edge_center, edge_axis: util.Vector3,
	edge_half_length: f32,
	box_extents: util.Vector3,
) -> (ta, depth: f32, normal: util.Vector3)
{
	tb: f32;
	ta, tb = collision_segment_closest_parameters(
		local_offset_a, capsule_axis, capsule_half_length,
		edge_center, edge_axis, edge_half_length,
	);
	closest_on_a := util.vector3_add(local_offset_a, util.vector3_scale(capsule_axis, ta));
	closest_on_b := util.vector3_add(edge_center, util.vector3_scale(edge_axis, tb));
	normal = util.vector3_subtract(closest_on_a, closest_on_b);
	squared_length := util.vector3_length_squared(normal);
	if squared_length < 1e-10
	{
		normal = util.vector3_cross(capsule_axis, edge_axis);
		squared_length = util.vector3_length_squared(normal);
		if squared_length < 1e-10
		{
			if abs(edge_axis.x) < 0.5
			{
				normal = {1, 0, 0};
			}
			else
			{
				normal = {0, 1, 0};
			}
			squared_length = 1;
		}
	}
	if util.vector3_dot(normal, local_offset_a) < 0
	{
		normal = util.vector3_negate(normal);
	}
	normal = util.vector3_scale(normal, 1 / math.sqrt(squared_length));
	box_extreme := abs(normal.x) * box_extents.x + abs(normal.y) * box_extents.y + abs(normal.z) * box_extents.z;
	capsule_extreme := util.vector3_dot(normal, closest_on_a);
	depth = box_extreme - capsule_extreme;
	return;
}

capsule_box_test_source :: proc "contextless" (
	a: Capsule, b: Box, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if capsule_validate(a) != .Ok || box_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	local_offset_a := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	world_capsule_axis := util.quaternion_transform_unit_y(pose_a.orientation);
	capsule_axis := util.quaternion_transform(world_capsule_axis, util.quaternion_conjugate(pose_b.orientation));
	dot_axis := util.vector3_dot(local_offset_a, capsule_axis);
	clamped_dot := max(-a.half_length, min(a.half_length, dot_axis));
	offset_to_capsule_from_box := util.vector3_subtract(local_offset_a, util.vector3_scale(capsule_axis, clamped_dot));
	edge_centers := util.Vector3{b.half_width, b.half_height, b.half_length};
	if offset_to_capsule_from_box.x < 0
	{
		edge_centers.x = -edge_centers.x;
	}
	if offset_to_capsule_from_box.y < 0
	{
		edge_centers.y = -edge_centers.y;
	}
	if offset_to_capsule_from_box.z < 0
	{
		edge_centers.z = -edge_centers.z;
	}
	extents := util.Vector3{b.half_width, b.half_height, b.half_length};

	x_edge_center := util.Vector3{0, edge_centers.y, edge_centers.z};
	ta, depth, local_normal := capsule_box_edge_test(
		local_offset_a, capsule_axis, a.half_length, x_edge_center, {1, 0, 0}, b.half_width, extents,
	);
	y_edge_center := util.Vector3{edge_centers.x, 0, edge_centers.z};
	y_ta, y_depth, y_normal := capsule_box_edge_test(
		local_offset_a, capsule_axis, a.half_length, y_edge_center, {0, 1, 0}, b.half_height, extents,
	);
	if y_depth < depth
	{
		ta = y_ta;
		depth = y_depth;
		local_normal = y_normal;
	}
	z_edge_center := util.Vector3{edge_centers.x, edge_centers.y, 0};
	z_ta, z_depth, z_normal := capsule_box_edge_test(
		local_offset_a, capsule_axis, a.half_length, z_edge_center, {0, 0, 1}, b.half_length, extents,
	);
	if z_depth < depth
	{
		ta = z_ta;
		depth = z_depth;
		local_normal = z_normal;
	}

	face_signs := util.Vector3{1, 1, 1};
	if local_offset_a.x <= 0
	{
		face_signs.x = -1;
	}
	if local_offset_a.y <= 0
	{
		face_signs.y = -1;
	}
	if local_offset_a.z <= 0
	{
		face_signs.z = -1;
	}
	face_depths := util.Vector3{
		b.half_width + abs(capsule_axis.x) * a.half_length - face_signs.x * local_offset_a.x,
		b.half_height + abs(capsule_axis.y) * a.half_length - face_signs.y * local_offset_a.y,
		b.half_length + abs(capsule_axis.z) * a.half_length - face_signs.z * local_offset_a.z,
	};
	if face_depths.x < depth
	{
		depth = face_depths.x;
		local_normal = {face_signs.x, 0, 0};
	}
	if face_depths.y < depth
	{
		depth = face_depths.y;
		local_normal = {0, face_signs.y, 0};
	}
	if face_depths.z < depth
	{
		depth = face_depths.z;
		local_normal = {0, 0, face_signs.z};
	}

	x_dot := local_normal.x * face_signs.x;
	y_dot := local_normal.y * face_signs.y;
	z_dot := local_normal.z * face_signs.z;
	face_axis := 2;
	if x_dot > max(y_dot, z_dot)
	{
		face_axis = 0;
	}
	else if y_dot > z_dot
	{
		face_axis = 1;
	}
	face_sign := face_signs.z;
	face_plane_offset := b.half_length;
	face_normal_dot_local_normal := z_dot;
	capsule_axis_dot_face_normal := capsule_axis.z * face_sign;
	capsule_center_dot_face_normal := local_offset_a.z * face_sign;
	if face_axis == 0
	{
		face_sign = face_signs.x;
		face_plane_offset = b.half_width;
		face_normal_dot_local_normal = x_dot;
		capsule_axis_dot_face_normal = capsule_axis.x * face_sign;
		capsule_center_dot_face_normal = local_offset_a.x * face_sign;
	}
	else if face_axis == 1
	{
		face_sign = face_signs.y;
		face_plane_offset = b.half_height;
		face_normal_dot_local_normal = y_dot;
		capsule_axis_dot_face_normal = capsule_axis.y * face_sign;
		capsule_center_dot_face_normal = local_offset_a.y * face_sign;
	}
	inverse_face_dot := 1 / max(f32(1e-15), face_normal_dot_local_normal);
	t_axis := capsule_axis_dot_face_normal * inverse_face_dot;
	t_center := (capsule_center_dot_face_normal - face_plane_offset) * inverse_face_dot;
	unprojected_axis := util.vector3_subtract(capsule_axis, util.vector3_scale(local_normal, t_axis));
	unprojected_center := util.vector3_subtract(local_offset_a, util.vector3_scale(local_normal, t_center));
	tangent_axis, tangent_center: util.Vector2;
	half_extent_x, half_extent_y: f32;
	if face_axis == 0
	{
		tangent_axis = {unprojected_axis.y, unprojected_axis.z};
		tangent_center = {unprojected_center.y, unprojected_center.z};
		half_extent_x = b.half_height;
		half_extent_y = b.half_length;
	}
	else if face_axis == 1
	{
		tangent_axis = {unprojected_axis.x, unprojected_axis.z};
		tangent_center = {unprojected_center.x, unprojected_center.z};
		half_extent_x = b.half_width;
		half_extent_y = b.half_length;
	}
	else
	{
		tangent_axis = {unprojected_axis.x, unprojected_axis.y};
		tangent_center = {unprojected_center.x, unprojected_center.y};
		half_extent_x = b.half_width;
		half_extent_y = b.half_height;
	}
	epsilon_scale := min(max(b.half_width, max(b.half_height, b.half_length)), max(a.half_length, a.radius));
	half_extent_x += epsilon_scale * 1e-3;
	half_extent_y += epsilon_scale * 1e-3;
	minimum_x, maximum_x: f32;
	if abs(tangent_axis.x) < 1e-15
	{
		if abs(tangent_center.x) <= half_extent_x
		{
			minimum_x = -f32(math.F32_MAX);
			maximum_x = f32(math.F32_MAX);
		}
		else
		{
			minimum_x = f32(math.F32_MAX);
			maximum_x = -f32(math.F32_MAX);
		}
	}
	else
	{
		t0 := -(tangent_center.x - half_extent_x) / tangent_axis.x;
		t1 := -(tangent_center.x + half_extent_x) / tangent_axis.x;
		minimum_x = min(t0, t1);
		maximum_x = max(t0, t1);
	}
	minimum_y, maximum_y: f32;
	if abs(tangent_axis.y) < 1e-15
	{
		if abs(tangent_center.y) <= half_extent_y
		{
			minimum_y = -f32(math.F32_MAX);
			maximum_y = f32(math.F32_MAX);
		}
		else
		{
			minimum_y = f32(math.F32_MAX);
			maximum_y = -f32(math.F32_MAX);
		}
	}
	else
	{
		t0 := -(tangent_center.y - half_extent_y) / tangent_axis.y;
		t1 := -(tangent_center.y + half_extent_y) / tangent_axis.y;
		minimum_y = min(t0, t1);
		maximum_y = max(t0, t1);
	}
	face_min := max(minimum_x, minimum_y);
	face_max := min(maximum_x, maximum_y);
	t_min := max(min(face_min, a.half_length), -a.half_length);
	t_max := max(min(face_max, a.half_length), -a.half_length);
	if face_max >= face_min
	{
		t_min = min(t_min, ta);
		t_max = max(t_max, ta);
	}
	else
	{
		t_min = ta;
		t_max = ta;
	}
	separation_min := t_center + t_axis * t_min;
	separation_max := t_center + t_axis * t_max;
	depths := [2]f32{a.radius - separation_min, a.radius - separation_max};
	local_offsets := [2]util.Vector3{
		util.vector3_scale(capsule_axis, t_min),
		util.vector3_scale(capsule_axis, t_max),
	};
	world_normal := util.quaternion_transform(local_normal, pose_b.orientation);
	manifold := Convex_Contact_Manifold{
		offset_b=util.vector3_subtract(pose_b.position, pose_a.position),
		normal=world_normal,
	};
	for contact_index in 0 ..< 2
	{
		if depths[contact_index] >= -speculative_margin &&
			(contact_index == 0 || t_max - t_min > 1e-7 * a.half_length)
		{
			world_local_offset := util.quaternion_transform(local_offsets[contact_index], pose_b.orientation);
			manifold.contacts[manifold.count] = {
				offset=util.vector3_add(
					world_local_offset,
					util.vector3_scale(world_normal, depths[contact_index] * 0.5 - a.radius)
				),
				depth=depths[contact_index],
				feature_id=i32(contact_index),
			};
			manifold.count += 1;
		}
	}
	return manifold, .Ok;
}
