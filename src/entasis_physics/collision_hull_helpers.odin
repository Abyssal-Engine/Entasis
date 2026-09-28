// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

MAXIMUM_HULL_FACE_VERTEX_COUNT :: 1024;

collision_build_orthonormal_basis :: proc "contextless" (
	normal: util.Vector3,
) -> (tangent_x, tangent_y: util.Vector3)
{
	sign := f32(1);
	if normal.z < 0
	{
		sign = -1;
	}
	scale := -1 / (sign + normal.z);
	tangent_x = {
		normal.x * normal.y * scale,
		sign + normal.y * normal.y * scale,
		-normal.y,
	};
	tangent_y = {
		1 + sign * normal.x * normal.x * scale,
		sign * tangent_x.x,
		-sign * normal.x,
	};
	return;
}

convex_hull_face_range_validated :: #force_inline proc "contextless" (
	hull: ^Convex_Hull,
	face_index: int,
) -> (start, end: int, status: Physics_Status)
{
	if face_index < 0 || face_index >= int(hull.face_start_indices.length)
	{
		return 0, 0, .Invalid_Argument;
	}
	start = int(hull.face_start_indices.memory[face_index]);
	end = int(hull.face_vertex_indices.length);
	if face_index + 1 < int(hull.face_start_indices.length)
	{
		end = int(hull.face_start_indices.memory[face_index + 1]);
	}
	if end - start < 3
	{
		return 0, 0, .Invalid_Description;
	}
	return start, end, .Ok;
}

convex_hull_face_range :: proc "contextless" (
	hull: ^Convex_Hull,
	face_index: int,
) -> (start, end: int, status: Physics_Status)
{
	if convex_hull_validate(hull) != .Ok
	{
		return 0, 0, .Invalid_Argument;
	}
	return convex_hull_face_range_validated(hull, face_index);
}

convex_hull_face_normal_and_offset_validated :: #force_inline proc "contextless" (
	hull: ^Convex_Hull,
	face_index: int,
) -> (normal: util.Vector3, offset: f32, status: Physics_Status)
{
	if face_index < 0 || face_index >= int(hull.face_start_indices.length)
	{
		return {}, 0, .Invalid_Argument;
	}
	normal, offset = convex_hull_get_face_plane(hull, face_index);
	return normal, offset, .Ok;
}

convex_hull_face_normal_and_offset :: proc "contextless" (
	hull: ^Convex_Hull,
	face_index: int,
) -> (normal: util.Vector3, offset: f32, status: Physics_Status)
{
	if convex_hull_validate(hull) != .Ok
	{
		return {}, 0, .Invalid_Argument;
	}
	return convex_hull_face_normal_and_offset_validated(hull, face_index);
}

convex_hull_pick_representative_face_validated :: proc "contextless" (
	hull: ^Convex_Hull,
	local_normal, closest_on_hull: util.Vector3,
	bounding_plane_epsilon: f32,
) -> (face_normal: util.Vector3, face_index: int, status: Physics_Status)
{
	if bounding_plane_epsilon < 0
	{
		return {}, -1, .Invalid_Argument;
	}
	best_normal, best_offset, face_status := convex_hull_face_normal_and_offset_validated(hull, 0);
	if face_status != .Ok
	{
		return {}, -1, face_status;
	}
	best_dot := util.vector3_dot(best_normal, local_normal);
	best_error := abs(util.vector3_dot(best_normal, closest_on_hull) - best_offset);
	face_index = 0;
	for candidate_index in 1 ..< int(hull.face_start_indices.length)
	{
		candidate_normal, candidate_offset, candidate_status := convex_hull_face_normal_and_offset_validated(
			hull,
			candidate_index,
		);
		if candidate_status != .Ok
		{
			return {}, -1, candidate_status;
		}
		candidate_dot := util.vector3_dot(candidate_normal, local_normal);
		candidate_error := abs(util.vector3_dot(candidate_normal, closest_on_hull) - candidate_offset);
		error_improvement := best_error - candidate_error;
		if error_improvement >= bounding_plane_epsilon ||
			error_improvement > -bounding_plane_epsilon && candidate_dot > best_dot
		{
			best_normal = candidate_normal;
			best_dot = candidate_dot;
			best_error = candidate_error;
			face_index = candidate_index;
		}
	}
	return best_normal, face_index, .Ok;
}

convex_hull_pick_representative_face :: proc "contextless" (
	hull: ^Convex_Hull,
	local_normal, closest_on_hull: util.Vector3,
	bounding_plane_epsilon: f32,
) -> (face_normal: util.Vector3, face_index: int, status: Physics_Status)
{
	if convex_hull_validate(hull) != .Ok
	{
		return {}, -1, .Invalid_Argument;
	}
	return convex_hull_pick_representative_face_validated(
		hull,
		local_normal,
		closest_on_hull,
		bounding_plane_epsilon,
	);
}
