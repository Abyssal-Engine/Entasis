// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

MAXIMUM_MANIFOLD_CANDIDATE_COUNT :: 2048;

Manifold_Candidate_Scalar :: struct
{
	x, y:       f32,
	feature_id: i32,
}

manifold_candidate_place :: proc "contextless" (
	candidate: Manifold_Candidate_Scalar,
	depth: f32,
	face_center_b, tangent_b_x, tangent_b_y: util.Vector3,
	rotation_to_world: util.Matrix3x3,
	world_offset_b: util.Vector3,
	feature_id: i32,
) -> Convex_Contact
{
	local_position := util.vector3_add(
		face_center_b,
		util.vector3_add(
		util.vector3_scale(tangent_b_x, candidate.x),
		util.vector3_scale(tangent_b_y, candidate.y),
	),
	);
	return {
		offset=util.vector3_add(util.matrix3x3_transform(local_position, rotation_to_world), world_offset_b),
		depth=depth,
		feature_id=feature_id,
	};
}

manifold_candidate_fast_remove :: proc "contextless" (
	candidates: ^[MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar,
	depths: ^[MAXIMUM_MANIFOLD_CANDIDATE_COUNT]f32,
	candidate_count: ^int,
	removal_index: int,
)
{
	last_index := candidate_count^ - 1;
	if removal_index < last_index
	{
		candidates[removal_index] = candidates[last_index];
		depths[removal_index] = depths[last_index];
	}
	candidate_count^ -= 1;
}

manifold_candidate_reduce :: proc "contextless" (
	candidates: ^[MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar,
	candidate_count: int,
	face_normal_a: util.Vector3,
	inverse_face_normal_a_dot_local_normal: f32,
	face_center_a, face_center_b, tangent_b_x, tangent_b_y: util.Vector3,
	epsilon_scale, minimum_depth: f32,
	rotation_to_world: util.Matrix3x3,
	world_offset_b: util.Vector3,
	world_normal: util.Vector3,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if candidates == nil || candidate_count < 0 || candidate_count > len(candidates^)
	{
		return {}, .Invalid_Argument;
	}
	remaining_count := candidate_count;
	manifold := Convex_Contact_Manifold{offset_b=world_offset_b, normal=world_normal};
	if remaining_count == 0
	{
		return manifold, .Ok;
	}
	dot_axis := util.vector3_scale(face_normal_a, inverse_face_normal_a_dot_local_normal);
	face_center_a_to_b := util.vector3_subtract(face_center_b, face_center_a);
	base_dot := util.vector3_dot(face_center_a_to_b, dot_axis);
	x_dot := util.vector3_dot(tangent_b_x, dot_axis);
	y_dot := util.vector3_dot(tangent_b_y, dot_axis);
	depths: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]f32 = ---;
	index := remaining_count - 1;
	for index >= 0
	{
		candidate := candidates[index];
		depths[index] = base_dot + candidate.x * x_dot + candidate.y * y_dot;
		if depths[index] < minimum_depth
		{
			manifold_candidate_fast_remove(candidates, &depths, &remaining_count, index);
		}
		index -= 1;
	}
	if remaining_count <= MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		for contact_index in 0 ..< remaining_count
		{
			candidate := candidates[contact_index];
			manifold.contacts[manifold.count] = manifold_candidate_place(
				candidate, depths[contact_index], face_center_b, tangent_b_x, tangent_b_y,
				rotation_to_world, world_offset_b, candidate.feature_id,
			);
			manifold.count += 1;
		}
		return manifold, .Ok;
	}

	best_score_0 := -f32(math.F32_MAX);
	best_index_0 := 0;
	extremity_x := 0.7946897654 * epsilon_scale * 1e-2;
	extremity_y := 0.60701579614 * epsilon_scale * 1e-2;
	for candidate_index in 0 ..< remaining_count
	{
		candidate_score := depths[candidate_index];
		if candidate_score >= 0
		{
			candidate_score += abs(candidates[candidate_index].x * extremity_x + candidates[candidate_index].y * extremity_y);
		}
		if candidate_score > best_score_0
		{
			best_score_0 = candidate_score;
			best_index_0 = candidate_index;
		}
	}
	candidate_0 := candidates[best_index_0];
	depth_0 := depths[best_index_0];
	manifold.contacts[manifold.count] = manifold_candidate_place(
		candidate_0, depth_0, face_center_b, tangent_b_x, tangent_b_y,
		rotation_to_world, world_offset_b, candidate_0.feature_id,
	);
	manifold.count += 1;
	manifold_candidate_fast_remove(candidates, &depths, &remaining_count, best_index_0);

	maximum_distance_squared := f32(-1);
	best_index_1 := 0;
	for candidate_index in 0 ..< remaining_count
	{
		offset_x := candidates[candidate_index].x - candidate_0.x;
		offset_y := candidates[candidate_index].y - candidate_0.y;
		distance_squared := offset_x * offset_x + offset_y * offset_y;
		if distance_squared > maximum_distance_squared
		{
			maximum_distance_squared = distance_squared;
			best_index_1 = candidate_index;
		}
	}
	if maximum_distance_squared < 1e-6 * epsilon_scale * epsilon_scale
	{
		return manifold, .Ok;
	}
	candidate_1 := candidates[best_index_1];
	manifold.contacts[manifold.count] = manifold_candidate_place(
		candidate_1, depths[best_index_1], face_center_b, tangent_b_x, tangent_b_y,
		rotation_to_world, world_offset_b, candidate_1.feature_id,
	);
	manifold.count += 1;
	manifold_candidate_fast_remove(candidates, &depths, &remaining_count, best_index_1);

	edge_offset_x := candidate_1.x - candidate_0.x;
	edge_offset_y := candidate_1.y - candidate_0.y;
	minimum_signed_area := f32(0);
	maximum_signed_area := f32(0);
	best_index_2 := 0;
	best_index_3 := 0;
	for candidate_index in 0 ..< remaining_count
	{
		candidate_offset_x := candidates[candidate_index].x - candidate_0.x;
		candidate_offset_y := candidates[candidate_index].y - candidate_0.y;
		signed_area := candidate_offset_x * edge_offset_y - candidate_offset_y * edge_offset_x;
		if depths[candidate_index] < 0
		{
			signed_area *= 0.25;
		}
		if signed_area < minimum_signed_area
		{
			minimum_signed_area = signed_area;
			best_index_2 = candidate_index;
		}
		if signed_area > maximum_signed_area
		{
			maximum_signed_area = signed_area;
			best_index_3 = candidate_index;
		}
	}
	area_epsilon := maximum_distance_squared * maximum_distance_squared * 1e-6;
	if minimum_signed_area * minimum_signed_area > area_epsilon
	{
		candidate := candidates[best_index_2];
		manifold.contacts[manifold.count] = manifold_candidate_place(
			candidate, depths[best_index_2], face_center_b, tangent_b_x, tangent_b_y,
			rotation_to_world, world_offset_b, candidate.feature_id,
		);
		manifold.count += 1;
	}
	if maximum_signed_area * maximum_signed_area > area_epsilon
	{
		candidate := candidates[best_index_3];
		manifold.contacts[manifold.count] = manifold_candidate_place(
			candidate, depths[best_index_3], face_center_b, tangent_b_x, tangent_b_y,
			rotation_to_world, world_offset_b, candidate.feature_id,
		);
		manifold.count += 1;
	}
	return manifold, .Ok;
}
