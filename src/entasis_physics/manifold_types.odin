// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

MAXIMUM_MANIFOLD_CONTACT_COUNT :: 4;

Contact :: struct
{
	offset:     util.Vector3,
	depth:      f32,
	normal:     util.Vector3,
	feature_id: i32,
}

Convex_Contact :: struct
{
	offset:     util.Vector3,
	depth:      f32,
	feature_id: i32,
}

Nonconvex_Contact_Manifold :: struct
{
	offset_b: util.Vector3,
	count:    i32,
	contacts: [MAXIMUM_MANIFOLD_CONTACT_COUNT]Contact,
}

Convex_Contact_Manifold :: struct
{
	offset_b: util.Vector3,
	count:    i32,
	normal:   util.Vector3,
	contacts: [MAXIMUM_MANIFOLD_CONTACT_COUNT]Convex_Contact,
}

Convex_1_Contact_Manifold_Wide :: struct
{
	offset_a:       util.Vector3_Wide,
	normal:         util.Vector3_Wide,
	depth:          util.F32x8,
	feature_id:     util.I32x8,
	contact_exists: util.I32x8,
}

Convex_2_Contact_Manifold_Wide :: struct
{
	offset_a_0:       util.Vector3_Wide,
	offset_a_1:       util.Vector3_Wide,
	normal:           util.Vector3_Wide,
	depth_0:          util.F32x8,
	depth_1:          util.F32x8,
	feature_id_0:     util.I32x8,
	feature_id_1:     util.I32x8,
	contact_0_exists: util.I32x8,
	contact_1_exists: util.I32x8,
}

Convex_4_Contact_Manifold_Wide :: struct
{
	offset_a_0:       util.Vector3_Wide,
	offset_a_1:       util.Vector3_Wide,
	offset_a_2:       util.Vector3_Wide,
	offset_a_3:       util.Vector3_Wide,
	normal:           util.Vector3_Wide,
	depth_0:          util.F32x8,
	depth_1:          util.F32x8,
	depth_2:          util.F32x8,
	depth_3:          util.F32x8,
	feature_id_0:     util.I32x8,
	feature_id_1:     util.I32x8,
	feature_id_2:     util.I32x8,
	feature_id_3:     util.I32x8,
	contact_0_exists: util.I32x8,
	contact_1_exists: util.I32x8,
	contact_2_exists: util.I32x8,
	contact_3_exists: util.I32x8,
}

Manifold_Kind :: enum u8
{
	Convex,
	Nonconvex,
}

Manifold_Result :: struct
{
	kind:      Manifold_Kind,
	convex:    Convex_Contact_Manifold,
	nonconvex: Nonconvex_Contact_Manifold,
}

Collision_Stored_Manifold :: struct
{
	kind: Manifold_Kind,
	using data: struct #raw_union
	{
		convex:    Convex_Contact_Manifold,
		nonconvex: Nonconvex_Contact_Manifold,
	},
}

#assert(size_of(Collision_Stored_Manifold) == 148);

collision_wide_select_i32 :: proc "contextless" (
	condition, left, right: util.I32x8,
) -> util.I32x8
{
	return (condition & left) | (~condition & right);
}

Manifold_Candidate_Wide :: struct
{
	x:          util.F32x8,
	y:          util.F32x8,
	depth:      util.F32x8,
	feature_id: util.I32x8,
}

manifold_candidate_wide_select :: proc "contextless" (
	condition: util.I32x8, left, right: Manifold_Candidate_Wide,
) -> Manifold_Candidate_Wide
{
	return {
		x=util.wide_select_f32(condition, left.x, right.x),
		y=util.wide_select_f32(condition, left.y, right.y),
		depth=util.wide_select_f32(condition, left.depth, right.depth),
		feature_id=collision_wide_select_i32(condition, left.feature_id, right.feature_id),
	};
}

manifold_candidate_wide_exists :: proc "contextless" (
	candidate: Manifold_Candidate_Wide, minimum_depth: util.F32x8,
	raw_contact_count: util.I32x8, candidate_index: int,
) -> util.I32x8
{
	return transmute(util.I32x8)simd.lanes_gt(candidate.depth, minimum_depth) &
		transmute(util.I32x8)simd.lanes_lt(util.I32x8(i32(candidate_index)), raw_contact_count);
}

manifold_candidate_wide_add :: proc "contextless" (
	candidates: ^[8]Manifold_Candidate_Wide, count: ^util.I32x8,
	candidate: Manifold_Candidate_Wide, new_contact_exists: util.I32x8, pair_count: int,
)
{
	for lane in 0 ..< pair_count
	{
		if simd.extract(new_contact_exists, lane) >= 0
		{
			continue;
		}
		target_index := int(simd.extract(count^, lane));
		candidates[target_index].x = simd.replace(candidates[target_index].x, lane, simd.extract(candidate.x, lane));
		candidates[target_index].y = simd.replace(candidates[target_index].y, lane, simd.extract(candidate.y, lane));
		candidates[target_index].depth = simd.replace(
			candidates[target_index].depth,
			lane,
			simd.extract(candidate.depth, lane)
		);
		candidates[target_index].feature_id = simd.replace(
			candidates[target_index].feature_id, lane, simd.extract(candidate.feature_id, lane),
		);
	}
	count^ = collision_wide_select_i32(new_contact_exists, simd.add(count^, util.I32x8(1)), count^);
}

manifold_candidate_wide_reduce_internal :: proc "contextless" (
	candidates: ^[8]Manifold_Candidate_Wide, raw_contact_count: util.I32x8,
	max_candidate_count: int, epsilon_scale, minimum_depth: util.F32x8,
) -> ([4]Manifold_Candidate_Wide, [4]util.I32x8)
{
	contacts: [4]Manifold_Candidate_Wide;
	exists: [4]util.I32x8;
	best_score := util.F32x8(-3.402823466e+38);
	for candidate_index in 0 ..< max_candidate_count
	{
		candidate := candidates[candidate_index];
		candidate_exists := manifold_candidate_wide_exists(
			candidate, minimum_depth, raw_contact_count, candidate_index,
		);
		extremity := simd.abs(simd.add(
			simd.mul(candidate.x, util.F32x8(0.7946897654)),
			simd.mul(candidate.y, util.F32x8(0.60701579614))
		));
		active_depth := transmute(util.I32x8)simd.lanes_ge(candidate.depth, util.F32x8(0));
		candidate_score := simd.add(
			candidate.depth,
			util.wide_select_f32(active_depth, simd.mul(extremity, util.F32x8(1e-2)), util.F32x8(0)),
		);
		use_candidate := candidate_exists & transmute(util.I32x8)simd.lanes_gt(candidate_score, best_score);
		contacts[0] = manifold_candidate_wide_select(use_candidate, candidate, contacts[0]);
		best_score = util.wide_select_f32(use_candidate, candidate_score, best_score);
	}
	exists[0] = transmute(util.I32x8)simd.lanes_gt(best_score, util.F32x8(-3.402823466e+38));

	max_distance_squared := util.F32x8(0);
	for candidate_index in 0 ..< max_candidate_count
	{
		candidate := candidates[candidate_index];
		offset_x := simd.sub(candidate.x, contacts[0].x);
		offset_y := simd.sub(candidate.y, contacts[0].y);
		distance_squared := simd.add(simd.mul(offset_x, offset_x), simd.mul(offset_y, offset_y));
		candidate_exists := manifold_candidate_wide_exists(
			candidate, minimum_depth, raw_contact_count, candidate_index,
		);
		use_candidate := candidate_exists & transmute(util.I32x8)simd.lanes_gt(distance_squared, max_distance_squared);
		contacts[1] = manifold_candidate_wide_select(use_candidate, candidate, contacts[1]);
		max_distance_squared = util.wide_select_f32(use_candidate, distance_squared, max_distance_squared);
	}
	distance_epsilon := simd.mul(simd.mul(epsilon_scale, epsilon_scale), util.F32x8(1e-6));
	exists[1] = transmute(util.I32x8)simd.lanes_gt(max_distance_squared, distance_epsilon);

	edge_offset_x := simd.sub(contacts[1].x, contacts[0].x);
	edge_offset_y := simd.sub(contacts[1].y, contacts[0].y);
	min_signed_area, max_signed_area := util.F32x8(0), util.F32x8(0);
	for candidate_index in 0 ..< max_candidate_count
	{
		candidate := candidates[candidate_index];
		candidate_offset_x := simd.sub(candidate.x, contacts[0].x);
		candidate_offset_y := simd.sub(candidate.y, contacts[0].y);
		signed_area := simd.sub(
			simd.mul(candidate_offset_x, edge_offset_y),
			simd.mul(candidate_offset_y, edge_offset_x)
		);
		signed_area = util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(candidate.depth, util.F32x8(0)),
			simd.mul(signed_area, util.F32x8(0.25)), signed_area,
		);
		candidate_exists := manifold_candidate_wide_exists(
			candidate, minimum_depth, raw_contact_count, candidate_index,
		);
		use_min := candidate_exists & transmute(util.I32x8)simd.lanes_lt(signed_area, min_signed_area);
		min_signed_area = util.wide_select_f32(use_min, signed_area, min_signed_area);
		contacts[2] = manifold_candidate_wide_select(use_min, candidate, contacts[2]);
		use_max := candidate_exists & transmute(util.I32x8)simd.lanes_gt(signed_area, max_signed_area);
		max_signed_area = util.wide_select_f32(use_max, signed_area, max_signed_area);
		contacts[3] = manifold_candidate_wide_select(use_max, candidate, contacts[3]);
	}
	area_epsilon := simd.mul(simd.mul(max_distance_squared, max_distance_squared), util.F32x8(1e-6));
	exists[2] = transmute(util.I32x8)simd.lanes_gt(simd.mul(min_signed_area, min_signed_area), area_epsilon);
	exists[3] = transmute(util.I32x8)simd.lanes_gt(simd.mul(max_signed_area, max_signed_area), area_epsilon);
	return contacts, exists;
}

manifold_candidate_wide_reduce :: proc "contextless" (
	candidates: ^[8]Manifold_Candidate_Wide, raw_contact_count: util.I32x8, max_candidate_count: int,
	face_normal_a: util.Vector3_Wide, inverse_face_normal_dot_normal: util.F32x8,
	face_center_b_to_face_center_a, tangent_bx, tangent_by: util.Vector3_Wide,
	epsilon_scale, minimum_depth: util.F32x8, pair_count: int,
) -> ([4]Manifold_Candidate_Wide, [4]util.I32x8)
{
	// Candidate counts are lane-local; reduce them without scalar lane extraction.
	valid_lanes := transmute(util.I32x8)simd.lanes_lt(
		util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(pair_count)),
	);
	masked_contact_count := raw_contact_count & valid_lanes;
	visited_candidate_count := min(
		max(0, int(simd.reduce_max(masked_contact_count))), max_candidate_count,
	);
	dot_axis := util.vector3_wide_scale(face_normal_a, inverse_face_normal_dot_normal);
	negative_base_dot := util.vector3_wide_dot(face_center_b_to_face_center_a, dot_axis);
	x_dot := util.vector3_wide_dot(tangent_bx, dot_axis);
	y_dot := util.vector3_wide_dot(tangent_by, dot_axis);
	for candidate_index in 0 ..< visited_candidate_count
	{
		candidate := &candidates[candidate_index];
		candidate.depth = simd.sub(
			simd.add(simd.mul(candidate.x, x_dot), simd.mul(candidate.y, y_dot)), negative_base_dot,
		);
	}
	depth_accepted_candidate_count := 0;
	for candidate_index in 0 ..< visited_candidate_count
	{
		candidate_exists := manifold_candidate_wide_exists(
			candidates[candidate_index], minimum_depth, masked_contact_count, candidate_index,
		);
		// Inactive lanes have zero counts and therefore cannot accept a candidate.
		if simd.reduce_or(candidate_exists) != 0
		{
			depth_accepted_candidate_count = candidate_index + 1;
		}
	}
	return manifold_candidate_wide_reduce_internal(
		candidates, masked_contact_count, depth_accepted_candidate_count, epsilon_scale, minimum_depth,
	);
}

CCD_Continuation_Index :: struct
{
	packed: u32,
}

Continuation_State :: enum u8
{
	Missing,
	Present,
}

ccd_continuation_create :: proc "contextless" (type_index, index: int) -> (CCD_Continuation_Index, Physics_Status)
{
	if type_index < 0 || type_index >= 2 || index < 0 || index >= 1 << 30
	{
		return {}, .Invalid_Argument;
	}
	return {u32(type_index << 30) | u32(index) | (u32(1) << 31)}, .Ok;
}

ccd_continuation_from_packed :: proc "contextless" (packed: u32) -> CCD_Continuation_Index
{
	return {packed};
}
ccd_continuation_state :: proc "contextless" (continuation: CCD_Continuation_Index) -> Continuation_State
{
	if continuation.packed & (u32(1) << 31) != 0
	{
		return .Present;
	}
	return .Missing;
}
ccd_continuation_type :: proc "contextless" (continuation: CCD_Continuation_Index) -> i32
{
	return i32((continuation.packed >> 30) & 1);
}
ccd_continuation_index :: proc "contextless" (continuation: CCD_Continuation_Index) -> i32
{
	return i32(continuation.packed & 0x3fff_ffff);
}

convex_manifold_append :: proc "contextless" (
	manifold: ^Convex_Contact_Manifold, contact: Convex_Contact,
) -> Physics_Status
{
	if manifold == nil || manifold.count < 0 || manifold.count >= MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return .Capacity_Missing;
	}
	manifold.contacts[manifold.count] = contact;
	manifold.count += 1;
	return .Ok;
}

nonconvex_manifold_append :: proc "contextless" (
	manifold: ^Nonconvex_Contact_Manifold, contact: Contact,
) -> Physics_Status
{
	if manifold == nil || manifold.count < 0 || manifold.count >= MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return .Capacity_Missing;
	}
	manifold.contacts[manifold.count] = contact;
	manifold.count += 1;
	return .Ok;
}

nonconvex_manifold_append_convex :: proc "contextless" (
	manifold: ^Nonconvex_Contact_Manifold, normal: util.Vector3, contact: Convex_Contact,
) -> Physics_Status
{
	return nonconvex_manifold_append(manifold, {contact.offset, contact.depth, normal, contact.feature_id});
}

convex_manifold_fast_remove :: proc "contextless" (manifold: ^Convex_Contact_Manifold, index: int) -> Physics_Status
{
	if manifold == nil || index < 0 || index >= int(manifold.count)
	{
		return .Invalid_Argument;
	}
	manifold.count -= 1;
	if index < int(manifold.count)
	{
		manifold.contacts[index] = manifold.contacts[manifold.count];
	}
	manifold.contacts[manifold.count] = {};
	return .Ok;
}

nonconvex_manifold_fast_remove :: proc "contextless" (
	manifold: ^Nonconvex_Contact_Manifold,
	index: int
) -> Physics_Status
{
	if manifold == nil || index < 0 || index >= int(manifold.count)
	{
		return .Invalid_Argument;
	}
	manifold.count -= 1;
	if index < int(manifold.count)
	{
		manifold.contacts[index] = manifold.contacts[manifold.count];
	}
	manifold.contacts[manifold.count] = {};
	return .Ok;
}

convex_manifold_flip :: proc "contextless" (source: Convex_Contact_Manifold) -> Convex_Contact_Manifold
{
	result := source;
	result.offset_b = util.vector3_negate(source.offset_b);
	result.normal = util.vector3_negate(source.normal);
	for index in 0 ..< int(source.count)
	{
		result.contacts[index].offset = util.vector3_subtract(source.contacts[index].offset, source.offset_b);
	}
	return result;
}

nonconvex_manifold_flip :: proc "contextless" (source: Nonconvex_Contact_Manifold) -> Nonconvex_Contact_Manifold
{
	result := source;
	result.offset_b = util.vector3_negate(source.offset_b);
	for index in 0 ..< int(source.count)
	{
		result.contacts[index].offset = util.vector3_subtract(source.contacts[index].offset, source.offset_b);
		result.contacts[index].normal = util.vector3_negate(source.contacts[index].normal);
	}
	return result;
}

convex_1_manifold_wide_apply_flip_mask :: proc "contextless" (
	manifold: ^Convex_1_Contact_Manifold_Wide, offset_b: ^util.Vector3_Wide, flip_mask: util.I32x8,
) -> Physics_Status
{
	if manifold == nil || offset_b == nil
	{
		return .Invalid_Argument;
	}
	manifold.normal = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(manifold.normal), manifold.normal);
	manifold.offset_a = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a, offset_b^), manifold.offset_a,
	);
	offset_b^ = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(offset_b^), offset_b^);
	return .Ok;
}

convex_2_manifold_wide_apply_flip_mask :: proc "contextless" (
	manifold: ^Convex_2_Contact_Manifold_Wide, offset_b: ^util.Vector3_Wide, flip_mask: util.I32x8,
) -> Physics_Status
{
	if manifold == nil || offset_b == nil
	{
		return .Invalid_Argument;
	}
	manifold.normal = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(manifold.normal), manifold.normal);
	manifold.offset_a_0 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_0, offset_b^), manifold.offset_a_0,
	);
	manifold.offset_a_1 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_1, offset_b^), manifold.offset_a_1,
	);
	offset_b^ = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(offset_b^), offset_b^);
	return .Ok;
}

convex_4_manifold_wide_apply_flip_mask :: proc "contextless" (
	manifold: ^Convex_4_Contact_Manifold_Wide, offset_b: ^util.Vector3_Wide, flip_mask: util.I32x8,
) -> Physics_Status
{
	if manifold == nil || offset_b == nil
	{
		return .Invalid_Argument;
	}
	manifold.normal = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(manifold.normal), manifold.normal);
	manifold.offset_a_0 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_0, offset_b^), manifold.offset_a_0,
	);
	manifold.offset_a_1 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_1, offset_b^), manifold.offset_a_1,
	);
	manifold.offset_a_2 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_2, offset_b^), manifold.offset_a_2,
	);
	manifold.offset_a_3 = util.vector3_wide_select(
		flip_mask, util.vector3_wide_subtract(manifold.offset_a_3, offset_b^), manifold.offset_a_3,
	);
	offset_b^ = util.vector3_wide_select(flip_mask, util.vector3_wide_negate(offset_b^), offset_b^);
	return .Ok;
}

convex_1_manifold_wide_read_lane_into :: proc "contextless" (
	manifold: ^Convex_1_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
	destination: ^Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || destination == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	destination^ = {};
	if simd.extract(manifold.contact_exists, lane) < 0
	{
		destination.count = 1;
		destination.offset_b = util.vector3_wide_read_slot(offset_b, lane);
		destination.normal = util.vector3_wide_read_slot(manifold.normal, lane);
		destination.contacts[0] = {
			offset=util.vector3_wide_read_slot(manifold.offset_a, lane),
			depth=simd.extract(manifold.depth, lane),
			feature_id=simd.extract(manifold.feature_id, lane),
		};
	}
	return .Ok;
}

convex_1_manifold_wide_read_lane :: proc "contextless" (
	manifold: ^Convex_1_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	result: Convex_Contact_Manifold;
	status := convex_1_manifold_wide_read_lane_into(manifold, offset_b, lane, &result);
	return result, status;
}

convex_2_manifold_wide_read_lane_into :: proc "contextless" (
	manifold: ^Convex_2_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
	destination: ^Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || destination == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	destination^ = {};
	if simd.extract(manifold.contact_0_exists, lane) < 0
	{
		destination.contacts[destination.count] = {
			offset=util.vector3_wide_read_slot(manifold.offset_a_0, lane),
			depth=simd.extract(manifold.depth_0, lane),
			feature_id=simd.extract(manifold.feature_id_0, lane),
		};
		destination.count += 1;
	}
	if simd.extract(manifold.contact_1_exists, lane) < 0
	{
		destination.contacts[destination.count] = {
			offset=util.vector3_wide_read_slot(manifold.offset_a_1, lane),
			depth=simd.extract(manifold.depth_1, lane),
			feature_id=simd.extract(manifold.feature_id_1, lane),
		};
		destination.count += 1;
	}
	if destination.count > 0
	{
		destination.offset_b = util.vector3_wide_read_slot(offset_b, lane);
		destination.normal = util.vector3_wide_read_slot(manifold.normal, lane);
	}
	return .Ok;
}

convex_2_manifold_wide_read_lane :: proc "contextless" (
	manifold: ^Convex_2_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	result: Convex_Contact_Manifold;
	status := convex_2_manifold_wide_read_lane_into(manifold, offset_b, lane, &result);
	return result, status;
}

convex_4_manifold_wide_read_lane_core :: proc "contextless" (
	manifold: ^Convex_4_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
	destination: ^Convex_Contact_Manifold,
)
{
	destination.count = 0;
	offsets := [4]^util.Vector3_Wide{
		&manifold.offset_a_0,
		&manifold.offset_a_1,
		&manifold.offset_a_2,
		&manifold.offset_a_3,
	};
	depths := [4]^util.F32x8{&manifold.depth_0, &manifold.depth_1, &manifold.depth_2, &manifold.depth_3};
	feature_ids := [4]^util.I32x8{
		&manifold.feature_id_0,
		&manifold.feature_id_1,
		&manifold.feature_id_2,
		&manifold.feature_id_3,
	};
	exists := [4]^util.I32x8{
		&manifold.contact_0_exists,
		&manifold.contact_1_exists,
		&manifold.contact_2_exists,
		&manifold.contact_3_exists,
	};
	for contact_index in 0 ..< 4
	{
		if simd.extract(exists[contact_index]^, lane) < 0
		{
			destination.contacts[destination.count] = {
				offset=util.vector3_wide_read_slot(offsets[contact_index]^, lane),
				depth=simd.extract(depths[contact_index]^, lane),
				feature_id=simd.extract(feature_ids[contact_index]^, lane),
			};
			destination.count += 1;
		}
	}
	if destination.count > 0
	{
		destination.offset_b = util.vector3_wide_read_slot(offset_b, lane);
		destination.normal = util.vector3_wide_read_slot(manifold.normal, lane);
	}
}

convex_4_manifold_wide_read_lane_into :: proc "contextless" (
	manifold: ^Convex_4_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
	destination: ^Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || destination == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	destination^ = {};
	convex_4_manifold_wide_read_lane_core(manifold, offset_b, lane, destination);
	return .Ok;
}

convex_4_manifold_wide_read_lane :: proc "contextless" (
	manifold: ^Convex_4_Contact_Manifold_Wide, offset_b: util.Vector3_Wide, lane: int,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	result: Convex_Contact_Manifold;
	status := convex_4_manifold_wide_read_lane_into(manifold, offset_b, lane, &result);
	return result, status;
}

convex_1_manifold_wide_write_lane :: proc "contextless" (
	manifold: ^Convex_1_Contact_Manifold_Wide, lane: int, source: Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT || source.count < 0 || source.count > 1
	{
		return .Invalid_Argument;
	}
	manifold.contact_exists = simd.replace(manifold.contact_exists, lane, 0);
	if source.count == 0
	{
		return .Ok;
	}
	util.vector3_wide_write_slot(&manifold.normal, lane, source.normal);
	util.vector3_wide_write_slot(&manifold.offset_a, lane, source.contacts[0].offset);
	manifold.depth = simd.replace(manifold.depth, lane, source.contacts[0].depth);
	manifold.feature_id = simd.replace(manifold.feature_id, lane, source.contacts[0].feature_id);
	manifold.contact_exists = simd.replace(manifold.contact_exists, lane, -1);
	return .Ok;
}

convex_2_manifold_wide_write_lane :: proc "contextless" (
	manifold: ^Convex_2_Contact_Manifold_Wide, lane: int, source: Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT || source.count < 0 || source.count > 2
	{
		return .Invalid_Argument;
	}
	manifold.contact_0_exists = simd.replace(manifold.contact_0_exists, lane, 0);
	manifold.contact_1_exists = simd.replace(manifold.contact_1_exists, lane, 0);
	if source.count == 0
	{
		return .Ok;
	}
	util.vector3_wide_write_slot(&manifold.normal, lane, source.normal);
	offsets := [2]^util.Vector3_Wide{&manifold.offset_a_0, &manifold.offset_a_1};
	depths := [2]^util.F32x8{&manifold.depth_0, &manifold.depth_1};
	feature_ids := [2]^util.I32x8{&manifold.feature_id_0, &manifold.feature_id_1};
	exists := [2]^util.I32x8{&manifold.contact_0_exists, &manifold.contact_1_exists};
	for contact_index in 0 ..< int(source.count)
	{
		util.vector3_wide_write_slot(offsets[contact_index], lane, source.contacts[contact_index].offset);
		depths[contact_index]^ = simd.replace(depths[contact_index]^, lane, source.contacts[contact_index].depth);
		feature_ids[contact_index]^ = simd.replace(
			feature_ids[contact_index]^,
			lane,
			source.contacts[contact_index].feature_id
		);
		exists[contact_index]^ = simd.replace(exists[contact_index]^, lane, -1);
	}
	return .Ok;
}

convex_4_manifold_wide_write_lane :: proc "contextless" (
	manifold: ^Convex_4_Contact_Manifold_Wide, lane: int, source: Convex_Contact_Manifold,
) -> Physics_Status
{
	if manifold == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT || source.count < 0 || source.count > 4
	{
		return .Invalid_Argument;
	}
	exists := [4]^util.I32x8{
		&manifold.contact_0_exists, &manifold.contact_1_exists,
		&manifold.contact_2_exists, &manifold.contact_3_exists,
	};
	for contact_index in 0 ..< 4
	{
		exists[contact_index]^ = simd.replace(exists[contact_index]^, lane, 0);
	}
	if source.count == 0
	{
		return .Ok;
	}
	util.vector3_wide_write_slot(&manifold.normal, lane, source.normal);
	offsets := [4]^util.Vector3_Wide{
		&manifold.offset_a_0, &manifold.offset_a_1,
		&manifold.offset_a_2, &manifold.offset_a_3,
	};
	depths := [4]^util.F32x8{&manifold.depth_0, &manifold.depth_1, &manifold.depth_2, &manifold.depth_3};
	feature_ids := [4]^util.I32x8{
		&manifold.feature_id_0, &manifold.feature_id_1,
		&manifold.feature_id_2, &manifold.feature_id_3,
	};
	for contact_index in 0 ..< int(source.count)
	{
		util.vector3_wide_write_slot(offsets[contact_index], lane, source.contacts[contact_index].offset);
		depths[contact_index]^ = simd.replace(depths[contact_index]^, lane, source.contacts[contact_index].depth);
		feature_ids[contact_index]^ = simd.replace(
			feature_ids[contact_index]^,
			lane,
			source.contacts[contact_index].feature_id
		);
		exists[contact_index]^ = simd.replace(exists[contact_index]^, lane, -1);
	}
	return .Ok;
}

convex_manifold_wide_clear_tail :: proc "contextless" (
	manifold: ^$T, lane_count: int,
) -> Physics_Status
{
	if manifold == nil || lane_count < 0 || lane_count > util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	when T != Convex_1_Contact_Manifold_Wide &&
		T != Convex_2_Contact_Manifold_Wide &&
		T != Convex_4_Contact_Manifold_Wide
	{
		return .Invalid_Argument;
	}
	for lane in lane_count ..< util.PRODUCTION_LANE_COUNT
	{
		if util.gather_clear_lane_checked(manifold, i32, lane) != .Ok
		{
			return .Invalid_Argument;
		}
	}
	return .Ok;
}

#assert(size_of(Contact) == 32);
#assert(size_of(Convex_Contact) == 20);
#assert(size_of(Nonconvex_Contact_Manifold) == 144);
#assert(size_of(Convex_Contact_Manifold) == 108);
#assert(size_of(CCD_Continuation_Index) == 4);
#assert(size_of(Convex_1_Contact_Manifold_Wide) == 288);
#assert(size_of(Convex_2_Contact_Manifold_Wide) == 480);
#assert(size_of(Convex_4_Contact_Manifold_Wide) == 864);
