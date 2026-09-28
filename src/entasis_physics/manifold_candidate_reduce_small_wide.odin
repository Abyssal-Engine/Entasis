// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Hull_Contact_Candidate_Wide :: struct
{
	x, y, depth: util.F32x8,
	feature_id: util.I32x8,
}

hull_contact_candidate_select :: #force_inline proc "contextless" (
	mask: util.I32x8, a, b: Hull_Contact_Candidate_Wide,
) -> Hull_Contact_Candidate_Wide
{
	return {
		x=util.wide_select_f32(mask, a.x, b.x),
		y=util.wide_select_f32(mask, a.y, b.y),
		depth=util.wide_select_f32(mask, a.depth, b.depth),
		feature_id=depth_refiner_select_i32(mask, a.feature_id, b.feature_id),
	};
}

hull_contact_candidate_at :: #force_inline proc "contextless" (
	candidates: ^[8]Hull_Contact_Candidate_Wide, index: util.I32x8,
) -> Hull_Contact_Candidate_Wide
{
	result := candidates[0];
	for i in 1 ..< 8
	{
		result = hull_contact_candidate_select(transmute(util.I32x8)simd.lanes_eq(index, util.I32x8(i32(i))), candidates[i], result);
	}
	return result;
}

hull_contact_candidate_remove :: #force_inline proc "contextless" (
	candidates: ^[8]Hull_Contact_Candidate_Wide, count: ^util.I32x8,
	index, active: util.I32x8,
)
{
	last := hull_contact_candidate_at(candidates, simd.sub(count^, util.I32x8(1)));
	for i in 0 ..< 8
	{
		use_last := active & transmute(util.I32x8)simd.lanes_eq(index, util.I32x8(i32(i)));
		candidates[i] = hull_contact_candidate_select(use_last, last, candidates[i]);
	}
	count^ = simd.sub(count^, active & util.I32x8(1));
}

manifold_candidate_reduce_small_wide :: #force_no_inline proc "contextless" (
	events: ^[12]Hull_Clip_Candidate_Wide, maximum_count, active: util.I32x8,
	face_normal_a: util.Vector3_Wide, inverse_normal_dot_a: util.F32x8,
	center_a, center_b, tangent_x, tangent_y: util.Vector3_Wide,
	epsilon_scale, minimum_depth: util.F32x8, rotation: util.Matrix3x3_Wide,
	offset_b, normal: util.Vector3_Wide,
) -> Convex_4_Contact_Manifold_Wide
{
	candidates: [8]Hull_Contact_Candidate_Wide;
	count: util.I32x8;
	uniform_count := 0;
	uniform := true;
	for event in events^
	{
		accepted := active & event.exists & transmute(util.I32x8)simd.lanes_lt(count, maximum_count);
		if simd.reduce_or(accepted) != 0
		{
			if uniform && simd.reduce_and(accepted | ~active) == -1
			{
				candidates[uniform_count] = {x=event.x, y=event.y, feature_id=event.feature_id};
				uniform_count += 1;
				count = simd.add(count, accepted & util.I32x8(1));
				continue;
			}
			uniform = false;
			first_slot := int(simd.reduce_min(depth_refiner_select_i32(accepted, count, util.I32x8(7))));
			last_slot := int(simd.reduce_max(depth_refiner_select_i32(accepted, count, util.I32x8(0))));
			for i in first_slot ..= last_slot
			{
				use := accepted & transmute(util.I32x8)simd.lanes_eq(count, util.I32x8(i32(i)));
				candidates[i].x = util.wide_select_f32(use, event.x, candidates[i].x);
				candidates[i].y = util.wide_select_f32(use, event.y, candidates[i].y);
				candidates[i].feature_id = depth_refiner_select_i32(use, event.feature_id, candidates[i].feature_id);
			}
			count = simd.add(count, accepted & util.I32x8(1));
		}
	}
	dot_axis := util.vector3_wide_scale(face_normal_a, inverse_normal_dot_a);
	base_dot := util.vector3_wide_dot(util.vector3_wide_subtract(center_b, center_a), dot_axis);
	x_dot := util.vector3_wide_dot(tangent_x, dot_axis);
	y_dot := util.vector3_wide_dot(tangent_y, dot_axis);
	for i := 7; i >= 0; i -= 1
	{
		candidate := &candidates[i];
		candidate.depth = simd.add(simd.add(base_dot, simd.mul(candidate.x, x_dot)), simd.mul(candidate.y, y_dot));
		remove := active & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(i32(i))) &
			transmute(util.I32x8)simd.lanes_lt(candidate.depth, minimum_depth);
		if simd.reduce_or(remove) != 0
		{
			last := hull_contact_candidate_at(&candidates, simd.sub(count, util.I32x8(1)));
			candidate^ = hull_contact_candidate_select(remove, last, candidate^);
			count = simd.sub(count, remove & util.I32x8(1));
		}
	}
	contacts: [4]Hull_Contact_Candidate_Wide;
	exists: [4]util.I32x8;
	for i in 0 ..< 4
	{
		contacts[i] = candidates[i];
		exists[i] = active & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(i32(i)));
	}
	complex := active & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(4));
	if simd.reduce_or(complex) != 0
	{
		best_score := util.F32x8(-f32(math.F32_MAX));
		best_index: util.I32x8;
		extremity_x := simd.mul(simd.mul(util.F32x8(0.7946897654), epsilon_scale), util.F32x8(1e-2));
		extremity_y := simd.mul(simd.mul(util.F32x8(0.60701579614), epsilon_scale), util.F32x8(1e-2));
		for i in 0 ..< 8
		{
			candidate := candidates[i];
			score := candidate.depth;
			bias := simd.abs(simd.add(simd.mul(candidate.x, extremity_x), simd.mul(candidate.y, extremity_y)));
			score = util.wide_select_f32(transmute(util.I32x8)simd.lanes_ge(score, util.F32x8(0)), simd.add(score, bias), score);
			use := complex & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(i32(i))) &
				transmute(util.I32x8)simd.lanes_gt(score, best_score);
			best_score = util.wide_select_f32(use, score, best_score);
			best_index = depth_refiner_select_i32(use, util.I32x8(i32(i)), best_index);
		}
		first := hull_contact_candidate_at(&candidates, best_index);
		contacts[0] = hull_contact_candidate_select(complex, first, contacts[0]);
		hull_contact_candidate_remove(&candidates, &count, best_index, complex);
		maximum_distance := util.F32x8(-1);
		best_index = {};
		for i in 0 ..< 8
		{
			x := simd.sub(candidates[i].x, first.x);
			y := simd.sub(candidates[i].y, first.y);
			distance := simd.add(simd.mul(x, x), simd.mul(y, y));
			use := complex & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(i32(i))) &
				transmute(util.I32x8)simd.lanes_gt(distance, maximum_distance);
			maximum_distance = util.wide_select_f32(use, distance, maximum_distance);
			best_index = depth_refiner_select_i32(use, util.I32x8(i32(i)), best_index);
		}
		use_second := complex & transmute(util.I32x8)simd.lanes_ge(maximum_distance,
			simd.mul(simd.mul(util.F32x8(1e-6), epsilon_scale), epsilon_scale));
		second := hull_contact_candidate_at(&candidates, best_index);
		contacts[1] = hull_contact_candidate_select(use_second, second, contacts[1]);
		exists[1] = (exists[1] & ~complex) | use_second;
		hull_contact_candidate_remove(&candidates, &count, best_index, use_second);
		edge_x := simd.sub(second.x, first.x);
		edge_y := simd.sub(second.y, first.y);
		minimum_area, maximum_area: util.F32x8;
		minimum_index, maximum_index: util.I32x8;
		for i in 0 ..< 8
		{
			x := simd.sub(candidates[i].x, first.x);
			y := simd.sub(candidates[i].y, first.y);
			area := simd.sub(simd.mul(x, edge_y), simd.mul(y, edge_x));
			area = util.wide_select_f32(transmute(util.I32x8)simd.lanes_lt(candidates[i].depth, util.F32x8(0)), simd.mul(area, util.F32x8(0.25)), area);
			present := use_second & transmute(util.I32x8)simd.lanes_gt(count, util.I32x8(i32(i)));
			use_min := present & transmute(util.I32x8)simd.lanes_lt(area, minimum_area);
			use_max := present & transmute(util.I32x8)simd.lanes_gt(area, maximum_area);
			minimum_area = util.wide_select_f32(use_min, area, minimum_area);
			maximum_area = util.wide_select_f32(use_max, area, maximum_area);
			minimum_index = depth_refiner_select_i32(use_min, util.I32x8(i32(i)), minimum_index);
			maximum_index = depth_refiner_select_i32(use_max, util.I32x8(i32(i)), maximum_index);
		}
		area_epsilon := simd.mul(simd.mul(maximum_distance, maximum_distance), util.F32x8(1e-6));
		use_minimum := use_second & transmute(util.I32x8)simd.lanes_gt(simd.mul(minimum_area, minimum_area), area_epsilon);
		use_maximum := use_second & transmute(util.I32x8)simd.lanes_gt(simd.mul(maximum_area, maximum_area), area_epsilon);
		third := hull_contact_candidate_at(&candidates, minimum_index);
		fourth := hull_contact_candidate_at(&candidates, maximum_index);
		contacts[2] = hull_contact_candidate_select(use_minimum, third, contacts[2]);
		contacts[2] = hull_contact_candidate_select(use_maximum & ~use_minimum, fourth, contacts[2]);
		contacts[3] = hull_contact_candidate_select(use_maximum & use_minimum, fourth, contacts[3]);
		exists[2] = (exists[2] & ~complex) | use_minimum | use_maximum;
		exists[3] = (exists[3] & ~complex) | (use_minimum & use_maximum);
	}
	positions: [4]util.Vector3_Wide;
	for i in 0 ..< 4
	{
		local_position := util.vector3_wide_add(center_b, util.vector3_wide_add(
			util.vector3_wide_scale(tangent_x, contacts[i].x), util.vector3_wide_scale(tangent_y, contacts[i].y)));
		positions[i] = util.vector3_wide_add(util.matrix3x3_wide_transform(local_position, rotation), offset_b);
	}
	return {
		offset_a_0=positions[0], offset_a_1=positions[1], offset_a_2=positions[2], offset_a_3=positions[3],
		normal=util.vector3_wide_select(exists[0], normal, {}),
		depth_0=contacts[0].depth, depth_1=contacts[1].depth, depth_2=contacts[2].depth, depth_3=contacts[3].depth,
		feature_id_0=contacts[0].feature_id, feature_id_1=contacts[1].feature_id,
		feature_id_2=contacts[2].feature_id, feature_id_3=contacts[3].feature_id,
		contact_0_exists=exists[0], contact_1_exists=exists[1], contact_2_exists=exists[2], contact_3_exists=exists[3],
	};
}
