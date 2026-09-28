// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Depth_Refiner_Support_State :: enum u8
{
	Reuse_Simplex,
	Add_Support,
}

Depth_Refiner_Support_Wide_Proc :: #type proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status);

Depth_Refiner_Vertex_With_Witness_Wide :: struct
{
	support:      util.Vector3_Wide,
	support_on_a: util.Vector3_Wide,
	weight:       util.F32x8,
	exists:       util.I32x8,
}

Depth_Refiner_Simplex_With_Witness_Wide :: struct
{
	a, b, c:          Depth_Refiner_Vertex_With_Witness_Wide,
	weight_denominator: util.F32x8,
}

depth_refiner_select_i32 :: proc "contextless" (condition, left, right: util.I32x8) -> util.I32x8
{
	return simd.select(transmute(util.Mask32x8)condition, left, right);
}

depth_refiner_any :: proc "contextless" (mask: util.I32x8) -> Reference_State
{
	if transmute(u8)simd.extract_msbs(mask) != 0
	{
		return .Present;
	}
	return .Missing;
}

depth_refiner_all :: proc "contextless" (mask: util.I32x8) -> Reference_State
{
	if transmute(u8)simd.extract_msbs(mask) == u8(0xff)
	{
		return .Present;
	}
	return .Missing;
}

depth_refiner_fill_slot_wide :: proc "contextless" (
	vertex: ^Depth_Refiner_Vertex_With_Witness_Wide,
	support, support_on_a: util.Vector3_Wide,
	terminated_lanes: util.I32x8,
)
{
	dont_fill_slot := vertex.exists | terminated_lanes;
	if depth_refiner_all(dont_fill_slot) == .Present
	{
		return;
	}
	vertex.support = util.vector3_wide_select(dont_fill_slot, vertex.support, support);
	vertex.support_on_a = util.vector3_wide_select(dont_fill_slot, vertex.support_on_a, support_on_a);
	vertex.exists = depth_refiner_select_i32(dont_fill_slot, vertex.exists, util.I32x8(-1));
}

depth_refiner_force_fill_slot_wide :: proc "contextless" (
	should_fill: util.I32x8,
	vertex: ^Depth_Refiner_Vertex_With_Witness_Wide,
	support, support_on_a: util.Vector3_Wide,
)
{
	vertex.exists |= should_fill;
	vertex.support = util.vector3_wide_select(should_fill, support, vertex.support);
	vertex.support_on_a = util.vector3_wide_select(should_fill, support_on_a, vertex.support_on_a);
}

depth_refiner_create_simplex_wide :: proc "contextless" (
	support, support_on_a: util.Vector3_Wide,
) -> Depth_Refiner_Simplex_With_Witness_Wide
{
	return {
		a={support=support, support_on_a=support_on_a, weight=util.F32x8(1), exists=util.I32x8(-1)},
		b={support=support, support_on_a=support_on_a},
		c={support=support, support_on_a=support_on_a},
		weight_denominator=util.F32x8(1),
	};
}

depth_refiner_get_next_normal_wide :: proc "contextless" (
	simplex: ^Depth_Refiner_Simplex_With_Witness_Wide,
	support, support_on_a: util.Vector3_Wide,
	terminated_lanes: ^util.I32x8,
	best_normal: util.Vector3_Wide,
	best_depth, convergence_threshold: util.F32x8,
	$support_state: Depth_Refiner_Support_State,
) -> util.Vector3_Wide
{
	search_target := util.vector3_wide_scale(best_normal, simd.max(util.F32x8(0), best_depth));
	negative_depth := transmute(util.I32x8)simd.lanes_lt(best_depth, util.F32x8(0));
	termination_epsilon := util.wide_select_f32(
		negative_depth, simd.sub(convergence_threshold, best_depth), convergence_threshold,
	);
	termination_epsilon_squared := simd.mul(termination_epsilon, termination_epsilon);

	if support_state == .Add_Support
	{
		simplex_full := (simplex.a.exists & simplex.b.exists & simplex.c.exists) & ~terminated_lanes^;
		depth_refiner_fill_slot_wide(&simplex.a, support, support_on_a, terminated_lanes^);
		depth_refiner_fill_slot_wide(&simplex.b, support, support_on_a, terminated_lanes^);
		depth_refiner_fill_slot_wide(&simplex.c, support, support_on_a, terminated_lanes^);
		if depth_refiner_any(simplex_full) == .Present
		{
			ab_early := util.vector3_wide_subtract(simplex.b.support, simplex.a.support);
			ca_early := util.vector3_wide_subtract(simplex.a.support, simplex.c.support);
			ad := util.vector3_wide_subtract(support, simplex.a.support);
			bd := util.vector3_wide_subtract(support, simplex.b.support);
			cd := util.vector3_wide_subtract(support, simplex.c.support);
			triangle_normal_early := util.vector3_wide_cross(ab_early, ca_early);
			target_to_support := util.vector3_wide_subtract(support, search_target);
			nx_offset := util.vector3_wide_cross(triangle_normal_early, target_to_support);
			ad_plane_test := util.vector3_wide_dot(nx_offset, ad);
			bd_plane_test := util.vector3_wide_dot(nx_offset, bd);
			cd_plane_test := util.vector3_wide_dot(nx_offset, cd);
			use_abd := transmute(util.I32x8)simd.lanes_ge(ad_plane_test, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(bd_plane_test, util.F32x8(0));
			use_bcd := transmute(util.I32x8)simd.lanes_ge(bd_plane_test, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(cd_plane_test, util.F32x8(0));
			use_cad := transmute(util.I32x8)simd.lanes_ge(cd_plane_test, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(ad_plane_test, util.F32x8(0));
			use_abd = depth_refiner_select_i32(~(use_abd | use_bcd | use_cad), util.I32x8(-1), use_abd);
			depth_refiner_force_fill_slot_wide(use_bcd & simplex_full, &simplex.a, support, support_on_a);
			depth_refiner_force_fill_slot_wide(use_cad & simplex_full, &simplex.b, support, support_on_a);
			depth_refiner_force_fill_slot_wide(use_abd & simplex_full, &simplex.c, support, support_on_a);
		}
	}
	else
	{
		depth_refiner_fill_slot_wide(&simplex.a, simplex.a.support, simplex.a.support_on_a, terminated_lanes^);
		depth_refiner_fill_slot_wide(&simplex.b, simplex.a.support, simplex.a.support_on_a, terminated_lanes^);
		depth_refiner_fill_slot_wide(&simplex.c, simplex.a.support, simplex.a.support_on_a, terminated_lanes^);
	}

	ab := util.vector3_wide_subtract(simplex.b.support, simplex.a.support);
	ca := util.vector3_wide_subtract(simplex.a.support, simplex.c.support);
	bc := util.vector3_wide_subtract(simplex.c.support, simplex.b.support);
	triangle_normal := util.vector3_wide_cross(ab, ca);
	triangle_normal_length_squared := util.vector3_wide_length_squared(triangle_normal);
	target_to_a := util.vector3_wide_subtract(simplex.a.support, search_target);
	target_to_c := util.vector3_wide_subtract(simplex.c.support, search_target);
	ab_cross_target_a := util.vector3_wide_cross(ab, target_to_a);
	ca_cross_target_c := util.vector3_wide_cross(ca, target_to_c);
	ab_plane_test := util.vector3_wide_dot(ab_cross_target_a, triangle_normal);
	ca_plane_test := util.vector3_wide_dot(ca_cross_target_c, triangle_normal);
	bc_plane_test := simd.sub(simd.sub(triangle_normal_length_squared, ca_plane_test), ab_plane_test);
	outside_ab := transmute(util.I32x8)simd.lanes_lt(ab_plane_test, util.F32x8(0));
	outside_bc := transmute(util.I32x8)simd.lanes_lt(bc_plane_test, util.F32x8(0));
	outside_ca := transmute(util.I32x8)simd.lanes_lt(ca_plane_test, util.F32x8(0));
	ab_length_squared := util.vector3_wide_length_squared(ab);
	bc_length_squared := util.vector3_wide_length_squared(bc);
	ca_length_squared := util.vector3_wide_length_squared(ca);
	longest_edge_length_squared := simd.max(simd.max(ab_length_squared, bc_length_squared), ca_length_squared);
	simplex_degenerate := transmute(util.I32x8)simd.lanes_le(
		triangle_normal_length_squared, simd.mul(longest_edge_length_squared, util.F32x8(1e-10)),
	);
	simplex_is_a_vertex := transmute(util.I32x8)simd.lanes_lt(longest_edge_length_squared, util.F32x8(1e-14));
	simplex_is_an_edge := simplex_degenerate & ~simplex_is_a_vertex;
	calibration_dot := util.vector3_wide_dot(triangle_normal, best_normal);
	triangle_normal = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_lt(calibration_dot, util.F32x8(0)), triangle_normal,
	);
	target_outside_triangle_edges := outside_ab | outside_bc | outside_ca;
	triangle_to_target := util.vector3_wide_negate(target_to_a);
	relevant_features := util.I32x8(1);
	simplex.a.weight = util.wide_select_f32(terminated_lanes^, simplex.a.weight, util.F32x8(1));
	simplex.b.weight = util.wide_select_f32(terminated_lanes^, simplex.b.weight, util.F32x8(0));
	simplex.c.weight = util.wide_select_f32(terminated_lanes^, simplex.c.weight, util.F32x8(0));
	simplex.weight_denominator = util.wide_select_f32(terminated_lanes^, simplex.weight_denominator, util.F32x8(1));
	target_to_a_length_squared := util.vector3_wide_length_squared(target_to_a);
	terminated_lanes^ |= simplex_is_a_vertex &
		transmute(util.I32x8)simd.lanes_lt(target_to_a_length_squared, termination_epsilon_squared);

	use_edge := (target_outside_triangle_edges | simplex_is_an_edge) & ~terminated_lanes^;
	if depth_refiner_any(use_edge) == .Present
	{
		inverse_ab_length_squared := simd.div(util.F32x8(1), ab_length_squared);
		inverse_bc_length_squared := simd.div(util.F32x8(1), bc_length_squared);
		inverse_ca_length_squared := simd.div(util.F32x8(1), ca_length_squared);
		target_to_b := util.vector3_wide_subtract(simplex.b.support, search_target);
		oa_dot_ab := util.vector3_wide_dot(target_to_a, ab);
		ob_dot_bc := util.vector3_wide_dot(target_to_b, bc);
		oc_dot_ca := util.vector3_wide_dot(target_to_c, ca);
		ab_scaled_t := simd.max(util.F32x8(0), simd.min(ab_length_squared, simd.neg(oa_dot_ab)));
		bc_scaled_t := simd.max(util.F32x8(0), simd.min(bc_length_squared, simd.neg(ob_dot_bc)));
		ca_scaled_t := simd.max(util.F32x8(0), simd.min(ca_length_squared, simd.neg(oc_dot_ca)));
		ab_t := simd.mul(ab_scaled_t, inverse_ab_length_squared);
		bc_t := simd.mul(bc_scaled_t, inverse_bc_length_squared);
		ca_t := simd.mul(ca_scaled_t, inverse_ca_length_squared);
		ab_closest_offset := util.vector3_wide_add(target_to_a, util.vector3_wide_scale(ab, ab_t));
		bc_closest_offset := util.vector3_wide_add(target_to_b, util.vector3_wide_scale(bc, bc_t));
		ca_closest_offset := util.vector3_wide_add(target_to_c, util.vector3_wide_scale(ca, ca_t));
		ab_distance_squared := util.vector3_wide_length_squared(ab_closest_offset);
		bc_distance_squared := util.vector3_wide_length_squared(bc_closest_offset);
		ca_distance_squared := util.vector3_wide_length_squared(ca_closest_offset);
		bc_degenerate := transmute(util.I32x8)simd.lanes_eq(bc_length_squared, util.F32x8(0));
		ca_degenerate := transmute(util.I32x8)simd.lanes_eq(ca_length_squared, util.F32x8(0));
		ab_closer_than_bc := bc_degenerate | transmute(util.I32x8)simd.lanes_lt(
			ab_distance_squared,
			bc_distance_squared
		);
		ab_closer_than_ca := ca_degenerate | transmute(util.I32x8)simd.lanes_lt(
			ab_distance_squared,
			ca_distance_squared
		);
		bc_closer_than_ca := ca_degenerate | transmute(util.I32x8)simd.lanes_lt(
			bc_distance_squared,
			ca_distance_squared
		);
		use_ab := ab_closer_than_bc & ab_closer_than_ca;
		use_bc := bc_closer_than_ca & ~use_ab;
		best_distance_squared := util.wide_select_f32(
			use_ab,
			ab_distance_squared,
			util.wide_select_f32(use_bc, bc_distance_squared, ca_distance_squared)
		);
		terminated_lanes^ |= use_edge & transmute(util.I32x8)simd.lanes_le(
			best_distance_squared,
			termination_epsilon_squared
		);

		t := util.wide_select_f32(use_ab, ab_t, util.wide_select_f32(use_bc, bc_t, ca_t));
		edge_offset := util.vector3_wide_select(use_ab, ab, ca);
		edge_start := util.vector3_wide_select(use_ab, target_to_a, target_to_c);
		edge_offset = util.vector3_wide_select(use_bc, bc, edge_offset);
		edge_start = util.vector3_wide_select(use_bc, target_to_b, edge_start);
		triangle_to_target_candidate := util.vector3_wide_subtract(
			util.vector3_wide_scale(edge_offset, simd.neg(t)),
			edge_start
		);
		origin_nearest_start := transmute(util.I32x8)simd.lanes_eq(t, util.F32x8(0));
		origin_nearest_end := transmute(util.I32x8)simd.lanes_eq(t, util.F32x8(1));
		feature_for_ab := depth_refiner_select_i32(
			origin_nearest_start,
			util.I32x8(1),
			depth_refiner_select_i32(origin_nearest_end, util.I32x8(2), util.I32x8(3))
		);
		feature_for_bc := depth_refiner_select_i32(
			origin_nearest_start,
			util.I32x8(2),
			depth_refiner_select_i32(origin_nearest_end, util.I32x8(4), util.I32x8(6))
		);
		feature_for_ca := depth_refiner_select_i32(
			origin_nearest_start,
			util.I32x8(4),
			depth_refiner_select_i32(origin_nearest_end, util.I32x8(1), util.I32x8(5))
		);
		selected_feature := depth_refiner_select_i32(
			use_ab,
			feature_for_ab,
			depth_refiner_select_i32(use_bc, feature_for_bc, feature_for_ca)
		);
		relevant_features = depth_refiner_select_i32(use_edge, selected_feature, relevant_features);
		triangle_to_target = util.vector3_wide_select(use_edge, triangle_to_target_candidate, triangle_to_target);
		weight_edge_start := simd.sub(util.F32x8(1), t);
		simplex.a.weight = util.wide_select_f32(
			use_edge,
			util.wide_select_f32(use_ab, weight_edge_start, util.wide_select_f32(use_bc, util.F32x8(0), t)),
			simplex.a.weight
		);
		simplex.b.weight = util.wide_select_f32(
			use_edge,
			util.wide_select_f32(use_ab, t, util.wide_select_f32(use_bc, weight_edge_start, util.F32x8(0))),
			simplex.b.weight
		);
		simplex.c.weight = util.wide_select_f32(
			use_edge,
			util.wide_select_f32(use_ab, util.F32x8(0), util.wide_select_f32(use_bc, t, weight_edge_start)),
			simplex.c.weight
		);
	}

	target_contained_in_edge_planes := (~target_outside_triangle_edges & ~simplex_degenerate) & ~terminated_lanes^;
	if depth_refiner_any(target_contained_in_edge_planes) == .Present
	{
		target_to_a_dot := util.vector3_wide_dot(target_to_a, triangle_normal);
		target_on_triangle_surface := transmute(util.I32x8)simd.lanes_lt(
			simd.mul(target_to_a_dot, target_to_a_dot),
			simd.mul(termination_epsilon_squared, triangle_normal_length_squared),
		);
		terminated_lanes^ |= target_contained_in_edge_planes & target_on_triangle_surface;
		triangle_to_target = util.vector3_wide_select(
			target_contained_in_edge_planes,
			triangle_normal,
			triangle_to_target
		);
		relevant_features = depth_refiner_select_i32(target_contained_in_edge_planes, util.I32x8(7), relevant_features);
		simplex.a.weight = util.wide_select_f32(target_contained_in_edge_planes, bc_plane_test, simplex.a.weight);
		simplex.b.weight = util.wide_select_f32(target_contained_in_edge_planes, ca_plane_test, simplex.b.weight);
		simplex.c.weight = util.wide_select_f32(target_contained_in_edge_planes, ab_plane_test, simplex.c.weight);
		simplex.weight_denominator = util.wide_select_f32(
			target_contained_in_edge_planes,
			triangle_normal_length_squared,
			simplex.weight_denominator
		);
	}

	simplex.a.exists = transmute(util.I32x8)simd.lanes_gt(relevant_features & util.I32x8(1), util.I32x8(0));
	simplex.b.exists = transmute(util.I32x8)simd.lanes_gt(relevant_features & util.I32x8(2), util.I32x8(0));
	simplex.c.exists = transmute(util.I32x8)simd.lanes_gt(relevant_features & util.I32x8(4), util.I32x8(0));
	if depth_refiner_all(terminated_lanes^) == .Missing
	{
		push_offset := util.vector3_wide_scale(triangle_to_target, util.F32x8(4));
		push_normal_candidate := util.vector3_wide_add(search_target, push_offset);
		use_direct := transmute(util.I32x8)simd.lanes_le(best_depth, util.F32x8(0)) | target_contained_in_edge_planes;
		triangle_to_target = util.vector3_wide_select(use_direct, triangle_to_target, push_normal_candidate);
		return util.vector3_wide_scale(
			triangle_to_target, simd.div(
			util.F32x8(1),
			simd.sqrt(util.vector3_wide_length_squared(triangle_to_target))
		),
		);
	}
	return {};
}

depth_refiner_find_minimum_depth_wide :: proc "contextless" (
	$support_proc: Depth_Refiner_Support_Wide_Proc,
	user_context: rawptr,
	initial_normal: util.Vector3_Wide,
	inactive_lanes: util.I32x8,
	convergence_threshold, minimum_depth_threshold, margin_a, margin_b: util.F32x8,
	maximum_iterations: int = DEPTH_REFINER_INITIAL_MAXIMUM_ITERATIONS,
) -> (refined_depth: util.F32x8, refined_normal, witness_on_a: util.Vector3_Wide, status: Physics_Status)
{
	if support_proc == nil || maximum_iterations <= 0
	{
		return {}, {}, {}, .Invalid_Argument;
	}
	initial_support, initial_support_on_a, support_status := support_proc(user_context, initial_normal, inactive_lanes);
	if support_status != .Ok
	{
		return {}, {}, {}, support_status;
	}
	initial_depth := util.vector3_wide_dot(initial_support, initial_normal);
	simplex := depth_refiner_create_simplex_wide(initial_support, initial_support_on_a);
	depth_threshold := simd.sub(simd.sub(minimum_depth_threshold, margin_a), margin_b);
	terminated_lanes := transmute(util.I32x8)simd.lanes_lt(initial_depth, depth_threshold) | inactive_lanes;
	refined_normal = initial_normal;
	refined_depth = initial_depth;
	if depth_refiner_all(terminated_lanes) == .Present
	{
		return refined_depth, refined_normal, {}, .Ok;
	}
	normal := depth_refiner_get_next_normal_wide(
		&simplex, {}, {}, &terminated_lanes, refined_normal, refined_depth, convergence_threshold, .Reuse_Simplex,
	);
	for _ in 0 ..< maximum_iterations
	{
		if depth_refiner_all(terminated_lanes) == .Present
		{
			break;
		}
		support, support_on_a, next_support_status := support_proc(user_context, normal, terminated_lanes);
		if next_support_status != .Ok
		{
			return {}, {}, {}, next_support_status;
		}
		depth := util.vector3_wide_dot(support, normal);
		use_new_depth := transmute(util.I32x8)simd.lanes_lt(depth, refined_depth) & ~terminated_lanes;
		refined_depth = util.wide_select_f32(use_new_depth, depth, refined_depth);
		refined_normal = util.vector3_wide_select(use_new_depth, normal, refined_normal);
		terminated_lanes |= transmute(util.I32x8)simd.lanes_le(refined_depth, depth_threshold);
		if depth_refiner_all(terminated_lanes) == .Present
		{
			break;
		}
		normal = depth_refiner_get_next_normal_wide(
			&simplex, support, support_on_a, &terminated_lanes,
			refined_normal, refined_depth, convergence_threshold, .Add_Support,
		);
	}
	refined_depth = simd.add(simd.add(refined_depth, margin_a), margin_b);
	inverse_denominator := simd.div(util.F32x8(1), simplex.weight_denominator);
	weighted_a := util.vector3_wide_scale(simplex.a.support_on_a, simd.mul(simplex.a.weight, inverse_denominator));
	weighted_b := util.vector3_wide_scale(simplex.b.support_on_a, simd.mul(simplex.b.weight, inverse_denominator));
	weighted_c := util.vector3_wide_scale(simplex.c.support_on_a, simd.mul(simplex.c.weight, inverse_denominator));
	witness_on_a = util.vector3_wide_add(util.vector3_wide_add(weighted_a, weighted_b), weighted_c);
	witness_on_a = util.vector3_wide_add(witness_on_a, util.vector3_wide_scale(refined_normal, margin_a));
	return refined_depth, refined_normal, witness_on_a, .Ok;
}
