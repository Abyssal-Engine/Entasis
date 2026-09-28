// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Sweep_GJK_Simplex_Wide :: struct
{
	vertices: [4]util.Vector3_Wide,
	on_a:     [4]util.Vector3_Wide,
	count:    util.I32x8,
}

sweep_gjk_shape_support_world_wide :: proc "contextless" (
	shape: rawptr, type_id: int, position: util.Vector3_Wide,
	orientation: util.Matrix3x3_Wide, direction: util.Vector3_Wide,
) -> (util.Vector3_Wide, Physics_Status)
{
	if type_id == SPHERE_TYPE_ID
	{
		if sphere_validate((^Sphere)(shape)^) != .Ok
		{
			return {}, .Invalid_Description;
		}
		return position, .Ok;
	}
	if type_id == CAPSULE_TYPE_ID
	{
		capsule := (^Capsule)(shape)^;
		if capsule_validate(capsule) != .Ok
		{
			return {}, .Invalid_Description;
		}
		axis := orientation.y;
		positive := transmute(util.I32x8)simd.lanes_ge(util.vector3_wide_dot(axis, direction), util.F32x8(0));
		scale := util.wide_select_f32(
			positive, util.F32x8(capsule.half_length), util.F32x8(-capsule.half_length),
		);
		return util.vector3_wide_add(position, util.vector3_wide_scale(axis, scale)), .Ok;
	}
	return sweep_shape_support_world_wide(shape, type_id, position, orientation, direction);
}

sweep_gjk_sample_wide :: proc "contextless" (
	ctx: ^Sweep_Distance_Support_Context_Wide, direction: util.Vector3_Wide,
) -> (support_on_a, support: util.Vector3_Wide, status: Physics_Status)
{
	if ctx == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	support_on_a, status = sweep_gjk_shape_support_world_wide(
		ctx.shape_a, ctx.type_a, ctx.position_a, ctx.orientation_a, direction,
	);
	if status != .Ok
	{
		return {}, {}, status;
	}
	support_on_b, b_status := sweep_gjk_shape_support_world_wide(
		ctx.shape_b, ctx.type_b, ctx.position_b, ctx.orientation_b,
		util.vector3_wide_negate(direction),
	);
	if b_status != .Ok
	{
		return {}, {}, b_status;
	}
	return support_on_a, util.vector3_wide_subtract(support_on_a, support_on_b), .Ok;
}

sweep_gjk_append_wide :: proc "contextless" (
	terminated: util.I32x8, simplex: ^Sweep_GJK_Simplex_Wide,
	support_on_a, support: util.Vector3_Wide,
)
{
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(terminated, lane) != 0
		{
			continue;
		}
		count := int(simd.extract(simplex.count, lane));
		util.vector3_wide_write_slot(&simplex.vertices[count], lane, util.vector3_wide_read_slot(support, lane));
		util.vector3_wide_write_slot(&simplex.on_a[count], lane, util.vector3_wide_read_slot(support_on_a, lane));
		simplex.count = simd.replace(simplex.count, lane, i32(count + 1));
	}
}

sweep_gjk_select_wide :: proc "contextless" (
	mask: util.I32x8,
	distance_squared: ^util.F32x8, closest, closest_a: ^util.Vector3_Wide, feature_id: ^util.I32x8,
	candidate_distance_squared: util.F32x8, candidate_closest, candidate_closest_a: util.Vector3_Wide,
	candidate_feature_id: util.I32x8,
)
{
	use_candidate := mask & transmute(util.I32x8)simd.lanes_lt(candidate_distance_squared, distance_squared^);
	distance_squared^ = util.wide_select_f32(use_candidate, candidate_distance_squared, distance_squared^);
	closest^ = util.vector3_wide_select(use_candidate, candidate_closest, closest^);
	closest_a^ = util.vector3_wide_select(use_candidate, candidate_closest_a, closest_a^);
	feature_id^ = depth_refiner_select_i32(use_candidate, candidate_feature_id, feature_id^);
}

sweep_gjk_edge_wide :: proc "contextless" (
	a, b, a_on_a, b_on_a: util.Vector3_Wide, a_feature_id, b_feature_id, mask: util.I32x8,
	distance_squared: ^util.F32x8, closest, closest_a: ^util.Vector3_Wide, feature_id: ^util.I32x8,
) -> (ab: util.Vector3_Wide, ab_ab, ab_a: util.F32x8)
{
	ab = util.vector3_wide_subtract(b, a);
	ab_ab = util.vector3_wide_dot(ab, ab);
	ab_a = util.vector3_wide_dot(ab, a);
	t := simd.div(simd.neg(ab_a), ab_ab);
	t = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(simd.abs(ab_ab), util.F32x8(1e-15)), t, util.F32x8(0),
	);
	a_contribution := depth_refiner_select_i32(
		transmute(util.I32x8)simd.lanes_lt(t, util.F32x8(1)), a_feature_id, util.I32x8(0),
	);
	b_contribution := depth_refiner_select_i32(
		transmute(util.I32x8)simd.lanes_gt(t, util.F32x8(0)), b_feature_id, util.I32x8(0),
	);
	candidate_feature := a_contribution | b_contribution;
	t = simd.max(util.F32x8(0), simd.min(util.F32x8(1), t));
	candidate_closest := util.vector3_wide_add(a, util.vector3_wide_scale(ab, t));
	candidate_closest_a := util.vector3_wide_add(
		a_on_a, util.vector3_wide_scale(util.vector3_wide_subtract(b_on_a, a_on_a), t),
	);
	sweep_gjk_select_wide(
		mask, distance_squared, closest, closest_a, feature_id,
		util.vector3_wide_length_squared(candidate_closest), candidate_closest, candidate_closest_a, candidate_feature,
	);
	return;
}

sweep_gjk_try_remove_wide :: proc "contextless" (
	simplex: ^Sweep_GJK_Simplex_Wide, index: int, feature_id, active_mask: util.I32x8,
)
{
	feature_bit := i32(1) << uint(index);
	should_remove := active_mask & transmute(util.I32x8)simd.lanes_eq(
		feature_id & util.I32x8(feature_bit), util.I32x8(0),
	);
	last_slot := simd.sub(simplex.count, util.I32x8(1));
	simplex.count = depth_refiner_select_i32(should_remove, last_slot, simplex.count);
	should_pull_last := should_remove & transmute(util.I32x8)simd.lanes_lt(util.I32x8(index), last_slot);
	if depth_refiner_any(should_pull_last) == .Missing
	{
		return;
	}
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(should_pull_last, lane) >= 0
		{
			continue;
		}
		source_index := int(simd.extract(last_slot, lane));
		util.vector3_wide_write_slot(
			&simplex.vertices[index], lane, util.vector3_wide_read_slot(simplex.vertices[source_index], lane),
		);
		util.vector3_wide_write_slot(
			&simplex.on_a[index], lane, util.vector3_wide_read_slot(simplex.on_a[source_index], lane),
		);
	}
}

sweep_gjk_triangle_wide :: proc "contextless" (
	a, b, c, a_on_a, b_on_a, c_on_a: util.Vector3_Wide,
	ab_a, ac_a: util.F32x8, ab, ac: util.Vector3_Wide, ab_ab, ac_ac: util.F32x8,
	candidate_feature_id, mask: util.I32x8,
	distance_squared: ^util.F32x8, closest, closest_a: ^util.Vector3_Wide, feature_id: ^util.I32x8,
)
{
	n := util.vector3_wide_cross(ab, ac);
	n_length_squared := util.vector3_wide_length_squared(n);
	a_n := util.vector3_wide_dot(a, n);
	inverse_n_length_squared := simd.div(util.F32x8(1), n_length_squared);
	candidate_closest := util.vector3_wide_scale(n, simd.mul(a_n, inverse_n_length_squared));
	ab_ac := util.vector3_wide_dot(ab, ac);
	c_weight := simd.mul(simd.sub(simd.mul(ab_a, ab_ac), simd.mul(ac_a, ab_ab)), inverse_n_length_squared);
	b_weight := simd.mul(simd.sub(simd.mul(ab_ac, ac_a), simd.mul(ac_ac, ab_a)), inverse_n_length_squared);
	a_weight := simd.sub(simd.sub(util.F32x8(1), b_weight), c_weight);
	projection_in_triangle :=
		transmute(util.I32x8)simd.lanes_ge(a_weight, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(b_weight, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(c_weight, util.F32x8(0));
	candidate_closest_a := util.vector3_wide_add(
		util.vector3_wide_add(
		util.vector3_wide_scale(a_on_a, a_weight), util.vector3_wide_scale(b_on_a, b_weight),
	),
		util.vector3_wide_scale(c_on_a, c_weight),
	);
	sweep_gjk_select_wide(
		mask & projection_in_triangle, distance_squared, closest, closest_a, feature_id,
		util.vector3_wide_length_squared(candidate_closest), candidate_closest, candidate_closest_a,
		candidate_feature_id,
	);
}

sweep_gjk_find_closest_wide :: proc "contextless" (
	terminated: util.I32x8, simplex: ^Sweep_GJK_Simplex_Wide,
) -> (distance_squared: util.F32x8, closest_a, closest: util.Vector3_Wide)
{
	active_mask := ~terminated;
	distance_squared = util.vector3_wide_length_squared(simplex.vertices[0]);
	feature_id := util.I32x8(1);
	closest = simplex.vertices[0];
	closest_a = simplex.on_a[0];
	active_mask &= transmute(util.I32x8)simd.lanes_ge(simplex.count, util.I32x8(2));
	if depth_refiner_any(active_mask) == .Missing
	{
		return;
	}
	ab, ab_ab, ab_a := sweep_gjk_edge_wide(
		simplex.vertices[0], simplex.vertices[1], simplex.on_a[0], simplex.on_a[1],
		util.I32x8(1), util.I32x8(2), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	next_active := active_mask & transmute(util.I32x8)simd.lanes_ge(simplex.count, util.I32x8(3));
	if depth_refiner_any(next_active) == .Missing
	{
		sweep_gjk_try_remove_wide(simplex, 1, feature_id, active_mask);
		sweep_gjk_try_remove_wide(simplex, 0, feature_id, active_mask);
		return;
	}
	active_mask = next_active;
	ac, ac_ac, ac_a := sweep_gjk_edge_wide(
		simplex.vertices[0], simplex.vertices[2], simplex.on_a[0], simplex.on_a[2],
		util.I32x8(1), util.I32x8(4), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	bc, bc_bc, bc_b := sweep_gjk_edge_wide(
		simplex.vertices[1], simplex.vertices[2], simplex.on_a[1], simplex.on_a[2],
		util.I32x8(2), util.I32x8(4), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	sweep_gjk_triangle_wide(
		simplex.vertices[0], simplex.vertices[1], simplex.vertices[2],
		simplex.on_a[0], simplex.on_a[1], simplex.on_a[2],
		ab_a, ac_a, ab, ac, ab_ab, ac_ac, util.I32x8(1 | 2 | 4), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	next_active = active_mask & transmute(util.I32x8)simd.lanes_ge(simplex.count, util.I32x8(4));
	if depth_refiner_any(next_active) == .Missing
	{
		sweep_gjk_try_remove_wide(simplex, 2, feature_id, active_mask);
		sweep_gjk_try_remove_wide(simplex, 1, feature_id, active_mask);
		sweep_gjk_try_remove_wide(simplex, 0, feature_id, active_mask);
		return;
	}
	active_mask = next_active;
	ad, ad_ad, ad_a := sweep_gjk_edge_wide(
		simplex.vertices[0], simplex.vertices[3], simplex.on_a[0], simplex.on_a[3],
		util.I32x8(1), util.I32x8(8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	bd, bd_bd, bd_b := sweep_gjk_edge_wide(
		simplex.vertices[1], simplex.vertices[3], simplex.on_a[1], simplex.on_a[3],
		util.I32x8(2), util.I32x8(8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	cd, _, _ := sweep_gjk_edge_wide(
		simplex.vertices[2], simplex.vertices[3], simplex.on_a[2], simplex.on_a[3],
		util.I32x8(4), util.I32x8(8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	sweep_gjk_triangle_wide(
		simplex.vertices[0], simplex.vertices[2], simplex.vertices[3],
		simplex.on_a[0], simplex.on_a[2], simplex.on_a[3],
		ac_a, ad_a, ac, ad, ac_ac, ad_ad, util.I32x8(1 | 4 | 8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	sweep_gjk_triangle_wide(
		simplex.vertices[0], simplex.vertices[1], simplex.vertices[3],
		simplex.on_a[0], simplex.on_a[1], simplex.on_a[3],
		ab_a, ad_a, ab, ad, ab_ab, ad_ad, util.I32x8(1 | 2 | 8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	sweep_gjk_triangle_wide(
		simplex.vertices[1], simplex.vertices[2], simplex.vertices[3],
		simplex.on_a[1], simplex.on_a[2], simplex.on_a[3],
		bc_b, bd_b, bc, bd, bc_bc, bd_bd, util.I32x8(2 | 4 | 8), active_mask,
		&distance_squared, &closest, &closest_a, &feature_id,
	);
	n_abc := util.vector3_wide_cross(bc, ab);
	n_abd := util.vector3_wide_cross(ad, bd);
	n_acd := util.vector3_wide_cross(cd, ac);
	n_bdc := util.vector3_wide_cross(bd, cd);
	flip_required := transmute(util.I32x8)simd.lanes_ge(util.vector3_wide_dot(ad, n_abc), util.F32x8(0));
	abc_inside := (transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(n_abc, simplex.vertices[0]),
		util.F32x8(0)
	)) ~ flip_required;
	abd_inside := (transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(n_abd, simplex.vertices[0]),
		util.F32x8(0)
	)) ~ flip_required;
	acd_inside := (transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(n_acd, simplex.vertices[0]),
		util.F32x8(0)
	)) ~ flip_required;
	bdc_inside := (transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(n_bdc, simplex.vertices[1]),
		util.F32x8(0)
	)) ~ flip_required;
	use_tetrahedron := active_mask & abc_inside & abd_inside & acd_inside & bdc_inside;
	sweep_gjk_select_wide(
		use_tetrahedron, &distance_squared, &closest, &closest_a, &feature_id,
		{}, {}, {}, util.I32x8(1 | 2 | 4 | 8),
	);
	sweep_gjk_try_remove_wide(simplex, 3, feature_id, active_mask);
	sweep_gjk_try_remove_wide(simplex, 2, feature_id, active_mask);
	sweep_gjk_try_remove_wide(simplex, 1, feature_id, active_mask);
	sweep_gjk_try_remove_wide(simplex, 0, feature_id, active_mask);
	return;
}

sweep_gjk_distance_wide :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide,
) -> (Sweep_Distance_Wide_Result, Physics_Status)
{
	ctx := Sweep_Distance_Support_Context_Wide{
		shape_a=shape_a, shape_b=shape_b, type_a=type_a, type_b=type_b,
		orientation_a=util.matrix3x3_wide_from_quaternion(orientation_a),
		orientation_b=util.matrix3x3_wide_from_quaternion(orientation_b),
		position_b=offset_b,
	};
	initial_on_a, initial, initial_status := sweep_gjk_sample_wide(&ctx, offset_b);
	if initial_status != .Ok
	{
		return {}, initial_status;
	}
	simplex := Sweep_GJK_Simplex_Wide{
		vertices={0=initial}, on_a={0=initial_on_a}, count=util.I32x8(1),
	};
	terminated: util.I32x8;
	intersected: util.I32x8;
	distance_squared := util.F32x8(f32(math.F32_MAX));
	closest_a, normal: util.Vector3_Wide;
	epsilon := util.F32x8(GJK_DISTANCE_TERMINATION_EPSILON);
	containment_epsilon_scalar := GJK_DISTANCE_CONTAINMENT_EPSILON;
	if type_a == SPHERE_TYPE_ID
	{
		containment_epsilon_scalar = max(containment_epsilon_scalar, (^Sphere)(shape_a).radius);
	}
	if type_a == CAPSULE_TYPE_ID
	{
		containment_epsilon_scalar = max(containment_epsilon_scalar, (^Capsule)(shape_a).radius);
	}
	if type_b == SPHERE_TYPE_ID
	{
		containment_epsilon_scalar = max(containment_epsilon_scalar, (^Sphere)(shape_b).radius);
	}
	if type_b == CAPSULE_TYPE_ID
	{
		containment_epsilon_scalar = max(containment_epsilon_scalar, (^Capsule)(shape_b).radius);
	}
	containment_epsilon := util.F32x8(containment_epsilon_scalar);
	containment_squared := simd.mul(containment_epsilon, containment_epsilon);
	for
	{
		new_distance_squared, simplex_closest_a, simplex_closest := sweep_gjk_find_closest_wide(terminated, &simplex);
		contains_origin := transmute(util.I32x8)simd.lanes_le(new_distance_squared, containment_squared);
		intersected |= contains_origin & ~terminated;
		terminated |= contains_origin;
		if depth_refiner_all(terminated) == .Present
		{
			break;
		}
		no_progress := transmute(util.I32x8)simd.lanes_ge(new_distance_squared, distance_squared);
		distance_squared = simd.min(distance_squared, new_distance_squared);
		about_to_terminate := no_progress & ~terminated;
		closest_a = util.vector3_wide_select(about_to_terminate, simplex_closest_a, closest_a);
		normal = util.vector3_wide_select(about_to_terminate, simplex_closest, normal);
		terminated |= about_to_terminate;
		if depth_refiner_all(terminated) == .Present
		{
			break;
		}
		sample_direction := util.vector3_wide_negate(simplex_closest);
		support_on_a, support, support_status := sweep_gjk_sample_wide(&ctx, sample_direction);
		if support_status != .Ok
		{
			return {}, support_status;
		}
		progress := util.vector3_wide_dot(util.vector3_wide_subtract(support, simplex_closest), sample_direction);
		maximum_vertex_distance_squared := util.vector3_wide_length_squared(simplex.vertices[0]);
		for vertex_index in 1 ..< 4
		{
			vertex_distance_squared := util.vector3_wide_length_squared(simplex.vertices[vertex_index]);
			vertex_active := transmute(util.I32x8)simd.lanes_gt(simplex.count, util.I32x8(vertex_index));
			maximum_vertex_distance_squared = util.wide_select_f32(
				vertex_active & transmute(util.I32x8)simd.lanes_gt(
				vertex_distance_squared,
				maximum_vertex_distance_squared
			),
				vertex_distance_squared, maximum_vertex_distance_squared,
			);
		}
		no_progress = transmute(util.I32x8)simd.lanes_le(
			progress, simd.mul(maximum_vertex_distance_squared, epsilon),
		);
		about_to_terminate = no_progress & ~terminated;
		closest_a = util.vector3_wide_select(about_to_terminate, simplex_closest_a, closest_a);
		normal = util.vector3_wide_select(about_to_terminate, simplex_closest, normal);
		terminated |= about_to_terminate;
		if depth_refiner_all(terminated) == .Present
		{
			break;
		}
		sweep_gjk_append_wide(terminated, &simplex, support_on_a, support);
	}
	original_distance := simd.sqrt(distance_squared);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), original_distance));
	closest_a = util.vector3_wide_add(
		closest_a, util.vector3_wide_scale(normal, simd.neg(containment_epsilon)),
	);
	return {
		intersected=intersected,
		distance=simd.sub(original_distance, containment_epsilon),
		closest_a=closest_a, normal=normal,
	}, .Ok;
}
