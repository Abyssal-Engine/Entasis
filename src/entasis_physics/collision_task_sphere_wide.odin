// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

collision_active_lane_mask :: proc "contextless" (pair_count: int) -> (util.I32x8, Physics_Status)
{
	mask, status := util.bundle_count_mask(pair_count);
	if status != .Ok
	{
		return {}, .Invalid_Argument;
	}
	return mask, .Ok;
}

sphere_pair_test_wide :: proc "contextless" (
	a, b: Sphere_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_1_Contact_Manifold_Wide;
	center_distance := util.vector3_wide_length(offset_b);
	manifold.normal = util.vector3_wide_scale(offset_b, simd.div(util.F32x8(-1), center_distance));
	normal_is_valid := transmute(util.I32x8)simd.lanes_gt(center_distance, util.F32x8(0));
	manifold.normal = util.vector3_wide_select(normal_is_valid, manifold.normal, {y=util.F32x8(1)});
	manifold.depth = simd.sub(simd.add(a.radius, b.radius), center_distance);
	negative_offset_from_a := simd.sub(simd.mul(manifold.depth, util.F32x8(0.5)), a.radius);
	manifold.offset_a = util.vector3_wide_scale(manifold.normal, negative_offset_from_a);
	manifold.contact_exists = transmute(util.I32x8)simd.lanes_gt(manifold.depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}

sphere_capsule_test_wide :: proc "contextless" (
	a: Sphere_Wide, b: Capsule_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_1_Contact_Manifold_Wide;
	x, y := util.quaternion_wide_transform_unit_xy(orientation_b);
	t := simd.neg(util.vector3_wide_dot(y, offset_b));
	t = simd.min(b.half_length, simd.max(simd.neg(b.half_length), t));
	capsule_local_closest := util.vector3_wide_scale(y, t);
	sphere_to_internal_segment := util.vector3_wide_add(offset_b, capsule_local_closest);
	internal_distance := util.vector3_wide_length(sphere_to_internal_segment);
	manifold.normal = util.vector3_wide_scale(sphere_to_internal_segment, simd.div(util.F32x8(-1), internal_distance));
	normal_is_valid := transmute(util.I32x8)simd.lanes_gt(internal_distance, util.F32x8(0));
	manifold.normal = util.vector3_wide_select(normal_is_valid, manifold.normal, x);
	manifold.depth = simd.sub(simd.add(a.radius, b.radius), internal_distance);
	negative_offset_from_sphere := simd.sub(simd.mul(manifold.depth, util.F32x8(0.5)), a.radius);
	manifold.offset_a = util.vector3_wide_scale(manifold.normal, negative_offset_from_sphere);
	manifold.contact_exists = transmute(util.I32x8)simd.lanes_gt(manifold.depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}

sphere_box_test_wide :: proc "contextless" (
	a: Sphere_Wide, b: Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_1_Contact_Manifold_Wide;
	orientation_matrix_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, orientation_matrix_b);
	clamped_local_offset_b := util.Vector3_Wide{
		simd.min(simd.max(local_offset_b.x, simd.neg(b.half_width)), b.half_width),
		simd.min(simd.max(local_offset_b.y, simd.neg(b.half_height)), b.half_height),
		simd.min(simd.max(local_offset_b.z, simd.neg(b.half_length)), b.half_length),
	};
	outside_normal := util.vector3_wide_subtract(clamped_local_offset_b, local_offset_b);
	distance := util.vector3_wide_length(outside_normal);
	outside_normal = util.vector3_wide_scale(outside_normal, simd.div(util.F32x8(1), distance));
	outside_depth := simd.sub(a.radius, distance);

	depth_x := simd.sub(b.half_width, simd.abs(local_offset_b.x));
	depth_y := simd.sub(b.half_height, simd.abs(local_offset_b.y));
	depth_z := simd.sub(b.half_length, simd.abs(local_offset_b.z));
	inside_depth := simd.min(depth_x, simd.min(depth_y, depth_z));
	use_x := transmute(util.I32x8)simd.lanes_eq(inside_depth, depth_x);
	use_y := transmute(util.I32x8)simd.lanes_eq(inside_depth, depth_y) & ~use_x;
	use_z := ~(use_x | use_y);
	inside_normal := util.Vector3_Wide{
		util.wide_select_f32(use_x, util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(local_offset_b.x, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		), util.F32x8(0)),
		util.wide_select_f32(use_y, util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(local_offset_b.y, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		), util.F32x8(0)),
		util.wide_select_f32(use_z, util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(local_offset_b.z, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		), util.F32x8(0)),
	};
	inside_depth = simd.add(inside_depth, a.radius);
	use_inside := transmute(util.I32x8)simd.lanes_eq(distance, util.F32x8(0));
	local_normal := util.vector3_wide_select(use_inside, inside_normal, outside_normal);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, orientation_matrix_b);
	manifold.depth = util.wide_select_f32(use_inside, inside_depth, outside_depth);
	negative_offset_from_sphere := simd.sub(simd.mul(manifold.depth, util.F32x8(0.5)), a.radius);
	manifold.offset_a = util.vector3_wide_scale(manifold.normal, negative_offset_from_sphere);
	manifold.contact_exists = transmute(util.I32x8)simd.lanes_gt(manifold.depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}

sphere_triangle_test_wide :: proc "contextless" (
	a: Sphere_Wide, b: Triangle_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_1_Contact_Manifold_Wide;
	orientation_matrix_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, orientation_matrix_b);
	ab := util.vector3_wide_subtract(b.b, b.a);
	ac := util.vector3_wide_subtract(b.c, b.a);
	pa := util.vector3_wide_add(b.a, local_offset_b);
	local_triangle_normal := util.vector3_wide_cross(ab, ac);
	triangle_normal_length := util.vector3_wide_length(local_triangle_normal);
	local_triangle_normal = util.vector3_wide_scale(
		local_triangle_normal,
		simd.div(util.F32x8(1), triangle_normal_length)
	);
	paxab := util.vector3_wide_cross(pa, ab);
	acxpa := util.vector3_wide_cross(ac, pa);
	edge_plane_test_ab := util.vector3_wide_dot(paxab, local_triangle_normal);
	edge_plane_test_ac := util.vector3_wide_dot(acxpa, local_triangle_normal);
	edge_plane_test_bc := simd.sub(
		util.F32x8(1), simd.mul(
		simd.add(edge_plane_test_ab, edge_plane_test_ac),
		simd.div(util.F32x8(1), triangle_normal_length)
	),
	);
	outside_ab := transmute(util.I32x8)simd.lanes_lt(edge_plane_test_ab, util.F32x8(0));
	outside_ac := transmute(util.I32x8)simd.lanes_lt(edge_plane_test_ac, util.F32x8(0));
	outside_bc := transmute(util.I32x8)simd.lanes_lt(edge_plane_test_bc, util.F32x8(0));
	outside_any_edge := outside_ab | outside_ac | outside_bc;
	edge_direction := util.vector3_wide_select(outside_ac, ac, ab);
	bc := util.vector3_wide_subtract(b.c, b.b);
	edge_direction = util.vector3_wide_select(outside_bc, bc, edge_direction);
	edge_start := util.vector3_wide_select(outside_bc, b.b, b.a);
	negative_edge_start_to_p := util.vector3_wide_add(local_offset_b, edge_start);
	negative_offset_dot_edge := util.vector3_wide_dot(negative_edge_start_to_p, edge_direction);
	edge_dot_edge := util.vector3_wide_dot(edge_direction, edge_direction);
	edge_scale := simd.max(
		util.F32x8(0), simd.min(util.F32x8(1), simd.div(simd.neg(negative_offset_dot_edge), edge_dot_edge)),
	);
	point_on_edge := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, edge_scale));
	pa_n := util.vector3_wide_dot(local_triangle_normal, pa);
	point_on_face := util.vector3_wide_subtract(util.vector3_wide_scale(local_triangle_normal, pa_n), local_offset_b);
	local_closest_on_triangle := util.vector3_wide_select(outside_any_edge, point_on_edge, point_on_face);
	manifold.feature_id = util.I32x8(0);
	manifold.feature_id = (outside_any_edge & manifold.feature_id) | (~outside_any_edge & util.I32x8(MESH_FACE_COLLISION_FLAG));
	manifold.offset_a = util.vector3_wide_add(
		util.matrix3x3_wide_transform(local_closest_on_triangle, orientation_matrix_b),
		offset_b
	);
	distance := util.vector3_wide_length(manifold.offset_a);
	manifold.normal = util.vector3_wide_scale(manifold.offset_a, simd.div(util.F32x8(-1), distance));
	manifold.depth = simd.sub(a.radius, distance);
	face_normal_dot_local_normal := util.vector3_wide_dot(local_triangle_normal, manifold.normal);
	ab_length_squared := util.vector3_wide_length_squared(ab);
	ac_length_squared := util.vector3_wide_length_squared(ac);
	epsilon_scale := simd.sqrt(simd.max(ab_length_squared, ac_length_squared));
	degenerate_epsilon := simd.mul(util.F32x8(1e-6), epsilon_scale);
	manifold.contact_exists =
		transmute(util.I32x8)simd.lanes_gt(distance, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(triangle_normal_length, degenerate_epsilon) &
		transmute(util.I32x8)simd.lanes_le(
		face_normal_dot_local_normal,
		util.F32x8(-TRIANGLE_BACKFACE_REJECTION_THRESHOLD)
	) &
		transmute(util.I32x8)simd.lanes_ge(manifold.depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}

sphere_cylinder_test_wide :: proc "contextless" (
	a: Sphere_Wide, b: Cylinder_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_1_Contact_Manifold_Wide;
	orientation_matrix_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	cylinder_local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, orientation_matrix_b);
	cylinder_local_offset_a := util.vector3_wide_negate(cylinder_local_offset_b);
	horizontal_offset_length := simd.sqrt(simd.add(
		simd.mul(cylinder_local_offset_a.x, cylinder_local_offset_a.x),
		simd.mul(cylinder_local_offset_a.z, cylinder_local_offset_a.z),
	));
	inverse_horizontal_offset_length := simd.div(util.F32x8(1), horizontal_offset_length);
	horizontal_clamp_multiplier := simd.mul(b.radius, inverse_horizontal_offset_length);
	horizontal_clamp_required := transmute(util.I32x8)simd.lanes_gt(horizontal_offset_length, b.radius);
	clamped_sphere_position_local_b := util.Vector3_Wide{
		util.wide_select_f32(
			horizontal_clamp_required,
			simd.mul(cylinder_local_offset_a.x, horizontal_clamp_multiplier),
			cylinder_local_offset_a.x
		),
		simd.min(b.half_length, simd.max(simd.neg(b.half_length), cylinder_local_offset_a.y)),
		util.wide_select_f32(
			horizontal_clamp_required,
			simd.mul(cylinder_local_offset_a.z, horizontal_clamp_multiplier),
			cylinder_local_offset_a.z
		),
	};
	sphere_to_closest_local_b := util.vector3_wide_add(clamped_sphere_position_local_b, cylinder_local_offset_b);
	manifold.offset_a = util.matrix3x3_wide_transform(sphere_to_closest_local_b, orientation_matrix_b);
	abs_y := simd.abs(cylinder_local_offset_a.y);
	depth_y := simd.sub(b.half_length, abs_y);
	horizontal_depth := simd.sub(b.radius, horizontal_offset_length);
	use_depth_y := transmute(util.I32x8)simd.lanes_le(depth_y, horizontal_depth);
	use_top_cap_normal := transmute(util.I32x8)simd.lanes_gt(cylinder_local_offset_a.y, util.F32x8(0));
	use_horizontal_fallback := transmute(util.I32x8)simd.lanes_le(
		horizontal_offset_length,
		simd.mul(b.radius, util.F32x8(1e-5))
	);
	local_internal_normal := util.Vector3_Wide{
		util.wide_select_f32(use_depth_y, util.F32x8(0), util.wide_select_f32(
			use_horizontal_fallback, util.F32x8(1), simd.mul(
			cylinder_local_offset_a.x,
			inverse_horizontal_offset_length
		),
		)),
		util.wide_select_f32(
			use_depth_y,
			util.wide_select_f32(use_top_cap_normal, util.F32x8(1), util.F32x8(-1)),
			util.F32x8(0)
		),
		util.wide_select_f32(use_depth_y, util.F32x8(0), util.wide_select_f32(
			use_horizontal_fallback, util.F32x8(0), simd.mul(
			cylinder_local_offset_a.z,
			inverse_horizontal_offset_length
		),
		)),
	};
	contact_distance := util.vector3_wide_length(sphere_to_closest_local_b);
	local_external_normal := util.vector3_wide_scale(
		sphere_to_closest_local_b,
		simd.div(util.F32x8(-1), contact_distance)
	);
	use_internal := transmute(util.I32x8)simd.lanes_lt(contact_distance, util.F32x8(1e-7));
	local_normal := util.vector3_wide_select(use_internal, local_internal_normal, local_external_normal);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, orientation_matrix_b);
	internal_depth := util.wide_select_f32(use_depth_y, depth_y, horizontal_depth);
	manifold.depth = simd.add(util.wide_select_f32(use_internal, internal_depth, simd.neg(contact_distance)), a.radius);
	manifold.contact_exists = transmute(util.I32x8)simd.lanes_ge(manifold.depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}

Sphere_Hull_Depth_Context_Wide :: struct
{
	hulls:          Convex_Hull_Wide,
	local_offset_a: util.Vector3_Wide,
}

sphere_hull_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Sphere_Hull_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(terminated_lanes, lane) < 0
		{
			continue;
		}
		hull := depth_context.hulls.hulls[lane];
		local_support, support_status := convex_hull_support(hull, util.vector3_wide_read_slot(direction, lane));
		if support_status != .Ok
		{
			return {}, {}, support_status;
		}
		util.vector3_wide_write_slot(&support_on_a, lane, local_support);
		util.vector3_wide_write_slot(
			&support, lane, util.vector3_subtract(
			local_support,
			util.vector3_wide_read_slot(depth_context.local_offset_a, lane)
		),
		);
	}
	return support, support_on_a, .Ok;
}

sphere_convex_hull_test_wide :: proc "contextless" (
	a: Sphere_Wide, b: Convex_Hull_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_1_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	inactive := ~active;
	hull_orientation := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, hull_orientation);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	center_distance := util.vector3_wide_length(local_offset_a);
	initial_normal := util.vector3_wide_scale(local_offset_a, simd.div(util.F32x8(1), center_distance));
	use_initial_fallback := transmute(util.I32x8)simd.lanes_lt(center_distance, util.F32x8(1e-8));
	initial_normal = util.vector3_wide_select(use_initial_fallback, {y=util.F32x8(1)}, initial_normal);
	epsilon_scale: util.F32x8;
	for lane in 0 ..< pair_count
	{
		hull := b.hulls[lane];
		if convex_hull_validate(hull) != .Ok
		{
			return {}, .Invalid_Description;
		}
		point := convex_hull_get_point(hull, 0);
		scale := (abs(point.x) + abs(point.y) + abs(point.z)) * (1.0 / 3.0);
		epsilon_scale = simd.replace(epsilon_scale, lane, min(simd.extract(a.radius, lane), scale));
	}
	depth_context := Sphere_Hull_Depth_Context_Wide{hulls=b, local_offset_a=local_offset_a};
	depth, local_normal, closest_on_hull, depth_status := depth_refiner_find_minimum_depth_wide(
		sphere_hull_depth_support_wide, &depth_context, initial_normal, inactive,
		simd.mul(util.F32x8(1e-5), epsilon_scale), simd.neg(speculative_margin),
		util.F32x8(0), a.radius,
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	manifold := Convex_1_Contact_Manifold_Wide{
		offset_a=util.vector3_wide_add(util.matrix3x3_wide_transform(closest_on_hull, hull_orientation), offset_b),
		normal=util.matrix3x3_wide_transform(local_normal, hull_orientation),
		depth=depth,
	};
	manifold.contact_exists = transmute(util.I32x8)simd.lanes_ge(depth, simd.neg(speculative_margin)) & active;
	return manifold, .Ok;
}
