// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

capsule_pair_test_wide :: proc "contextless" (
	a, b: Capsule_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_2_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	manifold: Convex_2_Contact_Manifold_Wide;
	xa, da := util.quaternion_wide_transform_unit_xy(orientation_a);
	db := util.quaternion_wide_transform_unit_y(orientation_b);
	da_offset_b := util.vector3_wide_dot(da, offset_b);
	db_offset_b := util.vector3_wide_dot(db, offset_b);
	dadb := util.vector3_wide_dot(da, db);
	ta := simd.div(
		simd.sub(da_offset_b, simd.mul(db_offset_b, dadb)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(dadb, dadb))),
	);
	tb := simd.sub(simd.mul(ta, dadb), db_offset_b);
	abs_dadb := simd.abs(dadb);
	b_onto_a_offset := simd.mul(b.half_length, abs_dadb);
	a_onto_b_offset := simd.mul(a.half_length, abs_dadb);
	a_min := simd.max(simd.neg(a.half_length), simd.min(a.half_length, simd.sub(da_offset_b, b_onto_a_offset)));
	a_max := simd.min(a.half_length, simd.max(simd.neg(a.half_length), simd.add(da_offset_b, b_onto_a_offset)));
	b_min := simd.max(
		simd.neg(b.half_length),
		simd.min(b.half_length, simd.sub(simd.neg(a_onto_b_offset), db_offset_b))
	);
	b_max := simd.min(b.half_length, simd.max(simd.neg(b.half_length), simd.sub(a_onto_b_offset, db_offset_b)));
	ta = simd.min(simd.max(ta, a_min), a_max);
	tb = simd.min(simd.max(tb, b_min), b_max);
	closest_point_on_a := util.vector3_wide_scale(da, ta);
	closest_point_on_b := util.vector3_wide_add(util.vector3_wide_scale(db, tb), offset_b);
	manifold.normal = util.vector3_wide_subtract(closest_point_on_a, closest_point_on_b);
	distance := util.vector3_wide_length(manifold.normal);
	manifold.normal = util.vector3_wide_scale(manifold.normal, simd.div(util.F32x8(1), distance));
	normal_is_valid := transmute(util.I32x8)simd.lanes_gt(distance, util.F32x8(1e-7));
	manifold.normal = util.vector3_wide_select(normal_is_valid, manifold.normal, xa);

	plane_normal := util.vector3_wide_cross(db, manifold.normal);
	plane_normal_length_squared := util.vector3_wide_length_squared(plane_normal);
	numerator := util.vector3_wide_dot(da, plane_normal);
	squared_angle := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(plane_normal_length_squared, util.F32x8(1e-10)),
		util.F32x8(0), simd.div(simd.mul(numerator, numerator), plane_normal_length_squared),
	);
	lower_threshold :: f32(0.01 * 0.01);
	upper_threshold :: f32(0.05 * 0.05);
	interval_weight := simd.max(
		util.F32x8(0), simd.min(
		util.F32x8(1), simd.mul(
		simd.sub(util.F32x8(upper_threshold), squared_angle),
		util.F32x8(1 / (upper_threshold - lower_threshold))
	),
	),
	);
	weighted_ta := simd.sub(ta, simd.mul(ta, interval_weight));
	a_min = simd.add(simd.mul(interval_weight, a_min), weighted_ta);
	a_max = simd.add(simd.mul(interval_weight, a_max), weighted_ta);
	manifold.offset_a_0 = util.vector3_wide_scale(da, a_min);
	manifold.offset_a_1 = util.vector3_wide_scale(da, a_max);
	db_normal := util.vector3_wide_dot(db, manifold.normal);
	offset_b_0 := util.vector3_wide_subtract(manifold.offset_a_0, offset_b);
	offset_b_1 := util.vector3_wide_subtract(manifold.offset_a_1, offset_b);
	inverse_dadb := simd.div(util.F32x8(1), dadb);
	projected_tb_0 := simd.max(b_min, simd.min(b_max, simd.mul(simd.sub(a_min, da_offset_b), inverse_dadb)));
	projected_tb_1 := simd.max(b_min, simd.min(b_max, simd.mul(simd.sub(a_max, da_offset_b), inverse_dadb)));
	b_0_normal := util.vector3_wide_dot(offset_b_0, manifold.normal);
	b_1_normal := util.vector3_wide_dot(offset_b_1, manifold.normal);
	capsules_are_perpendicular := transmute(util.I32x8)simd.lanes_lt(abs_dadb, util.F32x8(1e-7));
	distance_0 := util.wide_select_f32(
		capsules_are_perpendicular,
		distance,
		simd.sub(b_0_normal, simd.mul(db_normal, projected_tb_0))
	);
	distance_1 := util.wide_select_f32(
		capsules_are_perpendicular,
		distance,
		simd.sub(b_1_normal, simd.mul(db_normal, projected_tb_1))
	);
	combined_radius := simd.add(a.radius, b.radius);
	manifold.depth_0 = simd.sub(combined_radius, distance_0);
	manifold.depth_1 = simd.sub(combined_radius, distance_1);
	negative_offset_from_a_0 := simd.sub(simd.mul(manifold.depth_0, util.F32x8(0.5)), a.radius);
	negative_offset_from_a_1 := simd.sub(simd.mul(manifold.depth_1, util.F32x8(0.5)), a.radius);
	manifold.offset_a_0 = util.vector3_wide_add(
		manifold.offset_a_0,
		util.vector3_wide_scale(manifold.normal, negative_offset_from_a_0)
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		manifold.offset_a_1,
		util.vector3_wide_scale(manifold.normal, negative_offset_from_a_1)
	);
	manifold.feature_id_0 = util.I32x8(0);
	manifold.feature_id_1 = util.I32x8(1);
	minimum_accepted_depth := simd.neg(speculative_margin);
	manifold.contact_0_exists = transmute(util.I32x8)simd.lanes_ge(manifold.depth_0, minimum_accepted_depth) & active;
	manifold.contact_1_exists =
		transmute(util.I32x8)simd.lanes_ge(manifold.depth_1, minimum_accepted_depth) &
		transmute(util.I32x8)simd.lanes_gt(simd.sub(a_max, a_min), simd.mul(util.F32x8(1e-7), a.half_length)) & active;
	return manifold, .Ok;
}

capsule_box_edge_test_wide :: proc "contextless" (
	local_offset_a, capsule_axis: util.Vector3_Wide,
	capsule_half_length: util.F32x8,
	edge_center, edge_axis: util.Vector3_Wide,
	edge_half_length: util.F32x8,
	box_extents: util.Vector3_Wide,
) -> (ta, depth: util.F32x8, normal: util.Vector3_Wide)
{
	offset_a_to_b := util.vector3_wide_subtract(edge_center, local_offset_a);
	da_offset_b := util.vector3_wide_dot(capsule_axis, offset_a_to_b);
	db_offset_b := util.vector3_wide_dot(edge_axis, offset_a_to_b);
	dadb := util.vector3_wide_dot(capsule_axis, edge_axis);
	ta = simd.div(
		simd.sub(da_offset_b, simd.mul(db_offset_b, dadb)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(dadb, dadb))),
	);
	tb := simd.sub(simd.mul(ta, dadb), db_offset_b);
	abs_dadb := simd.abs(dadb);
	b_onto_a_offset := simd.mul(edge_half_length, abs_dadb);
	a_onto_b_offset := simd.mul(capsule_half_length, abs_dadb);
	ta_min := simd.max(
		simd.neg(capsule_half_length),
		simd.min(capsule_half_length, simd.sub(da_offset_b, b_onto_a_offset)),
	);
	ta_max := simd.min(
		capsule_half_length,
		simd.max(simd.neg(capsule_half_length), simd.add(da_offset_b, b_onto_a_offset)),
	);
	tb_min := simd.max(
		simd.neg(edge_half_length),
		simd.min(edge_half_length, simd.sub(simd.neg(db_offset_b), a_onto_b_offset)),
	);
	tb_max := simd.min(
		edge_half_length,
		simd.max(simd.neg(edge_half_length), simd.add(simd.neg(db_offset_b), a_onto_b_offset)),
	);
	ta = simd.min(simd.max(ta, ta_min), ta_max);
	tb = simd.min(simd.max(tb, tb_min), tb_max);
	closest_on_a := util.vector3_wide_add(local_offset_a, util.vector3_wide_scale(capsule_axis, ta));
	closest_on_b := util.vector3_wide_add(edge_center, util.vector3_wide_scale(edge_axis, tb));
	normal = util.vector3_wide_subtract(closest_on_a, closest_on_b);
	squared_length := util.vector3_wide_length_squared(normal);
	alternate_normal := util.vector3_wide_cross(edge_axis, capsule_axis);
	alternate_squared_length := util.vector3_wide_length_squared(alternate_normal);
	use_alternate := transmute(util.I32x8)simd.lanes_lt(squared_length, util.F32x8(1e-10));
	use_second_alternate := use_alternate & transmute(util.I32x8)simd.lanes_lt(
		alternate_squared_length,
		util.F32x8(1e-10)
	);
	axis_x := transmute(util.I32x8)simd.lanes_gt(simd.abs(edge_axis.x), util.F32x8(0.5));
	axis_y := transmute(util.I32x8)simd.lanes_gt(simd.abs(edge_axis.y), util.F32x8(0.5));
	second_alternate := util.Vector3_Wide{
		x=util.wide_select_f32(axis_y, util.F32x8(0), util.F32x8(1)),
		y=util.wide_select_f32(axis_x, util.F32x8(1), util.F32x8(0)),
		z=util.wide_select_f32(axis_x | axis_y, util.F32x8(0), util.F32x8(1)),
	};
	normal = util.vector3_wide_select(use_alternate, alternate_normal, normal);
	normal = util.vector3_wide_select(use_second_alternate, second_alternate, normal);
	squared_length = util.wide_select_f32(use_alternate, alternate_squared_length, squared_length);
	squared_length = util.wide_select_f32(use_second_alternate, util.F32x8(1), squared_length);
	should_negate := transmute(util.I32x8)simd.lanes_lt(util.vector3_wide_dot(normal, local_offset_a), util.F32x8(0));
	normal = util.vector3_wide_conditional_negate(should_negate, normal);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), simd.sqrt(squared_length)));
	box_extreme := simd.add(
		simd.add(simd.mul(simd.abs(normal.x), box_extents.x), simd.mul(simd.abs(normal.y), box_extents.y)),
		simd.mul(simd.abs(normal.z), box_extents.z),
	);
	depth = simd.sub(box_extreme, util.vector3_wide_dot(normal, closest_on_a));
	return;
}

capsule_box_select_edge_wide :: proc "contextless" (
	depth, ta: ^util.F32x8, normal: ^util.Vector3_Wide,
	candidate_depth, candidate_ta: util.F32x8, candidate_normal: util.Vector3_Wide,
)
{
	use_candidate := transmute(util.I32x8)simd.lanes_lt(candidate_depth, depth^);
	depth^ = util.wide_select_f32(use_candidate, candidate_depth, depth^);
	ta^ = util.wide_select_f32(use_candidate, candidate_ta, ta^);
	normal^ = util.vector3_wide_select(use_candidate, candidate_normal, normal^);
}

capsule_box_select_face_wide :: proc "contextless" (
	depth: ^util.F32x8, normal: ^util.Vector3_Wide,
	candidate_depth: util.F32x8, candidate_normal: util.Vector3_Wide,
)
{
	use_candidate := transmute(util.I32x8)simd.lanes_lt(candidate_depth, depth^);
	depth^ = util.wide_select_f32(use_candidate, candidate_depth, depth^);
	normal^ = util.vector3_wide_select(use_candidate, candidate_normal, normal^);
}

capsule_box_test_wide :: proc "contextless" (
	a: Capsule_Wide, b: Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_2_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	box_orientation := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_a := util.vector3_wide_negate(util.matrix3x3_wide_transform_transposed(offset_b, box_orientation));
	world_capsule_axis := util.quaternion_wide_transform_unit_y(orientation_a);
	capsule_axis := util.matrix3x3_wide_transform_transposed(world_capsule_axis, box_orientation);
	dot_axis := util.vector3_wide_dot(local_offset_a, capsule_axis);
	clamped_dot := simd.max(simd.neg(a.half_length), simd.min(a.half_length, dot_axis));
	offset_to_capsule_from_box := util.vector3_wide_subtract(
		local_offset_a, util.vector3_wide_scale(capsule_axis, clamped_dot),
	);
	edge_centers := util.Vector3_Wide{
		x=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(offset_to_capsule_from_box.x, util.F32x8(0)),
			simd.neg(b.half_width), b.half_width,
		),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(offset_to_capsule_from_box.y, util.F32x8(0)),
			simd.neg(b.half_height), b.half_height,
		),
		z=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(offset_to_capsule_from_box.z, util.F32x8(0)),
			simd.neg(b.half_length), b.half_length,
		),
	};
	extents := util.Vector3_Wide{b.half_width, b.half_height, b.half_length};
	ta, depth, local_normal := capsule_box_edge_test_wide(
		local_offset_a, capsule_axis, a.half_length,
		{y=edge_centers.y, z=edge_centers.z}, {x=util.F32x8(1)}, b.half_width, extents,
	);
	y_ta, y_depth, y_normal := capsule_box_edge_test_wide(
		local_offset_a, capsule_axis, a.half_length,
		{x=edge_centers.x, z=edge_centers.z}, {y=util.F32x8(1)}, b.half_height, extents,
	);
	capsule_box_select_edge_wide(&depth, &ta, &local_normal, y_depth, y_ta, y_normal);
	z_ta, z_depth, z_normal := capsule_box_edge_test_wide(
		local_offset_a, capsule_axis, a.half_length,
		{x=edge_centers.x, y=edge_centers.y}, {z=util.F32x8(1)}, b.half_length, extents,
	);
	capsule_box_select_edge_wide(&depth, &ta, &local_normal, z_depth, z_ta, z_normal);

	face_signs := util.Vector3_Wide{
		x=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(local_offset_a.x, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(local_offset_a.y, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		),
		z=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(local_offset_a.z, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		),
	};
	face_depth_x := simd.sub(
		simd.add(b.half_width, simd.mul(simd.abs(capsule_axis.x), a.half_length)),
		simd.mul(face_signs.x, local_offset_a.x),
	);
	face_depth_y := simd.sub(
		simd.add(b.half_height, simd.mul(simd.abs(capsule_axis.y), a.half_length)),
		simd.mul(face_signs.y, local_offset_a.y),
	);
	face_depth_z := simd.sub(
		simd.add(b.half_length, simd.mul(simd.abs(capsule_axis.z), a.half_length)),
		simd.mul(face_signs.z, local_offset_a.z),
	);
	capsule_box_select_face_wide(&depth, &local_normal, face_depth_x, {x=face_signs.x});
	capsule_box_select_face_wide(&depth, &local_normal, face_depth_y, {y=face_signs.y});
	capsule_box_select_face_wide(&depth, &local_normal, face_depth_z, {z=face_signs.z});

	x_dot := simd.mul(local_normal.x, face_signs.x);
	y_dot := simd.mul(local_normal.y, face_signs.y);
	z_dot := simd.mul(local_normal.z, face_signs.z);
	use_x := transmute(util.I32x8)simd.lanes_gt(x_dot, simd.max(y_dot, z_dot));
	use_y := (~use_x) & transmute(util.I32x8)simd.lanes_gt(y_dot, z_dot);
	face_normal_dot_local_normal := util.wide_select_f32(use_x, x_dot, util.wide_select_f32(use_y, y_dot, z_dot));
	face_plane_offset := util.wide_select_f32(
		use_x,
		b.half_width,
		util.wide_select_f32(use_y, b.half_height, b.half_length)
	);
	capsule_axis_dot_face_normal := util.wide_select_f32(
		use_x, simd.mul(capsule_axis.x, face_signs.x),
		util.wide_select_f32(use_y, simd.mul(capsule_axis.y, face_signs.y), simd.mul(capsule_axis.z, face_signs.z)),
	);
	capsule_center_dot_face_normal := util.wide_select_f32(
		use_x, simd.mul(local_offset_a.x, face_signs.x),
		util.wide_select_f32(use_y, simd.mul(local_offset_a.y, face_signs.y), simd.mul(local_offset_a.z, face_signs.z)),
	);
	inverse_face_dot := simd.div(util.F32x8(1), simd.max(util.F32x8(1e-15), face_normal_dot_local_normal));
	t_axis := simd.mul(capsule_axis_dot_face_normal, inverse_face_dot);
	t_center := simd.mul(simd.sub(capsule_center_dot_face_normal, face_plane_offset), inverse_face_dot);
	unprojected_axis := util.vector3_wide_subtract(capsule_axis, util.vector3_wide_scale(local_normal, t_axis));
	unprojected_center := util.vector3_wide_subtract(local_offset_a, util.vector3_wide_scale(local_normal, t_center));
	tangent_axis := util.Vector2_Wide{
		x=util.wide_select_f32(use_x, unprojected_axis.y, unprojected_axis.x),
		y=util.wide_select_f32(~use_y & ~use_x, unprojected_axis.y, unprojected_axis.z),
	};
	tangent_center := util.Vector2_Wide{
		x=util.wide_select_f32(use_x, unprojected_center.y, unprojected_center.x),
		y=util.wide_select_f32(~use_y & ~use_x, unprojected_center.y, unprojected_center.z),
	};
	epsilon_scale := simd.min(
		simd.max(b.half_width, simd.max(b.half_height, b.half_length)),
		simd.max(a.half_length, a.radius),
	);
	epsilon := simd.mul(epsilon_scale, util.F32x8(1e-3));
	half_extent_x := simd.add(epsilon, util.wide_select_f32(use_x, b.half_height, b.half_width));
	half_extent_y := simd.add(epsilon, util.wide_select_f32(~use_y & ~use_x, b.half_height, b.half_length));
	t_x_0 := simd.div(simd.neg(simd.sub(tangent_center.x, half_extent_x)), tangent_axis.x);
	t_x_1 := simd.div(simd.neg(simd.add(tangent_center.x, half_extent_x)), tangent_axis.x);
	t_y_0 := simd.div(simd.neg(simd.sub(tangent_center.y, half_extent_y)), tangent_axis.y);
	t_y_1 := simd.div(simd.neg(simd.add(tangent_center.y, half_extent_y)), tangent_axis.y);
	minimum_x := simd.min(t_x_0, t_x_1);
	maximum_x := simd.max(t_x_0, t_x_1);
	minimum_y := simd.min(t_y_0, t_y_1);
	maximum_y := simd.max(t_y_0, t_y_1);
	large_negative := util.F32x8(-3.402823466e+38);
	large_positive := util.F32x8(3.402823466e+38);
	parallel_x := transmute(util.I32x8)simd.lanes_lt(simd.abs(tangent_axis.x), util.F32x8(1e-15));
	parallel_y := transmute(util.I32x8)simd.lanes_lt(simd.abs(tangent_axis.y), util.F32x8(1e-15));
	contained_x := transmute(util.I32x8)simd.lanes_le(simd.abs(tangent_center.x), half_extent_x);
	contained_y := transmute(util.I32x8)simd.lanes_le(simd.abs(tangent_center.y), half_extent_y);
	minimum_x = util.wide_select_f32(
		parallel_x,
		util.wide_select_f32(contained_x, large_negative, large_positive),
		minimum_x
	);
	maximum_x = util.wide_select_f32(
		parallel_x,
		util.wide_select_f32(contained_x, large_positive, large_negative),
		maximum_x
	);
	minimum_y = util.wide_select_f32(
		parallel_y,
		util.wide_select_f32(contained_y, large_negative, large_positive),
		minimum_y
	);
	maximum_y = util.wide_select_f32(
		parallel_y,
		util.wide_select_f32(contained_y, large_positive, large_negative),
		maximum_y
	);
	face_min := simd.max(minimum_x, minimum_y);
	face_max := simd.min(maximum_x, maximum_y);
	t_min := simd.max(simd.min(face_min, a.half_length), simd.neg(a.half_length));
	t_max := simd.max(simd.min(face_max, a.half_length), simd.neg(a.half_length));
	face_interval_exists := transmute(util.I32x8)simd.lanes_ge(face_max, face_min);
	t_min = util.wide_select_f32(face_interval_exists, simd.min(t_min, ta), ta);
	t_max = util.wide_select_f32(face_interval_exists, simd.max(t_max, ta), ta);
	separation_min := simd.add(t_center, simd.mul(t_axis, t_min));
	separation_max := simd.add(t_center, simd.mul(t_axis, t_max));
	manifold := Convex_2_Contact_Manifold_Wide{
		depth_0=simd.sub(a.radius, separation_min),
		depth_1=simd.sub(a.radius, separation_max),
		feature_id_0=util.I32x8(0),
		feature_id_1=util.I32x8(1),
	};
	local_offset_0 := util.vector3_wide_scale(capsule_axis, t_min);
	local_offset_1 := util.vector3_wide_scale(capsule_axis, t_max);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, box_orientation);
	manifold.offset_a_0 = util.matrix3x3_wide_transform(local_offset_0, box_orientation);
	manifold.offset_a_1 = util.matrix3x3_wide_transform(local_offset_1, box_orientation);
	manifold.offset_a_0 = util.vector3_wide_add(
		manifold.offset_a_0,
		util.vector3_wide_scale(manifold.normal, simd.sub(simd.mul(manifold.depth_0, util.F32x8(0.5)), a.radius)),
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		manifold.offset_a_1,
		util.vector3_wide_scale(manifold.normal, simd.sub(simd.mul(manifold.depth_1, util.F32x8(0.5)), a.radius)),
	);
	minimum_accepted_depth := simd.neg(speculative_margin);
	manifold.contact_0_exists = transmute(util.I32x8)simd.lanes_ge(manifold.depth_0, minimum_accepted_depth) & active;
	manifold.contact_1_exists =
		transmute(util.I32x8)simd.lanes_ge(manifold.depth_1, minimum_accepted_depth) &
		transmute(util.I32x8)simd.lanes_gt(simd.sub(t_max, t_min), simd.mul(util.F32x8(1e-7), a.half_length)) & active;
	return manifold, .Ok;
}

capsule_triangle_test_edge_wide :: proc "contextless" (
	triangle: Triangle_Wide, triangle_normal, edge_start, edge_offset,
	capsule_center, capsule_axis: util.Vector3_Wide, capsule_half_length: util.F32x8,
) -> (
	edge_direction: util.Vector3_Wide,
	ta, tb, b_min, b_max, depth: util.F32x8,
	normal: util.Vector3_Wide,
)
{
	edge_length := util.vector3_wide_length(edge_offset);
	edge_direction = util.vector3_wide_scale(edge_offset, simd.div(util.F32x8(1), edge_length));
	offset_b := util.vector3_wide_subtract(edge_start, capsule_center);
	da_offset_b := util.vector3_wide_dot(capsule_axis, offset_b);
	db_offset_b := util.vector3_wide_dot(edge_direction, offset_b);
	dadb := util.vector3_wide_dot(capsule_axis, edge_direction);
	ta = simd.div(
		simd.sub(da_offset_b, simd.mul(db_offset_b, dadb)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(dadb, dadb))),
	);
	tb = simd.sub(simd.mul(ta, dadb), db_offset_b);
	ta_0 := simd.max(simd.neg(capsule_half_length), simd.min(capsule_half_length, da_offset_b));
	ta_1 := simd.min(
		capsule_half_length,
		simd.max(simd.neg(capsule_half_length), simd.add(da_offset_b, simd.mul(edge_length, dadb))),
	);
	a_min := simd.min(ta_0, ta_1);
	a_max := simd.max(ta_0, ta_1);
	a_onto_b_offset := simd.mul(capsule_half_length, simd.abs(dadb));
	b_min = simd.max(util.F32x8(0), simd.min(edge_length, simd.sub(simd.neg(a_onto_b_offset), db_offset_b)));
	b_max = simd.min(edge_length, simd.max(util.F32x8(0), simd.sub(a_onto_b_offset, db_offset_b)));
	ta = simd.min(simd.max(ta, a_min), a_max);
	tb = simd.min(simd.max(tb, b_min), b_max);
	closest_on_capsule := util.vector3_wide_add(capsule_center, util.vector3_wide_scale(capsule_axis, ta));
	closest_on_edge := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, tb));
	normal = util.vector3_wide_subtract(closest_on_capsule, closest_on_edge);
	normal_length_squared := util.vector3_wide_length_squared(normal);
	alternate_normal := util.vector3_wide_cross(capsule_axis, edge_offset);
	calibration_dot := util.vector3_wide_dot(alternate_normal, capsule_center);
	alternate_normal = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_lt(calibration_dot, util.F32x8(0)), alternate_normal,
	);
	alternate_normal_length_squared := util.vector3_wide_length_squared(alternate_normal);
	use_alternate := transmute(util.I32x8)simd.lanes_lt(normal_length_squared, util.F32x8(1e-13));
	normal = util.vector3_wide_select(use_alternate, alternate_normal, normal);
	normal_length_squared = util.wide_select_f32(use_alternate, alternate_normal_length_squared, normal_length_squared);
	second_alternate_normal := util.vector3_wide_cross(triangle_normal, edge_offset);
	second_alternate_length_squared := util.vector3_wide_length_squared(second_alternate_normal);
	use_second_alternate := transmute(util.I32x8)simd.lanes_lt(normal_length_squared, util.F32x8(1e-13));
	normal = util.vector3_wide_select(use_second_alternate, second_alternate_normal, normal);
	normal_length_squared = util.wide_select_f32(
		use_second_alternate,
		second_alternate_length_squared,
		normal_length_squared
	);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), simd.sqrt(normal_length_squared)));
	n_axis := util.vector3_wide_dot(capsule_axis, normal);
	n_capsule_center := util.vector3_wide_dot(normal, capsule_center);
	extreme_on_capsule := simd.sub(n_capsule_center, simd.mul(simd.abs(n_axis), capsule_half_length));
	extreme_on_triangle := simd.max(
		util.vector3_wide_dot(triangle.a, normal),
		simd.max(util.vector3_wide_dot(triangle.b, normal), util.vector3_wide_dot(triangle.c, normal)),
	);
	depth = simd.sub(extreme_on_triangle, extreme_on_capsule);
	return;
}

capsule_triangle_clip_edge_plane_wide :: proc "contextless" (
	edge_start, edge_offset, face_normal, capsule_center, capsule_axis: util.Vector3_Wide,
) -> (entry, exit: util.F32x8)
{
	edge_plane_normal := util.vector3_wide_cross(face_normal, edge_offset);
	edge_to_capsule := util.vector3_wide_subtract(capsule_center, edge_start);
	distance := util.vector3_wide_dot(edge_to_capsule, edge_plane_normal);
	velocity := util.vector3_wide_dot(capsule_axis, edge_plane_normal);
	velocity_positive := transmute(util.I32x8)simd.lanes_gt(velocity, util.F32x8(0));
	numerator := util.wide_select_f32(velocity_positive, simd.neg(distance), distance);
	t := simd.div(numerator, simd.max(util.F32x8(1e-15), simd.abs(velocity)));
	entry = util.wide_select_f32(velocity_positive, util.F32x8(-3.402823466e+38), t);
	exit = util.wide_select_f32(velocity_positive, t, util.F32x8(3.402823466e+38));
	return;
}

capsule_triangle_test_wide :: proc "contextless" (
	a: Capsule_Wide, b: Triangle_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_2_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_triangle_center := util.vector3_wide_scale(
		util.vector3_wide_add(b.a, util.vector3_wide_add(b.b, b.c)), util.F32x8(1.0 / 3.0),
	);
	local_offset_b := util.vector3_wide_add(
		util.matrix3x3_wide_transform_transposed(offset_b, r_b), local_triangle_center,
	);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	triangle := Triangle_Wide{
		a=util.vector3_wide_subtract(b.a, local_triangle_center),
		b=util.vector3_wide_subtract(b.b, local_triangle_center),
		c=util.vector3_wide_subtract(b.c, local_triangle_center),
	};
	world_capsule_axis := util.quaternion_wide_transform_unit_y(orientation_a);
	local_capsule_axis := util.matrix3x3_wide_transform_transposed(world_capsule_axis, r_b);
	ac := util.vector3_wide_subtract(b.c, b.a);
	ab := util.vector3_wide_subtract(b.b, b.a);
	ac_cross_ab := util.vector3_wide_cross(ac, ab);
	face_normal_length := util.vector3_wide_length(ac_cross_ab);
	face_normal := util.vector3_wide_scale(ac_cross_ab, simd.div(util.F32x8(1), face_normal_length));
	n_dot_axis := util.vector3_wide_dot(face_normal, local_capsule_axis);
	capsule_offset_along_normal := util.vector3_wide_dot(face_normal, local_offset_a);
	face_depth := simd.sub(simd.mul(a.half_length, simd.abs(n_dot_axis)), capsule_offset_along_normal);

	edge_direction, ta, tb, b_min, b_max, edge_depth, edge_normal := capsule_triangle_test_edge_wide(
		triangle, face_normal, triangle.a, ab, local_offset_a, local_capsule_axis, a.half_length,
	);
	candidate_direction, candidate_ta, candidate_tb, candidate_min, candidate_max, candidate_depth, candidate_normal :=
		capsule_triangle_test_edge_wide(
		triangle, face_normal, triangle.a, ac, local_offset_a, local_capsule_axis, a.half_length,
	);
	use_ac := transmute(util.I32x8)simd.lanes_lt(candidate_depth, edge_depth);
	edge_direction = util.vector3_wide_select(use_ac, candidate_direction, edge_direction);
	edge_normal = util.vector3_wide_select(use_ac, candidate_normal, edge_normal);
	ta = util.wide_select_f32(use_ac, candidate_ta, ta);
	tb = util.wide_select_f32(use_ac, candidate_tb, tb);
	b_min = util.wide_select_f32(use_ac, candidate_min, b_min);
	b_max = util.wide_select_f32(use_ac, candidate_max, b_max);
	edge_depth = simd.min(candidate_depth, edge_depth);
	bc := util.vector3_wide_subtract(b.c, b.b);
	candidate_direction, candidate_ta, candidate_tb, candidate_min, candidate_max, candidate_depth, candidate_normal =
		capsule_triangle_test_edge_wide(
		triangle, face_normal, triangle.b, bc, local_offset_a, local_capsule_axis, a.half_length,
	);
	use_bc := transmute(util.I32x8)simd.lanes_lt(candidate_depth, edge_depth);
	edge_start := util.vector3_wide_select(use_bc, triangle.b, triangle.a);
	edge_direction = util.vector3_wide_select(use_bc, candidate_direction, edge_direction);
	edge_normal = util.vector3_wide_select(use_bc, candidate_normal, edge_normal);
	ta = util.wide_select_f32(use_bc, candidate_ta, ta);
	tb = util.wide_select_f32(use_bc, candidate_tb, tb);
	b_min = util.wide_select_f32(use_bc, candidate_min, b_min);
	b_max = util.wide_select_f32(use_bc, candidate_max, b_max);
	edge_depth = simd.min(candidate_depth, edge_depth);
	depth := simd.min(edge_depth, face_depth);
	use_edge := transmute(util.I32x8)simd.lanes_lt(edge_depth, face_depth);
	local_normal := util.vector3_wide_select(use_edge, edge_normal, face_normal);
	local_normal_dot_face_normal := util.vector3_wide_dot(local_normal, face_normal);
	negative_margin := simd.neg(speculative_margin);
	allow_contacts := active &
		transmute(util.I32x8)simd.lanes_ge(simd.add(depth, a.radius), negative_margin) &
		transmute(util.I32x8)simd.lanes_ge(
		local_normal_dot_face_normal,
		util.F32x8(TRIANGLE_BACKFACE_REJECTION_THRESHOLD)
	) &
		transmute(util.I32x8)simd.lanes_ge(face_normal_length, util.F32x8(1e-7));
	manifold: Convex_2_Contact_Manifold_Wide;
	if depth_refiner_all(~allow_contacts) == .Present
	{
		return manifold, .Ok;
	}

	use_edge &= allow_contacts;
	plane_normal := util.vector3_wide_cross(edge_direction, edge_normal);
	plane_normal_length_squared := util.vector3_wide_length_squared(plane_normal);
	numerator_unsquared := util.vector3_wide_dot(local_capsule_axis, plane_normal);
	squared_angle := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(plane_normal_length_squared, util.F32x8(1e-10)),
		util.F32x8(0), simd.div(simd.mul(numerator_unsquared, numerator_unsquared), plane_normal_length_squared),
	);
	lower_threshold :: f32(0.01 * 0.01);
	upper_threshold :: f32(0.05 * 0.05);
	interval_weight := simd.max(
		util.F32x8(0),
		simd.min(
		util.F32x8(1),
		simd.mul(
		simd.sub(util.F32x8(upper_threshold), squared_angle),
		util.F32x8(1 / (upper_threshold - lower_threshold))
	),
	),
	);
	weighted_tb := simd.sub(tb, simd.mul(tb, interval_weight));
	b_min = simd.add(simd.mul(interval_weight, b_min), weighted_tb);
	b_max = simd.add(simd.mul(interval_weight, b_max), weighted_tb);
	b_0 := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, b_min));
	b_1 := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, b_max));
	edge_has_two := transmute(util.I32x8)simd.lanes_gt(b_max, b_min);
	contact_count := collision_wide_select_i32(
		use_edge, collision_wide_select_i32(edge_has_two, util.I32x8(2), util.I32x8(1)), util.I32x8(0),
	);
	needs_face_candidate := allow_contacts & transmute(util.I32x8)simd.lanes_le(contact_count, util.I32x8(1));
	if depth_refiner_all(~needs_face_candidate) != .Present
	{
		ab_entry, ab_exit := capsule_triangle_clip_edge_plane_wide(
			triangle.a, ab, face_normal, local_offset_a, local_capsule_axis,
		);
		bc_entry, bc_exit := capsule_triangle_clip_edge_plane_wide(
			triangle.b, bc, face_normal, local_offset_a, local_capsule_axis,
		);
		ca := util.vector3_wide_negate(ac);
		ca_entry, ca_exit := capsule_triangle_clip_edge_plane_wide(
			triangle.a, ca, face_normal, local_offset_a, local_capsule_axis,
		);
		triangle_interval_min := simd.max(ab_entry, simd.max(bc_entry, ca_entry));
		triangle_interval_max := simd.min(ab_exit, simd.min(bc_exit, ca_exit));
		negative_half_length := simd.neg(a.half_length);
		overlap_interval_min := simd.max(triangle_interval_min, negative_half_length);
		overlap_interval_max := simd.min(triangle_interval_max, a.half_length);
		interval_valid_for_second := transmute(util.I32x8)simd.lanes_ge(overlap_interval_max, overlap_interval_min);
		overlap_interval_min = simd.min(overlap_interval_min, a.half_length);
		overlap_interval_max = simd.max(overlap_interval_max, negative_half_length);
		clipped_a_0 := util.vector3_wide_add(
			local_offset_a, util.vector3_wide_scale(local_capsule_axis, overlap_interval_min),
		);
		distance_along_normal_a_0 := util.vector3_wide_dot(clipped_a_0, face_normal);
		face_candidate_0 := util.vector3_wide_subtract(
			clipped_a_0, util.vector3_wide_scale(face_normal, distance_along_normal_a_0),
		);
		clipped_a_1 := util.vector3_wide_add(
			local_offset_a, util.vector3_wide_scale(local_capsule_axis, overlap_interval_max),
		);
		distance_along_normal_a_1 := util.vector3_wide_dot(clipped_a_1, face_normal);
		face_candidate_1 := util.vector3_wide_subtract(
			clipped_a_1, util.vector3_wide_scale(face_normal, distance_along_normal_a_1),
		);
		no_edge_contacts := transmute(util.I32x8)simd.lanes_eq(contact_count, util.I32x8(0));
		allow_face_contacts := transmute(util.I32x8)simd.lanes_ge(capsule_offset_along_normal, util.F32x8(0));
		use_face_contacts := no_edge_contacts & allow_face_contacts & allow_contacts;
		b_0 = util.vector3_wide_select(use_face_contacts, face_candidate_0, b_0);
		b_1 = util.vector3_wide_select(use_face_contacts, face_candidate_1, b_1);
		contact_count = collision_wide_select_i32(use_face_contacts, util.I32x8(2), contact_count);
		use_face_contact_1 := transmute(util.I32x8)simd.lanes_gt(
			simd.abs(simd.sub(overlap_interval_max, ta)), simd.abs(simd.sub(overlap_interval_min, ta)),
		);
		second_contact_candidate := util.vector3_wide_select(use_face_contact_1, face_candidate_1, face_candidate_0);
		second_contact_distance := util.wide_select_f32(
			use_face_contact_1, distance_along_normal_a_1, distance_along_normal_a_0,
		);
		use_second_candidate := interval_valid_for_second &
			transmute(util.I32x8)simd.lanes_eq(contact_count, util.I32x8(1)) &
			transmute(util.I32x8)simd.lanes_gt(second_contact_distance, util.F32x8(0)) & allow_contacts;
		b_1 = util.vector3_wide_select(use_second_candidate, second_contact_candidate, b_1);
		contact_count = collision_wide_select_i32(use_second_candidate, util.I32x8(2), contact_count);
	}

	capsule_tangent := util.vector3_wide_cross(local_normal, local_capsule_axis);
	face_normal_a := util.vector3_wide_cross(capsule_tangent, local_capsule_axis);
	face_normal_a_dot_local_normal := util.vector3_wide_dot(face_normal_a, local_normal);
	inverse_face_dot := simd.div(util.F32x8(1), face_normal_a_dot_local_normal);
	t_0 := simd.mul(util.vector3_wide_dot(util.vector3_wide_add(local_offset_b, b_0), face_normal_a), inverse_face_dot);
	t_1 := simd.mul(util.vector3_wide_dot(util.vector3_wide_add(local_offset_b, b_1), face_normal_a), inverse_face_dot);
	manifold.depth_0 = simd.add(a.radius, t_0);
	manifold.depth_1 = simd.add(a.radius, t_1);
	collapse := transmute(util.I32x8)simd.lanes_lt(simd.abs(face_normal_a_dot_local_normal), util.F32x8(1e-7));
	manifold.depth_0 = util.wide_select_f32(collapse, simd.add(a.radius, depth), manifold.depth_0);
	manifold.contact_0_exists = allow_contacts &
		transmute(util.I32x8)simd.lanes_gt(contact_count, util.I32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(manifold.depth_0, negative_margin);
	manifold.contact_1_exists = allow_contacts & ~collapse &
		transmute(util.I32x8)simd.lanes_eq(contact_count, util.I32x8(2)) &
		transmute(util.I32x8)simd.lanes_gt(manifold.depth_1, negative_margin);
	local_offset_a_0 := util.vector3_wide_subtract(b_0, local_offset_a);
	local_offset_a_1 := util.vector3_wide_subtract(b_1, local_offset_a);
	ta_0 := util.vector3_wide_dot(local_offset_a_0, local_capsule_axis);
	ta_1 := util.vector3_wide_dot(local_offset_a_1, local_capsule_axis);
	flip_feature_ids := transmute(util.I32x8)simd.lanes_lt(ta_1, ta_0);
	manifold.feature_id_0 = collision_wide_select_i32(flip_feature_ids, util.I32x8(1), util.I32x8(0));
	manifold.feature_id_1 = collision_wide_select_i32(flip_feature_ids, util.I32x8(0), util.I32x8(1));
	face_flag := collision_wide_select_i32(
		transmute(util.I32x8)simd.lanes_ge(
		local_normal_dot_face_normal, util.F32x8(MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION),
	),
		util.I32x8(MESH_REDUCTION_FACE_COLLISION_FLAG), util.I32x8(0),
	);
	manifold.feature_id_0 += face_flag;
	manifold.offset_a_0 = util.matrix3x3_wide_transform(local_offset_a_0, r_b);
	manifold.offset_a_1 = util.matrix3x3_wide_transform(local_offset_a_1, r_b);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, r_b);
	return manifold, .Ok;
}

capsule_cylinder_bounce_wide :: proc "contextless" (
	line_origin, line_direction: util.Vector3_Wide, t: util.F32x8,
	b: Cylinder_Wide, radius_squared: util.F32x8,
) -> (point, clamped: util.Vector3_Wide)
{
	point = util.vector3_wide_add(line_origin, util.vector3_wide_scale(line_direction, t));
	horizontal_distance_squared := simd.add(simd.mul(point.x, point.x), simd.mul(point.z, point.z));
	need_horizontal_clamp := transmute(util.I32x8)simd.lanes_gt(horizontal_distance_squared, radius_squared);
	clamp_scale := simd.div(b.radius, simd.sqrt(horizontal_distance_squared));
	clamped = {
		x=util.wide_select_f32(need_horizontal_clamp, simd.mul(clamp_scale, point.x), point.x),
		y=simd.max(simd.neg(b.half_length), simd.min(b.half_length, point.y)),
		z=util.wide_select_f32(need_horizontal_clamp, simd.mul(clamp_scale, point.z), point.z),
	};
	return;
}

capsule_cylinder_closest_line_point_wide :: proc "contextless" (
	line_origin, line_direction: util.Vector3_Wide, half_length: util.F32x8,
	b: Cylinder_Wide, inactive_lanes: util.I32x8,
) -> (t: util.F32x8, offset_from_cylinder: util.Vector3_Wide)
{
	minimum := simd.neg(half_length);
	maximum := half_length;
	radius_squared := simd.mul(b.radius, b.radius);
	origin_dot := util.vector3_wide_dot(line_direction, line_origin);
	epsilon := simd.mul(half_length, util.F32x8(1e-7));
	lane_deactivated := inactive_lanes;
	for _ in 0 ..< 12
	{
		_, clamped := capsule_cylinder_bounce_wide(line_origin, line_direction, t, b, radius_squared);
		conservative_new_t := simd.max(
			minimum,
			simd.min(maximum, simd.sub(util.vector3_wide_dot(clamped, line_direction), origin_dot)),
		);
		change := simd.sub(conservative_new_t, t);
		lane_deactivated |= transmute(util.I32x8)simd.lanes_lt(simd.abs(change), epsilon);
		if depth_refiner_all(lane_deactivated) == .Present
		{
			break;
		}
		moved_up := transmute(util.I32x8)simd.lanes_gt(change, util.F32x8(0));
		minimum = util.wide_select_f32(moved_up, conservative_new_t, minimum);
		maximum = util.wide_select_f32(moved_up, maximum, conservative_new_t);
		new_t := simd.mul(util.F32x8(0.5), simd.add(minimum, maximum));
		t = util.wide_select_f32(lane_deactivated, t, new_t);
	}
	point_on_line, clamped_to_cylinder := capsule_cylinder_bounce_wide(
		line_origin, line_direction, t, b, radius_squared,
	);
	offset_from_cylinder = util.vector3_wide_subtract(point_on_line, clamped_to_cylinder);
	return;
}

capsule_cylinder_closest_segments_wide :: proc "contextless" (
	axis_a, local_offset_b: util.Vector3_Wide, a_half_length, b_half_length: util.F32x8,
) -> (ta, ta_min, ta_max, tb, tb_min, tb_max: util.F32x8)
{
	da_offset_b := util.vector3_wide_dot(axis_a, local_offset_b);
	db_offset_b := local_offset_b.y;
	dadb := axis_a.y;
	ta = simd.div(
		simd.sub(da_offset_b, simd.mul(db_offset_b, dadb)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(dadb, dadb))),
	);
	tb = simd.sub(simd.mul(ta, dadb), db_offset_b);
	abs_dadb := simd.abs(dadb);
	b_onto_a_offset := simd.mul(b_half_length, abs_dadb);
	a_onto_b_offset := simd.mul(a_half_length, abs_dadb);
	ta_min = simd.max(simd.neg(a_half_length), simd.min(a_half_length, simd.sub(da_offset_b, b_onto_a_offset)));
	ta_max = simd.min(a_half_length, simd.max(simd.neg(a_half_length), simd.add(da_offset_b, b_onto_a_offset)));
	tb_min = simd.max(
		simd.neg(b_half_length),
		simd.min(b_half_length, simd.sub(simd.neg(a_onto_b_offset), db_offset_b))
	);
	tb_max = simd.min(b_half_length, simd.max(simd.neg(b_half_length), simd.sub(a_onto_b_offset, db_offset_b)));
	ta = simd.min(simd.max(ta, ta_min), ta_max);
	tb = simd.min(simd.max(tb, tb_min), tb_max);
	return;
}

capsule_cylinder_contact_interval_wide :: proc "contextless" (
	a_half_length, b_half_length: util.F32x8, axis_a, local_normal: util.Vector3_Wide,
	inverse_horizontal_normal_length_squared: util.F32x8, offset_b: util.Vector3_Wide,
) -> (contact_t_min, contact_t_max: util.F32x8)
{
	_, _, _, tb, tb_min, tb_max := capsule_cylinder_closest_segments_wide(
		axis_a, offset_b, a_half_length, b_half_length,
	);
	dot := simd.sub(simd.mul(axis_a.x, local_normal.z), simd.mul(axis_a.z, local_normal.x));
	squared_angle := simd.mul(simd.mul(dot, dot), inverse_horizontal_normal_length_squared);
	lower_threshold :: f32(0.02 * 0.02);
	upper_threshold :: f32(0.15 * 0.15);
	interval_weight := simd.max(
		util.F32x8(0),
		simd.min(
		util.F32x8(1),
		simd.mul(
		simd.sub(util.F32x8(upper_threshold), squared_angle),
		util.F32x8(1 / (upper_threshold - lower_threshold))
	),
	),
	);
	weighted_tb := simd.sub(tb, simd.mul(tb, interval_weight));
	contact_t_min = simd.add(simd.mul(interval_weight, tb_min), weighted_tb);
	contact_t_max = simd.add(simd.mul(interval_weight, tb_max), weighted_tb);
	return;
}

capsule_cylinder_test_wide :: proc "contextless" (
	a: Capsule_Wide, b: Cylinder_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_2_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	inactive := ~active;
	world_r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	world_r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	r_a := util.matrix3x3_wide_multiply_by_transpose(world_r_a, world_r_b);
	capsule_axis := r_a.y;
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_b);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	_, local_normal := capsule_cylinder_closest_line_point_wide(
		local_offset_a, capsule_axis, a.half_length, b, inactive,
	);
	distance_squared := util.vector3_wide_length_squared(local_normal);
	internal_segment_intersected := transmute(util.I32x8)simd.lanes_lt(distance_squared, util.F32x8(1e-12));
	distance := simd.sqrt(distance_squared);
	local_normal = util.vector3_wide_scale(local_normal, simd.div(util.F32x8(1), distance));
	depth := util.wide_select_f32(internal_segment_intersected, util.F32x8(3.402823466e+38), simd.neg(distance));
	negative_margin := simd.neg(speculative_margin);
	inactive |= (~internal_segment_intersected) & transmute(util.I32x8)simd.lanes_lt(
		simd.add(depth, a.radius),
		negative_margin
	);

	endpoint_cap_depth := simd.sub(
		simd.add(b.half_length, simd.abs(simd.mul(capsule_axis.y, a.half_length))), simd.abs(local_offset_a.y),
	);
	use_endpoint_cap := internal_segment_intersected & transmute(util.I32x8)simd.lanes_lt(endpoint_cap_depth, depth);
	depth = util.wide_select_f32(use_endpoint_cap, endpoint_cap_depth, depth);
	cap_normal := util.Vector3_Wide{
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(local_offset_a.y, util.F32x8(0)), util.F32x8(1), util.F32x8(-1),
		),
	};
	local_normal = util.vector3_wide_select(use_endpoint_cap, cap_normal, local_normal);
	ta, _, _, tb, _, _ := capsule_cylinder_closest_segments_wide(
		capsule_axis, local_offset_b, a.half_length, b.half_length,
	);
	closest_a := util.vector3_wide_scale(capsule_axis, ta);
	internal_offset := util.vector3_wide_subtract(closest_a, local_offset_b);
	internal_offset.y = simd.sub(internal_offset.y, tb);
	internal_distance := util.vector3_wide_length(internal_offset);
	internal_edge_normal := util.vector3_wide_scale(internal_offset, simd.div(util.F32x8(1), internal_distance));
	internal_edge_normal = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(internal_distance, util.F32x8(1e-7)),
		{x=util.F32x8(1)}, internal_edge_normal,
	);
	center_separation := util.vector3_wide_dot(local_offset_a, internal_edge_normal);
	cylinder_contribution := simd.add(
		simd.abs(simd.mul(b.half_length, internal_edge_normal.y)),
		simd.mul(
		b.radius,
		simd.sqrt(simd.max(
		util.F32x8(0),
		simd.sub(util.F32x8(1), simd.mul(internal_edge_normal.y, internal_edge_normal.y))
	)),
	),
	);
	capsule_contribution := simd.mul(
		simd.abs(util.vector3_wide_dot(capsule_axis, internal_edge_normal)), a.half_length,
	);
	internal_edge_depth := simd.sub(simd.add(cylinder_contribution, capsule_contribution), center_separation);
	use_internal_edge := internal_segment_intersected & transmute(util.I32x8)simd.lanes_lt(internal_edge_depth, depth);
	depth = util.wide_select_f32(use_internal_edge, internal_edge_depth, depth);
	local_normal = util.vector3_wide_select(use_internal_edge, internal_edge_normal, local_normal);
	depth = simd.add(depth, a.radius);
	inactive |= transmute(util.I32x8)simd.lanes_lt(depth, negative_margin);
	manifold: Convex_2_Contact_Manifold_Wide;
	if depth_refiner_all(inactive) == .Present
	{
		return manifold, .Ok;
	}

	use_cap_contacts := transmute(util.I32x8)simd.lanes_gt(
		simd.abs(local_normal.y),
		util.F32x8(0.70710678118)
	) & ~inactive;
	inverse_horizontal_length_squared := simd.div(
		util.F32x8(1), simd.add(simd.mul(local_normal.x, local_normal.x), simd.mul(local_normal.z, local_normal.z)),
	);
	scale := simd.mul(b.radius, simd.sqrt(inverse_horizontal_length_squared));
	cylinder_segment_offset_x := simd.mul(local_normal.x, scale);
	cylinder_segment_offset_z := simd.mul(local_normal.z, scale);
	a_to_side_segment_center := util.Vector3_Wide{
		x=simd.add(local_offset_b.x, cylinder_segment_offset_x),
		y=local_offset_b.y,
		z=simd.add(local_offset_b.z, cylinder_segment_offset_z),
	};
	contact_t_min, contact_t_max := capsule_cylinder_contact_interval_wide(
		a.half_length, b.half_length, capsule_axis, local_normal,
		inverse_horizontal_length_squared, a_to_side_segment_center,
	);
	contact_0 := util.Vector3_Wide{x=cylinder_segment_offset_x, y=contact_t_min, z=cylinder_segment_offset_z};
	contact_1 := util.Vector3_Wide{x=cylinder_segment_offset_x, y=contact_t_max, z=cylinder_segment_offset_z};
	contact_count := collision_wide_select_i32(
		transmute(util.I32x8)simd.lanes_lt(
		simd.abs(simd.sub(contact_t_max, contact_t_min)), simd.mul(b.half_length, util.F32x8(1e-5)),
	),
		util.I32x8(1), util.I32x8(2),
	);
	cap_height := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(local_normal.y, util.F32x8(0)), b.half_length, simd.neg(b.half_length),
	);
	inverse_normal_y := simd.div(util.F32x8(1), local_normal.y);
	endpoint_offset := util.vector3_wide_scale(capsule_axis, a.half_length);
	positive := util.vector3_wide_add(local_offset_a, endpoint_offset);
	positive.y = simd.sub(positive.y, cap_height);
	negative := util.vector3_wide_subtract(local_offset_a, endpoint_offset);
	negative.y = simd.sub(negative.y, cap_height);
	t_negative := simd.mul(negative.y, inverse_normal_y);
	t_positive := simd.mul(positive.y, inverse_normal_y);
	projected_negative := util.Vector2_Wide{
		x=simd.sub(negative.x, simd.mul(local_normal.x, t_negative)),
		y=simd.sub(negative.z, simd.mul(local_normal.z, t_negative)),
	};
	projected_positive := util.Vector2_Wide{
		x=simd.sub(positive.x, simd.mul(local_normal.x, t_positive)),
		y=simd.sub(positive.z, simd.mul(local_normal.z, t_positive)),
	};
	projected_offset := util.vector2_wide_subtract(projected_positive, projected_negative);
	coefficient_c := simd.sub(
		util.vector2_wide_dot(projected_negative, projected_negative),
		simd.mul(b.radius, b.radius)
	);
	coefficient_b := util.vector2_wide_dot(projected_negative, projected_offset);
	coefficient_a := util.vector2_wide_dot(projected_offset, projected_offset);
	inverse_a := simd.div(util.F32x8(1), coefficient_a);
	t_offset := simd.mul(
		simd.sqrt(simd.max(
		util.F32x8(0),
		simd.sub(simd.mul(coefficient_b, coefficient_b), simd.mul(coefficient_a, coefficient_c))
	)),
		inverse_a,
	);
	t_base := simd.mul(simd.neg(coefficient_b), inverse_a);
	t_min := simd.max(util.F32x8(0), simd.min(util.F32x8(1), simd.sub(t_base, t_offset)));
	t_max := simd.max(util.F32x8(0), simd.min(util.F32x8(1), simd.add(t_base, t_offset)));
	use_cap_alternate := transmute(util.I32x8)simd.lanes_lt(simd.abs(coefficient_a), util.F32x8(1e-12));
	t_min = util.wide_select_f32(use_cap_alternate, util.F32x8(0), t_min);
	t_max = util.wide_select_f32(use_cap_alternate, util.F32x8(0), t_max);
	cap_contact_0 := util.Vector3_Wide{
		x=simd.add(simd.mul(t_min, projected_offset.x), projected_negative.x),
		y=cap_height,
		z=simd.add(simd.mul(t_min, projected_offset.y), projected_negative.y),
	};
	cap_contact_1 := util.Vector3_Wide{
		x=simd.add(simd.mul(t_max, projected_offset.x), projected_negative.x),
		y=cap_height,
		z=simd.add(simd.mul(t_max, projected_offset.y), projected_negative.y),
	};
	cap_contact_count := collision_wide_select_i32(
		transmute(util.I32x8)simd.lanes_gt(simd.sub(t_max, t_min), util.F32x8(1e-5)),
		util.I32x8(2), util.I32x8(1),
	);
	contact_count = collision_wide_select_i32(use_cap_contacts, cap_contact_count, contact_count);
	contact_0 = util.vector3_wide_select(use_cap_contacts, cap_contact_0, contact_0);
	contact_1 = util.vector3_wide_select(use_cap_contacts, cap_contact_1, contact_1);
	capsule_tangent := util.vector3_wide_cross(local_normal, capsule_axis);
	face_normal_a := util.vector3_wide_cross(capsule_tangent, capsule_axis);
	face_normal_a_dot_local_normal := util.vector3_wide_dot(face_normal_a, local_normal);
	inverse_face_dot := simd.div(util.F32x8(1), face_normal_a_dot_local_normal);
	manifold.depth_0 = simd.add(
		a.radius,
		simd.mul(
		util.vector3_wide_dot(util.vector3_wide_add(local_offset_b, contact_0), face_normal_a),
		inverse_face_dot
	),
	);
	manifold.depth_1 = simd.add(
		a.radius,
		simd.mul(
		util.vector3_wide_dot(util.vector3_wide_add(local_offset_b, contact_1), face_normal_a),
		inverse_face_dot
	),
	);
	collapse := transmute(util.I32x8)simd.lanes_lt(simd.abs(face_normal_a_dot_local_normal), util.F32x8(1e-7));
	manifold.depth_0 = util.wide_select_f32(collapse, depth, manifold.depth_0);
	manifold.contact_0_exists = transmute(util.I32x8)simd.lanes_ge(manifold.depth_0, negative_margin) & ~inactive;
	manifold.contact_1_exists = transmute(util.I32x8)simd.lanes_eq(contact_count, util.I32x8(2)) & ~collapse &
		transmute(util.I32x8)simd.lanes_ge(manifold.depth_1, negative_margin) & ~inactive;
	manifold.normal = util.matrix3x3_wide_transform(local_normal, world_r_b);
	manifold.offset_a_0 = util.vector3_wide_add(util.matrix3x3_wide_transform(contact_0, world_r_b), offset_b);
	manifold.offset_a_1 = util.vector3_wide_add(util.matrix3x3_wide_transform(contact_1, world_r_b), offset_b);
	manifold.feature_id_0 = util.I32x8(0);
	manifold.feature_id_1 = util.I32x8(1);
	return manifold, .Ok;
}

Capsule_Hull_Depth_Context_Wide :: struct
{
	hulls:              Convex_Hull_Wide,
	local_offset_a:     util.Vector3_Wide,
	local_capsule_axis: util.Vector3_Wide,
	half_length:        util.F32x8,
}

capsule_hull_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Capsule_Hull_Depth_Context_Wide)(user_context);
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
		direction_lane := util.vector3_wide_read_slot(direction, lane);
		hull_support, hull_status := convex_hull_support(depth_context.hulls.hulls[lane], direction_lane);
		if hull_status != .Ok
		{
			return {}, {}, hull_status;
		}
		axis := util.vector3_wide_read_slot(depth_context.local_capsule_axis, lane);
		capsule_support := util.vector3_scale(axis, -simd.extract(depth_context.half_length, lane));
		if util.vector3_dot(util.vector3_negate(direction_lane), axis) >= 0
		{
			capsule_support = util.vector3_negate(capsule_support);
		}
		capsule_support = util.vector3_add(
			capsule_support,
			util.vector3_wide_read_slot(depth_context.local_offset_a, lane)
		);
		util.vector3_wide_write_slot(&support_on_a, lane, hull_support);
		util.vector3_wide_write_slot(&support, lane, util.vector3_subtract(hull_support, capsule_support));
	}
	return support, support_on_a, .Ok;
}

capsule_convex_hull_test_wide :: proc "contextless" (
	a: Capsule_Wide, b: Convex_Hull_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_2_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	inactive := ~active;
	capsule_orientation := util.matrix3x3_wide_from_quaternion(orientation_a);
	hull_orientation := util.matrix3x3_wide_from_quaternion(orientation_b);
	hull_local_capsule_orientation := util.matrix3x3_wide_multiply_by_transpose(capsule_orientation, hull_orientation);
	local_capsule_axis := hull_local_capsule_orientation.y;
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
	depth_threshold := simd.neg(speculative_margin);
	depth_context := Capsule_Hull_Depth_Context_Wide{
		hulls=b,
		local_offset_a=local_offset_a,
		local_capsule_axis=local_capsule_axis,
		half_length=a.half_length,
	};
	depth, local_normal, closest_on_hull, depth_status := depth_refiner_find_minimum_depth_wide(
		capsule_hull_depth_support_wide, &depth_context, initial_normal, inactive,
		simd.mul(util.F32x8(1e-5), epsilon_scale), depth_threshold,
		util.F32x8(0), a.radius,
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	inactive |= transmute(util.I32x8)simd.lanes_lt(depth, depth_threshold);
	manifold: Convex_2_Contact_Manifold_Wide;
	if depth_refiner_all(inactive) == .Present
	{
		return manifold, .Ok;
	}

	face_normal_bundle: util.Vector3_Wide;
	latest_entry_numerator_bundle := util.F32x8(0);
	latest_entry_denominator_bundle := util.F32x8(0);
	earliest_exit_numerator_bundle := util.F32x8(0);
	earliest_exit_denominator_bundle := util.F32x8(0);
	bounding_plane_epsilon := simd.mul(util.F32x8(1e-3), epsilon_scale);
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		hull := b.hulls[lane];
		slot_local_normal := util.vector3_wide_read_slot(local_normal, lane);
		slot_closest_on_hull := util.vector3_wide_read_slot(closest_on_hull, lane);
		face_normal, best_face_index, face_status := convex_hull_pick_representative_face(
			hull, slot_local_normal, slot_closest_on_hull, simd.extract(bounding_plane_epsilon, lane),
		);
		if face_status != .Ok
		{
			return {}, face_status;
		}
		util.vector3_wide_write_slot(&face_normal_bundle, lane, face_normal);
		start, end, range_status := convex_hull_face_range(hull, best_face_index);
		if range_status != .Ok
		{
			return {}, range_status;
		}
		previous_vertex := convex_hull_get_point(hull, int(hull.face_vertex_indices.memory[end - 1]));
		slot_capsule_axis := util.vector3_wide_read_slot(local_capsule_axis, lane);
		slot_local_offset_a := util.vector3_wide_read_slot(local_offset_a, lane);
		latest_entry_numerator := f32(3.402823466e+38);
		latest_entry_denominator := f32(-1);
		earliest_exit_numerator := f32(3.402823466e+38);
		earliest_exit_denominator := f32(1);
		for vertex_index in start ..< end
		{
			vertex := convex_hull_get_point(hull, int(hull.face_vertex_indices.memory[vertex_index]));
			edge_offset := util.vector3_subtract(vertex, previous_vertex);
			edge_plane_normal := util.vector3_cross(edge_offset, slot_local_normal);
			capsule_to_edge := util.vector3_subtract(previous_vertex, slot_local_offset_a);
			numerator := util.vector3_dot(capsule_to_edge, edge_plane_normal);
			denominator := util.vector3_dot(edge_plane_normal, slot_capsule_axis);
			previous_vertex = vertex;
			edge_plane_normal_length_squared := util.vector3_length_squared(edge_plane_normal);
			denominator_squared := denominator * denominator;
			minimum :: f32(1e-5);
			maximum :: f32(3e-4);
			if denominator_squared > minimum * edge_plane_normal_length_squared
			{
				if denominator_squared < maximum * edge_plane_normal_length_squared
				{
					restrict_weight := (denominator_squared / edge_plane_normal_length_squared - minimum) / (maximum - minimum);
					restrict_weight = max(f32(0), min(f32(1), restrict_weight));
					unrestricted_numerator := simd.extract(a.half_length, lane) * denominator;
					if denominator < 0
					{
						unrestricted_numerator = -unrestricted_numerator;
					}
					numerator = restrict_weight * numerator + (1 - restrict_weight) * unrestricted_numerator;
				}
				if denominator < 0
				{
					if numerator * latest_entry_denominator > latest_entry_numerator * denominator
					{
						latest_entry_numerator = numerator;
						latest_entry_denominator = denominator;
					}
				}
				else if numerator * earliest_exit_denominator < earliest_exit_numerator * denominator
				{
					earliest_exit_numerator = numerator;
					earliest_exit_denominator = denominator;
				}
			}
		}
		latest_entry_numerator_bundle = simd.replace(latest_entry_numerator_bundle, lane, latest_entry_numerator);
		latest_entry_denominator_bundle = simd.replace(latest_entry_denominator_bundle, lane, latest_entry_denominator);
		earliest_exit_numerator_bundle = simd.replace(earliest_exit_numerator_bundle, lane, earliest_exit_numerator);
		earliest_exit_denominator_bundle = simd.replace(
			earliest_exit_denominator_bundle,
			lane,
			earliest_exit_denominator
		);
	}

	t_entry := simd.div(latest_entry_numerator_bundle, latest_entry_denominator_bundle);
	t_exit := simd.div(earliest_exit_numerator_bundle, earliest_exit_denominator_bundle);
	t_entry = simd.max(simd.neg(a.half_length), simd.min(a.half_length, t_entry));
	t_exit = simd.max(simd.neg(a.half_length), simd.min(a.half_length, t_exit));
	local_offset_0 := util.vector3_wide_scale(local_capsule_axis, t_entry);
	local_offset_1 := util.vector3_wide_scale(local_capsule_axis, t_exit);
	a_to_point_on_hull_face := util.vector3_wide_add(local_offset_b, closest_on_hull);
	depth_denominator := util.vector3_wide_dot(face_normal_bundle, local_normal);
	inverse_depth_denominator := simd.div(util.F32x8(1), depth_denominator);
	contact_0_to_hull_face := util.vector3_wide_subtract(a_to_point_on_hull_face, local_offset_0);
	contact_1_to_hull_face := util.vector3_wide_subtract(a_to_point_on_hull_face, local_offset_1);
	unexpanded_depth_0 := simd.mul(
		util.vector3_wide_dot(contact_0_to_hull_face, face_normal_bundle),
		inverse_depth_denominator
	);
	unexpanded_depth_1 := simd.mul(
		util.vector3_wide_dot(contact_1_to_hull_face, face_normal_bundle),
		inverse_depth_denominator
	);
	manifold.depth_0 = simd.add(a.radius, unexpanded_depth_0);
	manifold.depth_1 = simd.add(a.radius, unexpanded_depth_1);
	manifold.feature_id_0 = util.I32x8(0);
	manifold.feature_id_1 = util.I32x8(1);
	manifold.contact_0_exists = transmute(util.I32x8)simd.lanes_ge(manifold.depth_0, depth_threshold) & ~inactive;
	manifold.contact_1_exists =
		transmute(util.I32x8)simd.lanes_gt(simd.sub(t_exit, t_entry), simd.mul(a.half_length, util.F32x8(1e-3))) &
		transmute(util.I32x8)simd.lanes_ge(manifold.depth_1, depth_threshold) & ~inactive;
	manifold.offset_a_0 = util.matrix3x3_wide_transform(local_offset_0, hull_orientation);
	manifold.offset_a_1 = util.matrix3x3_wide_transform(local_offset_1, hull_orientation);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, hull_orientation);
	manifold.offset_a_0 = util.vector3_wide_add(
		manifold.offset_a_0,
		util.vector3_wide_scale(manifold.normal, unexpanded_depth_0)
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		manifold.offset_a_1,
		util.vector3_wide_scale(manifold.normal, unexpanded_depth_1)
	);
	return manifold, .Ok;
}
