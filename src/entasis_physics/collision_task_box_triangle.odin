// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

box_triangle_test_source :: proc "contextless" (
	box_a: Box, triangle_b: Triangle, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if box_validate(box_a) != .Ok || triangle_validate(triangle_b) != .Ok || speculative_margin < 0
	{
		return {}, .Invalid_Description;
	}
	box_wide: Box_Wide;
	triangle_wide: Triangle_Wide;
	_ = box_wide_write_slot(&box_wide, 0, box_a);
	_ = triangle_wide_write_slot(&triangle_wide, 0, triangle_b);
	offset_b: util.Vector3_Wide;
	orientation_a, orientation_b: util.Quaternion_Wide;
	util.vector3_wide_write_slot(&offset_b, 0, util.vector3_subtract(pose_b.position, pose_a.position));
	util.quaternion_wide_write_slot(&orientation_a, 0, pose_a.orientation);
	util.quaternion_wide_write_slot(&orientation_b, 0, pose_b.orientation);
	wide, status := box_triangle_test_wide(
		box_wide, triangle_wide, util.F32x8(speculative_margin),
		offset_b, orientation_a, orientation_b, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_4_manifold_wide_read_lane(&wide, offset_b, 0);
}

collision_box_triangle_interval_depth_wide :: proc "contextless" (
	box_extreme, a, b, c: util.F32x8,
) -> util.F32x8
{
	minimum := simd.min(a, simd.min(b, c));
	maximum := simd.max(a, simd.max(b, c));
	return simd.min(simd.sub(box_extreme, minimum), simd.add(maximum, box_extreme));
}

collision_box_triangle_test_edge_axis_wide :: proc "contextless" (
	triangle_edge_offset_y, triangle_edge_offset_z: util.F32x8,
	triangle_center_y, triangle_center_z: util.F32x8,
	edge_offset_y_squared, edge_offset_z_squared: util.F32x8,
	half_height, half_length: util.F32x8,
	v_ay, v_az, v_by, v_bz, v_cy, v_cz: util.F32x8,
) -> (depth, local_normal_x, local_normal_y, local_normal_z: util.F32x8)
{
	local_normal_x = util.F32x8(0);
	local_normal_y = triangle_edge_offset_z;
	local_normal_z = simd.neg(triangle_edge_offset_y);
	calibration_dot := simd.add(
		simd.mul(triangle_center_y, local_normal_y), simd.mul(triangle_center_z, local_normal_z),
	);
	length := simd.sqrt(simd.add(edge_offset_y_squared, edge_offset_z_squared));
	signed_one := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(calibration_dot, util.F32x8(0)),
		util.F32x8(1), util.F32x8(-1),
	);
	inverse_length := simd.div(signed_one, length);
	local_normal_y = simd.mul(local_normal_y, inverse_length);
	local_normal_z = simd.mul(local_normal_z, inverse_length);
	box_extreme := simd.add(
		simd.mul(simd.abs(local_normal_y), half_height),
		simd.mul(simd.abs(local_normal_z), half_length),
	);
	n_va := simd.add(simd.mul(v_ay, local_normal_y), simd.mul(v_az, local_normal_z));
	n_vb := simd.add(simd.mul(v_by, local_normal_y), simd.mul(v_bz, local_normal_z));
	n_vc := simd.add(simd.mul(v_cy, local_normal_y), simd.mul(v_cz, local_normal_z));
	depth = collision_box_triangle_interval_depth_wide(box_extreme, n_va, n_vb, n_vc);
	depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-7)),
		util.F32x8(3.402823466e+38), depth,
	);
	return;
}

collision_box_triangle_test_edges_wide :: proc "contextless" (
	box: Box_Wide, triangle_edge_offset, triangle_center: util.Vector3_Wide,
	v_a, v_b, v_c: util.Vector3_Wide,
) -> (depth: util.F32x8, local_normal: util.Vector3_Wide)
{
	x_squared := simd.mul(triangle_edge_offset.x, triangle_edge_offset.x);
	y_squared := simd.mul(triangle_edge_offset.y, triangle_edge_offset.y);
	z_squared := simd.mul(triangle_edge_offset.z, triangle_edge_offset.z);
	depth, local_normal.x, local_normal.y, local_normal.z = collision_box_triangle_test_edge_axis_wide(
		triangle_edge_offset.y, triangle_edge_offset.z,
		triangle_center.y, triangle_center.z, y_squared, z_squared,
		box.half_height, box.half_length,
		v_a.y, v_a.z, v_b.y, v_b.z, v_c.y, v_c.z,
	);
	candidate_depth, candidate_y, candidate_x, candidate_z := collision_box_triangle_test_edge_axis_wide(
		triangle_edge_offset.x, triangle_edge_offset.z,
		triangle_center.x, triangle_center.z, x_squared, z_squared,
		box.half_width, box.half_length,
		v_a.x, v_a.z, v_b.x, v_b.z, v_c.x, v_c.z,
	);
	candidate_normal := util.Vector3_Wide{x=candidate_x, y=candidate_y, z=candidate_z};
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, candidate_normal);
	candidate_depth, candidate_z, candidate_x, candidate_y = collision_box_triangle_test_edge_axis_wide(
		triangle_edge_offset.x, triangle_edge_offset.y,
		triangle_center.x, triangle_center.y, x_squared, y_squared,
		box.half_width, box.half_height,
		v_a.x, v_a.y, v_b.x, v_b.y, v_c.x, v_c.y,
	);
	candidate_normal = {x=candidate_x, y=candidate_y, z=candidate_z};
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, candidate_normal);
	return;
}

collision_box_triangle_add_candidate_wide :: proc "contextless" (
	point_on_triangle, triangle_center, triangle_tangent_x, triangle_tangent_y: util.Vector3_Wide,
	feature_id, exists: util.I32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, pair_count: int,
)
{
	offset := util.vector3_wide_subtract(point_on_triangle, triangle_center);
	candidate := Manifold_Candidate_Wide{
		x=util.vector3_wide_dot(offset, triangle_tangent_x),
		y=util.vector3_wide_dot(offset, triangle_tangent_y),
		feature_id=feature_id,
	};
	manifold_candidate_wide_add(candidates, candidate_count, candidate, exists, pair_count);
}

collision_box_triangle_clip_edge_planes_wide :: proc "contextless" (
	edge_direction, triangle_edge_start_to_box_edge_anchor_0,
	triangle_edge_start_to_box_edge_anchor_1, box_edge_plane_normal: util.Vector3_Wide,
) -> (minimum, maximum: util.F32x8)
{
	distance_0 := util.vector3_wide_dot(triangle_edge_start_to_box_edge_anchor_0, box_edge_plane_normal);
	distance_1 := util.vector3_wide_dot(triangle_edge_start_to_box_edge_anchor_1, box_edge_plane_normal);
	velocity := util.vector3_wide_dot(box_edge_plane_normal, edge_direction);
	inverse_velocity := simd.div(util.F32x8(1), velocity);
	edge_start_inside := transmute(util.I32x8)simd.lanes_le(
		simd.mul(distance_0, distance_1), util.F32x8(0),
	);
	nonparallel := transmute(util.I32x8)simd.lanes_gt(simd.abs(velocity), util.F32x8(1e-15));
	t_0 := simd.mul(distance_0, inverse_velocity);
	t_1 := simd.mul(distance_1, inverse_velocity);
	large_negative, large_positive := util.F32x8(-3.402823466e+38), util.F32x8(3.402823466e+38);
	minimum = util.wide_select_f32(
		nonparallel, simd.min(t_0, t_1),
		util.wide_select_f32(edge_start_inside, large_negative, large_positive),
	);
	maximum = util.wide_select_f32(
		nonparallel, simd.max(t_0, t_1),
		util.wide_select_f32(edge_start_inside, large_positive, large_negative),
	);
	return;
}

collision_box_triangle_clip_edge_wide :: proc "contextless" (
	edge_start, edge_direction: util.Vector3_Wide, edge_id: util.I32x8,
	box_vertex_00, box_vertex_11, edge_plane_normal_x, edge_plane_normal_y: util.Vector3_Wide,
	triangle_center, triangle_tangent_x, triangle_tangent_y: util.Vector3_Wide,
	allow_contacts: util.I32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, pair_count: int,
)
{
	edge_start_to_v00 := util.vector3_wide_subtract(box_vertex_00, edge_start);
	edge_start_to_v11 := util.vector3_wide_subtract(box_vertex_11, edge_start);
	min_x, max_x := collision_box_triangle_clip_edge_planes_wide(
		edge_direction, edge_start_to_v00, edge_start_to_v11, edge_plane_normal_x,
	);
	min_y, max_y := collision_box_triangle_clip_edge_planes_wide(
		edge_direction, edge_start_to_v00, edge_start_to_v11, edge_plane_normal_y,
	);
	minimum := simd.max(min_x, min_y);
	maximum := simd.min(util.F32x8(1), simd.min(max_x, max_y));
	minimum_location := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, minimum));
	maximum_location := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, maximum));
	minimum_exists := allow_contacts &
		transmute(util.I32x8)simd.lanes_lt(candidate_count^, util.I32x8(6)) &
		transmute(util.I32x8)simd.lanes_ge(simd.sub(maximum, minimum), util.F32x8(1e-5)) &
		transmute(util.I32x8)simd.lanes_lt(minimum, util.F32x8(1)) &
		transmute(util.I32x8)simd.lanes_gt(minimum, util.F32x8(0));
	collision_box_triangle_add_candidate_wide(
		minimum_location, triangle_center, triangle_tangent_x, triangle_tangent_y,
		edge_id, minimum_exists, candidates, candidate_count, pair_count,
	);
	maximum_exists := allow_contacts &
		transmute(util.I32x8)simd.lanes_lt(candidate_count^, util.I32x8(6)) &
		transmute(util.I32x8)simd.lanes_ge(maximum, minimum) &
		transmute(util.I32x8)simd.lanes_le(maximum, util.F32x8(1)) &
		transmute(util.I32x8)simd.lanes_ge(maximum, util.F32x8(0));
	collision_box_triangle_add_candidate_wide(
		maximum_location, triangle_center, triangle_tangent_x, triangle_tangent_y,
		simd.add(edge_id, util.I32x8(8)), maximum_exists, candidates, candidate_count, pair_count,
	);
}

collision_box_triangle_clip_edges_wide :: proc "contextless" (
	a, b, c, triangle_center, triangle_tangent_x, triangle_tangent_y: util.Vector3_Wide,
	ab, bc, ca, box_vertex_00, box_vertex_11, box_tangent_x,
	box_tangent_y, contact_normal: util.Vector3_Wide,
	allow_contacts: util.I32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, pair_count: int,
)
{
	edge_plane_normal_x := util.vector3_wide_cross(box_tangent_y, contact_normal);
	edge_plane_normal_y := util.vector3_wide_cross(box_tangent_x, contact_normal);
	base_id := util.I32x8(4);
	collision_box_triangle_clip_edge_wide(
		a, ab, base_id, box_vertex_00, box_vertex_11, edge_plane_normal_x, edge_plane_normal_y,
		triangle_center, triangle_tangent_x, triangle_tangent_y,
		allow_contacts, candidates, candidate_count, pair_count,
	);
	collision_box_triangle_clip_edge_wide(
		b, bc, simd.add(base_id, util.I32x8(1)), box_vertex_00, box_vertex_11,
		edge_plane_normal_x, edge_plane_normal_y, triangle_center, triangle_tangent_x, triangle_tangent_y,
		allow_contacts, candidates, candidate_count, pair_count,
	);
	collision_box_triangle_clip_edge_wide(
		c, ca, simd.add(base_id, util.I32x8(2)), box_vertex_00, box_vertex_11,
		edge_plane_normal_x, edge_plane_normal_y, triangle_center, triangle_tangent_x, triangle_tangent_y,
		allow_contacts, candidates, candidate_count, pair_count,
	);
}

collision_box_triangle_add_box_vertex_wide :: proc "contextless" (
	a, b, vertex, triangle_normal, contact_normal: util.Vector3_Wide,
	inverse_normal_dot: util.F32x8, ab_edge_plane_normal,
	bc_edge_plane_normal, ca_edge_plane_normal: util.Vector3_Wide,
	triangle_center, triangle_x, triangle_y: util.Vector3_Wide,
	feature_id, allow_contacts: util.I32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, pair_count: int,
)
{
	point_on_triangle_to_box_vertex := util.vector3_wide_subtract(vertex, a);
	plane_distance := util.vector3_wide_dot(triangle_normal, point_on_triangle_to_box_vertex);
	offset := util.vector3_wide_scale(contact_normal, simd.mul(plane_distance, inverse_normal_dot));
	vertex_on_plane := util.vector3_wide_subtract(vertex, offset);
	a_to_vertex := util.vector3_wide_subtract(vertex_on_plane, a);
	b_to_vertex := util.vector3_wide_subtract(vertex_on_plane, b);
	ab_dot := util.vector3_wide_dot(a_to_vertex, ab_edge_plane_normal);
	bc_dot := util.vector3_wide_dot(b_to_vertex, bc_edge_plane_normal);
	ca_dot := util.vector3_wide_dot(a_to_vertex, ca_edge_plane_normal);
	contained := allow_contacts &
		transmute(util.I32x8)simd.lanes_ge(ab_dot, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(bc_dot, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(ca_dot, util.F32x8(0));
	collision_box_triangle_add_candidate_wide(
		vertex_on_plane, triangle_center, triangle_x, triangle_y,
		feature_id, contained, candidates, candidate_count, pair_count,
	);
}

collision_box_triangle_add_box_vertices_wide :: proc "contextless" (
	a, b, ab, bc, ca, triangle_normal, contact_normal: util.Vector3_Wide,
	v_00, v_01, v_10, v_11, triangle_center, triangle_x, triangle_y: util.Vector3_Wide,
	base_feature_id, feature_id_x, feature_id_y, allow_contacts: util.I32x8,
	candidates: ^[8]Manifold_Candidate_Wide, candidate_count: ^util.I32x8, pair_count: int,
)
{
	normal_dot := util.vector3_wide_dot(triangle_normal, contact_normal);
	inverse_normal_dot := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(simd.abs(normal_dot), util.F32x8(1e-10)),
		simd.div(util.F32x8(1), normal_dot), util.F32x8(3.402823466e+38),
	);
	ab_edge_plane_normal := util.vector3_wide_cross(ab, triangle_normal);
	bc_edge_plane_normal := util.vector3_wide_cross(bc, triangle_normal);
	ca_edge_plane_normal := util.vector3_wide_cross(ca, triangle_normal);
	vertices := [4]util.Vector3_Wide{v_00, v_01, v_10, v_11};
	feature_ids := [4]util.I32x8{
		base_feature_id,
		simd.add(base_feature_id, feature_id_y),
		simd.add(base_feature_id, feature_id_x),
		simd.add(base_feature_id, simd.add(feature_id_x, feature_id_y)),
	};
	for vertex_index in 0 ..< 4
	{
		collision_box_triangle_add_box_vertex_wide(
			a, b, vertices[vertex_index], triangle_normal, contact_normal, inverse_normal_dot,
			ab_edge_plane_normal, bc_edge_plane_normal, ca_edge_plane_normal,
			triangle_center, triangle_x, triangle_y, feature_ids[vertex_index], allow_contacts,
			candidates, candidate_count, pair_count,
		);
	}
}

box_triangle_test_wide :: proc "contextless" (
	box_a: Box_Wide, triangle_b: Triangle_Wide, speculative_margin: util.F32x8,
	offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	world_r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	world_r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	r_b := util.matrix3x3_wide_multiply_by_transpose(world_r_b, world_r_a);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_a);
	v_a := util.vector3_wide_add(util.matrix3x3_wide_transform(triangle_b.a, r_b), local_offset_b);
	v_b := util.vector3_wide_add(util.matrix3x3_wide_transform(triangle_b.b, r_b), local_offset_b);
	v_c := util.vector3_wide_add(util.matrix3x3_wide_transform(triangle_b.c, r_b), local_offset_b);
	local_triangle_center := util.vector3_wide_scale(
		util.vector3_wide_add(v_a, util.vector3_wide_add(v_b, v_c)), util.F32x8(1.0 / 3.0),
	);
	ab := util.vector3_wide_subtract(v_b, v_a);
	bc := util.vector3_wide_subtract(v_c, v_b);
	ca := util.vector3_wide_subtract(v_a, v_c);
	depth, local_normal := collision_box_triangle_test_edges_wide(
		box_a, ab, local_triangle_center, v_a, v_b, v_c,
	);
	candidate_depth, candidate_normal := collision_box_triangle_test_edges_wide(
		box_a, bc, local_triangle_center, v_a, v_b, v_c,
	);
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, candidate_normal);
	candidate_depth, candidate_normal = collision_box_triangle_test_edges_wide(
		box_a, ca, local_triangle_center, v_a, v_b, v_c,
	);
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, candidate_normal);

	x_normal_sign := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(local_triangle_center.x, util.F32x8(0)),
		util.F32x8(1), util.F32x8(-1),
	);
	y_normal_sign := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(local_triangle_center.y, util.F32x8(0)),
		util.F32x8(1), util.F32x8(-1),
	);
	z_normal_sign := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(local_triangle_center.z, util.F32x8(0)),
		util.F32x8(1), util.F32x8(-1),
	);
	face_ax_depth := collision_box_triangle_interval_depth_wide(box_a.half_width, v_a.x, v_b.x, v_c.x);
	face_ay_depth := collision_box_triangle_interval_depth_wide(box_a.half_height, v_a.y, v_b.y, v_c.y);
	face_az_depth := collision_box_triangle_interval_depth_wide(box_a.half_length, v_a.z, v_b.z, v_c.z);
	collision_box_pair_select_axis_wide(&depth, &local_normal, face_ax_depth, {x=x_normal_sign});
	collision_box_pair_select_axis_wide(&depth, &local_normal, face_ay_depth, {y=y_normal_sign});
	collision_box_pair_select_axis_wide(&depth, &local_normal, face_az_depth, {z=z_normal_sign});

	triangle_normal_unnormalized := util.vector3_wide_cross(ab, ca);
	triangle_normal_length := util.vector3_wide_length(triangle_normal_unnormalized);
	triangle_normal := util.vector3_wide_scale(
		triangle_normal_unnormalized, simd.div(util.F32x8(1), triangle_normal_length),
	);
	triangle_plane_offset := util.vector3_wide_dot(triangle_normal, local_triangle_center);
	calibrated_triangle_normal := util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(triangle_plane_offset, util.F32x8(0)), triangle_normal,
	);
	triangle_face_depth := simd.sub(
		simd.add(
		simd.add(
		simd.mul(simd.abs(triangle_normal.x), box_a.half_width),
		simd.mul(simd.abs(triangle_normal.y), box_a.half_height),
	),
		simd.mul(simd.abs(triangle_normal.z), box_a.half_length),
	),
		simd.abs(triangle_plane_offset),
	);
	collision_box_pair_select_axis_wide(
		&depth, &local_normal, triangle_face_depth, calibrated_triangle_normal,
	);

	normal_dot := util.vector3_wide_dot(local_normal, triangle_normal);
	minimum_depth := simd.neg(speculative_margin);
	ab_length_squared := util.vector3_wide_length_squared(ab);
	ca_length_squared := util.vector3_wide_length_squared(ca);
	triangle_epsilon_scale := simd.sqrt(simd.max(ab_length_squared, ca_length_squared));
	nondegenerate := transmute(util.I32x8)simd.lanes_gt(
		triangle_normal_length, simd.mul(triangle_epsilon_scale, util.F32x8(1e-6)),
	);
	allow_contacts := nondegenerate &
		transmute(util.I32x8)simd.lanes_ge(normal_dot, util.F32x8(TRIANGLE_BACKFACE_REJECTION_THRESHOLD)) &
		transmute(util.I32x8)simd.lanes_ge(depth, minimum_depth) & active;
	manifold: Convex_4_Contact_Manifold_Wide;
	if depth_refiner_all(~allow_contacts) == .Present
	{
		return manifold, .Ok;
	}

	abs_normal_x := simd.abs(local_normal.x);
	abs_normal_y := simd.abs(local_normal.y);
	abs_normal_z := simd.abs(local_normal.z);
	use_ax := transmute(util.I32x8)simd.lanes_gt(abs_normal_x, abs_normal_y) &
		transmute(util.I32x8)simd.lanes_gt(abs_normal_x, abs_normal_z);
	use_ay := transmute(util.I32x8)simd.lanes_gt(abs_normal_y, abs_normal_z) & ~use_ax;
	use_az := ~(use_ax | use_ay);
	normal_is_negative_x := transmute(util.I32x8)simd.lanes_lt(local_normal.x, util.F32x8(0));
	normal_is_negative_y := transmute(util.I32x8)simd.lanes_lt(local_normal.y, util.F32x8(0));
	normal_is_negative_z := transmute(util.I32x8)simd.lanes_lt(local_normal.z, util.F32x8(0));
	zero, one, negative_one := util.F32x8(0), util.F32x8(1), util.F32x8(-1);
	box_tangent_x := util.Vector3_Wide{
		x=util.wide_select_f32(use_ay | use_az, one, zero),
		y=zero,
		z=util.wide_select_f32(use_ax, one, zero),
	};
	box_tangent_y := util.Vector3_Wide{
		x=zero,
		y=util.wide_select_f32(use_ax | use_az, one, zero),
		z=util.wide_select_f32(use_ay, one, zero),
	};
	box_face_normal := util.Vector3_Wide{
		x=util.wide_select_f32(
			use_ax, util.wide_select_f32(normal_is_negative_x, one, negative_one), zero,
		),
		y=util.wide_select_f32(
			use_ay, util.wide_select_f32(normal_is_negative_y, one, negative_one), zero,
		),
		z=util.wide_select_f32(
			use_az, util.wide_select_f32(normal_is_negative_z, one, negative_one), zero,
		),
	};
	half_extent_x := util.wide_select_f32(use_ax, box_a.half_length, box_a.half_width);
	half_extent_y := util.wide_select_f32(use_ay, box_a.half_length, box_a.half_height);
	half_extent_z := util.wide_select_f32(
		use_ax, box_a.half_width, util.wide_select_f32(use_ay, box_a.half_height, box_a.half_length),
	);
	box_face_center := util.vector3_wide_scale(box_face_normal, half_extent_z);
	local_x_id, local_y_id, local_z_id := util.I32x8(0), util.I32x8(1), util.I32x8(2);
	axis_id_tangent_x := collision_wide_select_i32(use_ax, local_z_id, local_x_id);
	axis_id_tangent_y := collision_wide_select_i32(use_ay, local_z_id, local_y_id);
	axis_id_normal := collision_wide_select_i32(
		use_ax, collision_wide_select_i32(normal_is_negative_x, local_x_id, util.I32x8(0)),
		collision_wide_select_i32(
		use_ay, collision_wide_select_i32(normal_is_negative_y, local_y_id, util.I32x8(0)),
		collision_wide_select_i32(normal_is_negative_z, local_z_id, util.I32x8(0)),
	),
	);
	epsilon_scale := simd.min(
		simd.max(box_a.half_width, simd.max(box_a.half_height, box_a.half_length)),
		triangle_epsilon_scale,
	);
	triangle_tangent_x := util.vector3_wide_scale(
		ab, simd.div(util.F32x8(1), simd.sqrt(ab_length_squared)),
	);
	triangle_tangent_y := util.vector3_wide_cross(triangle_tangent_x, triangle_normal);
	candidates: [8]Manifold_Candidate_Wide;
	candidate_count: util.I32x8;
	box_edge_offset_x := util.vector3_wide_scale(box_tangent_x, half_extent_x);
	box_edge_offset_y := util.vector3_wide_scale(box_tangent_y, half_extent_y);
	positive_x := util.vector3_wide_add(box_face_center, box_edge_offset_x);
	negative_x := util.vector3_wide_subtract(box_face_center, box_edge_offset_x);
	box_vertex_00 := util.vector3_wide_subtract(negative_x, box_edge_offset_y);
	box_vertex_01 := util.vector3_wide_add(negative_x, box_edge_offset_y);
	box_vertex_10 := util.vector3_wide_subtract(positive_x, box_edge_offset_y);
	box_vertex_11 := util.vector3_wide_add(positive_x, box_edge_offset_y);
	collision_box_triangle_add_box_vertices_wide(
		v_a, v_b, ab, bc, ca, triangle_normal, local_normal,
		box_vertex_00, box_vertex_01, box_vertex_10, box_vertex_11,
		local_triangle_center, triangle_tangent_x, triangle_tangent_y,
		axis_id_normal, axis_id_tangent_x, axis_id_tangent_y, allow_contacts,
		&candidates, &candidate_count, pair_count,
	);
	collision_box_triangle_clip_edges_wide(
		v_a, v_b, v_c, local_triangle_center, triangle_tangent_x, triangle_tangent_y,
		ab, bc, ca, box_vertex_00, box_vertex_11, box_tangent_x, box_tangent_y, local_normal,
		allow_contacts, &candidates, &candidate_count, pair_count,
	);
	face_center_b_to_face_center_a := util.vector3_wide_subtract(box_face_center, local_triangle_center);
	face_normal_dot_normal := util.vector3_wide_dot(box_face_normal, local_normal);
	contacts, contact_exists := manifold_candidate_wide_reduce(
		&candidates, candidate_count, 6, box_face_normal,
		simd.div(util.F32x8(1), face_normal_dot_normal), face_center_b_to_face_center_a,
		triangle_tangent_x, triangle_tangent_y, epsilon_scale, minimum_depth, pair_count,
	);
	world_triangle_center := util.matrix3x3_wide_transform(local_triangle_center, world_r_a);
	world_tangent_x := util.matrix3x3_wide_transform(triangle_tangent_x, world_r_a);
	world_tangent_y := util.matrix3x3_wide_transform(triangle_tangent_y, world_r_a);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, world_r_a);
	manifold.offset_a_0 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_x, contacts[0].x),
		util.vector3_wide_scale(world_tangent_y, contacts[0].y),
	),
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_x, contacts[1].x),
		util.vector3_wide_scale(world_tangent_y, contacts[1].y),
	),
	);
	manifold.offset_a_2 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_x, contacts[2].x),
		util.vector3_wide_scale(world_tangent_y, contacts[2].y),
	),
	);
	manifold.offset_a_3 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_x, contacts[3].x),
		util.vector3_wide_scale(world_tangent_y, contacts[3].y),
	),
	);
	manifold.depth_0, manifold.depth_1 = contacts[0].depth, contacts[1].depth;
	manifold.depth_2, manifold.depth_3 = contacts[2].depth, contacts[3].depth;
	manifold.feature_id_0, manifold.feature_id_1 = contacts[0].feature_id, contacts[1].feature_id;
	manifold.feature_id_2, manifold.feature_id_3 = contacts[2].feature_id, contacts[3].feature_id;
	manifold.contact_0_exists, manifold.contact_1_exists = contact_exists[0], contact_exists[1];
	manifold.contact_2_exists, manifold.contact_3_exists = contact_exists[2], contact_exists[3];
	face_flag := collision_wide_select_i32(
		transmute(util.I32x8)simd.lanes_ge(
		normal_dot, util.F32x8(MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION),
	),
		util.I32x8(MESH_REDUCTION_FACE_COLLISION_FLAG), util.I32x8(0),
	);
	manifold.feature_id_0 = simd.add(manifold.feature_id_0, face_flag);
	return manifold, .Ok;
}
