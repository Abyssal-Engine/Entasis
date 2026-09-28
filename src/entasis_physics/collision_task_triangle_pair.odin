// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

triangle_pair_test_source :: proc "contextless" (
	a, b: Triangle, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if triangle_validate(a) != .Ok || triangle_validate(b) != .Ok || speculative_margin < 0
	{
		return {}, .Invalid_Description;
	}
	a_wide, b_wide: Triangle_Wide;
	_ = triangle_wide_write_slot(&a_wide, 0, a);
	_ = triangle_wide_write_slot(&b_wide, 0, b);
	offset_b: util.Vector3_Wide;
	orientation_a, orientation_b: util.Quaternion_Wide;
	util.vector3_wide_write_slot(&offset_b, 0, util.vector3_subtract(pose_b.position, pose_a.position));
	util.quaternion_wide_write_slot(&orientation_a, 0, pose_a.orientation);
	util.quaternion_wide_write_slot(&orientation_b, 0, pose_b.orientation);
	wide, status := triangle_pair_test_wide(
		a_wide, b_wide, util.F32x8(speculative_margin),
		offset_b, orientation_a, orientation_b, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_4_manifold_wide_read_lane(&wide, offset_b, 0);
}

collision_triangle_pair_interval_wide :: proc "contextless" (
	a, b, c, normal: util.Vector3_Wide,
) -> (minimum, maximum: util.F32x8)
{
	dot_a := util.vector3_wide_dot(normal, a);
	dot_b := util.vector3_wide_dot(normal, b);
	dot_c := util.vector3_wide_dot(normal, c);
	minimum = simd.min(dot_a, simd.min(dot_b, dot_c));
	maximum = simd.max(dot_a, simd.max(dot_b, dot_c));
	return;
}

collision_triangle_pair_depth_wide :: proc "contextless" (
	vertices_a, vertices_b: [3]util.Vector3_Wide, normal: util.Vector3_Wide,
) -> util.F32x8
{
	minimum_a, maximum_a := collision_triangle_pair_interval_wide(
		vertices_a[0], vertices_a[1], vertices_a[2], normal,
	);
	minimum_b, maximum_b := collision_triangle_pair_interval_wide(
		vertices_b[0], vertices_b[1], vertices_b[2], normal,
	);
	return simd.min(simd.sub(maximum_a, minimum_b), simd.sub(maximum_b, minimum_a));
}

collision_triangle_pair_test_edge_wide :: proc "contextless" (
	edge_direction_a, edge_direction_b: util.Vector3_Wide,
	vertices_a, vertices_b: [3]util.Vector3_Wide,
) -> (depth: util.F32x8, normal: util.Vector3_Wide)
{
	normal = util.vector3_wide_cross(edge_direction_a, edge_direction_b);
	normal_length := util.vector3_wide_length(normal);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), normal_length));
	depth = collision_triangle_pair_depth_wide(vertices_a, vertices_b, normal);
	depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(normal_length, util.F32x8(1e-10)),
		util.F32x8(3.402823466e+38), depth,
	);
	return;
}

collision_triangle_pair_basis_wide :: proc "contextless" (
	normal: util.Vector3_Wide,
) -> (tangent_x, tangent_y: util.Vector3_Wide)
{
	negative_z := transmute(util.I32x8)simd.lanes_lt(normal.z, util.F32x8(0));
	sign := util.wide_select_f32(negative_z, util.F32x8(-1), util.F32x8(1));
	scale := simd.div(util.F32x8(-1), simd.add(sign, normal.z));
	tangent_x = {
		x=simd.mul(simd.mul(normal.x, normal.y), scale),
		y=simd.add(sign, simd.mul(simd.mul(normal.y, normal.y), scale)),
		z=simd.neg(normal.y),
	};
	tangent_y = {
		x=simd.add(
			util.F32x8(1), simd.mul(simd.mul(simd.mul(sign, normal.x), normal.x), scale),
		),
		y=simd.mul(sign, tangent_x.x),
		z=simd.neg(simd.mul(sign, normal.x)),
	};
	return;
}

collision_triangle_pair_try_add_a_vertex_wide :: proc "contextless" (
	vertex: util.Vector3_Wide, flattened_vertex: util.Vector2_Wide, vertex_id: util.I32x8,
	tangent_bx, tangent_by, triangle_center_b, contact_normal, face_normal_b: util.Vector3_Wide,
	edge_ab, edge_bc, edge_ca, flat_b_a, flat_b_b: util.Vector2_Wide,
	allow_contacts: util.I32x8, inverse_contact_normal_dot_face_normal_b,
	minimum_depth: util.F32x8, candidates: ^[8]Manifold_Candidate_Wide,
	candidate_count: ^util.I32x8, pair_count: int,
)
{
	b_a_to_vertex := util.vector2_wide_subtract(flattened_vertex, flat_b_a);
	b_b_to_vertex := util.vector2_wide_subtract(flattened_vertex, flat_b_b);
	ab_edge_plane_dot := simd.sub(
		simd.mul(b_a_to_vertex.y, edge_ab.x), simd.mul(b_a_to_vertex.x, edge_ab.y),
	);
	bc_edge_plane_dot := simd.sub(
		simd.mul(b_b_to_vertex.y, edge_bc.x), simd.mul(b_b_to_vertex.x, edge_bc.y),
	);
	ca_edge_plane_dot := simd.sub(
		simd.mul(b_a_to_vertex.y, edge_ca.x), simd.mul(b_a_to_vertex.x, edge_ca.y),
	);
	contained := transmute(util.I32x8)simd.lanes_gt(ab_edge_plane_dot, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(bc_edge_plane_dot, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(ca_edge_plane_dot, util.F32x8(0));
	offset := util.vector3_wide_subtract(triangle_center_b, vertex);
	distance := util.vector3_wide_dot(offset, face_normal_b);
	candidate := Manifold_Candidate_Wide{
		depth=simd.mul(distance, inverse_contact_normal_dot_face_normal_b),
		feature_id=vertex_id,
	};
	unprojected_vertex := util.vector3_wide_add(
		vertex, util.vector3_wide_scale(contact_normal, candidate.depth),
	);
	offset_on_b := util.vector3_wide_subtract(unprojected_vertex, triangle_center_b);
	candidate.x = util.vector3_wide_dot(offset_on_b, tangent_bx);
	candidate.y = util.vector3_wide_dot(offset_on_b, tangent_by);
	exists := transmute(util.I32x8)simd.lanes_ge(candidate.depth, minimum_depth) &
		allow_contacts & contained;
	manifold_candidate_wide_add(candidates, candidate_count, candidate, exists, pair_count);
}

collision_triangle_pair_clip_edge_wide :: proc "contextless" (
	edge_start_b, edge_offset_b, edge_start_a, edge_offset_a: util.Vector2_Wide,
	inverse_edge_length_squared_a, edge_start_a_dot_normal,
	edge_offset_a_dot_normal: util.F32x8,
) -> (intersection_exists: util.I32x8, t_b, depth_contribution_a: util.F32x8)
{
	edge_plane_normal_dot := simd.sub(
		simd.mul(simd.sub(edge_start_a.x, edge_start_b.x), edge_offset_a.y),
		simd.mul(simd.sub(edge_start_a.y, edge_start_b.y), edge_offset_a.x),
	);
	velocity := simd.sub(
		simd.mul(edge_offset_b.x, edge_offset_a.y), simd.mul(edge_offset_b.y, edge_offset_a.x),
	);
	parallel_threshold := util.F32x8(1e-20);
	parallel := transmute(util.I32x8)simd.lanes_lt(simd.abs(velocity), parallel_threshold);
	parallel_denominator := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(velocity, util.F32x8(0)),
		simd.neg(parallel_threshold), parallel_threshold,
	);
	denominator := util.wide_select_f32(parallel, parallel_denominator, velocity);
	t_b = simd.div(edge_plane_normal_dot, denominator);
	intersection_point_x := simd.add(simd.mul(t_b, edge_offset_b.x), edge_start_b.x);
	intersection_point_y := simd.add(simd.mul(t_b, edge_offset_b.y), edge_start_b.y);
	t_a := simd.mul(
		simd.add(
		simd.mul(simd.sub(intersection_point_x, edge_start_a.x), edge_offset_a.x),
		simd.mul(simd.sub(intersection_point_y, edge_start_a.y), edge_offset_a.y),
	),
		inverse_edge_length_squared_a,
	);
	intersection_exists = transmute(util.I32x8)simd.lanes_ge(t_a, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(t_a, util.F32x8(1));
	depth_contribution_a = simd.add(edge_start_a_dot_normal, simd.mul(edge_offset_a_dot_normal, t_a));
	return;
}

collision_triangle_pair_clip_b_edge_wide :: proc "contextless" (
	flat_vertices_a, flat_edges_a: [3]util.Vector2_Wide,
	inverse_flat_edge_length_squared_a, vertex_dot_normal_a,
	edge_dot_normal_a: [3]util.F32x8,
	flat_edge_start_b, flat_edge_offset_b: util.Vector2_Wide,
	edge_start_b, edge_offset_b: util.Vector3_Wide,
	entry_id, exit_id_offset: util.I32x8,
	triangle_center_b, tangent_bx, tangent_by, local_normal: util.Vector3_Wide,
	minimum_depth: util.F32x8, allow_contacts: util.I32x8,
	candidates: ^[8]Manifold_Candidate_Wide, candidate_count: ^util.I32x8, pair_count: int,
)
{
	intersections: [3]util.I32x8;
	t_values, depth_contributions_a: [3]util.F32x8;
	for edge_index in 0 ..< 3
	{
		intersections[edge_index], t_values[edge_index], depth_contributions_a[edge_index] =
			collision_triangle_pair_clip_edge_wide(
			flat_edge_start_b, flat_edge_offset_b,
			flat_vertices_a[edge_index], flat_edges_a[edge_index],
			inverse_flat_edge_length_squared_a[edge_index],
			vertex_dot_normal_a[edge_index], edge_dot_normal_a[edge_index],
		);
	}
	minimum_value, maximum_value := util.F32x8(-3.402823466e+38), util.F32x8(3.402823466e+38);
	entry_ab := util.wide_select_f32(intersections[0], t_values[0], maximum_value);
	entry_bc := util.wide_select_f32(intersections[1], t_values[1], maximum_value);
	entry_ca := util.wide_select_f32(intersections[2], t_values[2], maximum_value);
	exit_ab := util.wide_select_f32(intersections[0], t_values[0], minimum_value);
	exit_bc := util.wide_select_f32(intersections[1], t_values[1], minimum_value);
	exit_ca := util.wide_select_f32(intersections[2], t_values[2], minimum_value);
	entry := simd.min(entry_ab, simd.min(entry_bc, entry_ca));
	exit := simd.max(exit_ab, simd.max(exit_bc, exit_ca));
	use_ab_entry := transmute(util.I32x8)simd.lanes_eq(entry, t_values[0]);
	use_bc_entry := transmute(util.I32x8)simd.lanes_eq(entry, t_values[1]);
	use_ab_exit := transmute(util.I32x8)simd.lanes_eq(exit, t_values[0]);
	use_bc_exit := transmute(util.I32x8)simd.lanes_eq(exit, t_values[1]);
	depth_contribution_a_entry := util.wide_select_f32(
		use_ab_entry, depth_contributions_a[0],
		util.wide_select_f32(use_bc_entry, depth_contributions_a[1], depth_contributions_a[2]),
	);
	depth_contribution_a_exit := util.wide_select_f32(
		use_ab_exit, depth_contributions_a[0],
		util.wide_select_f32(use_bc_exit, depth_contributions_a[1], depth_contributions_a[2]),
	);
	invalid_interval := transmute(util.I32x8)simd.lanes_eq(entry, minimum_value) |
		transmute(util.I32x8)simd.lanes_eq(exit, maximum_value);
	clipped_allow_contacts := allow_contacts & ~invalid_interval;
	entry = simd.max(util.F32x8(0), entry);
	exit = simd.min(util.F32x8(1), exit);
	edge_start_b_dot_normal := util.vector3_wide_dot(edge_start_b, local_normal);
	edge_offset_b_dot_normal := util.vector3_wide_dot(edge_offset_b, local_normal);
	depth_contribution_b_entry := simd.add(
		edge_start_b_dot_normal, simd.mul(entry, edge_offset_b_dot_normal),
	);
	depth_contribution_b_exit := simd.add(
		edge_start_b_dot_normal, simd.mul(exit, edge_offset_b_dot_normal),
	);
	offset := util.vector3_wide_subtract(edge_start_b, triangle_center_b);
	offset_x := util.vector3_wide_dot(offset, tangent_bx);
	offset_y := util.vector3_wide_dot(offset, tangent_by);
	edge_direction_x := util.vector3_wide_dot(tangent_bx, edge_offset_b);
	edge_direction_y := util.vector3_wide_dot(tangent_by, edge_offset_b);
	candidate := Manifold_Candidate_Wide{
		depth=simd.sub(depth_contribution_b_entry, depth_contribution_a_entry),
		x=simd.add(simd.mul(entry, edge_direction_x), offset_x),
		y=simd.add(simd.mul(entry, edge_direction_y), offset_y),
		feature_id=entry_id,
	};
	entry_exists := clipped_allow_contacts &
		transmute(util.I32x8)simd.lanes_ge(candidate.depth, minimum_depth) &
		transmute(util.I32x8)simd.lanes_lt(candidate_count^, util.I32x8(6)) &
		transmute(util.I32x8)simd.lanes_ge(simd.sub(exit, entry), util.F32x8(1e-5)) &
		transmute(util.I32x8)simd.lanes_lt(entry, util.F32x8(1)) &
		transmute(util.I32x8)simd.lanes_gt(entry, util.F32x8(0));
	manifold_candidate_wide_add(candidates, candidate_count, candidate, entry_exists, pair_count);
	candidate.depth = simd.sub(depth_contribution_b_exit, depth_contribution_a_exit);
	candidate.x = simd.add(simd.mul(exit, edge_direction_x), offset_x);
	candidate.y = simd.add(simd.mul(exit, edge_direction_y), offset_y);
	candidate.feature_id = simd.add(entry_id, exit_id_offset);
	exit_exists := clipped_allow_contacts &
		transmute(util.I32x8)simd.lanes_ge(candidate.depth, minimum_depth) &
		transmute(util.I32x8)simd.lanes_lt(candidate_count^, util.I32x8(6)) &
		transmute(util.I32x8)simd.lanes_ge(exit, entry) &
		transmute(util.I32x8)simd.lanes_le(exit, util.F32x8(1)) &
		transmute(util.I32x8)simd.lanes_ge(exit, util.F32x8(0));
	manifold_candidate_wide_add(candidates, candidate_count, candidate, exit_exists, pair_count);
}

triangle_pair_test_wide :: proc "contextless" (
	a, b: Triangle_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
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
	vertices_a := [3]util.Vector3_Wide{a.a, a.b, a.c};
	vertices_b := [3]util.Vector3_Wide{
		util.vector3_wide_add(util.matrix3x3_wide_transform(b.a, r_b), local_offset_b),
		util.vector3_wide_add(util.matrix3x3_wide_transform(b.b, r_b), local_offset_b),
		util.vector3_wide_add(util.matrix3x3_wide_transform(b.c, r_b), local_offset_b),
	};
	local_triangle_center_a := util.vector3_wide_scale(
		util.vector3_wide_add(vertices_a[0], util.vector3_wide_add(vertices_a[1], vertices_a[2])),
		util.F32x8(1.0 / 3.0),
	);
	local_triangle_center_b := util.vector3_wide_scale(
		util.vector3_wide_add(vertices_b[0], util.vector3_wide_add(vertices_b[1], vertices_b[2])),
		util.F32x8(1.0 / 3.0),
	);
	edges_a := [3]util.Vector3_Wide{
		util.vector3_wide_subtract(vertices_a[1], vertices_a[0]),
		util.vector3_wide_subtract(vertices_a[2], vertices_a[1]),
		util.vector3_wide_subtract(vertices_a[0], vertices_a[2]),
	};
	edges_b := [3]util.Vector3_Wide{
		util.vector3_wide_subtract(vertices_b[1], vertices_b[0]),
		util.vector3_wide_subtract(vertices_b[2], vertices_b[1]),
		util.vector3_wide_subtract(vertices_b[0], vertices_b[2]),
	};
	depth := util.F32x8(3.402823466e+38);
	local_normal: util.Vector3_Wide;
	for edge_a in edges_a
	{
		for edge_b in edges_b
		{
			candidate_depth, candidate_normal := collision_triangle_pair_test_edge_wide(
				edge_a, edge_b, vertices_a, vertices_b,
			);
			collision_box_pair_select_axis_wide(
				&depth, &local_normal, candidate_depth, candidate_normal,
			);
		}
	}
	face_normal_a_unnormalized := util.vector3_wide_cross(edges_a[0], edges_a[2]);
	face_normal_a_length := util.vector3_wide_length(face_normal_a_unnormalized);
	face_normal_a := util.vector3_wide_scale(
		face_normal_a_unnormalized, simd.div(util.F32x8(1), face_normal_a_length),
	);
	candidate_depth := collision_triangle_pair_depth_wide(vertices_a, vertices_b, face_normal_a);
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, face_normal_a);
	face_normal_b_unnormalized := util.vector3_wide_cross(edges_b[0], edges_b[2]);
	face_normal_b_length := util.vector3_wide_length(face_normal_b_unnormalized);
	face_normal_b := util.vector3_wide_scale(
		face_normal_b_unnormalized, simd.div(util.F32x8(1), face_normal_b_length),
	);
	candidate_depth = collision_triangle_pair_depth_wide(vertices_a, vertices_b, face_normal_b);
	collision_box_pair_select_axis_wide(&depth, &local_normal, candidate_depth, face_normal_b);
	center_a_to_center_b := util.vector3_wide_subtract(local_triangle_center_b, local_triangle_center_a);
	local_normal = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(
		util.vector3_wide_dot(local_normal, center_a_to_center_b), util.F32x8(0),
	),
		local_normal,
	);
	minimum_depth := simd.neg(speculative_margin);
	local_normal_dot_face_normal_a := util.vector3_wide_dot(local_normal, face_normal_a);
	local_normal_dot_face_normal_b := util.vector3_wide_dot(local_normal, face_normal_b);
	ab_a_length_squared := util.vector3_wide_length_squared(edges_a[0]);
	ca_a_length_squared := util.vector3_wide_length_squared(edges_a[2]);
	ab_b_length_squared := util.vector3_wide_length_squared(edges_b[0]);
	ca_b_length_squared := util.vector3_wide_length_squared(edges_b[2]);
	epsilon_scale_a := simd.sqrt(simd.max(ab_a_length_squared, ca_a_length_squared));
	epsilon_scale_b := simd.sqrt(simd.max(ab_b_length_squared, ca_b_length_squared));
	nondegenerate_a := transmute(util.I32x8)simd.lanes_gt(
		face_normal_a_length, simd.mul(epsilon_scale_a, util.F32x8(1e-6)),
	);
	nondegenerate_b := transmute(util.I32x8)simd.lanes_gt(
		face_normal_b_length, simd.mul(epsilon_scale_b, util.F32x8(1e-6)),
	);
	allow_contacts := nondegenerate_a & nondegenerate_b &
		transmute(util.I32x8)simd.lanes_ge(depth, minimum_depth) & active &
		transmute(util.I32x8)simd.lanes_lt(
		local_normal_dot_face_normal_a, util.F32x8(-TRIANGLE_BACKFACE_REJECTION_THRESHOLD),
	) &
		transmute(util.I32x8)simd.lanes_gt(
		local_normal_dot_face_normal_b, util.F32x8(TRIANGLE_BACKFACE_REJECTION_THRESHOLD),
	);
	manifold: Convex_4_Contact_Manifold_Wide;
	if depth_refiner_all(~allow_contacts) == .Present
	{
		return manifold, .Ok;
	}
	flatten_x, flatten_y := collision_triangle_pair_basis_wide(local_normal);
	flat_vertices_a, flat_vertices_b: [3]util.Vector2_Wide;
	for vertex_index in 0 ..< 3
	{
		flat_vertices_a[vertex_index] = {
			x=util.vector3_wide_dot(vertices_a[vertex_index], flatten_x),
			y=util.vector3_wide_dot(vertices_a[vertex_index], flatten_y),
		};
		flat_vertices_b[vertex_index] = {
			x=util.vector3_wide_dot(vertices_b[vertex_index], flatten_x),
			y=util.vector3_wide_dot(vertices_b[vertex_index], flatten_y),
		};
	}
	flat_edges_a := [3]util.Vector2_Wide{
		util.vector2_wide_subtract(flat_vertices_a[1], flat_vertices_a[0]),
		util.vector2_wide_subtract(flat_vertices_a[2], flat_vertices_a[1]),
		util.vector2_wide_subtract(flat_vertices_a[0], flat_vertices_a[2]),
	};
	flat_edges_b := [3]util.Vector2_Wide{
		util.vector2_wide_subtract(flat_vertices_b[1], flat_vertices_b[0]),
		util.vector2_wide_subtract(flat_vertices_b[2], flat_vertices_b[1]),
		util.vector2_wide_subtract(flat_vertices_b[0], flat_vertices_b[2]),
	};
	use_face_case_b := allow_contacts & ~transmute(util.I32x8)simd.lanes_lt(
		simd.abs(local_normal_dot_face_normal_b), util.F32x8(0.2),
	);
	tangent_bx := util.vector3_wide_scale(
		edges_b[0], simd.div(util.F32x8(1), simd.sqrt(ab_b_length_squared)),
	);
	tangent_by := util.vector3_wide_cross(tangent_bx, face_normal_b);
	candidates: [8]Manifold_Candidate_Wide;
	candidate_count: util.I32x8;
	if depth_refiner_any(use_face_case_b) == .Present
	{
		inverse_contact_normal_dot_face_normal_b := simd.div(
			util.F32x8(1), local_normal_dot_face_normal_b,
		);
		for vertex_index in 0 ..< 3
		{
			collision_triangle_pair_try_add_a_vertex_wide(
				vertices_a[vertex_index], flat_vertices_a[vertex_index], util.I32x8(i32(vertex_index)),
				tangent_bx, tangent_by, local_triangle_center_b, local_normal, face_normal_b,
				flat_edges_b[0], flat_edges_b[1], flat_edges_b[2],
				flat_vertices_b[0], flat_vertices_b[1], use_face_case_b,
				inverse_contact_normal_dot_face_normal_b, minimum_depth,
				&candidates, &candidate_count, pair_count,
			);
		}
	}
	still_could_use_clipping_contacts := allow_contacts &
		transmute(util.I32x8)simd.lanes_lt(candidate_count, util.I32x8(3));
	if depth_refiner_any(still_could_use_clipping_contacts) == .Present
	{
		inverse_flat_edge_length_squared_a := [3]util.F32x8{
			simd.div(util.F32x8(1), util.vector2_wide_length_squared(flat_edges_a[0])),
			simd.div(util.F32x8(1), util.vector2_wide_length_squared(flat_edges_a[1])),
			simd.div(util.F32x8(1), util.vector2_wide_length_squared(flat_edges_a[2])),
		};
		vertex_dot_normal_a := [3]util.F32x8{
			util.vector3_wide_dot(local_normal, vertices_a[0]),
			util.vector3_wide_dot(local_normal, vertices_a[1]),
			util.vector3_wide_dot(local_normal, vertices_a[2]),
		};
		edge_dot_normal_a := [3]util.F32x8{
			util.vector3_wide_dot(local_normal, edges_a[0]),
			util.vector3_wide_dot(local_normal, edges_a[1]),
			util.vector3_wide_dot(local_normal, edges_a[2]),
		};
		for edge_index in 0 ..< 3
		{
			collision_triangle_pair_clip_b_edge_wide(
				flat_vertices_a, flat_edges_a, inverse_flat_edge_length_squared_a,
				vertex_dot_normal_a, edge_dot_normal_a,
				flat_vertices_b[edge_index], flat_edges_b[edge_index],
				vertices_b[edge_index], edges_b[edge_index],
				util.I32x8(i32(3 + edge_index)), util.I32x8(3),
				local_triangle_center_b, tangent_bx, tangent_by, local_normal, minimum_depth,
				still_could_use_clipping_contacts, &candidates, &candidate_count, pair_count,
			);
		}
	}
	epsilon_scale := simd.min(epsilon_scale_a, epsilon_scale_b);
	contacts, contact_exists := manifold_candidate_wide_reduce_internal(
		&candidates, candidate_count, 6, epsilon_scale, minimum_depth,
	);
	world_tangent_bx := util.matrix3x3_wide_transform(tangent_bx, world_r_a);
	world_tangent_by := util.matrix3x3_wide_transform(tangent_by, world_r_a);
	world_triangle_center := util.matrix3x3_wide_transform(local_triangle_center_b, world_r_a);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, world_r_a);
	manifold.contact_0_exists, manifold.contact_1_exists = contact_exists[0] & allow_contacts, contact_exists[1] & allow_contacts;
	manifold.contact_2_exists, manifold.contact_3_exists = contact_exists[2] & allow_contacts, contact_exists[3] & allow_contacts;
	manifold.offset_a_0 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_bx, contacts[0].x),
		util.vector3_wide_scale(world_tangent_by, contacts[0].y),
	),
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_bx, contacts[1].x),
		util.vector3_wide_scale(world_tangent_by, contacts[1].y),
	),
	);
	manifold.offset_a_2 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_bx, contacts[2].x),
		util.vector3_wide_scale(world_tangent_by, contacts[2].y),
	),
	);
	manifold.offset_a_3 = util.vector3_wide_add(
		world_triangle_center,
		util.vector3_wide_add(
		util.vector3_wide_scale(world_tangent_bx, contacts[3].x),
		util.vector3_wide_scale(world_tangent_by, contacts[3].y),
	),
	);
	manifold.depth_0, manifold.depth_1 = contacts[0].depth, contacts[1].depth;
	manifold.depth_2, manifold.depth_3 = contacts[2].depth, contacts[3].depth;
	manifold.feature_id_0, manifold.feature_id_1 = contacts[0].feature_id, contacts[1].feature_id;
	manifold.feature_id_2, manifold.feature_id_3 = contacts[2].feature_id, contacts[3].feature_id;
	face_flag := collision_wide_select_i32(
		transmute(util.I32x8)simd.lanes_ge(
		local_normal_dot_face_normal_b, util.F32x8(MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION),
	),
		util.I32x8(MESH_REDUCTION_FACE_COLLISION_FLAG), util.I32x8(0),
	);
	manifold.feature_id_0 = simd.add(manifold.feature_id_0, face_flag);
	return manifold, .Ok;
}
