// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

HULL_WIDE_CLIP :: #config(ENTASIS_HULL_WIDE_CLIP, true);

Hull_Clip_Candidate_Wide :: struct
{
	x, y: util.F32x8,
	feature_id, exists: util.I32x8,
}

// lanes are independent hull pairs. triangular and quadrilateral faces share
// the same ordered clipping operations. larger faces use the general kernel
collision_hull_pair_clip_small_faces_wide :: #force_no_inline proc "contextless" (
	hull_a, hull_b: ^Convex_Hull,
	offset_b: util.Vector3_Wide,
	b_local_r_a: util.Matrix3x3_Wide,
	local_offset_a, local_normal, face_normal_a_in_a, face_normal_b: util.Vector3_Wide,
	face_index_a, face_index_b: util.I32x8,
	epsilon_scale, depth_threshold: util.F32x8,
	world_r_b: util.Matrix3x3_Wide,
	active: util.I32x8,
	pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Reference_State, Physics_Status)
{
	vertices_a, vertices_b: [4]util.Vector3_Wide;
	previous_a, origin_b: util.Vector3_Wide;
	counts_a, counts_b: util.I32x8;
	exit_features, entry_features: [4]util.I32x8;
	point_indices_a, point_indices_b: [4][util.PRODUCTION_LANE_COUNT]i32;
	previous_indices_a, origin_indices_b: [util.PRODUCTION_LANE_COUNT]i32;
	single_bundle := hull_a.points.length == 1 && hull_b.points.length == 1;
	#unroll for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if lane < pair_count && simd.extract(active, lane) < 0
		{
			start_a, end_a, status_a := convex_hull_face_range_validated(hull_a, int(simd.extract(face_index_a, lane)));
			start_b, end_b, status_b := convex_hull_face_range_validated(hull_b, int(simd.extract(face_index_b, lane)));
			if status_a != .Ok
			{
				return {}, .Present, status_a;
			}
			if status_b != .Ok
			{
				return {}, .Present, status_b;
			}
			count_a := end_a - start_a;
			count_b := end_b - start_b;
			if count_a > 4 || count_b > 4
			{
				return {}, .Missing, .Ok;
			}
			counts_a = simd.replace(counts_a, lane, i32(count_a));
			counts_b = simd.replace(counts_b, lane, i32(count_b));
			previous_index_b := int(hull_b.face_vertex_indices.memory[end_b - 1]);
			previous_index_a := int(hull_a.face_vertex_indices.memory[end_a - 1]);
			if single_bundle
			{
				previous_indices_a[lane] = i32(previous_index_a);
				origin_indices_b[lane] = i32(previous_index_b);
			}
			else
			{
				util.vector3_wide_write_slot(&previous_a, lane, convex_hull_get_point(hull_a, previous_index_a));
				util.vector3_wide_write_slot(&origin_b, lane, convex_hull_get_point(hull_b, previous_index_b));
			}
			for vertex in 0 ..< 4
			{
				index_a := int(hull_a.face_vertex_indices.memory[start_a + min(vertex, count_a - 1)]);
				index_b := int(hull_b.face_vertex_indices.memory[start_b + min(vertex, count_b - 1)]);
				if single_bundle
				{
					point_indices_a[vertex][lane] = i32(index_a);
					point_indices_b[vertex][lane] = i32(index_b);
				}
				else
				{
					util.vector3_wide_write_slot(&vertices_a[vertex], lane, convex_hull_get_point(hull_a, index_a));
					util.vector3_wide_write_slot(&vertices_b[vertex], lane, convex_hull_get_point(hull_b, index_b));
					base_feature := (previous_index_b ~ index_b) << 8;
					exit_features[vertex] = simd.replace(exit_features[vertex], lane, i32(base_feature + index_b));
					entry_features[vertex] = simd.replace(entry_features[vertex], lane, i32(base_feature + previous_index_b));
				}
				previous_index_b = index_b;
			}
		}
	}

	if single_bundle
	{
		points_a := hull_a.points.memory[0];
		points_b := hull_b.points.memory[0];
		previous_a = hull_clip_select_point_bundle(points_a, transmute(util.I32x8)previous_indices_a);
		origin_b = hull_clip_select_point_bundle(points_b, transmute(util.I32x8)origin_indices_b);
		previous_index := transmute(util.I32x8)origin_indices_b;
		for vertex in 0 ..< 4
		{
			index_a := transmute(util.I32x8)point_indices_a[vertex];
			index_b := transmute(util.I32x8)point_indices_b[vertex];
			vertices_a[vertex] = hull_clip_select_point_bundle(points_a, index_a);
			vertices_b[vertex] = hull_clip_select_point_bundle(points_b, index_b);
			base_feature := simd.shl(previous_index ~ index_b, 8);
			exit_features[vertex] = simd.add(base_feature, index_b);
			entry_features[vertex] = simd.add(base_feature, previous_index);
			previous_index = index_b;
		}
	}
	face_normal_a := util.matrix3x3_wide_transform(face_normal_a_in_a, b_local_r_a);
	sign := util.wide_select_f32(transmute(util.I32x8)simd.lanes_lt(face_normal_b.z, util.F32x8(0)), util.F32x8(-1), util.F32x8(1));
	basis_scale := simd.div(util.F32x8(-1), simd.add(sign, face_normal_b.z));
	face_b_x := util.Vector3_Wide{
		x=simd.mul(simd.mul(face_normal_b.x, face_normal_b.y), basis_scale),
		y=simd.add(sign, simd.mul(simd.mul(face_normal_b.y, face_normal_b.y), basis_scale)),
		z=simd.neg(face_normal_b.y),
	};
	face_b_y := util.Vector3_Wide{
		x=simd.add(util.F32x8(1), simd.mul(simd.mul(simd.mul(sign, face_normal_b.x), face_normal_b.x), basis_scale)),
		y=simd.mul(sign, face_b_x.x),
		z=simd.neg(simd.mul(sign, face_normal_b.x)),
	};
	previous_a = util.vector3_wide_add(util.matrix3x3_wide_transform(previous_a, b_local_r_a), local_offset_a);
	edge_normals_a: [4]util.Vector3_Wide;
	maximum_containment: [4]util.F32x8;
	for vertex in 0 ..< 4
	{
		vertices_a[vertex] = util.vector3_wide_add(util.matrix3x3_wide_transform(vertices_a[vertex], b_local_r_a), local_offset_a);
		edge_normals_a[vertex] = util.vector3_wide_cross(local_normal, util.vector3_wide_subtract(vertices_a[vertex], previous_a));
		maximum_containment[vertex] = util.F32x8(-f32(math.F32_MAX));
		previous_a = vertices_a[vertex];
	}
	candidates: [12]Hull_Clip_Candidate_Wide;
	previous_b := origin_b;
	for vertex_b in 0 ..< 4
	{
		exists_b := active & transmute(util.I32x8)simd.lanes_gt(counts_b, util.I32x8(i32(vertex_b)));
		edge_offset_b := util.vector3_wide_subtract(vertices_b[vertex_b], previous_b);
		edge_normal_b := util.vector3_wide_cross(edge_offset_b, local_normal);
		latest_entry := util.F32x8(-f32(math.F32_MAX));
		earliest_exit := util.F32x8(f32(math.F32_MAX));
		for vertex_a in 0 ..< 4
		{
			exists := exists_b & transmute(util.I32x8)simd.lanes_gt(counts_a, util.I32x8(i32(vertex_a)));
			edge_b_to_a := util.vector3_wide_subtract(vertices_a[vertex_a], previous_b);
			containment := util.vector3_wide_dot(edge_b_to_a, edge_normal_b);
			maximum_containment[vertex_a] = util.wide_select_f32(exists,
				simd.max(maximum_containment[vertex_a], containment), maximum_containment[vertex_a]);
			numerator := util.vector3_wide_dot(edge_b_to_a, edge_normals_a[vertex_a]);
			denominator := util.vector3_wide_dot(edge_normals_a[vertex_a], edge_offset_b);
			use_entry := exists & transmute(util.I32x8)simd.lanes_lt(denominator, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(numerator, simd.mul(latest_entry, denominator));
			use_exit := exists & transmute(util.I32x8)simd.lanes_gt(denominator, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(numerator, simd.mul(earliest_exit, denominator));
			if simd.reduce_or(use_entry | use_exit) != 0
			{
				distance := simd.div(numerator, denominator);
				latest_entry = util.wide_select_f32(use_entry, distance, latest_entry);
				earliest_exit = util.wide_select_f32(use_exit, distance, earliest_exit);
			}
			outside := exists & transmute(util.I32x8)simd.lanes_eq(denominator, util.F32x8(0)) &
				transmute(util.I32x8)simd.lanes_lt(numerator, util.F32x8(0));
			latest_entry = util.wide_select_f32(outside, util.F32x8(f32(math.F32_MAX)), latest_entry);
			earliest_exit = util.wide_select_f32(outside, util.F32x8(-f32(math.F32_MAX)), earliest_exit);
		}
		valid_interval := exists_b & transmute(util.I32x8)simd.lanes_le(latest_entry, earliest_exit);
		latest_entry = simd.max(util.F32x8(0), latest_entry);
		earliest_exit = simd.min(util.F32x8(1), earliest_exit);
		exit_point := util.vector3_wide_subtract(util.vector3_wide_add(previous_b,
			util.vector3_wide_scale(edge_offset_b, earliest_exit)), origin_b);
		entry_point := util.vector3_wide_subtract(util.vector3_wide_add(previous_b,
			util.vector3_wide_scale(edge_offset_b, latest_entry)), origin_b);
		candidates[vertex_b * 2] = {
			x=util.vector3_wide_dot(exit_point, face_b_x), y=util.vector3_wide_dot(exit_point, face_b_y),
			feature_id=exit_features[vertex_b],
			exists=valid_interval & transmute(util.I32x8)simd.lanes_ge(earliest_exit, latest_entry),
		};
		candidates[vertex_b * 2 + 1] = {
			x=util.vector3_wide_dot(entry_point, face_b_x), y=util.vector3_wide_dot(entry_point, face_b_y),
			feature_id=entry_features[vertex_b],
			exists=valid_interval & transmute(util.I32x8)simd.lanes_lt(latest_entry, earliest_exit) &
				transmute(util.I32x8)simd.lanes_gt(latest_entry, util.F32x8(0)),
		};
		previous_b = vertices_b[vertex_b];
	}
	inverse_normal_dot_b := simd.div(util.F32x8(1), util.vector3_wide_dot(local_normal, face_normal_b));
	for vertex_a in 0 ..< 4
	{
		exists := active & transmute(util.I32x8)simd.lanes_gt(counts_a, util.I32x8(i32(vertex_a))) &
			transmute(util.I32x8)simd.lanes_le(maximum_containment[vertex_a], util.F32x8(0));
		face_to_a := util.vector3_wide_subtract(vertices_a[vertex_a], origin_b);
		distance := simd.mul(util.vector3_wide_dot(face_to_a, face_normal_b), inverse_normal_dot_b);
		projected := util.vector3_wide_subtract(face_to_a, util.vector3_wide_scale(local_normal, distance));
		candidates[8 + vertex_a] = {
			x=util.vector3_wide_dot(face_b_x, projected), y=util.vector3_wide_dot(face_b_y, projected),
			feature_id=util.I32x8(i32(vertex_a)), exists=exists,
		};
	}
	world_normal := util.matrix3x3_wide_transform(local_normal, world_r_b);
	inverse_normal_dot_a := simd.div(util.F32x8(1), util.vector3_wide_dot(face_normal_a, local_normal));
	maximum_count := simd.max(simd.max(counts_a, counts_b),
		simd.min(simd.add(counts_a, counts_a), simd.add(counts_b, counts_b)));
	return manifold_candidate_reduce_small_wide(&candidates, maximum_count, active,
		face_normal_a, inverse_normal_dot_a, vertices_a[0], origin_b, face_b_x, face_b_y,
		epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal), .Present, .Ok;
}

// the production backend requires AVX2. this keeps point selection in registers
@(private, default_calling_convention="none")
foreign _
{
	@(link_name="llvm.x86.avx2.permps")
	hull_clip_permute_points :: proc(points: util.F32x8, indices: util.I32x8) -> util.F32x8 ---;
}

hull_clip_select_point_bundle :: #force_inline proc "contextless" (
	points: util.Vector3_Wide, indices: util.I32x8,
) -> util.Vector3_Wide
{
	return {
		x=hull_clip_permute_points(points.x, indices),
		y=hull_clip_permute_points(points.y, indices),
		z=hull_clip_permute_points(points.z, indices),
	};
}
