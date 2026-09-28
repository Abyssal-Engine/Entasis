// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Hull_Pair_Depth_Context_Wide :: struct
{
	hull_a, hull_b:              [util.PRODUCTION_LANE_COUNT]^Convex_Hull,
	shared_hull_a, shared_hull_b: ^Convex_Hull,
	local_offset_a:              util.Vector3_Wide,
	b_local_r_a:                 util.Matrix3x3_Wide,
}

hull_pair_support_shared_bundle_candidate :: #force_inline proc "contextless" (
	points, direction: util.Vector3_Wide,
	$point_lane: int,
	best_index: ^util.I32x8,
	best_dot: ^util.F32x8,
)
{
	candidate := util.Vector3_Wide{
		x=util.F32x8(simd.extract(points.x, point_lane)),
		y=util.F32x8(simd.extract(points.y, point_lane)),
		z=util.F32x8(simd.extract(points.z, point_lane)),
	};
	candidate_dot := util.vector3_wide_dot(candidate, direction);
	use_candidate := transmute(util.I32x8)simd.lanes_gt(candidate_dot, best_dot^);
	best_index^ = collision_wide_select_i32(use_candidate, util.I32x8(point_lane), best_index^);
	best_dot^ = util.wide_select_f32(use_candidate, candidate_dot, best_dot^);
}

hull_pair_support_shared_bundle_wide :: #force_inline proc "contextless" (
	hull: ^Convex_Hull,
	direction: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	points := hull.points.memory[0];
	first := util.Vector3_Wide{
		x=util.F32x8(simd.extract(points.x, 0)),
		y=util.F32x8(simd.extract(points.y, 0)),
		z=util.F32x8(simd.extract(points.z, 0)),
	};
	best_index: util.I32x8;
	best_dot := util.vector3_wide_dot(first, direction);
	hull_pair_support_shared_bundle_candidate(points, direction, 1, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 2, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 3, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 4, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 5, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 6, &best_index, &best_dot);
	hull_pair_support_shared_bundle_candidate(points, direction, 7, &best_index, &best_dot);
	return hull_clip_select_point_bundle(points, best_index);
}

hull_pair_face_shared_bundle_candidate :: #force_inline proc "contextless" (
	planes: ^Hull_Bounding_Plane_Bundle,
	local_normal, closest_on_hull: util.Vector3_Wide,
	bounding_plane_epsilon: util.F32x8,
	$face_lane: int,
	best_dot, best_error: ^util.F32x8,
	best_index: ^util.I32x8,
)
{
	candidate_normal := util.Vector3_Wide{
		x=util.F32x8(simd.extract(planes.normal.x, face_lane)),
		y=util.F32x8(simd.extract(planes.normal.y, face_lane)),
		z=util.F32x8(simd.extract(planes.normal.z, face_lane)),
	};
	candidate_offset := util.F32x8(simd.extract(planes.offset, face_lane));
	candidate_dot := util.vector3_wide_dot(candidate_normal, local_normal);
	candidate_error := simd.abs(simd.sub(
		util.vector3_wide_dot(candidate_normal, closest_on_hull),
		candidate_offset,
	));
	error_improvement := simd.sub(best_error^, candidate_error);
	use_candidate := transmute(util.I32x8)simd.lanes_ge(error_improvement, bounding_plane_epsilon) |
		(transmute(util.I32x8)simd.lanes_gt(error_improvement, simd.neg(bounding_plane_epsilon)) &
		transmute(util.I32x8)simd.lanes_gt(candidate_dot, best_dot^));
	best_dot^ = util.wide_select_f32(use_candidate, candidate_dot, best_dot^);
	best_error^ = util.wide_select_f32(use_candidate, candidate_error, best_error^);
	best_index^ = collision_wide_select_i32(use_candidate, util.I32x8(face_lane), best_index^);
}

hull_pair_pick_representative_face_shared_bundle_wide :: #force_inline proc "contextless" (
	hull: ^Convex_Hull,
	local_normal, closest_on_hull: util.Vector3_Wide,
	bounding_plane_epsilon: util.F32x8,
) -> (face_normal: util.Vector3_Wide, face_index: util.I32x8)
{
	planes := &hull.bounding_planes.memory[0];
	face_normal = {
		x=util.F32x8(simd.extract(planes.normal.x, 0)),
		y=util.F32x8(simd.extract(planes.normal.y, 0)),
		z=util.F32x8(simd.extract(planes.normal.z, 0)),
	};
	best_offset := util.F32x8(simd.extract(planes.offset, 0));
	best_dot := util.vector3_wide_dot(face_normal, local_normal);
	best_error := simd.abs(simd.sub(
		util.vector3_wide_dot(face_normal, closest_on_hull),
		best_offset,
	));
	face_count := int(hull.face_start_indices.length);
	if face_count > 1
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			1, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 2
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			2, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 3
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			3, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 4
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			4, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 5
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			5, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 6
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			6, &best_dot, &best_error, &face_index,
		);
	}
	if face_count > 7
	{
		hull_pair_face_shared_bundle_candidate(
			planes, local_normal, closest_on_hull, bounding_plane_epsilon,
			7, &best_dot, &best_error, &face_index,
		);
	}
	face_normal = hull_clip_select_point_bundle(planes.normal, face_index);
	return;
}

hull_pair_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Hull_Pair_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	active_lanes := ~terminated_lanes;
	invalid_direction := active_lanes & transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_length_squared(direction), util.F32x8(1e-20),
	);
	if transmute(u8)simd.extract_msbs(invalid_direction) != 0
	{
		return {}, {}, .Invalid_Argument;
	}
	direction_a := util.matrix3x3_wide_transform_transposed(
		util.vector3_wide_negate(direction), depth_context.b_local_r_a,
	);
	support_a_local, support_b: util.Vector3_Wide;
	if depth_context.shared_hull_a != nil
	{
		support_a_local = hull_pair_support_shared_bundle_wide(depth_context.shared_hull_a, direction_a);
	}
	if depth_context.shared_hull_b != nil
	{
		support_b = hull_pair_support_shared_bundle_wide(depth_context.shared_hull_b, direction);
	}
	if depth_context.shared_hull_a == nil || depth_context.shared_hull_b == nil
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if simd.extract(terminated_lanes, lane) < 0
			{
				continue;
			}
			if depth_context.shared_hull_a == nil
			{
				local := convex_hull_support_trusted(
					depth_context.hull_a[lane], util.vector3_wide_read_slot(direction_a, lane),
				);
				util.vector3_wide_write_slot(&support_a_local, lane, local);
			}
			if depth_context.shared_hull_b == nil
			{
				local := convex_hull_support_trusted(
					depth_context.hull_b[lane], util.vector3_wide_read_slot(direction, lane),
				);
				util.vector3_wide_write_slot(&support_b, lane, local);
			}
		}
	}
	support_a := util.vector3_wide_add(
		util.matrix3x3_wide_transform(support_a_local, depth_context.b_local_r_a),
		depth_context.local_offset_a,
	);
	support_on_a = util.vector3_wide_select(active_lanes, support_b, {});
	support = util.vector3_wide_select(
		active_lanes, util.vector3_wide_subtract(support_b, support_a), {},
	);
	return support, support_on_a, .Ok;
}

Hull_Pair_Cached_Edge :: struct
{
	vertex:                  util.Vector3,
	edge_plane_normal:       util.Vector3,
	maximum_containment_dot: f32,
}

convex_hull_pair_test_source :: proc "contextless" (
	hull_a, hull_b: ^Convex_Hull, pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if convex_hull_validate(hull_a) != .Ok || convex_hull_validate(hull_b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	b_local_r_a := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	local_offset_a := util.vector3_negate(local_offset_b);
	center_distance := util.vector3_length(local_offset_a);
	initial_normal := util.Vector3{0, 1, 0};
	if center_distance >= 1e-8
	{
		initial_normal = util.vector3_scale(local_offset_a, 1 / center_distance);
	}

	depth_context: Hull_Pair_Depth_Context_Wide;
	depth_context.hull_a[0] = hull_a;
	depth_context.hull_b[0] = hull_b;
	util.vector3_wide_write_slot(&depth_context.local_offset_a, 0, local_offset_a);
	util.vector3_wide_write_slot(&depth_context.b_local_r_a.x, 0, b_local_r_a.x);
	util.vector3_wide_write_slot(&depth_context.b_local_r_a.y, 0, b_local_r_a.y);
	util.vector3_wide_write_slot(&depth_context.b_local_r_a.z, 0, b_local_r_a.z);
	initial_normal_wide: util.Vector3_Wide;
	util.vector3_wide_write_slot(&initial_normal_wide, 0, initial_normal);
	inactive_lanes := util.I32x8(-1);
	inactive_lanes = simd.replace(inactive_lanes, 0, 0);
	a_point := convex_hull_get_point(hull_a, 0);
	b_point := convex_hull_get_point(hull_b, 0);
	epsilon_scale := min(
		(abs(a_point.x) + abs(a_point.y) + abs(a_point.z)) * (1.0 / 3.0),
		(abs(b_point.x) + abs(b_point.y) + abs(b_point.z)) * (1.0 / 3.0),
	);
	depth_threshold := -speculative_margin;
	depth_wide, local_normal_wide, closest_on_b_wide, depth_status := depth_refiner_find_minimum_depth_wide(
		hull_pair_depth_support_wide, &depth_context, initial_normal_wide, inactive_lanes,
		util.F32x8(1e-5 * epsilon_scale), util.F32x8(depth_threshold), {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	depth := simd.extract(depth_wide, 0);
	if depth < depth_threshold
	{
		return {offset_b=offset_b}, .Ok;
	}
	local_normal := util.vector3_wide_read_slot(local_normal_wide, 0);
	closest_on_b := util.vector3_wide_read_slot(closest_on_b_wide, 0);
	local_normal_in_a := util.matrix3x3_transform_transpose(local_normal, b_local_r_a);
	closest_on_a := util.vector3_subtract(closest_on_b, util.vector3_scale(local_normal, depth));
	closest_on_a_in_a := util.matrix3x3_transform_transpose(
		util.vector3_subtract(closest_on_a, local_offset_a), b_local_r_a,
	);
	bounding_plane_epsilon := 1e-3 * epsilon_scale;
	face_normal_a_in_a, face_index_a, face_a_status := convex_hull_pick_representative_face(
		hull_a, util.vector3_negate(local_normal_in_a), closest_on_a_in_a, bounding_plane_epsilon,
	);
	if face_a_status != .Ok
	{
		return {}, face_a_status;
	}
	face_normal_a := util.matrix3x3_transform(face_normal_a_in_a, b_local_r_a);
	face_normal_b, face_index_b, face_b_status := convex_hull_pick_representative_face(
		hull_b, local_normal, closest_on_b, bounding_plane_epsilon,
	);
	if face_b_status != .Ok
	{
		return {}, face_b_status;
	}
	face_b_x, face_b_y := collision_build_orthonormal_basis(face_normal_b);
	start_a, end_a, range_a_status := convex_hull_face_range(hull_a, face_index_a);
	start_b, end_b, range_b_status := convex_hull_face_range(hull_b, face_index_b);
	if range_a_status != .Ok
	{
		return {}, range_a_status;
	}
	if range_b_status != .Ok
	{
		return {}, range_b_status;
	}
	count_a := end_a - start_a;
	count_b := end_b - start_b;
	maximum_candidate_count := max(max(count_a, count_b), min(count_a * 2, count_b * 2));
	if count_a > MAXIMUM_HULL_FACE_VERTEX_COUNT || maximum_candidate_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}

	cached_edges: [MAXIMUM_HULL_FACE_VERTEX_COUNT]Hull_Pair_Cached_Edge = ---;
	previous_point_index_a := int(hull_a.face_vertex_indices.memory[end_a - 1]);
	previous_vertex_a := util.vector3_add(
		util.matrix3x3_transform(convex_hull_get_point(hull_a, previous_point_index_a), b_local_r_a), local_offset_a,
	);
	for face_vertex_a in 0 ..< count_a
	{
		point_index_a := int(hull_a.face_vertex_indices.memory[start_a + face_vertex_a]);
		vertex_a := util.vector3_add(
			util.matrix3x3_transform(convex_hull_get_point(hull_a, point_index_a), b_local_r_a), local_offset_a,
		);
		cached_edges[face_vertex_a] = {
			vertex=vertex_a,
			edge_plane_normal=util.vector3_cross(local_normal, util.vector3_subtract(vertex_a, previous_vertex_a)),
			maximum_containment_dot=-f32(math.F32_MAX),
		};
		previous_vertex_a = vertex_a;
	}

	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar = ---;
	candidate_count := 0;
	previous_point_index_b := int(hull_b.face_vertex_indices.memory[end_b - 1]);
	face_origin_b := convex_hull_get_point(hull_b, previous_point_index_b);
	previous_vertex_b := face_origin_b;
	for face_vertex_b in 0 ..< count_b
	{
		point_index_b := int(hull_b.face_vertex_indices.memory[start_b + face_vertex_b]);
		vertex_b := convex_hull_get_point(hull_b, point_index_b);
		edge_offset_b := util.vector3_subtract(vertex_b, previous_vertex_b);
		edge_plane_normal_b := util.vector3_cross(edge_offset_b, local_normal);
		latest_entry := -f32(math.F32_MAX);
		earliest_exit := f32(math.F32_MAX);
		for face_vertex_a in 0 ..< count_a
		{
			edge_a := &cached_edges[face_vertex_a];
			edge_b_to_edge_a := util.vector3_subtract(edge_a.vertex, previous_vertex_b);
			containment_dot := util.vector3_dot(edge_b_to_edge_a, edge_plane_normal_b);
			edge_a.maximum_containment_dot = max(edge_a.maximum_containment_dot, containment_dot);
			numerator := util.vector3_dot(edge_b_to_edge_a, edge_a.edge_plane_normal);
			denominator := util.vector3_dot(edge_a.edge_plane_normal, edge_offset_b);
			if denominator < 0
			{
				if numerator < latest_entry * denominator
				{
					latest_entry = numerator / denominator;
				}
			}
			else if denominator > 0
			{
				if numerator < earliest_exit * denominator
				{
					earliest_exit = numerator / denominator;
				}
			}
			else if numerator < 0
			{
				earliest_exit = -f32(math.F32_MAX);
				latest_entry = f32(math.F32_MAX);
			}
		}
		if latest_entry <= earliest_exit
		{
			latest_entry = max(f32(0), latest_entry);
			earliest_exit = min(f32(1), earliest_exit);
			base_feature_id := (previous_point_index_b ~ point_index_b) << 8;
			if earliest_exit >= latest_entry && candidate_count < maximum_candidate_count
			{
				point := util.vector3_subtract(
					util.vector3_add(
					previous_vertex_b,
					util.vector3_scale(edge_offset_b, earliest_exit)
				), face_origin_b,
				);
				candidates[candidate_count] = {
					x=util.vector3_dot(point, face_b_x), y=util.vector3_dot(point, face_b_y),
					feature_id=i32(base_feature_id + point_index_b),
				};
				candidate_count += 1;
			}
			if latest_entry < earliest_exit && latest_entry > 0 && candidate_count < maximum_candidate_count
			{
				point := util.vector3_subtract(
					util.vector3_add(previous_vertex_b, util.vector3_scale(edge_offset_b, latest_entry)), face_origin_b,
				);
				candidates[candidate_count] = {
					x=util.vector3_dot(point, face_b_x), y=util.vector3_dot(point, face_b_y),
					feature_id=i32(base_feature_id + previous_point_index_b),
				};
				candidate_count += 1;
			}
		}
		previous_point_index_b = point_index_b;
		previous_vertex_b = vertex_b;
	}

	inverse_normal_dot_face_b := 1 / util.vector3_dot(local_normal, face_normal_b);
	for face_vertex_a in 0 ..< count_a
	{
		edge := cached_edges[face_vertex_a];
		if edge.maximum_containment_dot <= 0 && candidate_count < maximum_candidate_count
		{
			b_face_to_vertex_a := util.vector3_subtract(edge.vertex, face_origin_b);
			distance_to_b := util.vector3_dot(b_face_to_vertex_a, face_normal_b) * inverse_normal_dot_face_b;
			projected_vertex_a := util.vector3_subtract(
				b_face_to_vertex_a,
				util.vector3_scale(local_normal, distance_to_b)
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(face_b_x, projected_vertex_a),
				y=util.vector3_dot(face_b_y, projected_vertex_a),
				feature_id=i32(face_vertex_a),
			};
			candidate_count += 1;
		}
	}
	world_normal := util.matrix3x3_transform(local_normal, world_r_b);
	return manifold_candidate_reduce(
		&candidates, candidate_count, face_normal_a,
		1 / util.vector3_dot(face_normal_a, local_normal),
		cached_edges[0].vertex, face_origin_b, face_b_x, face_b_y,
		epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
	);
}

collision_hull_pair_build_manifold :: proc "contextless" (
	hull_a, hull_b: ^Convex_Hull,
	offset_b: util.Vector3,
	b_local_r_a: util.Matrix3x3,
	local_offset_a, local_normal, face_normal_a_in_a, face_normal_b: util.Vector3,
	face_index_a, face_index_b: int,
	epsilon_scale, depth_threshold: f32,
	world_r_b: util.Matrix3x3,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	face_normal_a := util.matrix3x3_transform(face_normal_a_in_a, b_local_r_a);
	face_b_x, face_b_y := collision_build_orthonormal_basis(face_normal_b);
	start_a, end_a, range_a_status := convex_hull_face_range_validated(hull_a, face_index_a);
	start_b, end_b, range_b_status := convex_hull_face_range_validated(hull_b, face_index_b);
	if range_a_status != .Ok
	{
		return {}, range_a_status;
	}
	if range_b_status != .Ok
	{
		return {}, range_b_status;
	}
	count_a := end_a - start_a;
	count_b := end_b - start_b;
	maximum_candidate_count := max(max(count_a, count_b), min(count_a * 2, count_b * 2));
	if count_a > MAXIMUM_HULL_FACE_VERTEX_COUNT || maximum_candidate_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}
	cached_edges: [MAXIMUM_HULL_FACE_VERTEX_COUNT]Hull_Pair_Cached_Edge = ---;
	previous_point_index_a := int(hull_a.face_vertex_indices.memory[end_a - 1]);
	previous_vertex_a := util.vector3_add(
		util.matrix3x3_transform(convex_hull_get_point(hull_a, previous_point_index_a), b_local_r_a), local_offset_a,
	);
	for face_vertex_a in 0 ..< count_a
	{
		point_index_a := int(hull_a.face_vertex_indices.memory[start_a + face_vertex_a]);
		vertex_a := util.vector3_add(
			util.matrix3x3_transform(convex_hull_get_point(hull_a, point_index_a), b_local_r_a), local_offset_a,
		);
		cached_edges[face_vertex_a] = {
			vertex=vertex_a,
			edge_plane_normal=util.vector3_cross(local_normal, util.vector3_subtract(vertex_a, previous_vertex_a)),
			maximum_containment_dot=-f32(math.F32_MAX),
		};
		previous_vertex_a = vertex_a;
	}
	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar = ---;
	candidate_count := 0;
	previous_point_index_b := int(hull_b.face_vertex_indices.memory[end_b - 1]);
	face_origin_b := convex_hull_get_point(hull_b, previous_point_index_b);
	previous_vertex_b := face_origin_b;
	for face_vertex_b in 0 ..< count_b
	{
		point_index_b := int(hull_b.face_vertex_indices.memory[start_b + face_vertex_b]);
		vertex_b := convex_hull_get_point(hull_b, point_index_b);
		edge_offset_b := util.vector3_subtract(vertex_b, previous_vertex_b);
		edge_plane_normal_b := util.vector3_cross(edge_offset_b, local_normal);
		latest_entry := -f32(math.F32_MAX);
		earliest_exit := f32(math.F32_MAX);
		for face_vertex_a in 0 ..< count_a
		{
			edge_a := &cached_edges[face_vertex_a];
			edge_b_to_edge_a := util.vector3_subtract(edge_a.vertex, previous_vertex_b);
			containment_dot := util.vector3_dot(edge_b_to_edge_a, edge_plane_normal_b);
			edge_a.maximum_containment_dot = max(edge_a.maximum_containment_dot, containment_dot);
			numerator := util.vector3_dot(edge_b_to_edge_a, edge_a.edge_plane_normal);
			denominator := util.vector3_dot(edge_a.edge_plane_normal, edge_offset_b);
			if denominator < 0
			{
				if numerator < latest_entry * denominator
				{
					latest_entry = numerator / denominator;
				}
			}
			else if denominator > 0
			{
				if numerator < earliest_exit * denominator
				{
					earliest_exit = numerator / denominator;
				}
			}
			else if numerator < 0
			{
				earliest_exit = -f32(math.F32_MAX);
				latest_entry = f32(math.F32_MAX);
			}
		}
		if latest_entry <= earliest_exit
		{
			latest_entry = max(f32(0), latest_entry);
			earliest_exit = min(f32(1), earliest_exit);
			base_feature_id := (previous_point_index_b ~ point_index_b) << 8;
			if earliest_exit >= latest_entry && candidate_count < maximum_candidate_count
			{
				point := util.vector3_subtract(
					util.vector3_add(
						previous_vertex_b,
						util.vector3_scale(edge_offset_b, earliest_exit),
					),
					face_origin_b,
				);
				candidates[candidate_count] = {
					x=util.vector3_dot(point, face_b_x),
					y=util.vector3_dot(point, face_b_y),
					feature_id=i32(base_feature_id + point_index_b),
				};
				candidate_count += 1;
			}
			if latest_entry < earliest_exit && latest_entry > 0 && candidate_count < maximum_candidate_count
			{
				point := util.vector3_subtract(
					util.vector3_add(previous_vertex_b, util.vector3_scale(edge_offset_b, latest_entry)),
					face_origin_b,
				);
				candidates[candidate_count] = {
					x=util.vector3_dot(point, face_b_x),
					y=util.vector3_dot(point, face_b_y),
					feature_id=i32(base_feature_id + previous_point_index_b),
				};
				candidate_count += 1;
			}
		}
		previous_point_index_b = point_index_b;
		previous_vertex_b = vertex_b;
	}
	inverse_normal_dot_face_b := 1 / util.vector3_dot(local_normal, face_normal_b);
	for face_vertex_a in 0 ..< count_a
	{
		edge := cached_edges[face_vertex_a];
		if edge.maximum_containment_dot <= 0 && candidate_count < maximum_candidate_count
		{
			b_face_to_vertex_a := util.vector3_subtract(edge.vertex, face_origin_b);
			distance_to_b := util.vector3_dot(b_face_to_vertex_a, face_normal_b) * inverse_normal_dot_face_b;
			projected_vertex_a := util.vector3_subtract(
				b_face_to_vertex_a,
				util.vector3_scale(local_normal, distance_to_b),
			);
			candidates[candidate_count] = {
				x=util.vector3_dot(face_b_x, projected_vertex_a),
				y=util.vector3_dot(face_b_y, projected_vertex_a),
				feature_id=i32(face_vertex_a),
			};
			candidate_count += 1;
		}
	}
	world_normal := util.matrix3x3_transform(local_normal, world_r_b);
	return manifold_candidate_reduce(
		&candidates, candidate_count, face_normal_a,
		1 / util.vector3_dot(face_normal_a, local_normal),
		cached_edges[0].vertex, face_origin_b, face_b_x, face_b_y,
		epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
	);
}

convex_hull_pair_test_wide :: proc "contextless" (
	hull_a, hull_b: Convex_Hull_Wide,
	speculative_margin: util.F32x8,
	offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide,
	pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	inactive := ~active;
	world_r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	world_r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	b_local_r_a := util.matrix3x3_wide_multiply_by_transpose(world_r_a, world_r_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_b);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	center_distance := util.vector3_wide_length(local_offset_a);
	initial_normal := util.vector3_wide_scale(local_offset_a, simd.div(util.F32x8(1), center_distance));
	initial_normal = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(center_distance, util.F32x8(1e-8)),
		{y=util.F32x8(1)},
		initial_normal,
	);
	depth_context := Hull_Pair_Depth_Context_Wide{
		local_offset_a=local_offset_a,
		b_local_r_a=b_local_r_a,
	};
	epsilon_scale: util.F32x8;
	for lane in 0 ..< pair_count
	{
		if convex_hull_validate(hull_a.hulls[lane]) != .Ok || convex_hull_validate(hull_b.hulls[lane]) != .Ok
		{
			return {}, .Invalid_Description;
		}
		depth_context.hull_a[lane] = hull_a.hulls[lane];
		depth_context.hull_b[lane] = hull_b.hulls[lane];
		if lane == 0
		{
			depth_context.shared_hull_a = hull_a.hulls[lane];
			depth_context.shared_hull_b = hull_b.hulls[lane];
		}
		else
		{
			if hull_a.hulls[lane] != depth_context.shared_hull_a
			{
				depth_context.shared_hull_a = nil;
			}
			if hull_b.hulls[lane] != depth_context.shared_hull_b
			{
				depth_context.shared_hull_b = nil;
			}
		}
		a_point := convex_hull_get_point(hull_a.hulls[lane], 0);
		b_point := convex_hull_get_point(hull_b.hulls[lane], 0);
		epsilon_scale = simd.replace(epsilon_scale, lane, min(
			(abs(a_point.x) + abs(a_point.y) + abs(a_point.z)) * (1.0 / 3.0),
			(abs(b_point.x) + abs(b_point.y) + abs(b_point.z)) * (1.0 / 3.0),
		));
	}
	shared_clip_hull_a := depth_context.shared_hull_a;
	shared_clip_hull_b := depth_context.shared_hull_b;
	shared_face_hull_a := depth_context.shared_hull_a;
	shared_face_hull_b := depth_context.shared_hull_b;
	if pair_count < 4 || shared_face_hull_a == nil ||
		shared_face_hull_a.bounding_planes.length != 1 ||
		shared_face_hull_a.face_start_indices.length > util.PRODUCTION_LANE_COUNT
	{
		shared_face_hull_a = nil;
	}
	if pair_count < 4 || shared_face_hull_b == nil ||
		shared_face_hull_b.bounding_planes.length != 1 ||
		shared_face_hull_b.face_start_indices.length > util.PRODUCTION_LANE_COUNT
	{
		shared_face_hull_b = nil;
	}
	if pair_count < 4 || depth_context.shared_hull_a == nil || depth_context.shared_hull_a.points.length != 1
	{
		depth_context.shared_hull_a = nil;
	}
	if pair_count < 4 || depth_context.shared_hull_b == nil || depth_context.shared_hull_b.points.length != 1
	{
		depth_context.shared_hull_b = nil;
	}
	depth_threshold := simd.neg(speculative_margin);
	depth, local_normal, closest_on_b, depth_status := depth_refiner_find_minimum_depth_wide(
		hull_pair_depth_support_wide,
		&depth_context,
		initial_normal,
		inactive,
		simd.mul(util.F32x8(1e-5), epsilon_scale),
		depth_threshold,
		{},
		{},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	inactive |= transmute(util.I32x8)simd.lanes_lt(depth, depth_threshold);
	if simd.reduce_and(inactive) == -1
	{
		return {}, .Ok;
	}

	local_normal_in_a := util.matrix3x3_wide_transform_transposed(local_normal, b_local_r_a);
	closest_on_a := util.vector3_wide_subtract(
		closest_on_b,
		util.vector3_wide_scale(local_normal, depth),
	);
	closest_on_a_in_a := util.matrix3x3_wide_transform_transposed(
		util.vector3_wide_subtract(closest_on_a, local_offset_a),
		b_local_r_a,
	);
	bounding_plane_epsilon := simd.mul(util.F32x8(1e-3), epsilon_scale);
	face_normal_a_in_a_wide, face_normal_b_wide: util.Vector3_Wide;
	face_index_a_wide, face_index_b_wide: util.I32x8;
	if shared_face_hull_a != nil
	{
		face_normal_a_in_a_wide, face_index_a_wide = hull_pair_pick_representative_face_shared_bundle_wide(
			shared_face_hull_a,
			util.vector3_wide_negate(local_normal_in_a),
			closest_on_a_in_a,
			bounding_plane_epsilon,
		);
	}
	if shared_face_hull_b != nil
	{
		face_normal_b_wide, face_index_b_wide = hull_pair_pick_representative_face_shared_bundle_wide(
			shared_face_hull_b,
			local_normal,
			closest_on_b,
			bounding_plane_epsilon,
		);
	}
	face_a_ready := shared_face_hull_a != nil;
	face_b_ready := shared_face_hull_b != nil;
	if HULL_WIDE_CLIP && shared_clip_hull_a != nil && shared_clip_hull_b != nil && pair_count >= 4
	{
		if !face_a_ready
		{
			face_status: Physics_Status;
			face_normal_a_in_a_wide, face_index_a_wide, face_status = hull_pair_pick_faces_scalar_wide(
				shared_clip_hull_a, util.vector3_wide_negate(local_normal_in_a), closest_on_a_in_a,
				bounding_plane_epsilon, inactive, pair_count);
			if face_status != .Ok
			{
				return {}, face_status;
			}
			face_a_ready = true;
		}
		if !face_b_ready
		{
			face_status: Physics_Status;
			face_normal_b_wide, face_index_b_wide, face_status = hull_pair_pick_faces_scalar_wide(
				shared_clip_hull_b, local_normal, closest_on_b, bounding_plane_epsilon, inactive, pair_count);
			if face_status != .Ok
			{
				return {}, face_status;
			}
			face_b_ready = true;
		}
		wide_manifold, handled, clip_status := collision_hull_pair_clip_small_faces_wide(
			shared_clip_hull_a, shared_clip_hull_b, offset_b, b_local_r_a,
			local_offset_a, local_normal, face_normal_a_in_a_wide, face_normal_b_wide,
			face_index_a_wide, face_index_b_wide, epsilon_scale, depth_threshold,
			world_r_b, ~inactive, pair_count,
		);
		if handled == .Present
		{
			return wide_manifold, clip_status;
		}
	}
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		face_normal_a_in_a: util.Vector3;
		face_index_a: int;
		if face_a_ready
		{
			face_normal_a_in_a = util.vector3_wide_read_slot(face_normal_a_in_a_wide, lane);
			face_index_a = int(simd.extract(face_index_a_wide, lane));
		}
		else
		{
			face_a_status: Physics_Status;
			face_normal_a_in_a, face_index_a, face_a_status = convex_hull_pick_representative_face_validated(
				hull_a.hulls[lane],
				util.vector3_negate(util.vector3_wide_read_slot(local_normal_in_a, lane)),
				util.vector3_wide_read_slot(closest_on_a_in_a, lane),
				simd.extract(bounding_plane_epsilon, lane),
			);
			if face_a_status != .Ok
			{
				return {}, face_a_status;
			}
		}
		face_normal_b: util.Vector3;
		face_index_b: int;
		if face_b_ready
		{
			face_normal_b = util.vector3_wide_read_slot(face_normal_b_wide, lane);
			face_index_b = int(simd.extract(face_index_b_wide, lane));
		}
		else
		{
			face_b_status: Physics_Status;
			face_normal_b, face_index_b, face_b_status = convex_hull_pick_representative_face_validated(
				hull_b.hulls[lane],
				util.vector3_wide_read_slot(local_normal, lane),
				util.vector3_wide_read_slot(closest_on_b, lane),
				simd.extract(bounding_plane_epsilon, lane),
			);
			if face_b_status != .Ok
			{
				return {}, face_b_status;
			}
		}
		lane_manifold, lane_status := collision_hull_pair_build_manifold(
			hull_a.hulls[lane],
			hull_b.hulls[lane],
			util.vector3_wide_read_slot(offset_b, lane),
			util.matrix3x3_wide_read_slot(b_local_r_a, lane),
			util.vector3_wide_read_slot(local_offset_a, lane),
			util.vector3_wide_read_slot(local_normal, lane),
			face_normal_a_in_a,
			face_normal_b,
			face_index_a,
			face_index_b,
			simd.extract(epsilon_scale, lane),
			simd.extract(depth_threshold, lane),
			util.matrix3x3_wide_read_slot(world_r_b, lane),
		);
		if lane_status != .Ok
		{
			return {}, lane_status;
		}
		write_status := convex_4_manifold_wide_write_lane(&manifold, lane, lane_manifold);
		if write_status != .Ok
		{
			return {}, write_status;
		}
	}
	return manifold, .Ok;
}

hull_pair_pick_faces_scalar_wide :: proc "contextless" (
	hull: ^Convex_Hull, normal, closest: util.Vector3_Wide,
	epsilon: util.F32x8, inactive: util.I32x8, pair_count: int,
) -> (normals: util.Vector3_Wide, indices: util.I32x8, status: Physics_Status)
{
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		face_normal, face_index, face_status := convex_hull_pick_representative_face_validated(
			hull, util.vector3_wide_read_slot(normal, lane), util.vector3_wide_read_slot(closest, lane), simd.extract(epsilon, lane));
		if face_status != .Ok
		{
			return {}, {}, face_status;
		}
		util.vector3_wide_write_slot(&normals, lane, face_normal);
		indices = simd.replace(indices, lane, i32(face_index));
	}
	return normals, indices, .Ok;
}
