package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

collision_triangle_cylinder_build_manifold :: proc "contextless" (
	a: Triangle, b: Cylinder, pose_a, pose_b: Rigid_Pose,
	world_normal, world_closest_on_b: util.Vector3,
	epsilon_scale, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_normal := util.matrix3x3_transform_transpose(world_normal, world_r_b);
	closest_on_b := util.matrix3x3_transform_transpose(
		util.vector3_subtract(world_closest_on_b, pose_b.position), world_r_b,
	);
	world_vertices := collision_triangle_world_vertices(a, pose_a);
	vertices: [3]util.Vector3;
	for vertex_index in 0 ..< 3
	{
		vertices[vertex_index] = util.matrix3x3_transform_transpose(
			util.vector3_subtract(world_vertices[vertex_index], pose_b.position), world_r_b,
		);
	}
	triangle_center := util.vector3_scale(
		util.vector3_add(util.vector3_add(vertices[0], vertices[1]), vertices[2]), f32(1.0 / 3.0),
	);
	edge_ab := util.vector3_subtract(vertices[1], vertices[0]);
	edge_bc := util.vector3_subtract(vertices[2], vertices[1]);
	edge_ca := util.vector3_subtract(vertices[0], vertices[2]);
	triangle_normal := util.vector3_normalize(util.vector3_cross(edge_ab, edge_ca));
	edge_planes := [3]util.Vector3{
		util.vector3_cross(edge_ab, triangle_normal),
		util.vector3_cross(edge_bc, triangle_normal),
		util.vector3_cross(edge_ca, triangle_normal),
	};
	face_dot_normal := util.vector3_dot(triangle_normal, local_normal);
	use_triangle_edge := abs(face_dot_normal) < 0.2;
	cap_center_y := b.half_length;
	if local_normal.y < 0
	{
		cap_center_y = -cap_center_y;
	}

	if abs(local_normal.y) > 0.70710678118
	{
		inverse_normal_y := 1 / local_normal.y;
		projected := [3]util.Vector2{
			collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, vertices[0]),
			collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, vertices[1]),
			collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, vertices[2]),
		};
		projected_edges := [3]util.Vector2{
			util.vector2_subtract(projected[1], projected[0]),
			util.vector2_subtract(projected[2], projected[1]),
			util.vector2_subtract(projected[0], projected[2]),
		};
		triangle_tangent_x := util.vector3_scale(edge_ab, 1 / util.vector3_length(edge_ab));
		triangle_tangent_y := util.vector3_cross(triangle_tangent_x, triangle_normal);
		tangent_vertices: [3]util.Vector2;
		for vertex_index in 0 ..< 3
		{
			from_center := util.vector3_subtract(vertices[vertex_index], triangle_center);
			tangent_vertices[vertex_index] = {
				util.vector3_dot(from_center, triangle_tangent_x),
				util.vector3_dot(from_center, triangle_tangent_y),
			};
		}
		tangent_edges := [3]util.Vector2{
			util.vector2_subtract(tangent_vertices[1], tangent_vertices[0]),
			util.vector2_subtract(tangent_vertices[2], tangent_vertices[1]),
			util.vector2_subtract(tangent_vertices[0], tangent_vertices[2]),
		};
		candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
		candidate_count := 0;
		for edge_index in 0 ..< 3
		{
			t_min, t_max, intersected := collision_intersect_line_circle(
				projected[edge_index], projected_edges[edge_index], b.radius,
			);
			t_min = min(f32(1), max(f32(0), t_min));
			t_max = min(f32(1), max(f32(0), t_max));
			add_status := collision_add_edge_circle_candidates(
				&candidates, &candidate_count,
				tangent_vertices[edge_index], tangent_edges[edge_index],
				t_min, t_max, intersected, i32(edge_index),
			);
			if add_status != .Ok
			{
				return {}, add_status;
			}
		}
		if !use_triangle_edge
		{
			interior_points := collision_generate_cylinder_interior_points(b, local_normal, closest_on_b);
			inverse_face_dot := 1 / face_dot_normal;
			for interior_index in 0 ..< 4
			{
				interior := interior_points[interior_index];
				point_on_cap := util.Vector3{interior.x, cap_center_y, interior.y};
				projection_t := util.vector3_dot(
					util.vector3_subtract(triangle_center, point_on_cap), local_normal,
				) * -inverse_face_dot;
				point_on_triangle := util.vector3_add(point_on_cap, util.vector3_scale(local_normal, projection_t));
				from_center := util.vector3_subtract(point_on_triangle, triangle_center);
				point := util.Vector2{
					util.vector3_dot(from_center, triangle_tangent_x),
					util.vector3_dot(from_center, triangle_tangent_y),
				};
				cross_ab := (point.x - tangent_vertices[0].x) * tangent_edges[0].y -
					(point.y - tangent_vertices[0].y) * tangent_edges[0].x;
				cross_bc := (point.x - tangent_vertices[1].x) * tangent_edges[1].y -
					(point.y - tangent_vertices[1].y) * tangent_edges[1].x;
				cross_ca := (point.x - tangent_vertices[2].x) * tangent_edges[2].y -
					(point.y - tangent_vertices[2].y) * tangent_edges[2].x;
				all_nonnegative := cross_ab >= 0 && cross_bc >= 0 && cross_ca >= 0;
				all_nonpositive := cross_ab <= 0 && cross_bc <= 0 && cross_ca <= 0;
				if all_nonnegative || all_nonpositive
				{
					candidates[candidate_count] = {
						x=point.x, y=point.y, feature_id=i32(8 + interior_index),
					};
					candidate_count += 1;
				}
			}
		}
		cap_normal := util.Vector3{0, -1, 0};
		if local_normal.y < 0
		{
			cap_normal.y = 1;
		}
		manifold, reduce_status := manifold_candidate_reduce(
			&candidates, candidate_count, cap_normal, -cap_normal.y / local_normal.y,
			{0, cap_center_y, 0}, triangle_center, triangle_tangent_x, triangle_tangent_y,
			epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
		);
		if reduce_status != .Ok
		{
			return {}, reduce_status;
		}
		if manifold.count > 0 && face_dot_normal < -MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION
		{
			manifold.contacts[0].feature_id += MESH_REDUCTION_FACE_COLLISION_FLAG;
		}
		return manifold, .Ok;
	}

	edge_alignments := [3]f32{
		util.vector3_dot(edge_planes[0], local_normal),
		util.vector3_dot(edge_planes[1], local_normal),
		util.vector3_dot(edge_planes[2], local_normal),
	};
	dominant_edge := 0;
	if edge_alignments[1] > edge_alignments[dominant_edge]
	{
		dominant_edge = 1;
	}
	if edge_alignments[2] > edge_alignments[dominant_edge]
	{
		dominant_edge = 2;
	}
	dominant_start := vertices[dominant_edge];
	dominant_offset := edge_ab;
	if dominant_edge == 1
	{
		dominant_offset = edge_bc;
	}
	else if dominant_edge == 2
	{
		dominant_offset = edge_ca;
	}
	dominant_dot_horizontal := dominant_offset.z * local_normal.x - dominant_offset.x * local_normal.z;
	dominant_dot_squared := dominant_dot_horizontal * dominant_dot_horizontal;
	dominant_length_squared := util.vector3_length_squared(dominant_offset);
	horizontal_normal_length_squared := local_normal.x * local_normal.x + local_normal.z * local_normal.z;
	lower_threshold :: f32(0.01 * 0.01);
	upper_threshold :: f32(0.02 * 0.02);
	restrict_weight := max(f32(0), min(f32(1),
		(dominant_dot_squared / (dominant_length_squared * horizontal_normal_length_squared) - lower_threshold) /
		(upper_threshold - lower_threshold),
	));
	contact_t_min, contact_t_max, depth_min, depth_max: f32;
	contact_0, contact_1: util.Vector3;
	if use_triangle_edge
	{
		start_to_side_x := dominant_start.x - closest_on_b.x;
		start_to_side_z := dominant_start.z - closest_on_b.z;
		edge_t := (start_to_side_x * local_normal.z - start_to_side_z * local_normal.x) /
			dominant_dot_horizontal;
		t_center := -(start_to_side_x * dominant_offset.x + dominant_start.y * dominant_offset.y +
			start_to_side_z * dominant_offset.z) / dominant_length_squared;
		projected_extent := b.half_length * abs(dominant_offset.y) / dominant_length_squared;
		interval_min := t_center - projected_extent;
		interval_max := t_center + projected_extent;
		regular_value := edge_t;
		if dominant_dot_squared < lower_threshold
		{
			regular_value = t_center;
		}
		regular_contribution := restrict_weight * regular_value;
		unrestrict_weight := 1 - restrict_weight;
		contact_t_min = min(f32(1), max(f32(0), regular_contribution + unrestrict_weight * interval_min));
		contact_t_max = min(f32(1), max(f32(0), regular_contribution + unrestrict_weight * interval_max));
		contact_0 = util.vector3_add(dominant_start, util.vector3_scale(dominant_offset, contact_t_min));
		contact_1 = util.vector3_add(dominant_start, util.vector3_scale(dominant_offset, contact_t_max));
		inverse_depth_denominator := -1 / horizontal_normal_length_squared;
		depth_base := (start_to_side_x * local_normal.x + start_to_side_z * local_normal.z) *
			inverse_depth_denominator;
		depth_scale := (dominant_offset.x * local_normal.x + dominant_offset.z * local_normal.z) *
			inverse_depth_denominator;
		depth_min = depth_base + depth_scale * contact_t_min;
		depth_max = depth_base + depth_scale * contact_t_max;
	}
	else
	{
		inverse_denominator := 1 / face_dot_normal;
		xz_contribution := (triangle_center.x - closest_on_b.x) * triangle_normal.x +
			(triangle_center.z - closest_on_b.z) * triangle_normal.z;
		t_min_to_triangle := (xz_contribution + (triangle_center.y + b.half_length) * triangle_normal.y) *
			inverse_denominator;
		t_max_to_triangle := (xz_contribution + (triangle_center.y - b.half_length) * triangle_normal.y) *
			inverse_denominator;
		min_on_triangle := util.Vector3{
			closest_on_b.x + t_min_to_triangle * local_normal.x,
			-b.half_length + t_min_to_triangle * local_normal.y,
			closest_on_b.z + t_min_to_triangle * local_normal.z,
		};
		max_on_triangle := util.Vector3{
			closest_on_b.x + t_max_to_triangle * local_normal.x,
			b.half_length + t_max_to_triangle * local_normal.y,
			closest_on_b.z + t_max_to_triangle * local_normal.z,
		};
		min_to_max := util.vector3_subtract(max_on_triangle, min_on_triangle);
		edge_ts: [3]f32;
		entries: [3]f32;
		exits: [3]f32;
		for edge_index in 0 ..< 3
		{
			numerator := util.vector3_dot(
				util.vector3_subtract(vertices[edge_index], min_on_triangle), edge_planes[edge_index],
			);
			denominator := util.vector3_dot(min_to_max, edge_planes[edge_index]);
			exiting := denominator <= 0;
			if abs(denominator) < 1e-30
			{
				denominator = 1e-30;
				if exiting
				{
					denominator = -1e-30;
				}
			}
			edge_ts[edge_index] = numerator / denominator;
			entries[edge_index] = -f32(math.F32_MAX);
			exits[edge_index] = f32(math.F32_MAX);
			if exiting
			{
				exits[edge_index] = edge_ts[edge_index];
			}
			else
			{
				entries[edge_index] = edge_ts[edge_index];
			}
		}
		entries[dominant_edge] *= restrict_weight;
		exits[dominant_edge] = exits[dominant_edge] * restrict_weight + (1 - restrict_weight);
		raw_min := max(entries[0], max(entries[1], entries[2]));
		raw_max := min(exits[0], min(exits[1], exits[2]));
		use_vertex_fallback := raw_max < raw_min;
		contact_t_min = min(f32(1), max(f32(0), raw_min));
		contact_t_max = min(f32(1), max(f32(0), raw_max));
		contact_0 = util.vector3_add(min_on_triangle, util.vector3_scale(min_to_max, contact_t_min));
		contact_1 = util.vector3_add(min_on_triangle, util.vector3_scale(min_to_max, contact_t_max));
		if use_vertex_fallback
		{
			contributed: [3]bool;
			for edge_index in 0 ..< 3
			{
				contributed[edge_index] = edge_ts[edge_index] == raw_min || edge_ts[edge_index] == raw_max;
			}
			contact_0 = vertices[2];
			if contributed[2] && contributed[0]
			{
				contact_0 = vertices[0];
			}
			else if contributed[0] && contributed[1]
			{
				contact_0 = vertices[1];
			}
		}
		inverse_depth_denominator := 1 / horizontal_normal_length_squared;
		depth_min = (local_normal.x * (closest_on_b.x - contact_0.x) +
			local_normal.z * (closest_on_b.z - contact_0.z)) * inverse_depth_denominator;
		depth_max = (local_normal.x * (closest_on_b.x - contact_1.x) +
			local_normal.z * (closest_on_b.z - contact_1.z)) * inverse_depth_denominator;
	}
	manifold := Convex_Contact_Manifold{offset_b=offset_b, normal=world_normal};
	if depth_min > depth_threshold
	{
		add_status := collision_manifold_add_local_b(&manifold, contact_0, depth_min, 0, world_r_b);
		if add_status != .Ok
		{
			return {}, add_status;
		}
	}
	if depth_max > depth_threshold && contact_t_max > contact_t_min
	{
		add_status := collision_manifold_add_local_b(&manifold, contact_1, depth_max, 1, world_r_b);
		if add_status != .Ok
		{
			return {}, add_status;
		}
	}
	if manifold.count > 0 && face_dot_normal < -MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION
	{
		manifold.contacts[0].feature_id += MESH_REDUCTION_FACE_COLLISION_FLAG;
	}
	return manifold, .Ok;
}
triangle_cylinder_test_source :: proc "contextless" (
	a: Triangle, b: Cylinder, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if triangle_validate(a) != .Ok || cylinder_validate(b) != .Ok || speculative_margin < 0 || shapes == nil
	{
		return {}, .Invalid_Description;
	}
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	r_a := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	triangle := Triangle{
		a=util.matrix3x3_transform(a.a, r_a),
		b=util.matrix3x3_transform(a.b, r_a),
		c=util.matrix3x3_transform(a.c, r_a),
	};
	centroid := util.vector3_scale(
		util.vector3_add(triangle.a, util.vector3_add(triangle.b, triangle.c)), f32(1.0 / 3.0),
	);
	triangle.a = util.vector3_subtract(triangle.a, centroid);
	triangle.b = util.vector3_subtract(triangle.b, centroid);
	triangle.c = util.vector3_subtract(triangle.c, centroid);
	local_triangle_center := util.vector3_subtract(centroid, local_offset_b);
	edge_ab := util.vector3_subtract(triangle.b, triangle.a);
	edge_bc := util.vector3_subtract(triangle.c, triangle.b);
	edge_ca := util.vector3_subtract(triangle.a, triangle.c);
	triangle_normal_unnormalized := util.vector3_cross(edge_ab, edge_ca);
	triangle_normal_length := util.vector3_length(triangle_normal_unnormalized);
	triangle_epsilon_scale := math.sqrt(max(
		util.vector3_length_squared(edge_ab), util.vector3_length_squared(edge_ca),
	));
	if triangle_normal_length <= triangle_epsilon_scale * 1e-6
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	triangle_normal := util.vector3_scale(triangle_normal_unnormalized, 1 / triangle_normal_length);
	triangle_a_local := util.vector3_add(triangle.a, local_triangle_center);
	triangle_b_local := util.vector3_add(triangle.b, local_triangle_center);
	triangle_c_local := util.vector3_add(triangle.c, local_triangle_center);
	edge_plane_ab := util.vector3_cross(edge_ab, triangle_normal);
	edge_plane_bc := util.vector3_cross(edge_bc, triangle_normal);
	edge_plane_ca := util.vector3_cross(edge_ca, triangle_normal);
	cylinder_below_plane := util.vector3_dot(triangle_normal, local_triangle_center) >= 0;
	cylinder_inside_edge_planes :=
		util.vector3_dot(edge_plane_ab, triangle_a_local) <= 0 &&
		util.vector3_dot(edge_plane_bc, triangle_b_local) <= 0 &&
		util.vector3_dot(edge_plane_ca, triangle_c_local) <= 0;
	if cylinder_below_plane && cylinder_inside_edge_planes
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	negated_triangle_normal := util.vector3_negate(triangle_normal);
	cylinder_support, support_status := cylinder_support(b, negated_triangle_normal);
	if support_status != .Ok
	{
		return {}, support_status;
	}
	triangle_face_depth := util.vector3_dot(
		util.vector3_subtract(cylinder_support, local_triangle_center), negated_triangle_normal,
	);
	triangle_normal_is_minimal := cylinder_inside_edge_planes && !cylinder_below_plane &&
		util.vector3_dot(edge_plane_ab, util.vector3_subtract(triangle_a_local, cylinder_support)) <= 0 &&
		util.vector3_dot(edge_plane_bc, util.vector3_subtract(triangle_b_local, cylinder_support)) <= 0 &&
		util.vector3_dot(edge_plane_ca, util.vector3_subtract(triangle_c_local, cylinder_support)) <= 0;
	cylinder_epsilon_scale := max(b.half_length, b.radius);
	depth := triangle_face_depth;
	local_normal := negated_triangle_normal;
	closest_on_b_local := cylinder_support;
	if !triangle_normal_is_minimal
	{
		center_distance := util.vector3_length(local_triangle_center);
		initial_normal := util.Vector3{0, 1, 0};
		if center_distance >= 1e-10
		{
			initial_normal = util.vector3_scale(local_triangle_center, 1 / center_distance);
		}
		shape_a := triangle;
		shape_b := b;
		triangle_pose := Rigid_Pose{
			position=util.vector3_add(
				pose_b.position, util.matrix3x3_transform(local_triangle_center, world_r_b),
			),
			orientation=pose_b.orientation,
		};
		view_a, view_b, view_status := collision_pair_views(
			&shape_a, &shape_b, TRIANGLE_TYPE_ID, CYLINDER_TYPE_ID, triangle_pose, pose_b, shapes,
		);
		if view_status != .Ok
		{
			return {}, view_status;
		}
		world_initial_normal := util.matrix3x3_transform(initial_normal, world_r_b);
		refined_normal, refined_closest_on_b: util.Vector3;
		refine_status: Physics_Status;
		depth, refined_normal, refined_closest_on_b, refine_status = depth_refiner_find_minimum_depth(
			view_b, view_a, world_initial_normal, cylinder_epsilon_scale * 1e-5,
			-speculative_margin, shapes,
		);
		if refine_status != .Ok
		{
			return {}, refine_status;
		}
		local_normal = util.matrix3x3_transform_transpose(refined_normal, world_r_b);
		closest_on_b_local = util.matrix3x3_transform_transpose(
			util.vector3_subtract(refined_closest_on_b, pose_b.position), world_r_b,
		);
	}
	if depth < -speculative_margin ||
		util.vector3_dot(triangle_normal, local_normal) > -TRIANGLE_BACKFACE_REJECTION_THRESHOLD
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	world_normal := util.matrix3x3_transform(local_normal, world_r_b);
	world_closest_on_b := util.vector3_add(
		pose_b.position, util.matrix3x3_transform(closest_on_b_local, world_r_b),
	);
	return collision_triangle_cylinder_build_manifold(
		a, b, pose_a, pose_b, world_normal, world_closest_on_b,
		cylinder_epsilon_scale, -speculative_margin,
	);
}
collision_triangle_support_world_wide :: proc "contextless" (
	vertices: [3]util.Vector3_Wide, direction: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	dot_0 := util.vector3_wide_dot(vertices[0], direction);
	dot_1 := util.vector3_wide_dot(vertices[1], direction);
	dot_2 := util.vector3_wide_dot(vertices[2], direction);
	use_1 := transmute(util.I32x8)simd.lanes_gt(dot_1, dot_0);
	best := util.vector3_wide_select(use_1, vertices[1], vertices[0]);
	best_dot := util.wide_select_f32(use_1, dot_1, dot_0);
	return util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_gt(dot_2, best_dot), vertices[2], best,
	);
}
Triangle_Cylinder_Depth_Context_Wide :: struct
{
	triangle:             Triangle_Wide,
	cylinder:             Cylinder_Wide,
	local_triangle_center: util.Vector3_Wide,
}
triangle_cylinder_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_b: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Triangle_Cylinder_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	support_on_b = collision_cylinder_support_local_wide(depth_context.cylinder, direction);
	support_on_a := collision_triangle_support_world_wide(
		[3]util.Vector3_Wide{
			depth_context.triangle.a, depth_context.triangle.b, depth_context.triangle.c,
		},
		util.vector3_wide_negate(direction),
	);
	support_on_a = util.vector3_wide_add(support_on_a, depth_context.local_triangle_center);
	return util.vector3_wide_subtract(support_on_b, support_on_a), support_on_b, .Ok;
}
triangle_cylinder_test_wide :: proc "contextless" (
	a: Triangle_Wide, b: Cylinder_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
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
	r_a := util.matrix3x3_wide_multiply_by_transpose(world_r_a, world_r_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_b);
	triangle := Triangle_Wide{
		a=util.matrix3x3_wide_transform(a.a, r_a),
		b=util.matrix3x3_wide_transform(a.b, r_a),
		c=util.matrix3x3_wide_transform(a.c, r_a),
	};
	centroid := util.vector3_wide_scale(
		util.vector3_wide_add(triangle.a, util.vector3_wide_add(triangle.b, triangle.c)), util.F32x8(1.0 / 3.0),
	);
	triangle.a = util.vector3_wide_subtract(triangle.a, centroid);
	triangle.b = util.vector3_wide_subtract(triangle.b, centroid);
	triangle.c = util.vector3_wide_subtract(triangle.c, centroid);
	local_triangle_center := util.vector3_wide_subtract(centroid, local_offset_b);
	triangle_ab := util.vector3_wide_subtract(triangle.b, triangle.a);
	triangle_bc := util.vector3_wide_subtract(triangle.c, triangle.b);
	triangle_ca := util.vector3_wide_subtract(triangle.a, triangle.c);
	triangle_a_local := util.vector3_wide_add(triangle.a, local_triangle_center);
	triangle_b_local := util.vector3_wide_add(triangle.b, local_triangle_center);
	triangle_c_local := util.vector3_wide_add(triangle.c, local_triangle_center);
	triangle_normal_unnormalized := util.vector3_wide_cross(triangle_ab, triangle_ca);
	triangle_normal_length := util.vector3_wide_length(triangle_normal_unnormalized);
	triangle_epsilon_scale := simd.sqrt(simd.max(
		util.vector3_wide_length_squared(triangle_ab), util.vector3_wide_length_squared(triangle_ca),
	));
	inactive |= transmute(util.I32x8)simd.lanes_le(
		triangle_normal_length, simd.mul(triangle_epsilon_scale, util.F32x8(1e-6)),
	);
	triangle_normal := util.vector3_wide_scale(
		triangle_normal_unnormalized, simd.div(util.F32x8(1), triangle_normal_length),
	);
	edge_plane_ab := util.vector3_wide_cross(triangle_ab, triangle_normal);
	edge_plane_bc := util.vector3_wide_cross(triangle_bc, triangle_normal);
	edge_plane_ca := util.vector3_wide_cross(triangle_ca, triangle_normal);
	cylinder_below_plane := transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(triangle_normal, local_triangle_center), util.F32x8(0),
	);
	cylinder_inside_edge_planes :=
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_ab, triangle_a_local), util.F32x8(0),
	) &
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_bc, triangle_b_local), util.F32x8(0),
	) &
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_ca, triangle_c_local), util.F32x8(0),
	);
	inactive |= cylinder_below_plane & cylinder_inside_edge_planes;
	negated_triangle_normal := util.vector3_wide_negate(triangle_normal);
	cylinder_support := collision_cylinder_support_local_wide(b, negated_triangle_normal);
	triangle_face_depth := util.vector3_wide_dot(
		util.vector3_wide_subtract(cylinder_support, local_triangle_center), negated_triangle_normal,
	);
	triangle_normal_is_minimal := ~cylinder_below_plane & cylinder_inside_edge_planes &
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_ab, util.vector3_wide_subtract(triangle_a_local, cylinder_support)),
		util.F32x8(0),
	) &
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_bc, util.vector3_wide_subtract(triangle_b_local, cylinder_support)),
		util.F32x8(0),
	) &
		transmute(util.I32x8)simd.lanes_le(
		util.vector3_wide_dot(edge_plane_ca, util.vector3_wide_subtract(triangle_c_local, cylinder_support)),
		util.F32x8(0),
	);
	center_distance := util.vector3_wide_length(local_triangle_center);
	initial_normal := util.vector3_wide_scale(
		local_triangle_center, simd.div(util.F32x8(1), center_distance),
	);
	initial_normal = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(center_distance, util.F32x8(1e-10)),
		{y=util.F32x8(1)}, initial_normal,
	);
	cylinder_epsilon_scale := simd.max(b.half_length, b.radius);
	depth_threshold := simd.neg(speculative_margin);
	depth_context := Triangle_Cylinder_Depth_Context_Wide{
		triangle=triangle, cylinder=b, local_triangle_center=local_triangle_center,
	};
	refined_depth, refined_normal, refined_closest_on_b, depth_status := depth_refiner_find_minimum_depth_wide(
		triangle_cylinder_depth_support_wide, &depth_context,
		initial_normal, inactive | triangle_normal_is_minimal,
		simd.mul(cylinder_epsilon_scale, util.F32x8(1e-5)), depth_threshold, {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	depth := util.wide_select_f32(triangle_normal_is_minimal, triangle_face_depth, refined_depth);
	local_normal := util.vector3_wide_select(triangle_normal_is_minimal, negated_triangle_normal, refined_normal);
	closest_on_b_local := util.vector3_wide_select(
		triangle_normal_is_minimal, cylinder_support, refined_closest_on_b,
	);
	inactive |= transmute(util.I32x8)simd.lanes_lt(depth, depth_threshold) |
		transmute(util.I32x8)simd.lanes_gt(
		util.vector3_wide_dot(triangle_normal, local_normal), util.F32x8(-TRIANGLE_BACKFACE_REJECTION_THRESHOLD),
	);
	world_normal := util.matrix3x3_wide_transform(local_normal, world_r_b);
	world_closest_on_b := util.vector3_wide_add(
		offset_b, util.matrix3x3_wide_transform(closest_on_b_local, world_r_b),
	);
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		cylinder := Cylinder{simd.extract(b.radius, lane), simd.extract(b.half_length, lane)};
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		lane_normal := util.vector3_wide_read_slot(world_normal, lane);
		lane_closest := util.vector3_wide_read_slot(world_closest_on_b, lane);
		lane_manifold, lane_status := collision_triangle_cylinder_build_manifold(
			Triangle{
				a=util.vector3_wide_read_slot(a.a, lane),
				b=util.vector3_wide_read_slot(a.b, lane),
				c=util.vector3_wide_read_slot(a.c, lane),
			},
			cylinder, pose_a, pose_b, lane_normal, lane_closest,
			simd.extract(cylinder_epsilon_scale, lane), -simd.extract(speculative_margin, lane),
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
