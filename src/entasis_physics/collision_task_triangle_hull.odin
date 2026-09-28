// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

Triangle_Hull_Depth_Context_Wide :: struct
{
	hull:                  ^Convex_Hull,
	triangle:              Triangle,
	local_triangle_center: util.Vector3,
}

triangle_hull_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_a: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Triangle_Hull_Depth_Context_Wide)(user_context);
	if depth_context == nil || convex_hull_validate(depth_context.hull) != .Ok
	{
		return {}, {}, .Invalid_Argument;
	}
	if simd.extract(terminated_lanes, 0) < 0
	{
		return {}, {}, .Ok;
	}
	direction_lane := util.vector3_wide_read_slot(direction, 0);
	hull_support, hull_status := convex_hull_support(depth_context.hull, direction_lane);
	if hull_status != .Ok
	{
		return {}, {}, hull_status;
	}
	triangle_direction := util.vector3_negate(direction_lane);
	triangle_support := depth_context.triangle.a;
	if util.vector3_dot(
		depth_context.triangle.b,
		triangle_direction
	) > util.vector3_dot(triangle_support, triangle_direction)
	{
		triangle_support = depth_context.triangle.b;
	}
	if util.vector3_dot(
		depth_context.triangle.c,
		triangle_direction
	) > util.vector3_dot(triangle_support, triangle_direction)
	{
		triangle_support = depth_context.triangle.c;
	}
	triangle_support = util.vector3_add(triangle_support, depth_context.local_triangle_center);
	util.vector3_wide_write_slot(&support_on_a, 0, hull_support);
	util.vector3_wide_write_slot(&support, 0, util.vector3_subtract(hull_support, triangle_support));
	return support, support_on_a, .Ok;
}

triangle_hull_add_edge_candidates :: proc "contextless" (
	candidates: ^[MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar,
	candidate_count: ^int,
	edge_start, edge_offset, tangent_x, tangent_y: util.Vector3,
	latest_entry, earliest_exit: f32,
	feature_id_min, feature_id_max: i32,
)
{
	entry := max(f32(0), latest_entry);
	exit := min(f32(1), earliest_exit);
	if exit >= entry && candidate_count^ < len(candidates^)
	{
		point := util.vector3_add(edge_start, util.vector3_scale(edge_offset, exit));
		candidates[candidate_count^] = {
			x=util.vector3_dot(point, tangent_x), y=util.vector3_dot(point, tangent_y),
			feature_id=feature_id_max,
		};
		candidate_count^ += 1;
	}
	if entry < exit && entry > 0 && candidate_count^ < len(candidates^)
	{
		point := util.vector3_add(edge_start, util.vector3_scale(edge_offset, entry));
		candidates[candidate_count^] = {
			x=util.vector3_dot(point, tangent_x), y=util.vector3_dot(point, tangent_y),
			feature_id=feature_id_min,
		};
		candidate_count^ += 1;
	}
}

triangle_convex_hull_test_source :: proc "contextless" (
	triangle_a: Triangle, hull_b: ^Convex_Hull,
	pose_a, pose_b: Rigid_Pose, speculative_margin: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if triangle_validate(triangle_a) != .Ok || convex_hull_validate(hull_b) != .Ok || speculative_margin < 0
	{
		return {}, .Invalid_Description;
	}
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	hull_local_triangle_orientation := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);

	triangle := Triangle{
		a=util.matrix3x3_transform(triangle_a.a, hull_local_triangle_orientation),
		b=util.matrix3x3_transform(triangle_a.b, hull_local_triangle_orientation),
		c=util.matrix3x3_transform(triangle_a.c, hull_local_triangle_orientation),
	};
	centroid := util.vector3_scale(util.vector3_add(triangle.a, util.vector3_add(triangle.b, triangle.c)), 1.0 / 3.0);
	triangle.a = util.vector3_subtract(triangle.a, centroid);
	triangle.b = util.vector3_subtract(triangle.b, centroid);
	triangle.c = util.vector3_subtract(triangle.c, centroid);
	local_triangle_center := util.vector3_subtract(centroid, local_offset_b);
	triangle_ab := util.vector3_subtract(triangle.b, triangle.a);
	triangle_bc := util.vector3_subtract(triangle.c, triangle.b);
	triangle_ca := util.vector3_subtract(triangle.a, triangle.c);
	triangle_a_local := util.vector3_add(triangle.a, local_triangle_center);
	triangle_b_local := util.vector3_add(triangle.b, local_triangle_center);
	triangle_c_local := util.vector3_add(triangle.c, local_triangle_center);
	triangle_normal_unnormalized := util.vector3_cross(triangle_ab, triangle_ca);
	triangle_normal_length := util.vector3_length(triangle_normal_unnormalized);
	epsilon_scale_triangle := math.sqrt(max(
		util.vector3_length_squared(triangle_ab), util.vector3_length_squared(triangle_ca),
	));
	if triangle_normal_length <= 1e-6 * epsilon_scale_triangle
	{
		return {offset_b=offset_b}, .Ok;
	}
	triangle_normal := util.vector3_scale(triangle_normal_unnormalized, 1 / triangle_normal_length);
	edge_plane_ab := util.vector3_cross(triangle_ab, triangle_normal);
	edge_plane_bc := util.vector3_cross(triangle_bc, triangle_normal);
	edge_plane_ca := util.vector3_cross(triangle_ca, triangle_normal);
	hull_below_plane := util.vector3_dot(triangle_normal, local_triangle_center) >= 0;
	hull_inside_triangle_edge_planes :=
		util.vector3_dot(edge_plane_ab, triangle_a_local) <= 0 &&
		util.vector3_dot(edge_plane_bc, triangle_b_local) <= 0 &&
		util.vector3_dot(edge_plane_ca, triangle_c_local) <= 0;
	if hull_below_plane && hull_inside_triangle_edge_planes
	{
		return {offset_b=offset_b}, .Ok;
	}

	hull_point := convex_hull_get_point(hull_b, 0);
	epsilon_scale_hull := (abs(hull_point.x) + abs(hull_point.y) + abs(hull_point.z)) * (1.0 / 3.0);
	epsilon_scale := min(epsilon_scale_triangle, epsilon_scale_hull);
	center_distance := util.vector3_length(local_triangle_center);
	initial_normal := util.Vector3{0, 1, 0};
	if center_distance >= 1e-10
	{
		initial_normal = util.vector3_scale(local_triangle_center, 1 / center_distance);
	}
	negated_triangle_normal := util.vector3_negate(triangle_normal);
	hull_support, support_status := convex_hull_support(hull_b, negated_triangle_normal);
	if support_status != .Ok
	{
		return {}, support_status;
	}
	support_from_triangle_center := util.vector3_subtract(hull_support, local_triangle_center);
	triangle_face_depth := util.vector3_dot(support_from_triangle_center, negated_triangle_normal);
	extreme_ab := util.vector3_dot(edge_plane_ab, util.vector3_subtract(triangle_a_local, hull_support));
	extreme_bc := util.vector3_dot(edge_plane_bc, util.vector3_subtract(triangle_b_local, hull_support));
	extreme_ca := util.vector3_dot(edge_plane_ca, util.vector3_subtract(triangle_c_local, hull_support));
	triangle_normal_is_minimal := !hull_below_plane && hull_inside_triangle_edge_planes &&
		extreme_ab <= 0 && extreme_bc <= 0 && extreme_ca <= 0;
	depth_threshold := -speculative_margin;
	local_normal := negated_triangle_normal;
	closest_on_hull := hull_support;
	depth := triangle_face_depth;
	if !triangle_normal_is_minimal
	{
		depth_context := Triangle_Hull_Depth_Context_Wide{
			hull=hull_b, triangle=triangle, local_triangle_center=local_triangle_center,
		};
		initial_normal_wide: util.Vector3_Wide;
		util.vector3_wide_write_slot(&initial_normal_wide, 0, initial_normal);
		inactive_lanes := util.I32x8(-1);
		inactive_lanes = simd.replace(inactive_lanes, 0, 0);
		depth_wide, normal_wide, closest_wide, refine_status := depth_refiner_find_minimum_depth_wide(
			triangle_hull_depth_support_wide, &depth_context, initial_normal_wide, inactive_lanes,
			util.F32x8(1e-4 * epsilon_scale), util.F32x8(depth_threshold), {}, {},
		);
		if refine_status != .Ok
		{
			return {}, refine_status;
		}
		depth = simd.extract(depth_wide, 0);
		local_normal = util.vector3_wide_read_slot(normal_wide, 0);
		closest_on_hull = util.vector3_wide_read_slot(closest_wide, 0);
	}
	triangle_normal_dot_local_normal := util.vector3_dot(triangle_normal, local_normal);
	if triangle_normal_dot_local_normal > -TRIANGLE_BACKFACE_REJECTION_THRESHOLD || depth < depth_threshold
	{
		return {offset_b=offset_b}, .Ok;
	}

	hull_face_normal, hull_face_index, face_status := convex_hull_pick_representative_face(
		hull_b, local_normal, closest_on_hull, 1e-3 * epsilon_scale,
	);
	if face_status != .Ok
	{
		return {}, face_status;
	}
	face_start, face_end, range_status := convex_hull_face_range(hull_b, hull_face_index);
	if range_status != .Ok
	{
		return {}, range_status;
	}
	face_vertex_count := face_end - face_start;
	if face_vertex_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}

	inverse_hull_face_denominator := 1 / util.vector3_dot(local_normal, hull_face_normal);
	project_to_hull := proc "contextless" (
		point, closest, normal, face_normal: util.Vector3, inverse_denominator: f32,
	) -> util.Vector3
	{
		t := util.vector3_dot(util.vector3_subtract(point, closest), face_normal) * inverse_denominator;
		return util.vector3_subtract(point, util.vector3_scale(normal, t));
	}
	a_on_hull := project_to_hull(
		triangle_a_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	b_on_hull := project_to_hull(
		triangle_b_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	c_on_hull := project_to_hull(
		triangle_c_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	ab_on_hull := util.vector3_subtract(b_on_hull, a_on_hull);
	bc_on_hull := util.vector3_subtract(c_on_hull, b_on_hull);
	ca_on_hull := util.vector3_subtract(a_on_hull, c_on_hull);
	ab_edge_plane_on_hull := util.vector3_cross(ab_on_hull, hull_face_normal);
	bc_edge_plane_on_hull := util.vector3_cross(bc_on_hull, hull_face_normal);
	ca_edge_plane_on_hull := util.vector3_cross(ca_on_hull, hull_face_normal);
	triangle_tangent_x := util.vector3_normalize(triangle_ab);
	triangle_tangent_y := util.vector3_cross(triangle_tangent_x, triangle_normal);
	inverse_triangle_normal_dot_local_normal := 1 / triangle_normal_dot_local_normal;

	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
	candidate_count := 0;
	previous_index := int(hull_b.face_vertex_indices.memory[face_end - 1]);
	hull_face_origin := convex_hull_get_point(hull_b, previous_index);
	previous_vertex := hull_face_origin;
	latest_entry_ab := -f32(math.F32_MAX);
	earliest_exit_ab := f32(math.F32_MAX);
	latest_entry_bc := -f32(math.F32_MAX);
	earliest_exit_bc := f32(math.F32_MAX);
	latest_entry_ca := -f32(math.F32_MAX);
	earliest_exit_ca := f32(math.F32_MAX);
	for face_vertex_index in 0 ..< face_vertex_count
	{
		point_index := int(hull_b.face_vertex_indices.memory[face_start + face_vertex_index]);
		vertex := convex_hull_get_point(hull_b, point_index);
		hull_edge_offset := util.vector3_subtract(vertex, previous_vertex);
		ap := util.vector3_subtract(vertex, a_on_hull);
		bp := util.vector3_subtract(vertex, b_on_hull);
		if util.vector3_dot(ap, ab_edge_plane_on_hull) < 0 &&
			util.vector3_dot(bp, bc_edge_plane_on_hull) < 0 &&
			util.vector3_dot(ap, ca_edge_plane_on_hull) < 0 &&
			candidate_count < len(candidates)
		{
			projection_t := util.vector3_dot(util.vector3_subtract(vertex, triangle_a_local), triangle_normal) *
				inverse_triangle_normal_dot_local_normal;
			projected_vertex := util.vector3_subtract(vertex, util.vector3_scale(local_normal, projection_t));
			to_vertex := util.vector3_subtract(projected_vertex, triangle_a_local);
			candidates[candidate_count] = {
				x=util.vector3_dot(to_vertex, triangle_tangent_x),
				y=util.vector3_dot(to_vertex, triangle_tangent_y),
				feature_id=i32(6 + face_vertex_index),
			};
			candidate_count += 1;
		}
		hull_edge_plane_normal := util.vector3_cross(hull_edge_offset, local_normal);
		starts := [3]util.Vector3{a_on_hull, b_on_hull, c_on_hull};
		offsets := [3]util.Vector3{ab_on_hull, bc_on_hull, ca_on_hull};
		for edge_index in 0 ..< 3
		{
			numerator := util.vector3_dot(
				util.vector3_subtract(previous_vertex, starts[edge_index]),
				hull_edge_plane_normal
			);
			denominator := util.vector3_dot(hull_edge_plane_normal, offsets[edge_index]);
			latest := &latest_entry_ab;
			earliest := &earliest_exit_ab;
			if edge_index == 1
			{
				latest=&latest_entry_bc;
				earliest=&earliest_exit_bc;
			}
			else if edge_index == 2
			{
				latest=&latest_entry_ca;
				earliest=&earliest_exit_ca;
			}
			if denominator < 0
			{
				if latest^ * denominator > numerator
				{
					latest^ = numerator / denominator;
				}
			}
			else if denominator > 0
			{
				if earliest^ * denominator > numerator
				{
					earliest^ = numerator / denominator;
				}
			}
			else if numerator < 0
			{
				earliest^ = -f32(math.F32_MAX);
				latest^ = f32(math.F32_MAX);
			}
		}
		previous_vertex = vertex;
	}
	triangle_ab_from_a := triangle_ab;
	triangle_bc_from_a := triangle_ab;
	triangle_ca_from_a := util.vector3_negate(triangle_ca);
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, {}, triangle_ab_from_a, triangle_tangent_x, triangle_tangent_y,
		latest_entry_ab, earliest_exit_ab, 1, 0,
	);
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, triangle_bc_from_a, triangle_bc, triangle_tangent_x, triangle_tangent_y,
		latest_entry_bc, earliest_exit_bc, 3, 2,
	);
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, triangle_ca_from_a, triangle_ca, triangle_tangent_x, triangle_tangent_y,
		latest_entry_ca, earliest_exit_ca, 5, 4,
	);
	world_normal := util.matrix3x3_transform(local_normal, world_r_b);
	manifold, reduce_status := manifold_candidate_reduce(
		&candidates, candidate_count, hull_face_normal,
		-1 / util.vector3_dot(hull_face_normal, local_normal),
		previous_vertex, triangle_a_local, triangle_tangent_x, triangle_tangent_y,
		epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
	);
	if reduce_status != .Ok
	{
		return {}, reduce_status;
	}
	if manifold.count > 0 && triangle_normal_dot_local_normal < -MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION
	{
		manifold.contacts[0].feature_id += MESH_REDUCTION_FACE_COLLISION_FLAG;
	}
	return manifold, .Ok;
}

Triangle_Hull_Batch_Depth_Context_Wide :: struct
{
	hulls:                 Convex_Hull_Wide,
	triangles:             Triangle_Wide,
	local_triangle_center: util.Vector3_Wide,
}

triangle_hull_batch_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_b: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Triangle_Hull_Batch_Depth_Context_Wide)(user_context);
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
		if convex_hull_validate(hull) != .Ok
		{
			return {}, {}, .Invalid_Description;
		}
		direction_lane := util.vector3_wide_read_slot(direction, lane);
		hull_support, hull_status := convex_hull_support(hull, direction_lane);
		if hull_status != .Ok
		{
			return {}, {}, hull_status;
		}
		triangle_direction := util.vector3_negate(direction_lane);
		triangle := Triangle{
			a=util.vector3_wide_read_slot(depth_context.triangles.a, lane),
			b=util.vector3_wide_read_slot(depth_context.triangles.b, lane),
			c=util.vector3_wide_read_slot(depth_context.triangles.c, lane),
		};
		triangle_support := triangle.a;
		if util.vector3_dot(triangle.b, triangle_direction) > util.vector3_dot(triangle_support, triangle_direction)
		{
			triangle_support = triangle.b;
		}
		if util.vector3_dot(triangle.c, triangle_direction) > util.vector3_dot(triangle_support, triangle_direction)
		{
			triangle_support = triangle.c;
		}
		triangle_support = util.vector3_add(
			triangle_support, util.vector3_wide_read_slot(depth_context.local_triangle_center, lane),
		);
		util.vector3_wide_write_slot(&support_on_b, lane, hull_support);
		util.vector3_wide_write_slot(&support, lane, util.vector3_subtract(hull_support, triangle_support));
	}
	return support, support_on_b, .Ok;
}

collision_triangle_hull_build_manifold :: proc "contextless" (
	hull: ^Convex_Hull, pose_a, pose_b: Rigid_Pose, triangle: Triangle,
	local_triangle_center, triangle_normal, local_normal, closest_on_hull: util.Vector3,
	world_r_b: util.Matrix3x3, epsilon_scale, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	triangle_ab := util.vector3_subtract(triangle.b, triangle.a);
	triangle_bc := util.vector3_subtract(triangle.c, triangle.b);
	triangle_ca := util.vector3_subtract(triangle.a, triangle.c);
	triangle_a_local := util.vector3_add(triangle.a, local_triangle_center);
	triangle_b_local := util.vector3_add(triangle.b, local_triangle_center);
	triangle_c_local := util.vector3_add(triangle.c, local_triangle_center);
	triangle_normal_dot_local_normal := util.vector3_dot(triangle_normal, local_normal);
	if triangle_normal_dot_local_normal > -TRIANGLE_BACKFACE_REJECTION_THRESHOLD
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	hull_face_normal, hull_face_index, face_status := convex_hull_pick_representative_face(
		hull, local_normal, closest_on_hull, 1e-3 * epsilon_scale,
	);
	if face_status != .Ok
	{
		return {}, face_status;
	}
	face_start, face_end, range_status := convex_hull_face_range(hull, hull_face_index);
	if range_status != .Ok
	{
		return {}, range_status;
	}
	face_vertex_count := face_end - face_start;
	if face_vertex_count > MAXIMUM_MANIFOLD_CANDIDATE_COUNT
	{
		return {}, .Capacity_Missing;
	}
	inverse_hull_face_denominator := 1 / util.vector3_dot(local_normal, hull_face_normal);
	project_to_hull := proc "contextless" (
		point, closest, normal, face_normal: util.Vector3, inverse_denominator: f32,
	) -> util.Vector3
	{
		t := util.vector3_dot(util.vector3_subtract(point, closest), face_normal) * inverse_denominator;
		return util.vector3_subtract(point, util.vector3_scale(normal, t));
	}
	a_on_hull := project_to_hull(
		triangle_a_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	b_on_hull := project_to_hull(
		triangle_b_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	c_on_hull := project_to_hull(
		triangle_c_local,
		closest_on_hull,
		local_normal,
		hull_face_normal,
		inverse_hull_face_denominator
	);
	ab_on_hull := util.vector3_subtract(b_on_hull, a_on_hull);
	bc_on_hull := util.vector3_subtract(c_on_hull, b_on_hull);
	ca_on_hull := util.vector3_subtract(a_on_hull, c_on_hull);
	ab_edge_plane_on_hull := util.vector3_cross(ab_on_hull, hull_face_normal);
	bc_edge_plane_on_hull := util.vector3_cross(bc_on_hull, hull_face_normal);
	ca_edge_plane_on_hull := util.vector3_cross(ca_on_hull, hull_face_normal);
	triangle_tangent_x := util.vector3_normalize(triangle_ab);
	triangle_tangent_y := util.vector3_cross(triangle_tangent_x, triangle_normal);
	inverse_triangle_normal_dot_local_normal := 1 / triangle_normal_dot_local_normal;
	candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
	candidate_count := 0;
	previous_index := int(hull.face_vertex_indices.memory[face_end - 1]);
	hull_face_origin := convex_hull_get_point(hull, previous_index);
	previous_vertex := hull_face_origin;
	latest_entry_ab := -f32(math.F32_MAX);
	earliest_exit_ab := f32(math.F32_MAX);
	latest_entry_bc := -f32(math.F32_MAX);
	earliest_exit_bc := f32(math.F32_MAX);
	latest_entry_ca := -f32(math.F32_MAX);
	earliest_exit_ca := f32(math.F32_MAX);
	for face_vertex_index in 0 ..< face_vertex_count
	{
		point_index := int(hull.face_vertex_indices.memory[face_start + face_vertex_index]);
		vertex := convex_hull_get_point(hull, point_index);
		hull_edge_offset := util.vector3_subtract(vertex, previous_vertex);
		ap := util.vector3_subtract(vertex, a_on_hull);
		bp := util.vector3_subtract(vertex, b_on_hull);
		if util.vector3_dot(ap, ab_edge_plane_on_hull) < 0 &&
			util.vector3_dot(bp, bc_edge_plane_on_hull) < 0 &&
			util.vector3_dot(ap, ca_edge_plane_on_hull) < 0 &&
			candidate_count < len(candidates)
		{
			projection_t := util.vector3_dot(util.vector3_subtract(vertex, triangle_a_local), triangle_normal) *
				inverse_triangle_normal_dot_local_normal;
			projected_vertex := util.vector3_subtract(vertex, util.vector3_scale(local_normal, projection_t));
			to_vertex := util.vector3_subtract(projected_vertex, triangle_a_local);
			candidates[candidate_count] = {
				x=util.vector3_dot(to_vertex, triangle_tangent_x),
				y=util.vector3_dot(to_vertex, triangle_tangent_y),
				feature_id=i32(6 + face_vertex_index),
			};
			candidate_count += 1;
		}
		hull_edge_plane_normal := util.vector3_cross(hull_edge_offset, local_normal);
		starts := [3]util.Vector3{a_on_hull, b_on_hull, c_on_hull};
		offsets := [3]util.Vector3{ab_on_hull, bc_on_hull, ca_on_hull};
		for edge_index in 0 ..< 3
		{
			numerator := util.vector3_dot(
				util.vector3_subtract(previous_vertex, starts[edge_index]),
				hull_edge_plane_normal
			);
			denominator := util.vector3_dot(hull_edge_plane_normal, offsets[edge_index]);
			latest := &latest_entry_ab;
			earliest := &earliest_exit_ab;
			if edge_index == 1
			{
				latest=&latest_entry_bc;
				earliest=&earliest_exit_bc;
			}
			else if edge_index == 2
			{
				latest=&latest_entry_ca;
				earliest=&earliest_exit_ca;
			}
			if denominator < 0
			{
				if latest^ * denominator > numerator
				{
					latest^ = numerator / denominator;
				}
			}
			else if denominator > 0
			{
				if earliest^ * denominator > numerator
				{
					earliest^ = numerator / denominator;
				}
			}
			else if numerator < 0
			{
				earliest^ = -f32(math.F32_MAX);
				latest^ = f32(math.F32_MAX);
			}
		}
		previous_vertex = vertex;
	}
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, {}, triangle_ab, triangle_tangent_x, triangle_tangent_y,
		latest_entry_ab, earliest_exit_ab, 1, 0,
	);
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, triangle_ab, triangle_bc, triangle_tangent_x, triangle_tangent_y,
		latest_entry_bc, earliest_exit_bc, 3, 2,
	);
	triangle_hull_add_edge_candidates(
		&candidates, &candidate_count, util.vector3_negate(triangle_ca), triangle_ca, triangle_tangent_x, triangle_tangent_y,
		latest_entry_ca, earliest_exit_ca, 5, 4,
	);
	world_normal := util.matrix3x3_transform(local_normal, world_r_b);
	manifold, reduce_status := manifold_candidate_reduce(
		&candidates, candidate_count, hull_face_normal,
		-1 / util.vector3_dot(hull_face_normal, local_normal),
		previous_vertex, triangle_a_local, triangle_tangent_x, triangle_tangent_y,
		epsilon_scale, depth_threshold, world_r_b,
		util.vector3_subtract(pose_b.position, pose_a.position), world_normal,
	);
	if reduce_status != .Ok
	{
		return {}, reduce_status;
	}
	if manifold.count > 0 && triangle_normal_dot_local_normal < -MESH_REDUCTION_MINIMUM_DOT_FOR_FACE_COLLISION
	{
		manifold.contacts[0].feature_id += MESH_REDUCTION_FACE_COLLISION_FLAG;
	}
	return manifold, .Ok;
}

triangle_convex_hull_test_wide :: proc "contextless" (
	triangle_a: Triangle_Wide, hulls: Convex_Hull_Wide, speculative_margin: util.F32x8,
	offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
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
	hull_local_triangle_orientation := util.matrix3x3_wide_multiply_by_transpose(world_r_a, world_r_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_b);
	triangle := Triangle_Wide{
		a=util.matrix3x3_wide_transform(triangle_a.a, hull_local_triangle_orientation),
		b=util.matrix3x3_wide_transform(triangle_a.b, hull_local_triangle_orientation),
		c=util.matrix3x3_wide_transform(triangle_a.c, hull_local_triangle_orientation),
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
	epsilon_scale_triangle := simd.sqrt(simd.max(
		util.vector3_wide_length_squared(triangle_ab), util.vector3_wide_length_squared(triangle_ca),
	));
	inactive |= transmute(util.I32x8)simd.lanes_le(
		triangle_normal_length, simd.mul(util.F32x8(1e-6), epsilon_scale_triangle),
	);
	triangle_normal := util.vector3_wide_scale(
		triangle_normal_unnormalized, simd.div(util.F32x8(1), triangle_normal_length),
	);
	edge_plane_ab := util.vector3_wide_cross(triangle_ab, triangle_normal);
	edge_plane_bc := util.vector3_wide_cross(triangle_bc, triangle_normal);
	edge_plane_ca := util.vector3_wide_cross(triangle_ca, triangle_normal);
	hull_below_plane := transmute(util.I32x8)simd.lanes_ge(
		util.vector3_wide_dot(triangle_normal, local_triangle_center), util.F32x8(0),
	);
	hull_inside_edge_planes :=
		transmute(util.I32x8)simd.lanes_le(util.vector3_wide_dot(edge_plane_ab, triangle_a_local), util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(util.vector3_wide_dot(edge_plane_bc, triangle_b_local), util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(util.vector3_wide_dot(edge_plane_ca, triangle_c_local), util.F32x8(0));
	inactive |= hull_below_plane & hull_inside_edge_planes;
	hull_epsilon_scale: util.F32x8;
	hull_support_bundle: util.Vector3_Wide;
	negated_triangle_normal := util.vector3_wide_negate(triangle_normal);
	for lane in 0 ..< pair_count
	{
		hull := hulls.hulls[lane];
		if convex_hull_validate(hull) != .Ok
		{
			return {}, .Invalid_Description;
		}
		point := convex_hull_get_point(hull, 0);
		hull_epsilon_scale = simd.replace(
			hull_epsilon_scale, lane, (abs(point.x) + abs(point.y) + abs(point.z)) * (1.0 / 3.0),
		);
		support, support_status := convex_hull_support(
			hull, util.vector3_wide_read_slot(negated_triangle_normal, lane),
		);
		if support_status != .Ok
		{
			return {}, support_status;
		}
		util.vector3_wide_write_slot(&hull_support_bundle, lane, support);
	}
	epsilon_scale := simd.min(epsilon_scale_triangle, hull_epsilon_scale);
	support_from_triangle_center := util.vector3_wide_subtract(hull_support_bundle, local_triangle_center);
	triangle_face_depth := util.vector3_wide_dot(support_from_triangle_center, negated_triangle_normal);
	extreme_ab := util.vector3_wide_dot(
		edge_plane_ab,
		util.vector3_wide_subtract(triangle_a_local, hull_support_bundle)
	);
	extreme_bc := util.vector3_wide_dot(
		edge_plane_bc,
		util.vector3_wide_subtract(triangle_b_local, hull_support_bundle)
	);
	extreme_ca := util.vector3_wide_dot(
		edge_plane_ca,
		util.vector3_wide_subtract(triangle_c_local, hull_support_bundle)
	);
	triangle_normal_minimal := ~hull_below_plane & hull_inside_edge_planes &
		transmute(util.I32x8)simd.lanes_le(extreme_ab, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(extreme_bc, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(extreme_ca, util.F32x8(0));
	center_distance := util.vector3_wide_length(local_triangle_center);
	initial_normal := util.vector3_wide_scale(local_triangle_center, simd.div(util.F32x8(1), center_distance));
	initial_normal = util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(center_distance, util.F32x8(1e-10)),
		{y=util.F32x8(1)}, initial_normal,
	);
	depth_threshold := simd.neg(speculative_margin);
	depth_context := Triangle_Hull_Batch_Depth_Context_Wide{
		hulls=hulls, triangles=triangle, local_triangle_center=local_triangle_center,
	};
	refined_depth, refined_normal, refined_closest, refine_status := depth_refiner_find_minimum_depth_wide(
		triangle_hull_batch_depth_support_wide, &depth_context, initial_normal,
		inactive | triangle_normal_minimal,
		simd.mul(util.F32x8(1e-4), epsilon_scale), depth_threshold, {}, {},
	);
	if refine_status != .Ok
	{
		return {}, refine_status;
	}
	depth := util.wide_select_f32(triangle_normal_minimal, triangle_face_depth, refined_depth);
	local_normal := util.vector3_wide_select(triangle_normal_minimal, negated_triangle_normal, refined_normal);
	closest_on_hull := util.vector3_wide_select(triangle_normal_minimal, hull_support_bundle, refined_closest);
	triangle_normal_dot_local_normal := util.vector3_wide_dot(triangle_normal, local_normal);
	inactive |= transmute(util.I32x8)simd.lanes_gt(
		triangle_normal_dot_local_normal, util.F32x8(-TRIANGLE_BACKFACE_REJECTION_THRESHOLD),
	) | transmute(util.I32x8)simd.lanes_lt(depth, depth_threshold);
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		triangle_lane := Triangle{
			a=util.vector3_wide_read_slot(triangle.a, lane),
			b=util.vector3_wide_read_slot(triangle.b, lane),
			c=util.vector3_wide_read_slot(triangle.c, lane),
		};
		lane_manifold, lane_status := collision_triangle_hull_build_manifold(
			hulls.hulls[lane], pose_a, pose_b, triangle_lane,
			util.vector3_wide_read_slot(local_triangle_center, lane),
			util.vector3_wide_read_slot(triangle_normal, lane),
			util.vector3_wide_read_slot(local_normal, lane),
			util.vector3_wide_read_slot(closest_on_hull, lane),
			util.matrix3x3_wide_read_slot(world_r_b, lane),
			simd.extract(epsilon_scale, lane), simd.extract(depth_threshold, lane),
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
