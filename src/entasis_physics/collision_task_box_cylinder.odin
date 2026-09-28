package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

collision_refine_pair_normal :: proc "contextless" (
	shape_a: rawptr, type_a: int, pose_a: Rigid_Pose,
	shape_b: rawptr, type_b: int, pose_b: Rigid_Pose,
	convergence_threshold, fallback_distance_squared, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (depth: f32, normal, closest_on_b: util.Vector3, status: Physics_Status)
{
	view_a, view_b, view_status := collision_pair_views(
		shape_a, shape_b, type_a, type_b, pose_a, pose_b, shapes,
	);
	if view_status != .Ok
	{
		return 0, {}, {}, view_status;
	}
	normal = util.vector3_subtract(pose_a.position, pose_b.position);
	length_squared := util.vector3_length_squared(normal);
	if length_squared < fallback_distance_squared
	{
		normal = util.quaternion_transform_unit_y(pose_b.orientation);
	}
	else
	{
		normal = util.vector3_scale(normal, 1 / math.sqrt(length_squared));
	}
	// the refiner's first support point comes from the manifold's B shape. its witness is
	// therefore already a point on B, matching the contact offset convention
	return depth_refiner_find_minimum_depth(
		view_b, view_a, normal, convergence_threshold,
		-speculative_margin, shapes,
	);
}
collision_intersect_line_circle :: proc "contextless" (
	line_position, line_direction: util.Vector2, radius: f32,
) -> (t_min, t_max: f32, intersected: Reference_State)
{
	a := max(f32(2e-38), util.vector2_dot(line_direction, line_direction));
	inverse_a := 1 / a;
	b := util.vector2_dot(line_position, line_direction);
	c := util.vector2_dot(line_position, line_position) - radius * radius;
	discriminant := b * b - a * c;
	intersected = .Missing;
	if discriminant >= 0
	{
		intersected = .Present;
	}
	t_offset := math.sqrt(max(f32(0), discriminant)) * inverse_a;
	t_base := -b * inverse_a;
	t_min = t_base - t_offset;
	t_max = t_base + t_offset;
	return;
}
collision_project_onto_cap_b :: proc "contextless" (
	cap_center_y, inverse_normal_y: f32, local_normal, point: util.Vector3,
) -> util.Vector2
{
	t := (point.y - cap_center_y) * inverse_normal_y;
	return {point.x - local_normal.x * t, point.z - local_normal.z * t};
}
collision_project_onto_cap_a :: proc "contextless" (
	cap_center_a: util.Vector3, orientation_a: util.Matrix3x3,
	inverse_normal_dot_a_y: f32, local_normal, point: util.Vector3,
) -> util.Vector2
{
	t := util.vector3_dot(util.vector3_subtract(cap_center_a, point), orientation_a.y) *
		inverse_normal_dot_a_y;
	projected := util.vector3_add(point, util.vector3_scale(local_normal, t));
	from_center := util.vector3_subtract(projected, cap_center_a);
	return {util.vector3_dot(from_center, orientation_a.x), util.vector3_dot(from_center, orientation_a.z)};
}
collision_add_edge_circle_candidates :: proc "contextless" (
	candidates: ^[MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar,
	candidate_count: ^int, edge_start, edge_offset: util.Vector2,
	t_min, t_max: f32, intersected: Reference_State, edge_id: i32,
) -> Physics_Status
{
	if candidates == nil || candidate_count == nil
	{
		return .Invalid_Argument;
	}
	if intersected == .Missing
	{
		return .Ok;
	}
	if t_min < t_max && t_min > 0
	{
		if candidate_count^ >= len(candidates^)
		{
			return .Capacity_Missing;
		}
		point := util.vector2_add(edge_start, util.vector2_scale(edge_offset, t_min));
		candidates[candidate_count^] = {x=point.x, y=point.y, feature_id=edge_id};
		candidate_count^ += 1;
	}
	if t_max > 0
	{
		if candidate_count^ >= len(candidates^)
		{
			return .Capacity_Missing;
		}
		point := util.vector2_add(edge_start, util.vector2_scale(edge_offset, t_max));
		candidates[candidate_count^] = {x=point.x, y=point.y, feature_id=edge_id + 4};
		candidate_count^ += 1;
	}
	return .Ok;
}
collision_generate_cylinder_interior_points :: proc "contextless" (
	cylinder: Cylinder, local_normal, closest_on_cylinder: util.Vector3,
) -> [4]util.Vector2
{
	parallel_weight := max(f32(0), min(f32(1), (abs(local_normal.y) - 0.9999) / 0.00005));
	extreme_weight := 1 - parallel_weight;
	scaled_radius := parallel_weight * cylinder.radius;
	points := [4]util.Vector2{{cylinder.radius, 0}, {-cylinder.radius, 0}, {0, cylinder.radius}, {0, -cylinder.radius}};
	if abs(closest_on_cylinder.x) > abs(closest_on_cylinder.z)
	{
		index := 0;
		if closest_on_cylinder.x <= 0
		{
			index = 1;
		}
		sign := f32(1);
		if index == 1
		{
			sign = -1;
		}
		points[index] = {
			extreme_weight * closest_on_cylinder.x + scaled_radius * sign,
			extreme_weight * closest_on_cylinder.z,
		};
	}
	else
	{
		index := 2;
		if closest_on_cylinder.z <= 0
		{
			index = 3;
		}
		sign := f32(1);
		if index == 3
		{
			sign = -1;
		}
		points[index] = {
			extreme_weight * closest_on_cylinder.x,
			extreme_weight * closest_on_cylinder.z + scaled_radius * sign,
		};
	}
	return points;
}
collision_manifold_push_contacts :: proc "contextless" (
	manifold: ^Convex_Contact_Manifold,
)
{
	if manifold == nil
	{
		return;
	}
	for contact_index in 0 ..< manifold.count
	{
		manifold.contacts[contact_index].offset = util.vector3_add(
			manifold.contacts[contact_index].offset,
			util.vector3_scale(manifold.normal, manifold.contacts[contact_index].depth),
		);
	}
}
collision_manifold_add_local_b :: proc "contextless" (
	manifold: ^Convex_Contact_Manifold, local_contact: util.Vector3,
	depth: f32, feature_id: i32, world_r_b: util.Matrix3x3,
) -> Physics_Status
{
	if manifold == nil
	{
		return .Invalid_Argument;
	}
	if manifold.count >= len(manifold.contacts)
	{
		return .Capacity_Missing;
	}
	manifold.contacts[manifold.count] = {
		offset=util.vector3_add(util.matrix3x3_transform(local_contact, world_r_b), manifold.offset_b),
		depth=depth,
		feature_id=feature_id,
	};
	manifold.count += 1;
	return .Ok;
}
collision_box_cylinder_build_manifold :: proc "contextless" (
	a: Box, b: Cylinder, pose_a, pose_b: Rigid_Pose,
	world_normal, world_closest_on_b: util.Vector3,
	epsilon_scale, depth_threshold: f32,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	world_r_a := util.matrix3x3_from_quaternion(pose_a.orientation);
	world_r_b := util.matrix3x3_from_quaternion(pose_b.orientation);
	r_a := util.matrix3x3_multiply(world_r_a, util.matrix3x3_transpose(world_r_b));
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	local_offset_b := util.matrix3x3_transform_transpose(offset_b, world_r_b);
	local_offset_a := util.vector3_negate(local_offset_b);
	local_normal := util.matrix3x3_transform_transpose(world_normal, world_r_b);
	closest_on_b := util.matrix3x3_transform_transpose(
		util.vector3_subtract(world_closest_on_b, pose_b.position), world_r_b,
	);

	local_normal_in_a := util.matrix3x3_transform_transpose(local_normal, r_a);
	abs_normal_in_a := util.vector3_abs(local_normal_in_a);
	face_axis := 2;
	if abs_normal_in_a.x > abs_normal_in_a.y && abs_normal_in_a.x > abs_normal_in_a.z
	{
		face_axis = 0;
	}
	else if abs_normal_in_a.y > abs_normal_in_a.z
	{
		face_axis = 1;
	}
	box_face_normal := r_a.z;
	box_face_x := r_a.x;
	box_face_y := r_a.y;
	box_face_half_width := a.half_width;
	box_face_half_height := a.half_height;
	box_face_normal_offset := a.half_length;
	negate_face := local_normal_in_a.z > 0;
	if face_axis == 0
	{
		box_face_normal = r_a.x;
		box_face_x = r_a.y;
		box_face_y = r_a.z;
		box_face_half_width = a.half_height;
		box_face_half_height = a.half_length;
		box_face_normal_offset = a.half_width;
		negate_face = local_normal_in_a.x > 0;
	}
	else if face_axis == 1
	{
		box_face_normal = r_a.y;
		box_face_x = r_a.z;
		box_face_y = r_a.x;
		box_face_half_width = a.half_length;
		box_face_half_height = a.half_width;
		box_face_normal_offset = a.half_height;
		negate_face = local_normal_in_a.y > 0;
	}
	if negate_face
	{
		box_face_normal = util.vector3_negate(box_face_normal);
		box_face_x = util.vector3_negate(box_face_x);
		box_face_y = util.vector3_negate(box_face_y);
	}
	box_face_center := util.vector3_add(local_offset_a, util.vector3_scale(box_face_normal, box_face_normal_offset));
	box_face_x_offset := util.vector3_scale(box_face_x, box_face_half_width);
	box_face_y_offset := util.vector3_scale(box_face_y, box_face_half_height);
	v_00 := util.vector3_subtract(util.vector3_subtract(box_face_center, box_face_x_offset), box_face_y_offset);
	v_11 := util.vector3_add(util.vector3_add(box_face_center, box_face_x_offset), box_face_y_offset);
	cap_center_y := b.half_length;
	if local_normal.y < 0
	{
		cap_center_y = -cap_center_y;
	}
	inverse_face_normal_dot_normal := 1 / util.vector3_dot(box_face_normal, local_normal);

	if abs(local_normal.y) > 0.70710678118
	{
		v_01 := util.vector3_add(util.vector3_subtract(box_face_center, box_face_x_offset), box_face_y_offset);
		v_10 := util.vector3_subtract(util.vector3_add(box_face_center, box_face_x_offset), box_face_y_offset);
		inverse_normal_y := 1 / local_normal.y;
		p_00 := collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, v_00);
		p_01 := collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, v_01);
		p_10 := collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, v_10);
		p_11 := collision_project_onto_cap_b(cap_center_y, inverse_normal_y, local_normal, v_11);
		edge_00_10 := util.vector2_subtract(p_10, p_00);
		edge_10_11 := util.vector2_subtract(p_11, p_10);
		edge_11_01 := util.vector2_subtract(p_01, p_11);
		edge_01_00 := util.vector2_subtract(p_00, p_01);
		starts := [4]util.Vector2{p_00, p_01, p_10, p_11};
		edges := [4]util.Vector2{edge_00_10, edge_01_00, edge_10_11, edge_11_01};
		candidates: [MAXIMUM_MANIFOLD_CANDIDATE_COUNT]Manifold_Candidate_Scalar;
		candidate_count := 0;
		for edge_index in 0 ..< 4
		{
			t_min, t_max, intersected := collision_intersect_line_circle(
				starts[edge_index],
				edges[edge_index],
				b.radius
			);
			t_min = min(f32(1), max(f32(0), t_min));
			t_max = min(f32(1), max(f32(0), t_max));
			add_status := collision_add_edge_circle_candidates(
				&candidates, &candidate_count, starts[edge_index], edges[edge_index],
				t_min, t_max, intersected, i32(edge_index),
			);
			if add_status != .Ok
			{
				return {}, add_status;
			}
		}
		interior_points := collision_generate_cylinder_interior_points(b, local_normal, closest_on_b);
		edge_00_10_plane_0 := p_00.x * edge_00_10.y - p_00.y * edge_00_10.x;
		edge_00_10_plane_1 := p_01.x * edge_00_10.y - p_01.y * edge_00_10.x;
		edge_10_11_plane_0 := p_10.x * edge_10_11.y - p_10.y * edge_10_11.x;
		edge_10_11_plane_1 := p_00.x * edge_10_11.y - p_00.y * edge_10_11.x;
		edge_00_10_min := min(edge_00_10_plane_0, edge_00_10_plane_1);
		edge_00_10_max := max(edge_00_10_plane_0, edge_00_10_plane_1);
		edge_10_11_min := min(edge_10_11_plane_0, edge_10_11_plane_1);
		edge_10_11_max := max(edge_10_11_plane_0, edge_10_11_plane_1);
		for interior_index in 0 ..< 4
		{
			point := interior_points[interior_index];
			dot_00_10 := point.x * edge_00_10.y - point.y * edge_00_10.x;
			dot_10_11 := point.x * edge_10_11.y - point.y * edge_10_11.x;
			if dot_00_10 >= edge_00_10_min && dot_00_10 <= edge_00_10_max &&
				dot_10_11 >= edge_10_11_min && dot_10_11 <= edge_10_11_max
			{
				candidates[candidate_count] = {x=point.x, y=point.y, feature_id=i32(8 + interior_index)};
				candidate_count += 1;
			}
		}
		return manifold_candidate_reduce(
			&candidates, candidate_count, box_face_normal, inverse_face_normal_dot_normal,
			box_face_center, {0, cap_center_y, 0}, {1, 0, 0}, {0, 0, 1},
			epsilon_scale, depth_threshold, world_r_b, offset_b, world_normal,
		);
	}

	edge_normal_x := util.vector3_cross(box_face_x, local_normal);
	edge_normal_y := util.vector3_cross(box_face_y, local_normal);
	v_00_to_side := util.Vector3{closest_on_b.x - v_00.x, -v_00.y, closest_on_b.z - v_00.z};
	v_11_to_side := util.Vector3{closest_on_b.x - v_11.x, -v_11.y, closest_on_b.z - v_11.z};
	t_x_min := -f32(math.F32_MAX);
	t_x_max := f32(math.F32_MAX);
	t_y_min := -f32(math.F32_MAX);
	t_y_max := f32(math.F32_MAX);
	lower_threshold :: f32(0.01 * 0.01);
	upper_threshold :: f32(0.02 * 0.02);
	if edge_normal_x.y != 0
	{
		t_0 := -util.vector3_dot(edge_normal_x, v_00_to_side) / edge_normal_x.y;
		t_1 := -util.vector3_dot(edge_normal_x, v_11_to_side) / edge_normal_x.y;
		length_squared := util.vector3_length_squared(edge_normal_x);
		unrestrict := max(
			f32(0),
			min(
			f32(1),
			(upper_threshold - edge_normal_x.y * edge_normal_x.y / length_squared) / (upper_threshold - lower_threshold)
		)
		);
		regular := 1 - unrestrict;
		t_x_min = unrestrict * -b.half_length + regular * min(t_0, t_1);
		t_x_max = unrestrict * b.half_length + regular * max(t_0, t_1);
	}
	if edge_normal_y.y != 0
	{
		t_0 := -util.vector3_dot(edge_normal_y, v_00_to_side) / edge_normal_y.y;
		t_1 := -util.vector3_dot(edge_normal_y, v_11_to_side) / edge_normal_y.y;
		length_squared := util.vector3_length_squared(edge_normal_y);
		unrestrict := max(
			f32(0),
			min(
			f32(1),
			(upper_threshold - edge_normal_y.y * edge_normal_y.y / length_squared) / (upper_threshold - lower_threshold)
		)
		);
		regular := 1 - unrestrict;
		t_y_min = unrestrict * -b.half_length + regular * min(t_0, t_1);
		t_y_max = unrestrict * b.half_length + regular * max(t_0, t_1);
	}
	t_min := min(b.half_length, max(-b.half_length, max(t_x_min, t_y_min)));
	t_max := min(b.half_length, max(-b.half_length, min(t_x_max, t_y_max)));
	manifold := Convex_Contact_Manifold{offset_b=offset_b, normal=world_normal};
	contacts := [2]util.Vector3{{closest_on_b.x, t_min, closest_on_b.z}, {closest_on_b.x, t_max, closest_on_b.z}};
	for contact_index in 0 ..< 2
	{
		if contact_index == 1 && t_max <= t_min
		{
			continue;
		}
		depth := util.vector3_dot(util.vector3_subtract(contacts[contact_index], box_face_center), box_face_normal) *
			inverse_face_normal_dot_normal;
		if depth < depth_threshold
		{
			continue;
		}
		add_status := collision_manifold_add_local_b(
			&manifold,
			contacts[contact_index],
			depth,
			i32(contact_index),
			world_r_b
		);
		if add_status != .Ok
		{
			return {}, add_status;
		}
	}
	return manifold, .Ok;
}
box_cylinder_test_source :: proc "contextless" (
	a: Box, b: Cylinder, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if box_validate(a) != .Ok || cylinder_validate(b) != .Ok || speculative_margin < 0 || shapes == nil
	{
		return {}, .Invalid_Description;
	}
	shape_a := a;
	shape_b := b;
	epsilon_scale := min(max(a.half_width, max(a.half_height, a.half_length)), max(b.radius, b.half_length));
	depth, normal, closest_on_b, refine_status := collision_refine_pair_normal(
		&shape_a, BOX_TYPE_ID, pose_a, &shape_b, CYLINDER_TYPE_ID, pose_b,
		epsilon_scale * 1e-6, 1e-20, speculative_margin, shapes,
	);
	if refine_status != .Ok
	{
		return {}, refine_status;
	}
	if depth < -speculative_margin
	{
		return {offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}, .Ok;
	}
	return collision_box_cylinder_build_manifold(
		a, b, pose_a, pose_b, normal, closest_on_b, epsilon_scale, -speculative_margin,
	);
}
collision_box_support_world_wide :: proc "contextless" (
	box: Box_Wide, orientation: util.Matrix3x3_Wide, position, direction: util.Vector3_Wide,
) -> util.Vector3_Wide
{
	local_direction := util.matrix3x3_wide_transform_transposed(direction, orientation);
	local_support := util.Vector3_Wide{
		x=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_ge(
			local_direction.x,
			util.F32x8(0)
		), box.half_width, simd.neg(box.half_width),
		),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_ge(
			local_direction.y,
			util.F32x8(0)
		), box.half_height, simd.neg(box.half_height),
		),
		z=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_ge(
			local_direction.z,
			util.F32x8(0)
		), box.half_length, simd.neg(box.half_length),
		),
	};
	return util.vector3_wide_add(position, util.matrix3x3_wide_transform(local_support, orientation));
}
Box_Cylinder_Depth_Context_Wide :: struct
{
	box:                  Box_Wide,
	cylinder:             Cylinder_Wide,
	box_orientation:      util.Matrix3x3_Wide,
	cylinder_orientation: util.Matrix3x3_Wide,
	offset_b:             util.Vector3_Wide,
}
box_cylinder_depth_support_wide :: proc "contextless" (
	user_context: rawptr, direction: util.Vector3_Wide, terminated_lanes: util.I32x8,
) -> (support, support_on_b: util.Vector3_Wide, status: Physics_Status)
{
	depth_context := (^Box_Cylinder_Depth_Context_Wide)(user_context);
	if depth_context == nil
	{
		return {}, {}, .Invalid_Argument;
	}
	support_on_b = collision_cylinder_support_world_wide(
		depth_context.cylinder, depth_context.cylinder_orientation, depth_context.offset_b, direction,
	);
	support_on_a := collision_box_support_world_wide(
		depth_context.box, depth_context.box_orientation, {}, util.vector3_wide_negate(direction),
	);
	return util.vector3_wide_subtract(support_on_b, support_on_a), support_on_b, .Ok;
}
box_cylinder_test_wide :: proc "contextless" (
	a: Box_Wide, b: Cylinder_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, pair_count: int,
) -> (Convex_4_Contact_Manifold_Wide, Physics_Status)
{
	active, status := collision_active_lane_mask(pair_count);
	if status != .Ok
	{
		return {}, status;
	}
	box_orientation := util.matrix3x3_wide_from_quaternion(orientation_a);
	cylinder_orientation := util.matrix3x3_wide_from_quaternion(orientation_b);
	epsilon_scale := simd.min(
		simd.max(a.half_width, simd.max(a.half_height, a.half_length)), simd.max(b.radius, b.half_length),
	);
	depth_context := Box_Cylinder_Depth_Context_Wide{
		box=a, cylinder=b, box_orientation=box_orientation,
		cylinder_orientation=cylinder_orientation, offset_b=offset_b,
	};
	depth, normal, closest_on_b, depth_status := depth_refiner_find_minimum_depth_wide(
		box_cylinder_depth_support_wide, &depth_context,
		collision_depth_initial_normal_wide(offset_b, cylinder_orientation.y, 1e-10), ~active,
		simd.mul(epsilon_scale, util.F32x8(1e-6)),
		simd.neg(speculative_margin), {}, {},
	);
	if depth_status != .Ok
	{
		return {}, depth_status;
	}
	inactive := ~active | transmute(util.I32x8)simd.lanes_lt(depth, simd.neg(speculative_margin));
	manifold: Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< pair_count
	{
		if simd.extract(inactive, lane) < 0
		{
			continue;
		}
		box := Box{
			simd.extract(a.half_width, lane),
			simd.extract(a.half_height, lane),
			simd.extract(a.half_length, lane),
		};
		cylinder := Cylinder{simd.extract(b.radius, lane), simd.extract(b.half_length, lane)};
		pose_a := Rigid_Pose{orientation=util.quaternion_wide_read_slot(orientation_a, lane)};
		pose_b := Rigid_Pose{
			position=util.vector3_wide_read_slot(offset_b, lane),
			orientation=util.quaternion_wide_read_slot(orientation_b, lane),
		};
		lane_normal := util.vector3_wide_read_slot(normal, lane);
		lane_closest := util.vector3_wide_read_slot(closest_on_b, lane);
		lane_manifold, lane_status := collision_box_cylinder_build_manifold(
			box, cylinder, pose_a, pose_b, lane_normal, lane_closest,
			simd.extract(epsilon_scale, lane), -simd.extract(speculative_margin, lane),
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
