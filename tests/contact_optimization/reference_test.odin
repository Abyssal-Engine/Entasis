package contact_optimization_tests

import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Frozen corrected-source implementations. These are differential references,
// not golden values regenerated from the optimized output.
reference_manifold_candidate_wide_reduce :: proc "contextless" (
	candidates: ^[8]physics.Manifold_Candidate_Wide, raw_contact_count: util.I32x8, max_candidate_count: int,
	face_normal_a: util.Vector3_Wide, inverse_face_normal_dot_normal: util.F32x8,
	face_center_b_to_face_center_a, tangent_bx, tangent_by: util.Vector3_Wide,
	epsilon_scale, minimum_depth: util.F32x8, pair_count: int,
) -> ([4]physics.Manifold_Candidate_Wide, [4]util.I32x8)
{
	masked_contact_count := raw_contact_count;
	for lane in pair_count ..< util.PRODUCTION_LANE_COUNT
	{
		masked_contact_count = simd.replace(masked_contact_count, lane, 0);
	}
	visited_candidate_count := 0;
	for lane in 0 ..< pair_count
	{
		visited_candidate_count = max(visited_candidate_count, int(simd.extract(masked_contact_count, lane)));
	}
	visited_candidate_count = min(visited_candidate_count, max_candidate_count);
	dot_axis := util.vector3_wide_scale(face_normal_a, inverse_face_normal_dot_normal);
	negative_base_dot := util.vector3_wide_dot(face_center_b_to_face_center_a, dot_axis);
	x_dot := util.vector3_wide_dot(tangent_bx, dot_axis);
	y_dot := util.vector3_wide_dot(tangent_by, dot_axis);
	for candidate_index in 0 ..< visited_candidate_count
	{
		candidate := &candidates[candidate_index];
		candidate.depth = simd.sub(
			simd.add(simd.mul(candidate.x, x_dot), simd.mul(candidate.y, y_dot)), negative_base_dot,
		);
	}
	depth_accepted_candidate_count := 0;
	for candidate_index in 0 ..< visited_candidate_count
	{
		candidate_exists := physics.manifold_candidate_wide_exists(
			candidates[candidate_index], minimum_depth, masked_contact_count, candidate_index,
		);
		for lane in 0 ..< pair_count
		{
			if simd.extract(candidate_exists, lane) < 0
			{
				depth_accepted_candidate_count = candidate_index + 1;
				break;
			}
		}
	}
	return physics.manifold_candidate_wide_reduce_internal(
		candidates, masked_contact_count, depth_accepted_candidate_count, epsilon_scale, minimum_depth,
	);
}

reference_box_pair_test_wide_core :: proc "contextless" (
	a, b: physics.Box_Wide, speculative_margin: util.F32x8, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide, active: util.I32x8, pair_count: int,
	manifold: ^physics.Convex_4_Contact_Manifold_Wide,
)
{
	manifold.contact_0_exists = util.I32x8(0);
	manifold.contact_1_exists = util.I32x8(0);
	manifold.contact_2_exists = util.I32x8(0);
	manifold.contact_3_exists = util.I32x8(0);
	world_r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	world_r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	r_b := util.matrix3x3_wide_multiply_by_transpose(world_r_b, world_r_a);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, world_r_a);
	best_depth, local_normal := physics.collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.x);
	candidate_depth, candidate_normal := physics.collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.y);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, candidate_normal);
	candidate_depth, candidate_normal = physics.collision_box_pair_test_edge_edge_wide(a, b, local_offset_b, r_b, r_b.z);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, candidate_normal);

	abs_r_bx := util.vector3_wide_abs(r_b.x);
	abs_r_by := util.vector3_wide_abs(r_b.y);
	abs_r_bz := util.vector3_wide_abs(r_b.z);
	candidate_depth = simd.sub(
		simd.add(
		a.half_width,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.x),
		simd.add(simd.mul(b.half_height, abs_r_by.x), simd.mul(b.half_length, abs_r_bz.x))
	)
	),
		simd.abs(local_offset_b.x),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {x=util.F32x8(1)});
	candidate_depth = simd.sub(
		simd.add(
		a.half_height,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.y),
		simd.add(simd.mul(b.half_height, abs_r_by.y), simd.mul(b.half_length, abs_r_bz.y))
	)
	),
		simd.abs(local_offset_b.y),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {y=util.F32x8(1)});
	candidate_depth = simd.sub(
		simd.add(
		a.half_length,
		simd.add(
		simd.mul(b.half_width, abs_r_bx.z),
		simd.add(simd.mul(b.half_height, abs_r_by.z), simd.mul(b.half_length, abs_r_bz.z))
	)
	),
		simd.abs(local_offset_b.z),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, {z=util.F32x8(1)});

	b_local_offset_b := util.matrix3x3_wide_transform_transposed(local_offset_b, r_b);
	candidate_depth = simd.sub(
		simd.add(
		b.half_width,
		simd.add(
		simd.mul(a.half_width, abs_r_bx.x),
		simd.add(simd.mul(a.half_height, abs_r_bx.y), simd.mul(a.half_length, abs_r_bx.z))
	)
	),
		simd.abs(b_local_offset_b.x),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.x);
	candidate_depth = simd.sub(
		simd.add(
		b.half_height,
		simd.add(
		simd.mul(a.half_width, abs_r_by.x),
		simd.add(simd.mul(a.half_height, abs_r_by.y), simd.mul(a.half_length, abs_r_by.z))
	)
	),
		simd.abs(b_local_offset_b.y),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.y);
	candidate_depth = simd.sub(
		simd.add(
		b.half_length,
		simd.add(
		simd.mul(a.half_width, abs_r_bz.x),
		simd.add(simd.mul(a.half_height, abs_r_bz.y), simd.mul(a.half_length, abs_r_bz.z))
	)
	),
		simd.abs(b_local_offset_b.z),
	);
	physics.collision_box_pair_select_axis_wide(&best_depth, &local_normal, candidate_depth, r_b.z);

	minimum_depth := simd.neg(speculative_margin);
	allow_contacts := active & transmute(util.I32x8)simd.lanes_ge(best_depth, minimum_depth);
	active_contact_lane_count := 0;
	for lane in 0 ..< pair_count
	{
		if simd.extract(allow_contacts, lane) < 0
		{
			active_contact_lane_count += 1;
		}
	}
	if active_contact_lane_count == 0
	{
		return;
	}
	local_normal = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(util.vector3_wide_dot(local_normal, local_offset_b), util.F32x8(0)),
		local_normal,
	);
	manifold.normal = util.matrix3x3_wide_transform(local_normal, world_r_a);
	axes_a := [3]util.Vector3_Wide{world_r_a.x, world_r_a.y, world_r_a.z};
	axes_b := [3]util.Vector3_Wide{world_r_b.x, world_r_b.y, world_r_b.z};

	ax_dot := util.vector3_wide_dot(manifold.normal, axes_a[0]);
	ay_dot := util.vector3_wide_dot(manifold.normal, axes_a[1]);
	az_dot := util.vector3_wide_dot(manifold.normal, axes_a[2]);
	max_a_dot := simd.max(simd.abs(ax_dot), simd.max(simd.abs(ay_dot), simd.abs(az_dot)));
	use_ax := transmute(util.I32x8)simd.lanes_eq(max_a_dot, simd.abs(ax_dot));
	use_ay := transmute(util.I32x8)simd.lanes_eq(max_a_dot, simd.abs(ay_dot)) & ~use_ax;
	normal_a := util.vector3_wide_select(use_ay, axes_a[1], util.vector3_wide_select(use_ax, axes_a[0], axes_a[2]));
	tangent_ax := util.vector3_wide_select(use_ay, axes_a[0], util.vector3_wide_select(use_ax, axes_a[2], axes_a[1]));
	tangent_ay := util.vector3_wide_select(use_ay, axes_a[2], util.vector3_wide_select(use_ax, axes_a[1], axes_a[0]));
	half_span_ax := util.wide_select_f32(
		use_ax,
		a.half_length,
		util.wide_select_f32(use_ay, a.half_width, a.half_height)
	);
	half_span_ay := util.wide_select_f32(
		use_ax,
		a.half_height,
		util.wide_select_f32(use_ay, a.half_length, a.half_width)
	);
	half_span_az := util.wide_select_f32(
		use_ax,
		a.half_width,
		util.wide_select_f32(use_ay, a.half_height, a.half_length)
	);
	local_x_id, local_y_id, local_z_id := util.I32x8(1), util.I32x8(4), util.I32x8(16);
	axis_id_ax := physics.collision_wide_select_i32(
		use_ax,
		local_z_id,
		physics.collision_wide_select_i32(use_ay, local_x_id, local_y_id)
	);
	axis_id_ay := physics.collision_wide_select_i32(
		use_ax,
		local_y_id,
		physics.collision_wide_select_i32(use_ay, local_z_id, local_x_id)
	);
	axis_id_az := physics.collision_wide_select_i32(
		use_ax,
		local_x_id,
		physics.collision_wide_select_i32(use_ay, local_y_id, local_z_id)
	);

	bx_dot := util.vector3_wide_dot(manifold.normal, axes_b[0]);
	by_dot := util.vector3_wide_dot(manifold.normal, axes_b[1]);
	bz_dot := util.vector3_wide_dot(manifold.normal, axes_b[2]);
	max_b_dot := simd.max(simd.abs(bx_dot), simd.max(simd.abs(by_dot), simd.abs(bz_dot)));
	use_bx := transmute(util.I32x8)simd.lanes_eq(max_b_dot, simd.abs(bx_dot));
	use_by := transmute(util.I32x8)simd.lanes_eq(max_b_dot, simd.abs(by_dot)) & ~use_bx;
	normal_b := util.vector3_wide_select(use_by, axes_b[1], util.vector3_wide_select(use_bx, axes_b[0], axes_b[2]));
	tangent_bx := util.vector3_wide_select(use_by, axes_b[0], util.vector3_wide_select(use_bx, axes_b[2], axes_b[1]));
	tangent_by := util.vector3_wide_select(use_by, axes_b[2], util.vector3_wide_select(use_bx, axes_b[1], axes_b[0]));
	half_span_bx := util.wide_select_f32(
		use_bx,
		b.half_length,
		util.wide_select_f32(use_by, b.half_width, b.half_height)
	);
	half_span_by := util.wide_select_f32(
		use_bx,
		b.half_height,
		util.wide_select_f32(use_by, b.half_length, b.half_width)
	);
	half_span_bz := util.wide_select_f32(
		use_bx,
		b.half_width,
		util.wide_select_f32(use_by, b.half_height, b.half_length)
	);
	axis_id_bx := physics.collision_wide_select_i32(
		use_bx,
		local_z_id,
		physics.collision_wide_select_i32(use_by, local_x_id, local_y_id)
	);
	axis_id_by := physics.collision_wide_select_i32(
		use_bx,
		local_y_id,
		physics.collision_wide_select_i32(use_by, local_z_id, local_x_id)
	);
	axis_id_bz := physics.collision_wide_select_i32(
		use_bx,
		local_x_id,
		physics.collision_wide_select_i32(use_by, local_y_id, local_z_id)
	);

	calibration_dot_a := util.vector3_wide_dot(normal_a, manifold.normal);
	normal_a = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_gt(calibration_dot_a, util.F32x8(0)), normal_a,
	);
	calibration_dot_b := util.vector3_wide_dot(normal_b, manifold.normal);
	normal_b = util.vector3_wide_conditional_negate(
		transmute(util.I32x8)simd.lanes_lt(calibration_dot_b, util.F32x8(0)), normal_b,
	);
	face_center_a := util.vector3_wide_scale(normal_a, half_span_az);
	face_center_b := util.vector3_wide_add(util.vector3_wide_scale(normal_b, half_span_bz), offset_b);
	face_center_b_to_face_center_a := util.vector3_wide_subtract(face_center_a, face_center_b);
	edge_offset_ax := util.vector3_wide_scale(tangent_ay, half_span_ay);
	edge_offset_ay := util.vector3_wide_scale(tangent_ax, half_span_ax);
	vertex_a0 := util.vector3_wide_subtract(face_center_a, edge_offset_ax);
	vertex_a00 := util.vector3_wide_subtract(vertex_a0, edge_offset_ay);
	vertex_a1 := util.vector3_wide_add(face_center_a, edge_offset_ax);
	vertex_a11 := util.vector3_wide_add(vertex_a1, edge_offset_ay);
	epsilon_scale := simd.min(
		simd.max(half_span_ax, simd.max(half_span_ay, half_span_az)),
		simd.max(half_span_bx, simd.max(half_span_by, half_span_bz)),
	);
	three := util.I32x8(3);
	axis_z_edge_id_contribution := simd.mul(axis_id_bz, three);
	edge_id_bx0 := simd.add(simd.mul(axis_id_bx, util.I32x8(2)), simd.add(axis_id_by, axis_z_edge_id_contribution));
	edge_id_bx1 := simd.add(
		simd.mul(axis_id_bx, util.I32x8(2)),
		simd.add(simd.mul(axis_id_by, three), axis_z_edge_id_contribution)
	);
	edge_id_by0 := simd.add(axis_id_bx, simd.add(simd.mul(axis_id_by, util.I32x8(2)), axis_z_edge_id_contribution));
	edge_id_by1 := simd.add(
		simd.mul(axis_id_bx, three),
		simd.add(simd.mul(axis_id_by, util.I32x8(2)), axis_z_edge_id_contribution)
	);
	candidates: [8]physics.Manifold_Candidate_Wide = ---
		candidate_count := util.I32x8(0);
	initialized_candidate_count := 0;
	physics.collision_box_pair_create_edge_contacts_wide(
		face_center_b, tangent_bx, tangent_by, half_span_bx, half_span_by,
		vertex_a00, vertex_a11, tangent_ax, tangent_ay, manifold.normal,
		edge_id_bx0, edge_id_bx1, edge_id_by0, edge_id_by1, epsilon_scale,
		&candidates, &candidate_count, &initialized_candidate_count, pair_count, allow_contacts,
	);
	vertex_a01 := util.vector3_wide_add(vertex_a0, edge_offset_ay);
	vertex_a10 := util.vector3_wide_subtract(vertex_a1, edge_offset_ay);
	vertices := [4]util.Vector3_Wide{vertex_a00, vertex_a01, vertex_a10, vertex_a11};
	feature_ids := [4]util.I32x8{
		simd.neg(axis_id_az), simd.neg(simd.add(axis_id_az, axis_id_ay)),
		simd.neg(simd.add(axis_id_az, axis_id_ax)),
		simd.neg(simd.add(axis_id_az, simd.add(axis_id_ax, axis_id_ay))),
	};
	physics.collision_box_pair_add_a_vertices_wide(
		face_center_b, tangent_bx, tangent_by, half_span_bx, half_span_by, normal_b, manifold.normal,
		vertices, feature_ids, &candidates, &candidate_count, &initialized_candidate_count,
		pair_count, allow_contacts,
	);
	contacts, contact_exists := reference_manifold_candidate_wide_reduce(
		&candidates, candidate_count, initialized_candidate_count, normal_a,
		simd.div(util.F32x8(-1), simd.abs(calibration_dot_a)),
		face_center_b_to_face_center_a, tangent_bx, tangent_by, epsilon_scale, minimum_depth, pair_count,
	);
	manifold.offset_a_0 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[0].x),
		util.vector3_wide_scale(tangent_by, contacts[0].y)
	)
	);
	manifold.offset_a_1 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[1].x),
		util.vector3_wide_scale(tangent_by, contacts[1].y)
	)
	);
	manifold.offset_a_2 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[2].x),
		util.vector3_wide_scale(tangent_by, contacts[2].y)
	)
	);
	manifold.offset_a_3 = util.vector3_wide_add(
		face_center_b,
		util.vector3_wide_add(
		util.vector3_wide_scale(tangent_bx, contacts[3].x),
		util.vector3_wide_scale(tangent_by, contacts[3].y)
	)
	);
	manifold.depth_0, manifold.depth_1 = contacts[0].depth, contacts[1].depth;
	manifold.depth_2, manifold.depth_3 = contacts[2].depth, contacts[3].depth;
	manifold.feature_id_0, manifold.feature_id_1 = contacts[0].feature_id, contacts[1].feature_id;
	manifold.feature_id_2, manifold.feature_id_3 = contacts[2].feature_id, contacts[3].feature_id;
	manifold.contact_0_exists, manifold.contact_1_exists = contact_exists[0] & allow_contacts, contact_exists[1] & allow_contacts;
	manifold.contact_2_exists, manifold.contact_3_exists = contact_exists[2] & allow_contacts, contact_exists[3] & allow_contacts;
}

reference_narrow_phase_box_direct_test_group :: #force_no_inline proc "contextless" (
	direct_records: [^]physics.Narrow_Phase_Convex_Direct_Record,
	group_start, group_count: int, offset_b: ^util.Vector3_Wide,
	wide: ^physics.Convex_4_Contact_Manifold_Wide,
)
{
	tail := &direct_records[group_start + group_count - 1];
	a := physics.Box_Wide{
		half_width=util.F32x8((^physics.Box)(tail.shape_data_a).half_width),
		half_height=util.F32x8((^physics.Box)(tail.shape_data_a).half_height),
		half_length=util.F32x8((^physics.Box)(tail.shape_data_a).half_length),
	};
	b := physics.Box_Wide{
		half_width=util.F32x8((^physics.Box)(tail.shape_data_b).half_width),
		half_height=util.F32x8((^physics.Box)(tail.shape_data_b).half_height),
		half_length=util.F32x8((^physics.Box)(tail.shape_data_b).half_length),
	};
	offset_b^ = util.vector3_wide_broadcast(
		util.vector3_subtract(tail.pose_b.position, tail.pose_a.position),
	);
	orientation_a := util.quaternion_wide_broadcast(tail.pose_a.orientation);
	orientation_b := util.quaternion_wide_broadcast(tail.pose_b.orientation);
	speculative_margin := util.F32x8(tail.speculative_margin);
	for lane in 0 ..< group_count
	{
		record := &direct_records[group_start + lane];
		physics.box_wide_write_slot_trusted(&a, lane, (^physics.Box)(record.shape_data_a)^);
		physics.box_wide_write_slot_trusted(&b, lane, (^physics.Box)(record.shape_data_b)^);
		util.vector3_wide_write_slot(
			offset_b, lane,
			util.vector3_subtract(record.pose_b.position, record.pose_a.position),
		);
		util.quaternion_wide_write_slot(
			&orientation_a, lane, record.pose_a.orientation,
		);
		util.quaternion_wide_write_slot(
			&orientation_b, lane, record.pose_b.orientation,
		);
		speculative_margin = simd.replace(
			speculative_margin, lane, record.speculative_margin,
		);
	}
	active := transmute(util.I32x8)simd.lanes_lt(
		util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(group_count)),
	);
	reference_box_pair_test_wide_core(
		a, b, speculative_margin, offset_b^, orientation_a, orientation_b,
		active, group_count, wide,
	);
}
