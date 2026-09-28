// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

MAXIMUM_COLLISION_TASK_BATCH_SIZE :: 32;
Collision_Task_Batch :: struct
{
	pair_indices: [MAXIMUM_COLLISION_TASK_BATCH_SIZE]i32,
	count:        int,
}

Collision_Wide_Shape_Payload :: struct #raw_union
{
	sphere:   Sphere_Wide,
	capsule:  Capsule_Wide,
	box:      Box_Wide,
	triangle: Triangle_Wide,
	cylinder: Cylinder_Wide,
	hull:     Convex_Hull_Wide,
}

Collision_Convex_Wide_Bundle :: struct
{
	shape_a, shape_b:   [util.PRODUCTION_LANE_COUNT]rawptr,
	a, b:               Collision_Wide_Shape_Payload,
	offset_b:           util.Vector3_Wide,
	orientation_a:      util.Quaternion_Wide,
	orientation_b:      util.Quaternion_Wide,
	speculative_margin: util.F32x8,
	pair_indices:       [util.PRODUCTION_LANE_COUNT]i32,
	flip_mask:          util.I32x8,
	count:              int,
}
#assert(size_of(Collision_Wide_Shape_Payload) == size_of(Triangle_Wide));
#assert(size_of(Collision_Convex_Wide_Bundle) <= 1216);
collision_convex_wide_write_shape_trusted :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, type_id, lane: int, shape: rawptr, slot_a: Reference_State,
)
{
	if slot_a == .Present
	{
		bundle.shape_a[lane] = shape;
		switch type_id
		{
			case SPHERE_TYPE_ID:
			sphere_wide_write_slot_trusted(&bundle.a.sphere, lane, (^Sphere)(shape)^);
			case CAPSULE_TYPE_ID:
			capsule_wide_write_slot_trusted(&bundle.a.capsule, lane, (^Capsule)(shape)^);
			case BOX_TYPE_ID:
			box_wide_write_slot_trusted(&bundle.a.box, lane, (^Box)(shape)^);
			case TRIANGLE_TYPE_ID:
			triangle_wide_write_slot_trusted(&bundle.a.triangle, lane, (^Triangle)(shape)^);
			case CYLINDER_TYPE_ID:
			cylinder_wide_write_slot_trusted(&bundle.a.cylinder, lane, (^Cylinder)(shape)^);
			case CONVEX_HULL_TYPE_ID:
			bundle.a.hull.hulls[lane] = (^Convex_Hull)(shape);
		}
	}
	else
	{
		bundle.shape_b[lane] = shape;
		switch type_id
		{
			case SPHERE_TYPE_ID:
			sphere_wide_write_slot_trusted(&bundle.b.sphere, lane, (^Sphere)(shape)^);
			case CAPSULE_TYPE_ID:
			capsule_wide_write_slot_trusted(&bundle.b.capsule, lane, (^Capsule)(shape)^);
			case BOX_TYPE_ID:
			box_wide_write_slot_trusted(&bundle.b.box, lane, (^Box)(shape)^);
			case TRIANGLE_TYPE_ID:
			triangle_wide_write_slot_trusted(&bundle.b.triangle, lane, (^Triangle)(shape)^);
			case CYLINDER_TYPE_ID:
			cylinder_wide_write_slot_trusted(&bundle.b.cylinder, lane, (^Cylinder)(shape)^);
			case CONVEX_HULL_TYPE_ID:
			bundle.b.hull.hulls[lane] = (^Convex_Hull)(shape);
		}
	}
}

collision_batcher_gather_convex_bundle :: proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task,
	batch: ^Collision_Task_Batch, start, count: int, bundle: ^Collision_Convex_Wide_Bundle,
) -> Physics_Status
{
	if batcher == nil || task == nil || batch == nil || bundle == nil ||
	count <= 0 || count > util.PRODUCTION_LANE_COUNT || start < 0 || start + count > batch.count
	{
		return .Invalid_Argument;
	}
	bundle.count = count;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		source_lane := min(lane, count - 1);
		pair_index := batch.pair_indices[start + source_lane];
		if pair_index < 0 || int(pair_index) >= batcher.pair_count
		{
			return .Invalid_Description;
		}
		pair := &batcher.pairs.memory[pair_index];
		expected_a := pair.input.shape_data_a;
		expected_b := pair.input.shape_data_b;
		pose_a := pair.input.pose_a;
		pose_b := pair.input.pose_b;
		flip := i32(0);
		if pair.input.order == .Flipped
		{
			expected_a, expected_b = expected_b, expected_a;
			pose_a, pose_b = pose_b, pose_a;
			flip = -1;
		}
		bundle.flip_mask = simd.replace(bundle.flip_mask, lane, flip);
		collision_convex_wide_write_shape_trusted(
			bundle, int(task.shape_type_a), lane, expected_a, .Present,
		);
		collision_convex_wide_write_shape_trusted(
			bundle, int(task.shape_type_b), lane, expected_b, .Missing,
		);
		util.vector3_wide_write_slot(
			&bundle.offset_b, lane, util.vector3_subtract(pose_b.position, pose_a.position),
		);
		util.quaternion_wide_write_slot(&bundle.orientation_a, lane, pose_a.orientation);
		util.quaternion_wide_write_slot(&bundle.orientation_b, lane, pose_b.orientation);
		bundle.speculative_margin = simd.replace(
			bundle.speculative_margin, lane, pair.input.speculative_margin,
		);
		bundle.pair_indices[lane] = pair_index;
	}
	return .Ok;
}

collision_batcher_resolve_subpair_child :: proc "contextless" (
	batcher: ^Collision_Batcher,
	continuation: ^Collision_Batcher_Continuation,
	source_child_index: i32,
	slot_a: Reference_State,
	triangle: ^Triangle,
) -> (Collision_Child, Physics_Status)
{
	if batcher == nil || continuation == nil || triangle == nil || continuation.parent_pair_index < 0 ||
	int(continuation.parent_pair_index) >= batcher.pair_count
	{
		return {}, .Invalid_Argument;
	}
	pair := &batcher.pairs.memory[continuation.parent_pair_index];
	shape_index := pair.input.shape_a;
	pose := pair.input.pose_a;
	if (slot_a == .Present && continuation.flip == .Present) ||
	(slot_a == .Missing && continuation.flip == .Missing)
	{
		shape_index = pair.input.shape_b;
		pose = pair.input.pose_b;
	}
	type_id := int(typed_index_type(shape_index));
	shape, batch, resolve_status := shape_registry_resolve(batcher.shapes, shape_index);
	if resolve_status != .Ok
	{
		return {}, resolve_status;
	}
	if batch == nil
	{
		return {}, .Invalid_Description;
	}
	switch batch.metadata.batch_type
	{
		case .Convex:
		return {shape=shape, type_id=type_id, pose=pose, child_index=int(source_child_index)}, .Ok;
		case .Homogeneous_Compound:
		if type_id != MESH_TYPE_ID
		{
			return {}, .Invalid_Description;
		}
		child_triangle, child_pose, child_status := collision_mesh_child(
			(^Mesh)(shape), int(source_child_index), pose,
		);
		if child_status != .Ok
		{
			return {}, child_status;
		}
		triangle^ = child_triangle;
		return {
			shape=triangle,
			type_id=TRIANGLE_TYPE_ID,
			pose=child_pose,
			child_index=int(source_child_index),
			mesh_owner=(^Mesh)(shape),
			mesh_pose=pose,
		}, .Ok;
		case .Compound:
		if type_id != COMPOUND_TYPE_ID && type_id != BIG_COMPOUND_TYPE_ID
		{
			return {}, .Invalid_Description;
		}
		return collision_compound_child(
			shape, type_id, int(source_child_index), pose, batcher.shapes,
		);
	}
	return {}, .Invalid_Description;
}

collision_batcher_gather_subpair_bundle :: proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task,
	batch: ^Collision_Task_Batch, start, count: int, bundle: ^Collision_Convex_Wide_Bundle,
) -> Physics_Status
{
	if batcher == nil || task == nil || batch == nil || bundle == nil ||
	count <= 0 || count > util.PRODUCTION_LANE_COUNT || start < 0 || start + count > batch.count
	{
		return .Invalid_Argument;
	}
	bundle.count = count;
	// the first lane is active and its prepared data remains valid through padding
	triangle_a, triangle_b: Triangle;
	expected_a, expected_b: rawptr;
	orientation_a, orientation_b: util.Quaternion;
	offset_b: util.Vector3;
	speculative_margin: f32;
	subpair_index: i32;
	flip: i32;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if lane < count
		{
			subpair_index = batch.pair_indices[start + lane];
			if subpair_index < 0 || int(subpair_index) >= batcher.subpair_count
			{
				return .Invalid_Description;
			}
			subpair := collision_child_storage_subpair(batcher.children, int(subpair_index));
			if subpair.continuation_index < 0 || int(subpair.continuation_index) >= batcher.continuation_count
			{
				return .Invalid_Description;
			}
			continuation := &batcher.continuations.memory[subpair.continuation_index];
			child_a, child_a_status := collision_batcher_resolve_subpair_child(
				batcher, continuation, i32(u32(subpair.source_child_a) & 0x7fffffff), .Present, &triangle_a,
			);
			if child_a_status != .Ok
			{
				return child_a_status;
			}
			child_b, child_b_status := collision_batcher_resolve_subpair_child(
				batcher, continuation, subpair.source_child_b, .Missing, &triangle_b,
			);
			if child_b_status != .Ok
			{
				return child_b_status;
			}
			expected_a, expected_b = child_a.shape, child_b.shape;
			pose_a, pose_b := child_a.pose, child_b.pose;
			flip = 0;
			if subpair.source_child_a < 0
			{
				expected_a, expected_b = expected_b, expected_a;
				pose_a, pose_b = pose_b, pose_a;
				flip = -1;
			}
			offset_b = util.vector3_subtract(pose_b.position, pose_a.position);
			orientation_a, orientation_b = pose_a.orientation, pose_b.orientation;
			speculative_margin = batcher.pairs.memory[continuation.parent_pair_index].input.speculative_margin;
		}
		bundle.flip_mask = simd.replace(bundle.flip_mask, lane, flip);
		collision_convex_wide_write_shape_trusted(
			bundle, int(task.shape_type_a), lane, expected_a, .Present,
		);
		collision_convex_wide_write_shape_trusted(
			bundle, int(task.shape_type_b), lane, expected_b, .Missing,
		);
		util.vector3_wide_write_slot(&bundle.offset_b, lane, offset_b);
		util.quaternion_wide_write_slot(&bundle.orientation_a, lane, orientation_a);
		util.quaternion_wide_write_slot(&bundle.orientation_b, lane, orientation_b);
		bundle.speculative_margin = simd.replace(bundle.speculative_margin, lane, speculative_margin);
		bundle.pair_indices[lane] = subpair_index;
	}
	return .Ok;
}

collision_batcher_complete_child_record :: #force_inline proc "contextless" (
	batcher: ^Collision_Batcher, continuation_index, record_index: int,
	manifold: Convex_Contact_Manifold,
) -> Physics_Status
{
	if batcher == nil || continuation_index < 0 || continuation_index >= batcher.continuation_count ||
	record_index < 0 || record_index >= batcher.child_count
	{
		return .Invalid_Description;
	}
	continuation := &batcher.continuations.memory[continuation_index];
	record := collision_child_storage_header(batcher.children, record_index);
	stored_manifold := manifold;
	if batcher.procedures.child_pair_completed != nil
	{
		status := batcher.procedures.child_pair_completed(
			batcher.user_context,
			batcher.pairs.memory[continuation.parent_pair_index].pair_id,
			record.child_a, record.child_b,
			&stored_manifold,
		);
		if status != .Ok
		{
			return status;
		}
	}
	contact_count := int(stored_manifold.count);
	if contact_count < 0 || contact_count > MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return .Invalid_Description;
	}
	contact_capacity := int(batcher.children.length) * MAXIMUM_MANIFOLD_CONTACT_COUNT;
	if batcher.child_contact_count > contact_capacity - contact_count
	{
		return .Capacity_Missing;
	}
	record.contact_start = i32(batcher.child_contact_count);
	record.contact_count = i8(contact_count);
	record.normal = stored_manifold.normal;
	for contact_index in 0 ..< contact_count
	{
		contact := stored_manifold.contacts[contact_index];
		collision_child_storage_write_contact(batcher.children, record_index, contact_index, {
				offset=contact.offset,
				depth=contact.depth,
				feature_id=contact.feature_id,
		});
	}
	batcher.child_contact_count += contact_count;
	record.completed = .Present;
	return .Ok;
}

collision_batcher_store_subpair_lane :: proc "contextless" (
	batcher: ^Collision_Batcher, bundle: ^Collision_Convex_Wide_Bundle,
	lane: int, manifold: Convex_Contact_Manifold,
) -> Physics_Status
{
	if batcher == nil || bundle == nil || lane < 0 || lane >= bundle.count
	{
		return .Invalid_Argument;
	}
	subpair_index := bundle.pair_indices[lane];
	if subpair_index < 0 || int(subpair_index) >= batcher.subpair_count
	{
		return .Invalid_Description;
	}
	subpair := collision_child_storage_subpair(batcher.children, int(subpair_index));
	return collision_batcher_complete_child_record(
		batcher, int(subpair.continuation_index), int(subpair.child_record_index), manifold,
	);
}

collision_task_execute_sphere_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_pair_test_wide(
		bundle.a.sphere, bundle.b.sphere, bundle.speculative_margin, bundle.offset_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_sphere_capsule_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_capsule_test_wide(
		bundle.a.sphere, bundle.b.capsule, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_capsule_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_capsule_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_sphere_box_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_box_test_wide(
		bundle.a.sphere, bundle.b.box, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_box_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_box_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_sphere_triangle_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_triangle_test_wide(
		bundle.a.sphere, bundle.b.triangle, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_triangle_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_triangle_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_sphere_cylinder_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_cylinder_test_wide(
		bundle.a.sphere, bundle.b.cylinder, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_cylinder_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_cylinder_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_sphere_hull_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := sphere_convex_hull_test_wide(
		bundle.a.sphere, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_b, bundle.count,
	);
	result.kind = .One_Contact;
	result.one = wide;
	return status;
}

collision_task_execute_sphere_hull_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_sphere_hull_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_capsule_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := capsule_pair_test_wide(
		bundle.a.capsule, bundle.b.capsule, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Two_Contact;
	result.two = wide;
	return status;
}

collision_task_execute_capsule_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_capsule_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_capsule_box_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := capsule_box_test_wide(
		bundle.a.capsule, bundle.b.box, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Two_Contact;
	result.two = wide;
	return status;
}

collision_task_execute_capsule_box_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_capsule_box_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_capsule_triangle_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := capsule_triangle_test_wide(
		bundle.a.capsule, bundle.b.triangle, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Two_Contact;
	result.two = wide;
	return status;
}

collision_task_execute_capsule_triangle_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_capsule_triangle_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_capsule_cylinder_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := capsule_cylinder_test_wide(
		bundle.a.capsule, bundle.b.cylinder, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Two_Contact;
	result.two = wide;
	return status;
}

collision_task_execute_capsule_cylinder_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_capsule_cylinder_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_capsule_hull_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := capsule_convex_hull_test_wide(
		bundle.a.capsule, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Two_Contact;
	result.two = wide;
	return status;
}

collision_task_execute_capsule_hull_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_capsule_hull_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_box_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	result.kind = .Four_Contact;
	return box_pair_test_wide_into(
		bundle.a.box, bundle.b.box, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count, &result.four,
	);
}

collision_task_execute_box_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_box_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_box_triangle_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := box_triangle_test_wide(
		bundle.a.box, bundle.b.triangle, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_box_triangle_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_box_triangle_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_box_cylinder_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := box_cylinder_test_wide(
		bundle.a.box, bundle.b.cylinder, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_box_cylinder_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_box_cylinder_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_box_hull_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := box_convex_hull_test_wide(
		bundle.a.box, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_box_hull_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_box_hull_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_triangle_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := triangle_pair_test_wide(
		bundle.a.triangle, bundle.b.triangle, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_triangle_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_triangle_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_triangle_cylinder_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := triangle_cylinder_test_wide(
		bundle.a.triangle, bundle.b.cylinder, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_triangle_cylinder_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_triangle_cylinder_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_triangle_hull_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := triangle_convex_hull_test_wide(
		bundle.a.triangle, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_triangle_hull_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_triangle_hull_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_cylinder_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := cylinder_pair_test_wide(
		bundle.a.cylinder, bundle.b.cylinder, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_cylinder_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_cylinder_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_cylinder_hull_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := cylinder_convex_hull_test_wide(
		bundle.a.cylinder, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_cylinder_hull_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_cylinder_hull_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_hull_pair_wide_into :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	_ = shapes;
	wide, status := convex_hull_pair_test_wide(
		bundle.a.hull, bundle.b.hull, bundle.speculative_margin,
		bundle.offset_b, bundle.orientation_a, bundle.orientation_b, bundle.count,
	);
	result.kind = .Four_Contact;
	result.four = wide;
	return status;
}

collision_task_execute_hull_pair_wide :: proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status)
{
	result: Collision_Wide_Manifold_Result = ---
	status := collision_task_execute_hull_pair_wide_into(bundle, shapes, &result);
	return result, status;
}

collision_task_execute_legacy_wide_into :: #force_no_inline proc "contextless" (
	test: Collision_Wide_Test_Proc, bundle: ^Collision_Convex_Wide_Bundle,
	shapes: ^Shape_Registry, result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status
{
	if test == nil || bundle == nil || result == nil
	{
		return .Invalid_Argument;
	}
	legacy_result, status := test(bundle, shapes);
	result^ = legacy_result;
	return status;
}

collision_wide_manifold_apply_flip_mask :: proc "contextless" (
	result: ^Collision_Wide_Manifold_Result, offset_b: ^util.Vector3_Wide, flip_mask: util.I32x8,
) -> Physics_Status
{
	if result == nil || offset_b == nil
	{
		return .Invalid_Argument;
	}
	switch result.kind
	{
		case .One_Contact:
		return convex_1_manifold_wide_apply_flip_mask(&result.one, offset_b, flip_mask);
		case .Two_Contact:
		return convex_2_manifold_wide_apply_flip_mask(&result.two, offset_b, flip_mask);
		case .Four_Contact:
		return convex_4_manifold_wide_apply_flip_mask(&result.four, offset_b, flip_mask);
	}
	return .Invalid_Description;
}

collision_wide_manifold_read_lane :: proc "contextless" (
	result: ^Collision_Wide_Manifold_Result, offset_b: util.Vector3_Wide, lane: int,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	manifold: Convex_Contact_Manifold;
	status := collision_wide_manifold_read_lane_into(result, offset_b, lane, &manifold);
	return manifold, status;
}

collision_wide_manifold_read_lane_into :: proc "contextless" (
	result: ^Collision_Wide_Manifold_Result, offset_b: util.Vector3_Wide, lane: int,
	destination: ^Convex_Contact_Manifold,
) -> Physics_Status
{
	if result == nil || destination == nil
	{
		return .Invalid_Argument;
	}
	switch result.kind
	{
		case .One_Contact:
		return convex_1_manifold_wide_read_lane_into(&result.one, offset_b, lane, destination);
		case .Two_Contact:
		return convex_2_manifold_wide_read_lane_into(&result.two, offset_b, lane, destination);
		case .Four_Contact:
		return convex_4_manifold_wide_read_lane_into(&result.four, offset_b, lane, destination);
	}
	return .Invalid_Description;
}

collision_batcher_execute_convex_task_batch_kernel :: proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, batch: ^Collision_Task_Batch,
	$dispatch: Collision_Task_Dispatch,
) -> Physics_Status
{
	if batcher == nil || task == nil || batch == nil || batch.count <= 0 ||
	batch.count > int(task.batch_size) || task.kind != .Convex
	{
		return .Invalid_Argument;
	}
	when dispatch == .Native
	{
		if task.dispatch == .Contextual
		{
			return #force_no_inline collision_batcher_execute_convex_task_batch_kernel(batcher, task, batch, .Contextual);
		}
		if task.convex_wide_test_into == nil && task.convex_wide_test == nil
		{
			return .Invalid_Argument;
		}
	}
	when dispatch == .Contextual
	{
		binding: ^Contextual_Collision_Task_Binding = task.contextual;
		if task.dispatch != .Contextual || binding == nil || binding.wide_test == nil
		{
			return .Invalid_Argument;
		}
	}
	for start := 0; start < batch.count; start += util.PRODUCTION_LANE_COUNT
	{
		count := min(util.PRODUCTION_LANE_COUNT, batch.count - start);
		bundle: Collision_Convex_Wide_Bundle = ---
		gather_status := collision_batcher_gather_convex_bundle(
			batcher, task, batch, start, count, &bundle,
		);
		if gather_status != .Ok
		{
			return gather_status;
		}
		wide: Collision_Wide_Manifold_Result = ---
		wide_status: Physics_Status;
		when dispatch == .Contextual
		{
			wide_status = binding.wide_test(binding.user_context, &bundle, batcher.shapes, &wide);
		}
		else
		{
			if task.convex_wide_test_into != nil
			{
				wide_status = task.convex_wide_test_into(&bundle, batcher.shapes, &wide);
			}
			else
			{
				wide_status = collision_task_execute_legacy_wide_into(
					task.convex_wide_test, &bundle, batcher.shapes, &wide,
				);
			}
		}
		if wide_status != .Ok
		{
			return wide_status;
		}
		flip_status := collision_wide_manifold_apply_flip_mask(&wide, &bundle.offset_b, bundle.flip_mask);
		if flip_status != .Ok
		{
			return flip_status;
		}
		for lane in 0 ..< bundle.count
		{
			pair_index := bundle.pair_indices[lane];
			if pair_index < 0 || int(pair_index) >= batcher.pair_count
			{
				return .Invalid_Description;
			}
			pair := &batcher.pairs.memory[pair_index];
			pair.result.kind = .Convex;
			read_status := collision_wide_manifold_read_lane_into(
				&wide, bundle.offset_b, lane, &pair.result.convex,
			);
			if read_status != .Ok
			{
				return read_status;
			}
			pair.result_state = .Complete;
		}
	}
	return .Ok;
}

// preserve the established inlined convex route in collision_batcher_flush.
// optional hierarchical expansion stays in its separate non-inlined owner
collision_batcher_execute_task_batches :: #force_inline proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, first_pair_index: i32,
) -> Physics_Status
{
	if batcher == nil || task == nil || first_pair_index < 0 || int(first_pair_index) >= batcher.pair_count ||
	task.batch_size <= 0 || int(task.batch_size) > MAXIMUM_COLLISION_TASK_BATCH_SIZE
	{
		return .Invalid_Argument;
	}
	if task.kind != .Convex
	{
		if batcher.shapes.hierarchy != nil && batcher.shapes.hierarchy.count > 0
		{
			return collision_batcher_execute_hierarchy_batches(batcher, first_pair_index);
		}

		pair_index := first_pair_index;
		for pair_index >= 0
		{
			pair := &batcher.pairs.memory[pair_index];
			next_pair_index := pair.input.next_in_task;
			status := #force_inline collision_batcher_expand_compound_pair(batcher, int(pair_index));
			if status != .Ok
			{
				return status;
			}
			pair_index = next_pair_index;
		}
		return .Ok;
	}
	batch: Collision_Task_Batch;
	pair_index := first_pair_index;
	for pair_index >= 0
	{
		pair := &batcher.pairs.memory[pair_index];
		next_pair_index := pair.input.next_in_task;
		batch.pair_indices[batch.count] = pair_index;
		batch.count += 1;
		if batch.count == int(task.batch_size)
		{
			status := collision_batcher_execute_convex_task_batch(batcher, task, &batch);
			if status != .Ok
			{
				return status;
			}
			batch = {};
		}
		pair_index = next_pair_index;
	}
	if batch.count > 0
	{
		return collision_batcher_execute_convex_task_batch(batcher, task, &batch);
	}
	return .Ok;
}

collision_batcher_execute_subtask_batch_kernel :: proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, batch: ^Collision_Task_Batch,
	$dispatch: Collision_Task_Dispatch,
) -> Physics_Status
{
	if batcher == nil || task == nil || batch == nil || batch.count <= 0 ||
	batch.count > int(task.batch_size) || task.kind != .Convex
	{
		return .Invalid_Argument;
	}
	when dispatch == .Native
	{
		if task.dispatch == .Contextual
		{
			return #force_no_inline collision_batcher_execute_subtask_batch_kernel(batcher, task, batch, .Contextual);
		}
		if task.convex_wide_test_into == nil && task.convex_wide_test == nil
		{
			return .Invalid_Argument;
		}
	}
	when dispatch == .Contextual
	{
		binding: ^Contextual_Collision_Task_Binding = task.contextual;
		if task.dispatch != .Contextual || binding == nil || binding.wide_test == nil
		{
			return .Invalid_Argument;
		}
	}
	for start := 0; start < batch.count; start += util.PRODUCTION_LANE_COUNT
	{
		count := min(util.PRODUCTION_LANE_COUNT, batch.count - start);
		bundle: Collision_Convex_Wide_Bundle = ---
		gather_status := collision_batcher_gather_subpair_bundle(
			batcher, task, batch, start, count, &bundle,
		);
		if gather_status != .Ok
		{
			return gather_status;
		}
		wide: Collision_Wide_Manifold_Result = ---
		wide_status: Physics_Status;
		when dispatch == .Contextual
		{
			wide_status = binding.wide_test(binding.user_context, &bundle, batcher.shapes, &wide);
		}
		else
		{
			if task.convex_wide_test_into != nil
			{
				wide_status = task.convex_wide_test_into(&bundle, batcher.shapes, &wide);
			}
			else
			{
				wide_status = collision_task_execute_legacy_wide_into(
					task.convex_wide_test, &bundle, batcher.shapes, &wide,
				);
			}
		}
		if wide_status != .Ok
		{
			return wide_status;
		}
		flip_status := collision_wide_manifold_apply_flip_mask(
			&wide, &bundle.offset_b, bundle.flip_mask,
		);
		if flip_status != .Ok
		{
			return flip_status;
		}
		for lane in 0 ..< bundle.count
		{
			manifold, read_status := collision_wide_manifold_read_lane(
				&wide, bundle.offset_b, lane,
			);
			if read_status != .Ok
			{
				return read_status;
			}
			store_status := collision_batcher_store_subpair_lane(
				batcher, &bundle, lane, manifold,
			);
			if store_status != .Ok
			{
				return store_status;
			}
		}
	}
	return .Ok;
}

collision_batcher_execute_subtask_batches :: #force_inline proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, first_subpair_index: i32,
) -> Physics_Status
{
	if batcher == nil || task == nil || task.kind != .Convex ||
	first_subpair_index < 0 || int(first_subpair_index) >= batcher.subpair_count ||
	task.batch_size <= 0 || int(task.batch_size) > MAXIMUM_COLLISION_TASK_BATCH_SIZE
	{
		return .Invalid_Argument;
	}
	batch: Collision_Task_Batch;
	subpair_index := first_subpair_index;
	for subpair_index >= 0
	{
		next_subpair_index := collision_child_storage_next(
			batcher.children, int(subpair_index),
		)^;
		batch.pair_indices[batch.count] = subpair_index;
		batch.count += 1;
		if batch.count == int(task.batch_size)
		{
			status := collision_batcher_execute_subtask_batch(batcher, task, &batch);
			if status != .Ok
			{
				return status;
			}
			batch = {};
		}
		subpair_index = next_subpair_index;
	}
	if batch.count > 0
	{
		return collision_batcher_execute_subtask_batch(batcher, task, &batch);
	}
	return .Ok;
}

collision_batcher_execute_convex_task_batch :: #force_inline proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, batch: ^Collision_Task_Batch,
) -> Physics_Status
{
	return collision_batcher_execute_convex_task_batch_kernel(batcher, task, batch, .Native);
}

collision_batcher_execute_subtask_batch :: #force_inline proc "contextless" (
	batcher: ^Collision_Batcher, task: ^Collision_Task, batch: ^Collision_Task_Batch,
) -> Physics_Status
{
	return collision_batcher_execute_subtask_batch_kernel(batcher, task, batch, .Native);
}
