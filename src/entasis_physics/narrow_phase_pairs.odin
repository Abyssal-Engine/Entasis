package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"

narrow_phase_body_handles :: proc "contextless" (
	pair: Broad_Phase_Pair,
) -> ([4]Body_Handle, int, Physics_Status)
{
	handles: [4]Body_Handle;
	mobility_a := collidable_reference_mobility(pair.a);
	mobility_b := collidable_reference_mobility(pair.b);
	if mobility_a == .Static && mobility_b == .Static
	{
		return handles, 0, .Invalid_Argument;
	}
	if mobility_a == .Static
	{
		handles[0] = {collidable_reference_raw_handle(pair.b)};
		return handles, 1, .Ok;
	}
	handles[0] = {collidable_reference_raw_handle(pair.a)};
	if mobility_b == .Static
	{
		return handles, 1, .Ok;
	}
	handles[1] = {collidable_reference_raw_handle(pair.b)};
	return handles, 2, .Ok;
}
narrow_phase_pair_completed :: proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^Manifold_Result,
) -> Physics_Status
{
	context = runtime.default_context();
	worker := (^Narrow_Phase_Worker_Context)(user_context);
	if worker == nil || worker.narrow == nil
	{
		return .Invalid_Argument;
	}
	narrow := worker.narrow;
	if narrow == nil ||
		narrow.state != .Stepping ||
		pair_id < 0 ||
		int(pair_id) >= int(narrow.candidates.length) ||
		manifold == nil
	{
		return .Invalid_Argument;
	}
	continuation := &narrow.continuations.memory[pair_id];
	narrow_phase_rewind_depths(continuation, manifold);
	broad_pair := narrow.candidates.memory[pair_id];
	contact_count := manifold.convex.count;
	if manifold.kind == .Nonconvex
	{
		contact_count = manifold.nonconvex.count;
	}
	result := &narrow.results.memory[pair_id];
	if contact_count == 0
	{
		result.state = .Rejected;
		return .Ok;
	}
	material: Contact_Material_Properties;
	if narrow.callbacks.configure(
		narrow.callbacks.user_context, int(worker.worker_index), broad_pair.a, broad_pair.b, manifold, &material,
	) != .Allow
	{
		result.state = .Rejected;
		return .Ok;
	}
	result.manifold.kind = manifold.kind;
	if manifold.kind == .Convex
	{
		result.manifold.convex = manifold.convex;
	}
	else
	{
		result.manifold.nonconvex = manifold.nonconvex;
	}
	result.material = material;
	result.state = .Accepted;
	return .Ok;
}
narrow_phase_pair_completed_stored :: proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^Collision_Stored_Manifold,
) -> Physics_Status
{
	context = runtime.default_context();
	worker := (^Narrow_Phase_Worker_Context)(user_context);
	if worker == nil || worker.narrow == nil
	{
		return .Invalid_Argument;
	}
	narrow := worker.narrow;
	if narrow.state != .Stepping || narrow.stored_completion_state != .Present ||
		pair_id < 0 || int(pair_id) >= int(narrow.candidates.length) || manifold == nil
	{
		return .Invalid_Argument;
	}
	continuation := &narrow.continuations.memory[pair_id];
	narrow_phase_rewind_stored_depths(continuation, manifold);
	contact_count := manifold.convex.count;
	if manifold.kind == .Nonconvex
	{
		contact_count = manifold.nonconvex.count;
	}
	result := &narrow.results.memory[pair_id];
	if contact_count == 0
	{
		result.state = .Rejected;
		return .Ok;
	}
	result.manifold = manifold^;
	result.material = narrow.stored_completion_material;
	result.state = .Accepted;
	return .Ok;
}
narrow_phase_pair_completed_stored_fused :: proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^Collision_Stored_Manifold,
) -> Physics_Status
{
	context = runtime.default_context();
	worker := (^Narrow_Phase_Worker_Context)(user_context);
	validation_status := Physics_Status.Ok;
	batcher_state := Collision_Batcher_State.Disposed;
	if worker == nil || worker.narrow == nil
	{
		validation_status = .Invalid_Argument;
	}
	else
	{
		narrow := worker.narrow;
		worker_index := int(worker.worker_index);
		if worker_index < 0 || worker_index >= narrow.active_worker_count
		{
			validation_status = .Invalid_Argument;
		}
		else
		{
			batcher_state = narrow.batchers[worker_index].state;
			if narrow.state != .Stepping || narrow.stored_completion_state != .Present ||
				manifold == nil
			{
				validation_status = .Invalid_Argument;
			}
			else
			{
				candidate_count := int(
					narrow.transaction_streams[worker_index].traversal_candidate_count,
				);
				candidate_start := worker_index * narrow.pending_capacity_per_worker;
				candidate_end := candidate_start + candidate_count;
				pair_index := int(pair_id);
				if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
					manifold.kind != .Convex || manifold.convex.count < 0 ||
					manifold.convex.count > MAXIMUM_MANIFOLD_CONTACT_COUNT
				{
					validation_status = .Invalid_Description;
				}
				else if batcher_state == .Ready
				{
					if pair_index < candidate_start || pair_index > candidate_end ||
						(pair_index == candidate_end &&
						(candidate_count >= narrow.pending_capacity_per_worker ||
						manifold.convex.count != 0))
					{
						validation_status = .Invalid_Description;
					}
				}
				else if batcher_state == .Flushing
				{
					if pair_index < candidate_start || pair_index >= candidate_end
					{
						validation_status = .Invalid_Description;
					}
				}
				else
				{
					validation_status = .Invalid_Description;
				}
			}
		}
	}
	if validation_status == .Ok || batcher_state == .Ready
	{
		return validation_status;
	}
	if batcher_state == .Flushing && worker != nil && worker.narrow != nil
	{
		worker_index := int(worker.worker_index);
		if worker.narrow.transaction_statuses[worker_index] == .Ok
		{
			worker.narrow.transaction_statuses[worker_index] = validation_status;
		}
		return .Ok;
	}
	return validation_status;
}
narrow_phase_build_stored_description :: proc "contextless" (
	narrow: ^Narrow_Phase, result: ^Narrow_Phase_Pair_Result, body_count: int,
	description: rawptr,
) -> (i32, [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32, Physics_Status)
{
	feature_ids: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32;
	for index in 0 ..< MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		feature_ids[index] = -1;
	}
	if narrow == nil || result == nil || description == nil ||
		body_count < 1 || body_count > 2
	{
		return -1, feature_ids, .Invalid_Argument;
	}
	contact_count := int(result.manifold.convex.count);
	kind := Contact_Constraint_Kind.Convex;
	if result.manifold.kind == .Nonconvex
	{
		contact_count = int(result.manifold.nonconvex.count);
		if contact_count > 1
		{
			kind = .Nonconvex;
		}
	}
	if contact_count < 1 || contact_count > MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return -1, feature_ids, .Invalid_Description;
	}
	type_id := contact_constraint_type_id(body_count, contact_count, kind);
	record, lookup_status := contact_constraint_accessor_lookup(&narrow.accessors, type_id);
	if lookup_status != .Ok
	{
		return -1, feature_ids, lookup_status;
	}
	if int(record.body_count) != body_count ||
		int(record.contact_count) != contact_count || record.kind != kind
	{
		return -1, feature_ids, .Invalid_Description;
	}
	write_status: Physics_Status;
	if result.manifold.kind == .Convex
	{
		for index in 0 ..< contact_count
		{
			feature_ids[index] = result.manifold.convex.contacts[index].feature_id;
		}
		write_status = contact_constraint_write_convex(
			description, body_count, &result.manifold.convex, result.material,
		);
	}
	else if contact_count == 1
	{
		contact := result.manifold.nonconvex.contacts[0];
		converted := Convex_Contact_Manifold{
			offset_b=result.manifold.nonconvex.offset_b,
			count=1,
			normal=contact.normal,
		};
		converted.contacts[0] = {contact.offset, contact.depth, contact.feature_id};
		feature_ids[0] = contact.feature_id;
		write_status = contact_constraint_write_convex(
			description, body_count, &converted, result.material,
		);
	}
	else
	{
		for index in 0 ..< contact_count
		{
			feature_ids[index] = result.manifold.nonconvex.contacts[index].feature_id;
		}
		write_status = contact_constraint_write_nonconvex(
			description, body_count, &result.manifold.nonconvex, result.material,
		);
	}
	return type_id, feature_ids, write_status;
}
narrow_phase_build_materialized_pair_description :: #force_no_inline proc "contextless" (
	narrow: ^Narrow_Phase, broad_pair: Broad_Phase_Pair,
	result: ^Narrow_Phase_Pair_Result, description: rawptr,
	handles: [4]Body_Handle, body_count: int,
) -> (Narrow_Phase_Prepared_Description, Physics_Status)
{
	prepared: Narrow_Phase_Prepared_Description;
	if narrow == nil || result == nil || description == nil
	{
		return prepared, .Invalid_Argument;
	}
	manifold_value := Manifold_Result{kind=result.manifold.kind};
	if result.manifold.kind == .Convex
	{
		manifold_value.convex = result.manifold.convex;
	}
	else
	{
		manifold_value.nonconvex = result.manifold.nonconvex;
	}
	manifold := &manifold_value;
	contact_count := manifold.convex.count;
	if manifold.kind == .Nonconvex
	{
		contact_count = manifold.nonconvex.count;
	}
	requested_type_id := i32(-1);
	if narrow.callbacks.select_constraint != nil
	{
		default_kind := Contact_Constraint_Kind.Convex;
		if manifold.kind == .Nonconvex && contact_count > 1
		{
			default_kind = .Nonconvex;
		}
		default_type_id := contact_constraint_type_id(
			body_count, int(contact_count), default_kind,
		);
		requested_type_id = narrow.callbacks.select_constraint(
			narrow.callbacks.user_context, broad_pair.a, broad_pair.b, manifold,
			default_type_id,
		);
	}
	type_id, features, build_status := contact_constraint_build_description(
		&narrow.accessors, manifold, body_count, result.material, description,
		requested_type_id,
	);
	if build_status != .Ok
	{
		return prepared, build_status;
	}
	prepared = {
		body_handles=handles,
		feature_ids=features,
		type_record=&narrow.solver.registry.records[type_id],
		accessor=&narrow.accessors.records[type_id],
		type_id=type_id,
		body_count=i32(body_count),
	};
	return prepared, .Ok;
}
narrow_phase_build_pair_description :: proc "contextless" (
	narrow: ^Narrow_Phase, pair: Collidable_Pair, result: ^Narrow_Phase_Pair_Result,
	description: rawptr,
) -> (Narrow_Phase_Prepared_Description, Physics_Status)
{
	prepared: Narrow_Phase_Prepared_Description;
	if narrow == nil || result == nil || description == nil
	{
		return prepared, .Invalid_Argument;
	}
	broad_pair := Broad_Phase_Pair{pair.a, pair.b};
	handles, body_count, handles_status := narrow_phase_body_handles(broad_pair);
	if handles_status != .Ok
	{
		return prepared, handles_status;
	}
	if narrow.stored_completion_state == .Present
	{
		type_id, features, build_status := narrow_phase_build_stored_description(
			narrow, result, body_count, description,
		);
		if build_status != .Ok
		{
			return prepared, build_status;
		}
		prepared = {
			body_handles=handles,
			feature_ids=features,
			type_record=&narrow.solver.registry.records[type_id],
			accessor=&narrow.accessors.records[type_id],
			type_id=type_id,
			body_count=i32(body_count),
		};
		return prepared, .Ok;
	}
	return narrow_phase_build_materialized_pair_description(
		narrow, broad_pair, result, description, handles, body_count,
	);
}
narrow_phase_child_completed :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32, manifold: ^Convex_Contact_Manifold,
) -> Physics_Status
{
	worker := (^Narrow_Phase_Worker_Context)(user_context);
	if worker == nil || worker.narrow == nil || manifold == nil || pair_id < 0 ||
		int(pair_id) >= int(worker.narrow.candidates.length)
	{
		return .Invalid_Argument;
	}
	pair := worker.narrow.candidates.memory[pair_id];
	if worker.narrow.callbacks.configure_child(
		worker.narrow.callbacks.user_context, int(worker.worker_index), pair.a, pair.b,
		int(child_a), int(child_b), manifold,
	) != .Allow
	{
		manifold.count = 0;
	}
	return .Ok;
}
narrow_phase_allow_child :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	worker := (^Narrow_Phase_Worker_Context)(user_context);
	if worker == nil || worker.narrow == nil || pair_id < 0 || int(pair_id) >= int(worker.narrow.candidates.length)
	{
		return .Reject;
	}
	pair := worker.narrow.candidates.memory[pair_id];
	return worker.narrow.callbacks.allow_child(
		worker.narrow.callbacks.user_context, int(worker.worker_index), pair.a, pair.b,
		int(child_a), int(child_b),
	);
}
narrow_phase_prepare_batchers :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Physics_Status
{
	if narrow == nil || worker_count <= 0 || worker_count > narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	base_capacity := narrow.pending_capacity_per_worker;
	base_child_capacity := narrow.collision_child_capacity / worker_count;
	extra_child_capacity_count := narrow.collision_child_capacity % worker_count;
	narrow.collision_capacity_per_worker = base_capacity;
	procedures := Collision_Result_Procedures{
		pair_completed=narrow_phase_pair_completed,
		child_pair_completed=narrow_phase_child_completed,
		allow_child_pair=narrow_phase_allow_child,
	};
	storage_offset := 0;
	parent_offset := 0;
	child_offset := 0;
	for worker_index in 0 ..< worker_count
	{
		capacity := base_capacity;
		child_capacity := base_child_capacity;
		if worker_index < extra_child_capacity_count
		{
			child_capacity += 1;
		}
		if storage_offset > int(narrow.collision_storage.length) - capacity
		{
			return .Capacity_Missing;
		}
		candidate_offset := child_offset * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		candidate_capacity := child_capacity * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		candidate_refs, candidate_status := collision_candidate_storage_slice(
			narrow.collision_scratch.candidate_refs, candidate_offset, candidate_capacity,
		);
		if candidate_status != .Ok
		{
			return candidate_status;
		}
		children, children_status := collision_child_storage_partition_slice(
			narrow.collision_scratch.children, child_offset, child_capacity,
		);
		if children_status != .Ok
		{
			return children_status;
		}
		storage := util.Buffer(Collision_Batcher_Pair){
			memory=&narrow.collision_storage.memory[storage_offset],
			length=i32(capacity),
			id=-1,
		};
		scratch := Collision_Batcher_Scratch{
			children=children,
			continuations={
				memory=&narrow.collision_scratch.continuations.memory[parent_offset], length=i32(capacity), id=-1,
			},
			candidate_refs=candidate_refs,
		};
		narrow.batchers[worker_index] = {};
		status := collision_batcher_initialize_bound(
			&narrow.batchers[worker_index], storage, narrow.shapes, narrow.tasks,
			procedures, &narrow.worker_contexts[worker_index], narrow.pool,
			scratch, child_capacity,
		);
		if status != .Ok
		{
			return status;
		}
		if narrow.stored_completion_state == .Present
		{
			narrow.batchers[worker_index].stored_pair_completed =
				narrow_phase_pair_completed_stored;
		}
		storage_offset += capacity;
		parent_offset += capacity;
		child_offset += child_capacity;
	}
	return .Ok;
}
narrow_phase_fused_contact_type_identity :: proc "contextless" (
	narrow: ^Narrow_Phase, body_count, contact_count: int,
) -> Reference_State
{
	type_id := int(contact_constraint_type_id(body_count, contact_count, .Convex));
	if type_id < CONTACT_1_ONE_BODY_TYPE_ID || type_id > CONTACT_4_TYPE_ID
	{
		return .Missing;
	}
	expected_description_size := 0;
	expected_prestep_bundle_size := 0;
	expected_impulse_bundle_size := 0;
	expected_kernel: Constraint_Kernel_Proc;
	switch type_id
	{
		case CONTACT_1_ONE_BODY_TYPE_ID:
			expected_description_size = size_of(Contact_1_One_Body);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_1_One_Body);
			expected_kernel = contact_1_one_body_kernel;
		case CONTACT_2_ONE_BODY_TYPE_ID:
			expected_description_size = size_of(Contact_2_One_Body);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_2_One_Body);
			expected_kernel = contact_2_one_body_kernel;
		case CONTACT_3_ONE_BODY_TYPE_ID:
			expected_description_size = size_of(Contact_3_One_Body);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_3_One_Body);
			expected_kernel = contact_3_one_body_kernel;
		case CONTACT_4_ONE_BODY_TYPE_ID:
			expected_description_size = size_of(Contact_4_One_Body);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_4_One_Body);
			expected_kernel = contact_4_one_body_kernel;
		case CONTACT_1_TYPE_ID:
			expected_description_size = size_of(Contact_1);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_1);
			expected_kernel = contact_1_kernel;
		case CONTACT_2_TYPE_ID:
			expected_description_size = size_of(Contact_2);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_2);
			expected_kernel = contact_2_kernel;
		case CONTACT_3_TYPE_ID:
			expected_description_size = size_of(Contact_3);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_3);
			expected_kernel = contact_3_kernel;
		case CONTACT_4_TYPE_ID:
			expected_description_size = size_of(Contact_4);
			expected_prestep_bundle_size, expected_impulse_bundle_size =
				constraint_type_layout(Contact_4);
			expected_kernel = contact_4_kernel;
		case:
			return .Missing;
	}
	accessor := &narrow.accessors.records[type_id];
	type_record := &narrow.solver.registry.records[type_id];
	no_pose := [4]Body_Access_Mask{
		BODY_ACCESS_NO_POSE, BODY_ACCESS_NO_POSE, {}, {},
	};
	if accessor.registration != .Present || accessor.type_id != i32(type_id) ||
		accessor.body_count != i32(body_count) ||
		accessor.contact_count != i32(contact_count) ||
		accessor.kind != .Convex ||
		accessor.build_description != contact_constraint_builtin_build ||
		accessor.impulse_offset != contact_constraint_builtin_impulse_offset_convex ||
		type_record.registration != .Present || type_record.type_id != i32(type_id) ||
		type_record.body_count != i32(body_count) ||
		type_record.description_size != i32(expected_description_size) ||
		type_record.prestep_bundle_size != i32(expected_prestep_bundle_size) ||
		type_record.impulse_bundle_size != i32(expected_impulse_bundle_size) ||
		type_record.initial_access != no_pose || type_record.solve_access != no_pose ||
		type_record.apply_description != constraint_description_transfer ||
		type_record.build_description != constraint_description_build ||
		type_record.validate_description != constraint_registry_validate_description ||
		type_record.prestep_warmstart_solve != expected_kernel ||
		type_record.incrementally_update != expected_kernel ||
		type_record.move_record != constraint_storage_move_lane ||
		type_record.remove_record != constraint_storage_remove_lane
	{
		return .Missing;
	}
	return .Present;
}
narrow_phase_fused_route_state :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Reference_State
{
	if narrow == nil ||
		narrow.stored_completion_state != .Present ||
		narrow.callbacks.select_constraint != nil ||
		constraint_contact_material_validate(narrow.stored_completion_material) != .Ok ||
		narrow.shapes == nil || narrow.shapes.state != .Allocated ||
		narrow.shapes.registered_type_count <= BOX_TYPE_ID ||
		narrow.tasks == nil || narrow.tasks.state != .Ready ||
		narrow.solver == nil || narrow.solver.state != .Ready ||
		narrow.accessors.state != .Ready || worker_count <= 0 ||
		worker_count > narrow.active_worker_count ||
		narrow.pending_capacity_per_worker <= 0 ||
		narrow.pending_capacity_per_worker >
		max(int) / size_of(Narrow_Phase_Pair_Result) ||
		narrow.active_worker_count > max(int) / narrow.pending_capacity_per_worker ||
		int(narrow.results.length) <
		narrow.pending_capacity_per_worker * narrow.active_worker_count
	{
		return .Missing;
	}
	shape_type := BOX_TYPE_ID;
	if narrow.shapes.registered_type_count > CONVEX_HULL_TYPE_ID &&
		narrow.shapes.batches[CONVEX_HULL_TYPE_ID].active_count > 0
	{
		shape_type = CONVEX_HULL_TYPE_ID;
	}
	if narrow.shapes.batches[shape_type].state != .Registered ||
		narrow.shapes.batches[shape_type].active_count <= 0
	{
		return .Missing;
	}
	for type_id in 0 ..< narrow.shapes.registered_type_count
	{
		if type_id != shape_type && narrow.shapes.batches[type_id].active_count > 0
		{
			return .Missing;
		}
	}
	task, reference, task_status := collision_task_registry_lookup(
		narrow.tasks, shape_type, shape_type,
	);
	expected_batch_size := i16(32);
	expected_test := box_pair_test;
	expected_wide_test := collision_task_execute_box_pair_wide;
	expected_wide_test_into := collision_task_execute_box_pair_wide_into;
	if shape_type == CONVEX_HULL_TYPE_ID
	{
		expected_batch_size = 16;
		expected_test = convex_hull_pair_test;
		expected_wide_test = collision_task_execute_hull_pair_wide;
		expected_wide_test_into = collision_task_execute_hull_pair_wide_into;
	}
	if task_status != .Ok || task == nil || reference.order != .Expected ||
		reference.batch_size != expected_batch_size || reference.pair_type != .Flipless ||
		task.task_id != reference.task_id || task.shape_type_a != i16(shape_type) ||
		task.shape_type_b != i16(shape_type) || task.batch_size != expected_batch_size ||
		task.pair_type != .Flipless || task.kind != .Convex ||
		task.capabilities != (Collision_Task_Capabilities{.Convex_Result, .Wide_Result}) ||
		task.convex_test != expected_test ||
		task.convex_wide_test != expected_wide_test ||
		task.convex_wide_test_into != expected_wide_test_into
	{
		return .Missing;
	}
	for body_count in 1 ..= 2
	{
		for contact_count in 1 ..= 4
		{
			if narrow_phase_fused_contact_type_identity(
				narrow, body_count, contact_count,
			) != .Present
			{
				return .Missing;
			}
		}
	}
	for worker_index in 0 ..< worker_count
	{
		batcher := &narrow.batchers[worker_index];
		if batcher.state != .Ready ||
			int(batcher.pairs.length) < narrow.pending_capacity_per_worker ||
			batcher.pair_count != 0 || batcher.shapes != narrow.shapes ||
			batcher.tasks != narrow.tasks ||
			batcher.procedures.pair_completed != narrow_phase_pair_completed ||
			batcher.procedures.child_pair_completed != narrow_phase_child_completed ||
			batcher.procedures.allow_child_pair != narrow_phase_allow_child ||
			batcher.stored_pair_completed != narrow_phase_pair_completed_stored ||
			batcher.user_context != &narrow.worker_contexts[worker_index]
		{
			return .Missing;
		}
	}
	return .Present;
}
narrow_phase_box_sphere_route_state :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Narrow_Phase_Collision_Route
{
	if narrow == nil ||
		narrow.stored_completion_state != .Present ||
		narrow.callbacks.select_constraint != nil ||
		constraint_contact_material_validate(narrow.stored_completion_material) != .Ok ||
		narrow.shapes == nil || narrow.shapes.state != .Allocated ||
		narrow.shapes.registered_type_count <= BOX_TYPE_ID ||
		narrow.tasks == nil || narrow.tasks.state != .Ready ||
		narrow.solver == nil || narrow.solver.state != .Ready ||
		narrow.accessors.state != .Ready || worker_count <= 0 ||
		worker_count > narrow.active_worker_count ||
		narrow.pending_capacity_per_worker <= 0 ||
		narrow.pending_capacity_per_worker >
		max(int) / size_of(Narrow_Phase_Pair_Result) ||
		narrow.active_worker_count > max(int) / narrow.pending_capacity_per_worker ||
		int(narrow.results.length) <
		narrow.pending_capacity_per_worker * narrow.active_worker_count
	{
		return .Predecessor;
	}
	if narrow.shapes.batches[SPHERE_TYPE_ID].state != .Registered ||
		narrow.shapes.batches[SPHERE_TYPE_ID].active_count <= 0 ||
		narrow.shapes.batches[BOX_TYPE_ID].state != .Registered ||
		narrow.shapes.batches[BOX_TYPE_ID].active_count <= 0
	{
		return .Predecessor;
	}
	for type_id in 0 ..< narrow.shapes.registered_type_count
	{
		if type_id != SPHERE_TYPE_ID && type_id != BOX_TYPE_ID &&
			narrow.shapes.batches[type_id].active_count > 0
		{
			return .Predecessor;
		}
	}
	for task_case in 0 ..< 3
	{
		type_a, type_b: int;
		type_a, type_b = SPHERE_TYPE_ID, SPHERE_TYPE_ID;
		expected_pair_type: Collision_Task_Pair_Type = Collision_Task_Pair_Type.Sphere;
		expected_test: Collision_Test_Proc = sphere_pair_test;
		expected_wide_test: Collision_Wide_Test_Proc = collision_task_execute_sphere_pair_wide;
		expected_wide_test_into: collision_wide_test_into_proc = collision_task_execute_sphere_pair_wide_into;
		if task_case == 1
		{
			type_b = BOX_TYPE_ID;
			expected_pair_type = .Sphere_Including;
			expected_test = sphere_box_test;
			expected_wide_test = collision_task_execute_sphere_box_wide;
			expected_wide_test_into = collision_task_execute_sphere_box_wide_into;
		}
		else if task_case == 2
		{
			type_a, type_b = BOX_TYPE_ID, BOX_TYPE_ID;
			expected_pair_type = .Flipless;
			expected_test = box_pair_test;
			expected_wide_test = collision_task_execute_box_pair_wide;
			expected_wide_test_into = collision_task_execute_box_pair_wide_into;
		}
		task: ^Collision_Task;
		reference: Collision_Task_Reference;
		task_status: Physics_Status;
		task, reference, task_status = collision_task_registry_lookup(
			narrow.tasks, type_a, type_b,
		);
		if task_status != .Ok || task == nil || reference.order != .Expected ||
			reference.batch_size != 32 || reference.pair_type != expected_pair_type ||
			reference.expected_first_type_id != i16(type_a) ||
			task.task_id != reference.task_id || task.shape_type_a != i16(type_a) ||
			task.shape_type_b != i16(type_b) || task.batch_size != 32 ||
			task.pair_type != expected_pair_type || task.kind != .Convex ||
			task.capabilities != (Collision_Task_Capabilities{.Convex_Result, .Wide_Result}) ||
			task.convex_test != expected_test || task.convex_wide_test != expected_wide_test ||
			task.convex_wide_test_into != expected_wide_test_into
		{
			return .Predecessor;
		}
		if type_a != type_b
		{
			reverse: Collision_Task_Reference = narrow.tasks.routes[collision_task_matrix_index(type_b, type_a)];
			if reverse.task_id != reference.task_id || reverse.order != .Flipped ||
				reverse.batch_size != 32 || reverse.pair_type != expected_pair_type ||
				reverse.expected_first_type_id != i16(type_a)
			{
				return .Predecessor;
			}
		}
	}
	for body_count in 1 ..= 2
	{
		for contact_count in 1 ..= 4
		{
			if narrow_phase_fused_contact_type_identity(
				narrow, body_count, contact_count,
			) != .Present
			{
				return .Predecessor;
			}
		}
	}
	for worker_index in 0 ..< worker_count
	{
		batcher: ^Collision_Batcher = &narrow.batchers[worker_index];
		if batcher.state != .Ready ||
			int(batcher.pairs.length) < narrow.pending_capacity_per_worker ||
			batcher.pair_count != 0 || batcher.shapes != narrow.shapes ||
			batcher.tasks != narrow.tasks ||
			batcher.procedures.pair_completed != narrow_phase_pair_completed ||
			batcher.procedures.child_pair_completed != narrow_phase_child_completed ||
			batcher.procedures.allow_child_pair != narrow_phase_allow_child ||
			batcher.stored_pair_completed != narrow_phase_pair_completed_stored ||
			batcher.user_context != &narrow.worker_contexts[worker_index]
		{
			return .Predecessor;
		}
	}
	return .Box_Sphere_Direct;
}
narrow_phase_collision_route_state :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_count: int,
) -> Narrow_Phase_Collision_Route
{
	if narrow_phase_fused_route_state(narrow, worker_count) != .Present
	{
		return .Predecessor;
	}
	if narrow.shapes.registered_type_count > CONVEX_HULL_TYPE_ID &&
		narrow.shapes.batches[CONVEX_HULL_TYPE_ID].active_count > 0
	{
		return .Hull_Direct;
	}
	return .Box_Direct;
}
narrow_phase_queue_candidate :: proc "contextless" (
	narrow: ^Narrow_Phase, pair_index, worker_index: int, dt: f32,
	fused_route: Narrow_Phase_Collision_Route,
	$record_route: Narrow_Phase_Collision_Route,
) -> Physics_Status
{
	pair := narrow.candidates.memory[pair_index];
	mobility_a := collidable_reference_mobility(pair.a);
	mobility_b := collidable_reference_mobility(pair.b);
	if mobility_a != .Dynamic && mobility_b != .Dynamic
	{
		return .Ok;
	}

	location_a, location_b: Body_Memory_Location;
	set_a, set_b: ^Body_Set;
	collidable_a, collidable_b: ^Collidable;
	margin_a, margin_b: f32;
	if mobility_a != .Static
	{
		location_a = narrow.bodies.handle_to_location.memory[collidable_reference_raw_handle(pair.a)];
		set_a = &narrow.bodies.sets.memory[location_a.set_index];
		collidable_a = &set_a.collidables.memory[location_a.index];
		margin_a = collidable_a.speculative_margin;
	}
	if mobility_b != .Static
	{
		location_b = narrow.bodies.handle_to_location.memory[collidable_reference_raw_handle(pair.b)];
		set_b = &narrow.bodies.sets.memory[location_b.set_index];
		collidable_b = &set_b.collidables.memory[location_b.index];
		margin_b = collidable_b.speculative_margin;
	}

	speculative_margin := margin_a + margin_b;
	if narrow.callbacks.allow(
		narrow.callbacks.user_context, worker_index, pair.a, pair.b, &speculative_margin,
	) != .Allow
	{
		return .Ok;
	}
	if speculative_margin < 0
	{
		return .Invalid_Argument;
	}

	shape_a, shape_b: Typed_Index;
	pose_a, pose_b: Rigid_Pose;
	velocity_a, velocity_b: Body_Velocity;
	continuity_a, continuity_b: Continuous_Detection;
	if mobility_a == .Static
	{
		static_index := narrow.statics.handle_to_index.memory[collidable_reference_raw_handle(pair.a)];
		value := &narrow.statics.statics.memory[static_index];
		shape_a = value.shape;
		pose_a = value.pose;
		continuity_a = value.continuity;
	}
	else
	{
		motion := &set_a.dynamics_state.memory[location_a.index].motion;
		shape_a = collidable_a.shape;
		pose_a = motion.pose;
		velocity_a = motion.velocity;
		continuity_a = collidable_a.continuity;
	}
	if mobility_b == .Static
	{
		static_index := narrow.statics.handle_to_index.memory[collidable_reference_raw_handle(pair.b)];
		value := &narrow.statics.statics.memory[static_index];
		shape_b = value.shape;
		pose_b = value.pose;
		continuity_b = value.continuity;
	}
	else
	{
		motion := &set_b.dynamics_state.memory[location_b.index].motion;
		shape_b = collidable_b.shape;
		pose_b = motion.pose;
		velocity_b = motion.velocity;
		continuity_b = collidable_b.continuity;
	}

	batcher := &narrow.batchers[worker_index];
	if fused_route != .Predecessor
	{
		if batcher.pair_count < 0 || batcher.pair_count >= int(batcher.pairs.length) ||
			batcher.pairs.memory == nil
		{
			return .Capacity_Missing;
		}
	}
	else if batcher.pair_count >= int(batcher.pairs.length)
	{
		flush_status := collision_batcher_flush(batcher);
		if flush_status != .Ok
		{
			return flush_status;
		}
	}
	when record_route == .Box_Sphere_Direct
	{
		if continuity_a.mode != .Continuous && continuity_b.mode != .Continuous
		{
			narrow.continuations.memory[pair_index] = Narrow_Phase_CCD_Continuation{
				pair=collidable_pair_create(pair.a, pair.b), worker_index=i32(worker_index), kind=.Discrete,
			};
		}
		else
		{
			continuation_status: Physics_Status = narrow_phase_prepare_continuation(
				narrow, pair_index, worker_index, pair, shape_a, shape_b, &pose_a, &pose_b,
				velocity_a, velocity_b, continuity_a, continuity_b, speculative_margin, dt,
			);
			if continuation_status != .Ok
			{
				return continuation_status;
			}
		}
	}
	else
	{
		continuation_status := narrow_phase_prepare_continuation(
			narrow, pair_index, worker_index, pair, shape_a, shape_b, &pose_a, &pose_b,
			velocity_a, velocity_b, continuity_a, continuity_b, speculative_margin, dt,
		);
		if continuation_status != .Ok
		{
			return continuation_status;
		}
	}
	when record_route == .Box_Sphere_Direct
	{
		type_a, type_b: i32;
		type_a, type_b = typed_index_type(shape_a), typed_index_type(shape_b);
		if (type_a != SPHERE_TYPE_ID && type_a != BOX_TYPE_ID) ||
			(type_b != SPHERE_TYPE_ID && type_b != BOX_TYPE_ID)
		{
			return .Invalid_Description;
		}
		reference: Collision_Task_Reference;
		task_status: Physics_Status;
		_, reference, task_status = collision_task_registry_lookup(narrow.tasks, int(type_a), int(type_b));
		if task_status != .Ok
		{
			return task_status;
		}
		shape_data_a: rawptr;
		resolve_a_status: Physics_Status;
		shape_data_a, _, resolve_a_status = shape_registry_resolve(narrow.shapes, shape_a);
		if resolve_a_status != .Ok
		{
			return resolve_a_status;
		}
		shape_data_b: rawptr;
		resolve_b_status: Physics_Status;
		shape_data_b, _, resolve_b_status = shape_registry_resolve(narrow.shapes, shape_b);
		if resolve_b_status != .Ok
		{
			return resolve_b_status;
		}
		if shape_data_a == nil || shape_data_b == nil
		{
			return .Invalid_Description;
		}
		records: [^]Narrow_Phase_Box_Sphere_Record = ([^]Narrow_Phase_Box_Sphere_Record)(rawptr(batcher.pairs.memory));
		record_index: i32 = i32(batcher.pair_count);
		records[record_index] = {
			pair_id=i32(pair_index), speculative_margin=speculative_margin,
			shape_data_a=shape_data_a, shape_data_b=shape_data_b,
			pose_a=pose_a, pose_b=pose_b,
			next_in_task=-1, order=reference.order,
		};
		if batcher.task_tails[reference.task_id] >= 0
		{
			records[batcher.task_tails[reference.task_id]].next_in_task = record_index;
		}
		else
		{
			batcher.task_heads[reference.task_id] = record_index;
		}
		batcher.task_tails[reference.task_id] = record_index;
		batcher.pair_count += 1;
		return .Ok;
	}
	else
	{
		if fused_route != .Predecessor
		{
			expected_type := BOX_TYPE_ID;
			if fused_route == .Hull_Direct
			{
				expected_type = CONVEX_HULL_TYPE_ID;
			}
			if typed_index_type(shape_a) != i32(expected_type) ||
				typed_index_type(shape_b) != i32(expected_type)
			{
				return .Invalid_Description;
			}
			shape_data_a, _, resolve_a_status := shape_registry_resolve(narrow.shapes, shape_a);
			if resolve_a_status != .Ok
			{
				return resolve_a_status;
			}
			shape_data_b, _, resolve_b_status := shape_registry_resolve(narrow.shapes, shape_b);
			if resolve_b_status != .Ok
			{
				return resolve_b_status;
			}
			if shape_data_a == nil || shape_data_b == nil
			{
				return .Invalid_Description;
			}
			direct_records := ([^]Narrow_Phase_Convex_Direct_Record)(rawptr(batcher.pairs.memory));
			direct_records[batcher.pair_count] = {
				pair_id=i32(pair_index),
				speculative_margin=speculative_margin,
				shape_data_a=shape_data_a,
				shape_data_b=shape_data_b,
				pose_a=pose_a,
				pose_b=pose_b,
			};
			batcher.pair_count += 1;
			return .Ok;
		}
		maximum_expansion := speculative_margin;
		if continuity_a.mode != .Discrete || continuity_b.mode != .Discrete
		{
			maximum_expansion = f32(math.F32_MAX);
		}
		return collision_batcher_add(
			batcher, shape_a, shape_b, pose_a, pose_b, speculative_margin, i32(pair_index),
			velocity_a, velocity_b, dt, maximum_expansion,
		);
	}
}
