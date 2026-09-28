package entasis_physics

import util "entasis:entasis_utilities"

// selected only for registered hierarchical geometry. every recursive level
// borrows a disjoint suffix of the existing child scratch. leaf tasks still use
// the original SIMD batcher, including its mesh edge correction and c adapters
Collision_Hierarchy_Callbacks :: struct
{
	root: ^Collision_Batcher,
	pair_id, report_a, report_b: i32,
	shape_a, shape_b: Typed_Index,
	filter: Reference_State,
	status: Physics_Status,
}

collision_hierarchy_completed :: proc "contextless" (user: rawptr, id: i32, manifold: ^Manifold_Result) -> Physics_Status
{
	state: ^Collision_Hierarchy_Callbacks = (^Collision_Hierarchy_Callbacks)(user);
	return state.status;
}

collision_hierarchy_allow :: proc "contextless" (user: rawptr, id, a, b: i32) -> Collision_Testing_State
{
	state: ^Collision_Hierarchy_Callbacks = (^Collision_Hierarchy_Callbacks)(user);
	shapes: ^Shape_Registry = state.root.shapes;
	data_a: rawptr;
	data_b: rawptr;
	data_a, _, _ = shape_registry_resolve(shapes, state.shape_a);
	data_b, _, _ = shape_registry_resolve(shapes, state.shape_b);
	type_a: int = query_overlap_any_child_type(data_a, int(typed_index_type(state.shape_a)), int(a));
	type_b: int = query_overlap_any_child_type(data_b, int(typed_index_type(state.shape_b)), int(b));
	if state.status != .Ok
	{
		return .Reject;
	}
	if state.filter == .Present && state.root.procedures.allow_child_pair != nil
	{
		if state.root.procedures.allow_child_pair(state.root.user_context, state.pair_id,
			state.report_a if state.report_a >= 0 else a, state.report_b if state.report_b >= 0 else b) != .Allow
		{
			return .Reject;
		}
	}
	_, _, state.status = collision_task_registry_lookup(state.root.tasks, type_a, type_b);
	if state.status != .Ok
	{
		return .Reject;
	}
	return .Allow;
}

collision_hierarchy_child_completed :: proc "contextless" (user: rawptr, id, a, b: i32, manifold: ^Convex_Contact_Manifold) -> Physics_Status
{
	state: ^Collision_Hierarchy_Callbacks = (^Collision_Hierarchy_Callbacks)(user);
	if state.root.procedures.child_pair_completed == nil
	{
		return .Ok;
	}
	return state.root.procedures.child_pair_completed(state.root.user_context, state.pair_id,
		state.report_a if state.report_a >= 0 else a, state.report_b if state.report_b >= 0 else b, manifold);
}

collision_hierarchy_leaf :: #force_no_inline proc "contextless" (
	root: ^Collision_Batcher, callbacks: ^Collision_Hierarchy_Callbacks,
	a, b: Typed_Index, pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	margin, dt, maximum_expansion: f32, storage: Collision_Batcher_Child_Storage,
) -> (Manifold_Result, Physics_Status)
{
	if storage.length <= 0
	{
		return {}, .Capacity_Missing;
	}
	lookup: Physics_Status;
	_, _, lookup = collision_task_registry_lookup(root.tasks, int(typed_index_type(a)), int(typed_index_type(b)));
	if lookup != .Ok
	{
		return {}, lookup;
	}
	pairs: [1]Collision_Batcher_Pair;
	continuations: [1]Collision_Batcher_Continuation;
	batcher: Collision_Batcher;
	local: Collision_Hierarchy_Callbacks = callbacks^;
	local.shape_a = a;
	local.shape_b = b;
	local.status = .Ok;
	status: Physics_Status = collision_batcher_initialize_bound(&batcher,
		{memory=raw_data(pairs[:]), length=1, id=-1}, root.shapes, root.tasks,
		{pair_completed=collision_hierarchy_completed, child_pair_completed=collision_hierarchy_child_completed,
			allow_child_pair=collision_hierarchy_allow}, &local, root.traversal_pool,
		{children=storage, continuations={memory=raw_data(continuations[:]), length=1, id=-1}, candidate_refs=root.candidate_refs},
		int(storage.length));
	if status != .Ok
	{
		return {}, status;
	}
	status = collision_batcher_add(&batcher, a, b, pose_a, pose_b, margin, 0, velocity_a, velocity_b, dt, maximum_expansion);
	if status != .Ok
	{
		return {}, status;
	}
	// the selected operands are convex, mesh, or flat compounds. calling the
	// original entry directly avoids re-selecting hierarchical execution
	for task_index in 0 ..< root.tasks.task_count
	{
		if batcher.task_heads[task_index] < 0
		{
			continue;
		}
		task: ^Collision_Task = &root.tasks.tasks[task_index];
		if task.kind == .Convex
		{
			batch: Collision_Task_Batch = {count=1};
			status = collision_batcher_execute_convex_task_batch(&batcher, task, &batch);
		}
		else
		{
			status = collision_batcher_expand_compound_pair(&batcher, 0);
		}
		if status != .Ok
		{
			return {}, status;
		}
	}
	for task_index in 0 ..< root.tasks.task_count
	{
		if batcher.subtask_heads[task_index] < 0
		{
			continue;
		}
		status = collision_batcher_execute_subtask_batches(&batcher, &root.tasks.tasks[task_index], batcher.subtask_heads[task_index]);
		if status != .Ok
		{
			return {}, status;
		}
	}
	for index in 0 ..< batcher.continuation_count
	{
		status = collision_batcher_finish_continuation(&batcher, index);
		if status != .Ok
		{
			return {}, status;
		}
	}
	if local.status != .Ok
	{
		return {}, local.status;
	}
	result: Manifold_Result = collision_stored_manifold_materialize(pairs[0].result);
	if batcher.continuation_count == 0 && root.procedures.child_pair_completed != nil
	{
		status = collision_hierarchy_child_completed(&local, 0, 0, 0, &result.convex);
		if status != .Ok
		{
			return {}, status;
		}
	}
	return result, .Ok;
}

collision_hierarchy_contact :: proc "contextless" (result: ^Manifold_Result, index: int) -> Contact
{
	if result.kind == .Nonconvex
	{
		return result.nonconvex.contacts[index];
	}
	contact: Convex_Contact = result.convex.contacts[index];
	return {offset=contact.offset, depth=contact.depth, normal=result.convex.normal, feature_id=contact.feature_id};
}

collision_hierarchy_contact_count :: proc "contextless" (result: ^Manifold_Result) -> int
{
	if result.kind == .Nonconvex
	{
		return int(result.nonconvex.count);
	}
	return int(result.convex.count);
}

collision_hierarchy_store :: proc "contextless" (
	storage: Collision_Batcher_Child_Storage, index, a, b: int, contact: Contact,
)
{
	header: ^Collision_Child_Manifold_Header = collision_child_storage_header(storage, index);
	header^ = {normal=contact.normal, child_a=i32(a), child_b=i32(b), mesh_child_b=-1,
		contact_start=i32(index*MAXIMUM_MANIFOLD_CONTACT_COUNT), contact_count=1, completed=.Present};
	collision_child_storage_offset_a(storage, index)^ = {};
	collision_child_storage_write_contact(storage, index, 0, {offset=contact.offset, depth=contact.depth, feature_id=contact.feature_id});
}

collision_hierarchy_test :: #force_no_inline proc "contextless" (
	root: ^Collision_Batcher, callbacks: ^Collision_Hierarchy_Callbacks,
	a, b: Typed_Index, pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	margin, dt, maximum_expansion: f32, storage: Collision_Batcher_Child_Storage,
) -> (Manifold_Result, Physics_Status)
{
	depth_a: int = shape_hierarchy_depth(root.shapes, a);
	depth_b: int = shape_hierarchy_depth(root.shapes, b);
	if depth_a == 0 && depth_b == 0
	{
		return collision_hierarchy_leaf(root, callbacks, a, b, pose_a, pose_b, velocity_a, velocity_b, margin, dt, maximum_expansion, storage);
	}
	selected: Typed_Index = a if depth_a > 0 else b;
	selected_pose: Rigid_Pose = pose_a if depth_a > 0 else pose_b;
	data: rawptr;
	data, _, _ = shape_registry_resolve(root.shapes, selected);
	count: int;
	count, _ = collision_compound_child_count(data, int(typed_index_type(selected)));
	written: int;
	for index in 0 ..< count
	{
		child: Collision_Child;
		child_index: Typed_Index;
		status: Physics_Status;
		child_index, child, status = shape_hierarchy_child(root.shapes, data, int(typed_index_type(selected)), index, selected_pose);
		if status != .Ok
		{
			return {}, status;
		}
		available: Collision_Batcher_Child_Storage;
		available, status = collision_child_storage_partition_slice(storage, written, int(storage.length)-written);
		if status != .Ok || available.length <= 0
		{
			return {}, .Capacity_Missing;
		}
		result: Manifold_Result;
		if depth_a > 0
		{
			velocity: Body_Velocity = velocity_a;
			velocity.linear = util.vector3_add(velocity.linear, util.vector3_cross(velocity.angular, util.vector3_subtract(child.pose.position, pose_a.position)));
			result, status = collision_hierarchy_test(root, callbacks, child_index, b, child.pose, pose_b, velocity, velocity_b,
				margin, dt, maximum_expansion, available);
		}
		else
		{
			velocity: Body_Velocity = velocity_b;
			velocity.linear = util.vector3_add(velocity.linear, util.vector3_cross(velocity.angular, util.vector3_subtract(child.pose.position, pose_b.position)));
			result, status = collision_hierarchy_test(root, callbacks, a, child_index, pose_a, child.pose, velocity_a, velocity,
				margin, dt, maximum_expansion, available);
		}
		if status != .Ok
		{
			return {}, status;
		}
		contacts: int = collision_hierarchy_contact_count(&result);
		if contacts > int(storage.length)-written
		{
			return {}, .Capacity_Missing;
		}
		for contact_index in 0 ..< contacts
		{
			contact: Contact = collision_hierarchy_contact(&result, contact_index);
			if depth_a > 0
			{
				contact.offset = util.vector3_add(contact.offset, util.vector3_subtract(child.pose.position, pose_a.position));
			}
			collision_hierarchy_store(storage, written, index if depth_a > 0 else 0, index if depth_a == 0 else 0, contact);
			written += 1;
		}
	}
	records: Collision_Batcher_Child_Storage = storage;
	records.length = i32(written);
	result: Manifold_Result = {kind=.Nonconvex, nonconvex={offset_b=util.vector3_subtract(pose_b.position, pose_a.position)}};
	status: Physics_Status = collision_reduction_finish_records(records, root.candidate_refs, pose_a.position, nil, {}, &result.nonconvex);
	return result, status;
}

collision_batcher_expand_hierarchy_pair :: #force_no_inline proc "contextless" (batcher: ^Collision_Batcher, pair_index: int) -> Physics_Status
{
	if batcher.continuation_count >= int(batcher.continuations.length)
	{
		return .Capacity_Missing;
	}
	pair: ^Collision_Batcher_Pair = &batcher.pairs.memory[pair_index];
	input: ^Collision_Batcher_Input = &pair.input;
	type_a: int = int(typed_index_type(input.shape_a));
	type_b: int = int(typed_index_type(input.shape_b));
	count_a, count_b: int = 1, 1;
	if type_a == COMPOUND_TYPE_ID || type_a == BIG_COMPOUND_TYPE_ID
	{
		count_a, _ = collision_compound_child_count(input.shape_data_a, type_a);
	}
	if type_b == COMPOUND_TYPE_ID || type_b == BIG_COMPOUND_TYPE_ID
	{
		count_b, _ = collision_compound_child_count(input.shape_data_b, type_b);
	}
	continuation: ^Collision_Batcher_Continuation = &batcher.continuations.memory[batcher.continuation_count];
	continuation^ = {parent_pair_index=i32(pair_index), child_start=i32(batcher.child_count),
		offset_b=util.vector3_subtract(input.pose_b.position, input.pose_a.position), parent_position_a=input.pose_a.position};
	batcher.continuation_count += 1;
	for ia in 0 ..< count_a
	{
		a: Typed_Index = input.shape_a;
		child_a: Collision_Child = {shape=input.shape_data_a, type_id=type_a, pose=input.pose_a};
		status: Physics_Status;
		if type_a == COMPOUND_TYPE_ID || type_a == BIG_COMPOUND_TYPE_ID
		{
			a, child_a, status = shape_hierarchy_child(batcher.shapes, input.shape_data_a, type_a, ia, input.pose_a);
			if status != .Ok
			{
				return status;
			}
		}
		for ib in 0 ..< count_b
		{
			b: Typed_Index = input.shape_b;
			child_b: Collision_Child = {shape=input.shape_data_b, type_id=type_b, pose=input.pose_b};
			if type_b == COMPOUND_TYPE_ID || type_b == BIG_COMPOUND_TYPE_ID
			{
				b, child_b, status = shape_hierarchy_child(batcher.shapes, input.shape_data_b, type_b, ib, input.pose_b);
				if status != .Ok
				{
					return status;
				}
			}
			callbacks: Collision_Hierarchy_Callbacks = {root=batcher, pair_id=pair.pair_id,
				report_a=-1 if type_a==MESH_TYPE_ID else i32(ia), report_b=-1 if type_b==MESH_TYPE_ID else i32(ib)};
			if type_a == MESH_TYPE_ID || type_b == MESH_TYPE_ID
			{
				callbacks.filter = .Present;
			}
			else if batcher.procedures.allow_child_pair != nil &&
			batcher.procedures.allow_child_pair(batcher.user_context, pair.pair_id, i32(ia), i32(ib)) != .Allow
			{
				continue;
			}
			available: Collision_Batcher_Child_Storage;
			available, status = collision_child_storage_partition_slice(batcher.children, batcher.child_count, int(batcher.children.length)-batcher.child_count);
			if status != .Ok || available.length <= 0
			{
				return .Capacity_Missing;
			}
			va: Body_Velocity = input.velocity_a;
			vb: Body_Velocity = input.velocity_b;
			va.linear = util.vector3_add(va.linear, util.vector3_cross(va.angular, util.vector3_subtract(child_a.pose.position, input.pose_a.position)));
			vb.linear = util.vector3_add(vb.linear, util.vector3_cross(vb.angular, util.vector3_subtract(child_b.pose.position, input.pose_b.position)));
			result: Manifold_Result;
			result, status = collision_hierarchy_test(batcher, &callbacks, a, b, child_a.pose, child_b.pose, va, vb,
				input.speculative_margin, input.dt, input.maximum_expansion, available);
			if status != .Ok
			{
				return status;
			}
			contacts: int = collision_hierarchy_contact_count(&result);
			if contacts > int(batcher.children.length)-batcher.child_count
			{
				return .Capacity_Missing;
			}
			for contact_index in 0 ..< contacts
			{
				contact: Contact = collision_hierarchy_contact(&result, contact_index);
				contact.offset = util.vector3_add(contact.offset, util.vector3_subtract(child_a.pose.position, input.pose_a.position));
				collision_hierarchy_store(batcher.children, batcher.child_count, ia, ib, contact);
				batcher.child_count += 1;
				continuation.child_count += 1;
			}
			batcher.child_contact_count = batcher.child_count*MAXIMUM_MANIFOLD_CONTACT_COUNT;
		}
	}
	return .Ok;
}

collision_batcher_execute_hierarchy_batches :: #force_no_inline proc "contextless" (
	batcher: ^Collision_Batcher, first: i32,
) -> Physics_Status
{
	index: i32 = first;
	for index >= 0
	{
		pair: ^Collision_Batcher_Pair = &batcher.pairs.memory[index];
		next: i32 = pair.input.next_in_task;
		status: Physics_Status;
		if shape_hierarchy_depth(batcher.shapes, pair.input.shape_a)>0 || shape_hierarchy_depth(batcher.shapes, pair.input.shape_b)>0
		{
			status = collision_batcher_expand_hierarchy_pair(batcher, int(index));
		}
		else
		{
			status = collision_batcher_expand_compound_pair(batcher, int(index));
		}
		if status != .Ok
		{
			return status;
		}
		index = next;
	}
	return .Ok;
}
