// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Collision_Batcher_State :: enum u8
{
	Uninitialized,
	Ready,
	Flushing,
	Faulted,
	Disposed,
}

Collision_Batcher_Result_State :: enum u8
{
	Pending,
	Complete,
}

collision_stored_manifold_materialize :: proc "contextless" (
	stored: Collision_Stored_Manifold,
) -> Manifold_Result
{
	switch stored.kind
	{
		case .Convex:
		return {kind=.Convex, convex=stored.convex};
		case .Nonconvex:
		return {kind=.Nonconvex, nonconvex=stored.nonconvex};
	}
	return {};
}

collision_batcher_complete_materialized_pair :: #force_no_inline proc "contextless" (
	batcher: ^Collision_Batcher, pair: ^Collision_Batcher_Pair,
) -> Physics_Status
{
	if batcher == nil || pair == nil
	{
		return .Invalid_Argument;
	}
	result := collision_stored_manifold_materialize(pair.result);
	return batcher.procedures.pair_completed(
		batcher.user_context, pair.pair_id, &result,
	);
}

Collision_Batcher_Input :: struct
{
	shape_a:             Typed_Index,
	shape_b:             Typed_Index,
	shape_data_a:        rawptr,
	shape_data_b:        rawptr,
	pose_a:              Rigid_Pose,
	pose_b:              Rigid_Pose,
	velocity_a:          Body_Velocity,
	velocity_b:          Body_Velocity,
	speculative_margin:  f32,
	dt:                  f32,
	maximum_expansion:   f32,
	next_in_task:         i32,
	order:                Collision_Task_Route_Order,
}

Collision_Batcher_Pair :: struct
{
	pair_id:              i32,
	result_state:         Collision_Batcher_Result_State,
	using payload: struct #raw_union
	{
		input:  Collision_Batcher_Input,
		result: Collision_Stored_Manifold,
	},
}
#assert(size_of(Collision_Batcher_Pair) <= 184);
Collision_Batcher_Subpair :: struct
{
	continuation_index:  i32,
	child_record_index:  i32,
	source_child_a:      i32,
	source_child_b:      i32,
}

Collision_Batcher_Continuation :: struct
{
	parent_pair_index:  i32,
	child_start:        i32,
	child_count:        i32,
	offset_b:           util.Vector3,
	parent_position_a:  util.Vector3,
	mesh_b:             ^Mesh,
	mesh_pose_b:        Rigid_Pose,
	flip:               Reference_State,
}
#assert(size_of(Collision_Batcher_Subpair) == 16);
Collision_Reduction_Candidate_Reference :: struct
{
	packed: u32,
}

Collision_Reduction_Candidate_Format :: enum u8
{
	Packed_U16,
	Packed_U32,
}

Collision_Reduction_Candidate_Storage :: struct
{
	memory: rawptr,
	length: i32,
	id:     i32,
	format: Collision_Reduction_Candidate_Format,
}
#assert(size_of(Collision_Reduction_Candidate_Reference) == 4);
#assert(size_of(Collision_Reduction_Candidate_Storage) == 24);
Collision_Batcher_Child_Scratch :: struct
{
	manifold: Collision_Child_Manifold_Record,
	subpair:  Collision_Batcher_Subpair,
	next:     i32,
}

Collision_Batcher_Child_Storage_Format :: enum u8
{
	Split,
	Combined,
}

Collision_Batcher_Child_Storage :: struct
{
	combined:  util.Buffer(Collision_Batcher_Child_Scratch),
	headers:   util.Buffer(Collision_Child_Manifold_Header),
	offsets_a: util.Buffer(util.Vector3),
	contact_data: util.Buffer(Collision_Child_Contact_Data),
	depths:    util.Buffer(f32),
	subpairs:  util.Buffer(Collision_Batcher_Subpair),
	next:      util.Buffer(i32),
	length:    i32,
	format:    Collision_Batcher_Child_Storage_Format,
}
#assert(size_of(Collision_Batcher_Child_Scratch) == 144);
COLLISION_BATCHER_SPLIT_CHILD_CAPACITY_MAX ::
(1 << 17) / (size_of(Collision_Child_Contact_Data) * MAXIMUM_MANIFOLD_CONTACT_COUNT);
Collision_Batcher_Scratch :: struct
{
	children:         Collision_Batcher_Child_Storage,
	continuations:    util.Buffer(Collision_Batcher_Continuation),
	candidate_refs:   Collision_Reduction_Candidate_Storage,
}

Collision_Batcher :: struct
{
	pairs:             util.Buffer(Collision_Batcher_Pair),
	pair_count:        int,
	children:          Collision_Batcher_Child_Storage,
	subpair_count:     int,
	continuations:     util.Buffer(Collision_Batcher_Continuation),
	continuation_count: int,
	child_count:       int,
	child_contact_count: int,
	candidate_refs:    Collision_Reduction_Candidate_Storage,
	shapes:            ^Shape_Registry,
	tasks:             ^Collision_Task_Registry,
	traversal_pool:    ^util.Buffer_Pool,
	procedures:        Collision_Result_Procedures,
	stored_pair_completed: Collision_Stored_Pair_Result_Proc,
	user_context:      rawptr,
	task_heads:        [MAXIMUM_COLLISION_TASK_COUNT]i32,
	task_tails:        [MAXIMUM_COLLISION_TASK_COUNT]i32,
	subtask_heads:     [MAXIMUM_COLLISION_TASK_COUNT]i32,
	subtask_tails:     [MAXIMUM_COLLISION_TASK_COUNT]i32,
	state:             Collision_Batcher_State,
}

collision_candidate_storage_stride :: proc "contextless" (
	format: Collision_Reduction_Candidate_Format,
) -> int
{
	switch format
	{
		case .Packed_U16:
		return size_of(u16);
		case .Packed_U32:
		return size_of(u32);
	}
	return 0;
}

collision_candidate_storage_take :: proc (
	pool: ^util.Buffer_Pool, child_capacity, candidate_capacity: int,
) -> (Collision_Reduction_Candidate_Storage, Physics_Status)
{
	if pool == nil || child_capacity <= 0 || candidate_capacity <= 0
	{
		return {}, .Invalid_Argument;
	}
	if child_capacity <= 1 << 14
	{
		buffer, status := util.buffer_pool_take_at_least(pool, u16, candidate_capacity);
		if status != .Ok
		{
			return {}, physics_memory_status(status);
		}
		return {
			memory=rawptr(buffer.memory),
			length=i32(candidate_capacity),
			id=buffer.id,
			format=.Packed_U16,
		}, .Ok;
	}
	buffer, status := util.buffer_pool_take_at_least(pool, u32, candidate_capacity);
	if status != .Ok
	{
		return {}, physics_memory_status(status);
	}
	return {
		memory=rawptr(buffer.memory),
		length=i32(candidate_capacity),
		id=buffer.id,
		format=.Packed_U32,
	}, .Ok;
}

collision_candidate_storage_return :: proc (
	pool: ^util.Buffer_Pool, storage: ^Collision_Reduction_Candidate_Storage,
)
{
	if pool == nil || storage == nil || storage.memory == nil
	{
		return;
	}
	_ = util.buffer_pool_return_unsafely(pool, storage.id);
	storage^ = {};
}

collision_candidate_storage_slice :: proc "contextless" (
	storage: Collision_Reduction_Candidate_Storage, start, count: int,
) -> (Collision_Reduction_Candidate_Storage, Physics_Status)
{
	if storage.memory == nil || start < 0 || count < 0 || start > int(storage.length) - count
	{
		return {}, .Invalid_Argument;
	}
	stride := collision_candidate_storage_stride(storage.format);
	if stride <= 0
	{
		return {}, .Invalid_Description;
	}
	result := storage;
	result.memory = rawptr(uintptr(storage.memory) + uintptr(start * stride));
	result.length = i32(count);
	result.id = -1;
	return result, .Ok;
}

collision_candidate_storage_read :: proc "contextless" (
	storage: Collision_Reduction_Candidate_Storage, index: int,
) -> Collision_Reduction_Candidate_Reference
{
	switch storage.format
	{
		case .Packed_U16:
		return {packed=u32(([^]u16)(storage.memory)[index])};
		case .Packed_U32:
		return {packed=([^]u32)(storage.memory)[index]};
	}
	return {};
}

collision_candidate_storage_write :: proc "contextless" (
	storage: Collision_Reduction_Candidate_Storage, index: int,
	reference: Collision_Reduction_Candidate_Reference,
)
{
	switch storage.format
	{
		case .Packed_U16:
		([^]u16)(storage.memory)[index] = u16(reference.packed);
		case .Packed_U32:
		([^]u32)(storage.memory)[index] = reference.packed;
	}
}

collision_child_storage_validate :: proc "contextless" (
	storage: Collision_Batcher_Child_Storage, capacity: int,
) -> Physics_Status
{
	if capacity < 0 || capacity > max(int) / MAXIMUM_MANIFOLD_CONTACT_COUNT ||
	int(storage.length) < capacity
	{
		return .Capacity_Missing;
	}
	contact_capacity := capacity * MAXIMUM_MANIFOLD_CONTACT_COUNT;
	switch storage.format
	{
		case .Split:
		if storage.headers.memory == nil || int(storage.headers.length) < capacity ||
		storage.offsets_a.memory == nil || int(storage.offsets_a.length) < capacity ||
		storage.contact_data.memory == nil || int(storage.contact_data.length) < contact_capacity ||
		storage.depths.memory == nil || int(storage.depths.length) < contact_capacity ||
		storage.subpairs.memory == nil || int(storage.subpairs.length) < capacity ||
		storage.next.memory == nil || int(storage.next.length) < capacity
		{
			return .Capacity_Missing;
		}
		case .Combined:
		if storage.combined.memory == nil || int(storage.combined.length) < capacity
		{
			return .Capacity_Missing;
		}
	}
	return .Ok;
}

collision_child_storage_header :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, index: int,
) -> ^Collision_Child_Manifold_Header
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Batcher phase=collision_reduction reason=checked_child_storage_reference lifetime=until_batcher_scratch_rebind_or_disposal
	if storage.format == .Combined
	{
		return &storage.combined.memory[index].manifold.header;
	}
	return &storage.headers.memory[index];
}

collision_child_storage_offset_a :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, index: int,
) -> ^util.Vector3
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Batcher phase=collision_reduction reason=checked_child_storage_reference lifetime=until_batcher_scratch_rebind_or_disposal
	if storage.format == .Combined
	{
		return &storage.combined.memory[index].manifold.offset_a;
	}
	return &storage.offsets_a.memory[index];
}

collision_child_storage_read_contact :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, child_index, contact_index: int,
) -> Collision_Child_Contact_Payload
{
	if storage.format == .Combined
	{
		return storage.combined.memory[child_index].manifold.contacts[contact_index];
	}
	header := &storage.headers.memory[child_index];
	payload_index := int(header.contact_start) + contact_index;
	data := storage.contact_data.memory[payload_index];
	return {
		offset=data.offset,
		depth=storage.depths.memory[payload_index],
		feature_id=data.feature_id,
	};
}

collision_child_storage_write_contact :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, child_index, contact_index: int,
	payload: Collision_Child_Contact_Payload,
)
{
	if storage.format == .Combined
	{
		storage.combined.memory[child_index].manifold.contacts[contact_index] = payload;
		return;
	}
	header := &storage.headers.memory[child_index];
	payload_index := int(header.contact_start) + contact_index;
	storage.contact_data.memory[payload_index] = {
		offset=payload.offset,
		feature_id=payload.feature_id,
	};
	storage.depths.memory[payload_index] = payload.depth;
}

collision_child_storage_subpair :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, index: int,
) -> ^Collision_Batcher_Subpair
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Batcher phase=collision_reduction reason=checked_child_storage_reference lifetime=until_batcher_scratch_rebind_or_disposal
	if storage.format == .Combined
	{
		return &storage.combined.memory[index].subpair;
	}
	return &storage.subpairs.memory[index];
}

collision_child_storage_next :: #force_inline proc "contextless" (
	storage: Collision_Batcher_Child_Storage, index: int,
) -> ^i32
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Batcher phase=collision_reduction reason=checked_child_storage_reference lifetime=until_batcher_scratch_rebind_or_disposal
	if storage.format == .Combined
	{
		return &storage.combined.memory[index].next;
	}
	return &storage.next.memory[index];
}

collision_child_storage_partition_slice :: proc "contextless" (
	storage: Collision_Batcher_Child_Storage, start, count: int,
) -> (Collision_Batcher_Child_Storage, Physics_Status)
{
	if start < 0 || count < 0 || start > int(storage.length) - count ||
	collision_child_storage_validate(storage, start + count) != .Ok
	{
		return {}, .Invalid_Argument;
	}
	result := Collision_Batcher_Child_Storage{length=i32(count), format=storage.format};
	switch storage.format
	{
		case .Split:
		contact_start := start * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		contact_count := count * MAXIMUM_MANIFOLD_CONTACT_COUNT;
		result.headers = {
			memory=&storage.headers.memory[start], length=i32(count), id=-1,
		};
		result.offsets_a = {
			memory=&storage.offsets_a.memory[start], length=i32(count), id=-1,
		};
		result.contact_data = {
			memory=&storage.contact_data.memory[contact_start], length=i32(contact_count), id=-1,
		};
		result.depths = {
			memory=&storage.depths.memory[contact_start], length=i32(contact_count), id=-1,
		};
		result.subpairs = {
			memory=&storage.subpairs.memory[start], length=i32(count), id=-1,
		};
		result.next = {
			memory=&storage.next.memory[start], length=i32(count), id=-1,
		};
		case .Combined:
		result.combined = {
			memory=&storage.combined.memory[start], length=i32(count), id=-1,
		};
	}
	return result, .Ok;
}

collision_child_storage_record_slice :: proc "contextless" (
	storage: Collision_Batcher_Child_Storage, start, count: int,
) -> (Collision_Batcher_Child_Storage, Physics_Status)
{
	if start < 0 || count < 0 || start > int(storage.length) - count ||
	collision_child_storage_validate(storage, start + count) != .Ok
	{
		return {}, .Invalid_Argument;
	}
	result := storage;
	result.length = i32(count);
	switch storage.format
	{
		case .Split:
		result.headers = {
			memory=&storage.headers.memory[start], length=i32(count), id=-1,
		};
		result.offsets_a = {
			memory=&storage.offsets_a.memory[start], length=i32(count), id=-1,
		};
		case .Combined:
		result.combined = {
			memory=&storage.combined.memory[start], length=i32(count), id=-1,
		};
	}
	return result, .Ok;
}

collision_child_storage_return :: proc (
	pool: ^util.Buffer_Pool, storage: ^Collision_Batcher_Child_Storage,
)
{
	if pool == nil || storage == nil
	{
		return;
	}
	if storage.combined.memory != nil && storage.combined.id >= 0
	{
		physics_return_buffer(pool, &storage.combined);
	}
	if storage.next.memory != nil && storage.next.id >= 0
	{
		physics_return_buffer(pool, &storage.next);
	}
	if storage.subpairs.memory != nil && storage.subpairs.id >= 0
	{
		physics_return_buffer(pool, &storage.subpairs);
	}
	if storage.depths.memory != nil && storage.depths.id >= 0
	{
		physics_return_buffer(pool, &storage.depths);
	}
	if storage.contact_data.memory != nil && storage.contact_data.id >= 0
	{
		physics_return_buffer(pool, &storage.contact_data);
	}
	if storage.offsets_a.memory != nil && storage.offsets_a.id >= 0
	{
		physics_return_buffer(pool, &storage.offsets_a);
	}
	if storage.headers.memory != nil && storage.headers.id >= 0
	{
		physics_return_buffer(pool, &storage.headers);
	}
	storage^ = {};
}

collision_batcher_bind_scratch :: proc "contextless" (
	batcher: ^Collision_Batcher, scratch: Collision_Batcher_Scratch,
	parent_capacity, child_capacity: int,
) -> Physics_Status
{
	if parent_capacity <= 0 || child_capacity <= 0 ||
	child_capacity > max(int) / MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return .Invalid_Argument;
	}
	candidate_capacity := child_capacity * MAXIMUM_MANIFOLD_CONTACT_COUNT;
	if collision_child_storage_validate(scratch.children, child_capacity) != .Ok ||
	scratch.continuations.memory == nil || int(scratch.continuations.length) < parent_capacity ||
	scratch.candidate_refs.memory == nil || int(scratch.candidate_refs.length) < candidate_capacity ||
	scratch.candidate_refs.format == .Packed_U16 && child_capacity > 1 << 14
	{
		return .Capacity_Missing;
	}
	bound := scratch;
	bound.children.length = i32(child_capacity);
	bound.continuations.length = i32(parent_capacity);
	bound.candidate_refs.length = i32(candidate_capacity);
	batcher.children = bound.children;
	batcher.continuations = bound.continuations;
	batcher.candidate_refs = bound.candidate_refs;
	return .Ok;
}

collision_batcher_reset_task_lists :: proc "contextless" (batcher: ^Collision_Batcher)
{
	for task_index in 0 ..< MAXIMUM_COLLISION_TASK_COUNT
	{
		batcher.task_heads[task_index] = -1;
		batcher.task_tails[task_index] = -1;
		batcher.subtask_heads[task_index] = -1;
		batcher.subtask_tails[task_index] = -1;
	}
}

collision_batcher_return_scratch :: proc (
	pool: ^util.Buffer_Pool, scratch: ^Collision_Batcher_Scratch,
)
{
	if pool == nil || scratch == nil
	{
		return;
	}
	if scratch.candidate_refs.memory != nil
	{
		collision_candidate_storage_return(pool, &scratch.candidate_refs);
	}
	if scratch.continuations.memory != nil
	{
		physics_return_buffer(pool, &scratch.continuations);
	}
	collision_child_storage_return(pool, &scratch.children);
	scratch^ = {};
}

collision_batcher_take_scratch :: proc (
	pool: ^util.Buffer_Pool, parent_capacity, child_capacity: int,
) -> (Collision_Batcher_Scratch, Physics_Status)
{
	if pool == nil || parent_capacity <= 0 || child_capacity <= 0 ||
	child_capacity > max(int) / MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		return {}, .Invalid_Argument;
	}
	candidate_capacity := child_capacity * MAXIMUM_MANIFOLD_CONTACT_COUNT;
	scratch: Collision_Batcher_Scratch;
	memory_status: util.Memory_Status;
	scratch.children.length = i32(child_capacity);
	if child_capacity <= COLLISION_BATCHER_SPLIT_CHILD_CAPACITY_MAX
	{
		scratch.children.format = .Split;
		scratch.children.headers, memory_status = util.buffer_pool_take_at_least(
			pool, Collision_Child_Manifold_Header, child_capacity,
		);
		if memory_status == .Ok
		{
			scratch.children.offsets_a, memory_status = util.buffer_pool_take_at_least(
				pool, util.Vector3, child_capacity,
			);
		}
		if memory_status == .Ok
		{
			scratch.children.contact_data, memory_status = util.buffer_pool_take_at_least(
				pool, Collision_Child_Contact_Data, candidate_capacity,
			);
		}
		if memory_status == .Ok
		{
			scratch.children.depths, memory_status = util.buffer_pool_take_at_least(
				pool, f32, candidate_capacity,
			);
		}
		if memory_status == .Ok
		{
			scratch.children.subpairs, memory_status = util.buffer_pool_take_at_least(
				pool, Collision_Batcher_Subpair, child_capacity,
			);
		}
		if memory_status == .Ok
		{
			scratch.children.next, memory_status = util.buffer_pool_take_at_least(
				pool, i32, child_capacity,
			);
		}
	}
	else
	{
		scratch.children.format = .Combined;
		scratch.children.combined, memory_status = util.buffer_pool_take_at_least(
			pool, Collision_Batcher_Child_Scratch, child_capacity,
		);
	}
	if memory_status != .Ok
	{
		collision_batcher_return_scratch(pool, &scratch);
		return {}, physics_memory_status(memory_status);
	}
	scratch.continuations, memory_status = util.buffer_pool_take_at_least(
		pool, Collision_Batcher_Continuation, parent_capacity,
	);
	if memory_status != .Ok
	{
		collision_batcher_return_scratch(pool, &scratch);
		return {}, physics_memory_status(memory_status);
	}
	candidate_status: Physics_Status;
	scratch.candidate_refs, candidate_status = collision_candidate_storage_take(
		pool, child_capacity, candidate_capacity,
	);
	if candidate_status != .Ok
	{
		collision_batcher_return_scratch(pool, &scratch);
		return {}, candidate_status;
	}
	return scratch, .Ok;
}

collision_batcher_initialize_bound :: proc "contextless" (
	batcher: ^Collision_Batcher,
	pair_storage: util.Buffer(Collision_Batcher_Pair),
	shapes: ^Shape_Registry,
	tasks: ^Collision_Task_Registry,
	procedures: Collision_Result_Procedures,
	user_context: rawptr,
	traversal_pool: ^util.Buffer_Pool,
	scratch: Collision_Batcher_Scratch,
	child_capacity: int,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Uninitialized ||
	pair_storage.memory == nil || pair_storage.length <= 0 ||
	shapes == nil || shapes.state != .Allocated ||
	tasks == nil || tasks.state != .Ready ||
	traversal_pool == nil || traversal_pool.state != .Ready ||
	procedures.pair_completed == nil
	{
		return .Invalid_Argument;
	}
	batcher^ = {
		pairs=pair_storage,
		shapes=shapes,
		tasks=tasks,
		traversal_pool=traversal_pool,
		procedures=procedures,
		user_context=user_context,
		state=.Ready,
	};
	bind_status := collision_batcher_bind_scratch(
		batcher, scratch, int(pair_storage.length), child_capacity,
	);
	if bind_status != .Ok
	{
		batcher^ = {};
		return bind_status;
	}
	collision_batcher_reset_task_lists(batcher);
	return .Ok;
}

collision_batcher_initialize :: proc (
	batcher: ^Collision_Batcher,
	pair_storage: util.Buffer(Collision_Batcher_Pair),
	shapes: ^Shape_Registry,
	tasks: ^Collision_Task_Registry,
	procedures: Collision_Result_Procedures,
	user_context: rawptr,
	traversal_pool: ^util.Buffer_Pool,
	child_capacity: int = 0,
) -> Physics_Status
{
	if shapes == nil || shapes.pool == nil || traversal_pool == nil ||
	traversal_pool.state != .Ready || pair_storage.length <= 0
	{
		return .Invalid_Argument;
	}
	parent_capacity := int(pair_storage.length);
	bound_child_capacity := child_capacity;
	if bound_child_capacity <= 0
	{
		if parent_capacity > max(int) / 16
		{
			return .Capacity_Missing;
		}
		bound_child_capacity = max(1024, parent_capacity * 16);
	}
	scratch, scratch_status := collision_batcher_take_scratch(
		shapes.pool, parent_capacity, bound_child_capacity,
	);
	if scratch_status != .Ok
	{
		return scratch_status;
	}
	status := collision_batcher_initialize_bound(
		batcher, pair_storage, shapes, tasks, procedures, user_context,
		traversal_pool, scratch, bound_child_capacity,
	);
	if status != .Ok
	{
		collision_batcher_return_scratch(shapes.pool, &scratch);
	}
	return status;
}

collision_batcher_add :: proc "contextless" (
	batcher: ^Collision_Batcher,
	shape_a, shape_b: Typed_Index,
	pose_a, pose_b: Rigid_Pose,
	speculative_margin: f32,
	pair_id: i32,
	velocity_a: Body_Velocity = {}, velocity_b: Body_Velocity = {},
	dt: f32 = 0, maximum_expansion: f32 = 0,
) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready || typed_index_state(shape_a) != .Present ||
	typed_index_state(shape_b) != .Present || speculative_margin < 0 || pair_id < 0 ||
	dt < 0 || maximum_expansion < 0
	{
		return .Invalid_Argument;
	}
	type_a := int(typed_index_type(shape_a));
	type_b := int(typed_index_type(shape_b));
	task, reference, lookup_status := collision_task_registry_lookup(batcher.tasks, type_a, type_b);
	if lookup_status == .Not_Found
	{
		_, _, resolve_a_status := shape_registry_resolve(batcher.shapes, shape_a);
		if resolve_a_status != .Ok
		{
			return resolve_a_status;
		}
		_, _, resolve_b_status := shape_registry_resolve(batcher.shapes, shape_b);
		if resolve_b_status != .Ok
		{
			return resolve_b_status;
		}
		if batcher.stored_pair_completed != nil
		{
			empty := Collision_Stored_Manifold{kind=.Convex};
			return batcher.stored_pair_completed(batcher.user_context, pair_id, &empty);
		}
		empty := Manifold_Result{kind=.Convex};
		return batcher.procedures.pair_completed(batcher.user_context, pair_id, &empty);
	}
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if batcher.pair_count >= int(batcher.pairs.length)
	{
		return .Capacity_Missing;
	}
	if task.kind == .Convex && collision_task_wide_test_state(task) == .Missing
	{
		return .Invalid_Description;
	}
	shape_data_a, _, resolve_a_status := shape_registry_resolve(batcher.shapes, shape_a);
	if resolve_a_status != .Ok
	{
		return resolve_a_status;
	}
	shape_data_b, _, resolve_b_status := shape_registry_resolve(batcher.shapes, shape_b);
	if resolve_b_status != .Ok
	{
		return resolve_b_status;
	}
	pair_index := i32(batcher.pair_count);
	pair := &batcher.pairs.memory[batcher.pair_count];
	pair.input.shape_a = shape_a;
	pair.input.shape_b = shape_b;
	pair.input.shape_data_a = shape_data_a;
	pair.input.shape_data_b = shape_data_b;
	pair.input.pose_a = pose_a;
	pair.input.pose_b = pose_b;
	pair.input.velocity_a = velocity_a;
	pair.input.velocity_b = velocity_b;
	pair.input.speculative_margin = speculative_margin;
	pair.input.dt = dt;
	pair.input.maximum_expansion = maximum_expansion;
	pair.pair_id = pair_id;
	pair.input.next_in_task = -1;
	pair.input.order = reference.order;
	pair.result_state = .Pending;
	if batcher.task_tails[reference.task_id] >= 0
	{
		batcher.pairs.memory[batcher.task_tails[reference.task_id]].input.next_in_task = pair_index;
	}
	else
	{
		batcher.task_heads[reference.task_id] = pair_index;
	}
	batcher.task_tails[reference.task_id] = pair_index;
	batcher.pair_count += 1;
	return .Ok;
}

collision_batcher_flush :: proc "contextless" (batcher: ^Collision_Batcher) -> Physics_Status
{
	if batcher == nil || batcher.state != .Ready
	{
		return .Invalid_Argument;
	}
	batcher.state = .Flushing;
	for task_index in 0 ..< batcher.tasks.task_count
	{
		if batcher.task_heads[task_index] < 0
		{
			continue;
		}
		task := &batcher.tasks.tasks[task_index];
		status := collision_batcher_execute_task_batches(batcher, task, batcher.task_heads[task_index]);
		if status != .Ok
		{
			batcher.state = .Faulted;
			return status;
		}
	}
	for task_index in 0 ..< batcher.tasks.task_count
	{
		if batcher.subtask_heads[task_index] < 0
		{
			continue;
		}
		task := &batcher.tasks.tasks[task_index];
		status := collision_batcher_execute_subtask_batches(batcher, task, batcher.subtask_heads[task_index]);
		if status != .Ok
		{
			batcher.state = .Faulted;
			return status;
		}
	}
	for continuation_index in 0 ..< batcher.continuation_count
	{
		// preserve the ordinary inlined reduction now also reused by selected nested tasks
		status := #force_inline collision_batcher_finish_continuation(batcher, continuation_index);
		if status != .Ok
		{
			batcher.state = .Faulted;
			return status;
		}
	}
	for pair_index in 0 ..< batcher.pair_count
	{
		pair := &batcher.pairs.memory[pair_index];
		if pair.result_state != .Complete
		{
			batcher.state = .Faulted;
			return .Invalid_Description;
		}
		status: Physics_Status;
		if batcher.stored_pair_completed != nil
		{
			status = batcher.stored_pair_completed(
				batcher.user_context, pair.pair_id, &pair.result,
			);
		}
		else
		{
			status = collision_batcher_complete_materialized_pair(batcher, pair);
		}
		if status != .Ok
		{
			batcher.state = .Faulted;
			return status;
		}
	}
	batcher.pair_count = 0;
	batcher.subpair_count = 0;
	batcher.continuation_count = 0;
	batcher.child_count = 0;
	batcher.child_contact_count = 0;
	collision_batcher_reset_task_lists(batcher);
	batcher.state = .Ready;
	return .Ok;
}

collision_batcher_reset_fault :: proc "contextless" (batcher: ^Collision_Batcher) -> Physics_Status
{
	if batcher == nil || batcher.state != .Faulted
	{
		return .Invalid_Argument;
	}
	batcher.pair_count = 0;
	batcher.subpair_count = 0;
	batcher.continuation_count = 0;
	batcher.child_count = 0;
	batcher.child_contact_count = 0;
	collision_batcher_reset_task_lists(batcher);
	batcher.state = .Ready;
	return .Ok;
}

collision_batcher_dispose :: proc (batcher: ^Collision_Batcher) -> Physics_Status
{
	if batcher == nil || batcher.state == .Disposed || batcher.state == .Flushing
	{
		return .Disposed;
	}
	if batcher.shapes != nil && batcher.shapes.pool != nil
	{
		if batcher.candidate_refs.id >= 0
		{
			collision_candidate_storage_return(batcher.shapes.pool, &batcher.candidate_refs);
		}
		if batcher.continuations.id >= 0
		{
			physics_return_buffer(batcher.shapes.pool, &batcher.continuations);
		}
		collision_child_storage_return(batcher.shapes.pool, &batcher.children);
	}
	batcher^ = {state=.Disposed};
	return .Ok;
}
