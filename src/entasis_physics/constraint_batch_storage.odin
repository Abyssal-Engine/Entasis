// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"
import "core:simd"

Constraint_Location :: struct
{
	set_index:           i32,
	batch_index:         i32,
	type_id:             i32,
	index_in_type_batch: i32,
}

Constraint_Reference :: struct
{
	handle:                  Constraint_Handle,
	body_handles:            [4]Body_Handle,
	encoded_body_references: [4]i32,
	body_count:              i32,
}

Type_Batch_State :: enum u8
{
	Unallocated,
	Allocated,
}

Type_Batch :: struct
{
	type_id:              i32,
	body_count:           i32,
	prestep_bundle_size:  i32,
	impulse_bundle_size:  i32,
	body_references:      util.Buffer(u8),
	prestep_data:         util.Buffer(u8),
	accumulated_impulses: util.Buffer(u8),
	index_to_handle:                  util.Buffer(i32),
	integration_flags_offset:         i32,
	integration_flag_word_count:      i32,
	count:                            i32,
	has_integration_responsibilities: Reference_State,
	state:                            Type_Batch_State,
}

Constraint_Batch :: struct
{
	type_batches:           util.Buffer(Type_Batch),
	type_id_to_batch_index: [CONSTRAINT_TYPE_ID_CAPACITY]i16,
	body_reference_counts:  util.Buffer(i32),
	type_batch_count:       i32,
	constraint_count:       i32,
	work_block_start:       i32,
	work_block_count:       i32,
	state:                  Body_Set_State,
}

Constraint_Set :: struct
{
	batches:          util.Buffer(Constraint_Batch),
	batch_count:      i32,
	constraint_count: i32,
	state:            Body_Set_State,
}

Sequential_Fallback_Batch :: struct
{
	batch_index:                    i32,
	constraint_count:               i32,
	dynamic_body_constraint_counts: ^util.Buffer(i32),
	state:                          Reference_State,
}

constraint_location_missing :: proc "contextless" () -> Constraint_Location
{
	return {-1, -1, -1, -1};
}

constraint_batch_initialize :: proc (
	batch: ^Constraint_Batch, body_handle_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if batch == nil || body_handle_capacity <= 0 || pool == nil || batch.state == .Allocated
	{
		return .Invalid_Argument;
	}
	type_batches, type_status := util.buffer_pool_take_at_least(pool, Type_Batch, CONSTRAINT_TYPE_ID_CAPACITY);
	if type_status != .Ok
	{
		return physics_memory_status(type_status);
	}
	_ = util.buffer_clear(type_batches, 0, int(type_batches.length));
	counts, count_status := util.buffer_pool_take_at_least(pool, i32, body_handle_capacity);
	if count_status != .Ok
	{
		physics_return_buffer(pool, &type_batches);
		return physics_memory_status(count_status);
	}
	_ = util.buffer_clear(counts, 0, int(counts.length));
	batch^ = {type_batches=type_batches, body_reference_counts=counts, state=.Allocated};
	for type_id in 0 ..< CONSTRAINT_TYPE_ID_CAPACITY
	{
		batch.type_id_to_batch_index[type_id] = -1;
	}
	return .Ok;
}

constraint_batch_ensure_body_capacity :: proc (
	batch: ^Constraint_Batch, body_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if batch == nil || batch.state != .Allocated || body_capacity <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	if body_capacity <= int(batch.body_reference_counts.length)
	{
		return .Ok;
	}
	counts, count_status := util.buffer_pool_take_at_least(pool, i32, body_capacity);
	if count_status != .Ok
	{
		return physics_memory_status(count_status);
	}
	_ = util.buffer_clear(counts, 0, int(counts.length));
	copy_status := util.buffer_copy(
		util.buffer_view(batch.body_reference_counts), 0, util.buffer_view(counts), 0, int(batch.body_reference_counts.length),
	);
	if copy_status != .Ok
	{
		physics_return_buffer(pool, &counts);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(pool, &batch.body_reference_counts);
	batch.body_reference_counts = counts;
	return .Ok;
}

type_batch_ensure_capacity :: proc (
	type_batch: ^Type_Batch, capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if type_batch == nil || type_batch.state != .Allocated || capacity <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	if capacity <= int(type_batch.index_to_handle.length)
	{
		return .Ok;
	}
	new_handles, handle_status := util.buffer_pool_take_at_least(pool, i32, capacity);
	if handle_status != .Ok
	{
		return physics_memory_status(handle_status);
	}
	bundle_capacity :=
		(int(new_handles.length) + util.PRODUCTION_LANE_COUNT - 1) /
		util.PRODUCTION_LANE_COUNT;
	body_bytes := bundle_capacity * int(type_batch.body_count) * size_of(util.I32x8);
	prestep_bytes := bundle_capacity * int(type_batch.prestep_bundle_size);
	impulse_bytes := bundle_capacity * int(type_batch.impulse_bundle_size);
	new_bodies, body_status := util.buffer_pool_take_at_least(pool, u8, body_bytes);
	if body_status != .Ok
	{
		physics_return_buffer(pool, &new_handles);
		return physics_memory_status(body_status);
	}
	new_prestep, prestep_status := util.buffer_pool_take_at_least(pool, u8, prestep_bytes);
	if prestep_status != .Ok
	{
		physics_return_buffer(pool, &new_bodies);
		physics_return_buffer(pool, &new_handles);
		return physics_memory_status(prestep_status);
	}
	new_impulses, impulse_status := util.buffer_pool_take_at_least(pool, u8, impulse_bytes);
	if impulse_status != .Ok
	{
		physics_return_buffer(pool, &new_prestep);
		physics_return_buffer(pool, &new_bodies);
		physics_return_buffer(pool, &new_handles);
		return physics_memory_status(impulse_status);
	}
	_ = util.buffer_clear(new_bodies, 0, int(new_bodies.length));
	_ = util.buffer_clear(new_prestep, 0, int(new_prestep.length));
	_ = util.buffer_clear(new_impulses, 0, int(new_impulses.length));
	for index in 0 ..< new_handles.length
	{
		new_handles.memory[index] = -1;
	}
	if type_batch.body_references.memory != nil
	{
		old_bundle_count := (int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
		intrinsics.mem_copy(
			new_bodies.memory,
			type_batch.body_references.memory,
			old_bundle_count * int(type_batch.body_count) * size_of(util.I32x8)
		);
		intrinsics.mem_copy(
			new_prestep.memory,
			type_batch.prestep_data.memory,
			old_bundle_count * int(type_batch.prestep_bundle_size)
		);
		intrinsics.mem_copy(
			new_impulses.memory,
			type_batch.accumulated_impulses.memory,
			old_bundle_count * int(type_batch.impulse_bundle_size)
		);
		intrinsics.mem_copy(
			new_handles.memory,
			type_batch.index_to_handle.memory,
			int(type_batch.count) * size_of(i32)
		);
		physics_return_buffer(pool, &type_batch.body_references);
		physics_return_buffer(pool, &type_batch.prestep_data);
		physics_return_buffer(pool, &type_batch.accumulated_impulses);
		physics_return_buffer(pool, &type_batch.index_to_handle);
	}
	type_batch.body_references = new_bodies;
	type_batch.prestep_data = new_prestep;
	type_batch.accumulated_impulses = new_impulses;
	type_batch.index_to_handle = new_handles;
	return .Ok;
}

type_batch_target_capacity :: proc "contextless" (
	type_batch: ^Type_Batch, required_record_count: int,
) -> (int, Physics_Status)
{
	if type_batch == nil || type_batch.state != .Allocated || required_record_count <= 0
	{
		return 0, .Invalid_Argument;
	}
	current_capacity := int(type_batch.index_to_handle.length);
	if required_record_count <= current_capacity
	{
		return current_capacity, .Ok;
	}
	if current_capacity > max(int) / 2
	{
		return 0, .Capacity_Missing;
	}
	return max(current_capacity * 2, required_record_count), .Ok;
}

type_batch_accumulate_resize_slot_requirements :: proc (
	type_batch: ^Type_Batch, target_capacity: int,
	requirements: ^[util.BUFFER_POOL_POWER_COUNT]int,
) -> Physics_Status
{
	if type_batch == nil || type_batch.state != .Allocated || target_capacity <= 0 ||
		requirements == nil
	{
		return .Invalid_Argument;
	}
	if target_capacity <= int(type_batch.index_to_handle.length)
	{
		return .Ok;
	}
	handle_capacity, handle_capacity_status := util.buffer_pool_capacity_for_count(
		i32, target_capacity,
	);
	if handle_capacity_status != .Ok
	{
		return physics_memory_status(handle_capacity_status);
	}
	bundle_capacity :=
		(i64(handle_capacity) + i64(util.PRODUCTION_LANE_COUNT) - 1) /
		i64(util.PRODUCTION_LANE_COUNT);
	byte_counts := [4]i64{
		bundle_capacity * i64(type_batch.body_count) * i64(size_of(util.I32x8)),
		bundle_capacity * i64(type_batch.prestep_bundle_size),
		bundle_capacity * i64(type_batch.impulse_bundle_size),
		i64(handle_capacity) * i64(size_of(i32)),
	};
	for byte_count in byte_counts
	{
		if byte_count <= 0 || byte_count > i64(max(int))
		{
			return .Capacity_Missing;
		}
		power, power_status := util.buffer_pool_power_for_count(u8, int(byte_count));
		if power_status != .Ok
		{
			return physics_memory_status(power_status);
		}
		if requirements[power] == max(int)
		{
			return .Capacity_Missing;
		}
		requirements[power] += 1;
	}
	return .Ok;
}

type_batch_resize :: proc (
	type_batch: ^Type_Batch, capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if type_batch == nil || type_batch.state != .Allocated || capacity <= 0 ||
		pool == nil
	{
		return .Invalid_Argument;
	}
	target_capacity := max(capacity, int(type_batch.count));
	target_handle_capacity, handle_capacity_status := util.buffer_pool_capacity_for_count(
		i32, target_capacity,
	);
	if handle_capacity_status != .Ok
	{
		return physics_memory_status(handle_capacity_status);
	}
	target_bundle_count :=
		(target_handle_capacity + util.PRODUCTION_LANE_COUNT - 1) /
		util.PRODUCTION_LANE_COUNT;
	live_bundle_count :=
		(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
		util.PRODUCTION_LANE_COUNT;
	status := physics_resize_buffer_capacity(
		pool, &type_batch.body_references,
		target_bundle_count * int(type_batch.body_count) *
		size_of(util.I32x8),
		live_bundle_count * int(type_batch.body_count) *
		size_of(util.I32x8),
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &type_batch.prestep_data,
		target_bundle_count * int(type_batch.prestep_bundle_size),
		live_bundle_count * int(type_batch.prestep_bundle_size),
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &type_batch.accumulated_impulses,
		target_bundle_count * int(type_batch.impulse_bundle_size),
		live_bundle_count * int(type_batch.impulse_bundle_size),
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &type_batch.index_to_handle, target_handle_capacity,
		int(type_batch.count),
	);
	if status != .Ok
	{
		return status;
	}
	for index in type_batch.count ..< type_batch.index_to_handle.length
	{
		type_batch.index_to_handle.memory[index] = -1;
	}
	return .Ok;
}

constraint_batch_get_or_create_type_batch :: proc (
	batch: ^Constraint_Batch, type_record: ^Constraint_Type_Record, initial_capacity: int, pool: ^util.Buffer_Pool,
) -> (^Type_Batch, Physics_Status)
{
	if batch == nil || batch.state != .Allocated || type_record == nil || type_record.registration != .Present ||
		type_record.type_id < 0 || type_record.type_id >= CONSTRAINT_TYPE_ID_CAPACITY || initial_capacity <= 0
	{
		return nil, .Invalid_Argument;
	}
	type_id := int(type_record.type_id);
	index := int(batch.type_id_to_batch_index[type_id]);
	if index >= 0
	{
		return &batch.type_batches.memory[index], .Ok;
	}
	if int(batch.type_batch_count) >= int(batch.type_batches.length)
	{
		return nil, .Capacity_Missing;
	}
	index = int(batch.type_batch_count);
	type_batch := &batch.type_batches.memory[index];
	type_batch^ = {
		type_id=i32(type_id),
		body_count=type_record.body_count,
		prestep_bundle_size=type_record.prestep_bundle_size,
		impulse_bundle_size=type_record.impulse_bundle_size,
		state=.Allocated,
	};
	capacity_status := type_batch_ensure_capacity(type_batch, initial_capacity, pool);
	if capacity_status != .Ok
	{
		type_batch^ = {};
		return nil, capacity_status;
	}
	batch.type_batch_count += 1;
	batch.type_id_to_batch_index[type_id] = i16(index);
	return type_batch, .Ok;
}

type_batch_bundle_address :: #force_inline proc "contextless" (
	buffer: util.Buffer(u8),
	stride,
	bundle_index: int
) -> rawptr
{
	return rawptr(uintptr(buffer.memory) + uintptr(stride * bundle_index));
}

type_batch_prestep_bundle :: #force_inline proc "contextless" (type_batch: ^Type_Batch, index: int) -> rawptr
{
	return type_batch_bundle_address(
		type_batch.prestep_data,
		int(type_batch.prestep_bundle_size),
		index / util.PRODUCTION_LANE_COUNT
	);
}

type_batch_impulse_bundle :: #force_inline proc "contextless" (type_batch: ^Type_Batch, index: int) -> rawptr
{
	return type_batch_bundle_address(
		type_batch.accumulated_impulses,
		int(type_batch.impulse_bundle_size),
		index / util.PRODUCTION_LANE_COUNT
	);
}

type_batch_body_bundle :: #force_inline proc "contextless" (type_batch: ^Type_Batch, index: int) -> [^]util.I32x8
{
	stride := int(type_batch.body_count) * size_of(util.I32x8);
	return ([^]util.I32x8)(type_batch_bundle_address(
		type_batch.body_references,
		stride,
		index / util.PRODUCTION_LANE_COUNT
	));
}

type_batch_write_reference :: proc "contextless" (
	type_batch: ^Type_Batch, index: int, reference: Constraint_Reference,
) -> Physics_Status
{
	if type_batch == nil ||
		type_batch.state != .Allocated ||
		index < 0 ||
		index >= int(type_batch.index_to_handle.length) ||
		reference.body_count != type_batch.body_count
	{
		return .Invalid_Argument;
	}
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	for body_index in 0 ..< int(type_batch.body_count)
	{
		body_bundle[body_index] = simd.replace(
			body_bundle[body_index],
			lane,
			reference.encoded_body_references[body_index]
		);
	}
	type_batch.index_to_handle.memory[index] = reference.handle.value;
	return .Ok;
}

type_batch_write_reference_trusted :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, index: int, reference: Constraint_Reference,
)
{
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := ([^]i32)(type_batch_body_bundle(type_batch, index));
	for body_index in 0 ..< int(type_batch.body_count)
	{
		body_bundle[body_index * util.PRODUCTION_LANE_COUNT + lane] =
			reference.encoded_body_references[body_index];
	}
	type_batch.index_to_handle.memory[index] = reference.handle.value;
}

type_batch_read_reference :: proc "contextless" (
	type_batch: ^Type_Batch, bodies: ^Bodies, index: int,
) -> (Constraint_Reference, Physics_Status)
{
	if type_batch == nil || type_batch.state != .Allocated || bodies == nil || bodies.state != .Allocated ||
		index < 0 || index >= int(type_batch.count)
	{
		return {}, .Not_Found;
	}
	reference := Constraint_Reference{
		handle={type_batch.index_to_handle.memory[index]},
		body_count=type_batch.body_count,
	};
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< int(type_batch.body_count)
	{
		encoded := simd.extract(body_bundle[body_index], lane);
		body_index_in_set := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
		if encoded < 0 || body_index_in_set < 0 || body_index_in_set >= active.count
		{
			return {}, .Not_Found;
		}
		reference.encoded_body_references[body_index] = encoded;
		reference.body_handles[body_index] = active.index_to_handle.memory[body_index_in_set];
	}
	return reference, .Ok;
}

type_batch_read_reference_trusted :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, bodies: ^Bodies, index: int,
) -> Constraint_Reference
{
	reference := Constraint_Reference{
		handle={type_batch.index_to_handle.memory[index]},
		body_count=type_batch.body_count,
	};
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	active := &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< int(type_batch.body_count)
	{
		encoded := simd.extract(body_bundle[body_index], lane);
		reference.encoded_body_references[body_index] = encoded;
		reference.body_handles[body_index] = active.index_to_handle.memory[int(u32(encoded) & BODY_REFERENCE_INDEX_MASK)];
	}
	return reference;
}

type_batch_update_body_memory_move :: proc "contextless" (
	type_batch: ^Type_Batch, index, body_index_in_constraint, original_body_index, new_body_index: int,
) -> (Body_Mobility, Physics_Status)
{
	if type_batch == nil || type_batch.state != .Allocated || index < 0 || index >= int(type_batch.count) ||
		body_index_in_constraint < 0 || body_index_in_constraint >= int(type_batch.body_count) ||
		original_body_index < 0 || new_body_index < 0
	{
		return .Dynamic, .Invalid_Argument;
	}
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	encoded := simd.extract(body_bundle[body_index_in_constraint], lane);
	if int(u32(encoded) & BODY_REFERENCE_INDEX_MASK) != original_body_index
	{
		return .Dynamic, .Invalid_Argument;
	}
	mobility := body_reference_mobility(encoded);
	updated := i32(u32(new_body_index) | (u32(encoded) & BODY_REFERENCE_KINEMATIC_MASK));
	body_bundle[body_index_in_constraint] = simd.replace(body_bundle[body_index_in_constraint], lane, updated);
	return mobility, .Ok;
}

type_batch_update_body_mobility :: proc "contextless" (
	type_batch: ^Type_Batch, index, body_index_in_constraint: int, mobility: Body_Mobility,
) -> Physics_Status
{
	if type_batch == nil || type_batch.state != .Allocated || index < 0 || index >= int(type_batch.count) ||
		body_index_in_constraint < 0 || body_index_in_constraint >= int(type_batch.body_count)
	{
		return .Invalid_Argument;
	}
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	encoded := simd.extract(body_bundle[body_index_in_constraint], lane);
	updated := u32(encoded) & BODY_REFERENCE_INDEX_MASK;
	if mobility == .Kinematic
	{
		updated |= BODY_REFERENCE_KINEMATIC_MASK;
	}
	body_bundle[body_index_in_constraint] = simd.replace(body_bundle[body_index_in_constraint], lane, i32(updated));
	return .Ok;
}

type_batch_clear_reference :: proc "contextless" (type_batch: ^Type_Batch, index: int)
{
	lane := index % util.PRODUCTION_LANE_COUNT;
	body_bundle := type_batch_body_bundle(type_batch, index);
	for body_index in 0 ..< int(type_batch.body_count)
	{
		body_bundle[body_index] = simd.replace(body_bundle[body_index], lane, -1);
	}
	type_batch.index_to_handle.memory[index] = -1;
}

constraint_batch_body_fit :: proc "contextless" (
	batch: ^Constraint_Batch, reference: ^Constraint_Reference,
) -> Reference_State
{
	if batch == nil || batch.state != .Allocated || reference == nil
	{
		return .Missing;
	}
	for body_index in 0 ..< int(reference.body_count)
	{
		if u32(reference.encoded_body_references[body_index]) >= BODY_REFERENCE_KINEMATIC_MASK
		{
			continue;
		}
		handle := reference.body_handles[body_index].value;
		if handle < 0 ||
			int(handle) >= int(batch.body_reference_counts.length) ||
			batch.body_reference_counts.memory[handle] != 0
		{
			return .Missing;
		}
	}
	return .Present;
}

constraint_batch_add_body_references :: proc "contextless" (
	batch: ^Constraint_Batch, reference: ^Constraint_Reference,
)
{
	for body_index in 0 ..< int(reference.body_count)
	{
		if u32(reference.encoded_body_references[body_index]) < BODY_REFERENCE_KINEMATIC_MASK
		{
			batch.body_reference_counts.memory[reference.body_handles[body_index].value] += 1;
		}
	}
}

constraint_batch_remove_body_references :: proc "contextless" (
	batch: ^Constraint_Batch, reference: ^Constraint_Reference,
)
{
	for body_index in 0 ..< int(reference.body_count)
	{
		if u32(reference.encoded_body_references[body_index]) < BODY_REFERENCE_KINEMATIC_MASK
		{
			batch.body_reference_counts.memory[reference.body_handles[body_index].value] -= 1;
		}
	}
}

sequential_fallback_add_body_references :: proc "contextless" (
	sequential_batch: ^Sequential_Fallback_Batch, reference: ^Constraint_Reference,
)
{
	counts := sequential_batch.dynamic_body_constraint_counts;
	for body_index in 0 ..< int(reference.body_count)
	{
		encoded := reference.encoded_body_references[body_index];
		if u32(encoded) < BODY_REFERENCE_KINEMATIC_MASK
		{
			counts.memory[int(u32(encoded) & BODY_REFERENCE_INDEX_MASK)] += 1;
		}
	}
}

sequential_fallback_remove_body_references :: proc "contextless" (
	sequential_batch: ^Sequential_Fallback_Batch, reference: ^Constraint_Reference,
)
{
	counts := sequential_batch.dynamic_body_constraint_counts;
	for body_index in 0 ..< int(reference.body_count)
	{
		encoded := reference.encoded_body_references[body_index];
		if u32(encoded) < BODY_REFERENCE_KINEMATIC_MASK
		{
			counts.memory[int(u32(encoded) & BODY_REFERENCE_INDEX_MASK)] -= 1;
		}
	}
}

sequential_fallback_update_body_memory_move :: proc "contextless" (
	sequential_batch: ^Sequential_Fallback_Batch, original_body_index, new_body_index: int,
)
{
	counts := sequential_batch.dynamic_body_constraint_counts;
	counts.memory[new_body_index] += counts.memory[original_body_index];
	counts.memory[original_body_index] = 0;
}

constraint_batch_dispose :: proc (batch: ^Constraint_Batch, pool: ^util.Buffer_Pool)
{
	if batch == nil || batch.state != .Allocated
	{
		return;
	}
	for index in 0 ..< int(batch.type_batch_count)
	{
		type_batch := &batch.type_batches.memory[index];
		if type_batch.state == .Allocated
		{
			physics_return_buffer(pool, &type_batch.body_references);
			physics_return_buffer(pool, &type_batch.prestep_data);
			physics_return_buffer(pool, &type_batch.accumulated_impulses);
			physics_return_buffer(pool, &type_batch.index_to_handle);
		}
	}
	physics_return_buffer(pool, &batch.body_reference_counts);
	physics_return_buffer(pool, &batch.type_batches);
	batch^ = {};
}
