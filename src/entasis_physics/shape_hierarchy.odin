package entasis_physics

import util "entasis:entasis_utilities"

// only compounds containing a nonconvex child acquire hierarchy metadata.
// geometry and Compound_Child remain unchanged. the depth is admitted at shape
// registration. immutable descendants form a DAG, not a mutable object graph
MAXIMUM_COMPOUND_HIERARCHY_DEPTH :: 32;
Shape_Hierarchy_Cache :: struct
{
	depths: [2]util.Buffer(u8),
	count: int,
	allocation_id: i32,
}

shape_hierarchy_depth :: proc "contextless" (registry: ^Shape_Registry, shape: Typed_Index) -> int
{
	if registry.hierarchy == nil
	{
		return 0;
	}
	type_id: int = int(typed_index_type(shape));
	if type_id != COMPOUND_TYPE_ID && type_id != BIG_COMPOUND_TYPE_ID
	{
		return 0;
	}
	depths: util.Buffer(u8) = registry.hierarchy.depths[type_id - COMPOUND_TYPE_ID];
	index: int = int(typed_index_index(shape));
	if index >= int(depths.length)
	{
		return 0;
	}
	return int(depths.memory[index]);
}

shape_hierarchy_raw_depth :: proc "contextless" (registry: ^Shape_Registry, shape: rawptr, type_id: int) -> int
{
	if registry.hierarchy == nil || (type_id != COMPOUND_TYPE_ID && type_id != BIG_COMPOUND_TYPE_ID)
	{
		return 0;
	}
	batch: ^Shape_Batch = &registry.batches[type_id];
	// raw views from current queries point into the registered batch. a caller's
	// unregistered description is not a hierarchy handle and is never published
	offset: uintptr = uintptr(shape) - uintptr(batch.data.memory);
	if offset >= uintptr(batch.data.length) || offset % uintptr(batch.stride) != 0
	{
		return 0;
	}
	index: int = int(offset / uintptr(batch.stride));
	depths: util.Buffer(u8) = registry.hierarchy.depths[type_id - COMPOUND_TYPE_ID];
	if index >= int(depths.length)
	{
		return 0;
	}
	return int(depths.memory[index]);
}

shape_hierarchy_prepare :: proc (registry: ^Shape_Registry, type_id, index: int) -> Physics_Status
{
	if registry.hierarchy == nil
	{
		memory: util.Buffer(u8);
		memory_status: util.Memory_Status;
		memory, memory_status = util.buffer_pool_take_at_least(registry.pool, u8, size_of(Shape_Hierarchy_Cache));
		if memory_status != .Ok
		{
			return physics_memory_status(memory_status);
		}
		registry.hierarchy = (^Shape_Hierarchy_Cache)(memory.memory);
		registry.hierarchy^ = {allocation_id=memory.id};
	}
	depths: ^util.Buffer(u8) = &registry.hierarchy.depths[type_id - COMPOUND_TYPE_ID];
	if index < int(depths.length)
	{
		return .Ok;
	}
	capacity: int = max(index + 1, max(16, int(depths.length) * 2));
	replacement: util.Buffer(u8);
	memory_status: util.Memory_Status;
	replacement, memory_status = util.buffer_pool_take_at_least(registry.pool, u8, capacity);
	if memory_status != .Ok
	{
		return physics_memory_status(memory_status);
	}
	_ = util.buffer_clear(replacement, 0, int(replacement.length));
	for cursor in 0 ..< int(depths.length)
	{
		replacement.memory[cursor] = depths.memory[cursor];
	}
	if depths.memory != nil
	{
		physics_return_buffer(registry.pool, depths);
	}
	depths^ = replacement;
	return .Ok;
}

shape_hierarchy_retire :: proc "contextless" (registry: ^Shape_Registry, shape: Typed_Index)
{
	if shape_hierarchy_depth(registry, shape) == 0
	{
		return;
	}
	registry.hierarchy.depths[int(typed_index_type(shape)) - COMPOUND_TYPE_ID].memory[typed_index_index(shape)] = 0;
	registry.hierarchy.count -= 1;
}

shape_hierarchy_dispose :: proc (registry: ^Shape_Registry)
{
	if registry.hierarchy == nil
	{
		return;
	}
	id: i32 = registry.hierarchy.allocation_id;
	for index in 0 ..< 2
	{
		physics_return_buffer(registry.pool, &registry.hierarchy.depths[index]);
	}
	_ = util.buffer_pool_return_unsafely(registry.pool, id);
	registry.hierarchy = nil;
}

// ordinary child access remains immutable. the descriptor is copied rather
// than retaining a pointer into a relocating shape batch
shape_hierarchy_child :: proc "contextless" (
	registry: ^Shape_Registry, shape: rawptr, type_id, index: int, pose: Rigid_Pose,
) -> (Typed_Index, Collision_Child, Physics_Status)
{
	children: util.Buffer(Compound_Child);
	if type_id == COMPOUND_TYPE_ID
	{
		children = (^Compound)(shape).children;
	}
	else
	{
		children = (^Big_Compound)(shape).children;
	}
	child: Compound_Child = children.memory[index];
	data: rawptr;
	batch: ^Shape_Batch;
	status: Physics_Status;
	data, batch, status = shape_registry_resolve(registry, child.shape_index);
	if status != .Ok
	{
		return {}, {}, status;
	}
	return child.shape_index, {
		shape=data, type_id=int(typed_index_type(child.shape_index)),
		pose=rigid_pose_concatenate({position=child.local_position, orientation=child.local_orientation}, pose),
		child_index=index,
	}, .Ok;
}

// cold topological teardown. references include both retained parents and
// external body/static owners. a blocked DAG is reported, never force-freed
shape_registry_clear_hierarchy :: proc (registry: ^Shape_Registry) -> Physics_Status
{
	for
	{
		remaining, removed: int;
		for position in 0 ..< registry.registered_type_count
		{
			type_id: int = shape_registry_teardown_type(registry.registered_type_count, position);
			batch: ^Shape_Batch = &registry.batches[type_id];
			for slot in 0 ..< batch.slot_count
			{
				if batch.references.memory[slot] < 0
				{
					continue;
				}
				if batch.references.memory[slot] > 0
				{
					remaining += 1;
					continue;
				}
				index: Typed_Index;
				status: Physics_Status;
				index, status = typed_index_create(type_id, slot);
				if status != .Ok
				{
					return status;
				}
				status = shape_registry_remove(registry, index);
				if status != .Ok
				{
					return status;
				}
				removed += 1;
			}
		}
		if remaining == 0
		{
			break;
		}
		if removed == 0
		{
			return .Shape_In_Use;
		}
	}
	for type_id in 0 ..< registry.registered_type_count
	{
		batch: ^Shape_Batch = &registry.batches[type_id];
		util.id_pool_clear(&batch.ids);
		batch.slot_count = 0;
		batch.active_count = 0;
	}
	return .Ok;
}
