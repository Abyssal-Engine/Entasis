package entasis

import physics "entasis:entasis_physics"

// Convex_Hull is the exact registered convex-hull storage type
Convex_Hull :: physics.Convex_Hull;
// Compound is the exact small compound storage type
Compound :: physics.Compound;
// Big_Compound is the tree-accelerated compound storage type
Big_Compound :: physics.Big_Compound;
// Mesh is the exact registered mesh storage type
Mesh :: physics.Mesh;
// Shape_Info describes one currently registered shape without exposing mutable
// registry metadata
Shape_Info :: struct
{
	handle:          Shape_Handle,
	type_id:         Shape_Type_ID,
	value_size:      int,
	value_alignment: int,
	reference_count: int,
}

// shape_inspect returns the registered type, value layout and live reference
// count of a shape. the reference count includes bodies, statics and compound
// child references
shape_inspect :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (Shape_Info, Status)
{
	registry, status := shape_registry_from_world(world);
	if status != .Ok
	{
		return {}, status;
	}
	_, batch, resolve_status := physics.shape_registry_resolve(registry, handle);
	if resolve_status != .Ok
	{
		return {}, resolve_status;
	}
	index := int(physics.typed_index_index(handle));
	return {
		handle=handle,
		type_id=Shape_Type_ID(physics.typed_index_type(handle)),
		value_size=batch.metadata.size,
		value_alignment=batch.metadata.alignment,
		reference_count=int(batch.references.memory[index]),
	}, .Ok;
}

// shape_borrow_raw returns a read-only pointer to the registered shape value.
// use it only on the owner thread while the world is idle. it transfers no
// ownership and is invalidated by shape removal, world_clear or world_destroy.
// shape storage growth or resizing can also relocate it while the handle stays
// valid, including another registration or import of the same shape type.
// reacquire it after shape registration or import, world_ensure_capacity,
// world_resize or any other operation that can resize its storage. do not
// mutate the value
shape_borrow_raw :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (rawptr, Shape_Info, Status)
{
	registry, status := shape_registry_from_world(world);
	if status != .Ok
	{
		return nil, {}, status;
	}
	value, batch, resolve_status := physics.shape_registry_resolve(registry, handle);
	if resolve_status != .Ok
	{
		return nil, {}, resolve_status;
	}
	index := int(physics.typed_index_index(handle));
	return value, {
		handle=handle,
		type_id=Shape_Type_ID(physics.typed_index_type(handle)),
		value_size=batch.metadata.size,
		value_alignment=batch.metadata.alignment,
		reference_count=int(batch.references.memory[index]),
	}, .Ok;
}

// shape_borrow_typed returns a read-only typed pointer after validating the
// compile-time type against the handle's registered type and value layout.
// it has the same ownership, thread and invalidation rules as shape_borrow_raw
shape_borrow_typed :: proc "contextless" (
	world: ^World, handle: Shape_Handle, $T: typeid,
) -> (^T, Status)
{
	expected := shape_type_id(T);
	if expected == SHAPE_TYPE_INVALID
	{
		return nil, .Invalid_Argument;
	}
	value, info, status := shape_borrow_raw(world, handle);
	if status != .Ok
	{
		return nil, status;
	}
	if info.type_id != expected || info.value_size != size_of(T) ||
	info.value_alignment != align_of(T)
	{
		return nil, .Invalid_Argument;
	}
	return (^T)(value), .Ok;
}

// shape_bounds computes local bounds for a registered shape at an orientation
shape_bounds :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
	orientation: Quaternion = {0, 0, 0, 1},
) -> (Shape_Bounds, Status)
{
	registry, status := shape_registry_from_world(world);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.shape_registry_compute_bounds(registry, handle, orientation);
}

// shape_ray tests one registered shape directly, without inserting it into a
// world or traversing the broad phase
shape_ray :: proc "contextless" (
	world: ^World, handle: Shape_Handle, shape_pose: Rigid_Pose, query: Ray,
) -> (Shape_Ray_Hit, Status)
{
	registry, status := shape_registry_from_world(world);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.shape_registry_ray_test(registry, handle, shape_pose, query);
}

@(private)
shape_recursive_upper_bound :: proc "contextless" (
	registry: ^physics.Shape_Registry, handle: Shape_Handle,
) -> (int, Status)
{
	value, _, status := physics.shape_registry_resolve(registry, handle);
	if status != .Ok
	{
		return 0, status;
	}
	type_id := int(physics.typed_index_type(handle));
	count := 1;
	if type_id == physics.COMPOUND_TYPE_ID || type_id == physics.BIG_COMPOUND_TYPE_ID
	{
		children := (^physics.Compound)(value).children;
		if type_id == physics.BIG_COMPOUND_TYPE_ID
		{
			children = (^physics.Big_Compound)(value).children;
		}
		for child_index in 0 ..< int(children.length)
		{
			child := children.memory[child_index];
			child_count, child_status := shape_recursive_upper_bound(registry, child.shape_index);
			if child_status != .Ok
			{
				return count, child_status;
			}
			if child_count > max(int) - count
			{
				return max(int), .Capacity_Missing;
			}
			count += child_count;
		}
	}
	return count, .Ok;
}

@(private)
shape_recursive_contains :: #force_inline proc "contextless" (
	handles: []Shape_Handle, count: int, handle: Shape_Handle,
) -> bool
{
	for index in 0 ..< count
	{
		if handles[index].packed == handle.packed
		{
			return true;
		}
	}
	return false;
}

@(private)
shape_recursive_collect :: proc "contextless" (
	registry: ^physics.Shape_Registry, handle: Shape_Handle,
	handles: []Shape_Handle, count: ^int,
) -> Status
{
	if shape_recursive_contains(handles, count^, handle)
	{
		return .Ok;
	}
	if count^ >= len(handles)
	{
		return .Capacity_Missing;
	}
	value, _, status := physics.shape_registry_resolve(registry, handle);
	if status != .Ok
	{
		return status;
	}
	handles[count^] = handle;
	count^ += 1;
	type_id := int(physics.typed_index_type(handle));
	if type_id != physics.COMPOUND_TYPE_ID && type_id != physics.BIG_COMPOUND_TYPE_ID
	{
		return .Ok;
	}
	children := (^physics.Compound)(value).children;
	if type_id == physics.BIG_COMPOUND_TYPE_ID
	{
		children = (^physics.Big_Compound)(value).children;
	}
	for child_index in 0 ..< int(children.length)
	{
		child := children.memory[child_index];
		child_status := shape_recursive_collect(registry, child.shape_index, handles, count);
		if child_status != .Ok
		{
			return child_status;
		}
	}
	return .Ok;
}

// shape_remove_recursive removes an unreferenced root and then removes every
// now-unreferenced reachable child shape. shared descendants remain registered.
// scratch is caller-owned and must hold the returned required upper bound. the
// upper bound counts duplicate references conservatively. no hidden allocation
// occurs. the root is never modified when capacity is insufficient
shape_remove_recursive :: proc (
	world: ^World, root: Shape_Handle, scratch: []Shape_Handle,
) -> (removed, required: int, status: Status)
{
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return 0, 0, registry_status;
	}
	_, root_info, root_status := shape_borrow_raw(world, root);
	if root_status != .Ok
	{
		return 0, 0, root_status;
	}
	if root_info.reference_count > 0
	{
		return 0, 0, .Shape_In_Use;
	}
	count_status: Status;
	required, count_status = shape_recursive_upper_bound(registry, root);
	if count_status != .Ok
	{
		return 0, required, count_status;
	}
	if len(scratch) < required
	{
		return 0, required, .Capacity_Missing;
	}
	count := 0;
	collect_status := shape_recursive_collect(registry, root, scratch, &count);
	if collect_status != .Ok
	{
		return 0, required, collect_status;
	}
	// parent references may keep a shared descendant alive until a later
	// sibling is removed. retry only the pending reachable set, not the world
	pending: int = count;
	for pending > 0
	{
		retained, progress: int;
		for index in 0 ..< pending
		{
			remove_status: Status = physics.shape_registry_remove(registry, scratch[index]);
			if remove_status == .Shape_In_Use
			{
				scratch[retained] = scratch[index];
				retained += 1;
				continue;
			}
			if remove_status != .Ok
			{
				return removed, required, remove_status;
			}
			removed += 1;
			progress += 1;
		}
		if progress == 0
		{
			break;
		}
		// other instances still own these descendants
		pending = retained;
	}
	return removed, required, .Ok;
}
