// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Body_Set_Storage :: enum u8
{
	Owned,
	Shared_Arena,
}

Body_Set :: struct
{
	dynamics_state:  util.Buffer(Body_Dynamics),
	index_to_handle: util.Buffer(Body_Handle),
	collidables:     util.Buffer(Collidable),
	activity:        util.Buffer(Body_Activity),
	constraints:     util.Buffer(util.Quick_List(Body_Constraint_Reference)),
	count:           int,
	arena_start:     i32,
	storage:         Body_Set_Storage,
	state:           Body_Set_State,
}

Body_Set_Arena :: struct
{
	dynamics_state:  util.Buffer(Body_Dynamics),
	index_to_handle: util.Buffer(Body_Handle),
	collidables:     util.Buffer(Collidable),
	activity:        util.Buffer(Body_Activity),
	constraints:     util.Buffer(util.Quick_List(Body_Constraint_Reference)),
	capacity:        int,
	state:           Body_Set_State,
}

#assert(size_of(Body_Set) == 96);
#assert(size_of(Body_Set_Arena) == 96);

body_set_capacity :: proc "contextless" (set: ^Body_Set) -> int
{
	if set == nil || set.state != .Allocated
	{
		return 0;
	}
	return min(
		int(set.dynamics_state.length),
		min(
		int(set.index_to_handle.length),
		min(
		int(set.collidables.length),
		min(int(set.activity.length), int(set.constraints.length)),
	),
	),
	);
}

physics_memory_status :: proc "contextless" (status: util.Memory_Status) -> Physics_Status
{
	if status == .Ok
	{
		return .Ok;
	}
	if status == .Pool_Disposed
	{
		return .Disposed;
	}
	if status == .Invalid_Count || status == .Invalid_Buffer
	{
		return .Invalid_Argument;
	}
	return .Capacity_Missing;
}

physics_collection_status :: proc "contextless" (status: util.Collection_Status) -> Physics_Status
{
	if status == .Ok
	{
		return .Ok;
	}
	if status == .Not_Found
	{
		return .Not_Found;
	}
	if status == .Disposed
	{
		return .Disposed;
	}
	if status == .Invalid_Argument
	{
		return .Invalid_Argument;
	}
	return .Capacity_Missing;
}

physics_return_buffer :: proc (pool: ^util.Buffer_Pool, buffer: ^util.Buffer($T))
{
	if buffer != nil && buffer.memory != nil
	{
		_ = util.buffer_pool_return(pool, buffer);
	}
}

physics_ensure_buffer_capacity :: proc (
	pool: ^util.Buffer_Pool, buffer: ^util.Buffer($T), capacity, copy_count: int,
) -> Physics_Status
{
	if pool == nil || buffer == nil || capacity <= 0 || copy_count < 0 ||
		copy_count > int(buffer.length)
	{
		return .Invalid_Argument;
	}
	if capacity <= int(buffer.length)
	{
		return .Ok;
	}
	status := util.buffer_pool_resize_to_at_least(
		pool, buffer, capacity, copy_count,
	);
	return physics_memory_status(status);
}

physics_ensure_zeroed_buffer_capacity :: proc (
	pool: ^util.Buffer_Pool, buffer: ^util.Buffer($T), capacity: int,
) -> Physics_Status
{
	if pool == nil || buffer == nil || capacity <= 0
	{
		return .Invalid_Argument;
	}
	old_memory := buffer.memory;
	status := physics_ensure_buffer_capacity(pool, buffer, capacity, 0);
	if status != .Ok
	{
		return status;
	}
	if buffer.memory != old_memory
	{
		clear_status := util.buffer_clear(buffer^, 0, int(buffer.length));
		if clear_status != .Ok
		{
			return physics_memory_status(clear_status);
		}
	}
	return .Ok;
}

physics_resize_buffer_capacity :: proc (
	pool: ^util.Buffer_Pool, buffer: ^util.Buffer($T), capacity, copy_count: int,
) -> Physics_Status
{
	if pool == nil || buffer == nil || capacity <= 0 || copy_count < 0 ||
		copy_count > capacity || copy_count > int(buffer.length)
	{
		return .Invalid_Argument;
	}
	target_capacity, target_status := util.buffer_pool_capacity_for_count(
		T, capacity,
	);
	if target_status != .Ok
	{
		return physics_memory_status(target_status);
	}
	current_capacity, current_status := util.buffer_capacity(buffer^);
	if current_status != .Ok
	{
		return physics_memory_status(current_status);
	}
	if target_capacity == current_capacity
	{
		return .Ok;
	}
	new_buffer, take_status := util.buffer_pool_take_at_least(
		pool, T, capacity,
	);
	if take_status != .Ok
	{
		return physics_memory_status(take_status);
	}
	copy_status := util.buffer_copy(
		util.buffer_view(buffer^), 0, util.buffer_view(new_buffer), 0,
		copy_count,
	);
	if copy_status != .Ok
	{
		physics_return_buffer(pool, &new_buffer);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(pool, buffer);
	buffer^ = new_buffer;
	return .Ok;
}

physics_resize_zeroed_buffer_capacity :: proc (
	pool: ^util.Buffer_Pool, buffer: ^util.Buffer($T), capacity: int,
) -> Physics_Status
{
	if pool == nil || buffer == nil || capacity <= 0
	{
		return .Invalid_Argument;
	}
	old_memory := buffer.memory;
	status := physics_resize_buffer_capacity(pool, buffer, capacity, 0);
	if status != .Ok
	{
		return status;
	}
	if buffer.memory != old_memory
	{
		clear_status := util.buffer_clear(buffer^, 0, int(buffer.length));
		if clear_status != .Ok
		{
			return physics_memory_status(clear_status);
		}
	}
	return .Ok;
}

body_set_arena_dispose :: proc (arena: ^Body_Set_Arena, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if arena == nil || pool == nil || arena.state != .Allocated
	{
		return .Disposed;
	}
	physics_return_buffer(pool, &arena.constraints);
	physics_return_buffer(pool, &arena.activity);
	physics_return_buffer(pool, &arena.collidables);
	physics_return_buffer(pool, &arena.index_to_handle);
	physics_return_buffer(pool, &arena.dynamics_state);
	arena^ = {};
	return .Ok;
}

body_set_arena_initialize :: proc (
	arena: ^Body_Set_Arena, capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if arena == nil || pool == nil || capacity <= 0 || arena.state == .Allocated
	{
		return .Invalid_Argument;
	}
	dynamics_state, dynamics_status := util.buffer_pool_take_at_least(pool, Body_Dynamics, capacity);
	if dynamics_status != .Ok
	{
		return physics_memory_status(dynamics_status);
	}
	storage_capacity := int(dynamics_state.length);
	index_to_handle, handles_status := util.buffer_pool_take_at_least(
		pool, Body_Handle, storage_capacity,
	);
	if handles_status != .Ok
	{
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(handles_status);
	}
	collidables, collidable_status := util.buffer_pool_take_at_least(
		pool, Collidable, storage_capacity,
	);
	if collidable_status != .Ok
	{
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(collidable_status);
	}
	activity, activity_status := util.buffer_pool_take_at_least(
		pool, Body_Activity, storage_capacity,
	);
	if activity_status != .Ok
	{
		physics_return_buffer(pool, &collidables);
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(activity_status);
	}
	constraints, constraints_status := util.buffer_pool_take_at_least(
		pool, util.Quick_List(Body_Constraint_Reference), storage_capacity,
	);
	if constraints_status != .Ok
	{
		physics_return_buffer(pool, &activity);
		physics_return_buffer(pool, &collidables);
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(constraints_status);
	}
	arena^ = {
		dynamics_state=dynamics_state,
		index_to_handle=index_to_handle,
		collidables=collidables,
		activity=activity,
		constraints=constraints,
		capacity=min(
			int(dynamics_state.length),
			min(
			int(index_to_handle.length),
			min(
			int(collidables.length),
			min(int(activity.length), int(constraints.length)),
		),
		),
		),
		state=.Allocated,
	};
	return .Ok;
}

body_set_bind_arena_range :: proc "contextless" (
	set: ^Body_Set, arena: ^Body_Set_Arena, start, capacity: int,
) -> Physics_Status
{
	if set == nil || arena == nil || arena.state != .Allocated ||
		start < 0 || capacity < 0 || start > arena.capacity - capacity || set.count > capacity
	{
		return .Invalid_Argument;
	}
	set.dynamics_state = {
		memory=&arena.dynamics_state.memory[start], length=i32(capacity), id=-1,
	};
	set.index_to_handle = {
		memory=&arena.index_to_handle.memory[start], length=i32(capacity), id=-1,
	};
	set.collidables = {
		memory=&arena.collidables.memory[start], length=i32(capacity), id=-1,
	};
	set.activity = {
		memory=&arena.activity.memory[start], length=i32(capacity), id=-1,
	};
	set.constraints = {
		memory=&arena.constraints.memory[start], length=i32(capacity), id=-1,
	};
	set.arena_start = i32(start);
	set.storage = .Shared_Arena;
	set.state = .Allocated;
	return .Ok;
}

body_set_arena_move_left :: proc "contextless" (
	set: ^Body_Set, arena: ^Body_Set_Arena, target_start: int,
) -> Physics_Status
{
	if set == nil || arena == nil || set.storage != .Shared_Arena ||
		target_start < 0 || target_start > int(set.arena_start) ||
		target_start > arena.capacity - set.count
	{
		return .Invalid_Argument;
	}
	source_start := int(set.arena_start);
	for index in 0 ..< set.count
	{
		target_index := target_start + index;
		source_index := source_start + index;
		arena.dynamics_state.memory[target_index] = arena.dynamics_state.memory[source_index];
		arena.index_to_handle.memory[target_index] = arena.index_to_handle.memory[source_index];
		arena.collidables.memory[target_index] = arena.collidables.memory[source_index];
		arena.activity.memory[target_index] = arena.activity.memory[source_index];
		arena.constraints.memory[target_index] = arena.constraints.memory[source_index];
	}
	return body_set_bind_arena_range(set, arena, target_start, set.count);
}

body_set_initialize :: proc (set: ^Body_Set, capacity: int, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if set == nil || pool == nil || capacity <= 0
	{
		return .Invalid_Argument;
	}
	if set.state == .Allocated
	{
		return .Invalid_Argument;
	}
	dynamics_state, dynamics_status := util.buffer_pool_take_at_least(pool, Body_Dynamics, capacity);
	if dynamics_status != .Ok
	{
		return physics_memory_status(dynamics_status);
	}
	storage_capacity := int(dynamics_state.length);
	index_to_handle, handles_status := util.buffer_pool_take_at_least(pool, Body_Handle, storage_capacity);
	if handles_status != .Ok
	{
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(handles_status);
	}
	collidables, collidable_status := util.buffer_pool_take_at_least(pool, Collidable, storage_capacity);
	if collidable_status != .Ok
	{
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(collidable_status);
	}
	activity, activity_status := util.buffer_pool_take_at_least(pool, Body_Activity, storage_capacity);
	if activity_status != .Ok
	{
		physics_return_buffer(pool, &collidables);
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(activity_status);
	}
	constraints, constraints_status := util.buffer_pool_take_at_least(
		pool, util.Quick_List(Body_Constraint_Reference), storage_capacity,
	);
	if constraints_status != .Ok
	{
		physics_return_buffer(pool, &activity);
		physics_return_buffer(pool, &collidables);
		physics_return_buffer(pool, &index_to_handle);
		physics_return_buffer(pool, &dynamics_state);
		return physics_memory_status(constraints_status);
	}
	_ = util.buffer_clear(dynamics_state, 0, int(dynamics_state.length));
	_ = util.buffer_clear(index_to_handle, 0, int(index_to_handle.length));
	_ = util.buffer_clear(collidables, 0, int(collidables.length));
	_ = util.buffer_clear(activity, 0, int(activity.length));
	_ = util.buffer_clear(constraints, 0, int(constraints.length));
	set^ = {
		dynamics_state=dynamics_state,
		index_to_handle=index_to_handle,
		collidables=collidables,
		activity=activity,
		constraints=constraints,
		arena_start=-1,
		storage=.Owned,
		state=.Allocated,
	};
	return .Ok;
}

body_set_ensure_capacity :: proc (set: ^Body_Set, capacity: int, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if set == nil || pool == nil || capacity <= 0
	{
		return .Invalid_Argument;
	}
	if set.storage == .Shared_Arena
	{
		if capacity <= body_set_capacity(set)
		{
			return .Ok;
		}
		return .Capacity_Missing;
	}
	if set.state != .Allocated
	{
		return body_set_initialize(set, capacity, pool);
	}
	if capacity <= body_set_capacity(set)
	{
		return .Ok;
	}
	copy_count := set.count;
	new_dynamics, dynamics_status := util.buffer_pool_take_at_least(pool, Body_Dynamics, capacity);
	if dynamics_status != .Ok
	{
		return physics_memory_status(dynamics_status);
	}
	storage_capacity := int(new_dynamics.length);
	new_handles, handles_status := util.buffer_pool_take_at_least(
		pool, Body_Handle, storage_capacity,
	);
	if handles_status != .Ok
	{
		physics_return_buffer(pool, &new_dynamics);
		return physics_memory_status(handles_status);
	}
	new_collidables, collidables_status := util.buffer_pool_take_at_least(
		pool, Collidable, storage_capacity,
	);
	if collidables_status != .Ok
	{
		physics_return_buffer(pool, &new_handles);
		physics_return_buffer(pool, &new_dynamics);
		return physics_memory_status(collidables_status);
	}
	new_activity, activity_status := util.buffer_pool_take_at_least(
		pool, Body_Activity, storage_capacity,
	);
	if activity_status != .Ok
	{
		physics_return_buffer(pool, &new_collidables);
		physics_return_buffer(pool, &new_handles);
		physics_return_buffer(pool, &new_dynamics);
		return physics_memory_status(activity_status);
	}
	new_constraints, constraints_status := util.buffer_pool_take_at_least(
		pool, util.Quick_List(Body_Constraint_Reference), storage_capacity,
	);
	if constraints_status != .Ok
	{
		physics_return_buffer(pool, &new_activity);
		physics_return_buffer(pool, &new_collidables);
		physics_return_buffer(pool, &new_handles);
		physics_return_buffer(pool, &new_dynamics);
		return physics_memory_status(constraints_status);
	}
	_ = util.buffer_clear(new_dynamics, 0, int(new_dynamics.length));
	_ = util.buffer_clear(new_handles, 0, int(new_handles.length));
	_ = util.buffer_clear(new_collidables, 0, int(new_collidables.length));
	_ = util.buffer_clear(new_activity, 0, int(new_activity.length));
	_ = util.buffer_clear(new_constraints, 0, int(new_constraints.length));
	copy_status := util.buffer_copy(
		util.buffer_view(set.dynamics_state),
		0,
		util.buffer_view(new_dynamics),
		0,
		copy_count
	);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(set.index_to_handle),
			0,
			util.buffer_view(new_handles),
			0,
			copy_count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(set.collidables),
			0,
			util.buffer_view(new_collidables),
			0,
			copy_count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(set.activity),
			0,
			util.buffer_view(new_activity),
			0,
			copy_count
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(set.constraints),
			0,
			util.buffer_view(new_constraints),
			0,
			copy_count
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(pool, &new_constraints);
		physics_return_buffer(pool, &new_activity);
		physics_return_buffer(pool, &new_collidables);
		physics_return_buffer(pool, &new_handles);
		physics_return_buffer(pool, &new_dynamics);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(pool, &set.constraints);
	physics_return_buffer(pool, &set.activity);
	physics_return_buffer(pool, &set.collidables);
	physics_return_buffer(pool, &set.index_to_handle);
	physics_return_buffer(pool, &set.dynamics_state);
	set.dynamics_state = new_dynamics;
	set.index_to_handle = new_handles;
	set.collidables = new_collidables;
	set.activity = new_activity;
	set.constraints = new_constraints;
	return .Ok;
}

body_set_ensure_constraint_capacities :: proc (
	set: ^Body_Set, minimum_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if set == nil || set.state != .Allocated || minimum_capacity <= 0 ||
		pool == nil
	{
		return .Invalid_Argument;
	}
	for body_index in 0 ..< set.count
	{
		status := util.quick_list_ensure_capacity(
			&set.constraints.memory[body_index], minimum_capacity, pool,
		);
		if status != .Ok
		{
			return physics_collection_status(status);
		}
	}
	return .Ok;
}

body_set_resize_capacity :: proc (
	set: ^Body_Set, capacity, minimum_constraint_capacity: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if set == nil || set.state != .Allocated || set.storage != .Owned ||
		capacity <= 0 || minimum_constraint_capacity <= 0 || pool == nil
	{
		return .Invalid_Argument;
	}
	target := max(capacity, set.count);
	status := physics_resize_buffer_capacity(
		pool, &set.dynamics_state, target, set.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &set.collidables, target, set.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &set.activity, target, set.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &set.constraints, target, set.count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		pool, &set.index_to_handle, target, set.count,
	);
	if status != .Ok
	{
		return status;
	}
	for body_index in 0 ..< set.count
	{
		list := &set.constraints.memory[body_index];
		list_target := max(minimum_constraint_capacity, list.count);
		status = physics_resize_buffer_capacity(
			pool, &list.span, list_target, list.count,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

body_set_apply_description :: proc "contextless" (
	set: ^Body_Set,
	index: int,
	description: ^Body_Description
) -> Physics_Status
{
	if set == nil || description == nil || index < 0 || index >= set.count
	{
		return .Invalid_Argument;
	}
	validation := body_description_validate(description);
	if validation != .Ok
	{
		return validation;
	}
	state := &set.dynamics_state.memory[index];
	state.motion.pose = description.pose;
	state.motion.velocity = description.velocity;
	state.inertia.local = description.local_inertia;
	state.inertia.world = {};
	collidable := &set.collidables.memory[index];
	collidable.shape = description.collidable.shape;
	collidable.continuity = description.collidable.continuity;
	collidable.minimum_speculative_margin = description.collidable.minimum_speculative_margin;
	collidable.maximum_speculative_margin = description.collidable.maximum_speculative_margin;
	collidable.speculative_margin = 0;
	collidable.broad_phase_index = -1;
	activity := &set.activity.memory[index];
	activity.sleep_threshold = description.activity.sleep_threshold;
	activity.minimum_timesteps_under_threshold = description.activity.minimum_timestep_count_under_threshold;
	activity.timesteps_under_threshold_count = 0;
	activity.sleep_candidate = .Not_Candidate;
	return .Ok;
}

body_set_get_description :: proc "contextless" (set: ^Body_Set, index: int) -> (Body_Description, Physics_Status)
{
	if set == nil || set.state != .Allocated || index < 0 || index >= set.count
	{
		return {}, .Not_Found;
	}
	state := &set.dynamics_state.memory[index];
	collidable := &set.collidables.memory[index];
	activity := &set.activity.memory[index];
	return {
		pose=state.motion.pose,
		velocity=state.motion.velocity,
		local_inertia=state.inertia.local,
		collidable={
			shape=collidable.shape,
			continuity=collidable.continuity,
			minimum_speculative_margin=collidable.minimum_speculative_margin,
			maximum_speculative_margin=collidable.maximum_speculative_margin,
		},
		activity={
			sleep_threshold=activity.sleep_threshold,
			minimum_timestep_count_under_threshold=activity.minimum_timesteps_under_threshold,
		},
	}, .Ok;
}

body_set_add :: proc (
	set: ^Body_Set, description: ^Body_Description, handle: Body_Handle,
	minimum_constraint_capacity: int, pool: ^util.Buffer_Pool,
) -> (int, Physics_Status)
{
	if set == nil || description == nil || pool == nil || set.state != .Allocated
	{
		return -1, .Invalid_Argument;
	}
	if set.count >= body_set_capacity(set)
	{
		if set.storage == .Shared_Arena
		{
			return -1, .Capacity_Missing;
		}
		status := body_set_ensure_capacity(
			set, max(body_set_capacity(set) * 2, 1), pool,
		);
		if status != .Ok
		{
			return -1, status;
		}
	}
	index := set.count;
	set.count += 1;
	set.index_to_handle.memory[index] = handle;
	constraint_status := util.quick_list_initialize(
		&set.constraints.memory[index], max(minimum_constraint_capacity, 1), pool,
	);
	if constraint_status != .Ok
	{
		set.count -= 1;
		set.index_to_handle.memory[index] = {};
		return -1, physics_collection_status(constraint_status);
	}
	status := body_set_apply_description(set, index, description);
	if status != .Ok
	{
		_ = util.quick_list_dispose(&set.constraints.memory[index], pool);
		set.count -= 1;
		set.index_to_handle.memory[index] = {};
		return -1, status;
	}
	return index, .Ok;
}

body_set_remove_at :: proc "contextless" (
	set: ^Body_Set, index: int,
) -> (removed_handle, moved_handle: Body_Handle, removed_constraints: util.Quick_List(Body_Constraint_Reference), moved: Reference_State, status: Physics_Status)
{
	if set == nil || set.state != .Allocated || index < 0 || index >= set.count
	{
		return {}, {}, {}, .Missing, .Not_Found;
	}
	removed_handle = set.index_to_handle.memory[index];
	removed_constraints = set.constraints.memory[index];
	set.count -= 1;
	if index < set.count
	{
		last := set.count;
		set.dynamics_state.memory[index] = set.dynamics_state.memory[last];
		set.index_to_handle.memory[index] = set.index_to_handle.memory[last];
		set.collidables.memory[index] = set.collidables.memory[last];
		set.activity.memory[index] = set.activity.memory[last];
		set.constraints.memory[index] = set.constraints.memory[last];
		moved_handle = set.index_to_handle.memory[index];
		moved = .Present;
	}
	last := set.count;
	set.dynamics_state.memory[last] = {};
	set.index_to_handle.memory[last] = {};
	set.collidables.memory[last] = {};
	set.activity.memory[last] = {};
	set.constraints.memory[last] = {};
	return removed_handle, moved_handle, removed_constraints, moved, .Ok;
}

body_set_dispose :: proc (set: ^Body_Set, pool: ^util.Buffer_Pool) -> Physics_Status
{
	if set == nil || pool == nil || set.state != .Allocated
	{
		return .Disposed;
	}
	for index in 0 ..< set.count
	{
		if set.constraints.memory[index].span.memory != nil
		{
			_ = util.quick_list_dispose(&set.constraints.memory[index], pool);
		}
	}
	if set.storage == .Owned
	{
		physics_return_buffer(pool, &set.constraints);
		physics_return_buffer(pool, &set.activity);
		physics_return_buffer(pool, &set.collidables);
		physics_return_buffer(pool, &set.index_to_handle);
		physics_return_buffer(pool, &set.dynamics_state);
	}
	set^ = {};
	return .Ok;
}
