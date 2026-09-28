package entasis

import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Query_Context owns an independent collision batcher and traversal pool. one
// caller may execute it at a time. keep its attached world alive until destroy
Query_Context :: distinct rawptr;
@(private)
Query_Context_Ownership :: enum u8
{
	Borrowed, Owned
}

@(private)
Query_Context_Execution :: enum u8
{
	Idle, Executing
}

@(private)
Query_Early_Out :: enum u8
{
	Disabled, Enabled
}

Query_Hit_State :: enum u8
{
	Miss, Hit
}

// Query_Context_Description reserves ordinary collision and traversal storage.
// reservation is a capacity hint, not a hard allocation budget. queries may
// grow only their private pool. they never borrow mutable world/worker scratch
Query_Context_Description :: struct
{
	pair_capacity: i32,
	child_capacity: i32,
	traversal_maximum_power: i32,
	traversal_slots_per_power: i32,
}

@(private)
query_context_data :: struct
{
	world: ^world_data,
	next: ^query_context_data,
	owned_pool: Buffer_Pool,
	pool: ^Buffer_Pool,
	allocator: mem.Allocator,
	pairs: util.Buffer(physics.Collision_Batcher_Pair),
	scratch: physics.Collision_Batcher_Scratch,
	collision: physics.Collision_Batcher,
	overlap: physics.Query_Overlap_Batcher,
	description: Query_Context_Description,
	ownership: Query_Context_Ownership,
	execution: Query_Context_Execution,
	distance_storage: ^distance_query_storage,
}

query_context_description_default :: proc "contextless" () -> Query_Context_Description
{
	return {pair_capacity=16, child_capacity=1024, traversal_maximum_power=12, traversal_slots_per_power=4};
}

@(private)
query_context_get :: #force_inline proc "contextless" (query_context: ^Query_Context) -> ^query_context_data
{
	if query_context == nil || query_context^ == nil
	{
		return nil;
	}
	return cast(^query_context_data)query_context^;
}

@(private)
query_context_description_validate :: proc "contextless" (description: Query_Context_Description) -> Status
{
	return .Ok if description.pair_capacity > 0 && description.child_capacity > 0 &&
	description.traversal_maximum_power >= 8 && description.traversal_maximum_power <= util.MAXIMUM_SPAN_SIZE_POWER &&
	description.traversal_slots_per_power > 0 else .Invalid_Description;
}

@(private)
query_context_reserve_returns :: proc(pool: ^Buffer_Pool, additional_slots: int) -> Status
{
	if pool.allocation_scope == .All_Owned
	{
		return .Ok;
	}
	// cold, context-local metadata reservation. this does not change Legacy world
	// pool growth. reserve enough IDs before taking replacement scratch, so every
	// valid return during partial-initialization cleanup or commit cannot allocate
	for &power in pool.pools
	{
		if power.next_slot > util.BUFFER_POOL_ID_SLOT_COUNT - additional_slots
		{
			return .Capacity_Missing;
		}
		status: util.Memory_Status = util.power_pool_resize_free_slots(pool, &power, power.next_slot + additional_slots);
		if status != .Ok
		{
			return world_memory_status(status);
		}
	}
	return .Ok;
}

@(private)
query_context_reserve_buffer :: proc (
	pool: ^Buffer_Pool, buffer: ^util.Buffer($T), count: int, returns_reserved: ^physics.Reference_State,
) -> Status
{
	if buffer.memory != nil
	{
		capacity: int;
		status: util.Memory_Status;
		capacity, status = util.buffer_capacity(buffer^);
		if status != .Ok
		{
			return world_memory_status(status);
		}
		if capacity >= count
		{
			buffer.length = i32(capacity);
			return .Ok;
		}
	}
	if returns_reserved^ == .Missing
	{
		// at most ten scratch buffers can be acquired. reserve cleanup IDs once,
		// only when real growth occurs, before taking any replacement storage
		status: Status = query_context_reserve_returns(pool, 32);
		if status != .Ok
		{
			return status;
		}
		returns_reserved^ = .Present;
	}
	replacement: util.Buffer(T);
	status: util.Memory_Status;
	replacement, status = util.buffer_pool_take_at_least(pool, T, count);
	if status != .Ok
	{
		return world_memory_status(status);
	}
	buffer^ = replacement;
	return .Ok;
}

@(private)
query_context_return_replaced_buffer :: proc (
	pool: ^Buffer_Pool, buffer: ^util.Buffer($T), retained: util.Buffer(T),
)
{
	if buffer.memory != nil && buffer.memory != retained.memory
	{
		_ = util.buffer_pool_return(pool, buffer);
	}
}

@(private)
query_context_return_replaced_scratch :: proc (
	pool: ^Buffer_Pool, scratch: ^physics.Collision_Batcher_Scratch,
	retained: ^physics.Collision_Batcher_Scratch,
)
{
	query_context_return_replaced_buffer(pool, &scratch.children.combined, retained.children.combined);
	query_context_return_replaced_buffer(pool, &scratch.children.headers, retained.children.headers);
	query_context_return_replaced_buffer(pool, &scratch.children.offsets_a, retained.children.offsets_a);
	query_context_return_replaced_buffer(pool, &scratch.children.contact_data, retained.children.contact_data);
	query_context_return_replaced_buffer(pool, &scratch.children.depths, retained.children.depths);
	query_context_return_replaced_buffer(pool, &scratch.children.subpairs, retained.children.subpairs);
	query_context_return_replaced_buffer(pool, &scratch.children.next, retained.children.next);
	query_context_return_replaced_buffer(pool, &scratch.continuations, retained.continuations);
	if scratch.candidate_refs.memory != nil && scratch.candidate_refs.memory != retained.candidate_refs.memory
	{
		physics.collision_candidate_storage_return(pool, &scratch.candidate_refs);
	}
}

@(private)
query_context_reserve_data :: proc (data: ^query_context_data, description: Query_Context_Description) -> Status
{
	if query_context_description_validate(description) != .Ok
	{
		return .Invalid_Description;
	}
	if data.pairs.memory != nil && int(data.pairs.length) >= int(description.pair_capacity) &&
	data.collision.children.length >= description.child_capacity
	{
		// reservation is a lower bound, not a request to shrink existing scratch.
		// check the live pool rather than caching a traversal reservation promise
		for power in 8 ..= int(description.traversal_maximum_power)
		{
			status: util.Memory_Status = util.buffer_pool_ensure_available_slot_count(data.pool, power, int(description.traversal_slots_per_power));
			if status != .Ok
			{
				return world_memory_status(status);
			}
		}
		data.description = description;
		return .Ok;
	}
	return query_context_grow_data(data, description);
}

@(private)
query_context_grow_data :: #force_no_inline proc (data: ^query_context_data, description: Query_Context_Description) -> Status
{
	// keep replacement storage and its larger stack frame off satisfied reserves
	pairs: util.Buffer(physics.Collision_Batcher_Pair) = data.pairs;
	scratch: physics.Collision_Batcher_Scratch = data.scratch;
	// until commit, original buffers remain bound. on failure return only newly
	// acquired buffers. already sufficient arrays are borrowed, not copied
	defer
	{
		query_context_return_replaced_buffer(data.pool, &pairs, data.pairs);
		query_context_return_replaced_scratch(data.pool, &scratch, &data.scratch);
	}
	returns_reserved: physics.Reference_State = .Missing;
	status: Status = query_context_reserve_buffer(data.pool, &pairs, int(description.pair_capacity), &returns_reserved);
	if status != .Ok
	{
		return status;
	}
	status = query_context_reserve_buffer(data.pool, &scratch.continuations, int(pairs.length), &returns_reserved);
	if status != .Ok
	{
		return status;
	}
	child_capacity: int = int(description.child_capacity);
	candidate_capacity: int = child_capacity * physics.MAXIMUM_MANIFOLD_CONTACT_COUNT;
	if scratch.children.format == .Combined || child_capacity > physics.COLLISION_BATCHER_SPLIT_CHILD_CAPACITY_MAX
	{
		combined: util.Buffer(physics.Collision_Batcher_Child_Scratch) = scratch.children.combined;
		status = query_context_reserve_buffer(data.pool, &combined, child_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		scratch.children = {format=.Combined, length=i32(child_capacity), combined=combined};
	}
	else
	{
		scratch.children.length = i32(child_capacity);
		status = query_context_reserve_buffer(data.pool, &scratch.children.headers, child_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		status = query_context_reserve_buffer(data.pool, &scratch.children.offsets_a, child_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		status = query_context_reserve_buffer(data.pool, &scratch.children.contact_data, candidate_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		status = query_context_reserve_buffer(data.pool, &scratch.children.depths, candidate_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		status = query_context_reserve_buffer(data.pool, &scratch.children.subpairs, child_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		status = query_context_reserve_buffer(data.pool, &scratch.children.next, child_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
	}
	if scratch.candidate_refs.format == .Packed_U32 || child_capacity > 1 << 14
	{
		candidates: util.Buffer(u32);
		if scratch.candidate_refs.format == .Packed_U32
		{
			candidates = {memory=cast([^]u32)scratch.candidate_refs.memory, length=scratch.candidate_refs.length, id=scratch.candidate_refs.id};
		}
		status = query_context_reserve_buffer(data.pool, &candidates, candidate_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		scratch.candidate_refs = {memory=rawptr(candidates.memory), length=candidates.length, id=candidates.id, format=.Packed_U32};
	}
	else
	{
		candidates: util.Buffer(u16) = util.Buffer(u16){memory=cast([^]u16)scratch.candidate_refs.memory, length=scratch.candidate_refs.length, id=scratch.candidate_refs.id};
		status = query_context_reserve_buffer(data.pool, &candidates, candidate_capacity, &returns_reserved);
		if status != .Ok
		{
			return status;
		}
		scratch.candidate_refs = {memory=rawptr(candidates.memory), length=candidates.length, id=candidates.id, format=.Packed_U16};
	}
	for power in 8 ..= int(description.traversal_maximum_power)
	{
		memory_status: util.Memory_Status = util.buffer_pool_ensure_available_slot_count(data.pool, power, int(description.traversal_slots_per_power));
		if memory_status != .Ok
		{
			return world_memory_status(memory_status);
		}
	}
	simulation: ^physics.Simulation = &data.world.simulation;
	collision: physics.Collision_Batcher;
	status = physics.collision_batcher_initialize_bound(&collision, pairs,
		physics.simulation_shape_registry(simulation), &simulation.collision_tasks,
		{pair_completed=physics.query_overlap_pair_completed}, &data.overlap, data.pool, scratch, child_capacity);
	if status != .Ok
	{
		return status;
	}
	query_context_return_replaced_buffer(data.pool, &data.pairs, pairs);
	query_context_return_replaced_scratch(data.pool, &data.scratch, &scratch);
	data.pairs = pairs;
	data.scratch = scratch;
	data.collision = collision;
	data.overlap = {collision=&data.collision, narrow=&simulation.narrow_phase, state=.Ready};
	data.description = description;
	return .Ok;
}

query_context_init :: proc (
	query_context: ^Query_Context, world: ^World,
	description: Query_Context_Description = {16, 1024, 12, 4},
	borrowed_pool: ^Buffer_Pool = nil,
) -> Status
{
	if query_context == nil || query_context^ != nil
	{
		return .Invalid_Argument;
	}
	owner: ^world_data;
	ready: Status;
	owner, ready = query_world_data(world);
	if ready != .Ok
	{
		return ready;
	}
	if query_context_description_validate(description) != .Ok
	{
		return .Invalid_Description;
	}
	if borrowed_pool != nil
	{
		if borrowed_pool.state != .Ready || borrowed_pool == owner.pool
		{
			return .Invalid_Argument;
		}
		if owner.dispatcher != nil && owner.dispatcher.worker_pool != nil
		{
			for worker in 0 ..< owner.dispatcher.worker_count
			{
				worker_pool: ^Buffer_Pool;
				worker_status: util.Threading_Status;
				worker_pool, worker_status = owner.dispatcher.worker_pool(owner.dispatcher, worker);
				if worker_status != .Ok || borrowed_pool == worker_pool
				{
					return .Invalid_Argument;
				}
			}
		}
		for attached: ^query_context_data = owner.query_contexts; attached != nil; attached = attached.next
		{
			if attached.pool == borrowed_pool
			{
				return .Invalid_Argument;
			}
		}
	}
	data: ^query_context_data;
	error: mem.Allocator_Error;
	data, error = new(query_context_data, allocator=owner.allocator);
	if error != nil || data == nil
	{
		return .Capacity_Missing;
	}
	data.world = owner;
	data.allocator = owner.allocator;
	data.pool = borrowed_pool;
	data.ownership = .Owned if borrowed_pool == nil else .Borrowed;
	if data.ownership == .Owned
	{
		status: Status = world_memory_status(util.buffer_pool_initialize_internal(
				&data.owned_pool, 16384, 16,
				world_allocator({}) if owner.allocation_scope == .Legacy else owner.allocator, owner.allocation_scope));
		if status != .Ok
		{
			_ = util.allocation_free(data, size_of(query_context_data), align_of(query_context_data), owner.allocator, owner.allocation_scope);
			return status;
		}
		data.pool = &data.owned_pool;
	}
	status: Status = query_context_reserve_data(data, description);
	if status != .Ok
	{
		if data.ownership == .Owned
		{
			_ = buffer_pool_destroy(data.pool);
		}
		_ = util.allocation_free(data, size_of(query_context_data), align_of(query_context_data), owner.allocator, owner.allocation_scope);
		return status;
	}
	data.next = owner.query_contexts;
	owner.query_contexts = data;
	query_context^ = Query_Context(data);
	return .Ok;
}

query_context_reserve :: proc (
	query_context: ^Query_Context, description: Query_Context_Description,
) -> Status
{
	data: ^query_context_data = query_context_get(query_context);
	if data == nil
	{
		return .Disposed;
	}
	if data.execution == .Executing || data.world.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	return query_context_reserve_data(data, description);
}

query_context_destroy :: proc (query_context: ^Query_Context) -> Status
{
	data: ^query_context_data = query_context_get(query_context);
	if data == nil
	{
		return .Disposed;
	}
	if data.execution == .Executing || data.world.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	distance_query_storage_destroy(&data.distance_storage, data.pool, data.allocator, data.world.allocation_scope);
	// an owned pool has no external live buffers and can be released directly.
	// a borrowed pool keeps its capacity and must receive each bound buffer back
	if data.ownership == .Owned
	{
		status: Status = buffer_pool_destroy(data.pool);
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		status: Status = query_context_reserve_returns(data.pool, 0);
		if status != .Ok
		{
			return status;
		}
		physics.collision_batcher_return_scratch(data.pool, &data.scratch);
		status = world_memory_status(util.buffer_pool_return(data.pool, &data.pairs));
		if status != .Ok
		{
			return status;
		}
	}
	link: ^^query_context_data = &data.world.query_contexts;
	for link^ != nil && link^ != data
	{
		link = &link^.next;
	}
	if link^ == data
	{
		link^ = data.next;
	}
	allocator: mem.Allocator = data.allocator;
	query_context^ = nil;
	_ = util.allocation_free(data, size_of(query_context_data), align_of(query_context_data), allocator, data.world.allocation_scope);
	return .Ok;
}

@(private)
query_context_begin :: proc "contextless" (query_context: ^Query_Context) -> (^query_context_data, Status)
{
	data: ^query_context_data = query_context_get(query_context);
	if data == nil
	{
		return nil, .Disposed;
	}
	if data.execution == .Executing || data.world.simulation.state != .Ready || data.pool.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	// registry roots stay at stable addresses. no epoch-local shape/broadphase
	// pointers are retained between calls. the resolver is constructed per query
	data.execution = .Executing;
	return data, .Ok;
}

@(private)
query_context_ray_execute :: proc "contextless" (
	data: ^query_context_data, ray_value: Ray, hits: []Ray_Hit,
	filter: Query_Filter, mode: physics.Query_Collection_Mode, early_out: Query_Early_Out,
) -> (int, Status)
{
	ctx: query_filter_context = query_filter_context{filter=filter, stop_on_first=early_out == .Enabled};
	collector: physics.Ray_Query_Collector;
	callbacks: physics.Ray_Query_Callbacks = physics.Ray_Query_Callbacks{filter=query_filter_low_level(filter), user_context=&ctx};
	if early_out == .Enabled
	{
		callbacks.hit = query_ray_stop_on_first;
	}
	status: Status = physics.ray_query_collector_initialize(&collector, query_ray_buffer(hits), mode, callbacks);
	if status != .Ok
	{
		return 0, status;
	}
	status = physics.simulation_ray_query(&data.world.simulation, ray_value, &collector, data.pool);
	return collector.count, status;
}

@(private)
query_context_sweep_execute :: proc "contextless" (
	data: ^query_context_data, shape: Shape_Handle, pose_value: Rigid_Pose,
	velocity_value: Body_Velocity, maximum_t: f32, hits: []Sweep_Hit,
	filter: Query_Filter, settings: Sweep_Settings, mode: physics.Query_Collection_Mode, early_out: Query_Early_Out,
) -> (int, Status)
{
	resolved: Sweep_Settings;
	settings_status: Status;
	resolved, settings_status = sweep_settings_resolve(data.world, shape, velocity_value, maximum_t, settings);
	if settings_status != .Ok
	{
		return 0, settings_status;
	}
	ctx: query_filter_context = query_filter_context{filter=filter};
	collector: physics.Sweep_Query_Collector = physics.Sweep_Query_Collector{hits=query_sweep_buffer(hits), mode=mode,
		callbacks={filter=query_filter_low_level(filter), user_context=&ctx}};
	if early_out == .Enabled
	{
		collector.callbacks.hit = query_sweep_stop_hit;
		collector.callbacks.hit_at_zero = query_sweep_stop_zero;
	}
	status: Status = physics.simulation_sweep_query(&data.world.simulation, shape, pose_value, velocity_value, maximum_t,
		resolved.minimum_progression, resolved.convergence_threshold, int(resolved.maximum_iteration_count), &collector, data.pool);
	return collector.count, status;
}

@(private)
query_context_overlap_execute :: proc "contextless" (
	data: ^query_context_data, shape: Shape_Handle, pose_value: Rigid_Pose, hits: []Overlap_Hit, filter: Query_Filter,
) -> (int, Status)
{
	temporary: [1]Overlap_Hit;
	output: []Overlap_Hit = hits;
	if len(output) == 0
	{
		output = temporary[:];
	}
	ctx: query_filter_context = query_filter_context{filter=filter};
	collector: physics.Overlap_Query_Collector = physics.Overlap_Query_Collector{hits=query_overlap_buffer(output), filter=query_filter_low_level(filter), user_context=&ctx};
	simulation: ^physics.Simulation = &data.world.simulation;
	shapes: ^physics.Shape_Registry = physics.simulation_shape_registry(simulation);
	active_context, static_context: physics.Simulation_Query_Target_Context;
	status: Status = physics.query_overlap_resolved(shape, pose_value, &simulation.broad_phase.active_tree,
		physics.simulation_query_resolver(&active_context, simulation, simulation.broad_phase.active_leaves),
		shapes, &simulation.collision_tasks, &collector, data.pool, &data.overlap);
	if status == .Ok
	{
		status = physics.query_overlap_resolved(shape, pose_value, &simulation.broad_phase.static_tree,
			physics.simulation_query_resolver(&static_context, simulation, simulation.broad_phase.static_leaves),
			shapes, &simulation.collision_tasks, &collector, data.pool, &data.overlap);
	}
	if len(hits) == 0 && collector.count > 0
	{
		return 0, .Capacity_Missing;
	}
	return collector.count, status;
}

@(private)
query_context_volume_execute :: proc "contextless" (
	data: ^query_context_data, bounds: Bounding_Box, hits: []Volume_Hit, filter: Query_Filter,
) -> (int, Status)
{
	temporary: [1]Volume_Hit;
	output: []Volume_Hit = hits;
	if len(output) == 0
	{
		output = temporary[:];
	}
	ctx: query_filter_context = query_filter_context{filter=filter};
	collector: physics.Volume_Query_Collector = physics.Volume_Query_Collector{hits=query_volume_buffer(output), filter=query_filter_low_level(filter), user_context=&ctx};
	status: Status = physics.simulation_volume_query(&data.world.simulation, bounds, &collector, data.pool);
	if len(hits) == 0 && collector.count > 0
	{
		return 0, .Capacity_Missing;
	}
	return collector.count, status;
}

ray_cast_any_with_context :: proc "contextless" (query_context: ^Query_Context, ray_value: Ray, filter: Query_Filter = {}) -> (Query_Hit_State, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return .Miss, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Ray_Hit;
	output: []Ray_Hit = temporary[:];
	count: int;
	status: Status;
	count, status = query_context_ray_execute(data, ray_value, output, filter, .Earliest, .Enabled);
	if status != .Ok
	{
		return .Miss, status;
	}
	return .Hit if count > 0 else .Miss, .Ok;
}

ray_cast_closest_with_context :: proc "contextless" (query_context: ^Query_Context, ray_value: Ray, filter: Query_Filter = {}) -> (Ray_Hit, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return {child_index=-1}, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Ray_Hit;
	output: []Ray_Hit = temporary[:];
	count: int;
	status: Status;
	count, status = query_context_ray_execute(data, ray_value, output, filter, .Earliest, .Disabled);
	if status != .Ok
	{
		return {child_index=-1}, status;
	}
	if count == 0
	{
		return {child_index=-1}, .Not_Found;
	}
	return temporary[0], .Ok;
}

ray_cast_all_with_context :: proc "contextless" (query_context: ^Query_Context, ray_value: Ray, hits: []Ray_Hit, filter: Query_Filter = {}) -> (int, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return 0, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Ray_Hit;
	output: []Ray_Hit = hits;
	if len(output) == 0
	{
		output = temporary[:];
	}
	if len(hits) == 0
	{
		count: int;
		status: Status;
		count, status = query_context_ray_execute(data, ray_value, output, filter, .Earliest, .Enabled);
		if status != .Ok
		{
			return 0, status;
		}
		if count > 0
		{
			return 0, .Capacity_Missing;
		}
		return 0, .Ok;
	}
	return query_context_ray_execute(data, ray_value, output, filter, .All, .Disabled);
}

sweep_any_with_context :: proc "contextless" (query_context: ^Query_Context, shape: Shape_Handle, pose_value: Rigid_Pose, velocity_value: Body_Velocity, maximum_t: f32, filter: Query_Filter = {}, settings: Sweep_Settings = {}) -> (Query_Hit_State, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return .Miss, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Sweep_Hit;
	output: []Sweep_Hit = temporary[:];
	count: int;
	status: Status;
	count, status = query_context_sweep_execute(data, shape, pose_value, velocity_value, maximum_t, output, filter, settings, .All, .Enabled);
	if status != .Ok && (status != .Capacity_Missing || count == 0)
	{
		return .Miss, status;
	}
	return .Hit if count > 0 else .Miss, .Ok;
}

sweep_closest_with_context :: proc "contextless" (query_context: ^Query_Context, shape: Shape_Handle, pose_value: Rigid_Pose, velocity_value: Body_Velocity, maximum_t: f32, filter: Query_Filter = {}, settings: Sweep_Settings = {}) -> (Sweep_Hit, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return {}, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Sweep_Hit;
	output: []Sweep_Hit = temporary[:];
	count: int;
	status: Status;
	count, status = query_context_sweep_execute(data, shape, pose_value, velocity_value, maximum_t, output, filter, settings, .Earliest, .Disabled);
	if status != .Ok
	{
		return {}, status;
	}
	if count == 0
	{
		return {}, .Not_Found;
	}
	return temporary[0], .Ok;
}

sweep_all_with_context :: proc "contextless" (query_context: ^Query_Context, shape: Shape_Handle, pose_value: Rigid_Pose, velocity_value: Body_Velocity, maximum_t: f32, hits: []Sweep_Hit, filter: Query_Filter = {}, settings: Sweep_Settings = {}) -> (int, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return 0, ready;
	}
	defer data.execution = .Idle;
	temporary: [1]Sweep_Hit;
	output: []Sweep_Hit = hits;
	if len(output) == 0
	{
		output = temporary[:];
	}
	if len(hits) == 0
	{
		count: int;
		status: Status;
		count, status = query_context_sweep_execute(data, shape, pose_value, velocity_value, maximum_t, output, filter, settings, .All, .Enabled);
		if status != .Ok && status != .Capacity_Missing
		{
			return 0, status;
		}
		if count > 0
		{
			return 0, .Capacity_Missing;
		}
		return 0, .Ok;
	}
	return query_context_sweep_execute(data, shape, pose_value, velocity_value, maximum_t, output, filter, settings, .All, .Disabled);
}

overlap_all_with_context :: proc "contextless" (query_context: ^Query_Context, shape: Shape_Handle, pose_value: Rigid_Pose, hits: []Overlap_Hit, filter: Query_Filter = {}) -> (int, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return 0, ready;
	}
	defer data.execution = .Idle;
	return query_context_overlap_execute(data, shape, pose_value, hits, filter);
}

volume_all_with_context :: proc "contextless" (query_context: ^Query_Context, bounds: Bounding_Box, hits: []Volume_Hit, filter: Query_Filter = {}) -> (int, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return 0, ready;
	}
	defer data.execution = .Idle;
	return query_context_volume_execute(data, bounds, hits, filter);
}

@(private)
query_context_collision_execute :: proc "contextless" (
	query_data: ^query_context_data,
	shape_a: Shape_Handle,
	pose_a: Rigid_Pose,
	shape_b: Shape_Handle,
	pose_b: Rigid_Pose,
	speculative_margin: f32 = 0,
) -> (Manifold_Result, Status)
{
	data: ^world_data = query_data.world;
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	if speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	simulation: ^physics.Simulation = &data.simulation;
	registry: ^physics.Shape_Registry = physics.simulation_shape_registry(simulation);
	raw_a: rawptr;
	resolve_a: Status;
	raw_a, _, resolve_a = physics.shape_registry_resolve(registry, shape_a);
	if resolve_a != .Ok
	{
		return {}, resolve_a;
	}
	raw_b: rawptr;
	resolve_b: Status;
	raw_b, _, resolve_b = physics.shape_registry_resolve(registry, shape_b);
	if resolve_b != .Ok
	{
		return {}, resolve_b;
	}
	type_a: int = int(physics.typed_index_type(shape_a));
	type_b: int = int(physics.typed_index_type(shape_b));
	task: ^physics.Collision_Task;
	lookup_status: Status;
	task, _, lookup_status = physics.collision_task_registry_lookup(
		&simulation.collision_tasks, type_a, type_b,
	);
	if lookup_status != .Ok
	{
		return {}, lookup_status;
	}
	if task.kind == .Convex
	{
		manifold: physics.Convex_Contact_Manifold;
		status: Status;
		manifold, status = physics.collision_task_registry_test_convex(
			&simulation.collision_tasks, type_a, type_b,
			raw_a, raw_b, pose_a, pose_b, speculative_margin, registry,
		);
		if status != .Ok
		{
			return {}, status;
		}
		result: Manifold_Result = Manifold_Result{kind=.Convex, convex=manifold};
		if manifold.count <= 0
		{
			return result, .Not_Found;
		}
		return result, .Ok;
	}
	if speculative_margin != 0
	{
		return {}, .Invalid_Argument;
	}
	hit: physics.Overlap_Query_Hit;
	collector: physics.Overlap_Query_Collector = physics.Overlap_Query_Collector{
		hits={memory=&hit, length=1, id=util.BUFFER_CALLER_OWNED_ID},
	};
	status: Status = physics.query_overlap_batcher_test(
		&query_data.overlap,
		shape_a, shape_b, pose_a, pose_b, 0, &collector,
	);
	if status != .Ok
	{
		return {}, status;
	}
	if collector.count == 0
	{
		return {}, .Not_Found;
	}
	return hit.manifold, .Ok;
}

collision_query_with_context :: proc "contextless" (
	query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Rigid_Pose,
	shape_b: Shape_Handle, pose_b: Rigid_Pose, speculative_margin: f32 = 0,
) -> (Manifold_Result, Status)
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return {}, ready;
	}
	defer data.execution = .Idle;
	return query_context_collision_execute(data, shape_a, pose_a, shape_b, pose_b, speculative_margin);
}

collision_query_batch_with_context :: proc "contextless" (
	query_context: ^Query_Context, queries: []Collision_Query, results: []Collision_Query_Result,
) -> Status
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return ready;
	}
	defer data.execution = .Idle;
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	overall: Status = Status.Ok;
	for query, index in queries
	{
		manifold: Manifold_Result;
		status: Status;
		manifold, status = query_context_collision_execute(data, query.shape_a, query.pose_a, query.shape_b, query.pose_b, query.speculative_margin);
		result: ^Collision_Query_Result = &results[index];
		result.manifold = manifold;
		result.status = status;
		result.hit = status == .Ok;
		if overall == .Ok && status != .Ok && status != .Not_Found
		{
			overall = status;
		}
	}
	return overall;
}

query_batch_with_context :: proc "contextless" (
	query_context: ^Query_Context, queries: []Query, results: []Query_Result, scratch: ^Query_Scratch = nil,
) -> Status
{
	data: ^query_context_data;
	ready: Status;
	data, ready = query_context_begin(query_context);
	if ready != .Ok
	{
		return ready;
	}
	defer data.execution = .Idle;
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	first_error: Status = Status.Ok;
	for query, index in queries
	{
		result: ^Query_Result = &results[index];
		result^ = {};
		switch query.kind
		{
			case .Ray_Any, .Ray_Closest:
			input: query_ray_data = query.ray_data;
			hit: [1]Ray_Hit;
			count: int;
			status: Status;
			count, status = query_context_ray_execute(data, input.ray, hit[:], input.filter, .Earliest, .Enabled if query.kind==.Ray_Any else .Disabled);
			result.status = status;
			if query.kind == .Ray_Closest
			{
				result.ray_hit = {child_index=-1};
			}
			if status == .Ok
			{
				result.count = i32(count);
				result.hit = count > 0;
				if query.kind == .Ray_Closest
				{
					if count > 0
					{
						result.ray_hit = hit[0];
					}
					else
					{
						result.status = .Not_Found;
					}
				}
			}
			case .Ray_All:
			input: query_ray_data = query.ray_data;
			if scratch == nil || !query_output_range_valid(input.output, len(scratch.ray_hits))
			{
				result.status = .Invalid_Argument;
				break;
			}
			hits: []Ray_Hit = scratch.ray_hits[int(input.output.start):int(input.output.start)+int(input.output.capacity)];
			temporary: [1]Ray_Hit;
			output: []Ray_Hit = hits;
			if len(output) == 0
			{
				output = temporary[:];
			}
			mode: physics.Query_Collection_Mode = physics.Query_Collection_Mode.All;
			if len(hits) == 0
			{
				mode = .Earliest;
			}
			count: int;
			status: Status;
			count, status = query_context_ray_execute(data, input.ray, output, input.filter, mode, .Enabled if len(hits) == 0 else .Disabled);
			if len(hits) == 0
			{
				if status == .Ok && count > 0
				{
					status = .Capacity_Missing;
				}
				count = 0;
			}
			result.status = status;
			result.count = i32(count);
			result.hit = count > 0;
			case .Sweep_Closest:
			input: query_sweep_data = query.sweep_data;
			hit: [1]Sweep_Hit;
			count: int;
			status: Status;
			count, status = query_context_sweep_execute(data, input.shape, input.pose, input.velocity, input.maximum_t,
				hit[:], input.filter, input.settings, .Earliest, .Disabled);
			result.status = status;
			if status == .Ok
			{
				result.hit = count > 0;
				result.count = i32(count);
				if count > 0
				{
					result.sweep_hit = hit[0];
				}
				else
				{
					result.status = .Not_Found;
				}
			}
			case .Overlap_All:
			input: query_overlap_data = query.overlap_data;
			if scratch == nil || !query_output_range_valid(input.output, len(scratch.overlap_hits))
			{
				result.status = .Invalid_Argument;
				break;
			}
			hits: []Overlap_Hit = scratch.overlap_hits[int(input.output.start):int(input.output.start)+int(input.output.capacity)];
			count: int;
			status: Status;
			count, status = query_context_overlap_execute(data, input.shape, input.pose, hits, input.filter);
			result.status = status;
			result.count = i32(count);
			result.hit = count > 0;
			case .Volume_All:
			input: query_volume_data = query.volume_data;
			if scratch == nil || !query_output_range_valid(input.output, len(scratch.volume_hits))
			{
				result.status = .Invalid_Argument;
				break;
			}
			hits: []Volume_Hit = scratch.volume_hits[int(input.output.start):int(input.output.start)+int(input.output.capacity)];
			count: int;
			status: Status;
			count, status = query_context_volume_execute(data, input.bounds, hits, input.filter);
			result.status = status;
			result.count = i32(count);
			result.hit = count > 0;
			case:
			result.status = .Invalid_Argument;
		}
		if first_error == .Ok && result.status != .Ok
		{
			first_error = result.status;
		}
	}
	return first_error;
}

// admits only callback-free closest-ray batches without live custom shapes
// before a C read phase may use the ordinary Odin batch implementation
query_batch_read_only_status :: proc "contextless" (world: ^World, queries: []Query) -> Status
{
	data: ^world_data;
	status: Status;
	data, status = query_world_data(world);
	if status != .Ok
	{
		return .Invalid_Argument;
	}
	shapes: ^physics.Shape_Registry = physics.simulation_shape_registry(&data.simulation);
	for type_id in physics.BUILT_IN_SHAPE_TYPE_COUNT ..< shapes.registered_type_count
	{
		if shapes.batches[type_id].active_count != 0
		{
			return .Invalid_Argument;
		}
	}
	for query in queries
	{
		if query.kind != .Ray_Closest || query.ray_data.filter.allow != nil ||
		query.ray_data.filter.allow_child != nil || physics.shape_ray_valid(query.ray_data.ray) != .Ok
		{
			return .Invalid_Argument;
		}
	}
	return .Ok;
}

// independent scratch and the same early-out route as the ordinary query
overlap_any_with_context :: proc "contextless" (
	query_context: ^Query_Context, shape: Shape_Handle, pose_value: Rigid_Pose, filter: Query_Filter = {},
) -> (Overlap_State, Status)
{
	data: ^query_context_data;
	status: Status;
	data, status = query_context_begin(query_context);
	if status != .Ok
	{
		return .Separated, status;
	}
	defer data.execution = .Idle;
	filter_context: query_filter_context = {filter=filter};
	return physics.simulation_overlap_any_query(&data.world.simulation, shape, pose_value,
		query_filter_low_level(filter), &filter_context, data.pool, &data.overlap);
}
