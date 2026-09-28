package entasis

import "base:intrinsics"
import "base:runtime"
import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(private)
world_resource :: enum u8
{
	Pool,
	Dispatcher,
}

@(private)
world_ownership :: distinct bit_set[world_resource; u8];
@(private)
world_data :: struct
{
	simulation:       physics.Simulation,
	owned_pool:       util.Buffer_Pool,
	owned_dispatcher: util.Thread_Dispatcher,
	pool:             ^util.Buffer_Pool,
	dispatcher:       ^Dispatcher,
	allocator:        mem.Allocator,
	view_epoch:       u64,
	ownership:        world_ownership,
	allocation_scope: Allocation_Scope,
	query_contexts:   ^query_context_data,
	contextual_tasks: ^contextual_task_record,
	contextual_shapes: ^contextual_shape_record,
	contextual_constraints: ^contextual_constraint_record,
	distance_storage: ^distance_query_storage,
}

// World is a caller-owned opaque handle for exactly one low-level simulation.
// initialize a zero value with world_init or world_init_with_pool. do not copy a
// ready World. destroy it on the owner thread while no step or query is active
World :: distinct rawptr;
Allocation_Scope :: util.Allocation_Scope;
#assert(size_of(World) == size_of(rawptr));
#assert(align_of(World) == align_of(rawptr));
@(private)
world_data_get :: #force_inline proc "contextless" (world: ^World) -> ^world_data
{
	if world == nil
	{
		return nil;
	}
	pointer := rawptr(world^);
	if pointer == nil
	{
		return nil;
	}
	return (^world_data)(pointer);
}

@(private)
world_invalidate_views_data :: #force_inline proc "contextless" (data: ^world_data)
{
	if data == nil
	{
		return;
	}
	data.view_epoch += 1;
	if data.view_epoch == 0
	{
		data.view_epoch = 1;
	}
}

@(private)
world_invalidate_views :: #force_inline proc "contextless" (world: ^World)
{
	world_invalidate_views_data(world_data_get(world));
}

@(private)
world_memory_status :: proc "contextless" (status: util.Memory_Status) -> Status
{
	switch status
	{
		case .Ok:
		return .Ok;
		case .Pool_Disposed:
		return .Disposed;
		case .Invalid_Count, .Invalid_Alignment, .Invalid_Power, .Invalid_Buffer:
		return .Invalid_Argument;
		case .Capacity_Missing, .Out_Of_Memory, .Overflow:
		return .Capacity_Missing;
	}
	return .Capacity_Missing;
}

@(private)
world_threading_status :: proc "contextless" (status: util.Threading_Status) -> Status
{
	switch status
	{
		case .Ok:
		return .Ok;
		case .Capacity_Missing:
		return .Capacity_Missing;
		case .Shutting_Down, .Disposed:
		return .Disposed;
		case .Invalid_Argument, .Already_Running, .Not_Running:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

@(private)
world_allocator :: #force_inline proc (allocator: mem.Allocator) -> mem.Allocator
{
	if allocator.procedure == nil
	{
		return runtime.heap_allocator();
	}
	return allocator;
}

@(private)
world_solve_low_level :: proc "contextless" (
	description: Solve_Description,
) -> (physics.Solve_Description, Status)
{
	low_level := physics.Solve_Description{
		velocity_iteration_count=description.velocity_iterations,
		substep_count=description.substeps,
		fallback_batch_threshold=description.fallback_batch_threshold,
		velocity_iteration_scheduler=description.velocity_iteration_scheduler,
		scheduler_context=description.scheduler_context,
	};
	status := physics.solve_description_validate(low_level);
	return low_level, status;
}

@(private)
world_capacity_low_level :: proc "contextless" (
	hints: Capacity_Hints, worker_count: int, solve: physics.Solve_Description,
) -> (physics.Simulation_Allocation_Sizes, Status)
{
	if worker_count <= 0 || worker_count > physics.MAXIMUM_SOLVER_WORKER_COUNT
	{
		return {}, .Invalid_Description;
	}
	if solve.fallback_batch_threshold >= max(i32)
	{
		return {}, .Invalid_Description;
	}
	collision_child_pairs := hints.collision_child_pairs;
	if collision_child_pairs == 0
	{
		if hints.pairs <= 0 || hints.pairs > max(i32) / 16
		{
			return {}, .Invalid_Description;
		}
		collision_child_pairs = hints.pairs * 16;
	}
	sizes := physics.Simulation_Allocation_Sizes{
		bodies=hints.bodies,
		statics=hints.statics,
		inactive_body_sets=hints.inactive_body_sets,
		shapes_per_type=hints.shapes_per_type,
		constraints=hints.constraints,
		constraint_batches=solve.fallback_batch_threshold + 1,
		initial_constraints_per_type_batch=hints.initial_constraints_per_type_batch,
		minimum_constraints_per_body=hints.minimum_constraints_per_body,
		broad_phase_candidates=hints.broad_phase_candidates,
		pairs=hints.pairs,
		collision_child_pairs=collision_child_pairs,
		inactive_pairs=hints.inactive_pairs,
		pending_pairs_per_worker=hints.pending_pairs_per_worker,
		workers=i32(worker_count),
	};
	normalized, status := physics.simulation_allocation_sizes_normalize(sizes, solve);
	return normalized, status;
}

@(private)
world_effective_worker_count :: proc "contextless" (
	threading: Threading_Description,
) -> (int, Status)
{
	if threading.external_dispatcher != nil
	{
		dispatcher := threading.external_dispatcher;
		if dispatcher.dispatch == nil || dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > physics.MAXIMUM_SOLVER_WORKER_COUNT
		{
			return 0, .Invalid_Description;
		}
		return dispatcher.worker_count, .Ok;
	}
	if threading.worker_count <= 0 ||
	threading.worker_count > physics.MAXIMUM_SOLVER_WORKER_COUNT ||
	threading.worker_pool_block_size <= 0
	{
		return 0, .Invalid_Description;
	}
	return int(threading.worker_count), .Ok;
}

@(private)
world_description_validate :: proc "contextless" (
	description: World_Description,
) -> (physics.Solve_Description, physics.Simulation_Allocation_Sizes, int, Status)
{
	if description.damping.linear < 0 || description.damping.linear > 1 ||
	description.damping.angular < 0 || description.damping.angular > 1
	{
		return {}, {}, 0, .Invalid_Description;
	}
	solve, solve_status := world_solve_low_level(description.solve);
	if solve_status != .Ok
	{
		return {}, {}, 0, solve_status;
	}
	workers, worker_status := world_effective_worker_count(description.threading);
	if worker_status != .Ok
	{
		return {}, {}, 0, worker_status;
	}
	capacity, capacity_status := world_capacity_low_level(description.capacity, workers, solve);
	if capacity_status != .Ok
	{
		return {}, {}, 0, capacity_status;
	}
	if narrow_callbacks_state(description.narrow_callbacks) == .Configured &&
	physics.narrow_phase_callbacks_validate(description.narrow_callbacks) != .Ok
	{
		return {}, {}, 0, .Invalid_Description;
	}
	if pose_callbacks_state(description.pose_callbacks) == .Configured &&
	physics.pose_integrator_callbacks_validate(description.pose_callbacks) != .Ok
	{
		return {}, {}, 0, .Invalid_Description;
	}
	return solve, capacity, workers, .Ok;
}

@(private)
world_release_data :: proc (data: ^world_data) -> Status
{
	if data == nil
	{
		return .Ok;
	}
	result := Status.Ok;
	allocator := data.allocator;
	if .Dispatcher in data.ownership
	{
		status := world_threading_status(util.thread_dispatcher_shutdown(&data.owned_dispatcher));
		if result == .Ok && status != .Ok
		{
			result = status;
		}
	}
	// simulation teardown precedes this owner release. no callback may retain a binding
	constraint_bindings_status: Status = contextual_constraint_records_release(data);
	if result == .Ok && constraint_bindings_status != .Ok
	{
		result = constraint_bindings_status;
	}
	shape_bindings_status: Status = contextual_shape_records_release(data);
	if result == .Ok && shape_bindings_status != .Ok
	{
		result = shape_bindings_status;
	}
	bindings_status: Status = contextual_task_records_release(data);
	if result == .Ok && bindings_status != .Ok
	{
		result = bindings_status;
	}
	distance_query_storage_destroy(&data.distance_storage, data.pool, allocator, data.allocation_scope);
	if .Pool in data.ownership
	{
		status := world_memory_status(util.buffer_pool_dispose(&data.owned_pool));
		if result == .Ok && status != .Ok
		{
			result = status;
		}
	}
	allocator_error: mem.Allocator_Error = util.allocation_free(data, size_of(world_data), align_of(world_data), allocator, data.allocation_scope); // odin-contracts-allow: setup rule=ODIN_HOT_DELETE_OR_FREE owner=World phase=destroy reason=facade_lifecycle_release
	if result == .Ok && allocator_error != .None
	{
		result = .Invalid_Argument;
	}
	return result;
}

@(private)
world_init_internal :: proc (
	world: ^World,
	description: World_Description,
	external_pool: ^Buffer_Pool,
	allocation_scope: Allocation_Scope = .Legacy,
) -> Status
{
	if world == nil || world_data_get(world) != nil ||
	(allocation_scope != .Legacy && allocation_scope != .All_Owned)
	{
		return .Invalid_Argument;
	}
	solve, capacity, workers, description_status := world_description_validate(description);
	if description_status != .Ok
	{
		return description_status;
	}
	if external_pool != nil && external_pool.state != .Ready
	{
		return .Invalid_Argument;
	}
	allocator := world_allocator(description.allocator);
	memory, allocation_error := mem.alloc( // odin-contracts-allow: setup rule=ODIN_HOT_ALLOCATE_OR_REALLOC owner=World phase=initialize reason=one_time_facade_owner_allocation
		size_of(world_data), align_of(world_data), allocator,
	);
	if allocation_error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	data := (^world_data)(memory);
	intrinsics.mem_zero(data, size_of(world_data));
	data.allocator = allocator;
	data.allocation_scope = allocation_scope;
	data.view_epoch = 1;
	pool := external_pool;
	if pool == nil
	{
		pool_status: Status = world_memory_status(util.buffer_pool_initialize_internal(
				&data.owned_pool, 131072, util.BUFFER_POOL_DEFAULT_EXPECTED_RESOURCE_COUNT,
				runtime.heap_allocator() if allocation_scope == .Legacy else allocator, allocation_scope,
		));
		if pool_status != .Ok
		{
			_ = world_release_data(data);
			return pool_status;
		}
		pool = &data.owned_pool;
		data.ownership += {.Pool};
	}
	data.pool = pool;
	dispatcher := description.threading.external_dispatcher;
	if dispatcher == nil && workers > 1
	{
		threading_status: Status = world_threading_status(util.thread_dispatcher_initialize_internal(
				&data.owned_dispatcher, workers, int(description.threading.worker_pool_block_size),
				runtime.heap_allocator() if allocation_scope == .Legacy else allocator, allocation_scope,
		));
		if threading_status != .Ok
		{
			_ = world_release_data(data);
			return threading_status;
		}
		dispatcher = util.thread_dispatcher_boundary(&data.owned_dispatcher);
		data.ownership += {.Dispatcher};
	}
	data.dispatcher = dispatcher;
	create_description := physics.simulation_create_description_default(pool);
	create_description.allocation_sizes = capacity;
	create_description.solve_description = solve;
	create_description.default_pose_context.gravity = description.gravity;
	create_description.default_pose_context.linear_damping = description.damping.linear;
	create_description.default_pose_context.angular_damping = description.damping.angular;
	if narrow_callbacks_state(description.narrow_callbacks) == .Configured
	{
		create_description.narrow_callbacks = description.narrow_callbacks;
		create_description.use_default_narrow = .Missing;
	}
	if pose_callbacks_state(description.pose_callbacks) == .Configured
	{
		create_description.pose_callbacks = description.pose_callbacks;
		create_description.use_default_pose = .Missing;
	}
	if description.timestepper.step != nil
	{
		create_description.timestepper = description.timestepper;
	}
	create_description.timestep_callbacks = description.timestep_callbacks;
	if description.profiling
	{
		create_description.profiling = .Enabled;
	}
	else
	{
		create_description.profiling = .Disabled;
	}
	create_result := physics.simulation_create(&data.simulation, &create_description);
	if create_result.status != .Ok
	{
		_ = world_release_data(data);
		return create_result.status;
	}
	stored_material, stored_material_state :=
	narrow_policy_default_stored_material(description.narrow_callbacks);
	if stored_material_state == .Present
	{
		stored_status := physics.narrow_phase_bind_default_stored_completion(
			&data.simulation.narrow_phase, stored_material,
		);
		if stored_status != .Ok
		{
			_ = physics.simulation_destroy(&data.simulation);
			_ = world_release_data(data);
			return stored_status;
		}
	}
	world^ = World(rawptr(data));
	return .Ok;
}

// world_init creates one world with a facade-owned Buffer_Pool. a dispatcher is
// also owned when description.threading.worker_count is greater than one and no
// external dispatcher was supplied
world_init :: proc (
	world: ^World,
	description: World_Description,
) -> Status
{
	return world_init_internal(world, description, nil);
}

// world_init_with_pool creates one world using a caller-owned Buffer_Pool. the
// world returns its buffers during destruction but never disposes the pool
world_init_with_pool :: proc (
	world: ^World,
	description: World_Description,
	pool: ^Buffer_Pool,
) -> Status
{
	if pool == nil
	{
		return .Invalid_Argument;
	}
	return world_init_internal(world, description, pool);
}

// allocator-aware siblings preserve the legacy World_Description layout. a
// borrowed pool or external dispatcher always retains its independent owner
world_init_with_allocation_scope :: proc (
	world: ^World, description: World_Description, scope: Allocation_Scope,
) -> Status
{
	return world_init_internal(world, description, nil, scope);
}

world_init_with_pool_and_allocation_scope :: proc (
	world: ^World, description: World_Description, pool: ^Buffer_Pool, scope: Allocation_Scope,
) -> Status
{
	if pool == nil
	{
		return .Invalid_Argument;
	}
	return world_init_internal(world, description, pool, scope);
}

// world_step advances one timestep using the explicit dispatcher when supplied,
// otherwise the dispatcher selected during initialization
world_step :: #force_inline proc (
	world: ^World,
	dt: f32,
	dispatcher: ^Dispatcher = nil,
) -> Status
{
	data := world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	world_invalidate_views_data(data);
	selected_dispatcher := dispatcher;
	if selected_dispatcher == nil
	{
		selected_dispatcher = data.dispatcher;
	}
	return physics.simulation_timestep(&data.simulation, dt, selected_dispatcher);
}

// world_clear removes all bodies, statics, constraints, shapes, sleeping sets,
// cached pairs, and accumulated step state while retaining allocated capacity
world_clear :: proc (world: ^World) -> Status
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return .Disposed;
	}
	world_invalidate_views_data(data);
	return physics.simulation_clear(&data.simulation);
}

// world_ensure_capacity grows world storage to satisfy the supplied initial
// capacity hints. it never shrinks storage or changes the initialized threading policy
world_ensure_capacity :: proc (
	world: ^World,
	hints: Capacity_Hints,
) -> Status
{
	data := world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	worker_count := int(data.simulation.allocation_sizes.workers);
	capacity, capacity_status := world_capacity_low_level(
		hints, worker_count, data.simulation.solve_description,
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_ensure_capacity(&data.simulation, capacity);
}

// world_resize grows or shrinks retained world storage toward the supplied
// capacity hints. live counts are always preserved by the low-level resize
// operation. a target below current occupancy is raised to the required live
// capacity. this is an owner-thread cold operation and invalidates direct views
world_resize :: proc (
	world: ^World,
	hints: Capacity_Hints,
) -> Status
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return .Disposed;
	}
	worker_count := int(data.simulation.allocation_sizes.workers);
	capacity, capacity_status := world_capacity_low_level(
		hints, worker_count, data.simulation.solve_description,
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_resize(&data.simulation, capacity);
}

// world_destroy releases the simulation and every facade-owned resource. it
// never disposes an external pool or external dispatcher. a destroyed World may
// be initialized again
world_destroy :: proc (world: ^World) -> Status
{
	data := world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	if data.query_contexts != nil
	{
		return .Invalid_Argument;
	}
	status := physics.simulation_destroy(&data.simulation);
	if status != .Ok
	{
		return status;
	}
	world^ = World(nil);
	return world_release_data(data);
}
