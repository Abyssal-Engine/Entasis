// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"

Island_Sleeper_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

Island_Sleep_Transaction :: struct
{
	set_index:       i32,
	record_start:    i32,
	body_count:      i32,
	constraint_count: i32,
}

Island_Sleeper :: struct
{
	scaffold:            Island_Scaffold,
	worker_scaffolds:    [MAXIMUM_SOLVER_WORKER_COUNT]Island_Scaffold,
	worker_count:        int,
	traversal_seeds:     [MAXIMUM_SOLVER_WORKER_COUNT]Body_Handle,
	traversal_body_counts: [MAXIMUM_SOLVER_WORKER_COUNT]int,
	traversal_constraint_counts: [MAXIMUM_SOLVER_WORKER_COUNT]int,
	traversal_can_sleep: [MAXIMUM_SOLVER_WORKER_COUNT]Reference_State,
	worker_statuses:     [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
	inactive_constraints: util.Buffer(Inactive_Constraint_Record),
	constraint_order:      util.Buffer(Solver_Remove_Order_Entry),
	constraint_sort_scratch: util.Buffer(Solver_Remove_Order_Entry),
	body_migrations:      util.Buffer(Broad_Phase_Body_Migration),
	body_claims:          util.Buffer(Solver_Transaction_Body_Claim),
	body_batch_claims:     util.Buffer(Solver_Transaction_Body_Batch_Claim),
	type_claims:          util.Buffer(Solver_Transaction_Count_Claim),
	inactive_constraint_count: int,
	claim_generation:    u32,
	bodies:              ^Bodies,
	broad_phase:         ^Broad_Phase,
	solver:              ^Solver,
	pair_cache:          ^Pair_Cache,
	pool:                ^util.Buffer_Pool,
	next_set_index:      int,
	schedule_offset:     int,
	tested_fraction_per_frame: f32,
	target_slept_fraction: f32,
	target_traversed_fraction: f32,
	state:               Island_Sleeper_State,
}

island_sleeper_resolve_inactive_constraint_index :: proc "contextless" (
	sleeper: ^Island_Sleeper, handle: Constraint_Handle,
) -> (int, Physics_Status)
{
	if sleeper == nil || sleeper.state != .Ready || sleeper.solver == nil ||
	sleeper.solver.state != .Ready || handle.value < 0 ||
	int(handle.value) >= int(sleeper.solver.handle_to_constraint.length)
	{
		return -1, .Not_Found;
	}
	location := sleeper.solver.handle_to_constraint.memory[handle.value];
	record_index := int(location.index_in_type_batch);
	if location.set_index <= 0 || record_index < 0 ||
	record_index >= sleeper.inactive_constraint_count
	{
		return -1, .Not_Found;
	}
	record := &sleeper.inactive_constraints.memory[record_index];
	if record.handle != handle || record.set_index != location.set_index ||
	record.type_id != location.type_id
	{
		return -1, .Not_Found;
	}
	return record_index, .Ok;
}

island_sleeper_initialize :: proc (
	sleeper: ^Island_Sleeper, bodies: ^Bodies, broad_phase: ^Broad_Phase,
	solver: ^Solver, pair_cache: ^Pair_Cache, body_capacity, constraint_capacity, worker_count: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Uninitialized || bodies == nil || bodies.state != .Allocated ||
	broad_phase == nil || broad_phase.state != .Ready || solver == nil || solver.state != .Ready ||
	pair_cache == nil || pair_cache.state != .Ready || pool == nil || worker_count <= 0 ||
	worker_count > MAXIMUM_SOLVER_WORKER_COUNT ||
	int(solver.fallback_batch_threshold) > max(int) /
	max(util.index_set_bundle_capacity(int(bodies.handle_to_location.length)), 1)
	{
		return .Invalid_Argument;
	}
	status := island_scaffold_initialize(&sleeper.scaffold, body_capacity, constraint_capacity, pool);
	if status != .Ok
	{
		return status;
	}
	initialized_worker_count := 0;
	for worker_index in 0 ..< worker_count
	{
		status = island_scaffold_initialize(
			&sleeper.worker_scaffolds[worker_index], body_capacity, constraint_capacity, pool,
		);
		if status != .Ok
		{
			for initialized_index in 0 ..< initialized_worker_count
			{
				_ = island_scaffold_dispose(&sleeper.worker_scaffolds[initialized_index]);
			}
			_ = island_scaffold_dispose(&sleeper.scaffold);
			return status;
		}
		initialized_worker_count += 1;
	}
	records, records_status := util.buffer_pool_take_at_least(pool, Inactive_Constraint_Record, constraint_capacity);
	if records_status != .Ok
	{
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(records_status);
	}
	constraint_order, order_status := util.buffer_pool_take_at_least(
		pool, Solver_Remove_Order_Entry, constraint_capacity,
	);
	if order_status != .Ok
	{
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(order_status);
	}
	constraint_sort_scratch, sort_status := util.buffer_pool_take_at_least(
		pool, Solver_Remove_Order_Entry, constraint_capacity,
	);
	if sort_status != .Ok
	{
		physics_return_buffer(pool, &constraint_order);
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(sort_status);
	}
	body_migrations, migration_status := util.buffer_pool_take_at_least(
		pool, Broad_Phase_Body_Migration, body_capacity,
	);
	if migration_status != .Ok
	{
		physics_return_buffer(pool, &constraint_sort_scratch);
		physics_return_buffer(pool, &constraint_order);
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(migration_status);
	}
	body_claims, body_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Body_Claim, int(bodies.handle_to_location.length),
	);
	if body_claim_status != .Ok
	{
		physics_return_buffer(pool, &body_migrations);
		physics_return_buffer(pool, &constraint_sort_scratch);
		physics_return_buffer(pool, &constraint_order);
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(body_claim_status);
	}
	body_batch_claims, body_batch_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Body_Batch_Claim,
		max(util.index_set_bundle_capacity(int(bodies.handle_to_location.length)), 1) *
		int(solver.fallback_batch_threshold),
	);
	if body_batch_claim_status != .Ok
	{
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &body_migrations);
		physics_return_buffer(pool, &constraint_sort_scratch);
		physics_return_buffer(pool, &constraint_order);
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(body_batch_claim_status);
	}
	type_claims, type_claim_status := util.buffer_pool_take_at_least(
		pool, Solver_Transaction_Count_Claim,
		int(solver.active_set.batches.length) * CONSTRAINT_TYPE_ID_CAPACITY,
	);
	if type_claim_status != .Ok
	{
		physics_return_buffer(pool, &body_batch_claims);
		physics_return_buffer(pool, &body_claims);
		physics_return_buffer(pool, &body_migrations);
		physics_return_buffer(pool, &constraint_sort_scratch);
		physics_return_buffer(pool, &constraint_order);
		physics_return_buffer(pool, &records);
		for worker_index in 0 ..< initialized_worker_count
		{
			_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
		}
		_ = island_scaffold_dispose(&sleeper.scaffold);
		return physics_memory_status(type_claim_status);
	}
	_ = util.buffer_clear(records, 0, int(records.length));
	_ = util.buffer_clear(body_claims, 0, int(body_claims.length));
	_ = util.buffer_clear(body_batch_claims, 0, int(body_batch_claims.length));
	_ = util.buffer_clear(type_claims, 0, int(type_claims.length));
	sleeper.inactive_constraints = records;
	sleeper.constraint_order = constraint_order;
	sleeper.constraint_sort_scratch = constraint_sort_scratch;
	sleeper.body_migrations = body_migrations;
	sleeper.body_claims = body_claims;
	sleeper.body_batch_claims = body_batch_claims;
	sleeper.type_claims = type_claims;
	sleeper.worker_count = worker_count;
	sleeper.bodies = bodies;
	sleeper.broad_phase = broad_phase;
	sleeper.solver = solver;
	sleeper.pair_cache = pair_cache;
	sleeper.pool = pool;
	sleeper.next_set_index = 1;
	sleeper.tested_fraction_per_frame = 0.01;
	sleeper.target_slept_fraction = 0.005;
	sleeper.target_traversed_fraction = 0.01;
	sleeper.state = .Ready;
	return .Ok;
}

island_sleeper_ensure_capacity :: proc (
	sleeper: ^Island_Sleeper, body_capacity, constraint_capacity: int,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready || body_capacity <= 0 ||
	constraint_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := island_scaffold_ensure_capacity(
		&sleeper.scaffold, body_capacity, constraint_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	for worker_index in 0 ..< sleeper.worker_count
	{
		status = island_scaffold_ensure_capacity(
			&sleeper.worker_scaffolds[worker_index],
			body_capacity, constraint_capacity,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status = physics_ensure_buffer_capacity(
		sleeper.pool, &sleeper.inactive_constraints, constraint_capacity,
		sleeper.inactive_constraint_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		sleeper.pool, &sleeper.constraint_order, constraint_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		sleeper.pool, &sleeper.constraint_sort_scratch,
		constraint_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		sleeper.pool, &sleeper.body_migrations, body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.body_claims, body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	body_batch_claim_capacity :=
	max(util.index_set_bundle_capacity(body_capacity), 1) *
	int(sleeper.solver.fallback_batch_threshold);
	status = physics_ensure_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.body_batch_claims,
		body_batch_claim_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	return physics_ensure_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.type_claims,
		int(sleeper.solver.active_set.batches.length) *
		CONSTRAINT_TYPE_ID_CAPACITY,
	);
}

island_sleeper_resize :: proc (
	sleeper: ^Island_Sleeper, body_capacity, constraint_capacity: int,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready || body_capacity <= 0 ||
	constraint_capacity <= 0
	{
		return .Invalid_Argument;
	}
	required_body_capacity := max(
		body_capacity, int(sleeper.bodies.handle_pool.next_index),
	);
	required_constraint_capacity := max(
		constraint_capacity, int(sleeper.solver.handle_pool.next_index),
	);
	status := island_scaffold_resize(
		&sleeper.scaffold, required_body_capacity,
		required_constraint_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	for worker_index in 0 ..< sleeper.worker_count
	{
		status = island_scaffold_resize(
			&sleeper.worker_scaffolds[worker_index],
			required_body_capacity, required_constraint_capacity,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status = physics_resize_buffer_capacity(
		sleeper.pool, &sleeper.inactive_constraints,
		max(required_constraint_capacity, sleeper.inactive_constraint_count),
		sleeper.inactive_constraint_count,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		sleeper.pool, &sleeper.constraint_order,
		required_constraint_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		sleeper.pool, &sleeper.constraint_sort_scratch,
		required_constraint_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		sleeper.pool, &sleeper.body_migrations,
		required_body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.body_claims, required_body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	body_batch_claim_capacity :=
	max(
		util.index_set_bundle_capacity(required_body_capacity), 1,
	) * int(sleeper.solver.fallback_batch_threshold);
	status = physics_resize_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.body_batch_claims,
		body_batch_claim_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	return physics_resize_zeroed_buffer_capacity(
		sleeper.pool, &sleeper.type_claims,
		int(sleeper.solver.active_set.batches.length) *
		CONSTRAINT_TYPE_ID_CAPACITY,
	);
}

island_sleeper_update_candidacy :: proc "contextless" (
	activity: ^Body_Activity, velocity: Body_Velocity,
)
{
	velocity_squared := util.vector3_length_squared(velocity.linear) +
	util.vector3_length_squared(velocity.angular);
	if velocity_squared <= activity.sleep_threshold
	{
		if activity.timesteps_under_threshold_count < 255
		{
			activity.timesteps_under_threshold_count += 1;
		}
		if activity.timesteps_under_threshold_count >= activity.minimum_timesteps_under_threshold
		{
			activity.sleep_candidate = .Candidate;
		}
	}
	else
	{
		activity.timesteps_under_threshold_count = 0;
		activity.sleep_candidate = .Not_Candidate;
	}
}

Island_Sleeper_Candidate_Job :: struct
{
	sleeper:      ^Island_Sleeper,
	active_count:  int,
	candidate_count: int,
	spacing:       int,
	schedule_offset: int,
	worker_count: int,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

island_sleeper_candidate_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Sleeper_Candidate_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	active := &job.sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for candidate_index := worker_index;
	candidate_index < job.candidate_count;
	candidate_index += job.worker_count
	{
		index := (job.schedule_offset + candidate_index * job.spacing) % job.active_count;
		job.sleeper.scaffold.stack.memory[candidate_index] =
		active.index_to_handle.memory[index];
	}
	job.statuses[worker_index] = status;
}

island_sleeper_update_candidates :: proc (
	sleeper: ^Island_Sleeper, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> (int, Physics_Status)
{
	active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if active.count == 0
	{
		return 0, .Ok;
	}
	candidate_count := max(
		1, int(f32(active.count) * sleeper.tested_fraction_per_frame),
	);
	spacing := active.count / candidate_count;
	if sleeper.schedule_offset > active.count
	{
		sleeper.schedule_offset = 0;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		worker_count = min(dispatcher.worker_count, candidate_count);
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return 0, .Capacity_Missing;
	}
	statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
	job := Island_Sleeper_Candidate_Job{
		sleeper=sleeper,
		active_count=active.count,
		candidate_count=candidate_count,
		spacing=spacing,
		schedule_offset=sleeper.schedule_offset,
		worker_count=worker_count,
		statuses=&statuses,
	};
	if dispatcher != nil && worker_count > 1
	{
		dispatch_status := dispatcher.dispatch(dispatcher, island_sleeper_candidate_worker, worker_count, &job);
		if dispatch_status != .Ok
		{
			return 0, .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if statuses[worker_index] != .Ok
			{
				return 0, statuses[worker_index];
			}
		}
	}
	else
	{
		boundary := util.Thread_Dispatcher_Boundary{unmanaged_context=&job};
		island_sleeper_candidate_worker(0, &boundary);
		if statuses[0] != .Ok
		{
			return 0, statuses[0];
		}
	}
	sleeper.schedule_offset += 1;
	return candidate_count, .Ok;
}

island_sleeper_build_island :: proc "contextless" (
	sleeper: ^Island_Sleeper, scaffold: ^Island_Scaffold, seed: Body_Handle,
) -> (body_count, constraint_count: int, can_sleep: Reference_State, status: Physics_Status)
{
	if scaffold == nil || scaffold.state != .Ready
	{
		status = .Invalid_Argument;
		return;
	}
	if seed.value < 0 || int(seed.value) >= int(scaffold.body_marks.length)
	{
		status = .Capacity_Missing;
		return;
	}
	stack_count := 1;
	scaffold.stack.memory[0] = seed;
	scaffold.body_marks.memory[seed.value] = 1;
	can_sleep = .Present;
	for stack_count > 0
	{
		stack_count -= 1;
		handle := scaffold.stack.memory[stack_count];
		location, resolve_status := bodies_resolve(sleeper.bodies, handle);
		if resolve_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			status = .Invalid_Argument;
			return;
		}
		if body_count >= int(scaffold.island_bodies.length)
		{
			status = .Capacity_Missing;
			return;
		}
		scaffold.island_bodies.memory[body_count] = handle;
		body_count += 1;
		set := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		if set.activity.memory[location.index].sleep_candidate != .Candidate
		{
			can_sleep = .Missing;
			status = .Ok;
			return;
		}
		constraints := &set.constraints.memory[location.index];
		for reference_index in 0 ..< constraints.count
		{
			constraint_handle := constraints.span.memory[reference_index].connecting_constraint_handle;
			if constraint_handle.value < 0 || int(constraint_handle.value) >= int(scaffold.constraint_marks.length)
			{
				status = .Capacity_Missing;
				return;
			}
			if scaffold.constraint_marks.memory[constraint_handle.value] != 0
			{
				continue;
			}
			scaffold.constraint_marks.memory[constraint_handle.value] = 1;
			if constraint_count >= int(scaffold.island_constraints.length)
			{
				status = .Capacity_Missing;
				return;
			}
			scaffold.island_constraints.memory[constraint_count] = constraint_handle;
			constraint_count += 1;
			constraint_location, constraint_status := solver_resolve(sleeper.solver, constraint_handle);
			if constraint_status != .Ok
			{
				status = constraint_status;
				return;
			}
			batch := &sleeper.solver.active_set.batches.memory[constraint_location.batch_index];
			type_batch := &batch.type_batches.memory[batch.type_id_to_batch_index[constraint_location.type_id]];
			constraint_reference, read_status := type_batch_read_reference(
				type_batch, sleeper.bodies, int(constraint_location.index_in_type_batch),
			);
			if read_status != .Ok
			{
				status = read_status;
				return;
			}
			for body_index in 0 ..< int(constraint_reference.body_count)
			{
				connected := constraint_reference.body_handles[body_index];
				if connected.value == handle.value
				{
					continue;
				}
				if connected.value < 0 || int(connected.value) >= int(scaffold.body_marks.length)
				{
					status = .Capacity_Missing;
					return;
				}
				if scaffold.body_marks.memory[connected.value] != 0
				{
					continue;
				}
				connected_location, connected_status := bodies_resolve(sleeper.bodies, connected);
				if connected_status != .Ok || connected_location.set_index != BODIES_ACTIVE_SET_INDEX
				{
					status = .Invalid_Argument;
					return;
				}
				if stack_count >= int(scaffold.stack.length)
				{
					status = .Capacity_Missing;
					return;
				}
				scaffold.body_marks.memory[connected.value] = 1;
				scaffold.stack.memory[stack_count] = connected;
				stack_count += 1;
			}
		}
	}
	status = .Ok;
	return;
}

Island_Sleeper_Constraint_Gather_Job :: struct
{
	sleeper:      ^Island_Sleeper,
	scaffold:     ^Island_Scaffold,
	set_index:    int,
	record_start: int,
	constraint_count: int,
	worker_count: int,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

island_sleeper_constraint_gather_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Sleeper_Constraint_Gather_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	for index := worker_index; index < job.constraint_count; index += job.worker_count
	{
		handle := job.scaffold.island_constraints.memory[index];
		status = solver_gather_inactive_constraint(
			job.sleeper.solver, handle, job.set_index,
			&job.sleeper.inactive_constraints.memory[job.record_start + index],
		);
		if status != .Ok
		{
			break;
		}
	}
	job.statuses[worker_index] = status;
}

Island_Sleeper_Body_Gather_Job :: struct
{
	sleeper:     ^Island_Sleeper,
	scaffold:    ^Island_Scaffold,
	target:      ^Body_Set,
	body_count:   int,
	worker_count: int,
	statuses:    ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

island_sleeper_body_gather_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Sleeper_Body_Gather_Job)(dispatcher.unmanaged_context);
	active := &job.sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	status := Physics_Status.Ok;
	for index := worker_index; index < job.body_count; index += job.worker_count
	{
		handle := job.scaffold.island_bodies.memory[index];
		location, resolve_status := bodies_resolve(job.sleeper.bodies, handle);
		if resolve_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			status = .Invalid_Argument;
			break;
		}
		source_index := int(location.index);
		job.target.dynamics_state.memory[index] = active.dynamics_state.memory[source_index];
		job.target.index_to_handle.memory[index] = handle;
		job.target.collidables.memory[index] = active.collidables.memory[source_index];
		job.target.activity.memory[index] = active.activity.memory[source_index];
		job.target.constraints.memory[index] = active.constraints.memory[source_index];
		job.target.constraints.memory[index].count = 0;
	}
	job.statuses[worker_index] = status;
}

island_sleeper_commit_body_migration :: proc "contextless" (
	sleeper: ^Island_Sleeper, handle: Body_Handle, target_set_index, target_index: int,
	broad_phase_migration: ^Broad_Phase_Body_Migration,
)
{
	context = runtime.default_context();
	location := sleeper.bodies.handle_to_location.memory[handle.value];
	active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	target := &sleeper.bodies.sets.memory[target_set_index];
	target.collidables.memory[target_index].broad_phase_index =
	active.collidables.memory[location.index].broad_phase_index;
	active.count -= 1;
	if int(location.index) < active.count
	{
		last := active.count;
		active.dynamics_state.memory[location.index] = active.dynamics_state.memory[last];
		active.index_to_handle.memory[location.index] = active.index_to_handle.memory[last];
		active.collidables.memory[location.index] = active.collidables.memory[last];
		active.activity.memory[location.index] = active.activity.memory[last];
		active.constraints.memory[location.index] = active.constraints.memory[last];
		moved_handle := active.index_to_handle.memory[location.index];
		solver_update_for_body_memory_move_trusted(sleeper.solver, active, last, int(location.index));
		sleeper.bodies.handle_to_location.memory[moved_handle.value] = location;
	}
	active.dynamics_state.memory[active.count] = {};
	active.index_to_handle.memory[active.count] = {};
	active.collidables.memory[active.count] = {};
	active.activity.memory[active.count] = {};
	active.constraints.memory[active.count] = {};
	sleeper.bodies.handle_to_location.memory[handle.value] = {i32(target_set_index), i32(target_index)};
	broad_phase_commit_body_migration(sleeper.broad_phase, broad_phase_migration);
}

island_sleeper_sleep_island :: proc (
	sleeper: ^Island_Sleeper, scaffold: ^Island_Scaffold, body_count, constraint_count: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready || body_count <= 0 ||
	constraint_count < 0 || scaffold == nil || scaffold.state != .Ready
	{
		return .Invalid_Argument;
	}
	if sleeper.inactive_constraint_count > max(int) - constraint_count
	{
		return .Capacity_Missing;
	}
	required_inactive_constraint_count :=
	sleeper.inactive_constraint_count + constraint_count;
	if required_inactive_constraint_count > int(sleeper.inactive_constraints.length)
	{
		target_constraint_capacity := max(
			required_inactive_constraint_count,
			max(int(sleeper.inactive_constraints.length) * 2, 1),
		);
		capacity_status := island_sleeper_ensure_capacity(
			sleeper, max(int(sleeper.bodies.handle_to_location.length), 1),
			target_constraint_capacity,
		);
		if capacity_status != .Ok
		{
			return capacity_status;
		}
	}
	set_index := -1;
	for offset in 0 ..< int(sleeper.bodies.sets.length) - 1
	{
		candidate := 1 + (sleeper.next_set_index - 1 + offset) % (int(sleeper.bodies.sets.length) - 1);
		set := &sleeper.bodies.sets.memory[candidate];
		if set.state == .Unallocated || set.count == 0
		{
			set_index = candidate;
			break;
		}
	}
	if set_index < 0
	{
		old_set_capacity := int(sleeper.bodies.sets.length);
		if old_set_capacity > max(int) / 2
		{
			return .Capacity_Missing;
		}
		set_capacity_status := bodies_ensure_set_capacity(
			sleeper.bodies, max(old_set_capacity * 2, old_set_capacity + 1),
		);
		if set_capacity_status != .Ok
		{
			return set_capacity_status;
		}
		set_index = old_set_capacity;
	}
	total_inactive_body_count := 0;
	for inactive_set_index in 1 ..< int(sleeper.bodies.sets.length)
	{
		set := &sleeper.bodies.sets.memory[inactive_set_index];
		if total_inactive_body_count > max(int) - set.count
		{
			return .Capacity_Missing;
		}
		total_inactive_body_count += set.count;
	}
	if total_inactive_body_count > max(int) - body_count
	{
		return .Capacity_Missing;
	}
	required_inactive_body_count := total_inactive_body_count + body_count;
	if required_inactive_body_count > sleeper.bodies.inactive_arena.capacity
	{
		current_capacity := sleeper.bodies.inactive_arena.capacity;
		if current_capacity > max(int) / 2
		{
			return .Capacity_Missing;
		}
		arena_status := bodies_ensure_inactive_arena_capacity(
			sleeper.bodies,
			max(required_inactive_body_count, max(current_capacity * 2, 1)),
		);
		if arena_status != .Ok
		{
			return arena_status;
		}
	}
	record_start := sleeper.inactive_constraint_count;
	target_capacity_status := bodies_prepare_inactive_set_capacity(
		sleeper.bodies, set_index, body_count,
	);
	if target_capacity_status != .Ok
	{
		return target_capacity_status;
	}
	target := &sleeper.bodies.sets.memory[set_index];
	if target.state != .Allocated || target.count != 0 || body_count > body_set_capacity(target)
	{
		return .Capacity_Missing;
	}
	transaction := Island_Sleep_Transaction{
		set_index=i32(set_index), record_start=i32(record_start),
		body_count=i32(body_count), constraint_count=i32(constraint_count),
	};
	active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if sleeper.broad_phase.static_tree.leaf_count > max(int) - body_count
	{
		return .Capacity_Missing;
	}
	broad_phase_status := broad_phase_ensure_capacity(
		sleeper.broad_phase,
		max(sleeper.broad_phase.active_tree.leaf_count, 1),
		max(sleeper.broad_phase.static_tree.leaf_count + body_count, 1),
		1,
	);
	if broad_phase_status != .Ok
	{
		return broad_phase_status;
	}
	migration_count := 0;
	for body_index in 0 ..< body_count
	{
		handle := scaffold.island_bodies.memory[body_index];
		location, resolve_status := bodies_resolve(sleeper.bodies, handle);
		if resolve_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return .Invalid_Argument;
		}
		migration, migration_status := broad_phase_prepare_body_migration(sleeper.broad_phase, handle, .Static);
		if migration_status != .Ok
		{
			return migration_status;
		}
		sleeper.body_migrations.memory[body_index] = migration;
		if migration.state == .Present
		{
			migration_count += 1;
		}
	}
	migration_capacity_status := broad_phase_validate_body_migration_capacity(
		sleeper.broad_phase, .Static, migration_count,
	);
	if migration_capacity_status != .Ok
	{
		return migration_capacity_status;
	}
	inactive_pair_count := 0;
	for constraint_index in 0 ..< constraint_count
	{
		handle := scaffold.island_constraints.memory[constraint_index];
		_, resolve_status := solver_resolve(sleeper.solver, handle);
		if resolve_status != .Ok
		{
			return resolve_status;
		}
		if handle.value >= 0 && int(handle.value) < int(sleeper.pair_cache.constraint_handle_to_pair.length)
		{
			pair := sleeper.pair_cache.constraint_handle_to_pair.memory[handle.value].pair;
			if pair_cache_index_of(sleeper.pair_cache, pair) >= 0
			{
				inactive_pair_count += 1;
			}
		}
	}
	if sleeper.pair_cache.inactive_count > max(int) - inactive_pair_count
	{
		return .Capacity_Missing;
	}
	required_inactive_pair_count := sleeper.pair_cache.inactive_count + inactive_pair_count;
	if required_inactive_pair_count > int(sleeper.pair_cache.inactive_entries.length)
	{
		pair_capacity_status := pair_cache_ensure_capacity(
			sleeper.pair_cache,
			max(int(sleeper.pair_cache.mapping.keys.length), 1),
			max(int(sleeper.pair_cache.constraint_handle_to_pair.length), 1),
			max(
				required_inactive_pair_count,
				max(int(sleeper.pair_cache.inactive_entries.length) * 2, 1),
			),
		);
		if pair_capacity_status != .Ok
		{
			return pair_capacity_status;
		}
	}
	constraint_worker_count := 1;
	if dispatcher != nil
	{
		constraint_worker_count = min(dispatcher.worker_count, constraint_count);
	}
	if constraint_count > 0 && dispatcher != nil && constraint_worker_count > 1
	{
		job := Island_Sleeper_Constraint_Gather_Job{
			sleeper, scaffold, set_index, record_start, constraint_count,
			constraint_worker_count, &sleeper.worker_statuses,
		};
		dispatch_status := dispatcher.dispatch(
			dispatcher, island_sleeper_constraint_gather_worker, constraint_worker_count, &job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< constraint_worker_count
		{
			if sleeper.worker_statuses[worker_index] != .Ok
			{
				return sleeper.worker_statuses[worker_index];
			}
		}
	}
	else
	{
		for constraint_index in 0 ..< constraint_count
		{
			handle := scaffold.island_constraints.memory[constraint_index];
			status := solver_gather_inactive_constraint(
				sleeper.solver, handle, set_index,
				&sleeper.inactive_constraints.memory[record_start + constraint_index],
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	body_worker_count := 1;
	if dispatcher != nil
	{
		body_worker_count = min(dispatcher.worker_count, body_count);
	}
	if body_count > 0 && dispatcher != nil && body_worker_count > 1
	{
		job := Island_Sleeper_Body_Gather_Job{
			sleeper, scaffold, target, body_count, body_worker_count, &sleeper.worker_statuses,
		};
		dispatch_status := dispatcher.dispatch(
			dispatcher, island_sleeper_body_gather_worker, body_worker_count, &job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< body_worker_count
		{
			if sleeper.worker_statuses[worker_index] != .Ok
			{
				return sleeper.worker_statuses[worker_index];
			}
		}
	}
	else
	{
		for body_index in 0 ..< body_count
		{
			handle := scaffold.island_bodies.memory[body_index];
			location, resolve_status := bodies_resolve(sleeper.bodies, handle);
			if resolve_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
			{
				return .Invalid_Argument;
			}
			source_index := int(location.index);
			target.dynamics_state.memory[body_index] = active.dynamics_state.memory[source_index];
			target.index_to_handle.memory[body_index] = handle;
			target.collidables.memory[body_index] = active.collidables.memory[source_index];
			target.activity.memory[body_index] = active.activity.memory[source_index];
			target.constraints.memory[body_index] = active.constraints.memory[source_index];
			target.constraints.memory[body_index].count = 0;
		}
	}
	for constraint_index in 0 ..< int(transaction.constraint_count)
	{
		record_index := record_start + constraint_index;
		record := &sleeper.inactive_constraints.memory[record_index];
		sleeper.constraint_order.memory[constraint_index] = {
			key=solver_remove_order_key(&record.solver_remove),
			record_index=i32(record_index),
		};
	}
	sort_status := solver_sort_remove_order_descending(
		sleeper.constraint_order, sleeper.constraint_sort_scratch, int(transaction.constraint_count),
	);
	if sort_status != .Ok
	{
		return sort_status;
	}
	for committed_index in 0 ..< int(transaction.constraint_count)
	{
		record_index := sleeper.constraint_order.memory[committed_index].record_index;
		record := &sleeper.inactive_constraints.memory[record_index];
		solver_commit_remove_transaction(sleeper.solver, &record.solver_remove, .Missing);
		sleeper.solver.handle_to_constraint.memory[record.handle.value] = {
			set_index=i32(set_index), batch_index=-1, type_id=record.type_id,
			index_in_type_batch=i32(record_index),
		};
		pair_cache_commit_move_constraint_to_inactive(sleeper.pair_cache, record.handle, set_index);
	}
	sleeper.inactive_constraint_count = int(transaction.record_start + transaction.constraint_count);
	target.count = int(transaction.body_count);
	for body_index in 0 ..< int(transaction.body_count)
	{
		handle := scaffold.island_bodies.memory[body_index];
		island_sleeper_commit_body_migration(
			sleeper, handle, set_index, body_index, &sleeper.body_migrations.memory[body_index],
		);
	}
	// sleeping can follow integration after the last trigger sample. queue the
	// committed handles once per island, outside the unchanged migration loop,
	// so final poses are sampled without waking bodies or touching disabled lanes
	if sleeper.bodies.triggers != nil
	{
		for body_index in 0 ..< int(transaction.body_count)
		{
			trigger_changed(sleeper.bodies.triggers, .Body,
				scaffold.island_bodies.memory[body_index].value, .Modified);
		}
	}
	sleeper.next_set_index = 1 + set_index % (int(sleeper.bodies.sets.length) - 1);
	return .Ok;
}

Island_Sleeper_Traversal_Job :: struct
{
	sleeper:      ^Island_Sleeper,
	seed_count:    int,
	worker_count: int,
}

island_sleeper_traversal_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Sleeper_Traversal_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	for seed_index := worker_index; seed_index < job.seed_count; seed_index += job.worker_count
	{
		scaffold := &job.sleeper.worker_scaffolds[seed_index];
		status = island_scaffold_clear(scaffold);
		if status != .Ok
		{
			job.sleeper.worker_statuses[seed_index] = status;
			continue;
		}
		body_count, constraint_count, can_sleep, build_status := island_sleeper_build_island(
			job.sleeper, scaffold, job.sleeper.traversal_seeds[seed_index],
		);
		job.sleeper.traversal_body_counts[seed_index] = body_count;
		job.sleeper.traversal_constraint_counts[seed_index] = constraint_count;
		job.sleeper.traversal_can_sleep[seed_index] = can_sleep;
		job.sleeper.worker_statuses[seed_index] = build_status;
	}
}

island_sleeper_update :: proc (
	sleeper: ^Island_Sleeper, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready
	{
		return .Disposed;
	}
	fractions := [3]f32{
		sleeper.tested_fraction_per_frame,
		sleeper.target_slept_fraction,
		sleeper.target_traversed_fraction,
	};
	for fraction in fractions
	{
		if math.is_nan(fraction) || math.is_inf(fraction, 0) ||
		fraction < 0 || fraction > 1
		{
			return .Invalid_Description;
		}
	}
	active_count := sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].count;
	sleep_dispatcher := dispatcher;
	if sleeper.tested_fraction_per_frame == 0 ||
	f32(active_count) < 2 / sleeper.tested_fraction_per_frame
	{
		sleep_dispatcher = nil;
	}
	body_capacity := max(int(sleeper.bodies.handle_to_location.length), 1);
	constraint_capacity := max(int(sleeper.solver.handle_to_constraint.length), 1);
	capacity_status := island_sleeper_ensure_capacity(
		sleeper, body_capacity, constraint_capacity,
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	candidate_count := 0;
	status := Physics_Status.Ok;
	use_filtered_candidates :=
	sleep_dispatcher != nil && sleep_dispatcher.worker_count > 1;
	if use_filtered_candidates
	{
		candidate_count, status = island_sleeper_update_filtered_candidates(
			sleeper,
		);
		if status != .Ok
		{
			return status;
		}
		if candidate_count == 0
		{
			return .Ok;
		}
	}
	else
	{
		candidate_count, status = island_sleeper_update_candidates(
			sleeper, sleep_dispatcher,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status = island_scaffold_clear(&sleeper.scaffold);
	if status != .Ok
	{
		return status;
	}
	target_slept_count := int(
		math.ceil(f32(active_count) * sleeper.target_slept_fraction),
	);
	target_traversed_count := int(
		math.ceil(f32(active_count) * sleeper.target_traversed_fraction),
	);
	candidate_index := 0;
	slept_count := 0;
	traversed_count := 0;
	for candidate_index < candidate_count &&
	slept_count < target_slept_count &&
	traversed_count < target_traversed_count
	{
		seed_capacity := 1;
		if sleep_dispatcher != nil
		{
			seed_capacity = min(sleep_dispatcher.worker_count, sleeper.worker_count);
		}
		seed_count := 0;
		for candidate_index < candidate_count && seed_count < seed_capacity
		{
			handle := sleeper.scaffold.stack.memory[candidate_index];
			candidate_index += 1;
			if handle.value < 0
			{
				continue;
			}
			if int(handle.value) >= int(sleeper.scaffold.body_marks.length)
			{
				return .Capacity_Missing;
			}
			if sleeper.scaffold.body_marks.memory[handle.value] != 0
			{
				continue;
			}
			location, resolve_status := bodies_resolve(sleeper.bodies, handle);
			if resolve_status != .Ok || location.set_index != BODIES_ACTIVE_SET_INDEX
			{
				continue;
			}
			sleeper.traversal_seeds[seed_count] = handle;
			seed_count += 1;
		}
		if seed_count == 0
		{
			break;
		}
		traversal_worker_count := 1;
		if sleep_dispatcher != nil
		{
			traversal_worker_count = min(sleep_dispatcher.worker_count, max(seed_count, 2));
		}
		if sleep_dispatcher != nil && traversal_worker_count > 1
		{
			job := Island_Sleeper_Traversal_Job{sleeper, seed_count, traversal_worker_count};
			dispatch_status := sleep_dispatcher.dispatch(
				sleep_dispatcher, island_sleeper_traversal_worker,
				traversal_worker_count, &job,
			);
			if dispatch_status != .Ok
			{
				return .Invalid_Argument;
			}
		}
		else
		{
			for seed_index in 0 ..< seed_count
			{
				scaffold := &sleeper.worker_scaffolds[seed_index];
				status = island_scaffold_clear(scaffold);
				if status != .Ok
				{
					return status;
				}
				body_count, constraint_count, can_sleep, build_status := island_sleeper_build_island(
					sleeper, scaffold, sleeper.traversal_seeds[seed_index],
				);
				sleeper.traversal_body_counts[seed_index] = body_count;
				sleeper.traversal_constraint_counts[seed_index] = constraint_count;
				sleeper.traversal_can_sleep[seed_index] = can_sleep;
				sleeper.worker_statuses[seed_index] = build_status;
			}
		}
		for seed_index in 0 ..< seed_count
		{
			if sleeper.worker_statuses[seed_index] != .Ok
			{
				return sleeper.worker_statuses[seed_index];
			}
			scaffold := &sleeper.worker_scaffolds[seed_index];
			body_count := sleeper.traversal_body_counts[seed_index];
			if body_count == 0
			{
				continue;
			}
			first_handle := scaffold.island_bodies.memory[0];
			if sleeper.scaffold.body_marks.memory[first_handle.value] != 0
			{
				continue;
			}
			traversed_count += body_count;
			for body_index in 0 ..< body_count
			{
				handle := scaffold.island_bodies.memory[body_index];
				sleeper.scaffold.body_marks.memory[handle.value] = 1;
			}
			if sleeper.traversal_can_sleep[seed_index] == .Present &&
			slept_count < target_slept_count
			{
				sleep_status := island_sleeper_sleep_island(
					sleeper, scaffold, body_count,
					sleeper.traversal_constraint_counts[seed_index],
					sleep_dispatcher,
				);
				if sleep_status != .Ok
				{
					return sleep_status;
				}
				slept_count += body_count;
			}
		}
	}
	return .Ok;
}

island_sleeper_clear :: proc (sleeper: ^Island_Sleeper) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready
	{
		return .Disposed;
	}
	status := island_scaffold_clear(&sleeper.scaffold);
	if status != .Ok
	{
		return status;
	}
	for worker_index in 0 ..< sleeper.worker_count
	{
		status = island_scaffold_clear(&sleeper.worker_scaffolds[worker_index]);
		if status != .Ok
		{
			return status;
		}
		sleeper.traversal_seeds[worker_index] = {};
		sleeper.traversal_body_counts[worker_index] = 0;
		sleeper.traversal_constraint_counts[worker_index] = 0;
		sleeper.traversal_can_sleep[worker_index] = .Missing;
		sleeper.worker_statuses[worker_index] = .Ok;
	}
	_ = util.buffer_clear(
		sleeper.inactive_constraints, 0, sleeper.inactive_constraint_count,
	);
	sleeper.inactive_constraint_count = 0;
	sleeper.claim_generation = 0;
	sleeper.next_set_index = 1;
	sleeper.schedule_offset = 0;
	return .Ok;
}

island_sleeper_dispose :: proc (sleeper: ^Island_Sleeper) -> Physics_Status
{
	if sleeper == nil || sleeper.state != .Ready || sleeper.pool == nil
	{
		return .Disposed;
	}
	pool := sleeper.pool;
	physics_return_buffer(pool, &sleeper.type_claims);
	physics_return_buffer(pool, &sleeper.body_batch_claims);
	physics_return_buffer(pool, &sleeper.body_claims);
	physics_return_buffer(pool, &sleeper.body_migrations);
	physics_return_buffer(pool, &sleeper.constraint_sort_scratch);
	physics_return_buffer(pool, &sleeper.constraint_order);
	physics_return_buffer(pool, &sleeper.inactive_constraints);
	for worker_index in 0 ..< sleeper.worker_count
	{
		_ = island_scaffold_dispose(&sleeper.worker_scaffolds[worker_index]);
	}
	_ = island_scaffold_dispose(&sleeper.scaffold);
	sleeper^ = {state=.Disposed};
	return .Ok;
}

island_sleeper_update_filtered_candidates :: #force_no_inline proc (
	sleeper: ^Island_Sleeper,
) -> (int, Physics_Status)
{
	active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if active.count == 0
	{
		return 0, .Ok;
	}
	candidate_count := max(
		1, int(f32(active.count) * sleeper.tested_fraction_per_frame),
	);
	spacing := active.count / candidate_count;
	if sleeper.schedule_offset > active.count
	{
		sleeper.schedule_offset = 0;
	}
	filtered_count: int = 0;
	for candidate_index in 0 ..< candidate_count
	{
		index: int = (sleeper.schedule_offset + candidate_index * spacing) % active.count;
		if active.activity.memory[index].sleep_candidate != .Candidate
		{
			continue;
		}
		handle: Body_Handle = active.index_to_handle.memory[index];
		if handle.value < 0
		{
			continue;
		}
		sleeper.scaffold.stack.memory[filtered_count] = handle;
		filtered_count += 1;
	}
	sleeper.schedule_offset += 1;
	return filtered_count, .Ok;
}
