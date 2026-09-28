package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"

solver_integration_flag_word_capacity :: proc "contextless" (
	constraint_capacity, batch_capacity: int,
) -> (int, Physics_Status)
{
	if constraint_capacity <= 0 || batch_capacity <= 0
	{
		return 0, .Invalid_Argument;
	}
	constraint_word_count :=
		(i64(constraint_capacity) + i64(util.INDEX_SET_MASK)) >> util.INDEX_SET_SHIFT;
	type_batch_count := i64(batch_capacity) * i64(CONSTRAINT_TYPE_ID_CAPACITY);
	word_count := i64(4) * (constraint_word_count + type_batch_count);
	if word_count <= 0 || word_count > i64(max(int))
	{
		return 0, .Capacity_Missing;
	}
	return int(word_count), .Ok;
}
solver_integration_seen_word_capacity :: proc "contextless" (
	body_capacity: int,
) -> (int, Physics_Status)
{
	if body_capacity <= 0
	{
		return 0, .Invalid_Argument;
	}
	word_count := max(util.index_set_bundle_capacity(body_capacity), 1);
	return word_count, .Ok;
}
solver_initialize :: proc (
	solver: ^Solver, bodies: ^Bodies, constraint_capacity, batch_capacity,
	fallback_batch_threshold, initial_type_batch_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if solver == nil || bodies == nil || bodies.state != .Allocated || pool == nil || constraint_capacity <= 0 ||
		batch_capacity <= fallback_batch_threshold || fallback_batch_threshold <= 0 ||
		initial_type_batch_capacity <= 0 || solver.state != .Uninitialized || bodies.solver != nil
	{
		return .Invalid_Argument;
	}
	locations, location_status := util.buffer_pool_take_at_least(pool, Constraint_Location, constraint_capacity);
	if location_status != .Ok
	{
		return physics_memory_status(location_status);
	}
	for index in 0 ..< locations.length
	{
		locations.memory[index] = constraint_location_missing();
	}
	handle_pool: util.Id_Pool;
	id_status := util.id_pool_initialize(&handle_pool, constraint_capacity, pool);
	if id_status != .Ok
	{
		physics_return_buffer(pool, &locations);
		return physics_memory_status(id_status);
	}
	batches, batches_status := util.buffer_pool_take_at_least(pool, Constraint_Batch, batch_capacity);
	if batches_status != .Ok
	{
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return physics_memory_status(batches_status);
	}
	_ = util.buffer_clear(batches, 0, int(batches.length));
	for batch_index in 0 ..< batches.length
	{
		batch_status := constraint_batch_initialize(
			&batches.memory[batch_index],
			int(bodies.handle_to_location.length),
			pool
		);
		if batch_status != .Ok
		{
			for release_index in 0 ..< batch_index
			{
				constraint_batch_dispose(&batches.memory[release_index], pool);
			}
			physics_return_buffer(pool, &batches);
			_ = util.id_pool_dispose(&handle_pool, pool);
			physics_return_buffer(pool, &locations);
			return batch_status;
		}
	}
	registry: Constraint_Type_Registry;
	registry_status := constraint_type_registry_initialize(&registry);
	if registry_status != .Ok
	{
		for batch_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[batch_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return registry_status;
	}
	for type_id in 0 ..< CONSTRAINT_TYPE_ID_CAPACITY
	{
		if type_id > CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID && type_id < CONTACT_2_NONCONVEX_TYPE_ID ||
			type_id > CONTACT_4_NONCONVEX_TYPE_ID
		{
			continue;
		}
		record := &registry.records[type_id];
		if record.registration != .Present
		{
			continue;
		}
		for batch_index in 0 ..< batches.length
		{
			_, type_batch_status := constraint_batch_get_or_create_type_batch(
				&batches.memory[batch_index], record, initial_type_batch_capacity, pool,
			);
			if type_batch_status != .Ok
			{
				_ = constraint_type_registry_dispose(&registry);
				for release_index in 0 ..< batches.length
				{
					constraint_batch_dispose(&batches.memory[release_index], pool);
				}
				physics_return_buffer(pool, &batches);
				_ = util.id_pool_dispose(&handle_pool, pool);
				physics_return_buffer(pool, &locations);
				return type_batch_status;
			}
		}
	}
	integration_flag_word_count, integration_flag_capacity_status :=
		solver_integration_flag_word_capacity(constraint_capacity, batch_capacity);
	if integration_flag_capacity_status != .Ok
	{
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return integration_flag_capacity_status;
	}
	integration_flags, integration_flags_status := util.buffer_pool_take_at_least(
		pool, u64, integration_flag_word_count,
	);
	if integration_flags_status != .Ok
	{
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return physics_memory_status(integration_flags_status);
	}
	_ = util.buffer_clear(integration_flags, 0, int(integration_flags.length));
	integration_seen_word_count, integration_seen_capacity_status :=
		solver_integration_seen_word_capacity(int(bodies.handle_to_location.length));
	if integration_seen_capacity_status != .Ok
	{
		physics_return_buffer(pool, &integration_flags);
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return integration_seen_capacity_status;
	}
	integration_body_seen, integration_seen_status := util.buffer_pool_take_at_least(
		pool, u64, integration_seen_word_count,
	);
	if integration_seen_status != .Ok
	{
		physics_return_buffer(pool, &integration_flags);
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return physics_memory_status(integration_seen_status);
	}
	_ = util.buffer_clear(integration_body_seen, 0, int(integration_body_seen.length));
	maximum_work_block_count_64 :=
		(i64(constraint_capacity) + i64(util.PRODUCTION_LANE_COUNT) - 1) / i64(util.PRODUCTION_LANE_COUNT) +
		i64(batch_capacity) * i64(CONSTRAINT_TYPE_ID_CAPACITY) * i64(util.PRODUCTION_LANE_COUNT - 1);
	maximum_work_block_count := min(i64(constraint_capacity), maximum_work_block_count_64);
	if maximum_work_block_count <= 0 || maximum_work_block_count > i64(max(i32))
	{
		physics_return_buffer(pool, &integration_body_seen);
		physics_return_buffer(pool, &integration_flags);
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return .Capacity_Missing;
	}
	work_blocks, work_status := util.buffer_pool_take_at_least(
		pool, Solver_Work_Block, int(maximum_work_block_count),
	);
	if work_status != .Ok
	{
		physics_return_buffer(pool, &integration_body_seen);
		physics_return_buffer(pool, &integration_flags);
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return physics_memory_status(work_status);
	}
	_ = util.buffer_clear(work_blocks, 0, int(work_blocks.length));
	compression_candidates, compression_status := util.buffer_pool_take_at_least(
		pool, Compression_Candidate, constraint_capacity,
	);
	if compression_status != .Ok
	{
		physics_return_buffer(pool, &work_blocks);
		physics_return_buffer(pool, &integration_body_seen);
		physics_return_buffer(pool, &integration_flags);
		_ = constraint_type_registry_dispose(&registry);
		for release_index in 0 ..< batches.length
		{
			constraint_batch_dispose(&batches.memory[release_index], pool);
		}
		physics_return_buffer(pool, &batches);
		_ = util.id_pool_dispose(&handle_pool, pool);
		physics_return_buffer(pool, &locations);
		return physics_memory_status(compression_status);
	}
	fallback_batch_index := fallback_batch_threshold;
	solver^ = {
		registry=registry,
		active_set={batches=batches, batch_count=1, state=.Allocated},
		handle_to_constraint=locations,
		integration_flags=integration_flags,
		integration_body_seen=integration_body_seen,
		work_blocks=work_blocks,
		compression_candidates=compression_candidates,
		handle_pool=handle_pool,
		bodies=bodies,
		pool=pool,
		sequential_batch={batch_index=i32(fallback_batch_index), state=.Present},
		fallback_batch_threshold=i32(fallback_batch_threshold),
		fallback_batch_index=i32(fallback_batch_index),
		initial_type_batch_capacity=i32(initial_type_batch_capacity),
		compression_batch_index=i32(fallback_batch_index),
		state=.Ready,
	};
	solver.sequential_batch.dynamic_body_constraint_counts =
		&solver.active_set.batches.memory[fallback_batch_index].body_reference_counts;
	bodies.solver = solver;
	return .Ok;
}
solver_bind_integrator :: proc "contextless" (
	solver: ^Solver, integrator: ^Pose_Integrator,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || integrator == nil || integrator.state != .Ready ||
		integrator.bodies != solver.bodies
	{
		return .Invalid_Argument;
	}
	solver.integrator = integrator;
	return .Ok;
}
solver_register_constraint_type :: proc "contextless" (
	solver: ^Solver, record: Constraint_Type_Record,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || solver.active_set.constraint_count != 0 ||
		record.type_id < FIRST_CALLER_CONSTRAINT_TYPE_ID
	{
		return .Invalid_Argument;
	}
	return constraint_type_registry_register(&solver.registry, record);
}
solver_resolve :: proc "contextless" (
	solver: ^Solver,
	handle: Constraint_Handle
) -> (Constraint_Location, Physics_Status)
{
	if solver == nil ||
		solver.state != .Ready ||
		handle.value < 0 ||
		int(handle.value) >= int(solver.handle_to_constraint.length)
	{
		return {}, .Not_Found;
	}
	location := solver.handle_to_constraint.memory[handle.value];
	if location.set_index != 0 ||
		location.batch_index < 0 ||
		int(location.batch_index) >= int(solver.active_set.batches.length) ||
		location.type_id < 0 || location.type_id >= CONSTRAINT_TYPE_ID_CAPACITY
	{
		return {}, .Not_Found;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	if type_batch_index < 0
	{
		return {}, .Not_Found;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	if location.index_in_type_batch < 0 || int(location.index_in_type_batch) >= int(type_batch.count) ||
		type_batch.index_to_handle.memory[location.index_in_type_batch] != handle.value
	{
		return {}, .Not_Found;
	}
	return location, .Ok;
}
solver_reference_state :: proc "contextless" (solver: ^Solver, handle: Constraint_Handle) -> Reference_State
{
	_, status := solver_resolve(solver, handle);
	if status == .Ok
	{
		return .Present;
	}
	if solver != nil &&
		solver.state == .Ready &&
		handle.value >= 0 &&
		int(handle.value) < int(solver.handle_to_constraint.length) &&
		solver.handle_to_constraint.memory[handle.value].set_index > 0
	{
		return .Present;
	}
	return .Missing;
}
solver_ensure_integration_flag_capacity :: proc (
	solver: ^Solver, constraint_capacity: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || constraint_capacity <= 0
	{
		return .Invalid_Argument;
	}
	word_count, capacity_status := solver_integration_flag_word_capacity(
		constraint_capacity, int(solver.active_set.batches.length),
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	return physics_ensure_buffer_capacity(
		solver.pool, &solver.integration_flags, word_count, 0,
	);
}
solver_ensure_integration_seen_capacity :: proc (
	solver: ^Solver, body_capacity: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || body_capacity <= 0
	{
		return .Invalid_Argument;
	}
	word_count, capacity_status := solver_integration_seen_word_capacity(body_capacity);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	return physics_ensure_buffer_capacity(
		solver.pool, &solver.integration_body_seen, word_count, 0,
	);
}
solver_ensure_body_capacity :: proc (solver: ^Solver, body_capacity: int) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || body_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := solver_ensure_integration_seen_capacity(solver, body_capacity);
	if status != .Ok
	{
		return status;
	}
	for batch_index in 0 ..< solver.active_set.batches.length
	{
		status = constraint_batch_ensure_body_capacity(
			&solver.active_set.batches.memory[batch_index], body_capacity, solver.pool,
		);
		if status != .Ok
		{
			return status;
		}
	}
	solver.sequential_batch.dynamic_body_constraint_counts =
		&solver.active_set.batches.memory[solver.fallback_batch_index].body_reference_counts;
	return .Ok;
}
solver_ensure_handle_capacity :: proc (solver: ^Solver, capacity: int) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || capacity <= 0
	{
		return .Invalid_Argument;
	}
	flag_status := solver_ensure_integration_flag_capacity(solver, capacity);
	if flag_status != .Ok
	{
		return flag_status;
	}
	compression_status := physics_ensure_buffer_capacity(
		solver.pool, &solver.compression_candidates, capacity, 0,
	);
	if compression_status != .Ok
	{
		return compression_status;
	}
	if capacity <= int(solver.handle_to_constraint.length)
	{
		return .Ok;
	}
	new_locations, location_status := util.buffer_pool_take_at_least(
		solver.pool, Constraint_Location, capacity,
	);
	if location_status != .Ok
	{
		return physics_memory_status(location_status);
	}
	new_ids, ids_status := util.buffer_pool_take_at_least(solver.pool, i32, capacity);
	if ids_status != .Ok
	{
		physics_return_buffer(solver.pool, &new_locations);
		return physics_memory_status(ids_status);
	}
	for index in 0 ..< new_locations.length
	{
		new_locations.memory[index] = constraint_location_missing();
	}
	_ = util.buffer_copy(
		util.buffer_view(solver.handle_to_constraint), 0,
		util.buffer_view(new_locations), 0,
		int(solver.handle_to_constraint.length),
	);
	_ = util.buffer_copy(
		util.buffer_view(solver.handle_pool.available_ids), 0,
		util.buffer_view(new_ids), 0,
		solver.handle_pool.available_id_count,
	);
	physics_return_buffer(solver.pool, &solver.handle_pool.available_ids);
	physics_return_buffer(solver.pool, &solver.handle_to_constraint);
	solver.handle_to_constraint = new_locations;
	solver.handle_pool.available_ids = new_ids;
	return .Ok;
}
solver_ensure_type_capacity :: proc (
	solver: ^Solver, type_id, capacity_per_batch: int,
) -> Physics_Status
{
	if solver == nil ||
		solver.state != .Ready ||
		type_id < 0 ||
		type_id >= CONSTRAINT_TYPE_ID_CAPACITY ||
		capacity_per_batch <= 0
	{
		return .Invalid_Argument;
	}
	type_record, lookup_status := constraint_type_registry_lookup(&solver.registry, i32(type_id));
	if lookup_status != .Ok
	{
		return .Not_Found;
	}
	for batch_index in 0 ..< solver.active_set.batches.length
	{
		type_batch, status := constraint_batch_get_or_create_type_batch(
			&solver.active_set.batches.memory[batch_index],
			type_record,
			capacity_per_batch,
			solver.pool
		);
		if status != .Ok
		{
			return status;
		}
		status = type_batch_ensure_capacity(type_batch, capacity_per_batch, solver.pool);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}
solver_ensure_capacity :: proc (
	solver: ^Solver, body_capacity, constraint_capacity,
	capacity_per_type_batch: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || body_capacity <= 0 ||
		constraint_capacity <= 0 || capacity_per_type_batch <= 0
	{
		return .Invalid_Argument;
	}
	status := solver_ensure_body_capacity(solver, body_capacity);
	if status != .Ok
	{
		return status;
	}
	status = solver_ensure_handle_capacity(solver, constraint_capacity);
	if status != .Ok
	{
		return status;
	}
	for type_id in 0 ..< CONSTRAINT_TYPE_ID_CAPACITY
	{
		if solver.registry.records[type_id].registration != .Present
		{
			continue;
		}
		status = solver_ensure_type_capacity(
			solver, type_id, capacity_per_type_batch,
		);
		if status != .Ok
		{
			return status;
		}
	}
	maximum_work_block_count_64 :=
		(i64(constraint_capacity) + i64(util.PRODUCTION_LANE_COUNT) - 1) /
		i64(util.PRODUCTION_LANE_COUNT) +
		i64(solver.active_set.batches.length) *
		i64(CONSTRAINT_TYPE_ID_CAPACITY) *
		i64(util.PRODUCTION_LANE_COUNT - 1);
	maximum_work_block_count := min(
		i64(constraint_capacity), maximum_work_block_count_64,
	);
	if maximum_work_block_count <= 0 ||
		maximum_work_block_count > i64(max(i32))
	{
		return .Capacity_Missing;
	}
	status = physics_ensure_buffer_capacity(
		solver.pool, &solver.work_blocks,
		int(maximum_work_block_count), 0,
	);
	if status != .Ok
	{
		return status;
	}
	solver.initial_type_batch_capacity = max(
		solver.initial_type_batch_capacity, i32(capacity_per_type_batch),
	);
	return .Ok;
}
solver_resize :: proc (
	solver: ^Solver, body_capacity, constraint_capacity,
	capacity_per_type_batch: int,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || body_capacity <= 0 ||
		constraint_capacity <= 0 || capacity_per_type_batch <= 0
	{
		return .Invalid_Argument;
	}
	status := solver_ensure_capacity(
		solver, body_capacity, constraint_capacity,
		capacity_per_type_batch,
	);
	if status != .Ok
	{
		return status;
	}
	required_body_capacity := max(
		body_capacity, int(solver.bodies.handle_pool.next_index),
	);
	for batch_index in 0 ..< solver.active_set.batches.length
	{
		batch := &solver.active_set.batches.memory[batch_index];
		copy_count := min(
			required_body_capacity, int(batch.body_reference_counts.length),
		);
		status = physics_resize_buffer_capacity(
			solver.pool, &batch.body_reference_counts,
			required_body_capacity, copy_count,
		);
		if status != .Ok
		{
			return status;
		}
		if copy_count < int(batch.body_reference_counts.length)
		{
			_ = util.buffer_clear(
				batch.body_reference_counts, copy_count,
				int(batch.body_reference_counts.length) - copy_count,
			);
		}
		for type_batch_index in 0 ..< batch.type_batch_count
		{
			status = type_batch_resize(
				&batch.type_batches.memory[type_batch_index],
				capacity_per_type_batch, solver.pool,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	required_constraint_capacity := max(
		constraint_capacity, int(solver.handle_pool.next_index),
	);
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.handle_to_constraint,
		required_constraint_capacity, int(solver.handle_pool.next_index),
	);
	if status != .Ok
	{
		return status;
	}
	for index in solver.handle_pool.next_index ..< solver.handle_to_constraint.length
	{
		solver.handle_to_constraint.memory[index] = constraint_location_missing();
	}
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.handle_pool.available_ids,
		max(
		required_constraint_capacity,
		solver.handle_pool.available_id_count,
	),
		solver.handle_pool.available_id_count,
	);
	if status != .Ok
	{
		return status;
	}
	integration_seen_word_count, integration_seen_capacity_status :=
		solver_integration_seen_word_capacity(required_body_capacity);
	if integration_seen_capacity_status != .Ok
	{
		return integration_seen_capacity_status;
	}
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.integration_body_seen,
		integration_seen_word_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	integration_flag_word_count, integration_flag_capacity_status :=
		solver_integration_flag_word_capacity(
		required_constraint_capacity, int(solver.active_set.batches.length),
	);
	if integration_flag_capacity_status != .Ok
	{
		return integration_flag_capacity_status;
	}
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.integration_flags,
		integration_flag_word_count, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.compression_candidates,
		required_constraint_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	maximum_work_block_count_64 :=
		(i64(required_constraint_capacity) +
		i64(util.PRODUCTION_LANE_COUNT) - 1) /
		i64(util.PRODUCTION_LANE_COUNT) +
		i64(solver.active_set.batches.length) *
		i64(CONSTRAINT_TYPE_ID_CAPACITY) *
		i64(util.PRODUCTION_LANE_COUNT - 1);
	maximum_work_block_count := min(
		i64(required_constraint_capacity), maximum_work_block_count_64,
	);
	status = physics_resize_buffer_capacity(
		solver.pool, &solver.work_blocks,
		int(maximum_work_block_count), 0,
	);
	if status != .Ok
	{
		return status;
	}
	solver.sequential_batch.dynamic_body_constraint_counts =
		&solver.active_set.batches.memory[
		solver.fallback_batch_index
	].body_reference_counts;
	solver.initial_type_batch_capacity = i32(capacity_per_type_batch);
	return .Ok;
}
solver_compression_budget_count :: proc "contextless" (
	constraint_count, divisor: int,
) -> int
{
	if constraint_count <= 0 || divisor <= 0 || divisor & 1 != 0
	{
		return 0;
	}
	quotient := constraint_count / divisor;
	remainder := constraint_count % divisor;
	midpoint := divisor / 2;
	if remainder > midpoint || remainder == midpoint && quotient & 1 != 0
	{
		quotient += 1;
	}
	return max(quotient, 1);
}
solver_compression_discovery_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Solver_Compression_Discovery_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	source_batch := &job.solver.active_set.batches.memory[job.source_batch_index];
	source_ordinal_start := 0;
	for type_offset in 0 ..< job.type_batch_count
	{
		type_batch := &source_batch.type_batches.memory[
			job.first_type_batch_index + type_offset
		];
		type_batch_count := int(type_batch.count);
		first_record := (
			worker_index - source_ordinal_start % job.worker_count +
			job.worker_count
		) % job.worker_count;
		for record_index := first_record;
			record_index < type_batch_count;
			record_index += job.worker_count
		{
			source_ordinal := source_ordinal_start + record_index;
			candidate := &job.solver.compression_candidates.memory[source_ordinal];
			candidate^ = {
				handle=constraint_handle_invalid(),
				target_batch=-1,
			};
			if type_batch.index_to_handle.memory[record_index] < 0
			{
				continue;
			}
			reference, reference_status := type_batch_read_reference(
				type_batch, job.solver.bodies, record_index,
			);
			if reference_status != .Ok
			{
				status = reference_status;
				break;
			}
			for target_batch_index := job.source_batch_index - 1;
				target_batch_index >= 0;
				target_batch_index -= 1
			{
				target_batch := &job.solver.active_set.batches.memory[target_batch_index];
				if constraint_batch_body_fit(target_batch, &reference) == .Missing
				{
					continue;
				}
				candidate^ = {
					handle=reference.handle,
					target_batch=i32(target_batch_index),
				};
				break;
			}
		}
		if status != .Ok
		{
			break;
		}
		source_ordinal_start += type_batch_count;
	}
	job.statuses[worker_index] = status;
}
solver_compress :: proc (
	solver: ^Solver, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready
	{
		return .Disposed;
	}
	if dispatcher != nil && (
		dispatcher.dispatch == nil || dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	)
	{
		return .Invalid_Argument;
	}
	constraint_count := int(solver.active_set.constraint_count);
	if constraint_count == 0
	{
		return .Ok;
	}
	if constraint_count > int(solver.compression_candidates.length)
	{
		return .Capacity_Missing;
	}
	batch_count := int(solver.active_set.batch_count);
	if batch_count <= 0
	{
		return .Ok;
	}
	if solver.compression_batch_index < 0 ||
		int(solver.compression_batch_index) >= batch_count
	{
		solver.compression_batch_index = 0;
		solver.compression_type_batch_index = 0;
	}
	for searched_batch_count in 0 ..< batch_count
	{
		batch := &solver.active_set.batches.memory[solver.compression_batch_index];
		if solver.compression_type_batch_index >= 0 &&
			int(solver.compression_type_batch_index) < int(batch.type_batch_count)
		{
			break;
		}
		solver.compression_batch_index =
			i32((int(solver.compression_batch_index) + 1) % batch_count);
		solver.compression_type_batch_index = 0;
		if searched_batch_count == batch_count - 1 &&
			solver.active_set.batches.memory[solver.compression_batch_index].type_batch_count == 0
		{
			return .Ok;
		}
	}
	source_batch_index := int(solver.compression_batch_index);
	source_batch := &solver.active_set.batches.memory[source_batch_index];
	target_candidate_count := solver_compression_budget_count(
		constraint_count, 200,
	);
	maximum_application_count := solver_compression_budget_count(
		constraint_count, 2000,
	);
	first_type_batch_index := int(solver.compression_type_batch_index);
	type_batch_count := 0;
	scheduled_count := 0;
	for first_type_batch_index + type_batch_count < int(source_batch.type_batch_count) &&
			scheduled_count < target_candidate_count
	{
		type_batch := &source_batch.type_batches.memory[
			first_type_batch_index + type_batch_count
		];
		scheduled_count += int(type_batch.count);
		type_batch_count += 1;
	}
	solver.compression_type_batch_index += i32(type_batch_count);
	if scheduled_count == 0
	{
		return .Ok;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		worker_count = min(dispatcher.worker_count, scheduled_count);
	}
	statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
	for worker_index in 0 ..< worker_count
	{
		statuses[worker_index] = .Ok;
	}
	job := Solver_Compression_Discovery_Job{
		solver=solver,
		source_batch_index=source_batch_index,
		first_type_batch_index=first_type_batch_index,
		type_batch_count=type_batch_count,
		scheduled_count=scheduled_count,
		worker_count=worker_count,
		statuses=&statuses,
	};
	if dispatcher != nil && worker_count > 1
	{
		dispatch_status := dispatcher.dispatch(
			dispatcher, solver_compression_discovery_worker, worker_count, &job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
	}
	else
	{
		local_boundary := util.Thread_Dispatcher_Boundary{
			unmanaged_context=&job,
		};
		solver_compression_discovery_worker(0, &local_boundary);
	}
	for worker_index in 0 ..< worker_count
	{
		if statuses[worker_index] != .Ok
		{
			return statuses[worker_index];
		}
	}
	candidate_count := 0;
	for source_ordinal in 0 ..< scheduled_count
	{
		candidate := solver.compression_candidates.memory[source_ordinal];
		if candidate.handle.value < 0
		{
			continue;
		}
		solver.compression_candidates.memory[candidate_count] = candidate;
		candidate_count += 1;
	}
	application_count := min(candidate_count, maximum_application_count);
	for application_index in 0 ..< application_count
	{
		candidate := solver.compression_candidates.memory[application_index];
		location, resolve_status := solver_resolve(solver, candidate.handle);
		if resolve_status != .Ok
		{
			return resolve_status;
		}
		if int(location.batch_index) != source_batch_index
		{
			continue;
		}
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
		type_batch := &batch.type_batches.memory[type_batch_index];
		reference, reference_status := type_batch_read_reference(
			type_batch, solver.bodies, int(location.index_in_type_batch),
		);
		if reference_status != .Ok
		{
			return reference_status;
		}
		target_batch_index := int(candidate.target_batch);
		if target_batch_index < 0 || target_batch_index >= source_batch_index
		{
			return .Invalid_Argument;
		}
		target_batch := &solver.active_set.batches.memory[target_batch_index];
		if constraint_batch_body_fit(target_batch, &reference) == .Missing
		{
			continue;
		}
		move_status := solver_move_to_batch_internal(
			solver, candidate.handle, target_batch_index,
		);
		if move_status != .Ok
		{
			return move_status;
		}
	}
	return .Ok;
}
solver_clear :: proc (solver: ^Solver) -> Physics_Status
{
	if solver == nil || solver.state != .Ready
	{
		return .Disposed;
	}
	for batch_index in 0 ..< solver.active_set.batches.length
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< batch.type_batch_count
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			_ = util.buffer_clear(
				type_batch.body_references, 0, int(type_batch.body_references.length),
			);
			_ = util.buffer_clear(
				type_batch.prestep_data, 0, int(type_batch.prestep_data.length),
			);
			_ = util.buffer_clear(
				type_batch.accumulated_impulses, 0,
				int(type_batch.accumulated_impulses.length),
			);
			for index in 0 ..< type_batch.index_to_handle.length
			{
				type_batch.index_to_handle.memory[index] = -1;
			}
			type_batch.integration_flags_offset = 0;
			type_batch.integration_flag_word_count = 0;
			type_batch.has_integration_responsibilities = .Missing;
			type_batch.count = 0;
		}
		_ = util.buffer_clear(
			batch.body_reference_counts, 0, int(batch.body_reference_counts.length),
		);
		batch.constraint_count = 0;
		batch.work_block_start = 0;
		batch.work_block_count = 0;
	}
	for index in 0 ..< solver.handle_to_constraint.length
	{
		solver.handle_to_constraint.memory[index] = constraint_location_missing();
	}
	_ = util.buffer_clear(
		solver.integration_flags, 0, int(solver.integration_flags.length),
	);
	_ = util.buffer_clear(
		solver.integration_body_seen, 0,
		int(solver.integration_body_seen.length),
	);
	_ = util.buffer_clear(solver.work_blocks, 0, int(solver.work_blocks.length));
	for index in 0 ..< solver.contact_coefficient_offsets.length
	{
		solver.contact_coefficient_offsets.memory[index] = -1;
	}
	if solver.joint_breaks != nil
	{
		joint_break_reset(solver.joint_breaks);
	}
	util.id_pool_clear(&solver.handle_pool);
	solver.active_set.batch_count = 1;
	solver.active_set.constraint_count = 0;
	solver.sequential_batch.constraint_count = 0;
	solver.compression_batch_index = solver.fallback_batch_index;
	solver.compression_type_batch_index = 0;
	return .Ok;
}
solver_dispose :: proc (solver: ^Solver) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || solver.pool == nil
	{
		return .Disposed;
	}
	if solver.joint_breaks != nil
	{
		joint_break_release(solver.joint_breaks);
		solver.joint_breaks = nil;
	}
	if solver.bodies != nil && solver.bodies.solver == solver
	{
		solver.bodies.solver = nil;
	}
	for batch_index in 0 ..< solver.active_set.batches.length
	{
		constraint_batch_dispose(&solver.active_set.batches.memory[batch_index], solver.pool);
	}
	physics_return_buffer(solver.pool, &solver.active_set.batches);
	_ = util.id_pool_dispose(&solver.handle_pool, solver.pool);
	physics_return_buffer(solver.pool, &solver.compression_candidates);
	physics_return_buffer(solver.pool, &solver.contact_coefficients);
	physics_return_buffer(solver.pool, &solver.contact_coefficient_offsets);
	physics_return_buffer(solver.pool, &solver.work_blocks);
	physics_return_buffer(solver.pool, &solver.integration_body_seen);
	physics_return_buffer(solver.pool, &solver.integration_flags);
	physics_return_buffer(solver.pool, &solver.handle_to_constraint);
	_ = constraint_type_registry_dispose(&solver.registry);
	solver^ = {};
	solver.state = .Disposed;
	return .Ok;
}
