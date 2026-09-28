// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"

Island_Awakener :: struct
{
	sleeper: ^Island_Sleeper,
}

Island_Awaken_Transaction :: struct
{
	set_index:        i32,
	target_start:     i32,
	body_count:       i32,
	constraint_count: i32,
}

island_awakener_initialize :: proc "contextless" (
	awakener: ^Island_Awakener, sleeper: ^Island_Sleeper,
) -> Physics_Status
{
	if awakener == nil || sleeper == nil || sleeper.state != .Ready
	{
		return .Invalid_Argument;
	}
	awakener.sleeper = sleeper;
	return .Ok;
}

Island_Awakener_Phase_One_Job :: struct
{
	awakener:      ^Island_Awakener,
	set_index:      int,
	target_start:   int,
	body_count:      int,
	reset_activity: Reference_State,
	job_count:      int,
	worker_count:   int,
}

island_awakener_phase_one_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Awakener_Phase_One_Job)(dispatcher.unmanaged_context);
	sleeper := job.awakener.sleeper;
	source := &sleeper.bodies.sets.memory[job.set_index];
	target := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for job_index := worker_index; job_index < job.job_count; job_index += job.worker_count
	{
		source_index := job_index;
		target_index := job.target_start + source_index;
		handle := source.index_to_handle.memory[source_index];
		target.dynamics_state.memory[target_index] = source.dynamics_state.memory[source_index];
		target.index_to_handle.memory[target_index] = handle;
		target.collidables.memory[target_index] = source.collidables.memory[source_index];
		target.activity.memory[target_index] = source.activity.memory[source_index];
		target.constraints.memory[target_index] = source.constraints.memory[source_index];
		if job.reset_activity == .Present
		{
			target.activity.memory[target_index].timesteps_under_threshold_count = 0;
			target.activity.memory[target_index].sleep_candidate = .Not_Candidate;
		}
	}
}

Island_Awakener_Phase_Two_Job :: struct
{
	awakener:    ^Island_Awakener,
	set_index:    int,
	target_start: int,
	body_count:    int,
	constraint_count: int,
	worker_count: int,
}

island_awakener_phase_two_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Island_Awakener_Phase_Two_Job)(dispatcher.unmanaged_context);
	sleeper := job.awakener.sleeper;
	if worker_index == 0
	{
		active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		active.count += job.body_count;
		for body_index in 0 ..< job.body_count
		{
			handle := active.index_to_handle.memory[job.target_start + body_index];
			sleeper.bodies.handle_to_location.memory[handle.value] = {
				BODIES_ACTIVE_SET_INDEX, i32(job.target_start + body_index),
			};
		}
		for body_index in 0 ..< job.body_count
		{
			broad_phase_commit_body_migration(sleeper.broad_phase, &sleeper.body_migrations.memory[body_index]);
		}
	}
	if worker_index == 1 || worker_index == 0 && job.worker_count == 1
	{
		pair_cache_commit_awaken_set(sleeper.pair_cache, job.set_index);
		for constraint_index in 0 ..< job.constraint_count
		{
			order_index := job.constraint_count - 1 - constraint_index;
			record_index := sleeper.constraint_order.memory[order_index].record_index;
			solver_commit_restore_inactive_constraint(
				sleeper.solver, &sleeper.inactive_constraints.memory[record_index],
			);
		}
	}
}

island_awakener_batch_accepts :: proc "contextless" (
	sleeper: ^Island_Sleeper, batch_index: int, reference: ^Constraint_Reference,
) -> Reference_State
{
	batch := &sleeper.solver.active_set.batches.memory[batch_index];
	claim_word_count := max(util.index_set_bundle_capacity(int(sleeper.body_claims.length)), 1);
	claim_start := batch_index * claim_word_count;
	for body_index in 0 ..< int(reference.body_count)
	{
		if u32(reference.encoded_body_references[body_index]) >= BODY_REFERENCE_KINEMATIC_MASK
		{
			continue;
		}
		handle := reference.body_handles[body_index];
		if batch.body_reference_counts.memory[handle.value] != 0
		{
			return .Missing;
		}
		word := &sleeper.body_batch_claims.memory[
			claim_start + (int(handle.value) >> util.INDEX_SET_SHIFT)
		];
		mask := u64(1) << uint(int(handle.value) & util.INDEX_SET_MASK);
		if word.generation == sleeper.claim_generation && (word.bits & mask) != 0
		{
			return .Missing;
		}
	}
	return .Present;
}

island_awakener_prepare_transaction :: proc (
	awakener: ^Island_Awakener, set_index: int,
) -> (Island_Awaken_Transaction, Physics_Status)
{
	sleeper := awakener.sleeper;
	capacity_status := island_sleeper_ensure_capacity(
		sleeper,
		max(int(sleeper.bodies.handle_to_location.length), 1),
		max(int(sleeper.solver.handle_to_constraint.length), 1),
	);
	if capacity_status != .Ok
	{
		return {}, capacity_status;
	}
	if sleeper.claim_generation == max(u32)
	{
		status := util.buffer_clear(sleeper.body_claims, 0, int(sleeper.body_claims.length));
		if status != .Ok
		{
			return {}, physics_memory_status(status);
		}
		status = util.buffer_clear(sleeper.body_batch_claims, 0, int(sleeper.body_batch_claims.length));
		if status != .Ok
		{
			return {}, physics_memory_status(status);
		}
		status = util.buffer_clear(sleeper.type_claims, 0, int(sleeper.type_claims.length));
		if status != .Ok
		{
			return {}, physics_memory_status(status);
		}
		sleeper.claim_generation = 1;
	}
	else
	{
		sleeper.claim_generation += 1;
	}
	set := &sleeper.bodies.sets.memory[set_index];
	active := &sleeper.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	body_count := set.count;
	target_start := active.count;
	if body_count > max(int) - target_start
	{
		return {}, .Capacity_Missing;
	}
	required_active_count := target_start + body_count;
	body_capacity_status := body_set_ensure_capacity(
		active, max(required_active_count, 1), sleeper.bodies.pool,
	);
	if body_capacity_status != .Ok
	{
		return {}, body_capacity_status;
	}
	if sleeper.broad_phase.active_tree.leaf_count > max(int) - body_count
	{
		return {}, .Capacity_Missing;
	}
	broad_phase_status := broad_phase_ensure_capacity(
		sleeper.broad_phase,
		max(sleeper.broad_phase.active_tree.leaf_count + body_count, 1),
		max(sleeper.broad_phase.static_tree.leaf_count, 1),
		1,
	);
	if broad_phase_status != .Ok
	{
		return {}, broad_phase_status;
	}
	pair_status := pair_cache_prepare_awaken_set(sleeper.pair_cache, set_index);
	if pair_status != .Ok
	{
		return {}, pair_status;
	}
	constraint_count := 0;
	for index in 0 ..< sleeper.inactive_constraint_count
	{
		record := &sleeper.inactive_constraints.memory[index];
		if int(record.set_index) != set_index
		{
			continue;
		}
		if constraint_count >= int(sleeper.constraint_order.length)
		{
			return {}, .Capacity_Missing;
		}
		sleeper.constraint_order.memory[constraint_count] = {
			key=u64(u32(index)), record_index=i32(index),
		};
		type_record, lookup_status := constraint_type_registry_lookup(&sleeper.solver.registry, record.type_id);
		if lookup_status != .Ok || type_record.body_count != record.body_count ||
		type_record.description_size != record.description_size
		{
			return {}, .Invalid_Description;
		}
		validation_status: Physics_Status = constraint_type_validate_description(type_record, record.type_id, &record.description[0]);
		if validation_status != .Ok
		{
			return {}, validation_status;
		}
		reference := Constraint_Reference{
			handle=record.handle, body_handles=record.body_handles, body_count=record.body_count,
		};
		for body_index in 0 ..< int(record.body_count)
		{
			body_location, body_status := bodies_resolve(sleeper.bodies, record.body_handles[body_index]);
			if body_status != .Ok || int(body_location.set_index) != set_index
			{
				return {}, .Invalid_Argument;
			}
			projected_index := target_start + int(body_location.index);
			encoded := u32(projected_index);
			inertia := set.dynamics_state.memory[body_location.index].inertia.local;
			if body_inertia_mobility(inertia) == .Kinematic
			{
				encoded |= BODY_REFERENCE_KINEMATIC_MASK;
			}
			reference.encoded_body_references[body_index] = i32(encoded);
		}
		batch_index := int(sleeper.solver.fallback_batch_index);
		for candidate_index in 0 ..< int(sleeper.solver.fallback_batch_index)
		{
			if island_awakener_batch_accepts(sleeper, candidate_index, &reference) == .Present
			{
				batch_index = candidate_index;
				break;
			}
		}
		if batch_index < int(sleeper.solver.fallback_batch_index)
		{
			claim_word_count := max(util.index_set_bundle_capacity(int(sleeper.body_claims.length)), 1);
			claim_start := batch_index * claim_word_count;
			for body_index in 0 ..< int(reference.body_count)
			{
				if u32(reference.encoded_body_references[body_index]) >= BODY_REFERENCE_KINEMATIC_MASK
				{
					continue;
				}
				handle := reference.body_handles[body_index];
				word := &sleeper.body_batch_claims.memory[
					claim_start + (int(handle.value) >> util.INDEX_SET_SHIFT)
				];
				if word.generation != sleeper.claim_generation
				{
					word^ = {generation=sleeper.claim_generation};
				}
				word.bits |= u64(1) << uint(int(handle.value) & util.INDEX_SET_MASK);
			}
		}
		batch := &sleeper.solver.active_set.batches.memory[batch_index];
		type_batch, type_batch_status := constraint_batch_get_or_create_type_batch(
			batch, type_record, max(int(sleeper.solver.initial_type_batch_capacity), 1),
			sleeper.solver.pool,
		);
		if type_batch_status != .Ok
		{
			return {}, type_batch_status;
		}
		type_batch_index := int(batch.type_id_to_batch_index[record.type_id]);
		type_claim_index := batch_index * CONSTRAINT_TYPE_ID_CAPACITY + int(record.type_id);
		type_claim := &sleeper.type_claims.memory[type_claim_index];
		if type_claim.generation != sleeper.claim_generation
		{
			type_claim^ = {generation=sleeper.claim_generation};
		}
		target_index := int(type_batch.count) + int(type_claim.count);
		if target_index >= int(type_batch.index_to_handle.length)
		{
			type_capacity_status := type_batch_ensure_capacity(
				type_batch, max(target_index + 1, int(type_batch.index_to_handle.length) * 2),
				sleeper.solver.pool,
			);
			if type_capacity_status != .Ok
			{
				return {}, type_capacity_status;
			}
		}
		type_claim.count += 1;
		record.solver_add = {
			type_record=type_record,
			batch=batch,
			type_batch=type_batch,
			reference=reference,
			batch_index=i32(batch_index),
			type_batch_index=i32(type_batch_index),
			record_index=i32(target_index),
		};
		for body_index in 0 ..< int(record.body_count)
		{
			body_location, _ := bodies_resolve(sleeper.bodies, record.body_handles[body_index]);
			source_list := &set.constraints.memory[body_location.index];
			claim := &sleeper.body_claims.memory[record.body_handles[body_index].value];
			if claim.generation != sleeper.claim_generation
			{
				claim^ = {generation=sleeper.claim_generation};
			}
			if source_list.count > max(int) - int(claim.list_count) - 1
			{
				return {}, .Capacity_Missing;
			}
			list_target_index := source_list.count + int(claim.list_count);
			list_capacity_status := util.quick_list_ensure_capacity(
				source_list, list_target_index + 1, sleeper.solver.pool,
			);
			if list_capacity_status != .Ok
			{
				return {}, physics_collection_status(list_capacity_status);
			}
			claim.list_count += 1;
			list := &active.constraints.memory[target_start + int(body_location.index)];
			record.solver_add.body_lists[body_index] = list;
			record.solver_add.body_list_indices[body_index] = i32(list_target_index);
		}
		constraint_count += 1;
	}
	sort_status := solver_sort_remove_order_descending(
		sleeper.constraint_order, sleeper.constraint_sort_scratch, constraint_count,
	);
	if sort_status != .Ok
	{
		return {}, sort_status;
	}
	migration_count := 0;
	for body_index in 0 ..< body_count
	{
		handle := set.index_to_handle.memory[body_index];
		location, location_status := bodies_resolve(sleeper.bodies, handle);
		if location_status != .Ok || int(location.set_index) != set_index || int(location.index) != body_index
		{
			return {}, .Invalid_Argument;
		}
		migration, migration_status := broad_phase_prepare_body_migration(sleeper.broad_phase, handle, .Active);
		if migration_status != .Ok
		{
			return {}, migration_status;
		}
		sleeper.body_migrations.memory[body_index] = migration;
		if migration.state == .Present
		{
			migration_count += 1;
		}
	}
	migration_capacity_status := broad_phase_validate_body_migration_capacity(
		sleeper.broad_phase, .Active, migration_count,
	);
	if migration_capacity_status != .Ok
	{
		return {}, migration_capacity_status;
	}
	return {
		set_index=i32(set_index), target_start=i32(target_start), body_count=i32(body_count),
		constraint_count=i32(constraint_count),
	}, .Ok;
}

island_awakener_awaken_set :: proc (
	awakener: ^Island_Awakener, set_index: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	reset_activity: Reference_State = .Present,
) -> Physics_Status
{
	if awakener == nil || awakener.sleeper == nil || awakener.sleeper.state != .Ready || set_index <= 0 ||
	set_index >= int(awakener.sleeper.bodies.sets.length)
	{
		return .Invalid_Argument;
	}
	sleeper := awakener.sleeper;
	set := &sleeper.bodies.sets.memory[set_index];
	if set.state != .Allocated
	{
		return .Not_Found;
	}
	body_count := set.count;
	if body_count == 0
	{
		return .Ok;
	}
	transaction, prepare_status := island_awakener_prepare_transaction(awakener, set_index);
	if prepare_status != .Ok
	{
		return prepare_status;
	}
	target_start := int(transaction.target_start);
	phase_one_job_count := body_count;
	phase_one_worker_count := 1;
	if dispatcher != nil
	{
		phase_one_worker_count = min(dispatcher.worker_count, phase_one_job_count);
	}
	phase_one_job := Island_Awakener_Phase_One_Job{
		awakener, set_index, target_start, body_count, reset_activity,
		phase_one_job_count, phase_one_worker_count,
	};
	if dispatcher != nil && phase_one_worker_count > 1
	{
		dispatch_status := dispatcher.dispatch(
			dispatcher, island_awakener_phase_one_worker, phase_one_worker_count, &phase_one_job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
	}
	else
	{
		local_boundary := util.Thread_Dispatcher_Boundary{unmanaged_context=&phase_one_job};
		island_awakener_phase_one_worker(0, &local_boundary);
	}
	phase_two_worker_count := 1;
	if dispatcher != nil
	{
		phase_two_worker_count = min(dispatcher.worker_count, 2);
	}
	phase_two_job := Island_Awakener_Phase_Two_Job{
		awakener=awakener,
		set_index=set_index,
		target_start=target_start,
		body_count=body_count,
		constraint_count=int(transaction.constraint_count),
		worker_count=phase_two_worker_count,
	};
	if dispatcher != nil && phase_two_worker_count > 1
	{
		dispatch_status := dispatcher.dispatch(
			dispatcher, island_awakener_phase_two_worker, phase_two_worker_count, &phase_two_job,
		);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
	}
	else
	{
		local_boundary := util.Thread_Dispatcher_Boundary{unmanaged_context=&phase_two_job};
		island_awakener_phase_two_worker(0, &local_boundary);
	}
	set.count = 0;
	for constraint_index in 0 ..< int(transaction.constraint_count)
	{
		index := int(sleeper.constraint_order.memory[constraint_index].record_index);
		sleeper.inactive_constraint_count -= 1;
		if index < sleeper.inactive_constraint_count
		{
			sleeper.inactive_constraints.memory[index] =
			sleeper.inactive_constraints.memory[sleeper.inactive_constraint_count];
			moved := &sleeper.inactive_constraints.memory[index];
			sleeper.solver.handle_to_constraint.memory[
				moved.handle.value
			].index_in_type_batch = i32(index);
		}
		sleeper.inactive_constraints.memory[sleeper.inactive_constraint_count] = {};
	}
	return .Ok;
}

island_awakener_awaken_body :: proc (
	awakener: ^Island_Awakener, handle: Body_Handle,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	reset_activity: Reference_State = .Present,
) -> Physics_Status
{
	if awakener == nil || awakener.sleeper == nil
	{
		return .Invalid_Argument;
	}
	location, status := bodies_resolve(awakener.sleeper.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	if location.set_index == BODIES_ACTIVE_SET_INDEX
	{
		return .Ok;
	}
	return island_awakener_awaken_set(awakener, int(location.set_index), dispatcher, reset_activity);
}
