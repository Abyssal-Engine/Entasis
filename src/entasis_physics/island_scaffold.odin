// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

MAXIMUM_INACTIVE_IMPULSE_SCALARS :: 32;

Inactive_Constraint_Record :: struct
{
	handle:           Constraint_Handle,
	type_id:          i32,
	body_count:       i32,
	set_index:        i32,
	description_size: i32,
	impulse_count:    i32,
	body_handles:     [4]Body_Handle,
	description:      [CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8,
	impulses:         [MAXIMUM_INACTIVE_IMPULSE_SCALARS]f32,
	solver_add:       Solver_Constraint_Add_Transaction,
	solver_remove:    Solver_Constraint_Remove_Transaction,
}

Island_Scaffold_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

Island_Scaffold :: struct
{
	body_marks:       util.Buffer(u8),
	constraint_marks: util.Buffer(u8),
	stack:            util.Buffer(Body_Handle),
	island_bodies:    util.Buffer(Body_Handle),
	island_constraints: util.Buffer(Constraint_Handle),
	pool:             ^util.Buffer_Pool,
	state:            Island_Scaffold_State,
}

island_scaffold_initialize :: proc (
	scaffold: ^Island_Scaffold, body_capacity, constraint_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if scaffold == nil ||
		scaffold.state != .Uninitialized ||
		body_capacity <= 0 ||
		constraint_capacity <= 0 ||
		pool == nil
	{
		return .Invalid_Argument;
	}
	body_marks, body_status := util.buffer_pool_take_at_least(pool, u8, body_capacity);
	if body_status != .Ok
	{
		return physics_memory_status(body_status);
	}
	constraint_marks, constraint_status := util.buffer_pool_take_at_least(pool, u8, constraint_capacity);
	if constraint_status != .Ok
	{
		physics_return_buffer(pool, &body_marks);
		return physics_memory_status(constraint_status);
	}
	stack, stack_status := util.buffer_pool_take_at_least(pool, Body_Handle, body_capacity);
	if stack_status != .Ok
	{
		physics_return_buffer(pool, &constraint_marks);
		physics_return_buffer(pool, &body_marks);
		return physics_memory_status(stack_status);
	}
	island_bodies, island_status := util.buffer_pool_take_at_least(pool, Body_Handle, body_capacity);
	if island_status != .Ok
	{
		physics_return_buffer(pool, &stack);
		physics_return_buffer(pool, &constraint_marks);
		physics_return_buffer(pool, &body_marks);
		return physics_memory_status(island_status);
	}
	island_constraints, island_constraint_status := util.buffer_pool_take_at_least(
		pool,
		Constraint_Handle,
		constraint_capacity
	);
	if island_constraint_status != .Ok
	{
		physics_return_buffer(pool, &island_bodies);
		physics_return_buffer(pool, &stack);
		physics_return_buffer(pool, &constraint_marks);
		physics_return_buffer(pool, &body_marks);
		return physics_memory_status(island_constraint_status);
	}
	_ = util.buffer_clear(body_marks, 0, int(body_marks.length));
	_ = util.buffer_clear(constraint_marks, 0, int(constraint_marks.length));
	scaffold^ = {
		body_marks=body_marks, constraint_marks=constraint_marks, stack=stack,
		island_bodies=island_bodies, island_constraints=island_constraints,
		pool=pool, state=.Ready,
	};
	return .Ok;
}

island_scaffold_clear :: proc (scaffold: ^Island_Scaffold) -> Physics_Status
{
	if scaffold == nil || scaffold.state != .Ready
	{
		return .Disposed;
	}
	_ = util.buffer_clear(scaffold.body_marks, 0, int(scaffold.body_marks.length));
	_ = util.buffer_clear(scaffold.constraint_marks, 0, int(scaffold.constraint_marks.length));
	return .Ok;
}

island_scaffold_ensure_capacity :: proc (
	scaffold: ^Island_Scaffold, body_capacity, constraint_capacity: int,
) -> Physics_Status
{
	if scaffold == nil || scaffold.state != .Ready ||
		body_capacity <= 0 || constraint_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := physics_ensure_zeroed_buffer_capacity(
		scaffold.pool, &scaffold.body_marks, body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		scaffold.pool, &scaffold.stack, body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		scaffold.pool, &scaffold.island_bodies, body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_zeroed_buffer_capacity(
		scaffold.pool, &scaffold.constraint_marks, constraint_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	return physics_ensure_buffer_capacity(
		scaffold.pool, &scaffold.island_constraints,
		constraint_capacity, 0,
	);
}

island_scaffold_resize :: proc (
	scaffold: ^Island_Scaffold, body_capacity, constraint_capacity: int,
) -> Physics_Status
{
	if scaffold == nil || scaffold.state != .Ready ||
		body_capacity <= 0 || constraint_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := physics_resize_zeroed_buffer_capacity(
		scaffold.pool, &scaffold.body_marks, body_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		scaffold.pool, &scaffold.stack, body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_buffer_capacity(
		scaffold.pool, &scaffold.island_bodies, body_capacity, 0,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_resize_zeroed_buffer_capacity(
		scaffold.pool, &scaffold.constraint_marks,
		constraint_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	return physics_resize_buffer_capacity(
		scaffold.pool, &scaffold.island_constraints,
		constraint_capacity, 0,
	);
}

solver_gather_inactive_constraint :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, set_index: int, target: ^Inactive_Constraint_Record,
) -> Physics_Status
{
	if target == nil || set_index <= 0
	{
		return .Invalid_Argument;
	}
	location, status := solver_resolve(solver, handle);
	if status != .Ok
	{
		return status;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	type_batch := &batch.type_batches.memory[type_batch_index];
	reference, reference_status := type_batch_read_reference(
		type_batch,
		solver.bodies,
		int(location.index_in_type_batch)
	);
	if reference_status != .Ok
	{
		return reference_status;
	}
	record, record_status := constraint_type_registry_lookup(&solver.registry, location.type_id);
	if record_status != .Ok
	{
		return record_status;
	}
	impulse_count := int(record.impulse_bundle_size) / size_of(util.F32x8);
	if impulse_count > MAXIMUM_INACTIVE_IMPULSE_SCALARS
	{
		return .Capacity_Missing;
	}
	target^ = {
		handle=handle,
		type_id=location.type_id,
		body_count=reference.body_count,
		set_index=i32(set_index),
		description_size=record.description_size,
		impulse_count=i32(impulse_count),
		body_handles=reference.body_handles,
	};
	build_status := record.build_description(
		type_batch_prestep_bundle(type_batch, int(location.index_in_type_batch)), &target.description[0],
		int(location.type_id), int(record.description_size), int(record.prestep_bundle_size),
		int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT,
	);
	if build_status != .Ok
	{
		return build_status;
	}
	impulses := ([^]util.F32x8)(type_batch_impulse_bundle(type_batch, int(location.index_in_type_batch)));
	lane := int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT;
	for index in 0 ..< impulse_count
	{
		target.impulses[index] = simd.extract(impulses[index], lane);
	}
	target.solver_remove = {
		type_record=record,
		batch=batch,
		type_batch=type_batch,
		reference=reference,
		batch_index=location.batch_index,
		type_batch_index=i32(type_batch_index),
		record_index=location.index_in_type_batch,
	};
	for body_index in 0 ..< int(reference.body_count)
	{
		body_location, body_status := bodies_resolve(solver.bodies, reference.body_handles[body_index]);
		if body_status != .Ok || body_location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			return .Invalid_Argument;
		}
		target.solver_remove.body_lists[body_index] =
			&solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].constraints.memory[body_location.index];
	}
	return .Ok;
}

solver_commit_restore_inactive_constraint :: proc "contextless" (
	solver: ^Solver, source: ^Inactive_Constraint_Record,
)
{
	prepared := &source.solver_add;
	record := prepared.type_record;
	type_batch := prepared.type_batch;
	index := int(prepared.record_index);
	prestep := type_batch_prestep_bundle(type_batch, index);
	impulses := type_batch_impulse_bundle(type_batch, index);
	lane := index % util.PRODUCTION_LANE_COUNT;
	record.remove_record(
		prestep, impulses, int(record.prestep_bundle_size), int(record.impulse_bundle_size), lane,
	);
	record.apply_description(
		&source.description[0], prestep, int(source.type_id), int(record.description_size),
		int(record.prestep_bundle_size), lane,
	);
	type_batch_write_reference_trusted(type_batch, index, prepared.reference);
	impulse_vectors := ([^]util.F32x8)(impulses);
	for impulse_index in 0 ..< int(source.impulse_count)
	{
		impulse_vectors[impulse_index] = simd.replace(
			impulse_vectors[impulse_index], lane, source.impulses[impulse_index],
		);
	}
	type_batch.count += 1;
	prepared.batch.constraint_count += 1;
	if prepared.batch_index == solver.fallback_batch_index
	{
		sequential_fallback_add_body_references(&solver.sequential_batch, &prepared.reference);
		solver.sequential_batch.constraint_count += 1;
	}
	else
	{
		constraint_batch_add_body_references(prepared.batch, &prepared.reference);
	}
	for body_index in 0 ..< int(prepared.reference.body_count)
	{
		list := prepared.body_lists[body_index];
		list.span.memory[prepared.body_list_indices[body_index]] = {
			prepared.reference.handle, i32(body_index),
		};
		list.count += 1;
	}
	solver.handle_to_constraint.memory[source.handle.value] = {
		0, prepared.batch_index, source.type_id, prepared.record_index,
	};
	solver.active_set.constraint_count += 1;
	if prepared.batch_index + 1 > solver.active_set.batch_count
	{
		solver.active_set.batch_count = prepared.batch_index + 1;
	}
}

island_scaffold_dispose :: proc (scaffold: ^Island_Scaffold) -> Physics_Status
{
	if scaffold == nil || scaffold.state != .Ready || scaffold.pool == nil
	{
		return .Disposed;
	}
	pool := scaffold.pool;
	physics_return_buffer(pool, &scaffold.island_constraints);
	physics_return_buffer(pool, &scaffold.island_bodies);
	physics_return_buffer(pool, &scaffold.stack);
	physics_return_buffer(pool, &scaffold.constraint_marks);
	physics_return_buffer(pool, &scaffold.body_marks);
	scaffold^ = {state=.Disposed};
	return .Ok;
}
