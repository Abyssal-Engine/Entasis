package solver_contacts

import "base:intrinsics"
import "core:simd"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Canonical_Type :: struct
{
	native: ^physics.Type_Batch,
	prestep: []u8,
	impulses: []u8,
}
Replay :: struct
{
	bodies: physics.Bodies,
	solver: physics.Solver,
	integrator: physics.Pose_Integrator,
	default_pose_context: physics.Default_Pose_Integrator_Context,
	input: ^Input,
	pool: ^util.Buffer_Pool,
	canonical: []Canonical_Type,
}

wide_put_vector :: proc(target: [^]util.F32x8, offset, lane: int, value: util.Vector3)
{
	target[offset] = simd.replace(target[offset], lane, value.x);
	target[offset+1] = simd.replace(target[offset+1], lane, value.y);
	target[offset+2] = simd.replace(target[offset+2], lane, value.z);
}

contact_write :: proc(type_batch: ^physics.Type_Batch, index: int, contact: Contact_Input)
{
	point_count: int = int(type_batch.type_id % 4 + 1);
	body_count: int = int(type_batch.body_count);
	lane: int = index % util.PRODUCTION_LANE_COUNT;
	prestep: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_prestep_bundle(type_batch, index));
	impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, index));
	references: [^]util.I32x8 = physics.type_batch_body_bundle(type_batch, index);
	type_batch.index_to_handle.memory[index] = contact.handle;
	for body in 0 ..< body_count
	{
		references[body] = simd.replace(references[body], lane, contact.bodies[body]);
	}
	for point in 0 ..< point_count
	{
		wide_put_vector(prestep, 4*point, lane, contact.offsets[point]);
		prestep[4*point+3] = simd.replace(prestep[4*point+3], lane, contact.depths[point]);
		impulses[2+point] = simd.replace(impulses[2+point], lane, contact.normal_impulses[point]);
	}
	cursor: int = point_count*4;
	if body_count == 2
	{
		wide_put_vector(prestep, cursor, lane, contact.offset_b);
		cursor += 3;
	}
	wide_put_vector(prestep, cursor, lane, contact.normal);
	prestep[cursor+3] = simd.replace(prestep[cursor+3], lane, contact.material.friction_coefficient);
	prestep[cursor+4] = simd.replace(prestep[cursor+4], lane, contact.material.spring_settings.angular_frequency);
	prestep[cursor+5] = simd.replace(prestep[cursor+5], lane, contact.material.spring_settings.twice_damping_ratio);
	prestep[cursor+6] = simd.replace(prestep[cursor+6], lane, contact.material.maximum_recovery_velocity);
	impulses[0] = simd.replace(impulses[0], lane, contact.tangent_impulse.x);
	impulses[1] = simd.replace(impulses[1], lane, contact.tangent_impulse.y);
	impulses[2+point_count] = simd.replace(impulses[2+point_count], lane, contact.twist_impulse);
}

replay_destroy :: proc(replay: ^Replay)
{
	for canonical in replay.canonical
	{
		delete(canonical.prestep);
		delete(canonical.impulses);
	}
	delete(replay.canonical);
	if replay.integrator.state == .Ready
	{
		_ = physics.pose_integrator_dispose(&replay.integrator);
	}
	if replay.solver.state == .Ready
	{
		_ = physics.solver_dispose(&replay.solver);
	}
	if replay.bodies.state == .Allocated
	{
		_ = physics.bodies_dispose(&replay.bodies);
	}
	replay^ = {};
}

// consumes input admitted by input_read or input_validate
replay_create :: proc(replay: ^Replay, input: ^Input, pool: ^util.Buffer_Pool) -> physics.Physics_Status
{
	replay.input = input;
	replay.pool = pool;
	body_capacity: int = len(input.dynamics);
	constraint_capacity: int = 1;
	type_count: int;
	for handle in input.body_handles
	{
		body_capacity = max(body_capacity, int(handle)+1);
	}
	degrees: []int = make([]int, len(input.dynamics));
	defer delete(degrees);
	for batch in input.batches
	{
		type_count += len(batch.types);
		for kind in batch.types
		{
			for contact in kind.contacts
			{
				constraint_capacity = max(constraint_capacity, int(contact.handle)+1);
				for body in 0 ..< 1+int(kind.type_id/4)
				{
					degrees[int(u32(contact.bodies[body]) & physics.BODY_REFERENCE_INDEX_MASK)] += 1;
				}
			}
		}
	}
	status: physics.Physics_Status = physics.bodies_initialize(&replay.bodies, body_capacity, 1, 1, pool);
	if status != .Ok
	{
		return status;
	}
	active: ^physics.Body_Set = &replay.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for index in 0 ..< len(input.dynamics)
	{
		collection_status: util.Collection_Status = util.quick_list_initialize(&active.constraints.memory[index], max(degrees[index], 1), pool);
		if collection_status != .Ok
		{
			replay_destroy(replay);
			return .Capacity_Missing;
		}
		active.count += 1;
		handle: i32 = input.body_handles[index];
		active.index_to_handle.memory[index] = {handle};
		active.activity.memory[index] = {sleep_threshold=-1};
		replay.bodies.handle_to_location.memory[handle] = {set_index=0, index=i32(index)};
	}
	status = physics.solver_initialize(&replay.solver, &replay.bodies, constraint_capacity, int(input.fallback_batch_threshold)+1, int(input.fallback_batch_threshold), 1, pool);
	if status != .Ok
	{
		replay_destroy(replay);
		return status;
	}
	replay.canonical = make([]Canonical_Type, type_count);
	replay.solver.active_set.batch_count = i32(len(input.batches));
	replay.solver.active_set.constraint_count = 0;
	canonical_index: int;
	for source_batch, batch_index in input.batches
	{
		batch: ^physics.Constraint_Batch = &replay.solver.active_set.batches.memory[batch_index];
		physics.constraint_batch_dispose(batch, pool);
		status = physics.constraint_batch_initialize(batch, body_capacity, pool);
		if status != .Ok
		{
			replay_destroy(replay);
			return status;
		}
		for kind in source_batch.types
		{
			type_batch: ^physics.Type_Batch;
			type_batch, status = physics.constraint_batch_get_or_create_type_batch(batch, &replay.solver.registry.records[kind.type_id], max(len(kind.contacts), 1), pool);
			if status != .Ok
			{
				replay_destroy(replay);
				return status;
			}
			type_batch.count = i32(len(kind.contacts));
			batch.constraint_count += type_batch.count;
			replay.solver.active_set.constraint_count += type_batch.count;
			for index in 0 ..< int(type_batch.body_references.length)
			{
				type_batch.body_references.memory[index] = 255;
			}
			for contact, index in kind.contacts
			{
				contact_write(type_batch, index, contact);
				replay.solver.handle_to_constraint.memory[contact.handle] = {set_index=0, batch_index=i32(batch_index), type_id=kind.type_id, index_in_type_batch=i32(index)};
				for body in 0 ..< int(type_batch.body_count)
				{
					body_index: int = int(u32(contact.bodies[body]) & physics.BODY_REFERENCE_INDEX_MASK);
					handle: i32 = input.body_handles[body_index];
					if u32(contact.bodies[body]) < physics.BODY_REFERENCE_KINEMATIC_MASK
					{
						count_index: int = int(handle);
						if batch_index == int(input.fallback_batch_index)
						{
							count_index = body_index;
						}
						batch.body_reference_counts.memory[count_index] += 1;
					}
					_ = util.quick_list_add_unsafely(&active.constraints.memory[body_index], physics.Body_Constraint_Reference{connecting_constraint_handle={contact.handle}, body_index_in_constraint=i32(body)});
				}
			}
			bundle_count: int = (len(kind.contacts)+util.PRODUCTION_LANE_COUNT-1)/util.PRODUCTION_LANE_COUNT;
			canonical: ^Canonical_Type = &replay.canonical[canonical_index];
			canonical_index += 1;
			canonical.native = type_batch;
			canonical.prestep = make([]u8, bundle_count*int(type_batch.prestep_bundle_size));
			canonical.impulses = make([]u8, bundle_count*int(type_batch.impulse_bundle_size));
			if len(canonical.prestep) > 0
			{
				intrinsics.mem_copy(raw_data(canonical.prestep), type_batch.prestep_data.memory, len(canonical.prestep));
				intrinsics.mem_copy(raw_data(canonical.impulses), type_batch.accumulated_impulses.memory, len(canonical.impulses));
			}
		}
	}
	replay.solver.sequential_batch.constraint_count = replay.solver.active_set.batches.memory[input.fallback_batch_index].constraint_count;
	replay.default_pose_context = input.pose_context;
	status = physics.pose_integrator_initialize(&replay.integrator, &replay.bodies, physics.pose_integrator_default_callbacks(&replay.default_pose_context));
	if status != .Ok
	{
		replay_destroy(replay);
		return status;
	}
	replay.solver.integrator = &replay.integrator;
	replay_reset(replay);
	return .Ok;
}

replay_reset :: proc(replay: ^Replay)
{
	active: ^physics.Body_Set = &replay.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	intrinsics.mem_copy(active.dynamics_state.memory, raw_data(replay.input.dynamics), len(replay.input.dynamics)*size_of(physics.Body_Dynamics));
	for canonical in replay.canonical
	{
		if len(canonical.prestep) > 0
		{
			intrinsics.mem_copy(canonical.native.prestep_data.memory, raw_data(canonical.prestep), len(canonical.prestep));
			intrinsics.mem_copy(canonical.native.accumulated_impulses.memory, raw_data(canonical.impulses), len(canonical.impulses));
		}
	}
	replay.default_pose_context = replay.input.pose_context;
}

replay_solve :: proc(replay: ^Replay, dispatcher: ^util.Thread_Dispatcher_Boundary = nil) -> physics.Physics_Status
{
	status: physics.Physics_Status = physics.pose_integrator_prepare(&replay.integrator, replay.input.dt/f32(replay.input.solve.substep_count));
	if status != .Ok
	{
		return status;
	}
	return physics.solver_solve(&replay.solver, replay.input.dt, &replay.input.solve, dispatcher);
}

logical_point_updates :: proc(input: ^Input, calls: int) -> i64
{
	counts: Counts = input_counts(input);
	return i64(calls)*i64(input.solve.substep_count)*i64(input.solve.velocity_iteration_count)*counts.points;
}
