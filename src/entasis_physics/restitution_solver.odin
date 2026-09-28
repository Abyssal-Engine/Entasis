package entasis_physics

import "base:runtime"
import "core:simd"
import util "entasis:entasis_utilities"

// optional buffers are indexed by the existing synchronized work blocks, followed
// by fourteen fallback contact-type slots. only selected blocks own lane/target
// spans. no lookup or allocation occurs in a contact Solve iteration
Restitution_Block :: struct
{
	target_offset: i32,
	bundle_offset: i32,
}

Restitution_Bundle :: struct
{
	coefficient: util.F32x8,
	threshold: util.F32x8,
	pair_indices: [util.PRODUCTION_LANE_COUNT]i32,
}

restitution_block_description :: proc "contextless" (
	storage: ^Restitution_Storage, index: int,
) -> Solver_Work_Block
{
	solver: ^Solver = &storage.simulation.solver;
	if index < storage.normal_work_count
	{
		return solver.work_blocks.memory[index];
	}
	type_id: int = index-storage.normal_work_count;
	batch: ^Constraint_Batch = &solver.active_set.batches.memory[solver.fallback_batch_index];
	type_index: i16 = batch.type_id_to_batch_index[type_id];
	return {
		batch_index=i16(solver.fallback_batch_index), type_batch_index=type_index,
		end_bundle=(batch.type_batches.memory[type_index].count+util.PRODUCTION_LANE_COUNT-1)/util.PRODUCTION_LANE_COUNT,
	};
}

// work blocks are emitted by solver_prepare_work_blocks in type-batch order,
// with increasing, nonoverlapping bundle ranges inside each type. use that
// producer ordering without a second index allocation or a linear pair scan
restitution_pair_block :: proc "contextless" (
	storage: ^Restitution_Storage, location: Constraint_Location,
) -> int
{
	solver: ^Solver = &storage.simulation.solver;
	if location.batch_index == solver.fallback_batch_index
	{
		return storage.normal_work_count+int(location.type_id);
	}
	batch: ^Constraint_Batch = &solver.active_set.batches.memory[location.batch_index];
	type_index: i16 = batch.type_id_to_batch_index[location.type_id];
	bundle: i32 = location.index_in_type_batch/util.PRODUCTION_LANE_COUNT;
	first: int = int(batch.work_block_start);
	end: int = first+int(batch.work_block_count);
	for first < end
	{
		middle: int = first+(end-first)/2;
		block: ^Solver_Work_Block = &solver.work_blocks.memory[middle];
		if block.type_batch_index < type_index ||
		(block.type_batch_index == type_index && block.end_bundle <= bundle)
		{
			first = middle+1;
		}
		else
		{
			end = middle;
		}
	}
	if first < int(batch.work_block_start+batch.work_block_count)
	{
		block: ^Solver_Work_Block = &solver.work_blocks.memory[first];
		if block.type_batch_index == type_index && bundle >= block.start_bundle && bundle < block.end_bundle
		{
			return first;
		}
	}
	return -1;
}

restitution_prepare_blocks :: proc (storage: ^Restitution_Storage) -> Physics_Status
{
	solver: ^Solver = &storage.simulation.solver;
	synchronized: int = min(int(solver.active_set.batch_count), int(solver.fallback_batch_index));
	storage.normal_work_count = 0;
	if synchronized > 0
	{
		last: ^Constraint_Batch = &solver.active_set.batches.memory[synchronized-1];
		storage.normal_work_count = int(last.work_block_start+last.work_block_count);
	}
	block_count: int = storage.normal_work_count;
	if solver.fallback_batch_index < solver.active_set.batch_count
	{
		block_count += int(CONTACT_4_NONCONVEX_TYPE_ID)+1;
	}
	status: Physics_Status = physics_ensure_buffer_capacity(solver.pool, &storage.blocks, block_count, 0);
	if status != .Ok
	{
		return status;
	}
	for index in 0 ..< block_count
	{
		storage.blocks.memory[index] = {target_offset=-1, bundle_offset=-1};
	}
	pending: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	for index in 0 ..< pending.count
	{
		pair: ^Restitution_Pair_State = &pending.values.memory[index];
		location: Constraint_Location = solver.handle_to_constraint.memory[pair.constraint.value];
		if location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			continue;
		}
		block_index: int = restitution_pair_block(storage, location);
		if block_index < 0
		{
			return .Invalid_Description;
		}
		storage.blocks.memory[block_index].target_offset = -2;
	}
	bundle_count, target_count: int = 0, 0;
	for index in 0 ..< block_count
	{
		if storage.blocks.memory[index].target_offset != -2
		{
			continue;
		}
		block: Solver_Work_Block = restitution_block_description(storage, index);
		type_batch: ^Type_Batch = &solver.active_set.batches.memory[block.batch_index].type_batches.memory[block.type_batch_index];
		accessor: ^Contact_Constraint_Accessor_Record = &storage.simulation.narrow_phase.accessors.records[type_batch.type_id];
		count: int = int(block.end_bundle-block.start_bundle);
		if count > ((1<<util.MAXIMUM_SPAN_SIZE_POWER)/size_of(Restitution_Target_Wide)-target_count)/int(accessor.contact_count) ||
		count > (1<<util.MAXIMUM_SPAN_SIZE_POWER)/size_of(Restitution_Bundle)-bundle_count
		{
			return .Capacity_Missing;
		}
		storage.blocks.memory[index] = {target_offset=i32(target_count), bundle_offset=i32(bundle_count)};
		bundle_count += count;
		target_count += count*int(accessor.contact_count);
	}
	status = physics_ensure_buffer_capacity(solver.pool, &storage.bundles, bundle_count, 0);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(solver.pool, &storage.targets, target_count, 0);
	if status != .Ok
	{
		return status;
	}
	for index in 0 ..< bundle_count
	{
		storage.bundles.memory[index] = {pair_indices={-1, -1, -1, -1, -1, -1, -1, -1}};
	}
	for index in 0 ..< pending.count
	{
		pair: ^Restitution_Pair_State = &pending.values.memory[index];
		location: Constraint_Location = solver.handle_to_constraint.memory[pair.constraint.value];
		if location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			continue;
		}
		block_index: int = restitution_pair_block(storage, location);
		block: Solver_Work_Block = restitution_block_description(storage, block_index);
		bundle_index: int = int(storage.blocks.memory[block_index].bundle_offset)+int(location.index_in_type_batch)/util.PRODUCTION_LANE_COUNT-int(block.start_bundle);
		lane: int = int(location.index_in_type_batch)%util.PRODUCTION_LANE_COUNT;
		bundle: ^Restitution_Bundle = &storage.bundles.memory[bundle_index];
		bundle.pair_indices[lane] = i32(index);
		bundle.coefficient = simd.replace(bundle.coefficient, lane, pair.settings.coefficient);
		bundle.threshold = simd.replace(bundle.threshold, lane, pair.settings.threshold);
	}
	return .Ok;
}

// all synchronized blocks within a batch have disjoint dynamic bodies. fallback
// calls this on the owner thread. the precomputed responsibility masks select
// exactly one occurrence of every constrained body, including joint-owned ones
restitution_integrate_work_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block, worker_index: int,
) -> Physics_Status
{
	solver: ^Solver = job.solver;
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		first: int = bundle_index*util.PRODUCTION_LANE_COUNT;
		active_mask: util.I32x8 = solver_active_mask(first, int(type_batch.count));
		references: [^]util.I32x8 = type_batch_body_bundle(type_batch, first);
		for body_index in 0 ..< int(type_batch.body_count)
		{
			mode: Solver_Bundle_Integration_Mode = .All;
			mask: util.I32x8 = active_mask;
			if block.batch_index != 0
			{
				mode, mask = solver_integration_mask(solver, type_batch, body_index, bundle_index, active_mask);
			}
			if mode == .None
			{
				continue;
			}
			encoded: util.I32x8 = solver_mask_unused_body_lanes(references[body_index], first, int(type_batch.count));
			mask = solver_filter_integration_mobility(encoded, mask, .Dynamic);
			if solver_mask_has_lanes(mask) == .Missing
			{
				continue;
			}
			position: util.Vector3_Wide;
			orientation: util.Quaternion_Wide;
			velocity: Body_Velocity_Wide;
			inertia: Body_Inertia_Wide;
			position, orientation, velocity, inertia = bodies_gather_active_trusted(solver.bodies, encoded, .Local, BODY_ACCESS_ALL);
			_ = solver_integrate_constraint_body(solver, encoded, mask, job.dt, job.phase, worker_index,
				&position, &orientation, &velocity, inertia);
		}
	}
	return .Ok;
}

restitution_capture_contact :: proc "contextless" (
	storage: ^Restitution_Storage, settings: ^Restitution_Bundle, target: ^Restitution_Target_Wide,
	contact_index: int, velocity, depth: util.F32x8,
)
{
	eligible: [util.PRODUCTION_LANE_COUNT]i32;
	incoming: util.F32x8 = velocity;
	pending: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		index: i32 = settings.pair_indices[lane];
		if index < 0
		{
			continue;
		}
		pair: ^Restitution_Pair_State = &pending.values.memory[index];
		normal_velocity: f32 = simd.extract(velocity, lane);
		// geometric separation or departure rearms a later impact. an unchanged
		// resting contact must not bounce repeatedly, even at threshold zero
		if normal_velocity > 0
		{
			pair.phases[contact_index] = .Armed;
			pair.approach[contact_index] = 0;
		}
		else
		{
			if simd.extract(depth, lane) < 0
			{
				pair.phases[contact_index] = .Armed;
			}
			if pair.phases[contact_index] == .Armed
			{
				// speculative contacts may consume approach speed before depth
				// reaches zero. preserve that pre-warmstart speed, not a solved
				// velocity weakened by the intervening speculative impulse
				pair.approach[contact_index] = min(pair.approach[contact_index], normal_velocity);
				incoming = simd.replace(incoming, lane, pair.approach[contact_index]);
				eligible[lane] = -1;
			}
		}
	}
	target^ = restitution_capture_target(incoming, depth, settings.coefficient, settings.threshold, transmute(util.I32x8)eligible);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if simd.extract(target.active_mask, lane) != 0
		{
			pair: ^Restitution_Pair_State = &pending.values.memory[settings.pair_indices[lane]];
			pair.phases[contact_index] = .Spent;
			pair.approach[contact_index] = 0;
		}
	}
}

restitution_capture_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block, index: int,
	$body_count, $contact_count: int, $kind: Contact_Constraint_Kind,
) -> Physics_Status
{
	storage: ^Restitution_Storage = job.restitution;
	output: Restitution_Block = storage.blocks.memory[index];
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		first: int = bundle_index*util.PRODUCTION_LANE_COUNT;
		local_bundle: int = bundle_index-int(block.start_bundle);
		settings: ^Restitution_Bundle = &storage.bundles.memory[int(output.bundle_offset)+local_bundle];
		targets: [^]Restitution_Target_Wide = &storage.targets.memory[int(output.target_offset)+local_bundle*contact_count];
		references: [^]util.I32x8 = type_batch_body_bundle(type_batch, first);
		bodies: [body_count]Constraint_Contact_Body_Wide = ---;
		for body_index in 0 ..< body_count
		{
			encoded: util.I32x8 = solver_mask_unused_body_lanes(references[body_index], first, int(type_batch.count));
			velocity: Body_Velocity_Wide;
			bodies_gather_velocity_kernel(job.solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory, encoded, &velocity);
			bodies[body_index].linear_velocity = velocity.linear;
			bodies[body_index].angular_velocity = velocity.angular;
		}
		prestep: rawptr = type_batch_prestep_bundle(type_batch, first);
		offset_b: util.Vector3_Wide;
		when kind == .Convex
		{
			contacts: [^]Convex_Contact_Wide = ([^]Convex_Contact_Wide)(prestep);
			cursor: uintptr = uintptr(prestep)+uintptr(contact_count*size_of(Convex_Contact_Wide));
			when body_count == 2
			{
				offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
				cursor += size_of(util.Vector3_Wide);
			}
			normal: util.Vector3_Wide = (^util.Vector3_Wide)(rawptr(cursor))^;
			for contact_index in 0 ..< contact_count
			{
				contact: ^Convex_Contact_Wide = &contacts[contact_index];
				velocity: util.F32x8 = restitution_normal_velocity(&bodies[0], normal, contact.offset_a,
					util.vector3_wide_subtract(contact.offset_a, offset_b), body_count);
				restitution_capture_contact(storage, settings, &targets[contact_index], contact_index, velocity, contact.depth);
			}
		}
		else
		{
			cursor: uintptr = uintptr(prestep)+size_of(Contact_Material_Wide);
			when body_count == 2
			{
				offset_b = (^util.Vector3_Wide)(rawptr(cursor))^;
				cursor += size_of(util.Vector3_Wide);
			}
			contacts: [^]Nonconvex_Contact_Wide = ([^]Nonconvex_Contact_Wide)(rawptr(cursor));
			for contact_index in 0 ..< contact_count
			{
				contact: ^Nonconvex_Contact_Wide = &contacts[contact_index];
				velocity: util.F32x8 = restitution_normal_velocity(&bodies[0], contact.normal, contact.offset,
					util.vector3_wide_subtract(contact.offset, offset_b), body_count);
				restitution_capture_contact(storage, settings, &targets[contact_index], contact_index, velocity, contact.depth);
			}
		}
	}
	return .Ok;
}

restitution_execute_contact_bundle :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, bundle_index: int,
	stage: Solver_Execution_Stage, active_mask: util.I32x8,
	targets: [^]Restitution_Target_Wide, impulses: rawptr,
	$body_count, $contact_count: int, $kind: Contact_Constraint_Kind,
)
{
	first: int = bundle_index*util.PRODUCTION_LANE_COUNT;
	references: [^]util.I32x8 = type_batch_body_bundle(type_batch, first);
	bodies: [4]Constraint_Kernel_Body_Wide = ---;
	encoded: [body_count]util.I32x8;
	for body_index in 0 ..< body_count
	{
		// mask nonexecuting lanes, including shared fallback references, before
		// gathering or scattering. only velocities/inertia are read by contacts
		encoded[body_index] = simd.select(transmute(util.Mask32x8)active_mask, references[body_index], util.I32x8(i32(BODY_REFERENCE_KINEMATIC_MASK)));
		velocity: Body_Velocity_Wide;
		inertia: Body_Inertia_Wide;
		bodies_gather_active_no_pose_trusted(job.solver.bodies, encoded[body_index], .World, &velocity, &inertia);
		bodies[body_index].linear_velocity = velocity.linear;
		bodies[body_index].angular_velocity = velocity.angular;
		bodies[body_index].inverse_mass = inertia.inverse_mass;
		bodies[body_index].inverse_inertia = inertia.inverse_inertia_tensor;
	}
	prestep: rawptr = type_batch_prestep_bundle(type_batch, first);
	when kind == .Convex
	{
		if stage == .Warmstart
		{
			constraint_contact_convex_kernel(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Prestep, body_count, contact_count);
			constraint_contact_convex_kernel(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Warmstart, body_count, contact_count);
		}
		else
		{
			constraint_contact_convex_kernel_response(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Solve, body_count, contact_count, targets, .Restitution);
		}
	}
	else
	{
		if stage == .Warmstart
		{
			constraint_contact_nonconvex_kernel(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Prestep, body_count, contact_count);
			constraint_contact_nonconvex_kernel(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Warmstart, body_count, contact_count);
		}
		else
		{
			constraint_contact_nonconvex_kernel_response(prestep, &bodies[0], job.dt, job.inverse_dt, impulses, active_mask, .Solve, body_count, contact_count, targets, .Restitution);
		}
	}
	for body_index in 0 ..< body_count
	{
		velocity: Body_Velocity_Wide = {linear=bodies[body_index].linear_velocity, angular=bodies[body_index].angular_velocity};
		bodies_scatter_active_velocities_trusted(job.solver.bodies, encoded[body_index], velocity, BODY_ACCESS_NO_POSE);
	}
}

restitution_execute_contact_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index, index: int,
	$body_count, $contact_count: int, $kind: Contact_Constraint_Kind,
) -> Physics_Status
{
	targets: [^]Restitution_Target_Wide = &job.restitution.targets.memory[job.restitution.blocks.memory[index].target_offset];
	when kind == .Convex
	{
		if index < job.restitution.normal_work_count && job.velocity_iteration_count > 1
		{
			offset: i32 = job.solver.contact_coefficient_offsets.memory[index];
			when contact_count == 4
			{
				data: [^]Contact_4_Solve_Data = ([^]Contact_4_Solve_Data)(&job.solver.contact_coefficients.memory[offset]);
				if stage == .Warmstart
				{
					return solver_execute_convex_contact_work_block_stage_cached_response(job, type_batch, block, worker_index,
						Solver_Execution_Stage.Warmstart, body_count, contact_count, Solver_Integration_Mode.Never, data, targets, .Restitution);
				}
				return solver_execute_convex_contact_work_block_stage_cached_response(job, type_batch, block, worker_index,
					Solver_Execution_Stage.Solve, body_count, contact_count, Solver_Integration_Mode.Never, data, targets, .Restitution);
			}
			else
			{
				data: [^]Contact_Convex_Solve_Data(contact_count) = ([^]Contact_Convex_Solve_Data(contact_count))(&job.solver.contact_coefficients.memory[offset]);
				if stage == .Warmstart
				{
					return solver_execute_convex_contact_work_block_stage_cached_response(job, type_batch, block, worker_index,
						Solver_Execution_Stage.Warmstart, body_count, contact_count, Solver_Integration_Mode.Never, data, targets, .Restitution);
				}
				return solver_execute_convex_contact_work_block_stage_cached_response(job, type_batch, block, worker_index,
					Solver_Execution_Stage.Solve, body_count, contact_count, Solver_Integration_Mode.Never, data, targets, .Restitution);
			}
		}
	}
	for bundle in int(block.start_bundle) ..< int(block.end_bundle)
	{
		first: int = bundle*util.PRODUCTION_LANE_COUNT;
		bundle_targets: [^]Restitution_Target_Wide = &targets[(bundle-int(block.start_bundle))*contact_count];
		impulses: rawptr = type_batch_impulse_bundle(type_batch, first);
		if index < job.restitution.normal_work_count
		{
			restitution_execute_contact_bundle(job, type_batch, bundle, stage, solver_active_mask(first, int(type_batch.count)),
				bundle_targets, impulses, body_count, contact_count, kind);
		}
		else
		{
			// the existing vector kernels mask nonexecuting impulse lanes. use a
			// bounded lane-local result and publish only its scalar lane so other
			// fallback constraints keep their warmstart/iteration history
			impulse_count :: contact_count+3 when kind == .Convex else contact_count*3;
			for lane in 0 ..< min(util.PRODUCTION_LANE_COUNT, int(type_batch.count)-first)
			{
				local_impulses: [impulse_count]util.F32x8 = ---;
				for scalar in 0 ..< impulse_count
				{
					local_impulses[scalar] = ([^]util.F32x8)(impulses)[scalar];
				}
				mask: util.I32x8 = simd.replace(util.I32x8(0), lane, i32(-1));
				restitution_execute_contact_bundle(job, type_batch, bundle, stage, mask, bundle_targets, &local_impulses[0], body_count, contact_count, kind);
				for scalar in 0 ..< impulse_count
				{
					([^]f32)(impulses)[scalar*util.PRODUCTION_LANE_COUNT+lane] = simd.extract(local_impulses[scalar], lane);
				}
			}
		}
	}
	return .Ok;
}

restitution_capture_work_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
) -> Physics_Status
{
	index: int = int((uintptr(block)-uintptr(job.solver.work_blocks.memory))/size_of(Solver_Work_Block));
	return restitution_capture_selected_block(job, type_batch, block, index);
}

restitution_capture_selected_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block, index: int,
) -> Physics_Status
{
	if job.restitution.blocks.memory[index].target_offset < 0
	{
		return .Ok;
	}
	switch type_batch.type_id
	{
		case CONTACT_1_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 1, .Convex);
		case CONTACT_2_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 2, .Convex);
		case CONTACT_3_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 3, .Convex);
		case CONTACT_4_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 4, .Convex);
		case CONTACT_1_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 1, .Convex);
		case CONTACT_2_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 2, .Convex);
		case CONTACT_3_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 3, .Convex);
		case CONTACT_4_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 4, .Convex);
		case CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 2, .Nonconvex);
		case CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 3, .Nonconvex);
		case CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 1, 4, .Nonconvex);
		case CONTACT_2_NONCONVEX_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 2, .Nonconvex);
		case CONTACT_3_NONCONVEX_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 3, .Nonconvex);
		case CONTACT_4_NONCONVEX_TYPE_ID:
		return restitution_capture_block(job, type_batch, block, index, 2, 4, .Nonconvex);
	}
	return .Invalid_Description;
}

restitution_execute_work_block :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index, index: int,
) -> Physics_Status
{
	switch type_batch.type_id
	{
		case CONTACT_1_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 1, .Convex);
		case CONTACT_2_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 2, .Convex);
		case CONTACT_3_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 3, .Convex);
		case CONTACT_4_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 4, .Convex);
		case CONTACT_1_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 1, .Convex);
		case CONTACT_2_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 2, .Convex);
		case CONTACT_3_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 3, .Convex);
		case CONTACT_4_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 4, .Convex);
		case CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 2, .Nonconvex);
		case CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 3, .Nonconvex);
		case CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 1, 4, .Nonconvex);
		case CONTACT_2_NONCONVEX_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 2, .Nonconvex);
		case CONTACT_3_NONCONVEX_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 3, .Nonconvex);
		case CONTACT_4_NONCONVEX_TYPE_ID:
		return restitution_execute_contact_block(job, type_batch, block, stage, worker_index, index, 2, 4, .Nonconvex);
	}
	return .Invalid_Description;
}

restitution_execute_fallback_type :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, stage: Solver_Execution_Stage,
) -> Physics_Status
{
	block: Solver_Work_Block = {batch_index=i16(job.solver.fallback_batch_index),
		end_bundle=(type_batch.count+util.PRODUCTION_LANE_COUNT-1)/util.PRODUCTION_LANE_COUNT};
	if stage == .Integrate_Constrained_Dynamics
	{
		return restitution_integrate_work_block(job, type_batch, &block, 0);
	}
	if type_batch.type_id <= CONTACT_4_NONCONVEX_TYPE_ID
	{
		index: int = job.restitution.normal_work_count+int(type_batch.type_id);
		if stage == .Capture_Restitution
		{
			return restitution_capture_selected_block(job, type_batch, &block, index);
		}
		if (stage == .Warmstart || stage == .Solve) && job.restitution.blocks.memory[index].target_offset >= 0
		{
			return restitution_execute_work_block(job, type_batch, &block, stage, 0, index);
		}
	}
	if stage == .Capture_Restitution
	{
		return .Ok;
	}
	return solver_execute_type_batch_integration(job.solver, type_batch, int(job.solver.fallback_batch_index),
		job.dt, job.inverse_dt, job.phase, stage, .Present, .Never);
}

solver_solve_restitution_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job: ^Solver_Substep_Job = (^Solver_Substep_Job)(dispatcher.unmanaged_context);
	if worker_index == 0
	{
		solver_solve_main_response(job, .Restitution, .Missing);
		return;
	}
	solver_solve_background_response(job, worker_index, .Restitution);
}

// this optional boundary owns preparation and success-only history publication.
// a standalone manual solve commits on success. a full/custom timestep defers
// publication until the timestep callback has successfully returned
restitution_solve :: #force_no_inline proc (
	storage: ^Restitution_Storage, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	simulation: ^Simulation = storage.simulation;
	mode: Restitution_Staging_Mode = .Initial;
	if storage.state == .Prepared
	{
		mode = .Continuation;
	}
	status: Physics_Status = restitution_storage_prepare_internal(storage, mode);
	if status != .Ok
	{
		return status;
	}
	active: Reference_State = .Missing;
	pending: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	for index in 0 ..< pending.count
	{
		if simulation.solver.handle_to_constraint.memory[pending.values.memory[index].constraint.value].set_index == BODIES_ACTIVE_SET_INDEX
		{
			active = .Present;
			break;
		}
	}
	if active == .Present
	{
		status = solver_solve_internal_response(&simulation.solver, dt, &simulation.solve_description,
			.Present, dispatcher, storage, .Restitution);
	}
	else
	{
		status = solver_solve_and_integrate(&simulation.solver, dt, &simulation.solve_description, dispatcher);
	}
	if simulation.state == .Ready || status != .Ok
	{
		restitution_step_complete(storage, status);
	}
	return status;
}

restitution_step_complete :: #force_no_inline proc (
	storage: ^Restitution_Storage, status: Physics_Status,
)
{
	if storage.state == .Prepared
	{
		_ = restitution_storage_complete(storage, status);
	}
}
