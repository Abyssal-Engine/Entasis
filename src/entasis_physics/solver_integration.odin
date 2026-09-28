package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"
import "base:runtime"
import "core:simd"

SOLVER_INTEGRATION_REGION_CONSTRAINT_COUNT :: 2048;
SOLVER_INTEGRATION_PARALLEL_BASE_THRESHOLD :: 4096;
SOLVER_INTEGRATION_PARALLEL_PER_WORKER_THRESHOLD :: 1024;
SOLVER_INTEGRATION_OWNER_NONE :: max(u64);
SOLVER_INTEGRATION_OWNER_EXACT_BIT :: u64(1) << 63;
SOLVER_INTEGRATION_OWNER_BATCH_SHIFT :: 48;
SOLVER_INTEGRATION_OWNER_TYPE_BATCH_SHIFT :: 32;
SOLVER_INTEGRATION_OWNER_BATCH_MASK :: u64(0x7fff);
solver_prepare_integration_responsibilities_serial :: proc (
	solver: ^Solver,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready || solver.bodies == nil ||
		solver.bodies.state != .Allocated
	{
		return .Invalid_Argument;
	}
	active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	seen_word_count := util.index_set_bundle_capacity(int(active.count));
	if seen_word_count > int(solver.integration_body_seen.length)
	{
		return .Capacity_Missing;
	}
	if seen_word_count > 0
	{
		_ = util.buffer_clear(solver.integration_body_seen, 0, seen_word_count);
	}
	flag_word_cursor := 0;
	for batch_index in 0 ..< int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			type_batch.integration_flags_offset = 0;
			type_batch.integration_flag_word_count = 0;
			type_batch.has_integration_responsibilities = .Missing;
			if type_batch.count <= 0
			{
				continue;
			}
			word_count := util.index_set_bundle_capacity(int(type_batch.count));
			required_word_count := word_count * int(type_batch.body_count);
			if required_word_count <= 0 ||
				flag_word_cursor > int(solver.integration_flags.length) - required_word_count
			{
				return .Capacity_Missing;
			}
			type_batch.integration_flags_offset = i32(flag_word_cursor);
			type_batch.integration_flag_word_count = i32(word_count);
			_ = util.buffer_clear(
				solver.integration_flags, flag_word_cursor, required_word_count,
			);
			flag_word_cursor += required_word_count;
		}
	}
	for batch_index in 0 ..< int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			if type_batch.count <= 0
			{
				continue;
			}
			word_count := int(type_batch.integration_flag_word_count);
			flag_offset := int(type_batch.integration_flags_offset);
			bundle_count :=
				(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
				util.PRODUCTION_LANE_COUNT;
			for bundle_index in 0 ..< bundle_count
			{
				first_constraint_index := bundle_index * util.PRODUCTION_LANE_COUNT;
				count_in_bundle := min(
					util.PRODUCTION_LANE_COUNT,
					int(type_batch.count) - first_constraint_index,
				);
				body_bundles := type_batch_body_bundle(
					type_batch, first_constraint_index,
				);
				for lane in 0 ..< count_in_bundle
				{
					constraint_index := first_constraint_index + lane;
					for body_index_in_constraint in 0 ..< int(type_batch.body_count)
					{
						encoded_body_index := simd.extract(
							body_bundles[body_index_in_constraint], lane,
						);
						if encoded_body_index == -1
						{
							continue;
						}
						body_index := int(
							u32(encoded_body_index) & BODY_REFERENCE_INDEX_MASK,
						);
						seen_word_index := body_index >> util.INDEX_SET_SHIFT;
						seen_bit := u64(1) << uint(body_index & util.INDEX_SET_MASK);
						if solver.integration_body_seen.memory[seen_word_index] & seen_bit != 0
						{
							continue;
						}
						solver.integration_body_seen.memory[seen_word_index] |= seen_bit;
						flag_word_index :=
							flag_offset + body_index_in_constraint * word_count +
							(constraint_index >> util.INDEX_SET_SHIFT);
						solver.integration_flags.memory[flag_word_index] |=
							u64(1) << uint(constraint_index & util.INDEX_SET_MASK);
						type_batch.has_integration_responsibilities = .Present;
					}
				}
			}
		}
	}
	return .Ok;
}
solver_integration_owner_batch :: #force_inline proc "contextless" (owner: u64) -> int
{
	return int((owner >> SOLVER_INTEGRATION_OWNER_BATCH_SHIFT) & SOLVER_INTEGRATION_OWNER_BATCH_MASK);
}
solver_integration_batch_owner :: #force_inline proc "contextless" (batch_index: int) -> u64
{
	return u64(u16(batch_index)) << SOLVER_INTEGRATION_OWNER_BATCH_SHIFT;
}
solver_integration_exact_owner :: #force_inline proc "contextless" (
	batch_index, type_batch_index, constraint_index: int,
) -> u64
{
	return SOLVER_INTEGRATION_OWNER_EXACT_BIT |
		u64(u16(batch_index)) << SOLVER_INTEGRATION_OWNER_BATCH_SHIFT |
		u64(u16(type_batch_index)) << SOLVER_INTEGRATION_OWNER_TYPE_BATCH_SHIFT |
		u64(u32(constraint_index));
}
solver_find_exact_integration_owner :: proc "contextless" (
	solver: ^Solver, body_index: int,
) -> u64
{
	active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	constraints := &active.constraints.memory[body_index];
	owner := SOLVER_INTEGRATION_OWNER_NONE;
	for reference_index in 0 ..< constraints.count
	{
		reference := constraints.span.memory[reference_index];
		location := solver.handle_to_constraint.memory[
			reference.connecting_constraint_handle.value
		];
		batch := &solver.active_set.batches.memory[location.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
		candidate := solver_integration_exact_owner(
			int(location.batch_index), type_batch_index,
			int(location.index_in_type_batch),
		);
		if candidate < owner
		{
			owner = candidate;
		}
	}
	return owner;
}
solver_integration_parallel_constraint_count :: proc "contextless" (
	solver: ^Solver,
) -> int
{
	if solver == nil || solver.state != .Ready
	{
		return 0;
	}
	synchronized_batch_count := min(
		int(solver.active_set.batch_count), int(solver.fallback_batch_index),
	);
	constraint_count := 0;
	for batch_index in 1 ..< synchronized_batch_count
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			constraint_count += int(batch.type_batches.memory[type_batch_index].count);
		}
	}
	return constraint_count;
}
solver_prepare_integration_parallel_layout_and_owners :: proc (
	solver: ^Solver,
) -> (Physics_Status, int)
{
	if solver == nil || solver.state != .Ready || solver.bodies == nil ||
		solver.bodies.state != .Allocated
	{
		return .Invalid_Argument, 0;
	}
	active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	owner_word_count := int(active.count);
	required_word_count := owner_word_count;
	for batch_index in 0 ..< int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			if type_batch.count <= 0
			{
				continue;
			}
			word_count := util.index_set_bundle_capacity(int(type_batch.count));
			required_word_count += word_count * int(type_batch.body_count);
		}
	}
	if required_word_count <= 0 ||
		required_word_count > int(solver.integration_flags.length)
	{
		return .Capacity_Missing, 0;
	}
	for body_index in 0 ..< active.count
	{
		solver.integration_flags.memory[body_index] = SOLVER_INTEGRATION_OWNER_NONE;
	}
	flag_word_cursor := owner_word_count;
	synchronized_batch_count := min(
		int(solver.active_set.batch_count), int(solver.fallback_batch_index),
	);
	parallel_constraint_count := 0;
	for batch_index in 0 ..< int(solver.active_set.batch_count)
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			type_batch.integration_flags_offset = 0;
			type_batch.integration_flag_word_count = 0;
			type_batch.has_integration_responsibilities = .Missing;
			if type_batch.count <= 0
			{
				continue;
			}
			word_count := util.index_set_bundle_capacity(int(type_batch.count));
			type_batch.integration_flags_offset = i32(flag_word_cursor);
			type_batch.integration_flag_word_count = i32(word_count);
			clear_count := word_count * int(type_batch.body_count);
			_ = util.buffer_clear(
				solver.integration_flags, flag_word_cursor, clear_count,
			);
			flag_word_cursor += clear_count;
			if batch_index > 0 && batch_index < synchronized_batch_count
			{
				parallel_constraint_count += int(type_batch.count);
			}
		}
	}
	fallback_exists :=
		int(solver.fallback_batch_index) < int(solver.active_set.batch_count);
	for body_index in 0 ..< active.count
	{
		dynamics := &active.dynamics_state.memory[body_index];
		if body_inertia_mobility(dynamics.inertia.local) == .Kinematic
		{
			solver.integration_flags.memory[body_index] =
				solver_find_exact_integration_owner(solver, body_index);
			continue;
		}
		handle := active.index_to_handle.memory[body_index].value;
		first_batch_index := -1;
		for batch_index in 0 ..< synchronized_batch_count
		{
			batch := &solver.active_set.batches.memory[batch_index];
			if batch.body_reference_counts.memory[handle] > 0
			{
				first_batch_index = batch_index;
				break;
			}
		}
		if first_batch_index > 0
		{
			solver.integration_flags.memory[body_index] =
				solver_integration_batch_owner(first_batch_index);
		}
		else if first_batch_index < 0 && fallback_exists
		{
			fallback := &solver.active_set.batches.memory[solver.fallback_batch_index];
			if fallback.body_reference_counts.memory[body_index] > 0
			{
				solver.integration_flags.memory[body_index] =
					solver_find_exact_integration_owner(solver, body_index);
			}
		}
	}
	return .Ok, parallel_constraint_count;
}
solver_prepare_integration_region_blocks :: proc "contextless" (
	solver: ^Solver,
) -> (Physics_Status, int)
{
	if solver == nil || solver.state != .Ready
	{
		return .Invalid_Argument, 0;
	}
	context = runtime.default_context();
	required_work_count := 0;
	synchronized_batch_count := min(
		int(solver.active_set.batch_count), int(solver.fallback_batch_index),
	);
	for batch_index in 1 ..< synchronized_batch_count
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			constraint_count := int(type_batch.count);
			if constraint_count <= 0
			{
				continue;
			}
			region_count :=
				(constraint_count + SOLVER_INTEGRATION_REGION_CONSTRAINT_COUNT - 1) /
				SOLVER_INTEGRATION_REGION_CONSTRAINT_COUNT;
			if required_work_count > max(int) - region_count
			{
				return .Capacity_Missing, 0;
			}
			required_work_count += region_count;
		}
	}
	if required_work_count > int(solver.work_blocks.length)
	{
		capacity_status := physics_ensure_buffer_capacity(
			solver.pool, &solver.work_blocks, required_work_count, 0,
		);
		if capacity_status != .Ok
		{
			return capacity_status, 0;
		}
	}
	work_count := 0;
	for batch_index in 1 ..< synchronized_batch_count
	{
		batch := &solver.active_set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			constraint_count := int(type_batch.count);
			for constraint_start := 0;
				constraint_start < constraint_count;
				constraint_start += SOLVER_INTEGRATION_REGION_CONSTRAINT_COUNT
			{
				if work_count >= int(solver.work_blocks.length)
				{
					return .Capacity_Missing, 0;
				}
				constraint_end := min(
					constraint_start + SOLVER_INTEGRATION_REGION_CONSTRAINT_COUNT,
					constraint_count,
				);
				solver.work_blocks.memory[work_count] = {
					batch_index=i16(batch_index),
					type_batch_index=i16(type_batch_index),
					start_bundle=i32(constraint_start / util.PRODUCTION_LANE_COUNT),
					end_bundle=i32(
						(constraint_end + util.PRODUCTION_LANE_COUNT - 1) /
						util.PRODUCTION_LANE_COUNT,
					),
					claim_generation=0,
				};
				work_count += 1;
			}
		}
	}
	return .Ok, work_count;
}
solver_prepare_integration_region_block :: proc "contextless" (
	solver: ^Solver, block: ^Solver_Work_Block,
) -> Physics_Status
{
	batch_index := int(block.batch_index);
	type_batch_index := int(block.type_batch_index);
	batch := &solver.active_set.batches.memory[batch_index];
	type_batch := &batch.type_batches.memory[type_batch_index];
	word_count := int(type_batch.integration_flag_word_count);
	flag_offset := int(type_batch.integration_flags_offset);
	has_responsibility := false;
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		first_constraint_index := bundle_index * util.PRODUCTION_LANE_COUNT;
		count_in_bundle := min(
			util.PRODUCTION_LANE_COUNT,
			int(type_batch.count) - first_constraint_index,
		);
		body_bundles := type_batch_body_bundle(type_batch, first_constraint_index);
		for lane in 0 ..< count_in_bundle
		{
			constraint_index := first_constraint_index + lane;
			for body_index_in_constraint in 0 ..< int(type_batch.body_count)
			{
				encoded_body_index := simd.extract(
					body_bundles[body_index_in_constraint], lane,
				);
				if encoded_body_index == -1 ||
					u32(encoded_body_index) >= BODY_REFERENCE_KINEMATIC_MASK
				{
					continue;
				}
				body_index := int(
					u32(encoded_body_index) & BODY_REFERENCE_INDEX_MASK,
				);
				owner := solver.integration_flags.memory[body_index];
				if owner == SOLVER_INTEGRATION_OWNER_NONE ||
					(owner & SOLVER_INTEGRATION_OWNER_EXACT_BIT) != 0 ||
					solver_integration_owner_batch(owner) != batch_index
				{
					continue;
				}
				flag_word_index :=
					flag_offset + body_index_in_constraint * word_count +
					(constraint_index >> util.INDEX_SET_SHIFT);
				solver.integration_flags.memory[flag_word_index] |=
					u64(1) << uint(constraint_index & util.INDEX_SET_MASK);
				has_responsibility = true;
			}
		}
	}
	if has_responsibility
	{
		block.end_bundle = -block.end_bundle;
	}
	return .Ok;
}
solver_prepare_exact_integration_responsibilities :: proc "contextless" (
	solver: ^Solver,
) -> Physics_Status
{
	active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for body_index in 0 ..< active.count
	{
		owner := solver.integration_flags.memory[body_index];
		if owner == SOLVER_INTEGRATION_OWNER_NONE ||
			(owner & SOLVER_INTEGRATION_OWNER_EXACT_BIT) == 0
		{
			continue;
		}
		batch_index := int(
			(owner >> SOLVER_INTEGRATION_OWNER_BATCH_SHIFT) &
			SOLVER_INTEGRATION_OWNER_BATCH_MASK,
		);
		type_batch_index := int(u16(owner >> SOLVER_INTEGRATION_OWNER_TYPE_BATCH_SHIFT));
		constraint_index := int(u32(owner));
		batch := &solver.active_set.batches.memory[batch_index];
		if type_batch_index < 0 || type_batch_index >= int(batch.type_batch_count)
		{
			return .Invalid_Description;
		}
		type_batch := &batch.type_batches.memory[type_batch_index];
		constraints := &active.constraints.memory[body_index];
		body_index_in_constraint := -1;
		for reference_index in 0 ..< constraints.count
		{
			reference := constraints.span.memory[reference_index];
			location := solver.handle_to_constraint.memory[
				reference.connecting_constraint_handle.value
			];
			if int(location.batch_index) == batch_index &&
				int(batch.type_id_to_batch_index[location.type_id]) == type_batch_index &&
				int(location.index_in_type_batch) == constraint_index
			{
				body_index_in_constraint = int(reference.body_index_in_constraint);
				break;
			}
		}
		if body_index_in_constraint < 0 ||
			body_index_in_constraint >= int(type_batch.body_count)
		{
			return .Invalid_Description;
		}
		word_count := int(type_batch.integration_flag_word_count);
		flag_word_index := int(type_batch.integration_flags_offset) +
			body_index_in_constraint * word_count +
			(constraint_index >> util.INDEX_SET_SHIFT);
		solver.integration_flags.memory[flag_word_index] |=
			u64(1) << uint(constraint_index & util.INDEX_SET_MASK);
		type_batch.has_integration_responsibilities = .Present;
	}
	return .Ok;
}
solver_finalize_integration_responsibilities :: proc "contextless" (
	solver: ^Solver, work_count: int,
)
{
	for work_index in 0 ..< work_count
	{
		block := &solver.work_blocks.memory[work_index];
		if block.end_bundle >= 0
		{
			continue;
		}
		batch := &solver.active_set.batches.memory[block.batch_index];
		type_batch := &batch.type_batches.memory[block.type_batch_index];
		type_batch.has_integration_responsibilities = .Present;
	}
}
solver_integration_mask :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, body_index, bundle_index: int,
	execution_mask: util.I32x8,
) -> (Solver_Bundle_Integration_Mode, util.I32x8)
{
	if type_batch.has_integration_responsibilities == .Missing ||
		type_batch.integration_flag_word_count <= 0
	{
		return .None, {};
	}
	word_count := int(type_batch.integration_flag_word_count);
	word_index :=
		int(type_batch.integration_flags_offset) + body_index * word_count +
		(bundle_index >> 3);
	shift := uint((bundle_index & 7) * util.PRODUCTION_LANE_COUNT);
	scalar_mask := u8(solver.integration_flags.memory[word_index] >> shift);
	if scalar_mask == 0
	{
		return .None, {};
	}
	if scalar_mask == max(u8)
	{
		return .All, execution_mask;
	}
	values: [util.PRODUCTION_LANE_COUNT]i32;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if scalar_mask & (u8(1) << u8(lane)) != 0
		{
			values[lane] = -1;
		}
	}
	return .Partial, transmute(util.I32x8)values & execution_mask;
}
solver_filter_integration_mobility :: proc "contextless" (
	encoded_references, integration_mask: util.I32x8,
	mobility: Body_Mobility,
) -> util.I32x8
{
	encoded_values := transmute([util.PRODUCTION_LANE_COUNT]i32)encoded_references;
	mask_values := transmute([util.PRODUCTION_LANE_COUNT]i32)integration_mask;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if mask_values[lane] == 0
		{
			continue;
		}
		if mobility == .Kinematic &&
			u32(encoded_values[lane]) < BODY_REFERENCE_KINEMATIC_MASK ||
			mobility == .Dynamic &&
			u32(encoded_values[lane]) >= BODY_REFERENCE_KINEMATIC_MASK
		{
			mask_values[lane] = 0;
		}
	}
	return transmute(util.I32x8)mask_values;
}
solver_mask_has_lanes :: #force_inline proc "contextless" (mask: util.I32x8) -> Reference_State
{
	if simd.reduce_or(mask) != 0
	{
		return .Present;
	}
	return .Missing;
}
solver_integrate_orientation_nonconserving_wide :: #force_inline proc "contextless" (
	start: util.Quaternion_Wide, angular_velocity: util.Vector3_Wide, half_dt: util.F32x8,
) -> util.Quaternion_Wide
{
	speed := util.vector3_wide_length(angular_velocity);
	half_angle := simd.mul(speed, half_dt);
	scale := simd.div(util.sin_approx_wide(half_angle), speed);
	delta := util.Quaternion_Wide{
		x=simd.mul(angular_velocity.x, scale),
		y=simd.mul(angular_velocity.y, scale),
		z=simd.mul(angular_velocity.z, scale),
		w=util.cos_approx_wide(half_angle),
	};
	integrated := util.quaternion_wide_normalize(
		util.quaternion_wide_concatenate(start, delta),
	);
	return util.quaternion_wide_select(
		transmute(util.I32x8)simd.lanes_gt(speed, util.F32x8(1e-15)),
		integrated,
		start,
	);
}
solver_integrate_constraint_body_nonconserving_wide :: #force_inline proc "contextless" (
	solver: ^Solver, encoded_references, integration_mask: util.I32x8,
	dt: f32, phase: Solver_Substep_Phase, worker_index: int,
	position: ^util.Vector3_Wide, orientation: ^util.Quaternion_Wide,
	velocity: ^Body_Velocity_Wide, local_inertia: Body_Inertia_Wide,
) -> Body_Inertia_Wide
{
	decoded :=
		(encoded_references & util.I32x8(i32(BODY_REFERENCE_INDEX_MASK))) |
		~integration_mask;
	if phase == .Continuation
	{
		integrated_position := util.vector3_wide_add(
			position^,
			util.vector3_wide_scale(velocity.linear, util.F32x8(dt)),
		);
		position^ = util.vector3_wide_select(
			integration_mask, integrated_position, position^,
		);
		integrated_orientation := solver_integrate_orientation_nonconserving_wide(
			orientation^, velocity.angular, util.F32x8(dt * 0.5),
		);
		orientation^ = util.quaternion_wide_select(
			integration_mask, integrated_orientation, orientation^,
		);
	}
	world_inertia := Body_Inertia_Wide{
		inverse_inertia_tensor=util.symmetric3x3_wide_rotation_sandwich(
			util.matrix3x3_wide_from_quaternion(orientation^),
			local_inertia.inverse_inertia_tensor,
		),
		inverse_mass=local_inertia.inverse_mass,
	};
	previous_velocity := velocity^;
	solver.integrator.callbacks.integrate_velocity(
		solver.integrator.callbacks.user_context,
		decoded,
		position^,
		orientation^,
		local_inertia,
		integration_mask,
		worker_index,
		util.F32x8(dt),
		velocity,
	);
	velocity.linear = util.vector3_wide_select(
		integration_mask, velocity.linear, previous_velocity.linear,
	);
	velocity.angular = util.vector3_wide_select(
		integration_mask, velocity.angular, previous_velocity.angular,
	);
	bodies_scatter_active_velocities_trusted(
		solver.bodies, decoded, velocity^, BODY_ACCESS_ALL,
	);
	if phase == .Continuation
	{
		bodies_scatter_active_pose_trusted(
			solver.bodies, encoded_references, integration_mask, position^, orientation^,
		);
	}
	bodies_scatter_active_inertia_trusted(
		solver.bodies, encoded_references, integration_mask, world_inertia,
	);
	return world_inertia;
}
solver_integrate_constraint_body :: proc "contextless" (
	solver: ^Solver, encoded_references, integration_mask: util.I32x8,
	dt: f32, phase: Solver_Substep_Phase, worker_index: int,
	position: ^util.Vector3_Wide, orientation: ^util.Quaternion_Wide,
	velocity: ^Body_Velocity_Wide, local_inertia: Body_Inertia_Wide,
) -> Body_Inertia_Wide
{
	if solver.integrator.callbacks.angular_mode == .Nonconserving
	{
		return solver_integrate_constraint_body_nonconserving_wide(
			solver, encoded_references, integration_mask,
			dt, phase, worker_index,
			position, orientation, velocity, local_inertia,
		);
	}
	integrator := solver.integrator;
	mask_values := transmute([util.PRODUCTION_LANE_COUNT]i32)integration_mask;
	encoded_values := transmute([util.PRODUCTION_LANE_COUNT]i32)encoded_references;
	callback_mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	decoded_values := [util.PRODUCTION_LANE_COUNT]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	world_inertia := local_inertia;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if mask_values[lane] == 0
		{
			continue;
		}
		encoded := encoded_values[lane];
		if encoded < 0
		{
			continue;
		}
		index := int(u32(encoded) & BODY_REFERENCE_INDEX_MASK);
		decoded_values[lane] = i32(index);
		local_tensor := util.Symmetric3x3{
			simd.extract(local_inertia.inverse_inertia_tensor.xx, lane),
			simd.extract(local_inertia.inverse_inertia_tensor.yx, lane),
			simd.extract(local_inertia.inverse_inertia_tensor.yy, lane),
			simd.extract(local_inertia.inverse_inertia_tensor.zx, lane),
			simd.extract(local_inertia.inverse_inertia_tensor.zy, lane),
			simd.extract(local_inertia.inverse_inertia_tensor.zz, lane),
		};
		lane_orientation := util.quaternion_wide_read_slot(orientation^, lane);
		lane_velocity := util.vector3_wide_read_slot(velocity.angular, lane);
		if phase == .Continuation
		{
			previous_orientation := lane_orientation;
			lane_position := util.vector3_wide_read_slot(position^, lane);
			lane_linear := util.vector3_wide_read_slot(velocity.linear, lane);
			lane_position = util.vector3_add(lane_position, util.vector3_scale(lane_linear, dt));
			lane_orientation = pose_integrator_integrate_orientation(lane_orientation, lane_velocity, dt);
			if integrator.callbacks.angular_mode == .Conserve_Momentum &&
				simd.extract(local_inertia.inverse_mass, lane) > 0
			{
				previous_inverse := util.symmetric3x3_rotation_sandwich(
					util.matrix3x3_from_quaternion(previous_orientation), local_tensor,
				);
				momentum := util.symmetric3x3_transform(lane_velocity, util.symmetric3x3_invert(previous_inverse));
				current_inverse := util.symmetric3x3_rotation_sandwich(
					util.matrix3x3_from_quaternion(lane_orientation), local_tensor,
				);
				lane_velocity = util.symmetric3x3_transform(momentum, current_inverse);
			}
			else if integrator.callbacks.angular_mode == .Conserve_Momentum_With_Gyroscopic_Torque &&
				simd.extract(local_inertia.inverse_mass, lane) > 0
			{
				pose_integrator_integrate_gyroscopic_velocity(
					lane_orientation, local_tensor, &lane_velocity, dt,
				);
			}
			util.vector3_wide_write_slot(position, lane, lane_position);
			util.quaternion_wide_write_slot(orientation, lane, lane_orientation);
		}
		else if integrator.callbacks.angular_mode == .Conserve_Momentum &&
			simd.extract(local_inertia.inverse_mass, lane) > 0
		{
			previous_orientation := pose_integrator_integrate_orientation(lane_orientation, lane_velocity, -dt);
			previous_inverse := util.symmetric3x3_rotation_sandwich(
				util.matrix3x3_from_quaternion(previous_orientation), local_tensor,
			);
			momentum := util.symmetric3x3_transform(lane_velocity, util.symmetric3x3_invert(previous_inverse));
			current_inverse := util.symmetric3x3_rotation_sandwich(
				util.matrix3x3_from_quaternion(lane_orientation), local_tensor,
			);
			lane_velocity = util.symmetric3x3_transform(momentum, current_inverse);
		}
		else if integrator.callbacks.angular_mode == .Conserve_Momentum_With_Gyroscopic_Torque &&
			simd.extract(local_inertia.inverse_mass, lane) > 0
		{
			pose_integrator_integrate_gyroscopic_velocity(
				lane_orientation, local_tensor, &lane_velocity, dt,
			);
		}
		util.vector3_wide_write_slot(&velocity.angular, lane, lane_velocity);
		rotated := util.symmetric3x3_rotation_sandwich(
			util.matrix3x3_from_quaternion(lane_orientation), local_tensor,
		);
		util.symmetric3x3_wide_write_slot(&world_inertia.inverse_inertia_tensor, lane, rotated);
		if u32(encoded) < BODY_REFERENCE_KINEMATIC_MASK ||
			integrator.callbacks.integrate_kinematic_velocity == .Enabled
		{
			callback_mask_values[lane] = -1;
		}
		else
		{
			decoded_values[lane] = -1;
		}
	}
	callback_mask := transmute(util.I32x8)callback_mask_values;
	if solver_mask_has_lanes(callback_mask) == .Present
	{
		previous_velocity := velocity^;
		integrator.callbacks.integrate_velocity(
			integrator.callbacks.user_context, transmute(util.I32x8)decoded_values,
			position^, orientation^, local_inertia, callback_mask, worker_index, util.F32x8(dt), velocity,
		);
		velocity.linear = util.vector3_wide_select(callback_mask, velocity.linear, previous_velocity.linear);
		velocity.angular = util.vector3_wide_select(callback_mask, velocity.angular, previous_velocity.angular);
		bodies_scatter_active_velocities_trusted(
			solver.bodies, transmute(util.I32x8)decoded_values, velocity^, BODY_ACCESS_ALL,
		);
	}
	if phase == .Continuation
	{
		bodies_scatter_active_pose_trusted(
			solver.bodies, encoded_references, integration_mask, position^, orientation^,
		);
	}
	bodies_scatter_active_inertia_trusted(
		solver.bodies, encoded_references, integration_mask, world_inertia,
	);
	return world_inertia;
}
solver_integrate_kinematic_lanes :: proc "contextless" (
	solver: ^Solver, encoded_references, integration_mask: util.I32x8,
	dt: f32, phase: Solver_Substep_Phase, worker_index: int,
)
{
	integrator := solver.integrator;
	encoded_values := transmute([util.PRODUCTION_LANE_COUNT]i32)encoded_references;
	mask_values := transmute([util.PRODUCTION_LANE_COUNT]i32)integration_mask;
	decoded_values := [util.PRODUCTION_LANE_COUNT]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if mask_values[lane] != 0
		{
			decoded_values[lane] = i32(u32(encoded_values[lane]) & BODY_REFERENCE_INDEX_MASK);
		}
	}
	decoded := transmute(util.I32x8)decoded_values;
	position, orientation, velocity, _ := bodies_gather_active_trusted(
		solver.bodies, encoded_references, .World, BODY_ACCESS_ALL,
	);
	if phase == .Continuation
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if mask_values[lane] == 0
			{
				continue;
			}
			lane_position := util.vector3_wide_read_slot(position, lane);
			lane_velocity := util.vector3_wide_read_slot(velocity.linear, lane);
			lane_orientation := util.quaternion_wide_read_slot(orientation, lane);
			lane_angular := util.vector3_wide_read_slot(velocity.angular, lane);
			lane_position = util.vector3_add(lane_position, util.vector3_scale(lane_velocity, dt));
			lane_orientation = pose_integrator_integrate_orientation(
				lane_orientation, lane_angular, dt,
			);
			util.vector3_wide_write_slot(&position, lane, lane_position);
			util.quaternion_wide_write_slot(&orientation, lane, lane_orientation);
		}
		bodies_scatter_active_pose_trusted(
			solver.bodies, encoded_references, integration_mask, position, orientation,
		);
	}
	if integrator.callbacks.integrate_kinematic_velocity == .Enabled
	{
		integrator.callbacks.integrate_velocity(
			integrator.callbacks.user_context, decoded, position, orientation, {},
			integration_mask, worker_index, util.F32x8(dt), &velocity,
		);
		bodies_scatter_active_velocities_trusted(
			solver.bodies, decoded, velocity, BODY_ACCESS_ALL,
		);
	}
}
solver_integrate_type_bundle_kinematics :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, bundle_index: int,
	dt: f32, phase: Solver_Substep_Phase, worker_index: int,
) -> Physics_Status
{
	first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	active_mask := solver_active_mask(first_index, int(type_batch.count));
	body_bundles := type_batch_body_bundle(type_batch, first_index);
	body_count := int(type_batch.body_count);
	for body_index in 0 ..< body_count
	{
		encoded := solver_mask_unused_body_lanes(
			body_bundles[body_index], first_index, int(type_batch.count),
		);
		integration_mode, integration_mask := solver_integration_mask(
			solver, type_batch, body_index, bundle_index, active_mask,
		);
		if integration_mode == .None
		{
			continue;
		}
		integration_mask = solver_filter_integration_mobility(
			encoded, integration_mask, .Kinematic,
		);
		if solver_mask_has_lanes(integration_mask) == .Missing
		{
			continue;
		}
		solver_integrate_kinematic_lanes(
			solver, encoded, integration_mask, dt, phase, worker_index,
		);
	}
	return .Ok;
}
solver_merge_integration_flags_256 :: #force_inline proc "contextless" (
	target, source: ^util.Buffer(u8), count: int,
) -> Physics_Status
{
	if target == nil || source == nil || count < 0 || count > int(target.length) || count > int(source.length)
	{
		return .Invalid_Argument;
	}
	index := 0;
	for index <= count - 32
	{
		a := intrinsics.unaligned_load((^simd.u8x32)(&target.memory[index]));
		b := intrinsics.unaligned_load((^simd.u8x32)(&source.memory[index]));
		intrinsics.unaligned_store((^simd.u8x32)(&target.memory[index]), a | b);
		index += 32;
	}
	for index < count
	{
		target.memory[index] |= source.memory[index];
		index += 1;
	}
	return .Ok;
}
@(enable_target_feature="avx512f")
solver_merge_integration_flags_512_kernel :: #force_inline proc "contextless" (
	target, source: ^util.Buffer(u8), count: int,
) -> Physics_Status
{
	if target == nil || source == nil || count < 0 || count > int(target.length) || count > int(source.length)
	{
		return .Invalid_Argument;
	}
	index := 0;
	for index <= count - 64
	{
		a := intrinsics.unaligned_load((^simd.u8x64)(&target.memory[index]));
		b := intrinsics.unaligned_load((^simd.u8x64)(&source.memory[index]));
		intrinsics.unaligned_store((^simd.u8x64)(&target.memory[index]), a | b);
		index += 64;
	}
	for index < count
	{
		target.memory[index] |= source.memory[index];
		index += 1;
	}
	return .Ok;
}
@(enable_target_feature="avx512f")
solver_merge_integration_flags_512 :: proc "contextless" (
	target, source: ^util.Buffer(u8), count: int,
) -> Physics_Status
{
	return solver_merge_integration_flags_512_kernel(target, source, count);
}
solver_merge_integration_flags :: proc "contextless" (
	target, source: ^util.Buffer(u8), count: int, configuration: util.Simd_Configuration,
) -> Physics_Status
{
	if configuration.tier == .AVX512_Bulk && .AVX512F in configuration.features
	{
		return solver_merge_integration_flags_512(target, source, count);
	}
	return solver_merge_integration_flags_256(target, source, count);
}
