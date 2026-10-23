#+vet !unused-imports

package entasis_physics

import "base:intrinsics"
import "core:simd"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// only the two absolute-target types append this payload to their existing
	// float prestep bundle. dword columns preserve exact f64 bits through the
	// existing lane move/remove and bulk-copy owners without widening all records
	Constraint_Global_Target_Wide :: struct
	{
		words: [6]util.I32x8,
	}

	// sleeping records already own 256 description bytes. these two descriptions
	// leave enough unused tail for the same authoritative target, without changing
	// any other sleeping constraint's allocation or the public float description
	CONSTRAINT_GLOBAL_TARGET_INACTIVE_OFFSET :: CONSTRAINT_DESCRIPTION_STORAGE_BYTES - size_of(Global_Position);
	#assert(size_of(One_Body_Linear_Servo) <= CONSTRAINT_GLOBAL_TARGET_INACTIVE_OFFSET);
	#assert(size_of(Body_Axis_Lock_Description) <= CONSTRAINT_GLOBAL_TARGET_INACTIVE_OFFSET);
	#assert(size_of(Constraint_Global_Target_Wide) == 8 * size_of(Global_Position));

	constraint_global_target_prefix_size :: #force_inline proc "contextless" (type_id: i32) -> int
	{
		switch type_id
		{
			case ONE_BODY_LINEAR_SERVO_TYPE_ID:
			return size_of(One_Body_Linear_Servo_Prestep);
			case BODY_AXIS_LOCK_TYPE_ID:
			return size_of(Body_Axis_Lock_Prestep);
			case:
			return 0;
		}
	}

	// trusted registered storage, supported type and lane 0..7. no pointer survives
	// a capacity mutation. the six columns are opaque bits, never float arithmetic
	constraint_global_target_read :: #force_inline proc "contextless" (
		prestep: rawptr, type_id: i32, lane: int,
	) -> Global_Position
	{
		payload: ^Constraint_Global_Target_Wide = (^Constraint_Global_Target_Wide)(
			&([^]u8)(prestep)[constraint_global_target_prefix_size(type_id)],
		);
		components: [3]f64;
		for axis in 0 ..< 3
		{
			low: u64 = u64(u32(simd.extract(payload.words[axis * 2], lane)));
			high: u64 = u64(u32(simd.extract(payload.words[axis * 2 + 1], lane)));
			components[axis] = transmute(f64)(low | (high << 32));
		}
		return {components[0], components[1], components[2]};
	}

	constraint_global_target_write :: #force_inline proc "contextless" (
		prestep: rawptr, type_id: i32, lane: int, target: Global_Position,
	)
	{
		payload: ^Constraint_Global_Target_Wide = (^Constraint_Global_Target_Wide)(
			&([^]u8)(prestep)[constraint_global_target_prefix_size(type_id)],
		);
		components: [3]f64 = {target.x, target.y, target.z};
		for axis in 0 ..< 3
		{
			bits: u64 = transmute(u64)components[axis];
			payload.words[axis * 2] = simd.replace(payload.words[axis * 2], lane, i32(u32(bits)));
			payload.words[axis * 2 + 1] = simd.replace(payload.words[axis * 2 + 1], lane, i32(u32(bits >> 32)));
		}
		cache: util.Vector3 = world_precision_position_cache(target);
		if type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
		{
			util.vector3_wide_write_slot(&(^One_Body_Linear_Servo_Prestep)(prestep).target, lane, cache);
		}
		else
		{
			util.vector3_wide_write_slot(&(^Body_Axis_Lock_Prestep)(prestep).position, lane, cache);
		}
	}

	// float authoring is an explicit replacement, including when its rounded
	// target equals the previous cache. high authoring uses the native setter
	constraint_global_target_apply_description :: proc "contextless" (
		source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
	)
	{
		prefix: int = constraint_global_target_prefix_size(i32(type_id));
		constraint_description_transfer(source, target, type_id, description_size, prefix, lane);
		position: util.Vector3;
		if type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
		{
			position = (^One_Body_Linear_Servo)(source).target;
		}
		else
		{
			position = (^Body_Axis_Lock_Description)(source).position;
		}
		constraint_global_target_write(target, i32(type_id), lane, world_precision_position_from_float(position));
	}

	constraint_global_target_build_description :: proc "contextless" (
		source, target: rawptr, type_id, description_size, prestep_bundle_size, lane: int,
	) -> Physics_Status
	{
		prefix: int = constraint_global_target_prefix_size(i32(type_id));
		if prefix == 0 || prestep_bundle_size != prefix + size_of(Constraint_Global_Target_Wide)
		{
			return .Invalid_Argument;
		}
		return constraint_description_build(source, target, type_id, description_size, prefix, lane);
	}

	constraint_global_target_inactive_read :: #force_inline proc "contextless" (
		record: ^Inactive_Constraint_Record,
	) -> Global_Position
	{
		target: Global_Position;
		intrinsics.mem_copy_non_overlapping(&target, &record.description[CONSTRAINT_GLOBAL_TARGET_INACTIVE_OFFSET], size_of(target));
		return target;
	}

	constraint_global_target_inactive_write :: #force_inline proc "contextless" (
		record: ^Inactive_Constraint_Record, target: Global_Position,
	)
	{
		value: Global_Position = target;
		intrinsics.mem_copy_non_overlapping(&record.description[CONSTRAINT_GLOBAL_TARGET_INACTIVE_OFFSET], &value, size_of(value));
		cache: util.Vector3 = world_precision_position_cache(target);
		if record.type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
		{
			(^One_Body_Linear_Servo)(rawptr(&record.description[0])).target = cache;
		}
		else
		{
			(^Body_Axis_Lock_Description)(rawptr(&record.description[0])).position = cache;
		}
	}

	// native mixed64 mutation/read boundary. only a one-body linear servo or an
	// engine-owned axis lock has an absolute target. public float descriptors and
	// caller-defined kernels retain their original contracts. borrowed world views
	// and snapshot/export contracts are not introduced by this raw native API
	simulation_get_constraint_global_target :: proc "contextless" (
		simulation: ^Simulation, handle: Constraint_Handle,
	) -> (Global_Position, Physics_Status)
	{
		if simulation == nil || simulation.state != .Ready
		{
			return {}, .Disposed;
		}
		if handle.value < 0 || int(handle.value) >= int(simulation.solver.handle_to_constraint.length)
		{
			return {}, .Not_Found;
		}
		location: Constraint_Location = simulation.solver.handle_to_constraint.memory[handle.value];
		if location.set_index < 0
		{
			return {}, .Not_Found;
		}
		if constraint_global_target_prefix_size(location.type_id) == 0
		{
			return {}, .Invalid_Argument;
		}
		if location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			index: int;
			status: Physics_Status;
			index, status = island_sleeper_resolve_inactive_constraint_index(&simulation.sleeper, handle);
			if status != .Ok
			{
				return {}, status;
			}
			return constraint_global_target_inactive_read(&simulation.sleeper.inactive_constraints.memory[index]), .Ok;
		}
		status: Physics_Status;
		location, status = solver_resolve(&simulation.solver, handle);
		if status != .Ok
		{
			return {}, status;
		}
		batch: ^Constraint_Batch = &simulation.solver.active_set.batches.memory[location.batch_index];
		type_batch: ^Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		index: int = int(location.index_in_type_batch);
		return constraint_global_target_read(type_batch_prestep_bundle(type_batch, index), location.type_id,
			index % util.PRODUCTION_LANE_COUNT), .Ok;
	}

	// validate before waking, an unsuccessful admission publishes no target. the
	// existing awaken transaction owns its capacity failures. preserve accumulated
	// impulses as ordinary servo description replacement does. axis-lock setting
	// changes retain their separate existing impulse-reset owner
	simulation_set_constraint_global_target :: proc (
		simulation: ^Simulation, handle: Constraint_Handle, target: Global_Position,
	) -> Physics_Status
	{
		status: Physics_Status;
		_, status = simulation_get_constraint_global_target(simulation, handle);
		if status != .Ok
		{
			return status;
		}
		status = world_precision_position_validate(target);
		if status != .Ok
		{
			return status;
		}
		location: Constraint_Location = simulation.solver.handle_to_constraint.memory[handle.value];
		simulation.state = .Stepping;
		defer simulation.state = .Ready;
		if location.set_index != BODIES_ACTIVE_SET_INDEX
		{
			status = island_awakener_awaken_set(&simulation.awakener, int(location.set_index));
			if status != .Ok
			{
				return status;
			}
			location = simulation.solver.handle_to_constraint.memory[handle.value];
		}
		batch: ^Constraint_Batch = &simulation.solver.active_set.batches.memory[location.batch_index];
		type_batch: ^Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
		index: int = int(location.index_in_type_batch);
		lane: int = index % util.PRODUCTION_LANE_COUNT;
		constraint_global_target_write(type_batch_prestep_bundle(type_batch, index), location.type_id, lane, target);
		body_index: int = int(u32(simd.extract(type_batch_body_bundle(type_batch, index)[0], lane)) & BODY_REFERENCE_INDEX_MASK);
		activity: ^Body_Activity = &simulation.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].activity.memory[body_index];
		activity.timesteps_under_threshold_count = 0;
		activity.sleep_candidate = .Not_Candidate;
		return .Ok;
	}
}
