package entasis_physics

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	world_precision_translate_global_position :: #force_inline proc "contextless" (
		position: Global_Position, delta: util.Vector3,
	) -> Global_Position
	{
		return {position.x - f64(delta.x), position.y - f64(delta.y), position.z - f64(delta.z)};
	}

	// raw active caches publish only at admitted idle stage entry
	// validate every changed vector before importing any authority
	world_precision_import_active_positions :: proc "contextless" (bodies: ^Bodies) -> Physics_Status
	{
		active: ^Body_Set = &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		for index in 0 ..< active.count
		{
			cache: util.Vector3 = active.dynamics_state.memory[index].motion.pose.position;
			if cache != world_precision_position_cache(active.global_positions.memory[index]) &&
				world_precision_position_validate(world_precision_position_from_float(cache)) != .Ok
			{
				return .Invalid_Argument;
			}
		}
		for index in 0 ..< active.count
		{
			cache: util.Vector3 = active.dynamics_state.memory[index].motion.pose.position;
			if cache != world_precision_position_cache(active.global_positions.memory[index])
			{
				active.global_positions.memory[index] = world_precision_position_from_float(cache);
			}
		}
		return .Ok;
	}

	// mixed64 stores authoritative body positions in a dense column
	// the existing 128-byte dynamics record retains a rounded f32 pose cache for
	// legacy consumers. relative motion uses the authoritative positions
	world_precision_position_from_float :: #force_inline proc "contextless" (
		position: util.Vector3,
	) -> Global_Position
	{
		return {f64(position.x), f64(position.y), f64(position.z)};
	}

	world_precision_position_cache :: #force_inline proc "contextless" (
		position: Global_Position,
	) -> util.Vector3
	{
		return {f32(position.x), f32(position.y), f32(position.z)};
	}

	// the float cache requires positions within finite binary32 range
	// bounds enclosure has separate geometry-dependent admission
	world_precision_position_validate :: #force_inline proc "contextless" (
		position: Global_Position,
	) -> Physics_Status
	{
		// round the library decimal through f32 before widening its finite limit
		limit: f64 = f64(max(f32));
		if !(abs(position.x) <= limit && abs(position.y) <= limit && abs(position.z) <= limit)
		{
			return .Invalid_Argument;
		}
		return .Ok;
	}

	bodies_get_global_position :: proc "contextless" (
		bodies: ^Bodies, handle: Body_Handle,
	) -> (Global_Position, Physics_Status)
	{
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(bodies, handle);
		if status != .Ok
		{
			return {}, status;
		}
		return bodies.sets.memory[location.set_index].global_positions.memory[location.index], .Ok;
	}

	// like bodies_apply_description, this is a raw storage mutation, not a world
	// transaction: callers own synchronization, awakening and broad-phase updates
	bodies_set_global_position :: proc "contextless" (
		bodies: ^Bodies, handle: Body_Handle, position: Global_Position,
	) -> Physics_Status
	{
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(bodies, handle);
		if status != .Ok
		{
			return status;
		}
		status = world_precision_position_validate(position);
		if status != .Ok
		{
			return status;
		}
		set: ^Body_Set = &bodies.sets.memory[location.set_index];
		set.global_positions.memory[location.index] = position;
		set.dynamics_state.memory[location.index].motion.pose.position = world_precision_position_cache(position);
		if bodies.triggers != nil
		{
			trigger_changed(bodies.triggers, .Body, handle.value, .Modified);
		}
		return .Ok;
	}

	statics_get_global_position :: proc "contextless" (
		statics: ^Statics, handle: Static_Handle,
	) -> (Global_Position, Physics_Status)
	{
		index: int;
		status: Physics_Status;
		index, status = statics_resolve(statics, handle);
		if status != .Ok
		{
			return {}, status;
		}
		return statics.global_positions.memory[index], .Ok;
	}

	statics_set_global_position :: proc "contextless" (
		statics: ^Statics, handle: Static_Handle, position: Global_Position,
	) -> Physics_Status
	{
		index: int;
		status: Physics_Status;
		index, status = statics_resolve(statics, handle);
		if status != .Ok
		{
			return status;
		}
		status = world_precision_position_validate(position);
		if status != .Ok
		{
			return status;
		}
		static: ^Static = &statics.statics.memory[index];
		statics.global_positions.memory[index] = position;
		static.pose.position = world_precision_position_cache(position);
		if statics.triggers != nil
		{
			trigger_changed(statics.triggers, .Static, handle.value, .Modified);
		}
		return .Ok;
	}

	// numeric mode: tolerant. displacement is computed in the unchanged local f32
	// arithmetic, then widened before addition to the authoritative f64 position
	// caller owns a live, unique body index and a finite representable trajectory
	world_precision_advance_position :: #force_inline proc "contextless" (
		position: ^Global_Position, displacement: util.Vector3,
	) -> util.Vector3
	{
		position.x += f64(displacement.x);
		position.y += f64(displacement.y);
		position.z += f64(displacement.z);
		return world_precision_position_cache(position^);
	}

	// pair-local displacement is formed before narrowing either operand. the
	// existing local collision/solver representation requires a finite f32 delta,
	// reject overflow before publishing a pair record or any part of a bundle
	world_precision_relative_position :: #force_inline proc "contextless" (
		position_a, position_b: Global_Position,
	) -> (util.Vector3, Physics_Status)
	{
		delta: Global_Position = {
			position_b.x - position_a.x,
			position_b.y - position_a.y,
			position_b.z - position_a.z,
		};
		status: Physics_Status = world_precision_position_validate(delta);
		if status != .Ok
		{
			return {}, status;
		}
		return world_precision_position_cache(delta), .Ok;
	}
}
