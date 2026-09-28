package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

// engine-owned, deliberately outside the caller registration range 56..63
BODY_AXIS_LOCK_TYPE_ID :: 48;
Body_Lock_Axis :: enum u8
{
	X, Y, Z
}

Body_Lock_Axes :: bit_set[Body_Lock_Axis; u8];
Body_Axis_Lock :: struct
{
	reference: Rigid_Pose,
	linear_axes, angular_axes: Body_Lock_Axes,
	spring_settings: Spring_Settings,
}

body_axis_lock_default :: proc "contextless" (reference: Rigid_Pose) -> Body_Axis_Lock
{
	return {reference=reference, spring_settings=spring_settings_create(30, 1)};
}

// only selected constraints carry these authored values. float row flags allow
// the existing scalar/AOSOA transfer and sleeping representation to be reused
Body_Axis_Lock_Description :: struct
{
	position: util.Vector3,
	orientation: util.Quaternion,
	linear, angular: [3]f32,
	spring_settings: Spring_Settings,
}

Body_Axis_Lock_Prestep :: struct
{
	position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide,
	linear, angular: [3]util.F32x8,
	spring_settings: Spring_Settings_Wide,
}

Body_Axis_Lock_Impulses :: struct
{
	linear, angular: [3]util.F32x8
}

body_axis_lock_validate :: proc "contextless" (value: Body_Axis_Lock) -> Physics_Status
{
	if (transmute(u8)value.linear_axes | transmute(u8)value.angular_axes) & ~u8(7) != 0 ||
	constraint_vector3_finite(value.reference.position) == .Missing ||
	constraint_unit_quaternion(value.reference.orientation) == .Missing
	{
		return .Invalid_Argument;
	}
	return spring_settings_validate(value.spring_settings);
}

body_axis_lock_pack :: proc "contextless" (value: Body_Axis_Lock) -> Body_Axis_Lock_Description
{
	result: Body_Axis_Lock_Description = {position=value.reference.position,
		orientation=value.reference.orientation, spring_settings=value.spring_settings};
	for axis in Body_Lock_Axis
	{
		result.linear[int(axis)] = 1 if axis in value.linear_axes else 0;
		result.angular[int(axis)] = 1 if axis in value.angular_axes else 0;
	}
	return result;
}

body_axis_lock_unpack :: proc "contextless" (value: Body_Axis_Lock_Description) -> Body_Axis_Lock
{
	result: Body_Axis_Lock = {reference={position=value.position, orientation=value.orientation}, spring_settings=value.spring_settings};
	for axis in Body_Lock_Axis
	{
		if value.linear[int(axis)] != 0
		{
			result.linear_axes |= {axis};
		}
		if value.angular[int(axis)] != 0
		{
			result.angular_axes |= {axis};
		}
	}
	return result;
}

@(private)
body_axis_lock_validate_raw :: proc "contextless" (type_id: i32, raw: rawptr) -> Physics_Status
{
	if raw == nil || type_id != BODY_AXIS_LOCK_TYPE_ID
	{
		return .Invalid_Argument;
	}
	value: ^Body_Axis_Lock_Description = (^Body_Axis_Lock_Description)(raw);
	for i in 0 ..< 3
	{
		if (value.linear[i] != 0 && value.linear[i] != 1) ||
		(value.angular[i] != 0 && value.angular[i] != 1)
		{
			return .Invalid_Argument;
		}
	}
	return body_axis_lock_validate(body_axis_lock_unpack(value^));
}

// no universal dispatch or per-body control lookup: the existing generic
// one-body solver calls this kernel only for selected type-48 bundles. rows
// share the normal impulse, warmstart, fallback and island lifetime owners
body_axis_lock_kernel :: proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses_raw: rawptr, active_mask: util.I32x8, phase: Constraint_Kernel_Phase,
)
{
	prestep: ^Body_Axis_Lock_Prestep = (^Body_Axis_Lock_Prestep)(prestep_raw);
	impulses: ^Body_Axis_Lock_Impulses = (^Body_Axis_Lock_Impulses)(impulses_raw);
	if phase == .Incremental_Update
	{
		return;
	}
	if phase == .Prestep
	{
		for axis in 0 ..< 3
		{
			impulses.linear[axis] = util.wide_select_f32(active_mask & transmute(util.I32x8)simd.lanes_gt(prestep.linear[axis], util.F32x8(0)), impulses.linear[axis], {});
			impulses.angular[axis] = util.wide_select_f32(active_mask & transmute(util.I32x8)simd.lanes_gt(prestep.angular[axis], util.F32x8(0)), impulses.angular[axis], {});
		}
		return;
	}
	body: ^Constraint_Kernel_Body_Wide = &bodies[0];
	position_scale, mass_scale, softness: util.F32x8;
	rotation_error: util.Vector3_Wide;
	if phase == .Solve
	{
		// inactive lanes contain zeroed prestep data. substitute valid spring and
		// quaternion inputs once so masked lanes never introduce NaNs
		settings: Spring_Settings_Wide = {
			angular_frequency=util.wide_select_f32(active_mask, prestep.spring_settings.angular_frequency, util.F32x8(1)),
			twice_damping_ratio=util.wide_select_f32(active_mask, prestep.spring_settings.twice_damping_ratio, util.F32x8(2)),
		};
		position_scale, mass_scale, softness = spring_settings_wide_compute(settings, dt);
		position_scale = simd.min(position_scale, util.F32x8(inverse_dt));
		error: util.Quaternion_Wide = util.quaternion_wide_concatenate(
			util.quaternion_wide_conjugate(body.orientation), prestep.orientation);
		error.w = util.wide_select_f32(active_mask, error.w, util.F32x8(1));
		axis: util.Vector3_Wide;
		angle: util.F32x8;
		axis, angle = util.quaternion_wide_axis_angle(error);
		rotation_error = util.vector3_wide_scale(axis, angle);
	}
	for index in 0 ..< 3
	{
		axis: util.Vector3;
		switch index
		{
			case 0: axis.x = 1;
			case 1: axis.y = 1;
			case 2: axis.z = 1;
		}
		linear_axis: util.Vector3_Wide = util.vector3_wide_broadcast(axis);
		linear_mask: util.I32x8 = active_mask & transmute(util.I32x8)simd.lanes_gt(prestep.linear[index], util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(body.inverse_mass, util.F32x8(0));
		linear_impulse: util.F32x8;
		if phase == .Warmstart
		{
			linear_impulse = util.wide_select_f32(linear_mask, impulses.linear[index], {});
		}
		else
		{
			error: util.F32x8 = util.vector3_wide_dot(util.vector3_wide_subtract(prestep.position, body.position), linear_axis);
			velocity: util.F32x8 = util.vector3_wide_dot(body.linear_velocity, linear_axis);
			inverse_mass: util.F32x8 = util.wide_select_f32(linear_mask, body.inverse_mass, util.F32x8(1));
			linear_impulse = simd.sub(simd.mul(simd.div(simd.sub(simd.mul(error, position_scale), velocity), inverse_mass), mass_scale),
				simd.mul(impulses.linear[index], softness));
			linear_impulse = util.wide_select_f32(linear_mask, linear_impulse, {});
			impulses.linear[index] = simd.add(impulses.linear[index], linear_impulse);
		}
		body.linear_velocity = util.vector3_wide_add(body.linear_velocity,
			util.vector3_wide_scale(linear_axis, simd.mul(linear_impulse, body.inverse_mass)));
		// angular rows use the captured reference frame, not body-local axes that
		// rotate underneath the lock. the same shortest-arc error drives servos
		angular_axis: util.Vector3_Wide = util.quaternion_wide_transform(linear_axis, prestep.orientation);
		response: util.Vector3_Wide = util.symmetric3x3_wide_transform(angular_axis, body.inverse_inertia);
		inverse_effective_mass: util.F32x8 = util.vector3_wide_dot(angular_axis, response);
		angular_mask: util.I32x8 = active_mask & transmute(util.I32x8)simd.lanes_gt(prestep.angular[index], util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(inverse_effective_mass, util.F32x8(0));
		angular_impulse: util.F32x8;
		if phase == .Warmstart
		{
			angular_impulse = util.wide_select_f32(angular_mask, impulses.angular[index], {});
		}
		else
		{
			error: util.F32x8 = util.vector3_wide_dot(rotation_error, angular_axis);
			velocity: util.F32x8 = util.vector3_wide_dot(body.angular_velocity, angular_axis);
			denominator: util.F32x8 = util.wide_select_f32(angular_mask, inverse_effective_mass, util.F32x8(1));
			angular_impulse = simd.sub(simd.mul(simd.div(simd.sub(simd.mul(error, position_scale), velocity), denominator), mass_scale),
				simd.mul(impulses.angular[index], softness));
			angular_impulse = util.wide_select_f32(angular_mask, angular_impulse, {});
			impulses.angular[index] = simd.add(impulses.angular[index], angular_impulse);
		}
		body.angular_velocity = util.vector3_wide_add(body.angular_velocity, util.vector3_wide_scale(response, angular_impulse));
	}
}

body_axis_lock_register :: proc "contextless" (registry: ^Constraint_Type_Registry) -> Physics_Status
{
	record: ^Constraint_Type_Record = &registry.records[BODY_AXIS_LOCK_TYPE_ID];
	if record.registration == .Present
	{
		return .Ok if record.prestep_warmstart_solve == body_axis_lock_kernel &&
		record.validate_description == body_axis_lock_validate_raw else .Invalid_Argument;
	}
	return constraint_type_registry_register(registry, {
			type_id=BODY_AXIS_LOCK_TYPE_ID, body_count=1,
			description_size=size_of(Body_Axis_Lock_Description), prestep_bundle_size=size_of(Body_Axis_Lock_Prestep),
			impulse_bundle_size=size_of(Body_Axis_Lock_Impulses),
			initial_access={BODY_ACCESS_ALL, {}, {}, {}}, solve_access={BODY_ACCESS_ALL, {}, {}, {}},
			apply_description=constraint_description_transfer, build_description=constraint_description_build,
			validate_description=body_axis_lock_validate_raw, prestep_warmstart_solve=body_axis_lock_kernel,
			incrementally_update=body_axis_lock_kernel, move_record=constraint_storage_move_lane,
			remove_record=constraint_storage_remove_lane,
	});
}

// configuration queries are cold and reuse the authoritative body reference
// list. no duplicate handle map or per-step lookup is needed for these rows
body_axis_lock_find :: proc "contextless" (storage: ^Body_Control_Storage, body: Body_Handle) -> (Constraint_Handle, Physics_Status)
{
	location: Body_Memory_Location;
	status: Physics_Status;
	_, location, status = body_control_resolve(storage, body);
	if status != .Ok
	{
		return constraint_handle_invalid(), status;
	}
	// sleeping sets store constraints centrally. their body reference lists
	// are rebuilt only when the island awakens
	if location.set_index != BODIES_ACTIVE_SET_INDEX
	{
		sleeper: ^Island_Sleeper = &storage.simulation.sleeper;
		for index in 0 ..< sleeper.inactive_constraint_count
		{
			record: ^Inactive_Constraint_Record = &sleeper.inactive_constraints.memory[index];
			if record.set_index == location.set_index && record.type_id == BODY_AXIS_LOCK_TYPE_ID && record.body_handles[0] == body
			{
				return record.handle, .Ok;
			}
		}
		return constraint_handle_invalid(), .Not_Found;
	}
	references: ^util.Quick_List(Body_Constraint_Reference) = &storage.simulation.bodies.sets.memory[location.set_index].constraints.memory[location.index];
	for index in 0 ..< references.count
	{
		handle: Constraint_Handle = references.span.memory[index].connecting_constraint_handle;
		if storage.simulation.solver.handle_to_constraint.memory[handle.value].type_id == BODY_AXIS_LOCK_TYPE_ID
		{
			return handle, .Ok;
		}
	}
	return constraint_handle_invalid(), .Not_Found;
}

body_axis_lock_get :: proc "contextless" (storage: ^Body_Control_Storage, body: Body_Handle) -> (Body_Axis_Lock, Physics_Status)
{
	handle: Constraint_Handle;
	status: Physics_Status;
	handle, status = body_axis_lock_find(storage, body);
	if status != .Ok
	{
		return {}, status;
	}
	description: Body_Axis_Lock_Description;
	status = simulation_get_constraint_description_raw(storage.simulation, handle, BODY_AXIS_LOCK_TYPE_ID,
		&description, size_of(Body_Axis_Lock_Description));
	if status != .Ok
	{
		return {}, status;
	}
	return body_axis_lock_unpack(description), .Ok;
}

body_axis_lock_clear :: proc (storage: ^Body_Control_Storage, body: Body_Handle) -> Physics_Status
{
	handle: Constraint_Handle;
	status: Physics_Status;
	handle, status = body_axis_lock_find(storage, body);
	if status == .Not_Found
	{
		_, _, status = body_control_resolve(storage, body);
		return status;
	}
	if status != .Ok
	{
		return status;
	}
	simulation: ^Simulation = storage.simulation;
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	status = island_awakener_awaken_body(&simulation.awakener, body);
	if status != .Ok
	{
		return status;
	}
	return solver_remove(&simulation.solver, handle);
}

body_axis_lock_set :: proc (storage: ^Body_Control_Storage, body: Body_Handle, value: Body_Axis_Lock) -> Physics_Status
{
	status: Physics_Status = body_axis_lock_validate(value);
	if status != .Ok
	{
		return status;
	}
	dynamics: ^Body_Dynamics;
	resolve_status: Physics_Status;
	dynamics, _, resolve_status = body_control_resolve(storage, body);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	if value.linear_axes == {} && value.angular_axes == {}
	{
		return body_axis_lock_clear(storage, body);
	}
	if body_inertia_mobility(dynamics.inertia.local) != .Dynamic
	{
		return .Invalid_Argument;
	}
	simulation: ^Simulation = storage.simulation;
	old_handle: Constraint_Handle;
	old_status: Physics_Status;
	old_handle, old_status = body_axis_lock_find(storage, body);
	if old_status != .Ok && old_status != .Not_Found
	{
		return old_status;
	}
	description: Body_Axis_Lock_Description = body_axis_lock_pack(value);
	status = body_axis_lock_register(&simulation.solver.registry);
	if status != .Ok
	{
		return status;
	}
	// allocator callbacks cannot reenter while awakening or creating storage
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	status = island_awakener_awaken_body(&simulation.awakener, body);
	if status != .Ok
	{
		return status;
	}
	if old_status == .Ok
	{
		status = solver_apply_description_raw(&simulation.solver, old_handle, BODY_AXIS_LOCK_TYPE_ID, &description);
		if status == .Ok
		{
			location: Constraint_Location;
			location, _ = solver_resolve(&simulation.solver, old_handle);
			batch: ^Constraint_Batch = &simulation.solver.active_set.batches.memory[location.batch_index];
			type_batch: ^Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[BODY_AXIS_LOCK_TYPE_ID]];
			impulses: [^]f32 = ([^]f32)(type_batch_impulse_bundle(type_batch, int(location.index_in_type_batch)));
			lane: int = int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT;
			for field in 0 ..< 6
			{
				impulses[field * util.PRODUCTION_LANE_COUNT + lane] = 0;
			}
		}
	}
	else
	{
		bodies: [4]Body_Handle;
		bodies[0] = body;
		_, status = solver_add_raw(&simulation.solver, BODY_AXIS_LOCK_TYPE_ID, &bodies, &description);
	}
	if status == .Ok
	{
		location: Body_Memory_Location;
		location, _ = bodies_resolve(&simulation.bodies, body);
		activity: ^Body_Activity = &simulation.bodies.sets.memory[location.set_index].activity.memory[location.index];
		activity.timesteps_under_threshold_count = 0;
		activity.sleep_candidate = .Not_Candidate;
	}
	return status;
}

// disable is an explicit cold mutation. wake only islands containing owned
// locks, then remove only type 48. explicit user joints are not detached
body_axis_lock_remove_all :: proc (simulation: ^Simulation) -> Physics_Status
{
	index: int = 0;
	for index < simulation.sleeper.inactive_constraint_count
	{
		record: ^Inactive_Constraint_Record = &simulation.sleeper.inactive_constraints.memory[index];
		if record.type_id != BODY_AXIS_LOCK_TYPE_ID
		{
			index += 1;
			continue;
		}
		status: Physics_Status = island_awakener_awaken_set(&simulation.awakener, int(record.set_index));
		if status != .Ok
		{
			return status;
		}
		// awakening compacts records. revisit the current slot
	}
	for batch_index in 0 ..< int(simulation.solver.active_set.batch_count)
	{
		batch: ^Constraint_Batch = &simulation.solver.active_set.batches.memory[batch_index];
		type_index: int = int(batch.type_id_to_batch_index[BODY_AXIS_LOCK_TYPE_ID]);
		if type_index < 0
		{
			continue;
		}
		type_batch: ^Type_Batch = &batch.type_batches.memory[type_index];
		for type_batch.count > 0
		{
			status: Physics_Status = solver_remove_record_at(&simulation.solver, batch_index, type_index, int(type_batch.count)-1, .Present);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
#assert(size_of(Body_Axis_Lock_Prestep) == size_of(Body_Axis_Lock_Description) * 8);
#assert(size_of(Body_Axis_Lock_Description) <= CONSTRAINT_DESCRIPTION_STORAGE_BYTES);
