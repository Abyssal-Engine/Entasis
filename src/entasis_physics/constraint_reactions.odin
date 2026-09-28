package entasis_physics

import "core:math"
import "core:simd"
import util "entasis:entasis_utilities"

// world-space impulses about each body's center of mass. these are physical
// wrenches, not norms of solver coordinates (which can have different units)
Constraint_Impulse_Wrench :: struct
{
	linear: util.Vector3,
	angular: util.Vector3,
}

Constraint_Reaction_Provider_Input :: struct
{
	type_id: i32,
	body_count: i32,
	poses: [4]Rigid_Pose,
	description: rawptr,
	description_size: i32,
	impulse_count: i32,
	impulses: [32]f32,
	substep_duration: f32,
}

// all input views expire on return. providers may read their own user data but
// must not mutate world storage or unwind. output contains impulses, not forces
Constraint_Reaction_Provider_Proc :: #type proc "contextless" (
	user_context: rawptr, input: ^Constraint_Reaction_Provider_Input,
	output: ^[4]Constraint_Impulse_Wrench,
) -> Physics_Status;
Constraint_Reaction_Provider :: struct
{
	read: Constraint_Reaction_Provider_Proc,
	user_context: rawptr,
}

constraint_builtin_reaction_support :: proc "contextless" (type_id: i32) -> Reference_State
{
	switch type_id
	{
		case BALL_SOCKET_TYPE_ID,
		ANGULAR_HINGE_TYPE_ID,
		ANGULAR_SWIVEL_HINGE_TYPE_ID,
		SWING_LIMIT_TYPE_ID,
		TWIST_SERVO_TYPE_ID,
		TWIST_LIMIT_TYPE_ID,
		TWIST_MOTOR_TYPE_ID,
		ANGULAR_SERVO_TYPE_ID,
		ANGULAR_MOTOR_TYPE_ID,
		WELD_TYPE_ID,
		VOLUME_CONSTRAINT_TYPE_ID,
		DISTANCE_SERVO_TYPE_ID,
		DISTANCE_LIMIT_TYPE_ID,
		CENTER_DISTANCE_CONSTRAINT_TYPE_ID,
		AREA_CONSTRAINT_TYPE_ID,
		POINT_ON_LINE_SERVO_TYPE_ID,
		LINEAR_AXIS_SERVO_TYPE_ID,
		LINEAR_AXIS_MOTOR_TYPE_ID,
		LINEAR_AXIS_LIMIT_TYPE_ID,
		ANGULAR_AXIS_MOTOR_TYPE_ID,
		ONE_BODY_ANGULAR_SERVO_TYPE_ID,
		ONE_BODY_ANGULAR_MOTOR_TYPE_ID,
		ONE_BODY_LINEAR_SERVO_TYPE_ID,
		ONE_BODY_LINEAR_MOTOR_TYPE_ID,
		SWIVEL_HINGE_TYPE_ID,
		HINGE_TYPE_ID,
		BALL_SOCKET_MOTOR_TYPE_ID,
		BALL_SOCKET_SERVO_TYPE_ID,
		ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID,
		CENTER_DISTANCE_LIMIT_TYPE_ID,
		BODY_AXIS_LOCK_TYPE_ID:
		return .Present;
		case:
		return .Missing;
	}
}

// inputs originate from a registered builtin and an active constraint lane.
// all four scratch bodies are isolated, with unit inverse mass/inertia and zero
// velocities. Warmstart applies exactly J^T * lambda, including anchor lever
// arms, mixed/angular rows and normalized area/volume Jacobians. no solve,
// prestep, impulse mutation, world scatter, allocator or application callback
constraint_builtin_reaction_kernel :: proc "contextless" (
	type_id: i32, prestep, impulses: rawptr,
	bodies: ^[4]Constraint_Kernel_Body_Wide, active_mask: util.I32x8,
)
{
	switch type_id
	{
		case BALL_SOCKET_TYPE_ID:
		solver_execute_direct_noncontact_kernel(BALL_SOCKET_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_HINGE_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_HINGE_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_SWIVEL_HINGE_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_SWIVEL_HINGE_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case SWING_LIMIT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(SWING_LIMIT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case TWIST_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(TWIST_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case TWIST_LIMIT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(TWIST_LIMIT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case TWIST_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(TWIST_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case WELD_TYPE_ID:
		solver_execute_direct_noncontact_kernel(WELD_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case VOLUME_CONSTRAINT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(VOLUME_CONSTRAINT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case DISTANCE_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(DISTANCE_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case DISTANCE_LIMIT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(DISTANCE_LIMIT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case CENTER_DISTANCE_CONSTRAINT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(CENTER_DISTANCE_CONSTRAINT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case AREA_CONSTRAINT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(AREA_CONSTRAINT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case POINT_ON_LINE_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(POINT_ON_LINE_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case LINEAR_AXIS_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(LINEAR_AXIS_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case LINEAR_AXIS_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(LINEAR_AXIS_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case LINEAR_AXIS_LIMIT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(LINEAR_AXIS_LIMIT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_AXIS_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_AXIS_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ONE_BODY_ANGULAR_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ONE_BODY_ANGULAR_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ONE_BODY_ANGULAR_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ONE_BODY_ANGULAR_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ONE_BODY_LINEAR_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ONE_BODY_LINEAR_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ONE_BODY_LINEAR_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ONE_BODY_LINEAR_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case SWIVEL_HINGE_TYPE_ID:
		solver_execute_direct_noncontact_kernel(SWIVEL_HINGE_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case HINGE_TYPE_ID:
		solver_execute_direct_noncontact_kernel(HINGE_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case BALL_SOCKET_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(BALL_SOCKET_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case BALL_SOCKET_SERVO_TYPE_ID:
		solver_execute_direct_noncontact_kernel(BALL_SOCKET_SERVO_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID:
		solver_execute_direct_noncontact_kernel(ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case CENTER_DISTANCE_LIMIT_TYPE_ID:
		solver_execute_direct_noncontact_kernel(CENTER_DISTANCE_LIMIT_TYPE_ID, prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case BODY_AXIS_LOCK_TYPE_ID:
		body_axis_lock_kernel(prestep, bodies, 1, 1, impulses, active_mask, .Warmstart);
		case:
		unreachable();
	}
}

constraint_reaction_wrench_finite :: proc "contextless" (
	linear, angular: util.Vector3,
) -> Reference_State
{
	if math.is_nan(linear.x) || math.is_inf(linear.x, 0) ||
	math.is_nan(linear.y) || math.is_inf(linear.y, 0) ||
	math.is_nan(linear.z) || math.is_inf(linear.z, 0) ||
	math.is_nan(angular.x) || math.is_inf(angular.x, 0) ||
	math.is_nan(angular.y) || math.is_inf(angular.y, 0) ||
	math.is_nan(angular.z) || math.is_inf(angular.z, 0)
	{
		return .Missing;
	}
	return .Present;
}

// the joined substep owner resolves handles afresh. neither source pointers nor
// borrowed body/prestep views are retained across migration, solve or removal
constraint_read_reaction_impulses :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, duration: f32,
	provider: Constraint_Reaction_Provider,
	output: ^[4]Constraint_Impulse_Wrench,
	body_handles: ^[4]Body_Handle,
) -> (body_count: i32, status: Physics_Status)
{
	if output == nil || body_handles == nil || !(duration > 0) || math.is_inf(duration, 0)
	{
		return 0, .Invalid_Argument;
	}
	location: Constraint_Location;
	location, status = solver_resolve(solver, handle);
	if status != .Ok
	{
		return 0, status;
	}
	batch: ^Constraint_Batch = &solver.active_set.batches.memory[location.batch_index];
	type_batch: ^Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
	record: ^Constraint_Type_Record = &solver.registry.records[location.type_id];
	builtin: Reference_State = constraint_builtin_reaction_support(record.type_id);
	// registry admission already establishes description size and callbacks.
	// the reaction provider adds only its scalar impulse bound and unit contract
	if builtin == .Missing && (record.type_id < FIRST_CALLER_CONSTRAINT_TYPE_ID ||
		provider.read == nil || record.impulse_bundle_size % size_of(util.F32x8) != 0 ||
		record.impulse_bundle_size / size_of(util.F32x8) > 32)
	{
		return 0, .Invalid_Argument;
	}
	body_count = record.body_count;
	first: int = int(location.index_in_type_batch);
	lane: int = first % util.PRODUCTION_LANE_COUNT;
	reference: Constraint_Reference = type_batch_read_reference_trusted(type_batch, solver.bodies, first);
	handles: [4]Body_Handle = reference.body_handles;
	poses: [4]Rigid_Pose;
	for body_index in 0 ..< int(body_count)
	{
		body_location: Body_Memory_Location = solver.bodies.handle_to_location.memory[handles[body_index].value];
		poses[body_index] = solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[body_location.index].motion.pose;
	}
	result: [4]Constraint_Impulse_Wrench;
	prestep: rawptr = type_batch_prestep_bundle(type_batch, first);
	impulses: rawptr = type_batch_impulse_bundle(type_batch, first);
	if builtin == .Present
	{
		bodies: [4]Constraint_Kernel_Body_Wide;
		for body_index in 0 ..< int(body_count)
		{
			body: ^Constraint_Kernel_Body_Wide = &bodies[body_index];
			body.position = util.vector3_wide_broadcast(poses[body_index].position);
			body.orientation = util.quaternion_wide_broadcast(poses[body_index].orientation);
			body.inverse_mass = util.F32x8(1);
			body.inverse_inertia.xx = util.F32x8(1);
			body.inverse_inertia.yy = util.F32x8(1);
			body.inverse_inertia.zz = util.F32x8(1);
		}
		mask: util.I32x8 = simd.replace(util.I32x8(0), lane, -1);
		constraint_builtin_reaction_kernel(record.type_id, prestep, impulses, &bodies, mask);
		for body_index in 0 ..< int(body_count)
		{
			result[body_index].linear = util.vector3_wide_read_slot(bodies[body_index].linear_velocity, lane);
			result[body_index].angular = util.vector3_wide_read_slot(bodies[body_index].angular_velocity, lane);
		}
	}
	else
	{
		// decode only this lane into caller-local scalar provider views
		input: Constraint_Reaction_Provider_Input = {
			type_id=record.type_id, body_count=body_count, poses=poses,
			description_size=record.description_size,
			impulse_count=record.impulse_bundle_size / size_of(util.F32x8),
			substep_duration=duration,
		};
		description: [CONSTRAINT_DESCRIPTION_STORAGE_BYTES / size_of(f32)]f32;
		status = record.build_description(prestep, &description[0], int(record.type_id), int(record.description_size), int(record.prestep_bundle_size), lane);
		if status != .Ok
		{
			return 0, status;
		}
		input.description = &description[0];
		vectors: [^]util.F32x8 = ([^]util.F32x8)(impulses);
		for index in 0 ..< int(input.impulse_count)
		{
			input.impulses[index] = simd.extract(vectors[index], lane);
		}
		status = provider.read(provider.user_context, &input, &result);
		if status != .Ok
		{
			return 0, status;
		}
	}
	for body_index in 0 ..< int(body_count)
	{
		if constraint_reaction_wrench_finite(result[body_index].linear, result[body_index].angular) == .Missing
		{
			return 0, .Invalid_Argument;
		}
	}
	output^ = result;
	body_handles^ = handles;
	return body_count, .Ok;
}

// a transient, joined-substep result. watch ownership and completed-step
// publication are separate. this function does not infer when lambda was solved
Constraint_Force_Wrench :: struct
{
	linear: util.Vector3,
	angular: util.Vector3,
}

Constraint_Reaction_Sample :: struct
{
	body_count: i32,
	bodies: [4]Body_Handle,
	forces: [4]Constraint_Force_Wrench,
	substep_duration: f32,
	maximum_force, maximum_torque: f32,
}

constraint_reaction_magnitude :: proc "contextless" (value: util.Vector3) -> f32
{
	scale: f32 = max(abs(value.x), abs(value.y), abs(value.z));
	if scale == 0
	{
		return 0;
	}
	x, y, z: f32 = value.x / scale, value.y / scale, value.z / scale;
	return scale * math.sqrt(x*x+y*y+z*z);
}

// caller must sample after a joined, actually executed substep, before pose
// integration or impulse rescaling. the routine never wakes sleeping bodies
constraint_capture_reaction :: proc "contextless" (
	solver: ^Solver, handle: Constraint_Handle, duration: f32,
	provider: Constraint_Reaction_Provider,
	output: ^Constraint_Reaction_Sample,
) -> Physics_Status
{
	if output == nil
	{
		return .Invalid_Argument;
	}
	impulses: [4]Constraint_Impulse_Wrench;
	sample: Constraint_Reaction_Sample;
	count: i32;
	status: Physics_Status;
	count, status = constraint_read_reaction_impulses(solver, handle, duration, provider, &impulses, &sample.bodies);
	if status != .Ok
	{
		return status;
	}
	sample.body_count = count;
	sample.substep_duration = duration;
	for index in 0 ..< int(count)
	{
		force: ^Constraint_Force_Wrench = &sample.forces[index];
		force.linear = {impulses[index].linear.x/duration, impulses[index].linear.y/duration, impulses[index].linear.z/duration};
		force.angular = {impulses[index].angular.x/duration, impulses[index].angular.y/duration, impulses[index].angular.z/duration};
		if constraint_reaction_wrench_finite(force.linear, force.angular) == .Missing
		{
			return .Invalid_Argument;
		}
		sample.maximum_force = max(sample.maximum_force, constraint_reaction_magnitude(force.linear));
		sample.maximum_torque = max(sample.maximum_torque, constraint_reaction_magnitude(force.angular));
	}
	if math.is_inf(sample.maximum_force, 0) || math.is_inf(sample.maximum_torque, 0)
	{
		return .Invalid_Argument;
	}
	output^ = sample;
	return .Ok;
}

// selected_lanes addresses the bundle containing anchor. unselected lanes are
// zero in a successful result. a failed read publishes no lanes. the scheduler
// can group watches by live type/bundle without attaching metadata to joints
constraint_capture_reaction_bundle :: proc "contextless" (
	solver: ^Solver, anchor: Constraint_Handle, selected_lanes: u8,
	duration: f32, provider: Constraint_Reaction_Provider,
	output: ^[util.PRODUCTION_LANE_COUNT]Constraint_Reaction_Sample,
) -> Physics_Status
{
	if output == nil || selected_lanes == 0 || !(duration > 0) || math.is_inf(duration, 0)
	{
		return .Invalid_Argument;
	}
	location: Constraint_Location;
	status: Physics_Status;
	location, status = solver_resolve(solver, anchor);
	if status != .Ok
	{
		return status;
	}
	batch: ^Constraint_Batch = &solver.active_set.batches.memory[location.batch_index];
	type_batch: ^Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
	first: int = int(location.index_in_type_batch) / util.PRODUCTION_LANE_COUNT * util.PRODUCTION_LANE_COUNT;
	lane_count: int = min(util.PRODUCTION_LANE_COUNT, int(type_batch.count)-first);
	if u32(selected_lanes) & ~((u32(1)<<u32(lane_count))-1) != 0
	{
		return .Invalid_Argument;
	}
	samples: [util.PRODUCTION_LANE_COUNT]Constraint_Reaction_Sample;
	record: ^Constraint_Type_Record = &solver.registry.records[location.type_id];
	if constraint_builtin_reaction_support(record.type_id) == .Missing
	{
		// custom coordinate systems need the explicit provider, never the
		// builtin warmstart reconstruction. a callback failure is batch-atomic
		for lane in 0 ..< lane_count
		{
			if selected_lanes & (u8(1)<<u32(lane)) == 0
			{
				continue;
			}
			handle: Constraint_Handle = {value=type_batch.index_to_handle.memory[first+lane]};
			status = constraint_capture_reaction(solver, handle, duration, provider, &samples[lane]);
			if status != .Ok
			{
				return status;
			}
		}
		output^ = samples;
		return .Ok;
	}
	bodies: [4]Constraint_Kernel_Body_Wide;
	for body_index in 0 ..< int(record.body_count)
	{
		bodies[body_index].inverse_mass=util.F32x8(1);
		bodies[body_index].inverse_inertia={xx=util.F32x8(1), yy=util.F32x8(1), zz=util.F32x8(1)};
		bodies[body_index].orientation.w=util.F32x8(1);
	}
	mask: util.I32x8;
	for lane in 0 ..< lane_count
	{
		if selected_lanes & (u8(1)<<u32(lane)) == 0
		{
			continue;
		}
		mask=simd.replace(mask, lane, -1);
		reference: Constraint_Reference=type_batch_read_reference_trusted(type_batch, solver.bodies, first+lane);
		samples[lane].bodies=reference.body_handles;
		samples[lane].body_count=record.body_count;
		samples[lane].substep_duration=duration;
		for body_index in 0 ..< int(record.body_count)
		{
			body_location: Body_Memory_Location=solver.bodies.handle_to_location.memory[reference.body_handles[body_index].value];
			pose: Rigid_Pose=solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[body_location.index].motion.pose;
			util.vector3_wide_write_slot(&bodies[body_index].position, lane, pose.position);
			util.quaternion_wide_write_slot(&bodies[body_index].orientation, lane, pose.orientation);
		}
	}
	constraint_builtin_reaction_kernel(record.type_id, type_batch_prestep_bundle(type_batch, first), type_batch_impulse_bundle(type_batch, first), &bodies, mask);
	for lane in 0 ..< lane_count
	{
		if selected_lanes & (u8(1)<<u32(lane)) == 0
		{
			continue;
		}
		sample: ^Constraint_Reaction_Sample=&samples[lane];
		for body_index in 0 ..< int(record.body_count)
		{
			linear: util.Vector3=util.vector3_wide_read_slot(bodies[body_index].linear_velocity, lane);
			angular: util.Vector3=util.vector3_wide_read_slot(bodies[body_index].angular_velocity, lane);
			force: ^Constraint_Force_Wrench=&sample.forces[body_index];
			force.linear={linear.x/duration, linear.y/duration, linear.z/duration};
			force.angular={angular.x/duration, angular.y/duration, angular.z/duration};
			if constraint_reaction_wrench_finite(force.linear, force.angular) == .Missing
			{
				return .Invalid_Argument;
			}
			sample.maximum_force=max(sample.maximum_force, constraint_reaction_magnitude(force.linear));
			sample.maximum_torque=max(sample.maximum_torque, constraint_reaction_magnitude(force.angular));
		}
		if math.is_inf(sample.maximum_force, 0) || math.is_inf(sample.maximum_torque, 0)
		{
			return .Invalid_Argument;
		}
	}
	output^=samples;
	return .Ok;
}
