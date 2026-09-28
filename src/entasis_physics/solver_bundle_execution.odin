package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"
import "core:sync"

solver_active_mask :: #force_inline proc "contextless" (first_index, count: int) -> util.I32x8
{
	if first_index + util.PRODUCTION_LANE_COUNT <= count
	{
		return util.I32x8(-1);
	}
	values: [util.PRODUCTION_LANE_COUNT]i32;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if first_index + lane < count
		{
			values[lane] = -1;
		}
	}
	return transmute(util.I32x8)values;
}

solver_mask_unused_body_lanes :: #force_inline proc "contextless" (
	encoded: util.I32x8, first_index, count: int,
) -> util.I32x8
{
	if first_index + util.PRODUCTION_LANE_COUNT <= count
	{
		return encoded;
	}
	result := encoded;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		if first_index + lane >= count
		{
			result = simd.replace(result, lane, i32(BODY_REFERENCE_KINEMATIC_MASK));
		}
	}
	return result;
}

solver_execute_type_bundle_dispatch_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int = 0,
	$callback_dispatch: Constraint_Callback_Dispatch,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	if stage == .Integrate_Constrained_Kinematics
	{
		return solver_integrate_type_bundle_kinematics(
			solver, type_batch, bundle_index, dt, phase, worker_index,
		);
	}
	first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	active_mask := solver_active_mask(first_index, int(type_batch.count));
	body_bundles := type_batch_body_bundle(type_batch, first_index);
	kernel_bodies: [4]Constraint_Kernel_Body_Wide;
	encoded_references: [4]util.I32x8;
	body_accesses: [4]Body_Access_Mask;
	for body_index in 0 ..< int(type_batch.body_count)
	{
		encoded := solver_mask_unused_body_lanes(
			body_bundles[body_index], first_index, int(type_batch.count),
		);
		encoded_references[body_index] = encoded;
		access := record.solve_access[body_index];
		if stage != .Solve
		{
			access = record.initial_access[body_index];
		}
		integration_mode := Solver_Bundle_Integration_Mode.None;
		integration_mask: util.I32x8;
		if stage == .Warmstart && velocity_integration != .Never
		{
			if batch_index == 0
			{
				integration_mode = .All;
				integration_mask = active_mask;
			}
			else
			{
				integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
		}
		body_accesses[body_index] = access;
		inertia_source := Inertia_Source.World;
		if integration_mode != .None
		{
			inertia_source = .Local;
		}
		position, orientation, velocity, inertia := bodies_gather_active_trusted(
			solver.bodies, encoded, inertia_source, access,
		);
		if integration_mode != .None
		{
			inertia = solver_integrate_constraint_body(
				solver, encoded, integration_mask, dt, phase, worker_index,
				&position, &orientation, &velocity, inertia,
			);
		}
		// write gathered fields directly, without a temporary 640-byte body
		body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
		body.position = position;
		body.orientation = orientation;
		body.linear_velocity = velocity.linear;
		body.angular_velocity = velocity.angular;
		body.inverse_mass = inertia.inverse_mass;
		body.inverse_inertia = inertia.inverse_inertia_tensor;
	}
	prestep := type_batch_prestep_bundle(type_batch, first_index);
	impulses := type_batch_impulse_bundle(type_batch, first_index);
	when callback_dispatch == .Contextual
	{
		if stage == .Incremental_Update && phase == .First
		{
			return .Ok;
		}
		status: Physics_Status = constraint_contextual_execute_kernel(
			record.contextual, prestep, &kernel_bodies, dt, inverse_dt,
			impulses, &active_mask, phase, stage,
		);
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		switch stage
		{
			case .Incremental_Update:
			if phase == .First
			{
				return .Ok;
			}
			record.incrementally_update(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask,
				.Incremental_Update,
			);
			case .Integrate_Constrained_Dynamics, .Capture_Restitution,
			.Prepare_Integration_Responsibilities, .Integrate_Constrained_Kinematics:
			return .Invalid_Description;
			case .Warmstart:
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Prestep,
			);
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Warmstart,
			);
			case .Solve:
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Solve,
			);
			case .Integrate_Constrained_Poses, .Integrate_Unconstrained_After_Substepping:
			return .Invalid_Description;
		}
	}
	for body_index in 0 ..< int(type_batch.body_count)
	{
		velocity := Body_Velocity_Wide{
			linear=kernel_bodies[body_index].linear_velocity,
			angular=kernel_bodies[body_index].angular_velocity,
		};
		bodies_scatter_active_velocities_trusted(
			solver.bodies, encoded_references[body_index], velocity,
			body_accesses[body_index],
		);
	}
	return .Ok;
}

solver_execute_direct_noncontact_kernel :: #force_inline proc "contextless" (
	$type_id: i32, prestep: rawptr, bodies: ^[4]Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32, impulses: rawptr, active_mask: util.I32x8,
	phase: Constraint_Kernel_Phase,
)
{
	when type_id == BALL_SOCKET_TYPE_ID
	{
		ball_socket_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_HINGE_TYPE_ID
	{
		angular_hinge_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_SWIVEL_HINGE_TYPE_ID
	{
		angular_swivel_hinge_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == SWING_LIMIT_TYPE_ID
	{
		swing_limit_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == TWIST_SERVO_TYPE_ID
	{
		twist_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == TWIST_LIMIT_TYPE_ID
	{
		twist_limit_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == TWIST_MOTOR_TYPE_ID
	{
		twist_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_SERVO_TYPE_ID
	{
		angular_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_MOTOR_TYPE_ID
	{
		angular_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == WELD_TYPE_ID
	{
		weld_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == VOLUME_CONSTRAINT_TYPE_ID
	{
		volume_constraint_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == DISTANCE_SERVO_TYPE_ID
	{
		distance_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == DISTANCE_LIMIT_TYPE_ID
	{
		distance_limit_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == CENTER_DISTANCE_CONSTRAINT_TYPE_ID
	{
		center_distance_constraint_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == AREA_CONSTRAINT_TYPE_ID
	{
		area_constraint_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == POINT_ON_LINE_SERVO_TYPE_ID
	{
		point_on_line_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == LINEAR_AXIS_SERVO_TYPE_ID
	{
		linear_axis_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == LINEAR_AXIS_MOTOR_TYPE_ID
	{
		linear_axis_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == LINEAR_AXIS_LIMIT_TYPE_ID
	{
		linear_axis_limit_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_AXIS_MOTOR_TYPE_ID
	{
		angular_axis_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ONE_BODY_ANGULAR_SERVO_TYPE_ID
	{
		one_body_angular_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ONE_BODY_ANGULAR_MOTOR_TYPE_ID
	{
		one_body_angular_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
	{
		one_body_linear_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ONE_BODY_LINEAR_MOTOR_TYPE_ID
	{
		one_body_linear_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == SWIVEL_HINGE_TYPE_ID
	{
		swivel_hinge_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == HINGE_TYPE_ID
	{
		hinge_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == BALL_SOCKET_MOTOR_TYPE_ID
	{
		ball_socket_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == BALL_SOCKET_SERVO_TYPE_ID
	{
		ball_socket_servo_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID
	{
		angular_axis_gear_motor_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else when type_id == CENTER_DISTANCE_LIMIT_TYPE_ID
	{
		center_distance_limit_kernel(prestep, bodies, dt, inverse_dt, impulses, active_mask, phase);
	}
	else
	{
	}
}

solver_direct_noncontact_body_access :: #force_inline proc "contextless" (
	$type_id: i32, body_index: int, solve: bool,
) -> Body_Access_Mask
{
	when type_id == BALL_SOCKET_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ALL;
		}
		return BODY_ACCESS_NO_POSITION;
	}
	else when type_id == ANGULAR_HINGE_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		if body_index == 0
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else when type_id == ANGULAR_SWIVEL_HINGE_TYPE_ID ||
	type_id == SWING_LIMIT_TYPE_ID ||
	type_id == TWIST_SERVO_TYPE_ID ||
	type_id == TWIST_LIMIT_TYPE_ID ||
	type_id == TWIST_MOTOR_TYPE_ID
	{
		return BODY_ACCESS_ONLY_ANGULAR;
	}
	else when type_id == ANGULAR_SERVO_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else when type_id == ANGULAR_MOTOR_TYPE_ID
	{
		if !solve
		{
			return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
		}
		if body_index == 0
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else when type_id == WELD_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ALL;
		}
		if body_index == 0
		{
			return BODY_ACCESS_NO_POSITION;
		}
		return BODY_ACCESS_NO_POSE;
	}
	else when type_id == VOLUME_CONSTRAINT_TYPE_ID ||
	type_id == CENTER_DISTANCE_CONSTRAINT_TYPE_ID ||
	type_id == AREA_CONSTRAINT_TYPE_ID ||
	type_id == CENTER_DISTANCE_LIMIT_TYPE_ID
	{
		return BODY_ACCESS_ONLY_LINEAR;
	}
	else when type_id == DISTANCE_SERVO_TYPE_ID ||
	type_id == DISTANCE_LIMIT_TYPE_ID ||
	type_id == POINT_ON_LINE_SERVO_TYPE_ID ||
	type_id == LINEAR_AXIS_SERVO_TYPE_ID ||
	type_id == LINEAR_AXIS_MOTOR_TYPE_ID ||
	type_id == LINEAR_AXIS_LIMIT_TYPE_ID ||
	type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
	{
		return BODY_ACCESS_ALL;
	}
	else when type_id == ANGULAR_AXIS_MOTOR_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		if body_index == 0
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else when type_id == ONE_BODY_ANGULAR_SERVO_TYPE_ID
	{
		return BODY_ACCESS_ONLY_ANGULAR;
	}
	else when type_id == ONE_BODY_ANGULAR_MOTOR_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else when type_id == ONE_BODY_LINEAR_MOTOR_TYPE_ID
	{
		return BODY_ACCESS_NO_POSITION;
	}
	else when type_id == SWIVEL_HINGE_TYPE_ID ||
	type_id == HINGE_TYPE_ID ||
	type_id == BALL_SOCKET_SERVO_TYPE_ID
	{
		if solve
		{
			return BODY_ACCESS_ALL;
		}
		return BODY_ACCESS_NO_POSITION;
	}
	else when type_id == BALL_SOCKET_MOTOR_TYPE_ID
	{
		if solve || body_index != 0
		{
			return BODY_ACCESS_ALL;
		}
		return BODY_ACCESS_NO_ORIENTATION;
	}
	else when type_id == ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID
	{
		if body_index == 0
		{
			return BODY_ACCESS_ONLY_ANGULAR;
		}
		return BODY_ACCESS_ONLY_ANGULAR_WITHOUT_POSE;
	}
	else
	{
		return {};
	}
}

solver_execute_direct_noncontact_bundle_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int,
	$type_id: i32, $body_count: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	if stage == .Integrate_Constrained_Kinematics
	{
		return solver_integrate_type_bundle_kinematics(
			solver, type_batch, bundle_index, dt, phase, worker_index,
		);
	}
	first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	active_mask := solver_active_mask(first_index, int(type_batch.count));
	body_bundles := type_batch_body_bundle(type_batch, first_index);
	kernel_bodies: [4]Constraint_Kernel_Body_Wide;
	encoded_references: [4]util.I32x8;
	body_accesses: [4]Body_Access_Mask;
	for body_index in 0 ..< body_count
	{
		encoded := solver_mask_unused_body_lanes(
			body_bundles[body_index], first_index, int(type_batch.count),
		);
		encoded_references[body_index] = encoded;
		access := solver_direct_noncontact_body_access(
			type_id, body_index, stage == .Solve,
		);
		integration_mode := Solver_Bundle_Integration_Mode.None;
		integration_mask: util.I32x8;
		if stage == .Warmstart && velocity_integration != .Never
		{
			if batch_index == 0
			{
				integration_mode = .All;
				integration_mask = active_mask;
			}
			else
			{
				integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
		}
		body_accesses[body_index] = access;
		inertia_source := Inertia_Source.World;
		if integration_mode != .None
		{
			inertia_source = .Local;
		}
		position, orientation, velocity, inertia := bodies_gather_active_trusted(
			solver.bodies, encoded, inertia_source, access,
		);
		if integration_mode != .None
		{
			inertia = solver_integrate_constraint_body(
				solver, encoded, integration_mask, dt, phase, worker_index,
				&position, &orientation, &velocity, inertia,
			);
		}
		// write gathered fields directly, without a temporary 640-byte body
		body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
		body.position = position;
		body.orientation = orientation;
		body.linear_velocity = velocity.linear;
		body.angular_velocity = velocity.angular;
		body.inverse_mass = inertia.inverse_mass;
		body.inverse_inertia = inertia.inverse_inertia_tensor;
	}
	prestep := type_batch_prestep_bundle(type_batch, first_index);
	impulses := type_batch_impulse_bundle(type_batch, first_index);
	switch stage
	{
		case .Incremental_Update:
		if phase == .First
		{
			return .Ok;
		}
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask,
			.Incremental_Update,
		);
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities, .Integrate_Constrained_Kinematics:
		return .Invalid_Description;
		case .Warmstart:
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Prestep,
		);
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Warmstart,
		);
		case .Solve:
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Solve,
		);
		case .Integrate_Constrained_Poses, .Integrate_Unconstrained_After_Substepping:
		return .Invalid_Description;
	}
	for body_index in 0 ..< body_count
	{
		velocity := Body_Velocity_Wide{
			linear=kernel_bodies[body_index].linear_velocity,
			angular=kernel_bodies[body_index].angular_velocity,
		};
		bodies_scatter_active_velocities_trusted(
			solver.bodies, encoded_references[body_index], velocity,
			body_accesses[body_index],
		);
	}
	return .Ok;
}

solver_execute_convex_contact_bundle_stage :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, bundle_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
) -> Physics_Status
{
	first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	active_mask := solver_active_mask(first_index, int(type_batch.count));
	body_bundles := type_batch_body_bundle(type_batch, first_index);
	kernel_bodies: [body_count]Constraint_Contact_Body_Wide = ---

	when stage == .Incremental_Update
	{
		// incremental depth updates read velocity only and never write body state
		active := &solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		for body_index in 0 ..< body_count
		{
			encoded := solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			velocity: Body_Velocity_Wide = ---
			bodies_gather_velocity_kernel(
				active.dynamics_state.memory, encoded, &velocity,
			);
			kernel_bodies[body_index].linear_velocity = velocity.linear;
			kernel_bodies[body_index].angular_velocity = velocity.angular;
		}
		prestep := type_batch_prestep_bundle(type_batch, first_index);
		impulses := type_batch_impulse_bundle(type_batch, first_index);
		constraint_contact_convex_kernel(
			prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
			.Incremental_Update, body_count, contact_count,
		);
		return .Ok;
	}
	when stage == .Warmstart
	{
		encoded_references: [body_count]util.I32x8 = ---
		body_accesses: [body_count]Body_Access_Mask = ---
		for body_index in 0 ..< body_count
		{
			encoded := solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			encoded_references[body_index] = encoded;
			access := BODY_ACCESS_NO_POSE;
			bundle_integration_mode := Solver_Bundle_Integration_Mode.None;
			integration_mask: util.I32x8;
			when integration_mode == .Always
			{
				bundle_integration_mode = .All;
				integration_mask = active_mask;
			}
			else when integration_mode == .Conditional
			{
				bundle_integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if bundle_integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					bundle_integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
			body_accesses[body_index] = access;
			velocity: Body_Velocity_Wide = ---
			inertia: Body_Inertia_Wide = ---
			if bundle_integration_mode != .None
			{
				position: util.Vector3_Wide = ---
				orientation: util.Quaternion_Wide = ---
				position, orientation, velocity, inertia = bodies_gather_active_trusted(
					solver.bodies, encoded, .Local, BODY_ACCESS_ALL,
				);
				inertia = solver_integrate_constraint_body(
					solver, encoded, integration_mask, dt, phase, worker_index,
					&position, &orientation, &velocity, inertia,
				);
			}
			else
			{
				bodies_gather_active_no_pose_trusted(
					solver.bodies, encoded, .World, &velocity, &inertia,
				);
			}
			kernel_body := &kernel_bodies[body_index];
			kernel_body.linear_velocity = velocity.linear;
			kernel_body.angular_velocity = velocity.angular;
			kernel_body.inverse_mass = inertia.inverse_mass;
			kernel_body.inverse_inertia = inertia.inverse_inertia_tensor;
		}
		prestep := type_batch_prestep_bundle(type_batch, first_index);
		impulses := type_batch_impulse_bundle(type_batch, first_index);
		constraint_contact_convex_kernel(
			prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
			.Prestep, body_count, contact_count,
		);
		constraint_contact_convex_kernel(
			prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
			.Warmstart, body_count, contact_count,
		);
		for body_index in 0 ..< body_count
		{
			velocity := Body_Velocity_Wide{
				linear=kernel_bodies[body_index].linear_velocity,
				angular=kernel_bodies[body_index].angular_velocity,
			};
			bodies_scatter_active_velocities_trusted(
				solver.bodies, encoded_references[body_index], velocity,
				body_accesses[body_index],
			);
		}
		return .Ok;
	}
	when stage == .Solve
	{
		encoded_references: [body_count]util.I32x8 = ---
		for body_index in 0 ..< body_count
		{
			encoded := solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			encoded_references[body_index] = encoded;
			velocity: Body_Velocity_Wide = ---
			inertia: Body_Inertia_Wide = ---
			bodies_gather_active_no_pose_trusted(
				solver.bodies, encoded, .World, &velocity, &inertia,
			);
			kernel_body := &kernel_bodies[body_index];
			kernel_body.linear_velocity = velocity.linear;
			kernel_body.angular_velocity = velocity.angular;
			kernel_body.inverse_mass = inertia.inverse_mass;
			kernel_body.inverse_inertia = inertia.inverse_inertia_tensor;
		}
		prestep := type_batch_prestep_bundle(type_batch, first_index);
		impulses := type_batch_impulse_bundle(type_batch, first_index);
		constraint_contact_convex_kernel(
			prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
			.Solve, body_count, contact_count,
		);
		for body_index in 0 ..< body_count
		{
			velocity := Body_Velocity_Wide{
				linear=kernel_bodies[body_index].linear_velocity,
				angular=kernel_bodies[body_index].angular_velocity,
			};
			bodies_scatter_active_velocities_trusted(
				solver.bodies, encoded_references[body_index], velocity,
				BODY_ACCESS_NO_POSE,
			);
		}
		return .Ok;
	}
	return .Invalid_Description;
}

solver_execute_convex_contact_work_block_stage_mode :: #force_no_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
) -> Physics_Status
{
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		status := solver_execute_convex_contact_bundle_stage(
			job.solver, type_batch, bundle_index, job.dt, job.inverse_dt,
			job.phase, worker_index, stage, body_count, contact_count, integration_mode,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_convex_contact_bundle_stage_cached :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, bundle_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
	data: ^$Solve_Data,
) -> Physics_Status
{
	return solver_execute_convex_contact_bundle_stage_cached_response(
		solver, type_batch, bundle_index, dt, inverse_dt, phase, worker_index,
		stage, body_count, contact_count, integration_mode, data, nil, .Default,
	);
}

solver_execute_convex_contact_bundle_stage_cached_response :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, bundle_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
	data: ^$Solve_Data, targets: [^]Restitution_Target_Wide,
	$response: Solver_Response_Mode,
) -> Physics_Status
{
	#assert(response == .Default || integration_mode == .Never);
	first_index: int = bundle_index * util.PRODUCTION_LANE_COUNT;
	active_mask: util.I32x8 = solver_active_mask(first_index, int(type_batch.count));
	body_bundles: [^]util.I32x8 = type_batch_body_bundle(type_batch, first_index);
	kernel_bodies: [body_count]Constraint_Contact_Body_Wide = ---

	when stage == .Warmstart
	{
		encoded_references: [body_count]util.I32x8 = ---
		body_accesses: [body_count]Body_Access_Mask = ---
		for body_index in 0 ..< body_count
		{
			encoded: util.I32x8 = solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			encoded_references[body_index] = encoded;
			access: Body_Access_Mask = BODY_ACCESS_NO_POSE;
			bundle_integration_mode: Solver_Bundle_Integration_Mode = Solver_Bundle_Integration_Mode.None;
			integration_mask: util.I32x8;
			when integration_mode == .Always
			{
				bundle_integration_mode = .All;
				integration_mask = active_mask;
			}
			else when integration_mode == .Conditional
			{
				bundle_integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if bundle_integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					bundle_integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
			body_accesses[body_index] = access;
			velocity: Body_Velocity_Wide = ---
			inertia: Body_Inertia_Wide = ---
			if bundle_integration_mode != .None
			{
				position: util.Vector3_Wide = ---
				orientation: util.Quaternion_Wide = ---
				position, orientation, velocity, inertia = bodies_gather_active_trusted(
					solver.bodies, encoded, .Local, BODY_ACCESS_ALL,
				);
				inertia = solver_integrate_constraint_body(
					solver, encoded, integration_mask, dt, phase, worker_index,
					&position, &orientation, &velocity, inertia,
				);
			}
			else
			{
				bodies_gather_active_no_pose_trusted(
					solver.bodies, encoded, .World, &velocity, &inertia,
				);
			}
			kernel_body: ^Constraint_Contact_Body_Wide = &kernel_bodies[body_index];
			kernel_body.linear_velocity = velocity.linear;
			kernel_body.angular_velocity = velocity.angular;
			kernel_body.inverse_mass = inertia.inverse_mass;
			kernel_body.inverse_inertia = inertia.inverse_inertia_tensor;
		}
		prestep: rawptr = type_batch_prestep_bundle(type_batch, first_index);
		impulses: rawptr = type_batch_impulse_bundle(type_batch, first_index);
		constraint_contact_convex_kernel(
			prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
			.Prestep, body_count, contact_count,
		);
		when contact_count == 4
		{
			constraint_contact_convex_warmstart_cached(
				prestep, &kernel_bodies[0], impulses, active_mask, body_count, data,
			);
		}
		else
		{
			constraint_contact_convex_kernel(
				prestep, &kernel_bodies[0], dt, inverse_dt, impulses, active_mask,
				.Warmstart, body_count, contact_count,
			);
		}
		when integration_mode == .Conditional
		{
			if solver.integrator.callbacks.angular_mode != .Nonconserving
			{
				// the original Warmstart response is complete. its partial scalar integration
				// can leave local tensors in untouched lanes. Solve uses stored world tensors
				states: [^]Body_Dynamics = solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory;
				for body_index in 0 ..< body_count
				{
					world: Body_Inertia_Wide = ---;
					bodies_gather_inertia_kernel(states, encoded_references[body_index], .World, &world);
					body: ^Constraint_Contact_Body_Wide = &kernel_bodies[body_index];
					body.inverse_inertia.xx = world.inverse_inertia_tensor.xx;
					body.inverse_inertia.yx = world.inverse_inertia_tensor.yx;
					body.inverse_inertia.yy = world.inverse_inertia_tensor.yy;
					body.inverse_inertia.zx = world.inverse_inertia_tensor.zx;
					body.inverse_inertia.zy = world.inverse_inertia_tensor.zy;
					body.inverse_inertia.zz = world.inverse_inertia_tensor.zz;
				}
			}
		}
		constraint_contact_convex_prepare_solve_response(
			prestep, &kernel_bodies[0], dt, inverse_dt, body_count, contact_count, data, targets, response,
		);
		for body_index in 0 ..< body_count
		{
			velocity: Body_Velocity_Wide = Body_Velocity_Wide{
				linear=kernel_bodies[body_index].linear_velocity,
				angular=kernel_bodies[body_index].angular_velocity,
			};
			bodies_scatter_active_velocities_trusted(
				solver.bodies, encoded_references[body_index], velocity,
				body_accesses[body_index],
			);
		}
		return .Ok;
	}
	when stage == .Solve
	{
		encoded_references: [body_count]util.I32x8 = ---
		for body_index in 0 ..< body_count
		{
			encoded: util.I32x8 = solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			encoded_references[body_index] = encoded;
			velocity: Body_Velocity_Wide = ---
			inertia: Body_Inertia_Wide = ---
			bodies_gather_active_no_pose_trusted(
				solver.bodies, encoded, .World, &velocity, &inertia,
			);
			kernel_body: ^Constraint_Contact_Body_Wide = &kernel_bodies[body_index];
			kernel_body.linear_velocity = velocity.linear;
			kernel_body.angular_velocity = velocity.angular;
			kernel_body.inverse_mass = inertia.inverse_mass;
			kernel_body.inverse_inertia = inertia.inverse_inertia_tensor;
		}
		prestep: rawptr = type_batch_prestep_bundle(type_batch, first_index);
		impulses: rawptr = type_batch_impulse_bundle(type_batch, first_index);
		when contact_count == 4
		{
			constraint_contact_convex_solve_cached_response(
				prestep, &kernel_bodies[0], impulses, active_mask,
				body_count, contact_count, data, inverse_dt, targets, response,
			);
		}
		else
		{
			constraint_contact_convex_solve_cached_response(
				prestep, &kernel_bodies[0], impulses, active_mask,
				body_count, contact_count, data, inverse_dt, targets, response,
			);
		}
		for body_index in 0 ..< body_count
		{
			velocity: Body_Velocity_Wide = Body_Velocity_Wide{
				linear=kernel_bodies[body_index].linear_velocity,
				angular=kernel_bodies[body_index].angular_velocity,
			};
			bodies_scatter_active_velocities_trusted(
				solver.bodies, encoded_references[body_index], velocity,
				BODY_ACCESS_NO_POSE,
			);
		}
		return .Ok;
	}
	return .Invalid_Description;
}

solver_execute_convex_contact_work_block_stage_cached :: #force_no_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
	data: [^]$Solve_Data,
) -> Physics_Status
{
	return solver_execute_convex_contact_work_block_stage_cached_response(
		job, type_batch, block, worker_index, stage, body_count, contact_count,
		integration_mode, data, nil, .Default,
	);
}

solver_execute_convex_contact_work_block_stage_cached_response :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	worker_index: int,
	$stage: Solver_Execution_Stage,
	$body_count, $contact_count: int,
	$integration_mode: Solver_Integration_Mode,
	data: [^]$Solve_Data, targets: [^]Restitution_Target_Wide,
	$response: Solver_Response_Mode,
) -> Physics_Status
{
	when contact_count == 4
	{
		geometry_data: [^]Contact_4_Solve_Data = data;
		for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
		{
			bundle_targets: [^]Restitution_Target_Wide;
			when response == .Restitution
			{
				bundle_targets = &targets[(bundle_index-int(block.start_bundle))*contact_count];
			}
			status: Physics_Status = solver_execute_convex_contact_bundle_stage_cached_response(
				job.solver, type_batch, bundle_index, job.dt, job.inverse_dt,
				job.phase, worker_index, stage, body_count, contact_count, integration_mode,
				&geometry_data[bundle_index - int(block.start_bundle)], bundle_targets, response,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	else
	{
		for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
		{
			bundle_targets: [^]Restitution_Target_Wide;
			when response == .Restitution
			{
				bundle_targets = &targets[(bundle_index-int(block.start_bundle))*contact_count];
			}
			status: Physics_Status = solver_execute_convex_contact_bundle_stage_cached_response(
				job.solver, type_batch, bundle_index, job.dt, job.inverse_dt,
				job.phase, worker_index, stage, body_count, contact_count, integration_mode,
				&data[bundle_index - int(block.start_bundle)], bundle_targets, response,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
}

solver_integrate_convex_contact_work_block_kinematics :: #force_no_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	worker_index: int,
) -> Physics_Status
{
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		status := solver_integrate_type_bundle_kinematics(
			job.solver, type_batch, bundle_index, job.dt, job.phase, worker_index,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_convex_contact_work_block_integration :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$body_count, $contact_count: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	// this function receives only current normal blocks. one-iteration substeps
	// skip the optional map entirely, including after a cached substep
	if job.velocity_iteration_count > 1 && (stage == .Warmstart || stage == .Solve)
	{
		block_index: int = int((uintptr(block) - uintptr(job.solver.work_blocks.memory)) / size_of(Solver_Work_Block));
		offset: i32 = job.solver.contact_coefficient_offsets.memory[block_index];
		if offset >= 0
		{
			when contact_count == 4
			{
				data: [^]Contact_4_Solve_Data = ([^]Contact_4_Solve_Data)(&job.solver.contact_coefficients.memory[offset]);
				if stage == .Solve
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Solve,
						body_count, contact_count, Solver_Integration_Mode.Never, data,
					);
				}
				if velocity_integration != .Never && block.batch_index == 0
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
						body_count, contact_count, Solver_Integration_Mode.Always, data,
					);
				}
				if velocity_integration != .Never && type_batch.has_integration_responsibilities == .Present
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
						body_count, contact_count, Solver_Integration_Mode.Conditional, data,
					);
				}
				return solver_execute_convex_contact_work_block_stage_cached(
					job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
					body_count, contact_count, Solver_Integration_Mode.Never, data,
				);
			}
			else
			{
				data: [^]Contact_Convex_Solve_Data(contact_count) = ([^]Contact_Convex_Solve_Data(contact_count))(&job.solver.contact_coefficients.memory[offset]);
				if stage == .Solve
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Solve,
						body_count, contact_count, Solver_Integration_Mode.Never, data,
					);
				}
				if velocity_integration != .Never && block.batch_index == 0
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
						body_count, contact_count, Solver_Integration_Mode.Always, data,
					);
				}
				if velocity_integration != .Never && type_batch.has_integration_responsibilities == .Present
				{
					return solver_execute_convex_contact_work_block_stage_cached(
						job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
						body_count, contact_count, Solver_Integration_Mode.Conditional, data,
					);
				}
				return solver_execute_convex_contact_work_block_stage_cached(
					job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
					body_count, contact_count, Solver_Integration_Mode.Never, data,
				);
			}
		}
	}
	// select the stage once per block. each stage accesses only the data it needs
	// and performs only its own contact arithmetic
	switch stage
	{
		case .Incremental_Update:
		if job.phase == .First
		{
			return .Ok;
		}
		return solver_execute_convex_contact_work_block_stage_mode(
			job, type_batch, block, worker_index,
			Solver_Execution_Stage.Incremental_Update,
			body_count, contact_count, Solver_Integration_Mode.Never,
		);
		case .Warmstart:
		if velocity_integration != .Never && block.batch_index == 0
		{
			return solver_execute_convex_contact_work_block_stage_mode(
				job, type_batch, block, worker_index,
				Solver_Execution_Stage.Warmstart,
				body_count, contact_count, Solver_Integration_Mode.Always,
			);
		}
		if velocity_integration != .Never && type_batch.has_integration_responsibilities == .Present
		{
			return solver_execute_convex_contact_work_block_stage_mode(
				job, type_batch, block, worker_index,
				Solver_Execution_Stage.Warmstart,
				body_count, contact_count, Solver_Integration_Mode.Conditional,
			);
		}
		return solver_execute_convex_contact_work_block_stage_mode(
			job, type_batch, block, worker_index,
			Solver_Execution_Stage.Warmstart,
			body_count, contact_count, Solver_Integration_Mode.Never,
		);
		case .Solve:
		return solver_execute_convex_contact_work_block_stage_mode(
			job, type_batch, block, worker_index,
			Solver_Execution_Stage.Solve,
			body_count, contact_count, Solver_Integration_Mode.Never,
		);
		case .Integrate_Constrained_Kinematics:
		return solver_integrate_convex_contact_work_block_kinematics(
			job, type_batch, block, worker_index,
		);
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities,
		.Integrate_Constrained_Poses,
		.Integrate_Unconstrained_After_Substepping:
		return .Invalid_Description;
	}
	return .Invalid_Description;
}

solver_execute_ball_socket_work_block_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	worker_index: int, $stage: Solver_Execution_Stage,
	$integration_mode: Solver_Integration_Mode,
) -> Physics_Status
{
	solver: ^Solver = job.solver;
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		first_index: int = bundle_index * util.PRODUCTION_LANE_COUNT;
		active_mask: util.I32x8 = solver_active_mask(first_index, int(type_batch.count));
		body_bundles: [^]util.I32x8 = type_batch_body_bundle(type_batch, first_index);
		kernel_bodies: [2]Constraint_Kernel_Body_Wide = ---;
		encoded_references: [2]util.I32x8 = ---;
		body_accesses: [2]Body_Access_Mask = ---;
		for body_index in 0 ..< 2
		{
			encoded: util.I32x8 = solver_mask_unused_body_lanes(
				body_bundles[body_index], first_index, int(type_batch.count),
			);
			encoded_references[body_index] = encoded;
			access: Body_Access_Mask = BODY_ACCESS_ALL;
			kernel_body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
			velocity: Body_Velocity_Wide = ---;
			inertia: Body_Inertia_Wide = ---;
			when stage == .Warmstart
			{
				states: [^]Body_Dynamics = solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory;
				bodies_gather_motion_kernel(
					states, encoded, &kernel_body.position, &kernel_body.orientation, &velocity,
				);
				access = BODY_ACCESS_NO_POSITION;
				bundle_integration_mode: Solver_Bundle_Integration_Mode = .None;
				integration_mask: util.I32x8 = ---;
				when integration_mode == .Always
				{
					bundle_integration_mode = .All;
					integration_mask = active_mask;
				}
				else when integration_mode == .Conditional
				{
					bundle_integration_mode, integration_mask = solver_integration_mask(
						solver, type_batch, body_index, bundle_index, active_mask,
					);
				}
				if bundle_integration_mode != .None
				{
					integration_mask = solver_filter_integration_mobility(
						encoded, integration_mask, .Dynamic,
					);
					if solver_mask_has_lanes(integration_mask) == .Missing
					{
						bundle_integration_mode = .None;
					}
					else
					{
						access = BODY_ACCESS_ALL;
					}
				}
				if bundle_integration_mode != .None
				{
					bodies_gather_inertia_kernel(states, encoded, .Local, &inertia);
					inertia = solver_integrate_constraint_body(
						solver, encoded, integration_mask, job.dt, job.phase, worker_index,
						&kernel_body.position, &kernel_body.orientation, &velocity, inertia,
					);
				}
				else
				{
					bodies_gather_inertia_kernel(states, encoded, .World, &inertia);
				}
			}
			else
			{
				bodies_gather_active_no_pose_trusted(solver.bodies, encoded, .World, &velocity, &inertia);
			}
			body_accesses[body_index] = access;
			kernel_body.linear_velocity.x = velocity.linear.x;
			kernel_body.linear_velocity.y = velocity.linear.y;
			kernel_body.linear_velocity.z = velocity.linear.z;
			kernel_body.angular_velocity.x = velocity.angular.x;
			kernel_body.angular_velocity.y = velocity.angular.y;
			kernel_body.angular_velocity.z = velocity.angular.z;
			kernel_body.inverse_mass = inertia.inverse_mass;
			kernel_body.inverse_inertia.xx = inertia.inverse_inertia_tensor.xx;
			kernel_body.inverse_inertia.yx = inertia.inverse_inertia_tensor.yx;
			kernel_body.inverse_inertia.yy = inertia.inverse_inertia_tensor.yy;
			kernel_body.inverse_inertia.zx = inertia.inverse_inertia_tensor.zx;
			kernel_body.inverse_inertia.zy = inertia.inverse_inertia_tensor.zy;
			kernel_body.inverse_inertia.zz = inertia.inverse_inertia_tensor.zz;
		}
		prestep: ^Ball_Socket_Prestep = (^Ball_Socket_Prestep)(type_batch_prestep_bundle(type_batch, first_index));
		impulses: ^util.Vector3_Wide = (^util.Vector3_Wide)(type_batch_impulse_bundle(type_batch, first_index));
		when stage == .Warmstart
		{
			constraint_kernel_ball_socket(
				prestep, &kernel_bodies[0], &kernel_bodies[1], job.dt, impulses, active_mask, .Prestep,
			);
			warmstart_inertia: [2]util.Symmetric3x3_Wide = ---;
			_ = &warmstart_inertia;
			when integration_mode == .Conditional
			{
				if solver.integrator.callbacks.angular_mode != .Nonconserving
				{
					// partial scalar integration leaves local tensors in nonintegrated lanes.
					// preserve that Warmstart behavior. cached Solve needs stored world tensors
					states: [^]Body_Dynamics = solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory;
					for body_index in 0 ..< 2
					{
						body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
						warmstart_inertia[body_index] = body.inverse_inertia;
						world: Body_Inertia_Wide = ---;
						bodies_gather_inertia_kernel(states, encoded_references[body_index], .World, &world);
						body.inverse_inertia.xx = world.inverse_inertia_tensor.xx;
						body.inverse_inertia.yx = world.inverse_inertia_tensor.yx;
						body.inverse_inertia.yy = world.inverse_inertia_tensor.yy;
						body.inverse_inertia.zx = world.inverse_inertia_tensor.zx;
						body.inverse_inertia.zy = world.inverse_inertia_tensor.zy;
						body.inverse_inertia.zz = world.inverse_inertia_tensor.zz;
					}
				}
			}
			constraint_kernel_ball_socket_prepare_solve(
				prestep, &kernel_bodies[0], &kernel_bodies[1], job.dt, &prestep.solve_data,
			);
			when integration_mode == .Conditional
			{
				if solver.integrator.callbacks.angular_mode != .Nonconserving
				{
					for body_index in 0 ..< 2
					{
						body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
						body.inverse_inertia.xx = warmstart_inertia[body_index].xx;
						body.inverse_inertia.yx = warmstart_inertia[body_index].yx;
						body.inverse_inertia.yy = warmstart_inertia[body_index].yy;
						body.inverse_inertia.zx = warmstart_inertia[body_index].zx;
						body.inverse_inertia.zy = warmstart_inertia[body_index].zy;
						body.inverse_inertia.zz = warmstart_inertia[body_index].zz;
					}
				}
			}
			constraint_kernel_apply_ball_socket(
				&kernel_bodies[0], &kernel_bodies[1], prestep.solve_data.offset_a,
				prestep.solve_data.offset_b, impulses^, active_mask,
			);
		}
		else when stage == .Solve
		{
			data: ^Ball_Socket_Solve_Data = &prestep.solve_data;
			constraint_kernel_ball_socket_solve(
				&kernel_bodies[0], &kernel_bodies[1], data.offset_a, data.offset_b, data.bias,
				data.effective_mass, data.softness, {}, impulses, active_mask, .Missing,
			);
		}
		for body_index in 0 ..< 2
		{
			velocity: Body_Velocity_Wide = {
				linear=kernel_bodies[body_index].linear_velocity,
				angular=kernel_bodies[body_index].angular_velocity,
			};
			bodies_scatter_active_velocities_trusted(
				solver.bodies, encoded_references[body_index], velocity, body_accesses[body_index],
			);
		}
	}
	return .Ok;
}

solver_execute_ball_socket_work_block_integration :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	switch stage
	{
		case .Incremental_Update:
		return .Ok;
		case .Warmstart:
		if velocity_integration != .Never && block.batch_index == 0
		{
			return solver_execute_ball_socket_work_block_stage(
				job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
				Solver_Integration_Mode.Always,
			);
		}
		if velocity_integration != .Never && type_batch.has_integration_responsibilities == .Present
		{
			return solver_execute_ball_socket_work_block_stage(
				job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
				Solver_Integration_Mode.Conditional,
			);
		}
		return solver_execute_ball_socket_work_block_stage(
			job, type_batch, block, worker_index, Solver_Execution_Stage.Warmstart,
			Solver_Integration_Mode.Never,
		);
		case .Solve:
		return solver_execute_ball_socket_work_block_stage(
			job, type_batch, block, worker_index, Solver_Execution_Stage.Solve,
			Solver_Integration_Mode.Never,
		);
		case .Integrate_Constrained_Kinematics:
		for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
		{
			status: Physics_Status = solver_integrate_type_bundle_kinematics(
				job.solver, type_batch, bundle_index, job.dt, job.phase, worker_index,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities, .Integrate_Constrained_Poses,
		.Integrate_Unconstrained_After_Substepping:
		return .Invalid_Description;
	}
	return .Invalid_Description;
}

solver_execute_direct_noncontact_work_block_integration :: proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$type_id: i32, $body_count: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	for bundle_index in int(block.start_bundle) ..< int(block.end_bundle)
	{
		status: Physics_Status = solver_execute_direct_noncontact_bundle_integration(
			job.solver, type_batch, int(block.batch_index), bundle_index,
			job.dt, job.inverse_dt, job.phase, stage, worker_index, type_id, body_count, velocity_integration,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_type_lane_dispatch_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage,
	worker_index: int = 0,
	$callback_dispatch: Constraint_Callback_Dispatch,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	bundle_index := constraint_index / util.PRODUCTION_LANE_COUNT;
	bundle_first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	lane := constraint_index % util.PRODUCTION_LANE_COUNT;
	mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	mask_values[lane] = -1;
	active_mask := transmute(util.I32x8)mask_values;
	body_bundles := type_batch_body_bundle(type_batch, bundle_first_index);
	kernel_bodies: [4]Constraint_Kernel_Body_Wide;
	encoded_references: [4]util.I32x8;
	body_accesses: [4]Body_Access_Mask;
	for body_index in 0 ..< int(type_batch.body_count)
	{
		encoded := body_bundles[body_index];
		for other_lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if other_lane != lane
			{
				encoded = simd.replace(
					encoded, other_lane, i32(BODY_REFERENCE_KINEMATIC_MASK),
				);
			}
		}
		encoded_references[body_index] = encoded;
		access := record.solve_access[body_index];
		if stage != .Solve
		{
			access = record.initial_access[body_index];
		}
		integration_mode := Solver_Bundle_Integration_Mode.None;
		integration_mask: util.I32x8;
		if stage == .Warmstart && velocity_integration != .Never
		{
			if batch_index == 0
			{
				integration_mode = .All;
				integration_mask = active_mask;
			}
			else
			{
				integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
		}
		body_accesses[body_index] = access;
		inertia_source := Inertia_Source.World;
		if integration_mode != .None
		{
			inertia_source = .Local;
		}
		position, orientation, velocity, inertia := bodies_gather_active_trusted(
			solver.bodies, encoded, inertia_source, access,
		);
		if integration_mode != .None
		{
			inertia = solver_integrate_constraint_body(
				solver, encoded, integration_mask, dt, phase, worker_index,
				&position, &orientation, &velocity, inertia,
			);
		}
		// write gathered fields directly, without a temporary 640-byte body
		body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
		body.position = position;
		body.orientation = orientation;
		body.linear_velocity = velocity.linear;
		body.angular_velocity = velocity.angular;
		body.inverse_mass = inertia.inverse_mass;
		body.inverse_inertia = inertia.inverse_inertia_tensor;
	}
	prestep := type_batch_prestep_bundle(type_batch, constraint_index);
	impulses := type_batch_impulse_bundle(type_batch, constraint_index);
	when callback_dispatch == .Contextual
	{
		if stage == .Incremental_Update && phase == .First
		{
			return .Ok;
		}
		status: Physics_Status = constraint_contextual_execute_kernel(
			record.contextual, prestep, &kernel_bodies, dt, inverse_dt,
			impulses, &active_mask, phase, stage,
		);
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		switch stage
		{
			case .Incremental_Update:
			if phase == .First
			{
				return .Ok;
			}
			record.incrementally_update(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask,
				.Incremental_Update,
			);
			case .Integrate_Constrained_Dynamics, .Capture_Restitution,
			.Prepare_Integration_Responsibilities, .Integrate_Constrained_Kinematics:
			return .Invalid_Description;
			case .Warmstart:
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Prestep,
			);
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Warmstart,
			);
			case .Solve:
			record.prestep_warmstart_solve(
				prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Solve,
			);
			case .Integrate_Constrained_Poses, .Integrate_Unconstrained_After_Substepping:
			return .Invalid_Description;
		}
	}
	for body_index in 0 ..< int(type_batch.body_count)
	{
		velocity := Body_Velocity_Wide{
			linear=kernel_bodies[body_index].linear_velocity,
			angular=kernel_bodies[body_index].angular_velocity,
		};
		bodies_scatter_active_velocities_trusted(
			solver.bodies, encoded_references[body_index], velocity,
			body_accesses[body_index],
		);
	}
	return .Ok;
}

solver_execute_direct_noncontact_lane_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage,
	worker_index: int, $type_id: i32, $body_count: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	bundle_index := constraint_index / util.PRODUCTION_LANE_COUNT;
	bundle_first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	lane := constraint_index % util.PRODUCTION_LANE_COUNT;
	mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	mask_values[lane] = -1;
	active_mask := transmute(util.I32x8)mask_values;
	body_bundles := type_batch_body_bundle(type_batch, bundle_first_index);
	kernel_bodies: [4]Constraint_Kernel_Body_Wide;
	encoded_references: [4]util.I32x8;
	body_accesses: [4]Body_Access_Mask;
	for body_index in 0 ..< body_count
	{
		encoded := body_bundles[body_index];
		for other_lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if other_lane != lane
			{
				encoded = simd.replace(
					encoded, other_lane, i32(BODY_REFERENCE_KINEMATIC_MASK),
				);
			}
		}
		encoded_references[body_index] = encoded;
		access := solver_direct_noncontact_body_access(
			type_id, body_index, stage == .Solve,
		);
		integration_mode := Solver_Bundle_Integration_Mode.None;
		integration_mask: util.I32x8;
		if stage == .Warmstart && velocity_integration != .Never
		{
			if batch_index == 0
			{
				integration_mode = .All;
				integration_mask = active_mask;
			}
			else
			{
				integration_mode, integration_mask = solver_integration_mask(
					solver, type_batch, body_index, bundle_index, active_mask,
				);
			}
			if integration_mode != .None
			{
				integration_mask = solver_filter_integration_mobility(
					encoded, integration_mask, .Dynamic,
				);
				if solver_mask_has_lanes(integration_mask) == .Missing
				{
					integration_mode = .None;
				}
				else
				{
					access = BODY_ACCESS_ALL;
				}
			}
		}
		body_accesses[body_index] = access;
		inertia_source := Inertia_Source.World;
		if integration_mode != .None
		{
			inertia_source = .Local;
		}
		position, orientation, velocity, inertia := bodies_gather_active_trusted(
			solver.bodies, encoded, inertia_source, access,
		);
		if integration_mode != .None
		{
			inertia = solver_integrate_constraint_body(
				solver, encoded, integration_mask, dt, phase, worker_index,
				&position, &orientation, &velocity, inertia,
			);
		}
		// write gathered fields directly, without a temporary 640-byte body
		body: ^Constraint_Kernel_Body_Wide = &kernel_bodies[body_index];
		body.position = position;
		body.orientation = orientation;
		body.linear_velocity = velocity.linear;
		body.angular_velocity = velocity.angular;
		body.inverse_mass = inertia.inverse_mass;
		body.inverse_inertia = inertia.inverse_inertia_tensor;
	}
	prestep := type_batch_prestep_bundle(type_batch, constraint_index);
	impulses := type_batch_impulse_bundle(type_batch, constraint_index);
	switch stage
	{
		case .Incremental_Update:
		if phase == .First
		{
			return .Ok;
		}
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask,
			.Incremental_Update,
		);
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities, .Integrate_Constrained_Kinematics:
		return .Invalid_Description;
		case .Warmstart:
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Prestep,
		);
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Warmstart,
		);
		case .Solve:
		solver_execute_direct_noncontact_kernel(
			type_id, prestep, &kernel_bodies, dt, inverse_dt, impulses, active_mask, .Solve,
		);
		case .Integrate_Constrained_Poses, .Integrate_Unconstrained_After_Substepping:
		return .Invalid_Description;
	}
	for body_index in 0 ..< body_count
	{
		velocity := Body_Velocity_Wide{
			linear=kernel_bodies[body_index].linear_velocity,
			angular=kernel_bodies[body_index].angular_velocity,
		};
		bodies_scatter_active_velocities_trusted(
			solver.bodies, encoded_references[body_index], velocity,
			body_accesses[body_index],
		);
	}
	return .Ok;
}

solver_execute_direct_noncontact_fallback_batch_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, batch_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, $type_id: i32, $body_count: int,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	bundle_count :=
	(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
	util.PRODUCTION_LANE_COUNT;
	if stage == .Integrate_Constrained_Kinematics
	{
		for bundle_index in 0 ..< bundle_count
		{
			status := solver_integrate_type_bundle_kinematics(
				solver, type_batch, bundle_index, dt, phase, 0,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	for constraint_index in 0 ..< int(type_batch.count)
	{
		status: Physics_Status = solver_execute_direct_noncontact_lane_integration(
			solver, type_batch, batch_index, constraint_index,
			dt, inverse_dt, phase, stage, 0, type_id, body_count, velocity_integration,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_type_batch_integration :: proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, batch_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, sequential: Reference_State,
	$velocity_integration: Solver_Integration_Mode,
) -> Physics_Status
{
	if sequential == .Present
	{
		switch type_batch.type_id
		{
			case BALL_SOCKET_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				BALL_SOCKET_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_HINGE_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_HINGE_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_SWIVEL_HINGE_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_SWIVEL_HINGE_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case SWING_LIMIT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				SWING_LIMIT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case TWIST_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				TWIST_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case TWIST_LIMIT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				TWIST_LIMIT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case TWIST_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				TWIST_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case WELD_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				WELD_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case VOLUME_CONSTRAINT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				VOLUME_CONSTRAINT_TYPE_ID,
				4, velocity_integration=velocity_integration,
			);
			case DISTANCE_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				DISTANCE_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case DISTANCE_LIMIT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				DISTANCE_LIMIT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case CENTER_DISTANCE_CONSTRAINT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				CENTER_DISTANCE_CONSTRAINT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case AREA_CONSTRAINT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				AREA_CONSTRAINT_TYPE_ID,
				3, velocity_integration=velocity_integration,
			);
			case POINT_ON_LINE_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				POINT_ON_LINE_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case LINEAR_AXIS_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				LINEAR_AXIS_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case LINEAR_AXIS_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				LINEAR_AXIS_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case LINEAR_AXIS_LIMIT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				LINEAR_AXIS_LIMIT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_AXIS_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_AXIS_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ONE_BODY_ANGULAR_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ONE_BODY_ANGULAR_SERVO_TYPE_ID,
				1, velocity_integration=velocity_integration,
			);
			case ONE_BODY_ANGULAR_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ONE_BODY_ANGULAR_MOTOR_TYPE_ID,
				1, velocity_integration=velocity_integration,
			);
			case ONE_BODY_LINEAR_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ONE_BODY_LINEAR_SERVO_TYPE_ID,
				1, velocity_integration=velocity_integration,
			);
			case ONE_BODY_LINEAR_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ONE_BODY_LINEAR_MOTOR_TYPE_ID,
				1, velocity_integration=velocity_integration,
			);
			case SWIVEL_HINGE_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				SWIVEL_HINGE_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case HINGE_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				HINGE_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case BALL_SOCKET_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				BALL_SOCKET_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case BALL_SOCKET_SERVO_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				BALL_SOCKET_SERVO_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case CENTER_DISTANCE_LIMIT_TYPE_ID:
			return solver_execute_direct_noncontact_fallback_batch_integration(
				solver,
				type_batch,
				batch_index,
				dt,
				inverse_dt,
				phase,
				stage,
				CENTER_DISTANCE_LIMIT_TYPE_ID,
				2, velocity_integration=velocity_integration,
			);
			case:
		}
	}
	record, status := constraint_type_registry_lookup(
		&solver.registry, type_batch.type_id,
	);
	if status != .Ok
	{
		return status;
	}
	bundle_count :=
	(int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
	util.PRODUCTION_LANE_COUNT;
	if stage == .Integrate_Constrained_Kinematics
	{
		for bundle_index in 0 ..< bundle_count
		{
			status = solver_integrate_type_bundle_kinematics(
				solver, type_batch, bundle_index, dt, phase, 0,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	if record.dispatch == .Contextual
	{
		return solver_execute_contextual_type_batch_integration(
			solver, type_batch, record, batch_index, dt, inverse_dt, phase, stage, sequential, velocity_integration=velocity_integration,
		);
	}
	if sequential == .Missing
	{
		for bundle_index in 0 ..< bundle_count
		{
			status = solver_execute_type_bundle_integration(
				solver, type_batch, record, batch_index, bundle_index,
				dt, inverse_dt, phase, stage, velocity_integration=velocity_integration,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
	}
	// fallback constraints may share bodies, so each lane is solved and scattered in source order
	for constraint_index in 0 ..< int(type_batch.count)
	{
		status = solver_execute_type_lane_integration(
			solver, type_batch, record, batch_index, constraint_index,
			dt, inverse_dt, phase, stage, velocity_integration=velocity_integration,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_post_solve_bundle :: proc "contextless" (
	job: ^Solver_Substep_Job, bundle_index: int,
	stage: Solver_Execution_Stage, worker_index: int,
) -> Physics_Status
{
	first_index := bundle_index * util.PRODUCTION_LANE_COUNT;
	switch stage
	{
		case .Integrate_Constrained_Poses:
		active := &job.solver.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			body_index := first_index + lane;
			if body_index >= active.count
			{
				break;
			}
			status := pose_integrator_integrate_pose_body(
				job.solver.integrator, body_index, job.dt, .Present,
			);
			if status != .Ok
			{
				return status;
			}
		}
		return .Ok;
		case .Integrate_Unconstrained_After_Substepping:
		return pose_integrator_integrate_after_substepping_bundle(
			job.solver.integrator, first_index,
			job.post_solve_callback_dt,
			job.post_solve_callback_substep_count,
			worker_index,
		);
		case .Integrate_Constrained_Dynamics, .Capture_Restitution,
		.Prepare_Integration_Responsibilities, .Incremental_Update,
		.Integrate_Constrained_Kinematics, .Warmstart, .Solve:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

solver_execute_post_solve_job :: proc "contextless" (
	job: ^Solver_Substep_Job, job_index: int,
	stage: Solver_Execution_Stage, worker_index: int,
) -> Physics_Status
{
	start_bundle := job.post_solve_bundle_count * job_index / job.post_solve_job_count;
	end_bundle := job.post_solve_bundle_count * (job_index + 1) / job.post_solve_job_count;
	for bundle_index in start_bundle ..< end_bundle
	{
		status := solver_execute_post_solve_bundle(
			job, bundle_index, stage, worker_index,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

solver_execute_post_solve_worker_stage :: proc "contextless" (
	job: ^Solver_Substep_Job, stage: Solver_Execution_Stage, worker_index: int,
)
{
	completed_count := 0;
	for
	{
		job_index := int(sync.atomic_add_explicit(
				&job.post_solve_cursor.value, i32(1), .Relaxed,
		));
		if job_index >= job.post_solve_job_count
		{
			break;
		}
		if Physics_Status(sync.atomic_load_explicit(&job.status, .Acquire)) == .Ok
		{
			status := solver_execute_post_solve_job(
				job, job_index, stage, worker_index,
			);
			solver_job_set_status(job, status);
		}
		completed_count += 1;
	}
	// workers must leave this loop before the barrier completes: the next
	// post-solve stage resets the shared cursor
	_ = sync.atomic_add_explicit(
		&job.completion.value, i32(completed_count + 1), .Release,
	);
}

solver_execute_type_bundle_dispatch :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int = 0,
	$callback_dispatch: Constraint_Callback_Dispatch,
) -> Physics_Status
{
	return solver_execute_type_bundle_dispatch_integration(solver, type_batch, record, batch_index, bundle_index, dt, inverse_dt, phase, stage, worker_index, callback_dispatch, .Conditional);
}

solver_execute_direct_noncontact_bundle :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch,
	batch_index, bundle_index: int, dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, worker_index: int,
	$type_id: i32, $body_count: int,
) -> Physics_Status
{
	return solver_execute_direct_noncontact_bundle_integration(solver, type_batch, batch_index, bundle_index, dt, inverse_dt, phase, stage, worker_index, type_id, body_count, .Conditional);
}

solver_execute_convex_contact_work_block :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$body_count, $contact_count: int,
) -> Physics_Status
{
	return solver_execute_convex_contact_work_block_integration(job, type_batch, block, stage, worker_index, body_count, contact_count, .Conditional);
}

solver_execute_ball_socket_work_block :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
) -> Physics_Status
{
	return solver_execute_ball_socket_work_block_integration(job, type_batch, block, stage, worker_index, .Conditional);
}

solver_execute_direct_noncontact_work_block :: #force_inline proc "contextless" (
	job: ^Solver_Substep_Job, type_batch: ^Type_Batch, block: ^Solver_Work_Block,
	stage: Solver_Execution_Stage, worker_index: int,
	$type_id: i32, $body_count: int,
) -> Physics_Status
{
	return solver_execute_direct_noncontact_work_block_integration(job, type_batch, block, stage, worker_index, type_id, body_count, .Conditional);
}

solver_execute_type_lane_dispatch :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, record: ^Constraint_Type_Record,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage,
	worker_index: int = 0,
	$callback_dispatch: Constraint_Callback_Dispatch,
) -> Physics_Status
{
	return solver_execute_type_lane_dispatch_integration(solver, type_batch, record, batch_index, constraint_index, dt, inverse_dt, phase, stage, worker_index, callback_dispatch, .Conditional);
}

solver_execute_direct_noncontact_lane :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch,
	batch_index, constraint_index: int, dt, inverse_dt: f32,
	phase: Solver_Substep_Phase, stage: Solver_Execution_Stage,
	worker_index: int, $type_id: i32, $body_count: int,
) -> Physics_Status
{
	return solver_execute_direct_noncontact_lane_integration(solver, type_batch, batch_index, constraint_index, dt, inverse_dt, phase, stage, worker_index, type_id, body_count, .Conditional);
}

solver_execute_direct_noncontact_fallback_batch :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, batch_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, $type_id: i32, $body_count: int,
) -> Physics_Status
{
	return solver_execute_direct_noncontact_fallback_batch_integration(solver, type_batch, batch_index, dt, inverse_dt, phase, stage, type_id, body_count, .Conditional);
}

solver_execute_type_batch :: #force_inline proc "contextless" (
	solver: ^Solver, type_batch: ^Type_Batch, batch_index: int,
	dt, inverse_dt: f32, phase: Solver_Substep_Phase,
	stage: Solver_Execution_Stage, sequential: Reference_State,
) -> Physics_Status
{
	return solver_execute_type_batch_integration(solver, type_batch, batch_index, dt, inverse_dt, phase, stage, sequential, .Conditional);
}
