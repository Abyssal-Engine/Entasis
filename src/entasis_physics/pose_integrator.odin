// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"
import "core:math"

Angular_Integration_Mode :: enum u8
{
	Nonconserving,
	Conserve_Momentum,
	Conserve_Momentum_With_Gyroscopic_Torque,
}

Velocity_Integration_State :: enum u8
{
	Disabled,
	Enabled,
}

Pose_Integrator_Initialize_Proc :: #type proc "contextless" (
	user_context: rawptr, simulation: ^Simulation,
) -> Physics_Status;
Pose_Integrator_Prepare_Proc :: #type proc "contextless" (user_context: rawptr, dt: f32) -> Physics_Status;
Pose_Integrator_Velocity_Proc :: #type proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^Body_Velocity_Wide,
);
Pose_Integrator_Dispose_Proc :: #type proc "contextless" (user_context: rawptr);

Pose_Integrator_Callbacks :: struct
{
	initialize:                  Pose_Integrator_Initialize_Proc,
	prepare_for_integration:     Pose_Integrator_Prepare_Proc,
	integrate_velocity:          Pose_Integrator_Velocity_Proc,
	dispose:                     Pose_Integrator_Dispose_Proc,
	angular_mode:                Angular_Integration_Mode,
	allow_substeps_for_unconstrained: Velocity_Integration_State,
	integrate_kinematic_velocity: Velocity_Integration_State,
	user_context:                rawptr,
}

Default_Pose_Integrator_Context :: struct
{
	gravity:             util.Vector3,
	linear_damping:      f32,
	angular_damping:     f32,
	gravity_wide_dt:     util.Vector3_Wide,
	linear_damping_dt:   util.F32x8,
	angular_damping_dt:  util.F32x8,
}

Pose_Integrator_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

Pose_Integrator :: struct
{
	callbacks:           Pose_Integrator_Callbacks,
	bodies:              ^Bodies,
	callback_activation: Reference_State,
	state:               Pose_Integrator_State,
}

pose_integrator_default_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^Simulation,
) -> Physics_Status
{
	_ = simulation;
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	value := (^Default_Pose_Integrator_Context)(user_context);
	if value.linear_damping < 0 || value.linear_damping > 1 || value.angular_damping < 0 || value.angular_damping > 1
	{
		return .Invalid_Description;
	}
	return .Ok;
}

pose_integrator_default_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> Physics_Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	value := (^Default_Pose_Integrator_Context)(user_context);
	linear_damping_base := clamp(1 - value.linear_damping, f32(0), f32(1));
	angular_damping_base := clamp(1 - value.angular_damping, f32(0), f32(1));
	value.gravity_wide_dt = util.vector3_wide_broadcast(util.vector3_scale(value.gravity, dt));
	value.linear_damping_dt = util.F32x8(f32(math.pow(f64(linear_damping_base), f64(dt))));
	value.angular_damping_dt = util.F32x8(f32(math.pow(f64(angular_damping_base), f64(dt))));
	return .Ok;
}

pose_integrator_default_velocity :: #force_inline proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^Body_Velocity_Wide,
)
{
	_, _, _, _, _, _, _ = body_indices, position, orientation, inertia, integration_mask, worker_index, dt;
	value := (^Default_Pose_Integrator_Context)(user_context);
	velocity.linear = util.vector3_wide_scale(
		util.vector3_wide_add(velocity.linear, value.gravity_wide_dt),
		value.linear_damping_dt,
	);
	velocity.angular = util.vector3_wide_scale(velocity.angular, value.angular_damping_dt);
}

pose_integrator_default_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}

pose_integrator_default_callbacks :: proc "contextless" (
	user_context: ^Default_Pose_Integrator_Context,
) -> Pose_Integrator_Callbacks
{
	return {
		initialize=pose_integrator_default_initialize,
		prepare_for_integration=pose_integrator_default_prepare,
		integrate_velocity=pose_integrator_default_velocity,
		dispose=pose_integrator_default_dispose,
		angular_mode=.Nonconserving,
		allow_substeps_for_unconstrained=.Disabled,
		integrate_kinematic_velocity=.Disabled,
		user_context=user_context,
	};
}

pose_integrator_callbacks_validate :: proc "contextless" (callbacks: Pose_Integrator_Callbacks) -> Physics_Status
{
	if callbacks.initialize == nil || callbacks.prepare_for_integration == nil ||
		callbacks.integrate_velocity == nil || callbacks.dispose == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

pose_integrator_initialize :: proc "contextless" (
	integrator: ^Pose_Integrator, bodies: ^Bodies, callbacks: Pose_Integrator_Callbacks,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Uninitialized || bodies == nil || bodies.state != .Allocated ||
		pose_integrator_callbacks_validate(callbacks) != .Ok
	{
		return .Invalid_Argument;
	}
	integrator^ = {callbacks=callbacks, bodies=bodies, state=.Ready};
	return .Ok;
}

pose_integrator_activate :: proc "contextless" (
	integrator: ^Pose_Integrator, simulation: ^Simulation,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready ||
		integrator.callback_activation != .Missing ||
		simulation == nil || simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	status := integrator.callbacks.initialize(
		integrator.callbacks.user_context, simulation,
	);
	if status != .Ok
	{
		return status;
	}
	integrator.callback_activation = .Present;
	return .Ok;
}

pose_integrator_update_world_inertia :: proc "contextless" (dynamics: ^Body_Dynamics)
{
	local := dynamics.inertia.local;
	dynamics.inertia.world.inverse_mass = local.inverse_mass;
	if body_inertia_mobility(local) == .Kinematic
	{
		dynamics.inertia.world.inverse_inertia_tensor = {};
		return;
	}
	rotation := util.matrix3x3_from_quaternion(dynamics.motion.pose.orientation);
	dynamics.inertia.world.inverse_inertia_tensor = util.symmetric3x3_rotation_sandwich(
		rotation,
		local.inverse_inertia_tensor
	);
}

pose_integrator_integrate_orientation :: proc "contextless" (
	orientation: util.Quaternion, angular_velocity: util.Vector3, dt: f32,
) -> util.Quaternion
{
	speed := util.vector3_length(angular_velocity);
	if speed <= 1e-15
	{
		return orientation;
	}
	half_angle := speed * dt * 0.5;
	scale := f32(math.sin(f64(half_angle))) / speed;
	delta := util.Quaternion{
		angular_velocity.x * scale,
		angular_velocity.y * scale,
		angular_velocity.z * scale,
		f32(math.cos(f64(half_angle))),
	};
	return util.quaternion_normalize(util.quaternion_concatenate(orientation, delta));
}

pose_integrator_prepare :: proc "contextless" (integrator: ^Pose_Integrator, dt: f32) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready || dt <= 0
	{
		return .Invalid_Argument;
	}
	return integrator.callbacks.prepare_for_integration(integrator.callbacks.user_context, dt);
}

pose_integrator_predict_velocity :: proc "contextless" (
	integrator: ^Pose_Integrator, body_index: int, dt: f32, velocity: ^Body_Velocity, worker_index: int = 0,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready || velocity == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if body_index < 0 || body_index >= active.count
	{
		return .Not_Found;
	}
	dynamics := &active.dynamics_state.memory[body_index];
	if body_inertia_mobility(dynamics.inertia.local) == .Kinematic &&
		integrator.callbacks.integrate_kinematic_velocity == .Disabled
	{
		return .Ok;
	}
	indices_values := [util.PRODUCTION_LANE_COUNT]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	indices_values[0] = i32(body_index);
	indices := transmute(util.I32x8)indices_values;
	position, orientation, _, inertia, gather_status := bodies_gather_active(
		integrator.bodies, indices, .Local, BODY_ACCESS_ALL,
	);
	if gather_status != .Ok
	{
		return gather_status;
	}
	wide_velocity: Body_Velocity_Wide;
	util.vector3_wide_write_slot(&wide_velocity.linear, 0, velocity.linear);
	util.vector3_wide_write_slot(&wide_velocity.angular, 0, velocity.angular);
	mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	mask_values[0] = -1;
	integrator.callbacks.integrate_velocity(
		integrator.callbacks.user_context, indices, position, orientation, inertia,
		transmute(util.I32x8)mask_values, worker_index, util.F32x8(dt), &wide_velocity,
	);
	velocity.linear = util.vector3_wide_read_slot(wide_velocity.linear, 0);
	velocity.angular = util.vector3_wide_read_slot(wide_velocity.angular, 0);
	return .Ok;
}

pose_integrator_integrate_velocity_bundle :: proc "contextless" (
	integrator: ^Pose_Integrator, first_index: int, dt: f32,
	constrained: Reference_State, worker_index: int,
) -> Physics_Status
{
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	gather_values := [util.PRODUCTION_LANE_COUNT]i32{-1, -1, -1, -1, -1, -1, -1, -1};
	scatter_values := gather_values;
	mask_values: [util.PRODUCTION_LANE_COUNT]i32;
	active_lane_count := 0;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		index := first_index + lane;
		if index >= active.count
		{
			continue;
		}
		is_constrained := active.constraints.memory[index].count > 0;
		if (constrained == .Present) != is_constrained
		{
			continue;
		}
		dynamics := &active.dynamics_state.memory[index];
		pose_integrator_update_world_inertia(dynamics);
		if body_inertia_mobility(dynamics.inertia.local) == .Kinematic &&
			integrator.callbacks.integrate_kinematic_velocity == .Disabled
		{
			continue;
		}
		gather_values[lane] = i32(index);
		scatter_values[lane] = i32(index);
		mask_values[lane] = -1;
		active_lane_count += 1;
	}
	if active_lane_count == 0
	{
		return .Ok;
	}
	gather_indices := transmute(util.I32x8)gather_values;
	position, orientation, velocity, inertia, gather_status := bodies_gather_active(
		integrator.bodies, gather_indices, .Local, BODY_ACCESS_ALL,
	);
	if gather_status != .Ok
	{
		return gather_status;
	}
	integrator.callbacks.integrate_velocity(
		integrator.callbacks.user_context, gather_indices, position, orientation, inertia,
		transmute(util.I32x8)mask_values, worker_index, util.F32x8(dt), &velocity,
	);
	return bodies_scatter_active_velocities(integrator.bodies, transmute(util.I32x8)scatter_values, velocity);
}

Pose_Integrator_Velocity_Job :: struct
{
	integrator:   ^Pose_Integrator,
	dt:           f32,
	constrained:  Reference_State,
	bundle_count: int,
	worker_count: int,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

pose_integrator_velocity_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Pose_Integrator_Velocity_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	for bundle_index := worker_index; bundle_index < job.bundle_count; bundle_index += job.worker_count
	{
		status = pose_integrator_integrate_velocity_bundle(
			job.integrator, bundle_index * util.PRODUCTION_LANE_COUNT,
			job.dt, job.constrained, worker_index,
		);
		if status != .Ok
		{
			break;
		}
	}
	job.statuses[worker_index] = status;
}

pose_integrator_integrate_velocities :: proc (
	integrator: ^Pose_Integrator, dt: f32, constrained: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready || dt <= 0
	{
		return .Invalid_Argument;
	}
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	bundle_count := (active.count + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
	if bundle_count == 0
	{
		return .Ok;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		worker_count = min(dispatcher.worker_count, bundle_count);
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Capacity_Missing;
	}
	if dispatcher != nil && worker_count > 1
	{
		statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
		job := Pose_Integrator_Velocity_Job{integrator, dt, constrained, bundle_count, worker_count, &statuses};
		dispatch_status := dispatcher.dispatch(dispatcher, pose_integrator_velocity_worker, worker_count, &job);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if statuses[worker_index] != .Ok
			{
				return statuses[worker_index];
			}
		}
		return .Ok;
	}
	for bundle_index in 0 ..< bundle_count
	{
		status := pose_integrator_integrate_velocity_bundle(
			integrator, bundle_index * util.PRODUCTION_LANE_COUNT, dt, constrained, 0,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

Pose_Integrator_Post_Solve_Job :: struct
{
	integrator:   ^Pose_Integrator,
	dt:           f32,
	substep_count: int,
	bundle_count: int,
	worker_count: int,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

pose_integrator_integrate_after_substepping_bundle :: proc "contextless" (
	integrator: ^Pose_Integrator, first_index: int, dt: f32, substep_count, worker_index: int,
) -> Physics_Status
{
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	for substep_index in 0 ..< substep_count
	{
		_ = substep_index;
		status := pose_integrator_integrate_velocity_bundle(integrator, first_index, dt, .Missing, worker_index);
		if status != .Ok
		{
			return status;
		}
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			index := first_index + lane;
			if index >= active.count
			{
				break;
			}
			status = pose_integrator_integrate_pose_body(integrator, index, dt, .Missing);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

pose_integrator_post_solve_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Pose_Integrator_Post_Solve_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	for bundle_index := worker_index; bundle_index < job.bundle_count; bundle_index += job.worker_count
	{
		status = pose_integrator_integrate_after_substepping_bundle(
			job.integrator, bundle_index * util.PRODUCTION_LANE_COUNT, job.dt, job.substep_count, worker_index,
		);
		if status != .Ok
		{
			break;
		}
	}
	job.statuses[worker_index] = status;
}

pose_integrator_integrate_after_substepping :: proc (
	integrator: ^Pose_Integrator, total_dt: f32, substep_count: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready || total_dt <= 0 || substep_count <= 0
	{
		return .Invalid_Argument;
	}
	callback_dt := total_dt;
	callback_substep_count := 1;
	if integrator.callbacks.allow_substeps_for_unconstrained == .Enabled
	{
		callback_dt = total_dt / f32(substep_count);
		callback_substep_count = substep_count;
	}
	status := pose_integrator_prepare(integrator, callback_dt);
	if status != .Ok
	{
		return status;
	}
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	bundle_count := (active.count + util.PRODUCTION_LANE_COUNT - 1) / util.PRODUCTION_LANE_COUNT;
	if bundle_count == 0
	{
		return .Ok;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		worker_count = min(dispatcher.worker_count, bundle_count);
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Capacity_Missing;
	}
	if dispatcher != nil && worker_count > 1
	{
		statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
		job := Pose_Integrator_Post_Solve_Job{
			integrator, callback_dt, callback_substep_count, bundle_count, worker_count, &statuses,
		};
		dispatch_status := dispatcher.dispatch(dispatcher, pose_integrator_post_solve_worker, worker_count, &job);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if statuses[worker_index] != .Ok
			{
				return statuses[worker_index];
			}
		}
		return .Ok;
	}
	for bundle_index in 0 ..< bundle_count
	{
		status = pose_integrator_integrate_after_substepping_bundle(
			integrator, bundle_index * util.PRODUCTION_LANE_COUNT, callback_dt, callback_substep_count, 0,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

pose_integrator_finite_vector3 :: proc "contextless" (value: util.Vector3) -> Reference_State
{
	if math.is_nan(value.x) || math.is_inf(value.x, 0) ||
		math.is_nan(value.y) || math.is_inf(value.y, 0) ||
		math.is_nan(value.z) || math.is_inf(value.z, 0)
	{
		return .Missing;
	}
	return .Present;
}

pose_integrator_integrate_gyroscopic_velocity :: proc "contextless" (
	orientation: util.Quaternion, local_inverse_inertia: util.Symmetric3x3,
	angular_velocity: ^util.Vector3, dt: f32,
)
{
	previous_velocity := angular_velocity^;
	orientation_matrix := util.matrix3x3_from_quaternion(orientation);
	local_velocity := util.matrix3x3_transform_transpose(angular_velocity^, orientation_matrix);
	local_inertia := util.symmetric3x3_invert(local_inverse_inertia);
	local_momentum := util.symmetric3x3_transform(local_velocity, local_inertia);
	residual := util.vector3_scale(util.vector3_cross(local_momentum, local_velocity), dt);
	skew_momentum := util.matrix3x3_create_cross_product(local_momentum);
	skew_velocity := util.matrix3x3_create_cross_product(local_velocity);
	transformed_skew_velocity := util.matrix3x3_multiply_symmetric(skew_velocity, local_inertia);
	change_over_dt := util.matrix3x3_subtract(transformed_skew_velocity, skew_momentum);
	jacobian := util.matrix3x3_add_symmetric(util.matrix3x3_scale(change_over_dt, dt), local_inertia);
	newton_step := util.matrix3x3_transform(residual, util.matrix3x3_invert(jacobian));
	local_velocity = util.vector3_subtract(local_velocity, newton_step);
	angular_velocity^ = util.matrix3x3_transform(local_velocity, orientation_matrix);
	if pose_integrator_finite_vector3(angular_velocity^) == .Missing
	{
		angular_velocity^ = previous_velocity;
	}
}

pose_integrator_integrate_pose_body :: proc "contextless" (
	integrator: ^Pose_Integrator, index: int, dt: f32, constrained: Reference_State,
) -> Physics_Status
{
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if index < 0 || index >= active.count
	{
		return .Not_Found;
	}
	is_constrained := active.constraints.memory[index].count > 0;
	if (constrained == .Present) != is_constrained
	{
		return .Ok;
	}
	dynamics := &active.dynamics_state.memory[index];
	velocity := &dynamics.motion.velocity;
	pose := &dynamics.motion.pose;
	angular_momentum: util.Vector3;
	conserves_momentum := integrator.callbacks.angular_mode == .Conserve_Momentum &&
		body_inertia_mobility(dynamics.inertia.local) == .Dynamic;
	if conserves_momentum
	{
		world_inertia := util.symmetric3x3_invert(dynamics.inertia.world.inverse_inertia_tensor);
		angular_momentum = util.symmetric3x3_transform(velocity.angular, world_inertia);
	}
	pose.position = util.vector3_add(pose.position, util.vector3_scale(velocity.linear, dt));
	pose.orientation = pose_integrator_integrate_orientation(pose.orientation, velocity.angular, dt);
	pose_integrator_update_world_inertia(dynamics);
	if conserves_momentum
	{
		velocity.angular = util.symmetric3x3_transform(angular_momentum, dynamics.inertia.world.inverse_inertia_tensor);
	}
	else if integrator.callbacks.angular_mode == .Conserve_Momentum_With_Gyroscopic_Torque &&
		body_inertia_mobility(dynamics.inertia.local) == .Dynamic
	{
		pose_integrator_integrate_gyroscopic_velocity(
			pose.orientation, dynamics.inertia.local.inverse_inertia_tensor, &velocity.angular, dt,
		);
	}
	return .Ok;
}

Pose_Integrator_Pose_Job :: struct
{
	integrator:   ^Pose_Integrator,
	dt:           f32,
	constrained:  Reference_State,
	body_count:    int,
	worker_count: int,
	statuses:     ^[MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status,
}

pose_integrator_pose_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	context = runtime.default_context();
	job := (^Pose_Integrator_Pose_Job)(dispatcher.unmanaged_context);
	status := Physics_Status.Ok;
	for index := worker_index; index < job.body_count; index += job.worker_count
	{
		status = pose_integrator_integrate_pose_body(job.integrator, index, job.dt, job.constrained);
		if status != .Ok
		{
			break;
		}
	}
	job.statuses[worker_index] = status;
}

pose_integrator_integrate_poses :: proc (
	integrator: ^Pose_Integrator, dt: f32, constrained: Reference_State,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready || dt <= 0
	{
		return .Invalid_Argument;
	}
	active := &integrator.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	if active.count == 0
	{
		return .Ok;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		worker_count = min(dispatcher.worker_count, active.count);
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Capacity_Missing;
	}
	if dispatcher != nil && worker_count > 1
	{
		statuses: [MAXIMUM_SOLVER_WORKER_COUNT]Physics_Status;
		job := Pose_Integrator_Pose_Job{integrator, dt, constrained, active.count, worker_count, &statuses};
		dispatch_status := dispatcher.dispatch(dispatcher, pose_integrator_pose_worker, worker_count, &job);
		if dispatch_status != .Ok
		{
			return .Invalid_Argument;
		}
		for worker_index in 0 ..< worker_count
		{
			if statuses[worker_index] != .Ok
			{
				return statuses[worker_index];
			}
		}
		return .Ok;
	}
	for index in 0 ..< active.count
	{
		status := pose_integrator_integrate_pose_body(integrator, index, dt, constrained);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

pose_integrator_integrate :: proc (integrator: ^Pose_Integrator, dt: f32) -> Physics_Status
{
	status := pose_integrator_prepare(integrator, dt);
	if status != .Ok
	{
		return status;
	}
	status = pose_integrator_integrate_velocities(integrator, dt, .Missing);
	if status != .Ok
	{
		return status;
	}
	status = pose_integrator_integrate_velocities(integrator, dt, .Present);
	if status != .Ok
	{
		return status;
	}
	status = pose_integrator_integrate_poses(integrator, dt, .Missing);
	if status != .Ok
	{
		return status;
	}
	return pose_integrator_integrate_poses(integrator, dt, .Present);
}

pose_integrator_dispose :: proc "contextless" (integrator: ^Pose_Integrator) -> Physics_Status
{
	if integrator == nil || integrator.state != .Ready
	{
		return .Disposed;
	}
	if integrator.callback_activation == .Present
	{
		integrator.callbacks.dispose(integrator.callbacks.user_context);
	}
	integrator^ = {state=.Disposed};
	return .Ok;
}
