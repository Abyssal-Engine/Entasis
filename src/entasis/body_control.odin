package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:simd"

Body_Control_Configuration :: physics.Body_Control_Configuration;
Body_Control_Wake :: physics.Body_Control_Wake;
Body_Input_Mode :: physics.Body_Input_Mode;
Body_Damping_Mode :: physics.Body_Damping_Mode;
Body_Damping :: physics.Body_Damping;
body_control_configuration_default :: physics.body_control_configuration_default;
// enables sparse optional force/damping/kinematic-target storage. configuration
// is copied, callbacks are borrowed from the existing world, and empty controls
// keep the original native integration callback selected
world_enable_body_control :: proc (
	world: ^World, configuration: Body_Control_Configuration = {input_capacity=16, settings_capacity=16},
) -> Status
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	callbacks: Pose_Callbacks = data.simulation.integrator.callbacks;
	undamped: physics.Pose_Integrator_Velocity_Proc;
	if callbacks.integrate_velocity == physics.pose_integrator_default_velocity
	{
		undamped = body_control_uniform_undamped;
	}
	else if callbacks.integrate_velocity == planetary_gravity_velocity
	{
		undamped = body_control_planetary_undamped;
	}
	else if callbacks.integrate_velocity == per_body_gravity_velocity
	{
		undamped = body_control_per_body_undamped;
	}
	return physics.body_control_initialize(&data.simulation, configuration, data.allocator, data.allocation_scope, undamped);
}

world_disable_body_control :: proc (world: ^World) -> Status
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	world_invalidate_views_data(data);
	return physics.body_control_destroy(&data.simulation);
}

@(private)
body_control_owner :: proc "contextless" (world: ^World) -> (^physics.Body_Control_Storage, Status)
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	if data.simulation.body_control == nil
	{
		return nil, .Not_Found;
	}
	return data.simulation.body_control, .Ok;
}

// explicit cold reservation. equal/smaller capacities reuse existing buffers
body_control_reserve :: proc (world: ^World, input_capacity, settings_capacity: i32) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	return physics.body_control_reserve_storage(owner, input_capacity, settings_capacity);
}

// accumulates a world-space force or mass-independent acceleration for one
// integrated step. prediction does not consume it. failed executed work does.
// Preserve_Sleep leaves the input pending without integrating sleeping time
body_add_force :: proc (
	world: ^World, body: Body_Handle, force: Vector3,
	mode: Body_Input_Mode = .Force, wake: Body_Control_Wake = .Wake,
) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_control_input_add(owner, body, force, {}, mode, wake);
}

body_add_torque :: proc (
	world: ^World, body: Body_Handle, torque: Vector3,
	mode: Body_Input_Mode = .Force, wake: Body_Control_Wake = .Wake,
) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_control_input_add(owner, body, {}, torque, mode, wake);
}

// freezes F and (position - center_of_mass) x F at submission. Acceleration
// mode is deliberately not exposed for a world-point wrench
body_add_force_at_position :: proc (
	world: ^World, body: Body_Handle, force, position: Vector3,
	wake: Body_Control_Wake = .Wake,
) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	if physics.constraint_vector3_finite(position) == .Missing
	{
		return .Invalid_Argument;
	}
	dynamics: ^physics.Body_Dynamics;
	dynamics, _, status = physics.body_control_resolve(owner, body);
	if status != .Ok
	{
		return status;
	}
	return physics.body_control_input_add(owner, body, force,
		util.vector3_cross(util.vector3_subtract(position, dynamics.motion.pose.position), force), .Force, wake);
}

body_clear_inputs :: proc (world: ^World, body: Body_Handle) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	return physics.body_control_input_clear(owner, body);
}

// rates are nonnegative exponential attenuation coefficients in inverse seconds.
// Additional composes after the existing callback. Override replaces damping
// only for recognized built-in gravity callbacks. opaque callbacks reject it
body_set_damping :: proc (world: ^World, body: Body_Handle, damping: Body_Damping) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	return physics.body_control_damping_set(owner, body, damping);
}

body_get_damping :: proc (world: ^World, body: Body_Handle) -> (Body_Damping, Status)
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return {}, status;
	}
	_, _, status = physics.body_control_resolve(owner, body);
	if status != .Ok
	{
		return {}, status;
	}
	key: i32 = body.value;
	index: int = util.quick_dictionary_index_of(&owner.damping, &key);
	if index < 0
	{
		return {}, .Ok;
	}
	return owner.damping.values.memory[index].settings, .Ok;
}

// queues a next-step pose without teleporting. target velocity is exposed during
// collision and solving, then cleared before the next untargeted step. a complete
// explicit body edit supersedes queued or expired target state
body_set_kinematic_target :: proc (
	world: ^World, body: Body_Handle, target: Rigid_Pose, wake: Body_Control_Wake = .Wake,
) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_control_target_set(owner, body, target, wake);
}

body_get_kinematic_target :: proc (world: ^World, body: Body_Handle) -> (Rigid_Pose, Status)
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return {}, status;
	}
	_, _, status = physics.body_control_resolve(owner, body);
	if status != .Ok
	{
		return {}, status;
	}
	key: i32 = body.value;
	index: int = util.quick_dictionary_index_of(&owner.targets, &key);
	if index < 0 || owner.targets.values.memory[index].phase == .Expired
	{
		return {}, .Not_Found;
	}
	return owner.targets.values.memory[index].pose, .Ok;
}

body_clear_kinematic_target :: proc (world: ^World, body: Body_Handle) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_control_target_clear(owner, body);
}

@(private)
body_control_uniform_undamped :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _, _, _ = body_indices, position, orientation, inertia, integration_mask, worker_index, dt;
	policy: ^Uniform_Gravity_Policy = (^Uniform_Gravity_Policy)(user_context);
	velocity.linear = util.vector3_wide_add(velocity.linear, policy.gravity_wide_dt);
}

@(private)
body_control_planetary_undamped :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _, _ = body_indices, orientation, inertia, integration_mask, worker_index, dt;
	policy: ^Planetary_Gravity_Policy = (^Planetary_Gravity_Policy)(user_context);
	offset: util.Vector3_Wide = util.vector3_wide_subtract(position, util.vector3_wide_broadcast(policy.center));
	distance_squared: util.F32x8 = util.vector3_wide_length_squared(offset);
	denominator: util.F32x8 = simd.max(util.F32x8(1), simd.mul(distance_squared, simd.sqrt(distance_squared)));
	velocity.linear = util.vector3_wide_add(velocity.linear,
		util.vector3_wide_scale(offset, simd.div(util.F32x8(-policy.gravity_dt), denominator)));
}

@(private)
body_control_per_body_undamped :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_, _, _, _, _ = position, orientation, inertia, integration_mask, worker_index;
	policy: ^Per_Body_Gravity_Policy = (^Per_Body_Gravity_Policy)(user_context);
	gravity: util.Vector3_Wide;
	indices: [8]i32 = transmute([8]i32)body_indices;
	active: ^physics.Body_Set = &policy.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for lane in 0 ..< 8
	{
		index: int = int(indices[lane]);
		if index < 0 || index >= int(active.count)
		{
			continue;
		}
		value: ^Vector3;
		status: Status;
		value, status = body_property_get(policy.gravities, active.index_to_handle.memory[index]);
		if status == .Ok
		{
			util.vector3_wide_write_slot(&gravity, lane, value^);
		}
	}
	velocity.linear = util.vector3_wide_add(velocity.linear, util.vector3_wide_scale(gravity, dt));
}

Body_Lock_Axis :: physics.Body_Lock_Axis;
Body_Lock_Axes :: physics.Body_Lock_Axes;
Body_Axis_Lock :: physics.Body_Axis_Lock;
body_axis_lock_default :: physics.body_axis_lock_default;
// configures one physical one-body constraint, with world-linear and
// reference-frame angular rows. the caller supplies the reference pose explicitly.
// configuration wakes sleeping bodies. zero axes remove the owned lock
body_set_axis_lock :: proc (world: ^World, body: Body_Handle, description: Body_Axis_Lock) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_axis_lock_set(owner, body, description);
}

// copies settings from active or sleeping constraint storage without awakening.
// returns Not_Found when the body has no lock. no allocation or borrowed pointer
body_get_axis_lock :: proc (world: ^World, body: Body_Handle) -> (Body_Axis_Lock, Status)
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.body_axis_lock_get(owner, body);
}

// removes only the owned lock, not application joints. a sleeping island may
// need awakening. failure leaves an unremoved lock available for retry
body_clear_axis_lock :: proc (world: ^World, body: Body_Handle) -> Status
{
	owner: ^physics.Body_Control_Storage;
	status: Status;
	owner, status = body_control_owner(world);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(world_data_get(world));
	return physics.body_axis_lock_clear(owner, body);
}
