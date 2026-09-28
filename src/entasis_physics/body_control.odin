package entasis_physics

import "base:runtime"
import "core:math"
import "core:mem"
import "core:simd"
import util "entasis:entasis_utilities"

Body_Control_Wake :: enum u8
{
	Wake,
	Preserve_Sleep,
}

Body_Input_Mode :: enum u8
{
	Force,
	Acceleration,
}

Body_Damping_Mode :: enum u8
{
	Additional,
	Override,
}

Body_Damping :: struct
{
	linear, angular: f32,
	mode: Body_Damping_Mode,
}

Body_Control_Configuration :: struct
{
	input_capacity, settings_capacity: i32,
}

Body_Control_Phase :: enum u8
{
	Idle,
	Preview,
	Integrating,
}

Body_Target_Phase :: enum u8
{
	Pending,
	Prepared,
	Expired,
}

// dense columns are separate for inputs, damping and targets. inactive bodies
// retain handle-keyed inputs, while workers read a prepared active-index remap
Body_Control_Input :: struct
{
	force, acceleration, torque, angular_acceleration: util.Vector3,
	applied: Reference_State,
}

// multipliers are derived once at the existing prepare callback, not in each
// body/SIMD integration invocation. public rates and layouts stay unchanged
Body_Control_Damping_Record :: struct
{
	settings: Body_Damping,
	linear_multiplier, angular_multiplier: f32,
}

Body_Control_Target :: struct
{
	pose: Rigid_Pose,
	velocity, previous_velocity: Body_Velocity,
	phase: Body_Target_Phase,
	applied: Reference_State,
}

Body_Control_Binding :: struct
{
	input, damping, target: i32,
}

Body_Control_Storage :: struct
{
	simulation: ^Simulation,
	allocator: mem.Allocator,
	scope: util.Allocation_Scope,
	configuration: Body_Control_Configuration,
	callbacks: Pose_Integrator_Callbacks,
	undamped: Pose_Integrator_Velocity_Proc,
	inputs: util.Quick_Dictionary(i32, Body_Control_Input),
	damping: util.Quick_Dictionary(i32, Body_Control_Damping_Record),
	targets: util.Quick_Dictionary(i32, Body_Control_Target),
	bindings: util.Buffer(Body_Control_Binding),
	activation: Reference_State,
	phase: Body_Control_Phase,
	step_dt: f32,
}

body_control_configuration_default :: proc "contextless" () -> Body_Control_Configuration
{
	return {input_capacity=16, settings_capacity=16};
}

body_control_configuration_validate :: proc "contextless" (value: Body_Control_Configuration) -> Physics_Status
{
	// the key/hash arrays and each value buffer must fit the existing pool spans
	if value.input_capacity <= 0 || value.settings_capacity <= 0 ||
	value.input_capacity > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(Body_Control_Target) ||
	value.settings_capacity > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(Body_Control_Damping_Record)
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

body_control_release :: proc (storage: ^Body_Control_Storage)
{
	pool: ^util.Buffer_Pool = storage.simulation.pool;
	if storage.inputs.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.inputs, pool);
	}
	if storage.damping.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.damping, pool);
	}
	if storage.targets.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.targets, pool);
	}
	if storage.bindings.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.bindings);
	}
	allocator: mem.Allocator = storage.allocator;
	scope: util.Allocation_Scope = storage.scope;
	_ = util.allocation_free(storage, size_of(Body_Control_Storage), align_of(Body_Control_Storage), allocator, scope);
}

body_control_initialize :: proc (
	simulation: ^Simulation, configuration: Body_Control_Configuration,
	allocator: mem.Allocator, scope: util.Allocation_Scope,
	undamped: Pose_Integrator_Velocity_Proc = nil,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready || simulation.body_control != nil ||
	(scope != .Legacy && scope != .All_Owned)
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = body_control_configuration_validate(configuration);
	if status != .Ok
	{
		return status;
	}
	actual: mem.Allocator = allocator;
	if actual.procedure == nil
	{
		actual = runtime.heap_allocator();
	}
	// freeze the boundary before invoking a caller allocator. no handle is published
	// until every initial buffer has been acquired successfully
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(Body_Control_Storage), align_of(Body_Control_Storage), actual);
	if error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	storage: ^Body_Control_Storage = (^Body_Control_Storage)(memory);
	storage^ = {simulation=simulation, allocator=actual, scope=scope,
		configuration=configuration, callbacks=simulation.integrator.callbacks, undamped=undamped};
	result: util.Collection_Status = util.quick_dictionary_initialize(
		&storage.inputs, int(configuration.input_capacity), simulation.pool, util.primitive_i32_hash_equal(), 1);
	if result == .Ok
	{
		result = util.quick_dictionary_initialize(&storage.targets, int(configuration.input_capacity),
			simulation.pool, util.primitive_i32_hash_equal(), 1);
	}
	if result == .Ok
	{
		result = util.quick_dictionary_initialize(&storage.damping, int(configuration.settings_capacity),
			simulation.pool, util.primitive_i32_hash_equal(), 1);
	}
	if result != .Ok
	{
		body_control_release(storage);
		return physics_collection_status(result);
	}
	status = physics_ensure_buffer_capacity(simulation.pool, &storage.bindings,
		int(simulation.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.length), 0);
	if status != .Ok
	{
		body_control_release(storage);
		return status;
	}
	simulation.body_control = storage;
	simulation.bodies.body_control = storage;
	return .Ok;
}

body_control_select_callbacks :: proc "contextless" (storage: ^Body_Control_Storage)
{
	selection, targets: Reference_State = .Missing, .Missing;
	locations: util.Buffer(Body_Memory_Location) = storage.simulation.bodies.handle_to_location;
	for index in 0 ..< storage.inputs.count
	{
		if locations.memory[storage.inputs.keys.memory[index]].set_index == BODIES_ACTIVE_SET_INDEX
		{
			selection = .Present;
			break;
		}
	}
	if selection == .Missing
	{
		for index in 0 ..< storage.damping.count
		{
			if locations.memory[storage.damping.keys.memory[index]].set_index == BODIES_ACTIVE_SET_INDEX
			{
				selection = .Present;
				break;
			}
		}
	}
	for index in 0 ..< storage.targets.count
	{
		if storage.targets.values.memory[index].phase != .Expired &&
		locations.memory[storage.targets.keys.memory[index]].set_index == BODIES_ACTIVE_SET_INDEX
		{
			selection = .Present;
			targets = .Present;
			break;
		}
	}
	if selection == .Missing
	{
		if storage.activation == .Present
		{
			storage.simulation.integrator.callbacks = storage.callbacks;
		}
	}
	else
	{
		callbacks: Pose_Integrator_Callbacks = storage.callbacks;
		callbacks.prepare_for_integration = body_control_prepare_callback;
		callbacks.integrate_velocity = body_control_velocity_callback;
		callbacks.user_context = storage;
		// sleeping-only controls do not wrap unrelated active bodies. re-selection
		// after collision awakening occurs at the solve preparation boundary
		if targets == .Present
		{
			callbacks.integrate_kinematic_velocity = .Enabled;
		}
		storage.simulation.integrator.callbacks = callbacks;
	}
	storage.activation = selection;
}

body_control_destroy :: proc (simulation: ^Simulation, detach_locks: Reference_State = .Present) -> Physics_Status
{
	if simulation == nil || simulation.body_control == nil
	{
		return .Not_Found;
	}
	if simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	storage: ^Body_Control_Storage = simulation.body_control;
	if detach_locks == .Present
	{
		simulation.state = .Stepping;
		status: Physics_Status = body_axis_lock_remove_all(simulation);
		simulation.state = .Ready;
		if status != .Ok
		{
			return status;
		}
	}
	for index in 0 ..< storage.targets.count
	{
		value: ^Body_Control_Target = &storage.targets.values.memory[index];
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(&simulation.bodies, {storage.targets.keys.memory[index]});
		if status == .Ok && value.phase != .Pending
		{
			dynamics: ^Body_Dynamics = &simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
			if dynamics.motion.velocity == value.velocity
			{
				dynamics.motion.velocity = value.previous_velocity if value.phase == .Prepared else Body_Velocity{};
			}
		}
	}
	simulation.integrator.callbacks = storage.callbacks;
	simulation.body_control = nil;
	simulation.bodies.body_control = nil;
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	body_control_release(storage);
	return .Ok;
}

body_control_reserve_storage :: proc (
	storage: ^Body_Control_Storage, input_capacity, settings_capacity: i32,
) -> Physics_Status
{
	if storage == nil || storage.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = body_control_configuration_validate({input_capacity, settings_capacity});
	if status != .Ok
	{
		return status;
	}
	simulation: ^Simulation = storage.simulation;
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	result: util.Collection_Status = util.quick_dictionary_ensure_capacity(&storage.inputs, int(input_capacity), simulation.pool);
	if result == .Ok
	{
		result = util.quick_dictionary_ensure_capacity(&storage.targets, int(input_capacity), simulation.pool);
	}
	if result == .Ok
	{
		result = util.quick_dictionary_ensure_capacity(&storage.damping, int(settings_capacity), simulation.pool);
	}
	if result != .Ok
	{
		return physics_collection_status(result);
	}
	status = physics_ensure_buffer_capacity(simulation.pool, &storage.bindings,
		int(simulation.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.length), 0);
	if status != .Ok
	{
		return status;
	}
	storage.configuration.input_capacity = max(input_capacity, storage.configuration.input_capacity);
	storage.configuration.settings_capacity = max(settings_capacity, storage.configuration.settings_capacity);
	return .Ok;
}

body_control_resolve :: proc "contextless" (
	storage: ^Body_Control_Storage, handle: Body_Handle,
) -> (^Body_Dynamics, Body_Memory_Location, Physics_Status)
{
	if storage == nil || storage.simulation.state != .Ready
	{
		return nil, {}, .Invalid_Argument;
	}
	location: Body_Memory_Location;
	status: Physics_Status;
	location, status = bodies_resolve(&storage.simulation.bodies, handle);
	if status != .Ok
	{
		return nil, {}, status;
	}
	return &storage.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index], location, .Ok;
}

body_control_input_add :: proc (
	storage: ^Body_Control_Storage, handle: Body_Handle,
	linear, angular: util.Vector3, mode: Body_Input_Mode, wake: Body_Control_Wake,
) -> Physics_Status
{
	dynamics: ^Body_Dynamics;
	location: Body_Memory_Location;
	status: Physics_Status;
	dynamics, location, status = body_control_resolve(storage, handle);
	if status != .Ok
	{
		return status;
	}
	if body_inertia_mobility(dynamics.inertia.local) != .Dynamic ||
	(mode != .Force && mode != .Acceleration) || (wake != .Wake && wake != .Preserve_Sleep) ||
	constraint_vector3_finite(linear) == .Missing || constraint_vector3_finite(angular) == .Missing
	{
		return .Invalid_Argument;
	}
	if linear == (util.Vector3{}) && angular == (util.Vector3{})
	{
		return .Ok;
	}
	key: i32 = handle.value;
	index: int = util.quick_dictionary_index_of(&storage.inputs, &key);
	value: Body_Control_Input;
	if index >= 0
	{
		value = storage.inputs.values.memory[index];
	}
	else if storage.inputs.count == int(storage.inputs.keys.length)
	{
		return .Capacity_Missing;
	}
	if mode == .Force
	{
		value.force = util.vector3_add(value.force, linear);
		value.torque = util.vector3_add(value.torque, angular);
	}
	else
	{
		value.acceleration = util.vector3_add(value.acceleration, linear);
		value.angular_acceleration = util.vector3_add(value.angular_acceleration, angular);
	}
	if constraint_vector3_finite(value.force) == .Missing || constraint_vector3_finite(value.torque) == .Missing ||
	constraint_vector3_finite(value.acceleration) == .Missing || constraint_vector3_finite(value.angular_acceleration) == .Missing
	{
		return .Invalid_Argument;
	}
	if wake == .Wake
	{
		storage.simulation.state = .Stepping;
		status = island_awakener_awaken_body(&storage.simulation.awakener, handle);
		storage.simulation.state = .Ready;
		if status != .Ok
		{
			return status;
		}
		location, _ = bodies_resolve(&storage.simulation.bodies, handle);
		storage.simulation.bodies.sets.memory[location.set_index].activity.memory[location.index].timesteps_under_threshold_count = 0;
		storage.simulation.bodies.sets.memory[location.set_index].activity.memory[location.index].sleep_candidate = .Not_Candidate;
	}
	if index >= 0
	{
		storage.inputs.values.memory[index] = value;
	}
	else
	{
		_ = util.quick_dictionary_add_unsafely(&storage.inputs, key, value);
	}
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_input_clear :: proc (storage: ^Body_Control_Storage, handle: Body_Handle) -> Physics_Status
{
	status: Physics_Status;
	_, _, status = body_control_resolve(storage, handle);
	if status != .Ok
	{
		return status;
	}
	key: i32 = handle.value;
	_ = util.quick_dictionary_fast_remove(&storage.inputs, &key);
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_damping_set :: proc (
	storage: ^Body_Control_Storage, handle: Body_Handle, value: Body_Damping,
) -> Physics_Status
{
	dynamics: ^Body_Dynamics;
	status: Physics_Status;
	dynamics, _, status = body_control_resolve(storage, handle);
	if status != .Ok
	{
		return status;
	}
	if body_inertia_mobility(dynamics.inertia.local) != .Dynamic ||
	!(value.linear >= 0) || !(value.angular >= 0) || math.is_inf(value.linear, 0) || math.is_inf(value.angular, 0) ||
	(value.mode != .Additional && value.mode != .Override) || (value.mode == .Override && storage.undamped == nil)
	{
		return .Invalid_Argument;
	}
	key: i32 = handle.value;
	if value.mode == .Additional && value.linear == 0 && value.angular == 0
	{
		_ = util.quick_dictionary_fast_remove(&storage.damping, &key);
	}
	else
	{
		index: int = util.quick_dictionary_index_of(&storage.damping, &key);
		if index >= 0
		{
			storage.damping.values.memory[index] = {settings=value};
		}
		else
		{
			result: util.Collection_Status = util.quick_dictionary_add_unsafely(&storage.damping, key, Body_Control_Damping_Record{settings=value});
			if result != .Ok
			{
				return physics_collection_status(result);
			}
		}
	}
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_target_set :: proc (
	storage: ^Body_Control_Storage, handle: Body_Handle, pose: Rigid_Pose, wake: Body_Control_Wake,
) -> Physics_Status
{
	dynamics: ^Body_Dynamics;
	location: Body_Memory_Location;
	status: Physics_Status;
	dynamics, location, status = body_control_resolve(storage, handle);
	if status != .Ok
	{
		return status;
	}
	if body_inertia_mobility(dynamics.inertia.local) != .Kinematic ||
	constraint_vector3_finite(pose.position) == .Missing || constraint_unit_quaternion(pose.orientation) == .Missing ||
	(wake != .Wake && wake != .Preserve_Sleep)
	{
		return .Invalid_Argument;
	}
	key: i32 = handle.value;
	index: int = util.quick_dictionary_index_of(&storage.targets, &key);
	if index < 0 && storage.targets.count == int(storage.targets.keys.length)
	{
		return .Capacity_Missing;
	}
	if wake == .Wake
	{
		storage.simulation.state = .Stepping;
		status = island_awakener_awaken_body(&storage.simulation.awakener, handle);
		storage.simulation.state = .Ready;
		if status != .Ok
		{
			return status;
		}
		location, _ = bodies_resolve(&storage.simulation.bodies, handle);
		dynamics = &storage.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
		storage.simulation.bodies.sets.memory[location.set_index].activity.memory[location.index].timesteps_under_threshold_count = 0;
		storage.simulation.bodies.sets.memory[location.set_index].activity.memory[location.index].sleep_candidate = .Not_Candidate;
	}
	value: Body_Control_Target = {pose=pose};
	if index >= 0
	{
		previous: ^Body_Control_Target = &storage.targets.values.memory[index];
		if dynamics.motion.velocity == previous.velocity
		{
			if previous.phase == .Expired
			{
				dynamics.motion.velocity = {};
			}
			else if previous.phase == .Prepared
			{
				dynamics.motion.velocity = previous.previous_velocity;
			}
		}
		storage.targets.values.memory[index] = value;
	}
	else
	{
		_ = util.quick_dictionary_add_unsafely(&storage.targets, key, value);
	}
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_target_clear :: proc (storage: ^Body_Control_Storage, handle: Body_Handle) -> Physics_Status
{
	dynamics: ^Body_Dynamics;
	status: Physics_Status;
	dynamics, _, status = body_control_resolve(storage, handle);
	if status != .Ok
	{
		return status;
	}
	key: i32 = handle.value;
	index: int = util.quick_dictionary_index_of(&storage.targets, &key);
	if index >= 0
	{
		value: ^Body_Control_Target = &storage.targets.values.memory[index];
		if dynamics.motion.velocity == value.velocity
		{
			if value.phase == .Expired
			{
				dynamics.motion.velocity = {};
			}
			else if value.phase == .Prepared
			{
				dynamics.motion.velocity = value.previous_velocity;
			}
		}
		_ = util.quick_dictionary_fast_remove(&storage.targets, &key);
	}
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_retire :: proc (storage: ^Body_Control_Storage, handle: Body_Handle)
{
	key: i32 = handle.value;
	_ = util.quick_dictionary_fast_remove(&storage.inputs, &key);
	_ = util.quick_dictionary_fast_remove(&storage.damping, &key);
	_ = util.quick_dictionary_fast_remove(&storage.targets, &key);
	body_control_select_callbacks(storage);
}

body_control_body_applied :: proc (
	storage: ^Body_Control_Storage, handle: Body_Handle, previous, current: Body_Mobility,
)
{
	key: i32 = handle.value;
	// a complete explicit body edit supersedes a queued/expired target velocity
	_ = util.quick_dictionary_fast_remove(&storage.targets, &key);
	if previous != current
	{
		_ = util.quick_dictionary_fast_remove(&storage.inputs, &key);
		_ = util.quick_dictionary_fast_remove(&storage.damping, &key);
	}
	body_control_select_callbacks(storage);
}

body_control_clear_storage :: proc (storage: ^Body_Control_Storage)
{
	util.quick_dictionary_clear(&storage.inputs);
	util.quick_dictionary_clear(&storage.damping);
	util.quick_dictionary_clear(&storage.targets);
	storage.phase = .Idle;
	body_control_select_callbacks(storage);
}

body_control_target_velocity :: proc "contextless" (start, target: Rigid_Pose, dt: f32) -> Body_Velocity
{
	delta: util.Quaternion = util.quaternion_concatenate(util.quaternion_conjugate(start.orientation), target.orientation);
	if delta.w < 0
	{
		delta = {-delta.x, -delta.y, -delta.z, -delta.w};
	}
	axis: util.Vector3 = {delta.x, delta.y, delta.z};
	length: f32 = util.vector3_length(axis);
	angular: util.Vector3;
	if length > 0
	{
		angle: f32 = 2 * f32(math.atan2(f64(length), f64(delta.w)));
		angular = util.vector3_scale(axis, angle / (length * dt));
	}
	return {linear=util.vector3_scale(util.vector3_subtract(target.position, start.position), 1 / dt), angular=angular};
}

body_control_step_prepare :: #force_no_inline proc (storage: ^Body_Control_Storage, dt: f32) -> Physics_Status
{
	if !(dt > 0) || math.is_inf(dt, 0)
	{
		return .Invalid_Argument;
	}
	for target_index in 0 ..< storage.targets.count
	{
		value: ^Body_Control_Target = &storage.targets.values.memory[target_index];
		if value.phase == .Expired
		{
			continue;
		}
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(&storage.simulation.bodies, {storage.targets.keys.memory[target_index]});
		if status != .Ok
		{
			return status;
		}
		if location.set_index == BODIES_ACTIVE_SET_INDEX
		{
			dynamics: ^Body_Dynamics = &storage.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
			velocity: Body_Velocity = body_control_target_velocity(dynamics.motion.pose, value.pose, dt);
			if constraint_vector3_finite(velocity.linear) == .Missing || constraint_vector3_finite(velocity.angular) == .Missing
			{
				return .Invalid_Argument;
			}
		}
	}
	storage.step_dt = dt;
	storage.phase = .Preview;
	index: int = 0;
	for index < storage.targets.count
	{
		key: i32 = storage.targets.keys.memory[index];
		value: ^Body_Control_Target = &storage.targets.values.memory[index];
		location: Body_Memory_Location;
		status: Physics_Status;
		location, status = bodies_resolve(&storage.simulation.bodies, {key});
		if status != .Ok
		{
			return status;
		}
		dynamics: ^Body_Dynamics = &storage.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
		if value.phase == .Expired
		{
			if dynamics.motion.velocity == value.velocity
			{
				dynamics.motion.velocity = {};
			}
			_ = util.quick_dictionary_fast_remove(&storage.targets, &key);
			continue;
		}
		if location.set_index == BODIES_ACTIVE_SET_INDEX
		{
			value.velocity = body_control_target_velocity(dynamics.motion.pose, value.pose, dt);
			if value.phase != .Prepared
			{
				value.previous_velocity = dynamics.motion.velocity;
			}
			value.phase = .Prepared;
			value.applied = .Missing;
			dynamics.motion.velocity = value.velocity;
		}
		index += 1;
	}
	body_control_select_callbacks(storage);
	return .Ok;
}

body_control_prepare_callback :: proc "contextless" (user_context: rawptr, dt: f32) -> Physics_Status
{
	context = runtime.default_context();
	storage: ^Body_Control_Storage = (^Body_Control_Storage)(user_context);
	body_control_select_callbacks(storage);
	if storage.activation == .Missing
	{
		return storage.callbacks.prepare_for_integration(storage.callbacks.user_context, dt);
	}
	active: ^Body_Set = &storage.simulation.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
	status: Physics_Status = physics_ensure_buffer_capacity(storage.simulation.pool, &storage.bindings, max(1, int(active.count)), 0);
	if status != .Ok
	{
		return status;
	}
	for index in 0 ..< int(active.count)
	{
		storage.bindings.memory[index] = {-1, -1, -1};
	}
	// rebind only opted-in handles. collision awakening and solver compaction can
	// change active indices between prediction and actual integration
	for index in 0 ..< storage.inputs.count
	{
		location: Body_Memory_Location = storage.simulation.bodies.handle_to_location.memory[storage.inputs.keys.memory[index]];
		if location.set_index == BODIES_ACTIVE_SET_INDEX
		{
			storage.bindings.memory[location.index].input = i32(index);
		}
	}
	for index in 0 ..< storage.damping.count
	{
		location: Body_Memory_Location = storage.simulation.bodies.handle_to_location.memory[storage.damping.keys.memory[index]];
		if location.set_index == BODIES_ACTIVE_SET_INDEX
		{
			storage.bindings.memory[location.index].damping = i32(index);
			value: ^Body_Control_Damping_Record = &storage.damping.values.memory[index];
			value.linear_multiplier = f32(math.exp(-f64(value.settings.linear) * f64(dt)));
			value.angular_multiplier = f32(math.exp(-f64(value.settings.angular) * f64(dt)));
		}
	}
	for index in 0 ..< storage.targets.count
	{
		location: Body_Memory_Location = storage.simulation.bodies.handle_to_location.memory[storage.targets.keys.memory[index]];
		if location.set_index == BODIES_ACTIVE_SET_INDEX
		{
			value: ^Body_Control_Target = &storage.targets.values.memory[index];
			if value.phase == .Pending
			{
				dynamics: ^Body_Dynamics = &active.dynamics_state.memory[location.index];
				value.velocity = body_control_target_velocity(dynamics.motion.pose, value.pose, storage.step_dt);
				if constraint_vector3_finite(value.velocity.linear) == .Missing || constraint_vector3_finite(value.velocity.angular) == .Missing
				{
					return .Invalid_Argument;
				}
				value.previous_velocity = dynamics.motion.velocity;
				value.phase = .Prepared;
				dynamics.motion.velocity = value.velocity;
			}
			if value.phase == .Prepared
			{
				storage.bindings.memory[location.index].target = i32(index);
			}
		}
	}
	return storage.callbacks.prepare_for_integration(storage.callbacks.user_context, dt);
}

body_control_velocity_callback :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^Body_Velocity_Wide,
)
{
	storage: ^Body_Control_Storage = (^Body_Control_Storage)(user_context);
	indices: [8]i32 = transmute([8]i32)body_indices;
	masks: [8]i32 = transmute([8]i32)integration_mask;
	base_indices: [8]i32 = indices;
	base_masks: [8]i32 = masks;
	override_masks: [8]i32;
	base_count, override_count: int;
	for lane in 0 ..< 8
	{
		if masks[lane] == 0
		{
			continue;
		}
		binding: Body_Control_Binding = storage.bindings.memory[indices[lane]];
		if binding.target >= 0 || (storage.callbacks.integrate_kinematic_velocity == .Disabled &&
			body_inertia_mobility(storage.simulation.bodies.sets.memory[BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[indices[lane]].inertia.local) == .Kinematic)
		{
			base_masks[lane] = 0;
			base_indices[lane] = -1;
			continue;
		}
		base_count += 1;
		if binding.damping >= 0 && storage.damping.values.memory[binding.damping].settings.mode == .Override
		{
			override_masks[lane] = -1;
			override_count += 1;
		}
	}
	prior: Body_Velocity_Wide = velocity^;
	if base_count > 0
	{
		storage.callbacks.integrate_velocity(storage.callbacks.user_context,
			transmute(util.I32x8)base_indices, position, orientation, inertia,
			transmute(util.I32x8)base_masks, worker_index, dt, velocity);
		velocity.linear = util.vector3_wide_select(transmute(util.I32x8)base_masks, velocity.linear, prior.linear);
		velocity.angular = util.vector3_wide_select(transmute(util.I32x8)base_masks, velocity.angular, prior.angular);
	}
	if override_count > 0
	{
		undamped: Body_Velocity_Wide = prior;
		storage.undamped(storage.callbacks.user_context, body_indices, position, orientation, inertia,
			transmute(util.I32x8)override_masks, worker_index, dt, &undamped);
		velocity.linear = util.vector3_wide_select(transmute(util.I32x8)override_masks, undamped.linear, velocity.linear);
		velocity.angular = util.vector3_wide_select(transmute(util.I32x8)override_masks, undamped.angular, velocity.angular);
	}
	for lane in 0 ..< 8
	{
		if masks[lane] == 0
		{
			continue;
		}
		binding: Body_Control_Binding = storage.bindings.memory[indices[lane]];
		if binding.target >= 0
		{
			target: ^Body_Control_Target = &storage.targets.values.memory[binding.target];
			util.vector3_wide_write_slot(&velocity.linear, lane, target.velocity.linear);
			util.vector3_wide_write_slot(&velocity.angular, lane, target.velocity.angular);
			if storage.phase == .Integrating
			{
				target.applied = .Present;
			}
			continue;
		}
		if binding.input < 0 && binding.damping < 0
		{
			continue;
		}
		h: f32 = simd.extract(dt, lane);
		linear: util.Vector3 = util.vector3_wide_read_slot(velocity.linear, lane);
		angular: util.Vector3 = util.vector3_wide_read_slot(velocity.angular, lane);
		if binding.input >= 0
		{
			input: ^Body_Control_Input = &storage.inputs.values.memory[binding.input];
			linear = util.vector3_add(linear, util.vector3_scale(util.vector3_add(
						util.vector3_scale(input.force, simd.extract(inertia.inverse_mass, lane)), input.acceleration), h));
			angular_delta: util.Vector3 = input.angular_acceleration;
			if input.torque != (util.Vector3{})
			{
				local_inertia: util.Symmetric3x3 = util.Symmetric3x3{
					xx=simd.extract(inertia.inverse_inertia_tensor.xx, lane), yx=simd.extract(inertia.inverse_inertia_tensor.yx, lane),
					yy=simd.extract(inertia.inverse_inertia_tensor.yy, lane), zx=simd.extract(inertia.inverse_inertia_tensor.zx, lane),
					zy=simd.extract(inertia.inverse_inertia_tensor.zy, lane), zz=simd.extract(inertia.inverse_inertia_tensor.zz, lane)};
				rotation: util.Matrix3x3 = util.matrix3x3_from_quaternion(util.quaternion_wide_read_slot(orientation, lane));
				world_inertia: util.Symmetric3x3 = util.symmetric3x3_rotation_sandwich(rotation, local_inertia);
				angular_delta = util.vector3_add(angular_delta, util.symmetric3x3_transform(input.torque, world_inertia));
			}
			angular = util.vector3_add(angular, util.vector3_scale(angular_delta, h));
			// existing integration responsibilities assign each body to one writer.
			// prediction never consumes. every actual substep uses the same input
			if storage.phase == .Integrating
			{
				input.applied = .Present;
			}
		}
		if binding.damping >= 0
		{
			value: ^Body_Control_Damping_Record = &storage.damping.values.memory[binding.damping];
			linear = util.vector3_scale(linear, value.linear_multiplier);
			angular = util.vector3_scale(angular, value.angular_multiplier);
		}
		util.vector3_wide_write_slot(&velocity.linear, lane, linear);
		util.vector3_wide_write_slot(&velocity.angular, lane, angular);
	}
}

body_control_solve_begin :: #force_no_inline proc (storage: ^Body_Control_Storage, dt: f32) -> Physics_Status
{
	if storage.phase == .Idle
	{
		status: Physics_Status = body_control_step_prepare(storage, dt);
		if status != .Ok
		{
			return status;
		}
	}
	body_control_select_callbacks(storage);
	storage.phase = .Integrating;
	return .Ok;
}

body_control_step_complete :: #force_no_inline proc (storage: ^Body_Control_Storage, status: Physics_Status)
{
	index: int = 0;
	for index < storage.inputs.count
	{
		if storage.inputs.values.memory[index].applied == .Present
		{
			key: i32 = storage.inputs.keys.memory[index];
			_ = util.quick_dictionary_fast_remove(&storage.inputs, &key);
		}
		else
		{
			index += 1;
		}
	}
	for target_index in 0 ..< storage.targets.count
	{
		value: ^Body_Control_Target = &storage.targets.values.memory[target_index];
		if value.phase != .Prepared
		{
			continue;
		}
		key: i32 = storage.targets.keys.memory[target_index];
		location: Body_Memory_Location;
		resolve: Physics_Status;
		location, resolve = bodies_resolve(&storage.simulation.bodies, {key});
		if resolve != .Ok
		{
			continue;
		}
		dynamics: ^Body_Dynamics = &storage.simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index];
		if value.applied == .Missing
		{
			dynamics.motion.velocity = value.previous_velocity;
			value.phase = .Pending;
		}
		else
		{
			// only roundoff is corrected. a custom stage sequence integrating a
			// different duration is never teleported through missing motion
			if status == .Ok
			{
				delta: util.Vector3 = util.vector3_subtract(dynamics.motion.pose.position, value.pose.position);
				if util.vector3_length_squared(delta) <= 1e-8
				{
					dynamics.motion.pose.position = value.pose.position;
				}
			}
			value.phase = .Expired;
		}
	}
	storage.phase = .Idle;
	body_control_select_callbacks(storage);
}
