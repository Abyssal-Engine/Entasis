package simulation_tests

import "core:mem"
import "core:math"
import "core:simd"
import "core:sync"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 256), util.Memory_Status.Ok);
	for power in 0 ..= 22
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 524288, power), util.Memory_Status.Ok);
	}
}

small_description :: proc(pool: ^util.Buffer_Pool) -> physics.Simulation_Create_Description
{
	description := physics.simulation_create_description_default(pool);
	description.allocation_sizes = {
		bodies=32, statics=32, inactive_body_sets=8, shapes_per_type=8,
		constraints=64, constraint_batches=8, initial_constraints_per_type_batch=8,
		minimum_constraints_per_body=8, broad_phase_candidates=128, pairs=128,
		inactive_pairs=64, pending_pairs_per_worker=64, workers=1,
	};
	description.solve_description = {velocity_iteration_count=2, substep_count=4, fallback_batch_threshold=4};
	description.default_pose_context = {gravity={0, -10, 0}};
	description.profiling = .Enabled;
	return description;
}

allocate_simulation :: proc(t: ^testing.T) -> ^physics.Simulation
{
	memory, allocation_error := mem.alloc(size_of(physics.Simulation), align_of(physics.Simulation));
	if !testing.expect(t, allocation_error == nil && memory != nil)
	{
		return nil;
	}
	simulation := (^physics.Simulation)(memory);
	simulation^ = {};
	return simulation;
}

free_simulation :: proc(t: ^testing.T, simulation: ^physics.Simulation)
{
	if simulation != nil
	{
		testing.expect_value(t, mem.free(simulation), mem.Allocator_Error.None);
	}
}

dynamic_body :: proc(shape: physics.Typed_Index = {}) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity()},
		velocity={linear={1, 0, 0}},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=0.5},
		activity={sleep_threshold=0.001, minimum_timestep_count_under_threshold=255},
	};
}

invalid_body_description_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = shape;
	_ = orientation;
	_ = registry;
	return {}, .Invalid_Description;
}

Integration_Schedule_State :: struct
{
	payload_valid:       physics.Reference_State,
	saw_dynamic:         physics.Reference_State,
	saw_kinematic:       physics.Reference_State,
	saw_masked_lane:     physics.Reference_State,
	saw_nonzero_position: physics.Reference_State,
	activation_simulation: ^physics.Simulation,
	owner_bindings_ready:  physics.Reference_State,
	custom_step_envelope:  physics.Reference_State,
	narrow_initialize_status: physics.Physics_Status,
	activation_sequence:   i32,
	pose_activation_order: i32,
	narrow_activation_order: i32,
	pose_dispose_count:    i32,
	narrow_dispose_count:  i32,
	custom_step_count:     i32,
	prepare_count:       i32,
	integrate_count:     i32,
	prepare_dts:         [3]f32,
	integrations_per_prepare: [3]i32,
	scheduler_count:     i32,
	scheduler_indices:   [4]i32,
	worker_mask:         u64,
	completion_status:   physics.Physics_Status,
	completion_calls:    i32,
	completion_stage:    physics.Timestep_Completion_Stage,
}

test_timestep_completion :: proc "contextless" (
	user_context: rawptr, stage: physics.Timestep_Completion_Stage, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> physics.Physics_Status
{
	_ = dt;
	_ = dispatcher;
	state := (^Integration_Schedule_State)(user_context);
	state.completion_calls += 1;
	state.completion_stage = stage;
	return state.completion_status;
}

test_simulation_owner_bindings :: proc "contextless" (
	simulation: ^physics.Simulation,
) -> physics.Reference_State
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Missing;
	}
	shapes := physics.simulation_shape_registry(simulation);
	if shapes == nil || shapes.state != .Allocated ||
		simulation.bodies.state != .Allocated ||
		simulation.statics.state != .Allocated ||
		simulation.broad_phase.state != .Ready ||
		simulation.solver.state != .Ready ||
		simulation.narrow_phase.state != .Ready ||
		simulation.integrator.state != .Ready ||
		simulation.sleeper.state != .Ready ||
		simulation.bodies.shapes != shapes ||
		simulation.statics.shapes != shapes ||
		simulation.broad_phase.shapes != shapes ||
		simulation.broad_phase.bodies != &simulation.bodies ||
		simulation.broad_phase.statics != &simulation.statics ||
		simulation.narrow_phase.shapes != shapes ||
		simulation.narrow_phase.bodies != &simulation.bodies ||
		simulation.narrow_phase.statics != &simulation.statics ||
		simulation.narrow_phase.broad_phase != &simulation.broad_phase ||
		simulation.narrow_phase.solver != &simulation.solver ||
		simulation.narrow_phase.awakener != &simulation.awakener ||
		simulation.integrator.bodies != &simulation.bodies ||
		simulation.sleeper.bodies != &simulation.bodies ||
		simulation.sleeper.broad_phase != &simulation.broad_phase ||
		simulation.sleeper.solver != &simulation.solver ||
		simulation.sleeper.pair_cache != &simulation.narrow_phase.pair_cache ||
		simulation.awakener.sleeper != &simulation.sleeper
	{
		return .Missing;
	}
	return .Present;
}

test_pose_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> physics.Physics_Status
{
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	state := (^Integration_Schedule_State)(user_context);
	state.activation_sequence += 1;
	state.pose_activation_order = state.activation_sequence;
	state.activation_simulation = simulation;
	state.owner_bindings_ready = test_simulation_owner_bindings(simulation);
	return .Ok;
}

test_narrow_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> physics.Physics_Status
{
	if user_context == nil
	{
		return .Invalid_Argument;
	}
	state := (^Integration_Schedule_State)(user_context);
	state.activation_sequence += 1;
	state.narrow_activation_order = state.activation_sequence;
	if state.activation_simulation != simulation
	{
		state.owner_bindings_ready = .Missing;
	}
	if test_simulation_owner_bindings(simulation) != .Present
	{
		state.owner_bindings_ready = .Missing;
	}
	return state.narrow_initialize_status;
}

test_narrow_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	speculative_margin: ^f32,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _ = user_context, worker_index, a, b, speculative_margin;
	return .Allow;
}

test_narrow_allow_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	child_a, child_b: int,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b;
	return .Allow;
}

test_narrow_configure :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	manifold: ^physics.Manifold_Result,
	material: ^physics.Contact_Material_Properties,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _ = user_context, worker_index, a, b, manifold;
	material^ = {
		friction_coefficient=1,
		spring_settings=physics.spring_settings_create(30, 1),
		maximum_recovery_velocity=2,
	};
	return .Allow;
}

test_narrow_configure_child :: proc "contextless" (
	user_context: rawptr, worker_index: int, a, b: physics.Collidable_Reference,
	child_a, child_b: int, manifold: ^physics.Convex_Contact_Manifold,
) -> physics.Collision_Testing_State
{
	_, _, _, _, _, _, _ =
		user_context, worker_index, a, b, child_a, child_b, manifold;
	return .Allow;
}

test_pose_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> physics.Physics_Status
{
	if user_context == nil || dt <= 0
	{
		return .Invalid_Argument;
	}
	state := (^Integration_Schedule_State)(user_context);
	if state.prepare_count < len(state.prepare_dts)
	{
		state.prepare_dts[state.prepare_count] = dt;
	}
	state.prepare_count += 1;
	return .Ok;
}

test_pose_integrate_velocity :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	state := (^Integration_Schedule_State)(user_context);
	state.integrate_count += 1;
	if state.prepare_count > 0 && state.prepare_count <= len(state.integrations_per_prepare)
	{
		state.integrations_per_prepare[state.prepare_count - 1] += 1;
	}
	state.worker_mask |= u64(1) << u64(worker_index);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		body_index := simd.extract(body_indices, lane);
		lane_mask := simd.extract(integration_mask, lane);
		if lane_mask == 0
		{
			state.saw_masked_lane = .Present;
			if body_index != -1
			{
				state.payload_valid = .Missing;
			}
			continue;
		}
		lane_dt := simd.extract(dt, lane);
		if body_index < 0 || lane_dt <= 0
		{
			state.payload_valid = .Missing;
			continue;
		}
		lane_position := util.vector3_wide_read_slot(position, lane);
		if util.vector3_length_squared(lane_position) > 0
		{
			state.saw_nonzero_position = .Present;
		}
		orientation_length_squared :=
			simd.extract(orientation.x, lane) * simd.extract(orientation.x, lane) +
			simd.extract(orientation.y, lane) * simd.extract(orientation.y, lane) +
			simd.extract(orientation.z, lane) * simd.extract(orientation.z, lane) +
			simd.extract(orientation.w, lane) * simd.extract(orientation.w, lane);
		if orientation_length_squared < 0.99 || orientation_length_squared > 1.01
		{
			state.payload_valid = .Missing;
		}
		inverse_mass := simd.extract(inertia.inverse_mass, lane);
		if inverse_mass == 0
		{
			state.saw_kinematic = .Present;
			continue;
		}
		state.saw_dynamic = .Present;
		linear := util.vector3_wide_read_slot(velocity.linear, lane);
		linear.y -= 10 * lane_dt;
		linear.z += lane_position.x * lane_dt;
		util.vector3_wide_write_slot(&velocity.linear, lane, linear);
	}
}

test_pose_dispose :: proc "contextless" (user_context: rawptr)
{
	if user_context != nil
	{
		(^Integration_Schedule_State)(user_context).pose_dispose_count += 1;
	}
}

test_narrow_dispose :: proc "contextless" (user_context: rawptr)
{
	if user_context != nil
	{
		(^Integration_Schedule_State)(user_context).narrow_dispose_count += 1;
	}
}

test_custom_timestep :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> physics.Physics_Status
{
	_ = dispatcher;
	if user_context == nil || simulation == nil || !(dt > 0)
	{
		return .Invalid_Argument;
	}
	state := (^Integration_Schedule_State)(user_context);
	state.custom_step_count += 1;
	if simulation.state == .Stepping &&
		simulation.profiler.trace_count == 0 &&
		simulation.profiler.stage_activity[physics.Simulation_Stage.Timestep] ==
		.Present
	{
		state.custom_step_envelope = .Present;
	}
	else
	{
		state.custom_step_envelope = .Missing;
	}
	return .Ok;
}

test_velocity_iteration_schedule :: proc "contextless" (user_context: rawptr, substep_index: int) -> i32
{
	state := (^Integration_Schedule_State)(user_context);
	if substep_index >= 0 && substep_index < len(state.scheduler_indices)
	{
		state.scheduler_indices[substep_index] = i32(substep_index);
	}
	state.scheduler_count += 1;
	schedule := [4]i32{1, 3, 2, 4};
	if substep_index < 0 || substep_index >= len(schedule)
	{
		return 0;
	}
	return schedule[substep_index];
}

integrate_angular_mode :: proc(
	t: ^testing.T, pool: ^util.Buffer_Pool, mode: physics.Angular_Integration_Mode,
) -> physics.Body_Description
{
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return {};
	}
	defer free_simulation(t, simulation);
	description := small_description(pool);
	description.default_pose_context.gravity = {};
	description.solve_description.substep_count = 1;
	if !testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok)
	{
		return {};
	}
	defer physics.simulation_destroy(simulation);
	simulation.integrator.callbacks.angular_mode = mode;
	body_description := dynamic_body();
	body_description.velocity.linear = {};
	body_description.velocity.angular = {1, 2, 3};
	body_description.local_inertia.inverse_inertia_tensor = {1, 0, 0.5, 0, 0, 0.25};
	body_description.activity.sleep_threshold = -1;
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	if !testing.expect_value(t, body_status, physics.Physics_Status.Ok)
	{
		return {};
	}
	if !testing.expect_value(t, physics.simulation_timestep(simulation, 0.1), physics.Physics_Status.Ok)
	{
		return {};
	}
	result, result_status := physics.bodies_get_description(&simulation.bodies, body);
	testing.expect_value(t, result_status, physics.Physics_Status.Ok);
	return result;
}

@(test)
default_stage_order_substeps_and_kinematics :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer free_simulation(t, simulation);
	description := small_description(&pool);
	description.allocation_sizes.workers = 10;
	callback_state := Integration_Schedule_State{payload_valid=.Present};
	failed_simulation := allocate_simulation(t);
	if failed_simulation == nil
	{
		return;
	}
	defer free_simulation(t, failed_simulation);
	failed_state := Integration_Schedule_State{
		payload_valid=.Present,
		narrow_initialize_status=.Invalid_Description,
	};
	failed_description := small_description(&pool);
	failed_description.pose_callbacks = {
		initialize=test_pose_initialize,
		prepare_for_integration=test_pose_prepare,
		integrate_velocity=test_pose_integrate_velocity,
		dispose=test_pose_dispose,
		user_context=&failed_state,
	};
	failed_description.use_default_pose = .Missing;
	failed_description.narrow_callbacks = {
		initialize=test_narrow_initialize,
		allow=test_narrow_allow,
		allow_child=test_narrow_allow_child,
		configure=test_narrow_configure,
		configure_child=test_narrow_configure_child,
		dispose=test_narrow_dispose,
		user_context=&failed_state,
	};
	failed_description.use_default_narrow = .Missing;
	testing.expect_value(
		t, physics.simulation_create(
		failed_simulation, &failed_description,
	).status,
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(t, failed_state.activation_simulation, failed_simulation);
	testing.expect_value(
		t, failed_state.owner_bindings_ready,
		physics.Reference_State.Present,
	);
	testing.expect_value(t, failed_state.pose_activation_order, i32(1));
	testing.expect_value(t, failed_state.narrow_activation_order, i32(2));
	testing.expect_value(t, failed_state.pose_dispose_count, i32(1));
	testing.expect_value(t, failed_state.narrow_dispose_count, i32(0));
	testing.expect_value(
		t, failed_simulation.state,
		physics.Simulation_State.Uninitialized,
	);
	description.pose_callbacks = {
		initialize=test_pose_initialize,
		prepare_for_integration=test_pose_prepare,
		integrate_velocity=test_pose_integrate_velocity,
		dispose=test_pose_dispose,
		angular_mode=.Nonconserving,
		allow_substeps_for_unconstrained=.Enabled,
		integrate_kinematic_velocity=.Enabled,
		user_context=&callback_state,
	};
	description.use_default_pose = .Missing;
	description.narrow_callbacks = {
		initialize=test_narrow_initialize,
		allow=test_narrow_allow,
		allow_child=test_narrow_allow_child,
		configure=test_narrow_configure,
		configure_child=test_narrow_configure_child,
		dispose=test_narrow_dispose,
		user_context=&callback_state,
	};
	description.use_default_narrow = .Missing;
	description.solve_description.velocity_iteration_scheduler = test_velocity_iteration_schedule;
	description.solve_description.scheduler_context = &callback_state;
	description.solve_description.fallback_batch_threshold = 2;
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	testing.expect_value(t, callback_state.activation_simulation, simulation);
	testing.expect_value(
		t, callback_state.owner_bindings_ready,
		physics.Reference_State.Present,
	);
	testing.expect_value(t, callback_state.pose_activation_order, i32(1));
	testing.expect_value(t, callback_state.narrow_activation_order, i32(2));
	body_description := dynamic_body();
	body, status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	kinematic_description := dynamic_body();
	kinematic_description.pose.position = {0, 2, 0};
	kinematic_description.local_inertia = {};
	kinematic, kinematic_status := physics.simulation_add_body(simulation, &kinematic_description);
	testing.expect_value(t, kinematic_status, physics.Physics_Status.Ok);
	extra_handles: [3]physics.Body_Handle;
	for index in 0 ..< len(extra_handles)
	{
		extra_description := dynamic_body();
		extra_description.pose.position = {f32(index + 1) * 2, f32(index & 1), 0};
		extra_description.velocity = {};
		extra_handles[index], status = physics.simulation_add_body(simulation, &extra_description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	connected := [4]physics.Body_Handle{body, kinematic, {}, {}};
	ball_socket := physics.Ball_Socket{
		spring_settings={angular_frequency=10, twice_damping_ratio=2},
	};
	_, constraint_status := physics.simulation_add_constraint(simulation, &connected, &ball_socket);
	testing.expect_value(t, constraint_status, physics.Physics_Status.Ok);
	connected[1] = extra_handles[0];
	_, constraint_status = physics.simulation_add_constraint(simulation, &connected, &ball_socket);
	testing.expect_value(t, constraint_status, physics.Physics_Status.Ok);
	connected[1] = extra_handles[1];
	_, constraint_status = physics.simulation_add_constraint(simulation, &connected, &ball_socket);
	testing.expect_value(t, constraint_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.solver.active_set.batch_count, i32(3));
	dispatcher: util.Thread_Dispatcher;
	if !testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher, 10), util.Threading_Status.Ok,
	)
	{
		return;
	}
	defer testing.expect_value(
		t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok,
	);
	dispatcher_boundary := util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Ok,
	);
	expected_stages := [12]physics.Simulation_Stage{
		.Sleep, .Slept_Callback, .Predict_Bounding_Boxes, .Before_Collision_Callback,
		.Broad_Phase, .Collision_Detection, .Collisions_Detected_Callback, .Solve,
		.Constraints_Solved_Callback, .Incrementally_Optimize, .Cleanup, .Timestep,
	};
	testing.expect_value(t, simulation.profiler.trace_count, len(expected_stages));
	for index in 0 ..< len(expected_stages)
	{
		testing.expect_value(t, simulation.profiler.trace[index], expected_stages[index]);
	}
	body_result, body_result_status := physics.bodies_get_description(&simulation.bodies, body);
	testing.expect_value(t, body_result_status, physics.Physics_Status.Ok);
	testing.expect(t, math.abs(body_result.velocity.linear.x - 3.5255663) < 2e-5);
	testing.expect(t, math.abs(body_result.velocity.linear.y - 4.563339) < 2e-5);
	testing.expect(t, math.abs(body_result.velocity.linear.z - 0.08822714) < 2e-5);
	testing.expect(t, math.abs(body_result.pose.position.x - 0.4523896) < 2e-5);
	testing.expect(t, math.abs(body_result.pose.position.y - 0.39503932) < 2e-5);
	testing.expect(t, math.abs(body_result.pose.position.z - 0.0056162775) < 2e-5);
	testing.expect_value(t, callback_state.payload_valid, physics.Reference_State.Present);
	testing.expect_value(t, callback_state.saw_dynamic, physics.Reference_State.Present);
	testing.expect_value(t, callback_state.saw_kinematic, physics.Reference_State.Present);
	testing.expect_value(t, callback_state.saw_masked_lane, physics.Reference_State.Present);
	testing.expect_value(t, callback_state.saw_nonzero_position, physics.Reference_State.Present);
	testing.expect_value(t, callback_state.prepare_count, i32(3));
	expected_prepare_dts := [3]f32{0.1, 0.025, 0.025};
	expected_integration_counts := [3]i32{1, 16, 4};
	for index in 0 ..< len(expected_prepare_dts)
	{
		testing.expect(t, math.abs(callback_state.prepare_dts[index] - expected_prepare_dts[index]) < 1e-7);
		testing.expect_value(t, callback_state.integrations_per_prepare[index], expected_integration_counts[index]);
	}
	testing.expect_value(t, callback_state.scheduler_count, i32(4));
	for index in 0 ..< len(callback_state.scheduler_indices)
	{
		testing.expect_value(t, callback_state.scheduler_indices[index], i32(index));
	}
	testing.expect_value(t, callback_state.worker_mask, u64(1));
	kinematic_result, kinematic_result_status := physics.bodies_get_description(&simulation.bodies, kinematic);
	testing.expect_value(t, kinematic_result_status, physics.Physics_Status.Ok);
	testing.expect_value(t, kinematic_result.pose.position.x, f32(0.1));
	testing.expect_value(t, kinematic_result.pose.position.y, f32(2));
	simulation.integrator.callbacks.allow_substeps_for_unconstrained = .Disabled;
	callback_state.prepare_count = 0;
	callback_state.integrate_count = 0;
	callback_state.prepare_dts = {};
	callback_state.integrations_per_prepare = {};
	testing.expect_value(t, physics.simulation_timestep(simulation, 0.1), physics.Physics_Status.Ok);
	testing.expect_value(t, callback_state.prepare_count, i32(3));
	expected_prepare_dts = {0.1, 0.025, 0.1};
	expected_integration_counts = {1, 16, 1};
	for index in 0 ..< len(expected_prepare_dts)
	{
		testing.expect(t, math.abs(callback_state.prepare_dts[index] - expected_prepare_dts[index]) < 1e-7);
		testing.expect_value(t, callback_state.integrations_per_prepare[index], expected_integration_counts[index]);
	}
	default_timestepper := simulation.timestepper;
	callback_state.completion_status = .Capacity_Missing;
	callback_state.completion_calls = 0;
	simulation.timestep_callbacks = {
		stage_completed=test_timestep_completion,
		user_context=&callback_state,
	};
	step_index_before_stage_error := simulation.step_index;
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Capacity_Missing,
	);
	testing.expect_value(t, callback_state.completion_calls, i32(1));
	testing.expect_value(
		t, callback_state.completion_stage,
		physics.Timestep_Completion_Stage.Slept,
	);
	testing.expect_value(t, simulation.step_index, step_index_before_stage_error);
	testing.expect_value(t, dispatcher_boundary.unmanaged_context, rawptr(nil));
	session := util.thread_dispatcher_session_pointer(&dispatcher);
	testing.expect_value(
		t,
		util.Thread_Dispatcher_Session_Lifecycle(sync.atomic_load_explicit(
		&session.lifecycle, .Acquire,
	)),
		util.Thread_Dispatcher_Session_Lifecycle.Inactive,
	);
	testing.expect_value(
		t, sync.atomic_load_explicit(&session.remaining, .Acquire), sync.Futex(0),
	);
	simulation.timestep_callbacks = {};
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Ok,
	);
	step_index_before_busy_entry := simulation.step_index;
	sync.atomic_store_explicit(
		&dispatcher.dispatch_state, u32(util.Dispatch_State.Running), .Release,
	);
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Invalid_Argument,
	);
	sync.atomic_store_explicit(
		&dispatcher.dispatch_state, u32(util.Dispatch_State.Idle), .Release,
	);
	testing.expect_value(t, simulation.step_index, step_index_before_busy_entry);
	sync.atomic_store_explicit(
		&dispatcher.lifecycle, u32(util.Dispatcher_Lifecycle.Shutting_Down), .Release,
	);
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Invalid_Argument,
	);
	sync.atomic_store_explicit(
		&dispatcher.lifecycle, u32(util.Dispatcher_Lifecycle.Ready), .Release,
	);
	testing.expect_value(t, simulation.step_index, step_index_before_busy_entry);
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Ok,
	);
	simulation.timestepper = {
		step=test_custom_timestep,
		user_context=&callback_state,
	};
	invalid_dispatcher := util.Thread_Dispatcher_Boundary{worker_count=1};
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(
		t, physics.simulation_timestep(
		simulation, 0.1, &invalid_dispatcher,
	),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(t, callback_state.custom_step_count, i32(0));
	testing.expect_value(
		t, physics.simulation_timestep(simulation, 0.1, dispatcher_boundary),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, callback_state.custom_step_count, i32(1));
	testing.expect_value(
		t, callback_state.custom_step_envelope,
		physics.Reference_State.Present,
	);
	testing.expect_value(t, simulation.profiler.trace_count, 1);
	testing.expect_value(
		t, simulation.profiler.trace[0],
		physics.Simulation_Stage.Timestep,
	);
	trace_count_before_clear := simulation.profiler.trace_count;
	trace_before_clear := simulation.profiler.trace[0];
	timestep_count_before_clear :=
		simulation.profiler.stage_counts[physics.Simulation_Stage.Timestep];
	timestep_duration_before_clear :=
		simulation.profiler.stage_durations_nanoseconds[
		physics.Simulation_Stage.Timestep
	];
	testing.expect_value(
		t, physics.simulation_clear(simulation),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, simulation.profiler.trace_count, trace_count_before_clear,
	);
	testing.expect_value(t, simulation.profiler.trace[0], trace_before_clear);
	testing.expect_value(
		t,
		simulation.profiler.stage_counts[
		physics.Simulation_Stage.Timestep
	],
		timestep_count_before_clear,
	);
	testing.expect_value(
		t,
		simulation.profiler.stage_durations_nanoseconds[
		physics.Simulation_Stage.Timestep
	],
		timestep_duration_before_clear,
	);
	simulation.timestepper = default_timestepper;
}

@(test)
simulation_queries_forward_to_phase_four_collectors :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer free_simulation(t, simulation);
	description := small_description(&pool);
	description.default_pose_context.gravity = {};
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	sphere := physics.Sphere{radius=1};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body_description := dynamic_body(shape);
	body_description.velocity = {};
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	static_description := physics.Static_Description{
		pose={orientation=util.quaternion_identity(), position={4, 0, 0}}, shape=shape,
	};
	static_handle, static_status := physics.simulation_add_static(simulation, &static_description);
	testing.expect_value(t, static_status, physics.Physics_Status.Ok);
	ray_hits: [8]physics.Ray_Query_Hit;
	ray_collector: physics.Ray_Query_Collector;
	testing.expect_value(
		t, physics.ray_query_collector_initialize(
		&ray_collector,
		{memory=&ray_hits[0], length=i32(len(ray_hits)), id=-1},
		.All
	),
		physics.Physics_Status.Ok,
	);
	ray := physics.Tree_Ray{origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=20};
	testing.expect_value(
		t,
		physics.simulation_ray_query(simulation, ray, &ray_collector, &pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, ray_collector.count, 2);
	volume_hits: [8]physics.Volume_Query_Hit;
	volume_collector := physics.Volume_Query_Collector{hits={
			memory=&volume_hits[0],
			length=i32(len(volume_hits)),
			id=-1,
	}};
	volume := util.Bounding_Box{min={-2, -2, -2}, max={6, 2, 2}};
	testing.expect_value(
		t,
		physics.simulation_volume_query(simulation, volume, &volume_collector, &pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, volume_collector.count, 2);
	overlap_hits: [8]physics.Overlap_Query_Hit;
	overlap_collector := physics.Overlap_Query_Collector{hits={
			memory=&overlap_hits[0],
			length=i32(len(overlap_hits)),
			id=-1,
	}};
	testing.expect_value(
		t, physics.simulation_overlap_query(
		simulation, shape, {orientation=util.quaternion_identity()}, &overlap_collector, &pool,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, overlap_collector.count, 1);
	sweep_hits: [8]physics.Sweep_Query_Hit;
	sweep_callback_count: i32;
	sweep_collector := physics.Sweep_Query_Collector{
		hits={memory=&sweep_hits[0], length=i32(len(sweep_hits)), id=-1},
		mode=.All,
		callbacks={
			hit=proc "contextless" (
				user_context: rawptr, hit: ^physics.Sweep_Query_Hit, maximum_t: ^f32,
			) -> physics.Physics_Status
			{
				callback_count := (^i32)(user_context);
				if callback_count == nil || hit == nil || maximum_t == nil
				{
					return .Invalid_Argument;
				}
				callback_count^ += 1;
				maximum_t^ = min(maximum_t^, hit.sweep.t1);
				return .Ok;
			},
			user_context=&sweep_callback_count,
		},
	};
	testing.expect_value(
		t, physics.simulation_sweep_query(
		simulation, shape, {orientation=util.quaternion_identity(), position={-4, 0, 0}},
		{linear={1, 0, 0}}, 10, 1e-4, 1e-4, 32, &sweep_collector,
		&pool,
	), physics.Physics_Status.Ok,
	);
	active_reference, active_reference_status := physics.collidable_reference_body(.Dynamic, body);
	static_reference, static_reference_status := physics.collidable_reference_static(static_handle);
	testing.expect_value(t, active_reference_status, physics.Physics_Status.Ok);
	testing.expect_value(t, static_reference_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sweep_callback_count, i32(1));
	testing.expect_value(t, sweep_collector.count, 1);
	testing.expect_value(t, sweep_collector.hits.memory[0].target_id, i32(active_reference.packed));
	testing.expect(t, sweep_collector.hits.memory[0].target_id != i32(static_reference.packed));

	triangles := [2]physics.Triangle{
		{{0, -2, -2}, {0, 2, -2}, {0, 2, 2}},
		{{0, -2, -2}, {0, 2, 2}, {0, -2, 2}},
	};
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(
		&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool,
	), physics.Physics_Status.Ok);
	mesh_shape, mesh_shape_status := physics.shape_registry_add(
		&simulation.shapes, physics.MESH_TYPE_ID, &mesh,
	);
	testing.expect_value(t, mesh_shape_status, physics.Physics_Status.Ok);
	unsupported_static, unsupported_static_status := physics.simulation_add_static(
		simulation, &physics.Static_Description{
			pose={orientation=util.quaternion_identity(), position={1, 0, 0}},
			shape=mesh_shape,
		},
	);
	testing.expect_value(t, unsupported_static_status, physics.Physics_Status.Ok);
	all_sweep_hits: [8]physics.Sweep_Query_Hit;
	all_sweep_collector := physics.Sweep_Query_Collector{
		hits={memory=&all_sweep_hits[0], length=i32(len(all_sweep_hits)), id=-1},
		mode=.All,
	};
	testing.expect_value(t, physics.simulation_sweep_query(
		simulation, mesh_shape,
		{orientation=util.quaternion_identity(), position={-4, 0, 0}},
		{linear={1, 0, 0}}, 10, 1e-4, 1e-4, 32, &all_sweep_collector, &pool,
	), physics.Physics_Status.Ok);
	unsupported_reference, unsupported_reference_status := physics.collidable_reference_static(
		unsupported_static,
	);
	testing.expect_value(t, unsupported_reference_status, physics.Physics_Status.Ok);
	testing.expect_value(t, all_sweep_collector.count, 2);
	supported_mask: u8;
	for hit_index in 0 ..< all_sweep_collector.count
	{
		target_id := all_sweep_collector.hits.memory[hit_index].target_id;
		testing.expect(t, target_id != i32(unsupported_reference.packed));
		if target_id == i32(active_reference.packed)
		{
			supported_mask |= 1;
		}
		if target_id == i32(static_reference.packed)
		{
			supported_mask |= 2;
		}
	}
	testing.expect_value(t, supported_mask, u8(3));
}

@(test)
continuous_body_uses_sweep_continuation_before_solve :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer free_simulation(t, simulation);
	description := small_description(&pool);
	description.default_pose_context.gravity = {};
	description.solve_description.substep_count = 1;
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	defer physics.simulation_destroy(simulation);
	stored_material := physics.Contact_Material_Properties{
		friction_coefficient=1,
		spring_settings=physics.spring_settings_create(30, 1),
		maximum_recovery_velocity=2,
	};
	testing.expect_value(
		t, physics.narrow_phase_bind_default_stored_completion(
		&simulation.narrow_phase, stored_material,
	),
		physics.Physics_Status.Ok,
	);
	box := physics.Box{0.75, 1, 0.35};
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body_description := dynamic_body(shape);
	body_description.pose.position = {-5, 0, 0};
	body_description.velocity.linear = {20, 0, 0};
	body_description.velocity.angular = {0, 3, 0};
	body_description.collidable.continuity = physics.continuous_detection_continuous(1e-4, 1e-4);
	body_description.collidable.maximum_speculative_margin = 0.1;
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	static_description := physics.Static_Description{pose={orientation=util.quaternion_identity()}, shape=shape};
	_, static_status := physics.simulation_add_static(simulation, &static_description);
	testing.expect_value(t, static_status, physics.Physics_Status.Ok);
	predecessor_simulation := allocate_simulation(t);
	if predecessor_simulation == nil
	{
		return;
	}
	defer free_simulation(t, predecessor_simulation);
	predecessor_description := small_description(&pool);
	predecessor_description.default_pose_context.gravity = {};
	predecessor_description.solve_description.substep_count = 1;
	testing.expect_value(
		t, physics.simulation_create(
		predecessor_simulation, &predecessor_description,
	).status,
		physics.Physics_Status.Ok,
	);
	defer physics.simulation_destroy(predecessor_simulation);
	testing.expect_value(
		t, physics.narrow_phase_bind_default_stored_completion(
		&predecessor_simulation.narrow_phase, stored_material,
	),
		physics.Physics_Status.Ok,
	);
	predecessor_shape, predecessor_shape_status := physics.shape_registry_add(
		&predecessor_simulation.shapes, physics.BOX_TYPE_ID, &box,
	);
	testing.expect_value(t, predecessor_shape_status, physics.Physics_Status.Ok);
	unused_capsule := physics.Capsule{radius=0.125, half_length=0.125};
	_, unused_capsule_status := physics.shape_registry_add(
		&predecessor_simulation.shapes, physics.CAPSULE_TYPE_ID, &unused_capsule,
	);
	testing.expect_value(t, unused_capsule_status, physics.Physics_Status.Ok);
	predecessor_body_description := body_description;
	predecessor_body_description.collidable.shape = predecessor_shape;
	_, predecessor_body_status := physics.simulation_add_body(
		predecessor_simulation, &predecessor_body_description,
	);
	testing.expect_value(t, predecessor_body_status, physics.Physics_Status.Ok);
	predecessor_static_description := static_description;
	predecessor_static_description.shape = predecessor_shape;
	_, predecessor_static_status := physics.simulation_add_static(
		predecessor_simulation, &predecessor_static_description,
	);
	testing.expect_value(t, predecessor_static_status, physics.Physics_Status.Ok);
	direct_status := physics.simulation_timestep(simulation, 0.5);
	predecessor_status := physics.simulation_timestep(predecessor_simulation, 0.5);
	testing.expect_value(t, direct_status, physics.Physics_Status.Ok);
	testing.expect_value(t, predecessor_status, direct_status);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		&simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Box_Direct,
	);
	testing.expect_value(
		t, physics.narrow_phase_collision_route_state(
		&predecessor_simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	continuation := simulation.narrow_phase.continuations.memory[0];
	testing.expect_value(
		t, physics.narrow_phase_box_sphere_route_state(
		&predecessor_simulation.narrow_phase, 1,
	),
		physics.Narrow_Phase_Collision_Route.Predecessor,
	);
	predecessor_continuation := predecessor_simulation.narrow_phase.continuations.memory[0];
	testing.expect_value(t, continuation, predecessor_continuation);
	testing.expect_value(t, continuation.kind, physics.Narrow_Phase_Continuation_Type.Continuous);
	testing.expect(t, continuation.t > 0 && continuation.t < 0.5);
	testing.expect_value(t, continuation.relative_linear_velocity.x, f32(-20));
	testing.expect_value(t, continuation.angular_a.y, f32(3));
	testing.expect_value(
		t, predecessor_simulation.narrow_phase.results.memory[0].state,
		physics.Narrow_Phase_Result_State.Accepted,
	);
	sampled_pose := physics.sweep_pose_at(body_description.pose, body_description.velocity, continuation.t);
	testing.expect(t, math.abs(sampled_pose.orientation.y) > 1e-4);
	raw_shape, _, raw_shape_status := physics.shape_registry_resolve(&simulation.shapes, shape);
	testing.expect_value(t, raw_shape_status, physics.Physics_Status.Ok);
	raw_manifold, raw_manifold_status := physics.collision_task_registry_test_convex(
		&simulation.collision_tasks, physics.BOX_TYPE_ID, physics.BOX_TYPE_ID,
		raw_shape, raw_shape, sampled_pose, static_description.pose, 0.1, &simulation.shapes,
	);
	testing.expect_value(t, raw_manifold_status, physics.Physics_Status.Ok);
	completed := predecessor_simulation.narrow_phase.results.memory[0].manifold.convex;
	if !testing.expect_value(t, completed.count, raw_manifold.count)
	{
		return;
	}
	for contact_index in 0 ..< int(completed.count)
	{
		contact := raw_manifold.contacts[contact_index];
		angular_a := util.vector3_cross(continuation.angular_a, contact.offset);
		contact_velocity := util.vector3_subtract(continuation.relative_linear_velocity, angular_a);
		expected_depth := contact.depth - util.vector3_dot(contact_velocity, raw_manifold.normal) * continuation.t;
		testing.expect(t, math.abs(completed.contacts[contact_index].depth - expected_depth) < 2e-4);
	}
	testing.expect_value(t, simulation.narrow_phase.pair_cache.mapping.count, 1);
	testing.expect_value(t, simulation.solver.active_set.constraint_count, i32(1));
	testing.expect_value(
		t, predecessor_simulation.narrow_phase.pair_cache.mapping.count,
		simulation.narrow_phase.pair_cache.mapping.count,
	);
	testing.expect_value(
		t, predecessor_simulation.solver.active_set.constraint_count,
		simulation.solver.active_set.constraint_count,
	);
	direct_handle := simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
	predecessor_handle := predecessor_simulation.narrow_phase.pair_cache.mapping.values.memory[0].constraint_handle;
	direct_view: physics.Contact_Constraint_Data_View;
	predecessor_view: physics.Contact_Constraint_Data_View;
	capture_view := proc "contextless" (
		user_context: rawptr, view: ^physics.Contact_Constraint_Data_View,
	)
	{
		(^physics.Contact_Constraint_Data_View)(user_context)^ = view^;
	}
	testing.expect_value(
		t, physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, direct_handle, capture_view, &direct_view,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.narrow_phase_try_extract_solver_contact_data(
		&predecessor_simulation.narrow_phase, predecessor_handle,
		capture_view, &predecessor_view,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, direct_view, predecessor_view);
	body_result, body_result_status := physics.bodies_get_description(&simulation.bodies, body);
	testing.expect_value(t, body_result_status, physics.Physics_Status.Ok);
	testing.expect(t, body_result.pose.position.x < 0);
	rotating_simulation := allocate_simulation(t);
	if rotating_simulation == nil
	{
		return;
	}
	defer free_simulation(t, rotating_simulation);
	rotating_description := small_description(&pool);
	rotating_description.default_pose_context.gravity = {};
	rotating_description.solve_description.substep_count = 1;
	testing.expect_value(
		t, physics.simulation_create(rotating_simulation, &rotating_description).status, physics.Physics_Status.Ok,
	);
	defer physics.simulation_destroy(rotating_simulation);
	rotating_box := physics.Box{half_width=2, half_height=0.1, half_length=0.1};
	rotating_shape, rotating_shape_status := physics.shape_registry_add(
		&rotating_simulation.shapes, physics.BOX_TYPE_ID, &rotating_box,
	);
	testing.expect_value(t, rotating_shape_status, physics.Physics_Status.Ok);
	target_sphere := physics.Sphere{radius=0.2};
	target_shape, target_shape_status := physics.shape_registry_add(
		&rotating_simulation.shapes, physics.SPHERE_TYPE_ID, &target_sphere,
	);
	testing.expect_value(t, target_shape_status, physics.Physics_Status.Ok);
	rotating_body_description := dynamic_body(rotating_shape);
	rotating_body_description.velocity.linear = {};
	rotating_body_description.velocity.angular = {0, 0, f32(math.PI)};
	rotating_body_description.collidable.continuity = physics.continuous_detection_continuous(1e-4, 1e-4);
	rotating_body_description.collidable.maximum_speculative_margin = 0.05;
	_, rotating_body_status := physics.simulation_add_body(rotating_simulation, &rotating_body_description);
	testing.expect_value(t, rotating_body_status, physics.Physics_Status.Ok);
	rotating_static := physics.Static_Description{
		pose={orientation=util.quaternion_identity(), position={0, 1.5, 0}}, shape=target_shape,
	};
	_, rotating_static_status := physics.simulation_add_static(rotating_simulation, &rotating_static);
	testing.expect_value(t, rotating_static_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(rotating_simulation, 0.5), physics.Physics_Status.Ok);
	testing.expect_value(
		t, rotating_simulation.narrow_phase.continuations.memory[0].kind,
		physics.Narrow_Phase_Continuation_Type.Continuous,
	);
	testing.expect_value(
		t, rotating_simulation.narrow_phase.results.memory[0].state, physics.Narrow_Phase_Result_State.Accepted,
	);
}

@(test)
angular_integration_modes_preserve_their_distinct_contracts :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	nonconserving := integrate_angular_mode(t, &pool, .Nonconserving);
	momentum := integrate_angular_mode(t, &pool, .Conserve_Momentum);
	gyroscopic := integrate_angular_mode(t, &pool, .Conserve_Momentum_With_Gyroscopic_Torque);
	testing.expect(t, math.abs(nonconserving.velocity.angular.x - 1) < 1e-6);
	testing.expect(t, math.abs(nonconserving.velocity.angular.y - 2) < 1e-6);
	testing.expect(t, math.abs(nonconserving.velocity.angular.z - 3) < 1e-6);
	momentum_delta := util.vector3_length(util.vector3_subtract(
		momentum.velocity.angular,
		nonconserving.velocity.angular
	));
	gyroscopic_delta := util.vector3_length(util.vector3_subtract(
		gyroscopic.velocity.angular,
		momentum.velocity.angular
	));
	testing.expect(t, momentum_delta > 1e-4);
	testing.expect(t, gyroscopic_delta > 1e-4);
	testing.expect_value(
		t,
		physics.pose_integrator_finite_vector3(momentum.velocity.angular),
		physics.Reference_State.Present
	);
	testing.expect_value(
		t,
		physics.pose_integrator_finite_vector3(gyroscopic.velocity.angular),
		physics.Reference_State.Present
	);
	initial_inverse := util.Symmetric3x3{1, 0, 0.5, 0, 0, 0.25};
	initial_momentum := util.symmetric3x3_transform({1, 2, 3}, util.symmetric3x3_invert(initial_inverse));
	final_rotation := util.matrix3x3_from_quaternion(momentum.pose.orientation);
	final_inverse := util.symmetric3x3_rotation_sandwich(final_rotation, initial_inverse);
	final_momentum := util.symmetric3x3_transform(momentum.velocity.angular, util.symmetric3x3_invert(final_inverse));
	testing.expect(t, util.vector3_length(util.vector3_subtract(final_momentum, initial_momentum)) < 2e-4);
}

@(test)
body_shape_transitions_keep_broad_phase_membership_consistent :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	prepared_shapes: physics.Shape_Registry;
	testing.expect_value(
		t, physics.shape_registry_initialize(&prepared_shapes, 8, &pool),
		physics.Physics_Status.Ok,
	);
	sphere := physics.Sphere{radius=0.5};
	shape, shape_status := physics.shape_registry_add(
		&prepared_shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	other_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&other_pool, 256),
		util.Memory_Status.Ok,
	);
	defer util.buffer_pool_dispose(&other_pool);
	failed_simulation := allocate_simulation(t);
	if failed_simulation == nil
	{
		return;
	}
	defer free_simulation(t, failed_simulation);
	create_failure_description := small_description(&other_pool);
	create_failure_description.shape_registry = &prepared_shapes;
	testing.expect_value(
		t, physics.simulation_create(
		failed_simulation, &create_failure_description,
	).status,
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(
		t, prepared_shapes.state, physics.Body_Set_State.Allocated,
	);
	_, _, resolve_status := physics.shape_registry_resolve(
		&prepared_shapes, shape,
	);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	simulation := allocate_simulation(t);
	if simulation == nil
	{
		return;
	}
	defer free_simulation(t, simulation);
	description := small_description(&pool);
	description.default_pose_context.gravity = {};
	description.allocation_sizes.bodies = 1;
	description.shape_registry = &prepared_shapes;
	testing.expect_value(t, physics.simulation_create(simulation, &description).status, physics.Physics_Status.Ok);
	shapes := physics.simulation_shape_registry(simulation);
	testing.expect_value(t, shapes, &prepared_shapes);
	resident_description := dynamic_body(shape);
	resident_description.pose.position = {-4, 0, 0};
	resident_description.velocity = {};
	resident, resident_status := physics.simulation_add_body(
		simulation, &resident_description,
	);
	testing.expect_value(t, resident_status, physics.Physics_Status.Ok);
	query_hits: [4]physics.Ray_Query_Hit;
	query_collector: physics.Ray_Query_Collector;
	testing.expect_value(
		t, physics.ray_query_collector_initialize(
		&query_collector,
		{memory=&query_hits[0], length=i32(len(query_hits)), id=-1},
		.All,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_ray_query(
		simulation,
		{origin={-6, 0, 0}, direction={1, 0, 0}, maximum_t=4},
		&query_collector, &pool,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, query_collector.count, 1);
	resident_reference, resident_reference_status :=
		physics.collidable_reference_body(.Dynamic, resident);
	testing.expect_value(
		t, resident_reference_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, query_collector.hits.memory[0].target_id,
		i32(resident_reference.packed),
	);
	body_description := dynamic_body();
	body_description.velocity = {};
	body, body_status := physics.simulation_add_body(simulation, &body_description);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 1);
	old_description, old_description_status := physics.bodies_get_description(
		&simulation.bodies, body,
	);
	testing.expect_value(t, old_description_status, physics.Physics_Status.Ok);
	body_location, body_location_status := physics.bodies_resolve(
		&simulation.bodies, body,
	);
	testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[body_location.set_index].collidables.memory[
		body_location.index
	].broad_phase_index,
		i32(-1),
	);
	old_shape_references := shapes.batches[
		physics.SPHERE_TYPE_ID
	].references.memory[physics.typed_index_index(shape)];
	old_resident_reference := simulation.broad_phase.active_leaves.memory[0];
	old_resident_bounds, old_resident_bounds_status := physics.tree_get_leaf_bounds(
		&simulation.broad_phase.active_tree, 0,
	);
	testing.expect_value(
		t, old_resident_bounds_status, physics.Physics_Status.Ok,
	);
	body_description.collidable.shape = shape;
	rejecting_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&rejecting_pool, 256),
		util.Memory_Status.Ok,
	);
	for power in 0 ..< util.BUFFER_POOL_POWER_COUNT
	{
		rejecting_pool.pools[power].next_slot = util.BUFFER_POOL_ID_SLOT_COUNT;
		rejecting_pool.pools[power].free_count = 0;
	}
	original_broad_phase_pool := simulation.broad_phase.pool;
	original_active_tree_pool := simulation.broad_phase.active_tree.pool;
	original_static_tree_pool := simulation.broad_phase.static_tree.pool;
	simulation.broad_phase.pool = &rejecting_pool;
	simulation.broad_phase.active_tree.pool = &rejecting_pool;
	simulation.broad_phase.static_tree.pool = &rejecting_pool;
	admission_status := physics.simulation_apply_body_description(
		simulation, body, &body_description,
	);
	simulation.broad_phase.pool = original_broad_phase_pool;
	simulation.broad_phase.active_tree.pool = original_active_tree_pool;
	simulation.broad_phase.static_tree.pool = original_static_tree_pool;
	testing.expect_value(
		t, admission_status, physics.Physics_Status.Capacity_Missing,
	);
	testing.expect_value(
		t, util.buffer_pool_dispose(&rejecting_pool), util.Memory_Status.Ok,
	);
	failed_description, failed_description_status := physics.bodies_get_description(
		&simulation.bodies, body,
	);
	testing.expect_value(t, failed_description_status, physics.Physics_Status.Ok);
	testing.expect_value(t, failed_description, old_description);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(shape)
	],
		old_shape_references,
	);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 1);
	testing.expect_value(
		t, simulation.broad_phase.active_leaves.memory[0],
		old_resident_reference,
	);
	resident_bounds_after_failure, resident_bounds_after_failure_status :=
		physics.tree_get_leaf_bounds(&simulation.broad_phase.active_tree, 0);
	testing.expect_value(
		t, resident_bounds_after_failure_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(t, resident_bounds_after_failure, old_resident_bounds);
	body_location, body_location_status = physics.bodies_resolve(
		&simulation.bodies, body,
	);
	testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[body_location.set_index].collidables.memory[
		body_location.index
	].broad_phase_index,
		i32(-1),
	);
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body, &body_description), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 2);
	body_location, body_location_status = physics.bodies_resolve(
		&simulation.bodies, body,
	);
	testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
	leaf_index := int(
		simulation.bodies.sets.memory[body_location.set_index].collidables.memory[
		body_location.index
	].broad_phase_index,
	);
	old_reference := simulation.broad_phase.active_leaves.memory[leaf_index];
	old_bounds, old_bounds_status := physics.tree_get_leaf_bounds(
		&simulation.broad_phase.active_tree, leaf_index,
	);
	testing.expect_value(t, old_bounds_status, physics.Physics_Status.Ok);
	invalid_registration := shapes.batches[
		physics.SPHERE_TYPE_ID
	].metadata;
	invalid_registration.bounds = invalid_body_description_bounds;
	invalid_type, invalid_type_status := physics.shape_registry_register_custom(
		shapes, invalid_registration,
	);
	testing.expect_value(t, invalid_type_status, physics.Physics_Status.Ok);
	invalid_shape, invalid_shape_status := physics.shape_registry_add(
		shapes, int(invalid_type), &sphere,
	);
	testing.expect_value(t, invalid_shape_status, physics.Physics_Status.Ok);
	shape_references_before_invalid := shapes.batches[
		physics.SPHERE_TYPE_ID
	].references.memory[physics.typed_index_index(shape)];
	invalid_references_before := shapes.batches[invalid_type].references.memory[
		physics.typed_index_index(invalid_shape)
	];
	valid_description, valid_description_status := physics.bodies_get_description(
		&simulation.bodies, body,
	);
	testing.expect_value(t, valid_description_status, physics.Physics_Status.Ok);
	invalid_description := valid_description;
	invalid_description.collidable.shape = invalid_shape;
	testing.expect_value(
		t,
		physics.simulation_apply_body_description(
		simulation, body, &invalid_description,
	),
		physics.Physics_Status.Invalid_Description,
	);
	failed_description, failed_description_status = physics.bodies_get_description(
		&simulation.bodies, body,
	);
	testing.expect_value(t, failed_description_status, physics.Physics_Status.Ok);
	testing.expect_value(t, failed_description, valid_description);
	testing.expect_value(
		t,
		shapes.batches[physics.SPHERE_TYPE_ID].references.memory[
		physics.typed_index_index(shape)
	],
		shape_references_before_invalid,
	);
	testing.expect_value(
		t,
		shapes.batches[invalid_type].references.memory[
		physics.typed_index_index(invalid_shape)
	],
		invalid_references_before,
	);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 2);
	body_location, body_location_status = physics.bodies_resolve(
		&simulation.bodies, body,
	);
	testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[body_location.set_index].collidables.memory[
		body_location.index
	].broad_phase_index,
		i32(leaf_index),
	);
	testing.expect_value(
		t, simulation.broad_phase.active_leaves.memory[leaf_index],
		old_reference,
	);
	bounds_after_invalid, bounds_after_invalid_status := physics.tree_get_leaf_bounds(
		&simulation.broad_phase.active_tree, leaf_index,
	);
	testing.expect_value(
		t, bounds_after_invalid_status, physics.Physics_Status.Ok,
	);
	testing.expect_value(t, bounds_after_invalid, old_bounds);
	valid_description.pose.position.x = 2;
	testing.expect_value(
		t,
		physics.simulation_apply_body_description(
		simulation, body, &valid_description,
	),
		physics.Physics_Status.Ok,
	);
	body_location, body_location_status = physics.bodies_resolve(
		&simulation.bodies, body,
	);
	testing.expect_value(t, body_location_status, physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		simulation.bodies.sets.memory[body_location.set_index].collidables.memory[
		body_location.index
	].broad_phase_index,
		i32(leaf_index),
	);
	updated_bounds, updated_bounds_status := physics.tree_get_leaf_bounds(
		&simulation.broad_phase.active_tree, leaf_index,
	);
	testing.expect_value(t, updated_bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, updated_bounds.min.x, old_bounds.min.x + 2);
	body_description.collidable.shape = {};
	testing.expect_value(
		t, physics.simulation_apply_body_description(simulation, body, &body_description), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, simulation.broad_phase.active_tree.leaf_count, 1);
	testing.expect_value(t, physics.simulation_remove_body(simulation, body), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_reference_state(&simulation.bodies, body), physics.Reference_State.Missing);
	testing.expect_value(
		t, physics.simulation_remove_body(simulation, resident),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.simulation_destroy(simulation),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, prepared_shapes.state, physics.Body_Set_State.Unallocated,
	);
}
