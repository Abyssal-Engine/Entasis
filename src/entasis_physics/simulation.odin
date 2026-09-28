// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import "base:intrinsics"
import util "entasis:entasis_utilities"

Simulation_State :: enum u8
{
	Uninitialized,
	Ready,
	Stepping,
	Disposed,
}

Simulation_Create_Description :: struct
{
	allocation_sizes:      Simulation_Allocation_Sizes,
	solve_description:     Solve_Description,
	narrow_callbacks:      Narrow_Phase_Callbacks,
	pose_callbacks:        Pose_Integrator_Callbacks,
	default_pose_context:  Default_Pose_Integrator_Context,
	use_default_narrow:    Reference_State,
	use_default_pose:      Reference_State,
	timestepper:           Timestepper,
	timestep_callbacks:    Timestep_Callbacks,
	profiling:             Simulation_Profiling_State,
	pool:                  ^util.Buffer_Pool,
	shape_registry:        ^Shape_Registry,
}

Simulation_Create_Result :: struct
{
	status:             Physics_Status,
	simd_configuration: util.Simd_Configuration,
}

Simulation :: struct
{
	shapes:              Shape_Registry,
	active_shapes:       ^Shape_Registry,
	shape_registry_ownership: Reference_State,
	bodies:              Bodies,
	statics:             Statics,
	broad_phase:         Broad_Phase,
	solver:              Solver,
	collision_tasks:     Collision_Task_Registry,
	sweep_tasks:         Sweep_Task_Registry,
	narrow_phase:        Narrow_Phase,
	integrator:          Pose_Integrator,
	sleeper:             Island_Sleeper,
	awakener:            Island_Awakener,
	profiler:            Simulation_Profiler,
	query_overlap_batcher: Query_Overlap_Batcher,
	allocation_sizes:    Simulation_Allocation_Sizes,
	solve_description:   Solve_Description,
	default_pose_context: Default_Pose_Integrator_Context,
	timestepper:         Timestepper,
	timestep_callbacks:  Timestep_Callbacks,
	simd_configuration:  util.Simd_Configuration,
	pool:                ^util.Buffer_Pool,
	step_index:          u64,
	state:               Simulation_State,
	triggers:            ^Trigger_System,
	trigger_epoch:       u64,
	restitution:         ^Restitution_Storage,
	body_control:        ^Body_Control_Storage,
}

@(private)
simulation_reset_uninitialized :: #force_inline proc "contextless" (
	simulation: ^Simulation,
)
{
	intrinsics.mem_zero(simulation, size_of(Simulation));
}

simulation_create_description_default :: proc "contextless" (
	pool: ^util.Buffer_Pool,
) -> Simulation_Create_Description
{
	return {
		allocation_sizes=simulation_allocation_sizes_default(),
		solve_description=solve_description_default(),
		default_pose_context={gravity={0, -9.81, 0}, linear_damping=0.01, angular_damping=0.01},
		use_default_narrow=.Present,
		use_default_pose=.Present,
		timestepper=default_timestepper(),
		pool=pool,
	};
}

simulation_release_partial :: proc (simulation: ^Simulation)
{
	if simulation == nil
	{
		return;
	}
	if simulation.body_control != nil
	{
		_ = body_control_destroy(simulation, .Missing);
	}
	if simulation.triggers != nil
	{
		trigger_system_destroy(simulation);
	}
	if simulation.restitution != nil
	{
		_ = restitution_storage_destroy(simulation);
	}
	if simulation.query_overlap_batcher.state == .Ready
	{
		_ = query_overlap_batcher_dispose(&simulation.query_overlap_batcher);
	}
	if simulation.sleeper.state == .Ready
	{
		_ = island_sleeper_dispose(&simulation.sleeper);
	}
	if simulation.integrator.state == .Ready
	{
		_ = pose_integrator_dispose(&simulation.integrator);
	}
	if simulation.narrow_phase.state == .Ready
	{
		_ = narrow_phase_dispose(&simulation.narrow_phase);
	}
	if simulation.solver.state == .Ready
	{
		_ = solver_dispose(&simulation.solver);
	}
	if simulation.broad_phase.state == .Ready
	{
		_ = broad_phase_dispose(&simulation.broad_phase);
	}
	if simulation.statics.state == .Allocated
	{
		_ = statics_dispose(&simulation.statics);
	}
	if simulation.bodies.state == .Allocated
	{
		_ = bodies_dispose(&simulation.bodies);
	}
	if simulation.shape_registry_ownership == .Present &&
	simulation.active_shapes != nil &&
	simulation.active_shapes.state == .Allocated
	{
		_ = shape_registry_dispose(simulation.active_shapes);
	}
}

simulation_shape_registry :: proc "contextless" (
	simulation: ^Simulation,
) -> ^Shape_Registry
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Simulation phase=shape_registry_access reason=single_active_registry_view lifetime=until_simulation_destroy
	if simulation == nil
	{
		return nil;
	}
	return simulation.active_shapes;
}

simulation_create :: proc (
	simulation: ^Simulation, description: ^Simulation_Create_Description,
) -> Simulation_Create_Result
{
	result: Simulation_Create_Result;
	if simulation == nil || description == nil
	{
		result.status = .Invalid_Description;
		return result;
	}
	sizes := description.allocation_sizes;
	if sizes.collision_child_pairs == 0
	{
		if sizes.pairs <= 0 || sizes.pairs > max(i32) / 16
		{
			result.status = .Invalid_Description;
			return result;
		}
		sizes.collision_child_pairs = sizes.pairs * 16;
	}
	normalized_sizes, sizes_status := simulation_allocation_sizes_normalize(
		sizes, description.solve_description,
	);
	if sizes_status != .Ok
	{
		result.status = sizes_status;
		return result;
	}
	sizes = normalized_sizes;
	if simulation.state != .Uninitialized || description.pool == nil ||
	description.timestepper.step == nil
	{
		result.status = .Invalid_Description;
		return result;
	}
	if description.shape_registry != nil &&
	(description.shape_registry.state != .Allocated ||
		description.shape_registry.pool != description.pool)
	{
		result.status = .Invalid_Description;
		return result;
	}
	configuration, simd_status := util.host_simd_configuration();
	result.simd_configuration = configuration;
	if simd_status != .Ok
	{
		result.status = .Invalid_Description;
		return result;
	}
	simulation_reset_uninitialized(simulation);
	simulation.allocation_sizes = sizes;
	simulation.solve_description = description.solve_description;
	simulation.default_pose_context = description.default_pose_context;
	simulation.timestepper = description.timestepper;
	simulation.timestep_callbacks = description.timestep_callbacks;
	simulation.simd_configuration = configuration;
	simulation.pool = description.pool;
	simulation.profiler.state = description.profiling;
	status := Physics_Status.Ok;
	if description.shape_registry == nil
	{
		status = shape_registry_initialize(
			&simulation.shapes, int(sizes.shapes_per_type), description.pool,
		);
		if status != .Ok
		{
			simulation_release_partial(simulation);
			simulation_reset_uninitialized(simulation);
			result.status = status;
			return result;
		}
		simulation.active_shapes = &simulation.shapes;
		simulation.shape_registry_ownership = .Present;
	}
	else
	{
		simulation.active_shapes = description.shape_registry;
	}
	shapes := simulation_shape_registry(simulation);
	status = bodies_initialize(
		&simulation.bodies, int(sizes.bodies), int(sizes.inactive_body_sets) + 1,
		int(sizes.minimum_constraints_per_body), description.pool,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = bodies_bind_shape_registry(&simulation.bodies, shapes);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = statics_initialize(&simulation.statics, int(sizes.statics), description.pool);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = statics_bind_shape_registry(&simulation.statics, shapes);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = solver_initialize(
		&simulation.solver, &simulation.bodies, int(sizes.constraints), int(sizes.constraint_batches),
		int(description.solve_description.fallback_batch_threshold), int(sizes.initial_constraints_per_type_batch), description.pool,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = broad_phase_initialize(
		&simulation.broad_phase, int(sizes.bodies), int(sizes.statics + sizes.bodies),
		int(sizes.broad_phase_candidates), int(sizes.workers), description.pool,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = broad_phase_bind_owners(
		&simulation.broad_phase, shapes, &simulation.bodies, &simulation.statics,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = collision_task_registry_initialize(&simulation.collision_tasks);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = sweep_task_registry_initialize(&simulation.sweep_tasks);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	narrow_callbacks := description.narrow_callbacks;
	if description.use_default_narrow == .Present
	{
		narrow_callbacks = narrow_phase_default_callbacks();
	}
	status = narrow_phase_initialize(
		&simulation.narrow_phase, &simulation.broad_phase, &simulation.bodies, &simulation.statics,
		shapes, &simulation.solver, &simulation.collision_tasks, &simulation.sweep_tasks, narrow_callbacks,
		int(sizes.broad_phase_candidates), int(sizes.pairs), int(sizes.constraints),
		int(sizes.inactive_pairs), int(sizes.workers),
		int(sizes.pending_pairs_per_worker), int(sizes.collision_child_pairs), description.pool,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	pose_callbacks := description.pose_callbacks;
	if description.use_default_pose == .Present
	{
		pose_callbacks = pose_integrator_default_callbacks(&simulation.default_pose_context);
	}
	status = pose_integrator_initialize(&simulation.integrator, &simulation.bodies, pose_callbacks);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = solver_bind_integrator(&simulation.solver, &simulation.integrator);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = island_sleeper_initialize(
		&simulation.sleeper, &simulation.bodies, &simulation.broad_phase, &simulation.solver,
		&simulation.narrow_phase.pair_cache, int(sizes.bodies), int(sizes.constraints), int(sizes.workers),
		description.pool,
	);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = island_awakener_initialize(&simulation.awakener, &simulation.sleeper);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = narrow_phase_bind_awakener(&simulation.narrow_phase, &simulation.awakener);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	simulation.state = .Ready;
	status = pose_integrator_activate(&simulation.integrator, simulation);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	status = narrow_phase_activate(&simulation.narrow_phase, simulation);
	if status != .Ok
	{
		simulation_release_partial(simulation);
		simulation_reset_uninitialized(simulation);
		result.status = status;
		return result;
	}
	simulation.shape_registry_ownership = .Present;
	result.status = .Ok;
	return result;
}

Simulation_Timestep_Session_Request :: struct
{
	simulation: ^Simulation,
	dt:         f32,
	status:     Physics_Status,
}

simulation_timestep_session_worker :: proc "contextless" (
	worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
)
{
	_ = worker_index;
	request := (^Simulation_Timestep_Session_Request)(dispatcher.unmanaged_context);
	dispatcher.unmanaged_context = nil;
	request.status = default_timestepper_step(
		nil, request.simulation, request.dt, dispatcher,
	);
}

simulation_timestep :: proc (
	simulation: ^Simulation, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready || simulation.timestepper.step == nil
	{
		return .Disposed;
	}
	if !(dt > 0)
	{
		return .Invalid_Argument;
	}
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if simulation.solver.joint_breaks != nil && simulation.solver.joint_breaks.phase != .Idle
	{
		return .Invalid_Argument;
	}
	if simulation.triggers != nil
	{
		status = trigger_step_prepare(simulation.triggers);
		if status != .Ok
		{
			return status;
		}
	}
	if simulation.body_control != nil
	{
		status = body_control_step_prepare(simulation.body_control, dt);
		if status != .Ok
		{
			if simulation.triggers != nil
			{
				trigger_step_complete(simulation.triggers, status);
			}
			return status;
		}
	}
	if simulation.solver.joint_breaks != nil
	{
		status = joint_break_step_prepare(simulation.solver.joint_breaks);
		if status != .Ok
		{
			if simulation.body_control != nil
			{
				body_control_step_complete(simulation.body_control, status);
			}
			if simulation.triggers != nil
			{
				trigger_step_complete(simulation.triggers, status);
			}
			return status;
		}
	}
	simulation_profiler_begin_step(&simulation.profiler);
	simulation_profiler_start(&simulation.profiler, .Timestep);
	simulation.state = .Stepping;
	if simulation.timestepper.step == default_timestepper_step &&
	simulation.timestepper.user_context == nil && dispatcher != nil &&
	dispatcher.worker_count >= util.THREAD_DISPATCHER_SESSION_MINIMUM_WORKERS &&
	dispatcher.worker_count <= util.THREAD_DISPATCHER_SESSION_MAXIMUM_WORKERS &&
	dispatcher.dispatcher != nil &&
	dispatcher.dispatch == util.thread_dispatcher_dispatch_boundary &&
	dispatcher.worker_pool == util.thread_dispatcher_worker_pool_boundary
	{
		included := (^util.Thread_Dispatcher)(dispatcher.dispatcher);
		if dispatcher == &included.boundary &&
		included.background_count + 1 == dispatcher.worker_count
		{
			request := Simulation_Timestep_Session_Request{
				simulation=simulation,
				dt=dt,
				status=.Ok,
			};
			session_status := util.thread_dispatcher_session_entry(
				dispatcher, simulation_timestep_session_worker,
				dispatcher.worker_count, &request,
			);
			if session_status == .Ok
			{
				status = request.status;
			}
			else
			{
				status = .Invalid_Argument;
			}
		}
		else
		{
			status = simulation.timestepper.step(
				simulation.timestepper.user_context, simulation, dt, dispatcher,
			);
		}
	}
	else
	{
		status = simulation.timestepper.step(
			simulation.timestepper.user_context, simulation, dt, dispatcher,
		);
	}
	// reject malformed custom completion before any optional feature commits history
	if status == .Ok && simulation.solver.joint_breaks != nil &&
	simulation.step_index != simulation.solver.joint_breaks.prepared_step + 1
	{
		status = .Invalid_Argument;
	}
	if simulation.body_control != nil
	{
		body_control_step_complete(simulation.body_control, status);
	}
	if simulation.restitution != nil
	{
		restitution_step_complete(simulation.restitution, status);
	}
	if simulation.triggers != nil
	{
		trigger_step_complete(simulation.triggers, status);
	}
	simulation_profiler_end(&simulation.profiler, .Timestep);
	simulation.state = .Ready;
	if simulation.solver.joint_breaks != nil
	{
		joint_break_step_complete(simulation.solver.joint_breaks, status);
	}
	return status;
}

simulation_stage_validate :: proc (
	simulation: ^Simulation, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	if simulation == nil ||
	(simulation.state != .Ready && simulation.state != .Stepping)
	{
		return .Disposed;
	}
	if dispatcher != nil &&
	(dispatcher.dispatch == nil || dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > int(simulation.allocation_sizes.workers))
	{
		return simulation_stage_validate_worker_capacity_cold(
			simulation, dispatcher,
		);
	}
	return .Ok;
}

simulation_sleep :: proc (
	simulation: ^Simulation, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	simulation_profiler_start(&simulation.profiler, .Sleep);
	status = island_sleeper_update(&simulation.sleeper, dispatcher);
	simulation_profiler_end(&simulation.profiler, .Sleep);
	return status;
}

simulation_predict_bounding_boxes :: proc (
	simulation: ^Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if dt <= 0
	{
		return .Invalid_Argument;
	}
	standalone_controls: ^Body_Control_Storage;
	if simulation.state == .Ready && simulation.body_control != nil
	{
		standalone_controls = simulation.body_control;
		status = body_control_step_prepare(standalone_controls, dt);
		if status != .Ok
		{
			return status;
		}
		simulation.state = .Stepping;
	}
	simulation_profiler_start(&simulation.profiler, .Predict_Bounding_Boxes);
	status = pose_integrator_prepare(&simulation.integrator, dt);
	if status == .Ok
	{
		status = broad_phase_predict_bounds(
			&simulation.broad_phase, dt, &simulation.integrator, dispatcher,
		);
	}
	simulation_profiler_end(&simulation.profiler, .Predict_Bounding_Boxes);
	if standalone_controls != nil
	{
		body_control_step_complete(standalone_controls, status);
		simulation.state = .Ready;
	}
	return status;
}

simulation_collision_detection :: proc (
	simulation: ^Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if dt <= 0
	{
		return .Invalid_Argument;
	}
	standalone_controls: ^Body_Control_Storage;
	if simulation.state == .Ready && simulation.body_control != nil
	{
		standalone_controls = simulation.body_control;
		status = body_control_step_prepare(standalone_controls, dt);
		if status != .Ok
		{
			return status;
		}
		simulation.state = .Stepping;
	}
	defer
	{
		if standalone_controls != nil
		{
			if status != .Ok
			{
				body_control_step_complete(standalone_controls, status);
			}
			simulation.state = .Ready;
		}
	}
	simulation_profiler_start(&simulation.profiler, .Broad_Phase);
	status = broad_phase_update(&simulation.broad_phase, dispatcher);
	simulation_profiler_end(&simulation.profiler, .Broad_Phase);
	if status != .Ok
	{
		return status;
	}
	simulation_profiler_start(&simulation.profiler, .Collision_Detection);
	if simulation.triggers == nil
	{
		status = narrow_phase_execute(
			&simulation.narrow_phase, dispatcher, dt,
		);
	}
	else
	{
		status = trigger_collision_stage(simulation.triggers, dispatcher, dt);
	}
	simulation_profiler_end(&simulation.profiler, .Collision_Detection);
	return status;
}

simulation_solve :: proc (
	simulation: ^Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	if dt <= 0
	{
		return .Invalid_Argument;
	}
	standalone_controls: Reference_State = .Missing;
	if simulation.body_control != nil
	{
		status = body_control_solve_begin(simulation.body_control, dt);
		if status != .Ok
		{
			return status;
		}
		if simulation.state == .Ready
		{
			standalone_controls = .Present;
			simulation.state = .Stepping;
		}
	}
	simulation_profiler_start(&simulation.profiler, .Solve);
	substep_dt := dt / f32(simulation.solve_description.substep_count);
	status = pose_integrator_prepare(&simulation.integrator, substep_dt);
	if status == .Ok
	{
		if simulation.restitution == nil
		{
			status = solver_solve_and_integrate(
				&simulation.solver, dt, &simulation.solve_description, dispatcher,
			);
		}
		else
		{
			status = restitution_solve(simulation.restitution, dt, dispatcher);
		}
	}
	if simulation.body_control != nil
	{
		if standalone_controls == .Present || status != .Ok
		{
			body_control_step_complete(simulation.body_control, status);
		}
		else
		{
			simulation.body_control.phase = .Preview;
		}
	}
	if standalone_controls == .Present
	{
		// restitution deferred successful history publication while body controls
		// temporarily owned the Stepping state for this standalone solve
		if status == .Ok && simulation.restitution != nil
		{
			restitution_step_complete(simulation.restitution, status);
		}
		simulation.state = .Ready;
	}
	simulation_profiler_end(&simulation.profiler, .Solve);
	return status;
}

simulation_incrementally_optimize_data_structures :: proc (
	simulation: ^Simulation, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	status := simulation_stage_validate(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	simulation_profiler_start(&simulation.profiler, .Incrementally_Optimize);
	status = solver_compress(&simulation.solver, dispatcher);
	simulation_profiler_end(&simulation.profiler, .Incrementally_Optimize);
	return status;
}

simulation_add_body :: proc (
	simulation: ^Simulation, description: ^Body_Description,
) -> (Body_Handle, Physics_Status)
{
	if simulation == nil || simulation.state != .Ready
	{
		return body_handle_invalid(), .Disposed;
	}
	handle, status := bodies_add(&simulation.bodies, description);
	if status != .Ok
	{
		return handle, status;
	}
	if typed_index_state(description.collidable.shape) == .Present
	{
		status = broad_phase_add_body(&simulation.broad_phase, handle);
		if status != .Ok
		{
			_ = bodies_remove(&simulation.bodies, handle);
			return body_handle_invalid(), status;
		}
	}
	return handle, .Ok;
}

simulation_add_static_internal :: proc (
	simulation: ^Simulation, description: ^Static_Description,
	awaken_bodies: bool,
) -> (Static_Handle, Physics_Status)
{
	if simulation == nil || simulation.state != .Ready
	{
		return static_handle_invalid(), .Disposed;
	}
	if static_description_validate(description) != .Ok
	{
		return static_handle_invalid(), .Invalid_Description;
	}
	if awaken_bodies
	{
		bounds, bounds_status := shape_registry_compute_world_bounds(
			simulation_shape_registry(simulation),
			description.shape, description.pose,
		);
		if bounds_status != .Ok
		{
			return static_handle_invalid(), bounds_status;
		}
		wake_status := simulation_awaken_inactive_bodies_in_bounds(
			simulation, bounds,
		);
		if wake_status != .Ok
		{
			return static_handle_invalid(), wake_status;
		}
	}
	handle, status := statics_add(&simulation.statics, description);
	if status != .Ok
	{
		return handle, status;
	}
	status = broad_phase_add_static(&simulation.broad_phase, handle);
	if status != .Ok
	{
		_ = statics_remove(&simulation.statics, handle);
		return static_handle_invalid(), status;
	}
	return handle, .Ok;
}

simulation_add_static :: proc (
	simulation: ^Simulation, description: ^Static_Description,
) -> (Static_Handle, Physics_Status)
{
	return simulation_add_static_internal(simulation, description, true);
}

simulation_add_static_without_awakening_bodies :: proc (
	simulation: ^Simulation, description: ^Static_Description,
) -> (Static_Handle, Physics_Status)
{
	return simulation_add_static_internal(simulation, description, false);
}

Static_Awakening_Filter_Proc :: #type proc "contextless" (
	user_context: rawptr, body: Body_Handle,
) -> bool;
simulation_awaken_inactive_bodies_in_bounds_filtered :: proc (
	simulation: ^Simulation, bounds: util.Bounding_Box,
	filter: Static_Awakening_Filter_Proc, user_context: rawptr,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	leaves := simulation.broad_phase.leaf_scratch;
	leaf_count, status := tree_query_overlaps(&simulation.broad_phase.static_tree, bounds, leaves);
	if status != .Ok
	{
		return status;
	}
	set_count := 0;
	for candidate_index in 0 ..< leaf_count
	{
		leaf_index := int(leaves.memory[candidate_index]);
		if leaf_index < 0 || leaf_index >= simulation.broad_phase.static_tree.leaf_count
		{
			return .Invalid_Description;
		}
		reference := simulation.broad_phase.static_leaves.memory[leaf_index];
		if collidable_reference_mobility(reference) != .Dynamic
		{
			continue;
		}
		handle := Body_Handle{collidable_reference_raw_handle(reference)};
		if filter != nil && !filter(user_context, handle)
		{
			continue;
		}
		location, resolve_status := bodies_resolve(&simulation.bodies, handle);
		if resolve_status != .Ok
		{
			return resolve_status;
		}
		if location.set_index <= BODIES_ACTIVE_SET_INDEX
		{
			continue;
		}
		leaves.memory[set_count] = location.set_index;
		set_count += 1;
	}
	if set_count == 0
	{
		return .Ok;
	}
	if set_count > 1
	{
		sort_status := util.quick_sort_keys(leaves.memory, 0, set_count - 1, util.primitive_i32_comparer());
		if sort_status != .Ok
		{
			return .Invalid_Description;
		}
	}
	previous_set := i32(-1);
	for candidate_index in 0 ..< set_count
	{
		set_index := leaves.memory[candidate_index];
		if set_index == previous_set
		{
			continue;
		}
		status = island_awakener_awaken_set(&simulation.awakener, int(set_index));
		if status != .Ok
		{
			return status;
		}
		previous_set = set_index;
	}
	return .Ok;
}

simulation_awaken_inactive_bodies_in_bounds :: proc (
	simulation: ^Simulation, bounds: util.Bounding_Box,
) -> Physics_Status
{
	return simulation_awaken_inactive_bodies_in_bounds_filtered(
		simulation, bounds, nil, nil,
	);
}

simulation_forget_pair_constraint :: proc (
	simulation: ^Simulation, handle: Constraint_Handle,
) -> Physics_Status
{
	cache := &simulation.narrow_phase.pair_cache;
	if handle.value < 0 || int(handle.value) >= int(cache.constraint_handle_to_pair.length)
	{
		return .Ok;
	}
	pair := cache.constraint_handle_to_pair.memory[handle.value].pair;
	index := pair_cache_index_of(cache, pair);
	if index < 0
	{
		return .Ok;
	}
	pair_cache_unbind_constraint(cache, handle);
	return physics_collection_status(util.quick_dictionary_fast_remove(&cache.mapping, &pair));
}

simulation_remove_body :: proc (simulation: ^Simulation, handle: Body_Handle) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	status := island_awakener_awaken_body(&simulation.awakener, handle);
	if status != .Ok
	{
		return status;
	}
	location, resolve_status := bodies_resolve(&simulation.bodies, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	constraints := &simulation.bodies.sets.memory[location.set_index].constraints.memory[location.index];
	for constraints.count > 0
	{
		constraint := constraints.span.memory[constraints.count - 1].connecting_constraint_handle;
		status = simulation_forget_pair_constraint(simulation, constraint);
		if status != .Ok
		{
			return status;
		}
		status = solver_remove(&simulation.solver, constraint);
		if status != .Ok
		{
			return status;
		}
	}
	if typed_index_state(simulation.bodies.sets.memory[location.set_index].collidables.memory[location.index].shape) == .Present
	{
		status = broad_phase_remove_body(&simulation.broad_phase, handle);
		if status != .Ok
		{
			return status;
		}
	}
	return bodies_remove(&simulation.bodies, handle);
}

simulation_remove_static_internal :: proc (
	simulation: ^Simulation, handle: Static_Handle, awaken_bodies: bool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if awaken_bodies && (simulation.triggers==nil || trigger_static_member(simulation.triggers, handle) == .Missing)
	{
		static_description, description_status := statics_get_description(
			&simulation.statics, handle,
		);
		if description_status != .Ok
		{
			return description_status;
		}
		bounds, bounds_status := shape_registry_compute_world_bounds(
			simulation_shape_registry(simulation),
			static_description.shape, static_description.pose,
		);
		if bounds_status != .Ok
		{
			return bounds_status;
		}
		status := mixed_colliders_awaken_static(
			simulation, handle, &static_description, &static_description, bounds, nil, nil,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status := broad_phase_remove_static(&simulation.broad_phase, handle);
	if status != .Ok
	{
		return status;
	}
	return statics_remove(&simulation.statics, handle);
}

simulation_remove_static :: proc (
	simulation: ^Simulation, handle: Static_Handle,
) -> Physics_Status
{
	return simulation_remove_static_internal(simulation, handle, true);
}

simulation_remove_static_without_awakening_bodies :: proc (
	simulation: ^Simulation, handle: Static_Handle,
) -> Physics_Status
{
	return simulation_remove_static_internal(simulation, handle, false);
}

simulation_apply_body_description :: proc (
	simulation: ^Simulation, handle: Body_Handle, description: ^Body_Description,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	status := island_awakener_awaken_body(&simulation.awakener, handle);
	if status != .Ok
	{
		return status;
	}
	broad_phase_change, broad_phase_status :=
	broad_phase_prepare_body_description_change(
		&simulation.broad_phase, handle, description,
	);
	if broad_phase_status != .Ok
	{
		return broad_phase_status;
	}
	status = bodies_apply_description(&simulation.bodies, handle, description);
	if status != .Ok
	{
		return status;
	}
	broad_phase_commit_body_description_change(
		&simulation.broad_phase, &broad_phase_change,
	);
	return .Ok;
}

simulation_apply_static_description_internal :: proc (
	simulation: ^Simulation, handle: Static_Handle,
	description: ^Static_Description, awaken_bodies: bool,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if static_description_validate(description) != .Ok
	{
		return .Invalid_Description;
	}
	old_description, old_status := statics_get_description(
		&simulation.statics, handle,
	);
	if old_status != .Ok
	{
		return old_status;
	}
	if awaken_bodies && (simulation.triggers==nil || trigger_static_member(simulation.triggers, handle) == .Missing)
	{
		old_bounds, old_bounds_status := shape_registry_compute_world_bounds(
			simulation_shape_registry(simulation),
			old_description.shape, old_description.pose,
		);
		if old_bounds_status != .Ok
		{
			return old_bounds_status;
		}
		new_bounds, new_bounds_status := shape_registry_compute_world_bounds(
			simulation_shape_registry(simulation),
			description.shape, description.pose,
		);
		if new_bounds_status != .Ok
		{
			return new_bounds_status;
		}
		status := mixed_colliders_awaken_static(
			simulation, handle, &old_description, description,
			util.bounding_box_merge(old_bounds, new_bounds), nil, nil,
		);
		if status != .Ok
		{
			return status;
		}
	}
	status := broad_phase_remove_static(&simulation.broad_phase, handle);
	if status != .Ok
	{
		return status;
	}
	status = statics_apply_description(&simulation.statics, handle, description);
	if status != .Ok
	{
		_ = broad_phase_add_static(&simulation.broad_phase, handle);
		return status;
	}
	status = broad_phase_add_static(&simulation.broad_phase, handle);
	if status != .Ok
	{
		_ = statics_apply_description(
			&simulation.statics, handle, &old_description,
		);
		_ = broad_phase_add_static(&simulation.broad_phase, handle);
		return status;
	}
	return .Ok;
}

simulation_apply_static_description :: proc (
	simulation: ^Simulation, handle: Static_Handle,
	description: ^Static_Description,
) -> Physics_Status
{
	return simulation_apply_static_description_internal(
		simulation, handle, description, true,
	);
}

simulation_apply_static_description_without_awakening_bodies :: proc (
	simulation: ^Simulation, handle: Static_Handle,
	description: ^Static_Description,
) -> Physics_Status
{
	return simulation_apply_static_description_internal(
		simulation, handle, description, false,
	);
}

simulation_set_static_shape :: proc (
	simulation: ^Simulation, handle: Static_Handle, shape: Typed_Index,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	description, status := statics_get_description(&simulation.statics, handle);
	if status != .Ok
	{
		return status;
	}
	description.shape = shape;
	return simulation_apply_static_description_internal(
		simulation, handle, &description, true,
	);
}

simulation_set_static_shape_without_awakening_bodies :: proc (
	simulation: ^Simulation, handle: Static_Handle, shape: Typed_Index,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	description, status := statics_get_description(&simulation.statics, handle);
	if status != .Ok
	{
		return status;
	}
	description.shape = shape;
	return simulation_apply_static_description_internal(
		simulation, handle, &description, false,
	);
}

simulation_add_constraint :: proc (
	simulation: ^Simulation, body_handles: ^[4]Body_Handle, description: ^$T,
) -> (Constraint_Handle, Physics_Status)
{
	if simulation == nil || simulation.state != .Ready
	{
		return constraint_handle_invalid(), .Disposed;
	}
	if body_handles == nil || description == nil
	{
		return constraint_handle_invalid(), .Invalid_Argument;
	}
	type_id := constraint_description_type_id(T);
	body_count := int(constraint_description_body_count(T));
	_, type_status := constraint_type_registry_lookup(&simulation.solver.registry, type_id);
	if type_status != .Ok
	{
		return constraint_handle_invalid(), type_status;
	}
	for body_index in 0 ..< body_count
	{
		status := island_awakener_awaken_body(&simulation.awakener, body_handles[body_index]);
		if status != .Ok
		{
			return constraint_handle_invalid(), status;
		}
	}
	return solver_add(&simulation.solver, body_handles, description);
}

simulation_get_constraint_description_raw :: proc "contextless" (
	simulation: ^Simulation, handle: Constraint_Handle, type_id: i32,
	target: rawptr, target_size: int,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if target == nil
	{
		return .Invalid_Argument;
	}
	if handle.value < 0 ||
	int(handle.value) >= int(simulation.solver.handle_to_constraint.length)
	{
		return .Not_Found;
	}
	location := simulation.solver.handle_to_constraint.memory[handle.value];
	if location.set_index == 0
	{
		return solver_get_description_raw(
			&simulation.solver, handle, type_id, target, target_size,
		);
	}
	record_index, resolve_status :=
	island_sleeper_resolve_inactive_constraint_index(
		&simulation.sleeper, handle,
	);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	record := &simulation.sleeper.inactive_constraints.memory[record_index];
	if record.type_id != type_id
	{
		return .Invalid_Argument;
	}
	type_record, lookup_status := constraint_type_registry_lookup(
		&simulation.solver.registry, type_id,
	);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if target_size != int(type_record.description_size) ||
	target_size != int(record.description_size)
	{
		return .Invalid_Argument;
	}
	target_bytes := ([^]u8)(target);
	for index in 0 ..< target_size
	{
		target_bytes[index] = record.description[index];
	}
	return .Ok;
}

simulation_get_constraint_description :: proc "contextless" (
	simulation: ^Simulation, handle: Constraint_Handle, target: ^$T,
) -> Physics_Status
{
	return simulation_get_constraint_description_raw(
		simulation, handle, constraint_description_type_id(T),
		target, size_of(T),
	);
}

simulation_apply_constraint_description_raw :: proc (
	simulation: ^Simulation, handle: Constraint_Handle, type_id: i32,
	description: rawptr,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	if description == nil
	{
		return .Invalid_Argument;
	}
	if handle.value < 0 ||
	int(handle.value) >= int(simulation.solver.handle_to_constraint.length)
	{
		return .Not_Found;
	}
	location := simulation.solver.handle_to_constraint.memory[handle.value];
	if location.set_index == 0
	{
		return solver_apply_description_raw(
			&simulation.solver, handle, type_id, description,
		);
	}
	record_index, resolve_status :=
	island_sleeper_resolve_inactive_constraint_index(
		&simulation.sleeper, handle,
	);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	record := &simulation.sleeper.inactive_constraints.memory[record_index];
	if record.type_id != type_id
	{
		return .Invalid_Argument;
	}
	type_record, lookup_status := constraint_type_registry_lookup(
		&simulation.solver.registry, type_id,
	);
	if lookup_status != .Ok
	{
		return lookup_status;
	}
	if int(record.description_size) != int(type_record.description_size)
	{
		return .Invalid_Description;
	}
	validation_status: Physics_Status = constraint_type_validate_description(type_record,
		type_id, description,
	);
	if validation_status != .Ok
	{
		return validation_status;
	}
	awaken_status := island_awakener_awaken_set(
		&simulation.awakener, int(location.set_index),
	);
	if awaken_status != .Ok
	{
		return awaken_status;
	}
	return solver_apply_description_raw(
		&simulation.solver, handle, type_id, description,
	);
}

simulation_apply_constraint_description :: proc (
	simulation: ^Simulation, handle: Constraint_Handle, description: ^$T,
) -> Physics_Status
{
	return simulation_apply_constraint_description_raw(
		simulation, handle, constraint_description_type_id(T), description,
	);
}

simulation_remove_constraint :: proc (simulation: ^Simulation, handle: Constraint_Handle) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	_, resolve_status := solver_resolve(&simulation.solver, handle);
	if resolve_status == .Not_Found
	{
		if handle.value < 0 ||
		int(handle.value) >= int(simulation.solver.handle_to_constraint.length)
		{
			return .Not_Found;
		}
		location := simulation.solver.handle_to_constraint.memory[handle.value];
		if location.set_index <= 0
		{
			return .Not_Found;
		}
		_, inactive_status :=
		island_sleeper_resolve_inactive_constraint_index(
			&simulation.sleeper, handle,
		);
		if inactive_status != .Ok
		{
			return inactive_status;
		}
		status := island_awakener_awaken_set(
			&simulation.awakener, int(location.set_index),
		);
		if status != .Ok
		{
			return status;
		}
	}
	else if resolve_status != .Ok
	{
		return resolve_status;
	}
	status := simulation_forget_pair_constraint(simulation, handle);
	if status != .Ok
	{
		return status;
	}
	return solver_remove(&simulation.solver, handle);
}

simulation_clear :: proc (simulation: ^Simulation) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	status := solver_clear(&simulation.solver);
	if status != .Ok
	{
		return status;
	}
	status = bodies_clear(&simulation.bodies);
	if status != .Ok
	{
		return status;
	}
	status = statics_clear(&simulation.statics);
	if status != .Ok
	{
		return status;
	}
	status = shape_registry_clear(simulation_shape_registry(simulation));
	if status != .Ok
	{
		return status;
	}
	status = broad_phase_clear(&simulation.broad_phase);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_clear(&simulation.narrow_phase);
	if status != .Ok
	{
		return status;
	}
	status = island_sleeper_clear(&simulation.sleeper);
	if status != .Ok
	{
		return status;
	}
	simulation.step_index = 0;
	return .Ok;
}

simulation_capacity_target_normalize :: proc "contextless" (
	simulation: ^Simulation, target: Simulation_Allocation_Sizes,
) -> (Simulation_Allocation_Sizes, Physics_Status)
{
	if simulation == nil
	{
		return {}, .Invalid_Description;
	}
	normalized_target := target;
	normalized_target.workers = max(
		normalized_target.workers, simulation.allocation_sizes.workers,
	);
	if normalized_target.collision_child_pairs == 0
	{
		normalized_target.collision_child_pairs = max(
			simulation.allocation_sizes.collision_child_pairs,
			normalized_target.workers,
		);
	}
	normalized_target.collision_child_pairs = max(
		normalized_target.collision_child_pairs, normalized_target.workers,
	);
	return simulation_allocation_sizes_normalize(
		normalized_target, simulation.solve_description,
	);
}

simulation_ensure_capacity :: proc (
	simulation: ^Simulation, target: Simulation_Allocation_Sizes,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	normalized_target, target_status := simulation_capacity_target_normalize(
		simulation, target,
	);
	if target_status != .Ok
	{
		return target_status;
	}
	status := simulation_ensure_worker_capacity(
		simulation, int(normalized_target.workers),
	);
	if status != .Ok
	{
		return status;
	}
	status = bodies_ensure_capacity(
		&simulation.bodies, int(normalized_target.bodies),
		int(normalized_target.inactive_body_sets) + 1,
		int(normalized_target.minimum_constraints_per_body),
	);
	if status != .Ok
	{
		return status;
	}
	status = solver_ensure_capacity(
		&simulation.solver, int(normalized_target.bodies),
		int(normalized_target.constraints),
		int(normalized_target.initial_constraints_per_type_batch),
	);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_ensure_capacity(
		&simulation.narrow_phase,
		int(normalized_target.broad_phase_candidates),
		int(normalized_target.pairs),
		int(normalized_target.constraints),
		int(normalized_target.inactive_pairs),
		int(normalized_target.pending_pairs_per_worker),
		int(normalized_target.collision_child_pairs),
	);
	if status != .Ok
	{
		return status;
	}
	status = island_sleeper_ensure_capacity(
		&simulation.sleeper, int(normalized_target.bodies),
		int(normalized_target.constraints),
	);
	if status != .Ok
	{
		return status;
	}
	status = statics_ensure_capacity(
		&simulation.statics, int(normalized_target.statics),
	);
	if status != .Ok
	{
		return status;
	}
	status = shape_registry_ensure_capacity(
		simulation_shape_registry(simulation),
		int(normalized_target.shapes_per_type),
	);
	if status != .Ok
	{
		return status;
	}
	status = broad_phase_ensure_capacity(
		&simulation.broad_phase, int(normalized_target.bodies),
		int(normalized_target.bodies + normalized_target.statics),
		int(normalized_target.broad_phase_candidates),
	);
	if status != .Ok
	{
		return status;
	}
	simulation.allocation_sizes = {
		bodies=max(simulation.allocation_sizes.bodies, normalized_target.bodies),
		statics=max(simulation.allocation_sizes.statics, normalized_target.statics),
		inactive_body_sets=max(
			simulation.allocation_sizes.inactive_body_sets,
			normalized_target.inactive_body_sets,
		),
		shapes_per_type=max(
			simulation.allocation_sizes.shapes_per_type,
			normalized_target.shapes_per_type,
		),
		constraints=max(
			simulation.allocation_sizes.constraints,
			normalized_target.constraints,
		),
		constraint_batches=normalized_target.constraint_batches,
		initial_constraints_per_type_batch=max(
			simulation.allocation_sizes.initial_constraints_per_type_batch,
			normalized_target.initial_constraints_per_type_batch,
		),
		minimum_constraints_per_body=max(
			simulation.allocation_sizes.minimum_constraints_per_body,
			normalized_target.minimum_constraints_per_body,
		),
		broad_phase_candidates=max(
			simulation.allocation_sizes.broad_phase_candidates,
			normalized_target.broad_phase_candidates,
		),
		pairs=max(simulation.allocation_sizes.pairs, normalized_target.pairs),
		collision_child_pairs=max(
			simulation.allocation_sizes.collision_child_pairs,
			normalized_target.collision_child_pairs,
		),
		inactive_pairs=max(
			simulation.allocation_sizes.inactive_pairs,
			normalized_target.inactive_pairs,
		),
		pending_pairs_per_worker=max(
			simulation.allocation_sizes.pending_pairs_per_worker,
			normalized_target.pending_pairs_per_worker,
		),
		workers=max(
			simulation.allocation_sizes.workers, normalized_target.workers,
		),
	};
	return .Ok;
}

simulation_resize :: proc (
	simulation: ^Simulation, target: Simulation_Allocation_Sizes,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	normalized_target, target_status := simulation_capacity_target_normalize(
		simulation, target,
	);
	if target_status != .Ok
	{
		return target_status;
	}
	status := simulation_ensure_capacity(simulation, normalized_target);
	if status != .Ok
	{
		return status;
	}
	status = bodies_resize(
		&simulation.bodies, int(normalized_target.bodies),
		int(normalized_target.inactive_body_sets) + 1,
		int(normalized_target.minimum_constraints_per_body),
	);
	if status != .Ok
	{
		return status;
	}
	status = solver_resize(
		&simulation.solver, int(normalized_target.bodies),
		int(normalized_target.constraints),
		int(normalized_target.initial_constraints_per_type_batch),
	);
	if status != .Ok
	{
		return status;
	}
	status = narrow_phase_resize(
		&simulation.narrow_phase,
		int(normalized_target.broad_phase_candidates),
		int(normalized_target.pairs),
		int(normalized_target.constraints),
		int(normalized_target.inactive_pairs),
		int(normalized_target.pending_pairs_per_worker),
		int(normalized_target.collision_child_pairs),
	);
	if status != .Ok
	{
		return status;
	}
	status = island_sleeper_resize(
		&simulation.sleeper, int(normalized_target.bodies),
		int(normalized_target.constraints),
	);
	if status != .Ok
	{
		return status;
	}
	status = statics_resize(&simulation.statics, int(normalized_target.statics));
	if status != .Ok
	{
		return status;
	}
	status = shape_registry_resize(
		simulation_shape_registry(simulation),
		int(normalized_target.shapes_per_type),
	);
	if status != .Ok
	{
		return status;
	}
	status = broad_phase_resize(
		&simulation.broad_phase, int(normalized_target.bodies),
		int(normalized_target.bodies + normalized_target.statics),
		int(normalized_target.broad_phase_candidates),
	);
	if status != .Ok
	{
		return status;
	}
	simulation.allocation_sizes = normalized_target;
	return .Ok;
}

simulation_destroy :: proc (simulation: ^Simulation) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	simulation_release_partial(simulation);
	simulation_reset_uninitialized(simulation);
	simulation.state = .Disposed;
	return .Ok;
}
