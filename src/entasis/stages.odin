package entasis

import physics "entasis:entasis_physics"

// Timestep_Completion_Stage identifies stable completion points in the default
// timestepper
Timestep_Completion_Stage :: physics.Timestep_Completion_Stage;
// Stage_Completed_Proc is an allocation-free contextless stage callback
Stage_Completed_Proc :: physics.Simulation_Stage_Completed_Proc;
// Timestep_Callbacks is the exact low-level stage callback table
Timestep_Callbacks :: physics.Timestep_Callbacks;
// Timestep_Proc is the advanced complete-timestep callback ABI
Timestep_Proc :: physics.Timestep_Proc;
// Timestepper selects a caller-owned complete timestep implementation
Timestepper :: physics.Timestepper;

// timestep_callbacks constructs the stable stage-completion callback table
timestep_callbacks :: #force_inline proc "contextless" (
	completed: Stage_Completed_Proc,
	user_context: rawptr = nil,
) -> Timestep_Callbacks
{
	return {stage_completed=completed, user_context=user_context};
}

// timestepper constructs an advanced complete-timestep policy
timestepper :: #force_inline proc "contextless" (
	step: Timestep_Proc,
	user_context: rawptr = nil,
) -> Timestepper
{
	return {step=step, user_context=user_context};
}

// world_description_set_timestep installs optional caller-owned stage callbacks
// and a custom complete timestepper. a zero custom timestepper keeps the
// default stage order
world_description_set_timestep :: proc "contextless" (
	description: ^World_Description,
	callbacks: Timestep_Callbacks = {},
	custom: Timestepper = {},
) -> Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	description.timestep_callbacks = callbacks;
	description.timestepper = custom;
	return .Ok;
}

@(private)
world_stage_ready :: #force_inline proc "contextless" (
	world: ^World, dispatcher: ^Dispatcher,
) -> (^world_data, ^Dispatcher, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, .Invalid_Argument;
	}
	selected := dispatcher;
	if selected == nil
	{
		selected = data.dispatcher;
	}
	return data, selected, .Ok;
}

// world_stage_sleep executes only sleeping/island migration
world_stage_sleep :: #force_inline proc (
	world: ^World, dispatcher: ^Dispatcher = nil,
) -> Status
{
	data, selected, status := world_stage_ready(world, dispatcher);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_sleep(&data.simulation, selected);
}

// world_stage_predict_bounds executes pose-policy preparation and predicted
// broad-phase leaf bounds for one timestep duration
world_stage_predict_bounds :: #force_inline proc (
	world: ^World, dt: f32, dispatcher: ^Dispatcher = nil,
) -> Status
{
	data, selected, status := world_stage_ready(world, dispatcher);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_predict_bounding_boxes(&data.simulation, dt, selected);
}

// world_stage_collision_detection updates the broad phase and narrow phase
world_stage_collision_detection :: #force_inline proc (
	world: ^World, dt: f32, dispatcher: ^Dispatcher = nil,
) -> Status
{
	data, selected, status := world_stage_ready(world, dispatcher);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_collision_detection(&data.simulation, dt, selected);
}

// world_stage_solve solves constraints and integrates bodies
world_stage_solve :: #force_inline proc (
	world: ^World, dt: f32, dispatcher: ^Dispatcher = nil,
) -> Status
{
	data, selected, status := world_stage_ready(world, dispatcher);
	if status != .Ok
	{
		return status;
	}
	world_invalidate_views_data(data);
	return physics.simulation_solve(&data.simulation, dt, selected);
}

// world_stage_optimize executes incremental solver/tree maintenance
world_stage_optimize :: #force_inline proc (
	world: ^World, dispatcher: ^Dispatcher = nil,
) -> Status
{
	data, selected, status := world_stage_ready(world, dispatcher);
	if status != .Ok
	{
		return status;
	}
	return physics.simulation_incrementally_optimize_data_structures(
		&data.simulation, selected,
	);
}
