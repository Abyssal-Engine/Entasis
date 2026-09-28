// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:runtime"

Timestep_Completion_Stage :: enum u8
{
	Slept,
	Before_Collision_Detection,
	Collisions_Detected,
	Constraints_Solved,
}

Simulation_Stage_Completed_Proc :: #type proc "contextless" (
	user_context: rawptr, stage: Timestep_Completion_Stage, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status;

Timestep_Callbacks :: struct
{
	stage_completed: Simulation_Stage_Completed_Proc,
	user_context:    rawptr,
}

Timestep_Proc :: #type proc "contextless" (
	user_context: rawptr, simulation: ^Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status;

Timestepper :: struct
{
	step:         Timestep_Proc,
	user_context: rawptr,
}

timestep_completion_callback :: proc "contextless" (
	simulation: ^Simulation, stage: Timestep_Completion_Stage, profiler_stage: Simulation_Stage,
	dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	simulation_profiler_start(&simulation.profiler, profiler_stage);
	status := Physics_Status.Ok;
	if simulation.timestep_callbacks.stage_completed != nil
	{
		status = simulation.timestep_callbacks.stage_completed(
			simulation.timestep_callbacks.user_context, stage, dt, dispatcher,
		);
	}
	simulation_profiler_end(&simulation.profiler, profiler_stage);
	return status;
}

default_timestepper_step :: proc "contextless" (
	user_context: rawptr, simulation: ^Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> Physics_Status
{
	context = runtime.default_context();
	_ = user_context;
	if simulation == nil || simulation.state != .Stepping
	{
		return .Invalid_Argument;
	}
	status := simulation_sleep(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	status = timestep_completion_callback(
		simulation, .Slept, .Slept_Callback, dt, dispatcher,
	);
	if status != .Ok
	{
		return status;
	}
	status = simulation_predict_bounding_boxes(simulation, dt, dispatcher);
	if status != .Ok
	{
		return status;
	}
	status = timestep_completion_callback(
		simulation, .Before_Collision_Detection, .Before_Collision_Callback, dt, dispatcher,
	);
	if status != .Ok
	{
		return status;
	}
	status = simulation_collision_detection(simulation, dt, dispatcher);
	if status != .Ok
	{
		return status;
	}
	status = timestep_completion_callback(
		simulation, .Collisions_Detected, .Collisions_Detected_Callback, dt, dispatcher,
	);
	if status != .Ok
	{
		return status;
	}
	status = simulation_solve(simulation, dt, dispatcher);
	if status != .Ok
	{
		return status;
	}
	status = timestep_completion_callback(
		simulation, .Constraints_Solved, .Constraints_Solved_Callback, dt, dispatcher,
	);
	if status != .Ok
	{
		return status;
	}
	status = simulation_incrementally_optimize_data_structures(simulation, dispatcher);
	if status != .Ok
	{
		return status;
	}
	simulation_profiler_start(&simulation.profiler, .Cleanup);
	simulation.step_index += 1;
	simulation_profiler_end(&simulation.profiler, .Cleanup);
	return .Ok;
}

default_timestepper :: proc "contextless" () -> Timestepper
{
	return {step=default_timestepper_step};
}
