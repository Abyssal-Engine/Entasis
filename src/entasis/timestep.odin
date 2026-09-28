package entasis

import "core:math"

// Fixed_Stepper is caller-owned accumulator state. maximum_steps limits the
// number of physics steps performed by one update call. when the accumulated
// backlog exceeds that limit, whole excess steps are dropped while the
// fractional interpolation remainder is retained
Fixed_Stepper :: struct
{
	timestep:      f32,
	accumulator:   f32,
	maximum_steps: u8,
	_padding:      [3]u8,
}

// fixed_stepper constructs caller-owned fixed-step state
fixed_stepper :: #force_inline proc "contextless" (
	timestep: f32 = 1.0 / 60.0,
	maximum_steps: u8 = 8,
) -> Fixed_Stepper
{
	return {timestep=timestep, maximum_steps=maximum_steps};
}

// fixed_stepper_reset clears accumulated elapsed time without modifying policy
fixed_stepper_reset :: #force_inline proc "contextless" (stepper: ^Fixed_Stepper) -> Status
{
	if stepper == nil
	{
		return .Invalid_Argument;
	}
	stepper.accumulator = 0;
	return .Ok;
}

// fixed_stepper_update accumulates elapsed time and advances the world in exact
// fixed-duration steps. alpha is the remaining accumulator fraction in [0, 1).
// a failed world step is not consumed and can be retried by the caller
fixed_stepper_update :: proc (
	stepper: ^Fixed_Stepper,
	world: ^World,
	elapsed: f32,
	dispatcher: ^Dispatcher = nil,
) -> (steps: int, alpha: f32, status: Status)
{
	if stepper == nil || world == nil
	{
		status = .Invalid_Argument;
		return;
	}
	if stepper.timestep <= 0 || stepper.maximum_steps == 0 ||
		math.is_nan(stepper.timestep) || math.is_inf(stepper.timestep, 0) ||
		stepper.accumulator < 0 || math.is_nan(stepper.accumulator) ||
		math.is_inf(stepper.accumulator, 0)
	{
		status = .Invalid_Description;
		return;
	}
	if elapsed < 0 || math.is_nan(elapsed) || math.is_inf(elapsed, 0)
	{
		status = .Invalid_Argument;
		return;
	}

	accumulator := stepper.accumulator + elapsed;
	if math.is_inf(accumulator, 0)
	{
		status = .Invalid_Argument;
		return;
	}

	whole_steps, fractional_steps := math.modf(accumulator / stepper.timestep);
	available_steps := int(min(whole_steps, f32(stepper.maximum_steps)));
	if whole_steps > f32(stepper.maximum_steps)
	{
		// drop only whole excess steps. keep the fractional remainder for
		// renderer interpolation and for the next update
		accumulator =
			f32(stepper.maximum_steps) * stepper.timestep +
			fractional_steps * stepper.timestep;
	}

	for steps < available_steps
	{
		status = world_step(world, stepper.timestep, dispatcher);
		if status != .Ok
		{
			stepper.accumulator = accumulator;
			alpha = min(accumulator / stepper.timestep, 1);
			return;
		}
		accumulator -= stepper.timestep;
		steps += 1;
	}

	stepper.accumulator = max(accumulator, 0);
	alpha = min(max(stepper.accumulator / stepper.timestep, 0), 1);
	status = .Ok;
	return;
}

// solve_description_substeps constructs a fixed-iteration substep policy.
// each substep executes velocity_iterations solver iterations
solve_description_substeps :: #force_inline proc "contextless" (
	substeps: int,
	velocity_iterations: int = 8,
	fallback_batch_threshold: int = 64,
) -> Solve_Description
{
	return {
		velocity_iterations=i32(velocity_iterations),
		substeps=i32(substeps),
		fallback_batch_threshold=i32(fallback_batch_threshold),
	};
}

// solve_description_substep_scheduler constructs a substep policy whose
// iteration count is selected by a caller-owned allocation-free callback
solve_description_substep_scheduler :: #force_inline proc "contextless" (
	substeps: int,
	scheduler: Substep_Velocity_Iteration_Scheduler_Proc,
	user_context: rawptr = nil,
	fallback_velocity_iterations: int = 8,
	fallback_batch_threshold: int = 64,
) -> Solve_Description
{
	return {
		velocity_iterations=i32(fallback_velocity_iterations),
		substeps=i32(substeps),
		fallback_batch_threshold=i32(fallback_batch_threshold),
		velocity_iteration_scheduler=scheduler,
		scheduler_context=user_context,
	};
}
