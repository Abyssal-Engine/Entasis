// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

DEFAULT_FALLBACK_BATCH_THRESHOLD :: 64;

Substep_Velocity_Iteration_Scheduler_Proc :: #type proc "contextless" (
	user_context: rawptr, substep_index: int,
) -> i32;

Solve_Description :: struct
{
	velocity_iteration_count: i32,
	substep_count:            i32,
	fallback_batch_threshold: i32,
	velocity_iteration_scheduler: Substep_Velocity_Iteration_Scheduler_Proc,
	scheduler_context:        rawptr,
}

solve_description_default :: proc "contextless" () -> Solve_Description
{
	return {
		velocity_iteration_count=8,
		substep_count=1,
		fallback_batch_threshold=DEFAULT_FALLBACK_BATCH_THRESHOLD,
	};
}

solve_description_validate :: proc "contextless" (description: Solve_Description) -> Physics_Status
{
	if description.velocity_iteration_count <= 0 || description.substep_count <= 0 ||
		description.fallback_batch_threshold <= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

solve_description_velocity_iterations :: proc "contextless" (
	description: ^Solve_Description, substep_index: int,
) -> int
{
	if description.velocity_iteration_scheduler != nil
	{
		count := description.velocity_iteration_scheduler(description.scheduler_context, substep_index);
		if count > 0
		{
			return int(count);
		}
	}
	return int(description.velocity_iteration_count);
}
