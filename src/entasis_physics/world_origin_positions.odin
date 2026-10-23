package entasis_physics

import util "entasis:entasis_utilities"

world_precision_validate_body_positions :: proc "contextless" (
	bodies: ^Bodies, delta: util.Vector3,
) -> Physics_Status
{
	for set_index in 0 ..< int(bodies.sets.length)
	{
		set: ^Body_Set = &bodies.sets.memory[set_index];
		for index in 0 ..< set.count
		{
			status: Physics_Status;
			when util.PRECISION_MODE == .Double_Coordinates
			{
				status = world_precision_position_validate(world_precision_translate_global_position(set.global_positions.memory[index], delta));
			}
			else
			{
				_, status = world_precision_translate_vector(set.dynamics_state.memory[index].motion.pose.position, delta);
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

world_precision_translate_body_positions :: proc "contextless" (
	bodies: ^Bodies, delta: util.Vector3,
)
{
	for set_index in 0 ..< int(bodies.sets.length)
	{
		set: ^Body_Set = &bodies.sets.memory[set_index];
		for index in 0 ..< set.count
		{
			when util.PRECISION_MODE == .Double_Coordinates
			{
				position: Global_Position = world_precision_translate_global_position(set.global_positions.memory[index], delta);
				set.global_positions.memory[index] = position;
				set.dynamics_state.memory[index].motion.pose.position = world_precision_position_cache(position);
			}
			else
			{
				set.dynamics_state.memory[index].motion.pose.position, _ = world_precision_translate_vector(set.dynamics_state.memory[index].motion.pose.position, delta);
			}
		}
	}
}

world_precision_validate_static_positions :: proc "contextless" (
	statics: ^Statics, delta: util.Vector3,
) -> Physics_Status
{
	for index in 0 ..< statics.count
	{
		status: Physics_Status;
		when util.PRECISION_MODE == .Double_Coordinates
		{
			status = world_precision_position_validate(world_precision_translate_global_position(statics.global_positions.memory[index], delta));
		}
		else
		{
			_, status = world_precision_translate_vector(statics.statics.memory[index].pose.position, delta);
		}
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

world_precision_translate_static_positions :: proc "contextless" (
	statics: ^Statics, delta: util.Vector3,
)
{
	for index in 0 ..< statics.count
	{
		when util.PRECISION_MODE == .Double_Coordinates
		{
			position: Global_Position = world_precision_translate_global_position(statics.global_positions.memory[index], delta);
			statics.global_positions.memory[index] = position;
			statics.statics.memory[index].pose.position = world_precision_position_cache(position);
		}
		else
		{
			statics.statics.memory[index].pose.position, _ = world_precision_translate_vector(statics.statics.memory[index].pose.position, delta);
		}
	}
}
