#+vet !unused-imports

package entasis_physics

import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// applies to caller-defined kernels and their matched reaction providers
	// Absolute_Float preserves the legacy rounded world positions. Body_A_Relative
	// subtracts authoritative positions before conversion, body A is zero, other
	// requested positions are relative to A, with unchanged world-oriented axes
	// one-body relative kernels receive zero and must use local description data
	// this does not reinterpret or rebase opaque absolute targets in descriptions
	Constraint_Position_Space :: enum u8
	{
		Absolute_Float,
		Body_A_Relative,
	}

	constraint_position_space_validate :: #force_inline proc "contextless" (
		space: Constraint_Position_Space,
	) -> Physics_Status
	{
		switch space
		{
		case .Absolute_Float, .Body_A_Relative:
			return .Ok;
		}
		return .Invalid_Description;
	}

	// reaction inputs expose every participant pose, independent of solve access
	// live active handles are established by the reaction owner before this call
	constraint_reaction_relative_poses :: proc "contextless" (
		bodies: ^Bodies, handles: ^[4]Body_Handle, body_count: int, poses: ^[4]Rigid_Pose,
	) -> Physics_Status
	{
		active: ^Body_Set = &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		index_a: int = int(bodies.handle_to_location.memory[handles[0].value].index);
		origin: Global_Position = active.global_positions.memory[index_a];
		poses[0].position = {};
		for body_index in 1 ..< body_count
		{
			index_b: int = int(bodies.handle_to_location.memory[handles[body_index].value].index);
			position: util.Vector3;
			status: Physics_Status;
			position, status = world_precision_relative_position(origin, active.global_positions.memory[index_b]);
			if status != .Ok
			{
				return status;
			}
			poses[body_index].position = position;
		}
		return .Ok;
	}
}
