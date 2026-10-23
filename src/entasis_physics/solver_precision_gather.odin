#+vet !unused-imports

package entasis_physics

import "core:simd"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// the solver owns 1..4 registered body columns and live active references in
	// selected lanes, including kinematic metadata. masks exclude every unused lane
	// and unrequested position column. body zero defines the frame origin
	// all participants finish integration first
	// only mutable relative range is checked here, not the admitted storage layout
	solver_gather_constraint_relative_positions :: proc "contextless" (
		bodies: ^Bodies, references: [^]util.I32x8, body_count: int,
		position_bodies: u8, active_mask: util.I32x8,
		output: [^]Constraint_Kernel_Body_Wide,
	) -> Physics_Status
	{
		if position_bodies == 0
		{
			return .Ok;
		}
		output[0].position = {};
		for body_index in 1 ..< body_count
		{
			if position_bodies & (u8(1) << u32(body_index)) != 0
			{
				output[body_index].position = {};
			}
		}
		if position_bodies & ~u8(1) == 0
		{
			return .Ok;
		}
		active: ^Body_Set = &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if simd.extract(active_mask, lane) == 0
			{
				continue;
			}
			index_a: int = int(u32(simd.extract(references[0], lane)) & BODY_REFERENCE_INDEX_MASK);
			origin: Global_Position = active.global_positions.memory[index_a];
			for body_index in 1 ..< body_count
			{
				if position_bodies & (u8(1) << u32(body_index)) == 0
				{
					continue;
				}
				index_b: int = int(u32(simd.extract(references[body_index], lane)) & BODY_REFERENCE_INDEX_MASK);
				delta: util.Vector3;
				status: Physics_Status;
				delta, status = world_precision_relative_position(origin, active.global_positions.memory[index_b]);
				if status != .Ok
				{
					return status;
				}
				util.vector3_wide_write_slot(&output[body_index].position, lane, delta);
			}
		}
		return .Ok;
	}
}
