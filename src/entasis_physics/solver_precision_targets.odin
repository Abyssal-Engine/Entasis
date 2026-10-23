#+vet !unused-imports

package entasis_physics

import "core:simd"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// one temporary float bundle, never retained or published. kernels still see
	// their original layout. only the selected target type uses this scratch
	Constraint_Local_Target_Prestep :: struct #raw_union
	{
		servo: One_Body_Linear_Servo_Prestep,
		axis_lock: Body_Axis_Lock_Prestep,
	}

	// the solver owns live active body references in selected lanes. integration
	// finishes first. each lane uses its body center as origin, subtracts in f64,
	// then narrows once. unlocked linear axes and inactive lanes require no delta
	// runtime range remains mutable, rejection discards scratch before execution
	solver_prepare_constraint_global_target :: proc "contextless" (
		bodies: ^Bodies, type_id: i32, source: rawptr, references, active_mask: util.I32x8,
		body: ^Constraint_Kernel_Body_Wide, output: ^Constraint_Local_Target_Prestep,
	) -> Physics_Status
	{
		if type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
		{
			output.servo = (^One_Body_Linear_Servo_Prestep)(source)^;
			output.servo.target = {};
		}
		else
		{
			output.axis_lock = (^Body_Axis_Lock_Prestep)(source)^;
			output.axis_lock.position = {};
		}
		body.position = {};
		active: ^Body_Set = &bodies.sets.memory[BODIES_ACTIVE_SET_INDEX];
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			if simd.extract(active_mask, lane) == 0
			{
				continue;
			}
			axes: u8 = 7;
			if type_id == BODY_AXIS_LOCK_TYPE_ID
			{
				axes = 0;
				for axis in 0 ..< 3
				{
					if simd.extract(output.axis_lock.linear[axis], lane) != 0
					{
						axes |= u8(1) << u32(axis);
					}
				}
				if axes == 0
				{
					continue;
				}
			}
			index: int = int(u32(simd.extract(references, lane)) & BODY_REFERENCE_INDEX_MASK);
			origin: Global_Position = active.global_positions.memory[index];
			target: Global_Position = constraint_global_target_read(source, type_id, lane);
			if axes & 1 == 0
			{
				target.x = origin.x;
			}
			if axes & 2 == 0
			{
				target.y = origin.y;
			}
			if axes & 4 == 0
			{
				target.z = origin.z;
			}
			delta: util.Vector3;
			status: Physics_Status;
			delta, status = world_precision_relative_position(origin, target);
			if status != .Ok
			{
				return status;
			}
			if type_id == ONE_BODY_LINEAR_SERVO_TYPE_ID
			{
				util.vector3_wide_write_slot(&output.servo.target, lane, delta);
			}
			else
			{
				util.vector3_wide_write_slot(&output.axis_lock.position, lane, delta);
			}
		}
		return .Ok;
	}
}
