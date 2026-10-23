package entasis_physics

import util "entasis:entasis_utilities"

Origin_Target_Operation :: enum u8
{
	Validate,
	Commit,
}

world_precision_translate_origin_target :: proc "contextless" (
	position: Tree_Position, delta: util.Vector3,
) -> (Tree_Position, Physics_Status)
{
	when util.PRECISION_MODE == .Double_Coordinates
	{
		translated: Global_Position = world_precision_translate_global_position(position, delta);
		return translated, world_precision_position_validate(translated);
	}
	else
	{
		return world_precision_translate_vector(position, delta);
	}
}

// live target storage is owned by the admitted idle simulation, only translated
// coordinate admission can fail and commit follows a successful complete preflight
world_precision_update_origin_targets :: proc "contextless" (
	simulation: ^Simulation, delta: util.Vector3, operation: Origin_Target_Operation,
) -> Physics_Status
{
	storage: ^Body_Control_Storage = simulation.body_control;
	if storage != nil
	{
		for target_index in 0 ..< storage.targets.count
		{
			target: ^Body_Control_Target = &storage.targets.values.memory[target_index];
			position: Tree_Position;
			when util.PRECISION_MODE == .Double_Coordinates
			{
				position = target.global_position;
			}
			else
			{
				position = target.pose.position;
			}
			translated: Tree_Position;
			status: Physics_Status;
			translated, status = world_precision_translate_origin_target(position, delta);
			if status != .Ok
			{
				return status;
			}
			if operation == .Commit
			{
				when util.PRECISION_MODE == .Double_Coordinates
				{
					target.global_position = translated;
					target.pose.position = world_precision_position_cache(translated);
				}
				else
				{
					target.pose.position = translated;
				}
			}
		}
	}
	for batch_index in 0 ..< int(simulation.solver.active_set.batch_count)
	{
		batch: ^Constraint_Batch = &simulation.solver.active_set.batches.memory[batch_index];
		for type_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch: ^Type_Batch = &batch.type_batches.memory[type_index];
			if type_batch.type_id != BODY_AXIS_LOCK_TYPE_ID && type_batch.type_id != ONE_BODY_LINEAR_SERVO_TYPE_ID
			{
				continue;
			}
			for constraint_index in 0 ..< int(type_batch.count)
			{
				prestep: rawptr = type_batch_prestep_bundle(type_batch, constraint_index);
				lane: int = constraint_index % util.PRODUCTION_LANE_COUNT;
				position: Tree_Position;
				when util.PRECISION_MODE == .Double_Coordinates
				{
					position = constraint_global_target_read(prestep, type_batch.type_id, lane);
				}
				else
				{
					if type_batch.type_id == BODY_AXIS_LOCK_TYPE_ID
					{
						position = util.vector3_wide_read_slot((^Body_Axis_Lock_Prestep)(prestep).position, lane);
					}
					else
					{
						position = util.vector3_wide_read_slot((^One_Body_Linear_Servo_Prestep)(prestep).target, lane);
					}
				}
				translated: Tree_Position;
				status: Physics_Status;
				translated, status = world_precision_translate_origin_target(position, delta);
				if status != .Ok
				{
					return status;
				}
				if operation == .Commit
				{
					when util.PRECISION_MODE == .Double_Coordinates
					{
						constraint_global_target_write(prestep, type_batch.type_id, lane, translated);
					}
					else
					{
						if type_batch.type_id == BODY_AXIS_LOCK_TYPE_ID
						{
							util.vector3_wide_write_slot(&(^Body_Axis_Lock_Prestep)(prestep).position, lane, translated);
						}
						else
						{
							util.vector3_wide_write_slot(&(^One_Body_Linear_Servo_Prestep)(prestep).target, lane, translated);
						}
					}
				}
			}
		}
	}
	for record_index in 0 ..< simulation.sleeper.inactive_constraint_count
	{
		record: ^Inactive_Constraint_Record = &simulation.sleeper.inactive_constraints.memory[record_index];
		if record.type_id != BODY_AXIS_LOCK_TYPE_ID && record.type_id != ONE_BODY_LINEAR_SERVO_TYPE_ID
		{
			continue;
		}
		position: Tree_Position;
		when util.PRECISION_MODE == .Double_Coordinates
		{
			position = constraint_global_target_inactive_read(record);
		}
		else
		{
			if record.type_id == BODY_AXIS_LOCK_TYPE_ID
			{
				position = (^Body_Axis_Lock_Description)(rawptr(&record.description[0])).position;
			}
			else
			{
				position = (^One_Body_Linear_Servo)(rawptr(&record.description[0])).target;
			}
		}
		translated: Tree_Position;
		status: Physics_Status;
		translated, status = world_precision_translate_origin_target(position, delta);
		if status != .Ok
		{
			return status;
		}
		if operation == .Commit
		{
			when util.PRECISION_MODE == .Double_Coordinates
			{
				constraint_global_target_inactive_write(record, translated);
			}
			else
			{
				if record.type_id == BODY_AXIS_LOCK_TYPE_ID
				{
					(^Body_Axis_Lock_Description)(rawptr(&record.description[0])).position = translated;
				}
				else
				{
					(^One_Body_Linear_Servo)(rawptr(&record.description[0])).target = translated;
				}
			}
		}
	}
	return .Ok;
}
