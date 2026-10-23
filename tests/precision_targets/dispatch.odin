package precision_targets_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Target_Kind :: enum u8
{
	Servo,
	Axis_Lock,
}

Target_Route :: enum u8
{
	Direct_Bundle,
	Registered_Bundle,
	Direct_Lane,
	Registered_Lane,
}

add_float_target :: proc(t: ^testing.T, scene: ^Scene, body: physics.Body_Handle,
	target: util.Vector3, kind: Target_Kind, axes: u8 = 7, offset: util.Vector3 = {},
	angular_axes: u8 = 0, orientation: util.Quaternion = {0, 0, 0, 1}) -> physics.Constraint_Handle
{
	handles: [4]physics.Body_Handle = {body, {}, {}, {}};
	handle: physics.Constraint_Handle;
	status: physics.Physics_Status;
	if kind == .Servo
	{
		description: physics.One_Body_Linear_Servo = {offset, target,
			{angular_frequency=12.5, twice_damping_ratio=1.5},
			{maximum_speed=100, maximum_force=1e6}};
		handle, status = physics.solver_add(&scene.simulation.solver, &handles, &description);
	}
	else
	{
		testing.expect_value(t, physics.body_axis_lock_register(&scene.simulation.solver.registry), physics.Physics_Status.Ok);
		description: physics.Body_Axis_Lock_Description = {position=target,
			orientation=orientation, spring_settings={angular_frequency=12.5, twice_damping_ratio=1.5}};
		for axis in 0 ..< 3
		{
			if axes & (u8(1) << u32(axis)) != 0
			{
				description.linear[axis] = 1;
			}
			if angular_axes & (u8(1) << u32(axis)) != 0
			{
				description.angular[axis] = 1;
			}
		}
		handle, status = physics.solver_add_raw(&scene.simulation.solver, physics.BODY_AXIS_LOCK_TYPE_ID, &handles, &description);
	}
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return handle;
}

execute_target_stage :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Constraint_Handle,
	route: Target_Route, stage: physics.Solver_Execution_Stage) -> physics.Physics_Status
{
	solver: ^physics.Solver = &scene.simulation.solver;
	batch: ^physics.Type_Batch = constraint_batch(t, scene, handle);
	record: ^physics.Constraint_Type_Record = &solver.registry.records[batch.type_id];
	batch_index: int = int(solver.handle_to_constraint.memory[handle.value].batch_index);
	status: physics.Physics_Status;
	if route == .Direct_Bundle || route == .Registered_Bundle
	{
		for bundle in 0 ..< (int(batch.count) + 7) / 8
		{
			if route == .Direct_Bundle
			{
				status = physics.solver_execute_direct_noncontact_bundle_integration(solver, batch, batch_index, bundle,
					1.0 / 64, 64, .First, stage, 0, physics.ONE_BODY_LINEAR_SERVO_TYPE_ID, 1, .Never);
			}
			else
			{
				status = physics.solver_execute_type_bundle_dispatch_integration(solver, batch, record, batch_index, bundle,
					1.0 / 64, 64, .First, stage, 0, .Native, .Never);
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	else
	{
		for index in 0 ..< int(batch.count)
		{
			if route == .Direct_Lane
			{
				status = physics.solver_execute_direct_noncontact_lane_integration(solver, batch, batch_index, index,
					1.0 / 64, 64, .First, stage, 0, physics.ONE_BODY_LINEAR_SERVO_TYPE_ID, 1, .Never);
			}
			else
			{
				status = physics.solver_execute_type_lane_dispatch_integration(solver, batch, record, batch_index, index,
					1.0 / 64, 64, .First, stage, 0, .Native, .Never);
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}
