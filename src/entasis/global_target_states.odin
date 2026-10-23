#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	global_constraint_target_kind :: proc "contextless" (type_id: i32) -> Global_Constraint_Target_Kind
	{
		switch type_id
		{
		case physics.ONE_BODY_LINEAR_SERVO_TYPE_ID:
			return .Linear_Servo;
		case physics.BODY_AXIS_LOCK_TYPE_ID:
			return .Axis_Lock;
		case:
			return .Invalid;
		}
	}

	constraint_get_global_target_state_ready :: proc "contextless" (
		simulation: ^physics.Simulation, handle: Constraint_Handle,
	) -> (Global_Constraint_Target_State, Status)
	{
		position: Global_Position;
		status: Status;
		position, status = physics.simulation_get_constraint_global_target(simulation, handle);
		if status != .Ok
		{
			return {}, status;
		}
		kind: Global_Constraint_Target_Kind = global_constraint_target_kind(
			simulation.solver.handle_to_constraint.memory[handle.value].type_id,
		);
		return {position=position, kind=kind}, .Ok;
	}

	// captures only the absolute target, not the constraint description or impulses
	// other built-in types and opaque custom representations return Invalid_Argument
	constraint_get_global_target_state :: proc "contextless" (
		world: ^World, handle: Constraint_Handle,
	) -> (Global_Constraint_Target_State, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		return constraint_get_global_target_state_ready(&data.simulation, handle);
	}

	constraint_apply_global_target_state :: proc (
		world: ^World, handle: Constraint_Handle, state: Global_Constraint_Target_State,
	) -> Status
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return status;
		}
		current: Global_Constraint_Target_State;
		current, status = constraint_get_global_target_state_ready(&data.simulation, handle);
		if status != .Ok
		{
			return status;
		}
		if state.kind != current.kind
		{
			return .Invalid_Argument;
		}
		world_invalidate_views_data(data);
		return physics.simulation_set_constraint_global_target(&data.simulation, handle, state.position);
	}

	body_get_global_target_state_ready :: proc (
		simulation: ^physics.Simulation, body: Body_Handle,
	) -> (Global_Kinematic_Target_State, Status)
	{
		location: physics.Body_Memory_Location;
		status: Status;
		location, status = physics.bodies_resolve(&simulation.bodies, body);
		if status != .Ok
		{
			return {}, status;
		}
		inertia: Body_Inertia = simulation.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index].inertia.local;
		if physics.body_inertia_mobility(inertia) != .Kinematic
		{
			return {}, .Invalid_Argument;
		}
		storage: ^physics.Body_Control_Storage = simulation.body_control;
		if storage == nil
		{
			return {}, .Ok;
		}
		key: i32 = body.value;
		index: int = util.quick_dictionary_index_of(&storage.targets, &key);
		if index < 0 || storage.targets.values.memory[index].phase == .Expired
		{
			return {}, .Ok;
		}
		target: ^physics.Body_Control_Target = &storage.targets.values.memory[index];
		return {
			pose={position=target.global_position, orientation=target.pose.orientation}, presence=.Present,
		}, .Ok;
	}

	// no target and an expired target are explicit Absent values, not read errors
	// bodies must be kinematic. no control owner is allocated by capture
	body_get_global_target_state :: proc (
		world: ^World, body: Body_Handle,
	) -> (Global_Kinematic_Target_State, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		return body_get_global_target_state_ready(&data.simulation, body);
	}

	body_apply_global_target_state :: proc (
		world: ^World, body: Body_Handle, state: Global_Kinematic_Target_State,
		wake: Body_Control_Wake = .Wake,
	) -> Status
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return status;
		}
		if state.presence != .Absent && state.presence != .Present
		{
			return .Invalid_Argument;
		}
		if wake != .Wake && wake != .Preserve_Sleep
		{
			return .Invalid_Argument;
		}
		_, status = body_get_global_target_state_ready(&data.simulation, body);
		if status != .Ok
		{
			return status;
		}
		if state.presence == .Present
		{
			// optional control storage must already be enabled by the caller
			return body_set_global_kinematic_target(world, body, state.pose, wake);
		}
		if data.simulation.body_control == nil
		{
			return .Ok;
		}
		return body_clear_kinematic_target(world, body);
	}
}
