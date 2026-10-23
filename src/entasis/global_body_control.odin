#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// freeze a local wrench at submission: subtract the authoritative center from
	// the authored world point in f64, then cross the f32 lever arm with the force
	// storage and wake semantics are identical to body_add_force. no input is
	// queued if the point or local lever cannot be represented
	body_add_force_at_global_position :: proc (
		world: ^World, body: Body_Handle, force: Vector3, position: Global_Position,
		wake: Body_Control_Wake = .Wake,
	) -> Status
	{
		owner: ^physics.Body_Control_Storage;
		status: Status;
		owner, status = body_control_owner(world);
		if status != .Ok
		{
			return status;
		}
		if physics.world_precision_position_validate(position) != .Ok
		{
			return .Invalid_Argument;
		}
		center: Global_Position;
		center, status = physics.bodies_get_global_position(&owner.simulation.bodies, body);
		if status != .Ok
		{
			return status;
		}
		lever: Vector3;
		lever, status = physics.world_precision_relative_position(center, position);
		if status != .Ok
		{
			return status;
		}
		world_invalidate_views_data(world_data_get(world));
		return physics.body_control_input_add(owner, body, force, util.vector3_cross(lever, force), .Force, wake);
	}

	// queue a high target through the existing sparse target owner. the float
	// setter remains an explicit float author, reads never promote its cache
	body_set_global_kinematic_target :: proc (
		world: ^World, body: Body_Handle, target: Global_Pose, wake: Body_Control_Wake = .Wake,
	) -> Status
	{
		owner: ^physics.Body_Control_Storage;
		status: Status;
		owner, status = body_control_owner(world);
		if status != .Ok
		{
			return status;
		}
		world_invalidate_views_data(world_data_get(world));
		return physics.body_control_target_set_global(owner, body, target.position, target.orientation, wake);
	}

	body_get_global_kinematic_target :: proc (
		world: ^World, body: Body_Handle,
	) -> (Global_Pose, Status)
	{
		owner: ^physics.Body_Control_Storage;
		status: Status;
		owner, status = body_control_owner(world);
		if status != .Ok
		{
			return {}, status;
		}
		_, _, status = physics.body_control_resolve(owner, body);
		if status != .Ok
		{
			return {}, status;
		}
		key: i32 = body.value;
		index: int = util.quick_dictionary_index_of(&owner.targets, &key);
		if index < 0 || owner.targets.values.memory[index].phase == .Expired
		{
			return {}, .Not_Found;
		}
		value: physics.Body_Control_Target = owner.targets.values.memory[index];
		return {position=value.global_position, orientation=value.pose.orientation}, .Ok;
	}
}
