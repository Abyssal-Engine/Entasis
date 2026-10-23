#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// property-only replacement retains the authoritative position, not the rounded
	// position returned by body_get. orientation and all other fields are replaced
	// like body_apply this wakes the body and supersedes queued kinematic targets
	body_apply_preserving_position :: proc (
		world: ^World, handle: Body_Handle, description: Body_Description,
	) -> Status
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return status;
		}
		position: Global_Position;
		position, status = physics.bodies_get_global_position(&data.simulation.bodies, handle);
		if status != .Ok
		{
			return status;
		}
		return body_apply_global(world, handle, description, position);
	}

	// native mixed64 authoring. position is in the simulation's current f64 frame,
	// description.pose.position is replaced by its rounded cache. all other fields
	// retain the complete-description contract. no temporary rounded tree leaf is
	// published. owner-thread, idle-world mutation, normal growth may allocate
	body_add_global :: proc (
		world: ^World, description: Body_Description, position: Global_Position,
	) -> (Body_Handle, Status)
	{
		stored: Body_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_add_body(world_simulation(world), &stored, &stored_position);
	}

	body_apply_global :: proc (
		world: ^World, handle: Body_Handle, description: Body_Description, position: Global_Position,
	) -> Status
	{
		stored: Body_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_apply_body_description(world_simulation(world), handle, &stored, &stored_position);
	}

	// explicitly replaces position, including a value equal to the rounded cache
	// unlike a bounds refresh, this operation never infers intent from equality
	body_set_global_pose :: proc (
		world: ^World, handle: Body_Handle, value: Global_Pose,
	) -> Status
	{
		state: Body_Description;
		status: Status;
		state, status = body_get(world, handle);
		if status != .Ok
		{
			return status;
		}
		state.pose.orientation = value.orientation;
		return body_apply_global(world, handle, state, value.position);
	}
}
