#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// retain high-position residue for shape/continuity-only edits. complete float
	// static_apply and static_set_pose remain intentional float position authors
	static_apply_preserving_position :: proc (
		world: ^World, handle: Static_Handle, description: Static_Description,
		awakening: Awakening_Policy = .Overlaps,
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
		position, status = physics.statics_get_global_position(&data.simulation.statics, handle);
		if status != .Ok
		{
			return status;
		}
		return static_apply_global(world, handle, description, position, awakening);
	}

	// positions are native simulation-frame f64 values. the description's position
	// is replaced by a rounded cache before invoking the existing transaction
	// wake selection uses authoritative old/new bounds. normal growth may allocate
	static_add_global :: proc (
		world: ^World, description: Static_Description, position: Global_Position,
		awakening: Awakening_Policy = .Overlaps,
	) -> (Static_Handle, Status)
	{
		wake: physics.Reference_State;
		switch awakening
		{
		case .Overlaps:
			wake = .Present;
		case .None:
			wake = .Missing;
		case:
			return static_handle_invalid(), .Invalid_Argument;
		}
		stored: Static_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_add_static_internal(world_simulation(world), &stored, wake, &stored_position);
	}

	static_apply_global :: proc (
		world: ^World, handle: Static_Handle, description: Static_Description, position: Global_Position,
		awakening: Awakening_Policy = .Overlaps,
	) -> Status
	{
		wake: physics.Reference_State;
		switch awakening
		{
		case .Overlaps:
			wake = .Present;
		case .None:
			wake = .Missing;
		case:
			return .Invalid_Argument;
		}
		stored: Static_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_apply_static_description_internal(
			world_simulation(world), handle, &stored, wake, &stored_position,
		);
	}

	static_set_global_pose :: proc (
		world: ^World, handle: Static_Handle, value: Global_Pose,
		awakening: Awakening_Policy = .Overlaps,
	) -> Status
	{
		state: Static_Description;
		status: Status;
		state, status = static_get(world, handle);
		if status != .Ok
		{
			return status;
		}
		state.pose.orientation = value.orientation;
		return static_apply_global(world, handle, state, value.position, awakening);
	}

	// filtered high authoring uses the same validation, wake selection and commit
	// owner as unfiltered authoring. the filter sees stable body handles, not float
	// positions. it may read high poses while idle but must not mutate world state
	// nil selects all overlapping sleeping bodies. rejected descriptions do not
	// invoke the filter. wake effects preceding a later capacity error are retained
	static_add_global_filtered :: proc (
		world: ^World, description: Static_Description, position: Global_Position,
		filter: Static_Awakening_Filter_Proc, user_context: rawptr = nil,
	) -> (Static_Handle, Status)
	{
		stored: Static_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_add_static_internal(
			world_simulation(world), &stored, .Present, &stored_position, filter, user_context,
		);
	}

	static_apply_global_filtered :: proc (
		world: ^World, handle: Static_Handle, description: Static_Description, position: Global_Position,
		filter: Static_Awakening_Filter_Proc, user_context: rawptr = nil,
	) -> Status
	{
		stored: Static_Description = description;
		stored.pose.position = physics.world_precision_position_cache(position);
		stored_position: Global_Position = position;
		world_invalidate_views(world);
		return physics.simulation_apply_static_description_internal(
			world_simulation(world), handle, &stored, .Present, &stored_position, filter, user_context,
		);
	}
}
