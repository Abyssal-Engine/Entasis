#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Pose :: physics.Global_Pose;

	// authoritative native f64 position in the simulation's current frame. no
	// awakening, cache import, borrowed pointers or storage invalidation occurs
	body_get_global_pose :: proc "contextless" (world: ^World, handle: Body_Handle) -> (Global_Pose, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		reference: Collidable_Reference;
		reference, status = physics.collidable_reference_create(.Dynamic, int(handle.value));
		if status != .Ok
		{
			return {}, status;
		}
		return physics.simulation_get_global_pose(&data.simulation, reference);
	}

	static_get_global_pose :: proc "contextless" (world: ^World, handle: Static_Handle) -> (Global_Pose, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		reference: Collidable_Reference;
		reference, status = physics.collidable_reference_create(.Static, int(handle.value));
		if status != .Ok
		{
			return {}, status;
		}
		return physics.simulation_get_global_pose(&data.simulation, reference);
	}

	// apply the optional caller-owned application origin in f64. results follow
	// reference order and include sleeping bodies/statics. capacity/frame failure
	// writes nothing, later invalid references return the valid prefix. coordinates
	// are value copies, valid independently of subsequent body storage relocation
	world_export_global_poses :: proc "contextless" (
		world: ^World, references: []Collidable_Reference, output: []Global_Pose,
		frame: Origin_Frame = {},
	) -> (int, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return 0, status;
		}
		return physics.simulation_export_global_poses(&data.simulation, references, output, frame);
	}
}
