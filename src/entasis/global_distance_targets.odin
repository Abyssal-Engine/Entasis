#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// resolve once and copy the shape plus authoritative pose for composing the
	// high shape queries. includes sleeping bodies and statics without awakening,
	// rounded-cache import or borrowed pointers. shapeless objects return the same
	// absent shape as the old resolver, subsequent geometry queries reject it
	// handle reuse follows the existing world lifetime contract, not generations
	collidable_shape_global_pose_with_context :: proc "contextless" (
		query_context: ^Query_Context, collidable: Collidable_Reference,
	) -> (Shape_Handle, Global_Pose, Status)
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return {}, {}, status;
		}
		defer data.execution = .Idle;
		if collidable_mobility(collidable) > .Static
		{
			return {}, {}, .Invalid_Argument;
		}
		target: physics.Shape_Query_Target;
		position: Global_Position;
		target, position, status = physics.simulation_query_global_target(&data.world.simulation, collidable);
		if status != .Ok
		{
			return {}, {}, status;
		}
		return target.shape, {position=position, orientation=target.pose.orientation}, .Ok;
	}
}
