#+vet !unused-imports

package entasis_physics

import "core:math"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Global_Pose :: struct
	{
		position: Global_Position,
		orientation: util.Quaternion,
	}

	// read authoritative positions, including sleeping bodies and shapeless bodies
	// a pose is a value copy, not a borrowed view into mutable simulation storage
	simulation_get_global_pose :: proc "contextless" (
		simulation: ^Simulation, reference: Collidable_Reference,
	) -> (Global_Pose, Physics_Status)
	{
		if simulation == nil || simulation.state != .Ready
		{
			return {}, .Disposed;
		}
		mobility: Body_Mobility = collidable_reference_mobility(reference);
		if mobility != .Dynamic && mobility != .Kinematic && mobility != .Static
		{
			return {}, .Invalid_Argument;
		}
		target: Shape_Query_Target;
		position: Global_Position;
		status: Physics_Status;
		target, position, status = simulation_query_global_target(simulation, reference);
		if status != .Ok
		{
			return {}, status;
		}
		return {position=position, orientation=target.pose.orientation}, .Ok;
	}

	// preserve input order. insufficient output or an invalid application frame
	// writes nothing. a later invalid handle/overflow returns the valid prefix
	// the caller owns stable, disjoint input/output and an immutable simulation
	simulation_export_global_poses :: proc "contextless" (
		simulation: ^Simulation, references: []Collidable_Reference, output: []Global_Pose,
		frame: Origin_Frame = {},
	) -> (int, Physics_Status)
	{
		if simulation == nil || simulation.state != .Ready
		{
			return 0, .Disposed;
		}
		if len(output) < len(references)
		{
			return 0, .Capacity_Missing;
		}
		if !(abs(frame.origin.x) <= math.F64_MAX && abs(frame.origin.y) <= math.F64_MAX && abs(frame.origin.z) <= math.F64_MAX)
		{
			return 0, .Invalid_Argument;
		}
		for reference, index in references
		{
			pose: Global_Pose;
			status: Physics_Status;
			pose, status = simulation_get_global_pose(simulation, reference);
			if status != .Ok
			{
				return index, status;
			}
			pose.position.x += frame.origin.x;
			pose.position.y += frame.origin.y;
			pose.position.z += frame.origin.z;
			// direct finite comparisons retain NaN/Inf rejection and let LLVM
			// keep floating-classifier calls out of the export loop
			if !(abs(pose.position.x) <= math.F64_MAX && abs(pose.position.y) <= math.F64_MAX && abs(pose.position.z) <= math.F64_MAX)
			{
				return index, .Invalid_Argument;
			}
			output[index] = pose;
		}
		return len(references), .Ok;
	}
}
