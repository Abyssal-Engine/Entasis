#+vet !unused-imports

package entasis

import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// both witnesses are in the current simulation frame. distance, depth, normal,
	// state and iteration count retain the existing tolerance-bounded f32 kernel
	// semantics. do not recompute distance from rounded absolute witness positions
	Global_Distance_Query_Geometry :: struct
	{
		state: Distance_Query_State,
		point_a, point_b: Global_Position,
		normal: Vector3,
		distance, depth: f32,
		iterations: i32,
	}

	Global_Shape_Distance_Result :: struct
	{
		geometry: Global_Distance_Query_Geometry,
		child_a, child_b: i32,
	}

	global_distance_witnesses :: proc "contextless" (
		local: Shape_Distance_Result, origin: Global_Position,
	) -> Global_Shape_Distance_Result
	{
		g: Distance_Query_Geometry = local.geometry;
		return {
			geometry={
				state=g.state,
				point_a={origin.x + f64(g.point_a.x), origin.y + f64(g.point_a.y), origin.z + f64(g.point_a.z)},
				point_b={origin.x + f64(g.point_b.x), origin.y + f64(g.point_b.y), origin.z + f64(g.point_b.z)},
				normal=g.normal, distance=g.distance, depth=g.depth, iterations=g.iterations,
			},
			child_a=local.child_a, child_b=local.child_b,
		};
	}

	// own context admission once, then feed the existing distance dispatcher. a is
	// zero in a translation-only frame, its orientation is not used as a basis
	// Closest_Point uses A's position as the point and ignores shape_a/orientation_a
	// no added storage, collision path, implicit EPA reserve or world mutation
	global_distance_execute :: proc "contextless" (
		query_context: ^Query_Context, kind: Distance_Query_Kind,
		shape_a: Shape_Handle, pose_a: Global_Pose, shape_b: Shape_Handle, pose_b: Global_Pose,
		settings: Distance_Query_Settings,
	) -> Distance_Query_Result
	{
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return {status=status};
		}
		defer data.execution = .Idle;
		return query_context_global_distance_execute(data, kind, shape_a, pose_a, shape_b, pose_b, settings);
	}

	// local geometry at high coordinates: subtract in f64 before narrowing, and
	// reconstruct both witnesses in f64. shape extents and relative separations
	// remain subject to the existing local-float geometry limits
	// the world, shape data and registrations must remain immutable. the context
	// is exclusive, independent contexts may query the same immutable world
	// registered support/bounds/collision callbacks keep local-float semantics,
	// any poses they observe are in this point-centered frame. old APIs are unchanged
	shape_closest_point_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, point: Global_Position, shape: Shape_Handle, pose: Global_Pose,
		settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
	) -> (Global_Shape_Distance_Result, Status)
	{
		result: Distance_Query_Result = global_distance_execute(query_context, .Closest_Point,
			{}, {position=point, orientation=util.quaternion_identity()}, shape, pose, settings);
		if result.status != .Ok
		{
			return {}, result.status;
		}
		return global_distance_witnesses(result.shape, point), .Ok;
	}

	// unions of supported convex leaves, including the existing nested hierarchy
	// paths. child indices and tie ordering are inherited unchanged. no scratch is
	// needed for distance/point queries, the kernel does not perform EPA here
	shape_distance_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Global_Pose,
		shape_b: Shape_Handle, pose_b: Global_Pose,
		settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
	) -> (Global_Shape_Distance_Result, Status)
	{
		result: Distance_Query_Result = global_distance_execute(query_context, .Distance,
			shape_a, pose_a, shape_b, pose_b, settings);
		if result.status != .Ok
		{
			return {}, result.status;
		}
		return global_distance_witnesses(result.shape, pose_a.position), .Ok;
	}

	// convex penetration only, with existing EPA/analytic eligibility and statuses
	// explicit distance_query_reserve_with_context owns EPA capacity. a failure
	// publishes no witnesses, including callback, scratch and convergence failures
	shape_penetration_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Global_Pose,
		shape_b: Shape_Handle, pose_b: Global_Pose,
		settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
	) -> (Global_Shape_Distance_Result, Status)
	{
		result: Distance_Query_Result = global_distance_execute(query_context, .Penetration,
			shape_a, pose_a, shape_b, pose_b, settings);
		if result.status != .Ok
		{
			return {}, result.status;
		}
		return global_distance_witnesses(result.shape, pose_a.position), .Ok;
	}

	// the correction translates A. it is a world-oriented local f32 displacement,
	// not an absolute position, so the existing result needs no widening. apply it
	// to the authoritative position in f64. existing compound/mesh requery and
	// sidedness remain intact, no globally minimal compound correction is promised
	shape_depenetrate_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, shape_a: Shape_Handle, pose_a: Global_Pose,
		shape_b: Shape_Handle, pose_b: Global_Pose,
		settings: Distance_Query_Settings = {1e-5, 1e-5, 128},
	) -> (Shape_Correction_Result, Status)
	{
		result: Distance_Query_Result = global_distance_execute(query_context, .Depenetration,
			shape_a, pose_a, shape_b, pose_b, settings);
		if result.status != .Ok
		{
			return {}, result.status;
		}
		return result.correction, .Ok;
	}

	// the single-query and batch admission owners hold the same context for the
	// entire call. geometry conversion and execution have only this shared owner
	query_context_global_distance_execute :: proc "contextless" (
		data: ^query_context_data, kind: Distance_Query_Kind,
		shape_a: Shape_Handle, pose_a: Global_Pose, shape_b: Shape_Handle, pose_b: Global_Pose,
		settings: Distance_Query_Settings,
	) -> Distance_Query_Result
	{
		if kind > .Depenetration
		{
			return {status=.Invalid_Argument};
		}
		status: Status;
		// subtracting first would let identical infinities/NaNs into local setup
		// the original dispatcher remains the settings/orientation/shape validator
		if !(abs(pose_a.position.x) <= math.F64_MAX && abs(pose_a.position.y) <= math.F64_MAX &&
			abs(pose_a.position.z) <= math.F64_MAX && abs(pose_b.position.x) <= math.F64_MAX &&
			abs(pose_b.position.y) <= math.F64_MAX && abs(pose_b.position.z) <= math.F64_MAX)
		{
			return {status=.Invalid_Argument};
		}
		local_b: Vector3;
		local_b, status = physics.world_precision_relative_position(pose_a.position, pose_b.position);
		if status != .Ok
		{
			return {status=status};
		}
		query: Distance_Query = {
			kind=kind, shape_a=shape_a, shape_b=shape_b,
			pose_a={orientation=pose_a.orientation},
			pose_b={position=local_b, orientation=pose_b.orientation}, settings=settings,
		};
		return distance_query_execute(data.world, data.distance_storage, &query);
	}
}
