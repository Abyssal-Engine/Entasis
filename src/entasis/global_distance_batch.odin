#+vet !unused-imports

package entasis
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// Closest_Point uses point and shape_b/pose_b. the other kinds use both shapes
	// and poses. settings retain the same explicit contract as Distance_Query
	Global_Distance_Query :: struct
	{
		kind: Distance_Query_Kind,
		shape_a, shape_b: Shape_Handle,
		pose_a, pose_b: Global_Pose,
		point: Global_Position,
		settings: Distance_Query_Settings,
	}

	Global_Distance_Query_Result :: struct
	{
		status: Status,
		shape: Global_Shape_Distance_Result,
		correction: Shape_Correction_Result,
	}

	// dense dedicated distance work avoids expanding the spatial batch's tagged
	// payload. every input executes in order under one context admission. return
	// the first failing status, later items still execute. result capacity failure
	// writes nothing. failed items publish no partial witness or correction data
	// inputs/results are independent caller spans. reserve EPA outside recurring
	// work with distance_query_reserve_with_context. no implicit reserve occurs
	// independent workers own distinct contexts/pools and disjoint output spans
	distance_query_batch_global_with_context :: proc "contextless" (
		query_context: ^Query_Context, queries: []Global_Distance_Query,
		results: []Global_Distance_Query_Result,
	) -> Status
	{
		if len(results) < len(queries)
		{
			return .Invalid_Argument;
		}
		data: ^query_context_data;
		status: Status;
		data, status = query_context_begin(query_context);
		if status != .Ok
		{
			return status;
		}
		defer data.execution = .Idle;
		first: Status = .Ok;
		for &query, index in queries
		{
			pose_a: Global_Pose = query.pose_a;
			if query.kind == .Closest_Point
			{
				pose_a = {position=query.point};
			}
			local: Distance_Query_Result = query_context_global_distance_execute(data, query.kind,
				query.shape_a, pose_a, query.shape_b, query.pose_b, query.settings);
			result: Global_Distance_Query_Result = {status=local.status};
			if local.status == .Ok
			{
				if query.kind == .Depenetration
				{
					result.correction = local.correction;
				}
				else
				{
					result.shape = global_distance_witnesses(local.shape, pose_a.position);
				}
			}
			results[index] = result;
			if first == .Ok && local.status != .Ok
			{
				first = local.status;
			}
		}
		return first;
	}
}
