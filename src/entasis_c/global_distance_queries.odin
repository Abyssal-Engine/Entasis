#+vet !unused-imports

package entasis_c

import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	#assert(size_of(Entasis_Global_Distance_Query) == size_of(entasis.Global_Distance_Query));
	#assert(align_of(Entasis_Global_Distance_Query) == align_of(entasis.Global_Distance_Query));
	#assert(offset_of(Entasis_Global_Distance_Query, kind) == offset_of(entasis.Global_Distance_Query, kind));
	#assert(offset_of(Entasis_Global_Distance_Query, shape_a) == offset_of(entasis.Global_Distance_Query, shape_a));
	#assert(offset_of(Entasis_Global_Distance_Query, shape_b) == offset_of(entasis.Global_Distance_Query, shape_b));
	#assert(offset_of(Entasis_Global_Distance_Query, pose_a) == offset_of(entasis.Global_Distance_Query, pose_a));
	#assert(offset_of(Entasis_Global_Distance_Query, pose_b) == offset_of(entasis.Global_Distance_Query, pose_b));
	#assert(offset_of(Entasis_Global_Distance_Query, point) == offset_of(entasis.Global_Distance_Query, point));
	#assert(offset_of(Entasis_Global_Distance_Query, settings) == offset_of(entasis.Global_Distance_Query, settings));
	#assert(size_of(Entasis_Global_Distance_Query_Result) == size_of(entasis.Global_Distance_Query_Result));
	#assert(align_of(Entasis_Global_Distance_Query_Result) == align_of(entasis.Global_Distance_Query_Result));
	#assert(offset_of(Entasis_Global_Distance_Query_Result, status) == offset_of(entasis.Global_Distance_Query_Result, status));
	#assert(offset_of(Entasis_Global_Distance_Query_Result, shape) == offset_of(entasis.Global_Distance_Query_Result, shape));
	#assert(offset_of(Entasis_Global_Distance_Query_Result, correction) == offset_of(entasis.Global_Distance_Query_Result, correction));

	#assert(size_of(Entasis_Global_Distance_Geometry) == size_of(entasis.Global_Distance_Query_Geometry));
	#assert(align_of(Entasis_Global_Distance_Geometry) == align_of(entasis.Global_Distance_Query_Geometry));
	#assert(offset_of(Entasis_Global_Distance_Geometry, state) == offset_of(entasis.Global_Distance_Query_Geometry, state));
	#assert(offset_of(Entasis_Global_Distance_Geometry, point_a) == offset_of(entasis.Global_Distance_Query_Geometry, point_a));
	#assert(offset_of(Entasis_Global_Distance_Geometry, point_b) == offset_of(entasis.Global_Distance_Query_Geometry, point_b));
	#assert(offset_of(Entasis_Global_Distance_Geometry, normal) == offset_of(entasis.Global_Distance_Query_Geometry, normal));
	#assert(offset_of(Entasis_Global_Distance_Geometry, distance) == offset_of(entasis.Global_Distance_Query_Geometry, distance));
	#assert(offset_of(Entasis_Global_Distance_Geometry, depth) == offset_of(entasis.Global_Distance_Query_Geometry, depth));
	#assert(offset_of(Entasis_Global_Distance_Geometry, iterations) == offset_of(entasis.Global_Distance_Query_Geometry, iterations));
	#assert(size_of(Entasis_Global_Shape_Distance_Result) == size_of(entasis.Global_Shape_Distance_Result));
	#assert(align_of(Entasis_Global_Shape_Distance_Result) == align_of(entasis.Global_Shape_Distance_Result));
	#assert(offset_of(Entasis_Global_Shape_Distance_Result, geometry) == offset_of(entasis.Global_Shape_Distance_Result, geometry));
	#assert(offset_of(Entasis_Global_Shape_Distance_Result, child_a) == offset_of(entasis.Global_Shape_Distance_Result, child_a));
	#assert(offset_of(Entasis_Global_Shape_Distance_Result, child_b) == offset_of(entasis.Global_Shape_Distance_Result, child_b));
	abi_query_context_shape_closest_point_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		point: Entasis_Global_Position,
		shape: Entasis_Shape_Handle,
		pose: Entasis_Global_Pose,
		settings: ^Entasis_Distance_Query_Settings,
		out_result: ^Entasis_Global_Shape_Distance_Result,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
		if out_result == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_result^ = {};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		result: entasis.Global_Shape_Distance_Result;
		status: entasis.Status;
		result, status = entasis.shape_closest_point_global_with_context(&resource.native, cast(entasis.Global_Position)point, abi_shape_handle_to_core(shape), abi_global_pose_to_core(pose), settings_value);
		if status == .Ok
		{
			out_result^ = transmute(Entasis_Global_Shape_Distance_Result)result;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_shape_distance_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape_a: Entasis_Shape_Handle,
		pose_a: Entasis_Global_Pose,
		shape_b: Entasis_Shape_Handle,
		pose_b: Entasis_Global_Pose,
		settings: ^Entasis_Distance_Query_Settings,
		out_result: ^Entasis_Global_Shape_Distance_Result,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
		if out_result == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_result^ = {};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		result: entasis.Global_Shape_Distance_Result;
		status: entasis.Status;
		result, status = entasis.shape_distance_global_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_global_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_global_pose_to_core(pose_b), settings_value);
		if status == .Ok
		{
			out_result^ = transmute(Entasis_Global_Shape_Distance_Result)result;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_shape_penetration_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape_a: Entasis_Shape_Handle,
		pose_a: Entasis_Global_Pose,
		shape_b: Entasis_Shape_Handle,
		pose_b: Entasis_Global_Pose,
		settings: ^Entasis_Distance_Query_Settings,
		out_result: ^Entasis_Global_Shape_Distance_Result,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
		if out_result == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_result^ = {};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		result: entasis.Global_Shape_Distance_Result;
		status: entasis.Status;
		result, status = entasis.shape_penetration_global_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_global_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_global_pose_to_core(pose_b), settings_value);
		if status == .Ok
		{
			out_result^ = transmute(Entasis_Global_Shape_Distance_Result)result;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_shape_depenetrate_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		shape_a: Entasis_Shape_Handle,
		pose_a: Entasis_Global_Pose,
		shape_b: Entasis_Shape_Handle,
		pose_b: Entasis_Global_Pose,
		settings: ^Entasis_Distance_Query_Settings,
		out_result: ^Entasis_Shape_Correction_Result,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
		if out_result == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_result^ = {};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		result: entasis.Shape_Correction_Result;
		status: entasis.Status;
		result, status = entasis.shape_depenetrate_global_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_global_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_global_pose_to_core(pose_b), settings_value);
		if status == .Ok
		{
			out_result^ = transmute(Entasis_Shape_Correction_Result)result;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	
	abi_query_context_collidable_shape_global_pose :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		collidable: Entasis_Collidable_Reference,
		out_shape: ^Entasis_Shape_Handle,
		out_pose: ^Entasis_Global_Pose,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if out_shape == nil || out_pose == nil
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		out_shape^ = {};
		out_pose^ = {};
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		shape: entasis.Shape_Handle;
		pose: entasis.Global_Pose;
		status: entasis.Status;
		shape, pose, status = entasis.collidable_shape_global_pose_with_context(&resource.native, cast(entasis.Collidable_Reference)collidable);
		if status == .Ok
		{
			out_shape^ = cast(Entasis_Shape_Handle)shape;
			out_pose^ = transmute(Entasis_Global_Pose)pose;
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	abi_query_context_distance_query_batch_global :: proc "contextless" (
		query_context: ^Entasis_Query_Context,
		queries: [^]Entasis_Global_Distance_Query,
		count: u64,
		results: [^]Entasis_Global_Distance_Query_Result,
		result_capacity: u64,
		diagnostic: ^Entasis_Diagnostic,
	) -> Entasis_Status
	{
		if result_capacity < count || count > u64(max(int))/u64(size_of(Entasis_Global_Distance_Query)) ||
		count > u64(max(int))/u64(size_of(Entasis_Global_Distance_Query_Result))
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		if count > 0
		{
			input: uintptr = uintptr(queries);
			output: uintptr = uintptr(results);
			input_bytes: uintptr = uintptr(count)*size_of(Entasis_Global_Distance_Query);
			output_bytes: uintptr = uintptr(count)*size_of(Entasis_Global_Distance_Query_Result);
			if input == 0 || output == 0 || input%align_of(Entasis_Global_Distance_Query) != 0 ||
			output%align_of(Entasis_Global_Distance_Query_Result) != 0 ||
			input > max(uintptr)-input_bytes || output > max(uintptr)-output_bytes ||
			(input < output+output_bytes && output < input+input_bytes)
			{
				return abi_status_finish(.Invalid_Argument, diagnostic, .None);
			}
		}
		resource: ^abi_query_context_resource;
		ready: Entasis_Status;
		resource, ready = abi_context_query_begin(query_context, diagnostic);
		if resource == nil
		{
			return ready;
		}
		defer abi_context_query_end(resource);
		status: entasis.Status = entasis.distance_query_batch_global_with_context(&resource.native,
			(cast([^]entasis.Global_Distance_Query)queries)[:int(count)],
			(cast([^]entasis.Global_Distance_Query_Result)results)[:int(count)]);
		return abi_status_finish(status, diagnostic, .None);
	}
}
