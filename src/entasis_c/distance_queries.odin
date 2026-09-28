package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

// typed pointer reinterpretation is admitted only by all field offsets, sizes
// and alignments, plus the public disjoint-span checks below. no vector ABI
// or native pointer crosses this interface. enum values are generated unchanged

#assert(size_of(Entasis_Distance_Query_Settings) == size_of(entasis.Distance_Query_Settings));
#assert(align_of(Entasis_Distance_Query_Settings) == align_of(entasis.Distance_Query_Settings));
#assert(offset_of(Entasis_Distance_Query_Settings, absolute_tolerance) == offset_of(entasis.Distance_Query_Settings, absolute_tolerance));
#assert(offset_of(Entasis_Distance_Query_Settings, relative_tolerance) == offset_of(entasis.Distance_Query_Settings, relative_tolerance));
#assert(offset_of(Entasis_Distance_Query_Settings, maximum_iterations) == offset_of(entasis.Distance_Query_Settings, maximum_iterations));
#assert(size_of(Entasis_Distance_Query_Capacity) == size_of(entasis.Distance_Query_Capacity));
#assert(align_of(Entasis_Distance_Query_Capacity) == align_of(entasis.Distance_Query_Capacity));
#assert(offset_of(Entasis_Distance_Query_Capacity, vertices) == offset_of(entasis.Distance_Query_Capacity, vertices));
#assert(offset_of(Entasis_Distance_Query_Capacity, faces) == offset_of(entasis.Distance_Query_Capacity, faces));
#assert(offset_of(Entasis_Distance_Query_Capacity, edges) == offset_of(entasis.Distance_Query_Capacity, edges));
#assert(size_of(Entasis_Distance_Geometry) == size_of(entasis.Distance_Query_Geometry));
#assert(align_of(Entasis_Distance_Geometry) == align_of(entasis.Distance_Query_Geometry));
#assert(offset_of(Entasis_Distance_Geometry, state) == offset_of(entasis.Distance_Query_Geometry, state));
#assert(offset_of(Entasis_Distance_Geometry, point_a) == offset_of(entasis.Distance_Query_Geometry, point_a));
#assert(offset_of(Entasis_Distance_Geometry, point_b) == offset_of(entasis.Distance_Query_Geometry, point_b));
#assert(offset_of(Entasis_Distance_Geometry, normal) == offset_of(entasis.Distance_Query_Geometry, normal));
#assert(offset_of(Entasis_Distance_Geometry, distance) == offset_of(entasis.Distance_Query_Geometry, distance));
#assert(offset_of(Entasis_Distance_Geometry, depth) == offset_of(entasis.Distance_Query_Geometry, depth));
#assert(offset_of(Entasis_Distance_Geometry, iterations) == offset_of(entasis.Distance_Query_Geometry, iterations));
#assert(size_of(Entasis_Shape_Distance_Result) == size_of(entasis.Shape_Distance_Result));
#assert(align_of(Entasis_Shape_Distance_Result) == align_of(entasis.Shape_Distance_Result));
#assert(offset_of(Entasis_Shape_Distance_Result, geometry) == offset_of(entasis.Shape_Distance_Result, geometry));
#assert(offset_of(Entasis_Shape_Distance_Result, child_a) == offset_of(entasis.Shape_Distance_Result, child_a));
#assert(offset_of(Entasis_Shape_Distance_Result, child_b) == offset_of(entasis.Shape_Distance_Result, child_b));
#assert(size_of(Entasis_Shape_Correction_Result) == size_of(entasis.Shape_Correction_Result));
#assert(align_of(Entasis_Shape_Correction_Result) == align_of(entasis.Shape_Correction_Result));
#assert(offset_of(Entasis_Shape_Correction_Result, state) == offset_of(entasis.Shape_Correction_Result, state));
#assert(offset_of(Entasis_Shape_Correction_Result, translation) == offset_of(entasis.Shape_Correction_Result, translation));
#assert(offset_of(Entasis_Shape_Correction_Result, iterations) == offset_of(entasis.Shape_Correction_Result, iterations));
#assert(size_of(Entasis_Distance_Query) == size_of(entasis.Distance_Query));
#assert(align_of(Entasis_Distance_Query) == align_of(entasis.Distance_Query));
#assert(offset_of(Entasis_Distance_Query, kind) == offset_of(entasis.Distance_Query, kind));
#assert(offset_of(Entasis_Distance_Query, shape_a) == offset_of(entasis.Distance_Query, shape_a));
#assert(offset_of(Entasis_Distance_Query, shape_b) == offset_of(entasis.Distance_Query, shape_b));
#assert(offset_of(Entasis_Distance_Query, pose_a) == offset_of(entasis.Distance_Query, pose_a));
#assert(offset_of(Entasis_Distance_Query, pose_b) == offset_of(entasis.Distance_Query, pose_b));
#assert(offset_of(Entasis_Distance_Query, point) == offset_of(entasis.Distance_Query, point));
#assert(offset_of(Entasis_Distance_Query, settings) == offset_of(entasis.Distance_Query, settings));
#assert(size_of(Entasis_Distance_Query_Result) == size_of(entasis.Distance_Query_Result));
#assert(align_of(Entasis_Distance_Query_Result) == align_of(entasis.Distance_Query_Result));
#assert(offset_of(Entasis_Distance_Query_Result, status) == offset_of(entasis.Distance_Query_Result, status));
#assert(offset_of(Entasis_Distance_Query_Result, shape) == offset_of(entasis.Distance_Query_Result, shape));
#assert(offset_of(Entasis_Distance_Query_Result, correction) == offset_of(entasis.Distance_Query_Result, correction));
abi_distance_settings :: proc "contextless" (settings: ^Entasis_Distance_Query_Settings) -> entasis.Distance_Query_Settings
{
	if settings == nil
	{
		return entasis.distance_query_settings_default();
	}
	return cast(entasis.Distance_Query_Settings)settings^;
}

abi_distance_query_settings_default :: proc "contextless" () -> Entasis_Distance_Query_Settings
{
	return cast(Entasis_Distance_Query_Settings)entasis.distance_query_settings_default();
}

abi_distance_query_capacity_default :: proc "contextless" () -> Entasis_Distance_Query_Capacity
{
	return cast(Entasis_Distance_Query_Capacity)entasis.distance_query_capacity_default();
}

abi_distance_query_reserve :: proc "contextless" (
	world: ^Entasis_World,
	capacity: ^Entasis_Distance_Query_Capacity,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	value: entasis.Distance_Query_Capacity = entasis.distance_query_capacity_default();
	if capacity != nil
	{
		value = cast(entasis.Distance_Query_Capacity)capacity^;
	}
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	context = runtime.default_context();
	return abi_status_finish(entasis.distance_query_reserve(&resource.world, value), diagnostic, .None);
}

abi_shape_closest_point :: proc "contextless" (
	world: ^Entasis_World,
	point: Entasis_Vector3,
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
	if out_result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_result^ = {};
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_closest_point(&resource.world, abi_vector3_to_core(point), abi_shape_handle_to_core(shape), abi_pose_to_core(pose), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_shape_distance :: proc "contextless" (
	world: ^Entasis_World,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
	if out_result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_result^ = {};
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_distance(&resource.world, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_shape_penetration :: proc "contextless" (
	world: ^Entasis_World,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	settings_value: entasis.Distance_Query_Settings = abi_distance_settings(settings);
	if out_result == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_result^ = {};
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_penetration(&resource.world, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_shape_depenetrate :: proc "contextless" (
	world: ^Entasis_World,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
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
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	result: entasis.Shape_Correction_Result;
	status: entasis.Status;
	result, status = entasis.shape_depenetrate(&resource.world, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Correction_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_shape_pose :: proc "contextless" (
	world: ^Entasis_World,
	collidable: Entasis_Collidable_Reference,
	out_shape: ^Entasis_Shape_Handle,
	out_pose: ^Entasis_Rigid_Pose,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_shape == nil || out_pose == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_shape^ = {};
	out_pose^ = {};
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	shape: entasis.Shape_Handle;
	pose: entasis.Rigid_Pose;
	status: entasis.Status;
	shape, pose, status = entasis.collidable_shape_pose(&resource.world, cast(entasis.Collidable_Reference)collidable);
	if status == .Ok
	{
		out_shape^ = cast(Entasis_Shape_Handle)shape;
		out_pose^ = transmute(Entasis_Rigid_Pose)pose;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_distance_query_batch :: proc "contextless" (
	world: ^Entasis_World,
	queries: [^]Entasis_Distance_Query,
	count: u64,
	results: [^]Entasis_Distance_Query_Result,
	result_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if result_capacity < count || count > u64(max(int))/u64(size_of(Entasis_Distance_Query)) ||
	count > u64(max(int))/u64(size_of(Entasis_Distance_Query_Result))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	if count > 0
	{
		input: uintptr = uintptr(queries);
		output: uintptr = uintptr(results);
		input_bytes: uintptr = uintptr(count)*size_of(Entasis_Distance_Query);
		output_bytes: uintptr = uintptr(count)*size_of(Entasis_Distance_Query_Result);
		if input == 0 || output == 0 || input%align_of(Entasis_Distance_Query) != 0 ||
		output%align_of(Entasis_Distance_Query_Result) != 0 ||
		input > max(uintptr)-input_bytes || output > max(uintptr)-output_bytes ||
		(input < output+output_bytes && output < input+input_bytes)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
	}
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	status: entasis.Status = entasis.distance_query_batch(&resource.world,
		(cast([^]entasis.Distance_Query)queries)[:int(count)],
		(cast([^]entasis.Distance_Query_Result)results)[:int(count)]);
	return abi_status_finish(status, diagnostic, .None);
}

abi_overlap_any :: proc "contextless" (
	world: ^Entasis_World,
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	filter: ^Entasis_Query_Filter,
	out_state: ^Entasis_Overlap_State,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	filter_value: Entasis_Query_Filter = abi_query_filter_value(filter);
	if out_state == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_state^ = Entasis_Overlap_State(entasis.Overlap_State.Separated);
	resource: ^abi_world_resource;
	ready: Entasis_Status;
	resource, ready = abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter: entasis.Query_Filter = abi_query_filter_to_core(filter_value, &bridge);
	state: entasis.Overlap_State;
	status: entasis.Status;
	state, status = entasis.overlap_any(&resource.world, abi_shape_handle_to_core(shape), abi_pose_to_core(pose), core_filter);
	if status == .Ok
	{
		out_state^ = Entasis_Overlap_State(state);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_distance_query_reserve :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	capacity: ^Entasis_Distance_Query_Capacity,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	value: entasis.Distance_Query_Capacity = entasis.distance_query_capacity_default();
	if capacity != nil
	{
		value = cast(entasis.Distance_Query_Capacity)capacity^;
	}
	resource: ^abi_query_context_resource = abi_query_context_get(query_context);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.executing == .Executing || resource.world.header.access != .Ready
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource.executing = .Executing;
	resource.world.header.access = .Exclusive;
	defer
	{
		resource.executing = .Idle;
		resource.world.header.access = .Ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.distance_query_reserve_with_context(&resource.native, value), diagnostic, .None);
}

abi_query_context_shape_closest_point :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	point: Entasis_Vector3,
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
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
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_closest_point_with_context(&resource.native, abi_vector3_to_core(point), abi_shape_handle_to_core(shape), abi_pose_to_core(pose), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_shape_distance :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
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
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_distance_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_shape_penetration :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
	settings: ^Entasis_Distance_Query_Settings,
	out_result: ^Entasis_Shape_Distance_Result,
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
	result: entasis.Shape_Distance_Result;
	status: entasis.Status;
	result, status = entasis.shape_penetration_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Distance_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_shape_depenetrate :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
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
	result, status = entasis.shape_depenetrate_with_context(&resource.native, abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a), abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b), settings_value);
	if status == .Ok
	{
		out_result^ = transmute(Entasis_Shape_Correction_Result)result;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_collidable_shape_pose :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	collidable: Entasis_Collidable_Reference,
	out_shape: ^Entasis_Shape_Handle,
	out_pose: ^Entasis_Rigid_Pose,
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
	pose: entasis.Rigid_Pose;
	status: entasis.Status;
	shape, pose, status = entasis.collidable_shape_pose_with_context(&resource.native, cast(entasis.Collidable_Reference)collidable);
	if status == .Ok
	{
		out_shape^ = cast(Entasis_Shape_Handle)shape;
		out_pose^ = transmute(Entasis_Rigid_Pose)pose;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_distance_query_batch :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	queries: [^]Entasis_Distance_Query,
	count: u64,
	results: [^]Entasis_Distance_Query_Result,
	result_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if result_capacity < count || count > u64(max(int))/u64(size_of(Entasis_Distance_Query)) ||
	count > u64(max(int))/u64(size_of(Entasis_Distance_Query_Result))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	if count > 0
	{
		input: uintptr = uintptr(queries);
		output: uintptr = uintptr(results);
		input_bytes: uintptr = uintptr(count)*size_of(Entasis_Distance_Query);
		output_bytes: uintptr = uintptr(count)*size_of(Entasis_Distance_Query_Result);
		if input == 0 || output == 0 || input%align_of(Entasis_Distance_Query) != 0 ||
		output%align_of(Entasis_Distance_Query_Result) != 0 ||
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
	status: entasis.Status = entasis.distance_query_batch_with_context(&resource.native,
		(cast([^]entasis.Distance_Query)queries)[:int(count)],
		(cast([^]entasis.Distance_Query_Result)results)[:int(count)]);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_overlap_any :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape: Entasis_Shape_Handle,
	pose: Entasis_Rigid_Pose,
	filter: ^Entasis_Query_Filter,
	out_state: ^Entasis_Overlap_State,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	filter_value: Entasis_Query_Filter = abi_query_filter_value(filter);
	if out_state == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_state^ = Entasis_Overlap_State(entasis.Overlap_State.Separated);
	resource: ^abi_query_context_resource;
	ready: Entasis_Status;
	resource, ready = abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter: entasis.Query_Filter = abi_query_filter_to_core(filter_value, &bridge);
	state: entasis.Overlap_State;
	status: entasis.Status;
	state, status = entasis.overlap_any_with_context(&resource.native, abi_shape_handle_to_core(shape), abi_pose_to_core(pose), core_filter);
	if status == .Ok
	{
		out_state^ = Entasis_Overlap_State(state);
	}
	return abi_status_finish(status, diagnostic, .None);
}
