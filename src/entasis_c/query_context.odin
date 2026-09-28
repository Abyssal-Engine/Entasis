package entasis_c

import shared "entasis:entasis_c_shared"
import "base:runtime"
import entasis "entasis:entasis"

ABI_QUERY_CONTEXT_MAGIC :: u64(0x454e545143545854);
abi_query_context_resource :: struct
{
	magic: u64,
	world: ^abi_world_resource,
	native: entasis.Query_Context,
	allocator: shared.Allocator,
	borrowed_pool: ^abi_buffer_pool_header,
	executing: abi_Execution_State,
	acquired_world: shared.World_Access,
}
#assert(size_of(Entasis_Query_Context_Description) == size_of(entasis.Query_Context_Description));
#assert(align_of(Entasis_Query_Context_Description) == align_of(entasis.Query_Context_Description));
abi_query_context_description_default :: proc "contextless" () -> Entasis_Query_Context_Description
{
	return cast(Entasis_Query_Context_Description)entasis.query_context_description_default();
}

abi_query_context_get :: proc "contextless" (query_context: ^Entasis_Query_Context) -> ^abi_query_context_resource
{
	if query_context == nil || query_context.opaque == nil
	{
		return nil;
	}
	resource := cast(^abi_query_context_resource)query_context.opaque;
	if resource.magic != ABI_QUERY_CONTEXT_MAGIC || resource.world == nil
	{
		return nil;
	}
	return resource;
}

abi_query_context_init :: proc "contextless" (
	query_context: ^Entasis_Query_Context, world: ^Entasis_World,
	description: ^Entasis_Query_Context_Description, borrowed_pool: ^Entasis_Buffer_Pool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if query_context == nil || query_context.opaque != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(owner);
	if owner == nil
	{
		return ready;
	}
	if owner.header.query_context_count == max(i32)
	{
		return abi_status_finish(.Capacity_Missing, diagnostic, .None);
	}
	pool_header: ^abi_buffer_pool_header;
	native_pool: ^entasis.Buffer_Pool;
	if borrowed_pool != nil
	{
		pool_header = abi_buffer_pool_header_get(borrowed_pool);
		if pool_header == nil || pool_header.borrowed || pool_header.attached != 0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		native_pool = pool_header.pool;
	}
	desc := entasis.query_context_description_default();
	if description != nil
	{
		desc = cast(entasis.Query_Context_Description)description^;
	}
	// claim the borrowed owner before invoking any application allocator. partial
	// initialization must not leave an apparently unattached pool available to reentry
	committed: abi_Initialization_State = .Pending;
	if pool_header != nil
	{
		pool_header.attached += 1;
		pool_header.query_context_attached = .Attached;
	}
	defer
	{
		if pool_header != nil && committed != .Committed
		{
			pool_header.attached -= 1;
			pool_header.query_context_attached = .Detached;
		}
	}
	memory, allocator, allocation_status := abi_resource_allocate_owned(size_of(abi_query_context_resource), align_of(abi_query_context_resource), &owner.allocator);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .None);
	}
	resource := cast(^abi_query_context_resource)memory;
	resource^ = {world=owner, allocator=allocator, borrowed_pool=pool_header};
	context = runtime.default_context();
	status := entasis.query_context_init(&resource.native, &owner.world, desc, native_pool);
	if status != .Ok
	{
		abi_resource_free(resource, size_of(abi_query_context_resource), align_of(abi_query_context_resource), &allocator);
		return abi_status_finish(status, diagnostic, .None);
	}
	committed = .Committed;
	owner.header.query_context_count += 1;
	resource.magic = ABI_QUERY_CONTEXT_MAGIC;
	query_context.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_query_context_reserve :: proc "contextless" (
	query_context: ^Entasis_Query_Context, description: ^Entasis_Query_Context_Description, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_query_context_get(query_context);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if description == nil || resource.executing == .Executing || resource.world.header.access != .Ready
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
	return abi_status_finish(entasis.query_context_reserve(&resource.native,
			cast(entasis.Query_Context_Description)description^), diagnostic, .None);
}

abi_query_context_destroy :: proc "contextless" (
	query_context: ^Entasis_Query_Context, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_query_context_get(query_context);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.world;
	if resource.executing == .Executing || owner.header.access != .Ready
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner.header.access = .Exclusive;
	resource.executing = .Executing;
	defer owner.header.access = .Ready;
	context = runtime.default_context();
	status := entasis.query_context_destroy(&resource.native);
	if status != .Ok
	{
		resource.executing = .Idle;
		return abi_status_finish(status, diagnostic, .None);
	}
	if resource.borrowed_pool != nil
	{
		resource.borrowed_pool.attached -= 1;
		resource.borrowed_pool.query_context_attached = .Detached;
	}
	owner.header.query_context_count -= 1;
	allocator := resource.allocator;
	resource.magic = 0;
	query_context.opaque = nil;
	abi_resource_free(resource, size_of(abi_query_context_resource), align_of(abi_query_context_resource), &allocator);
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_context_query_begin :: proc "contextless" (
	query_context: ^Entasis_Query_Context, diagnostic: ^Entasis_Diagnostic,
) -> (^abi_query_context_resource, Entasis_Status)
{
	resource := abi_query_context_get(query_context);
	if resource == nil
	{
		return nil, abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.executing == .Executing
	{
		return nil, abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	access := resource.world.header.access;
	if access != .Ready && access != .Read_Phase
	{
		return nil, abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource.executing = .Executing;
	resource.acquired_world = .Exclusive if access == .Ready else .Read_Phase;
	if resource.acquired_world == .Exclusive
	{
		resource.world.header.access = .Exclusive;
	}
	return resource, abi_status(.Ok);
}

abi_context_query_end :: proc "contextless" (resource: ^abi_query_context_resource)
{
	if resource.acquired_world == .Exclusive
	{
		resource.world.header.access = .Ready;
	}
	resource.acquired_world = .Ready;
	resource.executing = .Idle;
}

abi_query_context_ray_cast_any :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	ray_value: Entasis_Ray,
	filter: ^Entasis_Query_Filter,
	out_hit: ^Entasis_Bool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_hit^ = ENTASIS_FALSE;
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.ray_cast_any_with_context(
		&resource.native, transmute(entasis.Ray)ray_value, core_filter,
	);
	if status == .Ok
	{
		out_hit^ = abi_bool(hit == .Hit);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_ray_cast_closest :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	ray_value: Entasis_Ray,
	filter: ^Entasis_Query_Filter,
	out_hit: ^Entasis_Ray_Hit,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_hit^ = {child_index=-1};
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.ray_cast_closest_with_context(
		&resource.native, transmute(entasis.Ray)ray_value, core_filter,
	);
	if status == .Ok
	{
		out_hit^ = transmute(Entasis_Ray_Hit)hit;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_ray_cast_all :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	ray_value: Entasis_Ray,
	filter: ^Entasis_Query_Filter,
	hits: [^]Entasis_Ray_Hit,
	capacity: u64,
	out_written, out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
	(capacity > 0 && hits == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Ray_Hit)hits;
	written, status := entasis.ray_cast_all_with_context(
		&resource.native,
		transmute(entasis.Ray)ray_value,
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_sweep_any :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	velocity_value: Entasis_Body_Velocity,
	maximum_t: f32,
	filter: ^Entasis_Query_Filter,
	settings: ^Entasis_Sweep_Settings,
	out_hit: ^Entasis_Bool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_hit^ = ENTASIS_FALSE;
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.sweep_any_with_context(
		&resource.native,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		abi_velocity_to_core(velocity_value),
		maximum_t,
		core_filter,
		abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
	);
	if status == .Ok
	{
		out_hit^ = abi_bool(hit == .Hit);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_sweep_closest :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	velocity_value: Entasis_Body_Velocity,
	maximum_t: f32,
	filter: ^Entasis_Query_Filter,
	settings: ^Entasis_Sweep_Settings,
	out_hit: ^Entasis_Sweep_Hit,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_hit^ = {};
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.sweep_closest_with_context(
		&resource.native,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		abi_velocity_to_core(velocity_value),
		maximum_t,
		core_filter,
		abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
	);
	if status == .Ok
	{
		out_hit^ = transmute(Entasis_Sweep_Hit)hit;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_sweep_all :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	velocity_value: Entasis_Body_Velocity,
	maximum_t: f32,
	filter: ^Entasis_Query_Filter,
	settings: ^Entasis_Sweep_Settings,
	hits: [^]Entasis_Sweep_Hit,
	capacity: u64,
	out_written, out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
	(capacity > 0 && hits == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Sweep_Hit)hits;
	written, status := entasis.sweep_all_with_context(
		&resource.native,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		abi_velocity_to_core(velocity_value),
		maximum_t,
		core_hits[:int(capacity)],
		core_filter,
		abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_overlap_all :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	filter: ^Entasis_Query_Filter,
	hits: [^]Entasis_Overlap_Hit,
	capacity: u64,
	out_written, out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
	(capacity > 0 && hits == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Overlap_Hit)hits;
	written, status := entasis.overlap_all_with_context(
		&resource.native,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_volume_all :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	bounds: Entasis_Bounding_Box,
	filter: ^Entasis_Query_Filter,
	hits: [^]Entasis_Volume_Hit,
	capacity: u64,
	out_written, out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
	(capacity > 0 && hits == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Volume_Hit)hits;
	written, status := entasis.volume_all_with_context(
		&resource.native,
		transmute(entasis.Bounding_Box)bounds,
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_collision_query :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	shape_a: Entasis_Shape_Handle,
	pose_a: Entasis_Rigid_Pose,
	shape_b: Entasis_Shape_Handle,
	pose_b: Entasis_Rigid_Pose,
	speculative_margin: f32,
	out_manifold: ^Entasis_Contact_Manifold,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_manifold == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_manifold^ = {};
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	manifold, status := entasis.collision_query_with_context(
		&resource.native,
		abi_shape_handle_to_core(shape_a), abi_pose_to_core(pose_a),
		abi_shape_handle_to_core(shape_b), abi_pose_to_core(pose_b),
		speculative_margin,
	);
	if status == .Ok || status == .Not_Found
	{
		out_manifold^ = transmute(Entasis_Contact_Manifold)manifold;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_context_collision_query_batch :: proc "contextless" (
	query_context: ^Entasis_Query_Context,
	queries: [^]Entasis_Collision_Query,
	count: u64,
	results: [^]Entasis_Collision_Query_Result,
	result_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if !abi_count_valid(count) || result_capacity < count ||
	(count > 0 && (queries == nil || results == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	core_queries := cast([^]entasis.Collision_Query)queries;
	core_results := cast([^]entasis.Collision_Query_Result)results;
	status := entasis.collision_query_batch_with_context(
		&resource.native,
		core_queries[:int(count)],
		core_results[:int(count)],
	);
	return abi_status_finish(status, diagnostic, .None);
}

// one C access acquisition covers the entire batch, including filtered items.
// native descriptors keep foreign function pointers out of native callbacks
abi_query_context_query_batch :: proc "contextless" (
	query_context: ^Entasis_Query_Context, queries: [^]Entasis_Query, query_count: u64,
	results: [^]Entasis_Query_Result, result_capacity: u64, scratch: ^Entasis_Query_Scratch,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if !abi_count_valid(query_count) || result_capacity < query_count ||
	(query_count > 0 && (queries == nil || results == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	core_scratch: entasis.Query_Scratch;
	core_scratch_pointer: ^entasis.Query_Scratch;
	if scratch != nil
	{
		if !abi_count_valid(scratch.ray_capacity) || !abi_count_valid(scratch.overlap_capacity) ||
		!abi_count_valid(scratch.volume_capacity) ||
		(scratch.ray_capacity > 0 && scratch.ray_hits == nil) ||
		(scratch.overlap_capacity > 0 && scratch.overlap_hits == nil) ||
		(scratch.volume_capacity > 0 && scratch.volume_hits == nil)
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None);
		}
		core_scratch = {
			ray_hits=(cast([^]entasis.Ray_Hit)scratch.ray_hits)[:int(scratch.ray_capacity)],
			overlap_hits=(cast([^]entasis.Overlap_Hit)scratch.overlap_hits)[:int(scratch.overlap_capacity)],
			volume_hits=(cast([^]entasis.Volume_Hit)scratch.volume_hits)[:int(scratch.volume_capacity)],
		};
		core_scratch_pointer = &core_scratch;
	}
	resource, ready := abi_context_query_begin(query_context, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_context_query_end(resource);
	if abi_query_batch_is_core_compatible(queries, int(query_count))
	{
		native_queries := (cast([^]entasis.Query)queries)[:int(query_count)];
		native_results := (cast([^]entasis.Query_Result)results)[:int(query_count)];
		status := entasis.query_batch_with_context(&resource.native, native_queries, native_results, core_scratch_pointer);
		for index in 0 ..< int(query_count)
		{
			native := native_results[index];
			results[index] = {kind=queries[index].kind, status=abi_status(native.status), hit=abi_bool(native.hit), count=native.count};
			if native.hit
			{
				if queries[index].kind == 1
				{
					abi_query_result_write_ray(&results[index], transmute(Entasis_Ray_Hit)native.ray_hit);
				}
				if queries[index].kind == 3
				{
					abi_query_result_write_sweep(&results[index], transmute(Entasis_Sweep_Hit)native.sweep_hit);
				}
			}
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	overall := entasis.Status.Ok;
	for index in 0 ..< int(query_count)
	{
		input := &queries[index];
		output := &results[index];
		output^ = {kind=input.kind};
		native_queries: [1]entasis.Query;
		native_results: [1]entasis.Query_Result;
		bridge: abi_query_filter_bridge;
		switch input.kind
		{
			case 0, 1, 2:
			payload := cast(^abi_query_ray_payload)&input.payload[0];
			filter := abi_query_filter_to_core(payload.filter, &bridge);
			ray := transmute(entasis.Ray)payload.ray;
			if input.kind == 0
			{
				native_queries[0] = entasis.query_ray_any(ray, filter);
			}
			else if input.kind == 1
			{
				native_queries[0] = entasis.query_ray_closest(ray, filter);
			}
			else
			{
				native_queries[0] = entasis.query_ray_all(ray, cast(entasis.Query_Output)payload.output, filter);
			}
			case 3:
			payload := cast(^abi_query_sweep_payload)&input.payload[0];
			native_queries[0] = entasis.query_sweep_closest(abi_shape_handle_to_core(payload.shape),
				abi_pose_to_core(payload.pose), transmute(entasis.Body_Velocity)payload.velocity, payload.maximum_t,
				abi_query_filter_to_core(payload.filter, &bridge), abi_sweep_settings_to_core(payload.settings));
			case 4:
			payload := cast(^abi_query_overlap_payload)&input.payload[0];
			native_queries[0] = entasis.query_overlap_all(abi_shape_handle_to_core(payload.shape), abi_pose_to_core(payload.pose),
				cast(entasis.Query_Output)payload.output, abi_query_filter_to_core(payload.filter, &bridge));
			case 5:
			payload := cast(^abi_query_volume_payload)&input.payload[0];
			native_queries[0] = entasis.query_volume_all(transmute(entasis.Bounding_Box)payload.bounds,
				cast(entasis.Query_Output)payload.output, abi_query_filter_to_core(payload.filter, &bridge));
			case:
			output.status = abi_status(.Invalid_Argument);
			if overall == .Ok
			{
				overall = .Invalid_Argument;
			}
			continue;
		}
		status := entasis.query_batch_with_context(&resource.native, native_queries[:], native_results[:], core_scratch_pointer);
		native := native_results[0];
		output.status = abi_status(native.status);
		output.count = native.count;
		output.hit = abi_bool(native.hit);
		if native.hit
		{
			if input.kind == 1
			{
				abi_query_result_write_ray(output, transmute(Entasis_Ray_Hit)native.ray_hit);
			}
			if input.kind == 3
			{
				abi_query_result_write_sweep(output, transmute(Entasis_Sweep_Hit)native.sweep_hit);
			}
		}
		if overall == .Ok && status != .Ok
		{
			overall = status;
		}
	}
	return abi_status_finish(overall, diagnostic, .None);
}
