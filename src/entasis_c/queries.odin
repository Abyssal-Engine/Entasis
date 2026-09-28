package entasis_c

import entasis "entasis:entasis"

// -----------------------------------------------------------------------------
// query wire layouts and callback bridge
// -----------------------------------------------------------------------------

abi_query_filter_bridge :: struct
{
	filter: Entasis_Query_Filter,
}

abi_query_ray_payload :: struct
{
	ray:    Entasis_Ray,
	filter: Entasis_Query_Filter,
	output: Entasis_Query_Output,
}

abi_query_sweep_payload :: struct
{
	shape:     Entasis_Shape_Handle,
	pose:      Entasis_Rigid_Pose,
	velocity:  Entasis_Body_Velocity,
	maximum_t: f32,
	settings:  Entasis_Sweep_Settings,
	filter:    Entasis_Query_Filter,
}

abi_query_overlap_payload :: struct
{
	shape:  Entasis_Shape_Handle,
	pose:   Entasis_Rigid_Pose,
	filter: Entasis_Query_Filter,
	output: Entasis_Query_Output,
}

abi_query_volume_payload :: struct
{
	bounds: Entasis_Bounding_Box,
	filter: Entasis_Query_Filter,
	output: Entasis_Query_Output,
}

abi_query_filter_allow :: proc "contextless" (
	user_context: rawptr,
	collidable: entasis.Collidable_Reference,
) -> bool
{
	bridge := (^abi_query_filter_bridge)(user_context);
	if bridge == nil || bridge.filter.allow == nil
	{
		return true;
	}
	return bridge.filter.allow(
		bridge.filter.user_context,
		Entasis_Collidable_Reference(collidable),
	) != ENTASIS_FALSE;
}

abi_query_filter_allow_child :: proc "contextless" (
	user_context: rawptr,
	collidable: entasis.Collidable_Reference,
	child_index: i32,
) -> bool
{
	bridge := (^abi_query_filter_bridge)(user_context);
	if bridge == nil || bridge.filter.allow_child == nil
	{
		return true;
	}
	return bridge.filter.allow_child(
		bridge.filter.user_context,
		Entasis_Collidable_Reference(collidable),
		child_index,
	) != ENTASIS_FALSE;
}

abi_query_filter_value :: #force_inline proc "contextless" (
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query_Filter
{
	if filter == nil
	{
		return {};
	}
	return filter^;
}

abi_query_filter_to_core :: #force_inline proc "contextless" (
	value: Entasis_Query_Filter,
	bridge: ^abi_query_filter_bridge,
) -> entasis.Query_Filter
{
	bridge.filter = value;
	result := entasis.Query_Filter{
		include=transmute(entasis.Collidable_Mask)value.include,
		exclude=transmute(entasis.Collidable_Mask)value.exclude,
		user_context=bridge,
	};
	if value.allow != nil
	{
		result.allow = abi_query_filter_allow;
	}
	if value.allow_child != nil
	{
		result.allow_child = abi_query_filter_allow_child;
	}
	return result;
}

abi_sweep_settings_value :: #force_inline proc "contextless" (
	settings: ^Entasis_Sweep_Settings,
) -> Entasis_Sweep_Settings
{
	if settings == nil
	{
		return {};
	}
	return settings^;
}

abi_sweep_settings_to_core :: #force_inline proc "contextless" (
	value: Entasis_Sweep_Settings,
) -> entasis.Sweep_Settings
{
	return entasis.Sweep_Settings(value);
}

abi_query_begin :: #force_inline proc "contextless" (
	world: ^Entasis_World,
	diagnostic: ^Entasis_Diagnostic,
	access_mode: abi_Access_Mode = .Exclusive,
) -> (^abi_world_resource, Entasis_Status)
{
	return abi_world_resource_for_domain(world, diagnostic, .None, access_mode=access_mode);
}

abi_query_end :: #force_inline proc "contextless" (resource: ^abi_world_resource)
{
	abi_world_release(resource);
}

abi_query_required_lower_bound :: #force_inline proc "contextless" (
	written: int,
	status: entasis.Status,
) -> u64
{
	if written < 0
	{
		return 0;
	}
	if status == .Capacity_Missing && written < max(int)
	{
		return u64(written + 1);
	}
	return u64(written);
}

abi_ray :: #force_inline proc "contextless" (
	origin, direction: Entasis_Vector3,
	maximum_t: f32,
) -> Entasis_Ray
{
	return transmute(Entasis_Ray)entasis.ray(
		abi_vector3_to_core(origin), abi_vector3_to_core(direction), maximum_t,
	);
}

abi_query_filter_all :: #force_inline proc "contextless" () -> Entasis_Query_Filter
{
	return {include=ENTASIS_COLLIDABLE_ALL};
}

abi_query_filter_mobility :: #force_inline proc "contextless" (
	include: Entasis_Collidable_Mask,
) -> Entasis_Query_Filter
{
	return {include=include};
}

abi_collidable_mobility :: #force_inline proc "contextless" (
	collidable: Entasis_Collidable_Reference,
) -> Entasis_Body_Mobility
{
	return Entasis_Body_Mobility(entasis.collidable_mobility(
			entasis.Collidable_Reference(collidable),
	));
}

abi_collidable_body_handle :: proc "contextless" (
	collidable: Entasis_Collidable_Reference,
	out_handle: ^Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_handle^ = abi_body_handle_invalid();
	handle, status := entasis.collidable_body_handle(
		entasis.Collidable_Reference(collidable),
	);
	if status == .Ok
	{
		out_handle^ = Entasis_Body_Handle(handle);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_static_handle :: proc "contextless" (
	collidable: Entasis_Collidable_Reference,
	out_handle: ^Entasis_Static_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_handle^ = abi_static_handle_invalid();
	handle, status := entasis.collidable_static_handle(
		entasis.Collidable_Reference(collidable),
	);
	if status == .Ok
	{
		out_handle^ = Entasis_Static_Handle(handle);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_ray_cast_any :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.ray_cast_any(
		&resource.world, transmute(entasis.Ray)ray_value, core_filter,
	);
	if status == .Ok
	{
		out_hit^ = abi_bool(hit);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_ray_cast_closest :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.ray_cast_closest(
		&resource.world, transmute(entasis.Ray)ray_value, core_filter,
	);
	if status == .Ok
	{
		out_hit^ = transmute(Entasis_Ray_Hit)hit;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_ray_cast_all :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Ray_Hit)hits;
	written, status := entasis.ray_cast_all(
		&resource.world,
		transmute(entasis.Ray)ray_value,
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_sweep_any :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.sweep_any(
		&resource.world,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		abi_velocity_to_core(velocity_value),
		maximum_t,
		core_filter,
		abi_sweep_settings_to_core(abi_sweep_settings_value(settings)),
	);
	if status == .Ok
	{
		out_hit^ = abi_bool(hit);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_sweep_closest :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	hit, status := entasis.sweep_closest(
		&resource.world,
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

abi_sweep_all :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Sweep_Hit)hits;
	written, status := entasis.sweep_all(
		&resource.world,
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

abi_overlap_all :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Overlap_Hit)hits;
	written, status := entasis.overlap_all(
		&resource.world,
		abi_shape_handle_to_core(shape),
		abi_pose_to_core(pose_value),
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_volume_all :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	bridge: abi_query_filter_bridge;
	core_filter := abi_query_filter_to_core(abi_query_filter_value(filter), &bridge);
	core_hits := cast([^]entasis.Volume_Hit)hits;
	written, status := entasis.volume_all(
		&resource.world,
		transmute(entasis.Bounding_Box)bounds,
		core_hits[:int(capacity)],
		core_filter,
	);
	out_written^ = u64(max(written, 0));
	out_required^ = abi_query_required_lower_bound(written, status);
	return abi_status_finish(status, diagnostic, .None);
}

abi_query_output :: #force_inline proc "contextless" (start, capacity: i32) -> Entasis_Query_Output
{
	return {start=start, capacity=capacity};
}

abi_query_ray_any :: #force_inline proc "contextless" (
	ray_value: Entasis_Ray,
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(0)};
	payload := (^abi_query_ray_payload)(rawptr(&result.payload[0]));
	payload^ = {ray=ray_value, filter=abi_query_filter_value(filter)};
	return result;
}

abi_query_ray_closest :: #force_inline proc "contextless" (
	ray_value: Entasis_Ray,
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(1)};
	payload := (^abi_query_ray_payload)(rawptr(&result.payload[0]));
	payload^ = {ray=ray_value, filter=abi_query_filter_value(filter)};
	return result;
}

abi_query_ray_all :: #force_inline proc "contextless" (
	ray_value: Entasis_Ray,
	output: Entasis_Query_Output,
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(2)};
	payload := (^abi_query_ray_payload)(rawptr(&result.payload[0]));
	payload^ = {ray=ray_value, filter=abi_query_filter_value(filter), output=output};
	return result;
}

abi_query_sweep_closest :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	velocity_value: Entasis_Body_Velocity,
	maximum_t: f32,
	filter: ^Entasis_Query_Filter,
	settings: ^Entasis_Sweep_Settings,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(3)};
	payload := (^abi_query_sweep_payload)(rawptr(&result.payload[0]));
	payload^ = {
		shape=shape,
		pose=pose_value,
		velocity=velocity_value,
		maximum_t=maximum_t,
		settings=abi_sweep_settings_value(settings),
		filter=abi_query_filter_value(filter),
	};
	return result;
}

abi_query_overlap_all :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle,
	pose_value: Entasis_Rigid_Pose,
	output: Entasis_Query_Output,
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(4)};
	payload := (^abi_query_overlap_payload)(rawptr(&result.payload[0]));
	payload^ = {shape=shape, pose=pose_value, filter=abi_query_filter_value(filter), output=output};
	return result;
}

abi_query_volume_all :: #force_inline proc "contextless" (
	bounds: Entasis_Bounding_Box,
	output: Entasis_Query_Output,
	filter: ^Entasis_Query_Filter,
) -> Entasis_Query
{
	result := Entasis_Query{kind=Entasis_Query_Kind(5)};
	payload := (^abi_query_volume_payload)(rawptr(&result.payload[0]));
	payload^ = {bounds=bounds, filter=abi_query_filter_value(filter), output=output};
	return result;
}

abi_query_scratch :: #force_inline proc "contextless" (
	ray_hits: ^Entasis_Ray_Hit,
	ray_capacity: u64,
	overlap_hits: ^Entasis_Overlap_Hit,
	overlap_capacity: u64,
	volume_hits: ^Entasis_Volume_Hit,
	volume_capacity: u64,
) -> Entasis_Query_Scratch
{
	return {
		ray_hits=ray_hits,
		ray_capacity=ray_capacity,
		overlap_hits=overlap_hits,
		overlap_capacity=overlap_capacity,
		volume_hits=volume_hits,
		volume_capacity=volume_capacity,
	};
}

abi_query_output_range :: #force_inline proc "contextless" (
	output: Entasis_Query_Output,
	capacity: u64,
) -> (u64, u64, bool)
{
	if output.start < 0 || output.capacity < 0
	{
		return 0, 0, false;
	}
	start := u64(output.start);
	count := u64(output.capacity);
	if start > capacity || count > capacity - start
	{
		return 0, 0, false;
	}
	return start, count, true;
}

abi_query_result_write_ray :: #force_inline proc "contextless" (
	result: ^Entasis_Query_Result,
	hit: Entasis_Ray_Hit,
)
{
	(^Entasis_Ray_Hit)(rawptr(&result.payload[0]))^ = hit;
}

abi_query_result_write_sweep :: #force_inline proc "contextless" (
	result: ^Entasis_Query_Result,
	hit: Entasis_Sweep_Hit,
)
{
	(^Entasis_Sweep_Hit)(rawptr(&result.payload[0]))^ = hit;
}
#assert(size_of(Entasis_Query) == size_of(entasis.Query));
#assert(align_of(Entasis_Query) == align_of(entasis.Query));
#assert(offset_of(Entasis_Query, kind) == offset_of(entasis.Query, kind));
#assert(size_of(Entasis_Query_Result) == size_of(entasis.Query_Result));
#assert(align_of(Entasis_Query_Result) == align_of(entasis.Query_Result));
abi_query_filter_is_callback_free :: #force_inline proc "contextless" (
	filter: Entasis_Query_Filter,
) -> bool
{
	return filter.allow == nil && filter.allow_child == nil;
}

abi_query_batch_is_core_compatible :: proc "contextless" (
	queries: [^]Entasis_Query,
	count: int,
) -> bool
{
	for index in 0 ..< count
	{
		query := &queries[index];
		switch query.kind
		{
			case Entasis_Query_Kind(0), Entasis_Query_Kind(1), Entasis_Query_Kind(2):
			payload := (^abi_query_ray_payload)(rawptr(&query.payload[0]));
			if !abi_query_filter_is_callback_free(payload.filter)
			{
				return false;
			}
			case Entasis_Query_Kind(3):
			payload := (^abi_query_sweep_payload)(rawptr(&query.payload[0]));
			if !abi_query_filter_is_callback_free(payload.filter)
			{
				return false;
			}
			case Entasis_Query_Kind(4):
			payload := (^abi_query_overlap_payload)(rawptr(&query.payload[0]));
			if !abi_query_filter_is_callback_free(payload.filter)
			{
				return false;
			}
			case Entasis_Query_Kind(5):
			payload := (^abi_query_volume_payload)(rawptr(&query.payload[0]));
			if !abi_query_filter_is_callback_free(payload.filter)
			{
				return false;
			}
			case:
			return false;
		}
	}
	return true;
}

abi_query_batch :: #force_inline proc "contextless" (
	world: ^Entasis_World,
	queries: [^]Entasis_Query,
	query_count: u64,
	results: [^]Entasis_Query_Result,
	result_capacity: u64,
	scratch: ^Entasis_Query_Scratch,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if !abi_count_valid(query_count) || result_capacity < query_count ||
	(query_count > 0 && (queries == nil || results == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	count := int(query_count);
	owner := abi_world_get(world);
	read_phase := owner != nil && owner.header.access == .Read_Phase;
	if read_phase && entasis.query_batch_read_only_status(
		&owner.world, (cast([^]entasis.Query)queries)[:count],
	) != .Ok
	{
		// reject the entire batch before any result or hit-buffer write. the
		// ordinary mixed/scalar path borrows mutable scratch and is ineligible
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	if abi_query_batch_is_core_compatible(queries, count)
	{
		core_scratch: entasis.Query_Scratch;
		core_scratch_pointer: ^entasis.Query_Scratch;
		if scratch != nil
		{
			if !abi_count_valid(scratch.ray_capacity) ||
			!abi_count_valid(scratch.overlap_capacity) ||
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
		resource, ready := abi_query_begin(world, diagnostic, access_mode=(.Read_Only if read_phase else .Exclusive));
		if resource == nil
		{
			return ready;
		}
		defer abi_query_end(resource);
		core_queries := (cast([^]entasis.Query)queries)[:count];
		core_results := (cast([^]entasis.Query_Result)results)[:count];
		status := entasis.query_batch(
			&resource.world, core_queries, core_results, core_scratch_pointer,
		);
		for index in 0 ..< count
		{
			core_result := core_results[index];
			result := &results[index];
			result^ = {
				status=abi_status(core_result.status),
				hit=abi_bool(core_result.hit),
				kind=queries[index].kind,
				count=core_result.count,
			};
			if core_result.hit
			{
				switch queries[index].kind
				{
					case Entasis_Query_Kind(1):
					abi_query_result_write_ray(
						result, transmute(Entasis_Ray_Hit)core_result.ray_hit,
					);
					case Entasis_Query_Kind(3):
					abi_query_result_write_sweep(
						result, transmute(Entasis_Sweep_Hit)core_result.sweep_hit,
					);
				}
			}
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	overall := entasis.Status.Ok;
	for index := 0; index < count;
	{
		query := &queries[index];
		if query.kind == Entasis_Query_Kind(1)
		{
			payload := (^abi_query_ray_payload)(rawptr(&query.payload[0]));
			if abi_query_filter_is_callback_free(payload.filter)
			{
				end := index + 1;
				for end < count && queries[end].kind == Entasis_Query_Kind(1)
				{
					next := (^abi_query_ray_payload)(rawptr(&queries[end].payload[0]));
					if !abi_query_filter_is_callback_free(next.filter)
					{
						break;
					}
					end += 1;
				}
				resource, ready := abi_query_begin(world, nil);
				segment_status := entasis.Status(ready);
				if resource != nil
				{
					core_queries := (cast([^]entasis.Query)queries)[index:end];
					core_results := (cast([^]entasis.Query_Result)results)[index:end];
					segment_status = entasis.query_batch(&resource.world, core_queries, core_results, nil);
					for slot in index ..< end
					{
						core_result := core_results[slot-index];
						results[slot] = {kind=queries[slot].kind, status=abi_status(core_result.status),
							hit=abi_bool(core_result.hit), count=core_result.count};
						if core_result.hit
						{
							abi_query_result_write_ray(&results[slot], transmute(Entasis_Ray_Hit)core_result.ray_hit);
						}
					}
					abi_query_end(resource);
				}
				else
				{
					for slot in index ..< end
					{
						results[slot] = {kind=queries[slot].kind, status=ready};
					}
				}
				if overall == .Ok && segment_status != .Ok
				{
					overall = segment_status;
				}
				index = end;
				continue;
			}
		}
		result := &results[index];
		result^ = {kind=query.kind};
		status := entasis.Status.Invalid_Argument;
		switch query.kind
		{
			case Entasis_Query_Kind(0):
			payload := (^abi_query_ray_payload)(rawptr(&query.payload[0]));
			hit: Entasis_Bool;
			status = entasis.Status(abi_ray_cast_any(world, payload.ray, &payload.filter, &hit, nil));
			result.hit = hit;
			if hit != ENTASIS_FALSE
			{
				result.count = 1;
			}
			case Entasis_Query_Kind(1):
			payload := (^abi_query_ray_payload)(rawptr(&query.payload[0]));
			hit: Entasis_Ray_Hit;
			status = entasis.Status(abi_ray_cast_closest(world, payload.ray, &payload.filter, &hit, nil));
			if status == .Ok
			{
				result.hit = ENTASIS_TRUE;
				result.count = 1;
				abi_query_result_write_ray(result, hit);
			}
			case Entasis_Query_Kind(2):
			payload := (^abi_query_ray_payload)(rawptr(&query.payload[0]));
			if scratch == nil
			{
				status = .Invalid_Argument;
				break;
			}
			start, capacity, valid := abi_query_output_range(payload.output, scratch.ray_capacity);
			if !valid || (capacity > 0 && scratch.ray_hits == nil)
			{
				status = .Invalid_Argument;
				break;
			}
			written, required: u64;
			hit_pointer := cast([^]Entasis_Ray_Hit)scratch.ray_hits;
			output_pointer: [^]Entasis_Ray_Hit;
			if capacity > 0
			{
				output_pointer = &hit_pointer[int(start)];
			}
			status = entasis.Status(abi_ray_cast_all(
					world, payload.ray, &payload.filter,
					output_pointer, capacity, &written, &required, nil,
			));
			result.count = i32(min(written, u64(max(i32))));
			result.hit = abi_bool(written > 0);
			case Entasis_Query_Kind(3):
			payload := (^abi_query_sweep_payload)(rawptr(&query.payload[0]));
			hit: Entasis_Sweep_Hit;
			status = entasis.Status(abi_sweep_closest(
					world, payload.shape, payload.pose, payload.velocity, payload.maximum_t,
					&payload.filter, &payload.settings, &hit, nil,
			));
			if status == .Ok
			{
				result.hit = ENTASIS_TRUE;
				result.count = 1;
				abi_query_result_write_sweep(result, hit);
			}
			case Entasis_Query_Kind(4):
			payload := (^abi_query_overlap_payload)(rawptr(&query.payload[0]));
			if scratch == nil
			{
				status = .Invalid_Argument;
				break;
			}
			start, capacity, valid := abi_query_output_range(payload.output, scratch.overlap_capacity);
			if !valid || (capacity > 0 && scratch.overlap_hits == nil)
			{
				status = .Invalid_Argument;
				break;
			}
			written, required: u64;
			hit_pointer := cast([^]Entasis_Overlap_Hit)scratch.overlap_hits;
			output_pointer: [^]Entasis_Overlap_Hit;
			if capacity > 0
			{
				output_pointer = &hit_pointer[int(start)];
			}
			status = entasis.Status(abi_overlap_all(
					world, payload.shape, payload.pose, &payload.filter,
					output_pointer, capacity, &written, &required, nil,
			));
			result.count = i32(min(written, u64(max(i32))));
			result.hit = abi_bool(written > 0);
			case Entasis_Query_Kind(5):
			payload := (^abi_query_volume_payload)(rawptr(&query.payload[0]));
			if scratch == nil
			{
				status = .Invalid_Argument;
				break;
			}
			start, capacity, valid := abi_query_output_range(payload.output, scratch.volume_capacity);
			if !valid || (capacity > 0 && scratch.volume_hits == nil)
			{
				status = .Invalid_Argument;
				break;
			}
			written, required: u64;
			hit_pointer := cast([^]Entasis_Volume_Hit)scratch.volume_hits;
			output_pointer: [^]Entasis_Volume_Hit;
			if capacity > 0
			{
				output_pointer = &hit_pointer[int(start)];
			}
			status = entasis.Status(abi_volume_all(
					world, payload.bounds, &payload.filter,
					output_pointer, capacity, &written, &required, nil,
			));
			result.count = i32(min(written, u64(max(i32))));
			result.hit = abi_bool(written > 0);
			case:
			status = .Invalid_Argument;
		}
		result.status = abi_status(status);
		if overall == .Ok && status != .Ok
		{
			overall = status;
		}
		index += 1;
	}
	return abi_status_finish(overall, diagnostic, .None);
}

abi_collision_query :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	manifold, status := entasis.collision_query(
		&resource.world,
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

abi_collision_query_batch :: proc "contextless" (
	world: ^Entasis_World,
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
	resource, ready := abi_query_begin(world, diagnostic);
	if resource == nil
	{
		return ready;
	}
	defer abi_query_end(resource);
	core_queries := cast([^]entasis.Collision_Query)queries;
	core_results := cast([^]entasis.Collision_Query_Result)results;
	status := entasis.collision_query_batch(
		&resource.world,
		core_queries[:int(count)],
		core_results[:int(count)],
	);
	return abi_status_finish(status, diagnostic, .None);
}

// layout identity is required for direct caller-buffer writes and zero-copy views
#assert(size_of(Entasis_Ray_Hit) == size_of(entasis.Ray_Hit));
#assert(align_of(Entasis_Ray_Hit) == align_of(entasis.Ray_Hit));
#assert(size_of(Entasis_Sweep_Result) == size_of(entasis.Sweep_Result));
#assert(size_of(Entasis_Sweep_Hit) == size_of(entasis.Sweep_Hit));
#assert(size_of(Entasis_Contact_Manifold) == size_of(entasis.Manifold_Result));
#assert(align_of(Entasis_Contact_Manifold) == align_of(entasis.Manifold_Result));
#assert(size_of(Entasis_Overlap_Hit) == size_of(entasis.Overlap_Hit));
#assert(size_of(Entasis_Volume_Hit) == size_of(entasis.Volume_Hit));
#assert(size_of(Entasis_Collision_Query) == size_of(entasis.Collision_Query));
#assert(size_of(Entasis_Collision_Query_Result) == size_of(entasis.Collision_Query_Result));
#assert(size_of(abi_query_ray_payload) <= int(ENTASIS_QUERY_PAYLOAD_SIZE));
#assert(size_of(abi_query_sweep_payload) <= int(ENTASIS_QUERY_PAYLOAD_SIZE));
#assert(size_of(abi_query_overlap_payload) <= int(ENTASIS_QUERY_PAYLOAD_SIZE));
#assert(size_of(abi_query_volume_payload) <= int(ENTASIS_QUERY_PAYLOAD_SIZE));
#assert(size_of(Entasis_Ray_Hit) <= int(ENTASIS_QUERY_RESULT_PAYLOAD_SIZE));
#assert(size_of(Entasis_Sweep_Hit) <= int(ENTASIS_QUERY_RESULT_PAYLOAD_SIZE));
abi_body_collidable_reference :: proc "contextless" (world:^Entasis_World, handle:Entasis_Body_Handle, out_reference:^Entasis_Collidable_Reference, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_reference==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	reference, status:=entasis.body_collidable_reference(&owner.world, cast(entasis.Body_Handle)handle);
	if status==.Ok
	{
		out_reference^={reference.packed};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_collidable_reference :: proc "contextless" (world:^Entasis_World, handle:Entasis_Static_Handle, out_reference:^Entasis_Collidable_Reference, diagnostic:^Entasis_Diagnostic) -> Entasis_Status
{
	if out_reference==nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	owner, ready:=abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(owner);
	if owner==nil
	{
		return ready;
	}
	reference, status:=entasis.static_collidable_reference(&owner.world, cast(entasis.Static_Handle)handle);
	if status==.Ok
	{
		out_reference^={reference.packed};
	}
	return abi_status_finish(status, diagnostic, .None);
}
