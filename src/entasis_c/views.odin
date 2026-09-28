package entasis_c

import entasis "entasis:entasis"

// -----------------------------------------------------------------------------
// zero-copy world views and profiling snapshots
// -----------------------------------------------------------------------------

abi_active_body_view :: proc "contextless" (
	world: ^Entasis_World,
	out_view: ^Entasis_Active_Body_View,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_view == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_view^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	view, status := entasis.active_body_view(&resource.world);
	if status == .Ok
	{
		out_view^ = {
			handles=cast(^Entasis_Body_Handle)view.handles,
			dynamics=cast(^Entasis_Body_Dynamics)view.dynamics,
			collidables=cast(^Entasis_Collidable_State)view.collidables,
			activity=cast(^Entasis_Body_Activity)view.activity,
			count=u64(view.count),
			epoch=view.epoch,
		};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_view_valid :: proc "contextless" (
	world: ^Entasis_World,
	view: ^Entasis_Active_Body_View,
) -> Entasis_Bool
{
	resource := abi_world_get(world);
	if resource == nil || (resource.header.access != .Ready && resource.header.access != .Read_Phase) || view == nil || !abi_count_valid(view.count)
	{
		return ENTASIS_FALSE;
	}
	core := entasis.Active_Body_View{
		handles=cast([^]entasis.Body_Handle)view.handles,
		dynamics=cast([^]entasis.Body_Dynamics)view.dynamics,
		collidables=cast([^]entasis.Collidable)view.collidables,
		activity=cast([^]entasis.Body_Activity)view.activity,
		count=int(view.count),
		epoch=view.epoch,
	};
	return abi_bool(entasis.body_view_valid(&resource.world, core));
}

abi_active_body_row :: proc "contextless" (
	view: ^Entasis_Active_Body_View,
	index: u64,
	out_row: ^Entasis_Active_Body_Row,
) -> Entasis_Bool
{
	if out_row == nil
	{
		return ENTASIS_FALSE;
	}
	out_row^ = {};
	if view == nil || index >= view.count || view.handles == nil ||
	view.dynamics == nil || view.collidables == nil || view.activity == nil ||
	!abi_count_valid(index)
	{
		return ENTASIS_FALSE;
	}
	handles := cast([^]Entasis_Body_Handle)view.handles;
	dynamics := cast([^]Entasis_Body_Dynamics)view.dynamics;
	collidables := cast([^]Entasis_Collidable_State)view.collidables;
	activities := cast([^]Entasis_Body_Activity)view.activity;
	out_row^ = {
		handle=handles[int(index)],
		dynamics=&dynamics[int(index)],
		collidable=&collidables[int(index)],
		activity=&activities[int(index)],
	};
	return ENTASIS_TRUE;
}

abi_static_view :: proc "contextless" (
	world: ^Entasis_World,
	out_view: ^Entasis_Static_View,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_view == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_view^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	view, status := entasis.static_view(&resource.world);
	if status == .Ok
	{
		out_view^ = {
			handles=cast(^Entasis_Static_Handle)view.handles,
			records=cast(^Entasis_Static_View_Record)view.records,
			count=u64(view.count),
			epoch=view.epoch,
		};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_view_valid :: proc "contextless" (
	world: ^Entasis_World,
	view: ^Entasis_Static_View,
) -> Entasis_Bool
{
	resource := abi_world_get(world);
	if resource == nil || (resource.header.access != .Ready && resource.header.access != .Read_Phase) || view == nil || !abi_count_valid(view.count)
	{
		return ENTASIS_FALSE;
	}
	core := entasis.Static_View{
		handles=cast([^]entasis.Static_Handle)view.handles,
		records=cast([^]entasis.Static_Record)view.records,
		count=int(view.count),
		epoch=view.epoch,
	};
	return abi_bool(entasis.static_view_valid(&resource.world, core));
}

abi_static_view_row :: proc "contextless" (
	view: ^Entasis_Static_View,
	index: u64,
	out_row: ^Entasis_Static_Row,
) -> Entasis_Bool
{
	if out_row == nil
	{
		return ENTASIS_FALSE;
	}
	out_row^ = {};
	if view == nil || index >= view.count || view.handles == nil || view.records == nil ||
	!abi_count_valid(index)
	{
		return ENTASIS_FALSE;
	}
	handles := cast([^]Entasis_Static_Handle)view.handles;
	records := cast([^]Entasis_Static_View_Record)view.records;
	out_row^ = {
		handle=handles[int(index)],
		record=&records[int(index)],
	};
	return ENTASIS_TRUE;
}

abi_profile_stage_text :: #force_inline proc "contextless" (
	stage: Entasis_Profile_Stage,
) -> Entasis_String_View
{
	if u32(stage) >= ENTASIS_PROFILE_STAGE_COUNT
	{
		return {};
	}
	return abi_string_view(entasis.profile_stage_text(entasis.Profile_Stage(stage)));
}

abi_world_profile_enabled :: proc "contextless" (
	world: ^Entasis_World,
	out_enabled: ^Entasis_Bool,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_enabled == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_enabled^ = ENTASIS_FALSE;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	enabled, status := entasis.world_profile_enabled(&resource.world);
	if status == .Ok
	{
		out_enabled^ = abi_bool(enabled);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_world_profile_snapshot :: proc "contextless" (
	world: ^Entasis_World,
	out_snapshot: ^Entasis_Profile_Snapshot,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_snapshot == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_snapshot^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	snapshot, status := entasis.world_profile_snapshot(&resource.world);
	if status == .Ok
	{
		out_snapshot^ = transmute(Entasis_Profile_Snapshot)snapshot;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_world_solver_stats :: proc "contextless" (
	world: ^Entasis_World,
	out_stats: ^Entasis_World_Solver_Stats,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_stats == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_stats^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	stats, status := entasis.world_solver_stats(&resource.world);
	if status == .Ok
	{
		out_stats^ = {
			active_batches=i64(stats.active_batches),
			fallback_batch_index=i64(stats.fallback_batch_index),
			fallback_constraints=i64(stats.fallback_constraints),
		};
	}
	return abi_status_finish(status, diagnostic, .None);
}
#assert(size_of(Entasis_Motion_State) == size_of(entasis.Motion_State));
#assert(size_of(Entasis_Body_Inertias) == size_of(entasis.Body_Inertias));
#assert(size_of(Entasis_Body_Dynamics) == size_of(entasis.Body_Dynamics));
#assert(size_of(Entasis_Collidable_State) == size_of(entasis.Collidable));
#assert(size_of(Entasis_Body_Activity) == size_of(entasis.Body_Activity));
#assert(size_of(Entasis_Static_View_Record) == size_of(entasis.Static_Record));
#assert(size_of(Entasis_Profile_Snapshot) == size_of(entasis.Profile_Snapshot));
