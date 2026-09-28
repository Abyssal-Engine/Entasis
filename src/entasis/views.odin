package entasis

import physics "entasis:entasis_physics"

// Active_Body_View exposes the exact dense active-body columns. the dynamics
// column contains motion plus local and world inertia in the
// Body_Dynamics layout. it is not split into copied facade arrays
//
// the view is ephemeral. validate it once before a bulk loop and discard it
// before any world step, structural mutation, capacity operation, sleep or
// awakening transition. long-lived references must use Body_Handle
Active_Body_View :: struct
{
	handles:     [^]Body_Handle,
	dynamics:    [^]Body_Dynamics,
	collidables: [^]Collidable,
	activity:    [^]Body_Activity,
	count:       int,
	epoch:       u64,
}

// Active_Body_Row is a zero-copy row assembled from the active columns. the
// pointers inherit the lifetime of the source Active_Body_View
Active_Body_Row :: struct
{
	handle:     Body_Handle,
	dynamics:   ^Body_Dynamics,
	collidable: ^Collidable,
	activity:   ^Body_Activity,
}

// Static_View exposes the exact dense static records and reverse handle map.
// static records should be treated as read-only. use static_apply and the
// static setters to maintain broad-phase bounds and apply the awakening policy
Static_View :: struct
{
	handles: [^]Static_Handle,
	records: [^]Static_Record,
	count:   int,
	epoch:   u64,
}

// Static_Row is a zero-copy static row. its record pointer inherits the source
// Static_View lifetime
Static_Row :: struct
{
	handle: Static_Handle,
	record: ^Static_Record,
}

// active_body_view returns the dense active set without copying body data.
// sleeping bodies are intentionally excluded and remain accessible by handle
// ownership: owner thread only while the world is idle
active_body_view :: #force_inline proc "contextless" (
	world: ^World,
) -> (Active_Body_View, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	active := &data.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	if active.state != .Allocated
	{
		return {}, .Disposed;
	}
	return {
		handles=active.index_to_handle.memory,
		dynamics=active.dynamics_state.memory,
		collidables=active.collidables.memory,
		activity=active.activity.memory,
		count=active.count,
		epoch=data.view_epoch,
	}, .Ok;
}

// body_view_valid reports whether a previously acquired active-body view still
// names the current active columns. this is a diagnostic check, not permission
// to retain a view across a documented invalidation boundary
body_view_valid :: #force_inline proc "contextless" (
	world: ^World, view: Active_Body_View,
) -> bool
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready || view.epoch == 0 ||
		view.epoch != data.view_epoch
	{
		return false;
	}
	active := &data.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	return active.state == .Allocated &&
		view.handles == active.index_to_handle.memory &&
		view.dynamics == active.dynamics_state.memory &&
		view.collidables == active.collidables.memory &&
		view.activity == active.activity.memory &&
		view.count == active.count;
}

// active_body_row returns one zero-copy row. validate the view once before an
// iteration. this helper checks only the captured range
active_body_row :: #force_inline proc "contextless" (
	view: Active_Body_View, index: int,
) -> (Active_Body_Row, bool)
{
	if index < 0 || index >= view.count
	{
		return {}, false;
	}
	return {
		handle=view.handles[index],
		dynamics=&view.dynamics[index],
		collidable=&view.collidables[index],
		activity=&view.activity[index],
	}, true;
}

// static_view returns dense static records without copying
// ownership: owner thread only while the world is idle
static_view :: #force_inline proc "contextless" (
	world: ^World,
) -> (Static_View, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	statics := &data.simulation.statics;
	if statics.state != .Allocated
	{
		return {}, .Disposed;
	}
	return {
		handles=statics.index_to_handle.memory,
		records=statics.statics.memory,
		count=statics.count,
		epoch=data.view_epoch,
	}, .Ok;
}

// static_view_valid reports whether a previously acquired static view still
// names the current dense static storage
static_view_valid :: #force_inline proc "contextless" (
	world: ^World, view: Static_View,
) -> bool
{
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready || view.epoch == 0 ||
		view.epoch != data.view_epoch
	{
		return false;
	}
	statics := &data.simulation.statics;
	return statics.state == .Allocated &&
		view.handles == statics.index_to_handle.memory &&
		view.records == statics.statics.memory &&
		view.count == statics.count;
}

// static_view_row returns one zero-copy static row. validate the view once
// before an iteration. this helper checks only the captured range
static_view_row :: #force_inline proc "contextless" (
	view: Static_View, index: int,
) -> (Static_Row, bool)
{
	if index < 0 || index >= view.count
	{
		return {}, false;
	}
	return {handle=view.handles[index], record=&view.records[index]}, true;
}
