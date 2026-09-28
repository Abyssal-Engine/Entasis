package entasis

import physics "entasis:entasis_physics"

// Body_Activation_State reports whether a live body is stored in the active set
// or in one sleeping island. Missing is the zero-value sentinel
Body_Activation_State :: enum u8
{
	Missing,
	Active,
	Sleeping,
}

// body_activation_state reports the storage state of one live body without
// exposing inactive-set indices or internal island storage
// allocation: none. ownership: read-only while the world is idle
body_activation_state :: #force_inline proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (Body_Activation_State, Status)
{
	simulation := world_simulation(world);
	if simulation == nil || simulation.state == .Disposed
	{
		return .Missing, .Disposed;
	}
	if simulation.state != .Ready
	{
		return .Missing, .Invalid_Argument;
	}
	location, status := physics.bodies_resolve(&simulation.bodies, handle);
	if status != .Ok
	{
		return .Missing, status;
	}
	if location.set_index == physics.BODIES_ACTIVE_SET_INDEX
	{
		return .Active, .Ok;
	}
	return .Sleeping, .Ok;
}

// body_is_active reports whether one live body is in the active simulation set.
// missing handles return false and preserve the low-level Not_Found status
body_is_active :: #force_inline proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (bool, Status)
{
	state, status := body_activation_state(world, handle);
	return state == .Active, status;
}

// body_is_sleeping reports whether one live body is stored in a sleeping island.
// missing handles return false and preserve the low-level Not_Found status
body_is_sleeping :: #force_inline proc "contextless" (
	world: ^World, handle: Body_Handle,
) -> (bool, Status)
{
	state, status := body_activation_state(world, handle);
	return state == .Sleeping, status;
}

// body_awaken awakens the complete constraint-connected island containing the
// body. the initialized world dispatcher is used unless an explicit dispatcher
// is supplied. an already active body is a successful no-op.
// allocation may occur only when existing active storage must grow
// ownership: owner thread only while the world is idle
body_awaken :: proc (
	world: ^World,
	handle: Body_Handle,
	dispatcher: ^Dispatcher = nil,
) -> Status
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	location, status := physics.bodies_resolve(&data.simulation.bodies, handle);
	if status != .Ok
	{
		return status;
	}
	if location.set_index == physics.BODIES_ACTIVE_SET_INDEX
	{
		return .Ok;
	}
	selected_dispatcher := dispatcher;
	if selected_dispatcher == nil
	{
		selected_dispatcher = data.dispatcher;
	}
	world_invalidate_views_data(data);
	return physics.island_awakener_awaken_body(
		&data.simulation.awakener,
		handle,
		selected_dispatcher,
		.Present,
	);
}

// bodies_sleep_group moves one caller-selected group of active, constraint-disconnected
// bodies into a sleeping island. handles must be unique and active. this operation is
// intended for controlled fixture construction and explicit world-state control.
// allocation may occur only when the reusable island scaffold must grow
// ownership: owner thread only while the world is idle
bodies_sleep_group :: proc (
	world: ^World,
	handles: []Body_Handle,
	dispatcher: ^Dispatcher = nil,
) -> Status
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return .Disposed;
	}
	if data.simulation.state != .Ready || len(handles) <= 0
	{
		return .Invalid_Argument;
	}
	scaffold := &data.simulation.sleeper.scaffold;
	constraint_capacity := max(int(scaffold.island_constraints.length), 1);
	capacity_status := physics.island_scaffold_ensure_capacity(
		scaffold, len(handles), constraint_capacity,
	);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	// this controlled group operation does not gather constraints. reject
	// connected bodies before migration rather than orphaning solver records
	// (including engine-owned axis locks) in the active set
	for handle in handles
	{
		location, status := physics.bodies_resolve(&data.simulation.bodies, handle);
		if status != .Ok
		{
			return status;
		}
		if location.set_index != physics.BODIES_ACTIVE_SET_INDEX ||
		data.simulation.bodies.sets.memory[location.set_index].constraints.memory[location.index].count != 0
		{
			return .Invalid_Argument;
		}
	}
	for index in 0 ..< len(handles)
	{
		scaffold.island_bodies.memory[index] = handles[index];
	}
	selected_dispatcher := dispatcher;
	if selected_dispatcher == nil
	{
		selected_dispatcher = data.dispatcher;
	}
	world_invalidate_views_data(data);
	return physics.island_sleeper_sleep_island(
		&data.simulation.sleeper,
		scaffold,
		len(handles),
		0,
		selected_dispatcher,
	);
}
