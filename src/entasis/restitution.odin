package entasis

import physics "entasis:entasis_physics"

Restitution_Settings :: physics.Restitution_Settings;
Restitution_Configuration :: physics.Restitution_Configuration;
Restitution_Combine_Proc :: physics.Restitution_Combine_Proc;
restitution_configuration_default :: physics.restitution_configuration_default;
// enables optional solid-contact bounce. existing materials and trigger response
// are unchanged. the copied configuration borrows callback user data until disable
world_enable_restitution :: proc (
	world: ^World,
	configuration: Restitution_Configuration = {fallback={threshold=1}, collidable_capacity=16, pair_capacity=64},
) -> Status
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	return physics.restitution_storage_initialize(&data.simulation, configuration, data.allocator, data.allocation_scope);
}

world_disable_restitution :: proc (world: ^World) -> Status
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return .Disposed;
	}
	return physics.restitution_storage_destroy(&data.simulation);
}

@(private)
restitution_owner :: proc "contextless" (world: ^World) -> (^physics.Restitution_Storage, Status)
{
	data: ^world_data = world_data_get(world);
	if data == nil
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	if data.simulation.restitution == nil
	{
		return nil, .Not_Found;
	}
	return data.simulation.restitution, .Ok;
}

// reserves settings/history capacity without shrinking. target work buffers grow
// during selected solve preparation, before worker dispatch, and are then reused
restitution_reserve :: proc (world: ^World, collidable_capacity, pair_capacity: i32) -> Status
{
	owner: ^physics.Restitution_Storage;
	status: Status;
	owner, status = restitution_owner(world);
	if status != .Ok
	{
		return status;
	}
	return physics.restitution_storage_reserve(owner, collidable_capacity, pair_capacity);
}

restitution_set :: proc (world: ^World, reference: Collidable_Reference, settings: Restitution_Settings) -> Status
{
	owner: ^physics.Restitution_Storage;
	status: Status;
	owner, status = restitution_owner(world);
	if status != .Ok
	{
		return status;
	}
	return physics.restitution_storage_set(owner, reference, settings);
}

// returns the effective per-instance setting, including the configured fallback.
// changing a setting never implicitly awakens a sleeping body
restitution_get :: proc (world: ^World, reference: Collidable_Reference) -> (Restitution_Settings, Status)
{
	owner: ^physics.Restitution_Storage;
	status: Status;
	owner, status = restitution_owner(world);
	if status != .Ok
	{
		return {}, status;
	}
	_, status = physics.simulation_query_target(owner.simulation, reference);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.restitution_storage_lookup(owner, reference), .Ok;
}

// removes only the explicit override. the collidable remains alive and uses
// fallback settings at its next collision sample
restitution_remove :: proc (world: ^World, reference: Collidable_Reference) -> Status
{
	owner: ^physics.Restitution_Storage;
	status: Status;
	owner, status = restitution_owner(world);
	if status != .Ok
	{
		return status;
	}
	_, status = physics.simulation_query_target(owner.simulation, reference);
	if status != .Ok
	{
		return status;
	}
	return physics.restitution_storage_remove(owner, reference);
}
