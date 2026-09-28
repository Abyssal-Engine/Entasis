package entasis

import physics "entasis:entasis_physics"

// Simulation is the advanced low-level simulation type. a borrowed pointer is
// owned by World and remains valid only until world_destroy
Simulation :: physics.Simulation;

// world_borrow_simulation returns the low-level simulation owned by world.
// call only from the owner thread while the world is idle. the borrow transfers
// no ownership. any mutation through the pointer invalidates direct facade
// views, so this function advances the view epoch before returning
world_borrow_simulation :: proc "contextless" (
	world: ^World,
) -> (^Simulation, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	world_invalidate_views_data(data);
	return &data.simulation, .Ok;
}

// world_borrow_pool returns the Buffer_Pool used by the world. the pool remains
// world or caller owned according to world initialization. do not clear,
// dispose, or mutate it while the world is alive. temporary take/return use is
// permitted only on the owner thread while the world is idle
world_borrow_pool :: proc "contextless" (
	world: ^World,
) -> (^Buffer_Pool, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	if data.pool == nil || data.pool.state != .Ready
	{
		return nil, .Disposed;
	}
	return data.pool, .Ok;
}

// world_borrow_dispatcher returns the blocking low-level dispatcher currently
// owned or referenced by world. one-worker worlds may return nil with Ok.
// ownership is never transferred. do not dispatch concurrently with world_step
world_borrow_dispatcher :: proc "contextless" (
	world: ^World,
) -> (^Dispatcher, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, .Invalid_Argument;
	}
	return data.dispatcher, .Ok;
}
