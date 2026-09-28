package entasis

import physics "entasis:entasis_physics"

@(private)
world_simulation :: #force_inline proc "contextless" (world: ^World) -> ^physics.Simulation
{
	data := world_data_get(world);
	if data == nil
	{
		return nil;
	}
	return &data.simulation;
}

// ray_query executes the advanced collector-based ray path without allocation
//
// separate helpers provide ordinary any, closest, and caller-buffer queries
// ownership: owner thread while idle, or read-only during a documented query phase
ray_query :: #force_inline proc "contextless" (
	world: ^World, ray: Ray, collector: ^Ray_Query_Collector, pool: ^Buffer_Pool,
) -> Status
{
	return physics.simulation_ray_query(world_simulation(world), ray, collector, pool);
}
