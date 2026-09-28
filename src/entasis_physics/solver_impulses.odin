// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

// solver_scale_active_accumulated_impulses scales all live active-set warmstart
// impulses. it is intended for effective timestep duration changes
solver_scale_active_accumulated_impulses :: proc "contextless" (
	solver: ^Solver, scale: f32,
) -> Physics_Status
{
	if solver == nil || solver.state != .Ready
	{
		return .Disposed;
	}
	set := &solver.active_set;
	for batch_index in 0 ..< int(set.batch_count)
	{
		batch := &set.batches.memory[batch_index];
		for type_batch_index in 0 ..< int(batch.type_batch_count)
		{
			type_batch := &batch.type_batches.memory[type_batch_index];
			bundle_count := (int(type_batch.count) + util.PRODUCTION_LANE_COUNT - 1) /
				util.PRODUCTION_LANE_COUNT;
			float_count := bundle_count * int(type_batch.impulse_bundle_size) / size_of(f32);
			values := ([^]f32)(type_batch.accumulated_impulses.memory);
			for value_index in 0 ..< float_count
			{
				values[value_index] *= scale;
			}
		}
	}
	return .Ok;
}

// simulation_scale_accumulated_impulses scales active impulses and optionally
// the scalar impulses stored with sleeping constraints
simulation_scale_accumulated_impulses :: proc "contextless" (
	simulation: ^Simulation, scale: f32, include_inactive: bool = true,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready
	{
		return .Disposed;
	}
	status := solver_scale_active_accumulated_impulses(&simulation.solver, scale);
	if status != .Ok || !include_inactive
	{
		return status;
	}
	for record_index in 0 ..< simulation.sleeper.inactive_constraint_count
	{
		record := &simulation.sleeper.inactive_constraints.memory[record_index];
		for impulse_index in 0 ..< int(record.impulse_count)
		{
			record.impulses[impulse_index] *= scale;
		}
	}
	return .Ok;
}
