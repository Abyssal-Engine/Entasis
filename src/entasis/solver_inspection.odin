package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

@(private)
constraint_impulse_source :: proc "contextless" (
	world: ^World, handle: Constraint_Handle,
) -> (^physics.Simulation, ^physics.Type_Batch, ^physics.Inactive_Constraint_Record, int, int, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, nil, 0, 0, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, nil, 0, 0, .Invalid_Argument;
	}
	simulation := &data.simulation;
	solver := &simulation.solver;
	if handle.value < 0 || int(handle.value) >= int(solver.handle_to_constraint.length)
	{
		return simulation, nil, nil, 0, 0, .Not_Found;
	}
	location := solver.handle_to_constraint.memory[handle.value];
	if location.set_index == 0
	{
		resolved, resolve_status := physics.solver_resolve(solver, handle);
		if resolve_status != .Ok
		{
			return simulation, nil, nil, 0, 0, resolve_status;
		}
		record, lookup_status := physics.constraint_type_registry_lookup(
			&solver.registry, resolved.type_id,
		);
		if lookup_status != .Ok
		{
			return simulation, nil, nil, 0, 0, lookup_status;
		}
		batch := &solver.active_set.batches.memory[resolved.batch_index];
		type_batch_index := int(batch.type_id_to_batch_index[resolved.type_id]);
		if type_batch_index < 0 || type_batch_index >= int(batch.type_batch_count)
		{
			return simulation, nil, nil, 0, 0, .Not_Found;
		}
		return simulation,
		&batch.type_batches.memory[type_batch_index], nil,
		int(resolved.index_in_type_batch),
		int(record.impulse_bundle_size) / size_of(util.F32x8), .Ok;
	}
	if location.set_index > 0
	{
		record_index, resolve_status :=
			physics.island_sleeper_resolve_inactive_constraint_index(
			&simulation.sleeper, handle,
		);
		if resolve_status != .Ok
		{
			return simulation, nil, nil, 0, 0, resolve_status;
		}
		record := &simulation.sleeper.inactive_constraints.memory[record_index];
		return simulation, nil, record, 0, int(record.impulse_count), .Ok;
	}
	return simulation, nil, nil, 0, 0, .Not_Found;
}

// constraint_accumulated_impulses writes all scalar warmstart impulses for one
// active or sleeping constraint. allocation: none
constraint_accumulated_impulses :: proc "contextless" (
	world: ^World, handle: Constraint_Handle, output: []f32,
) -> (written, required: int, status: Status)
{
	_, type_batch, inactive, index, count, source_status :=
		constraint_impulse_source(world, handle);
	if source_status != .Ok
	{
		return 0, 0, source_status;
	}
	required = count;
	written = min(len(output), count);
	if type_batch != nil
	{
		vectors := ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, index));
		lane := index % util.PRODUCTION_LANE_COUNT;
		for impulse_index in 0 ..< written
		{
			output[impulse_index] = simd.extract(vectors[impulse_index], lane);
		}
	}
	else
	{
		for impulse_index in 0 ..< written
		{
			output[impulse_index] = inactive.impulses[impulse_index];
		}
	}
	if len(output) < count
	{
		return written, required, .Capacity_Missing;
	}
	return written, required, .Ok;
}

// constraint_accumulated_impulse_magnitude_squared returns the squared length
// of all scalar accumulated impulses associated with one constraint
constraint_accumulated_impulse_magnitude_squared :: proc "contextless" (
	world: ^World, handle: Constraint_Handle,
) -> (f32, Status)
{
	_, type_batch, inactive, index, count, source_status :=
		constraint_impulse_source(world, handle);
	if source_status != .Ok
	{
		return 0, source_status;
	}
	result := f32(0);
	if type_batch != nil
	{
		vectors := ([^]util.F32x8)(physics.type_batch_impulse_bundle(type_batch, index));
		lane := index % util.PRODUCTION_LANE_COUNT;
		for impulse_index in 0 ..< count
		{
			value := simd.extract(vectors[impulse_index], lane);
			result += value * value;
		}
	}
	else
	{
		for impulse_index in 0 ..< count
		{
			value := inactive.impulses[impulse_index];
			result += value * value;
		}
	}
	return result, .Ok;
}

// constraint_accumulated_impulse_magnitude returns the Euclidean length of all
// scalar accumulated impulses associated with one constraint
constraint_accumulated_impulse_magnitude :: proc "contextless" (
	world: ^World, handle: Constraint_Handle,
) -> (f32, Status)
{
	magnitude_squared, status := constraint_accumulated_impulse_magnitude_squared(
		world, handle,
	);
	if status != .Ok
	{
		return 0, status;
	}
	return f32(math.sqrt(f64(magnitude_squared))), .Ok;
}

// world_scale_active_accumulated_impulses rescales active warmstart impulses.
// use new_effective_dt / old_effective_dt after a timestep or substep change
world_scale_active_accumulated_impulses :: proc "contextless" (
	world: ^World, scale: f32,
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
	return physics.solver_scale_active_accumulated_impulses(
		&data.simulation.solver, scale,
	);
}

// world_scale_accumulated_impulses rescales active and sleeping warmstart
// impulses. this touches all constraint impulse storage
world_scale_accumulated_impulses :: proc "contextless" (
	world: ^World, scale: f32,
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
	return physics.simulation_scale_accumulated_impulses(
		&data.simulation, scale, true,
	);
}
