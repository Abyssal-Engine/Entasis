package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(private)
body_constraint_list :: #force_inline proc "contextless" (
	world: ^World, body: Body_Handle,
) -> (^physics.Simulation, ^physics.Body_Set, ^util.Quick_List(physics.Body_Constraint_Reference), int, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, nil, 0, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, nil, 0, .Invalid_Argument;
	}
	location, status := physics.bodies_resolve(&data.simulation.bodies, body);
	if status != .Ok
	{
		return &data.simulation, nil, nil, 0, status;
	}
	set := &data.simulation.bodies.sets.memory[location.set_index];
	list := &set.constraints.memory[location.index];
	return &data.simulation, set, list, list.count, .Ok;
}

// body_constraint_count returns the number of constraints connected to a body
body_constraint_count :: proc "contextless" (
	world: ^World, body: Body_Handle,
) -> (int, Status)
{
	_, _, _, count, status := body_constraint_list(world, body);
	return count, status;
}

// body_constraints writes the body's connected constraint handles in the
// runtime body-list order. if output is too small, the prefix is written and
// Capacity_Missing is returned with the required count. allocation: none
body_constraints :: proc "contextless" (
	world: ^World, body: Body_Handle, output: []Constraint_Handle,
) -> (written, required: int, status: Status)
{
	_, _, references, count, resolve_status := body_constraint_list(world, body);
	if resolve_status != .Ok
	{
		return 0, 0, resolve_status;
	}
	required = count;
	written = min(len(output), count);
	for index in 0 ..< written
	{
		output[index] = references.span.memory[index].connecting_constraint_handle;
	}
	if len(output) < count
	{
		return written, required, .Capacity_Missing;
	}
	return written, required, .Ok;
}

// constraint_connected_bodies writes the body handles referenced by one active
// or sleeping constraint. allocation: none
constraint_connected_bodies :: proc "contextless" (
	world: ^World, handle: Constraint_Handle, output: []Body_Handle,
) -> (written, required: int, status: Status)
{
	info, inspect_status := constraint_inspect(world, handle);
	if inspect_status != .Ok
	{
		return 0, 0, inspect_status;
	}
	required = int(info.body_count);
	written = min(len(output), required);
	for index in 0 ..< written
	{
		output[index] = info.bodies[index];
	}
	if len(output) < required
	{
		return written, required, .Capacity_Missing;
	}
	return written, required, .Ok;
}

// body_connected_bodies writes one entry for every connected constraint/body
// edge, excluding the source body. duplicate body handles are preserved when
// multiple constraints connect the same pair. allocation: none
body_connected_bodies :: proc "contextless" (
	world: ^World, body: Body_Handle, output: []Body_Handle,
) -> (written, required: int, status: Status)
{
	simulation, _, references, count, resolve_status := body_constraint_list(world, body);
	if resolve_status != .Ok
	{
		return 0, 0, resolve_status;
	}
	for constraint_index in 0 ..< count
	{
		info, inspect_status := constraint_inspect_ready(
			simulation, references.span.memory[constraint_index].connecting_constraint_handle,
		);
		if inspect_status != .Ok
		{
			return written, required, inspect_status;
		}
		for body_index in 0 ..< int(info.body_count)
		{
			connected := info.bodies[body_index];
			if connected.value == body.value
			{
				continue;
			}
			if written < len(output)
			{
				output[written] = connected;
			}
			written += 1;
		}
	}
	required = written;
	if len(output) < required
	{
		return len(output), required, .Capacity_Missing;
	}
	return required, required, .Ok;
}
