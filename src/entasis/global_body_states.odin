#+vet !unused-imports

package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	// one handle resolution reads the description and authoritative dense column
	// together. the simulation must remain idle and immutable for the whole read
	body_get_global_state_ready :: proc "contextless" (
		simulation: ^physics.Simulation, handle: Body_Handle,
	) -> (Global_Body_State, Status)
	{
		location: physics.Body_Memory_Location;
		status: Status;
		location, status = physics.bodies_resolve(&simulation.bodies, handle);
		if status != .Ok
		{
			return {}, status;
		}
		set: ^physics.Body_Set = &simulation.bodies.sets.memory[location.set_index];
		description: Body_Description;
		description, status = physics.body_set_get_description(set, int(location.index));
		if status != .Ok
		{
			return {}, status;
		}
		return {
			pose={position=set.global_positions.memory[location.index], orientation=description.pose.orientation},
			velocity=description.velocity, local_inertia=description.local_inertia,
			collidable=description.collidable, activity=description.activity,
		}, .Ok;
	}

	body_get_global_state :: proc "contextless" (
		world: ^World, handle: Body_Handle,
	) -> (Global_Body_State, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		return body_get_global_state_ready(&data.simulation, handle);
	}

	global_body_state_description :: proc "contextless" (state: Global_Body_State) -> Body_Description
	{
		return {
			pose={position=physics.world_precision_position_cache(state.pose.position), orientation=state.pose.orientation},
			velocity=state.velocity, local_inertia=state.local_inertia,
			collidable=state.collidable, activity=state.activity,
		};
	}

	// all fields are replaced, high position is never inferred from float equality
	// hidden activity counters, impulses and pending targets are not restored
	body_apply_global_state :: proc (world: ^World, handle: Body_Handle, state: Global_Body_State) -> Status
	{
		return body_apply_global(world, handle, global_body_state_description(state), state.pose.position);
	}

	body_add_global_state :: proc (world: ^World, state: Global_Body_State) -> (Body_Handle, Status)
	{
		return body_add_global(world, global_body_state_description(state), state.pose.position);
	}

	static_get_global_state_ready :: proc "contextless" (
		simulation: ^physics.Simulation, handle: Static_Handle,
	) -> (Global_Static_State, Status)
	{
		index: int;
		status: Status;
		index, status = physics.statics_resolve(&simulation.statics, handle);
		if status != .Ok
		{
			return {}, status;
		}
		stored: ^physics.Static = &simulation.statics.statics.memory[index];
		return {
			pose={position=simulation.statics.global_positions.memory[index], orientation=stored.pose.orientation},
			shape=stored.shape, continuity=stored.continuity,
		}, .Ok;
	}

	static_get_global_state :: proc "contextless" (
		world: ^World, handle: Static_Handle,
	) -> (Global_Static_State, Status)
	{
		data: ^world_data;
		status: Status;
		data, status = query_world_data(world);
		if status != .Ok
		{
			return {}, status;
		}
		return static_get_global_state_ready(&data.simulation, handle);
	}

	global_static_state_description :: proc "contextless" (state: Global_Static_State) -> Static_Description
	{
		return {
			pose={position=physics.world_precision_position_cache(state.pose.position), orientation=state.pose.orientation},
			shape=state.shape, continuity=state.continuity,
		};
	}

	static_apply_global_state :: proc (
		world: ^World, handle: Static_Handle, state: Global_Static_State,
		awakening: Awakening_Policy = .Overlaps,
	) -> Status
	{
		return static_apply_global(world, handle, global_static_state_description(state), state.pose.position, awakening);
	}

	static_add_global_state :: proc (
		world: ^World, state: Global_Static_State, awakening: Awakening_Policy = .Overlaps,
	) -> (Static_Handle, Status)
	{
		return static_add_global(world, global_static_state_description(state), state.pose.position, awakening);
	}
}
