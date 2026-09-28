package spatial_query_trace

import entasis "entasis:entasis"

Observation_State :: enum u8
{
	Miss, Hit,
}
Ray_Observation :: struct
{
	state: Observation_State,
	hit: entasis.Ray_Hit,
}
Sweep_Observation :: struct
{
	state: Observation_State,
	hit: entasis.Sweep_Hit,
}
Overlap_Observation :: struct
{
	state: Observation_State,
	target: entasis.Collidable_Reference,
}
Query_Observations :: struct
{
	rays: []Ray_Observation,
	sweeps: []Sweep_Observation,
	overlaps: []Overlap_Observation,
}

// optional output is written only by validation, outside the timed kernels
observations_create :: proc(owner: ^Benchmark_Owner, families: bit_set[Query_Family]) -> Case_Status
{
	if .Ray in families
	{
		owner.observations.rays = make([]Ray_Observation, len(owner.ray_inputs));
		if owner.observations.rays == nil
		{
			return .Allocation_Failed;
		}
	}
	if .Sphere_Cast in families
	{
		owner.observations.sweeps = make([]Sweep_Observation, len(owner.sphere_cast_inputs));
		if owner.observations.sweeps == nil
		{
			return .Allocation_Failed;
		}
	}
	if .Overlap in families
	{
		owner.observations.overlaps = make([]Overlap_Observation, len(owner.overlap_inputs));
		if owner.observations.overlaps == nil
		{
			return .Allocation_Failed;
		}
	}
	return .Ok;
}
