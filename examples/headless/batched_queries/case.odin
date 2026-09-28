package batched_queries

import "core:math"
import entasis "entasis:entasis"

STEPS :: 2;
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	ray: entasis.Ray,
	bounds: entasis.Bounding_Box,
	queries: [4]entasis.Query,
	results: [4]entasis.Query_Result,
	ray_hits: [8]entasis.Ray_Hit,
	volume_hits: [8]entasis.Volume_Hit,
	rays: [19]entasis.Query,
	ray_results: [19]entasis.Query_Result,
	tick: int,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	shape: entasis.Shape_Handle;
	shape, status = entasis.shape_add(&c.world, entasis.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	positions: [3]f32 = {0, 3, 6};
	for x in positions
	{
		_, status = entasis.static_add(&c.world, entasis.static_body(shape, entasis.pose({x, 0, 0})), .None);
		if status != .Ok
		{
			return status;
		}
	}
	c.ray = entasis.ray({-3, 0, 0}, {1, 0, 0}, 12);
	c.bounds = {min={-1, -1, -1}, max={7, 1, 1}};
	c.queries = {
		entasis.query_ray_any(c.ray),
		entasis.query_ray_closest(c.ray),
		entasis.query_ray_all(c.ray, entasis.query_output(0, 8)),
		entasis.query_volume_all(c.bounds, entasis.query_output(0, 8)),
	};
	for &query, index in c.rays
	{
		query = entasis.query_ray_closest(entasis.ray({-3, f32(index%3)*0.1, 0}, {1, 0, 0}, 12));
	}
	return .Ok;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	switch c.tick
	{
	case 0:
		scratch: entasis.Query_Scratch = entasis.query_scratch(c.ray_hits[:], nil, c.volume_hits[:]);
		status: entasis.Status = entasis.query_batch(&c.world, c.queries[:], c.results[:], &scratch);
		if status != .Ok
		{
			return status;
		}
		scalar: entasis.Ray_Hit;
		scalar, status = entasis.ray_cast_closest(&c.world, c.ray);
		if status != .Ok || !c.results[0].hit || !c.results[1].hit ||
			c.results[2].count != 3 || c.results[3].count != 3 ||
			math.abs(c.results[1].ray_hit.t-scalar.t) > 1e-5
		{
			return .Invalid_Argument;
		}
	case 1:
		status: entasis.Status = entasis.query_batch(&c.world, c.rays[:], c.ray_results[:], nil);
		if status != .Ok
		{
			return status;
		}
		for query, index in c.rays
		{
			expected: entasis.Ray_Hit;
			expected, status = entasis.ray_cast_closest(&c.world, query.ray_data.ray);
			if status != c.ray_results[index].status || expected.collidable != c.ray_results[index].ray_hit.collidable ||
				math.abs(expected.t-c.ray_results[index].ray_hit.t) > 1e-5
			{
				return .Invalid_Argument;
			}
		}
	case: return .Invalid_Argument;
	}
	c.tick += 1;
	return .Ok;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
