package query_extensions

import "base:runtime"
import "core:math"
import e "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

hull_ray_sides :: proc "contextless" (cell: string) -> int
{
	switch cell
	{
	case "hull-32":
		return 16;
	case "hull-256":
		return 128;
	}
	return 0;
}

hull_ray_prepare :: proc(fixture: ^Hierarchy_Case, sides: int)
{
	points: []util.Vector3;
	starts, indices: []i32;
	point_error, start_error, index_error: runtime.Allocator_Error;
	points, point_error = make([]util.Vector3, 2*sides);
	defer delete(points);
	starts, start_error = make([]i32, sides+2);
	defer delete(starts);
	indices, index_error = make([]i32, 6*sides);
	defer delete(indices);
	if point_error != nil || start_error != nil || index_error != nil
	{
		panic("hull ray geometry storage");
	}
	for index in 0 ..< sides
	{
		angle: f32 = f32(2*math.PI)*f32(index)/f32(sides);
		points[index] = {math.cos(angle), -2, math.sin(angle)};
		points[sides+index] = {points[index].x, 0, points[index].z};
		indices[index] = i32(index);
		indices[sides+index] = i32(2*sides-1-index);
	}
	starts[0], starts[1] = 0, i32(sides);
	for index in 0 ..< sides
	{
		start: int = 2*sides+4*index;
		next: int = (index+1)%sides;
		starts[index+2] = i32(start);
		indices[start], indices[start+1], indices[start+2], indices[start+3] = i32(index), i32(sides+index), i32(sides+next), i32(next);
	}
	pool: util.Buffer_Pool;
	if util.buffer_pool_initialize(&pool, 128) != .Ok
	{
		panic("hull ray preparation pool");
	}
	defer util.buffer_pool_dispose(&pool);
	hull: physics.Convex_Hull;
	support.extension_require(physics.convex_hull_create(&hull, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &pool), "hull ray topology");
	defer physics.convex_hull_dispose(&hull, &pool);
	status: e.Status;
	fixture.target, status = e.shape_import_convex_hull(&fixture.world, &hull);
	support.extension_require(status, "hull ray import");
	for &ray, index in fixture.rays
	{
		// binary oblique rays enter the top plane at t=4, away from every side
		x: f32 = f32((index/2)%4)*0.125-0.1875;
		z: f32 = f32((index/8)%4)*0.125-0.1875;
		if index%2 != 0
		{
			x += 4;
		}
		ray = e.ray({x-0.5, 4, z-0.25}, {0.125, -1, 0.0625}, 8);
	}
}

hull_ray_validate :: proc(fixture: ^Hierarchy_Case) -> (u64, f64, e.Status)
{
	hits: u64;
	for observation, index in fixture.observations
	{
		expected_count: int = 1 if index%2 == 0 else 0;
		expected_status: e.Status = .Not_Found if index%2 != 0 && fixture.operation == "ray-closest" else .Ok;
		if observation.status != expected_status || observation.count != expected_count
		{
			return 0, 0, .Invalid_Description;
		}
		if expected_count != 0 && fixture.operation != "ray-any" &&
			(observation.child != 0 || !(abs(observation.t-4) <= 0.000002 &&
			abs(observation.normal.x) <= 0.000002 && abs(observation.normal.y-1) <= 0.000002 && abs(observation.normal.z) <= 0.000002))
		{
			return 0, 0, .Invalid_Description;
		}
		hits += u64(observation.count)*HIERARCHY_PASSES;
	}
	return hits, f64(hits), .Ok;
}
