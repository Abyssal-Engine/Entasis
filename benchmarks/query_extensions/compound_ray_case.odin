package query_extensions

import "base:runtime"
import "core:fmt"
import "core:strings"
import e "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

COMPOUND_RAY_WORKER_LPS :: [32]u32{
	18, 16, 20, 22, 24, 26, 28, 30, 0, 2, 4, 6, 8, 10, 12, 14,
	19, 17, 21, 23, 25, 27, 29, 31, 1, 3, 5, 7, 9, 11, 13, 15,
};

compound_ray_dimensions :: proc "contextless" (cell: string) -> (int, int)
{
	switch cell
	{
	case "compound-8", "nested-compound-8":
		return 8, 4;
	case "big-128":
		return 128, 16;
	case "big-8192", "nested-big-8192":
		return 8192, 128;
	}
	return 0, 0;
}

compound_ray_prepare :: proc(fixture: ^Hierarchy_Case, cell: string)
{
	child_count, columns: int;
	child_count, columns = compound_ray_dimensions(cell);
	children: []e.Compound_Child;
	allocation_error: runtime.Allocator_Error;
	children, allocation_error = make([]e.Compound_Child, child_count);
	if allocation_error != nil
	{
		panic("compound ray child storage");
	}
	defer delete(children);
	box: e.Shape_Handle;
	status: e.Status;
	box, status = e.shape_add(&fixture.world, e.box(1, 1, 1));
	support.extension_require(status, "compound ray box");
	rotation: e.Quaternion = {0, 0.6, 0, 0.8};
	fixture.target_pose = e.pose({1, 0, -2}, rotation);
	for &child, index in children
	{
		orientation: e.Quaternion = util.quaternion_identity() if index%2 == 0 else rotation;
		child = e.compound_child(box, e.pose({f32(2*(index%columns)), 0, f32(2*(index/columns))}, orientation));
	}
	if cell == "compound-8" || cell == "nested-compound-8"
	{
		fixture.target, status = e.shape_import_compound(&fixture.world, children);
	}
	else
	{
		fixture.target, status = e.shape_import_big_compound(&fixture.world, children);
	}
	support.extension_require(status, "compound ray import");
	if strings.has_prefix(cell, "nested-")
	{
		outer: [2]e.Compound_Child = {
			e.compound_child(fixture.target, e.pose()),
			e.compound_child(fixture.target, e.pose({0, -2, 0})),
		};
		fixture.target, status = e.shape_import_compound(&fixture.world, outer[:]);
		support.extension_require(status, "compound ray nested import");
	}
	for &ray, index in fixture.rays
	{
		child: int = ((index/2)*73)%child_count;
		local: e.Vector3 = {f32(2*(child%columns)), 0, f32(2*(child/columns))};
		if index%2 != 0
		{
			local.x += f32(4*columns);
		}
		origin: e.Vector3 = util.vector3_add(util.quaternion_transform(local, rotation), fixture.target_pose.position);
		origin.y = 4.5;
		ray = e.ray(origin, {0, -1, 0}, 8);
	}
}

compound_ray_validate :: proc(fixture: ^Hierarchy_Case, cell: string) -> (u64, f64, e.Status)
{
	child_count: int;
	child_count, _ = compound_ray_dimensions(cell);
	hits: u64;
	for observation, index in fixture.observations
	{
		expected_count: int = 0;
		expected_status: e.Status = .Ok;
		if index%2 == 0
		{
			expected_count = 2 if fixture.operation == "ray-all" else 1;
		}
		else if fixture.operation == "ray-closest"
		{
			expected_status = .Not_Found;
		}
		if observation.status != expected_status || observation.count != expected_count
		{
			fmt.eprintln("compound ray independent status/count oracle");
			return 0, 0, .Invalid_Description;
		}
		if expected_count > 0 && fixture.operation != "ray-any"
		{
			expected_child: i32 = i32(((index/2)*73)%child_count);
			if strings.has_prefix(cell, "nested-")
			{
				expected_child = 0;
			}
			if observation.child != expected_child || observation.t != 4 || observation.normal != (e.Vector3{0, 1, 0})
			{
				fmt.eprintln("compound ray independent top face/child oracle");
			return 0, 0, .Invalid_Description;
			}
			if expected_count == 2 && (observation.second_t != 6 || observation.second_child != 1 || observation.second_normal != (e.Vector3{0, 1, 0}))
			{
				fmt.eprintln("compound ray independent second top face/child oracle");
			return 0, 0, .Invalid_Description;
			}
		}
		hits += u64(observation.count)*HIERARCHY_PASSES;
	}
	return hits, f64(hits), .Ok;
}

