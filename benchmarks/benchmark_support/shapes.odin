package benchmark_support

import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"

register_uncached_shape :: proc(world: ^entasis.World, kind: Shape, size: [3]f32) -> (entasis.Shape_Handle, entasis.Status)
{
	switch kind
	{
		case .Box: return entasis.shape_add(world, entasis.box(size[0], size[1], size[2]));
		case .Sphere: return entasis.shape_add(world, entasis.sphere(size[0] / 2));
		case .Capsule: return entasis.shape_add(world, entasis.capsule(size[0] / 2, size[1] - size[0]));
		case .Cylinder: return entasis.shape_add(world, entasis.cylinder(size[0] / 2, size[1]));
		case .Hull:
		cooker: cooking.Cooking_Context;
		if cooking.cooking_context_init(&cooker, 65_536) != .Ok
		{
			return {}, .Capacity_Missing;
		}
		x, y, z := size[0] / 2, size[1] / 2, size[2] / 2;
		points := [8]entasis.Vector3{{-x, -y, -z}, {x, -y, -z}, {-x, y, -z}, {x, y, -z}, {-x, -y, z}, {x, -y, z}, {-x, y, z}, {x, y, z}};
		cooked, status := cooking.cook_hull(&cooker, points[:]);
		shape: entasis.Shape_Handle;
		result := entasis.Status.Ok;
		if status == .Ok
		{
			shape, result = cooking.cooked_hull_import(world, &cooked);
			if cooking.cooked_hull_destroy(&cooked) != .Ok && result == .Ok
			{
				result = .Invalid_Description;
			}
		}
		else
		{
			result = .Invalid_Argument;
		}
		if cooking.cooking_context_destroy(&cooker) != .Ok && result == .Ok
		{
			result = .Invalid_Description;
		}
		return shape, result;
	}
	unreachable();
}

register_dynamic :: proc(world: ^entasis.World, cache: ^Shape_Cache, kind: Shape, size: [3]f32, density: f32) -> (entasis.Shape_Handle, entasis.Body_Inertia, entasis.Status)
{
	shape, status := register_shape(world, cache, kind, size);
	if status != .Ok
	{
		return {}, {}, status;
	}
	inertia, inertia_status := entasis.shape_registered_inertia(world, shape, density * shape_volume(kind, size));
	return shape, inertia, inertia_status;
}

// each workload registers at most four distinct shapes per world. the cache
// shares coincident custom cuboids as well as repeated floor/body instances
Shape_Entry :: struct
{
	kind: Shape,
	size: [3]f32,
	handle: entasis.Shape_Handle,
}
Shape_Cache :: struct
{
	entries: [4]Shape_Entry,
	count: int,
}
register_shape :: proc(world: ^entasis.World, cache: ^Shape_Cache, kind: Shape, size: [3]f32) -> (entasis.Shape_Handle, entasis.Status)
{
	for entry in cache.entries[:cache.count]
	{
		if entry.kind == kind && entry.size == size
		{
			return entry.handle, .Ok;
		}
	}
	handle, status := register_uncached_shape(world, kind, size);
	if status == .Ok
	{
		cache.entries[cache.count] = {kind, size, handle};
		cache.count += 1;
	}
	return handle, status;
}
