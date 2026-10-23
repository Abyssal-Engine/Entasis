package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_volume_closed_boundaries_are_not_rounded_to_float :: proc(t: ^testing.T)
{
	cases: int;
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		shape: physics.Typed_Index = add_shape(t, &scene, .Box);
		handle: physics.Static_Handle = add_static(t, &scene, shape, {origin, -origin, origin});
		reference: physics.Collidable_Reference;
		reference, _ = physics.collidable_reference_create(.Static, int(handle.value));
		bounds: physics.Tree_Bounds;
		bounds, _ = physics.tree_get_leaf_bounds(&scene.simulation.broad_phase.static_tree, 0);
		minimum: [3]f64 = {f64(bounds.min.x), f64(bounds.min.y), f64(bounds.min.z)};
		maximum: [3]f64 = {f64(bounds.max.x), f64(bounds.max.y), f64(bounds.max.z)};
		for axis in 0 ..< 3
		{
			for side in 0 ..< 2
			{
				for displacement in ([3]f64{-0.002, 0, 0.002})
				{
					point: [3]f64;
					for component in 0 ..< 3
					{
						point[component] = (minimum[component] + maximum[component]) * 0.5;
					}
					point[axis] = minimum[axis] - displacement if side == 0 else maximum[axis] + displacement;
					query: entasis.Global_Bounds = {min={point[0], point[1], point[2]}, max={point[0], point[1], point[2]}};
					stack: [1]i32;
					hits: [2]entasis.Volume_Hit;
					count: int;
					status: entasis.Status;
					count, status = entasis.volume_global_all_with_context(&scene.query, query, hits[:], stack[:]);
					testing.expect_value(t, status, entasis.Status.Ok);
					testing.expect_value(t, count, 0 if displacement > 0 else 1);
					if count > 0
					{
						testing.expect_value(t, hits[0].collidable, reference);
					}
					cases += 1;
				}
			}
		}
		scene_dispose(t, &scene);
	}
	testing.expect_value(t, cases, 72);
}

@(test)
global_volume_matches_independent_leaf_scan_and_preserves_expansion :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 256);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	for index in 0 ..< 128
	{
		position: physics.Global_Position = {1e9 + f64(index*3) + 0.125, f64((index*7)%29) - 14.25, f64((index*13)%17) - 8.375};
		if index % 2 == 0
		{
			_ = add_body(t, &scene, shape, position, .Kinematic if index % 4 == 0 else .Dynamic);
		}
		else
		{
			_ = add_static(t, &scene, shape, position);
		}
	}
	// expand real broad-phase leaves beyond geometry, like predicted motion
	// volume queries must preserve these candidates instead of retesting shapes
	broad: ^physics.Broad_Phase = &scene.simulation.broad_phase;
	for index in 0 ..< int(broad.active_tree.leaf_count)
	{
		bounds: physics.Tree_Bounds;
		bounds, _ = physics.tree_get_leaf_bounds(&broad.active_tree, index);
		bounds.min.y -= 2;
		bounds.max.y += 2;
		testing.expect_value(t, physics.tree_update_bounds(&broad.active_tree, index, bounds), physics.Physics_Status.Ok);
	}
	stack: [256]i32;
	hits: [128]entasis.Volume_Hit;
	for query_index in 0 ..< 1024
	{
		x: f64 = 1e9 + f64((query_index*37)%601) - 97.002;
		y: f64 = f64((query_index*17)%43) - 21.002;
		z: f64 = f64((query_index*11)%31) - 15.002;
		bounds: entasis.Global_Bounds = {min={x, y, z}, max={x + 63.996, y + 4.004, z + 4.004}};
		count: int;
		status: entasis.Status;
		count, status = entasis.volume_global_all_with_context(&scene.query, bounds, hits[:], stack[:]);
		testing.expect_value(t, status, entasis.Status.Ok);
		expected_count: int;
		for population in 0 ..< 2
		{
			tree: ^physics.Tree = &broad.active_tree if population == 0 else &broad.static_tree;
			leaves: util.Buffer(physics.Collidable_Reference) = broad.active_leaves if population == 0 else broad.static_leaves;
			for index in 0 ..< int(tree.leaf_count)
			{
				leaf: physics.Tree_Bounds;
				leaf, _ = physics.tree_get_leaf_bounds(tree, index);
				if f64(leaf.min.x) <= bounds.max.x && f64(leaf.max.x) >= bounds.min.x &&
					f64(leaf.min.y) <= bounds.max.y && f64(leaf.max.y) >= bounds.min.y &&
					f64(leaf.min.z) <= bounds.max.z && f64(leaf.max.z) >= bounds.min.z
				{
					expected_count += 1;
					matches: int;
					for hit in hits[:count]
					{
						if hit.collidable == leaves.memory[index]
						{
							matches += 1;
						}
					}
					testing.expect_value(t, matches, 1);
				}
			}
		}
		testing.expectf(t, count == expected_count, "query %v got %v expected %v", query_index, count, expected_count);
	}
}
