package precision_queries_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Sweep_Leaf_Observation :: struct
{
	ids: [128]i32,
	count: int,
}
sweep_observe_candidate :: proc "contextless" (user: rawptr, id: i32) -> physics.Query_Filter_Result
{
	state: ^Sweep_Leaf_Observation = (^Sweep_Leaf_Observation)(user);
	state.ids[state.count] = id;
	state.count += 1;
	return .Reject;
}

// exact integer rational interval intersection. no floating slab division,
// nextafter, production clipping helper, or float-to-global query conversion
sweep_rational_leaf_oracle :: proc (
	center: [3]i64, velocity: [3]i64, maximum_t: i64, leaf: physics.Tree_Bounds,
) -> physics.Reference_State
{
	minimum: [3]i64 = {i64(f64(leaf.min.x)*1024), i64(f64(leaf.min.y)*1024), i64(f64(leaf.min.z)*1024)};
	maximum: [3]i64 = {i64(f64(leaf.max.x)*1024), i64(f64(leaf.max.y)*1024), i64(f64(leaf.max.z)*1024)};
	entry_n: i128;
	entry_d: i128 = 1;
	exit_n: i128 = i128(maximum_t);
	exit_d: i128 = 1;
	for axis in 0 ..< 3
	{
		qmin: i64 = center[axis] - 512;
		qmax: i64 = center[axis] + 512;
		speed: i64 = velocity[axis];
		if speed == 0
		{
			if qmin > maximum[axis] || qmax < minimum[axis]
			{
				return .Missing;
			}
			continue;
		}
		lower, upper: i128;
		if speed > 0
		{
			lower = i128(minimum[axis] - qmax);
			upper = i128(maximum[axis] - qmin);
		}
		else
		{
			speed = -speed;
			lower = i128(qmin - maximum[axis]);
			upper = i128(qmax - minimum[axis]);
		}
		denominator: i128 = i128(speed);
		if lower*entry_d > entry_n*denominator
		{
			entry_n = lower;
			entry_d = denominator;
		}
		if upper*exit_d < exit_n*denominator
		{
			exit_n = upper;
			exit_d = denominator;
		}
		if entry_n*exit_d > exit_n*entry_d
		{
			return .Missing;
		}
	}
	return .Present;
}

@(test)
global_sweep_tree_matches_exact_rational_leaf_oracle :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 256);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
	for index in 0 ..< 128
	{
		position: physics.Global_Position = {1e9 + f64(index*3) + 0.125, f64((index*7)%29) - 14.25, f64((index*13)%17) - 8.375};
		if index%2 == 0
		{
			_ = add_body(t, &scene, shape, position, .Kinematic if index%4 == 0 else .Dynamic);
		}
		else
		{
			_ = add_static(t, &scene, shape, position);
		}
	}
	stack: [256]i32;
	broad: ^physics.Broad_Phase = &scene.simulation.broad_phase;
	// include expanded leaves. swept traversal must preserve predictive
	// candidates, even when their shape kernels would subsequently miss
	for index in 0 ..< int(broad.active_tree.leaf_count)
	{
		bounds: physics.Tree_Bounds;
		bounds, _ = physics.tree_get_leaf_bounds(&broad.active_tree, index);
		bounds.min.y -= 2;
		bounds.max.y += 2;
		testing.expect_value(t, physics.tree_update_bounds(&broad.active_tree, index, bounds), physics.Physics_Status.Ok);
	}
	observed: Sweep_Leaf_Observation;
	comparisons, candidates: int;
	for query_index in 0 ..< 1024
	{
		center: [3]i64 = {1_000_000_000*1024 + i64((query_index*37)%501 - 49)*1024 + i64(query_index%5 - 2),
			i64((query_index*17)%43 - 21)*1024 + 512, i64((query_index*11)%31 - 15)*1024 - 512};
		motion: [3]i64 = {i64((query_index*7)%129 - 64)*1024, i64((query_index*3)%17 - 8)*1024, i64(query_index%9 - 4)*1024};
		if query_index%2 == 0
		{
			index: int = (query_index/2)%128;
			center = {1_000_000_000*1024 + i64(index*3)*1024 + 128,
				i64((index*7)%29 - 14)*1024 - 256, i64((index*13)%17 - 8)*1024 - 384};
			motion[1] = 0;
			motion[2] = 0;
		}
		maximum_t: i64 = i64(query_index%4 + 1);
		pose: physics.Global_Pose = {position={f64(center[0])/1024, f64(center[1])/1024, f64(center[2])/1024}, orientation=util.quaternion_identity()};
		velocity: physics.Body_Velocity = {linear={f32(motion[0])/1024, f32(motion[1])/1024, f32(motion[2])/1024}};
		testing.expect_value(t, physics.world_precision_sweep_admit(scene.simulation, pose, velocity, f32(maximum_t), stack[:]), physics.Physics_Status.Ok);
		observed.count = 0;
		count, status := physics.simulation_global_sweep_query(scene.simulation, shape, pose, velocity, f32(maximum_t),
			1e-5, 1e-5, 25, .All, nil, stack[:], &scene.pool, {allow_target=sweep_observe_candidate}, &observed);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, count, 0);
		expected: int;
		for population in 0 ..< 2
		{
			tree: ^physics.Tree = &broad.active_tree if population == 0 else &broad.static_tree;
			leaves: util.Buffer(physics.Collidable_Reference) = broad.active_leaves if population == 0 else broad.static_leaves;
			for index in 0 ..< int(tree.leaf_count)
			{
				leaf: physics.Tree_Bounds;
				leaf, _ = physics.tree_get_leaf_bounds(tree, index);
				result: physics.Reference_State = sweep_rational_leaf_oracle(center, motion, maximum_t, leaf);
				matches: int;
				for id in observed.ids[:observed.count]
				{
					if id == i32(leaves.memory[index].packed)
					{
						matches += 1;
					}
				}
				testing.expectf(t, matches == (1 if result == .Present else 0), "query %v population %v leaf %v", query_index, population, index);
				if result == .Present
				{
					expected += 1;
				}
				comparisons += 1;
			}
		}
		testing.expect_value(t, observed.count, expected);
		candidates += expected;
	}
	testing.expect_value(t, comparisons, 131_072);
	testing.expectf(t, candidates >= 512, "expected at least 512 initial-overlap candidates; got %v", candidates);
}
