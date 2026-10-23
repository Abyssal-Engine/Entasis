package precision_collisions_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_parallel_tree_job_budget_preserves_complete_pair_sets :: proc(t: ^testing.T)
{
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	counts: [6]int = {0, 1, 2, 7, 31, 97};
	for count in counts
	{
		scene: Scene;
		scene_initialize(t, &scene);
		shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
		for index in 0 ..< count
		{
			position: physics.Global_Position = {1e9 + f64(index) / 1024, -1e9 + 0.125, 1e9 + 0.25};
			_ = add_body(t, &scene, shape, position);
			_ = add_static(t, &scene, shape, position);
		}
		broad: ^physics.Broad_Phase = &scene.simulation.broad_phase;
		original_limit: int = broad.query_workspace.maximum_jobs;
		output: util.Buffer(physics.Tree_Leaf_Pair);
		combined: util.Buffer(physics.Broad_Phase_Pair);
		memory_status: util.Memory_Status;
		output, memory_status = util.buffer_pool_take_at_least(&scene.pool, physics.Tree_Leaf_Pair, 97 * 97);
		testing.expect_value(t, memory_status, util.Memory_Status.Ok);
		combined, memory_status = util.buffer_pool_take_at_least(&scene.pool, physics.Broad_Phase_Pair, 97 * 97 * 2);
		testing.expect_value(t, memory_status, util.Memory_Status.Ok);
		budgets: [7]int = {5, 7, 16, 63, 64, 127, original_limit};
		allocations: i64 = scene.tracker.total_allocation_count;
		bytes: i64 = scene.tracker.current_memory_allocated;
		for budget in budgets
		{
			// force tiny bounded schedules, including one self-subtree job and
			// four intertree root jobs. each retained job visits its whole subtree
			broad.query_workspace.maximum_jobs = max(1, budget - 4);
			found: int;
			status: physics.Physics_Status;
			found, status = physics.tree_self_query_parallel(&broad.active_tree, boundary, &broad.query_workspace, output);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, found, count * (count - 1) / 2);
			cursor: int;
			for a in 0 ..< count
			{
				for b in a + 1 ..< count
				{
					testing.expect_value(t, output.memory[cursor], physics.Tree_Leaf_Pair{i32(a), i32(b)});
					cursor += 1;
				}
			}
			broad.query_workspace.maximum_jobs = max(4, budget - 1);
			found, status = physics.tree_intertree_query_parallel(&broad.active_tree, &broad.static_tree, boundary, &broad.query_workspace, output);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, found, count * count);
			cursor = 0;
			for a in 0 ..< count
			{
				for b in 0 ..< count
				{
					testing.expect_value(t, output.memory[cursor], physics.Tree_Leaf_Pair{i32(a), i32(b)});
					cursor += 1;
				}
			}
			broad.query_workspace.maximum_jobs = budget;
			found, status = physics.broad_phase_find_pairs(broad, combined, boundary);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, found, count * (count - 1) / 2 + count * count);
			seen: [97 * 97 * 2]u8;
			for index in 0 ..< found
			{
				pair: physics.Broad_Phase_Pair = combined.memory[index];
				a: int = int(physics.collidable_reference_raw_handle(pair.a));
				b: int = int(physics.collidable_reference_raw_handle(pair.b));
				offset: int;
				if physics.collidable_reference_mobility(pair.b) == .Static
				{
					offset = 97 * 97;
				}
				else
				{
					testing.expect(t, a != b);
					if a > b
					{
						a, b = b, a;
					}
				}
				testing.expect(t, a >= 0 && a < count && b >= 0 && b < count);
				slot: int = offset + a * 97 + b;
				testing.expect_value(t, seen[slot], u8(0));
				seen[slot] += 1;
			}
		}
		testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
		testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
		broad.query_workspace.maximum_jobs = original_limit;
		testing.expect_value(t, util.buffer_pool_return(&scene.pool, &combined), util.Memory_Status.Ok);
		testing.expect_value(t, util.buffer_pool_return(&scene.pool, &output), util.Memory_Status.Ok);
		scene_dispose(t, &scene);
	}
}
