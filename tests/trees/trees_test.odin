package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

MAX_LEAVES :: 96;
MAX_PAIRS :: MAX_LEAVES * MAX_LEAVES;

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 20
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 131072, power), util.Memory_Status.Ok);
	}
}

next_random :: proc(state: ^u32) -> u32
{
	state^ = state^ * 1664525 + 1013904223;
	return state^;
}

random_bounds :: proc(state: ^u32, index: int) -> util.Bounding_Box
{
	x := f32(i32(next_random(state) % 2000) - 1000) * 0.01;
	y := f32(i32(next_random(state) % 2000) - 1000) * 0.01;
	z := f32(i32(next_random(state) % 2000) - 1000) * 0.01;
	extent := f32(1 + next_random(state) % 80) * 0.01;
	// index bias prevents the randomized fixture from collapsing into identical bounds
	x += f32(index) * 0.0001;
	return {min={x - extent, y - extent, z - extent}, max={x + extent, y + extent, z + extent}};
}

pair_key :: proc "contextless" (a, b: int) -> int
{
	if a < b
	{
		return a * MAX_LEAVES + b;
	}
	return b * MAX_LEAVES + a;
}

tree_hash :: proc "contextless" (tree: ^physics.Tree) -> u64
{
	hash := u64(1469598103934665603);
	for node_index in 0 ..< tree.node_count
	{
		node := tree.nodes.memory[node_index];
		values := [6]u32{
			transmute(u32)node.a.min.x, transmute(u32)node.a.max.x, u32(node.a.index),
			transmute(u32)node.b.min.x, transmute(u32)node.b.max.x, u32(node.b.index),
		};
		for value in values
		{
			hash = (hash ~ u64(value)) * 1099511628211;
		}
	}
	return hash;
}

pair_hash :: proc "contextless" (pairs: [^]physics.Tree_Leaf_Pair, count: int) -> u64
{
	hash := u64(1469598103934665603);
	for index in 0 ..< count
	{
		hash = (hash ~ u64(u32(pairs[index].a))) * 1099511628211;
		hash = (hash ~ u64(u32(pairs[index].b))) * 1099511628211;
	}
	return hash;
}

validate_pinned_binned_builder_strategies :: proc(t: ^testing.T)
{
	MAX_SOURCE_FIXTURE_LEAVES :: 1025;
	counts := [4]int{48, 65, 512, 1025};
	seeds := [4]u32{1, 1, 3, 5};
	expected_root_a_counts := [4]i32{26, 39, 249, 555};
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	for power in 0 ..= 20
	{
		testing.expect_value(
			t,
			util.buffer_pool_ensure_capacity_for_power(&pool, 1048576, power),
			util.Memory_Status.Ok
		);
	}
	defer util.buffer_pool_dispose(&pool);
	bounds: [MAX_SOURCE_FIXTURE_LEAVES]util.Bounding_Box;
	for fixture_index in 0 ..< len(counts)
	{
		count := counts[fixture_index];
		random_state := seeds[fixture_index];
		for index in 0 ..< count
		{
			bounds[index] = random_bounds(&random_state, index);
		}
		tree: physics.Tree;
		testing.expect_value(t, physics.tree_initialize(&tree, count, &pool), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_build(&tree, &bounds[0], count), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
		testing.expect_value(t, tree.nodes.memory[0].a.leaf_count, expected_root_a_counts[fixture_index]);
		testing.expect_value(t, tree.nodes.memory[0].b.leaf_count, i32(count) - expected_root_a_counts[fixture_index]);
		expected_hash := tree_hash(&tree);
		testing.expect_value(t, physics.tree_build(&tree, &bounds[0], count), physics.Physics_Status.Ok);
		testing.expect_value(t, tree_hash(&tree), expected_hash);
		testing.expect_value(t, physics.tree_dispose(&tree), physics.Physics_Status.Ok);
	}
}

@(test)
randomized_add_update_refit_refine_query_and_remove_match_brute_force :: proc(t: ^testing.T)
{
	validate_pinned_binned_builder_strategies(t);
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, MAX_LEAVES, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	model: [MAX_LEAVES]util.Bounding_Box;
	random_state := u32(0x39ac_72e1);
	count := 72;
	for index in 0 ..< count
	{
		model[index] = random_bounds(&random_state, index);
		leaf_index, status := physics.tree_add(&tree, model[index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, leaf_index, index);
	}
	testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);

	for iteration in 0 ..< 512
	{
		leaf_index := int(next_random(&random_state) % u32(count));
		model[leaf_index] = random_bounds(&random_state, leaf_index + iteration);
		testing.expect_value(
			t,
			physics.tree_update_bounds(&tree, leaf_index, model[leaf_index]),
			physics.Physics_Status.Ok
		);
		query := random_bounds(&random_state, iteration);
		output_values: [MAX_LEAVES]i32;
		output := util.Buffer(i32){memory=&output_values[0], length=i32(len(output_values)), id=-1};
		output_count, query_status := physics.tree_query_overlaps(&tree, query, output);
		testing.expect_value(t, query_status, physics.Physics_Status.Ok);
		actual: [MAX_LEAVES]physics.Reference_State;
		for output_index in 0 ..< output_count
		{
			actual[output_values[output_index]] = .Present;
		}
		for model_index in 0 ..< count
		{
			expected := physics.tree_bounds_intersect(model[model_index], query);
			testing.expect_value(t, actual[model_index], expected);
		}
	}
	testing.expect_value(t, physics.tree_refit(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_refine(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_refine(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, tree.refinement_frame, u32(2));
	testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_optimize_cache(&tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);

	for _ in 0 ..< 40
	{
		remove_index := int(next_random(&random_state) % u32(count));
		result, status := physics.tree_remove_at(&tree, remove_index);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		count -= 1;
		if remove_index < count
		{
			model[remove_index] = model[count];
			testing.expect_value(t, result.moved_leaf_original_index, i32(count));
			testing.expect_value(t, result.moved_leaf_new_index, i32(remove_index));
		}
		else
		{
			testing.expect_value(t, result.moved_leaf_original_index, i32(-1));
		}
		testing.expect_value(t, physics.tree_validate(&tree), physics.Physics_Status.Ok);
	}
}

@(test)
self_intertree_ray_and_sweep_queries_match_direct_bounds_tests :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	tree_a, tree_b: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree_a, 16, &pool), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(&tree_b, 16, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree_a);
	defer physics.tree_dispose(&tree_b);
	bounds_a := [6]util.Bounding_Box{
		{min={-1, -1, -1}, max={1, 1, 1}},
		{min={0, 0, 0}, max={2, 2, 2}},
		{min={4, 0, 0}, max={5, 1, 1}},
		{min={4.5, 0, 0}, max={6, 1, 1}},
		{min={-5, -1, -1}, max={-4, 1, 1}},
		{min={8, 8, 8}, max={9, 9, 9}},
	};
	bounds_b := [3]util.Bounding_Box{
		{min={0.5, -2, -2}, max={0.75, 2, 2}},
		{min={5, -2, -2}, max={5.25, 2, 2}},
		{min={20, 20, 20}, max={21, 21, 21}},
	};
	testing.expect_value(t, physics.tree_build(&tree_a, &bounds_a[0], len(bounds_a)), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_build(&tree_b, &bounds_b[0], len(bounds_b)), physics.Physics_Status.Ok);
	pairs_storage: [MAX_PAIRS]physics.Tree_Leaf_Pair;
	pairs := util.Buffer(physics.Tree_Leaf_Pair){memory=&pairs_storage[0], length=i32(len(pairs_storage)), id=-1};
	self_count, self_status := physics.tree_self_query(&tree_a, pairs);
	testing.expect_value(t, self_status, physics.Physics_Status.Ok);
	actual_self: [MAX_PAIRS]physics.Reference_State;
	for index in 0 ..< self_count
	{
		actual_self[pair_key(int(pairs_storage[index].a), int(pairs_storage[index].b))] = .Present;
	}
	for a in 0 ..< len(bounds_a)
	{
		for b in a + 1 ..< len(bounds_a)
		{
			testing.expect_value(
				t,
				actual_self[pair_key(a, b)],
				physics.tree_bounds_intersect(bounds_a[a], bounds_a[b])
			);
		}
	}

	intertree_count, intertree_status := physics.tree_intertree_query(&tree_a, &tree_b, pairs);
	testing.expect_value(t, intertree_status, physics.Physics_Status.Ok);
	actual_intertree: [MAX_PAIRS]physics.Reference_State;
	for index in 0 ..< intertree_count
	{
		key := int(pairs_storage[index].a) * MAX_LEAVES + int(pairs_storage[index].b);
		actual_intertree[key] = .Present;
	}
	for a in 0 ..< len(bounds_a)
	{
		for b in 0 ..< len(bounds_b)
		{
			key := a * MAX_LEAVES + b;
			testing.expect_value(t, actual_intertree[key], physics.tree_bounds_intersect(bounds_a[a], bounds_b[b]));
		}
	}

	leaf_storage: [MAX_LEAVES]i32;
	leaves := util.Buffer(i32){memory=&leaf_storage[0], length=i32(len(leaf_storage)), id=-1};
	ray := physics.Tree_Ray{origin={-10, 0.5, 0.5}, direction={1, 0, 0}, maximum_t=30};
	ray_count, ray_status := physics.tree_ray_query(&tree_a, ray, leaves);
	testing.expect_value(t, ray_status, physics.Physics_Status.Ok);
	testing.expect_value(t, ray_count, 5);
	sweep_bounds := util.Bounding_Box{min={-0.25, -0.25, -0.25}, max={0.25, 0.25, 0.25}};
	sweep_count, sweep_status := physics.tree_sweep_query(&tree_a, sweep_bounds, {1, 0, 0}, 20, leaves);
	testing.expect_value(t, sweep_status, physics.Physics_Status.Ok);
	testing.expect(t, sweep_count >= 3);
}

@(test)
parallel_refit_refine_and_queries_are_valid_and_repeatable :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);

	tree_a, tree_b: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree_a, MAX_LEAVES, &pool), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(&tree_b, MAX_LEAVES, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree_a);
	defer physics.tree_dispose(&tree_b);
	random_state := u32(0x7e51_8923);
	bounds_a: [48]util.Bounding_Box;
	bounds_b: [37]util.Bounding_Box;
	for index in 0 ..< len(bounds_a)
	{
		bounds_a[index] = random_bounds(&random_state, index);
	}
	for index in 0 ..< len(bounds_b)
	{
		bounds_b[index] = random_bounds(&random_state, index + 100);
	}
	testing.expect_value(t, physics.tree_build(&tree_a, &bounds_a[0], len(bounds_a)), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_build(&tree_b, &bounds_b[0], len(bounds_b)), physics.Physics_Status.Ok);

	maintenance_workspace: physics.Tree_Refit_Refine_Workspace;
	testing.expect_value(
		t, physics.tree_refit_refine_workspace_initialize(&maintenance_workspace, 4, MAX_LEAVES, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.tree_refit_refine_workspace_dispose(&maintenance_workspace);
	testing.expect_value(
		t, physics.tree_refit_refine_parallel(&tree_a, 0, boundary, &maintenance_workspace),
		physics.Physics_Status.Ok,
	);
	for worker_index in 0 ..< boundary.worker_count
	{
		testing.expect(t, maintenance_workspace.worker_hits.memory[worker_index] > 0);
	}
	testing.expect_value(t, physics.tree_validate(&tree_a), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&tree_b), physics.Physics_Status.Ok);

	workspace: physics.Tree_Parallel_Query_Workspace;
	testing.expect_value(
		t, physics.tree_parallel_query_workspace_initialize(nil, 4, MAX_LEAVES, 1, &pool),
		physics.Physics_Status.Invalid_Argument,
	);
	testing.expect_value(
		t, physics.tree_parallel_query_workspace_initialize(&workspace, 4, MAX_LEAVES, 1, nil),
		physics.Physics_Status.Invalid_Argument,
	);
	invalid_sizes: [6][3]int = {{0, MAX_LEAVES, 1}, {-1, MAX_LEAVES, 1},
		{4, 0, 1}, {4, -1, 1}, {4, MAX_LEAVES, 0}, {4, MAX_LEAVES, -1}};
	for sizes in invalid_sizes
	{
		testing.expect_value(
			t, physics.tree_parallel_query_workspace_initialize(&workspace, sizes[0], sizes[1], sizes[2], &pool),
			physics.Physics_Status.Invalid_Argument,
		);
	}
	oversized: [2][2]int = {{4, max(int) / 2 + 1}, {max(int) / 16 + 1, MAX_LEAVES}};
	for sizes in oversized
	{
		testing.expect_value(
			t, physics.tree_parallel_query_workspace_initialize(&workspace, sizes[0], sizes[1], 1, &pool),
			physics.Physics_Status.Capacity_Missing,
		);
	}
	testing.expect_value(
		t, physics.tree_parallel_query_workspace_initialize(&workspace, 4, MAX_LEAVES, MAX_PAIRS / 2, &pool),
		physics.Physics_Status.Ok,
	);
	pairs_storage: [MAX_PAIRS]physics.Tree_Leaf_Pair;
	pairs := util.Buffer(physics.Tree_Leaf_Pair){memory=&pairs_storage[0], length=i32(len(pairs_storage)), id=-1};

	self_count, self_status := physics.tree_self_query_parallel(&tree_a, boundary, &workspace, pairs);
	testing.expect_value(t, self_status, physics.Physics_Status.Ok);
	self_hash := pair_hash(&pairs_storage[0], self_count);
	for _ in 0 ..< 8
	{
		count, status := physics.tree_self_query_parallel(&tree_a, boundary, &workspace, pairs);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, count, self_count);
		testing.expect_value(t, pair_hash(&pairs_storage[0], count), self_hash);
	}

	intertree_count, intertree_status := physics.tree_intertree_query_parallel(
		&tree_a,
		&tree_b,
		boundary,
		&workspace,
		pairs
	);
	testing.expect_value(t, intertree_status, physics.Physics_Status.Ok);
	intertree_hash := pair_hash(&pairs_storage[0], intertree_count);
	actual_intertree: [MAX_PAIRS]physics.Reference_State;
	for index in 0 ..< intertree_count
	{
		pair := pairs_storage[index];
		testing.expect(t, pair.a >= 0 && int(pair.a) < len(bounds_a));
		testing.expect(t, pair.b >= 0 && int(pair.b) < len(bounds_b));
		actual_intertree[int(pair.a) * MAX_LEAVES + int(pair.b)] = .Present;
	}
	for a in 0 ..< len(bounds_a)
	{
		for b in 0 ..< len(bounds_b)
		{
			testing.expect_value(
				t, actual_intertree[a * MAX_LEAVES + b], physics.tree_bounds_intersect(bounds_a[a], bounds_b[b]),
			);
		}
	}
	for _ in 0 ..< 8
	{
		count, status := physics.tree_intertree_query_parallel(&tree_a, &tree_b, boundary, &workspace, pairs);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, count, intertree_count);
		testing.expect_value(t, pair_hash(&pairs_storage[0], count), intertree_hash);
	}

	actual_self: [MAX_PAIRS]physics.Reference_State;
	count, _ := physics.tree_self_query_parallel(&tree_a, boundary, &workspace, pairs);
	for index in 0 ..< count
	{
		actual_self[pair_key(int(pairs_storage[index].a), int(pairs_storage[index].b))] = .Present;
	}
	for a in 0 ..< len(bounds_a)
	{
		for b in a + 1 ..< len(bounds_a)
		{
			testing.expect_value(
				t,
				actual_self[pair_key(a, b)],
				physics.tree_bounds_intersect(bounds_a[a], bounds_a[b])
			);
		}
	}
	testing.expect_value(t, physics.tree_parallel_query_workspace_dispose(&workspace), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_parallel_query_workspace_dispose(&workspace), physics.Physics_Status.Disposed);

	// dense small trees require more claims than the four initial worker jobs.
	// both constructors must reset the shared cursor after empty queries and failures
	dense_bounds: [8]util.Bounding_Box;
	for &bounds in dense_bounds
	{
		bounds = {min={-1, -1, -1}, max={1, 1, 1}};
	}
	finalizers: [2]physics.Tree_Parallel_Worker_Finalize_Proc = {
		proc "contextless" (user_context: rawptr, worker_index: int) -> physics.Physics_Status
		{
			counts: ^[4]i32 = (^[4]i32)(user_context);
			counts[worker_index] += 1;
			return .Ok;
		},
		proc "contextless" (user_context: rawptr, worker_index: int) -> physics.Physics_Status
		{
			counts: ^[4]i32 = (^[4]i32)(user_context);
			counts[worker_index] += 1;
			return .Invalid_Argument;
		},
	};
	pool_bytes_after_cycle: u64;
	allocations_after_cycle: u64;
	available_after_cycle: [21]int;
	for cycle in 0 ..< 4
	{
		testing.expect_value(
			t, physics.tree_parallel_query_workspace_initialize(&workspace, 4, MAX_LEAVES, MAX_PAIRS / 2, &pool),
			physics.Physics_Status.Ok,
		);
		leaf_counts: [4][2]int = {{0, 0}, {1, 1}, {8, 0}, {8, 6}};
		for sizes in leaf_counts
		{
			testing.expect_value(t, physics.tree_build(&tree_a, &dense_bounds[0], sizes[0]), physics.Physics_Status.Ok);
			testing.expect_value(t, physics.tree_build(&tree_b, &dense_bounds[0], sizes[1]), physics.Physics_Status.Ok);
			expected_self: int = sizes[0] * (sizes[0] - 1) / 2;
			expected_intertree: int = sizes[0] * sizes[1];
			self_count, self_status = physics.tree_self_query_parallel(&tree_a, boundary, &workspace, pairs);
			testing.expect_value(t, self_status, physics.Physics_Status.Ok);
			testing.expect_value(t, self_count, expected_self);
			intertree_count, intertree_status = physics.tree_intertree_query_parallel(
				&tree_a, &tree_b, boundary, &workspace, pairs,
			);
			testing.expect_value(t, intertree_status, physics.Physics_Status.Ok);
			testing.expect_value(t, intertree_count, expected_intertree);

			// buffer failures stop each worker before its finalizer. finalizer failures
			// propagate after its claimed jobs complete. the final pass proves reuse
			expected_statuses: [4]physics.Physics_Status = {.Ok, .Capacity_Missing, .Invalid_Argument, .Ok};
			for expected_status in expected_statuses
			{
				if sizes[0] < 8 && expected_status != .Ok
				{
					continue;
				}
				self_context: physics.Tree_Parallel_Buffer_Visitor_Context = {
					output={memory=&pairs_storage[0], length=MAX_PAIRS / 2, id=-1},
				};
				intertree_context: physics.Tree_Parallel_Buffer_Visitor_Context = {
					output={memory=&pairs_storage[MAX_PAIRS / 2], length=MAX_PAIRS / 2, id=-1},
				};
				finalized: [4]i32;
				finalizer: physics.Tree_Parallel_Worker_Finalize_Proc = finalizers[0];
				if expected_status == .Capacity_Missing
				{
					self_context.output.length = 0;
					intertree_context.output.length = 0;
				}
				else if expected_status == .Invalid_Argument
				{
					finalizer = finalizers[1];
				}
				testing.expect_value(
					t, physics.tree_self_intertree_visit_parallel(
						&tree_a, &tree_b, boundary, &workspace, physics.tree_parallel_buffer_visitor,
						&self_context, &intertree_context, finalizer, &finalized,
					), expected_status,
				);
				if expected_status == .Capacity_Missing
				{
					testing.expect_value(t, self_context.cursor, u32(0));
					testing.expect_value(t, intertree_context.cursor, u32(0));
					for finalized_count in finalized
					{
						testing.expect_value(t, finalized_count, i32(0));
					}
					continue;
				}
				testing.expect_value(t, int(self_context.cursor), expected_self);
				testing.expect_value(t, int(intertree_context.cursor), expected_intertree);
				active_workers: int = min(4, expected_self + expected_intertree);
				for finalized_count, worker_index in finalized
				{
					expected_finalized: i32 = 0;
					if worker_index < active_workers
					{
						expected_finalized = 1;
					}
					testing.expect_value(t, finalized_count, expected_finalized);
				}
				// callbacks may arrive in any order. every pair must appear exactly once
				seen_self: [MAX_PAIRS]physics.Reference_State;
				for pair_index in 0 ..< int(self_context.cursor)
				{
					pair: physics.Tree_Leaf_Pair = self_context.output.memory[pair_index];
					testing.expect(t, pair.a >= 0 && pair.a < pair.b && int(pair.b) < sizes[0]);
					key: int = pair_key(int(pair.a), int(pair.b));
					testing.expect_value(t, seen_self[key], physics.Reference_State.Missing);
					seen_self[key] = .Present;
				}
				seen_intertree: [MAX_PAIRS]physics.Reference_State;
				for pair_index in 0 ..< int(intertree_context.cursor)
				{
					pair: physics.Tree_Leaf_Pair = intertree_context.output.memory[pair_index];
					testing.expect(t, pair.a >= 0 && int(pair.a) < sizes[0] && pair.b >= 0 && int(pair.b) < sizes[1]);
					key: int = int(pair.a) * MAX_LEAVES + int(pair.b);
					testing.expect_value(t, seen_intertree[key], physics.Reference_State.Missing);
					seen_intertree[key] = .Present;
				}
			}
		}
		testing.expect_value(t, physics.tree_parallel_query_workspace_dispose(&workspace), physics.Physics_Status.Ok);
		if cycle == 0
		{
			pool_bytes_after_cycle = util.buffer_pool_total_allocated_byte_count(&pool);
			allocations_after_cycle = dispatcher.general_allocator_call_count;
		}
		testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_cycle);
		testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_cycle);
		for power in 0 ..= 20
		{
			available: int;
			status: util.Memory_Status;
			available, status = util.buffer_pool_available_slot_count(&pool, power);
			testing.expect_value(t, status, util.Memory_Status.Ok);
			if cycle == 0
			{
				available_after_cycle[power] = available;
			}
			testing.expect_value(t, available, available_after_cycle[power]);
		}
	}
}

@(test)
validate_root_refine2_budget_and_parallel_path :: proc(t: ^testing.T)
{
	REFINEMENT_LEAF_COUNT :: 512;
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	for power in 0 ..= 20
	{
		testing.expect_value(
			t,
			util.buffer_pool_ensure_capacity_for_power(&pool, 1048576, power),
			util.Memory_Status.Ok
		);
	}
	defer util.buffer_pool_dispose(&pool);
	bounds: [REFINEMENT_LEAF_COUNT]util.Bounding_Box;
	random_state := u32(0x8b71_4d29);
	for index in 0 ..< len(bounds)
	{
		bounds[index] = random_bounds(&random_state, index);
	}
	single_tree, multi_tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&single_tree, len(bounds), &pool), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(&multi_tree, len(bounds), &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&single_tree);
	defer physics.tree_dispose(&multi_tree);
	testing.expect_value(t, physics.tree_build(&single_tree, &bounds[0], len(bounds)), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_build(&multi_tree, &bounds[0], len(bounds)), physics.Physics_Status.Ok);

	workspace: physics.Tree_Refit_Refine_Workspace;
	testing.expect_value(
		t, physics.tree_refit_refine_workspace_initialize(&workspace, 4, len(bounds), &pool), physics.Physics_Status.Ok,
	);
	defer physics.tree_refit_refine_workspace_dispose(&workspace);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	schedule := physics.Tree_Refinement_Schedule{
		root_refinement_size=80,
		subtree_refinement_count=0,
		subtree_refinement_size=80,
		use_priority_queue=.Missing,
	};
	single_start, multi_start := 0, 0;
	testing.expect_value(
		t, physics.tree_refine2_single(
		&single_tree,
		schedule,
		&single_start,
		&workspace,
		.Active
	), physics.Physics_Status.Ok,
	);
	pool_bytes_after_startup := util.buffer_pool_total_allocated_byte_count(&pool);
	allocations_after_startup := dispatcher.general_allocator_call_count;
	testing.expect_value(
		t, physics.tree_refine2_parallel(
		&multi_tree,
		schedule,
		&multi_start,
		boundary,
		3,
		&workspace
	), physics.Physics_Status.Ok,
	);
	testing.expect_value(t, physics.tree_validate(&single_tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&multi_tree), physics.Physics_Status.Ok);
	single_root_bounds := physics.tree_bounds_merge(
		physics.tree_child_bounds(single_tree.nodes.memory[0].a),
		physics.tree_child_bounds(single_tree.nodes.memory[0].b),
	);
	multi_root_bounds := physics.tree_bounds_merge(
		physics.tree_child_bounds(multi_tree.nodes.memory[0].a),
		physics.tree_child_bounds(multi_tree.nodes.memory[0].b),
	);
	testing.expect_value(t, multi_root_bounds, single_root_bounds);
	testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_startup);
}

@(test)
validate_large_refine2_inner_tasks :: proc(t: ^testing.T)
{
	LEAF_COUNT :: 4096;
	REFINEMENT_SIZE :: 2048;
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	for power in 0 ..= 20
	{
		testing.expect_value(
			t,
			util.buffer_pool_ensure_capacity_for_power(&pool, 8388608, power),
			util.Memory_Status.Ok
		);
	}
	defer util.buffer_pool_dispose(&pool);
	bounds: [LEAF_COUNT]util.Bounding_Box;
	random_state := u32(0x4b81_29d7);
	for index in 0 ..< len(bounds)
	{
		bounds[index] = random_bounds(&random_state, index);
	}
	single_tree, parallel_tree: physics.Tree;
	trees := [2]^physics.Tree{&single_tree, &parallel_tree};
	for tree in trees
	{
		testing.expect_value(t, physics.tree_initialize(tree, len(bounds), &pool), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_build(tree, &bounds[0], len(bounds)), physics.Physics_Status.Ok);
	}
	defer physics.tree_dispose(&single_tree);
	defer physics.tree_dispose(&parallel_tree);
	workspace: physics.Tree_Refit_Refine_Workspace;
	testing.expect_value(
		t, physics.tree_refit_refine_workspace_initialize(&workspace, 4, len(bounds), &pool, REFINEMENT_SIZE),
		physics.Physics_Status.Ok,
	);
	defer physics.tree_refit_refine_workspace_dispose(&workspace);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	schedule := physics.Tree_Refinement_Schedule{
		root_refinement_size=REFINEMENT_SIZE,
		subtree_refinement_count=0,
		subtree_refinement_size=REFINEMENT_SIZE,
		use_priority_queue=.Missing,
	};
	single_start, parallel_start := 0, 0;
	pool_bytes_after_startup := util.buffer_pool_total_allocated_byte_count(&pool);
	allocations_after_startup := dispatcher.general_allocator_call_count;
	testing.expect_value(
		t, physics.tree_refine2_single(&single_tree, schedule, &single_start, &workspace, .Active),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.tree_refine2_parallel(
		&parallel_tree, schedule, &parallel_start, boundary, 3, &workspace,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, physics.tree_validate(&single_tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&parallel_tree), physics.Physics_Status.Ok);
	single_root_bounds := physics.tree_bounds_merge(
		physics.tree_child_bounds(single_tree.nodes.memory[0].a),
		physics.tree_child_bounds(single_tree.nodes.memory[0].b),
	);
	parallel_root_bounds := physics.tree_bounds_merge(
		physics.tree_child_bounds(parallel_tree.nodes.memory[0].a),
		physics.tree_child_bounds(parallel_tree.nodes.memory[0].b),
	);
	testing.expect_value(t, parallel_root_bounds, single_root_bounds);
	refinement_work_count: i32;
	for worker_index in 0 ..< workspace.worker_count
	{
		refinement_work_count += workspace.worker_hits.memory[worker_index];
	}
	testing.expect(t, refinement_work_count > 0);
	testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_startup);
}

@(test)
tree_growth_preserves_data_while_pool_expands :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 1, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	bounds := util.Bounding_Box{min={1, 2, 3}, max={4, 5, 6}};
	_, add_status := physics.tree_add(&tree, bounds);
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	blocked: [4]util.Buffer(physics.Tree_Node);
	blocked_count := 0;
	for blocked_count < len(blocked)
	{
		buffer, status := util.buffer_pool_take_at_least(&pool, physics.Tree_Node, 1023);
		if status != .Ok
		{
			break;
		}
		blocked[blocked_count] = buffer;
		blocked_count += 1;
	}
	testing.expect(t, blocked_count > 0);
	testing.expect_value(t, physics.tree_ensure_capacity(&tree, 1024), physics.Physics_Status.Ok);
	actual, bounds_status := physics.tree_get_leaf_bounds(&tree, 0);
	testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, bounds);
	for index in 0 ..< blocked_count
	{
		_ = util.buffer_pool_return(&pool, &blocked[index]);
	}
}
