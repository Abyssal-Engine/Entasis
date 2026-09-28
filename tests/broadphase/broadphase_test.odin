package broadphase_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

MAX_COLLIDABLES :: 96;
MAX_CANDIDATES :: 4096;

Broad_Phase_Fixture :: struct
{
	pool:        util.Buffer_Pool,
	registry:    physics.Shape_Registry,
	bodies:      physics.Bodies,
	statics:     physics.Statics,
	broad_phase: physics.Broad_Phase,
	shape:       physics.Typed_Index,
	body_handles:   [32]physics.Body_Handle,
	static_handles: [32]physics.Static_Handle,
	body_count:     int,
	static_count:   int,
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 256), util.Memory_Status.Ok);
	for power in 0 ..= 22
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 524288, power), util.Memory_Status.Ok);
	}
}

body_description :: proc(position: util.Vector3, shape: physics.Typed_Index) -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity(), position=position},
		local_inertia={inverse_inertia_tensor={1, 0, 1, 0, 0, 1}, inverse_mass=1},
		collidable={shape=shape, maximum_speculative_margin=2},
		activity={sleep_threshold=0.01, minimum_timestep_count_under_threshold=32},
	};
}

fixture_initialize :: proc(t: ^testing.T, fixture: ^Broad_Phase_Fixture)
{
	prepare_pool(t, &fixture.pool);
	testing.expect_value(
		t,
		physics.shape_registry_initialize(&fixture.registry, 8, &fixture.pool),
		physics.Physics_Status.Ok
	);
	sphere := physics.Sphere{radius=1};
	shape, shape_status := physics.shape_registry_add(&fixture.registry, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	fixture.shape = shape;
	testing.expect_value(
		t,
		physics.bodies_initialize(&fixture.bodies, 32, 4, 1, &fixture.pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.statics_initialize(&fixture.statics, 32, &fixture.pool), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.bodies_bind_shape_registry(&fixture.bodies, &fixture.registry),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.statics_bind_shape_registry(&fixture.statics, &fixture.registry),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t, physics.broad_phase_initialize(&fixture.broad_phase, 32, 64, MAX_CANDIDATES, 4, &fixture.pool),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, physics.broad_phase_bind_owners(&fixture.broad_phase, &fixture.registry, &fixture.bodies, &fixture.statics),
		physics.Physics_Status.Ok,
	);
	fixture.body_count = 18;
	for index in 0 ..< fixture.body_count
	{
		description := body_description({f32(index) * 1.35 - 9, f32(index % 3) * 0.2, 0}, fixture.shape);
		handle, status := physics.bodies_add(&fixture.bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		fixture.body_handles[index] = handle;
		testing.expect_value(t, physics.broad_phase_add_body(&fixture.broad_phase, handle), physics.Physics_Status.Ok);
	}
	fixture.static_count = 12;
	for index in 0 ..< fixture.static_count
	{
		description := physics.Static_Description{
			pose={orientation=util.quaternion_identity(), position={f32(index) * 2.1 - 8.5, 0.4, 0}},
			shape=fixture.shape,
		};
		handle, status := physics.statics_add(&fixture.statics, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		fixture.static_handles[index] = handle;
		testing.expect_value(
			t,
			physics.broad_phase_add_static(&fixture.broad_phase, handle),
			physics.Physics_Status.Ok
		);
	}
}

fixture_dispose :: proc(fixture: ^Broad_Phase_Fixture)
{
	_ = physics.broad_phase_dispose(&fixture.broad_phase);
	_ = physics.statics_dispose(&fixture.statics);
	_ = physics.bodies_dispose(&fixture.bodies);
	_ = physics.shape_registry_dispose(&fixture.registry);
	_ = util.buffer_pool_dispose(&fixture.pool);
}

pair_exists :: proc "contextless" (
	pairs: [^]physics.Broad_Phase_Pair, count: int,
	a, b: physics.Collidable_Reference,
) -> physics.Reference_State
{
	for index in 0 ..< count
	{
		if pairs[index].a.packed == a.packed && pairs[index].b.packed == b.packed
		{
			return .Present;
		}
	}
	return .Missing;
}

validate_pair_set_equal :: proc(
	t: ^testing.T,
	expected: [^]physics.Broad_Phase_Pair, expected_count: int,
	actual: [^]physics.Broad_Phase_Pair, actual_count: int,
)
{
	testing.expect_value(t, actual_count, expected_count);
	for actual_index in 0 ..< actual_count
	{
		pair := actual[actual_index];
		testing.expect_value(
			t, pair_exists(expected, expected_count, pair.a, pair.b),
			physics.Reference_State.Present,
		);
		for prior_index in 0 ..< actual_index
		{
			prior := actual[prior_index];
			testing.expect(
				t, prior.a.packed != pair.a.packed || prior.b.packed != pair.b.packed,
			);
		}
	}
}

reference_occurrence_count :: proc "contextless" (
	references: [^]physics.Collidable_Reference, count: int,
	reference: physics.Collidable_Reference,
) -> int
{
	occurrence_count := 0;
	for index in 0 ..< count
	{
		if references[index].packed == reference.packed
		{
			occurrence_count += 1;
		}
	}
	return occurrence_count;
}

validate_volume_any_equivalence :: proc(
	t: ^testing.T,
	fixture: ^Broad_Phase_Fixture,
	references: util.Buffer(physics.Collidable_Reference),
)
{
	for query_index in 0 ..< 144
	{
		center := util.Vector3{
			x=-16 + f32(query_index % 48) * 0.8,
			y=f32(query_index / 48) * 1.8 - 0.7,
			z=0,
		};
		half_extent := f32(0.1 + 0.35 * f32(query_index % 7));
		extent := util.Vector3{half_extent, half_extent, half_extent};
		bounds := util.Bounding_Box{
			min=util.vector3_subtract(center, extent),
			max=util.vector3_add(center, extent),
		};
		complete_count, complete_status := physics.broad_phase_volume_query(
			&fixture.broad_phase, bounds, references,
		);
		testing.expect_value(t, complete_status, physics.Physics_Status.Ok);
		any_reference, any_state, any_status := physics.broad_phase_volume_any_query(
			&fixture.broad_phase, bounds,
		);
		testing.expect_value(t, any_status, physics.Physics_Status.Ok);
		if complete_count == 0
		{
			testing.expect_value(t, any_state, physics.Reference_State.Missing);
		}
		else
		{
			testing.expect_value(t, any_state, physics.Reference_State.Present);
			testing.expect_value(
				t,
				reference_occurrence_count(references.memory, complete_count, any_reference),
				1,
			);
		}
	}
}

validate_candidates_against_brute_force :: proc(
	t: ^testing.T, fixture: ^Broad_Phase_Fixture,
	pairs: [^]physics.Broad_Phase_Pair, count: int,
)
{
	expected_count := 0;
	for a in 0 ..< fixture.broad_phase.active_tree.leaf_count
	{
		bounds_a, status := physics.tree_get_leaf_bounds(&fixture.broad_phase.active_tree, a);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		for b in a + 1 ..< fixture.broad_phase.active_tree.leaf_count
		{
			bounds_b, bounds_status := physics.tree_get_leaf_bounds(&fixture.broad_phase.active_tree, b);
			testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
			if physics.tree_bounds_intersect(bounds_a, bounds_b) == .Present
			{
				expected_count += 1;
				reference_a := fixture.broad_phase.active_leaves.memory[a];
				reference_b := fixture.broad_phase.active_leaves.memory[b];
				if reference_b.packed < reference_a.packed
				{
					reference_a, reference_b = reference_b, reference_a;
				}
				testing.expect_value(
					t,
					pair_exists(pairs, count, reference_a, reference_b),
					physics.Reference_State.Present
				);
			}
		}
		for b in 0 ..< fixture.broad_phase.static_tree.leaf_count
		{
			bounds_b, bounds_status := physics.tree_get_leaf_bounds(&fixture.broad_phase.static_tree, b);
			testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
			if physics.tree_bounds_intersect(bounds_a, bounds_b) == .Present
			{
				expected_count += 1;
				testing.expect_value(
					t, pair_exists(
					pairs, count, fixture.broad_phase.active_leaves.memory[a], fixture.broad_phase.static_leaves.memory[b],
				),
					physics.Reference_State.Present,
				);
			}
		}
	}
	testing.expect_value(t, count, expected_count);
}

validate_depth_first_cache_order :: proc(t: ^testing.T, tree: ^physics.Tree)
{
	for node_index in 0 ..< tree.node_count
	{
		node := tree.nodes.memory[node_index];
		if node.a.index >= 0
		{
			testing.expect_value(t, node.a.index, i32(node_index + 1));
		}
		if node.b.index >= 0
		{
			testing.expect_value(t, node.b.index, i32(node_index + int(node.a.leaf_count)));
		}
	}
}

validate_pinned_scheduler_and_parallel_update2 :: proc(t: ^testing.T)
{
	frame0_active := physics.broad_phase_default_active_refinement_scheduler(0, 400);
	testing.expect_value(t, frame0_active.root_refinement_size, 20);
	testing.expect_value(t, frame0_active.subtree_refinement_count, 0);
	testing.expect_value(t, frame0_active.subtree_refinement_size, 80);
	testing.expect_value(t, frame0_active.use_priority_queue, physics.Reference_State.Missing);
	frame1_active := physics.broad_phase_default_active_refinement_scheduler(1, 400);
	testing.expect_value(t, frame1_active.root_refinement_size, 0);
	testing.expect_value(t, frame1_active.subtree_refinement_count, 1);
	testing.expect_value(t, frame1_active.subtree_refinement_size, 80);
	testing.expect_value(t, frame1_active.use_priority_queue, physics.Reference_State.Missing);
	frame1_active_midpoint := physics.broad_phase_default_active_refinement_scheduler(1, 40000);
	testing.expect_value(t, frame1_active_midpoint.root_refinement_size, 0);
	testing.expect_value(t, frame1_active_midpoint.subtree_refinement_count, 2);
	testing.expect_value(t, frame1_active_midpoint.subtree_refinement_size, 800);
	frame2_active := physics.broad_phase_default_active_refinement_scheduler(2, 400);
	testing.expect_value(t, frame2_active.root_refinement_size, 20);
	testing.expect_value(t, frame2_active.subtree_refinement_count, 0);
	testing.expect_value(t, frame2_active.use_priority_queue, physics.Reference_State.Present);
	frame1_static := physics.broad_phase_default_static_refinement_scheduler(1, 400);
	testing.expect_value(t, frame1_static.root_refinement_size, 0);
	testing.expect_value(t, frame1_static.subtree_refinement_count, 1);
	testing.expect_value(t, frame1_static.subtree_refinement_size, 80);
	task_active, task_static := physics.broad_phase_refinement_task_counts(
		physics.broad_phase_default_active_refinement_scheduler(0, 300),
		physics.broad_phase_default_static_refinement_scheduler(0, 100), 4,
	);
	testing.expect_value(t, task_active, 3);
	testing.expect_value(t, task_static, 1);

	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);
	sphere := physics.Sphere{radius=0.25};
	shape, shape_status := physics.shape_registry_add(&registry, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	bodies: physics.Bodies;
	statics: physics.Statics;
	testing.expect_value(t, physics.bodies_initialize(&bodies, 320, 4, 1, &pool), physics.Physics_Status.Ok);
	defer physics.bodies_dispose(&bodies);
	testing.expect_value(t, physics.statics_initialize(&statics, 128, &pool), physics.Physics_Status.Ok);
	defer physics.statics_dispose(&statics);
	testing.expect_value(t, physics.bodies_bind_shape_registry(&bodies, &registry), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_bind_shape_registry(&statics, &registry), physics.Physics_Status.Ok);
	broad_phase: physics.Broad_Phase;
	testing.expect_value(
		t, physics.broad_phase_initialize(&broad_phase, 320, 128, MAX_CANDIDATES, 4, &pool),
		physics.Physics_Status.Ok,
	);
	defer physics.broad_phase_dispose(&broad_phase);
	testing.expect_value(
		t, physics.broad_phase_bind_owners(&broad_phase, &registry, &bodies, &statics),
		physics.Physics_Status.Ok,
	);
	for index in 0 ..< 300
	{
		description := body_description({f32(index) * 0.75, f32(index % 7) * 0.1, 0}, shape);
		handle, add_status := physics.bodies_add(&bodies, &description);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.broad_phase_add_body(&broad_phase, handle), physics.Physics_Status.Ok);
	}
	for index in 0 ..< 100
	{
		description := physics.Static_Description{
			pose={orientation=util.quaternion_identity(), position={f32(index) * 1.5, 4, 0}}, shape=shape,
		};
		handle, add_status := physics.statics_add(&statics, &description);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.broad_phase_add_static(&broad_phase, handle), physics.Physics_Status.Ok);
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	active_nodes_before := broad_phase.active_tree.nodes.memory;
	static_nodes_before := broad_phase.static_tree.nodes.memory;
	pool_bytes_after_startup := util.buffer_pool_total_allocated_byte_count(&pool);
	allocations_after_startup := dispatcher.general_allocator_call_count;
	testing.expect_value(t, physics.broad_phase_update(&broad_phase, boundary), physics.Physics_Status.Ok);
	testing.expect(t, broad_phase.active_tree.nodes.memory != active_nodes_before);
	testing.expect(t, broad_phase.static_tree.nodes.memory == static_nodes_before);
	validate_depth_first_cache_order(t, &broad_phase.active_tree);
	refinement_work_count: i32;
	for worker_index in 0 ..< broad_phase.maintenance_workspace.worker_count
	{
		worker_work_count := broad_phase.maintenance_workspace.worker_hits.memory[worker_index];
		refinement_work_count += worker_work_count;
	}
	testing.expect(t, refinement_work_count >= 5);
	testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_startup);
	testing.expect_value(t, physics.broad_phase_update(&broad_phase, boundary), physics.Physics_Status.Ok);
	testing.expect(t, broad_phase.active_subtree_refinement_start_index > 0);
	testing.expect(t, broad_phase.static_subtree_refinement_start_index > 0);
	validate_depth_first_cache_order(t, &broad_phase.active_tree);
	testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_startup);
	broad_phase.frame_index = max(i32);
	testing.expect_value(t, physics.broad_phase_update(&broad_phase, boundary), physics.Physics_Status.Ok);
	testing.expect_value(t, broad_phase.frame_index, i32(0));
	testing.expect_value(t, broad_phase.active_tree.refinement_frame, u32(0));
	testing.expect_value(t, broad_phase.static_tree.refinement_frame, u32(0));
	frame_after_rollover := physics.broad_phase_default_active_refinement_scheduler(
		broad_phase.frame_index, broad_phase.active_tree.leaf_count,
	);
	expected_frame_after_rollover := physics.broad_phase_default_active_refinement_scheduler(
		0, broad_phase.active_tree.leaf_count,
	);
	testing.expect_value(
		t, frame_after_rollover.root_refinement_size, expected_frame_after_rollover.root_refinement_size,
	);
	testing.expect_value(
		t, frame_after_rollover.subtree_refinement_count, expected_frame_after_rollover.subtree_refinement_count,
	);
	testing.expect_value(
		t, frame_after_rollover.subtree_refinement_size, expected_frame_after_rollover.subtree_refinement_size,
	);
	testing.expect_value(
		t, frame_after_rollover.use_priority_queue, expected_frame_after_rollover.use_priority_queue,
	);
	testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), pool_bytes_after_startup);
}

@(test)
single_and_multi_worker_candidates_match_brute_force_as_sets :: proc(t: ^testing.T)
{
	fixture: Broad_Phase_Fixture;
	fixture_initialize(t, &fixture);
	defer fixture_dispose(&fixture);
	single_storage: [MAX_CANDIDATES]physics.Broad_Phase_Pair;
	single_buffer := util.Buffer(physics.Broad_Phase_Pair){
		memory=&single_storage[0], length=i32(len(single_storage)), id=-1,
	};
	pair_storage: [MAX_CANDIDATES]physics.Broad_Phase_Pair;
	pair_buffer := util.Buffer(physics.Broad_Phase_Pair){
		memory=&pair_storage[0], length=i32(len(pair_storage)), id=-1,
	};

	single_count, single_status := physics.broad_phase_find_pairs(
		&fixture.broad_phase, single_buffer,
	);
	testing.expect_value(t, single_status, physics.Physics_Status.Ok);
	validate_candidates_against_brute_force(t, &fixture, &single_storage[0], single_count);

	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	multi_count, multi_status := physics.broad_phase_find_pairs(
		&fixture.broad_phase, pair_buffer, boundary,
	);
	testing.expect_value(t, multi_status, physics.Physics_Status.Ok);
	validate_candidates_against_brute_force(t, &fixture, &pair_storage[0], multi_count);
	validate_pair_set_equal(
		t, &single_storage[0], single_count, &pair_storage[0], multi_count,
	);
	for _ in 0 ..< 32
	{
		count, status := physics.broad_phase_find_pairs(
			&fixture.broad_phase, pair_buffer, boundary,
		);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		validate_pair_set_equal(
			t, &single_storage[0], single_count, &pair_storage[0], count,
		);
	}
}

@(test)
bounds_updates_membership_moves_and_dense_removals_keep_owner_indices_valid :: proc(t: ^testing.T)
{
	fixture: Broad_Phase_Fixture;
	fixture_initialize(t, &fixture);
	defer fixture_dispose(&fixture);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	allocations_after_startup := dispatcher.general_allocator_call_count;
	pool_bytes_after_startup := util.buffer_pool_total_allocated_byte_count(&fixture.pool);
	pair_storage: [MAX_CANDIDATES]physics.Broad_Phase_Pair;
	pair_buffer := util.Buffer(physics.Broad_Phase_Pair){memory=&pair_storage[0], length=i32(len(pair_storage)), id=-1};
	testing.expect_value(t, physics.broad_phase_update(&fixture.broad_phase), physics.Physics_Status.Ok);
	validate_depth_first_cache_order(t, &fixture.broad_phase.active_tree);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&fixture.pool), pool_bytes_after_startup);

	for iteration in 0 ..< 64
	{
		handle := fixture.body_handles[iteration % fixture.body_count];
		location, resolve_status := physics.bodies_resolve(&fixture.bodies, handle);
		testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
		fixture.bodies.sets.memory[location.set_index].dynamics_state.memory[location.index].motion.pose.position.y =
			f32((iteration % 7) - 3) * 0.15;
		testing.expect_value(t, physics.broad_phase_update(&fixture.broad_phase, boundary), physics.Physics_Status.Ok);
		count, status := physics.broad_phase_find_pairs(&fixture.broad_phase, pair_buffer, boundary);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		validate_candidates_against_brute_force(t, &fixture, &pair_storage[0], count);
		testing.expect_value(t, dispatcher.general_allocator_call_count, allocations_after_startup);
		testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&fixture.pool), pool_bytes_after_startup);
	}
	testing.expect(t, fixture.broad_phase.active_subtree_refinement_start_index > 0);
	testing.expect(t, fixture.broad_phase.static_subtree_refinement_start_index > 0);
	for worker_index in 0 ..< fixture.broad_phase.maintenance_workspace.worker_count
	{
		testing.expect_value(t, fixture.broad_phase.maintenance_workspace.worker_hits.memory[worker_index], i32(0));
	}

	moved_body := fixture.body_handles[4];
	location, _ := physics.bodies_resolve(&fixture.bodies, moved_body);
	old_active_count := fixture.broad_phase.active_tree.leaf_count;
	testing.expect_value(t, physics.bodies_move(&fixture.bodies, moved_body, 1), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.broad_phase_refresh_body_membership(&fixture.broad_phase, moved_body),
		physics.Physics_Status.Ok
	);
	location, _ = physics.bodies_resolve(&fixture.bodies, moved_body);
	new_index := int(fixture.bodies.sets.memory[location.set_index].collidables.memory[location.index].broad_phase_index);
	testing.expect_value(t, fixture.broad_phase.active_tree.leaf_count, old_active_count - 1);
	testing.expect_value(
		t,
		fixture.broad_phase.static_leaves.memory[new_index].packed & 0x3fff_ffff,
		u32(moved_body.value)
	);

	removed_body := fixture.body_handles[7];
	testing.expect_value(
		t,
		physics.broad_phase_remove_body(&fixture.broad_phase, removed_body),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.bodies_remove(&fixture.bodies, removed_body), physics.Physics_Status.Ok);
	removed_static := fixture.static_handles[3];
	testing.expect_value(
		t,
		physics.broad_phase_remove_static(&fixture.broad_phase, removed_static),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.statics_remove(&fixture.statics, removed_static), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&fixture.broad_phase.active_tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&fixture.broad_phase.static_tree), physics.Physics_Status.Ok);
	for leaf_index in 0 ..< fixture.broad_phase.active_tree.leaf_count
	{
		reference := fixture.broad_phase.active_leaves.memory[leaf_index];
		body_handle, status := physics.collidable_reference_body_handle(reference);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		body_location, _ := physics.bodies_resolve(&fixture.bodies, body_handle);
		testing.expect_value(
			t, fixture.bodies.sets.memory[body_location.set_index].collidables.memory[body_location.index].broad_phase_index,
			i32(leaf_index),
		);
	}
	validate_pinned_scheduler_and_parallel_update2(t);
}

@(test)
volume_ray_and_sweep_queries_return_complete_unique_results :: proc(t: ^testing.T)
{
	fixture: Broad_Phase_Fixture;
	fixture_initialize(t, &fixture);
	defer fixture_dispose(&fixture);
	reference_storage: [MAX_COLLIDABLES]physics.Collidable_Reference;
	references := util.Buffer(physics.Collidable_Reference){
		memory=&reference_storage[0], length=i32(len(reference_storage)), id=-1,
	};
	query_bounds := util.Bounding_Box{min={-4, -2, -2}, max={4, 2, 2}};
	volume_count, volume_status := physics.broad_phase_volume_query(
		&fixture.broad_phase, query_bounds, references,
	);
	testing.expect_value(t, volume_status, physics.Physics_Status.Ok);
	expected_volume_count := 0;
	for leaf_index in 0 ..< fixture.broad_phase.active_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.active_tree, leaf_index);
		if physics.tree_bounds_intersect(bounds, query_bounds) == .Present
		{
			expected_volume_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], volume_count,
				fixture.broad_phase.active_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	for leaf_index in 0 ..< fixture.broad_phase.static_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.static_tree, leaf_index);
		if physics.tree_bounds_intersect(bounds, query_bounds) == .Present
		{
			expected_volume_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], volume_count,
				fixture.broad_phase.static_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	testing.expect_value(t, volume_count, expected_volume_count);
	any_reference, any_state, any_status := physics.broad_phase_volume_any_query(
		&fixture.broad_phase, query_bounds,
	);
	testing.expect_value(t, any_status, physics.Physics_Status.Ok);
	testing.expect_value(t, any_state, physics.Reference_State.Present);
	testing.expect_value(
		t, reference_occurrence_count(&reference_storage[0], volume_count, any_reference), 1,
	);
	_, missing_state, missing_status := physics.broad_phase_volume_any_query(
		&fixture.broad_phase, {min={100, 100, 100}, max={101, 101, 101}},
	);
	testing.expect_value(t, missing_status, physics.Physics_Status.Ok);
	testing.expect_value(t, missing_state, physics.Reference_State.Missing);
	validate_volume_any_equivalence(t, &fixture, references);

	ray := physics.Tree_Ray{origin={-20, 0, 0}, direction={1, 0, 0}, maximum_t=40};
	ray_count, ray_status := physics.broad_phase_ray_query(
		&fixture.broad_phase, ray, references,
	);
	testing.expect_value(t, ray_status, physics.Physics_Status.Ok);
	expected_ray_count := 0;
	for leaf_index in 0 ..< fixture.broad_phase.active_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.active_tree, leaf_index);
		_, hit := physics.tree_ray_intersection(bounds, ray);
		if hit == .Present
		{
			expected_ray_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], ray_count,
				fixture.broad_phase.active_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	for leaf_index in 0 ..< fixture.broad_phase.static_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.static_tree, leaf_index);
		_, hit := physics.tree_ray_intersection(bounds, ray);
		if hit == .Present
		{
			expected_ray_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], ray_count,
				fixture.broad_phase.static_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	testing.expect_value(t, ray_count, expected_ray_count);

	sweep_bounds := util.Bounding_Box{
		min={-0.25, -0.25, -0.25}, max={0.25, 0.25, 0.25},
	};
	sweep_direction := util.Vector3{1, 0, 0};
	sweep_count, sweep_status := physics.broad_phase_sweep_query(
		&fixture.broad_phase, sweep_bounds, sweep_direction, 40, references,
	);
	testing.expect_value(t, sweep_status, physics.Physics_Status.Ok);
	expected_sweep_count := 0;
	sweep_ray := physics.Tree_Ray{direction=sweep_direction, maximum_t=40};
	for leaf_index in 0 ..< fixture.broad_phase.active_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.active_tree, leaf_index);
		expanded := util.Bounding_Box{
			min=util.vector3_subtract(bounds.min, sweep_bounds.max),
			max=util.vector3_subtract(bounds.max, sweep_bounds.min),
		};
		_, hit := physics.tree_ray_intersection(expanded, sweep_ray);
		if hit == .Present
		{
			expected_sweep_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], sweep_count,
				fixture.broad_phase.active_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	for leaf_index in 0 ..< fixture.broad_phase.static_tree.leaf_count
	{
		bounds, _ := physics.tree_get_leaf_bounds(&fixture.broad_phase.static_tree, leaf_index);
		expanded := util.Bounding_Box{
			min=util.vector3_subtract(bounds.min, sweep_bounds.max),
			max=util.vector3_subtract(bounds.max, sweep_bounds.min),
		};
		_, hit := physics.tree_ray_intersection(expanded, sweep_ray);
		if hit == .Present
		{
			expected_sweep_count += 1;
			testing.expect_value(
				t, reference_occurrence_count(
				&reference_storage[0], sweep_count,
				fixture.broad_phase.static_leaves.memory[leaf_index],
			),
				1,
			);
		}
	}
	testing.expect_value(t, sweep_count, expected_sweep_count);
}

Pair_Visit_Finalize_Context :: struct
{
	counts: [4]i32,
}

pair_visit_noop_visitor :: proc "contextless" (
	user_context: rawptr, worker_index: int, pair: physics.Broad_Phase_Pair,
) -> physics.Physics_Status
{
	_ = user_context;
	_ = worker_index;
	_ = pair;
	return .Ok;
}

pair_visit_count_finalize :: proc "contextless" (
	user_context: rawptr, worker_index: int,
) -> physics.Physics_Status
{
	ctx := (^Pair_Visit_Finalize_Context)(user_context);
	if ctx == nil || worker_index < 0 || worker_index >= len(ctx.counts)
	{
		return .Invalid_Argument;
	}
	ctx.counts[worker_index] += 1;
	return .Ok;
}

@(test)
pair_visit_finalizes_each_active_worker_once :: proc(t: ^testing.T)
{
	fixture: Broad_Phase_Fixture;
	fixture_initialize(t, &fixture);
	defer fixture_dispose(&fixture);

	single: Pair_Visit_Finalize_Context;
	status := physics.broad_phase_visit_pairs(
		&fixture.broad_phase, pair_visit_noop_visitor, nil, nil,
		pair_visit_count_finalize, &single,
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, single.counts[0], i32(1));
	for worker_index in 1 ..< len(single.counts)
	{
		testing.expect_value(t, single.counts[worker_index], i32(0));
	}

	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	multi: Pair_Visit_Finalize_Context;
	status = physics.broad_phase_visit_pairs(
		&fixture.broad_phase, pair_visit_noop_visitor, nil,
		util.thread_dispatcher_boundary(&dispatcher),
		pair_visit_count_finalize, &multi,
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	for worker_index in 0 ..< len(multi.counts)
	{
		testing.expect_value(t, multi.counts[worker_index], i32(1));
	}
}
