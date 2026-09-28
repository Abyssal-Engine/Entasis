package public_api_tests

import "core:mem"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

bulk_tree_signature :: proc(tree: ^physics.Tree) -> u64
{
	hash := u64(1469598103934665603);
	for byte in mem.slice_to_bytes(tree.nodes.memory[:tree.node_count])
	{
		hash = (hash ~ u64(byte)) * 1099511628211;
	}
	for byte in mem.slice_to_bytes(tree.metanodes.memory[:tree.node_count])
	{
		hash = (hash ~ u64(byte)) * 1099511628211;
	}
	for byte in mem.slice_to_bytes(tree.leaves.memory[:tree.leaf_count])
	{
		hash = (hash ~ u64(byte)) * 1099511628211;
	}
	return hash;
}

bulk_test_descriptions :: proc(shape: entasis.Shape_Handle, values: []entasis.Static_Description)
{
	for &value, index in values
	{
		value = entasis.static_body(shape, entasis.pose({f32(index % 16) * 4, f32(index / 16) * 4, 10}));
	}
}

@(test)
bulk_static_rebuild_preserves_handles_bounds_queries_and_storage :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	if !testing.expect_value(t, util.buffer_pool_initialize(&pool, 4096, 4), util.Memory_Status.Ok)
	{
		return;
	}
	defer util.buffer_pool_dispose(&pool);
	world: entasis.World;
	description := batch_world_description();
	description.capacity.statics = 512;
	if !testing.expect_value(t, entasis.world_init_with_pool(&world, description, &pool), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	shape, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return;
	}
	descriptions: [256]entasis.Static_Description;
	handles: [256]entasis.Static_Handle;
	bulk_test_descriptions(shape, descriptions[:]);
	bytes_before := util.buffer_pool_total_allocated_byte_count(&pool);
	written, status := entasis.static_add_batch(&world, descriptions[:], handles[:], .None);
	if !testing.expect_value(t, status, entasis.Status.Ok) || !testing.expect_value(t, written, len(handles))
	{
		return;
	}
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), bytes_before);
	simulation, borrow_status := entasis.world_borrow_simulation(&world);
	if !testing.expect_value(t, borrow_status, entasis.Status.Ok)
	{
		return;
	}
	tree := &simulation.broad_phase.static_tree;
	testing.expect_value(t, physics.tree_validate(tree), physics.Physics_Status.Ok);
	// the committed bulk boundary has already produced the deterministic binned tree
	built := bulk_tree_signature(tree);
	testing.expect_value(t, physics.tree_rebuild_binned(tree), physics.Physics_Status.Ok);
	testing.expect_value(t, bulk_tree_signature(tree), built);
	for handle, index in handles
	{
		state, get_status := entasis.static_get(&world, handle);
		testing.expect_value(t, get_status, entasis.Status.Ok);
		testing.expect_value(t, state, descriptions[index]);
		hit, hit_status := entasis.ray_cast_closest(&world, entasis.ray({state.pose.position.x, state.pose.position.y, state.pose.position.z - 3}, {0, 0, 1}, 6));
		testing.expect_value(t, hit_status, entasis.Status.Ok);
		actual, handle_status := entasis.collidable_static_handle(hit.collidable);
		testing.expect_value(t, handle_status, entasis.Status.Ok);
		testing.expect_value(t, actual, handle);
		testing.expect_value(t, hit.t, f32(2.5));
	}
	// read-only queries neither rebuild nor mutate any tree topology
	testing.expect_value(t, bulk_tree_signature(tree), built);
	for index in 0 ..< 32
	{
		testing.expect_value(t, entasis.static_remove(&world, handles[index], .None), entasis.Status.Ok);
	}
	for index in 32 ..< 64
	{
		moved := descriptions[index].pose;
		moved.position.z += 5;
		testing.expect_value(t, entasis.static_set_pose(&world, handles[index], moved, .None), entasis.Status.Ok);
		hit, hit_status := entasis.ray_cast_closest(&world, entasis.ray({moved.position.x, moved.position.y, moved.position.z - 3}, {0, 0, 1}, 6));
		testing.expect_value(t, hit_status, entasis.Status.Ok);
		actual, handle_status := entasis.collidable_static_handle(hit.collidable);
		testing.expect_value(t, handle_status, entasis.Status.Ok);
		testing.expect_value(t, actual, handles[index]);
	}
	testing.expect_value(t, physics.tree_validate(tree), physics.Physics_Status.Ok);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&pool), bytes_before);
}

@(test)
bulk_static_small_edits_match_incremental_topology :: proc(t: ^testing.T)
{
	worlds: [2]entasis.World;
	for &world in worlds
	{
		if !testing.expect_value(t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok)
		{
			return;
		}
	}
	defer entasis.world_destroy(&worlds[0]);
	defer entasis.world_destroy(&worlds[1]);
	for &world, version in worlds
	{
		shape, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
		testing.expect_value(t, shape_status, entasis.Status.Ok);
		descriptions: [64]entasis.Static_Description;
		handles: [64]entasis.Static_Handle;
		bulk_test_descriptions(shape, descriptions[:]);
		written, status := entasis.static_add_batch(&world, descriptions[:32], handles[:32], .None);
		testing.expect_value(t, written, 32);
		testing.expect_value(t, status, entasis.Status.Ok);
		if version == 0
		{
			written, status = entasis.static_add_batch(&world, descriptions[32:40], handles[32:40], .None);
			testing.expect_value(t, written, 8);
			testing.expect_value(t, status, entasis.Status.Ok);
		}
		else
		{
			for index in 32 ..< 40
			{
				_, add_status := entasis.static_add(&world, descriptions[index], .None);
				testing.expect_value(t, add_status, entasis.Status.Ok);
			}
		}
	}
	a, a_status := entasis.world_borrow_simulation(&worlds[0]);
	b, b_status := entasis.world_borrow_simulation(&worlds[1]);
	testing.expect_value(t, a_status, entasis.Status.Ok);
	testing.expect_value(t, b_status, entasis.Status.Ok);
	testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), bulk_tree_signature(&b.broad_phase.static_tree));
	before := bulk_tree_signature(&a.broad_phase.static_tree);
	written, status := entasis.static_add_batch(&worlds[0], nil, nil, .None);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), before);
	// at the exact population boundary, the bulk tree must match the existing
	// binned rebuild on the same successfully committed incremental additions
	for &world, version in worlds
	{
		extra: [40]entasis.Static_Description;
		extra_handles: [40]entasis.Static_Handle;
		shape, shape_status := entasis.shape_add(&world, entasis.sphere(0.5));
		testing.expect_value(t, shape_status, entasis.Status.Ok);
		bulk_test_descriptions(shape, extra[:]);
		for &value in extra
		{
			value.pose.position.z += 20;
		}
		if version == 0
		{
			count, add_status := entasis.static_add_batch(&world, extra[:], extra_handles[:], .None);
			testing.expect_value(t, count, len(extra));
			testing.expect_value(t, add_status, entasis.Status.Ok);
		}
		else
		{
			for value in extra
			{
				_, add_status := entasis.static_add(&world, value, .None);
				testing.expect_value(t, add_status, entasis.Status.Ok);
			}
			testing.expect_value(t, physics.tree_rebuild_binned(&b.broad_phase.static_tree), physics.Physics_Status.Ok);
		}
	}
	testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), bulk_tree_signature(&b.broad_phase.static_tree));
}

@(test)
bulk_static_failure_keeps_committed_prefix_without_rebuild :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	shape, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
	testing.expect_value(t, shape_status, entasis.Status.Ok);
	descriptions: [4]entasis.Static_Description;
	handles: [4]entasis.Static_Handle;
	bulk_test_descriptions(shape, descriptions[:]);
	written, status := entasis.static_add_batch(&world, descriptions[:], handles[:3], .None);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
	descriptions[2].pose.orientation = {};
	written, status = entasis.static_add_batch(&world, descriptions[:], handles[:], .None);
	testing.expect_value(t, written, 2);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	for index in 0 ..< 4
	{
		testing.expect_value(t, entasis.static_handle_is_valid(handles[index]), index < 2);
	}
	simulation, borrow_status := entasis.world_borrow_simulation(&world);
	testing.expect_value(t, borrow_status, entasis.Status.Ok);
	testing.expect_value(t, simulation.statics.count, 2);
	testing.expect_value(t, physics.tree_validate(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
	// invalid first item leaves the existing topology and all handles untouched/invalid
	before := bulk_tree_signature(&simulation.broad_phase.static_tree);
	written, status = entasis.static_add_batch(&world, descriptions[2:], handles[2:], .None);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	testing.expect_value(t, bulk_tree_signature(&simulation.broad_phase.static_tree), before);
}

@(test)
bulk_static_rebuild_keeps_sleeping_mapping_and_awakening_policy :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		world: entasis.World;
		if !testing.expect_value(t, entasis.world_init(&world, sleeping_world_description()), entasis.Status.Ok)
		{
			return;
		}
		shape, shape_status := entasis.shape_add(&world, entasis.sphere(0.5));
		testing.expect_value(t, shape_status, entasis.Status.Ok);
		body, body_status := entasis.body_add(&world, sleeping_body_description(shape));
		testing.expect_value(t, body_status, entasis.Status.Ok);
		if !sleep_body_automatically(t, &world, body)
		{
			_ = entasis.world_destroy(&world);
			return;
		}
		descriptions: [64]entasis.Static_Description;
		handles: [64]entasis.Static_Handle;
		bulk_test_descriptions(shape, descriptions[:]);
		// a grazing overlap tests awakening without touching the closest test ray
		descriptions[63].pose.position = {0.9, 0, 0};
		written, status := entasis.static_add_batch(&world, descriptions[:], handles[:], policy);
		testing.expect_value(t, written, len(handles));
		testing.expect_value(t, status, entasis.Status.Ok);
		sleeping, sleeping_status := entasis.body_is_sleeping(&world, body);
		testing.expect_value(t, sleeping_status, entasis.Status.Ok);
		testing.expect_value(t, sleeping, policy == .None);
		hit, hit_status := entasis.ray_cast_closest(&world, entasis.ray({-3, 0, 0}, {1, 0, 0}, 6));
		testing.expect_value(t, hit_status, entasis.Status.Ok);
		actual, handle_status := entasis.collidable_body_handle(hit.collidable);
		testing.expect_value(t, handle_status, entasis.Status.Ok);
		testing.expect_value(t, actual, body);
		testing.expect_value(t, hit.t, f32(2.5));
		testing.expect_value(t, entasis.body_awaken(&world, body), entasis.Status.Ok);
		testing.expect_value(t, entasis.static_remove(&world, handles[9], .None), entasis.Status.Ok);
		simulation, borrow_status := entasis.world_borrow_simulation(&world);
		testing.expect_value(t, borrow_status, entasis.Status.Ok);
		testing.expect_value(t, physics.tree_validate(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.tree_validate(&simulation.broad_phase.active_tree), physics.Physics_Status.Ok);
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	}
}
