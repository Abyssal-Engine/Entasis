package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

bulk_direct_test_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	value := (^Custom_Sphere)(shape)^;
	if value.radius == -1
	{
		return {}, .Not_Found;
	}
	if value.radius == -2
	{
		return {min={1, 1, 1}, max={-1, -1, -1}}, .Ok;
	}
	return custom_sphere_bounds(shape, orientation, registry);
}

@(test)
bulk_direct_empty_prefix_errors_match_incremental_handles_and_topology :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		for failure_kind in 0 ..< 4
		{
			for failure_index in ([5]int{0, 1, 2, 17, 63})
			{
				worlds: [2]entasis.World;
				counts: [2]int;
				statuses: [2]entasis.Status;
				handle_sets: [2][64]entasis.Static_Handle;
				for &world, version in worlds
				{
					description := batch_world_description();
					description.capacity.statics = 4096;
					testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok);
					shape, shape_status := entasis.shape_add(&world, entasis.box(1, 1, 1));
					testing.expect_value(t, shape_status, entasis.Status.Ok);
					// empty a previously populated tree to exercise reused IDs and stale capacity
					old: [8]entasis.Static_Handle;
					for &handle, index in old
					{
						handle, shape_status = entasis.static_add(&world, entasis.static_body(shape, entasis.pose({f32(index), 0, 0})), .None);
						testing.expect_value(t, shape_status, entasis.Status.Ok);
					}
					for handle in old
					{
						testing.expect_value(t, entasis.static_remove(&world, handle, .None), entasis.Status.Ok);
					}
					values: [64]entasis.Static_Description;
					bulk_test_descriptions(shape, values[:]);
					bad_shape := entasis.Shape_Handle{};
					if failure_kind == 0
					{
						values[failure_index].pose.orientation = {};
					}
					else if failure_kind == 1
					{
						stale, add_status := entasis.shape_add(&world, entasis.sphere(0.25));
						testing.expect_value(t, add_status, entasis.Status.Ok);
						testing.expect_value(t, entasis.shape_remove(&world, stale), entasis.Status.Ok);
						values[failure_index].shape = stale;
					}
					else
					{
						registration := entasis.custom_shape_registration(
							Custom_Sphere, .Convex, bulk_direct_test_bounds, custom_sphere_inertia,
							custom_sphere_ray, custom_sphere_support,
						);
						type_id, register_status := entasis.custom_shape_register(&world, registration);
						testing.expect_value(t, register_status, entasis.Status.Ok);
						bad := Custom_Sphere{radius=-f32(failure_kind - 1)};
						bad_shape, shape_status = entasis.custom_shape_add(&world, type_id, &bad);
						testing.expect_value(t, shape_status, entasis.Status.Ok);
						values[failure_index].shape = bad_shape;
					}
					if version == 0
					{
						counts[version], statuses[version] = entasis.static_add_batch(&world, values[:], handle_sets[version][:], policy);
					}
					else
					{
						for index in 0 ..< len(values)
						{
							handle, add_status := entasis.static_add(&world, values[index], policy);
							statuses[version] = add_status;
							if add_status != .Ok
							{
								break;
							}
							handle_sets[version][index] = handle;
							counts[version] += 1;
						}
					}
					if failure_kind >= 2
					{
						// a failing post-retain bounds callback must not retain a shape reference
						testing.expect_value(t, entasis.shape_remove(&world, bad_shape), entasis.Status.Ok);
					}
				}
				testing.expect_value(t, counts[0], failure_index);
				testing.expect_value(t, counts[0], counts[1]);
				testing.expect_value(t, statuses[0], statuses[1]);
				for index in 0 ..< failure_index
				{
					testing.expect_value(t, handle_sets[0][index], handle_sets[1][index]);
				}
				for index in failure_index ..< 64
				{
					testing.expect(t, !entasis.static_handle_is_valid(handle_sets[0][index]));
				}
				a, a_status := entasis.world_borrow_simulation(&worlds[0]);
				b, b_status := entasis.world_borrow_simulation(&worlds[1]);
				testing.expect_value(t, a_status, entasis.Status.Ok);
				testing.expect_value(t, b_status, entasis.Status.Ok);
				testing.expect_value(t, a.statics.handle_pool.next_index, b.statics.handle_pool.next_index);
				testing.expect_value(t, a.statics.handle_pool.available_id_count, b.statics.handle_pool.available_id_count);
				testing.expect_value(t, physics.tree_validate(&a.broad_phase.static_tree), physics.Physics_Status.Ok);
				testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), bulk_tree_signature(&b.broad_phase.static_tree));
				for &world in worlds
				{
					testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
				}
			}
		}
	}
}

@(test)
bulk_direct_empty_success_matches_rebuilt_incremental_and_active_tree :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		for count in ([6]int{0, 1, 2, 17, 63, 64})
		{
			worlds: [2]entasis.World;
			for &world, version in worlds
			{
				testing.expect_value(t, entasis.world_init(&world, batch_world_description()), entasis.Status.Ok);
				shape, shape_status := entasis.shape_add(&world, entasis.capsule(0.25, 1));
				testing.expect_value(t, shape_status, entasis.Status.Ok);
				body := sleeping_body_description(shape);
				body.activity = entasis.body_activity(-1, 255);
				_, body_status := entasis.body_add(&world, body);
				testing.expect_value(t, body_status, entasis.Status.Ok);
				simulation, borrow_status := entasis.world_borrow_simulation(&world);
				testing.expect_value(t, borrow_status, entasis.Status.Ok);
				active_before := bulk_tree_signature(&simulation.broad_phase.active_tree);
				values: [64]entasis.Static_Description;
				handles: [64]entasis.Static_Handle;
				bulk_test_descriptions(shape, values[:]);
				for &value in values
				{
					value.pose.orientation = {0, 0, 0.6, 0.8};
				}
				if version == 0
				{
					written, status := entasis.static_add_batch(&world, values[:count], handles[:count], policy);
					testing.expect_value(t, written, count);
					testing.expect_value(t, status, entasis.Status.Ok);
				}
				else
				{
					for index in 0 ..< count
					{
						handle, status := entasis.static_add(&world, values[index], policy);
						testing.expect_value(t, status, entasis.Status.Ok);
						handles[index] = handle;
					}
					if count > 1
					{
						testing.expect_value(t, physics.tree_rebuild_binned(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
					}
				}
				testing.expect_value(t, physics.tree_validate(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
				testing.expect_value(t, bulk_tree_signature(&simulation.broad_phase.active_tree), active_before);
				for index in 0 ..< count
				{
					state, status := entasis.static_get(&world, handles[index]);
					testing.expect_value(t, status, entasis.Status.Ok);
					testing.expect_value(t, state, values[index]);
					testing.expect_value(t, physics.collidable_reference_raw_handle(simulation.broad_phase.static_leaves.memory[index]), handles[index].value);
				}
			}
			a, _ := entasis.world_borrow_simulation(&worlds[0]);
			b, _ := entasis.world_borrow_simulation(&worlds[1]);
			testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), bulk_tree_signature(&b.broad_phase.static_tree));
			for &world in worlds
			{
				testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
			}
		}
	}
}
