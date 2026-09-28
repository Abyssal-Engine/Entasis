package public_api_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

bulk_populated_seed :: proc (
	t: ^testing.T, world: ^entasis.World, count: int, sleeper: bool,
) -> (entasis.Shape_Handle, entasis.Body_Handle)
{
	description := sleeping_world_description();
	description.capacity.statics = 4096;
	testing.expect_value(t, entasis.world_init(world, description), entasis.Status.Ok);
	shape, shape_status := entasis.shape_add(world, entasis.capsule(0.25, 1));
	testing.expect_value(t, shape_status, entasis.Status.Ok);
	values: [64]entasis.Static_Description;
	bulk_test_descriptions(shape, values[:]);
	for index in 0 ..< count
	{
		values[index].pose.position.z = 30;
		values[index].pose.orientation = {0, 0, 0.6, 0.8};
		_, status := entasis.static_add(world, values[index], .None);
		testing.expect_value(t, status, entasis.Status.Ok);
	}
	// reuse a freed ID without changing the existing leaf population
	old, old_status := entasis.static_add(world, entasis.static_body(shape, entasis.pose({0, 0, 80})), .None);
	testing.expect_value(t, old_status, entasis.Status.Ok);
	testing.expect_value(t, entasis.static_remove(world, old, .None), entasis.Status.Ok);
	body := entasis.Body_Handle{};
	if sleeper
	{
		body, shape_status = entasis.body_add(world, sleeping_body_description(shape));
		testing.expect_value(t, shape_status, entasis.Status.Ok);
		testing.expect(t, sleep_body_automatically(t, world, body));
	}
	return shape, body;
}

bulk_populated_expect_equal :: proc(t: ^testing.T, worlds: ^[2]entasis.World)
{
	a, a_status := entasis.world_borrow_simulation(&worlds[0]);
	b, b_status := entasis.world_borrow_simulation(&worlds[1]);
	testing.expect_value(t, a_status, entasis.Status.Ok);
	testing.expect_value(t, b_status, entasis.Status.Ok);
	testing.expect_value(t, a.statics.count, b.statics.count);
	testing.expect_value(t, a.statics.handle_pool.next_index, b.statics.handle_pool.next_index);
	testing.expect_value(t, a.statics.handle_pool.available_id_count, b.statics.handle_pool.available_id_count);
	testing.expect_value(t, a.broad_phase.static_tree.leaf_count, b.broad_phase.static_tree.leaf_count);
	testing.expect_value(t, a.broad_phase.active_tree.leaf_count, b.broad_phase.active_tree.leaf_count);
	testing.expect_value(t, physics.tree_validate(&a.broad_phase.static_tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&a.broad_phase.active_tree), physics.Physics_Status.Ok);
	testing.expect_value(t, bulk_tree_signature(&a.broad_phase.static_tree), bulk_tree_signature(&b.broad_phase.static_tree));
	testing.expect_value(t, bulk_tree_signature(&a.broad_phase.active_tree), bulk_tree_signature(&b.broad_phase.active_tree));
	for index in 0 ..< a.broad_phase.static_tree.leaf_count
	{
		testing.expect_value(t, a.broad_phase.static_leaves.memory[index], b.broad_phase.static_leaves.memory[index]);
		bounds_a, status_a := physics.tree_get_leaf_bounds(&a.broad_phase.static_tree, index);
		bounds_b, status_b := physics.tree_get_leaf_bounds(&b.broad_phase.static_tree, index);
		testing.expect_value(t, status_a, status_b);
		testing.expect_value(t, bounds_a, bounds_b);
	}
	for index in 0 ..< a.statics.count
	{
		testing.expect_value(t, a.statics.index_to_handle.memory[index], b.statics.index_to_handle.memory[index]);
		testing.expect_value(t, a.statics.statics.memory[index], b.statics.statics.memory[index]);
	}
}

@(test)
bulk_populated_success_matches_incremental_rebuild_and_storage :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		for initial_count in ([3]int{1, 17, 64})
		{
			for count in ([5]int{1, 2, 17, 64, 129})
			{
				worlds: [2]entasis.World;
				handles: [2][129]entasis.Static_Handle;
				for &world, version in worlds
				{
					shape, _ := bulk_populated_seed(t, &world, initial_count, false);
					values: [129]entasis.Static_Description;
					bulk_test_descriptions(shape, values[:]);
					for &value in values
					{
						value.pose.orientation = {0.6, 0, 0, 0.8};
					}
					simulation, _ := entasis.world_borrow_simulation(&world);
					before := util.buffer_pool_total_allocated_byte_count(simulation.pool);
					if version == 0
					{
						written, status := entasis.static_add_batch(&world, values[:count], handles[version][:count], policy);
						testing.expect_value(t, written, count);
						testing.expect_value(t, status, entasis.Status.Ok);
					}
					else
					{
						for index in 0 ..< count
						{
							handle, status := entasis.static_add(&world, values[index], policy);
							testing.expect_value(t, status, entasis.Status.Ok);
							handles[version][index] = handle;
						}
						if count >= initial_count
						{
							testing.expect_value(t, physics.tree_rebuild_binned(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
						}
					}
					testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(simulation.pool), before);
				}
				for index in 0 ..< count
				{
					testing.expect_value(t, handles[0][index], handles[1][index]);
				}
				bulk_populated_expect_equal(t, &worlds);
				for &world in worlds
				{
					testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
				}
			}
		}
	}
}

@(test)
bulk_populated_failed_prefixes_preserve_existing_tree_and_references :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		for sleeper in ([2]bool{false, true})
		{
			for failure_kind in 0 ..< 4
			{
				for failure_index in ([4]int{0, 1, 17, 63})
				{
					worlds: [2]entasis.World;
					handles: [2][64]entasis.Static_Handle;
					counts: [2]int;
					statuses: [2]entasis.Status;
					for &world, version in worlds
					{
						shape, _ := bulk_populated_seed(t, &world, 17, false);
						values: [64]entasis.Static_Description;
						bulk_test_descriptions(shape, values[:]);
						// overlapping prefix may wake a sleeping island before the error
						values[0].pose.position = {0.4, 0, 0};
						bad_shape := entasis.Shape_Handle{};
						if failure_kind == 0
						{
							values[failure_index].pose.orientation = {};
						}
						else if failure_kind == 1
						{
							stale, status := entasis.shape_add(&world, entasis.sphere(0.25));
							testing.expect_value(t, status, entasis.Status.Ok);
							testing.expect_value(t, entasis.shape_remove(&world, stale), entasis.Status.Ok);
							values[failure_index].shape = stale;
						}
						else
						{
							registration := entasis.custom_shape_registration(
								Custom_Sphere, .Convex, bulk_direct_test_bounds, custom_sphere_inertia,
								custom_sphere_ray, custom_sphere_support,
							);
							type_id, status := entasis.custom_shape_register(&world, registration);
							testing.expect_value(t, status, entasis.Status.Ok);
							bad := Custom_Sphere{radius=-f32(failure_kind - 1)};
							bad_shape, status = entasis.custom_shape_add(&world, type_id, &bad);
							testing.expect_value(t, status, entasis.Status.Ok);
							values[failure_index].shape = bad_shape;
						}
						if sleeper
						{
							// registration must precede the first simulation step
							body, status := entasis.body_add(&world, sleeping_body_description(shape));
							testing.expect_value(t, status, entasis.Status.Ok);
							testing.expect(t, sleep_body_automatically(t, &world, body));
						}
						if version == 0
						{
							counts[version], statuses[version] = entasis.static_add_batch(&world, values[:], handles[version][:], policy);
						}
						else
						{
							for index in 0 ..< len(values)
							{
								handle, status := entasis.static_add(&world, values[index], policy);
								statuses[version] = status;
								if status != .Ok
								{
									break;
								}
								handles[version][index] = handle;
								counts[version] += 1;
							}
						}
						if failure_kind >= 2
						{
							testing.expect_value(t, entasis.shape_remove(&world, bad_shape), entasis.Status.Ok);
						}
					}
					testing.expect_value(t, counts[0], failure_index);
					testing.expect_value(t, counts[0], counts[1]);
					testing.expect_value(t, statuses[0], statuses[1]);
					for index in 0 ..< failure_index
					{
						testing.expect_value(t, handles[0][index], handles[1][index]);
					}
					for index in failure_index ..< 64
					{
						testing.expect(t, !entasis.static_handle_is_valid(handles[0][index]));
					}
					bulk_populated_expect_equal(t, &worlds);
					for &world in worlds
					{
						testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
					}
				}
			}
		}
	}
}

@(test)
bulk_populated_sleeping_leaves_survive_rebuild_and_later_mutation :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		worlds: [2]entasis.World;
		bodies: [2]entasis.Body_Handle;
		handles: [2][64]entasis.Static_Handle;
		for &world, version in worlds
		{
			shape, body := bulk_populated_seed(t, &world, 17, true);
			bodies[version] = body;
			values: [64]entasis.Static_Description;
			bulk_test_descriptions(shape, values[:]);
			values[63].pose.position = {0.4, 0, 0};
			simulation, _ := entasis.world_borrow_simulation(&world);
			if version == 0
			{
				written, status := entasis.static_add_batch(&world, values[:], handles[version][:], policy);
				testing.expect_value(t, written, len(values));
				testing.expect_value(t, status, entasis.Status.Ok);
			}
			else
			{
				for index in 0 ..< len(values)
				{
					handle, status := entasis.static_add(&world, values[index], policy);
					testing.expect_value(t, status, entasis.Status.Ok);
					handles[version][index] = handle;
				}
				testing.expect_value(t, physics.tree_rebuild_binned(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
			}
			sleeping, status := entasis.body_is_sleeping(&world, body);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, sleeping, policy == .None);
		}
		bulk_populated_expect_equal(t, &worlds);
		for &world, version in worlds
		{
			testing.expect_value(t, entasis.body_awaken(&world, bodies[version]), entasis.Status.Ok);
			testing.expect_value(t, entasis.static_remove(&world, handles[version][9], .None), entasis.Status.Ok);
			value, status := entasis.static_get(&world, handles[version][10]);
			testing.expect_value(t, status, entasis.Status.Ok);
			value.pose.position = {100, 100, 100};
			testing.expect_value(t, entasis.static_apply(&world, handles[version][10], value, .None), entasis.Status.Ok);
		}
		bulk_populated_expect_equal(t, &worlds);
		for &world in worlds
		{
			testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
		}
	}
}

Bulk_Populated_Bounds_State :: struct
{
	calls:   int,
	fail_on: int,
}

Bulk_Populated_Custom_Shape :: struct
{
	radius: f32,
	state:  ^Bulk_Populated_Bounds_State,
}

bulk_populated_ordered_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	value := (^Bulk_Populated_Custom_Shape)(shape)^;
	value.state.calls += 1;
	if value.state.calls == value.state.fail_on
	{
		return {}, .Not_Found;
	}
	return custom_sphere_bounds(shape, orientation, registry);
}

@(test)
bulk_populated_bounds_callbacks_preserve_pre_and_post_retain_failures :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		for fail_on in ([7]int{0, 1, 2, 35, 36, 127, 128})
		{
			worlds: [2]entasis.World;
			states: [2]Bulk_Populated_Bounds_State;
			counts: [2]int;
			statuses: [2]entasis.Status;
			handles: [2][64]entasis.Static_Handle;
			for &world, version in worlds
			{
				_, _ = bulk_populated_seed(t, &world, 17, false);
				registration := entasis.custom_shape_registration(
					Bulk_Populated_Custom_Shape, .Convex, bulk_populated_ordered_bounds, custom_sphere_inertia,
					custom_sphere_ray, custom_sphere_support,
				);
				type_id, status := entasis.custom_shape_register(&world, registration);
				testing.expect_value(t, status, entasis.Status.Ok);
				value := Bulk_Populated_Custom_Shape{radius=0.5, state=&states[version]};
				shape, shape_status := entasis.custom_shape_add(&world, type_id, &value);
				testing.expect_value(t, shape_status, entasis.Status.Ok);
				states[version] = {fail_on=fail_on};
				values: [64]entasis.Static_Description;
				bulk_test_descriptions(shape, values[:]);
				if version == 0
				{
					counts[version], statuses[version] = entasis.static_add_batch(&world, values[:], handles[version][:], policy);
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
						handles[version][index] = handle;
						counts[version] += 1;
					}
					if statuses[version] == .Ok
					{
						simulation, _ := entasis.world_borrow_simulation(&world);
						testing.expect_value(t, physics.tree_rebuild_binned(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
					}
				}
			}
			testing.expect_value(t, counts[0], counts[1]);
			testing.expect_value(t, statuses[0], statuses[1]);
			testing.expect_value(t, states[0].calls, states[1].calls);
			for index in 0 ..< counts[0]
			{
				testing.expect_value(t, handles[0][index], handles[1][index]);
			}
			bulk_populated_expect_equal(t, &worlds);
			for &world in worlds
			{
				testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
			}
		}
	}
}

@(test)
bulk_populated_multiple_sleeping_sets_keep_owner_remaps_when_awakened :: proc(t: ^testing.T)
{
	for policy in ([2]entasis.Awakening_Policy{.None, .Overlaps})
	{
		worlds: [2]entasis.World;
		bodies: [2][16]entasis.Body_Handle;
		for &world, version in worlds
		{
			shape, _ := bulk_populated_seed(t, &world, 17, false);
			for &handle, index in bodies[version]
			{
				value := sleeping_body_description(shape);
				value.pose.position = {400 + f32(index) * 10, 0, 0};
				status: entasis.Status;
				handle, status = entasis.body_add(&world, value);
				testing.expect_value(t, status, entasis.Status.Ok);
			}
			for _ in 0 ..< 64
			{
				testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
				stats, status := entasis.world_stats(&world);
				testing.expect_value(t, status, entasis.Status.Ok);
				if stats.active_bodies == 0 && stats.sleeping_bodies == 16
				{
					break;
				}
			}
			for handle in bodies[version]
			{
				sleeping, status := entasis.body_is_sleeping(&world, handle);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect(t, sleeping);
			}
			values: [128]entasis.Static_Description;
			handles: [128]entasis.Static_Handle;
			bulk_test_descriptions(shape, values[:]);
			if version == 0
			{
				count, status := entasis.static_add_batch(&world, values[:], handles[:], policy);
				testing.expect_value(t, count, len(values));
				testing.expect_value(t, status, entasis.Status.Ok);
			}
			else
			{
				for value in values
				{
					_, status := entasis.static_add(&world, value, policy);
					testing.expect_value(t, status, entasis.Status.Ok);
				}
				simulation, _ := entasis.world_borrow_simulation(&world);
				testing.expect_value(t, physics.tree_rebuild_binned(&simulation.broad_phase.static_tree), physics.Physics_Status.Ok);
			}
		}
		bulk_populated_expect_equal(t, &worlds);
		for index in ([16]int{15, 0, 14, 1, 13, 2, 12, 3, 11, 4, 10, 5, 9, 6, 8, 7})
		{
			for &world, version in worlds
			{
				testing.expect_value(t, entasis.body_awaken(&world, bodies[version][index]), entasis.Status.Ok);
				hit, hit_status := entasis.ray_cast_closest(&world, entasis.ray({398 + f32(index) * 10, 0, 0}, {1, 0, 0}, 4));
				testing.expect_value(t, hit_status, entasis.Status.Ok);
				handle, handle_status := entasis.collidable_body_handle(hit.collidable);
				testing.expect_value(t, handle_status, entasis.Status.Ok);
				testing.expect_value(t, handle, bodies[version][index]);
			}
			bulk_populated_expect_equal(t, &worlds);
		}
		for &world in worlds
		{
			testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
		}
	}
}
