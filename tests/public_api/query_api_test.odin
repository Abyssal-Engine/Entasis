package public_api_tests

import "core:testing"
import "core:thread"
import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

query_allow_only_static :: proc "contextless" (
	_user_context: rawptr,
	collidable: entasis.Collidable_Reference,
) -> bool
{
	return entasis.collidable_mobility(collidable) == .Static;
}

query_reject_every_child :: proc "contextless" (
	_user_context: rawptr,
	_collidable: entasis.Collidable_Reference,
	_child_index: i32,
) -> bool
{
	return false;
}

Query_Test_Scene :: struct
{
	world:          entasis.World,
	pool:           util.Buffer_Pool,
	sphere_shape:   entasis.Shape_Handle,
	box_shape:      entasis.Shape_Handle,
	dynamic_body:   entasis.Body_Handle,
	kinematic_body: entasis.Body_Handle,
	static_body:    entasis.Static_Handle,
}

query_test_scene_init :: proc(
	t: ^testing.T,
	scene: ^Query_Test_Scene,
	worker_count: int = 1,
) -> bool
{
	if !testing.expect_value(
		t, util.buffer_pool_initialize(&scene.pool, 4096, 4), util.Memory_Status.Ok,
	)
	{
		return false;
	}
	description := small_world_description();
	description.threading.worker_count = i32(worker_count);
	description.capacity.bodies = 8;
	description.capacity.statics = 8;
	description.capacity.shapes_per_type = 8;
	description.capacity.broad_phase_candidates = 32;
	description.capacity.pairs = 32;
	description.capacity.pending_pairs_per_worker = 16;
	if !testing.expect_value(
		t, entasis.world_init_with_pool(&scene.world, description, &scene.pool), entasis.Status.Ok,
	)
	{
		_ = util.buffer_pool_dispose(&scene.pool);
		return false;
	}

	sphere_value := entasis.sphere(1);
	box_value := entasis.box(2, 2, 2);
	status: entasis.Status;
	scene.sphere_shape, status = entasis.shape_add(&scene.world, sphere_value);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return false;
	}
	scene.box_shape, status = entasis.shape_add(&scene.world, box_value);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return false;
	}
	inertia, inertia_status := entasis.shape_inertia(sphere_value, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return false;
	}

	scene.dynamic_body, status = entasis.body_add(
		&scene.world,
		entasis.body_dynamic(
		scene.sphere_shape,
		inertia,
		entasis.pose({0, 0, 0}),
		{},
		entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return false;
	}
	scene.kinematic_body, status = entasis.body_add(
		&scene.world,
		entasis.body_kinematic(
		scene.sphere_shape,
		entasis.pose({8, 0, 0}),
		{},
		entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return false;
	}
	scene.static_body, status = entasis.static_add(
		&scene.world,
		entasis.static_body(scene.box_shape, entasis.pose({5, 0, 0})),
		.None,
	);
	return testing.expect_value(t, status, entasis.Status.Ok);
}

query_test_scene_destroy :: proc(scene: ^Query_Test_Scene)
{
	_ = entasis.world_destroy(&scene.world);
	_ = util.buffer_pool_dispose(&scene.pool);
}

@(test)
ray_any_closest_all_and_filters_match_advanced_collector :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	any_hit, any_status := entasis.ray_cast_any(&scene.world, ray_value);
	testing.expect_value(t, any_status, entasis.Status.Ok);
	testing.expect(t, any_hit);

	closest, closest_status := entasis.ray_cast_closest(&scene.world, ray_value);
	if !testing.expect_value(t, closest_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.collidable_mobility(closest.collidable), entasis.Body_Mobility.Dynamic);
	testing.expect_value(t, closest.t, f32(9));
	body_handle, body_status := entasis.collidable_body_handle(closest.collidable);
	testing.expect_value(t, body_status, entasis.Status.Ok);
	testing.expect_value(t, body_handle, scene.dynamic_body);

	advanced_hits: [1]entasis.Ray_Query_Hit;
	advanced_collector: entasis.Ray_Query_Collector;
	advanced_storage := util.Buffer(entasis.Ray_Query_Hit){
		memory=&advanced_hits[0], length=1, id=util.BUFFER_CALLER_OWNED_ID,
	};
	testing.expect_value(
		t, physics.ray_query_collector_initialize(
		&advanced_collector, advanced_storage, .Earliest,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, entasis.ray_query(&scene.world, ray_value, &advanced_collector, &scene.pool),
		entasis.Status.Ok,
	);
	testing.expect_value(t, advanced_collector.count, 1);
	testing.expect_value(t, closest.t, advanced_hits[0].t);
	testing.expect_value(t, closest.location, advanced_hits[0].location);
	testing.expect_value(t, closest.normal, advanced_hits[0].normal);
	testing.expect_value(t, closest.collidable.packed, u32(advanced_hits[0].target_id));

	hits: [8]entasis.Ray_Hit;
	count, all_status := entasis.ray_cast_all(&scene.world, ray_value, hits[:]);
	if !testing.expect_value(t, all_status, entasis.Status.Ok)
	{
		return;
	}
	if !testing.expect_value(t, count, 3)
	{
		return;
	}
	found_dynamic, found_kinematic, found_static := false, false, false;
	for hit in hits[:count]
	{
		switch entasis.collidable_mobility(hit.collidable)
		{
			case .Dynamic:
				handle, status := entasis.collidable_body_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.dynamic_body);
				testing.expect_value(t, hit.t, f32(9));
				found_dynamic = true;
			case .Kinematic:
				handle, status := entasis.collidable_body_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.kinematic_body);
				testing.expect_value(t, hit.t, f32(17));
				found_kinematic = true;
			case .Static:
				handle, status := entasis.collidable_static_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.static_body);
				testing.expect_value(t, hit.t, f32(14));
				found_static = true;
		}
	}
	testing.expect(t, found_dynamic && found_kinematic && found_static);

	static_filter := entasis.query_filter_mobility(entasis.COLLIDABLE_STATIC);
	static_closest, static_status := entasis.ray_cast_closest(
		&scene.world, ray_value, static_filter,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.collidable_mobility(static_closest.collidable), entasis.Body_Mobility.Static);
	static_handle, static_handle_status := entasis.collidable_static_handle(static_closest.collidable);
	testing.expect_value(t, static_handle_status, entasis.Status.Ok);
	testing.expect_value(t, static_handle, scene.static_body);

	body_filter := entasis.query_filter_mobility(entasis.COLLIDABLE_BODIES);
	body_count, body_query_status := entasis.ray_cast_all(
		&scene.world, ray_value, hits[:], body_filter,
	);
	testing.expect_value(t, body_query_status, entasis.Status.Ok);
	testing.expect_value(t, body_count, 2);
	for index in 0 ..< body_count
	{
		testing.expect(t, entasis.collidable_mobility(hits[index].collidable) != .Static);
	}
}

@(test)
query_callbacks_filter_targets_and_children_without_allocation :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	hits: [8]entasis.Ray_Hit;
	static_count, static_status := entasis.ray_cast_all(
		&scene.world,
		ray_value,
		hits[:],
		{allow=query_allow_only_static},
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, static_count, 1);
	testing.expect_value(
		t, entasis.collidable_mobility(hits[0].collidable), entasis.Body_Mobility.Static,
	);

	child_count, child_status := entasis.ray_cast_all(
		&scene.world,
		ray_value,
		hits[:],
		{allow_child=query_reject_every_child},
	);
	testing.expect_value(t, child_status, entasis.Status.Ok);
	testing.expect_value(t, child_count, 0);
}

@(test)
all_hit_contents_match_the_low_level_collector_without_order_contract :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	facade_hits: [8]entasis.Ray_Hit;
	facade_count, facade_status := entasis.ray_cast_all(
		&scene.world, ray_value, facade_hits[:],
	);
	if !testing.expect_value(t, facade_status, entasis.Status.Ok)
	{
		return;
	}

	advanced_hits: [8]entasis.Ray_Query_Hit;
	advanced_storage := util.Buffer(entasis.Ray_Query_Hit){
		memory=&advanced_hits[0],
		length=i32(len(advanced_hits)),
		id=util.BUFFER_CALLER_OWNED_ID,
	};
	advanced_collector: entasis.Ray_Query_Collector;
	if !testing.expect_value(
		t,
		physics.ray_query_collector_initialize(
		&advanced_collector, advanced_storage, .All,
	),
		physics.Physics_Status.Ok,
	)
	{
		return;
	}
	if !testing.expect_value(
		t,
		entasis.ray_query(&scene.world, ray_value, &advanced_collector, &scene.pool),
		entasis.Status.Ok,
	)
	{
		return;
	}
	advanced_count := int(advanced_collector.count);
	if !testing.expect_value(t, facade_count, advanced_count)
	{
		return;
	}

	matched: [len(advanced_hits)]bool;
	for facade_hit in facade_hits[:facade_count]
	{
		found := false;
		for advanced_hit, advanced_index in advanced_hits[:advanced_count]
		{
			if matched[advanced_index]
			{
				continue;
			}
			if facade_hit.t != advanced_hit.t ||
				facade_hit.location != advanced_hit.location ||
				facade_hit.normal != advanced_hit.normal ||
				facade_hit.collidable.packed != u32(advanced_hit.target_id) ||
				facade_hit.child_index != advanced_hit.child_index ||
				facade_hit.ray_id != advanced_hit.ray_id
			{
				continue;
			}
			matched[advanced_index] = true;
			found = true;
			break;
		}
		testing.expect(t, found);
	}
}

query_all_hit_semantic_mask :: proc(
	t: ^testing.T,
	scene: ^Query_Test_Scene,
	hits: []entasis.Ray_Hit,
) -> u8
{
	mask: u8;
	for hit in hits
	{
		switch entasis.collidable_mobility(hit.collidable)
		{
			case .Dynamic:
				handle, status := entasis.collidable_body_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.dynamic_body);
				testing.expect_value(t, hit.t, f32(9));
				mask |= 1 << 0;
			case .Static:
				handle, status := entasis.collidable_static_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.static_body);
				testing.expect_value(t, hit.t, f32(14));
				mask |= 1 << 1;
			case .Kinematic:
				handle, status := entasis.collidable_body_handle(hit.collidable);
				testing.expect_value(t, status, entasis.Status.Ok);
				testing.expect_value(t, handle, scene.kinematic_body);
				testing.expect_value(t, hit.t, f32(17));
				mask |= 1 << 2;
		}
	}
	return mask;
}

@(test)
all_hit_contents_match_across_world_worker_counts_as_sets :: proc(t: ^testing.T)
{
	one_worker: Query_Test_Scene;
	if !query_test_scene_init(t, &one_worker, 1)
	{
		return;
	}
	defer query_test_scene_destroy(&one_worker);
	two_workers: Query_Test_Scene;
	if !query_test_scene_init(t, &two_workers, 2)
	{
		return;
	}
	defer query_test_scene_destroy(&two_workers);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	one_hits: [8]entasis.Ray_Hit;
	two_hits: [8]entasis.Ray_Hit;
	one_count, one_status := entasis.ray_cast_all(&one_worker.world, ray_value, one_hits[:]);
	two_count, two_status := entasis.ray_cast_all(&two_workers.world, ray_value, two_hits[:]);
	if !testing.expect_value(t, one_status, entasis.Status.Ok) ||
		!testing.expect_value(t, two_status, entasis.Status.Ok) ||
		!testing.expect_value(t, one_count, 3) ||
		!testing.expect_value(t, two_count, one_count)
	{
		return;
	}
	expected := u8(0b111);
	testing.expect_value(
		t, query_all_hit_semantic_mask(t, &one_worker, one_hits[:one_count]), expected,
	);
	testing.expect_value(
		t, query_all_hit_semantic_mask(t, &two_workers, two_hits[:two_count]), expected,
	);
}

@(test)
caller_owned_all_hit_buffers_report_capacity :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	tiny: [1]entasis.Ray_Hit;
	count, status := entasis.ray_cast_all(&scene.world, ray_value, tiny[:]);
	testing.expect_value(t, count, 1);
	testing.expect_value(t, status, entasis.Status.Capacity_Missing);

	empty_count, empty_status := entasis.ray_cast_all(
		&scene.world, ray_value, []entasis.Ray_Hit{},
	);
	testing.expect_value(t, empty_count, 0);
	testing.expect_value(t, empty_status, entasis.Status.Capacity_Missing);
}

@(test)
sweep_overlap_and_volume_helpers_use_registered_shapes_and_filters :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	sweep_hit, sweep_status := entasis.sweep_closest(
		&scene.world,
		scene.sphere_shape,
		entasis.pose({-5, 0, 0}),
		entasis.velocity({1, 0, 0}),
		20,
	);
	if !testing.expect_value(t, sweep_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, entasis.collidable_mobility(sweep_hit.collidable), entasis.Body_Mobility.Dynamic);
	testing.expect(t, sweep_hit.sweep.t1 >= 0);
	testing.expect(t, sweep_hit.sweep.t1 <= 5);

	overlap_hits: [8]entasis.Overlap_Hit;
	overlap_count, overlap_status := entasis.overlap_all(
		&scene.world,
		scene.sphere_shape,
		entasis.pose({0.5, 0, 0}),
		overlap_hits[:],
	);
	if !testing.expect_value(t, overlap_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, overlap_count >= 1);
	for index in 0 ..< overlap_count
	{
		for previous in 0 ..< index
		{
			testing.expect(
				t, overlap_hits[previous].collidable.packed != overlap_hits[index].collidable.packed,
			);
		}
	}

	volume_hits: [8]entasis.Volume_Hit;
	volume_count, volume_status := entasis.volume_all(
		&scene.world,
		{min={-2, -2, -2}, max={2, 2, 2}},
		volume_hits[:],
	);
	if !testing.expect_value(t, volume_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, volume_count, 1);
	testing.expect_value(t, entasis.collidable_mobility(volume_hits[0].collidable), entasis.Body_Mobility.Dynamic);
}

@(test)
query_batch_matches_scalar_results_and_uses_caller_owned_ranges :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	scalar_closest, scalar_closest_status := entasis.ray_cast_closest(&scene.world, ray_value);
	scalar_sweep, scalar_sweep_status := entasis.sweep_closest(
		&scene.world,
		scene.sphere_shape,
		entasis.pose({-5, 0, 0}),
		entasis.velocity({1, 0, 0}),
		20,
	);

	queries := [6]entasis.Query{
		entasis.query_ray_any(ray_value),
		entasis.query_ray_closest(ray_value),
		entasis.query_ray_all(ray_value, entasis.query_output(0, 8)),
		entasis.query_sweep_closest(
			scene.sphere_shape,
			entasis.pose({-5, 0, 0}),
			entasis.velocity({1, 0, 0}),
			20,
		),
		entasis.query_overlap_all(
			scene.sphere_shape, entasis.pose({0.5, 0, 0}), entasis.query_output(0, 8),
		),
		entasis.query_volume_all(
			{min={-2, -2, -2}, max={2, 2, 2}}, entasis.query_output(0, 8),
		),
	};
	results: [6]entasis.Query_Result;
	ray_hits: [8]entasis.Ray_Hit;
	overlap_hits: [8]entasis.Overlap_Hit;
	volume_hits: [8]entasis.Volume_Hit;
	scratch := entasis.query_scratch(ray_hits[:], overlap_hits[:], volume_hits[:]);
	status := entasis.query_batch(&scene.world, queries[:], results[:], &scratch);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}

	testing.expect(t, results[0].hit);
	testing.expect_value(t, results[0].count, i32(1));
	testing.expect_value(t, results[1].status, scalar_closest_status);
	testing.expect_value(t, results[1].ray_hit, scalar_closest);
	testing.expect_value(t, results[2].count, i32(3));
	testing.expect_value(t, results[3].status, scalar_sweep_status);
	testing.expect_value(t, results[3].sweep_hit, scalar_sweep);
	testing.expect(t, results[4].count >= 1);
	testing.expect_value(t, results[5].count, i32(1));
	packet_queries: [37]entasis.Query;
	packet_results: [37]entasis.Query_Result;
	for &query, i in packet_queries
	{
		query = entasis.query_ray_closest(entasis.ray({-10, f32(i%3)*0.75, 0}, {1, 0, 0}, 30));
	}
	packet_queries[18] = entasis.query_ray_closest(entasis.ray({}, {}, 30));
	packet_queries[0] = entasis.query_ray_all(ray_value, entasis.query_output(0, 4));
	packet_queries[36] = entasis.query_ray_all(ray_value, entasis.query_output(4, 4));
	testing.expect_value(t, entasis.query_batch(&scene.world, packet_queries[:], packet_results[:], &scratch), entasis.Status.Not_Found);
	for i in 1 ..< 36
	{
		query_expect_scalar(t, &scene.world, packet_queries[i], packet_results[i]);
	}
	testing.expect_value(t, packet_results[0].count, i32(3));
	testing.expect_value(t, packet_results[36].count, i32(3));
	for i in 0 ..< 3
	{
		testing.expect_value(t, ray_hits[i], ray_hits[i+4]);
	}

	// uninterrupted closest-ray runs cross the staging boundary while reusing
	// poisoned output storage. results outside the supplied subrange stay intact
	run_queries: [65]entasis.Query;
	run_results: [67]entasis.Query_Result;
	poison: entasis.Query_Result = results[1];
	poison.status = .Invalid_Argument;
	poison.count = -7;
	poison.ray_hit.t = -123;
	poison.ray_hit.child_index = 123;
	for run_length in ([4]int{31, 32, 33, 65})
	{
		for offset in 0 ..< 2
		{
			for &query, i in run_queries[:run_length]
			{
				query = entasis.query_ray_closest(entasis.ray({-10, f32((i + offset) % 2) * 100, 0}, {1, 0, 0}, 30));
			}
			for &result in run_results
			{
				result = poison;
			}
			testing.expect_value(
				t, entasis.query_batch(&scene.world, run_queries[:run_length], run_results[1:run_length + 1]), entasis.Status.Not_Found,
			);
			for query, i in run_queries[:run_length]
			{
				query_expect_scalar(t, &scene.world, query, run_results[i + 1]);
			}
			for result, i in run_results
			{
				if i == 0 || i > run_length
				{
					testing.expect_value(t, result.status, poison.status);
					testing.expect_value(t, result.count, poison.count);
					testing.expect_value(t, result.hit, poison.hit);
					testing.expect_value(t, result.ray_hit, poison.ray_hit);
				}
			}
		}
	}

	// a stale active leaf fails one lane, which must not visit its remaining
	// active siblings or the would-be static hit. other lanes still complete.
	// fixture-only access: world_data stores Simulation as its first field.
	// adversarial trees and stale references cannot be created through ordinary queries
	simulation := (^physics.Simulation)(rawptr(scene.world));
	original := simulation.broad_phase.active_leaves.memory[0];
	simulation.broad_phase.active_leaves.memory[0], _ = physics.collidable_reference_create(.Dynamic, (1<<30)-1);
	defer simulation.broad_phase.active_leaves.memory[0] = original;
	for &query, i in packet_queries
	{
		query = entasis.query_ray_closest(entasis.ray({-10+f32(i%2)*12, 0, 0}, {1, 0, 0}, 30));
	}
	_ = entasis.query_batch(&scene.world, packet_queries[:], packet_results[:], nil);
	for query, i in packet_queries
	{
		query_expect_scalar(t, &scene.world, query, packet_results[i]);
	}

	// the x=2 rays hit the active kinematic sphere at t=5 before stale static
	// bounds at t=2 fail resolution. the x=-10 rays clip those bounds out
	simulation.broad_phase.active_leaves.memory[0] = original;
	static_original: physics.Collidable_Reference = simulation.broad_phase.static_leaves.memory[0];
	simulation.broad_phase.static_leaves.memory[0], _ = physics.collidable_reference_create(.Static, (1 << 30) - 1);
	defer simulation.broad_phase.static_leaves.memory[0] = static_original;
	testing.expect_value(
		t, entasis.query_batch(&scene.world, packet_queries[:], packet_results[:], nil), entasis.Status.Not_Found,
	);
	for query, i in packet_queries
	{
		query_expect_scalar(t, &scene.world, query, packet_results[i]);
		if i % 2 != 0
		{
			testing.expect_value(t, packet_results[i].status, entasis.Status.Not_Found);
			testing.expect(t, !packet_results[i].hit);
			testing.expect_value(t, packet_results[i].count, i32(0));
			testing.expect_value(t, packet_results[i].ray_hit, entasis.Ray_Hit{child_index=-1});
		}
		else
		{
			testing.expect_value(t, packet_results[i].status, entasis.Status.Ok);
			testing.expect(t, packet_results[i].hit);
			testing.expect_value(t, packet_results[i].count, i32(1));
			testing.expect_value(t, packet_results[i].ray_hit.t, f32(9));
		}
	}
	simulation.broad_phase.static_leaves.memory[0] = static_original;
}

@(test)
query_wrappers_do_not_grow_the_pool_after_warmup :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);

	ray_value := entasis.ray({-10, 0, 0}, {1, 0, 0}, 30);
	ray_hits: [8]entasis.Ray_Hit;
	overlap_hits: [8]entasis.Overlap_Hit;
	volume_hits: [8]entasis.Volume_Hit;
	_, _ = entasis.ray_cast_all(&scene.world, ray_value, ray_hits[:]);
	_, _ = entasis.sweep_closest(
		&scene.world, scene.sphere_shape, entasis.pose({-5, 0, 0}),
		entasis.velocity({1, 0, 0}), 20,
	);
	_, _ = entasis.overlap_all(
		&scene.world, scene.sphere_shape, entasis.pose({0.5, 0, 0}), overlap_hits[:],
	);
	_, _ = entasis.volume_all(
		&scene.world, {min={-2, -2, -2}, max={2, 2, 2}}, volume_hits[:],
	);
	before := util.buffer_pool_total_allocated_byte_count(&scene.pool);
	for _ in 0 ..< 16
	{
		_, _ = entasis.ray_cast_all(&scene.world, ray_value, ray_hits[:]);
		_, _ = entasis.sweep_closest(
			&scene.world, scene.sphere_shape, entasis.pose({-5, 0, 0}),
			entasis.velocity({1, 0, 0}), 20,
		);
		_, _ = entasis.overlap_all(
			&scene.world, scene.sphere_shape, entasis.pose({0.5, 0, 0}), overlap_hits[:],
		);
		_, _ = entasis.volume_all(
			&scene.world, {min={-2, -2, -2}, max={2, 2, 2}}, volume_hits[:],
		);
	}
	after := util.buffer_pool_total_allocated_byte_count(&scene.pool);
	testing.expect_value(t, after, before);
}

query_expect_scalar :: proc(t: ^testing.T, world: ^entasis.World, query: entasis.Query, result: entasis.Query_Result)
{
	hit, status := entasis.ray_cast_closest(world, query.ray_data.ray, query.ray_data.filter);
	testing.expect_value(t, result.status, status);
	testing.expect_value(t, result.hit, status == .Ok);
	testing.expect_value(t, result.count, i32(status == .Ok));
	testing.expect_value(t, result.ray_hit.collidable, hit.collidable);
	testing.expect_value(t, result.ray_hit.child_index, hit.child_index);
	testing.expect_value(t, result.ray_hit.ray_id, hit.ray_id);
	testing.expect(t, abs(result.ray_hit.t-hit.t) <= 2e-4*max(f32(1), abs(hit.t)));
	testing.expect(t, util.vector3_distance(result.ray_hit.location, hit.location) <= 2e-4*max(f32(1), util.vector3_length(hit.location)));
	testing.expect(t, util.vector3_distance(result.ray_hit.normal, hit.normal) <= 2e-4);
}

Query_Order_State :: struct
{
	results: []entasis.Query_Result,
	calls, failures, published_index: int,
}
Query_Custom_Shape :: struct
{
	radius: f32, state: ^Query_Order_State,
}
query_order_custom_ray :: proc "contextless" (
shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	value := (^Query_Custom_Shape)(shape);
	state := value.state;
	index := state.calls;
	if index > 0 && state.results[index-1].count != i32(index-1 != 7)
	{
		state.failures += 1;
	}
	state.calls += 1;
	if index == 7
	{
		return {}, .Invalid_Description;
	}
	return physics.sphere_ray_test({radius=value.radius}, pose, ray);
}
query_order_allow :: proc "contextless" (user: rawptr, collidable: entasis.Collidable_Reference) -> bool
{
	state := (^Query_Order_State)(user);
	state.calls += 1;
	if state.results[state.published_index].count != 1
	{
		state.failures += 1;
	}
	return entasis.collidable_mobility(collidable) == .Static;
}
query_order_child :: proc "contextless" (user: rawptr, collidable: entasis.Collidable_Reference, child: i32) -> bool
{
	state := (^Query_Order_State)(user);
	state.calls += 1;
	if state.results[state.published_index].count != 1
	{
		state.failures += 1;
	}
	return child < 0; // callback-owned rejection is visible in this input's result
}

@(test)
query_batch_preserves_callback_and_custom_shape_order :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	queries: [21]entasis.Query;
	results: [21]entasis.Query_Result;
	for &query in queries
	{
		query = entasis.query_ray_closest(entasis.ray({-10, 0, 0}, {1, 0, 0}, 30));
	}
	state := Query_Order_State{results=results[:], published_index=9};
	queries[10].ray_data.filter = {allow=query_order_allow, allow_child=query_order_child, user_context=&state};
	testing.expect_value(t, entasis.query_batch(&scene.world, queries[:], results[:], nil), entasis.Status.Not_Found);
	testing.expect(t, state.calls > 0);
	testing.expect_value(t, state.failures, 0);
	testing.expect_value(t, results[10].count, i32(0));
	testing.expect_value(t, results[20].count, i32(1));

	world: entasis.World;
	testing.expect_value(t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok);
	defer entasis.world_destroy(&world);
	registration := entasis.custom_shape_registration(Query_Custom_Shape, .Convex, custom_sphere_bounds,
	custom_sphere_inertia, query_order_custom_ray, custom_sphere_support);
	type_id, registration_status := entasis.custom_shape_register(&world, registration);
	testing.expect_value(t, registration_status, entasis.Status.Ok);
	sphere, _ := entasis.shape_add(&world, entasis.sphere(1));
	static, _ := entasis.static_add(&world, entasis.static_body(sphere, entasis.pose()), .None);
	for &query in queries
	{
		query = entasis.query_ray_closest(entasis.ray({-10, 0, 0}, {1, 0, 0}, 30));
	}
	testing.expect_value(t, entasis.query_batch(&world, queries[:], results[:], nil), entasis.Status.Ok);
	for query, i in queries
	{
		query_expect_scalar(t, &world, query, results[i]);
	}
	testing.expect_value(t, entasis.static_remove(&world, static, .None), entasis.Status.Ok);
	state = {results=results[:]};
	custom, custom_status := entasis.custom_shape_add(&world, type_id, &Query_Custom_Shape{1, &state});
	testing.expect_value(t, custom_status, entasis.Status.Ok);
	children := [1]entasis.Compound_Child{entasis.compound_child(custom, entasis.pose())};
	compound, _ := entasis.shape_import_compound(&world, children[:]);
	_, add_status := entasis.static_add(&world, entasis.static_body(compound, entasis.pose()), .None);
	testing.expect_value(t, add_status, entasis.Status.Ok);
	results = {};
	testing.expect_value(t, entasis.query_batch(&world, queries[:], results[:], nil), entasis.Status.Invalid_Description);
	testing.expect_value(t, state.calls, len(queries));
	testing.expect_value(t, state.failures, 0);
	testing.expect_value(t, results[7].status, entasis.Status.Invalid_Description);
	testing.expect_value(t, results[20].count, i32(1));
}

@(test)
query_batch_covers_builtin_shapes_masks_and_closest_boundaries :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	shapes: [9]entasis.Shape_Handle;
	shapes[0] = scene.sphere_shape;
	shapes[1] = scene.box_shape;
	shapes[2], _ = entasis.shape_add(&scene.world, entasis.capsule(1, 2));
	shapes[3], _ = entasis.shape_add(&scene.world, entasis.cylinder(1, 2));
	triangle := physics.Triangle{{-1, 0, -1}, {0, 0, 1}, {1, 0, -1}};
	shapes[4], _ = entasis.shape_add(&scene.world, triangle);
	points := [8]util.Vector3{{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}};
	starts := [6]i32{0, 4, 8, 12, 16, 20};
	indices := [24]i32{0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(&hull, &points[0], 8, &starts[0], 6, &indices[0], 24, &scene.pool), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &scene.pool);
	shapes[5], _ = entasis.shape_import_convex_hull(&scene.world, &hull);
	children := [2]entasis.Compound_Child{entasis.compound_child(scene.sphere_shape, entasis.pose()), entasis.compound_child(scene.box_shape, entasis.pose({2, 0, 0}))};
	shapes[6], _ = entasis.shape_import_compound(&scene.world, children[:]);
	shapes[7], _ = entasis.shape_import_big_compound(&scene.world, children[:]);
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &scene.pool), physics.Physics_Status.Ok);
	defer physics.mesh_dispose(&mesh, &scene.pool);
	shapes[8], _ = entasis.shape_import_mesh(&scene.world, &mesh);
	rays := [12]entasis.Ray{
		entasis.ray({0, 5, 0}, {0, -1, 0}, 20), entasis.ray({-5, 0, 0}, {2, 0, 0}, 20),
		entasis.ray({-5, 1, 0}, {1, 0, 0}, 20), entasis.ray({-5, 0, 0}, {1, 1e-21, -0.0}, 4),
		entasis.ray({-5, 0, 0}, {1, 1e-19, 0}, 3.999), entasis.ray({0, 0, 0}, {1, 0, 0}, 0),
		entasis.ray({-5, 0, 0}, {1, 1e-16, 0}, 20), entasis.ray({5, 0, 0}, {-1, 0, 0}, 20),
		entasis.ray({-1e10, 0, 0}, {1e10, 0, 0}, 2), entasis.ray({-5, 0, 0}, {1, 0, 0}, 4),
		entasis.ray({-5, 2, 0}, {1, 0, 0}, 20), entasis.ray({0, 5, 0}, {0, -2, 0}, 2.5),
	};
	filters := [6]entasis.Query_Filter{{}, {include=entasis.COLLIDABLE_ALL}, {include={.Static}},
		{include={.Dynamic, .Kinematic}}, {exclude={.Static}}, {include={.Static}, exclude={.Static}}};
	queries: [25]entasis.Query;
	results: [25]entasis.Query_Result;
	for shape in shapes
	{
		target, status := entasis.static_add(&scene.world, entasis.static_body(shape, entasis.pose({0, 0, 20})), .None);
		testing.expect_value(t, status, entasis.Status.Ok);
		// duplicate target gives exact entry and geometry ties
		tied, _ := entasis.static_add(&scene.world, entasis.static_body(shape, entasis.pose({0, 0, 20})), .None);
		for filter in filters
		{
			for &query, i in queries
			{
				ray := rays[i%len(rays)];
				ray.origin.z += 20;
				query = entasis.query_ray_closest(ray, filter);
			}
			_ = entasis.query_batch(&scene.world, queries[:], results[:], nil);
			for query, i in queries
			{
				query_expect_scalar(t, &scene.world, query, results[i]);
			}
		}
		testing.expect_value(t, entasis.static_remove(&scene.world, tied, .None), entasis.Status.Ok);
		testing.expect_value(t, entasis.static_remove(&scene.world, target, .None), entasis.Status.Ok);
	}
	for filter in filters
	{
		for &query in queries
		{
			query = entasis.query_ray_closest(entasis.ray({-10, 0, 0}, {1, 0, 0}, 30), filter);
		}
		_ = entasis.query_batch(&scene.world, queries[:], results[:], nil);
		for query, i in queries
		{
			query_expect_scalar(t, &scene.world, query, results[i]);
		}
	}
	// an equal-distance static hit must not replace the active dynamic hit
	tied_static: entasis.Static_Handle;
	tie_status: entasis.Status;
	tied_static, tie_status = entasis.static_add(
		&scene.world, entasis.static_body(scene.sphere_shape, entasis.pose({0, 0, 0})), .None,
	);
	if !testing.expect_value(t, tie_status, entasis.Status.Ok)
	{
		return;
	}
	for &query in queries
	{
		query = entasis.query_ray_closest(entasis.ray({-10, 0, 0}, {1, 0, 0}, 30));
	}
	testing.expect_value(t, entasis.query_batch(&scene.world, queries[:], results[:], nil), entasis.Status.Ok);
	for query, i in queries
	{
		query_expect_scalar(t, &scene.world, query, results[i]);
		testing.expect_value(t, results[i].ray_hit.t, f32(9));
		testing.expect_value(t, entasis.collidable_mobility(results[i].ray_hit.collidable), entasis.Body_Mobility.Dynamic);
		hit_body: entasis.Body_Handle;
		hit_status: entasis.Status;
		hit_body, hit_status = entasis.collidable_body_handle(results[i].ray_hit.collidable);
		testing.expect_value(t, hit_status, entasis.Status.Ok);
		testing.expect_value(t, hit_body, scene.dynamic_body);
	}
	testing.expect_value(t, entasis.static_remove(&scene.world, tied_static, .None), entasis.Status.Ok);
	sleepers := [1]entasis.Body_Handle{scene.dynamic_body};
	testing.expect_value(t, entasis.bodies_sleep_group(&scene.world, sleepers[:]), entasis.Status.Ok);
	sleeping, sleep_status := entasis.body_is_sleeping(&scene.world, scene.dynamic_body);
	testing.expect_value(t, sleep_status, entasis.Status.Ok);
	testing.expect(t, sleeping);
	for filter in filters
	{
		for &query in queries
		{
			query = entasis.query_ray_closest(entasis.ray({-10, 0, 0}, {1, 0, 0}, 30), filter);
		}
		_ = entasis.query_batch(&scene.world, queries[:], results[:], nil);
		for query, i in queries
		{
			query_expect_scalar(t, &scene.world, query, results[i]);
		}
	}

}

query_make_deep_tree :: proc(tree: ^physics.Tree)
{
	count := tree.leaf_count-1;
	tree.node_count = count;
	for i in 0 ..< count
	{
		node := &tree.nodes.memory[i];
		node.a = {min={1, -1, -1}, max={3, 1, 1}, index=physics.tree_encode_leaf(i), leaf_count=1};
		node.b = {min={1, -1, -1}, max={3, 1, 1}, index=i32(i+1), leaf_count=i32(count-i)};
		tree.metanodes.memory[i] = {parent=i32(i-1), index_in_parent=1};
		tree.leaves.memory[i] = physics.tree_leaf_create(i, 0);
	}
	tree.nodes.memory[count-1].b.index = physics.tree_encode_leaf(count);
	tree.leaves.memory[count] = physics.tree_leaf_create(count-1, 1);
}

Query_Deep_Order :: struct
{
	ids: [512]entasis.Collidable_Reference, count: int,
}
query_deep_allow :: proc "contextless" (user: rawptr, collidable: entasis.Collidable_Reference) -> bool
{
	state := (^Query_Deep_Order)(user);
	state.ids[state.count] = collidable;
	state.count += 1;
	return entasis.collidable_mobility(collidable) == .Static;
}

@(test)
query_batch_deep_trees_continue_without_replay_or_pool_growth :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	testing.expect_value(t, entasis.static_remove(&scene.world, scene.static_body, .None), entasis.Status.Ok);
	children: [301]entasis.Compound_Child;
	for &child in children
	{
		child = entasis.compound_child(scene.box_shape, entasis.pose({2, 0, 0}));
	}
	compound, status := entasis.shape_import_big_compound(&scene.world, children[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	// fixture-only access: world_data stores Simulation as its first field.
	// adversarial trees and stale references cannot be created through ordinary queries
	simulation := (^physics.Simulation)(rawptr(scene.world));
	raw, _, resolve_status := physics.shape_registry_resolve(physics.simulation_shape_registry(simulation), compound);
	testing.expect_value(t, resolve_status, physics.Physics_Status.Ok);
	query_make_deep_tree(&(^physics.Big_Compound)(raw).tree);
	for _ in 0 ..< 301
	{
		_, add_status := entasis.static_add(&scene.world, entasis.static_body(compound, entasis.pose()), .None);
		testing.expect_value(t, add_status, entasis.Status.Ok);
	}
	query_make_deep_tree(&simulation.broad_phase.static_tree);
	before := scene.pool;
	queries: [19]entasis.Query;
	results: [19]entasis.Query_Result;
	for &query in queries
	{
		query = entasis.query_ray_closest(entasis.ray({0, 0, 0}, {1, 0, 0}, 10), {include={.Static}});
	}
	for _ in 0 ..< 3
	{
		testing.expect_value(t, entasis.query_batch(&scene.world, queries[:], results[:], nil), entasis.Status.Ok);
		for query, i in queries
		{
			query_expect_scalar(t, &scene.world, query, results[i]);
		}
	}
	expected_order, actual_order: Query_Deep_Order;
	expected_hit, expected_status := entasis.ray_cast_closest(&scene.world, queries[9].ray_data.ray,
	{include={.Static}, allow=query_deep_allow, user_context=&expected_order});
	queries[9].ray_data.filter = {include={.Static}, allow=query_deep_allow, user_context=&actual_order};
	testing.expect_value(t, entasis.query_batch(&scene.world, queries[:], results[:], nil), entasis.Status.Ok);
	testing.expect_value(t, results[9].status, expected_status);
	testing.expect_value(t, results[9].ray_hit, expected_hit);
	testing.expect_value(t, actual_order, expected_order);
	testing.expect_value(t, actual_order.count, 301);

	for pool, i in scene.pool.pools
	{
		testing.expect_value(t, pool.block_count, before.pools[i].block_count);
		testing.expect_value(t, pool.free_count, before.pools[i].free_count);
		testing.expect_value(t, pool.next_slot, before.pools[i].next_slot);
	}
}

Query_Concurrent_Call :: struct
{
	world: ^entasis.World,
	queries: [19]entasis.Query,
	results: [19]entasis.Query_Result,
	failures: int,
}
query_concurrent_entry :: proc(host: ^thread.Thread)
{
	context = runtime.default_context();
	call := (^Query_Concurrent_Call)(host.data);
	for _ in 0 ..< 50
	{
		status := entasis.query_batch(call.world, call.queries[:], call.results[:], nil);
		if status != .Ok
		{
			call.failures += 1;
		}
	}
}
@(test)
query_batch_concurrent_callers_own_independent_scratch :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	calls: [4]Query_Concurrent_Call;
	hosts: [4]^thread.Thread;
	before := scene.pool;
	for &call, worker in calls
	{
		call.world = &scene.world;
		for &query, i in call.queries
		{
			query = entasis.query_ray_closest(entasis.ray({-10, f32((i+worker)%4)*0.1, 0}, {1, 0, 0}, 30));
		}
		hosts[worker] = thread.create(query_concurrent_entry);
		hosts[worker].data = &call;
		thread.start(hosts[worker]);
	}
	for host in hosts
	{
		thread.join(host);
		thread.destroy(host);
	}
	for call in calls
	{
		testing.expect_value(t, call.failures, 0);
		for query, i in call.queries
		{
			query_expect_scalar(t, &scene.world, query, call.results[i]);
		}
	}
	for pool, i in scene.pool.pools
	{
		testing.expect_value(t, pool.block_count, before.pools[i].block_count);
		testing.expect_value(t, pool.free_count, before.pools[i].free_count);
		testing.expect_value(t, pool.next_slot, before.pools[i].next_slot);
	}
}
