package queries_tests

import "core:mem"
import "core:testing"
import "base:runtime"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 20
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 131072, power), util.Memory_Status.Ok);
	}
}

cube_hull_data :: proc() -> (points: [8]util.Vector3, face_starts: [6]i32, face_indices: [24]i32)
{
	points = {
		{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
	};
	face_starts = {0, 4, 8, 12, 16, 20};
	face_indices = {
		0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3,
		1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2,
	};
	return;
}

rebuild_target_tree :: proc(
	t: ^testing.T, tree: ^physics.Tree, shapes: ^physics.Shape_Registry,
	targets: util.Buffer(physics.Shape_Query_Target),
)
{
	TARGET_CAPACITY :: 16;
	testing.expect(t, targets.length <= TARGET_CAPACITY);
	bounds: [TARGET_CAPACITY]util.Bounding_Box;
	for index in 0 ..< targets.length
	{
		bounds[index], _ = physics.shape_registry_compute_world_bounds(
			shapes, targets.memory[index].shape, targets.memory[index].pose,
		);
	}
	testing.expect_value(t, physics.tree_build(tree, &bounds[0], int(targets.length)), physics.Physics_Status.Ok);
}

initialize_target_tree :: proc(
	t: ^testing.T, tree: ^physics.Tree, shapes: ^physics.Shape_Registry,
	targets: util.Buffer(physics.Shape_Query_Target), pool: ^util.Buffer_Pool,
)
{
	testing.expect_value(t, physics.tree_initialize(tree, int(targets.length), pool), physics.Physics_Status.Ok);
	rebuild_target_tree(t, tree, shapes, targets);
}

@(test)
ray_collectors_report_earliest_all_hits_capacity_and_wide_tail_lanes :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	pose_near := physics.rigid_pose_identity();
	pose_near.position = {0, 0, 0};
	pose_far := physics.rigid_pose_identity();
	pose_far.position = {5, 0, 0};
	targets_data := [2]physics.Shape_Query_Target{
		{sphere_index, pose_far, 50},
		{sphere_index, pose_near, 10},
	};
	targets := util.Buffer(physics.Shape_Query_Target){memory=&targets_data[0], length=2, id=-1};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	ray := physics.Tree_Ray{origin={-10, 0, 0}, direction={1, 0, 0}, maximum_t=30};
	earliest_data: [1]physics.Ray_Query_Hit;
	earliest: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&earliest, {memory=&earliest_data[0], length=1, id=-1}, .Earliest,
		), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.query_ray_targets(&shapes, &tree, targets, ray, &earliest, &pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, earliest.count, 1);
	testing.expect_value(t, earliest.hits.memory[0].target_id, i32(10));
	testing.expect(t, abs(earliest.hits.memory[0].t - 9) < 1e-5);
	all_data: [2]physics.Ray_Query_Hit;
	all_hits: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&all_hits, {memory=&all_data[0], length=2, id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.query_ray_targets(&shapes, &tree, targets, ray, &all_hits, &pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, all_hits.count, 2);
	limited_data: [1]physics.Ray_Query_Hit;
	limited: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&limited, {memory=&limited_data[0], length=1, id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.query_ray_targets(&shapes, &tree, targets, ray, &limited, &pool),
		physics.Physics_Status.Capacity_Missing
	);
	testing.expect_value(t, limited.count, 1);
	wide := physics.Wide_Ray_Query{lane_count=3};
	wide.rays[0] = ray;
	wide.rays[1] = {origin={-10, 3, 0}, direction={1, 0, 0}, maximum_t=30};
	wide.rays[2] = {origin={0, 0, 0}, direction={1, 0, 0}, maximum_t=30};
	wide_result := physics.wide_ray_test_shape(&shapes, targets_data[1], wide);
	testing.expect_value(t, wide_result.hits[0].state, physics.Reference_State.Present);
	testing.expect_value(t, wide_result.hits[1].state, physics.Reference_State.Missing);
	testing.expect_value(t, wide_result.hits[2].state, physics.Reference_State.Present);
	for lane in 3 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, wide_result.hits[lane].state, physics.Reference_State.Missing);
		testing.expect_value(t, wide_result.status[lane], physics.Physics_Status.Ok);
	}
}

@(test)
ray_batcher_and_mesh_all_hit_paths_preserve_request_and_child_order :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	triangles := [2]physics.Triangle{
		{{-2, 0, -2}, {2, 0, -2}, {0, 0, 2}},
		{{-2, -2, -2}, {2, -2, -2}, {0, -2, 2}},
	};
	mesh: physics.Mesh;
	testing.expect_value(
		t,
		physics.mesh_create(&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool),
		physics.Physics_Status.Ok
	);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);
	target := physics.Shape_Query_Target{mesh_index, physics.rigid_pose_identity(), 7};
	ray := physics.Tree_Ray{origin={0, 5, 0}, direction={0, -1, 0}, maximum_t=20};
	hit_data: [2]physics.Ray_Query_Hit;
	collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&collector, {memory=&hit_data[0], length=2, id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.query_ray_shape(&shapes, target, ray, &collector, &pool),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, collector.count, 2);
	testing.expect_value(t, collector.hits.memory[0].child_index, i32(0));
	testing.expect_value(t, collector.hits.memory[1].child_index, i32(1));
	request_data: [1]physics.Ray_Batcher_Request;
	batcher: physics.Ray_Batcher;
	testing.expect_value(t, physics.ray_batcher_initialize(
			&batcher, {memory=&request_data[0], length=1, id=-1}, &pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ray_batcher_add(&batcher, target, ray), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ray_batcher_add(&batcher, target, ray), physics.Physics_Status.Capacity_Missing);
	collector.count = 0;
	testing.expect_value(t, physics.ray_batcher_flush(&batcher, &shapes, &collector), physics.Physics_Status.Ok);
	testing.expect_value(t, batcher.count, 0);
	testing.expect_value(t, collector.count, 2);
}

@(test)
overlap_volume_and_sweep_queries_match_bruteforce_targets_and_earliest_policy :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	query_shape, _ := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	target_shape, _ := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	near_pose := physics.rigid_pose_identity();
	near_pose.position = {1.5, 0, 0};
	far_pose := physics.rigid_pose_identity();
	far_pose.position = {5, 0, 0};
	target_data := [2]physics.Shape_Query_Target{
		{target_shape, near_pose, 1},
		{target_shape, far_pose, 2},
	};
	targets := util.Buffer(physics.Shape_Query_Target){memory=&target_data[0], length=2, id=-1};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	overlap_data: [2]physics.Overlap_Query_Hit;
	overlaps := physics.Overlap_Query_Collector{hits={memory=&overlap_data[0], length=2, id=-1}};
	testing.expect_value(t, physics.query_overlap(
			query_shape, physics.rigid_pose_identity(), &tree, targets, &shapes, &collisions, &overlaps, &pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, overlaps.count, 1);
	testing.expect_value(t, overlaps.hits.memory[0].target_id, i32(1));
	volume_data: [2]physics.Volume_Query_Hit;
	volumes := physics.Volume_Query_Collector{hits={memory=&volume_data[0], length=2, id=-1}};
	testing.expect_value(t, physics.query_volume(
			{min={-2, -2, -2}, max={2, 2, 2}}, &tree, targets, &shapes, &volumes, &pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, volumes.count, 1);
	testing.expect_value(t, volumes.hits.memory[0].target_id, i32(1));
	target_data[0].pose.position = {5, 0, 0};
	target_data[1].pose.position = {8, 0, 0};
	rebuild_target_tree(t, &tree, &shapes, targets);
	sweep_data: [1]physics.Sweep_Query_Hit;
	sweep_collector := physics.Sweep_Query_Collector{
		hits={memory=&sweep_data[0], length=1, id=-1},
		mode=.Earliest,
	};
	testing.expect_value(t, physics.query_sweep_targets(
			query_shape, physics.rigid_pose_identity(), {linear={1, 0, 0}}, &tree, targets,
			10, 1e-5, 1e-5, 64, &shapes, &collisions, &sweeps, &sweep_collector, &pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, sweep_collector.count, 1);
	testing.expect_value(t, sweep_collector.hits.memory[0].target_id, i32(1));
	testing.expect(t, abs(sweep_collector.hits.memory[0].sweep.t1 - 3) < 2e-4);
}

@(test)
query_hot_collectors_make_no_general_allocator_calls_after_buffer_startup :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	query_shape, query_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	target_shape, target_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, query_status, physics.Physics_Status.Ok);
	testing.expect_value(t, target_status, physics.Physics_Status.Ok);
	target_data := [2]physics.Shape_Query_Target{
		{target_shape, {position={1.5, 0, 0}, orientation=util.quaternion_identity()}, 1},
		{target_shape, {position={5, 0, 0}, orientation=util.quaternion_identity()}, 2},
	};
	targets := util.Buffer(physics.Shape_Query_Target){memory=&target_data[0], length=2, id=-1};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	ray_hits: [2]physics.Ray_Query_Hit;
	ray_collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&ray_collector, {memory=&ray_hits[0], length=2, id=-1}, .All,
		), physics.Physics_Status.Ok);
	overlap_hits: [2]physics.Overlap_Query_Hit;
	overlap_collector := physics.Overlap_Query_Collector{
		hits={memory=&overlap_hits[0], length=2, id=-1},
	};
	volume_hits: [2]physics.Volume_Query_Hit;
	volume_collector := physics.Volume_Query_Collector{
		hits={memory=&volume_hits[0], length=2, id=-1},
	};
	ray := physics.Tree_Ray{origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=20};
	tracker := (^mem.Tracking_Allocator)(context.allocator.data);
	allocation_count_before := tracker.total_allocation_count;
	hot_status := physics.Physics_Status.Ok;
	for _ in 0 ..< 1024
	{
		ray_collector.count = 0;
		overlap_collector.count = 0;
		volume_collector.count = 0;
		hot_status = physics.query_ray_targets(&shapes, &tree, targets, ray, &ray_collector, &pool);
		if hot_status != .Ok
		{
			break;
		}
		hot_status = physics.query_overlap(
			query_shape, physics.rigid_pose_identity(), &tree, targets, &shapes, &collisions, &overlap_collector,
			&pool,
		);
		if hot_status != .Ok
		{
			break;
		}
		hot_status = physics.query_volume(
			{min={-2, -2, -2}, max={2, 2, 2}}, &tree, targets, &shapes, &volume_collector, &pool,
		);
		if hot_status != .Ok
		{
			break;
		}
	}
	allocation_count_after := tracker.total_allocation_count;
	testing.expect_value(t, hot_status, physics.Physics_Status.Ok);
	testing.expect_value(t, allocation_count_after, allocation_count_before);
}

@(test)
wide_ray_routes_match_scalar_results_for_every_convex_shape_and_mask_tails :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.75, 1};
	box := physics.Box{1, 0.75, 1.25};
	triangle := physics.Triangle{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}};
	cylinder := physics.Cylinder{1, 1};
	points, face_starts, face_indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
			&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
		), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	indices: [6]physics.Typed_Index;
	statuses: [6]physics.Physics_Status;
	indices[0], statuses[0] = physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	indices[1], statuses[1] = physics.shape_registry_add(&shapes, physics.CAPSULE_TYPE_ID, &capsule);
	indices[2], statuses[2] = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	indices[3], statuses[3] = physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	indices[4], statuses[4] = physics.shape_registry_add(&shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	indices[5], statuses[5] = physics.shape_registry_add(&shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	for status in statuses
	{
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	pose := physics.rigid_pose_identity();
	pose.orientation = util.quaternion_from_axis_angle({0, 1, 0}, 0.23);
	query := physics.Wide_Ray_Query{lane_count=util.PRODUCTION_LANE_COUNT};
	query.rays[0] = {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=20};
	query.rays[1] = {origin={0, -5, 0}, direction={0, 1, 0}, maximum_t=20};
	query.rays[2] = {origin={0, 0, -5}, direction={0, 0, 1}, maximum_t=20};
	query.rays[3] = {origin={-5, 4, 0}, direction={1, 0, 0}, maximum_t=20};
	query.rays[4] = {origin={0.1, 0.1, 0.1}, direction={0.3, 1, 0.2}, maximum_t=20};
	query.rays[5] = {origin={0, 0, 0}, direction={0, 1, 0}, maximum_t=20};
	query.rays[6] = {origin={0.25, 4, 0.25}, direction={0, -1, 0}, maximum_t=20};
	query.rays[7] = {origin={0.2, 3, 0.2}, direction={1e-7, -1, 1e-7}, maximum_t=20};
	for type_id in 0 ..< len(indices)
	{
		target := physics.Shape_Query_Target{indices[type_id], pose, i32(type_id)};
		wide := physics.wide_ray_test_shape(&shapes, target, query);
		for lane in 0 ..< query.lane_count
		{
			scalar, scalar_status := physics.shape_registry_ray_test(&shapes, target.shape, pose, query.rays[lane]);
			testing.expect_value(t, wide.status[lane], scalar_status);
			testing.expect_value(t, wide.hits[lane].state, scalar.state);
			if scalar.state == .Present
			{
				testing.expect(t, abs(wide.hits[lane].t-scalar.t) < 2e-5);
				testing.expect(t, util.vector3_distance(wide.hits[lane].normal, scalar.normal) < 2e-5);
			}
		}
		for lane in query.lane_count ..< util.PRODUCTION_LANE_COUNT
		{
			testing.expect_value(t, wide.hits[lane].state, physics.Reference_State.Missing);
			testing.expect_value(t, wide.status[lane], physics.Physics_Status.Ok);
		}
	}
}

@(test)
ray_batcher_executes_full_wide_bundle_and_tail_without_reordering_results :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	target := physics.Shape_Query_Target{sphere_index, physics.rigid_pose_identity(), 77};
	request_data: [9]physics.Ray_Batcher_Request;
	batcher: physics.Ray_Batcher;
	testing.expect_value(t, physics.ray_batcher_initialize(
			&batcher, {memory=&request_data[0], length=i32(len(request_data)), id=-1}, &pool,
		), physics.Physics_Status.Ok);
	for request_index in 0 ..< len(request_data)
	{
		raycast := physics.Tree_Ray{
			origin={-5, f32(request_index)*0.05, 0}, direction={1, 0, 0}, maximum_t=20,
		};
		testing.expect_value(t, physics.ray_batcher_add(&batcher, target, raycast), physics.Physics_Status.Ok);
	}
	hit_data: [9]physics.Ray_Query_Hit;
	collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&collector, {memory=&hit_data[0], length=i32(len(hit_data)), id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ray_batcher_flush(&batcher, &shapes, &collector), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, len(hit_data));
	for request_index in 0 ..< len(hit_data)
	{
		testing.expect_value(t, collector.hits.memory[request_index].target_id, i32(77));
		testing.expect(t, collector.hits.memory[request_index].t >= 4 && collector.hits.memory[request_index].t <= 5);
	}
}

@(test)
unsupported_sweep_target_does_not_hide_a_later_supported_hit :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 6, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	triangles := [2]physics.Triangle{
		{{0, -2, -2}, {0, 2, -2}, {0, 2, 2}},
		{{0, -2, -2}, {0, 2, 2}, {0, -2, 2}},
	};
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(
			&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool,
		), physics.Physics_Status.Ok);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	unsupported_pose := physics.rigid_pose_identity();
	unsupported_pose.position = {-2, 0, 0};
	supported_pose := physics.rigid_pose_identity();
	supported_pose.position = {3, 0, 0};
	target_data := [2]physics.Shape_Query_Target{
		{mesh_index, unsupported_pose, 41},
		{sphere_index, supported_pose, 42},
	};
	targets := util.Buffer(physics.Shape_Query_Target){memory=&target_data[0], length=2, id=-1};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	hit_data: [2]physics.Sweep_Query_Hit;
	collector := physics.Sweep_Query_Collector{
		hits={memory=&hit_data[0], length=2, id=-1}, mode=.All,
	};
	query_pose := physics.rigid_pose_identity();
	query_pose.position = {-5, 0, 0};
	testing.expect_value(t, physics.query_sweep_targets(
			mesh_index, query_pose, {linear={1, 0, 0}}, &tree, targets,
			10, 1e-5, 1e-5, 64, &shapes, &collisions, &sweeps, &collector, &pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, 1);
	testing.expect_value(t, collector.hits.memory[0].target_id, i32(42));
	testing.expect_value(t, collector.hits.memory[0].sweep.state, physics.Sweep_Hit_State.Hit);
}

@(test)
iterative_tree_ray_results_match_bruteforce_across_randomized_rays :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{0.65};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	target_data: [8]physics.Shape_Query_Target;
	for target_index in 0 ..< len(target_data)
	{
		pose := physics.rigid_pose_identity();
		pose.position = {f32(target_index)*1.5, f32(target_index%3)*0.35-0.35, f32(target_index%2)*0.4-0.2};
		target_data[target_index] = {sphere_index, pose, i32(100+target_index)};
	}
	targets := util.Buffer(physics.Shape_Query_Target){
		memory=&target_data[0], length=i32(len(target_data)), id=-1,
	};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	seed := u32(0x3141_5926);
	for _ in 0 ..< 48
	{
		seed ~= seed << 13;
		seed ~= seed >> 17;
		seed ~= seed << 5;
		ray := physics.Tree_Ray{
			origin={-3, (f32(seed & 0xff)/255.0)*3-1.5, (f32((seed>>8)&0xff)/255.0)*2-1},
			direction={1, 0.03, -0.02}, maximum_t=20,
		};
		tree_hits_data: [8]physics.Ray_Query_Hit;
		brute_hits_data: [8]physics.Ray_Query_Hit;
		tree_hits: physics.Ray_Query_Collector;
		brute_hits: physics.Ray_Query_Collector;
		testing.expect_value(t, physics.ray_query_collector_initialize(
				&tree_hits, {memory=&tree_hits_data[0], length=i32(len(tree_hits_data)), id=-1}, .All,
			), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.ray_query_collector_initialize(
				&brute_hits, {memory=&brute_hits_data[0], length=i32(len(brute_hits_data)), id=-1}, .All,
			), physics.Physics_Status.Ok);
		testing.expect_value(
			t,
			physics.query_ray_targets(&shapes, &tree, targets, ray, &tree_hits, &pool),
			physics.Physics_Status.Ok
		);
		for target in target_data
		{
			testing.expect_value(
				t,
				physics.query_ray_shape(&shapes, target, ray, &brute_hits, &pool),
				physics.Physics_Status.Ok
			);
		}
		testing.expect_value(t, tree_hits.count, brute_hits.count);
		for tree_index in 0 ..< tree_hits.count
		{
			matched := physics.Reference_State.Missing;
			for brute_index in 0 ..< brute_hits.count
			{
				if tree_hits.hits.memory[tree_index].target_id == brute_hits.hits.memory[brute_index].target_id
				{
					testing.expect(
						t,
						abs(tree_hits.hits.memory[tree_index].t-brute_hits.hits.memory[brute_index].t) < 1e-5
					);
					matched = .Present;
					break;
				}
			}
			testing.expect_value(t, matched, physics.Reference_State.Present);
		}
	}
}

@(test)
closest_tree_ray_and_sweep_results_match_bruteforce_across_nonaxial_queries :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	target_sphere := physics.Sphere{0.55};
	query_sphere := physics.Sphere{0.20};
	target_shape, target_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &target_sphere,
	);
	query_shape, query_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &query_sphere,
	);
	testing.expect_value(t, target_status, physics.Physics_Status.Ok);
	testing.expect_value(t, query_status, physics.Physics_Status.Ok);
	target_data: [12]physics.Shape_Query_Target;
	for target_index in 0 ..< len(target_data)
	{
		pose := physics.rigid_pose_identity();
		pose.position = {
			f32(target_index % 4) * 2.1,
			f32((target_index / 4) % 3) * 1.4 - 1.4,
			f32(target_index % 3) * 0.8 - 0.8,
		};
		target_data[target_index] = {target_shape, pose, i32(200 + target_index)};
	}
	targets := util.Buffer(physics.Shape_Query_Target){
		memory=&target_data[0], length=i32(len(target_data)), id=-1,
	};
	tree: physics.Tree;
	initialize_target_tree(t, &tree, &shapes, targets, &pool);
	defer physics.tree_dispose(&tree);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	seed := u32(0x8c13_5f27);
	for _ in 0 ..< 96
	{
		seed = seed * 1664525 + 1013904223;
		y0 := f32(i32(seed & 0xff) - 128) * (2.8 / 128.0);
		seed = seed * 1664525 + 1013904223;
		z0 := f32(i32(seed & 0xff) - 128) * (2.0 / 128.0);
		seed = seed * 1664525 + 1013904223;
		dy := f32(i32(seed & 0xff) - 128) * (0.18 / 128.0);
		seed = seed * 1664525 + 1013904223;
		dz := f32(i32(seed & 0xff) - 128) * (0.18 / 128.0);
		ray := physics.Tree_Ray{
			origin={-3.0, y0, z0}, direction={0.75, dy, dz}, maximum_t=16,
		};
		tree_ray_hit, brute_ray_hit: physics.Ray_Query_Hit;
		tree_ray_collector, brute_ray_collector: physics.Ray_Query_Collector;
		testing.expect_value(t, physics.ray_query_collector_initialize(
				&tree_ray_collector, {memory=&tree_ray_hit, length=1, id=-1}, .Earliest,
			), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.ray_query_collector_initialize(
				&brute_ray_collector, {memory=&brute_ray_hit, length=1, id=-1}, .Earliest,
			), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.query_ray_targets(
				&shapes, &tree, targets, ray, &tree_ray_collector, &pool,
			), physics.Physics_Status.Ok);
		for target in target_data
		{
			testing.expect_value(t, physics.query_ray_shape(
					&shapes, target, ray, &brute_ray_collector, &pool,
				), physics.Physics_Status.Ok);
		}
		testing.expect_value(t, tree_ray_collector.count, brute_ray_collector.count);
		if tree_ray_collector.count > 0
		{
			testing.expect_value(t, tree_ray_hit.target_id, brute_ray_hit.target_id);
			testing.expect(t, abs(tree_ray_hit.t - brute_ray_hit.t) < 1e-5);
		}
		query_pose := physics.rigid_pose_identity();
		query_pose.position = ray.origin;
		query_velocity := physics.Body_Velocity{linear=ray.direction};
		tree_sweep_hit: physics.Sweep_Query_Hit;
		tree_sweep_collector := physics.Sweep_Query_Collector{
			hits={memory=&tree_sweep_hit, length=1, id=-1}, mode=.Earliest,
		};
		testing.expect_value(t, physics.query_sweep_targets(
				query_shape, query_pose, query_velocity, &tree, targets,
				ray.maximum_t, 1e-5, 1e-5, 64, &shapes, &collisions, &sweeps,
				&tree_sweep_collector, &pool,
			), physics.Physics_Status.Ok);
		brute_sweep_count := 0;
		brute_sweep_target := i32(0);
		brute_sweep_t := ray.maximum_t;
		for target in target_data
		{
			result, status := physics.sweep_task_registry_test(
				&sweeps, query_shape, target.shape, query_pose, target.pose,
				query_velocity, {}, brute_sweep_t, 1e-5, 1e-5, 64,
				&shapes, &collisions, nil, nil, &pool,
			);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			if result.state == .Hit && (brute_sweep_count == 0 || result.t1 < brute_sweep_t)
			{
				brute_sweep_count = 1;
				brute_sweep_target = target.target_id;
				brute_sweep_t = result.t1;
			}
		}
		testing.expect_value(t, tree_sweep_collector.count, brute_sweep_count);
		if tree_sweep_collector.count > 0
		{
			testing.expect_value(t, tree_sweep_hit.target_id, brute_sweep_target);
			testing.expect(t, abs(tree_sweep_hit.sweep.t1 - brute_sweep_t) < 2e-4);
		}
	}
}

@(test)
pathological_tree_queries_grow_caller_and_worker_owned_scratch :: proc(t: ^testing.T)
{
	LEAF_COUNT :: 300;
	WORKER_COUNT :: 4;
	tree_pool: util.Buffer_Pool;
	prepare_pool(t, &tree_pool);
	defer util.buffer_pool_dispose(&tree_pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(
		t, physics.shape_registry_initialize(&shapes, 4, &tree_pool),
		physics.Physics_Status.Ok,
	);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{0.01};
	sphere_index, sphere_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &sphere,
	);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	overlap_sphere := physics.Sphere{2};
	overlap_shape, overlap_shape_status := physics.shape_registry_add(
		&shapes, physics.SPHERE_TYPE_ID, &overlap_sphere,
	);
	testing.expect_value(t, overlap_shape_status, physics.Physics_Status.Ok);
	target_data: [LEAF_COUNT]physics.Shape_Query_Target;
	deep_children: [LEAF_COUNT]physics.Compound_Child;
	tree: physics.Tree;
	testing.expect_value(
		t, physics.tree_initialize(&tree, LEAF_COUNT, &tree_pool),
		physics.Physics_Status.Ok,
	);
	defer physics.tree_dispose(&tree);
	x := f32(1);
	for leaf_index in 0 ..< LEAF_COUNT
	{
		target_data[leaf_index] = {
			shape=sphere_index,
			pose={position={x, 0, 0}, orientation=util.quaternion_identity()},
			target_id=i32(1000 + leaf_index),
		};
		deep_children[leaf_index] = {
			local_position={x, 0, 0},
			local_orientation=util.quaternion_identity(),
			shape_index=sphere_index,
		};
		added_leaf, add_status := physics.tree_add_without_refinement(
			&tree, {min={x, -0.5, -0.5}, max={x, 0.5, 0.5}},
		);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, added_leaf, leaf_index);
		x *= 0.99;
	}
	depth := 0;
	node_index := physics.tree_leaf_node_index(tree.leaves.memory[LEAF_COUNT - 1]);
	for node_index >= 0
	{
		depth += 1;
		node_index = int(tree.metanodes.memory[node_index].parent);
	}
	testing.expect(t, depth > physics.TREE_TRAVERSAL_STACK_CAPACITY);
	deep_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
			&deep_compound, &deep_children[0], len(deep_children), &shapes, &tree_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_dispose(&deep_compound.tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(
			&deep_compound.tree, LEAF_COUNT, &tree_pool,
		), physics.Physics_Status.Ok);
	for child_index in 0 ..< LEAF_COUNT
	{
		added_child, add_status := physics.tree_add_without_refinement(
			&deep_compound.tree, deep_compound.child_bounds.memory[child_index],
		);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, added_child, child_index);
	}
	deep_compound_depth := 0;
	deep_compound_node := physics.tree_leaf_node_index(
		deep_compound.tree.leaves.memory[LEAF_COUNT - 1],
	);
	for deep_compound_node >= 0
	{
		deep_compound_depth += 1;
		deep_compound_node = int(deep_compound.tree.metanodes.memory[deep_compound_node].parent);
	}
	testing.expect(t, deep_compound_depth > physics.TREE_TRAVERSAL_STACK_CAPACITY);
	deep_compound_index, deep_compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &deep_compound,
	);
	testing.expect_value(t, deep_compound_status, physics.Physics_Status.Ok);
	targets := util.Buffer(physics.Shape_Query_Target){
		memory=&target_data[0], length=LEAF_COUNT, id=-1,
	};
	query_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&query_pool, 128),
		util.Memory_Status.Ok,
	);
	defer util.buffer_pool_dispose(&query_pool);
	deep_target_data := [1]physics.Shape_Query_Target{{
			shape=deep_compound_index,
			pose=physics.rigid_pose_identity(),
			target_id=9000,
	}};
	deep_targets := util.Buffer(physics.Shape_Query_Target){
		memory=&deep_target_data[0], length=1, id=-1,
	};
	deep_target_tree: physics.Tree;
	initialize_target_tree(t, &deep_target_tree, &shapes, deep_targets, &tree_pool);
	defer physics.tree_dispose(&deep_target_tree);
	tree_pool_bytes := util.buffer_pool_total_allocated_byte_count(&tree_pool);
	query_pool_bytes := util.buffer_pool_total_allocated_byte_count(&query_pool);
	ray := physics.Tree_Ray{
		origin={-2, 0, 0}, direction={1, 0, 0}, maximum_t=4,
	};
	ray_hit_data: [LEAF_COUNT]physics.Ray_Query_Hit;
	ray_collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&ray_collector, {memory=&ray_hit_data[0], length=LEAF_COUNT, id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.query_ray_targets(
			&shapes, &tree, targets, ray, &ray_collector, &query_pool,
		),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, ray_collector.count, LEAF_COUNT);
	ray_seen: [LEAF_COUNT]u8;
	for hit_index in 0 ..< ray_collector.count
	{
		target_index := int(ray_collector.hits.memory[hit_index].target_id) - 1000;
		testing.expect(t, target_index >= 0 && target_index < LEAF_COUNT);
		if target_index >= 0 && target_index < LEAF_COUNT
		{
			testing.expect_value(t, ray_seen[target_index], u8(0));
			ray_seen[target_index] = 1;
		}
	}
	for seen in ray_seen
	{
		testing.expect_value(t, seen, u8(1));
	}
	closest_pool_bytes := util.buffer_pool_total_allocated_byte_count(&query_pool);
	closest_hit: physics.Ray_Query_Hit;
	closest_collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&closest_collector, {memory=&closest_hit, length=1, id=-1}, .Earliest,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.query_ray_targets(
			&shapes, &tree, targets, ray, &closest_collector, &query_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, closest_collector.count, 1);
	testing.expect_value(t, closest_hit.target_id, i32(1000 + LEAF_COUNT - 1));
	testing.expect(
		t, util.buffer_pool_total_allocated_byte_count(&query_pool) == closest_pool_bytes,
	);
	ray_batch_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&ray_batch_pool, 128),
		util.Memory_Status.Ok,
	);
	defer util.buffer_pool_dispose(&ray_batch_pool);
	ray_batch_pool_bytes := util.buffer_pool_total_allocated_byte_count(&ray_batch_pool);
	ray_batch_requests: [1]physics.Ray_Batcher_Request;
	ray_batcher: physics.Ray_Batcher;
	testing.expect_value(t, physics.ray_batcher_initialize(
			&ray_batcher, {memory=&ray_batch_requests[0], length=1, id=-1}, &ray_batch_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ray_batcher_add(
			&ray_batcher, deep_target_data[0], ray, 71,
		), physics.Physics_Status.Ok);
	ray_batch_hit_data: [LEAF_COUNT]physics.Ray_Query_Hit;
	ray_batch_collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&ray_batch_collector, {memory=&ray_batch_hit_data[0], length=LEAF_COUNT, id=-1}, .All,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.ray_batcher_flush(
			&ray_batcher, &shapes, &ray_batch_collector,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, ray_batch_collector.count, LEAF_COUNT);
	testing.expect(
		t, util.buffer_pool_total_allocated_byte_count(&ray_batch_pool) >
		ray_batch_pool_bytes,
	);
	volume_hit_data: [LEAF_COUNT]physics.Volume_Query_Hit;
	volume_collector := physics.Volume_Query_Collector{
		hits={memory=&volume_hit_data[0], length=LEAF_COUNT, id=-1},
	};
	testing.expect_value(t, physics.query_volume(
			{min={-0.1, -1, -1}, max={1.1, 1, 1}},
			&tree, targets, &shapes, &volume_collector, &query_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, volume_collector.count, LEAF_COUNT);
	volume_seen: [LEAF_COUNT]u8;
	for hit_index in 0 ..< volume_collector.count
	{
		target_index := int(volume_collector.hits.memory[hit_index].target_id) - 1000;
		testing.expect(t, target_index >= 0 && target_index < LEAF_COUNT);
		if target_index >= 0 && target_index < LEAF_COUNT
		{
			testing.expect_value(t, volume_seen[target_index], u8(0));
			volume_seen[target_index] = 1;
		}
	}
	for seen in volume_seen
	{
		testing.expect_value(t, seen, u8(1));
	}
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(
		t, physics.collision_task_registry_initialize(&collisions),
		physics.Physics_Status.Ok,
	);
	overlap_stack_power, overlap_stack_power_status := util.buffer_pool_power_for_count(
		i32, physics.TREE_TRAVERSAL_STACK_CAPACITY * 2,
	);
	testing.expect_value(t, overlap_stack_power_status, util.Memory_Status.Ok);
	available_before_overlap, available_before_overlap_status :=
	util.buffer_pool_available_slot_count(&query_pool, overlap_stack_power);
	testing.expect_value(t, available_before_overlap_status, util.Memory_Status.Ok);
	overlap_hit_data: [LEAF_COUNT]physics.Overlap_Query_Hit;
	overlap_collector := physics.Overlap_Query_Collector{
		hits={memory=&overlap_hit_data[0], length=LEAF_COUNT, id=-1},
	};
	testing.expect_value(t, physics.query_overlap(
			overlap_shape, physics.rigid_pose_identity(), &tree, targets,
			&shapes, &collisions, &overlap_collector, &query_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, overlap_collector.count, LEAF_COUNT);
	overlap_seen: [LEAF_COUNT]u8;
	for hit_index in 0 ..< overlap_collector.count
	{
		target_index := int(overlap_collector.hits.memory[hit_index].target_id) - 1000;
		testing.expect(t, target_index >= 0 && target_index < LEAF_COUNT);
		if target_index >= 0 && target_index < LEAF_COUNT
		{
			testing.expect_value(t, overlap_seen[target_index], u8(0));
			overlap_seen[target_index] = 1;
		}
	}
	for seen in overlap_seen
	{
		testing.expect_value(t, seen, u8(1));
	}
	available_after_overlap, available_after_overlap_status :=
	util.buffer_pool_available_slot_count(&query_pool, overlap_stack_power);
	testing.expect_value(t, available_after_overlap_status, util.Memory_Status.Ok);
	testing.expect_value(t, available_after_overlap, available_before_overlap);
	testing.expect_value(
		t, physics.sweep_task_registry_initialize(&sweeps),
		physics.Physics_Status.Ok,
	);
	sweep_hit_data: [LEAF_COUNT]physics.Sweep_Query_Hit;
	sweep_collector := physics.Sweep_Query_Collector{
		hits={memory=&sweep_hit_data[0], length=LEAF_COUNT, id=-1},
		mode=.All,
	};
	testing.expect_value(t, physics.query_sweep_targets(
			sphere_index, {position={-2, 0, 0}, orientation=util.quaternion_identity()},
			{linear={1, 0, 0}}, &tree, targets, 4, 1e-5, 1e-5, 32,
			&shapes, &collisions, &sweeps, &sweep_collector, &query_pool,
		), physics.Physics_Status.Ok);
	testing.expect_value(t, sweep_collector.count, LEAF_COUNT);
	sweep_seen: [LEAF_COUNT]u8;
	for hit_index in 0 ..< sweep_collector.count
	{
		target_index := int(sweep_collector.hits.memory[hit_index].target_id) - 1000;
		testing.expect(t, target_index >= 0 && target_index < LEAF_COUNT);
		if target_index >= 0 && target_index < LEAF_COUNT
		{
			testing.expect_value(t, sweep_seen[target_index], u8(0));
			sweep_seen[target_index] = 1;
		}
	}
	for seen in sweep_seen
	{
		testing.expect_value(t, seen, u8(1));
	}
	testing.expect_value(
		t, util.buffer_pool_total_allocated_byte_count(&tree_pool),
		tree_pool_bytes,
	);
	testing.expect(
		t, util.buffer_pool_total_allocated_byte_count(&query_pool) > query_pool_bytes,
	);
	stack_power, stack_power_status := util.buffer_pool_power_for_count(
		i32, physics.TREE_TRAVERSAL_STACK_CAPACITY * 2,
	);
	testing.expect_value(t, stack_power_status, util.Memory_Status.Ok);
	available_stack_count, available_stack_status :=
	util.buffer_pool_available_slot_count(&query_pool, stack_power);
	testing.expect_value(t, available_stack_status, util.Memory_Status.Ok);
	testing.expect(t, available_stack_count > 0);
	shallow_tree: physics.Tree;
	testing.expect_value(
		t, physics.tree_initialize(&shallow_tree, 2, &tree_pool),
		physics.Physics_Status.Ok,
	);
	defer physics.tree_dispose(&shallow_tree);
	for leaf_index in 0 ..< 2
	{
		target := target_data[leaf_index];
		_, add_status := physics.tree_add_without_refinement(
			&shallow_tree, {
				min={target.pose.position.x, -0.5, -0.5},
				max={target.pose.position.x, 0.5, 0.5},
			},
		);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	}
	shallow_targets := util.Buffer(physics.Shape_Query_Target){
		memory=&target_data[0], length=2, id=-1,
	};
	shallow_hit_data: [2]physics.Ray_Query_Hit;
	shallow_collector: physics.Ray_Query_Collector;
	testing.expect_value(t, physics.ray_query_collector_initialize(
			&shallow_collector, {memory=&shallow_hit_data[0], length=2, id=-1}, .All,
		), physics.Physics_Status.Ok);
	tracker := (^mem.Tracking_Allocator)(context.allocator.data);
	allocation_count_before := tracker.total_allocation_count;
	shallow_status := physics.Physics_Status.Ok;
	for _ in 0 ..< 1024
	{
		shallow_collector.count = 0;
		shallow_status = physics.query_ray_targets(
			&shapes, &shallow_tree, shallow_targets, ray, &shallow_collector, nil,
		);
		if shallow_status != .Ok
		{
			break;
		}
	}
	allocation_count_after := tracker.total_allocation_count;
	testing.expect_value(t, shallow_status, physics.Physics_Status.Ok);
	testing.expect_value(t, shallow_collector.count, 2);
	testing.expect_value(t, allocation_count_after, allocation_count_before);
	Parallel_Query_Context :: struct
	{
		shapes:         ^physics.Shape_Registry,
		tree:           ^physics.Tree,
		targets:        util.Buffer(physics.Shape_Query_Target),
		ray:            physics.Tree_Ray,
		hits:           [WORKER_COUNT]util.Buffer(physics.Ray_Query_Hit),
		statuses:       [WORKER_COUNT]physics.Physics_Status,
		counts:         [WORKER_COUNT]int,
		pool_addresses: [WORKER_COUNT]uintptr,
	}
	parallel_worker := proc "contextless" (
		worker_index: int, dispatcher: ^util.Thread_Dispatcher_Boundary,
	)
	{
		context = runtime.default_context();
		query_context := (^Parallel_Query_Context)(dispatcher.unmanaged_context);
		pool, pool_status := dispatcher.worker_pool(dispatcher, worker_index);
		if pool_status != .Ok || pool == nil
		{
			query_context.statuses[worker_index] = .Invalid_Argument;
			return;
		}
		query_context.pool_addresses[worker_index] = uintptr(pool);
		collector: physics.Ray_Query_Collector;
		status := physics.ray_query_collector_initialize(
			&collector, query_context.hits[worker_index], .All,
		);
		if status == .Ok
		{
			status = physics.query_ray_targets(
				query_context.shapes, query_context.tree, query_context.targets,
				query_context.ray, &collector, pool,
			);
		}
		query_context.statuses[worker_index] = status;
		query_context.counts[worker_index] = collector.count;
	}
	parallel_hit_data: [WORKER_COUNT][LEAF_COUNT]physics.Ray_Query_Hit;
	parallel_context := Parallel_Query_Context{
		shapes=&shapes, tree=&tree, targets=targets, ray=ray,
	};
	for worker_index in 0 ..< WORKER_COUNT
	{
		parallel_context.hits[worker_index] = {
			memory=&parallel_hit_data[worker_index][0], length=LEAF_COUNT, id=-1,
		};
	}
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(
		t, util.thread_dispatcher_initialize(&dispatcher, WORKER_COUNT, 65536),
		util.Threading_Status.Ok,
	);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary := util.thread_dispatcher_boundary(&dispatcher);
	testing.expect_value(t, boundary.dispatch(
			boundary, parallel_worker, WORKER_COUNT, &parallel_context,
		), util.Threading_Status.Ok);
	for worker_index in 0 ..< WORKER_COUNT
	{
		testing.expect_value(
			t, parallel_context.statuses[worker_index],
			physics.Physics_Status.Ok,
		);
		testing.expect_value(
			t, parallel_context.counts[worker_index], LEAF_COUNT,
		);
		for other_index in 0 ..< worker_index
		{
			testing.expect(
				t, parallel_context.pool_addresses[worker_index] !=
				parallel_context.pool_addresses[other_index],
			);
		}
	}
}

@(test)
overlap_collector_preserves_count_only_presence_and_complete_payload :: proc(t: ^testing.T)
{
	for kind in ([2]physics.Manifold_Kind{.Convex, .Nonconvex})
	{
		for count in -1 ..= physics.MAXIMUM_MANIFOLD_CONTACT_COUNT
		{
			manifold := physics.Manifold_Result{kind=kind};
			manifold.convex.offset_b = {1, 2, 3};
			manifold.convex.normal = {0, 1, 0};
			manifold.nonconvex.offset_b = {4, 5, 6};
			for index in 0 ..< physics.MAXIMUM_MANIFOLD_CONTACT_COUNT
			{
				// negative depths still count as contacts under the existing query
				// contract. the unused manifold variant must also survive the copy
				manifold.convex.contacts[index] = {{f32(index), 2, 3}, -f32(index + 1), i32(index + 10)};
				manifold.nonconvex.contacts[index] = {{4, f32(index), 6}, -f32(index + 2), {0, 0, 1}, i32(index + 20)};
			}
			if kind == .Convex
			{
				manifold.convex.count = i32(count);
			}
			else
			{
				manifold.nonconvex.count = i32(count);
			}
			original := manifold;
			hits := [1]physics.Overlap_Query_Hit{{target_id=-99}};
			collector := physics.Overlap_Query_Collector{
				hits={memory=&hits[0], length=1, id=util.BUFFER_CALLER_OWNED_ID},
			};
			batcher := physics.Query_Overlap_Batcher{
				collector=&collector, current_target_id=42, state=.Ready,
			};
			_, presence := physics.manifold_deepest_contact(&manifold);
			testing.expect_value(t, physics.query_overlap_pair_completed(&batcher, 0, &manifold), physics.Physics_Status.Ok);
			testing.expect_value(t, collector.count, int(presence == .Present));
			testing.expect_value(t, manifold, original);
			if count > 0
			{
				testing.expect_value(t, hits[0].target_id, i32(42));
				testing.expect_value(t, hits[0].manifold, original);
				before := hits;
				testing.expect_value(t, physics.query_overlap_pair_completed(&batcher, 0, &manifold), physics.Physics_Status.Capacity_Missing);
				testing.expect_value(t, hits, before);
				testing.expect_value(t, collector.count, 1);
				// reusing the exact destination manifold as the input is safe
				collector.count = 0;
				testing.expect_value(t, physics.query_overlap_pair_completed(&batcher, 0, &hits[0].manifold), physics.Physics_Status.Ok);
				testing.expect_value(t, hits[0].manifold, original);
			}
			else
			{
				testing.expect_value(t, hits[0].target_id, i32(-99));
				collector.count = 1;
				testing.expect_value(t, physics.query_overlap_pair_completed(&batcher, 0, &manifold), physics.Physics_Status.Ok);
				testing.expect_value(t, hits[0].target_id, i32(-99));
			}
		}
	}
}
