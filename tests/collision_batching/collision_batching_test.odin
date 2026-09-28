package collision_batching_tests

import "core:mem"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Child_Callback_Mode :: enum u8
{
	Observe,
	Mutate,
	Fail,
}

Callback_Context :: struct
{
	results:              [64]physics.Manifold_Result,
	pair_ids:             [64]i32,
	child_pair_ids:       [64]i32,
	child_as:             [64]i32,
	child_bs:             [64]i32,
	child_contact_counts: [64]i32,
	result_count:         int,
	child_count:          int,
	allowed_child:        i32,
	child_mode:           Child_Callback_Mode,
}

Custom_Wide_Shape :: struct
{
	radius: f32,
}

custom_wide_shape_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = orientation;
	_ = registry;
	radius := (^Custom_Wide_Shape)(shape).radius;
	if radius <= 0
	{
		return {}, .Invalid_Description;
	}
	extent := util.Vector3{radius, radius, radius};
	return {min=util.vector3_negate(extent), max=extent, maximum_radius=radius}, .Ok;
}

custom_wide_shape_inertia :: proc "contextless" (
	shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_inertia({(^Custom_Wide_Shape)(shape).radius}, mass);
}

custom_wide_shape_support :: proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_support({(^Custom_Wide_Shape)(shape).radius}, direction);
}

custom_wide_shape_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_ray_test({(^Custom_Wide_Shape)(shape).radius}, pose, ray);
}

custom_scalar_collision_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: physics.Rigid_Pose,
	speculative_margin: f32, shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	_ = shapes;
	a := physics.Sphere{(^Custom_Wide_Shape)(shape_a).radius};
	b := physics.Sphere{(^Custom_Wide_Shape)(shape_b).radius};
	return physics.sphere_pair_test(&a, &b, pose_a, pose_b, speculative_margin, nil);
}

custom_wide_collision_test :: proc "contextless" (
	bundle: ^physics.Collision_Convex_Wide_Bundle, shapes: ^physics.Shape_Registry,
) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	_ = shapes;
	if bundle == nil || bundle.count <= 0 || bundle.count > util.PRODUCTION_LANE_COUNT
	{
		return {}, .Invalid_Argument;
	}
	result := physics.Collision_Wide_Manifold_Result{kind=.One_Contact};
	for lane in 0 ..< bundle.count
	{
		if bundle.shape_a[lane] == nil || bundle.shape_b[lane] == nil
		{
			return {}, .Invalid_Description;
		}
		a := (^Custom_Wide_Shape)(bundle.shape_a[lane]);
		b := (^Custom_Wide_Shape)(bundle.shape_b[lane]);
		offset_b := util.vector3_wide_read_slot(bundle.offset_b, lane);
		source := physics.Convex_Contact_Manifold{
			count=1,
			normal={1, 0, 0},
		};
		source.contacts[0] = {
			offset={a.radius, 0, 0},
			depth=a.radius+b.radius-offset_b.x,
			feature_id=i32(bundle.count*100+lane),
		};
		status := physics.convex_1_manifold_wide_write_lane(&result.one, lane, source);
		if status != .Ok
		{
			return {}, status;
		}
	}
	return result, .Ok;
}

custom_wide_collision_failure_test :: proc "contextless" (
	bundle: ^physics.Collision_Convex_Wide_Bundle, shapes: ^physics.Shape_Registry,
) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	_ = bundle;
	_ = shapes;
	return {}, .Invalid_Description;
}

sphere_custom_scalar_collision_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: physics.Rigid_Pose,
	speculative_margin: f32, shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	_ = shapes;
	a := (^physics.Sphere)(shape_a);
	b := physics.Sphere{(^Custom_Wide_Shape)(shape_b).radius};
	return physics.sphere_pair_test(a, &b, pose_a, pose_b, speculative_margin, nil);
}

sphere_custom_wide_collision_test :: proc "contextless" (
	bundle: ^physics.Collision_Convex_Wide_Bundle, shapes: ^physics.Shape_Registry,
) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	_ = shapes;
	if bundle == nil || bundle.count <= 0 || bundle.count > util.PRODUCTION_LANE_COUNT
	{
		return {}, .Invalid_Argument;
	}
	result := physics.Collision_Wide_Manifold_Result{kind=.One_Contact};
	for lane in 0 ..< bundle.count
	{
		if bundle.shape_a[lane] == nil || bundle.shape_b[lane] == nil
		{
			return {}, .Invalid_Description;
		}
		a := (^physics.Sphere)(bundle.shape_a[lane]);
		b := (^Custom_Wide_Shape)(bundle.shape_b[lane]);
		offset_b := util.vector3_wide_read_slot(bundle.offset_b, lane);
		source := physics.Convex_Contact_Manifold{count=1, normal={1, 0, 0}};
		source.contacts[0] = {
			offset={a.radius, 0, 0},
			depth=a.radius+b.radius-offset_b.x,
			feature_id=i32(5000+lane),
		};
		status := physics.convex_1_manifold_wide_write_lane(&result.one, lane, source);
		if status != .Ok
		{
			return {}, status;
		}
	}
	return result, .Ok;
}

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

pair_completed :: proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^physics.Manifold_Result,
) -> physics.Physics_Status
{
	callback := (^Callback_Context)(user_context);
	if callback == nil || manifold == nil || callback.result_count >= len(callback.results)
	{
		return .Capacity_Missing;
	}
	callback.pair_ids[callback.result_count] = pair_id;
	callback.results[callback.result_count] = manifold^;
	callback.result_count += 1;
	return .Ok;
}

child_completed :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32, manifold: ^physics.Convex_Contact_Manifold,
) -> physics.Physics_Status
{
	callback := (^Callback_Context)(user_context);
	if callback == nil || manifold == nil || callback.child_count >= len(callback.child_pair_ids)
	{
		return .Capacity_Missing;
	}
	callback.child_pair_ids[callback.child_count] = pair_id;
	callback.child_as[callback.child_count] = child_a;
	callback.child_bs[callback.child_count] = child_b;
	callback.child_contact_counts[callback.child_count] = manifold.count;
	callback.child_count += 1;
	switch callback.child_mode
	{
		case .Observe:
		case .Mutate:
			manifold.count = 1;
			manifold.normal = {0, 1, 0};
			manifold.contacts[0] = {
				offset={1, 0, 0},
				depth=1,
				feature_id=77,
			};
		case .Fail:
			return .Invalid_Description;
	}
	return .Ok;
}

allow_child :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> physics.Collision_Testing_State
{
	_ = pair_id;
	_ = child_a;
	callback := (^Callback_Context)(user_context);
	if callback.allowed_child < 0 || callback.allowed_child == child_b
	{
		return .Allow;
	}
	return .Reject;
}

@(test)
subpair_gather_preserves_compound_mesh_payloads_for_all_tail_counts :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 16, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{0.75};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	children: [8]physics.Compound_Child;
	triangles: [8]physics.Triangle;
	for index in 0 ..< len(children)
	{
		child_sphere := physics.Sphere{0.25 + f32(index)*0.125};
		child_index, child_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &child_sphere);
		testing.expect_value(t, child_status, physics.Physics_Status.Ok);
		children[index] = {local_position={f32(index), 0.5, -0.25},
			local_orientation=util.quaternion_identity(), shape_index=child_index};
		x := f32(index);
		triangles[index] = {{x, 0, -1}, {x + 0.5, 1, 0}, {x + 1, 0, 1}};
	}
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(&compound, &children[0], len(children), &pool), physics.Physics_Status.Ok);
	defer physics.compound_dispose(&compound, &pool);
	compound_index, compound_status := physics.shape_registry_add(&shapes, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], len(triangles), {2, 0.5, 1.5}, &pool), physics.Physics_Status.Ok);
	defer physics.mesh_dispose(&mesh, &pool);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);

	pairs: [2]physics.Collision_Batcher_Pair;
	continuations: [2]physics.Collision_Batcher_Continuation;
	subpairs: [8]physics.Collision_Batcher_Subpair;
	batcher := physics.Collision_Batcher{
		pairs={memory=&pairs[0], length=i32(len(pairs)), id=-1}, pair_count=len(pairs),
		continuations={memory=&continuations[0], length=i32(len(continuations)), id=-1},
		continuation_count=len(continuations), shapes=&shapes, subpair_count=len(subpairs),
		children={format=.Split, length=i32(len(subpairs)),
			subpairs={memory=&subpairs[0], length=i32(len(subpairs)), id=-1}},
	};
	batch := physics.Collision_Task_Batch{count=9};
	batch.pair_indices[0] = -1;
	for index in 0 ..< len(subpairs)
	{
		batch.pair_indices[index + 1] = i32((index*3)%len(subpairs));
	}
	bundle: physics.Collision_Convex_Wide_Bundle;
	for route in 0 ..< 2
	{
		parent_index := compound_index;
		leaf_type := physics.SPHERE_TYPE_ID;
		if route == 1
		{
			parent_index = mesh_index;
			leaf_type = physics.TRIANGLE_TYPE_ID;
		}
		for parent_order in 0 ..< 2
		{
			for pair_index in 0 ..< len(pairs)
			{
				query_pose := physics.Rigid_Pose{position={-1, f32(pair_index), 0.25}, orientation=util.quaternion_identity()};
				parent_pose := physics.Rigid_Pose{position={2, -0.5, f32(pair_index)}, orientation={0, 0, 0.6, 0.8}};
				pairs[pair_index].input = {shape_a=sphere_index, shape_b=parent_index,
					pose_a=query_pose, pose_b=parent_pose, speculative_margin=f32(pair_index) + 0.125};
				continuations[pair_index] = {parent_pair_index=i32(pair_index), flip=.Missing};
				if parent_order == 1
				{
					pairs[pair_index].input.shape_a = parent_index;
					pairs[pair_index].input.shape_b = sphere_index;
					pairs[pair_index].input.pose_a = parent_pose;
					pairs[pair_index].input.pose_b = query_pose;
					continuations[pair_index].flip = .Present;
				}
			}
			for task_order in 0 ..< 2
			{
				task := physics.Collision_Task{shape_type_a=i16(physics.SPHERE_TYPE_ID), shape_type_b=i16(leaf_type)};
				for index in 0 ..< len(subpairs)
				{
					subpairs[index] = {continuation_index=i32(index%2), source_child_a=0, source_child_b=i32(index)};
					if task_order == 1
					{
						subpairs[index].source_child_a = -2147483648;
					}
				}
				if task_order == 1
				{
					task.shape_type_a, task.shape_type_b = task.shape_type_b, task.shape_type_a;
				}
				for remaining in 0 ..< util.PRODUCTION_LANE_COUNT
				{
					count := util.PRODUCTION_LANE_COUNT - remaining;
					testing.expect_value(t, physics.collision_batcher_gather_subpair_bundle(
						&batcher, &task, &batch, 1, count, &bundle), physics.Physics_Status.Ok);
					testing.expect_value(t, bundle.count, count);
					for lane in 0 ..< util.PRODUCTION_LANE_COUNT
					{
						subpair_index := batch.pair_indices[1 + min(lane, count - 1)];
						pair_index := int(subpair_index%2);
						query_pose := pairs[pair_index].input.pose_a;
						parent_pose := pairs[pair_index].input.pose_b;
						if parent_order == 1
						{
							query_pose, parent_pose = parent_pose, query_pose;
						}
						child_pose := physics.rigid_pose_concatenate(
							{position=children[subpair_index].local_position, orientation=children[subpair_index].local_orientation}, parent_pose);
						expected_triangle: physics.Triangle;
						if route == 1
						{
							triangle, pose, status := physics.collision_mesh_child(&mesh, int(subpair_index), parent_pose);
							testing.expect_value(t, status, physics.Physics_Status.Ok);
							expected_triangle, child_pose = triangle, pose;
						}
						pose_a, pose_b := query_pose, child_pose;
						flip := i32(0);
						if task_order == 1
						{
							pose_a, pose_b = pose_b, pose_a;
							flip = -1;
						}
						testing.expect_value(t, bundle.pair_indices[lane], subpair_index);
						testing.expect_value(t, simd.extract(bundle.flip_mask, lane), flip);
						testing.expect_value(t, simd.extract(bundle.speculative_margin, lane), f32(pair_index) + 0.125);
						testing.expect_value(t, util.vector3_wide_read_slot(bundle.offset_b, lane), util.vector3_subtract(pose_b.position, pose_a.position));
						testing.expect_value(t, util.quaternion_wide_read_slot(bundle.orientation_a, lane), pose_a.orientation);
						testing.expect_value(t, util.quaternion_wide_read_slot(bundle.orientation_b, lane), pose_b.orientation);
						query_payload, child_payload := &bundle.a, &bundle.b;
						if task_order == 1
						{
							query_payload, child_payload = child_payload, query_payload;
						}
						testing.expect_value(t, simd.extract(query_payload.sphere.radius, lane), sphere.radius);
						if route == 0
						{
							testing.expect_value(t, simd.extract(child_payload.sphere.radius, lane), 0.25 + f32(subpair_index)*0.125);
						}
						else
						{
							testing.expect_value(t, util.vector3_wide_read_slot(child_payload.triangle.a, lane), expected_triangle.a);
							testing.expect_value(t, util.vector3_wide_read_slot(child_payload.triangle.b, lane), expected_triangle.b);
							testing.expect_value(t, util.vector3_wide_read_slot(child_payload.triangle.c, lane), expected_triangle.c);
						}
					}
				}
				testing.expect_value(t, physics.collision_batcher_gather_subpair_bundle(&batcher, &task, &batch, 1, 0, &bundle), physics.Physics_Status.Invalid_Argument);
				testing.expect_value(t, physics.collision_batcher_gather_subpair_bundle(&batcher, &task, &batch, 1, 9, &bundle), physics.Physics_Status.Invalid_Argument);
				testing.expect_value(t, physics.collision_batcher_gather_subpair_bundle(&batcher, &task, &batch, 0, 1, &bundle), physics.Physics_Status.Invalid_Description);
				subpairs[0].continuation_index = 2;
				testing.expect_value(t, physics.collision_batcher_gather_subpair_bundle(&batcher, &task, &batch, 1, 1, &bundle), physics.Physics_Status.Invalid_Description);
			}
		}
	}
}

@(test)
batcher_preserves_continuation_identity_order_capacity_and_tail_reuse :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	box := physics.Box{1, 1, 1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	box_index, box_status := physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);

	storage_data: [3]physics.Collision_Batcher_Pair;
	storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&storage_data[0],
		length=i32(len(storage_data)),
		id=-1,
	};
	callback := Callback_Context{allowed_child=-1};
	procedures := physics.Collision_Result_Procedures{
		pair_completed=pair_completed,
		child_pair_completed=child_completed,
		allow_child_pair=allow_child,
	};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, storage, &shapes, &tasks, procedures, &callback, &pool,
	), physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {1.5, 0, 0};
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, sphere_index, pose_a, pose_b, 0, 7),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, box_index, sphere_index, pose_a, pose_b, 0, 3),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, box_index, pose_b, pose_a, 0, 11),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, box_index, pose_a, pose_b, 0, 99),
		physics.Physics_Status.Capacity_Missing
	);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 3);
	testing.expect_value(t, callback.pair_ids[0], i32(7));
	testing.expect_value(t, callback.pair_ids[1], i32(3));
	testing.expect_value(t, callback.pair_ids[2], i32(11));
	testing.expect_value(t, batcher.pair_count, 0);

	// the caller-owned batch storage is reusable without any allocator path
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, sphere_index, pose_a, pose_b, 0, 13),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.pair_ids[3], i32(13));

	// remapping submission order cannot change the manifold selected by continuation identity
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, box_index, pose_b, pose_a, 0, 11),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, sphere_index, pose_a, pose_b, 0, 7),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, box_index, sphere_index, pose_a, pose_b, 0, 3),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.pair_ids[4], i32(11));
	testing.expect_value(t, callback.pair_ids[5], i32(7));
	testing.expect_value(t, callback.pair_ids[6], i32(3));
	testing.expect_value(t, callback.results[4], callback.results[2]);
	testing.expect_value(t, callback.results[5], callback.results[0]);
	testing.expect_value(t, callback.results[6], callback.results[1]);
}

@(test)
compound_mesh_continuations_filter_children_and_reduce_to_bounded_nonconvex_results :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	children := [2]physics.Compound_Child{
		{local_position={0, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
		{local_position={0.5, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
	};
	compound: physics.Compound;
	testing.expect_value(
		t,
		physics.compound_create(&compound, &children[0], len(children), &pool),
		physics.Physics_Status.Ok
	);
	compound_index, compound_status := physics.shape_registry_add(&shapes, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	triangles := [2]physics.Triangle{
		{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}},
		{{-2, -0.25, -2}, {0, -0.25, 2}, {2, -0.25, -2}},
	};
	mesh: physics.Mesh;
	testing.expect_value(
		t,
		physics.mesh_create(&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool),
		physics.Physics_Status.Ok
	);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);

	DEEP_CHILD_COUNT :: 300;
	deep_children: [DEEP_CHILD_COUNT]physics.Compound_Child;
	x := f32(1);
	for child_index in 0 ..< DEEP_CHILD_COUNT
	{
		deep_children[child_index] = {
			local_position={x, 0, 0},
			local_orientation=util.quaternion_identity(),
			shape_index=sphere_index,
		};
		x *= 0.99;
	}
	deep_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
		&deep_compound, &deep_children[0], len(deep_children), &shapes, &pool,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_dispose(&deep_compound.tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(
		&deep_compound.tree, DEEP_CHILD_COUNT, &pool,
	), physics.Physics_Status.Ok);
	for child_index in 0 ..< DEEP_CHILD_COUNT
	{
		added_child, add_status := physics.tree_add_without_refinement(
			&deep_compound.tree, deep_compound.child_bounds.memory[child_index],
		);
		testing.expect_value(t, add_status, physics.Physics_Status.Ok);
		testing.expect_value(t, added_child, child_index);
	}
	deep_depth := 0;
	deep_node_index := physics.tree_leaf_node_index(
		deep_compound.tree.leaves.memory[DEEP_CHILD_COUNT - 1],
	);
	for deep_node_index >= 0
	{
		deep_depth += 1;
		deep_node_index = int(deep_compound.tree.metanodes.memory[deep_node_index].parent);
	}
	testing.expect(t, deep_depth > physics.TREE_TRAVERSAL_STACK_CAPACITY);
	deep_compound_index, deep_compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &deep_compound,
	);
	testing.expect_value(t, deep_compound_status, physics.Physics_Status.Ok);

	traversal_pool: util.Buffer_Pool;
	testing.expect_value(
		t, util.buffer_pool_initialize(&traversal_pool, 128),
		util.Memory_Status.Ok,
	);
	defer util.buffer_pool_dispose(&traversal_pool);
	storage_data: [4]physics.Collision_Batcher_Pair;
	storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&storage_data[0],
		length=i32(len(storage_data)),
		id=-1,
	};
	callback := Callback_Context{allowed_child=1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(&batcher, storage, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &traversal_pool), physics.Physics_Status.Ok);
	pose := physics.rigid_pose_identity();
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, sphere_index, compound_index, pose, pose, 0, 1),
		physics.Physics_Status.Ok
	);
	testing.expect_value(
		t,
		physics.collision_batcher_add(&batcher, compound_index, mesh_index, pose, pose, 0, 2),
		physics.Physics_Status.Ok
	);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, deep_compound_index, pose, pose, 0, 3,
	), physics.Physics_Status.Ok);
	construction_pool_bytes := util.buffer_pool_total_allocated_byte_count(&pool);
	traversal_pool_bytes := util.buffer_pool_total_allocated_byte_count(&traversal_pool);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 3);
	testing.expect_value(t, callback.pair_ids[2], i32(3));
	testing.expect(t, callback.child_count > 0);
	testing.expect_value(
		t, util.buffer_pool_total_allocated_byte_count(&pool),
		construction_pool_bytes,
	);
	testing.expect(
		t, util.buffer_pool_total_allocated_byte_count(&traversal_pool) >
		traversal_pool_bytes,
	);
	stack_power, stack_power_status := util.buffer_pool_power_for_count(
		i32, physics.TREE_TRAVERSAL_STACK_CAPACITY * 2,
	);
	testing.expect_value(t, stack_power_status, util.Memory_Status.Ok);
	available_stack_count, available_stack_status :=
		util.buffer_pool_available_slot_count(&traversal_pool, stack_power);
	testing.expect_value(t, available_stack_status, util.Memory_Status.Ok);
	testing.expect(t, available_stack_count > 0);
	for result_index in 0 ..< callback.result_count
	{
		testing.expect_value(t, callback.results[result_index].kind, physics.Manifold_Kind.Nonconvex);
		testing.expect(t, callback.results[result_index].nonconvex.count <= physics.MAXIMUM_MANIFOLD_CONTACT_COUNT);
		for contact_index in 0 ..< int(callback.results[result_index].nonconvex.count)
		{
			child_b := i32((u32(callback.results[result_index].nonconvex.contacts[contact_index].feature_id) >> 16) & 0xff);
			testing.expect_value(t, child_b, i32(1));
		}
	}
}

@(test)
every_builtin_compound_and_mesh_route_executes_through_the_batcher :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);

	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.75, 1};
	box := physics.Box{1, 1, 1};
	triangle := physics.Triangle{{-1, 0, -1}, {0, 0, 1}, {1, 0, -1}};
	cylinder := physics.Cylinder{1, 1};
	indices: [physics.BUILT_IN_SHAPE_TYPE_COUNT]physics.Typed_Index;
	indices[0], _ = physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	indices[1], _ = physics.shape_registry_add(&shapes, physics.CAPSULE_TYPE_ID, &capsule);
	indices[2], _ = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	indices[3], _ = physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	indices[4], _ = physics.shape_registry_add(&shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	points, face_starts, face_indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	indices[5], _ = physics.shape_registry_add(&shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	children := [2]physics.Compound_Child{
		{local_position={0, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=indices[0]},
		{local_position={0.25, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=indices[2]},
	};
	compound: physics.Compound;
	testing.expect_value(
		t,
		physics.compound_create(&compound, &children[0], len(children), &pool),
		physics.Physics_Status.Ok
	);
	indices[6], _ = physics.shape_registry_add(&shapes, physics.COMPOUND_TYPE_ID, &compound);
	big_compound: physics.Big_Compound;
	testing.expect_value(
		t,
		physics.big_compound_create(&big_compound, &children[0], len(children), &shapes, &pool),
		physics.Physics_Status.Ok
	);
	indices[7], _ = physics.shape_registry_add(&shapes, physics.BIG_COMPOUND_TYPE_ID, &big_compound);
	mesh_triangles := [1]physics.Triangle{{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}}};
	mesh: physics.Mesh;
	testing.expect_value(
		t,
		physics.mesh_create(&mesh, &mesh_triangles[0], 1, {1, 1, 1}, &pool),
		physics.Physics_Status.Ok
	);
	indices[8], _ = physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	for index in indices
	{
		testing.expect_value(t, physics.typed_index_state(index), physics.Reference_State.Present);
	}

	pair_storage_data: [24]physics.Collision_Batcher_Pair;
	pair_storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&pair_storage_data[0], length=i32(len(pair_storage_data)), id=-1,
	};
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(&batcher, pair_storage, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &pool), physics.Physics_Status.Ok);
	pair_id := i32(0);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {0.2, 0.1, 0.15};
	for type_a in 0 ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
	{
		for type_b in type_a ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
		{
			if type_b < physics.COMPOUND_TYPE_ID
			{
				continue;
			}
			testing.expect_value(t, physics.collision_batcher_add(
				&batcher, indices[type_a], indices[type_b], pose_a, pose_b, 0.1, pair_id,
			), physics.Physics_Status.Ok);
			pair_id += 1;
		}
	}
	testing.expect_value(t, pair_id, i32(24));
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 24);
	for result_index in 0 ..< callback.result_count
	{
		testing.expect_value(t, callback.pair_ids[result_index], i32(result_index));
		testing.expect_value(t, callback.results[result_index].kind, physics.Manifold_Kind.Nonconvex);
	}
}

@(test)
batcher_hot_add_flush_and_callback_delivery_make_no_general_allocator_calls :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	storage_data: [1]physics.Collision_Batcher_Pair;
	storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&storage_data[0], length=i32(len(storage_data)), id=-1,
	};
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(&batcher, storage, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &pool), physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {1.5, 0, 0};

	tracker := (^mem.Tracking_Allocator)(context.allocator.data);
	allocation_count_before := tracker.total_allocation_count;
	hot_status := physics.Physics_Status.Ok;
	for pair_id in 0 ..< 1024
	{
		callback.result_count = 0;
		hot_status = physics.collision_batcher_add(
			&batcher, sphere_index, sphere_index, pose_a, pose_b, 0, i32(pair_id),
		);
		if hot_status != .Ok
		{
			break;
		}
		hot_status = physics.collision_batcher_flush(&batcher);
		if hot_status != .Ok
		{
			break;
		}
	}
	allocation_count_after := tracker.total_allocation_count;

	testing.expect_value(t, hot_status, physics.Physics_Status.Ok);
	testing.expect_value(t, allocation_count_after, allocation_count_before);
	testing.expect_value(t, batcher.pair_count, 0);
}

@(test)
typed_wide_batch_executes_a_full_batch_and_masked_tail_in_submission_order :: proc(t: ^testing.T)
{
	offset_b: util.Vector3_Wide;
	one: physics.Convex_1_Contact_Manifold_Wide;
	two: physics.Convex_2_Contact_Manifold_Wide;
	four: physics.Convex_4_Contact_Manifold_Wide;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		lane_offset_b := util.Vector3{10+f32(lane), 20+f32(lane), 30+f32(lane)};
		util.vector3_wide_write_slot(&offset_b, lane, lane_offset_b);
		one_source := physics.Convex_Contact_Manifold{
			offset_b=lane_offset_b,
			count=i32(lane % 2),
			normal={1, f32(lane), -1},
		};
		two_source := physics.Convex_Contact_Manifold{
			offset_b=lane_offset_b,
			count=i32(lane % 3),
			normal={2, f32(lane), -2},
		};
		four_source := physics.Convex_Contact_Manifold{
			offset_b=lane_offset_b,
			count=i32(lane % 5),
			normal={4, f32(lane), -4},
		};
		for contact_index in 0 ..< int(one_source.count)
		{
			one_source.contacts[contact_index] = {
				offset={f32(lane), f32(contact_index), 1},
				depth=f32(100+lane*10+contact_index),
				feature_id=i32(1000+lane*10+contact_index),
			};
		}
		for contact_index in 0 ..< int(two_source.count)
		{
			two_source.contacts[contact_index] = {
				offset={f32(lane), f32(contact_index), 2},
				depth=f32(200+lane*10+contact_index),
				feature_id=i32(2000+lane*10+contact_index),
			};
		}
		for contact_index in 0 ..< int(four_source.count)
		{
			four_source.contacts[contact_index] = {
				offset={f32(lane), f32(contact_index), 4},
				depth=f32(400+lane*10+contact_index),
				feature_id=i32(4000+lane*10+contact_index),
			};
		}
		testing.expect_value(
			t,
			physics.convex_1_manifold_wide_write_lane(&one, lane, one_source),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.convex_2_manifold_wide_write_lane(&two, lane, two_source),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.convex_4_manifold_wide_write_lane(&four, lane, four_source),
			physics.Physics_Status.Ok
		);
	}
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		one_returned, one_returned_status := physics.convex_1_manifold_wide_read_lane(&one, offset_b, lane);
		two_returned, two_returned_status := physics.convex_2_manifold_wide_read_lane(&two, offset_b, lane);
		four_returned, four_returned_status := physics.convex_4_manifold_wide_read_lane(&four, offset_b, lane);
		one_into := physics.Convex_Contact_Manifold{count=4, normal={-1, -1, -1}};
		two_into := physics.Convex_Contact_Manifold{count=4, normal={-2, -2, -2}};
		four_into := physics.Convex_Contact_Manifold{count=4, normal={-4, -4, -4}};
		four_trusted: physics.Convex_Contact_Manifold = ---
			testing.expect_value(t, one_returned_status, physics.Physics_Status.Ok);
		testing.expect_value(t, two_returned_status, physics.Physics_Status.Ok);
		testing.expect_value(t, four_returned_status, physics.Physics_Status.Ok);
		testing.expect_value(
			t,
			physics.convex_1_manifold_wide_read_lane_into(&one, offset_b, lane, &one_into),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.convex_2_manifold_wide_read_lane_into(&two, offset_b, lane, &two_into),
			physics.Physics_Status.Ok
		);
		testing.expect_value(
			t,
			physics.convex_4_manifold_wide_read_lane_into(&four, offset_b, lane, &four_into),
			physics.Physics_Status.Ok
		);
		physics.convex_4_manifold_wide_read_lane_core(&four, offset_b, lane, &four_trusted);
		testing.expect_value(t, one_into, one_returned);
		testing.expect_value(t, two_into, two_returned);
		testing.expect_value(t, four_into, four_returned);
		testing.expect_value(t, four_trusted.count, four_returned.count);
		if four_returned.count > 0
		{
			testing.expect_value(t, four_trusted.offset_b, four_returned.offset_b);
			testing.expect_value(t, four_trusted.normal, four_returned.normal);
			for contact_index in 0 ..< int(four_returned.count)
			{
				testing.expect_value(
					t, four_trusted.contacts[contact_index],
					four_returned.contacts[contact_index],
				);
			}
		}
	}
	box_a_wide: physics.Box_Wide;
	box_b_wide: physics.Box_Wide;
	box_offset_wide: util.Vector3_Wide;
	box_orientation_a_wide: util.Quaternion_Wide;
	box_orientation_b_wide: util.Quaternion_Wide;
	box_margin_wide: util.F32x8;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		physics.box_wide_write_slot_trusted(&box_a_wide, lane, {1, 1, 1});
		physics.box_wide_write_slot_trusted(&box_b_wide, lane, {0.75, 0.8, 0.9});
		util.vector3_wide_write_slot(&box_offset_wide, lane, {1.25+f32(lane)*0.05, 0.1, -0.2});
		util.quaternion_wide_write_slot(&box_orientation_a_wide, lane, util.quaternion_identity());
		util.quaternion_wide_write_slot(&box_orientation_b_wide, lane, util.quaternion_identity());
		box_margin_wide = simd.replace(box_margin_wide, lane, f32(0.1));
	}
	for group_count in 1 ..= util.PRODUCTION_LANE_COUNT
	{
		checked: physics.Convex_4_Contact_Manifold_Wide;
		trusted: physics.Convex_4_Contact_Manifold_Wide;
		testing.expect_value(t, physics.box_pair_test_wide_into(
			box_a_wide, box_b_wide, box_margin_wide, box_offset_wide,
			box_orientation_a_wide, box_orientation_b_wide, group_count, &checked,
		), physics.Physics_Status.Ok);
		active, active_status := util.bundle_count_mask(group_count);
		testing.expect_value(t, active_status, util.Memory_Status.Ok);
		physics.box_pair_test_wide_core(
			box_a_wide, box_b_wide, box_margin_wide, box_offset_wide,
			box_orientation_a_wide, box_orientation_b_wide,
			active, group_count, &trusted,
		);
		testing.expect_value(t, trusted, checked);
	}
	direct_box_a := physics.Box{1, 1, 1};
	direct_box_b := physics.Box{0.75, 0.8, 0.9};
	direct_records: [util.PRODUCTION_LANE_COUNT]physics.Narrow_Phase_Convex_Direct_Record;
	identity := physics.rigid_pose_identity();
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		direct_records[lane] = {
			pair_id=i32(lane),
			speculative_margin=0.1,
			shape_data_a=&direct_box_a,
			shape_data_b=&direct_box_b,
			pose_a=identity,
			pose_b=identity,
		};
		direct_records[lane].pose_b.position = {
			1.25+f32(lane)*0.05, 0.1, -0.2,
		};
	}
	for group_count in 1 ..= util.PRODUCTION_LANE_COUNT
	{
		direct_offset: util.Vector3_Wide = ---
			direct_wide: physics.Convex_4_Contact_Manifold_Wide = ---
			physics.narrow_phase_box_direct_test_group(
			&direct_records[0], 0, group_count, &direct_offset, &direct_wide,
		);
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			source_lane := min(lane, group_count - 1);
			expected, expected_status := physics.box_pair_test_source(
				direct_box_a, direct_box_b,
				direct_records[source_lane].pose_a,
				direct_records[source_lane].pose_b,
				direct_records[source_lane].speculative_margin,
			);
			actual: physics.Convex_Contact_Manifold;
			actual_status := physics.convex_4_manifold_wide_read_lane_into(
				&direct_wide, direct_offset, lane, &actual,
			);
			testing.expect_value(t, actual_status, expected_status);
			if lane < group_count
			{
				testing.expect_value(t, actual, expected);
			}
			else
			{
				testing.expect_value(t, actual.count, i32(0));
			}
		}
	}

	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere_a := physics.Sphere{0.75};
	sphere_b := physics.Sphere{1.25};
	index_a, status_a := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere_a);
	index_b, status_b := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere_b);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	pair_storage_data: [33]physics.Collision_Batcher_Pair;
	pair_storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&pair_storage_data[0], length=i32(len(pair_storage_data)), id=-1,
	};
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(&batcher, pair_storage, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &pool), physics.Physics_Status.Ok);
	identity = physics.rigid_pose_identity();
	for pair_index in 0 ..< len(pair_storage_data)
	{
		pose_b := identity;
		pose_b.position = {1.25 + f32(pair_index) * 0.01, 0.05, -0.1};
		shape_a := index_a;
		shape_b := index_b;
		pose_a := identity;
		if pair_index & 1 != 0
		{
			shape_a, shape_b = shape_b, shape_a;
			pose_a, pose_b = pose_b, pose_a;
		}
		testing.expect_value(t, physics.collision_batcher_add(
			&batcher, shape_a, shape_b, pose_a, pose_b, 0.05, i32(1000+pair_index),
		), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, len(pair_storage_data));
	for pair_index in 0 ..< len(pair_storage_data)
	{
		testing.expect_value(t, callback.pair_ids[pair_index], i32(1000+pair_index));
		testing.expect_value(t, callback.results[pair_index].kind, physics.Manifold_Kind.Convex);
		testing.expect_value(t, callback.results[pair_index].convex.count, i32(1));
		testing.expect_value(t, callback.results[pair_index].convex.contacts[0].feature_id, i32(0));
	}
}

@(test)
all_repaired_builtin_wide_routes_execute_full_bundles_and_masked_tails :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 16, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);

	sphere := physics.Sphere{0.9};
	capsule := physics.Capsule{0.65, 0.9};
	box := physics.Box{0.9, 0.8, 1.0};
	triangle := physics.Triangle{{-1, 0, -1}, {0, 0, 1}, {1, 0, -1}};
	cylinder := physics.Cylinder{0.9, 0.8};
	indices: [physics.BUILT_IN_SHAPE_TYPE_COUNT]physics.Typed_Index;
	indices[0], _ = physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	indices[1], _ = physics.shape_registry_add(&shapes, physics.CAPSULE_TYPE_ID, &capsule);
	indices[2], _ = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	indices[3], _ = physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	indices[4], _ = physics.shape_registry_add(&shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	points, face_starts, face_indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	indices[5], _ = physics.shape_registry_add(&shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	for index in indices[:6]
	{
		testing.expect_value(t, physics.typed_index_state(index), physics.Reference_State.Present);
	}

	repaired_routes := [13][2]int{
		{1, 2}, {1, 3}, {1, 4}, {2, 2}, {2, 3}, {2, 4}, {2, 5},
		{3, 3}, {3, 4}, {3, 5}, {4, 4}, {4, 5}, {5, 5},
	};
	pair_storage_data: [11]physics.Collision_Batcher_Pair;
	pair_storage := util.Buffer(physics.Collision_Batcher_Pair){
		memory=&pair_storage_data[0], length=i32(len(pair_storage_data)), id=-1,
	};
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(&batcher, pair_storage, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &pool), physics.Physics_Status.Ok);
	identity := physics.rigid_pose_identity();

	for route_index in 0 ..< len(repaired_routes)
	{
		baseline: [len(pair_storage_data)]physics.Manifold_Result;
		for repetition in 0 ..< 2
		{
			expected_offsets: [len(pair_storage_data)]util.Vector3;
			callback.result_count = 0;
			for pair_index in 0 ..< len(pair_storage_data)
			{
				type_a := repaired_routes[route_index][0];
				type_b := repaired_routes[route_index][1];
				shape_a := indices[type_a];
				shape_b := indices[type_b];
				pose_a := identity;
				pose_b := identity;
				pose_b.position = {0.18 + f32(pair_index) * 0.015, 0.08, 0.12};
				pose_b.orientation = util.quaternion_from_axis_angle({0, 1, 0}, f32(pair_index) * 0.01);
				if type_a == physics.TRIANGLE_TYPE_ID && type_b == physics.TRIANGLE_TYPE_ID
				{
					pose_b.position.y = -0.04;
					pose_b.orientation = util.quaternion_from_axis_angle({1, 0, 0}, 3.14159265);
				}
				else if type_a == physics.TRIANGLE_TYPE_ID && type_b == physics.CONVEX_HULL_TYPE_ID
				{
					pose_b.position.y = -0.08;
				}
				if type_a != type_b && pair_index & 1 != 0
				{
					shape_a, shape_b = shape_b, shape_a;
					pose_a, pose_b = pose_b, pose_a;
				}
				expected_offsets[pair_index] = util.vector3_subtract(pose_b.position, pose_a.position);
				testing.expect_value(t, physics.collision_batcher_add(
					&batcher, shape_a, shape_b, pose_a, pose_b, 0.1, i32(route_index*100+pair_index),
				), physics.Physics_Status.Ok);
			}
			testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
			testing.expect_value(t, callback.result_count, len(pair_storage_data));
			for pair_index in 0 ..< len(pair_storage_data)
			{
				actual := callback.results[pair_index];
				testing.expect_value(t, callback.pair_ids[pair_index], i32(route_index*100+pair_index));
				testing.expect_value(t, actual.kind, physics.Manifold_Kind.Convex);
				testing.expectf(t, actual.convex.count >= 0 && actual.convex.count <= physics.MAXIMUM_MANIFOLD_CONTACT_COUNT,
					"route %d-%d pair %d: expected a bounded manifold result, got %d contacts",
					repaired_routes[route_index][0], repaired_routes[route_index][1], pair_index, actual.convex.count,
				);
				if actual.convex.count > 0
				{
					testing.expectf(t, util.vector3_distance(
						actual.convex.offset_b,
						expected_offsets[pair_index]
					) < 2e-5,
						"route %d-%d pair %d: offset_b expected %v, got %v",
						repaired_routes[route_index][0], repaired_routes[route_index][1], pair_index,
						expected_offsets[pair_index], actual.convex.offset_b,
					);
					normal_length_squared := util.vector3_length_squared(actual.convex.normal);
					testing.expectf(t, normal_length_squared > 0.99 && normal_length_squared < 1.01,
						"route %d-%d pair %d: expected a unit normal, got length squared %.7f",
						repaired_routes[route_index][0], repaired_routes[route_index][1], pair_index, normal_length_squared,
					);
				}
				for contact_index in 0 ..< int(actual.convex.count)
				{
					contact := actual.convex.contacts[contact_index];
					testing.expectf(t, contact.depth >= -0.1001 && contact.depth == contact.depth,
						"route %d-%d pair %d contact %d: invalid depth %.7f",
						repaired_routes[route_index][0], repaired_routes[route_index][1], pair_index, contact_index, contact.depth,
					);
					testing.expectf(t,
						contact.offset.x == contact.offset.x &&
						contact.offset.y == contact.offset.y &&
						contact.offset.z == contact.offset.z,
						"route %d-%d pair %d contact %d: non-finite offset %v",
						repaired_routes[route_index][0], repaired_routes[route_index][1], pair_index, contact_index, contact.offset,
					);
				}
				if repetition == 0
				{
					baseline[pair_index] = actual;
				}
				else
				{
					testing.expect_value(t, actual, baseline[pair_index]);
				}
			}
		}
	}
}

@(test)
caller_registered_wide_task_uses_the_same_full_and_tail_batch_boundary :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_type, shape_registration_status := physics.shape_registry_register_custom(&shapes, {
			size=size_of(Custom_Wide_Shape),
			alignment=align_of(Custom_Wide_Shape),
			batch_type=.Convex,
			bounds=custom_wide_shape_bounds,
			inertia=custom_wide_shape_inertia,
			ray=custom_wide_shape_ray,
			support=custom_wide_shape_support,
			sweep_support=custom_wide_shape_support,
			dispose=physics.shape_no_dispose,
		});
	testing.expect_value(t, shape_registration_status, physics.Physics_Status.Ok);
	a := Custom_Wide_Shape{0.75};
	b := Custom_Wide_Shape{1.25};
	index_a, add_a_status := physics.shape_registry_add(&shapes, int(custom_type), &a);
	index_b, add_b_status := physics.shape_registry_add(&shapes, int(custom_type), &b);
	testing.expect_value(t, add_a_status, physics.Physics_Status.Ok);
	testing.expect_value(t, add_b_status, physics.Physics_Status.Ok);

	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_convex(
		&tasks, int(custom_type), int(custom_type), 8, .Flipless,
		custom_scalar_collision_test, custom_wide_collision_test,
	), physics.Physics_Status.Ok);
	pair_storage_data: [11]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&pair_storage_data[0], length=i32(len(pair_storage_data)), id=-1},
		&shapes, &tasks, {
			pair_completed=pair_completed,
			child_pair_completed=child_completed,
			allow_child_pair=allow_child
		},
		&callback, &pool,
	), physics.Physics_Status.Ok);
	identity := physics.rigid_pose_identity();
	for pair_index in 0 ..< len(pair_storage_data)
	{
		pose_b := identity;
		pose_b.position = {1+f32(pair_index)*0.01, 0, 0};
		testing.expect_value(t, physics.collision_batcher_add(
			&batcher, index_a, index_b, identity, pose_b, 0, i32(2000+pair_index),
		), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, len(pair_storage_data));
	for pair_index in 0 ..< len(pair_storage_data)
	{
		expected_feature := i32(800+pair_index);
		if pair_index >= util.PRODUCTION_LANE_COUNT
		{
			expected_feature = i32(300+pair_index-util.PRODUCTION_LANE_COUNT);
		}
		testing.expect_value(t, callback.pair_ids[pair_index], i32(2000+pair_index));
		testing.expect_value(t, callback.results[pair_index].kind, physics.Manifold_Kind.Convex);
		testing.expect_value(t, callback.results[pair_index].convex.count, i32(1));
		testing.expect_value(t, callback.results[pair_index].convex.contacts[0].feature_id, expected_feature);
	}
	for tail_count in 1 ..= util.PRODUCTION_LANE_COUNT
	{
		callback.result_count = 0;
		for pair_index in 0 ..< tail_count
		{
			pose_b := identity;
			pose_b.position = {1+f32(pair_index)*0.01, 0, 0};
			testing.expect_value(t, physics.collision_batcher_add(
				&batcher, index_a, index_b, identity, pose_b, 0,
				i32(3000+tail_count*100+pair_index),
			), physics.Physics_Status.Ok);
		}
		testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
		testing.expect_value(t, callback.result_count, tail_count);
		for pair_index in 0 ..< tail_count
		{
			testing.expect_value(t, callback.pair_ids[pair_index], i32(3000+tail_count*100+pair_index));
			testing.expect_value(t, callback.results[pair_index].kind, physics.Manifold_Kind.Convex);
			testing.expect_value(t, callback.results[pair_index].convex.count, i32(1));
			testing.expect_value(
				t, callback.results[pair_index].convex.contacts[0].feature_id,
				i32(tail_count*100+pair_index),
			);
		}
	}

	task, _, lookup_status := physics.collision_task_registry_lookup(
		&tasks, int(custom_type), int(custom_type),
	);
	testing.expect_value(t, lookup_status, physics.Physics_Status.Ok);
	successful_wide_test := task.convex_wide_test;
	task.convex_wide_test = custom_wide_collision_failure_test;
	callback.result_count = 0;
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, index_a, index_b, identity, identity, 0, 4000,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Invalid_Description);
	testing.expect_value(t, pair_storage_data[0].result_state, physics.Collision_Batcher_Result_State.Pending);
	testing.expect_value(t, callback.result_count, 0);
	testing.expect_value(t, physics.collision_batcher_reset_fault(&batcher), physics.Physics_Status.Ok);
	task.convex_wide_test = successful_wide_test;
}

@(test)
missing_collision_task_completes_one_empty_result_before_batch_admission :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_type, registration_status := physics.shape_registry_register_custom(&shapes, {
			size=size_of(Custom_Wide_Shape),
			alignment=align_of(Custom_Wide_Shape),
			batch_type=.Convex,
			bounds=custom_wide_shape_bounds,
			inertia=custom_wide_shape_inertia,
			ray=custom_wide_shape_ray,
			support=custom_wide_shape_support,
			sweep_support=custom_wide_shape_support,
			dispose=physics.shape_no_dispose,
		});
	testing.expect_value(t, registration_status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	custom := Custom_Wide_Shape{1};
	custom_index, custom_status := physics.shape_registry_add(&shapes, int(custom_type), &custom);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	storage_data: [1]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&storage_data[0], length=1, id=-1}, &shapes, &tasks, {
			pair_completed=pair_completed,
		}, &callback, &pool,
	), physics.Physics_Status.Ok);
	pose := physics.rigid_pose_identity();
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, sphere_index, pose, pose, 0, 21,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, batcher.pair_count, 1);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, custom_index, sphere_index, pose, pose, 0, 22,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, batcher.pair_count, 1);
	testing.expect_value(t, callback.result_count, 1);
	testing.expect_value(t, callback.pair_ids[0], i32(22));
	testing.expect_value(t, callback.results[0].kind, physics.Manifold_Kind.Convex);
	testing.expect_value(t, callback.results[0].convex.count, i32(0));
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 2);
	testing.expect_value(t, callback.pair_ids[0], i32(22));
	testing.expect_value(t, callback.pair_ids[1], i32(21));
}

@(test)
compound_routes_reach_registered_custom_convex_child_tasks :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_type, registration_status := physics.shape_registry_register_custom(&shapes, {
			size=size_of(Custom_Wide_Shape),
			alignment=align_of(Custom_Wide_Shape),
			batch_type=.Convex,
			bounds=custom_wide_shape_bounds,
			inertia=custom_wide_shape_inertia,
			ray=custom_wide_shape_ray,
			support=custom_wide_shape_support,
			sweep_support=custom_wide_shape_support,
			dispose=physics.shape_no_dispose,
		});
	testing.expect_value(t, registration_status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	custom := Custom_Wide_Shape{1};
	custom_index, custom_status := physics.shape_registry_add(&shapes, int(custom_type), &custom);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	child := physics.Compound_Child{
		local_orientation=util.quaternion_identity(), shape_index=custom_index,
	};
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(&compound, &child, 1, &pool), physics.Physics_Status.Ok);
	compound_index, compound_status := physics.shape_registry_add(
		&shapes, physics.COMPOUND_TYPE_ID, &compound,
	);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	big_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
		&big_compound, &child, 1, &shapes, &pool,
	), physics.Physics_Status.Ok);
	big_compound_index, big_compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &big_compound,
	);
	testing.expect_value(t, big_compound_status, physics.Physics_Status.Ok);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_convex(
		&tasks, physics.SPHERE_TYPE_ID, int(custom_type), 8, .Standard,
		sphere_custom_scalar_collision_test, sphere_custom_wide_collision_test,
	), physics.Physics_Status.Ok);
	storage_data: [2]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&storage_data[0], length=2, id=-1}, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed,
		}, &callback, &pool,
	), physics.Physics_Status.Ok);
	pose := physics.rigid_pose_identity();
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, compound_index, pose, pose, 0, 31,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, big_compound_index, pose, pose, 0, 32,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 2);
	testing.expect_value(t, callback.child_count, 2);
	for result_index in 0 ..< callback.result_count
	{
		testing.expect_value(t, callback.results[result_index].kind, physics.Manifold_Kind.Nonconvex);
		testing.expect(t, callback.results[result_index].nonconvex.count > 0);
	}
}

@(test)
missing_compound_child_task_completes_the_shared_empty_contribution :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_type, registration_status := physics.shape_registry_register_custom(&shapes, {
			size=size_of(Custom_Wide_Shape),
			alignment=align_of(Custom_Wide_Shape),
			batch_type=.Convex,
			bounds=custom_wide_shape_bounds,
			inertia=custom_wide_shape_inertia,
			ray=custom_wide_shape_ray,
			support=custom_wide_shape_support,
			sweep_support=custom_wide_shape_support,
			dispose=physics.shape_no_dispose,
		});
	testing.expect_value(t, registration_status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	custom := Custom_Wide_Shape{1};
	custom_index, custom_status := physics.shape_registry_add(&shapes, int(custom_type), &custom);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	child := physics.Compound_Child{
		local_orientation=util.quaternion_identity(),
		shape_index=custom_index,
	};
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(&compound, &child, 1, &pool), physics.Physics_Status.Ok);
	defer physics.compound_dispose(&compound, &pool);
	compound_index, compound_status := physics.shape_registry_add(
		&shapes, physics.COMPOUND_TYPE_ID, &compound,
	);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	big_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
		&big_compound, &child, 1, &shapes, &pool,
	), physics.Physics_Status.Ok);
	defer physics.big_compound_dispose(&big_compound, &pool);
	big_compound_index, big_compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &big_compound,
	);
	testing.expect_value(t, big_compound_status, physics.Physics_Status.Ok);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	pose := physics.rigid_pose_identity();

	storage_data: [2]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&storage_data[0], length=len(storage_data), id=-1},
		&shapes, &tasks, {
			pair_completed=pair_completed,
			child_pair_completed=child_completed,
		}, &callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&batcher);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, compound_index, pose, pose, 0, 41,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, big_compound_index, pose, pose, 0, 42,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, batcher.subpair_count, 0);
	testing.expect_value(t, callback.child_count, 2);
	testing.expect_value(t, callback.result_count, 2);
	for child_index in 0 ..< callback.child_count
	{
		testing.expect_value(t, callback.child_contact_counts[child_index], i32(0));
	}
	for result_index in 0 ..< callback.result_count
	{
		testing.expect_value(t, callback.results[result_index].kind, physics.Manifold_Kind.Nonconvex);
		testing.expect_value(t, callback.results[result_index].nonconvex.count, i32(0));
	}

	mutating_storage: [1]physics.Collision_Batcher_Pair;
	mutating_callback := Callback_Context{allowed_child=-1, child_mode=.Mutate};
	mutating_batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&mutating_batcher, {memory=&mutating_storage[0], length=1, id=-1},
		&shapes, &tasks, {
			pair_completed=pair_completed,
			child_pair_completed=child_completed,
		}, &mutating_callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&mutating_batcher);
	testing.expect_value(t, physics.collision_batcher_add(
		&mutating_batcher, sphere_index, compound_index, pose, pose, 0, 43,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&mutating_batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, mutating_callback.child_count, 1);
	testing.expect_value(t, mutating_callback.child_contact_counts[0], i32(0));
	testing.expect_value(t, mutating_callback.result_count, 1);
	testing.expect_value(t, mutating_callback.results[0].kind, physics.Manifold_Kind.Nonconvex);
	testing.expect_value(t, mutating_callback.results[0].nonconvex.count, i32(1));

	failing_storage: [1]physics.Collision_Batcher_Pair;
	failing_callback := Callback_Context{allowed_child=-1, child_mode=.Fail};
	failing_batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&failing_batcher, {memory=&failing_storage[0], length=1, id=-1},
		&shapes, &tasks, {
			pair_completed=pair_completed,
			child_pair_completed=child_completed,
		}, &failing_callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&failing_batcher);
	testing.expect_value(t, physics.collision_batcher_add(
		&failing_batcher, sphere_index, compound_index, pose, pose, 0, 44,
	), physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.collision_batcher_flush(&failing_batcher),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(t, failing_batcher.subpair_count, 0);
	testing.expect_value(t, failing_callback.child_count, 1);
	testing.expect_value(t, failing_callback.child_contact_counts[0], i32(0));
	testing.expect_value(t, failing_callback.result_count, 0);
}

@(test)
registered_custom_convex_compound_routes_dispatch_by_task_kind :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 16, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_registration := physics.Shape_Type_Registration{
		size=size_of(Custom_Wide_Shape),
		alignment=align_of(Custom_Wide_Shape),
		batch_type=.Convex,
		bounds=custom_wide_shape_bounds,
		inertia=custom_wide_shape_inertia,
		ray=custom_wide_shape_ray,
		support=custom_wide_shape_support,
		sweep_support=custom_wide_shape_support,
		dispose=physics.shape_no_dispose,
	};
	custom_type, registration_status := physics.shape_registry_register_custom(
		&shapes, custom_registration,
	);
	testing.expect_value(t, registration_status, physics.Physics_Status.Ok);
	nonconvex_registration := custom_registration;
	nonconvex_registration.batch_type = .Compound;
	nonconvex_type, nonconvex_registration_status := physics.shape_registry_register_custom(
		&shapes, nonconvex_registration,
	);
	testing.expect_value(t, nonconvex_registration_status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	custom_parent := Custom_Wide_Shape{1};
	custom_parent_index, custom_parent_status := physics.shape_registry_add(
		&shapes, int(custom_type), &custom_parent,
	);
	nonconvex_parent_index, nonconvex_parent_status := physics.shape_registry_add(
		&shapes, int(nonconvex_type), &custom_parent,
	);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_parent_status, physics.Physics_Status.Ok);
	testing.expect_value(t, nonconvex_parent_status, physics.Physics_Status.Ok);
	children := [2]physics.Compound_Child{
		{local_position={0, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
		{local_position={1, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
	};
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(
		&compound, &children[0], len(children), &pool,
	), physics.Physics_Status.Ok);
	defer physics.compound_dispose(&compound, &pool);
	compound_index, compound_status := physics.shape_registry_add(
		&shapes, physics.COMPOUND_TYPE_ID, &compound,
	);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	big_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
		&big_compound, &children[0], len(children), &shapes, &pool,
	), physics.Physics_Status.Ok);
	defer physics.big_compound_dispose(&big_compound, &pool);
	big_compound_index, big_compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &big_compound,
	);
	testing.expect_value(t, big_compound_status, physics.Physics_Status.Ok);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_convex(
		&tasks, physics.SPHERE_TYPE_ID, int(custom_type), 8, .Standard,
		sphere_custom_scalar_collision_test, sphere_custom_wide_collision_test,
	), physics.Physics_Status.Ok);
	generator_capabilities := physics.Collision_Task_Capabilities{
		.Subtask_Generator,
		.Child_Order,
	};
	testing.expect_value(t, physics.collision_task_registry_register_compound(
		&tasks, int(custom_type), physics.COMPOUND_TYPE_ID, 16,
		.Convex_Compound, generator_capabilities,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_compound(
		&tasks, int(custom_type), physics.BIG_COMPOUND_TYPE_ID, 16,
		.Convex_Compound, generator_capabilities,
	), physics.Physics_Status.Ok);
	storage_data: [4]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&storage_data[0], length=len(storage_data), id=-1},
		&shapes, &tasks, {
			pair_completed=pair_completed,
			child_pair_completed=child_completed,
		}, &callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&batcher);
	pose := physics.rigid_pose_identity();
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, custom_parent_index, compound_index, pose, pose, 0, 51,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, compound_index, custom_parent_index, pose, pose, 0, 52,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, custom_parent_index, big_compound_index, pose, pose, 0, 53,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, big_compound_index, custom_parent_index, pose, pose, 0, 54,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 4);
	testing.expect_value(t, callback.child_count, 8);
	child_counts_by_pair: [4]int;
	for child_index in 0 ..< callback.child_count
	{
		pair_offset := int(callback.child_pair_ids[child_index] - 51);
		testing.expect(t, pair_offset >= 0 && pair_offset < len(child_counts_by_pair));
		if pair_offset < 0 || pair_offset >= len(child_counts_by_pair)
		{
			continue;
		}
		child_counts_by_pair[pair_offset] += 1;
		if pair_offset == 0 || pair_offset == 2
		{
			testing.expect_value(t, callback.child_as[child_index], i32(0));
			testing.expect(t, callback.child_bs[child_index] >= 0 && callback.child_bs[child_index] < 2);
		}
		else
		{
			testing.expect(t, callback.child_as[child_index] >= 0 && callback.child_as[child_index] < 2);
			testing.expect_value(t, callback.child_bs[child_index], i32(0));
		}
	}
	for count in child_counts_by_pair
	{
		testing.expect_value(t, count, 2);
	}
	for result_index in 0 ..< callback.result_count
	{
		testing.expect_value(t, callback.results[result_index].kind, physics.Manifold_Kind.Nonconvex);
		testing.expect(t, callback.results[result_index].nonconvex.count > 0);
	}

	wrong_tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&wrong_tasks), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_compound(
		&wrong_tasks, int(custom_type), physics.COMPOUND_TYPE_ID, 16,
		.Compound_Pair, generator_capabilities,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_task_registry_register_compound(
		&wrong_tasks, int(nonconvex_type), physics.COMPOUND_TYPE_ID, 16,
		.Convex_Compound, generator_capabilities,
	), physics.Physics_Status.Ok);
	wrong_kind_storage: [1]physics.Collision_Batcher_Pair;
	wrong_kind_callback := Callback_Context{allowed_child=-1};
	wrong_kind_batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&wrong_kind_batcher, {memory=&wrong_kind_storage[0], length=1, id=-1},
		&shapes, &wrong_tasks, {pair_completed=pair_completed},
		&wrong_kind_callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&wrong_kind_batcher);
	testing.expect_value(t, physics.collision_batcher_add(
		&wrong_kind_batcher, custom_parent_index, compound_index, pose, pose, 0, 55,
	), physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.collision_batcher_flush(&wrong_kind_batcher),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(t, wrong_kind_batcher.continuation_count, 0);

	wrong_metadata_storage: [1]physics.Collision_Batcher_Pair;
	wrong_metadata_callback := Callback_Context{allowed_child=-1};
	wrong_metadata_batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&wrong_metadata_batcher, {memory=&wrong_metadata_storage[0], length=1, id=-1},
		&shapes, &wrong_tasks, {pair_completed=pair_completed},
		&wrong_metadata_callback, &pool,
	), physics.Physics_Status.Ok);
	defer physics.collision_batcher_dispose(&wrong_metadata_batcher);
	testing.expect_value(t, physics.collision_batcher_add(
		&wrong_metadata_batcher, nonconvex_parent_index, compound_index, pose, pose, 0, 56,
	), physics.Physics_Status.Ok);
	testing.expect_value(
		t, physics.collision_batcher_flush(&wrong_metadata_batcher),
		physics.Physics_Status.Invalid_Description,
	);
	testing.expect_value(t, wrong_metadata_batcher.continuation_count, 0);
}

@(test)
child_feature_composition_preserves_indices_beyond_eight_bits :: proc(t: ^testing.T)
{
	feature_id := i32(0x1234567);
	child_a := 513;
	child_b := 257;
	expected := feature_id ~ (i32(child_a) << 8) ~ (i32(child_b) << 16);
	testing.expect_value(t, physics.collision_child_feature_id(child_a, child_b, feature_id), expected);
}

@(test)
mesh_reduction_consumes_temporary_face_flags_before_child_feature_composition :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 6, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	tasks: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&tasks), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	triangles := [2]physics.Triangle{
		{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}},
		{{2, 0, -2}, {0, 0, 2}, {4, 0, 2}},
	};
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(
		&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool,
	), physics.Physics_Status.Ok);
	mesh_index, mesh_status := physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);
	storage_data: [1]physics.Collision_Batcher_Pair;
	callback := Callback_Context{allowed_child=-1};
	batcher: physics.Collision_Batcher;
	testing.expect_value(t, physics.collision_batcher_initialize(
		&batcher, {memory=&storage_data[0], length=1, id=-1}, &shapes, &tasks, {
			pair_completed=pair_completed, child_pair_completed=child_completed, allow_child_pair=allow_child,
		}, &callback, &pool,
	), physics.Physics_Status.Ok);
	pose_sphere := physics.rigid_pose_identity();
	pose_sphere.position = {0, -0.5, 0};
	testing.expect_value(t, physics.collision_batcher_add(
		&batcher, sphere_index, mesh_index, pose_sphere, physics.rigid_pose_identity(), 0, 88,
	), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.collision_batcher_flush(&batcher), physics.Physics_Status.Ok);
	testing.expect_value(t, callback.result_count, 1);
	testing.expect_value(t, callback.results[0].kind, physics.Manifold_Kind.Nonconvex);
	testing.expect(t, callback.results[0].nonconvex.count > 0);
	for contact_index in 0 ..< int(callback.results[0].nonconvex.count)
	{
		testing.expect_value(t,
			callback.results[0].nonconvex.contacts[contact_index].feature_id & physics.MESH_REDUCTION_FACE_COLLISION_FLAG,
			i32(0),
		);
	}
}

@(test)
u32_candidate_scratch_binds_smaller_worker_partition :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);

	TOTAL_PARENT_CAPACITY :: 10_240;
	TOTAL_CHILD_CAPACITY :: 65_536;
	WORKER_PARENT_CAPACITY :: 2_048;
	WORKER_CHILD_CAPACITY :: 13_108;

	scratch, scratch_status := physics.collision_batcher_take_scratch(
		&pool, TOTAL_PARENT_CAPACITY, TOTAL_CHILD_CAPACITY,
	);
	if !testing.expect_value(t, scratch_status, physics.Physics_Status.Ok)
	{
		return;
	}
	defer physics.collision_batcher_return_scratch(&pool, &scratch);
	testing.expect_value(
		t, scratch.candidate_refs.format,
		physics.Collision_Reduction_Candidate_Format.Packed_U32,
	);

	candidate_count := WORKER_CHILD_CAPACITY * physics.MAXIMUM_MANIFOLD_CONTACT_COUNT;
	candidate_refs, candidate_status := physics.collision_candidate_storage_slice(
		scratch.candidate_refs, 0, candidate_count,
	);
	if !testing.expect_value(t, candidate_status, physics.Physics_Status.Ok)
	{
		return;
	}
	children, children_status := physics.collision_child_storage_partition_slice(
		scratch.children, 0, WORKER_CHILD_CAPACITY,
	);
	if !testing.expect_value(t, children_status, physics.Physics_Status.Ok)
	{
		return;
	}
	worker_scratch := physics.Collision_Batcher_Scratch{
		children=children,
		continuations={
			memory=scratch.continuations.memory,
			length=WORKER_PARENT_CAPACITY,
			id=-1,
		},
		candidate_refs=candidate_refs,
	};
	batcher: physics.Collision_Batcher;
	testing.expect_value(
		t,
		physics.collision_batcher_bind_scratch(
		&batcher, worker_scratch,
		WORKER_PARENT_CAPACITY, WORKER_CHILD_CAPACITY,
	),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(
		t, batcher.candidate_refs.format,
		physics.Collision_Reduction_Candidate_Format.Packed_U32,
	);
}
