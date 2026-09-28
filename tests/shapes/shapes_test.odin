package shapes_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Shape :: struct
{
	half_extent: f32,
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 20
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 131072, power), util.Memory_Status.Ok);
	}
}

expect_near :: proc(t: ^testing.T, actual, expected, epsilon: f32)
{
	testing.expect(t, abs(actual - expected) <= epsilon);
}

custom_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = orientation;
	_ = registry;
	half_extent := (^Custom_Shape)(shape).half_extent;
	if half_extent <= 0
	{
		return {}, .Invalid_Description;
	}
	extent := util.Vector3{half_extent, half_extent, half_extent};
	return {min=util.vector3_negate(extent), max=extent, maximum_radius=math.sqrt(3 * half_extent * half_extent)}, .Ok;
}

custom_inertia :: proc "contextless" (
	shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	half_extent := (^Custom_Shape)(shape).half_extent;
	return physics.box_inertia({half_extent, half_extent, half_extent}, mass);
}

custom_support :: proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	half_extent := (^Custom_Shape)(shape).half_extent;
	return physics.box_support({half_extent, half_extent, half_extent}, direction);
}

custom_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	half_extent := (^Custom_Shape)(shape).half_extent;
	return physics.box_ray_test({half_extent, half_extent, half_extent}, pose, ray);
}

cube_data :: proc() -> (
	points: [8]util.Vector3, face_starts: [6]i32, face_indices: [24]i32, triangles: [12]physics.Triangle,
)
{
	points = {
		{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
	};
	face_starts = {0, 4, 8, 12, 16, 20};
	face_indices = {
		0, 3, 2, 1,
		4, 5, 6, 7,
		0, 4, 7, 3,
		1, 2, 6, 5,
		0, 1, 5, 4,
		3, 7, 6, 2,
	};
	for face in 0 ..< len(face_starts)
	{
		start := int(face_starts[face]);
		a := points[face_indices[start]];
		b := points[face_indices[start + 1]];
		c := points[face_indices[start + 2]];
		d := points[face_indices[start + 3]];
		triangles[face * 2] = {a, b, c};
		triangles[face * 2 + 1] = {a, c, d};
	}
	return;
}

@(test)
primitive_shapes_match_pinned_bounds_inertia_support_and_wide_bounds :: proc(t: ^testing.T)
{
	identity := util.quaternion_identity();
	sphere := physics.Sphere{2};
	sphere_bounds := physics.sphere_bounds(sphere, identity);
	sphere_inertia, sphere_status := physics.sphere_inertia(sphere, 2);
	sphere_support, sphere_support_status := physics.sphere_support(sphere, {1, 0, 0});
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, sphere_bounds.min.x, f32(-2));
	testing.expect_value(t, sphere_support.x, f32(2));
	expect_near(t, sphere_inertia.inverse_inertia_tensor.xx, 0.3125, 1e-6);

	capsule := physics.Capsule{1, 2};
	capsule_bounds := physics.capsule_bounds(capsule, identity);
	capsule_inertia, capsule_status := physics.capsule_inertia(capsule, 3);
	capsule_support, capsule_support_status := physics.capsule_support(capsule, {0, 1, 0});
	testing.expect_value(t, capsule_status, physics.Physics_Status.Ok);
	testing.expect_value(t, capsule_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, capsule_bounds.max.y, f32(3));
	testing.expect_value(t, capsule_support.y, f32(3));
	testing.expect(t, capsule_inertia.inverse_inertia_tensor.xx > 0);

	box := physics.Box{1, 2, 3};
	box_bounds := physics.box_bounds(box, identity);
	box_inertia, box_status := physics.box_inertia(box, 6);
	box_support, box_support_status := physics.box_support(box, {-1, 1, -1});
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_bounds.max.z, f32(3));
	testing.expect_value(t, box_support, util.Vector3{-1, 2, -3});
	expect_near(t, box_inertia.inverse_inertia_tensor.xx, 0.03846154, 1e-6);

	triangle := physics.Triangle{{-1, 0, -1}, {1, 0, -1}, {0, 0, 1}};
	triangle_bounds := physics.triangle_bounds(triangle, identity);
	triangle_inertia, triangle_status := physics.triangle_inertia(triangle, 2);
	triangle_support, triangle_support_status := physics.triangle_support(triangle, {0, 0, 1});
	testing.expect_value(t, triangle_status, physics.Physics_Status.Ok);
	testing.expect_value(t, triangle_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, triangle_bounds.min.x, f32(-1));
	testing.expect_value(t, triangle_support.z, f32(1));
	testing.expect(t, triangle_inertia.inverse_inertia_tensor.xx > 0);

	cylinder := physics.Cylinder{1, 2};
	cylinder_bounds := physics.cylinder_bounds(cylinder, identity);
	cylinder_inertia, cylinder_status := physics.cylinder_inertia(cylinder, 4);
	cylinder_support, cylinder_support_status := physics.cylinder_support(cylinder, {1, 1, 0});
	testing.expect_value(t, cylinder_status, physics.Physics_Status.Ok);
	testing.expect_value(t, cylinder_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, cylinder_bounds.max.y, f32(2));
	testing.expect_value(t, cylinder_support, util.Vector3{1, 2, 0});
	testing.expect(t, cylinder_inertia.inverse_inertia_tensor.yy > 0);
	inside_cylinder_hit, inside_cylinder_status := physics.cylinder_ray_test(
		cylinder, physics.rigid_pose_identity(), {direction={0, 1, 0}, maximum_t=10},
	);
	testing.expect_value(t, inside_cylinder_status, physics.Physics_Status.Ok);
	testing.expect_value(t, inside_cylinder_hit.state, physics.Reference_State.Present);
	testing.expect_value(t, inside_cylinder_hit.t, f32(0));

	orientation_wide := util.quaternion_wide_broadcast(identity);
	wide_box := physics.Box_Wide{util.F32x8(1), util.F32x8(2), util.F32x8(3)};
	wide_min, wide_max := physics.box_wide_bounds(wide_box, orientation_wide);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(wide_min.x, lane), f32(-1));
		testing.expect_value(t, simd.extract(wide_max.y, lane), f32(2));
		testing.expect_value(t, simd.extract(wide_max.z, lane), f32(3));
	}
	wide_triangle := physics.Triangle_Wide{
		a=util.vector3_wide_broadcast(triangle.a),
		b=util.vector3_wide_broadcast(triangle.b),
		c=util.vector3_wide_broadcast(triangle.c),
	};
	triangle_wide_min, triangle_wide_max := physics.triangle_wide_bounds(wide_triangle, orientation_wide);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(triangle_wide_min.x, lane), f32(-1));
		testing.expect_value(t, simd.extract(triangle_wide_max.z, lane), f32(1));
	}
}

@(test)
registry_executes_all_nine_builtins_custom_dispatch_and_owned_child_lifetimes :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);

	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.5, 1};
	box := physics.Box{1, 1, 1};
	triangle := physics.Triangle{{-1, 0, -1}, {1, 0, -1}, {0, 0, 1}};
	cylinder := physics.Cylinder{0.75, 1.5};
	sphere_index, sphere_status := physics.shape_registry_add(&registry, physics.SPHERE_TYPE_ID, &sphere);
	capsule_index, capsule_status := physics.shape_registry_add(&registry, physics.CAPSULE_TYPE_ID, &capsule);
	box_index, box_status := physics.shape_registry_add(&registry, physics.BOX_TYPE_ID, &box);
	triangle_index, triangle_status := physics.shape_registry_add(&registry, physics.TRIANGLE_TYPE_ID, &triangle);
	cylinder_index, cylinder_status := physics.shape_registry_add(&registry, physics.CYLINDER_TYPE_ID, &cylinder);
	primitive_statuses := [5]physics.Physics_Status{
		sphere_status,
		capsule_status,
		box_status,
		triangle_status,
		cylinder_status,
	};
	for status in primitive_statuses
	{
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}

	points, face_starts, face_indices, triangles := cube_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	hull_index, hull_status := physics.shape_registry_add(&registry, physics.CONVEX_HULL_TYPE_ID, &hull);
	testing.expect_value(t, hull_status, physics.Physics_Status.Ok);
	wide_hull: physics.Convex_Hull_Wide;
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		wide_hull.hulls[lane] = &hull;
	}
	wide_hull_min, wide_hull_max, wide_hull_status := physics.convex_hull_wide_bounds(
		wide_hull, util.quaternion_wide_broadcast(util.quaternion_identity()),
	);
	testing.expect_value(t, wide_hull_status, physics.Physics_Status.Ok);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, simd.extract(wide_hull_min.x, lane), f32(-1));
		testing.expect_value(t, simd.extract(wide_hull_max.z, lane), f32(1));
	}

	children := [2]physics.Compound_Child{
		{local_position={-2, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
		{local_position={2, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=box_index},
	};
	compound: physics.Compound;
	testing.expect_value(
		t,
		physics.compound_create(&compound, &children[0], len(children), &pool),
		physics.Physics_Status.Ok
	);
	compound_index, compound_status := physics.shape_registry_add(&registry, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	big_compound: physics.Big_Compound;
	testing.expect_value(
		t,
		physics.big_compound_create(&big_compound, &children[0], len(children), &registry, &pool),
		physics.Physics_Status.Ok
	);
	big_compound_index, big_compound_status := physics.shape_registry_add(
		&registry,
		physics.BIG_COMPOUND_TYPE_ID,
		&big_compound
	);
	testing.expect_value(t, big_compound_status, physics.Physics_Status.Ok);
	mesh: physics.Mesh;
	testing.expect_value(
		t,
		physics.mesh_create(&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool),
		physics.Physics_Status.Ok
	);
	mesh_index, mesh_status := physics.shape_registry_add(&registry, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, mesh_status, physics.Physics_Status.Ok);

	indices := [9]physics.Typed_Index{
		sphere_index, capsule_index, box_index, triangle_index, cylinder_index,
		hull_index, compound_index, big_compound_index, mesh_index,
	};
	for index in indices
	{
		bounds, bounds_status := physics.shape_registry_compute_bounds(&registry, index, util.quaternion_identity());
		inertia, inertia_status := physics.shape_registry_compute_inertia(&registry, index, 6);
		testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
		testing.expect(t, bounds.min.x <= bounds.max.x);
		if index.packed == compound_index.packed || index.packed == big_compound_index.packed
		{
			testing.expect_value(t, inertia_status, physics.Physics_Status.Invalid_Argument);
		}
		else
		{
			testing.expect_value(t, inertia_status, physics.Physics_Status.Ok);
			testing.expect(t, inertia.inverse_mass > 0);
		}
		ray := physics.Tree_Ray{origin={-10, 0, 0}, direction={1, 0, 0}, maximum_t=20};
		if index.packed == triangle_index.packed
		{
			ray = {origin={0, 10, 0}, direction={0, -1, 0}, maximum_t=20};
		}
		hit, ray_status := physics.shape_registry_ray_test(&registry, index, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, ray_status, physics.Physics_Status.Ok);
		testing.expect_value(t, hit.state, physics.Reference_State.Present);
	}
	child_masses := [2]f32{2, 4};
	compound_inertia, compound_inertia_status := physics.compound_inertia_weighted(
		&compound, &registry, &child_masses[0], len(child_masses),
	);
	big_compound_inertia, big_compound_inertia_status := physics.big_compound_inertia_weighted(
		&big_compound, &registry, &child_masses[0], len(child_masses),
	);
	testing.expect_value(t, compound_inertia_status, physics.Physics_Status.Ok);
	testing.expect_value(t, big_compound_inertia_status, physics.Physics_Status.Ok);
	testing.expect(t, compound_inertia.inverse_mass > 0);
	testing.expect(t, big_compound_inertia.inverse_mass > 0);
	hull_support, hull_support_status := physics.shape_registry_support(&registry, hull_index, {1, 1, 1});
	testing.expect_value(t, hull_support_status, physics.Physics_Status.Ok);
	testing.expect_value(t, hull_support, util.Vector3{1, 1, 1});
	hull_inertia, hull_inertia_status := physics.shape_registry_compute_inertia(&registry, hull_index, 6);
	mesh_inertia, mesh_inertia_status := physics.shape_registry_compute_inertia(&registry, mesh_index, 6);
	testing.expect_value(t, hull_inertia_status, physics.Physics_Status.Ok);
	testing.expect_value(t, mesh_inertia_status, physics.Physics_Status.Ok);
	expect_near(t, hull_inertia.inverse_inertia_tensor.xx, 0.25, 1e-5);
	expect_near(t, mesh_inertia.inverse_inertia_tensor.xx, 0.25, 1e-5);
	testing.expect_value(
		t,
		physics.shape_registry_remove(&registry, sphere_index),
		physics.Physics_Status.Shape_In_Use
	);

	registration := physics.Shape_Type_Registration{
		size=size_of(Custom_Shape),
		alignment=align_of(Custom_Shape),
		batch_type=.Convex,
		bounds=custom_bounds,
		inertia=custom_inertia,
		ray=custom_ray,
		support=custom_support,
		sweep_support=custom_support,
		dispose=physics.shape_no_dispose,
	};
	custom_type, custom_registration_status := physics.shape_registry_register_custom(&registry, registration);
	testing.expect_value(t, custom_registration_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_type, i32(physics.BUILT_IN_SHAPE_TYPE_COUNT));
	custom := Custom_Shape{2};
	custom_index, custom_status := physics.shape_registry_add(&registry, int(custom_type), &custom);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	custom_bounds_result, custom_bounds_status := physics.shape_registry_compute_bounds(
		&registry,
		custom_index,
		util.quaternion_identity()
	);
	testing.expect_value(t, custom_bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_bounds_result.max, util.Vector3{2, 2, 2});
	testing.expect_value(t, physics.shape_registry_retain(&registry, custom_index), physics.Physics_Status.Ok);
	testing.expect_value(
		t,
		physics.shape_registry_remove(&registry, custom_index),
		physics.Physics_Status.Shape_In_Use
	);
	testing.expect_value(t, physics.shape_registry_release(&registry, custom_index), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.shape_registry_remove(&registry, custom_index), physics.Physics_Status.Ok);
}

@(test)
shape_creation_rejects_degenerate_and_empty_boundaries :: proc(t: ^testing.T)
{
	_, sphere_status := physics.sphere_inertia({0}, 1);
	_, capsule_status := physics.capsule_inertia({1, 0}, 1);
	_, box_status := physics.box_inertia({1, 0, 1}, 1);
	_, triangle_status := physics.triangle_inertia({{}, {}, {}}, 1);
	_, cylinder_status := physics.cylinder_inertia({0, 1}, 1);
	statuses := [5]physics.Physics_Status{sphere_status, capsule_status, box_status, triangle_status, cylinder_status};
	for status in statuses
	{
		testing.expect(t, status != .Ok);
	}
}

@(test)
point_cloud_hull_construction_recenters_and_preserves_source_face_results :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	cube_points, _, _, _ := cube_data();
	points: [11]util.Vector3;
	translation := util.Vector3{3, -4, 5};
	for index in 0 ..< len(cube_points)
	{
		points[index] = util.vector3_add(cube_points[index], translation);
	}
	points[8] = translation;
	points[9] = util.vector3_add(translation, {0.25, -0.5, 0.75});
	points[10] = util.vector3_add(translation, {-0.5, 0.25, -0.25});
	hull: physics.Convex_Hull;
	center: util.Vector3;
	status := physics.convex_hull_create_from_point_cloud(&hull, &center, &points[0], len(points), &pool);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	if status != .Ok
	{
		return;
	}
	defer physics.convex_hull_dispose(&hull, &pool);
	expect_near(t, center.x, translation.x, 1e-4);
	expect_near(t, center.y, translation.y, 1e-4);
	expect_near(t, center.z, translation.z, 1e-4);
	bounds := physics.convex_hull_bounds(&hull, util.quaternion_identity());
	expect_near(t, bounds.min.x, -1, 1e-4);
	expect_near(t, bounds.max.z, 1, 1e-4);
	inertia, inertia_status := physics.convex_hull_inertia(&hull, 6);
	testing.expect_value(t, inertia_status, physics.Physics_Status.Ok);
	expect_near(t, inertia.inverse_inertia_tensor.xx, 0.25, 1e-4);
}

@(test)
shape_slots_remain_stable_when_an_unrelated_shape_is_removed_and_reused :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);
	shapes := [3]physics.Sphere{{1}, {2}, {3}};
	indices: [3]physics.Typed_Index;
	for index in 0 ..< len(shapes)
	{
		shape_index, status := physics.shape_registry_add(&registry, physics.SPHERE_TYPE_ID, &shapes[index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		indices[index] = shape_index;
	}
	testing.expect_value(t, physics.shape_registry_remove(&registry, indices[1]), physics.Physics_Status.Ok);
	bounds, bounds_status := physics.shape_registry_compute_bounds(&registry, indices[2], util.quaternion_identity());
	testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, bounds.max.x, f32(3));
	replacement := physics.Sphere{4};
	replacement_index, replacement_status := physics.shape_registry_add(
		&registry,
		physics.SPHERE_TYPE_ID,
		&replacement
	);
	testing.expect_value(t, replacement_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.typed_index_index(replacement_index), physics.typed_index_index(indices[1]));
	bounds, bounds_status = physics.shape_registry_compute_bounds(&registry, indices[2], util.quaternion_identity());
	testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, bounds.max.x, f32(3));
}

@(test)
shape_batch_growth_preserves_data_while_pool_expands :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 1, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);
	sphere := physics.Sphere{3};
	shape_index, add_status := physics.shape_registry_add(&registry, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	batch := &registry.batches[physics.SPHERE_TYPE_ID];
	saved_next_index := batch.ids.next_index;
	saved_slot_count := batch.slot_count;
	saved_reference_value := batch.references.memory[0];
	blocked: [32]util.Buffer(i32);
	blocked_count := 0;
	for blocked_count < len(blocked)
	{
		buffer, status := util.buffer_pool_take_at_least(&pool, i32, 1024);
		if status != .Ok
		{
			break;
		}
		blocked[blocked_count] = buffer;
		blocked_count += 1;
	}
	testing.expect(t, blocked_count > 0);
	testing.expect_value(
		t, physics.shape_batch_ensure_capacity(&registry, physics.SPHERE_TYPE_ID, 1024),
		physics.Physics_Status.Ok,
	);
	testing.expect_value(t, batch.ids.next_index, saved_next_index);
	testing.expect_value(t, batch.slot_count, saved_slot_count);
	testing.expect_value(t, batch.references.memory[0], saved_reference_value);
	bounds, bounds_status := physics.shape_registry_compute_bounds(&registry, shape_index, util.quaternion_identity());
	testing.expect_value(t, bounds_status, physics.Physics_Status.Ok);
	testing.expect_value(t, bounds.max.x, f32(3));
	for index in 0 ..< blocked_count
	{
		_ = util.buffer_pool_return(&pool, &blocked[index]);
	}
	testing.expect_value(t, batch.references.memory[batch.references.length - 1], i32(-1));
}
