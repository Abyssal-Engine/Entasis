package sweeps_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Sweep_Shape :: struct
{
	radius: f32,
}

prepare_pool :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	testing.expect_value(t, util.buffer_pool_initialize(pool, 128), util.Memory_Status.Ok);
	for power in 0 ..= 20
	{
		testing.expect_value(t, util.buffer_pool_ensure_capacity_for_power(pool, 131072, power), util.Memory_Status.Ok);
	}
}

custom_sweep_shape_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = orientation;
	_ = registry;
	radius := (^Custom_Sweep_Shape)(shape).radius;
	if radius <= 0
	{
		return {}, .Invalid_Description;
	}
	extent := util.Vector3{radius, radius, radius};
	return {min=util.vector3_negate(extent), max=extent, maximum_radius=radius}, .Ok;
}

custom_sweep_shape_inertia :: proc "contextless" (
	shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_inertia({(^Custom_Sweep_Shape)(shape).radius}, mass);
}

custom_sweep_shape_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray,
	registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_ray_test({(^Custom_Sweep_Shape)(shape).radius}, pose, ray);
}

custom_sweep_shape_support :: proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_support({(^Custom_Sweep_Shape)(shape).radius}, direction);
}

custom_sweep :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: physics.Rigid_Pose, velocity_a, velocity_b: physics.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^physics.Shape_Registry, collision_tasks: ^physics.Collision_Task_Registry,
	filter: physics.Collision_Child_Filter_Proc, user_context: rawptr,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	_ = shape_a;
	_ = shape_b;
	_ = type_a;
	_ = type_b;
	_ = pose_a;
	_ = pose_b;
	_ = velocity_a;
	_ = velocity_b;
	_ = maximum_t;
	_ = minimum_progression;
	_ = convergence_threshold;
	_ = maximum_iteration_count;
	_ = shapes;
	_ = collision_tasks;
	_ = filter;
	_ = user_context;
	return {state=.Hit, t0=0.24, t1=0.25, normal={0, 1, 0}}, .Ok;
}

custom_child_sweep :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	parent_pose_a, parent_pose_b: physics.Rigid_Pose,
	local_pose_a, local_pose_b: physics.Rigid_Pose,
	velocity_a, velocity_b: physics.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^physics.Shape_Registry,
	collision_tasks: ^physics.Collision_Task_Registry,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	expected_parent_a := physics.rigid_pose_identity();
	expected_parent_b := physics.rigid_pose_identity();
	expected_parent_b.position = {1, 0, 0};
	expected_local_a := physics.rigid_pose_identity();
	expected_local_b := physics.rigid_pose_identity();
	expected_local_b.position = local_pose_b.position;
	expected_velocity_a := physics.Body_Velocity{linear={1, 2, 3}, angular={4, 5, 6}};
	expected_velocity_b := physics.Body_Velocity{linear={-1, -2, -3}, angular={0, 2, 0}};
	if shape_a == nil || shape_b == nil || type_a != physics.SPHERE_TYPE_ID ||
		type_b < physics.BUILT_IN_SHAPE_TYPE_COUNT ||
		parent_pose_a.position != expected_parent_a.position ||
		parent_pose_a.orientation != expected_parent_a.orientation ||
		parent_pose_b.position != expected_parent_b.position ||
		parent_pose_b.orientation != expected_parent_b.orientation ||
		local_pose_a.position != expected_local_a.position ||
		local_pose_a.orientation != expected_local_a.orientation ||
		local_pose_b.position != expected_local_b.position ||
		local_pose_b.orientation != expected_local_b.orientation ||
		(local_pose_b.position.x != 2 && local_pose_b.position.x != 4) ||
		local_pose_b.position.y != 0 || local_pose_b.position.z != 0 ||
		velocity_a.linear != expected_velocity_a.linear ||
		velocity_a.angular != expected_velocity_a.angular ||
		velocity_b.linear != expected_velocity_b.linear ||
		velocity_b.angular != expected_velocity_b.angular ||
		maximum_t <= 0 || minimum_progression != 1e-5 ||
		convergence_threshold != 1e-5 || maximum_iteration_count != 32 ||
		shapes == nil || collision_tasks == nil
	{
		return {}, .Invalid_Description;
	}
	hit_t := f32(0.4);
	if local_pose_b.position.x == 4
	{
		hit_t = 0.2;
	}
	return {
		state=.Hit, t0=hit_t-0.01, t1=hit_t, normal={0, 1, 0},
		child_a=-1, child_b=-1,
	}, .Ok;
}

Sweep_Filter_Count :: struct
{
	count:          int,
	expected_a:     i32,
	expected_b:     i32,
	mismatch_count: int,
}

Custom_Child_Filter_Context :: struct
{
	compound_side: physics.Reference_State,
	count:         int,
	seen_mask:     u32,
	mismatch_count: int,
}

custom_child_filter :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> physics.Collision_Testing_State
{
	_ = pair_id;
	filter := (^Custom_Child_Filter_Context)(user_context);
	if filter == nil
	{
		return .Reject;
	}
	filter.count += 1;
	compound_child := child_b;
	if filter.compound_side == .Present
	{
		compound_child = child_a;
		if child_b != 0
		{
			filter.mismatch_count += 1;
		}
	}
	else if child_a != 0
	{
		filter.mismatch_count += 1;
	}
	if compound_child < 0 || compound_child >= 2
	{
		filter.mismatch_count += 1;
		return .Reject;
	}
	filter.seen_mask |= u32(1) << u32(compound_child);
	return .Allow;
}

count_sweep_child_filter :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> physics.Collision_Testing_State
{
	_ = pair_id;
	ctx := (^Sweep_Filter_Count)(user_context);
	ctx.count += 1;
	if child_a != ctx.expected_a || child_b != ctx.expected_b
	{
		ctx.mismatch_count += 1;
	}
	return .Allow;
}

@(test)
sweep_registry_is_dense_complete_and_accepts_caller_tasks :: proc(t: ^testing.T)
{
	registry: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.sweep_task_registry_initialize(&registry), physics.Physics_Status.Ok);
	testing.expect_value(t, registry.task_count, physics.BUILT_IN_SWEEP_TASK_COUNT);
	for type_a in 0 ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
	{
		for type_b in type_a ..< physics.BUILT_IN_SHAPE_TYPE_COUNT
		{
			_, _, status := physics.sweep_task_registry_lookup(&registry, type_a, type_b);
			if type_a == physics.MESH_TYPE_ID && type_b == physics.MESH_TYPE_ID
			{
				testing.expect_value(t, status, physics.Physics_Status.Not_Found);
			}
			else
			{
				testing.expect_value(t, status, physics.Physics_Status.Ok);
				task, _, _ := physics.sweep_task_registry_lookup(&registry, type_a, type_b);
				if type_b <= physics.CONVEX_HULL_TYPE_ID
				{
					testing.expect(t, task.test == physics.sweep_task_test_convex_distance);
				}
				else if type_b == physics.MESH_TYPE_ID
				{
					if type_a <= physics.CONVEX_HULL_TYPE_ID
					{
						testing.expect(t, task.test == physics.sweep_task_test_convex_homogeneous_compound_distance);
					}
					else
					{
						testing.expect(t, task.test == physics.sweep_task_test_compound_homogeneous_compound_distance);
					}
				}
				else if type_a <= physics.CONVEX_HULL_TYPE_ID
				{
					testing.expect(t, task.test == physics.sweep_task_test_convex_compound_distance);
				}
				else
				{
					testing.expect(t, task.test == physics.sweep_task_test_compound_pair_distance);
				}
			}
		}
	}
	custom_type := physics.BUILT_IN_SHAPE_TYPE_COUNT;
	_, missing_child_status := physics.sweep_task_registry_register(
		&registry, custom_type, custom_type, custom_sweep, nil,
	);
	testing.expect_value(t, missing_child_status, physics.Physics_Status.Invalid_Argument);
	custom_id, custom_status := physics.sweep_task_registry_register(
		&registry, custom_type, custom_type, custom_sweep, custom_child_sweep,
	);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_id, i32(physics.BUILT_IN_SWEEP_TASK_COUNT));
	task, _, lookup_status := physics.sweep_task_registry_lookup(&registry, custom_type, custom_type);
	testing.expect_value(t, lookup_status, physics.Physics_Status.Ok);
	custom_shape := physics.Sphere{1};
	result, run_status := task.test(
		&custom_shape, &custom_shape, custom_type, custom_type,
		physics.rigid_pose_identity(), physics.rigid_pose_identity(), {}, {},
		1, 1e-4, 1e-4, 32, nil, nil, nil, nil,
	);
	testing.expect_value(t, run_status, physics.Physics_Status.Ok);
	testing.expect_value(t, result.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, result.t1, f32(0.25));
}

@(test)
linear_sphere_sweep_reports_source_time_bracket_normal_and_initial_overlap :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	sphere_a, status_a := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	sphere_b, status_b := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, status_a, physics.Physics_Status.Ok);
	testing.expect_value(t, status_b, physics.Physics_Status.Ok);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {5, 0, 0};
	velocity_b := physics.Body_Velocity{linear={-1, 0, 0}};
	result, status := physics.sweep_task_registry_test(
		&sweeps, sphere_a, sphere_b, pose_a, pose_b, {}, velocity_b,
		10, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, result.state, physics.Sweep_Hit_State.Hit);
	testing.expect(t, abs(result.t1 - 3) < 2e-4);
	testing.expect(t, result.t0 <= result.t1);
	testing.expect(t, util.vector3_dot(result.normal, util.Vector3{-1, 0, 0}) > 0.999);

	overlap_pose := pose_b;
	overlap_pose.position = {1, 0, 0};
	overlap, overlap_status := physics.sweep_task_registry_test(
		&sweeps, sphere_a, sphere_b, pose_a, overlap_pose, {}, {},
		10, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, overlap_status, physics.Physics_Status.Ok);
	testing.expect_value(t, overlap.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, overlap.t1, f32(0));
}

@(test)
axial_sphere_box_sweep_is_exact_for_faces_edges_motion_and_flipped_order :: proc(t: ^testing.T)
{
	sphere := physics.Sphere{0.5};
	box := physics.Box{0.5, 0.5, 0.5};
	sphere_pose := physics.rigid_pose_identity();
	box_pose := physics.rigid_pose_identity();
	sphere_pose.position = {-5, 0, 0};

	face, face_handled := physics.sweep_sphere_box_axial_linear(
		sphere, box, sphere_pose, box_pose, {linear={1, 0, 0}}, {}, 10,
	);
	testing.expect(t, face_handled);
	testing.expect_value(t, face.state, physics.Sweep_Hit_State.Hit);
	testing.expect(t, abs(face.t1 - 4) < 1e-6);
	testing.expect(t, util.vector3_dot(face.normal, util.Vector3{-1, 0, 0}) > 0.99999);

	edge_pose := sphere_pose;
	edge_pose.position.y = 0.75;
	edge, edge_handled := physics.sweep_sphere_box_axial_linear(
		sphere, box, edge_pose, box_pose, {linear={1, 0, 0}}, {}, 10,
	);
	testing.expect(t, edge_handled);
	testing.expect_value(t, edge.state, physics.Sweep_Hit_State.Hit);
	expected_edge_t := f32(4.5) - f32(0.4330127018922193);
	testing.expect(t, abs(edge.t1 - expected_edge_t) < 1e-6);

	miss_pose := sphere_pose;
	miss_pose.position.y = 1.01;
	miss, miss_handled := physics.sweep_sphere_box_axial_linear(
		sphere, box, miss_pose, box_pose, {linear={1, 0, 0}}, {}, 10,
	);
	testing.expect(t, miss_handled);
	testing.expect_value(t, miss.state, physics.Sweep_Hit_State.Miss);

	_, diagonal_handled := physics.sweep_sphere_box_axial_linear(
		sphere, box, sphere_pose, box_pose, {linear={1, 1, 0}}, {}, 10,
	);
	testing.expect(t, !diagonal_handled);

	overlap_pose := sphere_pose;
	overlap_pose.position = {0, 0, 0};
	overlap, overlap_handled := physics.sweep_sphere_box_axial_linear(
		sphere, box, overlap_pose, box_pose, {}, {}, 10,
	);
	testing.expect(t, overlap_handled);
	testing.expect_value(t, overlap.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, overlap.t1, f32(0));

	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	box_index, box_status := physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	forward, forward_status := physics.sweep_task_registry_test(
		&sweeps, sphere_index, box_index, sphere_pose, box_pose, {linear={1, 0, 0}}, {},
		10, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	reverse, reverse_status := physics.sweep_task_registry_test(
		&sweeps, box_index, sphere_index, box_pose, sphere_pose, {}, {linear={1, 0, 0}},
		10, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
	testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
	testing.expect_value(t, forward.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, reverse.state, physics.Sweep_Hit_State.Hit);
	testing.expect(t, abs(forward.t1 - 4) < 1e-6);
	testing.expect(t, abs(reverse.t1 - forward.t1) < 1e-6);
	testing.expect(t, util.vector3_dot(forward.normal, reverse.normal) < -0.99999);
}

@(test)
axial_sphere_box_specialization_matches_generic_convex_reference_grid :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	collisions: physics.Collision_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	sphere := physics.Sphere{0.6};
	box := physics.Box{0.75, 0.5, 1.0};
	box_pose := physics.rigid_pose_identity();
	box_pose.position = {1.25, -0.75, 0.5};
	transverse_values := [7]f32{-1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2};
	direction_signs := [2]f32{-1, 1};
	for axis in 0 ..< 3
	{
		for direction_sign in direction_signs
		{
			for transverse_a in transverse_values
			{
				for transverse_b in transverse_values
				{
					local_start := util.Vector3{};
					local_velocity := util.Vector3{};
					if axis == 0
					{
						local_start = {-direction_sign * 4, transverse_a, transverse_b};
						local_velocity.x = direction_sign;
					}
					else if axis == 1
					{
						local_start = {transverse_a, -direction_sign * 4, transverse_b};
						local_velocity.y = direction_sign;
					}
					else
					{
						local_start = {transverse_a, transverse_b, -direction_sign * 4};
						local_velocity.z = direction_sign;
					}
					sphere_pose := physics.rigid_pose_identity();
					sphere_pose.position = util.vector3_add(box_pose.position, local_start);
					velocity := physics.Body_Velocity{linear=local_velocity};
					fast, handled := physics.sweep_sphere_box_axial_linear(
						sphere, box, sphere_pose, box_pose, velocity, {}, 10,
					);
					testing.expect(t, handled);
					reference, reference_status := physics.sweep_task_test_convex_distance(
						&sphere, &box, physics.SPHERE_TYPE_ID, physics.BOX_TYPE_ID,
						sphere_pose, box_pose, velocity, {}, 10, 1e-5, 1e-5, 64,
						&shapes, &collisions, nil, nil,
					);
					testing.expect_value(t, reference_status, physics.Physics_Status.Ok);
					testing.expect_value(t, fast.state, reference.state);
					if fast.state == .Hit
					{
						testing.expect(t, abs(fast.t1 - reference.t1) < 3e-3);
					}
				}
			}
		}
	}
}

@(test)
separating_motion_and_iteration_boundaries_return_explicit_misses_or_rejections :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{1};
	a, _ := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	b, _ := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {5, 0, 0};
	miss, miss_status := physics.sweep_task_registry_test(
		&sweeps, a, b, pose_a, pose_b, {}, {linear={1, 0, 0}},
		10, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, miss_status, physics.Physics_Status.Ok);
	testing.expect_value(t, miss.state, physics.Sweep_Hit_State.Miss);
	_, invalid_status := physics.sweep_task_registry_test(
		&sweeps, a, b, pose_a, pose_b, {}, {}, 10, 0, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, invalid_status, physics.Physics_Status.Invalid_Argument);
}

@(test)
specialized_sphere_and_capsule_distance_families_report_source_core_distances :: proc(t: ^testing.T)
{
	identity := physics.rigid_pose_identity();
	pose_b := identity;
	pose_b.position = {5, 0, 0};
	sphere := physics.Sphere{1};
	box := physics.Box{1, 1, 1};
	triangle := physics.Triangle{{0, -2, -2}, {0, 2, -2}, {0, 0, 2}};
	cylinder := physics.Cylinder{1, 1};
	capsule := physics.Capsule{0.5, 1};
	sphere_box := physics.sweep_sphere_box_distance(sphere, box, identity, pose_b);
	sphere_triangle := physics.sweep_sphere_triangle_distance(sphere, triangle, identity, pose_b);
	sphere_cylinder := physics.sweep_sphere_cylinder_distance(sphere, cylinder, identity, pose_b);
	capsule_pair := physics.sweep_capsule_pair_distance(capsule, capsule, identity, pose_b);
	capsule_box := physics.sweep_capsule_box_distance(capsule, box, identity, pose_b);
	capsule_cylinder := physics.sweep_capsule_cylinder_distance(capsule, cylinder, identity, pose_b);
	testing.expect(t, abs(sphere_box.distance-3) < 1e-5);
	testing.expect(t, abs(sphere_triangle.distance-4) < 1e-5);
	testing.expect(t, abs(sphere_cylinder.distance-3) < 1e-5);
	testing.expect(t, abs(capsule_pair.distance-4) < 1e-5);
	testing.expect(t, abs(capsule_box.distance-3.5) < 1e-4);
	testing.expect(t, abs(capsule_cylinder.distance-3.5) < 1e-4);
	results := [6]physics.Sweep_Distance_Result{
		sphere_box, sphere_triangle, sphere_cylinder, capsule_pair, capsule_box, capsule_cylinder,
	};
	for result in results
	{
		testing.expect_value(t, result.intersected, physics.Reference_State.Missing);
		testing.expect(t, util.vector3_dot(result.normal, util.Vector3{-1, 0, 0}) > 0.999);
	}
}

@(test)
complete_convex_sweep_matrix_executes_repeatably_in_both_orders :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	capsule := physics.Capsule{0.75, 1};
	box := physics.Box{1, 1, 1};
	triangle := physics.Triangle{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}};
	cylinder := physics.Cylinder{1, 1};
	points := [8]util.Vector3{
		{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
		{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
	};
	face_starts := [6]i32{0, 4, 8, 12, 16, 20};
	face_indices := [24]i32{0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &face_starts[0], len(face_starts), &face_indices[0], len(face_indices), &pool,
	), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	indices: [6]physics.Typed_Index;
	indices[0], _ = physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	indices[1], _ = physics.shape_registry_add(&shapes, physics.CAPSULE_TYPE_ID, &capsule);
	indices[2], _ = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	indices[3], _ = physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	indices[4], _ = physics.shape_registry_add(&shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	indices[5], _ = physics.shape_registry_add(&shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	pose_a := physics.rigid_pose_identity();
	pose_b := physics.rigid_pose_identity();
	pose_b.position = {3.5, 0.2, -0.1};
	pose_b.orientation = util.quaternion_from_axis_angle({0.2, 0.8, 0.1}, 0.17);
	for type_a in 0 ..< len(indices)
	{
		for type_b in type_a ..< len(indices)
		{
			forward, forward_status := physics.sweep_task_registry_test(
				&sweeps, indices[type_a], indices[type_b], pose_a, pose_b, {}, {linear={-0.5, 0, 0}},
				10, 1e-5, 1e-5, 64, &shapes, &collisions,
			);
			repeated, repeated_status := physics.sweep_task_registry_test(
				&sweeps, indices[type_a], indices[type_b], pose_a, pose_b, {}, {linear={-0.5, 0, 0}},
				10, 1e-5, 1e-5, 64, &shapes, &collisions,
			);
			reverse, reverse_status := physics.sweep_task_registry_test(
				&sweeps, indices[type_b], indices[type_a], pose_b, pose_a, {linear={-0.5, 0, 0}}, {},
				10, 1e-5, 1e-5, 64, &shapes, &collisions,
			);
			testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
			testing.expect_value(t, repeated_status, physics.Physics_Status.Ok);
			testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
			testing.expect_value(t, forward, repeated);
			testing.expect_value(t, forward.state, reverse.state);
			if forward.state == .Hit && reverse.state == .Hit
			{
				testing.expect(t, abs(forward.t1-reverse.t1) < 2e-3);
			}

			// at unit speed the Y crossing is inside (1, 6), a 20-unit X gap cannot close
			known_a: physics.Rigid_Pose = physics.rigid_pose_identity();
			known_b: physics.Rigid_Pose = physics.rigid_pose_identity();
			known_b.position = {0, -5, 0};
			if type_b == physics.TRIANGLE_TYPE_ID
			{
				known_b.orientation = util.quaternion_from_axis_angle({1, 0, 0}, 3.14159265);
				if type_a == physics.TRIANGLE_TYPE_ID
				{
					// a transverse crossing avoids coincident zero-thickness triangles
					known_b.position.x = -0.2;
					known_b.orientation = util.quaternion_from_axis_angle({0, 0, 1}, 1.57079633);
				}
			}
			for separation in ([2]f32{0, 20})
			{
				separated_b: physics.Rigid_Pose = known_b;
				separated_b.position.x += separation;
				for order in 0 ..< 2
				{
					a: int = type_a if order == 0 else type_b;
					b: int = type_b if order == 0 else type_a;
					first: physics.Rigid_Pose = known_a if order == 0 else separated_b;
					second: physics.Rigid_Pose = separated_b if order == 0 else known_a;
					velocity_a: physics.Body_Velocity;
					velocity_b: physics.Body_Velocity;
					if order == 0
					{
						velocity_b.linear = {0, 1, 0};
					}
					else
					{
						velocity_a.linear = {0, 1, 0};
					}
					result: physics.Sweep_Result;
					status: physics.Physics_Status;
					result, status = physics.sweep_task_registry_test(
						&sweeps, indices[a], indices[b], first, second, velocity_a, velocity_b,
						10, 1e-5, 1e-5, 64, &shapes, &collisions,
					);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					if separation == 0
					{
						testing.expectf(t, result.state == .Hit, "pair %d/%d: expected hit, got %v", a, b, result);
						testing.expectf(t, result.t0 > 1 && result.t0 <= result.t1 && result.t1 < 6,
							"pair %d/%d: time interval %f..%f", a, b, result.t0, result.t1);
						testing.expect(t, abs(util.vector3_dot(result.normal, result.normal)-1) < 1e-3);
						if type_a != physics.TRIANGLE_TYPE_ID || type_b != physics.TRIANGLE_TYPE_ID
						{
							direction: f32 = 1 if order == 0 else -1;
							testing.expectf(t, result.normal.y*direction > 0.99, "pair %d/%d: normal %v", a, b, result.normal);
						}
						for component in ([3]f32{result.location.x, result.location.y, result.location.z})
						{
							testing.expect(t, abs(component) < 10);
						}
					}
					else
					{
						testing.expect_value(t, result.state, physics.Sweep_Hit_State.Miss);
					}
				}
			}
		}
	}
}

@(test)
compound_sweep_selects_the_earliest_child_and_preserves_child_identity :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 6, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	children := [2]physics.Compound_Child{
		{local_position={0, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
		{local_position={4, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=sphere_index},
	};
	compound: physics.Compound;
	testing.expect_value(
		t,
		physics.compound_create(&compound, &children[0], len(children), &pool),
		physics.Physics_Status.Ok
	);
	compound_index, compound_status := physics.shape_registry_add(&shapes, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_a.position = {-5, 0, 0};
	result, status := physics.sweep_task_registry_test(
		&sweeps, sphere_index, compound_index, pose_a, physics.rigid_pose_identity(), {linear={1, 0, 0}}, {},
		12, 1e-5, 1e-5, 64, &shapes, &collisions,
	);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, result.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, result.child_b, i32(0));
	testing.expect(t, abs(result.t1-3) < 2e-3);
}

@(test)
big_compound_sweep_uses_tree_overlap_culling_and_preserves_flipped_child_identity :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 12, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	children: [8]physics.Compound_Child;
	for child_index in 0 ..< len(children)
	{
		children[child_index] = {
			local_position={0, f32(child_index * 20), 0},
			local_orientation=util.quaternion_identity(), shape_index=sphere_index,
		};
	}
	big_compound: physics.Big_Compound;
	testing.expect_value(t, physics.big_compound_create(
		&big_compound, &children[0], len(children), &shapes, &pool,
	), physics.Physics_Status.Ok);
	defer physics.big_compound_dispose(&big_compound, &pool);
	compound_index, compound_status := physics.shape_registry_add(
		&shapes, physics.BIG_COMPOUND_TYPE_ID, &big_compound,
	);
	testing.expect_value(t, compound_status, physics.Physics_Status.Ok);
	pose_a := physics.rigid_pose_identity();
	pose_a.position = {-5, 60, 0};
	filter_context := Sweep_Filter_Count{expected_a=0, expected_b=3};
	forward, forward_status := physics.sweep_task_registry_test(
		&sweeps, sphere_index, compound_index, pose_a, physics.rigid_pose_identity(), {linear={1, 0, 0}}, {},
		12, 1e-5, 1e-5, 64, &shapes, &collisions, count_sweep_child_filter, &filter_context,
	);
	testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
	testing.expect_value(t, forward.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, forward.child_b, i32(3));
	testing.expect(t, filter_context.count > 0 && filter_context.count < len(children));
	testing.expect_value(t, filter_context.mismatch_count, 0);

	filter_context.count = 0;
	filter_context.expected_a = 3;
	filter_context.expected_b = 0;
	filter_context.mismatch_count = 0;
	reverse, reverse_status := physics.sweep_task_registry_test(
		&sweeps, compound_index, sphere_index, physics.rigid_pose_identity(), pose_a, {}, {linear={1, 0, 0}},
		12, 1e-5, 1e-5, 64, &shapes, &collisions, count_sweep_child_filter, &filter_context,
	);
	testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
	testing.expect_value(t, reverse.state, physics.Sweep_Hit_State.Hit);
	testing.expect_value(t, reverse.child_a, i32(3));
	testing.expect(t, abs(forward.t1-reverse.t1) < 2e-4);
	testing.expect(t, util.vector3_dot(forward.normal, reverse.normal) < -0.999);
	testing.expect(t, filter_context.count > 0 && filter_context.count < len(children));
	testing.expect_value(t, filter_context.mismatch_count, 0);
}

@(test)
compound_child_sweep_preserves_custom_local_pose_motion_and_route_identity :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 12, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	custom_registration := physics.Shape_Type_Registration{
		size=size_of(Custom_Sweep_Shape),
		alignment=align_of(Custom_Sweep_Shape),
		batch_type=.Convex,
		bounds=custom_sweep_shape_bounds,
		inertia=custom_sweep_shape_inertia,
		ray=custom_sweep_shape_ray,
		support=custom_sweep_shape_support,
		sweep_support=custom_sweep_shape_support,
		dispose=physics.shape_no_dispose,
	};
	custom_type, registration_status := physics.shape_registry_register_custom(
		&shapes, custom_registration,
	);
	testing.expect_value(t, registration_status, physics.Physics_Status.Ok);
	missing_type, missing_registration_status := physics.shape_registry_register_custom(
		&shapes, custom_registration,
	);
	testing.expect_value(t, missing_registration_status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{10};
	sphere_index, sphere_status := physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	custom := Custom_Sweep_Shape{10};
	custom_index, custom_status := physics.shape_registry_add(&shapes, int(custom_type), &custom);
	missing_custom_index, missing_custom_status := physics.shape_registry_add(
		&shapes, int(missing_type), &custom,
	);
	testing.expect_value(t, sphere_status, physics.Physics_Status.Ok);
	testing.expect_value(t, custom_status, physics.Physics_Status.Ok);
	testing.expect_value(t, missing_custom_status, physics.Physics_Status.Ok);
	children := [2]physics.Compound_Child{
		{local_position={2, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=custom_index},
		{local_position={4, 0, 0}, local_orientation=util.quaternion_identity(), shape_index=custom_index},
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
	missing_child := physics.Compound_Child{
		local_position={2, 0, 0},
		local_orientation=util.quaternion_identity(),
		shape_index=missing_custom_index,
	};
	missing_compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(
		&missing_compound, &missing_child, 1, &pool,
	), physics.Physics_Status.Ok);
	defer physics.compound_dispose(&missing_compound, &pool);
	missing_compound_index, missing_compound_status := physics.shape_registry_add(
		&shapes, physics.COMPOUND_TYPE_ID, &missing_compound,
	);
	testing.expect_value(t, missing_compound_status, physics.Physics_Status.Ok);
	collisions: physics.Collision_Task_Registry;
	sweeps: physics.Sweep_Task_Registry;
	testing.expect_value(t, physics.collision_task_registry_initialize(&collisions), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.sweep_task_registry_initialize(&sweeps), physics.Physics_Status.Ok);
	_, custom_task_status := physics.sweep_task_registry_register(
		&sweeps, physics.SPHERE_TYPE_ID, int(custom_type), custom_sweep, custom_child_sweep,
	);
	testing.expect_value(t, custom_task_status, physics.Physics_Status.Ok);
	parent_pose_a := physics.rigid_pose_identity();
	parent_pose_b := physics.rigid_pose_identity();
	parent_pose_b.position = {1, 0, 0};
	velocity_a := physics.Body_Velocity{linear={1, 2, 3}, angular={4, 5, 6}};
	velocity_b := physics.Body_Velocity{linear={-1, -2, -3}, angular={0, 2, 0}};
	targets := [2]physics.Typed_Index{compound_index, big_compound_index};
	for target in targets
	{
		filter := Custom_Child_Filter_Context{};
		forward, forward_status := physics.sweep_task_registry_test(
			&sweeps, sphere_index, target, parent_pose_a, parent_pose_b, velocity_a, velocity_b,
			1, 1e-5, 1e-5, 32, &shapes, &collisions, custom_child_filter, &filter, &pool,
		);
		testing.expect_value(t, forward_status, physics.Physics_Status.Ok);
		testing.expect_value(t, forward.state, physics.Sweep_Hit_State.Hit);
		testing.expect_value(t, forward.t1, f32(0.2));
		testing.expect_value(t, forward.child_b, i32(1));
		testing.expect_value(t, forward.normal, util.Vector3{0, 1, 0});
		testing.expect(t, filter.count > 0);
		testing.expect(t, (filter.seen_mask & 2) != 0);
		testing.expect_value(t, filter.mismatch_count, 0);

		filter = {compound_side=.Present};
		reverse, reverse_status := physics.sweep_task_registry_test(
			&sweeps, target, sphere_index, parent_pose_b, parent_pose_a, velocity_b, velocity_a,
			1, 1e-5, 1e-5, 32, &shapes, &collisions, custom_child_filter, &filter, &pool,
		);
		testing.expect_value(t, reverse_status, physics.Physics_Status.Ok);
		testing.expect_value(t, reverse.state, physics.Sweep_Hit_State.Hit);
		testing.expect_value(t, reverse.t1, f32(0.2));
		testing.expect_value(t, reverse.child_a, i32(1));
		testing.expect_value(t, reverse.child_b, forward.child_a);
		testing.expect_value(t, reverse.normal, util.Vector3{0, -1, 0});
		testing.expect(t, filter.count > 0);
		testing.expect(t, (filter.seen_mask & 2) != 0);
		testing.expect_value(t, filter.mismatch_count, 0);
	}
	miss, miss_status := physics.sweep_task_registry_test(
		&sweeps, sphere_index, missing_compound_index,
		parent_pose_a, parent_pose_b, velocity_a, velocity_b,
		1, 1e-5, 1e-5, 32, &shapes, &collisions, nil, nil, &pool,
	);
	testing.expect_value(t, miss_status, physics.Physics_Status.Ok);
	testing.expect_value(t, miss.state, physics.Sweep_Hit_State.Miss);
}
