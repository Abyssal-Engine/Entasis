package queries_tests

import "core:testing"
import "core:math"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
box_rays_preserve_nonzero_sub_epsilon_axes_and_exact_endpoints :: proc(t: ^testing.T)
{
	box: physics.Box = {1, 4503599627370496.0, 1};
	pose: physics.Rigid_Pose = physics.rigid_pose_identity();
	query: physics.Wide_Ray_Query = {lane_count=8};
	query.rays[0] = {origin={2, 0, 0}, direction={-4.44089209850062616169452667236328125e-16, 1, 0}, maximum_t=4503599627370496.0};
	query.rays[1] = query.rays[0];
	query.rays[1].maximum_t = 2251799813685248.0;
	query.rays[2] = query.rays[0];
	query.rays[2].maximum_t = math.nextafter_f32(2251799813685248.0, 0);
	negative_zero: f32 = transmute(f32)u32(0x80000000);
	for lane in 3..<8
	{
		query.rays[lane] = {origin={1, -2, 0}, direction={0, 1, 0}, maximum_t=4};
		if lane == 4 || lane == 6
		{
			query.rays[lane].direction.x = negative_zero;
		}
		if lane >= 5
		{
			query.rays[lane].origin.x = 2;
		}
	}
	wide: physics.Wide_Ray_Result = physics.wide_ray_test_box(box, pose, query);
	for ray, lane in query.rays
	{
		scalar: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		scalar, status = physics.box_ray_test(box, pose, ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, wide.status[lane], physics.Physics_Status.Ok);
		expected: physics.Reference_State = .Missing;
		if lane < 2 || lane == 3 || lane == 4
		{
			expected = .Present;
		}
		testing.expect_value(t, scalar.state, expected);
		testing.expect_value(t, wide.hits[lane].state, expected);
		if lane < 2
		{
			for hit in ([2]physics.Shape_Ray_Hit{scalar, wide.hits[lane]})
			{
				testing.expect_value(t, hit.t, f32(2251799813685248.0));
				testing.expect_value(t, hit.normal, util.Vector3{1, 0, 0});
				testing.expect_value(t, util.vector3_add(ray.origin, util.vector3_scale(ray.direction, hit.t)), util.Vector3{1, 2251799813685248.0, 0});
			}
		}
	}
}

resolution_custom_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray,
	registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	if ray.maximum_t == 7
	{
		return {}, .Invalid_Description;
	}
	hit, status := physics.sphere_ray_test((^physics.Sphere)(shape)^, pose, ray);
	if hit.state == .Present
	{
		hit.child_index = 73;
	}
	return hit, status;
}

@(test)
resolved_ray_dispatch_matches_every_primitive_and_custom_registration :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	sphere := physics.Sphere{0.75};
	capsule := physics.Capsule{0.5, 0.75};
	box := physics.Box{0.7, 1.0, 0.8};
	cylinder := physics.Cylinder{0.8, 0.7};
	triangle := physics.Triangle{{-2, 0, -2}, {0, 0, 2}, {2, 0, -2}};
	points, starts, indices := cube_hull_data();
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(
		&hull, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &pool,
	), physics.Physics_Status.Ok);
	defer physics.convex_hull_dispose(&hull, &pool);
	shape_indices: [7]physics.Typed_Index;
	statuses: [7]physics.Physics_Status;
	shape_indices[0], statuses[0] = physics.shape_registry_add(&shapes, physics.SPHERE_TYPE_ID, &sphere);
	shape_indices[1], statuses[1] = physics.shape_registry_add(&shapes, physics.CAPSULE_TYPE_ID, &capsule);
	shape_indices[2], statuses[2] = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
	shape_indices[3], statuses[3] = physics.shape_registry_add(&shapes, physics.CYLINDER_TYPE_ID, &cylinder);
	shape_indices[4], statuses[4] = physics.shape_registry_add(&shapes, physics.TRIANGLE_TYPE_ID, &triangle);
	shape_indices[5], statuses[5] = physics.shape_registry_add(&shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	registration := shapes.batches[physics.SPHERE_TYPE_ID].metadata;
	registration.ray = resolution_custom_ray;
	custom_type, status := physics.shape_registry_register_custom(&shapes, registration);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	shape_indices[6], statuses[6] = physics.shape_registry_add(&shapes, int(custom_type), &sphere);
	for shape_status in statuses
	{
		testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	}
	directions := [8]util.Vector3{
		{1, 0.37, 0.21}, {-0.81, 0.46, -0.35}, {0.42, -0.73, 0.54}, {-0.58, -0.31, 0.75},
		{1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {1e-7, -1, 1e-7},
	};
	modes := [2]physics.Query_Collection_Mode{.Earliest, .All};
	for shape_index, type_slot in shape_indices
	{
		for rotation in 0 ..< 4
		{
			pose := physics.Rigid_Pose{
				position={f32(type_slot) * 4 - 8, 1.25, -2.5},
				orientation=util.quaternion_from_axis_angle({0.36, 0.48, 0.8}, f32(rotation) * 0.37),
			};
			for direction in directions
			{
				for variant in 0 ..< 4
				{
					local_origin := util.vector3_scale(direction, -4);
					if variant == 1
					{
						local_origin = {};
					}
					else if variant == 2
					{
						local_origin = util.vector3_add(local_origin, {0, 5, 3});
					}
					ray := physics.Tree_Ray{
						origin=util.vector3_add(pose.position, util.quaternion_transform(local_origin, pose.orientation)),
						direction=util.quaternion_transform(direction, pose.orientation),
						maximum_t=20,
					};
					if variant == 3
					{
						ray.maximum_t = 0.125;
					}
					reference, reference_status := physics.shape_registry_ray_test(&shapes, shape_index, pose, ray);
					for mode in modes
					{
						hit: physics.Ray_Query_Hit;
						collector := physics.Ray_Query_Collector{
							hits={memory=&hit, length=1, id=-1}, mode=mode,
						};
						actual_status := physics.query_ray_shape(
							&shapes, {shape=shape_index, pose=pose, target_id=i32(type_slot)},
							ray, &collector, &pool,
						);
						testing.expect_value(t, actual_status, reference_status);
						testing.expect_value(t, collector.count > 0, reference.state == .Present);
						if reference.state == .Present
						{
							testing.expect_value(t, hit.t, reference.t);
							testing.expect_value(t, hit.normal, reference.normal);
							testing.expect_value(t, hit.child_index, reference.child_index);
							testing.expect_value(t, hit.target_id, i32(type_slot));
							testing.expect_value(t, hit.ray_id, i32(0));
							testing.expect_value(t, hit.location, util.vector3_add(ray.origin, util.vector3_scale(ray.direction, reference.t)));
						}
					}
				}
			}
		}
	}
}

Resolution_Callback_State :: struct
{
	target_calls: int,
	child_calls:  int,
	hit_calls:    int,
	allow_target: bool,
	allow_child:  bool,
	hit_status:   physics.Physics_Status,
}

resolution_allow_target :: proc "contextless" (user_context: rawptr, target_id: i32) -> physics.Query_Filter_Result
{
	_ = target_id;
	state := (^Resolution_Callback_State)(user_context);
	state.target_calls += 1;
	if state.allow_target
	{
		return .Allow;
	}
	return .Reject;
}

resolution_allow_child :: proc "contextless" (user_context: rawptr, target_id, child_index: i32) -> physics.Query_Filter_Result
{
	_, _ = target_id, child_index;
	state := (^Resolution_Callback_State)(user_context);
	state.child_calls += 1;
	if state.allow_child
	{
		return .Allow;
	}
	return .Reject;
}

resolution_report_hit :: proc "contextless" (
	user_context: rawptr, ray: physics.Tree_Ray, hit: ^physics.Ray_Query_Hit, maximum_t: ^f32,
) -> physics.Physics_Status
{
	_ = ray;
	state := (^Resolution_Callback_State)(user_context);
	state.hit_calls += 1;
	maximum_t^ = min(maximum_t^, hit.t);
	return state.hit_status;
}

@(test)
resolved_ray_dispatch_preserves_filters_errors_and_custom_child_ids :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 4, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	registration := shapes.batches[physics.SPHERE_TYPE_ID].metadata;
	registration.ray = resolution_custom_ray;
	custom_type, status := physics.shape_registry_register_custom(&shapes, registration);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	sphere := physics.Sphere{1};
	shape, add_status := physics.shape_registry_add(&shapes, int(custom_type), &sphere);
	testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	target := physics.Shape_Query_Target{shape=shape, pose=physics.rigid_pose_identity(), target_id=91};
	ray := physics.Tree_Ray{origin={-4, 0, 0}, direction={1, 0, 0}, maximum_t=12};
	for variant in 0 ..< 4
	{
		state := Resolution_Callback_State{allow_target=variant != 0, allow_child=variant != 1};
		if variant == 3
		{
			state.hit_status = .Invalid_Description;
		}
		hit: physics.Ray_Query_Hit;
		collector := physics.Ray_Query_Collector{
			hits={memory=&hit, length=1, id=-1}, mode=.Earliest,
			callbacks={filter={allow_target=resolution_allow_target, allow_child=resolution_allow_child},
				hit=resolution_report_hit, user_context=&state},
		};
		query_status := physics.query_ray_shape(&shapes, target, ray, &collector, &pool);
		testing.expect_value(t, query_status, state.hit_status);
		testing.expect_value(t, state.target_calls, 1);
		testing.expect_value(t, state.child_calls, int(variant != 0));
		testing.expect_value(t, state.hit_calls, int(variant >= 2));
		if variant >= 2
		{
			testing.expect_value(t, hit.child_index, i32(73));
		}
	}
	hit: physics.Ray_Query_Hit;
	collector := physics.Ray_Query_Collector{hits={memory=&hit, length=1, id=-1}, mode=.Earliest};
	ray.maximum_t = 7;
	testing.expect_value(t, physics.query_ray_shape(&shapes, target, ray, &collector, &pool), physics.Physics_Status.Invalid_Description);
	testing.expect_value(t, collector.count, 0);
	testing.expect_value(t, physics.shape_registry_remove(&shapes, shape), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.query_ray_shape(&shapes, target, ray, &collector, &pool), physics.Physics_Status.Not_Found);
}

resolution_mesh_scan :: proc "contextless" (mesh: ^physics.Mesh, pose: physics.Rigid_Pose, ray: physics.Tree_Ray) -> physics.Shape_Ray_Hit
{
	inverse: util.Quaternion = util.quaternion_conjugate(pose.orientation);
	origin: util.Vector3 = util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse);
	direction: util.Vector3 = util.quaternion_transform(ray.direction, inverse);
	best: physics.Shape_Ray_Hit = physics.shape_ray_miss();
	limit: f32 = ray.maximum_t;
	for index in 0..<int(mesh.triangles.length)
	{
		triangle: physics.Triangle = mesh.triangles.memory[index];
		t: f32;
		normal: util.Vector3;
		state: physics.Reference_State;
		t, normal, state = physics.triangle_ray_test_local(
			util.vector3_multiply(triangle.a, mesh.scale), util.vector3_multiply(triangle.b, mesh.scale),
			util.vector3_multiply(triangle.c, mesh.scale), origin, direction,
		);
		if state == .Present && t <= limit
		{
			best = {t=t, normal=util.quaternion_transform(normal, pose.orientation), child_index=i32(index), state=.Present};
			limit = t;
		}
	}
	return best;
}

@(test)
direct_and_nested_mesh_rays_preserve_ties_endpoints_scale_and_deep_continuation :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	// cover complete bundles, every scalar remainder and ascending equal-hit publication
	for count in 1..=17
	{
		triangles: [17]physics.Triangle;
		for &triangle in triangles
		{
			triangle = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}};
		}
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], count, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		for probe in 0..<4
		{
			ray: physics.Tree_Ray = {origin={0.75, 1, 0.25}, direction={0, -1, 0}, maximum_t=1};
			switch probe
			{
			case 1:
				ray.maximum_t = math.nextafter_f32(1, 0);
			case 2:
				ray.origin.x = 2;
			case 3:
				ray.origin.y = -1;
				ray.direction.y = 1;
			}
			reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
			testing.expect_value(t, reference.state, physics.Reference_State.Present if probe == 0 else physics.Reference_State.Missing);
			if probe == 0
			{
				testing.expect_value(t, reference.child_index, i32(count-1));
			}
			actual: physics.Shape_Ray_Hit;
			status: physics.Physics_Status;
			actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, actual, reference);
			testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
			testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
			testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
			testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
		}
	}
	Mesh_Scan_Probe :: struct
	{
		triangle: physics.Triangle,
		ray: physics.Tree_Ray,
		scale: util.Vector3,
	}
	probes: [9]Mesh_Scan_Probe = {
		{triangle={{-1, 0, 0}, {1, 0, 0}, {1, 0, 1}}, ray={origin={math.nextafter_f32(1, math.F32_MAX), 1, 0.25}, direction={0, -1, 0}, maximum_t=1}, scale={1, 1, 1}},
		{triangle={{-1048576, 0, 0}, {0, 0, 0}, {0, 0, 1}}, ray={origin={0.03125, 1, 0.25}, direction={0, -1, 0}, maximum_t=1}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {1, 0, 0}, {1, 0, 1}}, ray={origin={0.75, 1, 0.25}, direction={0, -25, 0}, maximum_t=f32(1)/25}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {1, -1, 0}, {-1, -2, 1}}, ray={origin={1, 0.25, 3.25}, direction={-1, -1, -3}, maximum_t=1}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {1267650600228229401496703205376, 0, 0}, {0, 0, 1.0/1267650600228229401496703205376}}, ray={origin={1267650600228229401496703205376, 1, 1267650600228229401496703205376}, direction={1267650600228229401496703205376, -1, 1267650600228229401496703205376}, maximum_t=1}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {1099511627776, 0, 0}, {0, 0, 1099511627776}}, ray={origin={274877906944, 1, 274877906944}, direction={0, -1, 0}, maximum_t=1}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {0, 0, 1}, {0, 1, 0}}, ray={origin={0, 0.25, 0.25}, direction={-1.0/170141183460469231731687303715884105728, 1, 0}, maximum_t=0}, scale={1, 1, 1}},
		{triangle={{0, 0, 0}, {1, 0, 0}, {1, 0, 1}}, ray={origin={0.75/17179869184, 1, 0.25/17179869184}, direction={0, -1, 0}, maximum_t=1}, scale={1.0/17179869184, 1, 1.0/17179869184}},
		{triangle={{0, 0, 0}, {1, 0, 0}, {1, 0, 1}}, ray={origin={0.75/4294967296, 1, 0.25/4294967296}, direction={0, -1, 0}, maximum_t=1}, scale={1.0/4294967296, 1, 1.0/4294967296}},
	};
	// each exceptional input reaches every complete-bundle lane and the last scalar remainder
	for probe, probe_index in probes
	{
		for placement in 0..=8
		{
			count: int = 8 if placement < 8 else 17;
			index: int = placement if placement < 8 else 16;
			triangles: [17]physics.Triangle;
			for &triangle in triangles
			{
				triangle = {probe.triangle.a, probe.triangle.c, probe.triangle.b};
			}
			triangles[index] = probe.triangle;
			mesh: physics.Mesh;
			testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], count, probe.scale, &pool), physics.Physics_Status.Ok);
			defer physics.mesh_dispose(&mesh, &pool);
			reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), probe.ray);
			if probe_index < 6 || probe_index == 8
			{
				testing.expect_value(t, reference.state, physics.Reference_State.Present);
				testing.expect_value(t, reference.child_index, i32(index));
			}
			if probe_index == 3
			{
				testing.expect_value(t, transmute(u32)reference.normal.z, u32(0x3f678f8b));
			}
			actual: physics.Shape_Ray_Hit;
			status: physics.Physics_Status;
			actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), probe.ray);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, actual, reference);
			testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
			testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
			testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
			testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
		}
	}
	// equal rounded parameters cross both a bundle boundary and the final remainder
	for count in ([3]int{8, 16, 17})
	{
		first: int = 6 if count == 8 else 7 if count == 16 else 15;
		triangles: [17]physics.Triangle;
		for &triangle in triangles
		{
			triangle = {{0, 0, 0}, {1, 0, 1}, {1, 0, 0}};
		}
		triangles[first] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}};
		triangles[first+1] = {{0, -1e-8, 0}, {1, -1e-8, 0}, {1, -1e-8, 1}};
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], count, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		ray: physics.Tree_Ray = {origin={0.75, 1, 0.25}, direction={0, -25, 0}, maximum_t=1};
		reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, reference.child_index, i32(first+1));
		actual: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reference);
		testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
		testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
		testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
		testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
	}
	// a discarded farther hit must not overwrite the closer triangle's different normal
	for order in 0..<2
	{
		triangles: [16]physics.Triangle;
		for &triangle in triangles
		{
			triangle = {probes[3].triangle.a, probes[3].triangle.c, probes[3].triangle.b};
		}
		triangles[7+order] = probes[3].triangle;
		triangles[8-order] = {{0, 0, 0}, {4, 0, 4}, {0, 0, 4}};
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), probes[3].ray);
		testing.expect_value(t, reference.child_index, i32(8-order));
		testing.expect_value(t, reference.t, f32(0.25));
		actual: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), probes[3].ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reference);
		testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
		testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
		testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
		testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
	}
	// the scalar predicate can accept a point beyond a parallel geometric face
	{
		triangle: physics.Triangle = {{-1, 0, 0}, {1, 0, 0}, {1, 0, 1}};
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		ray: physics.Tree_Ray = {origin={math.nextafter_f32(1, math.F32_MAX), 1, 0.25}, direction={0, -1, 0}, maximum_t=1};
		reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, reference.state, physics.Reference_State.Present);
		actual: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reference);
		testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
		testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
		testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
		testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
	}
	// inclusive f32 endpoints and live equal-hit limits need the same acceptance
	for count in 1..=2
	{
		triangles: [2]physics.Triangle = {
			{{0, 0, 0}, {1, 0, 0}, {1, 0, 1}},
			{{0, -1e-8, 0}, {1, -1e-8, 0}, {1, -1e-8, 1}},
		};
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], count, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		ray: physics.Tree_Ray = {origin={0.75, 1, 0.25}, direction={0, -25, 0}, maximum_t=f32(1)/25 if count == 1 else 1};
		reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, reference.state, physics.Reference_State.Present);
		testing.expect_value(t, reference.child_index, i32(count-1));
		actual: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reference);
		testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
		testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
		testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
		testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
	}
	for scale in ([2]util.Vector3{{1, 1, 1}, {-2, 1, 0.5}})
	{
		for count in ([3]int{4, 8, 17})
		{
			triangles: [17]physics.Triangle;
			for &triangle in triangles
			{
				triangle = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}};
			}
			mesh: physics.Mesh;
			testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], count, scale, &pool), physics.Physics_Status.Ok);
			for recentered in 0..<2
			{
				if recentered == 1
				{
					testing.expect_value(t, physics.mesh_recenter(&mesh, {0.25, 0.5, 0.75}), physics.Physics_Status.Ok);
				}
				pose: physics.Rigid_Pose = {position={3, -2, 1}, orientation=util.quaternion_from_axis_angle({0, 0, 1}, 0.3)};
				local_origin: util.Vector3 = {0.5*scale.x, 4 if scale.x > 0 else -4, -0.5*scale.z};
				if recentered == 1
				{
					local_origin = util.vector3_subtract(local_origin, {0.25, 0.5, 0.75});
				}
				ray: physics.Tree_Ray = {origin=util.vector3_add(pose.position, util.quaternion_transform(local_origin, pose.orientation)), direction=util.quaternion_transform({0, -1 if scale.x > 0 else 1, 0}, pose.orientation), maximum_t=8};
				reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, pose, ray);
				testing.expect_value(t, reference.state, physics.Reference_State.Present);
				testing.expect_value(t, reference.child_index, i32(count-1));
				for sample in 0..<64
				{
					nonaxial: physics.Tree_Ray = {
						origin=util.vector3_add(pose.position, util.quaternion_transform({f32(sample%8)*0.5-2, 4 if scale.x > 0 else -4, f32(sample/8)*0.5-2}, pose.orientation)),
						direction=util.quaternion_transform({0.125, -1 if scale.x > 0 else 1, 0.0625}, pose.orientation), maximum_t=8,
					};
					actual: physics.Shape_Ray_Hit;
					status: physics.Physics_Status;
					actual, status = physics.mesh_ray_test(&mesh, pose, nonaxial);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					reference_bits_1: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, pose, nonaxial);
					testing.expect_value(t, actual, reference_bits_1);
					testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference_bits_1.t);
					testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference_bits_1.normal.x);
					testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference_bits_1.normal.y);
					testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference_bits_1.normal.z);
				}
				for endpoint in 0..<3
				{
					bounded: physics.Tree_Ray = ray;
					bounded.maximum_t = reference.t if endpoint == 1 else math.nextafter_f32(reference.t, 0) if endpoint == 2 else 8;
					actual: physics.Shape_Ray_Hit;
					status: physics.Physics_Status;
					actual, status = physics.mesh_ray_test(&mesh, pose, bounded);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					reference_bits_2: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, pose, bounded);
					testing.expect_value(t, actual, reference_bits_2);
					testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference_bits_2.t);
					testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference_bits_2.normal.x);
					testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference_bits_2.normal.y);
					testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference_bits_2.normal.z);
				}
			}
			testing.expect_value(t, physics.mesh_dispose(&mesh, &pool), physics.Physics_Status.Ok);
		}
	}
	// a non-dyadic recenter must still enclose the actual rounded triangle vertices
	{
		triangle: physics.Triangle = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}};
		mesh: physics.Mesh;
		testing.expect_value(t, physics.mesh_create(&mesh, &triangle, 1, {1, 1, 1}, &pool), physics.Physics_Status.Ok);
		defer physics.mesh_dispose(&mesh, &pool);
		testing.expect_value(t, physics.mesh_recenter(&mesh, {1e-8, 0, 0}), physics.Physics_Status.Ok);
		ray: physics.Tree_Ray = {origin={1, 1, 0.25}, direction={0, -1, 0}, maximum_t=1};
		reference: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, reference.state, physics.Reference_State.Present);
		actual: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, reference);
		testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference.t);
		testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference.normal.x);
		testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference.normal.y);
		testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference.normal.z);
	}
	triangles: [300]physics.Triangle;
	x: f32 = 1;
	for &triangle, index in triangles
	{
		triangle = {{x, -1, 1}, {x, 1, 1}, {x, 1, -1}};
		if index == 0
		{
			triangle = {{x, -1, -1}, {x, 1, -1}, {x, -1, 1}};
		}
		x *= 0.99;
	}
	mesh: physics.Mesh;
	testing.expect_value(t, physics.mesh_create(&mesh, &triangles[0], len(triangles), {1, 1, 1}, &pool), physics.Physics_Status.Ok);
	defer physics.mesh_dispose(&mesh, &pool);
	testing.expect_value(t, physics.tree_dispose(&mesh.tree), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_initialize(&mesh.tree, len(triangles), &pool), physics.Physics_Status.Ok);
	for index in 0..<len(triangles)
	{
		leaf: int;
		status: physics.Physics_Status;
		leaf, status = physics.tree_add_without_refinement(&mesh.tree, physics.tree_bounds_from_local(mesh.triangle_bounds.memory[index]));
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, leaf, index);
	}
	depth: int;
	node: int = physics.tree_leaf_node_index(mesh.tree.leaves.memory[len(triangles)-1]);
	for node >= 0
	{
		depth += 1;
		node = int(mesh.tree.metanodes.memory[node].parent);
	}
	testing.expect(t, depth > physics.TREE_TRAVERSAL_STACK_CAPACITY);
	ray: physics.Tree_Ray = {origin={0, -0.125, -0.125}, direction={1, 0, 0}, maximum_t=1};
	actual: physics.Shape_Ray_Hit;
	status: physics.Physics_Status;
	actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	reference_bits_3: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, actual, reference_bits_3);
	testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference_bits_3.t);
	testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference_bits_3.normal.x);
	testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference_bits_3.normal.y);
	testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference_bits_3.normal.z);
	testing.expect_value(t, actual.state, physics.Reference_State.Present);
	testing.expect_value(t, actual.t, f32(1));
	ray.direction = {25, 0, 0};
	ray.maximum_t = f32(1)/25;
	actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	reference_bits_4: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, actual, reference_bits_4);
	testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference_bits_4.t);
	testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference_bits_4.normal.x);
	testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference_bits_4.normal.y);
	testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference_bits_4.normal.z);
	ray.direction = {1, 0, 0};
	ray.maximum_t = 1;
	testing.expect_value(t, physics.mesh_recenter(&mesh, {1e-8, 0, 0}), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.tree_validate(&mesh.tree), physics.Physics_Status.Ok);
	actual, status = physics.mesh_ray_test(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	reference_bits_5: physics.Shape_Ray_Hit = resolution_mesh_scan(&mesh, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, actual, reference_bits_5);
	testing.expect_value(t, transmute(u32)actual.t, transmute(u32)reference_bits_5.t);
	testing.expect_value(t, transmute(u32)actual.normal.x, transmute(u32)reference_bits_5.normal.x);
	testing.expect_value(t, transmute(u32)actual.normal.y, transmute(u32)reference_bits_5.normal.y);
	testing.expect_value(t, transmute(u32)actual.normal.z, transmute(u32)reference_bits_5.normal.z);
	shapes: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&shapes);
	mesh_index: physics.Typed_Index;
	mesh_index, status = physics.shape_registry_add(&shapes, physics.MESH_TYPE_ID, &mesh);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	children: [2]physics.Compound_Child = {{local_orientation=util.quaternion_identity(), shape_index=mesh_index}, {local_orientation=util.quaternion_identity(), shape_index=mesh_index}};
	compound: physics.Compound;
	testing.expect_value(t, physics.compound_create(&compound, &children[0], len(children), &pool), physics.Physics_Status.Ok);
	defer physics.compound_dispose(&compound, &pool);
	compound_index: physics.Typed_Index;
	compound_index, status = physics.shape_registry_add(&shapes, physics.COMPOUND_TYPE_ID, &compound);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	actual, status = physics.shape_registry_ray_test(&shapes, compound_index, physics.rigid_pose_identity(), ray);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual.child_index, i32(1));
	hits: [4]physics.Ray_Query_Hit;
	collector: physics.Ray_Query_Collector = {hits={memory=&hits[0], length=4, id=-1}, mode=.All};
	testing.expect_value(t, physics.query_ray_shape(&shapes, {shape=compound_index, pose=physics.rigid_pose_identity(), target_id=7}, ray, &collector, &pool), physics.Physics_Status.Ok);
	testing.expect_value(t, collector.count, 2);
	for hit, index in hits[:collector.count]
	{
		testing.expect_value(t, hit.child_index, i32(index));
		testing.expect_value(t, hit.t, f32(1));
	}
}

Compound_Ray_Probe_Mode :: enum
{
	Mixed, Miss, Equal,
}

Compound_Ray_Probe_Observation :: struct
{
	pose: [7]u32,
	maximum_t: u32,
	child: int,
}

Compound_Ray_Probe_State :: struct
{
	observations: [34]Compound_Ray_Probe_Observation,
	count, fail_at: int,
	mode: Compound_Ray_Probe_Mode,
}

Compound_Ray_Probe :: struct
{
	sphere: physics.Sphere,
	state: ^Compound_Ray_Probe_State,
	child: int,
}

resolution_pose_bits :: proc "contextless" (pose: physics.Rigid_Pose) -> [7]u32
{
	return {
		transmute(u32)pose.position.x, transmute(u32)pose.position.y, transmute(u32)pose.position.z,
		transmute(u32)pose.orientation.x, transmute(u32)pose.orientation.y,
		transmute(u32)pose.orientation.z, transmute(u32)pose.orientation.w,
	};
}

resolution_compound_probe_ray :: proc "contextless" (
	shape: rawptr, pose: physics.Rigid_Pose, ray: physics.Tree_Ray,
	registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	probe: ^Compound_Ray_Probe = (^Compound_Ray_Probe)(shape);
	state: ^Compound_Ray_Probe_State = probe.state;
	index: int = state.count;
	state.observations[index] = {resolution_pose_bits(pose), transmute(u32)ray.maximum_t, probe.child};
	state.count += 1;
	if index == state.fail_at
	{
		return physics.shape_ray_miss(), .Invalid_Description;
	}
	if state.mode == .Miss || (state.mode == .Mixed && probe.child%3 == 1)
	{
		return physics.shape_ray_miss(), .Ok;
	}
	return {state=.Present, t=8 if state.mode == .Equal else 32-f32(probe.child/3)*4,
		normal={f32(probe.child+1), 2, 3}, child_index=73}, .Ok;
}

resolution_probe_composite_add :: proc(
	t: ^testing.T, registry: ^physics.Shape_Registry, children: []physics.Compound_Child, type_id: int,
) -> physics.Typed_Index
{
	index: physics.Typed_Index;
	status: physics.Physics_Status;
	if type_id == physics.COMPOUND_TYPE_ID
	{
		compound: physics.Compound;
		status = physics.compound_create(&compound, raw_data(children), len(children), registry.pool);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		if status != .Ok
		{
			return {};
		}
		index, status = physics.shape_registry_add(registry, type_id, &compound);
		if status != .Ok
		{
			_ = physics.compound_dispose(&compound, registry.pool);
		}
	}
	else
	{
		compound: physics.Big_Compound;
		status = physics.big_compound_create(&compound, raw_data(children), len(children), registry, registry.pool);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		if status != .Ok
		{
			return {};
		}
		index, status = physics.shape_registry_add(registry, type_id, &compound);
		if status != .Ok
		{
			_ = physics.big_compound_dispose(&compound, registry.pool);
		}
	}
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	return index;
}

@(test)
compound_and_big_compound_rays_preserve_exact_child_poses_order_limits_and_errors :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	registry: physics.Shape_Registry;
	testing.expect_value(t, physics.shape_registry_initialize(&registry, 32, &pool), physics.Physics_Status.Ok);
	defer physics.shape_registry_dispose(&registry);
	registration: physics.Shape_Type_Registration = registry.batches[physics.SPHERE_TYPE_ID].metadata;
	registration.size, registration.alignment = size_of(Compound_Ray_Probe), align_of(Compound_Ray_Probe);
	registration.ray = resolution_compound_probe_ray;
	custom_type: i32;
	status: physics.Physics_Status;
	custom_type, status = physics.shape_registry_register_custom(&registry, registration);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	state: Compound_Ray_Probe_State;
	negative_zero: f32 = transmute(f32)u32(0x80000000);
	subnormal: f32 = transmute(f32)u32(1);
	orientations: [6]util.Quaternion = {
		util.quaternion_identity(), {negative_zero, 0, negative_zero, 1}, {0.5, -0.5, 0.5, 0.5},
		{0.5, 0.5, 0.5, 0.500002}, {subnormal, -subnormal, 0, 1}, {0.4, 0.6, 0, 0.692820323},
	};
	positions: [6]util.Vector3 = {
		{}, {negative_zero, 0, negative_zero}, {1.25, -2.5, 3.75},
		{1048576, 0.03125, -1048576}, {subnormal, -subnormal, transmute(f32)u32(0x007fffff)},
		{16777216, -16777216, 1048576},
	};
	children: [17]physics.Compound_Child;
	for &child, index in children
	{
		probe: Compound_Ray_Probe = {sphere={0.5}, state=&state, child=index};
		child.shape_index, status = physics.shape_registry_add(&registry, int(custom_type), &probe);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		child.local_orientation = orientations[index%len(orientations)];
		child.local_position = positions[index%len(positions)];
	}
	for child_count in ([3]int{1, 8, 17})
	{
		for inner_type in ([2]int{physics.COMPOUND_TYPE_ID, physics.BIG_COMPOUND_TYPE_ID})
		{
			inner: physics.Typed_Index = resolution_probe_composite_add(t, &registry, children[:child_count], inner_type);
			outer_children: [2]physics.Compound_Child = {
				{local_orientation=orientations[2], local_position=positions[2], shape_index=inner},
				{local_orientation=orientations[4], local_position=positions[4], shape_index=inner},
			};
			outer_type: int = physics.BIG_COMPOUND_TYPE_ID if inner_type == physics.COMPOUND_TYPE_ID else physics.COMPOUND_TYPE_ID;
			outer: physics.Typed_Index = resolution_probe_composite_add(t, &registry, outer_children[:], outer_type);
			for target, route in ([2]physics.Typed_Index{inner, outer})
			{
				groups: int = 1 if route == 0 else 2;
				for orientation, pose_index in orientations
				{
					parent: physics.Rigid_Pose = {orientation=orientation, position=positions[pose_index]};
					testing.expect_value(t, physics.constraint_unit_quaternion(orientation), physics.Reference_State.Present);
					for mode in ([3]Compound_Ray_Probe_Mode{.Mixed, .Miss, .Equal})
					{
						for fail_at in ([4]int{-1, 0, groups*child_count/2, groups*child_count-1})
						{
							state = {mode=mode, fail_at=fail_at};
							ray: physics.Tree_Ray = {origin={2, 3, 4}, direction={0, -1, 0}, maximum_t=64};
							actual: physics.Shape_Ray_Hit;
							actual, status = physics.shape_registry_ray_test(&registry, target, parent, ray);
							expected: physics.Shape_Ray_Hit = physics.shape_ray_miss();
							maximum_t: f32 = ray.maximum_t;
							expected_calls: int;
							for group in 0..<groups
							{
								group_pose: physics.Rigid_Pose = parent;
								if route == 1
								{
									child: physics.Compound_Child = outer_children[group];
									group_pose = physics.rigid_pose_concatenate({orientation=child.local_orientation, position=child.local_position}, parent);
								}
								for child, index in children[:child_count]
								{
									if fail_at >= 0 && expected_calls > fail_at
									{
										break;
									}
									pose: physics.Rigid_Pose = physics.rigid_pose_concatenate({orientation=child.local_orientation, position=child.local_position}, group_pose);
									observation: Compound_Ray_Probe_Observation = state.observations[expected_calls];
									testing.expect_value(t, observation.pose, resolution_pose_bits(pose));
									testing.expect_value(t, observation.maximum_t, transmute(u32)maximum_t);
									testing.expect_value(t, observation.child, index);
									expected_calls += 1;
									if mode != .Miss && (mode != .Mixed || index%3 != 1)
									{
										maximum_t = 8 if mode == .Equal else 32-f32(index/3)*4;
										expected = {state=.Present, t=maximum_t, normal={f32(index+1), 2, 3}, child_index=i32(index) if route == 0 else i32(group)};
									}
								}
							}
							testing.expect_value(t, state.count, expected_calls);
							if fail_at >= 0
							{
								testing.expect_value(t, status, physics.Physics_Status.Invalid_Description);
								testing.expect_value(t, actual, physics.shape_ray_miss());
							}
							else
							{
								testing.expect_value(t, status, physics.Physics_Status.Ok);
								testing.expect_value(t, actual, expected);
							}
						}
					}
				}
			}
		}
	}
	triangle: physics.Triangle = {{-1048576, 0, 0}, {0, 0, 0}, {0, 0, 1}};
	triangle_index: physics.Typed_Index;
	triangle_index, status = physics.shape_registry_add(&registry, physics.TRIANGLE_TYPE_ID, &triangle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	triangle_children: [2]physics.Compound_Child;
	for &child in triangle_children
	{
		child = {local_orientation=util.quaternion_identity(), shape_index=triangle_index};
	}
	for type_id in ([2]int{physics.COMPOUND_TYPE_ID, physics.BIG_COMPOUND_TYPE_ID})
	{
		for count in 1..=2
		{
			index: physics.Typed_Index = resolution_probe_composite_add(t, &registry, triangle_children[:count], type_id);
			hit: physics.Shape_Ray_Hit;
			hit, status = physics.shape_registry_ray_test(&registry, index, physics.rigid_pose_identity(),
				{origin={0.03125, 1, 0.25}, direction={0, -1, 0}, maximum_t=1});
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, hit.state, physics.Reference_State.Present);
			testing.expect_value(t, hit.t, f32(1));
			testing.expect_value(t, hit.normal, util.Vector3{0, 1, 0});
			testing.expect_value(t, hit.child_index, i32(count-1));
		}
	}
}

hull_plane_scalar_reference :: proc "contextless" (
	hull: ^physics.Convex_Hull, pose: physics.Rigid_Pose, ray: physics.Tree_Ray,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	if physics.shape_ray_valid(ray) != .Ok || physics.convex_hull_validate(hull) != .Ok
	{
		return physics.shape_ray_miss(), .Invalid_Argument;
	}
	inverse: util.Quaternion = util.quaternion_conjugate(pose.orientation);
	origin: util.Vector3 = util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse);
	direction: util.Vector3 = util.quaternion_transform(ray.direction, inverse);
	entry: f32 = -f32(math.F32_MAX);
	exit: f32 = f32(math.F32_MAX);
	normal: util.Vector3;
	for index in 0 ..< int(hull.face_start_indices.length)
	{
		plane_normal: util.Vector3;
		offset: f32;
		plane_normal, offset = physics.convex_hull_get_face_plane(hull, index);
		numerator: f32 = offset-((plane_normal.x*origin.x+plane_normal.y*origin.y)+plane_normal.z*origin.z);
		denominator: f32 = (plane_normal.x*direction.x+plane_normal.y*direction.y)+plane_normal.z*direction.z;
		if abs(denominator) <= 1e-14
		{
			if numerator < 0
			{
				return physics.shape_ray_miss(), .Ok;
			}
			continue;
		}
		parameter: f32 = numerator/denominator;
		if denominator > 0
		{
			exit = min(exit, parameter);
		}
		else if parameter > entry
		{
			entry = parameter;
			normal = plane_normal;
		}
	}
	if exit < 0 || entry > exit
	{
		return physics.shape_ray_miss(), .Ok;
	}
	parameter: f32 = max(f32(0), entry);
	if parameter < 0 || parameter > ray.maximum_t
	{
		return physics.shape_ray_miss(), .Ok;
	}
	return {t=parameter, normal=util.quaternion_transform(normal, pose.orientation), child_index=0, state=.Present}, .Ok;
}

hull_plane_expect_exact :: proc(t: ^testing.T, hull: ^physics.Convex_Hull, pose: physics.Rigid_Pose, ray: physics.Tree_Ray)
{
	actual, expected: physics.Shape_Ray_Hit;
	actual_status, expected_status: physics.Physics_Status;
	actual, actual_status = physics.convex_hull_ray_test(hull, pose, ray);
	expected, expected_status = hull_plane_scalar_reference(hull, pose, ray);
	testing.expect_value(t, actual_status, expected_status);
	testing.expect_value(t, actual.state, expected.state);
	testing.expect_value(t, actual.child_index, expected.child_index);
	testing.expect_value(t, transmute(u32)actual.t, transmute(u32)expected.t);
	testing.expect_value(t, transmute([3]u32)actual.normal, transmute([3]u32)expected.normal);
}

hull_plane_test_prism :: proc(t: ^testing.T, pool: ^util.Buffer_Pool, sides: int) -> physics.Convex_Hull
{
	points: [20]util.Vector3;
	starts: [12]i32;
	indices: [60]i32;
	for index in 0 ..< sides
	{
		angle: f32 = f32(2*math.PI)*f32(index)/f32(sides);
		points[index] = {math.cos(angle), -1, math.sin(angle)};
		points[sides+index] = {points[index].x, 1, points[index].z};
		indices[index] = i32(index);
		indices[sides+index] = i32(2*sides-1-index);
	}
	starts[0], starts[1] = 0, i32(sides);
	for index in 0 ..< sides
	{
		start: int = 2*sides+4*index;
		next: int = (index+1)%sides;
		starts[index+2] = i32(start);
		indices[start], indices[start+1], indices[start+2], indices[start+3] = i32(index), i32(sides+index), i32(sides+next), i32(next);
	}
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(&hull, &points[0], 2*sides, &starts[0], sides+2, &indices[0], 6*sides, pool), physics.Physics_Status.Ok);
	return hull;
}

@(test)
hull_plane_bundles_preserve_scalar_boundaries_and_face_ties :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	prepare_pool(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	poses: [3]physics.Rigid_Pose = {physics.rigid_pose_identity(), {position={3, -1.25, 2.5}, orientation=util.quaternion_from_axis_angle({0.36, 0.48, 0.8}, 0.37)}, {position={-4, 2, 1}, orientation={0, 0, 0, 1.00001}}};
	negative_zero: f32 = transmute(f32)u32(0x80000000);
	rays: [11]physics.Tree_Ray = {
		{origin={0, 3, 0}, direction={0, -1, 0}, maximum_t=8},
		{origin={0, 0, 0}, direction={1, 0, 0}, maximum_t=8},
		{origin={4, 0, 0}, direction={-1, 0, 0}, maximum_t=8},
		{origin={4, 3, 0}, direction={0, -1, 0}, maximum_t=8},
		{origin={0, 1, 0}, direction={1, negative_zero, 0}, maximum_t=8},
		{origin={0, 2, 0}, direction={1, 0, 0}, maximum_t=8},
		{origin={-1, 3, 0.5}, direction={0.31, -1, 0.21}, maximum_t=8},
		{origin={0, 3, 0}, direction={0, -1, 0}, maximum_t=2},
		{origin={0, 3, 0}, direction={negative_zero, -1, 0}, maximum_t=math.nextafter_f32(2, 0)},
		{origin={0, 3, 0}, direction={0, -1, 0}, maximum_t=math.nextafter_f32(2, math.F32_MAX)},
		{origin={0, 3, 0}, direction={0, -1, 0}, maximum_t=-1},
	};
	for sides in 3 ..< 11
	{
		hull: physics.Convex_Hull = hull_plane_test_prism(t, &pool, sides);
		for pose in poses
		{
			for local in rays
			{
				ray: physics.Tree_Ray = local;
				ray.origin = util.vector3_add(pose.position, util.quaternion_transform(local.origin, pose.orientation));
				ray.direction = util.quaternion_transform(local.direction, pose.orientation);
				hull_plane_expect_exact(t, &hull, pose, ray);
			}
		}
		testing.expect_value(t, physics.convex_hull_dispose(&hull, &pool), physics.Physics_Status.Ok);
	}
	// triangulated cube faces place equal entry planes in different bundles
	for permutation in 0 ..< 2
	{
		points: [8]util.Vector3;
		quad_starts: [6]i32;
		quads: [24]i32;
		points, quad_starts, quads = cube_hull_data();
		starts: [12]i32;
		indices: [36]i32;
		order: [6]int = {0, 1, 2, 3, 4, 5};
		if permutation == 1
		{
			order = {4, 0, 1, 3, 2, 5};
		}
		for quad, index in order
		{
			base: int = int(quad_starts[quad]);
			starts[2*index], starts[2*index+1] = i32(6*index), i32(6*index+3);
			indices[6*index], indices[6*index+1], indices[6*index+2] = quads[base], quads[base+1], quads[base+2];
			indices[6*index+3], indices[6*index+4], indices[6*index+5] = quads[base], quads[base+2], quads[base+3];
		}
		hull: physics.Convex_Hull;
		testing.expect_value(t, physics.convex_hull_create(&hull, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &pool), physics.Physics_Status.Ok);
		ray: physics.Tree_Ray = {origin={-2, -2, 0}, direction={1, 1, 0}, maximum_t=1};
		hull_plane_expect_exact(t, &hull, physics.rigid_pose_identity(), ray);
		hit: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		hit, status = physics.convex_hull_ray_test(&hull, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, hit.state, physics.Reference_State.Present);
		testing.expect_value(t, hit.t, f32(1));
		testing.expect_value(t, hit.normal, util.Vector3{-1, 0, 0} if permutation == 0 else util.Vector3{0, -1, 0});
		ray.maximum_t = math.nextafter_f32(1, 0);
		hull_plane_expect_exact(t, &hull, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, physics.convex_hull_dispose(&hull, &pool), physics.Physics_Status.Ok);
	}
	// the dominant X direction keeps the ray valid around the Y parallel threshold
	points: [8]util.Vector3;
	starts: [6]i32;
	indices: [24]i32;
	points, starts, indices = cube_hull_data();
	for &point in points
	{
		point.x *= 268435456;
	}
	hull: physics.Convex_Hull;
	testing.expect_value(t, physics.convex_hull_create(&hull, &points[0], len(points), &starts[0], len(starts), &indices[0], len(indices), &pool), physics.Physics_Status.Ok);
	for epsilon, index in ([3]f32{math.nextafter_f32(1e-14, 0), 1e-14, math.nextafter_f32(1e-14, math.F32_MAX)})
	{
		ray: physics.Tree_Ray = {origin={0, math.nextafter_f32(1, math.F32_MAX), 0}, direction={1, -epsilon, 0}, maximum_t=1e8};
		hull_plane_expect_exact(t, &hull, physics.rigid_pose_identity(), ray);
		hit: physics.Shape_Ray_Hit;
		status: physics.Physics_Status;
		hit, status = physics.convex_hull_ray_test(&hull, physics.rigid_pose_identity(), ray);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, hit.state, physics.Reference_State.Present if index == 2 else physics.Reference_State.Missing);
	}
	testing.expect_value(t, physics.convex_hull_dispose(&hull, &pool), physics.Physics_Status.Ok);
}
