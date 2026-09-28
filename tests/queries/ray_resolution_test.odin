package queries_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

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
