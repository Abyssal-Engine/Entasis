package public_api_tests

import "core:testing"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Sphere :: struct
{
	radius: f32,
}

custom_sphere_value :: #force_inline proc "contextless" (shape: rawptr) -> physics.Sphere
{
	return {radius=(^Custom_Sphere)(shape).radius};
}

custom_sphere_bounds :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^physics.Shape_Registry,
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	_ = registry;
	value := custom_sphere_value(shape);
	if physics.sphere_validate(value) != .Ok
	{
		return {}, .Invalid_Description;
	}
	return physics.sphere_bounds(value, orientation), .Ok;
}

custom_sphere_inertia :: proc "contextless" (
	shape: rawptr, registry: ^physics.Shape_Registry, mass: f32,
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_inertia(custom_sphere_value(shape), mass);
}

custom_sphere_ray :: proc "contextless" (
	shape: rawptr, pose_value: physics.Rigid_Pose, ray_value: physics.Tree_Ray,
	registry: ^physics.Shape_Registry,
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_ray_test(custom_sphere_value(shape), pose_value, ray_value);
}

custom_sphere_support :: proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^physics.Shape_Registry,
) -> (util.Vector3, physics.Physics_Status)
{
	_ = registry;
	return physics.sphere_support(custom_sphere_value(shape), direction);
}

// custom IDs do not populate built-in packed lanes. this fixture's radius
// payload is explicitly gathered before calling the existing sphere SIMD kernel
custom_sphere_pair_wide :: proc "contextless" (bundle:^physics.Collision_Convex_Wide_Bundle, registry:^physics.Shape_Registry) -> (physics.Collision_Wide_Manifold_Result, physics.Physics_Status)
{
	_=registry;
	a, b:physics.Sphere_Wide;
	for lane in 0..<util.PRODUCTION_LANE_COUNT
	{
		physics.sphere_wide_write_slot_trusted(&a, lane, custom_sphere_value(bundle.shape_a[lane]));
		physics.sphere_wide_write_slot_trusted(&b, lane, custom_sphere_value(bundle.shape_b[lane]));
	}
	wide, status:=physics.sphere_pair_test_wide(a, b, bundle.speculative_margin, bundle.offset_b, bundle.count);
	return {kind=.One_Contact, one=wide}, status;
}

custom_sphere_pair :: proc "contextless" (
	shape_a, shape_b: rawptr,
	pose_a, pose_b: physics.Rigid_Pose,
	speculative_margin: f32,
	shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	a := custom_sphere_value(shape_a);
	b := custom_sphere_value(shape_b);
	return physics.sphere_pair_test(&a, &b, pose_a, pose_b, speculative_margin, shapes);
}

custom_sphere_builtin_pair :: proc "contextless" (
	shape_a, shape_b: rawptr,
	pose_a, pose_b: physics.Rigid_Pose,
	speculative_margin: f32,
	shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	a := custom_sphere_value(shape_a);
	return physics.sphere_pair_test(&a, shape_b, pose_a, pose_b, speculative_margin, shapes);
}

custom_sweep_radius :: #force_inline proc "contextless" (shape: rawptr, type_id: int) -> f32
{
	if type_id == physics.SPHERE_TYPE_ID
	{
		return (^physics.Sphere)(shape).radius;
	}
	return (^Custom_Sphere)(shape).radius;
}

custom_sphere_sweep :: proc "contextless" (
	shape_a, shape_b: rawptr,
	type_a, type_b: int,
	pose_a, pose_b: physics.Rigid_Pose,
	velocity_a, velocity_b: physics.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^physics.Shape_Registry,
	collision_tasks: ^physics.Collision_Task_Registry,
	filter: physics.Collision_Child_Filter_Proc,
	user_context: rawptr,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	_, _, _, _, _ = minimum_progression, convergence_threshold, maximum_iteration_count, shapes, collision_tasks;
	_, _ = filter, user_context;
	radius := custom_sweep_radius(shape_a, type_a) + custom_sweep_radius(shape_b, type_b);
	position := util.vector3_subtract(pose_a.position, pose_b.position);
	velocity_value := util.vector3_subtract(velocity_a.linear, velocity_b.linear);
	c := util.vector3_dot(position, position) - radius * radius;
	t := f32(0);
	if c > 0
	{
		a := util.vector3_dot(velocity_value, velocity_value);
		if a <= 1e-20
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
		b := util.vector3_dot(position, velocity_value);
		discriminant := b * b - a * c;
		if discriminant < 0
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
		t = (-b - math.sqrt(discriminant)) / a;
		if t < 0 || t > maximum_t
		{
			return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
		}
	}
	center_a := util.vector3_add(pose_a.position, util.vector3_scale(velocity_a.linear, t));
	center_b := util.vector3_add(pose_b.position, util.vector3_scale(velocity_b.linear, t));
	offset := util.vector3_subtract(center_a, center_b);
	distance := util.vector3_length(offset);
	normal := util.Vector3{1, 0, 0};
	if distance > 1e-20
	{
		normal = util.vector3_scale(offset, 1 / distance);
	}
	return {
		state=.Hit, t0=t, t1=t,
		location=util.vector3_add(center_b, util.vector3_scale(normal, custom_sweep_radius(shape_b, type_b))),
		normal=normal, child_a=-1, child_b=-1,
	}, .Ok;
}

custom_sphere_sweep_child :: proc "contextless" (
	shape_a, shape_b: rawptr,
	type_a, type_b: int,
	parent_pose_a, parent_pose_b: physics.Rigid_Pose,
	local_pose_a, local_pose_b: physics.Rigid_Pose,
	velocity_a, velocity_b: physics.Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^physics.Shape_Registry,
	collision_tasks: ^physics.Collision_Task_Registry,
) -> (physics.Sweep_Result, physics.Physics_Status)
{
	return custom_sphere_sweep(
		shape_a, shape_b, type_a, type_b,
		physics.rigid_pose_concatenate(local_pose_a, parent_pose_a),
		physics.rigid_pose_concatenate(local_pose_b, parent_pose_b),
		velocity_a, velocity_b, maximum_t, minimum_progression,
		convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, nil, nil,
	);
}

register_custom_sphere :: proc(
	t: ^testing.T, world: ^entasis.World,
) -> (entasis.Shape_Type_ID, bool)
{
	next_id, next_status := entasis.custom_shape_next_type_id(world);
	if !testing.expect_value(t, next_status, entasis.Status.Ok)
	{
		return {}, false;
	}
	registration := entasis.custom_shape_registration(
		Custom_Sphere,
		.Convex,
		custom_sphere_bounds,
		custom_sphere_inertia,
		custom_sphere_ray,
		custom_sphere_support,
		expected_type_id=next_id,
	);
	type_id, status := entasis.custom_shape_register(world, registration);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return {}, false;
	}
	if !testing.expect_value(t, type_id, next_id)
	{
		return {}, false;
	}
	_, status = entasis.collision_task_register(
		world,
		entasis.collision_task_convex(
			type_id,
			type_id,
			16,
			.Sphere,
			custom_sphere_pair,
			custom_sphere_pair_wide
		),
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return {}, false;
	}
	_, status = entasis.collision_task_register(
		world,
		entasis.collision_task_convex(
			type_id,
			entasis.SHAPE_TYPE_SPHERE,
			16,
			.Sphere,
			custom_sphere_builtin_pair,
			custom_sphere_pair_wide
		),
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return {}, false;
	}
	_, status = entasis.sweep_task_register(
		world,
		entasis.sweep_task_registration(type_id, type_id, custom_sphere_sweep, custom_sphere_sweep_child)
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return {}, false;
	}
	_, status = entasis.sweep_task_register(
		world,
		entasis.sweep_task_registration(
			entasis.SHAPE_TYPE_SPHERE,
			type_id,
			custom_sphere_sweep,
			custom_sphere_sweep_child
		)
	);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return {}, false;
	}
	return type_id, true;
}

@(test)
custom_shape_registration_ray_collision_and_sweep_use_registered_tasks :: proc(t: ^testing.T)
{
	description := small_world_description();
	description.gravity = {};
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	type_id, ok := register_custom_sphere(t, &world);
	if !ok
	{
		return;
	}
	custom_shape := Custom_Sphere{radius=1};
	custom_handle, custom_status := entasis.custom_shape_add(&world, type_id, &custom_shape);
	if !testing.expect_value(t, custom_status, entasis.Status.Ok)
	{
		return;
	}
	_, static_status := entasis.static_add(
		&world, entasis.static_body(custom_handle, entasis.pose({0, 0, 0})), .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return;
	}
	hit, ray_status := entasis.ray_cast_closest(
		&world, entasis.ray({-4, 0, 0}, {1, 0, 0}, 10),
	);
	if !testing.expect_value(t, ray_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, hit.t > 2.9 && hit.t < 3.1);
	built_in_handle, sphere_status := entasis.shape_add(&world, entasis.sphere(0.5));
	if !testing.expect_value(t, sphere_status, entasis.Status.Ok)
	{
		return;
	}
	sweep_hit, sweep_status := entasis.sweep_closest(
		&world,
		built_in_handle,
		entasis.pose({-4, 0, 0}),
		entasis.velocity({1, 0, 0}),
		10,
	);
	if !testing.expect_value(t, sweep_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect(t, sweep_hit.sweep.t0 > 2.4 && sweep_hit.sweep.t0 < 2.6);
	hits: [4]entasis.Overlap_Hit;
	overlap_count, overlap_status := entasis.overlap_all(
		&world, built_in_handle, entasis.pose({0, 0.5, 0}), hits[:],
	);
	testing.expect_value(t, overlap_status, entasis.Status.Ok);
	testing.expect_value(t, overlap_count, 1);
}

@(test)
custom_shape_registration_rejects_wrong_order_and_late_registration :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	next_id, status := entasis.custom_shape_next_type_id(&world);
	if !testing.expect_value(t, status, entasis.Status.Ok)
	{
		return;
	}
	wrong := entasis.custom_shape_registration(
		Custom_Sphere, .Convex, custom_sphere_bounds, custom_sphere_inertia,
		custom_sphere_ray, custom_sphere_support,
		expected_type_id=entasis.Shape_Type_ID(int(next_id) + 1),
	);
	_, status = entasis.custom_shape_register(&world, wrong);
	testing.expect_value(t, status, entasis.Status.Invalid_Description);
	if !testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok)
	{
		return;
	}
	valid := entasis.custom_shape_registration(
		Custom_Sphere, .Convex, custom_sphere_bounds, custom_sphere_inertia,
		custom_sphere_ray, custom_sphere_support,
	);
	_, status = entasis.custom_shape_register(&world, valid);
	testing.expect_value(t, status, entasis.Status.Invalid_Argument);
}

@(test)
custom_shape_registration_reports_fixed_registry_capacity :: proc(t: ^testing.T)
{
	world: entasis.World;
	if !testing.expect_value(t, entasis.world_init(&world, small_world_description()), entasis.Status.Ok)
	{
		return;
	}
	defer entasis.world_destroy(&world);
	for expected in entasis.BUILT_IN_SHAPE_TYPE_COUNT ..< entasis.MAXIMUM_SHAPE_TYPE_COUNT
	{
		next_id, next_status := entasis.custom_shape_next_type_id(&world);
		if !testing.expect_value(t, next_status, entasis.Status.Ok)
		{
			return;
		}
		if !testing.expect_value(t, int(next_id), expected)
		{
			return;
		}
		registration := entasis.custom_shape_registration(
			Custom_Sphere, .Convex,
			custom_sphere_bounds, custom_sphere_inertia,
			custom_sphere_ray, custom_sphere_support,
			expected_type_id=next_id,
		);
		registered, register_status := entasis.custom_shape_register(&world, registration);
		if !testing.expect_value(t, register_status, entasis.Status.Ok)
		{
			return;
		}
		if !testing.expect_value(t, registered, next_id)
		{
			return;
		}
	}
	next_id, next_status := entasis.custom_shape_next_type_id(&world);
	testing.expect_value(t, next_status, entasis.Status.Capacity_Missing);
	testing.expect_value(t, next_id, entasis.SHAPE_TYPE_INVALID);
	registration := entasis.custom_shape_registration(
		Custom_Sphere, .Convex,
		custom_sphere_bounds, custom_sphere_inertia,
		custom_sphere_ray, custom_sphere_support,
	);
	_, register_status := entasis.custom_shape_register(&world, registration);
	testing.expect_value(t, register_status, entasis.Status.Capacity_Missing);
}
