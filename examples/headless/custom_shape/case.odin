package custom_shape

import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Custom_Sphere :: struct
{
	radius: f32,
}

value :: #force_inline proc "contextless" (shape: rawptr) -> physics.Sphere
{
	return {radius=(^Custom_Sphere)(shape).radius};
}
bounds :: proc "contextless" (
	shape: rawptr,
	q: util.Quaternion,
	_: ^physics.Shape_Registry
) -> (physics.Shape_Bounds, physics.Physics_Status)
{
	v := value(shape);
	if physics.sphere_validate(v) != .Ok
	{
		return {}, .Invalid_Description;
	}
	return physics.sphere_bounds(v, q), .Ok;
}
inertia :: proc "contextless" (
	shape: rawptr,
	_: ^physics.Shape_Registry,
	mass: f32
) -> (physics.Body_Inertia, physics.Physics_Status)
{
	return physics.sphere_inertia(value(shape), mass);
}
ray_test :: proc "contextless" (
	shape: rawptr,
	pose: physics.Rigid_Pose,
	ray: physics.Tree_Ray,
	_: ^physics.Shape_Registry
) -> (physics.Shape_Ray_Hit, physics.Physics_Status)
{
	return physics.sphere_ray_test(value(shape), pose, ray);
}
support :: proc "contextless" (
	shape: rawptr,
	d: util.Vector3,
	_: ^physics.Shape_Registry
) -> (util.Vector3, physics.Physics_Status)
{
	return physics.sphere_support(value(shape), d);
}
pair :: proc "contextless" (
	a,
	b: rawptr,
	pa,
	pb: physics.Rigid_Pose,
	margin: f32,
	shapes: ^physics.Shape_Registry
) -> (physics.Convex_Contact_Manifold, physics.Physics_Status)
{
	sa, sb := value(a), value(b);
	return physics.sphere_pair_test(&sa, &sb, pa, pb, margin, shapes);
}

STEPS :: 1;
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	shape: entasis.Shape_Handle,
	shape_value: Custom_Sphere,
	ray: entasis.Ray,
	hit: entasis.Ray_Hit,
	tick: int,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	type_id: entasis.Shape_Type_ID;
	type_id, status = entasis.custom_shape_next_type_id(&c.world);
	if status != .Ok
	{
		return status;
	}
	registration: entasis.Custom_Shape_Registration = entasis.custom_shape_registration(
		Custom_Sphere, .Convex, bounds, inertia, ray_test, support, expected_type_id=type_id);
	_, status = entasis.custom_shape_register(&c.world, registration);
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.collision_task_register(&c.world, entasis.collision_task_convex(
		type_id, type_id, 16, .Sphere, pair, physics.collision_task_execute_sphere_pair_wide));
	if status != .Ok
	{
		return status;
	}
	c.shape_value = {radius=1};
	c.shape, status = entasis.custom_shape_add(&c.world, type_id, &c.shape_value);
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(c.shape, entasis.pose()), .None);
	c.ray = entasis.ray({-3, 0, 0}, {1, 0, 0}, 6);
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status;
	c.hit, status = entasis.ray_cast_closest(&c.world, c.ray);
	if status == .Ok
	{
		c.tick += 1;
	}
	return status;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
