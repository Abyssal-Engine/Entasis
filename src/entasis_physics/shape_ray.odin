// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

Shape_Ray_Hit :: struct
{
	t:           f32,
	normal:      util.Vector3,
	child_index: i32,
	state:       Reference_State,
}

shape_ray_miss :: proc "contextless" () -> Shape_Ray_Hit
{
	return {child_index=-1};
}

shape_ray_valid :: proc "contextless" (ray: Tree_Ray) -> Physics_Status
{
	if ray.maximum_t < 0 || util.vector3_length_squared(ray.direction) <= 1e-20
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

shape_ray_finish :: proc "contextless" (
	t: f32,
	normal: util.Vector3,
	ray: Tree_Ray,
	child_index: i32 = 0
) -> Shape_Ray_Hit
{
	if t < 0 || t > ray.maximum_t
	{
		return shape_ray_miss();
	}
	return {t=t, normal=normal, child_index=child_index, state=.Present};
}

sphere_ray_test :: proc "contextless" (
	sphere: Sphere,
	pose: Rigid_Pose,
	ray: Tree_Ray
) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || sphere_validate(sphere) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	direction_length := util.vector3_length(ray.direction);
	inverse_direction_length := 1 / direction_length;
	d := util.vector3_scale(ray.direction, inverse_direction_length);
	o := util.vector3_subtract(ray.origin, pose.position);
	t_offset := max(f32(0), -util.vector3_dot(o, d) - sphere.radius);
	o = util.vector3_add(o, util.vector3_scale(d, t_offset));
	b := util.vector3_dot(o, d);
	c := util.vector3_dot(o, o) - sphere.radius * sphere.radius;
	if b > 0 && c > 0
	{
		return shape_ray_miss(), .Ok;
	}
	discriminant := b*b - c;
	if discriminant < 0
	{
		return shape_ray_miss(), .Ok;
	}
	local_t := max(-b - math.sqrt(discriminant), -t_offset);
	normal := util.vector3_scale(util.vector3_add(o, util.vector3_scale(d, local_t)), 1 / sphere.radius);
	return shape_ray_finish((local_t + t_offset) * inverse_direction_length, normal, ray), .Ok;
}

capsule_ray_test :: proc "contextless" (
	capsule: Capsule,
	pose: Rigid_Pose,
	ray: Tree_Ray
) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || capsule_validate(capsule) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	direction_length := util.vector3_length(d);
	inverse_direction_length := 1 / direction_length;
	d = util.vector3_scale(d, inverse_direction_length);
	t_offset := max(f32(0), -util.vector3_dot(o, d) - (capsule.half_length + capsule.radius));
	o = util.vector3_add(o, util.vector3_scale(d, t_offset));
	o_horizontal := util.Vector3{o.x, 0, o.z};
	d_horizontal := util.Vector3{d.x, 0, d.z};
	a := util.vector3_dot(d_horizontal, d_horizontal);
	b := util.vector3_dot(o_horizontal, d_horizontal);
	radius_squared := capsule.radius * capsule.radius;
	c := util.vector3_dot(o_horizontal, o_horizontal) - radius_squared;
	if b > 0 && c > 0
	{
		return shape_ray_miss(), .Ok;
	}
	sphere_y: f32;
	if a > 1e-8
	{
		discriminant := b*b - a*c;
		if discriminant < 0
		{
			return shape_ray_miss(), .Ok;
		}
		local_t := max((-b - math.sqrt(discriminant)) / a, -t_offset);
		hit := util.vector3_add(o, util.vector3_scale(d, local_t));
		if hit.y < -capsule.half_length
		{
			sphere_y = -capsule.half_length;
		}
		else if hit.y > capsule.half_length
		{
			sphere_y = capsule.half_length;
		}
		else
		{
			local_normal := util.Vector3{hit.x / capsule.radius, 0, hit.z / capsule.radius};
			normal := util.quaternion_transform(local_normal, pose.orientation);
			return shape_ray_finish((local_t + t_offset) * inverse_direction_length, normal, ray), .Ok;
		}
	}
	else
	{
		if d.y > 0
		{
			sphere_y = max(min(capsule.half_length, o.y), -capsule.half_length);
		}
		else
		{
			sphere_y = min(max(-capsule.half_length, o.y), capsule.half_length);
		}
	}
	o_sphere := util.vector3_subtract(o, {0, sphere_y, 0});
	cap_b := util.vector3_dot(o_sphere, d);
	cap_c := util.vector3_dot(o_sphere, o_sphere) - radius_squared;
	if cap_b > 0 && cap_c > 0
	{
		return shape_ray_miss(), .Ok;
	}
	discriminant := cap_b*cap_b - cap_c;
	if discriminant < 0
	{
		return shape_ray_miss(), .Ok;
	}
	local_t := max(-cap_b - math.sqrt(discriminant), -t_offset);
	local_normal := util.vector3_scale(util.vector3_add(o_sphere, util.vector3_scale(d, local_t)), 1 / capsule.radius);
	normal := util.quaternion_transform(local_normal, pose.orientation);
	return shape_ray_finish((local_t + t_offset) * inverse_direction_length, normal, ray), .Ok;
}

box_ray_test :: proc "contextless" (box: Box, pose: Rigid_Pose, ray: Tree_Ray) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || box_validate(box) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	half_extent := util.Vector3{box.half_width, box.half_height, box.half_length};
	latest_entry := -f32(math.F32_MAX);
	earliest_exit := f32(math.F32_MAX);
	entry_axis := 0;
	entry_sign: f32 = -1;
	for axis in 0 ..< 3
	{
		origin: f32;
		direction: f32;
		extent: f32;
		if axis == 0
		{
			origin=o.x;
			direction=d.x;
			extent=half_extent.x;
		}
		else if axis == 1
		{
			origin=o.y;
			direction=d.y;
			extent=half_extent.y;
		}
		else
		{
			origin=o.z;
			direction=d.z;
			extent=half_extent.z;
		}
		if abs(direction) <= 1e-15
		{
			if origin < -extent || origin > extent
			{
				return shape_ray_miss(), .Ok;
			}
			continue;
		}
		entry := (-extent - origin) / direction;
		exit := (extent - origin) / direction;
		sign: f32 = -1;
		if entry > exit
		{
			entry, exit = exit, entry;
			sign = 1;
		}
		if entry > latest_entry
		{
			latest_entry=entry;
			entry_axis=axis;
			entry_sign=sign;
		}
		earliest_exit = min(earliest_exit, exit);
	}
	if earliest_exit < 0 || earliest_exit < latest_entry
	{
		return shape_ray_miss(), .Ok;
	}
	local_normal: util.Vector3;
	if entry_axis == 0
	{
		local_normal.x = entry_sign;
	}
	else if entry_axis == 1
	{
		local_normal.y = entry_sign;
	}
	else
	{
		local_normal.z = entry_sign;
	}
	if latest_entry < 0
	{
		latest_entry = 0;
		if util.vector3_dot(local_normal, o) < 0
		{
			local_normal = util.vector3_negate(local_normal);
		}
	}
	normal := util.quaternion_transform(local_normal, pose.orientation);
	return shape_ray_finish(latest_entry, normal, ray), .Ok;
}

triangle_ray_test_local :: proc "contextless" (
	a, b, c, origin, direction: util.Vector3,
) -> (f32, util.Vector3, Reference_State)
{
	ab := util.vector3_subtract(b, a);
	ac := util.vector3_subtract(c, a);
	normal := util.vector3_cross(ac, ab);
	dn := -util.vector3_dot(direction, normal);
	if dn <= 0
	{
		return 0, {}, .Missing;
	}
	ao := util.vector3_subtract(origin, a);
	t := util.vector3_dot(ao, normal);
	if t < 0
	{
		return 0, {}, .Missing;
	}
	ao_cross_d := util.vector3_cross(ao, direction);
	v := -util.vector3_dot(ac, ao_cross_d);
	if v < 0 || v > dn
	{
		return 0, {}, .Missing;
	}
	w := util.vector3_dot(ab, ao_cross_d);
	if w < 0 || v + w > dn
	{
		return 0, {}, .Missing;
	}
	normal_length := util.vector3_length(normal);
	if normal_length <= 1e-20
	{
		return 0, {}, .Missing;
	}
	return t / dn, util.vector3_scale(normal, 1 / normal_length), .Present;
}

triangle_ray_test :: proc "contextless" (
	triangle: Triangle,
	pose: Rigid_Pose,
	ray: Tree_Ray
) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || triangle_validate(triangle) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	t, local_normal, state := triangle_ray_test_local(triangle.a, triangle.b, triangle.c, o, d);
	if state != .Present
	{
		return shape_ray_miss(), .Ok;
	}
	normal := util.quaternion_transform(local_normal, pose.orientation);
	return shape_ray_finish(t, normal, ray), .Ok;
}

cylinder_ray_test :: proc "contextless" (
	cylinder: Cylinder,
	pose: Rigid_Pose,
	ray: Tree_Ray
) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || cylinder_validate(cylinder) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	direction_length := util.vector3_length(d);
	inverse_direction_length := 1 / direction_length;
	d = util.vector3_scale(d, inverse_direction_length);
	t_offset := max(f32(0), -util.vector3_dot(o, d) - (cylinder.half_length + cylinder.radius));
	o = util.vector3_add(o, util.vector3_scale(d, t_offset));
	o_horizontal := util.Vector3{o.x, 0, o.z};
	d_horizontal := util.Vector3{d.x, 0, d.z};
	a := util.vector3_dot(d_horizontal, d_horizontal);
	b := util.vector3_dot(o_horizontal, d_horizontal);
	radius_squared := cylinder.radius * cylinder.radius;
	c := util.vector3_dot(o_horizontal, o_horizontal) - radius_squared;
	if b > 0 && c > 0
	{
		return shape_ray_miss(), .Ok;
	}
	disc_y: f32;
	if a > 1e-8
	{
		discriminant := b*b - a*c;
		if discriminant < 0
		{
			return shape_ray_miss(), .Ok;
		}
		local_t := max((-b - math.sqrt(discriminant)) / a, -t_offset);
		hit := util.vector3_add(o, util.vector3_scale(d, local_t));
		if hit.y < -cylinder.half_length
		{
			disc_y = -cylinder.half_length;
		}
		else if hit.y > cylinder.half_length
		{
			disc_y = cylinder.half_length;
		}
		else
		{
			local_normal := util.Vector3{hit.x / cylinder.radius, 0, hit.z / cylinder.radius};
			normal := util.quaternion_transform(local_normal, pose.orientation);
			return shape_ray_finish((local_t + t_offset) * inverse_direction_length, normal, ray), .Ok;
		}
	}
	else
	{
		disc_y = cylinder.half_length;
		if d.y > 0
		{
			disc_y = -cylinder.half_length;
		}
		if abs(o.y) < abs(disc_y)
		{
			disc_y = o.y;
		}
	}
	if abs(o.y) > cylinder.half_length && o.y*d.y >= 0 || abs(d.y) <= 1e-20
	{
		return shape_ray_miss(), .Ok;
	}
	local_t := (disc_y - o.y) / d.y;
	hit := util.vector3_add(o, util.vector3_scale(d, local_t));
	if hit.x*hit.x + hit.z*hit.z > radius_squared
	{
		return shape_ray_miss(), .Ok;
	}
	local_normal := util.Vector3{0, 1, 0};
	if d.y > 0
	{
		local_normal.y = -1;
	}
	normal := util.quaternion_transform(local_normal, pose.orientation);
	return shape_ray_finish((local_t + t_offset) * inverse_direction_length, normal, ray), .Ok;
}

convex_hull_ray_test :: proc "contextless" (
	hull: ^Convex_Hull,
	pose: Rigid_Pose,
	ray: Tree_Ray
) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || convex_hull_validate(hull) != .Ok
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	latest_entry := -f32(math.F32_MAX);
	earliest_exit := f32(math.F32_MAX);
	entry_normal: util.Vector3;
	for face_index in 0 ..< int(hull.face_start_indices.length)
	{
		normal, offset := convex_hull_get_face_plane(hull, face_index);
		numerator := offset - util.vector3_dot(normal, o);
		denominator := util.vector3_dot(normal, d);
		if abs(denominator) <= 1e-14
		{
			if numerator < 0
			{
				return shape_ray_miss(), .Ok;
			}
			continue;
		}
		plane_t := numerator / denominator;
		if denominator > 0
		{
			earliest_exit = min(earliest_exit, plane_t);
		}
		else if plane_t > latest_entry
		{
			latest_entry=plane_t;
			entry_normal=normal;
		}
	}
	if earliest_exit < 0 || latest_entry > earliest_exit
	{
		return shape_ray_miss(), .Ok;
	}
	t := max(f32(0), latest_entry);
	normal := util.quaternion_transform(entry_normal, pose.orientation);
	return shape_ray_finish(t, normal, ray), .Ok;
}

compound_ray_test :: proc "contextless" (
	children: util.Buffer(Compound_Child), pose: Rigid_Pose, ray: Tree_Ray, registry: ^Shape_Registry,
) -> (Shape_Ray_Hit, Physics_Status)
{
	if children.memory == nil || children.length <= 0
	{
		return shape_ray_miss(), .Invalid_Description;
	}
	best := shape_ray_miss();
	best_t := ray.maximum_t;
	for child_index in 0 ..< children.length
	{
		child := children.memory[child_index];
		child_pose := rigid_pose_concatenate(
			{orientation=child.local_orientation, position=child.local_position}, pose,
		);
		child_ray := ray;
		child_ray.maximum_t = best_t;
		hit, status := shape_registry_ray_test(registry, child.shape_index, child_pose, child_ray);
		if status != .Ok
		{
			return shape_ray_miss(), status;
		}
		if hit.state == .Present
		{
			best = hit;
			best.child_index = i32(child_index);
			best_t = hit.t;
		}
	}
	return best, .Ok;
}

mesh_ray_test :: proc "contextless" (mesh: ^Mesh, pose: Rigid_Pose, ray: Tree_Ray) -> (Shape_Ray_Hit, Physics_Status)
{
	status := shape_ray_valid(ray);
	if status != .Ok || mesh == nil || mesh.triangles.memory == nil || mesh.triangles.length <= 0
	{
		return shape_ray_miss(), .Invalid_Argument;
	}
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	o := util.quaternion_transform(util.vector3_subtract(ray.origin, pose.position), inverse_orientation);
	d := util.quaternion_transform(ray.direction, inverse_orientation);
	best := shape_ray_miss();
	best_t := ray.maximum_t;
	for triangle_index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[triangle_index];
		a := util.vector3_multiply(triangle.a, mesh.scale);
		b := util.vector3_multiply(triangle.b, mesh.scale);
		c := util.vector3_multiply(triangle.c, mesh.scale);
		t, local_normal, state := triangle_ray_test_local(a, b, c, o, d);
		if state == .Present && t <= best_t
		{
			best = {
				t=t,
				normal=util.quaternion_transform(local_normal, pose.orientation),
				child_index=i32(triangle_index),
				state=.Present,
			};
			best_t = t;
		}
	}
	return best, .Ok;
}
