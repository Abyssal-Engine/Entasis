// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

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
		if direction == 0
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
	face_count: int = int(hull.face_start_indices.length);
	for bundle_index in 0 ..< (face_count+util.PRODUCTION_LANE_COUNT-1)/util.PRODUCTION_LANE_COUNT
	{
		bundle: Hull_Bounding_Plane_Bundle = hull.bounding_planes.memory[bundle_index];
		lane_count: int = min(util.PRODUCTION_LANE_COUNT, face_count-bundle_index*util.PRODUCTION_LANE_COUNT);
		numerator: util.F32x8 = bundle.offset - (bundle.normal.x*o.x + bundle.normal.y*o.y + bundle.normal.z*o.z);
		denominator: util.F32x8 = bundle.normal.x*d.x + bundle.normal.y*d.y + bundle.normal.z*d.z;
		active: util.I32x8 = transmute(util.I32x8)simd.lanes_lt(util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(lane_count)));
		parallel: util.I32x8 = transmute(util.I32x8)simd.lanes_le(simd.abs(denominator), util.F32x8(1e-14));
		// producer-owned bundles include padding, skipped divisions use zero over one
		divide: util.I32x8 = active & ~parallel;
		plane_times: util.F32x8 = util.wide_select_f32(divide, numerator, {}) / util.wide_select_f32(divide, denominator, util.F32x8(1));
		for lane in 0 ..< lane_count
		{
			lane_numerator: f32 = simd.extract(numerator, lane);
			lane_denominator: f32 = simd.extract(denominator, lane);
			if abs(lane_denominator) <= 1e-14
			{
				if lane_numerator < 0
				{
					return shape_ray_miss(), .Ok;
				}
				continue;
			}
			plane_t: f32 = simd.extract(plane_times, lane);
			if lane_denominator > 0
			{
				earliest_exit = min(earliest_exit, plane_t);
			}
			else if plane_t > latest_entry
			{
				latest_entry = plane_t;
				entry_normal = util.vector3_wide_read_slot(bundle.normal, lane);
			}
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
	q: util.Quaternion = pose.orientation;
	x2: f32 = q.x+q.x;
	y2: f32 = q.y+q.y;
	z2: f32 = q.z+q.z;
	xx2: f32 = q.x*x2;
	xy2: f32 = q.x*y2;
	xz2: f32 = q.x*z2;
	yy2: f32 = q.y*y2;
	yz2: f32 = q.y*z2;
	zz2: f32 = q.z*z2;
	wx2: f32 = q.w*x2;
	wy2: f32 = q.w*y2;
	wz2: f32 = q.w*z2;
	c00: f32 = (f32(1)-yy2)-zz2;
	c01: f32 = xy2-wz2;
	c02: f32 = xz2+wy2;
	c10: f32 = xy2+wz2;
	c11: f32 = (f32(1)-xx2)-zz2;
	c12: f32 = yz2-wx2;
	c20: f32 = xz2-wy2;
	c21: f32 = yz2+wx2;
	c22: f32 = (f32(1)-xx2)-yy2;
	best := shape_ray_miss();
	best_t := ray.maximum_t;
	for child_index in 0 ..< children.length
	{
		child := children.memory[child_index];
		v: util.Vector3 = child.local_position;
		child_pose: Rigid_Pose = {
			orientation=util.quaternion_concatenate(child.local_orientation, pose.orientation),
			position={
				(((v.x*c00)+(v.y*c01))+(v.z*c02))+pose.position.x,
				(((v.x*c10)+(v.y*c11))+(v.z*c12))+pose.position.y,
				(((v.x*c20)+(v.y*c21))+(v.z*c22))+pose.position.z,
			},
		};
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

@(private="file")
mesh_ray_test_bundle :: proc "contextless" (
	triangle: Triangle_Wide, origin, direction: util.Vector3_Wide,
) -> (util.F32x8, util.Vector3_Wide, util.I32x8)
{
	ab: util.Vector3_Wide = util.vector3_wide_subtract(triangle.b, triangle.a);
	ac: util.Vector3_Wide = util.vector3_wide_subtract(triangle.c, triangle.a);
	normal: util.Vector3_Wide = util.vector3_wide_cross(ac, ab);
	dn: util.F32x8 = simd.neg(util.vector3_wide_dot(direction, normal));
	// complement scalar rejection predicates so unordered intermediates retain their behavior
	mask: util.I32x8 = util.I32x8(-1) & ~transmute(util.I32x8)simd.lanes_le(dn, util.F32x8(0));
	if simd.reduce_or(mask) == 0
	{
		return {}, {}, {};
	}
	ao: util.Vector3_Wide = util.vector3_wide_subtract(origin, triangle.a);
	numerator: util.F32x8 = util.vector3_wide_dot(ao, normal);
	mask &= ~transmute(util.I32x8)simd.lanes_lt(numerator, util.F32x8(0));
	if simd.reduce_or(mask) == 0
	{
		return {}, {}, {};
	}
	ao_cross_d: util.Vector3_Wide = util.vector3_wide_cross(ao, direction);
	v: util.F32x8 = simd.neg(util.vector3_wide_dot(ac, ao_cross_d));
	mask &= ~(transmute(util.I32x8)simd.lanes_lt(v, util.F32x8(0)) | transmute(util.I32x8)simd.lanes_gt(v, dn));
	if simd.reduce_or(mask) == 0
	{
		return {}, {}, {};
	}
	w: util.F32x8 = util.vector3_wide_dot(ab, ao_cross_d);
	mask &= ~(transmute(util.I32x8)simd.lanes_lt(w, util.F32x8(0)) | transmute(util.I32x8)simd.lanes_gt(simd.add(v, w), dn));
	if simd.reduce_or(mask) == 0
	{
		return {}, {}, {};
	}
	normal_length: util.F32x8 = simd.sqrt(util.vector3_wide_dot(normal, normal));
	mask &= ~transmute(util.I32x8)simd.lanes_le(normal_length, util.F32x8(1e-20));
	if simd.reduce_or(mask) == 0
	{
		return {}, {}, {};
	}
	// only rejected lanes receive division sentinels. accepted operands stay exact
	numerator = util.wide_select_f32(mask, numerator, {});
	dn = util.wide_select_f32(mask, dn, util.F32x8(1));
	normal = util.vector3_wide_select(mask, normal, {});
	normal_length = util.wide_select_f32(mask, normal_length, util.F32x8(1));
	t: util.F32x8 = simd.div(numerator, dn);
	inverse_length: util.F32x8 = simd.div(util.F32x8(1), normal_length);
	return t, util.vector3_wide_scale(normal, inverse_length), mask;
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
	origin_wide: util.Vector3_Wide = util.vector3_wide_broadcast(o);
	direction_wide: util.Vector3_Wide = util.vector3_wide_broadcast(d);
	triangle_index: int;
	for triangle_index+util.PRODUCTION_LANE_COUNT <= int(mesh.triangles.length)
	{
		bundle: Triangle_Wide;
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			triangle: Triangle = mesh.triangles.memory[triangle_index+lane];
			triangle.a = util.vector3_multiply(triangle.a, mesh.scale);
			triangle.b = util.vector3_multiply(triangle.b, mesh.scale);
			triangle.c = util.vector3_multiply(triangle.c, mesh.scale);
			triangle_wide_write_slot_trusted(&bundle, lane, triangle);
		}
		t: util.F32x8;
		local_normal: util.Vector3_Wide;
		mask: util.I32x8;
		t, local_normal, mask = #force_inline mesh_ray_test_bundle(bundle, origin_wide, direction_wide);
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			lane_t: f32 = simd.extract(t, lane);
			if simd.extract(mask, lane) != 0 && lane_t <= best_t
			{
				best = {
					t=lane_t,
					normal=util.quaternion_transform(util.vector3_wide_read_slot(local_normal, lane), pose.orientation),
					child_index=i32(triangle_index+lane),
					state=.Present,
				};
				best_t = lane_t;
			}
		}
		triangle_index += util.PRODUCTION_LANE_COUNT;
	}
	for ; triangle_index < int(mesh.triangles.length); triangle_index += 1
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
