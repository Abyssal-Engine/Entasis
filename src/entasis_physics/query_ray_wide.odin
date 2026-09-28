// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

wide_ray_gather_local :: proc "contextless" (
	pose: Rigid_Pose, query: Wide_Ray_Query, result: ^Wide_Ray_Result,
) -> (origin, direction: util.Vector3_Wide, maximum_t: util.F32x8, active: util.I32x8)
{
	inverse_orientation := util.quaternion_conjugate(pose.orientation);
	for lane in 0 ..< query.lane_count
	{
		ray := query.rays[lane];
		status := shape_ray_valid(ray);
		result.status[lane] = status;
		result.hits[lane] = shape_ray_miss();
		if status != .Ok
		{
			continue;
		}
		util.vector3_wide_write_slot(&origin, lane, util.quaternion_transform(
			util.vector3_subtract(ray.origin, pose.position), inverse_orientation,
		));
		util.vector3_wide_write_slot(&direction, lane, util.quaternion_transform(ray.direction, inverse_orientation));
		maximum_t = simd.replace(maximum_t, lane, ray.maximum_t);
		active = simd.replace(active, lane, -1);
	}
	for lane in query.lane_count ..< util.PRODUCTION_LANE_COUNT
	{
		result.hits[lane] = shape_ray_miss();
		result.status[lane] = .Ok;
	}
	return;
}

wide_ray_write_hits :: proc "contextless" (
	query: Wide_Ray_Query, pose: Rigid_Pose, hit_mask: util.I32x8,
	t: util.F32x8, local_normal: util.Vector3_Wide, result: ^Wide_Ray_Result,
)
{
	for lane in 0 ..< query.lane_count
	{
		if simd.extract(hit_mask, lane) < 0
		{
			result.hits[lane] = {
				t=simd.extract(t, lane),
				normal=util.quaternion_transform(util.vector3_wide_read_slot(local_normal, lane), pose.orientation),
				child_index=0, state=.Present,
			};
		}
	}
}

wide_ray_box_axis :: proc "contextless" (
	origin, direction: util.F32x8, extent: f32,
) -> (entry, exit: util.F32x8, entry_sign: util.F32x8, valid: util.I32x8)
{
	parallel := transmute(util.I32x8)simd.lanes_le(simd.abs(direction), util.F32x8(1e-15));
	safe_direction := util.wide_select_f32(parallel, util.F32x8(1), direction);
	t0 := simd.div(simd.sub(util.F32x8(-extent), origin), safe_direction);
	t1 := simd.div(simd.sub(util.F32x8(extent), origin), safe_direction);
	entry = util.wide_select_f32(parallel, util.F32x8(-math.F32_MAX), simd.min(t0, t1));
	exit = util.wide_select_f32(parallel, util.F32x8(math.F32_MAX), simd.max(t0, t1));
	positive_direction := transmute(util.I32x8)simd.lanes_gt(direction, util.F32x8(0));
	entry_sign = util.wide_select_f32(positive_direction, util.F32x8(-1), util.F32x8(1));
	outside := transmute(util.I32x8)simd.lanes_gt(simd.abs(origin), util.F32x8(extent));
	valid = ~(parallel & outside);
	return;
}

wide_ray_test_box :: proc "contextless" (
	box: Box, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if box_validate(box) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction, maximum_t, active := wide_ray_gather_local(pose, query, &result);
	latest := util.F32x8(-math.F32_MAX);
	earliest := util.F32x8(math.F32_MAX);
	local_normal: util.Vector3_Wide;
	entries := [3]util.F32x8{};
	exits := [3]util.F32x8{};
	signs := [3]util.F32x8{};
	valid := active;
	valid_x, valid_y, valid_z: util.I32x8;
	entries[0], exits[0], signs[0], valid_x = wide_ray_box_axis(origin.x, direction.x, box.half_width);
	entries[1], exits[1], signs[1], valid_y = wide_ray_box_axis(origin.y, direction.y, box.half_height);
	entries[2], exits[2], signs[2], valid_z = wide_ray_box_axis(origin.z, direction.z, box.half_length);
	valid &= valid_x & valid_y & valid_z;
	for axis in 0 ..< 3
	{
		use_entry := transmute(util.I32x8)simd.lanes_gt(entries[axis], latest);
		latest = util.wide_select_f32(use_entry, entries[axis], latest);
		earliest = simd.min(earliest, exits[axis]);
		axis_normal: util.Vector3_Wide;
		if axis == 0
		{
			axis_normal.x = signs[axis];
		}
		else if axis == 1
		{
			axis_normal.y = signs[axis];
		}
		else
		{
			axis_normal.z = signs[axis];
		}
		local_normal = util.vector3_wide_select(use_entry, axis_normal, local_normal);
	}
	inside := transmute(util.I32x8)simd.lanes_lt(latest, util.F32x8(0));
	flip_inside := inside & transmute(util.I32x8)simd.lanes_lt(
		util.vector3_wide_dot(local_normal, origin), util.F32x8(0),
	);
	local_normal = util.vector3_wide_conditional_negate(flip_inside, local_normal);
	t := simd.max(util.F32x8(0), latest);
	hit_mask := valid &
		transmute(util.I32x8)simd.lanes_ge(earliest, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(earliest, latest) &
		transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	wide_ray_write_hits(query, pose, hit_mask, t, local_normal, &result);
	return result;
}

wide_ray_test_capsule :: proc "contextless" (
	capsule: Capsule, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if capsule_validate(capsule) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction, maximum_t, active := wide_ray_gather_local(pose, query, &result);
	direction_length := util.vector3_wide_length(direction);
	inverse_direction_length := simd.div(util.F32x8(1), direction_length);
	direction = util.vector3_wide_scale(direction, inverse_direction_length);
	t_offset := simd.max(
		util.F32x8(0),
		simd.sub(
		simd.neg(util.vector3_wide_dot(origin, direction)),
		util.F32x8(capsule.half_length + capsule.radius),
	),
	);
	origin = util.vector3_wide_add(origin, util.vector3_wide_scale(direction, t_offset));
	a := simd.add(simd.mul(direction.x, direction.x), simd.mul(direction.z, direction.z));
	b := simd.add(simd.mul(origin.x, direction.x), simd.mul(origin.z, direction.z));
	radius_squared := util.F32x8(capsule.radius * capsule.radius);
	c := simd.sub(simd.add(simd.mul(origin.x, origin.x), simd.mul(origin.z, origin.z)), radius_squared);
	radial := transmute(util.I32x8)simd.lanes_gt(a, util.F32x8(1e-8));
	discriminant := simd.sub(simd.mul(b, b), simd.mul(a, c));
	safe_a := util.wide_select_f32(radial, a, util.F32x8(1));
	radial_t := simd.max(
		simd.div(simd.sub(simd.neg(b), simd.sqrt(simd.max(util.F32x8(0), discriminant))), safe_a),
		simd.neg(t_offset),
	);
	radial_hit := util.vector3_wide_add(origin, util.vector3_wide_scale(direction, radial_t));
	radial_hit_valid := radial &
		transmute(util.I32x8)simd.lanes_ge(discriminant, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(radial_hit.y, util.F32x8(-capsule.half_length)) &
		transmute(util.I32x8)simd.lanes_le(radial_hit.y, util.F32x8(capsule.half_length));

	clamped_radial_y := simd.max(
		util.F32x8(-capsule.half_length), simd.min(util.F32x8(capsule.half_length), radial_hit.y),
	);
	clamped_origin_y := simd.max(
		util.F32x8(-capsule.half_length), simd.min(util.F32x8(capsule.half_length), origin.y),
	);
	sphere_y := util.wide_select_f32(radial, clamped_radial_y, clamped_origin_y);
	o_sphere := origin;
	o_sphere.y = simd.sub(o_sphere.y, sphere_y);
	cap_b := util.vector3_wide_dot(o_sphere, direction);
	cap_c := simd.sub(util.vector3_wide_dot(o_sphere, o_sphere), radius_squared);
	cap_discriminant := simd.sub(simd.mul(cap_b, cap_b), cap_c);
	cap_t := simd.max(
		simd.sub(simd.neg(cap_b), simd.sqrt(simd.max(util.F32x8(0), cap_discriminant))),
		simd.neg(t_offset),
	);
	use_cap := ~radial_hit_valid;
	local_t := util.wide_select_f32(use_cap, cap_t, radial_t);
	side_normal := util.Vector3_Wide{
		x=simd.mul(radial_hit.x, util.F32x8(1 / capsule.radius)),
		z=simd.mul(radial_hit.z, util.F32x8(1 / capsule.radius)),
	};
	cap_normal := util.vector3_wide_scale(
		util.vector3_wide_add(o_sphere, util.vector3_wide_scale(direction, cap_t)),
		util.F32x8(1 / capsule.radius),
	);
	local_normal := util.vector3_wide_select(use_cap, cap_normal, side_normal);
	t := simd.mul(simd.add(local_t, t_offset), inverse_direction_length);
	early_miss := transmute(util.I32x8)simd.lanes_gt(b, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(c, util.F32x8(0));
	radial_miss := radial & transmute(util.I32x8)simd.lanes_lt(discriminant, util.F32x8(0));
	cap_miss := use_cap & (
		(transmute(util.I32x8)simd.lanes_gt(cap_b, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(cap_c, util.F32x8(0))) |
		transmute(util.I32x8)simd.lanes_lt(cap_discriminant, util.F32x8(0))
	);
	hit_mask := active & ~early_miss & ~radial_miss & ~cap_miss &
		transmute(util.I32x8)simd.lanes_ge(t, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	wide_ray_write_hits(query, pose, hit_mask, t, local_normal, &result);
	return result;
}

wide_ray_test_cylinder :: proc "contextless" (
	cylinder: Cylinder, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if cylinder_validate(cylinder) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction, maximum_t, active := wide_ray_gather_local(pose, query, &result);
	direction_length := util.vector3_wide_length(direction);
	inverse_direction_length := simd.div(util.F32x8(1), direction_length);
	direction = util.vector3_wide_scale(direction, inverse_direction_length);
	t_offset := simd.max(
		util.F32x8(0),
		simd.sub(
		simd.neg(util.vector3_wide_dot(origin, direction)),
		util.F32x8(cylinder.half_length + cylinder.radius),
	),
	);
	origin = util.vector3_wide_add(origin, util.vector3_wide_scale(direction, t_offset));
	a := simd.add(simd.mul(direction.x, direction.x), simd.mul(direction.z, direction.z));
	b := simd.add(simd.mul(origin.x, direction.x), simd.mul(origin.z, direction.z));
	radius_squared := util.F32x8(cylinder.radius * cylinder.radius);
	c := simd.sub(simd.add(simd.mul(origin.x, origin.x), simd.mul(origin.z, origin.z)), radius_squared);
	radial := transmute(util.I32x8)simd.lanes_gt(a, util.F32x8(1e-8));
	discriminant := simd.sub(simd.mul(b, b), simd.mul(a, c));
	safe_a := util.wide_select_f32(radial, a, util.F32x8(1));
	radial_t := simd.max(
		simd.div(simd.sub(simd.neg(b), simd.sqrt(simd.max(util.F32x8(0), discriminant))), safe_a),
		simd.neg(t_offset),
	);
	radial_hit := util.vector3_wide_add(origin, util.vector3_wide_scale(direction, radial_t));
	radial_hit_valid := radial &
		transmute(util.I32x8)simd.lanes_ge(discriminant, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(radial_hit.y, util.F32x8(-cylinder.half_length)) &
		transmute(util.I32x8)simd.lanes_le(radial_hit.y, util.F32x8(cylinder.half_length));

	radial_disc_y := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(radial_hit.y, util.F32x8(-cylinder.half_length)),
		util.F32x8(-cylinder.half_length), util.F32x8(cylinder.half_length),
	);
	parallel_disc_y := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(direction.y, util.F32x8(0)),
		util.F32x8(-cylinder.half_length), util.F32x8(cylinder.half_length),
	);
	inside_height := transmute(util.I32x8)simd.lanes_lt(simd.abs(origin.y), util.F32x8(cylinder.half_length));
	parallel_disc_y = util.wide_select_f32(inside_height, origin.y, parallel_disc_y);
	disc_y := util.wide_select_f32(radial, radial_disc_y, parallel_disc_y);
	safe_direction_y := util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_le(simd.abs(direction.y), util.F32x8(1e-20)),
		util.F32x8(1), direction.y,
	);
	cap_t := simd.div(simd.sub(disc_y, origin.y), safe_direction_y);
	cap_hit := util.vector3_wide_add(origin, util.vector3_wide_scale(direction, cap_t));
	use_cap := ~radial_hit_valid;
	local_t := util.wide_select_f32(use_cap, cap_t, radial_t);
	side_normal := util.Vector3_Wide{
		x=simd.mul(radial_hit.x, util.F32x8(1 / cylinder.radius)),
		z=simd.mul(radial_hit.z, util.F32x8(1 / cylinder.radius)),
	};
	cap_normal := util.Vector3_Wide{y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_gt(direction.y, util.F32x8(0)), util.F32x8(-1), util.F32x8(1),
		)};
	local_normal := util.vector3_wide_select(use_cap, cap_normal, side_normal);
	t := simd.mul(simd.add(local_t, t_offset), inverse_direction_length);
	early_miss := transmute(util.I32x8)simd.lanes_gt(b, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_gt(c, util.F32x8(0));
	radial_miss := radial & transmute(util.I32x8)simd.lanes_lt(discriminant, util.F32x8(0));
	outside_height := transmute(util.I32x8)simd.lanes_gt(simd.abs(origin.y), util.F32x8(cylinder.half_length));
	moving_outward := transmute(util.I32x8)simd.lanes_ge(simd.mul(origin.y, direction.y), util.F32x8(0));
	parallel_y := transmute(util.I32x8)simd.lanes_le(simd.abs(direction.y), util.F32x8(1e-20));
	outside_disc := transmute(util.I32x8)simd.lanes_gt(
		simd.add(simd.mul(cap_hit.x, cap_hit.x), simd.mul(cap_hit.z, cap_hit.z)), radius_squared,
	);
	cap_miss := use_cap & ((outside_height & moving_outward) | parallel_y | outside_disc);
	hit_mask := active & ~early_miss & ~radial_miss & ~cap_miss &
		transmute(util.I32x8)simd.lanes_ge(t, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	wide_ray_write_hits(query, pose, hit_mask, t, local_normal, &result);
	return result;
}

wide_ray_test_triangle :: proc "contextless" (
	triangle: Triangle, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if triangle_validate(triangle) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction, maximum_t, active := wide_ray_gather_local(pose, query, &result);
	ab_scalar := util.vector3_subtract(triangle.b, triangle.a);
	ac_scalar := util.vector3_subtract(triangle.c, triangle.a);
	normal_scalar := util.vector3_cross(ac_scalar, ab_scalar);
	normal_length := util.vector3_length(normal_scalar);
	if normal_length <= 1e-20
	{
		for lane in 0 ..< query.lane_count
		{
			result.status[lane] = .Invalid_Description;
		}
		return result;
	}
	ab := util.Vector3_Wide{x=util.F32x8(ab_scalar.x), y=util.F32x8(ab_scalar.y), z=util.F32x8(ab_scalar.z)};
	ac := util.Vector3_Wide{x=util.F32x8(ac_scalar.x), y=util.F32x8(ac_scalar.y), z=util.F32x8(ac_scalar.z)};
	normal := util.Vector3_Wide{
		x=util.F32x8(normal_scalar.x),
		y=util.F32x8(normal_scalar.y),
		z=util.F32x8(normal_scalar.z),
	};
	a := util.Vector3_Wide{x=util.F32x8(triangle.a.x), y=util.F32x8(triangle.a.y), z=util.F32x8(triangle.a.z)};
	dn := simd.neg(util.vector3_wide_dot(direction, normal));
	ao := util.vector3_wide_subtract(origin, a);
	numerator := util.vector3_wide_dot(ao, normal);
	ao_cross_d := util.vector3_wide_cross(ao, direction);
	v := simd.neg(util.vector3_wide_dot(ac, ao_cross_d));
	w := util.vector3_wide_dot(ab, ao_cross_d);
	t := simd.div(numerator, dn);
	hit_mask := active &
		transmute(util.I32x8)simd.lanes_gt(dn, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(numerator, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(v, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(v, dn) &
		transmute(util.I32x8)simd.lanes_ge(w, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_le(simd.add(v, w), dn) &
		transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	unit_normal := util.vector3_wide_scale(normal, util.F32x8(1/normal_length));
	wide_ray_write_hits(query, pose, hit_mask, t, unit_normal, &result);
	return result;
}

wide_ray_test_convex_hull :: proc "contextless" (
	hull: ^Convex_Hull, pose: Rigid_Pose, query: Wide_Ray_Query,
) -> Wide_Ray_Result
{
	result: Wide_Ray_Result;
	if convex_hull_validate(hull) != .Ok || query.lane_count < 0 || query.lane_count > util.PRODUCTION_LANE_COUNT
	{
		for lane in 0 ..< util.PRODUCTION_LANE_COUNT
		{
			result.status[lane] = .Invalid_Argument;
		}
		return result;
	}
	origin, direction, maximum_t, active := wide_ray_gather_local(pose, query, &result);
	latest := util.F32x8(-math.F32_MAX);
	earliest := util.F32x8(math.F32_MAX);
	entry_normal: util.Vector3_Wide;
	valid := active;
	for face_index in 0 ..< hull.face_start_indices.length
	{
		normal_scalar, offset, face_status := convex_hull_face_normal_and_offset(hull, int(face_index));
		if face_status != .Ok
		{
			for lane in 0 ..< query.lane_count
			{
				result.status[lane] = face_status;
			}
			return result;
		}
		normal := util.Vector3_Wide{
			x=util.F32x8(normal_scalar.x),
			y=util.F32x8(normal_scalar.y),
			z=util.F32x8(normal_scalar.z),
		};
		numerator := simd.sub(util.F32x8(offset), util.vector3_wide_dot(normal, origin));
		denominator := util.vector3_wide_dot(normal, direction);
		parallel := transmute(util.I32x8)simd.lanes_le(simd.abs(denominator), util.F32x8(1e-14));
		valid &= ~(parallel & transmute(util.I32x8)simd.lanes_lt(numerator, util.F32x8(0)));
		safe_denominator := util.wide_select_f32(parallel, util.F32x8(1), denominator);
		plane_t := simd.div(numerator, safe_denominator);
		exit_plane := ~parallel & transmute(util.I32x8)simd.lanes_gt(denominator, util.F32x8(0));
		entry_plane := ~parallel & ~exit_plane;
		earliest = util.wide_select_f32(exit_plane, simd.min(earliest, plane_t), earliest);
		use_entry := entry_plane & transmute(util.I32x8)simd.lanes_gt(plane_t, latest);
		latest = util.wide_select_f32(use_entry, plane_t, latest);
		entry_normal = util.vector3_wide_select(use_entry, normal, entry_normal);
	}
	t := simd.max(util.F32x8(0), latest);
	hit_mask := valid &
		transmute(util.I32x8)simd.lanes_ge(earliest, util.F32x8(0)) &
		transmute(util.I32x8)simd.lanes_ge(earliest, latest) &
		transmute(util.I32x8)simd.lanes_le(t, maximum_t);
	wide_ray_write_hits(query, pose, hit_mask, t, entry_normal, &result);
	return result;
}
