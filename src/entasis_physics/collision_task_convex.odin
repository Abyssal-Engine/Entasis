// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

MESH_FACE_COLLISION_FLAG :: i32(32768);
TRIANGLE_BACKFACE_REJECTION_THRESHOLD :: f32(-1e-2);
Collision_Test_Proc :: #type proc "contextless" (
	shape_a, shape_b: rawptr,
	pose_a, pose_b: Rigid_Pose,
	speculative_margin: f32,
	shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status);
Collision_Shape_View :: struct
{
	shape: rawptr,
	batch: ^Shape_Batch,
	pose:  Rigid_Pose,
}

collision_shape_view :: proc "contextless" (
	shape: rawptr, type_id: int, pose: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Collision_Shape_View, Physics_Status)
{
	if shape == nil || shapes == nil || type_id < 0 || type_id >= shapes.registered_type_count
	{
		return {}, .Invalid_Argument;
	}
	return {shape, &shapes.batches[type_id], pose}, .Ok;
}

// public low-level support dispatch. iterative kernels select their native or
// contextual representation once, then call the corresponding leaf directly
collision_support_world :: proc "contextless" (
	view: Collision_Shape_View, direction: util.Vector3, shapes: ^Shape_Registry,
) -> (util.Vector3, Physics_Status)
{
	if view.batch.dispatch == .Contextual
	{
		return collision_support_world_contextual(collision_contextual_shape_view(view), direction, shapes);
	}
	return collision_support_world_native(view, direction, shapes);
}

collision_support_world_native :: proc "contextless" (
	view: Collision_Shape_View, direction: util.Vector3, shapes: ^Shape_Registry,
) -> (util.Vector3, Physics_Status)
{
	if util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	local_direction := util.quaternion_transform(direction, util.quaternion_conjugate(view.pose.orientation));
	local_support, status := view.batch.metadata.support(view.shape, local_direction, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	return rigid_pose_transform(local_support, view.pose), .Ok;
}

collision_pose_axes :: proc "contextless" (pose: Rigid_Pose) -> [3]util.Vector3
{
	return {
		util.quaternion_transform_unit_x(pose.orientation),
		util.quaternion_transform_unit_y(pose.orientation),
		util.quaternion_transform_unit_z(pose.orientation),
	};
}

collision_triangle_world_vertices :: proc "contextless" (triangle: Triangle, pose: Rigid_Pose) -> [3]util.Vector3
{
	return {
		rigid_pose_transform(triangle.a, pose),
		rigid_pose_transform(triangle.b, pose),
		rigid_pose_transform(triangle.c, pose),
	};
}

collision_single_contact :: proc "contextless" (
	pose_a, pose_b: Rigid_Pose, normal: util.Vector3, depth: f32,
	point_a, point_b: util.Vector3, speculative_margin: f32, feature_id: i32 = 0,
) -> Convex_Contact_Manifold
{
	manifold := Convex_Contact_Manifold{offset_b=util.vector3_subtract(pose_b.position, pose_a.position)};
	if depth < -max(speculative_margin, 0)
	{
		return manifold;
	}
	contact_world := util.vector3_scale(util.vector3_add(point_a, point_b), 0.5);
	manifold.count = 1;
	manifold.normal = normal;
	manifold.contacts[0] = {util.vector3_subtract(contact_world, pose_a.position), depth, feature_id};
	return manifold;
}

sphere_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Sphere)(shape_a)^;
	b := (^Sphere)(shape_b)^;
	if sphere_validate(a) != .Ok || sphere_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	distance := util.vector3_length(offset_b);
	normal := util.Vector3{0, 1, 0};
	if distance > 0
	{
		normal = util.vector3_scale(offset_b, -1 / distance);
	}
	depth := a.radius + b.radius - distance;
	point_a := util.vector3_add(pose_a.position, util.vector3_scale(normal, -a.radius));
	point_b := util.vector3_add(pose_b.position, util.vector3_scale(normal, b.radius));
	return collision_single_contact(pose_a, pose_b, normal, depth, point_a, point_b, speculative_margin), .Ok;
}

sphere_capsule_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Sphere)(shape_a)^;
	b := (^Capsule)(shape_b)^;
	if sphere_validate(a) != .Ok || capsule_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	axis_b := util.quaternion_transform_unit_y(pose_b.orientation);
	sphere_from_b := util.vector3_subtract(pose_a.position, pose_b.position);
	t := max(-b.half_length, min(b.half_length, util.vector3_dot(sphere_from_b, axis_b)));
	segment_point := util.vector3_add(pose_b.position, util.vector3_scale(axis_b, t));
	segment_to_sphere := util.vector3_subtract(pose_a.position, segment_point);
	distance := util.vector3_length(segment_to_sphere);
	normal := util.quaternion_transform_unit_x(pose_b.orientation);
	if distance > 0
	{
		normal = util.vector3_scale(segment_to_sphere, 1 / distance);
	}
	depth := a.radius + b.radius - distance;
	point_a := util.vector3_add(pose_a.position, util.vector3_scale(normal, -a.radius));
	point_b := util.vector3_add(segment_point, util.vector3_scale(normal, b.radius));
	return collision_single_contact(pose_a, pose_b, normal, depth, point_a, point_b, speculative_margin), .Ok;
}

sphere_box_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Sphere)(shape_a)^;
	b := (^Box)(shape_b)^;
	if sphere_validate(a) != .Ok || box_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	local_sphere := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	closest := util.Vector3{
		max(-b.half_width, min(b.half_width, local_sphere.x)),
		max(-b.half_height, min(b.half_height, local_sphere.y)),
		max(-b.half_length, min(b.half_length, local_sphere.z)),
	};
	local_delta := util.vector3_subtract(local_sphere, closest);
	distance := util.vector3_length(local_delta);
	local_normal: util.Vector3;
	depth: f32;
	if distance > 0
	{
		local_normal = util.vector3_scale(local_delta, 1 / distance);
		depth = a.radius - distance;
	}
	else
	{
		depth_x := b.half_width - abs(local_sphere.x);
		depth_y := b.half_height - abs(local_sphere.y);
		depth_z := b.half_length - abs(local_sphere.z);
		if depth_x <= depth_y && depth_x <= depth_z
		{
			local_normal = {1, 0, 0};
			if local_sphere.x < 0
			{
				local_normal.x = -1;
			}
			depth = a.radius + depth_x;
			closest.x = local_normal.x * b.half_width;
		}
		else if depth_y <= depth_z
		{
			local_normal = {0, 1, 0};
			if local_sphere.y < 0
			{
				local_normal.y = -1;
			}
			depth = a.radius + depth_y;
			closest.y = local_normal.y * b.half_height;
		}
		else
		{
			local_normal = {0, 0, 1};
			if local_sphere.z < 0
			{
				local_normal.z = -1;
			}
			depth = a.radius + depth_z;
			closest.z = local_normal.z * b.half_length;
		}
	}
	normal := util.quaternion_transform(local_normal, pose_b.orientation);
	point_a := util.vector3_add(pose_a.position, util.vector3_scale(normal, -a.radius));
	point_b := rigid_pose_transform(closest, pose_b);
	return collision_single_contact(pose_a, pose_b, normal, depth, point_a, point_b, speculative_margin), .Ok;
}

collision_closest_point_triangle :: proc "contextless" (
	point, a, b, c: util.Vector3,
) -> (util.Vector3, i32)
{
	ab := util.vector3_subtract(b, a);
	ac := util.vector3_subtract(c, a);
	ap := util.vector3_subtract(point, a);
	d1 := util.vector3_dot(ab, ap);
	d2 := util.vector3_dot(ac, ap);
	if d1 <= 0 && d2 <= 0
	{
		return a, 0;
	}
	bp := util.vector3_subtract(point, b);
	d3 := util.vector3_dot(ab, bp);
	d4 := util.vector3_dot(ac, bp);
	if d3 >= 0 && d4 <= d3
	{
		return b, 0;
	}
	vc := d1*d4 - d3*d2;
	if vc <= 0 && d1 >= 0 && d3 <= 0
	{
		v := d1 / (d1 - d3);
		return util.vector3_add(a, util.vector3_scale(ab, v)), 0;
	}
	cp := util.vector3_subtract(point, c);
	d5 := util.vector3_dot(ab, cp);
	d6 := util.vector3_dot(ac, cp);
	if d6 >= 0 && d5 <= d6
	{
		return c, 0;
	}
	vb := d5*d2 - d1*d6;
	if vb <= 0 && d2 >= 0 && d6 <= 0
	{
		w := d2 / (d2 - d6);
		return util.vector3_add(a, util.vector3_scale(ac, w)), 0;
	}
	va := d3*d6 - d5*d4;
	if va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0
	{
		bc := util.vector3_subtract(c, b);
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return util.vector3_add(b, util.vector3_scale(bc, w)), 0;
	}
	denominator := 1 / (va + vb + vc);
	v := vb * denominator;
	w := vc * denominator;
	return util.vector3_add(
		a,
		util.vector3_add(util.vector3_scale(ab, v), util.vector3_scale(ac, w))
	), MESH_FACE_COLLISION_FLAG;
}

sphere_triangle_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Sphere)(shape_a)^;
	b := (^Triangle)(shape_b)^;
	if sphere_validate(a) != .Ok || triangle_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	vertices := collision_triangle_world_vertices(b, pose_b);
	closest, feature_id := collision_closest_point_triangle(pose_a.position, vertices[0], vertices[1], vertices[2]);
	delta := util.vector3_subtract(pose_a.position, closest);
	distance := util.vector3_length(delta);
	manifold := Convex_Contact_Manifold{offset_b=util.vector3_subtract(pose_b.position, pose_a.position)};
	if distance <= 0
	{
		return manifold, .Ok;
	}
	normal := util.vector3_scale(delta, 1 / distance);
	local_ab := util.vector3_subtract(b.b, b.a);
	local_ac := util.vector3_subtract(b.c, b.a);
	local_triangle_normal := util.vector3_normalize(util.vector3_cross(local_ab, local_ac));
	if util.vector3_dot(local_triangle_normal, normal) > -TRIANGLE_BACKFACE_REJECTION_THRESHOLD
	{
		return manifold, .Ok;
	}
	depth := a.radius - distance;
	if depth < -speculative_margin
	{
		return manifold, .Ok;
	}
	manifold.count = 1;
	manifold.normal = normal;
	manifold.contacts[0] = {
		offset=util.vector3_subtract(closest, pose_a.position),
		depth=depth,
		feature_id=feature_id,
	};
	return manifold, .Ok;
}

sphere_cylinder_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Sphere)(shape_a)^;
	b := (^Cylinder)(shape_b)^;
	if sphere_validate(a) != .Ok || cylinder_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	local_sphere := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	horizontal_length := math.sqrt(local_sphere.x*local_sphere.x + local_sphere.z*local_sphere.z);
	closest := local_sphere;
	if horizontal_length > b.radius
	{
		scale := b.radius / horizontal_length;
		closest.x *= scale;
		closest.z *= scale;
	}
	closest.y = max(-b.half_length, min(b.half_length, closest.y));
	sphere_to_closest_local_b := util.vector3_subtract(closest, local_sphere);
	distance := util.vector3_length(sphere_to_closest_local_b);
	local_normal: util.Vector3;
	depth: f32;
	if distance > 1e-7
	{
		local_normal = util.vector3_scale(sphere_to_closest_local_b, -1 / distance);
		depth = a.radius - distance;
	}
	else
	{
		depth_y := b.half_length - abs(local_sphere.y);
		depth_horizontal := b.radius - horizontal_length;
		if depth_y <= depth_horizontal
		{
			local_normal = {0, 1, 0};
			if local_sphere.y < 0
			{
				local_normal.y = -1;
			}
			depth = a.radius + depth_y;
		}
		else
		{
			local_normal = {1, 0, 0};
			if horizontal_length > b.radius * 1e-5
			{
				local_normal = {local_sphere.x / horizontal_length, 0, local_sphere.z / horizontal_length};
			}
			depth = a.radius + depth_horizontal;
		}
	}
	manifold := Convex_Contact_Manifold{offset_b=util.vector3_subtract(pose_b.position, pose_a.position)};
	if depth < -speculative_margin
	{
		return manifold, .Ok;
	}
	normal := util.quaternion_transform(local_normal, pose_b.orientation);
	manifold.count = 1;
	manifold.normal = normal;
	manifold.contacts[0] = {
		offset=util.quaternion_transform(sphere_to_closest_local_b, pose_b.orientation),
		depth=depth,
	};
	return manifold, .Ok;
}

sphere_convex_hull_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil
	{
		return {}, .Invalid_Argument;
	}
	a := (^Sphere)(shape_a)^;
	b := (^Convex_Hull)(shape_b);
	if sphere_validate(a) != .Ok || convex_hull_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	a_wide: Sphere_Wide;
	b_wide: Convex_Hull_Wide;
	offset_b_wide: util.Vector3_Wide;
	orientation_b_wide: util.Quaternion_Wide;
	_ = sphere_wide_write_slot(&a_wide, 0, a);
	b_wide.hulls[0] = b;
	util.vector3_wide_write_slot(&offset_b_wide, 0, util.vector3_subtract(pose_b.position, pose_a.position));
	util.quaternion_wide_write_slot(&orientation_b_wide, 0, pose_b.orientation);
	wide, status := sphere_convex_hull_test_wide(
		a_wide, b_wide, util.F32x8(speculative_margin), offset_b_wide, orientation_b_wide, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_1_manifold_wide_read_lane(&wide, offset_b_wide, 0);
}

collision_segment_closest_parameters :: proc "contextless" (
	center_a, axis_a: util.Vector3, half_a: f32,
	center_b, axis_b: util.Vector3, half_b: f32,
) -> (f32, f32)
{
	offset_b := util.vector3_subtract(center_b, center_a);
	axis_a_offset := util.vector3_dot(axis_a, offset_b);
	axis_b_offset := util.vector3_dot(axis_b, offset_b);
	axis_dot := util.vector3_dot(axis_a, axis_b);
	denominator := max(1e-15, 1 - axis_dot*axis_dot);
	ta := (axis_a_offset - axis_b_offset*axis_dot) / denominator;
	tb := ta*axis_dot - axis_b_offset;
	abs_axis_dot := abs(axis_dot);
	b_onto_a := half_b * abs_axis_dot;
	a_onto_b := half_a * abs_axis_dot;
	a_min := max(-half_a, min(half_a, axis_a_offset - b_onto_a));
	a_max := min(half_a, max(-half_a, axis_a_offset + b_onto_a));
	b_min := max(-half_b, min(half_b, -a_onto_b - axis_b_offset));
	b_max := min(half_b, max(-half_b, a_onto_b - axis_b_offset));
	ta = max(a_min, min(a_max, ta));
	tb = max(b_min, min(b_max, tb));
	return ta, tb;
}

capsule_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	a := (^Capsule)(shape_a)^;
	b := (^Capsule)(shape_b)^;
	if capsule_validate(a) != .Ok || capsule_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	da := util.quaternion_transform_unit_y(pose_a.orientation);
	xa := util.quaternion_transform_unit_x(pose_a.orientation);
	db := util.quaternion_transform_unit_y(pose_b.orientation);
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	da_offset_b := util.vector3_dot(da, offset_b);
	db_offset_b := util.vector3_dot(db, offset_b);
	dadb := util.vector3_dot(da, db);
	ta := (da_offset_b - db_offset_b * dadb) / max(f32(1e-15), 1 - dadb * dadb);
	tb := ta * dadb - db_offset_b;
	abs_dadb := abs(dadb);
	b_onto_a_offset := b.half_length * abs_dadb;
	a_onto_b_offset := a.half_length * abs_dadb;
	a_min := max(-a.half_length, min(a.half_length, da_offset_b - b_onto_a_offset));
	a_max := min(a.half_length, max(-a.half_length, da_offset_b + b_onto_a_offset));
	b_min := max(-b.half_length, min(b.half_length, -a_onto_b_offset - db_offset_b));
	b_max := min(b.half_length, max(-b.half_length, a_onto_b_offset - db_offset_b));
	ta = min(max(ta, a_min), a_max);
	tb = min(max(tb, b_min), b_max);
	closest_a := util.vector3_scale(da, ta);
	closest_b := util.vector3_add(util.vector3_scale(db, tb), offset_b);
	normal := util.vector3_subtract(closest_a, closest_b);
	distance := util.vector3_length(normal);
	if distance > 1e-7
	{
		normal = util.vector3_scale(normal, 1 / distance);
	}
	else
	{
		normal = xa;
	}
	plane_normal := util.vector3_cross(db, normal);
	plane_normal_length_squared := util.vector3_length_squared(plane_normal);
	squared_angle: f32;
	if plane_normal_length_squared >= 1e-10
	{
		numerator := util.vector3_dot(da, plane_normal);
		squared_angle = numerator * numerator / plane_normal_length_squared;
	}
	lower_threshold :: f32(0.01 * 0.01);
	upper_threshold :: f32(0.05 * 0.05);
	interval_weight := max(
		f32(0),
		min(f32(1), (upper_threshold - squared_angle) / (upper_threshold - lower_threshold))
	);
	weighted_ta := ta - ta * interval_weight;
	a_min = interval_weight * a_min + weighted_ta;
	a_max = interval_weight * a_max + weighted_ta;
	offsets := [2]util.Vector3{util.vector3_scale(da, a_min), util.vector3_scale(da, a_max)};
	db_normal := util.vector3_dot(db, normal);
	inverse_dadb := f32(0);
	if abs_dadb > 0
	{
		inverse_dadb = 1 / dadb;
	}
	projected_tb := [2]f32{
		max(b_min, min(b_max, (a_min - da_offset_b) * inverse_dadb)),
		max(b_min, min(b_max, (a_max - da_offset_b) * inverse_dadb)),
	};
	combined_radius := a.radius + b.radius;
	manifold := Convex_Contact_Manifold{offset_b=offset_b, normal=normal};
	for contact_index in 0 ..< 2
	{
		offset_from_b := util.vector3_subtract(offsets[contact_index], offset_b);
		contact_distance := distance;
		if abs_dadb >= 1e-7
		{
			contact_distance = util.vector3_dot(offset_from_b, normal) - db_normal * projected_tb[contact_index];
		}
		depth := combined_radius - contact_distance;
		exists := depth >= -speculative_margin;
		if contact_index == 1
		{
			exists = exists && a_max - a_min > 1e-7 * a.half_length;
		}
		if exists
		{
			manifold.contacts[manifold.count] = {
				offset=util.vector3_add(offsets[contact_index], util.vector3_scale(normal, depth * 0.5 - a.radius)),
				depth=depth,
				feature_id=i32(contact_index),
			};
			manifold.count += 1;
		}
	}
	return manifold, .Ok;
}

collision_pair_views :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Collision_Shape_View, Collision_Shape_View, Physics_Status)
{
	a, status_a := collision_shape_view(shape_a, type_a, pose_a, shapes);
	if status_a != .Ok
	{
		return {}, {}, status_a;
	}
	b, status_b := collision_shape_view(shape_b, type_b, pose_b, shapes);
	if status_b != .Ok
	{
		return {}, {}, status_b;
	}
	return a, b, .Ok;
}

capsule_box_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil
	{
		return {}, .Invalid_Argument;
	}
	return capsule_box_test_source(
		(^Capsule)(shape_a)^, (^Box)(shape_b)^, pose_a, pose_b, speculative_margin,
	);
}

capsule_triangle_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return capsule_triangle_test_source(
		(^Capsule)(shape_a)^, (^Triangle)(shape_b)^, pose_a, pose_b, speculative_margin,
	);
}

capsule_cylinder_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return capsule_cylinder_test_source(
		(^Capsule)(shape_a)^, (^Cylinder)(shape_b)^, pose_a, pose_b, speculative_margin,
	);
}

capsule_convex_hull_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil
	{
		return {}, .Invalid_Argument;
	}
	a := (^Capsule)(shape_a)^;
	b := (^Convex_Hull)(shape_b);
	if capsule_validate(a) != .Ok || convex_hull_validate(b) != .Ok
	{
		return {}, .Invalid_Description;
	}
	a_wide: Capsule_Wide;
	b_wide: Convex_Hull_Wide;
	offset_b_wide: util.Vector3_Wide;
	orientation_a_wide: util.Quaternion_Wide;
	orientation_b_wide: util.Quaternion_Wide;
	_ = capsule_wide_write_slot(&a_wide, 0, a);
	b_wide.hulls[0] = b;
	util.vector3_wide_write_slot(&offset_b_wide, 0, util.vector3_subtract(pose_b.position, pose_a.position));
	util.quaternion_wide_write_slot(&orientation_a_wide, 0, pose_a.orientation);
	util.quaternion_wide_write_slot(&orientation_b_wide, 0, pose_b.orientation);
	wide, status := capsule_convex_hull_test_wide(
		a_wide, b_wide, util.F32x8(speculative_margin), offset_b_wide,
		orientation_a_wide, orientation_b_wide, 1,
	);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_2_manifold_wide_read_lane(&wide, offset_b_wide, 0);
}

collision_box_face_contacts :: proc "contextless" (
	manifold: ^Convex_Contact_Manifold, a, b: Box, pose_a, pose_b: Rigid_Pose,
)
{
	if manifold == nil || manifold.count == 0
	{
		return;
	}
	axes_a := collision_pose_axes(pose_a);
	axes_b := collision_pose_axes(pose_b);
	for axis_index in 0 ..< 3
	{
		if abs(util.vector3_dot(axes_a[axis_index], axes_b[axis_index])) < 0.9999
		{
			return;
		}
	}
	face_axis := -1;
	for axis_index in 0 ..< 3
	{
		if abs(util.vector3_dot(manifold.normal, axes_a[axis_index])) > 0.9999
		{
			face_axis = axis_index;
			break;
		}
	}
	if face_axis < 0
	{
		return;
	}
	extents_a := [3]f32{a.half_width, a.half_height, a.half_length};
	extents_b := [3]f32{b.half_width, b.half_height, b.half_length};
	side_0 := (face_axis + 1) % 3;
	side_1 := (face_axis + 2) % 3;
	extent_0 := min(extents_a[side_0], extents_b[side_0]);
	extent_1 := min(extents_a[side_1], extents_b[side_1]);
	face_center_a := util.vector3_add(pose_a.position, util.vector3_scale(manifold.normal, -extents_a[face_axis]));
	face_center_b := util.vector3_add(pose_b.position, util.vector3_scale(manifold.normal, extents_b[face_axis]));
	contact_center := util.vector3_scale(util.vector3_add(face_center_a, face_center_b), 0.5);
	contact_index := 0;
	signs := [2]f32{-1, 1};
	for sign_0 in signs
	{
		for sign_1 in signs
		{
			offset := util.vector3_add(
				util.vector3_scale(axes_a[side_0], sign_0 * extent_0),
				util.vector3_scale(axes_a[side_1], sign_1 * extent_1),
			);
			manifold.contacts[contact_index] = {
				util.vector3_subtract(util.vector3_add(contact_center, offset), pose_a.position),
				manifold.contacts[0].depth,
				i32(contact_index),
			};
			contact_index += 1;
		}
	}
	manifold.count = 4;
}

box_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return box_pair_test_source((^Box)(shape_a)^, (^Box)(shape_b)^, pose_a, pose_b, speculative_margin);
}

box_triangle_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return box_triangle_test_source((^Box)(shape_a)^, (^Triangle)(shape_b)^, pose_a, pose_b, speculative_margin);
}

box_cylinder_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return box_cylinder_test_source(
		(^Box)(shape_a)^, (^Cylinder)(shape_b)^, pose_a, pose_b, speculative_margin, shapes,
	);
}

box_convex_hull_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return box_convex_hull_test_source(
		(^Box)(shape_a)^, (^Convex_Hull)(shape_b), pose_a, pose_b, speculative_margin,
	);
}

triangle_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return triangle_pair_test_source((^Triangle)(shape_a)^, (^Triangle)(shape_b)^, pose_a, pose_b, speculative_margin);
}

triangle_cylinder_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return triangle_cylinder_test_source(
		(^Triangle)(shape_a)^, (^Cylinder)(shape_b)^, pose_a, pose_b, speculative_margin, shapes,
	);
}

triangle_convex_hull_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return triangle_convex_hull_test_source(
		(^Triangle)(shape_a)^, (^Convex_Hull)(shape_b), pose_a, pose_b, speculative_margin,
	);
}

cylinder_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return cylinder_pair_test_source(
		(^Cylinder)(shape_a)^, (^Cylinder)(shape_b)^, pose_a, pose_b, speculative_margin, shapes,
	);
}

cylinder_convex_hull_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return cylinder_convex_hull_test_source(
		(^Cylinder)(shape_a)^, (^Convex_Hull)(shape_b), pose_a, pose_b, speculative_margin, shapes,
	);
}

convex_hull_pair_test :: proc "contextless" (
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose, speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	_ = shapes;
	if shape_a == nil || shape_b == nil || speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	return convex_hull_pair_test_source(
		(^Convex_Hull)(shape_a), (^Convex_Hull)(shape_b), pose_a, pose_b, speculative_margin,
	);
}
