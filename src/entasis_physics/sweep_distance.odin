// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

GJK_DISTANCE_TERMINATION_EPSILON :: f32(1e-7);
GJK_DISTANCE_CONTAINMENT_EPSILON :: f32(0.000316227766);
GJK_DISTANCE_MAXIMUM_ITERATIONS :: 64;
Sweep_Distance_Result :: struct
{
	intersected: Reference_State,
	distance:    f32,
	closest_a:   util.Vector3,
	normal:      util.Vector3,
}

sweep_gjk_tetrahedron_contains_origin :: proc "contextless" (
	a, b, c, d: util.Vector3,
) -> Reference_State
{
	face_contains := proc "contextless" (x, y, z, opposite: util.Vector3) -> Reference_State
	{
		normal := util.vector3_cross(util.vector3_subtract(y, x), util.vector3_subtract(z, x));
		origin_dot := util.vector3_dot(normal, util.vector3_negate(x));
		opposite_dot := util.vector3_dot(normal, util.vector3_subtract(opposite, x));
		if origin_dot * opposite_dot >= 0
		{
			return .Present;
		}
		return .Missing;
	}
	if face_contains(a, b, c, d) == .Missing || face_contains(a, c, d, b) == .Missing ||
	face_contains(a, d, b, c) == .Missing || face_contains(b, d, c, a) == .Missing
	{
		return .Missing;
	}
	return .Present;
}

sweep_gjk_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Sweep_Distance_Result, Physics_Status)
{
	if shape_a == nil || shape_b == nil || shapes == nil ||
	type_a < SPHERE_TYPE_ID || type_a > CONVEX_HULL_TYPE_ID ||
	type_b < SPHERE_TYPE_ID || type_b > CONVEX_HULL_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	a, a_status := collision_shape_view(shape_a, type_a, pose_a, shapes);
	if a_status != .Ok
	{
		return {}, a_status;
	}
	b, b_status := collision_shape_view(shape_b, type_b, pose_b, shapes);
	if b_status != .Ok
	{
		return {}, b_status;
	}
	direction := util.vector3_subtract(pose_b.position, pose_a.position);
	if util.vector3_length_squared(direction) <= 1e-20
	{
		direction = {1, 0, 0};
	}
	initial: Depth_Refiner_Vertex;
	support_status: Physics_Status;
	initial, support_status = depth_refiner_support_resolved(a, b, direction, shapes);
	if support_status != .Ok
	{
		return {}, support_status;
	}
	simplex := Depth_Refiner_Simplex{vertices={0=initial}, count=1};
	best_distance_squared := f32(math.F32_MAX);
	best_closest := initial.support;
	best_witness := initial.support_a;
	containment_squared := GJK_DISTANCE_CONTAINMENT_EPSILON * GJK_DISTANCE_CONTAINMENT_EPSILON;
	for _ in 0 ..< GJK_DISTANCE_MAXIMUM_ITERATIONS
	{
		closest := depth_refiner_closest_simplex(&simplex, {});
		depth_refiner_reduce_simplex(&simplex, closest.mask);
		if closest.distance_2 <= containment_squared
		{
			return {
				intersected=.Present, closest_a=closest.witness_a,
				normal=util.vector3_normalize(util.vector3_negate(direction)),
			}, .Ok;
		}
		if closest.distance_2 >= best_distance_squared
		{
			break;
		}
		best_distance_squared = closest.distance_2;
		best_closest = closest.point;
		best_witness = closest.witness_a;
		direction = util.vector3_negate(closest.point);
		next: Depth_Refiner_Vertex;
		next_status: Physics_Status;
		next, next_status = depth_refiner_support_resolved(a, b, direction, shapes);
		if next_status != .Ok
		{
			return {}, next_status;
		}
		progress := util.vector3_dot(util.vector3_subtract(next.support, closest.point), direction);
		maximum_vertex_distance_squared := f32(0);
		for vertex_index in 0 ..< simplex.count
		{
			maximum_vertex_distance_squared = max(
				maximum_vertex_distance_squared,
				util.vector3_length_squared(simplex.vertices[vertex_index].support),
			);
		}
		if progress <= maximum_vertex_distance_squared * GJK_DISTANCE_TERMINATION_EPSILON
		{
			break;
		}
		if simplex.count == 3 && sweep_gjk_tetrahedron_contains_origin(
			simplex.vertices[0].support, simplex.vertices[1].support,
			simplex.vertices[2].support, next.support,
		) == .Present
		{
			return {intersected=.Present, closest_a=best_witness}, .Ok;
		}
		depth_refiner_insert_support(&simplex, next, {});
	}
	distance_without_margin := math.sqrt(best_distance_squared);
	if distance_without_margin <= GJK_DISTANCE_CONTAINMENT_EPSILON
	{
		return {intersected=.Present, closest_a=best_witness}, .Ok;
	}
	normal := util.vector3_scale(best_closest, 1 / distance_without_margin);
	return {
		distance=max(f32(0), distance_without_margin - GJK_DISTANCE_CONTAINMENT_EPSILON),
		closest_a=util.vector3_subtract(best_witness, util.vector3_scale(normal, GJK_DISTANCE_CONTAINMENT_EPSILON)),
		normal=normal,
	}, .Ok;
}

sweep_sphere_pair_distance :: proc "contextless" (
	a: Sphere, b: Sphere, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	offset_b := util.vector3_subtract(pose_b.position, pose_a.position);
	center_distance := util.vector3_length(offset_b);
	normal := util.Vector3{1, 0, 0};
	if center_distance > 1e-12
	{
		normal = util.vector3_scale(offset_b, -1 / center_distance);
	}
	distance := center_distance - a.radius - b.radius;
	intersected := Reference_State.Missing;
	if distance <= 0
	{
		intersected = .Present;
	}
	return {
		intersected=intersected,
		distance=max(f32(0), distance),
		closest_a=util.vector3_add(pose_a.position, util.vector3_scale(normal, -a.radius)),
		normal=normal,
	};
}

sweep_sphere_capsule_distance :: proc "contextless" (
	sphere: Sphere, capsule: Capsule, sphere_pose, capsule_pose: Rigid_Pose,
) -> Sweep_Distance_Result
{
	axis := util.quaternion_transform_unit_y(capsule_pose.orientation);
	to_sphere := util.vector3_subtract(sphere_pose.position, capsule_pose.position);
	t := max(-capsule.half_length, min(capsule.half_length, util.vector3_dot(to_sphere, axis)));
	segment_point := util.vector3_add(capsule_pose.position, util.vector3_scale(axis, t));
	segment_to_sphere := util.vector3_subtract(sphere_pose.position, segment_point);
	center_distance := util.vector3_length(segment_to_sphere);
	normal := util.quaternion_transform_unit_x(capsule_pose.orientation);
	if center_distance > 1e-12
	{
		normal = util.vector3_scale(segment_to_sphere, 1 / center_distance);
	}
	distance := center_distance - sphere.radius - capsule.radius;
	intersected := Reference_State.Missing;
	if distance <= 0
	{
		intersected = .Present;
	}
	return {
		intersected=intersected,
		distance=max(f32(0), distance),
		closest_a=util.vector3_add(sphere_pose.position, util.vector3_scale(normal, -sphere.radius)),
		normal=normal,
	};
}

sweep_distance_from_core_points :: proc "contextless" (
	core_a, core_b: util.Vector3, radius_a: f32, default_normal: util.Vector3,
) -> Sweep_Distance_Result
{
	offset := util.vector3_subtract(core_a, core_b);
	core_distance := util.vector3_length(offset);
	normal := default_normal;
	if core_distance > 1e-12
	{
		normal = util.vector3_scale(offset, 1 / core_distance);
	}
	distance := core_distance - radius_a;
	intersected := Reference_State.Missing;
	if distance <= 0
	{
		intersected = .Present;
	}
	return {
		intersected=intersected, distance=max(f32(0), distance),
		closest_a=util.vector3_subtract(core_a, util.vector3_scale(normal, radius_a)), normal=normal,
	};
}

sweep_sphere_box_distance :: proc "contextless" (
	a: Sphere, b: Box, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	local_center := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	closest := util.Vector3{
		max(-b.half_width, min(b.half_width, local_center.x)),
		max(-b.half_height, min(b.half_height, local_center.y)),
		max(-b.half_length, min(b.half_length, local_center.z)),
	};
	if closest == local_center
	{
		depths := [3]f32{
			b.half_width-abs(local_center.x),
			b.half_height-abs(local_center.y),
			b.half_length-abs(local_center.z),
		};
		axis := 0;
		if depths[1] < depths[axis]
		{
			axis = 1;
		}
		if depths[2] < depths[axis]
		{
			axis = 2;
		}
		if axis == 0
		{
			closest.x = b.half_width;
			if local_center.x < 0
			{
				closest.x = -closest.x;
			}
		}
		else if axis == 1
		{
			closest.y = b.half_height;
			if local_center.y < 0
			{
				closest.y = -closest.y;
			}
		}
		else
		{
			closest.z = b.half_length;
			if local_center.z < 0
			{
				closest.z = -closest.z;
			}
		}
	}
	world_closest := rigid_pose_transform(closest, pose_b);
	return sweep_distance_from_core_points(
		pose_a.position, world_closest, a.radius, util.quaternion_transform_unit_x(pose_b.orientation),
	);
}

sweep_vector3_component :: #force_inline proc "contextless" (value: util.Vector3, component: int) -> f32
{
	if component == 0
	{
		return value.x;
	}
	if component == 1
	{
		return value.y;
	}
	return value.z;
}

sweep_sphere_box_axial_linear :: proc "contextless" (
	sphere: Sphere, box: Box, sphere_pose, box_pose: Rigid_Pose,
	sphere_velocity, box_velocity: Body_Velocity, maximum_t: f32,
) -> (Sweep_Result, bool)
{
	if box_velocity.angular.x != 0 || box_velocity.angular.y != 0 ||
	box_velocity.angular.z != 0 || maximum_t < 0
	{
		return {}, false;
	}
	relative_velocity_world := util.vector3_subtract(
		sphere_velocity.linear, box_velocity.linear,
	);
	inverse_box_orientation := util.quaternion_conjugate(box_pose.orientation);
	local_velocity := util.quaternion_transform(relative_velocity_world, inverse_box_orientation);
	nonzero_count := 0;
	axis := 0;
	if local_velocity.x != 0
	{
		nonzero_count += 1;
		axis = 0;
	}
	if local_velocity.y != 0
	{
		nonzero_count += 1;
		axis = 1;
	}
	if local_velocity.z != 0
	{
		nonzero_count += 1;
		axis = 2;
	}
	if nonzero_count > 1
	{
		return {}, false;
	}
	local_center := rigid_pose_transform_by_inverse(sphere_pose.position, box_pose);
	extents := util.Vector3{box.half_width, box.half_height, box.half_length};
	transverse_distance_squared := f32(0);
	for component in 0 ..< 3
	{
		if nonzero_count != 0 && component == axis
		{
			continue;
		}
		distance := abs(sweep_vector3_component(local_center, component)) -
		sweep_vector3_component(extents, component);
		if distance > 0
		{
			transverse_distance_squared += distance * distance;
		}
	}
	radius_squared := sphere.radius * sphere.radius;
	if transverse_distance_squared > radius_squared
	{
		return {
			state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
		}, true;
	}
	hit_t := f32(0);
	if nonzero_count != 0
	{
		axial_expansion := math.sqrt(max(f32(0), radius_squared - transverse_distance_squared));
		center_axis := sweep_vector3_component(local_center, axis);
		velocity_axis := sweep_vector3_component(local_velocity, axis);
		extent_axis := sweep_vector3_component(extents, axis);
		minimum_axis := -extent_axis - axial_expansion;
		maximum_axis := extent_axis + axial_expansion;
		if center_axis < minimum_axis
		{
			if velocity_axis <= 0
			{
				return {
					state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
				}, true;
			}
			hit_t = (minimum_axis - center_axis) / velocity_axis;
		}
		else if center_axis > maximum_axis
		{
			if velocity_axis >= 0
			{
				return {
					state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
				}, true;
			}
			hit_t = (maximum_axis - center_axis) / velocity_axis;
		}
		if hit_t < 0 || hit_t > maximum_t
		{
			return {
				state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
			}, true;
		}
	}
	else
	{
		closest := util.Vector3{
			max(-box.half_width, min(box.half_width, local_center.x)),
			max(-box.half_height, min(box.half_height, local_center.y)),
			max(-box.half_length, min(box.half_length, local_center.z)),
		};
		if util.vector3_length_squared(util.vector3_subtract(local_center, closest)) > radius_squared
		{
			return {
				state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
			}, true;
		}
	}
	hit_sphere_pose := sphere_pose;
	hit_box_pose := box_pose;
	hit_sphere_pose.position = util.vector3_add(
		sphere_pose.position, util.vector3_scale(sphere_velocity.linear, hit_t),
	);
	hit_box_pose.position = util.vector3_add(
		box_pose.position, util.vector3_scale(box_velocity.linear, hit_t),
	);
	distance := sweep_sphere_box_distance(sphere, box, hit_sphere_pose, hit_box_pose);
	return {
		state=.Hit, t0=hit_t, t1=hit_t, location=distance.closest_a, normal=distance.normal,
		child_a=-1, child_b=-1,
	}, true;
}

sweep_sphere_triangle_distance :: proc "contextless" (
	a: Sphere, b: Triangle, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	vertices := collision_triangle_world_vertices(b, pose_b);
	closest, _ := collision_closest_point_triangle(pose_a.position, vertices[0], vertices[1], vertices[2]);
	default_normal := util.vector3_cross(
		util.vector3_subtract(vertices[1], vertices[0]), util.vector3_subtract(vertices[0], vertices[2]),
	);
	if util.vector3_length_squared(default_normal) > 1e-20
	{
		default_normal = util.vector3_normalize(default_normal);
	}
	else
	{
		default_normal = util.quaternion_transform_unit_y(pose_b.orientation);
	}
	return sweep_distance_from_core_points(pose_a.position, closest, a.radius, default_normal);
}

sweep_sphere_cylinder_distance :: proc "contextless" (
	a: Sphere, b: Cylinder, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	local_center := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	horizontal_length := math.sqrt(local_center.x*local_center.x + local_center.z*local_center.z);
	closest := local_center;
	if horizontal_length > b.radius
	{
		scale := b.radius / horizontal_length;
		closest.x *= scale;
		closest.z *= scale;
	}
	closest.y = max(-b.half_length, min(b.half_length, closest.y));
	if closest == local_center
	{
		cap_depth := b.half_length - abs(local_center.y);
		side_depth := b.radius - horizontal_length;
		if cap_depth <= side_depth
		{
			closest.y = b.half_length;
			if local_center.y < 0
			{
				closest.y = -closest.y;
			}
		}
		else if horizontal_length > 1e-12
		{
			closest.x = local_center.x * b.radius / horizontal_length;
			closest.z = local_center.z * b.radius / horizontal_length;
		}
		else
		{
			closest.x = b.radius;
		}
	}
	return sweep_distance_from_core_points(
		pose_a.position, rigid_pose_transform(closest, pose_b), a.radius,
		util.quaternion_transform_unit_x(pose_b.orientation),
	);
}

sweep_capsule_pair_distance :: proc "contextless" (
	a, b: Capsule, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	axis_a := util.quaternion_transform_unit_y(pose_a.orientation);
	axis_b := util.quaternion_transform_unit_y(pose_b.orientation);
	ta, tb := collision_segment_closest_parameters(
		pose_a.position, axis_a, a.half_length, pose_b.position, axis_b, b.half_length,
	);
	core_a := util.vector3_add(pose_a.position, util.vector3_scale(axis_a, ta));
	core_b := util.vector3_add(pose_b.position, util.vector3_scale(axis_b, tb));
	result := sweep_distance_from_core_points(core_a, core_b, a.radius+b.radius, axis_a);
	result.closest_a = util.vector3_subtract(core_a, util.vector3_scale(result.normal, a.radius));
	return result;
}

sweep_capsule_box_distance :: proc "contextless" (
	a: Capsule, b: Box, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	inverse_b := util.quaternion_conjugate(pose_b.orientation);
	line_origin := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	line_direction := util.quaternion_transform(util.quaternion_transform_unit_y(pose_a.orientation), inverse_b);
	t := max(-a.half_length, min(a.half_length, -util.vector3_dot(line_origin, line_direction)));
	closest: util.Vector3;
	for _ in 0 ..< 16
	{
		point := util.vector3_add(line_origin, util.vector3_scale(line_direction, t));
		closest = {
			max(-b.half_width, min(b.half_width, point.x)),
			max(-b.half_height, min(b.half_height, point.y)),
			max(-b.half_length, min(b.half_length, point.z)),
		};
		new_t := max(-a.half_length, min(a.half_length, util.vector3_dot(
					util.vector3_subtract(closest, line_origin), line_direction,
		)));
		if abs(new_t-t) <= max(f32(1e-7), a.half_length*1e-7)
		{
			t = new_t;
			break;
		}
		t = new_t;
	}
	core_a_local := util.vector3_add(line_origin, util.vector3_scale(line_direction, t));
	if closest == core_a_local
	{
		depths := [3]f32{
			b.half_width-abs(core_a_local.x),
			b.half_height-abs(core_a_local.y),
			b.half_length-abs(core_a_local.z),
		};
		axis := 0;
		if depths[1] < depths[axis]
		{
			axis = 1;
		}
		if depths[2] < depths[axis]
		{
			axis = 2;
		}
		if axis == 0
		{
			closest.x=b.half_width;
			if core_a_local.x<0
			{
				closest.x=-closest.x;
			}
		}
		else if axis == 1
		{
			closest.y=b.half_height;
			if core_a_local.y<0
			{
				closest.y=-closest.y;
			}
		}
		else
		{
			closest.z=b.half_length;
			if core_a_local.z<0
			{
				closest.z=-closest.z;
			}
		}
	}
	core_a := rigid_pose_transform(core_a_local, pose_b);
	core_b := rigid_pose_transform(closest, pose_b);
	return sweep_distance_from_core_points(
		core_a,
		core_b,
		a.radius,
		util.quaternion_transform_unit_x(pose_b.orientation)
	);
}

sweep_capsule_cylinder_distance :: proc "contextless" (
	a: Capsule, b: Cylinder, pose_a, pose_b: Rigid_Pose,
) -> Sweep_Distance_Result
{
	inverse_b := util.quaternion_conjugate(pose_b.orientation);
	line_origin := rigid_pose_transform_by_inverse(pose_a.position, pose_b);
	line_direction := util.quaternion_transform(util.quaternion_transform_unit_y(pose_a.orientation), inverse_b);
	t, offset := capsule_cylinder_closest_line_point(line_origin, line_direction, a.half_length, b);
	core_a_local := util.vector3_add(line_origin, util.vector3_scale(line_direction, t));
	core_b_local := util.vector3_subtract(core_a_local, offset);
	return sweep_distance_from_core_points(
		rigid_pose_transform(core_a_local, pose_b), rigid_pose_transform(core_b_local, pose_b),
		a.radius, util.quaternion_transform_unit_x(pose_b.orientation),
	);
}

sweep_distance_flip :: proc "contextless" (result: Sweep_Distance_Result) -> Sweep_Distance_Result
{
	flipped := result;
	flipped.normal = util.vector3_negate(result.normal);
	flipped.closest_a = util.vector3_add(result.closest_a, util.vector3_scale(result.normal, result.distance));
	return flipped;
}

sweep_convex_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, shapes: ^Shape_Registry,
) -> (Sweep_Distance_Result, Physics_Status)
{
	if type_a == SPHERE_TYPE_ID && type_b == SPHERE_TYPE_ID
	{
		return sweep_sphere_pair_distance((^Sphere)(shape_a)^, (^Sphere)(shape_b)^, pose_a, pose_b), .Ok;
	}
	if type_a == SPHERE_TYPE_ID && type_b == CAPSULE_TYPE_ID
	{
		return sweep_sphere_capsule_distance(
			(^Sphere)(shape_a)^, (^Capsule)(shape_b)^, pose_a, pose_b,
		), .Ok;
	}
	if type_a == CAPSULE_TYPE_ID && type_b == SPHERE_TYPE_ID
	{
		result := sweep_sphere_capsule_distance(
			(^Sphere)(shape_b)^, (^Capsule)(shape_a)^, pose_b, pose_a,
		);
		result.normal = util.vector3_negate(result.normal);
		return result, .Ok;
	}
	if type_a == SPHERE_TYPE_ID
	{
		switch type_b
		{
			case BOX_TYPE_ID:
			return sweep_sphere_box_distance((^Sphere)(shape_a)^, (^Box)(shape_b)^, pose_a, pose_b), .Ok;
			case TRIANGLE_TYPE_ID:
			return sweep_sphere_triangle_distance((^Sphere)(shape_a)^, (^Triangle)(shape_b)^, pose_a, pose_b), .Ok;
			case CYLINDER_TYPE_ID:
			return sweep_sphere_cylinder_distance((^Sphere)(shape_a)^, (^Cylinder)(shape_b)^, pose_a, pose_b), .Ok;
		}
	}
	if type_b == SPHERE_TYPE_ID
	{
		result, status := sweep_convex_distance(shape_b, shape_a, type_b, type_a, pose_b, pose_a, shapes);
		return sweep_distance_flip(result), status;
	}
	if type_a == CAPSULE_TYPE_ID
	{
		switch type_b
		{
			case CAPSULE_TYPE_ID:
			return sweep_capsule_pair_distance((^Capsule)(shape_a)^, (^Capsule)(shape_b)^, pose_a, pose_b), .Ok;
			case BOX_TYPE_ID:
			return sweep_capsule_box_distance((^Capsule)(shape_a)^, (^Box)(shape_b)^, pose_a, pose_b), .Ok;
			case CYLINDER_TYPE_ID:
			return sweep_capsule_cylinder_distance(
				(^Capsule)(shape_a)^,
				(^Cylinder)(shape_b)^,
				pose_a,
				pose_b
			), .Ok;
		}
	}
	if type_b == CAPSULE_TYPE_ID && (type_a == BOX_TYPE_ID || type_a == CYLINDER_TYPE_ID)
	{
		result, status := sweep_convex_distance(shape_b, shape_a, type_b, type_a, pose_b, pose_a, shapes);
		return sweep_distance_flip(result), status;
	}
	return sweep_gjk_distance(shape_a, shape_b, type_a, type_b, pose_a, pose_b, shapes);
}

Sweep_Distance_Wide_Result :: struct
{
	intersected: util.I32x8,
	distance:    util.F32x8,
	closest_a:   util.Vector3_Wide,
	normal:      util.Vector3_Wide,
}

Sweep_Distance_Support_Context_Wide :: struct
{
	shape_a, shape_b:       rawptr,
	type_a, type_b:         int,
	position_a, position_b: util.Vector3_Wide,
	orientation_a:          util.Matrix3x3_Wide,
	orientation_b:          util.Matrix3x3_Wide,
}

sweep_integrate_orientation_wide :: proc "contextless" (
	start: util.Quaternion_Wide, angular_velocity: util.Vector3_Wide, half_times: util.F32x8,
) -> util.Quaternion_Wide
{
	speed := util.vector3_wide_length(angular_velocity);
	integrate := transmute(util.I32x8)simd.lanes_gt(speed, util.F32x8(1e-15));
	// the final lane selection already preserves start for inactive angular motion.
	// avoid trigonometry, division and normalization when every lane is inactive
	if simd.reduce_or(integrate) == 0
	{
		return start;
	}
	half_angle := simd.mul(speed, half_times);
	scale := simd.div(util.sin_approx_wide(half_angle), speed);
	delta := util.Quaternion_Wide{
		x=simd.mul(angular_velocity.x, scale),
		y=simd.mul(angular_velocity.y, scale),
		z=simd.mul(angular_velocity.z, scale),
		w=util.cos_approx_wide(half_angle),
	};
	integrated := util.quaternion_wide_normalize(util.quaternion_wide_concatenate(start, delta));
	return util.quaternion_wide_select(
		integrate, integrated, start,
	);
}

sweep_hull_support_world_wide :: proc "contextless" (
	hull: ^Convex_Hull, orientation: util.Matrix3x3_Wide, position, direction: util.Vector3_Wide,
) -> (util.Vector3_Wide, Physics_Status)
{
	if convex_hull_validate(hull) != .Ok
	{
		return {}, .Invalid_Description;
	}
	best_point: util.Vector3_Wide;
	best_dot := util.F32x8(-f32(math.F32_MAX));
	for point_index in 0 ..< int(hull.point_count)
	{
		local_point := util.vector3_wide_broadcast(convex_hull_get_point(hull, point_index));
		world_point := util.vector3_wide_add(position, util.matrix3x3_wide_transform(local_point, orientation));
		candidate_dot := util.vector3_wide_dot(world_point, direction);
		use_candidate := transmute(util.I32x8)simd.lanes_gt(candidate_dot, best_dot);
		best_dot = util.wide_select_f32(use_candidate, candidate_dot, best_dot);
		best_point = util.vector3_wide_select(use_candidate, world_point, best_point);
	}
	return best_point, .Ok;
}

sweep_shape_support_world_wide :: proc "contextless" (
	shape: rawptr, type_id: int, position: util.Vector3_Wide,
	orientation: util.Matrix3x3_Wide, direction: util.Vector3_Wide,
) -> (util.Vector3_Wide, Physics_Status)
{
	if shape == nil || type_id < SPHERE_TYPE_ID || type_id > CONVEX_HULL_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	switch type_id
	{
		case SPHERE_TYPE_ID:
		sphere := (^Sphere)(shape)^;
		if sphere_validate(sphere) != .Ok
		{
			return {}, .Invalid_Description;
		}
		inverse_length := simd.div(util.F32x8(sphere.radius), util.vector3_wide_length(direction));
		return util.vector3_wide_add(position, util.vector3_wide_scale(direction, inverse_length)), .Ok;
		case CAPSULE_TYPE_ID:
		capsule := (^Capsule)(shape)^;
		if capsule_validate(capsule) != .Ok
		{
			return {}, .Invalid_Description;
		}
		axis := orientation.y;
		use_positive := transmute(util.I32x8)simd.lanes_ge(util.vector3_wide_dot(axis, direction), util.F32x8(0));
		axis_scale := util.wide_select_f32(
			use_positive, util.F32x8(capsule.half_length), util.F32x8(-capsule.half_length),
		);
		cap := util.vector3_wide_scale(axis, axis_scale);
		radius_offset := util.vector3_wide_scale(
			direction, simd.div(util.F32x8(capsule.radius), util.vector3_wide_length(direction)),
		);
		return util.vector3_wide_add(position, util.vector3_wide_add(cap, radius_offset)), .Ok;
		case BOX_TYPE_ID:
		box := (^Box)(shape)^;
		if box_validate(box) != .Ok
		{
			return {}, .Invalid_Description;
		}
		return collision_box_support_world_wide(
			{util.F32x8(box.half_width), util.F32x8(box.half_height), util.F32x8(box.half_length)},
			orientation, position, direction,
		), .Ok;
		case TRIANGLE_TYPE_ID:
		triangle := (^Triangle)(shape)^;
		if triangle_validate(triangle) != .Ok
		{
			return {}, .Invalid_Description;
		}
		vertices := [3]util.Vector3_Wide{
			util.vector3_wide_add(
				position,
				util.matrix3x3_wide_transform(util.vector3_wide_broadcast(triangle.a), orientation)
			),
			util.vector3_wide_add(
				position,
				util.matrix3x3_wide_transform(util.vector3_wide_broadcast(triangle.b), orientation)
			),
			util.vector3_wide_add(
				position,
				util.matrix3x3_wide_transform(util.vector3_wide_broadcast(triangle.c), orientation)
			),
		};
		return collision_triangle_support_world_wide(vertices, direction), .Ok;
		case CYLINDER_TYPE_ID:
		cylinder := (^Cylinder)(shape)^;
		if cylinder_validate(cylinder) != .Ok
		{
			return {}, .Invalid_Description;
		}
		return collision_cylinder_support_world_wide(
			{util.F32x8(cylinder.radius), util.F32x8(cylinder.half_length)}, orientation, position, direction,
		), .Ok;
		case CONVEX_HULL_TYPE_ID:
		return sweep_hull_support_world_wide((^Convex_Hull)(shape), orientation, position, direction);
	}
	return {}, .Not_Found;
}

sweep_sphere_pair_distance_wide :: proc "contextless" (
	a, b: Sphere, offset_b: util.Vector3_Wide,
) -> Sweep_Distance_Wide_Result
{
	center_distance := util.vector3_wide_length(offset_b);
	normal := util.vector3_wide_scale(offset_b, simd.div(util.F32x8(-1), center_distance));
	distance := simd.sub(center_distance, util.F32x8(a.radius + b.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance,
		closest_a=util.vector3_wide_scale(normal, util.F32x8(-a.radius)),
		normal=normal,
	};
}

sweep_sphere_capsule_distance_wide :: proc "contextless" (
	a: Sphere, b: Capsule, offset_b: util.Vector3_Wide, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	axis_b := util.quaternion_wide_transform_unit_y(orientation_b);
	t := simd.max(
		util.F32x8(-b.half_length),
		simd.min(util.F32x8(b.half_length), simd.neg(util.vector3_wide_dot(axis_b, offset_b))),
	);
	sphere_to_segment := util.vector3_wide_add(offset_b, util.vector3_wide_scale(axis_b, t));
	internal_distance := util.vector3_wide_length(sphere_to_segment);
	normal := util.vector3_wide_scale(sphere_to_segment, simd.div(util.F32x8(-1), internal_distance));
	distance := simd.sub(internal_distance, util.F32x8(a.radius + b.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance,
		closest_a=util.vector3_wide_scale(normal, util.F32x8(-a.radius)),
		normal=normal,
	};
}

sweep_sphere_box_distance_wide :: proc "contextless" (
	a: Sphere, b: Box, offset_b: util.Vector3_Wide, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, r_b);
	clamped := util.Vector3_Wide{
		x=simd.min(simd.max(local_offset_b.x, util.F32x8(-b.half_width)), util.F32x8(b.half_width)),
		y=simd.min(simd.max(local_offset_b.y, util.F32x8(-b.half_height)), util.F32x8(b.half_height)),
		z=simd.min(simd.max(local_offset_b.z, util.F32x8(-b.half_length)), util.F32x8(b.half_length)),
	};
	local_normal := util.vector3_wide_subtract(clamped, local_offset_b);
	inner_distance := util.vector3_wide_length(local_normal);
	local_normal = util.vector3_wide_scale(local_normal, simd.div(util.F32x8(1), inner_distance));
	normal := util.matrix3x3_wide_transform(local_normal, r_b);
	distance := simd.sub(inner_distance, util.F32x8(a.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance,
		closest_a=util.vector3_wide_scale(normal, util.F32x8(-a.radius)),
		normal=normal,
	};
}

sweep_sphere_triangle_distance_wide :: proc "contextless" (
	a: Sphere, b: Triangle, offset_b: util.Vector3_Wide, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, r_b);
	triangle_a := util.vector3_wide_broadcast(b.a);
	triangle_b := util.vector3_wide_broadcast(b.b);
	triangle_c := util.vector3_wide_broadcast(b.c);
	ab := util.vector3_wide_subtract(triangle_b, triangle_a);
	ac := util.vector3_wide_subtract(triangle_c, triangle_a);
	pa := util.vector3_wide_add(triangle_a, local_offset_b);
	triangle_normal := util.vector3_wide_cross(ab, ac);
	pa_n := util.vector3_wide_dot(triangle_normal, pa);
	edge_plane_ab := util.vector3_wide_dot(util.vector3_wide_cross(pa, ab), triangle_normal);
	edge_plane_ac := util.vector3_wide_dot(util.vector3_wide_cross(ac, pa), triangle_normal);
	normal_length_squared := util.vector3_wide_length_squared(triangle_normal);
	edge_plane_bc := simd.sub(simd.sub(normal_length_squared, edge_plane_ab), edge_plane_ac);
	outside_ab := transmute(util.I32x8)simd.lanes_lt(edge_plane_ab, util.F32x8(0));
	outside_ac := transmute(util.I32x8)simd.lanes_lt(edge_plane_ac, util.F32x8(0));
	outside_bc := transmute(util.I32x8)simd.lanes_lt(edge_plane_bc, util.F32x8(0));
	outside_any := outside_ab | outside_ac | outside_bc;
	local_closest: util.Vector3_Wide;
	if depth_refiner_any(outside_any) == .Present
	{
		edge_direction := util.vector3_wide_select(outside_ac, ac, ab);
		bc := util.vector3_wide_subtract(triangle_c, triangle_b);
		edge_direction = util.vector3_wide_select(outside_bc, bc, edge_direction);
		edge_start := util.vector3_wide_select(outside_bc, triangle_b, triangle_a);
		negative_edge_start_to_p := util.vector3_wide_add(local_offset_b, edge_start);
		edge_scale := simd.max(
			util.F32x8(0), simd.min(
				util.F32x8(1), simd.div(
					simd.neg(util.vector3_wide_dot(negative_edge_start_to_p, edge_direction)),
					util.vector3_wide_dot(edge_direction, edge_direction),
				),
			),
		);
		point_on_edge := util.vector3_wide_add(edge_start, util.vector3_wide_scale(edge_direction, edge_scale));
		local_closest = util.vector3_wide_select(outside_any, point_on_edge, local_closest);
	}
	if depth_refiner_any(~outside_any) == .Present
	{
		offset_to_plane := util.vector3_wide_scale(triangle_normal, simd.div(pa_n, normal_length_squared));
		point_on_face := util.vector3_wide_subtract(offset_to_plane, local_offset_b);
		local_closest = util.vector3_wide_select(outside_any, local_closest, point_on_face);
	}
	local_normal := util.vector3_wide_add(local_offset_b, local_closest);
	local_normal_length := util.vector3_wide_length(local_normal);
	local_normal = util.vector3_wide_scale(local_normal, simd.div(util.F32x8(-1), local_normal_length));
	normal := util.matrix3x3_wide_transform(local_normal, r_b);
	distance := simd.sub(local_normal_length, util.F32x8(a.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance,
		closest_a=util.vector3_wide_scale(normal, util.F32x8(-a.radius)),
		normal=normal,
	};
}

sweep_sphere_cylinder_distance_wide :: proc "contextless" (
	a: Sphere, b: Cylinder, offset_b: util.Vector3_Wide, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, r_b);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	horizontal_length := simd.sqrt(simd.add(
			simd.mul(local_offset_a.x, local_offset_a.x),
			simd.mul(local_offset_a.z, local_offset_a.z)
	));
	horizontal_scale := simd.div(util.F32x8(b.radius), horizontal_length);
	clamp_horizontal := transmute(util.I32x8)simd.lanes_gt(horizontal_length, util.F32x8(b.radius));
	clamped := util.Vector3_Wide{
		x=util.wide_select_f32(clamp_horizontal, simd.mul(local_offset_a.x, horizontal_scale), local_offset_a.x),
		y=simd.min(util.F32x8(b.half_length), simd.max(util.F32x8(-b.half_length), local_offset_a.y)),
		z=util.wide_select_f32(clamp_horizontal, simd.mul(local_offset_a.z, horizontal_scale), local_offset_a.z),
	};
	sphere_to_closest_local := util.vector3_wide_add(clamped, local_offset_b);
	closest_a := util.matrix3x3_wide_transform(sphere_to_closest_local, r_b);
	contact_distance := util.vector3_wide_length(closest_a);
	normal := util.vector3_wide_scale(closest_a, simd.div(util.F32x8(-1), contact_distance));
	distance := simd.sub(contact_distance, util.F32x8(a.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_lt(distance, util.F32x8(0)),
		distance=distance, closest_a=closest_a, normal=normal,
	};
}

sweep_capsule_pair_distance_wide :: proc "contextless" (
	a, b: Capsule, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	da := util.quaternion_wide_transform_unit_y(orientation_a);
	db := util.quaternion_wide_transform_unit_y(orientation_b);
	da_offset_b := util.vector3_wide_dot(da, offset_b);
	db_offset_b := util.vector3_wide_dot(db, offset_b);
	dadb := util.vector3_wide_dot(da, db);
	ta := simd.div(
		simd.sub(da_offset_b, simd.mul(db_offset_b, dadb)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(dadb, dadb))),
	);
	tb := simd.sub(simd.mul(ta, dadb), db_offset_b);
	abs_dadb := simd.abs(dadb);
	b_onto_a := simd.mul(util.F32x8(b.half_length), abs_dadb);
	a_onto_b := simd.mul(util.F32x8(a.half_length), abs_dadb);
	a_min := simd.max(util.F32x8(-a.half_length), simd.min(util.F32x8(a.half_length), simd.sub(da_offset_b, b_onto_a)));
	a_max := simd.min(util.F32x8(a.half_length), simd.max(util.F32x8(-a.half_length), simd.add(da_offset_b, b_onto_a)));
	b_min := simd.max(
		util.F32x8(-b.half_length),
		simd.min(util.F32x8(b.half_length), simd.sub(simd.neg(a_onto_b), db_offset_b))
	);
	b_max := simd.min(util.F32x8(b.half_length), simd.max(util.F32x8(-b.half_length), simd.sub(a_onto_b, db_offset_b)));
	ta = simd.min(simd.max(ta, a_min), a_max);
	tb = simd.min(simd.max(tb, b_min), b_max);
	closest_a_core := util.vector3_wide_scale(da, ta);
	closest_b_core := util.vector3_wide_add(offset_b, util.vector3_wide_scale(db, tb));
	normal := util.vector3_wide_subtract(closest_a_core, closest_b_core);
	core_distance := util.vector3_wide_length(normal);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), core_distance));
	distance := simd.sub(core_distance, util.F32x8(a.radius + b.radius));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance,
		closest_a=util.vector3_wide_subtract(closest_a_core, util.vector3_wide_scale(normal, util.F32x8(a.radius))),
		normal=normal,
	};
}

sweep_capsule_box_test_edge_wide :: proc "contextless" (
	offset_x, offset_y, offset_z, axis_x, axis_y, axis_z,
	half_width, half_height, half_length: util.F32x8,
) -> (depth, normal_x, normal_y, normal_z: util.F32x8)
{
	length := simd.sqrt(simd.add(simd.mul(axis_x, axis_x), simd.mul(axis_y, axis_y)));
	inverse_length := simd.div(util.F32x8(1), length);
	use_fallback := transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-7));
	normal_x = util.wide_select_f32(use_fallback, util.F32x8(1), simd.neg(simd.mul(axis_y, inverse_length)));
	normal_y = util.wide_select_f32(use_fallback, util.F32x8(0), simd.mul(axis_x, inverse_length));
	normal_z = util.F32x8(0);
	depth = simd.sub(
		simd.add(
			simd.add(simd.mul(simd.abs(normal_x), half_width), simd.mul(simd.abs(normal_y), half_height)),
			simd.mul(simd.abs(normal_z), half_length),
		),
		simd.abs(simd.add(
				simd.add(simd.mul(offset_x, normal_x), simd.mul(offset_y, normal_y)),
				simd.mul(offset_z, normal_z)
		)),
	);
	return;
}

sweep_capsule_box_get_edge_closest_wide :: proc "contextless" (
	normal: ^util.Vector3_Wide, edge_direction_index: util.I32x8, box: Box_Wide,
	offset_a, capsule_axis: util.Vector3_Wide, capsule_half_length: util.F32x8,
) -> util.Vector3_Wide
{
	flip_normal := transmute(util.I32x8)simd.lanes_lt(util.vector3_wide_dot(normal^, offset_a), util.F32x8(0));
	normal^ = util.vector3_wide_conditional_negate(flip_normal, normal^);
	zero := util.F32x8(0);
	edge_center := util.Vector3_Wide{
		x=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_eq(normal.x, zero), zero,
			util.wide_select_f32(
				transmute(util.I32x8)simd.lanes_gt(normal.x, zero),
				box.half_width,
				simd.neg(box.half_width)
			),
		),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_eq(normal.y, zero), zero,
			util.wide_select_f32(
				transmute(util.I32x8)simd.lanes_gt(normal.y, zero),
				box.half_height,
				simd.neg(box.half_height)
			),
		),
		z=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_eq(normal.z, zero), zero,
			util.wide_select_f32(
				transmute(util.I32x8)simd.lanes_gt(normal.z, zero),
				box.half_length,
				simd.neg(box.half_length)
			),
		),
	};
	use_x := transmute(util.I32x8)simd.lanes_eq(edge_direction_index, util.I32x8(0));
	use_y := transmute(util.I32x8)simd.lanes_eq(edge_direction_index, util.I32x8(1));
	use_z := transmute(util.I32x8)simd.lanes_eq(edge_direction_index, util.I32x8(2));
	edge_direction := util.Vector3_Wide{
		x=util.wide_select_f32(use_x, util.F32x8(1), zero),
		y=util.wide_select_f32(use_y, util.F32x8(1), zero),
		z=util.wide_select_f32(use_z, util.F32x8(1), zero),
	};
	ab := util.vector3_wide_subtract(edge_center, offset_a);
	ab_da := util.vector3_wide_dot(ab, capsule_axis);
	ab_db := util.vector3_wide_dot(ab, edge_direction);
	da_db := util.vector3_wide_dot(capsule_axis, edge_direction);
	ta := simd.div(
		simd.sub(ab_da, simd.mul(ab_db, da_db)),
		simd.max(util.F32x8(1e-15), simd.sub(util.F32x8(1), simd.mul(da_db, da_db))),
	);
	edge_half_extent := util.wide_select_f32(
		use_x,
		box.half_width,
		util.wide_select_f32(use_y, box.half_height, box.half_length)
	);
	projected_offset := simd.mul(edge_half_extent, simd.abs(da_db));
	ta_min := simd.max(simd.neg(capsule_half_length), simd.min(capsule_half_length, simd.sub(ab_da, projected_offset)));
	ta_max := simd.min(capsule_half_length, simd.max(simd.neg(capsule_half_length), simd.add(ab_da, projected_offset)));
	ta = simd.min(simd.max(ta, ta_min), ta_max);
	return util.vector3_wide_add(offset_a, util.vector3_wide_scale(capsule_axis, ta));
}

sweep_capsule_box_test_endpoint_wide :: proc "contextless" (
	offset_a, capsule_axis: util.Vector3_Wide, capsule_half_length: util.F32x8,
	endpoint: util.Vector3_Wide, box: Box_Wide,
) -> (depth: util.F32x8, normal: util.Vector3_Wide)
{
	clamped := util.Vector3_Wide{
		x=simd.min(box.half_width, simd.max(simd.neg(box.half_width), endpoint.x)),
		y=simd.min(box.half_height, simd.max(simd.neg(box.half_height), endpoint.y)),
		z=simd.min(box.half_length, simd.max(simd.neg(box.half_length), endpoint.z)),
	};
	normal = util.vector3_wide_subtract(endpoint, clamped);
	length := util.vector3_wide_length(normal);
	normal = util.vector3_wide_scale(normal, simd.div(util.F32x8(1), length));
	ba_n := util.vector3_wide_dot(offset_a, normal);
	da_n := util.vector3_wide_dot(capsule_axis, normal);
	depth = simd.sub(
		simd.add(
			simd.add(
				simd.add(simd.mul(simd.abs(normal.x), box.half_width), simd.mul(simd.abs(normal.y), box.half_height)),
				simd.mul(simd.abs(normal.z), box.half_length),
			),
			simd.abs(simd.mul(da_n, capsule_half_length)),
		),
		simd.abs(ba_n),
	);
	depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_gt(length, util.F32x8(1e-10)), depth, util.F32x8(f32(math.F32_MAX)),
	);
	return;
}

sweep_capsule_box_test_vertex_wide :: proc "contextless" (
	box: Box_Wide, offset_a, capsule_axis: util.Vector3_Wide, capsule_half_length: util.F32x8,
) -> (depth: util.F32x8, normal, closest_a: util.Vector3_Wide)
{
	dot := util.vector3_wide_dot(offset_a, capsule_axis);
	clamped_dot := simd.min(capsule_half_length, simd.max(simd.neg(capsule_half_length), dot));
	closest_on_axis := util.vector3_wide_subtract(offset_a, util.vector3_wide_scale(capsule_axis, clamped_dot));
	vertex := util.Vector3_Wide{
		x=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(closest_on_axis.x, util.F32x8(0)),
			simd.neg(box.half_width),
			box.half_width
		),
		y=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(closest_on_axis.y, util.F32x8(0)),
			simd.neg(box.half_height),
			box.half_height
		),
		z=util.wide_select_f32(
			transmute(util.I32x8)simd.lanes_lt(closest_on_axis.z, util.F32x8(0)),
			simd.neg(box.half_length),
			box.half_length
		),
	};
	center_to_vertex := util.vector3_wide_subtract(vertex, offset_a);
	vertex_dot := util.vector3_wide_dot(center_to_vertex, capsule_axis);
	closest_a = util.vector3_wide_add(offset_a, util.vector3_wide_scale(capsule_axis, vertex_dot));
	vertex_to_capsule := util.vector3_wide_subtract(closest_a, vertex);
	length := util.vector3_wide_length(vertex_to_capsule);
	normal = util.vector3_wide_scale(vertex_to_capsule, simd.div(util.F32x8(1), length));
	depth = simd.sub(
		simd.add(
			simd.add(simd.mul(simd.abs(normal.x), box.half_width), simd.mul(simd.abs(normal.y), box.half_height)),
			simd.mul(simd.abs(normal.z), box.half_length),
		),
		simd.abs(util.vector3_wide_dot(offset_a, normal)),
	);
	depth = util.wide_select_f32(
		transmute(util.I32x8)simd.lanes_lt(length, util.F32x8(1e-10)), util.F32x8(f32(math.F32_MAX)), depth,
	);
	return;
}

sweep_capsule_box_select_wide :: proc "contextless" (
	depth: ^util.F32x8, normal, closest: ^util.Vector3_Wide,
	candidate_depth: util.F32x8, candidate_normal, candidate_closest: util.Vector3_Wide,
)
{
	use_candidate := transmute(util.I32x8)simd.lanes_lt(candidate_depth, depth^);
	depth^ = util.wide_select_f32(use_candidate, candidate_depth, depth^);
	normal^ = util.vector3_wide_select(use_candidate, candidate_normal, normal^);
	closest^ = util.vector3_wide_select(use_candidate, candidate_closest, closest^);
}

sweep_capsule_box_distance_wide :: proc "contextless" (
	a: Capsule, b: Box, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	wide_a := Capsule_Wide{util.F32x8(a.radius), util.F32x8(a.half_length)};
	wide_b := Box_Wide{util.F32x8(b.half_width), util.F32x8(b.half_height), util.F32x8(b.half_length)};
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	world_capsule_axis := util.quaternion_wide_transform_unit_y(orientation_a);
	capsule_axis := util.matrix3x3_wide_transform_transposed(world_capsule_axis, r_b);
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, r_b);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	endpoint_offset := util.vector3_wide_scale(capsule_axis, wide_a.half_length);
	endpoint_0 := util.vector3_wide_subtract(local_offset_a, endpoint_offset);
	depth, local_normal := sweep_capsule_box_test_endpoint_wide(
		local_offset_a, capsule_axis, wide_a.half_length, endpoint_0, wide_b,
	);
	endpoint_1 := util.vector3_wide_add(local_offset_a, endpoint_offset);
	candidate_depth, candidate_normal := sweep_capsule_box_test_endpoint_wide(
		local_offset_a, capsule_axis, wide_a.half_length, endpoint_1, wide_b,
	);
	use_candidate := transmute(util.I32x8)simd.lanes_lt(candidate_depth, depth);
	depth = util.wide_select_f32(use_candidate, candidate_depth, depth);
	local_normal = util.vector3_wide_select(use_candidate, candidate_normal, local_normal);
	endpoint_choice := util.vector3_wide_dot(capsule_axis, local_normal);
	local_closest := util.vector3_wide_select(
		transmute(util.I32x8)simd.lanes_lt(endpoint_choice, util.F32x8(0)), endpoint_1, endpoint_0,
	);
	edge_depth, edge_normal_y, edge_normal_z, edge_normal_x := sweep_capsule_box_test_edge_wide(
		local_offset_a.y, local_offset_a.z, local_offset_a.x,
		capsule_axis.y, capsule_axis.z, capsule_axis.x,
		wide_b.half_height, wide_b.half_length, wide_b.half_width,
	);
	edge_normal := util.Vector3_Wide{edge_normal_x, edge_normal_y, edge_normal_z};
	edge_direction_index := util.I32x8(0);
	edge_candidate_depth, edge_candidate_z, edge_candidate_x, edge_candidate_y := sweep_capsule_box_test_edge_wide(
		local_offset_a.z, local_offset_a.x, local_offset_a.y,
		capsule_axis.z, capsule_axis.x, capsule_axis.y,
		wide_b.half_length, wide_b.half_width, wide_b.half_height,
	);
	edge_candidate_normal := util.Vector3_Wide{edge_candidate_x, edge_candidate_y, edge_candidate_z};
	use_edge_candidate := transmute(util.I32x8)simd.lanes_lt(edge_candidate_depth, edge_depth);
	edge_depth = util.wide_select_f32(use_edge_candidate, edge_candidate_depth, edge_depth);
	edge_normal = util.vector3_wide_select(use_edge_candidate, edge_candidate_normal, edge_normal);
	edge_direction_index = depth_refiner_select_i32(use_edge_candidate, util.I32x8(1), edge_direction_index);
	edge_candidate_depth, edge_candidate_x, edge_candidate_y, edge_candidate_z = sweep_capsule_box_test_edge_wide(
		local_offset_a.x, local_offset_a.y, local_offset_a.z,
		capsule_axis.x, capsule_axis.y, capsule_axis.z,
		wide_b.half_width, wide_b.half_height, wide_b.half_length,
	);
	edge_candidate_normal = {edge_candidate_x, edge_candidate_y, edge_candidate_z};
	use_edge_candidate = transmute(util.I32x8)simd.lanes_lt(edge_candidate_depth, edge_depth);
	edge_depth = util.wide_select_f32(use_edge_candidate, edge_candidate_depth, edge_depth);
	edge_normal = util.vector3_wide_select(use_edge_candidate, edge_candidate_normal, edge_normal);
	edge_direction_index = depth_refiner_select_i32(use_edge_candidate, util.I32x8(2), edge_direction_index);
	if depth_refiner_any(transmute(util.I32x8)simd.lanes_lt(edge_depth, depth)) == .Present
	{
		edge_closest := sweep_capsule_box_get_edge_closest_wide(
			&edge_normal, edge_direction_index, wide_b, local_offset_a, capsule_axis, wide_a.half_length,
		);
		sweep_capsule_box_select_wide(&depth, &local_normal, &local_closest, edge_depth, edge_normal, edge_closest);
	}
	vertex_depth, vertex_normal, vertex_closest := sweep_capsule_box_test_vertex_wide(
		wide_b, local_offset_a, capsule_axis, wide_a.half_length,
	);
	sweep_capsule_box_select_wide(
		&depth, &local_normal, &local_closest, vertex_depth, vertex_normal, vertex_closest,
	);
	normal := util.matrix3x3_wide_transform(local_normal, r_b);
	closest_a := util.vector3_wide_add(offset_b, util.matrix3x3_wide_transform(local_closest, r_b));
	closest_a = util.vector3_wide_subtract(closest_a, util.vector3_wide_scale(normal, wide_a.radius));
	distance := simd.sub(simd.neg(depth), wide_a.radius);
	return {
		intersected=transmute(util.I32x8)simd.lanes_lt(distance, util.F32x8(0)),
		distance=distance, closest_a=closest_a, normal=normal,
	};
}

sweep_capsule_cylinder_distance_wide :: proc "contextless" (
	a: Capsule, b: Cylinder, offset_b: util.Vector3_Wide,
	orientation_a, orientation_b: util.Quaternion_Wide,
) -> Sweep_Distance_Wide_Result
{
	r_a := util.matrix3x3_wide_from_quaternion(orientation_a);
	r_b := util.matrix3x3_wide_from_quaternion(orientation_b);
	local_r_a := util.matrix3x3_wide_multiply_by_transpose(r_a, r_b);
	capsule_axis := local_r_a.y;
	local_offset_b := util.matrix3x3_wide_transform_transposed(offset_b, r_b);
	local_offset_a := util.vector3_wide_negate(local_offset_b);
	t, offset_from_cylinder := capsule_cylinder_closest_line_point_wide(
		local_offset_a, capsule_axis, util.F32x8(a.half_length),
		{util.F32x8(b.radius), util.F32x8(b.half_length)}, {},
	);
	core_distance := util.vector3_wide_length(offset_from_cylinder);
	local_normal := util.vector3_wide_scale(offset_from_cylinder, simd.div(util.F32x8(1), core_distance));
	distance := simd.sub(core_distance, util.F32x8(a.radius));
	local_closest_a := util.vector3_wide_add(local_offset_a, util.vector3_wide_scale(capsule_axis, t));
	local_closest_a = util.vector3_wide_subtract(
		local_closest_a, util.vector3_wide_scale(local_normal, util.F32x8(a.radius)),
	);
	closest_a := util.vector3_wide_add(offset_b, util.matrix3x3_wide_transform(local_closest_a, r_b));
	return {
		intersected=transmute(util.I32x8)simd.lanes_le(distance, util.F32x8(0)),
		distance=distance, closest_a=closest_a,
		normal=util.matrix3x3_wide_transform(local_normal, r_b),
	};
}

sweep_convex_distance_wide :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide,
) -> (Sweep_Distance_Wide_Result, Physics_Status)
{
	if shape_a == nil || shape_b == nil || type_a < SPHERE_TYPE_ID || type_a > CONVEX_HULL_TYPE_ID ||
	type_b < SPHERE_TYPE_ID || type_b > CONVEX_HULL_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	if type_a == SPHERE_TYPE_ID
	{
		a := (^Sphere)(shape_a)^;
		switch type_b
		{
			case SPHERE_TYPE_ID:
			return sweep_sphere_pair_distance_wide(a, (^Sphere)(shape_b)^, offset_b), .Ok;
			case CAPSULE_TYPE_ID:
			return sweep_sphere_capsule_distance_wide(a, (^Capsule)(shape_b)^, offset_b, orientation_b), .Ok;
			case BOX_TYPE_ID:
			return sweep_sphere_box_distance_wide(a, (^Box)(shape_b)^, offset_b, orientation_b), .Ok;
			case TRIANGLE_TYPE_ID:
			return sweep_sphere_triangle_distance_wide(a, (^Triangle)(shape_b)^, offset_b, orientation_b), .Ok;
			case CYLINDER_TYPE_ID:
			return sweep_sphere_cylinder_distance_wide(a, (^Cylinder)(shape_b)^, offset_b, orientation_b), .Ok;
		}
	}
	if type_a == CAPSULE_TYPE_ID && type_b == CAPSULE_TYPE_ID
	{
		return sweep_capsule_pair_distance_wide(
			(^Capsule)(shape_a)^, (^Capsule)(shape_b)^, offset_b, orientation_a, orientation_b,
		), .Ok;
	}
	if type_a == CAPSULE_TYPE_ID && type_b == BOX_TYPE_ID
	{
		return sweep_capsule_box_distance_wide(
			(^Capsule)(shape_a)^, (^Box)(shape_b)^, offset_b, orientation_a, orientation_b,
		), .Ok;
	}
	if type_a == CAPSULE_TYPE_ID && type_b == CYLINDER_TYPE_ID
	{
		return sweep_capsule_cylinder_distance_wide(
			(^Capsule)(shape_a)^, (^Cylinder)(shape_b)^, offset_b, orientation_a, orientation_b,
		), .Ok;
	}
	return sweep_gjk_distance_wide(
		shape_a, shape_b, type_a, type_b, offset_b, orientation_a, orientation_b,
	);
}

sweep_sphere_cast_interval :: proc "contextless" (
	origin, direction: util.Vector3, radius: f32,
) -> (hit: Reference_State, t0, t1: f32)
{
	direction_length := util.vector3_length(direction);
	if direction_length <= 1e-20
	{
		if util.vector3_length_squared(origin) <= radius * radius
		{
			return .Present, 0, f32(math.F32_MAX);
		}
		return .Missing, 0, 0;
	}
	inverse_direction_length := 1 / direction_length;
	d := util.vector3_scale(direction, inverse_direction_length);
	t_offset := max(f32(0), -util.vector3_dot(origin, d) - radius);
	o := util.vector3_add(origin, util.vector3_scale(d, t_offset));
	b := util.vector3_dot(o, d);
	c := util.vector3_dot(o, o) - radius * radius;
	if b > 0 && c > 0
	{
		return .Missing, 0, 0;
	}
	discriminant := b*b - c;
	if discriminant < 0
	{
		return .Missing, 0, 0;
	}
	interval_radius := math.sqrt(discriminant);
	return .Present,
	(t_offset - interval_radius - b) * inverse_direction_length,
	(t_offset + interval_radius - b) * inverse_direction_length;
}

sweep_sample_times_wide :: proc "contextless" (t0, t1: f32) -> util.F32x8
{
	samples: util.F32x8;
	spacing := (t1 - t0) / f32(util.PRODUCTION_LANE_COUNT - 1);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		samples = simd.replace(samples, lane, t0 + f32(lane) * spacing);
	}
	return samples;
}

sweep_construct_samples_wide :: proc "contextless" (
	t0, t1: f32, initial_parent_offset_b, linear_b, angular_a, angular_b: util.Vector3_Wide,
	initial_parent_orientation_a, initial_parent_orientation_b: util.Quaternion_Wide,
	local_pose_a, local_pose_b: Rigid_Pose,
) -> (samples: util.F32x8, offset_b: util.Vector3_Wide, orientation_a, orientation_b: util.Quaternion_Wide,
	child_offset_b: util.Vector3_Wide)
{
	samples = sweep_sample_times_wide(t0, t1);
	offset_b = util.vector3_wide_add(initial_parent_offset_b, util.vector3_wide_scale(linear_b, samples));
	half_samples := simd.mul(samples, util.F32x8(0.5));
	parent_orientation_a := sweep_integrate_orientation_wide(initial_parent_orientation_a, angular_a, half_samples);
	parent_orientation_b := sweep_integrate_orientation_wide(initial_parent_orientation_b, angular_b, half_samples);
	orientation_a = util.quaternion_wide_concatenate(
		util.quaternion_wide_broadcast(local_pose_a.orientation), parent_orientation_a,
	);
	orientation_b = util.quaternion_wide_concatenate(
		util.quaternion_wide_broadcast(local_pose_b.orientation), parent_orientation_b,
	);
	child_position_a := util.quaternion_wide_transform(
		util.vector3_wide_broadcast(local_pose_a.position), parent_orientation_a,
	);
	child_position_b := util.quaternion_wide_transform(
		util.vector3_wide_broadcast(local_pose_b.position), parent_orientation_b,
	);
	child_offset_b = util.vector3_wide_subtract(child_position_b, child_position_a);
	offset_b = util.vector3_wide_add(offset_b, child_offset_b);
	return;
}

sweep_task_test_convex_distance_internal :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	parent_pose_a, parent_pose_b, local_pose_a, local_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, shapes: ^Shape_Registry,
) -> (Sweep_Result, Physics_Status)
{
	if shape_a == nil || shape_b == nil || shapes == nil || maximum_t < 0 ||
	minimum_progression <= 0 || convergence_threshold <= 0 || maximum_iteration_count <= 0
	{
		return {}, .Invalid_Argument;
	}
	if type_a < SPHERE_TYPE_ID || type_a > CONVEX_HULL_TYPE_ID ||
	type_b < SPHERE_TYPE_ID || type_b > CONVEX_HULL_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	shape_orientation_a: util.Quaternion = util.quaternion_concatenate(local_pose_a.orientation, parent_pose_a.orientation);
	shape_orientation_b: util.Quaternion = util.quaternion_concatenate(local_pose_b.orientation, parent_pose_b.orientation);
	bounds_a, bounds_a_status := shapes.batches[type_a].metadata.bounds(shape_a, shape_orientation_a, shapes);
	if bounds_a_status != .Ok
	{
		return {}, bounds_a_status;
	}
	bounds_b, bounds_b_status := shapes.batches[type_b].metadata.bounds(shape_b, shape_orientation_b, shapes);
	if bounds_b_status != .Ok
	{
		return {}, bounds_b_status;
	}
	relative_linear := util.vector3_subtract(velocity_b.linear, velocity_a.linear);
	parent_offset_b := util.vector3_subtract(parent_pose_b.position, parent_pose_a.position);
	world_child_offset_a := util.quaternion_transform(local_pose_a.position, parent_pose_a.orientation);
	world_child_offset_b := util.quaternion_transform(local_pose_b.position, parent_pose_b.orientation);
	offset_b := util.vector3_add(
		parent_offset_b, util.vector3_subtract(world_child_offset_b, world_child_offset_a),
	);
	child_tangent_speed_a := util.vector3_length(util.vector3_cross(world_child_offset_a, velocity_a.angular));
	child_tangent_speed_b := util.vector3_length(util.vector3_cross(world_child_offset_b, velocity_b.angular));
	child_twice_radius_a := 2 * util.vector3_length(local_pose_a.position);
	child_twice_radius_b := 2 * util.vector3_length(local_pose_b.position);
	nonlinear_sphere_expansion := min(
		maximum_t * (child_tangent_speed_a + child_tangent_speed_b),
		child_twice_radius_a + child_twice_radius_b,
	);
	interval_state, lower, upper := sweep_sphere_cast_interval(
		offset_b, relative_linear,
		bounds_a.maximum_radius + bounds_b.maximum_radius + nonlinear_sphere_expansion,
	);
	if interval_state == .Missing || lower > maximum_t || upper < 0
	{
		return {state=.Miss, t0=maximum_t, t1=maximum_t}, .Ok;
	}
	lower = max(f32(0), lower);
	upper = min(maximum_t, upper);
	angular_speed_a := util.vector3_length(velocity_a.angular);
	angular_speed_b := util.vector3_length(velocity_b.angular);
	tangent_speed_a := util.F32x8(bounds_a.maximum_radius * angular_speed_a);
	tangent_speed_b := util.F32x8(bounds_b.maximum_radius * angular_speed_b);
	maximum_angular_expansion_a := util.F32x8(bounds_a.maximum_angular_expansion);
	maximum_angular_expansion_b := util.F32x8(bounds_b.maximum_angular_expansion);
	initial_parent_offset_b := util.vector3_wide_broadcast(parent_offset_b);
	wide_linear_b := util.vector3_wide_broadcast(relative_linear);
	wide_angular_a := util.vector3_wide_broadcast(velocity_a.angular);
	wide_angular_b := util.vector3_wide_broadcast(velocity_b.angular);
	initial_parent_orientation_a := util.quaternion_wide_broadcast(parent_pose_a.orientation);
	initial_parent_orientation_b := util.quaternion_wide_broadcast(parent_pose_b.orientation);
	angular_direction_a := util.Vector3{};
	angular_direction_b := util.Vector3{};
	if angular_speed_a > 1e-8
	{
		angular_direction_a = util.vector3_scale(velocity_a.angular, 1 / angular_speed_a);
	}
	if angular_speed_b > 1e-8
	{
		angular_direction_b = util.vector3_scale(velocity_b.angular, 1 / angular_speed_b);
	}
	wide_angular_direction_a := util.vector3_wide_broadcast(angular_direction_a);
	wide_angular_direction_b := util.vector3_wide_broadcast(angular_direction_b);
	hit_location := util.vector3_add(offset_b, util.vector3_scale(relative_linear, lower));
	hit_distance := util.vector3_length(hit_location);
	hit_normal := util.Vector3{1, 0, 0};
	if hit_distance > 1e-20
	{
		hit_normal = util.vector3_scale(hit_location, -1 / hit_distance);
	}
	hit_location = util.vector3_add(
		hit_location, util.vector3_scale(hit_normal, bounds_b.maximum_radius + nonlinear_sphere_expansion),
	);
	linear_only := velocity_a.angular.x == 0 && velocity_a.angular.y == 0 && velocity_a.angular.z == 0 &&
		velocity_b.angular.x == 0 && velocity_b.angular.y == 0 && velocity_b.angular.z == 0;
	samples, sample_offset_b, sample_orientation_a, sample_orientation_b, fixed_child_offset_b := sweep_construct_samples_wide(
		lower, upper, initial_parent_offset_b, wide_linear_b, wide_angular_a, wide_angular_b,
		initial_parent_orientation_a, initial_parent_orientation_b, local_pose_a, local_pose_b,
	);
	next_lower := lower;
	next_upper := upper;
	intersection_encountered := Reference_State.Missing;
	iteration_index := 0;
	for
	{
		distance, distance_status := sweep_convex_distance_wide(
			shape_a, shape_b, type_a, type_b, sample_offset_b, sample_orientation_a, sample_orientation_b,
		);
		if distance_status != .Ok
		{
			return {}, distance_status;
		}
		linear_velocity_along_normal := util.vector3_wide_dot(distance.normal, wide_linear_b);
		angular_dot_a := util.vector3_wide_dot(distance.normal, wide_angular_direction_a);
		angular_dot_b := util.vector3_wide_dot(distance.normal, wide_angular_direction_b);
		nonlinear_scale_a := simd.sqrt(simd.max(
				util.F32x8(0), simd.sub(util.F32x8(1), simd.mul(angular_dot_a, angular_dot_a)),
		));
		nonlinear_scale_b := simd.sqrt(simd.max(
				util.F32x8(0), simd.sub(util.F32x8(1), simd.mul(angular_dot_b, angular_dot_b)),
		));
		nonlinear_velocity_a := simd.mul(util.F32x8(child_tangent_speed_a), nonlinear_scale_a);
		nonlinear_velocity_b := simd.mul(util.F32x8(child_tangent_speed_b), nonlinear_scale_b);
		nonlinear_displacement_a := simd.mul(util.F32x8(child_twice_radius_a), nonlinear_scale_a);
		nonlinear_displacement_b := simd.mul(util.F32x8(child_twice_radius_b), nonlinear_scale_b);
		a_worst_distance := simd.max(
			util.F32x8(0), simd.sub(simd.sub(distance.distance, maximum_angular_expansion_a), nonlinear_displacement_a),
		);
		angular_displacement_b := simd.add(maximum_angular_expansion_b, nonlinear_displacement_b);
		b_worst_distance := simd.max(util.F32x8(0), simd.sub(distance.distance, angular_displacement_b));
		both_worst_distance := simd.max(util.F32x8(0), simd.sub(a_worst_distance, angular_displacement_b));
		division_guard := util.F32x8(1e-15);
		both_next := simd.div(both_worst_distance, simd.max(division_guard, linear_velocity_along_normal));
		angular_contribution_a := simd.add(nonlinear_velocity_a, tangent_speed_a);
		angular_contribution_b := simd.add(nonlinear_velocity_b, tangent_speed_b);
		a_next := simd.div(
			a_worst_distance, simd.max(division_guard, simd.add(linear_velocity_along_normal, angular_contribution_b)),
		);
		b_next := simd.div(
			b_worst_distance, simd.max(division_guard, simd.add(linear_velocity_along_normal, angular_contribution_a)),
		);
		best_next := simd.div(
			distance.distance,
			simd.max(
				division_guard,
				simd.add(linear_velocity_along_normal, simd.add(angular_contribution_a, angular_contribution_b))
			),
		);
		time_to_next := simd.max(simd.max(both_next, a_next), simd.max(b_next, best_next));
		a_previous := simd.div(
			a_worst_distance, simd.max(division_guard, simd.sub(angular_contribution_b, linear_velocity_along_normal)),
		);
		b_previous := simd.div(
			b_worst_distance, simd.max(division_guard, simd.sub(angular_contribution_a, linear_velocity_along_normal)),
		);
		best_previous := simd.div(
			distance.distance,
			simd.max(
				division_guard,
				simd.sub(simd.add(angular_contribution_a, angular_contribution_b), linear_velocity_along_normal)
			),
		);
		time_to_previous := simd.max(simd.max(simd.neg(both_next), a_previous), simd.max(b_previous, best_previous));
		safe_interval_start := simd.sub(samples, time_to_previous);
		safe_interval_end := simd.add(samples, time_to_next);
		forced_interval_end := simd.add(samples, simd.max(time_to_next, util.F32x8(minimum_progression)));
		if simd.extract(distance.intersected, 0) < 0
		{
			next_upper = simd.extract(samples, 0);
			intersection_encountered = .Present;
		}
		else
		{
			first_intersecting_index := util.PRODUCTION_LANE_COUNT;
			for lane in 0 ..< util.PRODUCTION_LANE_COUNT
			{
				if simd.extract(distance.intersected, lane) < 0
				{
					first_intersecting_index = lane;
					next_upper = simd.extract(samples, lane);
					intersection_encountered = .Present;
					break;
				}
			}
			last_safe_index := 0;
			for lane in 0 ..< first_intersecting_index
			{
				last_safe_index = lane;
				if lane + 1 < first_intersecting_index &&
				simd.extract(safe_interval_start, lane + 1) > simd.extract(forced_interval_end, lane)
				{
					break;
				}
			}
			next_lower = simd.extract(safe_interval_end, last_safe_index);
			hit_normal = util.vector3_wide_read_slot(distance.normal, last_safe_index);
			hit_location = util.vector3_wide_read_slot(distance.closest_a, last_safe_index);
			if intersection_encountered == .Missing
			{
				for lane := util.PRODUCTION_LANE_COUNT - 1; lane >= 0; lane -= 1
				{
					next_upper = simd.extract(safe_interval_start, lane);
					if lane > 0 && simd.extract(forced_interval_end, lane - 1) < next_upper
					{
						break;
					}
				}
			}
		}
		sample_lower := lower + minimum_progression;
		sample_upper := upper - minimum_progression;
		previous_interval_span := upper - lower;
		lower += (next_lower - lower) * 0.9999;
		upper = next_upper;
		interval_span := upper - lower;
		iteration_index += 1;
		if interval_span < 0 ||
		(intersection_encountered == .Present && interval_span < convergence_threshold) ||
		interval_span >= previous_interval_span || iteration_index >= maximum_iteration_count
		{
			break;
		}
		sample_lower = max(lower, min(upper, sample_lower));
		sample_upper = max(lower, min(upper, sample_upper));
		minimum_span := minimum_progression * f32(util.PRODUCTION_LANE_COUNT - 1);
		sample_span := sample_upper - sample_lower;
		if sample_span < minimum_span
		{
			sample_lower = max(lower, sample_lower - (minimum_span - sample_span));
			sample_span = sample_upper - sample_lower;
			if sample_span < minimum_span
			{
				sample_upper += min(minimum_span - sample_span, (upper - sample_upper) * 0.5);
			}
		}
		if linear_only
		{
			// preserve the general sampler's addition order without reconstructing fixed poses
			samples = sweep_sample_times_wide(sample_lower, sample_upper);
			sample_offset_b = util.vector3_wide_add(
				util.vector3_wide_add(initial_parent_offset_b, util.vector3_wide_scale(wide_linear_b, samples)),
				fixed_child_offset_b,
			);
		}
		else
		{
			samples, sample_offset_b, sample_orientation_a, sample_orientation_b, _ = sweep_construct_samples_wide(
				sample_lower, sample_upper, initial_parent_offset_b, wide_linear_b, wide_angular_a, wide_angular_b,
				initial_parent_orientation_a, initial_parent_orientation_b, local_pose_a, local_pose_b,
			);
		}
	}
	if intersection_encountered == .Missing
	{
		return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
	}
	parent_pose_at_a := sweep_pose_at(parent_pose_a, velocity_a, lower);
	hit_location = util.vector3_add(
		hit_location,
		util.vector3_add(
			parent_pose_at_a.position,
			util.quaternion_transform(local_pose_a.position, parent_pose_at_a.orientation),
		),
	);
	return {
		state=.Hit, t0=lower, t1=upper, location=hit_location, normal=hit_normal,
		child_a=-1, child_b=-1,
	}, .Ok;
}

sweep_task_test_convex_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	_ = collision_tasks;
	_ = filter;
	_ = user_context;
	return sweep_task_test_convex_distance_internal(
		shape_a, shape_b, type_a, type_b,
		pose_a, pose_b, rigid_pose_identity(), rigid_pose_identity(), velocity_a, velocity_b,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count, shapes,
	);
}

sweep_compound_get_child :: proc "contextless" (
	shape: rawptr, type_id, child_index: int, pose: Rigid_Pose, shapes: ^Shape_Registry,
	mesh_triangle_storage: ^Triangle,
) -> (Collision_Child, Physics_Status)
{
	if mesh_triangle_storage == nil
	{
		return {}, .Invalid_Argument;
	}
	if type_id == MESH_TYPE_ID
	{
		triangle, child_pose, status := collision_mesh_child((^Mesh)(shape), child_index, pose);
		if status != .Ok
		{
			return {}, status;
		}
		mesh_triangle_storage^ = triangle;
		return {
			shape=mesh_triangle_storage, type_id=TRIANGLE_TYPE_ID, pose=child_pose, child_index=child_index,
		}, .Ok;
	}
	return collision_compound_child(shape, type_id, child_index, pose, shapes);
}

sweep_angular_bounds_expansion :: proc "contextless" (
	angular_speed, maximum_t, maximum_radius, maximum_angular_expansion: f32,
) -> f32
{
	angle := min(angular_speed * maximum_t, f32(math.PI / 3));
	angle_2 := angle * angle;
	angle_4 := angle_2 * angle_2;
	angle_6 := angle_4 * angle_2;
	cosine_minus_one := angle_2 * (-1.0 / 2.0) + angle_4 * (1.0 / 24.0) - angle_6 * (1.0 / 720.0);
	return min(
		maximum_angular_expansion,
		math.sqrt(max(f32(0), -2 * maximum_radius * maximum_radius * cosine_minus_one))
	);
}

sweep_child_bounds_in_compound_space :: proc "contextless" (
	child: Collision_Child, parent_pose_a: Rigid_Pose, velocity_a: Body_Velocity,
	pose_b: Rigid_Pose, velocity_b: Body_Velocity, maximum_t: f32,
	shapes: ^Shape_Registry,
) -> (bounds: util.Bounding_Box, direction: util.Vector3, status: Physics_Status)
{
	if child.shape == nil || shapes == nil || child.type_id < 0 ||
	child.type_id >= shapes.registered_type_count || maximum_t < 0
	{
		return {}, {}, .Invalid_Argument;
	}
	child_batch := &shapes.batches[child.type_id];
	if child_batch.state != .Registered || child_batch.metadata.batch_type != .Convex ||
	shape_batch_bounds_state(child_batch) == .Missing
	{
		return {}, {}, .Invalid_Description;
	}
	inverse_orientation_b := util.quaternion_conjugate(pose_b.orientation);
	local_orientation_a := util.quaternion_concatenate(child.pose.orientation, inverse_orientation_b);
	shape_bounds: Shape_Bounds;
	bounds_status: Physics_Status;
	shape_bounds, bounds_status = shape_batch_compute_bounds(
		child_batch, child.shape, local_orientation_a, shapes,
	);
	if bounds_status != .Ok
	{
		return {}, {}, bounds_status;
	}
	local_position_a := util.quaternion_transform(
		util.vector3_subtract(child.pose.position, pose_b.position), inverse_orientation_b,
	);
	direction = util.quaternion_transform(
		util.vector3_subtract(velocity_a.linear, velocity_b.linear), inverse_orientation_b,
	);
	child_offset_radius := util.vector3_length(util.vector3_subtract(child.pose.position, parent_pose_a.position));
	angular_expansion_a := sweep_angular_bounds_expansion(
		util.vector3_length(velocity_a.angular), maximum_t,
		child_offset_radius + shape_bounds.maximum_radius,
		child_offset_radius + shape_bounds.maximum_angular_expansion,
	);
	parent_offset_radius := util.vector3_length(util.vector3_subtract(parent_pose_a.position, pose_b.position));
	worst_case_radius_b := util.vector3_length(direction) * maximum_t + parent_offset_radius + child_offset_radius;
	angular_expansion_b := sweep_angular_bounds_expansion(
		util.vector3_length(velocity_b.angular), maximum_t,
		worst_case_radius_b + shape_bounds.maximum_radius,
		worst_case_radius_b + shape_bounds.maximum_angular_expansion,
	);
	combined_expansion := util.Vector3{
		angular_expansion_a + angular_expansion_b,
		angular_expansion_a + angular_expansion_b,
		angular_expansion_a + angular_expansion_b,
	};
	return {
		min=util.vector3_subtract(util.vector3_add(shape_bounds.min, local_position_a), combined_expansion),
		max=util.vector3_add(util.vector3_add(shape_bounds.max, local_position_a), combined_expansion),
	}, direction, .Ok;
}

sweep_task_test_registered_child :: proc "contextless" (
	call_context: ^Sweep_Task_Call_Context,
	child_a, child_b: Collision_Child,
	parent_pose_a, parent_pose_b, local_pose_a, local_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
) -> (Sweep_Result, Physics_Status)
{
	miss := Sweep_Result{
		state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1,
	};
	if call_context == nil || call_context.state != .Ready || call_context.registry == nil ||
	child_a.shape == nil || child_b.shape == nil
	{
		return {}, .Invalid_Argument;
	}
	task, reference, lookup_status := sweep_task_registry_lookup(
		call_context.registry, child_a.type_id, child_b.type_id,
	);
	if lookup_status == .Not_Found
	{
		return miss, .Ok;
	}
	if lookup_status != .Ok
	{
		return {}, lookup_status;
	}
	if task.kind == .Contextual
	{
		return sweep_task_test_contextual_child(task.contextual, reference.order,
			child_a, child_b, parent_pose_a, parent_pose_b, local_pose_a, local_pose_b,
			velocity_a, velocity_b, maximum_t, minimum_progression, convergence_threshold,
			maximum_iteration_count, shapes, collision_tasks);
	}
	result: Sweep_Result;
	status: Physics_Status;
	if task.kind == .Built_In
	{
		if reference.order == .Expected
		{
			return sweep_task_test_convex_distance_internal(
				child_a.shape, child_b.shape, child_a.type_id, child_b.type_id,
				parent_pose_a, parent_pose_b, local_pose_a, local_pose_b,
				velocity_a, velocity_b, maximum_t, minimum_progression,
				convergence_threshold, maximum_iteration_count, shapes,
			);
		}
		result, status = sweep_task_test_convex_distance_internal(
			child_b.shape, child_a.shape, child_b.type_id, child_a.type_id,
			parent_pose_b, parent_pose_a, local_pose_b, local_pose_a,
			velocity_b, velocity_a, maximum_t, minimum_progression,
			convergence_threshold, maximum_iteration_count, shapes,
		);
	}
	else if reference.order == .Expected
	{
		return task.child_test(
			child_a.shape, child_b.shape, child_a.type_id, child_b.type_id,
			parent_pose_a, parent_pose_b, local_pose_a, local_pose_b,
			velocity_a, velocity_b,
			maximum_t, minimum_progression, convergence_threshold,
			maximum_iteration_count, shapes, collision_tasks,
		);
	}
	else
	{
		result, status = task.child_test(
			child_b.shape, child_a.shape, child_b.type_id, child_a.type_id,
			parent_pose_b, parent_pose_a, local_pose_b, local_pose_a,
			velocity_b, velocity_a,
			maximum_t, minimum_progression, convergence_threshold,
			maximum_iteration_count, shapes, collision_tasks,
		);
	}
	if status != .Ok
	{
		return {}, status;
	}
	result.normal = util.vector3_negate(result.normal);
	result.child_a, result.child_b = result.child_b, result.child_a;
	return result, .Ok;
}

Sweep_Compound_Tree_Context :: struct
{
	child_a:                    Collision_Child,
	parent_pose_a:              Rigid_Pose,
	parent_pose_b:              Rigid_Pose,
	local_pose_a:               Rigid_Pose,
	shape_b:                    rawptr,
	type_b:                     int,
	pose_b:                     Rigid_Pose,
	velocity_a, velocity_b:     Body_Velocity,
	minimum_progression:        f32,
	convergence_threshold:      f32,
	maximum_iteration_count:    int,
	shapes:                     ^Shape_Registry,
	collision_tasks:            ^Collision_Task_Registry,
	filter:                     Collision_Child_Filter_Proc,
	user_context:               rawptr,
	call_context:               ^Sweep_Task_Call_Context,
	best:                       ^Sweep_Result,
}

sweep_compound_tree_leaf :: proc "contextless" (
	user_context: rawptr, leaf_index: int, maximum_t: ^f32,
) -> Physics_Status
{
	query_context := (^Sweep_Compound_Tree_Context)(user_context);
	if query_context == nil || query_context.best == nil || maximum_t == nil || leaf_index < 0
	{
		return .Invalid_Argument;
	}
	if query_context.filter != nil && query_context.filter(
		query_context.user_context, 0, i32(query_context.child_a.child_index), i32(leaf_index),
	) != .Allow
	{
		return .Ok;
	}
	mesh_triangle_b: Triangle;
	child_b, child_status := sweep_compound_get_child(
		query_context.shape_b, query_context.type_b, leaf_index, query_context.pose_b,
		query_context.shapes, &mesh_triangle_b,
	);
	if child_status != .Ok
	{
		return child_status;
	}
	local_pose_b := rigid_pose_concatenate(child_b.pose, rigid_pose_invert(query_context.parent_pose_b));
	child_result, test_status := sweep_task_test_registered_child(
		query_context.call_context, query_context.child_a, child_b,
		query_context.parent_pose_a, query_context.parent_pose_b,
		query_context.local_pose_a, local_pose_b,
		query_context.velocity_a, query_context.velocity_b,
		maximum_t^, query_context.minimum_progression, query_context.convergence_threshold,
		query_context.maximum_iteration_count, query_context.shapes, query_context.collision_tasks,
	);
	if test_status != .Ok
	{
		return test_status;
	}
	if child_result.state == .Hit &&
	(query_context.best.state == .Miss || child_result.t1 < query_context.best.t1)
	{
		query_context.best^ = child_result;
		query_context.best.child_a = i32(query_context.child_a.child_index);
		query_context.best.child_b = i32(leaf_index);
		maximum_t^ = child_result.t1;
	}
	return .Ok;
}

sweep_task_test_compound_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if shape_a == nil || shape_b == nil || shapes == nil ||
	type_a < SPHERE_TYPE_ID || type_b < SPHERE_TYPE_ID ||
	type_a >= BUILT_IN_SHAPE_TYPE_COUNT || type_b >= BUILT_IN_SHAPE_TYPE_COUNT
	{
		return {}, .Invalid_Argument;
	}
	call_context := (^Sweep_Task_Call_Context)(user_context);
	if call_context == nil || call_context.state != .Ready || call_context.registry == nil
	{
		return {}, .Invalid_Argument;
	}
	if shape_hierarchy_raw_depth(shapes, shape_a, type_a) > 0 || shape_hierarchy_raw_depth(shapes, shape_b, type_b) > 0
	{
		return query_hierarchy_sweep(shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
			maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count, shapes, collision_tasks, filter, user_context);
	}

	a_is_convex := type_a <= CONVEX_HULL_TYPE_ID;
	b_is_convex := type_b <= CONVEX_HULL_TYPE_ID;
	if a_is_convex && b_is_convex
	{
		return sweep_task_test_convex_distance(
			shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
			maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
			shapes, collision_tasks, filter, user_context,
		);
	}
	count_a := 1;
	count_b := 1;
	if !a_is_convex
	{
		count, status := collision_compound_child_count(shape_a, type_a);
		if status != .Ok
		{
			return {}, status;
		}
		count_a = count;
	}
	if !b_is_convex
	{
		count, status := collision_compound_child_count(shape_b, type_b);
		if status != .Ok
		{
			return {}, status;
		}
		count_b = count;
	}
	best := Sweep_Result{state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1};
	for child_index_a in 0 ..< count_a
	{
		mesh_triangle_a: Triangle;
		child_a := Collision_Child{shape=shape_a, type_id=type_a, pose=pose_a, child_index=child_index_a};
		if !a_is_convex
		{
			child, child_status := sweep_compound_get_child(
				shape_a, type_a, child_index_a, pose_a, shapes, &mesh_triangle_a,
			);
			if child_status != .Ok
			{
				return {}, child_status;
			}
			child_a = child;
		}
		local_pose_a := rigid_pose_identity();
		if !a_is_convex
		{
			local_pose_a = rigid_pose_concatenate(child_a.pose, rigid_pose_invert(pose_a));
		}
		if type_b == BIG_COMPOUND_TYPE_ID || type_b == MESH_TYPE_ID
		{
			tree, tree_status := collision_tree_for_shape(shape_b, type_b);
			if tree_status != .Ok
			{
				return {}, tree_status;
			}
			bounds, direction, bounds_status := sweep_child_bounds_in_compound_space(
				child_a, pose_a, velocity_a, pose_b, velocity_b, best.t1, shapes,
			);
			if bounds_status != .Ok
			{
				return {}, bounds_status;
			}
			query_context := Sweep_Compound_Tree_Context{
				child_a=child_a, parent_pose_a=pose_a, parent_pose_b=pose_b,
				local_pose_a=local_pose_a, shape_b=shape_b, type_b=type_b, pose_b=pose_b,
				velocity_a=velocity_a, velocity_b=velocity_b,
				minimum_progression=minimum_progression,
				convergence_threshold=convergence_threshold,
				maximum_iteration_count=maximum_iteration_count,
				shapes=shapes, collision_tasks=collision_tasks, filter=filter,
				user_context=user_context, call_context=call_context, best=&best,
			};
			_, traversal_status := tree_sweep_traverse_closest(
				tree, bounds, direction, best.t1, sweep_compound_tree_leaf, &query_context,
				call_context.pool,
			);
			if traversal_status != .Ok
			{
				return {}, traversal_status;
			}
			continue;
		}
		for child_index_b in 0 ..< count_b
		{
			if filter != nil && filter(user_context, 0, i32(child_index_a), i32(child_index_b)) != .Allow
			{
				continue;
			}
			mesh_triangle_b: Triangle;
			child_b := Collision_Child{shape=shape_b, type_id=type_b, pose=pose_b, child_index=child_index_b};
			if !b_is_convex
			{
				child, child_status := sweep_compound_get_child(
					shape_b, type_b, child_index_b, pose_b, shapes, &mesh_triangle_b,
				);
				if child_status != .Ok
				{
					return {}, child_status;
				}
				child_b = child;
			}
			local_pose_b := rigid_pose_identity();
			if !b_is_convex
			{
				local_pose_b = rigid_pose_concatenate(child_b.pose, rigid_pose_invert(pose_b));
			}
			child_result, child_status := sweep_task_test_registered_child(
				call_context, child_a, child_b,
				pose_a, pose_b, local_pose_a, local_pose_b, velocity_a, velocity_b,
				best.t1, minimum_progression, convergence_threshold, maximum_iteration_count,
				shapes, collision_tasks,
			);
			if child_status != .Ok
			{
				return {}, child_status;
			}
			if child_result.state == .Hit && (best.state == .Miss || child_result.t1 < best.t1)
			{
				best = child_result;
				best.child_a = -1;
				best.child_b = -1;
				if !a_is_convex
				{
					best.child_a = i32(child_index_a);
				}
				if !b_is_convex
				{
					best.child_b = i32(child_index_b);
				}
			}
		}
	}
	return best, .Ok;
}

sweep_task_test_convex_compound_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if type_a > CONVEX_HULL_TYPE_ID || type_b < COMPOUND_TYPE_ID || type_b > BIG_COMPOUND_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	return sweep_task_test_compound_distance(
		shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, filter, user_context,
	);
}

sweep_task_test_convex_homogeneous_compound_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if type_a > CONVEX_HULL_TYPE_ID || type_b != MESH_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	return sweep_task_test_compound_distance(
		shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, filter, user_context,
	);
}

sweep_task_test_compound_pair_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if type_a < COMPOUND_TYPE_ID || type_a > BIG_COMPOUND_TYPE_ID ||
	type_b < COMPOUND_TYPE_ID || type_b > BIG_COMPOUND_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	return sweep_task_test_compound_distance(
		shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, filter, user_context,
	);
}

sweep_task_test_compound_homogeneous_compound_distance :: proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry, collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, user_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if type_a < COMPOUND_TYPE_ID || type_a > BIG_COMPOUND_TYPE_ID || type_b != MESH_TYPE_ID
	{
		return {}, .Invalid_Argument;
	}
	return sweep_task_test_compound_distance(
		shape_a, shape_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, filter, user_context,
	);
}
