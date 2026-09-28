// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"
import "core:simd"

SPHERE_TYPE_ID :: 0;
CAPSULE_TYPE_ID :: 1;
BOX_TYPE_ID :: 2;
TRIANGLE_TYPE_ID :: 3;
CYLINDER_TYPE_ID :: 4;
CONVEX_HULL_TYPE_ID :: 5;
COMPOUND_TYPE_ID :: 6;
BIG_COMPOUND_TYPE_ID :: 7;
MESH_TYPE_ID :: 8;
BUILT_IN_SHAPE_TYPE_COUNT :: 9;

Sphere :: struct
{
	radius: f32,
}
Capsule :: struct
{
	radius, half_length: f32,
}
Box :: struct
{
	half_width, half_height, half_length: f32,
}
Triangle :: struct
{
	a, b, c: util.Vector3,
}
Cylinder :: struct
{
	radius, half_length: f32,
}

Shape_Bounds :: struct
{
	min:                       util.Vector3,
	max:                       util.Vector3,
	maximum_radius:            f32,
	maximum_angular_expansion: f32,
}

Sphere_Wide :: struct
{
	radius: util.F32x8,
}
Capsule_Wide :: struct
{
	radius, half_length: util.F32x8,
}
Box_Wide :: struct
{
	half_width, half_height, half_length: util.F32x8,
}
Triangle_Wide :: struct
{
	a, b, c: util.Vector3_Wide,
}
Cylinder_Wide :: struct
{
	radius, half_length: util.F32x8,
}

sphere_wide_write_slot_trusted :: #force_inline proc "contextless" (
	wide: ^Sphere_Wide, lane: int, shape: Sphere,
)
{
	wide.radius = simd.replace(wide.radius, lane, shape.radius);
}

sphere_wide_write_slot :: proc "contextless" (wide: ^Sphere_Wide, lane: int, shape: Sphere) -> Physics_Status
{
	if wide == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	sphere_wide_write_slot_trusted(wide, lane, shape);
	return .Ok;
}

capsule_wide_write_slot_trusted :: #force_inline proc "contextless" (
	wide: ^Capsule_Wide, lane: int, shape: Capsule,
)
{
	wide.radius = simd.replace(wide.radius, lane, shape.radius);
	wide.half_length = simd.replace(wide.half_length, lane, shape.half_length);
}

capsule_wide_write_slot :: proc "contextless" (wide: ^Capsule_Wide, lane: int, shape: Capsule) -> Physics_Status
{
	if wide == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	capsule_wide_write_slot_trusted(wide, lane, shape);
	return .Ok;
}

box_wide_write_slot_trusted :: #force_inline proc "contextless" (
	wide: ^Box_Wide, lane: int, shape: Box,
)
{
	wide.half_width = simd.replace(wide.half_width, lane, shape.half_width);
	wide.half_height = simd.replace(wide.half_height, lane, shape.half_height);
	wide.half_length = simd.replace(wide.half_length, lane, shape.half_length);
}

box_wide_write_slot :: proc "contextless" (wide: ^Box_Wide, lane: int, shape: Box) -> Physics_Status
{
	if wide == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	box_wide_write_slot_trusted(wide, lane, shape);
	return .Ok;
}

triangle_wide_write_slot_trusted :: #force_inline proc "contextless" (
	wide: ^Triangle_Wide, lane: int, shape: Triangle,
)
{
	util.vector3_wide_write_slot(&wide.a, lane, shape.a);
	util.vector3_wide_write_slot(&wide.b, lane, shape.b);
	util.vector3_wide_write_slot(&wide.c, lane, shape.c);
}

triangle_wide_write_slot :: proc "contextless" (wide: ^Triangle_Wide, lane: int, shape: Triangle) -> Physics_Status
{
	if wide == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	triangle_wide_write_slot_trusted(wide, lane, shape);
	return .Ok;
}

cylinder_wide_write_slot_trusted :: #force_inline proc "contextless" (
	wide: ^Cylinder_Wide, lane: int, shape: Cylinder,
)
{
	wide.radius = simd.replace(wide.radius, lane, shape.radius);
	wide.half_length = simd.replace(wide.half_length, lane, shape.half_length);
}

cylinder_wide_write_slot :: proc "contextless" (wide: ^Cylinder_Wide, lane: int, shape: Cylinder) -> Physics_Status
{
	if wide == nil || lane < 0 || lane >= util.PRODUCTION_LANE_COUNT
	{
		return .Invalid_Argument;
	}
	cylinder_wide_write_slot_trusted(wide, lane, shape);
	return .Ok;
}

shape_inertia_from_tensor :: proc "contextless" (tensor: util.Symmetric3x3, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if mass <= 0 || util.symmetric3x3_determinant(tensor) <= 0
	{
		return {}, .Invalid_Argument;
	}
	return {inverse_inertia_tensor=util.symmetric3x3_invert(tensor), inverse_mass=1 / mass}, .Ok;
}

sphere_validate :: proc "contextless" (shape: Sphere) -> Physics_Status
{
	if shape.radius <= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

sphere_bounds :: proc "contextless" (shape: Sphere, orientation: util.Quaternion) -> Shape_Bounds
{
	_ = orientation;
	extent := util.Vector3{shape.radius, shape.radius, shape.radius};
	return {min=util.vector3_negate(extent), max=extent, maximum_radius=shape.radius};
}

sphere_inertia :: proc "contextless" (shape: Sphere, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if sphere_validate(shape) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	inverse_mass := 1 / mass;
	diagonal := inverse_mass / ((2.0 / 5.0) * shape.radius * shape.radius);
	return {inverse_inertia_tensor={diagonal, 0, diagonal, 0, 0, diagonal}, inverse_mass=inverse_mass}, .Ok;
}

sphere_support :: proc "contextless" (shape: Sphere, direction: util.Vector3) -> (util.Vector3, Physics_Status)
{
	length_squared := util.vector3_length_squared(direction);
	if length_squared <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	return util.vector3_scale(direction, shape.radius / math.sqrt(length_squared)), .Ok;
}

capsule_validate :: proc "contextless" (shape: Capsule) -> Physics_Status
{
	if shape.radius <= 0 || shape.half_length <= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

capsule_bounds :: proc "contextless" (shape: Capsule, orientation: util.Quaternion) -> Shape_Bounds
{
	axis := util.quaternion_transform_unit_y(orientation);
	max_bound := util.vector3_add(
		util.vector3_scale(util.vector3_abs(axis), shape.half_length),
		{shape.radius, shape.radius, shape.radius},
	);
	return {
		min=util.vector3_negate(max_bound),
		max=max_bound,
		maximum_radius=shape.half_length + shape.radius,
		maximum_angular_expansion=shape.half_length,
	};
}

capsule_inertia :: proc "contextless" (shape: Capsule, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if capsule_validate(shape) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	inverse_mass := 1 / mass;
	r2 := shape.radius * shape.radius;
	h2 := shape.half_length * shape.half_length;
	cylinder_volume := 2 * shape.half_length * r2 * f32(math.PI);
	sphere_volume := (4.0 / 3.0) * r2 * shape.radius * f32(math.PI);
	inverse_total := 1 / (cylinder_volume + sphere_volume);
	cylinder_fraction := cylinder_volume * inverse_total;
	sphere_fraction := sphere_volume * inverse_total;
	xz := inverse_mass / (
		cylinder_fraction * ((3.0 / 12.0) * r2 + (4.0 / 12.0) * h2) +
		sphere_fraction * ((2.0 / 5.0) * r2 + (6.0 / 8.0) * shape.radius * shape.half_length + h2)
	);
	y := inverse_mass / (cylinder_fraction * 0.5 * r2 + sphere_fraction * (2.0 / 5.0) * r2);
	return {inverse_inertia_tensor={xz, 0, y, 0, 0, xz}, inverse_mass=inverse_mass}, .Ok;
}

capsule_support :: proc "contextless" (shape: Capsule, direction: util.Vector3) -> (util.Vector3, Physics_Status)
{
	length_squared := util.vector3_length_squared(direction);
	if length_squared <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	point := util.vector3_scale(direction, shape.radius / math.sqrt(length_squared));
	if direction.y >= 0
	{
		point.y += shape.half_length;
	}
	else
	{
		point.y -= shape.half_length;
	}
	return point, .Ok;
}

box_validate :: proc "contextless" (shape: Box) -> Physics_Status
{
	if shape.half_width <= 0 || shape.half_height <= 0 || shape.half_length <= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

box_bounds :: proc "contextless" (shape: Box, orientation: util.Quaternion) -> Shape_Bounds
{
	basis := util.matrix3x3_from_quaternion(orientation);
	x := util.vector3_scale(basis.x, shape.half_width);
	y := util.vector3_scale(basis.y, shape.half_height);
	z := util.vector3_scale(basis.z, shape.half_length);
	max_bound := util.vector3_add(util.vector3_add(util.vector3_abs(x), util.vector3_abs(y)), util.vector3_abs(z));
	maximum_radius := math.sqrt(shape.half_width * shape.half_width + shape.half_height * shape.half_height + shape.half_length * shape.half_length);
	return {
		min=util.vector3_negate(max_bound),
		max=max_bound,
		maximum_radius=maximum_radius,
		maximum_angular_expansion=maximum_radius - min(shape.half_width, min(shape.half_height, shape.half_length)),
	};
}

box_inertia :: proc "contextless" (shape: Box, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if box_validate(shape) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	inverse_mass := 1 / mass;
	x2 := shape.half_width * shape.half_width;
	y2 := shape.half_height * shape.half_height;
	z2 := shape.half_length * shape.half_length;
	return {
		inverse_inertia_tensor={
			inverse_mass * 3 / (y2 + z2), 0,
			inverse_mass * 3 / (x2 + z2), 0, 0,
			inverse_mass * 3 / (x2 + y2),
		},
		inverse_mass=inverse_mass,
	}, .Ok;
}

box_support :: proc "contextless" (shape: Box, direction: util.Vector3) -> (util.Vector3, Physics_Status)
{
	if util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	result := util.Vector3{shape.half_width, shape.half_height, shape.half_length};
	if direction.x < 0
	{
		result.x = -result.x;
	}
	if direction.y < 0
	{
		result.y = -result.y;
	}
	if direction.z < 0
	{
		result.z = -result.z;
	}
	return result, .Ok;
}

triangle_validate :: proc "contextless" (shape: Triangle) -> Physics_Status
{
	ab := util.vector3_subtract(shape.b, shape.a);
	ac := util.vector3_subtract(shape.c, shape.a);
	if util.vector3_length_squared(util.vector3_cross(ab, ac)) <= 1e-14
	{
		return .Invalid_Description;
	}
	return .Ok;
}

triangle_bounds :: proc "contextless" (shape: Triangle, orientation: util.Quaternion) -> Shape_Bounds
{
	a := util.quaternion_transform(shape.a, orientation);
	b := util.quaternion_transform(shape.b, orientation);
	c := util.quaternion_transform(shape.c, orientation);
	maximum_radius := math.sqrt(max(
		util.vector3_length_squared(shape.a),
		max(util.vector3_length_squared(shape.b), util.vector3_length_squared(shape.c)),
	));
	return {
		min=util.vector3_min(a, util.vector3_min(b, c)),
		max=util.vector3_max(a, util.vector3_max(b, c)),
		maximum_radius=maximum_radius,
		maximum_angular_expansion=maximum_radius,
	};
}

triangle_inertia_tensor :: proc "contextless" (shape: Triangle, mass: f32) -> util.Symmetric3x3
{
	a, b, c := shape.a, shape.b, shape.c;
	diagonal_scaling := mass * (2.0 / 12.0);
	off_scaling := mass * (2.0 / 24.0);
	return {
		diagonal_scaling * (
			a.y*a.y + a.z*a.z + b.y*b.y + b.z*b.z + c.y*c.y + c.z*c.z +
			a.y*b.y + a.z*b.z + a.y*c.y + b.y*c.y + a.z*c.z + b.z*c.z),
		off_scaling * (-a.y*(b.x+c.x) - b.y*(2*b.x+c.x) - (b.x+2*c.x)*c.y - a.x*(2*a.y+b.y+c.y)),
		diagonal_scaling * (
			a.x*a.x + a.z*a.z + b.x*b.x + b.z*b.z + c.x*c.x + c.z*c.z +
			a.x*b.x + a.z*b.z + a.x*c.x + b.x*c.x + a.z*c.z + b.z*c.z),
		off_scaling * (-a.z*(b.x+c.x) - b.z*(2*b.x+c.x) - (b.x+2*c.x)*c.z - a.x*(2*a.z+b.z+c.z)),
		off_scaling * (-a.z*(b.y+c.y) - b.z*(2*b.y+c.y) - (b.y+2*c.y)*c.z - a.y*(2*a.z+b.z+c.z)),
		diagonal_scaling * (
			a.x*a.x + a.y*a.y + b.x*b.x + b.y*b.y + c.x*c.x + c.y*c.y +
			a.x*b.x + a.y*b.y + a.x*c.x + b.x*c.x + a.y*c.y + b.y*c.y),
	};
}

triangle_inertia :: proc "contextless" (shape: Triangle, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if triangle_validate(shape) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	return shape_inertia_from_tensor(triangle_inertia_tensor(shape, mass), mass);
}

triangle_support :: proc "contextless" (shape: Triangle, direction: util.Vector3) -> (util.Vector3, Physics_Status)
{
	if util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	best := shape.a;
	best_dot := util.vector3_dot(best, direction);
	b_dot := util.vector3_dot(shape.b, direction);
	c_dot := util.vector3_dot(shape.c, direction);
	if b_dot > best_dot
	{
		best = shape.b;
		best_dot = b_dot;
	}
	if c_dot > best_dot
	{
		best = shape.c;
	}
	return best, .Ok;
}

cylinder_validate :: proc "contextless" (shape: Cylinder) -> Physics_Status
{
	if shape.radius <= 0 || shape.half_length <= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

cylinder_bounds :: proc "contextless" (shape: Cylinder, orientation: util.Quaternion) -> Shape_Bounds
{
	y := util.quaternion_transform_unit_y(orientation);
	squared := util.vector3_max({}, util.vector3_subtract({1, 1, 1}, util.vector3_multiply(y, y)));
	disc := util.Vector3{
		math.sqrt(squared.x) * shape.radius,
		math.sqrt(squared.y) * shape.radius,
		math.sqrt(squared.z) * shape.radius,
	};
	max_bound := util.vector3_add(util.vector3_scale(util.vector3_abs(y), shape.half_length), disc);
	maximum_radius := math.sqrt(shape.half_length * shape.half_length + shape.radius * shape.radius);
	return {
		min=util.vector3_negate(max_bound),
		max=max_bound,
		maximum_radius=maximum_radius,
		maximum_angular_expansion=maximum_radius - min(shape.half_length, shape.radius),
	};
}

cylinder_inertia :: proc "contextless" (shape: Cylinder, mass: f32) -> (Body_Inertia, Physics_Status)
{
	if cylinder_validate(shape) != .Ok || mass <= 0
	{
		return {}, .Invalid_Argument;
	}
	inverse_mass := 1 / mass;
	xz := inverse_mass / ((4 * 0.0833333333) * shape.half_length * shape.half_length + 0.25 * shape.radius * shape.radius);
	y := inverse_mass / (0.5 * shape.radius * shape.radius);
	return {inverse_inertia_tensor={xz, 0, y, 0, 0, xz}, inverse_mass=inverse_mass}, .Ok;
}

cylinder_support :: proc "contextless" (shape: Cylinder, direction: util.Vector3) -> (util.Vector3, Physics_Status)
{
	if util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	horizontal_length := math.sqrt(direction.x * direction.x + direction.z * direction.z);
	result: util.Vector3;
	if horizontal_length > 1e-20
	{
		result.x = direction.x * shape.radius / horizontal_length;
		result.z = direction.z * shape.radius / horizontal_length;
	}
	if direction.y >= 0
	{
		result.y = shape.half_length;
	}
	else
	{
		result.y = -shape.half_length;
	}
	return result, .Ok;
}

sphere_wide_bounds :: proc "contextless" (shape: Sphere_Wide) -> (min_bound, max_bound: util.Vector3_Wide)
{
	max_bound = {shape.radius, shape.radius, shape.radius};
	min_bound = util.vector3_wide_negate(max_bound);
	return;
}

capsule_wide_bounds :: proc "contextless" (
	shape: Capsule_Wide, orientation: util.Quaternion_Wide,
) -> (min_bound, max_bound: util.Vector3_Wide)
{
	axis := util.quaternion_wide_transform_unit_y(orientation);
	max_bound = util.vector3_wide_add_scalar(
		util.vector3_wide_abs(util.vector3_wide_scale(axis, shape.half_length)), shape.radius,
	);
	min_bound = util.vector3_wide_negate(max_bound);
	return;
}

box_wide_bounds :: proc "contextless" (
	shape: Box_Wide, orientation: util.Quaternion_Wide,
) -> (min_bound, max_bound: util.Vector3_Wide)
{
	basis := util.matrix3x3_wide_from_quaternion(orientation);
	x := util.vector3_wide_abs(util.vector3_wide_scale(basis.x, shape.half_width));
	y := util.vector3_wide_abs(util.vector3_wide_scale(basis.y, shape.half_height));
	z := util.vector3_wide_abs(util.vector3_wide_scale(basis.z, shape.half_length));
	max_bound = util.vector3_wide_add(util.vector3_wide_add(x, y), z);
	min_bound = util.vector3_wide_negate(max_bound);
	return;
}

triangle_wide_bounds :: proc "contextless" (
	shape: Triangle_Wide, orientation: util.Quaternion_Wide,
) -> (min_bound, max_bound: util.Vector3_Wide)
{
	a := util.quaternion_wide_transform(shape.a, orientation);
	b := util.quaternion_wide_transform(shape.b, orientation);
	c := util.quaternion_wide_transform(shape.c, orientation);
	min_bound = util.vector3_wide_min(a, util.vector3_wide_min(b, c));
	max_bound = util.vector3_wide_max(a, util.vector3_wide_max(b, c));
	return;
}

cylinder_wide_bounds :: proc "contextless" (
	shape: Cylinder_Wide, orientation: util.Quaternion_Wide,
) -> (min_bound, max_bound: util.Vector3_Wide)
{
	y := util.quaternion_wide_transform_unit_y(orientation);
	squared := util.vector3_wide_max(
		{},
		util.vector3_wide_scalar_subtract(util.F32x8(1), util.vector3_wide_multiply(y, y))
	);
	disc := util.Vector3_Wide{
		simd.mul(simd.sqrt(squared.x), shape.radius),
		simd.mul(simd.sqrt(squared.y), shape.radius),
		simd.mul(simd.sqrt(squared.z), shape.radius),
	};
	max_bound = util.vector3_wide_add(util.vector3_wide_abs(util.vector3_wide_scale(y, shape.half_length)), disc);
	min_bound = util.vector3_wide_negate(max_bound);
	return;
}
