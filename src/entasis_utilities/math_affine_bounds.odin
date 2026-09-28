// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

affine_identity :: proc "contextless" () -> Affine_Transform
{
	return {linear_transform=matrix3x3_identity()};
}

affine_from_translation :: proc "contextless" (translation: Vector3) -> Affine_Transform
{
	return {translation=translation, linear_transform=matrix3x3_identity()};
}

affine_from_orientation_translation :: proc "contextless" (
	orientation: Quaternion,
	translation: Vector3
) -> Affine_Transform
{
	return {translation=translation, linear_transform=matrix3x3_from_quaternion(orientation)};
}

affine_from_scale_orientation_translation :: proc "contextless" (
	scaling: Vector3,
	orientation: Quaternion,
	translation: Vector3
) -> Affine_Transform
{
	return {
		translation=translation,
		linear_transform=matrix3x3_multiply(matrix3x3_create_scale(scaling), matrix3x3_from_quaternion(orientation)),
	};
}

affine_transform_position :: proc "contextless" (position: Vector3, transform: Affine_Transform) -> Vector3
{
	return vector3_add(matrix3x3_transform(position, transform.linear_transform), transform.translation);
}

affine_invert :: proc "contextless" (transform: Affine_Transform) -> Affine_Transform
{
	linear_inverse := matrix3x3_invert(transform.linear_transform);
	return {
		translation=vector3_negate(matrix3x3_transform(transform.translation, linear_inverse)),
		linear_transform=linear_inverse,
	};
}

affine_invert_rigid :: proc "contextless" (transform: Affine_Transform) -> Affine_Transform
{
	linear_inverse := matrix3x3_transpose(transform.linear_transform);
	return {
		translation=vector3_negate(matrix3x3_transform(transform.translation, linear_inverse)),
		linear_transform=linear_inverse,
	};
}

affine_multiply :: proc "contextless" (a, b: Affine_Transform) -> Affine_Transform
{
	translation := vector3_add(b.translation, matrix3x3_transform(a.translation, b.linear_transform));
	return {translation=translation, linear_transform=matrix3x3_multiply(a.linear_transform, b.linear_transform)};
}

bounding_box_intersection_state :: proc "contextless" (a, b: Bounding_Box) -> Intersection_State
{
	if a.max.x >= b.min.x && a.max.y >= b.min.y && a.max.z >= b.min.z &&
		b.max.x >= a.min.x && b.max.y >= a.min.y && b.max.z >= a.min.z
	{
		return .Intersecting;
	}
	return .Separate;
}

bounding_box_volume :: proc "contextless" (box: Bounding_Box) -> f32
{
	diagonal := vector3_subtract(box.max, box.min);
	return diagonal.x * diagonal.y * diagonal.z;
}

bounding_box_merge :: proc "contextless" (a, b: Bounding_Box) -> Bounding_Box
{
	return Bounding_Box{min=vector3_min(a.min, b.min), max=vector3_max(a.max, b.max)};
}

bounding_box_intersects_sphere :: proc "contextless" (box: Bounding_Box, sphere: Bounding_Sphere) -> Intersection_State
{
	closest := vector3_min(vector3_max(sphere.center, box.min), box.max);
	offset := vector3_subtract(sphere.center, closest);
	if vector3_dot(offset, offset) <= sphere.radius * sphere.radius
	{
		return .Intersecting;
	}
	return .Separate;
}

bounding_box_contains :: proc "contextless" (box, other: Bounding_Box) -> Containment_Type
{
	if box.max.x < other.min.x || box.min.x > other.max.x ||
		box.max.y < other.min.y || box.min.y > other.max.y ||
		box.max.z < other.min.z || box.min.z > other.max.z
	{
		return .Disjoint;
	}
	if box.min.x <= other.min.x && box.max.x >= other.max.x &&
		box.min.y <= other.min.y && box.max.y >= other.max.y &&
		box.min.z <= other.min.z && box.max.z >= other.max.z
	{
		return .Contains;
	}
	return .Intersects;
}

bounding_box_from_points :: proc "contextless" (points: []Vector3) -> (box: Bounding_Box, status: Memory_Status)
{
	if len(points) == 0
	{
		return {}, .Invalid_Count;
	}
	box = Bounding_Box{min=points[0], max=points[0]};
	for index := len(points) - 1; index >= 1; index -= 1
	{
		box.min = vector3_min(points[index], box.min);
		box.max = vector3_max(points[index], box.max);
	}
	return box, .Ok;
}

bounding_box_from_sphere :: proc "contextless" (sphere: Bounding_Sphere) -> Bounding_Box
{
	radius := Vector3{sphere.radius, sphere.radius, sphere.radius};
	return Bounding_Box{min=vector3_subtract(sphere.center, radius), max=vector3_add(sphere.center, radius)};
}
