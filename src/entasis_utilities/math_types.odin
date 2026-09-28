// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Int2 :: struct
{
	x, y: i32,
}
Int3 :: struct
{
	x, y, z: i32,
}
Int4 :: struct
{
	x, y, z, w: i32,
}

Vector2 :: struct
{
	x, y: f32,
}
Vector3 :: struct
{
	x, y, z: f32,
}
Vector4 :: struct
{
	x, y, z, w: f32,
}
Quaternion :: struct
{
	x, y, z, w: f32,
}

Matrix3x3 :: struct
{
	x: Vector3,
	y: Vector3,
	z: Vector3,
}

Matrix :: struct
{
	x: Vector4,
	y: Vector4,
	z: Vector4,
	w: Vector4,
}

Symmetric3x3 :: struct
{
	xx: f32,
	yx: f32,
	yy: f32,
	zx: f32,
	zy: f32,
	zz: f32,
}

Affine_Transform :: struct
{
	translation:      Vector3,
	linear_transform: Matrix3x3,
}

Bounding_Sphere :: struct
{
	center: Vector3,
	radius: f32,
}

Bounding_Box :: struct
{
	min:          Vector3,
	_min_padding: f32,
	max:          Vector3,
	_max_padding: f32,
}

Bounding_Box4 :: struct
{
	min: Vector4,
	max: Vector4,
}

Containment_Type :: enum u8
{
	Disjoint,
	Contains,
	Intersects,
}

Intersection_State :: enum u8
{
	Separate,
	Intersecting,
}

Math_Check_Status :: enum u8
{
	Ok,
	Non_Finite,
}

#assert(size_of(Int2) == 8);
#assert(size_of(Int3) == 12);
#assert(size_of(Int4) == 16);
#assert(size_of(Vector2) == 8);
#assert(size_of(Vector3) == 12);
#assert(size_of(Vector4) == 16);
#assert(size_of(Quaternion) == 16);
#assert(size_of(Matrix3x3) == 36);
#assert(size_of(Matrix) == 64);
#assert(size_of(Symmetric3x3) == 24);
#assert(size_of(Affine_Transform) == 48);
#assert(size_of(Bounding_Sphere) == 16);
#assert(size_of(Bounding_Box) == 32);
#assert(offset_of(Bounding_Box, max) == 16);
#assert(size_of(Bounding_Box4) == 32);
