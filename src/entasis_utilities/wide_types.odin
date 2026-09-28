// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

Vector2_Wide :: struct
{
	x, y: F32x8,
}
Vector3_Wide :: struct
{
	x, y, z: F32x8,
}
Vector4_Wide :: struct
{
	x, y, z, w: F32x8,
}
Quaternion_Wide :: struct
{
	x, y, z, w: F32x8,
}

Matrix2x2_Wide :: struct
{
	x, y: Vector2_Wide,
}
Matrix2x3_Wide :: struct
{
	x, y: Vector3_Wide,
}
Matrix3x3_Wide :: struct
{
	x, y, z: Vector3_Wide,
}

Symmetric2x2_Wide :: struct
{
	xx, yx, yy: F32x8,
}
Symmetric3x3_Wide :: struct
{
	xx, yx, yy, zx, zy, zz: F32x8,
}
Symmetric4x4_Wide :: struct
{
	xx, yx, yy, zx, zy, zz, wx, wy, wz, ww: F32x8,
}
Symmetric5x5_Wide :: struct
{
	a: Symmetric3x3_Wide, b: Matrix2x3_Wide, d: Symmetric2x2_Wide,
}
Symmetric6x6_Wide :: struct
{
	a: Symmetric3x3_Wide, b: Matrix3x3_Wide, d: Symmetric3x3_Wide,
}

#assert(align_of(Vector2_Wide) == PRODUCTION_ALIGNMENT);
#assert(size_of(Vector2_Wide) == 64);
#assert(size_of(Vector3_Wide) == 96);
#assert(size_of(Vector4_Wide) == 128);
#assert(size_of(Quaternion_Wide) == 128);
#assert(size_of(Matrix2x2_Wide) == 128);
#assert(size_of(Matrix2x3_Wide) == 192);
#assert(size_of(Matrix3x3_Wide) == 288);
#assert(size_of(Symmetric2x2_Wide) == 96);
#assert(size_of(Symmetric3x3_Wide) == 192);
#assert(size_of(Symmetric4x4_Wide) == 320);
#assert(size_of(Symmetric5x5_Wide) == 480);
#assert(size_of(Symmetric6x6_Wide) == 672);
#assert(offset_of(Vector3_Wide, y) == 32);
#assert(offset_of(Vector3_Wide, z) == 64);
#assert(offset_of(Matrix3x3_Wide, y) == 96);
#assert(offset_of(Matrix3x3_Wide, z) == 192);
#assert(offset_of(Symmetric5x5_Wide, b) == 192);
#assert(offset_of(Symmetric5x5_Wide, d) == 384);
#assert(offset_of(Symmetric6x6_Wide, b) == 192);
#assert(offset_of(Symmetric6x6_Wide, d) == 480);
