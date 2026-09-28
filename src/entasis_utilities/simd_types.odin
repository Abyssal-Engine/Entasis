// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "core:simd"

F32x8 :: simd.f32x8;
I32x8 :: simd.i32x8;
Mask32x8 :: simd.u32x8;

PRODUCTION_LANE_COUNT :: 8;
PRODUCTION_ALIGNMENT :: 32;

Simd_Tier :: enum u8
{
	Unsupported,
	AVX2,
	AVX512_Bulk,
}

Cpu_Feature :: enum u8
{
	SSE41,
	AVX,
	AVX2,
	FMA,
	POPCNT,
	AVX512F,
}

Cpu_Feature_Mask :: distinct bit_set[Cpu_Feature; u16];

Simd_Status :: enum u8
{
	Ok,
	Unsupported_Hardware,
}

Simd_Configuration :: struct
{
	tier:     Simd_Tier,
	features: Cpu_Feature_Mask,
}
