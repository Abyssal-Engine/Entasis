// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import simd_x86 "core:simd/x86"

@(enable_target_feature="avx")
approx_reciprocal_f32x8 :: proc "contextless" (value: F32x8) -> F32x8
{
	return simd_x86._mm256_rcp_ps(value);
}

@(enable_target_feature="avx")
approx_reciprocal_sqrt_f32x8 :: proc "contextless" (value: F32x8) -> F32x8
{
	return simd_x86._mm256_rsqrt_ps(value);
}
