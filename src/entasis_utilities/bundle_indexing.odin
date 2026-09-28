// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"
import "core:simd"

BUNDLE_VECTOR_MASK :: PRODUCTION_LANE_COUNT - 1;
BUNDLE_VECTOR_SHIFT :: 3;

bundle_indices :: proc "contextless" (linear_index: int) -> (bundle_index, index_in_bundle: int)
{
	return linear_index >> BUNDLE_VECTOR_SHIFT, linear_index & BUNDLE_VECTOR_MASK;
}

bundle_count :: proc "contextless" (element_count: int) -> int
{
	return (element_count + BUNDLE_VECTOR_MASK) >> BUNDLE_VECTOR_SHIFT;
}

bundle_trailing_mask :: proc "contextless" (count_in_bundle: int) -> (I32x8, Memory_Status)
{
	if count_in_bundle < 0 || count_in_bundle > PRODUCTION_LANE_COUNT
	{
		return {}, .Invalid_Count;
	}
	indices := F32x8{0, 1, 2, 3, 4, 5, 6, 7};
	return transmute(I32x8)simd.lanes_le(F32x8(f32(count_in_bundle)), indices), .Ok;
}

bundle_count_mask :: proc "contextless" (count_in_bundle: int) -> (I32x8, Memory_Status)
{
	if count_in_bundle < 0 || count_in_bundle > PRODUCTION_LANE_COUNT
	{
		return {}, .Invalid_Count;
	}
	indices := F32x8{0, 1, 2, 3, 4, 5, 6, 7};
	return transmute(I32x8)simd.lanes_gt(F32x8(f32(count_in_bundle)), indices), .Ok;
}

bundle_first_set_lane_index :: proc "contextless" (mask: I32x8) -> int
{
	scalar_mask := u32(transmute(u8)simd.extract_msbs(mask));
	if scalar_mask == 0
	{
		return -1;
	}
	return int(intrinsics.count_trailing_zeros(scalar_mask));
}

bundle_last_set_lane_count :: proc "contextless" (mask: I32x8) -> int
{
	scalar_mask := u32(transmute(u8)simd.extract_msbs(mask));
	if scalar_mask == 0
	{
		return 0;
	}
	return 32 - int(intrinsics.count_leading_zeros(scalar_mask));
}
