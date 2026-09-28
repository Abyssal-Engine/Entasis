package layout_tests

import "core:testing"
import entasis "entasis:entasis_utilities"

@(test)
fixed_simd_layout :: proc(t: ^testing.T)
{
	testing.expect_value(t, size_of(entasis.F32x8), 32);
	testing.expect_value(t, align_of(entasis.F32x8), 32);
	testing.expect_value(t, size_of(entasis.I32x8), 32);
	testing.expect_value(t, size_of(entasis.Mask32x8), 32);
}

@(test)
scalar_math_and_memory_layout :: proc(t: ^testing.T)
{
	testing.expect_value(t, offset_of(entasis.Bounding_Box, min), uintptr(0));
}

@(test)
wide_soa_layout_and_offsets :: proc(t: ^testing.T)
{
	testing.expect_value(t, align_of(entasis.Vector3_Wide), entasis.PRODUCTION_ALIGNMENT);
}
