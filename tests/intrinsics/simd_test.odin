package intrinsics_tests

import "core:simd"
import simd_x86 "core:simd/x86"
import sysinfo "core:sys/info"
import "core:testing"
import entasis "entasis:entasis_utilities"

F32x4 :: simd.f32x4;
U64x4 :: simd.u64x4;

expect_f32x8 :: proc(t: ^testing.T, actual, expected: entasis.F32x8, tolerance: f32 = 0)
{
	for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
	{
		actual_lane := simd.extract(actual, lane);
		expected_lane := simd.extract(expected, lane);
		difference := abs(actual_lane - expected_lane);
		testing.expectf(
			t,
			difference <= tolerance,
			"lane %d: expected %.9f, got %.9f, difference %.9f",
			lane,
			expected_lane,
			actual_lane,
			difference,
		);
	}
}

expect_f32x4 :: proc(t: ^testing.T, actual, expected: F32x4, tolerance: f32 = 0)
{
	for lane in 0 ..< 4
	{
		actual_lane := simd.extract(actual, lane);
		expected_lane := simd.extract(expected, lane);
		difference := abs(actual_lane - expected_lane);
		testing.expectf(
			t,
			difference <= tolerance,
			"lane %d: expected %.9f, got %.9f",
			lane,
			expected_lane,
			actual_lane
		);
	}
}

@(test)
fixed_width_simd_semantics :: proc(t: ^testing.T)
{
	a := entasis.F32x8{1.25, -2.5, 4.0, 9.0, 16.0, 3.75, -8.0, 0.5};
	b := entasis.F32x8{0.75, 1.5, -2.0, 7.0, 4.0, -1.25, -4.0, 2.5};
	expect_f32x8(t, simd.add(a, b), entasis.F32x8{2, -1, 2, 16, 20, 2.5, -12, 3});
	expect_f32x8(t, simd.sub(a, b), entasis.F32x8{0.5, -4, 6, 2, 12, 5, -4, -2});
	expect_f32x8(t, simd.mul(a, b), entasis.F32x8{0.9375, -3.75, -8, 63, 64, -4.6875, 32, 1.25});
	expect_f32x8(t, simd.min(a, b), entasis.F32x8{0.75, -2.5, -2, 7, 4, -1.25, -8, 0.5});
	expect_f32x8(t, simd.max(a, b), entasis.F32x8{1.25, 1.5, 4, 9, 16, 3.75, -4, 2.5});
	expect_f32x8(t, simd.sqrt(entasis.F32x8{1, 4, 9, 16, 25, 36, 49, 64}), entasis.F32x8{1, 2, 3, 4, 5, 6, 7, 8});
	expect_f32x8(
		t,
		simd.floor(entasis.F32x8{1.9, -1.1, 2.5, -2.5, 0.0, 7.99, -7.01, 3.0}),
		entasis.F32x8{1, -2, 2, -3, 0, 7, -8, 3}
	);

	mask := simd.lanes_lt(a, b);
	testing.expect_value(t, mask, entasis.Mask32x8{0, ~u32(0), 0, 0, 0, 0, ~u32(0), ~u32(0)});
	expect_f32x8(t, simd.select(mask, a, b), entasis.F32x8{0.75, -2.5, -2, 7, 4, -1.25, -8, 0.5});
	testing.expect_value(t, simd.reduce_add_ordered(entasis.I32x8{1, 2, 3, 4, 5, 6, 7, 8}), i32(36));
	testing.expect_value(t, simd.reduce_min(entasis.I32x8{1, -2, 3, 4, -5, 6, 7, 8}), i32(-5));
	testing.expect_value(t, simd.reduce_max(entasis.I32x8{1, -2, 3, 14, -5, 6, 7, 8}), i32(14));
}

@(test, enable_target_feature="sse,sse4.1")
x86_128_semantics :: proc(t: ^testing.T)
{
	a := F32x4{1, 4, 9, 16};
	b := F32x4{2, 3, 10, 8};
	less := simd_x86._mm_cmplt_ps(a, b);
	testing.expect_value(t, simd_x86._mm_movemask_ps(less), u32(0b0101));
	expect_f32x4(t, simd_x86._mm_blendv_ps(a, b, less), F32x4{2, 4, 10, 16});
	expect_f32x4(t, simd_x86._mm_rcp_ps(a), F32x4{1, 0.25, 1.0 / 9.0, 0.0625}, 0.0005);
	expect_f32x4(t, simd_x86._mm_rsqrt_ps(a), F32x4{1, 0.5, 1.0 / 3.0, 0.25}, 0.0005);
}

@(test, enable_target_feature="avx")
x86_256_permutation_and_mask_semantics :: proc(t: ^testing.T)
{
	a := entasis.F32x8{0, 1, 2, 3, 4, 5, 6, 7};
	b := entasis.F32x8{8, 9, 10, 11, 12, 13, 14, 15};

	less := simd_x86._mm256_cmp_ps(a, entasis.F32x8(4), simd_x86._CMP_LT_OQ);
	testing.expect_value(t, simd_x86._mm256_movemask_ps(less), i32(0b00001111));
	expect_f32x8(t, simd_x86._mm256_shuffle_ps(a, b, 0b01_00_11_10), entasis.F32x8{2, 3, 8, 9, 6, 7, 12, 13});
	expect_f32x8(t, simd.shuffle(a, b, 0, 1, 2, 3, 8, 9, 10, 11), entasis.F32x8{0, 1, 2, 3, 8, 9, 10, 11});
	expect_f32x8(t, simd.shuffle(a, b, 4, 5, 6, 7, 12, 13, 14, 15), entasis.F32x8{4, 5, 6, 7, 12, 13, 14, 15});
	expect_f32x8(t, simd_x86._mm256_unpacklo_ps(a, b), entasis.F32x8{0, 8, 1, 9, 4, 12, 5, 13});
	expect_f32x8(t, simd_x86._mm256_unpackhi_ps(a, b), entasis.F32x8{2, 10, 3, 11, 6, 14, 7, 15});

	indices := transmute(simd_x86.__m256i)entasis.I32x8{3, 2, 1, 0, 2, 3, 0, 1};
	expect_f32x8(t, simd_x86._mm256_permutevar_ps(a, indices), entasis.F32x8{3, 2, 1, 0, 6, 7, 4, 5});
}

@(test)
variable_logical_shift_semantics :: proc(t: ^testing.T)
{
	state := u32(0x9e3779b9);
	for iteration in 0 ..< 512
	{
		values_array: [4]u32;
		counts_array: [4]u32;
		expected_array: [4]u32;
		for lane in 0 ..< 4
		{
			state = state * 1664525 + 1013904223;
			values_array[lane] = state;
			state = state * 1664525 + 1013904223;
			counts_array[lane] = state & 63;
			if counts_array[lane] < 32
			{
				expected_array[lane] = values_array[lane] >> counts_array[lane];
			}
		}
		values := simd.from_array(values_array);
		counts := simd.from_array(counts_array);
		expected := simd.from_array(expected_array);
		actual := simd.shr(values, counts);
		testing.expectf(t, actual == expected, "iteration %d: expected %v, got %v", iteration, expected, actual);
	}
}

@(test)
approximate_math_semantics :: proc(t: ^testing.T)
{
	values := entasis.F32x8{1, 2, 3, 4, 5, 8, 16, 64};
	expect_f32x8(
		t,
		entasis.approx_reciprocal_f32x8(values),
		entasis.F32x8{1, 0.5, 1.0 / 3.0, 0.25, 0.2, 0.125, 0.0625, 0.015625},
		0.0005
	);
	expect_f32x8(
		t,
		entasis.approx_reciprocal_sqrt_f32x8(values),
		entasis.F32x8{1, 0.70710677, 0.57735026, 0.5, 0.4472136, 0.35355338, 0.25, 0.125},
		0.0005
	);
}

@(test)
bulk_flag_256_fallback_semantics :: proc(t: ^testing.T)
{
	merged := U64x4{0x01, 0x10, 0x00, 0xf0};
	batch := U64x4{0x03, 0x08, 0x40, 0x30};
	combined := simd.bit_or(merged, batch);
	first_observed := simd.bit_and_not(batch, merged);
	not_equal := simd.bit_xor(simd.lanes_eq(first_observed, U64x4{}), U64x4(~u64(0)));
	testing.expect_value(t, combined, U64x4{0x03, 0x18, 0x40, 0xf0});
	testing.expect_value(t, first_observed, U64x4{0x02, 0x08, 0x40, 0x00});
	testing.expect_value(t, simd.extract_msbs(not_equal), bit_set[0 ..= 3]{0, 1, 2});
}

@(test)
cpu_feature_boundary :: proc(t: ^testing.T)
{
	unsupported_configuration, unsupported_status := entasis.simd_configuration_from_cpu_features({});
	testing.expect_value(t, unsupported_status, entasis.Simd_Status.Unsupported_Hardware);
	testing.expect_value(t, unsupported_configuration.tier, entasis.Simd_Tier.Unsupported);

	avx2_features := sysinfo.CPU_Features{.sse41, .avx, .avx2, .fma, .popcnt};
	avx2_configuration, avx2_status := entasis.simd_configuration_from_cpu_features(avx2_features);
	testing.expect_value(t, avx2_status, entasis.Simd_Status.Ok);
	testing.expect_value(t, avx2_configuration.tier, entasis.Simd_Tier.AVX2);
	testing.expect(t, .FMA in avx2_configuration.features);

	avx512_features := avx2_features + {.avx512f};
	avx512_configuration, avx512_status := entasis.simd_configuration_from_cpu_features(avx512_features);
	testing.expect_value(t, avx512_status, entasis.Simd_Status.Ok);
	testing.expect_value(t, avx512_configuration.tier, entasis.Simd_Tier.AVX512_Bulk);

	host_configuration, host_status := entasis.host_simd_configuration();
	testing.expect_value(t, host_status, entasis.Simd_Status.Ok);
	testing.expect(t, host_configuration.tier == .AVX2 || host_configuration.tier == .AVX512_Bulk);
}
