package utilities_bundle_tests

import "core:simd"
import "core:testing"
import entasis "entasis:entasis_utilities"

Bundle_Probe :: struct #align(32)
{
	a: entasis.F32x8,
	b: entasis.I32x8,
	c: entasis.F32x8,
}

Truncated_Probe :: struct
{
	values: [19]i32,
}

@(test)
bundle_indexing_and_masks_cover_counts_zero_through_eight :: proc(t: ^testing.T)
{
	for linear_index in 0 ..< 40
	{
		bundle_index, lane := entasis.bundle_indices(linear_index);
		testing.expect_value(t, bundle_index, linear_index / entasis.PRODUCTION_LANE_COUNT);
		testing.expect_value(t, lane, linear_index % entasis.PRODUCTION_LANE_COUNT);
	}
	for count in 0 ..= entasis.PRODUCTION_LANE_COUNT
	{
		testing.expect_value(t, entasis.bundle_count(count), (count + 7) / 8);
		active, active_status := entasis.bundle_count_mask(count);
		trailing, trailing_status := entasis.bundle_trailing_mask(count);
		testing.expect_value(t, active_status, entasis.Memory_Status.Ok);
		testing.expect_value(t, trailing_status, entasis.Memory_Status.Ok);
		for lane in 0 ..< entasis.PRODUCTION_LANE_COUNT
		{
			expected_active := i32(0);
			expected_trailing := i32(0);
			if lane < count
			{
				expected_active = -1;
			}
			else
			{
				expected_trailing = -1;
			}
			testing.expect_value(t, simd.extract(active, lane), expected_active);
			testing.expect_value(t, simd.extract(trailing, lane), expected_trailing);
		}
		if count == 0
		{
			testing.expect_value(t, entasis.bundle_first_set_lane_index(active), -1);
			testing.expect_value(t, entasis.bundle_last_set_lane_count(active), 0);
		}
		else
		{
			testing.expect_value(t, entasis.bundle_first_set_lane_index(active), 0);
			testing.expect_value(t, entasis.bundle_last_set_lane_count(active), count);
		}
	}
	_, invalid_status := entasis.bundle_count_mask(9);
	testing.expect_value(t, invalid_status, entasis.Memory_Status.Invalid_Count);
}

@(test)
get_offset_and_first_expose_pinned_lane_references :: proc(t: ^testing.T)
{
	bundle := Bundle_Probe{
		a={0, 1, 2, 3, 4, 5, 6, 7},
		b={10, 11, 12, 13, 14, 15, 16, 17},
		c={20, 21, 22, 23, 24, 25, 26, 27},
	};
	value, get_status := entasis.gather_get_checked(&bundle.a, f32, 3);
	testing.expect_value(t, get_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, value^, f32(3));
	first, first_status := entasis.gather_get_first_checked(&bundle.a, f32);
	testing.expect_value(t, first_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, first^, f32(0));
	offset, offset_status := entasis.gather_get_offset_instance_checked(&bundle, 2);
	testing.expect_value(t, offset_status, entasis.Memory_Status.Ok);
	shifted_first, shifted_status := entasis.gather_get_first_checked((^entasis.F32x8)(offset), f32);
	testing.expect_value(t, shifted_status, entasis.Memory_Status.Ok);
	testing.expect_value(t, shifted_first^, f32(2));
	_, invalid_get := entasis.gather_get_checked(&bundle.a, f32, entasis.PRODUCTION_LANE_COUNT);
	testing.expect_value(t, invalid_get, entasis.Memory_Status.Invalid_Count);
	_, invalid_offset := entasis.gather_get_offset_instance_checked(&bundle, -1);
	testing.expect_value(t, invalid_offset, entasis.Memory_Status.Invalid_Count);
}

@(test)
copy_and_clear_preserve_lane_mapping_and_copy_truncation :: proc(t: ^testing.T)
{
	source := Bundle_Probe{
		a={0, 1, 2, 3, 4, 5, 6, 7},
		b={10, 11, 12, 13, 14, 15, 16, 17},
		c={20, 21, 22, 23, 24, 25, 26, 27},
	};
	target := Bundle_Probe{
		a=entasis.F32x8(-1),
		b=entasis.I32x8(-1),
		c=entasis.F32x8(-1),
	};
	testing.expect_value(t, uintptr(&source) & 31, uintptr(0));
	testing.expect_value(t, uintptr(&target) & 31, uintptr(0));
	testing.expect_value(t, entasis.gather_copy_lane_checked(&source, &target, 3, 6), entasis.Memory_Status.Ok);
	testing.expect_value(t, simd.extract(target.a, 6), f32(3));
	testing.expect_value(t, simd.extract(target.b, 6), i32(13));
	testing.expect_value(t, simd.extract(target.c, 6), f32(23));
	testing.expect_value(t, entasis.gather_clear_lane_checked(&target, i32, 6), entasis.Memory_Status.Ok);
	testing.expect_value(t, simd.extract(target.a, 6), f32(0));
	testing.expect_value(t, simd.extract(target.b, 6), i32(0));
	testing.expect_value(t, simd.extract(target.c, 6), f32(0));
	misaligned := (^Bundle_Probe)(rawptr(uintptr(&source) + 4));
	testing.expect_value(
		t,
		entasis.gather_copy_lane_checked(misaligned, &target, 0, 0),
		entasis.Memory_Status.Invalid_Alignment
	);
	testing.expect_value(
		t,
		entasis.gather_clear_lane_checked(&target, i32, entasis.PRODUCTION_LANE_COUNT),
		entasis.Memory_Status.Invalid_Count
	);

	source_memory, source_status := entasis.aligned_allocate(size_of(Truncated_Probe), entasis.PRODUCTION_ALIGNMENT);
	target_memory, target_status := entasis.aligned_allocate(size_of(Truncated_Probe), entasis.PRODUCTION_ALIGNMENT);
	if !testing.expect_value(t, source_status, entasis.Memory_Status.Ok) ||
		!testing.expect_value(t, target_status, entasis.Memory_Status.Ok)
	{
		if source_status == .Ok
		{
			_ = entasis.aligned_release(&source_memory);
		}
		if target_status == .Ok
		{
			_ = entasis.aligned_release(&target_memory);
		}
		return;
	}
	truncated_source := (^Truncated_Probe)(source_memory.memory);
	truncated_target := (^Truncated_Probe)(target_memory.memory);
	for index in 0 ..< len(truncated_source.values)
	{
		truncated_source.values[index] = i32(index + 100);
		truncated_target.values[index] = -1;
	}
	testing.expect_value(
		t,
		entasis.gather_copy_lane_checked(truncated_source, truncated_target, 2, 5),
		entasis.Memory_Status.Ok
	);
	testing.expect_value(t, truncated_target.values[5], i32(102));
	testing.expect_value(t, truncated_target.values[13], i32(110));
	testing.expect_value(t, truncated_target.values[18], i32(-1));
	testing.expect_value(t, entasis.aligned_release(&source_memory), entasis.Memory_Status.Ok);
	testing.expect_value(t, entasis.aligned_release(&target_memory), entasis.Memory_Status.Ok);
}
