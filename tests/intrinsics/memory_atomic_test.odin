package intrinsics_tests

import "base:intrinsics"
import "core:mem"
import "core:testing"

@(test)
bit_operation_edges :: proc(t: ^testing.T)
{
	values := [10]u32{0, 1, 2, 3, 4, 7, 8, 0x7fff_ffff, 0x8000_0000, 0xffff_ffff};
	leading := [10]u32{32, 31, 30, 30, 29, 29, 28, 1, 0, 0};
	trailing := [10]u32{32, 0, 1, 0, 2, 0, 3, 0, 31, 0};
	popcounts := [10]u32{0, 1, 1, 2, 1, 3, 1, 31, 1, 32};
	logarithms := [10]u32{0, 0, 1, 1, 2, 2, 3, 30, 31, 31};
	rounded := [10]u32{0, 1, 2, 4, 4, 8, 8, 0x8000_0000, 0x8000_0000, 0};

	for value, index in values
	{
		testing.expect_value(t, intrinsics.count_leading_zeros(value), leading[index]);
		testing.expect_value(t, intrinsics.count_trailing_zeros(value), trailing[index]);
		testing.expect_value(t, intrinsics.count_ones(value), popcounts[index]);

		logarithm := u32(0);
		if value != 0
		{
			logarithm = 31 - intrinsics.count_leading_zeros(value);
		}
		testing.expect_value(t, logarithm, logarithms[index]);

		rounded_value := value;
		if value > 1
		{
			exponent := 32 - intrinsics.count_leading_zeros(value - 1);
			rounded_value = 0;
			if exponent < 32
			{
				rounded_value = u32(1) << exponent;
			}
		}
		testing.expect_value(t, rounded_value, rounded[index]);
	}
}

@(test)
unaligned_copy_zero_and_overlap :: proc(t: ^testing.T)
{
	allocation, allocation_error := mem.alloc(160, 64);
	if !testing.expect(t, allocation_error == nil && allocation != nil)
	{
		return;
	}
	defer testing.expect_value(t, mem.free(allocation), mem.Allocator_Error.None);

	bytes := ([^]u8)(allocation);
	testing.expect_value(t, uintptr(bytes) & 63, uintptr(0));
	intrinsics.unaligned_store((^u32)(&bytes[1]), 0xa1b2_c3d4);
	testing.expect_value(t, intrinsics.unaligned_load((^u32)(&bytes[1])), u32(0xa1b2_c3d4));

	for index in 0 ..< 32
	{
		bytes[index] = u8(index);
	}
	intrinsics.mem_copy(&bytes[1], bytes, 31);
	for index in 1 ..< 32
	{
		testing.expect_value(t, bytes[index], u8(index - 1));
	}

	intrinsics.mem_zero(bytes, 32);
	for index in 0 ..< 32
	{
		testing.expect_value(t, bytes[index], u8(0));
	}
}

@(test)
atomic_and_volatile_semantics :: proc(t: ^testing.T)
{
	value := i32(10);
	testing.expect_value(t, intrinsics.atomic_add_explicit(&value, 5, .Seq_Cst), i32(10));
	testing.expect_value(t, value, i32(15));
	testing.expect_value(t, intrinsics.atomic_add_explicit(&value, 1, .Seq_Cst), i32(15));
	testing.expect_value(t, value, i32(16));
	testing.expect_value(t, intrinsics.atomic_sub_explicit(&value, 1, .Seq_Cst), i32(16));
	testing.expect_value(t, value, i32(15));
	previous, _ := intrinsics.atomic_compare_exchange_strong_explicit(&value, 15, 99, .Seq_Cst, .Seq_Cst);
	testing.expect_value(t, previous, i32(15));
	testing.expect_value(t, value, i32(99));
	previous, _ = intrinsics.atomic_compare_exchange_strong_explicit(&value, 15, 7, .Seq_Cst, .Seq_Cst);
	testing.expect_value(t, previous, i32(99));
	testing.expect_value(t, value, i32(99));
	intrinsics.atomic_store_explicit(&value, 42, .Release);
	testing.expect_value(t, intrinsics.atomic_load_explicit(&value, .Acquire), i32(42));
}
