// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_utilities

import "base:intrinsics"
import "core:simd"

Sort_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Capacity_Missing,
	Unsupported,
}

Sorted_Buffer :: enum u8
{
	Input,
	Scratch,
}

sort_swap :: proc "contextless" (a, b: ^$T)
{
	temporary := a^;
	a^ = b^;
	b^ = temporary;
}

insertion_sort_keys_values :: proc "contextless" (
	keys: [^]$K, values: [^]$V, start, inclusive_end: int, comparer: Comparer_Procs,
) -> Sort_Status
{
	if keys == nil || values == nil || comparer.compare == nil || start < 0 || inclusive_end < start
	{
		return .Invalid_Argument;
	}
	for index := start + 1; index <= inclusive_end; index += 1
	{
		original_key := keys[index];
		original_value := values[index];
		compare_index := index - 1;
		for compare_index >= start
		{
			if comparer.compare(&original_key, &keys[compare_index], comparer.comparer_context) >= 0
			{
				break;
			}
			keys[compare_index + 1] = keys[compare_index];
			values[compare_index + 1] = values[compare_index];
			compare_index -= 1;
		}
		target_index := compare_index + 1;
		if target_index != index
		{
			keys[target_index] = original_key;
			values[target_index] = original_value;
		}
	}
	return .Ok;
}

insertion_sort_keys :: proc "contextless" (
	keys: [^]$K, start, inclusive_end: int, comparer: Comparer_Procs,
) -> Sort_Status
{
	if keys == nil || comparer.compare == nil || start < 0 || inclusive_end < start
	{
		return .Invalid_Argument;
	}
	for index := start + 1; index <= inclusive_end; index += 1
	{
		original_key := keys[index];
		compare_index := index - 1;
		for compare_index >= start
		{
			if comparer.compare(&original_key, &keys[compare_index], comparer.comparer_context) >= 0
			{
				break;
			}
			keys[compare_index + 1] = keys[compare_index];
			compare_index -= 1;
		}
		keys[compare_index + 1] = original_key;
	}
	return .Ok;
}

quick_sort_find_median_of_three :: proc "contextless" (
	keys: [^]$K, left, right: int, comparer: Comparer_Procs,
) -> int
{
	middle := (left + right) / 2;
	if comparer.compare(&keys[left], &keys[middle], comparer.comparer_context) <= 0 &&
		comparer.compare(&keys[left], &keys[right], comparer.comparer_context) <= 0
	{
		if comparer.compare(&keys[middle], &keys[right], comparer.comparer_context) <= 0
		{
			return middle;
		}
		return right;
	}
	if comparer.compare(&keys[middle], &keys[right], comparer.comparer_context) <= 0
	{
		if comparer.compare(&keys[left], &keys[right], comparer.comparer_context) <= 0
		{
			return left;
		}
		return right;
	}
	if comparer.compare(&keys[left], &keys[middle], comparer.comparer_context) <= 0
	{
		return left;
	}
	return middle;
}

quick_sort_keys_values :: proc "contextless" (
	keys: [^]$K, values: [^]$V, left, right: int, comparer: Comparer_Procs,
) -> Sort_Status
{
	if keys == nil || values == nil || comparer.compare == nil || left < 0 || right < left
	{
		return .Invalid_Argument;
	}
	if right - left <= 30
	{
		return insertion_sort_keys_values(keys, values, left, right, comparer);
	}
	pivot_index := quick_sort_find_median_of_three(keys, left, right, comparer);
	pivot := keys[pivot_index];
	pivot_value := values[pivot_index];
	keys[pivot_index] = keys[right];
	values[pivot_index] = values[right];
	i := left - 1;
	j := right;
	for
	{
		for
		{
			i += 1;
			if comparer.compare(&keys[i], &pivot, comparer.comparer_context) >= 0
			{
				break;
			}
		}
		for
		{
			j -= 1;
			if comparer.compare(&pivot, &keys[j], comparer.comparer_context) >= 0 || j <= i
			{
				break;
			}
		}
		if i >= j
		{
			break;
		}
		sort_swap(&keys[i], &keys[j]);
		sort_swap(&values[i], &values[j]);
	}
	keys[right] = keys[i];
	values[right] = values[i];
	keys[i] = pivot;
	values[i] = pivot_value;
	if i - 1 > left
	{
		_ = quick_sort_keys_values(keys, values, left, i - 1, comparer);
	}
	if right > i + 1
	{
		_ = quick_sort_keys_values(keys, values, i + 1, right, comparer);
	}
	return .Ok;
}

quick_sort_keys :: proc "contextless" (
	keys: [^]$K, left, right: int, comparer: Comparer_Procs,
) -> Sort_Status
{
	if keys == nil || comparer.compare == nil || left < 0 || right < left
	{
		return .Invalid_Argument;
	}
	if right - left <= 30
	{
		return insertion_sort_keys(keys, left, right, comparer);
	}
	pivot_index := quick_sort_find_median_of_three(keys, left, right, comparer);
	pivot := keys[pivot_index];
	keys[pivot_index] = keys[right];
	i := left - 1;
	j := right;
	for
	{
		for
		{
			i += 1;
			if comparer.compare(&keys[i], &pivot, comparer.comparer_context) >= 0
			{
				break;
			}
		}
		for
		{
			j -= 1;
			if comparer.compare(&pivot, &keys[j], comparer.comparer_context) >= 0 || j <= i
			{
				break;
			}
		}
		if i >= j
		{
			break;
		}
		sort_swap(&keys[i], &keys[j]);
	}
	keys[right] = keys[i];
	keys[i] = pivot;
	if i - 1 > left
	{
		_ = quick_sort_keys(keys, left, i - 1, comparer);
	}
	if right > i + 1
	{
		_ = quick_sort_keys(keys, i + 1, right, comparer);
	}
	return .Ok;
}

lsb_radix_reorder_byte :: proc "contextless" (
	source_keys: [^]i32, target_keys: [^]i32, source_values: [^]$V, target_values: [^]V,
	key_count: int, indices: [^]i32, shift: uint,
)
{
	for index in 0 ..< key_count
	{
		key := source_keys[index];
		bucket := int((u32(key) >> shift) & 0xff);
		target_index := int(indices[bucket]);
		target_keys[target_index] = key;
		target_values[target_index] = source_values[index];
		indices[bucket] += 1;
	}
}

lsb_radix_sort_passes :: proc "contextless" (
	input_keys: [^]i32, input_values: [^]$V, scratch_keys: [^]i32, scratch_values: [^]V,
	bucket_counts: [^]i32, key_count, bucket_set_count: int,
) -> Sorted_Buffer
{
	for index in 0 ..< key_count
	{
		key := u32(input_keys[index]);
		for pass in 0 ..< bucket_set_count
		{
			bucket_counts[pass * 256 + int((key >> uint(pass * 8)) & 0xff)] += 1;
		}
	}
	for pass in 0 ..< bucket_set_count
	{
		sum: i32 = 0;
		for bucket in 0 ..< 256
		{
			count_index := pass * 256 + bucket;
			previous := sum;
			sum += bucket_counts[count_index];
			bucket_counts[count_index] = previous;
		}
	}
	for pass in 0 ..< bucket_set_count
	{
		if (pass & 1) == 0
		{
			lsb_radix_reorder_byte(
				input_keys,
				scratch_keys,
				input_values,
				scratch_values,
				key_count,
				&bucket_counts[pass * 256],
				uint(pass * 8)
			);
		}
		else
		{
			lsb_radix_reorder_byte(
				scratch_keys,
				input_keys,
				scratch_values,
				input_values,
				key_count,
				&bucket_counts[pass * 256],
				uint(pass * 8)
			);
		}
	}
	if (bucket_set_count & 1) == 0
	{
		return .Input;
	}
	return .Scratch;
}

lsb_radix_sort :: proc (
	input_keys: Buffer(i32), input_values: Buffer($V), scratch_keys: Buffer(i32), scratch_values: Buffer(V),
	start_index, count: int, keys_upper_bound: u32, pool: ^Buffer_Pool,
) -> (Sorted_Buffer, Sort_Status)
{
	if start_index < 0 || count < 0 || start_index > int(input_keys.length) - count ||
		start_index > int(input_values.length) - count || start_index > int(scratch_keys.length) - count ||
		start_index > int(scratch_values.length) - count || pool == nil
	{
		return .Input, .Invalid_Argument;
	}
	bucket_set_count := 4;
	if keys_upper_bound < (1 << 8)
	{
		bucket_set_count = 1;
	}
	else if keys_upper_bound < (1 << 16)
	{
		bucket_set_count = 2;
	}
	else if keys_upper_bound < (1 << 24)
	{
		bucket_set_count = 3;
	}
	bucket_counts, take_status := buffer_pool_take(pool, i32, bucket_set_count * 256);
	if take_status != .Ok
	{
		return .Input, .Capacity_Missing;
	}
	_ = buffer_clear(bucket_counts, 0, int(bucket_counts.length));
	selected := lsb_radix_sort_passes(
		&input_keys.memory[start_index], &input_values.memory[start_index],
		&scratch_keys.memory[start_index], &scratch_values.memory[start_index],
		bucket_counts.memory, count, bucket_set_count,
	);
	return_status := buffer_pool_return(pool, &bucket_counts);
	if return_status != .Ok
	{
		return selected, .Invalid_Argument;
	}
	return selected, .Ok;
}

msb_radix_sort_u32 :: proc "contextless" (
	keys: [^]i32, values: [^]$V, bucket_storage: [^]i32, key_count: int, shift: uint,
) -> Sort_Status
{
	if key_count <= 1
	{
		return .Ok;
	}
	if key_count < 32
	{
		return insertion_sort_keys_values(keys, values, 0, key_count - 1, primitive_i32_comparer());
	}
	bucket_counts := bucket_storage;
	original_starts := ([^]i32)(&bucket_storage[1024]);
	for bucket in 0 ..< 256
	{
		bucket_counts[bucket] = 0;
	}
	for index in 0 ..< key_count
	{
		bucket_counts[int((u32(keys[index]) >> shift) & 0xff)] += 1;
	}
	sum: i32 = 0;
	for bucket in 0 ..< 256
	{
		previous := sum;
		sum += bucket_counts[bucket];
		original_starts[bucket] = previous;
		bucket_counts[bucket] = previous;
	}
	for bucket in 0 ..< 256
	{
		next_start := key_count;
		if bucket < 255
		{
			next_start = int(original_starts[bucket + 1]);
		}
		for int(bucket_counts[bucket]) < next_start
		{
			local_key := keys[bucket_counts[bucket]];
			local_value := values[bucket_counts[bucket]];
			for
			{
				target_bucket := int((u32(local_key) >> shift) & 0xff);
				if target_bucket == bucket
				{
					keys[bucket_counts[bucket]] = local_key;
					values[bucket_counts[bucket]] = local_value;
					bucket_counts[bucket] += 1;
					break;
				}
				target_index := bucket_counts[target_bucket];
				sort_swap(&keys[target_index], &local_key);
				sort_swap(&values[target_index], &local_value);
				bucket_counts[target_bucket] += 1;
			}
		}
	}
	if shift > 0
	{
		new_shift := shift - 8;
		previous_end := 0;
		for bucket in 0 ..< 256
		{
			bucket_end := int(bucket_counts[bucket]);
			bucket_count := bucket_end - previous_end;
			if bucket_count > 0
			{
				_ = msb_radix_sort_u32(
					&keys[previous_end],
					&values[previous_end],
					&bucket_storage[256],
					bucket_count,
					new_shift
				);
			}
			previous_end = bucket_end;
		}
	}
	return .Ok;
}

F32x4 :: simd.f32x4;
I32x4 :: simd.i32x4;

vector_counting_sort_f32x8 :: proc "contextless" (
	padded_keys: Buffer(f32), padded_target_indices: Buffer(i32), element_count: int,
) -> Sort_Status
{
	if padded_keys.memory == nil ||
		padded_target_indices.memory == nil ||
		padded_keys.length != padded_target_indices.length ||
		(padded_keys.length & 7) != 0 || element_count < 0 || element_count > int(padded_keys.length)
	{
		return .Invalid_Argument;
	}
	index_offsets := I32x8{0, 1, 2, 3, 4, 5, 6, 7};
	one := I32x8(1);
	for base := 0; base < element_count; base += 8
	{
		values := intrinsics.unaligned_load((^F32x8)(&padded_keys.memory[base]));
		counts := I32x8(0);
		for test_index in 0 ..< base
		{
			mask := simd.lanes_le(F32x8(padded_keys.memory[test_index]), values);
			counts += transmute(I32x8)mask & one;
		}
		equality_end := min(base + 8, element_count);
		slot_indices := I32x8(i32(base)) + index_offsets;
		for test_index := base; test_index < equality_end; test_index += 1
		{
			test_values := F32x8(padded_keys.memory[test_index]);
			less_mask := transmute(I32x8)simd.lanes_lt(test_values, values);
			index_mask := transmute(I32x8)simd.lanes_lt(I32x8(i32(test_index)), slot_indices);
			equal_mask := transmute(I32x8)simd.lanes_eq(test_values, values);
			counts += (less_mask | (equal_mask & index_mask)) & one;
		}
		for test_index := equality_end; test_index < element_count; test_index += 1
		{
			mask := simd.lanes_lt(F32x8(padded_keys.memory[test_index]), values);
			counts += transmute(I32x8)mask & one;
		}
		intrinsics.unaligned_store((^I32x8)(&padded_target_indices.memory[base]), counts);
	}
	return .Ok;
}

vector_counting_sort_f32x4 :: proc "contextless" (
	padded_keys: Buffer(f32), padded_target_indices: Buffer(i32), element_count: int,
) -> Sort_Status
{
	if padded_keys.memory == nil ||
		padded_target_indices.memory == nil ||
		padded_keys.length != padded_target_indices.length ||
		(padded_keys.length & 3) != 0 || element_count < 0 || element_count > int(padded_keys.length)
	{
		return .Invalid_Argument;
	}
	index_offsets := I32x4{0, 1, 2, 3};
	one := I32x4(1);
	for base := 0; base < element_count; base += 4
	{
		values := intrinsics.unaligned_load((^F32x4)(&padded_keys.memory[base]));
		counts := I32x4(0);
		for test_index in 0 ..< base
		{
			counts += transmute(I32x4)simd.lanes_le(F32x4(padded_keys.memory[test_index]), values) & one;
		}
		equality_end := min(base + 4, element_count);
		slot_indices := I32x4(i32(base)) + index_offsets;
		for test_index := base; test_index < equality_end; test_index += 1
		{
			test_values := F32x4(padded_keys.memory[test_index]);
			less_mask := transmute(I32x4)simd.lanes_lt(test_values, values);
			index_mask := transmute(I32x4)simd.lanes_lt(I32x4(i32(test_index)), slot_indices);
			equal_mask := transmute(I32x4)simd.lanes_eq(test_values, values);
			counts += (less_mask | (equal_mask & index_mask)) & one;
		}
		for test_index := equality_end; test_index < element_count; test_index += 1
		{
			counts += transmute(I32x4)simd.lanes_lt(F32x4(padded_keys.memory[test_index]), values) & one;
		}
		intrinsics.unaligned_store((^I32x4)(&padded_target_indices.memory[base]), counts);
	}
	return .Ok;
}
