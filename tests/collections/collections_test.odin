package collections_tests

import "core:testing"
import entasis "entasis:entasis_utilities"

prepare_pool :: proc(t: ^testing.T, pool: ^entasis.Buffer_Pool, byte_count: int = 4096)
{
	testing.expect_value(t, entasis.buffer_pool_initialize(pool, 128), entasis.Memory_Status.Ok);
	for power in 0 ..= 12
	{
		testing.expect_value(
			t,
			entasis.buffer_pool_ensure_capacity_for_power(pool, byte_count, power),
			entasis.Memory_Status.Ok
		);
	}
}

collision_hash :: proc "contextless" (value: rawptr) -> i32
{
	_ = value;
	return 7;
}

collision_hash_equal :: proc "contextless" () -> entasis.Hash_Equal_Procs
{
	return {hash=collision_hash, equal=entasis.primitive_equal_i32};
}

next_random :: proc(state: ^u32) -> u32
{
	state^ = state^ * 1664525 + 1013904223;
	return state^;
}

Centroid_Comparer_Context :: struct
{
	centroids: [^]f32,
}

compare_centroid_indices :: proc "contextless" (a, b, comparer_context: rawptr) -> int
{
	ctx := (^Centroid_Comparer_Context)(comparer_context);
	left := ctx.centroids[int((^i32)(a)^)];
	right := ctx.centroids[int((^i32)(b)^)];
	if left < right
	{
		return -1;
	}
	if left > right
	{
		return 1;
	}
	return 0;
}

LSB_Pass_Case :: struct
{
	upper_bound:     u32,
	input_keys:      [6]i32,
	expected_keys:   [6]i32,
	expected_values: [6]i32,
	expected_buffer: entasis.Sorted_Buffer,
}

@(test)
quick_list_and_queue_preserve_order_across_growth_and_wrap :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);

	list: entasis.Quick_List(i32);
	testing.expect_value(t, entasis.quick_list_initialize(&list, 4, &pool), entasis.Collection_Status.Ok);
	for value in 0 ..< 10
	{
		testing.expect_value(t, entasis.quick_list_add(&list, i32(value), &pool), entasis.Collection_Status.Ok);
	}
	testing.expect_value(t, entasis.quick_list_remove_at(&list, 3), entasis.Collection_Status.Ok);
	testing.expect_value(t, list.count, 9);
	testing.expect_value(t, list.span.memory[3], i32(4));
	testing.expect_value(t, entasis.quick_list_fast_remove_at(&list, 1), entasis.Collection_Status.Ok);
	testing.expect_value(t, list.span.memory[1], i32(9));
	last, pop_status := entasis.quick_list_pop(&list);
	testing.expect_value(t, pop_status, entasis.Collection_Status.Ok);
	testing.expect_value(t, last, i32(8));
	testing.expect_value(t, entasis.quick_list_dispose(&list, &pool), entasis.Collection_Status.Ok);

	queue: entasis.Quick_Queue(i32);
	testing.expect_value(t, entasis.quick_queue_initialize(&queue, 4, &pool), entasis.Collection_Status.Ok);
	for value in 0 ..< 4
	{
		testing.expect_value(t, entasis.quick_queue_enqueue(&queue, i32(value), &pool), entasis.Collection_Status.Ok);
	}
	for expected in 0 ..< 2
	{
		value, status := entasis.quick_queue_dequeue(&queue);
		testing.expect_value(t, status, entasis.Collection_Status.Ok);
		testing.expect_value(t, value, i32(expected));
	}
	for value in 4 ..< 9
	{
		testing.expect_value(t, entasis.quick_queue_enqueue(&queue, i32(value), &pool), entasis.Collection_Status.Ok);
	}
	testing.expect_value(t, entasis.quick_queue_enqueue_first(&queue, i32(-1), &pool), entasis.Collection_Status.Ok);
	expected := [8]i32{-1, 2, 3, 4, 5, 6, 7, 8};
	for expected_value in expected
	{
		value, status := entasis.quick_queue_dequeue(&queue);
		testing.expect_value(t, status, entasis.Collection_Status.Ok);
		testing.expect_value(t, value, expected_value);
	}
	_, empty_status := entasis.quick_queue_dequeue(&queue);
	testing.expect_value(t, empty_status, entasis.Collection_Status.Empty);
	testing.expect_value(t, entasis.quick_queue_dispose(&queue, &pool), entasis.Collection_Status.Ok);
}

@(test)
quick_hash_collections_preserve_probe_remove_and_dense_iteration_semantics :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);

	dictionary: entasis.Quick_Dictionary(i32, i32);
	testing.expect_value(
		t,
		entasis.quick_dictionary_initialize(&dictionary, 4, &pool, collision_hash_equal()),
		entasis.Collection_Status.Ok
	);
	model_present: [64]entasis.Presence_Status;
	model_values: [64]i32;
	random_state := u32(0x9382_71ab);
	for iteration in 0 ..< 2048
	{
		random_value := next_random(&random_state);
		key := i32(random_value & 63);
		key_index := int(key);
		if (random_value & 3) != 0
		{
			value := i32(iteration * 17 + key_index);
			_, status := entasis.quick_dictionary_add_and_replace(&dictionary, key, value, &pool);
			testing.expect_value(t, status, entasis.Collection_Status.Ok);
			model_present[key_index] = .Present;
			model_values[key_index] = value;
		}
		else
		{
			key_copy := key;
			status := entasis.quick_dictionary_fast_remove(&dictionary, &key_copy);
			if model_present[key_index] == .Present
			{
				testing.expect_value(t, status, entasis.Collection_Status.Ok);
				model_present[key_index] = .Missing;
			}
			else
			{
				testing.expect_value(t, status, entasis.Collection_Status.Not_Found);
			}
		}
		for verify_index in 0 ..< len(model_present)
		{
			verify_key := i32(verify_index);
			value, presence := entasis.quick_dictionary_try_get(&dictionary, &verify_key);
			testing.expect_value(t, presence, model_present[verify_index]);
			if presence == .Present
			{
				testing.expect_value(t, value^, model_values[verify_index]);
			}
		}
	}
	for index in 0 ..< dictionary.count
	{
		key := int(dictionary.keys.memory[index]);
		testing.expect_value(t, model_present[key], entasis.Presence_Status.Present);
		testing.expect_value(t, dictionary.values.memory[index], model_values[key]);
	}
	testing.expect_value(t, entasis.quick_dictionary_dispose(&dictionary, &pool), entasis.Collection_Status.Ok);

	set: entasis.Quick_Set(i32);
	testing.expect_value(
		t,
		entasis.quick_set_initialize(&set, 4, &pool, collision_hash_equal()),
		entasis.Collection_Status.Ok
	);
	for value in 0 ..< 24
	{
		testing.expect_value(t, entasis.quick_set_add(&set, i32(value), &pool), entasis.Collection_Status.Ok);
	}
	for value in 0 ..< 24
	{
		if (value & 1) == 0
		{
			value_copy := i32(value);
			testing.expect_value(t, entasis.quick_set_fast_remove(&set, &value_copy), entasis.Collection_Status.Ok);
		}
	}
	for value in 0 ..< 24
	{
		value_copy := i32(value);
		expected_index_state := entasis.Presence_Status.Present;
		if (value & 1) == 0
		{
			expected_index_state = .Missing;
		}
		actual := entasis.Presence_Status.Missing;
		if entasis.quick_set_index_of(&set, &value_copy) >= 0
		{
			actual = .Present;
		}
		testing.expect_value(t, actual, expected_index_state);
	}
	testing.expect_value(t, entasis.quick_set_dispose(&set, &pool), entasis.Collection_Status.Ok);
}

@(test)
index_set_and_allocator_cover_fit_resize_compaction_and_churn :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);

	index_set: entasis.Index_Set;
	testing.expect_value(t, entasis.index_set_initialize(&index_set, 8, &pool), entasis.Collection_Status.Ok);
	testing.expect_value(t, entasis.index_set_add(&index_set, 3, &pool), entasis.Collection_Status.Ok);
	testing.expect_value(t, entasis.index_set_add(&index_set, 130, &pool), entasis.Collection_Status.Ok);
	testing.expect_value(t, entasis.index_set_contains(&index_set, 3), entasis.Presence_Status.Present);
	testing.expect_value(t, entasis.index_set_contains(&index_set, 129), entasis.Presence_Status.Missing);
	testing.expect_value(t, entasis.index_set_unset(&index_set, 3), entasis.Collection_Status.Ok);
	testing.expect_value(t, entasis.index_set_dispose(&index_set, &pool), entasis.Collection_Status.Ok);

	allocator: entasis.Memory_Allocator;
	testing.expect_value(t, entasis.memory_allocator_initialize(&allocator, 32, 8, &pool), entasis.Allocator_Status.Ok);
	start1, status1 := entasis.memory_allocator_allocate(&allocator, 1, 8);
	start2, status2 := entasis.memory_allocator_allocate(&allocator, 2, 8);
	start3, status3 := entasis.memory_allocator_allocate(&allocator, 3, 8);
	testing.expect_value(t, status1, entasis.Allocator_Status.Ok);
	testing.expect_value(t, status2, entasis.Allocator_Status.Ok);
	testing.expect_value(t, status3, entasis.Allocator_Status.Ok);
	testing.expect_value(t, start1, i64(0));
	testing.expect_value(t, start2, i64(8));
	testing.expect_value(t, start3, i64(16));
	testing.expect_value(t, entasis.memory_allocator_deallocate(&allocator, 2), entasis.Allocator_Status.Ok);
	testing.expect_value(t, entasis.memory_allocator_can_fit(&allocator, 8), entasis.Allocator_Fit_Status.Can_Fit);
	start4, status4 := entasis.memory_allocator_allocate(&allocator, 4, 8);
	testing.expect_value(t, status4, entasis.Allocator_Status.Ok);
	testing.expect_value(t, start4, i64(8));
	testing.expect_value(t, entasis.memory_allocator_deallocate(&allocator, 1), entasis.Allocator_Status.Ok);
	_, _, old_start, new_start, compact_status := entasis.memory_allocator_incremental_compact(&allocator);
	testing.expect_value(t, compact_status, entasis.Allocator_Status.Ok);
	testing.expect_value(t, old_start, i64(8));
	testing.expect_value(t, new_start, i64(0));
	_, resized_start, resize_status := entasis.memory_allocator_resize(&allocator, 3, 12);
	testing.expect_value(t, resize_status, entasis.Allocator_Status.Ok);
	testing.expect(t, resized_start >= 0);
	largest, total := entasis.memory_allocator_largest_contiguous(&allocator);
	testing.expect(t, largest <= total);
	testing.expect_value(t, entasis.memory_allocator_dispose(&allocator), entasis.Allocator_Status.Ok);
}

@(test)
allocator_randomized_churn_preserves_membership_and_nonoverlap :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);
	allocator: entasis.Memory_Allocator;
	testing.expect_value(
		t,
		entasis.memory_allocator_initialize(&allocator, 256, 64, &pool),
		entasis.Allocator_Status.Ok
	);
	defer entasis.memory_allocator_dispose(&allocator);
	present: [64]entasis.Presence_Status;
	starts: [64]i64;
	sizes: [64]i64;
	random_state := u32(5);
	for _ in 0 ..< 4096
	{
		random_value := next_random(&random_state);
		id_index := int(random_value & 63);
		id := u64(id_index);
		if present[id_index] == .Present && (random_value & 1) == 0
		{
			testing.expect_value(t, entasis.memory_allocator_deallocate(&allocator, id), entasis.Allocator_Status.Ok);
			present[id_index] = .Missing;
		}
		else if present[id_index] == .Missing
		{
			size := i64(1 + ((random_value >> 8) % 5));
			start, status := entasis.memory_allocator_allocate(&allocator, id, size);
			if status == .Ok
			{
				present[id_index] = .Present;
				starts[id_index] = start;
				sizes[id_index] = size;
			}
			else
			{
				testing.expect_value(t, status, entasis.Allocator_Status.Cannot_Fit);
			}
		}
		for verify_index in 0 ..< len(present)
		{
			testing.expect_value(
				t,
				entasis.memory_allocator_contains(&allocator, u64(verify_index)),
				present[verify_index]
			);
			if present[verify_index] == .Present
			{
				for other_index := verify_index + 1; other_index < len(present); other_index += 1
				{
					if present[other_index] == .Present
					{
						testing.expect(
							t,
							starts[verify_index] + sizes[verify_index] <= starts[other_index] ||
							starts[other_index] + sizes[other_index] <= starts[verify_index]
						);
					}
				}
			}
		}
	}
	for index in 0 ..< len(present)
	{
		if present[index] == .Present
		{
			testing.expect_value(
				t,
				entasis.memory_allocator_deallocate(&allocator, u64(index)),
				entasis.Allocator_Status.Ok
			);
		}
		testing.expect_value(
			t,
			entasis.memory_allocator_contains(&allocator, u64(index)),
			entasis.Presence_Status.Missing
		);
	}
}

@(test)
radix_and_vectorized_sorts_preserve_keys_permutation_and_stable_ties :: proc(t: ^testing.T)
{
	pool: entasis.Buffer_Pool;
	prepare_pool(t, &pool);
	defer entasis.buffer_pool_dispose(&pool);

	keys := [12]i32{9, 1, 7, 1, 0, 255, 4, 9, 3, 2, 255, 1};
	values := [12]i32{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
	scratch_keys: [12]i32;
	scratch_values: [12]i32;
	key_buffer := entasis.Buffer(i32){memory=&keys[0], length=i32(len(keys)), id=-1};
	value_buffer := entasis.Buffer(i32){memory=&values[0], length=i32(len(values)), id=-1};
	scratch_key_buffer := entasis.Buffer(i32){memory=&scratch_keys[0], length=i32(len(scratch_keys)), id=-1};
	scratch_value_buffer := entasis.Buffer(i32){memory=&scratch_values[0], length=i32(len(scratch_values)), id=-1};
	selected, sort_status := entasis.lsb_radix_sort(
		key_buffer,
		value_buffer,
		scratch_key_buffer,
		scratch_value_buffer,
		0,
		len(keys),
		255,
		&pool
	);
	testing.expect_value(t, sort_status, entasis.Sort_Status.Ok);
	testing.expect_value(t, selected, entasis.Sorted_Buffer.Scratch);
	expected_keys := [12]i32{0, 1, 1, 1, 2, 3, 4, 7, 9, 9, 255, 255};
	expected_values := [12]i32{4, 1, 3, 11, 9, 8, 6, 2, 0, 7, 5, 10};
	for index in 0 ..< len(expected_keys)
	{
		testing.expect_value(t, scratch_keys[index], expected_keys[index]);
		testing.expect_value(t, scratch_values[index], expected_values[index]);
	}

	pass_cases := [4]LSB_Pass_Case{
		{255, {255, 0, 7, 7, 1, 128}, {0, 1, 7, 7, 128, 255}, {1, 4, 2, 3, 5, 0}, .Scratch},
		{65_535, {256, 255, 256, 0, 65_535, 1}, {0, 1, 255, 256, 256, 65_535}, {3, 5, 1, 0, 2, 4}, .Input},
		{
			16_777_215,
			{65_536, 65_535, 65_536, 0, 16_777_215, 256},
			{0, 256, 65_535, 65_536, 65_536, 16_777_215},
			{3, 5, 1, 0, 2, 4},
			.Scratch,
		},
		{
			0xffff_ffff,
			{16_777_216, 1, 16_777_216, 0, -1, -2_147_483_648},
			{0, 1, 16_777_216, 16_777_216, -2_147_483_648, -1},
			{3, 1, 0, 2, 5, 4},
			.Input,
		},
	};
	for pass_case in pass_cases
	{
		pass_keys := pass_case.input_keys;
		pass_values := [6]i32{0, 1, 2, 3, 4, 5};
		pass_scratch_keys: [6]i32;
		pass_scratch_values: [6]i32;
		pass_key_buffer := entasis.Buffer(i32){memory=&pass_keys[0], length=6, id=-1};
		pass_value_buffer := entasis.Buffer(i32){memory=&pass_values[0], length=6, id=-1};
		pass_scratch_key_buffer := entasis.Buffer(i32){memory=&pass_scratch_keys[0], length=6, id=-1};
		pass_scratch_value_buffer := entasis.Buffer(i32){memory=&pass_scratch_values[0], length=6, id=-1};
		pass_selected, pass_status := entasis.lsb_radix_sort(
			pass_key_buffer, pass_value_buffer, pass_scratch_key_buffer, pass_scratch_value_buffer,
			0, 6, pass_case.upper_bound, &pool,
		);
		testing.expect_value(t, pass_status, entasis.Sort_Status.Ok);
		testing.expect_value(t, pass_selected, pass_case.expected_buffer);
		actual_keys := pass_keys[:];
		actual_values := pass_values[:];
		if pass_selected == .Scratch
		{
			actual_keys = pass_scratch_keys[:];
			actual_values = pass_scratch_values[:];
		}
		for index in 0 ..< 6
		{
			testing.expect_value(t, actual_keys[index], pass_case.expected_keys[index]);
			testing.expect_value(t, actual_values[index], pass_case.expected_values[index]);
		}
	}

	msb_keys := [40]i32{
		39,
		8,
		17,
		2,
		31,
		5,
		29,
		12,
		0,
		24,
		10,
		7,
		35,
		19,
		4,
		22,
		15,
		38,
		1,
		28,
		9,
		33,
		6,
		26,
		13,
		20,
		3,
		37,
		11,
		30,
		18,
		25,
		14,
		34,
		16,
		27,
		21,
		32,
		23,
		36,
	};
	msb_values: [40]i32;
	for index in 0 ..< len(msb_values)
	{
		msb_values[index] = msb_keys[index] * 3;
	}
	buckets: [2048]i32;
	testing.expect_value(
		t,
		entasis.msb_radix_sort_u32(&msb_keys[0], &msb_values[0], &buckets[0], len(msb_keys), 24),
		entasis.Sort_Status.Ok
	);
	for index in 0 ..< len(msb_keys)
	{
		testing.expect_value(t, msb_keys[index], i32(index));
		testing.expect_value(t, msb_values[index], i32(index * 3));
	}
	quick_keys := [40]i32{
		39,
		8,
		17,
		2,
		31,
		5,
		29,
		12,
		0,
		24,
		10,
		7,
		35,
		19,
		4,
		22,
		15,
		38,
		1,
		28,
		9,
		33,
		6,
		26,
		13,
		20,
		3,
		37,
		11,
		30,
		18,
		25,
		14,
		34,
		16,
		27,
		21,
		32,
		23,
		36,
	};
	testing.expect_value(
		t,
		entasis.quick_sort_keys(&quick_keys[0], 0, len(quick_keys) - 1, entasis.primitive_i32_comparer()),
		entasis.Sort_Status.Ok
	);
	for index in 0 ..< len(quick_keys)
	{
		testing.expect_value(t, quick_keys[index], i32(index));
	}
	centroids: [40]f32;
	centroid_indices: [40]i32;
	for index in 0 ..< len(centroids)
	{
		centroids[index] = f32(len(centroids) - 1 - index);
		centroid_indices[index] = i32(index);
	}
	centroid_context := Centroid_Comparer_Context{centroids=&centroids[0]};
	centroid_comparer := entasis.Comparer_Procs{compare=compare_centroid_indices, comparer_context=&centroid_context};
	testing.expect_value(
		t,
		entasis.quick_sort_keys(&centroid_indices[0], 0, len(centroid_indices) - 1, centroid_comparer),
		entasis.Sort_Status.Ok
	);
	for index in 0 ..< len(centroid_indices)
	{
		testing.expect_value(t, centroid_indices[index], i32(len(centroid_indices) - 1 - index));
	}

	vector_keys := [8]f32{4, 1, 4, -2, 3, 1, 0, 8};
	indices8: [8]i32;
	vector_buffer := entasis.Buffer(f32){memory=&vector_keys[0], length=8, id=-1};
	indices8_buffer := entasis.Buffer(i32){memory=&indices8[0], length=8, id=-1};
	testing.expect_value(
		t,
		entasis.vector_counting_sort_f32x8(vector_buffer, indices8_buffer, 8),
		entasis.Sort_Status.Ok
	);
	expected_indices := [8]i32{5, 2, 6, 0, 4, 3, 1, 7};
	for index in 0 ..< 8
	{
		testing.expect_value(t, indices8[index], expected_indices[index]);
	}

	indices4: [8]i32;
	indices4_buffer := entasis.Buffer(i32){memory=&indices4[0], length=8, id=-1};
	testing.expect_value(
		t,
		entasis.vector_counting_sort_f32x4(vector_buffer, indices4_buffer, 8),
		entasis.Sort_Status.Ok
	);
	for index in 0 ..< 8
	{
		testing.expect_value(t, indices4[index], expected_indices[index]);
	}
}
