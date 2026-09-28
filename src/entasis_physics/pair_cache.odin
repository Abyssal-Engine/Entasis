// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Collidable_Pair :: struct
{
	a: Collidable_Reference,
	b: Collidable_Reference,
}

Constraint_Cache :: struct
{
	constraint_handle: Constraint_Handle,
	feature_ids:       [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
}

Pair_Cache_Add_Transaction :: struct
{
	mapping_index: i32,
	table_index:   i32,
}

Collision_Pair_Location :: struct
{
	pair:                Collidable_Pair,
	inactive_set_index:  i32,
	inactive_pair_index: i32,
}

Inactive_Pair_Cache_Entry :: struct
{
	pair:      Collidable_Pair,
	cache:     Constraint_Cache,
	set_index: i32,
}

Pair_Cache_State :: enum u8
{
	Uninitialized,
	Ready,
	Prepared,
	Disposed,
}

Pair_Cache :: struct
{
	mapping:                    util.Quick_Dictionary(Collidable_Pair, Constraint_Cache),
	pair_freshness:             util.Buffer(u8),
	constraint_handle_to_pair: util.Buffer(Collision_Pair_Location),
	inactive_entries:           util.Buffer(Inactive_Pair_Cache_Entry),
	inactive_count:             int,
	pool:                       ^util.Buffer_Pool,
	freshness_generation:       u8,
	state:                      Pair_Cache_State,
}

collidable_pair_create :: proc "contextless" (a, b: Collidable_Reference) -> Collidable_Pair
{
	mobility_a := collidable_reference_mobility(a);
	mobility_b := collidable_reference_mobility(b);
	if mobility_a == .Static && mobility_b != .Static
	{
		return {b, a};
	}
	if mobility_b == .Static
	{
		return {a, b};
	}
	if collidable_reference_raw_handle(b) < collidable_reference_raw_handle(a)
	{
		return {b, a};
	}
	return {a, b};
}

collidable_pair_hash :: proc "contextless" (value: rawptr) -> i32
{
	pair := (^Collidable_Pair)(value);
	p1 :: u64(961748927);
	p2 :: u64(899809343);
	hash := u64(pair.a.packed) * (p1 * p2) + u64(pair.b.packed) * p2;
	return i32(u32(hash) ~ u32(hash >> 32));
}

collidable_pair_equal :: proc "contextless" (a, b: rawptr) -> util.Comparison_Status
{
	left := (^Collidable_Pair)(a);
	right := (^Collidable_Pair)(b);
	if left.a.packed == right.a.packed && left.b.packed == right.b.packed
	{
		return .Equal;
	}
	return .Different;
}

pair_cache_initialize :: proc (
	cache: ^Pair_Cache, pair_capacity, constraint_capacity, inactive_pair_capacity: int,
	pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if cache == nil || pool == nil || pair_capacity <= 0 || constraint_capacity <= 0 ||
		inactive_pair_capacity <= 0 ||
		cache.state != .Uninitialized
	{
		return .Invalid_Argument;
	}
	status := util.quick_dictionary_initialize(
		&cache.mapping, pair_capacity, pool,
		{hash=collidable_pair_hash, equal=collidable_pair_equal}, table_power_offset=1,
	);
	if status != .Ok
	{
		return physics_collection_status(status);
	}
	freshness, freshness_status := util.buffer_pool_take_at_least(pool, u8, int(cache.mapping.keys.length));
	if freshness_status != .Ok
	{
		_ = util.quick_dictionary_dispose(&cache.mapping, pool);
		return physics_memory_status(freshness_status);
	}
	locations, locations_status := util.buffer_pool_take_at_least(pool, Collision_Pair_Location, constraint_capacity);
	if locations_status != .Ok
	{
		physics_return_buffer(pool, &freshness);
		_ = util.quick_dictionary_dispose(&cache.mapping, pool);
		return physics_memory_status(locations_status);
	}
	inactive, inactive_status := util.buffer_pool_take_at_least(
		pool,
		Inactive_Pair_Cache_Entry,
		inactive_pair_capacity
	);
	if inactive_status != .Ok
	{
		physics_return_buffer(pool, &locations);
		physics_return_buffer(pool, &freshness);
		_ = util.quick_dictionary_dispose(&cache.mapping, pool);
		return physics_memory_status(inactive_status);
	}
	_ = util.buffer_clear(freshness, 0, int(freshness.length));
	_ = util.buffer_clear(locations, 0, int(locations.length));
	_ = util.buffer_clear(inactive, 0, int(inactive.length));
	for index in 0 ..< locations.length
	{
		locations.memory[index].inactive_set_index = -1;
		locations.memory[index].inactive_pair_index = -1;
	}
	cache.pair_freshness = freshness;
	cache.constraint_handle_to_pair = locations;
	cache.inactive_entries = inactive;
	cache.pool = pool;
	cache.state = .Ready;
	return .Ok;
}

pair_cache_prepare :: proc (cache: ^Pair_Cache) -> Physics_Status
{
	if cache == nil || cache.state != .Ready
	{
		return .Invalid_Argument;
	}
	if cache.freshness_generation == 1
	{
		cache.freshness_generation = 2;
	}
	else
	{
		cache.freshness_generation = 1;
	}
	cache.state = .Prepared;
	return .Ok;
}

pair_cache_ensure_active_capacity :: proc (
	cache: ^Pair_Cache, mapping_capacity, constraint_handle_capacity: int,
) -> Physics_Status
{
	if cache == nil || (cache.state != .Prepared && cache.state != .Ready) || mapping_capacity < cache.mapping.count ||
		constraint_handle_capacity < 0
	{
		return .Invalid_Argument;
	}
	status := util.quick_dictionary_ensure_capacity(&cache.mapping, mapping_capacity, cache.pool);
	if status != .Ok
	{
		return physics_collection_status(status);
	}
	if int(cache.pair_freshness.length) < int(cache.mapping.keys.length)
	{
		freshness, freshness_status := util.buffer_pool_take_at_least(cache.pool, u8, int(cache.mapping.keys.length));
		if freshness_status != .Ok
		{
			return physics_memory_status(freshness_status);
		}
		_ = util.buffer_clear(freshness, 0, int(freshness.length));
		_ = util.buffer_copy(
			util.buffer_view(cache.pair_freshness), 0, util.buffer_view(freshness), 0, cache.mapping.count,
		);
		physics_return_buffer(cache.pool, &cache.pair_freshness);
		cache.pair_freshness = freshness;
	}
	if constraint_handle_capacity > int(cache.constraint_handle_to_pair.length)
	{
		locations, location_status := util.buffer_pool_take_at_least(
			cache.pool, Collision_Pair_Location, constraint_handle_capacity,
		);
		if location_status != .Ok
		{
			return physics_memory_status(location_status);
		}
		for index in 0 ..< locations.length
		{
			locations.memory[index] = {inactive_set_index=-1, inactive_pair_index=-1};
		}
		_ = util.buffer_copy(
			util.buffer_view(cache.constraint_handle_to_pair), 0, util.buffer_view(locations), 0,
			int(cache.constraint_handle_to_pair.length),
		);
		physics_return_buffer(cache.pool, &cache.constraint_handle_to_pair);
		cache.constraint_handle_to_pair = locations;
	}
	return .Ok;
}

pair_cache_ensure_capacity :: proc (
	cache: ^Pair_Cache, mapping_capacity, constraint_handle_capacity,
	inactive_pair_capacity: int,
) -> Physics_Status
{
	if cache == nil || cache.state != .Ready || inactive_pair_capacity <= 0
	{
		return .Invalid_Argument;
	}
	status := pair_cache_ensure_active_capacity(
		cache, mapping_capacity, constraint_handle_capacity,
	);
	if status != .Ok
	{
		return status;
	}
	status = physics_ensure_buffer_capacity(
		cache.pool, &cache.inactive_entries, inactive_pair_capacity,
		cache.inactive_count,
	);
	if status != .Ok
	{
		return status;
	}
	return .Ok;
}

pair_cache_resize :: proc (
	cache: ^Pair_Cache, mapping_capacity, constraint_handle_capacity,
	inactive_pair_capacity: int,
) -> Physics_Status
{
	if cache == nil || cache.state != .Ready || mapping_capacity <= 0 ||
		constraint_handle_capacity <= 0 || inactive_pair_capacity <= 0
	{
		return .Invalid_Argument;
	}
	mapping_target := max(mapping_capacity, cache.mapping.count);
	status := util.quick_dictionary_resize(
		&cache.mapping, mapping_target, cache.pool,
	);
	if status != .Ok
	{
		return physics_collection_status(status);
	}
	resize_status := physics_resize_buffer_capacity(
		cache.pool, &cache.pair_freshness,
		int(cache.mapping.keys.length), 0,
	);
	if resize_status != .Ok
	{
		return resize_status;
	}
	_ = util.buffer_clear(
		cache.pair_freshness, 0, int(cache.pair_freshness.length),
	);
	cache.freshness_generation = 0;
	required_constraint_capacity := constraint_handle_capacity;
	location_copy_count := min(
		required_constraint_capacity,
		int(cache.constraint_handle_to_pair.length),
	);
	resize_status = physics_resize_buffer_capacity(
		cache.pool, &cache.constraint_handle_to_pair,
		required_constraint_capacity,
		location_copy_count,
	);
	if resize_status != .Ok
	{
		return resize_status;
	}
	for index in location_copy_count ..< int(cache.constraint_handle_to_pair.length)
	{
		cache.constraint_handle_to_pair.memory[index] = {
			inactive_set_index=-1, inactive_pair_index=-1,
		};
	}
	resize_status = physics_resize_buffer_capacity(
		cache.pool, &cache.inactive_entries,
		max(inactive_pair_capacity, cache.inactive_count),
		cache.inactive_count,
	);
	if resize_status != .Ok
	{
		return resize_status;
	}
	return .Ok;
}

pair_cache_index_of :: #force_inline proc "contextless" (
	cache: ^Pair_Cache, pair: Collidable_Pair,
) -> int
{
	if cache == nil || cache.state != .Prepared && cache.state != .Ready
	{
		return -1;
	}
	pair_copy := pair;
	table_index := int(u32(util.hash_rehash(collidable_pair_hash(&pair_copy)))) &
		cache.mapping.table_mask;
	for
	{
		encoded_index := int(cache.mapping.table.memory[table_index]);
		if encoded_index <= 0
		{
			return -1;
		}
		element_index := encoded_index - 1;
		key := cache.mapping.keys.memory[element_index];
		if key.a.packed == pair.a.packed && key.b.packed == pair.b.packed
		{
			return element_index;
		}
		table_index = (table_index + 1) & cache.mapping.table_mask;
	}
}

pair_cache_mark_fresh :: proc "contextless" (cache: ^Pair_Cache, index: int) -> Physics_Status
{
	if cache == nil || cache.state != .Prepared || index < 0 || index >= cache.mapping.count
	{
		return .Invalid_Argument;
	}
	cache.pair_freshness.memory[index] = cache.freshness_generation;
	return .Ok;
}

pair_cache_update_direct :: proc "contextless" (
	cache: ^Pair_Cache, index: int, value: Constraint_Cache,
) -> Physics_Status
{
	if cache == nil || index < 0 || index >= cache.mapping.count
	{
		return .Not_Found;
	}
	cache.mapping.values.memory[index] = value;
	return pair_cache_bind_constraint(cache, cache.mapping.keys.memory[index], value.constraint_handle);
}

pair_cache_write_add_reserved :: #force_inline proc "contextless" (
	cache: ^Pair_Cache, transaction: Pair_Cache_Add_Transaction,
	pair: Collidable_Pair, value: Constraint_Cache,
)
{
	index := int(transaction.mapping_index);
	cache.mapping.keys.memory[index] = pair;
	cache.mapping.values.memory[index] = value;
	cache.mapping.table.memory[transaction.table_index] = transaction.mapping_index + 1;
	cache.pair_freshness.memory[index] = cache.freshness_generation;
	cache.constraint_handle_to_pair.memory[value.constraint_handle.value] = {
		pair=pair, inactive_set_index=-1, inactive_pair_index=-1,
	};
}

pair_cache_commit_add :: #force_inline proc "contextless" (
	cache: ^Pair_Cache, transaction: Pair_Cache_Add_Transaction,
	pair: Collidable_Pair, value: Constraint_Cache,
)
{
	pair_cache_write_add_reserved(cache, transaction, pair, value);
	cache.mapping.count += 1;
}

pair_cache_commit_update :: #force_inline proc "contextless" (
	cache: ^Pair_Cache, mapping_index: int, pair: Collidable_Pair, value: Constraint_Cache,
)
{
	old_handle := cache.mapping.values.memory[mapping_index].constraint_handle;
	if old_handle.value != value.constraint_handle.value
	{
		if old_handle.value >= 0 &&
			int(old_handle.value) < int(cache.constraint_handle_to_pair.length)
		{
			cache.constraint_handle_to_pair.memory[old_handle.value] = {
				inactive_set_index=-1, inactive_pair_index=-1,
			};
		}
		cache.constraint_handle_to_pair.memory[value.constraint_handle.value] = {
			pair=pair, inactive_set_index=-1, inactive_pair_index=-1,
		};
	}
	cache.mapping.keys.memory[mapping_index] = pair;
	cache.mapping.values.memory[mapping_index] = value;
}

pair_cache_commit_remove :: proc "contextless" (cache: ^Pair_Cache, pair: Collidable_Pair)
{
	pair_copy := pair;
	table_index := int(u32(util.hash_rehash(collidable_pair_hash(&pair_copy)))) & cache.mapping.table_mask;
	for
	{
		encoded_index := int(cache.mapping.table.memory[table_index]);
		if encoded_index <= 0
		{
			return;
		}
		element_index := encoded_index - 1;
		if collidable_pair_equal(&cache.mapping.keys.memory[element_index], &pair_copy) == .Equal
		{
			old_handle := cache.mapping.values.memory[element_index].constraint_handle;
			cache.constraint_handle_to_pair.memory[old_handle.value] = {
				inactive_set_index=-1, inactive_pair_index=-1,
			};
			table_cursor := table_index;
			gap_index := table_cursor;
			for
			{
				table_cursor = (table_cursor + 1) & cache.mapping.table_mask;
				move_candidate := int(cache.mapping.table.memory[table_cursor]);
				if move_candidate <= 0
				{
					break;
				}
				move_candidate -= 1;
				desired_index := int(u32(util.hash_rehash(
					collidable_pair_hash(&cache.mapping.keys.memory[move_candidate]),
				))) & cache.mapping.table_mask;
				distance_from_gap := (table_cursor - gap_index) & cache.mapping.table_mask;
				distance_from_ideal := (table_cursor - desired_index) & cache.mapping.table_mask;
				if distance_from_gap <= distance_from_ideal
				{
					cache.mapping.table.memory[gap_index] = cache.mapping.table.memory[table_cursor];
					gap_index = table_cursor;
				}
			}
			cache.mapping.table.memory[gap_index] = 0;
			cache.mapping.count -= 1;
			if element_index < cache.mapping.count
			{
				cache.mapping.keys.memory[element_index] = cache.mapping.keys.memory[cache.mapping.count];
				cache.mapping.values.memory[element_index] = cache.mapping.values.memory[cache.mapping.count];
				cache.pair_freshness.memory[element_index] = cache.pair_freshness.memory[cache.mapping.count];
				moved_table_index := int(u32(util.hash_rehash(
					collidable_pair_hash(&cache.mapping.keys.memory[element_index]),
				))) & cache.mapping.table_mask;
				for int(cache.mapping.table.memory[moved_table_index]) - 1 != cache.mapping.count
				{
					moved_table_index = (moved_table_index + 1) & cache.mapping.table_mask;
				}
				cache.mapping.table.memory[moved_table_index] = i32(element_index + 1);
			}
			cache.mapping.keys.memory[cache.mapping.count] = {};
			cache.mapping.values.memory[cache.mapping.count] = {};
			cache.pair_freshness.memory[cache.mapping.count] = 0;
			return;
		}
		table_index = (table_index + 1) & cache.mapping.table_mask;
	}
}

pair_cache_bind_constraint :: proc "contextless" (
	cache: ^Pair_Cache, pair: Collidable_Pair, handle: Constraint_Handle,
) -> Physics_Status
{
	if cache == nil || handle.value < 0 || int(handle.value) >= int(cache.constraint_handle_to_pair.length)
	{
		return .Capacity_Missing;
	}
	cache.constraint_handle_to_pair.memory[handle.value] = {pair=pair, inactive_set_index=-1, inactive_pair_index=-1};
	return .Ok;
}

pair_cache_unbind_constraint :: proc "contextless" (cache: ^Pair_Cache, handle: Constraint_Handle)
{
	if cache == nil || handle.value < 0 || int(handle.value) >= int(cache.constraint_handle_to_pair.length)
	{
		return;
	}
	cache.constraint_handle_to_pair.memory[handle.value] = {inactive_set_index=-1, inactive_pair_index=-1};
}

pair_cache_move_constraint_to_inactive :: proc (
	cache: ^Pair_Cache, handle: Constraint_Handle, set_index: int,
) -> Physics_Status
{
	if cache == nil ||
		cache.state != .Ready ||
		handle.value < 0 ||
		int(handle.value) >= int(cache.constraint_handle_to_pair.length) ||
		set_index <= 0 || cache.inactive_count >= int(cache.inactive_entries.length)
	{
		return .Invalid_Argument;
	}
	pair := cache.constraint_handle_to_pair.memory[handle.value].pair;
	mapping_index := util.quick_dictionary_index_of(&cache.mapping, &pair);
	if mapping_index < 0
	{
		return .Not_Found;
	}
	inactive_index := cache.inactive_count;
	cache.inactive_count += 1;
	cache.inactive_entries.memory[inactive_index] = {pair, cache.mapping.values.memory[mapping_index], i32(set_index)};
	cache.constraint_handle_to_pair.memory[handle.value] = {pair, i32(set_index), i32(inactive_index)};
	return physics_collection_status(util.quick_dictionary_fast_remove(&cache.mapping, &pair));
}

pair_cache_awaken_set :: proc (cache: ^Pair_Cache, set_index: int) -> Physics_Status
{
	if cache == nil || (cache.state != .Ready && cache.state != .Prepared) || set_index <= 0
	{
		return .Invalid_Argument;
	}
	index := cache.inactive_count - 1;
	for index >= 0
	{
		entry := cache.inactive_entries.memory[index];
		if int(entry.set_index) == set_index
		{
			status := util.quick_dictionary_add_unsafely(&cache.mapping, entry.pair, entry.cache);
			if status != .Ok
			{
				return physics_collection_status(status);
			}
			cache.pair_freshness.memory[cache.mapping.count - 1] = cache.freshness_generation;
			bind_status := pair_cache_bind_constraint(cache, entry.pair, entry.cache.constraint_handle);
			if bind_status != .Ok
			{
				return bind_status;
			}
			cache.inactive_count -= 1;
			if index < cache.inactive_count
			{
				moved := cache.inactive_entries.memory[cache.inactive_count];
				cache.inactive_entries.memory[index] = moved;
				moved_handle := moved.cache.constraint_handle;
				if moved_handle.value >= 0 && int(moved_handle.value) < int(cache.constraint_handle_to_pair.length)
				{
					cache.constraint_handle_to_pair.memory[moved_handle.value].inactive_pair_index = i32(index);
				}
			}
			cache.inactive_entries.memory[cache.inactive_count] = {};
		}
		index -= 1;
	}
	return .Ok;
}

pair_cache_prepare_awaken_set :: proc (cache: ^Pair_Cache, set_index: int) -> Physics_Status
{
	if cache == nil || (cache.state != .Ready && cache.state != .Prepared) || set_index <= 0
	{
		return .Invalid_Argument;
	}
	count := 0;
	for index in 0 ..< cache.inactive_count
	{
		if int(cache.inactive_entries.memory[index].set_index) == set_index
		{
			count += 1;
		}
	}
	if count > max(int) - cache.mapping.count
	{
		return .Capacity_Missing;
	}
	required_mapping_capacity := cache.mapping.count + count;
	return pair_cache_ensure_active_capacity(
		cache, max(required_mapping_capacity, 1),
		int(cache.constraint_handle_to_pair.length),
	);
}

pair_cache_commit_awaken_set :: proc "contextless" (cache: ^Pair_Cache, set_index: int)
{
	index := cache.inactive_count - 1;
	for index >= 0
	{
		entry := cache.inactive_entries.memory[index];
		if int(entry.set_index) == set_index
		{
			table_index := int(u32(util.hash_rehash(collidable_pair_hash(&entry.pair)))) & cache.mapping.table_mask;
			for cache.mapping.table.memory[table_index] != 0
			{
				table_index = (table_index + 1) & cache.mapping.table_mask;
			}
			mapping_index := cache.mapping.count;
			pair_cache_commit_add(
				cache, {i32(mapping_index), i32(table_index)}, entry.pair, entry.cache,
			);
			cache.inactive_count -= 1;
			if index < cache.inactive_count
			{
				moved := cache.inactive_entries.memory[cache.inactive_count];
				cache.inactive_entries.memory[index] = moved;
				cache.constraint_handle_to_pair.memory[moved.cache.constraint_handle.value].inactive_pair_index = i32(index);
			}
			cache.inactive_entries.memory[cache.inactive_count] = {};
		}
		index -= 1;
	}
}

pair_cache_commit_move_constraint_to_inactive :: proc (
	cache: ^Pair_Cache, handle: Constraint_Handle, set_index: int,
)
{
	pair := cache.constraint_handle_to_pair.memory[handle.value].pair;
	pair_copy := pair;
	mapping_index := util.quick_dictionary_index_of(&cache.mapping, &pair_copy);
	if mapping_index < 0
	{
		return;
	}
	inactive_index := cache.inactive_count;
	cache.inactive_count += 1;
	cache.inactive_entries.memory[inactive_index] = {
		pair, cache.mapping.values.memory[mapping_index], i32(set_index),
	};
	pair_cache_commit_remove(cache, pair);
	cache.constraint_handle_to_pair.memory[handle.value] = {
		pair, i32(set_index), i32(inactive_index),
	};
}

pair_cache_clear :: proc (cache: ^Pair_Cache) -> Physics_Status
{
	if cache == nil || cache.state != .Ready
	{
		return .Invalid_Argument;
	}
	util.quick_dictionary_clear(&cache.mapping);
	_ = util.buffer_clear(cache.pair_freshness, 0, int(cache.pair_freshness.length));
	cache.freshness_generation = 0;
	cache.inactive_count = 0;
	_ = util.buffer_clear(cache.inactive_entries, 0, int(cache.inactive_entries.length));
	_ = util.buffer_clear(cache.constraint_handle_to_pair, 0, int(cache.constraint_handle_to_pair.length));
	for index in 0 ..< cache.constraint_handle_to_pair.length
	{
		cache.constraint_handle_to_pair.memory[index].inactive_set_index = -1;
		cache.constraint_handle_to_pair.memory[index].inactive_pair_index = -1;
	}
	return .Ok;
}

pair_cache_dispose :: proc (cache: ^Pair_Cache) -> Physics_Status
{
	if cache == nil || cache.state == .Disposed || cache.pool == nil
	{
		return .Disposed;
	}
	pool := cache.pool;
	physics_return_buffer(pool, &cache.inactive_entries);
	physics_return_buffer(pool, &cache.constraint_handle_to_pair);
	physics_return_buffer(pool, &cache.pair_freshness);
	_ = util.quick_dictionary_dispose(&cache.mapping, pool);
	cache^ = {state=.Disposed};
	return .Ok;
}

#assert(size_of(Collidable_Pair) == 8);
#assert(size_of(Constraint_Cache) == 20);
