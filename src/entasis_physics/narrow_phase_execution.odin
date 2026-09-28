package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"
import "base:runtime"
import "core:simd"

Narrow_Phase_Overlap_Context :: struct
{
	narrow:       ^Narrow_Phase,
	worker_count: int,
	dt:           f32,
	fused_route:  Narrow_Phase_Collision_Route,
}
#assert(size_of(Narrow_Phase_Overlap_Context) == 24);
narrow_phase_flush_worker_chunk_single :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, candidate_count, candidate_start: int,
) -> Physics_Status
{
	context = runtime.default_context();
	deferred_count := 0;
	for candidate_index in 0 ..< candidate_count
	{
		pair_index := candidate_start + candidate_index;
		result := &narrow.results.memory[pair_index];
		if result.state != .Accepted
		{
			continue;
		}
		candidate := narrow.candidates.memory[pair_index];
		pair := collidable_pair_create(candidate.a, candidate.b);
		mapping_index := pair_cache_index_of(&narrow.pair_cache, pair);
		if mapping_index >= 0 && narrow_phase_try_update_builtin_convex_existing(
			narrow, pair, result, mapping_index,
		) == .Present
		{
			result.state = .Skipped;
			continue;
		}
		deferred_count += 1;
	}
	if deferred_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	transaction_reservation, reserve_status := narrow_phase_transaction_stream_reserve(
		narrow, worker_index, deferred_count,
	);
	if reserve_status != .Ok
	{
		return reserve_status;
	}
	transaction_count := 0;
	for candidate_index in 0 ..< candidate_count
	{
		pair_index := candidate_start + candidate_index;
		result := &narrow.results.memory[pair_index];
		if result.state != .Accepted
		{
			continue;
		}
		candidate := narrow.candidates.memory[pair_index];
		pair := collidable_pair_create(candidate.a, candidate.b);
		mapping_index := pair_cache_index_of(&narrow.pair_cache, pair);
		transaction_index, index_status := narrow_phase_transaction_reservation_index(
			transaction_reservation, transaction_count,
		);
		if index_status != .Ok
		{
			return index_status;
		}
		header := &narrow.transaction_headers.memory[transaction_index];
		contact := &narrow.contact_transactions.memory[transaction_index];
		if mapping_index < 0
		{
			awakening_set_index := i32(-1);
			if narrow.inactive_sets_present == .Present &&
			collidable_reference_mobility(pair.a) != .Static &&
			collidable_reference_mobility(pair.b) != .Static
			{
				handle_a := collidable_reference_raw_handle(pair.a);
				handle_b := collidable_reference_raw_handle(pair.b);
				location_a := narrow.bodies.handle_to_location.memory[handle_a];
				location_b := narrow.bodies.handle_to_location.memory[handle_b];
				if location_a.set_index != location_b.set_index
				{
					awakening_set_index = location_a.set_index;
					if awakening_set_index == BODIES_ACTIVE_SET_INDEX
					{
						awakening_set_index = location_b.set_index;
					}
					if awakening_set_index <= BODIES_ACTIVE_SET_INDEX
					{
						awakening_set_index = -1;
					}
				}
			}
			header^ = {
				pair=pair,
				payload_index=i32(transaction_index),
				pair_mapping_index=awakening_set_index,
				kind=.Pending,
			};
			intrinsics.mem_copy(
				&contact.description[0], result,
				size_of(Narrow_Phase_Pair_Result),
			);
			transaction_count += 1;
			continue;
		}
		header.payload_index = i32(transaction_index);
		deferred, update_status := narrow_phase_prepare_existing_pair_transaction(
			narrow, pair, result, mapping_index, header, contact,
		);
		if update_status != .Ok
		{
			return update_status;
		}
		_ = deferred;
		transaction_count += 1;
	}
	narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
	return .Ok;
}

narrow_phase_flush_worker_chunk_fused :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, candidate_count, candidate_start,
	captured_pair_count: int,
) -> Physics_Status
{
	context = runtime.default_context();
	batcher := &narrow.batchers[worker_index];
	if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
	captured_pair_count < 0 || captured_pair_count > candidate_count ||
	captured_pair_count > int(batcher.pairs.length) || candidate_start < 0 ||
	candidate_start > int(narrow.candidates.length) - candidate_count ||
	candidate_start > int(narrow.continuations.length) - candidate_count ||
	candidate_start > int(narrow.results.length) - candidate_count
	{
		return .Invalid_Argument;
	}
	candidate_end := candidate_start + candidate_count;
	for pair_slot in 0 ..< captured_pair_count
	{
		pair_index := int(batcher.pairs.memory[pair_slot].pair_id);
		if pair_index < candidate_start || pair_index >= candidate_end
		{
			return .Invalid_Description;
		}
	}
	deferred_records := ([^]Narrow_Phase_Fused_Deferred_Record)(
		rawptr(&narrow.results.memory[candidate_start])
	);
	deferred_count := 0;
	for pair_slot in 0 ..< captured_pair_count
	{
		captured_pair := &batcher.pairs.memory[pair_slot];
		pair_index := int(captured_pair.pair_id);
		continuation := &narrow.continuations.memory[pair_index];
		narrow_phase_rewind_stored_depths(continuation, &captured_pair.result);
		if captured_pair.result.convex.count == 0
		{
			continue;
		}
		candidate := narrow.candidates.memory[pair_index];
		pair := collidable_pair_create(candidate.a, candidate.b);
		mapping_index := pair_cache_index_of(&narrow.pair_cache, pair);
		if mapping_index >= 0 && narrow_phase_try_update_fused_builtin_convex_existing(
			narrow, pair, &captured_pair.result.convex, mapping_index,
		) == .Present
		{
			continue;
		}
		deferred_records[deferred_count] = {
			pair_slot=i32(pair_slot), mapping_index=i32(mapping_index),
		};
		deferred_count += 1;
	}
	if deferred_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	transaction_reservation, reserve_status := narrow_phase_transaction_stream_reserve(
		narrow, worker_index, deferred_count,
	);
	if reserve_status != .Ok
	{
		return reserve_status;
	}
	for deferred_index in 0 ..< deferred_count
	{
		record := deferred_records[deferred_index];
		pair_slot := int(record.pair_slot);
		mapping_index := int(record.mapping_index);
		captured_pair := &batcher.pairs.memory[pair_slot];
		candidate := narrow.candidates.memory[captured_pair.pair_id];
		pair := collidable_pair_create(candidate.a, candidate.b);
		transaction_index, index_status := narrow_phase_transaction_reservation_index(
			transaction_reservation, deferred_index,
		);
		if index_status != .Ok
		{
			return index_status;
		}
		header := &narrow.transaction_headers.memory[transaction_index];
		transaction := &narrow.contact_transactions.memory[transaction_index];
		header^ = {
			pair=pair,
			payload_index=i32(transaction_index),
			pair_mapping_index=i32(mapping_index),
			kind=.None,
		};
		narrow_phase_write_fused_stored_description_trusted(
			narrow, pair, &captured_pair.result, header, transaction,
		);
		if mapping_index >= 0
		{
			prepare_status := narrow_phase_prepare_fused_existing_mapping(
				narrow, mapping_index, header, transaction,
			);
			if prepare_status != .Ok
			{
				return prepare_status;
			}
			continue;
		}
		header.pair_mapping_index = narrow_phase_fused_awakening_set_index(narrow, pair);
		header.kind = .Pending_Prepared;
	}
	narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
	return .Ok;
}

narrow_phase_box_direct_test_group :: #force_no_inline proc "contextless" (
	direct_records: [^]Narrow_Phase_Convex_Direct_Record,
	group_start, group_count: int, offset_b: ^util.Vector3_Wide,
	wide: ^Convex_4_Contact_Manifold_Wide,
)
{
	tail := &direct_records[group_start + group_count - 1];
	a := Box_Wide{
		half_width=util.F32x8((^Box)(tail.shape_data_a).half_width),
		half_height=util.F32x8((^Box)(tail.shape_data_a).half_height),
		half_length=util.F32x8((^Box)(tail.shape_data_a).half_length),
	};
	b := Box_Wide{
		half_width=util.F32x8((^Box)(tail.shape_data_b).half_width),
		half_height=util.F32x8((^Box)(tail.shape_data_b).half_height),
		half_length=util.F32x8((^Box)(tail.shape_data_b).half_length),
	};
	offset_b^ = util.vector3_wide_broadcast(
		util.vector3_subtract(tail.pose_b.position, tail.pose_a.position),
	);
	orientation_a := util.quaternion_wide_broadcast(tail.pose_a.orientation);
	orientation_b := util.quaternion_wide_broadcast(tail.pose_b.orientation);
	speculative_margin := util.F32x8(tail.speculative_margin);
	for lane in 0 ..< group_count
	{
		record := &direct_records[group_start + lane];
		// This gather owns the complete transient bundle. Indexed scalar stores avoid
		// rebuilding every SIMD vector with a broadcast and blend for each lane.
		box_a := (^Box)(record.shape_data_a);
		box_b := (^Box)(record.shape_data_b);
		simd.to_array_ptr(&a.half_width)[lane] = box_a.half_width;
		simd.to_array_ptr(&a.half_height)[lane] = box_a.half_height;
		simd.to_array_ptr(&a.half_length)[lane] = box_a.half_length;
		simd.to_array_ptr(&b.half_width)[lane] = box_b.half_width;
		simd.to_array_ptr(&b.half_height)[lane] = box_b.half_height;
		simd.to_array_ptr(&b.half_length)[lane] = box_b.half_length;
		local_offset := util.vector3_subtract(record.pose_b.position, record.pose_a.position);
		simd.to_array_ptr(&offset_b.x)[lane] = local_offset.x;
		simd.to_array_ptr(&offset_b.y)[lane] = local_offset.y;
		simd.to_array_ptr(&offset_b.z)[lane] = local_offset.z;
		simd.to_array_ptr(&orientation_a.x)[lane] = record.pose_a.orientation.x;
		simd.to_array_ptr(&orientation_a.y)[lane] = record.pose_a.orientation.y;
		simd.to_array_ptr(&orientation_a.z)[lane] = record.pose_a.orientation.z;
		simd.to_array_ptr(&orientation_a.w)[lane] = record.pose_a.orientation.w;
		simd.to_array_ptr(&orientation_b.x)[lane] = record.pose_b.orientation.x;
		simd.to_array_ptr(&orientation_b.y)[lane] = record.pose_b.orientation.y;
		simd.to_array_ptr(&orientation_b.z)[lane] = record.pose_b.orientation.z;
		simd.to_array_ptr(&orientation_b.w)[lane] = record.pose_b.orientation.w;
		simd.to_array_ptr(&speculative_margin)[lane] = record.speculative_margin;
	}
	active := transmute(util.I32x8)simd.lanes_lt(
		util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(group_count)),
	);
	box_pair_test_wide_core(
		a, b, speculative_margin, offset_b^, orientation_a, orientation_b,
		active, group_count, wide,
	);
}

narrow_phase_convex_direct_execute_group :: #force_no_inline proc "contextless" (
	narrow: ^Narrow_Phase, direct_records: [^]Narrow_Phase_Convex_Direct_Record,
	group_start, group_count: int,
	deferred_records: [^]Narrow_Phase_Convex_Deferred_Record,
	deferred_count: ^int, $hull_pairs: bool,
) -> Physics_Status
{
	offset_b: util.Vector3_Wide = ---;
	wide: Convex_4_Contact_Manifold_Wide = ---;
	mapping_indices: [util.PRODUCTION_LANE_COUNT]int = ---;
	pairs: [util.PRODUCTION_LANE_COUNT]Collidable_Pair = ---;
	when !hull_pairs
	{
		// The mapping is structurally stable until the joined transaction commit.
		// Start the independent bucket reads before the wide geometry calculation.
		for lane in 0 ..< group_count
		{
			record := &direct_records[group_start + lane];
			candidate := narrow.candidates.memory[record.pair_id];
			pairs[lane] = collidable_pair_create(candidate.a, candidate.b);
			table_index := int(u32(util.hash_rehash(collidable_pair_hash(&pairs[lane])))) &
				narrow.pair_cache.mapping.table_mask;
			intrinsics.prefetch_read_data(&narrow.pair_cache.mapping.table.memory[table_index], 3);
		}
	}
	when hull_pairs
	{
		status := narrow_phase_hull_direct_test_group(direct_records, group_start, group_count, &offset_b, &wide);
		if status != .Ok
		{
			return status;
		}
	}
	else
	{
		narrow_phase_box_direct_test_group(direct_records, group_start, group_count, &offset_b, &wide);
	}
	lane_manifolds: [util.PRODUCTION_LANE_COUNT]Convex_Contact_Manifold = ---;
	_ = lane_manifolds;
	when hull_pairs
	{
		narrow_phase_hull_direct_unpack(&wide, offset_b, &lane_manifolds);
	}

	when hull_pairs
	{
		for lane in 0 ..< group_count
		{
			record := &direct_records[group_start + lane];
			candidate := narrow.candidates.memory[record.pair_id];
			pairs[lane] = collidable_pair_create(candidate.a, candidate.b);
			mapping_indices[lane] = -1;
			if lane_manifolds[lane].count > 0
			{
				mapping_indices[lane] = pair_cache_index_of(&narrow.pair_cache, pairs[lane]);
			}
		}
	}
	when !hull_pairs
	{
		// Resolve independent pairs before chasing their constraint locations.
		for lane in 0 ..< group_count
		{
			convex_4_manifold_wide_read_lane_core(&wide, offset_b, lane, &lane_manifolds[lane]);
			mapping_indices[lane] = -1;
			if lane_manifolds[lane].count > 0
			{
				mapping_index := pair_cache_index_of(&narrow.pair_cache, pairs[lane]);
				mapping_indices[lane] = mapping_index;
				if mapping_index >= 0
				{
					intrinsics.prefetch_read_data(&narrow.pair_cache.mapping.values.memory[mapping_index], 3);
				}
			}
		}
		for lane in 0 ..< group_count
		{
			mapping_index := mapping_indices[lane];
			if mapping_index >= 0
			{
				handle := narrow.pair_cache.mapping.values.memory[mapping_index].constraint_handle;
				intrinsics.prefetch_read_data(&narrow.solver.handle_to_constraint.memory[handle.value], 3);
			}
		}
	}
	for lane in 0 ..< group_count
	{
		record := &direct_records[group_start + lane];
		convex := lane_manifolds[lane];
		pair_index := int(record.pair_id);
		narrow_phase_rewind_convex_depths(
			&narrow.continuations.memory[pair_index], &convex, .Predecessor,
		);
		if convex.count == 0
		{
			continue;
		}
		pair, mapping_index := pairs[lane], mapping_indices[lane];
		if mapping_index >= 0 && narrow_phase_try_update_fused_builtin_convex_existing(
			narrow, pair, &convex, mapping_index,
		) == .Present
		{
			continue;
		}
		deferred := &deferred_records[deferred_count^];
		deferred^ = {};
		deferred.pair = pair;
		deferred.mapping_index = i32(mapping_index);
		deferred.manifold.kind = .Convex;
		deferred.manifold.convex.count = convex.count;
		deferred.manifold.convex.offset_b = convex.offset_b;
		deferred.manifold.convex.normal = convex.normal;
		for contact_index in 0 ..< int(convex.count)
		{
			deferred.manifold.convex.contacts[contact_index] = convex.contacts[contact_index];
		}
		deferred_count^ += 1;
	}
	return .Ok;
}

narrow_phase_flush_worker_chunk_convex_direct :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, candidate_count, candidate_start,
	direct_count: int, $hull_pairs: bool,
) -> Physics_Status
{
	context = runtime.default_context();
	if narrow == nil || worker_index < 0 || worker_index >= narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	batcher := &narrow.batchers[worker_index];
	defer batcher.pair_count = 0
	if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
	direct_count < 0 || direct_count > candidate_count || candidate_start < 0 ||
	candidate_start > int(narrow.candidates.length) - candidate_count ||
	candidate_start > int(narrow.continuations.length) - candidate_count ||
	candidate_start > int(narrow.results.length) - candidate_count
	{
		return .Invalid_Argument;
	}
	if batcher.state != .Ready || batcher.pairs.memory == nil ||
	direct_count > int(batcher.pairs.length)
	{
		return .Invalid_Argument;
	}
	direct_records := ([^]Narrow_Phase_Convex_Direct_Record)(rawptr(batcher.pairs.memory));
	candidate_end := candidate_start + candidate_count;
	for direct_index in 0 ..< direct_count
	{
		record := &direct_records[direct_index];
		pair_index := int(record.pair_id);
		if pair_index < candidate_start || pair_index >= candidate_end ||
		record.shape_data_a == nil || record.shape_data_b == nil ||
		record.speculative_margin < 0
		{
			return .Invalid_Description;
		}
	}
	if direct_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	deferred_records := ([^]Narrow_Phase_Convex_Deferred_Record)(
		rawptr(&narrow.results.memory[candidate_start])
	);
	deferred_count := 0;
	for group_start := 0; group_start < direct_count; group_start += util.PRODUCTION_LANE_COUNT
	{
		group_count := min(util.PRODUCTION_LANE_COUNT, direct_count - group_start);
		status := narrow_phase_convex_direct_execute_group(
			narrow, direct_records, group_start, group_count,
			deferred_records, &deferred_count, hull_pairs,
		);
		if status != .Ok
		{
			return status;
		}
	}
	if deferred_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	transaction_reservation, reserve_status := narrow_phase_transaction_stream_reserve(
		narrow, worker_index, deferred_count,
	);
	if reserve_status != .Ok
	{
		return reserve_status;
	}
	for deferred_index in 0 ..< deferred_count
	{
		record := &deferred_records[deferred_index];
		transaction_index, index_status := narrow_phase_transaction_reservation_index(
			transaction_reservation, deferred_index,
		);
		if index_status != .Ok
		{
			return index_status;
		}
		header := &narrow.transaction_headers.memory[transaction_index];
		transaction := &narrow.contact_transactions.memory[transaction_index];
		header^ = {
			pair=record.pair,
			payload_index=i32(transaction_index),
			pair_mapping_index=record.mapping_index,
			kind=.None,
		};
		narrow_phase_write_fused_stored_description_trusted(
			narrow, record.pair, &record.manifold, header, transaction,
		);
		if record.mapping_index >= 0
		{
			prepare_status := narrow_phase_prepare_fused_existing_mapping(
				narrow, int(record.mapping_index), header, transaction,
			);
			if prepare_status != .Ok
			{
				return prepare_status;
			}
			continue;
		}
		header.pair_mapping_index = narrow_phase_fused_awakening_set_index(
			narrow, record.pair,
		);
		header.kind = .Pending_Prepared;
	}
	narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
	return .Ok;
}

narrow_phase_box_sphere_test_box_group :: #force_no_inline proc "contextless" (
	narrow: ^Narrow_Phase, records: [^]Narrow_Phase_Box_Sphere_Record,
	pair_indices: ^[util.PRODUCTION_LANE_COUNT]i32, group_count, candidate_start, candidate_end: int,
) -> Physics_Status
{
	tail := &records[pair_indices[group_count - 1]];
	a := Box_Wide{
		half_width=util.F32x8((^Box)(tail.shape_data_a).half_width),
		half_height=util.F32x8((^Box)(tail.shape_data_a).half_height),
		half_length=util.F32x8((^Box)(tail.shape_data_a).half_length),
	};
	b := Box_Wide{
		half_width=util.F32x8((^Box)(tail.shape_data_b).half_width),
		half_height=util.F32x8((^Box)(tail.shape_data_b).half_height),
		half_length=util.F32x8((^Box)(tail.shape_data_b).half_length),
	};
	offset_b := util.vector3_wide_broadcast(util.vector3_subtract(tail.pose_b.position, tail.pose_a.position));
	orientation_a := util.quaternion_wide_broadcast(tail.pose_a.orientation);
	orientation_b := util.quaternion_wide_broadcast(tail.pose_b.orientation);
	speculative_margin := util.F32x8(tail.speculative_margin);
	for lane in 0 ..< group_count
	{
		record := &records[pair_indices[lane]];
		shape_a, shape_b := (^Box)(record.shape_data_a), (^Box)(record.shape_data_b);
		offset := util.vector3_subtract(record.pose_b.position, record.pose_a.position);
		simd.to_array_ptr(&a.half_width)[lane] = shape_a.half_width;
		simd.to_array_ptr(&a.half_height)[lane] = shape_a.half_height;
		simd.to_array_ptr(&a.half_length)[lane] = shape_a.half_length;
		simd.to_array_ptr(&b.half_width)[lane] = shape_b.half_width;
		simd.to_array_ptr(&b.half_height)[lane] = shape_b.half_height;
		simd.to_array_ptr(&b.half_length)[lane] = shape_b.half_length;
		simd.to_array_ptr(&offset_b.x)[lane] = offset.x;
		simd.to_array_ptr(&offset_b.y)[lane] = offset.y;
		simd.to_array_ptr(&offset_b.z)[lane] = offset.z;
		simd.to_array_ptr(&orientation_a.x)[lane] = record.pose_a.orientation.x;
		simd.to_array_ptr(&orientation_a.y)[lane] = record.pose_a.orientation.y;
		simd.to_array_ptr(&orientation_a.z)[lane] = record.pose_a.orientation.z;
		simd.to_array_ptr(&orientation_a.w)[lane] = record.pose_a.orientation.w;
		simd.to_array_ptr(&orientation_b.x)[lane] = record.pose_b.orientation.x;
		simd.to_array_ptr(&orientation_b.y)[lane] = record.pose_b.orientation.y;
		simd.to_array_ptr(&orientation_b.z)[lane] = record.pose_b.orientation.z;
		simd.to_array_ptr(&orientation_b.w)[lane] = record.pose_b.orientation.w;
		simd.to_array_ptr(&speculative_margin)[lane] = record.speculative_margin;
	}
	active := transmute(util.I32x8)simd.lanes_lt(
		util.I32x8{0, 1, 2, 3, 4, 5, 6, 7}, util.I32x8(i32(group_count)),
	);
	wide: Convex_4_Contact_Manifold_Wide = ---;
	box_pair_test_wide_core(a, b, speculative_margin, offset_b, orientation_a, orientation_b, active, group_count, &wide);
	for lane in 0 ..< group_count
	{
		pair_index := int(records[pair_indices[lane]].pair_id);
		if pair_index < candidate_start || pair_index >= candidate_end
		{
			return .Invalid_Description;
		}
		result := &narrow.results.memory[pair_index];
		result.manifold.kind = .Convex;
		status := #force_no_inline convex_4_manifold_wide_read_lane_into(&wide, offset_b, lane, &result.manifold.convex);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

narrow_phase_box_sphere_test_sphere_group :: #force_no_inline proc "contextless" (
	narrow: ^Narrow_Phase, records: [^]Narrow_Phase_Box_Sphere_Record,
	pair_indices: ^[util.PRODUCTION_LANE_COUNT]i32, group_count, candidate_start, candidate_end: int,
	$shape_type_b: int,
) -> Physics_Status
{
	tail := &records[pair_indices[group_count - 1]];
	tail_a, tail_b := tail.shape_data_a, tail.shape_data_b;
	tail_pose_a, tail_pose_b := tail.pose_a, tail.pose_b;
	tail_flip := i32(0);
	when shape_type_b == BOX_TYPE_ID
	{
		if tail.order == .Flipped
		{
			tail_a, tail_b = tail_b, tail_a;
			tail_pose_a, tail_pose_b = tail_pose_b, tail_pose_a;
			tail_flip = -1;
		}
	}
	a := Sphere_Wide{radius=util.F32x8((^Sphere)(tail_a).radius)};
	b_sphere: Sphere_Wide = ---;
	b_box: Box_Wide = ---;
	orientation_b: util.Quaternion_Wide = ---;
	flip_mask: util.I32x8 = ---;
	_ = b_sphere;
	_ = b_box;
	_ = orientation_b;
	_ = flip_mask;
	_ = tail_flip;
	when shape_type_b == BOX_TYPE_ID
	{
		b_box = {
			half_width=util.F32x8((^Box)(tail_b).half_width),
			half_height=util.F32x8((^Box)(tail_b).half_height),
			half_length=util.F32x8((^Box)(tail_b).half_length),
		};
		orientation_b = util.quaternion_wide_broadcast(tail_pose_b.orientation);
		flip_mask = util.I32x8(tail_flip);
	}
	else
	{
		b_sphere = {radius=util.F32x8((^Sphere)(tail_b).radius)};
	}
	offset_b := util.vector3_wide_broadcast(util.vector3_subtract(tail_pose_b.position, tail_pose_a.position));
	speculative_margin := util.F32x8(tail.speculative_margin);
	for lane in 0 ..< group_count
	{
		record := &records[pair_indices[lane]];
		shape_a, shape_b := record.shape_data_a, record.shape_data_b;
		pose_a, pose_b := record.pose_a, record.pose_b;
		when shape_type_b == BOX_TYPE_ID
		{
			flip := i32(0);
			if record.order == .Flipped
			{
				shape_a, shape_b = shape_b, shape_a;
				pose_a, pose_b = pose_b, pose_a;
				flip = -1;
			}
			simd.to_array_ptr(&flip_mask)[lane] = flip;
			simd.to_array_ptr(&b_box.half_width)[lane] = (^Box)(shape_b).half_width;
			simd.to_array_ptr(&b_box.half_height)[lane] = (^Box)(shape_b).half_height;
			simd.to_array_ptr(&b_box.half_length)[lane] = (^Box)(shape_b).half_length;
			simd.to_array_ptr(&orientation_b.x)[lane] = pose_b.orientation.x;
			simd.to_array_ptr(&orientation_b.y)[lane] = pose_b.orientation.y;
			simd.to_array_ptr(&orientation_b.z)[lane] = pose_b.orientation.z;
			simd.to_array_ptr(&orientation_b.w)[lane] = pose_b.orientation.w;
		}
		else
		{
			simd.to_array_ptr(&b_sphere.radius)[lane] = (^Sphere)(shape_b).radius;
		}
		simd.to_array_ptr(&a.radius)[lane] = (^Sphere)(shape_a).radius;
		offset := util.vector3_subtract(pose_b.position, pose_a.position);
		simd.to_array_ptr(&offset_b.x)[lane] = offset.x;
		simd.to_array_ptr(&offset_b.y)[lane] = offset.y;
		simd.to_array_ptr(&offset_b.z)[lane] = offset.z;
		simd.to_array_ptr(&speculative_margin)[lane] = record.speculative_margin;
	}
	wide: Convex_1_Contact_Manifold_Wide = ---;
	status: Physics_Status;
	when shape_type_b == BOX_TYPE_ID
	{
		wide, status = sphere_box_test_wide(a, b_box, speculative_margin, offset_b, orientation_b, group_count);
	}
	else
	{
		wide, status = sphere_pair_test_wide(a, b_sphere, speculative_margin, offset_b, group_count);
	}
	if status != .Ok
	{
		return status;
	}
	when shape_type_b == BOX_TYPE_ID
	{
		flip_status := convex_1_manifold_wide_apply_flip_mask(&wide, &offset_b, flip_mask);
		if flip_status != .Ok
		{
			return flip_status;
		}
	}
	for lane in 0 ..< group_count
	{
		pair_index := int(records[pair_indices[lane]].pair_id);
		if pair_index < candidate_start || pair_index >= candidate_end
		{
			return .Invalid_Description;
		}
		result := &narrow.results.memory[pair_index];
		result.manifold.kind = .Convex;
		read_status := #force_no_inline convex_1_manifold_wide_read_lane_into(&wide, offset_b, lane, &result.manifold.convex);
		if read_status != .Ok
		{
			return read_status;
		}
	}
	return .Ok;
}

narrow_phase_flush_worker_chunk_box_sphere :: proc "contextless" (
	narrow: ^Narrow_Phase, worker_index, candidate_count, candidate_start, direct_count: int,
) -> Physics_Status
{
	context = runtime.default_context();
	if narrow == nil || worker_index < 0 || worker_index >= narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	batcher := &narrow.batchers[worker_index];
	defer
	{
		batcher.pair_count = 0;
		collision_batcher_reset_task_lists(batcher);
	}
	if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
	direct_count < 0 || direct_count > candidate_count || candidate_start < 0 ||
	candidate_start > int(narrow.candidates.length) - candidate_count ||
	candidate_start > int(narrow.continuations.length) - candidate_count ||
	candidate_start > int(narrow.results.length) - candidate_count ||
	batcher.state != .Ready || batcher.pairs.memory == nil ||
	direct_count > int(batcher.pairs.length)
	{
		return .Invalid_Argument;
	}
	if direct_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	records := ([^]Narrow_Phase_Box_Sphere_Record)(rawptr(batcher.pairs.memory));
	candidate_end := candidate_start + candidate_count;
	// private links retain task order. 32-pair builtin batches consist of four identical eight-lane groups
	for task_index in 0 ..< narrow.tasks.task_count
	{
		pair_index := batcher.task_heads[task_index];
		if pair_index < 0
		{
			continue;
		}
		task := &narrow.tasks.tasks[task_index];
		for pair_index >= 0
		{
			pair_indices: [util.PRODUCTION_LANE_COUNT]i32 = ---;
			group_count := 0;
			for pair_index >= 0 && group_count < util.PRODUCTION_LANE_COUNT
			{
				if int(pair_index) >= direct_count
				{
					return .Invalid_Description;
				}
				pair_indices[group_count] = pair_index;
				group_count += 1;
				pair_index = records[pair_index].next_in_task;
			}
			status: Physics_Status;
			if task.shape_type_a == BOX_TYPE_ID
			{
				status = narrow_phase_box_sphere_test_box_group(
					narrow, records, &pair_indices, group_count, candidate_start, candidate_end,
				);
			}
			else if task.shape_type_b == BOX_TYPE_ID
			{
				status = narrow_phase_box_sphere_test_sphere_group(
					narrow, records, &pair_indices, group_count, candidate_start, candidate_end, BOX_TYPE_ID,
				);
			}
			else
			{
				status = narrow_phase_box_sphere_test_sphere_group(
					narrow, records, &pair_indices, group_count, candidate_start, candidate_end, SPHERE_TYPE_ID,
				);
			}
			if status != .Ok
			{
				return status;
			}
		}
	}
	deferred_records := ([^]Narrow_Phase_Convex_Deferred_Record)(rawptr(&narrow.results.memory[candidate_start]));
	deferred_count := 0;
	for record_index in 0 ..< direct_count
	{
		record := &records[record_index];
		pair_index := int(record.pair_id);
		result := &narrow.results.memory[pair_index];
		narrow_phase_rewind_convex_depths(&narrow.continuations.memory[pair_index], &result.manifold.convex, .Box_Sphere_Direct);
		if result.manifold.convex.count == 0
		{
			result.state = .Rejected;
			continue;
		}
		result.material = narrow.stored_completion_material;
		result.state = .Accepted;
		candidate := narrow.candidates.memory[pair_index];
		pair := collidable_pair_create(candidate.a, candidate.b);
		mapping_index := pair_cache_index_of(&narrow.pair_cache, pair);
		if mapping_index >= 0 && narrow_phase_try_update_fused_builtin_convex_existing(
			narrow, pair, &result.manifold.convex, mapping_index,
		) == .Present
		{
			result.state = .Skipped;
			continue;
		}
		// read before compaction: deferred_count <= record_index <= pair_index - candidate_start
		convex := result.manifold.convex;
		deferred := &deferred_records[deferred_count];
		deferred^ = {};
		deferred.pair = pair;
		deferred.mapping_index = i32(mapping_index);
		deferred.manifold.kind = .Convex;
		deferred.manifold.convex = convex;
		deferred_count += 1;
	}
	if deferred_count == 0
	{
		narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
		return .Ok;
	}
	transaction_reservation, reserve_status := narrow_phase_transaction_stream_reserve(
		narrow, worker_index, deferred_count,
	);
	if reserve_status != .Ok
	{
		return reserve_status;
	}
	for deferred_index in 0 ..< deferred_count
	{
		record := &deferred_records[deferred_index];
		transaction_index, index_status := narrow_phase_transaction_reservation_index(
			transaction_reservation, deferred_index,
		);
		if index_status != .Ok
		{
			return index_status;
		}
		header := &narrow.transaction_headers.memory[transaction_index];
		transaction := &narrow.contact_transactions.memory[transaction_index];
		header^ = {
			pair=record.pair,
			payload_index=i32(transaction_index),
			pair_mapping_index=record.mapping_index,
			kind=.None,
		};
		narrow_phase_write_fused_stored_description_trusted(
			narrow, record.pair, &record.manifold, header, transaction,
		);
		if record.mapping_index >= 0
		{
			prepare_status := narrow_phase_prepare_fused_existing_mapping(
				narrow, int(record.mapping_index), header, transaction,
			);
			if prepare_status != .Ok
			{
				return prepare_status;
			}
			continue;
		}
		header.pair_mapping_index = narrow_phase_fused_awakening_set_index(
			narrow, record.pair,
		);
		header.kind = .Pending_Prepared;
	}
	narrow.transaction_streams[worker_index].traversal_candidate_count = 0;
	return .Ok;
}

narrow_phase_flush_worker_chunk :: proc "contextless" (
	ctx: ^Narrow_Phase_Overlap_Context, worker_index: int,
	$record_route: Narrow_Phase_Collision_Route,
) -> Physics_Status
{
	context = runtime.default_context();
	if ctx == nil || ctx.narrow == nil || worker_index < 0 ||
	worker_index >= ctx.narrow.active_worker_count || ctx.worker_count <= 0 ||
	ctx.worker_count > ctx.narrow.active_worker_count
	{
		return .Invalid_Argument;
	}
	narrow := ctx.narrow;
	candidate_count := int(
		narrow.transaction_streams[worker_index].traversal_candidate_count,
	);
	batcher := &narrow.batchers[worker_index];
	if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
	batcher.pair_count < 0 || batcher.pair_count > candidate_count
	{
		return .Invalid_Argument;
	}
	if candidate_count == 0
	{
		return .Ok;
	}
	when record_route == .Box_Sphere_Direct
	{
		candidate_start: int = worker_index * narrow.pending_capacity_per_worker;
		return narrow_phase_flush_worker_chunk_box_sphere(
			narrow, worker_index, candidate_count, candidate_start, batcher.pair_count,
		);
	}
	else
	{
		if ctx.fused_route == .Predecessor
		{
			flush_status := collision_batcher_flush(batcher);
			if flush_status != .Ok
			{
				return flush_status;
			}
			candidate_start := worker_index * narrow.pending_capacity_per_worker;
			return narrow_phase_flush_worker_chunk_single(
				narrow, worker_index, candidate_count, candidate_start,
			);
		}
		candidate_start := worker_index * narrow.pending_capacity_per_worker;
		if ctx.fused_route == .Hull_Direct
		{
			return narrow_phase_flush_worker_chunk_convex_direct(
				narrow, worker_index, candidate_count, candidate_start, batcher.pair_count, true,
			);
		}
		return narrow_phase_flush_worker_chunk_convex_direct(
			narrow, worker_index, candidate_count, candidate_start, batcher.pair_count, false,
		);
	}
}

narrow_phase_pair_traversal_finalize :: #force_inline proc "contextless" (
	$record_route: Narrow_Phase_Collision_Route,
) -> Tree_Parallel_Worker_Finalize_Proc
{
	return proc "contextless" (
		user_context: rawptr, worker_index: int,
	) -> Physics_Status
	{
		ctx: ^Narrow_Phase_Overlap_Context = (^Narrow_Phase_Overlap_Context)(user_context);
		if ctx == nil || ctx.narrow == nil || worker_index < 0 ||
		worker_index >= ctx.worker_count
		{
			return .Invalid_Argument;
		}
		return narrow_phase_flush_worker_chunk(ctx, worker_index, record_route);
	};
}

// select a callback at collision setup. the compile-time route adds no callback argument
narrow_phase_overlap_visitor :: #force_inline proc "contextless" (
	$record_route: Narrow_Phase_Collision_Route,
) -> Broad_Phase_Pair_Visitor_Proc
{
	return proc "contextless" (
		user_context: rawptr, worker_index: int, pair: Broad_Phase_Pair,
	) -> Physics_Status
	{
		ctx := (^Narrow_Phase_Overlap_Context)(user_context);
		if ctx == nil || ctx.narrow == nil || worker_index < 0 ||
		worker_index >= ctx.worker_count
		{
			return .Invalid_Argument;
		}
		canonical_pair := collidable_pair_create(pair.a, pair.b);
		canonical_candidate := Broad_Phase_Pair{
			canonical_pair.a, canonical_pair.b,
		};
		narrow := ctx.narrow;
		candidate_count := int(
			narrow.transaction_streams[worker_index].traversal_candidate_count,
		);
		batcher := &narrow.batchers[worker_index];
		if candidate_count < 0 || candidate_count > narrow.pending_capacity_per_worker ||
		batcher.pair_count < 0 || batcher.pair_count > candidate_count
		{
			return .Invalid_Argument;
		}
		if candidate_count >= narrow.pending_capacity_per_worker
		{
			flush_status := narrow_phase_flush_worker_chunk(ctx, worker_index, record_route);
			if flush_status != .Ok
			{
				return flush_status;
			}
			candidate_count = 0;
		}
		pair_index := worker_index * narrow.pending_capacity_per_worker + candidate_count;
		if pair_index < 0 || pair_index >= int(narrow.candidates.length)
		{
			return .Capacity_Missing;
		}
		narrow.candidates.memory[pair_index] = canonical_candidate;
		when record_route == .Box_Sphere_Direct
		{
			narrow.results.memory[pair_index].state = .Skipped;
		}
		else
		{
			if ctx.fused_route == .Predecessor
			{
				narrow.results.memory[pair_index].state = .Skipped;
			}
		}
		queue_route := ctx.fused_route;
		when record_route == .Box_Sphere_Direct
		{
			queue_route = .Box_Sphere_Direct;
		}
		queue_status := narrow_phase_queue_candidate(
			narrow, pair_index, worker_index, ctx.dt, queue_route, record_route,
		);
		if queue_status != .Ok
		{
			return queue_status;
		}
		if batcher.pair_count < 0 || batcher.pair_count > candidate_count + 1 ||
		batcher.pair_count > narrow.pending_capacity_per_worker
		{
			return .Invalid_Argument;
		}
		narrow.transaction_streams[worker_index].traversal_candidate_count =
		i32(candidate_count + 1);
		return .Ok;
	};
}

narrow_phase_awaken_contact_sets :: proc (
	narrow: ^Narrow_Phase, transaction_count: int,
	dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
) -> Physics_Status
{
	if narrow == nil || narrow.awakener == nil
	{
		return .Invalid_Description;
	}
	if narrow.inactive_sets_present == .Missing
	{
		return .Ok;
	}
	generation_status := narrow_phase_advance_claim_generation(narrow);
	if generation_status != .Ok
	{
		return generation_status;
	}
	unique_set_count := 0;
	for order_index in 0 ..< transaction_count
	{
		transaction_index := int(narrow.transaction_order.memory[order_index]);
		header := &narrow.transaction_headers.memory[transaction_index];
		if (header.kind != .Pending && header.kind != .Pending_Prepared) ||
		header.pair_mapping_index <= BODIES_ACTIVE_SET_INDEX
		{
			continue;
		}
		set_index := int(header.pair_mapping_index);
		if narrow.pair_table_claim_generations.memory[set_index] == narrow.claim_generation
		{
			continue;
		}
		narrow.pair_table_claim_generations.memory[set_index] = narrow.claim_generation;
		narrow.touched_claim_indices.memory[unique_set_count] = i32(set_index);
		unique_set_count += 1;
	}
	if unique_set_count > 1
	{
		_ = util.quick_sort_keys(
			narrow.touched_claim_indices.memory, 0, unique_set_count - 1,
			util.primitive_i32_comparer(),
		);
	}
	for set_order_index in 0 ..< unique_set_count
	{
		set_index := int(narrow.touched_claim_indices.memory[set_order_index]);
		status := island_awakener_awaken_set(
			narrow.awakener, set_index, dispatcher, .Missing,
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

narrow_phase_execute_mode :: proc (
	narrow: ^Narrow_Phase, dispatcher: ^util.Thread_Dispatcher_Boundary = nil,
	dt: f32 = 1.0 / 60.0,
	$with_triggers: Trigger_Selection, triggers: ^Trigger_System = nil, $mixed: Reference_State,
) -> Physics_Status
{
	if narrow == nil || narrow.state != .Ready || narrow.awakener == nil || dt <= 0
	{
		return .Disposed;
	}
	worker_count := 1;
	if dispatcher != nil
	{
		if dispatcher.worker_count <= 0 ||
		dispatcher.worker_count > narrow.active_worker_count
		{
			return .Capacity_Missing;
		}
		worker_count = dispatcher.worker_count;
	}
	if worker_count > MAXIMUM_SOLVER_WORKER_COUNT
	{
		return .Capacity_Missing;
	}
	worker_pools: [MAXIMUM_SOLVER_WORKER_COUNT]^util.Buffer_Pool;
	for worker_index in 0 ..< worker_count
	{
		pool := narrow.pool;
		if dispatcher != nil
		{
			if dispatcher.worker_pool == nil
			{
				return .Invalid_Argument;
			}
			pool_status: util.Threading_Status;
			pool, pool_status = dispatcher.worker_pool(dispatcher, worker_index);
			if pool_status != .Ok
			{
				return .Invalid_Argument;
			}
		}
		if pool == nil || pool.state != .Ready
		{
			return .Invalid_Argument;
		}
		worker_pools[worker_index] = pool;
	}
	capacity_status := narrow_phase_ensure_runtime_contact_capacity(narrow);
	if capacity_status != .Ok
	{
		return capacity_status;
	}
	prepare_status := pair_cache_prepare(&narrow.pair_cache);
	if prepare_status != .Ok
	{
		return prepare_status;
	}
	batcher_status := narrow_phase_prepare_batchers(narrow, worker_count);
	if batcher_status != .Ok
	{
		narrow.pair_cache.state = .Ready;
		return batcher_status;
	}
	overlap_context := Narrow_Phase_Overlap_Context{
		narrow=narrow, worker_count=worker_count, dt=dt,
	};
	overlap_context.fused_route = narrow_phase_collision_route_state(
		narrow, worker_count,
	);
	if overlap_context.fused_route == .Predecessor
	{
		overlap_context.fused_route = #force_no_inline narrow_phase_box_sphere_route_state(
			narrow, worker_count,
		);
	}
	for worker_index in 0 ..< worker_count
	{
		narrow.batchers[worker_index].traversal_pool = worker_pools[worker_index];
		when mixed == .Present
		{
			narrow.batchers[worker_index].procedures.allow_child_pair = mixed_narrow_allow_child;
		}
	}
	defer
	{
		for worker_index in 0 ..< worker_count
		{
			if overlap_context.fused_route != .Predecessor
			{
				narrow.batchers[worker_index].pair_count = 0;
			}
			if overlap_context.fused_route == .Box_Sphere_Direct
			{
				collision_batcher_reset_task_lists(&narrow.batchers[worker_index]);
			}
			narrow.batchers[worker_index].traversal_pool = narrow.pool;
		}
	}
	narrow.last_stage = .Prepared;
	narrow.state = .Stepping;
	stream_reset_status := narrow_phase_reset_transaction_streams(
		narrow, worker_count,
	);
	if stream_reset_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return stream_reset_status;
	}
	narrow.inactive_sets_present = .Missing;
	for set_index in 1 ..< int(narrow.bodies.sets.length)
	{
		set := &narrow.bodies.sets.memory[set_index];
		if set.state == .Allocated && set.count > 0
		{
			narrow.inactive_sets_present = .Present;
			break;
		}
	}
	for worker_index in 0 ..< worker_count
	{
		narrow.candidate_counts[worker_index] = 0;
		narrow.transaction_statuses[worker_index] = .Ok;
	}
	overlap_visitor: Broad_Phase_Pair_Visitor_Proc = narrow_phase_overlap_visitor(.Predecessor);
	worker_finalize: Tree_Parallel_Worker_Finalize_Proc = narrow_phase_pair_traversal_finalize(.Predecessor);
	if overlap_context.fused_route == .Box_Sphere_Direct
	{
		overlap_visitor = narrow_phase_overlap_visitor(.Box_Sphere_Direct);
		worker_finalize = narrow_phase_pair_traversal_finalize(.Box_Sphere_Direct);
	}
	visit_status: Physics_Status;
	when with_triggers == .Enabled
	{
		trigger_context: Trigger_Overlap_Context = Trigger_Overlap_Context{base=&overlap_context, triggers=triggers};
		trigger_visitor: Broad_Phase_Pair_Visitor_Proc = trigger_overlap_visitor(.Predecessor, mixed);
		if overlap_context.fused_route == .Box_Sphere_Direct
		{
			trigger_visitor = trigger_overlap_visitor(.Box_Sphere_Direct, mixed);
		}
		visit_status = broad_phase_visit_pairs(narrow.broad_phase, trigger_visitor, &trigger_context, dispatcher, worker_finalize, &overlap_context);
		when mixed == .Present
		{
			if visit_status == .Ok
			{
				for index in 0..<worker_count
				{
					if narrow.transaction_statuses[index] != .Ok
					{
						visit_status = narrow.transaction_statuses[index];
						break;
					}
				}
			}
		}
		if visit_status == .Ok
		{
			visit_status = trigger_detect(triggers, dispatcher, worker_count, mixed);
		}
	}
	else
	{
		visit_status = broad_phase_visit_pairs(narrow.broad_phase, overlap_visitor, &overlap_context, dispatcher, worker_finalize, &overlap_context);
	}
	if visit_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return visit_status;
	}
	narrow.last_stage = .Pairs_Found;
	narrow.last_stage = .Pairs_Queued;
	narrow.last_stage = .Batch_Flushed;
	current_transaction_count, order_status := narrow_phase_prepare_transaction_order(
		narrow, worker_count,
	);
	if order_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return order_status;
	}
	wake_status := narrow_phase_awaken_contact_sets(
		narrow, current_transaction_count, dispatcher,
	);
	if wake_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return wake_status;
	}
	preflight_counts: [MAXIMUM_SOLVER_WORKER_COUNT]Narrow_Phase_Preflight_Worker_Counts;
	transaction_count, preflight_summary, transaction_status := narrow_phase_prepare_transactions(
		narrow, current_transaction_count,
		narrow.transaction_stream_storage_capacity,
		worker_count, dispatcher, &preflight_counts,
	);
	if transaction_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return transaction_status;
	}
	preflight_status := narrow_phase_preflight_transactions(
		narrow, transaction_count, worker_count, preflight_summary,
	);
	if preflight_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return preflight_status;
	}
	commit_status := narrow_phase_commit_transactions(narrow, transaction_count, worker_count, dispatcher);
	if commit_status != .Ok
	{
		narrow.state = .Ready;
		narrow.pair_cache.state = .Ready;
		return commit_status;
	}
	narrow.last_stage = .Stale_Removed;
	narrow.state = .Ready;
	narrow.last_stage = .Changes_Flushed;
	return .Ok;
}

narrow_phase_hull_direct_test_group :: #force_no_inline proc "contextless" (
	direct_records: [^]Narrow_Phase_Convex_Direct_Record,
	group_start, group_count: int, offset_b: ^util.Vector3_Wide,
	wide: ^Convex_4_Contact_Manifold_Wide,
) -> Physics_Status
{
	tail := &direct_records[group_start + group_count - 1];
	a, b: Convex_Hull_Wide;
	offset_b^ = util.vector3_wide_broadcast(util.vector3_subtract(tail.pose_b.position, tail.pose_a.position));
	orientation_a := util.quaternion_wide_broadcast(tail.pose_a.orientation);
	orientation_b := util.quaternion_wide_broadcast(tail.pose_b.orientation);
	speculative_margin := util.F32x8(tail.speculative_margin);
	for lane in 0 ..< group_count
	{
		record := &direct_records[group_start + lane];
		a.hulls[lane] = (^Convex_Hull)(record.shape_data_a);
		b.hulls[lane] = (^Convex_Hull)(record.shape_data_b);
		util.vector3_wide_write_slot(offset_b, lane, util.vector3_subtract(record.pose_b.position, record.pose_a.position));
		util.quaternion_wide_write_slot(&orientation_a, lane, record.pose_a.orientation);
		util.quaternion_wide_write_slot(&orientation_b, lane, record.pose_b.orientation);
		speculative_margin = simd.replace(speculative_margin, lane, record.speculative_margin);
	}
	manifold, status := convex_hull_pair_test_wide(a, b, speculative_margin, offset_b^, orientation_a, orientation_b, group_count);
	wide^ = manifold;
	return status;
}

// hull clipping produces a packed contact prefix. transpose once at this
// boundary instead of repeatedly extracting variable SIMD lanes during updates
narrow_phase_hull_direct_unpack :: #force_no_inline proc "contextless" (
	wide: ^Convex_4_Contact_Manifold_Wide, offset_b: util.Vector3_Wide,
	manifolds: ^[util.PRODUCTION_LANE_COUNT]Convex_Contact_Manifold,
)
{
	counts := simd.neg(simd.add(simd.add(wide.contact_0_exists, wide.contact_1_exists), simd.add(wide.contact_2_exists, wide.contact_3_exists)));
	#unroll for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		manifold := &manifolds[lane];
		manifold.count = simd.extract(counts, lane);
		manifold.offset_b = util.vector3_wide_read_slot(offset_b, lane);
		manifold.normal = util.vector3_wide_read_slot(wide.normal, lane);
		manifold.contacts[0] = {offset=util.vector3_wide_read_slot(wide.offset_a_0, lane), depth=simd.extract(wide.depth_0, lane), feature_id=simd.extract(wide.feature_id_0, lane)};
		manifold.contacts[1] = {offset=util.vector3_wide_read_slot(wide.offset_a_1, lane), depth=simd.extract(wide.depth_1, lane), feature_id=simd.extract(wide.feature_id_1, lane)};
		manifold.contacts[2] = {offset=util.vector3_wide_read_slot(wide.offset_a_2, lane), depth=simd.extract(wide.depth_2, lane), feature_id=simd.extract(wide.feature_id_2, lane)};
		manifold.contacts[3] = {offset=util.vector3_wide_read_slot(wide.offset_a_3, lane), depth=simd.extract(wide.depth_3, lane), feature_id=simd.extract(wide.feature_id_3, lane)};
	}
}

// the disabled specialization has no trigger membership or result work in its loops
narrow_phase_execute :: proc(narrow: ^Narrow_Phase, dispatcher: ^util.Thread_Dispatcher_Boundary = nil, dt: f32 = 1.0 / 60.0) -> Physics_Status
{
	return narrow_phase_execute_mode(narrow, dispatcher, dt, .Disabled, nil, .Missing);
}
