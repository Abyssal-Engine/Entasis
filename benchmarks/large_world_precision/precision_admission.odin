package large_world_precision

@(require) import "core:fmt"
@(require) import entasis "entasis:entasis"
@(require) import support "../benchmark_support"

when PRECISION_NATIVE_GLOBAL
{
	Precision_Contact_Errors :: struct
	{
		depth: f64,
		offset: f64,
		normal: f64,
		contacts: int,
	}

	// compare geometry, not solver batch/lane order or accumulated impulses
	// a used-bit mask makes this a multiset comparison, including duplicate IDs
	precision_contact_geometry_matches :: proc "contextless" (
		a, b: ^entasis.Solver_Contact_Data, errors: ^Precision_Contact_Errors,
	) -> Precision_Admission
	{
		if a.kind != b.kind || a.body_count != b.body_count || a.contact_count != b.contact_count ||
			a.material != b.material ||
			precision_vector_close(a.offset_b, b.offset_b, POSITION_TOLERANCE) != .Accepted
		{
			return .Rejected;
		}
		errors.offset = max(errors.offset, f64(abs(a.offset_b.x - b.offset_b.x)),
			f64(abs(a.offset_b.y - b.offset_b.y)), f64(abs(a.offset_b.z - b.offset_b.z)));
		used: u32;
		for ai in 0 ..< int(a.contact_count)
		{
			ac: entasis.Solver_Contact_Point = a.contacts[ai];
			matched: int = -1;
			for bi in 0 ..< int(b.contact_count)
			{
				bc: entasis.Solver_Contact_Point = b.contacts[bi];
				if used & (u32(1) << u32(bi)) == 0 && ac.feature_id == bc.feature_id &&
					precision_close(f64(ac.depth), f64(bc.depth), POSITION_TOLERANCE) == .Accepted &&
					precision_vector_close(ac.offset_a, bc.offset_a, POSITION_TOLERANCE) == .Accepted &&
					precision_vector_close(ac.normal, bc.normal, 0.0001) == .Accepted
				{
					matched = bi;
					break;
				}
			}
			if matched < 0
			{
				return .Rejected;
			}
			used |= u32(1) << u32(matched);
			bc: entasis.Solver_Contact_Point = b.contacts[matched];
			errors.depth = max(errors.depth, f64(abs(ac.depth - bc.depth)));
			errors.offset = max(errors.offset, f64(abs(ac.offset_a.x - bc.offset_a.x)),
				f64(abs(ac.offset_a.y - bc.offset_a.y)), f64(abs(ac.offset_a.z - bc.offset_a.z)));
			errors.normal = max(errors.normal, f64(abs(ac.normal.x - bc.normal.x)),
				f64(abs(ac.normal.y - bc.normal.y)), f64(abs(ac.normal.z - bc.normal.z)));
			errors.contacts += 1;
		}
		return .Accepted;
	}

	precision_contact_pair_matches :: proc "contextless" (
		far, near: ^entasis.Solver_Contact_Data, body_base: i32,
		far_floor, near_floor: entasis.Collidable_Reference,
	) -> Precision_Admission
	{
		if far.body_count != near.body_count
		{
			return .Rejected;
		}
		for index in 0 ..< int(far.body_count)
		{
			if far.body_handles[index].value != near.body_handles[index].value + body_base
			{
				return .Rejected;
			}
		}
		if far.body_count == 1 && !((far.pair_a == far_floor && near.pair_a == near_floor) ||
			(far.pair_b == far_floor && near.pair_b == near_floor))
		{
			return .Rejected;
		}
		return .Accepted;
	}

	// the fixture has at most six face neighbours and one floor per body in
	// these eight frames. overflow is an admission failure, never truncation
	// public connectivity APIs avoid depending on the opaque world layout
	precision_replay_compare_group :: proc(
		owner: ^Benchmark_Owner, region: ^Replay_Group, group: int,
		errors: ^Precision_Contact_Errors,
	) -> (count: int, result: support.Benchmark_Result)
	{
		base: int = group * DISTRIBUTED_BODY_COUNT;
		far_floor, far_status := entasis.static_collidable_reference(&owner.world, {i32(group)});
		near_floor, near_status := entasis.static_collidable_reference(&region.world, {0});
		if far_status != .Ok || near_status != .Ok
		{
			return 0, {status=.Validation_Failed};
		}
		for near_handle, local_index in region.bodies
		{
			far_handles, near_handles: [32]entasis.Constraint_Handle;
			far_handle: entasis.Body_Handle = owner.bodies[base + local_index];
			far_count, _, far_list_status := entasis.body_constraints(&owner.world, far_handle, far_handles[:]);
			near_count, _, near_list_status := entasis.body_constraints(&region.world, near_handle, near_handles[:]);
			if far_list_status != .Ok || near_list_status != .Ok || far_count != near_count
			{
				fmt.eprintfln("precision_replay_pair_set_failed group=%d body=%d far_count=%d near_count=%d far_status=%v near_status=%v",
					group, local_index, far_count, near_count, far_list_status, near_list_status);
				return count, {status=.Validation_Failed};
			}
			for handle in far_handles[:far_count]
			{
				far, status := entasis.solver_contact_data(&owner.world, handle);
				if status != .Ok
				{
					return count, {status=.Validation_Failed, physics_status=status};
				}
				if far.body_handles[0] != far_handle
				{
					continue;
				}
				matched: Precision_Admission = .Rejected;
				for near_constraint in near_handles[:near_count]
				{
					near, near_data_status := entasis.solver_contact_data(&region.world, near_constraint);
					if near_data_status != .Ok
					{
						return count, {status=.Validation_Failed, physics_status=near_data_status};
					}
					if precision_contact_pair_matches(&far, &near, i32(base), far_floor, near_floor) != .Accepted
					{
						continue;
					}
					matched = precision_contact_geometry_matches(&far, &near, errors);
					break;
				}
				if matched != .Accepted
				{
					fmt.eprintfln("precision_replay_geometry_failed group=%d body=%d pair_a=%v pair_b=%v", group, local_index, far.pair_a, far.pair_b);
					return count, {status=.Validation_Failed};
				}
				count += 1;
			}
		}
		stats, status := entasis.world_stats(&region.world);
		if status != .Ok || count != stats.active_constraints
		{
			return count, {status=.Validation_Failed, physics_status=status};
		}
		return count, {status=.Ok};
	}

	// replay identical absolute inputs in an independent one-group world
	// recentring changes outward-rounded predicted bounds at binary64 boundary
	// cases, so exact pair-set equality is not a translation-invariance oracle
	// controlled near/far trajectories above independently enforce the original
	// coordinate-accuracy limits. this replay instead checks complete pair sets,
	// identities and geometry independently of solver batching and worker order
	// it never changes the measured fixture, solver settings or tolerances
	precision_distributed_replay :: proc(
		pool: ^entasis.Buffer_Pool, workers: int,
	) -> (result: support.Benchmark_Result)
	{
		owner: Benchmark_Owner;
		result = benchmark_owner_create(&owner, pool, precision_options(workers, "", .Distributed), .Distributed);
		defer
		{
			status: entasis.Status = benchmark_owner_destroy(&owner);
			if status != .Ok
			{
				result.release_status = status;
				result.status = .Release_Failed;
			}
		}
		if result.status != .Ok
		{
			return;
		}
		regions: [1]Replay_Group;
		oracle: Replay_Owner;
		result = replay_owner_initialize(&oracle, regions[:], distributed_options(workers, "", .Distributed), .Distributed);
		defer
		{
			status: entasis.Status = replay_owner_destroy(&oracle);
			if status != .Ok
			{
				result.release_status = status;
				result.status = .Release_Failed;
			}
		}
		if result.status != .Ok
		{
			return;
		}
		errors: Precision_Contact_Errors;
		total_pairs: int;
		for frame in 0 ..< 8
		{
			status: entasis.Status = entasis.world_stage_predict_bounds(&owner.world, 1 / f32(60));
			if status == .Ok
			{
				status = entasis.world_stage_collision_detection(&owner.world, 1 / f32(60));
			}
			if status != .Ok
			{
				return {status=.Step_Failed, physics_status=status, failed_step=frame + 1};
			}
			far_stats, far_stats_status := entasis.world_stats(&owner.world);
			if far_stats_status != .Ok || (frame == 0 && (far_stats.active_pairs != 10000 || far_stats.active_constraints != 10000))
			{
				return {status=.Validation_Failed, physics_status=far_stats_status, failed_step=frame + 1};
			}
			frame_pairs: int;
			for group in 0 ..< DISTRIBUTED_WORLD_COUNT
			{
				floor, floor_status := entasis.static_get(&regions[0].world, {0});
				floor_pose, floor_pose_status := entasis.static_get_global_pose(&owner.world, {i32(group)});
				if floor_status != .Ok || floor_pose_status != .Ok
				{
					return {status=.Validation_Failed, failed_step=frame + 1};
				}
				floor.pose.orientation = floor_pose.orientation;
				status = entasis.static_apply_global(&regions[0].world, {0}, floor, floor_pose.position, .None);
				if status != .Ok
				{
					return {status=.Validation_Failed, physics_status=status, failed_step=frame + 1};
				}
				for near_handle, local_index in regions[0].bodies
				{
					far_handle: entasis.Body_Handle = owner.bodies[group * DISTRIBUTED_BODY_COUNT + local_index];
					description, read_status := entasis.body_get(&owner.world, far_handle);
					pose, pose_status := entasis.body_get_global_pose(&owner.world, far_handle);
					if read_status != .Ok || pose_status != .Ok
					{
						return {status=.Validation_Failed, failed_step=frame + 1};
					}
					status = entasis.body_apply_global(&regions[0].world, near_handle, description, pose.position);
					if status != .Ok
					{
						return {status=.Validation_Failed, physics_status=status, failed_step=frame + 1};
					}
				}
				status = entasis.world_stage_predict_bounds(&regions[0].world, 1 / f32(60));
				if status == .Ok
				{
					status = entasis.world_stage_collision_detection(&regions[0].world, 1 / f32(60));
				}
				if status != .Ok
				{
					return {status=.Step_Failed, physics_status=status, failed_step=frame + 1};
				}
				count: int;
				count, result = precision_replay_compare_group(&owner, &regions[0], group, &errors);
				if result.status != .Ok
				{
					result.failed_step = frame + 1;
					return;
				}
				frame_pairs += count;
			}
			if frame_pairs != far_stats.active_constraints
			{
				return {status=.Validation_Failed, failed_step=frame + 1};
			}
			total_pairs += frame_pairs;
			result = step_world(&owner, 1, .Warmup);
			if result.status != .Ok
			{
				return;
			}
			result = export_global_positions(&owner);
			if result.status != .Ok
			{
				return;
			}
			stats, stats_status := entasis.world_stats(&owner.world);
			if stats_status != .Ok
			{
				return {status=.Validation_Failed, physics_status=stats_status};
			}
			sample: support.Benchmark_Sample = {stats=stats};
			result = distributed_global_validate(&owner, &sample);
			if result.status != .Ok
			{
				return;
			}
		}
		fmt.printfln("precision_same_input_replay_passed workers=%d frames=8 groups=16 pairs=%d contacts=%d depth_error=%.17g offset_error=%.17g normal_error=%.17g",
			workers, total_pairs, errors.contacts, errors.depth, errors.offset, errors.normal);
		return {status=.Ok};
	}

	// admission is untimed and precedes warmup, measurement and CSV mutation
	// controlled trajectories and matched geometry use the fixed tolerances
	// independently evolved worlds are checked against stability invariants
	mixed64_candidate_admission :: proc(workers: int = 1) -> (result: support.Benchmark_Result)
	{
		pool: entasis.Buffer_Pool;
		status: entasis.Status = entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE);
		if status != .Ok
		{
			return {status=.Allocation_Failed, physics_status=status};
		}
		defer
		{
			release_status: entasis.Status = entasis.buffer_pool_destroy(&pool);
			if release_status != .Ok
			{
				result.release_status = release_status;
				result.status = .Release_Failed;
			}
		}
		origins: [4]entasis.Global_Position = {{10_000, 0, 0}, {1_000_000, 0, 0}, {1e9, 0, 0}, {-1e9, 1e9, -1e9}};
		for origin in origins
		{
			result = precision_control_run(&pool, origin, workers);
			if result.status != .Ok
			{
				return;
			}
		}
		return precision_distributed_replay(&pool, workers);
	}
}
