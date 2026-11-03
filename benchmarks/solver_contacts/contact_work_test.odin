package solver_contacts

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

contact_test_input :: proc() -> Input
{
	input: Input = {
		origin=.Accepted, step=150, dt=1.0/60,
		solve={velocity_iteration_count=1, substep_count=4, fallback_batch_threshold=2},
		fallback_batch_threshold=2, fallback_batch_index=2,
		pose_context={gravity={0, -10, 0}},
		dynamics=make([]physics.Body_Dynamics, 20), body_handles=make([]i32, 20),
		batches=make([]Batch_Input, 3),
	};
	for index in 0 ..< len(input.dynamics)
	{
		input.body_handles[index] = i32((index+7)%20);
		input.dynamics[index] = {
			motion={pose={orientation={0, 0, 0, 1}, position={f32(index), 2, 0}}, velocity={linear={0, -0.3, 0}}},
			inertia={local={inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}, world={inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}}},
		};
	}
	input.batches[0].types = make([]Type_Input, 2);
	input.batches[1].types = make([]Type_Input, 2);
	input.batches[2].types = make([]Type_Input, 1);
	input.batches[0].types[0] = {type_id=0, contacts=make([]Contact_Input, 1)};
	input.batches[0].types[1] = {type_id=5, contacts=make([]Contact_Input, 9)};
	input.batches[1].types[0] = {type_id=6, contacts=make([]Contact_Input, 1)};
	input.batches[1].types[1] = {type_id=7, contacts=make([]Contact_Input, 1)};
	input.batches[2].types[0] = {type_id=4, contacts=make([]Contact_Input, 1)};
	handle: i32 = 40;
	for batch, batch_index in input.batches
	{
		for kind, type_index in batch.types
		{
			for _, index in kind.contacts
			{
				contact: ^Contact_Input = &input.batches[batch_index].types[type_index].contacts[index];
				contact^ = {
					handle=handle, bodies={-1, -1}, normal={0, 1, 0}, offset_b={0, 1, 0},
					material={friction_coefficient=0.5, maximum_recovery_velocity=2, spring_settings={angular_frequency=188.49556, twice_damping_ratio=2}},
				};
				handle += 1;
				if kind.type_id == 0
				{
					contact.bodies[0] = 18;
				}
				else if kind.type_id == 5
				{
					contact.bodies = {i32(2*index), i32(2*index+1)};
				}
				else if kind.type_id == 6
				{
					contact.bodies = {0, 2};
				}
				else if kind.type_id == 4
				{
					contact.bodies = {19, 6};
				}
				else
				{
					contact.bodies = {4, 6};
				}
				for point in 0 ..< int(kind.type_id%4+1)
				{
					contact.offsets[point] = {f32(point%2)*0.5-0.25, -0.5, f32(point/2)*0.5-0.25};
					contact.depths[point] = 0.01;
					if index == 0
					{
						contact.normal_impulses[point] = 0.05;
					}
				}
			}
		}
	}
	return input;
}

@(test)
contact_input_preserves_graph_and_rejects_invalid_data :: proc(t: ^testing.T)
{
	input: Input = contact_test_input();
	defer input_destroy(&input);
	data: []u8 = input_encode(&input);
	defer delete(data);
	handles: []i32 = input.body_handles;
	input.body_handles = handles[:len(handles)-1];
	testing.expect_value(t, input_validate(&input), Status.Unsupported);
	input.body_handles = handles;
	input.batches[0].types[0].type_id = 18;
	testing.expect_value(t, input_validate(&input), Status.Unsupported);
	input.batches[0].types[0].type_id = 0;
	decoded: Input;
	testing.expect_value(t, input_decode(data, &decoded), Status.Ok);
	defer input_destroy(&decoded);
	testing.expect_value(t, input_counts(&decoded), input_counts(&input));
	encoded: []u8 = input_encode(&decoded);
	defer delete(encoded);
	testing.expect_value(t, len(encoded), len(data));
	for value, index in encoded
	{
		testing.expect_value(t, value, data[index]);
	}
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 65536), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	replay: Replay;
	testing.expect_value(t, replay_create(&replay, &decoded, &pool), physics.Physics_Status.Ok);
	defer replay_destroy(&replay);
	testing.expect_value(t, replay.solver.active_set.batch_count, i32(3));
	testing.expect_value(t, replay.solver.active_set.batches.memory[0].type_batches.memory[1].count, i32(9));
	testing.expect_value(t, replay.solver.handle_to_constraint.memory[40].type_id, i32(0));
	testing.expect_value(t, replay.solver.handle_to_constraint.memory[51].batch_index, i32(1));
	testing.expect_value(t, replay.bodies.sets.memory[0].constraints.memory[4].count, 2);
	testing.expect_value(t, replay.bodies.sets.memory[0].constraints.memory[19].count, 1);
	testing.expect_value(t, replay.solver.active_set.batches.memory[2].body_reference_counts.memory[19], i32(1));
	testing.expect_value(t, replay_solve(&replay), physics.Physics_Status.Ok);
	bad: Input;
	testing.expect(t, input_decode(data[:len(data)-1], &bad) != .Ok);
	testing.expect_value(t, len(bad.dynamics), 0);
	original: u8 = data[79];
	data[79] = 255;
	testing.expect(t, input_decode(data, &bad) != .Ok);
	data[79] = original;
	original = data[84+20*112+4];
	data[84+20*112+4] = 17;
	testing.expect_value(t, input_decode(data, &bad), Status.Unsupported);
	data[84+20*112+4] = original;
	quaternion_scales: [6]f32 = {0, 2, f32(math.sqrt(f64(1.0009))), f32(math.sqrt(f64(1.0011))),
		f32(math.sqrt(f64(0.9991))), f32(math.sqrt(f64(0.9989)))};
	for value, index in quaternion_scales
	{
		input.dynamics[0].motion.pose.orientation.w = value;
		geometry_data: []u8 = input_encode(&input);
		expected: Status = .Invalid_Input;
		if index == 2 || index == 4
		{
			expected = .Ok;
		}
		testing.expect_value(t, input_validate(&input), expected);
		testing.expect_value(t, input_decode(geometry_data, &bad), expected);
		if expected == .Ok
		{
			testing.expect_value(t, bad.dynamics[0].motion.pose.orientation.w, value);
		}
		else
		{
			testing.expect_value(t, len(bad.dynamics), 0);
		}
		input_destroy(&bad);
		delete(geometry_data);
	}
	input.dynamics[0].motion.pose.orientation.w = 1;
	normal_scales: [5]f32 = {1.001, f32(math.sqrt(f64(1.000009))), f32(math.sqrt(f64(0.999991))),
		f32(math.sqrt(f64(1.000011))), f32(math.sqrt(f64(0.999989)))};
	for value, index in normal_scales
	{
		input.batches[0].types[0].contacts[0].normal.y = value;
		geometry_data: []u8 = input_encode(&input);
		expected: Status = .Invalid_Input;
		if index == 1 || index == 2
		{
			expected = .Ok;
		}
		testing.expect_value(t, input_validate(&input), expected);
		testing.expect_value(t, input_decode(geometry_data, &bad), expected);
		if expected == .Ok
		{
			testing.expect_value(t, bad.batches[0].types[0].contacts[0].normal.y, value);
		}
		else
		{
			testing.expect_value(t, len(bad.dynamics), 0);
		}
		input_destroy(&bad);
		delete(geometry_data);
	}
	input.batches[0].types[0].contacts[0].normal.y = 1;
	writer: Input_Writer = {data=data, offset=88};
	write_scalar(&writer, transmute(f32)u32(0x7fc00000));
	testing.expect(t, input_decode(data, &bad) != .Ok);
	testing.expect_value(t, len(bad.dynamics), 0);
}

contact_test_parallel_owners :: proc(t: ^testing.T, replay: ^Replay)
{
	status: physics.Physics_Status;
	status, _ = physics.solver_prepare_integration_parallel_layout_and_owners(&replay.solver);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_prepare_exact_integration_responsibilities(&replay.solver), physics.Physics_Status.Ok);
	work_count: int;
	status, work_count = physics.solver_prepare_integration_region_blocks(&replay.solver);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	for index in 0 ..< work_count
	{
		testing.expect_value(t, physics.solver_prepare_integration_region_block(&replay.solver, &replay.solver.work_blocks.memory[index]), physics.Physics_Status.Ok);
	}
	physics.solver_finalize_integration_responsibilities(&replay.solver, work_count);
	owners: [20]int;
	constrained: [20]physics.Reference_State;
	for batch_index in 0 ..< replay.solver.active_set.batch_count
	{
		batch := &replay.solver.active_set.batches.memory[batch_index];
		for type_index in 0 ..< batch.type_batch_count
		{
			kind := &batch.type_batches.memory[type_index];
			for first := 0; first < int(kind.count); first += util.PRODUCTION_LANE_COUNT
			{
				execution := physics.solver_active_mask(first, int(kind.count));
				references := physics.type_batch_body_bundle(kind, first);
				for role in 0 ..< int(kind.body_count)
				{
					// batch zero integrates every live dynamic lane without consulting per-type flags
					integration := execution;
					if batch_index > 0
					{
						_, integration = physics.solver_integration_mask(&replay.solver, kind, role, first/util.PRODUCTION_LANE_COUNT, execution);
					}
					for lane in 0 ..< util.PRODUCTION_LANE_COUNT
					{
						encoded := simd.extract(references[role], lane);
						if simd.extract(execution, lane) == 0 || encoded < 0 || u32(encoded) >= physics.BODY_REFERENCE_KINEMATIC_MASK
						{
							continue;
						}
						body := int(u32(encoded) & physics.BODY_REFERENCE_INDEX_MASK);
						constrained[body] = .Present;
						if simd.extract(integration, lane) != 0
						{
							owners[body] += 1;
						}
					}
				}
			}
		}
	}
	for state, index in constrained
	{
		expected := 0;
		if state == .Present
		{
			expected = 1;
		}
		testing.expect_value(t, owners[index], expected);
	}
	testing.expect_value(t, owners[18], 1);
	testing.expect_value(t, owners[19], 1);
}

@(test)
contact_replay_reset_is_independent :: proc(t: ^testing.T)
{
	input: Input = contact_test_input();
	defer input_destroy(&input);
	// body eighteen first appears in a synchronized region and nineteen only in fallback
	delete(input.batches[0].types[0].contacts);
	input.batches[0].types[0].contacts = nil;
	input.batches[1].types[0].contacts[0].bodies[0] = 18;
	testing.expect_value(t, input_validate(&input), Status.Ok);
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 65536), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	left, right: Replay;
	testing.expect_value(t, replay_create(&left, &input, &pool), physics.Physics_Status.Ok);
	defer replay_destroy(&left);
	testing.expect_value(t, replay_create(&right, &input, &pool), physics.Physics_Status.Ok);
	defer replay_destroy(&right);
	contact_test_parallel_owners(t, &left);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 2), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	scratch: []f64 = make([]f64, int(input_counts(&input).points));
	defer delete(scratch);
	for workers in 1 ..= 2
	{
		boundary: ^util.Thread_Dispatcher_Boundary;
		if workers == 2
		{
			boundary = util.thread_dispatcher_boundary(&dispatcher);
		}
		replay_reset(&left);
		testing.expect_value(t, replay_solve(&left, boundary), physics.Physics_Status.Ok);
		warmup_quality: Quality_Result = quality_observe(&left, scratch);
		testing.expect_value(t, warmup_quality.observed_rows, int(input_counts(&input).points));
		testing.expect_value(t, warmup_quality.residual_rows, int(input_counts(&input).points));
		left.bodies.sets.memory[0].dynamics_state.memory[0].motion.velocity.linear.x = 123;
		kind: ^physics.Type_Batch = &left.solver.active_set.batches.memory[0].type_batches.memory[1];
		prestep: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_prestep_bundle(kind, 0));
		impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, 0));
		prestep[3] = util.F32x8(111);
		impulses[2] = util.F32x8(222);
		replay_reset(&left);
		replay_reset(&right);
		testing.expect_value(t, replay_solve(&left, boundary), physics.Physics_Status.Ok);
		testing.expect_value(t, replay_solve(&right, boundary), physics.Physics_Status.Ok);
		for index in 0 ..< len(input.dynamics)
		{
			testing.expect_value(t, left.bodies.sets.memory[0].dynamics_state.memory[index], right.bodies.sets.memory[0].dynamics_state.memory[index]);
		}
		for canonical, index in left.canonical
		{
			for byte in 0 ..< len(canonical.prestep)
			{
				testing.expect_value(t, canonical.native.prestep_data.memory[byte], right.canonical[index].native.prestep_data.memory[byte]);
			}
			for byte in 0 ..< len(canonical.impulses)
			{
				testing.expect_value(t, canonical.native.accumulated_impulses.memory[byte], right.canonical[index].native.accumulated_impulses.memory[byte]);
			}
		}
		tail: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, 8));
		testing.expect_value(t, simd.extract(tail[2], 1), f32(0));
	}
}

@(test)
contact_work_counts_logical_point_updates :: proc(t: ^testing.T)
{
	input: Input = contact_test_input();
	defer input_destroy(&input);
	counts: Counts = input_counts(&input);
	testing.expect_value(t, input_validate(&input), Status.Ok);
	testing.expect_value(t, counts.manifolds, i64(13));
	testing.expect_value(t, counts.points, i64(27));
	testing.expect_value(t, counts.fallback_manifolds, i64(1));
	testing.expect_value(t, logical_point_updates(&input, 3), i64(324));
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 65536), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	replay: Replay;
	testing.expect_value(t, replay_create(&replay, &input, &pool), physics.Physics_Status.Ok);
	defer replay_destroy(&replay);
	scratch: []f64 = make([]f64, int(counts.points));
	defer delete(scratch);
	coverage: int;
	for call in 0 ..< 3
	{
		replay_reset(&replay);
		testing.expect_value(t, replay_solve(&replay), physics.Physics_Status.Ok);
		quality: Quality_Result = quality_observe(&replay, scratch);
		testing.expect_value(t, quality.observed_rows, int(counts.points));
		testing.expect_value(t, quality.residual_rows, int(counts.points));
		coverage += 1;
		_ = call;
	}
	testing.expect_value(t, coverage, 3);
}

contact_quality_test_manifold :: proc(a, b: Quality_Body) -> Quality_Manifold
{
	return {
		bodies={a, b}, body_count=2, contact_count=2,
		offsets={{-0.5, 0, 0}, {0.5, 0, 0}, {}, {}}, depths={8.5, 8.5, 0, 0}, normal_impulses={1, 1, 0, 0},
		normal={0, 1, 0}, friction=0.5, angular_frequency=1, twice_damping_ratio=2,
		maximum_recovery_velocity=10, dt=1,
	};
}

@(test)
contact_endpoint_residual_observes_shared_bodies :: proc(t: ^testing.T)
{
	// four unit normal impulses give these velocities and cancel the paired torques
	bodies: [3]Quality_Body;
	for &body in bodies
	{
		body.inverse_mass = 1;
		body.inverse_inertia = {1, 0, 1, 0, 0, 1};
	}
	bodies[0].linear[1] = 2;
	bodies[2].linear[1] = -2;
	first := contact_quality_test_manifold(bodies[0], bodies[1]);
	second := contact_quality_test_manifold(bodies[1], bodies[2]);
	scratch: [4]f64;
	result: Quality_Result = {expected_rows=4};
	quality_manifold_observe(&first, &result, scratch[:]);
	quality_manifold_observe(&second, &result, scratch[:]);
	quality_finish(&result, scratch[:]);
	testing.expect_value(t, result.status, Quality_Status.Pass);
	testing.expect_value(t, result.observed_rows, 4);
	testing.expect(t, result.residual_max_mps < 1e-12);

	// a later manifold changes the shared body, so the earlier rows must be observed again
	bodies[1].linear[1] = 0.25;
	first.bodies[1] = bodies[1];
	second.bodies[0] = bodies[1];
	result = {expected_rows=4};
	quality_manifold_observe(&first, &result, scratch[:]);
	testing.expect(t, math.abs(scratch[0] - 0.25) < 1e-12);
	quality_manifold_observe(&second, &result, scratch[:]);
	quality_finish(&result, scratch[:]);
	testing.expect_value(t, result.status, Quality_Status.Pass);
	testing.expect(t, math.abs(result.residual_max_mps - 0.25) < 1e-12);
	testing.expect(t, math.abs(result.residual_rms_mps - 0.25) < 1e-12);

	probe := contact_quality_test_manifold(bodies[0], bodies[1]);
	probe.body_count = 1;
	probe.contact_count = 1;
	probe.offsets[0] = {};
	probe.normal_impulses[0] = 0;
	probe.depths[0] = -1;
	probe.bodies[0].linear[1] = -0.5;
	result = {expected_rows=1};
	quality_manifold_observe(&probe, &result, scratch[:]);
	quality_finish(&result, scratch[:]);
	testing.expect_value(t, result.status, Quality_Status.Pass);
	testing.expect_value(t, result.residual_max_mps, f64(0));
	probe.depths[0] = 30;
	probe.maximum_recovery_velocity = 2;
	probe.bodies[0].linear[1] = 2;
	result = {expected_rows=1};
	quality_manifold_observe(&probe, &result, scratch[:]);
	quality_finish(&result, scratch[:]);
	testing.expect_value(t, result.residual_max_mps, f64(0));

	// retain the full J*M^-1*J^T term for a rounded, approximately unit normal
	probe.normal = {0, 1.0001, 0};
	probe.normal_impulses[0] = 1;
	probe.depths[0] = 1.0001*1.0001;
	probe.maximum_recovery_velocity = 10;
	probe.bodies[0].linear[1] = 0;
	result = {expected_rows=1};
	quality_manifold_observe(&probe, &result, scratch[:]);
	quality_finish(&result, scratch[:]);
	testing.expect(t, result.residual_max_mps < 1e-12);
}

@(test)
contact_quality_detects_invalid_impulses :: proc(t: ^testing.T)
{
	input := contact_test_input();
	defer input_destroy(&input);
	for &batch in input.batches
	{
		for &kind in batch.types
		{
			for &contact in kind.contacts
			{
				contact.normal_impulses = {1, 1, 1, 1};
				contact.tangent_impulse = {};
				contact.twist_impulse = 0;
			}
		}
	}
	testing.expect_value(t, input_validate(&input), Status.Ok);
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 256), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	replay: Replay;
	testing.expect_value(t, replay_create(&replay, &input, &pool), physics.Physics_Status.Ok);
	defer replay_destroy(&replay);
	scratch := make([]f64, int(input_counts(&input).points));
	defer delete(scratch);
	valid := quality_observe(&replay, scratch);
	testing.expect_value(t, valid.status, Quality_Status.Pass);
	testing.expect_value(t, valid.observed_rows, int(input_counts(&input).points));

	kind := &replay.solver.active_set.batches.memory[0].type_batches.memory[0];
	impulses := ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, 0));
	impulses[2] = simd.replace(impulses[2], 0, -1e-6);
	valid = quality_observe(&replay, scratch);
	testing.expect_value(t, valid.status, Quality_Status.Pass);
	impulses[2] = simd.replace(impulses[2], 0, -0.1);
	bad := quality_observe(&replay, scratch);
	testing.expect_value(t, bad.status, Quality_Status.Fail);
	testing.expect(t, bad.negative_impulse_max > 0.09);
	impulses[2] = simd.replace(impulses[2], 0, 1);
	impulses[0] = simd.replace(impulses[0], 0, 0.500001);
	valid = quality_observe(&replay, scratch);
	testing.expect_value(t, valid.status, Quality_Status.Pass);
	impulses[0] = simd.replace(impulses[0], 0, 2);
	bad = quality_observe(&replay, scratch);
	testing.expect_value(t, bad.status, Quality_Status.Fail);
	testing.expect(t, bad.friction_violation_max > 0);
	impulses[0] = simd.replace(impulses[0], 0, 0);
	impulses[3] = simd.replace(impulses[3], 0, 0.005001);
	valid = quality_observe(&replay, scratch);
	testing.expect_value(t, valid.status, Quality_Status.Pass);
	impulses[3] = simd.replace(impulses[3], 0, 1);
	bad = quality_observe(&replay, scratch);
	testing.expect_value(t, bad.status, Quality_Status.Fail);
	testing.expect(t, bad.twist_violation_max > 0);
	impulses[3] = simd.replace(impulses[3], 0, 0);

	poison: f32 = transmute(f32)u32(0x7fc00001);
	impulses[2] = simd.replace(impulses[2], 0, poison);
	bad = quality_observe(&replay, scratch);
	testing.expect_value(t, bad.status, Quality_Status.Fail);
	testing.expect(t, bad.finite_failures > 0);
	impulses[2] = simd.replace(impulses[2], 0, 1);
	last_index := int(kind.count) - 1;
	prestep := ([^]util.F32x8)(physics.type_batch_prestep_bundle(kind, last_index));
	tail_impulses := ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, last_index));
	for lane in last_index%8 + 1 ..< 8
	{
		for vector in 0 ..< int(kind.prestep_bundle_size)/size_of(util.F32x8)
		{
			prestep[vector] = simd.replace(prestep[vector], lane, poison);
		}
		for vector in 0 ..< int(kind.impulse_bundle_size)/size_of(util.F32x8)
		{
			tail_impulses[vector] = simd.replace(tail_impulses[vector], lane, poison);
		}
	}
	valid = quality_observe(&replay, scratch);
	testing.expect_value(t, valid.status, Quality_Status.Pass);
	testing.expect_value(t, valid.residual_rows, valid.expected_rows);
	body := &replay.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX].dynamics_state.memory[0];
	body.motion.velocity.linear.y = poison;
	bad = quality_observe(&replay, scratch);
	testing.expect_value(t, bad.status, Quality_Status.Fail);
	testing.expect(t, bad.finite_failures > 0);
}
