package simulation_stage_work

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import bounds "../shape_mixed_bounds"
import container "../container"
import support "../benchmark_support"
import contacts "../solver_contacts"

Test_Dispatch_Record :: struct
{
	inner: ^util.Thread_Dispatcher_Boundary,
	calls, maximum_workers: int,
}

test_record_dispatch :: proc "contextless" (
	boundary: ^util.Thread_Dispatcher_Boundary, worker: util.Dispatcher_Worker_Proc,
	maximum_workers: int, unmanaged_context: rawptr,
) -> util.Threading_Status
{
	record: ^Test_Dispatch_Record = (^Test_Dispatch_Record)(boundary.dispatcher);
	record.calls += 1;
	record.maximum_workers = max(record.maximum_workers, maximum_workers);
	return record.inner.dispatch(record.inner, worker, maximum_workers, unmanaged_context);
}

test_record_pool :: proc(boundary: ^util.Thread_Dispatcher_Boundary, worker: int) -> (^util.Buffer_Pool, util.Threading_Status)
{
	record: ^Test_Dispatch_Record = (^Test_Dispatch_Record)(boundary.dispatcher);
	return record.inner.worker_pool(record.inner, worker);
}

@(test)
stage_prescribed_input_is_independent :: proc(t: ^testing.T)
{
	run: Run;
	defer run_destroy(&run);
	o: Options = {stage=.Tree_Update, fixture=.Container, motion=.Churn, workers=1};
	testing.expect(t, run_create(&run, o, {4, 4, 4}) == .Ok);
	testing.expect(t, entasis.world_step(run_world(&run), TIMESTEP) == .Ok);
	active: ^physics.Body_Set = &run.simulation.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	active.dynamics_state.memory[3].motion.pose.position = {999, -999, 17};
	active.dynamics_state.memory[3].motion.velocity = {linear={99, 100, 101}, angular={3, 2, 1}};
	prescribe(&run, run.simulation, 1);
	testing.expect_value(t, active.dynamics_state.memory[3].motion.pose.position, prescribed_position(&run, 3, 1));
	testing.expect_value(t, active.dynamics_state.memory[3].motion.pose.orientation, util.Quaternion{w=1});
	testing.expect_value(t, active.dynamics_state.memory[3].motion.velocity, physics.Body_Velocity{});
	testing.expect_value(t, run.last_pairs, expected_pairs(&run, 0));
	// default construction still installs the ordinary timestepper
	plain: container.Benchmark_Owner;
	defer container.benchmark_owner_destroy(&plain);
	options: support.Options;
	admission: support.Admission;
	options, admission = run_scene_options(o, {2, 2, 2}, {1, 1});
	testing.expect(t, admission == .Ok);
	testing.expect(t, container.benchmark_owner_create(&plain, &run.pool, options).status == .Ok);
	testing.expect(t, entasis.world_step(&plain.world, TIMESTEP) == .Ok);
	testing.expect_value(t, run.frame_index, 1);
}

@(test)
stage_pair_output_matches_independent_aabb_oracle :: proc(t: ^testing.T)
{
	for fixture in Fixture
	{
		for motion in Motion
		{
			run: Run;
			o: Options = {stage=.Pair_Traversal, fixture=fixture, motion=motion, workers=2};
			testing.expect(t, run_create(&run, o, {8, 4, 8}) == .Ok);
			for frame in 0 ..< 2
			{
				testing.expect(t, entasis.world_step(run_world(&run), TIMESTEP) == .Ok);
				exhaustive: int;
				for i in 0 ..< len(run.initial)
				{
					a: util.Vector3 = prescribed_position(&run, i, frame);
					for j in i+1 ..< len(run.initial)
					{
						exhaustive += positions_overlap(a, prescribed_position(&run, j, frame));
					}
					for box in run.statics
					{
						exhaustive += static_overlap(a, box);
					}
				}
				testing.expect_value(t, run.last_pairs, exhaustive);
			}
			dispatcher: util.Thread_Dispatcher;
			testing.expect(t, util.thread_dispatcher_initialize(&dispatcher, 2, 65536) == .Ok);
			record: Test_Dispatch_Record = {inner=util.thread_dispatcher_boundary(&dispatcher)};
			boundary: util.Thread_Dispatcher_Boundary = {
				dispatcher=&record, dispatch=test_record_dispatch, worker_pool=test_record_pool, worker_count=2,
			};
			testing.expect(t, physics.simulation_predict_bounding_boxes(run.simulation, TIMESTEP, &boundary) == .Ok);
			record.calls = 0;
			record.maximum_workers = 0;
			testing.expect(t, physics.broad_phase_update(&run.simulation.broad_phase, &boundary) == .Ok);
			testing.expect(t, record.calls > 0 && record.maximum_workers == 2);
			testing.expect(t, util.thread_dispatcher_shutdown(&dispatcher) == .Ok);
			run_destroy(&run);
		}
	}
	for layout in bounds.Bounds_Layout
	{
		for count in ([3]int{8, 19, 10000})
		{
			seen: [10000]u8;
			for i in 0 ..< count
			{
				logical: int = bounds.benchmark_body_index(layout, count, i);
				testing.expect(t, logical >= 0 && logical < count);
				testing.expect_value(t, seen[logical], u8(0));
				seen[logical] = 1;
			}
		}
		for count in ([2]int{8, 19})
		{
			pool: entasis.Buffer_Pool;
			testing.expect(t, entasis.buffer_pool_init(&pool, 65536) == .Ok);
			owner: bounds.Benchmark_Simulation_Owner;
			testing.expect(t, bounds.benchmark_owner_create(&owner, &pool, 1, layout, count) == .Ok);
			view: entasis.Active_Body_View;
			view_status: entasis.Status;
			view, view_status = entasis.active_body_view(&owner.world);
			testing.expect_value(t, view_status, entasis.Status.Ok);
			first_motion: physics.Motion_State = view.dynamics[0].motion;
			last_motion: physics.Motion_State = view.dynamics[count - 1].motion;
			testing.expect(t, bounds.benchmark_owner_predict(&owner, 2, .Warmup).status == .Ok);
			testing.expect(t, bounds.benchmark_owner_predict(&owner, 1, .Measured).status == .Ok);
			testing.expect_value(t, view.dynamics[0].motion, first_motion);
			testing.expect_value(t, view.dynamics[count - 1].motion, last_motion);
			counters: bounds.Bounds_Counters;
			testing.expect(t, bounds.benchmark_validate_bounds(&owner, &counters) == .Ok);
			testing.expect_value(t, counters.body_count, count);
			testing.expect(t, bounds.benchmark_owner_destroy(&owner) == .Ok);
			testing.expect(t, entasis.buffer_pool_destroy(&pool) == .Ok);
		}
	}
}

@(test)
stage_collision_history_preserves_update_remove_reentry :: proc(t: ^testing.T)
{
	for stage in ([2]Stage{.Narrow_Phase, .Collision})
	{
		run: Run;
		o: Options = {stage=stage, fixture=.Islands, motion=.Churn, workers=2};
		testing.expect(t, run_create(&run, o, {4, 4, 4}) == .Ok);
		counts: [3]int;
		measured_pairs: i64;
		for frame in 0 ..< WARMUP_FRAMES + 3
		{
			testing.expect(t, entasis.world_step(run_world(&run), TIMESTEP) == .Ok);
			testing.expect_value(t, run.last_manifolds, run.last_pairs);
			if frame < 3
			{
				counts[frame] = run.last_manifolds;
			}
			if frame < WARMUP_FRAMES
			{
				testing.expect_value(t, run.completed, 0);
				testing.expect_value(t, run.pair_count, i64(0));
				testing.expect_value(t, run.manifold_count, i64(0));
				testing.expect_value(t, run.contact_points, i64(0));
			}
			else
			{
				measured_pairs += i64(run.last_pairs);
			}
		}
		testing.expect_value(t, run.frame_index, WARMUP_FRAMES + 3);
		testing.expect_value(t, run.completed, 3);
		testing.expect_value(t, run.failures, 0);
		testing.expect_value(t, run.pair_count, measured_pairs);
		testing.expect_value(t, run.manifold_count, measured_pairs);
		testing.expect(t, run.contact_points >= measured_pairs && run.contact_points <= 4 * measured_pairs);
		testing.expect(t, run.pool_before > 0 && run.pool_after >= run.pool_before);
		testing.expect(t, counts[0] > counts[1]);
		testing.expect_value(t, counts[2], counts[0]);
		checked: [2]int;
		cache: ^physics.Pair_Cache = &run.simulation.narrow_phase.pair_cache;
		for entry in 0 ..< cache.mapping.count
		{
			pair: physics.Collidable_Pair = cache.mapping.keys.memory[entry];
			location: physics.Constraint_Location;
			status: physics.Physics_Status;
			location, status = physics.solver_resolve(&run.simulation.solver, cache.mapping.values.memory[entry].constraint_handle);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			batch: ^physics.Constraint_Batch = &run.simulation.solver.active_set.batches.memory[location.batch_index];
			kind: ^physics.Type_Batch = &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]];
			if checked[kind.body_count-1] != 0
			{
				continue;
			}
			checked[kind.body_count-1] += 1;
			index: int = int(location.index_in_type_batch);
			original: contacts.Contact_Input = contacts.contact_read(kind, index);
			testing.expect_value(t, contact_geometry_status(&run, run.simulation, pair, kind, index, 2), physics.Physics_Status.Ok);
			mutated: contacts.Contact_Input = original;
			mutated.normal = util.vector3_scale(original.normal, -1);
			contacts.contact_write(kind, index, mutated);
			testing.expect_value(t, contact_geometry_status(&run, run.simulation, pair, kind, index, 2), physics.Physics_Status.Invalid_Argument);
			for point in 0 ..< int(kind.type_id%4+1)
			{
				for axis in 0 ..< 3
				{
					for invalid in ([2]f32{transmute(f32)u32(0x7fc00000), transmute(f32)u32(0x7f800000)})
					{
						mutated = original;
						components: ^[3]f32 = cast(^[3]f32)&mutated.offsets[point];
						components[axis] = invalid;
						contacts.contact_write(kind, index, mutated);
						testing.expect_value(t, contact_geometry_status(&run, run.simulation, pair, kind, index, 2), physics.Physics_Status.Invalid_Argument);
					}
				}
			}
			if kind.body_count == 2
			{
				for axis in 0 ..< 3
				{
					mutated = original;
					components: ^[3]f32 = cast(^[3]f32)&mutated.offset_b;
					components[axis] = transmute(f32)u32(0x7fc00000);
					contacts.contact_write(kind, index, mutated);
					testing.expect_value(t, contact_geometry_status(&run, run.simulation, pair, kind, index, 2), physics.Physics_Status.Invalid_Argument);
				}
			}
			contacts.contact_write(kind, index, original);
		}
		testing.expect_value(t, checked, [2]int{1, 1});
		run_destroy(&run);
	}
	for fixture in Fixture
	{
		run: Run;
		o: Options = {stage=.Collision, fixture=fixture, motion=.Steady, workers=1};
		testing.expect_value(t, run_create(&run, o, {4, 4, 4}), physics.Physics_Status.Ok);
		testing.expect_value(t, entasis.world_step(run_world(&run), TIMESTEP), entasis.Status.Ok);
		run_destroy(&run);
	}
}

@(test)
stage_arguments_reject_inapplicable_modes :: proc(t: ^testing.T)
{
	valid: [5]string = {"--stage=collision", "--fixture=container", "--motion=steady", "--worker-count=1", "--output=result.csv"};
	o: Options;
	status: support.Admission;
	o, status = parse_options(valid[:]);
	testing.expect(t, status == .Ok);
	testing.expect_value(t, o.stage, Stage.Collision);
	_, status = parse_options(valid[:4]);
	testing.expect_value(t, status, support.Admission.Invalid);
	for bad in ([6]string{"--stage=collision", "--stage=solve", "--body-count=1", "--substeps=4", "--worker-count=0", "--unknown=value"})
	{
		arguments: [6]string;
		copy(arguments[:5], valid[:]);
		arguments[5] = bad;
		_, status = parse_options(arguments[:]);
		testing.expect_value(t, status, support.Admission.Invalid);
	}
}
