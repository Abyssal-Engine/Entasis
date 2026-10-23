package precision_storage_tests

import "base:runtime"
import "core:math"
import "core:mem"
import "core:os"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Scene :: struct
{
	pool: util.Buffer_Pool,
	tracker: mem.Tracking_Allocator,
	simulation: ^physics.Simulation,
	simulation_storage: util.Buffer(physics.Simulation),
}

scene_initialize :: proc(t: ^testing.T, scene: ^Scene, capacity: i32 = 64, workers: i32 = 1)
{
	mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
	testing.expect_value(t, util.buffer_pool_initialize_with_allocator(
		&scene.pool, mem.tracking_allocator(&scene.tracker), 4096,
	), util.Memory_Status.Ok);
	status: util.Memory_Status;
	scene.simulation_storage, status = util.buffer_pool_take_at_least(&scene.pool, physics.Simulation, 1);
	testing.expect_value(t, status, util.Memory_Status.Ok);
	scene.simulation = &scene.simulation_storage.memory[0];
	scene.simulation^ = {};
	description: physics.Simulation_Create_Description = physics.simulation_create_description_default(&scene.pool);
	description.allocation_sizes = {
		bodies=capacity, statics=16, inactive_body_sets=capacity, shapes_per_type=8,
		constraints=capacity * 4, constraint_batches=8, initial_constraints_per_type_batch=16,
		minimum_constraints_per_body=4, broad_phase_candidates=capacity * 8, pairs=capacity * 8,
		inactive_pairs=capacity * 8, pending_pairs_per_worker=capacity * 4, workers=workers,
	};
	description.default_pose_context = {};
	description.solve_description = {velocity_iteration_count=2, substep_count=4, fallback_batch_threshold=4};
	testing.expect_value(t, physics.simulation_create(scene.simulation, &description).status, physics.Physics_Status.Ok);
}

scene_dispose :: proc(t: ^testing.T, scene: ^Scene)
{
	testing.expect_value(t, physics.simulation_destroy(scene.simulation), physics.Physics_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&scene.pool, &scene.simulation_storage), util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_dispose(&scene.pool), util.Memory_Status.Ok);
	testing.expect_value(t, scene.tracker.current_memory_allocated, i64(0));
	mem.tracking_allocator_destroy(&scene.tracker);
}

body_description :: proc "contextless" () -> physics.Body_Description
{
	return {
		pose={orientation=util.quaternion_identity()},
		local_inertia={inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
		collidable={maximum_speculative_margin=0.1},
		activity={sleep_threshold=0, minimum_timestep_count_under_threshold=255},
	};
}

far_position :: proc "contextless" (index: int) -> physics.Global_Position
{
	return {1e9 + f64(index) / 512, -1e9 - f64(index) / 256, 1e9 + f64(index) / 128};
}

expect_position :: proc(t: ^testing.T, bodies: ^physics.Bodies, handle: physics.Body_Handle, expected: physics.Global_Position)
{
	actual: physics.Global_Position;
	status: physics.Physics_Status;
	actual, status = physics.bodies_get_global_position(bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, expected);
	location: physics.Body_Memory_Location;
	location, status = physics.bodies_resolve(bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, bodies.sets.memory[location.set_index].dynamics_state.memory[location.index].motion.pose.position,
		physics.world_precision_position_cache(expected));
}

@(test)
precision_dense_positions_follow_growth_compaction_sleep_sets_and_handle_reuse :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	bodies: ^physics.Bodies = &scene.simulation.bodies;
	handles: [40]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description: physics.Body_Description = body_description();
		description.pose.position = {f32(index), f32(-index), f32(index * 2)};
		status: physics.Physics_Status;
		handles[index], status = physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		expect_position(t, bodies, handles[index], {f64(index), f64(-index), f64(index * 2)});
		testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[index], far_position(index)), physics.Physics_Status.Ok);
	}
	for index in 1 ..= 3
	{
		testing.expect_value(t, physics.bodies_move(bodies, handles[index], index), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.bodies_ensure_capacity(bodies, 512, 32, 4), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_resize(bodies, 40, 8, 4), physics.Physics_Status.Ok);
	// moving set 1 back creates a hole. set 4 then compacts the remaining arena
	testing.expect_value(t, physics.bodies_move(bodies, handles[1], 0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_move(bodies, handles[4], 4), physics.Physics_Status.Ok);
	for index in 0 ..< len(handles)
	{
		expect_position(t, bodies, handles[index], far_position(index));
	}
	testing.expect_value(t, physics.bodies_remove(bodies, handles[9]), physics.Physics_Status.Ok);
	for index in 0 ..< len(handles)
	{
		if index != 9
		{
			expect_position(t, bodies, handles[index], far_position(index));
		}
	}
	description: physics.Body_Description = body_description();
	description.pose.position = {-2, 3, -4};
	reused: physics.Body_Handle;
	status: physics.Physics_Status;
	reused, status = physics.bodies_add(bodies, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, reused, handles[9]);
	expect_position(t, bodies, reused, {-2, 3, -4});
	testing.expect_value(t, physics.bodies_clear(bodies), physics.Physics_Status.Ok);
	reused, status = physics.bodies_add(bodies, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	expect_position(t, bodies, reused, {-2, 3, -4});
}

@(test)
precision_static_positions_follow_dense_moves_resize_and_explicit_replacement :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	statics: ^physics.Statics = &scene.simulation.statics;
	box: physics.Box = {0.5, 0.5, 0.5};
	shape: physics.Typed_Index;
	status: physics.Physics_Status;
	shape, status = physics.shape_registry_add(&scene.simulation.shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	description: physics.Static_Description = {pose={orientation=util.quaternion_identity()}, shape=shape};
	handles: [20]physics.Static_Handle;
	for index in 0 ..< len(handles)
	{
		handles[index], status = physics.statics_add(statics, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.statics_set_global_position(statics, handles[index], far_position(index)), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.statics_ensure_capacity(statics, 256), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_resize(statics, 24), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_remove(statics, handles[3]), physics.Physics_Status.Ok);
	for index in 0 ..< len(handles)
	{
		if index == 3
		{
			continue;
		}
		actual: physics.Global_Position;
		actual, status = physics.statics_get_global_position(statics, handles[index]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, actual, far_position(index));
	}
	description.pose.position = {1, 2, 3};
	testing.expect_value(t, physics.statics_apply_description(statics, handles[0], &description), physics.Physics_Status.Ok);
	actual: physics.Global_Position;
	actual, status = physics.statics_get_global_position(statics, handles[0]);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, physics.Global_Position{1, 2, 3});
}

@(test)
precision_position_admission_is_atomic :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	description: physics.Body_Description = body_description();
	handle: physics.Body_Handle;
	status: physics.Physics_Status;
	handle, status = physics.bodies_add(&scene.simulation.bodies, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	position: physics.Global_Position = far_position(7);
	testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, position), physics.Physics_Status.Ok);
	invalid: [5]physics.Global_Position = {
		{math.nan_f64(), 0, 0}, {0, math.inf_f64(1), 0}, {0, 0, math.inf_f64(-1)},
		{f64(math.F32_MAX) * 2, 0, 0}, {-f64(math.F32_MAX) * 2, 0, 0},
	};
	for value in invalid
	{
		testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, value), physics.Physics_Status.Invalid_Argument);
		expect_position(t, &scene.simulation.bodies, handle, position);
	}
	testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, {-1}, position), physics.Physics_Status.Not_Found);
	description.pose.position.x = math.nan_f32();
	testing.expect_value(t, physics.bodies_apply_description(&scene.simulation.bodies, handle, &description), physics.Physics_Status.Invalid_Description);
	expect_position(t, &scene.simulation.bodies, handle, position);
}

@(test)
precision_relative_gathers_preserve_two_millimetres_with_masks_and_kinematic_bits :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bodies: ^physics.Bodies = &scene.simulation.bodies;
	handles: [16]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description: physics.Body_Description = body_description();
		status: physics.Physics_Status;
		handles[index], status = physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	offsets: [5]f64 = {0, 1e4, -1e6, 1e9, -1e9};
	for offset in offsets
	{
		for index in 0 ..< len(handles)
		{
			position: physics.Global_Position = {offset + f64(index) * 0.002, -offset - f64(index) * 0.002, offset};
			testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[index], position), physics.Physics_Status.Ok);
		}
		for bits in 0 ..< 256
		{
			a, b, mask: [8]i32;
			for lane in 0 ..< 8
			{
				a[lane] = i32(lane * 2);
				b[lane] = i32(lane * 2 + 1) | i32(physics.BODY_REFERENCE_KINEMATIC_MASK);
				if bits & (1 << uint(lane)) != 0
				{
					mask[lane] = -1;
				}
			}
			delta: util.Vector3_Wide;
			status: physics.Physics_Status;
			delta, status = physics.bodies_gather_relative_positions(bodies,
				transmute(util.I32x8)a, transmute(util.I32x8)b, transmute(util.I32x8)mask);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			for lane in 0 ..< 8
			{
				actual: util.Vector3 = util.vector3_wide_read_slot(delta, lane);
				if mask[lane] == 0
				{
					testing.expect_value(t, actual, util.Vector3{});
				}
				else
				{
					testing.expect(t, abs(f64(actual.x) - 0.002) < 1.3e-7);
					testing.expect(t, abs(f64(actual.y) + 0.002) < 1.3e-7);
					testing.expect_value(t, actual.z, f32(0));
				}
			}
		}
	}
	// a bad late lane must not publish any previously computed results
	a: util.I32x8 = 0;
	b: util.I32x8 = 1;
	b = simd.replace(b, 7, i32(16));
	delta: util.Vector3_Wide;
	status: physics.Physics_Status;
	delta, status = physics.bodies_gather_relative_positions(bodies, a, b, util.I32x8(-1));
	testing.expect_value(t, status, physics.Physics_Status.Not_Found);
	testing.expect_value(t, delta, util.Vector3_Wide{});
	b = simd.replace(b, 7, i32(-1));
	delta, status = physics.bodies_gather_relative_positions(bodies, a, b, util.I32x8(-1));
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, util.vector3_wide_read_slot(delta, 7), util.Vector3{});
	testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[0], {-f64(math.F32_MAX), 0, 0}), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[1], {f64(math.F32_MAX), 0, 0}), physics.Physics_Status.Ok);
	delta, status = physics.bodies_gather_relative_positions(bodies, util.I32x8(0), util.I32x8(1), util.I32x8(-1));
	testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, delta, util.Vector3_Wide{});
}

@(test)
precision_unconstrained_steps_accumulate_sub_float_ulp_motion :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	handles: [9]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description: physics.Body_Description = body_description();
		description.velocity.linear = {1.0 / 256, -1.0 / 128, 1.0 / 64};
		if index == 8
		{
			description.local_inertia = {};
		}
		status: physics.Physics_Status;
		handles[index], status = physics.simulation_add_body(scene.simulation, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handles[index], far_position(index)), physics.Physics_Status.Ok);
	}
	for _ in 0 ..< 1024
	{
		testing.expect_value(t, physics.simulation_timestep(scene.simulation, 0.125), physics.Physics_Status.Ok);
	}
	for index in 0 ..< len(handles)
	{
		expected: physics.Global_Position = far_position(index);
		expected.x += 0.5;
		expected.y -= 1;
		expected.z += 2;
		expect_position(t, &scene.simulation.bodies, handles[index], expected);
	}
}

@(test)
precision_constrained_substeps_integrate_double_positions_in_all_angular_modes :: proc(t: ^testing.T)
{
	modes: [3]physics.Angular_Integration_Mode = {.Nonconserving, .Conserve_Momentum, .Conserve_Momentum_With_Gyroscopic_Torque};
	for mode in modes
	{
		scene: Scene;
		scene_initialize(t, &scene);
		scene.simulation.integrator.callbacks.angular_mode = mode;
		handles: [3]physics.Body_Handle;
		for index in 0 ..< len(handles)
		{
			description: physics.Body_Description = body_description();
			description.velocity.linear = {1.0 / 256, -1.0 / 128, 1.0 / 64};
			if index == 2
			{
				description.local_inertia = {};
			}
			status: physics.Physics_Status;
			handles[index], status = physics.simulation_add_body(scene.simulation, &description);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handles[index], far_position(0)), physics.Physics_Status.Ok);
		}
		joint: physics.Ball_Socket = {spring_settings={angular_frequency=10, twice_damping_ratio=2}};
		for index in 0 ..< 2
		{
			connected: [4]physics.Body_Handle = {handles[index], handles[index + 1], {}, {}};
			status: physics.Physics_Status;
			_, status = physics.simulation_add_constraint(scene.simulation, &connected, &joint);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		for _ in 0 ..< 256
		{
			testing.expect_value(t, physics.simulation_timestep(scene.simulation, 0.125), physics.Physics_Status.Ok);
		}
		for handle in handles
		{
			expect_position(t, &scene.simulation.bodies, handle, {1e9 + 0.125, -1e9 - 0.25, 1e9 + 0.5});
		}
		scene_dispose(t, &scene);
	}
}

@(test)
precision_kinematic_global_target_uses_relative_motion_and_never_snaps_missing_motion :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	testing.expect_value(t, physics.body_control_initialize(scene.simulation, {input_capacity=8, settings_capacity=8},
		os.heap_allocator(), .Legacy), physics.Physics_Status.Ok);
	description: physics.Body_Description = body_description();
	description.local_inertia = {};
	handle: physics.Body_Handle;
	status: physics.Physics_Status;
	handle, status = physics.simulation_add_body(scene.simulation, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	start: physics.Global_Position = far_position(0);
	target: physics.Global_Position = {1e9 + 0.002, -1e9 - 0.002, 1e9};
	testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, start), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.body_control_target_set_global(scene.simulation.body_control, handle,
		target, util.quaternion_identity(), .Wake), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(scene.simulation, 0.125), physics.Physics_Status.Ok);
	expect_position(t, &scene.simulation.bodies, handle, target);
	// both float caches are identical. high precision must still detect that no
	// integration happened, rather than applying the ordinary roundoff snap
	target.x += 0.25;
	testing.expect_value(t, physics.body_control_target_set_global(scene.simulation.body_control, handle,
		target, util.quaternion_identity(), .Wake), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.body_control_step_prepare(scene.simulation.body_control, 0.125), physics.Physics_Status.Ok);
	scene.simulation.body_control.targets.values.memory[0].applied = .Present;
	physics.body_control_step_complete(scene.simulation.body_control, .Ok);
	expect_position(t, &scene.simulation.bodies, handle, {1e9 + 0.002, -1e9 - 0.002, 1e9});
}

@(test)
precision_reserved_gather_and_integration_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bodies: ^physics.Bodies = &scene.simulation.bodies;
	for index in 0 ..< 8
	{
		description: physics.Body_Description = body_description();
		handle: physics.Body_Handle;
		status: physics.Physics_Status;
		handle, status = physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.bodies_set_global_position(bodies, handle, far_position(index)), physics.Physics_Status.Ok);
	}
	indices: util.I32x8 = transmute(util.I32x8)[8]i32{0, 1, 2, 3, 4, 5, 6, 7};
	displacement: util.Vector3_Wide = util.vector3_wide_broadcast({1.0 / 1024, -1.0 / 512, 1.0 / 256});
	position: util.Vector3_Wide;
	before: i64 = scene.tracker.total_allocation_count;
	retained: u64 = util.buffer_pool_total_allocated_byte_count(&scene.pool);
	for _ in 0 ..< 4096
	{
		position = physics.bodies_integrate_global_positions_trusted(bodies, indices, util.I32x8(-1), displacement, position);
		delta: util.Vector3_Wide;
		status: physics.Physics_Status;
		delta, status = physics.bodies_gather_relative_positions(bodies, util.I32x8(0), indices, util.I32x8(-1));
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, simd.extract(delta.x, 7), f32(7.0 / 512));
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, before);
	testing.expect_value(t, util.buffer_pool_total_allocated_byte_count(&scene.pool), retained);
	testing.expect_value(t, simd.extract(position.x, 0), f32(1e9 + 4));
	actual: physics.Global_Position;
	status: physics.Physics_Status;
	actual, status = physics.bodies_get_global_position(bodies, {0});
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, physics.Global_Position{1e9 + 4, -1e9 - 8, 1e9 + 16});
}

Dispatch_Record :: struct
{
	inner: ^util.Thread_Dispatcher_Boundary,
	sleep_workers: int,
	wake_workers: int,
}

dispatch_record :: proc "contextless" (
	boundary: ^util.Thread_Dispatcher_Boundary, worker: util.Dispatcher_Worker_Proc,
	maximum_worker_count: int, unmanaged_context: rawptr,
) -> util.Threading_Status
{
	record: ^Dispatch_Record = (^Dispatch_Record)(boundary.dispatcher);
	if worker == physics.island_sleeper_body_gather_worker
	{
		record.sleep_workers = max(record.sleep_workers, maximum_worker_count);
	}
	if worker == physics.island_awakener_phase_one_worker
	{
		record.wake_workers = max(record.wake_workers, maximum_worker_count);
	}
	return record.inner.dispatch(record.inner, worker, maximum_worker_count, unmanaged_context);
}

@(private)
dispatch_pool :: proc (
	boundary: ^util.Thread_Dispatcher_Boundary, worker_index: int,
) -> (^util.Buffer_Pool, util.Threading_Status)
{
	record: ^Dispatch_Record = (^Dispatch_Record)(boundary.dispatcher);
	return record.inner.worker_pool(record.inner, worker_index);
}

@(test)
precision_actual_serial_and_parallel_island_transfers_preserve_positions :: proc(t: ^testing.T)
{
	worker_counts: [2]i32 = {1, 4};
	for workers in worker_counts
	{
		scene: Scene;
		scene_initialize(t, &scene, 32, workers);
		dispatcher: util.Thread_Dispatcher;
		testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, int(workers), 65536), util.Threading_Status.Ok);
		record: Dispatch_Record = {inner=util.thread_dispatcher_boundary(&dispatcher)};
		boundary: util.Thread_Dispatcher_Boundary = {
			dispatcher=&record, dispatch=dispatch_record, worker_pool=dispatch_pool, worker_count=int(workers),
		};
		handles: [16]physics.Body_Handle;
		bodies: ^physics.Bodies = &scene.simulation.bodies;
		status: physics.Physics_Status;
		for index in 0 ..< len(handles)
		{
			description: physics.Body_Description = body_description();
			handles[index], status = physics.simulation_add_body(scene.simulation, &description);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[index], far_position(index)), physics.Physics_Status.Ok);
			if index > 0
			{
				pair: [4]physics.Body_Handle = {handles[index - 1], handles[index], {}, {}};
				constraint: physics.Ball_Socket = {spring_settings={angular_frequency=10, twice_damping_ratio=2}};
				_, status = physics.simulation_add_constraint(scene.simulation, &pair, &constraint);
				testing.expect_value(t, status, physics.Physics_Status.Ok);
			}
		}
		for iteration in 0 ..< 4
		{
			for index in 0 ..< len(handles)
			{
				bodies.sets.memory[0].activity.memory[index].sleep_candidate = .Candidate;
			}
			sleeper: ^physics.Island_Sleeper = &scene.simulation.sleeper;
			testing.expect_value(t, physics.island_scaffold_clear(&sleeper.scaffold), physics.Physics_Status.Ok);
			body_count, constraint_count, can_sleep, build_status := physics.island_sleeper_build_island(sleeper, &sleeper.scaffold, handles[0]);
			testing.expect_value(t, build_status, physics.Physics_Status.Ok);
			testing.expect_value(t, can_sleep, physics.Reference_State.Present);
			testing.expect_value(t, body_count, 16);
			testing.expect_value(t, constraint_count, 15);
			testing.expect_value(t, physics.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count, &boundary), physics.Physics_Status.Ok);
			testing.expect_value(t, bodies.sets.memory[0].count, 0);
			// force another arena relocation while the real island is asleep
			testing.expect_value(t, physics.bodies_ensure_capacity(bodies, 128 << u32(iteration), 32, 4), physics.Physics_Status.Ok);
			for index in 0 ..< len(handles)
			{
				expect_position(t, bodies, handles[index], far_position(index));
			}
			testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, handles[7], &boundary), physics.Physics_Status.Ok);
			testing.expect_value(t, bodies.sets.memory[0].count, 16);
			for index in 0 ..< len(handles)
			{
				expect_position(t, bodies, handles[index], far_position(index));
			}
		}
		if workers > 1
		{
			testing.expect_value(t, record.sleep_workers, int(workers));
			testing.expect_value(t, record.wake_workers, int(workers));
		}
		testing.expect_value(t, util.thread_dispatcher_shutdown(&dispatcher), util.Threading_Status.Ok);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_public_pose_scatter_imports_floats_atomically_and_ignores_masked_values :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bodies: ^physics.Bodies = &scene.simulation.bodies;
	handles: [8]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		description: physics.Body_Description = body_description();
		status: physics.Physics_Status;
		handles[index], status = physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[index], far_position(index)), physics.Physics_Status.Ok);
	}
	indices: util.I32x8 = {0, 1, 2, 3, 4, 5, 6, 7};
	mask: util.I32x8 = util.I32x8{-1, -1, -1, -1, -1, -1, -1, -1};
	orientation: util.Quaternion_Wide = util.quaternion_wide_broadcast(util.quaternion_identity());
	positions: util.Vector3_Wide = util.vector3_wide_broadcast({1, 2, 3});
	positions.x = simd.replace(positions.x, 7, math.nan_f32());
	testing.expect_value(t, physics.bodies_scatter_active_pose(bodies, indices, mask, positions, orientation), physics.Physics_Status.Invalid_Argument);
	for index in 0 ..< len(handles)
	{
		expect_position(t, bodies, handles[index], far_position(index));
	}
	mask = simd.replace(mask, 7, 0);
	testing.expect_value(t, physics.bodies_scatter_active_pose(bodies, indices, mask, positions, orientation), physics.Physics_Status.Ok);
	for index in 0 ..< 7
	{
		expect_position(t, bodies, handles[index], {1, 2, 3});
	}
	expect_position(t, bodies, handles[7], far_position(7));
	mask = util.I32x8{-1, -1, -1, -1, -1, -1, -1, -1};
	indices = simd.replace(indices, 7, 8);
	positions = util.vector3_wide_broadcast({4, 5, 6});
	testing.expect_value(t, physics.bodies_scatter_active_pose(bodies, indices, mask, positions, orientation), physics.Physics_Status.Not_Found);
	for index in 0 ..< 7
	{
		expect_position(t, bodies, handles[index], {1, 2, 3});
	}
}

@(test)
precision_origin_translation_uses_authoritative_bodies_statics_and_targets :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	simulation: ^physics.Simulation = scene.simulation;
	testing.expect_value(t, physics.body_control_initialize(simulation, {input_capacity=8, settings_capacity=8},
		os.heap_allocator(), .Legacy), physics.Physics_Status.Ok);
	handles: [2]physics.Body_Handle;
	status: physics.Physics_Status;
	for index in 0 ..< len(handles)
	{
		description: physics.Body_Description = body_description();
		if index == 0
		{
			description.local_inertia = {};
		}
		handles[index], status = physics.bodies_add(&simulation.bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, physics.bodies_set_global_position(&simulation.bodies, handles[index], far_position(index)), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, physics.bodies_move(&simulation.bodies, handles[1], 1), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.body_control_target_set_global(simulation.body_control, handles[0],
		far_position(3), util.quaternion_identity(), .Wake), physics.Physics_Status.Ok);
	box: physics.Box = {0.5, 0.5, 0.5};
	shape: physics.Typed_Index;
	shape, status = physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	description: physics.Static_Description = {pose={orientation=util.quaternion_identity()}, shape=shape};
	static_handle: physics.Static_Handle;
	static_handle, status = physics.statics_add(&simulation.statics, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_set_global_position(&simulation.statics, static_handle, far_position(2)), physics.Physics_Status.Ok);
	delta: util.Vector3 = {1e9, -1e9, 1e9};
	testing.expect_value(t, physics.simulation_translate_origin(simulation, delta), physics.Physics_Status.Ok);
	for index in 0 ..< len(handles)
	{
		expect_position(t, &simulation.bodies, handles[index], {f64(index) / 512, -f64(index) / 256, f64(index) / 128});
	}
	actual: physics.Global_Position;
	actual, status = physics.statics_get_global_position(&simulation.statics, static_handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, physics.Global_Position{2.0 / 512, -2.0 / 256, 2.0 / 128});
	testing.expect_value(t, simulation.body_control.targets.values.memory[0].global_position,
		physics.Global_Position{3.0 / 512, -3.0 / 256, 3.0 / 128});
	testing.expect_value(t, physics.simulation_translate_origin(simulation, {math.inf_f32(1), 0, 0}), physics.Physics_Status.Invalid_Argument);
	expect_position(t, &simulation.bodies, handles[1], {1.0 / 512, -1.0 / 256, 1.0 / 128});
	testing.expect_value(t, physics.simulation_translate_origin(simulation, util.vector3_negate(delta)), physics.Physics_Status.Ok);
	for index in 0 ..< len(handles)
	{
		expect_position(t, &simulation.bodies, handles[index], far_position(index));
	}
	actual, status = physics.statics_get_global_position(&simulation.statics, static_handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, far_position(2));
	testing.expect_value(t, simulation.body_control.targets.values.memory[0].global_position, far_position(3));
}

@(test)
precision_static_shape_only_updates_and_failed_replacement_preserve_global_position :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	simulation: ^physics.Simulation = scene.simulation;
	box: physics.Box = {0.5, 0.5, 0.5};
	shape: physics.Typed_Index;
	status: physics.Physics_Status;
	shape, status = physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	description: physics.Static_Description = {pose={orientation=util.quaternion_identity()}, shape=shape};
	handle: physics.Static_Handle;
	handle, status = physics.simulation_add_static(simulation, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.statics_set_global_position(&simulation.statics, handle, far_position(7)), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_set_static_shape(simulation, handle, shape), physics.Physics_Status.Ok);
	actual: physics.Global_Position;
	actual, status = physics.statics_get_global_position(&simulation.statics, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, far_position(7));
	testing.expect_value(t, physics.simulation_set_static_shape_without_awakening_bodies(simulation, handle, shape), physics.Physics_Status.Ok);
	actual, status = physics.statics_get_global_position(&simulation.statics, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, far_position(7));
	// a syntactically valid but unregistered shape makes broad-phase insertion
	// fail after storage mutation, exercising the actual description rollback
	description.shape, status = physics.typed_index_create(physics.BOX_TYPE_ID, 12345);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect(t, physics.simulation_apply_static_description_without_awakening_bodies(simulation, handle, &description) != .Ok);
	actual, status = physics.statics_get_global_position(&simulation.statics, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, actual, far_position(7));
}

Fallible_Allocation :: struct
{
	backing: mem.Allocator,
	requests: int,
	reject_at: int,
}

@(private)
fallible_allocate :: proc (
	data: rawptr, mode: mem.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int, location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, mem.Allocator_Error)
{
	state: ^Fallible_Allocation = (^Fallible_Allocation)(data);
	if mode == .Alloc || mode == .Alloc_Non_Zeroed || mode == .Resize || mode == .Resize_Non_Zeroed
	{
		state.requests += 1;
		if state.reject_at > 0 && state.requests >= state.reject_at
		{
			return nil, .Out_Of_Memory;
		}
	}
	return state.backing.procedure(state.backing.data, mode, size, alignment, old_memory, old_size, location);
}

expect_no_pool_loans :: proc(t: ^testing.T, pool: ^util.Buffer_Pool)
{
	for power in 0 ..< len(pool.pools)
	{
		value: ^util.Power_Pool = &pool.pools[power];
		testing.expect_value(t, value.next_slot - value.free_count, 0);
	}
}

@(test)
precision_position_column_allocation_failures_preserve_owners_and_return_loans :: proc(t: ^testing.T)
{
	// fail each backing-allocation boundary for body and static initialization
	// and growth, retaining live owners and returning every borrowed buffer
	for fail_after in 1 ..= 32
	{
		for operation in 0 ..< 5
		{
			tracker: mem.Tracking_Allocator;
			mem.tracking_allocator_init(&tracker, os.heap_allocator(), os.heap_allocator());
			state: Fallible_Allocation = {backing=mem.tracking_allocator(&tracker)};
			pool: util.Buffer_Pool;
			testing.expect_value(t, util.buffer_pool_initialize_with_allocator(&pool, {procedure=fallible_allocate, data=&state}, 128), util.Memory_Status.Ok);
			set: physics.Body_Set;
			arena: physics.Body_Set_Arena;
			statics: physics.Statics;
			shapes: physics.Shape_Registry;
			handle: physics.Static_Handle;
			description_static: physics.Static_Description;
			if operation == 1
			{
				testing.expect_value(t, physics.body_set_initialize(&set, 8, &pool), physics.Physics_Status.Ok);
				description: physics.Body_Description = body_description();
				index: int;
				status: physics.Physics_Status;
				index, status = physics.body_set_add(&set, &description, {0}, 4, &pool);
				testing.expect_value(t, status, physics.Physics_Status.Ok);
				set.global_positions.memory[index] = far_position(7);
				set.dynamics_state.memory[index].motion.pose.position = physics.world_precision_position_cache(far_position(7));
			}
			if operation == 4
			{
				testing.expect_value(t, physics.shape_registry_initialize(&shapes, 8, &pool), physics.Physics_Status.Ok);
				testing.expect_value(t, physics.statics_initialize(&statics, 8, &pool), physics.Physics_Status.Ok);
				testing.expect_value(t, physics.statics_bind_shape_registry(&statics, &shapes), physics.Physics_Status.Ok);
				box: physics.Box = {0.5, 0.5, 0.5};
				shape: physics.Typed_Index;
				shape, _ = physics.shape_registry_add(&shapes, physics.BOX_TYPE_ID, &box);
				description_static = {pose={orientation=util.quaternion_identity()}, shape=shape};
				handle, _ = physics.statics_add(&statics, &description_static);
				testing.expect_value(t, physics.statics_set_global_position(&statics, handle, far_position(7)), physics.Physics_Status.Ok);
			}
			original_statics: rawptr = statics.statics.memory;
			original_positions: rawptr = statics.global_positions.memory;
			original_bounds: rawptr = statics.local_bounds.memory;
			state.reject_at = state.requests + fail_after;
			status: physics.Physics_Status;
			if operation == 0
			{
				status = physics.body_set_initialize(&set, 256, &pool);
			}
			else if operation == 1
			{
				status = physics.body_set_ensure_capacity(&set, 256, &pool);
				testing.expect_value(t, set.count, 1);
				testing.expect_value(t, set.global_positions.memory[0], far_position(7));
				testing.expect_value(t, set.dynamics_state.memory[0].motion.pose.position, physics.world_precision_position_cache(far_position(7)));
			}
			else if operation == 2
			{
				status = physics.body_set_arena_initialize(&arena, 256, &pool);
			}
			else if operation == 3
			{
				status = physics.statics_initialize(&statics, 256, &pool);
			}
			else
			{
				status = physics.statics_ensure_capacity(&statics, 256);
				testing.expect_value(t, statics.count, 1);
				position: physics.Global_Position;
				position, _ = physics.statics_get_global_position(&statics, handle);
				testing.expect_value(t, position, far_position(7));
				actual: physics.Static_Description;
				actual, _ = physics.statics_get_description(&statics, handle);
				testing.expect_value(t, actual.shape, description_static.shape);
				if status != .Ok
				{
					testing.expect_value(t, rawptr(statics.statics.memory), original_statics);
					testing.expect_value(t, rawptr(statics.global_positions.memory), original_positions);
					testing.expect_value(t, rawptr(statics.local_bounds.memory), original_bounds);
				}
			}
			testing.expect(t, status == .Ok || status == .Capacity_Missing);
			if set.state == .Allocated
			{
				testing.expect_value(t, physics.body_set_dispose(&set, &pool), physics.Physics_Status.Ok);
			}
			if arena.state == .Allocated
			{
				testing.expect_value(t, physics.body_set_arena_dispose(&arena, &pool), physics.Physics_Status.Ok);
			}
			if statics.state == .Allocated
			{
				testing.expect_value(t, physics.statics_dispose(&statics), physics.Physics_Status.Ok);
			}
			if shapes.state == .Allocated
			{
				testing.expect_value(t, physics.shape_registry_dispose(&shapes), physics.Physics_Status.Ok);
			}
			expect_no_pool_loans(t, &pool);
			testing.expect_value(t, util.buffer_pool_dispose(&pool), util.Memory_Status.Ok);
			testing.expect_value(t, tracker.current_memory_allocated, i64(0));
			mem.tracking_allocator_destroy(&tracker);
		}
	}
}

@(test)
precision_integration_obeys_every_mask_and_missing_lane_without_cache_feedback :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	bodies: ^physics.Bodies = &scene.simulation.bodies;
	handles: [8]physics.Body_Handle;
	for index in 0 ..< 8
	{
		description: physics.Body_Description = body_description();
		status: physics.Physics_Status;
		handles[index], status = physics.bodies_add(bodies, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	displacement: util.Vector3_Wide = util.vector3_wide_broadcast({1.0 / 512, -1.0 / 256, 1.0 / 128});
	previous: util.Vector3_Wide = util.vector3_wide_broadcast({-71, 89, 13});
	for mask_bits in 0 ..< 256
	{
		indices: util.I32x8;
		mask: util.I32x8;
		for lane in 0 ..< 8
		{
			testing.expect_value(t, physics.bodies_set_global_position(bodies, handles[lane], far_position(lane)), physics.Physics_Status.Ok);
			if mask_bits & (1 << u32(lane)) != 0
			{
				indices = simd.replace(indices, lane, i32(lane));
				mask = simd.replace(mask, lane, -1);
			}
			else
			{
				indices = simd.replace(indices, lane, i32(100000));
			}
		}
		// a masked-in absent reference must never index the dense column
		indices = simd.replace(indices, 7, -1);
		result: util.Vector3_Wide = physics.bodies_integrate_global_positions_trusted(bodies, indices, mask, displacement, previous);
		for lane in 0 ..< 8
		{
			expected: physics.Global_Position = far_position(lane);
			cache: util.Vector3 = {-71, 89, 13};
			if lane != 7 && mask_bits & (1 << u32(lane)) != 0
			{
				expected = far_position(lane + 1);
				cache = physics.world_precision_position_cache(expected);
			}
			actual, status := physics.bodies_get_global_position(bodies, handles[lane]);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
			testing.expect_value(t, actual, expected);
			testing.expect_value(t, util.vector3_wide_read_slot(result, lane), cache);
		}
	}
}

@(test)
precision_origin_shift_keeps_sub_cache_residue_after_simulation_without_other_state_changes :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	simulation: ^physics.Simulation = scene.simulation;
	description: physics.Body_Description = body_description();
	description.pose.position = {512, 10, -512};
	description.velocity.linear = {0.001, -0.003, 0.007};
	handle: physics.Body_Handle;
	status: physics.Physics_Status;
	handle, status = physics.simulation_add_body(simulation, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 0.125), physics.Physics_Status.Ok);
	before: physics.Body_Description;
	before, status = physics.bodies_get_description(&simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	position: physics.Global_Position;
	position, status = physics.bodies_get_global_position(&simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	shift: util.Vector3 = {512, 10, -512};
	expected: physics.Global_Position = {position.x - f64(shift.x), position.y - f64(shift.y), position.z - f64(shift.z)};
	// this deliberately distinguishes the mixed contract from the old exact
	// float-cache subtraction oracle. no tolerance masks the difference
	legacy: util.Vector3 = util.vector3_subtract(before.pose.position, shift);
	testing.expect(t, legacy != physics.world_precision_position_cache(expected));
	testing.expect_value(t, physics.simulation_translate_origin(simulation, shift), physics.Physics_Status.Ok);
	expect_position(t, &simulation.bodies, handle, expected);
	after: physics.Body_Description;
	after, status = physics.bodies_get_description(&simulation.bodies, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	before.pose.position = physics.world_precision_position_cache(expected);
	testing.expect_value(t, after, before);
}
