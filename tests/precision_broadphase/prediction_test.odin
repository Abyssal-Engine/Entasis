package precision_broadphase_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

expect_linear_prediction :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Body_Handle, velocity: util.Vector3, dt: f32)
{
	location: physics.Body_Memory_Location;
	location, _ = physics.bodies_resolve(&scene.simulation.bodies, handle);
	set: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[location.set_index];
	collidable: ^physics.Collidable = &set.collidables.memory[location.index];
	if physics.typed_index_state(collidable.shape) != .Present
	{
		return;
	}
	local: physics.Shape_Bounds = local_bounds(t, scene, collidable.shape, set.dynamics_state.memory[location.index].motion.pose.orientation);
	displacement: util.Vector3 = {velocity.x * dt, velocity.y * dt, velocity.z * dt};
	limit: f32 = 0.0625;
	if collidable.continuity.mode != .Discrete
	{
		limit = math.F32_MAX;
	}
	minimum: util.Vector3 = {
		local.min.x + max(-limit, min(f32(0), displacement.x)),
		local.min.y + max(-limit, min(f32(0), displacement.y)),
		local.min.z + max(-limit, min(f32(0), displacement.z)),
	};
	maximum: util.Vector3 = {
		local.max.x + min(limit, max(f32(0), displacement.x)),
		local.max.y + min(limit, max(f32(0), displacement.y)),
		local.max.z + min(limit, max(f32(0), displacement.z)),
	};
	expect_enclosure(t, body_bounds(t, scene, handle), set.global_positions.memory[location.index], util.Bounding_Box{min=minimum, max=maximum});
	expect_precise_exact(t, body_precise_bounds(t, scene, handle), set.global_positions.memory[location.index], util.Bounding_Box{min=minimum, max=maximum});
	testing.expect_value(t, collidable.speculative_margin, f32(0.0625));
}

@(test)
precision_prediction_homogeneous_shapes_all_tail_lengths :: proc(t: ^testing.T)
{
	// homogeneous primitive packets, other built-in type batches, and custom
	// scalar callbacks must translate only after their local motion envelope
	for shape_index in 0 ..< 10
	{
		for count in 1 ..= 17
		{
			scene: Scene;
			scene_initialize(t, &scene, 32, 1);
			handles: [17]physics.Body_Handle;
			for index in 0 ..< count
			{
				position: physics.Global_Position = {1e9 + 32.001953125 + f64(index) / 128, -1e9 - 16.00390625, 1e9 + 0.001953125};
				if count & 1 == 0
				{
					position = {32.001953125 + f64(index) / 128, -16.00390625, 0.001953125};
				}
				handles[index] = add_body(t, &scene, scene.shapes[shape_index], position);
				active: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[0];
				active.dynamics_state.memory[index].motion.velocity.linear = {8, -4, 2};
				// exact signed-axis permutations also expose orientation/lane mixups
				// without introducing a different trigonometric rounding oracle
				if index & 1 == 0
				{
					active.dynamics_state.memory[index].motion.pose.orientation = {0.5, 0.5, 0.5, 0.5};
				}
				else
				{
					active.dynamics_state.memory[index].motion.pose.orientation = {0, 0, 1, 0};
				}
				if index % 3 == 1
				{
					active.collidables.memory[index].continuity.mode = .Passive;
				}
				else if index % 3 == 2
				{
					active.collidables.memory[index].continuity.mode = .Continuous;
				}
			}
			testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), physics.Physics_Status.Ok);
			testing.expect_value(t, scene.simulation.broad_phase.bounds_state, physics.Broad_Phase_Bounds_State.Predicted);
			for index in 0 ..< count
			{
				expect_linear_prediction(t, &scene, handles[index], {8, -4, 2}, 0.125);
			}
			scene_dispose(t, &scene);
		}
	}
}

Callback_Record :: struct
{
	visits: [512]u32,
}

predict_velocity :: proc "contextless" (
	user_context: rawptr, body_indices: util.I32x8,
	position: util.Vector3_Wide, orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	integration_mask: util.I32x8, worker_index: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	_ = position;
	_ = orientation;
	_ = inertia;
	_ = worker_index;
	_ = dt;
	record: ^Callback_Record = (^Callback_Record)(user_context);
	for lane in 0 ..< util.PRODUCTION_LANE_COUNT
	{
		index: int = int(simd.extract(body_indices, lane));
		if index >= 0 && index < len(record.visits) && simd.extract(integration_mask, lane) != 0
		{
			record.visits[index] += 1;
			linear: util.Vector3 = util.vector3_wide_read_slot(velocity.linear, lane);
			util.vector3_wide_write_slot(&velocity.linear, lane, {linear.x + 2, linear.y - 1, linear.z + 0.5});
		}
	}
}

@(test)
precision_prediction_grouped_holes_callbacks_and_parallel_refit :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 512, 4);
	defer scene_dispose(t, &scene);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	handles: [401]physics.Body_Handle;
	reference: [401]physics.Global_Bounds;
	for index in 0 ..< len(handles)
	{
		shape: physics.Typed_Index = scene.shapes[index % 10];
		if index % 13 == 0
		{
			shape = {};
		}
		handles[index] = add_body(t, &scene, shape, {1e9 + 32.001953125 + f64(index) / 32, -1e9 - 16.00390625, 1e9});
		active: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[0];
		active.dynamics_state.memory[index].motion.velocity.linear = {8, -4, 2};
		active.collidables.memory[index].continuity.mode = .Passive;
	}
	record: Callback_Record;
	original: physics.Pose_Integrator_Callbacks = scene.simulation.integrator.callbacks;
	scene.simulation.integrator.callbacks.integrate_velocity = predict_velocity;
	scene.simulation.integrator.callbacks.user_context = &record;
	defer scene.simulation.integrator.callbacks = original;
	for iteration in 0 ..< 9
	{
		record = {};
		selected: ^util.Thread_Dispatcher_Boundary;
		if iteration != 0
		{
			selected = boundary;
		}
		testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125, &scene.simulation.integrator, selected), physics.Physics_Status.Ok);
		for handle, index in handles
		{
			testing.expect_value(t, record.visits[index], u32(1));
			if index % 13 == 0
			{
				continue;
			}
			expect_linear_prediction(t, &scene, handle, {10, -5, 2.5}, 0.125);
			actual: physics.Global_Bounds = body_bounds(t, &scene, handle);
			if iteration == 0
			{
				reference[index] = actual;
			}
			else
			{
				testing.expect_value(t, actual, reference[index]);
			}
			// predict callbacks must not commit their temporary velocity to state
			testing.expect_value(t, scene.simulation.bodies.sets.memory[0].dynamics_state.memory[index].motion.velocity.linear, util.Vector3{8, -4, 2});
		}
		// more than 256 live leaves enters actual threaded maintenance/refinement
		testing.expect_value(t, physics.broad_phase_update(&scene.simulation.broad_phase, selected), physics.Physics_Status.Ok);
		for handle, index in handles
		{
			if index % 13 != 0
			{
				testing.expect_value(t, body_bounds(t, &scene, handle), reference[index]);
			}
		}
	}
}

@(test)
precision_prepared_prediction_and_bounds_updates_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	for index in 0 ..< 97
	{
		_ = add_body(t, &scene, scene.shapes[index % 10], {1e9 + 32.001953125, -1e9 - f64(index) / 128, 1e9});
		scene.simulation.bodies.sets.memory[0].dynamics_state.memory[index].motion.velocity.linear = {8, -4, 2};
	}
	testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for _ in 0 ..< 1024
	{
		testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

@(test)
precision_prediction_rejects_invalid_leaf_before_writes :: proc(t: ^testing.T)
{
	// exercise contiguous primitive, indexed built-in, and scalar custom stores
	shape_indices: [3]int = {0, 5, 9};
	for shape_index in shape_indices
	{
		scene: Scene;
		scene_initialize(t, &scene);
		handle: physics.Body_Handle = add_body(t, &scene, scene.shapes[shape_index], {1e9 + 32.001953125, 0, 0});
		before: physics.Global_Bounds = body_bounds(t, &scene, handle);
		precise_before: physics.Global_Bounds = body_precise_bounds(t, &scene, handle);
		before_margin: f32 = scene.simulation.bodies.sets.memory[0].collidables.memory[0].speculative_margin;
		limit: f64 = f64(f32(math.F32_MAX));
		// raw setters deliberately leave tree synchronization to their owner
		testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, handle, {limit, 0, 0}), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, body_bounds(t, &scene, handle), before);
		testing.expect_value(t, body_precise_bounds(t, &scene, handle), precise_before);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].collidables.memory[0].speculative_margin, before_margin);
		testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, body_bounds(t, &scene, handle), before);
		testing.expect_value(t, body_precise_bounds(t, &scene, handle), precise_before);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_finite_float_domain_uses_exact_binary32_limit :: proc(t: ^testing.T)
{
	limit: f64 = f64(transmute(f32)u32(0x7f7f_ffff));
	outside: f64 = math.nextafter_f64(limit, math.inf_f64(1));
	signs: [2]f64 = {-1, 1};
	for sign in signs
	{
		testing.expect_value(t, physics.world_precision_position_validate({sign * limit, 0, 0}), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.world_precision_position_validate({sign * outside, 0, 0}), physics.Physics_Status.Invalid_Argument);
		bounds: physics.Tree_Bounds;
		status: physics.Physics_Status;
		bounds, status = physics.broad_phase_bounds_from_local({sign * limit, 0, 0}, {});
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, f64(bounds.min.x), sign * limit);
		testing.expect_value(t, f64(bounds.max.x), sign * limit);
		local: util.Bounding_Box = {min={-0.0625, 0, 0}, max={0.0625, 0, 0}};
		_, status = physics.broad_phase_bounds_from_local({sign * limit, 0, 0}, local);
		testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
	}
}
