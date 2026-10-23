package precision_targets_tests

import "core:math"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_target_boundary_rejects_invalid_mutations_before_waking :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		scene: Scene;
		scene_initialize(t, &scene);
		body: physics.Body_Handle = add_body(t, &scene, {1e9, 0, 0});
		handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {1e9, 0, 0}, kind);
		target: physics.Global_Position = {1e9 + 0.001953125, 0, 0};
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
		sleep_body(t, &scene, body);
		set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
		invalid: [4]physics.Global_Position = {
			{transmute(f64)u64(0x7ff8000000000001), 0, 0},
			{0, transmute(f64)u64(0x7ff0000000000000), 0},
			{0, 0, f64(math.F32_MAX) * 2}, {-f64(math.F32_MAX) * 2, 0, 0},
		};
		for value in invalid
		{
			testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, value), physics.Physics_Status.Invalid_Argument);
			expect_target(t, &scene, handle, target);
			testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
		}
		testing.expect_value(t, physics.simulation_set_constraint_global_target(nil, handle, target), physics.Physics_Status.Disposed);
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, {-1}, target), physics.Physics_Status.Not_Found);
		scene.simulation.state = .Stepping;
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Disposed);
		scene.simulation.state = .Ready;
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
		expect_target(t, &scene, handle, target);
		testing.expect_value(t, physics.solver_remove(&scene.simulation.solver, handle), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Not_Found);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_target_relative_overflow_rejects_without_kernel_scatter :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		for route in Target_Route
		{
			if kind == .Axis_Lock && (route == .Direct_Bundle || route == .Direct_Lane)
			{
				continue;
			}
			scene: Scene;
			scene_initialize(t, &scene);
			body: physics.Body_Handle = add_body(t, &scene, {-f64(math.F32_MAX), 0, 0});
			handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {math.F32_MAX, 0, 0}, kind);
			motion: ^physics.Motion_State = body_motion(t, &scene, body);
			motion.velocity = {linear={0.125, 0.25, 0.5}, angular={0.0625, 0.125, 0.25}};
			before: physics.Motion_State = motion^;
			batch: ^physics.Type_Batch = constraint_batch(t, &scene, handle);
			before_impulses: [6]util.F32x8;
			for row in 0 ..< int(batch.impulse_bundle_size) / size_of(util.F32x8)
			{
				before_impulses[row] = ([^]util.F32x8)(physics.type_batch_impulse_bundle(batch, 0))[row];
			}
			testing.expect_value(t, execute_target_stage(t, &scene, handle, route, .Warmstart), physics.Physics_Status.Invalid_Argument);
			testing.expect_value(t, execute_target_stage(t, &scene, handle, route, .Solve), physics.Physics_Status.Invalid_Argument);
			testing.expect_value(t, motion^, before);
			for row in 0 ..< int(batch.impulse_bundle_size) / size_of(util.F32x8)
			{
				testing.expect_value(t, ([^]util.F32x8)(physics.type_batch_impulse_bundle(batch, 0))[row], before_impulses[row]);
			}
			scene_dispose(t, &scene);
		}
	}
	// overflow on an unlocked axis is irrelevant, including angular-only locks
	for axes in ([2]u8{0, 2})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		body: physics.Body_Handle = add_body(t, &scene, {-f64(math.F32_MAX), 1e9, 0});
		handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {math.F32_MAX, 1e9, 0}, .Axis_Lock, axes, angular_axes=7, orientation={0, 0, 0.70710677, 0.70710677});
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, {f64(math.F32_MAX), 1e9 + 0.001953125, 0}), physics.Physics_Status.Ok);
		testing.expect_value(t, execute_target_stage(t, &scene, handle, .Registered_Bundle, .Solve), physics.Physics_Status.Ok);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_target_boundary_does_not_reinterpret_other_one_body_constraints :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	body: physics.Body_Handle = add_body(t, &scene, {1e9, 0, 0});
	description: physics.One_Body_Angular_Servo = {util.quaternion_identity(),
		{angular_frequency=12.5, twice_damping_ratio=1.5}, {maximum_speed=100, maximum_force=1e6}};
	references: [4]physics.Body_Handle = {body, {}, {}, {}};
	handle: physics.Constraint_Handle;
	status: physics.Physics_Status;
	handle, status = physics.solver_add(&scene.simulation.solver, &references, &description);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	for pass in 0 ..< 2
	{
		if pass == 1
		{
			sleep_body(t, &scene, body);
		}
		before: physics.Body_Memory_Location = scene.simulation.bodies.handle_to_location.memory[body.value];
		_, status = physics.simulation_get_constraint_global_target(scene.simulation, handle);
		testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, {1e9, 0, 0}), physics.Physics_Status.Invalid_Argument);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value], before);
	}
}
