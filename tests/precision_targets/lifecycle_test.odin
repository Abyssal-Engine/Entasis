package precision_targets_tests

import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

expect_target :: proc(t: ^testing.T, scene: ^Scene, handle: physics.Constraint_Handle, expected: physics.Global_Position)
{
	actual: physics.Global_Position;
	status: physics.Physics_Status;
	actual, status = physics.simulation_get_constraint_global_target(scene.simulation, handle);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, transmute([3]u64)actual, transmute([3]u64)expected);
}

sleep_body :: proc(t: ^testing.T, scene: ^Scene, body: physics.Body_Handle)
{
	index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].index;
	scene.simulation.bodies.sets.memory[0].activity.memory[index].sleep_candidate = .Candidate;
	sleeper: ^physics.Island_Sleeper = &scene.simulation.sleeper;
	testing.expect_value(t, physics.island_scaffold_clear(&sleeper.scaffold), physics.Physics_Status.Ok);
	body_count, constraint_count: int;
	can_sleep: physics.Reference_State;
	status: physics.Physics_Status;
	body_count, constraint_count, can_sleep, status = physics.island_sleeper_build_island(sleeper, &sleeper.scaffold, body);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	testing.expect_value(t, can_sleep, physics.Reference_State.Present);
	testing.expect_value(t, physics.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), physics.Physics_Status.Ok);
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
}

@(test)
precision_targets_preserve_bits_across_growth_lane_moves_and_handle_reuse :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		scene: Scene;
		scene_initialize(t, &scene, 16);
		bodies: [65]physics.Body_Handle;
		handles: [65]physics.Constraint_Handle;
		targets: [65]physics.Global_Position;
		spacer: physics.Body_Handle = add_body(t, &scene, {-123, 0, 0});
		for index in 0 ..< 65
		{
			bodies[index] = add_body(t, &scene, {1e9, -1e9, 1e9});
			// the low halves deliberately include NaN-looking f32 bit patterns.
			// they are finite f64 positions and must be transported as opaque bits
			x: f64 = transmute(f64)((transmute(u64)f64(1e9) & 0xffffffff00000000) | (0x7fc00000 + u64(index)));
			targets[index] = {x, -x, 1e9 + f64(index) * 0.001953125};
			handles[index] = add_float_target(t, &scene, bodies[index], {1e9, -1e9, 1e9}, kind);
			testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handles[index], targets[index]), physics.Physics_Status.Ok);
		}
		testing.expect_value(t, physics.solver_remove(&scene.simulation.solver, handles[7]), physics.Physics_Status.Ok);
		replacement: physics.Constraint_Handle = add_float_target(t, &scene, bodies[7], {1e9, -1e9, 1e9}, kind);
		testing.expect_value(t, replacement, handles[7]);
		targets[7] = {1e9, -1e9, 1e9};
		// another one-body constraint forces a second sequential batch to exist
		second: physics.Constraint_Handle = add_float_target(t, &scene, bodies[0], {1e9, -1e9, 1e9}, kind);
		testing.expect_value(t, scene.simulation.solver.handle_to_constraint.memory[second.value].batch_index, i32(1));
		testing.expect_value(t, physics.solver_move_constraint(&scene.simulation.solver, handles[20], 1), physics.Physics_Status.Ok);
		expect_target(t, &scene, handles[20], targets[20]);
		testing.expect_value(t, physics.solver_move_constraint(&scene.simulation.solver, handles[20], 0), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.bodies_remove(&scene.simulation.bodies, spacer), physics.Physics_Status.Ok);
		for index in 0 ..< 65
		{
			expect_target(t, &scene, handles[index], targets[index]);
		}
		scene_dispose(t, &scene);
	}
}

@(test)
precision_targets_survive_sleep_wake_origin_shifts_and_float_replacement :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		scene: Scene;
		scene_initialize(t, &scene);
		body: physics.Body_Handle = add_body(t, &scene, {1e9, -1e9, 1e9});
		handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {1e9, -1e9, 1e9}, kind);
		target: physics.Global_Position = {1e9 + 0.001953125, -1e9 - 0.00390625, 1e9 + 0.0078125};
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
		batch: ^physics.Type_Batch = constraint_batch(t, &scene, handle);
		impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(batch, 0));
		for row in 0 ..< int(batch.impulse_bundle_size) / size_of(util.F32x8)
		{
			impulses[row] = simd.replace(impulses[row], 0, f32(row + 1) * 0.125);
		}
		sleep_body(t, &scene, body);
		expect_target(t, &scene, handle, target);
		shift: util.Vector3 = {512, -256, 128};
		testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, shift), physics.Physics_Status.Ok);
		target = {target.x - 512, target.y + 256, target.z - 128};
		expect_target(t, &scene, handle, target);
		testing.expect_value(t, physics.bodies_ensure_capacity(&scene.simulation.bodies, 512, 32, 8), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, body), physics.Physics_Status.Ok);
		expect_target(t, &scene, handle, target);
		batch = constraint_batch(t, &scene, handle);
		impulses = ([^]util.F32x8)(physics.type_batch_impulse_bundle(batch, 0));
		for row in 0 ..< int(batch.impulse_bundle_size) / size_of(util.F32x8)
		{
			testing.expect_value(t, simd.extract(impulses[row], 0), f32(row + 1) * 0.125);
		}
		testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, {-512, 256, -128}), physics.Physics_Status.Ok);
		target = {target.x + 512, target.y - 256, target.z + 128};
		expect_target(t, &scene, handle, target);
		sleep_body(t, &scene, body);
		target.x += 0.015625;
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, i32(0));
		expect_target(t, &scene, handle, target);
		if kind == .Servo
		{
			description: physics.One_Body_Linear_Servo;
			testing.expect_value(t, physics.solver_get_description(&scene.simulation.solver, handle, &description), physics.Physics_Status.Ok);
			testing.expect_value(t, description.target, physics.world_precision_position_cache(target));
			testing.expect_value(t, physics.solver_apply_description(&scene.simulation.solver, handle, &description), physics.Physics_Status.Ok);
			expect_target(t, &scene, handle, physics.world_precision_position_from_float(description.target));
		}
		else
		{
			description: physics.Body_Axis_Lock_Description;
			testing.expect_value(t, physics.solver_get_description_raw(&scene.simulation.solver, handle, physics.BODY_AXIS_LOCK_TYPE_ID, &description, size_of(description)), physics.Physics_Status.Ok);
			testing.expect_value(t, description.position, physics.world_precision_position_cache(target));
			testing.expect_value(t, physics.solver_apply_description_raw(&scene.simulation.solver, handle, physics.BODY_AXIS_LOCK_TYPE_ID, &description), physics.Physics_Status.Ok);
			expect_target(t, &scene, handle, physics.world_precision_position_from_float(description.position));
		}
		scene_dispose(t, &scene);
	}
}
