package precision_targets_tests

import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_double_targets_cover_bundle_lane_routes_and_tails :: proc(t: ^testing.T)
{
	for kind in Target_Kind
	{
		for route in Target_Route
		{
			if kind == .Axis_Lock && (route == .Direct_Bundle || route == .Direct_Lane)
			{
				continue;
			}
			for count in 1 ..= 17
			{
				scenes: [2]Scene;
				bodies: [2][17]physics.Body_Handle;
				handles: [2][17]physics.Constraint_Handle;
				for variant in 0 ..< 2
				{
					scene_initialize(t, &scenes[variant]);
					base: f64 = f64(variant) * 1e9;
					for index in 0 ..< count
					{
						body: physics.Body_Handle = add_body(t, &scenes[variant], {base + f64(index), -base + 0.25, base - 0.125});
						bodies[variant][index] = body;
						motion: ^physics.Motion_State = body_motion(t, &scenes[variant], body);
						motion.velocity = {linear={0.125, -0.25, 0.0625}, angular={0.0625, 0.125, -0.25}};
						target: physics.Global_Position = {base + f64(index) + 0.001953125, -base + 0.28125, base - 0.15625};
						handle: physics.Constraint_Handle = add_float_target(t, &scenes[variant], body,
							physics.world_precision_position_cache(target), kind, 7, {0.125, -0.25, 0.5});
						handles[variant][index] = handle;
						testing.expect_value(t, physics.simulation_set_constraint_global_target(scenes[variant].simulation, handle, target), physics.Physics_Status.Ok);
						batch: ^physics.Type_Batch = constraint_batch(t, &scenes[variant], handle);
						for row in 0 ..< int(batch.impulse_bundle_size) / size_of(util.F32x8)
						{
							impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(batch, index));
							impulses[row] = simd.replace(impulses[row], index % 8, f32(row + 1) * 0.015625);
						}
					}
				}
				for iteration in 0 ..< 4
				{
					stage: physics.Solver_Execution_Stage = .Solve;
					if iteration == 0
					{
						stage = .Warmstart;
					}
					for variant in 0 ..< 2
					{
						testing.expect_value(t, execute_target_stage(t, &scenes[variant], handles[variant][0], route, stage), physics.Physics_Status.Ok);
					}
					for index in 0 ..< count
					{
						testing.expect_value(t, body_motion(t, &scenes[0], bodies[0][index]).velocity, body_motion(t, &scenes[1], bodies[1][index]).velocity);
						a, b: physics.Constraint_Reaction_Sample;
						testing.expect_value(t, physics.constraint_capture_reaction(&scenes[0].simulation.solver, handles[0][index], 1.0 / 64, {}, &a), physics.Physics_Status.Ok);
						testing.expect_value(t, physics.constraint_capture_reaction(&scenes[1].simulation.solver, handles[1][index], 1.0 / 64, {}, &b), physics.Physics_Status.Ok);
						testing.expect_value(t, a, b);
					}
				}
				for variant in 0 ..< 2
				{
					scene_dispose(t, &scenes[variant]);
				}
			}
		}
	}
}

@(test)
precision_target_preparation_respects_all_lane_masks_and_locked_axes :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	handles: [8]physics.Constraint_Handle;
	for lane in 0 ..< 8
	{
		body: physics.Body_Handle = add_body(t, &scene, {1e9 + f64(lane), -1e9, 1e9});
		handles[lane] = add_float_target(t, &scene, body, {}, .Axis_Lock, u8(lane));
		testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handles[lane],
			{1e9 + f64(lane) + 0.001953125, -1e9 + 0.125, 1e9 - 0.25}), physics.Physics_Status.Ok);
	}
	batch: ^physics.Type_Batch = constraint_batch(t, &scene, handles[0]);
	for mask in 0 ..< 256
	{
		active: util.I32x8;
		references: util.I32x8 = util.I32x8(0x7fff_ffff);
		for lane in 0 ..< 8
		{
			if mask & (1 << u32(lane)) != 0
			{
				active = simd.replace(active, lane, -1);
				references = simd.replace(references, lane, i32(u32(lane) | physics.BODY_REFERENCE_KINEMATIC_MASK));
			}
		}
		body: physics.Constraint_Kernel_Body_Wide = {position=util.vector3_wide_broadcast({99, 98, 97})};
		prestep: physics.Constraint_Local_Target_Prestep;
		testing.expect_value(t, physics.solver_prepare_constraint_global_target(&scene.simulation.bodies,
			physics.BODY_AXIS_LOCK_TYPE_ID, physics.type_batch_prestep_bundle(batch, 0), references, active, &body, &prestep), physics.Physics_Status.Ok);
		testing.expect_value(t, body.position, util.Vector3_Wide{});
		for lane in 0 ..< 8
		{
			expected: util.Vector3;
			if mask & (1 << u32(lane)) != 0
			{
				if lane & 1 != 0
				{
					expected.x = 0.001953125;
				}
				if lane & 2 != 0
				{
					expected.y = 0.125;
				}
				if lane & 4 != 0
				{
					expected.z = -0.25;
				}
			}
			testing.expect_value(t, util.vector3_wide_read_slot(prestep.axis_lock.position, lane), expected);
		}
	}
}
