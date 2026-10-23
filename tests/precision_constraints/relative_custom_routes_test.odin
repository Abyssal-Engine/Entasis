package precision_constraints_tests

import "core:math"
import "core:simd"
import "core:testing"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

exercise_relative_custom_routes :: proc(t: ^testing.T, $dispatch: p.Constraint_Callback_Dispatch)
{
	for count in 1 ..= 4
	{
		for origin in ([4]f64{0, 1e4, 1e6, 1e9})
		{
			scene: Scene;
			scene_initialize(t, &scene);
			when dispatch == .Contextual
			{
				probe_context: Relative_Probe_Context = {marker=7};
				binding: p.Contextual_Constraint_Binding = relative_probe_binding(&probe_context);
			}
			all: u8 = u8((u32(1) << u32(count)) - 1);
			record: p.Constraint_Type_Record = relative_probe_record(count, all, all);
			when dispatch == .Contextual
			{
				record.dispatch = .Contextual;
				record.contextual = &binding;
			}
			testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
			fixture: Joint_Fixture;
			relative_probe_populate(t, &scene, record, &fixture, origin);
			batch: ^p.Type_Batch = constraint_batch(t, &scene, fixture.joints[0]);
			for route in ([2]Execution_Route{.Registered_Bundle, .Registered_Lane})
			{
				for stage in ([3]p.Solver_Execution_Stage{.Warmstart, .Solve, .Incremental_Update})
				{
					relative_probe_dispatch(t, &scene, &record, fixture.joints[0], stage, route, dispatch);
					for group in 0 ..< 17
					{
						data: ^Relative_Probe_Impulses = (^Relative_Probe_Impulses)(p.type_batch_impulse_bundle(batch, group));
						lane: int = group % 8;
						testing.expect_value(t, simd.extract(data.marker, lane), f32(7) if dispatch == .Contextual else f32(0));
						for body in 0 ..< count
						{
							expected: util.Vector3 = {f32(body) / 512, -f32(body) / 256, f32(body) / 128};
							testing.expect_value(t, util.vector3_wide_read_slot(data.positions[body], lane), expected);
							if stage == .Solve
							{
								testing.expect_value(t, body_motion(t, &scene, fixture.bodies[group][body]).velocity.linear, expected);
							}
						}
					}
				}
			}
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_custom_native_relative_callbacks_full_tail_lane_and_all_phases :: proc(t: ^testing.T)
{
	exercise_relative_custom_routes(t, .Native);
}

@(test)
precision_custom_contextual_relative_callbacks_full_tail_lane_and_all_phases :: proc(t: ^testing.T)
{
	exercise_relative_custom_routes(t, .Contextual);
}

@(test)
precision_custom_relative_access_masks_are_phase_specific :: proc(t: ^testing.T)
{
	for count in 1 ..= 4
	{
		for mask in 0 ..< (1 << u32(count))
		{
			scene: Scene;
			scene_initialize(t, &scene);
			all: u8 = u8((u32(1) << u32(count)) - 1);
			record: p.Constraint_Type_Record = relative_probe_record(count, u8(mask), all & ~u8(mask));
			testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
			fixture: Joint_Fixture;
			relative_probe_populate(t, &scene, record, &fixture, 1e9, 9);
			batch: ^p.Type_Batch = constraint_batch(t, &scene, fixture.joints[0]);
			for route in ([2]Execution_Route{.Registered_Bundle, .Registered_Lane})
			{
				for stage in ([3]p.Solver_Execution_Stage{.Warmstart, .Solve, .Incremental_Update})
				{
					for bundle in 0 ..< 2
					{
						data: ^Relative_Probe_Impulses = (^Relative_Probe_Impulses)(p.type_batch_impulse_bundle(batch, bundle * 8));
						data^ = {};
					}
					relative_probe_dispatch(t, &scene, &record, fixture.joints[0], stage, route, .Native);
					selected: u8 = all & ~u8(mask) if stage == .Solve else u8(mask);
					for group in 0 ..< 9
					{
						data: ^Relative_Probe_Impulses = (^Relative_Probe_Impulses)(p.type_batch_impulse_bundle(batch, group));
						for body in 0 ..< count
						{
							expected: util.Vector3;
							if selected & (u8(1) << u32(body)) != 0
							{
								expected = {f32(body) / 512, -f32(body) / 256, f32(body) / 128};
							}
							testing.expect_value(t, util.vector3_wide_read_slot(data.positions[body], group % 8), expected);
						}
					}
				}
			}
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_custom_relative_range_failure_precedes_callback_and_scatter :: proc(t: ^testing.T)
{
	for mask in ([2]u8{0, 3})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		record: p.Constraint_Type_Record = relative_probe_record(2, mask, mask);
		testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
		fixture: Joint_Fixture;
		relative_probe_populate(t, &scene, record, &fixture, 0, 1);
		limit: f64 = f64(f32(math.F32_MAX));
		testing.expect_value(t, p.bodies_set_global_position(&scene.simulation.bodies, fixture.bodies[0][0], {-limit, 0, 0}), p.Physics_Status.Ok);
		testing.expect_value(t, p.bodies_set_global_position(&scene.simulation.bodies, fixture.bodies[0][1], {limit, 0, 0}), p.Physics_Status.Ok);
		batch: ^p.Type_Batch = constraint_batch(t, &scene, fixture.joints[0]);
		data: ^Relative_Probe_Impulses = (^Relative_Probe_Impulses)(p.type_batch_impulse_bundle(batch, 0));
		data.marker = util.F32x8(123);
		status: p.Physics_Status = p.solver_execute_type_bundle_dispatch_integration(&scene.simulation.solver,
			batch, &record, 0, 0, 1.0 / 64, 64, .First, .Solve, 0, .Native, .Never);
		testing.expect_value(t, status, p.Physics_Status.Ok if mask == 0 else p.Physics_Status.Invalid_Argument);
		testing.expect_value(t, simd.extract(data.marker, 0), f32(0) if mask == 0 else f32(123));
		for body in 0 ..< 2
		{
			testing.expect_value(t, body_motion(t, &scene, fixture.bodies[0][body]).velocity.linear, util.Vector3{});
		}
		scene_dispose(t, &scene);
	}
}
