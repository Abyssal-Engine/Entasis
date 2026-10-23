package precision_callbacks_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_callback_solver_continuation_masks_use_advanced_authority :: proc(t: ^testing.T)
{
	for mode in e.Angular_Integration_Mode
	{
		capture: Capture;
		callbacks: e.Pose_Callbacks = capture_callbacks(&capture);
		callbacks.angular_mode = mode;
		scene: Scene;
		scene_initialize(t, &scene, callbacks=callbacks);
		bodies: [8]e.Body_Handle;
		for lane in 0 ..< 8
		{
			bodies[lane] = add_body(t, &scene, {1e9 + f64(lane) / 512, -1e9, 1e9}, {});
		}
		for bits in 0 ..< 256
		{
			encoded, masks: [8]i32;
			capture.counts = {};
			for lane in 0 ..< 8
			{
				testing.expect_value(t, p.bodies_set_global_position(&scene.simulation.bodies, bodies[lane],
					{1e9 + f64(lane) / 512, -1e9, 1e9}), e.Status.Ok);
				testing.expect_value(t, e.body_set_velocity(&scene.world, bodies[lane], {linear={0.125, -0.25, 0.5}}), e.Status.Ok);
				encoded[lane] = -1;
				if bits & (1 << uint(lane)) != 0
				{
					encoded[lane] = i32(lane);
					masks[lane] = -1;
				}
			}
			indices: util.I32x8 = transmute(util.I32x8)encoded;
			position: util.Vector3_Wide;
			orientation: util.Quaternion_Wide;
			velocity: p.Body_Velocity_Wide;
			inertia: p.Body_Inertia_Wide;
			position, orientation, velocity, inertia = p.bodies_gather_active_trusted(&scene.simulation.bodies, indices, .Local, p.BODY_ACCESS_ALL);
			_ = p.solver_integrate_constraint_body(&scene.simulation.solver, indices, transmute(util.I32x8)masks,
				0.125, .First, 0, &position, &orientation, &velocity, inertia);
			_ = p.solver_integrate_constraint_body(&scene.simulation.solver, indices, transmute(util.I32x8)masks,
				0.125, .Continuation, 0, &position, &orientation, &velocity, inertia);
			for lane in 0 ..< 8
			{
				if masks[lane] == 0
				{
					testing.expect_value(t, capture.counts[bodies[lane].value], 0);
					continue;
				}
				handle: i32 = bodies[lane].value;
				testing.expect_value(t, capture.counts[handle], 2);
				testing.expect_value(t, capture.positions[handle][0], e.Global_Position{1e9 + f64(lane) / 512, -1e9, 1e9});
				testing.expect_value(t, capture.positions[handle][1],
					e.Global_Position{1e9 + f64(lane) / 512 + 0.015625, -1e9 - 0.03125, 1e9 + 0.0625});
			}
		}
		scene_dispose(t, &scene);
	}
}

@(test)
global_callback_constrained_fallback_and_workers_keep_substep_instants :: proc(t: ^testing.T)
{
	for mode in e.Angular_Integration_Mode
	{
		for workers in ([2]i32{1, 4})
		{
			capture: Capture;
			callbacks: e.Pose_Callbacks = capture_callbacks(&capture);
			callbacks.angular_mode = mode;
			scene: Scene;
			scene_initialize(t, &scene, capacity=32, substeps=4, workers=workers, callbacks=callbacks, threshold=1);
			bodies: [17]e.Body_Handle;
			for index in 0 ..< 17
			{
				bodies[index] = add_body(t, &scene, {1e9 + f64(index) + 0.001953125, 0, 0}, {});
				testing.expect_value(t, e.body_set_velocity(&scene.world, bodies[index], {linear={0.125, 0, 0}}), e.Status.Ok);
				for constraint in 0 ..< 2
				{
					_ = constraint;
					status: e.Status;
					_, status = e.constraint_add_1(&scene.world, bodies[index],
						e.One_Body_Linear_Motor{target_velocity={0.125, 0, 0}, settings={maximum_force=0, damping=1}});
					testing.expect_value(t, status, e.Status.Ok);
				}
			}
			testing.expect(t, scene.simulation.solver.active_set.batches.memory[1].constraint_count > 0);
			testing.expect_value(t, e.world_step(&scene.world, 0.25), e.Status.Ok);
			for index in 0 ..< 17
			{
				handle: i32 = bodies[index].value;
				testing.expect_value(t, capture.counts[handle], 5);
				for visit in 0 ..< 5
				{
					testing.expect_value(t, capture.positions[handle][visit],
						e.Global_Position{1e9 + f64(index) + 0.001953125 + f64(max(0, visit - 1)) * 0.0078125, 0, 0});
				}
				expect_body_position(t, &scene, bodies[index], {1e9 + f64(index) + 0.001953125 + 0.03125, 0, 0});
			}
			scene_dispose(t, &scene);
		}
	}
}

@(test)
global_callback_kinematic_continuation_observes_new_position :: proc(t: ^testing.T)
{
	capture: Capture;
	callbacks: e.Pose_Callbacks = capture_callbacks(&capture);
	callbacks.integrate_kinematic_velocity = .Enabled;
	scene: Scene;
	scene_initialize(t, &scene, callbacks=callbacks);
	defer scene_dispose(t, &scene);
	bodies: [8]e.Body_Handle;
	for lane in 0 ..< 8
	{
		status: e.Status;
		bodies[lane], status = e.body_add_global(&scene.world,
			e.body_kinematic({}, e.pose(), {linear={0.125, 0, 0}}, e.body_activity(-1, 255)),
			{1e9 + f64(lane) / 512, 0, 0});
		testing.expect_value(t, status, e.Status.Ok);
	}
	for bits in 0 ..< 256
	{
		encoded, masks: [8]i32;
		capture.counts = {};
		for lane in 0 ..< 8
		{
			testing.expect_value(t, p.bodies_set_global_position(&scene.simulation.bodies, bodies[lane], {1e9 + f64(lane) / 512, 0, 0}), e.Status.Ok);
			encoded[lane] = -1;
			if bits & (1 << uint(lane)) != 0
			{
				encoded[lane] = i32(u32(lane) | p.BODY_REFERENCE_KINEMATIC_MASK);
				masks[lane] = -1;
			}
		}
		p.solver_integrate_kinematic_lanes(&scene.simulation.solver,
			transmute(util.I32x8)encoded, transmute(util.I32x8)masks, 0.125, .Continuation, 0);
		for lane in 0 ..< 8
		{
			if masks[lane] == 0
			{
				testing.expect_value(t, capture.counts[bodies[lane].value], 0);
				continue;
			}
			testing.expect_value(t, capture.last[bodies[lane].value], e.Global_Position{1e9 + f64(lane) / 512 + 0.015625, 0, 0});
		}
	}
}
