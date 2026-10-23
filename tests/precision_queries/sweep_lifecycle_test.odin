package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_sweep_lifecycle_reads_preserve_authority_and_sleep :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	positions: [4]physics.Global_Position = {
		{1e9 + 0.125, 1e9 + 0.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 4.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 8.25, -1e9 - 0.375},
		{1e9 + 0.125, 1e9 + 12.25, -1e9 - 0.375},
	};
	bodies: [3]physics.Body_Handle;
	bodies[0] = add_body(t, &scene, shape, positions[0]);
	bodies[1] = add_body(t, &scene, shape, positions[1], .Kinematic);
	bodies[2] = add_body(t, &scene, shape, positions[2]);
	stationary: physics.Static_Handle = add_static(t, &scene, shape, positions[3]);
	references: [4]physics.Collidable_Reference;
	for body, index in bodies
	{
		references[index], _ = physics.collidable_reference_create(.Kinematic if index == 1 else .Dynamic, int(body.value));
	}
	references[3], _ = physics.collidable_reference_create(.Static, int(stationary.value));
	stack: [256]i32;
	hits: [128]entasis.Global_Sweep_Hit;
	before, after: [4]entasis.Global_Pose;
	frame: physics.Origin_Frame;
	velocity: entasis.Body_Velocity = {linear={-1, 0, 0}};
	for phase in 0 ..< 4
	{
		if phase == 1
		{
			sleep_body(t, &scene, bodies[0]);
			sleep_body(t, &scene, bodies[2]);
		}
		if phase == 2
		{
			testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, bodies[0]), physics.Physics_Status.Ok);
			testing.expect_value(t, entasis.world_shift_origin(&scene.world, &frame, {1e9, 1e9, -1e9}), entasis.Status.Ok);
		}
		if phase == 3
		{
			testing.expect_value(t, entasis.body_remove(&scene.world, bodies[0]), entasis.Status.Ok);
			replacement: physics.Body_Handle = add_body(t, &scene, shape, {0.125, 0.25, -0.375});
			testing.expect_value(t, replacement, bodies[0]);
			for index in 0 ..< 64
			{
				_ = add_body(t, &scene, shape, {128 + f64(index)*4, 0, 0});
			}
		}
		count, status := entasis.world_export_global_poses(&scene.world, references[:], before[:], frame);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 4);
		sleep_before, sleep_after: physics.Body_Memory_Location;
		sleep_before, status = physics.bodies_resolve(&scene.simulation.bodies, bodies[2]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		for position, index in positions
		{
			expect_position(t, before[index].position, position, 0);
			local: physics.Global_Position = {position.x - frame.origin.x, position.y - frame.origin.y, position.z - frame.origin.z};
			pose: entasis.Global_Pose = {position={local.x + 2.998, local.y, local.z}, orientation=util.quaternion_identity()};
			count, status = entasis.sweep_global_all_with_context(&scene.query, shape, pose, velocity, 2, hits[:], stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, count, 1);
			testing.expect_value(t, hits[0].collidable, references[index]);
			expect_position(t, hits[0].sweep.location, {local.x + 0.5, local.y, local.z}, 2e-5);
			testing.expect(t, abs(hits[0].sweep.t1 - 1.998) < 2e-5);
			result: entasis.Query_Hit_State;
			result, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 2, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, result, entasis.Query_Hit_State.Hit);
			pose.position.x = local.x + 3.002;
			result, status = entasis.sweep_global_any_with_context(&scene.query, shape, pose, velocity, 2, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, result, entasis.Query_Hit_State.Miss);
		}
		count, status = entasis.world_export_global_poses(&scene.world, references[:], after[:], frame);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 4);
		testing.expect_value(t, before, after);
		sleep_after, status = physics.bodies_resolve(&scene.simulation.bodies, bodies[2]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, sleep_before, sleep_after);
		if phase >= 1
		{
			testing.expect(t, sleep_after.set_index > 0);
		}
	}
}
