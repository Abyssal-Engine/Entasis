package precision_queries_tests

import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
global_overlap_volume_lifecycle_reads_preserve_authority_and_sleep :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	shape: physics.Typed_Index = add_shape(t, &scene, .Box);
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
	hits: [128]entasis.Overlap_Hit;
	volumes: [128]entasis.Volume_Hit;
	before, after: [4]entasis.Global_Pose;
	frame: physics.Origin_Frame;
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
				_ = add_body(t, &scene, shape, {128 + f64(index) * 4, 0, 0});
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
			pose: entasis.Global_Pose = {position=local, orientation=util.quaternion_identity()};
			pose.position.x += 0.998;
			count, status = entasis.overlap_global_all_with_context(&scene.query, shape, pose, hits[:], stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, count, 1);
			testing.expect_value(t, hits[0].collidable, references[index]);
			result: entasis.Overlap_State;
			result, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, result, entasis.Overlap_State.Intersecting);
			pose.position.x = local.x + 1.002;
			result, status = entasis.overlap_global_any_with_context(&scene.query, shape, pose, stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			testing.expect_value(t, result, entasis.Overlap_State.Separated);
			count, status = entasis.volume_global_all_with_context(&scene.query, {min=local, max=local}, volumes[:], stack[:]);
			testing.expect_value(t, status, entasis.Status.Ok);
			matches: int;
			for hit in volumes[:count]
			{
				if hit.collidable == references[index]
				{
					matches += 1;
				}
			}
			testing.expect_value(t, matches, 1);
		}
		count, status = entasis.world_export_global_poses(&scene.world, references[:], after[:], frame);
		testing.expect_value(t, status, entasis.Status.Ok);
		testing.expect_value(t, count, 4);
		testing.expect_value(t, after, before);
		sleep_after, status = physics.bodies_resolve(&scene.simulation.bodies, bodies[2]);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		testing.expect_value(t, sleep_after, sleep_before);
		if phase > 0
		{
			testing.expect(t, sleep_after.set_index > 0);
		}
	}
}

@(test)
global_overlap_any_missing_child_route_and_recovery :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	state: Overlap_Callback_State = {query=&scene.query};
	type_id, status := entasis.custom_shape_register(&scene.world, entasis.custom_shape_registration(
		Overlap_Custom_Sphere, .Convex, overlap_custom_bounds, overlap_custom_inertia, overlap_custom_ray, overlap_custom_support));
	testing.expect_value(t, status, entasis.Status.Ok);
	value: Overlap_Custom_Sphere = {radius=0.5, state=&state};
	custom: physics.Typed_Index;
	custom, status = entasis.custom_shape_add(&scene.world, type_id, &value);
	testing.expect_value(t, status, entasis.Status.Ok);
	children: [1]entasis.Compound_Child = {entasis.compound_child(custom, entasis.pose())};
	compound: entasis.Shape_Handle;
	compound, status = entasis.shape_import_compound(&scene.world, children[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	target: physics.Static_Handle = add_static(t, &scene, compound, {1e9, 0, 0});
	sphere: physics.Typed_Index = add_shape(t, &scene, .Sphere);
	stack: [128]i32;
	pose: entasis.Global_Pose = {position={1e9, 0, 0}, orientation=util.quaternion_identity()};
	result: entasis.Overlap_State;
	result, status = entasis.overlap_global_any_with_context(&scene.query, sphere, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, result, entasis.Overlap_State.Separated);
	// reverse the compound role to exercise original child ordering through
	// the existing flipped parent task before a child route is installed
	testing.expect_value(t, entasis.static_remove(&scene.world, target, .None), entasis.Status.Ok);
	_ = add_static(t, &scene, sphere, {1e9, 0, 0});
	result, status = entasis.overlap_global_any_with_context(&scene.query, compound, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Not_Found);
	testing.expect_value(t, result, entasis.Overlap_State.Separated);
	_, status = entasis.collision_task_register(&scene.world, entasis.collision_task_convex(
		type_id, entasis.SHAPE_TYPE_SPHERE, 16, .Sphere, overlap_custom_pair, overlap_custom_pair_wide));
	testing.expect_value(t, status, entasis.Status.Ok);
	result, status = entasis.overlap_global_any_with_context(&scene.query, compound, pose, stack[:]);
	testing.expect_value(t, status, entasis.Status.Ok);
	testing.expect_value(t, result, entasis.Overlap_State.Intersecting);
}
