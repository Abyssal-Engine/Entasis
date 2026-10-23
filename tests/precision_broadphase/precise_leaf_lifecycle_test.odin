package precision_broadphase_tests

import "core:testing"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

@(test)
precision_leaf_columns_follow_resize_migrations_removal_and_handle_reuse :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 32, 4);
	defer scene_dispose(t, &scene);
	dispatcher: u.Thread_Dispatcher;
	testing.expect_value(t, u.thread_dispatcher_initialize(&dispatcher, 4, 65536), u.Threading_Status.Ok);
	defer u.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^u.Thread_Dispatcher_Boundary = u.thread_dispatcher_boundary(&dispatcher);
	bodies: [19]p.Body_Handle;
	statics: [19]p.Static_Handle;
	body_before: [19]p.Global_Bounds;
	static_before: [19]p.Global_Bounds;
	for index in 0 ..< len(bodies)
	{
		position: p.Global_Position = {1e9 + f64(index) * 4 + 1.0/512, -1e9, 1e9};
		bodies[index] = add_body(t, &scene, scene.shapes[index % 10], position);
		statics[index] = add_static(t, &scene, scene.shapes[index % 10], position);
		body_before[index] = body_precise_bounds(t, &scene, bodies[index]);
		static_before[index] = static_precise_bounds(t, &scene, statics[index]);
	}
	testing.expect_value(t, pair_count(t, &scene), 19);
	for index in 0 ..< 9
	{
		sleep_body(t, &scene, bodies[index], boundary);
		testing.expect_value(t, body_precise_bounds(t, &scene, bodies[index]), body_before[index]);
	}
	testing.expect_value(t, pair_count(t, &scene), 10);
	testing.expect_value(t, p.broad_phase_ensure_capacity(&scene.simulation.broad_phase, 512, 512, 4096), p.Physics_Status.Ok);
	testing.expect_value(t, p.broad_phase_resize(&scene.simulation.broad_phase, 32, 32, 4096), p.Physics_Status.Ok);
	for index in 0 ..< 9
	{
		testing.expect_value(t, p.island_awakener_awaken_body(&scene.simulation.awakener, bodies[index], boundary), p.Physics_Status.Ok);
	}
	for index in 0 ..< len(bodies)
	{
		testing.expect_value(t, body_precise_bounds(t, &scene, bodies[index]), body_before[index]);
		testing.expect_value(t, static_precise_bounds(t, &scene, statics[index]), static_before[index]);
	}
	testing.expect_value(t, pair_count(t, &scene), 19);
	testing.expect_value(t, p.simulation_remove_body(scene.simulation, bodies[4]), p.Physics_Status.Ok);
	testing.expect_value(t, p.simulation_remove_static_without_awakening_bodies(scene.simulation, statics[7]), p.Physics_Status.Ok);
	for index in 0 ..< len(bodies)
	{
		if index != 4
		{
			testing.expect_value(t, body_precise_bounds(t, &scene, bodies[index]), body_before[index]);
		}
		if index != 7
		{
			testing.expect_value(t, static_precise_bounds(t, &scene, statics[index]), static_before[index]);
		}
	}
	testing.expect_value(t, pair_count(t, &scene), 17);
	reused: p.Body_Handle = add_body(t, &scene, scene.shapes[0], {-1e9, 1e9, -1e9});
	testing.expect_value(t, reused, bodies[4]);
	expect_precise_exact(t, body_precise_bounds(t, &scene, reused), {-1e9, 1e9, -1e9},
		{min={-0.0625, -0.0625, -0.0625}, max={0.0625, 0.0625, 0.0625}});
	testing.expect_value(t, pair_count(t, &scene), 17);
}
