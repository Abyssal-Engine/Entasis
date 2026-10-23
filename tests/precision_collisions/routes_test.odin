package precision_collisions_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

current_route :: proc "contextless" (scene: ^Scene, workers: int) -> physics.Narrow_Phase_Collision_Route
{
	route: physics.Narrow_Phase_Collision_Route = physics.narrow_phase_collision_route_state(&scene.simulation.narrow_phase, workers);
	if route == .Predecessor
	{
		route = physics.narrow_phase_box_sphere_route_state(&scene.simulation.narrow_phase, workers);
	}
	return route;
}

@(test)
precision_direct_routes_match_materialized_for_every_simd_tail :: proc(t: ^testing.T)
{
	kinds: [3]Shape_Kind = {.Box, .Hull, .Sphere};
	for kind in kinds
	{
		for count in 1 ..= 17
		{
			scenes: [3]Scene;
			for variant in 0 ..< 3
			{
				scene: ^Scene = &scenes[variant];
				completion: Completion = .Stored;
				if variant == 2
				{
					completion = .Materialized;
				}
				scene_initialize(t, scene, completion);
				shape_a: physics.Typed_Index = add_shape(t, scene, kind);
				shape_b: physics.Typed_Index = shape_a;
				if kind == .Sphere
				{
					shape_b = add_shape(t, scene, .Box);
				}
				base: f64 = 0;
				if variant != 0
				{
					base = 1e9;
				}
				for pair in 0 ..< count
				{
					a: physics.Typed_Index = shape_a;
					b: physics.Typed_Index = shape_b;
					if kind == .Sphere
					{
						switch pair % 4
						{
						case 0:
							b = shape_a;
						case 1:
							a = shape_b;
						case 2:
						case 3:
							a, b = b, a;
						}
					}
					x: f64 = base + f64(pair) * 128 + 0.03125;
					body: physics.Body_Handle = add_body(t, scene, a, {x, -base + 0.9375, base + 0.125});
					body_motion(t, scene, body).pose.orientation = util.quaternion_from_axis_angle({0, 1, 0}, f32(pair % 3) * 0.0625);
					if pair % 3 == 0
					{
						_ = add_body(t, scene, b, {x, -base, base + 0.125}, .Kinematic);
					}
					else
					{
						_ = add_static(t, scene, b, {x, -base, base + 0.125});
					}
				}
				testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
				expected: physics.Narrow_Phase_Collision_Route = .Box_Direct;
				if kind == .Hull
				{
					expected = .Hull_Direct;
				}
				else if kind == .Sphere
				{
					expected = .Box_Sphere_Direct;
				}
				if variant == 2
				{
					expected = .Predecessor;
				}
				testing.expect_value(t, physics.narrow_phase_prepare_batchers(&scene.simulation.narrow_phase, 1), physics.Physics_Status.Ok);
				testing.expect_value(t, current_route(scene, 1), expected);
				testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase), physics.Physics_Status.Ok);
				testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.count, count);
			}
			expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
			expect_contact_sets_equal(t, &scenes[0], &scenes[2]);
			for variant in 0 ..< 3
			{
				scene_dispose(t, &scenes[variant]);
			}
		}
	}
}

@(test)
precision_coarse_candidates_parallel_contacts_and_prepared_allocations :: proc(t: ^testing.T)
{
	scenes: [2]Scene;
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	for variant in 0 ..< 2
	{
		scene: ^Scene = &scenes[variant];
		scene_initialize(t, scene, .Stored, 256);
		// reserve for conservative active/static overlap candidates before measuring
		capacity: physics.Simulation_Allocation_Sizes = scene.simulation.allocation_sizes;
		capacity.broad_phase_candidates = 32768;
		testing.expect_value(t, physics.simulation_ensure_capacity(scene.simulation, capacity), physics.Physics_Status.Ok);
		shape: physics.Typed_Index = add_shape(t, scene, .Box);
		for pair in 0 ..< 97
		{
			x: f64 = 1e9 + f64(pair % 10) * 4 + 0.03125;
			z: f64 = -1e9 + f64(pair / 10) * 4 + 0.125;
			_ = add_body(t, scene, shape, {x, 1e9 + 0.9375, z});
			_ = add_static(t, scene, shape, {x, 1e9, z});
		}
		selected: ^util.Thread_Dispatcher_Boundary;
		if variant == 1
		{
			selected = boundary;
		}
		testing.expect_value(t, physics.narrow_phase_execute(&scene.simulation.narrow_phase, selected), physics.Physics_Status.Ok);
		testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.count, 97);
	}
	expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
	allocations: i64 = scenes[0].tracker.total_allocation_count;
	bytes: i64 = scenes[0].tracker.current_memory_allocated;
	for _ in 0 ..< 128
	{
		testing.expect_value(t, physics.narrow_phase_execute(&scenes[0].simulation.narrow_phase), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.narrow_phase_execute(&scenes[1].simulation.narrow_phase, boundary), physics.Physics_Status.Ok);
		expect_contact_sets_equal(t, &scenes[0], &scenes[1]);
	}
	testing.expect_value(t, scenes[0].tracker.total_allocation_count, allocations);
	testing.expect_value(t, scenes[0].tracker.current_memory_allocated, bytes);
	scene_dispose(t, &scenes[1]);
	scene_dispose(t, &scenes[0]);
}
