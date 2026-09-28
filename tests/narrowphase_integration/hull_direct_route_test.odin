package narrowphase_integration_tests

import "core:testing"
import "core:mem"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
hull_direct_route_preserves_contacts_and_custom_task_guards :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	test_pool_prepare(t, &pool);
	defer util.buffer_pool_dispose(&pool);
	simulation := new(physics.Simulation);
	defer mem.free(simulation);
	description := test_simulation_description(&pool, physics.narrow_phase_default_callbacks());
	description.use_default_narrow = .Present;
	created := physics.simulation_create(simulation, &description);
	testing.expect_value(t, created.status, physics.Physics_Status.Ok);
	if created.status != .Ok
	{
		return;
	}
	defer physics.simulation_destroy(simulation);
	narrow := &simulation.narrow_phase;
	testing.expect_value(t, physics.narrow_phase_bind_default_stored_completion(narrow, {
		friction_coefficient=1,
		spring_settings=physics.spring_settings_create(30, 1),
		maximum_recovery_velocity=2,
	}), physics.Physics_Status.Ok);
	points := [8]util.Vector3{
		{-0.4, -0.5, -0.5}, {0.5, -0.5, -0.5}, {-0.5, 0.45, -0.5}, {0.5, 0.5, -0.5},
		{-0.5, -0.5, 0.5}, {0.5, -0.5, 0.4}, {-0.5, 0.5, 0.5}, {0.5, 0.5, 0.5},
	};
	hull: physics.Convex_Hull;
	center: util.Vector3;
	testing.expect_value(t, physics.convex_hull_create_from_point_cloud(&hull, &center, &points[0], 8, &pool), physics.Physics_Status.Ok);
	shape, shape_status := physics.shape_registry_add(&simulation.shapes, physics.CONVEX_HULL_TYPE_ID, &hull);
	testing.expect_value(t, shape_status, physics.Physics_Status.Ok);
	body := test_body_description(shape, {0, 0.9, 0});
	body.activity.sleep_threshold = -1;
	_, body_status := physics.simulation_add_body(simulation, &body);
	testing.expect_value(t, body_status, physics.Physics_Status.Ok);
	_, static_status := physics.simulation_add_static(simulation, &physics.Static_Description{
		pose={orientation=util.quaternion_identity()}, shape=shape,
	});
	testing.expect_value(t, static_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Hull_Direct);
	testing.expect_value(t, narrow.pair_cache.mapping.count, 1);
	for _ in 0 ..< 8
	{
		testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	}
	route_index := physics.collision_task_matrix_index(physics.CONVEX_HULL_TYPE_ID, physics.CONVEX_HULL_TYPE_ID);
	task_id := simulation.collision_tasks.routes[route_index].task_id;
	task := &simulation.collision_tasks.tasks[task_id];
	original_into := task.convex_wide_test_into;
	task.convex_wide_test_into = nil;
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	task.convex_wide_test_into = original_into;
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Hull_Direct);
	narrow.stored_completion_state = .Missing;
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	narrow.stored_completion_state = .Present;
	box := physics.Box{0.5, 0.5, 0.5};
	box_shape, box_status := physics.shape_registry_add(&simulation.shapes, physics.BOX_TYPE_ID, &box);
	testing.expect_value(t, box_status, physics.Physics_Status.Ok);
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Predecessor);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.shape_registry_remove(&simulation.shapes, box_shape), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.narrow_phase_collision_route_state(narrow, 1), physics.Narrow_Phase_Collision_Route.Hull_Direct);
	testing.expect_value(t, physics.simulation_timestep(simulation, 1.0 / 60.0), physics.Physics_Status.Ok);
}
