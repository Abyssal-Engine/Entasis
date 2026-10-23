package precision_mutations_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
precision_property_mutation_survives_growth_compaction_rebase_and_handle_reuse :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 4);
	bodies: [65]e.Body_Handle;
	statics: [65]e.Static_Handle;
	positions: [65]e.Global_Position;
	for index in 0 ..< len(bodies)
	{
		positions[index] = {1e9 + f64(index) * 4 + 0.001953125, -1e9 + 0.125, 1e9};
		bodies[index] = add_body(t, &scene, positions[index], scene.sphere);
		statics[index] = add_static(t, &scene, positions[index]);
	}
	// removing the first dense slot compacts a different handle into it
	testing.expect_value(t, e.body_remove(&scene.world, bodies[0]), e.Status.Ok);
	testing.expect_value(t, e.static_remove(&scene.world, statics[0], .None), e.Status.Ok);
	for index in 1 ..< len(bodies)
	{
		if index % 2 == 0
		{
			sleep_body(t, &scene, bodies[index]);
		}
		testing.expect_value(t, e.body_set_activity(&scene.world, bodies[index], e.body_activity(-1, 255)), e.Status.Ok);
		testing.expect_value(t, e.static_set_shape(&scene.world, statics[index], scene.box, .None), e.Status.Ok);
		expect_body_position(t, &scene, bodies[index], positions[index]);
		expect_static_position(t, &scene, statics[index], positions[index]);
	}
	reused_position: e.Global_Position = {1e9 + 1024.001953125, -1e9 + 0.375, 1e9};
	reused_body: e.Body_Handle = add_body(t, &scene, reused_position, scene.box);
	reused_static: e.Static_Handle = add_static(t, &scene, reused_position);
	testing.expect_value(t, reused_body, bodies[0]);
	testing.expect_value(t, reused_static, statics[0]);
	testing.expect_value(t, p.simulation_translate_origin(scene.simulation, {1e9, -1e9, 1e9}), p.Physics_Status.Ok);
	for index in 1 ..< len(bodies)
	{
		positions[index] = {positions[index].x - 1e9, positions[index].y + 1e9, positions[index].z - 1e9};
		testing.expect_value(t, e.body_set_inertia(&scene.world, bodies[index], body_description(scene.box).local_inertia), e.Status.Ok);
		testing.expect_value(t, e.static_set_continuity(&scene.world, statics[index], {mode=.Passive}, .None), e.Status.Ok);
		expect_body_position(t, &scene, bodies[index], positions[index]);
		expect_static_position(t, &scene, statics[index], positions[index]);
	}
	reused_position = {reused_position.x - 1e9, reused_position.y + 1e9, reused_position.z - 1e9};
	testing.expect_value(t, e.body_set_shape(&scene.world, reused_body, scene.sphere), e.Status.Ok);
	testing.expect_value(t, e.static_set_shape(&scene.world, reused_static, scene.box, .None), e.Status.Ok);
	expect_body_position(t, &scene, reused_body, reused_position);
	expect_static_position(t, &scene, reused_static, reused_position);
	scene_dispose(t, &scene);
}

@(test)
precision_mutation_lifetime_and_busy_world_rejection :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	scene.simulation.state = .Stepping;
	testing.expect(t, e.body_set_global_pose(&scene.world, body, {{}, {w=1}}) != .Ok);
	testing.expect(t, e.static_set_global_pose(&scene.world, fixed, {{}, {w=1}}) != .Ok);
	testing.expect(t, e.body_set_velocity(&scene.world, body, {}) != .Ok);
	testing.expect(t, e.static_set_continuity(&scene.world, fixed, {}, .None) != .Ok);
	testing.expect(t, e.body_add_force_at_global_position(&scene.world, body, {1, 0, 0}, position) != .Ok);
	scene.simulation.state = .Ready;
	expect_body_position(t, &scene, body, position);
	expect_static_position(t, &scene, fixed, position);
	testing.expect_value(t, e.body_remove(&scene.world, body), e.Status.Ok);
	testing.expect_value(t, e.static_remove(&scene.world, fixed, .None), e.Status.Ok);
	testing.expect(t, e.body_set_global_pose(&scene.world, body, {position, {w=1}}) != .Ok);
	testing.expect(t, e.static_set_global_pose(&scene.world, fixed, {position, {w=1}}) != .Ok);
	scene_dispose(t, &scene);
	testing.expect_value(t, e.body_apply_global(&scene.world, body, body_description({}), position), e.Status.Disposed);
	testing.expect_value(t, e.static_apply_global(&scene.world, fixed, e.static_body({}, e.pose()), position), e.Status.Disposed);
	_, status := e.body_get_global_kinematic_target(&scene.world, body);
	testing.expect_value(t, status, e.Status.Disposed);
}
