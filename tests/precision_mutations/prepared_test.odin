package precision_mutations_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
precision_prepared_mutations_control_and_trigger_recovery_allocate_nothing :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 32, 4);
	position: e.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	kinematic: e.Body_Handle;
	kinematic, _ = e.body_add_global(&scene.world, e.body_kinematic({}, e.pose(), {}, e.body_activity(-1, 255)), position);
	testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
	testing.expect_value(t, e.world_enable_triggers(&scene.world, {pair_capacity=32, candidates_per_worker=64, child_capacity=128}), e.Status.Ok);
	reference: e.Collidable_Reference;
	reference, _ = p.collidable_reference_static(fixed);
	testing.expect_value(t, e.trigger_set(&scene.world, reference, {stay=.Enabled}), e.Status.Ok);
	allocations, bytes: i64;
	for iteration in 0 ..< 1028
	{
		if iteration == 4
		{
			allocations = scene.tracker.total_allocation_count;
			bytes = scene.tracker.current_memory_allocated;
		}
		updated: e.Global_Position = {position.x + f64(iteration % 2) * 0.001953125, position.y, position.z};
		testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {updated, {w=1}}), e.Status.Ok);
		testing.expect_value(t, e.body_set_velocity(&scene.world, body, {}), e.Status.Ok);
		testing.expect_value(t, e.body_set_shape(&scene.world, body, scene.box if iteration % 2 == 0 else scene.sphere), e.Status.Ok);
		testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {position, {w=1}}, .None), e.Status.Ok);
		testing.expect_value(t, e.static_set_continuity(&scene.world, fixed, {}, .None), e.Status.Ok);
		testing.expect_value(t, e.body_add_force_at_global_position(&scene.world, body, {1, 0, 0}, {updated.x, updated.y + 0.001953125, updated.z}), e.Status.Ok);
		testing.expect_value(t, e.body_clear_inputs(&scene.world, body), e.Status.Ok);
		testing.expect_value(t, e.body_set_global_kinematic_target(&scene.world, kinematic, {updated, {w=1}}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
		expect_body_position(t, &scene, body, updated);
		expect_body_position(t, &scene, kinematic, updated);
		// an output-capacity error must preserve the event for a later drain
		count, required: int;
		status: e.Status;
		count, required, status = e.trigger_events_drain(&scene.world, nil);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, count, 0);
		testing.expect_value(t, required, 1);
		expect_trigger_event(t, &scene, 1, .Enter if iteration == 0 else .Stay);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
	scene_dispose(t, &scene);
}
