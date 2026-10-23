package precision_mutations_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

expect_trigger_event :: proc(t: ^testing.T, scene: ^Scene, count: int, kind: e.Trigger_Event_Kind)
{
	events: [8]e.Trigger_Event;
	n, required: int;
	status: e.Status;
	n, required, status = e.trigger_events_drain(&scene.world, events[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, n, count);
	testing.expect_value(t, required, count);
	if count > 0
	{
		testing.expect_value(t, events[0].kind, kind);
	}
}

@(test)
precision_trigger_two_millimeter_entry_exit_and_property_edits :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for workers in ([2]i32{1, 4})
		{
			scene: Scene;
			scene_initialize(t, &scene, 32, 1, workers);
			center: e.Global_Position = {origin + 0.125, -origin, origin + 0.25};
			body: e.Body_Handle = add_body(t, &scene, {center.x + 1.001953125, center.y, center.z}, scene.sphere);
			fixed: e.Static_Handle = add_static(t, &scene, center);
			reference: e.Collidable_Reference;
			reference, _ = p.collidable_reference_static(fixed);
			testing.expect_value(t, e.world_enable_triggers(&scene.world, {pair_capacity=32, candidates_per_worker=64, child_capacity=128}), e.Status.Ok);
			testing.expect_value(t, e.trigger_set(&scene.world, reference, {stay=.Enabled}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
			expect_trigger_event(t, &scene, 0, .Enter);
			overlap: e.Global_Position = {center.x + 0.998046875, center.y, center.z};
			testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {overlap, {w=1}}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
			expect_trigger_event(t, &scene, 1, .Enter);
			testing.expect_value(t, e.body_set_activity(&scene.world, body, e.body_activity(-1, 255)), e.Status.Ok);
			testing.expect_value(t, e.static_set_continuity(&scene.world, fixed, {mode=.Passive}, .None), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
			expect_trigger_event(t, &scene, 1, .Stay);
			expect_body_position(t, &scene, body, overlap);
			expect_static_position(t, &scene, fixed, center);
			overlap.x += 0.00390625;
			testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {overlap, {w=1}}), e.Status.Ok);
			testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
			expect_trigger_event(t, &scene, 1, .Exit);
			testing.expect_value(t, scene.simulation.narrow_phase.pair_cache.mapping.count, 0);
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_trigger_dirty_discovery_uses_high_sleeping_and_static_bounds :: proc(t: ^testing.T)
{
	for static_pair in 0 ..< 2
	{
		scene: Scene;
		scene_initialize(t, &scene, 32, 1, 4);
		// halfway between coarse f32 centers: discovery from the rounded
		// point alone can miss the actual overlap or include a false one
		center: e.Global_Position = {1e9 + 31.875, -1e9 + 31.875, 1e9};
		fixed: e.Static_Handle = add_static(t, &scene, center);
		reference: e.Collidable_Reference;
		reference, _ = p.collidable_reference_static(fixed);
		other: e.Global_Position = {center.x + 0.998046875, center.y, center.z};
		body: e.Body_Handle;
		if static_pair == 0
		{
			body = add_body(t, &scene, other, scene.sphere);
			sleep_body(t, &scene, body);
		}
		else
		{
			add_static(t, &scene, other);
		}
		testing.expect_value(t, e.world_enable_triggers(&scene.world, {pair_capacity=32, candidates_per_worker=64, child_capacity=128}), e.Status.Ok);
		testing.expect_value(t, e.trigger_set(&scene.world, reference, {static_static=.Enabled}), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
		expect_trigger_event(t, &scene, 1, .Enter);
		if static_pair == 0
		{
			testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
			expect_body_position(t, &scene, body, other);
		}
		center.x -= 0.00390625;
		testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {center, {w=1}}, .None), e.Status.Ok);
		testing.expect_value(t, e.world_step(&scene.world, 0.125), e.Status.Ok);
		expect_trigger_event(t, &scene, 1, .Exit);
		scene_dispose(t, &scene);
	}
}
