package precision_mutations_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
precision_filtered_static_growth_compaction_rebase_and_prepared_allocation :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 8);
	defer scene_dispose(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	sleep_body(t, &scene, body);
	spacer: e.Static_Handle = add_static(t, &scene, {0, 200, 0});
	fixed: e.Static_Handle = add_static(t, &scene, position);
	for index in 0 ..< 65
	{
		add_static(t, &scene, {f64(index) * 4, 200, 0});
	}
	testing.expect_value(t, e.static_remove(&scene.world, spacer, .None), e.Status.Ok);
	filter: High_Static_Filter = {world=&scene.world, accepted=e.body_handle_invalid(), fixed=fixed};
	frame: e.Origin_Frame;
	testing.expect_value(t, e.world_shift_origin(&scene.world, &frame, {512, -256, 128}), e.Status.Ok);
	position = {position.x - 512, position.y + 256, position.z - 128};
	for _ in 0 ..< 4
	{
		testing.expect_value(t, e.static_apply_global_filtered(&scene.world, fixed, e.static_body(scene.sphere, e.pose()), position, high_static_filter, &filter), e.Status.Ok);
	}
	calls: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for iteration in 0 ..< 1024
	{
		next: e.Global_Position = {position.x + f64(iteration % 2) * 0.001953125, position.y, position.z};
		testing.expect_value(t, e.static_apply_global_filtered(&scene.world, fixed, e.static_body(scene.sphere, e.pose()), next, high_static_filter, &filter), e.Status.Ok);
		expect_static_position(t, &scene, fixed, next);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, calls);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
	testing.expect(t, filter.calls >= 1024);
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
	testing.expect_value(t, e.static_remove_filtered(&scene.world, fixed, high_static_filter, &filter), e.Status.Ok);
	filter.fixed = e.static_handle_invalid();
	reused: e.Static_Handle;
	status: e.Status;
	reused, status = e.static_add_global_filtered(&scene.world, e.static_body(scene.box, e.pose()), position, high_static_filter, &filter);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, reused, fixed);
	expect_static_position(t, &scene, reused, position);
}

@(test)
precision_filtered_static_allocation_failure_does_not_publish_replacement :: proc(t: ^testing.T)
{
	failures, successes: int;
	for operation in 0 ..< 2
	{
		for fail_after in 1 ..= 32
		{
			scene: Scene;
			allocator: Fallible_Mutation_Allocation = {tracker=&scene.tracker};
			scene_initialize(t, &scene, 16, 1, 1, {procedure=fallible_mutation_allocate, data=&allocator}, 128);
			position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9};
			body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
			sleep_body(t, &scene, body);
			fixed: e.Static_Handle = add_static(t, &scene, position);
			capacity: int = int(scene.simulation.bodies.sets.memory[0].dynamics_state.length);
			for index in 0 ..< capacity
			{
				add_body(t, &scene, {f64(index) * 4, 0, 0}, scene.sphere);
			}
			filter: High_Static_Filter = {world=&scene.world, accepted=body, fixed=fixed};
			allocator.reject_at = allocator.requests + fail_after;
			next: e.Global_Position = {position.x + 0.001953125, position.y, position.z};
			status: e.Status;
			added: e.Static_Handle;
			if operation == 0
			{
				added, status = e.static_add_global_filtered(&scene.world, e.static_body(scene.box, e.pose()), next, high_static_filter, &filter);
			}
			else
			{
				status = e.static_apply_global_filtered(&scene.world, fixed, e.static_body(scene.box, e.pose()), next, high_static_filter, &filter);
			}
			allocator.reject_at = 0;
			testing.expect(t, status == .Ok || status == .Capacity_Missing);
			testing.expect_value(t, scene.simulation.state, p.Simulation_State.Ready);
			if status == .Ok
			{
				successes += 1;
				expect_static_position(t, &scene, added if operation == 0 else fixed, next);
			}
			else
			{
				failures += 1;
				expect_static_position(t, &scene, fixed, position);
				testing.expect_value(t, scene.simulation.statics.count, 1);
			}
			scene_dispose(t, &scene);
		}
	}
	testing.expect(t, failures > 0 && successes > 0);
}
