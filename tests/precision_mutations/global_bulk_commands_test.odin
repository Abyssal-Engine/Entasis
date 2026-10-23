package precision_mutations_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"

@(test)
global_bulk_add_apply_preserves_positions :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	positions: [3]e.Global_Position = {{1e9 + 0.002, -1e9, 1e9}, {1e9 + 2.002, -1e9, 1e9}, {1e9 + 4.002, -1e9, 1e9}};
	bodies: [3]e.Body_Description;
	statics: [3]e.Static_Description;
	body_handles: [3]e.Body_Handle;
	static_handles: [3]e.Static_Handle;
	for index in 0 ..< len(positions)
	{
		bodies[index] = body_description(scene.sphere);
		statics[index] = e.static_body(scene.sphere, e.pose());
		// the position stream replaces this float field before description admission
		bodies[index].pose.position.x = math.nan_f32();
		statics[index].pose.position.x = math.nan_f32();
	}
	count: int;
	status: e.Status;
	count, status = e.body_add_batch_global(&scene.world, bodies[:], body_handles[:], positions[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, len(positions));
	count, status = e.static_add_batch_global(&scene.world, statics[:], static_handles[:], positions[:], .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, len(positions));
	for index in 0 ..< len(positions)
	{
		expect_body_position(t, &scene, body_handles[index], positions[index]);
		expect_static_position(t, &scene, static_handles[index], positions[index]);
		positions[index].x += 0.002;
	}
	count, status = e.body_apply_batch_global(&scene.world, body_handles[:], bodies[:], positions[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, len(positions));
	count, status = e.static_apply_batch_global(&scene.world, static_handles[:], statics[:], positions[:], .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, len(positions));
	for index in 0 ..< len(positions)
	{
		expect_body_position(t, &scene, body_handles[index], positions[index]);
		expect_static_position(t, &scene, static_handles[index], positions[index]);
	}
	positions[1].x = math.inf_f64(1);
	count, status = e.body_add_batch_global(&scene.world, bodies[:], body_handles[:], positions[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, count, 1);
	testing.expect(t, body_handles[0].value >= 0);
	testing.expect_value(t, body_handles[1], e.body_handle_invalid());
	testing.expect_value(t, body_handles[2], e.body_handle_invalid());
	count, status = e.body_add_batch_global(nil, nil, nil, nil);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, 0);
}

@(test)
global_commands_preserve_order_and_failure_prefix :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	initial: e.Global_Position = {1e9 + 0.002, -1e9, 1e9};
	existing: e.Body_Handle = add_body(t, &scene, initial, scene.sphere);
	description: e.Body_Description = body_description(scene.sphere);
	description.pose.position.x = math.nan_f32();
	commands: [5]e.Command = {
		e.command_body_add(description, 0),
		e.command_static_add(e.static_body(scene.sphere, e.pose()), 0, .None),
		e.command_body_remove(existing),
		e.command_body_add(description, 1),
		e.command_body_add(description, 2),
	};
	positions: [5]e.Global_Position = {initial, {initial.x + 0.002, initial.y, initial.z},
		{math.nan_f64(), 0, 0}, {math.inf_f64(1), 0, 0}, initial};
	statuses: [5]e.Status = {.Disposed, .Disposed, .Disposed, .Disposed, .Disposed};
	body_results: [3]e.Body_Handle;
	static_results: [1]e.Static_Handle;
	buffer: e.Command_Buffer = e.command_buffer(commands[:], statuses[:], body_results[:], static_results[:]);
	processed: int;
	status: e.Status;
	processed, status = e.world_apply_commands_global(&scene.world, &buffer, positions[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, processed, 3);
	for index in 0 ..< processed
	{
		testing.expect_value(t, statuses[index], e.Status.Ok);
	}
	testing.expect_value(t, statuses[3], e.Status.Invalid_Argument);
	testing.expect_value(t, statuses[4], e.Status.Disposed);
	testing.expect_value(t, body_results[1], e.body_handle_invalid());
	testing.expect_value(t, body_results[2], e.body_handle_invalid());
	expect_body_position(t, &scene, body_results[0], positions[0]);
	expect_static_position(t, &scene, static_results[0], positions[1]);
	_, status = e.body_get(&scene.world, existing);
	testing.expect_value(t, status, e.Status.Not_Found);
}

@(test)
global_commands_reject_malformed_position_stream :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	commands: [1]e.Command = {e.command_body_add(body_description(scene.sphere), 0)};
	statuses: [1]e.Status = {.Disposed};
	results: [1]e.Body_Handle = {{777}};
	positions: [2]e.Global_Position = {{1e9 + 0.002, 0, 0}, {}};
	buffer: e.Command_Buffer = e.command_buffer(commands[:], statuses[:], results[:]);
	for count in ([2]int{0, 2})
	{
		processed: int;
		status: e.Status;
		processed, status = e.world_apply_commands_global(&scene.world, &buffer, positions[:count]);
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		testing.expect_value(t, processed, 0);
		testing.expect_value(t, results[0], e.Body_Handle{777});
		testing.expect_value(t, statuses[0], e.Status.Disposed);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 0);
	}
}
