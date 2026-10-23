package precision_mutations_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

High_Static_Filter :: struct
{
	world: ^e.World,
	accepted: e.Body_Handle,
	fixed: e.Static_Handle,
	calls: int,
	observed_body, observed_static: e.Global_Position,
	read_status: e.Status,
}

high_static_filter :: proc "contextless" (user: rawptr, body: e.Body_Handle) -> e.Collision_Testing_State
{
	state: ^High_Static_Filter = (^High_Static_Filter)(user);
	state.calls += 1;
	pose: e.Global_Pose;
	status: e.Status;
	pose, status = e.body_get_global_pose(state.world, body);
	state.read_status = status;
	if status == .Ok
	{
		if body == state.accepted
		{
			state.observed_body = pose.position;
		}
		if state.fixed.value >= 0
		{
			pose, status = e.static_get_global_pose(state.world, state.fixed);
			state.read_status = status;
			state.observed_static = pose.position;
		}
	}
	return .Allow if state.read_status == .Ok && body == state.accepted else .Reject;
}

@(test)
precision_filtered_static_add_preserves_high_position_and_filter_reads :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {origin + 0.001953125, -origin + 0.125, origin + 0.25};
		a: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
		b_position: e.Global_Position = {position.x + 0.001953125, position.y, position.z};
		b: e.Body_Handle = add_body(t, &scene, b_position, scene.sphere);
		sleep_body(t, &scene, a);
		sleep_body(t, &scene, b);
		filter: High_Static_Filter = {world=&scene.world, accepted=b, fixed=e.static_handle_invalid()};
		description: e.Static_Description = e.static_body(scene.sphere, e.pose({12345, 0, 0}));
		fixed: e.Static_Handle;
		status: e.Status;
		fixed, status = e.static_add_global_filtered(&scene.world, description, position, high_static_filter, &filter);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, filter.calls, 2);
		testing.expect_value(t, filter.read_status, e.Status.Ok);
		testing.expect_value(t, filter.observed_body, b_position);
		testing.expect(t, scene.simulation.bodies.handle_to_location.memory[a.value].set_index > 0);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[b.value].set_index, i32(0));
		expect_static_position(t, &scene, fixed, position);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_filtered_static_apply_visits_old_and_new_bounds_before_commit :: proc(t: ^testing.T)
{
	for origin in ([2]f64{0, 1e9})
	{
		for accepted_index in 0 ..< 2
		{
			scene: Scene;
			scene_initialize(t, &scene);
			old: e.Global_Position = {origin + 0.001953125, -origin, origin};
			next: e.Global_Position = {old.x + 256.001953125, old.y, old.z};
			positions: [2]e.Global_Position = {old, next};
			bodies: [2]e.Body_Handle;
			for index in 0 ..< 2
			{
				bodies[index] = add_body(t, &scene, positions[index], scene.sphere);
				sleep_body(t, &scene, bodies[index]);
			}
			fixed: e.Static_Handle = add_static(t, &scene, old);
			filter: High_Static_Filter = {world=&scene.world, accepted=bodies[accepted_index], fixed=fixed};
			testing.expect_value(t, e.static_apply_global_filtered(&scene.world, fixed,
				e.static_body(scene.box, e.pose()), next, high_static_filter, &filter), e.Status.Ok);
			testing.expect_value(t, filter.calls, 2);
			testing.expect_value(t, filter.observed_static, old);
			testing.expect_value(t, filter.observed_body, positions[accepted_index]);
			testing.expect_value(t, filter.read_status, e.Status.Ok);
			for index in 0 ..< 2
			{
				testing.expect(t, (scene.simulation.bodies.handle_to_location.memory[bodies[index].value].set_index == 0) == (index == accepted_index));
			}
			expect_static_position(t, &scene, fixed, next);
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_filtered_static_invalid_authoring_never_invokes_filter :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	sleep_body(t, &scene, body);
	filter: High_Static_Filter = {world=&scene.world, accepted=body, fixed=fixed};
	for variant in 0 ..< 7
	{
		description: e.Static_Description = e.static_body(scene.sphere, e.pose());
		bad: e.Global_Position = position;
		switch variant
		{
		case 0:
			bad.x = math.nan_f64();
		case 1:
			bad.y = math.inf_f64(1);
		case 2:
			bad.z = 1e40;
		case 3:
			bad.x = f64(f32(math.F32_MAX));
		case 4:
			description.shape = {};
		case 5:
			description.pose.orientation = {};
		case 6:
			description.shape, _ = p.typed_index_create(0, 99999);
		}
		status: e.Status;
		_, status = e.static_add_global_filtered(&scene.world, description, bad, high_static_filter, &filter);
		testing.expect(t, status != .Ok);
		testing.expect(t, e.static_apply_global_filtered(&scene.world, fixed, description, bad, high_static_filter, &filter) != .Ok);
		testing.expect_value(t, filter.calls, 0);
		testing.expect_value(t, scene.simulation.statics.count, 1);
		testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
		expect_static_position(t, &scene, fixed, position);
	}
	// float filtered authoring now uses the same admission before callbacks
	invalid: e.Static_Description = e.static_body(scene.sphere, e.pose({1e9, -1e9, 1e9}));
	invalid.pose.orientation = {};
	status: e.Status;
	_, status = e.static_add_filtered(&scene.world, invalid, high_static_filter, &filter);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	testing.expect_value(t, e.static_apply_filtered(&scene.world, fixed, invalid, high_static_filter, &filter), e.Status.Invalid_Description);
	testing.expect_value(t, filter.calls, 0);
}

@(test)
precision_filtered_static_nil_and_legacy_float_authoring_are_explicit :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	sleep_body(t, &scene, body);
	fixed: e.Static_Handle;
	status: e.Status;
	fixed, status = e.static_add_global_filtered(&scene.world, e.static_body(scene.sphere, e.pose()), position, nil);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, i32(0));
	expect_static_position(t, &scene, fixed, position);
	sleep_body(t, &scene, body);
	testing.expect_value(t, e.static_apply_global_filtered(&scene.world, fixed, e.static_body(scene.box, e.pose()), position, nil), e.Status.Ok);
	testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, i32(0));
	state: e.Static_Description;
	state, status = e.static_get(&scene.world, fixed);
	testing.expect_value(t, status, e.Status.Ok);
	sleep_body(t, &scene, body);
	filter: High_Static_Filter = {world=&scene.world, accepted=e.body_handle_invalid(), fixed=fixed};
	testing.expect_value(t, e.static_apply_filtered(&scene.world, fixed, state, high_static_filter, &filter), e.Status.Ok);
	expect_static_position(t, &scene, fixed, {f64(state.pose.position.x), f64(state.pose.position.y), f64(state.pose.position.z)});
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
}

@(test)
precision_filtered_trigger_static_edits_do_not_awaken_or_call_filter :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9, 1e9};
	fixed: e.Static_Handle = add_static(t, &scene, position);
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	sleep_body(t, &scene, body);
	testing.expect_value(t, e.world_enable_triggers(&scene.world, {pair_capacity=32, candidates_per_worker=64, child_capacity=128}), e.Status.Ok);
	reference: e.Collidable_Reference;
	reference, _ = p.collidable_reference_static(fixed);
	testing.expect_value(t, e.trigger_set(&scene.world, reference, {}), e.Status.Ok);
	filter: High_Static_Filter = {world=&scene.world, accepted=body, fixed=fixed};
	next: e.Global_Position = {position.x + 0.001953125, position.y, position.z};
	testing.expect_value(t, e.static_apply_global_filtered(&scene.world, fixed, e.static_body(scene.box, e.pose()), next, high_static_filter, &filter), e.Status.Ok);
	testing.expect_value(t, filter.calls, 0);
	testing.expect(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index > 0);
	expect_static_position(t, &scene, fixed, next);
	testing.expect_value(t, e.static_remove_filtered(&scene.world, fixed, high_static_filter, &filter), e.Status.Ok);
	testing.expect_value(t, filter.calls, 0);
}
