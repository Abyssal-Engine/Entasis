package precision_mutations_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
precision_property_setters_preserve_body_residue_across_shape_and_mobility_changes :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 10000, 1000000, 1000000000})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {origin + 0.001953125, -origin + 0.125, origin - 0.25};
		body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
		for sleeping in 0 ..< 2
		{
			if sleeping == 1
			{
				sleep_body(t, &scene, body);
			}
			testing.expect_value(t, e.body_set_velocity(&scene.world, body, e.velocity({1, 2, 3})), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, i32(0));
			testing.expect_value(t, e.body_set_activity(&scene.world, body, e.body_activity(-1, 17)), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, e.body_set_inertia(&scene.world, body, {}), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, e.body_set_inertia(&scene.world, body, body_description(scene.sphere).local_inertia), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, e.body_set_shape(&scene.world, body, {}), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, e.body_set_shape(&scene.world, body, scene.box), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			testing.expect_value(t, e.body_set_collidable(&scene.world, body, e.collidable(scene.sphere, {}, 0.125, 0.25)), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			bounds: e.Bounding_Box;
			status: e.Status;
			bounds, status = e.body_bounds(&scene.world, body);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect(t, f64(bounds.min.x) <= position.x - 0.5 && f64(bounds.max.x) >= position.x + 0.5);
			state: e.Body_State;
			state, status = e.body_get(&scene.world, body);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, state.velocity.linear, e.Vector3{1, 2, 3});
			testing.expect_value(t, state.activity.minimum_timestep_count_under_threshold, u8(17));
			testing.expect_value(t, state.collidable.minimum_speculative_margin, f32(0.125));
		}
		scene_dispose(t, &scene);
	}
}

@(test)
precision_static_properties_preserve_residue_with_both_wake_policies :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 10000, 1000000, 1000000000})
	{
		for awakening in e.Awakening_Policy
		{
			scene: Scene;
			scene_initialize(t, &scene);
			position: e.Global_Position = {origin + 0.001953125, -origin + 0.125, origin - 0.25};
			handle: e.Static_Handle = add_static(t, &scene, position);
			testing.expect_value(t, e.static_set_shape(&scene.world, handle, scene.box, awakening), e.Status.Ok);
			expect_static_position(t, &scene, handle, position);
			testing.expect_value(t, e.static_set_continuity(&scene.world, handle, {mode=.Passive}, awakening), e.Status.Ok);
			expect_static_position(t, &scene, handle, position);
			state: e.Static_State;
			status: e.Status;
			state, status = e.static_get(&scene.world, handle);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, state.shape, scene.box);
			testing.expect_value(t, state.continuity.mode, e.Continuous_Detection_Mode.Passive);
			// the preexisting low-level shape convenience owners retain the same contract
			testing.expect_value(t, p.simulation_set_static_shape(scene.simulation, handle, scene.sphere), p.Physics_Status.Ok);
			expect_static_position(t, &scene, handle, position);
			testing.expect_value(t, p.simulation_set_static_shape_without_awakening_bodies(scene.simulation, handle, scene.box), p.Physics_Status.Ok);
			expect_static_position(t, &scene, handle, position);
			scene_dispose(t, &scene);
		}
	}
}

@(test)
precision_float_pose_and_complete_replacements_intentionally_drop_residue :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9 - 0.25};
	rounded: e.Global_Position = {1e9, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	handle: e.Static_Handle = add_static(t, &scene, position);
	for mode in 0 ..< 2
	{
		state: e.Body_State;
		static_state: e.Static_State;
		status: e.Status;
		state, status = e.body_get(&scene.world, body);
		testing.expect_value(t, status, e.Status.Ok);
		static_state, status = e.static_get(&scene.world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		if mode == 0
		{
			testing.expect_value(t, e.body_set_pose(&scene.world, body, state.pose), e.Status.Ok);
			testing.expect_value(t, e.static_set_pose(&scene.world, handle, static_state.pose, .None), e.Status.Ok);
		}
		else
		{
			testing.expect_value(t, e.body_apply(&scene.world, body, state), e.Status.Ok);
			testing.expect_value(t, e.static_apply(&scene.world, handle, static_state, .None), e.Status.Ok);
		}
		expect_body_position(t, &scene, body, rounded);
		expect_static_position(t, &scene, handle, rounded);
		testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {position, {w=1}}), e.Status.Ok);
		testing.expect_value(t, e.static_set_global_pose(&scene.world, handle, {position, {w=1}}, .None), e.Status.Ok);
	}
	// an explicit high author can deliberately choose the exact cache as well
	testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {rounded, {w=1}}), e.Status.Ok);
	testing.expect_value(t, e.static_set_global_pose(&scene.world, handle, {rounded, {w=1}}, .None), e.Status.Ok);
	expect_body_position(t, &scene, body, rounded);
	expect_static_position(t, &scene, handle, rounded);
}

@(test)
precision_legacy_batches_and_commands_remain_complete_float_authors :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9 - 0.25};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	body_state: e.Body_Description;
	static_state: e.Static_Description;
	body_state, _ = e.body_get(&scene.world, body);
	static_state, _ = e.static_get(&scene.world, fixed);
	body_handles: [1]e.Body_Handle = {body};
	static_handles: [1]e.Static_Handle = {fixed};
	body_descriptions: [1]e.Body_Description = {body_state};
	static_descriptions: [1]e.Static_Description = {static_state};
	count: int;
	status: e.Status;
	count, status = e.body_apply_batch(&scene.world, body_handles[:], body_descriptions[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, 1);
	count, status = e.static_apply_batch(&scene.world, static_handles[:], static_descriptions[:], .None);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, 1);
	expect_body_position(t, &scene, body, {1e9, -1e9, 1e9});
	expect_static_position(t, &scene, fixed, {1e9, -1e9, 1e9});
	testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {position, {w=1}}), e.Status.Ok);
	testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {position, {w=1}}, .None), e.Status.Ok);
	commands: [2]e.Command = {
		e.command_body_apply(body, body_state), e.command_static_apply(fixed, static_state, .None),
	};
	buffer: e.Command_Buffer = e.command_buffer(commands[:]);
	count, status = e.world_apply_commands(&scene.world, &buffer);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, count, 2);
	expect_body_position(t, &scene, body, {1e9, -1e9, 1e9});
	expect_static_position(t, &scene, fixed, {1e9, -1e9, 1e9});
	scene_dispose(t, &scene);
}
