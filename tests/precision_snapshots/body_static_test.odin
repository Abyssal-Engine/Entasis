package precision_snapshots_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
global_state_body_static_roundtrip_preserves_authoritative_bits_and_properties :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {origin + 0.001953125, -origin - 0.00390625, origin + 0.0078125};
		body: e.Body_Handle = add_body(t, &scene, position, scene.box);
		fixed: e.Static_Handle = add_static(t, &scene, position);
		testing.expect_value(t, e.body_set_velocity(&scene.world, body, e.velocity({1, 2, 3}, {4, 5, 6})), e.Status.Ok);
		state: e.Global_Body_State;
		static_state: e.Global_Static_State;
		status: e.Status;
		state, status = e.body_get_global_state(&scene.world, body);
		testing.expect_value(t, status, e.Status.Ok);
		static_state, status = e.static_get_global_state(&scene.world, fixed);
		testing.expect_value(t, status, e.Status.Ok);
		expect_position_bits(t, state.pose.position, position);
		expect_position_bits(t, static_state.pose.position, position);
		testing.expect_value(t, state.velocity, e.velocity({1, 2, 3}, {4, 5, 6}));
		testing.expect_value(t, state.local_inertia.inverse_mass, f32(1));
		testing.expect_value(t, state.collidable.shape, scene.box);
		testing.expect_value(t, state.activity.sleep_threshold, f32(-1));
		testing.expect_value(t, static_state.shape, scene.sphere);
		clone: e.Body_Handle;
		clone_static: e.Static_Handle;
		clone, status = e.body_add_global_state(&scene.world, state);
		testing.expect_value(t, status, e.Status.Ok);
		clone_static, status = e.static_add_global_state(&scene.world, static_state, .None);
		testing.expect_value(t, status, e.Status.Ok);
		read: e.Global_Body_State;
		read_static: e.Global_Static_State;
		read, status = e.body_get_global_state(&scene.world, clone);
		testing.expect_value(t, read, state);
		read_static, status = e.static_get_global_state(&scene.world, clone_static);
		testing.expect_value(t, read_static, static_state);
		testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {{42, 0, 0}, {w=1}}), e.Status.Ok);
		testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {{43, 0, 0}, {w=1}}, .None), e.Status.Ok);
		testing.expect_value(t, e.body_apply_global_state(&scene.world, body, state), e.Status.Ok);
		testing.expect_value(t, e.static_apply_global_state(&scene.world, fixed, static_state, .None), e.Status.Ok);
		read, status = e.body_get_global_state(&scene.world, body);
		testing.expect_value(t, read, state);
		read_static, status = e.static_get_global_state(&scene.world, fixed);
		testing.expect_value(t, read_static, static_state);
		scene_dispose(t, &scene);
	}
}

@(test)
global_state_capture_sleeping_kinematic_and_shapeless_is_read_only :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9 - 0.00390625, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, e.Shape_Handle{});
	kinematic: e.Body_Handle = add_kinematic(t, &scene, position);
	sleep_body(t, &scene, body);
	sleep_body(t, &scene, kinematic);
	active: int = scene.simulation.bodies.sets.memory[0].count;
	step: u64 = scene.simulation.step_index;
	for handle in ([2]e.Body_Handle{body, kinematic})
	{
		location: p.Body_Memory_Location = scene.simulation.bodies.handle_to_location.memory[handle.value];
		state: e.Global_Body_State;
		status: e.Status;
		state, status = e.body_get_global_state(&scene.world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		expect_position_bits(t, state.pose.position, position);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[handle.value], location);
		testing.expect_value(t, state.collidable.shape, e.Shape_Handle{});
	}
	testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, active);
	testing.expect_value(t, scene.simulation.step_index, step);
	scene_dispose(t, &scene);
}

@(test)
global_state_capture_does_not_round_nan_looking_f64_words_or_signed_zero :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	x: f64 = transmute(f64)((transmute(u64)f64(1e9) & 0xffffffff00000000) | 0x7fc00042);
	position: e.Global_Position = {x, -x, transmute(f64)u64(0x8000000000000000)};
	body: e.Body_Handle = add_body(t, &scene, position, {});
	fixed: e.Static_Handle = add_static(t, &scene, position);
	state, status := e.body_get_global_state(&scene.world, body);
	testing.expect_value(t, status, e.Status.Ok);
	expect_position_bits(t, state.pose.position, position);
	static_state, static_status := e.static_get_global_state(&scene.world, fixed);
	testing.expect_value(t, static_status, e.Status.Ok);
	expect_position_bits(t, static_state.pose.position, position);
	// the existing float snapshot deliberately cannot retain these low bits
	legacy, legacy_status := e.body_get(&scene.world, body);
	testing.expect_value(t, legacy_status, e.Status.Ok);
	testing.expect(t, f64(legacy.pose.position.x) != position.x);
	scene_dispose(t, &scene);
}

@(test)
global_state_rejects_invalid_values_before_publishing_position :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.001953125, -1e9, 0};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	body_state, _ := e.body_get_global_state(&scene.world, body);
	static_state, _ := e.static_get_global_state(&scene.world, fixed);
	for value in ([3]f64{math.nan_f64(), math.inf_f64(1), 1e40})
	{
		body_state.pose.position.x = value;
		static_state.pose.position.x = value;
		testing.expect(t, e.body_apply_global_state(&scene.world, body, body_state) != .Ok);
		testing.expect(t, e.static_apply_global_state(&scene.world, fixed, static_state, .None) != .Ok);
		expect_body_position(t, &scene, body, position);
		expect_static_position(t, &scene, fixed, position);
	}
	_, status := e.body_get_global_state(&scene.world, {value=-1});
	testing.expect_value(t, status, e.Status.Not_Found);
	_, status = e.static_get_global_state(&scene.world, {value=-1});
	testing.expect_value(t, status, e.Status.Not_Found);
	world: e.World;
	_, status = e.body_get_global_state(&world, body);
	testing.expect_value(t, status, e.Status.Disposed);
	scene_dispose(t, &scene);
}
