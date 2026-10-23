package precision_mutations_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

@(test)
precision_global_authoring_publishes_correct_tree_bounds_and_shape_pose :: proc(t: ^testing.T)
{
	for origin in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {origin + 0.001953125, -origin + 0.125, origin + 0.25};
		body: e.Body_Handle = add_body(t, &scene, position, scene.box);
		static_position: e.Global_Position = {position.x, position.y + 10, position.z};
		fixed: e.Static_Handle = add_static(t, &scene, static_position);
		testing.expect_value(t, e.static_set_shape(&scene.world, fixed, scene.box, .None), e.Status.Ok);
		query: e.Query_Context;
		testing.expect_value(t, e.query_context_init(&query, &scene.world), e.Status.Ok);
		stack: [128]i32;
		for _ in 0 ..< 3
		{
			for kind in 0 ..< 2
			{
				center: e.Global_Position = position if kind == 0 else static_position;
				for gap in ([2]f64{-0.002, 0.002})
				{
					segment: p.World_Precision_Segment = {{center.x - 2, center.y + 0.5 + gap, center.z}, {center.x + 2, center.y + 0.5 + gap, center.z}};
					hit: e.Global_Ray_Hit;
					status: e.Status;
					hit, status = e.ray_cast_global_closest_with_context(&query, segment, stack[:]);
					if gap > 0
					{
						testing.expect_value(t, status, e.Status.Not_Found);
					}
					else
					{
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect_value(t, p.collidable_reference_raw_handle(hit.collidable), body.value if kind == 0 else fixed.value);
						testing.expect(t, abs(hit.location.x - (center.x - 0.5)) < 1.3e-7);
					}
				}
			}
			position.x += 0.001953125;
			static_position.x -= 0.001953125;
			testing.expect_value(t, e.body_set_global_pose(&scene.world, body, {position, {w=1}}), e.Status.Ok);
			testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {static_position, {w=1}}, .None), e.Status.Ok);
			expect_body_position(t, &scene, body, position);
			expect_static_position(t, &scene, fixed, static_position);
		}
		testing.expect_value(t, e.query_context_destroy(&query), e.Status.Ok);
		scene_dispose(t, &scene);
	}
}

@(test)
precision_global_mutation_invalid_coordinates_reject_before_publishing :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	position: e.Global_Position = {1e9 + 0.125, -1e9, 1e9};
	body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
	fixed: e.Static_Handle = add_static(t, &scene, position);
	sleep_body(t, &scene, body);
	set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
	body_count: int = scene.simulation.bodies.sets.memory[0].count;
	static_count: int = scene.simulation.statics.count;
	for invalid in ([4]e.Global_Position{{math.nan_f64(), 0, 0}, {0, math.inf_f64(1), 0}, {0, 0, math.inf_f64(-1)}, {1e40, 0, 0}})
	{
		status: e.Status;
		_, status = e.body_add_global(&scene.world, body_description(scene.sphere), invalid);
		testing.expect(t, status != .Ok);
		_, status = e.static_add_global(&scene.world, e.static_body(scene.sphere, e.pose()), invalid);
		testing.expect(t, status != .Ok);
		testing.expect(t, e.body_set_global_pose(&scene.world, body, {invalid, {w=1}}) != .Ok);
		testing.expect(t, e.static_set_global_pose(&scene.world, fixed, {invalid, {w=1}}) != .Ok);
		expect_body_position(t, &scene, body, position);
		expect_static_position(t, &scene, fixed, position);
		testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, body_count);
		testing.expect_value(t, scene.simulation.statics.count, static_count);
	}
	// largest finite f32 is allowed for a shapeless high position. a shape
	// extending beyond that range must be rejected by the bounds owner
	limit: f64 = f64(f32(math.F32_MAX));
	remote: e.Body_Handle;
	status: e.Status;
	remote, status = e.body_add_global(&scene.world, body_description(e.Shape_Handle{}), {limit, 0, 0});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.body_set_shape(&scene.world, remote, scene.sphere), e.Status.Invalid_Argument);
	expect_body_position(t, &scene, remote, {limit, 0, 0});
	scene_dispose(t, &scene);
}

@(test)
precision_global_static_edits_honor_awakening_policy :: proc(t: ^testing.T)
{
	for wake in ([2]e.Awakening_Policy{.None, .Overlaps})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: e.Global_Position = {1e9 + 0.125, -1e9 + 0.25, 1e9};
		body: e.Body_Handle = add_body(t, &scene, position, scene.box);
		fixed: e.Static_Handle = add_static(t, &scene, {position.x + 256, position.y, position.z});
		sleep_body(t, &scene, body);
		testing.expect_value(t, e.static_set_global_pose(&scene.world, fixed, {position, {w=1}}, wake), e.Status.Ok);
		set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
		testing.expect(t, (set_index == 0) == (wake == .Overlaps));
		expect_body_position(t, &scene, body, position);
		expect_static_position(t, &scene, fixed, position);
		scene_dispose(t, &scene);
	}
}
