package precision_broadphase_tests

import "core:math"
import "core:mem"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_tree_insertion_and_refit_use_authoritative_centers :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	offsets: [5]f64 = {0, 10000, 1000000, 1000000000, -1000000000};
	bodies: [50]physics.Body_Handle;
	statics: [50]physics.Static_Handle;
	for offset, group in offsets
	{
		for shape, index in scene.shapes
		{
			position: physics.Global_Position = {offset + 32.001953125, -offset - 16.00390625, offset + f64(index) / 512};
			bodies[group * 10 + index] = add_body(t, &scene, shape, position);
			statics[group * 10 + index] = add_static(t, &scene, shape, position);
			local: physics.Shape_Bounds = local_bounds(t, &scene, shape, util.quaternion_identity());
			expect_enclosure(t, body_bounds(t, &scene, bodies[group * 10 + index]), position, util.Bounding_Box{min=local.min, max=local.max});
			expect_enclosure(t, static_bounds(t, &scene, statics[group * 10 + index]), position, util.Bounding_Box{min=local.min, max=local.max});
		}
	}
	for frame in 0 ..< 4
	{
		for body, index in bodies
		{
			position: physics.Global_Position;
			position, _ = physics.bodies_get_global_position(&scene.simulation.bodies, body);
			position.x += f64(frame + 1) / 512;
			position.y -= f64(frame + 1) / 256;
			testing.expect_value(t, physics.bodies_set_global_position(&scene.simulation.bodies, body, position), physics.Physics_Status.Ok);
			testing.expect_value(t, physics.statics_set_global_position(&scene.simulation.statics, statics[index], position), physics.Physics_Status.Ok);
		}
		testing.expect_value(t, physics.broad_phase_update(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
		for body, index in bodies
		{
			expect_body_geometry(t, &scene, body);
			position: physics.Global_Position;
			position, _ = physics.statics_get_global_position(&scene.simulation.statics, statics[index]);
			local: physics.Shape_Bounds = local_bounds(t, &scene, scene.shapes[index % 10], util.quaternion_identity());
			expect_enclosure(t, static_bounds(t, &scene, statics[index]), position, util.Bounding_Box{min=local.min, max=local.max});
		}
	}
}

@(test)
precision_description_replacement_and_finite_tree_admission :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: physics.Global_Position = {1e9 + 32.001953125, -1e9 - 0.001953125, 1e9};
	body: physics.Body_Handle = add_body(t, &scene, scene.shapes[0], position);
	static: physics.Static_Handle = add_static(t, &scene, scene.shapes[0], position);
	testing.expect_value(t, physics.simulation_set_static_shape_without_awakening_bodies(scene.simulation, static, scene.shapes[2]), physics.Physics_Status.Ok);
	actual_position: physics.Global_Position;
	actual_position, _ = physics.statics_get_global_position(&scene.simulation.statics, static);
	testing.expect_value(t, actual_position, position);
	local: physics.Shape_Bounds = local_bounds(t, &scene, scene.shapes[2], util.quaternion_identity());
	expect_enclosure(t, static_bounds(t, &scene, static), position, util.Bounding_Box{min=local.min, max=local.max});

	body_before: physics.Body_Description;
	body_before, _ = physics.bodies_get_description(&scene.simulation.bodies, body);
	body_leaf_before: physics.Global_Bounds = body_bounds(t, &scene, body);
	body_bad: physics.Body_Description = body_before;
	body_bad.pose.position.x = math.F32_MAX;
	testing.expect_value(t, physics.simulation_apply_body_description(scene.simulation, body, &body_bad), physics.Physics_Status.Invalid_Argument);
	actual_position, _ = physics.bodies_get_global_position(&scene.simulation.bodies, body);
	testing.expect_value(t, actual_position, position);
	testing.expect_value(t, body_bounds(t, &scene, body), body_leaf_before);

	static_before: physics.Static_Description;
	static_before, _ = physics.statics_get_description(&scene.simulation.statics, static);
	static_leaf_before: physics.Global_Bounds = static_bounds(t, &scene, static);
	static_bad: physics.Static_Description = static_before;
	static_bad.pose.position.x = math.F32_MAX;
	testing.expect_value(t, physics.simulation_apply_static_description_without_awakening_bodies(scene.simulation, static, &static_bad), physics.Physics_Status.Invalid_Argument);
	actual_position, _ = physics.statics_get_global_position(&scene.simulation.statics, static);
	testing.expect_value(t, actual_position, position);
	testing.expect_value(t, static_bounds(t, &scene, static), static_leaf_before);

	body_count: int = scene.simulation.bodies.sets.memory[0].count;
	static_count: int = scene.simulation.statics.count;
	status: physics.Physics_Status;
	_, status = physics.simulation_add_body(scene.simulation, &body_bad);
	testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
	_, status = physics.simulation_add_static_without_awakening_bodies(scene.simulation, &static_bad);
	testing.expect_value(t, status, physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, body_count);
	testing.expect_value(t, scene.simulation.statics.count, static_count);
	testing.expect_value(t, scene.simulation.broad_phase.active_tree.leaf_count, 1);
	testing.expect_value(t, scene.simulation.broad_phase.static_tree.leaf_count, 1);

	body_next: physics.Body_Description = body_before;
	body_next.pose.position = {1e9, -1e9, 1e9};
	body_next.collidable.shape = scene.shapes[6];
	testing.expect_value(t, physics.simulation_apply_body_description(scene.simulation, body, &body_next), physics.Physics_Status.Ok);
	expect_body_geometry(t, &scene, body);
	actual_position, _ = physics.bodies_get_global_position(&scene.simulation.bodies, body);
	testing.expect_value(t, actual_position, physics.Global_Position{1e9, -1e9, 1e9});
	body_next.collidable.shape = {};
	testing.expect_value(t, physics.simulation_apply_body_description(scene.simulation, body, &body_next), physics.Physics_Status.Ok);
	testing.expect_value(t, scene.simulation.broad_phase.active_tree.leaf_count, 0);
	body_next.collidable.shape = scene.shapes[2];
	testing.expect_value(t, physics.simulation_apply_body_description(scene.simulation, body, &body_next), physics.Physics_Status.Ok);
	expect_body_geometry(t, &scene, body);
}

@(test)
precision_sleep_wake_compaction_and_membership_preserve_bounds :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	dispatcher: util.Thread_Dispatcher;
	testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 4, 65536), util.Threading_Status.Ok);
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	handles: [19]physics.Body_Handle;
	for index in 0 ..< len(handles)
	{
		handles[index] = add_body(t, &scene, scene.shapes[index % 10], {1e9 + f64(index) / 64, -1e9 - 32.001953125, 1e9});
	}
	for index in 0 ..< 10
	{
		before: physics.Global_Bounds = body_bounds(t, &scene, handles[index]);
		sleep_body(t, &scene, handles[index], boundary);
		testing.expect_value(t, body_bounds(t, &scene, handles[index]), before);
		expect_body_geometry(t, &scene, handles[index]);
	}
	testing.expect_value(t, physics.bodies_ensure_capacity(&scene.simulation.bodies, 512, 256, 4), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.broad_phase_update(&scene.simulation.broad_phase, boundary), physics.Physics_Status.Ok);
	for index in 0 ..< 10
	{
		expect_body_geometry(t, &scene, handles[index]);
		testing.expect_value(t, physics.island_awakener_awaken_body(&scene.simulation.awakener, handles[index], boundary), physics.Physics_Status.Ok);
		expect_body_geometry(t, &scene, handles[index]);
	}
	testing.expect_value(t, physics.simulation_remove_body(scene.simulation, handles[5]), physics.Physics_Status.Ok);
	for handle, index in handles
	{
		if index != 5
		{
			expect_body_geometry(t, &scene, handle);
		}
	}
	reused: physics.Body_Handle = add_body(t, &scene, scene.shapes[2], {-1e9 - 32.001953125, 1e9 + 0.001953125, -1e9});
	testing.expect_value(t, reused, handles[5]);
	expect_body_geometry(t, &scene, reused);
}

@(test)
precision_origin_shift_keeps_predicted_envelopes_and_rejects_overflow :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	position: physics.Global_Position = {1e9 + 32.001953125, -1e9 - 16.00390625, 1e9 + 0.001953125};
	handles: [10]physics.Body_Handle;
	before: [10]physics.Global_Bounds;
	for shape, index in scene.shapes
	{
		handles[index] = add_body(t, &scene, shape, position);
		scene.simulation.bodies.sets.memory[0].dynamics_state.memory[index].motion.velocity.linear = {8, -4, 2};
		scene.simulation.bodies.sets.memory[0].collidables.memory[index].continuity.mode = .Passive;
	}
	static: physics.Static_Handle = add_static(t, &scene, scene.shapes[6], position);
	testing.expect_value(t, physics.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), physics.Physics_Status.Ok);
	for handle, index in handles
	{
		before[index] = body_bounds(t, &scene, handle);
	}
	delta: util.Vector3 = {1e9, -1e9, 1e9};
	testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, delta), physics.Physics_Status.Ok);
	for handle, index in handles
	{
		actual: physics.Global_Bounds = body_bounds(t, &scene, handle);
		expect_enclosure(t, actual, {-f64(delta.x), -f64(delta.y), -f64(delta.z)}, before[index]);
		expect_body_geometry(t, &scene, handle);
	}
	shifted: physics.Global_Position = {position.x - f64(delta.x), position.y - f64(delta.y), position.z - f64(delta.z)};
	local: physics.Shape_Bounds = local_bounds(t, &scene, scene.shapes[6], util.quaternion_identity());
	expect_enclosure(t, static_bounds(t, &scene, static), shifted, util.Bounding_Box{min=local.min, max=local.max});
	testing.expect_value(t, physics.broad_phase_update_bounds(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
	for handle in handles
	{
		expect_body_geometry(t, &scene, handle);
	}
	// at +F32_MAX the center remains representable, but any positive extent
	// leaves the finite tree domain. reject before publishing high positions
	before_body: physics.Global_Bounds = body_bounds(t, &scene, handles[0]);
	before_static: physics.Global_Bounds = static_bounds(t, &scene, static);
	testing.expect_value(t, physics.simulation_translate_origin(scene.simulation, {-math.F32_MAX, 0, 0}), physics.Physics_Status.Invalid_Argument);
	actual_position: physics.Global_Position;
	actual_position, _ = physics.bodies_get_global_position(&scene.simulation.bodies, handles[0]);
	testing.expect_value(t, actual_position, shifted);
	testing.expect_value(t, body_bounds(t, &scene, handles[0]), before_body);
	testing.expect_value(t, static_bounds(t, &scene, static), before_static);
}

pair_index :: proc "contextless" (pairs: []physics.Broad_Phase_Pair, a, b: physics.Collidable_Reference) -> int
{
	for pair, index in pairs
	{
		if (pair.a == a && pair.b == b) || (pair.a == b && pair.b == a)
		{
			return index;
		}
	}
	return -1;
}

@(test)
precision_actual_broadphase_pairs_enclose_double_geometry :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	handles: [32]physics.Body_Handle;
	statics: [16]physics.Static_Handle;
	positions: [48]physics.Global_Position;
	references: [48]physics.Collidable_Reference;
	for index in 0 ..< len(positions)
	{
		// dyadic integer-grid geometry gives an independent overlap oracle
		position: physics.Global_Position = {1e9 + f64(index / 2) / 2 + f64(index % 2) / 32, -1e9 + 0.001953125, 1e9};
		positions[index] = position;
		if index < len(handles)
		{
			handles[index] = add_body(t, &scene, scene.shapes[2], position);
			references[index], _ = physics.collidable_reference_body(.Dynamic, handles[index]);
		}
		else
		{
			position.x -= 8;
			positions[index] = position;
			statics[index - len(handles)] = add_static(t, &scene, scene.shapes[2], position);
			references[index], _ = physics.collidable_reference_static(statics[index - len(handles)]);
		}
	}
	pair_memory: [2048]physics.Broad_Phase_Pair;
	output: util.Buffer(physics.Broad_Phase_Pair) = {memory=&pair_memory[0], length=len(pair_memory)};
	for _ in 0 ..< 4
	{
		testing.expect_value(t, physics.broad_phase_update(&scene.simulation.broad_phase), physics.Physics_Status.Ok);
		count: int;
		status: physics.Physics_Status;
		count, status = physics.broad_phase_find_pairs(&scene.simulation.broad_phase, output);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		required: int;
		for a in 0 ..< len(handles)
		{
			for b in a + 1 ..< len(positions)
			{
				if abs(positions[a].x - positions[b].x) <= 0.125 &&
					abs(positions[a].y - positions[b].y) <= 0.25 && abs(positions[a].z - positions[b].z) <= 0.5
				{
					required += 1;
					testing.expect(t, pair_index(pair_memory[:count], references[a], references[b]) >= 0);
				}
			}
		}
		testing.expect(t, required > 0);
		testing.expect(t, count >= required);
	}
}

@(test)
precision_static_removal_and_shape_change_awaken_high_position_islands :: proc(t: ^testing.T)
{
	for mode in 0 ..< 3
	{
		scene: Scene;
		scene_initialize(t, &scene);
		position: physics.Global_Position = {1e9 + 32.001953125, -1e9 - 16.00390625, 1e9};
		body: physics.Body_Handle = add_body(t, &scene, scene.shapes[2], position);
		static: physics.Static_Handle = add_static(t, &scene, scene.shapes[6], position);
		if mode == 2
		{
			testing.expect_value(t, physics.trigger_system_enable(scene.simulation,
				{pair_capacity=16, candidates_per_worker=32, child_capacity=64},
				mem.tracking_allocator(&scene.tracker), .Legacy), physics.Physics_Status.Ok);
			testing.expect_value(t, physics.mixed_colliders_initialize(scene.simulation.triggers, {instance_capacity=4, part_capacity=8}), physics.Physics_Status.Ok);
			// keep one actual solid child, with the other excluded as a trigger
			reference: physics.Collidable_Reference;
			reference, _ = physics.collidable_reference_static(static);
			status: physics.Physics_Status;
			_, status = physics.mixed_colliders_part_set(scene.simulation.triggers.mixed, reference, 0, {role=.Trigger});
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		sleep_body(t, &scene, body);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 0);
		if mode == 0
		{
			testing.expect_value(t, physics.simulation_remove_static(scene.simulation, static), physics.Physics_Status.Ok);
		}
		else
		{
			testing.expect_value(t, physics.simulation_set_static_shape(scene.simulation, static, scene.shapes[2]), physics.Physics_Status.Ok);
			actual_position: physics.Global_Position;
			actual_position, _ = physics.statics_get_global_position(&scene.simulation.statics, static);
			testing.expect_value(t, actual_position, position);
		}
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 1);
		expect_body_geometry(t, &scene, body);
		scene_dispose(t, &scene);
	}
}
