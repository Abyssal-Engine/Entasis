package precision_broadphase_tests

import "core:math"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Awakening_Record :: struct
{
	accepted: physics.Body_Handle,
	visits: u32,
}

accept_static_candidate :: proc "contextless" (user: rawptr, body: physics.Body_Handle) -> physics.Collision_Testing_State
{
	record: ^Awakening_Record = (^Awakening_Record)(user);
	record.visits += 1;
	return .Allow if body == record.accepted else .Reject;
}

@(test)
precision_public_filtered_static_mutations_use_global_bounds :: proc(t: ^testing.T)
{
	for mode in 0 ..< 3
	{
		world: entasis.World;
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {};
		description.capacity = {
			bodies=8, statics=8, inactive_body_sets=4, shapes_per_type=2, constraints=16,
			initial_constraints_per_type_batch=4, minimum_constraints_per_body=4,
			broad_phase_candidates=32, pairs=32, inactive_pairs=16, pending_pairs_per_worker=16,
		};
		testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok);
		scene: Scene;
		status: physics.Physics_Status;
		scene.simulation, status = entasis.world_borrow_simulation(&world);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		box: physics.Box = {0.0625, 0.125, 0.25};
		shape: physics.Typed_Index;
		shape, status = physics.shape_registry_add(&scene.simulation.shapes, physics.BOX_TYPE_ID, &box);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		position: physics.Global_Position = {1e9 + 32.001953125, -1e9 - 16.00390625, 1e9};
		if mode == 0
		{
			// this legacy add authors a float position, place its overlap oracle at
			// that actual position rather than a discarded experimental proxy hit
			position = physics.world_precision_position_from_float(physics.world_precision_position_cache(position));
		}
		body: physics.Body_Handle = add_body(t, &scene, shape, position);
		static: physics.Static_Handle;
		if mode != 0
		{
			static = add_static(t, &scene, shape, position);
		}
		sleep_body(t, &scene, body);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 0);
		record: Awakening_Record = {accepted=body};
		state: entasis.Static_Description = {
			shape=shape, pose={position=physics.world_precision_position_cache(position), orientation=util.quaternion_identity()},
		};
		if mode == 0
		{
			static, status = entasis.static_add_filtered(&world, state, accept_static_candidate, &record);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		else if mode == 1
		{
			testing.expect_value(t, entasis.static_remove_filtered(&world, static, accept_static_candidate, &record), entasis.Status.Ok);
		}
		else
		{
			// numeric admission occurs before a user callback or membership change
			bad: entasis.Static_Description = state;
			bad.pose.position.x = math.F32_MAX;
			before: physics.Global_Bounds = static_bounds(t, &scene, static);
			testing.expect_value(t, entasis.static_apply_filtered(&world, static, bad, accept_static_candidate, &record), entasis.Status.Invalid_Argument);
			testing.expect_value(t, record.visits, u32(0));
			testing.expect_value(t, static_bounds(t, &scene, static), before);
			retained: physics.Global_Position;
			retained, _ = physics.statics_get_global_position(&scene.simulation.statics, static);
			testing.expect_value(t, retained, position);
			testing.expect_value(t, entasis.static_apply_filtered(&world, static, state, accept_static_candidate, &record), entasis.Status.Ok);
		}
		testing.expect(t, record.visits > 0);
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 1);
		expect_body_geometry(t, &scene, body);
		if mode != 1
		{
			local: physics.Shape_Bounds = physics.box_bounds(box, util.quaternion_identity());
			expect_enclosure(t, static_bounds(t, &scene, static), physics.world_precision_position_from_float(state.pose.position), util.Bounding_Box{min=local.min, max=local.max});
		}
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	}
}

@(test)
precision_public_explicit_bounds_refresh_preserves_or_imports_positions :: proc(t: ^testing.T)
{
	for asleep in 0 ..< 2
	{
		world: entasis.World;
		description: entasis.World_Description = entasis.world_description_default();
		description.gravity = {};
		description.capacity = {
			bodies=8, statics=8, inactive_body_sets=4, shapes_per_type=2, constraints=16,
			initial_constraints_per_type_batch=4, minimum_constraints_per_body=4,
			broad_phase_candidates=32, pairs=32, inactive_pairs=16, pending_pairs_per_worker=16,
		};
		testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok);
		scene: Scene;
		status: physics.Physics_Status;
		scene.simulation, status = entasis.world_borrow_simulation(&world);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		box: physics.Box = {0.0625, 0.125, 0.25};
		shape: physics.Typed_Index;
		shape, status = physics.shape_registry_add(&scene.simulation.shapes, physics.BOX_TYPE_ID, &box);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		position: physics.Global_Position = {1e9 + 32.001953125, -1e9 - 16.00390625, 1e9};
		body: physics.Body_Handle = add_body(t, &scene, shape, position);
		static: physics.Static_Handle = add_static(t, &scene, shape, position);
		if asleep != 0
		{
			sleep_body(t, &scene, body);
		}
		location: physics.Body_Memory_Location;
		location, status = physics.bodies_resolve(&scene.simulation.bodies, body);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		set: ^physics.Body_Set = &scene.simulation.bodies.sets.memory[location.set_index];
		static_index: int;
		static_index, status = physics.statics_resolve(&scene.simulation.statics, static);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
		static_value: ^physics.Static = &scene.simulation.statics.statics.memory[static_index];

		// an unchanged rounded cache is not an explicit position edit
		testing.expect_value(t, entasis.body_update_bounds(&world, body), entasis.Status.Ok);
		testing.expect_value(t, entasis.static_update_bounds(&world, static), entasis.Status.Ok);
		testing.expect_value(t, set.global_positions.memory[location.index], position);
		testing.expect_value(t, scene.simulation.statics.global_positions.memory[static_index], position);
		expect_body_geometry(t, &scene, body);
		local: physics.Shape_Bounds = physics.box_bounds(box, util.quaternion_identity());
		expect_enclosure(t, static_bounds(t, &scene, static), position, util.Bounding_Box{min=local.min, max=local.max});

		// a changed cache imports that explicit f32 position, after admission
		explicit_position: util.Vector3 = {1e9 + 128, -1e9 + 256, 1e9 - 128};
		set.dynamics_state.memory[location.index].motion.pose.position = explicit_position;
		static_value.pose.position = explicit_position;
		testing.expect_value(t, entasis.body_update_bounds(&world, body), entasis.Status.Ok);
		testing.expect_value(t, entasis.static_update_bounds(&world, static), entasis.Status.Ok);
		expected: physics.Global_Position = physics.world_precision_position_from_float(explicit_position);
		testing.expect_value(t, set.global_positions.memory[location.index], expected);
		testing.expect_value(t, scene.simulation.statics.global_positions.memory[static_index], expected);
		expect_body_geometry(t, &scene, body);
		expect_enclosure(t, static_bounds(t, &scene, static), expected, util.Bounding_Box{min=local.min, max=local.max});

		body_before: physics.Global_Bounds = body_bounds(t, &scene, body);
		static_before: physics.Global_Bounds = static_bounds(t, &scene, static);
		set.dynamics_state.memory[location.index].motion.pose.position.x = math.F32_MAX;
		static_value.pose.position.x = math.F32_MAX;
		testing.expect_value(t, entasis.body_update_bounds(&world, body), entasis.Status.Invalid_Argument);
		testing.expect_value(t, entasis.static_update_bounds(&world, static), entasis.Status.Invalid_Argument);
		testing.expect_value(t, body_bounds(t, &scene, body), body_before);
		testing.expect_value(t, static_bounds(t, &scene, static), static_before);
		testing.expect_value(t, set.global_positions.memory[location.index], expected);
		testing.expect_value(t, scene.simulation.statics.global_positions.memory[static_index], expected);
		set.dynamics_state.memory[location.index].motion.pose.position = explicit_position;
		static_value.pose.position = explicit_position;
		testing.expect_value(t, scene.simulation.bodies.sets.memory[0].count, 1 - asleep);
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	}
}
