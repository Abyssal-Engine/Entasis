package precision_targets_tests

import "base:runtime"
import "core:mem"
import "core:os"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Fallible_Target_Allocation :: struct
{
	backing: mem.Allocator,
	requests, reject_at: int,
}

@(private)
fallible_target_allocate :: proc(
	data: rawptr, mode: mem.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int, location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, mem.Allocator_Error)
{
	state: ^Fallible_Target_Allocation = (^Fallible_Target_Allocation)(data);
	if mode == .Alloc || mode == .Alloc_Non_Zeroed || mode == .Resize || mode == .Resize_Non_Zeroed
	{
		state.requests += 1;
		if state.reject_at > 0 && state.requests >= state.reject_at
		{
			return nil, .Out_Of_Memory;
		}
	}
	return state.backing.procedure(state.backing.data, mode, size, alignment, old_memory, old_size, location);
}

@(test)
precision_target_wake_allocation_failures_preserve_sleeping_authority :: proc(t: ^testing.T)
{
	failures, successes: int;
	for kind in Target_Kind
	{
		for fail_after in 1 ..= 32
		{
			scene: Scene;
			mem.tracking_allocator_init(&scene.tracker, os.heap_allocator(), os.heap_allocator());
			state: Fallible_Target_Allocation = {backing=mem.tracking_allocator(&scene.tracker)};
			testing.expect_value(t, util.buffer_pool_initialize_with_allocator(&scene.pool,
				{procedure=fallible_target_allocate, data=&state}, 128), util.Memory_Status.Ok);
			memory_status: util.Memory_Status;
			scene.storage, memory_status = util.buffer_pool_take_at_least(&scene.pool, physics.Simulation, 1);
			testing.expect_value(t, memory_status, util.Memory_Status.Ok);
			scene.simulation = &scene.storage.memory[0];
			scene.simulation^ = {};
			description: physics.Simulation_Create_Description = physics.simulation_create_description_default(&scene.pool);
			description.allocation_sizes = {
				bodies=16, statics=8, inactive_body_sets=16, shapes_per_type=8, constraints=32,
				constraint_batches=4, initial_constraints_per_type_batch=8, minimum_constraints_per_body=4,
				broad_phase_candidates=32, pairs=32, collision_child_pairs=32, inactive_pairs=32,
				pending_pairs_per_worker=32, workers=1,
			};
			description.solve_description = {velocity_iteration_count=2, substep_count=1, fallback_batch_threshold=2};
			testing.expect_value(t, physics.simulation_create(scene.simulation, &description).status, physics.Physics_Status.Ok);
			position: physics.Global_Position = {1e9, -1e9, 1e9};
			body: physics.Body_Handle = add_body(t, &scene, position);
			handle: physics.Constraint_Handle = add_float_target(t, &scene, body, {1e9, -1e9, 1e9}, kind);
			target: physics.Global_Position = {1e9 + 0.001953125, -1e9, 1e9};
			testing.expect_value(t, physics.simulation_set_constraint_global_target(scene.simulation, handle, target), physics.Physics_Status.Ok);
			sleep_body(t, &scene, body);
			set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
			// consume the released active slot, then fill its existing capacity.
			// waking now crosses genuine backing-allocation boundaries
			capacity: int = int(scene.simulation.bodies.sets.memory[0].dynamics_state.length);
			for index in 0 ..< capacity
			{
				add_body(t, &scene, {f64(index), 0, 0});
			}
			state.reject_at = state.requests + fail_after;
			updated: physics.Global_Position = {target.x + 0.015625, target.y, target.z};
			status: physics.Physics_Status = physics.simulation_set_constraint_global_target(scene.simulation, handle, updated);
			state.reject_at = 0;
			testing.expect_value(t, scene.simulation.state, physics.Simulation_State.Ready);
			testing.expect(t, status == .Ok || status == .Capacity_Missing);
			if status == .Ok
			{
				successes += 1;
				expect_target(t, &scene, handle, updated);
			}
			else
			{
				failures += 1;
				expect_target(t, &scene, handle, target);
				testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
			}
			testing.expect_value(t, body_position(t, &scene, body), position);
			scene_dispose(t, &scene);
		}
	}
	testing.expect(t, failures > 0 && successes > 0);
}
