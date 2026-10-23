package precision_mutations_tests

import "base:runtime"
import "core:mem"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

Fallible_Mutation_Allocation :: struct
{
	tracker: ^mem.Tracking_Allocator,
	requests, reject_at: int,
}

@(private)
fallible_mutation_allocate :: proc(
	data: rawptr, mode: mem.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int, location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, mem.Allocator_Error)
{
	state: ^Fallible_Mutation_Allocation = (^Fallible_Mutation_Allocation)(data);
	if mode == .Alloc || mode == .Alloc_Non_Zeroed || mode == .Resize || mode == .Resize_Non_Zeroed
	{
		state.requests += 1;
		if state.reject_at > 0 && state.requests >= state.reject_at
		{
			return nil, .Out_Of_Memory;
		}
	}
	backing: mem.Allocator = mem.tracking_allocator(state.tracker);
	return backing.procedure(backing.data, mode, size, alignment, old_memory, old_size, location);
}

@(test)
precision_global_and_property_edits_preserve_sleeping_authority_on_wake_failure :: proc(t: ^testing.T)
{
	failures, successes: int;
	for authored_position in 0 ..< 2
	{
		for fail_after in 1 ..= 32
		{
			scene: Scene;
			allocator: Fallible_Mutation_Allocation = {tracker=&scene.tracker};
			scene_initialize(t, &scene, 16, 1, 1, {procedure=fallible_mutation_allocate, data=&allocator}, 128);
			position: e.Global_Position = {1e9 + 0.001953125, -1e9 + 0.125, 1e9};
			body: e.Body_Handle = add_body(t, &scene, position, scene.sphere);
			sleep_body(t, &scene, body);
			set_index: i32 = scene.simulation.bodies.handle_to_location.memory[body.value].set_index;
			capacity: int = int(scene.simulation.bodies.sets.memory[0].dynamics_state.length);
			for index in 0 ..< capacity
			{
				add_body(t, &scene, {f64(index) * 4, 0, 0}, scene.sphere);
			}
			allocator.reject_at = allocator.requests + fail_after;
			updated: e.Global_Position = {position.x + 0.001953125, position.y, position.z};
			status: e.Status;
			if authored_position == 0
			{
				status = e.body_set_velocity(&scene.world, body, e.velocity({1, 0, 0}));
			}
			else
			{
				status = e.body_set_global_pose(&scene.world, body, {updated, {w=1}});
			}
			allocator.reject_at = 0;
			testing.expect(t, status == .Ok || status == .Capacity_Missing);
			testing.expect_value(t, scene.simulation.state, p.Simulation_State.Ready);
			if status == .Ok
			{
				successes += 1;
				expect_body_position(t, &scene, body, position if authored_position == 0 else updated);
			}
			else
			{
				failures += 1;
				expect_body_position(t, &scene, body, position);
				testing.expect_value(t, scene.simulation.bodies.handle_to_location.memory[body.value].set_index, set_index);
			}
			scene_dispose(t, &scene);
		}
	}
	testing.expect(t, successes > 0 && failures > 0);
}
