package precision_broadphase_tests

import "base:runtime"
import "core:mem"
import "core:os"
import "core:testing"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Fallible_Leaf_Allocation :: struct
{
	backing: mem.Allocator,
	requests: int,
	reject_at: int,
}

@(private)
fallible_leaf_allocate :: proc(
	data: rawptr, mode: mem.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int, location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, mem.Allocator_Error)
{
	state: ^Fallible_Leaf_Allocation = (^Fallible_Leaf_Allocation)(data);
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
precision_leaf_initialization_failures_release_all_ownership_and_allow_retry :: proc(t: ^testing.T)
{
	failures: int;
	successes: int;
	for fail_after in 1 ..= 256
	{
		tracker: mem.Tracking_Allocator;
		mem.tracking_allocator_init(&tracker, os.heap_allocator(), os.heap_allocator());
		allocation: Fallible_Leaf_Allocation = {backing=mem.tracking_allocator(&tracker)};
		pool: u.Buffer_Pool;
		testing.expect_value(t, u.buffer_pool_initialize_with_allocator(&pool,
			{procedure=fallible_leaf_allocate, data=&allocation}, 128), u.Memory_Status.Ok);
		allocation.reject_at = allocation.requests + fail_after;
		broad_phase: p.Broad_Phase;
		status: p.Physics_Status = p.broad_phase_initialize(&broad_phase, 17, 19, 256, 1, &pool);
		allocation.reject_at = 0;
		if status == .Ok
		{
			successes += 1;
		}
		else
		{
			failures += 1;
			testing.expect_value(t, status, p.Physics_Status.Capacity_Missing);
			testing.expect(t, broad_phase.pool == nil && broad_phase.state != .Ready);
			testing.expect_value(t, p.broad_phase_initialize(&broad_phase, 17, 19, 256, 1, &pool), p.Physics_Status.Ok);
		}
		testing.expect_value(t, p.broad_phase_dispose(&broad_phase), p.Physics_Status.Ok);
		testing.expect_value(t, u.buffer_pool_dispose(&pool), u.Memory_Status.Ok);
		testing.expect_value(t, tracker.current_memory_allocated, i64(0));
		mem.tracking_allocator_destroy(&tracker);
	}
	testing.expect(t, failures > 0 && successes > 0);
}

@(test)
precision_leaf_growth_failures_preserve_live_pairs_and_retry_cleanly :: proc(t: ^testing.T)
{
	failures: int;
	successes: int;
	for fail_after in 1 ..= 64
	{
		scene: Scene;
		scene_initialize(t, &scene, 16, 1);
		origin: p.Global_Position = {1e9 + 1.0/512, -1e9, 1e9};
		body: p.Body_Handle = add_body(t, &scene, scene.shapes[0], origin);
		stationary: p.Static_Handle = add_static(t, &scene, scene.shapes[0], origin);
		body_before: p.Global_Bounds = body_precise_bounds(t, &scene, body);
		static_before: p.Global_Bounds = static_precise_bounds(t, &scene, stationary);
		allocation: Fallible_Leaf_Allocation = {backing=scene.pool.allocator, reject_at=fail_after};
		scene.pool.allocator = {procedure=fallible_leaf_allocate, data=&allocation};
		status: p.Physics_Status = p.broad_phase_ensure_capacity(&scene.simulation.broad_phase, 4096, 4096, 4096);
		allocation.reject_at = 0;
		if status == .Ok
		{
			successes += 1;
		}
		else
		{
			failures += 1;
			testing.expect_value(t, status, p.Physics_Status.Capacity_Missing);
		}
		testing.expect_value(t, body_precise_bounds(t, &scene, body), body_before);
		testing.expect_value(t, static_precise_bounds(t, &scene, stationary), static_before);
		testing.expect_value(t, pair_count(t, &scene), 1);
		testing.expect_value(t, p.broad_phase_ensure_capacity(&scene.simulation.broad_phase, 4096, 4096, 4096), p.Physics_Status.Ok);
		testing.expect_value(t, body_precise_bounds(t, &scene, body), body_before);
		testing.expect_value(t, static_precise_bounds(t, &scene, stationary), static_before);
		testing.expect_value(t, pair_count(t, &scene), 1);
		testing.expect_value(t, p.broad_phase_resize(&scene.simulation.broad_phase, 16, 16, 256), p.Physics_Status.Ok);
		testing.expect_value(t, pair_count(t, &scene), 1);
		scene_dispose(t, &scene);
	}
	testing.expect(t, failures > 0 && successes > 0);
}
