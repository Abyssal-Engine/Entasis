package precision_broadphase_tests

import "core:testing"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

// test-only closed interval oracle uses authored centers and exact sphere
// extents/motion. it never reads either production bounds representation
oracle_interval_overlap :: proc "contextless" (a, b: p.Global_Bounds) -> p.Reference_State
{
	x_min: f64 = max(a.min.x, b.min.x);
	y_min: f64 = max(a.min.y, b.min.y);
	z_min: f64 = max(a.min.z, b.min.z);
	x_max: f64 = min(a.max.x, b.max.x);
	y_max: f64 = min(a.max.y, b.max.y);
	z_max: f64 = min(a.max.z, b.max.z);
	if x_min <= x_max && y_min <= y_max && z_min <= z_max
	{
		return .Present;
	}
	return .Missing;
}

@(test)
precision_parallel_pair_sets_match_independent_predicted_interval_oracle :: proc(t: ^testing.T)
{
	dispatcher: u.Thread_Dispatcher;
	testing.expect_value(t, u.thread_dispatcher_initialize(&dispatcher, 4, 65536), u.Threading_Status.Ok);
	defer u.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^u.Thread_Dispatcher_Boundary = u.thread_dispatcher_boundary(&dispatcher);
	offsets: [3]f64 = {0, 1e9, -1e9};
	for offset in offsets
	{
		scene: Scene;
		scene_initialize(t, &scene, 256, 4);
		oracle: [160]p.Global_Bounds;
		for index in 0 ..< len(oracle)
		{
			position: p.Global_Position = {offset + f64(index % 8) * 65 / 512,
				-offset + f64(index / 8 % 4) * 65 / 512, offset + f64(index / 32) / 8};
			displacement: f64;
			if index < 128
			{
				_ = add_body(t, &scene, scene.shapes[0], position);
				velocity: f32 = f32(index % 3 - 1) * 0.5;
				scene.simulation.bodies.sets.memory[0].dynamics_state.memory[index].motion.velocity.linear.x = velocity;
				displacement = f64(velocity) / 8;
			}
			else
			{
				_ = add_static(t, &scene, scene.shapes[0], position);
			}
			oracle[index] = {
				min={position.x - 0.0625 + min(f64(0), displacement), position.y - 0.0625, position.z - 0.0625},
				max={position.x + 0.0625 + max(f64(0), displacement), position.y + 0.0625, position.z + 0.0625},
			};
		}
		testing.expect_value(t, p.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125, nil, boundary), p.Physics_Status.Ok);
		testing.expect_value(t, p.broad_phase_update(&scene.simulation.broad_phase, boundary), p.Physics_Status.Ok);
		pairs: [16384]p.Broad_Phase_Pair;
		output: u.Buffer(p.Broad_Phase_Pair) = {memory=&pairs[0], length=i32(len(pairs)), id=-1};
		for pass in 0 ..< 4
		{
			selected: ^u.Thread_Dispatcher_Boundary;
			if pass != 0
			{
				selected = boundary;
			}
			count: int;
			status: p.Physics_Status;
			count, status = p.broad_phase_find_pairs(&scene.simulation.broad_phase, output, selected);
			testing.expect_value(t, status, p.Physics_Status.Ok);
			seen: [128 * 160]u8;
			for pair in pairs[:count]
			{
				a: int = int(p.collidable_reference_raw_handle(pair.a));
				b: int = int(p.collidable_reference_raw_handle(pair.b));
				if p.collidable_reference_mobility(pair.b) == .Static
				{
					b += 128;
				}
				testing.expect(t, a >= 0 && a < 128 && b > a && b < 160);
				testing.expect_value(t, seen[a * 160 + b], u8(0));
				seen[a * 160 + b] = 1;
			}
			expected_count: int;
			for a in 0 ..< 128
			{
				for b in a + 1 ..< 160
				{
					expected: u8 = 1 if oracle_interval_overlap(oracle[a], oracle[b]) == .Present else 0;
					testing.expect_value(t, seen[a * 160 + b], expected);
					expected_count += int(expected);
				}
			}
			testing.expect_value(t, count, expected_count);
		}
		scene_dispose(t, &scene);
	}
}

@(test)
precision_prepared_pair_filter_and_parallel_reads_do_not_allocate :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 64, 4);
	defer scene_dispose(t, &scene);
	for index in 0 ..< 36
	{
		_ = add_body(t, &scene, scene.shapes[0], {1e9 + f64(index) * 65 / 512, -1e9, 1e9});
		_ = add_static(t, &scene, scene.shapes[0], {1e9 + f64(index) * 65 / 512, -1e9, 1e9});
	}
	dispatcher: u.Thread_Dispatcher;
	testing.expect_value(t, u.thread_dispatcher_initialize(&dispatcher, 4, 65536), u.Threading_Status.Ok);
	defer u.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^u.Thread_Dispatcher_Boundary = u.thread_dispatcher_boundary(&dispatcher);
	pairs: [4096]p.Broad_Phase_Pair;
	output: u.Buffer(p.Broad_Phase_Pair) = {memory=&pairs[0], length=i32(len(pairs)), id=-1};
	count: int;
	status: p.Physics_Status;
	count, status = p.broad_phase_find_pairs(&scene.simulation.broad_phase, output, boundary);
	testing.expect_value(t, status, p.Physics_Status.Ok);
	testing.expect_value(t, count, 36);
	allocations: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	dispatcher_allocations: u64 = dispatcher.general_allocator_call_count;
	for iteration in 0 ..< 1024
	{
		selected: ^u.Thread_Dispatcher_Boundary = boundary;
		if iteration & 1 == 0
		{
			selected = nil;
		}
		count, status = p.broad_phase_find_pairs(&scene.simulation.broad_phase, output, selected);
		testing.expect_value(t, status, p.Physics_Status.Ok);
		testing.expect_value(t, count, 36);
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, allocations);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
	testing.expect_value(t, dispatcher.general_allocator_call_count, dispatcher_allocations);
}

@(test)
precision_pair_filter_retains_angular_reach_beyond_static_geometry :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene, 16, 1);
	defer scene_dispose(t, &scene);
	origin: p.Global_Position = {1e9, -1e9, 1e9};
	_ = add_body(t, &scene, scene.shapes[2], origin);
	scene.simulation.bodies.sets.memory[0].dynamics_state.memory[0].motion.velocity.angular = {0, 16, 0};
	scene.simulation.bodies.sets.memory[0].collidables.memory[0].continuity.mode = .Continuous;
	// the long box can rotate into x=0.25. its initial x half-extent is 0.0625
	_ = add_static(t, &scene, scene.shapes[0], {origin.x + 0.25, origin.y, origin.z});
	_ = add_static(t, &scene, scene.shapes[0], {origin.x - 0.25, origin.y, origin.z});
	testing.expect_value(t, pair_count(t, &scene), 0);
	testing.expect_value(t, p.broad_phase_predict_bounds(&scene.simulation.broad_phase, 0.125), p.Physics_Status.Ok);
	testing.expect_value(t, pair_count(t, &scene), 2);
}
