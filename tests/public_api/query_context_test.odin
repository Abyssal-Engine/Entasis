package public_api_tests

import "core:testing"
import "core:thread"
import "base:runtime"
import e "entasis:entasis"
import cooking "entasis:entasis_cooking"
import util "entasis:entasis_utilities"

Context_Filter_Trace :: struct
{
	count: int, ids: [16]u32,
}

context_trace_filter :: proc "contextless" (raw: rawptr, collidable: e.Collidable_Reference) -> bool
{
	trace: ^Context_Filter_Trace = cast(^Context_Filter_Trace)raw;
	if trace.count < len(trace.ids)
	{
		trace.ids[trace.count] = collidable.packed;
	}
	trace.count += 1;
	return true;
}

@(test)
query_context_matches_scalar_capacity_misses_filters_and_batches :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	ctx: e.Query_Context;
	if !testing.expect_value(t, e.query_context_init(&ctx, &scene.world), e.Status.Ok)
	{
		return;
	}
	defer e.query_context_destroy(&ctx);
	ray: e.Ray = e.ray({-10, 0, 0}, {1, 0, 0}, 30);
	for y in ([2]f32{0, 100})
	{
		value: e.Ray = ray;
		value.origin.y = y;
		a: e.Ray_Hit;
		sa: e.Status;
		a, sa = e.ray_cast_closest(&scene.world, value);
		b: e.Ray_Hit;
		sb: e.Status;
		b, sb = e.ray_cast_closest_with_context(&ctx, value);
		testing.expect_value(t, sb, sa);
		testing.expect_value(t, b, a);
	}
	invalid: e.Ray = ray;
	invalid.maximum_t = -1;
	a: e.Ray_Hit;
	sa: e.Status;
	a, sa = e.ray_cast_closest(&scene.world, invalid);
	b: e.Ray_Hit;
	sb: e.Status;
	b, sb = e.ray_cast_closest_with_context(&ctx, invalid);
	testing.expect_value(t, sb, sa);
	testing.expect_value(t, b, a);
	for capacity in ([3]int{0, 1, 8})
	{
		trace_a, trace_b: Context_Filter_Trace;
		filter_a: e.Query_Filter = e.Query_Filter{allow=context_trace_filter, user_context=&trace_a};
		filter_b: e.Query_Filter = e.Query_Filter{allow=context_trace_filter, user_context=&trace_b};
		rays_a, rays_b: [8]e.Ray_Hit;
		na: int;
		ra: e.Status;
		na, ra = e.ray_cast_all(&scene.world, ray, rays_a[:capacity], filter_a);
		nb: int;
		rb: e.Status;
		nb, rb = e.ray_cast_all_with_context(&ctx, ray, rays_b[:capacity], filter_b);
		testing.expect_value(t, nb, na);
		testing.expect_value(t, rb, ra);
		testing.expect_value(t, trace_b, trace_a);
		for i in 0..<na
		{
			testing.expect_value(t, rays_b[i], rays_a[i]);
		}
		for x in ([2]f32{-5, 0})
		{
			trace_a = {};
			trace_b = {};
			hits_a, hits_b: [8]e.Sweep_Hit;
			pose: e.Rigid_Pose = e.pose({x, 0, 0});
			velocity: e.Body_Velocity = e.velocity({1, 0, 0});
			sa_count: int;
			sa_status: e.Status;
			sa_count, sa_status = e.sweep_all(&scene.world, scene.sphere_shape, pose, velocity, 20, hits_a[:capacity], filter_a);
			sb_count: int;
			sb_status: e.Status;
			sb_count, sb_status = e.sweep_all_with_context(&ctx, scene.sphere_shape, pose, velocity, 20, hits_b[:capacity], filter_b);
			testing.expect_value(t, sb_status, sa_status);
			testing.expect_value(t, sb_count, sa_count);
			testing.expect_value(t, trace_b, trace_a);
			for i in 0..<sa_count
			{
				testing.expect_value(t, hits_b[i], hits_a[i]);
			}
		}
	}
	queries: [8]e.Query = [8]e.Query{
		e.query_ray_any(ray), e.query_ray_closest(ray),
		e.query_ray_all(ray, e.query_output(0, 8)),
		e.query_sweep_closest(scene.sphere_shape, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 20),
		e.query_overlap_all(scene.sphere_shape, e.pose({0.5, 0, 0}), e.query_output(0, 8)),
		e.query_volume_all({min={-2, -2, -2}, max={2, 2, 2}}, e.query_output(0, 8)),
		e.query_ray_closest(e.ray({0, 100, 0}, {1, 0, 0}, 30)), e.query_ray_closest(invalid),
	};
	results_a, results_b: [8]e.Query_Result;
	rays_a, rays_b: [8]e.Ray_Hit;
	overlaps_a, overlaps_b: [8]e.Overlap_Hit;
	volumes_a, volumes_b: [8]e.Volume_Hit;
	scratch_a: e.Query_Scratch = e.Query_Scratch{ray_hits=rays_a[:], overlap_hits=overlaps_a[:], volume_hits=volumes_a[:]};
	scratch_b: e.Query_Scratch = e.Query_Scratch{ray_hits=rays_b[:], overlap_hits=overlaps_b[:], volume_hits=volumes_b[:]};
	status_a: e.Status = e.query_batch(&scene.world, queries[:], results_a[:], &scratch_a);
	status_b: e.Status = e.query_batch_with_context(&ctx, queries[:], results_b[:], &scratch_b);
	testing.expect_value(t, status_b, status_a);
	for result, i in results_a
	{
		testing.expect_value(t, results_b[i].status, result.status);
		testing.expect_value(t, results_b[i].hit, result.hit);
		testing.expect_value(t, results_b[i].count, result.count);
		if queries[i].kind == .Ray_Closest
		{
			testing.expect_value(t, results_b[i].ray_hit, result.ray_hit);
		}
		if queries[i].kind == .Sweep_Closest
		{
			testing.expect_value(t, results_b[i].sweep_hit, result.sweep_hit);
		}
	}
	testing.expect_value(t, rays_b, rays_a);
	testing.expect_value(t, overlaps_b, overlaps_a);
	testing.expect_value(t, volumes_b, volumes_a);
	pairs: [4]e.Collision_Query = [4]e.Collision_Query{
		{shape_a=scene.sphere_shape, shape_b=scene.box_shape, pose_a=e.pose(), pose_b=e.pose({0.5, 0, 0})},
		{shape_a=scene.sphere_shape, shape_b=scene.box_shape, pose_a=e.pose(), pose_b=e.pose({50, 0, 0})},
		{shape_a=scene.sphere_shape, shape_b=scene.box_shape, pose_a=e.pose(), pose_b=e.pose(), speculative_margin=-1},
		{shape_a=scene.sphere_shape, shape_b=scene.box_shape, pose_a=e.pose(), pose_b=e.pose({0.25, 0, 0})},
	};
	pair_a, pair_b: [4]e.Collision_Query_Result;
	testing.expect_value(t, e.collision_query_batch_with_context(&ctx, pairs[:], pair_b[:]), e.collision_query_batch(&scene.world, pairs[:], pair_a[:]));
	testing.expect_value(t, pair_b, pair_a);
	testing.expect_value(t, pair_a[0].status, e.Status.Ok);
	testing.expect_value(t, pair_a[1].status, e.Status.Not_Found);
	testing.expect_value(t, pair_a[2].status, e.Status.Invalid_Argument);
	testing.expect_value(t, pair_a[3].status, e.Status.Ok);
}

@(test)
query_context_lifetime_reservation_and_borrowed_pool_ownership :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	ctx, other: e.Query_Context;
	description: e.Query_Context_Description = e.query_context_description_default();
	testing.expect_value(t, e.query_context_init(&ctx, &scene.world, description, &scene.pool), e.Status.Invalid_Argument);
	pool: e.Buffer_Pool;
	if !testing.expect_value(t, e.buffer_pool_init(&pool, 4096), e.Status.Ok)
	{
		return;
	}
	defer e.buffer_pool_destroy(&pool);
	if !testing.expect_value(t, e.query_context_init(&ctx, &scene.world, description, &pool), e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.query_context_init(&other, &scene.world, description, &pool), e.Status.Invalid_Argument);
	testing.expect_value(t, e.world_destroy(&scene.world), e.Status.Invalid_Argument);
	invalid: e.Query_Context_Description = description;
	invalid.pair_capacity = 0;
	testing.expect_value(t, e.query_context_reserve(&ctx, invalid), e.Status.Invalid_Description);
	description.child_capacity *= 2;
	testing.expect_value(t, e.query_context_reserve(&ctx, description), e.Status.Ok);
	testing.expect_value(t, e.world_clear(&scene.world), e.Status.Ok);
	hit: e.Query_Hit_State;
	status: e.Status;
	hit, status = e.ray_cast_any_with_context(&ctx, e.ray({-10, 0, 0}, {1, 0, 0}, 30));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, hit, e.Query_Hit_State.Miss);
	shape: e.Shape_Handle;
	shape_status: e.Status;
	shape, shape_status = e.shape_add(&scene.world, e.sphere(1));
	testing.expect_value(t, shape_status, e.Status.Ok);
	add_status: e.Status;
	_, add_status = e.static_add(&scene.world, e.static_body(shape, e.pose()), .None);
	testing.expect_value(t, add_status, e.Status.Ok);
	hit, status = e.ray_cast_any_with_context(&ctx, e.ray({-10, 0, 0}, {1, 0, 0}, 30));
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, hit, e.Query_Hit_State.Hit);
	testing.expect_value(t, e.query_context_destroy(&ctx), e.Status.Ok);
	testing.expect_value(t, ctx, e.Query_Context(nil));
	testing.expect(t, pool.state == .Ready);
	testing.expect_value(t, e.query_context_destroy(&ctx), e.Status.Disposed);
	buffer: util.Buffer(u8);
	take_status: util.Memory_Status;
	buffer, take_status = util.buffer_pool_take_at_least(&pool, u8, 128);
	testing.expect_value(t, take_status, util.Memory_Status.Ok);
	testing.expect_value(t, util.buffer_pool_return(&pool, &buffer), util.Memory_Status.Ok);
}

Context_Concurrent_Call :: struct
{
	ctx: e.Query_Context,
	query_shape, target: e.Shape_Handle,
	failures: int,
}

context_concurrent_entry :: proc(host: ^thread.Thread)
{
	context = runtime.default_context();
	call: ^Context_Concurrent_Call = cast(^Context_Concurrent_Call)host.data;
	for _ in 0..<32
	{
		ray: e.Ray_Hit;
		ray_status: e.Status;
		ray, ray_status = e.ray_cast_closest_with_context(&call.ctx, e.ray({0, 4, 0}, {0, -1, 0}, 8));
		if ray_status != .Ok || ray.t < 0 || ray.t > 4
		{
			call.failures += 1;
		}
		sweep: e.Sweep_Hit;
		sweep_status: e.Status;
		sweep, sweep_status = e.sweep_closest_with_context(&call.ctx, call.query_shape, e.pose({0, 4, 0}), e.velocity({0, -5, 0}), 1);
		if sweep_status != .Ok || sweep.sweep.t1 < 0 || sweep.sweep.t1 > 1
		{
			call.failures += 1;
		}
		overlaps: [8]e.Overlap_Hit;
		count: int;
		overlap_status: e.Status;
		count, overlap_status = e.overlap_all_with_context(&call.ctx, call.query_shape, e.pose({0, 0.25, 0}), overlaps[:]);
		if overlap_status != .Ok || count != 1
		{
			call.failures += 1;
		}
		presence: e.Overlap_State;
		presence_status: e.Status;
		presence, presence_status = e.overlap_any_with_context(&call.ctx, call.query_shape, e.pose({0, 0.25, 0}));
		if presence_status != .Ok || presence != .Intersecting
		{
			call.failures += 1;
		}
		collision_status: e.Status;
		_, collision_status = e.collision_query_with_context(&call.ctx, call.query_shape, e.pose({0, 0.25, 0}), call.target, e.pose());
		if collision_status != .Ok
		{
			call.failures += 1;
		}
	}
}

@(test)
query_context_concurrent_convex_compound_mesh_and_native_custom_routes :: proc(t: ^testing.T)
{
	for route in 0..<4
	{
		world: e.World;
		description: e.World_Description = small_world_description();
		description.gravity = {};
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		query_shape: e.Shape_Handle;
		status: e.Status;
		query_shape, status = e.shape_add(&world, e.sphere(0.5));
		testing.expect_value(t, status, e.Status.Ok);
		target: e.Shape_Handle;
		switch route
		{
			case 0: target, status = e.shape_add(&world, e.sphere(1));
			case 1:
			child: e.Shape_Handle;
			child_status: e.Status;
			child, child_status = e.shape_add(&world, e.sphere(0.7));
			testing.expect_value(t, child_status, e.Status.Ok);
			children: [2]e.Compound_Child = [2]e.Compound_Child{e.compound_child(child, e.pose({-0.4, 0, 0})), e.compound_child(child, e.pose({0.4, 0, 0}))};
			target, status = e.shape_import_compound(&world, children[:]);
			case 2:
			cook: cooking.Cooking_Context;
			testing.expect_value(t, cooking.cooking_context_init(&cook), e.Status.Ok);
			triangles: [2]e.Triangle = [2]e.Triangle{e.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}), e.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2})};
			mesh: cooking.Cooked_Mesh;
			cook_status: e.Status;
			mesh, cook_status = cooking.cook_mesh(&cook, triangles[:]);
			testing.expect_value(t, cook_status, e.Status.Ok);
			target, status = cooking.cooked_mesh_import(&world, &mesh);
			testing.expect_value(t, cooking.cooked_mesh_destroy(&mesh), e.Status.Ok);
			testing.expect_value(t, cooking.cooking_context_destroy(&cook), e.Status.Ok);
			case 3:
			type_id: e.Shape_Type_ID;
			ok: bool;
			type_id, ok = register_custom_sphere(t, &world);
			if !ok
			{
				_ = e.world_destroy(&world);
				return;
			}
			value: Custom_Sphere = Custom_Sphere{radius=1};
			target, status = e.custom_shape_add(&world, type_id, &value);
		}
		testing.expect_value(t, status, e.Status.Ok);
		_, status = e.static_add(&world, e.static_body(target, e.pose()), .None);
		testing.expect_value(t, status, e.Status.Ok);
		calls: [4]Context_Concurrent_Call;
		hosts: [4]^thread.Thread;
		for &call, i in calls
		{
			call.query_shape = query_shape;
			call.target = target;
			testing.expect_value(t, e.query_context_init(&call.ctx, &world), e.Status.Ok);
			hosts[i] = thread.create(context_concurrent_entry);
			hosts[i].data = &call;
		}
		for host in hosts
		{
			thread.start(host);
		}
		for host in hosts
		{
			thread.join(host);
			thread.destroy(host);
		}
		for &call in calls
		{
			testing.expect_value(t, call.failures, 0);
			testing.expect_value(t, e.query_context_destroy(&call.ctx), e.Status.Ok);
		}
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}

@(test)
query_context_equal_and_smaller_reservations_preserve_borrowed_pool_storage :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	for scope in ([2]util.Allocation_Scope{.Legacy, .All_Owned})
	{
		pool: e.Buffer_Pool;
		status: util.Memory_Status = util.buffer_pool_initialize_internal(&pool, 16384, 16, runtime.heap_allocator(), scope);
		if !testing.expect_value(t, status, util.Memory_Status.Ok)
		{
			return;
		}
		ctx: e.Query_Context;
		description: e.Query_Context_Description = e.query_context_description_default();
		if !testing.expect_value(t, e.query_context_init(&ctx, &scene.world, description, &pool), e.Status.Ok)
		{
			_ = e.buffer_pool_destroy(&pool);
			return;
		}
		snapshot: [util.BUFFER_POOL_POWER_COUNT]util.Power_Pool = pool.pools;
		smaller: e.Query_Context_Description = description;
		smaller.pair_capacity = 1;
		smaller.child_capacity = 1;
		smaller.traversal_maximum_power = 8;
		smaller.traversal_slots_per_power = 1;
		for _ in 0..<4
		{
			testing.expect_value(t, e.query_context_reserve(&ctx, smaller), e.Status.Ok);
			testing.expect_value(t, e.query_context_reserve(&ctx, description), e.Status.Ok);
		}
		// no new slots, returned slots, blocks or metadata arrays on either path
		testing.expect_value(t, pool.pools, snapshot);
		testing.expect_value(t, e.query_context_destroy(&ctx), e.Status.Ok);
		testing.expect_value(t, pool.state, util.Pool_State.Ready);
		testing.expect_value(t, e.buffer_pool_destroy(&pool), e.Status.Ok);
	}
}

@(test)
query_context_reuses_rounded_capacity_without_allocation :: proc(t: ^testing.T)
{
	scene: Query_Test_Scene;
	if !query_test_scene_init(t, &scene)
	{
		return;
	}
	defer query_test_scene_destroy(&scene);
	state: allocation_test_tracker;
	pool: e.Buffer_Pool;
	if !testing.expect_value(t, e.buffer_pool_init_with_allocator(&pool, allocation_test_allocator(&state), 16384), e.Status.Ok)
	{
		return;
	}
	ctx: e.Query_Context;
	description: e.Query_Context_Description = e.Query_Context_Description{3, 17, 8, 2};
	if !testing.expect_value(t, e.query_context_init(&ctx, &scene.world, description, &pool), e.Status.Ok)
	{
		_ = e.buffer_pool_destroy(&pool);
		return;
	}
	requests: int;
	bytes: int;
	requests, bytes = state.requests, state.live_bytes;
	state.fail_from = requests + 1;
	// both requests fit the rounded backing allocations, but exceed the original
	// logical hints. this must rebind in place instead of acquiring a new group
	description.pair_capacity = 4;
	description.child_capacity = 18;
	testing.expect_value(t, e.query_context_reserve(&ctx, description), e.Status.Ok);
	testing.expect_value(t, state.requests, requests);
	testing.expect_value(t, state.live_bytes, bytes);
	expected: e.Manifold_Result;
	expected_status: e.Status;
	expected, expected_status = e.collision_query(&scene.world, scene.sphere_shape, e.pose(), scene.box_shape, e.pose({0.5, 0, 0}));
	actual: e.Manifold_Result;
	actual_status: e.Status;
	actual, actual_status = e.collision_query_with_context(&ctx, scene.sphere_shape, e.pose(), scene.box_shape, e.pose({0.5, 0, 0}));
	testing.expect_value(t, actual_status, expected_status);
	testing.expect_value(t, actual, expected);
	testing.expect_value(t, e.query_context_destroy(&ctx), e.Status.Ok);
	testing.expect_value(t, e.buffer_pool_destroy(&pool), e.Status.Ok);
	testing.expect_value(t, state.requests, requests);
	allocation_test_empty(t, &state);
}
