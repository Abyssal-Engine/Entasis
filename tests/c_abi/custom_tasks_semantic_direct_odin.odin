package main

import "core:fmt"
import "core:simd"
import cooking "entasis:entasis_cooking"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Payload :: struct
{
	child: e.Shape_Handle, radius: f32, tag: u32
}

State :: struct
{
	disposed: int
}

require :: proc(status: e.Status)
{
	if status != .Ok
	{
		panic(fmt.tprintf("custom shape semantic status: %v", status));
	}
}

bounds :: proc "contextless" (ctx, raw: rawptr, q: ^u.Quaternion, access: e.Shape_Access, out: ^e.Shape_Bounds) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_compute_bounds(access, (^Payload)(raw).child, q^);
	out^ = value;
	return status;
}

inertia :: proc "contextless" (ctx, raw: rawptr, mass: f32, access: e.Shape_Access, out: ^e.Body_Inertia) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_compute_inertia(access, (^Payload)(raw).child, mass);
	out^ = value;
	return status;
}

ray :: proc "contextless" (ctx, raw: rawptr, pose: ^e.Rigid_Pose, input: ^p.Tree_Ray, access: e.Shape_Access, out: ^e.Shape_Ray_Hit) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_ray_test(access, (^Payload)(raw).child, pose^, input^);
	out^ = value;
	return status;
}

support :: proc "contextless" (ctx, raw: rawptr, direction: ^u.Vector3, access: e.Shape_Access, out: ^u.Vector3) -> e.Status
{
	_ = ctx;
	value, status := e.shape_access_support(access, (^Payload)(raw).child, direction^);
	out^ = value;
	return status;
}

dispose :: proc "contextless" (ctx, raw: rawptr, access: e.Shape_Access, pool: ^u.Buffer_Pool) -> e.Status
{
	_, _, _ = raw, access, pool;
	(^State)(ctx).disposed += 1;
	return .Ok;
}

Task_State :: struct
{
	sweep_status, wide_status: e.Status
}

scalar_task :: proc "contextless" (ctx, a, b: rawptr, pa, pb: p.Rigid_Pose, margin: f32, shapes: ^p.Shape_Registry) -> (p.Convex_Contact_Manifold, e.Status)
{
	_ = ctx;
	first := e.sphere((^Payload)(a).radius);
	result, status := p.sphere_pair_test(&first, b, pa, pb, margin, shapes);
	for i in 0..<int(result.count)
	{
		result.contacts[i].feature_id = 73;
	}
	return result, status;
}

wide_task :: proc "contextless" (ctx: rawptr, bundle: ^p.Collision_Convex_Wide_Bundle, shapes: ^p.Shape_Registry, out: ^p.Collision_Wide_Manifold_Result) -> e.Status
{
	_ = shapes;
	if (^Task_State)(ctx).wide_status != .Ok
	{
		return (^Task_State)(ctx).wide_status;
	}
	a, b: p.Sphere_Wide;
	for i in 0..<bundle.count
	{
		_ = p.sphere_wide_write_slot(&a, i, e.sphere((^Payload)(bundle.shape_a[i]).radius));
		_ = p.sphere_wide_write_slot(&b, i, (^p.Sphere)(bundle.shape_b[i])^);
	}
	manifold, status := p.sphere_pair_test_wide(a, b, bundle.speculative_margin, bundle.offset_b, bundle.count);
	manifold.feature_id = u.I32x8(73);
	out.kind = .One_Contact;
	out.one = manifold;
	return status;
}

child_task :: proc "contextless" (ctx, a, b: rawptr, ta, tb: int, pa, pb, la, lb: p.Rigid_Pose, va, vb: p.Body_Velocity,
	maximum_t, progression, convergence: f32, iterations: int, shapes: ^p.Shape_Registry, tasks: ^p.Collision_Task_Registry) -> (p.Sweep_Result, e.Status)
{
	_, _, _, _ = ctx, ta, tb, tasks;
	first := e.sphere((^Payload)(a).radius);
	result, status := p.sweep_task_test_convex_distance_internal(&first, b, p.SPHERE_TYPE_ID, p.SPHERE_TYPE_ID,
		pa, pb, la, lb, va, vb, maximum_t, progression, convergence, iterations, shapes);
	if result.state == .Hit
	{
		result.child_a = 3;
		result.child_b = 7;
	}
	return result, status;
}

sweep_task :: proc "contextless" (ctx, a, b: rawptr, ta, tb: int, pa, pb: p.Rigid_Pose, va, vb: p.Body_Velocity,
	maximum_t, progression, convergence: f32, iterations: int, shapes: ^p.Shape_Registry, tasks: ^p.Collision_Task_Registry,
	filter: p.Collision_Child_Filter_Proc, filter_context: rawptr) -> (p.Sweep_Result, e.Status)
{
	state := (^Task_State)(ctx);
	if state.sweep_status != .Ok
	{
		return {}, state.sweep_status;
	}
	if filter != nil && filter(filter_context, 19, 3, 7) == .Reject
	{
		return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
	}
	return child_task(ctx, a, b, ta, tb, pa, pb, e.pose(), e.pose(), va, vb, maximum_t, progression, convergence, iterations, shapes, tasks);
}

triangle_scalar :: proc "contextless" (ctx, a, b: rawptr, pa, pb: p.Rigid_Pose, margin: f32, shapes: ^p.Shape_Registry) -> (p.Convex_Contact_Manifold, e.Status)
{
	_ = ctx;
	sphere := e.sphere((^Payload)(a).radius);
	return p.sphere_triangle_test(&sphere, b, pa, pb, margin, shapes);
}

triangle_wide :: proc "contextless" (ctx: rawptr, bundle: ^p.Collision_Convex_Wide_Bundle, shapes: ^p.Shape_Registry, out: ^p.Collision_Wide_Manifold_Result) -> e.Status
{
	if (^Task_State)(ctx).wide_status != .Ok
	{
		return (^Task_State)(ctx).wide_status;
	}
	out.kind = .One_Contact;
	for i in 0..<bundle.count
	{
		pa := e.pose();
		pb := e.pose(u.vector3_wide_read_slot(bundle.offset_b, i));
		pa.orientation = u.quaternion_wide_read_slot(bundle.orientation_a, i);
		pb.orientation = u.quaternion_wide_read_slot(bundle.orientation_b, i);
		triangle := p.Triangle{u.vector3_wide_read_slot(bundle.b.triangle.a, i),
			u.vector3_wide_read_slot(bundle.b.triangle.b, i), u.vector3_wide_read_slot(bundle.b.triangle.c, i)};
		result, status := triangle_scalar(ctx, bundle.shape_a[i], &triangle, pa, pb, simd.extract(bundle.speculative_margin, i), shapes);
		if status != .Ok
		{
			return status;
		}
		if result.count > 1
		{
			return .Invalid_Description;
		}
		if result.count == 0
		{
			continue;
		}
		c := result.contacts[0];
		u.vector3_wide_write_slot(&out.one.normal, i, result.normal);
		out.one.contact_exists=simd.replace(out.one.contact_exists, i, -1);
		out.one.depth=simd.replace(out.one.depth, i, c.depth);
		out.one.feature_id=simd.replace(out.one.feature_id, i, c.feature_id);
		u.vector3_wide_write_slot(&out.one.offset_a, i, c.offset);
	}
	return .Ok;
}

print_compound :: proc(target, mode, batch, flipped: int, m: e.Manifold_Result)
{
	fmt.printf("compound target=%d context=%d batch=%d flip=%d kind=%d count=%d offset=%d,%d,%d\n",
		target, mode, batch, flipped, u32(m.kind), m.nonconvex.count, transmute(u32)m.nonconvex.offset_b.x, transmute(u32)m.nonconvex.offset_b.y, transmute(u32)m.nonconvex.offset_b.z);
	for i in 0..<int(m.nonconvex.count)
	{
		c := m.nonconvex.contacts[i];
		fmt.printf("contact index=%d offset=%d,%d,%d normal=%d,%d,%d depth=%d feature=%d\n",
			i, transmute(u32)c.offset.x, transmute(u32)c.offset.y, transmute(u32)c.offset.z,
			transmute(u32)c.normal.x, transmute(u32)c.normal.y, transmute(u32)c.normal.z, transmute(u32)c.depth, c.feature_id);
	}
}

main :: proc()
{
	world: e.World;
	description := e.world_description_default();
	description.threading.worker_count = 1;
	description.gravity = {};
	description.capacity.bodies = 32;
	description.capacity.statics = 16;
	description.capacity.shapes_per_type = 4;
	description.capacity.constraints = 32;
	description.capacity.pairs = 128;
	description.capacity.broad_phase_candidates = 128;
	description.capacity.collision_child_pairs = 256;
	require(e.world_init(&world, description));
	shape_state: State;
	task_state: Task_State;
	registration := e.custom_shape_registration_contextual(Payload, .Convex, &shape_state, bounds, inertia, ray, support, support, dispose);
	registration.alignment = 16;
	type_id, status := e.custom_shape_register_contextual(&world, registration);
	require(status);
	sphere, sphere_status := e.shape_add(&world, e.sphere(1));
	require(sphere_status);
	payload := Payload{child=sphere, radius=1};
	custom, custom_status := e.custom_shape_add_raw(&world, type_id, &payload, size_of(payload));
	require(custom_status);
	_, status = e.collision_task_register_contextual(&world, {shape_type_a=type_id, shape_type_b=e.SHAPE_TYPE_SPHERE,
			batch_size=32, pair_type=.Standard, user_context=&task_state, test=scalar_task, wide_test=wide_task});
	require(status);
	_, status = e.sweep_task_register_contextual(&world, {shape_type_a=type_id, shape_type_b=e.SHAPE_TYPE_SPHERE,
			user_context=&task_state, test=sweep_task, child_test=child_task});
	require(status);
	for flipped in 0..<2
	{
		m: e.Manifold_Result;
		if flipped == 0
		{
			m, status = e.collision_query(&world, custom, e.pose(), sphere, e.pose({0, 1, 0}));
		}
		else
		{
			m, status = e.collision_query(&world, sphere, e.pose({0, 1, 0}), custom, e.pose());
		}
		require(status);
		fmt.printf("collision flip=%d count=%d depth=%d normal=%d feature=%d\n", flipped, m.convex.count,
			transmute(u32)m.convex.contacts[0].depth, transmute(u32)m.convex.normal.y, m.convex.contacts[0].feature_id);
	}
	handle, add_status := e.static_add(&world, e.static_body(custom, e.pose()), .None);
	require(add_status);
	query: e.Query_Context;
	require(e.query_context_init(&query, &world));
	hit: e.Sweep_Hit;
	for mode in 0..<2
	{
		if mode == 0
		{
			hit, status = e.sweep_closest(&world, sphere, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 10);
		}
		else
		{
			hit, status = e.sweep_closest_with_context(&query, sphere, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 10);
		}
		require(status);
		fmt.printf("sweep context=%d t0=%d t1=%d normal=%d child_a=%d child_b=%d\n", mode,
			transmute(u32)hit.sweep.t0, transmute(u32)hit.sweep.t1, transmute(u32)hit.sweep.normal.x, hit.sweep.child_a, hit.sweep.child_b);
		task_state.sweep_status = .Capacity_Missing;
		found: e.Query_Hit_State;
		if mode == 0
		{
			native_hit, native_status := e.sweep_any(&world, sphere, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 10);
			status = native_status;
			found = .Hit if native_hit else .Miss;
		}
		else
		{
			found, status = e.sweep_any_with_context(&query, sphere, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 10);
		}
		if status != .Capacity_Missing || found == .Hit
		{
			panic("lost capacity failure");
		}
		fmt.printf("failure context=%d status=%d found=%d\n", mode, u32(status), int(found == .Hit));
		task_state.sweep_status = .Ok;
	}
	require(e.query_context_destroy(&query));
	require(e.static_remove(&world, handle, .None));
	children := [2]e.Compound_Child{e.compound_child(custom, e.pose({-0.4, 0, 0})), e.compound_child(custom, e.pose({0.4, 0, 0}))};
	compound, compound_status := e.shape_import_compound(&world, children[:]);
	require(compound_status);
	handle, status = e.static_add(&world, e.static_body(compound, e.pose()), .None);
	require(status);
	hit, status = e.sweep_closest(&world, sphere, e.pose({-5, 0, 0}), e.velocity({1, 0, 0}), 10);
	require(status);
	fmt.printf("child t0=%d t1=%d normal=%d child_b=%d\n", transmute(u32)hit.sweep.t0, transmute(u32)hit.sweep.t1, transmute(u32)hit.sweep.normal.x, hit.sweep.child_b);
	require(e.static_remove(&world, handle, .None));
	_, status = e.collision_task_register_contextual(&world, {shape_type_a=type_id, shape_type_b=e.SHAPE_TYPE_TRIANGLE,
			batch_size=32, pair_type=.Standard, user_context=&task_state, test=triangle_scalar, wide_test=triangle_wide});
	require(status);
	for i in 0..<3
	{
		caps := e.Collision_Task_Capabilities{.Subtask_Generator, .Child_Order};
		if i == 2
		{
			caps += {.Mesh_Reduction};
		}
		_, status = e.collision_task_register(&world, e.collision_task_compound(type_id, e.Shape_Type_ID(p.COMPOUND_TYPE_ID+i), 16, .Convex_Compound, caps));
		require(status);
	}
	targets: [3]e.Shape_Handle;
	target_children := [2]e.Compound_Child{e.compound_child(sphere, e.pose({-1, 0, 0})), e.compound_child(sphere, e.pose({1, 0, 0}))};
	targets[0], status = e.shape_import_compound(&world, target_children[:]);
	require(status);
	targets[1], status = e.shape_import_big_compound(&world, target_children[:]);
	require(status);
	cooking_context: cooking.Cooking_Context;
	require(cooking.cooking_context_init(&cooking_context));
	triangles := [8]e.Triangle{
		{{0, 0, 0}, {-4, 0, -4}, {0, 0, -4}}, {{0, 0, 0}, {0, 0, -4}, {4, 0, -4}},
		{{0, 0, 0}, {4, 0, -4}, {4, 0, 0}}, {{0, 0, 0}, {4, 0, 0}, {4, 0, 4}},
		{{0, 0, 0}, {4, 0, 4}, {0, 0, 4}}, {{0, 0, 0}, {0, 0, 4}, {-4, 0, 4}},
		{{0, 0, 0}, {-4, 0, 4}, {-4, 0, 0}}, {{0, 0, 0}, {-4, 0, 0}, {-4, 0, -4}}};
	mesh, mesh_status := cooking.cook_mesh(&cooking_context, triangles[:]);
	require(mesh_status);
	targets[2], status = cooking.cooked_mesh_import(&world, &mesh);
	require(status);
	require(cooking.cooked_mesh_destroy(&mesh));
	require(cooking.cooking_context_destroy(&cooking_context));
	require(e.query_context_init(&query, &world));
	for target in 0..<3
	{
		a := e.pose({0, target == 2 ? f32(0.5) : f32(0), 0});
		b := e.pose();
		pairs := [2]e.Collision_Query{{shape_a=custom, pose_a=a, shape_b=targets[target], pose_b=b},
			{shape_a=targets[target], pose_a=b, shape_b=custom, pose_b=a}};
		for mode in 0..<2
		{
			for flipped in 0..<2
			{
				pair := pairs[flipped];
				m: e.Manifold_Result;
				if mode == 0
				{
					m, status=e.collision_query(&world, pair.shape_a, pair.pose_a, pair.shape_b, pair.pose_b);
				}
				else
				{
					m, status=e.collision_query_with_context(&query, pair.shape_a, pair.pose_a, pair.shape_b, pair.pose_b);
				}
				require(status);
				print_compound(target, mode, 0, flipped, m);
			}
			results: [2]e.Collision_Query_Result;
			if mode == 0
			{
				status=e.collision_query_batch(&world, pairs[:], results[:]);
			}
			else
			{
				status=e.collision_query_batch_with_context(&query, pairs[:], results[:]);
			}
			require(status);
			for flipped in 0..<2
			{
				require(results[flipped].status);
				if !results[flipped].hit
				{
					panic("missing compound batch hit");
				}
				print_compound(target, mode, 1, flipped, results[flipped].manifold);
			}
			task_state.wide_status=.Capacity_Missing;
			if mode == 0
			{
				_, status=e.collision_query(&world, custom, a, targets[target], b);
			}
			else
			{
				_, status=e.collision_query_with_context(&query, custom, a, targets[target], b);
			}
			if status != .Capacity_Missing
			{
				panic("lost compound capacity failure");
			}
			fmt.printf("compound-failure target=%d context=%d status=%d\n", target, mode, u32(status));
			task_state.wide_status=.Ok;
			if mode == 0
			{
				_, status=e.collision_query(&world, custom, a, targets[target], b);
			}
			else
			{
				_, status=e.collision_query_with_context(&query, custom, a, targets[target], b);
			}
			require(status);
		}
	}
	require(e.query_context_destroy(&query));
	require(e.world_destroy(&world));
}
