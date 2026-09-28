package main

import "core:fmt"
import "base:runtime"
import "core:simd"
import "core:math"
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
	_, _ = ctx, shapes;
	out.kind = .One_Contact;
	for i in 0..<bundle.count
	{
		offset := u.vector3_wide_read_slot(bundle.offset_b, i);
		x, y, z := offset.x, offset.y, offset.z;
		distance := math.sqrt(x*x+y*y+z*z);
		a := (^Payload)(bundle.shape_a[i]).radius;
		b := (^p.Sphere)(bundle.shape_b[i]).radius;
		depth := a+b-distance;
		if depth < -simd.extract(bundle.speculative_margin, i)
		{
			continue;
		}
		nx, ny, nz:f32;
		if distance > 0
		{
			nx=-x/distance;
			ny=-y/distance;
			nz=-z/distance;
		}
		else
		{
			ny=1;
		}
		u.vector3_wide_write_slot(&out.one.normal, i, {nx, ny, nz});
		out.one.contact_exists=simd.replace(out.one.contact_exists, i, -1);
		out.one.depth=simd.replace(out.one.depth, i, depth);
		out.one.feature_id=simd.replace(out.one.feature_id, i, 73);
		u.vector3_wide_write_slot(&out.one.offset_a, i, {(x+(b-a)*nx)*0.5, (y+(b-a)*ny)*0.5, (z+(b-a)*nz)*0.5});
	}
	return .Ok;
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

// independent native callback representation, not the new contextual/C bridge
validate :: proc "contextless" (id: i32, raw: rawptr) -> e.Status
{
	if id < 56 || raw == nil
	{
		return .Invalid_Argument;
	}
	value := (^f32)(raw)^;
	if value != value || value < -10000 || value > 10000
	{
		return .Invalid_Description;
	}
	return .Ok;
}

execute :: #force_inline proc "contextless" (prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	impulses_raw: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase, $arity: int)
{
	if phase != .Solve
	{
		return;
	}
	prestep := (^e.F32x8)(prestep_raw);
	impulses := (^e.F32x8)(impulses_raw);
	for lane in 0 ..< 8
	{
		if simd.extract(mask, lane) == 0
		{
			continue;
		}
		for body in 0 ..< arity
		{
			if simd.extract(bodies[body].inverse_mass, lane) > 0
			{
				bodies[body].linear_velocity.x = simd.replace(bodies[body].linear_velocity.x, lane,
					simd.extract(prestep^, lane)+f32(body));
			}
		}
		impulses^ = simd.replace(impulses^, lane, simd.extract(prestep^, lane));
	}
}

kernel1 :: proc "contextless" (prestep: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase)
{
	_, _ = dt, inverse_dt;
	execute(prestep, bodies, impulses, mask, phase, 1);
}

Integration :: struct
{
	initialized, disposed, completed, scheduled, velocity, contacts, child_contacts: int
}

initialize :: proc "contextless" (ctx: rawptr, sim: ^p.Simulation) -> e.Status
{
	_=sim;
	(^Integration)(ctx).initialized+=1;
	return .Ok;
}

finish :: proc "contextless" (ctx: rawptr)
{
	(^Integration)(ctx).disposed+=1;
}

prepare :: proc "contextless" (ctx: rawptr, dt:f32) -> e.Status
{
	_=ctx;
	return .Ok if dt>0 else .Invalid_Argument;
}

integrate :: proc "contextless" (ctx:rawptr, indices:u.I32x8, position:u.Vector3_Wide, orientation:u.Quaternion_Wide,
	inertia:p.Body_Inertia_Wide, mask:u.I32x8, worker:int, dt:u.F32x8, velocity:^p.Body_Velocity_Wide)
{
	_, _, _, _, _, _=indices, position, orientation, inertia, worker, dt;
	(^Integration)(ctx).velocity+=1;
	for i in 0..<8
	{
		if simd.extract(mask, i)!=0
		{
			velocity.linear.z=simd.replace(velocity.linear.z, i, 0.25);
		}
	}
}

allow :: proc "contextless" (ctx:rawptr, worker:int, a, b:e.Collidable_Reference, margin:^f32) -> e.Collision_Testing_State
{
	_, _, _, _=ctx, worker, a, b;
	margin^=0.125;
	return .Allow;
}

allow_child :: proc "contextless" (ctx:rawptr, worker:int, a, b:e.Collidable_Reference, ca, cb:int) -> e.Collision_Testing_State
{
	_, _, _, _, _, _=ctx, worker, a, b, ca, cb;
	return .Allow;
}

configure :: proc "contextless" (ctx:rawptr, worker:int, a, b:e.Collidable_Reference, m:^e.Manifold_Result, material:^e.Contact_Material) -> e.Collision_Testing_State
{
	_, _, _=worker, a, b;
	(^Integration)(ctx).contacts+=1;
	material^=e.contact_material_default();
	material.friction_coefficient=0;
	material.maximum_recovery_velocity=0;
	if m.kind==.Convex
	{
		for i in 0..<int(m.convex.count)
		{
			m.convex.contacts[i].depth=0;
		}
	}
	else
	{
		for i in 0..<int(m.nonconvex.count)
		{
			m.nonconvex.contacts[i].depth=0;
		}
	}
	return .Allow;
}

configure_child :: proc "contextless" (ctx:rawptr, worker:int, a, b:e.Collidable_Reference, ca, cb:int, m:^p.Convex_Contact_Manifold) -> e.Collision_Testing_State
{
	_, _, _, _, _=worker, a, b, ca, cb;
	(^Integration)(ctx).child_contacts+=1;
	for i in 0..<int(m.count)
	{
		m.contacts[i].feature_id=91;
	}
	return .Allow;
}

completed :: proc "contextless" (ctx:rawptr, stage:p.Timestep_Completion_Stage, dt:f32, dispatcher:^u.Thread_Dispatcher_Boundary) -> e.Status
{
	_=dispatcher;
	s:=(^Integration)(ctx);
	if int(stage)!=s.completed%4 || dt!=1.0/64.0
	{
		return .Invalid_Argument;
	}
	s.completed+=1;
	return .Ok;
}

schedule :: proc "contextless" (ctx:rawptr, index:int) -> i32
{
	s:=(^Integration)(ctx);
	if index!=s.scheduled%2
	{
		return 0;
	}
	s.scheduled+=1;
	return 3;
}

step :: proc "contextless" (ctx:rawptr, sim:^p.Simulation, dt:f32, dispatcher:^u.Thread_Dispatcher_Boundary) -> e.Status
{
	_=ctx;
	context=runtime.default_context();
	status:=p.simulation_sleep(sim, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.timestep_completion_callback(sim, .Slept, .Slept_Callback, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_predict_bounding_boxes(sim, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.timestep_completion_callback(sim, .Before_Collision_Detection, .Before_Collision_Callback, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_collision_detection(sim, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.timestep_completion_callback(sim, .Collisions_Detected, .Collisions_Detected_Callback, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_solve(sim, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.timestep_completion_callback(sim, .Constraints_Solved, .Constraints_Solved_Callback, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_incrementally_optimize_data_structures(sim, dispatcher);
	if status==.Ok
	{
		sim.step_index+=1;
	}
	return status;
}

run :: proc(scope:e.Allocation_Scope, custom_step:int)
{
	world:e.World;
	s:Integration;
	shape_state:State;
	task_state:Task_State;
	d:=e.world_description_default();
	d.threading.worker_count=1;
	d.gravity={};
	d.solve.substeps=2;
	d.solve.velocity_iterations=3;
	d.capacity.bodies=32;
	d.capacity.statics=16;
	d.capacity.shapes_per_type=4;
	d.capacity.constraints=32;
	d.capacity.pairs=128;
	d.capacity.broad_phase_candidates=128;
	d.capacity.collision_child_pairs=256;
	d.pose_callbacks={initialize=initialize, prepare_for_integration=prepare, integrate_velocity=integrate, dispose=finish, user_context=&s};
	d.narrow_callbacks={initialize=initialize, allow=allow, allow_child=allow_child, configure=configure, configure_child=configure_child, dispose=finish, user_context=&s};
	d.timestep_callbacks={stage_completed=completed, user_context=&s};
	d.solve.velocity_iteration_scheduler=schedule;
	d.solve.scheduler_context=&s;
	if custom_step!=0
	{
		d.timestepper={step=step, user_context=&s};
	}
	require(e.world_init_with_allocation_scope(&world, d, scope));
	sphere, status:=e.shape_add(&world, e.sphere(1));
	require(status);
	r:=e.custom_shape_registration_contextual(Payload, .Convex, &shape_state, bounds, inertia, ray, support, support, dispose);
	r.alignment=16;
	type_id, register_status:=e.custom_shape_register_contextual(&world, r);
	require(register_status);
	payload:=Payload{sphere, 1, 17};
	custom, custom_status:=e.custom_shape_add_raw(&world, type_id, &payload, size_of(payload));
	require(custom_status);
	_, status=e.collision_task_register_contextual(&world, {shape_type_a=type_id, shape_type_b=e.SHAPE_TYPE_SPHERE, batch_size=32, pair_type=.Standard, user_context=&task_state, test=scalar_task, wide_test=wide_task});
	require(status);
	_, status=e.sweep_task_register_contextual(&world, {shape_type_a=type_id, shape_type_b=e.SHAPE_TYPE_SPHERE, user_context=&task_state, test=sweep_task, child_test=child_task});
	require(status);
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
	initial, solve:[4]e.Body_Access_Mask;
	initial[0]=e.BODY_ACCESS_ALL;
	solve[0]={.Mass, .Linear_Velocity};
	cr:=e.custom_constraint_registration(f32, e.F32x8, e.F32x8, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, 1, initial, solve, validate, kernel1);
	require(e.custom_constraint_register(&world, cr));
	body_inertia, inertia_status:=e.custom_shape_inertia(&world, custom, 1);
	require(inertia_status);
	bodies:[9]e.Body_Handle;
	constraints:[9]e.Constraint_Handle;
	for i in 0..<9
	{
		bodies[i], status=e.body_add(&world, e.body_dynamic(custom, body_inertia, e.pose({f32(i)*4, 1.5, 0}), e.velocity(), e.body_activity(-1, 255)));
		require(status);
		target_shape:=sphere;
		if i==0
		{
			target_shape=targets[0];
		}
		_, status=e.static_add(&world, e.static_body(target_shape, e.pose({f32(i)*4, 0, 0})), .None);
		require(status);
		target:=f32(i+1)*0.125;
		constraints[i], status=e.custom_constraint_add(&world, bodies[i:i+1], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &target, 4);
		require(status);
	}
	_, status=e.static_add(&world, e.static_body(custom, e.pose({50, 0, 0})), .None);
	require(status);
	children:=[2]e.Compound_Child{e.compound_child(custom, e.pose({-0.5, 0, 0})), e.compound_child(custom, e.pose({0.5, 0, 0}))};
	compound, compound_status:=e.shape_import_compound(&world, children[:]);
	require(compound_status);
	_, status=e.static_add(&world, e.static_body(compound, e.pose({60, 0, 0})), .None);
	require(status);
	query:e.Query_Context;
	require(e.query_context_init(&query, &world));
	for _ in 0..<3
	{
		require(e.world_step(&world, 1.0/64.0));
	}
	m, mstatus:=e.collision_query_with_context(&query, custom, e.pose(), sphere, e.pose({0, 1, 0}));
	require(mstatus);
	rh, rstatus:=e.ray_cast_closest_with_context(&query, {origin={45, 0, 0}, direction={1, 0, 0}, maximum_t=8});
	require(rstatus);
	sh, sstatus:=e.sweep_closest_with_context(&query, sphere, e.pose({55, 0, 0}), e.velocity({1, 0, 0}), 10);
	require(sstatus);
	fmt.printf("integrated scope=%d custom=%d ray=%08x sweep=%08x child=%d contact=%08x\n", int(scope), custom_step, transmute(u32)rh.t, transmute(u32)sh.sweep.t1, sh.sweep.child_b, transmute(u32)m.convex.contacts[0].depth);
	for i in 0..<9
	{
		b, bstatus:=e.body_get(&world, bodies[i]);
		require(bstatus);
		impulses:[1]f32;
		written, required, impulse_status:=e.constraint_accumulated_impulses(&world, constraints[i], impulses[:]);
		require(impulse_status);
		if written!=1||required!=1
		{
			panic("impulse coverage");
		}
		fmt.printf("body=%d p=%08x,%08x,%08x v=%08x,%08x,%08x impulse=%08x\n", i, transmute(u32)b.pose.position.x, transmute(u32)b.pose.position.y, transmute(u32)b.pose.position.z, transmute(u32)b.velocity.linear.x, transmute(u32)b.velocity.linear.y, transmute(u32)b.velocity.linear.z, transmute(u32)impulses[0]);
	}
	stats, stats_status:=e.world_stats(&world);
	require(stats_status);
	if stats.step_index!=3 || s.initialized!=2 || s.completed!=12 || s.scheduled!=6 || s.contacts==0 || s.child_contacts==0 || s.velocity==0
	{
		panic("missing integrated callback path");
	}
	require(e.query_context_destroy(&query));
	require(e.world_destroy(&world));
	if s.disposed!=2 || shape_state.disposed!=1
	{
		panic("callback lifecycle");
	}
	fmt.printf("completed=%d scheduled=%d disposed=%d\n", s.completed, s.scheduled, s.disposed);
}

restitution_semantic :: proc(scope:e.Allocation_Scope, iterations:i32)
{
	world:e.World;
	description:=e.world_description_default();
	description.gravity={0, -6, 0};
	description.damping={};
	description.threading.worker_count=2;
	description.solve.velocity_iterations=iterations;
	require(e.world_init_with_allocation_scope(&world, description, scope));
	sphere:=e.sphere(1);
	shape, status:=e.shape_add(&world, sphere);
	require(status);
	inertia, inertia_status:=e.shape_inertia(sphere, 1);
	require(inertia_status);
	config:=e.restitution_configuration_default();
	config.fallback.coefficient=0.5;
	require(e.world_enable_restitution(&world, config));
	bodies:[9]e.Body_Handle;
	for i in 0..<9
	{
		_, status=e.static_add(&world, e.static_body(shape, e.pose({f32(i)*4, 0, 0})), .None);
		require(status);
		bodies[i], status=e.body_add(&world, e.body_dynamic(shape, inertia, e.pose({f32(i)*4, 2, 0}), e.velocity({0, -10, 0}), e.body_activity(-1, 255)));
		require(status);
	}
	require(e.world_step(&world, 1.0/64));
	fmt.printf("restitution scope=%d iterations=%d\n", int(scope), iterations);
	for i in 0..<9
	{
		body, read_status:=e.body_get(&world, bodies[i]);
		require(read_status);
		if abs(body.velocity.linear.y-5.046875)>0.0002
		{
			panic("restitution semantic target");
		}
		fmt.printf("bounce=%d y=%08x vy=%08x\n", i, transmute(u32)body.pose.position.y, transmute(u32)body.velocity.linear.y);
	}
	require(e.world_destroy(&world));
}

body_control_semantic :: proc (scope: e.Allocation_Scope, substeps: i32)
{
	world: e.World;
	desc:=e.world_description_default();
	desc.gravity={};
	desc.damping={};
	desc.threading.worker_count=2;
	desc.solve.substeps=substeps;
	require(e.world_init_with_allocation_scope(&world, desc, scope));
	require(e.world_enable_body_control(&world));
	shape, status:=e.shape_add(&world, e.sphere(1));
	require(status);
	inertia, mass_status:=e.shape_inertia(e.sphere(1), 2);
	require(mass_status);
	body, add_status:=e.body_add(&world, e.body_dynamic(shape, inertia, e.pose(), {}, e.body_activity(-1, 255)));
	require(add_status);
	require(e.body_add_force(&world, body, {8, 0, 0}));
	require(e.body_add_torque(&world, body, {0, 0, 2}));
	require(e.world_step(&world, 0.25));
	value, get_status:=e.body_get(&world, body);
	require(get_status);
	fmt.printf("body-control scope=%d substeps=%d x=%08x vx=%08x wz=%08x\n", int(scope), substeps, transmute(u32)value.pose.position.x, transmute(u32)value.velocity.linear.x, transmute(u32)value.velocity.angular.z);
	lock:=e.body_axis_lock_default(value.pose);
	lock.linear_axes={.X};
	lock.angular_axes={.Z};
	require(e.body_set_axis_lock(&world, body, lock));
	for _ in 0..<4
	{
		require(e.world_step(&world, 1.0/64));
	}
	value, get_status=e.body_get(&world, body);
	require(get_status);
	fmt.printf("axis-lock scope=%d substeps=%d x=%08x vx=%08x wz=%08x\n", int(scope), substeps, transmute(u32)value.pose.position.x, transmute(u32)value.velocity.linear.x, transmute(u32)value.velocity.angular.z);
	require(e.world_destroy(&world));
}

Semantic_Delivery :: enum u8
{
	Parts,
	Parts_And_Breaks,
}

mixed_contact_semantic :: proc (scope: e.Allocation_Scope, custom_step: int, delivery: Semantic_Delivery)
{
	world: e.World;
	desc: e.World_Description = e.world_description_default();
	desc.gravity = {};
	desc.damping = {};
	desc.threading.worker_count = 2;
	desc.solve.substeps = 2;
	if custom_step != 0
	{
		desc.timestepper = {step=step};
	}
	require(e.world_init_with_allocation_scope(&world, desc, scope));
	require(e.world_enable_triggers(&world));
	configuration: e.Collider_Part_Configuration = e.collider_part_configuration_default();
	configuration.event_subscription = .Enabled;
	require(e.collider_parts_reserve(&world, configuration));
	require(e.world_enable_body_control(&world));
	require(e.world_enable_restitution(&world));
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.shape_add(&world, e.sphere(1));
	require(status);
	children: [2]e.Compound_Child = {e.compound_child(shape, e.pose()), e.compound_child(shape, e.pose({3, 0, 0}))};
	compound: e.Shape_Handle;
	compound, status = e.shape_import_compound(&world, children[:]);
	require(status);
	stat: e.Static_Handle;
	stat, status = e.static_add(&world, e.static_body(compound, e.pose()), .None);
	require(status);
	reference: e.Collidable_Reference;
	reference, status = e.static_collidable_reference(&world, stat);
	require(status);
	_, status = e.collider_part_set(&world, reference, 1, {user_id=52, role=.Trigger, options={.Stay}});
	require(status);
	require(e.restitution_set(&world, reference, {coefficient=0.5, threshold=0}));
	mass: e.Body_Inertia;
	mass, status = e.shape_inertia(e.sphere(1), 1);
	require(status);
	body: e.Body_Handle;
	body, status = e.body_add(&world, e.body_dynamic(shape, mass, e.pose({1.5, 0, 0}), e.velocity({-1, 0, 0}), e.body_activity(-1, 255)));
	require(status);
	body_reference: e.Collidable_Reference;
	body_reference, status = e.body_collidable_reference(&world, body);
	require(status);
	require(e.restitution_set(&world, body_reference, {coefficient=0.5, threshold=0}));
	lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose({1.5, 0, 0}));
	lock.linear_axes = {.Z};
	require(e.body_set_axis_lock(&world, body, lock));
	require(e.body_add_force(&world, body, {0, 0, 8}));
	joint: e.Constraint_Handle;
	if delivery == .Parts_And_Breaks
	{
		require(e.world_enable_joint_breaks(&world, 1));
		joint_body: e.Body_Handle;
		joint_body, status = e.body_add(&world, e.body_shapeless(mass, e.pose({10, 0, 0}), e.velocity(), e.body_activity(-1, 255)));
		require(status);
		joint, status = e.constraint_add_1(&world, joint_body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
		require(status);
		require(e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=0, user_id=417}));
	}
	tracker: e.Contact_Tracker;
	require(e.contact_tracker_init(&tracker, 8));
	require(e.contact_tracker_bind(&tracker, &world));
	parent: [8]e.Trigger_Event;
	parts: [8]e.Collider_Part_Event;
	contacts: [8]e.Contact_Event;
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	result: e.Collider_Part_Contact_Update_Result;
	if delivery == .Parts_And_Breaks
	{
		breaks: [2]e.Joint_Break_Event;
		combined: e.Joint_Break_Update_Result;
		combined, status = e.joint_break_stepper_update(&stepper, &world, 3.0/64, nil, parent[:], parts[:], &tracker, contacts[:]);
		if status != .Capacity_Missing || combined.completed_steps != 1 || combined.break_required != 1 || .Break not_in combined.pending || stepper.accumulator != 2.0/64
		{
			panic("joint stream backpressure");
		}
		parent_prefix: i32 = combined.events_written;
		part_prefix: i32 = combined.part_events_written;
		contact_prefix: i32 = combined.contact_events_written;
		combined, status = e.joint_break_stepper_update(&stepper, &world, 0, breaks[:], parent[parent_prefix:], parts[part_prefix:], &tracker, contacts[contact_prefix:]);
		require(status);
		if combined.completed_steps != 2 || combined.break_events_written != 1 || combined.pending != {} || stepper.accumulator != 0 ||
		breaks[0].constraint != joint || breaks[0].user_id != 417 || !(breaks[0].reaction.maximum_force > 0) || breaks[0].reaction.sample.substep_duration != 1.0/128
		{
			panic("joint stream completion");
		}
		result.completed_steps = combined.completed_steps + 1;
		result.events_written = combined.events_written + parent_prefix;
		result.part_events_written = combined.part_events_written + part_prefix;
		result.contact_events_written = combined.contact_events_written + contact_prefix;
		fmt.printf("joint-stream scope=%d custom=%d step=%d lifetime=%d force=%08x torque=%08x duration=%08x\n", int(scope), custom_step,
			breaks[0].reaction.step, breaks[0].lifetime, transmute(u32)breaks[0].reaction.maximum_force,
			transmute(u32)breaks[0].reaction.maximum_torque, transmute(u32)breaks[0].reaction.sample.substep_duration);
	}
	else
	{
		result, status = e.collider_part_stepper_update_with_contacts(&stepper, &world, 3.0/64, parent[:], parts[:], &tracker, contacts[:]);
		require(status);
	}
	if result.completed_steps != 3 || result.events_written != 3 || result.part_events_written != 3 ||
	result.contact_events_written != 3 || result.pending != {}
	{
		panic("mixed stream counts");
	}
	state: e.Body_State;
	state, status = e.body_get(&world, body);
	require(status);
	fmt.printf("mixed scope=%d custom=%d steps=%d parent=%d parts=%d contacts=%d x=%08x vx=%08x z=%08x vz=%08x\n", int(scope), custom_step,
		result.completed_steps, result.events_written, result.part_events_written, result.contact_events_written,
		transmute(u32)state.pose.position.x, transmute(u32)state.velocity.linear.x, transmute(u32)state.pose.position.z, transmute(u32)state.velocity.linear.z);
	for i: int = 0; i < 3; i += 1
	{
		if parent[i].step != parts[i].step || parts[i].step != u64(i+1)
		{
			panic("mixed stream step");
		}
		fmt.printf("mixed-event=%d parent=%d part=%d contact=%d normal=%08x flags=%d\n", i, int(parent[i].kind), int(parts[i].kind), int(contacts[i].kind), transmute(u32)contacts[i].normal.x, parts[i].pair.flags);
	}
	require(e.contact_tracker_destroy(&tracker));
	require(e.world_destroy(&world));
}

main :: proc()
{
	for scope in 0..<2
	{
		for custom_step in 0..<2
		{
			run(e.Allocation_Scope(scope), custom_step);
		}
	}
	for scope in 0..<2
	{
		for iterations in ([2]i32{1, 4})
		{
			restitution_semantic(e.Allocation_Scope(scope), iterations);
		}
	}
	for scope in 0..<2
	{
		for substeps in ([2]i32{1, 4})
		{
			body_control_semantic(e.Allocation_Scope(scope), substeps);
		}
	}
	for scope: int = 0; scope < 2; scope += 1
	{
		for custom_step: int = 0; custom_step < 2; custom_step += 1
		{
			for delivery: int = 0; delivery < 2; delivery += 1
			{
				mixed_contact_semantic(e.Allocation_Scope(scope), custom_step, Semantic_Delivery(delivery));
			}
		}
	}
}
