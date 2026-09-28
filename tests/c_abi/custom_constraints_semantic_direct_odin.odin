package main

import "core:fmt"
import "core:simd"
import e "entasis:entasis"

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

kernel2 :: proc "contextless" (prestep: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase)
{
	_, _ = dt, inverse_dt;
	execute(prestep, bodies, impulses, mask, phase, 2);
}

kernel3 :: proc "contextless" (prestep: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase)
{
	_, _ = dt, inverse_dt;
	execute(prestep, bodies, impulses, mask, phase, 3);
}

kernel4 :: proc "contextless" (prestep: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase)
{
	_, _ = dt, inverse_dt;
	execute(prestep, bodies, impulses, mask, phase, 4);
}

require :: proc(status: e.Status)
{
	if status != .Ok
	{
		panic(fmt.tprintf("constraint semantics: %v", status));
	}
}

init :: proc(world: ^e.World, arity, substeps, fallback: int)
{
	d := e.world_description_default();
	d.gravity={};
	d.threading.worker_count=1;
	d.capacity={bodies=4, statics=4, inactive_body_sets=2, shapes_per_type=2, constraints=16, initial_constraints_per_type_batch=4,
		minimum_constraints_per_body=4, broad_phase_candidates=16, pairs=16, collision_child_pairs=0, inactive_pairs=8, pending_pairs_per_worker=8};
	d.solve={velocity_iterations=4, substeps=i32(substeps), fallback_batch_threshold=i32(fallback)};
	require(e.world_init(world, d));
	initial, solve: [4]e.Body_Access_Mask;
	for i in 0 ..< arity
	{
		initial[i]=e.BODY_ACCESS_ALL;
		solve[i]={.Mass, .Linear_Velocity};
	}
	kernels := [4]e.Constraint_Kernel_Proc{kernel1, kernel2, kernel3, kernel4};
	r := e.custom_constraint_registration(f32, e.F32x8, e.F32x8, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID,
		arity, initial, solve, validate, kernels[arity-1]);
	require(e.custom_constraint_register(world, r));
}

Body_Activation_State :: enum u8
{
	Awake,
	Sleeping,
}

body :: proc(world: ^e.World, x: f32, activation: Body_Activation_State) -> e.Body_Handle
{
	activity := e.body_activity(-1, 255);
	if activation == .Sleeping
	{
		activity=e.body_activity(10, 1);
	}
	result, status := e.body_add(world, e.body_shapeless({inverse_mass=1, inverse_inertia_tensor={xx=1, yy=1, zz=1}},
			e.pose({x, 0, 0}), e.velocity(), activity));
	require(status);
	return result;
}

main :: proc()
{
	for arity in 1 ..= 4
	{
		world: e.World;
		init(&world, arity, 2, 8);
		bodies: [36]e.Body_Handle;
		handles: [9]e.Constraint_Handle;
		for ci in 0 ..< 9
		{
			for bi in 0 ..< arity
			{
				bodies[ci*arity+bi]=body(&world, f32((ci*arity+bi)*3), .Awake);
			}
			target := f32(ci+1);
			h, status:=e.custom_constraint_add(&world, bodies[ci*arity:(ci+1)*arity], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &target, 4);
			require(status);
			handles[ci]=h;
		}
		target:f32=12;
		require(e.custom_constraint_apply(&world, handles[8], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &target, 4));
		require(e.world_step(&world, 1.0/60.0));
		for ci in 0 ..< 9
		{
			stored:f32;
			require(e.custom_constraint_get(&world, handles[ci], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &stored, 4));
			impulses:[1]f32;
			written, required, status:=e.constraint_accumulated_impulses(&world, handles[ci], impulses[:]);
			require(status);
			assert(written==1 && required==1);
			fmt.printf("arity=%d index=%d handle=%d description=%08x impulse=%08x", arity, ci, handles[ci].value, transmute(u32)stored, transmute(u32)impulses[0]);
			for bi in 0 ..< arity
			{
				state, get_status:=e.body_get(&world, bodies[ci*arity+bi]);
				require(get_status);
				fmt.printf(" v%d=%08x", bi, transmute(u32)state.velocity.linear.x);
			}
			fmt.println();
		}
		require(e.world_destroy(&world));
	}
	world:e.World;
	init(&world, 1, 1, 1);
	b:=body(&world, 0, .Sleeping);
	handles:[3]e.Constraint_Handle;
	for &h in handles
	{
		zero:f32;
		value, status:=e.custom_constraint_add(&world, []e.Body_Handle{b}, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &zero, 4);
		require(status);
		h=value;
	}
	for _ in 0 ..< 4
	{
		require(e.world_step(&world, 1.0/60.0));
	}
	sleeping, status:=e.body_is_sleeping(&world, b);
	require(status);
	invalid:=transmute(f32)u32(0x7fc00000);
	rejected:=e.custom_constraint_apply(&world, handles[2], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &invalid, 4);
	target:f32=2;
	require(e.custom_constraint_apply(&world, handles[2], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &target, 4));
	require(e.body_awaken(&world, b));
	require(e.world_step(&world, 1.0/60.0));
	state, get_status:=e.body_get(&world, b);
	require(get_status);
	fmt.printf("fallback sleeping=%d invalid=%d velocity=%08x\n", int(sleeping), u32(rejected), transmute(u32)state.velocity.linear.x);
	require(e.world_destroy(&world));
}
