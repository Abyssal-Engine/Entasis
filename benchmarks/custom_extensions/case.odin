package custom_extensions

import "core:fmt"
import "core:simd"
import e "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

BODY_COUNT :: 1024;
WARMUP_STEPS :: 30;
MEASURED_STEPS :: 300;
DT :: f32(1.0 / 60.0);
MODES: [5]string = {"arity1", "arity2", "arity3", "arity4", "sequential_fallback"};
Description :: struct
{
	target: f32,
}

Prestep :: struct
{
	target: e.F32x8,
}

Result :: struct
{
	elapsed: f64,
	active_bodies, custom_constraints, fallback_constraints, contact_pairs: int,
}

Owner :: struct
{
	world: e.World,
	description: e.World_Description,
	shape: e.Shape_Handle,
	shape_value: support.Custom_Sphere,
	bodies: [BODY_COUNT]e.Body_Handle,
	constraints: [BODY_COUNT * 3]e.Constraint_Handle,
	constraint_count: int,
	pose: e.Uniform_Gravity_Policy,
	material: e.Default_Narrow_Policy,
}

validate_description :: proc "contextless" (_: i32, raw: rawptr) -> e.Status
{
	if raw == nil
	{
		return .Invalid_Argument;
	}
	value: f32 = (^Description)(raw).target;
	if value != value || abs(value) > 10
	{
		return .Invalid_Description;
	}
	return .Ok;
}

// distinct native procedure addresses deliberately select the existing custom
// pose/material path, not the engine's recognized built-in specializations
custom_integrate :: proc "contextless" (
	ctx: rawptr, indices: util.I32x8, position: util.Vector3_Wide,
	orientation: util.Quaternion_Wide, inertia: physics.Body_Inertia_Wide,
	mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	physics.pose_integrator_default_velocity(ctx, indices, position, orientation, inertia, mask, worker, dt, velocity);
}

custom_material :: proc "contextless" (
	ctx: rawptr, _: int, a, b: e.Collidable_Reference,
	manifold: ^e.Manifold_Result, material: ^e.Contact_Material,
) -> e.Collision_Testing_State
{
	_, _, _ = a, b, manifold;
	material^ = (^e.Default_Narrow_Policy)(ctx).material;
	return .Allow;
}

Impulses1 :: struct
{
	values: [1]e.F32x8,
}

kernel1 :: proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	_: f32, _: f32, impulses_raw: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase,
)
{
	prestep: ^Prestep = (^Prestep)(prestep_raw);
	impulses: ^Impulses1 = (^Impulses1)(impulses_raw);
	for index in 0..<1
	{
		body: ^e.Constraint_Kernel_Body_Wide = &bodies[index];
		if phase == .Warmstart
		{
			v: e.F32x8 = simd.add(body.linear_velocity.x, simd.mul(impulses.values[index], body.inverse_mass));
			body.linear_velocity.x = util.wide_select_f32(mask, v, body.linear_velocity.x);
		}
		else if phase == .Solve
		{
			dv: e.F32x8 = simd.sub(prestep.target, body.linear_velocity.x);
			// inactive lanes must not produce 0/0, even though their outputs are masked
			inverse_mass: e.F32x8 = util.wide_select_f32(mask, body.inverse_mass, e.F32x8(1));
			di: e.F32x8 = simd.div(dv, inverse_mass);
			body.linear_velocity.x = util.wide_select_f32(mask, simd.add(body.linear_velocity.x, simd.mul(di, inverse_mass)), body.linear_velocity.x);
			impulses.values[index] = util.wide_select_f32(mask, simd.add(impulses.values[index], di), impulses.values[index]);
		}
	}
}

Impulses2 :: struct
{
	values: [2]e.F32x8,
}

kernel2 :: proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	_: f32, _: f32, impulses_raw: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase,
)
{
	prestep: ^Prestep = (^Prestep)(prestep_raw);
	impulses: ^Impulses2 = (^Impulses2)(impulses_raw);
	for index in 0..<2
	{
		body: ^e.Constraint_Kernel_Body_Wide = &bodies[index];
		if phase == .Warmstart
		{
			v: e.F32x8 = simd.add(body.linear_velocity.x, simd.mul(impulses.values[index], body.inverse_mass));
			body.linear_velocity.x = util.wide_select_f32(mask, v, body.linear_velocity.x);
		}
		else if phase == .Solve
		{
			dv: e.F32x8 = simd.sub(prestep.target, body.linear_velocity.x);
			// inactive lanes must not produce 0/0, even though their outputs are masked
			inverse_mass: e.F32x8 = util.wide_select_f32(mask, body.inverse_mass, e.F32x8(1));
			di: e.F32x8 = simd.div(dv, inverse_mass);
			body.linear_velocity.x = util.wide_select_f32(mask, simd.add(body.linear_velocity.x, simd.mul(di, inverse_mass)), body.linear_velocity.x);
			impulses.values[index] = util.wide_select_f32(mask, simd.add(impulses.values[index], di), impulses.values[index]);
		}
	}
}

Impulses3 :: struct
{
	values: [3]e.F32x8,
}

kernel3 :: proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	_: f32, _: f32, impulses_raw: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase,
)
{
	prestep: ^Prestep = (^Prestep)(prestep_raw);
	impulses: ^Impulses3 = (^Impulses3)(impulses_raw);
	for index in 0..<3
	{
		body: ^e.Constraint_Kernel_Body_Wide = &bodies[index];
		if phase == .Warmstart
		{
			v: e.F32x8 = simd.add(body.linear_velocity.x, simd.mul(impulses.values[index], body.inverse_mass));
			body.linear_velocity.x = util.wide_select_f32(mask, v, body.linear_velocity.x);
		}
		else if phase == .Solve
		{
			dv: e.F32x8 = simd.sub(prestep.target, body.linear_velocity.x);
			// inactive lanes must not produce 0/0, even though their outputs are masked
			inverse_mass: e.F32x8 = util.wide_select_f32(mask, body.inverse_mass, e.F32x8(1));
			di: e.F32x8 = simd.div(dv, inverse_mass);
			body.linear_velocity.x = util.wide_select_f32(mask, simd.add(body.linear_velocity.x, simd.mul(di, inverse_mass)), body.linear_velocity.x);
			impulses.values[index] = util.wide_select_f32(mask, simd.add(impulses.values[index], di), impulses.values[index]);
		}
	}
}

Impulses4 :: struct
{
	values: [4]e.F32x8,
}

kernel4 :: proc "contextless" (
	prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	_: f32, _: f32, impulses_raw: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase,
)
{
	prestep: ^Prestep = (^Prestep)(prestep_raw);
	impulses: ^Impulses4 = (^Impulses4)(impulses_raw);
	for index in 0..<4
	{
		body: ^e.Constraint_Kernel_Body_Wide = &bodies[index];
		if phase == .Warmstart
		{
			v: e.F32x8 = simd.add(body.linear_velocity.x, simd.mul(impulses.values[index], body.inverse_mass));
			body.linear_velocity.x = util.wide_select_f32(mask, v, body.linear_velocity.x);
		}
		else if phase == .Solve
		{
			dv: e.F32x8 = simd.sub(prestep.target, body.linear_velocity.x);
			// inactive lanes must not produce 0/0, even though their outputs are masked
			inverse_mass: e.F32x8 = util.wide_select_f32(mask, body.inverse_mass, e.F32x8(1));
			di: e.F32x8 = simd.div(dv, inverse_mass);
			body.linear_velocity.x = util.wide_select_f32(mask, simd.add(body.linear_velocity.x, simd.mul(di, inverse_mass)), body.linear_velocity.x);
			impulses.values[index] = util.wide_select_f32(mask, simd.add(impulses.values[index], di), impulses.values[index]);
		}
	}
}

owner_init :: proc(owner: ^Owner, workers, mode: int) -> e.Status
{
	desc: e.World_Description = e.world_description_default();
	desc.threading.worker_count = i32(workers);
	desc.capacity.bodies = BODY_COUNT;
	desc.capacity.constraints = BODY_COUNT * 4;
	desc.capacity.initial_constraints_per_type_batch = BODY_COUNT;
	desc.solve.velocity_iterations = 4;
	desc.solve.substeps = 1;
	if mode == 4
	{
		desc.solve.fallback_batch_threshold = 1;
	}
	owner.pose = e.Uniform_Gravity_Policy{gravity={0, -2, 0}, linear_damping=0, angular_damping=0};
	desc.pose_callbacks = e.pose_policy_uniform(&owner.pose);
	desc.pose_callbacks.integrate_velocity = custom_integrate;
	owner.material.material = e.contact_material(0, 2, e.spring_settings(30, 1));
	desc.narrow_callbacks = e.narrow_policy_default(&owner.material);
	desc.narrow_callbacks.configure = custom_material;
	owner.description = desc;
	if failure: e.Status = e.world_init(&owner.world, desc); failure != .Ok
	{
		return failure;
	}
	shape_id: e.Shape_Type_ID;
	registration_status: e.Status;
	shape_id, registration_status = support.extension_register_sphere(&owner.world);
	if registration_status != .Ok
	{
		return registration_status;
	}
	value: support.Custom_Sphere = support.Custom_Sphere{radius=0.5};
	shape: e.Shape_Handle;
	status: e.Status;
	shape, status = e.custom_shape_add(&owner.world, shape_id, &value);
	if status != .Ok
	{
		return status;
	}
	owner.shape, owner.shape_value = shape, value;
	floor: e.Shape_Handle;
	floor_status: e.Status;
	floor, floor_status = e.shape_add(&owner.world, e.box(128, 2, 128));
	if floor_status != .Ok
	{
		return floor_status;
	}
	_, status = e.static_add(&owner.world, e.static_body(floor, e.pose({0, -1, 0})), .None);
	if status != .Ok
	{
		return status;
	}
	inertia: e.Body_Inertia;
	inertia_status: e.Status;
	inertia, inertia_status = e.shape_inertia(e.sphere(0.5), 1);
	if inertia_status != .Ok
	{
		return inertia_status;
	}
	for index in 0..<BODY_COUNT
	{
		body: e.Body_Handle;
		add_status: e.Status;
		body, add_status = e.body_add(&owner.world, e.body_dynamic(shape, inertia,
				e.pose({f32(index % 32) * 2 - 32, 0.5, f32(index / 32) * 2 - 32}), {}, e.body_activity(-1, 255)));
		if add_status != .Ok
		{
			return add_status;
		}
		owner.bodies[index] = body;
	}
	type_id: e.Constraint_Type_ID;
	next_status: e.Status;
	type_id, next_status = e.custom_constraint_next_type_id(&owner.world);
	if next_status != .Ok
	{
		return next_status;
	}
	arity: int = mode + 1;
	if mode == 4
	{
		arity = 1;
	}
	access: [4]e.Body_Access_Mask;
	for i in 0..<arity
	{
		access[i] = e.BODY_ACCESS_NO_POSE;
	}
	registration: e.Custom_Constraint_Registration;
	switch arity
	{
		case 1: registration = e.custom_constraint_registration(Description, Prestep, Impulses1, type_id, 1, access, access, validate_description, kernel1);
		case 2: registration = e.custom_constraint_registration(Description, Prestep, Impulses2, type_id, 2, access, access, validate_description, kernel2);
		case 3: registration = e.custom_constraint_registration(Description, Prestep, Impulses3, type_id, 3, access, access, validate_description, kernel3);
		case 4: registration = e.custom_constraint_registration(Description, Prestep, Impulses4, type_id, 4, access, access, validate_description, kernel4);
	}
	if failure: e.Status = e.custom_constraint_register(&owner.world, registration); failure != .Ok
	{
		return failure;
	}
	copies: int = 1;
	if mode == 4
	{
		copies = 3;
	}
	for group in 0..<(BODY_COUNT + arity - 1) / arity
	{
		for _ in 0..<copies
		{
			target: Description = Description{target=1};
			first: int = min(group * arity, BODY_COUNT - arity);
			handle: e.Constraint_Handle;
			add_status: e.Status;
			handle, add_status = e.custom_constraint_add_typed(&owner.world, owner.bodies[first:first+arity], type_id, &target);
			if add_status != .Ok
			{
				return add_status;
			}
			owner.constraints[owner.constraint_count] = handle;
			owner.constraint_count += 1;
		}
	}
	return .Ok;
}

validate_owner :: proc(owner: ^Owner, mode: int) -> (Result, e.Status)
{
	stats: e.World_Stats;
	status: e.Status;
	stats, status = e.world_stats(&owner.world);
	if status != .Ok
	{
		return {}, status;
	}
	if stats.active_bodies != BODY_COUNT || stats.sleeping_bodies != 0 || stats.active_pairs < BODY_COUNT
	{
		fmt.eprintfln("custom coverage invalid mode=%s stats=%v", MODES[mode], stats);
		return {}, .Invalid_Argument;
	}
	arity: int = mode + 1;
	if mode == 4
	{
		arity = 1;
	}
	for handle, index in owner.bodies
	{
		state: e.Body_State;
		get_status: e.Status;
		state, get_status = e.body_get(&owner.world, handle);
		if get_status != .Ok
		{
			return {}, get_status;
		}
		expected_x: f32 = f32(1);
		if abs(state.velocity.linear.x - expected_x) > 1e-3 ||
		state.pose.position.y < 0.45 || state.pose.position.y > 0.55 ||
		state.pose.position.x != state.pose.position.x
		{
			fmt.eprintfln("custom body failed mode=%s body=%d state=%v", MODES[mode], index, state);
			return {}, .Invalid_Argument;
		}
	}
	fallback: int = 0;
	simulation: ^physics.Simulation;
	borrow_status: e.Status;
	simulation, borrow_status = e.world_borrow_simulation(&owner.world);
	if borrow_status != .Ok
	{
		return {}, borrow_status;
	}
	for handle in owner.constraints[:owner.constraint_count]
	{
		info: e.Constraint_Info;
		inspect_status: e.Status;
		info, inspect_status = e.constraint_inspect(&owner.world, handle);
		if inspect_status != .Ok
		{
			return {}, inspect_status;
		}
		if int(info.body_count) != arity
		{
			return {}, .Invalid_Argument;
		}
		location: physics.Constraint_Location = simulation.solver.handle_to_constraint.memory[handle.value];
		if mode == 4 && location.batch_index == 1
		{
			fallback += 1;
		}
	}
	if mode == 4 && fallback != BODY_COUNT * 2
	{
		return {}, .Invalid_Argument;
	}
	return {active_bodies=stats.active_bodies, custom_constraints=owner.constraint_count, fallback_constraints=fallback, contact_pairs=stats.active_pairs}, .Ok;
}
