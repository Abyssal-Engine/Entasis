package custom_constraint

import "core:simd"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"

Description :: struct
{
	target_speed: f32,
}
Prestep :: struct
{
	target_speed: entasis.F32x8,
}
Impulses :: struct
{
	accumulated: entasis.F32x8,
}

validate :: proc "contextless" (_: i32, description: rawptr) -> entasis.Status
{
	if description == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}
kernel :: proc "contextless" (
	prestep_raw: rawptr,
	bodies: ^[4]entasis.Constraint_Kernel_Body_Wide,
	_: f32,
	_: f32,
	impulses_raw: rawptr,
	mask: entasis.I32x8,
	phase: entasis.Constraint_Kernel_Phase
)
{
	prestep := (^Prestep)(prestep_raw);
	impulses := (^Impulses)(impulses_raw);
	body := &bodies[0];
	if phase == .Warmstart
	{
		v := simd.add(body.linear_velocity.x, simd.mul(impulses.accumulated, body.inverse_mass));
		body.linear_velocity.x = util.wide_select_f32(mask, v, body.linear_velocity.x);
	}
	else if phase == .Solve
	{
		dv := simd.sub(prestep.target_speed, body.linear_velocity.x);
		di := simd.div(dv, body.inverse_mass);
		body.linear_velocity.x = util.wide_select_f32(
			mask,
			simd.add(body.linear_velocity.x, simd.mul(di, body.inverse_mass)),
			body.linear_velocity.x
		);
		impulses.accumulated = util.wide_select_f32(mask, simd.add(impulses.accumulated, di), impulses.accumulated);
	}
}

STEPS :: 1;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	target: Description,
	tick: int,
	velocity: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	description: entasis.World_Description = entasis.world_description_default();
	description.gravity = {};
	c.description = description;
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	type_id: entasis.Constraint_Type_ID;
	type_id, status = entasis.custom_constraint_next_type_id(&c.world);
	if status != .Ok
	{
		return status;
	}
	access: [4]entasis.Body_Access_Mask = {entasis.BODY_ACCESS_NO_POSE, {}, {}, {}};
	registration: entasis.Custom_Constraint_Registration = entasis.custom_constraint_registration(
		Description, Prestep, Impulses, type_id, 1, access, access, validate, kernel);
	status = entasis.custom_constraint_register(&c.world, registration);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_shapeless(
		{inverse_inertia_tensor={xx=1, yy=1, zz=1}, inverse_mass=1}, entasis.pose()));
	if status != .Ok
	{
		return status;
	}
	c.target = {target_speed=4};
	_, status = entasis.custom_constraint_add_typed(&c.world, []entasis.Body_Handle{c.body}, type_id, &c.target);
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status != .Ok
	{
		return status;
	}
	state: entasis.Body_State;
	state, status = entasis.body_get(&c.world, c.body);
	if status == .Ok
	{
		c.tick += 1;
		c.velocity = state.velocity.linear.x;
	}
	return status;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
