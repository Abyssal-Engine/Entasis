package compounds

import entasis "entasis:entasis"

STEPS :: 10;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
	bodies: [2]entasis.Body_Handle,
	compound, big: entasis.Compound_Build_Result,
	tick: int,
}

make_children :: proc(sphere_shape, box_shape: entasis.Shape_Handle) -> ([3]entasis.Compound_Child, [3]f32)
{
	return [3]entasis.Compound_Child{
		entasis.compound_child(sphere_shape, entasis.pose({-1, 0, 0})),
		entasis.compound_child(box_shape, entasis.pose()),
		entasis.compound_child(sphere_shape, entasis.pose({2, 0, 0})),
	}, [3]f32{1, 2, 3};
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.description = entasis.world_description_default();
	status: entasis.Status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	sphere_shape, box_shape: entasis.Shape_Handle;
	sphere_shape, status = entasis.shape_add(&c.world, entasis.sphere(0.5));
	if status != .Ok
	{
		return status;
	}
	box_shape, status = entasis.shape_add(&c.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	children: [3]entasis.Compound_Child;
	masses: [3]f32;
	children, masses = make_children(sphere_shape, box_shape);
	c.compound, status = entasis.compound_build_dynamic(&c.world, entasis.compound_builder(children[:], masses[:]), true);
	if status != .Ok || c.compound.inertia.inverse_mass <= 0 || c.compound.center.x <= 0
	{
		return .Invalid_Argument;
	}
	children, masses = make_children(sphere_shape, box_shape);
	c.big, status = entasis.big_compound_build_dynamic(&c.world, entasis.compound_builder(children[:], masses[:]), true);
	if status != .Ok || c.big.inertia.inverse_mass <= 0 || c.big.center.x <= 0
	{
		return .Invalid_Argument;
	}
	c.bodies[0], status = entasis.body_add(&c.world, entasis.body_dynamic(c.compound.shape, c.compound.inertia,
		entasis.pose({-3, 5, 0}), {}, entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	c.bodies[1], status = entasis.body_add(&c.world, entasis.body_dynamic(c.big.shape, c.big.inertia,
		entasis.pose({3, 5, 0}), {}, entasis.body_activity(-1, 255)));
	return status;
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = entasis.world_step(&c.world, TIMESTEP);
	if status == .Ok
	{
		c.tick += 1;
	}
	return status;
}

case_check :: proc(c: ^Case) -> entasis.Status
{
	for body in c.bodies
	{
		state: entasis.Body_State;
		status: entasis.Status;
		state, status = entasis.body_get(&c.world, body);
		if status != .Ok || state.pose.position.y >= 5
		{
			return .Invalid_Argument;
		}
	}
	return .Ok if c.tick == STEPS else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
