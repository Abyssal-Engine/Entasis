package meshes

import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"

STEPS :: 60;
TIMESTEP :: f32(1.0 / 60.0);
Case :: struct
{
	ctx: cooking.Cooking_Context,
	closed, open: cooking.Cooked_Mesh,
	closed_mass, open_mass: entasis.Mesh_Mass_Properties,
	world: entasis.World,
	description: entasis.World_Description,
	body: entasis.Body_Handle,
	ray: entasis.Ray,
	hit: entasis.Ray_Hit,
	tick: int,
	final_y: f32,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	status: entasis.Status = cooking.cooking_context_init(&c.ctx);
	if status != .Ok
	{
		return status;
	}
	// closed tetrahedron with deliberately offset source geometry
	closed_triangles: [4]entasis.Triangle = {
		entasis.triangle({1, 1, 1}, {3, 1, 1}, {1, 3, 1}),
		entasis.triangle({1, 1, 1}, {1, 1, 3}, {3, 1, 1}),
		entasis.triangle({1, 1, 1}, {1, 3, 1}, {1, 1, 3}),
		entasis.triangle({3, 1, 1}, {1, 1, 3}, {1, 3, 1}),
	};
	c.closed, status = cooking.cook_mesh(&c.ctx, closed_triangles[:]);
	if status != .Ok
	{
		return status;
	}
	c.closed_mass, status = cooking.cooked_mesh_closed_mass_properties(&c.closed, 2, true);
	if status != .Ok || c.closed_mass.inertia.inverse_mass <= 0 ||
		abs(c.closed_mass.volume) <= 0.1 || c.closed_mass.center.x <= 1
	{
		return .Invalid_Argument;
	}
	open_triangles: [2]entasis.Triangle = {
		entasis.triangle({-2, 0, -2}, {2, 0, -2}, {2, 0, 2}),
		entasis.triangle({-2, 0, -2}, {2, 0, 2}, {-2, 0, 2}),
	};
	c.open, status = cooking.cook_mesh(&c.ctx, open_triangles[:]);
	if status != .Ok
	{
		return status;
	}
	c.open_mass, status = cooking.cooked_mesh_open_mass_properties(&c.open, 1, false);
	if status != .Ok || c.open_mass.inertia.inverse_mass <= 0 || abs(c.open_mass.center.y) > 1e-4
	{
		return .Invalid_Argument;
	}
	c.description = entasis.world_description_default();
	status = entasis.world_init(&c.world, c.description);
	if status != .Ok
	{
		return status;
	}
	floor_mesh, closed_mesh: entasis.Shape_Handle;
	floor_mesh, status = cooking.cooked_mesh_import(&c.world, &c.open);
	if status != .Ok
	{
		return status;
	}
	_, status = entasis.static_add(&c.world, entasis.static_body(floor_mesh, entasis.pose()), .None);
	if status != .Ok
	{
		return status;
	}
	closed_mesh, status = cooking.cooked_mesh_import(&c.world, &c.closed);
	if status != .Ok
	{
		return status;
	}
	c.body, status = entasis.body_add(&c.world, entasis.body_dynamic(closed_mesh, c.closed_mass.inertia,
		entasis.pose({0, 4, 0}), {}, entasis.body_activity(-1, 255)));
	if status != .Ok
	{
		return status;
	}
	c.ray = entasis.ray({0, 2, 0}, {0, -1, 0}, 5);
	c.hit, status = entasis.ray_cast_closest(&c.world, c.ray);
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
	state: entasis.Body_State;
	status: entasis.Status;
	state, status = entasis.body_get(&c.world, c.body);
	if status != .Ok
	{
		return status;
	}
	c.final_y = state.pose.position.y;
	return .Ok if c.tick == STEPS && c.final_y < 3.9 && c.final_y > -1 else .Invalid_Argument;
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	cooking.cooked_mesh_destroy(&c.open);
	cooking.cooked_mesh_destroy(&c.closed);
	cooking.cooking_context_destroy(&c.ctx);
	c^ = {};
}
