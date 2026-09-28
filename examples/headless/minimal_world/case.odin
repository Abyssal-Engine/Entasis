package minimal_world

import entasis "entasis:entasis"

STEPS :: 1;
TIMESTEP :: f32(1.0/60.0);
Case :: struct
{
	world: entasis.World,
	description: entasis.World_Description,
}

case_create :: proc(c: ^Case) -> entasis.Status
{
	c.description = entasis.world_description_default();
	return entasis.world_init(&c.world, c.description);
}

case_step :: proc(c: ^Case) -> entasis.Status
{
	return entasis.world_step(&c.world, TIMESTEP);
}

case_destroy :: proc(c: ^Case)
{
	entasis.world_destroy(&c.world);
	c^ = {};
}
