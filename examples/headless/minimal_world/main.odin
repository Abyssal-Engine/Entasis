package minimal_world

import "core:fmt"

main :: proc()
{
	c: Case;
	if case_create(&c) != .Ok
	{
		panic("world_init");
	}
	defer case_destroy(&c);
	if case_step(&c) != .Ok
	{
		panic("world_step");
	}
	fmt.println("minimal world stepped");
}
