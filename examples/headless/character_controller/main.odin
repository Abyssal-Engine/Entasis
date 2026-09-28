package character_controller

import "core:fmt"

main :: proc()
{
	c: Case;
	defer case_destroy(&c);
	if case_create(&c) != .Ok
	{
		panic("create");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&c) != .Ok
		{
			panic("step");
		}
	}
	if case_check(&c) != .Ok
	{
		panic("controller result");
	}
	fmt.println("controller x:", c.final_x, "grounded frames:", c.grounded_frames);
}
