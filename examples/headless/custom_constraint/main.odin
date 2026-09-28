package custom_constraint

import "core:fmt"

main :: proc()
{
	c: Case;
	defer case_destroy(&c);
	if case_create(&c) != .Ok
	{
		panic("create");
	}
	if case_step(&c) != .Ok
	{
		panic("step");
	}
	fmt.println("custom constraint velocity:", c.velocity);
}
