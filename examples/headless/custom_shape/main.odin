package custom_shape

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
	fmt.println("custom shape ray t:", c.hit.t);
}
