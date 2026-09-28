package tank_controller

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
		panic("tank controller");
	}
	fmt.println("tank position:", c.position, "lateral turn distance:", c.position.z, "chassis constraints:", c.constraint_count);
}
