package ragdoll

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
		panic("ragdoll invariant");
	}
	fmt.println("ragdoll limited constraints:", c.constraint_count, "max socket error:", c.maximum_error);
}
