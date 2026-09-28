package direct_collision_queries

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
			panic("operation");
		}
	}
	fmt.println("direct contacts:", c.manifold.convex.count, "sweep hits:", c.count);
}
