package materials_and_filtering

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
		panic("material response");
	}
	fmt.println("low/high friction x:", c.results[0], c.results[1], "low/high recovery vy:", c.results[2], c.results[3]);
}
