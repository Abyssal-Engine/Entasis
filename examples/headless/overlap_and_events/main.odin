package overlap_and_events

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
			panic("contact lifecycle");
		}
	}
	fmt.println("overlaps:", c.overlap_count, "contact lifecycle: Begin Persist End");
}
