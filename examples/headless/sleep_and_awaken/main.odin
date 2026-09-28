package sleep_and_awaken

import "core:fmt"

main :: proc()
{
	c: Case;
	if case_create(&c)!=.Ok
	{
		panic("world setup");
	}
	defer case_destroy(&c);
	for _ in 0 ..< STEPS
	{
		if case_step(&c)!=.Ok
		{
			panic("sleep and awaken");
		}
	}
	fmt.println("body slept and awakened");
}
