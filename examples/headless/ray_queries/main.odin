package ray_queries

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("ray_queries creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("ray_queries operation");
		}
	}
	fmt.println("ray hits:", owner.count, "closest t:", owner.hits[0].t);
}
