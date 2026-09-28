package sweep_queries

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("sweep_queries creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("sweep_queries operation");
		}
	}
	fmt.println("sweep impact t:", owner.hit.sweep.t0, "normal:", owner.hit.sweep.normal);
}
