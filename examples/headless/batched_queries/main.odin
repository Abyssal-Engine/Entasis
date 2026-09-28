package batched_queries

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("batched query creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("batch/scalar mismatch");
		}
	}
	fmt.println("batched queries: closest t:", owner.results[1].ray_hit.t, "all hits:", owner.results[2].count);
}
