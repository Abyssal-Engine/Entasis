package substepping

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("substepping creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("step");
		}
	}
	if case_check(&owner) != .Ok
	{
		panic("substep error");
	}
	fmt.println("constraint error, 1 substep:", owner.errors[0], "4 substeps:", owner.errors[1]);
}
