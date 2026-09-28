package continuous_collision

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("continuous_collision creation");
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
		panic("CCD tunneling result");
	}
	fmt.println("discrete x:", owner.positions[0], "continuous x:", owner.positions[1]);
}
