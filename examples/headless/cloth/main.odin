package cloth

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("cloth creation");
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
		panic("cloth invariant");
	}
	fmt.println("cloth bottom y:", owner.bottom_y, "maximum edge error:", owner.maximum_error);
}
