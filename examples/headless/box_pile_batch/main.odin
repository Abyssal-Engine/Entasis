package box_pile_batch

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("box_pile_batch creation");
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
		panic("box_pile_batch result");
	}
	fmt.println("batch pile bodies:", len(owner.bodies), "minimum y:", owner.minimum_y);
}
