package rope

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("rope creation");
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
		panic("rope invariant");
	}
	fmt.println("rope end y:", owner.end_y, "maximum joint error:", owner.maximum_error,
		"distance and twist constraints:", owner.constraint_count);
}
