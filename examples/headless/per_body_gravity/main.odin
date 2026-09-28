package per_body_gravity

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("per-body gravity creation");
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
		panic("per-body gravity");
	}
	fmt.println("light gravity y:", owner.slow_y, "heavy gravity y:", owner.fast_y);
}
