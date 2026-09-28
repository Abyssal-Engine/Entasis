package kinematic_platform

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("kinematic platform creation");
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
		panic("kinematic displacement");
	}
	fmt.println("platform y:", owner.platform_y, "passenger y:", owner.passenger_y);
}
