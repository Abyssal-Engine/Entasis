package gyroscope

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("gyroscope creation");
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
		panic("gyroscopic response");
	}
	fmt.println("nonconserving angular:", owner.velocities[0].angular,
		"gyroscopic angular:", owner.velocities[1].angular, "delta:", owner.delta);
}
