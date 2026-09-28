package external_dispatcher

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("external_dispatcher creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("external_dispatcher step");
		}
	}
	fmt.println("external dispatcher matched included state");
}
