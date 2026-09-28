package background_cooking

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("background_cooking creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("background_cooking step");
		}
	}
	fmt.println("background hull import t:", owner.hit.t, "independent checksum:", owner.independent_checksum);
}
