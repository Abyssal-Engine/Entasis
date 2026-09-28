package bulk_body_update

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("bulk_body_update creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("step");
		}
	}
	fmt.println("bulk body checksum:", owner.checksum);
}
