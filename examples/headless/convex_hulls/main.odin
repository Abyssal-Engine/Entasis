package convex_hulls

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok || case_step(&owner) != .Ok
	{
		panic("cooked hull import");
	}
	fmt.println("cooked hull imported; center:", owner.cooked.center);
}
