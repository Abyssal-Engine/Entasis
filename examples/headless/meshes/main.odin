package meshes

import "core:fmt"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("meshes creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("meshes step");
		}
	}
	if case_check(&owner) != .Ok
	{
		panic("dynamic mesh state");
	}
	fmt.println("closed volume:", owner.closed_mass.volume, "closed center:", owner.closed_mass.center,
		"open center:", owner.open_mass.center, "dynamic y:", owner.final_y, "ray t:", owner.hit.t);
}
