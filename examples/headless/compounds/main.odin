package compounds

import "core:fmt"
import entasis "entasis:entasis"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("compounds creation");
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
		panic("compounds result");
	}
	standard_info, big_info: entasis.Shape_Info;
	standard_info, _ = entasis.shape_inspect(&owner.world, owner.compound.shape);
	big_info, _ = entasis.shape_inspect(&owner.world, owner.big.shape);
	fmt.println("compound type:", standard_info.type_id, "big type:", big_info.type_id, "center:", owner.compound.center);
}
