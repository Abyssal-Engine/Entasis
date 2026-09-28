package falling_box

import "core:fmt"
import entasis "entasis:entasis"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("falling box creation");
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
		panic("falling box result");
	}
	state: entasis.Body_State;
	state, _ = entasis.body_get(&owner.world, owner.body);
	fmt.println("falling box settled at y:", state.pose.position.y);
}
