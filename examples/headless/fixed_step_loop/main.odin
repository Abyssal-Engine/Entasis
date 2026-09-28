package fixed_step_loop

import "core:fmt"
import entasis "entasis:entasis"

main :: proc()
{
	c: Case;
	if case_create(&c)!=.Ok
	{
		panic("world setup");
	}
	defer case_destroy(&c);
	for _ in 0 ..< STEPS
	{
		if case_step(&c)!=.Ok
		{
			panic("fixed update");
		}
	}
	if case_check(&c)!=.Ok
	{
		panic("fixed-step invariant");
	}
	state: entasis.Body_State;
	state, _ = entasis.body_get(&c.world, c.body);
	fmt.println("fixed steps:", c.total_steps, "alpha:", c.alpha, "x:", state.pose.position.x);
}
