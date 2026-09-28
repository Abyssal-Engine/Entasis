package impulses_solver_contacts

import "core:fmt"

main :: proc()
{
	c: Case;
	defer case_destroy(&c);
	if case_create(&c) != .Ok
	{
		panic("create");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&c) != .Ok
		{
			panic("operation");
		}
	}
	fmt.println("contacts:", c.events[0].contact_data.contact_count, "feature:", c.events[0].contact_data.contacts[0].feature_id, "normal impulse:", c.events[0].contact_data.contacts[0].normal_impulse);
}
