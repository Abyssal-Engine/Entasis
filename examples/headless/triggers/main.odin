package triggers

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
			panic("trigger operation");
		}
		if c.cycle == 0
		{
			for event in c.events[:c.written]
			{
				fmt.printf("step=%d kind=%v sensor_user=%d lifetimes=(%d,%d)\n", event.step, event.kind, event.pair.user_a, event.pair.token_a, event.pair.token_b);
			}
		}
	}
	fmt.printf("mixed body: parent=%d part=%d contacts=%d solid velocity=%g\n",
		c.result.events_written, c.result.part_events_written, c.result.contact_events_written, c.solid_velocity);
}
