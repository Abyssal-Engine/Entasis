package physics_scene

Playback :: enum
{
	Paused, Running, Completed,
}
Loop :: enum
{
	Off, On,
}
Cursor :: struct
{
	index, count: int,
	playback: Playback,
	loop: Loop,
	speed: int,
	accumulator: f64,
}
SPEEDS :: [6]f64{0.1, 0.25, 0.5, 1, 2, 4};

cursor_seek :: proc(c: ^Cursor, index: int) -> Status
{
	if index < 0 || index >= c.count
	{
		return .Invalid_Data;
	}
	c.index = index;
	c.accumulator = 0;
	c.playback = .Paused;
	return .Ok;
}

cursor_advance :: proc(c: ^Cursor, elapsed, interval: f64, maximum_steps: int) -> int
{
	if c.playback != .Running
	{
		c.accumulator = 0;
		return 0;
	}
	speeds: [6]f64 = SPEEDS;
	c.accumulator += elapsed * speeds[c.speed];
	steps: int;
	for c.accumulator >= interval && steps < maximum_steps
	{
		if c.index+1 >= c.count
		{
			if c.loop == .Off
			{
				c.playback = .Completed;
				c.accumulator = 0;
				break;
			}
			c.index = -1;
		}
		c.index += 1;
		steps += 1;
		c.accumulator -= interval;
	}
	return steps;
}

Configuration_Comparison :: enum
{
	Unavailable, Matching, Different,
}

comparison_configuration :: proc(a, b: Metadata) -> Configuration_Comparison
{
	if a.settings_state == .Unavailable || b.settings_state == .Unavailable
	{
		return .Unavailable;
	}
	if a.settings != b.settings || a.configuration != b.configuration || a.components != b.components
	{
		return .Different;
	}
	return .Matching;
}

comparison_admit :: proc(a, b: Metadata) -> Status
{
	if len(a.scenario) == 0 || len(b.scenario) == 0 || a.scenario != b.scenario || a.axis != b.axis || a.timestep != b.timestep
	{
		return .Unsupported;
	}
	return .Ok;
}
