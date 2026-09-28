package physics_scenarios

import scene "../physics_scene"

session_settings :: proc(s: ^Session, buffer: []u8) -> (string, scene.Availability, scene.Status)
{
	return example_settings(s, buffer);
}
