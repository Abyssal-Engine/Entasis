package physics_scenarios

import observation "../physics_observation"

session_bytes :: proc(s: ^Session) -> u64
{
	return observation.observation_bytes(&s.observation)+example_fixture_size(s.recipe.scenario)+size_of(Session)-size_of(observation.Observation)-size_of(s.pool);
}
