package public_api_tests

import "core:testing"
import entasis "entasis:entasis"

@(test)
default_runtime_initializes_steps_and_disposes :: proc(t: ^testing.T)
{
	world: entasis.World;
	description := small_world_description();
	if !testing.expect_value(
		t, entasis.world_init(&world, description), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect_value(t, entasis.world_step(&world, 1.0 / 60.0), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Disposed);
}
