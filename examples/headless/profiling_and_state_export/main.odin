package profiling_and_state_export

import "core:fmt"
import entasis "entasis:entasis"

main :: proc()
{
	owner: Case;
	defer case_destroy(&owner);
	if case_create(&owner) != .Ok
	{
		panic("profiling_and_state_export creation");
	}
	for _ in 0 ..< STEPS
	{
		if case_step(&owner) != .Ok
		{
			panic("profiling_and_state_export step");
		}
	}
	if case_check(&owner) != .Ok
	{
		panic("profile/state export");
	}
	fmt.println("profile stage:", entasis.profile_stage_text(entasis.Profile_Stage.Timestep),
		"ns:", owner.profile.stage_durations_nanoseconds[entasis.Profile_Stage.Timestep], "state checksum:", owner.checksum);
}
