package scene_stability

import "core:math"
import "core:os"
import "core:strings"
import "core:testing"
import entasis "entasis:entasis"
import observations "../scene_observation"
import support "../benchmark_support"

@(test)
observation_reports_collapsed_island :: proc(t: ^testing.T)
{
	intact, collapsed: Observation
	for height: int = 0; height < 8; height += 1
	{
		initial: entasis.Vector3 = {0, 0.5 + f32(height), 0}
		state: entasis.Body_State = {pose=entasis.pose(initial)}
		observations.observe_body(&intact, state, initial, {1, 1, 1}, {}, {6, 6})
		state.pose.position.y = 0.5
		observations.observe_body(&collapsed, state, initial, {1, 1, 1}, {}, {6, 6})
	}
	testing.expect_value(t, intact.sum_y / intact.initial_sum_y, f64(1))
	testing.expect(t, collapsed.sum_y / collapsed.initial_sum_y < 0.95)
	testing.expect_value(t, collapsed.dropped, 6)
}

@(test)
observation_uses_rotated_floor_extent :: proc(t: ^testing.T)
{
	state: entasis.Body_State = {pose=entasis.pose({0, 1, 0}, {z=math.sqrt(f32(0.5)), w=math.sqrt(f32(0.5))})}
	observation: Observation
	observations.observe_body(&observation, state, {0, 1, 0}, {4, 1, 1}, {}, {6, 6})
	testing.expect(t, math.abs(observation.maximum_floor_penetration - 1) < 0.000001)
	testing.expect(t, math.abs(observation.apex - 3) < 0.000001)
}

@(test)
observation_rejects_nonfinite_state :: proc(t: ^testing.T)
{
	for component: int = 0; component < 3; component += 1
	{
		state: entasis.Body_State = {pose=entasis.pose({0, 0.5, 0})}
		switch component
		{
			case 0:
			state.pose.position.x = transmute(f32)u32(0x7fc0_0000)
			case 1:
			state.pose.orientation.w = transmute(f32)u32(0x7f80_0000)
			case 2:
			state.velocity.linear.z = transmute(f32)u32(0xff80_0000)
		}
		observation: Observation
		observations.observe_body(&observation, state, {0, 0.5, 0}, {1, 1, 1}, {}, {6, 6})
		testing.expect_value(t, observation.invalid, 1)
		testing.expect_value(t, observation.count, 1)
	}
}

@(test)
observation_rejects_failed_output :: proc(t: ^testing.T)
{
	file: ^os.File
	error: os.Error
	file, error = os.create_temp_file("build/tests", "observation-output-")
	testing.expect_value(t, error, os.Error(nil))
	if error != nil
	{
		return
	}
	path: string = strings.clone(os.name(file))
	defer delete(path)
	defer os.remove(path)
	testing.expect_value(t, os.close(file), os.Error(nil))
	file, error = os.open(path, {.Read})
	testing.expect_value(t, error, os.Error(nil))
	if error != nil
	{
		return
	}
	defer os.close(file)
	observation: Observation = {count=1, sum_y=0.5, initial_sum_y=0.5}
	testing.expect_value(t, write_header(file), support.Case_Status.Output_Failed)
	testing.expect_value(t, write_observation(file, observation, 0, 60, 0, 1, 1), support.Case_Status.Output_Failed)
	testing.expect_value(t, write_snapshot(file, nil, nil), support.Case_Status.Output_Failed)
}
