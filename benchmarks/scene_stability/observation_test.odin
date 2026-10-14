package scene_stability

import "core:math"
import "base:runtime"
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

qualification_fixture :: proc(fixture: observations.Fixture, steps: int = 61) -> observations.Fixture_Configuration
{
	c: observations.Fixture_Configuration = {fixture=fixture, half_extent={0.5, 0.5, 0.5}, grid={2, 2, 2}, spacing={1, 1, 1},
		island_grid={1, 1}, island_spacing={12, 12}, floor_half_size={6, 6}, spawn_height=0.5, rows=2, horizontal_spacing=1,
		container_half_size={5, 5}, container_rim=4, steps=steps, timestep_hz=60, population=8, body_count=8}
	if fixture == .Pyramid
	{
		c.population = 5
		c.body_count = 5
	}
	if fixture == .Pyramid_Wall
	{
		c.population = 3
		c.body_count = 3
	}
	if fixture == .Container
	{
		c.population = 1
		c.body_count = 1
	}
	return c
}

qualification_test_create :: proc(t: ^testing.T, q: ^observations.Qualification, c: observations.Fixture_Configuration, mode: observations.Verification_Mode = .On)
{
	testing.expect_value(t, observations.qualification_create(q, c, mode), runtime.Allocator_Error(nil))
	if mode == .Off
	{
		return
	}
	if c.fixture == .Contact_Islands
	{
		for &state, index in q.current
		{
			x, y, z: int = index%c.grid[0], index/(c.grid[0]*c.grid[2]), (index/c.grid[0])%c.grid[2]
			state.pose = entasis.pose({(f32(x)-0.5*f32(c.grid[0]-1))*f32(c.spacing[0]), f32(c.spawn_height)+f32(y)*f32(c.spacing[1]), (f32(z)-0.5*f32(c.grid[2]-1))*f32(c.spacing[2])})
		}
	}
	else if c.fixture == .Pyramid || c.fixture == .Pyramid_Wall
	{
		index: int
		for layer: int = 0; layer < c.rows; layer += 1
		{
			side: int = c.rows-layer
			depths: int = side if c.fixture == .Pyramid else 1
			for z: int = 0; z < depths; z += 1
			{
				for x: int = 0; x < side; x += 1
				{
					q.current[index].pose = entasis.pose({(f32(x)-0.5*f32(side-1))*f32(c.horizontal_spacing), f32(c.half_extent[1])+f32(layer)*f32(2*c.half_extent[1]), (f32(z)-0.5*f32(depths-1))*f32(c.horizontal_spacing)})
					index += 1
				}
			}
		}
	}
	else
	{
		q.current[0].pose = entasis.pose({0, 6, 0})
	}
	for projectile: int = c.population; projectile < c.body_count; projectile += 1
	{
		q.current[projectile].pose = entasis.pose({0, 2, 20})
	}
	observations.qualification_bind(q, .Construction)
}

qualification_test_finish :: proc(q: ^observations.Qualification)
{
	for step: int = 1; step <= q.configuration.steps; step += 1
	{
		observations.qualification_assess(q, .Measured, step)
	}
	observations.qualification_complete(q)
}

@(test)
qualification_intact_boxes_and_threshold_boundaries :: proc(t: ^testing.T)
{
	for scale in ([3]f64{0.5, 1, 2})
	{
		c: observations.Fixture_Configuration = qualification_fixture(.Contact_Islands)
		c.half_extent *= scale
		c.spacing *= scale
		c.spawn_height *= scale
		q: observations.Qualification
		qualification_test_create(t, &q, c)
		qualification_test_finish(&q)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
		limit: f64 = 0.1*scale
		reference: observations.Box_Reference = {lower={0, 0, 0}, upper={0, 0, 0}}
		for value in ([3]f64{limit-0.00001, limit, limit+0.00001})
		{
			excess: f64 = observations.box_slot_excess({value, 0, 0}, c.half_extent, c.half_extent, reference)
			testing.expect(t, math.abs(excess-value) < 1e-12)
			testing.expect_value(t, excess > limit, value > limit)
		}
		observations.qualification_destroy(&q)
	}
	q: observations.Qualification
	qualification_test_create(t, &q, qualification_fixture(.Pyramid))
	testing.expect_value(t, q.result.shape_limit, f64(0.1))
	q.current[4].pose.position.x += 0.101
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
	observations.qualification_destroy(&q)
}

@(test)
qualification_rotated_slots_and_neighbor_compression :: proc(t: ^testing.T)
{
	state: entasis.Body_State = {pose=entasis.pose({}, {z=math.sqrt(f32(0.5)), w=math.sqrt(f32(0.5))})}
	radius: [3]f64
	validity: observations.State_Status
	radius, validity = observations.projected_box_radius(state, {2, 0.5, 0.5})
	testing.expect_value(t, validity, observations.State_Status.Valid)
	testing.expect(t, math.abs(radius[1]-2) < 1e-6)
	state.pose.orientation.z *= 3
	state.pose.orientation.w *= 3
	radius, validity = observations.projected_box_radius(state, {2, 0.5, 0.5})
	testing.expect(t, math.abs(radius[1]-2) < 1e-6)
	for axis: int = 0; axis < 3; axis += 1
	{
		p: [3]f64
		p[axis] = 0.8
		testing.expect(t, math.abs(observations.assigned_compression({}, {0.5, 0.5, 0.5}, p, {0.5, 0.5, 0.5}, axis)-0.2) < 1e-12)
	}
	q: observations.Qualification
	qualification_test_create(t, &q, qualification_fixture(.Contact_Islands))
	q.current[1].pose.position.x -= 0.11
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
	testing.expect(t, q.result.maximum_neighbor.value > q.result.shape_limit)
	observations.qualification_destroy(&q)
}

@(test)
qualification_authored_gaps_and_support_planes :: proc(t: ^testing.T)
{
	c: observations.Fixture_Configuration = qualification_fixture(.Contact_Islands)
	c.spacing = {0.995, 1.01, 0.995}
	q: observations.Qualification
	qualification_test_create(t, &q, c)
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
	testing.expect(t, q.references[0].lower[0] < q.references[0].upper[0])
	testing.expect(t, math.abs(q.result.maximum_neighbor.value-0.005) < 1e-6)
	observations.qualification_destroy(&q)
	c = qualification_fixture(.Pyramid)
	qualification_test_create(t, &q, c)
	q.current[4].pose.position.y = 0.9
	observations.qualification_assess(&q, .Measured, 1)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
	observations.qualification_destroy(&q)
	c.spawn_height = 1
	c.fixture = .Contact_Islands
	testing.expect_value(t, observations.fixture_eligibility(c), observations.Eligibility.Unsupported)
	c = qualification_fixture(.Contact_Islands)
	c.floor_top = 4
	c.spawn_height = 4.5
	qualification_test_create(t, &q, c)
	testing.expect_value(t, q.references[0].lower[1], f64(4.5))
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
	observations.qualification_destroy(&q)
	c.spawn_height += 0.0000001
	testing.expect_value(t, observations.fixture_eligibility(c), observations.Eligibility.Unsupported)
	for phase in ([2]observations.Assessment_Phase{.Warmup, .Measured})
	{
		for body in ([2]int{0, 4})
		{
			c = qualification_fixture(.Contact_Islands, 62)
			c.floor_top, c.spawn_height, c.warmup_steps = 4, 4.5, 1
			qualification_test_create(t, &q, c)
			initial: entasis.Body_State = q.current[body]
			plane: f32 = 4 if body == 0 else 5
			if phase == .Warmup
			{
				q.current[body].pose.position.y = plane-0.125
			}
			observations.qualification_assess(&q, .Warmup, 1)
			q.current[body] = initial
			for step: int = 1; step <= c.steps; step += 1
			{
				if phase == .Measured && step == 1
				{
					q.current[body].pose.position.y = plane-0.125
				}
				observations.qualification_assess(&q, .Measured, step)
				q.current[body] = initial
			}
			observations.qualification_complete(&q)
			testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
			testing.expect_value(t, q.result.reason, observations.Breach_Reason.Support_Plane)
			testing.expect_value(t, q.result.coverage, observations.Coverage.Complete)
			testing.expect_value(t, q.result.first.phase, phase)
			testing.expect_value(t, q.result.first.step, 1)
			testing.expect_value(t, q.result.first.body, body)
			testing.expect_value(t, q.result.first.value, f64(0.125))
			testing.expect_value(t, q.result.maximum_slot.value, f64(0))
			observations.qualification_destroy(&q)
		}
	}
	c = qualification_fixture(.Contact_Islands, 62)
	c.grid, c.population, c.body_count = {1, 1, 1}, 1, 1
	testing.expect_value(t, observations.qualification_create(&q, c, .On), runtime.Allocator_Error(nil))
	q.current[0].pose = entasis.pose({0, -0.125, 0})
	observations.qualification_bind(&q, .Construction)
	q.current[0].pose.position.y = 0.5
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
	testing.expect_value(t, q.result.reason, observations.Breach_Reason.Support_Plane)
	testing.expect_value(t, q.result.first.phase, observations.Assessment_Phase.Construction)
	testing.expect_value(t, q.result.coverage, observations.Coverage.Complete)
	observations.qualification_destroy(&q)
}

@(test)
qualification_pyramid_and_wall_support_identity :: proc(t: ^testing.T)
{
	for fixture in ([2]observations.Fixture{.Pyramid, .Pyramid_Wall})
	{
		q: observations.Qualification
		qualification_test_create(t, &q, qualification_fixture(fixture))
		last: int = q.configuration.population-1
		reference: observations.Box_Reference = q.references[last]
		testing.expect_value(t, reference.support_count, 4 if fixture == .Pyramid else 2)
		for index: int = 0; index < reference.neighbor_count; index += 1
		{
			testing.expect_value(t, reference.neighbors[index].body, index)
			testing.expect_value(t, reference.neighbors[index].axis, 1)
		}
		qualification_test_finish(&q)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
		observations.qualification_destroy(&q)
	}
	c: observations.Fixture_Configuration = qualification_fixture(.Pyramid, 2)
	c.rows, c.population, c.body_count = 1, 1, 1
	q: observations.Qualification
	qualification_test_create(t, &q, c)
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
	testing.expect_value(t, q.result.maximum_neighbor.evidence, observations.Evidence.Absent)
	observations.qualification_destroy(&q)

}

@(test)
qualification_invalid_pose_and_velocity_fail :: proc(t: ^testing.T)
{
	for field: int = 0; field < 4; field += 1
	{
		q: observations.Qualification
		qualification_test_create(t, &q, qualification_fixture(.Contact_Islands, 62))
		switch field
		{
			case 0: q.current[0].pose.orientation = {}
			case 1: q.current[0].pose.position.x = transmute(f32)u32(0x7f80_0000)
			case 2: q.current[0].velocity.linear.y = transmute(f32)u32(0x7fc0_0000)
			case 3: q.current[0].velocity.angular.z = transmute(f32)u32(0xff80_0000)
		}
		observations.qualification_assess(&q, .Measured, 1)
		q.current[0] = {pose=entasis.pose({0, 0.5, 0})}
		observations.qualification_assess(&q, .Measured, 2)
		testing.expect_value(t, q.result.reason, observations.Breach_Reason.Invalid_State)
		testing.expect_value(t, q.result.first.step, 1)
		observations.qualification_destroy(&q)
	}
}

@(test)
qualification_container_entry_and_escape :: proc(t: ^testing.T)
{
	q: observations.Qualification
	qualification_test_create(t, &q, qualification_fixture(.Container, 4))
	q.current[0].pose.position = {4.8, 3, 0}
	observations.qualification_assess(&q, .Measured, 1)
	testing.expect_value(t, q.entries[0], observations.Container_Entry.Outside)
	q.current[0].pose.position = {0, 3, 0}
	observations.qualification_assess(&q, .Measured, 2)
	testing.expect_value(t, q.entries[0], observations.Container_Entry.Entered)
	q.current[0].pose.position.y = 4.4
	observations.qualification_assess(&q, .Measured, 3)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Unassessed)
	q.current[0].pose.position.y = 4.6
	observations.qualification_assess(&q, .Measured, 4)
	observations.qualification_complete(&q)
	testing.expect_value(t, q.result.reason, observations.Breach_Reason.Container_Escape)
	testing.expect_value(t, q.result.first.step, 4)
	observations.qualification_destroy(&q)
	qualification_test_create(t, &q, qualification_fixture(.Container, 2))
	q.current[0].pose.position = {0, 0.499, 0}
	observations.qualification_assess(&q, .Measured, 1)
	testing.expect_value(t, q.entries[0], observations.Container_Entry.Entered)
	q.current[0].pose.position.y = 4.6
	observations.qualification_assess(&q, .Measured, 2)
	observations.qualification_complete(&q)
	testing.expect_value(t, q.result.reason, observations.Breach_Reason.Container_Escape)
	observations.qualification_destroy(&q)

}

@(test)
qualification_complete_terminal_window :: proc(t: ^testing.T)
{
	for sequence: int = 0; sequence < 4; sequence += 1
	{
		q: observations.Qualification
		qualification_test_create(t, &q, qualification_fixture(.Contact_Islands, 62 if sequence != 3 else 60))
		for step: int = 1; step <= q.configuration.steps; step += 1
		{
			if sequence == 1 && step == 3
			{
				continue
			}
			observations.qualification_assess(&q, .Measured, step)
			if sequence == 2 && step == 3
			{
				observations.qualification_assess(&q, .Measured, step)
			}
		}
		observations.qualification_complete(&q)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Pass if sequence == 0 else observations.Physical_Status.Unassessed)
		if sequence == 0
		{
			testing.expect_value(t, q.result.terminal_samples, 61)
		}
		observations.qualification_destroy(&q)
	}
}

@(test)
qualification_projectile_cutoff_preserves_failure :: proc(t: ^testing.T)
{
	testing.expect_value(t, observations.possible_projectile_impact({4, 0, 0}, {-4, 0, 0}, 1), observations.State_Status.Invalid)
	testing.expect_value(t, observations.possible_projectile_impact({4, 2, 0}, {-4, 2, 0}, 1), observations.State_Status.Valid)
	testing.expect_value(t, observations.possible_projectile_impact({0.5, 0, 0}, {0.5, 0, 0}, 1), observations.State_Status.Invalid)
	for prior_failure: int = 0; prior_failure < 2; prior_failure += 1
	{
		c: observations.Fixture_Configuration = qualification_fixture(.Pyramid, 2)
		c.body_count += 1
		c.projectile_radius = 0.5
		q: observations.Qualification
		qualification_test_create(t, &q, c)
		if prior_failure == 1
		{
			q.current[4].pose.position.x += 0.2
		}
		observations.qualification_assess(&q, .Measured, 1)
		q.current[5].pose.position = {0, 1.5, 0}
		q.current[4].pose.position.x += 1
		observations.qualification_assess(&q, .Measured, 2)
		observations.qualification_complete(&q)
		testing.expect_value(t, q.result.impact.step, 2)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Fail if prior_failure == 1 else observations.Physical_Status.Pass)
		if prior_failure == 1
		{
			testing.expect_value(t, q.result.first.step, 1)
		}
		observations.qualification_destroy(&q)
	}
	for impact_step: int = 1; impact_step <= 2; impact_step += 1
	{
		c: observations.Fixture_Configuration = qualification_fixture(.Pyramid, 2)
		c.body_count += 1
		c.projectile_radius = 0.5
		q: observations.Qualification
		qualification_test_create(t, &q, c)
		for step: int = 1; step <= c.steps; step += 1
		{
			if step == impact_step
			{
				q.current[c.population].pose.position = {0, 1.5, 0}
				q.current[4].pose.position.x += 1
			}
			observations.qualification_assess(&q, .Measured, step)
		}
		observations.qualification_complete(&q)
		testing.expect_value(t, q.result.coverage, observations.Coverage.Complete)
		testing.expect_value(t, q.result.impact.step, impact_step)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Unassessed if impact_step == 1 else observations.Physical_Status.Pass)
		testing.expect_value(t, q.result.reason, observations.Breach_Reason.Impact_Before_Assessment if impact_step == 1 else observations.Breach_Reason.None)
		testing.expect_value(t, q.result.maximum_slot.value, f64(0))
		observations.qualification_destroy(&q)
	}
	c: observations.Fixture_Configuration = qualification_fixture(.Pyramid, 2)
	c.body_count += 1
	c.projectile_radius = 0.5
	q: observations.Qualification
	testing.expect_value(t, observations.qualification_create(&q, c, .On), runtime.Allocator_Error(nil))
	positions: [6]entasis.Vector3 = {{-0.5, 0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}, {0, 1.5, 0}, {0, 1.5, 0}}
	for position, index in positions
	{
		q.current[index].pose = entasis.pose(position)
	}
	observations.qualification_bind(&q, .Construction)
	testing.expect_value(t, q.result.impact.step, 0)
	q.current[0].velocity.angular.x = transmute(f32)u32(0x7f80_0000)
	observations.qualification_assess(&q, .Measured, 1)
	testing.expect_value(t, q.result.reason, observations.Breach_Reason.Invalid_State)
	observations.qualification_destroy(&q)

}

@(test)
qualification_off_and_unsupported_are_unassessed :: proc(t: ^testing.T)
{
	for mode in observations.Verification_Mode
	{
		c: observations.Fixture_Configuration = qualification_fixture(.Pyramid)
		c.horizontal_spacing = 2
		q: observations.Qualification
		qualification_test_create(t, &q, c, mode)
		qualification_test_finish(&q)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Unassessed)
		testing.expect_value(t, q.result.reason, observations.Breach_Reason.Verification_Off if mode == .Off else observations.Breach_Reason.Unsupported_Fixture)
		observations.qualification_destroy(&q)
	}
	c: observations.Fixture_Configuration = qualification_fixture(.Pyramid)
	c.half_extent = {1e-7, 1e-7, 1e-7}
	c.horizontal_spacing = 1e-7
	testing.expect_value(t, observations.fixture_eligibility(c), observations.Eligibility.Unsupported)
	for axis: int = 0; axis < 3; axis += 1
	{
		c = qualification_fixture(.Container, 2)
		c.half_extent[axis] = 1
		q: observations.Qualification
		qualification_test_create(t, &q, c)
		qualification_test_finish(&q)
		testing.expect_value(t, q.eligibility, observations.Eligibility.Unsupported)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Unassessed)
		testing.expect_value(t, q.result.reason, observations.Breach_Reason.Unsupported_Fixture)
		testing.expect_value(t, q.result.coverage, observations.Coverage.Complete)
		observations.qualification_destroy(&q)
	}
	for half in ([3]f64{0.25, 0.5, 1})
	{
		c = qualification_fixture(.Container, 2)
		c.half_extent = {half, half, half}
		q: observations.Qualification
		qualification_test_create(t, &q, c)
		qualification_test_finish(&q)
		testing.expect_value(t, q.eligibility, observations.Eligibility.Supported)
		testing.expect_value(t, q.result.status, observations.Physical_Status.Pass)
		observations.qualification_destroy(&q)
	}
}

@(test)
qualification_world_reset_preserves_phase_evidence :: proc(t: ^testing.T)
{
	c: observations.Fixture_Configuration = qualification_fixture(.Container, 2)
	c.warmup_steps = 1
	q: observations.Qualification
	testing.expect_value(t, observations.qualification_create(&q, c, .On), runtime.Allocator_Error(nil))
	q.current[0].pose = entasis.pose({0, 2, 0})
	observations.qualification_bind(&q, .Warmup_Construction)
	q.current[0].velocity.linear.x = transmute(f32)u32(0x7f80_0000)
	observations.qualification_assess(&q, .Warmup, 1)
	q.current[0] = {pose=entasis.pose({0, 6, 0})}
	observations.qualification_bind(&q, .Construction)
	qualification_test_finish(&q)
	testing.expect_value(t, q.result.status, observations.Physical_Status.Fail)
	testing.expect_value(t, q.result.coverage, observations.Coverage.Complete)
	testing.expect_value(t, q.result.first.phase, observations.Assessment_Phase.Warmup)
	testing.expect_value(t, q.entries[0], observations.Container_Entry.Outside)
	testing.expect_value(t, q.result.measured_samples, 2)
	observations.qualification_destroy(&q)
}
