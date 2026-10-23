package precision_constraints_tests

import "core:testing"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Relative_Reaction_Probe :: struct
{
	calls, fail_at: int,
}

relative_reaction_provider :: proc "contextless" (user: rawptr, input: ^p.Constraint_Reaction_Provider_Input,
	output: ^[4]p.Constraint_Impulse_Wrench) -> p.Physics_Status
{
	state: ^Relative_Reaction_Probe = (^Relative_Reaction_Probe)(user);
	state.calls += 1;
	if state.fail_at != 0 && state.calls >= state.fail_at
	{
		return .Capacity_Missing;
	}
	for body in 0 ..< int(input.body_count)
	{
		output[body].linear = input.poses[body].position;
		output[body].angular = {input.poses[body].orientation.x, input.poses[body].orientation.y, input.poses[body].orientation.z};
	}
	return .Ok;
}

@(test)
precision_custom_reaction_frame_match_and_failed_output_are_explicit :: proc(t: ^testing.T)
{
	for space in ([2]p.Constraint_Position_Space{.Absolute_Float, .Body_A_Relative})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		record: p.Constraint_Type_Record = relative_probe_record(4, 15, 15, space);
		testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
		fixture: Joint_Fixture;
		relative_probe_populate(t, &scene, record, &fixture, 1e9, 9);
		state: Relative_Reaction_Probe;
		provider: p.Constraint_Reaction_Provider = {read=relative_reaction_provider, user_context=&state, position_space=space};
		for group in 0 ..< 9
		{
			output: p.Constraint_Reaction_Sample = {maximum_force=123};
			testing.expect_value(t, p.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[group], 0.5, provider, &output), p.Physics_Status.Ok);
			for body in 0 ..< 4
			{
				expected: util.Vector3 = {f32(body) / 256, -f32(body) / 128, f32(body) / 64};
				if space == .Absolute_Float
				{
					position: util.Vector3 = body_motion(t, &scene, fixture.bodies[group][body]).pose.position;
					expected = {position.x * 2, position.y * 2, position.z * 2};
				}
				testing.expect_value(t, output.forces[body].linear, expected);
			}
			before: p.Constraint_Reaction_Sample = output;
			provider.position_space = .Body_A_Relative if space == .Absolute_Float else .Absolute_Float;
			calls: int = state.calls;
			testing.expect_value(t, p.constraint_capture_reaction(&scene.simulation.solver, fixture.joints[group], 0.5, provider, &output), p.Physics_Status.Invalid_Argument);
			testing.expect_value(t, output, before);
			testing.expect_value(t, state.calls, calls);
			provider.position_space = space;
		}
		scene_dispose(t, &scene);
	}
}

@(test)
precision_custom_relative_reaction_all_masks_and_batch_atomic_failure :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	record: p.Constraint_Type_Record = relative_probe_record(4, 15, 15);
	testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
	fixture: Joint_Fixture;
	relative_probe_populate(t, &scene, record, &fixture, 1e9, 9);
	state: Relative_Reaction_Probe;
	provider: p.Constraint_Reaction_Provider = {read=relative_reaction_provider, user_context=&state, position_space=.Body_A_Relative};
	for mask in 0 ..< 256
	{
		output: [8]p.Constraint_Reaction_Sample;
		output[0].maximum_force = 123;
		before: [8]p.Constraint_Reaction_Sample = output;
		status: p.Physics_Status = p.constraint_capture_reaction_bundle(&scene.simulation.solver, fixture.joints[0], u8(mask), 1, provider, &output);
		if mask == 0
		{
			testing.expect_value(t, status, p.Physics_Status.Invalid_Argument);
			testing.expect_value(t, output, before);
			continue;
		}
		testing.expect_value(t, status, p.Physics_Status.Ok);
		for lane in 0 ..< 8
		{
			if u32(mask) & (u32(1) << u32(lane)) == 0
			{
				testing.expect_value(t, output[lane], p.Constraint_Reaction_Sample{});
				continue;
			}
			for body in 0 ..< 4
			{
				testing.expect_value(t, output[lane].forces[body].linear,
					util.Vector3{f32(body) / 512, -f32(body) / 256, f32(body) / 128});
			}
		}
	}
	output: [8]p.Constraint_Reaction_Sample;
	output[0].maximum_force = 123;
	before: [8]p.Constraint_Reaction_Sample = output;
	state = {fail_at=4};
	testing.expect_value(t, p.constraint_capture_reaction_bundle(&scene.simulation.solver, fixture.joints[0], 0xff, 1, provider, &output), p.Physics_Status.Capacity_Missing);
	testing.expect_value(t, output, before);
	testing.expect_value(t, state.calls, 4);
	state = {};
	testing.expect_value(t, p.constraint_capture_reaction_bundle(&scene.simulation.solver, fixture.joints[8], 1, 1, provider, &output), p.Physics_Status.Ok);
	testing.expect_value(t, output[0].body_count, i32(4));
	testing.expect_value(t, output[1], p.Constraint_Reaction_Sample{});
}
