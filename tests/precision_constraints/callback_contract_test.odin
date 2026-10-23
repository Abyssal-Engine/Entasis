package precision_constraints_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Position_Echo :: struct
{
	reserved: f32,
}

position_echo_validate :: proc "contextless" (type_id: i32, description: rawptr) -> physics.Physics_Status
{
	if type_id < physics.FIRST_CALLER_CONSTRAINT_TYPE_ID || description == nil
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

position_echo_kernel :: proc "contextless" (prestep: rawptr, bodies: ^[4]physics.Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32, impulses: rawptr, mask: util.I32x8, phase: physics.Constraint_Kernel_Phase)
{
	_, _, _, _ = prestep, dt, inverse_dt, impulses;
	if phase == .Solve
	{
		for index in 0 ..< 2
		{
			bodies[index].linear_velocity = util.vector3_wide_select(mask, bodies[index].position, bodies[index].linear_velocity);
		}
	}
}

position_echo_reaction :: proc "contextless" (user_context: rawptr, input: ^physics.Constraint_Reaction_Provider_Input,
	output: ^[4]physics.Constraint_Impulse_Wrench) -> physics.Physics_Status
{
	_ = user_context;
	for index in 0 ..< int(input.body_count)
	{
		output[index].linear = input.poses[index].position;
	}
	return .Ok;
}

@(test)
precision_caller_constraints_and_reaction_providers_keep_absolute_float_contract :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	access: [4]physics.Body_Access_Mask = {physics.BODY_ACCESS_ALL, physics.BODY_ACCESS_ALL, {}, {}};
	record: physics.Constraint_Type_Record = {
		type_id=physics.FIRST_CALLER_CONSTRAINT_TYPE_ID, body_count=2,
		description_size=size_of(Position_Echo), prestep_bundle_size=size_of(util.F32x8), impulse_bundle_size=size_of(util.F32x8),
		initial_access=access, solve_access=access,
		apply_description=physics.constraint_description_transfer, build_description=physics.constraint_description_build,
		validate_description=position_echo_validate, prestep_warmstart_solve=position_echo_kernel, incrementally_update=position_echo_kernel,
		move_record=physics.constraint_storage_move_lane, remove_record=physics.constraint_storage_remove_lane,
	};
	testing.expect_value(t, physics.solver_register_constraint_type(&scene.simulation.solver, record), physics.Physics_Status.Ok);
	handles: [9][4]physics.Body_Handle;
	joints: [9]physics.Constraint_Handle;
	positions: [9][2]util.Vector3;
	for group in 0 ..< 9
	{
		for body in 0 ..< 2
		{
			position: physics.Global_Position = {1e9 + f64(group * 256) + f64(body) * 0.002, -1e9 + f64(body) * 0.003, 1e6 + f64(group)};
			handles[group][body] = add_body(t, &scene, position);
			positions[group][body] = {f32(position.x), f32(position.y), f32(position.z)};
		}
		description: Position_Echo;
		status: physics.Physics_Status;
		joints[group], status = physics.solver_add_raw(&scene.simulation.solver, record.type_id, &handles[group], &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	batch: ^physics.Type_Batch = constraint_batch(t, &scene, joints[0]);
	for route in 0 ..< 2
	{
		if route == 0
		{
			for bundle in 0 ..< 2
			{
				testing.expect_value(t, physics.solver_execute_type_bundle_dispatch_integration(&scene.simulation.solver, batch, &record,
					0, bundle, 1.0 / 64, 64, .First, .Solve, 0, .Native, .Never), physics.Physics_Status.Ok);
			}
		}
		else
		{
			for lane in 0 ..< 9
			{
				testing.expect_value(t, physics.solver_execute_type_lane_dispatch_integration(&scene.simulation.solver, batch, &record,
					0, lane, 1.0 / 64, 64, .First, .Solve, 0, .Native, .Never), physics.Physics_Status.Ok);
			}
		}
		for group in 0 ..< 9
		{
			reaction: physics.Constraint_Reaction_Sample;
			provider: physics.Constraint_Reaction_Provider = {read=position_echo_reaction};
			testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, joints[group], 1, provider, &reaction), physics.Physics_Status.Ok);
			for body in 0 ..< 2
			{
				testing.expect_value(t, body_motion(t, &scene, handles[group][body]).velocity.linear, positions[group][body]);
				testing.expect_value(t, reaction.forces[body].linear, positions[group][body]);
				body_motion(t, &scene, handles[group][body]).velocity = {};
			}
		}
	}
}
