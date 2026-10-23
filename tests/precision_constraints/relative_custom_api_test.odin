package precision_constraints_tests

import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_custom_public_registration_frame_admission_and_provider_match :: proc(t: ^testing.T)
{
	world: e.World;
	description: e.World_Description = e.world_description_default();
	description.gravity = {};
	description.damping = {};
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	defer e.world_destroy(&world);
	access: [4]e.Body_Access_Mask = {e.BODY_ACCESS_ALL, e.BODY_ACCESS_ALL, {}, {}};
	registration: e.Custom_Constraint_Registration = e.custom_constraint_registration(
		Relative_Probe_Description, Relative_Probe_Prestep, Relative_Probe_Impulses,
		e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, 2, access, access, relative_probe_validate, relative_probe_kernel,
		position_space=.Body_A_Relative,
	);
	registration.position_space = e.Constraint_Position_Space(255);
	testing.expect_value(t, e.custom_constraint_register(&world, registration), e.Status.Invalid_Description);
	registration.position_space = .Body_A_Relative;
	testing.expect_value(t, e.custom_constraint_register(&world, registration), e.Status.Ok);
	probe_context: Relative_Probe_Context = {marker=7};
	contextual: e.Contextual_Custom_Constraint_Registration = e.custom_constraint_registration_contextual(
		Relative_Probe_Description, Relative_Probe_Prestep, Relative_Probe_Impulses,
		e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID + 1, 2, access, access, &probe_context,
		relative_probe_validate_contextual, relative_probe_kernel_contextual, position_space=.Body_A_Relative,
	);
	contextual.position_space = e.Constraint_Position_Space(254);
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, contextual), e.Status.Invalid_Description);
	contextual.position_space = .Body_A_Relative;
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, contextual), e.Status.Ok);
	simulation: ^p.Simulation;
	status: e.Status;
	simulation, status = e.world_borrow_simulation(&world);
	testing.expect_value(t, status, e.Status.Ok);
	for id in int(e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID) ..= int(e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID + 1)
	{
		testing.expect_value(t, simulation.solver.registry.records[id].position_space, p.Constraint_Position_Space.Body_A_Relative);
	}
	testing.expect_value(t, e.world_enable_joint_breaks(&world, 16), e.Status.Ok);
	probe: Relative_Reaction_Probe;
	provider: e.Joint_Reaction_Provider = {read=relative_reaction_provider, user_context=&probe};
	testing.expect_value(t, e.constraint_set_reaction_provider(&world, registration.type_id, provider), e.Status.Invalid_Argument);
	provider.position_space = .Body_A_Relative;
	testing.expect_value(t, e.constraint_set_reaction_provider(&world, registration.type_id, provider), e.Status.Ok);
	testing.expect_value(t, e.constraint_set_reaction_provider(&world, contextual.type_id, provider), e.Status.Ok);
	for kind in 0 ..< 2
	{
		handles: [2]e.Body_Handle;
		for body in 0 ..< 2
		{
			body_description: e.Body_Description = e.body_dynamic({}, {inverse_mass=1, inverse_inertia_tensor={1, 0, 1, 0, 0, 1}},
				e.pose(), {}, e.body_activity(-1, 255));
			handles[body], status = e.body_add_global(&world, body_description, {1e9 + f64(kind) * 16 + f64(body) / 512, -1e9, 1e9});
			testing.expect_value(t, status, e.Status.Ok);
		}
		joint: e.Constraint_Handle;
		joint_description: Relative_Probe_Description = {2, 3, 3};
		joint, status = e.custom_constraint_add_typed(&world, handles[:], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID + e.Constraint_Type_ID(kind), &joint_description);
		testing.expect_value(t, status, e.Status.Ok);
		output: p.Constraint_Reaction_Sample;
		testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 1, provider, &output), p.Physics_Status.Ok);
		testing.expect_value(t, output.forces[1].linear, util.Vector3{1.0 / 512, 0, 0});
	}
	testing.expect_value(t, e.world_step(&world, 1.0 / 64), e.Status.Ok);
	// provider removal remains possible without inventing a second frame owner
	testing.expect_value(t, e.constraint_set_reaction_provider(&world, registration.type_id, {}), e.Status.Ok);
}

@(test)
precision_custom_low_level_invalid_frame_rejected_before_registration :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	before: i32 = scene.simulation.solver.registry.registered_count;
	record: p.Constraint_Type_Record = relative_probe_record(2, 3, 3);
	record.position_space = p.Constraint_Position_Space(99);
	testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Invalid_Argument);
	testing.expect_value(t, scene.simulation.solver.registry.registered_count, before);
	testing.expect_value(t, scene.simulation.solver.registry.records[record.type_id].registration, p.Reference_State.Missing);
	record.position_space = .Body_A_Relative;
	testing.expect_value(t, p.solver_register_constraint_type(&scene.simulation.solver, record), p.Physics_Status.Ok);
}
