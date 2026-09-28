package public_api_tests

import "core:math"
import "core:simd"
import "core:testing"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

callback_expect_f32 :: proc(
	t: ^testing.T, actual, expected, tolerance: f32, label: string,
)
{
	difference := abs(actual - expected);
	testing.expectf(
		t, difference <= tolerance,
		"%s: expected %.9f, got %.9f, difference %.9f",
		label, expected, actual, difference,
	);
}

Callback_Policy_Call :: enum u8
{
	Initialize,
	Allow,
	Configure,
}

Mixed_Callback_Policy_State :: struct
{
	policy: entasis.Default_Narrow_Policy,
	calls:  [8]Callback_Policy_Call,
	count:  int,
}

mixed_callback_record :: proc "contextless" (
	state: ^Mixed_Callback_Policy_State, call: Callback_Policy_Call,
)
{
	if state == nil || state.count >= len(state.calls)
	{
		return;
	}
	state.calls[state.count] = call;
	state.count += 1;
}

mixed_callback_initialize :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation,
) -> entasis.Status
{
	_ = simulation;
	state := (^Mixed_Callback_Policy_State)(user_context);
	if state == nil
	{
		return .Invalid_Argument;
	}
	mixed_callback_record(state, .Initialize);
	return physics.constraint_contact_material_validate(state.policy.material);
}

mixed_callback_allow :: proc "contextless" (
	user_context: rawptr, worker_index: int,
	a, b: entasis.Collidable_Reference, speculative_margin: ^f32,
) -> entasis.Collision_Testing_State
{
	_, _, _, _ = worker_index, a, b, speculative_margin;
	state := (^Mixed_Callback_Policy_State)(user_context);
	if state == nil
	{
		return .Reject;
	}
	mixed_callback_record(state, .Allow);
	return .Allow;
}

mixed_callback_configure :: proc "contextless" (
	user_context: rawptr, worker_index: int,
	a, b: entasis.Collidable_Reference, manifold: ^entasis.Manifold_Result,
	material: ^entasis.Contact_Material,
) -> entasis.Collision_Testing_State
{
	_, _, _, _ = worker_index, a, b, manifold;
	state := (^Mixed_Callback_Policy_State)(user_context);
	if state == nil || material == nil
	{
		return .Reject;
	}
	mixed_callback_record(state, .Configure);
	material^ = state.policy.material;
	return .Allow;
}

callback_policy_add_contact_pair :: proc(
	t: ^testing.T, world: ^entasis.World,
) -> (entasis.Shape_Handle, entasis.Status)
{
	shape_value := entasis.sphere(1);
	shape, shape_status := entasis.shape_add(world, shape_value);
	if !testing.expect_value(t, shape_status, entasis.Status.Ok)
	{
		return {}, shape_status;
	}
	inertia, inertia_status := entasis.shape_inertia(shape_value, 1);
	if !testing.expect_value(t, inertia_status, entasis.Status.Ok)
	{
		return {}, inertia_status;
	}
	_, body_status := entasis.body_add(
		world,
		entasis.body_dynamic(
		shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 255),
	),
	);
	if !testing.expect_value(t, body_status, entasis.Status.Ok)
	{
		return {}, body_status;
	}
	_, static_status := entasis.static_add(
		world, entasis.static_body(shape, entasis.pose({1.5, 0, 0})), .None,
	);
	if !testing.expect_value(t, static_status, entasis.Status.Ok)
	{
		return {}, static_status;
	}
	return shape, .Ok;
}

callback_policy_expect_contact_material :: proc(
	t: ^testing.T, world: ^entasis.World, expected: entasis.Contact_Material,
)
{
	infos: [1]entasis.Constraint_Info;
	written, total, enumerate_status := entasis.constraint_enumerate(world, infos[:]);
	if !testing.expect_value(t, enumerate_status, entasis.Status.Ok) ||
		!testing.expect_value(t, written, 1) ||
		!testing.expect_value(t, total, 1)
	{
		return;
	}
	contact, contact_status := entasis.solver_contact_data(world, infos[0].handle);
	if !testing.expect_value(t, contact_status, entasis.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, contact.material, expected);
}

@(test)
default_narrow_policy_matches_qualified_default :: proc(t: ^testing.T)
{
	policy := entasis.default_narrow_policy();
	constructed := entasis.contact_material(0.75, 3, entasis.spring_settings(20, 0.8));
	testing.expect_value(t, constructed.friction_coefficient, f32(0.75));
	testing.expect_value(t, constructed.maximum_recovery_velocity, f32(3));
	callbacks := entasis.narrow_policy_default(&policy);
	qualified := physics.narrow_phase_default_callbacks();
	fresh := entasis.narrow_policy_default(&policy);

	testing.expect(t, callbacks.initialize == fresh.initialize);
	testing.expect(t, callbacks.allow == qualified.allow);
	testing.expect(t, callbacks.allow_child == qualified.allow_child);
	testing.expect(t, callbacks.configure == fresh.configure);
	testing.expect(t, callbacks.configure_child == qualified.configure_child);
	testing.expect(t, callbacks.dispose == qualified.dispose);
	testing.expect(t, callbacks.select_constraint == qualified.select_constraint);
	testing.expect_value(t, callbacks.user_context, rawptr(&policy));
	testing.expect_value(t, callbacks.initialize(callbacks.user_context, nil), entasis.Status.Ok);

	configured: entasis.Contact_Material;
	manifold: entasis.Manifold_Result;
	result := callbacks.configure(
		callbacks.user_context, 0, {}, {}, &manifold, &configured,
	);
	testing.expect_value(t, result, entasis.Collision_Testing_State.Allow);
	testing.expect_value(t, configured, policy.material);
	testing.expect_value(t, configured, entasis.contact_material_default());
}

@(test)
uniform_pose_policy_reuses_qualified_wide_callbacks :: proc(t: ^testing.T)
{
	policy := entasis.uniform_gravity_policy({1, -12, 3}, 0.02, 0.03);
	callbacks := entasis.pose_policy_uniform(&policy);
	qualified := physics.pose_integrator_default_callbacks(&policy);

	testing.expect(t, callbacks.initialize == qualified.initialize);
	testing.expect(t, callbacks.prepare_for_integration == qualified.prepare_for_integration);
	testing.expect(t, callbacks.integrate_velocity == qualified.integrate_velocity);
	testing.expect(t, callbacks.dispose == qualified.dispose);
	testing.expect_value(t, callbacks.angular_mode, qualified.angular_mode);
	testing.expect_value(t, callbacks.user_context, rawptr(&policy));
	testing.expect_value(t, callbacks.initialize(callbacks.user_context, nil), entasis.Status.Ok);
	testing.expect_value(
		t, callbacks.prepare_for_integration(callbacks.user_context, 0.25), entasis.Status.Ok,
	);

	gravity_dt := util.vector3_wide_read_slot(policy.gravity_wide_dt, 0);
	testing.expect_value(t, gravity_dt, entasis.Vector3{0.25, -3, 0.75});
	callback_expect_f32(
		t, simd.extract(policy.linear_damping_dt, 0),
		f32(math.pow(f64(0.98), f64(0.25))), 1e-6, "linear damping",
	);
	callback_expect_f32(
		t, simd.extract(policy.angular_damping_dt, 0),
		f32(math.pow(f64(0.97), f64(0.25))), 1e-6, "angular damping",
	);
}

@(test)
world_uses_caller_owned_policies_with_included_workers :: proc(t: ^testing.T)
{
	worker_counts := [2]i32{1, 2};
	for worker_count in worker_counts
	{
		narrow_policy := entasis.default_narrow_policy();
		narrow_policy.material = entasis.contact_material(
			0.6, 4, entasis.spring_settings(18, 0.75),
		);
		pose_policy := entasis.uniform_gravity_policy({0, -15, 0}, 0.04, 0.05);
		description := small_world_description();
		description.gravity = {};
		description.threading.worker_count = worker_count;
		status := entasis.world_description_set_callbacks(
			&description,
			entasis.narrow_policy_default(&narrow_policy),
			entasis.pose_policy_uniform(&pose_policy),
		);
		if !testing.expect_value(t, status, entasis.Status.Ok)
		{
			continue;
		}

		world: entasis.World;
		if !testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok)
		{
			continue;
		}
		simulation, borrow_status := entasis.world_borrow_simulation(&world);
		testing.expect_value(t, borrow_status, entasis.Status.Ok);
		testing.expect_value(
			t, simulation.narrow_phase.stored_completion_state,
			physics.Reference_State.Present,
		);
		testing.expect_value(
			t, simulation.narrow_phase.stored_completion_material,
			narrow_policy.material,
		);
		shape, pair_status := callback_policy_add_contact_pair(t, &world);
		if pair_status != .Ok
		{
			_ = entasis.world_destroy(&world);
			continue;
		}
		overlap_hits: [4]entasis.Overlap_Hit;
		overlap_count, overlap_status := entasis.overlap_all(
			&world, shape, entasis.pose({0.75, 0, 0}), overlap_hits[:],
		);
		testing.expect_value(t, overlap_status, entasis.Status.Ok);
		testing.expect(t, overlap_count > 0);
		stored_completion_restored := physics.Reference_State.Missing;
		if simulation.narrow_phase.batchers[0].stored_pair_completed ==
			physics.narrow_phase_pair_completed_stored
		{
			stored_completion_restored = .Present;
		}
		testing.expect_value(
			t, stored_completion_restored, physics.Reference_State.Present,
		);
		testing.expect_value(t, entasis.world_step(&world, 0.2), entasis.Status.Ok);
		testing.expect_value(
			t, util.vector3_wide_read_slot(pose_policy.gravity_wide_dt, 0),
			entasis.Vector3{0, -3, 0},
		);
		callback_policy_expect_contact_material(t, &world, narrow_policy.material);
		testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
	}

	mixed_state := Mixed_Callback_Policy_State{
		policy={
			material=entasis.contact_material(
				0.35, 6, entasis.spring_settings(12, 0.5),
			),
		},
	};
	mixed_callbacks := entasis.narrow_policy_default(&mixed_state.policy);
	mixed_callbacks.initialize = mixed_callback_initialize;
	mixed_callbacks.allow = mixed_callback_allow;
	mixed_callbacks.configure = mixed_callback_configure;
	mixed_callbacks.user_context = &mixed_state;
	mixed_pose_policy := entasis.uniform_gravity_policy({}, 0, 0);
	mixed_description := small_world_description();
	mixed_description.gravity = {};
	if !testing.expect_value(
		t,
		entasis.world_description_set_callbacks(
		&mixed_description, mixed_callbacks,
		entasis.pose_policy_uniform(&mixed_pose_policy),
	),
		entasis.Status.Ok,
	)
	{
		return;
	}
	mixed_world: entasis.World;
	if !testing.expect_value(
		t, entasis.world_init(&mixed_world, mixed_description), entasis.Status.Ok,
	)
	{
		return;
	}
	defer entasis.world_destroy(&mixed_world);
	mixed_simulation, mixed_borrow_status := entasis.world_borrow_simulation(&mixed_world);
	testing.expect_value(t, mixed_borrow_status, entasis.Status.Ok);
	testing.expect_value(
		t, mixed_simulation.narrow_phase.stored_completion_state,
		physics.Reference_State.Missing,
	);
	_, mixed_pair_status := callback_policy_add_contact_pair(t, &mixed_world);
	if mixed_pair_status != .Ok
	{
		return;
	}
	if !testing.expect_value(
		t, entasis.world_step(&mixed_world, 1.0 / 60.0), entasis.Status.Ok,
	)
	{
		return;
	}
	testing.expect_value(t, mixed_state.count, 3);
	testing.expect_value(t, mixed_state.calls[0], Callback_Policy_Call.Initialize);
	testing.expect_value(t, mixed_state.calls[1], Callback_Policy_Call.Allow);
	testing.expect_value(t, mixed_state.calls[2], Callback_Policy_Call.Configure);
	callback_policy_expect_contact_material(t, &mixed_world, mixed_state.policy.material);
}

@(test)
incomplete_callback_tables_are_rejected_without_mutation :: proc(t: ^testing.T)
{
	description := entasis.world_description_default();
	narrow_policy := entasis.default_narrow_policy();
	pose_policy := entasis.uniform_gravity_policy();
	valid_narrow := entasis.narrow_policy_default(&narrow_policy);
	valid_pose := entasis.pose_policy_uniform(&pose_policy);
	invalid_narrow: entasis.Narrow_Callbacks;

	testing.expect_value(
		t,
		entasis.world_description_set_callbacks(&description, invalid_narrow, valid_pose),
		entasis.Status.Invalid_Description,
	);
	testing.expect_value(t, description.narrow_callbacks, entasis.Narrow_Callbacks{});
	testing.expect_value(t, description.pose_callbacks, entasis.Pose_Callbacks{});
	testing.expect_value(
		t,
		entasis.world_description_set_callbacks(&description, valid_narrow, valid_pose),
		entasis.Status.Ok,
	);
}

@(test)
invalid_custom_material_is_rejected_without_poisoning_world :: proc(t: ^testing.T)
{
	narrow_policy := entasis.default_narrow_policy();
	narrow_policy.material.friction_coefficient = -1;
	pose_policy := entasis.uniform_gravity_policy();
	description := small_world_description();
	testing.expect_value(
		t,
		entasis.world_description_set_callbacks(
		&description,
		entasis.narrow_policy_default(&narrow_policy),
		entasis.pose_policy_uniform(&pose_policy),
	),
		entasis.Status.Ok,
	);

	world: entasis.World;
	testing.expect_value(
		t, entasis.world_init(&world, description), entasis.Status.Invalid_Description,
	);
	narrow_policy.material = entasis.contact_material_default();
	testing.expect_value(t, entasis.world_init(&world, description), entasis.Status.Ok);
	testing.expect_value(t, entasis.world_destroy(&world), entasis.Status.Ok);
}
