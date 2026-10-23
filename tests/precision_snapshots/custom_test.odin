package precision_snapshots_tests

import "core:testing"
import e "entasis:entasis"

Snapshot_Custom_Value :: struct
{
	marker: f32,
}

Snapshot_Custom_Wide :: struct
{
	marker: e.F32x8,
}

snapshot_custom_validate :: proc "contextless" (type_id: i32, description: rawptr) -> e.Status
{
	_ = type_id;
	value: ^Snapshot_Custom_Value = (^Snapshot_Custom_Value)(description);
	return .Ok if value.marker == 7 else .Invalid_Description;
}

snapshot_custom_kernel :: proc "contextless" (
	prestep: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide, dt, inverse_dt: f32,
	impulses: rawptr, mask: e.I32x8, phase: e.Constraint_Kernel_Phase,
)
{
	_, _, _, _, _, _, _ = prestep, bodies, dt, inverse_dt, impulses, mask, phase;
}

@(test)
global_state_custom_constraint_payload_is_rejected_without_reinterpreting_coordinates :: proc(t: ^testing.T)
{
	for space in ([2]e.Constraint_Position_Space{.Absolute_Float, .Body_A_Relative})
	{
		scene: Scene;
		scene_initialize(t, &scene);
		access: [4]e.Body_Access_Mask = {e.BODY_ACCESS_ALL, {}, {}, {}};
		registration: e.Custom_Constraint_Registration = e.custom_constraint_registration(
			Snapshot_Custom_Value, Snapshot_Custom_Wide, Snapshot_Custom_Wide,
			e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, 1, access, access,
			snapshot_custom_validate, snapshot_custom_kernel, position_space=space,
		);
		testing.expect_value(t, e.custom_constraint_register(&scene.world, registration), e.Status.Ok);
		body: e.Body_Handle = add_body(t, &scene, {1e9 + 0.001953125, 0, 0}, {});
		bodies: [1]e.Body_Handle = {body};
		description: Snapshot_Custom_Value = {marker=7};
		joint, status := e.custom_constraint_add_typed(&scene.world, bodies[:], registration.type_id, &description);
		testing.expect_value(t, status, e.Status.Ok);
		state, read_status := e.constraint_get_global_target_state(&scene.world, joint);
		testing.expect_value(t, read_status, e.Status.Invalid_Argument);
		testing.expect_value(t, state, e.Global_Constraint_Target_State{});
		state = {position={1e9 + 0.125, 0, 0}, kind=.Linear_Servo};
		testing.expect_value(t, e.constraint_apply_global_target_state(&scene.world, joint, state), e.Status.Invalid_Argument);
		joints: [1]e.Constraint_Handle = {joint};
		states: [1]e.Global_Constraint_Target_State = {state};
		counts, export_status := e.world_export_global_states(&scene.world, {constraint_targets=joints[:]}, {constraint_targets=states[:]});
		testing.expect_value(t, export_status, e.Status.Invalid_Argument);
		testing.expect_value(t, counts, e.Global_State_Counts{});
		testing.expect_value(t, states[0], state);
		counts, status = e.world_apply_global_states(&scene.world, {constraint_targets=joints[:]}, {constraint_targets=states[:]});
		testing.expect_value(t, status, e.Status.Invalid_Argument);
		testing.expect_value(t, counts, e.Global_State_Counts{});
		actual: Snapshot_Custom_Value;
		testing.expect_value(t, e.custom_constraint_get_typed(&scene.world, joint, registration.type_id, &actual), e.Status.Ok);
		testing.expect_value(t, actual, description);
		testing.expect_value(t, scene.simulation.solver.registry.records[registration.type_id].position_space, space);
		expect_body_position(t, &scene, body, {1e9 + 0.001953125, 0, 0});
		scene_dispose(t, &scene);
	}
}
