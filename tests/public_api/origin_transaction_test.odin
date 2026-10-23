package public_api_tests

import "core:testing"
import "base:runtime"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Origin_Bounds_Execution :: enum u8
{
	Setup,
	Shift,
}

Origin_Bounds_State :: struct
{
	// existing contextual callbacks receive this leading sphere state
	sphere: Contextual_Shape_State,
	world: ^e.World,
	frame: ^e.Origin_Frame,
	query: ^e.Query_Context,
	execution: Origin_Bounds_Execution,
	calls: int,
	fail_at: int,
	nested_shift: e.Status,
	nested_stage: e.Status,
	nested_query: e.Status,
}

origin_shift_bounds_callback :: proc "contextless" (
	user_context, shape: rawptr, orientation: ^u.Quaternion,
	access: e.Shape_Access, result: ^p.Shape_Bounds,
) -> e.Status
{
	state: ^Origin_Bounds_State = (^Origin_Bounds_State)(user_context);
	if state.execution == .Shift
	{
		context = runtime.default_context();
		state.calls += 1;
		state.nested_shift = e.world_shift_origin(state.world, state.frame, {1, 0, 0});
		state.nested_stage = e.world_stage_predict_bounds(state.world, 1.0 / 60.0);
		_, state.nested_query = e.ray_cast_closest_with_context(state.query, e.ray({}, {1, 0, 0}, 100));
		if state.calls == state.fail_at
		{
			return .Capacity_Missing;
		}
	}
	return contextual_shape_bounds_test(&state.sphere, shape, orientation, access, result);
}

@(test)
origin_shift_custom_bounds_are_preflighted_once :: proc(t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	frame: e.Origin_Frame = {origin={x=1e9}};
	query: e.Query_Context;
	if !testing.expect_value(t, e.query_context_init(&query, &world), e.Status.Ok)
	{
		return;
	}
	defer e.query_context_destroy(&query);
	state: Origin_Bounds_State = {
		sphere={scale=1}, world=&world, frame=&frame, query=&query,
	};
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state.sphere);
	registration.user_context = &state;
	registration.bounds = origin_shift_bounds_callback;
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register_contextual(&world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	shape: e.Shape_Handle;
	shape, status = e.custom_shape_add(&world, type_id, &Custom_Sphere{1});
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	handles: [2]e.Body_Handle;
	for &handle, index in handles
	{
		handle, status = e.body_add(&world, e.body_kinematic(shape, e.pose({f32(index) * 4, 3, 0})));
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
	}
	poses: [2]e.Body_State;
	bounds: [2]e.Bounding_Box;
	for handle, index in handles
	{
		poses[index], status = e.body_get(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		bounds[index], status = e.body_bounds(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
	}
	view: e.Active_Body_View;
	view, status = e.active_body_view(&world);
	testing.expect_value(t, status, e.Status.Ok);
	original_frame: e.Origin_Frame = frame;
	state.execution = .Shift;
	state.fail_at = 2;
	testing.expect_value(t, e.world_shift_origin(&world, &frame, {256, 0, 0}), e.Status.Capacity_Missing);
	testing.expect_value(t, state.calls, 2);
	testing.expect_value(t, state.nested_shift, e.Status.Invalid_Argument);
	testing.expect_value(t, state.nested_stage, e.Status.Invalid_Argument);
	testing.expect_value(t, state.nested_query, e.Status.Invalid_Argument);
	testing.expect_value(t, frame, original_frame);
	testing.expect(t, e.body_view_valid(&world, view));
	for handle, index in handles
	{
		actual_pose: e.Body_State;
		actual_pose, status = e.body_get(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, actual_pose, poses[index]);
		actual_bounds: e.Bounding_Box;
		actual_bounds, status = e.body_bounds(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, actual_bounds, bounds[index]);
	}
	state.calls = 0;
	testing.expect_value(t, e.world_shift_origin(&world, &frame, {}), e.Status.Ok);
	testing.expect_value(t, state.calls, 0);
	testing.expect_value(t, frame, original_frame);
	testing.expect(t, e.body_view_valid(&world, view));
	state.fail_at = 0;
	testing.expect_value(t, e.world_shift_origin(&world, &frame, {256, 0, 0}), e.Status.Ok);
	testing.expect_value(t, state.calls, 2);
	testing.expect_value(t, frame.origin.x, original_frame.origin.x + 256);
	testing.expect(t, !e.body_view_valid(&world, view));
	for handle, index in handles
	{
		actual_pose: e.Body_State;
		actual_pose, status = e.body_get(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, actual_pose.pose.position.x, poses[index].pose.position.x - 256);
	}
}
