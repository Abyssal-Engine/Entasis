package entasis_physics

import util "entasis:entasis_utilities"
import "core:math"

Narrow_Phase_Sweep_Filter_Context :: struct
{
	narrow:       ^Narrow_Phase,
	pair:         Broad_Phase_Pair,
	worker_index: i32,
}
narrow_phase_sweep_allow_child :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	ctx := (^Narrow_Phase_Sweep_Filter_Context)(user_context);
	if ctx == nil
	{
		return .Reject;
	}
	_ = pair_id;
	return ctx.narrow.callbacks.allow_child(
		ctx.narrow.callbacks.user_context, int(ctx.worker_index), ctx.pair.a, ctx.pair.b,
		int(child_a), int(child_b),
	);
}
narrow_phase_prepare_continuation :: #force_no_inline proc "contextless" (
	narrow: ^Narrow_Phase, pair_index, worker_index: int, pair: Broad_Phase_Pair,
	shape_a, shape_b: Typed_Index, pose_a, pose_b: ^Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	continuity_a, continuity_b: Continuous_Detection,
	speculative_margin, dt: f32,
) -> Physics_Status
{
	continuation := Narrow_Phase_CCD_Continuation{
		pair=collidable_pair_create(pair.a, pair.b), worker_index=i32(worker_index), kind=.Discrete,
	};
	narrow.continuations.memory[pair_index] = continuation;
	if continuity_a.mode != .Continuous && continuity_b.mode != .Continuous
	{
		return .Ok;
	}
	_, _, sweep_route_status := sweep_task_registry_lookup(
		narrow.sweep_tasks, int(typed_index_type(shape_a)), int(typed_index_type(shape_b)),
	);
	if sweep_route_status == .Not_Found
	{
		return .Ok;
	}
	if sweep_route_status != .Ok
	{
		return sweep_route_status;
	}
	bounds_a, bounds_a_status := shape_registry_compute_bounds(narrow.shapes, shape_a, pose_a.orientation);
	if bounds_a_status != .Ok
	{
		return bounds_a_status;
	}
	bounds_b, bounds_b_status := shape_registry_compute_bounds(narrow.shapes, shape_b, pose_b.orientation);
	if bounds_b_status != .Ok
	{
		return bounds_b_status;
	}
	relative_linear := util.vector3_subtract(velocity_b.linear, velocity_a.linear);
	maximum_displacement := (
		util.vector3_length(relative_linear) +
		util.vector3_length(velocity_a.angular) * bounds_a.maximum_radius +
		util.vector3_length(velocity_b.angular) * bounds_b.maximum_radius
	) * dt;
	if maximum_displacement <= speculative_margin
	{
		return .Ok;
	}
	minimum_progression := f32(math.F32_MAX);
	convergence_threshold := f32(math.F32_MAX);
	if continuity_a.mode == .Continuous
	{
		minimum_progression = min(minimum_progression, continuity_a.minimum_sweep_timestep);
		convergence_threshold = min(convergence_threshold, continuity_a.sweep_convergence_threshold);
	}
	if continuity_b.mode == .Continuous
	{
		minimum_progression = min(minimum_progression, continuity_b.minimum_sweep_timestep);
		convergence_threshold = min(convergence_threshold, continuity_b.sweep_convergence_threshold);
	}
	filter_context := Narrow_Phase_Sweep_Filter_Context{narrow, pair, i32(worker_index)};
	traversal_pool := narrow.batchers[worker_index].traversal_pool;
	sweep, sweep_status := sweep_task_registry_test(
		narrow.sweep_tasks, shape_a, shape_b, pose_a^, pose_b^, velocity_a, velocity_b,
		dt, minimum_progression, convergence_threshold, 25, narrow.shapes, narrow.tasks,
		narrow_phase_sweep_allow_child, &filter_context, traversal_pool,
	);
	if sweep_status != .Ok
	{
		return sweep_status;
	}
	if sweep.state != .Hit
	{
		return .Ok;
	}
	pose_a^ = sweep_pose_at(pose_a^, velocity_a, sweep.t1);
	pose_b^ = sweep_pose_at(pose_b^, velocity_b, sweep.t1);
	continuation.relative_linear_velocity = relative_linear;
	continuation.angular_a = velocity_a.angular;
	continuation.angular_b = velocity_b.angular;
	continuation.t = sweep.t1;
	continuation.kind = .Continuous;
	narrow.continuations.memory[pair_index] = continuation;
	return .Ok;
}
narrow_phase_rewind_convex_depths :: proc "contextless" (
	continuation: ^Narrow_Phase_CCD_Continuation,
	manifold: ^Convex_Contact_Manifold,
	$record_route: Narrow_Phase_Collision_Route,
)
{
	if continuation == nil || manifold == nil || continuation.kind != .Continuous
	{
		return;
	}
	for contact_index in 0 ..< int(manifold.count)
	{
		contact := &manifold.contacts[contact_index];
		angular_a := util.vector3_cross(continuation.angular_a, contact.offset);
		angular_b := util.vector3_cross(
			continuation.angular_b, util.vector3_subtract(contact.offset, manifold.offset_b),
		);
		contact_velocity := util.vector3_add(
			util.vector3_subtract(angular_b, angular_a), continuation.relative_linear_velocity,
		);
		contact.depth -= util.vector3_dot(contact_velocity, manifold.normal) * continuation.t;
	}
}
narrow_phase_rewind_nonconvex_depths :: proc "contextless" (
	continuation: ^Narrow_Phase_CCD_Continuation,
	manifold: ^Nonconvex_Contact_Manifold,
)
{
	if continuation == nil || manifold == nil || continuation.kind != .Continuous
	{
		return;
	}
	for contact_index in 0 ..< int(manifold.count)
	{
		contact := &manifold.contacts[contact_index];
		angular_a := util.vector3_cross(continuation.angular_a, contact.offset);
		angular_b := util.vector3_cross(
			continuation.angular_b, util.vector3_subtract(contact.offset, manifold.offset_b),
		);
		contact_velocity := util.vector3_add(
			util.vector3_subtract(angular_b, angular_a), continuation.relative_linear_velocity,
		);
		contact.depth -= util.vector3_dot(contact_velocity, contact.normal) * continuation.t;
	}
}
narrow_phase_rewind_depths :: proc "contextless" (
	continuation: ^Narrow_Phase_CCD_Continuation, manifold: ^Manifold_Result,
)
{
	if continuation == nil || manifold == nil
	{
		return;
	}
	if manifold.kind == .Convex
	{
		narrow_phase_rewind_convex_depths(continuation, &manifold.convex, .Predecessor);
		return;
	}
	narrow_phase_rewind_nonconvex_depths(continuation, &manifold.nonconvex);
}
narrow_phase_rewind_stored_depths :: proc "contextless" (
	continuation: ^Narrow_Phase_CCD_Continuation,
	manifold: ^Collision_Stored_Manifold,
)
{
	if continuation == nil || manifold == nil
	{
		return;
	}
	if manifold.kind == .Convex
	{
		narrow_phase_rewind_convex_depths(continuation, &manifold.convex, .Predecessor);
		return;
	}
	narrow_phase_rewind_nonconvex_depths(continuation, &manifold.nonconvex);
}
