// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

// contextual bindings are immutable registration records. low-level callers own
// their storage until registry reset and completion of all work using the route.
// the facade copies them into world-owned storage. no shape payload carries a
// binding, and a query's filter context is independent of registration context
Collision_Task_Dispatch :: enum u8
{
	Native,
	Contextual,
}

Contextual_Collision_Test_Proc :: #type proc "contextless" (
	user_context: rawptr,
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose,
	speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status);
Contextual_Collision_Wide_Test_Proc :: #type proc "contextless" (
	user_context: rawptr, bundle: ^Collision_Convex_Wide_Bundle,
	shapes: ^Shape_Registry, result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status;
Contextual_Collision_Task_Binding :: struct
{
	user_context: rawptr,
	test:         Contextual_Collision_Test_Proc,
	wide_test:    Contextual_Collision_Wide_Test_Proc,
}

Contextual_Sweep_Test_Proc :: #type proc "contextless" (
	user_context: rawptr,
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, filter_context: rawptr,
) -> (Sweep_Result, Physics_Status);
Contextual_Sweep_Child_Test_Proc :: #type proc "contextless" (
	user_context: rawptr,
	shape_a, shape_b: rawptr, type_a, type_b: int,
	parent_pose_a, parent_pose_b, local_pose_a, local_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
) -> (Sweep_Result, Physics_Status);
Contextual_Sweep_Task_Binding :: struct
{
	user_context: rawptr,
	test:         Contextual_Sweep_Test_Proc,
	child_test:   Contextual_Sweep_Child_Test_Proc,
}

collision_task_wide_test_state :: #force_inline proc "contextless" (
	task: ^Collision_Task,
) -> Reference_State
{
	if task.dispatch == .Contextual
	{
		return .Present if task.contextual != nil && task.contextual.wide_test != nil else .Missing;
	}
	// preserve the legacy batch-admission contract for native registrations
	return .Present if task.convex_wide_test != nil else .Missing;
}

collision_task_test_contextual :: proc "contextless" (
	binding: ^Contextual_Collision_Task_Binding,
	order: Collision_Task_Route_Order,
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose,
	speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	if binding == nil || binding.test == nil
	{
		return {}, .Invalid_Description;
	}
	if order == .Expected
	{
		return binding.test(binding.user_context, shape_a, shape_b,
			pose_a, pose_b, speculative_margin, shapes);
	}
	manifold: Convex_Contact_Manifold;
	status: Physics_Status;
	manifold, status = binding.test(binding.user_context, shape_b, shape_a,
		pose_b, pose_a, speculative_margin, shapes);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_manifold_flip(manifold), .Ok;
}

@(private)
contextual_sweep_filter_context :: struct
{
	filter: Collision_Child_Filter_Proc,
	user_context: rawptr,
}

@(private)
contextual_sweep_filter_flipped :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	filter_context: ^contextual_sweep_filter_context = (^contextual_sweep_filter_context)(user_context);
	return filter_context.filter(filter_context.user_context, pair_id, child_b, child_a);
}

sweep_task_test_contextual :: proc "contextless" (
	binding: ^Contextual_Sweep_Task_Binding, order: Sweep_Task_Route_Order,
	shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose, velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc, filter_context: rawptr,
) -> (Sweep_Result, Physics_Status)
{
	if binding == nil || binding.test == nil
	{
		return {}, .Invalid_Description;
	}
	if order == .Expected
	{
		return binding.test(binding.user_context, shape_a, shape_b, type_a, type_b,
			pose_a, pose_b, velocity_a, velocity_b, maximum_t, minimum_progression,
			convergence_threshold, maximum_iteration_count, shapes, collision_tasks,
			filter, filter_context);
	}
	flipped_context: contextual_sweep_filter_context = contextual_sweep_filter_context{filter, filter_context};
	task_filter: Collision_Child_Filter_Proc = filter;
	task_context: rawptr = filter_context;
	if filter != nil
	{
		task_filter = contextual_sweep_filter_flipped;
		task_context = &flipped_context;
	}
	result: Sweep_Result;
	status: Physics_Status;
	result, status = binding.test(binding.user_context, shape_b, shape_a, type_b, type_a,
		pose_b, pose_a, velocity_b, velocity_a, maximum_t, minimum_progression,
		convergence_threshold, maximum_iteration_count, shapes, collision_tasks,
		task_filter, task_context);
	if status != .Ok
	{
		return {}, status;
	}
	result.normal = util.vector3_negate(result.normal);
	result.child_a, result.child_b = result.child_b, result.child_a;
	return result, .Ok;
}

sweep_task_test_contextual_child :: proc "contextless" (
	binding: ^Contextual_Sweep_Task_Binding, order: Sweep_Task_Route_Order,
	child_a, child_b: Collision_Child,
	parent_pose_a, parent_pose_b, local_pose_a, local_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int, shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
) -> (Sweep_Result, Physics_Status)
{
	if binding == nil || binding.child_test == nil
	{
		return {}, .Invalid_Description;
	}
	if order == .Expected
	{
		return binding.child_test(binding.user_context,
			child_a.shape, child_b.shape, child_a.type_id, child_b.type_id,
			parent_pose_a, parent_pose_b, local_pose_a, local_pose_b,
			velocity_a, velocity_b, maximum_t, minimum_progression,
			convergence_threshold, maximum_iteration_count, shapes, collision_tasks);
	}
	result: Sweep_Result;
	status: Physics_Status;
	result, status = binding.child_test(binding.user_context,
		child_b.shape, child_a.shape, child_b.type_id, child_a.type_id,
		parent_pose_b, parent_pose_a, local_pose_b, local_pose_a,
		velocity_b, velocity_a, maximum_t, minimum_progression,
		convergence_threshold, maximum_iteration_count, shapes, collision_tasks);
	if status != .Ok
	{
		return {}, status;
	}
	result.normal = util.vector3_negate(result.normal);
	result.child_a, result.child_b = result.child_b, result.child_a;
	return result, .Ok;
}
