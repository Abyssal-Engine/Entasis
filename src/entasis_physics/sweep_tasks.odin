// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

MAXIMUM_SWEEP_TASK_COUNT :: 128;
SWEEP_TASK_MATRIX_ENTRY_COUNT :: MAXIMUM_SHAPE_TYPE_COUNT * MAXIMUM_SHAPE_TYPE_COUNT;
BUILT_IN_SWEEP_TASK_COUNT :: 44;
Sweep_Hit_State :: enum u8
{
	Miss,
	Hit,
}

Sweep_Task_Registry_State :: enum u8
{
	Uninitialized,
	Ready,
}

Sweep_Task_Route_Order :: enum u8
{
	Expected,
	Flipped,
}

Sweep_Task_Kind :: enum u8
{
	Custom,
	Built_In,
	Contextual,
}

Sweep_Task_Call_State :: enum u8
{
	Uninitialized,
	Ready,
}

Sweep_Result :: struct
{
	state:       Sweep_Hit_State,
	t0:          f32,
	t1:          f32,
	location:    util.Vector3,
	normal:      util.Vector3,
	child_a:     i32,
	child_b:     i32,
}

Sweep_Test_Proc :: #type proc "contextless" (
	shape_a, shape_b: rawptr,
	type_a, type_b: int,
	pose_a, pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc,
	user_context: rawptr,
) -> (Sweep_Result, Physics_Status);
Sweep_Child_Test_Proc :: #type proc "contextless" (
	shape_a, shape_b: rawptr,
	type_a, type_b: int,
	parent_pose_a, parent_pose_b: Rigid_Pose,
	local_pose_a, local_pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
) -> (Sweep_Result, Physics_Status);
Sweep_Task :: struct
{
	task_id:      i32,
	shape_type_a: i16,
	shape_type_b: i16,
	using callbacks: struct #raw_union
	{
		using native: struct
		{
			test:       Sweep_Test_Proc,
			child_test: Sweep_Child_Test_Proc,
		},
		contextual: ^Contextual_Sweep_Task_Binding,
	},
	kind: Sweep_Task_Kind,
}

Sweep_Task_Reference :: struct
{
	task_id: i32,
	order:   Sweep_Task_Route_Order,
}

Sweep_Task_Registry :: struct
{
	tasks:      [MAXIMUM_SWEEP_TASK_COUNT]Sweep_Task,
	routes:     [SWEEP_TASK_MATRIX_ENTRY_COUNT]Sweep_Task_Reference,
	task_count: int,
	state:      Sweep_Task_Registry_State,
}

sweep_task_matrix_index :: proc "contextless" (type_a, type_b: int) -> int
{
	return type_a * MAXIMUM_SHAPE_TYPE_COUNT + type_b;
}

sweep_task_registry_reset :: proc "contextless" (registry: ^Sweep_Task_Registry) -> Physics_Status
{
	if registry == nil
	{
		return .Invalid_Argument;
	}
	registry^ = {};
	for index in 0 ..< SWEEP_TASK_MATRIX_ENTRY_COUNT
	{
		registry.routes[index].task_id = -1;
	}
	return .Ok;
}

sweep_task_registry_validate_registration :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int,
	test: Sweep_Test_Proc, child_test: Sweep_Child_Test_Proc,
	kind: Sweep_Task_Kind,
	contextual: ^Contextual_Sweep_Task_Binding = nil,
) -> Physics_Status
{
	if registry == nil || type_a < 0 || type_b < 0 || type_a >= MAXIMUM_SHAPE_TYPE_COUNT ||
	type_b >= MAXIMUM_SHAPE_TYPE_COUNT || kind > .Contextual ||
	(kind != .Contextual && test == nil) ||
	(kind == .Custom && child_test == nil) ||
	(kind == .Built_In && child_test != nil) ||
	(kind == .Contextual && (contextual == nil || contextual.test == nil || contextual.child_test == nil)) ||
	registry.task_count >= MAXIMUM_SWEEP_TASK_COUNT
	{
		return .Invalid_Argument;
	}
	if registry.routes[sweep_task_matrix_index(type_a, type_b)].task_id >= 0 ||
	registry.routes[sweep_task_matrix_index(type_b, type_a)].task_id >= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

sweep_task_registry_register_internal :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int,
	test: Sweep_Test_Proc, child_test: Sweep_Child_Test_Proc,
	kind: Sweep_Task_Kind, contextual: ^Contextual_Sweep_Task_Binding = nil,
) -> (i32, Physics_Status)
{
	status: Physics_Status = sweep_task_registry_validate_registration(registry, type_a, type_b, test, child_test, kind, contextual);
	if status != .Ok
	{
		return -1, status;
	}
	task_id := i32(registry.task_count);
	registry.tasks[registry.task_count] = {
		task_id=task_id, shape_type_a=i16(type_a), shape_type_b=i16(type_b),
		test=test, child_test=child_test, kind=kind,
	};
	if kind == .Contextual
	{
		registry.tasks[registry.task_count].contextual = contextual;
	}
	reference := Sweep_Task_Reference{task_id=task_id};
	registry.routes[sweep_task_matrix_index(type_a, type_b)] = reference;
	if type_a != type_b
	{
		reference.order = .Flipped;
	}
	registry.routes[sweep_task_matrix_index(type_b, type_a)] = reference;
	registry.task_count += 1;
	return task_id, .Ok;
}

sweep_task_registry_register :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int,
	test: Sweep_Test_Proc, child_test: Sweep_Child_Test_Proc,
) -> (i32, Physics_Status)
{
	return sweep_task_registry_register_internal(
		registry, type_a, type_b, test, child_test, .Custom,
	);
}

sweep_task_registry_register_built_in :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int, test: Sweep_Test_Proc,
) -> (i32, Physics_Status)
{
	return sweep_task_registry_register_internal(
		registry, type_a, type_b, test, nil, .Built_In,
	);
}

sweep_pose_at :: proc "contextless" (pose: Rigid_Pose, velocity: Body_Velocity, t: f32) -> Rigid_Pose
{
	result := pose;
	result.position = util.vector3_add(pose.position, util.vector3_scale(velocity.linear, t));
	angular_speed := util.vector3_length(velocity.angular);
	if angular_speed > 1e-12
	{
		axis := util.vector3_scale(velocity.angular, 1 / angular_speed);
		delta := util.quaternion_from_axis_angle(axis, angular_speed * t);
		result.orientation = util.quaternion_normalize(util.quaternion_concatenate(pose.orientation, delta));
	}
	return result;
}

manifold_deepest_contact :: proc "contextless" (
	manifold: ^Manifold_Result,
) -> (Contact, Reference_State)
{
	if manifold == nil
	{
		return {}, .Missing;
	}
	if manifold.kind == .Convex
	{
		if manifold.convex.count <= 0
		{
			return {}, .Missing;
		}
		best_index := 0;
		for index in 1 ..< int(manifold.convex.count)
		{
			if manifold.convex.contacts[index].depth > manifold.convex.contacts[best_index].depth
			{
				best_index = index;
			}
		}
		source := manifold.convex.contacts[best_index];
		return {source.offset, source.depth, manifold.convex.normal, source.feature_id}, .Present;
	}
	if manifold.nonconvex.count <= 0
	{
		return {}, .Missing;
	}
	best_index := 0;
	for index in 1 ..< int(manifold.nonconvex.count)
	{
		if manifold.nonconvex.contacts[index].depth > manifold.nonconvex.contacts[best_index].depth
		{
			best_index = index;
		}
	}
	return manifold.nonconvex.contacts[best_index], .Present;
}

sweep_task_registry_initialize :: proc "contextless" (registry: ^Sweep_Task_Registry) -> Physics_Status
{
	status := sweep_task_registry_reset(registry);
	if status != .Ok
	{
		return status;
	}
	for type_a in 0 ..< BUILT_IN_SHAPE_TYPE_COUNT
	{
		for type_b in type_a ..< BUILT_IN_SHAPE_TYPE_COUNT
		{
			if type_a == MESH_TYPE_ID && type_b == MESH_TYPE_ID
			{
				continue;
			}
			test := sweep_task_test_convex_distance;
			if type_b == MESH_TYPE_ID
			{
				if type_a <= CONVEX_HULL_TYPE_ID
				{
					test = sweep_task_test_convex_homogeneous_compound_distance;
				}
				else
				{
					test = sweep_task_test_compound_homogeneous_compound_distance;
				}
			}
			else if type_b >= COMPOUND_TYPE_ID
			{
				if type_a <= CONVEX_HULL_TYPE_ID
				{
					test = sweep_task_test_convex_compound_distance;
				}
				else
				{
					test = sweep_task_test_compound_pair_distance;
				}
			}
			_, register_status := sweep_task_registry_register_built_in(
				registry, type_a, type_b, test,
			);
			if register_status != .Ok
			{
				_ = sweep_task_registry_reset(registry);
				return register_status;
			}
		}
	}
	if registry.task_count != BUILT_IN_SWEEP_TASK_COUNT
	{
		_ = sweep_task_registry_reset(registry);
		return .Invalid_Description;
	}
	registry.state = .Ready;
	return .Ok;
}

sweep_task_registry_lookup :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int,
) -> (^Sweep_Task, Sweep_Task_Reference, Physics_Status)
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Sweep_Task_Registry phase=query reason=checked_task_reference lifetime=until_registry_reset_or_owner_lifetime_end
	if registry == nil || registry.state != .Ready || type_a < 0 || type_b < 0 ||
	type_a >= MAXIMUM_SHAPE_TYPE_COUNT || type_b >= MAXIMUM_SHAPE_TYPE_COUNT
	{
		return nil, {}, .Invalid_Argument;
	}
	reference := registry.routes[sweep_task_matrix_index(type_a, type_b)];
	if reference.task_id < 0 || int(reference.task_id) >= registry.task_count
	{
		return nil, reference, .Not_Found;
	}
	return &registry.tasks[reference.task_id], reference, .Ok;
}

Sweep_Task_Call_Context :: struct
{
	state:        Sweep_Task_Call_State,
	order:        Sweep_Task_Route_Order,
	filter:       Collision_Child_Filter_Proc,
	user_context: rawptr,
	registry:     ^Sweep_Task_Registry,
	pool:         ^util.Buffer_Pool,
}

sweep_task_call_filter :: proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	call_context := (^Sweep_Task_Call_Context)(user_context);
	if call_context == nil || call_context.state != .Ready || call_context.filter == nil
	{
		return .Reject;
	}
	source_child_a := child_a;
	source_child_b := child_b;
	if call_context.order == .Flipped
	{
		source_child_a, source_child_b = source_child_b, source_child_a;
	}
	return call_context.filter(
		call_context.user_context, pair_id, source_child_a, source_child_b,
	);
}

sweep_task_registry_test :: proc "contextless" (
	registry: ^Sweep_Task_Registry,
	shape_a, shape_b: Typed_Index,
	pose_a, pose_b: Rigid_Pose,
	velocity_a, velocity_b: Body_Velocity,
	maximum_t, minimum_progression, convergence_threshold: f32,
	maximum_iteration_count: int,
	shapes: ^Shape_Registry,
	collision_tasks: ^Collision_Task_Registry,
	filter: Collision_Child_Filter_Proc = nil,
	user_context: rawptr = nil,
	pool: ^util.Buffer_Pool = nil,
) -> (Sweep_Result, Physics_Status)
{
	raw_a, _, resolve_a_status := shape_registry_resolve(shapes, shape_a);
	if resolve_a_status != .Ok
	{
		return {}, resolve_a_status;
	}
	raw_b, _, resolve_b_status := shape_registry_resolve(shapes, shape_b);
	if resolve_b_status != .Ok
	{
		return {}, resolve_b_status;
	}
	type_a := int(typed_index_type(shape_a));
	type_b := int(typed_index_type(shape_b));
	task, reference, lookup_status := sweep_task_registry_lookup(registry, type_a, type_b);
	if lookup_status != .Ok
	{
		return {}, lookup_status;
	}
	if task.kind == .Contextual
	{
		return sweep_task_test_contextual(task.contextual, reference.order,
			raw_a, raw_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
			maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
			shapes, collision_tasks, filter, user_context);
	}
	if task.kind == .Built_In && task.test == sweep_task_test_convex_distance &&
	maximum_t >= 0 && minimum_progression > 0 && convergence_threshold > 0 &&
	maximum_iteration_count > 0
	{
		if type_a == SPHERE_TYPE_ID && type_b == BOX_TYPE_ID
		{
			result, handled := sweep_sphere_box_axial_linear(
				(^Sphere)(raw_a)^, (^Box)(raw_b)^, pose_a, pose_b,
				velocity_a, velocity_b, maximum_t,
			);
			if handled
			{
				return result, .Ok;
			}
		}
		else if type_a == BOX_TYPE_ID && type_b == SPHERE_TYPE_ID
		{
			result, handled := sweep_sphere_box_axial_linear(
				(^Sphere)(raw_b)^, (^Box)(raw_a)^, pose_b, pose_a,
				velocity_b, velocity_a, maximum_t,
			);
			if handled
			{
				result.normal = util.vector3_negate(result.normal);
				result.child_a, result.child_b = result.child_b, result.child_a;
				return result, .Ok;
			}
		}
	}
	task_filter := filter;
	task_user_context := user_context;
	call_context: Sweep_Task_Call_Context;
	if task.kind == .Built_In
	{
		call_context = {
			state=.Ready, order=reference.order, filter=filter, user_context=user_context,
			registry=registry, pool=pool,
		};
		if filter != nil
		{
			task_filter = sweep_task_call_filter;
		}
		task_user_context = &call_context;
	}
	if reference.order == .Expected
	{
		return task.test(
			raw_a, raw_b, type_a, type_b, pose_a, pose_b, velocity_a, velocity_b,
			maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
			shapes, collision_tasks, task_filter, task_user_context,
		);
	}
	result, status := task.test(
		raw_b, raw_a, type_b, type_a, pose_b, pose_a, velocity_b, velocity_a,
		maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
		shapes, collision_tasks, task_filter, task_user_context,
	);
	if status != .Ok
	{
		return {}, status;
	}
	result.normal = util.vector3_negate(result.normal);
	result.child_a, result.child_b = result.child_b, result.child_a;
	return result, .Ok;
}

// the binding is borrowed and must remain immutable and alive until registry
// reset and completion of all users. the facade owns a copy for world lifetime
sweep_task_registry_register_contextual :: proc "contextless" (
	registry: ^Sweep_Task_Registry, type_a, type_b: int,
	binding: ^Contextual_Sweep_Task_Binding,
) -> (i32, Physics_Status)
{
	return sweep_task_registry_register_internal(registry, type_a, type_b, nil, nil, .Contextual, binding);
}
