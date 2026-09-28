// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

MAXIMUM_COLLISION_TASK_COUNT :: 128;
COLLISION_TASK_MATRIX_ENTRY_COUNT :: MAXIMUM_SHAPE_TYPE_COUNT * MAXIMUM_SHAPE_TYPE_COUNT;
BUILT_IN_COLLISION_TASK_COUNT :: 45;
Collision_Task_Pair_Type :: enum u8
{
	Standard,
	Flipless,
	Sphere,
	Sphere_Including,
	Bounds_Tested,
}

Collision_Task_Kind :: enum u8
{
	Convex,
	Convex_Compound,
	Compound_Pair,
}

Collision_Task_Capability :: enum u8
{
	Convex_Result,
	Wide_Result,
	Subtask_Generator,
	Child_Order,
	Mesh_Reduction,
}

Collision_Task_Capabilities :: distinct bit_set[Collision_Task_Capability; u8];
Collision_Task_Route_Order :: enum u8
{
	Expected,
	Flipped,
}

Collision_Task_Registry_State :: enum u8
{
	Uninitialized,
	Ready,
}

Collision_Testing_State :: enum u8
{
	Reject,
	Allow,
}

Collision_Pair_Result_Proc :: #type proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^Manifold_Result,
) -> Physics_Status;
Collision_Stored_Pair_Result_Proc :: #type proc "contextless" (
	user_context: rawptr, pair_id: i32, manifold: ^Collision_Stored_Manifold,
) -> Physics_Status;
Collision_Child_Result_Proc :: #type proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32, manifold: ^Convex_Contact_Manifold,
) -> Physics_Status;
Collision_Child_Filter_Proc :: #type proc "contextless" (
	user_context: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State;
Collision_Wide_Manifold_Kind :: enum u8
{
	One_Contact,
	Two_Contact,
	Four_Contact,
}

Collision_Wide_Manifold_Result :: struct
{
	kind: Collision_Wide_Manifold_Kind,
	using data: struct #raw_union
	{
		one:  Convex_1_Contact_Manifold_Wide,
		two:  Convex_2_Contact_Manifold_Wide,
		four: Convex_4_Contact_Manifold_Wide,
	},
}
#assert(size_of(Collision_Wide_Manifold_Result) <= size_of(Convex_4_Contact_Manifold_Wide) + 32);
Collision_Wide_Test_Proc :: #type proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
) -> (Collision_Wide_Manifold_Result, Physics_Status);
collision_wide_test_into_proc :: #type proc "contextless" (
	bundle: ^Collision_Convex_Wide_Bundle, shapes: ^Shape_Registry,
	result: ^Collision_Wide_Manifold_Result,
) -> Physics_Status;
Collision_Result_Procedures :: struct
{
	pair_completed:       Collision_Pair_Result_Proc,
	child_pair_completed: Collision_Child_Result_Proc,
	allow_child_pair:     Collision_Child_Filter_Proc,
}

Collision_Task :: struct
{
	task_id:          i32,
	shape_type_a:     i16,
	shape_type_b:     i16,
	batch_size:       i16,
	pair_type:        Collision_Task_Pair_Type,
	kind:             Collision_Task_Kind,
	capabilities:     Collision_Task_Capabilities,
	// the tag occupies existing padding. the union preserves native field offsets
	dispatch: Collision_Task_Dispatch,
	using callbacks: struct #raw_union
	{
		using native: struct
		{
			convex_test:           Collision_Test_Proc,
			convex_wide_test:      Collision_Wide_Test_Proc,
			convex_wide_test_into: collision_wide_test_into_proc,
		},
		contextual: ^Contextual_Collision_Task_Binding,
	},
}

Collision_Task_Reference :: struct
{
	task_id:                i32,
	batch_size:             i16,
	expected_first_type_id: i16,
	pair_type:              Collision_Task_Pair_Type,
	order:                  Collision_Task_Route_Order,
}

Collision_Task_Registry :: struct
{
	tasks:      [MAXIMUM_COLLISION_TASK_COUNT]Collision_Task,
	routes:     [COLLISION_TASK_MATRIX_ENTRY_COUNT]Collision_Task_Reference,
	task_count: int,
	state:      Collision_Task_Registry_State,
}

collision_task_matrix_index :: proc "contextless" (type_a, type_b: int) -> int
{
	return type_a * MAXIMUM_SHAPE_TYPE_COUNT + type_b;
}

collision_task_registry_reset :: proc "contextless" (registry: ^Collision_Task_Registry) -> Physics_Status
{
	if registry == nil
	{
		return .Invalid_Argument;
	}
	registry^ = {};
	for index in 0 ..< COLLISION_TASK_MATRIX_ENTRY_COUNT
	{
		registry.routes[index].task_id = -1;
	}
	return .Ok;
}

collision_task_registry_validate_registration :: proc "contextless" (
	registry: ^Collision_Task_Registry, task: Collision_Task,
) -> Physics_Status
{
	if registry == nil || task.shape_type_a < 0 || task.shape_type_b < 0 ||
	int(task.shape_type_a) >= MAXIMUM_SHAPE_TYPE_COUNT || int(task.shape_type_b) >= MAXIMUM_SHAPE_TYPE_COUNT ||
	task.batch_size <= 0 || int(task.batch_size) > MAXIMUM_COLLISION_TASK_BATCH_SIZE ||
	registry.task_count >= MAXIMUM_COLLISION_TASK_COUNT
	{
		return .Invalid_Argument;
	}
	if task.dispatch == .Contextual
	{
		if task.kind != .Convex || task.contextual == nil ||
		task.contextual.test == nil || task.contextual.wide_test == nil ||
		task.capabilities != (Collision_Task_Capabilities{.Convex_Result, .Wide_Result})
		{
			return .Invalid_Description;
		}
	}
	else if task.dispatch == .Native
	{
		if task.kind == .Convex && task.convex_test == nil ||
		task.kind != .Convex && .Subtask_Generator not_in task.capabilities
		{
			return .Invalid_Argument;
		}
		if task.kind == .Convex
		{
			has_wide_test := task.convex_wide_test != nil || task.convex_wide_test_into != nil;
			if .Wide_Result in task.capabilities && !has_wide_test
			{
				return .Invalid_Description;
			}
			if .Wide_Result not_in task.capabilities && has_wide_test
			{
				return .Invalid_Description;
			}
		}
	}
	else
	{
		return .Invalid_Description;
	}
	a := int(task.shape_type_a);
	b := int(task.shape_type_b);
	if registry.routes[collision_task_matrix_index(a, b)].task_id >= 0 ||
	registry.routes[collision_task_matrix_index(b, a)].task_id >= 0
	{
		return .Invalid_Description;
	}
	return .Ok;
}

collision_task_registry_register :: proc "contextless" (
	registry: ^Collision_Task_Registry, task: Collision_Task,
) -> (i32, Physics_Status)
{
	status: Physics_Status = collision_task_registry_validate_registration(registry, task);
	if status != .Ok
	{
		return -1, status;
	}
	a := int(task.shape_type_a);
	b := int(task.shape_type_b);
	task_id := i32(registry.task_count);
	registered := task;
	registered.task_id = task_id;
	registry.tasks[registry.task_count] = registered;
	reference := Collision_Task_Reference{
		task_id=task_id,
		batch_size=task.batch_size,
		expected_first_type_id=task.shape_type_a,
		pair_type=task.pair_type,
	};
	registry.routes[collision_task_matrix_index(a, b)] = reference;
	reference.order = .Flipped;
	if a == b
	{
		reference.order = .Expected;
	}
	registry.routes[collision_task_matrix_index(b, a)] = reference;
	registry.task_count += 1;
	return task_id, .Ok;
}

collision_task_registry_register_convex :: proc "contextless" (
	registry: ^Collision_Task_Registry, type_a, type_b, batch_size: int,
	pair_type: Collision_Task_Pair_Type, test: Collision_Test_Proc,
	wide_test: Collision_Wide_Test_Proc = nil,
) -> Physics_Status
{
	capabilities := Collision_Task_Capabilities{.Convex_Result};
	if wide_test != nil
	{
		capabilities += {.Wide_Result};
	}
	_, status := collision_task_registry_register(registry, {
			shape_type_a=i16(type_a), shape_type_b=i16(type_b), batch_size=i16(batch_size),
			pair_type=pair_type, kind=.Convex, capabilities=capabilities,
			convex_test=test, convex_wide_test=wide_test,
	});
	return status;
}

collision_task_registry_register_convex_built_in :: proc "contextless" (
	registry: ^Collision_Task_Registry, type_a, type_b, batch_size: int,
	pair_type: Collision_Task_Pair_Type, test: Collision_Test_Proc,
	wide_test: Collision_Wide_Test_Proc, wide_test_into: collision_wide_test_into_proc,
) -> Physics_Status
{
	if wide_test == nil || wide_test_into == nil
	{
		return .Invalid_Argument;
	}
	_, status := collision_task_registry_register(registry, {
			shape_type_a=i16(type_a), shape_type_b=i16(type_b), batch_size=i16(batch_size),
			pair_type=pair_type, kind=.Convex, capabilities={.Convex_Result, .Wide_Result},
			convex_test=test, convex_wide_test=wide_test, convex_wide_test_into=wide_test_into,
	});
	return status;
}

collision_task_registry_register_compound :: proc "contextless" (
	registry: ^Collision_Task_Registry, type_a, type_b, batch_size: int,
	kind: Collision_Task_Kind, capabilities: Collision_Task_Capabilities,
) -> Physics_Status
{
	_, status := collision_task_registry_register(registry, {
			shape_type_a=i16(type_a), shape_type_b=i16(type_b), batch_size=i16(batch_size),
			pair_type=.Bounds_Tested, kind=kind, capabilities=capabilities,
	});
	return status;
}

collision_task_registry_register_built_ins :: proc "contextless" (registry: ^Collision_Task_Registry) -> Physics_Status
{
	if registry == nil
	{
		return .Invalid_Argument;
	}
	convex_registrations := [21]struct
	{
		a, b, batch_size: int,
		pair_type: Collision_Task_Pair_Type,
		test: Collision_Test_Proc,
		wide_test: Collision_Wide_Test_Proc,
		wide_test_into: collision_wide_test_into_proc,
	}{
		{
			0,
			0,
			32,
			.Sphere,
			sphere_pair_test,
			collision_task_execute_sphere_pair_wide,
			collision_task_execute_sphere_pair_wide_into,
		},
		{
			0,
			1,
			32,
			.Sphere_Including,
			sphere_capsule_test,
			collision_task_execute_sphere_capsule_wide,
			collision_task_execute_sphere_capsule_wide_into,
		},
		{
			0,
			2,
			32,
			.Sphere_Including,
			sphere_box_test,
			collision_task_execute_sphere_box_wide,
			collision_task_execute_sphere_box_wide_into,
		},
		{
			0,
			3,
			32,
			.Sphere_Including,
			sphere_triangle_test,
			collision_task_execute_sphere_triangle_wide,
			collision_task_execute_sphere_triangle_wide_into,
		},
		{
			0,
			4,
			32,
			.Sphere_Including,
			sphere_cylinder_test,
			collision_task_execute_sphere_cylinder_wide,
			collision_task_execute_sphere_cylinder_wide_into,
		},
		{
			0,
			5,
			16,
			.Sphere_Including,
			sphere_convex_hull_test,
			collision_task_execute_sphere_hull_wide,
			collision_task_execute_sphere_hull_wide_into,
		},
		{
			1,
			1,
			32,
			.Flipless,
			capsule_pair_test,
			collision_task_execute_capsule_pair_wide,
			collision_task_execute_capsule_pair_wide_into,
		},
		{
			1,
			2,
			32,
			.Standard,
			capsule_box_test,
			collision_task_execute_capsule_box_wide,
			collision_task_execute_capsule_box_wide_into,
		},
		{
			1,
			3,
			32,
			.Standard,
			capsule_triangle_test,
			collision_task_execute_capsule_triangle_wide,
			collision_task_execute_capsule_triangle_wide_into,
		},
		{
			1,
			4,
			32,
			.Standard,
			capsule_cylinder_test,
			collision_task_execute_capsule_cylinder_wide,
			collision_task_execute_capsule_cylinder_wide_into,
		},
		{
			1,
			5,
			16,
			.Standard,
			capsule_convex_hull_test,
			collision_task_execute_capsule_hull_wide,
			collision_task_execute_capsule_hull_wide_into,
		},
		{
			2,
			2,
			32,
			.Flipless,
			box_pair_test,
			collision_task_execute_box_pair_wide,
			collision_task_execute_box_pair_wide_into,
		},
		{
			2,
			3,
			32,
			.Standard,
			box_triangle_test,
			collision_task_execute_box_triangle_wide,
			collision_task_execute_box_triangle_wide_into,
		},
		{
			2,
			4,
			16,
			.Standard,
			box_cylinder_test,
			collision_task_execute_box_cylinder_wide,
			collision_task_execute_box_cylinder_wide_into,
		},
		{
			2,
			5,
			16,
			.Standard,
			box_convex_hull_test,
			collision_task_execute_box_hull_wide,
			collision_task_execute_box_hull_wide_into,
		},
		{
			3,
			3,
			32,
			.Flipless,
			triangle_pair_test,
			collision_task_execute_triangle_pair_wide,
			collision_task_execute_triangle_pair_wide_into,
		},
		{
			3,
			4,
			16,
			.Standard,
			triangle_cylinder_test,
			collision_task_execute_triangle_cylinder_wide,
			collision_task_execute_triangle_cylinder_wide_into,
		},
		{
			3,
			5,
			16,
			.Standard,
			triangle_convex_hull_test,
			collision_task_execute_triangle_hull_wide,
			collision_task_execute_triangle_hull_wide_into,
		},
		{
			4,
			4,
			16,
			.Flipless,
			cylinder_pair_test,
			collision_task_execute_cylinder_pair_wide,
			collision_task_execute_cylinder_pair_wide_into,
		},
		{
			4,
			5,
			16,
			.Standard,
			cylinder_convex_hull_test,
			collision_task_execute_cylinder_hull_wide,
			collision_task_execute_cylinder_hull_wide_into,
		},
		{
			5,
			5,
			16,
			.Flipless,
			convex_hull_pair_test,
			collision_task_execute_hull_pair_wide,
			collision_task_execute_hull_pair_wide_into,
		},
	};
	for registration in convex_registrations
	{
		status := collision_task_registry_register_convex_built_in(
			registry, registration.a, registration.b, registration.batch_size,
			registration.pair_type, registration.test, registration.wide_test,
			registration.wide_test_into,
		);
		if status != .Ok
		{
			return status;
		}
	}
	for convex_type in SPHERE_TYPE_ID ..= CONVEX_HULL_TYPE_ID
	{
		for compound_type in COMPOUND_TYPE_ID ..= MESH_TYPE_ID
		{
			capabilities := Collision_Task_Capabilities{.Subtask_Generator, .Child_Order};
			if compound_type == MESH_TYPE_ID
			{
				capabilities += {.Mesh_Reduction};
			}
			status := collision_task_registry_register_compound(
				registry, convex_type, compound_type, 16, .Convex_Compound, capabilities,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	for type_a in COMPOUND_TYPE_ID ..= MESH_TYPE_ID
	{
		for type_b in type_a ..= MESH_TYPE_ID
		{
			capabilities := Collision_Task_Capabilities{.Subtask_Generator, .Child_Order};
			if type_a == MESH_TYPE_ID || type_b == MESH_TYPE_ID
			{
				capabilities += {.Mesh_Reduction};
			}
			status := collision_task_registry_register_compound(
				registry, type_a, type_b, 16, .Compound_Pair, capabilities,
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

collision_task_registry_validate_built_ins :: proc "contextless" (registry: ^Collision_Task_Registry) -> Physics_Status
{
	if registry == nil || registry.task_count < BUILT_IN_COLLISION_TASK_COUNT
	{
		return .Invalid_Description;
	}
	for type_a in 0 ..< BUILT_IN_SHAPE_TYPE_COUNT
	{
		for type_b in type_a ..< BUILT_IN_SHAPE_TYPE_COUNT
		{
			reference := registry.routes[collision_task_matrix_index(type_a, type_b)];
			if reference.task_id < 0 || int(reference.task_id) >= registry.task_count
			{
				return .Invalid_Description;
			}
			reverse := registry.routes[collision_task_matrix_index(type_b, type_a)];
			if reverse.task_id != reference.task_id
			{
				return .Invalid_Description;
			}
		}
	}
	for task_index in 0 ..< 21
	{
		task := &registry.tasks[task_index];
		if task.kind != .Convex || task.convex_wide_test == nil || task.convex_wide_test_into == nil ||
		.Wide_Result not_in task.capabilities
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

collision_task_registry_initialize :: proc "contextless" (registry: ^Collision_Task_Registry) -> Physics_Status
{
	status := collision_task_registry_reset(registry);
	if status != .Ok
	{
		return status;
	}
	status = collision_task_registry_register_built_ins(registry);
	if status != .Ok
	{
		_ = collision_task_registry_reset(registry);
		return status;
	}
	status = collision_task_registry_validate_built_ins(registry);
	if status != .Ok
	{
		_ = collision_task_registry_reset(registry);
		return status;
	}
	registry.state = .Ready;
	return .Ok;
}

collision_task_registry_lookup :: proc "contextless" (
	registry: ^Collision_Task_Registry, type_a, type_b: int,
) -> (^Collision_Task, Collision_Task_Reference, Physics_Status)
{ // odin-contracts-allow: perf-internal rule=ODIN_PUBLIC_RAWPTR_RETURN owner=Collision_Task_Registry phase=task_lookup reason=direct_registered_task_access lifetime=until_registry_reset_or_disposal
	if registry == nil || registry.state != .Ready || type_a < 0 || type_b < 0 ||
	type_a >= MAXIMUM_SHAPE_TYPE_COUNT || type_b >= MAXIMUM_SHAPE_TYPE_COUNT
	{
		return nil, {}, .Invalid_Argument;
	}
	reference := registry.routes[collision_task_matrix_index(type_a, type_b)];
	if reference.task_id < 0 || int(reference.task_id) >= registry.task_count
	{
		return nil, reference, .Not_Found;
	}
	return &registry.tasks[reference.task_id], reference, .Ok;
}

collision_task_registry_test_convex :: proc "contextless" (
	registry: ^Collision_Task_Registry, type_a, type_b: int,
	shape_a, shape_b: rawptr, pose_a, pose_b: Rigid_Pose,
	speculative_margin: f32, shapes: ^Shape_Registry,
) -> (Convex_Contact_Manifold, Physics_Status)
{
	task, reference, status := collision_task_registry_lookup(registry, type_a, type_b);
	if status != .Ok
	{
		return {}, status;
	}
	if task.kind != .Convex
	{
		return {}, .Invalid_Argument;
	}
	if task.dispatch == .Contextual
	{
		return collision_task_test_contextual(task.contextual, reference.order,
			shape_a, shape_b, pose_a, pose_b, speculative_margin, shapes);
	}
	if reference.order == .Expected
	{
		return task.convex_test(shape_a, shape_b, pose_a, pose_b, speculative_margin, shapes);
	}
	manifold, test_status := task.convex_test(shape_b, shape_a, pose_b, pose_a, speculative_margin, shapes);
	if test_status != .Ok
	{
		return {}, test_status;
	}
	return convex_manifold_flip(manifold), .Ok;
}
