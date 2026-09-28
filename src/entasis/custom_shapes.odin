package entasis

import physics "entasis:entasis_physics"

// MAXIMUM_SHAPE_TYPE_COUNT is the fixed shape type registry capacity
MAXIMUM_SHAPE_TYPE_COUNT     :: physics.MAXIMUM_SHAPE_TYPE_COUNT;
// BUILT_IN_SHAPE_TYPE_COUNT is the number of shape type IDs reserved by Entasis
BUILT_IN_SHAPE_TYPE_COUNT    :: physics.BUILT_IN_SHAPE_TYPE_COUNT;
// MAXIMUM_COLLISION_TASK_COUNT is the fixed collision task registry capacity
MAXIMUM_COLLISION_TASK_COUNT :: physics.MAXIMUM_COLLISION_TASK_COUNT;
// MAXIMUM_SWEEP_TASK_COUNT is the fixed sweep task registry capacity
MAXIMUM_SWEEP_TASK_COUNT     :: physics.MAXIMUM_SWEEP_TASK_COUNT;
// Shape_Batch_Type classifies custom shape storage and traversal behavior
Shape_Batch_Type :: physics.Shape_Batch_Type;
// Shape_Bounds stores a custom shape bounding box and angular expansion
Shape_Bounds     :: physics.Shape_Bounds;
// Shape_Ray_Hit stores one ray hit against a custom shape
Shape_Ray_Hit    :: physics.Shape_Ray_Hit;
// Shape_Registry is the low-level registry used by custom shape callbacks
Shape_Registry   :: physics.Shape_Registry;
// Shape_Bounds_Proc computes bounds for one custom shape value
Shape_Bounds_Proc  :: physics.Shape_Bounds_Proc;
// Shape_Inertia_Proc computes inertia for one custom shape value
Shape_Inertia_Proc :: physics.Shape_Inertia_Proc;
// Shape_Ray_Proc tests one ray against a custom shape value
Shape_Ray_Proc     :: physics.Shape_Ray_Proc;
// Shape_Support_Proc returns a support point for one custom convex shape
Shape_Support_Proc :: physics.Shape_Support_Proc;
// Shape_Dispose_Proc releases caller-owned data stored by a custom shape
Shape_Dispose_Proc :: physics.Shape_Dispose_Proc;
// Collision_Task_Pair_Type selects direct or bounds-tested pair batching
Collision_Task_Pair_Type   :: physics.Collision_Task_Pair_Type;
// Collision_Task_Kind classifies the manifold produced by a collision task
Collision_Task_Kind        :: physics.Collision_Task_Kind;
// Collision_Task_Capability identifies one result form supported by a collision task
Collision_Task_Capability  :: physics.Collision_Task_Capability;
// Collision_Task_Capabilities combines supported collision task result forms
Collision_Task_Capabilities :: physics.Collision_Task_Capabilities;
// Collision_Test_Proc tests one scalar pair for a convex manifold
Collision_Test_Proc        :: physics.Collision_Test_Proc;
// Collision_Wide_Test_Proc tests one SIMD pair bundle
Collision_Wide_Test_Proc   :: physics.Collision_Wide_Test_Proc;
// Collision_Wide_Manifold_Result stores SIMD manifold output
Collision_Wide_Manifold_Result :: physics.Collision_Wide_Manifold_Result;
// Collision_Convex_Wide_Bundle stores SIMD convex pair input
Collision_Convex_Wide_Bundle :: physics.Collision_Convex_Wide_Bundle;
// Sweep_Test_Proc sweeps one registered shape pair
Sweep_Test_Proc       :: physics.Sweep_Test_Proc;
// Sweep_Child_Test_Proc sweeps one child pair inside a compound route
Sweep_Child_Test_Proc :: physics.Sweep_Child_Test_Proc;
// Custom_Shape_Registration describes storage and callbacks for one custom shape type
Custom_Shape_Registration :: struct
{
	expected_type_id: Shape_Type_ID,
	size:             int,
	alignment:        int,
	batch_type:       Shape_Batch_Type,
	bounds:           Shape_Bounds_Proc,
	inertia:          Shape_Inertia_Proc,
	ray:              Shape_Ray_Proc,
	support:          Shape_Support_Proc,
	sweep_support:    Shape_Support_Proc,
	dispose:          Shape_Dispose_Proc,
}

// custom_shape_registration infers storage size and alignment for one custom
// shape value type. SHAPE_TYPE_INVALID accepts the next sequential custom ID
custom_shape_registration :: #force_inline proc "contextless" (
	$T: typeid,
	batch_type: Shape_Batch_Type,
	bounds: Shape_Bounds_Proc,
	inertia: Shape_Inertia_Proc,
	ray: Shape_Ray_Proc,
	support: Shape_Support_Proc,
	sweep_support: Shape_Support_Proc = nil,
	dispose: Shape_Dispose_Proc = physics.shape_no_dispose,
	expected_type_id: Shape_Type_ID = SHAPE_TYPE_INVALID,
) -> Custom_Shape_Registration
{
	resolved_sweep_support := sweep_support;
	if resolved_sweep_support == nil
	{
		resolved_sweep_support = support;
	}
	return {
		expected_type_id=expected_type_id,
		size=size_of(T),
		alignment=align_of(T),
		batch_type=batch_type,
		bounds=bounds,
		inertia=inertia,
		ray=ray,
		support=support,
		sweep_support=resolved_sweep_support,
		dispose=dispose,
	};
}

@(private)
extension_world_ready :: #force_inline proc "contextless" (
	world: ^World,
) -> (^world_data, ^physics.Simulation, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return nil, nil, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return nil, nil, .Invalid_Argument;
	}
	return data, &data.simulation, .Ok;
}

// custom_shape_next_type_id returns the sequential ID required by the registry
custom_shape_next_type_id :: proc "contextless" (
	world: ^World,
) -> (Shape_Type_ID, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return SHAPE_TYPE_INVALID, status;
	}
	registry := physics.simulation_shape_registry(simulation);
	if registry == nil || registry.state != .Allocated
	{
		return SHAPE_TYPE_INVALID, .Disposed;
	}
	if registry.registered_type_count >= MAXIMUM_SHAPE_TYPE_COUNT
	{
		return SHAPE_TYPE_INVALID, .Capacity_Missing;
	}
	return Shape_Type_ID(registry.registered_type_count), .Ok;
}

// custom_shape_register installs one custom shape type. register custom types and
// their collision/sweep routes before the first world step
custom_shape_register :: proc (
	world: ^World,
	registration: Custom_Shape_Registration,
) -> (Shape_Type_ID, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return SHAPE_TYPE_INVALID, status;
	}
	if simulation.step_index != 0
	{
		return SHAPE_TYPE_INVALID, .Invalid_Argument;
	}
	registry := physics.simulation_shape_registry(simulation);
	if registry == nil
	{
		return SHAPE_TYPE_INVALID, .Disposed;
	}
	if registry.registered_type_count >= MAXIMUM_SHAPE_TYPE_COUNT
	{
		return SHAPE_TYPE_INVALID, .Capacity_Missing;
	}
	next_id := Shape_Type_ID(registry.registered_type_count);
	if registration.expected_type_id != SHAPE_TYPE_INVALID &&
	registration.expected_type_id != next_id
	{
		return SHAPE_TYPE_INVALID, .Invalid_Description;
	}
	low_level := physics.Shape_Type_Registration{
		size=registration.size,
		alignment=registration.alignment,
		batch_type=registration.batch_type,
		bounds=registration.bounds,
		inertia=registration.inertia,
		ray=registration.ray,
		support=registration.support,
		sweep_support=registration.sweep_support,
		dispose=registration.dispose,
	};
	type_id, register_status := physics.shape_registry_register_custom(registry, low_level);
	if register_status != .Ok
	{
		return SHAPE_TYPE_INVALID, register_status;
	}
	return Shape_Type_ID(type_id), .Ok;
}

// custom_shape_add stores one value under a previously registered custom type
custom_shape_add :: #force_inline proc (
	world: ^World,
	type_id: Shape_Type_ID,
	shape: ^$T,
) -> (Shape_Handle, Status)
{
	if int(type_id) < BUILT_IN_SHAPE_TYPE_COUNT
	{
		return shape_handle_invalid(), .Invalid_Argument;
	}
	return shape_add_typed(world, type_id, shape);
}

// Collision_Task_Registration describes one custom collision pair route
Collision_Task_Registration :: struct
{
	shape_type_a: Shape_Type_ID,
	shape_type_b: Shape_Type_ID,
	batch_size:   int,
	pair_type:    Collision_Task_Pair_Type,
	kind:         Collision_Task_Kind,
	capabilities: Collision_Task_Capabilities,
	convex_test:  Collision_Test_Proc,
	wide_test:    Collision_Wide_Test_Proc,
}

// collision_task_convex builds a convex custom collision task registration
collision_task_convex :: #force_inline proc "contextless" (
	shape_type_a, shape_type_b: Shape_Type_ID,
	batch_size: int,
	pair_type: Collision_Task_Pair_Type,
	test: Collision_Test_Proc,
	wide_test: Collision_Wide_Test_Proc,
) -> Collision_Task_Registration
{
	capabilities := Collision_Task_Capabilities{.Convex_Result, .Wide_Result};
	return {
		shape_type_a=shape_type_a,
		shape_type_b=shape_type_b,
		batch_size=batch_size,
		pair_type=pair_type,
		kind=.Convex,
		capabilities=capabilities,
		convex_test=test,
		wide_test=wide_test,
	};
}

// collision_task_compound builds a bounds-tested compound collision task registration
collision_task_compound :: #force_inline proc "contextless" (
	shape_type_a, shape_type_b: Shape_Type_ID,
	batch_size: int,
	kind: Collision_Task_Kind,
	capabilities: Collision_Task_Capabilities,
) -> Collision_Task_Registration
{
	return {
		shape_type_a=shape_type_a,
		shape_type_b=shape_type_b,
		batch_size=batch_size,
		pair_type=.Bounds_Tested,
		kind=kind,
		capabilities=capabilities,
	};
}

// collision_task_register installs one pair route. existing built-in routes
// cannot be replaced
collision_task_register :: proc (
	world: ^World,
	registration: Collision_Task_Registration,
) -> (i32, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return -1, status;
	}
	if simulation.step_index != 0
	{
		return -1, .Invalid_Argument;
	}
	if registration.kind == .Convex &&
	(registration.convex_test == nil || registration.wide_test == nil)
	{
		return -1, .Invalid_Description;
	}
	task := physics.Collision_Task{
		shape_type_a=i16(registration.shape_type_a),
		shape_type_b=i16(registration.shape_type_b),
		batch_size=i16(registration.batch_size),
		pair_type=registration.pair_type,
		kind=registration.kind,
		capabilities=registration.capabilities,
		convex_test=registration.convex_test,
		convex_wide_test=registration.wide_test,
	};
	return physics.collision_task_registry_register(&simulation.collision_tasks, task);
}

// Sweep_Task_Registration describes one custom sweep pair route
Sweep_Task_Registration :: struct
{
	shape_type_a: Shape_Type_ID,
	shape_type_b: Shape_Type_ID,
	test:          Sweep_Test_Proc,
	child_test:   Sweep_Child_Test_Proc,
}

// sweep_task_registration builds a custom sweep task registration
sweep_task_registration :: #force_inline proc "contextless" (
	shape_type_a, shape_type_b: Shape_Type_ID,
	test: Sweep_Test_Proc,
	child_test: Sweep_Child_Test_Proc,
) -> Sweep_Task_Registration
{
	return {shape_type_a, shape_type_b, test, child_test};
}

// sweep_task_register installs one custom sweep route. custom registrations must
// provide both top-level and child-test procedures
sweep_task_register :: proc (
	world: ^World,
	registration: Sweep_Task_Registration,
) -> (i32, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return -1, status;
	}
	if simulation.step_index != 0
	{
		return -1, .Invalid_Argument;
	}
	return physics.sweep_task_registry_register(
		&simulation.sweep_tasks,
		int(registration.shape_type_a),
		int(registration.shape_type_b),
		registration.test,
		registration.child_test,
	);
}

// custom_shape_add_raw copies an exact-sized payload into a registered custom
// batch. source alignment is immaterial. destination alignment is the registered
// alignment. resources described by the bytes transfer only on success
custom_shape_add_raw :: proc (
	world: ^World, type_id: Shape_Type_ID, shape: rawptr, size: int,
) -> (Shape_Handle, Status)
{
	if int(type_id) < BUILT_IN_SHAPE_TYPE_COUNT || shape == nil || size <= 0
	{
		return shape_handle_invalid(), .Invalid_Argument;
	}
	registry: ^physics.Shape_Registry;
	status: Status;
	registry, status = shape_registry_from_world(world);
	if status != .Ok
	{
		return shape_handle_invalid(), status;
	}
	handle: Shape_Handle;
	add_status: Status;
	handle, add_status = physics.shape_registry_add_raw(registry, int(type_id), shape, size);
	if add_status != .Ok
	{
		return shape_handle_invalid(), add_status;
	}
	return handle, .Ok;
}

// custom_shape_data returns an immutable borrowed custom payload description.
// it does not write world state. the borrow expires at the next world mutation.
// callers must not alter or free the bytes or infer ownership from a copy
custom_shape_data :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (Shape_Access_Value, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return {}, status;
	}
	value: Shape_Access_Value;
	resolve_status: Status;
	value, resolve_status = physics.shape_access_resolve(
		Shape_Access(physics.simulation_shape_registry(simulation)), handle);
	if resolve_status != .Ok
	{
		return {}, resolve_status;
	}
	if int(value.type_id) < BUILT_IN_SHAPE_TYPE_COUNT
	{
		return {}, .Invalid_Argument;
	}
	return value, .Ok;
}

// custom_shape_inertia invokes an existing custom instance's registered inertia
// callback. ordinary owner-thread/idle-world callback rules apply
custom_shape_inertia :: proc "contextless" (
	world: ^World, handle: Shape_Handle, mass: f32,
) -> (Body_Inertia, Status)
{
	_, simulation, status := extension_world_ready(world);
	if status != .Ok
	{
		return {}, status;
	}
	if int(physics.typed_index_type(handle)) < BUILT_IN_SHAPE_TYPE_COUNT
	{
		return {}, .Invalid_Argument;
	}
	return physics.shape_registry_compute_inertia(physics.simulation_shape_registry(simulation), handle, mass);
}
