// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"

Shape_Support_Mode :: enum u8
{
	Collision, Sweep
}

Shape_Callback_Dispatch :: enum u8
{
	Native,
	Contextual,
}

// borrowed, read-only callback scope. it may not be retained, used to mutate the
// registry, or used after the callback returns. it contains no world scratch
Shape_Access :: distinct rawptr;
Contextual_Shape_Bounds_Proc :: #type proc "contextless" (
	user_context, shape: rawptr, orientation: ^util.Quaternion,
	access: Shape_Access, result: ^Shape_Bounds,
) -> Physics_Status;
Contextual_Shape_Inertia_Proc :: #type proc "contextless" (
	user_context, shape: rawptr, mass: f32,
	access: Shape_Access, result: ^Body_Inertia,
) -> Physics_Status;
Contextual_Shape_Ray_Proc :: #type proc "contextless" (
	user_context, shape: rawptr, pose: ^Rigid_Pose, ray: ^Tree_Ray,
	access: Shape_Access, result: ^Shape_Ray_Hit,
) -> Physics_Status;
Contextual_Shape_Support_Proc :: #type proc "contextless" (
	user_context, shape: rawptr, direction: ^util.Vector3,
	access: Shape_Access, result: ^util.Vector3,
) -> Physics_Status;
Contextual_Shape_Dispose_Proc :: #type proc "contextless" (
	user_context, shape: rawptr, access: Shape_Access, pool: ^util.Buffer_Pool,
) -> Physics_Status;
// immutable while registered. low-level callers retain this binding through
// registry disposal. the facade instead copies it into world-owned storage
Contextual_Shape_Binding :: struct
{
	user_context: rawptr,
	bounds: Contextual_Shape_Bounds_Proc,
	inertia: Contextual_Shape_Inertia_Proc,
	ray: Contextual_Shape_Ray_Proc,
	support: Contextual_Shape_Support_Proc,
	sweep_support: Contextual_Shape_Support_Proc,
	dispose: Contextual_Shape_Dispose_Proc,
}

Contextual_Shape_Type_Metadata :: struct
{
	size: int,
	alignment: int,
	batch_type: Shape_Batch_Type,
	binding: ^Contextual_Shape_Binding,
}

// overlay only mutually exclusive registration metadata. instance bytes and all
// existing native registration fields retain their exact representation
#assert(offset_of(Contextual_Shape_Type_Metadata, size) == offset_of(Shape_Type_Registration, size));
#assert(offset_of(Contextual_Shape_Type_Metadata, alignment) == offset_of(Shape_Type_Registration, alignment));
#assert(offset_of(Contextual_Shape_Type_Metadata, batch_type) == offset_of(Shape_Type_Registration, batch_type));
#assert(size_of(Contextual_Shape_Type_Metadata) <= size_of(Shape_Type_Registration));
#assert(align_of(Contextual_Shape_Type_Metadata) == align_of(Shape_Type_Registration));
shape_registry_validate_contextual :: proc "contextless" (
	registry: ^Shape_Registry, registration: Contextual_Shape_Type_Metadata,
) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated
	{
		return .Disposed;
	}
	if registry.registered_type_count < BUILT_IN_SHAPE_TYPE_COUNT
	{
		return .Invalid_Argument;
	}
	if registry.registered_type_count >= MAXIMUM_SHAPE_TYPE_COUNT
	{
		return .Capacity_Missing;
	}
	// pool blocks guarantee 128-byte base alignment. rounded shape stride makes
	// every slot satisfy supported alignment, including after growth/resize
	if registration.size <= 0 || registration.alignment <= 0 ||
	registration.alignment > util.BUFFER_POOL_BLOCK_ALIGNMENT ||
	registration.alignment & (registration.alignment - 1) != 0 ||
	registration.size > max(int) - (registration.alignment - 1) ||
	registration.batch_type > .Homogeneous_Compound || registration.binding == nil
	{
		return .Invalid_Description;
	}
	binding: ^Contextual_Shape_Binding = registration.binding;
	if binding.bounds == nil || binding.inertia == nil || binding.ray == nil ||
	binding.support == nil || binding.sweep_support == nil || binding.dispose == nil
	{
		return .Invalid_Description;
	}
	return .Ok;
}

shape_registry_register_contextual :: proc (
	registry: ^Shape_Registry, registration: Contextual_Shape_Type_Metadata,
) -> (i32, Physics_Status)
{
	status: Physics_Status = shape_registry_validate_contextual(registry, registration);
	if status != .Ok
	{
		return -1, status;
	}
	type_id: int = registry.registered_type_count;
	status = shape_registry_register_batch(registry, type_id, {
			stride=shape_stride(registration.size, registration.alignment),
			contextual_metadata=registration, state=.Registered, dispatch=.Contextual,
	});
	if status != .Ok
	{
		return -1, status;
	}
	return i32(type_id), .Ok;
}

shape_contextual_no_dispose :: proc "contextless" (
	user_context, shape: rawptr, access: Shape_Access, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	_ = user_context;
	_ = shape;
	_ = access;
	_ = pool;
	return .Ok;
}

shape_batch_bounds_state :: #force_inline proc "contextless" (batch: ^Shape_Batch) -> Reference_State
{
	if batch.dispatch == .Contextual
	{
		return .Present if batch.contextual_metadata.binding != nil && batch.contextual_metadata.binding.bounds != nil else .Missing;
	}
	return .Present if batch.metadata.bounds != nil else .Missing;
}

shape_batch_compute_bounds :: #force_inline proc "contextless" (
	batch: ^Shape_Batch, shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	if batch.dispatch == .Contextual
	{
		return shape_contextual_compute_bounds(batch.contextual_metadata.binding, shape, orientation, registry);
	}
	return batch.metadata.bounds(shape, orientation, registry);
}

@(private)
shape_contextual_compute_bounds :: #force_no_inline proc "contextless" (
	binding: ^Contextual_Shape_Binding, shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	result: Shape_Bounds;
	value: util.Quaternion = orientation;
	status: Physics_Status = binding.bounds(binding.user_context, shape, &value, Shape_Access(registry), &result);
	if status != .Ok
	{
		return {}, status;
	}
	return result, .Ok;
}

shape_batch_compute_inertia :: #force_inline proc "contextless" (
	batch: ^Shape_Batch, shape: rawptr, registry: ^Shape_Registry, mass: f32,
) -> (Body_Inertia, Physics_Status)
{
	if batch.dispatch == .Contextual
	{
		binding: ^Contextual_Shape_Binding = batch.contextual_metadata.binding;
		result: Body_Inertia;
		status: Physics_Status = binding.inertia(binding.user_context, shape, mass, Shape_Access(registry), &result);
		if status != .Ok
		{
			return {}, status;
		}
		return result, .Ok;
	}
	return batch.metadata.inertia(shape, registry, mass);
}

shape_batch_ray_test :: #force_inline proc "contextless" (
	batch: ^Shape_Batch, shape: rawptr, pose: Rigid_Pose, ray: Tree_Ray, registry: ^Shape_Registry,
) -> (Shape_Ray_Hit, Physics_Status)
{
	if batch.dispatch == .Contextual
	{
		return shape_contextual_ray_test(batch.contextual_metadata.binding, shape, pose, ray, registry);
	}
	return batch.metadata.ray(shape, pose, ray, registry);
}

@(private)
shape_contextual_ray_test :: #force_no_inline proc "contextless" (
	binding: ^Contextual_Shape_Binding, shape: rawptr, pose: Rigid_Pose, ray: Tree_Ray, registry: ^Shape_Registry,
) -> (Shape_Ray_Hit, Physics_Status)
{
	result: Shape_Ray_Hit = shape_ray_miss();
	pose_value: Rigid_Pose = pose;
	ray_value: Tree_Ray = ray;
	status: Physics_Status = binding.ray(binding.user_context, shape, &pose_value, &ray_value, Shape_Access(registry), &result);
	if status != .Ok
	{
		return shape_ray_miss(), status;
	}
	return result, .Ok;
}

shape_batch_support :: #force_inline proc "contextless" (
	batch: ^Shape_Batch, shape: rawptr, direction: util.Vector3, registry: ^Shape_Registry,
	$mode: Shape_Support_Mode,
) -> (util.Vector3, Physics_Status)
{
	if batch.dispatch == .Contextual
	{
		return shape_contextual_support(batch.contextual_metadata.binding, shape, direction, registry, mode);
	}
	when mode == .Sweep
	{
		return batch.metadata.sweep_support(shape, direction, registry);
	}
	else
	{
		return batch.metadata.support(shape, direction, registry);
	}
}

shape_contextual_support :: #force_inline proc "contextless" (
	binding: ^Contextual_Shape_Binding, shape: rawptr, direction: util.Vector3, registry: ^Shape_Registry,
	$mode: Shape_Support_Mode,
) -> (util.Vector3, Physics_Status)
{
	value: util.Vector3 = direction;
	result: util.Vector3;
	callback: Contextual_Shape_Support_Proc = binding.support;
	when mode == .Sweep
	{
		callback = binding.sweep_support;
	}
	status: Physics_Status = callback(binding.user_context, shape, &value, Shape_Access(registry), &result);
	if status != .Ok
	{
		return {}, status;
	}
	return result, .Ok;
}

shape_batch_dispose_value :: proc (
	batch: ^Shape_Batch, shape: rawptr, registry: ^Shape_Registry,
) -> Physics_Status
{
	if batch.dispatch == .Contextual
	{
		binding: ^Contextual_Shape_Binding = batch.contextual_metadata.binding;
		return binding.dispose(binding.user_context, shape, Shape_Access(registry), registry.pool);
	}
	return batch.metadata.dispose(shape, registry.pool);
}

// child payloads are immutable borrowed views. handles and registry bounds are
// checked. the caller must not retain a view across callback return or mutation
Shape_Access_Value :: struct
{
	data: rawptr,
	size, alignment: int,
	type_id: i32,
	batch_type: Shape_Batch_Type,
}

shape_access_resolve :: proc "contextless" (
	access: Shape_Access, index: Typed_Index,
) -> (Shape_Access_Value, Physics_Status)
{
	shape: rawptr;
	batch: ^Shape_Batch;
	status: Physics_Status;
	shape, batch, status = shape_registry_resolve((^Shape_Registry)(rawptr(access)), index);
	if status != .Ok
	{
		return {}, status;
	}
	return {shape, batch.metadata.size, batch.metadata.alignment, i32(typed_index_type(index)), batch.metadata.batch_type}, .Ok;
}

shape_access_compute_bounds :: proc "contextless" (
	access: Shape_Access, index: Typed_Index, orientation: util.Quaternion,
) -> (Shape_Bounds, Physics_Status)
{
	return shape_registry_compute_bounds((^Shape_Registry)(rawptr(access)), index, orientation);
}

shape_access_compute_inertia :: proc "contextless" (
	access: Shape_Access, index: Typed_Index, mass: f32,
) -> (Body_Inertia, Physics_Status)
{
	return shape_registry_compute_inertia((^Shape_Registry)(rawptr(access)), index, mass);
}

shape_access_ray_test :: proc "contextless" (
	access: Shape_Access, index: Typed_Index, pose: Rigid_Pose, ray: Tree_Ray,
) -> (Shape_Ray_Hit, Physics_Status)
{
	return shape_registry_ray_test((^Shape_Registry)(rawptr(access)), index, pose, ray);
}

shape_access_support :: proc "contextless" (
	access: Shape_Access, index: Typed_Index, direction: util.Vector3,
) -> (util.Vector3, Physics_Status)
{
	return shape_registry_support((^Shape_Registry)(rawptr(access)), index, direction);
}

shape_access_sweep_support :: proc "contextless" (
	access: Shape_Access, index: Typed_Index, direction: util.Vector3,
) -> (util.Vector3, Physics_Status)
{
	registry: ^Shape_Registry = (^Shape_Registry)(rawptr(access));
	shape: rawptr;
	batch: ^Shape_Batch;
	status: Physics_Status;
	shape, batch, status = shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return {}, status;
	}
	return shape_batch_support(batch, shape, direction, registry, .Sweep);
}

// a callback-local view resolves its binding once before iterative support work.
// this is not stored on shapes or bodies and does not alter native view layout
Contextual_Collision_Shape_View :: struct
{
	shape: rawptr,
	binding: ^Contextual_Shape_Binding,
	pose: Rigid_Pose,
}

collision_contextual_shape_view :: #force_inline proc "contextless" (
	view: Collision_Shape_View,
) -> Contextual_Collision_Shape_View
{
	return {view.shape, view.batch.contextual_metadata.binding, view.pose};
}

collision_support_world_contextual :: proc "contextless" (
	view: Contextual_Collision_Shape_View, direction: util.Vector3, shapes: ^Shape_Registry,
) -> (util.Vector3, Physics_Status)
{
	if util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	local_direction: util.Vector3 = util.quaternion_transform(direction, util.quaternion_conjugate(view.pose.orientation));
	local_support: util.Vector3;
	status: Physics_Status;
	local_support, status = shape_contextual_support(view.binding, view.shape, local_direction, shapes, .Collision);
	if status != .Ok
	{
		return {}, status;
	}
	return rigid_pose_transform(local_support, view.pose), .Ok;
}

// compile-time view selection leaves native support loops with the original
// direct metadata callback. there is no per-iteration tag or binding-table read
depth_refiner_support_resolved :: proc "contextless" (
	a: $A, b: $B, direction: util.Vector3, shapes: ^Shape_Registry,
) -> (Depth_Refiner_Vertex, Physics_Status)
{
	support_a, support_b: util.Vector3;
	status_a, status_b: Physics_Status;
	when A == Contextual_Collision_Shape_View
	{
		support_a, status_a = collision_support_world_contextual(a, direction, shapes);
	}
	else
	{
		#assert(A == Collision_Shape_View);
		support_a, status_a = collision_support_world_native(a, direction, shapes);
	}
	if status_a != .Ok
	{
		return {}, status_a;
	}
	when B == Contextual_Collision_Shape_View
	{
		support_b, status_b = collision_support_world_contextual(b, util.vector3_negate(direction), shapes);
	}
	else
	{
		#assert(B == Collision_Shape_View);
		support_b, status_b = collision_support_world_native(b, util.vector3_negate(direction), shapes);
	}
	if status_b != .Ok
	{
		return {}, status_b;
	}
	return {support=util.vector3_subtract(support_a, support_b), support_a=support_a}, .Ok;
}
