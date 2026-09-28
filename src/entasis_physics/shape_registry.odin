// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "base:intrinsics"

MAXIMUM_SHAPE_TYPE_COUNT :: 128;
Shape_Batch_Type :: enum u8
{
	Convex,
	Compound,
	Homogeneous_Compound,
}

Shape_Batch_State :: enum u8
{
	Unregistered,
	Registered,
}

Shape_Bounds_Proc :: #type proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status);
Shape_Inertia_Proc :: #type proc "contextless" (
	shape: rawptr, registry: ^Shape_Registry, mass: f32,
) -> (Body_Inertia, Physics_Status);
Shape_Ray_Proc :: #type proc "contextless" (
	shape: rawptr, pose: Rigid_Pose, ray: Tree_Ray, registry: ^Shape_Registry,
) -> (Shape_Ray_Hit, Physics_Status);
Shape_Support_Proc :: #type proc "contextless" (
	shape: rawptr, direction: util.Vector3, registry: ^Shape_Registry,
) -> (util.Vector3, Physics_Status);
Shape_Dispose_Proc :: #type proc (
	shape: rawptr, pool: ^util.Buffer_Pool,
) -> Physics_Status;
Shape_Type_Registration :: struct
{
	size:          int,
	alignment:     int,
	batch_type:    Shape_Batch_Type,
	bounds:        Shape_Bounds_Proc,
	inertia:       Shape_Inertia_Proc,
	ray:           Shape_Ray_Proc,
	support:       Shape_Support_Proc,
	sweep_support: Shape_Support_Proc,
	dispose:       Shape_Dispose_Proc,
}

Shape_Batch :: struct
{
	data:       util.Buffer(u8),
	references: util.Buffer(i32),
	ids:        util.Id_Pool,
	slot_count: int,
	active_count: int,
	stride:     int,
	using callbacks: struct #raw_union
	{
		metadata: Shape_Type_Registration,
		contextual_metadata: Contextual_Shape_Type_Metadata,
	},
	state:      Shape_Batch_State,
	dispatch:   Shape_Callback_Dispatch,
}

Shape_Registry :: struct
{
	batches:               [MAXIMUM_SHAPE_TYPE_COUNT]Shape_Batch,
	registered_type_count: int,
	default_capacity:      int,
	pool:                  ^util.Buffer_Pool,
	state:                 Body_Set_State,
	hierarchy:             ^Shape_Hierarchy_Cache,
}

shape_batch_capacity :: proc "contextless" (batch: ^Shape_Batch) -> int
{
	if batch == nil || batch.state != .Registered || batch.stride <= 0 ||
	batch.data.memory == nil || batch.references.memory == nil
	{
		return 0;
	}
	return min(
		int(batch.references.length),
		int(batch.data.length) / batch.stride,
	);
}

shape_no_dispose :: proc (shape: rawptr, pool: ^util.Buffer_Pool) -> Physics_Status
{
	_ = shape;
	_ = pool;
	return .Ok;
}

shape_stride :: proc "contextless" (size, alignment: int) -> int
{
	return (size + alignment - 1) & ~(alignment - 1);
}

shape_registry_register_type :: proc (
	registry: ^Shape_Registry, type_id: int, registration: Shape_Type_Registration,
) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated || type_id < 0 || type_id >= MAXIMUM_SHAPE_TYPE_COUNT ||
	type_id != registry.registered_type_count || registration.size <= 0 ||
	registration.alignment <= 0 || registration.alignment & (registration.alignment - 1) != 0 ||
	registration.bounds == nil || registration.inertia == nil || registration.ray == nil ||
	registration.support == nil || registration.sweep_support == nil || registration.dispose == nil
	{
		return .Invalid_Argument;
	}
	return shape_registry_register_batch(registry, type_id, {
			stride=shape_stride(registration.size, registration.alignment),
			metadata=registration,
			state=.Registered,
	});
}

// storage initialization is shared by native and contextual registrations only.
// no callback is installed until its real registration has passed validation
@(private)
shape_registry_register_batch :: proc (
	registry: ^Shape_Registry, type_id: int, batch: Shape_Batch,
) -> Physics_Status
{
	registry.batches[type_id] = batch;
	id_status := util.id_pool_initialize(&registry.batches[type_id].ids, registry.default_capacity, registry.pool);
	if id_status != .Ok
	{
		registry.batches[type_id] = {};
		return physics_memory_status(id_status);
	}
	registry.registered_type_count += 1;
	return .Ok;
}

sphere_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Sphere)(shape);
	status := sphere_validate(typed^);
	if status != .Ok
	{
		return {}, status;
	}
	return sphere_bounds(typed^, orientation), .Ok;
}

sphere_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return sphere_inertia((^Sphere)(shape)^, mass);
}

sphere_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return sphere_ray_test((^Sphere)(shape)^, pose, ray);
}

sphere_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return sphere_support((^Sphere)(shape)^, direction);
}

capsule_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Capsule)(shape);
	status := capsule_validate(typed^);
	if status != .Ok
	{
		return {}, status;
	}
	return capsule_bounds(typed^, orientation), .Ok;
}

capsule_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return capsule_inertia((^Capsule)(shape)^, mass);
}

capsule_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return capsule_ray_test((^Capsule)(shape)^, pose, ray);
}

capsule_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return capsule_support((^Capsule)(shape)^, direction);
}

box_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Box)(shape);
	status := box_validate(typed^);
	if status != .Ok
	{
		return {}, status;
	}
	return box_bounds(typed^, orientation), .Ok;
}

box_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return box_inertia((^Box)(shape)^, mass);
}

box_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return box_ray_test((^Box)(shape)^, pose, ray);
}

box_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return box_support((^Box)(shape)^, direction);
}

triangle_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Triangle)(shape);
	status := triangle_validate(typed^);
	if status != .Ok
	{
		return {}, status;
	}
	return triangle_bounds(typed^, orientation), .Ok;
}

triangle_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return triangle_inertia((^Triangle)(shape)^, mass);
}

triangle_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return triangle_ray_test((^Triangle)(shape)^, pose, ray);
}

triangle_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return triangle_support((^Triangle)(shape)^, direction);
}

cylinder_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Cylinder)(shape);
	status := cylinder_validate(typed^);
	if status != .Ok
	{
		return {}, status;
	}
	return cylinder_bounds(typed^, orientation), .Ok;
}

cylinder_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return cylinder_inertia((^Cylinder)(shape)^, mass);
}

cylinder_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return cylinder_ray_test((^Cylinder)(shape)^, pose, ray);
}

cylinder_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return cylinder_support((^Cylinder)(shape)^, direction);
}

convex_hull_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	typed := (^Convex_Hull)(shape);
	status := convex_hull_validate(typed);
	if status != .Ok
	{
		return {}, status;
	}
	return convex_hull_bounds(typed, orientation), .Ok;
}

convex_hull_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return convex_hull_inertia((^Convex_Hull)(shape), mass);
}

convex_hull_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return convex_hull_ray_test((^Convex_Hull)(shape), pose, ray);
}

convex_hull_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	return convex_hull_support((^Convex_Hull)(shape), direction);
}

convex_hull_dispose_proc :: proc (shape: rawptr, pool: ^util.Buffer_Pool) -> Physics_Status
{
	return convex_hull_dispose((^Convex_Hull)(shape), pool);
}

compound_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	return compound_bounds((^Compound)(shape), orientation, registry);
}

compound_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_, _, _ = shape, registry, mass;
	return {}, .Invalid_Argument;
}

compound_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	return compound_ray_test((^Compound)(shape).children, pose, ray, registry);
}

compound_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = shape;
	_ = direction;
	_ = registry;
	return {}, .Invalid_Argument;
}

compound_dispose_proc :: proc (shape: rawptr, pool: ^util.Buffer_Pool) -> Physics_Status
{
	return compound_dispose((^Compound)(shape), pool);
}

big_compound_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	return big_compound_bounds((^Big_Compound)(shape), orientation), .Ok;
}

big_compound_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_, _, _ = shape, registry, mass;
	return {}, .Invalid_Argument;
}

big_compound_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	return compound_ray_test((^Big_Compound)(shape).children, pose, ray, registry);
}

big_compound_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = shape;
	_ = direction;
	_ = registry;
	return {}, .Invalid_Argument;
}

big_compound_dispose_proc :: proc (shape: rawptr, pool: ^util.Buffer_Pool) -> Physics_Status
{
	return big_compound_dispose((^Big_Compound)(shape), pool);
}

mesh_bounds_proc :: proc "contextless" (
	shape: rawptr, orientation: util.Quaternion, registry: ^Shape_Registry,
) -> (Shape_Bounds, Physics_Status)
{
	_ = registry;
	mesh := (^Mesh)(shape);
	if mesh.triangles.memory == nil || mesh.triangles.length <= 0
	{
		return {}, .Invalid_Description;
	}
	return mesh_bounds(mesh, orientation), .Ok;
}

mesh_inertia_proc :: proc "contextless" (
	shape: rawptr,
	registry: ^Shape_Registry,
	mass: f32
) -> (Body_Inertia, Physics_Status)
{
	_ = registry;
	return mesh_closed_inertia((^Mesh)(shape), mass);
}

mesh_ray_proc :: proc "contextless" (
	shape: rawptr,
	pose: Rigid_Pose,
	ray: Tree_Ray,
	registry: ^Shape_Registry
) -> (Shape_Ray_Hit, Physics_Status)
{
	_ = registry;
	return mesh_ray_test((^Mesh)(shape), pose, ray);
}

mesh_support_proc :: proc "contextless" (
	shape: rawptr,
	direction: util.Vector3,
	registry: ^Shape_Registry
) -> (util.Vector3, Physics_Status)
{
	_ = registry;
	mesh := (^Mesh)(shape);
	if mesh.triangles.memory == nil || mesh.triangles.length <= 0 || util.vector3_length_squared(direction) <= 1e-20
	{
		return {}, .Invalid_Argument;
	}
	best := util.vector3_multiply(mesh.triangles.memory[0].a, mesh.scale);
	best_dot := util.vector3_dot(best, direction);
	for index in 0 ..< mesh.triangles.length
	{
		triangle := mesh.triangles.memory[index];
		vertices := [3]util.Vector3{
			util.vector3_multiply(triangle.a, mesh.scale),
			util.vector3_multiply(triangle.b, mesh.scale),
			util.vector3_multiply(triangle.c, mesh.scale),
		};
		for vertex in vertices
		{
			candidate_dot := util.vector3_dot(vertex, direction);
			if candidate_dot > best_dot
			{
				best = vertex;
				best_dot = candidate_dot;
			}
		}
	}
	return best, .Ok;
}

mesh_dispose_proc :: proc (shape: rawptr, pool: ^util.Buffer_Pool) -> Physics_Status
{
	return mesh_dispose((^Mesh)(shape), pool);
}

shape_registry_initialize :: proc (
	registry: ^Shape_Registry, default_capacity: int, pool: ^util.Buffer_Pool,
) -> Physics_Status
{
	if registry == nil || pool == nil || default_capacity <= 0
	{
		return .Invalid_Argument;
	}
	registry^ = {default_capacity=default_capacity, pool=pool, state=.Allocated};
	registrations := [BUILT_IN_SHAPE_TYPE_COUNT]Shape_Type_Registration{
		{
			size_of(Sphere),
			align_of(Sphere),
			.Convex,
			sphere_bounds_proc,
			sphere_inertia_proc,
			sphere_ray_proc,
			sphere_support_proc,
			sphere_support_proc,
			shape_no_dispose,
		},
		{
			size_of(Capsule),
			align_of(Capsule),
			.Convex,
			capsule_bounds_proc,
			capsule_inertia_proc,
			capsule_ray_proc,
			capsule_support_proc,
			capsule_support_proc,
			shape_no_dispose,
		},
		{
			size_of(Box),
			align_of(Box),
			.Convex,
			box_bounds_proc,
			box_inertia_proc,
			box_ray_proc,
			box_support_proc,
			box_support_proc,
			shape_no_dispose,
		},
		{
			size_of(Triangle),
			align_of(Triangle),
			.Convex,
			triangle_bounds_proc,
			triangle_inertia_proc,
			triangle_ray_proc,
			triangle_support_proc,
			triangle_support_proc,
			shape_no_dispose,
		},
		{
			size_of(Cylinder),
			align_of(Cylinder),
			.Convex,
			cylinder_bounds_proc,
			cylinder_inertia_proc,
			cylinder_ray_proc,
			cylinder_support_proc,
			cylinder_support_proc,
			shape_no_dispose,
		},
		{
			size_of(Convex_Hull),
			align_of(Convex_Hull),
			.Convex,
			convex_hull_bounds_proc,
			convex_hull_inertia_proc,
			convex_hull_ray_proc,
			convex_hull_support_proc,
			convex_hull_support_proc,
			convex_hull_dispose_proc,
		},
		{
			size_of(Compound),
			align_of(Compound),
			.Compound,
			compound_bounds_proc,
			compound_inertia_proc,
			compound_ray_proc,
			compound_support_proc,
			compound_support_proc,
			compound_dispose_proc,
		},
		{
			size_of(Big_Compound),
			align_of(Big_Compound),
			.Compound,
			big_compound_bounds_proc,
			big_compound_inertia_proc,
			big_compound_ray_proc,
			big_compound_support_proc,
			big_compound_support_proc,
			big_compound_dispose_proc,
		},
		{
			size_of(Mesh),
			align_of(Mesh),
			.Homogeneous_Compound,
			mesh_bounds_proc,
			mesh_inertia_proc,
			mesh_ray_proc,
			mesh_support_proc,
			mesh_support_proc,
			mesh_dispose_proc,
		},
	};
	for type_id in 0 ..< BUILT_IN_SHAPE_TYPE_COUNT
	{
		status := shape_registry_register_type(registry, type_id, registrations[type_id]);
		if status != .Ok
		{
			for registered_index in 0 ..< registry.registered_type_count
			{
				_ = util.id_pool_dispose(&registry.batches[registered_index].ids, pool);
			}
			registry^ = {};
			return status;
		}
	}
	return .Ok;
}

shape_registry_register_custom :: proc (
	registry: ^Shape_Registry, registration: Shape_Type_Registration,
) -> (i32, Physics_Status)
{
	if registry == nil || registry.state != .Allocated
	{
		return -1, .Disposed;
	}
	type_id := registry.registered_type_count;
	status := shape_registry_register_type(registry, type_id, registration);
	if status != .Ok
	{
		return -1, status;
	}
	return i32(type_id), .Ok;
}

shape_batch_ensure_capacity :: proc (
	registry: ^Shape_Registry, type_id, capacity: int,
) -> Physics_Status
{
	batch := &registry.batches[type_id];
	current_capacity := shape_batch_capacity(batch);
	if capacity <= current_capacity
	{
		return .Ok;
	}
	target_capacity := max(
		max(capacity, registry.default_capacity),
		max(current_capacity * 2, 1),
	);
	if batch.stride <= 0 || target_capacity > max(int) / batch.stride
	{
		return .Capacity_Missing;
	}
	byte_count := target_capacity * batch.stride;
	old_byte_count := batch.slot_count * batch.stride;
	new_data, data_status := util.buffer_pool_take_at_least(registry.pool, u8, byte_count);
	if data_status != .Ok
	{
		return physics_memory_status(data_status);
	}
	storage_capacity := int(new_data.length) / batch.stride;
	if storage_capacity < target_capacity
	{
		physics_return_buffer(registry.pool, &new_data);
		return .Capacity_Missing;
	}
	new_references, reference_status := util.buffer_pool_take_at_least(
		registry.pool, i32, storage_capacity,
	);
	if reference_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(reference_status);
	}
	new_available_ids, ids_status := util.buffer_pool_take_at_least(
		registry.pool, i32, storage_capacity,
	);
	if ids_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_references);
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(ids_status);
	}
	_ = util.buffer_clear(new_data, 0, int(new_data.length));
	for index in 0 ..< new_references.length
	{
		new_references.memory[index] = -1;
	}
	copy_status := util.buffer_copy(util.buffer_view(batch.data), 0, util.buffer_view(new_data), 0, old_byte_count);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(batch.references), 0, util.buffer_view(new_references), 0, batch.slot_count,
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(batch.ids.available_ids), 0, util.buffer_view(new_available_ids), 0,
			batch.ids.available_id_count,
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_available_ids);
		physics_return_buffer(registry.pool, &new_references);
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(registry.pool, &batch.ids.available_ids);
	physics_return_buffer(registry.pool, &batch.references);
	physics_return_buffer(registry.pool, &batch.data);
	batch.data = new_data;
	batch.references = new_references;
	batch.ids.available_ids = new_available_ids;
	return .Ok;
}

shape_registry_ensure_capacity :: proc (
	registry: ^Shape_Registry, capacity_per_type: int,
) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated ||
	capacity_per_type <= 0
	{
		return .Invalid_Argument;
	}
	for type_id in 0 ..< registry.registered_type_count
	{
		status := shape_batch_ensure_capacity(
			registry, type_id, capacity_per_type,
		);
		if status != .Ok
		{
			return status;
		}
	}
	registry.default_capacity = max(
		registry.default_capacity, capacity_per_type,
	);
	return .Ok;
}

shape_batch_resize :: proc (
	registry: ^Shape_Registry, type_id, capacity: int,
) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated || type_id < 0 ||
	type_id >= registry.registered_type_count || capacity <= 0
	{
		return .Invalid_Argument;
	}
	batch := &registry.batches[type_id];
	target_capacity := max(
		capacity,
		max(batch.slot_count, batch.ids.available_id_count),
	);
	target_reference_capacity, target_status :=
	util.buffer_pool_capacity_for_count(i32, target_capacity);
	if target_status != .Ok
	{
		return physics_memory_status(target_status);
	}
	current_reference_capacity := 0;
	if batch.references.memory != nil
	{
		current_reference_capacity = int(batch.references.length);
	}
	if target_capacity <= shape_batch_capacity(batch) &&
	target_reference_capacity == current_reference_capacity
	{
		return .Ok;
	}
	if batch.stride <= 0 || target_capacity > max(int) / batch.stride
	{
		return .Capacity_Missing;
	}
	new_data, data_status := util.buffer_pool_take_at_least(
		registry.pool, u8, target_capacity * batch.stride,
	);
	if data_status != .Ok
	{
		return physics_memory_status(data_status);
	}
	storage_capacity := int(new_data.length) / batch.stride;
	if storage_capacity < target_capacity
	{
		physics_return_buffer(registry.pool, &new_data);
		return .Capacity_Missing;
	}
	new_references, references_status := util.buffer_pool_take_at_least(
		registry.pool, i32, storage_capacity,
	);
	if references_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(references_status);
	}
	new_ids, ids_status := util.buffer_pool_take_at_least(
		registry.pool, i32, storage_capacity,
	);
	if ids_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_references);
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(ids_status);
	}
	_ = util.buffer_clear(new_data, 0, int(new_data.length));
	for index in 0 ..< new_references.length
	{
		new_references.memory[index] = -1;
	}
	copy_status := util.buffer_copy(
		util.buffer_view(batch.data), 0, util.buffer_view(new_data), 0,
		batch.slot_count * batch.stride,
	);
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(batch.references), 0,
			util.buffer_view(new_references), 0, batch.slot_count,
		);
	}
	if copy_status == .Ok
	{
		copy_status = util.buffer_copy(
			util.buffer_view(batch.ids.available_ids), 0,
			util.buffer_view(new_ids), 0, batch.ids.available_id_count,
		);
	}
	if copy_status != .Ok
	{
		physics_return_buffer(registry.pool, &new_ids);
		physics_return_buffer(registry.pool, &new_references);
		physics_return_buffer(registry.pool, &new_data);
		return physics_memory_status(copy_status);
	}
	physics_return_buffer(registry.pool, &batch.ids.available_ids);
	physics_return_buffer(registry.pool, &batch.references);
	physics_return_buffer(registry.pool, &batch.data);
	batch.data = new_data;
	batch.references = new_references;
	batch.ids.available_ids = new_ids;
	return .Ok;
}

shape_registry_resize :: proc (
	registry: ^Shape_Registry, capacity_per_type: int,
) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated ||
	capacity_per_type <= 0
	{
		return .Invalid_Argument;
	}
	for type_id in 0 ..< registry.registered_type_count
	{
		status := shape_batch_resize(
			registry, type_id, capacity_per_type,
		);
		if status != .Ok
		{
			return status;
		}
	}
	registry.default_capacity = capacity_per_type;
	return .Ok;
}

shape_registry_add_raw :: proc (
	registry: ^Shape_Registry, type_id: int, shape: rawptr, shape_size: int,
) -> (Typed_Index, Physics_Status)
{
	if registry == nil || registry.state != .Allocated || type_id < 0 ||
	type_id >= registry.registered_type_count || shape == nil
	{
		return {}, .Invalid_Argument;
	}
	batch: ^Shape_Batch = &registry.batches[type_id];
	if shape_size != batch.metadata.size
	{
		return {}, .Invalid_Argument;
	}
	children: util.Buffer(Compound_Child);
	depth: int;
	if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID
	{
		children = (^Compound)(shape).children;
		if type_id == BIG_COMPOUND_TYPE_ID
		{
			children = (^Big_Compound)(shape).children;
		}
		if children.memory == nil || children.length <= 0
		{
			return {}, .Invalid_Description;
		}
		for child_index in 0 ..< int(children.length)
		{
			child: Typed_Index = children.memory[child_index].shape_index;
			child_batch: ^Shape_Batch;
			child_status: Physics_Status;
			_, child_batch, child_status = shape_registry_resolve(registry, child);
			if child_status != .Ok
			{
				return {}, .Invalid_Description;
			}
			if child_batch.metadata.batch_type != .Convex
			{
				child_type: int = int(typed_index_type(child));
				if child_type != COMPOUND_TYPE_ID && child_type != BIG_COMPOUND_TYPE_ID && child_type != MESH_TYPE_ID
				{
					return {}, .Invalid_Description;
				}
				depth = max(depth, shape_hierarchy_depth(registry, child) + 1);
			}
		}
		if depth > MAXIMUM_COMPOUND_HIERARCHY_DEPTH
		{
			return {}, .Invalid_Description;
		}
	}
	retained: int;
	shape_index: i32 = -1;
	publication: Reference_State = .Missing;
	defer
	{
		if publication == .Missing
		{
			for child_index in 0 ..< retained
			{
				_ = shape_registry_release(registry, children.memory[child_index].shape_index);
			}
			if shape_index >= 0
			{
				_ = util.id_pool_return(&batch.ids, shape_index, registry.pool);
			}
		}
	}
	for child_index in 0 ..< int(children.length)
	{
		status: Physics_Status = shape_registry_retain(registry, children.memory[child_index].shape_index);
		if status != .Ok
		{
			return {}, status;
		}
		retained += 1;
	}
	id_status: util.Memory_Status;
	shape_index, id_status = util.id_pool_take(&batch.ids);
	if id_status != .Ok
	{
		return {}, physics_memory_status(id_status);
	}
	if int(shape_index) >= shape_batch_capacity(batch)
	{
		status: Physics_Status = shape_batch_ensure_capacity(registry, type_id,
			max(int(shape_index) + 1, max(shape_batch_capacity(batch) * 2, 1)));
		if status != .Ok
		{
			return {}, status;
		}
	}
	if depth > 0
	{
		status: Physics_Status = shape_hierarchy_prepare(registry, type_id, int(shape_index));
		if status != .Ok
		{
			return {}, status;
		}
	}
	index: Typed_Index;
	typed_status: Physics_Status;
	index, typed_status = typed_index_create(type_id, int(shape_index));
	if typed_status != .Ok
	{
		return {}, typed_status;
	}
	intrinsics.mem_copy(&batch.data.memory[int(shape_index) * batch.stride], shape, batch.metadata.size);
	batch.references.memory[shape_index] = 0;
	batch.slot_count = max(batch.slot_count, int(shape_index) + 1);
	batch.active_count += 1;
	if depth > 0
	{
		registry.hierarchy.depths[type_id - COMPOUND_TYPE_ID].memory[shape_index] = u8(depth);
		registry.hierarchy.count += 1;
	}
	publication = .Present;
	return index, .Ok;
}

shape_registry_add :: proc (
	registry: ^Shape_Registry, type_id: int, shape: ^$T,
) -> (Typed_Index, Physics_Status)
{
	return shape_registry_add_raw(registry, type_id, shape, size_of(T));
}

shape_registry_resolve :: proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index,
) -> (rawptr, ^Shape_Batch, Physics_Status)
{
	if registry == nil || registry.state != .Allocated || typed_index_state(index) != .Present
	{
		return nil, nil, .Not_Found;
	}
	type_id := int(typed_index_type(index));
	shape_index := int(typed_index_index(index));
	if type_id < 0 || type_id >= registry.registered_type_count
	{
		return nil, nil, .Not_Found;
	}
	batch := &registry.batches[type_id];
	if shape_index < 0 || shape_index >= batch.slot_count || batch.references.memory[shape_index] < 0
	{
		return nil, nil, .Not_Found;
	}
	return rawptr(&batch.data.memory[shape_index * batch.stride]), batch, .Ok;
}

shape_registry_compute_bounds :: #force_inline proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index, orientation: util.Quaternion,
) -> (Shape_Bounds, Physics_Status)
{
	shape, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return {}, status;
	}
	return shape_batch_compute_bounds(batch, shape, orientation, registry);
}

shape_registry_compute_world_bounds :: proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index, pose: Rigid_Pose,
) -> (util.Bounding_Box, Physics_Status)
{
	bounds, status := shape_registry_compute_bounds(registry, index, pose.orientation);
	if status != .Ok
	{
		return {}, status;
	}
	return {min=util.vector3_add(bounds.min, pose.position), max=util.vector3_add(bounds.max, pose.position)}, .Ok;
}

shape_registry_compute_inertia :: proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index, mass: f32,
) -> (Body_Inertia, Physics_Status)
{
	shape, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return {}, status;
	}
	return shape_batch_compute_inertia(batch, shape, registry, mass);
}

shape_registry_ray_test :: #force_inline proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index, pose: Rigid_Pose, ray: Tree_Ray,
) -> (Shape_Ray_Hit, Physics_Status)
{
	shape, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return shape_ray_miss(), status;
	}
	return shape_batch_ray_test(batch, shape, pose, ray, registry);
}

shape_registry_support :: proc "contextless" (
	registry: ^Shape_Registry, index: Typed_Index, direction: util.Vector3,
) -> (util.Vector3, Physics_Status)
{
	shape, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return {}, status;
	}
	return shape_batch_support(batch, shape, direction, registry, .Collision);
}

shape_registry_retain :: proc "contextless" (registry: ^Shape_Registry, index: Typed_Index) -> Physics_Status
{
	_, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return status;
	}
	shape_index := int(typed_index_index(index));
	if batch.references.memory[shape_index] == max(i32)
	{
		return .Capacity_Missing;
	}
	batch.references.memory[shape_index] += 1;
	return .Ok;
}

shape_registry_release :: proc "contextless" (registry: ^Shape_Registry, index: Typed_Index) -> Physics_Status
{
	_, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return status;
	}
	shape_index := int(typed_index_index(index));
	if batch.references.memory[shape_index] <= 0
	{
		return .Invalid_Argument;
	}
	batch.references.memory[shape_index] -= 1;
	return .Ok;
}

shape_registry_remove :: proc (registry: ^Shape_Registry, index: Typed_Index) -> Physics_Status
{
	shape, batch, status := shape_registry_resolve(registry, index);
	if status != .Ok
	{
		return status;
	}
	shape_index := int(typed_index_index(index));
	if batch.references.memory[shape_index] > 0
	{
		return .Shape_In_Use;
	}
	type_id := int(typed_index_type(index));
	if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID
	{
		children := (^Compound)(shape).children;
		if type_id == BIG_COMPOUND_TYPE_ID
		{
			children = (^Big_Compound)(shape).children;
		}
		for child_index in 0 ..< children.length
		{
			_ = shape_registry_release(registry, children.memory[child_index].shape_index);
		}
	}
	dispose_status: Physics_Status = shape_batch_dispose_value(batch, shape, registry);
	if dispose_status != .Ok
	{
		if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID
		{
			children := (^Compound)(shape).children;
			if type_id == BIG_COMPOUND_TYPE_ID
			{
				children = (^Big_Compound)(shape).children;
			}
			for child_index in 0 ..< children.length
			{
				_ = shape_registry_retain(registry, children.memory[child_index].shape_index);
			}
		}
		return dispose_status;
	}
	shape_hierarchy_retire(registry, index);
	intrinsics.mem_zero(&batch.data.memory[shape_index * batch.stride], batch.stride);
	batch.references.memory[shape_index] = -1;
	return_status := util.id_pool_return(&batch.ids, i32(shape_index), registry.pool);
	if return_status != .Ok
	{
		return physics_memory_status(return_status);
	}
	batch.active_count -= 1;
	return .Ok;
}

// built-in compounds may retain any registered convex type, including custom
// types whose IDs follow the compound IDs. release those parents before their
// children. a reverse numeric type order alone cannot satisfy that lifetime
@(private)
shape_registry_teardown_type :: #force_inline proc "contextless" (
	registered_count, position: int,
) -> int
{
	parent_count: int = int(registered_count > COMPOUND_TYPE_ID) + int(registered_count > BIG_COMPOUND_TYPE_ID);
	if position < parent_count
	{
		return min(registered_count - 1, BIG_COMPOUND_TYPE_ID) - position;
	}
	type_id: int = registered_count - 1 - (position - parent_count);
	if type_id <= BIG_COMPOUND_TYPE_ID
	{
		type_id -= parent_count;
	}
	return type_id;
}

shape_registry_clear :: proc (registry: ^Shape_Registry) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated
	{
		return .Disposed;
	}
	if registry.hierarchy != nil && registry.hierarchy.count > 0
	{
		return shape_registry_clear_hierarchy(registry);
	}
	for reverse_index in 0 ..< registry.registered_type_count
	{
		type_id: int = shape_registry_teardown_type(registry.registered_type_count, reverse_index);
		batch := &registry.batches[type_id];
		for shape_index in 0 ..< batch.slot_count
		{
			if batch.references.memory[shape_index] < 0
			{
				continue;
			}
			index, index_status := typed_index_create(type_id, shape_index);
			if index_status != .Ok
			{
				return index_status;
			}
			status := shape_registry_remove(registry, index);
			if status != .Ok
			{
				return status;
			}
		}
		util.id_pool_clear(&batch.ids);
		batch.slot_count = 0;
		batch.active_count = 0;
	}
	return .Ok;
}

shape_registry_dispose :: proc (registry: ^Shape_Registry) -> Physics_Status
{
	if registry == nil || registry.state != .Allocated || registry.pool == nil
	{
		return .Disposed;
	}
	if registry.hierarchy != nil && registry.hierarchy.count > 0
	{
		status: Physics_Status = shape_registry_clear_hierarchy(registry);
		if status != .Ok
		{
			return status;
		}
	}
	for reverse_index in 0 ..< registry.registered_type_count
	{
		type_id: int = shape_registry_teardown_type(registry.registered_type_count, reverse_index);
		batch := &registry.batches[type_id];
		for index in 0 ..< batch.slot_count
		{
			if batch.references.memory[index] < 0
			{
				continue;
			}
			shape := rawptr(&batch.data.memory[index * batch.stride]);
			if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID
			{
				children := (^Compound)(shape).children;
				if type_id == BIG_COMPOUND_TYPE_ID
				{
					children = (^Big_Compound)(shape).children;
				}
				for child_index in 0 ..< children.length
				{
					_ = shape_registry_release(registry, children.memory[child_index].shape_index);
				}
			}
			_ = shape_batch_dispose_value(batch, shape, registry);
		}
		_ = util.id_pool_dispose(&batch.ids, registry.pool);
		physics_return_buffer(registry.pool, &batch.references);
		physics_return_buffer(registry.pool, &batch.data);
		batch^ = {};
	}
	shape_hierarchy_dispose(registry);
	registry^ = {};
	return .Ok;
}
