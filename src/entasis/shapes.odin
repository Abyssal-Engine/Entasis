package entasis

import "core:math"
import physics "entasis:entasis_physics"

// stable built-in primitive shape aliases. their storage is the exact low-level
// representation used by the runtime
Sphere   :: physics.Sphere;
// Capsule zero-copy built-in Y-axis capsule shape
Capsule  :: physics.Capsule;
// Box zero-copy built-in box shape using half extents internally
Box      :: physics.Box;
// Triangle zero-copy clockwise one-sided triangle shape
Triangle :: physics.Triangle;
// Cylinder zero-copy built-in Y-axis cylinder shape
Cylinder :: physics.Cylinder;

// Shape_Type_ID is the stable numeric identity used by the registry. ordinary
// built-in calls infer it at compile time. explicit IDs are reserved for the
// advanced shape_add_typed path and custom-shape registration
Shape_Type_ID :: distinct i32;

// SHAPE_TYPE_INVALID invalid explicit shape type sentinel
SHAPE_TYPE_INVALID     :: Shape_Type_ID(-1);
// SHAPE_TYPE_SPHERE built-in sphere registry type ID
SHAPE_TYPE_SPHERE      :: Shape_Type_ID(physics.SPHERE_TYPE_ID);
// SHAPE_TYPE_CAPSULE built-in capsule registry type ID
SHAPE_TYPE_CAPSULE     :: Shape_Type_ID(physics.CAPSULE_TYPE_ID);
// SHAPE_TYPE_BOX built-in box registry type ID
SHAPE_TYPE_BOX         :: Shape_Type_ID(physics.BOX_TYPE_ID);
// SHAPE_TYPE_TRIANGLE built-in triangle registry type ID
SHAPE_TYPE_TRIANGLE    :: Shape_Type_ID(physics.TRIANGLE_TYPE_ID);
// SHAPE_TYPE_CYLINDER built-in cylinder registry type ID
SHAPE_TYPE_CYLINDER    :: Shape_Type_ID(physics.CYLINDER_TYPE_ID);
// SHAPE_TYPE_CONVEX_HULL is the built-in convex hull registry type ID
SHAPE_TYPE_CONVEX_HULL :: Shape_Type_ID(physics.CONVEX_HULL_TYPE_ID);
// SHAPE_TYPE_COMPOUND is the built-in compound registry type ID
SHAPE_TYPE_COMPOUND    :: Shape_Type_ID(physics.COMPOUND_TYPE_ID);
// SHAPE_TYPE_BIG_COMPOUND is the built-in big compound registry type ID
SHAPE_TYPE_BIG_COMPOUND :: Shape_Type_ID(physics.BIG_COMPOUND_TYPE_ID);
// SHAPE_TYPE_MESH is the built-in mesh registry type ID
SHAPE_TYPE_MESH        :: Shape_Type_ID(physics.MESH_TYPE_ID);

// sphere constructs a sphere from its radius. validation occurs in
// shape_validate, shape_add, and shape_inertia
sphere :: #force_inline proc "contextless" (radius: f32) -> Sphere
{
	return {radius=radius};
}

// box constructs a box from full width, height, and depth
box :: #force_inline proc "contextless" (width, height, depth: f32) -> Box
{
	return {
		half_width=width * 0.5,
		half_height=height * 0.5,
		half_length=depth * 0.5,
	};
}

// box_half_extents constructs a box from explicit half extents
box_half_extents :: #force_inline proc "contextless" (
	half_width, half_height, half_depth: f32,
) -> Box
{
	return {
		half_width=half_width,
		half_height=half_height,
		half_length=half_depth,
	};
}

// capsule constructs a Y-axis capsule from radius and full cylindrical length
capsule :: #force_inline proc "contextless" (radius, length: f32) -> Capsule
{
	return {radius=radius, half_length=length * 0.5};
}

// capsule_half_length constructs a Y-axis capsule from radius and half length
capsule_half_length :: #force_inline proc "contextless" (
	radius, half_length: f32,
) -> Capsule
{
	return {radius=radius, half_length=half_length};
}

// cylinder constructs a Y-axis cylinder from radius and full length
cylinder :: #force_inline proc "contextless" (radius, length: f32) -> Cylinder
{
	return {radius=radius, half_length=length * 0.5};
}

// cylinder_half_length constructs a Y-axis cylinder from radius and half length
cylinder_half_length :: #force_inline proc "contextless" (
	radius, half_length: f32,
) -> Cylinder
{
	return {radius=radius, half_length=half_length};
}

// triangle constructs a clockwise one-sided triangle
triangle :: #force_inline proc "contextless" (a, b, c: Vector3) -> Triangle
{
	return {a=a, b=b, c=c};
}

// shape_type_id maps supported built-in shape types to compile-time constants.
// unsupported types return SHAPE_TYPE_INVALID and are rejected by shape_add
shape_type_id :: #force_inline proc "contextless" ($T: typeid) -> Shape_Type_ID
{
	when T == Sphere
	{
		return SHAPE_TYPE_SPHERE;
	}
	else when T == Capsule
	{
		return SHAPE_TYPE_CAPSULE;
	}
	else when T == Box
	{
		return SHAPE_TYPE_BOX;
	}
	else when T == Triangle
	{
		return SHAPE_TYPE_TRIANGLE;
	}
	else when T == Cylinder
	{
		return SHAPE_TYPE_CYLINDER;
	}
	else when T == physics.Convex_Hull
	{
		return SHAPE_TYPE_CONVEX_HULL;
	}
	else when T == physics.Compound
	{
		return SHAPE_TYPE_COMPOUND;
	}
	else when T == physics.Big_Compound
	{
		return SHAPE_TYPE_BIG_COMPOUND;
	}
	else when T == physics.Mesh
	{
		return SHAPE_TYPE_MESH;
	}
	return SHAPE_TYPE_INVALID;
}

@(private)
shape_scalar_is_finite :: #force_inline proc "contextless" (value: f32) -> bool
{
	return !math.is_nan(value) && !math.is_inf(value, 0);
}

@(private)
shape_vector_is_finite :: #force_inline proc "contextless" (value: Vector3) -> bool
{
	return shape_scalar_is_finite(value.x) &&
		shape_scalar_is_finite(value.y) &&
		shape_scalar_is_finite(value.z);
}

// shape_validate validates public built-in primitive descriptions without
// allocation. container and custom shapes use their dedicated advanced paths
shape_validate :: #force_inline proc "contextless" (shape: $T) -> Status
{
	when T == Sphere
	{
		if !shape_scalar_is_finite(shape.radius)
		{
			return .Invalid_Description;
		}
		return physics.sphere_validate(shape);
	}
	else when T == Capsule
	{
		if !shape_scalar_is_finite(shape.radius) ||
			!shape_scalar_is_finite(shape.half_length)
		{
			return .Invalid_Description;
		}
		return physics.capsule_validate(shape);
	}
	else when T == Box
	{
		if !shape_scalar_is_finite(shape.half_width) ||
			!shape_scalar_is_finite(shape.half_height) ||
			!shape_scalar_is_finite(shape.half_length)
		{
			return .Invalid_Description;
		}
		return physics.box_validate(shape);
	}
	else when T == Triangle
	{
		if !shape_vector_is_finite(shape.a) ||
			!shape_vector_is_finite(shape.b) ||
			!shape_vector_is_finite(shape.c)
		{
			return .Invalid_Description;
		}
		return physics.triangle_validate(shape);
	}
	else when T == Cylinder
	{
		if !shape_scalar_is_finite(shape.radius) ||
			!shape_scalar_is_finite(shape.half_length)
		{
			return .Invalid_Description;
		}
		return physics.cylinder_validate(shape);
	}
	return .Invalid_Argument;
}

@(private)
shape_registry_from_world :: #force_inline proc "contextless" (
	world: ^World,
) -> (^physics.Shape_Registry, Status)
{
	simulation := world_simulation(world);
	if simulation == nil || simulation.state != .Ready
	{
		return nil, .Disposed;
	}
	registry := physics.simulation_shape_registry(simulation);
	if registry == nil
	{
		return nil, .Disposed;
	}
	return registry, .Ok;
}

// shape_add validates and registers one built-in primitive. type selection is a
// compile-time constant and adds no runtime type lookup
// ownership: owner thread only while the world is idle
shape_add :: proc (
	world: ^World, shape: $T,
) -> (Shape_Handle, Status)
{
	type_id := shape_type_id(T);
	when T == Sphere || T == Capsule || T == Box || T == Triangle || T == Cylinder
	{
		validation := shape_validate(shape);
		if validation != .Ok
		{
			return shape_handle_invalid(), validation;
		}
	}
	else
	{
		return shape_handle_invalid(), .Invalid_Argument;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return shape_handle_invalid(), registry_status;
	}
	stored_shape := shape;
	return physics.shape_registry_add(registry, int(type_id), &stored_shape);
}

// shape_add_typed is the advanced explicit-ID path for cooked container shapes
// and registered custom shapes. the caller is responsible for matching the
// type ID, value type, and ownership contract of the registered shape type.
// ownership of internal buffers transfers to the registry after success
shape_add_typed :: proc (
	world: ^World, type_id: Shape_Type_ID, shape: ^$T,
) -> (Shape_Handle, Status)
{
	if shape == nil || type_id == SHAPE_TYPE_INVALID
	{
		return shape_handle_invalid(), .Invalid_Argument;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return shape_handle_invalid(), registry_status;
	}
	return physics.shape_registry_add(registry, int(type_id), shape);
}

// shape_remove removes an unreferenced shape. bodies, statics, and compound
// children retain shapes automatically. referenced shapes return Shape_In_Use
// ownership: owner thread only while the world is idle
shape_remove :: proc (world: ^World, handle: Shape_Handle) -> Status
{
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return registry_status;
	}
	return physics.shape_registry_remove(registry, handle);
}
