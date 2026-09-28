package entasis

import "core:math"
import physics "entasis:entasis_physics"

@(private)
shape_mass_is_valid :: #force_inline proc "contextless" (mass: f32) -> bool
{
	return mass > 0 && !math.is_nan(mass) && !math.is_inf(mass, 0);
}

// shape_inertia computes inertia directly from a built-in primitive value.
// it performs no registry lookup and allocates nothing
shape_inertia :: #force_inline proc "contextless" (
	shape: $T, mass: f32,
) -> (Body_Inertia, Status)
{
	validation := shape_validate(shape);
	if validation != .Ok
	{
		return {}, validation;
	}
	if !shape_mass_is_valid(mass)
	{
		return {}, .Invalid_Argument;
	}
	when T == Sphere
	{
		return physics.sphere_inertia(shape, mass);
	}
	else when T == Capsule
	{
		return physics.capsule_inertia(shape, mass);
	}
	else when T == Box
	{
		return physics.box_inertia(shape, mass);
	}
	else when T == Triangle
	{
		return physics.triangle_inertia(shape, mass);
	}
	else when T == Cylinder
	{
		return physics.cylinder_inertia(shape, mass);
	}
	return {}, .Invalid_Argument;
}

// shape_registered_inertia computes inertia through the world's shape registry.
// this advanced path supports registered hulls, closed meshes, primitives, and
// custom shapes whose registration provides an inertia implementation. Compound
// mass properties require child masses and use compound_inertia_weighted or the
// caller-owned Compound_Builder API
shape_registered_inertia :: proc "contextless" (
	world: ^World, handle: Shape_Handle, mass: f32,
) -> (Body_Inertia, Status)
{
	if !shape_mass_is_valid(mass)
	{
		return {}, .Invalid_Argument;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return {}, registry_status;
	}
	return physics.shape_registry_compute_inertia(registry, handle, mass);
}
