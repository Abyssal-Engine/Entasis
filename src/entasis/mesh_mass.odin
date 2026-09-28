package entasis

import physics "entasis:entasis_physics"

// Mesh_Mass_Properties contains inertia and the center used by recentering
// operations. volume is signed for closed meshes and zero for open meshes
Mesh_Mass_Properties :: struct
{
	inertia: Body_Inertia,
	center:  Vector3,
	volume:  f32,
}

@(private)
registered_mesh :: #force_inline proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (^physics.Mesh, Status)
{
	if physics.typed_index_type(handle) != physics.MESH_TYPE_ID
	{
		return nil, .Invalid_Argument;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return nil, registry_status;
	}
	shape, _, status := physics.shape_registry_resolve(registry, handle);
	if status != .Ok
	{
		return nil, status;
	}
	return (^physics.Mesh)(shape), .Ok;
}

// mesh_closed_inertia computes closed-solid inertia for a registered mesh
mesh_closed_inertia :: proc "contextless" (
	world: ^World, handle: Shape_Handle, mass: f32,
) -> (Body_Inertia, Status)
{
	if !shape_mass_is_valid(mass)
	{
		return {}, .Invalid_Argument;
	}
	mesh, status := registered_mesh(world, handle);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.mesh_closed_inertia(mesh, mass);
}

// mesh_open_inertia computes triangle-soup inertia for a registered mesh
mesh_open_inertia :: proc "contextless" (
	world: ^World, handle: Shape_Handle, mass: f32,
) -> (Body_Inertia, Status)
{
	if !shape_mass_is_valid(mass)
	{
		return {}, .Invalid_Argument;
	}
	mesh, status := registered_mesh(world, handle);
	if status != .Ok
	{
		return {}, status;
	}
	return physics.mesh_open_inertia(mesh, mass);
}

// mesh_closed_center_of_mass returns signed closed volume and center of mass
mesh_closed_center_of_mass :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (volume: f32, center: Vector3, status: Status)
{
	mesh, mesh_status := registered_mesh(world, handle);
	if mesh_status != .Ok
	{
		return 0, {}, mesh_status;
	}
	return physics.mesh_closed_center_of_mass(mesh);
}

// mesh_open_center_of_mass returns the area-weighted center of an open triangle soup
mesh_open_center_of_mass :: proc "contextless" (
	world: ^World, handle: Shape_Handle,
) -> (Vector3, Status)
{
	mesh, mesh_status := registered_mesh(world, handle);
	if mesh_status != .Ok
	{
		return {}, mesh_status;
	}
	return physics.mesh_open_center_of_mass(mesh);
}
