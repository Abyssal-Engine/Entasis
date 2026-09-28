package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Compound_Builder is a caller-owned zero-allocation view over child and mass
// arrays. the arrays must have equal nonzero lengths and remain valid for the
// duration of each builder operation
Compound_Builder :: struct
{
	children: []Compound_Child,
	masses:   []f32,
}

// Compound_Build_Result contains a registered shape and mass properties. when
// recentering is requested, center is the compound-local offset subtracted from
// every child local position before registration
Compound_Build_Result :: struct
{
	shape:   Shape_Handle,
	inertia: Body_Inertia,
	center:  Vector3,
}

// compound_builder binds caller-owned child and mass arrays without allocation
compound_builder :: #force_inline proc "contextless" (
	children: []Compound_Child, masses: []f32,
) -> Compound_Builder
{
	return {children=children, masses=masses};
}

@(private)
compound_builder_proxy :: #force_inline proc "contextless" (
	builder: Compound_Builder,
) -> (physics.Compound, Status)
{
	if len(builder.children) <= 0 || len(builder.children) != len(builder.masses)
	{
		return {}, .Invalid_Description;
	}
	for mass in builder.masses
	{
		if !shape_mass_is_valid(mass)
		{
			return {}, .Invalid_Description;
		}
	}
	return {
		children=util.Buffer(physics.Compound_Child){
			memory=raw_data(builder.children),
			length=i32(len(builder.children)),
			id=-1,
		},
	}, .Ok;
}

// compound_center_of_mass computes the weighted center and inverse total mass
// without allocating or mutating children
compound_center_of_mass :: proc "contextless" (
	builder: Compound_Builder,
) -> (center: Vector3, inverse_mass: f32, status: Status)
{
	proxy, proxy_status := compound_builder_proxy(builder);
	if proxy_status != .Ok
	{
		return {}, 0, proxy_status;
	}
	return physics.compound_center_of_mass(
		&proxy, raw_data(builder.masses), len(builder.masses),
	);
}

// compound_inertia_weighted computes mass properties around the current child
// origin. it performs no allocation and does not modify the builder
compound_inertia_weighted :: proc "contextless" (
	world: ^World, builder: Compound_Builder,
) -> (Body_Inertia, Status)
{
	proxy, proxy_status := compound_builder_proxy(builder);
	if proxy_status != .Ok
	{
		return {}, proxy_status;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return {}, registry_status;
	}
	return physics.compound_inertia_weighted(
		&proxy, registry, raw_data(builder.masses), len(builder.masses),
	);
}

// compound_inertia_weighted_recenter computes mass properties about the center
// of mass and subtracts that center from every caller-owned child position
compound_inertia_weighted_recenter :: proc "contextless" (
	world: ^World, builder: Compound_Builder,
) -> (Body_Inertia, Vector3, Status)
{
	proxy, proxy_status := compound_builder_proxy(builder);
	if proxy_status != .Ok
	{
		return {}, {}, proxy_status;
	}
	registry, registry_status := shape_registry_from_world(world);
	if registry_status != .Ok
	{
		return {}, {}, registry_status;
	}
	return physics.compound_inertia_weighted_recenter(
		&proxy, registry, raw_data(builder.masses), len(builder.masses),
	);
}

// compound_build_dynamic optionally recenters caller-owned children, computes
// weighted inertia, and registers a standard Compound in one owner-thread call
compound_build_dynamic :: proc (
	world: ^World,
	builder: Compound_Builder,
	recenter: bool = true,
) -> (Compound_Build_Result, Status)
{
	result := Compound_Build_Result{shape=shape_handle_invalid()};
	if recenter
	{
		inertia, center, status := compound_inertia_weighted_recenter(world, builder);
		if status != .Ok
		{
			return result, status;
		}
		result.inertia = inertia;
		result.center = center;
	}
	else
	{
		inertia, status := compound_inertia_weighted(world, builder);
		if status != .Ok
		{
			return result, status;
		}
		result.inertia = inertia;
	}
	shape, status := shape_import_compound(world, builder.children);
	if status != .Ok
	{
		return result, status;
	}
	result.shape = shape;
	return result, .Ok;
}

// big_compound_build_dynamic performs the same weighted mass calculation and
// registers a tree-accelerated Big_Compound. it is intended for larger child
// counts whose local queries benefit from an internal tree
big_compound_build_dynamic :: proc (
	world: ^World,
	builder: Compound_Builder,
	recenter: bool = true,
) -> (Compound_Build_Result, Status)
{
	result := Compound_Build_Result{shape=shape_handle_invalid()};
	if recenter
	{
		inertia, center, status := compound_inertia_weighted_recenter(world, builder);
		if status != .Ok
		{
			return result, status;
		}
		result.inertia = inertia;
		result.center = center;
	}
	else
	{
		inertia, status := compound_inertia_weighted(world, builder);
		if status != .Ok
		{
			return result, status;
		}
		result.inertia = inertia;
	}
	shape, status := shape_import_big_compound(world, builder.children);
	if status != .Ok
	{
		return result, status;
	}
	result.shape = shape;
	return result, .Ok;
}
