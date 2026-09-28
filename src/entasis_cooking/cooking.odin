// package entasis_cooking provides optional cold collision-asset preparation.
// it owns no scheduler and never mutates a live physics world while cooking
package entasis_cooking

import "core:mem"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Cooking_Context_State :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

// Cooking_Context is caller owned and must remain at a stable address while any
// cooked asset references it. one ctx may be used by one worker at a time
Cooking_Context :: struct
{
	pool:        util.Buffer_Pool,
	live_assets: int,
	state:       Cooking_Context_State,
}

Cooked_Asset_State :: enum u8
{
	Empty,
	Ready,
	Disposed,
}

Cooked_Hull :: struct
{
	hull:   physics.Convex_Hull,
	center: entasis.Vector3,
	owner:  ^Cooking_Context,
	state:  Cooked_Asset_State,
}

Cooked_Mesh :: struct
{
	mesh:  physics.Mesh,
	owner: ^Cooking_Context,
	state: Cooked_Asset_State,
}

Cooked_Compound_Child :: struct
{
	local_pose: entasis.Rigid_Pose,
	shape_slot: i32,
}

Cooked_Compound :: struct
{
	children: util.Buffer(Cooked_Compound_Child),
	owner:    ^Cooking_Context,
	state:    Cooked_Asset_State,
}

@(private)
cooking_status :: proc "contextless" (status: util.Memory_Status) -> entasis.Status
{
	switch status
	{
		case .Ok:
		return .Ok;
		case .Pool_Disposed:
		return .Disposed;
		case .Capacity_Missing, .Out_Of_Memory, .Overflow:
		return .Capacity_Missing;
		case .Invalid_Count, .Invalid_Alignment, .Invalid_Power, .Invalid_Buffer:
		return .Invalid_Argument;
	}
	return .Invalid_Argument;
}

// cooking_context_init initializes one worker-owned cooking pool
cooking_context_init :: proc (
	ctx: ^Cooking_Context,
	minimum_block_size: int = 131072,
	expected_resource_count: int = 16,
) -> entasis.Status
{
	if ctx == nil || ctx.state != .Uninitialized
	{
		return .Invalid_Argument;
	}
	status := util.buffer_pool_initialize(
		&ctx.pool, minimum_block_size, expected_resource_count,
	);
	if status != .Ok
	{
		return cooking_status(status);
	}
	ctx.state = .Ready;
	return .Ok;
}

// full ownership includes pool metadata, geometry and temporary cooking buffers
cooking_context_init_with_allocator :: proc (
	ctx: ^Cooking_Context, allocator: mem.Allocator,
	minimum_block_size: int = 131072, expected_resource_count: int = 16,
) -> entasis.Status
{
	if ctx == nil || ctx.state != .Uninitialized
	{
		return .Invalid_Argument;
	}
	status: util.Memory_Status = util.buffer_pool_initialize_with_allocator(
		&ctx.pool, allocator, minimum_block_size, expected_resource_count,
	);
	if status != .Ok
	{
		return cooking_status(status);
	}
	ctx.state = .Ready;
	return .Ok;
}

// cooking_context_clear releases native blocks while retaining pool metadata.
// all assets must be destroyed first
cooking_context_clear :: proc (ctx: ^Cooking_Context) -> entasis.Status
{
	if ctx == nil || ctx.state != .Ready
	{
		return .Disposed;
	}
	if ctx.live_assets != 0
	{
		return .Shape_In_Use;
	}
	return cooking_status(util.buffer_pool_clear(&ctx.pool));
}

// cooking_context_destroy destroys an idle ctx. all assets must be destroyed first
cooking_context_destroy :: proc (ctx: ^Cooking_Context) -> entasis.Status
{
	if ctx == nil || ctx.state != .Ready
	{
		return .Disposed;
	}
	if ctx.live_assets != 0
	{
		return .Shape_In_Use;
	}
	status := cooking_status(util.buffer_pool_dispose(&ctx.pool));
	if status == .Ok
	{
		ctx^ = {state=.Disposed};
	}
	return status;
}

// cook_hull builds immutable convex-hull topology from one point cloud. safe on
// a background worker when the ctx and input are owned by that worker
cook_hull :: proc (
	ctx: ^Cooking_Context,
	points: []entasis.Vector3,
) -> (Cooked_Hull, entasis.Status)
{
	if ctx == nil || ctx.state != .Ready || len(points) < 4
	{
		return {}, .Invalid_Description;
	}
	cooked: Cooked_Hull;
	status := physics.convex_hull_create_from_point_cloud(
		&cooked.hull, &cooked.center, raw_data(points), len(points), &ctx.pool,
	);
	if status != .Ok
	{
		return {}, status;
	}
	cooked.owner = ctx;
	cooked.state = .Ready;
	ctx.live_assets += 1;
	return cooked, .Ok;
}

cooked_hull_validate :: #force_inline proc "contextless" (
	cooked: ^Cooked_Hull,
) -> entasis.Status
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return .Disposed;
	}
	return physics.convex_hull_validate(&cooked.hull);
}

cooked_hull_import :: proc (
	world: ^entasis.World,
	cooked: ^Cooked_Hull,
) -> (entasis.Shape_Handle, entasis.Status)
{
	validation := cooked_hull_validate(cooked);
	if validation != .Ok
	{
		return entasis.shape_handle_invalid(), validation;
	}
	return entasis.shape_import_convex_hull(world, &cooked.hull);
}

cooked_hull_destroy :: proc (cooked: ^Cooked_Hull) -> entasis.Status
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return .Disposed;
	}
	owner := cooked.owner;
	status := physics.convex_hull_dispose(&cooked.hull, &owner.pool);
	if status != .Ok
	{
		return status;
	}
	owner.live_assets -= 1;
	cooked^ = {state=.Disposed};
	return .Ok;
}

// cook_mesh builds the immutable mesh bounds tree in the cooking ctx
cook_mesh :: proc (
	ctx: ^Cooking_Context,
	triangles: []entasis.Triangle,
	scale: entasis.Vector3 = {1, 1, 1},
) -> (Cooked_Mesh, entasis.Status)
{
	if ctx == nil || ctx.state != .Ready || len(triangles) <= 0
	{
		return {}, .Invalid_Description;
	}
	cooked: Cooked_Mesh;
	status := physics.mesh_create(
		&cooked.mesh, raw_data(triangles), len(triangles), scale, &ctx.pool,
	);
	if status != .Ok
	{
		return {}, status;
	}
	cooked.owner = ctx;
	cooked.state = .Ready;
	ctx.live_assets += 1;
	return cooked, .Ok;
}

cooked_mesh_import :: proc (
	world: ^entasis.World,
	cooked: ^Cooked_Mesh,
) -> (entasis.Shape_Handle, entasis.Status)
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return entasis.shape_handle_invalid(), .Disposed;
	}
	return entasis.shape_import_mesh(world, &cooked.mesh);
}

cooked_mesh_destroy :: proc (cooked: ^Cooked_Mesh) -> entasis.Status
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return .Disposed;
	}
	owner := cooked.owner;
	status := physics.mesh_dispose(&cooked.mesh, &owner.pool);
	if status != .Ok
	{
		return status;
	}
	owner.live_assets -= 1;
	cooked^ = {state=.Disposed};
	return .Ok;
}

// cook_compound copies portable local poses and child-shape slots. import maps
// each slot through a caller-provided shape-handle table
cook_compound :: proc (
	ctx: ^Cooking_Context,
	children: []Cooked_Compound_Child,
) -> (Cooked_Compound, entasis.Status)
{
	if ctx == nil || ctx.state != .Ready || len(children) <= 0
	{
		return {}, .Invalid_Description;
	}
	for child in children
	{
		if child.shape_slot < 0
		{
			return {}, .Invalid_Description;
		}
	}
	buffer, status := util.buffer_pool_take_at_least(
		&ctx.pool, Cooked_Compound_Child, len(children),
	);
	if status != .Ok
	{
		return {}, cooking_status(status);
	}
	copy_status := util.buffer_copy(
		util.Buffer_View(Cooked_Compound_Child){
			memory=raw_data(children), length=len(children), capacity=len(children),
		},
		0, util.buffer_view(buffer), 0, len(children),
	);
	if copy_status != .Ok
	{
		_ = util.buffer_pool_return(&ctx.pool, &buffer);
		return {}, cooking_status(copy_status);
	}
	buffer.length = i32(len(children));
	ctx.live_assets += 1;
	return {children=buffer, owner=ctx, state=.Ready}, .Ok;
}

cooked_compound_import :: proc (
	world: ^entasis.World,
	cooked: ^Cooked_Compound,
	shape_slots: []entasis.Shape_Handle,
) -> (entasis.Shape_Handle, entasis.Status)
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return entasis.shape_handle_invalid(), .Disposed;
	}
	if len(shape_slots) <= 0
	{
		return entasis.shape_handle_invalid(), .Invalid_Argument;
	}
	temporary, take_status := util.buffer_pool_take_at_least(
		&cooked.owner.pool, entasis.Compound_Child, int(cooked.children.length),
	);
	if take_status != .Ok
	{
		return entasis.shape_handle_invalid(), cooking_status(take_status);
	}
	temporary.length = cooked.children.length;
	for index in 0 ..< int(cooked.children.length)
	{
		source := cooked.children.memory[index];
		if source.shape_slot < 0 || int(source.shape_slot) >= len(shape_slots)
		{
			_ = util.buffer_pool_return(&cooked.owner.pool, &temporary);
			return entasis.shape_handle_invalid(), .Invalid_Description;
		}
		temporary.memory[index] = entasis.compound_child(
			shape_slots[source.shape_slot], source.local_pose,
		);
	}
	handle, status := entasis.shape_import_compound(
		world, temporary.memory[:int(temporary.length)],
	);
	_ = util.buffer_pool_return(&cooked.owner.pool, &temporary);
	return handle, status;
}

cooked_compound_destroy :: proc (cooked: ^Cooked_Compound) -> entasis.Status
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return .Disposed;
	}
	owner := cooked.owner;
	status := cooking_status(util.buffer_pool_return(&owner.pool, &cooked.children));
	if status != .Ok
	{
		return status;
	}
	owner.live_assets -= 1;
	cooked^ = {state=.Disposed};
	return .Ok;
}

// Cooked_Big_Compound_Child carries a portable child slot and local-space
// bounds. local_bounds must already include local_pose translation and rotation
Cooked_Big_Compound_Child :: struct
{
	local_pose:   entasis.Rigid_Pose,
	shape_slot:   i32,
	local_bounds: entasis.Bounding_Box,
}

Cooked_Big_Compound :: struct
{
	children: util.Buffer(Cooked_Compound_Child),
	bounds:   util.Buffer(entasis.Bounding_Box),
	tree:     physics.Tree,
	owner:    ^Cooking_Context,
	state:    Cooked_Asset_State,
}

// cooked_mesh_closed_mass_properties computes closed-solid mass properties.
// when recenter is true, the cooked mesh geometry and acceleration tree are
// shifted so the returned center becomes the body pose offset
cooked_mesh_closed_mass_properties :: proc (
	cooked: ^Cooked_Mesh, mass: f32, recenter: bool = true,
) -> (entasis.Mesh_Mass_Properties, entasis.Status)
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return {}, .Disposed;
	}
	if recenter
	{
		inertia, center, status := physics.mesh_closed_inertia_recenter(&cooked.mesh, mass);
		if status != .Ok
		{
			return {}, status;
		}
		volume, _, center_status := physics.mesh_closed_center_of_mass(&cooked.mesh);
		if center_status != .Ok
		{
			return {}, center_status;
		}
		return {inertia=inertia, center=center, volume=volume}, .Ok;
	}
	inertia, inertia_status := physics.mesh_closed_inertia(&cooked.mesh, mass);
	if inertia_status != .Ok
	{
		return {}, inertia_status;
	}
	volume, center, center_status := physics.mesh_closed_center_of_mass(&cooked.mesh);
	if center_status != .Ok
	{
		return {}, center_status;
	}
	return {inertia=inertia, center=center, volume=volume}, .Ok;
}

// cooked_mesh_open_mass_properties computes triangle-soup mass properties
cooked_mesh_open_mass_properties :: proc (
	cooked: ^Cooked_Mesh, mass: f32, recenter: bool = true,
) -> (entasis.Mesh_Mass_Properties, entasis.Status)
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return {}, .Disposed;
	}
	if recenter
	{
		inertia, center, status := physics.mesh_open_inertia_recenter(&cooked.mesh, mass);
		if status != .Ok
		{
			return {}, status;
		}
		return {inertia=inertia, center=center}, .Ok;
	}
	inertia, inertia_status := physics.mesh_open_inertia(&cooked.mesh, mass);
	if inertia_status != .Ok
	{
		return {}, inertia_status;
	}
	center, center_status := physics.mesh_open_center_of_mass(&cooked.mesh);
	if center_status != .Ok
	{
		return {}, center_status;
	}
	return {inertia=inertia, center=center}, .Ok;
}

// cook_big_compound prepares a portable tree-accelerated compound without a
// live World. child bounds are supplied by the asset pipeline
cook_big_compound :: proc (
	ctx: ^Cooking_Context,
	children: []Cooked_Big_Compound_Child,
) -> (Cooked_Big_Compound, entasis.Status)
{
	if ctx == nil || ctx.state != .Ready || len(children) <= 0
	{
		return {}, .Invalid_Description;
	}
	portable, child_status := util.buffer_pool_take_at_least(
		&ctx.pool, Cooked_Compound_Child, len(children),
	);
	if child_status != .Ok
	{
		return {}, cooking_status(child_status);
	}
	bounds, bounds_status := util.buffer_pool_take_at_least(
		&ctx.pool, entasis.Bounding_Box, len(children),
	);
	if bounds_status != .Ok
	{
		_ = util.buffer_pool_return(&ctx.pool, &portable);
		return {}, cooking_status(bounds_status);
	}
	portable.length = i32(len(children));
	bounds.length = i32(len(children));
	for index in 0 ..< len(children)
	{
		child := children[index];
		if child.shape_slot < 0
		{
			_ = util.buffer_pool_return(&ctx.pool, &bounds);
			_ = util.buffer_pool_return(&ctx.pool, &portable);
			return {}, .Invalid_Description;
		}
		portable.memory[index] = {
			local_pose=child.local_pose,
			shape_slot=child.shape_slot,
		};
		bounds.memory[index] = child.local_bounds;
	}
	tree: physics.Tree;
	tree_status := physics.tree_initialize(&tree, len(children), &ctx.pool);
	if tree_status != .Ok
	{
		_ = util.buffer_pool_return(&ctx.pool, &bounds);
		_ = util.buffer_pool_return(&ctx.pool, &portable);
		return {}, tree_status;
	}
	build_status := physics.tree_build(&tree, bounds.memory, len(children));
	if build_status != .Ok
	{
		_ = physics.tree_dispose(&tree);
		_ = util.buffer_pool_return(&ctx.pool, &bounds);
		_ = util.buffer_pool_return(&ctx.pool, &portable);
		return {}, build_status;
	}
	ctx.live_assets += 1;
	return {
		children=portable,
		bounds=bounds,
		tree=tree,
		owner=ctx,
		state=.Ready,
	}, .Ok;
}

cooked_big_compound_import :: proc (
	world: ^entasis.World,
	cooked: ^Cooked_Big_Compound,
	shape_slots: []entasis.Shape_Handle,
) -> (entasis.Shape_Handle, entasis.Status)
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return entasis.shape_handle_invalid(), .Disposed;
	}
	mapped, take_status := util.buffer_pool_take_at_least(
		&cooked.owner.pool, entasis.Compound_Child, int(cooked.children.length),
	);
	if take_status != .Ok
	{
		return entasis.shape_handle_invalid(), cooking_status(take_status);
	}
	mapped.length = cooked.children.length;
	for index in 0 ..< int(cooked.children.length)
	{
		source := cooked.children.memory[index];
		if source.shape_slot < 0 || int(source.shape_slot) >= len(shape_slots)
		{
			_ = util.buffer_pool_return(&cooked.owner.pool, &mapped);
			return entasis.shape_handle_invalid(), .Invalid_Description;
		}
		mapped.memory[index] = entasis.compound_child(
			shape_slots[source.shape_slot], source.local_pose,
		);
	}
	handle, status := entasis.shape_import_big_compound_cooked(
		world,
		mapped.memory[:int(mapped.length)],
		cooked.bounds.memory[:int(cooked.bounds.length)],
		&cooked.tree,
	);
	_ = util.buffer_pool_return(&cooked.owner.pool, &mapped);
	return handle, status;
}

cooked_big_compound_destroy :: proc (
	cooked: ^Cooked_Big_Compound,
) -> entasis.Status
{
	if cooked == nil || cooked.state != .Ready || cooked.owner == nil ||
	cooked.owner.state != .Ready
	{
		return .Disposed;
	}
	owner := cooked.owner;
	_ = physics.tree_dispose(&cooked.tree);
	_ = util.buffer_pool_return(&owner.pool, &cooked.bounds);
	_ = util.buffer_pool_return(&owner.pool, &cooked.children);
	owner.live_assets -= 1;
	cooked^ = {state=.Disposed};
	return .Ok;
}
