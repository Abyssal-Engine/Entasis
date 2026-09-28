package entasis_cooking_c

import "base:intrinsics"
import "base:runtime"
import entasis "entasis:entasis"
import cooking "entasis:entasis_cooking"
import shared "entasis:entasis_c_shared"

ENTASIS_COOKING_STRUCT_VERSION :: u32(1);
ENTASIS_COOKING_CONTEXT_MAGIC :: u64(0x454E54434F4F4B31);
ENTASIS_COOKED_HULL_MAGIC :: u64(0x454E5448554C4C31);
ENTASIS_COOKED_MESH_MAGIC :: u64(0x454E544D45534831);
ENTASIS_COOKED_COMPOUND_MAGIC :: u64(0x454E54434D504431);
ENTASIS_COOKED_BIG_COMPOUND_MAGIC :: u64(0x454E5442434D5031);
abi_cooking_status :: #force_inline proc "contextless" (status: entasis.Status) -> Entasis_Status
{
	return Entasis_Status(status);
}

abi_cooking_diagnostic_clear :: #force_inline proc "contextless" (diagnostic: ^Entasis_Diagnostic)
{
	if diagnostic != nil
	{
		diagnostic^ = {};
	}
}

abi_cooking_finish :: #force_inline proc "contextless" (
	status: entasis.Status, diagnostic: ^Entasis_Diagnostic, detail: i32 = 0,
) -> Entasis_Status
{
	result := abi_cooking_status(status);
	if diagnostic != nil
	{
		if status == .Ok
		{
			diagnostic^ = {};
		}
		else
		{
			diagnostic^ = {
				status=result,
				operation=Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Asset_Import),
				detail=detail,
			};
		}
	}
	return result;
}

abi_cooking_allocator_is_default :: #force_inline proc "contextless" (allocator: ^Entasis_Allocator) -> bool
{
	return allocator == nil ||
	(allocator.allocate == nil && allocator.reallocate == nil && allocator.deallocate == nil);
}

abi_cooking_allocator_validate :: #force_inline proc "contextless" (allocator: ^Entasis_Allocator) -> entasis.Status
{
	if abi_cooking_allocator_is_default(allocator)
	{
		return .Ok;
	}
	if allocator.struct_size < u32(size_of(Entasis_Allocator)) ||
	allocator.struct_version != ENTASIS_COOKING_STRUCT_VERSION ||
	allocator.allocate == nil || allocator.deallocate == nil
	{
		return .Invalid_Description;
	}
	return .Ok;
}

abi_cooking_resource_allocate :: proc "contextless" (
	size, alignment: int, allocator: ^Entasis_Allocator,
	scope: entasis.Allocation_Scope = .Legacy,
) -> (rawptr, shared.Allocator, entasis.Status)
{
	status := abi_cooking_allocator_validate(allocator);
	if status != .Ok
	{
		return nil, {}, status;
	}
	copy := shared.Allocator{scope=scope};
	if allocator != nil
	{
		copy.user_context = allocator.user_context;
		copy.allocate = allocator.allocate;
		copy.reallocate = allocator.reallocate;
		copy.deallocate = allocator.deallocate;
	}
	context = runtime.default_context();
	return shared.Resource_Allocate(size, alignment, &copy);
}

abi_cooking_resource_allocate_owned :: proc "contextless" (
	size, alignment: int, allocator: ^shared.Allocator,
) -> (rawptr, shared.Allocator, entasis.Status)
{
	context = runtime.default_context();
	return shared.Resource_Allocate(size, alignment, allocator);
}

abi_cooking_resource_free :: proc "contextless" (
	memory: rawptr, size, alignment: int, allocator: ^shared.Allocator,
)
{
	context = runtime.default_context();
	shared.Resource_Free(memory, size, alignment, allocator);
}

abi_cooking_count_valid :: #force_inline proc "contextless" (count: u64) -> bool
{
	return count <= u64(max(int));
}

abi_cooking_vector_to_core :: #force_inline proc "contextless" (value: Entasis_Vector3) -> entasis.Vector3
{
	return {value.x, value.y, value.z};
}

abi_cooking_vector_from_core :: #force_inline proc "contextless" (value: entasis.Vector3) -> Entasis_Vector3
{
	return {value.x, value.y, value.z};
}

abi_cooking_shape_to_core :: #force_inline proc "contextless" (value: Entasis_Shape_Handle) -> entasis.Shape_Handle
{
	return {packed=value.packed};
}

abi_cooking_shape_from_core :: #force_inline proc "contextless" (value: entasis.Shape_Handle) -> Entasis_Shape_Handle
{
	return {packed=value.packed};
}

abi_cooking_shape_invalid :: #force_inline proc "contextless" () -> Entasis_Shape_Handle
{
	return abi_cooking_shape_from_core(entasis.shape_handle_invalid());
}

abi_cooking_inertia_from_core :: #force_inline proc "contextless" (value: entasis.Body_Inertia) -> Entasis_Body_Inertia
{
	return transmute(Entasis_Body_Inertia)value;
}

abi_cooking_mass_from_core :: #force_inline proc "contextless" (
	value: entasis.Mesh_Mass_Properties,
) -> Entasis_Mesh_Mass_Properties
{
	return {
		inertia=abi_cooking_inertia_from_core(value.inertia),
		center=abi_cooking_vector_from_core(value.center),
		volume=value.volume,
	};
}

abi_cooking_context_resource :: struct
{
	magic:     u64,
	allocator: shared.Allocator,
	ctx:       cooking.Cooking_Context,
}

abi_cooked_hull_resource :: struct
{
	magic:  u64,
	owner:  ^abi_cooking_context_resource,
	cooked: cooking.Cooked_Hull,
}

abi_cooked_mesh_resource :: struct
{
	magic:  u64,
	owner:  ^abi_cooking_context_resource,
	cooked: cooking.Cooked_Mesh,
}

abi_cooked_compound_resource :: struct
{
	magic:  u64,
	owner:  ^abi_cooking_context_resource,
	cooked: cooking.Cooked_Compound,
}

abi_cooked_big_compound_resource :: struct
{
	magic:  u64,
	owner:  ^abi_cooking_context_resource,
	cooked: cooking.Cooked_Big_Compound,
}

abi_cooking_context_get :: #force_inline proc "contextless" (
	handle: ^Entasis_Cooking_Context,
) -> ^abi_cooking_context_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_cooking_context_resource)(handle.opaque);
	if resource.magic != ENTASIS_COOKING_CONTEXT_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_cooked_hull_get :: #force_inline proc "contextless" (handle: ^Entasis_Cooked_Hull) -> ^abi_cooked_hull_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_cooked_hull_resource)(handle.opaque);
	if resource.magic != ENTASIS_COOKED_HULL_MAGIC ||
	resource.owner == nil ||
	resource.owner.magic != ENTASIS_COOKING_CONTEXT_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_cooked_mesh_get :: #force_inline proc "contextless" (handle: ^Entasis_Cooked_Mesh) -> ^abi_cooked_mesh_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_cooked_mesh_resource)(handle.opaque);
	if resource.magic != ENTASIS_COOKED_MESH_MAGIC ||
	resource.owner == nil ||
	resource.owner.magic != ENTASIS_COOKING_CONTEXT_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_cooked_compound_get :: #force_inline proc "contextless" (handle: ^Entasis_Cooked_Compound) -> ^abi_cooked_compound_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_cooked_compound_resource)(handle.opaque);
	if resource.magic != ENTASIS_COOKED_COMPOUND_MAGIC ||
	resource.owner == nil ||
	resource.owner.magic != ENTASIS_COOKING_CONTEXT_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_cooked_big_compound_get :: #force_inline proc "contextless" (handle: ^Entasis_Cooked_Big_Compound) -> ^abi_cooked_big_compound_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_cooked_big_compound_resource)(handle.opaque);
	if resource.magic != ENTASIS_COOKED_BIG_COMPOUND_MAGIC ||
	resource.owner == nil ||
	resource.owner.magic != ENTASIS_COOKING_CONTEXT_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_cooking_context_init :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	minimum_block_size, expected_resource_count: i32,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_cooking_context_init_extended(handle, minimum_block_size, expected_resource_count, allocator, 0, diagnostic);
}

abi_cooking_context_init_extended :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	minimum_block_size, expected_resource_count: i32,
	allocator: ^Entasis_Allocator,
	allocation_scope: Entasis_Allocation_Scope,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if handle == nil || handle.opaque != nil || minimum_block_size <= 0 || expected_resource_count <= 0 || allocation_scope > 1
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	allocator_status := abi_cooking_allocator_validate(allocator);
	if allocator_status != .Ok
	{
		return abi_cooking_finish(allocator_status, diagnostic);
	}
	memory, allocator_copy, allocation_status := abi_cooking_resource_allocate(
		size_of(abi_cooking_context_resource), align_of(abi_cooking_context_resource), allocator, entasis.Allocation_Scope(allocation_scope),
	);
	if allocation_status != .Ok
	{
		return abi_cooking_finish(allocation_status, diagnostic);
	}
	resource := (^abi_cooking_context_resource)(memory);
	resource.magic = ENTASIS_COOKING_CONTEXT_MAGIC;
	resource.allocator = allocator_copy;
	context = runtime.default_context();
	status := entasis.Status.Ok;
	if allocation_scope == 1
	{
		status = cooking.cooking_context_init_with_allocator(&resource.ctx,
			shared.Allocator_To_Core(&resource.allocator), int(minimum_block_size), int(expected_resource_count));
	}
	else
	{
		status = cooking.cooking_context_init(&resource.ctx, int(minimum_block_size), int(expected_resource_count));
	}
	if status != .Ok
	{
		allocator_copy = resource.allocator;
		resource.magic = 0;
		abi_cooking_resource_free(
			resource,
			size_of(abi_cooking_context_resource),
			align_of(abi_cooking_context_resource),
			&allocator_copy
		);
		return abi_cooking_finish(status, diagnostic);
	}
	handle.opaque = resource;
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooking_context_clear :: proc "contextless" (
	handle: ^Entasis_Cooking_Context, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	return abi_cooking_finish(cooking.cooking_context_clear(&resource.ctx), diagnostic);
}

abi_cooking_context_destroy :: proc "contextless" (
	handle: ^Entasis_Cooking_Context, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	status := cooking.cooking_context_destroy(&resource.ctx);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	allocator := resource.allocator;
	resource.magic = 0;
	handle.opaque = nil;
	abi_cooking_resource_free(
		resource,
		size_of(abi_cooking_context_resource),
		align_of(abi_cooking_context_resource),
		&allocator
	);
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cook_hull :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	points: [^]Entasis_Vector3, count: u64,
	out_cooked: ^Entasis_Cooked_Hull,
	out_center: ^Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_cooked == nil || out_center == nil || out_cooked.opaque != nil ||
	!abi_cooking_count_valid(count) || count < 4 || points == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_center^ = {};
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	cooked, status := cooking.cook_hull(&resource.ctx, (cast([^]entasis.Vector3)points)[:int(count)]);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	memory, _, allocation_status := abi_cooking_resource_allocate_owned(
		size_of(abi_cooked_hull_resource), align_of(abi_cooked_hull_resource), &resource.allocator,
	);
	if allocation_status != .Ok
	{
		_ = cooking.cooked_hull_destroy(&cooked);
		return abi_cooking_finish(allocation_status, diagnostic);
	}
	asset := (^abi_cooked_hull_resource)(memory);
	asset.magic = ENTASIS_COOKED_HULL_MAGIC;
	asset.owner = resource;
	asset.cooked = cooked;
	out_cooked.opaque = asset;
	out_center^ = abi_cooking_vector_from_core(cooked.center);
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooked_hull_validate :: proc "contextless" (handle: ^Entasis_Cooked_Hull) -> Entasis_Status
{
	resource := abi_cooked_hull_get(handle);
	if resource == nil
	{
		return abi_cooking_status(.Disposed);
	}
	return abi_cooking_status(cooking.cooked_hull_validate(&resource.cooked));
}

abi_cooked_hull_import :: proc "contextless" (
	world: ^Entasis_World, handle: ^Entasis_Cooked_Hull,
	out_handle: ^Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_handle^ = abi_cooking_shape_invalid();
	resource := abi_cooked_hull_get(handle);
	world_header := shared.World_Header_Get(world.opaque) if world != nil else nil;
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	if world_header == nil || world_header.access != .Ready
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	world_header.access = .Exclusive;
	defer world_header.access = .Ready;
	core_world := world_header.world;
	context = runtime.default_context();
	shape, status := cooking.cooked_hull_import(core_world, &resource.cooked);
	if status == .Ok
	{
		out_handle^ = abi_cooking_shape_from_core(shape);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cooked_hull_destroy :: proc "contextless" (
	handle: ^Entasis_Cooked_Hull, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooked_hull_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	owner := resource.owner;
	context = runtime.default_context();
	status := cooking.cooked_hull_destroy(&resource.cooked);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	resource.magic = 0;
	handle.opaque = nil;
	abi_cooking_resource_free(
		resource,
		size_of(abi_cooked_hull_resource),
		align_of(abi_cooked_hull_resource),
		&owner.allocator
	);
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cook_mesh :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	triangles: [^]Entasis_Triangle, count: u64,
	scale: Entasis_Vector3,
	out_cooked: ^Entasis_Cooked_Mesh,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_cooked == nil || out_cooked.opaque != nil ||
	!abi_cooking_count_valid(count) || count == 0 || triangles == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	cooked, status := cooking.cook_mesh(
		&resource.ctx, (cast([^]entasis.Triangle)triangles)[:int(count)], abi_cooking_vector_to_core(scale),
	);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	memory, _, allocation_status := abi_cooking_resource_allocate_owned(
		size_of(abi_cooked_mesh_resource), align_of(abi_cooked_mesh_resource), &resource.allocator,
	);
	if allocation_status != .Ok
	{
		_ = cooking.cooked_mesh_destroy(&cooked);
		return abi_cooking_finish(allocation_status, diagnostic);
	}
	asset := (^abi_cooked_mesh_resource)(memory);
	asset.magic = ENTASIS_COOKED_MESH_MAGIC;
	asset.owner = resource;
	asset.cooked = cooked;
	out_cooked.opaque = asset;
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooked_mesh_import :: proc "contextless" (
	world: ^Entasis_World, handle: ^Entasis_Cooked_Mesh,
	out_handle: ^Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_handle^ = abi_cooking_shape_invalid();
	resource := abi_cooked_mesh_get(handle);
	world_header := shared.World_Header_Get(world.opaque) if world != nil else nil;
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	if world_header == nil || world_header.access != .Ready
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	world_header.access = .Exclusive;
	defer world_header.access = .Ready;
	core_world := world_header.world;
	context = runtime.default_context();
	shape, status := cooking.cooked_mesh_import(core_world, &resource.cooked);
	if status == .Ok
	{
		out_handle^ = abi_cooking_shape_from_core(shape);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cooked_mesh_destroy :: proc "contextless" (
	handle: ^Entasis_Cooked_Mesh, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooked_mesh_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	owner := resource.owner;
	context = runtime.default_context();
	status := cooking.cooked_mesh_destroy(&resource.cooked);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	resource.magic = 0;
	handle.opaque = nil;
	abi_cooking_resource_free(
		resource,
		size_of(abi_cooked_mesh_resource),
		align_of(abi_cooked_mesh_resource),
		&owner.allocator
	);
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooked_mesh_closed_mass_properties :: proc "contextless" (
	handle: ^Entasis_Cooked_Mesh, mass: f32, recenter: Entasis_Bool,
	out_properties: ^Entasis_Mesh_Mass_Properties, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_properties == nil || recenter > 1
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_properties^ = {};
	resource := abi_cooked_mesh_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	value, status := cooking.cooked_mesh_closed_mass_properties(&resource.cooked, mass, recenter != 0);
	if status == .Ok
	{
		out_properties^ = abi_cooking_mass_from_core(value);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cooked_mesh_open_mass_properties :: proc "contextless" (
	handle: ^Entasis_Cooked_Mesh, mass: f32, recenter: Entasis_Bool,
	out_properties: ^Entasis_Mesh_Mass_Properties, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_properties == nil || recenter > 1
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_properties^ = {};
	resource := abi_cooked_mesh_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	value, status := cooking.cooked_mesh_open_mass_properties(&resource.cooked, mass, recenter != 0);
	if status == .Ok
	{
		out_properties^ = abi_cooking_mass_from_core(value);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cook_compound :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	children: [^]Entasis_Cooked_Compound_Child, count: u64,
	out_cooked: ^Entasis_Cooked_Compound,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_cooked == nil || out_cooked.opaque != nil ||
	!abi_cooking_count_valid(count) || count == 0 || children == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	cooked, status := cooking.cook_compound(
		&resource.ctx, (cast([^]cooking.Cooked_Compound_Child)children)[:int(count)],
	);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	memory, _, allocation_status := abi_cooking_resource_allocate_owned(
		size_of(abi_cooked_compound_resource), align_of(abi_cooked_compound_resource), &resource.allocator,
	);
	if allocation_status != .Ok
	{
		_ = cooking.cooked_compound_destroy(&cooked);
		return abi_cooking_finish(allocation_status, diagnostic);
	}
	asset := (^abi_cooked_compound_resource)(memory);
	asset.magic = ENTASIS_COOKED_COMPOUND_MAGIC;
	asset.owner = resource;
	asset.cooked = cooked;
	out_cooked.opaque = asset;
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooked_compound_import :: proc "contextless" (
	world: ^Entasis_World, handle: ^Entasis_Cooked_Compound,
	shape_slots: [^]Entasis_Shape_Handle, shape_slot_count: u64,
	out_handle: ^Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil || !abi_cooking_count_valid(shape_slot_count) || shape_slot_count == 0 || shape_slots == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_handle^ = abi_cooking_shape_invalid();
	resource := abi_cooked_compound_get(handle);
	world_header := shared.World_Header_Get(world.opaque) if world != nil else nil;
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	if world_header == nil || world_header.access != .Ready
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	world_header.access = .Exclusive;
	defer world_header.access = .Ready;
	core_world := world_header.world;
	context = runtime.default_context();
	shape, status := cooking.cooked_compound_import(
		core_world, &resource.cooked, (cast([^]entasis.Shape_Handle)shape_slots)[:int(shape_slot_count)],
	);
	if status == .Ok
	{
		out_handle^ = abi_cooking_shape_from_core(shape);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cooked_compound_destroy :: proc "contextless" (
	handle: ^Entasis_Cooked_Compound, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooked_compound_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	owner := resource.owner;
	context = runtime.default_context();
	status := cooking.cooked_compound_destroy(&resource.cooked);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	resource.magic = 0;
	handle.opaque = nil;
	abi_cooking_resource_free(
		resource,
		size_of(abi_cooked_compound_resource),
		align_of(abi_cooked_compound_resource),
		&owner.allocator
	);
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cook_big_compound :: proc "contextless" (
	handle: ^Entasis_Cooking_Context,
	children: [^]Entasis_Cooked_Big_Compound_Child, count: u64,
	out_cooked: ^Entasis_Cooked_Big_Compound,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_cooked == nil || out_cooked.opaque != nil ||
	!abi_cooking_count_valid(count) || count == 0 || children == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	resource := abi_cooking_context_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	context = runtime.default_context();
	cooked, status := cooking.cook_big_compound(
		&resource.ctx, (cast([^]cooking.Cooked_Big_Compound_Child)children)[:int(count)],
	);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	memory, _, allocation_status := abi_cooking_resource_allocate_owned(
		size_of(abi_cooked_big_compound_resource), align_of(abi_cooked_big_compound_resource), &resource.allocator,
	);
	if allocation_status != .Ok
	{
		_ = cooking.cooked_big_compound_destroy(&cooked);
		return abi_cooking_finish(allocation_status, diagnostic);
	}
	asset := (^abi_cooked_big_compound_resource)(memory);
	asset.magic = ENTASIS_COOKED_BIG_COMPOUND_MAGIC;
	asset.owner = resource;
	asset.cooked = cooked;
	out_cooked.opaque = asset;
	return abi_cooking_finish(.Ok, diagnostic);
}

abi_cooked_big_compound_import :: proc "contextless" (
	world: ^Entasis_World, handle: ^Entasis_Cooked_Big_Compound,
	shape_slots: [^]Entasis_Shape_Handle, shape_slot_count: u64,
	out_handle: ^Entasis_Shape_Handle, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle == nil || !abi_cooking_count_valid(shape_slot_count) || shape_slot_count == 0 || shape_slots == nil
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	out_handle^ = abi_cooking_shape_invalid();
	resource := abi_cooked_big_compound_get(handle);
	world_header := shared.World_Header_Get(world.opaque) if world != nil else nil;
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	if world_header == nil || world_header.access != .Ready
	{
		return abi_cooking_finish(.Invalid_Argument, diagnostic);
	}
	world_header.access = .Exclusive;
	defer world_header.access = .Ready;
	core_world := world_header.world;
	context = runtime.default_context();
	shape, status := cooking.cooked_big_compound_import(
		core_world, &resource.cooked, (cast([^]entasis.Shape_Handle)shape_slots)[:int(shape_slot_count)],
	);
	if status == .Ok
	{
		out_handle^ = abi_cooking_shape_from_core(shape);
	}
	return abi_cooking_finish(status, diagnostic);
}

abi_cooked_big_compound_destroy :: proc "contextless" (
	handle: ^Entasis_Cooked_Big_Compound, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_cooked_big_compound_get(handle);
	if resource == nil
	{
		return abi_cooking_finish(.Disposed, diagnostic);
	}
	owner := resource.owner;
	context = runtime.default_context();
	status := cooking.cooked_big_compound_destroy(&resource.cooked);
	if status != .Ok
	{
		return abi_cooking_finish(status, diagnostic);
	}
	resource.magic = 0;
	handle.opaque = nil;
	abi_cooking_resource_free(
		resource,
		size_of(abi_cooked_big_compound_resource),
		align_of(abi_cooked_big_compound_resource),
		&owner.allocator
	);
	return abi_cooking_finish(.Ok, diagnostic);
}
#assert(size_of(Entasis_Vector3) == size_of(entasis.Vector3));
#assert(size_of(Entasis_Triangle) == size_of(entasis.Triangle));
#assert(size_of(Entasis_Shape_Handle) == size_of(entasis.Shape_Handle));
#assert(size_of(Entasis_Cooked_Compound_Child) == size_of(cooking.Cooked_Compound_Child));
#assert(size_of(Entasis_Cooked_Big_Compound_Child) == size_of(cooking.Cooked_Big_Compound_Child));
#assert(size_of(Entasis_Body_Inertia) == size_of(entasis.Body_Inertia));
