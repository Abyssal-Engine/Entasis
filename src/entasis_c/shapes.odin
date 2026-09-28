package entasis_c

import "base:runtime"
import entasis "entasis:entasis"

abi_sphere :: #force_inline proc "contextless" (radius: f32) -> Entasis_Sphere
{
	return cast(Entasis_Sphere)entasis.sphere(radius);
}

abi_box :: #force_inline proc "contextless" (width, height, depth: f32) -> Entasis_Box
{
	return cast(Entasis_Box)entasis.box(width, height, depth);
}

abi_box_half_extents :: #force_inline proc "contextless" (
	half_width, half_height, half_depth: f32,
) -> Entasis_Box
{
	return cast(Entasis_Box)entasis.box_half_extents(half_width, half_height, half_depth);
}

abi_capsule :: #force_inline proc "contextless" (radius, length: f32) -> Entasis_Capsule
{
	return cast(Entasis_Capsule)entasis.capsule(radius, length);
}

abi_capsule_half_length :: #force_inline proc "contextless" (radius, half_length: f32) -> Entasis_Capsule
{
	return cast(Entasis_Capsule)entasis.capsule_half_length(radius, half_length);
}

abi_cylinder :: #force_inline proc "contextless" (radius, length: f32) -> Entasis_Cylinder
{
	return cast(Entasis_Cylinder)entasis.cylinder(radius, length);
}

abi_cylinder_half_length :: #force_inline proc "contextless" (radius, half_length: f32) -> Entasis_Cylinder
{
	return cast(Entasis_Cylinder)entasis.cylinder_half_length(radius, half_length);
}

abi_triangle_to_core :: #force_inline proc "contextless" (value: Entasis_Triangle) -> entasis.Triangle
{
	return {
		a=abi_vector3_to_core(value.a),
		b=abi_vector3_to_core(value.b),
		c=abi_vector3_to_core(value.c),
	};
}

abi_triangle_from_core :: #force_inline proc "contextless" (value: entasis.Triangle) -> Entasis_Triangle
{
	return {
		a=abi_vector3_from_core(value.a),
		b=abi_vector3_from_core(value.b),
		c=abi_vector3_from_core(value.c),
	};
}

abi_triangle :: #force_inline proc "contextless" (a, b, c: Entasis_Vector3) -> Entasis_Triangle
{
	return abi_triangle_from_core(entasis.triangle(
			abi_vector3_to_core(a), abi_vector3_to_core(b), abi_vector3_to_core(c),
	));
}

abi_shape_type_id :: #force_inline proc "contextless" (
	type_id: Entasis_Shape_Type_ID,
) -> Entasis_Shape_Type_ID
{
	switch type_id
	{
		case i32(entasis.SHAPE_TYPE_SPHERE), i32(entasis.SHAPE_TYPE_CAPSULE),
		i32(entasis.SHAPE_TYPE_BOX), i32(entasis.SHAPE_TYPE_TRIANGLE),
		i32(entasis.SHAPE_TYPE_CYLINDER), i32(entasis.SHAPE_TYPE_CONVEX_HULL),
		i32(entasis.SHAPE_TYPE_COMPOUND), i32(entasis.SHAPE_TYPE_BIG_COMPOUND),
		i32(entasis.SHAPE_TYPE_MESH):
		return type_id;
	}
	return Entasis_Shape_Type_ID(entasis.SHAPE_TYPE_INVALID);
}

abi_shape_validate :: proc "contextless" (
	type_id: Entasis_Shape_Type_ID,
	shape_value: rawptr,
) -> Entasis_Status
{
	if shape_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	status: entasis.Status;
	switch type_id
	{
		case i32(entasis.SHAPE_TYPE_SPHERE):
		status = entasis.shape_validate(cast(entasis.Sphere)(^Entasis_Sphere)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_CAPSULE):
		status = entasis.shape_validate(cast(entasis.Capsule)(^Entasis_Capsule)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_BOX):
		status = entasis.shape_validate(cast(entasis.Box)(^Entasis_Box)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_TRIANGLE):
		status = entasis.shape_validate(abi_triangle_to_core((^Entasis_Triangle)(shape_value)^));
		case i32(entasis.SHAPE_TYPE_CYLINDER):
		status = entasis.shape_validate(cast(entasis.Cylinder)(^Entasis_Cylinder)(shape_value)^);
		case:
		status = .Invalid_Argument;
	}
	return abi_status(status);
}

abi_shape_add :: proc "contextless" (
	world: ^Entasis_World, type_id: Entasis_Shape_Type_ID,
	shape_value: rawptr, out_handle: ^Entasis_Shape_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_shape_handle_invalid();
	}
	if shape_value == nil || out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	handle: entasis.Shape_Handle;
	status: entasis.Status;
	switch type_id
	{
		case i32(entasis.SHAPE_TYPE_SPHERE):
		handle, status = entasis.shape_add(&resource.world, cast(entasis.Sphere)(^Entasis_Sphere)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_CAPSULE):
		handle, status = entasis.shape_add(&resource.world, cast(entasis.Capsule)(^Entasis_Capsule)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_BOX):
		handle, status = entasis.shape_add(&resource.world, cast(entasis.Box)(^Entasis_Box)(shape_value)^);
		case i32(entasis.SHAPE_TYPE_TRIANGLE):
		handle, status = entasis.shape_add(
			&resource.world,
			abi_triangle_to_core((^Entasis_Triangle)(shape_value)^)
		);
		case i32(entasis.SHAPE_TYPE_CYLINDER):
		handle, status = entasis.shape_add(
			&resource.world,
			cast(entasis.Cylinder)(^Entasis_Cylinder)(shape_value)^
		);
		case:
		status = .Invalid_Argument;
	}
	if status == .Ok
	{
		out_handle^ = abi_shape_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_shape_add_batch :: proc "contextless" (
	world: ^Entasis_World, type_ids: [^]Entasis_Shape_Type_ID,
	shape_values: rawptr, count: u64,
	out_handles: [^]Entasis_Shape_Handle, out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && (type_ids == nil || shape_values == nil || out_handles == nil))
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	values := ([^]rawptr)(shape_values);
	for index in 0 ..< int(count)
	{
		status := abi_shape_add(world, type_ids[index], values[index], &out_handles[index], diagnostic);
		if status != abi_status(.Ok)
		{
			abi_diagnostic_record(
				diagnostic,
				status,
				Entasis_Diagnostic_Operation(entasis.Diagnostic_Operation.Shape_Register),
				i32(index)
			);
			return status;
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_shape_inertia :: proc "contextless" (
	type_id: Entasis_Shape_Type_ID, shape_value: rawptr,
	mass: f32, out_inertia: ^Entasis_Body_Inertia,
) -> Entasis_Status
{
	if out_inertia == nil || shape_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_inertia^ = {};
	inertia: entasis.Body_Inertia;
	status: entasis.Status;
	switch type_id
	{
		case i32(entasis.SHAPE_TYPE_SPHERE):
		inertia, status = entasis.shape_inertia(cast(entasis.Sphere)(^Entasis_Sphere)(shape_value)^, mass);
		case i32(entasis.SHAPE_TYPE_CAPSULE):
		inertia, status = entasis.shape_inertia(cast(entasis.Capsule)(^Entasis_Capsule)(shape_value)^, mass);
		case i32(entasis.SHAPE_TYPE_BOX):
		inertia, status = entasis.shape_inertia(cast(entasis.Box)(^Entasis_Box)(shape_value)^, mass);
		case i32(entasis.SHAPE_TYPE_TRIANGLE):
		inertia, status = entasis.shape_inertia(abi_triangle_to_core((^Entasis_Triangle)(shape_value)^), mass);
		case i32(entasis.SHAPE_TYPE_CYLINDER):
		inertia, status = entasis.shape_inertia(cast(entasis.Cylinder)(^Entasis_Cylinder)(shape_value)^, mass);
		case:
		status = .Invalid_Argument;
	}
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
	}
	return abi_status(status);
}

abi_shape_remove :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.shape_remove(
			&resource.world, abi_shape_handle_to_core(handle),
		), diagnostic, .Shape_Register);
}

abi_shape_remove_batch :: proc "contextless" (
	world: ^Entasis_World, handles: [^]Entasis_Shape_Handle,
	count: u64, out_completed: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if !abi_count_valid(count) || (count > 0 && handles == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	for index in 0 ..< int(count)
	{
		status := entasis.shape_remove(&resource.world, abi_shape_handle_to_core(handles[index]));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .Shape_Register, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_shape_inspect :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	out_info: ^Entasis_Shape_Info, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_info == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	out_info^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	info, status := entasis.shape_inspect(&resource.world, abi_shape_handle_to_core(handle));
	if status == .Ok
	{
		out_info^ = {
			handle=abi_shape_handle_from_core(info.handle),
			type_id=i32(info.type_id),
			value_size=u64(info.value_size),
			value_alignment=u64(info.value_alignment),
			reference_count=u64(info.reference_count),
		};
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_shape_bounds :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	orientation: Entasis_Quaternion, out_bounds: ^Entasis_Shape_Bounds,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_bounds == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	out_bounds^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	bounds, status := entasis.shape_bounds(
		&resource.world, abi_shape_handle_to_core(handle), cast(entasis.Quaternion)orientation,
	);
	if status == .Ok
	{
		out_bounds^ = transmute(Entasis_Shape_Bounds)bounds;
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_shape_ray :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	shape_pose: Entasis_Rigid_Pose, query: Entasis_Ray,
	out_hit: ^Entasis_Shape_Ray_Hit, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_hit == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	out_hit^ = {child_index=-1};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register, access_mode=.Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	hit, status := entasis.shape_ray(
		&resource.world, abi_shape_handle_to_core(handle), abi_pose_to_core(shape_pose), abi_ray_to_core(query),
	);
	if status == .Ok
	{
		out_hit^ = {
			t=hit.t,
			normal=abi_vector3_from_core(hit.normal),
			child_index=hit.child_index,
			hit=abi_bool(hit.state == .Present),
		};
	}
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_shape_remove_recursive :: proc "contextless" (
	world: ^Entasis_World, root: Entasis_Shape_Handle,
	scratch: [^]Entasis_Shape_Handle, scratch_count: u64,
	out_removed, out_required: ^u64, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_removed != nil
	{
		out_removed^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if !abi_count_valid(scratch_count) ||
	out_removed == nil ||
	out_required == nil ||
	(scratch_count > 0 && scratch == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_scratch := (cast([^]entasis.Shape_Handle)scratch)[:int(scratch_count)];
	removed, required, status := entasis.shape_remove_recursive(
		&resource.world, abi_shape_handle_to_core(root), core_scratch,
	);
	out_removed^ = u64(max(removed, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .Shape_Register);
}

abi_compound_child :: #force_inline proc "contextless" (
	shape: Entasis_Shape_Handle, local_pose: Entasis_Rigid_Pose,
) -> Entasis_Compound_Child
{
	return {local_position=local_pose.position, local_orientation=local_pose.orientation, shape=shape};
}

abi_compound_child_to_core :: #force_inline proc "contextless" (
	value: Entasis_Compound_Child,
) -> entasis.Compound_Child
{
	return entasis.Compound_Child{
		local_position=abi_vector3_to_core(value.local_position),
		local_orientation=cast(entasis.Quaternion)value.local_orientation,
		shape_index=abi_shape_handle_to_core(value.shape),
	};
}

abi_compound_builder :: #force_inline proc "contextless" (
	children: [^]Entasis_Compound_Child, masses: [^]f32, count: u64,
) -> Entasis_Compound_Builder
{
	return {children=children, masses=masses, count=count};
}

abi_compound_builder_to_core :: proc "contextless" (
	builder: Entasis_Compound_Builder,
) -> (entasis.Compound_Builder, entasis.Status)
{
	if !abi_count_valid(builder.count) || builder.count == 0 || builder.children == nil || builder.masses == nil
	{
		return {}, .Invalid_Description;
	}
	// ABI child layout is intentionally identical to the facade compound child
	children := (cast([^]entasis.Compound_Child)builder.children)[:int(builder.count)];
	masses := builder.masses[:int(builder.count)];
	return entasis.compound_builder(children, masses), .Ok;
}

abi_compound_center_of_mass :: proc "contextless" (
	builder: Entasis_Compound_Builder, out_center: ^Entasis_Vector3,
	out_inverse_mass: ^f32,
) -> Entasis_Status
{
	if out_center == nil || out_inverse_mass == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_center^ = {};
	out_inverse_mass^ = 0;
	core, convert_status := abi_compound_builder_to_core(builder);
	if convert_status != .Ok
	{
		return abi_status(convert_status);
	}
	center, inverse_mass, status := entasis.compound_center_of_mass(core);
	if status == .Ok
	{
		out_center^ = abi_vector3_from_core(center);
		out_inverse_mass^ = inverse_mass;
	}
	return abi_status(status);
}

abi_compound_inertia_weighted :: proc "contextless" (
	world: ^Entasis_World, builder: Entasis_Compound_Builder,
	out_inertia: ^Entasis_Body_Inertia, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_inertia == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_inertia^ = {};
	core, convert_status := abi_compound_builder_to_core(builder);
	if convert_status != .Ok
	{
		return abi_status_finish(convert_status, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	inertia, status := entasis.compound_inertia_weighted(&resource.world, core);
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_compound_inertia_weighted_recenter :: proc "contextless" (
	world: ^Entasis_World, builder: Entasis_Compound_Builder,
	out_inertia: ^Entasis_Body_Inertia, out_center: ^Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_inertia == nil || out_center == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_inertia^ = {};
	out_center^ = {};
	core, convert_status := abi_compound_builder_to_core(builder);
	if convert_status != .Ok
	{
		return abi_status_finish(convert_status, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	inertia, center, status := entasis.compound_inertia_weighted_recenter(&resource.world, core);
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
		out_center^ = abi_vector3_from_core(center);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_compound_build_result_from_core :: #force_inline proc "contextless" (
	value: entasis.Compound_Build_Result,
) -> Entasis_Compound_Build_Result
{
	return {
		shape=abi_shape_handle_from_core(value.shape),
		inertia=abi_inertia_from_core(value.inertia),
		center=abi_vector3_from_core(value.center),
	};
}

abi_compound_build_dynamic :: proc "contextless" (
	world: ^Entasis_World, builder: Entasis_Compound_Builder,
	recenter: Entasis_Bool, out_result: ^Entasis_Compound_Build_Result,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_result == nil || recenter > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_result^ = {shape=abi_shape_handle_invalid()};
	core, convert_status := abi_compound_builder_to_core(builder);
	if convert_status != .Ok
	{
		return abi_status_finish(convert_status, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	result, status := entasis.compound_build_dynamic(&resource.world, core, recenter != 0);
	if status == .Ok
	{
		out_result^ = abi_compound_build_result_from_core(result);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_big_compound_build_dynamic :: proc "contextless" (
	world: ^Entasis_World, builder: Entasis_Compound_Builder,
	recenter: Entasis_Bool, out_result: ^Entasis_Compound_Build_Result,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_result == nil || recenter > 1
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_result^ = {shape=abi_shape_handle_invalid()};
	core, convert_status := abi_compound_builder_to_core(builder);
	if convert_status != .Ok
	{
		return abi_status_finish(convert_status, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	result, status := entasis.big_compound_build_dynamic(&resource.world, core, recenter != 0);
	if status == .Ok
	{
		out_result^ = abi_compound_build_result_from_core(result);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_shape_import_compound :: proc "contextless" (
	world: ^Entasis_World, children: [^]Entasis_Compound_Child,
	count: u64, out_handle: ^Entasis_Shape_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_shape_handle_invalid();
	}
	if !abi_count_valid(count) || count == 0 || children == nil || out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_children := (cast([^]entasis.Compound_Child)children)[:int(count)];
	handle, status := entasis.shape_import_compound(&resource.world, core_children);
	if status == .Ok
	{
		out_handle^ = abi_shape_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_shape_import_big_compound :: proc "contextless" (
	world: ^Entasis_World, children: [^]Entasis_Compound_Child,
	count: u64, out_handle: ^Entasis_Shape_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_handle != nil
	{
		out_handle^ = abi_shape_handle_invalid();
	}
	if !abi_count_valid(count) || count == 0 || children == nil || out_handle == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	core_children := (cast([^]entasis.Compound_Child)children)[:int(count)];
	handle, status := entasis.shape_import_big_compound(&resource.world, core_children);
	if status == .Ok
	{
		out_handle^ = abi_shape_handle_from_core(handle);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_mesh_closed_inertia :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	mass: f32, out_inertia: ^Entasis_Body_Inertia,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_inertia == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_inertia^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	inertia, status := entasis.mesh_closed_inertia(&resource.world, abi_shape_handle_to_core(handle), mass);
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_mesh_open_inertia :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	mass: f32, out_inertia: ^Entasis_Body_Inertia,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_inertia == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_inertia^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	inertia, status := entasis.mesh_open_inertia(&resource.world, abi_shape_handle_to_core(handle), mass);
	if status == .Ok
	{
		out_inertia^ = abi_inertia_from_core(inertia);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_mesh_closed_center_of_mass :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	out_volume: ^f32, out_center: ^Entasis_Vector3,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_volume == nil || out_center == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_volume^ = 0;
	out_center^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	volume, center, status := entasis.mesh_closed_center_of_mass(&resource.world, abi_shape_handle_to_core(handle));
	if status == .Ok
	{
		out_volume^ = volume;
		out_center^ = abi_vector3_from_core(center);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}

abi_mesh_open_center_of_mass :: proc "contextless" (
	world: ^Entasis_World, handle: Entasis_Shape_Handle,
	out_center: ^Entasis_Vector3, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_center == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .Asset_Import);
	}
	out_center^ = {};
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Asset_Import);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	center, status := entasis.mesh_open_center_of_mass(&resource.world, abi_shape_handle_to_core(handle));
	if status == .Ok
	{
		out_center^ = abi_vector3_from_core(center);
	}
	return abi_status_finish(status, diagnostic, .Asset_Import);
}
#assert(size_of(Entasis_Sphere) == size_of(entasis.Sphere));
#assert(size_of(Entasis_Capsule) == size_of(entasis.Capsule));
#assert(size_of(Entasis_Box) == size_of(entasis.Box));
#assert(size_of(Entasis_Triangle) == size_of(entasis.Triangle));
#assert(size_of(Entasis_Cylinder) == size_of(entasis.Cylinder));
#assert(size_of(Entasis_Shape_Bounds) == size_of(entasis.Shape_Bounds));
#assert(size_of(Entasis_Compound_Child) == size_of(entasis.Compound_Child));
