package entasis_c

import shared "entasis:entasis_c_shared"
import "base:runtime"
import entasis "entasis:entasis"

ENTASIS_COLLISION_PROPERTY_MAGIC :: u64(0x454E54434F4C5031);
ENTASIS_NARROW_POLICY_DEFAULT_KIND :: Entasis_Narrow_Policy_Kind(0);
ENTASIS_NARROW_POLICY_LAYERS_MATERIALS_KIND :: Entasis_Narrow_Policy_Kind(1);
abi_material_to_core_value :: #force_inline proc "contextless" (value: Entasis_Material) -> entasis.Material
{
	return {
		friction=value.friction,
		maximum_recovery_velocity=value.maximum_recovery_velocity,
		spring=abi_spring_to_core(value.spring),
	};
}

abi_material_from_core_value :: #force_inline proc "contextless" (value: entasis.Material) -> Entasis_Material
{
	return {
		friction=value.friction,
		maximum_recovery_velocity=value.maximum_recovery_velocity,
		spring=abi_spring_from_core(value.spring),
	};
}

abi_material_id :: proc "contextless" (index: u32, out_id: ^Entasis_Material_ID) -> Entasis_Status
{
	if out_id == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_id^ = ENTASIS_MATERIAL_ID_INVALID;
	id, status := entasis.material_id(int(index));
	if status == .Ok
	{
		out_id^ = u16(id);
	}
	return abi_status(status);
}

abi_material_id_invalid :: #force_inline proc "contextless" () -> Entasis_Material_ID
{
	return u16(entasis.material_id_invalid());
}

abi_material_id_is_valid :: #force_inline proc "contextless" (id: Entasis_Material_ID) -> Entasis_Bool
{
	return abi_bool(entasis.material_id_is_valid(entasis.Material_ID(id)));
}

abi_material :: #force_inline proc "contextless" (
	friction, maximum_recovery_velocity: f32, spring: Entasis_Spring_Settings,
) -> Entasis_Material
{
	return abi_material_from_core_value(entasis.material(
			friction,
			maximum_recovery_velocity,
			abi_spring_to_core(spring)
	));
}

abi_material_default :: #force_inline proc "contextless" () -> Entasis_Material
{
	return abi_material_from_core_value(entasis.material_default());
}

abi_material_table :: #force_inline proc "contextless" (
	values: [^]Entasis_Material, count: u64,
) -> Entasis_Material_Table
{
	if !abi_count_valid(count) || (count > 0 && values == nil)
	{
		return {};
	}
	return {values=values, count=count};
}

abi_material_table_to_core :: #force_inline proc "contextless" (
	table: Entasis_Material_Table,
) -> (entasis.Material_Table, entasis.Status)
{
	if !abi_count_valid(table.count) || (table.count > 0 && table.values == nil)
	{
		return {}, .Invalid_Argument;
	}
	return entasis.material_table((cast([^]entasis.Material)table.values)[:int(table.count)]), .Ok;
}

abi_material_table_get :: proc "contextless" (
	table: Entasis_Material_Table, id: Entasis_Material_ID,
	out_value: ^Entasis_Material,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_value^ = {};
	core, convert_status := abi_material_table_to_core(table);
	if convert_status != .Ok
	{
		return abi_status(convert_status);
	}
	value, status := entasis.material_table_get(core, entasis.Material_ID(id));
	if status == .Ok
	{
		out_value^ = abi_material_from_core_value(value^);
	}
	return abi_status(status);
}

abi_material_to_contact :: #force_inline proc "contextless" (value: Entasis_Material) -> Entasis_Contact_Material
{
	return abi_material_from_core(entasis.material_to_contact(abi_material_to_core_value(value)));
}

abi_material_validate :: #force_inline proc "contextless" (value: Entasis_Material) -> Entasis_Status
{
	return abi_status(entasis.material_validate(abi_material_to_core_value(value)));
}

abi_material_combine_default :: #force_inline proc "contextless" (a, b: Entasis_Material) -> Entasis_Material
{
	return abi_material_from_core_value(entasis.material_combine_default(
			abi_material_to_core_value(a),
			abi_material_to_core_value(b)
	));
}

abi_collision_filter_to_core :: #force_inline proc "contextless" (value: Entasis_Collision_Filter) -> entasis.Collision_Filter
{
	return {layer=entasis.Collision_Layer(value.layer), mask=entasis.Layer_Mask(value.mask)};
}

abi_collision_filter_from_core :: #force_inline proc "contextless" (value: entasis.Collision_Filter) -> Entasis_Collision_Filter
{
	return {mask=u64(value.mask), layer=u8(value.layer)};
}

abi_collision_properties_to_core :: #force_inline proc "contextless" (value: Entasis_Collision_Properties) -> entasis.Collision_Properties
{
	return {
		mask=entasis.Layer_Mask(value.mask),
		material_id=entasis.Material_ID(value.material_id),
		layer=entasis.Collision_Layer(value.layer),
	};
}

abi_collision_properties_from_core :: #force_inline proc "contextless" (value: entasis.Collision_Properties) -> Entasis_Collision_Properties
{
	return {mask=u64(value.mask), material_id=u16(value.material_id), layer=u8(value.layer)};
}

abi_collision_layer :: proc "contextless" (index: u32, out_layer: ^Entasis_Collision_Layer) -> Entasis_Status
{
	if out_layer == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_layer^ = 0;
	layer, status := entasis.collision_layer(int(index));
	if status == .Ok
	{
		out_layer^ = u8(layer);
	}
	return abi_status(status);
}

abi_collision_layer_index :: #force_inline proc "contextless" (layer: Entasis_Collision_Layer) -> u32
{
	return u32(entasis.collision_layer_index(entasis.Collision_Layer(layer)));
}

abi_collision_layer_is_valid :: #force_inline proc "contextless" (layer: Entasis_Collision_Layer) -> Entasis_Bool
{
	return abi_bool(entasis.collision_layer_is_valid(entasis.Collision_Layer(layer)));
}

abi_layer_mask :: #force_inline proc "contextless" (layer: Entasis_Collision_Layer) -> Entasis_Layer_Mask
{
	return u64(entasis.layer_mask(entasis.Collision_Layer(layer)));
}

abi_layer_mask_none :: #force_inline proc "contextless" () -> Entasis_Layer_Mask
{
	return u64(entasis.layer_mask_none());
}

abi_layer_mask_all :: #force_inline proc "contextless" () -> Entasis_Layer_Mask
{
	return u64(entasis.layer_mask_all());
}

abi_layer_mask_add :: #force_inline proc "contextless" (
	mask: Entasis_Layer_Mask,
	layer: Entasis_Collision_Layer
) -> Entasis_Layer_Mask
{
	return u64(entasis.layer_mask_add(entasis.Layer_Mask(mask), entasis.Collision_Layer(layer)));
}

abi_layer_mask_remove :: #force_inline proc "contextless" (
	mask: Entasis_Layer_Mask,
	layer: Entasis_Collision_Layer
) -> Entasis_Layer_Mask
{
	return u64(entasis.layer_mask_remove(entasis.Layer_Mask(mask), entasis.Collision_Layer(layer)));
}

abi_layer_mask_contains :: #force_inline proc "contextless" (
	mask: Entasis_Layer_Mask,
	layer: Entasis_Collision_Layer
) -> Entasis_Bool
{
	return abi_bool(entasis.layer_mask_contains(entasis.Layer_Mask(mask), entasis.Collision_Layer(layer)));
}

abi_collision_filter :: #force_inline proc "contextless" (
	layer: Entasis_Collision_Layer,
	mask: Entasis_Layer_Mask
) -> Entasis_Collision_Filter
{
	return abi_collision_filter_from_core(entasis.collision_filter(
			entasis.Collision_Layer(layer),
			entasis.Layer_Mask(mask)
	));
}

abi_collision_filter_default :: #force_inline proc "contextless" () -> Entasis_Collision_Filter
{
	return abi_collision_filter_from_core(entasis.collision_filter_default());
}

abi_collision_filter_allows :: #force_inline proc "contextless" (
	a, b: Entasis_Collision_Filter, layer_matrix: ^Entasis_Layer_Matrix,
) -> Entasis_Bool
{
	core_matrix: ^entasis.Layer_Matrix;
	if layer_matrix != nil
	{
		core_matrix = cast(^entasis.Layer_Matrix)layer_matrix;
	}
	return abi_bool(entasis.collision_filter_allows(
			abi_collision_filter_to_core(a),
			abi_collision_filter_to_core(b),
			core_matrix
	));
}

abi_collision_properties :: #force_inline proc "contextless" (
	filter: Entasis_Collision_Filter, material_id: Entasis_Material_ID,
) -> Entasis_Collision_Properties
{
	return abi_collision_properties_from_core(entasis.collision_properties(
			abi_collision_filter_to_core(filter),
			entasis.Material_ID(material_id)
	));
}

abi_collision_properties_default :: #force_inline proc "contextless" () -> Entasis_Collision_Properties
{
	return abi_collision_properties_from_core(entasis.collision_properties_default());
}

abi_collision_properties_filter :: #force_inline proc "contextless" (value: Entasis_Collision_Properties) -> Entasis_Collision_Filter
{
	return abi_collision_filter_from_core(entasis.collision_properties_filter(abi_collision_properties_to_core(value)));
}

abi_collision_properties_validate :: #force_inline proc "contextless" (value: Entasis_Collision_Properties) -> Entasis_Status
{
	return abi_status(entasis.collision_properties_validate(abi_collision_properties_to_core(value)));
}

abi_layer_matrix_none :: #force_inline proc "contextless" () -> Entasis_Layer_Matrix
{
	return transmute(Entasis_Layer_Matrix)entasis.layer_matrix_none();
}

abi_layer_matrix_all :: #force_inline proc "contextless" () -> Entasis_Layer_Matrix
{
	return transmute(Entasis_Layer_Matrix)entasis.layer_matrix_all();
}

abi_layer_matrix_set :: proc "contextless" (
	matrix_: ^Entasis_Layer_Matrix, a, b: Entasis_Collision_Layer, allow: Entasis_Bool,
) -> Entasis_Status
{
	if allow > 1
	{
		return abi_status(.Invalid_Argument);
	}
	return abi_status(entasis.layer_matrix_set(
			cast(^entasis.Layer_Matrix)matrix_,
			entasis.Collision_Layer(a),
			entasis.Collision_Layer(b),
			allow != 0
	));
}

abi_layer_matrix_allow :: proc "contextless" (
	matrix_: ^Entasis_Layer_Matrix, a, b: Entasis_Collision_Layer,
) -> Entasis_Status
{
	return abi_status(entasis.layer_matrix_allow(
			cast(^entasis.Layer_Matrix)matrix_,
			entasis.Collision_Layer(a),
			entasis.Collision_Layer(b)
	));
}

abi_layer_matrix_deny :: proc "contextless" (
	matrix_: ^Entasis_Layer_Matrix, a, b: Entasis_Collision_Layer,
) -> Entasis_Status
{
	return abi_status(entasis.layer_matrix_deny(
			cast(^entasis.Layer_Matrix)matrix_,
			entasis.Collision_Layer(a),
			entasis.Collision_Layer(b)
	));
}

abi_layer_matrix_allows :: #force_inline proc "contextless" (
	matrix_: ^Entasis_Layer_Matrix, a, b: Entasis_Collision_Layer,
) -> Entasis_Bool
{
	return abi_bool(entasis.layer_matrix_allows(
			cast(^entasis.Layer_Matrix)matrix_,
			entasis.Collision_Layer(a),
			entasis.Collision_Layer(b)
	));
}
#assert(size_of(Entasis_Material) == size_of(entasis.Material));
#assert(size_of(Entasis_Layer_Matrix) == size_of(entasis.Layer_Matrix));
abi_collision_property_resource :: struct
{
	magic:           u64,
	allocator:       shared.Allocator,
	table:           entasis.Collision_Property_Table,
	attached_worlds: i32,
	reserved:        i32,
}

abi_collision_property_get_resource :: #force_inline proc "contextless" (
	handle: ^Entasis_Collision_Property_Table,
) -> ^abi_collision_property_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_collision_property_resource)(handle.opaque);
	if resource.magic != ENTASIS_COLLISION_PROPERTY_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_collision_property_init :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	body_capacity, static_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if table == nil || table.opaque != nil || !abi_count_valid(body_capacity) || !abi_count_valid(static_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	memory, allocator_copy, allocation_status := abi_resource_allocate(
		size_of(abi_collision_property_resource), align_of(abi_collision_property_resource), allocator,
	);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .World_Initialize);
	}
	resource := (^abi_collision_property_resource)(memory);
	resource.magic = ENTASIS_COLLISION_PROPERTY_MAGIC;
	resource.allocator = allocator_copy;
	core_allocator, allocator_status := abi_allocator_to_core(&resource.allocator);
	if allocator_status != .Ok
	{
		resource.magic = 0;
		abi_resource_free(
			resource,
			size_of(abi_collision_property_resource),
			align_of(abi_collision_property_resource),
			&resource.allocator
		);
		return abi_status_finish(allocator_status, diagnostic, .World_Initialize);
	}
	context = runtime.default_context();
	status := entasis.collision_property_init(
		&resource.table, int(body_capacity), int(static_capacity), core_allocator,
	);
	if status != .Ok
	{
		allocator_copy = resource.allocator;
		resource.magic = 0;
		abi_resource_free(
			resource,
			size_of(abi_collision_property_resource),
			align_of(abi_collision_property_resource),
			&allocator_copy
		);
		return abi_status_finish(status, diagnostic, .World_Initialize);
	}
	table.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .World_Initialize);
}

abi_collision_property_ensure_capacity :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	body_capacity, static_capacity: u64,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	if !abi_count_valid(body_capacity) || !abi_count_valid(static_capacity)
	{
		return abi_status(.Invalid_Argument);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_ensure_capacity(
			&resource.table,
			int(body_capacity),
			int(static_capacity)
	));
}

abi_collision_property_set_body :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	handle: Entasis_Body_Handle,
	value: Entasis_Collision_Properties,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_set_body(
			&resource.table,
			abi_body_handle_to_core(handle),
			abi_collision_properties_to_core(value)
	));
}

abi_collision_property_get_body :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	handle: Entasis_Body_Handle,
	out_value: ^Entasis_Collision_Properties,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_value^ = {};
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	value, status := entasis.collision_property_get_body(&resource.table, abi_body_handle_to_core(handle));
	if status == .Ok
	{
		out_value^ = abi_collision_properties_from_core(value^);
	}
	return abi_status(status);
}

abi_collision_property_set_static :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	handle: Entasis_Static_Handle,
	value: Entasis_Collision_Properties,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_set_static(
			&resource.table,
			abi_static_handle_to_core(handle),
			abi_collision_properties_to_core(value)
	));
}

abi_collision_property_get_static :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	handle: Entasis_Static_Handle,
	out_value: ^Entasis_Collision_Properties,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_value^ = {};
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	value, status := entasis.collision_property_get_static(&resource.table, abi_static_handle_to_core(handle));
	if status == .Ok
	{
		out_value^ = abi_collision_properties_from_core(value^);
	}
	return abi_status(status);
}

abi_collidable_reference_to_core :: #force_inline proc "contextless" (
	value: Entasis_Collidable_Reference,
) -> entasis.Collidable_Reference
{
	return {packed=value.packed};
}

abi_collision_property_set :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	collidable: Entasis_Collidable_Reference,
	value: Entasis_Collision_Properties,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_set(
			&resource.table,
			abi_collidable_reference_to_core(collidable),
			abi_collision_properties_to_core(value)
	));
}

abi_collision_property_get :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	collidable: Entasis_Collidable_Reference,
	out_value: ^Entasis_Collision_Properties,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_value^ = {};
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	value, status := entasis.collision_property_get(&resource.table, abi_collidable_reference_to_core(collidable));
	if status == .Ok
	{
		out_value^ = abi_collision_properties_from_core(value^);
	}
	return abi_status(status);
}

abi_collision_property_set_batch :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	collidables: [^]Entasis_Collidable_Reference,
	values: [^]Entasis_Collision_Properties,
	count: u64,
	out_completed: ^u64,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if out_completed == nil || !abi_count_valid(count) || (count > 0 && (collidables == nil || values == nil))
	{
		return abi_status(.Invalid_Argument);
	}
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	core_collidables := (cast([^]entasis.Collidable_Reference)collidables)[:int(count)];
	core_values := (cast([^]entasis.Collision_Properties)values)[:int(count)];
	context = runtime.default_context();
	completed, status := entasis.collision_property_set_batch(&resource.table, core_collidables, core_values);
	out_completed^ = u64(max(completed, 0));
	return abi_status(status);
}

abi_collision_property_remove_body :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table, handle: Entasis_Body_Handle,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_remove_body(&resource.table, abi_body_handle_to_core(handle)));
}

abi_collision_property_remove_static :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table, handle: Entasis_Static_Handle,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_remove_static(&resource.table, abi_static_handle_to_core(handle)));
}

abi_collision_property_remove :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table, collidable: Entasis_Collidable_Reference,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_remove(&resource.table, abi_collidable_reference_to_core(collidable)));
}

abi_collision_property_remove_batch :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
	collidables: [^]Entasis_Collidable_Reference,
	count: u64,
	out_completed: ^u64,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	if out_completed == nil || !abi_count_valid(count) || (count > 0 && collidables == nil)
	{
		return abi_status(.Invalid_Argument);
	}
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	core_collidables := (cast([^]entasis.Collidable_Reference)collidables)[:int(count)];
	context = runtime.default_context();
	completed, status := entasis.collision_property_remove_batch(&resource.table, core_collidables);
	out_completed^ = u64(max(completed, 0));
	return abi_status(status);
}

abi_collision_property_clear :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	context = runtime.default_context();
	return abi_status(entasis.collision_property_clear(&resource.table));
}

abi_collision_property_destroy :: proc "contextless" (
	table: ^Entasis_Collision_Property_Table,
) -> Entasis_Status
{
	resource := abi_collision_property_get_resource(table);
	if resource == nil
	{
		return abi_status(.Disposed);
	}
	if resource.attached_worlds != 0
	{
		return abi_status(.Shape_In_Use);
	}
	context = runtime.default_context();
	status := entasis.collision_property_destroy(&resource.table);
	if status != .Ok
	{
		return abi_status(status);
	}
	allocator := resource.allocator;
	resource.magic = 0;
	table.opaque = nil;
	abi_resource_free(
		resource,
		size_of(abi_collision_property_resource),
		align_of(abi_collision_property_resource),
		&allocator
	);
	return abi_status(.Ok);
}

abi_layer_material_context :: struct
{
	public_policy:    Entasis_Layer_Material_Policy,
	core_policy:      entasis.Layer_Material_Policy,
	property_resource: ^abi_collision_property_resource,
}

abi_pair_filter_bridge :: proc "contextless" (
	user_context: rawptr, a, b: entasis.Collidable_Reference,
) -> bool
{
	if user_context == nil
	{
		return false;
	}
	bridge := (^abi_layer_material_context)(user_context);
	callback := bridge.public_policy.pair_filter;
	if callback == nil
	{
		return true;
	}
	return callback(
		bridge.public_policy.user_context,
		{packed=a.packed}, {packed=b.packed},
	) != 0;
}

abi_child_filter_bridge :: proc "contextless" (
	user_context: rawptr, a, b: entasis.Collidable_Reference, child_a, child_b: i32,
) -> bool
{
	if user_context == nil
	{
		return false;
	}
	bridge := (^abi_layer_material_context)(user_context);
	callback := bridge.public_policy.child_filter;
	if callback == nil
	{
		return true;
	}
	return callback(
		bridge.public_policy.user_context,
		{packed=a.packed}, {packed=b.packed}, child_a, child_b,
	) != 0;
}

abi_material_combine_bridge :: proc "contextless" (
	user_context: rawptr,
	a_id: entasis.Material_ID, a: entasis.Material,
	b_id: entasis.Material_ID, b: entasis.Material,
) -> entasis.Material
{
	default_value := entasis.material_combine_default(a, b);
	if user_context == nil
	{
		return default_value;
	}
	bridge := (^abi_layer_material_context)(user_context);
	callback := bridge.public_policy.material_combine;
	if callback == nil
	{
		return default_value;
	}
	abi_a := abi_material_from_core_value(a);
	abi_b := abi_material_from_core_value(b);
	out := abi_material_from_core_value(default_value);
	accepted := callback(
		bridge.public_policy.user_context,
		u16(a_id), &abi_a, u16(b_id), &abi_b, &out,
	);
	if accepted == 0
	{
		return default_value;
	}
	candidate := abi_material_to_core_value(out);
	if entasis.material_validate(candidate) != .Ok
	{
		return default_value;
	}
	return candidate;
}

abi_layer_material_policy :: proc "contextless" (
	properties: ^Entasis_Collision_Property_Table,
	layer_matrix: ^Entasis_Layer_Matrix,
	materials: Entasis_Material_Table,
) -> Entasis_Layer_Material_Policy
{
	return {
		struct_size=u32(size_of(Entasis_Layer_Material_Policy)),
		struct_version=ENTASIS_STRUCT_VERSION,
		properties=properties,
		layer_matrix=layer_matrix,
		materials=materials,
		default_properties=abi_collision_properties_default(),
		fallback_material=abi_material_default(),
	};
}

abi_narrow_policy_layers_materials :: proc "contextless" (
	policy: ^Entasis_Layer_Material_Policy,
) -> Entasis_Narrow_Policy
{
	return {
		struct_size=u32(size_of(Entasis_Narrow_Policy)),
		struct_version=ENTASIS_STRUCT_VERSION,
		kind=ENTASIS_NARROW_POLICY_LAYERS_MATERIALS_KIND,
		layer_material_policy=policy,
	};
}

abi_world_description_set_narrow_policy :: proc "contextless" (
	description: ^Entasis_World_Description,
	narrow: ^Entasis_Narrow_Policy,
) -> Entasis_Status
{
	if description == nil ||
	description.struct_size < u32(size_of(Entasis_World_Description)) ||
	description.struct_version != ENTASIS_STRUCT_VERSION
	{
		return abi_status(.Invalid_Argument);
	}
	if narrow != nil &&
	(narrow.struct_size < u32(size_of(Entasis_Narrow_Policy)) ||
		narrow.struct_version != ENTASIS_STRUCT_VERSION)
	{
		return abi_status(.Invalid_Description);
	}
	description.narrow_policy_v1 = narrow;
	description.use_narrow_policy = ENTASIS_FALSE;
	return abi_status(.Ok);
}

abi_layer_material_context_build :: proc "contextless" (
	bridge: ^abi_layer_material_context,
	public_policy: ^Entasis_Layer_Material_Policy,
) -> entasis.Status
{
	if bridge == nil || public_policy == nil ||
	public_policy.struct_size < u32(size_of(Entasis_Layer_Material_Policy)) ||
	public_policy.struct_version != ENTASIS_STRUCT_VERSION
	{
		return .Invalid_Description;
	}
	material_table, table_status := abi_material_table_to_core(public_policy.materials);
	if table_status != .Ok
	{
		return table_status;
	}
	property_resource: ^abi_collision_property_resource;
	if public_policy.properties != nil
	{
		property_resource = abi_collision_property_get_resource(public_policy.properties);
		if property_resource == nil
		{
			return .Disposed;
		}
	}
	if entasis.collision_properties_validate(abi_collision_properties_to_core(public_policy.default_properties)) != .Ok ||
	entasis.material_validate(abi_material_to_core_value(public_policy.fallback_material)) != .Ok
	{
		return .Invalid_Description;
	}
	bridge.public_policy = public_policy^;
	bridge.property_resource = property_resource;
	properties: ^entasis.Collision_Property_Table;
	if property_resource != nil
	{
		properties = &property_resource.table;
	}
	layer_rules: ^entasis.Layer_Matrix;
	if public_policy.layer_matrix != nil
	{
		layer_rules = cast(^entasis.Layer_Matrix)public_policy.layer_matrix;
	}
	bridge.core_policy = entasis.layer_material_policy(properties, layer_rules, material_table);
	bridge.core_policy.default_properties = abi_collision_properties_to_core(public_policy.default_properties);
	bridge.core_policy.fallback_material = abi_material_to_core_value(public_policy.fallback_material);
	if public_policy.pair_filter != nil
	{
		bridge.core_policy.pair_filter = abi_pair_filter_bridge;
	}
	if public_policy.child_filter != nil
	{
		bridge.core_policy.child_filter = abi_child_filter_bridge;
	}
	if public_policy.material_combine != nil
	{
		bridge.core_policy.material_combine = abi_material_combine_bridge;
	}
	bridge.core_policy.user_context = bridge;
	return .Ok;
}
#assert(size_of(Entasis_Collidable_Reference) == size_of(entasis.Collidable_Reference));
#assert(size_of(Entasis_Collision_Properties) == size_of(entasis.Collision_Properties));
#assert(size_of(Entasis_Material) == size_of(entasis.Material));
