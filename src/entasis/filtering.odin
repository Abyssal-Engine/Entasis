package entasis

import "base:runtime"
import physics "entasis:entasis_physics"

// MAX_COLLISION_LAYER_COUNT number of public collision layers
MAX_COLLISION_LAYER_COUNT :: 64;

// Collision_Layer is a compact layer index in the inclusive range [0, 63]
Collision_Layer :: distinct u8;

// Layer_Mask stores one bit per Collision_Layer
Layer_Mask :: distinct u64;

// LAYER_MASK_NONE empty collision layer mask
LAYER_MASK_NONE :: Layer_Mask(0);
// LAYER_MASK_ALL mask containing every public collision layer
LAYER_MASK_ALL  :: Layer_Mask(max(u64));

// Layer_Matrix stores a symmetric global layer-pair policy. use the helpers
// below rather than mutating rows directly so both directions remain equal
Layer_Matrix :: struct
{
	rows: [MAX_COLLISION_LAYER_COUNT]Layer_Mask,
}

// Collision_Filter combines a layer with the layers this collidable accepts.
// two collidables interact only when both masks accept the opposite layer and
// the optional Layer_Matrix accepts the pair
Collision_Filter :: struct
{
	layer: Collision_Layer,
	mask:  Layer_Mask,
}

// Collision_Properties is one dense property-table record per collidable. layer,
// mask, and material ID are intentionally colocated so the narrow phase performs
// one O(1) table lookup per collidable instead of separate filtering and material
// lookups
Collision_Properties :: struct
{
	mask:        Layer_Mask,
	material_id: Material_ID,
	layer:       Collision_Layer,
}

#assert(size_of(Collision_Properties) == 16);

// Collision_Property_Table is the generation-aware collidable
// property table specialized for collision filtering and materials
Collision_Property_Table :: Collidable_Property_Table(Collision_Properties);

// Collision_Pair_Filter_Proc optionally applies engine-specific pair filtering
// after layer and mask checks. it may execute concurrently on narrow-phase
// workers and must be allocation free
Collision_Pair_Filter_Proc :: #type proc "contextless" (
	user_context: rawptr,
	a, b: Collidable_Reference,
) -> bool;

// Collision_Child_Filter_Proc optionally filters compound children or mesh
// triangles after the parent pair has passed layer filtering
Collision_Child_Filter_Proc :: #type proc "contextless" (
	user_context: rawptr,
	a, b: Collidable_Reference,
	child_a, child_b: i32,
) -> bool;

// Layer_Material_Policy is caller owned and must remain at a stable address
// while its world can step. every referenced table and slice must remain alive
// and immutable during world_step
Layer_Material_Policy :: struct
{
	properties:         ^Collision_Property_Table,
	layer_matrix:       ^Layer_Matrix,
	materials:          Material_Table,
	default_properties: Collision_Properties,
	fallback_material:  Material,
	pair_filter:        Collision_Pair_Filter_Proc,
	child_filter:       Collision_Child_Filter_Proc,
	material_combine:   Material_Combine_Proc,
	user_context:       rawptr,
}

// collision_layer validates and constructs one compact layer index
collision_layer :: #force_inline proc "contextless" (index: int) -> (Collision_Layer, Status)
{
	if index < 0 || index >= MAX_COLLISION_LAYER_COUNT
	{
		return {}, .Invalid_Argument;
	}
	return Collision_Layer(u8(index)), .Ok;
}

// collision_layer_index returns the integer layer index
collision_layer_index :: #force_inline proc "contextless" (layer: Collision_Layer) -> int
{
	return int(u8(layer));
}

// collision_layer_is_valid validates the compact layer value
collision_layer_is_valid :: #force_inline proc "contextless" (layer: Collision_Layer) -> bool
{
	return collision_layer_index(layer) >= 0 && collision_layer_index(layer) < MAX_COLLISION_LAYER_COUNT;
}

// layer_mask creates a mask containing one layer
layer_mask :: #force_inline proc "contextless" (layer: Collision_Layer) -> Layer_Mask
{
	if !collision_layer_is_valid(layer)
	{
		return LAYER_MASK_NONE;
	}
	return Layer_Mask(u64(1) << u64(collision_layer_index(layer)));
}

// layer_mask_none returns an empty layer mask
layer_mask_none :: #force_inline proc "contextless" () -> Layer_Mask
{
	return LAYER_MASK_NONE;
}

// layer_mask_all returns a mask containing every public collision layer
layer_mask_all :: #force_inline proc "contextless" () -> Layer_Mask
{
	return LAYER_MASK_ALL;
}

// layer_mask_add returns a copy with one layer enabled
layer_mask_add :: #force_inline proc "contextless" (
	mask: Layer_Mask,
	layer: Collision_Layer,
) -> Layer_Mask
{
	return Layer_Mask(u64(mask) | u64(layer_mask(layer)));
}

// layer_mask_remove returns a copy with one layer disabled
layer_mask_remove :: #force_inline proc "contextless" (
	mask: Layer_Mask,
	layer: Collision_Layer,
) -> Layer_Mask
{
	return Layer_Mask(u64(mask) & ~u64(layer_mask(layer)));
}

// layer_mask_contains checks one layer bit
layer_mask_contains :: #force_inline proc "contextless" (
	mask: Layer_Mask,
	layer: Collision_Layer,
) -> bool
{
	return (u64(mask) & u64(layer_mask(layer))) != 0;
}

// collision_filter constructs one layer and mask pair
collision_filter :: #force_inline proc "contextless" (
	layer: Collision_Layer,
	mask: Layer_Mask = LAYER_MASK_ALL,
) -> Collision_Filter
{
	return {layer=layer, mask=mask};
}

// collision_filter_default returns layer zero accepting every layer
collision_filter_default :: #force_inline proc "contextless" () -> Collision_Filter
{
	return {layer=Collision_Layer(0), mask=LAYER_MASK_ALL};
}

// collision_properties constructs one dense per-collidable policy record
collision_properties :: #force_inline proc "contextless" (
	filter: Collision_Filter,
	material_id: Material_ID = MATERIAL_ID_INVALID,
) -> Collision_Properties
{
	return {mask=filter.mask, material_id=material_id, layer=filter.layer};
}

// collision_properties_default uses the default filter and fallback material
collision_properties_default :: #force_inline proc "contextless" () -> Collision_Properties
{
	return {
		mask=LAYER_MASK_ALL,
		material_id=MATERIAL_ID_INVALID,
		layer=Collision_Layer(0),
	};
}

// collision_properties_filter reconstructs the transient layer/mask view from
// the compact 16-byte property record
collision_properties_filter :: #force_inline proc "contextless" (
	value: Collision_Properties,
) -> Collision_Filter
{
	return {layer=value.layer, mask=value.mask};
}

// collision_properties_validate checks the stable record representation. material
// table bounds are policy-specific and are checked by the narrow-phase adapter
collision_properties_validate :: #force_inline proc "contextless" (
	value: Collision_Properties,
) -> Status
{
	if !collision_layer_is_valid(value.layer)
	{
		return .Invalid_Description;
	}
	return .Ok;
}

// layer_matrix_none rejects every layer pair
layer_matrix_none :: proc "contextless" () -> Layer_Matrix
{
	return {};
}

// layer_matrix_all accepts every layer pair
layer_matrix_all :: proc "contextless" () -> Layer_Matrix
{
	layer_rules: Layer_Matrix;
	for index in 0 ..< MAX_COLLISION_LAYER_COUNT
	{
		layer_rules.rows[index] = LAYER_MASK_ALL;
	}
	return layer_rules;
}

// layer_matrix_set updates both directions of one layer pair
layer_matrix_set :: proc "contextless" (
	layer_rules: ^Layer_Matrix,
	a, b: Collision_Layer,
	allow: bool,
) -> Status
{
	if layer_rules == nil || !collision_layer_is_valid(a) || !collision_layer_is_valid(b)
	{
		return .Invalid_Argument;
	}
	a_index := collision_layer_index(a);
	b_index := collision_layer_index(b);
	if allow
	{
		layer_rules.rows[a_index] = layer_mask_add(layer_rules.rows[a_index], b);
		layer_rules.rows[b_index] = layer_mask_add(layer_rules.rows[b_index], a);
	}
	else
	{
		layer_rules.rows[a_index] = layer_mask_remove(layer_rules.rows[a_index], b);
		layer_rules.rows[b_index] = layer_mask_remove(layer_rules.rows[b_index], a);
	}
	return .Ok;
}

// layer_matrix_allow enables one symmetric layer pair
layer_matrix_allow :: #force_inline proc "contextless" (
	layer_rules: ^Layer_Matrix,
	a, b: Collision_Layer,
) -> Status
{
	return layer_matrix_set(layer_rules, a, b, true);
}

// layer_matrix_deny disables one symmetric layer pair
layer_matrix_deny :: #force_inline proc "contextless" (
	layer_rules: ^Layer_Matrix,
	a, b: Collision_Layer,
) -> Status
{
	return layer_matrix_set(layer_rules, a, b, false);
}

// layer_matrix_allows checks both matrix directions. requiring both directions
// makes direct row edits fail closed when symmetry is accidentally broken
layer_matrix_allows :: #force_inline proc "contextless" (
	layer_rules: ^Layer_Matrix,
	a, b: Collision_Layer,
) -> bool
{
	if layer_rules == nil
	{
		return true;
	}
	if !collision_layer_is_valid(a) || !collision_layer_is_valid(b)
	{
		return false;
	}
	return layer_mask_contains(layer_rules.rows[collision_layer_index(a)], b) &&
		layer_mask_contains(layer_rules.rows[collision_layer_index(b)], a);
}

// collision_filter_allows applies both per-collidable masks and the optional
// global layer matrix
collision_filter_allows :: #force_inline proc "contextless" (
	a, b: Collision_Filter,
	layer_rules: ^Layer_Matrix = nil,
) -> bool
{
	if !collision_layer_is_valid(a.layer) || !collision_layer_is_valid(b.layer)
	{
		return false;
	}
	return layer_mask_contains(a.mask, b.layer) &&
		layer_mask_contains(b.mask, a.layer) &&
		layer_matrix_allows(layer_rules, a.layer, b.layer);
}

// collision_property_init initializes the generation-aware body and static
// property spaces. capacity hints may be zero and grow geometrically
collision_property_init :: #force_inline proc (
	table: ^Collision_Property_Table,
	body_capacity: int = 0,
	static_capacity: int = 0,
	allocator: Allocator = {},
) -> Status
{
	return collidable_property_init(table, body_capacity, static_capacity, allocator);
}

// collision_property_ensure_capacity grows both namespaces independently
collision_property_ensure_capacity :: #force_inline proc (
	table: ^Collision_Property_Table,
	body_capacity, static_capacity: int,
) -> Status
{
	return collidable_property_ensure_capacity(table, body_capacity, static_capacity);
}

// collision_property_set_body creates or updates one body policy record
collision_property_set_body :: #force_inline proc (
	table: ^Collision_Property_Table,
	handle: Body_Handle,
	value: Collision_Properties,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	status := collision_properties_validate(value);
	if status != .Ok
	{
		return status;
	}
	return body_property_set(&table.bodies, handle, value);
}

// collision_property_get_body returns one body policy record
collision_property_get_body :: #force_inline proc "contextless" (
	table: ^Collision_Property_Table,
	handle: Body_Handle,
) -> (^Collision_Properties, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	return body_property_get(&table.bodies, handle);
}

// collision_property_set_static creates or updates one static policy record
collision_property_set_static :: #force_inline proc (
	table: ^Collision_Property_Table,
	handle: Static_Handle,
	value: Collision_Properties,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	status := collision_properties_validate(value);
	if status != .Ok
	{
		return status;
	}
	return static_property_set(&table.statics, handle, value);
}

// collision_property_get_static returns one static policy record
collision_property_get_static :: #force_inline proc "contextless" (
	table: ^Collision_Property_Table,
	handle: Static_Handle,
) -> (^Collision_Properties, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	return static_property_get(&table.statics, handle);
}

// collision_property_set creates or updates a body or static policy record
collision_property_set :: #force_inline proc (
	table: ^Collision_Property_Table,
	collidable: Collidable_Reference,
	value: Collision_Properties,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	status := collision_properties_validate(value);
	if status != .Ok
	{
		return status;
	}
	return collidable_property_set(table, collidable, value);
}

// collision_property_set_batch applies records in exact input order
collision_property_set_batch :: #force_inline proc (
	table: ^Collision_Property_Table,
	collidables: []Collidable_Reference,
	values: []Collision_Properties,
) -> (int, Status)
{
	if table == nil || len(collidables) != len(values)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(values)
	{
		status := collision_properties_validate(values[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return collidable_property_set_batch(table, collidables, values);
}

// collision_property_get returns one body or static policy record
collision_property_get :: #force_inline proc "contextless" (
	table: ^Collision_Property_Table,
	collidable: Collidable_Reference,
) -> (^Collision_Properties, Status)
{
	return collidable_property_get(table, collidable);
}

// collision_property_remove_body invalidates one body policy lifetime
collision_property_remove_body :: #force_inline proc (
	table: ^Collision_Property_Table,
	handle: Body_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return body_property_remove(&table.bodies, handle);
}

// collision_property_remove_static invalidates one static policy lifetime
collision_property_remove_static :: #force_inline proc (
	table: ^Collision_Property_Table,
	handle: Static_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return static_property_remove(&table.statics, handle);
}

// collision_property_remove invalidates one body or static policy record
collision_property_remove :: #force_inline proc (
	table: ^Collision_Property_Table,
	collidable: Collidable_Reference,
) -> Status
{
	return collidable_property_remove(table, collidable);
}

// collision_property_remove_batch invalidates records in exact input order
collision_property_remove_batch :: #force_inline proc (
	table: ^Collision_Property_Table,
	collidables: []Collidable_Reference,
) -> (int, Status)
{
	return collidable_property_remove_batch(table, collidables);
}

// collision_property_clear invalidates every property while retaining capacity
collision_property_clear :: #force_inline proc (
	table: ^Collision_Property_Table,
) -> Status
{
	return collidable_property_clear(table);
}

// collision_property_destroy releases caller-owned property storage
collision_property_destroy :: #force_inline proc (
	table: ^Collision_Property_Table,
) -> Status
{
	return collidable_property_destroy(table);
}

// layer_material_policy constructs a complete allocation-free narrow-phase
// policy. nil property or matrix pointers select the documented defaults
layer_material_policy :: proc "contextless" (
	properties: ^Collision_Property_Table = nil,
	layer_rules: ^Layer_Matrix = nil,
	materials: Material_Table = {},
) -> Layer_Material_Policy
{
	return {
		properties=properties,
		layer_matrix=layer_rules,
		materials=materials,
		default_properties=collision_properties_default(),
		fallback_material=material_default(),
	};
}

@(private)
layer_material_properties :: #force_inline proc "contextless" (
	policy: ^Layer_Material_Policy,
	collidable: Collidable_Reference,
) -> (Collision_Properties, bool)
{
	if policy.properties == nil
	{
		return policy.default_properties, true;
	}
	value, status := collidable_property_get(policy.properties, collidable);
	if status == .Ok
	{
		return value^, true;
	}
	if status == .Not_Found
	{
		return policy.default_properties, true;
	}
	return {}, false;
}

@(private)
layer_material_resolve :: #force_inline proc "contextless" (
	policy: ^Layer_Material_Policy,
	id: Material_ID,
) -> (Material, bool)
{
	if !material_id_is_valid(id)
	{
		return policy.fallback_material, true;
	}
	value, status := material_table_get(policy.materials, id);
	if status != .Ok
	{
		return {}, false;
	}
	return value^, true;
}

@(private)
layer_material_initialize :: proc "contextless" (
	user_context: rawptr,
	simulation: ^physics.Simulation,
) -> Status
{
	if user_context == nil || simulation == nil
	{
		return .Invalid_Argument;
	}
	context = runtime.default_context();
	policy := (^Layer_Material_Policy)(user_context);
	if !collision_layer_is_valid(policy.default_properties.layer) ||
		material_validate(policy.fallback_material) != .Ok ||
		len(policy.materials.values) > int(max(u16))
	{
		return .Invalid_Description;
	}
	for value in policy.materials.values
	{
		if material_validate(value) != .Ok
		{
			return .Invalid_Description;
		}
	}
	if material_id_is_valid(policy.default_properties.material_id)
	{
		_, default_material_status := material_table_get(
			policy.materials, policy.default_properties.material_id,
		);
		if default_material_status != .Ok
		{
			return .Invalid_Description;
		}
	}
	if policy.properties != nil
	{
		status := collidable_property_ensure_capacity(
			policy.properties,
			int(simulation.bodies.handle_to_location.length),
			int(simulation.statics.handle_to_index.length),
		);
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

@(private)
layer_material_allow :: #force_inline proc "contextless" (
	user_context: rawptr,
	worker_index: int,
	a, b: Collidable_Reference,
	speculative_margin: ^f32,
) -> Collision_Testing_State
{
	_, _ = worker_index, speculative_margin;
	if user_context == nil
	{
		return .Reject;
	}
	policy := (^Layer_Material_Policy)(user_context);
	a_properties, a_ok := layer_material_properties(policy, a);
	b_properties, b_ok := layer_material_properties(policy, b);
	if !a_ok || !b_ok ||
		!collision_filter_allows(
		collision_properties_filter(a_properties),
		collision_properties_filter(b_properties),
		policy.layer_matrix
	)
	{
		return .Reject;
	}
	if policy.pair_filter != nil && !policy.pair_filter(policy.user_context, a, b)
	{
		return .Reject;
	}
	return .Allow;
}

@(private)
layer_material_allow_child :: #force_inline proc "contextless" (
	user_context: rawptr,
	worker_index: int,
	a, b: Collidable_Reference,
	child_a, child_b: int,
) -> Collision_Testing_State
{
	_ = worker_index;
	if user_context == nil
	{
		return .Reject;
	}
	policy := (^Layer_Material_Policy)(user_context);
	if policy.child_filter != nil &&
		!policy.child_filter(policy.user_context, a, b, i32(child_a), i32(child_b))
	{
		return .Reject;
	}
	return .Allow;
}

@(private)
layer_material_configure :: #force_inline proc "contextless" (
	user_context: rawptr,
	worker_index: int,
	a, b: Collidable_Reference,
	manifold: ^Manifold_Result,
	contact: ^Contact_Material,
) -> Collision_Testing_State
{
	_, _ = worker_index, manifold;
	if user_context == nil || contact == nil
	{
		return .Reject;
	}
	policy := (^Layer_Material_Policy)(user_context);
	a_properties, a_ok := layer_material_properties(policy, a);
	b_properties, b_ok := layer_material_properties(policy, b);
	if !a_ok || !b_ok
	{
		return .Reject;
	}
	a_material, a_material_ok := layer_material_resolve(policy, a_properties.material_id);
	b_material, b_material_ok := layer_material_resolve(policy, b_properties.material_id);
	if !a_material_ok || !b_material_ok
	{
		return .Reject;
	}
	combined := material_combine_default(a_material, b_material);
	if policy.material_combine != nil
	{
		combined = policy.material_combine(
			policy.user_context,
			a_properties.material_id, a_material,
			b_properties.material_id, b_material,
		);
	}
	if material_validate(combined) != .Ok
	{
		return .Reject;
	}
	contact^ = material_to_contact(combined);
	return .Allow;
}

@(private)
layer_material_configure_child :: #force_inline proc "contextless" (
	user_context: rawptr,
	worker_index: int,
	a, b: Collidable_Reference,
	child_a, child_b: int,
	manifold: ^Convex_Contact_Manifold,
) -> Collision_Testing_State
{
	_, _, _, _, _, _, _ = user_context, worker_index, a, b, child_a, child_b, manifold;
	return .Allow;
}

@(private)
layer_material_dispose :: proc "contextless" (user_context: rawptr)
{
	_ = user_context;
}

// narrow_policy_layers_materials builds the stable layer, mask, pair-filter,
// child-filter, and material-table adapter around caller-owned context
// allocation: none after optional property-table growth during world_init
narrow_policy_layers_materials :: #force_inline proc "contextless" (
	policy: ^Layer_Material_Policy,
) -> Narrow_Callbacks
{
	callbacks := physics.narrow_phase_default_callbacks();
	callbacks.initialize = layer_material_initialize;
	callbacks.allow = layer_material_allow;
	callbacks.allow_child = layer_material_allow_child;
	callbacks.configure = layer_material_configure;
	callbacks.configure_child = layer_material_configure_child;
	callbacks.dispose = layer_material_dispose;
	callbacks.user_context = policy;
	return callbacks;
}
