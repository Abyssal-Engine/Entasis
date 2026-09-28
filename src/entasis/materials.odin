package entasis

import physics "entasis:entasis_physics"

// Material_ID indexes one caller-owned Material_Table. the all-bits-set value
// selects a policy's fallback material instead of indexing the table
Material_ID :: distinct u16;

// MATERIAL_ID_INVALID fallback-material sentinel value
MATERIAL_ID_INVALID :: Material_ID(max(u16));

// Material is the stable per-collidable material description used by the
// layer-and-material narrow-phase policy. it converts directly to the
// low-level Contact_Material without allocating or changing solver semantics
Material :: struct
{
	friction:                 f32,
	maximum_recovery_velocity: f32,
	spring:                   Spring_Settings,
}

// Material_Table is a caller-owned immutable view while a world is stepping.
// the backing slice must remain alive and at a stable address until the world is
// destroyed or another callback policy is installed in a new world
Material_Table :: struct
{
	values: []Material,
}

// Material_Combine_Proc optionally replaces material_combine_default. the
// callback may execute concurrently on narrow-phase workers and must not
// allocate, mutate shared state without synchronization, or depend on worker
// completion order
Material_Combine_Proc :: #type proc "contextless" (
	user_context: rawptr,
	a_id: Material_ID,
	a: Material,
	b_id: Material_ID,
	b: Material,
) -> Material;

// material_id converts a nonnegative table index into a Material_ID. the
// sentinel value is reserved for fallback-material selection
material_id :: #force_inline proc "contextless" (index: int) -> (Material_ID, Status)
{
	if index < 0 || index >= int(max(u16))
	{
		return MATERIAL_ID_INVALID, .Invalid_Argument;
	}
	return Material_ID(u16(index)), .Ok;
}

// material_id_invalid returns the fallback-material sentinel
material_id_invalid :: #force_inline proc "contextless" () -> Material_ID
{
	return MATERIAL_ID_INVALID;
}

// material_id_is_valid checks only the sentinel representation. table bounds
// are validated by material_table_get
material_id_is_valid :: #force_inline proc "contextless" (id: Material_ID) -> bool
{
	return id != MATERIAL_ID_INVALID;
}

// material constructs one per-collidable material without allocation
material :: #force_inline proc "contextless" (
	friction: f32,
	maximum_recovery_velocity: f32,
	spring: Spring_Settings,
) -> Material
{
	return {
		friction=friction,
		maximum_recovery_velocity=maximum_recovery_velocity,
		spring=spring,
	};
}

// material_default returns the material used by the default narrow
// phase when two default collidables interact
material_default :: #force_inline proc "contextless" () -> Material
{
	return material(1, 2, spring_settings(30, 1));
}

// material_table binds caller-owned material storage. allocation: none
material_table :: #force_inline proc "contextless" (values: []Material) -> Material_Table
{
	return {values=values};
}

// material_to_contact converts the facade material to the exact low-level
// contact material consumed by contact constraints
material_to_contact :: #force_inline proc "contextless" (value: Material) -> Contact_Material
{
	return {
		friction_coefficient=value.friction,
		spring_settings=value.spring,
		maximum_recovery_velocity=value.maximum_recovery_velocity,
	};
}

// material_validate applies the same validation used by the contact
// constraint path
material_validate :: #force_inline proc "contextless" (value: Material) -> Status
{
	status := physics.constraint_contact_material_validate(material_to_contact(value));
	if status != .Ok
	{
		return .Invalid_Description;
	}
	return .Ok;
}

// material_table_get performs one bounds check and returns a pointer into the
// caller-owned table. allocation: none
material_table_get :: #force_inline proc "contextless" (
	table: Material_Table,
	id: Material_ID,
) -> (^Material, Status)
{
	if !material_id_is_valid(id)
	{
		return nil, .Not_Found;
	}
	index := int(u16(id));
	if index < 0 || index >= len(table.values)
	{
		return nil, .Not_Found;
	}
	return &table.values[index], .Ok;
}

@(private)
material_spring_prefers_a :: #force_inline proc "contextless" (a, b: Material) -> bool
{
	if a.maximum_recovery_velocity != b.maximum_recovery_velocity
	{
		return a.maximum_recovery_velocity > b.maximum_recovery_velocity;
	}
	if a.spring.angular_frequency != b.spring.angular_frequency
	{
		return a.spring.angular_frequency > b.spring.angular_frequency;
	}
	return a.spring.twice_damping_ratio >= b.spring.twice_damping_ratio;
}

// material_combine_default implements a commutative order-independent combination:
// friction is multiplied, maximum recovery velocity is the larger value, and
// spring settings come from the material with the larger recovery velocity.
// ties select the stiffer spring and then the more damped spring. the rule is
// independent of collidable ordering and worker count
material_combine_default :: #force_inline proc "contextless" (a, b: Material) -> Material
{
	selected_spring := b.spring;
	if material_spring_prefers_a(a, b)
	{
		selected_spring = a.spring;
	}
	maximum_recovery_velocity := b.maximum_recovery_velocity;
	if a.maximum_recovery_velocity > b.maximum_recovery_velocity
	{
		maximum_recovery_velocity = a.maximum_recovery_velocity;
	}
	return {
		friction=a.friction * b.friction,
		maximum_recovery_velocity=maximum_recovery_velocity,
		spring=selected_spring,
	};
}
