package entasis

import physics "entasis:entasis_physics"

// Contact_Constraint_Kind identifies convex and nonconvex solver contact layouts
Contact_Constraint_Kind :: physics.Contact_Constraint_Kind;

// MAXIMUM_SOLVER_CONTACT_COUNT is the largest built-in manifold contact count
MAXIMUM_SOLVER_CONTACT_COUNT :: physics.MAXIMUM_MANIFOLD_CONTACT_COUNT;
// MAXIMUM_SOLVER_IMPULSE_COUNT bounds one pointer-free impulse snapshot
MAXIMUM_SOLVER_IMPULSE_COUNT :: physics.MAXIMUM_INACTIVE_IMPULSE_SCALARS;

// Solver_Contact_Point is one contact point reconstructed from solver prestep
// data, pair-cache feature identity, and accumulated impulse state
Solver_Contact_Point :: struct
{
	offset_a:       Vector3,
	normal:         Vector3,
	depth:          f32,
	feature_id:     i32,
	normal_impulse: f32,
}

// Solver_Contact_Data is a pointer-free snapshot of one active or sleeping
// built-in contact constraint. allocation: none
Solver_Contact_Data :: struct
{
	constraint:    Constraint_Handle,
	pair_a:        Collidable_Reference,
	pair_b:        Collidable_Reference,
	kind:          Contact_Constraint_Kind,
	body_count:    u8,
	contact_count: u8,
	impulse_count: u8,
	body_handles:  [4]Body_Handle,
	offset_b:      Vector3,
	normal:        Vector3,
	material:      Contact_Material,
	contacts:      [MAXIMUM_SOLVER_CONTACT_COUNT]Solver_Contact_Point,
	impulses:      [MAXIMUM_SOLVER_IMPULSE_COUNT]f32,
}

@(private)
solver_contact_copy_context :: struct
{
	target: ^physics.Contact_Constraint_Data_View,
}

@(private)
solver_contact_copy_visitor :: proc "contextless" (
	user_context: rawptr, view: ^physics.Contact_Constraint_Data_View,
)
{
	if user_context == nil || view == nil
	{
		return;
	}
	ctx := (^solver_contact_copy_context)(user_context);
	ctx.target^ = view^;
}

@(private)
solver_contact_pair_cache :: proc "contextless" (
	simulation: ^physics.Simulation, handle: Constraint_Handle,
) -> (physics.Collidable_Pair, physics.Constraint_Cache, Status)
{
	if simulation == nil || simulation.state != .Ready
	{
		return {}, {}, .Disposed;
	}
	cache := &simulation.narrow_phase.pair_cache;
	if handle.value < 0 || int(handle.value) >= int(cache.constraint_handle_to_pair.length)
	{
		return {}, {}, .Not_Found;
	}
	location := cache.constraint_handle_to_pair.memory[handle.value];
	if location.inactive_set_index >= 0
	{
		index := int(location.inactive_pair_index);
		if index < 0 || index >= cache.inactive_count
		{
			return {}, {}, .Not_Found;
		}
		entry := cache.inactive_entries.memory[index];
		if entry.cache.constraint_handle.value != handle.value
		{
			return {}, {}, .Not_Found;
		}
		return entry.pair, entry.cache, .Ok;
	}
	index := physics.pair_cache_index_of(cache, location.pair);
	if index < 0 || index >= cache.mapping.count
	{
		return {}, {}, .Not_Found;
	}
	stored := cache.mapping.values.memory[index];
	if stored.constraint_handle.value != handle.value
	{
		return {}, {}, .Not_Found;
	}
	return cache.mapping.keys.memory[index], stored, .Ok;
}

@(private)
solver_contact_data_from_simulation :: proc "contextless" (
	simulation: ^physics.Simulation, handle: Constraint_Handle,
) -> (Solver_Contact_Data, Status)
{
	if simulation == nil || simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	view: physics.Contact_Constraint_Data_View;
	copy_ctx := solver_contact_copy_context{target=&view};
	status := physics.narrow_phase_try_extract_solver_contact_data(
		&simulation.narrow_phase, handle, solver_contact_copy_visitor, &copy_ctx,
	);
	if status != .Ok
	{
		return {}, status;
	}
	pair, cache, pair_status := solver_contact_pair_cache(simulation, handle);
	if pair_status != .Ok
	{
		return {}, pair_status;
	}
	result := Solver_Contact_Data{
		constraint=handle,
		pair_a=pair.a,
		pair_b=pair.b,
		kind=view.kind,
		body_count=u8(view.body_count),
		contact_count=u8(view.contact_count),
		impulse_count=u8(view.impulse_count),
		body_handles=view.body_handles,
	};
	for impulse_index in 0 ..< int(view.impulse_count)
	{
		result.impulses[impulse_index] = view.impulses[impulse_index];
	}
	prestep := rawptr(&view.prestep[0]);
	if view.kind == .Convex
	{
		contacts := ([^]physics.Constraint_Contact_Data)(prestep);
		offset := int(view.contact_count) * size_of(physics.Constraint_Contact_Data);
		if view.body_count == 2
		{
			result.offset_b = (^[1]Vector3)(rawptr(uintptr(prestep) + uintptr(offset)))[0];
			offset += size_of(Vector3);
		}
		result.normal = (^[1]Vector3)(rawptr(uintptr(prestep) + uintptr(offset)))[0];
		offset += size_of(Vector3);
		result.material = (^[1]Contact_Material)(rawptr(uintptr(prestep) + uintptr(offset)))[0];
		for contact_index in 0 ..< int(view.contact_count)
		{
			impulse_index := 2 + contact_index;
			normal_impulse := f32(0);
			if impulse_index < int(view.impulse_count)
			{
				normal_impulse = view.impulses[impulse_index];
			}
			result.contacts[contact_index] = {
				offset_a=contacts[contact_index].offset_a,
				normal=result.normal,
				depth=contacts[contact_index].penetration_depth,
				feature_id=cache.feature_ids[contact_index],
				normal_impulse=normal_impulse,
			};
		}
	}
	else
	{
		offset := 0;
		if view.body_count == 2
		{
			common := (^physics.Nonconvex_Two_Body_Properties)(prestep);
			result.offset_b = common.offset_b;
			result.material = {
				friction_coefficient=common.friction_coefficient,
				spring_settings=common.spring_settings,
				maximum_recovery_velocity=common.maximum_recovery_velocity,
			};
			offset = size_of(physics.Nonconvex_Two_Body_Properties);
		}
		else
		{
			common := (^physics.Nonconvex_One_Body_Properties)(prestep);
			result.material = {
				friction_coefficient=common.friction_coefficient,
				spring_settings=common.spring_settings,
				maximum_recovery_velocity=common.maximum_recovery_velocity,
			};
			offset = size_of(physics.Nonconvex_One_Body_Properties);
		}
		contacts := ([^]physics.Nonconvex_Constraint_Contact_Data)(
			rawptr(uintptr(prestep) + uintptr(offset)),
		);
		for contact_index in 0 ..< int(view.contact_count)
		{
			impulse_index := contact_index * 3 + 2;
			normal_impulse := f32(0);
			if impulse_index < int(view.impulse_count)
			{
				normal_impulse = view.impulses[impulse_index];
			}
			contact := contacts[contact_index];
			result.contacts[contact_index] = {
				offset_a=contact.offset_a,
				normal=contact.normal,
				depth=contact.penetration_depth,
				feature_id=cache.feature_ids[contact_index],
				normal_impulse=normal_impulse,
			};
		}
	}
	return result, .Ok;
}

// solver_contact_data extracts one built-in contact constraint including
// contact offsets, depths, normals, feature IDs, material and accumulated
// impulses. it supports active and sleeping contacts. allocation: none
solver_contact_data :: proc "contextless" (
	world: ^World, handle: Constraint_Handle,
) -> (Solver_Contact_Data, Status)
{
	data := world_data_get(world);
	if data == nil
	{
		return {}, .Disposed;
	}
	return solver_contact_data_from_simulation(&data.simulation, handle);
}
