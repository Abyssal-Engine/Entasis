// derived from BEPUphysics2 by Ross Nordby.
// ported to Odin and substantially modified for Entasis.
// see LICENSE and NOTICE
package entasis_physics

import util "entasis:entasis_utilities"
import "core:simd"

Contact_Constraint_Kind :: enum u8
{
	Convex,
	Nonconvex,
}

@(private)
Contact_Feature_Equality :: enum u8
{
	Different,
	Equal,
}

Contact_Constraint_Data_View :: struct
{
	type_id:        i32,
	kind:           Contact_Constraint_Kind,
	body_count:     i32,
	contact_count:  i32,
	prestep_size:   i32,
	impulse_count:  i32,
	body_handles:   [4]Body_Handle,
	prestep:        [CONSTRAINT_DESCRIPTION_STORAGE_BYTES]u8,
	impulses:       [MAXIMUM_INACTIVE_IMPULSE_SCALARS]f32,
}

// the view is borrowed only for the duration of the synchronous call
Contact_Constraint_Data_Visitor_Proc :: #type proc "contextless" (
	user_context: rawptr, view: ^Contact_Constraint_Data_View,
);

Contact_Constraint_Build_Proc :: #type proc "contextless" (
	manifold: ^Manifold_Result, body_count: int, material: Contact_Material_Properties,
	target: rawptr, feature_ids: ^[MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
) -> Physics_Status;

Contact_Constraint_Impulse_Offset_Proc :: #type proc "contextless" (
	contact_index: int,
) -> int;

Contact_Constraint_Accessor_Record :: struct
{
	type_id:       i32,
	body_count:    i32,
	contact_count: i32,
	kind:          Contact_Constraint_Kind,
	build_description: Contact_Constraint_Build_Proc,
	impulse_offset: Contact_Constraint_Impulse_Offset_Proc,
	registration:  Reference_State,
}

Contact_Constraint_Accessors :: struct
{
	records: [CONSTRAINT_TYPE_ID_CAPACITY]Contact_Constraint_Accessor_Record,
	state:   Constraint_Registry_State,
}

contact_constraint_type_id :: proc "contextless" (
	body_count, contact_count: int, kind: Contact_Constraint_Kind,
) -> i32
{
	if body_count < 1 || body_count > 2 || contact_count < 1 || contact_count > 4
	{
		return -1;
	}
	if kind == .Convex || contact_count == 1
	{
		if body_count == 1
		{
			return i32(CONTACT_1_ONE_BODY_TYPE_ID + contact_count - 1);
		}
		return i32(CONTACT_1_TYPE_ID + contact_count - 1);
	}
	if body_count == 1
	{
		return i32(CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID + contact_count - 2);
	}
	return i32(CONTACT_2_NONCONVEX_TYPE_ID + contact_count - 2);
}

contact_constraint_accessors_register :: proc "contextless" (
	accessors: ^Contact_Constraint_Accessors, record: Contact_Constraint_Accessor_Record,
) -> Physics_Status
{
	if accessors == nil || accessors.state != .Ready || record.type_id < 0 ||
		record.type_id >= CONSTRAINT_TYPE_ID_CAPACITY || record.body_count < 1 || record.body_count > 2 ||
		record.contact_count < 1 || record.contact_count > MAXIMUM_MANIFOLD_CONTACT_COUNT ||
		record.build_description == nil || record.impulse_offset == nil ||
		accessors.records[record.type_id].registration == .Present
	{
		return .Invalid_Argument;
	}
	stored_record := record;
	stored_record.registration = .Present;
	accessors.records[record.type_id] = stored_record;
	return .Ok;
}

contact_constraint_builtin_impulse_offset_convex :: proc "contextless" (contact_index: int) -> int
{
	return 2 + contact_index;
}

contact_constraint_builtin_impulse_offset_nonconvex :: proc "contextless" (contact_index: int) -> int
{
	return contact_index * 3 + 2;
}

contact_constraint_accessors_initialize :: proc "contextless" (
	accessors: ^Contact_Constraint_Accessors,
) -> Physics_Status
{
	if accessors == nil || accessors.state != .Uninitialized
	{
		return .Invalid_Argument;
	}
	accessors^ = {state=.Ready};
	for body_count in 1 ..= 2
	{
		for contact_count in 1 ..= 4
		{
			type_id := contact_constraint_type_id(body_count, contact_count, .Convex);
			status := contact_constraint_accessors_register(accessors, {
					type_id=type_id,
					body_count=i32(body_count),
					contact_count=i32(contact_count),
					kind=.Convex,
					build_description=contact_constraint_builtin_build,
					impulse_offset=contact_constraint_builtin_impulse_offset_convex,
				});
			if status != .Ok
			{
				accessors^ = {};
				return status;
			}
			if contact_count > 1
			{
				type_id = contact_constraint_type_id(body_count, contact_count, .Nonconvex);
				status = contact_constraint_accessors_register(accessors, {
						type_id=type_id,
						body_count=i32(body_count),
						contact_count=i32(contact_count),
						kind=.Nonconvex,
						build_description=contact_constraint_builtin_build,
						impulse_offset=contact_constraint_builtin_impulse_offset_nonconvex,
					});
				if status != .Ok
				{
					accessors^ = {};
					return status;
				}
			}
		}
	}
	return .Ok;
}

contact_constraint_accessors_register_custom :: proc (
	accessors: ^Contact_Constraint_Accessors, solver: ^Solver,
	record: Contact_Constraint_Accessor_Record,
) -> Physics_Status
{
	if solver == nil || record.type_id < FIRST_CALLER_CONSTRAINT_TYPE_ID
	{
		return .Invalid_Argument;
	}
	type_record, status := constraint_type_registry_lookup(&solver.registry, record.type_id);
	if status != .Ok || type_record.body_count != record.body_count
	{
		return .Invalid_Argument;
	}
	status = solver_ensure_type_capacity(solver, int(record.type_id), int(solver.initial_type_batch_capacity));
	if status != .Ok
	{
		return status;
	}
	return contact_constraint_accessors_register(accessors, record);
}

@(private)
contact_constraint_accessor_lookup :: proc "contextless" (
	accessors: ^Contact_Constraint_Accessors, type_id: i32,
) -> (^Contact_Constraint_Accessor_Record, Physics_Status)
{
	if accessors == nil || accessors.state != .Ready || type_id < 0 || type_id >= CONSTRAINT_TYPE_ID_CAPACITY
	{
		return nil, .Not_Found;
	}
	record := &accessors.records[type_id];
	if record.registration != .Present
	{
		return nil, .Not_Found;
	}
	return record, .Ok;
}

contact_constraint_write_convex :: proc "contextless" (
	target: rawptr, body_count: int, manifold: ^Convex_Contact_Manifold,
	material: Contact_Material_Properties,
) -> Physics_Status
{
	if target == nil || manifold == nil || manifold.count < 1 || manifold.count > 4
	{
		return .Invalid_Argument;
	}
	contacts := ([^]Constraint_Contact_Data)(target);
	for index in 0 ..< int(manifold.count)
	{
		contacts[index] = {manifold.contacts[index].offset, manifold.contacts[index].depth};
	}
	offset := int(manifold.count) * size_of(Constraint_Contact_Data);
	if body_count == 2
	{
		(^util.Vector3)(rawptr(uintptr(target) + uintptr(offset)))^ = manifold.offset_b;
		offset += size_of(util.Vector3);
	}
	(^util.Vector3)(rawptr(uintptr(target) + uintptr(offset)))^ = manifold.normal;
	offset += size_of(util.Vector3);
	(^Contact_Material_Properties)(rawptr(uintptr(target) + uintptr(offset)))^ = material;
	return .Ok;
}

contact_constraint_write_nonconvex :: proc "contextless" (
	target: rawptr, body_count: int, manifold: ^Nonconvex_Contact_Manifold,
	material: Contact_Material_Properties,
) -> Physics_Status
{
	if target == nil || manifold == nil || manifold.count < 2 || manifold.count > 4
	{
		return .Invalid_Argument;
	}
	offset := 0;
	if body_count == 2
	{
		common := (^Nonconvex_Two_Body_Properties)(target);
		common^ = {
			manifold.offset_b,
			material.friction_coefficient,
			material.spring_settings,
			material.maximum_recovery_velocity,
		};
		offset = size_of(Nonconvex_Two_Body_Properties);
	}
	else
	{
		common := (^Nonconvex_One_Body_Properties)(target);
		common^ = {material.friction_coefficient, material.spring_settings, material.maximum_recovery_velocity};
		offset = size_of(Nonconvex_One_Body_Properties);
	}
	contacts := ([^]Nonconvex_Constraint_Contact_Data)(rawptr(uintptr(target) + uintptr(offset)));
	for index in 0 ..< int(manifold.count)
	{
		contact := manifold.contacts[index];
		contacts[index] = {contact.offset, contact.normal, contact.depth};
	}
	return .Ok;
}

contact_constraint_builtin_build :: proc "contextless" (
	manifold: ^Manifold_Result, body_count: int, material: Contact_Material_Properties,
	target: rawptr, feature_ids: ^[MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
) -> Physics_Status
{
	if manifold == nil || target == nil || feature_ids == nil
	{
		return .Invalid_Argument;
	}
	if manifold.kind == .Convex
	{
		for index in 0 ..< int(manifold.convex.count)
		{
			feature_ids[index] = manifold.convex.contacts[index].feature_id;
		}
		return contact_constraint_write_convex(target, body_count, &manifold.convex, material);
	}
	for index in 0 ..< int(manifold.nonconvex.count)
	{
		feature_ids[index] = manifold.nonconvex.contacts[index].feature_id;
	}
	return contact_constraint_write_nonconvex(target, body_count, &manifold.nonconvex, material);
}

contact_constraint_build_description :: proc "contextless" (
	accessors: ^Contact_Constraint_Accessors, manifold: ^Manifold_Result, body_count: int,
	material: Contact_Material_Properties, target: rawptr, requested_type_id: i32 = -1,
) -> (i32, [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32, Physics_Status)
{
	feature_ids: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32;
	for index in 0 ..< MAXIMUM_MANIFOLD_CONTACT_COUNT
	{
		feature_ids[index] = -1;
	}
	if accessors == nil || manifold == nil || target == nil || body_count < 1 || body_count > 2
	{
		return -1, feature_ids, .Invalid_Argument;
	}
	kind := Contact_Constraint_Kind.Convex;
	contact_count := int(manifold.convex.count);
	build_manifold := manifold;
	converted: Manifold_Result;
	if manifold.kind == .Nonconvex
	{
		contact_count = int(manifold.nonconvex.count);
		if contact_count == 1
		{
			contact := manifold.nonconvex.contacts[0];
			converted = {
				kind=.Convex,
				convex={
					offset_b=manifold.nonconvex.offset_b,
					count=1,
					normal=contact.normal,
				},
			};
			converted.convex.contacts[0] = {contact.offset, contact.depth, contact.feature_id};
			build_manifold = &converted;
		}
		else
		{
			kind = .Nonconvex;
		}
	}
	if contact_count < 1 || contact_count > 4
	{
		return -1, feature_ids, .Invalid_Description;
	}
	type_id := requested_type_id;
	if type_id < 0
	{
		type_id = contact_constraint_type_id(body_count, contact_count, kind);
	}
	record, lookup_status := contact_constraint_accessor_lookup(accessors, type_id);
	if lookup_status != .Ok
	{
		return -1, feature_ids, lookup_status;
	}
	if int(record.body_count) != body_count || int(record.contact_count) != contact_count || record.kind != kind
	{
		return -1, feature_ids, .Invalid_Description;
	}
	status := record.build_description(build_manifold, body_count, material, target, &feature_ids);
	return type_id, feature_ids, status;
}

contact_constraint_impulse_offset :: proc "contextless" (
	record: ^Contact_Constraint_Accessor_Record, contact_index: int,
) -> int
{
	return record.impulse_offset(contact_index);
}

contact_constraint_gather_impulses_trusted :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, record: ^Contact_Constraint_Accessor_Record, record_index: int,
) -> [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32
{
	result: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32;
	lane := record_index % util.PRODUCTION_LANE_COUNT;
	impulses := ([^]f32)(type_batch_impulse_bundle(type_batch, record_index));
	for index in 0 ..< int(record.contact_count)
	{
		field := contact_constraint_impulse_offset(record, index);
		result[index] = impulses[field * util.PRODUCTION_LANE_COUNT + lane];
	}
	return result;
}

contact_constraint_gather_impulses :: proc "contextless" (
	solver: ^Solver, accessors: ^Contact_Constraint_Accessors, handle: Constraint_Handle,
) -> ([MAXIMUM_MANIFOLD_CONTACT_COUNT]f32, i32, Physics_Status)
{
	result: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32;
	location, status := solver_resolve(solver, handle);
	if status != .Ok
	{
		return result, 0, status;
	}
	record, accessor_status := contact_constraint_accessor_lookup(accessors, location.type_id);
	if accessor_status != .Ok
	{
		return result, 0, accessor_status;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	if type_batch_index < 0 || type_batch_index >= int(batch.type_batches.length)
	{
		return result, 0, .Invalid_Argument;
	}
	result = contact_constraint_gather_impulses_trusted(
		&batch.type_batches.memory[type_batch_index], record,
		int(location.index_in_type_batch),
	);
	return result, record.contact_count, .Ok;
}

contact_constraint_extract_active_data :: proc "contextless" (
	solver: ^Solver, accessors: ^Contact_Constraint_Accessors,
	handle: Constraint_Handle, target: ^Contact_Constraint_Data_View,
) -> Physics_Status
{
	if target == nil
	{
		return .Invalid_Argument;
	}
	location, resolve_status := solver_resolve(solver, handle);
	if resolve_status != .Ok
	{
		return resolve_status;
	}
	accessor, accessor_status := contact_constraint_accessor_lookup(
		accessors, location.type_id,
	);
	if accessor_status != .Ok
	{
		return .Not_Found;
	}
	type_record, lookup_status := constraint_type_registry_lookup(
		&solver.registry, location.type_id,
	);
	if lookup_status != .Ok || type_record.body_count != accessor.body_count
	{
		return .Invalid_Description;
	}
	impulse_count := int(type_record.impulse_bundle_size) /
		size_of(util.F32x8);
	if impulse_count < 0 ||
		impulse_count > MAXIMUM_INACTIVE_IMPULSE_SCALARS
	{
		return .Capacity_Missing;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(
		batch.type_id_to_batch_index[location.type_id],
	);
	if type_batch_index < 0 ||
		type_batch_index >= int(batch.type_batch_count)
	{
		return .Not_Found;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	index := int(location.index_in_type_batch);
	reference, reference_status := type_batch_read_reference(
		type_batch, solver.bodies, index,
	);
	if reference_status != .Ok
	{
		return reference_status;
	}
	target^ = {
		type_id=location.type_id,
		kind=accessor.kind,
		body_count=accessor.body_count,
		contact_count=accessor.contact_count,
		prestep_size=type_record.description_size,
		impulse_count=i32(impulse_count),
		body_handles=reference.body_handles,
	};
	build_status := type_record.build_description(
		type_batch_prestep_bundle(type_batch, index),
		&target.prestep[0],
		int(location.type_id),
		int(type_record.description_size),
		int(type_record.prestep_bundle_size),
		index % util.PRODUCTION_LANE_COUNT,
	);
	if build_status != .Ok
	{
		return build_status;
	}
	impulse_vectors := ([^]util.F32x8)(
		type_batch_impulse_bundle(type_batch, index),
	);
	lane := index % util.PRODUCTION_LANE_COUNT;
	for impulse_index in 0 ..< impulse_count
	{
		target.impulses[impulse_index] = simd.extract(
			impulse_vectors[impulse_index], lane,
		);
	}
	return .Ok;
}

contact_constraint_extract_inactive_data :: proc "contextless" (
	solver: ^Solver, accessors: ^Contact_Constraint_Accessors,
	source: ^Inactive_Constraint_Record,
	target: ^Contact_Constraint_Data_View,
) -> Physics_Status
{
	if solver == nil || source == nil || target == nil
	{
		return .Invalid_Argument;
	}
	accessor, accessor_status := contact_constraint_accessor_lookup(
		accessors, source.type_id,
	);
	if accessor_status != .Ok
	{
		return .Not_Found;
	}
	type_record, lookup_status := constraint_type_registry_lookup(
		&solver.registry, source.type_id,
	);
	if lookup_status != .Ok
	{
		return .Invalid_Description;
	}
	expected_impulse_count := int(type_record.impulse_bundle_size) /
		size_of(util.F32x8);
	if type_record.body_count != accessor.body_count ||
		source.body_count != accessor.body_count ||
		source.description_size != type_record.description_size ||
		source.impulse_count < 0 ||
		source.impulse_count > MAXIMUM_INACTIVE_IMPULSE_SCALARS ||
		int(source.impulse_count) != expected_impulse_count
	{
		return .Invalid_Description;
	}
	target^ = {
		type_id=source.type_id,
		kind=accessor.kind,
		body_count=accessor.body_count,
		contact_count=accessor.contact_count,
		prestep_size=source.description_size,
		impulse_count=source.impulse_count,
		body_handles=source.body_handles,
	};
	for byte_index in 0 ..< int(source.description_size)
	{
		target.prestep[byte_index] = source.description[byte_index];
	}
	for impulse_index in 0 ..< int(source.impulse_count)
	{
		target.impulses[impulse_index] = source.impulses[impulse_index];
	}
	return .Ok;
}

contact_constraint_redistribute_impulses :: #force_inline proc "contextless" (
	new_features, old_features: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	old_impulses: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32,
	new_contact_count, old_contact_count: int,
) -> [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32
{
	new_impulses: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32;
	unmatched: [MAXIMUM_MANIFOLD_CONTACT_COUNT]u8;
	remaining_old_impulses := old_impulses;
	unmatched_count := 0;
	for new_index in 0 ..< new_contact_count
	{
		matched_old_index := -1;
		for old_index in 0 ..< old_contact_count
		{
			if old_features[old_index] == new_features[new_index]
			{
				new_impulses[new_index] = remaining_old_impulses[old_index];
				remaining_old_impulses[old_index] = 0;
				matched_old_index = old_index;
				break;
			}
		}
		if matched_old_index < 0
		{
			unmatched[new_index] = 1;
			unmatched_count += 1;
		}
	}
	if unmatched_count > 0
	{
		unmatched_impulse := f32(0);
		for old_index in 0 ..< old_contact_count
		{
			unmatched_impulse += remaining_old_impulses[old_index];
		}
		impulse_per_unmatched := unmatched_impulse / f32(unmatched_count);
		for new_index in 0 ..< new_contact_count
		{
			if unmatched[new_index] != 0
			{
				new_impulses[new_index] = impulse_per_unmatched;
			}
		}
	}
	return new_impulses;
}

contact_constraint_scatter_impulses :: proc "contextless" (
	solver: ^Solver, accessors: ^Contact_Constraint_Accessors, handle: Constraint_Handle,
	new_features, old_features: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	old_impulses: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32, old_contact_count: int,
) -> Physics_Status
{
	location, status := solver_resolve(solver, handle);
	if status != .Ok
	{
		return status;
	}
	record, accessor_status := contact_constraint_accessor_lookup(accessors, location.type_id);
	if accessor_status != .Ok
	{
		return accessor_status;
	}
	batch := &solver.active_set.batches.memory[location.batch_index];
	type_batch_index := int(batch.type_id_to_batch_index[location.type_id]);
	if type_batch_index < 0 || type_batch_index >= int(batch.type_batches.length)
	{
		return .Invalid_Argument;
	}
	type_batch := &batch.type_batches.memory[type_batch_index];
	lane := int(location.index_in_type_batch) % util.PRODUCTION_LANE_COUNT;
	impulses := ([^]f32)(type_batch_impulse_bundle(type_batch, int(location.index_in_type_batch)));
	new_impulses := contact_constraint_redistribute_impulses(
		new_features, old_features, old_impulses, int(record.contact_count), old_contact_count,
	);
	for new_index in 0 ..< int(record.contact_count)
	{
		field := contact_constraint_impulse_offset(record, new_index);
		impulses[field * util.PRODUCTION_LANE_COUNT + lane] = new_impulses[new_index];
	}
	return .Ok;
}

contact_constraint_scatter_impulses_trusted :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, record: ^Contact_Constraint_Accessor_Record, record_index: int,
	new_features, old_features: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	old_impulses: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32, old_contact_count: int,
)
{
	lane := record_index % util.PRODUCTION_LANE_COUNT;
	impulses := ([^]f32)(type_batch_impulse_bundle(type_batch, record_index));
	new_impulses := contact_constraint_redistribute_impulses(
		new_features, old_features, old_impulses, int(record.contact_count), old_contact_count,
	);
	for new_index in 0 ..< int(record.contact_count)
	{
		field := contact_constraint_impulse_offset(record, new_index);
		impulses[field * util.PRODUCTION_LANE_COUNT + lane] = new_impulses[new_index];
	}
}

contact_constraint_write_convex_prestep_lane :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, record_index: int,
	manifold: ^Convex_Contact_Manifold, material: Contact_Material_Properties,
	$body_count, $contact_count: int,
)
{
	lane := record_index % util.PRODUCTION_LANE_COUNT;
	target := ([^]f32)(type_batch_prestep_bundle(type_batch, record_index));
	field := 0;
	for contact_index in 0 ..< contact_count
	{
		contact := manifold.contacts[contact_index];
		target[(field + 0) * util.PRODUCTION_LANE_COUNT + lane] = contact.offset.x;
		target[(field + 1) * util.PRODUCTION_LANE_COUNT + lane] = contact.offset.y;
		target[(field + 2) * util.PRODUCTION_LANE_COUNT + lane] = contact.offset.z;
		target[(field + 3) * util.PRODUCTION_LANE_COUNT + lane] = contact.depth;
		field += 4;
	}
	when body_count == 2
	{
		target[(field + 0) * util.PRODUCTION_LANE_COUNT + lane] = manifold.offset_b.x;
		target[(field + 1) * util.PRODUCTION_LANE_COUNT + lane] = manifold.offset_b.y;
		target[(field + 2) * util.PRODUCTION_LANE_COUNT + lane] = manifold.offset_b.z;
		field += 3;
	}
	target[(field + 0) * util.PRODUCTION_LANE_COUNT + lane] = manifold.normal.x;
	target[(field + 1) * util.PRODUCTION_LANE_COUNT + lane] = manifold.normal.y;
	target[(field + 2) * util.PRODUCTION_LANE_COUNT + lane] = manifold.normal.z;
	field += 3;
	target[(field + 0) * util.PRODUCTION_LANE_COUNT + lane] = material.friction_coefficient;
	target[(field + 1) * util.PRODUCTION_LANE_COUNT + lane] = material.spring_settings.angular_frequency;
	target[(field + 2) * util.PRODUCTION_LANE_COUNT + lane] = material.spring_settings.twice_damping_ratio;
	target[(field + 3) * util.PRODUCTION_LANE_COUNT + lane] = material.maximum_recovery_velocity;
}

contact_constraint_convex_features_equal :: #force_inline proc "contextless" (
	old_features: ^[MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	manifold: ^Convex_Contact_Manifold, $contact_count: int,
) -> Contact_Feature_Equality
{
	for contact_index in 0 ..< contact_count
	{
		if old_features[contact_index] != manifold.contacts[contact_index].feature_id
		{
			return .Different;
		}
	}
	return .Equal;
}

contact_constraint_redistribute_convex_impulses_trusted :: #force_inline proc "contextless" (
	type_batch: ^Type_Batch, record_index: int,
	old_features: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	manifold: ^Convex_Contact_Manifold, $contact_count: int,
)
{
	new_features := [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32{-1, -1, -1, -1};
	old_impulses: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32;
	lane := record_index % util.PRODUCTION_LANE_COUNT;
	impulses := ([^]f32)(type_batch_impulse_bundle(type_batch, record_index));
	for contact_index in 0 ..< contact_count
	{
		new_features[contact_index] = manifold.contacts[contact_index].feature_id;
		old_impulses[contact_index] =
			impulses[(2 + contact_index) * util.PRODUCTION_LANE_COUNT + lane];
	}
	new_impulses := contact_constraint_redistribute_impulses(
		new_features, old_features, old_impulses, contact_count, contact_count,
	);
	for contact_index in 0 ..< contact_count
	{
		impulses[(2 + contact_index) * util.PRODUCTION_LANE_COUNT + lane] =
			new_impulses[contact_index];
	}
}

contact_constraint_accessors_dispose :: proc "contextless" (
	accessors: ^Contact_Constraint_Accessors,
) -> Physics_Status
{
	if accessors == nil || accessors.state != .Ready
	{
		return .Disposed;
	}
	accessors^ = {state=.Disposed};
	return .Ok;
}
