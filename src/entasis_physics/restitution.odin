package entasis_physics

import "base:runtime"
import "core:math"
import "core:mem"
import "core:simd"
import util "entasis:entasis_utilities"

// optional contact-response settings, impact history, and packed solve targets
Solver_Response_Mode :: enum u8
{
	Default,
	Restitution,
}

Restitution_Settings :: struct
{
	coefficient: f32,
	threshold: f32,
}

Restitution_Combine_Proc :: #type proc "contextless" (
	user_context: rawptr, pair: Collidable_Pair,
	a, b: Restitution_Settings,
) -> Restitution_Settings;
Restitution_Configuration :: struct
{
	fallback: Restitution_Settings,
	collidable_capacity: i32,
	pair_capacity: i32,
	combine: Restitution_Combine_Proc,
	user_context: rawptr,
}

Restitution_Preparation_State :: enum u8
{
	Idle,
	Preparing,
	Prepared,
}

Restitution_Staging_Mode :: enum u8
{
	Initial,
	Continuation,
}

Restitution_Impact_Phase :: enum u8
{
	Armed,
	Spent,
}

Restitution_Pair_State :: struct
{
	constraint: Constraint_Handle,
	settings: Restitution_Settings,
	features: [MAXIMUM_MANIFOLD_CONTACT_COUNT]i32,
	phases: [MAXIMUM_MANIFOLD_CONTACT_COUNT]Restitution_Impact_Phase,
	// preserve the incoming speed before a speculative normal constraint brakes it.
	// it is consumed only at geometric contact and cleared on departure
	approach: [MAXIMUM_MANIFOLD_CONTACT_COUNT]f32,
	contact_count: u8,
	staged: Reference_State,
}

Restitution_Storage :: struct
{
	simulation: ^Simulation,
	allocator: mem.Allocator,
	scope: util.Allocation_Scope,
	configuration: Restitution_Configuration,
	settings: util.Quick_Dictionary(u64, Restitution_Settings),
	pairs: [2]util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State),
	committed: u8,
	state: Restitution_Preparation_State,
	blocks: util.Buffer(Restitution_Block),
	bundles: util.Buffer(Restitution_Bundle),
	targets: util.Buffer(Restitution_Target_Wide),
	normal_work_count: int,
}

restitution_configuration_default :: proc "contextless" () -> Restitution_Configuration
{
	return {fallback={threshold=1}, collidable_capacity=16, pair_capacity=64};
}

restitution_settings_validate :: proc "contextless" (settings: Restitution_Settings) -> Physics_Status
{
	if !(settings.coefficient >= 0 && settings.coefficient <= 1) ||
	!(settings.threshold >= 0) || math.is_inf(settings.threshold, 0)
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

restitution_configuration_validate :: proc "contextless" (
	configuration: Restitution_Configuration,
) -> Physics_Status
{
	if restitution_settings_validate(configuration.fallback) != .Ok ||
	configuration.collidable_capacity <= 0 || configuration.pair_capacity <= 0
	{
		return .Invalid_Argument;
	}
	// both the value array and the dictionary's two-bucket-per-entry table
	// must fit the existing pool's maximum span. no overflow reaches growth
	limit: int = (1 << util.MAXIMUM_SPAN_SIZE_POWER);
	if i64(configuration.collidable_capacity) > i64(limit / size_of(u64)) ||
	i64(configuration.pair_capacity) > i64(limit / size_of(Restitution_Pair_State))
	{
		return .Capacity_Missing;
	}
	return .Ok;
}

restitution_key :: #force_inline proc "contextless" (reference: Collidable_Reference) -> u64
{
	// mobility changes do not replace a body. static/body namespaces do differ
	kind: u64 = 0;
	if collidable_reference_mobility(reference) == .Static
	{
		kind = u64(1) << 32;
	}
	return kind | u64(u32(collidable_reference_raw_handle(reference)));
}

restitution_key_hash :: proc "contextless" (value: rawptr) -> i32
{
	key: u64 = (^u64)(value)^;
	return i32(u32(key) ~ u32(key >> 32) * 961748927);
}

restitution_key_equal :: proc "contextless" (a, b: rawptr) -> util.Comparison_Status
{
	if (^u64)(a)^ == (^u64)(b)^
	{
		return .Equal;
	}
	return .Different;
}

restitution_combine_default :: proc "contextless" (
	a, b: Restitution_Settings,
) -> Restitution_Settings
{
	return {coefficient=max(a.coefficient, b.coefficient), threshold=max(a.threshold, b.threshold)};
}

restitution_storage_initialize :: proc (
	simulation: ^Simulation, configuration: Restitution_Configuration,
	allocator: mem.Allocator, scope: util.Allocation_Scope,
) -> Physics_Status
{
	if simulation == nil || simulation.state != .Ready || simulation.restitution != nil ||
	(scope != .Legacy && scope != .All_Owned)
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = restitution_configuration_validate(configuration);
	if status != .Ok
	{
		return status;
	}
	// snapshot before invoking a foreign allocator or publishing a bound owner
	copied: Restitution_Configuration = configuration;
	actual: mem.Allocator = allocator;
	if actual.procedure == nil
	{
		actual = runtime.heap_allocator();
	}
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(Restitution_Storage), align_of(Restitution_Storage), actual);
	if error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	storage: ^Restitution_Storage = (^Restitution_Storage)(memory);
	storage^ = {simulation=simulation, allocator=actual, scope=scope, configuration=copied};
	result: util.Collection_Status = util.quick_dictionary_initialize(
		&storage.settings, int(copied.collidable_capacity), simulation.pool,
		{hash=restitution_key_hash, equal=restitution_key_equal}, table_power_offset=1,
	);
	if result == .Ok
	{
		for index in 0 ..< 2
		{
			result = util.quick_dictionary_initialize(
				&storage.pairs[index], int(copied.pair_capacity), simulation.pool,
				{hash=collidable_pair_hash, equal=collidable_pair_equal}, table_power_offset=1,
			);
			if result != .Ok
			{
				break;
			}
		}
	}
	if result != .Ok
	{
		restitution_storage_release(storage);
		return physics_collection_status(result);
	}
	simulation.restitution = storage;
	simulation.bodies.restitution = storage;
	simulation.statics.restitution = storage;
	return .Ok;
}

restitution_storage_release :: proc (storage: ^Restitution_Storage)
{
	pool: ^util.Buffer_Pool = storage.simulation.pool;
	if storage.settings.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.settings, pool);
	}
	for index in 0 ..< 2
	{
		if storage.pairs[index].keys.memory != nil
		{
			_ = util.quick_dictionary_dispose(&storage.pairs[index], pool);
		}
	}
	physics_return_buffer(pool, &storage.blocks);
	physics_return_buffer(pool, &storage.bundles);
	physics_return_buffer(pool, &storage.targets);
	allocator: mem.Allocator = storage.allocator;
	scope: util.Allocation_Scope = storage.scope;
	_ = util.allocation_free(storage, size_of(Restitution_Storage), align_of(Restitution_Storage), allocator, scope);
}

restitution_storage_destroy :: proc (simulation: ^Simulation) -> Physics_Status
{
	if simulation == nil || simulation.restitution == nil
	{
		return .Not_Found;
	}
	storage: ^Restitution_Storage = simulation.restitution;
	if storage.state == .Preparing || simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	simulation.restitution = nil;
	simulation.bodies.restitution = nil;
	simulation.statics.restitution = nil;
	restitution_storage_release(storage);
	return .Ok;
}

restitution_storage_reserve :: proc (
	storage: ^Restitution_Storage, collidable_capacity, pair_capacity: i32,
) -> Physics_Status
{
	if storage == nil || storage.state != .Idle || storage.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	configuration: Restitution_Configuration = storage.configuration;
	configuration.collidable_capacity = collidable_capacity;
	configuration.pair_capacity = pair_capacity;
	status: Physics_Status = restitution_configuration_validate(configuration);
	if status != .Ok
	{
		return status;
	}
	pool: ^util.Buffer_Pool = storage.simulation.pool;
	result: util.Collection_Status = util.quick_dictionary_ensure_capacity(&storage.settings, int(collidable_capacity), pool);
	if result != .Ok
	{
		return physics_collection_status(result);
	}
	for index in 0 ..< 2
	{
		result = util.quick_dictionary_ensure_capacity(&storage.pairs[index], int(pair_capacity), pool);
		if result != .Ok
		{
			return physics_collection_status(result);
		}
	}
	// equal/smaller requests cannot shrink admission or replace sufficient storage
	storage.configuration.collidable_capacity = max(storage.configuration.collidable_capacity, collidable_capacity);
	storage.configuration.pair_capacity = max(storage.configuration.pair_capacity, pair_capacity);
	return .Ok;
}

restitution_storage_lookup :: proc (
	storage: ^Restitution_Storage, reference: Collidable_Reference,
) -> Restitution_Settings
{
	key: u64 = restitution_key(reference);
	value: ^Restitution_Settings;
	presence: util.Presence_Status;
	value, presence = util.quick_dictionary_try_get(&storage.settings, &key);
	if presence == .Present
	{
		return value^;
	}
	return storage.configuration.fallback;
}

restitution_storage_invalidate :: proc (storage: ^Restitution_Storage, key: u64)
{
	// called at committed cold edits/removal, never from pair or solver workers
	for table_index in 0 ..< 2
	{
		table: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[table_index];
		index: int = 0;
		for index < table.count
		{
			pair: Collidable_Pair = table.keys.memory[index];
			if restitution_key(pair.a) == key || restitution_key(pair.b) == key
			{
				_ = util.quick_dictionary_fast_remove(table, &pair);
			}
			else
			{
				index += 1;
			}
		}
	}
	storage.state = .Idle;
}

restitution_storage_set :: proc (
	storage: ^Restitution_Storage, reference: Collidable_Reference, settings: Restitution_Settings,
) -> Physics_Status
{
	if storage == nil || storage.state != .Idle || storage.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = restitution_settings_validate(settings);
	if status != .Ok
	{
		return status;
	}
	resolve: Physics_Status;
	_, resolve = simulation_query_target(storage.simulation, reference);
	if resolve != .Ok
	{
		return resolve;
	}
	key: u64 = restitution_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.settings, &key);
	if index >= 0
	{
		if storage.settings.values.memory[index] == settings
		{
			return .Ok;
		}
		storage.settings.values.memory[index] = settings;
	}
	else
	{
		// explicit reserve owns growth. a warmed set never allocates or relocates
		result: util.Collection_Status = util.quick_dictionary_add_unsafely(&storage.settings, key, settings);
		if result != .Ok
		{
			return physics_collection_status(result);
		}
	}
	restitution_storage_invalidate(storage, key);
	return .Ok;
}

restitution_storage_remove :: proc (
	storage: ^Restitution_Storage, reference: Collidable_Reference,
) -> Physics_Status
{
	if storage == nil || storage.state != .Idle || storage.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	key: u64 = restitution_key(reference);
	result: util.Collection_Status = util.quick_dictionary_fast_remove(&storage.settings, &key);
	if result != .Ok
	{
		return physics_collection_status(result);
	}
	restitution_storage_invalidate(storage, key);
	return .Ok;
}

restitution_storage_retire :: proc (
	storage: ^Restitution_Storage, reference: Collidable_Reference,
)
{
	key: u64 = restitution_key(reference);
	_ = util.quick_dictionary_fast_remove(&storage.settings, &key);
	restitution_storage_invalidate(storage, key);
}

restitution_storage_clear_kind :: proc (storage: ^Restitution_Storage, mobility: Body_Mobility)
{
	kind: u64 = 0;
	if mobility == .Static
	{
		kind = u64(1) << 32;
	}
	index: int = 0;
	for index < storage.settings.count
	{
		key: u64 = storage.settings.keys.memory[index];
		if key & (u64(1) << 32) == kind
		{
			_ = util.quick_dictionary_fast_remove(&storage.settings, &key);
		}
		else
		{
			index += 1;
		}
	}
	for table_index in 0 ..< 2
	{
		table: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[table_index];
		index = 0;
		for index < table.count
		{
			pair: Collidable_Pair = table.keys.memory[index];
			if restitution_key(pair.a) & (u64(1) << 32) == kind || restitution_key(pair.b) & (u64(1) << 32) == kind
			{
				_ = util.quick_dictionary_fast_remove(table, &pair);
			}
			else
			{
				index += 1;
			}
		}
	}
	storage.state = .Idle;
}

restitution_stage_pair :: proc (
	storage: ^Restitution_Storage, pair: Collidable_Pair, cache: ^Constraint_Cache,
	source: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State),
) -> Physics_Status
{
	a: Restitution_Settings = restitution_storage_lookup(storage, pair.a);
	b: Restitution_Settings = restitution_storage_lookup(storage, pair.b);
	settings: Restitution_Settings = restitution_combine_default(a, b);
	if storage.configuration.combine != nil
	{
		settings = storage.configuration.combine(storage.configuration.user_context, pair, a, b);
		status: Physics_Status = restitution_settings_validate(settings);
		if status != .Ok
		{
			return status;
		}
	}
	if settings.coefficient == 0
	{
		return .Ok;
	}
	location: Constraint_Location = storage.simulation.solver.handle_to_constraint.memory[cache.constraint_handle.value];
	if location.type_id < CONTACT_1_ONE_BODY_TYPE_ID || location.type_id > CONTACT_4_NONCONVEX_TYPE_ID
	{
		return .Invalid_Description;
	}
	accessor: ^Contact_Constraint_Accessor_Record = &storage.simulation.narrow_phase.accessors.records[location.type_id];
	if accessor.registration != .Present ||
	location.type_id != contact_constraint_type_id(int(accessor.body_count), int(accessor.contact_count), accessor.kind)
	{
		return .Invalid_Description;
	}
	value: Restitution_Pair_State = {constraint=cache.constraint_handle, settings=settings, features=cache.feature_ids, contact_count=u8(accessor.contact_count), staged=.Present};
	pair_key: Collidable_Pair = pair;
	previous: ^Restitution_Pair_State;
	presence: util.Presence_Status;
	previous, presence = util.quick_dictionary_try_get(source, &pair_key);
	if presence == .Present && previous.constraint == value.constraint && previous.settings == settings
	{
		for index in 0 ..< int(value.contact_count)
		{
			for old_index in 0 ..< int(previous.contact_count)
			{
				if value.features[index] == previous.features[old_index]
				{
					value.phases[index] = previous.phases[old_index];
					value.approach[index] = previous.approach[old_index];
					break;
				}
			}
		}
	}
	output: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	existing: ^Restitution_Pair_State;
	found: util.Presence_Status;
	existing, found = util.quick_dictionary_try_get(output, &pair_key);
	if found == .Present
	{
		existing^ = value;
		return .Ok;
	}
	return physics_collection_status(util.quick_dictionary_add_unsafely(output, pair, value));
}

restitution_storage_prepare :: #force_no_inline proc (storage: ^Restitution_Storage) -> Physics_Status
{
	return restitution_storage_prepare_internal(storage, .Initial);
}

restitution_storage_prepare_internal :: #force_no_inline proc (storage: ^Restitution_Storage, mode: Restitution_Staging_Mode) -> Physics_Status
{
	if storage == nil || (mode == .Initial && storage.state != .Idle) || (mode == .Continuation && storage.state != .Prepared)
	{
		return .Invalid_Argument;
	}
	previous_state: Simulation_State = storage.simulation.state;
	if previous_state != .Ready && previous_state != .Stepping
	{
		return .Invalid_Argument;
	}
	storage.simulation.state = .Stepping;
	defer storage.simulation.state = previous_state;
	storage.state = .Preparing;
	pending: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[1-storage.committed];
	source: ^util.Quick_Dictionary(Collidable_Pair, Restitution_Pair_State) = &storage.pairs[storage.committed];
	if mode == .Initial
	{
		util.quick_dictionary_clear(pending);
	}
	else
	{
		source = pending;
		for index in 0 ..< pending.count
		{
			pending.values.memory[index].staged = .Missing;
		}
	}
	cache: ^Pair_Cache = &storage.simulation.narrow_phase.pair_cache;
	for index in 0 ..< cache.mapping.count
	{
		status: Physics_Status = restitution_stage_pair(storage, cache.mapping.keys.memory[index], &cache.mapping.values.memory[index], source);
		if status != .Ok
		{
			util.quick_dictionary_clear(pending);
			storage.state = .Idle;
			return status;
		}
	}
	// sleeping is not a new impact. retain the same feature state without
	// invoking the application combine callback on unchanged dormant pairs
	for index in 0 ..< cache.inactive_count
	{
		entry: ^Inactive_Pair_Cache_Entry = &cache.inactive_entries.memory[index];
		previous: ^Restitution_Pair_State;
		presence: util.Presence_Status;
		previous, presence = util.quick_dictionary_try_get(source, &entry.pair);
		if presence == .Present && previous.constraint == entry.cache.constraint_handle
		{
			if mode == .Continuation
			{
				previous.staged = .Present;
				continue;
			}
			value: Restitution_Pair_State = previous^;
			value.staged = .Present;
			result: util.Collection_Status = util.quick_dictionary_add_unsafely(pending, entry.pair, value);
			if result != .Ok
			{
				util.quick_dictionary_clear(pending);
				storage.state = .Idle;
				return physics_collection_status(result);
			}
		}
	}
	if mode == .Continuation
	{
		index: int = 0;
		for index < pending.count
		{
			if pending.values.memory[index].staged == .Missing
			{
				key: Collidable_Pair = pending.keys.memory[index];
				_ = util.quick_dictionary_fast_remove(pending, &key);
			}
			else
			{
				index += 1;
			}
		}
	}
	storage.state = .Prepared;
	return .Ok;
}

restitution_storage_complete :: #force_no_inline proc (
	storage: ^Restitution_Storage, status: Physics_Status,
) -> Physics_Status
{
	if storage == nil || storage.state != .Prepared
	{
		return .Invalid_Argument;
	}
	if status == .Ok
	{
		storage.committed = 1-storage.committed;
	}
	else
	{
		util.quick_dictionary_clear(&storage.pairs[1-storage.committed]);
	}
	storage.state = .Idle;
	return .Ok;
}

// optional packed contact data. the selected block owner supplies eight
// initialized lanes, valid unit normals and disjoint body/output storage. inactive
// lanes have mask zero. coefficients and thresholds were validated at ingestion.
// no ordinary contact record or registered callback representation contains this
Restitution_Target_Wide :: struct
{
	velocity: util.F32x8,
	active_mask: util.I32x8,
}

restitution_normal_velocity :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	$body_count: int,
) -> util.F32x8
{
	#assert(body_count == 1 || body_count == 2);
	angular_a: util.Vector3_Wide = util.vector3_wide_cross(offset_a, normal);
	velocity: util.F32x8 = simd.add(
		util.vector3_wide_dot(bodies[0].linear_velocity, normal),
		util.vector3_wide_dot(bodies[0].angular_velocity, angular_a),
	);
	when body_count == 2
	{
		angular_b: util.Vector3_Wide = util.vector3_wide_cross(normal, offset_b);
		velocity = simd.add(
			simd.sub(velocity, util.vector3_wide_dot(bodies[1].linear_velocity, normal)),
			util.vector3_wide_dot(bodies[1].angular_velocity, angular_b),
		);
	}
	return velocity;
}

restitution_capture_normal :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	depth, coefficient, threshold: util.F32x8, eligible_mask: util.I32x8,
	$body_count: int,
) -> Restitution_Target_Wide
{
	velocity: util.F32x8 = restitution_normal_velocity(bodies, normal, offset_a, offset_b, body_count);
	return restitution_capture_target(velocity, depth, coefficient, threshold, eligible_mask);
}

restitution_capture_target :: #force_inline proc "contextless" (
	velocity, depth, coefficient, threshold: util.F32x8, eligible_mask: util.I32x8,
) -> Restitution_Target_Wide
{
	mask: util.I32x8 = eligible_mask &
	transmute(util.I32x8)simd.lanes_gt(coefficient, util.F32x8(0)) &
	transmute(util.I32x8)simd.lanes_ge(depth, util.F32x8(0)) &
	transmute(util.I32x8)simd.lanes_lt(velocity, simd.neg(threshold));
	return {
		velocity=util.wide_select_f32(mask, simd.mul(simd.neg(coefficient), velocity), util.F32x8(0)),
		active_mask=mask,
	};
}

// this is preparation, not another impulse solver. selected impact lanes use a
// rigid normal target. applying spring softness to impact velocity would attenuate
// the authored restitution coefficient. other lanes preserve ordinary compliant
// contact coefficients, including negative speculative bias. existing cached
// penetration solving consumes these values without a bounce branch in its loop
restitution_prepare_normal :: #force_inline proc "contextless" (
	bodies: [^]$Body, normal, offset_a, offset_b: util.Vector3_Wide,
	depth, position_error_to_velocity, effective_mass_scale, maximum_recovery_velocity,
	contact_softness: util.F32x8, inverse_dt: f32,
	target: ^Restitution_Target_Wide, $body_count: int,
) -> (effective_mass, bias, softness: util.F32x8)
{
	#assert(body_count == 1 || body_count == 2);
	angular_a: util.Vector3_Wide = util.vector3_wide_cross(offset_a, normal);
	angular_b: util.Vector3_Wide;
	when body_count == 2
	{
		angular_b = util.vector3_wide_cross(normal, offset_b);
	}
	scale: util.F32x8 = util.wide_select_f32(target.active_mask, util.F32x8(1), effective_mass_scale);
	effective_mass, bias = constraint_contact_penetration_prepare_solve(
		bodies, angular_a, angular_b, depth, position_error_to_velocity,
		scale, maximum_recovery_velocity, inverse_dt, body_count,
	);
	bias = util.wide_select_f32(target.active_mask, simd.max(bias, target.velocity), bias);
	softness = util.wide_select_f32(target.active_mask, util.F32x8(0), contact_softness);
	return;
}
