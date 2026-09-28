package entasis_physics

import "core:mem"
import "base:runtime"
import util "entasis:entasis_utilities"

// per-instance metadata. shared geometry and body/SIMD records are untouched
Collider_Part_Role :: enum u8
{
	Solid,
	Trigger,
}

Collider_Part_Option :: enum u8
{
	Stay,
	Static_Static,
}

Collider_Part_Options :: bit_set[Collider_Part_Option; u8];
Collider_Part_Settings :: struct
{
	user_id: u64,
	role: Collider_Part_Role,
	options: Collider_Part_Options,
}

Collider_Part_Configuration :: struct
{
	instance_capacity, part_capacity: i32,
	pair_capacity, observations_per_worker: i32,
	event_subscription: Collider_Part_Event_Subscription,
}

Collider_Part_Identity :: struct
{
	instance, incarnation, serial: u64,
	child_index: i32,
}

Collider_Part_Info :: struct
{
	identity: Collider_Part_Identity,
	settings: Collider_Part_Settings,
}

Collider_Part_Key :: struct
{
	incarnation: u64,
	child_index: i32,
}

Collider_Instance_Record :: struct
{
	shape: Typed_Index,
	incarnation: u64,
	part_count, configured_count, trigger_count: i32,
	options: Collider_Part_Options,
}

Collider_Part_Record :: struct
{
	serial: u64,
	settings: Collider_Part_Settings,
	binding: Reference_State,
}

Mixed_Collider_Storage :: struct
{
	owner: ^Trigger_System,
	trigger_count: int,
	instances: util.Quick_Dictionary(u64, Collider_Instance_Record),
	parts: util.Quick_Dictionary(Collider_Part_Key, Collider_Part_Record),
	configuration: Collider_Part_Configuration,
	workers: util.Buffer(Mixed_Collider_Worker),
	previous, current: util.Buffer(Collider_Part_Pair),
	pair_slots: util.Buffer(i32),
	events: util.Buffer(Collider_Part_Event),
	previous_count, current_count, event_count: int,
}

collider_part_key_hash :: proc "contextless" (data: rawptr) -> i32
{
	key: ^Collider_Part_Key = (^Collider_Part_Key)(data);
	value: u64 = key.incarnation * u64(961748927) + u64(u32(key.child_index)) * u64(899809343);
	return i32(u32(value ~ (value >> 32)));
}

collider_part_key_equal :: proc "contextless" (a, b: rawptr) -> util.Comparison_Status
{
	lhs: ^Collider_Part_Key = (^Collider_Part_Key)(a);
	rhs: ^Collider_Part_Key = (^Collider_Part_Key)(b);
	if lhs.incarnation == rhs.incarnation && lhs.child_index == rhs.child_index
	{
		return .Equal;
	}
	return .Different;
}

collider_instance_key :: #force_inline proc "contextless" (reference: Collidable_Reference) -> u64
{
	return u64(u32(collidable_reference_raw_handle(reference))) |
	(u64(1) << 32 if collidable_reference_mobility(reference) == .Static else 0);
}

collider_part_configuration_validate :: proc "contextless" (configuration: Collider_Part_Configuration) -> Physics_Status
{
	if configuration.instance_capacity <= 0 || configuration.part_capacity <= 0 ||
	configuration.instance_capacity > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(Collider_Instance_Record) ||
	configuration.part_capacity > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(Collider_Part_Record)
	{
		return .Invalid_Argument;
	}
	if configuration.event_subscription != .Disabled && configuration.event_subscription != .Enabled
	{
		return .Invalid_Argument;
	}
	if configuration.event_subscription == .Enabled &&
	(configuration.pair_capacity <= 0 || configuration.observations_per_worker <= 0 ||
		configuration.pair_capacity > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / (2*size_of(Collider_Part_Event)) ||
		configuration.observations_per_worker > (1 << util.MAXIMUM_SPAN_SIZE_POWER) / size_of(Mixed_Collider_Observation))
	{
		return .Invalid_Argument;
	}
	return .Ok;
}

mixed_colliders_release :: proc (storage: ^Mixed_Collider_Storage)
{
	owner: ^Trigger_System = storage.owner;
	pool: ^util.Buffer_Pool = owner.simulation.pool;
	mixed_colliders_release_events(storage);
	if storage.parts.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.parts, pool);
	}
	if storage.instances.keys.memory != nil
	{
		_ = util.quick_dictionary_dispose(&storage.instances, pool);
	}
	_ = util.allocation_free(storage, size_of(Mixed_Collider_Storage), align_of(Mixed_Collider_Storage), owner.allocator, owner.scope);
}

mixed_colliders_initialize :: proc (owner: ^Trigger_System, configuration: Collider_Part_Configuration) -> Physics_Status
{
	if owner == nil || owner.simulation.state != .Ready || owner.mixed != nil
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = collider_part_configuration_validate(configuration);
	if status != .Ok
	{
		return status;
	}
	// an application allocator must not reenter through a Ready native world
	owner.simulation.state = .Stepping;
	defer owner.simulation.state = .Ready;
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(Mixed_Collider_Storage), align_of(Mixed_Collider_Storage), owner.allocator);
	if error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	storage: ^Mixed_Collider_Storage = (^Mixed_Collider_Storage)(memory);
	storage^ = {owner=owner, configuration=configuration};
	result: util.Collection_Status = util.quick_dictionary_initialize(&storage.instances,
		int(configuration.instance_capacity), owner.simulation.pool, util.primitive_u64_hash_equal(), 1);
	if result == .Ok
	{
		result = util.quick_dictionary_initialize(&storage.parts, int(configuration.part_capacity),
			owner.simulation.pool, {hash=collider_part_key_hash, equal=collider_part_key_equal}, 1);
	}
	if result != .Ok
	{
		mixed_colliders_release(storage);
		return physics_collection_status(result);
	}
	status = mixed_colliders_reserve_events(storage, configuration);
	if status!=.Ok
	{
		mixed_colliders_release(storage);
		return status;
	}
	owner.mixed = storage;
	return .Ok;
}

mixed_colliders_reserve :: proc (storage: ^Mixed_Collider_Storage, configuration: Collider_Part_Configuration) -> Physics_Status
{
	if storage == nil || storage.owner.simulation.state != .Ready ||
	configuration.event_subscription != storage.configuration.event_subscription
	{
		return .Invalid_Argument;
	}
	status: Physics_Status = collider_part_configuration_validate(configuration);
	if status != .Ok
	{
		return status;
	}
	storage.owner.simulation.state = .Stepping;
	defer storage.owner.simulation.state = .Ready;
	result: util.Collection_Status = util.quick_dictionary_ensure_capacity(&storage.instances,
		int(configuration.instance_capacity), storage.owner.simulation.pool);
	if result == .Ok
	{
		result = util.quick_dictionary_ensure_capacity(&storage.parts, int(configuration.part_capacity), storage.owner.simulation.pool);
	}
	if result!=.Ok
	{
		return physics_collection_status(result);
	}
	grown:Collider_Part_Configuration=configuration;
	grown.instance_capacity=max(grown.instance_capacity, storage.configuration.instance_capacity);
	grown.part_capacity=max(grown.part_capacity, storage.configuration.part_capacity);
	grown.pair_capacity=max(grown.pair_capacity, storage.configuration.pair_capacity);
	grown.observations_per_worker=max(grown.observations_per_worker, storage.configuration.observations_per_worker);
	status = mixed_colliders_reserve_events(storage, grown);
	if status==.Ok
	{
		storage.configuration=grown;
	}
	return status;
}

// called at committed removal/shape replacement, never from collision workers.
// retired serials remain in value-copied history. raw handle reuse cannot revive them
mixed_colliders_retire :: proc (storage: ^Mixed_Collider_Storage, key: u64)
{
	lookup: u64 = key;
	index: int = util.quick_dictionary_index_of(&storage.instances, &lookup);
	if index < 0
	{
		return;
	}
	incarnation: u64 = storage.instances.values.memory[index].incarnation;
	storage.trigger_count -= int(storage.instances.values.memory[index].trigger_count);
	cursor: int = storage.parts.count;
	for cursor > 0
	{
		cursor -= 1;
		part_key: Collider_Part_Key = storage.parts.keys.memory[cursor];
		if part_key.incarnation == incarnation
		{
			_ = util.quick_dictionary_fast_remove(&storage.parts, &part_key);
		}
	}
	_ = util.quick_dictionary_fast_remove(&storage.instances, &lookup);
}

mixed_colliders_changed :: proc (storage: ^Mixed_Collider_Storage, key: u64, removed: Reference_State)
{
	if removed == .Present
	{
		mixed_colliders_retire(storage, key);
		return;
	}
	lookup: u64 = key;
	index: int = util.quick_dictionary_index_of(&storage.instances, &lookup);
	if index < 0
	{
		return;
	}
	reference: Collidable_Reference;
	status: Physics_Status;
	reference, status = trigger_reference(storage.owner, key);
	if status != .Ok
	{
		mixed_colliders_retire(storage, key);
		return;
	}
	target: Shape_Query_Target;
	target, status = simulation_query_target(storage.owner.simulation, reference);
	if status != .Ok || target.shape.packed != storage.instances.values.memory[index].shape.packed
	{
		mixed_colliders_retire(storage, key);
	}
}

mixed_colliders_clear :: proc (storage: ^Mixed_Collider_Storage)
{
	util.quick_dictionary_clear(&storage.parts);
	util.quick_dictionary_clear(&storage.instances);
	storage.previous_count=0;
	storage.current_count=0;
	storage.event_count=0;
	storage.trigger_count=0;
}

mixed_colliders_membership :: proc (storage: ^Mixed_Collider_Storage, reference: Collidable_Reference) -> Reference_State
{
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index >= 0 && storage.instances.values.memory[index].configured_count > 0
	{
		return .Present;
	}
	return .Missing;
}

collider_part_count :: proc "contextless" (shapes: ^Shape_Registry, shape: Typed_Index) -> (i32, Physics_Status)
{
	data: rawptr;
	batch: ^Shape_Batch;
	status: Physics_Status;
	data, batch, status = shape_registry_resolve(shapes, shape);
	if status != .Ok
	{
		return 0, status;
	}
	switch typed_index_type(shape)
	{
		case COMPOUND_TYPE_ID: return (^Compound)(data).children.length, .Ok;
		case BIG_COMPOUND_TYPE_ID: return (^Big_Compound)(data).children.length, .Ok;
		case MESH_TYPE_ID: return 1, .Ok;
		case:
		if batch.metadata.batch_type != .Convex
		{
			return 0, .Invalid_Description;
		}
		return 1, .Ok;
	}
}

mixed_colliders_part_set :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference,
	child_index: i32, settings: Collider_Part_Settings,
) -> (Collider_Part_Info, Physics_Status)
{
	if storage == nil || storage.owner.simulation.state != .Ready ||
	(settings.role != .Solid && settings.role != .Trigger) ||
	(settings.options - Collider_Part_Options{.Stay, .Static_Static}) != {}
	{
		return {}, .Invalid_Argument;
	}
	owner: ^Trigger_System = storage.owner;
	target: Shape_Query_Target;
	status: Physics_Status;
	target, status = simulation_query_target(owner.simulation, reference);
	if status != .Ok
	{
		return {}, status;
	}
	count: i32;
	count, status = collider_part_count(simulation_shape_registry(owner.simulation), target.shape);
	if status != .Ok
	{
		return {}, status;
	}
	if child_index < 0 || child_index >= count
	{
		return {}, .Invalid_Argument;
	}
	slot: ^Trigger_Slot = trigger_slot(owner, reference);
	if slot == nil
	{
		return {}, .Capacity_Missing;
	}
	if slot.enabled == .Enabled
	{
		return {}, .Invalid_Argument;
	}
	key: u64 = collider_instance_key(reference);
	instance_index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	instance: Collider_Instance_Record;
	if instance_index >= 0
	{
		instance = storage.instances.values.memory[instance_index];
	}
	else if storage.instances.count == int(storage.instances.keys.length)
	{
		return {}, .Capacity_Missing;
	}
	part_key: Collider_Part_Key = {incarnation=instance.incarnation, child_index=child_index};
	part_index: int = -1;
	if instance_index >= 0
	{
		part_index = util.quick_dictionary_index_of(&storage.parts, &part_key);
	}
	if part_index < 0 && storage.parts.count == int(storage.parts.keys.length)
	{
		return {}, .Capacity_Missing;
	}
	required: u64 = u64(1) if slot.token == 0 else 0;
	if instance_index < 0
	{
		required += 1;
	}
	if part_index < 0
	{
		required += 1;
	}
	if owner.serial > max(u64) - required
	{
		return {}, .Capacity_Missing;
	}
	// retire old physical contacts before publishing a role change. user joints stay
	if settings.role == .Trigger || (part_index >= 0 && storage.parts.values.memory[part_index].settings.role == .Trigger)
	{
		owner.simulation.state = .Stepping;
		status = trigger_retire_contacts(owner, reference);
		owner.simulation.state = .Ready;
		if status != .Ok
		{
			return {}, status;
		}
	}
	if slot.token == 0
	{
		owner.serial += 1;
		slot.token = owner.serial;
	}
	if instance_index < 0
	{
		owner.serial += 1;
		instance = {shape=target.shape, incarnation=owner.serial, part_count=count};
		part_key.incarnation = instance.incarnation;
	}
	part: Collider_Part_Record;
	if part_index >= 0
	{
		part = storage.parts.values.memory[part_index];
		if part.binding==.Missing
		{
			instance.configured_count+=1;
		}
		if part.settings.role == .Trigger
		{
			instance.trigger_count -= 1;
			storage.trigger_count -= 1;
		}
	}
	else
	{
		owner.serial += 1;
		part.serial = owner.serial;
		instance.configured_count += 1;
	}
	part.settings = settings;
	part.binding = .Present;
	if settings.role == .Trigger
	{
		instance.trigger_count += 1;
		storage.trigger_count += 1;
	}
	if instance_index >= 0
	{
		storage.instances.values.memory[instance_index] = instance;
	}
	else
	{
		_ = util.quick_dictionary_add_unsafely(&storage.instances, key, instance);
	}
	if part_index >= 0
	{
		storage.parts.values.memory[part_index] = part;
	}
	else
	{
		_ = util.quick_dictionary_add_unsafely(&storage.parts, part_key, part);
	}
	mixed_colliders_refresh_options(storage, &storage.instances.values.memory[util.quick_dictionary_index_of(&storage.instances, &key)]);
	trigger_changed(owner, .Static if collidable_reference_mobility(reference) == .Static else .Body, collidable_reference_raw_handle(reference), .Modified);
	return {{instance=slot.token, incarnation=instance.incarnation, serial=part.serial, child_index=child_index}, settings}, .Ok;
}

mixed_colliders_part_get :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference, child_index: i32,
) -> (Collider_Part_Info, Physics_Status)
{
	if storage == nil || storage.owner.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index < 0
	{
		return {}, .Not_Found;
	}
	instance: ^Collider_Instance_Record = &storage.instances.values.memory[index];
	part_key: Collider_Part_Key = {incarnation=instance.incarnation, child_index=child_index};
	part_index: int = util.quick_dictionary_index_of(&storage.parts, &part_key);
	if part_index < 0
	{
		return {}, .Not_Found;
	}
	part: ^Collider_Part_Record = &storage.parts.values.memory[part_index];
	if part.binding!=.Present
	{
		return {}, .Not_Found;
	}
	return {{instance=trigger_slot(storage.owner, reference).token, incarnation=instance.incarnation,
			serial=part.serial, child_index=child_index}, part.settings}, .Ok;
}

mixed_colliders_part_remove :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference, child_index: i32,
) -> Physics_Status
{
	if storage == nil || storage.owner.simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index < 0
	{
		return .Not_Found;
	}
	instance: ^Collider_Instance_Record = &storage.instances.values.memory[index];
	part_key: Collider_Part_Key = {incarnation=instance.incarnation, child_index=child_index};
	part_index: int = util.quick_dictionary_index_of(&storage.parts, &part_key);
	if part_index < 0 || storage.parts.values.memory[part_index].binding!=.Present
	{
		return .Not_Found;
	}
	if storage.parts.values.memory[part_index].settings.role == .Trigger
	{
		instance.trigger_count -= 1;
		storage.trigger_count -= 1;
	}
	instance.configured_count -= 1;
	_ = util.quick_dictionary_fast_remove(&storage.parts, &part_key);
	mixed_colliders_refresh_options(storage, instance);
	// keep the incarnation until geometry changes. remove/rebind still receives a new serial
	trigger_changed(storage.owner, .Static if collidable_reference_mobility(reference) == .Static else .Body, collidable_reference_raw_handle(reference), .Modified);
	return .Ok;
}

mixed_colliders_sensor :: proc (storage: ^Mixed_Collider_Storage, reference: Collidable_Reference) -> Reference_State
{
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index >= 0 && storage.instances.values.memory[index].trigger_count > 0
	{
		return .Present;
	}
	return .Missing;
}

mixed_colliders_part_settings :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference, child_index: i32,
) -> Collider_Part_Settings
{
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index < 0
	{
		return {};
	}
	instance: ^Collider_Instance_Record = &storage.instances.values.memory[index];
	type_id: i32 = typed_index_type(instance.shape);
	part_key: Collider_Part_Key = {incarnation=instance.incarnation,
		child_index=child_index if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID else 0};
	part_index: int = util.quick_dictionary_index_of(&storage.parts, &part_key);
	if part_index < 0
	{
		return {};
	}
	return storage.parts.values.memory[part_index].settings;
}

mixed_colliders_sensor_options :: proc (storage: ^Mixed_Collider_Storage, reference: Collidable_Reference) -> Collider_Part_Options
{
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index < 0
	{
		return {};
	}
	return storage.instances.values.memory[index].options;
}

mixed_colliders_refresh_options :: proc (storage: ^Mixed_Collider_Storage, instance: ^Collider_Instance_Record)
{
	instance.options = {};
	// a cold edit visits this instance's parts, not every configured world part
	for index:i32=0;index<instance.part_count;index+=1
	{
		key:Collider_Part_Key={incarnation=instance.incarnation, child_index=index};
		lookup:int=util.quick_dictionary_index_of(&storage.parts, &key);
		if lookup<0
		{
			continue;
		}
		part:^Collider_Part_Record=&storage.parts.values.memory[lookup];
		if part.settings.role==.Trigger
		{
			instance.options+=part.settings.options;
		}
	}
}

mixed_colliders_solid_admission :: proc (storage: ^Mixed_Collider_Storage, reference: Collidable_Reference) -> Reference_State
{
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	if index >= 0 && storage.instances.values.memory[index].trigger_count == storage.instances.values.memory[index].part_count
	{
		return .Missing;
	}
	return .Present;
}

// selected only on mixed-enabled workers. unselected workers keep the original
// allow-child procedure and numeric kernels, without an added membership load
mixed_narrow_allow_child :: proc "contextless" (
	user: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	context = runtime.default_context();
	worker: ^Narrow_Phase_Worker_Context = (^Narrow_Phase_Worker_Context)(user);
	pair: Broad_Phase_Pair = worker.narrow.candidates.memory[pair_id];
	storage: ^Mixed_Collider_Storage = worker.narrow.bodies.triggers.mixed;
	a: Collider_Part_Settings = mixed_colliders_part_settings(storage, pair.a, mixed_colliders_part_index(storage, pair.a, child_a));
	b: Collider_Part_Settings = mixed_colliders_part_settings(storage, pair.b, mixed_colliders_part_index(storage, pair.b, child_b));
	if a.role == .Trigger || b.role == .Trigger
	{
		return .Reject;
	}
	if narrow_phase_allow_child(user, pair_id, child_a, child_b)!=.Allow
	{
		return .Reject;
	}
	status:Physics_Status=mixed_colliders_child_route(storage.owner.simulation, pair, child_a, child_b);
	if status!=.Ok
	{
		worker.narrow.transaction_statuses[worker.worker_index]=status;
		return .Reject;
	}
	return .Allow;
}

mixed_trigger_allow_child :: proc "contextless" (
	user: rawptr, pair_id, child_a, child_b: i32,
) -> Collision_Testing_State
{
	context = runtime.default_context();
	worker: ^Trigger_Worker = (^Trigger_Worker)(user);
	pair: Broad_Phase_Pair = worker.candidates[pair_id];
	owner: ^Trigger_System = worker.owner;
	a: Collider_Part_Settings = mixed_colliders_part_settings(owner.mixed, pair.a, mixed_colliders_part_index(owner.mixed, pair.a, child_a));
	b: Collider_Part_Settings = mixed_colliders_part_settings(owner.mixed, pair.b, mixed_colliders_part_index(owner.mixed, pair.b, child_b));
	slot_a: ^Trigger_Slot = trigger_slot(owner, pair.a);
	slot_b: ^Trigger_Slot = trigger_slot(owner, pair.b);
	if slot_a.enabled == .Disabled && slot_b.enabled == .Disabled && a.role != .Trigger && b.role != .Trigger
	{
		return .Reject;
	}
	if collidable_reference_mobility(pair.a)==.Static && collidable_reference_mobility(pair.b)==.Static &&
	!(slot_a.enabled == .Enabled && slot_a.settings.static_static == .Enabled) && !(slot_b.enabled == .Enabled && slot_b.settings.static_static == .Enabled) &&
	!(a.role==.Trigger && .Static_Static in a.options) && !(b.role==.Trigger && .Static_Static in b.options)
	{
		return .Reject;
	}
	if trigger_allow_child(user, pair_id, child_a, child_b)!=.Allow
	{
		return .Reject;
	}
	status:Physics_Status=mixed_colliders_child_route(owner.simulation, pair, child_a, child_b);
	if status!=.Ok
	{
		worker.status=status;
		return .Reject;
	}
	return .Allow;
}

// existing static mutation awakens intersecting bounds, not contact manifolds.
// restrict those bounds to the actual solid parts, retaining old/new pose unions
Mixed_Static_Awakening :: struct
{
	storage: ^Mixed_Collider_Storage,
	reference: Collidable_Reference,
	old, next: Static_Description,
	filter: Static_Awakening_Filter_Proc,
	user: rawptr,
	status: Physics_Status,
}

mixed_collider_part_bounds :: proc "contextless" (
	registry: ^Shape_Registry, shape: Typed_Index, pose: Rigid_Pose, index: i32,
) -> (util.Bounding_Box, Physics_Status)
{
	type_id: i32 = typed_index_type(shape);
	if index < 0 || (type_id != COMPOUND_TYPE_ID && type_id != BIG_COMPOUND_TYPE_ID)
	{
		return shape_registry_compute_world_bounds(registry, shape, pose);
	}
	data: rawptr;
	status: Physics_Status;
	data, _, status = shape_registry_resolve(registry, shape);
	if status != .Ok
	{
		return {}, status;
	}
	children: util.Buffer(Compound_Child);
	if type_id == COMPOUND_TYPE_ID
	{
		children = (^Compound)(data).children;
	}
	else
	{
		children = (^Big_Compound)(data).children;
	}
	if index >= children.length
	{
		return {}, .Invalid_Argument;
	}
	child: ^Compound_Child = &children.memory[index];
	child_pose: Rigid_Pose = rigid_pose_concatenate({position=child.local_position, orientation=child.local_orientation}, pose);
	return shape_registry_compute_world_bounds(registry, child.shape_index, child_pose);
}

mixed_static_awaken_filter :: proc "contextless" (user: rawptr, body: Body_Handle) -> bool
{
	context = runtime.default_context();
	state: ^Mixed_Static_Awakening = (^Mixed_Static_Awakening)(user);
	if state.status != .Ok
	{
		return false;
	}
	simulation: ^Simulation = state.storage.owner.simulation;
	registry: ^Shape_Registry = simulation_shape_registry(simulation);
	description: Body_Description;
	status: Physics_Status;
	description, status = bodies_get_description(&simulation.bodies, body);
	if status != .Ok
	{
		state.status = status;
		return false;
	}
	body_reference: Collidable_Reference;
	body_reference, _ = collidable_reference_body(.Dynamic, body);
	body_parts: Reference_State = mixed_colliders_membership(state.storage, body_reference);
	static_parts: Reference_State = mixed_colliders_membership(state.storage, state.reference);
	body_count: i32 = 1;
	static_count: i32 = 1;
	if body_parts == .Present
	{
		body_count, status = collider_part_count(registry, description.collidable.shape);
		if status != .Ok
		{
			state.status = status;
			return false;
		}
	}
	if static_parts == .Present
	{
		static_count, status = collider_part_count(registry, state.old.shape);
		if status != .Ok
		{
			state.status = status;
			return false;
		}
	}
	for body_index: i32 = 0; body_index < body_count; body_index += 1
	{
		body_part: i32 = body_index if body_parts == .Present else -1;
		if body_parts == .Present && mixed_colliders_part_settings(state.storage, body_reference, body_part).role == .Trigger
		{
			continue;
		}
		body_bounds: util.Bounding_Box;
		body_bounds, status = mixed_collider_part_bounds(registry, description.collidable.shape, description.pose, body_part);
		if status != .Ok
		{
			state.status = status;
			return false;
		}
		for static_index: i32 = 0; static_index < static_count; static_index += 1
		{
			static_part: i32 = static_index if static_parts == .Present else -1;
			if static_parts == .Present && mixed_colliders_part_settings(state.storage, state.reference, static_part).role == .Trigger
			{
				continue;
			}
			old_bounds: util.Bounding_Box;
			old_bounds, status = mixed_collider_part_bounds(registry, state.old.shape, state.old.pose, static_part);
			if status != .Ok
			{
				state.status = status;
				return false;
			}
			if state.old.shape == state.next.shape
			{
				next_bounds: util.Bounding_Box;
				next_bounds, status = mixed_collider_part_bounds(registry, state.next.shape, state.next.pose, static_part);
				if status != .Ok
				{
					state.status = status;
					return false;
				}
				old_bounds = util.bounding_box_merge(old_bounds, next_bounds);
			}
			if util.bounding_box_intersection_state(body_bounds, old_bounds) == .Intersecting
			{
				return state.filter == nil || state.filter(state.user, body);
			}
		}
		if state.old.shape != state.next.shape
		{
			// replacement geometry has no inherited overrides and therefore is solid
			next_bounds: util.Bounding_Box;
			next_bounds, status = shape_registry_compute_world_bounds(registry, state.next.shape, state.next.pose);
			if status != .Ok
			{
				state.status = status;
				return false;
			}
			if util.bounding_box_intersection_state(body_bounds, next_bounds) == .Intersecting
			{
				return state.filter == nil || state.filter(state.user, body);
			}
		}
	}
	return false;
}

mixed_colliders_awaken_static :: proc(
	simulation:^Simulation, handle:Static_Handle, old, next:^Static_Description,
	bounds:util.Bounding_Box, filter:Static_Awakening_Filter_Proc, user:rawptr,
) -> Physics_Status
{
	reference: Collidable_Reference;
	reference, _ = collidable_reference_static(handle);
	if simulation.triggers==nil || simulation.triggers.mixed==nil
	{
		return simulation_awaken_inactive_bodies_in_bounds_filtered(simulation, bounds, filter, user);
	}
	state:Mixed_Static_Awakening={storage=simulation.triggers.mixed, reference=reference, old=old^, next=next^, filter=filter, user=user};
	status:Physics_Status=simulation_awaken_inactive_bodies_in_bounds_filtered(simulation, bounds, mixed_static_awaken_filter, &state);
	if status!=.Ok
	{
		return status;
	}
	return state.status;
}

// the normal collision batcher intentionally treats an absent child task as
// empty. selected mixed consumers instead report the missing registration
mixed_colliders_child_route :: proc "contextless" (simulation:^Simulation, pair:Broad_Phase_Pair, a, b:i32) -> Physics_Status
{
	target_a: Shape_Query_Target;
	sa: Physics_Status;
	target_a, sa = simulation_query_target(simulation, pair.a);
	if sa!=.Ok
	{
		return sa;
	}
	target_b: Shape_Query_Target;
	sb: Physics_Status;
	target_b, sb = simulation_query_target(simulation, pair.b);
	if sb!=.Ok
	{
		return sb;
	}
	shapes: ^Shape_Registry = simulation_shape_registry(simulation);
	raw_a: rawptr;
	ra: Physics_Status;
	raw_a, _, ra = shape_registry_resolve(shapes, target_a.shape);
	if ra!=.Ok
	{
		return ra;
	}
	raw_b: rawptr;
	rb: Physics_Status;
	raw_b, _, rb = shape_registry_resolve(shapes, target_b.shape);
	if rb!=.Ok
	{
		return rb;
	}
	type_a: int = query_overlap_any_child_type(raw_a, int(typed_index_type(target_a.shape)), int(a));
	type_b: int = query_overlap_any_child_type(raw_b, int(typed_index_type(target_b.shape)), int(b));
	status: Physics_Status;
	_, _, status = collision_task_registry_lookup(&simulation.collision_tasks, type_a, type_b);
	return status;
}
