package entasis

import "base:intrinsics"
import "base:runtime"
import "core:mem"
import physics "entasis:entasis_physics"

// Contact_Event_Kind describes the pair transition observed after one completed
// world step. pair identity, rather than individual manifold feature IDs, defines
// the lifetime
Contact_Event_Kind :: enum u8
{
	Begin,
	Persist,
	End,
}

// Contact_Event_Flag describes which optional event fields are available
Contact_Event_Flag :: enum u8
{
	Normal_Available,
	User_A_Available,
	User_B_Available,
	Solver_Data_Available,
}

// Contact_Event_Flags is a compact availability bitset for optional event fields
Contact_Event_Flags :: distinct bit_set[Contact_Event_Flag; u8];

// Contact_Event is one owner-thread pair transition. within each event, a and b
// preserve the canonical low-level pair orientation. this does not define array
// order. drain emits events for ended lifetimes first, then current events in
// pair-cache discovery order. End events retain the last known normal and caller IDs
Contact_Event :: struct
{
	kind:          Contact_Event_Kind,
	contact_count: u8,
	flags:         Contact_Event_Flags,
	a:             Collidable_Reference,
	b:             Collidable_Reference,
	normal:        Vector3,
	user_a:        u64,
	user_b:        u64,
	constraint:    Constraint_Handle,
	contact_data:  Solver_Contact_Data,
}

// Contact_User_Table is the generation-aware property table used
// to translate collidable references into caller-selected stable IDs
Contact_User_Table :: Collidable_Property_Table(u64);

// contact_user_table_init initializes caller-owned body and static ID spaces
contact_user_table_init :: #force_inline proc (
	table: ^Contact_User_Table,
	body_capacity: int = 0,
	static_capacity: int = 0,
	allocator: Allocator = {},
) -> Status
{
	return collidable_property_init(table, body_capacity, static_capacity, allocator);
}

// contact_user_table_ensure_capacity grows body and static user-ID capacities
contact_user_table_ensure_capacity :: #force_inline proc (
	table: ^Contact_User_Table,
	body_capacity, static_capacity: int,
) -> Status
{
	return collidable_property_ensure_capacity(table, body_capacity, static_capacity);
}

// contact_user_set_body sets one generation-aware body user ID
contact_user_set_body :: #force_inline proc (
	table: ^Contact_User_Table,
	handle: Body_Handle,
	user_id: u64,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return body_property_set(&table.bodies, handle, user_id);
}

// contact_user_set_static sets one generation-aware static user ID
contact_user_set_static :: #force_inline proc (
	table: ^Contact_User_Table,
	handle: Static_Handle,
	user_id: u64,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return static_property_set(&table.statics, handle, user_id);
}

// contact_user_remove_body ends one body user-ID property lifetime
contact_user_remove_body :: #force_inline proc (
	table: ^Contact_User_Table,
	handle: Body_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return body_property_remove(&table.bodies, handle);
}

// contact_user_remove_static ends one static user-ID property lifetime
contact_user_remove_static :: #force_inline proc (
	table: ^Contact_User_Table,
	handle: Static_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return static_property_remove(&table.statics, handle);
}

// contact_user_table_clear clears user-ID properties while retaining capacity
contact_user_table_clear :: #force_inline proc (table: ^Contact_User_Table) -> Status
{
	return collidable_property_clear(table);
}

// contact_user_table_destroy releases user-ID property storage
contact_user_table_destroy :: #force_inline proc (table: ^Contact_User_Table) -> Status
{
	return collidable_property_destroy(table);
}

@(private)
contact_tracker_lifecycle :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

@(private)
contact_user_identity :: struct
{
	value:      u64,
	generation: u64,
	epoch:      u64,
	present:    bool,
}

@(private)
contact_pair_state :: struct
{
	pair:           physics.Collidable_Pair,
	normal:         Vector3,
	user_a:         contact_user_identity,
	user_b:         contact_user_identity,
	contact_count:       u8,
	normal_present:      bool,
	solver_data_present: bool,
	solver_data:         Solver_Contact_Data,
}

@(private)
contact_pair_table :: struct
{
	entries:       [^]contact_pair_state,
	slots:         [^]i32,
	count:         int,
	capacity:      int,
	slot_capacity: int,
	slot_mask:     int,
}

// Contact_Tracker is caller-owned event state. keep a ready tracker at a stable
// address, bind it to one ready World, then drain or discard exactly once after
// every successful world_step. unbind or destroy it before destroying or
// reinitializing the bound world
Contact_Tracker :: struct
{
	tables:               [2]contact_pair_table,
	allocator:            Allocator,
	simulation:           ^physics.Simulation,
	users:                ^Contact_User_Table,
	last_step_index:      u64,
	previous_table_index: u8,
	lifecycle:            contact_tracker_lifecycle,
}

@(private)
contact_tracker_allocator :: #force_inline proc (allocator: Allocator) -> Allocator
{
	if allocator.procedure == nil
	{
		return runtime.heap_allocator();
	}
	return allocator;
}

@(private)
contact_pair_slot_capacity :: proc "contextless" (pair_capacity: int) -> (int, Status)
{
	if pair_capacity < 0
	{
		return 0, .Invalid_Argument;
	}
	if pair_capacity == 0
	{
		return 0, .Ok;
	}
	if pair_capacity > max(int) / 2
	{
		return 0, .Capacity_Missing;
	}
	required := max(pair_capacity * 2, 16);
	capacity := 16;
	for capacity < required
	{
		if capacity > max(int) / 2
		{
			return 0, .Capacity_Missing;
		}
		capacity *= 2;
	}
	return capacity, .Ok;
}

@(private)
contact_pair_table_allocate :: proc (
	table: ^contact_pair_table,
	pair_capacity: int,
	allocator: Allocator,
) -> Status
{
	if table == nil || pair_capacity < 0 || table.entries != nil || table.slots != nil
	{
		return .Invalid_Argument;
	}
	slot_capacity, slot_status := contact_pair_slot_capacity(pair_capacity);
	if slot_status != .Ok
	{
		return slot_status;
	}
	if pair_capacity == 0
	{
		table^ = {};
		return .Ok;
	}
	if pair_capacity > max(int) / size_of(contact_pair_state) ||
		slot_capacity > max(int) / size_of(i32)
	{
		return .Capacity_Missing;
	}
	entry_bytes := pair_capacity * size_of(contact_pair_state);
	entry_memory, entry_error := mem.alloc(
		entry_bytes, align_of(contact_pair_state), allocator,
	);
	if entry_error != nil || entry_memory == nil
	{
		return .Capacity_Missing;
	}
	slot_bytes := slot_capacity * size_of(i32);
	slot_memory, slot_error := mem.alloc(slot_bytes, align_of(i32), allocator);
	if slot_error != nil || slot_memory == nil
	{
		_ = mem.free(entry_memory, allocator);
		return .Capacity_Missing;
	}
	intrinsics.mem_zero(entry_memory, entry_bytes);
	intrinsics.mem_zero(slot_memory, slot_bytes);
	table^ = {
		entries=([^]contact_pair_state)(entry_memory),
		slots=([^]i32)(slot_memory),
		capacity=pair_capacity,
		slot_capacity=slot_capacity,
		slot_mask=slot_capacity - 1,
	};
	return .Ok;
}

@(private)
contact_pair_table_release :: proc (
	table: ^contact_pair_table,
	allocator: Allocator,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	result := Status.Ok;
	if table.entries != nil && mem.free(table.entries, allocator) != .None
	{
		result = .Invalid_Argument;
	}
	if table.slots != nil && mem.free(table.slots, allocator) != .None && result == .Ok
	{
		result = .Invalid_Argument;
	}
	table^ = {};
	return result;
}

@(private)
contact_pair_table_clear :: #force_inline proc "contextless" (table: ^contact_pair_table)
{
	if table == nil
	{
		return;
	}
	if table.slots != nil && table.slot_capacity > 0
	{
		intrinsics.mem_zero(table.slots, table.slot_capacity * size_of(i32));
	}
	table.count = 0;
}

@(private)
contact_pair_hash_index :: #force_inline proc "contextless" (
	pair: physics.Collidable_Pair,
	mask: int,
) -> int
{
	pair_copy := pair;
	return int(u32(physics.collidable_pair_hash(&pair_copy))) & mask;
}

@(private)
contact_pair_table_index_of :: #force_inline proc "contextless" (
	table: ^contact_pair_table,
	pair: physics.Collidable_Pair,
) -> int
{
	if table == nil || table.count <= 0 || table.slots == nil
	{
		return -1;
	}
	slot := contact_pair_hash_index(pair, table.slot_mask);
	for
	{
		encoded := int(table.slots[slot]);
		if encoded <= 0
		{
			return -1;
		}
		index := encoded - 1;
		stored := table.entries[index].pair;
		if stored.a.packed == pair.a.packed && stored.b.packed == pair.b.packed
		{
			return index;
		}
		slot = (slot + 1) & table.slot_mask;
	}
}

@(private)
contact_pair_table_set :: proc "contextless" (
	table: ^contact_pair_table,
	value: contact_pair_state,
) -> Status
{
	if table == nil || table.capacity <= 0 || table.entries == nil || table.slots == nil
	{
		return .Capacity_Missing;
	}
	slot := contact_pair_hash_index(value.pair, table.slot_mask);
	for
	{
		encoded := int(table.slots[slot]);
		if encoded <= 0
		{
			if table.count >= table.capacity
			{
				return .Capacity_Missing;
			}
			index := table.count;
			table.count += 1;
			table.entries[index] = value;
			table.slots[slot] = i32(index + 1);
			return .Ok;
		}
		index := encoded - 1;
		stored := table.entries[index].pair;
		if stored.a.packed == value.pair.a.packed && stored.b.packed == value.pair.b.packed
		{
			table.entries[index] = value;
			return .Ok;
		}
		slot = (slot + 1) & table.slot_mask;
	}
}

@(private)
contact_tracker_pair_capacity_growth :: proc "contextless" (
	current_capacity, required_capacity: int,
) -> (int, Status)
{
	if current_capacity < 0 || required_capacity < 0
	{
		return 0, .Invalid_Argument;
	}
	if required_capacity <= current_capacity
	{
		return current_capacity, .Ok;
	}
	target := max(current_capacity, 16);
	for target < required_capacity
	{
		if target > max(int) / 2
		{
			target = required_capacity;
			break;
		}
		target *= 2;
	}
	return target, .Ok;
}

// contact_tracker_init initializes caller-owned pair history. initial_capacity is
// the maximum simultaneously retained pair count before explicit growth
contact_tracker_init :: proc (
	tracker: ^Contact_Tracker,
	initial_capacity: int = 64,
	allocator: Allocator = {},
) -> Status
{
	if tracker == nil || initial_capacity < 0 || tracker.lifecycle == .Ready
	{
		return .Invalid_Argument;
	}
	actual_allocator := contact_tracker_allocator(allocator);
	first_status := contact_pair_table_allocate(
		&tracker.tables[0], initial_capacity, actual_allocator,
	);
	if first_status != .Ok
	{
		return first_status;
	}
	second_status := contact_pair_table_allocate(
		&tracker.tables[1], initial_capacity, actual_allocator,
	);
	if second_status != .Ok
	{
		_ = contact_pair_table_release(&tracker.tables[0], actual_allocator);
		return second_status;
	}
	tracker.allocator = actual_allocator;
	tracker.lifecycle = .Ready;
	return .Ok;
}

// contact_tracker_capacity returns the current pair-history capacity
contact_tracker_capacity :: #force_inline proc "contextless" (
	tracker: ^Contact_Tracker,
) -> int
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return 0;
	}
	return tracker.tables[0].capacity;
}

// contact_tracker_ensure_capacity grows both pair-history buffers geometrically.
// existing pair lifetimes and the current binding are preserved
contact_tracker_ensure_capacity :: proc (
	tracker: ^Contact_Tracker,
	required_capacity: int,
) -> Status
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return .Disposed;
	}
	if required_capacity < 0
	{
		return .Invalid_Argument;
	}
	current_capacity := contact_tracker_capacity(tracker);
	if required_capacity <= current_capacity
	{
		return .Ok;
	}
	target, target_status := contact_tracker_pair_capacity_growth(
		current_capacity, required_capacity,
	);
	if target_status != .Ok
	{
		return target_status;
	}
	new_tables: [2]contact_pair_table;
	status := contact_pair_table_allocate(&new_tables[0], target, tracker.allocator);
	if status != .Ok
	{
		return status;
	}
	status = contact_pair_table_allocate(&new_tables[1], target, tracker.allocator);
	if status != .Ok
	{
		_ = contact_pair_table_release(&new_tables[0], tracker.allocator);
		return status;
	}
	previous := &tracker.tables[int(tracker.previous_table_index)];
	for index in 0 ..< previous.count
	{
		status = contact_pair_table_set(&new_tables[0], previous.entries[index]);
		if status != .Ok
		{
			_ = contact_pair_table_release(&new_tables[1], tracker.allocator);
			_ = contact_pair_table_release(&new_tables[0], tracker.allocator);
			return status;
		}
	}
	_ = contact_pair_table_release(&tracker.tables[0], tracker.allocator);
	_ = contact_pair_table_release(&tracker.tables[1], tracker.allocator);
	tracker.tables = new_tables;
	tracker.previous_table_index = 0;
	return .Ok;
}

// contact_tracker_bind attaches a ready tracker to one ready world and optional
// caller-owned user-ID table. binding begins a new event history. the first
// successfully drained step reports Begin for every retained pair
contact_tracker_bind :: proc "contextless" (
	tracker: ^Contact_Tracker,
	world: ^World,
	users: ^Contact_User_Table = nil,
) -> Status
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return .Disposed;
	}
	if tracker.simulation != nil
	{
		return .Invalid_Argument;
	}
	data := world_data_get(world);
	if data == nil || data.simulation.state != .Ready
	{
		return .Disposed;
	}
	if users != nil &&
		(users.bodies.storage.state != .Ready || users.statics.storage.state != .Ready)
	{
		return .Disposed;
	}
	contact_pair_table_clear(&tracker.tables[0]);
	contact_pair_table_clear(&tracker.tables[1]);
	tracker.simulation = &data.simulation;
	tracker.users = users;
	tracker.last_step_index = data.simulation.step_index;
	tracker.previous_table_index = 0;
	return .Ok;
}

// contact_tracker_clear discards all pair history while retaining allocation and
// binding. the next completed step begins a new event stream
contact_tracker_clear :: proc "contextless" (tracker: ^Contact_Tracker) -> Status
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return .Disposed;
	}
	contact_pair_table_clear(&tracker.tables[0]);
	contact_pair_table_clear(&tracker.tables[1]);
	tracker.previous_table_index = 0;
	if tracker.simulation != nil
	{
		if tracker.simulation.state != .Ready
		{
			return .Disposed;
		}
		tracker.last_step_index = tracker.simulation.step_index;
	}
	return .Ok;
}

// contact_tracker_unbind discards history and detaches the tracker from its
// world. the tracker may then bind to another world
contact_tracker_unbind :: proc "contextless" (tracker: ^Contact_Tracker) -> Status
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return .Disposed;
	}
	contact_pair_table_clear(&tracker.tables[0]);
	contact_pair_table_clear(&tracker.tables[1]);
	tracker.simulation = nil;
	tracker.users = nil;
	tracker.last_step_index = 0;
	tracker.previous_table_index = 0;
	return .Ok;
}

// contact_tracker_destroy releases pair-history storage. destroy or unbind the
// tracker before destroying the bound world
contact_tracker_destroy :: proc (tracker: ^Contact_Tracker) -> Status
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return .Disposed;
	}
	result := contact_pair_table_release(&tracker.tables[0], tracker.allocator);
	second := contact_pair_table_release(&tracker.tables[1], tracker.allocator);
	if result == .Ok && second != .Ok
	{
		result = second;
	}
	tracker.simulation = nil;
	tracker.users = nil;
	tracker.last_step_index = 0;
	tracker.previous_table_index = 0;
	tracker.lifecycle = .Disposed;
	return result;
}

@(private)
contact_tracker_user_identity :: #force_inline proc "contextless" (
	tracker: ^Contact_Tracker,
	collidable: Collidable_Reference,
) -> contact_user_identity
{
	if tracker.users == nil
	{
		return {};
	}
	key, key_status := collidable_property_key(tracker.users, collidable);
	if key_status != .Ok
	{
		return {};
	}
	value, value_status := collidable_property_get_key(tracker.users, key);
	if value_status != .Ok
	{
		return {};
	}
	return {
		value=value^,
		generation=key.generation,
		epoch=key.epoch,
		present=true,
	};
}

@(private)
contact_user_identity_equal :: #force_inline proc "contextless" (
	a, b: contact_user_identity,
) -> bool
{
	if a.present != b.present
	{
		return false;
	}
	if !a.present
	{
		return true;
	}
	return a.generation == b.generation && a.epoch == b.epoch;
}

@(private)
contact_pair_lifetime_equal :: #force_inline proc "contextless" (
	a, b: contact_pair_state,
) -> bool
{
	return contact_user_identity_equal(a.user_a, b.user_a) &&
		contact_user_identity_equal(a.user_b, b.user_b);
}

@(private)
contact_description_copy :: #force_inline proc "contextless" (
	view: ^physics.Contact_Constraint_Data_View,
	target: ^$T,
) -> bool
{
	if view == nil || target == nil || int(view.prestep_size) < size_of(T)
	{
		return false;
	}
	intrinsics.mem_copy(target, &view.prestep[0], size_of(T));
	return true;
}

@(private)
contact_summary_from_view :: proc "contextless" (
	view: ^physics.Contact_Constraint_Data_View,
) -> (Vector3, u8)
{
	if view == nil || view.contact_count <= 0
	{
		return {}, 0;
	}
	count := u8(min(int(view.contact_count), physics.MAXIMUM_MANIFOLD_CONTACT_COUNT));
	switch view.type_id
	{
		case physics.CONTACT_1_ONE_BODY_TYPE_ID:
			value: physics.Contact_1_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_2_ONE_BODY_TYPE_ID:
			value: physics.Contact_2_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_3_ONE_BODY_TYPE_ID:
			value: physics.Contact_3_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_4_ONE_BODY_TYPE_ID:
			value: physics.Contact_4_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_1_TYPE_ID:
			value: physics.Contact_1;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_2_TYPE_ID:
			value: physics.Contact_2;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_3_TYPE_ID:
			value: physics.Contact_3;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_4_TYPE_ID:
			value: physics.Contact_4;
			if contact_description_copy(view, &value)
			{
				return value.normal, count;
			}
		case physics.CONTACT_2_NONCONVEX_ONE_BODY_TYPE_ID:
			value: physics.Contact_2_Nonconvex_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
		case physics.CONTACT_3_NONCONVEX_ONE_BODY_TYPE_ID:
			value: physics.Contact_3_Nonconvex_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
		case physics.CONTACT_4_NONCONVEX_ONE_BODY_TYPE_ID:
			value: physics.Contact_4_Nonconvex_One_Body;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
		case physics.CONTACT_2_NONCONVEX_TYPE_ID:
			value: physics.Contact_2_Nonconvex;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
		case physics.CONTACT_3_NONCONVEX_TYPE_ID:
			value: physics.Contact_3_Nonconvex;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
		case physics.CONTACT_4_NONCONVEX_TYPE_ID:
			value: physics.Contact_4_Nonconvex;
			if contact_description_copy(view, &value)
			{
				return value.contact_0.normal, count;
			}
	}
	return {}, count;
}

@(private)
contact_summary_capture :: struct
{
	normal:        Vector3,
	contact_count: u8,
}

@(private)
contact_summary_visitor :: proc "contextless" (
	user_context: rawptr,
	view: ^physics.Contact_Constraint_Data_View,
)
{
	if user_context == nil
	{
		return;
	}
	capture := (^contact_summary_capture)(user_context);
	capture.normal, capture.contact_count = contact_summary_from_view(view);
}

@(private)
contact_pair_state_from_cache :: proc "contextless" (
	tracker: ^Contact_Tracker,
	pair: physics.Collidable_Pair,
	handle: Constraint_Handle,
) -> (contact_pair_state, Status)
{
	result := contact_pair_state{
		pair=pair,
		user_a=contact_tracker_user_identity(tracker, pair.a),
		user_b=contact_tracker_user_identity(tracker, pair.b),
	};
	data, status := solver_contact_data_from_simulation(tracker.simulation, handle);
	if status == .Ok
	{
		result.solver_data = data;
		result.solver_data_present = true;
		result.contact_count = data.contact_count;
		if data.contact_count > 0
		{
			result.normal = data.normal;
			if data.kind == .Nonconvex
			{
				result.normal = data.contacts[0].normal;
			}
			result.normal_present = true;
		}
		return result, .Ok;
	}
	// a custom selected contact constraint may not expose a built-in accessor.
	// pair lifetime remains valid. detailed fields remain zero
	if status == .Not_Found
	{
		return result, .Ok;
	}
	return {}, status;
}

@(private)
contact_tracker_build_current :: proc "contextless" (
	tracker: ^Contact_Tracker,
	current: ^contact_pair_table,
) -> Status
{
	contact_pair_table_clear(current);
	cache := &tracker.simulation.narrow_phase.pair_cache;
	required := cache.mapping.count + cache.inactive_count;
	if required > current.capacity
	{
		return .Capacity_Missing;
	}
	for index in 0 ..< cache.mapping.count
	{
		state, state_status := contact_pair_state_from_cache(
			tracker,
			cache.mapping.keys.memory[index],
			cache.mapping.values.memory[index].constraint_handle,
		);
		if state_status != .Ok
		{
			return state_status;
		}
		set_status := contact_pair_table_set(current, state);
		if set_status != .Ok
		{
			return set_status;
		}
	}
	for index in 0 ..< cache.inactive_count
	{
		entry := cache.inactive_entries.memory[index];
		state, state_status := contact_pair_state_from_cache(
			tracker, entry.pair, entry.cache.constraint_handle,
		);
		if state_status != .Ok
		{
			return state_status;
		}
		set_status := contact_pair_table_set(current, state);
		if set_status != .Ok
		{
			return set_status;
		}
	}
	return .Ok;
}

@(private)
contact_event_from_state :: #force_inline proc "contextless" (
	kind: Contact_Event_Kind,
	state: contact_pair_state,
) -> Contact_Event
{
	flags: Contact_Event_Flags;
	if state.normal_present
	{
		flags += {.Normal_Available};
	}
	if state.user_a.present
	{
		flags += {.User_A_Available};
	}
	if state.user_b.present
	{
		flags += {.User_B_Available};
	}
	if state.solver_data_present
	{
		flags += {.Solver_Data_Available};
	}
	contact_count := state.contact_count;
	if kind == .End
	{
		contact_count = 0;
	}
	return {
		kind=kind,
		contact_count=contact_count,
		flags=flags,
		a=state.pair.a,
		b=state.pair.b,
		normal=state.normal,
		user_a=state.user_a.value,
		user_b=state.user_b.value,
		constraint=state.solver_data.constraint,
		contact_data=state.solver_data,
	};
}

// contact_events_drain snapshots the pair cache after exactly one successful
// world_step and writes Begin, Persist, and End events into caller-owned storage.
// it performs no global ordering pass. on Capacity_Missing no history is
// committed and the call may be retried with a larger output slice or after
// contact_tracker_ensure_capacity
contact_events_drain :: proc "contextless" (
	tracker: ^Contact_Tracker,
	events: []Contact_Event,
) -> (written, required: int, status: Status)
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return 0, 0, .Disposed;
	}
	if tracker.simulation == nil || tracker.simulation.state != .Ready
	{
		return 0, 0, .Disposed;
	}
	expected_step := tracker.last_step_index + 1;
	if tracker.simulation.step_index != expected_step
	{
		return 0, 0, .Invalid_Argument;
	}
	previous_index := int(tracker.previous_table_index);
	current_index := 1 - previous_index;
	previous := &tracker.tables[previous_index];
	current := &tracker.tables[current_index];
	build_status := contact_tracker_build_current(tracker, current);
	if build_status != .Ok
	{
		pair_count := tracker.simulation.narrow_phase.pair_cache.mapping.count +
			tracker.simulation.narrow_phase.pair_cache.inactive_count;
		return 0, pair_count, build_status;
	}
	end_count := 0;
	for index in 0 ..< previous.count
	{
		old_state := previous.entries[index];
		current_lookup := contact_pair_table_index_of(current, old_state.pair);
		if current_lookup < 0 ||
			!contact_pair_lifetime_equal(old_state, current.entries[current_lookup])
		{
			end_count += 1;
		}
	}
	required = current.count + end_count;
	if len(events) < required
	{
		return 0, required, .Capacity_Missing;
	}
	written = 0;
	// end old lifetimes before beginning replacement lifetimes. preserve this
	// two-phase discovery order without a global ordering pass
	for index in 0 ..< previous.count
	{
		old_state := previous.entries[index];
		current_lookup := contact_pair_table_index_of(current, old_state.pair);
		if current_lookup >= 0 &&
			contact_pair_lifetime_equal(old_state, current.entries[current_lookup])
		{
			continue;
		}
		events[written] = contact_event_from_state(.End, old_state);
		written += 1;
	}
	for index in 0 ..< current.count
	{
		state := current.entries[index];
		previous_lookup := contact_pair_table_index_of(previous, state.pair);
		kind := Contact_Event_Kind.Begin;
		if previous_lookup >= 0 &&
			contact_pair_lifetime_equal(previous.entries[previous_lookup], state)
		{
			kind = .Persist;
		}
		events[written] = contact_event_from_state(kind, state);
		written += 1;
	}
	tracker.previous_table_index = u8(current_index);
	tracker.last_step_index = tracker.simulation.step_index;
	return written, required, .Ok;
}

// contact_events_discard advances pair history after exactly one completed step
// without producing events. it is useful when a frame intentionally ignores
// contact notifications. on Capacity_Missing history is not committed
contact_events_discard :: proc "contextless" (
	tracker: ^Contact_Tracker,
) -> (required: int, status: Status)
{
	if tracker == nil || tracker.lifecycle != .Ready
	{
		return 0, .Disposed;
	}
	if tracker.simulation == nil || tracker.simulation.state != .Ready
	{
		return 0, .Disposed;
	}
	if tracker.simulation.step_index != tracker.last_step_index + 1
	{
		return 0, .Invalid_Argument;
	}
	current_index := 1 - int(tracker.previous_table_index);
	current := &tracker.tables[current_index];
	status = contact_tracker_build_current(tracker, current);
	if status != .Ok
	{
		required = tracker.simulation.narrow_phase.pair_cache.mapping.count +
			tracker.simulation.narrow_phase.pair_cache.inactive_count;
		return required, status;
	}
	tracker.previous_table_index = u8(current_index);
	tracker.last_step_index = tracker.simulation.step_index;
	return current.count, .Ok;
}
