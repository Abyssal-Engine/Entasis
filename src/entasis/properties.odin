package entasis

import "base:intrinsics"
import "base:runtime"
import "core:mem"

// Body_Property_Key identifies one body-property lifetime. a key becomes invalid
// when its entry is removed, the table is cleared or destroyed, or the table is
// reinitialized. ready tables must remain at a stable address and must not be copied
Body_Property_Key :: struct
{
	handle:     Body_Handle,
	generation: u64,
	epoch:      u64,
	owner:      rawptr,
}

// Static_Property_Key identifies one static-property lifetime and rejects stale
// references after numeric static-handle reuse
Static_Property_Key :: struct
{
	handle:     Static_Handle,
	generation: u64,
	epoch:      u64,
	owner:      rawptr,
}

// Collidable_Property_Key identifies one body or static property lifetime while
// preserving the separate body and static handle namespaces
Collidable_Property_Key :: struct
{
	collidable: Collidable_Reference,
	generation: u64,
	epoch:      u64,
	owner:      rawptr,
}

@(private)
property_table_state :: enum u8
{
	Uninitialized,
	Ready,
	Disposed,
}

@(private)
property_storage :: struct($T: typeid)
{
	values:      [^]T,
	generations: [^]u64,
	capacity:    int,
	allocator:   Allocator,
	epoch:       u64,
	state:       property_table_state,
}

// Body_Property_Table stores caller-owned dense values indexed by Body_Handle.
// values and generation metadata use separate contiguous arrays. the table grows
// geometrically through its selected allocator and performs no allocation on get
Body_Property_Table :: struct($T: typeid)
{
	storage: property_storage(T),
}

// Static_Property_Table stores caller-owned dense values indexed by Static_Handle
Static_Property_Table :: struct($T: typeid)
{
	storage: property_storage(T),
}

// Collidable_Property_Table combines independent body and static tables so equal
// numeric body and static handles never alias the same property slot
Collidable_Property_Table :: struct($T: typeid)
{
	bodies:  Body_Property_Table(T),
	statics: Static_Property_Table(T),
}

@(private)
property_allocator :: #force_inline proc (allocator: Allocator) -> Allocator
{
	if allocator.procedure == nil
	{
		return runtime.heap_allocator();
	}
	return allocator;
}

@(private)
property_epoch_next :: #force_inline proc "contextless" (epoch: u64) -> u64
{
	next := epoch + 1;
	if next == 0
	{
		next = 1;
	}
	return next;
}

@(private)
property_generation_present :: #force_inline proc "contextless" (generation: u64) -> bool
{
	return generation & 1 == 1;
}

@(private)
property_generation_activate :: #force_inline proc "contextless" (generation: u64) -> u64
{
	if property_generation_present(generation)
	{
		return generation;
	}
	return generation + 1;
}

@(private)
property_storage_memory_size :: proc "contextless" ($T: typeid, capacity: int) -> (int, Status)
{
	if capacity < 0
	{
		return 0, .Invalid_Argument;
	}
	if capacity == 0
	{
		return 0, .Ok;
	}
	value_size := size_of(T);
	if value_size <= 0 || capacity > max(int) / value_size
	{
		return 0, .Capacity_Missing;
	}
	return capacity * value_size, .Ok;
}

@(private)
property_storage_allocate_values :: proc (
	$T: typeid, capacity: int, allocator: Allocator,
) -> ([^]T, Status)
{
	byte_count, size_status := property_storage_memory_size(T, capacity);
	if size_status != .Ok
	{
		return nil, size_status;
	}
	if byte_count == 0
	{
		return nil, .Ok;
	}
	memory, allocation_error := mem.alloc(byte_count, align_of(T), allocator);
	if allocation_error != nil || memory == nil
	{
		return nil, .Capacity_Missing;
	}
	intrinsics.mem_zero(memory, byte_count);
	return ([^]T)(memory), .Ok;
}

@(private)
property_storage_allocate_generations :: proc (
	capacity: int, allocator: Allocator,
) -> ([^]u64, Status)
{
	if capacity < 0 || capacity > max(int) / size_of(u64)
	{
		return nil, .Capacity_Missing;
	}
	if capacity == 0
	{
		return nil, .Ok;
	}
	byte_count := capacity * size_of(u64);
	memory, allocation_error := mem.alloc(byte_count, align_of(u64), allocator);
	if allocation_error != nil || memory == nil
	{
		return nil, .Capacity_Missing;
	}
	intrinsics.mem_zero(memory, byte_count);
	return ([^]u64)(memory), .Ok;
}

@(private)
property_storage_release :: proc (storage: ^property_storage($T)) -> Status
{
	if storage == nil
	{
		return .Invalid_Argument;
	}
	result := Status.Ok;
	if storage.values != nil
	{
		if mem.free(storage.values, storage.allocator) != .None
		{
			result = .Invalid_Argument;
		}
	}
	if storage.generations != nil
	{
		if mem.free(storage.generations, storage.allocator) != .None && result == .Ok
		{
			result = .Invalid_Argument;
		}
	}
	storage.values = nil;
	storage.generations = nil;
	storage.capacity = 0;
	return result;
}

@(private)
property_storage_init :: proc (
	storage: ^property_storage($T), initial_capacity: int, allocator: Allocator,
) -> Status
{
	if storage == nil || initial_capacity < 0 || storage.state == .Ready
	{
		return .Invalid_Argument;
	}
	actual_allocator := property_allocator(allocator);
	values, values_status := property_storage_allocate_values(T, initial_capacity, actual_allocator);
	if values_status != .Ok
	{
		return values_status;
	}
	generations, generations_status := property_storage_allocate_generations(initial_capacity, actual_allocator);
	if generations_status != .Ok
	{
		if values != nil
		{
			_ = mem.free(values, actual_allocator);
		}
		return generations_status;
	}
	epoch := property_epoch_next(storage.epoch);
	storage^ = {
		values=values,
		generations=generations,
		capacity=initial_capacity,
		allocator=actual_allocator,
		epoch=epoch,
		state=.Ready,
	};
	return .Ok;
}

@(private)
property_storage_growth_capacity :: proc "contextless" (
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

@(private)
property_storage_ensure_capacity :: proc (
	storage: ^property_storage($T), required_capacity: int,
) -> Status
{
	if storage == nil || storage.state != .Ready
	{
		return .Disposed;
	}
	if required_capacity < 0
	{
		return .Invalid_Argument;
	}
	if required_capacity <= storage.capacity
	{
		return .Ok;
	}
	target, growth_status := property_storage_growth_capacity(storage.capacity, required_capacity);
	if growth_status != .Ok
	{
		return growth_status;
	}
	new_values, values_status := property_storage_allocate_values(T, target, storage.allocator);
	if values_status != .Ok
	{
		return values_status;
	}
	new_generations, generations_status := property_storage_allocate_generations(target, storage.allocator);
	if generations_status != .Ok
	{
		if new_values != nil
		{
			_ = mem.free(new_values, storage.allocator);
		}
		return generations_status;
	}
	if storage.capacity > 0
	{
		intrinsics.mem_copy(
			new_values,
			storage.values,
			storage.capacity * size_of(T),
		);
		intrinsics.mem_copy(
			new_generations,
			storage.generations,
			storage.capacity * size_of(u64),
		);
	}
	old_values := storage.values;
	old_generations := storage.generations;
	storage.values = new_values;
	storage.generations = new_generations;
	storage.capacity = target;
	if old_values != nil
	{
		_ = mem.free(old_values, storage.allocator);
	}
	if old_generations != nil
	{
		_ = mem.free(old_generations, storage.allocator);
	}
	return .Ok;
}

@(private)
property_storage_set :: proc (
	storage: ^property_storage($T), index: int, value: T,
) -> (u64, Status)
{
	if storage == nil || storage.state != .Ready
	{
		return 0, .Disposed;
	}
	if index < 0
	{
		return 0, .Invalid_Argument;
	}
	capacity_status := property_storage_ensure_capacity(storage, index + 1);
	if capacity_status != .Ok
	{
		return 0, capacity_status;
	}
	generation := property_generation_activate(storage.generations[index]);
	storage.generations[index] = generation;
	storage.values[index] = value;
	return generation, .Ok;
}

@(private)
property_storage_get :: #force_inline proc "contextless" (
	storage: ^property_storage($T), index: int,
) -> (^T, Status)
{
	if storage == nil || storage.state != .Ready
	{
		return nil, .Disposed;
	}
	if index < 0
	{
		return nil, .Invalid_Argument;
	}
	if index >= storage.capacity || !property_generation_present(storage.generations[index])
	{
		return nil, .Not_Found;
	}
	return &storage.values[index], .Ok;
}

@(private)
property_storage_get_generation :: #force_inline proc "contextless" (
	storage: ^property_storage($T), index: int, generation: u64, epoch: u64,
) -> (^T, Status)
{
	if storage == nil || storage.state != .Ready
	{
		return nil, .Disposed;
	}
	if index < 0
	{
		return nil, .Invalid_Argument;
	}
	if epoch != storage.epoch || index >= storage.capacity ||
		generation == 0 || storage.generations[index] != generation ||
		!property_generation_present(generation)
	{
		return nil, .Not_Found;
	}
	return &storage.values[index], .Ok;
}

@(private)
property_storage_key_data :: #force_inline proc "contextless" (
	storage: ^property_storage($T), index: int,
) -> (u64, u64, Status)
{
	if storage == nil || storage.state != .Ready
	{
		return 0, 0, .Disposed;
	}
	if index < 0
	{
		return 0, 0, .Invalid_Argument;
	}
	if index >= storage.capacity || !property_generation_present(storage.generations[index])
	{
		return 0, 0, .Not_Found;
	}
	return storage.generations[index], storage.epoch, .Ok;
}

@(private)
property_storage_remove :: proc (
	storage: ^property_storage($T), index: int,
) -> Status
{
	if storage == nil || storage.state != .Ready
	{
		return .Disposed;
	}
	if index < 0
	{
		return .Invalid_Argument;
	}
	if index >= storage.capacity || !property_generation_present(storage.generations[index])
	{
		return .Not_Found;
	}
	storage.values[index] = {};
	generation := storage.generations[index];
	if generation == max(u64)
	{
		storage.generations[index] = 0;
		storage.epoch = property_epoch_next(storage.epoch);
	}
	else
	{
		storage.generations[index] = generation + 1;
	}
	return .Ok;
}

@(private)
property_storage_clear :: proc (storage: ^property_storage($T)) -> Status
{
	if storage == nil || storage.state != .Ready
	{
		return .Disposed;
	}
	if storage.capacity > 0
	{
		intrinsics.mem_zero(storage.values, storage.capacity * size_of(T));
		intrinsics.mem_zero(storage.generations, storage.capacity * size_of(u64));
	}
	storage.epoch = property_epoch_next(storage.epoch);
	return .Ok;
}

@(private)
property_storage_destroy :: proc (storage: ^property_storage($T)) -> Status
{
	if storage == nil || storage.state != .Ready
	{
		return .Disposed;
	}
	epoch := property_epoch_next(storage.epoch);
	result := property_storage_release(storage);
	storage.epoch = epoch;
	storage.state = .Disposed;
	return result;
}

// body_property_init initializes a zero or previously destroyed table. capacity is
// an initial hint and may be zero. a nil allocator selects the runtime heap
// ownership: caller owned. keep the table at a stable address until destroy
body_property_init :: proc (
	table: ^Body_Property_Table($T),
	initial_capacity: int = 0,
	allocator: Allocator = {},
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_init(&table.storage, initial_capacity, allocator);
}

// body_property_ensure_capacity grows storage geometrically when required. existing
// values and lifetime generations are preserved
body_property_ensure_capacity :: proc (
	table: ^Body_Property_Table($T), capacity: int,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_ensure_capacity(&table.storage, capacity);
}

// body_property_capacity returns the current dense slot capacity, or zero when the
// table is not ready
body_property_capacity :: #force_inline proc "contextless" (
	table: ^Body_Property_Table($T),
) -> int
{
	if table == nil || table.storage.state != .Ready
	{
		return 0;
	}
	return table.storage.capacity;
}

// body_property_set creates or updates one value. creating a missing value begins a
// new generation. updating a present value preserves existing keys. allocation may
// occur only when the handle index exceeds current capacity
body_property_set :: proc (
	table: ^Body_Property_Table($T), handle: Body_Handle, value: T,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	_, status := property_storage_set(&table.storage, int(handle.value), value);
	return status;
}

// body_property_set_batch applies values in input order and returns the committed
// prefix before the first failure. input and output storage remain caller owned
body_property_set_batch :: proc (
	table: ^Body_Property_Table($T), handles: []Body_Handle, values: []T,
) -> (int, Status)
{
	if table == nil || len(handles) != len(values)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(handles)
	{
		status := body_property_set(table, handles[index], values[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(handles), .Ok;
}

// body_property_get returns the current value for a handle. allocation: none. the
// returned pointer is invalidated by growth, clear, destroy, or reinitialization
body_property_get :: #force_inline proc "contextless" (
	table: ^Body_Property_Table($T), handle: Body_Handle,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	return property_storage_get(&table.storage, int(handle.value));
}

// body_property_key captures a generation-protected key for the current entry
body_property_key :: #force_inline proc "contextless" (
	table: ^Body_Property_Table($T), handle: Body_Handle,
) -> (Body_Property_Key, Status)
{
	if table == nil
	{
		return {}, .Invalid_Argument;
	}
	generation, epoch, status := property_storage_key_data(&table.storage, int(handle.value));
	if status != .Ok
	{
		return {}, status;
	}
	return {
		handle=handle,
		generation=generation,
		epoch=epoch,
		owner=rawptr(table),
	}, .Ok;
}

// body_property_get_key resolves a key only while the same table and property
// lifetime remain active. allocation: none
body_property_get_key :: #force_inline proc "contextless" (
	table: ^Body_Property_Table($T), key: Body_Property_Key,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	if key.owner != rawptr(table)
	{
		return nil, .Not_Found;
	}
	return property_storage_get_generation(
		&table.storage,
		int(key.handle.value),
		key.generation,
		key.epoch,
	);
}

// body_property_remove clears one property and invalidates every key for its prior
// lifetime. call it after a successful body removal or for the committed prefix of
// body_remove_batch
body_property_remove :: proc (
	table: ^Body_Property_Table($T), handle: Body_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_remove(&table.storage, int(handle.value));
}

// body_property_remove_batch removes entries in input order and returns the removed
// prefix before the first missing or invalid handle
body_property_remove_batch :: proc (
	table: ^Body_Property_Table($T), handles: []Body_Handle,
) -> (int, Status)
{
	if table == nil
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(handles)
	{
		status := body_property_remove(table, handles[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(handles), .Ok;
}

// body_property_clear removes all entries, retains capacity, and invalidates all
// previously issued keys
body_property_clear :: proc (table: ^Body_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_clear(&table.storage);
}

// body_property_destroy releases owned storage and invalidates all keys. the table
// may be initialized again at the same address
body_property_destroy :: proc (table: ^Body_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_destroy(&table.storage);
}

// static_property_init initializes a dense caller-owned static property table
static_property_init :: proc (
	table: ^Static_Property_Table($T),
	initial_capacity: int = 0,
	allocator: Allocator = {},
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_init(&table.storage, initial_capacity, allocator);
}

// static_property_ensure_capacity grows static property storage geometrically
static_property_ensure_capacity :: proc (
	table: ^Static_Property_Table($T), capacity: int,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_ensure_capacity(&table.storage, capacity);
}

// static_property_capacity returns the current static slot capacity
static_property_capacity :: #force_inline proc "contextless" (
	table: ^Static_Property_Table($T),
) -> int
{
	if table == nil || table.storage.state != .Ready
	{
		return 0;
	}
	return table.storage.capacity;
}

// static_property_set creates or updates one static value. allocation may occur on
// growth. updating an existing value preserves its key generation
static_property_set :: proc (
	table: ^Static_Property_Table($T), handle: Static_Handle, value: T,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	_, status := property_storage_set(&table.storage, int(handle.value), value);
	return status;
}

// static_property_set_batch applies values in input order and returns the committed
// prefix before the first failure
static_property_set_batch :: proc (
	table: ^Static_Property_Table($T), handles: []Static_Handle, values: []T,
) -> (int, Status)
{
	if table == nil || len(handles) != len(values)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(handles)
	{
		status := static_property_set(table, handles[index], values[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(handles), .Ok;
}

// static_property_get returns one current value without allocation
static_property_get :: #force_inline proc "contextless" (
	table: ^Static_Property_Table($T), handle: Static_Handle,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	return property_storage_get(&table.storage, int(handle.value));
}

// static_property_key captures a generation-protected key for one entry
static_property_key :: #force_inline proc "contextless" (
	table: ^Static_Property_Table($T), handle: Static_Handle,
) -> (Static_Property_Key, Status)
{
	if table == nil
	{
		return {}, .Invalid_Argument;
	}
	generation, epoch, status := property_storage_key_data(&table.storage, int(handle.value));
	if status != .Ok
	{
		return {}, status;
	}
	return {
		handle=handle,
		generation=generation,
		epoch=epoch,
		owner=rawptr(table),
	}, .Ok;
}

// static_property_get_key rejects removed, cleared, destroyed, or reused lifetimes
static_property_get_key :: #force_inline proc "contextless" (
	table: ^Static_Property_Table($T), key: Static_Property_Key,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	if key.owner != rawptr(table)
	{
		return nil, .Not_Found;
	}
	return property_storage_get_generation(
		&table.storage,
		int(key.handle.value),
		key.generation,
		key.epoch,
	);
}

// static_property_remove clears one property and invalidates its prior keys. call
// it after the matching successful static removal
static_property_remove :: proc (
	table: ^Static_Property_Table($T), handle: Static_Handle,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_remove(&table.storage, int(handle.value));
}

// static_property_remove_batch removes entries in input order and returns the
// successful prefix
static_property_remove_batch :: proc (
	table: ^Static_Property_Table($T), handles: []Static_Handle,
) -> (int, Status)
{
	if table == nil
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(handles)
	{
		status := static_property_remove(table, handles[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(handles), .Ok;
}

// static_property_clear removes all entries while retaining capacity
static_property_clear :: proc (table: ^Static_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_clear(&table.storage);
}

// static_property_destroy releases storage and permits later reinitialization
static_property_destroy :: proc (table: ^Static_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	return property_storage_destroy(&table.storage);
}

// collidable_property_init initializes independent body and static property spaces
// using one allocator policy
collidable_property_init :: proc (
	table: ^Collidable_Property_Table($T),
	body_capacity: int = 0,
	static_capacity: int = 0,
	allocator: Allocator = {},
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	body_status := body_property_init(&table.bodies, body_capacity, allocator);
	if body_status != .Ok
	{
		return body_status;
	}
	static_status := static_property_init(&table.statics, static_capacity, allocator);
	if static_status != .Ok
	{
		_ = body_property_destroy(&table.bodies);
		return static_status;
	}
	return .Ok;
}

// collidable_property_ensure_capacity grows body and static spaces independently
collidable_property_ensure_capacity :: proc (
	table: ^Collidable_Property_Table($T), body_capacity, static_capacity: int,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	body_status := body_property_ensure_capacity(&table.bodies, body_capacity);
	if body_status != .Ok
	{
		return body_status;
	}
	return static_property_ensure_capacity(&table.statics, static_capacity);
}

@(private)
collidable_property_body_index :: #force_inline proc "contextless" (
	collidable: Collidable_Reference,
) -> (int, Status)
{
	mobility := collidable_mobility(collidable);
	if mobility != .Dynamic && mobility != .Kinematic
	{
		return -1, .Invalid_Argument;
	}
	handle, status := collidable_body_handle(collidable);
	if status != .Ok
	{
		return -1, status;
	}
	return int(handle.value), .Ok;
}

@(private)
collidable_property_static_index :: #force_inline proc "contextless" (
	collidable: Collidable_Reference,
) -> (int, Status)
{
	if collidable_mobility(collidable) != .Static
	{
		return -1, .Invalid_Argument;
	}
	handle, status := collidable_static_handle(collidable);
	if status != .Ok
	{
		return -1, status;
	}
	return int(handle.value), .Ok;
}

// collidable_property_set dispatches to the body or static namespace encoded in the
// collidable reference
collidable_property_set :: proc (
	table: ^Collidable_Property_Table($T), collidable: Collidable_Reference, value: T,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	if collidable_mobility(collidable) == .Static
	{
		index, status := collidable_property_static_index(collidable);
		if status != .Ok
		{
			return status;
		}
		_, status = property_storage_set(&table.statics.storage, index, value);
		return status;
	}
	index, status := collidable_property_body_index(collidable);
	if status != .Ok
	{
		return status;
	}
	_, status = property_storage_set(&table.bodies.storage, index, value);
	return status;
}

// collidable_property_get performs allocation-free dense lookup in the selected
// body or static namespace
collidable_property_get :: #force_inline proc "contextless" (
	table: ^Collidable_Property_Table($T), collidable: Collidable_Reference,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	if collidable_mobility(collidable) == .Static
	{
		index, status := collidable_property_static_index(collidable);
		if status != .Ok
		{
			return nil, status;
		}
		return property_storage_get(&table.statics.storage, index);
	}
	index, status := collidable_property_body_index(collidable);
	if status != .Ok
	{
		return nil, status;
	}
	return property_storage_get(&table.bodies.storage, index);
}

// collidable_property_key captures one generation-protected collidable property key
collidable_property_key :: #force_inline proc "contextless" (
	table: ^Collidable_Property_Table($T), collidable: Collidable_Reference,
) -> (Collidable_Property_Key, Status)
{
	if table == nil
	{
		return {}, .Invalid_Argument;
	}
	generation, epoch: u64;
	status: Status;
	if collidable_mobility(collidable) == .Static
	{
		index, index_status := collidable_property_static_index(collidable);
		if index_status != .Ok
		{
			return {}, index_status;
		}
		generation, epoch, status = property_storage_key_data(&table.statics.storage, index);
	}
	else
	{
		index, index_status := collidable_property_body_index(collidable);
		if index_status != .Ok
		{
			return {}, index_status;
		}
		generation, epoch, status = property_storage_key_data(&table.bodies.storage, index);
	}
	if status != .Ok
	{
		return {}, status;
	}
	return {
		collidable=collidable,
		generation=generation,
		epoch=epoch,
		owner=rawptr(table),
	}, .Ok;
}

// collidable_property_get_key validates table identity, epoch, and slot generation
collidable_property_get_key :: #force_inline proc "contextless" (
	table: ^Collidable_Property_Table($T), key: Collidable_Property_Key,
) -> (^T, Status)
{
	if table == nil
	{
		return nil, .Invalid_Argument;
	}
	if key.owner != rawptr(table)
	{
		return nil, .Not_Found;
	}
	if collidable_mobility(key.collidable) == .Static
	{
		index, status := collidable_property_static_index(key.collidable);
		if status != .Ok
		{
			return nil, status;
		}
		return property_storage_get_generation(
			&table.statics.storage, index, key.generation, key.epoch,
		);
	}
	index, status := collidable_property_body_index(key.collidable);
	if status != .Ok
	{
		return nil, status;
	}
	return property_storage_get_generation(
		&table.bodies.storage, index, key.generation, key.epoch,
	);
}

// collidable_property_remove clears the selected body or static property lifetime
collidable_property_remove :: proc (
	table: ^Collidable_Property_Table($T), collidable: Collidable_Reference,
) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	if collidable_mobility(collidable) == .Static
	{
		index, status := collidable_property_static_index(collidable);
		if status != .Ok
		{
			return status;
		}
		return property_storage_remove(&table.statics.storage, index);
	}
	index, status := collidable_property_body_index(collidable);
	if status != .Ok
	{
		return status;
	}
	return property_storage_remove(&table.bodies.storage, index);
}

// collidable_property_clear clears both namespaces and retains their capacities
collidable_property_clear :: proc (table: ^Collidable_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	body_status := body_property_clear(&table.bodies);
	static_status := static_property_clear(&table.statics);
	if body_status != .Ok
	{
		return body_status;
	}
	return static_status;
}

// collidable_property_destroy releases both namespaces
collidable_property_destroy :: proc (table: ^Collidable_Property_Table($T)) -> Status
{
	if table == nil
	{
		return .Invalid_Argument;
	}
	body_status := body_property_destroy(&table.bodies);
	static_status := static_property_destroy(&table.statics);
	if body_status != .Ok
	{
		return body_status;
	}
	return static_status;
}

// collidable_property_set_batch applies body and static values in exact input order
collidable_property_set_batch :: proc (
	table: ^Collidable_Property_Table($T),
	collidables: []Collidable_Reference,
	values: []T,
) -> (int, Status)
{
	if table == nil || len(collidables) != len(values)
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(collidables)
	{
		status := collidable_property_set(table, collidables[index], values[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(collidables), .Ok;
}

// collidable_property_remove_batch removes entries in exact input order and returns
// the successful prefix
collidable_property_remove_batch :: proc (
	table: ^Collidable_Property_Table($T),
	collidables: []Collidable_Reference,
) -> (int, Status)
{
	if table == nil
	{
		return 0, .Invalid_Argument;
	}
	for index in 0 ..< len(collidables)
	{
		status := collidable_property_remove(table, collidables[index]);
		if status != .Ok
		{
			return index, status;
		}
	}
	return len(collidables), .Ok;
}
