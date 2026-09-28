package entasis_c

import shared "entasis:entasis_c_shared"
import "base:intrinsics"
import entasis "entasis:entasis"

// all bridge operations stay outside the direct Odin hot path.
// query and view records use fixed layouts. all-hit outputs and event drains
// write directly to caller-owned storage and perform no bridge allocation

ENTASIS_BODY_PROPERTY_MAGIC       :: u64(0x454E544250524F50);
ENTASIS_STATIC_PROPERTY_MAGIC     :: u64(0x454E545350524F50);
ENTASIS_COLLIDABLE_PROPERTY_MAGIC :: u64(0x454E544350524F50);
ENTASIS_PROPERTY_MINIMUM_CAPACITY :: 16;
ENTASIS_PROPERTY_MAX_ALIGNMENT    :: 4096;
// -----------------------------------------------------------------------------
// type-erased dense property tables
// -----------------------------------------------------------------------------

abi_property_storage :: struct
{
	allocation:           rawptr,
	generations:          [^]u64,
	values:               rawptr,
	capacity:             int,
	value_size:           int,
	value_stride:         int,
	value_alignment:      int,
	allocation_size:      int,
	allocation_alignment: int,
	epoch:                u64,
}

abi_body_property_resource :: struct
{
	magic:     u64,
	allocator: shared.Allocator,
	storage:   abi_property_storage,
}

abi_static_property_resource :: struct
{
	magic:     u64,
	allocator: shared.Allocator,
	storage:   abi_property_storage,
}

abi_collidable_property_resource_v1 :: struct
{
	magic:     u64,
	allocator: shared.Allocator,
	bodies:    abi_property_storage,
	statics:   abi_property_storage,
}

abi_property_power_of_two :: #force_inline proc "contextless" (value: int) -> bool
{
	return value > 0 && value & (value - 1) == 0;
}

abi_property_align_up :: #force_inline proc "contextless" (
	value, alignment: int,
) -> (int, bool)
{
	if !abi_property_power_of_two(alignment) || value < 0 || value > max(int) - (alignment - 1)
	{
		return 0, false;
	}
	return (value + alignment - 1) & ~(alignment - 1), true;
}

abi_property_epoch_next :: #force_inline proc "contextless" (epoch: u64) -> u64
{
	if epoch == max(u64)
	{
		return 1;
	}
	result := epoch + 1;
	if result == 0
	{
		return 1;
	}
	return result;
}

abi_property_generation_present :: #force_inline proc "contextless" (generation: u64) -> bool
{
	return generation & 1 != 0;
}

abi_property_storage_layout :: proc "contextless" (
	capacity, value_stride, value_alignment: int,
) -> (values_offset, allocation_size, allocation_alignment: int, ok: bool)
{
	if capacity < 0 || value_stride <= 0 || value_alignment <= 0 ||
	capacity > max(int) / size_of(u64)
	{
		return 0, 0, 0, false;
	}
	generation_bytes := capacity * size_of(u64);
	values_offset, ok = abi_property_align_up(generation_bytes, value_alignment);
	if !ok || capacity > (max(int) - values_offset) / value_stride
	{
		return 0, 0, 0, false;
	}
	allocation_size = values_offset + capacity * value_stride;
	allocation_alignment = max(align_of(u64), value_alignment);
	return values_offset, allocation_size, allocation_alignment, true;
}

abi_property_storage_release :: proc "contextless" (
	storage: ^abi_property_storage,
	allocator: ^shared.Allocator,
)
{
	if storage == nil
	{
		return;
	}
	if storage.allocation != nil
	{
		abi_resource_free(
			storage.allocation,
			storage.allocation_size,
			storage.allocation_alignment,
			allocator,
		);
	}
	storage.allocation = nil;
	storage.generations = nil;
	storage.values = nil;
	storage.capacity = 0;
	storage.allocation_size = 0;
	storage.allocation_alignment = 0;
}

abi_property_storage_grow :: proc "contextless" (
	storage: ^abi_property_storage,
	minimum_capacity: int,
	allocator: ^shared.Allocator,
) -> entasis.Status
{
	if storage == nil || minimum_capacity < 0
	{
		return .Invalid_Argument;
	}
	if minimum_capacity <= storage.capacity
	{
		return .Ok;
	}
	target := max(storage.capacity, ENTASIS_PROPERTY_MINIMUM_CAPACITY);
	for target < minimum_capacity
	{
		if target > max(int) / 2
		{
			target = minimum_capacity;
			break;
		}
		target *= 2;
	}
	values_offset, allocation_size, allocation_alignment, valid := abi_property_storage_layout(
		target, storage.value_stride, storage.value_alignment,
	);
	if !valid
	{
		return .Capacity_Missing;
	}
	memory, _, status := abi_resource_allocate_owned(allocation_size, allocation_alignment, allocator);
	if status != .Ok
	{
		return status;
	}
	new_generations := ([^]u64)(memory);
	new_values := rawptr(uintptr(memory) + uintptr(values_offset));
	if storage.capacity > 0
	{
		intrinsics.mem_copy_non_overlapping(
			new_generations, storage.generations, storage.capacity * size_of(u64),
		);
		intrinsics.mem_copy_non_overlapping(
			new_values, storage.values, storage.capacity * storage.value_stride,
		);
	}
	old_allocation := storage.allocation;
	old_size := storage.allocation_size;
	old_alignment := storage.allocation_alignment;
	storage.allocation = memory;
	storage.generations = new_generations;
	storage.values = new_values;
	storage.capacity = target;
	storage.allocation_size = allocation_size;
	storage.allocation_alignment = allocation_alignment;
	if old_allocation != nil
	{
		abi_resource_free(old_allocation, old_size, old_alignment, allocator);
	}
	return .Ok;
}

abi_property_storage_configure :: proc "contextless" (
	storage: ^abi_property_storage,
	value_size, value_alignment, initial_capacity: u64,
	allocator: ^shared.Allocator,
) -> entasis.Status
{
	if storage == nil || value_size == 0 || !abi_count_valid(value_size) ||
	!abi_count_valid(value_alignment) || !abi_count_valid(initial_capacity)
	{
		return .Invalid_Argument;
	}
	alignment := int(value_alignment);
	if alignment == 0
	{
		alignment = align_of(uintptr);
	}
	if alignment > ENTASIS_PROPERTY_MAX_ALIGNMENT || !abi_property_power_of_two(alignment)
	{
		return .Invalid_Argument;
	}
	stride, ok := abi_property_align_up(int(value_size), alignment);
	if !ok || stride <= 0
	{
		return .Capacity_Missing;
	}
	storage.value_size = int(value_size);
	storage.value_stride = stride;
	storage.value_alignment = alignment;
	storage.epoch = 1;
	if initial_capacity > 0
	{
		return abi_property_storage_grow(storage, int(initial_capacity), allocator);
	}
	return .Ok;
}

abi_property_value_pointer :: #force_inline proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
) -> rawptr
{
	return rawptr(uintptr(storage.values) + uintptr(index * storage.value_stride));
}

abi_property_storage_set :: proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
	value: rawptr,
	allocator: ^shared.Allocator,
) -> entasis.Status
{
	if storage == nil || value == nil || index < 0
	{
		return .Invalid_Argument;
	}
	status := abi_property_storage_grow(storage, index + 1, allocator);
	if status != .Ok
	{
		return status;
	}
	generation := storage.generations[index];
	if !abi_property_generation_present(generation)
	{
		if generation == max(u64)
		{
			generation = 1;
			storage.epoch = abi_property_epoch_next(storage.epoch);
		}
		else
		{
			generation += 1;
			if generation == 0
			{
				generation = 1;
			}
		}
		storage.generations[index] = generation;
	}
	destination := abi_property_value_pointer(storage, index);
	intrinsics.mem_zero(destination, storage.value_stride);
	intrinsics.mem_copy_non_overlapping(destination, value, storage.value_size);
	return .Ok;
}

abi_property_storage_get :: #force_inline proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
) -> (rawptr, entasis.Status)
{
	if storage == nil || index < 0
	{
		return nil, .Invalid_Argument;
	}
	if index >= storage.capacity || !abi_property_generation_present(storage.generations[index])
	{
		return nil, .Not_Found;
	}
	return abi_property_value_pointer(storage, index), .Ok;
}

abi_property_storage_key_data :: #force_inline proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
) -> (u64, u64, entasis.Status)
{
	if storage == nil || index < 0
	{
		return 0, 0, .Invalid_Argument;
	}
	if index >= storage.capacity || !abi_property_generation_present(storage.generations[index])
	{
		return 0, 0, .Not_Found;
	}
	return storage.generations[index], storage.epoch, .Ok;
}

abi_property_storage_get_key :: #force_inline proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
	generation, epoch: u64,
) -> (rawptr, entasis.Status)
{
	if storage == nil || index < 0
	{
		return nil, .Invalid_Argument;
	}
	if epoch == 0 || epoch != storage.epoch || generation == 0 ||
	index >= storage.capacity || storage.generations[index] != generation ||
	!abi_property_generation_present(generation)
	{
		return nil, .Not_Found;
	}
	return abi_property_value_pointer(storage, index), .Ok;
}

abi_property_storage_remove :: proc "contextless" (
	storage: ^abi_property_storage,
	index: int,
) -> entasis.Status
{
	value, status := abi_property_storage_get(storage, index);
	if status != .Ok
	{
		return status;
	}
	intrinsics.mem_zero(value, storage.value_stride);
	generation := storage.generations[index];
	if generation == max(u64)
	{
		storage.generations[index] = 0;
		storage.epoch = abi_property_epoch_next(storage.epoch);
	}
	else
	{
		storage.generations[index] = generation + 1;
	}
	return .Ok;
}

abi_property_storage_clear :: proc "contextless" (
	storage: ^abi_property_storage,
) -> entasis.Status
{
	if storage == nil
	{
		return .Invalid_Argument;
	}
	if storage.capacity > 0
	{
		intrinsics.mem_zero(storage.generations, storage.capacity * size_of(u64));
		intrinsics.mem_zero(storage.values, storage.capacity * storage.value_stride);
	}
	storage.epoch = abi_property_epoch_next(storage.epoch);
	return .Ok;
}

abi_body_property_get_resource :: #force_inline proc "contextless" (
	handle: ^Entasis_Body_Property_Table,
) -> ^abi_body_property_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_body_property_resource)(handle.opaque);
	if resource.magic != ENTASIS_BODY_PROPERTY_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_static_property_get_resource :: #force_inline proc "contextless" (
	handle: ^Entasis_Static_Property_Table,
) -> ^abi_static_property_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_static_property_resource)(handle.opaque);
	if resource.magic != ENTASIS_STATIC_PROPERTY_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_collidable_property_get_resource :: #force_inline proc "contextless" (
	handle: ^Entasis_Collidable_Property_Table,
) -> ^abi_collidable_property_resource_v1
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_collidable_property_resource_v1)(handle.opaque);
	if resource.magic != ENTASIS_COLLIDABLE_PROPERTY_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_property_identity_index :: #force_inline proc "contextless" (value: i32) -> (int, entasis.Status)
{
	if value < 0
	{
		return -1, .Invalid_Argument;
	}
	return int(value), .Ok;
}

abi_property_collidable_index :: #force_inline proc "contextless" (
	collidable: Entasis_Collidable_Reference,
) -> (index: int, static_namespace: bool, status: entasis.Status)
{
	mobility := u8(collidable.packed >> 30);
	if mobility > 2
	{
		return -1, false, .Invalid_Argument;
	}
	index = int(collidable.packed & 0x3fff_ffff);
	return index, mobility == 2, .Ok;
}

// body property wrappers
abi_body_property_init :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	value_size, value_alignment, initial_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if table == nil || table.opaque != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	memory, allocator_copy, status := abi_resource_allocate(
		size_of(abi_body_property_resource), align_of(abi_body_property_resource), allocator,
	);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	resource := (^abi_body_property_resource)(memory);
	resource.allocator = allocator_copy;
	status = abi_property_storage_configure(
		&resource.storage, value_size, value_alignment, initial_capacity, &resource.allocator,
	);
	if status != .Ok
	{
		abi_property_storage_release(&resource.storage, &resource.allocator);
		abi_resource_free(
			resource,
			size_of(abi_body_property_resource),
			align_of(abi_body_property_resource),
			&resource.allocator
		);
		return abi_status_finish(status, diagnostic, .None);
	}
	resource.magic = ENTASIS_BODY_PROPERTY_MAGIC;
	table.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_property_ensure_capacity :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	return abi_status_finish(abi_property_storage_grow(
			&resource.storage, int(capacity), &resource.allocator,
		), diagnostic, .None);
}

abi_body_property_capacity :: #force_inline proc "contextless" (
	table: ^Entasis_Body_Property_Table,
) -> u64
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return 0;
	}
	return u64(resource.storage.capacity);
}

abi_body_property_set :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handle: Entasis_Body_Handle,
	value: rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, status := abi_property_identity_index(handle.value);
	if status == .Ok
	{
		status = abi_property_storage_set(&resource.storage, index, value, &resource.allocator);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_property_set_batch :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handles: [^]Entasis_Body_Handle,
	values: rawptr,
	value_stride, count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || !abi_count_valid(value_stride) ||
	(count > 0 && (handles == nil || values == nil)) ||
	value_stride < u64(resource.storage.value_size)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	maximum_index := -1;
	for index in 0 ..< int(count)
	{
		if handles[index].value < 0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None, i32(index));
		}
		maximum_index = max(maximum_index, int(handles[index].value));
	}
	status := entasis.Status.Ok;
	if maximum_index >= 0
	{
		status = abi_property_storage_grow(&resource.storage, maximum_index + 1, &resource.allocator);
	}
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		value := rawptr(uintptr(values) + uintptr(index) * uintptr(value_stride));
		status = abi_property_storage_set(&resource.storage, int(handles[index].value), value, &resource.allocator);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_property_get :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handle: Entasis_Body_Handle,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	value, status := abi_property_storage_get(&resource.storage, int(handle.value));
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_property_key :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handle: Entasis_Body_Handle,
	out_key: ^Entasis_Body_Property_Key,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_key == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_key^ = {};
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	generation, epoch, status := abi_property_storage_key_data(&resource.storage, int(handle.value));
	if status == .Ok
	{
		out_key^ = {handle=handle, generation=generation, epoch=epoch, owner=u64(uintptr(resource))};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_property_get_key :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	key: Entasis_Body_Property_Key,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if key.owner != u64(uintptr(resource))
	{
		return abi_status_finish(.Not_Found, diagnostic, .None);
	}
	value, status := abi_property_storage_get_key(&resource.storage, int(key.handle.value), key.generation, key.epoch);
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_body_property_remove :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handle: Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	return abi_status_finish(abi_property_storage_remove(&resource.storage, int(handle.value)), diagnostic, .None);
}

abi_body_property_remove_batch :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	handles: [^]Entasis_Body_Handle,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || (count > 0 && handles == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		status := abi_property_storage_remove(&resource.storage, int(handles[index].value));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_body_property_clear :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	return abi_status_finish(abi_property_storage_clear(&resource.storage), diagnostic, .None);
}

abi_body_property_destroy :: proc "contextless" (
	table: ^Entasis_Body_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_body_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	allocator := resource.allocator;
	abi_property_storage_release(&resource.storage, &allocator);
	resource.magic = 0;
	table.opaque = nil;
	abi_resource_free(resource, size_of(abi_body_property_resource), align_of(abi_body_property_resource), &allocator);
	return abi_status_finish(.Ok, diagnostic, .None);
}

// static property wrappers mirror the body table while preserving an independent namespace
abi_static_property_init :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	value_size, value_alignment, initial_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if table == nil || table.opaque != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	memory, allocator_copy, status := abi_resource_allocate(
		size_of(abi_static_property_resource), align_of(abi_static_property_resource), allocator,
	);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	resource := (^abi_static_property_resource)(memory);
	resource.allocator = allocator_copy;
	status = abi_property_storage_configure(
		&resource.storage,
		value_size,
		value_alignment,
		initial_capacity,
		&resource.allocator
	);
	if status != .Ok
	{
		abi_property_storage_release(&resource.storage, &resource.allocator);
		abi_resource_free(
			resource,
			size_of(abi_static_property_resource),
			align_of(abi_static_property_resource),
			&resource.allocator
		);
		return abi_status_finish(status, diagnostic, .None);
	}
	resource.magic = ENTASIS_STATIC_PROPERTY_MAGIC;
	table.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_property_ensure_capacity :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	return abi_status_finish(
		abi_property_storage_grow(&resource.storage, int(capacity), &resource.allocator),
		diagnostic,
		.None
	);
}

abi_static_property_capacity :: #force_inline proc "contextless" (table: ^Entasis_Static_Property_Table) -> u64
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return 0;
	}
	return u64(resource.storage.capacity);
}

abi_static_property_set :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handle: Entasis_Static_Handle,
	value: rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, status := abi_property_identity_index(handle.value);
	if status == .Ok
	{
		status = abi_property_storage_set(&resource.storage, index, value, &resource.allocator);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_property_set_batch :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handles: [^]Entasis_Static_Handle,
	values: rawptr,
	value_stride, count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || !abi_count_valid(value_stride) ||
	(count > 0 && (handles == nil || values == nil)) || value_stride < u64(resource.storage.value_size)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	maximum_index := -1;
	for index in 0 ..< int(count)
	{
		if handles[index].value < 0
		{
			return abi_status_finish(.Invalid_Argument, diagnostic, .None, i32(index));
		}
		maximum_index = max(maximum_index, int(handles[index].value));
	}
	status := entasis.Status.Ok;
	if maximum_index >= 0
	{
		status = abi_property_storage_grow(&resource.storage, maximum_index + 1, &resource.allocator);
	}
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		value := rawptr(uintptr(values) + uintptr(index) * uintptr(value_stride));
		status = abi_property_storage_set(&resource.storage, int(handles[index].value), value, &resource.allocator);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_property_get :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handle: Entasis_Static_Handle,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	value, status := abi_property_storage_get(&resource.storage, int(handle.value));
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_property_key :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handle: Entasis_Static_Handle,
	out_key: ^Entasis_Static_Property_Key,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_key == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_key^ = {};
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	generation, epoch, status := abi_property_storage_key_data(&resource.storage, int(handle.value));
	if status == .Ok
	{
		out_key^ = {handle=handle, generation=generation, epoch=epoch, owner=u64(uintptr(resource))};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_property_get_key :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	key: Entasis_Static_Property_Key,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if key.owner != u64(uintptr(resource))
	{
		return abi_status_finish(.Not_Found, diagnostic, .None);
	}
	value, status := abi_property_storage_get_key(&resource.storage, int(key.handle.value), key.generation, key.epoch);
	if status == .Ok
	{
		out_value^ = value;
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_static_property_remove :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handle: Entasis_Static_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	return abi_status_finish(abi_property_storage_remove(&resource.storage, int(handle.value)), diagnostic, .None);
}

abi_static_property_remove_batch :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	handles: [^]Entasis_Static_Handle,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || (count > 0 && handles == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		status := abi_property_storage_remove(&resource.storage, int(handles[index].value));
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_static_property_clear :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	return abi_status_finish(abi_property_storage_clear(&resource.storage), diagnostic, .None);
}

abi_static_property_destroy :: proc "contextless" (
	table: ^Entasis_Static_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_static_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	allocator := resource.allocator;
	abi_property_storage_release(&resource.storage, &allocator);
	resource.magic = 0;
	table.opaque = nil;
	abi_resource_free(
		resource,
		size_of(abi_static_property_resource),
		align_of(abi_static_property_resource),
		&allocator
	);
	return abi_status_finish(.Ok, diagnostic, .None);
}

// collidable properties keep body and static handle spaces separate within one resource
abi_collidable_property_init :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	value_size, value_alignment, body_capacity, static_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if table == nil || table.opaque != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	memory, allocator_copy, status := abi_resource_allocate(
		size_of(abi_collidable_property_resource_v1), align_of(abi_collidable_property_resource_v1), allocator,
	);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	resource := (^abi_collidable_property_resource_v1)(memory);
	resource.allocator = allocator_copy;
	status = abi_property_storage_configure(
		&resource.bodies,
		value_size,
		value_alignment,
		body_capacity,
		&resource.allocator
	);
	if status == .Ok
	{
		status = abi_property_storage_configure(
			&resource.statics,
			value_size,
			value_alignment,
			static_capacity,
			&resource.allocator
		);
	}
	if status != .Ok
	{
		abi_property_storage_release(&resource.bodies, &resource.allocator);
		abi_property_storage_release(&resource.statics, &resource.allocator);
		abi_resource_free(
			resource,
			size_of(abi_collidable_property_resource_v1),
			align_of(abi_collidable_property_resource_v1),
			&resource.allocator
		);
		return abi_status_finish(status, diagnostic, .None);
	}
	resource.magic = ENTASIS_COLLIDABLE_PROPERTY_MAGIC;
	table.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_collidable_property_ensure_capacity :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	body_capacity, static_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(body_capacity) || !abi_count_valid(static_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	status := abi_property_storage_grow(&resource.bodies, int(body_capacity), &resource.allocator);
	if status == .Ok
	{
		status = abi_property_storage_grow(&resource.statics, int(static_capacity), &resource.allocator);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_set :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidable: Entasis_Collidable_Reference,
	value: rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, static_namespace, status := abi_property_collidable_index(collidable);
	if status == .Ok
	{
		storage := static_namespace ? &resource.statics : &resource.bodies;
		status = abi_property_storage_set(storage, index, value, &resource.allocator);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_set_batch :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidables: [^]Entasis_Collidable_Reference,
	values: rawptr,
	value_stride, count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || !abi_count_valid(value_stride) ||
	(count > 0 && (collidables == nil || values == nil)) || value_stride < u64(resource.bodies.value_size)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	body_max, static_max := -1, -1;
	for index in 0 ..< int(count)
	{
		slot, static_namespace, status := abi_property_collidable_index(collidables[index]);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if static_namespace
		{
			static_max = max(static_max, slot);
		}
		else
		{
			body_max = max(body_max, slot);
		}
	}
	status := entasis.Status.Ok;
	if body_max >= 0
	{
		status = abi_property_storage_grow(&resource.bodies, body_max + 1, &resource.allocator);
	}
	if status == .Ok && static_max >= 0
	{
		status = abi_property_storage_grow(&resource.statics, static_max + 1, &resource.allocator);
	}
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		slot, static_namespace, _ := abi_property_collidable_index(collidables[index]);
		storage := static_namespace ? &resource.statics : &resource.bodies;
		value := rawptr(uintptr(values) + uintptr(index) * uintptr(value_stride));
		status = abi_property_storage_set(storage, slot, value, &resource.allocator);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_collidable_property_get :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidable: Entasis_Collidable_Reference,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, static_namespace, status := abi_property_collidable_index(collidable);
	if status == .Ok
	{
		storage := static_namespace ? &resource.statics : &resource.bodies;
		value: rawptr;
		value, status = abi_property_storage_get(storage, index);
		if status == .Ok
		{
			out_value^ = value;
		}
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_key :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidable: Entasis_Collidable_Reference,
	out_key: ^Entasis_Collidable_Property_Key,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_key == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_key^ = {};
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, static_namespace, status := abi_property_collidable_index(collidable);
	if status == .Ok
	{
		storage := static_namespace ? &resource.statics : &resource.bodies;
		generation, epoch: u64;
		generation, epoch, status = abi_property_storage_key_data(storage, index);
		if status == .Ok
		{
			out_key^ = {collidable=collidable, generation=generation, epoch=epoch, owner=u64(uintptr(resource))};
		}
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_get_key :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	key: Entasis_Collidable_Property_Key,
	out_value: ^rawptr,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_value == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_value^ = nil;
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if key.owner != u64(uintptr(resource))
	{
		return abi_status_finish(.Not_Found, diagnostic, .None);
	}
	index, static_namespace, status := abi_property_collidable_index(key.collidable);
	if status == .Ok
	{
		storage := static_namespace ? &resource.statics : &resource.bodies;
		value: rawptr;
		value, status = abi_property_storage_get_key(storage, index, key.generation, key.epoch);
		if status == .Ok
		{
			out_value^ = value;
		}
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_remove :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidable: Entasis_Collidable_Reference,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	index, static_namespace, status := abi_property_collidable_index(collidable);
	if status == .Ok
	{
		storage := static_namespace ? &resource.statics : &resource.bodies;
		status = abi_property_storage_remove(storage, index);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_remove_batch :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	collidables: [^]Entasis_Collidable_Reference,
	count: u64,
	out_completed: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	abi_batch_begin(out_completed);
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(count) || (count > 0 && collidables == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	for index in 0 ..< int(count)
	{
		slot, static_namespace, status := abi_property_collidable_index(collidables[index]);
		if status == .Ok
		{
			storage := static_namespace ? &resource.statics : &resource.bodies;
			status = abi_property_storage_remove(storage, slot);
		}
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None, i32(index));
		}
		if out_completed != nil
		{
			out_completed^ = u64(index + 1);
		}
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_collidable_property_clear :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	status := abi_property_storage_clear(&resource.bodies);
	if status == .Ok
	{
		status = abi_property_storage_clear(&resource.statics);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_collidable_property_destroy :: proc "contextless" (
	table: ^Entasis_Collidable_Property_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_collidable_property_get_resource(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	allocator := resource.allocator;
	abi_property_storage_release(&resource.bodies, &allocator);
	abi_property_storage_release(&resource.statics, &allocator);
	resource.magic = 0;
	table.opaque = nil;
	abi_resource_free(
		resource,
		size_of(abi_collidable_property_resource_v1),
		align_of(abi_collidable_property_resource_v1),
		&allocator
	);
	return abi_status_finish(.Ok, diagnostic, .None);
}
