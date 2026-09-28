package entasis_c

import shared "entasis:entasis_c_shared"
import "base:runtime"
import entasis "entasis:entasis"

ENTASIS_CONTACT_USER_MAGIC        :: u64(0x454E544355534552);
ENTASIS_CONTACT_TRACKER_MAGIC     :: u64(0x454E544354524143);
// -----------------------------------------------------------------------------
// contact users and pair lifetime tracking
// -----------------------------------------------------------------------------

abi_contact_user_resource :: struct
{
	magic:             u64,
	allocator:         shared.Allocator,
	table:             entasis.Contact_User_Table,
	attached_trackers: i32,
	reserved:          i32,
}

abi_contact_tracker_resource :: struct
{
	magic:       u64,
	allocator:   shared.Allocator,
	tracker:     entasis.Contact_Tracker,
	bound_world: ^abi_world_resource,
	users:       ^abi_contact_user_resource,
}

abi_contact_user_get :: #force_inline proc "contextless" (
	handle: ^Entasis_Contact_User_Table,
) -> ^abi_contact_user_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_contact_user_resource)(handle.opaque);
	if resource.magic != ENTASIS_CONTACT_USER_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_contact_tracker_get :: #force_inline proc "contextless" (
	handle: ^Entasis_Contact_Tracker,
) -> ^abi_contact_tracker_resource
{
	if handle == nil || handle.opaque == nil
	{
		return nil;
	}
	resource := (^abi_contact_tracker_resource)(handle.opaque);
	if resource.magic != ENTASIS_CONTACT_TRACKER_MAGIC
	{
		return nil;
	}
	return resource;
}

abi_contact_user_table_init :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	body_capacity, static_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if table == nil || table.opaque != nil || !abi_count_valid(body_capacity) ||
	!abi_count_valid(static_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	memory, allocator_copy, status := abi_resource_allocate(
		size_of(abi_contact_user_resource), align_of(abi_contact_user_resource), allocator,
	);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	resource := (^abi_contact_user_resource)(memory);
	resource.allocator = allocator_copy;
	core_allocator, allocator_status := abi_allocator_to_core(&resource.allocator);
	if allocator_status != .Ok
	{
		abi_resource_free(
			memory,
			size_of(abi_contact_user_resource),
			align_of(abi_contact_user_resource),
			&resource.allocator
		);
		return abi_status_finish(allocator_status, diagnostic, .None);
	}
	context = runtime.default_context();
	status = entasis.contact_user_table_init(
		&resource.table, int(body_capacity), int(static_capacity), core_allocator,
	);
	if status != .Ok
	{
		abi_resource_free(
			memory,
			size_of(abi_contact_user_resource),
			align_of(abi_contact_user_resource),
			&resource.allocator
		);
		return abi_status_finish(status, diagnostic, .None);
	}
	resource.magic = ENTASIS_CONTACT_USER_MAGIC;
	table.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_contact_user_table_ensure_capacity :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	body_capacity, static_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if !abi_count_valid(body_capacity) || !abi_count_valid(static_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_table_ensure_capacity(
			&resource.table, int(body_capacity), int(static_capacity),
		), diagnostic, .None);
}

abi_contact_user_set_body :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	handle: Entasis_Body_Handle,
	user_id: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_set_body(
			&resource.table, entasis.Body_Handle(handle), user_id,
		), diagnostic, .None);
}

abi_contact_user_set_static :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	handle: Entasis_Static_Handle,
	user_id: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_set_static(
			&resource.table, entasis.Static_Handle(handle), user_id,
		), diagnostic, .None);
}

abi_contact_user_remove_body :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	handle: Entasis_Body_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_remove_body(
			&resource.table, entasis.Body_Handle(handle),
		), diagnostic, .None);
}

abi_contact_user_remove_static :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	handle: Entasis_Static_Handle,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_remove_static(
			&resource.table, entasis.Static_Handle(handle),
		), diagnostic, .None);
}

abi_contact_user_table_clear :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_user_table_clear(&resource.table), diagnostic, .None);
}

abi_contact_user_table_destroy :: proc "contextless" (
	table: ^Entasis_Contact_User_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_user_get(table);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.attached_trackers != 0
	{
		return abi_status_finish(.Shape_In_Use, diagnostic, .None);
	}
	context = runtime.default_context();
	status := entasis.contact_user_table_destroy(&resource.table);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	allocator := resource.allocator;
	resource.magic = 0;
	table.opaque = nil;
	abi_resource_free(resource, size_of(abi_contact_user_resource), align_of(abi_contact_user_resource), &allocator);
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_contact_tracker_init :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	initial_capacity: u64,
	allocator: ^Entasis_Allocator,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if tracker == nil || tracker.opaque != nil || !abi_count_valid(initial_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	memory, allocator_copy, status := abi_resource_allocate(
		size_of(abi_contact_tracker_resource), align_of(abi_contact_tracker_resource), allocator,
	);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	resource := (^abi_contact_tracker_resource)(memory);
	resource.allocator = allocator_copy;
	core_allocator, allocator_status := abi_allocator_to_core(&resource.allocator);
	if allocator_status != .Ok
	{
		abi_resource_free(
			resource,
			size_of(abi_contact_tracker_resource),
			align_of(abi_contact_tracker_resource),
			&resource.allocator
		);
		return abi_status_finish(allocator_status, diagnostic, .None);
	}
	context = runtime.default_context();
	status = entasis.contact_tracker_init(&resource.tracker, int(initial_capacity), core_allocator);
	if status != .Ok
	{
		abi_resource_free(
			resource,
			size_of(abi_contact_tracker_resource),
			align_of(abi_contact_tracker_resource),
			&resource.allocator
		);
		return abi_status_finish(status, diagnostic, .None);
	}
	resource.magic = ENTASIS_CONTACT_TRACKER_MAGIC;
	tracker.opaque = resource;
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_contact_tracker_capacity :: #force_inline proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
) -> u64
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return 0;
	}
	return u64(max(entasis.contact_tracker_capacity(&resource.tracker), 0));
}

// a bound tracker participates in its world's mutation boundary. capture the
// owner before unbind/destroy so release never dereferences the freed tracker
abi_contact_tracker_acquire :: proc "contextless" (resource: ^abi_contact_tracker_resource) -> entasis.Status
{
	owner := resource.bound_world;
	if owner == nil
	{
		return .Ok;
	}
	if owner.header.access != .Ready
	{
		return .Invalid_Argument;
	}
	owner.header.access = .Exclusive;
	return .Ok;
}

abi_contact_tracker_ensure_capacity :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	required_capacity: u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	if !abi_count_valid(required_capacity)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.contact_tracker_ensure_capacity(
			&resource.tracker, int(required_capacity),
		), diagnostic, .None);
}

abi_contact_tracker_bind :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	world: ^Entasis_World,
	users: ^Entasis_Contact_User_Table,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	if resource.bound_world != nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	world_resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(world_resource);
	if world_resource == nil
	{
		return ready;
	}
	user_resource: ^abi_contact_user_resource;
	if users != nil
	{
		user_resource = abi_contact_user_get(users);
		if user_resource == nil
		{
			return abi_status_finish(.Disposed, diagnostic, .None);
		}
	}
	context = runtime.default_context();
	user_table: ^entasis.Contact_User_Table;
	if user_resource != nil
	{
		user_table = &user_resource.table;
	}
	status := entasis.contact_tracker_bind(&resource.tracker, &world_resource.world, user_table);
	if status == .Ok
	{
		resource.bound_world = world_resource;
		resource.users = user_resource;
		world_resource.active_contact_trackers += 1;
		if user_resource != nil
		{
			user_resource.attached_trackers += 1;
		}
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_contact_tracker_clear :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	return abi_status_finish(entasis.contact_tracker_clear(&resource.tracker), diagnostic, .None);
}

abi_contact_tracker_detach :: #force_inline proc "contextless" (
	resource: ^abi_contact_tracker_resource,
)
{
	if resource.bound_world != nil
	{
		resource.bound_world.active_contact_trackers -= 1;
		resource.bound_world = nil;
	}
	if resource.users != nil
	{
		resource.users.attached_trackers -= 1;
		resource.users = nil;
	}
}

abi_contact_tracker_unbind :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	status := entasis.contact_tracker_unbind(&resource.tracker);
	if status == .Ok
	{
		abi_contact_tracker_detach(resource);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_contact_tracker_destroy :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	context = runtime.default_context();
	status := entasis.contact_tracker_destroy(&resource.tracker);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .None);
	}
	abi_contact_tracker_detach(resource);
	allocator := resource.allocator;
	resource.magic = 0;
	tracker.opaque = nil;
	abi_resource_free(
		resource,
		size_of(abi_contact_tracker_resource),
		align_of(abi_contact_tracker_resource),
		&allocator
	);
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_contact_events_drain :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	events: [^]Entasis_Contact_Event,
	capacity: u64,
	out_written, out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_written != nil
	{
		out_written^ = 0;
	}
	if out_required != nil
	{
		out_required^ = 0;
	}
	if out_written == nil || out_required == nil || !abi_count_valid(capacity) ||
	(capacity > 0 && events == nil)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	core_events := cast([^]entasis.Contact_Event)events;
	written, required, status := entasis.contact_events_drain(
		&resource.tracker, core_events[:int(capacity)],
	);
	out_written^ = u64(max(written, 0));
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .None);
}

abi_contact_events_discard :: proc "contextless" (
	tracker: ^Entasis_Contact_Tracker,
	out_required: ^u64,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if out_required == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	out_required^ = 0;
	resource := abi_contact_tracker_get(tracker);
	if resource == nil
	{
		return abi_status_finish(.Disposed, diagnostic, .None);
	}
	owner := resource.bound_world;
	if abi_contact_tracker_acquire(resource) != .Ok
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer abi_world_release(owner);
	required, status := entasis.contact_events_discard(&resource.tracker);
	out_required^ = u64(max(required, 0));
	return abi_status_finish(status, diagnostic, .None);
}
#assert(size_of(Entasis_Contact_Event) == size_of(entasis.Contact_Event));
#assert(align_of(Entasis_Contact_Event) == align_of(entasis.Contact_Event));
