package entasis_physics
import "base:runtime"
import "core:mem"
import "core:slice"
import util "entasis:entasis_utilities"
Trigger_Selection :: enum u8
{
	Disabled, Enabled
}

Trigger_Collidable_Kind :: enum u8
{
	Body, Static
}

Trigger_Change :: enum u8
{
	Modified, Removed
}

// optional root storage. no trigger state is appended to bodies, contacts or SIMD lanes
Trigger_Configuration :: struct
{
	pair_capacity: i32,
	candidates_per_worker: i32,
	child_capacity: i32,
}

Trigger_Settings :: struct
{
	user_id: u64,
	stay: Trigger_Selection,
	static_static: Trigger_Selection,
}

Trigger_Pair :: struct
{
	a, b: Collidable_Reference,
	token_a, token_b: u64,
	user_a, user_b: u64,
	flags: u32, // bit 0: a sensor. bit 1: b sensor. bit 2: Stay requested
	reserved: u32, // explicit zeroed padding for the public pointer-free view
}

Trigger_Event_Kind :: enum u32
{
	Enter, Stay, Exit
}

Trigger_Exit_Reason :: enum u32
{
	Separation, Removed, Disabled, Filter_Changed
}

Trigger_Event :: struct
{
	pair: Trigger_Pair,
	step, epoch: u64,
	kind: Trigger_Event_Kind,
	reason: Trigger_Exit_Reason,
}

Trigger_Slot :: struct
{
	token: u64,
	settings: Trigger_Settings,
	enabled: Trigger_Selection,
	dirty: Reference_State,
	late_dirty: Reference_State, // mutation after the latest collision sample. uses existing slot padding
}

Trigger_Worker :: struct
{
	owner: ^Trigger_System,
	pool: util.Buffer_Pool,
	batcher: Collision_Batcher,
	scratch: Collision_Batcher_Scratch,
	storage: util.Buffer(Collision_Batcher_Pair),
	candidates: []Broad_Phase_Pair,
	hits: []Reference_State,
	count: int,
	index: int,
	status: Physics_Status,
}

Trigger_System :: struct
{
	simulation: ^Simulation,
	allocator: mem.Allocator,
	scope: util.Allocation_Scope,
	configuration: Trigger_Configuration,
	workers: []Trigger_Worker,
	bodies, statics: []Trigger_Slot,
	dirty: []u64,
	dirty_count: int,
	enabled_count: int,
	previous, current: []Trigger_Pair,
	pair_slots: []i32,
	previous_count, current_count: int,
	events: []Trigger_Event,
	event_count: int,
	serial, epoch, sampled_step: u64,
	sample_valid: Reference_State,
	notification_status: Physics_Status,
	all_dirty: Reference_State,
	filters_dirty: Reference_State,
	mixed: ^Mixed_Collider_Storage,
}

trigger_configuration_default :: proc "contextless" () -> Trigger_Configuration
{
	return {pair_capacity=1024, candidates_per_worker=1024, child_capacity=4096};
}

trigger_configuration_validate :: proc "contextless" (c: Trigger_Configuration) -> Physics_Status
{
	return .Ok if c.pair_capacity > 0 && c.pair_capacity <= max(i32)/2 && c.candidates_per_worker > 0 && c.child_capacity > 0 else .Invalid_Description;
}

trigger_free_slice :: proc(s: ^Trigger_System, data: ^[]$T)
{
	if len(data^) > 0
	{
		_ = util.allocation_free(raw_data(data^), len(data^)*size_of(T), align_of(T), s.allocator, s.scope);
	}
	data^ = nil;
}

trigger_grow_slice :: proc(s: ^Trigger_System, data: ^[]$T, count: int) -> Physics_Status
{
	if count <= len(data^)
	{
		return .Ok;
	}
	if count <= 0 || count > max(int)/size_of(T)
	{
		return .Capacity_Missing;
	}
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(count*size_of(T), align_of(T), s.allocator);
	if error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	next: []T = ([^]T)(memory)[:count];
	copy(next, data^);
	trigger_free_slice(s, data);
	data^ = next;
	return .Ok;
}

trigger_slot :: #force_inline proc "contextless" (s: ^Trigger_System, reference: Collidable_Reference) -> ^Trigger_Slot
{
	slots: []Trigger_Slot = s.bodies;
	if collidable_reference_mobility(reference) == .Static
	{
		slots = s.statics;
	}
	handle: int = int(collidable_reference_raw_handle(reference));
	if handle < 0 || handle >= len(slots)
	{
		return nil;
	}
	return &slots[handle];
}

trigger_member :: #force_inline proc "contextless" (s: ^Trigger_System, r: Collidable_Reference) -> Reference_State
{
	slot: ^Trigger_Slot = trigger_slot(s, r);
	return .Present if slot != nil && slot.enabled == .Enabled else .Missing;
}

trigger_changed :: proc "contextless" (s: ^Trigger_System, source: Trigger_Collidable_Kind, handle: i32, change: Trigger_Change)
{
	if s.mixed != nil
	{
		context = runtime.default_context();
		key: u64 = u64(u32(handle)) | (u64(1)<<32 if source == .Static else 0);
		mixed_colliders_changed(s.mixed, key, .Present if change == .Removed else .Missing);
	}
	slots: []Trigger_Slot = s.bodies;
	if source == .Static
	{
		slots = s.statics;
	}
	if handle < 0 || int(handle) >= len(slots)
	{
		s.all_dirty = .Present;
		return;
	}
	slot: ^Trigger_Slot = &slots[handle];
	if s.sample_valid == .Present
	{
		slot.late_dirty = .Present;
	}
	if change == .Removed
	{
		if slot.enabled == .Enabled
		{
			s.enabled_count -= 1;
		}
		slot.token = 0;
		slot.enabled = .Disabled;
		slot.settings = {};
	}
	if slot.dirty == .Missing
	{
		slot.dirty = .Present;
		if s.dirty_count < len(s.dirty)
		{
			s.dirty[s.dirty_count] = u64(u32(handle)) | (u64(1) << 32 if source == .Static else 0);
			s.dirty_count += 1;
		}
		else
		{
			s.all_dirty = .Present;
		}
	}
}

trigger_reference :: proc "contextless" (s: ^Trigger_System, key: u64) -> (Collidable_Reference, Physics_Status)
{
	handle: i32 = i32(u32(key));
	if (key >> 32) != 0
	{
		status: Physics_Status;
		_, status = statics_resolve(&s.simulation.statics, {handle});
		r: Collidable_Reference;
		r, _ = collidable_reference_create(.Static, int(handle));
		return r, status;
	}
	loc: Body_Memory_Location;
	status: Physics_Status;
	loc, status = bodies_resolve(&s.simulation.bodies, {handle});
	if status != .Ok
	{
		return {}, status;
	}
	set: ^Body_Set = &s.simulation.bodies.sets.memory[loc.set_index];
	mobility: Body_Mobility = body_inertia_mobility(set.dynamics_state.memory[loc.index].inertia.local);
	return collidable_reference_create(mobility, int(handle));
}

trigger_active :: proc "contextless" (s: ^Trigger_System, r: Collidable_Reference) -> Reference_State
{
	if collidable_reference_mobility(r) == .Static
	{
		return .Missing;
	}
	loc: Body_Memory_Location;
	status: Physics_Status;
	loc, status = bodies_resolve(&s.simulation.bodies, {collidable_reference_raw_handle(r)});
	return .Present if status == .Ok && loc.set_index == 0 else .Missing;
}

trigger_eligible :: proc "contextless" (s: ^Trigger_System, pair: Broad_Phase_Pair, $mixed: Reference_State) -> Reference_State
{
	a: ^Trigger_Slot = trigger_slot(s, pair.a);
	b: ^Trigger_Slot = trigger_slot(s, pair.b);
	sensor_a: Reference_State = .Present if a != nil && a.enabled == .Enabled else .Missing;
	sensor_b: Reference_State = .Present if b != nil && b.enabled == .Enabled else .Missing;
	options_a, options_b: Collider_Part_Options;
	when mixed == .Present
	{
		context = runtime.default_context();
		sensor_a = .Present if sensor_a == .Present || mixed_colliders_sensor(s.mixed, pair.a)==.Present else .Missing;
		sensor_b = .Present if sensor_b == .Present || mixed_colliders_sensor(s.mixed, pair.b)==.Present else .Missing;
		options_a = mixed_colliders_sensor_options(s.mixed, pair.a);
		options_b = mixed_colliders_sensor_options(s.mixed, pair.b);
	}
	if sensor_a == .Missing && sensor_b == .Missing
	{
		return .Missing;
	}
	if pair.a.packed == pair.b.packed
	{
		return .Missing;
	}
	if collidable_reference_mobility(pair.a) == .Static && collidable_reference_mobility(pair.b) == .Static
	{
		return .Present if (a != nil && a.enabled == .Enabled && a.settings.static_static == .Enabled) || (b != nil && b.enabled == .Enabled && b.settings.static_static == .Enabled) || .Static_Static in options_a || .Static_Static in options_b else .Missing;
	}
	return .Present;
}

trigger_candidate :: proc "contextless" (s: ^Trigger_System, worker_index: int, pair: Broad_Phase_Pair, $mixed: Reference_State) -> Physics_Status
{
	if trigger_eligible(s, pair, mixed) == .Missing
	{
		return .Ok;
	}
	w: ^Trigger_Worker = &s.workers[worker_index];
	if w.count >= len(w.candidates)
	{
		return .Capacity_Missing;
	}
	canonical: Collidable_Pair = collidable_pair_create(pair.a, pair.b);
	if canonical.a.packed > canonical.b.packed && collidable_reference_mobility(canonical.a)==.Static && collidable_reference_mobility(canonical.b)==.Static
	{
		canonical.a, canonical.b=canonical.b, canonical.a;
	}
	w.candidates[w.count] = {canonical.a, canonical.b};
	w.count += 1;
	return .Ok;
}

Trigger_Overlap_Context :: struct
{
	base: ^Narrow_Phase_Overlap_Context, triggers: ^Trigger_System
}

trigger_overlap_visitor :: #force_inline proc "contextless" ($route: Narrow_Phase_Collision_Route, $mixed: Reference_State) -> Broad_Phase_Pair_Visitor_Proc
{
	return proc "contextless" (user: rawptr, worker: int, pair: Broad_Phase_Pair) -> Physics_Status
	{
		ctx: ^Trigger_Overlap_Context = (^Trigger_Overlap_Context)(user);
		if trigger_member(ctx.triggers, pair.a) == .Present || trigger_member(ctx.triggers, pair.b) == .Present
		{
			return trigger_candidate(ctx.triggers, worker, pair, mixed);
		}
		when mixed == .Present
		{
			context = runtime.default_context();
			storage: ^Mixed_Collider_Storage = ctx.triggers.mixed;
			if mixed_colliders_sensor(storage, pair.a)==.Present || mixed_colliders_sensor(storage, pair.b)==.Present
			{
				status: Physics_Status = trigger_candidate(ctx.triggers, worker, pair, mixed);
				if status!=.Ok
				{
					return status;
				}
				if mixed_colliders_solid_admission(storage, pair.a)==.Missing || mixed_colliders_solid_admission(storage, pair.b)==.Missing
				{
					return .Ok;
				}
			}
		}
		return narrow_phase_overlap_visitor(route)(ctx.base, worker, pair);
	};
}

trigger_completed :: proc "contextless" (user: rawptr, id: i32, manifold: ^Manifold_Result) -> Physics_Status
{
	w: ^Trigger_Worker = (^Trigger_Worker)(user);
	if id < 0 || int(id) >= w.count || manifold == nil
	{
		return .Invalid_Argument;
	}
	// AABB/speculative-only overlap is never a geometric trigger event
	if manifold.kind == .Convex
	{
		for i in 0..<int(manifold.convex.count)
		{
			if manifold.convex.contacts[i].depth >= 0
			{
				w.hits[id] = .Present;
				break;
			}
		}
	}
	else
	{
		for i in 0..<int(manifold.nonconvex.count)
		{
			if manifold.nonconvex.contacts[i].depth >= 0
			{
				w.hits[id] = .Present;
				break;
			}
		}
	}
	return .Ok;
}

trigger_allow_child :: proc "contextless" (user: rawptr, id, a, b: i32) -> Collision_Testing_State
{
	w: ^Trigger_Worker = (^Trigger_Worker)(user);
	if id < 0 || int(id) >= w.count
	{
		return .Reject;
	}
	callbacks: ^Narrow_Phase_Callbacks = &w.owner.simulation.narrow_phase.callbacks;
	if callbacks.allow_child == nil
	{
		return .Allow;
	}
	pair: Broad_Phase_Pair = w.candidates[id];
	return callbacks.allow_child(callbacks.user_context, w.index, pair.a, pair.b, int(a), int(b));
}

trigger_worker_release :: proc(s: ^Trigger_System, w: ^Trigger_Worker)
{
	if w.pool.state == .Ready
	{
		collision_batcher_return_scratch(&w.pool, &w.scratch);
		if w.storage.memory != nil
		{
			_ = util.buffer_pool_return(&w.pool, &w.storage);
		}
		_ = util.buffer_pool_dispose(&w.pool);
	}
	trigger_free_slice(s, &w.candidates);
	trigger_free_slice(s, &w.hits);
	w^ = {};
}

trigger_worker_initialize :: proc(s: ^Trigger_System, w: ^Trigger_Worker, index: int, c: Trigger_Configuration) -> Physics_Status
{
	w.owner = s;
	w.index = index;
	pool_status: util.Memory_Status;
	if s.scope == .All_Owned
	{
		pool_status = util.buffer_pool_initialize_with_allocator(&w.pool, s.allocator, 16384, 16);
	}
	else
	{
		pool_status = util.buffer_pool_initialize(&w.pool, 16384, 16);
	}
	if pool_status != .Ok
	{
		return physics_memory_status(pool_status);
	}
	status: Physics_Status = trigger_grow_slice(s, &w.candidates, int(c.candidates_per_worker));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &w.hits, int(c.candidates_per_worker));
	if status != .Ok
	{
		return status;
	}
	storage: util.Buffer(Collision_Batcher_Pair);
	take: util.Memory_Status;
	storage, take = util.buffer_pool_take_at_least(&w.pool, Collision_Batcher_Pair, min(256, int(c.candidates_per_worker)));
	if take != .Ok
	{
		return physics_memory_status(take);
	}
	w.storage = storage;
	scratch: Collision_Batcher_Scratch;
	scratch_status: Physics_Status;
	scratch, scratch_status = collision_batcher_take_scratch(&w.pool, int(storage.length), int(c.child_capacity));
	if scratch_status != .Ok
	{
		return scratch_status;
	}
	w.scratch = scratch;
	return collision_batcher_initialize_bound(&w.batcher, storage, simulation_shape_registry(s.simulation), &s.simulation.collision_tasks,
		{pair_completed=trigger_completed, allow_child_pair=trigger_allow_child}, w, &w.pool, scratch, int(c.child_capacity));
}

trigger_system_reserve :: proc(s: ^Trigger_System, requested: Trigger_Configuration) -> Physics_Status
{
	if trigger_configuration_validate(requested) != .Ok
	{
		return .Invalid_Description;
	}
	c: Trigger_Configuration = Trigger_Configuration{max(requested.pair_capacity, s.configuration.pair_capacity), max(requested.candidates_per_worker, s.configuration.candidates_per_worker), max(requested.child_capacity, s.configuration.child_capacity)};
	status: Physics_Status = trigger_grow_slice(s, &s.bodies, int(s.simulation.bodies.handle_to_location.length));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &s.statics, int(s.simulation.statics.handle_to_index.length));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &s.dirty, len(s.bodies)+len(s.statics));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &s.previous, int(c.pair_capacity));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &s.current, int(c.pair_capacity));
	if status != .Ok
	{
		return status;
	}
	status = trigger_grow_slice(s, &s.events, 2*int(c.pair_capacity));
	if status != .Ok
	{
		return status;
	}
	slots: int = 1;
	for slots<2*len(s.current)
	{
		slots*=2;
	}
	status = trigger_grow_slice(s, &s.pair_slots, slots);
	if status != .Ok
	{
		return status;
	}
	workers: int = int(s.simulation.allocation_sizes.workers);
	if len(s.workers) < workers || c.candidates_per_worker > s.configuration.candidates_per_worker || c.child_capacity > s.configuration.child_capacity
	{
		next: []Trigger_Worker;
		status = trigger_grow_slice(s, &next, workers);
		if status != .Ok
		{
			return status;
		}
		committed: Reference_State = .Missing;
		defer
		{
			if committed == .Missing
			{
				for &w in next
				{
					trigger_worker_release(s, &w);
				};
				trigger_free_slice(s, &next);
			}
		}
		for &w, i in next
		{
			status = trigger_worker_initialize(s, &w, i, c);
			if status != .Ok
			{
				return status;
			}
		}
		for &w in s.workers
		{
			trigger_worker_release(s, &w);
		}
		trigger_free_slice(s, &s.workers);
		s.workers = next;
		committed = .Present;
	}
	s.configuration = c;
	return .Ok;
}

trigger_system_enable :: proc(sim: ^Simulation, c: Trigger_Configuration, allocator: mem.Allocator, scope: util.Allocation_Scope) -> Physics_Status
{
	if sim == nil || sim.state != .Ready || sim.triggers != nil || trigger_configuration_validate(c) != .Ok
	{
		return .Invalid_Argument;
	}
	if sim.trigger_epoch == max(u64)
	{
		return .Capacity_Missing;
	}
	selected: mem.Allocator = util.allocation_allocator(allocator);
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(Trigger_System), align_of(Trigger_System), selected);
	if error != nil || memory == nil
	{
		return .Capacity_Missing;
	}
	s: ^Trigger_System = (^Trigger_System)(memory);
	s^ = {simulation=sim, allocator=selected, scope=scope, epoch=sim.trigger_epoch+1, all_dirty = .Present};
	status: Physics_Status = trigger_system_reserve(s, c);
	if status != .Ok
	{
		trigger_system_release(s);
		return status;
	}
	sim.trigger_epoch = s.epoch;
	sim.triggers = s;
	sim.bodies.triggers = s;
	sim.statics.triggers = s;
	return .Ok;
}

trigger_system_release :: proc(s: ^Trigger_System)
{
	if s == nil
	{
		return;
	}
	if s.mixed != nil
	{
		mixed_colliders_release(s.mixed);
		s.mixed=nil;
	}
	for &w in s.workers
	{
		trigger_worker_release(s, &w);
	}
	trigger_free_slice(s, &s.workers);
	trigger_free_slice(s, &s.bodies);
	trigger_free_slice(s, &s.statics);
	trigger_free_slice(s, &s.pair_slots);
	trigger_free_slice(s, &s.dirty);
	trigger_free_slice(s, &s.previous);
	trigger_free_slice(s, &s.current);
	trigger_free_slice(s, &s.events);
	allocator: mem.Allocator = s.allocator;
	scope: util.Allocation_Scope = s.scope;
	_ = util.allocation_free(s, size_of(Trigger_System), align_of(Trigger_System), allocator, scope);
}

trigger_system_destroy :: proc(sim: ^Simulation)
{
	s: ^Trigger_System = sim.triggers;
	sim.triggers = nil;
	sim.bodies.triggers = nil;
	sim.statics.triggers = nil;
	trigger_system_release(s);
}

trigger_reset :: proc "contextless" (s: ^Trigger_System)
{
	if s.mixed != nil
	{
		context = runtime.default_context();
		mixed_colliders_clear(s.mixed);
	}
	for &slot in s.bodies
	{
		slot = {};
	}
	for &slot in s.statics
	{
		slot = {};
	}
	s.previous_count = 0;
	s.current_count = 0;
	s.event_count = 0;
	s.dirty_count = 0;
	s.enabled_count = 0;
	s.sample_valid = .Missing;
	s.all_dirty = .Present;
	s.filters_dirty = .Missing;
	s.notification_status = .Ok;
	if s.epoch==max(u64)
	{
		s.notification_status=.Capacity_Missing;
	}
	else
	{
		s.epoch+=1;
		s.simulation.trigger_epoch=s.epoch;
	}
	// serial never rewinds, so reset cannot revive a former lifetime in this owner
}

trigger_step_prepare :: #force_no_inline proc(s: ^Trigger_System) -> Physics_Status
{
	if s.event_count != 0 || (s.mixed!=nil && s.mixed.event_count!=0)
	{
		return .Invalid_Argument;
	}
	if s.notification_status != .Ok
	{
		return s.notification_status;
	}
	status: Physics_Status = trigger_system_reserve(s, s.configuration);
	if status != .Ok
	{
		return status;
	}
	if s.mixed!=nil
	{
		status=mixed_colliders_reserve_events(s.mixed, s.mixed.configuration);
		if status!=.Ok
		{
			return status;
		}
	}
	s.sample_valid = .Missing;
	return .Ok;
}

Trigger_Dirty_Visit :: struct
{
	owner: ^Trigger_System, reference: Collidable_Reference, leaves: util.Buffer(Collidable_Reference), worker: int
}

trigger_dirty_leaf :: proc "contextless" ($mixed: Reference_State) -> Tree_Volume_Leaf_Proc
{
	return proc "contextless" (user: rawptr, leaf: int) -> Physics_Status
	{
		ctx: ^Trigger_Dirty_Visit = (^Trigger_Dirty_Visit)(user);
		if leaf < 0 || leaf >= int(ctx.leaves.length)
		{
			return .Invalid_Argument;
		}
		other: Collidable_Reference = ctx.leaves.memory[leaf];
		if trigger_same_instance(ctx.reference, other) == .Present
		{
			return .Ok;
		}
		slot: ^Trigger_Slot = trigger_slot(ctx.owner, other);
		// both endpoints may be dirty, or both sensors globally invalidated. only
		// one endpoint discovers their pair. no extra hash table or buffer is needed
		other_visited: Reference_State = .Present if slot != nil && (slot.enabled == .Enabled if ctx.owner.all_dirty == .Present else slot.dirty == .Present) else .Missing;
		when mixed==.Present
		{
			context=runtime.default_context();
			if ctx.owner.all_dirty == .Present
			{
				other_visited = .Present if other_visited == .Present || mixed_colliders_sensor(ctx.owner.mixed, other)==.Present else .Missing;
			}
		}
		if other_visited == .Present && other.packed < ctx.reference.packed
		{
			return .Ok;
		}
		return trigger_candidate(ctx.owner, ctx.worker, {ctx.reference, other}, mixed);
	};
}

trigger_discover :: proc "contextless" (s: ^Trigger_System, reference: Collidable_Reference, worker: int, $mixed: Reference_State) -> Physics_Status
{
	// the ordinary broad-phase pass already covers every pair with an active
	// endpoint. dirty discovery only fills its inactive/inactive coverage gap
	if trigger_active(s, reference) == .Present
	{
		return .Ok;
	}
	target: Shape_Query_Target;
	status: Physics_Status;
	target, status = simulation_query_target(s.simulation, reference);
	if status != .Ok
	{
		return .Ok;
	}
	// a removed endpoint is handled from committed history
	if typed_index_state(target.shape) != .Present
	{
		return .Ok;
	}
	bounds: util.Bounding_Box;
	bs: Physics_Status;
	bounds, bs = shape_registry_compute_world_bounds(simulation_shape_registry(s.simulation), target.shape, target.pose);
	if bs != .Ok
	{
		return bs;
	}
	broad: ^Broad_Phase = &s.simulation.broad_phase;
	ctx: Trigger_Dirty_Visit = Trigger_Dirty_Visit{owner=s, reference=reference, leaves=broad.static_leaves, worker=worker};
	return tree_volume_traverse(&broad.static_tree, bounds, trigger_dirty_leaf(mixed), &ctx, &s.workers[worker].pool);
}

trigger_pair_compare :: proc(lhs, rhs, user: rawptr) -> slice.Ordering
{
	_ = user;
	a: ^Broad_Phase_Pair = (^Broad_Phase_Pair)(lhs);
	b: ^Broad_Phase_Pair = (^Broad_Phase_Pair)(rhs);
	if a.a.packed < b.a.packed
	{
		return .Less;
	}
	if a.a.packed > b.a.packed
	{
		return .Greater;
	}
	if a.b.packed < b.b.packed
	{
		return .Less;
	}
	if a.b.packed > b.b.packed
	{
		return .Greater;
	}
	return .Equal;
}

trigger_geometry_mode :: proc(w: ^Trigger_Worker, $mixed: Reference_State) -> Physics_Status
{
	s: ^Trigger_System = w.owner;
	// per-stage dedup preserves callback order within canonical parent pairs
	slice.sort_by_generic_cmp(w.candidates[:w.count], trigger_pair_compare, nil);
	count: int = 0;
	for i in 0..<w.count
	{
		pair: Broad_Phase_Pair = w.candidates[i];
		if count != 0 && w.candidates[count-1] == pair
		{
			continue;
		}
		w.candidates[count] = pair;
		w.hits[count] = .Missing;
		count += 1;
	}
	w.count = count;
	when mixed==.Present
	{
		output: ^Mixed_Collider_Worker = &s.mixed.workers.memory[w.index];
		output.count=0;
		_=util.buffer_clear(output.roles, 0, w.count);
	}
	batcher: ^Collision_Batcher = &w.batcher;
	when mixed == .Present
	{
		batcher.procedures.allow_child_pair=mixed_trigger_allow_child;
		if s.mixed.configuration.event_subscription==.Enabled
		{
			batcher.procedures.child_pair_completed=nil;
			batcher.procedures.pair_completed=mixed_trigger_completed(.Present);
		}
		else
		{
			batcher.procedures.child_pair_completed=nil;
			batcher.procedures.pair_completed=mixed_trigger_completed(.Missing);
		}
	}
	else
	{
		batcher.procedures.allow_child_pair=trigger_allow_child;
		batcher.procedures.child_pair_completed=nil;
		batcher.procedures.pair_completed=trigger_completed;
	}
	if batcher.state == .Faulted
	{
		_ = collision_batcher_reset_fault(batcher);
	}
	batcher.pair_count = 0;
	collision_batcher_reset_task_lists(batcher);
	for pair, i in w.candidates[:w.count]
	{
		a: Shape_Query_Target;
		sa: Physics_Status;
		a, sa = simulation_query_target(s.simulation, pair.a);
		b: Shape_Query_Target;
		sb: Physics_Status;
		b, sb = simulation_query_target(s.simulation, pair.b);
		if sa != .Ok || sb != .Ok || typed_index_state(a.shape) != .Present || typed_index_state(b.shape) != .Present
		{
			continue;
		}
		callbacks: ^Narrow_Phase_Callbacks = &s.simulation.narrow_phase.callbacks;
		margin: f32;
		if callbacks.allow != nil && callbacks.allow(callbacks.user_context, w.index, pair.a, pair.b, &margin) != .Allow
		{
			continue;
		}
		route: Physics_Status;
		_, _, route = collision_task_registry_lookup(&s.simulation.collision_tasks, int(typed_index_type(a.shape)), int(typed_index_type(b.shape)));
		if route != .Ok
		{
			return route;
		}
		// candidates describe the whole stage, but parent/child scratch is bounded.
		// flush before reusing it. callbacks still receive the original candidate ID
		if batcher.pair_count == int(batcher.pairs.length)
		{
			when mixed==.Present
			{
				s.mixed.workers.memory[w.index].reduction_state=.Missing;
			}
			status: Physics_Status = collision_batcher_flush(batcher);
			if status != .Ok
			{
				return status;
			}
		}
		status: Physics_Status = collision_batcher_add(batcher, a.shape, b.shape, a.pose, b.pose, 0, i32(i));
		if status != .Ok
		{
			return status;
		}
	}
	when mixed==.Present
	{
		s.mixed.workers.memory[w.index].reduction_state=.Missing;
	}
	status: Physics_Status = collision_batcher_flush(batcher);
	if status!=.Ok
	{
		return status;
	}
	when mixed==.Present
	{
		if w.status!=.Ok
		{
			return w.status;
		}
	}
	return .Ok;
}

trigger_geometry :: proc(w:^Trigger_Worker) -> Physics_Status
{
	return trigger_geometry_mode(w, .Missing);
}

trigger_geometry_worker :: proc "contextless" ($mixed: Reference_State) -> util.Dispatcher_Worker_Proc
{
	return proc "contextless" (index: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
	{
		context = runtime.default_context();
		s: ^Trigger_System = (^Trigger_System)(dispatcher.unmanaged_context);
		w: ^Trigger_Worker = &s.workers[index];
		if w.count != 0
		{
			w.status = trigger_geometry_mode(w, mixed);
		}
	};
}

trigger_pair_record_compare :: proc(lhs, rhs, user: rawptr) -> slice.Ordering
{
	_ = user;
	a: ^Trigger_Pair = (^Trigger_Pair)(lhs);
	b: ^Trigger_Pair = (^Trigger_Pair)(rhs);
	if a.token_a < b.token_a
	{
		return .Less;
	}
	if a.token_a > b.token_a
	{
		return .Greater;
	}
	if a.token_b < b.token_b
	{
		return .Less;
	}
	if a.token_b > b.token_b
	{
		return .Greater;
	}
	return .Equal;
}

trigger_pair_same :: #force_inline proc "contextless" (a, b: Trigger_Pair) -> Reference_State
{
	return .Present if a.token_a == b.token_a && a.token_b == b.token_b else .Missing;
}

trigger_append :: proc "contextless" (s: ^Trigger_System, pair: Trigger_Pair) -> Physics_Status
{
	mask: u64 = u64(len(s.pair_slots)-1);
	hash: u64 = (pair.token_a*u64(961748927)*u64(899809343)+pair.token_b*u64(899809343));
	slot: int = int((hash~(hash>>32))&mask);
	for
	{
		encoded: int = int(s.pair_slots[slot]);
		if encoded==0
		{
			break;
		}
		if trigger_pair_same(s.current[encoded-1], pair) == .Present
		{
			s.current[encoded-1]=pair;
			return .Ok;
		}
		slot=(slot+1)&int(mask);
	}
	if s.current_count >= int(s.configuration.pair_capacity)
	{
		return .Capacity_Missing;
	}
	s.current[s.current_count] = pair;
	s.pair_slots[slot]=i32(s.current_count+1);
	s.current_count += 1;
	return .Ok;
}

trigger_token :: proc "contextless" (s: ^Trigger_System, slot: ^Trigger_Slot) -> Physics_Status
{
	if slot == nil
	{
		return .Capacity_Missing;
	}
	if slot.token == 0
	{
		if s.serial == max(u64)
		{
			return .Capacity_Missing;
		}
		s.serial += 1;
		slot.token = s.serial;
	}
	return .Ok;
}

trigger_record :: proc "contextless" (s: ^Trigger_System, pair: Broad_Phase_Pair, $mixed: Reference_State, roles: u32=0) -> Physics_Status
{
	a: ^Trigger_Slot = trigger_slot(s, pair.a);
	b: ^Trigger_Slot = trigger_slot(s, pair.b);
	status: Physics_Status = trigger_token(s, a);
	if status != .Ok
	{
		return status;
	}
	status = trigger_token(s, b);
	if status != .Ok
	{
		return status;
	}
	value: Trigger_Pair = Trigger_Pair{a=pair.a, b=pair.b, token_a=a.token, token_b=b.token, user_a=a.settings.user_id, user_b=b.settings.user_id};
	value.flags = (u32(1) if a.enabled == .Enabled else 0) | (u32(2) if b.enabled == .Enabled else 0) | (u32(4) if (a.enabled == .Enabled&&a.settings.stay == .Enabled)||(b.enabled == .Enabled&&b.settings.stay == .Enabled) else 0);
	when mixed == .Present
	{
		value.flags=roles&7;
	}
	if value.token_b < value.token_a
	{
		value.a, value.b = value.b, value.a;
		value.token_a, value.token_b = value.token_b, value.token_a;
		value.user_a, value.user_b = value.user_b, value.user_a;
		value.flags = ((value.flags&1)<<1)|((value.flags&2)>>1)|(value.flags&4);
	}
	return trigger_append(s, value);
}

trigger_detect :: proc(s: ^Trigger_System, dispatcher: ^util.Thread_Dispatcher_Boundary, workers: int, $mixed: Reference_State) -> Physics_Status
{
	s.current_count = 0;
	if s.all_dirty == .Present
	{
		// global invalidation is explicit or a rare capacity-expanding topology edit
		for slot, i in s.bodies
		{
			selected: Reference_State = .Present if slot.enabled == .Enabled else .Missing;
			when mixed==.Present
			{
				candidate: Collidable_Reference;
				resolve: Physics_Status;
				candidate, resolve = trigger_reference(s, u64(i));
				if resolve==.Ok
				{
					selected = .Present if selected == .Present || mixed_colliders_sensor(s.mixed, candidate)==.Present else .Missing;
				}
			}
			if selected == .Present
			{
				r: Collidable_Reference;
				st: Physics_Status;
				r, st = trigger_reference(s, u64(i));
				if st==.Ok
				{
					status: Physics_Status = trigger_discover(s, r, i%workers, mixed);
					if status!=.Ok
					{
						return status;
					}
				}
			}
		}
		for slot, i in s.statics
		{
			selected: Reference_State = .Present if slot.enabled == .Enabled else .Missing;
			when mixed==.Present
			{
				candidate: Collidable_Reference;
				resolve: Physics_Status;
				candidate, resolve = trigger_reference(s, u64(i)|(u64(1)<<32));
				if resolve==.Ok
				{
					selected = .Present if selected == .Present || mixed_colliders_sensor(s.mixed, candidate)==.Present else .Missing;
				}
			}
			if selected == .Present
			{
				r: Collidable_Reference;
				st: Physics_Status;
				r, st = trigger_reference(s, u64(i)|(u64(1)<<32));
				if st==.Ok
				{
					status: Physics_Status = trigger_discover(s, r, i%workers, mixed);
					if status!=.Ok
					{
						return status;
					}
				}
			}
		}
	}
	else
	{
		for key, i in s.dirty[:s.dirty_count]
		{
			r: Collidable_Reference;
			st: Physics_Status;
			r, st = trigger_reference(s, key);
			if st!=.Ok
			{
				continue;
			}
			status: Physics_Status = trigger_discover(s, r, i%workers, mixed);
			if status!=.Ok
			{
				return status;
			}
		}
	}
	candidates_available: Reference_State = .Missing;
	for &w in s.workers[:workers]
	{
		if w.count != 0
		{
			candidates_available = .Present;
			break;
		}
	}
	if candidates_available == .Present
	{
		for &slot in s.pair_slots
		{
			slot = 0;
		}
	}
	// filtering an already ordered, unique history preserves both properties.
	// with no new candidates, do not touch the capacity-sized hash scratch or
	// sort retained dormant records. dirty/removed lifetimes still exit normally
	for &old in s.previous[:s.previous_count]
	{
		a: ^Trigger_Slot = trigger_slot(s, old.a);
		b: ^Trigger_Slot = trigger_slot(s, old.b);
		if a == nil || b == nil || a.token != old.token_a || b.token != old.token_b || trigger_eligible(s, {old.a, old.b}, mixed) == .Missing
		{
			continue;
		}
		if s.all_dirty == .Missing && a.dirty == .Missing && b.dirty == .Missing && trigger_active(s, old.a) == .Missing && trigger_active(s, old.b) == .Missing
		{
			if candidates_available == .Present
			{
				status: Physics_Status = trigger_append(s, old);
				if status != .Ok
				{
					return status;
				}
			}
			else
			{
				s.current[s.current_count] = old;
				s.current_count += 1;
			}
		}
	}
	if candidates_available == .Present
	{
		if dispatcher != nil && workers > 1
		{
			ds: util.Threading_Status = dispatcher.dispatch(dispatcher, trigger_geometry_worker(mixed), workers, s);
			if ds != .Ok
			{
				return .Invalid_Argument;
			}
		}
		else
		{
			s.workers[0].status=trigger_geometry_mode(&s.workers[0], mixed);
		}
		for &w in s.workers[:workers]
		{
			if w.status != .Ok
			{
				return w.status;
			}
			for pair, i in w.candidates[:w.count]
			{
				if w.hits[i] == .Present
				{
					roles:u32;
					when mixed==.Present
					{
						roles=u32(s.mixed.workers.memory[w.index].roles.memory[i]);
					}
					st: Physics_Status = trigger_record(s, pair, mixed, roles);
					if st!=.Ok
					{
						return st;
					}
				}
			}
		}
		slice.sort_by_generic_cmp(s.current[:s.current_count], trigger_pair_record_compare, nil);
	}
	when mixed==.Present
	{
		status: Physics_Status = mixed_colliders_detect_events(s.mixed, workers);
		if status!=.Ok
		{
			return status;
		}
	}
	s.sampled_step=s.simulation.step_index;
	s.sample_valid = .Present;
	return .Ok;
}

trigger_collision_stage :: proc(s: ^Trigger_System, dispatcher: ^util.Thread_Dispatcher_Boundary, dt:f32) -> Physics_Status
{
	// a later collision stage consumes earlier invalidations, including those
	// from a previous manual stage or failed custom step. keep the dirty set
	// until successful publication. only its late-mutation marker restarts here
	if s.all_dirty == .Present
	{
		for &slot in s.bodies
		{
			slot.late_dirty = .Missing;
		}
		for &slot in s.statics
		{
			slot.late_dirty = .Missing;
		}
	}
	else
	{
		for key in s.dirty[:s.dirty_count]
		{
			slots: []Trigger_Slot = s.bodies;
			if key>>32!=0
			{
				slots=s.statics;
			}
			if int(u32(key))<len(slots)
			{
				slots[u32(key)].late_dirty = .Missing;
			}
		}
	}
	s.sample_valid = .Missing;
	if s.enabled_count == 0 && s.previous_count == 0 && (s.mixed==nil || (s.mixed.trigger_count==0 && s.mixed.previous_count==0))
	{
		// once final exits are committed, an enabled-but-empty system uses exactly
		// the solid worker specialization, without trigger lookups in its pair loop
		s.current_count = 0;
		if s.mixed!=nil
		{
			s.mixed.current_count=0;
		}
		status: Physics_Status = narrow_phase_execute(&s.simulation.narrow_phase, dispatcher, dt);
		if status == .Ok
		{
			s.sampled_step = s.simulation.step_index;
			s.sample_valid = .Present;
		}
		return status;
	}
	for &w in s.workers
	{
		w.count=0;
		w.status=.Ok;
	}
	if s.mixed!=nil
	{
		for index in 0..<len(s.workers)
		{
			worker: ^Mixed_Collider_Worker = &s.mixed.workers.memory[index];
			worker.count=0;
		}
	}
	if s.mixed!=nil
	{
		return narrow_phase_execute_mode(&s.simulation.narrow_phase, dispatcher, dt, .Enabled, s, .Present);
	}
	return narrow_phase_execute_mode(&s.simulation.narrow_phase, dispatcher, dt, .Enabled, s, .Missing);
}

trigger_step_complete_mode :: #force_no_inline proc "contextless" (s:^Trigger_System, status:Physics_Status, $mixed:Reference_State)
{
	if status!=.Ok
	{
		s.sample_valid = .Missing;
		return;
	}
	if s.sample_valid == .Missing || s.simulation.step_index!=s.sampled_step+1
	{
		s.notification_status=.Invalid_Argument;
		return;
	}
	// all capacity was reserved before the stage. successful publication cannot allocate
	s.event_count=0;
	i: int;
	j: int;
	i, j = 0, 0;
	for i<s.previous_count || j<s.current_count
	{
		old:Trigger_Pair;
		next:Trigger_Pair;
		if i<s.previous_count
		{
			old=s.previous[i];
		};
		if j<s.current_count
		{
			next=s.current[j];
		}
		kind: Trigger_Event_Kind = Trigger_Event_Kind.Enter;
		pair: Trigger_Pair = next;
		emit: Reference_State = .Present;
		reason: Trigger_Exit_Reason = Trigger_Exit_Reason.Separation;
		if j>=s.current_count || (i<s.previous_count && (old.token_a<next.token_a || (old.token_a==next.token_a&&old.token_b<next.token_b)))
		{
			pair=old;
			kind=.Exit;
			i+=1;
			a: ^Trigger_Slot = trigger_slot(s, old.a);
			b: ^Trigger_Slot = trigger_slot(s, old.b);
			if a==nil||b==nil||a.token!=old.token_a||b.token!=old.token_b
			{
				reason=.Removed;
			}
			else if trigger_eligible(s, {old.a, old.b}, mixed) == .Missing
			{
				reason=.Disabled;
			}
			else if s.filters_dirty == .Present
			{
				reason=.Filter_Changed;
			}
		}
		else if i<s.previous_count&&trigger_pair_same(old, next) == .Present
		{
			kind=.Stay;
			emit = .Present if (next.flags&4)!=0 else .Missing;
			i+=1;
			j+=1;
		}
		else
		{
			j+=1;
		}
		if emit == .Present
		{
			s.events[s.event_count]={pair=pair, step=s.simulation.step_index, epoch=s.epoch, kind=kind, reason=reason};
			s.event_count+=1;
		}
	}
	when mixed==.Present
	{
		mixed_colliders_publish(s.mixed);
	}
	s.previous, s.current=s.current, s.previous;
	s.previous_count=s.current_count;
	s.current_count=0;
	// publication acknowledges only changes represented by this sample.
	// a custom timestep may sleep bodies after collision. retain those keys in
	// the same compact queue so the next sample observes their final poses
	retained: int = 0;
	if s.all_dirty == .Present
	{
		for &slot, slot_index in s.bodies
		{
			slot.dirty = slot.late_dirty;
			if slot.late_dirty == .Present
			{
				s.dirty[retained]=u64(slot_index);
				retained+=1;
			}
			slot.late_dirty = .Missing;
		}
		for &slot, slot_index in s.statics
		{
			slot.dirty = slot.late_dirty;
			if slot.late_dirty == .Present
			{
				s.dirty[retained]=u64(slot_index)|(u64(1)<<32);
				retained+=1;
			}
			slot.late_dirty = .Missing;
		}
	}
	else
	{
		for key in s.dirty[:s.dirty_count]
		{
			slots: []Trigger_Slot = s.bodies;
			if key>>32!=0
			{
				slots=s.statics;
			}
			if int(u32(key))>=len(slots)
			{
				continue;
			}
			slot: ^Trigger_Slot = &slots[u32(key)];
			slot.dirty = slot.late_dirty;
			if slot.late_dirty == .Present
			{
				s.dirty[retained]=key;
				retained+=1;
			}
			slot.late_dirty = .Missing;
		}
	}
	s.dirty_count=retained;
	s.all_dirty = .Missing;
	s.filters_dirty = .Missing;
	s.sample_valid = .Missing;
}

trigger_step_complete :: #force_no_inline proc "contextless" (s:^Trigger_System, status:Physics_Status)
{
	if s.mixed!=nil
	{
		trigger_step_complete_mode(s, status, .Present);
	}
	else
	{
		trigger_step_complete_mode(s, status, .Missing);
	}
}

trigger_static_member :: proc "contextless" (s:^Trigger_System, handle:Static_Handle) -> Reference_State
{
	return .Present if handle.value>=0&&int(handle.value)<len(s.statics)&&s.statics[handle.value].enabled == .Enabled else .Missing;
}

trigger_same_instance :: proc "contextless" (a, b:Collidable_Reference) -> Reference_State
{
	return .Present if (collidable_reference_mobility(a)==.Static)==(collidable_reference_mobility(b)==.Static)&&collidable_reference_raw_handle(a)==collidable_reference_raw_handle(b) else .Missing;
}

// mode changes retire only contact-cache constraints, never application joints.
// sleeping records are compacted in place, without awakening their bodies
trigger_retire_contacts :: proc(s:^Trigger_System, reference:Collidable_Reference) -> Physics_Status
{
	sim: ^Simulation = s.simulation;
	cache: ^Pair_Cache = &sim.narrow_phase.pair_cache;
	solver: ^Solver = &sim.solver;
	sleeper: ^Island_Sleeper = &sim.sleeper;
	count: int = 0;
	for i in 0..<cache.mapping.count
	{
		p: Collidable_Pair = cache.mapping.keys.memory[i];
		if trigger_same_instance(p.a, reference) == .Present||trigger_same_instance(p.b, reference) == .Present
		{
			count+=1;
		}
	}
	for i in 0..<cache.inactive_count
	{
		p: Collidable_Pair = cache.inactive_entries.memory[i].pair;
		if trigger_same_instance(p.a, reference) == .Present||trigger_same_instance(p.b, reference) == .Present
		{
			count+=1;
		}
	}
	if count==0
	{
		return .Ok;
	}
	capacity: util.Memory_Status = util.id_pool_ensure_capacity(&solver.handle_pool, solver.handle_pool.available_id_count+count, solver.pool);
	if capacity!=.Ok
	{
		return physics_memory_status(capacity);
	}
	i: int = cache.mapping.count-1;
	for i>=0
	{
		pair: Collidable_Pair = cache.mapping.keys.memory[i];
		if trigger_same_instance(pair.a, reference) == .Present||trigger_same_instance(pair.b, reference) == .Present
		{
			handle: Constraint_Handle = cache.mapping.values.memory[i].constraint_handle;
			status: Physics_Status = solver_remove(solver, handle);
			if status!=.Ok
			{
				return status;
			}
			pair_cache_commit_remove(cache, pair);
		}
		i-=1;
	}
	i=cache.inactive_count-1;
	for i>=0
	{
		entry: Inactive_Pair_Cache_Entry = cache.inactive_entries.memory[i];
		if trigger_same_instance(entry.pair.a, reference) == .Present||trigger_same_instance(entry.pair.b, reference) == .Present
		{
			record_index: int;
			status: Physics_Status;
			record_index, status = island_sleeper_resolve_inactive_constraint_index(sleeper, entry.cache.constraint_handle);
			if status!=.Ok
			{
				return status;
			}
			sleeper.inactive_constraint_count-=1;
			if record_index<sleeper.inactive_constraint_count
			{
				moved: ^Inactive_Constraint_Record = &sleeper.inactive_constraints.memory[sleeper.inactive_constraint_count];
				sleeper.inactive_constraints.memory[record_index]=moved^;
				solver.handle_to_constraint.memory[moved.handle.value].index_in_type_batch=i32(record_index);
			}
			sleeper.inactive_constraints.memory[sleeper.inactive_constraint_count]={};
			solver.handle_to_constraint.memory[entry.cache.constraint_handle.value]=constraint_location_missing();
			_=util.id_pool_return_unsafely(&solver.handle_pool, entry.cache.constraint_handle.value);
			pair_cache_unbind_constraint(cache, entry.cache.constraint_handle);
			cache.inactive_count-=1;
			if i<cache.inactive_count
			{
				moved: Inactive_Pair_Cache_Entry = cache.inactive_entries.memory[cache.inactive_count];
				cache.inactive_entries.memory[i]=moved;
				cache.constraint_handle_to_pair.memory[moved.cache.constraint_handle.value].inactive_pair_index=i32(i);
			}
			cache.inactive_entries.memory[cache.inactive_count]={};
		}
		i-=1;
	}
	return .Ok;
}
