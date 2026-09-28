package entasis_physics

import "base:runtime"
import "core:slice"
import util "entasis:entasis_utilities"

Collider_Part_Event_Subscription :: enum u8
{
	Disabled,
	Enabled,
}

Collider_Part_Endpoint :: struct
{
	reference: Collidable_Reference,
	child_index: i32,
	instance, incarnation, serial, user_id: u64,
}

Collider_Part_Pair :: struct
{
	a, b: Collider_Part_Endpoint,
	flags: u32,
	reserved: u32,
}

Collider_Part_Event :: struct
{
	pair: Collider_Part_Pair,
	step, epoch: u64,
	kind: Trigger_Event_Kind,
	reason: Trigger_Exit_Reason,
}

Mixed_Collider_Observation :: struct
{
	candidate, child_a, child_b: i32,
	flags: u32,
}

Mixed_Collider_Worker :: struct
{
	observations: util.Buffer(Mixed_Collider_Observation),
	roles: util.Buffer(u8),
	count: int,
	reduction_state: Reference_State,
}

mixed_colliders_reserve_events :: proc (
	storage: ^Mixed_Collider_Storage, configuration: Collider_Part_Configuration,
) -> Physics_Status
{
	pool: ^util.Buffer_Pool = storage.owner.simulation.pool;
	old_workers: int = int(storage.workers.length);
	status: Physics_Status = physics_ensure_buffer_capacity(pool, &storage.workers,
		len(storage.owner.workers), int(storage.workers.length));
	if status != .Ok
	{
		return status;
	}
	// pool buffers are not zeroed. every newly owned worker must start empty,
	// including rounded spare slots visited during partial-init cleanup
	_ = util.buffer_clear(storage.workers, old_workers, int(storage.workers.length)-old_workers);
	for index in 0 ..< int(storage.workers.length)
	{
		worker: ^Mixed_Collider_Worker = &storage.workers.memory[index];
		if index >= len(storage.owner.workers)
		{
			break;
		}
		status = physics_ensure_buffer_capacity(pool, &worker.roles,
			len(storage.owner.workers[index].candidates), 0);
		if status != .Ok
		{
			return status;
		}
		if configuration.event_subscription == .Enabled
		{
			status = physics_ensure_buffer_capacity(pool, &worker.observations, int(configuration.observations_per_worker), 0);
			if status != .Ok
			{
				return status;
			}
		}
	}
	if configuration.event_subscription == .Enabled
	{
		status = physics_ensure_buffer_capacity(pool, &storage.previous, int(configuration.pair_capacity), storage.previous_count);
		if status == .Ok
		{
			status = physics_ensure_buffer_capacity(pool, &storage.current, int(configuration.pair_capacity), storage.current_count);
		}
		if status == .Ok
		{
			status = physics_ensure_buffer_capacity(pool, &storage.events, 2*int(configuration.pair_capacity), storage.event_count);
		}
		if status == .Ok
		{
			buckets: int = 1;
			for buckets < 2*int(storage.current.length)
			{
				buckets *= 2;
			}
			status = physics_ensure_buffer_capacity(pool, &storage.pair_slots, buckets, 0);
		}
		if status != .Ok
		{
			return status;
		}
	}
	return .Ok;
}

mixed_colliders_release_events :: proc (storage: ^Mixed_Collider_Storage)
{
	pool: ^util.Buffer_Pool = storage.owner.simulation.pool;
	for index in 0 ..< int(storage.workers.length)
	{
		worker: ^Mixed_Collider_Worker = &storage.workers.memory[index];
		if worker.roles.memory != nil
		{
			_ = util.buffer_pool_return(pool, &worker.roles);
		}
		if worker.observations.memory != nil
		{
			_ = util.buffer_pool_return(pool, &worker.observations);
		}
	}
	if storage.workers.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.workers);
	}
	if storage.previous.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.previous);
	}
	if storage.current.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.current);
	}
	if storage.events.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.events);
	}
	if storage.pair_slots.memory != nil
	{
		_ = util.buffer_pool_return(pool, &storage.pair_slots);
	}
}

mixed_colliders_part_index :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference, child_index: i32,
) -> i32
{
	target: Shape_Query_Target;
	status: Physics_Status;
	target, status = simulation_query_target(storage.owner.simulation, reference);
	if status != .Ok
	{
		return child_index;
	}
	type_id: i32 = typed_index_type(target.shape);
	return child_index if type_id == COMPOUND_TYPE_ID || type_id == BIG_COMPOUND_TYPE_ID else 0;
}

mixed_trigger_observe :: proc (
	worker: ^Trigger_Worker, id, child_a, child_b: i32, $collect: Reference_State,
) -> Physics_Status
{
	storage: ^Mixed_Collider_Storage = worker.owner.mixed;
	pair: Broad_Phase_Pair = worker.candidates[id];
	part_a: i32 = mixed_colliders_part_index(storage, pair.a, child_a);
	part_b: i32 = mixed_colliders_part_index(storage, pair.b, child_b);
	a: Collider_Part_Settings = mixed_colliders_part_settings(storage, pair.a, part_a);
	b: Collider_Part_Settings = mixed_colliders_part_settings(storage, pair.b, part_b);
	slot_a: ^Trigger_Slot = trigger_slot(worker.owner, pair.a);
	slot_b: ^Trigger_Slot = trigger_slot(worker.owner, pair.b);
	flags: u32;
	if slot_a.enabled == .Enabled || a.role == .Trigger
	{
		flags |= 1;
	}
	if slot_b.enabled == .Enabled || b.role == .Trigger
	{
		flags |= 2;
	}
	if (slot_a.enabled == .Enabled && slot_a.settings.stay == .Enabled) || (slot_b.enabled == .Enabled && slot_b.settings.stay == .Enabled) ||
	(a.role == .Trigger && .Stay in a.options) || (b.role == .Trigger && .Stay in b.options)
	{
		flags |= 4;
	}
	if (flags & 3) == 0
	{
		return .Ok;
	}
	if collidable_reference_mobility(pair.a) == .Static && collidable_reference_mobility(pair.b) == .Static &&
	!(slot_a.enabled == .Enabled && slot_a.settings.static_static == .Enabled) && !(slot_b.enabled == .Enabled && slot_b.settings.static_static == .Enabled) &&
	!(a.role == .Trigger && .Static_Static in a.options) && !(b.role == .Trigger && .Static_Static in b.options)
	{
		return .Ok;
	}
	output: ^Mixed_Collider_Worker = &storage.workers.memory[worker.index];
	worker.hits[id] = .Present;
	output.roles.memory[id] |= u8(flags);
	when collect == .Present
	{
		if output.count == int(output.observations.length)
		{
			return .Capacity_Missing;
		}
		output.observations.memory[output.count] = {candidate=id, child_a=part_a, child_b=part_b, flags=flags};
		output.count += 1;
	}
	return .Ok;
}

// the batcher completes every continuation before publishing any parent.
// inspect the reduced child storage once per bounded flush, after mesh edge
// correction has removed invalid contacts. never retain provisional children
mixed_trigger_completed :: proc "contextless" ($collect: Reference_State) -> Collision_Pair_Result_Proc
{
	return proc "contextless" (user: rawptr, id: i32, manifold: ^Manifold_Result) -> Physics_Status
	{
		context = runtime.default_context();
		worker: ^Trigger_Worker = (^Trigger_Worker)(user);
		output: ^Mixed_Collider_Worker = &worker.owner.mixed.workers.memory[worker.index];
		batcher: ^Collision_Batcher = &worker.batcher;
		if output.reduction_state == .Missing
		{
			output.reduction_state = .Present;
			for continuation_index in 0 ..< batcher.continuation_count
			{
				continuation: ^Collision_Batcher_Continuation = &batcher.continuations.memory[continuation_index];
				parent_id: i32 = batcher.pairs.memory[continuation.parent_pair_index].pair_id;
				output.roles.memory[parent_id] |= 8;
				for child_index in int(continuation.child_start) ..< int(continuation.child_start + continuation.child_count)
				{
					header: ^Collision_Child_Manifold_Header = collision_child_storage_header(batcher.children, child_index);
					for contact_index in 0 ..< int(header.contact_count)
					{
						contact: Collision_Child_Contact_Payload = collision_child_storage_read_contact(batcher.children, child_index, contact_index);
						if contact.depth < 0
						{
							continue;
						}
						status: Physics_Status = mixed_trigger_observe(worker, parent_id, header.child_a, header.child_b, collect);
						if status != .Ok
						{
							return status;
						}
						break;
					}
				}
			}
		}
		if (output.roles.memory[id] & 8) != 0
		{
			return .Ok;
		}
		if manifold.kind == .Convex
		{
			for index in 0 ..< int(manifold.convex.count)
			{
				if manifold.convex.contacts[index].depth >= 0
				{
					return mixed_trigger_observe(worker, id, 0, 0, collect);
				}
			}
		}
		else
		{
			for index in 0 ..< int(manifold.nonconvex.count)
			{
				if manifold.nonconvex.contacts[index].depth >= 0
				{
					return mixed_trigger_observe(worker, id, 0, 0, collect);
				}
			}
		}
		return .Ok;
	};
}

// participation is assigned only after all workers join. it uses pre-reserved
// dictionaries. no worker inserts into a shared map or allocates per observation
mixed_colliders_endpoint :: proc (
	storage: ^Mixed_Collider_Storage, reference: Collidable_Reference, child_index: i32,
) -> (Collider_Part_Endpoint, Physics_Status)
{
	owner: ^Trigger_System = storage.owner;
	key: u64 = collider_instance_key(reference);
	index: int = util.quick_dictionary_index_of(&storage.instances, &key);
	instance: Collider_Instance_Record;
	if index >= 0
	{
		instance = storage.instances.values.memory[index];
	}
	else
	{
		if storage.instances.count == int(storage.instances.keys.length) || owner.serial == max(u64)
		{
			return {}, .Capacity_Missing;
		}
		target: Shape_Query_Target;
		status: Physics_Status;
		target, status = simulation_query_target(owner.simulation, reference);
		if status != .Ok
		{
			return {}, status;
		}
		instance.part_count, status=collider_part_count(simulation_shape_registry(owner.simulation), target.shape);
		if status != .Ok
		{
			return {}, status;
		}
		owner.serial += 1;
		instance.shape=target.shape;
		instance.incarnation=owner.serial;
		_ = util.quick_dictionary_add_unsafely(&storage.instances, key, instance);
	}
	part_key: Collider_Part_Key = {incarnation=instance.incarnation, child_index=child_index};
	part_index: int = util.quick_dictionary_index_of(&storage.parts, &part_key);
	part: Collider_Part_Record;
	if part_index >= 0
	{
		part=storage.parts.values.memory[part_index];
	}
	else
	{
		if storage.parts.count==int(storage.parts.keys.length) || owner.serial==max(u64)
		{
			return {}, .Capacity_Missing;
		}
		owner.serial+=1;
		part.serial=owner.serial;
		_ = util.quick_dictionary_add_unsafely(&storage.parts, part_key, part);
	}
	slot: ^Trigger_Slot = trigger_slot(owner, reference);
	user_id: u64 = part.settings.user_id if part.binding==.Present else slot.settings.user_id;
	return {reference=reference, child_index=child_index, instance=slot.token, incarnation=instance.incarnation, serial=part.serial, user_id=user_id}, .Ok;
}

mixed_colliders_endpoint_state :: proc (storage: ^Mixed_Collider_Storage, endpoint: ^Collider_Part_Endpoint) -> Reference_State
{
	slot: ^Trigger_Slot = trigger_slot(storage.owner, endpoint.reference);
	if slot==nil || slot.token!=endpoint.instance
	{
		return .Missing;
	}
	key: Collider_Part_Key = {incarnation=endpoint.incarnation, child_index=endpoint.child_index};
	index: int = util.quick_dictionary_index_of(&storage.parts, &key);
	if index<0 || storage.parts.values.memory[index].serial!=endpoint.serial
	{
		return .Missing;
	}
	return .Present;
}

mixed_colliders_pair_compare :: proc (a, b, user:rawptr) -> slice.Ordering
{
	_ = user;
	lhs: ^Collider_Part_Pair=(^Collider_Part_Pair)(a);
	rhs: ^Collider_Part_Pair=(^Collider_Part_Pair)(b);
	if lhs.a.serial<rhs.a.serial
	{
		return .Less;
	}
	if lhs.a.serial>rhs.a.serial
	{
		return .Greater;
	}
	if lhs.b.serial<rhs.b.serial
	{
		return .Less;
	}
	if lhs.b.serial>rhs.b.serial
	{
		return .Greater;
	}
	return .Equal;
}

mixed_colliders_pair_append :: proc (storage:^Mixed_Collider_Storage, pair:Collider_Part_Pair) -> Physics_Status
{
	mask: u64=u64(storage.pair_slots.length-1);
	hash: u64=pair.a.serial*u64(961748927)+pair.b.serial*u64(899809343);
	bucket: int=int((hash~(hash>>32))&mask);
	for
	{
		encoded: i32=storage.pair_slots.memory[bucket];
		if encoded==0
		{
			break;
		}
		previous: ^Collider_Part_Pair=&storage.current.memory[encoded-1];
		if previous.a.serial==pair.a.serial && previous.b.serial==pair.b.serial
		{
			previous.flags|=pair.flags;
			return .Ok;
		}
		bucket=(bucket+1)&int(mask);
	}
	if storage.current_count==int(storage.configuration.pair_capacity)
	{
		return .Capacity_Missing;
	}
	storage.current.memory[storage.current_count]=pair;
	storage.pair_slots.memory[bucket]=i32(storage.current_count+1);
	storage.current_count+=1;
	return .Ok;
}

mixed_colliders_detect_events :: proc (storage:^Mixed_Collider_Storage, workers:int) -> Physics_Status
{
	if storage.configuration.event_subscription!=.Enabled
	{
		return .Ok;
	}
	owner: ^Trigger_System=storage.owner;
	observations: int;
	for index in 0..<workers
	{
		observations+=storage.workers.memory[index].count;
	}
	storage.current_count=0;
	if observations>0
	{
		_=util.buffer_clear(storage.pair_slots, 0, int(storage.pair_slots.length));
	}
	for index in 0..<storage.previous_count
	{
		pair: ^Collider_Part_Pair=&storage.previous.memory[index];
		if mixed_colliders_endpoint_state(storage, &pair.a)!=.Present || mixed_colliders_endpoint_state(storage, &pair.b)!=.Present
		{
			continue;
		}
		a: ^Trigger_Slot=trigger_slot(owner, pair.a.reference);
		b: ^Trigger_Slot=trigger_slot(owner, pair.b.reference);
		if owner.all_dirty == .Present || a.dirty == .Present || b.dirty == .Present || trigger_active(owner, pair.a.reference) == .Present || trigger_active(owner, pair.b.reference) == .Present
		{
			continue;
		}
		if observations>0
		{
			status: Physics_Status=mixed_colliders_pair_append(storage, pair^);
			if status!=.Ok
			{
				return status;
			}
		}
		else
		{
			storage.current.memory[storage.current_count]=pair^;
			storage.current_count+=1;
		}
	}
	if observations>0
	{
		for worker_index in 0..<workers
		{
			worker: ^Mixed_Collider_Worker=&storage.workers.memory[worker_index];
			for index in 0..<worker.count
			{
				observation: Mixed_Collider_Observation=worker.observations.memory[index];
				candidate: Broad_Phase_Pair=owner.workers[worker_index].candidates[observation.candidate];
				pair: Collider_Part_Pair;
				status: Physics_Status;
				pair.a, status=mixed_colliders_endpoint(storage, candidate.a, observation.child_a);
				if status!=.Ok
				{
					return status;
				}
				pair.b, status=mixed_colliders_endpoint(storage, candidate.b, observation.child_b);
				if status!=.Ok
				{
					return status;
				}
				pair.flags=observation.flags;
				if pair.b.serial<pair.a.serial
				{
					pair.a, pair.b=pair.b, pair.a;
					pair.flags=((pair.flags&1)<<1)|((pair.flags&2)>>1)|(pair.flags&4);
				}
				status=mixed_colliders_pair_append(storage, pair);
				if status!=.Ok
				{
					return status;
				}
			}
		}
		slice.sort_by_generic_cmp(storage.current.memory[:storage.current_count], mixed_colliders_pair_compare, nil);
	}
	return .Ok;
}

mixed_colliders_publish :: #force_no_inline proc "contextless" (storage:^Mixed_Collider_Storage)
{
	context=runtime.default_context();
	if storage.configuration.event_subscription!=.Enabled
	{
		return;
	}
	storage.event_count=0;
	i, j: int;
	for i<storage.previous_count || j<storage.current_count
	{
		old, next: Collider_Part_Pair;
		if i<storage.previous_count
		{
			old=storage.previous.memory[i];
		}
		if j<storage.current_count
		{
			next=storage.current.memory[j];
		}
		kind: Trigger_Event_Kind=.Enter;
		pair: Collider_Part_Pair=next;
		emit: Reference_State=.Present;
		if j>=storage.current_count || (i<storage.previous_count && (old.a.serial<next.a.serial || (old.a.serial==next.a.serial&&old.b.serial<next.b.serial)))
		{
			pair=old;
			kind=.Exit;
			i+=1;
		}
		else if i<storage.previous_count && old.a.serial==next.a.serial && old.b.serial==next.b.serial
		{
			kind=.Stay;
			emit=.Present if (next.flags&4)!=0 else .Missing;
			i+=1;
			j+=1;
		}
		else
		{
			j+=1;
		}
		if emit==.Present
		{
			reason: Trigger_Exit_Reason=.Separation;
			if kind==.Exit
			{
				if mixed_colliders_endpoint_state(storage, &pair.a)!=.Present || mixed_colliders_endpoint_state(storage, &pair.b)!=.Present
				{
					reason=.Removed;
				}
				else
				{
					a: Collider_Part_Settings=mixed_colliders_part_settings(storage, pair.a.reference, pair.a.child_index);
					b: Collider_Part_Settings=mixed_colliders_part_settings(storage, pair.b.reference, pair.b.child_index);
					if trigger_member(storage.owner, pair.a.reference) == .Missing && trigger_member(storage.owner, pair.b.reference) == .Missing && a.role!=.Trigger && b.role!=.Trigger
					{
						reason=.Disabled;
					}
					else if storage.owner.filters_dirty == .Present
					{
						reason=.Filter_Changed;
					}
				}
			}
			storage.events.memory[storage.event_count]={pair=pair, step=storage.owner.simulation.step_index,
				epoch=storage.owner.epoch, kind=kind, reason=reason};
			storage.event_count+=1;
		}
	}
	storage.previous, storage.current=storage.current, storage.previous;
	storage.previous_count=storage.current_count;
	storage.current_count=0;
}
