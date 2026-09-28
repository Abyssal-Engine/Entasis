package entasis_physics

import "base:runtime"
import "core:math"
import "core:mem"
import "core:slice"
import util "entasis:entasis_utilities"

Joint_Break_Metric :: enum u32
{
	Force, Torque
}

Joint_Break_Metrics :: bit_set[Joint_Break_Metric; u32];
Joint_Break_Limits :: struct
{
	metrics: Joint_Break_Metrics, force, torque: f32, user_id: u64
}

Joint_Reaction_State :: enum u32
{
	Unsolved, Solved
}

// representative sample: strongest normalized selected load. independent peaks
// include all completed substeps. torque is about each body's center of mass
Joint_Reaction :: struct
{
	sample: Constraint_Reaction_Sample,
	step: u64,
	substep: u32,
	state: Joint_Reaction_State,
	maximum_force, maximum_torque: f32,
}

Joint_Break_Event :: struct
{
	constraint: Constraint_Handle,
	type_id: i32,
	lifetime, user_id: u64,
	exceeded: Joint_Break_Metrics,
	reaction: Joint_Reaction,
}

Joint_Break_Watch :: struct
{
	limits: Joint_Break_Limits,
	lifetime: u64,
	type_id: i32,
	exceeded: Joint_Break_Metrics,
	committed, pending: Joint_Reaction,
	score: f64,
}

Joint_Break_Selection :: struct
{
	watch, batch, type_id, bundle: i32, lane: u8
}

Joint_Break_Phase :: enum u8
{
	Idle, Sampling, Commit_Pending, Published
}

Joint_Break_System :: struct
{
	simulation: ^Simulation,
	allocator: mem.Allocator,
	scope: util.Allocation_Scope,
	watches: util.Quick_Dictionary(i32, Joint_Break_Watch),
	selections: util.Buffer(Joint_Break_Selection),
	events: util.Buffer(Joint_Break_Event),
	providers: [CONSTRAINT_TYPE_ID_CAPACITY-FIRST_CALLER_CONSTRAINT_TYPE_ID]Constraint_Reaction_Provider,
	selection_count, event_count: int,
	next_lifetime, prepared_step: u64,
	substep_count: u32,
	phase: Joint_Break_Phase,
	notification_status: Physics_Status,
}

joint_break_key_hash :: proc "contextless" (key:rawptr) -> i32
{
	return (^i32)(key)^;
}

joint_break_key_equal :: proc "contextless" (a, b:rawptr) -> util.Comparison_Status
{
	if (^i32)(a)^ == (^i32)(b)^
	{
		return .Equal;
	}
	return .Different;
}

joint_break_release :: proc(owner:^Joint_Break_System)
{
	simulation: ^Simulation = owner.simulation;
	previous_state: Simulation_State = simulation.state;
	simulation.state = .Stepping;
	defer simulation.state = previous_state;
	pool: ^util.Buffer_Pool = simulation.pool;
	if owner.watches.keys.memory!=nil
	{
		_=util.quick_dictionary_dispose(&owner.watches, pool);
	}
	physics_return_buffer(pool, &owner.selections);
	physics_return_buffer(pool, &owner.events);
	allocator: mem.Allocator;
	scope: util.Allocation_Scope;
	allocator, scope = owner.allocator, owner.scope;
	_=util.allocation_free(owner, size_of(Joint_Break_System), align_of(Joint_Break_System), allocator, scope);
}

joint_break_reserve_storage :: proc(owner:^Joint_Break_System, capacity:int) -> Physics_Status
{
	if capacity<=0
	{
		return .Invalid_Argument;
	}
	if capacity>(1<<util.MAXIMUM_SPAN_SIZE_POWER)/size_of(Joint_Break_Watch)
	{
		return .Capacity_Missing;
	}
	status: Physics_Status = physics_ensure_buffer_capacity(owner.simulation.pool, &owner.events, capacity, owner.event_count);
	if status!=.Ok
	{
		return status;
	}
	status=physics_ensure_buffer_capacity(owner.simulation.pool, &owner.selections, capacity, 0);
	if status!=.Ok
	{
		return status;
	}
	return physics_collection_status(util.quick_dictionary_ensure_capacity(&owner.watches, capacity, owner.simulation.pool));
}

joint_break_enable :: proc(simulation:^Simulation, capacity:int, allocator:mem.Allocator, scope:util.Allocation_Scope) -> Physics_Status
{
	if simulation==nil || simulation.state!=.Ready || simulation.solver.joint_breaks!=nil || capacity<=0
	{
		return .Invalid_Argument;
	}
	if capacity>(1<<util.MAXIMUM_SPAN_SIZE_POWER)/size_of(Joint_Break_Watch)
	{
		return .Capacity_Missing;
	}
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	actual: mem.Allocator = allocator;
	if actual.procedure==nil
	{
		actual=runtime.heap_allocator();
	}
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(Joint_Break_System), align_of(Joint_Break_System), actual);
	if error!=nil || memory==nil
	{
		return .Capacity_Missing;
	}
	owner: ^Joint_Break_System = (^Joint_Break_System)(memory);
	owner^={simulation=simulation, allocator=actual, scope=scope, next_lifetime=1};
	result: util.Collection_Status = util.quick_dictionary_initialize(&owner.watches, capacity, simulation.pool, {hash=joint_break_key_hash, equal=joint_break_key_equal}, table_power_offset=1);
	if result!=.Ok
	{
		joint_break_release(owner);
		return physics_collection_status(result);
	}
	status: Physics_Status = joint_break_reserve_storage(owner, capacity);
	if status!=.Ok
	{
		joint_break_release(owner);
		return status;
	}
	simulation.solver.joint_breaks=owner;
	return .Ok;
}

// authoritative noncontact handle retirement only. record relocation/sleep does
// not retire the lifetime. contact-removal workers never enter this dictionary
joint_break_retire :: proc "contextless" (owner:^Joint_Break_System, handle:Constraint_Handle)
{
	context=runtime.default_context();
	key: i32 = handle.value;
	_=util.quick_dictionary_fast_remove(&owner.watches, &key);
}

joint_break_reset :: proc(owner:^Joint_Break_System)
{
	util.quick_dictionary_clear(&owner.watches);
	owner.selection_count=0;
	owner.event_count=0;
	owner.phase=.Idle;
	owner.notification_status=.Ok;
}

joint_break_provider :: #force_inline proc "contextless" (owner:^Joint_Break_System, type_id:i32) -> Constraint_Reaction_Provider
{
	if type_id<FIRST_CALLER_CONSTRAINT_TYPE_ID
	{
		return {};
	}
	return owner.providers[type_id-FIRST_CALLER_CONSTRAINT_TYPE_ID];
}

joint_break_set :: proc(owner:^Joint_Break_System, handle:Constraint_Handle, limits:Joint_Break_Limits) -> Physics_Status
{
	if owner.simulation.state!=.Ready || owner.phase!=.Idle
	{
		return .Invalid_Argument;
	}
	if (transmute(u32)limits.metrics)&~u32(3)!=0
	{
		return .Invalid_Argument;
	}
	if .Force in limits.metrics && (!(limits.force>=0)||math.is_inf(limits.force, 0))
	{
		return .Invalid_Argument;
	}
	if .Torque in limits.metrics && (!(limits.torque>=0)||math.is_inf(limits.torque, 0))
	{
		return .Invalid_Argument;
	}
	solver: ^Solver = &owner.simulation.solver;
	if handle.value<0 || handle.value>=solver.handle_to_constraint.length
	{
		return .Not_Found;
	}
	location: Constraint_Location = solver.handle_to_constraint.memory[handle.value];
	if location.set_index<0
	{
		return .Not_Found;
	}
	if location.set_index>0
	{
		status: Physics_Status;
		_, status = island_sleeper_resolve_inactive_constraint_index(&owner.simulation.sleeper, handle);
		if status!=.Ok
		{
			return status;
		}
	}
	else
	{
		status: Physics_Status;
		_, status = solver_resolve(solver, handle);
		if status!=.Ok
		{
			return status;
		}
	}
	key: i32 = handle.value;
	index: int = util.quick_dictionary_index_of(&owner.watches, &key);
	if limits.metrics=={}
	{
		if index>=0
		{
			joint_break_retire(owner, handle);
		};
		return .Ok;
	}
	if constraint_builtin_reaction_support(location.type_id)==.Missing &&
	(location.type_id<FIRST_CALLER_CONSTRAINT_TYPE_ID || joint_break_provider(owner, location.type_id).read==nil)
	{
		return .Invalid_Argument;
	}
	if index>=0
	{
		owner.watches.values.memory[index].limits=limits;
		return .Ok;
	}
	if owner.next_lifetime==max(u64)
	{
		return .Capacity_Missing;
	}
	owner.simulation.state = .Stepping;
	defer owner.simulation.state = .Ready;
	capacity: int = owner.watches.count+1;
	if capacity>int(owner.watches.keys.length)
	{
		capacity=max(capacity, int(owner.watches.keys.length)*2);
	}
	status: Physics_Status = joint_break_reserve_storage(owner, capacity);
	if status!=.Ok
	{
		return status;
	}
	result: util.Collection_Status = util.quick_dictionary_add_unsafely(&owner.watches, key, Joint_Break_Watch{limits=limits, lifetime=owner.next_lifetime, type_id=location.type_id});
	if result!=.Ok
	{
		return physics_collection_status(result);
	}
	owner.next_lifetime+=1;
	return .Ok;
}

joint_break_selection_compare :: proc(a, b, user:rawptr) -> slice.Ordering
{
	_=user;
	l: ^Joint_Break_Selection;
	r: ^Joint_Break_Selection;
	l, r = (^Joint_Break_Selection)(a), (^Joint_Break_Selection)(b);
	if l.type_id<r.type_id
	{
		return .Less;
	};
	if l.type_id>r.type_id
	{
		return .Greater;
	}
	if l.batch<r.batch
	{
		return .Less;
	};
	if l.batch>r.batch
	{
		return .Greater;
	}
	if l.bundle<r.bundle
	{
		return .Less;
	};
	if l.bundle>r.bundle
	{
		return .Greater;
	}
	if l.lane<r.lane
	{
		return .Less;
	};
	if l.lane>r.lane
	{
		return .Greater;
	};
	return .Equal;
}

// rebuild only the watch selection at the solve boundary: sleeping/collision
// stages may have changed locations since the previous solve. reuse across its
// substeps. never retain body/prestep pointers across an owning phase
joint_break_prepare_solve :: proc(owner:^Joint_Break_System) -> Physics_Status
{
	owner.selection_count=0;
	solver: ^Solver = &owner.simulation.solver;
	for index in 0..<owner.watches.count
	{
		location: Constraint_Location;
		status: Physics_Status;
		location, status = solver_resolve(solver, {owner.watches.keys.memory[index]});
		if status==.Not_Found
		{
			continue;
		};
		if status!=.Ok
		{
			return status;
		}
		owner.selections.memory[owner.selection_count]={watch=i32(index), batch=location.batch_index, type_id=location.type_id,
			bundle=location.index_in_type_batch/util.PRODUCTION_LANE_COUNT, lane=u8(location.index_in_type_batch%util.PRODUCTION_LANE_COUNT)};
		owner.selection_count+=1;
	}
	slice.sort_by_generic_cmp(owner.selections.memory[:owner.selection_count], joint_break_selection_compare, nil);
	return .Ok;
}

joint_break_capture_substep :: #force_no_inline proc "contextless" (owner:^Joint_Break_System, duration:f32) -> Physics_Status
{
	if owner.substep_count==max(u32)
	{
		return .Capacity_Missing;
	}
	first: int = 0;
	for first<owner.selection_count
	{
		anchor: ^Joint_Break_Selection = &owner.selections.memory[first];
		stop: int = first;
		mask:u8;
		for stop<owner.selection_count
		{
			entry: ^Joint_Break_Selection = &owner.selections.memory[stop];
			if entry.type_id!=anchor.type_id || entry.batch!=anchor.batch || entry.bundle!=anchor.bundle
			{
				break;
			}
			mask|=u8(1)<<u32(entry.lane);
			stop+=1;
		}
		samples:[util.PRODUCTION_LANE_COUNT]Constraint_Reaction_Sample;
		status: Physics_Status = constraint_capture_reaction_bundle(&owner.simulation.solver, {owner.watches.keys.memory[anchor.watch]}, mask, duration, joint_break_provider(owner, anchor.type_id), &samples);
		if status!=.Ok
		{
			return status;
		}
		for index in first..<stop
		{
			entry: ^Joint_Break_Selection = &owner.selections.memory[index];
			watch: ^Joint_Break_Watch = &owner.watches.values.memory[entry.watch];
			sample: ^Constraint_Reaction_Sample = &samples[entry.lane];
			score:f64;
			if .Force in watch.limits.metrics
			{
				score=f64(sample.maximum_force)/(f64(watch.limits.force) if watch.limits.force>0 else 1);
				if sample.maximum_force>watch.limits.force
				{
					watch.exceeded+={.Force};
				}
			}
			if .Torque in watch.limits.metrics
			{
				score=max(score, f64(sample.maximum_torque)/(f64(watch.limits.torque) if watch.limits.torque>0 else 1));
				if sample.maximum_torque>watch.limits.torque
				{
					watch.exceeded+={.Torque};
				}
			}
			if watch.pending.state==.Unsolved || score>watch.score
			{
				watch.pending.sample=sample^;
				watch.pending.substep=owner.substep_count;
				watch.score=score;
			}
			watch.pending.state=.Solved;
			watch.pending.maximum_force=max(watch.pending.maximum_force, sample.maximum_force);
			watch.pending.maximum_torque=max(watch.pending.maximum_torque, sample.maximum_torque);
		}
		first=stop;
	}
	owner.substep_count+=1;
	return .Ok;
}

joint_break_event_compare :: proc(a, b, user:rawptr) -> slice.Ordering
{
	_=user;
	l: ^Joint_Break_Event;
	r: ^Joint_Break_Event;
	l, r = (^Joint_Break_Event)(a), (^Joint_Break_Event)(b);
	if l.lifetime<r.lifetime
	{
		return .Less;
	};
	if l.lifetime>r.lifetime
	{
		return .Greater;
	};
	return .Equal;
}

// custom steps may sleep sampled joints. finish the potentially allocating
// awakening phase for every crossed joint before removing any. exhaustion keeps
// publication pending, blocks another step, and is retried through drain
joint_break_finish :: #force_no_inline proc(owner:^Joint_Break_System) -> Physics_Status
{
	if owner.phase!=.Commit_Pending
	{
		return .Ok;
	}
	simulation: ^Simulation = owner.simulation;
	if simulation.state != .Ready
	{
		return .Invalid_Argument;
	}
	simulation.state = .Stepping;
	defer simulation.state = .Ready;
	for index in 0..<owner.watches.count
	{
		watch: ^Joint_Break_Watch = &owner.watches.values.memory[index];
		if watch.exceeded=={}
		{
			continue;
		}
		handle: i32 = owner.watches.keys.memory[index];
		location: Constraint_Location = simulation.solver.handle_to_constraint.memory[handle];
		if location.set_index>0
		{
			status: Physics_Status = island_awakener_awaken_set(&simulation.awakener, int(location.set_index));
			if status!=.Ok
			{
				owner.notification_status=status;
				return status;
			}
		}
	}
	owner.event_count=0;
	for index in 0..<owner.watches.count
	{
		watch: ^Joint_Break_Watch = &owner.watches.values.memory[index];
		watch.pending.step=simulation.step_index;
		watch.committed=watch.pending;
		if watch.exceeded!={}
		{
			owner.events.memory[owner.event_count]={constraint={owner.watches.keys.memory[index]}, type_id=watch.type_id,
				lifetime=watch.lifetime, user_id=watch.limits.user_id, exceeded=watch.exceeded, reaction=watch.pending};
			owner.event_count+=1;
		}
	}
	slice.sort_by_generic_cmp(owner.events.memory[:owner.event_count], joint_break_event_compare, nil);
	for index in 0..<owner.event_count
	{
		status: Physics_Status = solver_remove(&simulation.solver, owner.events.memory[index].constraint);
		if status!=.Ok
		{
			owner.notification_status=status;
			return status;
		}
	}
	owner.phase=.Published if owner.event_count>0 else .Idle;
	owner.notification_status=.Ok;
	return .Ok;
}

joint_break_step_prepare :: #force_no_inline proc(owner:^Joint_Break_System) -> Physics_Status
{
	if owner.phase!=.Idle
	{
		return .Invalid_Argument;
	}
	owner.prepared_step=owner.simulation.step_index;
	owner.substep_count=0;
	owner.selection_count=0;
	for index in 0..<owner.watches.count
	{
		watch: ^Joint_Break_Watch = &owner.watches.values.memory[index];
		watch.pending={};
		watch.score=0;
		watch.exceeded={};
	}
	owner.phase=.Sampling;
	return .Ok;
}

joint_break_step_complete :: #force_no_inline proc(owner:^Joint_Break_System, status:Physics_Status)
{
	if status!=.Ok
	{
		owner.phase=.Idle;
		owner.notification_status=.Ok;
		owner.selection_count=0;
		return;
	}
	owner.phase=.Commit_Pending;
	_=joint_break_finish(owner);
}

joint_break_drain :: proc(owner:^Joint_Break_System, output:[]Joint_Break_Event) -> (written, required:int, status:Physics_Status)
{
	status=joint_break_finish(owner);
	if status!=.Ok
	{
		return 0, 0, status;
	}
	required=owner.event_count;
	if len(output)<required
	{
		return 0, required, .Capacity_Missing;
	}
	copy(output, owner.events.memory[:required]);
	written=required;
	owner.event_count=0;
	owner.phase=.Idle;
	return written, required, .Ok;
}
