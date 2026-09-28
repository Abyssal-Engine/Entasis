package entasis

import "base:intrinsics"
import "core:mem"
import util "entasis:entasis_utilities"
import physics "entasis:entasis_physics"

// contextual task callbacks carry explicit registration data. shape payloads,
// native callback signatures and eight-lane collision bundles remain unchanged
Contextual_Collision_Test_Proc      :: physics.Contextual_Collision_Test_Proc;
Contextual_Collision_Wide_Test_Proc :: physics.Contextual_Collision_Wide_Test_Proc;
Contextual_Sweep_Test_Proc          :: physics.Contextual_Sweep_Test_Proc;
Contextual_Sweep_Child_Test_Proc    :: physics.Contextual_Sweep_Child_Test_Proc;
Contextual_Collision_Task_Registration :: struct
{
	shape_type_a, shape_type_b: Shape_Type_ID,
	batch_size: int,
	pair_type: Collision_Task_Pair_Type,
	user_context: rawptr,
	test: Contextual_Collision_Test_Proc,
	wide_test: Contextual_Collision_Wide_Test_Proc,
}

Contextual_Sweep_Task_Registration :: struct
{
	shape_type_a, shape_type_b: Shape_Type_ID,
	user_context: rawptr,
	test: Contextual_Sweep_Test_Proc,
	child_test: Contextual_Sweep_Child_Test_Proc,
}

@(private)
contextual_task_record :: struct
{
	next: ^contextual_task_record,
	using binding: struct #raw_union
	{
		collision: physics.Contextual_Collision_Task_Binding,
		sweep: physics.Contextual_Sweep_Task_Binding,
	},
}

@(private)
contextual_task_record_create :: proc (data: ^world_data) -> (^contextual_task_record, Status)
{
	memory: rawptr;
	error: mem.Allocator_Error;
	memory, error = mem.alloc(size_of(contextual_task_record), align_of(contextual_task_record), data.allocator);
	if error != .None || memory == nil
	{
		return nil, .Capacity_Missing;
	}
	record: ^contextual_task_record = (^contextual_task_record)(memory);
	intrinsics.mem_zero(record, size_of(contextual_task_record));
	return record, .Ok;
}

@(private)
contextual_task_records_release :: proc (data: ^world_data) -> Status
{
	result: Status = Status.Ok;
	record: ^contextual_task_record = data.contextual_tasks;
	data.contextual_tasks = nil;
	for record != nil
	{
		next: ^contextual_task_record = record.next;
		if util.allocation_free(record, size_of(contextual_task_record), align_of(contextual_task_record), data.allocator, data.allocation_scope) != .None
		{
			result = .Invalid_Argument;
		}
		record = next;
	}
	return result;
}

@(private)
contextual_task_world_ready :: proc "contextless" (
	world: ^World, type_a, type_b: Shape_Type_ID,
) -> (^world_data, ^physics.Simulation, Status)
{
	data: ^world_data;
	simulation: ^physics.Simulation;
	status: Status;
	data, simulation, status = extension_world_ready(world);
	if status != .Ok
	{
		return nil, nil, status;
	}
	if simulation.step_index != 0
	{
		return nil, nil, .Invalid_Argument;
	}
	shapes: ^physics.Shape_Registry = physics.simulation_shape_registry(simulation);
	if shapes == nil || int(type_a) < 0 || int(type_b) < 0 ||
	int(type_a) >= shapes.registered_type_count || int(type_b) >= shapes.registered_type_count
	{
		return nil, nil, .Invalid_Description;
	}
	return data, simulation, .Ok;
}

// collision_task_register_contextual copies the descriptor and callback table.
// user_context remains caller-owned until world destruction. callback invocation
// is once per scalar pair or existing eight-lane bundle, never once per wide lane.
// register before the first step. existing routes, including built-ins, cannot
// be replaced. independent worlds may bind the same IDs to different contexts
collision_task_register_contextual :: proc (
	world: ^World, registration: Contextual_Collision_Task_Registration,
) -> (i32, Status)
{
	data: ^world_data;
	simulation: ^physics.Simulation;
	status: Status;
	data, simulation, status = contextual_task_world_ready(world, registration.shape_type_a, registration.shape_type_b);
	if status != .Ok
	{
		return -1, status;
	}
	if registration.batch_size <= 0 || registration.batch_size > physics.MAXIMUM_COLLISION_TASK_BATCH_SIZE ||
	registration.pair_type > .Bounds_Tested
	{
		return -1, .Invalid_Description;
	}
	binding: physics.Contextual_Collision_Task_Binding = physics.Contextual_Collision_Task_Binding{
		user_context=registration.user_context, test=registration.test, wide_test=registration.wide_test,
	};
	task: physics.Collision_Task = physics.Collision_Task{
		shape_type_a=i16(registration.shape_type_a), shape_type_b=i16(registration.shape_type_b),
		batch_size=i16(registration.batch_size), pair_type=registration.pair_type,
		kind=.Convex, capabilities={.Convex_Result, .Wide_Result}, dispatch=.Contextual, contextual=&binding,
	};
	status = physics.collision_task_registry_validate_registration(&simulation.collision_tasks, task);
	if status != .Ok
	{
		return -1, status;
	}
	record: ^contextual_task_record;
	allocation_status: Status;
	record, allocation_status = contextual_task_record_create(data);
	if allocation_status != .Ok
	{
		return -1, allocation_status;
	}
	record.collision = binding;
	task.contextual = &record.collision;
	task_id: i32;
	register_status: physics.Physics_Status;
	task_id, register_status = physics.collision_task_registry_register(&simulation.collision_tasks, task);
	if register_status != .Ok
	{
		_ = util.allocation_free(record, size_of(contextual_task_record), align_of(contextual_task_record), data.allocator, data.allocation_scope);
		return -1, register_status;
	}
	record.next = data.contextual_tasks;
	data.contextual_tasks = record;
	return task_id, .Ok;
}

// sweep_task_register_contextual copies both top-level and child callbacks into
// world-owned stable storage. the query filter and its context are passed
// separately from registration user_context. flipped routes remap child indices
// for both filter invocations and reported hits. data is borrowed only during a
// callback. use caller synchronization for queries and world lifetime
sweep_task_register_contextual :: proc (
	world: ^World, registration: Contextual_Sweep_Task_Registration,
) -> (i32, Status)
{
	data: ^world_data;
	simulation: ^physics.Simulation;
	status: Status;
	data, simulation, status = contextual_task_world_ready(world, registration.shape_type_a, registration.shape_type_b);
	if status != .Ok
	{
		return -1, status;
	}
	binding: physics.Contextual_Sweep_Task_Binding = physics.Contextual_Sweep_Task_Binding{
		user_context=registration.user_context, test=registration.test, child_test=registration.child_test,
	};
	status = physics.sweep_task_registry_validate_registration(&simulation.sweep_tasks,
		int(registration.shape_type_a), int(registration.shape_type_b), nil, nil, .Contextual, &binding);
	if status != .Ok
	{
		return -1, status;
	}
	record: ^contextual_task_record;
	allocation_status: Status;
	record, allocation_status = contextual_task_record_create(data);
	if allocation_status != .Ok
	{
		return -1, allocation_status;
	}
	record.sweep = binding;
	task_id: i32;
	register_status: physics.Physics_Status;
	task_id, register_status = physics.sweep_task_registry_register_contextual(&simulation.sweep_tasks,
		int(registration.shape_type_a), int(registration.shape_type_b), &record.sweep);
	if register_status != .Ok
	{
		_ = util.allocation_free(record, size_of(contextual_task_record), align_of(contextual_task_record), data.allocator, data.allocation_scope);
		return -1, register_status;
	}
	record.next = data.contextual_tasks;
	data.contextual_tasks = record;
	return task_id, .Ok;
}
