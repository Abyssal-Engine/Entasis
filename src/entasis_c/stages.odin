package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

abi_world_extensions_default :: proc "contextless" () -> Entasis_World_Extensions
{
	return {struct_size=u32(size_of(Entasis_World_Extensions)), struct_version=ENTASIS_STRUCT_VERSION};
}

abi_world_extensions_validate :: proc "contextless" (extensions: ^Entasis_World_Extensions) -> entasis.Status
{
	if extensions == nil
	{
		return .Ok;
	}
	if extensions.struct_size < u32(size_of(Entasis_World_Extensions)) || extensions.struct_version != ENTASIS_STRUCT_VERSION
	{
		return .Invalid_Description;
	}
	if extensions.allocation_scope > 1
	{
		return .Invalid_Description;
	}
	stages := extensions.timestep_callbacks;
	if stages != nil && (stages.struct_size < u32(size_of(Entasis_Timestep_Callbacks)) || stages.struct_version != ENTASIS_STRUCT_VERSION || stages.stage_completed == nil)
	{
		return .Invalid_Description;
	}
	stepper := extensions.timestepper;
	if stepper != nil && (stepper.struct_size < u32(size_of(Entasis_Timestepper)) || stepper.struct_version != ENTASIS_STRUCT_VERSION || stepper.step == nil)
	{
		return .Invalid_Description;
	}
	scheduler := extensions.substep_scheduler;
	if scheduler != nil && (scheduler.struct_size < u32(size_of(Entasis_Substep_Scheduler)) || scheduler.struct_version != ENTASIS_STRUCT_VERSION || scheduler.iterations == nil)
	{
		return .Invalid_Description;
	}
	if abi_velocity_callbacks_validate(extensions.velocity_callbacks) != .Ok || abi_contact_callbacks_validate(extensions.contact_callbacks) != .Ok
	{
		return .Invalid_Description;
	}
	return .Ok;
}

abi_world_extensions_bind :: proc "contextless" (
	resource: ^abi_world_resource, description: ^entasis.World_Description, extensions: ^Entasis_World_Extensions,
)
{
	if extensions == nil
	{
		return;
	}
	abi_extension_callbacks_bind(resource, description, extensions);
	if extensions.timestep_callbacks != nil
	{
		resource.stage_callbacks = extensions.timestep_callbacks^;
		description.timestep_callbacks = {stage_completed=abi_stage_completed_bridge, user_context=resource};
	}
	if extensions.timestepper != nil
	{
		resource.custom_timestepper = extensions.timestepper^;
		description.timestepper = {step=abi_timestep_bridge, user_context=resource};
	}
	if extensions.substep_scheduler != nil
	{
		resource.substep_scheduler = extensions.substep_scheduler^;
		description.solve.velocity_iteration_scheduler = abi_substep_scheduler_bridge;
		description.solve.scheduler_context = resource;
	}
}

abi_world_init_extended :: proc "contextless" (
	world: ^Entasis_World, description: ^Entasis_World_Description,
	extensions: ^Entasis_World_Extensions, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	return abi_world_init_internal(world, description, nil, diagnostic, extensions);
}

abi_world_init_with_pool_extended :: proc "contextless" (
	world: ^Entasis_World, description: ^Entasis_World_Description, pool: ^Entasis_Buffer_Pool,
	extensions: ^Entasis_World_Extensions, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if pool == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .World_Initialize);
	}
	return abi_world_init_internal(world, description, pool, diagnostic, extensions);
}

abi_stage_completed_bridge :: proc "contextless" (
	user_context: rawptr, stage: physics.Timestep_Completion_Stage, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> entasis.Status
{
	_ = dispatcher;
	resource := (^abi_world_resource)(user_context);
	return entasis.Status(resource.stage_callbacks.stage_completed(
			resource.stage_callbacks.user_context, {opaque=&resource.header}, Entasis_Timestep_Completion_Stage(stage), dt));
}

abi_substep_scheduler_bridge :: proc "contextless" (user_context: rawptr, substep_index: int) -> i32
{
	resource := (^abi_world_resource)(user_context);
	return resource.substep_scheduler.iterations(resource.substep_scheduler.user_context, i32(substep_index));
}

abi_timestep_bridge :: proc "contextless" (
	user_context: rawptr, simulation: ^physics.Simulation, dt: f32,
	dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> entasis.Status
{
	resource := (^abi_world_resource)(user_context);
	if resource == nil || simulation == nil || resource.step_scope_live == .Live || resource.step_scope_generation == max(u64)
	{
		return .Invalid_Argument;
	}
	resource.step_scope_generation += 1;
	resource.step_scope_live = .Live;
	resource.step_scope_executing = .Idle;
	resource.step_scope_simulation = simulation;
	resource.step_scope_dispatcher = dispatcher;
	defer
	{
		resource.step_scope_live = .Inactive;
		resource.step_scope_executing = .Idle;
		resource.step_scope_simulation = nil;
		resource.step_scope_dispatcher = nil;
	}
	scope := Entasis_Step_Scope{world={opaque=&resource.header}, generation=resource.step_scope_generation};
	status := entasis.Status(resource.custom_timestepper.step(resource.custom_timestepper.user_context, scope, dt));
	if status == .Ok
	{
		// only this selected C adapter accounts a completed custom step. individual
		// scope stages cannot invoke the default timestep or advance this counter
		physics.simulation_profiler_start(&simulation.profiler, .Cleanup);
		simulation.step_index += 1;
		physics.simulation_profiler_end(&simulation.profiler, .Cleanup);
	}
	return status;
}

abi_step_scope_begin :: proc "contextless" (scope: Entasis_Step_Scope) -> ^abi_world_resource
{
	world := scope.world;
	resource := abi_world_get(&world);
	if resource == nil || resource.header.access != .Exclusive || resource.step_scope_live != .Live ||
	resource.step_scope_generation != scope.generation || resource.step_scope_executing == .Executing || resource.step_scope_simulation == nil
	{
		return nil;
	}
	resource.step_scope_executing = .Executing;
	return resource;
}

abi_step_scope_report_completion :: proc "contextless" (
	scope: Entasis_Step_Scope, stage: Entasis_Timestep_Completion_Stage, dt: f32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if stage > 3 || !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	stages := [?]physics.Simulation_Stage{.Slept_Callback, .Before_Collision_Callback, .Collisions_Detected_Callback, .Constraints_Solved_Callback};
	return abi_status_finish(physics.timestep_completion_callback(
			resource.step_scope_simulation, physics.Timestep_Completion_Stage(stage), stages[stage], dt, resource.step_scope_dispatcher), diagnostic, .None);
}

abi_world_stage_sleep :: proc "contextless" (world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_stage_sleep(&resource.world), diagnostic, .None);
}

abi_step_scope_sleep :: proc "contextless" (scope: Entasis_Step_Scope, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	context = runtime.default_context();
	return abi_status_finish(physics.simulation_sleep(resource.step_scope_simulation, resource.step_scope_dispatcher), diagnostic, .None);
}

abi_world_stage_predict_bounds :: proc "contextless" (world: ^Entasis_World, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_stage_predict_bounds(&resource.world, dt), diagnostic, .None);
}

abi_step_scope_predict_bounds :: proc "contextless" (scope: Entasis_Step_Scope, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	context = runtime.default_context();
	return abi_status_finish(physics.simulation_predict_bounding_boxes(resource.step_scope_simulation, dt, resource.step_scope_dispatcher), diagnostic, .None);
}

abi_world_stage_collision_detection :: proc "contextless" (world: ^Entasis_World, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_stage_collision_detection(&resource.world, dt), diagnostic, .None);
}

abi_step_scope_collision_detection :: proc "contextless" (scope: Entasis_Step_Scope, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	context = runtime.default_context();
	return abi_status_finish(physics.simulation_collision_detection(resource.step_scope_simulation, dt, resource.step_scope_dispatcher), diagnostic, .None);
}

abi_world_stage_solve :: proc "contextless" (world: ^Entasis_World, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_stage_solve(&resource.world, dt), diagnostic, .None);
}

abi_step_scope_solve :: proc "contextless" (scope: Entasis_Step_Scope, dt: f32, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	if !(dt > 0)
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	context = runtime.default_context();
	return abi_status_finish(physics.simulation_solve(resource.step_scope_simulation, dt, resource.step_scope_dispatcher), diagnostic, .None);
}

abi_world_stage_optimize :: proc "contextless" (world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.world_stage_optimize(&resource.world), diagnostic, .None);
}

abi_step_scope_optimize :: proc "contextless" (scope: Entasis_Step_Scope, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	resource := abi_step_scope_begin(scope);
	if resource == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	defer resource.step_scope_executing = .Idle;
	context = runtime.default_context();
	return abi_status_finish(physics.simulation_incrementally_optimize_data_structures(resource.step_scope_simulation, resource.step_scope_dispatcher), diagnostic, .None);
}
