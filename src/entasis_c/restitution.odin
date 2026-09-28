package entasis_c

import "base:runtime"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"

abi_restitution_binding :: struct
{
	combine: Entasis_Restitution_Combine_Proc,
	user_context: rawptr,
}

// the native configuration already owns the stable callback context. reuse it
// instead of enlarging every C world record for an optional callback binding
abi_restitution_binding_get :: proc "contextless" (resource: ^abi_world_resource) -> ^abi_restitution_binding
{
	simulation, status := entasis.world_borrow_simulation(&resource.world);
	if status != .Ok || simulation.restitution == nil
	{
		return nil;
	}
	configuration := &simulation.restitution.configuration;
	if configuration.combine != abi_restitution_combine
	{
		return nil;
	}
	return (^abi_restitution_binding)(configuration.user_context);
}

abi_restitution_binding_release :: proc "contextless" (resource: ^abi_world_resource, binding: ^abi_restitution_binding)
{
	if binding != nil
	{
		abi_resource_free(binding, size_of(abi_restitution_binding), align_of(abi_restitution_binding), &resource.allocator);
	}
}

abi_restitution_combine :: proc "contextless" (
	user_context: rawptr, pair: physics.Collidable_Pair,
	a, b: entasis.Restitution_Settings,
) -> entasis.Restitution_Settings
{
	binding := (^abi_restitution_binding)(user_context);
	settings_a := Entasis_Restitution_Settings{coefficient=a.coefficient, threshold=a.threshold};
	settings_b := Entasis_Restitution_Settings{coefficient=b.coefficient, threshold=b.threshold};
	output := Entasis_Restitution_Settings{coefficient=max(a.coefficient, b.coefficient), threshold=max(a.threshold, b.threshold)};
	binding.combine(binding.user_context, {packed=pair.a.packed}, {packed=pair.b.packed}, &settings_a, &settings_b, &output);
	return {coefficient=output.coefficient, threshold=output.threshold};
}

abi_restitution_configuration_default :: proc "contextless" () -> Entasis_Restitution_Configuration
{
	value := entasis.restitution_configuration_default();
	return {struct_size=u32(size_of(Entasis_Restitution_Configuration)), struct_version=ENTASIS_STRUCT_VERSION,
		fallback={coefficient=value.fallback.coefficient, threshold=value.fallback.threshold},
		collidable_capacity=value.collidable_capacity, pair_capacity=value.pair_capacity};
}

abi_world_enable_restitution :: proc "contextless" (
	world: ^Entasis_World, configuration: ^Entasis_Restitution_Configuration, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	// snapshot before allocating or entering caller code, including aliased output
	selected := abi_restitution_configuration_default();
	if configuration != nil
	{
		selected = configuration^;
	}
	if selected.struct_size != size_of(Entasis_Restitution_Configuration) || selected.struct_version != ENTASIS_STRUCT_VERSION
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	value := entasis.Restitution_Configuration{fallback={selected.fallback.coefficient, selected.fallback.threshold},
		collidable_capacity=selected.collidable_capacity, pair_capacity=selected.pair_capacity};
	validation := physics.restitution_configuration_validate(value);
	if validation != .Ok
	{
		return abi_status_finish(validation, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	binding: ^abi_restitution_binding;
	if selected.combine != nil
	{
		memory, _, status := abi_resource_allocate_owned(size_of(abi_restitution_binding), align_of(abi_restitution_binding), &resource.allocator);
		if status != .Ok
		{
			return abi_status_finish(status, diagnostic, .None);
		}
		binding = (^abi_restitution_binding)(memory);
		binding^ = {combine=selected.combine, user_context=selected.user_context};
		value.combine = abi_restitution_combine;
		value.user_context = binding;
	}
	context = runtime.default_context();
	status := entasis.world_enable_restitution(&resource.world, value);
	if status != .Ok
	{
		if binding != nil
		{
			abi_resource_free(binding, size_of(abi_restitution_binding), align_of(abi_restitution_binding), &resource.allocator);
		}
		return abi_status_finish(status, diagnostic, .None);
	}
	return abi_status_finish(.Ok, diagnostic, .None);
}

abi_world_disable_restitution :: proc "contextless" (world: ^Entasis_World, diagnostic: ^Entasis_Diagnostic) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	binding := abi_restitution_binding_get(resource);
	status := entasis.world_disable_restitution(&resource.world);
	if status == .Ok
	{
		abi_restitution_binding_release(resource, binding);
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_restitution_reserve :: proc "contextless" (
	world: ^Entasis_World, collidable_capacity, pair_capacity: i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.restitution_reserve(&resource.world, collidable_capacity, pair_capacity), diagnostic, .None);
}

abi_restitution_set :: proc "contextless" (
	world: ^Entasis_World, reference: Entasis_Collidable_Reference, settings: ^Entasis_Restitution_Settings,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if settings == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	value := settings^;
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.restitution_set(&resource.world, {reference.packed}, {value.coefficient, value.threshold}), diagnostic, .None);
}

abi_restitution_get :: proc "contextless" (
	world: ^Entasis_World, reference: Entasis_Collidable_Reference, output: ^Entasis_Restitution_Settings,
	diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if output == nil
	{
		return abi_status_finish(.Invalid_Argument, diagnostic, .None);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None, .Read_Only);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	value, status := entasis.restitution_get(&resource.world, {reference.packed});
	if status == .Ok
	{
		output^ = {value.coefficient, value.threshold};
	}
	return abi_status_finish(status, diagnostic, .None);
}

abi_restitution_remove :: proc "contextless" (
	world: ^Entasis_World, reference: Entasis_Collidable_Reference, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .None);
	defer abi_world_release(resource);
	if resource == nil
	{
		return ready;
	}
	context = runtime.default_context();
	return abi_status_finish(entasis.restitution_remove(&resource.world, {reference.packed}), diagnostic, .None);
}
