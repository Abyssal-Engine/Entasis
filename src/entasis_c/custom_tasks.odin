package entasis_c

import "base:runtime"
import "core:math"
import entasis "entasis:entasis"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// cold, per-registration ownership. native kernels never reference this record.
// tables survive clear/reuse and are released after native world destruction
abi_custom_task_binding :: struct
{
	next: ^abi_custom_task_binding,
	collision_tasks: ^physics.Collision_Task_Registry,
	task_id: i32,
	using description: struct #raw_union
	{
		collision: Entasis_Collision_Task_Registration,
		sweep: Entasis_Sweep_Task_Registration,
	},
}

// each invocation owns its stack scope, including any current query filter.
// no per-world current-callback pointer, atomics, lock or TLS routing is needed
abi_task_access_scope :: struct
{
	shape_access: abi_shape_access_scope,
	shapes: ^physics.Shape_Registry,
	collision_tasks: ^physics.Collision_Task_Registry,
	filter: physics.Collision_Child_Filter_Proc,
	filter_context: rawptr,
	blocked_collision_id: i32,
	executing: abi_Execution_State,
}

abi_task_scope_get :: #force_inline proc "contextless" (access: Entasis_Task_Access) -> ^abi_task_access_scope
{
	if access.opaque == nil
	{
		return nil;
	}
	scope := (^abi_task_access_scope)(access.opaque);
	if scope.shapes == nil || scope.executing == .Executing
	{
		return nil;
	}
	return scope;
}

abi_task_scope :: #force_inline proc "contextless" (
	binding: ^abi_custom_task_binding, shapes: ^physics.Shape_Registry, blocked_collision_id: i32 = -1,
	filter: physics.Collision_Child_Filter_Proc = nil, filter_context: rawptr = nil,
) -> abi_task_access_scope
{
	return {shape_access={native=entasis.Shape_Access(shapes)}, shapes=shapes,
		collision_tasks=binding.collision_tasks, blocked_collision_id=blocked_collision_id,
		filter=filter, filter_context=filter_context};
}

abi_task_scalar_status :: #force_inline proc "contextless" (value: f32) -> entasis.Status
{
	return .Ok if value >= -math.F32_MAX && value <= math.F32_MAX else .Invalid_Argument;
}

abi_task_vector_status :: #force_inline proc "contextless" (v: Entasis_Vector3) -> entasis.Status
{
	return .Ok if (abi_task_scalar_status(v.x) == .Ok) && (abi_task_scalar_status(v.y) == .Ok) && (abi_task_scalar_status(v.z) == .Ok) else .Invalid_Argument;
}

abi_task_manifold_validate :: proc "contextless" (result: ^Entasis_Convex_Contact_Manifold) -> entasis.Status
{
	if result.count < 0 || result.count > 4 || (abi_task_vector_status(result.offset_b) != .Ok)
	{
		return .Invalid_Description;
	}
	if result.count > 0 && (abi_task_vector_status(result.normal) != .Ok)
	{
		return .Invalid_Description;
	}
	for i in 0..<int(result.count)
	{
		if (abi_task_scalar_status(result.contacts[i].depth) != .Ok) || (abi_task_vector_status(result.contacts[i].offset) != .Ok)
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

abi_collision_task_bridge :: proc "contextless" (
	user_context, shape_a, shape_b: rawptr, pose_a, pose_b: physics.Rigid_Pose,
	margin: f32, shapes: ^physics.Shape_Registry,
) -> (physics.Convex_Contact_Manifold, entasis.Status)
{
	binding := (^abi_custom_task_binding)(user_context);
	scope := abi_task_scope(binding, shapes, binding.task_id);
	input := Entasis_Collision_Task_Input{shape_a=shape_a, shape_b=shape_b,
		type_a=binding.collision.shape_type_a, type_b=binding.collision.shape_type_b,
		pose_a=abi_pose_from_core(pose_a), pose_b=abi_pose_from_core(pose_b), speculative_margin=margin};
	output: Entasis_Convex_Contact_Manifold;
	status := abi_custom_callback_status(binding.collision.test(binding.collision.user_context, &input, {opaque=&scope}, &output));
	if status != .Ok
	{
		return {}, status;
	}
	status = abi_task_manifold_validate(&output);
	if status != .Ok
	{
		return {}, status;
	}
	return transmute(physics.Convex_Contact_Manifold)output, .Ok;
}

// native mesh gathering packs triangle geometry by value. its temporary raw
// triangle pointers do not survive gathering, so triangle-bearing C routes use
// a registration-selected adapter with callback-local scalar storage instead
abi_collision_task_wide_input :: #force_inline proc "contextless" (
	binding: ^abi_custom_task_binding, bundle: ^physics.Collision_Convex_Wide_Bundle,
) -> Entasis_Collision_Task_Wide_Input
{
	return Entasis_Collision_Task_Wide_Input{
		shape_a=raw_data(bundle.shape_a[:]), shape_b=raw_data(bundle.shape_b[:]),
		offset_b=(^Entasis_Vector3_Wide)(&bundle.offset_b),
		orientation_a=(^Entasis_Quaternion_Wide)(&bundle.orientation_a),
		orientation_b=(^Entasis_Quaternion_Wide)(&bundle.orientation_b),
		speculative_margin=(^Entasis_F32x8)(&bundle.speculative_margin), count=u32(bundle.count),
		type_a=binding.collision.shape_type_a, type_b=binding.collision.shape_type_b,
	};
}

abi_collision_task_wide_invoke :: #force_inline proc "contextless" (
	binding: ^abi_custom_task_binding, input: ^Entasis_Collision_Task_Wide_Input,
	shapes: ^physics.Shape_Registry, result: ^physics.Collision_Wide_Manifold_Result,
) -> entasis.Status
{
	scope := abi_task_scope(binding, shapes, binding.task_id);
	output: Entasis_Convex_Manifold_Wide;
	status := abi_custom_callback_status(binding.collision.wide_test(binding.collision.user_context, input, {opaque=&scope}, &output));
	if status != .Ok
	{
		return status;
	}
	// validate bounded mask values before the native manifold writer sees them.
	// this is output conversion, not one foreign callback per lane
	for &contact in output.contacts
	{
		for lane in 0..<8
		{
			exists := contact.exists.lanes[lane];
			if exists == 0
			{
				continue;
			}
			if exists != -1 || lane >= int(input.count) ||
			(abi_task_scalar_status(contact.depth.lanes[lane]) != .Ok) ||
			(abi_task_vector_status({contact.offset_a.x.lanes[lane], contact.offset_a.y.lanes[lane], contact.offset_a.z.lanes[lane]}) != .Ok) ||
			(abi_task_vector_status({output.normal.x.lanes[lane], output.normal.y.lanes[lane], output.normal.z.lanes[lane]}) != .Ok)
			{
				return .Invalid_Description;
			}
		}
	}
	result.kind = .Four_Contact;
	result.four.normal = transmute(util.Vector3_Wide)output.normal;
	result.four.offset_a_0 = transmute(util.Vector3_Wide)output.contacts[0].offset_a;
	result.four.offset_a_1 = transmute(util.Vector3_Wide)output.contacts[1].offset_a;
	result.four.offset_a_2 = transmute(util.Vector3_Wide)output.contacts[2].offset_a;
	result.four.offset_a_3 = transmute(util.Vector3_Wide)output.contacts[3].offset_a;
	result.four.depth_0 = transmute(util.F32x8)output.contacts[0].depth;
	result.four.depth_1 = transmute(util.F32x8)output.contacts[1].depth;
	result.four.depth_2 = transmute(util.F32x8)output.contacts[2].depth;
	result.four.depth_3 = transmute(util.F32x8)output.contacts[3].depth;
	result.four.feature_id_0 = transmute(util.I32x8)output.contacts[0].feature_id;
	result.four.feature_id_1 = transmute(util.I32x8)output.contacts[1].feature_id;
	result.four.feature_id_2 = transmute(util.I32x8)output.contacts[2].feature_id;
	result.four.feature_id_3 = transmute(util.I32x8)output.contacts[3].feature_id;
	result.four.contact_0_exists = transmute(util.I32x8)output.contacts[0].exists;
	result.four.contact_1_exists = transmute(util.I32x8)output.contacts[1].exists;
	result.four.contact_2_exists = transmute(util.I32x8)output.contacts[2].exists;
	result.four.contact_3_exists = transmute(util.I32x8)output.contacts[3].exists;
	return .Ok;
}

abi_collision_task_wide_bridge :: proc "contextless" (
	user_context: rawptr, bundle: ^physics.Collision_Convex_Wide_Bundle,
	shapes: ^physics.Shape_Registry, result: ^physics.Collision_Wide_Manifold_Result,
) -> entasis.Status
{
	if bundle == nil || result == nil || bundle.count < 1 || bundle.count > 8
	{
		return .Invalid_Argument;
	}
	binding := (^abi_custom_task_binding)(user_context);
	input := abi_collision_task_wide_input(binding, bundle);
	return abi_collision_task_wide_invoke(binding, &input, shapes, result);
}

abi_collision_task_triangle_wide_bridge :: #force_no_inline proc "contextless" (
	user_context: rawptr, bundle: ^physics.Collision_Convex_Wide_Bundle,
	shapes: ^physics.Shape_Registry, result: ^physics.Collision_Wide_Manifold_Result,
) -> entasis.Status
{
	if bundle == nil || result == nil || bundle.count < 1 || bundle.count > 8
	{
		return .Invalid_Argument;
	}
	binding := (^abi_custom_task_binding)(user_context);
	input := abi_collision_task_wide_input(binding, bundle);
	// a replaceable C route has at least one custom shape, so it has at most
	// one built-in triangle side. built-in/built-in routes remain protected
	triangles: [8]physics.Triangle;
	pointers: [8]rawptr;
	wide := &bundle.b.triangle;
	if binding.collision.shape_type_a == physics.TRIANGLE_TYPE_ID
	{
		wide = &bundle.a.triangle;
		input.shape_a = raw_data(pointers[:]);
	}
	else
	{
		input.shape_b = raw_data(pointers[:]);
	}
	for lane in 0..<bundle.count
	{
		triangles[lane] = {util.vector3_wide_read_slot(wide.a, lane),
			util.vector3_wide_read_slot(wide.b, lane), util.vector3_wide_read_slot(wide.c, lane)};
		pointers[lane] = &triangles[lane];
	}
	// only this explicitly selected C path pays for the 288-byte geometry
	// array and 64-byte pointer array. no allocation or bundle mutation occurs
	return abi_collision_task_wide_invoke(binding, &input, shapes, result);
}

abi_sweep_task_input :: #force_inline proc "contextless" (
	shape_a, shape_b: rawptr, type_a, type_b: int,
	parent_a, parent_b, local_a, local_b: physics.Rigid_Pose,
	va, vb: physics.Body_Velocity, maximum_t, progression, convergence: f32, iterations: int,
) -> Entasis_Sweep_Task_Input
{
	return {shape_a=shape_a, shape_b=shape_b, type_a=Entasis_Shape_Type_ID(type_a), type_b=Entasis_Shape_Type_ID(type_b),
		parent_pose_a=abi_pose_from_core(parent_a), parent_pose_b=abi_pose_from_core(parent_b),
		local_pose_a=abi_pose_from_core(local_a), local_pose_b=abi_pose_from_core(local_b),
		velocity_a=abi_velocity_from_core(va), velocity_b=abi_velocity_from_core(vb),
		maximum_t=maximum_t, minimum_progression=progression, convergence_threshold=convergence, maximum_iteration_count=i32(iterations)};
}

abi_sweep_task_result_from_core :: #force_inline proc "contextless" (result: physics.Sweep_Result) -> Entasis_Sweep_Task_Result
{
	return {t0=result.t0, t1=result.t1, location=abi_vector3_from_core(result.location), normal=abi_vector3_from_core(result.normal),
		child_a=result.child_a, child_b=result.child_b, hit=abi_bool(result.state == .Hit)};
}

abi_sweep_task_result_to_core :: proc "contextless" (
	value: Entasis_Sweep_Task_Result, maximum_t: f32,
) -> (physics.Sweep_Result, entasis.Status)
{
	if value.hit > ENTASIS_TRUE
	{
		return {}, .Invalid_Description;
	}
	if value.hit == ENTASIS_FALSE
	{
		return {state=.Miss, t0=maximum_t, t1=maximum_t, child_a=-1, child_b=-1}, .Ok;
	}
	if !(value.t0 >= 0 && value.t0 <= maximum_t && value.t1 >= value.t0) || (abi_task_scalar_status(value.t1) != .Ok) ||
	(abi_task_vector_status(value.location) != .Ok) || (abi_task_vector_status(value.normal) != .Ok)
	{
		return {}, .Invalid_Description;
	}
	return {state=.Hit, t0=value.t0, t1=value.t1, location=abi_vector3_to_core(value.location), normal=abi_vector3_to_core(value.normal),
		child_a=value.child_a, child_b=value.child_b}, .Ok;
}

abi_Sweep_Target :: enum u8
{
	Whole,
	Child,
}

abi_sweep_task_invoke :: #force_inline proc "contextless" (
	binding: ^abi_custom_task_binding, input: ^Entasis_Sweep_Task_Input, shapes: ^physics.Shape_Registry,
	filter: physics.Collision_Child_Filter_Proc, filter_context: rawptr, $child: abi_Sweep_Target,
) -> (physics.Sweep_Result, entasis.Status)
{
	scope := abi_task_scope(binding, shapes, -1, filter, filter_context);
	output := Entasis_Sweep_Task_Result{t0=input.maximum_t, t1=input.maximum_t, child_a=-1, child_b=-1};
	callback := binding.sweep.test;
	when child == .Child
	{
		callback = binding.sweep.child_test;
	}
	status := abi_custom_callback_status(callback(binding.sweep.user_context, input, {opaque=&scope}, &output));
	if status != .Ok
	{
		return {}, status;
	}
	return abi_sweep_task_result_to_core(output, input.maximum_t);
}

abi_sweep_task_bridge :: proc "contextless" (
	user_context, shape_a, shape_b: rawptr, type_a, type_b: int,
	pose_a, pose_b: physics.Rigid_Pose, va, vb: physics.Body_Velocity,
	maximum_t, progression, convergence: f32, iterations: int,
	shapes: ^physics.Shape_Registry, tasks: ^physics.Collision_Task_Registry,
	filter: physics.Collision_Child_Filter_Proc, filter_context: rawptr,
) -> (physics.Sweep_Result, entasis.Status)
{
	_ = tasks;
	input := abi_sweep_task_input(shape_a, shape_b, type_a, type_b, pose_a, pose_b,
		physics.rigid_pose_identity(), physics.rigid_pose_identity(), va, vb, maximum_t, progression, convergence, iterations);
	return abi_sweep_task_invoke((^abi_custom_task_binding)(user_context), &input, shapes, filter, filter_context, .Whole);
}

abi_sweep_task_child_bridge :: proc "contextless" (
	user_context, shape_a, shape_b: rawptr, type_a, type_b: int,
	parent_a, parent_b, local_a, local_b: physics.Rigid_Pose, va, vb: physics.Body_Velocity,
	maximum_t, progression, convergence: f32, iterations: int,
	shapes: ^physics.Shape_Registry, tasks: ^physics.Collision_Task_Registry,
) -> (physics.Sweep_Result, entasis.Status)
{
	_ = tasks;
	input := abi_sweep_task_input(shape_a, shape_b, type_a, type_b, parent_a, parent_b, local_a, local_b,
		va, vb, maximum_t, progression, convergence, iterations);
	return abi_sweep_task_invoke((^abi_custom_task_binding)(user_context), &input, shapes, nil, nil, .Child);
}

abi_custom_task_bindings_release :: proc "contextless" (resource: ^abi_world_resource)
{
	binding := resource.custom_tasks;
	resource.custom_tasks = nil;
	for binding != nil
	{
		next := binding.next;
		abi_resource_free(binding, size_of(abi_custom_task_binding), align_of(abi_custom_task_binding), &resource.allocator);
		binding = next;
	}
}

abi_collision_task_registration_default :: proc "contextless" () -> Entasis_Collision_Task_Registration
{
	return {struct_size=u32(size_of(Entasis_Collision_Task_Registration)), struct_version=ENTASIS_STRUCT_VERSION,
		shape_type_a=-1, shape_type_b=-1, batch_size=u32(physics.MAXIMUM_COLLISION_TASK_BATCH_SIZE)};
}

abi_sweep_task_registration_default :: proc "contextless" () -> Entasis_Sweep_Task_Registration
{
	return {struct_size=u32(size_of(Entasis_Sweep_Task_Registration)), struct_version=ENTASIS_STRUCT_VERSION, shape_type_a=-1, shape_type_b=-1};
}

abi_task_registration_simulation :: proc "contextless" (
	resource: ^abi_world_resource, type_a, type_b: Entasis_Shape_Type_ID,
) -> (^physics.Simulation, entasis.Status)
{
	simulation, status := entasis.world_borrow_simulation(&resource.world);
	if status != .Ok
	{
		return nil, status;
	}
	if simulation.step_index != 0
	{
		return nil, .Invalid_Argument;
	}
	shapes := physics.simulation_shape_registry(simulation);
	if shapes == nil || type_a < 0 || type_b < 0 || int(type_a) >= shapes.registered_type_count || int(type_b) >= shapes.registered_type_count
	{
		return nil, .Invalid_Description;
	}
	return simulation, .Ok;
}

abi_collision_task_register :: proc "contextless" (
	world: ^Entasis_World, registration: ^Entasis_Collision_Task_Registration, out_task_id: ^i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if registration == nil || out_task_id == nil
	{
		if out_task_id != nil
		{
			out_task_id^ = -1;
		}
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		out_task_id^ = -1;
		return ready;
	}
	if registration.struct_size < u32(size_of(Entasis_Collision_Task_Registration)) || registration.struct_version != ENTASIS_STRUCT_VERSION
	{
		out_task_id^ = -1;
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	// copy before aliased output writes and before application allocation code
	selected := registration^;
	out_task_id^ = -1;
	if selected.test == nil || selected.wide_test == nil || selected.batch_size == 0 ||
	selected.batch_size > u32(physics.MAXIMUM_COLLISION_TASK_BATCH_SIZE) || selected.pair_type > u32(physics.Collision_Task_Pair_Type.Bounds_Tested)
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	simulation, status := abi_task_registration_simulation(resource, selected.shape_type_a, selected.shape_type_b);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	wide_bridge: physics.Contextual_Collision_Wide_Test_Proc = abi_collision_task_wide_bridge;
	if selected.shape_type_a == physics.TRIANGLE_TYPE_ID || selected.shape_type_b == physics.TRIANGLE_TYPE_ID
	{
		wide_bridge = abi_collision_task_triangle_wide_bridge;
	}
	probe := physics.Contextual_Collision_Task_Binding{test=abi_collision_task_bridge, wide_test=wide_bridge};
	status = physics.collision_task_registry_validate_registration(&simulation.collision_tasks, {
			shape_type_a=i16(selected.shape_type_a), shape_type_b=i16(selected.shape_type_b), batch_size=i16(selected.batch_size),
			pair_type=physics.Collision_Task_Pair_Type(selected.pair_type), kind=.Convex, capabilities={.Convex_Result, .Wide_Result}, dispatch=.Contextual, contextual=&probe});
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	memory, _, allocation_status := abi_resource_allocate_owned(size_of(abi_custom_task_binding), align_of(abi_custom_task_binding), &resource.allocator);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .Shape_Register);
	}
	binding := (^abi_custom_task_binding)(memory);
	binding.collision = selected;
	binding.collision_tasks = &simulation.collision_tasks;
	context = runtime.default_context();
	id, register_status := entasis.collision_task_register_contextual(&resource.world, {
			shape_type_a=entasis.Shape_Type_ID(selected.shape_type_a), shape_type_b=entasis.Shape_Type_ID(selected.shape_type_b),
			batch_size=int(selected.batch_size), pair_type=entasis.Collision_Task_Pair_Type(selected.pair_type),
			user_context=binding, test=abi_collision_task_bridge, wide_test=wide_bridge});
	if register_status != .Ok
	{
		abi_resource_free(binding, size_of(abi_custom_task_binding), align_of(abi_custom_task_binding), &resource.allocator);
		return abi_status_finish(register_status, diagnostic, .Shape_Register);
	}
	binding.task_id = id;
	binding.next = resource.custom_tasks;
	resource.custom_tasks = binding;
	out_task_id^ = id;
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_sweep_task_register :: proc "contextless" (
	world: ^Entasis_World, registration: ^Entasis_Sweep_Task_Registration, out_task_id: ^i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if registration == nil || out_task_id == nil
	{
		if out_task_id != nil
		{
			out_task_id^ = -1;
		}
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		out_task_id^ = -1;
		return ready;
	}
	if registration.struct_size < u32(size_of(Entasis_Sweep_Task_Registration)) || registration.struct_version != ENTASIS_STRUCT_VERSION
	{
		out_task_id^ = -1;
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	selected := registration^;
	out_task_id^ = -1;
	if selected.test == nil || selected.child_test == nil
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	simulation, status := abi_task_registration_simulation(resource, selected.shape_type_a, selected.shape_type_b);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	probe := physics.Contextual_Sweep_Task_Binding{test=abi_sweep_task_bridge, child_test=abi_sweep_task_child_bridge};
	status = physics.sweep_task_registry_validate_registration(&simulation.sweep_tasks, int(selected.shape_type_a), int(selected.shape_type_b), nil, nil, .Contextual, &probe);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	memory, _, allocation_status := abi_resource_allocate_owned(size_of(abi_custom_task_binding), align_of(abi_custom_task_binding), &resource.allocator);
	if allocation_status != .Ok
	{
		return abi_status_finish(allocation_status, diagnostic, .Shape_Register);
	}
	binding := (^abi_custom_task_binding)(memory);
	binding.sweep = selected;
	binding.collision_tasks = &simulation.collision_tasks;
	context = runtime.default_context();
	id, register_status := entasis.sweep_task_register_contextual(&resource.world, {
			shape_type_a=entasis.Shape_Type_ID(selected.shape_type_a), shape_type_b=entasis.Shape_Type_ID(selected.shape_type_b),
			user_context=binding, test=abi_sweep_task_bridge, child_test=abi_sweep_task_child_bridge});
	if register_status != .Ok
	{
		abi_resource_free(binding, size_of(abi_custom_task_binding), align_of(abi_custom_task_binding), &resource.allocator);
		return abi_status_finish(register_status, diagnostic, .Shape_Register);
	}
	binding.task_id = id;
	binding.next = resource.custom_tasks;
	resource.custom_tasks = binding;
	out_task_id^ = id;
	return abi_status_finish(.Ok, diagnostic, .Shape_Register);
}

abi_task_shape_access :: proc "contextless" (access: Entasis_Task_Access, out_access: ^Entasis_Shape_Access) -> Entasis_Status
{
	if out_access == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_access^ = {};
	scope := abi_task_scope_get(access);
	if scope == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_access.opaque = &scope.shape_access;
	return abi_status(.Ok);
}

abi_task_allow_child :: proc "contextless" (
	access: Entasis_Task_Access, pair_id, child_a, child_b: i32, out_allowed: ^Entasis_Bool,
) -> Entasis_Status
{
	if out_allowed == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_allowed^ = ENTASIS_FALSE;
	scope := abi_task_scope_get(access);
	if scope == nil
	{
		return abi_status(.Invalid_Argument);
	}
	scope.executing = .Executing;
	defer scope.executing = .Idle;
	out_allowed^ = abi_bool(scope.filter == nil || scope.filter(scope.filter_context, pair_id, child_a, child_b) == .Allow);
	return abi_status(.Ok);
}

abi_task_typed_shapes_status :: #force_inline proc "contextless" (
	scope: ^abi_task_access_scope, a, b: rawptr, type_a, type_b: Entasis_Shape_Type_ID,
) -> entasis.Status
{
	return .Ok if a != nil && b != nil && type_a >= 0 && type_b >= 0 &&
	int(type_a) < scope.shapes.registered_type_count && int(type_b) < scope.shapes.registered_type_count else .Invalid_Argument;
}

abi_task_collide_convex :: proc "contextless" (
	access: Entasis_Task_Access, input: ^Entasis_Collision_Task_Input, out_result: ^Entasis_Convex_Contact_Manifold,
) -> Entasis_Status
{
	if out_result == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_result^ = {};
	scope := abi_task_scope_get(access);
	if scope == nil || input == nil || (abi_task_typed_shapes_status(scope, input.shape_a, input.shape_b, input.type_a, input.type_b) != .Ok) ||
	(abi_task_scalar_status(input.speculative_margin) != .Ok) || input.speculative_margin < 0
	{
		return abi_status(.Invalid_Argument);
	}
	task, _, status := physics.collision_task_registry_lookup(scope.collision_tasks, int(input.type_a), int(input.type_b));
	if status != .Ok
	{
		return abi_status(status);
	}
	if task.task_id == scope.blocked_collision_id
	{
		return abi_status(.Invalid_Argument);
	}
	scope.executing = .Executing;
	defer scope.executing = .Idle;
	value, test_status := physics.collision_task_registry_test_convex(scope.collision_tasks, int(input.type_a), int(input.type_b),
		input.shape_a, input.shape_b, abi_pose_to_core(input.pose_a), abi_pose_to_core(input.pose_b), input.speculative_margin, scope.shapes);
	if test_status == .Ok
	{
		out_result^ = transmute(Entasis_Convex_Contact_Manifold)value;
	}
	return abi_status(test_status);
}

abi_task_sweep_convex :: proc "contextless" (
	access: Entasis_Task_Access, input: ^Entasis_Sweep_Task_Input, out_result: ^Entasis_Sweep_Task_Result,
) -> Entasis_Status
{
	if out_result == nil
	{
		return abi_status(.Invalid_Argument);
	}
	out_result^ = {child_a=-1, child_b=-1};
	scope := abi_task_scope_get(access);
	if scope == nil || input == nil || (abi_task_typed_shapes_status(scope, input.shape_a, input.shape_b, input.type_a, input.type_b) != .Ok) ||
	input.type_a > physics.CONVEX_HULL_TYPE_ID || input.type_b > physics.CONVEX_HULL_TYPE_ID ||
	!(input.maximum_t >= 0 && (abi_task_scalar_status(input.maximum_t) == .Ok)) ||
	!(input.minimum_progression > 0 && (abi_task_scalar_status(input.minimum_progression) == .Ok)) ||
	!(input.convergence_threshold > 0 && (abi_task_scalar_status(input.convergence_threshold) == .Ok)) || input.maximum_iteration_count <= 0
	{
		return abi_status(.Invalid_Argument);
	}
	out_result.t0 = input.maximum_t;
	out_result.t1 = input.maximum_t;
	scope.executing = .Executing;
	defer scope.executing = .Idle;
	// this existing kernel supports built-in convex payloads only. a custom
	// callback may convert its payload or implement its own sweep algorithm
	value, status := physics.sweep_task_test_convex_distance_internal(input.shape_a, input.shape_b, int(input.type_a), int(input.type_b),
		abi_pose_to_core(input.parent_pose_a), abi_pose_to_core(input.parent_pose_b), abi_pose_to_core(input.local_pose_a), abi_pose_to_core(input.local_pose_b),
		abi_velocity_to_core(input.velocity_a), abi_velocity_to_core(input.velocity_b), input.maximum_t,
		input.minimum_progression, input.convergence_threshold, int(input.maximum_iteration_count), scope.shapes);
	if status == .Ok
	{
		out_result^ = abi_sweep_task_result_from_core(value);
	}
	return abi_status(status);
}

// callback-free compound routes live entirely in the existing native registry.
// custom payload classifications do not grant access to native compound layouts
abi_compound_task_registration_default :: proc "contextless" () -> Entasis_Compound_Task_Registration
{
	return {
		struct_size=u32(size_of(Entasis_Compound_Task_Registration)), struct_version=ENTASIS_STRUCT_VERSION,
		shape_type_a=-1, shape_type_b=-1, batch_size=16,
		kind=u32(physics.Collision_Task_Kind.Convex_Compound),
		capabilities=(u32(1) << u32(physics.Collision_Task_Capability.Subtask_Generator)) | (u32(1) << u32(physics.Collision_Task_Capability.Child_Order)),
	};
}

abi_collision_task_compound_register :: proc "contextless" (
	world: ^Entasis_World, registration: ^Entasis_Compound_Task_Registration,
	out_task_id: ^i32, diagnostic: ^Entasis_Diagnostic,
) -> Entasis_Status
{
	if registration == nil || out_task_id == nil
	{
		if out_task_id != nil
		{
			out_task_id^ = -1;
		}
		return abi_status_finish(.Invalid_Argument, diagnostic, .Shape_Register);
	}
	resource, ready := abi_world_resource_for_domain(world, diagnostic, .Shape_Register);
	defer abi_world_release(resource);
	if resource == nil
	{
		out_task_id^ = -1;
		return ready;
	}
	if registration.struct_size < u32(size_of(Entasis_Compound_Task_Registration)) || registration.struct_version != ENTASIS_STRUCT_VERSION
	{
		out_task_id^ = -1;
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	selected := registration^;
	out_task_id^ = -1;
	allowed :: (u32(1) << u32(physics.Collision_Task_Capability.Subtask_Generator)) | (u32(1) << u32(physics.Collision_Task_Capability.Child_Order)) | (u32(1) << u32(physics.Collision_Task_Capability.Mesh_Reduction));
	if selected.batch_size == 0 || selected.batch_size > u32(physics.MAXIMUM_COLLISION_TASK_BATCH_SIZE) ||
	(selected.kind != u32(physics.Collision_Task_Kind.Convex_Compound) && selected.kind != u32(physics.Collision_Task_Kind.Compound_Pair)) ||
	selected.capabilities & ~allowed != 0 || selected.capabilities & (u32(1) << u32(physics.Collision_Task_Capability.Subtask_Generator)) == 0
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	simulation, status := abi_task_registration_simulation(resource, selected.shape_type_a, selected.shape_type_b);
	if status != .Ok
	{
		return abi_status_finish(status, diagnostic, .Shape_Register);
	}
	if selected.shape_type_b < physics.COMPOUND_TYPE_ID || selected.shape_type_b > physics.MESH_TYPE_ID
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	shapes := physics.simulation_shape_registry(simulation);
	if selected.kind == u32(physics.Collision_Task_Kind.Convex_Compound)
	{
		if shapes.batches[int(selected.shape_type_a)].metadata.batch_type != .Convex
		{
			return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
		}
	}
	else if selected.shape_type_a < physics.COMPOUND_TYPE_ID || selected.shape_type_a > physics.MESH_TYPE_ID
	{
		return abi_status_finish(.Invalid_Description, diagnostic, .Shape_Register);
	}
	context = runtime.default_context();
	id, register_status := entasis.collision_task_register(&resource.world, entasis.collision_task_compound(
			entasis.Shape_Type_ID(selected.shape_type_a), entasis.Shape_Type_ID(selected.shape_type_b), int(selected.batch_size),
			entasis.Collision_Task_Kind(selected.kind), transmute(entasis.Collision_Task_Capabilities)u8(selected.capabilities)));
	if register_status == .Ok
	{
		out_task_id^ = id;
	}
	return abi_status_finish(register_status, diagnostic, .Shape_Register);
}
