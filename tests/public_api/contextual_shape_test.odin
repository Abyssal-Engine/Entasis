package public_api_tests

import "base:runtime"
import "core:mem"
import "core:sync"
import "core:testing"
import "core:thread"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

Contextual_Child_Forwarding :: enum u8
{
	Local,
	Forward,
}

Contextual_Compound_Kind :: enum u8
{
	Small,
	Big,
}

Contextual_Shape_State :: struct
{
	scale: f32,
	calls: [6]i32,
	failure: e.Status,
	child: e.Shape_Handle,
	forward_child: Contextual_Child_Forwarding,
}

contextual_shape_bounds_test :: proc "contextless" (
	user_context, shape: rawptr, orientation: ^u.Quaternion,
	access: e.Shape_Access, result: ^p.Shape_Bounds,
) -> e.Status
{
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[0], 1, .Relaxed);
	if state.failure != .Ok
	{
		result.max = {123, 123, 123};
		return state.failure;
	}
	if state.forward_child == .Forward
	{
		value: p.Shape_Access_Value;
		status: p.Physics_Status;
		value, status = e.shape_access_resolve(access, state.child);
		if status != .Ok || value.data == nil || value.size <= 0
		{
			return .Not_Found;
		}
		result^, status = e.shape_access_compute_bounds(access, state.child, orientation^);
		return status;
	}
	sphere: e.Sphere = e.sphere((^Custom_Sphere)(shape).radius * state.scale);
	if p.sphere_validate(sphere) != .Ok
	{
		return .Invalid_Description;
	}
	result^ = p.sphere_bounds(sphere, orientation^);
	return .Ok;
}

contextual_shape_inertia_test :: proc "contextless" (
	user_context, shape: rawptr, mass: f32, access: e.Shape_Access, result: ^p.Body_Inertia,
) -> e.Status
{
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[1], 1, .Relaxed);
	if state.failure != .Ok
	{
		result.inverse_mass = 123;
		return state.failure;
	}
	if state.forward_child == .Forward
	{
		value: p.Body_Inertia;
		status: p.Physics_Status;
		value, status = e.shape_access_compute_inertia(access, state.child, mass);
		result^ = value;
		return status;
	}
	value: p.Body_Inertia;
	status: p.Physics_Status;
	value, status = p.sphere_inertia(e.sphere((^Custom_Sphere)(shape).radius * state.scale), mass);
	result^ = value;
	return status;
}

contextual_shape_ray_test :: proc "contextless" (
	user_context, shape: rawptr, pose: ^p.Rigid_Pose, ray: ^p.Tree_Ray,
	access: e.Shape_Access, result: ^p.Shape_Ray_Hit,
) -> e.Status
{
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[2], 1, .Relaxed);
	if state.failure != .Ok
	{
		result.state = .Present;
		return state.failure;
	}
	if state.forward_child == .Forward
	{
		value: p.Shape_Ray_Hit;
		status: p.Physics_Status;
		value, status = e.shape_access_ray_test(access, state.child, pose^, ray^);
		result^ = value;
		return status;
	}
	value: p.Shape_Ray_Hit;
	status: p.Physics_Status;
	value, status = p.sphere_ray_test(e.sphere((^Custom_Sphere)(shape).radius * state.scale), pose^, ray^);
	result^ = value;
	return status;
}

contextual_shape_support_test :: proc "contextless" (
	user_context, shape: rawptr, direction: ^u.Vector3,
	access: e.Shape_Access, result: ^u.Vector3,
) -> e.Status
{
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[3], 1, .Relaxed);
	if state.failure != .Ok
	{
		result^ = {123, 123, 123};
		return state.failure;
	}
	if state.forward_child == .Forward
	{
		value: u.Vector3;
		status: p.Physics_Status;
		value, status = e.shape_access_support(access, state.child, direction^);
		result^ = value;
		return status;
	}
	value: u.Vector3;
	status: p.Physics_Status;
	value, status = p.sphere_support(e.sphere((^Custom_Sphere)(shape).radius * state.scale), direction^);
	result^ = value;
	return status;
}

contextual_shape_sweep_support_test :: proc "contextless" (
	user_context, shape: rawptr, direction: ^u.Vector3,
	access: e.Shape_Access, result: ^u.Vector3,
) -> e.Status
{
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[4], 1, .Relaxed);
	if state.forward_child == .Forward
	{
		value: u.Vector3;
		status: p.Physics_Status;
		value, status = e.shape_access_sweep_support(access, state.child, direction^);
		result^ = value;
		return status;
	}
	value: u.Vector3;
	status: p.Physics_Status;
	value, status = p.sphere_support(e.sphere((^Custom_Sphere)(shape).radius * state.scale), direction^);
	result^ = value;
	return status;
}

contextual_shape_dispose_test :: proc "contextless" (
	user_context, shape: rawptr, access: e.Shape_Access, pool: ^u.Buffer_Pool,
) -> e.Status
{
	_ = shape;
	_ = access;
	_ = pool;
	state: ^Contextual_Shape_State = (^Contextual_Shape_State)(user_context);
	sync.atomic_add_explicit(&state.calls[5], 1, .Relaxed);
	return state.failure;
}

contextual_sphere_registration :: proc "contextless" (
	state: ^Contextual_Shape_State,
) -> e.Contextual_Custom_Shape_Registration
{
	return e.custom_shape_registration_contextual(Custom_Sphere, .Convex, state,
		contextual_shape_bounds_test, contextual_shape_inertia_test, contextual_shape_ray_test,
		contextual_shape_support_test, contextual_shape_sweep_support_test, contextual_shape_dispose_test);
}

contextual_sphere_add :: proc (
	t: ^testing.T, world: ^e.World, state: ^Contextual_Shape_State,
) -> (e.Shape_Handle, e.Shape_Type_ID, e.Status)
{
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(state);
	type_id: e.Shape_Type_ID;
	status: e.Status;
	type_id, status = e.custom_shape_register_contextual(world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return {}, {}, status;
	}
	// the registered table must not reference this stack descriptor
	registration.bounds = nil;
	registration.user_context = nil;
	handle: e.Shape_Handle;
	add_status: e.Status;
	handle, add_status = e.custom_shape_add(world, type_id, &Custom_Sphere{1});
	testing.expect_value(t, add_status, e.Status.Ok);
	return handle, type_id, add_status;
}

@(test)
contextual_shapes_copy_tables_and_keep_world_bindings_independent :: proc(t: ^testing.T)
{
	worlds: [2]e.World;
	states: [2]Contextual_Shape_State = [2]Contextual_Shape_State{{scale=1}, {scale=2}};
	types: [2]e.Shape_Type_ID;
	defer for &world in worlds
	{
		_ = e.world_destroy(&world);
	}
	for &world, i in worlds
	{
		if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
		{
			return;
		}
		handle: e.Shape_Handle;
		type_id: e.Shape_Type_ID;
		ok: e.Status;
		handle, type_id, ok = contextual_sphere_add(t, &world, &states[i]);
		if ok != .Ok
		{
			return;
		}
		types[i] = type_id;
		info: e.Shape_Info;
		status: e.Status;
		info, status = e.shape_inspect(&world, handle);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, info.value_size, size_of(Custom_Sphere));
		testing.expect_value(t, info.value_alignment, align_of(Custom_Sphere));
		bounds: e.Shape_Bounds;
		bounds_status: e.Status;
		bounds, bounds_status = e.shape_bounds(&world, handle);
		testing.expect_value(t, bounds_status, e.Status.Ok);
		testing.expect_value(t, bounds.max.x, states[i].scale);
		registry: ^p.Shape_Registry = contextual_shape_test_registry(&world);
		inertia: p.Body_Inertia;
		inertia_status: p.Physics_Status;
		inertia, inertia_status = p.shape_registry_compute_inertia(registry, handle, 2);
		expected: p.Body_Inertia;
		expected, _ = p.sphere_inertia(e.sphere(states[i].scale), 2);
		testing.expect_value(t, inertia_status, e.Status.Ok);
		testing.expect_value(t, inertia, expected);
		support: u.Vector3;
		support_status: p.Physics_Status;
		support, support_status = p.shape_registry_support(registry, handle, {1, 0, 0});
		testing.expect_value(t, support_status, e.Status.Ok);
		testing.expect_value(t, support, u.Vector3{states[i].scale, 0, 0});
		ray: e.Shape_Ray_Hit;
		ray_status: e.Status;
		ray, ray_status = e.shape_ray(&world, handle, e.pose(), {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
		testing.expect_value(t, ray_status, e.Status.Ok);
		testing.expect_value(t, ray.t, 5-states[i].scale);
	}
	testing.expect_value(t, types[0], types[1]);
}

@(test)
contextual_shapes_reject_invalid_tables_layouts_and_late_registration :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	initial: e.Shape_Type_ID;
	initial, _ = e.custom_shape_next_type_id(&world);
	for invalid in 0 ..< 13
	{
		bad: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state);
		switch invalid
		{
			case 0: bad.bounds = nil;
			case 1: bad.inertia = nil;
			case 2: bad.ray = nil;
			case 3: bad.support = nil;
			case 4: bad.sweep_support = nil;
			case 5: bad.dispose = nil;
			case 6: bad.size = 0;
			case 7: bad.alignment = 0;
			case 8: bad.alignment = 3;
			case 9: bad.alignment = 256;
			case 10: bad.size = max(int);
			case 11: bad.expected_type_id = e.Shape_Type_ID(int(initial)+1);
			case 12: bad.batch_type = cast(e.Shape_Batch_Type)255;
		}
		id: e.Shape_Type_ID;
		status: e.Status;
		id, status = e.custom_shape_register_contextual(&world, bad);
		testing.expect_value(t, status, e.Status.Invalid_Description);
		testing.expect_value(t, id, e.SHAPE_TYPE_INVALID);
		next: e.Shape_Type_ID;
		next, _ = e.custom_shape_next_type_id(&world);
		testing.expect_value(t, next, initial);
	}
	testing.expect_value(t, e.world_step(&world, 1.0/60.0), e.Status.Ok);
	status: e.Status;
	_, status = e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
}

@(test)
contextual_shapes_keep_full_type_capacity_and_allow_native_neighbors :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	for expected in e.BUILT_IN_SHAPE_TYPE_COUNT ..< e.MAXIMUM_SHAPE_TYPE_COUNT
	{
		id: e.Shape_Type_ID;
		status: e.Status;
		if expected & 1 == 0
		{
			id, status = e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
		}
		else
		{
			id, status = e.custom_shape_register(&world, e.custom_shape_registration(Custom_Sphere, .Convex,
					custom_sphere_bounds, custom_sphere_inertia, custom_sphere_ray, custom_sphere_support));
		}
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, int(id), expected);
	}
	status: e.Status;
	_, status = e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
	testing.expect_value(t, status, e.Status.Capacity_Missing);
}

@(test)
contextual_shapes_callback_errors_do_not_publish_partial_results :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	handle: e.Shape_Handle;
	ok: e.Status;
	handle, _, ok = contextual_sphere_add(t, &world, &state);
	if ok != .Ok
	{
		return;
	}
	registry: ^p.Shape_Registry = contextual_shape_test_registry(&world);
	state.failure = .Invalid_Description;
	bounds: e.Shape_Bounds;
	bs: e.Status;
	bounds, bs = e.shape_bounds(&world, handle);
	inertia: p.Body_Inertia;
	is: p.Physics_Status;
	inertia, is = p.shape_registry_compute_inertia(registry, handle, 1);
	ray: e.Shape_Ray_Hit;
	rs: e.Status;
	ray, rs = e.shape_ray(&world, handle, e.pose(), {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
	support: u.Vector3;
	ss: p.Physics_Status;
	support, ss = p.shape_registry_support(registry, handle, {1, 0, 0});
	testing.expect_value(t, bs, state.failure);
	testing.expect_value(t, is, state.failure);
	testing.expect_value(t, rs, state.failure);
	testing.expect_value(t, ss, state.failure);
	testing.expect_value(t, bounds, p.Shape_Bounds{});
	testing.expect_value(t, inertia, p.Body_Inertia{});
	testing.expect_value(t, ray.state, p.Reference_State.Missing);
	testing.expect_value(t, support, u.Vector3{});
	testing.expect_value(t, e.shape_remove(&world, handle), state.failure);
	inspect_status: e.Status;
	_, inspect_status = e.shape_inspect(&world, handle);
	testing.expect_value(t, inspect_status, e.Status.Ok);
	state.failure = .Ok;
	testing.expect_value(t, e.shape_remove(&world, handle), e.Status.Ok);
	testing.expect_value(t, e.shape_remove(&world, handle), e.Status.Not_Found);
	testing.expect_value(t, state.calls[5], i32(2));
}

@(test)
contextual_shapes_child_scope_handles_native_and_contextual_values :: proc(t: ^testing.T)
{
	world: e.World;
	states: [2]Contextual_Shape_State = [2]Contextual_Shape_State{{scale=2}, {scale=1, forward_child=.Forward}};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	child: e.Shape_Handle;
	child_ok: e.Status;
	child, _, child_ok = contextual_sphere_add(t, &world, &states[0]);
	parent: e.Shape_Handle;
	parent_ok: e.Status;
	parent, _, parent_ok = contextual_sphere_add(t, &world, &states[1]);
	if child_ok != .Ok || parent_ok != .Ok
	{
		return;
	}
	native_child: e.Shape_Handle;
	native_child, _ = e.shape_add(&world, e.sphere(2));
	registry: ^p.Shape_Registry = contextual_shape_test_registry(&world);
	for selected in ([2]e.Shape_Handle{native_child, child})
	{
		states[1].child = selected;
		bounds: e.Shape_Bounds;
		bs: e.Status;
		bounds, bs = e.shape_bounds(&world, parent);
		testing.expect_value(t, bs, e.Status.Ok);
		testing.expect_value(t, bounds.max.x, f32(2));
		inertia: p.Body_Inertia;
		is: p.Physics_Status;
		inertia, is = p.shape_registry_compute_inertia(registry, parent, 1);
		expected: p.Body_Inertia;
		expected, _ = p.sphere_inertia(e.sphere(2), 1);
		testing.expect_value(t, is, e.Status.Ok);
		testing.expect_value(t, inertia, expected);
		support: u.Vector3;
		ss: p.Physics_Status;
		support, ss = p.shape_registry_support(registry, parent, {1, 0, 0});
		testing.expect_value(t, ss, e.Status.Ok);
		testing.expect_value(t, support.x, f32(2));
		// test-only borrowed scope permits checking the distinct sweep callback
		sweep: u.Vector3;
		sws: p.Physics_Status;
		sweep, sws = p.shape_access_sweep_support(p.Shape_Access(registry), parent, {1, 0, 0});
		testing.expect_value(t, sws, e.Status.Ok);
		testing.expect_value(t, sweep.x, f32(2));
		ray: e.Shape_Ray_Hit;
		rs: e.Status;
		ray, rs = e.shape_ray(&world, parent, e.pose(), {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
		testing.expect_value(t, rs, e.Status.Ok);
		testing.expect_value(t, ray.t, f32(3));
	}
	testing.expect_value(t, states[0].calls[4], i32(1));
	states[1].child = e.shape_handle_invalid();
	missing_status: e.Status;
	_, missing_status = e.shape_bounds(&world, parent);
	testing.expect_value(t, missing_status, e.Status.Not_Found);
	nil_status: p.Physics_Status;
	_, nil_status = e.shape_access_resolve(nil, child);
	testing.expect_value(t, nil_status, e.Status.Not_Found);
}

Contextual_Aligned_Shape :: struct #align(128)
{
	radius: f32,
	marker: u32,
	payload: [120]u8,
}

@(test)
contextual_shapes_preserve_payload_alignment_growth_resize_and_reuse :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state);
	registration.size = size_of(Contextual_Aligned_Shape);
	registration.alignment = align_of(Contextual_Aligned_Shape);
	id: e.Shape_Type_ID;
	status: e.Status;
	id, status = e.custom_shape_register_contextual(&world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	handles: [33]e.Shape_Handle;
	for &handle, i in handles
	{
		value: Contextual_Aligned_Shape = Contextual_Aligned_Shape{radius=1, marker=u32(i)};
		for &byte in value.payload
		{
			byte = 0xa7;
		}
		handle, status = e.custom_shape_add(&world, id, &value);
		testing.expect_value(t, status, e.Status.Ok);
	}
	registry: ^p.Shape_Registry = contextual_shape_test_registry(&world);
	for capacity in ([2]int{256, 33})
	{
		testing.expect_value(t, p.shape_batch_resize(registry, int(id), capacity), e.Status.Ok);
		for handle, i in handles
		{
			pointer: rawptr;
			info: e.Shape_Info;
			borrow_status: e.Status;
			pointer, info, borrow_status = e.shape_borrow_raw(&world, handle);
			testing.expect_value(t, borrow_status, e.Status.Ok);
			testing.expect_value(t, uintptr(pointer)%128, uintptr(0));
			testing.expect_value(t, info.value_size, 128);
			value: ^Contextual_Aligned_Shape = (^Contextual_Aligned_Shape)(pointer);
			testing.expect_value(t, value.marker, u32(i));
			for byte in value.payload
			{
				testing.expect_value(t, byte, u8(0xa7));
			}
		}
	}
	testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
	testing.expect_value(t, state.calls[5], i32(33));
	_, status = e.custom_shape_add(&world, id, &Contextual_Aligned_Shape{radius=1});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, state.calls[5], i32(34));
}

@(test)
contextual_shapes_bindings_have_one_cold_owner_and_fail_without_consuming_id :: proc(t: ^testing.T)
{
	tracker: mem.Tracking_Allocator;
	mem.tracking_allocator_init(&tracker, context.allocator);
	defer mem.tracking_allocator_destroy(&tracker);
	owner: Contextual_Fallible_Allocator = Contextual_Fallible_Allocator{backing=mem.tracking_allocator(&tracker)};
	description: e.World_Description = small_world_description();
	description.allocator = {procedure=contextual_fallible_allocator, data=&owner};
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	initial_bytes: i64 = tracker.current_memory_allocated;
	next: e.Shape_Type_ID;
	next, _ = e.custom_shape_next_type_id(&world);
	owner.failure = .Reject;
	status: e.Status;
	_, status = e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
	testing.expect_value(t, status, e.Status.Capacity_Missing);
	current: e.Shape_Type_ID;
	current, _ = e.custom_shape_next_type_id(&world);
	testing.expect_value(t, current, next);
	testing.expect_value(t, tracker.current_memory_allocated, initial_bytes);
	owner.failure = .Allow;
	handle: e.Shape_Handle;
	ok: e.Status;
	handle, _, ok = contextual_sphere_add(t, &world, &state);
	if ok != .Ok
	{
		_ = e.world_destroy(&world);
		return;
	}
	testing.expect_value(t, tracker.current_memory_allocated-initial_bytes, i64(64));
	_ = handle;
	owner.failure = .Reject;
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, state.calls[5], i32(1));
	testing.expect_value(t, owner.failed, 1);
	testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	testing.expect_value(t, len(tracker.allocation_map), 0);
}

@(test)
contextual_shapes_compound_children_retain_and_use_registered_callbacks :: proc(t: ^testing.T)
{
	for compound_kind in ([2]Contextual_Compound_Kind{.Small, .Big})
	{
		f: Contextual_Task_Fixture;
		state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
		description: e.World_Description = small_world_description();
		description.capacity.collision_child_pairs = 256;
		testing.expect_value(t, e.world_init(&f.world, description), e.Status.Ok);
		defer _ = e.world_destroy(&f.world);
		var_ok: e.Status;
		f.custom, f.type_id, var_ok = contextual_sphere_add(t, &f.world, &state);
		if var_ok != .Ok
		{
			return;
		}
		f.sphere, _ = e.shape_add(&f.world, e.sphere(0.5));
		if contextual_bind_fixture(t, &f) != .Ok
		{
			return;
		}
		children: [2]e.Compound_Child = [2]e.Compound_Child{
			e.compound_child(f.custom, e.pose({-0.4, 0, 0})),
			e.compound_child(f.custom, e.pose({0.4, 0, 0})),
		};
		compound: e.Shape_Handle;
		status: e.Status;
		if compound_kind == .Big
		{
			compound, status = e.shape_import_big_compound(&f.world, children[:]);
		}
		else
		{
			compound, status = e.shape_import_compound(&f.world, children[:]);
		}
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, e.shape_remove(&f.world, f.custom), e.Status.Shape_In_Use);
		result: e.Manifold_Result;
		collision_status: e.Status;
		result, collision_status = e.collision_query(&f.world, f.sphere, e.pose({0, 1, 0}), compound, e.pose());
		testing.expect_value(t, collision_status, e.Status.Ok);
		testing.expect(t, result.nonconvex.count > 0);
		_, status = e.static_add(&f.world, e.static_body(compound, e.pose()), .None);
		testing.expect_value(t, status, e.Status.Ok);
		hit: e.Ray_Hit;
		ray_status: e.Status;
		hit, ray_status = e.ray_cast_closest(&f.world, {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
		testing.expect_value(t, ray_status, e.Status.Ok);
		testing.expect(t, hit.t > 3 && hit.t < 4);
		sweep_status: e.Status;
		_, sweep_status = e.sweep_closest(&f.world, f.sphere, e.pose({-4, 0, 0}), e.velocity({1, 0, 0}), 10);
		testing.expect_value(t, sweep_status, e.Status.Ok);
		testing.expect(t, state.calls[0] > 0 && state.calls[2] > 0);
		testing.expect_value(t, e.world_clear(&f.world), e.Status.Ok);
		testing.expect_value(t, state.calls[5], i32(1));
	}
}

@(test)
contextual_shapes_depth_refiner_dispatch_is_selected_before_iteration :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	custom: e.Shape_Handle;
	custom_type: e.Shape_Type_ID;
	ok: e.Status;
	custom, custom_type, ok = contextual_sphere_add(t, &world, &state);
	if ok != .Ok
	{
		return;
	}
	sphere: e.Shape_Handle;
	sphere, _ = e.shape_add(&world, e.sphere(1));
	registry: ^p.Shape_Registry = contextual_shape_test_registry(&world);
	custom_ptr: rawptr;
	custom_ptr, _, _ = p.shape_registry_resolve(registry, custom);
	native_ptr: rawptr;
	native_ptr, _, _ = p.shape_registry_resolve(registry, sphere);
	views_a: [2]p.Collision_Shape_View;
	views_b: [2]p.Collision_Shape_View;
	views_a[0], _ = p.collision_shape_view(native_ptr, p.SPHERE_TYPE_ID, e.pose(), registry);
	views_a[1], _ = p.collision_shape_view(custom_ptr, int(custom_type), e.pose(), registry);
	views_b[0], _ = p.collision_shape_view(native_ptr, p.SPHERE_TYPE_ID, e.pose({0, 1.5, 0}), registry);
	views_b[1], _ = p.collision_shape_view(custom_ptr, int(custom_type), e.pose({0, 1.5, 0}), registry);
	expected_depth: f32;
	expected_normal: u.Vector3;
	expected_witness: u.Vector3;
	expected_status: p.Physics_Status;
	expected_depth, expected_normal, expected_witness, expected_status = p.depth_refiner_find_minimum_depth(
		views_a[0], views_b[0], {0, 1, 0}, 1e-4, -0.1, registry);
	testing.expect_value(t, expected_status, e.Status.Ok);
	for a in views_a
	{
		for b in views_b
		{
			depth: f32;
			normal: u.Vector3;
			witness: u.Vector3;
			status: p.Physics_Status;
			depth, normal, witness, status = p.depth_refiner_find_minimum_depth(a, b, {0, 1, 0}, 1e-4, -0.1, registry);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, depth, expected_depth);
			testing.expect_value(t, normal, expected_normal);
			testing.expect_value(t, witness, expected_witness);
		}
	}
	testing.expect(t, state.calls[3] > 0);
	state.failure = .Capacity_Missing;
	status: p.Physics_Status;
	_, _, _, status = p.depth_refiner_find_minimum_depth(views_a[1], views_b[1], {0, 1, 0}, 1e-4, -0.1, registry);
	testing.expect_value(t, status, e.Status.Capacity_Missing);
}

Contextual_Shape_Query_Call :: struct
{
	query: e.Query_Context,
	failures: int,
}

contextual_shape_query_thread :: proc(host: ^thread.Thread)
{
	context = runtime.default_context();
	call: ^Contextual_Shape_Query_Call = (^Contextual_Shape_Query_Call)(host.data);
	for _ in 0 ..< 40
	{
		hit: e.Ray_Hit;
		status: e.Status;
		hit, status = e.ray_cast_closest_with_context(&call.query, {origin={-5, 0, 0}, direction={1, 0, 0}, maximum_t=10});
		if status != .Ok || hit.t != 4
		{
			call.failures += 1;
		}
	}
}

@(test)
contextual_shapes_queries_use_independent_contexts_concurrently :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	handle: e.Shape_Handle;
	ok: e.Status;
	handle, _, ok = contextual_sphere_add(t, &world, &state);
	if ok != .Ok
	{
		return;
	}
	static_status: e.Status;
	_, static_status = e.static_add(&world, e.static_body(handle, e.pose()), .None);
	testing.expect_value(t, static_status, e.Status.Ok);
	calls: [4]Contextual_Shape_Query_Call;
	hosts: [4]^thread.Thread;
	defer for host in hosts
	{
		if host != nil
		{
			thread.destroy(host);
		}
	}
	initialized: int = 0;
	defer for i in 0 ..< initialized
	{
		_ = e.query_context_destroy(&calls[i].query);
	}
	for &call, i in calls
	{
		if !testing.expect_value(t, e.query_context_init(&call.query, &world), e.Status.Ok)
		{
			return;
		}
		initialized += 1;
		hosts[i] = thread.create(contextual_shape_query_thread);
		if !testing.expect(t, hosts[i] != nil)
		{
			return;
		}
		hosts[i].data = &call;
	}
	for host in hosts
	{
		thread.start(host);
	}
	for host in hosts
	{
		thread.join(host);
	}
	for call in calls
	{
		testing.expect_value(t, call.failures, 0);
	}
	testing.expect_value(t, state.calls[2], i32(160));
}

@(test)
contextual_shapes_preserve_native_shape_storage_layout :: proc(t: ^testing.T)
{
	testing.expect_value(t, size_of(p.Shape_Type_Registration), 72);
	testing.expect_value(t, align_of(p.Shape_Type_Registration), 8);
	testing.expect_value(t, size_of(p.Shape_Batch), 168);
	testing.expect_value(t, size_of(p.Shape_Registry), 21544);
	testing.expect_value(t, size_of(p.Collision_Shape_View), 48);
	batch: p.Shape_Batch;
	testing.expect_value(t, uintptr(&batch.metadata)-uintptr(&batch), uintptr(88));
	testing.expect_value(t, uintptr(&batch.state)-uintptr(&batch), uintptr(160));
	testing.expect_value(t, uintptr(&batch.stride)-uintptr(&batch), uintptr(80));
}

contextual_shape_test_registry :: proc "contextless" (world: ^e.World) -> ^p.Shape_Registry
{
	simulation: ^e.Simulation;
	simulation, _ = e.world_borrow_simulation(world);
	return p.simulation_shape_registry(simulation);
}

@(test)
shape_clear_releases_native_custom_children_after_builtin_compounds :: proc(t: ^testing.T)
{
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer _ = e.world_destroy(&world);
	registration: e.Custom_Shape_Registration = e.custom_shape_registration(Custom_Sphere, .Convex,
		custom_sphere_bounds, custom_sphere_inertia, custom_sphere_ray, custom_sphere_support);
	id: e.Shape_Type_ID;
	status: e.Status;
	id, status = e.custom_shape_register(&world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	// both parent batches retain the same later-ID native custom child. exercise
	// clear, reuse with the retained registration, and direct world destruction
	for iteration in 0 ..< 2
	{
		child: e.Shape_Handle;
		child_status: e.Status;
		child, child_status = e.custom_shape_add(&world, id, &Custom_Sphere{1});
		if !testing.expect_value(t, child_status, e.Status.Ok)
		{
			return;
		}
		children: [1]e.Compound_Child = [1]e.Compound_Child{e.compound_child(child, e.pose())};
		compound_status: e.Status;
		_, compound_status = e.shape_import_compound(&world, children[:]);
		big_status: e.Status;
		_, big_status = e.shape_import_big_compound(&world, children[:]);
		testing.expect_value(t, compound_status, e.Status.Ok);
		testing.expect_value(t, big_status, e.Status.Ok);
		testing.expect_value(t, e.shape_remove(&world, child), e.Status.Shape_In_Use);
		if iteration == 0
		{
			testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
			old_status: e.Status;
			_, old_status = e.shape_bounds(&world, child);
			testing.expect_value(t, old_status, e.Status.Not_Found);
		}
	}
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
}

@(test)
contextual_shapes_default_sweep_and_disposal_are_real_operations :: proc(t: ^testing.T)
{
	world: e.World;
	state: Contextual_Shape_State = Contextual_Shape_State{scale=2};
	if !testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok)
	{
		return;
	}
	defer _ = e.world_destroy(&world);
	registration: e.Contextual_Custom_Shape_Registration = e.custom_shape_registration_contextual(Custom_Sphere, .Convex, &state,
		contextual_shape_bounds_test, contextual_shape_inertia_test, contextual_shape_ray_test, contextual_shape_support_test);
	id: e.Shape_Type_ID;
	status: e.Status;
	id, status = e.custom_shape_register_contextual(&world, registration);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	handle: e.Shape_Handle;
	add_status: e.Status;
	handle, add_status = e.custom_shape_add(&world, id, &Custom_Sphere{1});
	if !testing.expect_value(t, add_status, e.Status.Ok)
	{
		return;
	}
	access: e.Shape_Access = e.Shape_Access(contextual_shape_test_registry(&world));
	result: u.Vector3;
	support_status: p.Physics_Status;
	result, support_status = e.shape_access_sweep_support(access, handle, {1, 0, 0});
	testing.expect_value(t, support_status, e.Status.Ok);
	testing.expect_value(t, result, u.Vector3{2, 0, 0});
	testing.expect_value(t, state.calls[3], i32(1));
	testing.expect_value(t, state.calls[4], i32(0));
	testing.expect_value(t, e.shape_remove(&world, handle), e.Status.Ok);
	testing.expect_value(t, state.calls[5], i32(0));
}

@(test)
custom_shape_raw_bytes_require_exact_registered_size :: proc(t: ^testing.T)
{
	world: e.World;
	value: Custom_Sphere = Custom_Sphere{2};
	status: e.Status;
	_, status = e.custom_shape_add_raw(&world, e.Shape_Type_ID(9), &value, size_of(value));
	testing.expect_value(t, status, e.Status.Disposed);
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	registration: e.Contextual_Custom_Shape_Registration = contextual_sphere_registration(&state);
	registration.alignment = 128;
	id: e.Shape_Type_ID;
	register_status: e.Status;
	id, register_status = e.custom_shape_register_contextual(&world, registration);
	testing.expect_value(t, register_status, e.Status.Ok);
	for size in ([3]int{0, size_of(value)-1, size_of(value)+1})
	{
		handle: e.Shape_Handle;
		add_status: e.Status;
		handle, add_status = e.custom_shape_add_raw(&world, id, &value, size);
		testing.expect_value(t, add_status, e.Status.Invalid_Argument);
		testing.expect(t, !e.shape_handle_is_valid(handle));
	}
	_, status = e.custom_shape_add_raw(&world, e.Shape_Type_ID(p.SPHERE_TYPE_ID), &value, size_of(value));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	_, status = e.custom_shape_add_raw(&world, id, nil, size_of(value));
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	storage: [size_of(value)+1]u8;
	mem.copy(&storage[1], &value, size_of(value));
	for i in 0 ..< 12
	{
		handle: e.Shape_Handle;
		add_status: e.Status;
		handle, add_status = e.custom_shape_add_raw(&world, id, &storage[1], size_of(value));
		testing.expect_value(t, add_status, e.Status.Ok);
		data: e.Shape_Access_Value;
		get_status: e.Status;
		data, get_status = e.custom_shape_data(&world, handle);
		testing.expect_value(t, get_status, e.Status.Ok);
		testing.expect_value(t, data.type_id, i32(id));
		testing.expect_value(t, data.size, size_of(value));
		testing.expect_value(t, data.alignment, 128);
		testing.expect_value(t, uintptr(data.data) % 128, uintptr(0));
		testing.expect_value(t, (^Custom_Sphere)(data.data)^, value);
		_ = i;
	}
}

@(test)
custom_shape_raw_get_rejects_builtin_missing_and_disposed_payloads :: proc(t: ^testing.T)
{
	world: e.World;
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	handle: e.Shape_Handle;
	ok: e.Status;
	handle, _, ok = contextual_sphere_add(t, &world, &state);
	if ok != .Ok
	{
		_ = e.world_destroy(&world);
		return;
	}
	data: e.Shape_Access_Value;
	status: e.Status;
	data, status = e.custom_shape_data(&world, handle);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect(t, data.data != nil);
	built_in: e.Shape_Handle;
	add_status: e.Status;
	built_in, add_status = e.shape_add(&world, e.sphere(1));
	testing.expect_value(t, add_status, e.Status.Ok);
	data, status = e.custom_shape_data(&world, built_in);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect(t, data.data == nil);
	testing.expect_value(t, e.shape_remove(&world, handle), e.Status.Ok);
	data, status = e.custom_shape_data(&world, handle);
	testing.expect_value(t, status, e.Status.Not_Found);
	testing.expect(t, data.data == nil);
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	data, status = e.custom_shape_data(&world, handle);
	testing.expect_value(t, status, e.Status.Disposed);
	testing.expect(t, data.data == nil);
}

@(test)
custom_shape_instance_inertia_preserves_native_and_contextual_callbacks :: proc(t: ^testing.T)
{
	world: e.World;
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	state: Contextual_Shape_State = Contextual_Shape_State{scale=1};
	native_type: e.Shape_Type_ID;
	native_status: e.Status;
	native_type, native_status = e.custom_shape_register(&world, e.custom_shape_registration(
			Custom_Sphere, .Convex, custom_sphere_bounds, custom_sphere_inertia, custom_sphere_ray, custom_sphere_support));
	testing.expect_value(t, native_status, e.Status.Ok);
	contextual_type: e.Shape_Type_ID;
	contextual_status: e.Status;
	contextual_type, contextual_status = e.custom_shape_register_contextual(&world, contextual_sphere_registration(&state));
	testing.expect_value(t, contextual_status, e.Status.Ok);
	for id in ([2]e.Shape_Type_ID{native_type, contextual_type})
	{
		value: Custom_Sphere = Custom_Sphere{2};
		handle: e.Shape_Handle;
		add_status: e.Status;
		handle, add_status = e.custom_shape_add_raw(&world, id, &value, size_of(value));
		testing.expect_value(t, add_status, e.Status.Ok);
		inertia: e.Body_Inertia;
		status: e.Status;
		inertia, status = e.custom_shape_inertia(&world, handle, 3);
		expected: p.Body_Inertia;
		expected_status: p.Physics_Status;
		expected, expected_status = p.sphere_inertia(e.sphere(2), 3);
		testing.expect_value(t, expected_status, e.Status.Ok);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, inertia, expected);
	}
	state.failure = .Invalid_Description;
	value: Custom_Sphere = Custom_Sphere{1};
	handle: e.Shape_Handle;
	handle, _ = e.custom_shape_add_raw(&world, contextual_type, &value, size_of(value));
	inertia: e.Body_Inertia;
	status: e.Status;
	inertia, status = e.custom_shape_inertia(&world, handle, 1);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	testing.expect_value(t, inertia, e.Body_Inertia{});
	state.failure = .Ok;
}
