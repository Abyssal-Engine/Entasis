package public_api_tests

import "core:mem"
import "core:simd"
import "core:sync"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"

Contextual_Constraint_State :: struct
{
	body_count: int,
	bias: f32,
	validation_error: e.Status,
	validations: i32,
	phases: [4]i32,
	full_bundles, partial_bundles: i32,
	misaligned: i32,
}

contextual_constraint_validate_test :: proc "contextless" (
	user_context: rawptr, type_id: i32, description: rawptr,
) -> e.Status
{
	state := (^Contextual_Constraint_State)(user_context);
	sync.atomic_add_explicit(&state.validations, 1, .Relaxed);
	if state.validation_error != .Ok
	{
		return state.validation_error;
	}
	return velocity_target_validate(type_id, description);
}

contextual_constraint_kernel_test :: proc "contextless" (
	user_context, prestep_raw: rawptr, bodies: ^[4]e.Constraint_Kernel_Body_Wide,
	dt, inverse_dt: f32, impulses_raw: rawptr, active_mask: ^e.I32x8,
	phase: e.Constraint_Kernel_Phase,
)
{
	_ = dt;
	_ = inverse_dt;
	state := (^Contextual_Constraint_State)(user_context);
	sync.atomic_add_explicit(&state.phases[int(phase)], 1, .Relaxed);
	if uintptr(prestep_raw)%32 != 0 || uintptr(bodies)%32 != 0 || uintptr(impulses_raw)%32 != 0
	{
		sync.atomic_add_explicit(&state.misaligned, 1, .Relaxed);
	}
	active_count := 0;
	for lane in 0 ..< 8
	{
		if simd.extract(active_mask^, lane) != 0
		{
			active_count += 1;
		}
	}
	if active_count == 8
	{
		sync.atomic_add_explicit(&state.full_bundles, 1, .Relaxed);
	}
	else
	{
		sync.atomic_add_explicit(&state.partial_bundles, 1, .Relaxed);
	}
	if phase != .Solve
	{
		return;
	}
	prestep := (^Velocity_Target_Prestep)(prestep_raw);
	impulses := (^Velocity_Target_Impulses)(impulses_raw);
	for lane in 0 ..< 8
	{
		if simd.extract(active_mask^, lane) == 0
		{
			continue;
		}
		for body_index in 0 ..< state.body_count
		{
			if simd.extract(bodies[body_index].inverse_mass, lane) > 0
			{
				bodies[body_index].linear_velocity.x = simd.replace(bodies[body_index].linear_velocity.x, lane,
					simd.extract(prestep.target_speed, lane) + state.bias + f32(body_index));
			}
		}
		impulses.accumulated = simd.replace(impulses.accumulated, lane, simd.extract(prestep.target_speed, lane));
	}
}

contextual_constraint_registration_test :: proc "contextless" (
	state: ^Contextual_Constraint_State, id := e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID,
) -> e.Contextual_Custom_Constraint_Registration
{
	access: [4]e.Body_Access_Mask;
	for i in 0 ..< state.body_count
	{
		access[i] = e.BODY_ACCESS_NO_POSE;
	}
	return e.custom_constraint_registration_contextual(
		Velocity_Target_Constraint, Velocity_Target_Prestep, Velocity_Target_Impulses,
		id, state.body_count, access, access, state,
		contextual_constraint_validate_test, contextual_constraint_kernel_test,
	);
}

@(test)
contextual_constraints_run_arities_full_tail_substeps_and_independent_worlds :: proc(t: ^testing.T)
{
	for arity in 1 ..= 4
	{
		for workers in ([2]i32{1, 2})
		{
			for bias in ([2]f32{-3, 7})
			{
				world: e.World;
				state := Contextual_Constraint_State{body_count=arity, bias=bias};
				description := small_world_description();
				description.gravity = {};
				description.threading.worker_count = workers;
				description.solve.substeps = 2;
				if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
				{
					return;
				}
				defer _ = e.world_destroy(&world);
				registration := contextual_constraint_registration_test(&state);
				if !testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Ok)
				{
					return;
				}
				// the stored table is independent of subsequent caller changes
				registration.kernel = nil;
				registration.user_context = nil;
				bodies: [36]e.Body_Handle;
				constraints: [9]e.Constraint_Handle;
				for ci in 0 ..< 9
				{
					for bi in 0 ..< arity
					{
						body, status := e.body_add(&world, custom_constraint_body_description(
								{f32(ci*arity+bi)*3, 0, 0}, e.body_activity(-1, 255)));
						if !testing.expect_value(t, status, e.Status.Ok)
						{
							return;
						}
						bodies[ci*arity+bi] = body;
					}
					target := Velocity_Target_Constraint{target_speed=f32(ci+1)};
					constraint, status := e.custom_constraint_add_typed(&world,
						bodies[ci*arity:(ci+1)*arity], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &target);
					if !testing.expect_value(t, status, e.Status.Ok)
					{
						return;
					}
					constraints[ci] = constraint;
				}
				if !testing.expect_value(t, e.world_step(&world, 1.0/60.0), e.Status.Ok)
				{
					return;
				}
				for ci in 0 ..< 9
				{
					for bi in 0 ..< arity
					{
						body, status := e.body_get(&world, bodies[ci*arity+bi]);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect_value(t, body.velocity.linear.x, f32(ci+1)+bias+f32(bi));
					}
					stored: Velocity_Target_Constraint;
					testing.expect_value(t, e.custom_constraint_get_typed(&world, constraints[ci], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &stored), e.Status.Ok);
					testing.expect_value(t, stored.target_speed, f32(ci+1));
				}
				for calls in state.phases
				{
					testing.expect(t, calls > 0);
				}
				testing.expect(t, state.full_bundles > 0 && state.partial_bundles > 0);
				testing.expect_value(t, state.misaligned, i32(0));
				// removal compacts a lane without losing the moved description
				testing.expect_value(t, e.constraint_remove(&world, constraints[0]), e.Status.Ok);
				stored: Velocity_Target_Constraint;
				testing.expect_value(t, e.custom_constraint_get_typed(&world, constraints[8], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &stored), e.Status.Ok);
				testing.expect_value(t, stored.target_speed, f32(9));
			}
		}
	}
}

@(test)
contextual_constraints_fallback_sleeping_apply_and_awaken_validation :: proc(t: ^testing.T)
{
	world: e.World;
	state := Contextual_Constraint_State{body_count=1};
	description := small_world_description();
	description.gravity = {};
	description.solve.fallback_batch_threshold = 1;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, contextual_constraint_registration_test(&state)), e.Status.Ok);
	body, status := e.body_add(&world, custom_constraint_body_description({}, e.body_activity(10, 1)));
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	constraints: [3]e.Constraint_Handle;
	for &constraint in constraints
	{
		constraint, status = e.custom_constraint_add_typed(&world, []e.Body_Handle{body}, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &Velocity_Target_Constraint{});
		if !testing.expect_value(t, status, e.Status.Ok)
		{
			return;
		}
	}
	for _ in 0 ..< 4
	{
		if !testing.expect_value(t, e.world_step(&world, 1.0/60.0), e.Status.Ok)
		{
			return;
		}
	}
	sleeping, _ := e.body_is_sleeping(&world, body);
	testing.expect(t, sleeping);
	stored: Velocity_Target_Constraint;
	testing.expect_value(t, e.custom_constraint_get_typed(&world, constraints[2], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &stored), e.Status.Ok);
	testing.expect_value(t, stored.target_speed, f32(0));
	state.validation_error = .Invalid_Description;
	testing.expect_value(t, e.custom_constraint_apply_typed(&world, constraints[2], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &Velocity_Target_Constraint{2}), e.Status.Invalid_Description);
	testing.expect_value(t, e.body_awaken(&world, body), e.Status.Invalid_Description);
	state.validation_error = .Ok;
	testing.expect_value(t, e.custom_constraint_apply_typed(&world, constraints[2], e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID, &Velocity_Target_Constraint{2}), e.Status.Ok);
	testing.expect_value(t, e.body_awaken(&world, body), e.Status.Ok);
	if !testing.expect_value(t, e.world_step(&world, 1.0/60.0), e.Status.Ok)
	{
		return;
	}
	body_state, _ := e.body_get(&world, body);
	testing.expect_value(t, body_state.velocity.linear.x, f32(2));
	testing.expect(t, state.validations > 3 && state.partial_bundles > 0);
}

@(test)
contextual_constraints_registration_rejects_invalid_storage_callbacks_and_ids :: proc(t: ^testing.T)
{
	world: e.World;
	state := Contextual_Constraint_State{body_count=1};
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	registration := contextual_constraint_registration_test(&state);
	for variant in 0 ..< 15
	{
		bad := registration;
		switch variant
		{
			case 0: bad.type_id = e.Constraint_Type_ID(0);
			case 1: bad.type_id = e.Constraint_Type_ID(e.MAXIMUM_CONSTRAINT_TYPE_COUNT);
			case 2: bad.body_count = 0;
			case 3: bad.body_count = 5;
			case 4: bad.description_size = 0;
			case 5: bad.description_size = e.MAXIMUM_CUSTOM_DESCRIPTION_BYTES+4;
			case 6: bad.description_size = 3;
			case 7: bad.prestep_bundle_size = 1;
			case 8: bad.impulse_bundle_size = 0;
			case 9: bad.impulse_bundle_size = (e.MAXIMUM_INACTIVE_IMPULSE_SCALARS+1)*32;
			case 10: bad.validate_description = nil;
			case 11: bad.kernel = nil;
			case 12: bad.incremental_kernel = nil;
			case 13: bad.initial_access[1] = e.BODY_ACCESS_ALL;
			case 14: bad.solve_access[0] = transmute(e.Body_Access_Mask)u8(128);
		}
		testing.expect_value(t, e.custom_constraint_register_contextual(&world, bad), e.Status.Invalid_Description);
		id, _ := e.custom_constraint_next_type_id(&world);
		testing.expect_value(t, id, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID);
	}
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Ok);
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Invalid_Argument);
	_, ok := register_velocity_target_constraint(t, &world);
	testing.expect(t, ok);
	testing.expect_value(t, e.world_step(&world, 1.0/60.0), e.Status.Ok);
	registration.type_id += 2;
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Invalid_Argument);
}

@(test)
contextual_constraints_bindings_copy_release_and_preserve_capacity_on_failure :: proc(t: ^testing.T)
{
	tracker: mem.Tracking_Allocator;
	mem.tracking_allocator_init(&tracker, context.allocator);
	defer mem.tracking_allocator_destroy(&tracker);
	owner := Contextual_Fallible_Allocator{backing=mem.tracking_allocator(&tracker)};
	description := small_world_description();
	description.allocator = {procedure=contextual_fallible_allocator, data=&owner};
	world: e.World;
	state := Contextual_Constraint_State{body_count=1};
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	defer _ = e.world_destroy(&world);
	initial := tracker.current_memory_allocated;
	registration := contextual_constraint_registration_test(&state);
	owner.failure = .Reject;
	testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Capacity_Missing);
	testing.expect_value(t, tracker.current_memory_allocated, initial);
	id, _ := e.custom_constraint_next_type_id(&world);
	testing.expect_value(t, id, e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID);
	owner.failure = .Allow;
	for type_id in int(e.FIRST_CUSTOM_CONSTRAINT_TYPE_ID) ..< e.MAXIMUM_CONSTRAINT_TYPE_COUNT
	{
		registration.type_id = e.Constraint_Type_ID(type_id);
		testing.expect_value(t, e.custom_constraint_register_contextual(&world, registration), e.Status.Ok);
	}
	_, capacity := e.custom_constraint_next_type_id(&world);
	testing.expect_value(t, capacity, e.Status.Capacity_Missing);
	testing.expect_value(t, tracker.current_memory_allocated-initial, i64(40*8));
	testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
	_, capacity = e.custom_constraint_next_type_id(&world);
	testing.expect_value(t, capacity, e.Status.Capacity_Missing);
	owner.failure = .Reject;
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, tracker.current_memory_allocated, i64(0));
	testing.expect_value(t, len(tracker.allocation_map), 0);
	testing.expect_value(t, owner.failed, 1);
}

@(test)
contextual_constraint_record_overlay_preserves_native_layout :: proc(t: ^testing.T)
{
	testing.expect_value(t, size_of(p.Constraint_Type_Record), 96);
	testing.expect_value(t, align_of(p.Constraint_Type_Record), 8);
	testing.expect_value(t, offset_of(p.Constraint_Type_Record, contextual), offset_of(p.Constraint_Type_Record, validate_description));
	testing.expect_value(t, offset_of(p.Constraint_Type_Record, prestep_warmstart_solve), 56);
	testing.expect_value(t, offset_of(p.Constraint_Type_Record, incrementally_update), 64);
	testing.expect_value(t, offset_of(p.Constraint_Type_Record, registration), 88);
	testing.expect_value(t, size_of(p.Constraint_Kernel_Body_Wide), 640);
	testing.expect_value(t, align_of(p.Constraint_Kernel_Body_Wide), 32);
}
