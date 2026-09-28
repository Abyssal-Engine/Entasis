package public_api_tests

import "core:math"
import "core:log"
import "base:runtime"
import "core:simd"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Joint_Break_Manual_Drain :: enum u8
{
	Wrapper,
	Parent_First,
	Break_First,
}

@(test)
mixed_parts_breaks_complete_step_once :: proc(t: ^testing.T)
{
	for mode in ([3]Joint_Break_Manual_Drain{.Wrapper, .Parent_First, .Break_First})
	{
		scene: Trigger_Test_Scene;
		if mixed_test_scene(t, &scene, 2) != .Ok
		{
			return;
		}
		defer e.world_destroy(&scene.world);
		testing.expect_value(t, e.body_set_pose(&scene.world, scene.body, e.pose({1.5, 0, 0})), e.Status.Ok);
		testing.expect_value(t, e.world_enable_joint_breaks(&scene.world, 8), e.Status.Ok);
		joints: [2]e.Constraint_Handle;
		for &joint, index in joints
		{
			body: e.Body_Handle;
			status: e.Status;
			body, status = e.body_add(&scene.world, custom_constraint_body_description({10+f32(index)*4, 5, 0}, e.body_activity(-1, 255)));
			testing.expect_value(t, status, e.Status.Ok);
			joint, status = e.constraint_add_1(&scene.world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, e.constraint_set_break_limits(&scene.world, joint, {metrics={.Force}, force=0, user_id=u64(index+7)}), e.Status.Ok);
		}
		tracker: e.Contact_Tracker;
		testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
		defer e.contact_tracker_destroy(&tracker);
		testing.expect_value(t, e.contact_tracker_bind(&tracker, &scene.world), e.Status.Ok);
		simulation: ^e.Simulation;
		simulation, _ = e.world_borrow_simulation(&scene.world);
		stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
		parents: [8]e.Trigger_Event;
		parts: [8]e.Collider_Part_Event;
		contacts: [8]e.Contact_Event;
		breaks: [8]e.Joint_Break_Event;
		result: e.Joint_Break_Update_Result;
		status: e.Status;
		result, status = e.joint_break_stepper_update(&stepper, &scene.world, 1.0/64, nil, tracker=&tracker);
		testing.expect_value(t, status, e.Status.Capacity_Missing);
		testing.expect_value(t, result.completed_steps, i32(1));
		testing.expect_value(t, result.pending, e.Joint_Break_Pending_Streams{.Parent, .Part, .Contact, .Break});
		testing.expect_value(t, result.required, i32(1));
		testing.expect_value(t, result.part_required, i32(1));
		testing.expect_value(t, result.contact_required, i32(1));
		testing.expect_value(t, result.break_required, i32(2));
		testing.expect_value(t, stepper.accumulator, f32(0));
		testing.expect_value(t, simulation.step_index, u64(1));
		testing.expect_value(t, tracker.last_step_index, u64(0));
		parent_total, part_total, contact_total, break_total: int;
		written, required: int;
		if mode == .Parent_First
		{
			written, required, status = e.trigger_events_drain(&scene.world, parents[:]);
			testing.expect_value(t, status, e.Status.Ok);
			parent_total += written;
			written, required, status = e.trigger_part_events_drain(&scene.world, parts[:]);
			testing.expect_value(t, status, e.Status.Ok);
			part_total += written;
		}
		else if mode == .Break_First
		{
			written, required, status = e.constraint_break_events_drain(&scene.world, breaks[:]);
			testing.expect_value(t, status, e.Status.Ok);
			break_total += written;
			written, required, status = e.contact_events_drain(&tracker, contacts[:]);
			testing.expect_value(t, status, e.Status.Ok);
			contact_total += written;
		}
		else
		{
			result, status = e.joint_break_stepper_update(&stepper, &scene.world, 0, breaks[:1], nil, parts[:], &tracker, contacts[:]);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, result.completed_steps, i32(0));
			testing.expect_value(t, result.contact_events_written, i32(1));
			testing.expect_value(t, result.pending, e.Joint_Break_Pending_Streams{.Parent, .Part, .Break});
			testing.expect_value(t, result.break_required, i32(2));
			contact_total += int(result.contact_events_written);
		}
		result, status = e.joint_break_stepper_update(&stepper, &scene.world, 0, breaks[:], parents[:], parts[:], &tracker, contacts[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.completed_steps, i32(0));
		testing.expect_value(t, result.pending, e.Joint_Break_Pending_Streams{});
		parent_total += int(result.events_written);
		part_total += int(result.part_events_written);
		contact_total += int(result.contact_events_written);
		break_total += int(result.break_events_written);
		testing.expect_value(t, parent_total, 1);
		testing.expect_value(t, part_total, 1);
		testing.expect_value(t, contact_total, 1);
		testing.expect_value(t, break_total, 2);
		testing.expect_value(t, simulation.step_index, u64(1));
		testing.expect_value(t, tracker.last_step_index, u64(1));
		testing.expect_value(t, stepper.accumulator, f32(0));
		testing.expect_value(t, parents[0].step, u64(1));
		testing.expect_value(t, parts[0].step, u64(1));
		for event, index in breaks[:break_total]
		{
			testing.expect_value(t, event.constraint, joints[index]);
			testing.expect_value(t, event.user_id, u64(index+7));
			testing.expect_value(t, event.reaction.step, u64(1));
		}
		state: e.Body_State;
		state, status = e.body_get(&scene.world, scene.body);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect(t, state.velocity.linear.x>0);
		result, status = e.joint_break_stepper_update(&stepper, &scene.world, 0, nil, tracker=&tracker);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, result.completed_steps+result.events_written+result.part_events_written+result.contact_events_written+result.break_events_written, i32(0));
	}
	world: e.World;
	testing.expect_value(t, e.world_init(&world, small_world_description()), e.Status.Ok);
	defer e.world_destroy(&world);
	testing.expect_value(t, e.world_enable_joint_breaks(&world, 8), e.Status.Ok);
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	parents: [1]e.Trigger_Event;
	result: e.Joint_Break_Update_Result;
	status: e.Status;
	result, status = e.joint_break_stepper_update(&stepper, &world, 1.0/64, nil, parents[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, stepper.accumulator, f32(0));
	result, status = e.joint_break_stepper_update(&stepper, &world, 1.0/64, nil);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
}

@(test)
mixed_parts_breaks_failed_step_keeps_history :: proc(t: ^testing.T)
{
	scene: Trigger_Test_Scene;
	if mixed_test_scene(t, &scene, 2) != .Ok
	{
		return;
	}
	defer e.world_destroy(&scene.world);
	testing.expect_value(t, e.body_set_pose(&scene.world, scene.body, e.pose({1.5, 0, 0})), e.Status.Ok);
	testing.expect_value(t, e.world_enable_joint_breaks(&scene.world, 8), e.Status.Ok);
	testing.expect_value(t, e.world_enable_body_control(&scene.world, {input_capacity=8, settings_capacity=8}), e.Status.Ok);
	testing.expect_value(t, e.world_enable_restitution(&scene.world, {fallback={coefficient=0.5, threshold=0}, collidable_capacity=8, pair_capacity=8}), e.Status.Ok);
	body: e.Body_Handle;
	status: e.Status;
	body, status = e.body_add(&scene.world, custom_constraint_body_description({10, 5, 0}, e.body_activity(-1, 255)));
	testing.expect_value(t, status, e.Status.Ok);
	joint: e.Constraint_Handle;
	joint, status = e.constraint_add_1(&scene.world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, e.constraint_set_break_limits(&scene.world, joint, {metrics={.Force}, force=1e20}), e.Status.Ok);
	tracker: e.Contact_Tracker;
	testing.expect_value(t, e.contact_tracker_init(&tracker, 8), e.Status.Ok);
	defer e.contact_tracker_destroy(&tracker);
	testing.expect_value(t, e.contact_tracker_bind(&tracker, &scene.world), e.Status.Ok);
	stepper: e.Fixed_Stepper = e.fixed_stepper(1.0/64, 4);
	parents: [8]e.Trigger_Event;
	parts: [8]e.Collider_Part_Event;
	contacts: [8]e.Contact_Event;
	breaks: [8]e.Joint_Break_Event;
	result: e.Joint_Break_Update_Result;
	result, status = e.joint_break_stepper_update(&stepper, &scene.world, 1.0/64, breaks[:], parents[:], parts[:], &tracker, contacts[:]);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.events_written, i32(1));
	testing.expect_value(t, result.part_events_written, i32(1));
	testing.expect_value(t, result.contact_events_written, i32(1));
	reaction: e.Joint_Reaction;
	reaction, status = e.constraint_reaction(&scene.world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&scene.world);
	parent_count: int = simulation.triggers.previous_count;
	part_count: int = simulation.triggers.mixed.previous_count;
	restitution_table: u8 = simulation.restitution.committed;
	saved: p.Timestepper = simulation.timestepper;
	simulation.timestepper = {step=joint_break_failed_step_callback};
	testing.expect_value(t, e.constraint_set_break_limits(&scene.world, joint, {metrics={.Force}, force=0}), e.Status.Ok);
	testing.expect_value(t, e.body_add_force(&scene.world, body, {1, 0, 0}), e.Status.Ok);
	result, status = e.joint_break_stepper_update(&stepper, &scene.world, 1.0/64, breaks[:], parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Invalid_Argument);
	testing.expect_value(t, result.completed_steps+result.events_written+result.part_events_written+result.contact_events_written+result.break_events_written, i32(0));
	testing.expect_value(t, stepper.accumulator, f32(1.0/64));
	testing.expect_value(t, simulation.step_index, u64(1));
	testing.expect_value(t, tracker.last_step_index, u64(1));
	testing.expect_value(t, simulation.triggers.previous_count, parent_count);
	testing.expect_value(t, simulation.triggers.mixed.previous_count, part_count);
	testing.expect_value(t, simulation.restitution.committed, restitution_table);
	testing.expect_value(t, simulation.body_control.inputs.count, 0);
	testing.expect_value(t, simulation.solver.joint_breaks.phase, p.Joint_Break_Phase.Idle);
	previous: e.Joint_Reaction;
	previous, status = e.constraint_reaction(&scene.world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, previous, reaction);
	_, status = e.constraint_inspect(&scene.world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	simulation.timestepper = saved;
	// failure does not undo executed physics. the caller explicitly chooses recovery
	result, status = e.joint_break_stepper_update(&stepper, &scene.world, 0, breaks[:], parents[:], parts[:], &tracker, contacts[:]);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, result.completed_steps, i32(1));
	testing.expect_value(t, result.break_events_written, i32(1));
	testing.expect_value(t, simulation.step_index, u64(2));
	testing.expect_value(t, stepper.accumulator, f32(0));
}

@(private)
reaction_test_type_batch :: proc "contextless" (simulation: ^p.Simulation, handle: e.Constraint_Handle) -> (^p.Type_Batch, int)
{
	location: p.Constraint_Location;
	location, _ = p.solver_resolve(&simulation.solver, handle);
	batch: ^p.Constraint_Batch = &simulation.solver.active_set.batches.memory[location.batch_index];
	return &batch.type_batches.memory[batch.type_id_to_batch_index[location.type_id]], int(location.index_in_type_batch);
}

@(test)
joint_reaction_reads_actual_lanes_without_world_mutation :: proc(t: ^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&world);
	bodies: [17]e.Body_Handle;
	joints: [17]e.Constraint_Handle;
	for index in 0 ..< len(bodies)
	{
		bodies[index], _ = e.body_add(&world, custom_constraint_body_description({f32(index)*4, 0, 0}, e.body_activity(-1, 255)));
		settings: e.Motor_Settings = e.motor_settings(10000, 0.01);
		joints[index], _ = e.constraint_add_1(&world, bodies[index], e.One_Body_Linear_Motor{{1, 0, 0}, {}, settings});
		batch: ^p.Type_Batch;
		lane_index: int;
		batch, lane_index = reaction_test_type_batch(simulation, joints[index]);
		vectors: [^]util.F32x8 = ([^]util.F32x8)(p.type_batch_impulse_bundle(batch, lane_index));
		vectors[0] = simd.replace(vectors[0], lane_index%8, 0);
		vectors[1] = simd.replace(vectors[1], lane_index%8, f32(index+1));
		vectors[2] = simd.replace(vectors[2], lane_index%8, 2);
	}
	for index in 0 ..< len(joints)
	{
		before: e.Body_Description;
		before, _ = e.body_get(&world, bodies[index]);
		sample: p.Constraint_Reaction_Sample;
		testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[index], 0.25, {}, &sample), e.Status.Ok);
		testing.expect_value(t, sample.body_count, i32(1));
		testing.expect_value(t, sample.bodies[0], bodies[index]);
		testing.expect_value(t, sample.forces[0].linear, e.Vector3{0, f32(index+1)*4, 8});
		testing.expect_value(t, sample.forces[0].angular, e.Vector3{0, -8, f32(index+1)*4});
		testing.expect(t, abs(sample.maximum_force-math.sqrt(f32((index+1)*(index+1)*16+64)))<1e-5);
		testing.expect_value(t, sample.maximum_torque, sample.maximum_force);
		after: e.Body_Description;
		after, _ = e.body_get(&world, bodies[index]);
		testing.expect_value(t, after, before);
		impulses: [3]f32;
		status: e.Status;
		_, _, status = e.constraint_accumulated_impulses(&world, joints[index], impulses[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, impulses, ([3]f32{0, f32(index+1), 2}));
	}
	for mask in 1 ..< 256
	{
		grouped: [8]p.Constraint_Reaction_Sample;
		testing.expect_value(t, p.constraint_capture_reaction_bundle(&simulation.solver, joints[4], u8(mask), 0.25, {}, &grouped), e.Status.Ok);
		for lane in 0 ..< 8
		{
			if mask & (1<<u32(lane)) != 0
			{
				scalar: p.Constraint_Reaction_Sample;
				testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[lane], 0.25, {}, &scalar), e.Status.Ok);
				testing.expect_value(t, grouped[lane], scalar);
			}
			else
			{
				testing.expect_value(t, grouped[lane], p.Constraint_Reaction_Sample{});
			}
		}
	}
	tail: [8]p.Constraint_Reaction_Sample;
	testing.expect_value(t, p.constraint_capture_reaction_bundle(&simulation.solver, joints[16], 1, 0.25, {}, &tail), e.Status.Ok);
	testing.expect_value(t, tail[0].forces[0].linear, e.Vector3{0, 68, 8});
	previous_tail: [8]p.Constraint_Reaction_Sample =tail;
	testing.expect_value(t, p.constraint_capture_reaction_bundle(&simulation.solver, joints[16], 2, 0.25, {}, &tail), e.Status.Invalid_Argument);
	testing.expect_value(t, tail, previous_tail);
	sample: p.Constraint_Reaction_Sample = {body_count=77};
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[0], 0, {}, &sample), e.Status.Invalid_Argument);
	testing.expect_value(t, sample.body_count, i32(77));
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[0], math.nan_f32(), {}, &sample), e.Status.Invalid_Argument);
	testing.expect_value(t, sample.body_count, i32(77));
	testing.expect_value(t, e.constraint_remove(&world, joints[0]), e.Status.Ok);
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[0], 1, {}, &sample), e.Status.Not_Found);
	testing.expect_value(t, sample.body_count, i32(77));
}

Reaction_Provider_Output_Mode :: enum u8
{
	Normal,
	Nonfinite,
}

Reaction_Provider_Test :: struct
{
	calls: int,
	status: e.Status,
	expected_speed: f32,
	output_mode: Reaction_Provider_Output_Mode,
}

reaction_test_provider :: proc "contextless" (
	user_context: rawptr, input: ^p.Constraint_Reaction_Provider_Input,
	output: ^[4]p.Constraint_Impulse_Wrench,
) -> e.Status
{
	state: ^Reaction_Provider_Test = (^Reaction_Provider_Test)(user_context);
	state.calls+=1;
	if input.body_count != 1 || input.impulse_count != 1 || input.description_size != size_of(Velocity_Target_Constraint) ||
	(^Velocity_Target_Constraint)(input.description).target_speed != state.expected_speed || input.substep_duration <= 0
	{
		return .Invalid_Description;
	}
	output[0].linear.x = input.impulses[0];
	if state.output_mode == .Nonfinite
	{
		output[0].linear.x=math.inf_f32(1);
	}
	return state.status;
}

@(test)
joint_reaction_custom_provider_explicit_and_failure_atomic :: proc(t: ^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	world: e.World;
	if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
	{
		return;
	}
	defer e.world_destroy(&world);
	type_id: e.Constraint_Type_ID;
	ok: bool;
	type_id, ok = register_velocity_target_constraint(t, &world);
	if !ok
	{
		return;
	}
	body: e.Body_Handle;
	body, _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
	bodies: [1]e.Body_Handle={body};
	custom: Velocity_Target_Constraint={target_speed=3};
	joint: e.Constraint_Handle;
	status: e.Status;
	joint, status = e.custom_constraint_add_typed(&world, bodies[:], type_id, &custom);
	if !testing.expect_value(t, status, e.Status.Ok)
	{
		return;
	}
	testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&world);
	sample: p.Constraint_Reaction_Sample={body_count=77};
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 0.125, {}, &sample), e.Status.Invalid_Argument);
	testing.expect_value(t, sample.body_count, i32(77));
	state: Reaction_Provider_Test={expected_speed=3};
	provider: p.Constraint_Reaction_Provider={read=reaction_test_provider, user_context=&state};
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 0.125, provider, &sample), e.Status.Ok);
	testing.expect_value(t, sample.forces[0].linear, e.Vector3{24, 0, 0});
	previous: p.Constraint_Reaction_Sample = sample;
	state.status=.Capacity_Missing;
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 0.125, provider, &sample), e.Status.Capacity_Missing);
	testing.expect_value(t, sample, previous);
	state.status=.Ok;
	state.output_mode=.Nonfinite;
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 0.125, provider, &sample), e.Status.Invalid_Argument);
	testing.expect_value(t, sample, previous);
	testing.expect_value(t, state.calls, 3);
	// sleep the connected island through its authoritative owner, not the
	// independent-body convenience API (which correctly rejects joints)
	sleeper: ^p.Island_Sleeper = &simulation.sleeper;
	location: p.Body_Memory_Location;
	location, _ = p.bodies_resolve(&simulation.bodies, body);
	simulation.bodies.sets.memory[0].activity.memory[location.index].sleep_candidate=.Candidate;
	testing.expect_value(t, p.island_scaffold_clear(&sleeper.scaffold), e.Status.Ok);
	body_count: int;
	constraint_count: int;
	can_sleep: p.Reference_State;
	build_status: p.Physics_Status;
	body_count, constraint_count, can_sleep, build_status = p.island_sleeper_build_island(sleeper, &sleeper.scaffold, body);
	testing.expect_value(t, build_status, e.Status.Ok);
	testing.expect_value(t, can_sleep, p.Reference_State.Present);
	testing.expect_value(t, p.island_sleeper_sleep_island(sleeper, &sleeper.scaffold, body_count, constraint_count), e.Status.Ok);
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joint, 0.125, provider, &sample), e.Status.Not_Found);
	testing.expect_value(t, state.calls, 3);
	testing.expect_value(t, sample, previous);
}

@(test)
joint_reactions_read_fallback_and_reuse_without_allocation :: proc(t: ^testing.T)
{
	tracker: allocation_test_tracker;
	description: e.World_Description = small_world_description();
	description.allocator=allocation_test_allocator(&tracker);
	description.gravity={};
	description.damping={};
	description.solve.fallback_batch_threshold=1;
	world: e.World;
	if !testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok)
	{
		return;
	}
	body: e.Body_Handle;
	body, _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
	joints: [11]e.Constraint_Handle;
	for &joint in joints
	{
		joint, _=e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{1, 0, 0}, {}, e.motor_settings(10000, 0.01)});
	}
	simulation: ^p.Simulation;
	simulation, _ =e.world_borrow_simulation(&world);
	for index in 0 ..< len(joints)
	{
		batch: ^p.Type_Batch;
		location: int;
		batch, location =reaction_test_type_batch(simulation, joints[index]);
		if index>0
		{
			resolved: p.Constraint_Location;
			resolved, _ =p.solver_resolve(&simulation.solver, joints[index]);
			testing.expect_value(t, resolved.batch_index, i32(description.solve.fallback_batch_threshold));
		}
		impulse: ^util.Vector3_Wide=(^util.Vector3_Wide)(p.type_batch_impulse_bundle(batch, location));
		util.vector3_wide_write_slot(impulse, location%8, {0, f32(index+1), 0});
	}
	requests: int =tracker.requests;
	tracker.fail_from=requests+1;
	for _ in 0 ..< 100
	{
		for index in 0 ..< len(joints)
		{
			for duration in ([3]f32{0.125, 0.25, 0.5})
			{
				sample:p.Constraint_Reaction_Sample;
				testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, joints[index], duration, {}, &sample), e.Status.Ok);
				testing.expect_value(t, sample.forces[0].linear.y, f32(index+1)/duration);
				testing.expect_value(t, sample.forces[0].angular.z, f32(index+1)/duration);
			}
		}
	}
	testing.expect_value(t, tracker.requests, requests);
	tracker.fail_from=0;
	removed: e.Constraint_Handle =joints[3];
	testing.expect_value(t, e.constraint_remove(&world, removed), e.Status.Ok);
	replacement: e.Constraint_Handle;
	status: e.Status;
	replacement, status =e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {}, e.motor_settings(10000, 0.01)});
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, replacement, removed);
	sample:p.Constraint_Reaction_Sample;
	testing.expect_value(t, p.constraint_capture_reaction(&simulation.solver, replacement, 0.25, {}, &sample), e.Status.Ok);
	testing.expect_value(t, sample.maximum_force, f32(0));
	testing.expect_value(t, sample.maximum_torque, f32(0));
	requests=tracker.requests;
	tracker.fail_from=requests+1;
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	testing.expect_value(t, tracker.requests, requests);
	allocation_test_empty(t, &tracker);
}

@(test)
joint_reaction_custom_bundle_failure_is_atomic_and_world_local :: proc(t: ^testing.T)
{
	// two worlds deliberately reuse the same type and handle ids. a provider
	// must be selected by the owner, not recovered from a global callback map
	for bias in ([2]f32{0, 20})
	{
		description: e.World_Description =small_world_description();
		description.gravity={};
		description.damping={};
		world:e.World;
		if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
		{
			return;
		}
		defer e.world_destroy(&world);
		type_id: e.Constraint_Type_ID;
		ok: bool;
		type_id, ok =register_velocity_target_constraint(t, &world);
		if !ok
		{
			return;
		}
		joints:[9]e.Constraint_Handle;
		simulation: ^p.Simulation;
		simulation, _ =e.world_borrow_simulation(&world);
		for index in 0..<len(joints)
		{
			body: e.Body_Handle;
			body, _ =e.body_add(&world, custom_constraint_body_description({f32(index)*3, 0, 0}, e.body_activity(-1, 255)));
			bodies:[1]e.Body_Handle={body};
			custom: Velocity_Target_Constraint =Velocity_Target_Constraint{target_speed=3};
			joints[index], _=e.custom_constraint_add_typed(&world, bodies[:], type_id, &custom);
			batch: ^p.Type_Batch;
			location: int;
			batch, location =reaction_test_type_batch(simulation, joints[index]);
			impulses: ^util.F32x8=(^util.F32x8)(p.type_batch_impulse_bundle(batch, location));
			impulses^=simd.replace(impulses^, location%8, bias+f32(index+1));
		}
		state:Reaction_Provider_Test={expected_speed=3};
		provider:p.Constraint_Reaction_Provider={read=reaction_test_provider, user_context=&state};
		output:[8]p.Constraint_Reaction_Sample;
		testing.expect_value(t, p.constraint_capture_reaction_bundle(&simulation.solver, joints[3], 0xff, 0.5, provider, &output), e.Status.Ok);
		testing.expect_value(t, state.calls, 8);
		for lane in 0..<8
		{
			testing.expect_value(t, output[lane].forces[0].linear.x, (bias+f32(lane+1))*2);
		}
		previous: [8]p.Constraint_Reaction_Sample = output;
		batch: ^p.Type_Batch;
		location: int;
		batch, location =reaction_test_type_batch(simulation, joints[3]);
		invalid: Velocity_Target_Constraint =Velocity_Target_Constraint{target_speed=99};
		record: ^p.Constraint_Type_Record =&simulation.solver.registry.records[int(type_id)];
		record.apply_description(&invalid, p.type_batch_prestep_bundle(batch, location), int(type_id), size_of(invalid), int(record.prestep_bundle_size), location%8);
		// the fourth provider fails its descriptor assertion after three lanes
		// succeeded. no part of the candidate output is externally published
		testing.expect_value(t, p.constraint_capture_reaction_bundle(&simulation.solver, joints[0], 0xff, 0.5, provider, &output), e.Status.Invalid_Description);
		testing.expect_value(t, state.calls, 12);
		testing.expect_value(t, output, previous);
	}
}

@(test)
joint_reaction_magnitude_and_conversion_numeric_boundaries :: proc(t: ^testing.T)
{
	testing.expect_value(t, p.constraint_reaction_magnitude({3, 4, 0}), f32(5));
	large: f32 =p.constraint_reaction_magnitude({3e30, 4e30, 0});
	testing.expect(t, abs(large/5e30-1)<1e-6);
	tiny: f32 =p.constraint_reaction_magnitude({3e-30, 4e-30, 0});
	testing.expect(t, abs(tiny/5e-30-1)<1e-6);
	testing.expect_value(t, p.constraint_reaction_magnitude({}), f32(0));
	for type_id in 0..<p.CONSTRAINT_TYPE_ID_CAPACITY
	{
		expected: p.Reference_State =p.constraint_builtin_reaction_support(i32(type_id));
		if type_id>=p.FIRST_CALLER_CONSTRAINT_TYPE_ID
		{
			testing.expect_value(t, expected, p.Reference_State.Missing);
		}
	}
	output:p.Constraint_Reaction_Sample={body_count=99};
	testing.expect_value(t, p.constraint_capture_reaction(nil, {}, -1, {}, &output), e.Status.Invalid_Argument);
	testing.expect_value(t, p.constraint_capture_reaction(nil, {}, math.inf_f32(1), {}, &output), e.Status.Invalid_Argument);
	testing.expect_value(t, p.constraint_capture_reaction(nil, {}, 0.25, {}, &output), e.Status.Not_Found);
	testing.expect_value(t, output.body_count, i32(99));
}

@(test)
joint_break_deferred_watches_and_event_retry :: proc(t:^testing.T)
{
	joint_break_threshold_boundaries(t);
	for workers in ([3]int{1, 2, 4})
	{
		for substeps in ([2]int{1, 4})
		{
			description: e.World_Description = small_world_description();
			description.gravity={};
			description.damping={};
			description.threading.worker_count=i32(workers);
			description.solve=e.solve_description_substeps(substeps, 8);
			world:e.World;
			if !testing.expect_value(t, e.world_init(&world, description), e.Status.Ok)
			{
				continue;
			}
			testing.expect_value(t, e.world_enable_joint_breaks(&world, 17), e.Status.Ok);
			joints:[17]e.Constraint_Handle;
			for i in 0..<17
			{
				body: e.Body_Handle;
				body, _ = e.body_add(&world, custom_constraint_body_description({f32(i)*4, 0, 0}, e.body_activity(-1, 255)));
				joints[i], _=e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(10000, 0.01)});
				testing.expect_value(t, e.constraint_set_break_limits(&world, joints[i], {metrics={.Force}, force=0, user_id=u64(i+1)}), e.Status.Ok);
			}
			testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Ok);
			simulation: ^p.Simulation;
			simulation, _ = e.world_borrow_simulation(&world);
			testing.expect_value(t, simulation.step_index, u64(1));
			for handle in joints
			{
				status: e.Status;
				_, status = e.constraint_inspect(&world, handle);
				testing.expect_value(t, status, e.Status.Not_Found);
			}
			written: int;
			required: int;
			status: e.Status;
			written, required, status = e.constraint_break_events_drain(&world, nil);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, written, 0);
			testing.expect_value(t, required, 17);
			testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Invalid_Argument);
			testing.expect_value(t, simulation.step_index, u64(1));
			events:[17]e.Joint_Break_Event;
			written, required, status=e.constraint_break_events_drain(&world, events[:]);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, written, 17);
			testing.expect_value(t, required, 17);
			for event, index in events
			{
				testing.expect_value(t, event.constraint, joints[index]);
				testing.expect_value(t, event.user_id, u64(index+1));
				testing.expect(t, event.reaction.maximum_force>0);
				testing.expect(t, .Force in event.exceeded);
				testing.expect_value(t, event.reaction.step, u64(1));
				if index>0
				{
					testing.expect(t, event.lifetime>events[index-1].lifetime);
				}
			}
			testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Ok);
			testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		}
	}
}

@(test)
joint_break_cancellation_and_reused_handle :: proc(t:^testing.T)
{
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	world:e.World;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	defer e.world_destroy(&world);
	testing.expect_value(t, e.world_enable_joint_breaks(&world, 1), e.Status.Ok);
	body: e.Body_Handle;
	body, _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
	joint: e.Constraint_Handle;
	joint, _ = e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(10000, 0.01)});
	testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=100000}), e.Status.Ok);
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&world);
	serial: u64 = simulation.solver.joint_breaks.watches.values.memory[0].lifetime;
	testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Ok);
	reaction: e.Joint_Reaction;
	status: e.Status;
	reaction, status = e.constraint_reaction(&world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Solved);
	testing.expect_value(t, e.constraint_remove(&world, joint), e.Status.Ok);
	_, status=e.constraint_get_break_limits(&world, joint);
	testing.expect_value(t, status, e.Status.Not_Found);
	replacement: e.Constraint_Handle;
	replacement, _ = e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {15, 0, 0}, e.motor_settings(10000, 0.01)});
	testing.expect_value(t, replacement, joint);
	testing.expect_value(t, e.constraint_set_break_limits(&world, replacement, {metrics={.Force}, force=0}), e.Status.Ok);
	testing.expect(t, simulation.solver.joint_breaks.watches.values.memory[0].lifetime>serial);
	testing.expect_value(t, e.body_remove(&world, body), e.Status.Ok);
	testing.expect_value(t, simulation.solver.joint_breaks.watches.count, 0);
	testing.expect_value(t, e.world_clear(&world), e.Status.Ok);
	testing.expect_value(t, e.world_disable_joint_breaks(&world), e.Status.Ok);
}

joint_break_failed_step_callback :: proc "contextless"(user:rawptr, simulation:^p.Simulation, dt:f32, dispatcher:^util.Thread_Dispatcher_Boundary)->p.Physics_Status
{
	_=user;
	context=runtime.default_context();
	status: p.Physics_Status = p.simulation_predict_bounding_boxes(simulation, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_collision_detection(simulation, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	status=p.simulation_solve(simulation, dt, dispatcher);
	if status!=.Ok
	{
		return status;
	}
	return .Invalid_Argument;
}

@(test)
joint_break_failed_custom_step_keeps_joints_and_committed_history :: proc(t:^testing.T)
{
	joint_break_scheduled_provider_failure(t);
	description: e.World_Description = small_world_description();
	description.gravity={};
	description.damping={};
	world:e.World;
	testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
	defer e.world_destroy(&world);
	testing.expect_value(t, e.world_enable_joint_breaks(&world), e.Status.Ok);
	body: e.Body_Handle;
	body, _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
	joint: e.Constraint_Handle;
	joint, _ = e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
	testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=0}), e.Status.Ok);
	simulation: ^p.Simulation;
	simulation, _ = e.world_borrow_simulation(&world);
	saved: p.Timestepper = simulation.timestepper;
	simulation.timestepper={step=joint_break_failed_step_callback};
	testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Invalid_Argument);
	testing.expect_value(t, simulation.step_index, u64(0));
	status: e.Status;
	_, status = e.constraint_inspect(&world, joint);
	testing.expect_value(t, status, e.Status.Ok);
	reaction: e.Joint_Reaction;
	reaction_status: e.Status;
	reaction, reaction_status = e.constraint_reaction(&world, joint);
	testing.expect_value(t, reaction_status, e.Status.Ok);
	testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Unsolved);
	written: int;
	required: int;
	drain: e.Status;
	written, required, drain = e.constraint_break_events_drain(&world, nil);
	testing.expect_value(t, drain, e.Status.Ok);
	testing.expect_value(t, written, 0);
	testing.expect_value(t, required, 0);
	simulation.timestepper=saved;
	testing.expect_value(t, e.constraint_clear_break_limits(&world, joint), e.Status.Ok);
	testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Ok);
	_, status=e.constraint_inspect(&world, joint);
	testing.expect_value(t, status, e.Status.Ok);
}

joint_break_threshold_boundaries :: proc (t: ^testing.T)
{
	// binary-exact durations and saturated motors give exactly 32 units of load
	for metric in ([2]e.Joint_Break_Metric{.Force, .Torque})
	{
		for duration in ([2]f32{0.125, 0.25})
		{
			for substeps in ([2]int{1, 4})
			{
				for limit in ([3]f32{31, 32, 33})
				{
					description: e.World_Description = small_world_description();
					description.gravity = {};
					description.damping = {};
					description.solve = e.solve_description_substeps(substeps, 4);
					world: e.World;
					testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
					testing.expect_value(t, e.world_enable_joint_breaks(&world, 1), e.Status.Ok);
					body: e.Body_Handle;
					body, _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
					joint: e.Constraint_Handle;
					if metric == .Force
					{
						joint, _ = e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {1000, 0, 0}, e.motor_settings(32, 0)});
					}
					else
					{
						joint, _ = e.constraint_add_1(&world, body, e.One_Body_Angular_Motor{{1000, 0, 0}, e.motor_settings(32, 0)});
					}
					testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={metric}, force=limit, torque=limit}), e.Status.Ok);
					testing.expect_value(t, e.world_step(&world, duration), e.Status.Ok);
					reaction: e.Joint_Reaction;
					status: e.Status;
					events: [1]e.Joint_Break_Event;
					written, required: int;
					if limit < 32
					{
						written, required, status = e.constraint_break_events_drain(&world, events[:]);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect_value(t, written, 1);
						testing.expect_value(t, events[0].exceeded, e.Joint_Break_Metrics{metric});
						reaction = events[0].reaction;
					}
					else
					{
						reaction, status = e.constraint_reaction(&world, joint);
						testing.expect_value(t, status, e.Status.Ok);
						written, required, status = e.constraint_break_events_drain(&world, events[:]);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect_value(t, written, 0);
					}
					testing.expect_value(t, required, written);
					testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Solved);
					testing.expect_value(t, reaction.sample.substep_duration, duration/f32(substeps));
					testing.expect_value(t, reaction.maximum_force if metric == .Force else reaction.maximum_torque, f32(32));
					if metric == .Force && duration == 0.125 && substeps == 1 && limit == 33
					{
						simulation: ^p.Simulation;
						simulation, _ = e.world_borrow_simulation(&world);
						simulation.sleeper.scaffold.island_bodies.memory[0] = body;
						simulation.sleeper.scaffold.island_constraints.memory[0] = joint;
						testing.expect_value(t, p.island_sleeper_sleep_island(&simulation.sleeper, &simulation.sleeper.scaffold, 1, 1), e.Status.Ok);
						testing.expect_value(t, e.world_step(&world, duration), e.Status.Ok);
						reaction, status = e.constraint_reaction(&world, joint);
						testing.expect_value(t, status, e.Status.Ok);
						testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Unsolved);
						testing.expect_value(t, reaction.maximum_force, f32(0));
					}
					testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
				}
			}
		}
	}
}

Joint_Break_Invalid_Completion :: struct
{
	advance: u64,
}

joint_break_invalid_completion :: proc "contextless" (
	user: rawptr, simulation: ^p.Simulation, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> e.Status
{
	context = runtime.default_context();
	state: ^Joint_Break_Invalid_Completion = (^Joint_Break_Invalid_Completion)(user);
	before: u64 = simulation.step_index;
	status: e.Status = p.default_timestepper_step(nil, simulation, dt, dispatcher);
	if status == .Ok
	{
		simulation.step_index = before + state.advance;
	}
	return status;
}

@(test)
joint_break_custom_success_requires_one_step :: proc (t: ^testing.T)
{
	for advance in ([2]u64{0, 2})
	{
		scene: Restitution_Test_Scene;
		if !testing.expect_value(t, restitution_test_scene(&scene), e.Status.Ok)
		{
			return;
		}
		testing.expect_value(t, e.world_enable_joint_breaks(&scene.world, 1), e.Status.Ok);
		testing.expect_value(t, e.world_enable_body_control(&scene.world), e.Status.Ok);
		testing.expect_value(t, e.world_enable_restitution(&scene.world), e.Status.Ok);
		testing.expect_value(t, e.world_enable_triggers(&scene.world, {pair_capacity=16, candidates_per_worker=16, child_capacity=16}), e.Status.Ok);
		sensor: e.Static_Handle;
		sensor, _ = e.static_add(&scene.world, e.static_body(scene.shape, e.pose({0, 2, 0})), .None);
		reference: e.Collidable_Reference;
		reference, _ = p.collidable_reference_static(sensor);
		testing.expect_value(t, e.trigger_set(&scene.world, reference), e.Status.Ok);
		joint: e.Constraint_Handle;
		joint, _ = e.constraint_add_1(&scene.world, scene.body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
		testing.expect_value(t, e.constraint_set_break_limits(&scene.world, joint, {metrics={.Force}, force=0}), e.Status.Ok);
		testing.expect_value(t, e.body_add_force(&scene.world, scene.body, {1, 0, 0}), e.Status.Ok);
		committed: u8 = scene.simulation.restitution.committed;
		state: Joint_Break_Invalid_Completion = {advance=advance};
		scene.simulation.timestepper = {step=joint_break_invalid_completion, user_context=&state};
		testing.expect_value(t, e.world_step(&scene.world, 0.02), e.Status.Invalid_Argument);
		testing.expect_value(t, scene.simulation.step_index, advance);
		testing.expect_value(t, scene.simulation.body_control.phase, p.Body_Control_Phase.Idle);
		testing.expect_value(t, scene.simulation.restitution.committed, committed);
		testing.expect_value(t, scene.simulation.triggers.previous_count, 0);
		testing.expect_value(t, scene.simulation.triggers.event_count, 0);
		reaction: e.Joint_Reaction;
		status: e.Status;
		reaction, status = e.constraint_reaction(&scene.world, joint);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Unsolved);
		written, required: int;
		written, required, status = e.constraint_break_events_drain(&scene.world, nil);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 0);
		testing.expect_value(t, required, 0);
		scene.simulation.timestepper = p.default_timestepper();
		testing.expect_value(t, e.world_step(&scene.world, 0.02), e.Status.Ok);
		events: [1]e.Joint_Break_Event;
		written, required, status = e.constraint_break_events_drain(&scene.world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 1);
		testing.expect_value(t, events[0].reaction.step, advance+1);
		testing.expect(t, scene.simulation.triggers.event_count > 0);
		testing.expect_value(t, e.world_destroy(&scene.world), e.Status.Ok);
	}
}

joint_break_scheduled_provider_failure :: proc (t: ^testing.T)
{
	for workers in ([2]i32{4, 10})
	{
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = workers;
		world: e.World;
		testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
		type_id: e.Constraint_Type_ID;
		registered: bool;
		type_id, registered = register_velocity_target_constraint(t, &world);
		if !registered
		{
			e.world_destroy(&world);
			return;
		}
		testing.expect_value(t, e.world_enable_joint_breaks(&world, 128), e.Status.Ok);
		provider: Reaction_Provider_Test = {expected_speed=3};
		testing.expect_value(t, e.constraint_set_reaction_provider(&world, type_id, {read=reaction_test_provider, user_context=&provider}), e.Status.Ok);
		bodies: [128]e.Body_Handle;
		joints: [128]e.Constraint_Handle;
		for index in 0 ..< len(joints)
		{
			bodies[index], _ = e.body_add(&world, custom_constraint_body_description({f32(index)*4, 0, 0}, e.body_activity(-1, 255)));
			custom: Velocity_Target_Constraint = {target_speed=3};
			joints[index], _ = e.custom_constraint_add_typed(&world, bodies[index:index+1], type_id, &custom);
			testing.expect_value(t, e.constraint_set_break_limits(&world, joints[index], {metrics={.Force}, force=100000}), e.Status.Ok);
		}
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
		committed: [128]e.Joint_Reaction;
		for joint, index in joints
		{
			committed[index], _ = e.constraint_reaction(&world, joint);
			testing.expect_value(t, committed[index].state, e.Joint_Reaction_State.Solved);
			testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=0}), e.Status.Ok);
			testing.expect_value(t, e.body_set_velocity(&world, bodies[index], {}), e.Status.Ok);
		}
		provider.status = .Capacity_Missing;
		provider.calls = 0;
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Capacity_Missing);
		testing.expect(t, provider.calls > 0);
		for joint, index in joints
		{
			reaction: e.Joint_Reaction;
			status: e.Status;
			reaction, status = e.constraint_reaction(&world, joint);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, reaction, committed[index]);
		}
		written, required: int;
		status: e.Status;
		written, required, status = e.constraint_break_events_drain(&world, nil);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 0);
		testing.expect_value(t, required, 0);
		provider.status = .Ok;
		for body in bodies
		{
			testing.expect_value(t, e.body_set_velocity(&world, body, {}), e.Status.Ok);
		}
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
		events: [128]e.Joint_Break_Event;
		written, required, status = e.constraint_break_events_drain(&world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, len(joints));
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}

Joint_Break_Retry_State :: struct
{
	tracker: ^allocation_test_tracker,
	bodies: [2]e.Body_Handle,
	joints: [2]e.Constraint_Handle,
}

joint_break_sleep_after_solve :: proc "contextless" (
	user: rawptr, simulation: ^p.Simulation, dt: f32, dispatcher: ^util.Thread_Dispatcher_Boundary,
) -> e.Status
{
	context = runtime.default_context();
	state: ^Joint_Break_Retry_State = (^Joint_Break_Retry_State)(user);
	status: e.Status = p.default_timestepper_step(nil, simulation, dt, dispatcher);
	if status != .Ok
	{
		return status;
	}
	// sleep the sampled island, then fill its vacated active capacity so waking
	// must grow storage through the application allocator
	scaffold: ^p.Island_Scaffold = &simulation.sleeper.scaffold;
	scaffold.island_bodies.memory[0] = state.bodies[0];
	scaffold.island_bodies.memory[1] = state.bodies[1];
	scaffold.island_constraints.memory[0] = state.joints[0];
	scaffold.island_constraints.memory[1] = state.joints[1];
	status = p.island_sleeper_sleep_island(&simulation.sleeper, scaffold, 2, 2);
	if status != .Ok
	{
		return status;
	}
	active: ^p.Body_Set = &simulation.bodies.sets.memory[0];
	capacity: int = p.body_set_capacity(active);
	for active.count < capacity
	{
		body: e.Body_Description = custom_constraint_body_description({}, e.body_activity(-1, 255));
		_, status = p.bodies_add(&simulation.bodies, &body);
		if status != .Ok
		{
			return status;
		}
	}
	state.tracker.fail_from = state.tracker.requests + 1;
	return .Ok;
}

@(test)
joint_break_publication_retry_invalidates_views :: proc (t: ^testing.T)
{
	for delivery in 0 ..< 3
	{
		probe: Joint_Break_Allocation_Probe;
		tracker: ^allocation_test_tracker = &probe.tracker;
		allocator: runtime.Allocator = {procedure=joint_break_allocation_proc, data=&probe};
		pool: e.Buffer_Pool;
		testing.expect_value(t, e.buffer_pool_init_with_allocator(&pool, allocator, 128), e.Status.Ok);
		description: e.World_Description = small_world_description();
		description.gravity = {};
		description.damping = {};
		description.allocator = allocator;
		world: e.World;
		testing.expect_value(t, e.world_init_with_pool_and_allocation_scope(&world, description, &pool, .All_Owned), e.Status.Ok);
		testing.expect_value(t, e.world_enable_joint_breaks(&world, 2), e.Status.Ok);
		state: Joint_Break_Retry_State = {tracker=tracker};
		for index in 0 ..< 2
		{
			state.bodies[index], _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
			state.joints[index], _ = e.constraint_add_1(&world, state.bodies[index], e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
			testing.expect_value(t, e.constraint_set_break_limits(&world, state.joints[index], {metrics={.Force}, force=0, user_id=u64(index+1)}), e.Status.Ok);
		}
		simulation: ^p.Simulation;
		simulation, _ = e.world_borrow_simulation(&world);
		simulation.timestepper = {step=joint_break_sleep_after_solve, user_context=&state};
		contact_tracker: e.Contact_Tracker;
		stepper: e.Fixed_Stepper = e.fixed_stepper(0.02, 1);
		result: e.Joint_Break_Update_Result;
		status: e.Status;
		if delivery == 2
		{
			testing.expect_value(t, e.contact_tracker_init(&contact_tracker, 4), e.Status.Ok);
			testing.expect_value(t, e.contact_tracker_bind(&contact_tracker, &world), e.Status.Ok);
		}
		probe.world = &world;
		probe.body = state.bodies[0];
		probe.observe = .Present;
		if delivery == 2
		{
			result, status = e.joint_break_stepper_update(&stepper, &world, 0.02, nil, tracker=&contact_tracker);
			testing.expect_value(t, status, e.Status.Capacity_Missing);
			testing.expect_value(t, result.completed_steps, i32(1));
			testing.expect_value(t, result.pending, e.Joint_Break_Pending_Streams{.Contact, .Break});
			testing.expect_value(t, result.contact_required, i32(0));
			testing.expect_value(t, contact_tracker.last_step_index, u64(0));
			testing.expect_value(t, stepper.accumulator, f32(0));
		}
		else
		{
			testing.expect_value(t, e.world_step(&world, 0.02), e.Status.Ok);
		}
		testing.expect_value(t, simulation.solver.joint_breaks.phase, p.Joint_Break_Phase.Commit_Pending);
		testing.expect_value(t, simulation.solver.joint_breaks.notification_status, e.Status.Capacity_Missing);
		testing.expect_value(t, simulation.step_index, u64(1));
		for joint in state.joints
		{
			reaction: e.Joint_Reaction;
			reaction, _ = e.constraint_reaction(&world, joint);
			testing.expect_value(t, reaction.state, e.Joint_Reaction_State.Unsolved);
		}
		view: e.Active_Body_View;
		view, _ = e.active_body_view(&world);
		testing.expect(t, e.body_view_valid(&world, view));
		tracker.fail_from = 0;
		events: [2]e.Joint_Break_Event;
		written, required: int;
		if delivery != 1
		{
			if delivery == 2
			{
				result, status = e.joint_break_stepper_update(&stepper, &world, 0, events[:], tracker=&contact_tracker);
				written = int(result.break_events_written);
				testing.expect_value(t, result.completed_steps, i32(0));
				testing.expect_value(t, result.pending, e.Joint_Break_Pending_Streams{});
				testing.expect_value(t, contact_tracker.last_step_index, u64(1));
				testing.expect_value(t, stepper.accumulator, f32(0));
			}
			else
			{
				written, required, status = e.constraint_break_events_drain(&world, events[:]);
			}
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, written, 2);
			for event, index in events
			{
				testing.expect_value(t, event.constraint, state.joints[index]);
				testing.expect_value(t, event.user_id, u64(index+1));
				testing.expect_value(t, event.reaction.step, u64(1));
				testing.expect_value(t, event.reaction.state, e.Joint_Reaction_State.Solved);
			}
			testing.expect(t, events[0].lifetime < events[1].lifetime);
		}
		else
		{
			testing.expect_value(t, e.constraint_break_events_discard(&world), e.Status.Ok);
		}
		testing.expect(t, !e.body_view_valid(&world, view));
		probe.observe = .Missing;
		testing.expect(t, probe.attempts > 0);
		testing.expect_value(t, probe.violations, 0);
		for joint in state.joints
		{
			_, status = e.constraint_get_break_limits(&world, joint);
			testing.expect_value(t, status, e.Status.Not_Found);
		}
		written, required, status = e.constraint_break_events_drain(&world, events[:]);
		testing.expect_value(t, status, e.Status.Ok);
		testing.expect_value(t, written, 0);
		testing.expect_value(t, required, 0);
		testing.expect_value(t, simulation.step_index, u64(1));
		if delivery == 2
		{
			testing.expect_value(t, e.contact_tracker_destroy(&contact_tracker), e.Status.Ok);
		}
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
		testing.expect_value(t, e.buffer_pool_destroy(&pool), e.Status.Ok);
		allocation_test_empty(t, tracker);
	}
}

Joint_Break_Allocation_Operation :: enum u8
{
	Enable,
	Reserve,
	Growth,
}

Joint_Break_Allocation_Probe :: struct
{
	tracker: allocation_test_tracker,
	world: ^e.World,
	body: e.Body_Handle,
	observe: p.Reference_State,
	attempts, violations: int,
	requested_bytes, peak_live_bytes, allocator_calls: int,
}

joint_break_allocation_proc :: proc (
	data: rawptr, mode: runtime.Allocator_Mode, size, alignment: int,
	old_memory: rawptr, old_size: int,
	location: runtime.Source_Code_Location = #caller_location,
) -> ([]byte, runtime.Allocator_Error)
{
	probe: ^Joint_Break_Allocation_Probe = (^Joint_Break_Allocation_Probe)(data);
	if mode != .Query_Features
	{
		probe.allocator_calls += 1;
	}
	if mode == .Alloc || mode == .Alloc_Non_Zeroed || mode == .Resize || mode == .Resize_Non_Zeroed
	{
		probe.requested_bytes += size;
	}
	if probe.observe == .Present && mode != .Query_Features
	{
		probe.attempts += 1;
		if e.body_set_velocity(probe.world, probe.body, {}) != .Disposed
		{
			probe.violations += 1;
		}
	}
	memory: []byte;
	error: runtime.Allocator_Error;
	memory, error = allocation_test_proc(&probe.tracker, mode, size, alignment, old_memory, old_size, location);
	probe.peak_live_bytes = max(probe.peak_live_bytes, probe.tracker.live_bytes);
	return memory, error;
}

@(test)
joint_break_all_owned_failures_and_reserved_reuse :: proc (t: ^testing.T)
{
	for operation in ([3]Joint_Break_Allocation_Operation{.Enable, .Reserve, .Growth})
	{
		request_count: int = 0;
		for fail: int = 0; fail <= request_count; fail += 1
		{
			probe: Joint_Break_Allocation_Probe;
			allocator: runtime.Allocator = {procedure=joint_break_allocation_proc, data=&probe};
			pool: e.Buffer_Pool;
			testing.expect_value(t, e.buffer_pool_init_with_allocator(&pool, allocator, 128), e.Status.Ok);
			description: e.World_Description = small_world_description();
			description.gravity = {};
			description.damping = {};
			description.allocator = allocator;
			world: e.World;
			testing.expect_value(t, e.world_init_with_pool_and_allocation_scope(&world, description, &pool, .All_Owned), e.Status.Ok);
			probe.world = &world;
			bodies: [2]e.Body_Handle;
			joints: [2]e.Constraint_Handle;
			for index in 0 ..< 2
			{
				bodies[index], _ = e.body_add(&world, custom_constraint_body_description({}, e.body_activity(-1, 255)));
				joints[index], _ = e.constraint_add_1(&world, bodies[index], e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
			}
			probe.body = bodies[0];
			if operation != .Enable
			{
				testing.expect_value(t, e.world_enable_joint_breaks(&world, 1), e.Status.Ok);
				testing.expect_value(t, e.constraint_set_break_limits(&world, joints[0], {metrics={.Force}, force=100000, user_id=17}), e.Status.Ok);
			}
			before: int = probe.tracker.requests;
			if fail > 0
			{
				probe.tracker.fail_from = before + fail;
			}
			probe.observe = .Present;
			status: e.Status;
			switch operation
			{
				case .Enable:
					status = e.world_enable_joint_breaks(&world, 1);
				case .Reserve:
					status = e.joint_break_reserve(&world, 128);
				case .Growth:
					status = e.constraint_set_break_limits(&world, joints[1], {metrics={.Force}, force=100000});
			}
			probe.observe = .Missing;
			testing.expect_value(t, probe.violations, 0);
			testing.expect(t, probe.attempts > 0);
			if fail == 0
			{
				testing.expect_value(t, status, e.Status.Ok);
				request_count = probe.tracker.requests - before;
				testing.expect(t, request_count > 0);
			}
			else
			{
				testing.expect_value(t, status, e.Status.Capacity_Missing);
			}
			if operation != .Enable
			{
				limits: e.Joint_Break_Limits;
				limits, status = e.constraint_get_break_limits(&world, joints[0]);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect_value(t, limits.user_id, u64(17));
			}
			if fail == 0
			{
				testing.expect_value(t, e.joint_break_reserve(&world, 8), e.Status.Ok);
				for joint in joints
				{
					testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=100000}), e.Status.Ok);
				}
				testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
				testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
				before = probe.tracker.requests;
				probe.tracker.fail_from = before + 1;
				for _ in 0 ..< 4
				{
					testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
					written, required: int;
					written, required, status = e.constraint_break_events_drain(&world, nil);
					testing.expect_value(t, status, e.Status.Ok);
					testing.expect_value(t, written, 0);
					testing.expect_value(t, required, 0);
				}
				for joint, index in joints
				{
					testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=0}), e.Status.Ok);
					testing.expect_value(t, e.body_set_velocity(&world, bodies[index], {}), e.Status.Ok);
				}
				testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
				events: [2]e.Joint_Break_Event;
				written, required: int;
				written, required, status = e.constraint_break_events_drain(&world, events[:]);
				testing.expect_value(t, status, e.Status.Ok);
				testing.expect_value(t, written, 2);
				testing.expect_value(t, required, 2);
				testing.expect_value(t, probe.tracker.requests, before);
			}
			// frees must also reject reentry while retaining exact allocation metadata
			probe.observe = .Present;
			probe.attempts = 0;
			status = e.world_disable_joint_breaks(&world);
			probe.observe = .Missing;
			testing.expect_value(t, status, e.Status.Not_Found if operation == .Enable && fail > 0 else e.Status.Ok);
			testing.expect_value(t, probe.violations, 0);
			if status == .Ok
			{
				testing.expect(t, probe.attempts > 0);
			}
			testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
			testing.expect_value(t, e.buffer_pool_destroy(&pool), e.Status.Ok);
			allocation_test_empty(t, &probe.tracker);
		}
	}
	// one retained world isolates optional ownership from identical body/joint
	// storage. counters describe application allocator requests, not process RSS
	probe: Joint_Break_Allocation_Probe;
	description: e.World_Description = small_world_description();
	description.gravity = {};
	description.damping = {};
	description.capacity.bodies = 1024;
	description.capacity.constraints = 1024;
	description.capacity.initial_constraints_per_type_batch = 1024;
	description.allocator = {procedure=joint_break_allocation_proc, data=&probe};
	world: e.World;
	testing.expect_value(t, e.world_init_with_allocation_scope(&world, description, .All_Owned), e.Status.Ok);
	joints: [1024]e.Constraint_Handle;
	for &joint, index in joints
	{
		body: e.Body_Handle;
		body, _ = e.body_add(&world, custom_constraint_body_description({f32(index)*4, 0, 0}, e.body_activity(-1, 255)));
		joint, _ = e.constraint_add_1(&world, body, e.One_Body_Linear_Motor{{}, {10, 0, 0}, e.motor_settings(1000, 0.01)});
	}
	state_names: [4]string = {"disabled", "enabled_empty", "sparse", "dense"};
	for state in 0 ..< len(state_names)
	{
		requested_before: int = probe.requested_bytes;
		calls_before: int = probe.allocator_calls;
		if state == 0
		{
			requested_before = 0;
			calls_before = 0;
		}
		else
		{
			probe.peak_live_bytes = probe.tracker.live_bytes;
		}
		if state == 1
		{
			testing.expect_value(t, e.world_enable_joint_breaks(&world, 1024), e.Status.Ok);
		}
		watch_count: int;
		if state == 2
		{
			watch_count = 64;
		}
		else if state == 3
		{
			watch_count = len(joints);
		}
		for joint in joints[:watch_count]
		{
			testing.expect_value(t, e.constraint_set_break_limits(&world, joint, {metrics={.Force}, force=100000}), e.Status.Ok);
		}
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
		warm_requests: int = probe.tracker.requests;
		warm_calls: int = probe.allocator_calls;
		probe.tracker.fail_from = warm_requests + 1;
		testing.expect_value(t, e.world_step(&world, 0.125), e.Status.Ok);
		if state > 0
		{
			written, required: int;
			status: e.Status;
			written, required, status = e.constraint_break_events_drain(&world, nil);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, written, 0);
			testing.expect_value(t, required, 0);
		}
		testing.expect_value(t, probe.tracker.requests, warm_requests);
		testing.expect_value(t, probe.allocator_calls, warm_calls);
		probe.tracker.fail_from = 0;
		log.infof("joint_break_memory state=%s watches=%d requested_bytes=%d state_requested_bytes=%d peak_live_bytes=%d live_bytes=%d allocation_requests=%d allocator_calls=%d state_allocator_calls=%d warm_allocator_calls=%d",
			state_names[state], watch_count, probe.requested_bytes, probe.requested_bytes-requested_before,
			probe.peak_live_bytes, probe.tracker.live_bytes, probe.tracker.requests, probe.allocator_calls,
			probe.allocator_calls-calls_before, probe.allocator_calls-warm_calls);
	}
	testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	allocation_test_empty(t, &probe.tracker);
}
