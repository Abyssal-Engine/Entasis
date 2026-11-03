package solver_fixed_work

import "core:simd"
import "core:math"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"
import contacts "../solver_contacts"

fixed_test_options :: proc(case_name: support.Solver_Fixed_Case) -> Options
{
	o: Options = {case_name=case_name, workers=1, passes=2, warmup=1, bundles=8,
		layout="full", angular="nonconserving", chains=2, links=9, mass_ratio=1,
		velocity_iterations=1, substeps=4, unconstrained_substeps=.Enabled};
	if case_name == .Joints || case_name == .Conserving
	{
		o.angular = "momentum";
	}
	if case_name == .Cached_Convex
	{
		o.velocity_iterations, o.substeps = 2, 2;
	}
	return o;
}

Fixed_Dispatch_Trace :: struct
{
	native: ^util.Thread_Dispatcher_Boundary,
	calls, maximum_workers: int,
}

fixed_trace_dispatch :: proc "contextless" (
	boundary: ^util.Thread_Dispatcher_Boundary, worker: util.Dispatcher_Worker_Proc,
	maximum_workers: int, unmanaged_context: rawptr,
) -> util.Threading_Status
{
	trace: ^Fixed_Dispatch_Trace = (^Fixed_Dispatch_Trace)(boundary.dispatcher);
	trace.calls += 1;
	trace.maximum_workers = max(trace.maximum_workers, maximum_workers);
	return trace.native.dispatch(trace.native, worker, maximum_workers, unmanaged_context);
}

fixed_trace_pool :: proc(boundary: ^util.Thread_Dispatcher_Boundary, worker: int) -> (^util.Buffer_Pool, util.Threading_Status)
{
	trace: ^Fixed_Dispatch_Trace = (^Fixed_Dispatch_Trace)(boundary.dispatcher);
	return trace.native.worker_pool(trace.native, worker);
}

fixed_expect_dynamics :: proc(t: ^testing.T, a, b: ^Fixed_State)
{
	left: ^physics.Body_Set = &a.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	right: ^physics.Body_Set = &b.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	testing.expect_value(t, left.count, right.count);
	for i in 0 ..< min(left.count, right.count)
	{
		x: ^physics.Body_Dynamics = &left.dynamics_state.memory[i];
		y: ^physics.Body_Dynamics = &right.dynamics_state.memory[i];
		components_x: [6]f32 = {x.motion.velocity.linear.x, x.motion.velocity.linear.y, x.motion.velocity.linear.z,
			x.motion.velocity.angular.x, x.motion.velocity.angular.y, x.motion.velocity.angular.z};
		components_y: [6]f32 = {y.motion.velocity.linear.x, y.motion.velocity.linear.y, y.motion.velocity.linear.z,
			y.motion.velocity.angular.x, y.motion.velocity.angular.y, y.motion.velocity.angular.z};
		for value, axis in components_x
		{
			testing.expect(t, abs(f64(value)-f64(components_y[axis])) <= 2e-5*max(1, abs(f64(value))));
		}
	}
}

@(test)
fixed_native_solver_reset_is_independent :: proc(t: ^testing.T)
{
	o: Options = fixed_test_options(.Convex);
	o.workers = 2;
	a, b: Fixed_State;
	defer fixed_dispose(&a);
	defer fixed_dispose(&b);
	// enough constraints beyond batch zero to exercise native parallel ownership
	if !testing.expect_value(t, fixed_create(&a, o, 4096), physics.Physics_Status.Ok) ||
		!testing.expect_value(t, fixed_create(&b, o, 4096), physics.Physics_Status.Ok)
	{
		return;
	}
	dispatcher: util.Thread_Dispatcher;
	if !testing.expect_value(t, util.thread_dispatcher_initialize(&dispatcher, 2), util.Threading_Status.Ok)
	{
		return;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	trace: Fixed_Dispatch_Trace = {native=util.thread_dispatcher_boundary(&dispatcher)};
	boundary: util.Thread_Dispatcher_Boundary = {dispatcher=&trace, dispatch=fixed_trace_dispatch,
		worker_pool=fixed_trace_pool, worker_count=2};
	testing.expect_value(t, fixed_execute(&a, &boundary), physics.Physics_Status.Ok);
	warmup_quality: Fixed_Quality = fixed_observe(&a);
	testing.expect_value(t, warmup_quality.failures, 0);
	testing.expect_value(t, warmup_quality.observed_units, a.points);
	a.bodies.sets.memory[0].dynamics_state.memory[0].motion.velocity.linear.y = 999;
	a.pose_context.gravity.y = 500;
	for canonical in a.canonical
	{
		if len(canonical.prestep) > 0
		{
			canonical.native.prestep_data.memory[0] = 255;
			canonical.native.accumulated_impulses.memory[0] = 255;
		}
	}
	fixed_restore(&a);
	testing.expect_value(t, a.pose_context.gravity, a.canonical_context.gravity);
	testing.expect_value(t, fixed_execute(&a, &boundary), physics.Physics_Status.Ok);
	testing.expect_value(t, fixed_execute(&b, &boundary), physics.Physics_Status.Ok);
	fixed_expect_dynamics(t, &a, &b);
	testing.expect(t, trace.calls > 0 && trace.maximum_workers >= 2);
	testing.expect_value(t, fixed_observe(&a).failures, 0);
}

Fixed_Callback_Trace :: struct
{
	base: ^physics.Default_Pose_Integrator_Context,
	calls: int,
	dt: f32,
	first_y, last_y: f32,
}

fixed_callback_prepare :: proc "contextless" (user_context: rawptr, dt: f32) -> physics.Physics_Status
{
	trace: ^Fixed_Callback_Trace = (^Fixed_Callback_Trace)(user_context);
	trace.dt = dt;
	return physics.pose_integrator_default_prepare(trace.base, dt);
}

fixed_callback_velocity :: proc "contextless" (
	user_context: rawptr, indices: util.I32x8, position: util.Vector3_Wide, orientation: util.Quaternion_Wide,
	inertia: physics.Body_Inertia_Wide, mask: util.I32x8, worker: int, dt: util.F32x8, velocity: ^physics.Body_Velocity_Wide,
)
{
	trace: ^Fixed_Callback_Trace = (^Fixed_Callback_Trace)(user_context);
	if trace.calls == 0
	{
		trace.first_y = simd.extract(position.y, 0);
	}
	trace.last_y = simd.extract(position.y, 0);
	trace.calls += 1;
	physics.pose_integrator_default_velocity(trace.base, indices, position, orientation, inertia, mask, worker, dt, velocity);
}

fixed_install_callback_trace :: proc(s: ^Fixed_State, trace: ^Fixed_Callback_Trace)
{
	trace.base = &s.pose_context;
	s.integrator.callbacks.user_context = trace;
	s.integrator.callbacks.prepare_for_integration = fixed_callback_prepare;
	s.integrator.callbacks.integrate_velocity = fixed_callback_velocity;
}

@(test)
fixed_final_integration_consumers_preserve_callbacks :: proc(t: ^testing.T)
{
	for kind in ([3]support.Solver_Fixed_Case{.Pose, .Constrained_Final, .Free_Final})
	{
		for policy in support.Fixed_Substeps
		{
			o: Options = fixed_test_options(kind);
			o.unconstrained_substeps = policy;
			s: Fixed_State;
			if !testing.expect_value(t, fixed_create(&s, o, 17), physics.Physics_Status.Ok)
			{
				fixed_dispose(&s);
				continue;
			}
			// zero inverse mass with rotational inertia remains a rotating dynamic body
			s.initial[16].inertia.local.inverse_mass = 0;
			s.initial[16].inertia.world.inverse_mass = 0;
			fixed_restore(&s);
			trace: Fixed_Callback_Trace;
			fixed_install_callback_trace(&s, &trace);
			testing.expect_value(t, fixed_execute(&s), physics.Physics_Status.Ok);
			active: ^physics.Body_Set = &s.bodies.sets.memory[0];
			expected_dt: f32 = 1.0/240.0 if kind == .Constrained_Final else 1.0/60.0;
			testing.expect(t, abs(active.dynamics_state.memory[16].motion.pose.position.x-(s.initial[16].motion.pose.position.x+0.13*expected_dt)) < 3e-5);
			testing.expect(t, active.dynamics_state.memory[16].motion.pose.orientation != s.initial[16].motion.pose.orientation);
			if kind == .Free_Final
			{
				expected_calls: int = 12 if policy == .Enabled else 3;
				testing.expect_value(t, trace.calls, expected_calls);
				testing.expect_value(t, trace.dt, f32(1.0/240.0) if policy == .Enabled else f32(1.0/60.0));
				testing.expect_value(t, trace.first_y, s.initial[0].motion.pose.position.y);
			}
			else
			{
				testing.expect_value(t, trace.calls, 0);
			}
			testing.expect_value(t, fixed_observe(&s).failures, 0);
			fixed_dispose(&s);
		}
	}
}

@(test)
fixed_kinematics_preserve_masked_continuation :: proc(t: ^testing.T)
{
	for callbacks in physics.Velocity_Integration_State
	{
		o: Options = fixed_test_options(.Kinematic);
		o.layout = "sparse";
		s: Fixed_State;
		if !testing.expect_value(t, fixed_create(&s, o, 17), physics.Physics_Status.Ok)
		{
			fixed_dispose(&s);
			continue;
		}
		trace: Fixed_Callback_Trace;
		fixed_install_callback_trace(&s, &trace);
		s.integrator.callbacks.integrate_kinematic_velocity = callbacks;
		testing.expect_value(t, fixed_execute(&s), physics.Physics_Status.Ok);
		active: ^physics.Body_Set = &s.bodies.sets.memory[0];
		for i in 0 ..< active.count
		{
			mask: util.I32x8 = fixed_lane_mask(o.layout, int(i)/8, int(active.count));
			if simd.extract(mask, int(i)%8) == 0
			{
				testing.expect_value(t, active.dynamics_state.memory[i].motion.pose.position, s.initial[i].motion.pose.position);
			}
		}
		testing.expect_value(t, trace.calls, 12 if callbacks == .Enabled else 0);
		testing.expect(t, active.dynamics_state.memory[0].motion.pose.position.x > s.initial[0].motion.pose.position.x);
		testing.expect_value(t, fixed_observe(&s).failures, 0);
		fixed_dispose(&s);
	}
}

@(test)
fixed_solver_cases_execute_cached_nonconvex_and_joint_paths :: proc(t: ^testing.T)
{
	for kind in ([3]support.Solver_Fixed_Case{.Cached_Convex, .Nonconvex, .Joints})
	{
		o: Options = fixed_test_options(kind);
		o.scenario = .Mixed;
		s: Fixed_State;
		if !testing.expect_value(t, fixed_create(&s, o), physics.Physics_Status.Ok)
		{
			fixed_dispose(&s);
			continue;
		}
		testing.expect(t, s.solver.active_set.constraint_count > 0);
		testing.expect_value(t, fixed_execute(&s), physics.Physics_Status.Ok);
		q: Fixed_Quality = fixed_observe(&s);
		testing.expect_value(t, q.failures, 0);
		if kind != .Joints
		{
			testing.expect_value(t, q.observed_units, s.points);
			testing.expect(t, s.points > 0);
		}
		testing.expect(t, s.bodies.sets.memory[0].dynamics_state.memory[len(s.initial)-1].motion.velocity != s.initial[len(s.initial)-1].motion.velocity);
		fixed_restore(&s);
		testing.expect_value(t, s.bodies.sets.memory[0].dynamics_state.memory[0].motion.velocity, s.initial[0].motion.velocity);
		fixed_dispose(&s);
	}
	for scenario in ([2]support.Solver_Joint_Scenario{.Hinge, .Weld})
	{
		o: Options = fixed_test_options(.Joints);
		o.scenario, o.chains, o.links = scenario, 1, 1;
		s: Fixed_State;
		if !testing.expect_value(t, fixed_create(&s, o), physics.Physics_Status.Ok)
		{
			fixed_dispose(&s);
			continue;
		}
		active: ^physics.Body_Set = &s.bodies.sets.memory[0];
		a: ^physics.Rigid_Pose = &active.dynamics_state.memory[0].motion.pose;
		b: ^physics.Rigid_Pose = &active.dynamics_state.memory[1].motion.pose;
		a^ = {orientation={w=1}};
		b^ = {position={0, -1, 0}, orientation={w=1}};
		if scenario == .Hinge
		{
			// a hinge can rotate around its shared axis without anchor or angle error
			b^ = {position={0.5, -0.5, 0}, orientation={z=math.sqrt(f32(0.5)), w=math.sqrt(f32(0.5))}};
		}
		q: Fixed_Quality = fixed_observe(&s);
		testing.expect_value(t, q.failures, 0);
		testing.expect(t, q.position_error_m < 1e-6);
		testing.expect(t, q.joint_angle_error_rad < 1e-6);
		b.position.x += 0.25;
		q = fixed_observe(&s);
		testing.expect(t, abs(q.position_error_m-0.25) < 1e-6);
		testing.expect(t, q.joint_angle_error_rad < 1e-6);
		b.orientation = {x=math.sqrt(f32(0.5)), w=math.sqrt(f32(0.5))};
		b.position = {0, -1, 0};
		if scenario == .Hinge
		{
			b.position = util.vector3_subtract({0, -0.5, 0}, util.quaternion_transform({0, 0.5, 0}, b.orientation));
		}
		q = fixed_observe(&s);
		testing.expect_value(t, q.failures, 0);
		testing.expect(t, q.position_error_m < 1e-6);
		testing.expect(t, abs(q.joint_angle_error_rad-math.PI/2) < 1e-6);
		testing.expect(t, q.orientation_norm_error < 1e-6);
		fixed_dispose(&s);
	}
}

@(test)
fixed_work_counts_only_executed_units :: proc(t: ^testing.T)
{
	o: Options = fixed_test_options(.Nonconserving);
	o.layout = "sparse";
	s: Fixed_State;
	if !testing.expect_value(t, fixed_create(&s, o, 17), physics.Physics_Status.Ok)
	{
		return;
	}
	defer fixed_dispose(&s);
	testing.expect_value(t, len(s.initial), 17);
	testing.expect_value(t, s.admitted_units, 2);
	testing.expect_value(t, fixed_work_units(&s, 100), i64(800));
	convex: Fixed_State;
	defer fixed_dispose(&convex);
	o = fixed_test_options(.Convex);
	if !testing.expect_value(t, fixed_create(&convex, o, 17), physics.Physics_Status.Ok)
	{
		return;
	}
	testing.expect_value(t, fixed_work_units(&convex, 100), i64(convex.points)*400);
	testing.expect_value(t, fixed_execute(&convex), physics.Physics_Status.Ok);
	testing.expect_value(t, fixed_observe(&convex).observed_units, convex.points);
	zero_rows, padded_tails: int;
	for canonical in convex.canonical
	{
		kind: ^physics.Type_Batch = canonical.native;
		if kind.type_id == 0 && zero_rows == 0
		{
			contact: contacts.Contact_Input = contacts.contact_read(kind, 0);
			contact.normal_impulses, contact.tangent_impulse, contact.twist_impulse = {}, {}, 0;
			contact.depths[0] = -0.01;
			body: int = int(u32(contact.bodies[0]) & physics.BODY_REFERENCE_INDEX_MASK);
			convex.bodies.sets.memory[0].dynamics_state.memory[body].motion.velocity = {linear={0, 1, 0}};
			contacts.contact_write(kind, 0, contact);
			testing.expect_value(t, contacts.contact_read(kind, 0).normal_impulses[0], f32(0));
			zero_rows += 1;
		}
		if kind.count%8 != 0
		{
			impulses: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, int(kind.count)-1));
			impulses[0] = simd.replace(impulses[0], int(kind.count)%8, transmute(f32)u32(0x7fc00000));
			padded_tails += 1;
		}
	}
	q: Fixed_Quality = fixed_observe(&convex);
	testing.expect_value(t, q.failures, 0);
	testing.expect_value(t, q.observed_units, convex.points);
	testing.expect_value(t, zero_rows, 1);
	testing.expect(t, padded_tails > 0);
	o.passes, o.warmup = 100, 10;
	m: Fixed_Measurement = fixed_measure(o);
	testing.expect_value(t, m.status, physics.Physics_Status.Ok);
	testing.expect_value(t, m.completed, 100);
	testing.expect_value(t, m.quality_failures, 0);
	testing.expect_value(t, m.quality.observed_units, m.points);
	testing.expect_value(t, m.work_units, i64(m.points)*400);
	for kind in ([4]support.Solver_Fixed_Case{.Convex, .Cached_Convex, .Nonconvex, .Nonconserving})
	{
		o = fixed_test_options(kind);
		o.passes, o.warmup = 2, 0;
		cold: Fixed_Measurement = fixed_measure(o);
		o.warmup = 2;
		warmed: Fixed_Measurement = fixed_measure(o);
		for measurement in ([2]Fixed_Measurement{cold, warmed})
		{
			testing.expect_value(t, measurement.status, physics.Physics_Status.Ok);
			testing.expect_value(t, measurement.completed, 2);
			testing.expect_value(t, measurement.quality_failures, 0);
			testing.expect_value(t, measurement.quality.failures, 0);
			testing.expect_value(t, measurement.bodies, o.bundles*8);
			if kind == .Nonconserving
			{
				testing.expect_value(t, measurement.points, 0);
				testing.expect_value(t, measurement.quality.observed_units, measurement.admitted_units);
				testing.expect_value(t, measurement.work_units, i64(measurement.admitted_units)*8);
			}
			else
			{
				testing.expect(t, measurement.points > 0);
				testing.expect_value(t, measurement.quality.observed_units, measurement.points);
				testing.expect_value(t, measurement.work_units,
					i64(measurement.points)*i64(o.passes)*i64(o.velocity_iterations)*i64(o.substeps));
			}
		}
		testing.expect_value(t, warmed.bodies, cold.bodies);
		testing.expect_value(t, warmed.dynamic_bodies, cold.dynamic_bodies);
		testing.expect_value(t, warmed.kinematic_bodies, cold.kinematic_bodies);
		testing.expect_value(t, warmed.admitted_units, cold.admitted_units);
		testing.expect_value(t, warmed.constraints, cold.constraints);
		testing.expect_value(t, warmed.points, cold.points);
		testing.expect_value(t, warmed.work_units, cold.work_units);
		testing.expect_value(t, warmed.quality, cold.quality);
		testing.expect_value(t, warmed.pose_context.gravity, cold.pose_context.gravity);
		testing.expect_value(t, warmed.pose_context.linear_damping, cold.pose_context.linear_damping);
		testing.expect_value(t, warmed.pose_context.angular_damping, cold.pose_context.angular_damping);
	}
	expected_admitted: [4]int = {64, 13, 8, 64};
	for layout, i in ([4]string{"full", "partial", "sparse", "isotropic"})
	{
		o = fixed_test_options(.Conserving);
		o.layout, o.passes, o.warmup = layout, 1, 0;
		elapsed: f64;
		hash: u64;
		admitted: int;
		status: support.Admission;
		elapsed, hash, admitted, status = conserving_measure(o);
		testing.expect_value(t, status, support.Admission.Ok);
		testing.expect(t, elapsed > 0);
		testing.expect(t, hash != 0);
		testing.expect_value(t, admitted, expected_admitted[i]);
	}
}
