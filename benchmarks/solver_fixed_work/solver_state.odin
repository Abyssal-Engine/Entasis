package solver_fixed_work

import "base:intrinsics"
import "core:math"
import "core:simd"
import "core:time"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"
import contacts "../solver_contacts"

Options :: support.Solver_Fixed_Options;

Canonical_Batch :: struct
{
	native: ^physics.Type_Batch,
	prestep, impulses: []u8,
}

Joint_Edge :: struct
{
	a, b: int,
	kind: support.Solver_Joint_Scenario,
}

Fixed_State :: struct
{
	pool: util.Buffer_Pool,
	bodies: physics.Bodies,
	solver: physics.Solver,
	integrator: physics.Pose_Integrator,
	pose_context, canonical_context: physics.Default_Pose_Integrator_Context,
	initial: []physics.Body_Dynamics,
	canonical: []Canonical_Batch,
	handles: []physics.Body_Handle,
	edges: []Joint_Edge,
	scratch: []f64,
	options: Options,
	points, admitted_units, dynamic_bodies, kinematic_bodies: int,
	worker_statuses: [64]physics.Physics_Status,
}

Fixed_Quality :: struct
{
	failures, observed_units: int,
	residual_max_mps, residual_rms_mps, residual_p95_mps: f64,
	position_error_m, orientation_norm_error, joint_angle_error_rad: f64,
}

Fixed_Measurement :: struct
{
	elapsed_ms: f64,
	completed, quality_failures: int,
	quality: Fixed_Quality,
	bodies, dynamic_bodies, kinematic_bodies, admitted_units, constraints, points: int,
	work_units: i64,
	pool_before, pool_after, worker_pool_before, worker_pool_after: u64,
	status: physics.Physics_Status,
	pose_context: physics.Default_Pose_Integrator_Context,
}

fixed_angular_mode :: proc(value: string) -> physics.Angular_Integration_Mode
{
	switch value
	{
	case "momentum": return .Conserve_Momentum;
	case "gyro": return .Conserve_Momentum_With_Gyroscopic_Torque;
	case: return .Nonconserving;
	}
}

fixed_lane_mask :: proc(layout: string, bundle, count: int) -> util.I32x8
{
	mask: util.I32x8;
	bits: u8 = 255;
	if layout == "partial"
	{
		bits = u8(bundle%254+1);
	}
	else if layout == "sparse"
	{
		bits = u8(1)<<uint(bundle%8);
	}
	for lane in 0 ..< 8
	{
		if bundle*8+lane < count && bits & (u8(1)<<uint(lane)) != 0
		{
			mask = simd.replace(mask, lane, -1);
		}
	}
	return mask;
}

fixed_dispose :: proc(s: ^Fixed_State)
{
	for canonical in s.canonical
	{
		delete(canonical.prestep);
		delete(canonical.impulses);
	}
	delete(s.canonical);
	delete(s.initial);
	delete(s.handles);
	delete(s.edges);
	delete(s.scratch);
	if s.integrator.state == .Ready
	{
		_ = physics.pose_integrator_dispose(&s.integrator);
	}
	if s.solver.state == .Ready
	{
		_ = physics.solver_dispose(&s.solver);
	}
	if s.bodies.state == .Allocated
	{
		_ = physics.bodies_dispose(&s.bodies);
	}
	if s.pool.state == .Ready
	{
		_ = util.buffer_pool_dispose(&s.pool);
	}
	s^ = {};
}

fixed_create :: proc(s: ^Fixed_State, o: Options, population: int = 0) -> physics.Physics_Status
{
	s.options = o;
	count: int = o.bundles*8 if population == 0 else population;
	if o.case_name == .Joints
	{
		count = o.chains*(o.links+1);
	}
	status: physics.Physics_Status;
	if util.buffer_pool_initialize(&s.pool, 65536) != .Ok
	{
		return .Capacity_Missing;
	}
	status = physics.bodies_initialize(&s.bodies, count, 1, 8, &s.pool);
	if status != .Ok
	{
		return status;
	}
	status = physics.solver_initialize(&s.solver, &s.bodies, max(4*count, 8), 65, 64, 8, &s.pool);
	if status != .Ok
	{
		return status;
	}
	s.pose_context = {gravity={0, -10, 0}, linear_damping=0, angular_damping=0};
	s.canonical_context = s.pose_context;
	callbacks: physics.Pose_Integrator_Callbacks = physics.pose_integrator_default_callbacks(&s.pose_context);
	callbacks.angular_mode = fixed_angular_mode(o.angular);
	callbacks.allow_substeps_for_unconstrained = .Enabled if o.unconstrained_substeps == .Enabled else .Disabled;
	callbacks.integrate_kinematic_velocity = .Enabled if o.case_name == .Kinematic else .Disabled;
	status = physics.pose_integrator_initialize(&s.integrator, &s.bodies, callbacks);
	if status != .Ok
	{
		return status;
	}
	s.solver.integrator = &s.integrator;
	s.handles = make([]physics.Body_Handle, count);
	for i in 0 ..< count
	{
		body: physics.Body_Description = {
			pose={orientation={w=1}, position={f32(i%100)*1.1, f32(i/100)*1.05+1, 0}},
			velocity={linear={0.13, -0.17, 0.03}, angular={0.21+f32(i%7)*0.03, 0.17, -0.11}},
			local_inertia={inverse_mass=0.5, inverse_inertia_tensor={xx=0.7, yx=0.03, yy=1.1, zx=-0.01, zy=0.02, zz=1.9}},
			activity={sleep_threshold=-1, minimum_timestep_count_under_threshold=255},
		};
		if o.layout == "isotropic"
		{
			body.local_inertia.inverse_inertia_tensor = {xx=1, yy=1, zz=1};
		}
		if o.case_name == .Kinematic
		{
			body.local_inertia = {};
			s.kinematic_bodies += 1;
		}
		else if o.case_name == .Joints && i%(o.links+1) == 0
		{
			body.local_inertia = {};
			s.kinematic_bodies += 1;
		}
		else
		{
			s.dynamic_bodies += 1;
		}
		if o.case_name == .Joints
		{
			chain: int = i/(o.links+1);
			link: int = i%(o.links+1);
			angle: f32 = 0.18+f32(chain%7)*0.017;
			qz: util.Quaternion = {z=math.sin(angle/2), w=math.cos(angle/2)};
			qx: util.Quaternion = {x=math.sin(f32(0.11)), w=math.cos(f32(0.11))};
			body.pose.orientation = util.quaternion_concatenate(qz, qx);
			body.pose.position = util.vector3_add({f32(chain%32)*3, f32(o.links+2), f32(chain/32)*3},
				util.quaternion_transform({0, -f32(link), 0}, body.pose.orientation));
			body.velocity = {};
			if link > 0
			{
				mass: f32 = f32(o.mass_ratio) if link == o.links else 1;
				body.local_inertia = {inverse_mass=1/mass, inverse_inertia_tensor={xx=11/mass, yy=66/mass, zz=11/mass}};
			}
		}
		s.handles[i], status = physics.bodies_add(&s.bodies, &body);
		if status != .Ok
		{
			return status;
		}
		mask: util.I32x8 = fixed_lane_mask(o.layout, i/8, count);
		if simd.extract(mask, i%8) != 0
		{
			s.admitted_units += 1;
		}
	}
	switch o.case_name
	{
	case .Convex, .Cached_Convex, .Nonconvex, .Constrained_Final:
		status = fixed_add_contacts(s);
	case .Joints:
		status = fixed_add_joints(s);
	case .Conserving, .Nonconserving, .Pose, .Free_Final, .Kinematic:
		status = .Ok;
	}
	if status != .Ok
	{
		return status;
	}
	s.initial = make([]physics.Body_Dynamics, count);
	active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	intrinsics.mem_copy(raw_data(s.initial), active.dynamics_state.memory, count*size_of(physics.Body_Dynamics));
	type_count: int;
	for batch_index in 0 ..< s.solver.active_set.batch_count
	{
		type_count += int(s.solver.active_set.batches.memory[batch_index].type_batch_count);
	}
	s.canonical = make([]Canonical_Batch, type_count);
	canonical_index: int;
	for batch_index in 0 ..< s.solver.active_set.batch_count
	{
		batch: ^physics.Constraint_Batch = &s.solver.active_set.batches.memory[batch_index];
		for type_index in 0 ..< batch.type_batch_count
		{
			kind: ^physics.Type_Batch = &batch.type_batches.memory[type_index];
			canonical: ^Canonical_Batch = &s.canonical[canonical_index];
			canonical_index += 1;
			canonical.native = kind;
			bundles: int = (int(kind.count)+7)/8;
			canonical.prestep = make([]u8, bundles*int(kind.prestep_bundle_size));
			canonical.impulses = make([]u8, bundles*int(kind.impulse_bundle_size));
			if bundles > 0
			{
				intrinsics.mem_copy(raw_data(canonical.prestep), kind.prestep_data.memory, len(canonical.prestep));
				intrinsics.mem_copy(raw_data(canonical.impulses), kind.accumulated_impulses.memory, len(canonical.impulses));
			}
		}
	}
	s.scratch = make([]f64, max(s.points, 1));
	return .Ok;
}

fixed_restore :: proc(s: ^Fixed_State)
{
	active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	intrinsics.mem_copy(active.dynamics_state.memory, raw_data(s.initial), len(s.initial)*size_of(physics.Body_Dynamics));
	for canonical in s.canonical
	{
		if len(canonical.prestep) > 0
		{
			intrinsics.mem_copy(canonical.native.prestep_data.memory, raw_data(canonical.prestep), len(canonical.prestep));
			intrinsics.mem_copy(canonical.native.accumulated_impulses.memory, raw_data(canonical.impulses), len(canonical.impulses));
		}
	}
	s.pose_context = s.canonical_context;
}

fixed_region :: proc(case_name: support.Solver_Fixed_Case) -> string
{
	switch case_name
	{
	case .Convex, .Cached_Convex, .Nonconvex, .Joints: return "prepare-and-native-solver";
	case .Nonconserving: return "dispatch-reset-gather-four-substeps";
	case .Pose: return "prepare-and-unconstrained-pose-stage";
	case .Constrained_Final: return "prepare-and-dispatched-constrained-final-bundles";
	case .Free_Final: return "prepare-and-free-after-substeps";
	case .Kinematic: return "prepare-and-dispatched-kinematic-bundles";
	case .Conserving: return "reset-and-four-substeps";
	}
	unreachable();
}

fixed_work_units :: proc(s: ^Fixed_State, calls: int) -> i64
{
	if s.points > 0 && s.options.case_name != .Constrained_Final
	{
		return i64(calls)*i64(s.points)*i64(s.options.velocity_iterations)*i64(s.options.substeps);
	}
	if s.options.case_name == .Joints
	{
		return i64(calls)*i64(s.solver.active_set.constraint_count)*4;
	}
	factor: int = 4 if s.options.case_name == .Nonconserving || s.options.case_name == .Kinematic else 1;
	if s.options.case_name == .Free_Final && s.options.unconstrained_substeps == .Enabled
	{
		factor = 4;
	}
	return i64(calls)*i64(s.admitted_units)*i64(factor);
}

fixed_observe :: proc(s: ^Fixed_State) -> Fixed_Quality
{
	q: Fixed_Quality;
	active: ^physics.Body_Set = &s.bodies.sets.memory[physics.BODIES_ACTIVE_SET_INDEX];
	for i in 0 ..< active.count
	{
		body: ^physics.Body_Dynamics = &active.dynamics_state.memory[i];
		values: [20]f32 = {body.motion.pose.position.x, body.motion.pose.position.y, body.motion.pose.position.z,
			body.motion.pose.orientation.x, body.motion.pose.orientation.y, body.motion.pose.orientation.z, body.motion.pose.orientation.w,
			body.motion.velocity.linear.x, body.motion.velocity.linear.y, body.motion.velocity.linear.z,
			body.motion.velocity.angular.x, body.motion.velocity.angular.y, body.motion.velocity.angular.z,
			body.inertia.world.inverse_mass, body.inertia.world.inverse_inertia_tensor.xx, body.inertia.world.inverse_inertia_tensor.yx,
			body.inertia.world.inverse_inertia_tensor.yy, body.inertia.world.inverse_inertia_tensor.zx,
			body.inertia.world.inverse_inertia_tensor.zy, body.inertia.world.inverse_inertia_tensor.zz};
		for value in values
		{
			if contacts.quality_finite(f64(value)) == .Missing
			{
				q.failures += 1;
			}
		}
		rotation: util.Quaternion = body.motion.pose.orientation;
		error: f64 = abs(f64(rotation.x)*f64(rotation.x)+f64(rotation.y)*f64(rotation.y)+f64(rotation.z)*f64(rotation.z)+f64(rotation.w)*f64(rotation.w)-1);
		q.orientation_norm_error = max(q.orientation_norm_error, error);
	}
	if s.points > 0
	{
		fixed_observe_contacts(s, &q);
	}
	else
	{
		q.observed_units = s.admitted_units;
		fixed_observe_joints(s, &q);
		for canonical in s.canonical
		{
			kind: ^physics.Type_Batch = canonical.native;
			for i in 0 ..< kind.count
			{
				values: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(kind, int(i)));
				for component in 0 ..< int(kind.impulse_bundle_size)/size_of(util.F32x8)
				{
					if contacts.quality_finite(f64(simd.extract(values[component], int(i)%8))) == .Missing
					{
						q.failures += 1;
					}
				}
			}
		}
	}
	return q;
}

fixed_measure :: proc(o: Options) -> Fixed_Measurement
{
	s: Fixed_State;
	defer fixed_dispose(&s);
	m: Fixed_Measurement;
	m.status = fixed_create(&s, o);
	if m.status != .Ok
	{
		return m;
	}
	dispatcher: util.Thread_Dispatcher;
	if util.thread_dispatcher_initialize(&dispatcher, o.workers, 65536) != .Ok
	{
		m.status = .Capacity_Missing;
		return m;
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	for pass: int = -o.warmup; pass < o.passes; pass += 1
	{
		if pass == 0
		{
			m.pool_before = util.buffer_pool_total_allocated_byte_count(&s.pool);
			m.worker_pool_before = util.worker_buffer_pools_total_allocated_byte_count(&dispatcher.worker_pools);
		}
		if o.case_name == .Nonconserving
		{
			s.pose_context = s.canonical_context;
		}
		else
		{
			fixed_restore(&s);
		}
		start: time.Tick = time.tick_now();
		m.status = fixed_execute(&s, boundary);
		elapsed_ms: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
		if pass >= 0
		{
			m.elapsed_ms += elapsed_ms;
		}
		if m.status != .Ok
		{
			if pass < 0
			{
				return m;
			}
			break;
		}
		q: Fixed_Quality = fixed_observe(&s);
		if pass < 0
		{
			continue;
		}
		m.completed += 1;
		m.quality_failures += int(q.failures > 0);
		m.quality.failures += q.failures;
		m.quality.observed_units = q.observed_units;
		m.quality.residual_max_mps = max(m.quality.residual_max_mps, q.residual_max_mps);
		m.quality.residual_rms_mps = max(m.quality.residual_rms_mps, q.residual_rms_mps);
		m.quality.residual_p95_mps = max(m.quality.residual_p95_mps, q.residual_p95_mps);
		m.quality.position_error_m = max(m.quality.position_error_m, q.position_error_m);
		m.quality.orientation_norm_error = max(m.quality.orientation_norm_error, q.orientation_norm_error);
		m.quality.joint_angle_error_rad = max(m.quality.joint_angle_error_rad, q.joint_angle_error_rad);
	}
	m.pose_context = s.canonical_context;
	m.bodies = len(s.initial);
	m.dynamic_bodies = s.dynamic_bodies;
	m.kinematic_bodies = s.kinematic_bodies;
	m.admitted_units = s.admitted_units;
	m.constraints = int(s.solver.active_set.constraint_count);
	m.points = s.points;
	m.work_units = fixed_work_units(&s, m.completed);
	m.pool_after = util.buffer_pool_total_allocated_byte_count(&s.pool);
	m.worker_pool_after = util.worker_buffer_pools_total_allocated_byte_count(&dispatcher.worker_pools);
	return m;
}
