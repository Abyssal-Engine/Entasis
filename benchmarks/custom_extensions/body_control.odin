package custom_extensions

@(require)
import "core:fmt"
@(require)
import "core:math"
@(require)
import "core:time"
@(require)
import e "entasis:entasis"
@(require)
import util "entasis:entasis_utilities"
import support "../benchmark_support"

when support.BENCHMARK_COMPONENTS == "all"
{
	Body_Control_Case :: enum u8
	{
		Disabled,
		Enabled_Empty,
		Force_Damping,
		Kinematic_Target,
		Axis_Locks,
	}
	BODY_CONTROL_NAMES: [5]string = {"disabled", "enabled_empty", "force_damping", "kinematic_target", "axis_locks"};
	Body_Control_Owner :: struct
	{
		world: e.World,
		description: e.World_Description,
		bodies: [BODY_COUNT]e.Body_Handle,
		origins: [BODY_COUNT]e.Vector3,
	}
	Body_Control_Result :: struct
	{
		submission_ms, step_ms: f64,
		submissions, active_bodies, active_constraints: int,
		position_checksum, velocity_checksum: f64,
		world_pool_bytes, worker_pool_bytes: u64,
	}

	body_control_owner_init :: proc(owner: ^Body_Control_Owner, workers: int, mode: Body_Control_Case) -> e.Status
	{
		description: e.World_Description = e.world_description_default();
		description.gravity = {};
		description.damping = {};
		description.threading.worker_count = i32(workers);
		description.capacity.bodies = BODY_COUNT;
		description.capacity.constraints = BODY_COUNT;
		description.capacity.initial_constraints_per_type_batch = BODY_COUNT;
		description.solve = {velocity_iterations=4, substeps=1, fallback_batch_threshold=64};
		owner.description = description;
		if failure: e.Status = e.world_init(&owner.world, description); failure != .Ok
		{
			return failure;
		}
		if mode != .Disabled
		{
			if failure: e.Status = e.world_enable_body_control(&owner.world, {BODY_COUNT, BODY_COUNT}); failure != .Ok
			{
				return failure;
			}
		}
		shape: e.Shape_Handle;
		status: e.Status;
		shape, status = e.shape_add(&owner.world, e.sphere(0.5));
		if status != .Ok
		{
			return status;
		}
		inertia: e.Body_Inertia;
		inertia, status = e.shape_inertia(e.sphere(0.5), 1);
		if status != .Ok
		{
			return status;
		}
		for &body, index in owner.bodies
		{
			owner.origins[index] = {f32(index%32)*4, 0, f32(index/32)*4};
			pose: e.Rigid_Pose = e.pose(owner.origins[index]);
			body_description: e.Body_Description;
			if mode == .Kinematic_Target
			{
				body_description = e.body_kinematic(shape, pose, {}, e.body_activity(-1, 255));
			}
			else
			{
				body_description = e.body_dynamic(shape, inertia, pose, e.velocity({1, 0, 0}), e.body_activity(-1, 255));
			}
			body, status = e.body_add(&owner.world, body_description);
			if status != .Ok
			{
				return status;
			}
			if mode == .Force_Damping
			{
				if failure: e.Status = e.body_set_damping(&owner.world, body, {linear=0.5, angular=0.25, mode=.Additional}); failure != .Ok
				{
					return failure;
				}
			}
			if mode == .Axis_Locks
			{
				lock: e.Body_Axis_Lock = e.body_axis_lock_default(pose);
				lock.linear_axes = {.Y};
				lock.angular_axes = {.X, .Y, .Z};
				if failure: e.Status = e.body_set_axis_lock(&owner.world, body, lock); failure != .Ok
				{
					return failure;
				}
			}
		}
		if mode != .Disabled
		{
			if failure: e.Status = e.body_control_reserve(&owner.world, BODY_COUNT, BODY_COUNT); failure != .Ok
			{
				return failure;
			}
		}
		return .Ok;
	}

	body_control_validate :: proc(owner: ^Body_Control_Owner, mode: Body_Control_Case, steps: int, axis_before: e.Body_State, result: ^Body_Control_Result) -> e.Status
	{
		stats: e.World_Stats;
		status: e.Status;
		stats, status = e.world_stats(&owner.world);
		if status != .Ok
		{
			return status;
		}
		expected_constraints: int = 0;
		if mode == .Axis_Locks
		{
			expected_constraints = BODY_COUNT;
		}
		if stats.active_bodies != BODY_COUNT || stats.sleeping_bodies != 0 || stats.active_pairs != 0 || stats.active_constraints != expected_constraints
		{
			return .Invalid_Argument;
		}
		result.active_bodies = stats.active_bodies;
		result.active_constraints = stats.active_constraints;
		expected_velocity: f32 = 1;
		if mode == .Force_Damping
		{
			factor: f32 = f32(math.exp(-0.5*f64(DT)));
			for _ in 0..<steps
			{
				expected_velocity = (expected_velocity+2*DT)*factor;
			}
		}
		else if mode == .Axis_Locks
		{
			expected_velocity += 2*f32(steps)*DT;
		}
		expected_axis_linear, expected_axis_angular, axis_response_tolerance: f64;
		if mode == .Axis_Locks
		{
			lock: e.Body_Axis_Lock = e.body_axis_lock_default(e.pose());
			omega: f64 = f64(lock.spring_settings.angular_frequency);
			omega_dt: f64 = omega*f64(DT);
			damping: f64 = f64(lock.spring_settings.twice_damping_ratio);
			softness: f64 = 1/(1+omega_dt*(omega_dt+damping));
			position_scale: f64 = min(omega/(omega_dt+damping), 1/f64(DT));
			// isolated spring row: v' = s*(v+a*dt) - (1-s)*p*error.
			// Warmstart cancels algebraically. the identical spheres have equal
			// locked coordinates. use scalar math, not the engine's SIMD helpers.
			// stored unit quaternions are rounded f32 values. clamp w for acos
			// without renormalizing the orientation consumed by the solver
			angle: f64 = 2*math.acos(min(f64(1), abs(f64(axis_before.pose.orientation.w))));
			if axis_before.pose.orientation.x*axis_before.pose.orientation.w < 0
			{
				angle = -angle;
			}
			expected_axis_linear = softness*(f64(axis_before.velocity.linear.y)+3*f64(DT)) -
				(1-softness)*position_scale*f64(axis_before.pose.position.y);
			// a unit-mass sphere of radius 0.5 has inverse inertia 10
			expected_axis_angular = softness*(f64(axis_before.velocity.angular.x)+10*f64(DT)) -
				(1-softness)*position_scale*angle;
			// allow f32 row/inertia arithmetic roundoff, scaled by the input
			axis_response_tolerance = 64*math.F32_EPSILON*max(f64(1), abs(f64(axis_before.velocity.angular.x)+10*f64(DT)));
		}
		for body, index in owner.bodies
		{
			state: e.Body_State;
			state, status = e.body_get(&owner.world, body);
			if status != .Ok
			{
				return status;
			}
			displacement: f32 = state.pose.position.x-owner.origins[index].x;
			if !(displacement > 0 && abs(state.velocity.linear.x-expected_velocity) < 0.002) ||
				abs(state.pose.position.z-owner.origins[index].z) > 0.0002
			{
				fmt.eprintfln("body_control_motion_failed mode=%v body=%d displacement=%v velocity=%v expected=%v", mode, index, displacement, state.velocity.linear.x, expected_velocity);
				return .Invalid_Argument;
			}
			if mode == .Disabled || mode == .Enabled_Empty || mode == .Kinematic_Target
			{
				if abs(displacement-f32(steps)*DT) > 0.004
				{
					return .Invalid_Argument;
				}
			}
			if mode == .Axis_Locks
			{
				if math.is_nan(axis_response_tolerance) || math.is_inf(axis_response_tolerance) ||
					!(abs(state.pose.position.y) <= 0.002 &&
					abs(state.pose.orientation.x)+abs(state.pose.orientation.y)+abs(state.pose.orientation.z) <= 0.002 &&
					abs(abs(state.pose.orientation.w)-1) <= 0.002 &&
					abs(f64(state.velocity.linear.y)-expected_axis_linear) <= axis_response_tolerance &&
					abs(f64(state.velocity.angular.x)-expected_axis_angular)+abs(f64(state.velocity.angular.y))+abs(f64(state.velocity.angular.z)) <= axis_response_tolerance)
				{
					fmt.eprintfln("body_control_lock_failed body=%d steps=%d dt=%v position=%v orientation=%v linear_velocity=%v angular_velocity=%v origin=%v expected_linear_y=%v expected_angular_x=%v response_tolerance=%v before=%v",
						index, steps, DT, state.pose.position, state.pose.orientation, state.velocity.linear, state.velocity.angular, owner.origins[index], expected_axis_linear, expected_axis_angular, axis_response_tolerance, axis_before);
					return .Invalid_Argument;
				}
			}
			result.position_checksum += f64(displacement);
			result.velocity_checksum += f64(state.velocity.linear.x);
		}
		pool: ^e.Buffer_Pool;
		pool, status = e.world_borrow_pool(&owner.world);
		if status != .Ok
		{
			return status;
		}
		result.world_pool_bytes = util.buffer_pool_total_allocated_byte_count(pool);
		dispatcher: ^e.Dispatcher;
		dispatcher, status = e.world_borrow_dispatcher(&owner.world);
		if status != .Ok
		{
			return status;
		}
		if dispatcher != nil
		{
			for index in 0..<dispatcher.worker_count
			{
				worker: ^util.Buffer_Pool;
				pool_status: util.Threading_Status;
				worker, pool_status = dispatcher.worker_pool(dispatcher, index);
				if pool_status != .Ok
				{
					return .Invalid_Argument;
				}
				result.worker_pool_bytes += util.buffer_pool_total_allocated_byte_count(worker);
			}
		}
		return .Ok;
	}

	body_control_targets :: proc(owner: ^Body_Control_Owner, step: int, targets: []e.Rigid_Pose)
	{
		for &target, index in targets
		{
			target = e.pose({owner.origins[index].x+f32(step+1)*DT, 0, owner.origins[index].z});
		}
	}

	body_control_submit :: #force_inline proc(owner: ^Body_Control_Owner, mode: Body_Control_Case, targets: []e.Rigid_Pose, submission_statuses: []e.Status) -> int
	{
		submission_count: int;
		switch mode
		{
		case .Force_Damping:
			for body, index in owner.bodies
			{
				submission_statuses[index] = e.body_add_force(&owner.world, body, {2, 0, 0});
			}
			submission_count = BODY_COUNT;
		case .Kinematic_Target:
			for body, index in owner.bodies
			{
				submission_statuses[index] = e.body_set_kinematic_target(&owner.world, body, targets[index]);
			}
			submission_count = BODY_COUNT;
		case .Axis_Locks:
			for body, index in owner.bodies
			{
				submission_statuses[index*2] = e.body_add_force(&owner.world, body, {2, 3, 0});
				submission_statuses[index*2+1] = e.body_add_torque(&owner.world, body, {1, 0, 0});
			}
			submission_count = BODY_COUNT*2;
		case .Disabled, .Enabled_Empty:
		}
		return submission_count;
	}

	body_control_measure_scene :: proc(workers: int, mode: Body_Control_Case, steps: int) -> Body_Control_Result
	{
		owner: Body_Control_Owner;
		support.extension_require(body_control_owner_init(&owner, workers, mode), "body control create");
		result: Body_Control_Result;
		submission_statuses: [BODY_COUNT*2]e.Status;
		targets: [BODY_COUNT]e.Rigid_Pose;
		axis_before: e.Body_State;
		for step in 0..<steps
		{
			if mode == .Axis_Locks && step == steps-1
			{
				status: e.Status;
				axis_before, status = e.body_get(&owner.world, owner.bodies[0]);
				support.extension_require(status, "body control spring oracle input");
			}
			if mode == .Kinematic_Target
			{
				body_control_targets(&owner, step, targets[:]);
			}
			submission_count: int = 0;
			if mode == .Force_Damping || mode == .Kinematic_Target || mode == .Axis_Locks
			{
				start: time.Tick = time.tick_now();
				submission_count = body_control_submit(&owner, mode, targets[:], submission_statuses[:]);
				result.submission_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
			}
			for status in submission_statuses[:submission_count]
			{
				support.extension_require(status, "body control submission");
			}
			result.submissions += submission_count;
			start: time.Tick = time.tick_now();
			status: e.Status = e.world_step(&owner.world, DT);
			result.step_ms += f64(time.tick_diff(start, time.tick_now()))/1e6;
			support.extension_require(status, "body control measured step");
		}
		support.extension_require(body_control_validate(&owner, mode, steps, axis_before, &result), "body control validation");
		support.extension_require(e.world_destroy(&owner.world), "body control destroy");
		if result.step_ms <= 0 || (result.submissions > 0 && result.submission_ms <= 0)
		{
			panic("invalid body control timing");
		}
		return result;
	}

	measure_body_control_components :: proc(workers: int) -> [5]Body_Control_Result
	{
		results: [5]Body_Control_Result;
		for &result, index in results
		{
			mode: Body_Control_Case = Body_Control_Case(index);
			_ = body_control_measure_scene(workers, mode, WARMUP_STEPS);
			result = body_control_measure_scene(workers, mode, MEASURED_STEPS);
		}
		return results;
	}
}
