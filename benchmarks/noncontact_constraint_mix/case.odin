package noncontact_constraint_mix

import support "../benchmark_support"
import entasis "entasis:entasis"
import "base:runtime"

CASE_ID :: "noncontact_constraint_mix";
CONSTRAINTS_PER_TYPE :: 128;
BUILT_IN_NONCONTACT_TYPE_COUNT :: 30;
BODY_SLOTS_PER_TYPE_SET :: 59;
PROTOCOL_CONSTRAINT_COUNT :: CONSTRAINTS_PER_TYPE * BUILT_IN_NONCONTACT_TYPE_COUNT;
PROTOCOL_BODY_COUNT :: CONSTRAINTS_PER_TYPE * BODY_SLOTS_PER_TYPE_SET;
WARMUP_STEP_COUNT :: 30;
MEASURED_STEP_COUNT :: 300;
TIMESTEP_DURATION :: f32(1.0 / 60.0);
MAXIMUM_WORKER_COUNT :: entasis.MAXIMUM_WORKER_COUNT;
WORKER_POOL_BLOCK_SIZE :: 65536;
POOL_MINIMUM_BLOCK_SIZE :: 65536;
PINNED_FALLBACK_BATCH_INDEX :: 64;
CONSTRAINT_CAPACITY :: 16384;
BROAD_PHASE_TRANSACTION_CAPACITY :: 1024;
PAIR_CACHE_CAPACITY :: 1024;
PENDING_PAIR_CAPACITY_PER_WORKER :: 64;

#assert(PROTOCOL_CONSTRAINT_COUNT == 3840);
#assert(PROTOCOL_BODY_COUNT == 7552);
#assert(CONSTRAINT_CAPACITY >= PROTOCOL_CONSTRAINT_COUNT);

Case_Status :: enum u8
{
	Ok,
	Invalid_Argument,
	Allocation_Failed,
	Creation_Failed,
	Fixture_Failed,
	Step_Failed,
	Validation_Failed,
	Release_Failed,
}

Benchmark_Step_Phase :: enum u8
{
	None,
	Warmup,
	Measured,
}

Benchmark_Run_Result :: struct
{
	status:                    Case_Status,
	physics_status:            entasis.Status,
	step_phase:                Benchmark_Step_Phase,
	failed_step:               int,
	exhausted_pool_power_mask: u32,
}

Owner_State :: enum u8
{
	Empty,
	Ready,
	Released,
}

Benchmark_Simulation_Owner :: struct
{
	world: entasis.World,
	state: Owner_State,
	joints: []entasis.Constraint_Handle,
	joint_count: int,
	recording_bodies: [dynamic]entasis.Body_Handle,
}

Stability_Counters :: struct
{
	invalid_state_count: int,
	dynamic_body_count:  int,
	constraint_count:    int,
}

benchmark_world_description :: proc(
	worker_count: int,
) -> entasis.World_Description
{
	description := entasis.world_description_default();
	description.gravity = {};
	description.damping = {};
	description.capacity = {
		bodies=PROTOCOL_BODY_COUNT,
		statics=5,
		inactive_body_sets=1,
		shapes_per_type=4,
		constraints=CONSTRAINT_CAPACITY,
		initial_constraints_per_type_batch=CONSTRAINTS_PER_TYPE,
		minimum_constraints_per_body=8,
		broad_phase_candidates=BROAD_PHASE_TRANSACTION_CAPACITY,
		pairs=PAIR_CACHE_CAPACITY,
		collision_child_pairs=i32(worker_count),
		inactive_pairs=1,
		pending_pairs_per_worker=PENDING_PAIR_CAPACITY_PER_WORKER,
	};
	description.solve = {
		velocity_iterations=8,
		substeps=1,
		fallback_batch_threshold=PINNED_FALLBACK_BATCH_INDEX,
	};
	description.threading = {
		worker_count=i32(worker_count),
		worker_pool_block_size=WORKER_POOL_BLOCK_SIZE,
	};
	return description;
}

benchmark_body_description :: #force_inline proc "contextless" (
	position: entasis.Vector3,
) -> entasis.Body_Description
{
	inertia := entasis.Body_Inertia{
		inverse_inertia_tensor={xx=1, yy=1, zz=1},
		inverse_mass=1,
	};
	return entasis.body_shapeless(
		inertia,
		entasis.pose(position),
		{},
		entasis.body_activity(-1, 32),
	);
}

benchmark_add_constraint_group :: proc(
	owner: ^Benchmark_Simulation_Owner,
	description: ^$T,
	positions: [4]entasis.Vector3,
	$body_count: int,
) -> Case_Status
{
	for _ in 0 ..< CONSTRAINTS_PER_TYPE
	{
		handles: [4]entasis.Body_Handle;
		for body_index in 0 ..< body_count
		{
			body_description := benchmark_body_description(positions[body_index]);
			handle, body_status := entasis.body_add(&owner.world, body_description);
			if body_status != .Ok
			{
				return .Fixture_Failed;
			}
			when support.RECORDING_ENABLED
			{
				append(&owner.recording_bodies, handle);
			}
			handles[body_index] = handle;
		}
		joint, constraint_status := entasis.constraint_add(
			&owner.world,
			handles[:body_count],
			description^,
		);
		if constraint_status != .Ok
		{
			return .Fixture_Failed;
		}
		owner.joints[owner.joint_count] = joint;
		owner.joint_count += 1;
	}
	return .Ok;
}

benchmark_build_fixture :: proc(owner: ^Benchmark_Simulation_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	spring := entasis.spring_settings(30, 1);
	servo := entasis.Servo_Settings{maximum_speed=100, base_speed=0, maximum_force=1000};
	motor := entasis.Motor_Settings{maximum_force=1000, damping=1};
	identity := entasis.Quaternion{w=1};
	x_axis := entasis.Vector3{1, 0, 0};
	y_axis := entasis.Vector3{0, 1, 0};
	one_body := [4]entasis.Vector3{{0, 0, 0}, {}, {}, {}};
	two_body := [4]entasis.Vector3{{0, 0, 0}, {1, 0, 0}, {}, {}};
	three_body := [4]entasis.Vector3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {}};
	four_body := [4]entasis.Vector3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	angular_axis_gear_motor := entasis.Angular_Axis_Gear_Motor{local_axis_a=x_axis, velocity_scale=1, settings=motor};
	if benchmark_add_constraint_group(owner, &angular_axis_gear_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	angular_axis_motor := entasis.Angular_Axis_Motor{local_axis_a=x_axis, target_velocity=0, settings=motor};
	if benchmark_add_constraint_group(owner, &angular_axis_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	angular_hinge := entasis.Angular_Hinge{
		local_hinge_axis_a=x_axis,
		local_hinge_axis_b=x_axis,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &angular_hinge, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	angular_motor := entasis.Angular_Motor{target_velocity_local_a={}, settings=motor};
	if benchmark_add_constraint_group(owner, &angular_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	angular_servo := entasis.Angular_Servo{
		target_relative_rotation_local_a=identity,
		spring_settings=spring,
		servo_settings=servo,
	};
	if benchmark_add_constraint_group(owner, &angular_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	angular_swivel_hinge := entasis.Angular_Swivel_Hinge{
		local_swivel_axis_a=x_axis,
		local_hinge_axis_b=y_axis,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &angular_swivel_hinge, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	area_constraint := entasis.Area_Constraint{target_scaled_area=1, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &area_constraint, three_body, 3) != .Ok
	{
		return .Fixture_Failed;
	}
	ball_socket := entasis.Ball_Socket{local_offset_a={0.5, 0, 0}, local_offset_b={-0.5, 0, 0}, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &ball_socket, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	ball_socket_motor := entasis.Ball_Socket_Motor{local_offset_b={}, target_velocity_local_a={}, settings=motor};
	if benchmark_add_constraint_group(owner, &ball_socket_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	ball_socket_servo := entasis.Ball_Socket_Servo{
		local_offset_a={0.5, 0, 0},
		local_offset_b={-0.5, 0, 0},
		spring_settings=spring,
		servo_settings=servo,
	};
	if benchmark_add_constraint_group(owner, &ball_socket_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	center_distance_constraint := entasis.Center_Distance_Constraint{target_distance=1, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &center_distance_constraint, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	center_distance_limit := entasis.Center_Distance_Limit{
		minimum_distance=0.5,
		maximum_distance=1.5,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &center_distance_limit, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	distance_limit := entasis.Distance_Limit{minimum_distance=0.5, maximum_distance=1.5, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &distance_limit, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	distance_servo := entasis.Distance_Servo{target_distance=1, servo_settings=servo, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &distance_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	hinge := entasis.Hinge{
		local_offset_a={0.5, 0, 0},
		local_hinge_axis_a=x_axis,
		local_offset_b={-0.5, 0, 0},
		local_hinge_axis_b=x_axis,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &hinge, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	linear_axis_limit := entasis.Linear_Axis_Limit{
		local_axis=x_axis,
		minimum_offset=0.5,
		maximum_offset=1.5,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &linear_axis_limit, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	linear_axis_motor := entasis.Linear_Axis_Motor{local_axis=x_axis, target_velocity=0, settings=motor};
	if benchmark_add_constraint_group(owner, &linear_axis_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	linear_axis_servo := entasis.Linear_Axis_Servo{
		local_plane_normal=x_axis,
		target_offset=1,
		servo_settings=servo,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &linear_axis_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	one_body_angular_motor := entasis.One_Body_Angular_Motor{target_velocity={}, settings=motor};
	if benchmark_add_constraint_group(owner, &one_body_angular_motor, one_body, 1) != .Ok
	{
		return .Fixture_Failed;
	}
	one_body_angular_servo := entasis.One_Body_Angular_Servo{
		target_orientation=identity,
		spring_settings=spring,
		servo_settings=servo,
	};
	if benchmark_add_constraint_group(owner, &one_body_angular_servo, one_body, 1) != .Ok
	{
		return .Fixture_Failed;
	}
	one_body_linear_motor := entasis.One_Body_Linear_Motor{target_velocity={}, settings=motor};
	if benchmark_add_constraint_group(owner, &one_body_linear_motor, one_body, 1) != .Ok
	{
		return .Fixture_Failed;
	}
	one_body_linear_servo := entasis.One_Body_Linear_Servo{target={}, spring_settings=spring, servo_settings=servo};
	if benchmark_add_constraint_group(owner, &one_body_linear_servo, one_body, 1) != .Ok
	{
		return .Fixture_Failed;
	}
	point_on_line_servo := entasis.Point_On_Line_Servo{
		local_direction=x_axis,
		servo_settings=servo,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &point_on_line_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	swing_limit := entasis.Swing_Limit{axis_local_a=x_axis, axis_local_b=x_axis, minimum_dot=0, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &swing_limit, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	swivel_hinge := entasis.Swivel_Hinge{
		local_offset_a={0.5, 0, 0},
		local_swivel_axis_a=x_axis,
		local_offset_b={-0.5, 0, 0},
		local_hinge_axis_b=y_axis,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &swivel_hinge, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	twist_limit := entasis.Twist_Limit{
		local_basis_a=identity,
		local_basis_b=identity,
		minimum_angle=-0.5,
		maximum_angle=0.5,
		spring_settings=spring,
	};
	if benchmark_add_constraint_group(owner, &twist_limit, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	twist_motor := entasis.Twist_Motor{local_axis_a=x_axis, local_axis_b=x_axis, target_velocity=0, settings=motor};
	if benchmark_add_constraint_group(owner, &twist_motor, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	twist_servo := entasis.Twist_Servo{
		local_basis_a=identity,
		local_basis_b=identity,
		target_angle=0,
		spring_settings=spring,
		servo_settings=servo,
	};
	if benchmark_add_constraint_group(owner, &twist_servo, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}
	volume_constraint := entasis.Volume_Constraint{target_scaled_volume=1, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &volume_constraint, four_body, 4) != .Ok
	{
		return .Fixture_Failed;
	}
	weld := entasis.Weld{local_offset={1, 0, 0}, local_orientation=identity, spring_settings=spring};
	if benchmark_add_constraint_group(owner, &weld, two_body, 2) != .Ok
	{
		return .Fixture_Failed;
	}

	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok || stats.active_bodies != PROTOCOL_BODY_COUNT ||
		stats.sleeping_bodies != 0 || stats.active_constraints != PROTOCOL_CONSTRAINT_COUNT
	{
		return .Fixture_Failed;
	}
	return .Ok;
}

benchmark_owner_create :: proc(
	owner: ^Benchmark_Simulation_Owner,
	pool: ^entasis.Buffer_Pool,
	worker_count: int,
) -> Case_Status
{

	if owner == nil || pool == nil || owner.state != .Empty ||
		worker_count <= 0 || worker_count > MAXIMUM_WORKER_COUNT
	{
		return .Invalid_Argument;
	}
	status := entasis.world_init_with_pool(
		&owner.world,
		benchmark_world_description(worker_count),
		pool,
	);
	if status != .Ok
	{
		return .Creation_Failed;
	}
	owner.state = .Ready;
	when support.RECORDING_ENABLED
	{
		if reserve(&owner.recording_bodies, PROTOCOL_BODY_COUNT) != nil
		{
			benchmark_owner_destroy(owner);
			return .Allocation_Failed;
		}
	}
	allocation_error: runtime.Allocator_Error;
	owner.joints, allocation_error = make([]entasis.Constraint_Handle, PROTOCOL_CONSTRAINT_COUNT);
	if allocation_error != nil
	{
		benchmark_owner_destroy(owner);
		return .Allocation_Failed;
	}
	solver_stats, solver_status := entasis.world_solver_stats(&owner.world);
	if solver_status != .Ok || solver_stats.fallback_batch_index != PINNED_FALLBACK_BATCH_INDEX
	{
		_ = benchmark_owner_destroy(owner);
		return .Creation_Failed;
	}
	fixture_status := benchmark_build_fixture(owner);
	if fixture_status != .Ok
	{
		_ = benchmark_owner_destroy(owner);
		return fixture_status;
	}
	return .Ok;
}

benchmark_owner_step :: #force_inline proc(
	owner: ^Benchmark_Simulation_Owner,
	step_count: int,
	phase: Benchmark_Step_Phase,
) -> Benchmark_Run_Result
{
	if owner == nil || owner.state != .Ready || step_count <= 0 || phase == .None
	{
		return {status=.Invalid_Argument, physics_status=.Invalid_Argument, step_phase=phase};
	}
	for step_index in 0 ..< step_count
	{
		step_status := entasis.world_step(&owner.world, TIMESTEP_DURATION);
		if step_status != .Ok
		{
			return {
				status=.Step_Failed,
				physics_status=step_status,
				step_phase=phase,
				failed_step=step_index + 1,
			};
		}
	}
	return {status=.Ok, physics_status=.Ok, step_phase=phase};
}

benchmark_owner_destroy :: proc(owner: ^Benchmark_Simulation_Owner) -> Case_Status
{
	if owner == nil || owner.state != .Ready
	{
		return .Invalid_Argument;
	}
	status := entasis.world_destroy(&owner.world);
	delete(owner.joints);
	when support.RECORDING_ENABLED
	{
		delete(owner.recording_bodies);
	}
	owner^ = {state=.Released};
	if status != .Ok
	{
		return .Release_Failed;
	}
	return .Ok;
}

benchmark_f32_validation_status :: #force_inline proc "contextless" (value: f32) -> Case_Status
{
	if transmute(u32)value & 0x7f80_0000 != 0x7f80_0000
	{
		return .Ok;
	}
	return .Validation_Failed;
}

benchmark_count_stability :: proc(
	owner: ^Benchmark_Simulation_Owner,
	counters: ^Stability_Counters,
) -> Case_Status
{
	if owner == nil || owner.state != .Ready || counters == nil
	{
		return .Invalid_Argument;
	}
	counters^ = {};
	view, view_status := entasis.active_body_view(&owner.world);
	if view_status != .Ok
	{
		return .Validation_Failed;
	}
	for body_index in 0 ..< view.count
	{
		state := view.dynamics[body_index].motion;
		p := state.pose.position;
		q := state.pose.orientation;
		linear := state.velocity.linear;
		angular := state.velocity.angular;
		counters.dynamic_body_count += 1;
		if benchmark_f32_validation_status(p.x) != .Ok ||
			benchmark_f32_validation_status(p.y) != .Ok ||
			benchmark_f32_validation_status(p.z) != .Ok ||
			benchmark_f32_validation_status(q.x) != .Ok ||
			benchmark_f32_validation_status(q.y) != .Ok ||
			benchmark_f32_validation_status(q.z) != .Ok ||
			benchmark_f32_validation_status(q.w) != .Ok ||
			benchmark_f32_validation_status(linear.x) != .Ok ||
			benchmark_f32_validation_status(linear.y) != .Ok ||
			benchmark_f32_validation_status(linear.z) != .Ok ||
			benchmark_f32_validation_status(angular.x) != .Ok ||
			benchmark_f32_validation_status(angular.y) != .Ok ||
			benchmark_f32_validation_status(angular.z) != .Ok
		{
			counters.invalid_state_count += 1;
		}
	}
	stats, stats_status := entasis.world_stats(&owner.world);
	if stats_status != .Ok
	{
		return .Validation_Failed;
	}
	counters.constraint_count = stats.active_constraints;

	if counters.dynamic_body_count != PROTOCOL_BODY_COUNT ||
		stats.sleeping_bodies != 0 || counters.constraint_count != PROTOCOL_CONSTRAINT_COUNT ||
		counters.invalid_state_count != 0
	{
		return .Validation_Failed;
	}
	return .Ok;
}
