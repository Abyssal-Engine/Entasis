package precision_constraints_tests

import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Execution_Route :: enum u8
{
	Direct_Bundle,
	Registered_Bundle,
	Direct_Lane,
	Registered_Lane,
	Cached_Ball_Socket,
}

Joint_Fixture :: struct
{
	bodies: [17][4]physics.Body_Handle,
	joints: [17]physics.Constraint_Handle,
}

populate_joints :: proc(t: ^testing.T, scene: ^Scene, fixture: ^Joint_Fixture, origin: physics.Global_Position,
	description: ^$T, $body_count: int, count: int = 17)
{
	offsets: [4]util.Vector3 = {{}, {1.5, 0.75, -0.25}, {-0.75, 1.75, 0.5}, {0.125, -0.25, 2.125}};
	for group in 0 ..< count
	{
		for body in 0 ..< body_count
		{
			position: physics.Global_Position = {origin.x + f64(group * 4) + 0.03125 + f64(offsets[body].x),
				origin.y + f64(group % 3 * 2) + 0.125 + f64(offsets[body].y), origin.z - 0.0625 + f64(offsets[body].z)};
			mobility: physics.Body_Mobility = .Dynamic;
			if body == body_count - 1 && group % 2 == 1
			{
				mobility = .Kinematic;
			}
			handle: physics.Body_Handle = add_body(t, scene, position, mobility);
			fixture.bodies[group][body] = handle;
			motion: ^physics.Motion_State = body_motion(t, scene, handle);
			motion.velocity = {linear={f32(body) * 0.125, -0.25, f32(group % 3) * 0.25}, angular={0.125, f32(body) * -0.0625, 0.25}};
		}
		status: physics.Physics_Status;
		fixture.joints[group], status = physics.solver_add(&scene.simulation.solver, &fixture.bodies[group], description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
}

execute_stage :: proc(t: ^testing.T, scene: ^Scene, anchor: physics.Constraint_Handle, route: Execution_Route,
	stage: physics.Solver_Execution_Stage, $type_id: i32, $body_count: int)
{
	solver: ^physics.Solver = &scene.simulation.solver;
	batch: ^physics.Type_Batch = constraint_batch(t, scene, anchor);
	record: ^physics.Constraint_Type_Record = &solver.registry.records[type_id];
	status: physics.Physics_Status;
	if route == .Cached_Ball_Socket
	{
		when type_id == physics.BALL_SOCKET_TYPE_ID
		{
			job: physics.Solver_Substep_Job = {solver=solver, dt=1.0 / 64, inverse_dt=64, phase=.First};
			block: physics.Solver_Work_Block = {start_bundle=0, end_bundle=(batch.count + 7) / 8};
			status = physics.solver_execute_ball_socket_work_block_integration(&job, batch, &block, stage, 0, .Never);
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
		return;
	}
	if route == .Direct_Bundle || route == .Registered_Bundle
	{
		for bundle in 0 ..< (int(batch.count) + 7) / 8
		{
			if route == .Direct_Bundle
			{
				status = physics.solver_execute_direct_noncontact_bundle_integration(solver, batch, 0, bundle,
					1.0 / 64, 64, .First, stage, 0, type_id, body_count, .Never);
			}
			else
			{
				status = physics.solver_execute_type_bundle_dispatch_integration(solver, batch, record, 0, bundle,
					1.0 / 64, 64, .First, stage, 0, .Native, .Never);
			}
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
	}
	else
	{
		for index in 0 ..< int(batch.count)
		{
			if route == .Direct_Lane
			{
				status = physics.solver_execute_direct_noncontact_lane_integration(solver, batch, 0, index,
					1.0 / 64, 64, .First, stage, 0, type_id, body_count, .Never);
			}
			else
			{
				status = physics.solver_execute_type_lane_dispatch_integration(solver, batch, record, 0, index,
					1.0 / 64, 64, .First, stage, 0, .Native, .Never);
			}
			testing.expect_value(t, status, physics.Physics_Status.Ok);
		}
	}
}

compare_reactions :: proc(t: ^testing.T, near_scene, far_scene: ^Scene, near_fixture, far_fixture: ^Joint_Fixture, count: int)
{
	for group in 0 ..< count
	{
		a, b: physics.Constraint_Reaction_Sample;
		testing.expect_value(t, physics.constraint_capture_reaction(&near_scene.simulation.solver, near_fixture.joints[group], 1.0 / 64, {}, &a), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.constraint_capture_reaction(&far_scene.simulation.solver, far_fixture.joints[group], 1.0 / 64, {}, &b), physics.Physics_Status.Ok);
		testing.expect_value(t, a, b);
	}
	for first: int = 0; first < count; first += 8
	{
		lane_count: int = min(8, count - first);
		mask: u8 = u8((u32(1) << u32(lane_count)) - 1) & 0xad;
		a, b: [8]physics.Constraint_Reaction_Sample;
		testing.expect_value(t, physics.constraint_capture_reaction_bundle(&near_scene.simulation.solver, near_fixture.joints[first], mask, 1.0 / 64, {}, &a), physics.Physics_Status.Ok);
		testing.expect_value(t, physics.constraint_capture_reaction_bundle(&far_scene.simulation.solver, far_fixture.joints[first], mask, 1.0 / 64, {}, &b), physics.Physics_Status.Ok);
		testing.expect_value(t, a, b);
		for lane in 0 ..< lane_count
		{
			if mask & (u8(1) << u32(lane)) != 0
			{
				scalar: physics.Constraint_Reaction_Sample;
				testing.expect_value(t, physics.constraint_capture_reaction(&far_scene.simulation.solver, far_fixture.joints[first + lane], 1.0 / 64, {}, &scalar), physics.Physics_Status.Ok);
				testing.expect_value(t, b[lane], scalar);
			}
		}
	}
}

exercise_joint_type :: proc(t: ^testing.T, description: ^$T, $type_id: i32, $body_count: int, count: int = 17)
{
	for route in Execution_Route
	{
		when type_id != physics.BALL_SOCKET_TYPE_ID
		{
			if route == .Cached_Ball_Socket
			{
				continue;
			}
		}
		near_scene, far_scene: Scene;
		scene_initialize(t, &near_scene);
		scene_initialize(t, &far_scene);
		near_fixture, far_fixture: Joint_Fixture;
		populate_joints(t, &near_scene, &near_fixture, {}, description, body_count, count);
		populate_joints(t, &far_scene, &far_fixture, {1e9, -1e9, 1e9}, description, body_count, count);
		near_batch: ^physics.Type_Batch = constraint_batch(t, &near_scene, near_fixture.joints[0]);
		far_batch: ^physics.Type_Batch = constraint_batch(t, &far_scene, far_fixture.joints[0]);
		// nonzero retained impulses exercise warmstart as well as the solve path
		for group in 0 ..< count
		{
			for row in 0 ..< int(near_batch.impulse_bundle_size) / size_of(util.F32x8)
			{
				value: f32 = f32(row + 1) * 0.03125;
				a: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(near_batch, group));
				b: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(far_batch, group));
				a[row] = simd.replace(a[row], group % 8, value);
				b[row] = simd.replace(b[row], group % 8, value);
			}
		}
		near_calls: i64 = near_scene.tracker.total_allocation_count;
		far_calls: i64 = far_scene.tracker.total_allocation_count;
		for iteration in 0 ..< 4
		{
			stage: physics.Solver_Execution_Stage = .Solve;
			if iteration == 0
			{
				stage = .Warmstart;
			}
			execute_stage(t, &near_scene, near_fixture.joints[0], route, stage, type_id, body_count);
			execute_stage(t, &far_scene, far_fixture.joints[0], route, stage, type_id, body_count);
			for group in 0 ..< count
			{
				for body in 0 ..< body_count
				{
					a: physics.Body_Velocity = body_motion(t, &near_scene, near_fixture.bodies[group][body]).velocity;
					b: physics.Body_Velocity = body_motion(t, &far_scene, far_fixture.bodies[group][body]).velocity;
					testing.expectf(t, a == b, "type %v route %v stage %v group %v body %v: near %v far %v", type_id, route, stage, group, body, a, b);
				}
				for row in 0 ..< int(near_batch.impulse_bundle_size) / size_of(util.F32x8)
				{
					a: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(near_batch, group));
					b: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_impulse_bundle(far_batch, group));
					testing.expect_value(t, simd.extract(a[row], group % 8), simd.extract(b[row], group % 8));
				}
			}
			compare_reactions(t, &near_scene, &far_scene, &near_fixture, &far_fixture, count);
		}
		testing.expect_value(t, near_scene.tracker.total_allocation_count, near_calls);
		testing.expect_value(t, far_scene.tracker.total_allocation_count, far_calls);
		scene_dispose(t, &far_scene);
		scene_dispose(t, &near_scene);
	}
}

@(test)
precision_multibody_all_builtin_routes_and_reactions :: proc(t: ^testing.T)
{
	spring: physics.Spring_Settings = {angular_frequency=12.5, twice_damping_ratio=1.5};
	servo: physics.Servo_Settings = {maximum_speed=25, base_speed=0.5, maximum_force=1000};
	motor: physics.Motor_Settings = {maximum_force=750, damping=20};
	q: util.Quaternion = util.quaternion_identity();
	x: util.Vector3 = {1, 0, 0};
	y: util.Vector3 = {0, 1, 0};
	ball_socket: physics.Ball_Socket = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring};
	exercise_joint_type(t, &ball_socket, physics.BALL_SOCKET_TYPE_ID, 2);
	angular_hinge: physics.Angular_Hinge = {x, y, spring};
	exercise_joint_type(t, &angular_hinge, physics.ANGULAR_HINGE_TYPE_ID, 2);
	angular_swivel_hinge: physics.Angular_Swivel_Hinge = {x, y, spring};
	exercise_joint_type(t, &angular_swivel_hinge, physics.ANGULAR_SWIVEL_HINGE_TYPE_ID, 2);
	swing_limit: physics.Swing_Limit = {x, y, -0.5, spring};
	exercise_joint_type(t, &swing_limit, physics.SWING_LIMIT_TYPE_ID, 2);
	twist_servo: physics.Twist_Servo = {q, q, 0.5, spring, servo};
	exercise_joint_type(t, &twist_servo, physics.TWIST_SERVO_TYPE_ID, 2);
	twist_limit: physics.Twist_Limit = {q, q, -1, 1, spring};
	exercise_joint_type(t, &twist_limit, physics.TWIST_LIMIT_TYPE_ID, 2);
	twist_motor: physics.Twist_Motor = {x, y, 2, motor};
	exercise_joint_type(t, &twist_motor, physics.TWIST_MOTOR_TYPE_ID, 2);
	angular_servo: physics.Angular_Servo = {q, spring, servo};
	exercise_joint_type(t, &angular_servo, physics.ANGULAR_SERVO_TYPE_ID, 2);
	angular_motor: physics.Angular_Motor = {{1, 2, 3}, motor};
	exercise_joint_type(t, &angular_motor, physics.ANGULAR_MOTOR_TYPE_ID, 2);
	weld: physics.Weld = {{0.1, 0.2, 0.3}, q, spring};
	exercise_joint_type(t, &weld, physics.WELD_TYPE_ID, 2);
	distance_servo: physics.Distance_Servo = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 2, servo, spring};
	exercise_joint_type(t, &distance_servo, physics.DISTANCE_SERVO_TYPE_ID, 2);
	distance_limit: physics.Distance_Limit = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, 0.5, 2, spring};
	exercise_joint_type(t, &distance_limit, physics.DISTANCE_LIMIT_TYPE_ID, 2);
	center_distance_constraint: physics.Center_Distance_Constraint = {2, spring};
	exercise_joint_type(t, &center_distance_constraint, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID, 2);
	point_on_line_servo: physics.Point_On_Line_Servo = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, x, servo, spring};
	exercise_joint_type(t, &point_on_line_servo, physics.POINT_ON_LINE_SERVO_TYPE_ID, 2);
	linear_axis_servo: physics.Linear_Axis_Servo = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, x, 0.75, servo, spring};
	exercise_joint_type(t, &linear_axis_servo, physics.LINEAR_AXIS_SERVO_TYPE_ID, 2);
	linear_axis_motor: physics.Linear_Axis_Motor = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, x, 2, motor};
	exercise_joint_type(t, &linear_axis_motor, physics.LINEAR_AXIS_MOTOR_TYPE_ID, 2);
	linear_axis_limit: physics.Linear_Axis_Limit = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, x, -0.5, 2, spring};
	exercise_joint_type(t, &linear_axis_limit, physics.LINEAR_AXIS_LIMIT_TYPE_ID, 2);
	angular_axis_motor: physics.Angular_Axis_Motor = {x, 2, motor};
	exercise_joint_type(t, &angular_axis_motor, physics.ANGULAR_AXIS_MOTOR_TYPE_ID, 2);
	swivel_hinge: physics.Swivel_Hinge = {{0.1, 0.2, 0.3}, x, {0.4, 0.5, 0.6}, y, spring};
	exercise_joint_type(t, &swivel_hinge, physics.SWIVEL_HINGE_TYPE_ID, 2);
	hinge: physics.Hinge = {{0.1, 0.2, 0.3}, x, {0.4, 0.5, 0.6}, y, spring};
	exercise_joint_type(t, &hinge, physics.HINGE_TYPE_ID, 2);
	ball_socket_motor: physics.Ball_Socket_Motor = {{0.4, 0.5, 0.6}, {1, 2, 3}, motor};
	exercise_joint_type(t, &ball_socket_motor, physics.BALL_SOCKET_MOTOR_TYPE_ID, 2);
	ball_socket_servo: physics.Ball_Socket_Servo = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, spring, servo};
	exercise_joint_type(t, &ball_socket_servo, physics.BALL_SOCKET_SERVO_TYPE_ID, 2);
	angular_axis_gear_motor: physics.Angular_Axis_Gear_Motor = {x, 2, motor};
	exercise_joint_type(t, &angular_axis_gear_motor, physics.ANGULAR_AXIS_GEAR_MOTOR_TYPE_ID, 2);
	center_distance_limit: physics.Center_Distance_Limit = {0.5, 2, spring};
	exercise_joint_type(t, &center_distance_limit, physics.CENTER_DISTANCE_LIMIT_TYPE_ID, 2);
	area_constraint: physics.Area_Constraint = {2, spring};
	exercise_joint_type(t, &area_constraint, physics.AREA_CONSTRAINT_TYPE_ID, 3);
	volume_constraint: physics.Volume_Constraint = {2, spring};
	exercise_joint_type(t, &volume_constraint, physics.VOLUME_CONSTRAINT_TYPE_ID, 4);
}

@(test)
precision_constraint_bundle_tails :: proc(t: ^testing.T)
{
	description: physics.Center_Distance_Constraint = {1, {angular_frequency=12.5, twice_damping_ratio=1.5}};
	for count in 1 ..= 17
	{
		exercise_joint_type(t, &description, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID, 2, count);
	}
}
