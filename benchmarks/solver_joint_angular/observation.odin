package solver_joint_angular

import "core:fmt"
import "core:math"
import "core:os"
import entasis "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

Observation :: struct
{
	invalid, late_count, late_bodies, samples: int,
	max_anchor, max_angle, max_speed, late_anchor, late_angle, late_speed: f64,
	hash: u64,
}

observe :: proc(owner: ^Benchmark_Owner, step: int, observation: ^Observation) -> entasis.Status
{
	for body in owner.bodies
	{
		state: entasis.Body_State;
		status: entasis.Status;
		state, status = entasis.body_get(&owner.world, body);
		if status != .Ok
		{
			return status;
		}
		values: [13]f32 = {state.pose.position.x, state.pose.position.y, state.pose.position.z,
			state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
			state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
			state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z};
		for value in values
		{
			if math.is_nan(value) || math.is_inf(value)
			{
				observation.invalid += 1;
			}
			if step%30 == 0 || step == owner.options.steps
			{
				observation.hash = (observation.hash ~ u64(transmute(u32)value))*1099511628211;
			}
		}
		speed2: f64 = f64(util.vector3_length_squared(state.velocity.linear));
		observation.max_speed = max(observation.max_speed, math.sqrt(speed2));
		if step > owner.options.steps*3/4
		{
			observation.late_speed += speed2;
			observation.late_bodies += 1;
		}
	}
	if step%30 != 0 && step != owner.options.steps
	{
		return .Ok;
	}
	observation.samples += 1;
	for edge in owner.edges
	{
		a, b: entasis.Body_State;
		status: entasis.Status;
		a, status = entasis.body_get(&owner.world, edge.a);
		if status != .Ok
		{
			return status;
		}
		b, status = entasis.body_get(&owner.world, edge.b);
		if status != .Ok
		{
			return status;
		}
		pa: entasis.Vector3 = util.vector3_add(a.pose.position, util.quaternion_transform({0, -1 if edge.kind == .Weld else -0.5, 0}, a.pose.orientation));
		pb: entasis.Vector3 = b.pose.position;
		angle: f64;
		if edge.kind == .Weld
		{
			qa, qb: entasis.Quaternion = a.pose.orientation, b.pose.orientation;
			dot: f64 = math.abs(f64(qa.x)*f64(qb.x)+f64(qa.y)*f64(qb.y)+f64(qa.z)*f64(qb.z)+f64(qa.w)*f64(qb.w));
			na: f64 = math.sqrt(f64(qa.x)*f64(qa.x)+f64(qa.y)*f64(qa.y)+f64(qa.z)*f64(qa.z)+f64(qa.w)*f64(qa.w));
			nb: f64 = math.sqrt(f64(qb.x)*f64(qb.x)+f64(qb.y)*f64(qb.y)+f64(qb.z)*f64(qb.z)+f64(qb.w)*f64(qb.w));
			angle = 2*math.acos(clamp(dot/max(1e-30, na*nb), f64(0), f64(1)));
		}
		else
		{
			pb = util.vector3_add(pb, util.quaternion_transform({0, 0.5, 0}, b.pose.orientation));
			axis_a: entasis.Vector3 = util.quaternion_transform({0, 0, 1}, a.pose.orientation);
			axis_b: entasis.Vector3 = util.quaternion_transform({0, 0, 1}, b.pose.orientation);
			angle = math.atan2(f64(util.vector3_length(util.vector3_cross(axis_a, axis_b))), f64(util.vector3_dot(axis_a, axis_b)));
		}
		anchor: f64 = f64(util.vector3_length(util.vector3_subtract(pb, pa)));
		observation.max_anchor = max(observation.max_anchor, anchor);
		observation.max_angle = max(observation.max_angle, angle);
		if step > owner.options.steps*3/4
		{
			observation.late_anchor += anchor*anchor;
			observation.late_angle += angle*angle;
			observation.late_count += 1;
		}
	}
	return .Ok;
}

OBSERVATION_HEADER :: "invalid,max_anchor_error,max_angle_error,late_rms_anchor_error,late_rms_angle_error,late_rms_speed,max_speed,sleeping_bodies,samples,hash";

observation_csv :: proc(o: Observation, stats: entasis.World_Stats, buffer: []u8) -> string
{
	return fmt.bprintf(buffer, "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%016x", o.invalid,
		o.max_anchor, o.max_angle, math.sqrt(o.late_anchor/f64(max(1, o.late_count))),
		math.sqrt(o.late_angle/f64(max(1, o.late_count))), math.sqrt(o.late_speed/f64(max(1, o.late_bodies))),
		o.max_speed, stats.sleeping_bodies, o.samples, o.hash);
}

write_snapshot :: proc(owner: ^Benchmark_Owner, path: string) -> support.Case_Status
{
	file: ^os.File;
	error: os.Error;
	file, error = os.open(path, {.Write, .Create, .Excl});
	if error != nil
	{
		return .Output_Failed;
	}
	defer os.close(file);
	fmt.fprintln(file, "body,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz");
	for body, index in owner.bodies
	{
		state: entasis.Body_State;
		status: entasis.Status;
		state, status = entasis.body_get(&owner.world, body);
		if status != .Ok
		{
			return .Validation_Failed;
		}
		buffer: [1024]u8;
		line: string = fmt.bprintf(buffer[:], "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", index,
			state.pose.position.x, state.pose.position.y, state.pose.position.z,
			state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
			state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
			state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z);
		written: int;
		written, error = os.write(file, transmute([]u8)line);
		if error != nil || written != len(line)
		{
			return .Output_Failed;
		}
	}
	return .Ok;
}
