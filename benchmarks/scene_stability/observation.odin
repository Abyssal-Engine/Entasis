package scene_stability

import "core:fmt"
import "core:math"
import "core:os"
import entasis "entasis:entasis"
import observations "../scene_observation"
import support "../benchmark_support"

Observation :: observations.Observation

write_header :: proc(file: ^os.File) -> support.Case_Status
{
	header: string = "step,simulated_seconds,group,expected_bodies,world_bodies,observed_bodies,invalid,escaped,below_floor,mean_height,height_fraction,lateral_rms,plane_rms,maximum_drop,dropped,displaced,outside_plane,maximum_center_height,apex,maximum_floor_penetration,maximum_speed\n"
	written: int
	error: os.Error
	written, error = os.write(file, transmute([]byte)header)
	if error != nil || written != len(header)
	{
		return .Output_Failed
	}
	return .Ok
}

write_observation :: proc(file: ^os.File, observation: Observation, step, hz, group, expected, world_bodies: int) -> support.Case_Status
{
	count: f64 = f64(max(observation.count, 1))
	height_fraction: f64 = observation.sum_y / observation.initial_sum_y
	buffer: [8192]byte
	row: string = fmt.bprintf(buffer[:], "%d,%.9f,%d,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%d,%.9f,%.9f,%.9f,%.9f\n",
		step, f64(step)/f64(hz), group, expected, world_bodies, observation.count,
		observation.invalid, observation.escaped, observation.below_floor,
		observation.sum_y/count, height_fraction,
		math.sqrt(observation.lateral_squared/count), math.sqrt(observation.plane_squared/count), observation.maximum_drop,
		observation.dropped, observation.displaced, observation.outside_plane, observation.maximum_center_height,
		observation.apex, observation.maximum_floor_penetration, observation.maximum_speed,
	)
	written: int
	error: os.Error
	written, error = os.write(file, transmute([]byte)row)
	if error != nil || written != len(row)
	{
		return .Output_Failed
	}
	return .Ok
}

write_snapshot :: proc(file: ^os.File, world: ^entasis.World, bodies: []entasis.Body_Handle) -> support.Case_Status
{
	header: string = "body,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n"
	header_written: int
	header_error: os.Error
	header_written, header_error = os.write(file, transmute([]byte)header)
	if header_error != nil || header_written != len(header)
	{
		return .Output_Failed
	}
	for body, index in bodies
	{
		state: entasis.Body_State
		status: entasis.Status
		state, status = entasis.body_get(world, body)
		if status != .Ok
		{
			return .Validation_Failed
		}
		buffer: [8192]byte
		row: string = fmt.bprintf(buffer[:], "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
			index, state.pose.position.x, state.pose.position.y, state.pose.position.z,
			state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
			state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
			state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z,
		)
		written: int
		error: os.Error
		written, error = os.write(file, transmute([]byte)row)
		if error != nil || written != len(row)
		{
			return .Output_Failed
		}
	}
	return .Ok
}
