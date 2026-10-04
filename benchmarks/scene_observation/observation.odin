package scene_observation

import "core:math"
import entasis "entasis:entasis"

State_Status :: enum u8
{
	Valid,
	Invalid,
}

Observation :: struct
{
	count: int,
	invalid: int,
	escaped: int,
	below_floor: int,
	dropped: int,
	displaced: int,
	outside_plane: int,
	sum_y: f64,
	initial_sum_y: f64,
	lateral_squared: f64,
	plane_squared: f64,
	maximum_drop: f64,
	maximum_speed: f64,
	maximum_center_height: f64,
	apex: f64,
	maximum_floor_penetration: f64,
}

state_status :: proc(state: entasis.Body_State) -> State_Status
{
	values: [13]f32 = {
		state.pose.position.x, state.pose.position.y, state.pose.position.z,
		state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w,
		state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
		state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z,
	}
	for value in values
	{
		if transmute(u32)value & 0x7f80_0000 == 0x7f80_0000
		{
			return .Invalid
		}
	}
	return .Valid
}

rotated_half_height :: proc(state: entasis.Body_State, size: [3]f32) -> f64
{
	q: entasis.Quaternion = state.pose.orientation
	return 0.5 * f64(
		size[0] * math.abs(2 * (q.x*q.y + q.w*q.z)) +
		size[1] * math.abs(1 - 2 * (q.x*q.x + q.z*q.z)) +
		size[2] * math.abs(2 * (q.y*q.z - q.w*q.x)),
	)
}

observe_body :: proc(
	observation: ^Observation, state: entasis.Body_State, initial: entasis.Vector3,
	size: [3]f32, floor_center: entasis.Vector3, floor_half_size: [2]f32,
)
{
	observation.count += 1
	observation.initial_sum_y += f64(initial.y)
	if state_status(state) == .Invalid
	{
		observation.invalid += 1
		return
	}
	position: entasis.Vector3 = state.pose.position
	dx: f64 = f64(position.x - initial.x)
	dz: f64 = f64(position.z - initial.z)
	drop: f64 = f64(initial.y - position.y)
	half_height: f64 = rotated_half_height(state, size)
	observation.sum_y += f64(position.y)
	observation.lateral_squared += dx*dx + dz*dz
	observation.plane_squared += dz*dz
	observation.maximum_drop = max(observation.maximum_drop, drop)
	observation.maximum_center_height = max(observation.maximum_center_height, f64(position.y))
	observation.apex = max(observation.apex, f64(position.y) + half_height)
	velocity: entasis.Vector3 = state.velocity.linear
	observation.maximum_speed = max(observation.maximum_speed, math.sqrt(f64(velocity.x)*f64(velocity.x) + f64(velocity.y)*f64(velocity.y) + f64(velocity.z)*f64(velocity.z)))
	if drop > f64(size[1])
	{
		observation.dropped += 1
	}
	if dx*dx + dz*dz > f64(size[0])*f64(size[0])
	{
		observation.displaced += 1
	}
	if math.abs(dz) > f64(size[2])
	{
		observation.outside_plane += 1
	}
	if position.y < floor_center.y
	{
		observation.below_floor += 1
	}
	if math.abs(position.x - floor_center.x) > floor_half_size[0] ||
		math.abs(position.z - floor_center.z) > floor_half_size[1]
	{
		observation.escaped += 1
	}
	else
	{
		observation.maximum_floor_penetration = max(observation.maximum_floor_penetration, half_height - f64(position.y - floor_center.y))
	}
}
