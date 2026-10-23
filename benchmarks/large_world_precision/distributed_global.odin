package large_world_precision

import "core:math"
import entasis "entasis:entasis"
import support "../benchmark_support"

// sixteen interacting groups share one production world and solver
// shape records use the existing world-owned registry
distributed_global_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	floor: entasis.Shape_Handle;
	shape: entasis.Shape_Handle;
	status: entasis.Status;
	floor, status = entasis.shape_add(&owner.world, entasis.box(48, 1, 24));
	if status != .Ok
	{
		return status;
	}
	shape, status = entasis.shape_add(&owner.world, entasis.box(1, 1, 1));
	if status != .Ok
	{
		return status;
	}
	inertia: entasis.Body_Inertia;
	inertia, status = entasis.shape_registered_inertia(&owner.world, shape, 1);
	if status != .Ok
	{
		return status;
	}
	description: entasis.Body_Description = entasis.body_dynamic(shape, inertia, entasis.pose(), {}, entasis.body_activity(-1, 32));
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
	for group in 0 ..< DISTRIBUTED_WORLD_COUNT
	{
		origin: entasis.Global_Position = {DISTRIBUTED_OFFSET + f64(group) * DISTRIBUTED_SEPARATION, 0, 0};
		_, status = candidate_static_add(owner, floor, {origin.x, -0.5, 0}, .None);
		if status != .Ok
		{
			return status;
		}
		for local_index in 0 ..< DISTRIBUTED_BODY_COUNT
		{
			index: int = group * DISTRIBUTED_BODY_COUNT + local_index;
			position: entasis.Global_Position = distributed_fixture_position({origin=origin}, local_index);
			owner.expected_global_positions[index] = position;
			owner.bodies[index], status = candidate_body_add(owner, description, position);
			if status != .Ok
			{
				return status;
			}
			// fresh deletion-free world: this direct identity map is established
			// once, before query validation indexes the caller's fixed arrays
			if owner.bodies[index].value != i32(index)
			{
				return .Invalid_Description;
			}
		}
	}
	return .Ok;
}

distributed_global_validate :: proc(owner: ^Benchmark_Owner, sample: ^support.Benchmark_Sample) -> support.Benchmark_Result
{
	if sample.stats.active_pairs == 0 || sample.stats.active_constraints == 0 ||
		sample.stats.active_bodies != DISTRIBUTED_BODY_COUNT * DISTRIBUTED_WORLD_COUNT || sample.stats.sleeping_bodies != 0
	{
		return {status=.Validation_Failed};
	}
	for global, index in owner.exported_global_positions
	{
		group: int = index / DISTRIBUTED_BODY_COUNT;
		x: f64 = global.x - (DISTRIBUTED_OFFSET + f64(group) * DISTRIBUTED_SEPARATION);
		body: entasis.Body_Description;
		status: entasis.Status;
		body, status = entasis.body_get(&owner.world, owner.bodies[index]);
		if status != .Ok
		{
			return {status=.Validation_Failed, physics_status=status};
		}
		values: [13]f64 = {x, global.y, global.z,
			f64(body.pose.orientation.x), f64(body.pose.orientation.y), f64(body.pose.orientation.z), f64(body.pose.orientation.w),
			f64(body.velocity.linear.x), f64(body.velocity.linear.y), f64(body.velocity.linear.z),
			f64(body.velocity.angular.x), f64(body.velocity.angular.y), f64(body.velocity.angular.z)};
		for value in values
		{
			if math.is_nan(value) || math.is_inf(value, 0)
			{
				sample.invalid_transform_count += 1;
			}
		}
		if global.y < 0
		{
			sample.below_floor_count += 1;
		}
		if math.abs(x) > 24.5 || math.abs(global.z) > 12.5 || global.y > 16
		{
			sample.out_of_bounds_count += 1;
		}
	}
	if sample.invalid_transform_count != 0 || sample.below_floor_count != 0 || sample.out_of_bounds_count != 0
	{
		return {status=.Validation_Failed};
	}
	return {status=.Ok};
}
