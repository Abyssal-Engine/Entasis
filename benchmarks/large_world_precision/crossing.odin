package large_world_precision

import "core:math"
import entasis "entasis:entasis"
import support "../benchmark_support"

CROSSING_PAIR_COUNT :: CROSSING_BODY_COUNT / 2;
CROSSING_COLUMN_COUNT :: 50;
CROSSING_LAYER_COUNT :: CROSSING_PAIR_COUNT / CROSSING_COLUMN_COUNT;
CROSSING_CENTER_SEPARATION :: f32(1.2);
CROSSING_LINEAR_SPEED :: f32(0.75);
CROSSING_LAYER_SPACING :: f32(1.5);
CROSSING_COLUMN_SPACING :: f32(1.5);

#assert(CROSSING_PAIR_COUNT * 2 == CROSSING_BODY_COUNT);
#assert(CROSSING_LAYER_COUNT * CROSSING_COLUMN_COUNT == CROSSING_PAIR_COUNT);
#assert(CROSSING_JOINT_COUNT <= CROSSING_PAIR_COUNT);

crossing_joint_description :: #force_inline proc "contextless" () -> entasis.Center_Distance_Constraint
{
	return {
		target_distance=0.9,
		spring_settings=entasis.spring_settings(30, 1),
	};
}

crossing_body_description :: #force_inline proc "contextless" (
	shape: entasis.Shape_Handle,
	inertia: entasis.Body_Inertia,
	position, linear_velocity: entasis.Vector3,
) -> entasis.Body_Description
{
	description := entasis.body_dynamic(
		shape,
		inertia,
		entasis.pose(position),
		entasis.velocity(linear_velocity),
		entasis.body_activity(-1, 32),
	);
	description.collidable.maximum_speculative_margin = f32(math.F32_MAX);
	return description;
}

crossing_add_body :: proc(
	owner: ^Benchmark_Owner,
	shape: entasis.Shape_Handle,
	inertia: entasis.Body_Inertia,
	body_index: int,
	logical_position, velocity: entasis.Vector3,
) -> entasis.Status
{
	global := global_from_local_fixture(owner, logical_position);
	owner.expected_global_positions[body_index] = global;
	description := crossing_body_description(shape, inertia, {}, velocity);
	status: entasis.Status;
	owner.bodies[body_index], status = candidate_body_add(owner, description, global);
	return status;
}

crossing_build_fixture :: proc(owner: ^Benchmark_Owner) -> entasis.Status
{
	floor_shape, floor_status := support.register_shape(
		&owner.world,
		&owner.shapes,
		.Box,
		{32, 1, 80},
	);
	if floor_status != .Ok
	{
		return floor_status;
	}
	status := add_static_fixture(owner, floor_shape, {0, -16, 0});
	if status != .Ok
	{
		return status;
	}
	shape, inertia, shape_status := support.register_dynamic(
		&owner.world,
		&owner.shapes,
		.Box,
		{1, 1, 1},
		1,
	);
	if shape_status != .Ok
	{
		return shape_status;
	}
	half_separation := CROSSING_CENTER_SEPARATION * 0.5;
	for pair_index in 0 ..< CROSSING_PAIR_COUNT
	{
		column := pair_index % CROSSING_COLUMN_COUNT;
		layer := pair_index / CROSSING_COLUMN_COUNT;
		y := (f32(layer) - 0.5*f32(CROSSING_LAYER_COUNT-1))*CROSSING_LAYER_SPACING;
		z := (f32(column) - 0.5*f32(CROSSING_COLUMN_COUNT-1))*CROSSING_COLUMN_SPACING;
		a_index := pair_index*2;
		b_index := a_index+1;
		status = crossing_add_body(
			owner,
			shape,
			inertia,
			a_index,
			{-half_separation, y, z},
			{CROSSING_LINEAR_SPEED, 0, 0},
		);
		if status != .Ok
		{
			return status;
		}
		status = crossing_add_body(
			owner,
			shape,
			inertia,
			b_index,
			{half_separation, y, z},
			{-CROSSING_LINEAR_SPEED, 0, 0},
		);
		if status != .Ok
		{
			return status;
		}
		if pair_index < CROSSING_JOINT_COUNT
		{
			owner.constraints[pair_index], status = entasis.constraint_add_2(
				&owner.world,
				owner.bodies[a_index],
				owner.bodies[b_index],
				crossing_joint_description(),
			);
			if status != .Ok
			{
				return status;
			}
		}
	}
	return .Ok;
}

crossing_validate :: proc(
	owner: ^Benchmark_Owner,
	sample: ^support.Benchmark_Sample,
) -> support.Benchmark_Result
{
	for global in owner.exported_global_positions
	{
		x := global.x-owner.frame.origin.x;
		y := global.y-owner.frame.origin.y;
		z := global.z-owner.frame.origin.z;
		if math.is_nan(x) || math.is_inf(x, 0) ||
			math.is_nan(y) || math.is_inf(y, 0) ||
			math.is_nan(z) || math.is_inf(z, 0)
		{
			sample.invalid_transform_count += 1;
			continue;
		}
		if math.abs(x) > 32 || math.abs(y) > 32 || math.abs(z) > 80
		{
			sample.out_of_bounds_count += 1;
		}
	}
	if sample.invalid_transform_count != 0 || sample.out_of_bounds_count != 0
	{
		return {status=.Validation_Failed};
	}
	for pair_index in 0 ..< CROSSING_PAIR_COUNT
	{
		a := owner.exported_global_positions[pair_index*2];
		b := owner.exported_global_positions[pair_index*2+1];
		if a.x > owner.frame.origin.x || b.x < owner.frame.origin.x || a.x >= b.x
		{
			return {status=.Validation_Failed};
		}
	}
	expected_joint := crossing_joint_description();
	for handle in owner.constraints
	{
		actual_joint: entasis.Center_Distance_Constraint;
		status := entasis.constraint_get(&owner.world, handle, &actual_joint);
		if status != .Ok || actual_joint != expected_joint
		{
			return {status=.Validation_Failed, physics_status=status};
		}
	}
	return {status=.Ok};
}
