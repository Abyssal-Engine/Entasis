package precision_constraints_tests

import "core:math"
import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_constraint_gather_all_lane_and_position_masks :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	positions: [4][8]physics.Global_Position;
	references: [4]util.I32x8;
	for lane in 0 ..< 8
	{
		for body in 0 ..< 4
		{
			position: physics.Global_Position = {1e9 + f64(lane * 1024) + f64(body) * 0.002,
				-1e9 + f64(lane) * 0.125 - f64(body) * 0.003, 1e6 + f64(lane) * 0.25 + f64(body) * 0.005};
			positions[body][lane] = position;
			mobility: physics.Body_Mobility = .Dynamic;
			if body == 3
			{
				mobility = .Kinematic;
			}
			handle: physics.Body_Handle = add_body(t, &scene, position, mobility);
			index: i32 = scene.simulation.bodies.handle_to_location.memory[handle.value].index;
			if mobility == .Kinematic
			{
				index = i32(u32(index) | physics.BODY_REFERENCE_KINEMATIC_MASK);
			}
			references[body] = simd.replace(references[body], lane, index);
		}
	}
	calls: i64 = scene.tracker.total_allocation_count;
	bytes: i64 = scene.tracker.current_memory_allocated;
	for body_count in 2 ..= 4
	{
		for bits in 0 ..< 256
		{
			mask: util.I32x8;
			encoded: [4]util.I32x8 = references;
			for lane in 0 ..< 8
			{
				if bits & (1 << u32(lane)) != 0
				{
					mask = simd.replace(mask, lane, -1);
				}
				else
				{
					for body in 0 ..< 4
					{
						encoded[body] = simd.replace(encoded[body], lane, -1);
					}
				}
			}
			for position_bits in 1 ..< (1 << u32(body_count))
			{
				bodies: [4]physics.Constraint_Kernel_Body_Wide;
				for body in 0 ..< 4
				{
					bodies[body].position = util.vector3_wide_broadcast({17, 19, 23});
					bodies[body].linear_velocity = util.vector3_wide_broadcast({29, 31, 37});
				}
				testing.expect_value(t, physics.solver_gather_constraint_relative_positions(
					&scene.simulation.bodies, &encoded[0], body_count, u8(position_bits), mask, &bodies[0],
				), physics.Physics_Status.Ok);
				for body in 0 ..< 4
				{
					for lane in 0 ..< 8
					{
						expected: util.Vector3 = {17, 19, 23};
						if body == 0
						{
							expected = {};
						}
						else if body < body_count && position_bits & (1 << u32(body)) != 0
						{
							expected = {};
							if bits & (1 << u32(lane)) != 0
							{
								a: physics.Global_Position = positions[0][lane];
								b: physics.Global_Position = positions[body][lane];
								expected = {f32(b.x - a.x), f32(b.y - a.y), f32(b.z - a.z)};
							}
						}
						testing.expect_value(t, util.vector3_wide_read_slot(bodies[body].position, lane), expected);
						testing.expect_value(t, util.vector3_wide_read_slot(bodies[body].linear_velocity, lane), util.Vector3{29, 31, 37});
					}
				}
			}
		}
	}
	testing.expect_value(t, scene.tracker.total_allocation_count, calls);
	testing.expect_value(t, scene.tracker.current_memory_allocated, bytes);
}

@(test)
precision_constraint_gather_rejects_only_requested_unrepresentable_displacements :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	limit: f64 = f64(f32(math.F32_MAX));
	a: physics.Body_Handle = add_body(t, &scene, {-limit, 0, 0});
	b: physics.Body_Handle = add_body(t, &scene, {limit, 0, 0});
	c: physics.Body_Handle = add_body(t, &scene, {-limit, 0.002, 0});
	references: [3]util.I32x8;
	references[0] = util.I32x8(scene.simulation.bodies.handle_to_location.memory[a.value].index);
	references[1] = util.I32x8(scene.simulation.bodies.handle_to_location.memory[b.value].index);
	references[2] = util.I32x8(scene.simulation.bodies.handle_to_location.memory[c.value].index);
	bodies: [4]physics.Constraint_Kernel_Body_Wide;
	testing.expect_value(t, physics.solver_gather_constraint_relative_positions(
		&scene.simulation.bodies, &references[0], 3, 7, util.I32x8(-1), &bodies[0],
	), physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, physics.solver_gather_constraint_relative_positions(
		&scene.simulation.bodies, &references[0], 3, 5, util.I32x8(-1), &bodies[0],
	), physics.Physics_Status.Ok);
	testing.expect_value(t, util.vector3_wide_read_slot(bodies[2].position, 0), util.Vector3{0, 0.002, 0});
	// empty access and lane masks require no body storage reads
	testing.expect_value(t, physics.solver_gather_constraint_relative_positions(nil, nil, 3, 0, {}, &bodies[0]), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.solver_gather_constraint_relative_positions(&scene.simulation.bodies, nil, 3, 7, {}, &bodies[0]), physics.Physics_Status.Ok);
	testing.expect_value(t, body_position(t, &scene, a), physics.Global_Position{-limit, 0, 0});
	testing.expect_value(t, body_position(t, &scene, b), physics.Global_Position{limit, 0, 0});
}
