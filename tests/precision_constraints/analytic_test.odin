package precision_constraints_tests

import "core:math"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

#assert(size_of(physics.Constraint_Kernel_Body_Wide) == 640);

@(test)
precision_distance_two_millimeter_error_matches_scalar_oracle :: proc(t: ^testing.T)
{
	bases: [4]f64 = {0, 1e4, 1e6, 1e9};
	for base in bases
	{
		for axis in 0 ..< 3
		{
			for sign in -1 ..= 1
			{
				if sign == 0
				{
					continue;
				}
				for partner in 0 ..< 2
				{
					scene: Scene;
					scene_initialize(t, &scene);
					a: physics.Global_Position = {base + 0.03125, -base + 0.125, base - 0.0625};
					b: physics.Global_Position = a;
					displacement: f64 = f64(sign) * 1.002;
					switch axis
					{
						case 0: b.x += displacement;
						case 1: b.y += displacement;
						case 2: b.z += displacement;
					}
					mobility: physics.Body_Mobility = .Dynamic;
					if partner == 1
					{
						mobility = .Kinematic;
					}
					handles: [4]physics.Body_Handle = {add_body(t, &scene, a), add_body(t, &scene, b, mobility), {}, {}};
					description: physics.Center_Distance_Constraint = {1, {angular_frequency=12.5, twice_damping_ratio=1.5}};
					joint: physics.Constraint_Handle;
					status: physics.Physics_Status;
					joint, status = physics.solver_add(&scene.simulation.solver, &handles, &description);
					testing.expect_value(t, status, physics.Physics_Status.Ok);
					execute_stage(t, &scene, joint, .Direct_Bundle, .Warmstart, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID, 2);
					execute_stage(t, &scene, joint, .Direct_Bundle, .Solve, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID, 2);
					// independent double scalar spring equation, using the admitted coordinates
					dx, dy, dz: f64 = b.x - a.x, b.y - a.y, b.z - a.z;
					distance: f64 = math.sqrt(dx * dx + dy * dy + dz * dz);
					denominator: f64 = 12.5 / 64 + 1.5;
					error_to_velocity: f64 = 12.5 / denominator;
					effective_scale: f64 = 1 / (1 + 1 / ((12.5 / 64) * denominator));
					impulse: f64 = (distance - 1) * error_to_velocity * effective_scale / f64(2 - partner);
					expected: util.Vector3 = {f32(dx / distance * impulse), f32(dy / distance * impulse), f32(dz / distance * impulse)};
					// 2 micrometers per second covers the existing float kernel approximations
					expect_vector(t, body_motion(t, &scene, handles[0]).velocity.linear, expected, 2e-6);
					if partner == 0
					{
						expect_vector(t, body_motion(t, &scene, handles[1]).velocity.linear, util.vector3_negate(expected), 2e-6);
					}
					else
					{
						testing.expect_value(t, body_motion(t, &scene, handles[1]).velocity, physics.Body_Velocity{});
					}
					reaction: physics.Constraint_Reaction_Sample;
					testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, joint, 1.0 / 64, {}, &reaction), physics.Physics_Status.Ok);
					expect_vector(t, reaction.forces[0].linear, util.vector3_scale(expected, 64), 0.000128);
					testing.expect_value(t, body_position(t, &scene, handles[0]), a);
					testing.expect_value(t, body_position(t, &scene, handles[1]), b);
					scene_dispose(t, &scene);
				}
			}
		}
	}
}

@(test)
precision_constraint_range_failure_preserves_published_motion_and_reaction :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	limit: f64 = f64(f32(math.F32_MAX));
	handles: [4]physics.Body_Handle = {add_body(t, &scene, {-limit, 0, 0}), add_body(t, &scene, {limit, 0, 0}), {}, {}};
	distance: physics.Center_Distance_Constraint = {1, {angular_frequency=12.5, twice_damping_ratio=1.5}};
	joint: physics.Constraint_Handle;
	status: physics.Physics_Status;
	joint, status = physics.solver_add(&scene.simulation.solver, &handles, &distance);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	batch: ^physics.Type_Batch = constraint_batch(t, &scene, joint);
	before_a: physics.Motion_State = body_motion(t, &scene, handles[0])^;
	before_b: physics.Motion_State = body_motion(t, &scene, handles[1])^;
	testing.expect_value(t, physics.solver_execute_direct_noncontact_bundle_integration(&scene.simulation.solver,
		batch, 0, 0, 1.0 / 64, 64, .First, .Solve, 0, physics.CENTER_DISTANCE_CONSTRAINT_TYPE_ID, 2, .Never), physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, body_motion(t, &scene, handles[0])^, before_a);
	testing.expect_value(t, body_motion(t, &scene, handles[1])^, before_b);
	sample: physics.Constraint_Reaction_Sample = {maximum_force=123};
	before: physics.Constraint_Reaction_Sample = sample;
	testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, joint, 1.0 / 64, {}, &sample), physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, sample, before);
	samples: [8]physics.Constraint_Reaction_Sample;
	samples[0] = before;
	testing.expect_value(t, physics.constraint_capture_reaction_bundle(&scene.simulation.solver, joint, 1, 1.0 / 64, {}, &samples), physics.Physics_Status.Invalid_Argument);
	testing.expect_value(t, samples[0], before);
	// a position-free angular motor does not reject an irrelevant separation
	angular: physics.Angular_Motor = {{1, 2, 3}, {maximum_force=750, damping=20}};
	joint, status = physics.solver_add(&scene.simulation.solver, &handles, &angular);
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	batch = constraint_batch(t, &scene, joint);
	testing.expect_value(t, physics.solver_execute_direct_noncontact_bundle_integration(&scene.simulation.solver,
		batch, 1, 0, 1.0 / 64, 64, .First, .Solve, 0, physics.ANGULAR_MOTOR_TYPE_ID, 2, .Never), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, joint, 1.0 / 64, {}, &sample), physics.Physics_Status.Ok);
	testing.expect_value(t, physics.constraint_capture_reaction_bundle(&scene.simulation.solver, joint, 1, 1.0 / 64, {}, &samples), physics.Physics_Status.Ok);
	testing.expect(t, sample.maximum_torque > 0);
}
