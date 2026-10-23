package precision_targets_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

#assert(size_of(physics.Type_Batch) == 96);
#assert(size_of(physics.Constraint_Kernel_Body_Wide) == 640);

// float-authored targets must subtract authoritative high body positions
// before narrowing the solver displacement
@(test)
precision_float_authored_targets_use_high_body_positions :: proc(t: ^testing.T)
{
	for base in ([4]f64{0, 1e4, 1e6, 1e9})
	{
		for kind in Target_Kind
		{
			for axis in 0 ..< 3
			{
				for sign in ([2]f64{-1, 1})
				{
					scene: Scene;
					scene_initialize(t, &scene);
					target: util.Vector3 = {f32(base), f32(-base), f32(base)};
					position: physics.Global_Position = {base, -base, base};
					switch axis
					{
						case 0: position.x -= sign * 0.002;
						case 1: position.y -= sign * 0.002;
						case 2: position.z -= sign * 0.002;
					}
					body: physics.Body_Handle = add_body(t, &scene, position);
					handle: physics.Constraint_Handle = add_float_target(t, &scene, body, target, kind);
					testing.expect_value(t, execute_target_stage(t, &scene, handle, .Registered_Bundle, .Warmstart), physics.Physics_Status.Ok);
					testing.expect_value(t, execute_target_stage(t, &scene, handle, .Registered_Bundle, .Solve), physics.Physics_Status.Ok);
					// independent double equation, unit inverse mass, zero offset and
					// velocity. limits do not engage for this two-millimeter error
					denominator: f64 = 12.5 / 64 + 1.5;
					gain: f64 = (12.5 / denominator) / (1 + 1 / ((12.5 / 64) * denominator));
					expected: util.Vector3 = {f32((f64(target.x) - position.x) * gain),
						f32((f64(target.y) - position.y) * gain), f32((f64(target.z) - position.z) * gain)};
					expect_vector(t, body_motion(t, &scene, body).velocity.linear, expected, 2e-6);
					reaction: physics.Constraint_Reaction_Sample;
					testing.expect_value(t, physics.constraint_capture_reaction(&scene.simulation.solver, handle, 1.0 / 64, {}, &reaction), physics.Physics_Status.Ok);
					expect_vector(t, reaction.forces[0].linear, util.vector3_scale(expected, 64), 0.000128);
					testing.expect_value(t, body_position(t, &scene, body), position);
					scene_dispose(t, &scene);
				}
			}
		}
	}
}
