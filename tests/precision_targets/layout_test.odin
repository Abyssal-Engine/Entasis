package precision_targets_tests

import "core:simd"
import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

@(test)
precision_target_payload_keeps_original_float_prefix_and_source_signature :: proc(t: ^testing.T)
{
	scene: Scene;
	scene_initialize(t, &scene);
	defer scene_dispose(t, &scene);
	description: physics.One_Body_Linear_Servo = {{0.25, 0.5, 0.75}, {2, 1, -1},
		{angular_frequency=12.5, twice_damping_ratio=1.5}, {maximum_speed=25, base_speed=0.5, maximum_force=1000}};
	handles: [8]physics.Constraint_Handle;
	for lane in 0 ..< 8
	{
		body: physics.Body_Handle = add_body(t, &scene, {});
		references: [4]physics.Body_Handle = {body, {}, {}, {}};
		status: physics.Physics_Status;
		handles[lane], status = physics.solver_add(&scene.simulation.solver, &references, &description);
		testing.expect_value(t, status, physics.Physics_Status.Ok);
	}
	batch: ^physics.Type_Batch = constraint_batch(t, &scene, handles[0]);
	prefix: physics.One_Body_Linear_Servo_Prestep;
	testing.expect_value(t, int(batch.prestep_bundle_size), size_of(prefix) + 8 * size_of(physics.Global_Position));
	testing.expect_value(t, int(scene.simulation.solver.registry.records[physics.ONE_BODY_LINEAR_SERVO_TYPE_ID].description_size), size_of(description));
	for lane in 0 ..< 8
	{
		physics.constraint_description_transfer(&description, &prefix, physics.ONE_BODY_LINEAR_SERVO_TYPE_ID,
			size_of(description), size_of(prefix), lane);
	}
	actual: [^]util.F32x8 = ([^]util.F32x8)(physics.type_batch_prestep_bundle(batch, 0));
	expected: [^]util.F32x8 = ([^]util.F32x8)(&prefix);
	for field in 0 ..< size_of(prefix) / size_of(util.F32x8)
	{
		testing.expect_value(t, actual[field], expected[field]);
	}
	for lane in 0 ..< 8
	{
		// the unchanged BEPU-derived float-prefix oracle is 11340. the suffix
		// contains opaque f64 bits, so it is not another set of float fields
		signature: f32;
		for field in 0 ..< size_of(prefix) / size_of(util.F32x8)
		{
			signature += simd.extract(actual[field], lane) * f32(field + 1);
		}
		testing.expect_value(t, signature, f32(11340));
		expect_target(t, &scene, handles[lane], {2, 1, -1});
	}
}
