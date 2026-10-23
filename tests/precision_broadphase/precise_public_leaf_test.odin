package precision_broadphase_tests

import "core:math"
import "core:testing"
import e "entasis:entasis"
import p "entasis:entasis_physics"
import u "entasis:entasis_utilities"

@(test)
precision_bulk_static_leaf_columns_preserve_valid_prefix_and_pair_admission :: proc(t: ^testing.T)
{
	bad_indices: [2]int = {-1, 17};
	for bad_index in bad_indices
	{
		world: e.World;
		description: e.World_Description = e.world_description_default();
		description.gravity = {};
		testing.expect_value(t, e.world_init(&world, description), e.Status.Ok);
		scene: Scene;
		status: e.Status;
		scene.simulation, status = e.world_borrow_simulation(&world);
		testing.expect_value(t, status, e.Status.Ok);
		sphere: p.Sphere = {0.0625};
		shape: p.Typed_Index;
		shape, status = p.shape_registry_add(&scene.simulation.shapes, p.SPHERE_TYPE_ID, &sphere);
		testing.expect_value(t, status, e.Status.Ok);
		input: [33]e.Static_Description;
		output: [33]e.Static_Handle;
		for index in 0 ..< len(input)
		{
			input[index] = {shape=shape, pose={position={1e9 + f32(index) * 128, -1e9, 1e9}, orientation={w=1}}};
		}
		if bad_index >= 0
		{
			input[bad_index].pose.position.x = math.F32_MAX;
		}
		count: int;
		count, status = e.static_add_batch(&world, input[:], output[:]);
		testing.expect_value(t, count, len(input) if bad_index < 0 else bad_index);
		testing.expect_value(t, status, e.Status.Ok if bad_index < 0 else e.Status.Invalid_Argument);
		testing.expect_value(t, scene.simulation.broad_phase.static_tree.leaf_count, count);
		for index in 0 ..< count
		{
			position: p.Global_Position = {f64(input[index].pose.position.x), -1e9, 1e9};
			expect_precise_exact(t, static_precise_bounds(t, &scene, output[index]), position,
				{min={-0.0625, -0.0625, -0.0625}, max={0.0625, 0.0625, 0.0625}});
		}
		body: p.Body_Handle = add_body(t, &scene, shape, {1e9 + 0.125 + 1.0/512, -1e9, 1e9});
		testing.expect_value(t, pair_count(t, &scene), 0);
		testing.expect_value(t, e.body_set_global_pose(&world, body,
			{position={1e9 + 0.125, -1e9, 1e9}, orientation=u.quaternion_identity()}), e.Status.Ok);
		testing.expect_value(t, pair_count(t, &scene), 1);
		testing.expect_value(t, e.world_destroy(&world), e.Status.Ok);
	}
}
