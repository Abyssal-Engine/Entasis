package trees_tests

import "core:testing"
import "core:fmt"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

Rescaling_Bounds :: physics.Tree_Bounds;
Rescaling_Position :: physics.Tree_Position;
Rescaling_Ray :: physics.Tree_Coordinate_Ray;

Rescaling_Visitor :: struct
{
    count: int,
}

rescaling_visit :: proc "contextless" (raw: rawptr, index: int, maximum_t: ^physics.Tree_Scalar) -> physics.Physics_Status
{
    state := (^Rescaling_Visitor)(raw);
    state.count += 1;
    return .Ok;
}

@(test)
finite_nonunit_ray_and_sweep_rescaling_preserves_candidates :: proc(t: ^testing.T)
{
    pool: util.Buffer_Pool;
    testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
    defer util.buffer_pool_dispose(&pool);
    tree: physics.Tree;
    testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
    defer physics.tree_dispose(&tree);
    _, added := physics.tree_add(&tree, Rescaling_Bounds{min={1, -1, -1}, max={1.5, 1, 1}});
    testing.expect_value(t, added, physics.Physics_Status.Ok);
    scales := [3]physics.Tree_Scalar{1, 1e-19, 1e-21};
    for scale in scales
    {
        // every ray/sweep traverses the same segment x=0..2, y=z=0
        ray := Rescaling_Ray{origin={}, direction={scale, 0, 0}, maximum_t=2 / scale};
        testing.expect(t, abs(ray.direction.x * ray.maximum_t - 2) < 1e-5);
        for operation in 0 ..< 4
        {
            state: Rescaling_Visitor;
            status: physics.Physics_Status;
            switch operation
            {
                case 0: _, status = physics.tree_ray_traverse(&tree, ray, rescaling_visit, &state);
                case 1: _, status = physics.tree_ray_traverse_closest(&tree, ray, rescaling_visit, &state);
                case 2: _, status = physics.tree_sweep_traverse(&tree, Rescaling_Bounds{}, Rescaling_Position{scale, 0, 0}, ray.maximum_t, rescaling_visit, &state);
                case 3: _, status = physics.tree_sweep_traverse_closest(&tree, Rescaling_Bounds{}, Rescaling_Position{scale, 0, 0}, ray.maximum_t, rescaling_visit, &state);
            }
            fmt.printf("scale=%g operation=%d status=%v visits=%d\n", scale, operation, status, state.count);
            testing.expect_value(t, status, physics.Physics_Status.Ok);
            testing.expect_value(t, state.count, 1);
        }
    }
}

@(test)
subnormal_direction_preserves_boundary_hits_without_reciprocal_overflow :: proc(t: ^testing.T)
{
	pool: util.Buffer_Pool;
	testing.expect_value(t, util.buffer_pool_initialize(&pool, 128), util.Memory_Status.Ok);
	defer util.buffer_pool_dispose(&pool);
	tree: physics.Tree;
	testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
	defer physics.tree_dispose(&tree);
	step: physics.Tree_Scalar = 1e-40;
	_, status := physics.tree_add(&tree, Rescaling_Bounds{min={step, -1, -1}, max={2 * step, 1, 1}});
	testing.expect_value(t, status, physics.Physics_Status.Ok);
	for origin in ([3]physics.Tree_Scalar{0, step, 2 * step})
	{
		for sign in ([2]physics.Tree_Scalar{1, -1})
		{
			ray: Rescaling_Ray = {origin={origin, 0, 0}, direction={sign * step, 0, 0}, maximum_t=2};
			for operation in 0 ..< 4
			{
				state: Rescaling_Visitor;
				switch operation
				{
				case 0: _, status = physics.tree_ray_traverse(&tree, ray, rescaling_visit, &state);
				case 1: _, status = physics.tree_ray_traverse_closest(&tree, ray, rescaling_visit, &state);
				case 2: _, status = physics.tree_sweep_traverse(&tree,
					Rescaling_Bounds{min=ray.origin, max=ray.origin}, ray.direction, ray.maximum_t, rescaling_visit, &state);
				case 3: _, status = physics.tree_sweep_traverse_closest(&tree,
					Rescaling_Bounds{min=ray.origin, max=ray.origin}, ray.direction, ray.maximum_t, rescaling_visit, &state);
				}
				expected: int = 1;
				if origin == 0 && sign < 0
				{
					expected = 0;
				}
				testing.expect_value(t, status, physics.Physics_Status.Ok);
				testing.expect_value(t, state.count, expected);
			}
		}
	}
}
