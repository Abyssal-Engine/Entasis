#+vet !unused-imports

package trees_tests

import "core:testing"
import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

when util.PRECISION_MODE == .Double_Coordinates
{
	Bounds :: physics.Global_Bounds;
	Position :: physics.Global_Position;
	Ray :: physics.Tree_Global_Ray;
	Callback :: physics.Tree_Global_Ray_Leaf_Proc;
	
	Limit_Probe :: struct
	{
	    incoming: f64,
	    reduced: f64,
	    visits: int,
	    status: physics.Physics_Status,
	}
	
	record_double_limit :: proc "contextless" (raw: rawptr, index: int, maximum_t: ^f64) -> physics.Physics_Status
	{
	    probe := (^Limit_Probe)(raw);
	    probe.incoming = maximum_t^;
	    probe.visits += 1;
	    maximum_t^ = min(maximum_t^, probe.reduced);
	    return probe.status;
	}
	
	@(test)
	double_tree_callbacks_preserve_limit_residue_and_failures :: proc(t: ^testing.T)
	{
	    pool: util.Buffer_Pool;
	    prepare_pool(t, &pool);
	    defer util.buffer_pool_dispose(&pool);
	    tree: physics.Tree;
	    testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
	    defer physics.tree_dispose(&tree);
	    // the only leaf intersects before the callback's reduced limit
	    _, add_status := physics.tree_add(&tree, Bounds{min={1e9, -1, -1}, max={1e9 + 0.001, 1, 1}});
	    testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	    ray := Ray{origin={}, direction={1, 0, 0}, maximum_t=1e9 + 0.01};
	    callback: Callback = record_double_limit;
	    for _ in 0 ..< 1024
	    {
	        for operation in 0 ..< 4
	        {
	            probe := Limit_Probe{reduced=1e9 + 0.002, status=.Ok};
	            actual: f64;
	            status: physics.Physics_Status;
	            switch operation
	            {
	                case 0: actual, status = physics.tree_ray_traverse(&tree, ray, record_double_limit, &probe);
	                case 1: actual, status = physics.tree_ray_traverse_closest(&tree, ray, callback, &probe);
	                case 2: actual, status = physics.tree_sweep_traverse(&tree, Bounds{}, Position{1, 0, 0}, ray.maximum_t, callback, &probe);
	                case 3: actual, status = physics.tree_sweep_traverse_closest(&tree, Bounds{}, Position{1, 0, 0}, ray.maximum_t, record_double_limit, &probe);
	            }
	            testing.expect_value(t, status, physics.Physics_Status.Ok);
	            testing.expect_value(t, probe.visits, 1);
	            testing.expect_value(t, probe.incoming, ray.maximum_t);
	            testing.expect_value(t, actual, probe.reduced);
	        }
	    }
	    failed := Limit_Probe{reduced=1e9 + 0.002, status=.Capacity_Missing};
	    actual, status := physics.tree_ray_traverse(&tree, ray, callback, &failed);
	    testing.expect_value(t, status, physics.Physics_Status.Capacity_Missing);
	    testing.expect_value(t, actual, failed.reduced);
	    testing.expect_value(t, failed.visits, 1);
	    missing: Callback;
	    _, nil_status := physics.tree_ray_traverse(&tree, ray, missing, &failed);
	    testing.expect_value(t, nil_status, physics.Physics_Status.Invalid_Argument);
	    testing.expect_value(t, failed.visits, 1);
	}
	
	@(test)
	double_tree_parallel_axes_do_not_clip_at_binary32_maximum :: proc(t: ^testing.T)
	{
	    pool: util.Buffer_Pool;
	    prepare_pool(t, &pool);
	    defer util.buffer_pool_dispose(&pool);
	    tree: physics.Tree;
	    testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
	    defer physics.tree_dispose(&tree);
	    for count in 1 ..= 2
	    {
	        bounds := Bounds{min={f64(count) * 1e100, -1, -1}, max={f64(count) * 1e100 + 1e90, 1, 1}};
	        _, add_status := physics.tree_add(&tree, bounds);
	        testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	        ray := Ray{origin={}, direction={1, 0, 0}, maximum_t=3e100};
	        _, reference := physics.tree_ray_intersection_prepared(bounds, physics.tree_ray_traversal_data(ray), ray.maximum_t);
	        testing.expect_value(t, reference, physics.Reference_State.Present);
	        for operation in 0 ..< 4
	        {
	            probe := Limit_Probe{reduced=ray.maximum_t, status=.Ok};
	            actual: f64;
	            status: physics.Physics_Status;
	            switch operation
	            {
	                case 0: actual, status = physics.tree_ray_traverse(&tree, ray, record_double_limit, &probe);
	                case 1: actual, status = physics.tree_ray_traverse_closest(&tree, ray, record_double_limit, &probe);
	                case 2: actual, status = physics.tree_sweep_traverse(&tree, Bounds{}, Position{1, 0, 0}, ray.maximum_t, record_double_limit, &probe);
	                case 3: actual, status = physics.tree_sweep_traverse_closest(&tree, Bounds{}, Position{1, 0, 0}, ray.maximum_t, record_double_limit, &probe);
	            }
	            testing.expect_value(t, status, physics.Physics_Status.Ok);
	            testing.expect_value(t, probe.visits, count);
	            testing.expect_value(t, actual, ray.maximum_t);
	        }
	    }
	}
	
	record_local_limit :: proc "contextless" (raw: rawptr, index: int, maximum_t: ^f32) -> physics.Physics_Status
	{
	    probe := (^Limit_Probe)(raw);
	    probe.incoming = f64(maximum_t^);
	    probe.visits += 1;
	    maximum_t^ = min(maximum_t^, f32(probe.reduced));
	    return probe.status;
	}
	
	@(test)
	mixed64_local_float_callback_contract_remains_explicit :: proc(t: ^testing.T)
	{
	    pool: util.Buffer_Pool;
	    prepare_pool(t, &pool);
	    defer util.buffer_pool_dispose(&pool);
	    tree: physics.Tree;
	    testing.expect_value(t, physics.tree_initialize(&tree, 4, &pool), physics.Physics_Status.Ok);
	    defer physics.tree_dispose(&tree);
	    _, add_status := physics.tree_add(&tree, Bounds{min={1, -1, -1}, max={2, 1, 1}});
	    testing.expect_value(t, add_status, physics.Physics_Status.Ok);
	    global_ray := Ray{origin={}, direction={1, 0, 0}, maximum_t=1e9 + 0.01};
	    probe := Limit_Probe{reduced=1e9 + 0.002, status=.Ok};
	    local_callback: physics.Tree_Ray_Leaf_Proc = record_local_limit;
	    actual, status := physics.tree_ray_traverse(&tree, global_ray, local_callback, &probe);
	    testing.expect_value(t, status, physics.Physics_Status.Ok);
	    testing.expect_value(t, probe.visits, 1);
	    testing.expect_value(t, probe.incoming, f64(1e9 + 64));
	    testing.expect_value(t, actual, f64(f32(probe.reduced)));
	    probe = {reduced=3, status=.Ok};
	    local_ray := physics.Tree_Ray{origin={}, direction={1, 0, 0}, maximum_t=4};
	    local_result, local_status := physics.tree_ray_traverse(&tree, local_ray, record_local_limit, &probe);
	    testing.expect_value(t, local_status, physics.Physics_Status.Ok);
	    testing.expect_value(t, local_result, f32(3));
	    testing.expect_value(t, probe.incoming, f64(4));
	}
}
