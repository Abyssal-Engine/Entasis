package large_world_precision

@(require) import "core:testing"
@(require) import entasis "entasis:entasis"
@(require) import support "../benchmark_support"

when PRECISION_NATIVE_GLOBAL
{
	@(test)
	mixed64_controlled_contacts_and_joint_keep_original_accuracy_limits :: proc(t: ^testing.T)
	{
		pool: entasis.Buffer_Pool;
		testing.expect_value(t, entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE), entasis.Status.Ok);
		defer testing.expect_value(t, entasis.buffer_pool_destroy(&pool), entasis.Status.Ok);
		origins: [4]entasis.Global_Position = {{10_000, 0, 0}, {1_000_000, 0, 0}, {1e9, 0, 0}, {-1e9, 1e9, -1e9}};
		for workers in ([2]int{1, 4})
		{
			for origin in origins
			{
				result: support.Benchmark_Result = precision_control_run(&pool, origin, workers);
				testing.expectf(t, result.status == .Ok, "origin=%v workers=%d result=%v", origin, workers, result);
			}
		}
	}

	@(test)
	mixed64_distributed_replays_all_groups_at_identical_authoritative_poses :: proc(t: ^testing.T)
	{
		pool: entasis.Buffer_Pool;
		testing.expect_value(t, entasis.buffer_pool_init(&pool, POOL_MINIMUM_BLOCK_SIZE), entasis.Status.Ok);
		defer testing.expect_value(t, entasis.buffer_pool_destroy(&pool), entasis.Status.Ok);
		for workers in ([2]int{1, 4})
		{
			result: support.Benchmark_Result = precision_distributed_replay(&pool, workers);
			testing.expectf(t, result.status == .Ok, "workers=%d result=%v", workers, result);
		}
	}
}
