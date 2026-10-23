package query_extensions

import "core:math"
import "core:testing"
import e "entasis:entasis"

@(test)
hull_ray_oracle_rejects_invalid_hit_values :: proc(t: ^testing.T)
{
	for operation in ([3]string{"ray-direct", "ray-closest", "ray-all"})
	{
		observations: [2]Hierarchy_Observation = {
			{status=.Ok, count=1, t=4, normal={0, 1, 0}},
			{status=.Not_Found if operation == "ray-closest" else .Ok, t=math.nan_f32(), child=17, normal={math.nan_f32(), 0, 0}},
		};
		fixture: Hierarchy_Case = {operation=operation, observations=observations[:]};
		valid: Hierarchy_Observation = observations[0];
		for error in ([3]f32{-0.000001, 0, 0.000001})
		{
			observations[0].t = 4+error;
			observations[0].normal = {error, 1+error, -error};
			hits: u64;
			checksum: f64;
			status: e.Status;
			hits, checksum, status = hull_ray_validate(&fixture);
			testing.expect_value(t, status, e.Status.Ok);
			testing.expect_value(t, hits, u64(HIERARCHY_PASSES));
			testing.expect_value(t, checksum, f64(HIERARCHY_PASSES));
		}
		for field in 0 ..< 4
		{
			expected: [4]f32 = {4, 0, 1, 0};
			for value in ([4]f32{math.nan_f32(), math.inf_f32(1), math.inf_f32(-1), expected[field]+0.000004})
			{
				observations[0] = valid;
				switch field
				{
				case 0: observations[0].t = value;
				case 1: observations[0].normal.x = value;
				case 2: observations[0].normal.y = value;
				case 3: observations[0].normal.z = value;
				}
				hits: u64;
				checksum: f64;
				status: e.Status;
				hits, checksum, status = hull_ray_validate(&fixture);
				testing.expect_value(t, status, e.Status.Invalid_Description);
				testing.expect_value(t, hits, u64(0));
				testing.expect_value(t, checksum, f64(0));
			}
		}
		observations[0] = valid;
		observations[0].child = 1;
		status: e.Status;
		_, _, status = hull_ray_validate(&fixture);
		testing.expect_value(t, status, e.Status.Invalid_Description);
	}

	observations: [2]Hierarchy_Observation = {
		{status=.Ok, count=1, t=math.nan_f32(), child=17, normal={math.nan_f32(), math.nan_f32(), math.nan_f32()}},
		{status=.Ok, t=math.inf_f32(1), normal={math.nan_f32(), 0, 0}},
	};
	fixture: Hierarchy_Case = {operation="ray-any", observations=observations[:]};
	hits: u64;
	checksum: f64;
	status: e.Status;
	hits, checksum, status = hull_ray_validate(&fixture);
	testing.expect_value(t, status, e.Status.Ok);
	testing.expect_value(t, hits, u64(HIERARCHY_PASSES));
	testing.expect_value(t, checksum, f64(HIERARCHY_PASSES));
	observations[1].status = .Not_Found;
	_, _, status = hull_ray_validate(&fixture);
	testing.expect_value(t, status, e.Status.Invalid_Description);
	observations[1].status = .Ok;
	observations[0].count = 0;
	_, _, status = hull_ray_validate(&fixture);
	testing.expect_value(t, status, e.Status.Invalid_Description);
}
