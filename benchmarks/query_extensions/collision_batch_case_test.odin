package query_extensions

import "core:testing"
import "core:fmt"
import e "entasis:entasis"
import util "entasis:entasis_utilities"

@(test)
collision_batch_geometry_oracle_rejects_invalid_payloads :: proc(t: ^testing.T)
{
	// check actual fixture geometry without a benchmark dispatch or timer
	world: e.World;
	testing.expect_value(t, e.world_init(&world, world_description(1)), e.Status.Ok);
	defer e.world_destroy(&world);
	for shape in ([2]Collision_Batch_Shape{.Triangle, .Hull})
	{
		a, b: e.Shape_Handle;
		a, b = collision_scalar_holdout_shapes(&world, shape);
		for index in 0..<4
		{
			query: e.Collision_Query = collision_batch_input(a, b, index, shape);
			result: e.Collision_Query_Result;
			result.manifold, result.status = e.collision_query(&world, query.shape_a, query.pose_a, query.shape_b, query.pose_b);
			result.hit = result.status == .Ok;
			testing.expect(t, collision_batch_geometry_oracle(query, result, index, shape) == .Ok, fmt.tprintf("shape=%v index=%d result=%v", shape, index, result));
			if index%2 == 0
			{
				expected_offset: e.Vector3 = result.manifold.convex.offset_b;
				nan: f32 = transmute(f32)u32(0x7fc00000);
				infinity: f32 = transmute(f32)u32(0x7f800000);
				for corrupt_offset in ([9]e.Vector3{
					util.vector3_add(expected_offset, {1, 0, 0}),
					util.vector3_add(expected_offset, {0, 1, 0}),
					util.vector3_add(expected_offset, {0, 0, 1}),
					{nan, expected_offset.y, expected_offset.z},
					{expected_offset.x, nan, expected_offset.z},
					{expected_offset.x, expected_offset.y, nan},
					{infinity, expected_offset.y, expected_offset.z},
					{expected_offset.x, -infinity, expected_offset.z},
					{expected_offset.x, expected_offset.y, infinity},
				})
				{
					result.manifold.convex.offset_b = corrupt_offset;
					testing.expect_value(t, collision_batch_geometry_oracle(query, result, index, shape), e.Status.Invalid_Description);
				}
				result.manifold.convex.offset_b = expected_offset;
				result.manifold.convex.contacts[0].depth = transmute(f32)u32(0x7fc00000);
				testing.expect_value(t, collision_batch_geometry_oracle(query, result, index, shape), e.Status.Invalid_Description);
			}
		}
	}

	for shape in ([2]Collision_Batch_Shape{.Box, .Sphere})
	{
		a, b: e.Shape_Handle;
		status: e.Status;
		if shape == .Box
		{
			a, status = e.shape_add(&world, e.box(2, 2, 2));
		}
		else
		{
			a, status = e.shape_add(&world, e.sphere(0.5));
		}
		testing.expect_value(t, status, e.Status.Ok);
		b, status = e.shape_add(&world, e.box(1, 1, 1) if shape == .Box else e.box(2, 2, 2));
		testing.expect_value(t, status, e.Status.Ok);
		for index in 0..<4
		{
			query: e.Collision_Query = {shape_a=a, shape_b=b, pose_a=e.pose(), pose_b=e.pose({1.25 if index%2 == 0 else 4, 0, 0})};
			if index/2 != 0
			{
				query.shape_a, query.shape_b = query.shape_b, query.shape_a;
				query.pose_a, query.pose_b = query.pose_b, query.pose_a;
			}
			result: e.Collision_Query_Result;
			result.manifold, result.status = e.collision_query(&world, query.shape_a, query.pose_a, query.shape_b, query.pose_b);
			result.hit = result.status == .Ok;
			testing.expect(t, collision_batch_geometry_oracle(query, result, index, shape) == .Ok, fmt.tprintf("shape=%v index=%d result=%v", shape, index, result));
		}
	}

	for shape in ([2]Collision_Batch_Shape{.Box, .Sphere})
	{
		for index in 0..<4
		{
			query: e.Collision_Query = {pose_a=e.pose(), pose_b=e.pose({1.25, 0, 0})};
			result: e.Collision_Query_Result = {status=.Ok, hit=index%2 == 0, manifold={kind=.Convex}};
			result.manifold.convex.offset_b = {1.25, 0, 0};
			result.manifold.convex.count = 4 if shape == .Box else 1;
			result.manifold.convex.normal = {-1 if index/2 == 0 else 1, 0, 0};
			for &contact, corner in result.manifold.convex.contacts
			{
				contact.depth = 0.25;
				contact.offset.x = (0.75 if index/2 == 0 else -0.25) if shape == .Box else (0.375 if index/2 == 0 else -0.875);
				if shape == .Box
				{
					contact.offset.y = 0.5 if corner&1 != 0 else -0.5;
					contact.offset.z = 0.5 if corner&2 != 0 else -0.5;
				}
			}
			if index%2 == 1
			{
				result.status, result.hit, result.manifold.convex.count = .Not_Found, index%2 == 0, 0;
				if shape == .Box
				{
					result.manifold.convex.offset_b = {};
				}
			}
			testing.expect(t, collision_batch_geometry_oracle(query, result, index, shape) == .Ok, fmt.tprintf("shape=%v index=%d result=%v", shape, index, result));
			if index%2 == 0
			{
				result.manifold.convex.contacts[0].offset.x = transmute(f32)u32(0x7fc00000);
				testing.expect_value(t, collision_batch_geometry_oracle(query, result, index, shape), e.Status.Invalid_Description);
				result.manifold.convex.contacts[0].offset.x = 0;
				result.manifold.convex.contacts[0].depth = transmute(f32)u32(0x7fc00000);
				testing.expect_value(t, collision_batch_geometry_oracle(query, result, index, shape), e.Status.Invalid_Description);
			}
			result.status = .Invalid_Argument;
			testing.expect_value(t, collision_batch_geometry_oracle(query, result, index, shape), e.Status.Invalid_Description);
		}
	}
}
