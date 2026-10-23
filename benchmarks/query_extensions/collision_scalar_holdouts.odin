package query_extensions

import e "entasis:entasis"
import p "entasis:entasis_physics"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

// these cells measure scalar box-triangle and box-hull collision routes
collision_scalar_holdout_shapes :: proc(world: ^e.World, shape: Collision_Batch_Shape) -> (e.Shape_Handle, e.Shape_Handle)
{
	a, b: e.Shape_Handle;
	status: e.Status;
	a, status = e.shape_add(world, e.box(2, 2, 2) if shape == .Triangle else e.box(1, 1, 1));
	support.extension_require(status, "collision scalar Box");
	if shape == .Triangle
	{
		b, status = e.shape_add(world, e.triangle({0, -4, -4}, {0, 4, -4}, {0, 0, 4}));
		support.extension_require(status, "collision scalar Triangle");
	}
	else
	{
		points: [8]util.Vector3 = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}};
		starts: [6]i32 = {0, 4, 8, 12, 16, 20};
		indices: [24]i32 = {0, 3, 2, 1, 4, 5, 6, 7, 0, 4, 7, 3, 1, 2, 6, 5, 0, 1, 5, 4, 3, 7, 6, 2};
		pool: util.Buffer_Pool;
		if util.buffer_pool_initialize(&pool, 128) != .Ok
		{
			panic("collision scalar hull pool");
		}
		defer util.buffer_pool_dispose(&pool);
		hull: p.Convex_Hull;
		support.extension_require(p.convex_hull_create(&hull, &points[0], 8, &starts[0], 6, &indices[0], 24, &pool), "collision scalar hull topology");
		defer p.convex_hull_dispose(&hull, &pool);
		b, status = e.shape_import_convex_hull(world, &hull);
		support.extension_require(status, "collision scalar hull import");
	}
	return a, b;
}

collision_batch_input :: proc "contextless" (a, b: e.Shape_Handle, index: int, shape: Collision_Batch_Shape) -> e.Collision_Query
{
	separation: f32 = 0.75 if shape == .Triangle else 1.25;
	query: e.Collision_Query = {shape_a=a, shape_b=b, pose_a=e.pose(), pose_b=e.pose({separation if index%2 == 0 else 4, 0, 0})};
	if index/2%2 != 0
	{
		query.shape_a, query.shape_b = query.shape_b, query.shape_a;
		query.pose_a, query.pose_b = query.pose_b, query.pose_a;
	}
	return query;
}

collision_scalar_holdout_oracle :: proc "contextless" (query: e.Collision_Query, result: e.Collision_Query_Result, index: int, shape: Collision_Batch_Shape) -> e.Status
{
	expected_status: e.Status = .Ok if index%2 == 0 else .Not_Found;
	if result.status != expected_status || result.hit != (expected_status == .Ok) || result.manifold.kind != .Convex
	{
		return .Invalid_Description;
	}
	manifold: e.Convex_Contact_Manifold = result.manifold.convex;
	if expected_status == .Not_Found
	{
		return .Ok if manifold.count == 0 else .Invalid_Description;
	}
	expected_offset: e.Vector3 = util.vector3_subtract(query.pose_b.position, query.pose_a.position);
	if manifold.offset_b != expected_offset
	{
		return .Invalid_Description;
	}
	normal_x: f32 = -1 if index/2%2 == 0 else 1;
	contact_x: f32 = (0.75 if index/2%2 == 0 else 0) if shape == .Triangle else (0.25 if index/2%2 == 0 else -1);
	span: f32 = 1 if shape == .Triangle else 0.5;
	if manifold.count < 1 || manifold.count > 4 || !(abs(manifold.normal.x-normal_x) <= 1e-5) ||
		!(abs(manifold.normal.y) <= 1e-5) || !(abs(manifold.normal.z) <= 1e-5)
	{
		return .Invalid_Description;
	}
	for contact in manifold.contacts[:manifold.count]
	{
		if !(abs(contact.depth-0.25) <= 1e-5) || !(abs(contact.offset.x-contact_x) <= 1e-5) ||
			!(abs(contact.offset.y) <= span+1e-5) || !(abs(contact.offset.z) <= span+1e-5)
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}
