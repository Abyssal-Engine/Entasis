package entasis

import physics "entasis:entasis_physics"
import util "entasis:entasis_utilities"

// Collision_Query describes one direct registered-shape pair test
Collision_Query :: struct
{
	shape_a: Shape_Handle,
	shape_b: Shape_Handle,
	pose_a:  Rigid_Pose,
	pose_b:  Rigid_Pose,
	speculative_margin: f32,
}

// Collision_Query_Result is a pointer-free direct manifold result
Collision_Query_Result :: struct
{
	manifold: Manifold_Result,
	status:   Status,
	hit:      bool,
}

// collision_query tests two registered shapes directly without inserting them
// into the world. it supports convex and compound/mesh task routes
// allocation: existing world collision scratch may be reused
collision_query :: proc "contextless" (
	world: ^World,
	shape_a: Shape_Handle,
	pose_a: Rigid_Pose,
	shape_b: Shape_Handle,
	pose_b: Rigid_Pose,
	speculative_margin: f32 = 0,
) -> (Manifold_Result, Status)
{
	data := world_data_get(world);
	if data == nil || data.simulation.state == .Disposed
	{
		return {}, .Disposed;
	}
	if data.simulation.state != .Ready
	{
		return {}, .Invalid_Argument;
	}
	if speculative_margin < 0
	{
		return {}, .Invalid_Argument;
	}
	simulation := &data.simulation;
	registry := physics.simulation_shape_registry(simulation);
	raw_a, _, resolve_a := physics.shape_registry_resolve(registry, shape_a);
	if resolve_a != .Ok
	{
		return {}, resolve_a;
	}
	raw_b, _, resolve_b := physics.shape_registry_resolve(registry, shape_b);
	if resolve_b != .Ok
	{
		return {}, resolve_b;
	}
	type_a := int(physics.typed_index_type(shape_a));
	type_b := int(physics.typed_index_type(shape_b));
	task, _, lookup_status := physics.collision_task_registry_lookup(
		&simulation.collision_tasks, type_a, type_b,
	);
	if lookup_status != .Ok
	{
		return {}, lookup_status;
	}
	if task.kind == .Convex
	{
		manifold, status := physics.collision_task_registry_test_convex(
			&simulation.collision_tasks, type_a, type_b,
			raw_a, raw_b, pose_a, pose_b, speculative_margin, registry,
		);
		if status != .Ok
		{
			return {}, status;
		}
		result := Manifold_Result{kind=.Convex, convex=manifold};
		if manifold.count <= 0
		{
			return result, .Not_Found;
		}
		return result, .Ok;
	}
	if speculative_margin != 0
	{
		return {}, .Invalid_Argument;
	}
	if simulation.query_overlap_batcher.state == .Uninitialized
	{
		bind_status := physics.query_overlap_batcher_bind(
			&simulation.query_overlap_batcher, &simulation.narrow_phase,
		);
		if bind_status != .Ok
		{
			return {}, bind_status;
		}
	}
	hit: physics.Overlap_Query_Hit;
	collector := physics.Overlap_Query_Collector{
		hits={memory=&hit, length=1, id=util.BUFFER_CALLER_OWNED_ID},
	};
	status := physics.query_overlap_batcher_test(
		&simulation.query_overlap_batcher,
		shape_a, shape_b, pose_a, pose_b, 0, &collector,
	);
	if status != .Ok
	{
		return {}, status;
	}
	if collector.count == 0
	{
		return {}, .Not_Found;
	}
	return hit.manifold, .Ok;
}

// consumes only a contiguous callback-free prefix, publishing before any barrier
collision_query_box_run :: #force_no_inline proc "contextless" (
	data: ^world_data, queries: []Collision_Query, results: []Collision_Query_Result,
) -> int
{
	if data == nil || data.simulation.state != .Ready
	{
		return 0;
	}
	registry: ^physics.Shape_Registry = physics.simulation_shape_registry(&data.simulation);
	canonical: physics.Shape_Type_Registration = {
		size_of(physics.Box), align_of(physics.Box), .Convex,
		physics.box_bounds_proc, physics.box_inertia_proc, physics.box_ray_proc,
		physics.box_support_proc, physics.box_support_proc, physics.shape_no_dispose,
	};
	a, b: physics.Box_Wide;
	offset_b: util.Vector3_Wide;
	orientation_a, orientation_b: util.Quaternion_Wide;
	margins: [8]f32;
	flipped: u8;
	count: int;
	for query in queries[:min(len(queries), 8)]
	{
		if query.speculative_margin < 0
		{
			break;
		}
		raw_a, raw_b: rawptr;
		batch_a, batch_b: ^physics.Shape_Batch;
		status: Status;
		raw_a, batch_a, status = physics.shape_registry_resolve(registry, query.shape_a);
		if status != .Ok
		{
			break;
		}
		raw_b, batch_b, status = physics.shape_registry_resolve(registry, query.shape_b);
		if status != .Ok
		{
			break;
		}
		type_a: int = int(physics.typed_index_type(query.shape_a));
		type_b: int = int(physics.typed_index_type(query.shape_b));
		task: ^physics.Collision_Task;
		reference: physics.Collision_Task_Reference;
		task, reference, status = physics.collision_task_registry_lookup(&data.simulation.collision_tasks, type_a, type_b);
		if status != .Ok || type_a != int(SHAPE_TYPE_BOX) || type_b != int(SHAPE_TYPE_BOX) ||
			task.kind != .Convex || task.dispatch != .Native || task.convex_test != physics.box_pair_test ||
			batch_a.dispatch != .Native || batch_b.dispatch != .Native ||
			batch_a.metadata != canonical || batch_b.metadata != canonical
		{
			break;
		}
		box_a: physics.Box = (^physics.Box)(raw_a)^;
		box_b: physics.Box = (^physics.Box)(raw_b)^;
		if physics.box_validate(box_a) != .Ok || physics.box_validate(box_b) != .Ok
		{
			break;
		}
		pose_a: Rigid_Pose = query.pose_a;
		pose_b: Rigid_Pose = query.pose_b;
		if reference.order == .Flipped
		{
			box_a, box_b = box_b, box_a;
			pose_a, pose_b = pose_b, pose_a;
			flipped |= u8(1)<<u8(count);
		}
		_ = physics.box_wide_write_slot(&a, count, box_a);
		_ = physics.box_wide_write_slot(&b, count, box_b);
		util.vector3_wide_write_slot(&offset_b, count, util.vector3_subtract(pose_b.position, pose_a.position));
		util.quaternion_wide_write_slot(&orientation_a, count, pose_a.orientation);
		util.quaternion_wide_write_slot(&orientation_b, count, pose_b.orientation);
		margins[count] = query.speculative_margin;
		count += 1;
	}
	if count == 0
	{
		return 0;
	}
	wide: physics.Convex_4_Contact_Manifold_Wide;
	wide_status: Status;
	wide, wide_status = physics.box_pair_test_wide(a, b, transmute(util.F32x8)margins, offset_b, orientation_a, orientation_b, count);
	for lane in 0..<count
	{
		manifold: physics.Convex_Contact_Manifold;
		status: Status = wide_status;
		if status == .Ok
		{
			manifold, status = physics.convex_4_manifold_wide_read_lane(&wide, offset_b, lane);
		}
		result: Collision_Query_Result = {status=status};
		if status == .Ok
		{
			if flipped & (u8(1)<<u8(lane)) != 0
			{
				manifold = physics.convex_manifold_flip(manifold);
			}
			result.manifold = {kind=.Convex, convex=manifold};
			result.status = .Ok if manifold.count > 0 else .Not_Found;
			result.hit = result.status == .Ok;
		}
		destination: ^Collision_Query_Result = &results[lane];
		destination.manifold = result.manifold;
		destination.status = result.status;
		destination.hit = result.hit;
	}
	return count;
}

// collision_query_batch executes direct shape-pair tests in input order. every
// input is attempted. the return value is the first non-Ok result status
collision_query_batch :: #force_inline proc "contextless" (
	world: ^World,
	queries: []Collision_Query,
	results: []Collision_Query_Result,
) -> Status
{
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	if len(queries) < util.PRODUCTION_LANE_COUNT
	{
		return collision_query_batch_scalar_range(world, queries, results);
	}
	return collision_query_batch_wide(world, queries, results);
}

@(private="file")
collision_query_batch_scalar_range :: #force_inline proc "contextless" (
	world: ^World, queries: []Collision_Query, results: []Collision_Query_Result,
) -> Status
{
	overall: Status = .Ok;
	for index in 0..<len(queries)
	{
		query: Collision_Query = queries[index];
		manifold, status := collision_query(
			world, query.shape_a, query.pose_a,
			query.shape_b, query.pose_b, query.speculative_margin,
		);
		result: ^Collision_Query_Result = &results[index];
		result.manifold = manifold;
		result.status = status;
		result.hit = status == .Ok;
		if overall == .Ok && status != .Ok && status != .Not_Found
		{
			overall = status;
		}
	}
	return overall;
}

@(private="file")
collision_query_batch_wide :: #force_no_inline proc "contextless" (
	world: ^World, queries: []Collision_Query, results: []Collision_Query_Result,
) -> Status
{
	overall: Status = .Ok;
	index: int;
	for index < len(queries)
	{
		consumed: int;
		if physics.typed_index_type(queries[index].shape_a) == i32(SHAPE_TYPE_BOX) &&
			physics.typed_index_type(queries[index].shape_b) == i32(SHAPE_TYPE_BOX)
		{
			consumed = collision_query_box_run(world_data_get(world), queries[index:], results[index:]);
		}
		if consumed > 0
		{
			index += consumed;
			continue;
		}
		status: Status = collision_query_batch_scalar_range(world, queries[index:index+1], results[index:index+1]);
		if overall == .Ok
		{
			overall = status;
		}
		index += 1;
	}
	return overall;
}
