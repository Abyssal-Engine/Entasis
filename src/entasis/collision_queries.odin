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

// collision_query_batch executes direct shape-pair tests in input order. every
// input is attempted. the return value is the first non-Ok result status
collision_query_batch :: proc "contextless" (
	world: ^World,
	queries: []Collision_Query,
	results: []Collision_Query_Result,
) -> Status
{
	if len(results) < len(queries)
	{
		return .Invalid_Argument;
	}
	overall := Status.Ok;
	for index in 0 ..< len(queries)
	{
		query := queries[index];
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
