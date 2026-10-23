package query_extensions

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:time"
import e "entasis:entasis"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

COLLISION_BATCH_INPUTS :: 24576;

Collision_Batch_Route :: enum
{
	Native_Batch, Context_Batch, Native_Scalar, Context_Scalar,
}

Collision_Batch_Shape :: enum
{
	Box, Sphere, Triangle, Hull,
}

Collision_Batch_Cell :: struct
{
	route: Collision_Batch_Route,
	shape: Collision_Batch_Shape,
	batch_size: int,
}

collision_batch_cell :: proc "contextless" (cell: string) -> (Collision_Batch_Cell, support.Admission)
{
	selection: Collision_Batch_Cell;
	switch cell
	{
	case "box-native-1", "box-context-1":
		selection.batch_size = 1;
	case "box-native-8", "box-context-8":
		selection.batch_size = 8;
	case "box-native-32", "box-context-32":
		selection.batch_size = 32;
	case "box-native-256", "box-context-256", "sphere-native-256", "sphere-context-256", "box-native-scalar", "box-context-scalar", "sphere-native-scalar", "sphere-context-scalar", "box-triangle-scalar", "box-hull-scalar":
		selection.batch_size = 256;
	case: return {}, .Invalid;
	}
	selection.shape = .Sphere if cell[:7] == "sphere-" else .Box;
	switch cell
	{
	case "box-context-1", "box-context-8", "box-context-32", "box-context-256", "sphere-context-256":
		selection.route = .Context_Batch;
	case "box-context-scalar", "sphere-context-scalar":
		selection.route = .Context_Scalar;
	case "box-native-scalar", "sphere-native-scalar", "box-triangle-scalar", "box-hull-scalar":
		selection.route = .Native_Scalar;
	}

	if cell == "box-triangle-scalar"
	{
		selection.shape = .Triangle;
	}
	else if cell == "box-hull-scalar"
	{
		selection.shape = .Hull;
	}
	return selection, .Ok;
}

Collision_Batch_Case :: struct
{
	world: e.World,
	contexts: []e.Query_Context,
	queries: []e.Collision_Query,
	results, expected: []e.Collision_Query_Result,
	statuses: []e.Status,
	selection: Collision_Batch_Cell,
	workers, batches: int,
}

collision_batch_worker :: proc "contextless" (worker: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	fixture: ^Collision_Batch_Case = (^Collision_Batch_Case)(dispatcher.unmanaged_context);
	batch_size: int = fixture.selection.batch_size;
	begin: int = worker*(COLLISION_BATCH_INPUTS/fixture.workers);
	fixture.statuses[worker] = .Ok;
	for batch in 0..<fixture.batches
	{
		first: int = begin+batch*batch_size;
		queries: []e.Collision_Query = fixture.queries[first:first+batch_size];
		results: []e.Collision_Query_Result = fixture.results[first:first+batch_size];
		status: e.Status;
		switch fixture.selection.route
		{
		case .Native_Batch:
			status = e.collision_query_batch(&fixture.world, queries, results);
		case .Context_Batch:
			status = e.collision_query_batch_with_context(&fixture.contexts[worker], queries, results);
		case .Native_Scalar, .Context_Scalar:
			for query, index in queries
			{
				result: ^e.Collision_Query_Result = &results[index];
				if fixture.selection.route == .Native_Scalar
				{
					result.manifold, result.status = e.collision_query(&fixture.world, query.shape_a, query.pose_a, query.shape_b, query.pose_b, query.speculative_margin);
				}
				else
				{
					result.manifold, result.status = e.collision_query_with_context(&fixture.contexts[worker], query.shape_a, query.pose_a, query.shape_b, query.pose_b, query.speculative_margin);
				}
				result.hit = result.status == .Ok;
				if status == .Ok && result.status != .Ok && result.status != .Not_Found
				{
					status = result.status;
				}
			}
		}
		if fixture.statuses[worker] == .Ok && status != .Ok
		{
			fixture.statuses[worker] = status;
		}
	}
}

// independent axis-aligned geometry checks supplement exact scalar parity
collision_batch_geometry_oracle :: proc "contextless" (query: e.Collision_Query, result: e.Collision_Query_Result, index: int, shape: Collision_Batch_Shape) -> e.Status
{
	if shape == .Triangle || shape == .Hull
	{
		return collision_scalar_holdout_oracle(query, result, index, shape);
	}
	expected_status: e.Status = .Ok if index%2 == 0 else .Not_Found;
	if result.status != expected_status || result.hit != (expected_status == .Ok) || result.manifold.kind != .Convex
	{
		return .Invalid_Description;
	}
	manifold: e.Convex_Contact_Manifold = result.manifold.convex;
	expected_count: i32 = (4 if shape == .Box else 1) if expected_status == .Ok else 0;
	expected_offset: e.Vector3 = util.vector3_subtract(query.pose_b.position, query.pose_a.position);
	if shape == .Box && expected_status == .Not_Found
	{
		expected_offset = {};
	}
	if manifold.count != expected_count || manifold.offset_b != expected_offset
	{
		return .Invalid_Description;
	}
	if expected_status == .Ok
	{
		expected_normal: e.Vector3 = {-1 if (index/2)%2 == 0 else 1, 0, 0};
		if manifold.normal != expected_normal
		{
			return .Invalid_Description;
		}
		contact_x: f32 = (0.75 if index/2%2 == 0 else -0.25) if shape == .Box else (0.375 if index/2%2 == 0 else -0.875);
		corners: u8;
		for contact in manifold.contacts[:manifold.count]
		{
			if contact.offset.x != contact_x || contact.depth != 0.25 || !(contact.offset.x >= -2 && contact.offset.x <= 2) || !(contact.offset.y >= -2 && contact.offset.y <= 2) || !(contact.offset.z >= -2 && contact.offset.z <= 2)
			{
				return .Invalid_Description;
			}
			if shape == .Box
			{
				if abs(contact.offset.y) != 0.5 || abs(contact.offset.z) != 0.5
				{
					return .Invalid_Description;
				}
				corner: u8 = (1 if contact.offset.y > 0 else 0)+(2 if contact.offset.z > 0 else 0);
				corners |= u8(1)<<corner;
			}
			else if contact.offset.y != 0 || contact.offset.z != 0
			{
				return .Invalid_Description;
			}
		}
		if shape == .Box && corners != 15
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

collision_batch_validate :: proc "contextless" (fixture: ^Collision_Batch_Case) -> e.Status
{
	for status in fixture.statuses
	{
		if status != .Ok
		{
			return status;
		}
	}
	for result, index in fixture.results
	{
		if result != fixture.expected[index]
		{
			return .Invalid_Description;
		}
	}
	return .Ok;
}

collision_batch_prepare :: proc(fixture: ^Collision_Batch_Case, cell: string, workers: int)
{
	admission: support.Admission;
	fixture.selection, admission = collision_batch_cell(cell);
	if admission != .Ok
	{
		panic("collision fixture cell");
	}
	fixture.workers = workers;
	context_error, query_error, result_error, expected_error, status_error: runtime.Allocator_Error;
	fixture.contexts, context_error = make([]e.Query_Context, workers);
	fixture.queries, query_error = make([]e.Collision_Query, COLLISION_BATCH_INPUTS);
	fixture.results, result_error = make([]e.Collision_Query_Result, COLLISION_BATCH_INPUTS);
	fixture.expected, expected_error = make([]e.Collision_Query_Result, COLLISION_BATCH_INPUTS);
	fixture.statuses, status_error = make([]e.Status, workers);
	if context_error != nil || query_error != nil || result_error != nil || expected_error != nil || status_error != nil
	{
		panic("collision fixture storage");
	}
	support.extension_require(e.world_init(&fixture.world, world_description(1)), "collision fixture world");
	for &query_context in fixture.contexts
	{
		support.extension_require(e.query_context_init(&query_context, &fixture.world), "collision fixture context");
	}
	a, b: e.Shape_Handle;
	status: e.Status;
	if fixture.selection.shape == .Triangle || fixture.selection.shape == .Hull
	{
		a, b = collision_scalar_holdout_shapes(&fixture.world, fixture.selection.shape);
	}
	else
	{
		if fixture.selection.shape == .Box
		{
			a, status = e.shape_add(&fixture.world, e.box(2, 2, 2));
		}
		else
		{
			a, status = e.shape_add(&fixture.world, e.sphere(0.5));
		}
		support.extension_require(status, "collision fixture A");
		b, status = e.shape_add(&fixture.world, e.box(1, 1, 1) if fixture.selection.shape == .Box else e.box(2, 2, 2));
		support.extension_require(status, "collision fixture B");
	}
	for &query, index in fixture.queries
	{
		query = collision_batch_input(a, b, index, fixture.selection.shape);
		result: ^e.Collision_Query_Result = &fixture.expected[index];
		result.manifold, result.status = e.collision_query(&fixture.world, query.shape_a, query.pose_a, query.shape_b, query.pose_b);
		result.hit = result.status == .Ok;
		support.extension_require(collision_batch_geometry_oracle(query, result^, index, fixture.selection.shape), "collision independent geometry");
	}
}

collision_batch_destroy :: proc(fixture: ^Collision_Batch_Case)
{
	for &query_context in fixture.contexts
	{
		e.query_context_destroy(&query_context);
	}
	e.world_destroy(&fixture.world);
	delete(fixture.contexts);
	delete(fixture.queries);
	delete(fixture.results);
	delete(fixture.expected);
	delete(fixture.statuses);
}

run_collision_batch_case :: proc(args: Query_Arguments) -> e.Status
{
	fixture: Collision_Batch_Case;
	collision_batch_prepare(&fixture, args.query_cell, args.workers);
	defer collision_batch_destroy(&fixture);
	dispatcher: util.Thread_Dispatcher;
	if util.thread_dispatcher_initialize(&dispatcher, args.workers) != .Ok
	{
		panic("collision fixture dispatcher");
	}
	defer util.thread_dispatcher_shutdown(&dispatcher);
	boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
	fixture.batches = 1;
	if boundary.dispatch(boundary, collision_batch_worker, args.workers, &fixture) != .Ok
	{
		return .Invalid_Description;
	}
	fixture.batches = COLLISION_BATCH_INPUTS/(args.workers*fixture.selection.batch_size);
	if boundary.dispatch(boundary, collision_batch_worker, args.workers, &fixture) != .Ok || collision_batch_validate(&fixture) != .Ok
	{
		return .Invalid_Description;
	}
	start: time.Tick = time.tick_now();
	dispatch_status: util.Threading_Status = boundary.dispatch(boundary, collision_batch_worker, args.workers, &fixture);
	elapsed: f64 = f64(time.tick_diff(start, time.tick_now()))/1e6;
	if dispatch_status != .Ok || elapsed <= 0 || collision_batch_validate(&fixture) != .Ok
	{
		return .Invalid_Description;
	}
	file: ^os.File;
	state: support.Extension_CSV_State;
	file, state = support.extension_csv_open(args.output);
	if state == .Empty
	{
		if fmt.fprintln(file, "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries") <= 0
		{
			panic("collision CSV header");
		}
	}
	if fmt.fprintf(file, "%d,ok,ok,%.9f,all,%s,collision-batch,collision,%s,24576,1,24576,%d,12288,12288,24576\n", args.workers, elapsed, e.PRECISION_NAME, args.query_cell, fixture.selection.batch_size*args.workers) <= 0
	{
		panic("collision CSV row");
	}
	support.extension_csv_close(file);
	return .Ok;
}
