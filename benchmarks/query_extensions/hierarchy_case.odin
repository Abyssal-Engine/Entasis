package query_extensions

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:time"
import "core:strings"
import e "entasis:entasis"
import cooking "entasis:entasis_cooking"
import util "entasis:entasis_utilities"
import support "../benchmark_support"

HIERARCHY_INPUTS :: 1536;
HIERARCHY_PASSES :: 8;

Hierarchy_Observation :: struct
{
	status: e.Status,
	count: int,
	t: f32,
	child: i32,
	normal: e.Vector3,
	second_t: f32,
	second_child: i32,
	second_normal: e.Vector3,
}

Hierarchy_Case :: struct
{
	world: e.World,
	target: e.Shape_Handle,
	target_pose: e.Rigid_Pose,
	contexts: []e.Query_Context,
	rays: []e.Ray,
	observations: []Hierarchy_Observation,
	operation: string,
	workers, count, passes: int,
}

hierarchy_worker :: proc "contextless" (worker: int, dispatcher: ^util.Thread_Dispatcher_Boundary)
{
	fixture: ^Hierarchy_Case = (^Hierarchy_Case)(dispatcher.unmanaged_context);
	begin: int = worker*HIERARCHY_INPUTS/fixture.workers;
	for _ in 0..<fixture.passes
	{
		for offset in 0..<fixture.count
		{
			index: int = begin+offset%(HIERARCHY_INPUTS/fixture.workers);
			ray: e.Ray = fixture.rays[index];
			observation: Hierarchy_Observation;
			switch fixture.operation
			{
			case "ray-direct":
				hit: e.Shape_Ray_Hit;
				hit, observation.status = e.shape_ray(&fixture.world, fixture.target, fixture.target_pose, ray);
				observation.count = 1 if hit.state == .Present else 0;
				observation.t, observation.child, observation.normal = hit.t, hit.child_index, hit.normal;
			case "ray-any":
				state: e.Query_Hit_State;
				state, observation.status = e.ray_cast_any_with_context(&fixture.contexts[worker], ray);
				observation.count = 1 if state == .Hit else 0;
			case "ray-closest":
				hit: e.Ray_Hit;
				hit, observation.status = e.ray_cast_closest_with_context(&fixture.contexts[worker], ray);
				observation.count = 1 if observation.status == .Ok else 0;
				observation.t, observation.child, observation.normal = hit.t, hit.child_index, hit.normal;
			case "ray-all":
				hits: [2]e.Ray_Hit;
				observation.count, observation.status = e.ray_cast_all_with_context(&fixture.contexts[worker], ray, hits[:]);
				observation.t, observation.child, observation.normal = hits[0].t, hits[0].child_index, hits[0].normal;
				observation.second_t, observation.second_child, observation.second_normal = hits[1].t, hits[1].child_index, hits[1].normal;
			}
			fixture.observations[index] = observation;
		}
	}
}

hierarchy_validate :: proc(fixture: ^Hierarchy_Case, cell: string) -> (u64, f64, e.Status)
{
	switch cell
	{
	case "compound-8", "big-128", "big-8192", "nested-compound-8", "nested-big-8192":
		return compound_ray_validate(fixture, cell);
	case "hull-32", "hull-256":
		return hull_ray_validate(fixture);
	}
	hits: u64;
	checksum: f64;
	for observation, index in fixture.observations
	{
		expected_count: int = 0;
		expected_status: e.Status = .Ok;
		if index%2 == 0
		{
			expected_count = 2 if fixture.operation == "ray-all" && strings.has_prefix(cell, "nested-") else 1;
		}
		else if fixture.operation == "ray-closest"
		{
			expected_status = .Not_Found;
		}
		if observation.status != expected_status || observation.count != expected_count
		{
			fmt.eprintln("hierarchy independent hit/miss/cardinality oracle");
			return 0, 0, .Invalid_Description;
		}
		if expected_count > 0 && fixture.operation != "ray-any"
		{
			width: int = 8 if strings.has_suffix(cell, "128") else 64;
			expected_child: i32 = i32(2*((index/2)%(width*width)));
			if cell == "box-1" || strings.has_prefix(cell, "nested-")
			{
				expected_child = 0;
			}
			if expected_count == 2 && (observation.second_t != 6 || observation.second_child != 1 || observation.second_normal != (e.Vector3{0, 1, 0}))
			{
				fmt.eprintln("hierarchy second nested result oracle");
			return 0, 0, .Invalid_Description;
			}
			if observation.child != expected_child || observation.t != 4 || observation.normal != (e.Vector3{0, 1, 0})
			{
				fmt.eprintln("hierarchy independent plane/box oracle");
			return 0, 0, .Invalid_Description;
			}
		}
		hits += u64(observation.count)*HIERARCHY_PASSES;
		checksum += f64(observation.count)*HIERARCHY_PASSES;
	}
	return hits, checksum, .Ok;
}

run_hierarchy_case :: proc(args: Query_Arguments) -> e.Status
{
	fixture: Hierarchy_Case = {operation=args.query_operation, workers=args.workers, target_pose=e.pose()};
	// fixture storage and all dispatcher/context construction precede the timer
	context_error, ray_error, observation_error: runtime.Allocator_Error;
	fixture.contexts, context_error = make([]e.Query_Context, args.workers);
	defer delete(fixture.contexts);
	fixture.rays, ray_error = make([]e.Ray, HIERARCHY_INPUTS);
	defer delete(fixture.rays);
	fixture.observations, observation_error = make([]Hierarchy_Observation, HIERARCHY_INPUTS);
	defer delete(fixture.observations);
	if context_error != nil || ray_error != nil || observation_error != nil
	{
		panic("hierarchy fixture storage");
	}
	support.extension_require(e.world_init(&fixture.world, world_description(1)), "hierarchy world");
	defer e.world_destroy(&fixture.world);
	for &query_context in fixture.contexts
	{
		support.extension_require(e.query_context_init(&query_context, &fixture.world), "hierarchy context");
	}
	defer
	{
		for &query_context in fixture.contexts
		{
			e.query_context_destroy(&query_context);
		}
	}
	status: e.Status;
	width: int = 8 if strings.has_suffix(args.query_cell, "128") else 64;
	compound_count: int;
	compound_count, _ = compound_ray_dimensions(args.query_cell);
	hull_sides: int = hull_ray_sides(args.query_cell);
	if hull_sides > 0
	{
		hull_ray_prepare(&fixture, hull_sides);
	}
	else if compound_count > 0
	{
		compound_ray_prepare(&fixture, args.query_cell);
	}
	else if args.query_cell == "box-1"
	{
		fixture.target, status = e.shape_add(&fixture.world, e.box(1, 1, 1));
		support.extension_require(status, "hierarchy box");
	}
	else
	{
		triangles: []e.Triangle;
		allocation_error: runtime.Allocator_Error;
		triangles, allocation_error = make([]e.Triangle, 2*width*width);
		if allocation_error != nil
		{
			panic("hierarchy triangle storage");
		}
		defer delete(triangles);
		for z in 0..<width
		{
			for x in 0..<width
			{
				a: e.Vector3 = {f32(x), 0, f32(z)};
				b: e.Vector3 = {f32(x+1), 0, f32(z)};
				c: e.Vector3 = {f32(x+1), 0, f32(z+1)};
				d: e.Vector3 = {f32(x), 0, f32(z+1)};
				triangles[2*(z*width+x)] = e.triangle(a, b, c);
				triangles[2*(z*width+x)+1] = e.triangle(a, c, d);
			}
		}
		cooker: cooking.Cooking_Context;
		support.extension_require(cooking.cooking_context_init(&cooker), "hierarchy cooker");
		defer cooking.cooking_context_destroy(&cooker);
		mesh: cooking.Cooked_Mesh;
		mesh, status = cooking.cook_mesh(&cooker, triangles);
		support.extension_require(status, "hierarchy cook");
		defer cooking.cooked_mesh_destroy(&mesh);
		fixture.target, status = cooking.cooked_mesh_import(&fixture.world, &mesh);
		support.extension_require(status, "hierarchy import");
		if strings.has_prefix(args.query_cell, "nested-")
		{
			children: [2]e.Compound_Child = {
				e.compound_child(fixture.target, e.pose()),
				e.compound_child(fixture.target, e.pose({0, -2, 0})),
			};
			fixture.target, status = e.shape_import_compound(&fixture.world, children[:]);
			support.extension_require(status, "hierarchy compound");
		}
	}
	_, status = e.static_add(&fixture.world, e.static_body(fixture.target, fixture.target_pose), .None);
	support.extension_require(status, "hierarchy static");
	for &ray, index in fixture.rays
	{
		if compound_count > 0 || hull_sides > 0
		{
			break;
		}
		x: f32 = f32((index/2)%width)+0.75;
		z: f32 = f32((index/2/width)%width)+0.25;
		if args.query_cell == "box-1"
		{
			x, z = 0, 0;
		}
		if index%2 != 0
		{
			x += f32(width)+4;
		}
		ray = e.ray({x, 4 if args.query_cell != "box-1" else 4.5, z}, {0, -1, 0}, 8);
	}
	hits: u64;
	checksum, elapsed: f64;
	{
		dispatcher: util.Thread_Dispatcher;
		if util.thread_dispatcher_initialize(&dispatcher, args.workers) != .Ok
		{
			panic("hierarchy dispatcher");
		}
		previous: uintptr;
		defer
		{
			shutdown_status: util.Threading_Status = util.thread_dispatcher_shutdown(&dispatcher);
			restore_status: e.Status = .Ok;
			when ODIN_OS == .Windows
			{
				if previous != 0
				{
					restore_status = compound_ray_restore_caller(previous);
				}
			}
			if shutdown_status != .Ok || restore_status != .Ok
			{
				panic("hierarchy worker cleanup");
			}
		}
		if compound_count > 0
		{
			when ODIN_OS == .Windows
			{
				previous, status = compound_ray_pin_workers(&dispatcher, args.query_cell);
				if status != .Ok
				{
					return status;
				}
			}
			else
			{
				return .Invalid_Argument;
			}
		}
		boundary: ^util.Thread_Dispatcher_Boundary = util.thread_dispatcher_boundary(&dispatcher);
		fixture.count, fixture.passes = 128, 1;
		if boundary.dispatch(boundary, hierarchy_worker, args.workers, &fixture) != .Ok
		{
			fmt.eprintln("hierarchy warmup dispatch");
			return .Invalid_Description;
		}
		// every input is checked before measurement, independently of the query path
		fixture.count = HIERARCHY_INPUTS/args.workers;
		if boundary.dispatch(boundary, hierarchy_worker, args.workers, &fixture) != .Ok
		{
			fmt.eprintln("hierarchy oracle dispatch");
			return .Invalid_Description;
		}
		_, _, status = hierarchy_validate(&fixture, args.query_cell);
		if status != .Ok
		{
			return status;
		}
		fixture.passes = HIERARCHY_PASSES;
		start: time.Tick = time.tick_now();
		dispatch_status: util.Threading_Status = boundary.dispatch(boundary, hierarchy_worker, args.workers, &fixture);
		elapsed = f64(time.tick_diff(start, time.tick_now()))/1e6;
		if dispatch_status != .Ok || elapsed <= 0
		{
			fmt.eprintln("hierarchy measurement dispatch");
			return .Invalid_Description;
		}
		hits, checksum, status = hierarchy_validate(&fixture, args.query_cell);
		if status != .Ok
		{
			return status;
		}
	}
	file: ^os.File;
	state: support.Extension_CSV_State;
	file, state = support.extension_csv_open(args.output);
	if state == .Empty
	{
		if fmt.fprintln(file, "worker_count,case_status,metric_status,physics_elapsed_ms,benchmark_components,precision,query_case,query_operation,query_cell,input_count,passes,query_count,warmup_queries,hit_count,checksum,oracle_queries") <= 0
		{
			panic("hierarchy CSV header");
		}
	}
	if fmt.fprintf(file, "%d,ok,ok,%.9f,all,%s,%s,%s,%s,1536,8,12288,%d,%d,%.0f,1536\n", args.workers, elapsed, e.PRECISION_NAME, args.query_case, args.query_operation, args.query_cell, 128*args.workers, hits, checksum) <= 0
	{
		panic("hierarchy CSV row");
	}
	support.extension_csv_close(file);
	return .Ok;
}
